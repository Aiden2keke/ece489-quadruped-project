#!/usr/bin/env python3
import argparse
import csv
import math
import os
import re
import warnings
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-cache")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT_ROOT = PROJECT_ROOT / "real_robot_outputs" / "logs" / "side_logger"
DEFAULT_FIG_DIR = PROJECT_ROOT / "real_robot_outputs" / "figures_academic"
DEFAULT_SUMMARY_DIR = PROJECT_ROOT / "real_robot_outputs" / "summary_academic"

REMOTE_STAGE = "remote_control_walk"
PUSH_STAGE = "push_light"


def _to_float(value):
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _finite(values):
    arr = np.asarray(values, dtype=float)
    return arr[np.isfinite(arr)]


def _nanmean(values):
    finite = _finite(values)
    return float(np.mean(finite)) if finite.size else math.nan


def _nanrms(values):
    finite = _finite(values)
    return float(np.sqrt(np.mean(finite ** 2))) if finite.size else math.nan


def _nanmax_abs(values):
    finite = _finite(values)
    return float(np.max(np.abs(finite))) if finite.size else math.nan


def _nanmax_abs_axis0(matrix):
    arr = np.abs(np.asarray(matrix, dtype=float))
    valid = np.isfinite(arr)
    filled = np.where(valid, arr, -np.inf)
    out = np.max(filled, axis=0)
    out[~np.any(valid, axis=0)] = math.nan
    return out


def _nanmin(values):
    finite = _finite(values)
    return float(np.min(finite)) if finite.size else math.nan


def _nanmax(values):
    finite = _finite(values)
    return float(np.max(finite)) if finite.size else math.nan


def _read_rows(path):
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def _series(rows, key):
    if not rows or key not in rows[0]:
        return np.full(len(rows), math.nan)
    return np.array([_to_float(row.get(key)) for row in rows], dtype=float)


def _has_finite(rows, key):
    return np.isfinite(_series(rows, key)).any()


def _time(rows):
    t = _series(rows, "time")
    if not np.isfinite(t).any():
        t = _series(rows, "wall_time")
    if not np.isfinite(t).any():
        return np.arange(len(rows), dtype=float)
    first = t[np.isfinite(t)][0]
    return t - first


def _matrix_from_prefix(rows, prefix, count=12):
    cols = []
    for idx in range(1, count + 1):
        key = f"{prefix}_{idx}"
        if key in rows[0]:
            cols.append(_series(rows, key))
    if len(cols) != count:
        return None
    mat = np.vstack(cols)
    return mat if np.isfinite(mat).any() else None


def _joint_tracking_error(rows):
    err = _matrix_from_prefix(rows, "joint_tracking_error", 12)
    if err is not None:
        return err, "joint_tracking_error_i"
    target = _matrix_from_prefix(rows, "target_joint_pos", 12)
    actual = _matrix_from_prefix(rows, "joint_pos", 12)
    if target is not None and actual is not None:
        return target - actual, "target_joint_pos_i - joint_pos_i"
    return None, "unavailable"


def _joint_torque(rows):
    return _matrix_from_prefix(rows, "joint_torque", 12)


def _angular_velocity_norm(rows):
    wx = _series(rows, "angular_vel_x")
    wy = _series(rows, "angular_vel_y")
    wz = _series(rows, "angular_vel_z")
    return np.sqrt(wx ** 2 + wy ** 2 + wz ** 2)


def _foot_force_matrix(rows):
    cols = []
    for prefix in ("foot_force", "foot_contact"):
        for idx in range(1, 5):
            key = f"{prefix}_{idx}"
            if key in rows[0] and _has_finite(rows, key):
                cols.append(_series(rows, key))
        if cols:
            break
    if not cols:
        return None, "unavailable"
    mat = np.vstack(cols)
    if not np.isfinite(mat).any():
        return None, "unavailable"
    return mat, "foot_force" if "foot_force_1" in rows[0] else "foot_contact"


def _moving_average(y, window):
    y = np.asarray(y, dtype=float)
    if window <= 1 or y.size == 0:
        return y
    valid = np.isfinite(y).astype(float)
    clean = np.where(np.isfinite(y), y, 0.0)
    kernel = np.ones(int(window), dtype=float)
    numerator = np.convolve(clean, kernel, mode="same")
    denominator = np.convolve(valid, kernel, mode="same")
    out = numerator / np.maximum(denominator, 1.0)
    out[denominator == 0] = math.nan
    return out


def _smooth_window_samples(t, window_s, no_smoothing):
    if no_smoothing or window_s <= 0:
        return 1
    finite_t = _finite(t)
    if finite_t.size < 2:
        return 1
    diffs = np.diff(finite_t)
    diffs = diffs[diffs > 0]
    if diffs.size == 0:
        return 1
    dt = float(np.median(diffs))
    return max(1, int(round(window_s / dt)))


def _style():
    plt.rcParams.update({
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "legend.fontsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })


def _safe_stage(rows, fallback):
    if rows:
        return rows[0].get("experiment_stage") or fallback
    return fallback


def _safe_trial(rows, fallback):
    if rows:
        return rows[0].get("trial_name") or fallback
    return fallback


def _is_side_logger_csv(path):
    try:
        rows = _read_rows(path)
    except Exception:
        return False, []
    if not rows:
        return False, rows
    return rows[0].get("logger_type") == "lcm_side_logger", rows


def _find_csv(input_root, stage_name, explicit=None):
    if explicit:
        path = Path(explicit).expanduser().resolve()
        ok, rows = _is_side_logger_csv(path)
        if not ok:
            raise ValueError(f"{path} is not a side logger CSV with logger_type=lcm_side_logger.")
        return path, rows

    input_root = Path(input_root).expanduser().resolve()
    candidates = []
    for path in sorted(input_root.rglob("*.csv")):
        ok, rows = _is_side_logger_csv(path)
        if not ok:
            continue
        stage = _safe_stage(rows, path.parent.name)
        text = f"{path} {stage} {_safe_trial(rows, path.stem)}".lower()
        if stage_name.lower() in text:
            score = 0
            if stage == stage_name:
                score += 10
            if stage_name in path.parent.name:
                score += 5
            if "trial01" in path.name:
                score += 2
            candidates.append((score, path, rows))
    if not candidates:
        raise FileNotFoundError(f"No side logger CSV found for stage '{stage_name}' under {input_root}.")
    candidates.sort(key=lambda item: (-item[0], str(item[1])))
    return candidates[0][1], candidates[0][2]


def _parse_metadata_push_time(rows):
    if not rows:
        return math.nan
    metadata_path = rows[0].get("metadata_path", "")
    if not metadata_path:
        return math.nan
    path = Path(metadata_path).expanduser()
    if not path.exists():
        return math.nan
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("push_time") and ":" in stripped:
            return _to_float(stripped.split(":", 1)[1].strip())
    return math.nan


def _estimate_push_time(rows, cli_push_time=None):
    if cli_push_time is not None:
        return float(cli_push_time), "manual_cli"
    metadata_push = _parse_metadata_push_time(rows)
    if math.isfinite(metadata_push):
        return metadata_push, "metadata"
    t = _time(rows)
    omega_norm = _angular_velocity_norm(rows)
    if np.isfinite(omega_norm).any():
        idx = int(np.nanargmax(omega_norm))
        return float(t[idx]), "auto_max_angular_velocity_norm"
    return math.nan, "unavailable"


def compute_metrics(rows, csv_path, push_time=None):
    t = _time(rows)
    cmd_vx = _series(rows, "cmd_vx")
    cmd_yaw = _series(rows, "cmd_yaw")
    roll = _series(rows, "roll")
    pitch = _series(rows, "pitch")
    omega_norm = _angular_velocity_norm(rows)
    joint_err, err_source = _joint_tracking_error(rows)
    torque = _joint_torque(rows)
    finite_t = _finite(t)
    duration = float(finite_t[-1] - finite_t[0]) if finite_t.size >= 2 else math.nan
    motion = (np.abs(cmd_vx) > 0.05) | (np.abs(cmd_yaw) > 0.05)
    motion_fraction = float(np.nanmean(motion.astype(float))) if motion.size else math.nan

    stage = _safe_stage(rows, Path(csv_path).parent.name)
    trial_name = _safe_trial(rows, Path(csv_path).stem)
    estimated_push_time = math.nan
    push_time_source = ""
    if stage == PUSH_STAGE:
        estimated_push_time, push_time_source = _estimate_push_time(rows, push_time)

    metrics = {
        "trial_name": trial_name,
        "experiment_stage": stage,
        "duration_s": duration,
        "num_samples": len(rows),
        "logger_type": rows[0].get("logger_type", "") if rows else "",
        "source_csv": str(Path(csv_path).resolve()),
        "cmd_vx_min": _nanmin(cmd_vx),
        "cmd_vx_max": _nanmax(cmd_vx),
        "cmd_vx_mean": _nanmean(cmd_vx),
        "cmd_yaw_min": _nanmin(cmd_yaw),
        "cmd_yaw_max": _nanmax(cmd_yaw),
        "cmd_yaw_mean": _nanmean(cmd_yaw),
        "motion_fraction": motion_fraction,
        "roll_rms_rad": _nanrms(roll),
        "pitch_rms_rad": _nanrms(pitch),
        "roll_rms_deg": math.degrees(_nanrms(roll)) if math.isfinite(_nanrms(roll)) else math.nan,
        "pitch_rms_deg": math.degrees(_nanrms(pitch)) if math.isfinite(_nanrms(pitch)) else math.nan,
        "max_abs_roll_rad": _nanmax_abs(roll),
        "max_abs_pitch_rad": _nanmax_abs(pitch),
        "max_abs_roll_deg": math.degrees(_nanmax_abs(roll)) if math.isfinite(_nanmax_abs(roll)) else math.nan,
        "max_abs_pitch_deg": math.degrees(_nanmax_abs(pitch)) if math.isfinite(_nanmax_abs(pitch)) else math.nan,
        "max_angular_velocity_norm": _nanmax(omega_norm),
        "max_abs_joint_tracking_error": _nanmax_abs(joint_err) if joint_err is not None else math.nan,
        "mean_abs_joint_tracking_error": _nanmean(np.abs(joint_err)) if joint_err is not None else math.nan,
        "max_abs_joint_torque": _nanmax_abs(torque) if torque is not None else math.nan,
        "mean_abs_joint_torque": _nanmean(np.abs(torque)) if torque is not None else math.nan,
        "estimated_push_time_s": estimated_push_time,
        "push_time_source": push_time_source,
        "push_note": "manual qualitative light push, not calibrated 40 N" if stage == PUSH_STAGE else "",
        "joint_tracking_error_source": err_source,
    }
    return metrics


def _plot_commands_attitude(ax_cmd, ax_att, rows, title_prefix, push_time, smooth_n):
    t = _time(rows)
    cmd_vx = _series(rows, "cmd_vx")
    cmd_yaw = _series(rows, "cmd_yaw")
    roll_deg = np.degrees(_series(rows, "roll"))
    pitch_deg = np.degrees(_series(rows, "pitch"))

    ax_cmd.plot(t, _moving_average(cmd_vx, smooth_n), label=r"$v_x$ command", color="#1f77b4", linewidth=1.5)
    ax_cmd.plot(t, _moving_average(cmd_yaw, smooth_n), label="yaw-rate command", color="#d62728", linewidth=1.5)
    ax_cmd.set_title(f"{title_prefix}: Command Input")
    ax_cmd.set_xlabel("Time (s)")
    ax_cmd.set_ylabel("Command")
    ax_cmd.legend(loc="best")

    ax_att.plot(t, _moving_average(roll_deg, smooth_n), label="roll", color="#2ca02c", linewidth=1.5)
    ax_att.plot(t, _moving_average(pitch_deg, smooth_n), label="pitch", color="#9467bd", linewidth=1.5)
    ax_att.set_title(f"{title_prefix}: Body Attitude")
    ax_att.set_xlabel("Time (s)")
    ax_att.set_ylabel("Angle (deg)")
    ax_att.legend(loc="best")

    if push_time is not None and math.isfinite(push_time):
        for ax in (ax_cmd, ax_att):
            ax.axvline(push_time, color="black", linestyle="--", linewidth=1.2, label="Estimated light-push event")
        handles, labels = ax_att.get_legend_handles_labels()
        seen = set()
        unique = [(h, l) for h, l in zip(handles, labels) if not (l in seen or seen.add(l))]
        ax_att.legend([h for h, _ in unique], [l for _, l in unique], loc="best")


def plot_overview(remote_rows, push_rows, out_path, push_time, smooth_n_remote, smooth_n_push):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    _plot_commands_attitude(
        axes[0, 0], axes[0, 1], remote_rows,
        "Remote-Controlled Walking", None, smooth_n_remote
    )
    _plot_commands_attitude(
        axes[1, 0], axes[1, 1], push_rows,
        "Light-Push Test", push_time, smooth_n_push
    )
    labels = ["(a)", "(b)", "(c)", "(d)"]
    for label, ax in zip(labels, axes.ravel()):
        ax.text(0.01, 0.98, label, transform=ax.transAxes, va="top", ha="left", fontweight="bold")
    fig.savefig(out_path)
    plt.close(fig)


def plot_push_zoom(push_rows, out_path, push_time, smooth_n, warnings_list):
    if not math.isfinite(push_time):
        warnings.warn("Cannot generate push zoom response: push time unavailable.")
        warnings_list.append("push_light_zoom_response.png skipped: push time unavailable.")
        return False

    t = _time(push_rows)
    mask = (t >= push_time - 2.0) & (t <= push_time + 3.0)
    if not mask.any():
        warnings.warn("Cannot generate push zoom response: requested window has no samples.")
        warnings_list.append("push_light_zoom_response.png skipped: push window has no samples.")
        return False

    roll_deg = np.degrees(_series(push_rows, "roll"))
    pitch_deg = np.degrees(_series(push_rows, "pitch"))
    omega_norm = _angular_velocity_norm(push_rows)
    foot, foot_source = _foot_force_matrix(push_rows)
    joint_err, _ = _joint_tracking_error(push_rows)

    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t[mask], _moving_average(roll_deg, smooth_n)[mask], label="roll", color="#2ca02c")
    axes[0].plot(t[mask], _moving_average(pitch_deg, smooth_n)[mask], label="pitch", color="#9467bd")
    axes[0].set_ylabel("Angle (deg)")
    axes[0].legend(loc="best")

    axes[1].plot(t[mask], _moving_average(omega_norm, smooth_n)[mask], color="#d62728", label=r"$||\omega||$")
    axes[1].set_ylabel("Ang. vel. norm\n(rad/s)")
    axes[1].legend(loc="best")

    if foot is not None:
        foot_norm = np.sqrt(np.nansum(foot ** 2, axis=0))
        axes[2].plot(t[mask], _moving_average(foot_norm, smooth_n)[mask], color="#1f77b4", label=f"{foot_source} norm")
        axes[2].set_ylabel("Foot signal norm")
        axes[2].legend(loc="best")
        if np.nanmax(np.abs(foot_norm)) == 0:
            warnings_list.append("Foot force/contact fields are present but all zero in push-light CSV.")
    else:
        axes[2].text(0.5, 0.5, "Foot force/contact unavailable", ha="center", va="center", transform=axes[2].transAxes)
        axes[2].set_ylabel("Foot signal")
        warnings_list.append("Foot force/contact fields unavailable for push-light zoom plot.")

    if joint_err is not None:
        max_err = _nanmax_abs_axis0(joint_err)
        axes[3].plot(t[mask], _moving_average(max_err, smooth_n)[mask], color="#ff7f0e", label="max joint tracking error")
        axes[3].set_ylabel("Max joint err. (rad)")
        axes[3].legend(loc="best")
    else:
        axes[3].text(0.5, 0.5, "Joint tracking error unavailable", ha="center", va="center", transform=axes[3].transAxes)
        axes[3].set_ylabel("Joint err.")
        warnings_list.append("Joint tracking error unavailable for push-light zoom plot.")

    for ax in axes:
        ax.axvline(push_time, color="black", linestyle="--", linewidth=1.2)
    axes[0].set_title("Light-Push Test: Zoomed Response")
    axes[-1].set_xlabel("Time (s)")
    axes[0].text(
        push_time, axes[0].get_ylim()[1], "Estimated light-push event",
        ha="left", va="top", fontsize=9
    )
    fig.savefig(out_path)
    plt.close(fig)
    return True


def _decimate_time_matrix(t, mat, max_points=6000):
    if mat.shape[1] <= max_points:
        return t, mat
    idx = np.linspace(0, mat.shape[1] - 1, max_points).astype(int)
    return t[idx], mat[:, idx]


def plot_heatmap(rows, out_path, title, warnings_list):
    err, source = _joint_tracking_error(rows)
    if err is None:
        warnings.warn(f"Cannot generate {out_path.name}: joint tracking error unavailable.")
        warnings_list.append(f"{out_path.name} skipped: joint tracking error unavailable.")
        return False
    t = _time(rows)
    t_plot, err_plot = _decimate_time_matrix(t, err)
    vmax = _nanmax_abs(err_plot)
    if not math.isfinite(vmax) or vmax <= 0:
        vmax = 1e-6
    fig, ax = plt.subplots(figsize=(9, 4.2), constrained_layout=True)
    im = ax.imshow(
        err_plot,
        extent=[float(np.nanmin(t_plot)), float(np.nanmax(t_plot)), 0.5, 12.5],
        origin="lower",
        aspect="auto",
        cmap="coolwarm",
        vmin=-vmax,
        vmax=vmax,
        interpolation="nearest",
    )
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Joint index")
    ax.set_yticks(np.arange(1, 13))
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Tracking error (rad)")
    ax.text(0.99, 1.03, f"source: {source}", transform=ax.transAxes, ha="right", va="bottom", fontsize=8)
    fig.savefig(out_path)
    plt.close(fig)
    return True


def plot_torque_bar(remote_rows, push_rows, out_path, warnings_list):
    remote_torque = _joint_torque(remote_rows)
    push_torque = _joint_torque(push_rows)
    if remote_torque is None or push_torque is None:
        warnings.warn("Cannot generate torque bar: joint torque fields unavailable.")
        warnings_list.append("joint_torque_summary_bar.png skipped: joint torque fields unavailable.")
        return False
    remote_max = np.nanmax(np.abs(remote_torque), axis=1)
    push_max = np.nanmax(np.abs(push_torque), axis=1)
    x = np.arange(1, 13)
    width = 0.38
    fig, ax = plt.subplots(figsize=(9, 4.2), constrained_layout=True)
    ax.bar(x - width / 2, remote_max, width, label="Remote Walk", color="#1f77b4")
    ax.bar(x + width / 2, push_max, width, label="Push Light", color="#ff7f0e")
    ax.set_title("Estimated Joint Torque Peaks")
    ax.set_xlabel("Joint index")
    ax.set_ylabel("Max Abs Estimated Joint Torque")
    ax.set_xticks(x)
    ax.legend()
    fig.savefig(out_path)
    plt.close(fig)
    return True


def plot_attitude_bar(remote_metrics, push_metrics, out_path):
    labels = ["Remote Walk", "Push Light"]
    keys = [
        ("roll_rms_deg", "Roll RMS"),
        ("pitch_rms_deg", "Pitch RMS"),
        ("max_abs_roll_deg", "Max |Roll|"),
        ("max_abs_pitch_deg", "Max |Pitch|"),
    ]
    x = np.arange(len(labels))
    width = 0.18
    fig, ax = plt.subplots(figsize=(8.5, 4.2), constrained_layout=True)
    colors = ["#1f77b4", "#2ca02c", "#d62728", "#9467bd"]
    for idx, (key, label) in enumerate(keys):
        values = [remote_metrics.get(key, math.nan), push_metrics.get(key, math.nan)]
        ax.bar(x + (idx - 1.5) * width, values, width, label=label, color=colors[idx])
    ax.set_title("Real-Robot Attitude Summary")
    ax.set_ylabel("Angle (deg)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend(ncol=2)
    fig.savefig(out_path)
    plt.close(fig)


def write_summary_csv(path, metrics_rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(metrics_rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(metrics_rows)


def _fmt(value, digits=3):
    if value is None or not math.isfinite(_to_float(value)):
        return "N/A"
    return f"{float(value):.{digits}f}"


def write_summary_md(path, remote_metrics, push_metrics, warnings_list, generated_figures):
    report_sentence = (
        "A preliminary real-robot deployment was conducted on the Unitree Go2 using the TS_re3 policy. "
        f"During the remote-controlled walking trial ({_fmt(remote_metrics['duration_s'], 2)} s), "
        f"the robot exhibited roll and pitch RMS values of {_fmt(remote_metrics['roll_rms_deg'], 2)} deg and "
        f"{_fmt(remote_metrics['pitch_rms_deg'], 2)} deg, with peak absolute roll/pitch of "
        f"{_fmt(remote_metrics['max_abs_roll_deg'], 2)} deg / {_fmt(remote_metrics['max_abs_pitch_deg'], 2)} deg. "
        f"During the qualitative light-push trial ({_fmt(push_metrics['duration_s'], 2)} s), "
        f"the estimated push event occurred at {_fmt(push_metrics['estimated_push_time_s'], 2)} s and the peak "
        f"absolute roll/pitch reached {_fmt(push_metrics['max_abs_roll_deg'], 2)} deg / "
        f"{_fmt(push_metrics['max_abs_pitch_deg'], 2)} deg. "
        "These results provide qualitative deployment validation rather than a calibrated fixed-condition benchmark."
    )

    text = f"""# Real-Robot Deployment Summary

## Data Source

- Source: side-channel LCM logger (`lcm_side_logger.py`).
- The logger subscribes to LCM topics only and does not insert logging into the policy control loop.
- The logger does not send control commands and does not affect the action send order.

## Remote Walk Summary

- CSV: `{remote_metrics['source_csv']}`
- Duration: {_fmt(remote_metrics['duration_s'], 2)} s
- Samples: {remote_metrics['num_samples']}
- Roll / pitch RMS: {_fmt(remote_metrics['roll_rms_deg'], 2)} deg / {_fmt(remote_metrics['pitch_rms_deg'], 2)} deg
- Max absolute roll / pitch: {_fmt(remote_metrics['max_abs_roll_deg'], 2)} deg / {_fmt(remote_metrics['max_abs_pitch_deg'], 2)} deg
- Command range:
  - cmd_vx: [{_fmt(remote_metrics['cmd_vx_min'], 3)}, {_fmt(remote_metrics['cmd_vx_max'], 3)}], mean {_fmt(remote_metrics['cmd_vx_mean'], 3)}
  - cmd_yaw: [{_fmt(remote_metrics['cmd_yaw_min'], 3)}, {_fmt(remote_metrics['cmd_yaw_max'], 3)}], mean {_fmt(remote_metrics['cmd_yaw_mean'], 3)}
- Motion fraction: {_fmt(remote_metrics['motion_fraction'], 3)}
- Interpretation: this trial contains both standing and remote-controlled walking, so it should be treated as a qualitative deployment validation trial.

## Push-Light Summary

- CSV: `{push_metrics['source_csv']}`
- Duration: {_fmt(push_metrics['duration_s'], 2)} s
- Samples: {push_metrics['num_samples']}
- Estimated push time: {_fmt(push_metrics['estimated_push_time_s'], 2)} s ({push_metrics.get('push_time_source', 'unknown')})
- Roll / pitch RMS: {_fmt(push_metrics['roll_rms_deg'], 2)} deg / {_fmt(push_metrics['pitch_rms_deg'], 2)} deg
- Max absolute roll / pitch: {_fmt(push_metrics['max_abs_roll_deg'], 2)} deg / {_fmt(push_metrics['max_abs_pitch_deg'], 2)} deg
- Max angular velocity norm: {_fmt(push_metrics['max_angular_velocity_norm'], 3)} rad/s
- Push note: manual qualitative light push, not calibrated 40 N.

## Limitations

- These real-robot logs are not strict one-to-one quantitative benchmarks against MuJoCo fixed-vx or calibrated 40 N push conditions.
- Base velocity, policy action, policy observation, and policy inference time are not available from the side logger unless exposed as LCM topics.
- Push strength is qualitative and should not be reported as a calibrated force.
- Foot contact/force fields may be unreliable if the contact fields are sparse or all zero.

## Generated Figures

"""
    for fig in generated_figures:
        text += f"- `{fig}`\n"
    if warnings_list:
        text += "\n## Warnings / Missing Fields\n\n"
        for item in warnings_list:
            text += f"- {item}\n"
    text += f"\n## Suggested Report Wording\n\n{report_sentence}\n"
    path.write_text(text, encoding="utf-8")
    return report_sentence


def main():
    parser = argparse.ArgumentParser(description="Generate academic figures and summary metrics for real Go2 side-logger data.")
    parser.add_argument("--input_root", default=str(DEFAULT_INPUT_ROOT))
    parser.add_argument("--out_fig_dir", default=str(DEFAULT_FIG_DIR))
    parser.add_argument("--out_summary_dir", default=str(DEFAULT_SUMMARY_DIR))
    parser.add_argument("--remote_csv", default=None)
    parser.add_argument("--push_csv", default=None)
    parser.add_argument("--push_time", type=float, default=None)
    parser.add_argument("--smooth_window_s", type=float, default=0.05)
    parser.add_argument("--no_smoothing", action="store_true")
    args = parser.parse_args()

    _style()
    out_fig_dir = Path(args.out_fig_dir).expanduser().resolve()
    out_summary_dir = Path(args.out_summary_dir).expanduser().resolve()
    out_fig_dir.mkdir(parents=True, exist_ok=True)
    out_summary_dir.mkdir(parents=True, exist_ok=True)

    remote_csv, remote_rows = _find_csv(args.input_root, REMOTE_STAGE, args.remote_csv)
    push_csv, push_rows = _find_csv(args.input_root, PUSH_STAGE, args.push_csv)

    push_time, push_time_source = _estimate_push_time(push_rows, args.push_time)
    remote_metrics = compute_metrics(remote_rows, remote_csv)
    push_metrics = compute_metrics(push_rows, push_csv, push_time=args.push_time)
    push_metrics["estimated_push_time_s"] = push_time
    push_metrics["push_time_source"] = push_time_source

    smooth_remote = _smooth_window_samples(_time(remote_rows), args.smooth_window_s, args.no_smoothing)
    smooth_push = _smooth_window_samples(_time(push_rows), args.smooth_window_s, args.no_smoothing)

    warnings_list = []
    generated = []

    figure_path = out_fig_dir / "real_robot_overview_commands_attitude.png"
    plot_overview(remote_rows, push_rows, figure_path, push_time, smooth_remote, smooth_push)
    generated.append(str(figure_path))

    figure_path = out_fig_dir / "push_light_zoom_response.png"
    if plot_push_zoom(push_rows, figure_path, push_time, smooth_push, warnings_list):
        generated.append(str(figure_path))

    figure_path = out_fig_dir / "joint_tracking_error_heatmap_remote_walk.png"
    if plot_heatmap(remote_rows, figure_path, "Remote-Controlled Walking: Joint Tracking Error", warnings_list):
        generated.append(str(figure_path))

    figure_path = out_fig_dir / "joint_tracking_error_heatmap_push_light.png"
    if plot_heatmap(push_rows, figure_path, "Light-Push Test: Joint Tracking Error", warnings_list):
        generated.append(str(figure_path))

    figure_path = out_fig_dir / "joint_torque_summary_bar.png"
    if plot_torque_bar(remote_rows, push_rows, figure_path, warnings_list):
        generated.append(str(figure_path))

    figure_path = out_fig_dir / "real_robot_attitude_comparison_bar.png"
    plot_attitude_bar(remote_metrics, push_metrics, figure_path)
    generated.append(str(figure_path))

    foot_remote, _ = _foot_force_matrix(remote_rows)
    foot_push, _ = _foot_force_matrix(push_rows)
    for label, foot in (("remote walk", foot_remote), ("push light", foot_push)):
        if foot is None:
            warnings_list.append(f"Foot force/contact unavailable for {label}.")
        elif np.nanmax(np.abs(foot)) == 0:
            warnings_list.append(f"Foot force/contact fields are present but all zero for {label}.")

    summary_csv = out_summary_dir / "real_robot_summary_metrics.csv"
    write_summary_csv(summary_csv, [remote_metrics, push_metrics])
    summary_md = out_summary_dir / "real_robot_summary_metrics.md"
    report_sentence = write_summary_md(summary_md, remote_metrics, push_metrics, warnings_list, generated)

    print(f"Remote CSV: {remote_csv}")
    print(f"Push CSV: {push_csv}")
    print(f"Push time: {push_time:.6f} s ({push_time_source})" if math.isfinite(push_time) else f"Push time: unavailable ({push_time_source})")
    print(f"Wrote summary CSV: {summary_csv}")
    print(f"Wrote summary MD: {summary_md}")
    print("Generated figures:")
    for item in generated:
        print(f"  - {item}")
    if warnings_list:
        print("Warnings:")
        for item in warnings_list:
            print(f"  - {item}")
    print("Suggested report wording:")
    print(report_sentence)


if __name__ == "__main__":
    main()
