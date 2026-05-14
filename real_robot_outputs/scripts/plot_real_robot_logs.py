#!/usr/bin/env python3
import argparse
import csv
import math
import re
import warnings
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "real_robot_outputs"
ACTIVE_STAGES = {"stand", "remote_control_walk", "push_light"}


def _safe_name(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)


def _to_float(value):
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def _read_csv(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return rows


def _series(rows, key):
    if not rows or key not in rows[0]:
        return None
    data = np.array([_to_float(row.get(key)) for row in rows], dtype=float)
    if not np.isfinite(data).any():
        return None
    return data


def _time(rows):
    t = _series(rows, "time")
    if t is None:
        return np.arange(len(rows), dtype=float)
    return t


def _save(fig, path):
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_trial(csv_path, output_root):
    import matplotlib.pyplot as plt

    rows = _read_csv(csv_path)
    if not rows:
        warnings.warn(f"Skip empty CSV: {csv_path}")
        return

    t = _time(rows)
    trial = _safe_name(rows[0].get("trial_name", csv_path.stem))
    stage = _safe_name(rows[0].get("experiment_stage", csv_path.parent.name))
    prefix = f"{stage}_{trial}"
    figures = output_root / "figures"

    cmd_vx = _series(rows, "cmd_vx")
    base_vx = _series(rows, "base_vx")
    if cmd_vx is not None and base_vx is not None:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, cmd_vx, label="cmd_vx")
        ax.plot(t, base_vx, label="base_vx")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("velocity [m/s]")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "velocity_tracking" / f"{prefix}_cmd_vx_vs_base_vx.png")
    else:
        warnings.warn(f"Skip velocity plot for {csv_path}: cmd_vx or base_vx missing/NaN.")

    remote_lx = _series(rows, "remote_left_stick_x")
    remote_ly = _series(rows, "remote_left_stick_y")
    remote_rx = _series(rows, "remote_right_stick_x")
    remote_ry = _series(rows, "remote_right_stick_y")
    if any(series is not None for series in [remote_lx, remote_ly, remote_rx, remote_ry]):
        fig, ax = plt.subplots(figsize=(8, 4))
        if remote_lx is not None:
            ax.plot(t, remote_lx, label="left_x")
        if remote_ly is not None:
            ax.plot(t, remote_ly, label="left_y")
        if remote_rx is not None:
            ax.plot(t, remote_rx, label="right_x")
        if remote_ry is not None:
            ax.plot(t, remote_ry, label="right_y")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("remote stick")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "remote_command" / f"{prefix}_remote_sticks.png")

    roll = _series(rows, "roll")
    pitch = _series(rows, "pitch")
    if roll is not None or pitch is not None:
        fig, ax = plt.subplots(figsize=(8, 4))
        if roll is not None:
            ax.plot(t, roll, label="roll")
        if pitch is not None:
            ax.plot(t, pitch, label="pitch")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("angle [rad]")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "roll_pitch" / f"{prefix}_roll_pitch.png")
    else:
        warnings.warn(f"Skip roll/pitch plot for {csv_path}.")

    representative_joints = [1, 4, 7, 10]
    has_joint_plot = False
    fig, axes = plt.subplots(len(representative_joints), 1, figsize=(8, 8), sharex=True)
    for ax, joint_idx in zip(axes, representative_joints):
        actual = _series(rows, f"joint_pos_{joint_idx}")
        target = _series(rows, f"target_joint_pos_{joint_idx}")
        if actual is None or target is None:
            ax.set_visible(False)
            continue
        has_joint_plot = True
        ax.plot(t, actual, label=f"joint_{joint_idx}")
        ax.plot(t, target, label=f"target_{joint_idx}", linestyle="--")
        ax.set_ylabel("rad")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    if has_joint_plot:
        _save(fig, figures / "joint_tracking_error" / f"{prefix}_joint_target_actual.png")
    else:
        plt.close(fig)
        warnings.warn(f"Skip joint target plot for {csv_path}.")

    err_cols = [_series(rows, f"joint_tracking_error_{idx}") for idx in range(1, 13)]
    err_cols = [col for col in err_cols if col is not None]
    if err_cols:
        err = np.vstack(err_cols)
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, np.sqrt(np.nanmean(err ** 2, axis=0)), label="RMS joint tracking error")
        ax.plot(t, np.nanmax(np.abs(err), axis=0), label="max abs joint tracking error")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("rad")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "joint_tracking_error" / f"{prefix}_joint_tracking_error.png")
    else:
        warnings.warn(f"Skip joint tracking error plot for {csv_path}.")

    action_cols = [_series(rows, f"action_{idx}") for idx in range(1, 13)]
    action_cols = [col for col in action_cols if col is not None]
    if action_cols:
        actions = np.vstack(action_cols)
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, np.linalg.norm(actions, axis=0), label="action norm")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("magnitude")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "action_magnitude" / f"{prefix}_action_magnitude.png")
    else:
        warnings.warn(f"Skip action magnitude plot for {csv_path}.")

    joint_vel_cols = [_series(rows, f"joint_vel_{idx}") for idx in range(1, 13)]
    joint_vel_cols = [col for col in joint_vel_cols if col is not None]
    joint_torque_cols = [_series(rows, f"joint_torque_{idx}") for idx in range(1, 13)]
    joint_torque_cols = [col for col in joint_torque_cols if col is not None]
    if joint_vel_cols:
        joint_vel = np.vstack(joint_vel_cols)
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, np.nanmean(np.abs(joint_vel), axis=0), label="mean abs joint velocity")
        ax.plot(t, np.nanmax(np.abs(joint_vel), axis=0), label="max abs joint velocity")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("rad/s")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "joint_tracking_error" / f"{prefix}_joint_velocity.png")
    if joint_torque_cols:
        joint_torque = np.vstack(joint_torque_cols)
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, np.nanmean(np.abs(joint_torque), axis=0), label="mean abs joint torque")
        ax.plot(t, np.nanmax(np.abs(joint_torque), axis=0), label="max abs joint torque")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("tau_est")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "joint_tracking_error" / f"{prefix}_joint_torque.png")

    policy_ms = _series(rows, "policy_inference_ms")
    loop_dt = _series(rows, "loop_dt_ms")
    freq = _series(rows, "control_frequency_hz")
    if policy_ms is not None:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t, policy_ms, label="policy inference")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("ms")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "latency" / f"{prefix}_policy_inference_ms.png")
    else:
        warnings.warn(f"Skip policy latency plot for {csv_path}.")

    if loop_dt is not None or freq is not None:
        fig, ax1 = plt.subplots(figsize=(8, 4))
        if loop_dt is not None:
            ax1.plot(t, loop_dt, label="loop_dt_ms", color="tab:blue")
            ax1.set_ylabel("loop dt [ms]", color="tab:blue")
        if freq is not None:
            ax2 = ax1.twinx()
            ax2.plot(t, freq, label="control_frequency_hz", color="tab:orange")
            ax2.set_ylabel("frequency [Hz]", color="tab:orange")
        ax1.set_xlabel("time [s]")
        ax1.grid(True, alpha=0.3)
        _save(fig, figures / "latency" / f"{prefix}_loop_timing.png")
    else:
        warnings.warn(f"Skip loop timing plot for {csv_path}.")

    battery_voltage = _series(rows, "battery_voltage")
    battery_current = _series(rows, "battery_current")
    if battery_voltage is not None or battery_current is not None:
        fig, ax = plt.subplots(figsize=(8, 4))
        if battery_voltage is not None:
            ax.plot(t, battery_voltage, label="battery_voltage")
        if battery_current is not None:
            ax.plot(t, battery_current, label="battery_current")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid(True, alpha=0.3)
        _save(fig, figures / "latency" / f"{prefix}_battery.png")
    else:
        warnings.warn(f"Skip battery plot for {csv_path}: battery fields are missing/NaN.")


def main():
    parser = argparse.ArgumentParser(description="Plot real Unitree Go2 CSV logs.")
    parser.add_argument("--output_root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--csv", default=None, help="Optional single CSV path. Default plots every CSV under logs/.")
    parser.add_argument("--include_deprecated", action="store_true",
                        help="Also include deprecated fixed-speed stages when plotting all CSV logs.")
    args = parser.parse_args()

    output_root = Path(args.output_root).expanduser().resolve()
    csv_paths = [Path(args.csv).expanduser().resolve()] if args.csv else sorted((output_root / "logs").rglob("*.csv"))
    if not csv_paths:
        print(f"No CSV logs found under {output_root / 'logs'}")
        return
    for path in csv_paths:
        if not args.csv and not args.include_deprecated:
            rows = _read_csv(path)
            if rows and rows[0].get("experiment_stage", path.parent.name) not in ACTIVE_STAGES:
                continue
        plot_trial(path, output_root)
    print(f"Wrote figures under {output_root / 'figures'}")


if __name__ == "__main__":
    main()
