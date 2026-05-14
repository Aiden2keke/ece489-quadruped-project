#!/usr/bin/env python3
import argparse
import csv
import math
import warnings
from collections import defaultdict
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "real_robot_outputs"
STAGE_ORDER = {
    "stand": 0,
    "remote_control_walk": 1,
    "push_light": 2,
}
ACTIVE_STAGES = set(STAGE_ORDER)


def _to_float(value):
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def _truthy(value):
    if value is None:
        return False
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y", "是"}
    return bool(value)


def _read_csv(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return rows


def _series(rows, key):
    return np.array([_to_float(row.get(key)) for row in rows], dtype=float)


def _mean(values):
    values = np.asarray(values, dtype=float)
    return float(np.nanmean(values)) if np.isfinite(values).any() else math.nan


def _rms(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.nanmean(values ** 2))) if np.isfinite(values).any() else math.nan


def _max_abs(values):
    values = np.asarray(values, dtype=float)
    return float(np.nanmax(np.abs(values))) if np.isfinite(values).any() else math.nan


def _vector_columns(rows, prefix, count=12):
    cols = []
    for idx in range(1, count + 1):
        key = f"{prefix}_{idx}"
        if key in rows[0]:
            cols.append(_series(rows, key))
    return cols


def _matrix_from_columns(rows, prefix, count=12):
    cols = _vector_columns(rows, prefix, count)
    return np.vstack(cols).T if cols else np.empty((len(rows), 0))


def _stick_norm(rows, x_key, y_key):
    x = _series(rows, x_key)
    y = _series(rows, y_key)
    return np.sqrt(x ** 2 + y ** 2)


def _parse_metadata(path):
    metadata = {}
    if not path or not Path(path).exists():
        return metadata
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, value = line.split(":", 1)
        metadata[key.strip()] = value.strip()
    return metadata


def summarize_trial(csv_path):
    rows = _read_csv(csv_path)
    if not rows:
        return None

    time_s = _series(rows, "time")
    cmd_vx = _series(rows, "cmd_vx")
    base_vx = _series(rows, "base_vx")
    roll = _series(rows, "roll")
    pitch = _series(rows, "pitch")
    policy_ms = _series(rows, "policy_inference_ms")
    loop_dt_ms = _series(rows, "loop_dt_ms")
    frequency = _series(rows, "control_frequency_hz")
    battery_voltage = _series(rows, "battery_voltage")

    joint_err_cols = _vector_columns(rows, "joint_tracking_error", 12)
    action_cols = _vector_columns(rows, "action", 12)
    joint_vel = _matrix_from_columns(rows, "joint_vel", 12)
    joint_torque = _matrix_from_columns(rows, "joint_torque", 12)

    joint_err = np.vstack(joint_err_cols).T if joint_err_cols else np.empty((len(rows), 0))
    actions = np.vstack(action_cols).T if action_cols else np.empty((len(rows), 0))
    action_mag = np.linalg.norm(actions, axis=1) if actions.size else np.full(len(rows), math.nan)
    remote_left_norm = _stick_norm(rows, "remote_left_stick_x", "remote_left_stick_y")
    remote_right_norm = _stick_norm(rows, "remote_right_stick_x", "remote_right_stick_y")

    metadata = _parse_metadata(rows[0].get("metadata_path"))
    success = metadata.get("success", rows[0].get("success_flag", ""))
    stage = rows[0].get("experiment_stage", csv_path.parent.name)

    finite_time = time_s[np.isfinite(time_s)]
    duration = float(finite_time[-1] - finite_time[0]) if len(finite_time) >= 2 else math.nan
    battery_drop = math.nan
    finite_battery = battery_voltage[np.isfinite(battery_voltage)]
    if len(finite_battery) >= 2:
        battery_drop = float(finite_battery[0] - finite_battery[-1])

    velocity_error = base_vx - cmd_vx
    if rows[0].get("logger_type") == "lcm_side_logger" and not actions.size:
        warnings.warn(f"{csv_path} is a side_logger CSV: policy obs/action/latency fields are expected to be absent.")

    return {
        "csv_path": str(csv_path),
        "trial_name": rows[0].get("trial_name", csv_path.stem),
        "experiment_stage": stage,
        "command_mode": rows[0].get("command_mode", ""),
        "duration": duration,
        "success": success,
        "fall_flag": int(any(_truthy(row.get("fall_flag")) for row in rows) or _truthy(metadata.get("fall"))),
        "manual_stop_flag": int(any(_truthy(row.get("manual_stop_flag")) for row in rows) or _truthy(metadata.get("manual_stop"))),
        "mean_cmd_vx": _mean(cmd_vx),
        "mean_base_vx": _mean(base_vx),
        "rms_velocity_tracking_error": _rms(velocity_error),
        "rms_roll": _rms(roll),
        "rms_pitch": _rms(pitch),
        "max_abs_roll": _max_abs(roll),
        "max_abs_pitch": _max_abs(pitch),
        "mean_policy_inference_ms": _mean(policy_ms),
        "mean_loop_dt_ms": _mean(loop_dt_ms),
        "mean_control_frequency_hz": _mean(frequency),
        "mean_joint_tracking_error": _mean(np.abs(joint_err)) if joint_err.size else math.nan,
        "max_joint_tracking_error": _max_abs(joint_err) if joint_err.size else math.nan,
        "mean_action_magnitude": _mean(action_mag),
        "mean_joint_velocity_abs": _mean(np.abs(joint_vel)) if joint_vel.size else math.nan,
        "max_joint_velocity_abs": _max_abs(joint_vel) if joint_vel.size else math.nan,
        "mean_joint_torque_abs": _mean(np.abs(joint_torque)) if joint_torque.size else math.nan,
        "max_joint_torque_abs": _max_abs(joint_torque) if joint_torque.size else math.nan,
        "mean_remote_left_stick_norm": _mean(remote_left_norm),
        "mean_remote_right_stick_norm": _mean(remote_right_norm),
        "battery_drop": battery_drop,
        "video_filename_suggested": rows[0].get("video_filename_suggested", ""),
    }


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def build_stage_summary(trial_rows):
    grouped = defaultdict(list)
    for row in trial_rows:
        grouped[row["experiment_stage"]].append(row)

    summary = []
    metric_keys = [
        "duration",
        "mean_cmd_vx",
        "mean_base_vx",
        "rms_velocity_tracking_error",
        "rms_roll",
        "rms_pitch",
        "max_abs_roll",
        "max_abs_pitch",
        "mean_policy_inference_ms",
        "mean_loop_dt_ms",
        "mean_control_frequency_hz",
        "mean_joint_tracking_error",
        "max_joint_tracking_error",
        "mean_action_magnitude",
        "mean_joint_velocity_abs",
        "max_joint_velocity_abs",
        "mean_joint_torque_abs",
        "max_joint_torque_abs",
        "mean_remote_left_stick_norm",
        "mean_remote_right_stick_norm",
        "battery_drop",
    ]
    def stage_sort_key(item):
        stage, _ = item
        return (STAGE_ORDER.get(stage, 100), stage)

    for stage, rows in sorted(grouped.items(), key=stage_sort_key):
        out = {
            "experiment_stage": stage,
            "num_trials": len(rows),
            "num_success": sum(1 for row in rows if _truthy(row.get("success"))),
            "num_fall": sum(int(row.get("fall_flag", 0)) for row in rows),
            "num_manual_stop": sum(int(row.get("manual_stop_flag", 0)) for row in rows),
        }
        for key in metric_keys:
            values = np.array([_to_float(row.get(key)) for row in rows], dtype=float)
            out[f"{key}_mean"] = _mean(values)
            out[f"{key}_std"] = float(np.nanstd(values)) if np.isfinite(values).any() else math.nan
        summary.append(out)
    return summary


def main():
    parser = argparse.ArgumentParser(description="Summarize real Unitree Go2 CSV logs.")
    parser.add_argument("--output_root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--include_deprecated", action="store_true",
                        help="Also include deprecated fixed-speed stages such as flat_vx0p1 and speed_sweep.")
    args = parser.parse_args()

    output_root = Path(args.output_root).expanduser().resolve()
    logs_root = output_root / "logs"
    summary_root = output_root / "summary"
    csv_paths = sorted(logs_root.rglob("*.csv"))
    if not csv_paths:
        print(f"No CSV logs found under {logs_root}")
        return

    trial_rows = []
    for path in csv_paths:
        row = summarize_trial(path)
        if row is not None and (args.include_deprecated or row["experiment_stage"] in ACTIVE_STAGES):
            trial_rows.append(row)

    write_csv(summary_root / "trial_metrics.csv", trial_rows)
    write_csv(summary_root / "summary_metrics.csv", build_stage_summary(trial_rows))
    print(f"Wrote {summary_root / 'trial_metrics.csv'}")
    print(f"Wrote {summary_root / 'summary_metrics.csv'}")


if __name__ == "__main__":
    main()
