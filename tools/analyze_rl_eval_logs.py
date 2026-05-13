#!/usr/bin/env python3
"""Analyze fixed-condition Go2 RL evaluation CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
from typing import Iterable

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-cache")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


G = 9.81
DEFAULT_GO2_MASS_KG = 15.0
MIN_BASE_HEIGHT = 0.18
MAX_ABS_ROLL_PITCH = 0.8


def read_trial(path: Path) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key, value in row.items():
                try:
                    columns.setdefault(key, []).append(float(value))
                except (TypeError, ValueError):
                    columns.setdefault(key, []).append(float("nan"))
    return {key: np.asarray(values, dtype=float) for key, values in columns.items()}


def rms(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(values))))


def compute_dt(time_values: np.ndarray, data: dict[str, np.ndarray]) -> np.ndarray:
    if time_values.size <= 1:
        dt = data.get("sim_dt", np.asarray([0.005], dtype=float))
        return np.full(time_values.shape, float(dt[0]) if dt.size else 0.005)
    diffs = np.diff(time_values)
    first = float(np.median(diffs[diffs > 0])) if np.any(diffs > 0) else 0.005
    return np.concatenate([[first], diffs])


def compute_cot(data: dict[str, np.ndarray]) -> float:
    torque_cols = [data.get(f"joint_torque_{i}") for i in range(1, 13)]
    vel_cols = [data.get(f"joint_vel_{i}") for i in range(1, 13)]
    if any(col is None for col in torque_cols + vel_cols):
        return float("nan")
    torques = np.vstack(torque_cols).T
    qdots = np.vstack(vel_cols).T
    time_values = data.get("time", np.arange(torques.shape[0]) * 0.005)
    dt = compute_dt(time_values, data)
    power = np.sum(np.abs(torques * qdots), axis=1)
    energy = float(np.sum(power * dt[: power.shape[0]]))
    distance_col = data.get("distance_traveled")
    distance = float(distance_col[-1]) if distance_col is not None and distance_col.size else float("nan")
    if not np.isfinite(distance) or distance <= 1e-6:
        return float("nan")
    mass_col = data.get("robot_mass")
    mass = float(mass_col[-1]) if mass_col is not None and mass_col.size and mass_col[-1] > 0 else DEFAULT_GO2_MASS_KG
    return energy / (mass * G * distance)


def compute_success(data: dict[str, np.ndarray]) -> int:
    fall = data.get("fall_flag", np.asarray([1.0]))
    base_z = data.get("base_z", np.asarray([0.0]))
    roll = data.get("roll", np.asarray([math.inf]))
    pitch = data.get("pitch", np.asarray([math.inf]))
    success_logged = data.get("success_flag")
    if fall.size == 0 or base_z.size == 0:
        return 0
    final_ok = (
        base_z[-1] >= MIN_BASE_HEIGHT
        and abs(roll[-1]) <= MAX_ABS_ROLL_PITCH
        and abs(pitch[-1]) <= MAX_ABS_ROLL_PITCH
    )
    logged_ok = True
    if success_logged is not None:
        finite_success = success_logged[np.isfinite(success_logged)]
        if finite_success.size:
            logged_ok = bool(finite_success[-1] > 0.5)
    return int(logged_ok and np.nanmax(fall) == 0 and final_ok)


def compute_push_recovery(data: dict[str, np.ndarray]) -> float:
    push = data.get("external_push_flag")
    if push is None or push.size == 0 or np.nanmax(push) <= 0:
        return float("nan")
    time_values = data.get("time", np.arange(push.size) * 0.005)
    push_times = time_values[push > 0]
    if push_times.size == 0:
        return float("nan")
    push_end = float(push_times[-1])
    after = time_values >= push_end
    if not np.any(after):
        return 0.0
    fall = data.get("fall_flag", np.ones_like(time_values))
    if np.nanmax(fall[after]) > 0:
        return 0.0
    roll = data.get("roll", np.full_like(time_values, math.inf))
    pitch = data.get("pitch", np.full_like(time_values, math.inf))
    cmd_vx = data.get("cmd_vx", np.zeros_like(time_values))
    base_vx = data.get("base_vx", np.full_like(time_values, math.inf))
    recovery_window = time_values >= min(push_end + 2.0, float(time_values[-1]))
    if not np.any(recovery_window):
        recovery_window = after
    stable = (
        (np.abs(roll[recovery_window]) < 0.4)
        & (np.abs(pitch[recovery_window]) < 0.4)
        & (np.abs(cmd_vx[recovery_window] - base_vx[recovery_window]) < 0.35)
    )
    return float(np.mean(stable) >= 0.5)


def analyze_trial(path: Path, eval_root: Path) -> dict[str, object]:
    data = read_trial(path)
    condition = path.parent.relative_to(eval_root).as_posix()
    cmd_vx = data.get("cmd_vx", np.asarray([float("nan")]))
    base_vx = data.get("base_vx", np.asarray([], dtype=float))
    roll = data.get("roll", np.asarray([], dtype=float))
    pitch = data.get("pitch", np.asarray([], dtype=float))
    distance = data.get("distance_traveled", np.asarray([float("nan")]))
    duration = data.get("time", np.asarray([float("nan")]))
    success = compute_success(data)
    return {
        "condition": condition,
        "trial_file": str(path),
        "cmd_vx": float(cmd_vx[-1]) if cmd_vx.size else float("nan"),
        "duration_s": float(duration[-1]) if duration.size else float("nan"),
        "distance_m": float(distance[-1]) if distance.size else float("nan"),
        "rms_velocity_error": rms(cmd_vx[: base_vx.size] - base_vx) if cmd_vx.size and base_vx.size else float("nan"),
        "rms_roll": rms(roll),
        "rms_pitch": rms(pitch),
        "cot": compute_cot(data),
        "success": success,
        "push_recovery_success": compute_push_recovery(data),
        "fall_any": int(np.nanmax(data.get("fall_flag", np.asarray([1.0]))) > 0),
    }


def write_trial_metrics(rows: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "condition",
        "trial_file",
        "cmd_vx",
        "duration_s",
        "distance_m",
        "rms_velocity_error",
        "rms_roll",
        "rms_pitch",
        "cot",
        "success",
        "push_recovery_success",
        "fall_any",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def mean_std(values: Iterable[object]) -> tuple[float, float]:
    arr = np.asarray([float(v) for v in values], dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return float("nan"), float("nan")
    return float(np.mean(arr)), float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0


def write_summary(rows: list[dict[str, object]], path: Path) -> list[dict[str, object]]:
    by_condition: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        by_condition.setdefault(str(row["condition"]), []).append(row)

    summary_rows = []
    for condition, condition_rows in sorted(by_condition.items()):
        summary: dict[str, object] = {"condition": condition, "num_trials": len(condition_rows)}
        for metric in ["rms_velocity_error", "rms_roll", "rms_pitch", "cot", "success", "push_recovery_success"]:
            mean, std = mean_std(row[metric] for row in condition_rows)
            summary[f"{metric}_mean"] = mean
            summary[f"{metric}_std"] = std
            summary[f"{metric}_mean_std"] = "nan" if not np.isfinite(mean) else f"{mean:.4g} +/- {std:.4g}"
        summary_rows.append(summary)

    fieldnames = list(summary_rows[0].keys()) if summary_rows else ["condition", "num_trials"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)
    return summary_rows


def bar_plot(summary_rows: list[dict[str, object]], metric: str, out_path: Path, title: str, ylabel: str) -> None:
    rows = [row for row in summary_rows if np.isfinite(float(row.get(f"{metric}_mean", float("nan"))))]
    if not rows:
        print(f"WARNING: no finite data for {out_path.name}")
        return
    labels = [str(row["condition"]) for row in rows]
    means = [float(row[f"{metric}_mean"]) for row in rows]
    stds = [float(row[f"{metric}_std"]) for row in rows]
    plt.figure(figsize=(8, 4.8))
    plt.bar(labels, means, yerr=stds, capsize=4)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xticks(rotation=30, ha="right")
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")


def roll_pitch_bar(summary_rows: list[dict[str, object]], out_path: Path) -> None:
    rows = [row for row in summary_rows if np.isfinite(float(row.get("rms_roll_mean", float("nan"))))]
    if not rows:
        print(f"WARNING: no finite data for {out_path.name}")
        return
    labels = [str(row["condition"]) for row in rows]
    x = np.arange(len(labels))
    width = 0.36
    roll_mean = [float(row["rms_roll_mean"]) for row in rows]
    pitch_mean = [float(row["rms_pitch_mean"]) for row in rows]
    roll_std = [float(row["rms_roll_std"]) for row in rows]
    pitch_std = [float(row["rms_pitch_std"]) for row in rows]
    plt.figure(figsize=(8, 4.8))
    plt.bar(x - width / 2, roll_mean, width, yerr=roll_std, capsize=4, label="RMS roll")
    plt.bar(x + width / 2, pitch_mean, width, yerr=pitch_std, capsize=4, label="RMS pitch")
    plt.title("RMS Body Roll/Pitch")
    plt.ylabel("Radians")
    plt.xticks(x, labels, rotation=30, ha="right")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")


def find_example(eval_root: Path, prefix: str) -> Path | None:
    candidates = sorted(p for p in eval_root.rglob("*.csv") if p.parent.name.startswith(prefix))
    return candidates[0] if candidates else None


def plot_velocity_example(path: Path | None, out_path: Path, title: str) -> None:
    if path is None:
        print(f"WARNING: no example log for {out_path.name}")
        return
    data = read_trial(path)
    time_values = data.get("time")
    if time_values is None or "base_vx" not in data or "cmd_vx" not in data:
        print(f"WARNING: missing velocity columns in {path}")
        return
    plt.figure(figsize=(8, 4.8))
    plt.plot(time_values, data["cmd_vx"], label="command vx")
    plt.plot(time_values, data["base_vx"], label="base vx")
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")


def plot_roll_pitch_example(path: Path | None, out_path: Path, title: str) -> None:
    if path is None:
        print(f"WARNING: no example log for {out_path.name}")
        return
    data = read_trial(path)
    time_values = data.get("time")
    if time_values is None or "roll" not in data or "pitch" not in data:
        print(f"WARNING: missing roll/pitch columns in {path}")
        return
    plt.figure(figsize=(8, 4.8))
    plt.plot(time_values, data["roll"], label="roll")
    plt.plot(time_values, data["pitch"], label="pitch")
    if "external_push_flag" in data and np.nanmax(data["external_push_flag"]) > 0:
        push_times = time_values[data["external_push_flag"] > 0]
        plt.axvspan(push_times[0], push_times[-1], color="tab:red", alpha=0.2, label="push")
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")


def plot_contact_gait(path: Path | None, out_path: Path) -> None:
    if path is None:
        print(f"WARNING: no example log for {out_path.name}")
        return
    data = read_trial(path)
    time_values = data.get("time")
    contact_cols = [f"foot_contact_{name}" for name in ["FL", "FR", "RL", "RR"]]
    if time_values is None or any(col not in data for col in contact_cols):
        print("WARNING: contact state columns unavailable; skipping contact_gait_diagram.png")
        return
    contacts = np.vstack([data[col] for col in contact_cols])
    plt.figure(figsize=(9, 3.2))
    plt.imshow(
        contacts,
        aspect="auto",
        interpolation="nearest",
        extent=[time_values[0], time_values[-1], -0.5, len(contact_cols) - 0.5],
        origin="lower",
        cmap="Greys",
    )
    plt.yticks(range(len(contact_cols)), [name.replace("foot_contact_", "") for name in contact_cols])
    plt.xlabel("Time (s)")
    plt.title("Contact Gait Diagram")
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval_root", type=Path, default=Path("project_outputs/eval_logs"))
    parser.add_argument("--out_dir", type=Path, default=Path("project_outputs/eval_results"))
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    trial_files = sorted(args.eval_root.rglob("*.csv"))
    if not trial_files:
        print(f"WARNING: no evaluation CSV logs found under {args.eval_root}")
        return 0

    trial_rows = [analyze_trial(path, args.eval_root) for path in trial_files]
    write_trial_metrics(trial_rows, args.out_dir / "trial_metrics.csv")
    summary_rows = write_summary(trial_rows, args.out_dir / "summary_metrics.csv")

    bar_plot(summary_rows, "rms_velocity_error", args.out_dir / "rms_velocity_error.png", "RMS Velocity Tracking Error", "m/s")
    roll_pitch_bar(summary_rows, args.out_dir / "rms_roll_pitch.png")
    bar_plot(summary_rows, "cot", args.out_dir / "cot_comparison.png", "Cost of Transport", "CoT")
    push_rows = [row for row in summary_rows if str(row["condition"]).startswith("push")]
    bar_plot(push_rows, "push_recovery_success", args.out_dir / "push_recovery_success.png", "Push Recovery Success Rate", "Success rate")

    flat_example = find_example(args.eval_root, "flat")
    terrain_example = find_example(args.eval_root, "terrain")
    push_example = find_example(args.eval_root, "push")
    plot_velocity_example(flat_example, args.out_dir / "example_velocity_tracking_flat.png", "Example Velocity Tracking: Flat")
    plot_velocity_example(terrain_example, args.out_dir / "example_velocity_tracking_terrain.png", "Example Velocity Tracking: Terrain")
    plot_roll_pitch_example(flat_example, args.out_dir / "example_roll_pitch_flat.png", "Example Roll/Pitch: Flat")
    plot_roll_pitch_example(terrain_example, args.out_dir / "example_roll_pitch_terrain.png", "Example Roll/Pitch: Terrain")
    plot_roll_pitch_example(push_example, args.out_dir / "example_push_recovery.png", "Example Push Recovery")
    plot_contact_gait(flat_example or terrain_example or push_example, args.out_dir / "contact_gait_diagram.png")
    print(f"Wrote metrics and figures to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
