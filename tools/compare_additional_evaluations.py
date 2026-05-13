#!/usr/bin/env python3
"""Build additional RL-vs-baseline comparison tables and plots.

The script prefers per-trial metrics so CoT can be restricted to successful
full-length trials. It falls back to summary_metrics.csv when trial_metrics.csv
is unavailable, and labels that fallback in the notes column.
"""

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


METHOD_ORDER = ["RL", "baseline"]
SPEED_ORDER = ["flat_0p4", "flat_0p6", "flat_0p8", "flat_1p0", "flat_1p2"]
ROBUSTNESS_ORDER = ["friction_low", "payload_high", "motor_weak"]
FULL_LENGTH_TOL_S = 0.1


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(rows: list[dict[str, object]], path: Path, fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {path}")


def as_float(value: object, default: float = float("nan")) -> float:
    try:
        if value in ("", None):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def mean_std(values: Iterable[object]) -> tuple[float, float]:
    arr = np.asarray([as_float(v) for v in values], dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return float("nan"), float("nan")
    return float(np.mean(arr)), float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0


def condition_sort_key(condition: str, preferred: list[str]) -> tuple[int, str]:
    try:
        return preferred.index(condition), condition
    except ValueError:
        return len(preferred), condition


def method_sort_key(method: str) -> tuple[int, str]:
    try:
        return METHOD_ORDER.index(method), method
    except ValueError:
        return len(METHOD_ORDER), method


def sidecar_controller_failure(row: dict[str, str]) -> int:
    if "controller_failure" in row:
        value = as_float(row.get("controller_failure"))
        if np.isfinite(value):
            return int(value > 0.5)
    trial_file = row.get("trial_file")
    if trial_file:
        return int(Path(trial_file).with_suffix(".error.txt").exists())
    return 0


def aggregate_trial_rows(
    method: str,
    source_task: str,
    trial_rows: list[dict[str, str]],
    expected_duration_s: float,
) -> list[dict[str, object]]:
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in trial_rows:
        grouped.setdefault(str(row.get("condition", "")), []).append(row)

    result: list[dict[str, object]] = []
    for condition, rows in grouped.items():
        successes = [as_float(row.get("success")) for row in rows]
        success_rate, _ = mean_std(successes)
        controller_failure_rate, _ = mean_std(sidecar_controller_failure(row) for row in rows)
        completed = [
            row
            for row in rows
            if as_float(row.get("success")) > 0.5
            and as_float(row.get("duration_s")) >= expected_duration_s - FULL_LENGTH_TOL_S
        ]
        mean_cot, std_cot = mean_std(row.get("cot") for row in completed)
        fall_times = [as_float(row.get("fall_time_s")) for row in rows]
        fall_time_failed_mean, _ = mean_std(fall_times)
        mean_vx, _ = mean_std(row.get("cmd_vx") for row in rows)
        mean_velocity_error, _ = mean_std(row.get("rms_velocity_error") for row in rows)
        mean_roll, _ = mean_std(row.get("rms_roll") for row in rows)
        mean_pitch, _ = mean_std(row.get("rms_pitch") for row in rows)
        notes = "CoT from successful full-length trials only; failed/partial trials excluded"
        if not completed:
            notes = "no successful full-length trials; CoT not reported"

        result.append(
            {
                "method": method,
                "source_task": source_task,
                "condition": condition,
                "commanded_vx": mean_vx,
                "num_trials": len(rows),
                "success_rate": success_rate,
                "controller_failure_rate": controller_failure_rate,
                "mean_CoT": mean_cot,
                "std_CoT": std_cot,
                "mean_RMS_velocity_error": mean_velocity_error,
                "mean_RMS_roll": mean_roll,
                "mean_RMS_pitch": mean_pitch,
                "fall_time_failed_mean": fall_time_failed_mean,
                "notes": notes,
            }
        )
    return result


def aggregate_summary_rows(method: str, source_task: str, summary_rows: list[dict[str, str]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for row in summary_rows:
        result.append(
            {
                "method": method,
                "source_task": source_task,
                "condition": row.get("condition", ""),
                "commanded_vx": float("nan"),
                "num_trials": as_float(row.get("num_trials")),
                "success_rate": as_float(row.get("success_mean")),
                "controller_failure_rate": as_float(row.get("controller_failure_mean")),
                "mean_CoT": as_float(row.get("cot_mean")),
                "std_CoT": as_float(row.get("cot_std")),
                "mean_RMS_velocity_error": as_float(row.get("rms_velocity_error_mean")),
                "mean_RMS_roll": as_float(row.get("rms_roll_mean")),
                "mean_RMS_pitch": as_float(row.get("rms_pitch_mean")),
                "fall_time_failed_mean": as_float(row.get("fall_time_s_mean")),
                "notes": "summary fallback; CoT may include failed/partial trials",
            }
        )
    return result


def load_condition_metrics(
    method: str,
    source_task: str,
    results_dir: Path,
    expected_duration_s: float,
) -> list[dict[str, object]]:
    trial_rows = read_csv(results_dir / "trial_metrics.csv")
    if trial_rows:
        return aggregate_trial_rows(method, source_task, trial_rows, expected_duration_s)
    summary_rows = read_csv(results_dir / "summary_metrics.csv")
    if summary_rows:
        return aggregate_summary_rows(method, source_task, summary_rows)
    print(f"WARNING: no metrics found under {results_dir}")
    return []


def save_grouped_bar(
    rows: list[dict[str, object]],
    metric: str,
    out_path: Path,
    title: str,
    ylabel: str,
    preferred_conditions: list[str],
) -> bool:
    finite_rows = [row for row in rows if np.isfinite(as_float(row.get(metric)))]
    if not finite_rows:
        print(f"WARNING: no finite data for {out_path.name}; skipping")
        return False

    conditions = sorted({str(row["condition"]) for row in rows}, key=lambda c: condition_sort_key(c, preferred_conditions))
    methods = [method for method in METHOD_ORDER if any(row.get("method") == method for row in rows)]
    x = np.arange(len(conditions))
    width = 0.8 / max(1, len(methods))

    plt.figure(figsize=(max(7.5, 1.1 * len(conditions)), 4.8))
    for i, method in enumerate(methods):
        values = []
        for condition in conditions:
            match = next((row for row in rows if row.get("method") == method and row.get("condition") == condition), None)
            values.append(as_float(match.get(metric)) if match else float("nan"))
        offsets = x + (i - (len(methods) - 1) / 2) * width
        plt.bar(offsets, values, width=width, label=method)

    plt.title(title)
    plt.ylabel(ylabel)
    plt.xticks(x, conditions, rotation=30, ha="right")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")
    return True


def save_energy_plot(rows: list[dict[str, object]], out_path: Path) -> bool:
    finite_rows = [row for row in rows if np.isfinite(as_float(row.get("mean_CoT")))]
    if not finite_rows:
        print(f"WARNING: no finite data for {out_path.name}; skipping")
        return False

    labels = []
    for row in finite_rows:
        labels.append(f"{row['source_task']}:{row['condition']}")
    labels = sorted(set(labels))
    methods = [method for method in METHOD_ORDER if any(row.get("method") == method for row in finite_rows)]
    x = np.arange(len(labels))
    width = 0.8 / max(1, len(methods))

    plt.figure(figsize=(max(9.0, 0.7 * len(labels)), 5.2))
    for i, method in enumerate(methods):
        values = []
        for label in labels:
            source_task, condition = label.split(":", 1)
            match = next(
                (
                    row
                    for row in finite_rows
                    if row.get("method") == method
                    and row.get("source_task") == source_task
                    and row.get("condition") == condition
                ),
                None,
            )
            values.append(as_float(match.get("mean_CoT")) if match else float("nan"))
        offsets = x + (i - (len(methods) - 1) / 2) * width
        plt.bar(offsets, values, width=width, label=method)

    plt.title("Energy Efficiency: Successful Full-Length Trials")
    plt.ylabel("Mean CoT")
    plt.xticks(x, labels, rotation=35, ha="right")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")
    return True


def sort_rows(rows: list[dict[str, object]], preferred_conditions: list[str]) -> list[dict[str, object]]:
    return sorted(
        rows,
        key=lambda row: (
            str(row.get("source_task", "")),
            condition_sort_key(str(row.get("condition", "")), preferred_conditions),
            method_sort_key(str(row.get("method", ""))),
        ),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--additional_root", type=Path, default=Path("project_outputs_additional"))
    parser.add_argument("--core_rl_results", type=Path, default=Path("project_outputs_RL/eval_results"))
    parser.add_argument("--core_baseline_results", type=Path, default=Path("project_outputs_baseline/eval_results"))
    parser.add_argument("--expected_duration", type=float, default=20.0)
    args = parser.parse_args()

    comparison_dir = args.additional_root / "comparison"
    comparison_dir.mkdir(parents=True, exist_ok=True)

    speed_rows = []
    speed_rows.extend(
        load_condition_metrics(
            "RL",
            "speed_sweep",
            args.additional_root / "RL" / "speed_sweep" / "eval_results",
            args.expected_duration,
        )
    )
    speed_rows.extend(
        load_condition_metrics(
            "baseline",
            "speed_sweep",
            args.additional_root / "baseline" / "speed_sweep" / "eval_results",
            args.expected_duration,
        )
    )
    speed_rows = sort_rows(speed_rows, SPEED_ORDER)

    robustness_rows = []
    robustness_rows.extend(
        load_condition_metrics(
            "RL",
            "robustness",
            args.additional_root / "RL" / "robustness" / "eval_results",
            args.expected_duration,
        )
    )
    robustness_rows.extend(
        load_condition_metrics(
            "baseline",
            "robustness",
            args.additional_root / "baseline" / "robustness" / "eval_results",
            args.expected_duration,
        )
    )
    robustness_rows = sort_rows(robustness_rows, ROBUSTNESS_ORDER)

    comparison_fields = [
        "method",
        "source_task",
        "condition",
        "commanded_vx",
        "num_trials",
        "success_rate",
        "controller_failure_rate",
        "mean_CoT",
        "std_CoT",
        "mean_RMS_velocity_error",
        "mean_RMS_roll",
        "mean_RMS_pitch",
        "fall_time_failed_mean",
        "notes",
    ]
    write_csv(speed_rows, comparison_dir / "comparison_speed_sweep.csv", comparison_fields)
    write_csv(robustness_rows, comparison_dir / "comparison_robustness.csv", comparison_fields)

    energy_rows = []
    energy_rows.extend(load_condition_metrics("RL", "core_eval", args.core_rl_results, args.expected_duration))
    energy_rows.extend(load_condition_metrics("baseline", "core_eval", args.core_baseline_results, args.expected_duration))
    energy_rows.extend(speed_rows)
    energy_rows = sorted(
        energy_rows,
        key=lambda row: (
            str(row.get("source_task", "")),
            condition_sort_key(str(row.get("condition", "")), SPEED_ORDER),
            method_sort_key(str(row.get("method", ""))),
        ),
    )
    energy_fields = [
        "method",
        "source_task",
        "condition",
        "success_rate",
        "mean_CoT",
        "std_CoT",
        "mean_RMS_velocity_error",
        "mean_RMS_roll",
        "mean_RMS_pitch",
        "notes",
    ]
    write_csv(
        [
            {field: row.get(field, "") for field in energy_fields}
            for row in energy_rows
        ],
        comparison_dir / "comparison_energy_efficiency.csv",
        energy_fields,
    )

    generated = []
    if save_grouped_bar(
        speed_rows,
        "success_rate",
        comparison_dir / "comparison_speed_sweep_success_rate.png",
        "Flat Speed Sweep Success Rate",
        "Success rate",
        SPEED_ORDER,
    ):
        generated.append("comparison_speed_sweep_success_rate.png")
    if save_grouped_bar(
        speed_rows,
        "mean_CoT",
        comparison_dir / "comparison_speed_sweep_cot.png",
        "Flat Speed Sweep CoT",
        "Mean CoT",
        SPEED_ORDER,
    ):
        generated.append("comparison_speed_sweep_cot.png")
    if save_grouped_bar(
        speed_rows,
        "mean_RMS_velocity_error",
        comparison_dir / "comparison_speed_sweep_velocity_error.png",
        "Flat Speed Sweep Velocity Error",
        "RMS error (m/s)",
        SPEED_ORDER,
    ):
        generated.append("comparison_speed_sweep_velocity_error.png")
    if save_energy_plot(energy_rows, comparison_dir / "comparison_energy_efficiency.png"):
        generated.append("comparison_energy_efficiency.png")
    if save_grouped_bar(
        robustness_rows,
        "success_rate",
        comparison_dir / "comparison_robustness_success_rate.png",
        "Domain Randomization Robustness Success Rate",
        "Success rate",
        ROBUSTNESS_ORDER,
    ):
        generated.append("comparison_robustness_success_rate.png")
    if save_grouped_bar(
        robustness_rows,
        "mean_CoT",
        comparison_dir / "comparison_robustness_cot.png",
        "Domain Randomization Robustness CoT",
        "Mean CoT",
        ROBUSTNESS_ORDER,
    ):
        generated.append("comparison_robustness_cot.png")
    if save_grouped_bar(
        robustness_rows,
        "mean_RMS_velocity_error",
        comparison_dir / "comparison_robustness_velocity_error.png",
        "Domain Randomization Robustness Velocity Error",
        "RMS error (m/s)",
        ROBUSTNESS_ORDER,
    ):
        generated.append("comparison_robustness_velocity_error.png")

    print("Generated comparison plots: " + (", ".join(generated) if generated else "none"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
