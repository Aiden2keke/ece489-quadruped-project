#!/usr/bin/env python3
"""Compare RL and Convex MPC baseline summary metrics."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-cache")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


METRICS = [
    ("rms_velocity_error", "RMS Velocity Error", "m/s", "comparison_velocity_error.png"),
    ("rms_roll", "RMS Roll", "rad", "comparison_roll_pitch.png"),
    ("cot", "Cost of Transport", "CoT", "comparison_cot.png"),
    ("push_recovery_success", "Push Recovery Success", "success rate", "comparison_push_success.png"),
]


def read_summary(path: Path) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return {row["condition"]: row for row in reader}


def finite_float(value: str | None) -> float:
    try:
        return float(value) if value is not None else float("nan")
    except ValueError:
        return float("nan")


def write_comparison(rl: dict[str, dict[str, str]], baseline: dict[str, dict[str, str]], out_path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for condition in sorted(set(rl) | set(baseline)):
        for metric, _, _, _ in METRICS:
            rl_mean = finite_float(rl.get(condition, {}).get(f"{metric}_mean"))
            base_mean = finite_float(baseline.get(condition, {}).get(f"{metric}_mean"))
            rows.append(
                {
                    "condition": condition,
                    "metric": metric,
                    "rl_mean": rl_mean,
                    "rl_std": finite_float(rl.get(condition, {}).get(f"{metric}_std")),
                    "baseline_mean": base_mean,
                    "baseline_std": finite_float(baseline.get(condition, {}).get(f"{metric}_std")),
                    "baseline_minus_rl": base_mean - rl_mean if np.isfinite(base_mean) and np.isfinite(rl_mean) else float("nan"),
                }
            )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["condition", "metric"])
        writer.writeheader()
        writer.writerows(rows)
    return rows


def plot_metric(rl: dict[str, dict[str, str]], baseline: dict[str, dict[str, str]], metric: str, title: str, ylabel: str, out_path: Path) -> None:
    conditions = sorted(set(rl) & set(baseline))
    if metric == "push_recovery_success":
        conditions = [condition for condition in conditions if condition.startswith("push")]
    finite_conditions = [
        condition
        for condition in conditions
        if np.isfinite(finite_float(rl[condition].get(f"{metric}_mean")))
        or np.isfinite(finite_float(baseline[condition].get(f"{metric}_mean")))
    ]
    if not finite_conditions:
        print(f"WARNING: no finite data for {out_path.name}")
        return

    x = np.arange(len(finite_conditions))
    width = 0.36
    rl_mean = [finite_float(rl[condition].get(f"{metric}_mean")) for condition in finite_conditions]
    rl_std = [finite_float(rl[condition].get(f"{metric}_std")) for condition in finite_conditions]
    baseline_mean = [finite_float(baseline[condition].get(f"{metric}_mean")) for condition in finite_conditions]
    baseline_std = [finite_float(baseline[condition].get(f"{metric}_std")) for condition in finite_conditions]

    plt.figure(figsize=(8, 4.8))
    plt.bar(x - width / 2, rl_mean, width, yerr=rl_std, capsize=4, label="RL")
    plt.bar(x + width / 2, baseline_mean, width, yerr=baseline_std, capsize=4, label="Convex MPC")
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xticks(x, finite_conditions, rotation=30, ha="right")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")


def plot_roll_pitch(rl: dict[str, dict[str, str]], baseline: dict[str, dict[str, str]], out_path: Path) -> None:
    conditions = sorted(set(rl) & set(baseline))
    if not conditions:
        print(f"WARNING: no shared conditions for {out_path.name}")
        return
    x = np.arange(len(conditions))
    width = 0.2

    plt.figure(figsize=(9, 4.8))
    for offset, source, label, metric in [
        (-1.5 * width, rl, "RL roll", "rms_roll"),
        (-0.5 * width, rl, "RL pitch", "rms_pitch"),
        (0.5 * width, baseline, "MPC roll", "rms_roll"),
        (1.5 * width, baseline, "MPC pitch", "rms_pitch"),
    ]:
        values = [finite_float(source[condition].get(f"{metric}_mean")) for condition in conditions]
        stds = [finite_float(source[condition].get(f"{metric}_std")) for condition in conditions]
        plt.bar(x + offset, values, width, yerr=stds, capsize=3, label=label)

    plt.title("RMS Roll/Pitch Comparison")
    plt.ylabel("rad")
    plt.xticks(x, conditions, rotation=30, ha="right")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rl_summary", type=Path, default=Path("project_outputs_RL/eval_results/summary_metrics.csv"))
    parser.add_argument("--baseline_summary", type=Path, default=Path("project_outputs_baseline/eval_results/summary_metrics.csv"))
    parser.add_argument("--out_dir", type=Path, default=Path("project_outputs_comparison"))
    args = parser.parse_args()

    rl = read_summary(args.rl_summary)
    baseline = read_summary(args.baseline_summary)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    write_comparison(rl, baseline, args.out_dir / "comparison_summary.csv")
    plot_metric(rl, baseline, "rms_velocity_error", "RMS Velocity Error Comparison", "m/s", args.out_dir / "comparison_velocity_error.png")
    plot_roll_pitch(rl, baseline, args.out_dir / "comparison_roll_pitch.png")
    plot_metric(rl, baseline, "cot", "Cost of Transport Comparison", "CoT", args.out_dir / "comparison_cot.png")
    plot_metric(rl, baseline, "push_recovery_success", "Push Recovery Success Comparison", "success rate", args.out_dir / "comparison_push_success.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
