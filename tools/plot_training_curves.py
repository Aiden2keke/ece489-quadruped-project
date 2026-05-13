#!/usr/bin/env python3
"""Plot report-ready training curves from exported TensorBoard scalar CSVs."""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-cache")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_scalars(csv_dir: Path) -> dict[str, list[dict[str, object]]]:
    scalars: dict[str, list[dict[str, object]]] = defaultdict(list)
    for path in sorted(csv_dir.glob("*.csv")):
        with path.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            if not {"run", "tag", "step", "value"}.issubset(reader.fieldnames or []):
                continue
            for row in reader:
                try:
                    step = float(row["step"])
                    value = float(row["value"])
                except (TypeError, ValueError):
                    continue
                scalars[row["tag"]].append({"run": row.get("run", ""), "step": step, "value": value})
    return scalars


def moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1 or len(values) < window:
        return values
    averaged = []
    running = 0.0
    for i, value in enumerate(values):
        running += value
        if i >= window:
            running -= values[i - window]
        denom = min(i + 1, window)
        averaged.append(running / denom)
    return averaged


def plot_tags(
    scalars: dict[str, list[dict[str, object]]],
    tags: list[str],
    out_path: Path,
    title: str,
    ylabel: str,
    smooth_window: int,
) -> bool:
    available = [tag for tag in tags if tag in scalars]
    if not available:
        print(f"WARNING: no data for {out_path.name}; expected tags like {tags}")
        return False

    plt.figure(figsize=(8, 4.8))
    for tag in available:
        rows_by_run: dict[str, list[dict[str, object]]] = defaultdict(list)
        for row in scalars[tag]:
            rows_by_run[str(row["run"])].append(row)
        for run, rows in sorted(rows_by_run.items()):
            rows = sorted(rows, key=lambda r: r["step"])
            xs = [float(r["step"]) for r in rows]
            ys = moving_average([float(r["value"]) for r in rows], smooth_window)
            if len(available) == 1 and len(rows_by_run) > 1:
                label = short_run_name(run)
            elif len(rows_by_run) > 1:
                label = f"{tag} [{short_run_name(run)}]"
            else:
                label = tag
            plt.plot(xs, ys, label=label)

    plt.title(title)
    plt.xlabel("Iteration")
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Wrote {out_path}")
    return True


def reward_label(tag: str) -> str:
    prefix = "Episode/rew_"
    if tag.startswith(prefix):
        tag = tag[len(prefix) :]
    return tag.replace("_", " ")


def short_run_name(run: str) -> str:
    path = Path(run)
    if path.name == "rl_tfevents_logs" and path.parent.name:
        return path.parent.name
    if path.name:
        return path.name
    return run or "run"


def sort_reward_tags(tags: list[str]) -> list[str]:
    preferred = [
        "tracking_lin_vel",
        "tracking_ang_vel",
        "orientation",
        "base_height",
        "lin_vel_z",
        "ang_vel_xy",
        "feet_air_time",
        "feet_clearance",
        "action_rate",
        "action_smoothness",
        "torques",
        "dof_acc",
        "dof_pos_limits",
        "collision",
        "termination",
    ]
    order = {f"Episode/rew_{name}": i for i, name in enumerate(preferred)}
    return sorted(tags, key=lambda tag: (order.get(tag, len(order)), tag))


def plot_reward_components(
    scalars: dict[str, list[dict[str, object]]],
    reward_tags: list[str],
    out_path: Path,
    smooth_window: int,
) -> bool:
    reward_tags = sort_reward_tags([tag for tag in reward_tags if tag in scalars])
    if not reward_tags:
        print(f"WARNING: no reward component data for {out_path.name}")
        return False

    ncols = 4 if len(reward_tags) > 9 else 3
    nrows = math.ceil(len(reward_tags) / ncols)
    fig, axes = plt.subplots(
        nrows,
        ncols,
        figsize=(4.2 * ncols, 2.45 * nrows),
        sharex=True,
        squeeze=False,
    )
    flat_axes = list(axes.ravel())

    legend_handles = []
    legend_labels = []
    for ax, tag in zip(flat_axes, reward_tags):
        rows_by_run: dict[str, list[dict[str, object]]] = defaultdict(list)
        for row in scalars[tag]:
            rows_by_run[str(row["run"])].append(row)

        for run, rows in sorted(rows_by_run.items()):
            rows = sorted(rows, key=lambda r: r["step"])
            xs = [float(r["step"]) for r in rows]
            ys = moving_average([float(r["value"]) for r in rows], smooth_window)
            label = short_run_name(run)
            (line,) = ax.plot(xs, ys, linewidth=1.3, label=label)
            if label not in legend_labels:
                legend_handles.append(line)
                legend_labels.append(label)

        ax.set_title(reward_label(tag), fontsize=9)
        ax.grid(True, alpha=0.28)
        ax.tick_params(labelsize=8)
        ax.axhline(0.0, color="black", linewidth=0.7, alpha=0.35)

    for ax in flat_axes[len(reward_tags) :]:
        ax.axis("off")

    for ax in axes[-1, :]:
        ax.set_xlabel("Iteration")
    for ax in axes[:, 0]:
        ax.set_ylabel("Contribution")

    fig.suptitle("Reward Components", fontsize=14)
    if len(legend_labels) > 1:
        fig.legend(
            legend_handles,
            legend_labels,
            loc="upper center",
            ncol=min(len(legend_labels), 4),
            bbox_to_anchor=(0.5, 0.985),
            fontsize=8,
        )
        fig.tight_layout(rect=(0, 0, 1, 0.94))
    else:
        fig.tight_layout(rect=(0, 0, 1, 0.96))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)
    print(f"Wrote {out_path}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv_dir", type=Path, default=Path("project_outputs/train_curves_csv"))
    parser.add_argument("--out_dir", type=Path, default=Path("project_outputs/figures"))
    parser.add_argument("--smooth_window", type=int, default=10)
    args = parser.parse_args()

    scalars = load_scalars(args.csv_dir)
    if not scalars:
        print(f"WARNING: no scalar CSV data found in {args.csv_dir}")
        return 0

    plot_tags(
        scalars,
        ["Train/mean_reward"],
        args.out_dir / "training_mean_reward.png",
        "Training Mean Reward",
        "Mean episode reward",
        args.smooth_window,
    )
    plot_tags(
        scalars,
        ["Train/mean_episode_length"],
        args.out_dir / "episode_length.png",
        "Episode Length",
        "Steps",
        args.smooth_window,
    )

    reward_tags = sorted(tag for tag in scalars if tag.startswith("Episode/rew_"))
    plot_reward_components(
        scalars,
        reward_tags,
        args.out_dir / "reward_components.png",
        args.smooth_window,
    )

    loss_tags = [tag for tag in ["Loss/value_function", "Loss/surrogate", "Loss/extra"] if tag in scalars]
    plot_tags(
        scalars,
        loss_tags,
        args.out_dir / "ppo_losses.png",
        "PPO Losses",
        "Loss",
        args.smooth_window,
    )

    curriculum_tags = sorted(
        tag
        for tag in scalars
        if "terrain_level" in tag.lower() or "curriculum" in tag.lower() or tag.endswith("max_command_x")
    )
    plot_tags(
        scalars,
        curriculum_tags,
        args.out_dir / "terrain_curriculum.png",
        "Terrain/Curriculum Progress",
        "Value",
        args.smooth_window,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
