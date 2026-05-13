#!/usr/bin/env python3
"""Export TensorBoard scalar event data to per-tag CSV files."""

from __future__ import annotations

import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path


def sanitize_tag(tag: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", tag.strip())
    return safe.strip("_") or "scalar"


def find_event_files(log_root: Path) -> list[Path]:
    return sorted(p for p in log_root.rglob("events.out.tfevents*") if p.is_file())


def write_readme(out_dir: Path) -> None:
    readme = out_dir / "README.md"
    readme.write_text(
        "# Training Curve CSV Export\n\n"
        "Run from the repository root:\n\n"
        "```bash\n"
        "python tools/export_tensorboard_scalars.py --log_root legged_gym/logs --out_dir project_outputs/train_curves_csv\n"
        "```\n\n"
        "Each scalar tag is exported to one CSV with columns: `run`, `event_file`, `tag`, `wall_time`, `step`, `value`.\n"
        "If no TensorBoard event files exist, the script prints a warning and leaves this README in place.\n",
        encoding="utf-8",
    )


def export_scalars(log_root: Path, out_dir: Path) -> int:
    try:
        from tensorboard.backend.event_processing import event_accumulator
    except ImportError as exc:
        raise SystemExit(
            "TensorBoard is not installed. Install tensorboard or run inside the training environment."
        ) from exc

    out_dir.mkdir(parents=True, exist_ok=True)
    write_readme(out_dir)

    event_files = find_event_files(log_root)
    if not event_files:
        print(f"WARNING: no TensorBoard event files found under {log_root}")
        return 0

    rows_by_tag: dict[str, list[dict[str, object]]] = defaultdict(list)
    for event_file in event_files:
        run = str(event_file.parent.relative_to(log_root))
        accumulator = event_accumulator.EventAccumulator(
            str(event_file),
            size_guidance={event_accumulator.SCALARS: 0},
        )
        try:
            accumulator.Reload()
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: failed to read {event_file}: {exc}")
            continue
        for tag in accumulator.Tags().get("scalars", []):
            for scalar in accumulator.Scalars(tag):
                rows_by_tag[tag].append(
                    {
                        "run": run,
                        "event_file": str(event_file),
                        "tag": tag,
                        "wall_time": scalar.wall_time,
                        "step": scalar.step,
                        "value": scalar.value,
                    }
                )

    if not rows_by_tag:
        print(f"WARNING: event files found under {log_root}, but no scalar tags were exported")
        return 0

    fieldnames = ["run", "event_file", "tag", "wall_time", "step", "value"]
    for tag, rows in sorted(rows_by_tag.items()):
        path = out_dir / f"{sanitize_tag(tag)}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"Exported {tag}: {len(rows)} rows -> {path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_root", type=Path, default=Path("legged_gym/logs"))
    parser.add_argument("--out_dir", type=Path, default=Path("project_outputs/train_curves_csv"))
    args = parser.parse_args()
    return export_scalars(args.log_root, args.out_dir)


if __name__ == "__main__":
    raise SystemExit(main())
