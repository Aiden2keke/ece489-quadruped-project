#!/usr/bin/env python3
"""Analyze baseline CSV logs using the shared RL-compatible metrics script."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

from analyze_eval_logs import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
