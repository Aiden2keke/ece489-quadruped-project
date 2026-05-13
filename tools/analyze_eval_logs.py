#!/usr/bin/env python3
"""Method-agnostic wrapper for fixed-condition locomotion evaluation analysis.

This wrapper intentionally reuses tools/analyze_rl_eval_logs.py. The analysis is
valid for RL and model-based controllers as long as the logged CSV schema is the
same.
"""

from __future__ import annotations

from analyze_rl_eval_logs import main


if __name__ == "__main__":
    raise SystemExit(main())
