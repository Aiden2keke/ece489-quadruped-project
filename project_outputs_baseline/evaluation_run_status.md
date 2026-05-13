# Baseline Evaluation Run Status

Generated: 2026-05-13

## Completed Outputs

- Final eval log root: `project_outputs_baseline/eval_logs`
- Archived failed adapter attempt: `project_outputs_baseline/eval_logs_failed_rl_scene_init`
- Batch run log: `project_outputs_baseline/baseline_batch_run.log`
- Analysis output root: `project_outputs_baseline/eval_results`
- Optional RL-vs-baseline output root: `project_outputs_comparison`

## Final Batch Evaluation

Command used:

```bash
PYTHON_BIN=/home/yd/anaconda3/envs/go2-convex-mpc/bin/python baseline_tools/run_baseline_evaluation.sh
```

Initialization:

- `INIT_MODE=baseline_demo`
- RL flat/terrain XML files were still used.
- Base pose and joint pose came from the upstream baseline demo because
  `INIT_MODE=rl_scene` caused early QP failure.

CSV count:

- Total CSV files: 60
- Total sidecar `.error.txt` files: 20

Per-condition status:

| Condition | CSV files | Error sidecars | Notes |
| --- | ---: | ---: | --- |
| `flat_0p4` | 10 | 0 | Completed 20 s |
| `flat_0p8` | 10 | 10 | Controller QP failed at about 1.5 s; partial CSV retained |
| `terrain_0p4` | 10 | 0 | Completed 20 s |
| `terrain_0p8` | 10 | 10 | Controller QP failed at about 1.5 s; partial CSV retained |
| `push_flat_0p4` | 10 | 0 | Completed 20 s |
| `push_terrain_0p4` | 10 | 0 | Completed 20 s |

The 0.8 m/s failures are not fabricated or removed.  They are reflected in
`summary_metrics.csv` as `success_mean = 0` for `flat_0p8` and `terrain_0p8`.

## Archived RL-Scene-Init Attempt

Command mode:

- `INIT_MODE=rl_scene`

Archived output:

- `project_outputs_baseline/eval_logs_failed_rl_scene_init`

Status:

- Total CSV files: 60
- Total sidecar `.error.txt` files: 60
- All trials hit early OSQP failure at about 0.14-0.18 s.

This archived attempt is retained for traceability and is not used as the final
baseline summary input.

## Analysis

Command used:

```bash
python tools/analyze_eval_logs.py \
  --eval_root project_outputs_baseline/eval_logs \
  --out_dir project_outputs_baseline/eval_results
```

Generated:

- `trial_metrics.csv`
- `summary_metrics.csv`
- all requested baseline plots, including `contact_gait_diagram.png`

The shared analyzer was updated to respect the logger's `success_flag` in
addition to recomputing posture/fall checks.  This prevents controller-error
partial CSV files from being misclassified as successful merely because the
robot had not yet fallen.

## Comparison

Command used:

```bash
python tools/compare_rl_baseline_metrics.py \
  --rl_summary project_outputs_RL/eval_results/summary_metrics.csv \
  --baseline_summary project_outputs_baseline/eval_results/summary_metrics.csv \
  --out_dir project_outputs_comparison
```

Generated:

- `comparison_summary.csv`
- `comparison_velocity_error.png`
- `comparison_roll_pitch.png`
- `comparison_cot.png`
- `comparison_push_success.png`
