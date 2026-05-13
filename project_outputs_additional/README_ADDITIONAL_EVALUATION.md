# Additional Evaluation

## 1. Purpose

This directory contains supplemental evaluations for comparing the trained RL policy with the convex MPC baseline outside the fixed core evaluation set. The added tasks focus on flat-ground speed scaling, energy efficiency on completed tasks, and sim-to-sim robustness under runtime model perturbations.

The core evaluation outputs remain in `project_outputs_RL/` and `project_outputs_baseline/`. They are not modified by the additional pipeline.

## 2. Fairness And Relevance

The additional tasks keep the same MuJoCo robot model family, flat scene XML, command format, duration, trial count, and CSV schema used by the existing evaluation loggers. Both methods are run through their own controllers, but the logged quantities are shared: command, base state, roll/pitch/yaw, joint state, joint torque, foot contacts, fall flag, success flag, distance, robot mass, and simulation timestep.

The speed sweep is relevant because high commanded velocity can expose stability and controller feasibility limits. The energy summary is relevant only for successful full-length trials, so failed partial runs are not treated as completed-task CoT. The robustness tests are relevant because RL training used domain randomization, while the convex MPC baseline is a deterministic model-based controller.

## 3. Generated Outputs

The one-command runner writes only under `project_outputs_additional/`:

- `RL/speed_sweep/eval_logs/` and `baseline/speed_sweep/eval_logs/`
- `RL/speed_sweep/eval_results/` and `baseline/speed_sweep/eval_results/`
- `RL/robustness/eval_logs/` and `baseline/robustness/eval_logs/`
- `RL/robustness/eval_results/` and `baseline/robustness/eval_results/`
- `comparison/comparison_speed_sweep.csv`
- `comparison/comparison_speed_sweep_success_rate.png`
- `comparison/comparison_speed_sweep_cot.png`
- `comparison/comparison_speed_sweep_velocity_error.png`
- `comparison/comparison_energy_efficiency.csv`
- `comparison/comparison_energy_efficiency.png`
- `comparison/comparison_robustness.csv`
- `comparison/comparison_robustness_success_rate.png`
- `comparison/comparison_robustness_cot.png`
- `comparison/comparison_robustness_velocity_error.png`

If a plot cannot be generated because there is no finite data for that metric, the comparison script skips that plot and records the missing file in `additional_run_status.md`.

## 4. Rerun Speed Sweep

Run the full pipeline:

```bash
scripts/run_additional_evaluation.sh
```

To rerun existing additional trials instead of skipping completed outputs:

```bash
FORCE=1 scripts/run_additional_evaluation.sh
```

The speed sweep conditions are `flat_0p4`, `flat_0p6`, `flat_0p8`, `flat_1p0`, and `flat_1p2`, with 10 trials per method by default. Override the default trial count or duration with `TRIALS=<n>` and `DURATION=<seconds>`.

## 5. Rerun Robustness Tests

The same runner executes robustness tests after the speed sweep:

```bash
scripts/run_additional_evaluation.sh
```

The robustness conditions are:

- `friction_low`: the MuJoCo ground geom named `floor` has sliding friction set to `0.5` at model load time.
- `payload_high`: the `base_link` body mass and diagonal inertia are multiplied by `1.2` at model load time.
- `motor_weak`: applied joint torques are multiplied by `0.9`; the logged torque is the scaled applied torque.

The XML files in `mujoco_test/data/go2/` are not edited.

## 6. Analyze Outputs

Per-method analysis uses the existing schema-compatible analyzer:

```bash
python tools/analyze_eval_logs.py --eval_root project_outputs_additional/RL/speed_sweep/eval_logs --out_dir project_outputs_additional/RL/speed_sweep/eval_results
python tools/analyze_eval_logs.py --eval_root project_outputs_additional/baseline/speed_sweep/eval_logs --out_dir project_outputs_additional/baseline/speed_sweep/eval_results
python tools/analyze_eval_logs.py --eval_root project_outputs_additional/RL/robustness/eval_logs --out_dir project_outputs_additional/RL/robustness/eval_results
python tools/analyze_eval_logs.py --eval_root project_outputs_additional/baseline/robustness/eval_logs --out_dir project_outputs_additional/baseline/robustness/eval_results
```

Then regenerate comparison tables and plots:

```bash
python tools/compare_additional_evaluations.py --additional_root project_outputs_additional
```

## 7. Reading Comparison Plots

Success-rate plots show the fraction of trials with no logged fall and a successful final state. Velocity-error plots show mean RMS forward velocity tracking error. CoT plots show mean cost of transport using successful full-length trials only when per-trial metrics are available.

Missing bars mean that the corresponding metric is not finite for that method and condition. For example, if every trial failed before completing the task, CoT is intentionally left blank rather than filled with a partial-run value.

## 8. Known Failures And Limitations

The authoritative record for the latest run is `additional_run_status.md`. It lists generated CSV counts, `.error.txt` counts, failed or incomplete conditions, plot generation status, and next commands.

Latest recorded run:

- All 160 expected additional CSV files were generated.
- Baseline speed sweep at `flat_0p8`, `flat_1p0`, and `flat_1p2` produced 10/10 controller-solve failures per condition. Partial CSV files and same-stem `.error.txt` files were retained.
- RL speed sweep, RL robustness, and baseline robustness produced no `.error.txt` files in the latest run.

Limitations:

- The payload perturbation scales `base_link` mass and diagonal inertia; it does not add a separate physical payload body.
- The friction perturbation changes the flat-ground `floor` sliding friction at runtime; it does not rewrite XML.
- Controller infeasibility or import/runtime errors are represented by `.error.txt` sidecars and partial CSV when available.
- The repeated trials are deterministic for the current loggers because the trial index seeds the rollout but no extra noise is injected by the additional runner.

## 9. Do-Not-Fabricate-Data Policy

This evaluation does not create synthetic CSV files, success rates, CoT values, or fake plots. Failed trials are retained. Missing data remains missing. Partial trials are excluded from completed-task CoT in the comparison tables. Core evaluation results are not overwritten.
