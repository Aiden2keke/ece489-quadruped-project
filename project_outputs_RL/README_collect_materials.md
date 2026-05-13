# ME446/ECE489 RL Material Collection

This repo now has scripts for config export, TensorBoard scalar export, curve plotting, MuJoCo fixed-condition evaluation, and metric analysis. These scripts do not fabricate reward curves or evaluation data. If logs are missing, they warn and skip outputs.

## Manual Screenshots and Videos

Mid-term:
- Isaac Gym / legged_gym Go2 loaded successfully.
- Multiple parallel training environments.
- `train.py` running in the terminal.
- TensorBoard curve or the regenerated reward curve from `project_outputs/figures/`.
- Preliminary rollout/play screenshot.

Final:
- Flat walking video.
- Challenging terrain walking video.
- Lateral push recovery video.
- Failure case video or screenshot if any trial falls.
- Final plots from `project_outputs/eval_results/`.
- `summary_metrics.csv` table from `project_outputs/eval_results/`.

## Training and Play Commands

Run from repository root:

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/train.py --task=go2 --headless
```

Flat play from the existing checkpoint:

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/play.py --task=go2 --load_run=TS_re3 --checkpoint=9000
```

Challenging terrain training:

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/train.py --task=go2_rough --headless
```

Challenging terrain play:

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/play.py --task=go2_rough --load_run=<RUN_DIR_NAME> --checkpoint=<ITERATION>
```

Aliases: `go2` and `go2_flat` use flat terrain. `go2_rough` and `go2_challenging` use legged_gym trimesh rough terrain.

## Config Summary

```bash
python tools/export_rl_config_summary.py
```

Outputs:
- `project_outputs/rl_config_summary.md`
- `project_outputs/rl_config_summary.csv`

## Training Curves

Export TensorBoard scalars:

```bash
python tools/export_tensorboard_scalars.py --log_root legged_gym/logs --out_dir project_outputs/train_curves_csv
```

Plot curves from CSV:

```bash
python tools/plot_training_curves.py --csv_dir project_outputs/train_curves_csv --out_dir project_outputs/figures
```

Expected figures include `training_mean_reward.png`, `episode_length.png`, `reward_components.png`, `ppo_losses.png`, and `terrain_curriculum.png` when the corresponding scalar tags exist.

## Evaluation

Single flat trial:

```bash
python mujoco_test/test_script/eval_go2_policy_logger.py \
  --scene flat \
  --cmd_vx 0.4 \
  --duration 20 \
  --trial 0 \
  --out_dir project_outputs/eval_logs/flat_0p4 \
  --checkpoint legged_gym/logs/rough_go2/TS_re3/model_9000.pt \
  --headless
```

Single terrain trial with lateral push:

```bash
python mujoco_test/test_script/eval_go2_policy_logger.py \
  --scene terrain \
  --cmd_vx 0.4 \
  --duration 20 \
  --trial 0 \
  --out_dir project_outputs/eval_logs/push_terrain_0p4 \
  --checkpoint legged_gym/logs/rough_go2/TS_re3/model_9000.pt \
  --push_force 40 \
  --push_start 5.0 \
  --push_duration 0.1 \
  --headless
```

Batch evaluation:

```bash
scripts/run_me446_rl_evaluation.sh
```

By default the batch script writes to `project_outputs/eval_logs/`. To use another absolute directory:

```bash
OUT_ROOT=/home/yd/ece489/project/rsl_rl_teacher_student/project_outputs/eval_logs scripts/run_me446_rl_evaluation.sh
```

Analyze evaluation logs:

```bash
python tools/analyze_rl_eval_logs.py --eval_root project_outputs/eval_logs --out_dir project_outputs/eval_results
```

Outputs:
- `trial_metrics.csv`
- `summary_metrics.csv`
- `rms_velocity_error.png`
- `rms_roll_pitch.png`
- `cot_comparison.png`
- `push_recovery_success.png`
- Example tracking/attitude/push plots
- `contact_gait_diagram.png` if contact state columns exist
