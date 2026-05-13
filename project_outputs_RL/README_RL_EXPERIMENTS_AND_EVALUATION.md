# RL Locomotion Experiment and Evaluation Guide

## 1. Project Overview

This document describes the RL side of the ME446 / ECE489 Quadruped Locomotion project.

- Robot: Unitree Go2
- Training framework: `legged_gym` + `rsl_rl` + PPO
- Training type: teacher-student locomotion policy
- Training schedule used for the final RL policy: 7500 iterations teacher / teacher-student stage, followed by 1500 iterations student reinforcing
- Deployment and fixed-condition evaluation: MuJoCo
- Main purpose: generate reproducible metrics and plots for fair comparison between RL and model-based locomotion methods

The final evaluated RL controller uses the exported MuJoCo actor and proprioceptive encoder:

```bash
mujoco_test/model/rsl_rl_teacher_student/actor/actor_oracle_ece489.pth
mujoco_test/model/rsl_rl_teacher_student/proprio_encoder/proprio_oracle_ece489.pth
```

## 2. Repository Structure

Key files and directories:

- `legged_gym/legged_gym/scripts/train.py`: Isaac Gym / legged_gym PPO training entry point
- `legged_gym/legged_gym/scripts/play.py`: Isaac Gym visualization / rollout entry point
- `legged_gym/legged_gym/envs/go2/go2_config.py`: Go2 RL task, reward, terrain, and domain randomization config
- `legged_gym/legged_gym/envs/base/legged_robot.py`: base legged robot environment and curriculum logic
- `mujoco_test/test_script/eval_go2_policy_logger.py`: fixed-condition MuJoCo RL evaluation logger
- `scripts/run_me446_rl_evaluation.sh`: batch script for the six RL evaluation conditions
- `tools/export_tensorboard_scalars.py`: TensorBoard scalar export tool
- `tools/plot_training_curves.py`: report plot generator for training curves
- `tools/analyze_rl_eval_logs.py`: evaluation CSV analysis and plot generator
- `tools/analyze_eval_logs.py`: method-agnostic wrapper around the same analysis logic
- `tools/capture_mujoco_eval_scenes.py`: MuJoCo scene screenshot generator
- `project_outputs/`: generated logs, figures, evaluation results, screenshots, and report materials

## 3. Training Commands

Run from the repository root:

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
```

Rough / challenging terrain training:

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/train.py \
  --task=go2_rough \
  --headless \
  --experiment_name=rough_go2_challenging \
  --run_name=ece489
```

Task names:

- `go2` and `go2_flat`: flat terrain config
- `go2_rough` and `go2_challenging`: rough / challenging terrain config

The final run used for this report is:

```bash
legged_gym/logs/rough_go2_challenging/ece489/model_9000.pt
```

The checkpoint and TensorBoard packages were also copied into `project_outputs/` for report collection.

## 4. Teacher-Student Training Stages

Stage 1: iterations 0 to 7500, teacher / teacher-student training.

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/train.py \
  --task=go2_rough \
  --headless \
  --max_iterations=7500 \
  --experiment_name=rough_go2_challenging \
  --run_name=ece489
```

Stage 2: iterations 7500 to 9000, student reinforcing. The command below resumes from checkpoint 7500 and runs 1500 more iterations:

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/train.py \
  --task=go2_rough \
  --headless \
  --max_iterations=1500 \
  --student_reinforcing \
  --resume \
  --experiment_name=rough_go2_challenging \
  --run_name=ece489 \
  --load_run=ece489 \
  --checkpoint=7500
```

If launching from the parent directory that contains an `ece489/` copy of this repository, the equivalent form is:

```bash
python ece489/legged_gym/legged_gym/scripts/train.py \
  --task=go2_rough \
  --headless \
  --max_iterations=1500 \
  --student_reinforcing \
  --resume \
  --experiment_name=rough_go2_challenging \
  --run_name=ece489 \
  --checkpoint=7500
```

TensorBoard event packages used for this report:

- `project_outputs/ece489_7500.tgz`: first 7500 iterations
- `project_outputs/ece489_SR_1500(9000).tgz`: final 1500 student-reinforcing iterations

## 5. TensorBoard Log Export and Plotting

Extract the TensorBoard event packages:

```bash
mkdir -p project_outputs/tensorboard_raw/ece489_7500
mkdir -p project_outputs/tensorboard_raw/ece489_SR_1500

tar -xf project_outputs/ece489_7500.tgz \
  -C project_outputs/tensorboard_raw/ece489_7500

tar -xf 'project_outputs/ece489_SR_1500(9000).tgz' \
  -C project_outputs/tensorboard_raw/ece489_SR_1500
```

Export scalars to CSV:

```bash
python tools/export_tensorboard_scalars.py \
  --log_root project_outputs/tensorboard_raw \
  --out_dir project_outputs/train_curves_csv
```

Replot report figures from CSV:

```bash
python tools/plot_training_curves.py \
  --csv_dir project_outputs/train_curves_csv \
  --out_dir project_outputs/figures
```

Current generated training figures:

- `project_outputs/figures/training_mean_reward.png`
- `project_outputs/figures/episode_length.png`
- `project_outputs/figures/reward_components.png`
- `project_outputs/figures/ppo_losses.png`
- `project_outputs/figures/terrain_curriculum.png`

## 6. Play / Visualization

Isaac Gym play command for the rough terrain checkpoint:

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/play.py \
  --task=go2_rough \
  --experiment_name=rough_go2_challenging \
  --load_run=ece489 \
  --checkpoint=9000
```

The `play.py` script sets `resume=True` internally, so `--load_run=ece489` and `--checkpoint=9000` select `model_9000.pt`.

Record or save the following report materials manually from play / viewer sessions:

- flat walking video
- rough / challenging terrain walking video
- failure case video or screenshot if the policy falls
- Isaac Gym environment setup screenshots
- preliminary rollout plot, such as `project_outputs/play_plot.png`

## 7. MuJoCo Evaluation Conditions

The batch evaluation runs six conditions, each with 10 trials and duration 20 s. The flat scene is:

```bash
mujoco_test/data/go2/scene.xml
```

The terrain scene is:

```bash
mujoco_test/data/go2/scene_terrain.xml
```

1. `flat_0p4`, 10 trials
   - Scene: flat ground
   - MuJoCo XML: `mujoco_test/data/go2/scene.xml`
   - Command: `vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
   - Push: none
   - Duration: 20 s
   - Trials: 10

2. `flat_0p8`, 10 trials
   - Scene: flat ground
   - MuJoCo XML: `mujoco_test/data/go2/scene.xml`
   - Command: `vx = 0.8 m/s`, `vy = 0`, `yaw rate = 0`
   - Push: none
   - Duration: 20 s
   - Trials: 10

3. `terrain_0p4`, 10 trials
   - Scene: challenging terrain
   - MuJoCo XML: `mujoco_test/data/go2/scene_terrain.xml`
   - Command: `vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
   - Push: none
   - Duration: 20 s
   - Trials: 10

4. `terrain_0p8`, 10 trials
   - Scene: challenging terrain
   - MuJoCo XML: `mujoco_test/data/go2/scene_terrain.xml`
   - Command: `vx = 0.8 m/s`, `vy = 0`, `yaw rate = 0`
   - Push: none
   - Duration: 20 s
   - Trials: 10

5. `push_flat_0p4`, 10 trials
   - Scene: flat ground
   - MuJoCo XML: `mujoco_test/data/go2/scene.xml`
   - Command: `vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
   - External disturbance: 40 N lateral push
   - Push direction: +Y force applied to `base_link` through `data.xfrc_applied[base_body_id, 1]`
   - Push duration: 0.1 s
   - Push start time: 5.0 s
   - Duration: 20 s
   - Trials: 10

6. `push_terrain_0p4`, 10 trials
   - Scene: challenging terrain
   - MuJoCo XML: `mujoco_test/data/go2/scene_terrain.xml`
   - Command: `vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
   - External disturbance: 40 N lateral push
   - Push direction: same as `push_flat_0p4`, +Y force on `base_link`
   - Push duration: 0.1 s
   - Push start time: 5.0 s
   - Duration: 20 s
   - Trials: 10

## 8. Running RL MuJoCo Evaluation

Current verified RL evaluation command:

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student

POLICY_PATH=${PWD}/mujoco_test/model/rsl_rl_teacher_student/actor/actor_oracle_ece489.pth \
ENCODER_PATH=${PWD}/mujoco_test/model/rsl_rl_teacher_student/proprio_encoder/proprio_oracle_ece489.pth \
DURATION=20 \
OUT_ROOT=${PWD}/project_outputs/eval_logs \
scripts/run_me446_rl_evaluation.sh
```

Behavior of `scripts/run_me446_rl_evaluation.sh`:

- If `POLICY_PATH` is set, the evaluation uses actor + proprio encoder mode.
- If `POLICY_PATH` is not set, the evaluation uses checkpoint mode through `--checkpoint "$CHECKPOINT"`.
- Output directory: `project_outputs/eval_logs/`

The current RL evaluation set contains 60 CSV files:

- `flat_0p4`: 10 trials
- `flat_0p8`: 10 trials
- `terrain_0p4`: 10 trials
- `terrain_0p8`: 10 trials
- `push_flat_0p4`: 10 trials
- `push_terrain_0p4`: 10 trials

## 9. Evaluation CSV Schema

Every trial CSV must contain these fields so it can be analyzed consistently:

- `time`: simulation time in seconds
- `cmd_vx`, `cmd_vy`, `cmd_yaw`: commanded body velocity and yaw rate
- `base_x`, `base_y`, `base_z`: floating base position
- `com_x`, `com_y`, `com_z`: robot center of mass position
- `base_vx`, `base_vy`, `base_vz`: base linear velocity in the body frame
- `roll`, `pitch`, `yaw`: base orientation in roll-pitch-yaw form
- `joint_pos_1` to `joint_pos_12`: joint positions
- `joint_vel_1` to `joint_vel_12`: joint velocities
- `joint_torque_1` to `joint_torque_12`: commanded/applied joint torques used for logging CoT
- `foot_contact_FL`, `foot_contact_FR`, `foot_contact_RL`, `foot_contact_RR`: binary contact flags
- `foot_force_FL`, `foot_force_FR`, `foot_force_RL`, `foot_force_RR`: scalar contact force norms per foot
- `external_push_flag`: 1 during the push interval, 0 otherwise
- `fall_flag`: 1 if the robot violates the fall criterion at that time step
- `success_flag`: final trial success flag written to all rows
- `distance_traveled`: horizontal distance traveled from the start pose
- `robot_mass`: robot mass used for CoT normalization
- `sim_dt`: MuJoCo simulation time step

The RL logger does not fabricate full 3D foot force vectors. It logs one scalar contact force norm per foot.

## 10. Metrics

The analysis script computes these metrics per trial and then reports mean +/- std per condition.

RMS velocity tracking error:

```text
sqrt(mean((cmd_vx - base_vx)^2))
```

RMS roll:

```text
sqrt(mean(roll^2))
```

RMS pitch:

```text
sqrt(mean(pitch^2))
```

Cost of Transport:

```text
CoT = sum(abs(tau_i * qdot_i) * dt) / (m * g * distance)
```

Success rule used by `tools/analyze_rl_eval_logs.py`:

- `fall_flag` must be 0 for the whole trial.
- Final `base_z` must be at least 0.18 m.
- Final `abs(roll)` and `abs(pitch)` must each be no greater than 0.8 rad.

The MuJoCo RL logger uses the same fall thresholds by default:

- `--min_base_height=0.18`
- `--max_abs_roll_pitch=0.8`

Push recovery success rule used by `tools/analyze_rl_eval_logs.py`:

- The trial must contain an external push interval.
- After the push ends, `fall_flag` must remain 0.
- In the recovery window, normally from 2 s after the push end until the rollout end, at least 50 percent of samples must satisfy:
  - `abs(roll) < 0.4`
  - `abs(pitch) < 0.4`
  - `abs(cmd_vx - base_vx) < 0.35 m/s`

## 11. Analyzing Evaluation Logs

Analyze RL evaluation logs:

```bash
python tools/analyze_rl_eval_logs.py \
  --eval_root project_outputs/eval_logs \
  --out_dir project_outputs/eval_results
```

Equivalent method-agnostic wrapper:

```bash
python tools/analyze_eval_logs.py \
  --eval_root project_outputs/eval_logs \
  --out_dir project_outputs/eval_results
```

Current generated outputs:

- `project_outputs/eval_results/summary_metrics.csv`
- `project_outputs/eval_results/trial_metrics.csv`
- `project_outputs/eval_results/rms_velocity_error.png`
- `project_outputs/eval_results/rms_roll_pitch.png`
- `project_outputs/eval_results/cot_comparison.png`
- `project_outputs/eval_results/push_recovery_success.png`
- `project_outputs/eval_results/example_velocity_tracking_flat.png`
- `project_outputs/eval_results/example_velocity_tracking_terrain.png`
- `project_outputs/eval_results/example_roll_pitch_flat.png`
- `project_outputs/eval_results/example_roll_pitch_terrain.png`
- `project_outputs/eval_results/example_push_recovery.png`
- `project_outputs/eval_results/contact_gait_diagram.png`

## 12. Interface for Model-Based Comparison

For a fair comparison, the model-based method must match the RL evaluation setup:

- Use the same MuJoCo robot model.
- Use the same flat scene: `mujoco_test/data/go2/scene.xml`.
- Use the same terrain scene: `mujoco_test/data/go2/scene_terrain.xml`.
- Use the same initial robot pose and joint pose as the RL logger.
- Use the same command set:
  - `flat_0p4`
  - `flat_0p8`
  - `terrain_0p4`
  - `terrain_0p8`
  - `push_flat_0p4`
  - `push_terrain_0p4`
- Use the same duration: 20 s.
- Use the same lateral push settings:
  - 40 N
  - 0.1 s
  - start at 5.0 s
  - +Y force applied to `base_link`
- Output the same CSV schema listed in Section 9.
- Keep failed trials. Do not delete failures before analysis.

Recommended output directory for model-based logs:

```bash
project_outputs/eval_logs_model_based/
```

The condition directory names must match the RL names:

```text
flat_0p4
flat_0p8
terrain_0p4
terrain_0p8
push_flat_0p4
push_terrain_0p4
```

Model-based logger interface template:

```bash
python mujoco_test/test_script/eval_go2_model_based_logger.py \
  --scene flat \
  --cmd_vx 0.4 \
  --cmd_vy 0 \
  --cmd_yaw 0 \
  --duration 20 \
  --trial 0 \
  --out_dir project_outputs/eval_logs_model_based/flat_0p4 \
  --headless
```

For push conditions, the model-based logger should expose the same options:

```bash
python mujoco_test/test_script/eval_go2_model_based_logger.py \
  --scene flat \
  --cmd_vx 0.4 \
  --cmd_vy 0 \
  --cmd_yaw 0 \
  --duration 20 \
  --trial 0 \
  --out_dir project_outputs/eval_logs_model_based/push_flat_0p4 \
  --push_force 40 \
  --push_start 5.0 \
  --push_duration 0.1 \
  --headless
```

The model-based logger is not provided here because the controller implementation belongs to the model-based project part. The required interface is the command-line contract and CSV schema above.

## 13. Optional: Make Analysis Support Multiple Methods

`tools/analyze_rl_eval_logs.py` is method-agnostic as long as the input CSV schema is the same. The filename says `rl` because it was first written for the RL controller.

Analyze model-based logs with the same script:

```bash
python tools/analyze_rl_eval_logs.py \
  --eval_root project_outputs/eval_logs_model_based \
  --out_dir project_outputs/eval_results_model_based
```

Or use the generic wrapper:

```bash
python tools/analyze_eval_logs.py \
  --eval_root project_outputs/eval_logs_model_based \
  --out_dir project_outputs/eval_results_model_based
```

To compare RL and model-based in the final report, place the two summary files side by side:

```text
project_outputs/eval_results/summary_metrics.csv
project_outputs/eval_results_model_based/summary_metrics.csv
```

## 14. Generate MuJoCo Scene Screenshots

Generate screenshots from the same MuJoCo XML files used for evaluation:

```bash
python tools/capture_mujoco_eval_scenes.py \
  --gl_backend egl \
  --out_dir project_outputs/scene_screenshots
```

If `egl` is not available, try:

```bash
python tools/capture_mujoco_eval_scenes.py \
  --gl_backend osmesa \
  --out_dir project_outputs/scene_screenshots
```

Generated files:

- `project_outputs/scene_screenshots/flat_scene.png`
- `project_outputs/scene_screenshots/terrain_scene.png`
- `project_outputs/scene_screenshots/push_flat_0p4_scene.png`
- `project_outputs/scene_screenshots/push_terrain_0p4_scene.png`

The push screenshots use the real flat or terrain XML render and overlay a labeled arrow for the 40 N, 0.1 s +Y lateral push. They are scene illustrations, not evaluation data.

If offscreen rendering is unavailable on a machine, manually open the XML in MuJoCo viewer and save screenshots:

```bash
python -m mujoco.viewer --mjcf mujoco_test/data/go2/scene.xml
python -m mujoco.viewer --mjcf mujoco_test/data/go2/scene_terrain.xml
```

## 15. Scene Screenshots

Current scene screenshots:

- `flat_scene.png`: flat ground scene for `flat_0p4`, `flat_0p8`, and `push_flat_0p4`
- `terrain_scene.png`: challenging terrain scene for `terrain_0p4`, `terrain_0p8`, and `push_terrain_0p4`
- `push_flat_0p4_scene.png`: flat scene with annotated lateral push setting
- `push_terrain_0p4_scene.png`: terrain scene with annotated lateral push setting

## 16. Current Evaluation Result Notes

The current RL summary is stored at:

```bash
project_outputs/eval_results/summary_metrics.csv
```

Observed success rates from the current 60-trial RL evaluation:

- `flat_0p4`: success rate 1.0
- `flat_0p8`: success rate 1.0
- `terrain_0p4`: success rate 1.0
- `terrain_0p8`: success rate 0.0
- `push_flat_0p4`: no-fall success rate 1.0, push recovery success rate 0.0
- `push_terrain_0p4`: no-fall success rate 1.0, push recovery success rate 0.0

The `terrain_0p8` condition is currently a failure case according to the analysis script's success rule. Push trials did not satisfy the stricter push recovery criterion, even though the no-fall success flag was 1.0 for both push conditions.

Failure cases should be retained for report discussion. Do not remove failed trials from the summary.

## 17. Do Not Fabricate Data

- Do not generate fake evaluation CSV files.
- Do not generate fake metrics.
- Do not delete failed trials before analysis.
- Do not replace failed results with successful examples.
- If a field is unavailable for a controller, document it and leave the value blank or `nan` rather than inserting a fake constant.
- The current RL data and figures are generated from existing logs and evaluation CSVs under `project_outputs/`.
- Model-based results are not available yet unless the model-based team generates CSVs with the schema in Section 9.
