# RL Locomotion Experiment and Evaluation Guide

本文档是 ME446 / ECE489 四足机器人项目中 RL 部分的实验、可视化、数据导出、MuJoCo 评估和 model-based 对比接口说明。

## 1. Project Overview

本项目的 RL 部分目标是训练并评估一个 Unitree Go2 的 learning-based locomotion policy，并生成可与组员 model-based 方法公平比较的指标。

- 机器人平台：Unitree Go2
- 训练框架：`legged_gym` + `rsl_rl` + PPO
- 训练方式：teacher-student locomotion policy
- 本次最终训练流程：前 7500 iterations 为 teacher / teacher-student 阶段，后 1500 iterations 为 student reinforcing 阶段
- 部署和固定条件评估：MuJoCo fixed-condition evaluation
- 主要用途：为 RL 和 model-based locomotion 生成同一套可比较的 metrics、plots、summary tables 和 failure cases

当前 MuJoCo evaluation 使用的是已经导出的 actor 和 proprioceptive encoder：

```bash
mujoco_test/model/rsl_rl_teacher_student/actor/actor_oracle_ece489.pth
mujoco_test/model/rsl_rl_teacher_student/proprio_encoder/proprio_oracle_ece489.pth
```

## 2. Repository Structure

关键文件和目录如下：

- `legged_gym/legged_gym/scripts/train.py`：Isaac Gym / legged_gym PPO 训练入口
- `legged_gym/legged_gym/scripts/play.py`：Isaac Gym policy rollout / visualization 入口
- `legged_gym/legged_gym/envs/go2/go2_config.py`：Go2 RL task、reward、terrain、domain randomization 配置
- `legged_gym/legged_gym/envs/base/legged_robot.py`：基础四足环境、reward、curriculum 和 reset 逻辑
- `mujoco_test/test_script/eval_go2_policy_logger.py`：固定 commanded speed 的 MuJoCo RL evaluation logger
- `scripts/run_me446_rl_evaluation.sh`：批量运行 6 个 RL evaluation condition 的脚本
- `tools/export_tensorboard_scalars.py`：TensorBoard scalar 导出工具
- `tools/plot_training_curves.py`：从 CSV 重画 training curves 的工具
- `tools/analyze_rl_eval_logs.py`：evaluation CSV 分析和 plotting 脚本
- `tools/analyze_eval_logs.py`：通用 analysis wrapper，可用于 RL 或 model-based，只要 CSV schema 一致
- `tools/capture_mujoco_eval_scenes.py`：MuJoCo flat / terrain / push 场景截图工具
- `project_outputs/`：训练日志、figures、evaluation logs、summary tables、screenshots 和报告素材输出目录

## 3. Training Commands

从 repo 根目录运行：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
```

rough / challenging terrain training 命令：

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/train.py \
  --task=go2_rough \
  --headless \
  --experiment_name=rough_go2_challenging \
  --run_name=ece489
```

task 名称说明：

- `go2` 和 `go2_flat`：flat terrain config
- `go2_rough` 和 `go2_challenging`：rough / challenging terrain config

本次 final report 使用的 checkpoint：

```bash
legged_gym/logs/rough_go2_challenging/ece489/model_9000.pt
```

该 checkpoint 和 TensorBoard packages 也已经复制到 `project_outputs/`，方便报告材料整理。

## 4. Teacher-Student Training Stages

Stage 1：第 0 到 7500 iterations，teacher / teacher-student training。

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/train.py \
  --task=go2_rough \
  --headless \
  --max_iterations=7500 \
  --experiment_name=rough_go2_challenging \
  --run_name=ece489
```

Stage 2：第 7500 到 9000 iterations，student reinforcing。下面命令从 checkpoint 7500 resume，并继续训练 1500 iterations：

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

如果从包含 `ece489/` repo 副本的上级目录运行，也可以使用下面这种形式：

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

本次使用的 TensorBoard event packages：

- `project_outputs/ece489_7500.tgz`：前 7500 iterations
- `project_outputs/ece489_SR_1500(9000).tgz`：后 1500 iterations student reinforcing

## 5. TensorBoard Log Export and Plotting

先解压 TensorBoard event packages：

```bash
mkdir -p project_outputs/tensorboard_raw/ece489_7500
mkdir -p project_outputs/tensorboard_raw/ece489_SR_1500

tar -xf project_outputs/ece489_7500.tgz \
  -C project_outputs/tensorboard_raw/ece489_7500

tar -xf 'project_outputs/ece489_SR_1500(9000).tgz' \
  -C project_outputs/tensorboard_raw/ece489_SR_1500
```

导出所有 scalar tags 到 CSV：

```bash
python tools/export_tensorboard_scalars.py \
  --log_root project_outputs/tensorboard_raw \
  --out_dir project_outputs/train_curves_csv
```

从 CSV 重新绘制报告图：

```bash
python tools/plot_training_curves.py \
  --csv_dir project_outputs/train_curves_csv \
  --out_dir project_outputs/figures
```

当前已经生成的 training figures：

- `project_outputs/figures/training_mean_reward.png`
- `project_outputs/figures/episode_length.png`
- `project_outputs/figures/reward_components.png`
- `project_outputs/figures/ppo_losses.png`
- `project_outputs/figures/terrain_curriculum.png`

注意：这些图是从 TensorBoard scalar CSV 重新绘制的，不是伪造曲线，也不是 TensorBoard 截图。

## 6. Play / Visualization

Isaac Gym 中 rough terrain checkpoint 的 play 命令：

```bash
PYTHONPATH=legged_gym:rsl_rl python legged_gym/legged_gym/scripts/play.py \
  --task=go2_rough \
  --experiment_name=rough_go2_challenging \
  --load_run=ece489 \
  --checkpoint=9000
```

当前 `play.py` 内部会设置 `resume=True`，因此 `--load_run=ece489` 和 `--checkpoint=9000` 会加载对应的 `model_9000.pt`。

建议手动录制或保存以下素材：

- flat walking video
- rough / challenging terrain walking video
- 如果 policy 摔倒，保留 failure case video 或 screenshot
- Isaac Gym environment setup screenshots
- preliminary rollout plot，例如 `project_outputs/play_plot.png`

## 7. MuJoCo Evaluation Conditions

批量 evaluation 一共运行 6 个 condition，每个 condition 10 trials，每个 trial duration 为 20 s。

flat scene 使用：

```bash
mujoco_test/data/go2/scene.xml
```

terrain scene 使用：

```bash
mujoco_test/data/go2/scene_terrain.xml
```

### 7.1 `flat_0p4`, 10 trials

- Scene：flat ground
- MuJoCo XML：`mujoco_test/data/go2/scene.xml`
- Command：`vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
- Push：none
- Duration：20 s
- Trials：10

### 7.2 `flat_0p8`, 10 trials

- Scene：flat ground
- MuJoCo XML：`mujoco_test/data/go2/scene.xml`
- Command：`vx = 0.8 m/s`, `vy = 0`, `yaw rate = 0`
- Push：none
- Duration：20 s
- Trials：10

### 7.3 `terrain_0p4`, 10 trials

- Scene：challenging terrain
- MuJoCo XML：`mujoco_test/data/go2/scene_terrain.xml`
- Command：`vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
- Push：none
- Duration：20 s
- Trials：10

### 7.4 `terrain_0p8`, 10 trials

- Scene：challenging terrain
- MuJoCo XML：`mujoco_test/data/go2/scene_terrain.xml`
- Command：`vx = 0.8 m/s`, `vy = 0`, `yaw rate = 0`
- Push：none
- Duration：20 s
- Trials：10

### 7.5 `push_flat_0p4`, 10 trials

- Scene：flat ground
- MuJoCo XML：`mujoco_test/data/go2/scene.xml`
- Command：`vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
- External disturbance：40 N lateral push
- Push direction：+Y direction force on `base_link`
- 代码实现：`data.xfrc_applied[base_body_id, 1] = args.push_force`
- Push duration：0.1 s
- Push start time：5.0 s
- Duration：20 s
- Trials：10

### 7.6 `push_terrain_0p4`, 10 trials

- Scene：challenging terrain
- MuJoCo XML：`mujoco_test/data/go2/scene_terrain.xml`
- Command：`vx = 0.4 m/s`, `vy = 0`, `yaw rate = 0`
- External disturbance：40 N lateral push
- Push direction：与 `push_flat_0p4` 相同，即 +Y force on `base_link`
- Push duration：0.1 s
- Push start time：5.0 s
- Duration：20 s
- Trials：10

## 8. Running RL MuJoCo Evaluation

当前已经验证成功的 RL evaluation 命令：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student

POLICY_PATH=${PWD}/mujoco_test/model/rsl_rl_teacher_student/actor/actor_oracle_ece489.pth \
ENCODER_PATH=${PWD}/mujoco_test/model/rsl_rl_teacher_student/proprio_encoder/proprio_oracle_ece489.pth \
DURATION=20 \
OUT_ROOT=${PWD}/project_outputs/eval_logs \
scripts/run_me446_rl_evaluation.sh
```

`scripts/run_me446_rl_evaluation.sh` 的行为：

- 如果设置了 `POLICY_PATH`，则使用 actor + proprio encoder 模式。
- 如果没有设置 `POLICY_PATH`，则使用 checkpoint 模式，即传入 `--checkpoint "$CHECKPOINT"`。
- 输出目录：`project_outputs/eval_logs/`

当前已经生成了 60 个 RL evaluation CSV：

- `flat_0p4`：10 trials
- `flat_0p8`：10 trials
- `terrain_0p4`：10 trials
- `terrain_0p8`：10 trials
- `push_flat_0p4`：10 trials
- `push_terrain_0p4`：10 trials

## 9. Evaluation CSV Schema

每个 trial CSV 必须包含以下字段，才能被统一 analysis script 正确分析：

- `time`：simulation time，单位 s
- `cmd_vx`, `cmd_vy`, `cmd_yaw`：期望的 body velocity 和 yaw rate command
- `base_x`, `base_y`, `base_z`：floating base position
- `com_x`, `com_y`, `com_z`：robot center of mass position
- `base_vx`, `base_vy`, `base_vz`：body frame 下的 base linear velocity
- `roll`, `pitch`, `yaw`：base orientation
- `joint_pos_1` 到 `joint_pos_12`：12 个 joint positions
- `joint_vel_1` 到 `joint_vel_12`：12 个 joint velocities
- `joint_torque_1` 到 `joint_torque_12`：12 个 joint torques，用于计算 CoT
- `foot_contact_FL`, `foot_contact_FR`, `foot_contact_RL`, `foot_contact_RR`：四只脚的 binary contact flag
- `foot_force_FL`, `foot_force_FR`, `foot_force_RL`, `foot_force_RR`：四只脚 contact force norm
- `external_push_flag`：push interval 内为 1，否则为 0
- `fall_flag`：当前 time step 是否满足 fall criterion
- `success_flag`：最终 trial success flag，会写到该 trial 所有 rows 中
- `distance_traveled`：从初始位置开始的水平位移距离
- `robot_mass`：用于 CoT normalization 的 robot mass
- `sim_dt`：MuJoCo simulation time step

当前 RL logger 不伪造 full 3D foot force vectors。它只记录每只脚一个 scalar contact force norm。

## 10. Metrics

analysis script 会先对每个 trial 计算 metrics，再对每个 condition 统计 mean +/- std。

RMS velocity tracking error：

```text
sqrt(mean((cmd_vx - base_vx)^2))
```

RMS roll：

```text
sqrt(mean(roll^2))
```

RMS pitch：

```text
sqrt(mean(pitch^2))
```

Cost of Transport：

```text
CoT = sum(abs(tau_i * qdot_i) * dt) / (m * g * distance)
```

`tools/analyze_rl_eval_logs.py` 中使用的 success rule：

- 整个 trial 中 `fall_flag` 必须始终为 0。
- 最终 `base_z` 必须大于等于 0.18 m。
- 最终 `abs(roll)` 和 `abs(pitch)` 都必须小于等于 0.8 rad。

MuJoCo RL logger 默认使用相同 fall thresholds：

- `--min_base_height=0.18`
- `--max_abs_roll_pitch=0.8`

`tools/analyze_rl_eval_logs.py` 中使用的 push recovery success rule：

- trial 必须包含 external push interval。
- push 结束后 `fall_flag` 必须保持为 0。
- recovery window 通常从 push 结束 2 s 后开始，到 rollout 结束为止。
- 在 recovery window 中，至少 50% samples 必须同时满足：
  - `abs(roll) < 0.4`
  - `abs(pitch) < 0.4`
  - `abs(cmd_vx - base_vx) < 0.35 m/s`

## 11. Analyzing Evaluation Logs

分析 RL evaluation logs：

```bash
python tools/analyze_rl_eval_logs.py \
  --eval_root project_outputs/eval_logs \
  --out_dir project_outputs/eval_results
```

也可以使用通用 wrapper：

```bash
python tools/analyze_eval_logs.py \
  --eval_root project_outputs/eval_logs \
  --out_dir project_outputs/eval_results
```

当前已经生成的 outputs：

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

为了和 RL 做公平比较，model-based 方法必须满足以下接口要求：

- 使用同一个 MuJoCo robot model。
- 使用同一个 flat scene：`mujoco_test/data/go2/scene.xml`。
- 使用同一个 terrain scene：`mujoco_test/data/go2/scene_terrain.xml`。
- 使用和 RL logger 一致的 initial robot pose 和 initial joint pose。
- 使用同一组 command set：
  - `flat_0p4`
  - `flat_0p8`
  - `terrain_0p4`
  - `terrain_0p8`
  - `push_flat_0p4`
  - `push_terrain_0p4`
- 每个 trial 使用相同 duration：20 s。
- push 条件使用相同 lateral push 设置：
  - 40 N
  - 0.1 s
  - start at 5.0 s
  - +Y force applied to `base_link`
- 输出 Section 9 中完全相同的 CSV schema。
- failure trials 必须保留，不能在 analysis 前删除。

建议 model-based logs 输出到：

```bash
project_outputs/eval_logs_model_based/
```

condition 子目录名称必须和 RL 一致：

```text
flat_0p4
flat_0p8
terrain_0p4
terrain_0p8
push_flat_0p4
push_terrain_0p4
```

model-based logger 的命令行接口模板：

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

push condition 的 model-based logger 应支持同样参数：

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

这里没有提供 model-based controller logger 的具体实现，因为 controller 属于组员的 model-based 部分。这里定义的是公平比较所需的 command-line interface 和 CSV schema。

## 13. Optional: Make Analysis Support Multiple Methods

`tools/analyze_rl_eval_logs.py` 虽然名字里有 `rl`，但实际逻辑是 method-agnostic 的。只要 CSV schema 和 Section 9 一致，它同样可以分析 model-based logs。

分析 model-based logs：

```bash
python tools/analyze_rl_eval_logs.py \
  --eval_root project_outputs/eval_logs_model_based \
  --out_dir project_outputs/eval_results_model_based
```

或者使用更通用的 wrapper：

```bash
python tools/analyze_eval_logs.py \
  --eval_root project_outputs/eval_logs_model_based \
  --out_dir project_outputs/eval_results_model_based
```

final report 中可以并排比较两个 summary files：

```text
project_outputs/eval_results/summary_metrics.csv
project_outputs/eval_results_model_based/summary_metrics.csv
```

## 14. Generate MuJoCo Scene Screenshots

使用 evaluation 中同一套 MuJoCo XML 生成场景截图：

```bash
python tools/capture_mujoco_eval_scenes.py \
  --gl_backend egl \
  --out_dir project_outputs/scene_screenshots
```

如果 `egl` 不可用，可以尝试：

```bash
python tools/capture_mujoco_eval_scenes.py \
  --gl_backend osmesa \
  --out_dir project_outputs/scene_screenshots
```

当前已经生成：

- `project_outputs/scene_screenshots/flat_scene.png`
- `project_outputs/scene_screenshots/terrain_scene.png`
- `project_outputs/scene_screenshots/push_flat_0p4_scene.png`
- `project_outputs/scene_screenshots/push_terrain_0p4_scene.png`

push screenshots 来自真实 flat 或 terrain XML render，只是在图片上叠加了 “40 N lateral push, 0.1 s” 的说明箭头。它们是 scene illustration，不是 evaluation data。

如果某台机器不能 offscreen render，可以手动打开 MuJoCo viewer 截图：

```bash
python -m mujoco.viewer --mjcf mujoco_test/data/go2/scene.xml
python -m mujoco.viewer --mjcf mujoco_test/data/go2/scene_terrain.xml
```

## 15. Scene Screenshots

当前 scene screenshots 对应关系：

- `flat_scene.png`：`flat_0p4`、`flat_0p8` 和 `push_flat_0p4` 使用的 flat ground scene
- `terrain_scene.png`：`terrain_0p4`、`terrain_0p8` 和 `push_terrain_0p4` 使用的 challenging terrain scene
- `push_flat_0p4_scene.png`：flat scene 加 40 N lateral push annotation
- `push_terrain_0p4_scene.png`：terrain scene 加 40 N lateral push annotation

## 16. Current Evaluation Result Notes

当前 RL summary 文件：

```bash
project_outputs/eval_results/summary_metrics.csv
```

当前 60-trial RL evaluation 的 success rate 如下：

- `flat_0p4`：success rate 1.0
- `flat_0p8`：success rate 1.0
- `terrain_0p4`：success rate 1.0
- `terrain_0p8`：success rate 0.0
- `push_flat_0p4`：no-fall success rate 1.0，push recovery success rate 0.0
- `push_terrain_0p4`：no-fall success rate 1.0，push recovery success rate 0.0

根据当前 analysis script 的 success rule，`terrain_0p8` 是 failure condition。两个 push condition 虽然 no-fall success flag 为 1.0，但没有满足更严格的 push recovery criterion，因此 push recovery success rate 为 0.0。

这些失败结果需要保留，用于 final report 中讨论 policy limitation。不要删除失败 trials，也不要用成功视频替换失败数据。

## 17. Do Not Fabricate Data

- 不要生成假的 evaluation CSV。
- 不要生成假的 metrics。
- 不要删除失败 trials。
- 不要用成功 trial 覆盖失败结果。
- 如果某个 controller 无法提供某个字段，应明确说明，并留空或写 `nan`，不要填假的常数。
- 当前 RL data 和 figures 均来自 `project_outputs/` 下已有 training logs 和 evaluation CSV。
- model-based results 目前还不可用，除非 model-based 组员按照 Section 9 的 CSV schema 生成对应 logs。

