# Go2 实机部署与数据采集说明

本文档用于 `/home/yd/ece489/project/rsl_rl_teacher_student` 项目中的 Unitree Go2 实机部署和数据采集。当前推荐方案是：部署控制路径保持原始 `deploy_policy.py`，数据记录使用旁路 LCM logger。旁路 logger 只订阅 LCM 消息，不发送控制命令，不 import `deploy_policy.py`，不插入 policy inference 或 action send 的主控制 loop。

## 1. 目的

实机数据用于分析 RL policy 的 sim-to-real gap，并和 MuJoCo 中的 policy 表现做对比。重点观察姿态稳定性、遥控命令下的速度响应、关节跟踪误差、控制目标、关节速度/力矩估计、以及轻扰动后的恢复情况。

已有的 stand CSV 保留，但之后更推荐使用 `real_robot_outputs/scripts/lcm_side_logger.py` 旁路采集。

## 2. 前期依赖安装

安装 LCM：

```bash
git clone https://github.com/lcm-proj/lcm.git
mkdir build
cd build
cmake ..
make
sudo make install
```

编译 Unitree SDK2：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2
rm -rf build
sudo ./install.sh
mkdir build
cd build
cmake ..
make
```

编译 `go2_gym_deploy`：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy
rm -rf build
mkdir build
cd build
cmake ..
make -j
```

## 3. 网络配置

PC 有线网卡 IPv4 设置为：

```text
192.168.123.10
```

检查能否连到机器人：

```bash
ping 192.168.123.161
```

查看网口名：

```bash
ip addr
# 或
ifconfig
```

`enp52s0` 只是当前机器上的示例网口名，正式运行时必须替换成 `ip addr` 或 `ifconfig` 中看到的实际有线网口名。

## 4. 部署前通信测试

可以用 `lcm_receive` 检查 bridge 是否在发布 LCM 数据。它不是每次部署必须跑，但通信异常时建议先跑。

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy/build
sudo ./lcm_receive
```

## 5. 原始部署方式

原始部署仍然是两个 terminal。这个流程不带数据 logging，应该优先用于确认 TS_re3 policy 是否恢复到原始表现。

Terminal 1：底层 bridge

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy/build
sudo ./lcm_position_go2 enp52s0
```

Terminal 2：原始 policy

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy/scripts
PYTHONPATH=/home/yd/ece489/project/rsl_rl_teacher_student/deployment python deploy_policy.py
```

`deploy_policy.py` 当前只加载 TS_re3：

```text
actor_oracle_TS_re3.pth
proprio_oracle_TS_re3.pth
```

如果这两个文件不存在，程序应直接报错，不会 fallback 到其他 policy。

## 6. 推荐三终端采集流程

正式采集时使用三个 terminal。Terminal 2 保持原始部署行为；Terminal 3 只监听 LCM，不控制机器人。

Terminal 1：底层 bridge

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy/build
sudo ./lcm_position_go2 enp52s0
```

Terminal 2：原始 policy，不带 logging

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/deployment/go2_gym_deploy/scripts
PYTHONPATH=/home/yd/ece489/project/rsl_rl_teacher_student/deployment python deploy_policy.py
```

Terminal 3：旁路 logger

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
PYTHONPATH=/home/yd/ece489/project/rsl_rl_teacher_student/deployment \
python real_robot_outputs/scripts/lcm_side_logger.py \
  --trial_name real_remote_walk_trial01 \
  --experiment_stage remote_control_walk \
  --duration 60 \
  --log_dir /home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/logs/side_logger/remote_control_walk \
  --video_filename real_remote_walk_trial01.mp4
```

如果 Terminal 3 崩溃，理论上不应影响机器人控制，因为它不发送控制命令。如果机器人表现不稳，先关闭 Terminal 3，只跑 Terminal 1 + Terminal 2 复现原始表现。

## 7. 实验阶段

当前流程为三阶段：

1. Stage 1：静态站立 `stand`
2. Stage 2：遥控自由行走 `remote_control_walk`
3. Stage 3：小扰动测试 `push_light`

旧的固定速度低速直线测试和速度扫描测试已弃用，不再推荐通过 `deploy_policy.py` 内部 logging 采集数据。

### Stage 1：静态站立

用途：检查姿态是否稳定、roll/pitch 是否过大、PD target 和 joint actual 是否一致。

建议命令：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
PYTHONPATH=/home/yd/ece489/project/rsl_rl_teacher_student/deployment \
python real_robot_outputs/scripts/lcm_side_logger.py \
  --trial_name real_stand_trial01 \
  --experiment_stage stand \
  --duration 10 \
  --log_dir /home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/logs/side_logger/stand \
  --video_filename real_stand_trial01.mp4
```

### Stage 2：遥控自由行走

用途：操作者用遥控器控制 Go2 在室内平地自由行走约 60 秒，记录遥控命令、机器人状态、关节状态、PD target、关节跟踪误差和控制目标。它不是固定速度测试，不应和 MuJoCo fixed-vx results 做严格一一比较。

建议命令：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
PYTHONPATH=/home/yd/ece489/project/rsl_rl_teacher_student/deployment \
python real_robot_outputs/scripts/lcm_side_logger.py \
  --trial_name real_remote_walk_trial01 \
  --experiment_stage remote_control_walk \
  --duration 60 \
  --log_dir /home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/logs/side_logger/remote_control_walk \
  --video_filename real_remote_walk_trial01.mp4
```

Stage 2 仍然要从小速度、轻遥控开始，不要突然给大速度。

### Stage 3：小扰动测试

用途：遥控或低速行走时人工轻踢/轻推机器人，观察是否摔倒、是否恢复、roll/pitch 峰值和关节响应。

建议命令：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
PYTHONPATH=/home/yd/ece489/project/rsl_rl_teacher_student/deployment \
python real_robot_outputs/scripts/lcm_side_logger.py \
  --trial_name real_push_light_trial01 \
  --experiment_stage push_light \
  --duration 30 \
  --log_dir /home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/logs/side_logger/push_light \
  --video_filename real_push_light_trial01.mp4
```

`push_light` 不是精确 40 N push，不能和 MuJoCo 中的 40 N push 做严格定量比较。只能作为 qualitative disturbance test。`push_time` 需要通过手机视频或人工记录后填写到 metadata。

## 8. 数据记录内容

`lcm_side_logger.py` 会订阅以下 LCM topic：

- `leg_control_data`
- `state_estimator_data`
- `rc_command`
- `rc_command_data`
- `pd_plustau_targets`

CSV 主要字段包括：

- 基础信息：`time`, `wall_time`, `loop_index`, `trial_name`, `experiment_stage`, `video_filename_suggested`
- 遥控命令：`rc_left_stick_x/y`, `rc_right_stick_x/y`, switches，以及兼容旧字段的 `remote_*`
- command 估计：`cmd_vx`, `cmd_vy`, `cmd_yaw`, `cmd_source`
- 机身状态：`base_vx`, `base_vy`, `base_yaw_rate`, `roll`, `pitch`, `yaw`, `angular_vel_x/y/z`, `base_height`
- 关节状态：`joint_pos_1 ... joint_pos_12`, `joint_vel_1 ... joint_vel_12`, `joint_torque_1 ... joint_torque_12`
- 控制目标：`target_joint_pos_1 ... target_joint_pos_12`, `target_joint_vel_*`, `target_tau_ff_*`, `pd_kp_*`, `pd_kd_*`
- 跟踪误差：`joint_tracking_error_1 ... joint_tracking_error_12`
- 足端信息：`foot_force_1 ... foot_force_4`, `foot_contact_1 ... foot_contact_4`

注意：旁路 logger 不能看到 policy 内部 observation、actor action 或 policy inference latency，因此这些字段不会出现在 side logger CSV 中。如果某个 topic 没有收到，对应字段会是 NaN，终端会 warning。

`cmd_vx/cmd_vy/cmd_yaw` 是根据当前恢复版 `StateEstimator.get_command()` 和默认 deploy scale 旁路估计出来的，不是从 `deploy_policy.py` 内部直接读出的变量。原始遥控 raw values 以 `rc_*` / `remote_*` 字段为准。

## 9. 视频记录

手机录像即可。建议角度为斜前方或侧前方，能看到 Go2 全身、地面和操作者轻扰动动作。

推荐文件名和 CSV 对齐：

```text
real_stand_trial01.mp4
real_remote_walk_trial01.mp4
real_push_light_trial01.mp4
```

## 10. Metadata

每个 trial 前复制：

```bash
cp /home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/metadata/trial_metadata_template.yaml \
   /home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/metadata/real_remote_walk_trial01.yaml
```

填写 `trial_name`, `operator`, `experiment_stage`, `video_filename`, `planned_duration`。trial 后补充 `actual_duration`, `success`, `fall`, `manual_stop`, `emergency_stop`, `notes`。

`push_light` 必须额外填写：

- `push_time`
- `push_direction`
- `push_strength_description: light kick by foot`

## 11. 如何退出

`lcm_side_logger.py --duration 60` 到时会自动保存 CSV。看到 `Side logger CSV saved` 后可以按 Ctrl+C 退出其他不需要的程序，或重新启动下一次 trial。

如果实验异常，先保证机器人安全，再 Ctrl+C。side logger 会尽量保存 partial CSV。

## 12. 安全注意事项

- 必须有人在旁边保护机器人。
- 必须随时准备急停或 damping 操作。
- Stage 2 先慢慢遥控，不要突然给大速度。
- Stage 3 只能轻踢/轻推，不要大力踢。
- 出现剧烈抖动、异常声音、摔倒趋势、电机异常时立即停止。
- 不要在复杂地形上测试。
- 如果表现不稳，先停用 Terminal 3，只保留 Terminal 1 + Terminal 2 做 A/B 排查。

## 13. 后处理

汇总 CSV：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
python real_robot_outputs/scripts/summarize_real_logs.py
```

画图：

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
python real_robot_outputs/scripts/plot_real_robot_logs.py
```

输出：

- `real_robot_outputs/summary/trial_metrics.csv`
- `real_robot_outputs/summary/summary_metrics.csv`
- `real_robot_outputs/figures/`

后处理脚本会尽量有字段就计算，缺少 policy action / obs / latency 字段时 warning 后跳过，不应崩溃。

## 14. 与仿真对比

建议对比：

- RMS roll / pitch
- max abs roll / pitch
- velocity tracking error，如果 `cmd_vx` 估计可靠
- joint tracking error
- joint velocity / torque statistics
- 成功率和失败原因

`remote_control_walk` 不是 fixed-vx 测试，`push_light` 不是精确 40 N push，所以不要和 MuJoCo fixed-vx 或 40 N push 做严格定量一一比较。

## 15. 常见问题

ping 不通：检查网线、有线 IPv4、机器人 IP、网口是否选对。

找不到网口：用 `ip addr` 或 `ifconfig` 查，不要盲目使用 `enp52s0`。

`lcm_receive` 没数据：先确认 Terminal 1 bridge 是否启动，LCM 是否安装，网络是否通。

Python 找不到 package：确认使用：

```bash
PYTHONPATH=/home/yd/ece489/project/rsl_rl_teacher_student/deployment
```

policy 抖动：先停用 Terminal 3，只运行 Terminal 1 + Terminal 2；如果仍抖动，问题不在 side logger。

机器人不动：确认 Terminal 2 原始 policy 正常运行，确认遥控器模式和启动按键流程。

方向不对或速度异常：检查 `rc_*` raw values 和 `cmd_vx/cmd_yaw` 估计值。

Ctrl+C 后 partial log：`lcm_side_logger.py` 会在退出时写 CSV；如果没有收到 LCM 消息，CSV 可能只有表头。
