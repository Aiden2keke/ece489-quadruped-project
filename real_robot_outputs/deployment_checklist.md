# Go2 实机部署 Checklist

## 部署前

- [ ] 电池电量充足
- [ ] 机器人周围空间清空
- [ ] 地面为室内平地，不是复杂地形
- [ ] 网线连接
- [ ] PC 有线 IPv4 设置为 `192.168.123.10`
- [ ] `ping 192.168.123.161` 成功
- [ ] 已用 `ip addr` 或 `ifconfig` 确认实际网口名
- [ ] LCM 编译安装完成
- [ ] Unitree SDK2 编译完成
- [ ] `go2_gym_deploy` 编译完成
- [ ] TS_re3 actor/proprio checkpoint 路径确认
- [ ] Terminal 1 `lcm_position_go2` 准备
- [ ] Terminal 2 原始 `deploy_policy.py` 准备
- [ ] Terminal 3 `lcm_side_logger.py` 准备
- [ ] 手机录像准备
- [ ] 急停或 damping 操作准备
- [ ] 保护人员就位

## 每个 trial 前

- [ ] 确认 Terminal 1 bridge 正常
- [ ] 确认 Terminal 2 原始 policy 正常
- [ ] 确认 Terminal 3 side logger 输出路径正确
- [ ] `trial_name` 与 metadata 一致
- [ ] `video_filename` 与 metadata 一致
- [ ] 手机视频文件名已设置
- [ ] CSV 和视频文件名对应
- [ ] 遥控器在手
- [ ] 操作者知道先小速度遥控，不突然给大速度
- [ ] Stage 1 `stand` 三次已完成，或确认需要重跑 stand
- [ ] Stage 2 `remote_control_walk` 的 `trial_name` 已设置
- [ ] Stage 3 `push_light` 只做轻踢/轻推
- [ ] Stage 3 `push_light` 准备从视频或人工记录 `push_time`
- [ ] 保护人员已站在合适位置

## 每个 trial 后

- [ ] side logger CSV 已生成
- [ ] 视频已保存
- [ ] CSV 和视频文件名对应
- [ ] metadata 更新 `actual_duration`
- [ ] metadata 更新 `success/fall/manual_stop/emergency_stop`
- [ ] metadata 记录异常声音、抖动、摔倒趋势或通信问题
- [ ] Stage 3 `push_light` 已记录 `push_time`
- [ ] Stage 3 已记录 `push_direction`
- [ ] Stage 3 已记录 `push_strength_description`
- [ ] 如果机器人异常，保留失败数据和视频，不删除

## 后处理前

- [ ] 所有 side logger CSV 在 `real_robot_outputs/logs/side_logger/` 下
- [ ] metadata 与 trial 名称一致
- [ ] 视频文件名与 CSV 对应
- [ ] 运行 `summarize_real_logs.py`
- [ ] 运行 `plot_real_robot_logs.py`
- [ ] 检查 `summary/trial_metrics.csv`
- [ ] 检查 `summary/summary_metrics.csv`
- [ ] 检查 `figures/` 中的图
