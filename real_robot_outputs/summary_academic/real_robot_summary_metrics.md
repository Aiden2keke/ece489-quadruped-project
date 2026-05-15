# Real-Robot Deployment Summary

## Data Source

- Source: side-channel LCM logger (`lcm_side_logger.py`).
- The logger subscribes to LCM topics only and does not insert logging into the policy control loop.
- The logger does not send control commands and does not affect the action send order.

## Remote Walk Summary

- CSV: `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/logs/side_logger/remote_control_walk_(including_stand)/real_remote_walk_trial01.csv`
- Duration: 57.90 s
- Samples: 28911
- Roll / pitch RMS: 2.60 deg / 2.44 deg
- Max absolute roll / pitch: 9.68 deg / 6.61 deg
- Command range:
  - cmd_vx: [-0.700, 0.579], mean -0.031
  - cmd_yaw: [-0.963, 0.342], mean -0.059
- Motion fraction: 0.647
- Interpretation: this trial contains both standing and remote-controlled walking, so it should be treated as a qualitative deployment validation trial.

## Push-Light Summary

- CSV: `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/logs/side_logger/push_light/real_push_light_trial01.csv`
- Duration: 30.00 s
- Samples: 14982
- Estimated push time: 25.16 s (auto_max_angular_velocity_norm)
- Roll / pitch RMS: 3.19 deg / 3.00 deg
- Max absolute roll / pitch: 22.17 deg / 16.19 deg
- Max angular velocity norm: 5.075 rad/s
- Push note: manual qualitative light push, not calibrated 40 N.

## Limitations

- These real-robot logs are not strict one-to-one quantitative benchmarks against MuJoCo fixed-vx or calibrated 40 N push conditions.
- Base velocity, policy action, policy observation, and policy inference time are not available from the side logger unless exposed as LCM topics.
- Push strength is qualitative and should not be reported as a calibrated force.
- Foot contact/force fields may be unreliable if the contact fields are sparse or all zero.

## Generated Figures

- `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/figures_academic/real_robot_overview_commands_attitude.png`
- `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/figures_academic/push_light_zoom_response.png`
- `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/figures_academic/joint_tracking_error_heatmap_remote_walk.png`
- `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/figures_academic/joint_tracking_error_heatmap_push_light.png`
- `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/figures_academic/joint_torque_summary_bar.png`
- `/home/yd/ece489/project/rsl_rl_teacher_student/real_robot_outputs/figures_academic/real_robot_attitude_comparison_bar.png`

## Suggested Report Wording

A preliminary real-robot deployment was conducted on the Unitree Go2 using the TS_re3 policy. During the remote-controlled walking trial (57.90 s), the robot exhibited roll and pitch RMS values of 2.60 deg and 2.44 deg, with peak absolute roll/pitch of 9.68 deg / 6.61 deg. During the qualitative light-push trial (30.00 s), the estimated push event occurred at 25.16 s and the peak absolute roll/pitch reached 22.17 deg / 16.19 deg. These results provide qualitative deployment validation rather than a calibrated fixed-condition benchmark.
