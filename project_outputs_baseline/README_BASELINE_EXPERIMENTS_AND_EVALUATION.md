# Convex MPC Baseline Experiment and Evaluation Guide

## 1. Baseline Repository

- Repo URL: https://github.com/elijah-waichong-chan/go2-convex-mpc
- Local path: `/home/yd/ece489/project/rsl_rl_teacher_student/baselines/go2-convex-mpc`
- Current commit at inspection time: `1c63c6a762779887ab0431fd60db681dede6cb32`
- Branch: `main`

Baseline results are stored in:

```bash
project_outputs_baseline
```

RL results are stored in:

```bash
project_outputs_RL
```

Do not overwrite or delete `project_outputs_RL`.

## 2. Installation / Environment Notes

The upstream repo expects Python 3.10 and the dependencies listed in
`baselines/go2-convex-mpc/environment.yml`.

Recommended setup:

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/baselines/go2-convex-mpc
conda env create -f environment.yml
conda activate go2-convex-mpc
pip install -e .
```

Import check:

```bash
python - <<'PY'
import mujoco, pinocchio, casadi, convex_mpc
print("mujoco:", mujoco.__version__)
print("pinocchio:", pinocchio.__version__)
print("casadi:", casadi.__version__)
print("convex_mpc: OK")
PY
```

Current default project `python` is missing `mujoco`, `pinocchio`, and `casadi`,
so baseline evaluation cannot run in the default environment.  The
`go2-convex-mpc` conda environment was created from `environment.yml` and passed
the import check with `mujoco 3.1.6`, `pinocchio 2.7.1`, and `casadi 3.7.2`.

## 3. Original Baseline Demos

Run from the upstream repo root after activating the environment:

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/baselines/go2-convex-mpc
python -m examples.ex00_demo
python -m examples.ex01_trot_in_place
python -m examples.ex02_trot_forward
python -m examples.ex03_trot_sideway
python -m examples.ex04_trot_rotation
```

Forward walking demo:

```bash
python -m examples.ex02_trot_forward
```

The original demos create plots and launch replay/viewer code.  They are not the
batch logger used for RL comparison.

In the current non-GUI session, `examples.ex02_trot_forward` ran the 5 s
simulation loop but exited at the final GLFW replay/viewer step because X11
display `:0` is unavailable.  Use the project wrapper with `--headless` for
batch evaluation.

## 4. Wrapper Adaptation For RL Comparison

Project-local wrapper:

```bash
baseline_tools/eval_go2_convex_mpc_logger.py
```

The wrapper:

- reuses upstream `PinGo2Model`, `ComTraj`, `CentroidalMPC`, `LegController`, and `Gait`;
- directly loads the RL MuJoCo XML scenes;
- applies the same fixed commands and push timing as the RL evaluation;
- logs CSV files with the RL-compatible schema;
- supports `--headless`;
- records failed controller trials as failures, not fabricated successes.

No upstream baseline repo files were modified.

Initialization note: the wrapper defaults to `--init_mode baseline_demo`, using
the upstream demo standing pose `[-5, 0, 0.27]` with identity yaw and joint
angles `[0, 0.9, -1.8]` per leg.  A first attempt with `--init_mode rl_scene`
kept the RL scene embedded base pose and RL logger joint pose, but all 60 trials
hit early OSQP failures around 0.14-0.18 s.  Those failed partial logs are kept
under `project_outputs_baseline/eval_logs_failed_rl_scene_init`.

## 5. Evaluation Conditions

Each condition uses duration 20 s and 10 trials by default.

| Condition | Scene | Command | Push |
| --- | --- | --- | --- |
| `flat_0p4` | `mujoco_test/data/go2/scene.xml` | `vx=0.4, vy=0, yaw=0` | none |
| `flat_0p8` | `mujoco_test/data/go2/scene.xml` | `vx=0.8, vy=0, yaw=0` | none |
| `terrain_0p4` | `mujoco_test/data/go2/scene_terrain.xml` | `vx=0.4, vy=0, yaw=0` | none |
| `terrain_0p8` | `mujoco_test/data/go2/scene_terrain.xml` | `vx=0.8, vy=0, yaw=0` | none |
| `push_flat_0p4` | `mujoco_test/data/go2/scene.xml` | `vx=0.4, vy=0, yaw=0` | +Y 40 N on `base_link`, 0.1 s, starts at 5.0 s |
| `push_terrain_0p4` | `mujoco_test/data/go2/scene_terrain.xml` | `vx=0.4, vy=0, yaw=0` | +Y 40 N on `base_link`, 0.1 s, starts at 5.0 s |

## 6. Run Baseline Batch Evaluation

From the project root:

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
PYTHON_BIN=/home/yd/anaconda3/envs/go2-convex-mpc/bin/python baseline_tools/run_baseline_evaluation.sh
```

Supported environment variables:

- `DURATION`, default `20`
- `TRIALS`, default `10`
- `OUT_ROOT`, default `project_outputs_baseline/eval_logs`
- `BASELINE_REPO`, default `baselines/go2-convex-mpc`
- `PYTHON_BIN`, optional path to the baseline conda environment Python
- `INIT_MODE`, default `baseline_demo`; alternative `rl_scene` is retained for debugging but failed in this setup

Example short smoke test:

```bash
DURATION=1 TRIALS=1 PYTHON_BIN=/home/yd/anaconda3/envs/go2-convex-mpc/bin/python baseline_tools/run_baseline_evaluation.sh
```

## 7. Analyze Baseline Logs

Use the shared RL-compatible analyzer:

```bash
python tools/analyze_eval_logs.py \
  --eval_root project_outputs_baseline/eval_logs \
  --out_dir project_outputs_baseline/eval_results
```

Equivalent wrapper:

```bash
python baseline_tools/analyze_baseline_eval_logs.py \
  --eval_root project_outputs_baseline/eval_logs \
  --out_dir project_outputs_baseline/eval_results
```

Expected outputs:

- `project_outputs_baseline/eval_results/trial_metrics.csv`
- `project_outputs_baseline/eval_results/summary_metrics.csv`
- `project_outputs_baseline/eval_results/rms_velocity_error.png`
- `project_outputs_baseline/eval_results/rms_roll_pitch.png`
- `project_outputs_baseline/eval_results/cot_comparison.png`
- `project_outputs_baseline/eval_results/push_recovery_success.png`
- `project_outputs_baseline/eval_results/example_velocity_tracking_flat.png`
- `project_outputs_baseline/eval_results/example_velocity_tracking_terrain.png`
- `project_outputs_baseline/eval_results/example_roll_pitch_flat.png`
- `project_outputs_baseline/eval_results/example_roll_pitch_terrain.png`
- `project_outputs_baseline/eval_results/example_push_recovery.png`
- `project_outputs_baseline/eval_results/contact_gait_diagram.png` if contact columns are available

## 8. CSV Schema

Each trial CSV contains:

```text
time
cmd_vx, cmd_vy, cmd_yaw
base_x, base_y, base_z
com_x, com_y, com_z
base_vx, base_vy, base_vz
roll, pitch, yaw
joint_pos_1 ... joint_pos_12
joint_vel_1 ... joint_vel_12
joint_torque_1 ... joint_torque_12
foot_contact_FL, foot_contact_FR, foot_contact_RL, foot_contact_RR
foot_force_FL, foot_force_FR, foot_force_RL, foot_force_RR
external_push_flag
fall_flag
success_flag
distance_traveled
robot_mass
sim_dt
```

Joint order:

```text
FL_hip, FL_thigh, FL_calf,
FR_hip, FR_thigh, FR_calf,
RL_hip, RL_thigh, RL_calf,
RR_hip, RR_thigh, RR_calf
```

## 9. Metrics Definitions

The analyzer computes:

- `rms_velocity_error`: RMS of `cmd_vx - base_vx`
- `rms_roll`: RMS roll angle
- `rms_pitch`: RMS pitch angle
- `cot`: sum of absolute joint mechanical power divided by `mass * 9.81 * distance`
- `success`: 1 if no fall and final pose satisfies height and roll/pitch limits
- `push_recovery_success`: for push trials, 1 if the robot remains upright after push and later returns to stable roll/pitch and velocity tracking

Fall thresholds:

- `min_base_height = 0.18`
- `max_abs_roll_pitch = 0.8 rad`

## 10. Unavailable Or Approximated Fields

- `joint_torque_*` records the clipped torque command produced by the baseline leg controller and applied to MuJoCo.
- `foot_force_*` is a scalar MuJoCo contact-force norm per foot, not a full 3D world-frame foot force.
- `foot_contact_*` is physical MuJoCo foot contact, not merely the planned gait contact schedule.
- If the controller fails before a valid torque is available, torque fields are written as `NaN` in the partial CSV.
- No metric is fabricated.  Failed trials remain in logs when a CSV can be written.

## 11. Scene Screenshot Explanation

Baseline and RL use the same MuJoCo scene XML files for this comparison.  The
baseline screenshots in `project_outputs_baseline/scene_screenshots/` are copied
from `project_outputs_RL/scene_screenshots/` unless regenerated later.

Required screenshots:

- `flat_scene.png`
- `terrain_scene.png`
- `push_flat_0p4_scene.png`
- `push_terrain_0p4_scene.png`

## 12. Compare With RL

Final comparison should use:

```bash
project_outputs_RL/eval_results/summary_metrics.csv
project_outputs_baseline/eval_results/summary_metrics.csv
```

Optional comparison script:

```bash
python tools/compare_rl_baseline_metrics.py \
  --rl_summary project_outputs_RL/eval_results/summary_metrics.csv \
  --baseline_summary project_outputs_baseline/eval_results/summary_metrics.csv \
  --out_dir project_outputs_comparison
```

Outputs:

- `project_outputs_comparison/comparison_summary.csv`
- `project_outputs_comparison/comparison_velocity_error.png`
- `project_outputs_comparison/comparison_roll_pitch.png`
- `project_outputs_comparison/comparison_cot.png`
- `project_outputs_comparison/comparison_push_success.png`

## 12.5 Completed Run Status

Final baseline run outputs are in `project_outputs_baseline/eval_logs`.

- Total final CSV files: 60
- Final error sidecars: 20
- Error sidecars occur in `flat_0p8` and `terrain_0p8`, 10 each.
- `flat_0p8` and `terrain_0p8` are partial controller-failure trials and are
  summarized with `success_mean = 0`.
- The first failed adapter attempt with `INIT_MODE=rl_scene` is archived in
  `project_outputs_baseline/eval_logs_failed_rl_scene_init`.

See `project_outputs_baseline/evaluation_run_status.md` for the exact run
status.

## 13. Known Limitations And Failure Cases

- The upstream examples do not provide a headless CLI; the project wrapper supplies headless evaluation.
- The upstream MuJoCo loader hard-codes the baseline repo scene path; the wrapper bypasses that loader to use the RL scene XML.
- The baseline Pinocchio model is loaded from the upstream URDF while MuJoCo simulation uses the RL MJCF scene.  The models have matching Go2 joint/body naming in the inspected code, but small inertial or friction differences may remain.
- The final baseline logs use the upstream demo initial pose because the RL scene embedded initial pose caused early OSQP failures.
- With `INIT_MODE=baseline_demo`, the terrain and flat 0.4 m/s trajectories are
  very similar because the terrain obstacles in the shared XML are not on the
  executed path for this initialization.
- Current default project `python` cannot run the baseline because required dependencies are missing; use the `go2-convex-mpc` conda environment.
- Upstream demo replay/viewer requires a GUI display.  The 5 s original forward demo simulation ran, but the final viewer failed in this session with `X11: Failed to open display :0`.
- Terrain trials may fail if the controller cannot clear the RL terrain obstacles.  These failures should be retained for failure-mode analysis.

## 14. Do-Not-Fabricate-Data Policy

- Do not generate fake baseline CSV files.
- Do not generate fake `summary_metrics.csv`.
- Do not delete failed trials.
- Do not convert falls into successes.
- Do not fill unavailable torque/contact values with arbitrary zeros; use `NaN` when values are not reliable.
- Do not overwrite or modify `project_outputs_RL`.
