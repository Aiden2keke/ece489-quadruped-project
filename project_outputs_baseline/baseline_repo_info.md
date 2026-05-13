# Baseline Repository Info

Generated: 2026-05-13

## Repository

- Repo URL: https://github.com/elijah-waichong-chan/go2-convex-mpc
- Local path: `/home/yd/ece489/project/rsl_rl_teacher_student/baselines/go2-convex-mpc`
- Current git commit hash: `1c63c6a762779887ab0431fd60db681dede6cb32`
- Branch: `main`

## Dependency Status

- Default project `python` can import the local `convex_mpc` package when `baselines/go2-convex-mpc/src` is on `PYTHONPATH`.
- Default project `python` is missing required runtime dependencies:
  - `mujoco`: missing
  - `pinocchio`: missing
  - `casadi`: missing
- Existing `rlgpu` conda environment has `mujoco 3.2.2` but is missing `pinocchio` and `casadi`.
- The upstream `go2-convex-mpc` conda environment was created from `environment.yml`.
- Import check in `go2-convex-mpc` passed:
  - `mujoco 3.1.6`
  - `pinocchio 2.7.1`
  - `casadi 3.7.2`
  - `convex_mpc OK`
- Upstream environment file: `baselines/go2-convex-mpc/environment.yml`
- `examples.ex02_trot_forward` executed the 5 s simulation loop, then failed at the replay/viewer stage with `GLFWError: X11: Failed to open display :0`. This is expected in the current non-GUI session and is why the project wrapper supports `--headless`.
- Project wrapper smoke test passed in the `go2-convex-mpc` environment and wrote `/tmp/baseline_smoke/flat_vx0p4_trial00.csv`.

## Main Example Scripts Found

- `examples/ex00_demo.py`
- `examples/ex01_trot_in_place.py`
- `examples/ex02_trot_forward.py`
- `examples/ex03_trot_sideway.py`
- `examples/ex04_trot_rotation.py`

The forward walking demo is `examples/ex02_trot_forward.py`.

## MuJoCo Model Paths Found In Baseline Repo

- `models/MJCF/go2/scene.xml`
- `models/MJCF/go2/scene_terrain.xml`
- `models/MJCF/go2/go2.xml`

## Modifications Made For Our Evaluation

- No files inside `baselines/go2-convex-mpc` were modified.
- Added project-local wrappers under `baseline_tools/`.
- Added baseline documentation and output directories under `project_outputs_baseline/`.
- Updated shared analysis logic in `tools/analyze_rl_eval_logs.py` so logged
  `success_flag=0` is respected for controller-error partial CSV files.
- Added optional comparison script `tools/compare_rl_baseline_metrics.py`.
- The wrapper loads the RL evaluation scenes from:
  - `/home/yd/ece489/project/rsl_rl_teacher_student/mujoco_test/data/go2/scene.xml`
  - `/home/yd/ece489/project/rsl_rl_teacher_student/mujoco_test/data/go2/scene_terrain.xml`
- The wrapper defaults to the upstream baseline demo standing initialization:
  - base position `[-5, 0, 0.27]`
  - base quaternion `[1, 0, 0, 0]` in MuJoCo `[w, x, y, z]` order
  - joint angles `[0, 0.9, -1.8]` per leg
- A first attempt using the RL scene embedded initial pose and RL logger joint
  pose was archived at `project_outputs_baseline/eval_logs_failed_rl_scene_init`
  because every trial hit an early OSQP failure at about 0.14-0.18 s.
- The wrapper reuses upstream controller classes:
  - `PinGo2Model`
  - `ComTraj`
  - `CentroidalMPC`
  - `LegController`
  - `Gait`
- Push disturbances are applied through MuJoCo `data.xfrc_applied[base_body_id, 1]`.
