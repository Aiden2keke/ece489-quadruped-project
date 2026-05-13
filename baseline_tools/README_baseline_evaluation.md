# Baseline Evaluation Tools

This directory contains wrappers for evaluating `baselines/go2-convex-mpc`
without editing the upstream controller source.

Files:

- `eval_go2_convex_mpc_logger.py`: fixed-condition MuJoCo rollout logger.
- `run_baseline_evaluation.sh`: runs the six RL-matched conditions and 10 trials each by default.
- `analyze_baseline_eval_logs.py`: thin wrapper around the shared metrics analyzer.

Expected environment:

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student/baselines/go2-convex-mpc
conda env create -f environment.yml
conda activate go2-convex-mpc
pip install -e .
```

Run all baseline trials from the project root:

```bash
cd /home/yd/ece489/project/rsl_rl_teacher_student
PYTHON_BIN=/home/yd/anaconda3/envs/go2-convex-mpc/bin/python baseline_tools/run_baseline_evaluation.sh
```

Analyze logs:

```bash
python tools/analyze_eval_logs.py \
  --eval_root project_outputs_baseline/eval_logs \
  --out_dir project_outputs_baseline/eval_results
```

The logger uses the RL MuJoCo scene XML files:

- `mujoco_test/data/go2/scene.xml`
- `mujoco_test/data/go2/scene_terrain.xml`

It keeps the upstream MPC, trajectory generator, gait scheduler, and leg
controller intact.  Unavailable values are written as `NaN`; contact force is a
MuJoCo contact-force norm per foot, matching the RL logger convention.

Default initialization is `INIT_MODE=baseline_demo`: base position `[-5, 0,
0.27]`, identity yaw, and upstream demo joint angles `[0, 0.9, -1.8]` per leg.
The alternative `INIT_MODE=rl_scene` keeps the RL scene initial base pose and RL
joint angles; in this codebase that mode caused early MPC QP failures and is
archived as an adapter-failure attempt.
