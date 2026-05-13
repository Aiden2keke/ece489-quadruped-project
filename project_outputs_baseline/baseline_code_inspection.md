# Baseline Code Inspection

Generated: 2026-05-13

## Examples Directory

Found demo scripts:

- `examples/ex00_demo.py`
- `examples/ex01_trot_in_place.py`
- `examples/ex02_trot_forward.py`
- `examples/ex03_trot_sideway.py`
- `examples/ex04_trot_rotation.py`

Forward walking demo:

- `examples/ex02_trot_forward.py`

## Controller Entry Points

The upstream examples are executable scripts rather than a single reusable CLI.
`examples/ex02_trot_forward.py` constructs the complete controller stack:

- `PinGo2Model` from `src/convex_mpc/go2_robot_data.py`
- `MuJoCo_GO2_Model` from `src/convex_mpc/mujoco_model.py`
- `ComTraj` from `src/convex_mpc/com_trajectory.py`
- `CentroidalMPC` from `src/convex_mpc/centroidal_mpc.py`
- `LegController` from `src/convex_mpc/leg_controller.py`
- `Gait` from `src/convex_mpc/gait.py`

The simulation loop in the example runs:

- MuJoCo physics at 1000 Hz
- leg controller at 200 Hz
- convex MPC at approximately 48 Hz for the default 3 Hz trot and 16-step horizon

## Commanded Velocity

The example defines a `BodyCmdPhase` dataclass and `CMD_SCHEDULE`.
`get_body_cmd(t)` returns:

- `x_vel`
- `y_vel`
- `z_pos`
- `yaw_rate`

These values are passed into:

```python
traj.generate_traj(go2, gait, time_now_s, x_vel_des_body, y_vel_des_body, z_pos_des_body, yaw_rate_des_body, time_step=MPC_DT)
```

For our fixed-condition wrapper, the CLI arguments `--cmd_vx`, `--cmd_vy`, and
`--cmd_yaw` are passed to the same `generate_traj` call at every control update.

## Scene / XML Selection

The upstream `MuJoCo_GO2_Model` hard-codes:

```python
XML_PATH = REPO / "models" / "MJCF" / "go2" / "scene.xml"
```

The upstream examples do not expose a scene CLI argument.  The project wrapper
does not use this hard-coded loader.  It directly loads the RL comparison scenes:

- flat: `mujoco_test/data/go2/scene.xml`
- terrain: `mujoco_test/data/go2/scene_terrain.xml`

This keeps scene choice explicit and avoids modifying the upstream repo.

## Base State

MuJoCo state access:

- base position: `data.qpos[0:3]`
- base quaternion in MuJoCo order: `data.qpos[3:7]` as `[w, x, y, z]`
- base world linear velocity: `data.qvel[0:3]`
- base angular velocity: `data.qvel[3:6]`

The upstream `MuJoCo_GO2_Model.update_pin_with_mujoco` converts MuJoCo state to
Pinocchio order:

- quaternion becomes `[x, y, z, w]`
- world linear velocity is rotated into the body frame for Pinocchio

The wrapper implements the same conversion locally so it can use the RL scene
files.

## Joint State And Torque

Joint state in MuJoCo:

- joint positions: `data.qpos[7:19]`
- joint velocities: `data.qvel[6:18]`

The upstream low-level controller computes one 3-DoF torque vector per leg via:

```python
LegController.compute_leg_torque(...)
```

The wrapper records the clipped controller torque command in the log-order:

```text
FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
```

The torque command is applied to MuJoCo actuators by actuator name, so it does
not depend on actuator ordering in the XML.

## Contact And Foot Forces

The upstream examples do not log contacts.  The wrapper reads MuJoCo contacts
from `data.contact` and identifies foot contacts using geoms named:

- `FL`
- `FR`
- `RL`
- `RR`

For each contact involving a foot geom, the wrapper calls `mujoco.mj_contactForce`
and logs the norm of the first three contact force components.  This matches the
RL logger convention: per-foot scalar contact-force norm, not a full 3D world
force vector.

## External Push

The wrapper applies push disturbances through MuJoCo:

```python
data.xfrc_applied[base_body_id, 1] = push_force
```

This applies a +Y force on `base_link`, matching the RL evaluation setup as
closely as possible.

## Headless Support

The upstream examples always generate plots and call replay/viewer utilities
after simulation.  They do not provide a headless CLI mode.

The wrapper supports `--headless` and does not import or launch `mujoco.viewer`
unless headless mode is disabled.  This minimizes viewer dependency for batch
evaluation.

## Notes

- The wrapper keeps upstream controller code unchanged.
- The wrapper can use the RL scene initial pose via `--init_mode rl_scene`, but
  the default is `--init_mode baseline_demo` because the RL embedded pose caused
  early OSQP failures in the baseline controller.  The default still uses the RL
  flat/terrain XML files, but initializes the robot with the upstream demo base
  pose and joint angles.
- If the baseline controller fails at runtime, the wrapper writes any partial
  CSV rows and a sidecar `.error.txt` file; it does not convert failures into
  fake successes.
