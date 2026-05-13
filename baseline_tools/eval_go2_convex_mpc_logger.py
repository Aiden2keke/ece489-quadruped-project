#!/usr/bin/env python3
"""Fixed-condition logger for the go2-convex-mpc baseline.

The upstream examples are script-style demos with plotting and replay.  This
wrapper reuses the upstream controller components but owns the MuJoCo rollout,
scene selection, push injection, and CSV schema so results can be compared with
the RL evaluation logs.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE_REPO = REPO_ROOT / "baselines" / "go2-convex-mpc"
ROBOT_DATA_DIR = REPO_ROOT / "mujoco_test" / "data" / "go2"

FOOT_NAMES = ("FL", "FR", "RL", "RR")
LEG_SLICES = {
    "FL": slice(0, 3),
    "FR": slice(3, 6),
    "RL": slice(6, 9),
    "RR": slice(9, 12),
}
ACTUATOR_NAMES = (
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
)

# Match the RL evaluation logger's initial joint pose when using the RL scenes.
DEFAULT_JOINT_ANGLES = np.array(
    [
        0.1,
        0.8,
        -1.5,
        -0.1,
        0.8,
        -1.5,
        0.1,
        1.0,
        -1.5,
        -0.1,
        1.0,
        -1.5,
    ],
    dtype=np.float64,
)
BASELINE_DEMO_JOINT_ANGLES = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float64)
BASELINE_DEMO_BASE_POS = np.array([-5.0, 0.0, 0.27], dtype=np.float64)
BASELINE_DEMO_BASE_QUAT_WXYZ = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

SIM_HZ = 1000
CTRL_HZ = 200
SIM_DT = 1.0 / SIM_HZ
CTRL_DECIM = SIM_HZ // CTRL_HZ

GAIT_HZ = 3.0
GAIT_DUTY = 0.6
MPC_DT = (1.0 / GAIT_HZ) / 16.0
MPC_HZ = 1.0 / MPC_DT
STEPS_PER_MPC = max(1, int(CTRL_HZ // MPC_HZ))
Z_POS_DES_BODY = 0.27

HIP_LIM = 23.7
ABD_LIM = 23.7
KNEE_LIM = 45.43
SAFETY = 0.9
TAU_LIM = SAFETY * np.array(
    [
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
    ],
    dtype=np.float64,
)


def resolve_path(path: Path) -> Path:
    return path if path.is_absolute() else (REPO_ROOT / path).resolve()


def load_baseline_modules(baseline_repo: Path) -> dict[str, Any]:
    src_dir = baseline_repo / "src"
    if not src_dir.exists():
        raise FileNotFoundError(f"Baseline src directory not found: {src_dir}")
    sys.path.insert(0, str(src_dir))

    import mujoco as mj  # noqa: PLC0415

    from convex_mpc.centroidal_mpc import CentroidalMPC  # noqa: PLC0415
    from convex_mpc.com_trajectory import ComTraj  # noqa: PLC0415
    from convex_mpc.gait import Gait  # noqa: PLC0415
    from convex_mpc.go2_robot_data import PinGo2Model  # noqa: PLC0415
    from convex_mpc.leg_controller import LegController  # noqa: PLC0415

    return {
        "mj": mj,
        "CentroidalMPC": CentroidalMPC,
        "ComTraj": ComTraj,
        "Gait": Gait,
        "PinGo2Model": PinGo2Model,
        "LegController": LegController,
    }


def quat_wxyz_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q_w = q[0]
    q_vec = q[1:]
    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * (np.dot(q_vec, v) * 2.0)
    return a - b + c


def quat_to_rpy(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def update_pin_from_mujoco(go2: Any, data: Any) -> None:
    mujoco_q = np.asarray(data.qpos, dtype=float).reshape(-1)
    mujoco_dq = np.asarray(data.qvel, dtype=float).reshape(-1)
    qw, qx, qy, qz = mujoco_q[3:7]
    rotation_body_to_world = quat_wxyz_to_rot(np.array([qw, qx, qy, qz], dtype=np.float64))
    v_world = mujoco_dq[0:3]
    w_body = mujoco_dq[3:6]
    v_body = rotation_body_to_world.T @ v_world
    q_pin = np.concatenate([mujoco_q[0:3], [qx, qy, qz, qw], mujoco_q[7:19]])
    dq_pin = np.concatenate([v_body, w_body, mujoco_dq[6:18]])
    go2.update_model(q_pin, dq_pin)


def make_fieldnames() -> list[str]:
    fields = [
        "time",
        "cmd_vx",
        "cmd_vy",
        "cmd_yaw",
        "base_x",
        "base_y",
        "base_z",
        "com_x",
        "com_y",
        "com_z",
        "base_vx",
        "base_vy",
        "base_vz",
        "roll",
        "pitch",
        "yaw",
    ]
    fields.extend([f"joint_pos_{i}" for i in range(1, 13)])
    fields.extend([f"joint_vel_{i}" for i in range(1, 13)])
    fields.extend([f"joint_torque_{i}" for i in range(1, 13)])
    fields.extend([f"foot_contact_{name}" for name in FOOT_NAMES])
    fields.extend([f"foot_force_{name}" for name in FOOT_NAMES])
    fields.extend(["external_push_flag", "fall_flag", "success_flag", "distance_traveled", "robot_mass", "sim_dt"])
    return fields


def get_robot_mass(model: Any) -> float:
    return float(np.sum(model.body_mass[1:]))


def get_foot_contact_forces(mj: Any, model: Any, data: Any, foot_geom_ids: dict[str, int]) -> tuple[dict[str, int], dict[str, float]]:
    contacts = {name: 0 for name in FOOT_NAMES}
    forces = {name: 0.0 for name in FOOT_NAMES}
    force_buffer = np.zeros(6, dtype=np.float64)
    for idx in range(data.ncon):
        contact = data.contact[idx]
        for foot_name, geom_id in foot_geom_ids.items():
            if contact.geom1 == geom_id or contact.geom2 == geom_id:
                mj.mj_contactForce(model, data, idx, force_buffer)
                contacts[foot_name] = 1
                forces[foot_name] += float(np.linalg.norm(force_buffer[:3]))
    return contacts, forces


def build_row(
    mj: Any,
    model: Any,
    data: Any,
    command: np.ndarray,
    torques: np.ndarray,
    foot_geom_ids: dict[str, int],
    base_body_id: int,
    start_xy: np.ndarray,
    push_flag: int,
    fall_flag: int,
    robot_mass: float,
) -> dict[str, float | int]:
    quat = np.array(data.qpos[3:7], dtype=np.float64)
    roll, pitch, yaw = quat_to_rpy(quat)
    base_pos = np.array(data.qpos[0:3], dtype=np.float64)
    base_vel_local = quat_rotate_inverse(quat, np.array(data.qvel[0:3], dtype=np.float64))
    com = np.array(data.subtree_com[base_body_id], dtype=np.float64) if model.nbody > base_body_id else base_pos
    contacts, forces = get_foot_contact_forces(mj, model, data, foot_geom_ids)
    distance = float(np.linalg.norm(base_pos[:2] - start_xy))

    row: dict[str, float | int] = {
        "time": float(data.time),
        "cmd_vx": float(command[0]),
        "cmd_vy": float(command[1]),
        "cmd_yaw": float(command[2]),
        "base_x": float(base_pos[0]),
        "base_y": float(base_pos[1]),
        "base_z": float(base_pos[2]),
        "com_x": float(com[0]),
        "com_y": float(com[1]),
        "com_z": float(com[2]),
        "base_vx": float(base_vel_local[0]),
        "base_vy": float(base_vel_local[1]),
        "base_vz": float(base_vel_local[2]),
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "external_push_flag": int(push_flag),
        "fall_flag": int(fall_flag),
        "success_flag": 0,
        "distance_traveled": distance,
        "robot_mass": robot_mass,
        "sim_dt": float(model.opt.timestep),
    }

    joint_pos = np.array(data.qpos[7:19], dtype=np.float64)
    joint_vel = np.array(data.qvel[6:18], dtype=np.float64)
    for i in range(12):
        row[f"joint_pos_{i + 1}"] = float(joint_pos[i])
        row[f"joint_vel_{i + 1}"] = float(joint_vel[i])
        row[f"joint_torque_{i + 1}"] = float(torques[i]) if np.isfinite(torques[i]) else float("nan")
    for foot_name in FOOT_NAMES:
        row[f"foot_contact_{foot_name}"] = contacts[foot_name]
        row[f"foot_force_{foot_name}"] = forces[foot_name]
    return row


def get_named_ids(mj: Any, model: Any) -> tuple[dict[str, int], list[int], int]:
    foot_geom_ids: dict[str, int] = {}
    for name in FOOT_NAMES:
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise RuntimeError(f"Could not find foot geom named {name}")
        foot_geom_ids[name] = geom_id

    actuator_ids = []
    for name in ACTUATOR_NAMES:
        actuator_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id < 0:
            raise RuntimeError(f"Could not find actuator named {name}")
        actuator_ids.append(actuator_id)

    base_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base_link")
    if base_body_id < 0:
        raise RuntimeError("Could not find base body named base_link")
    return foot_geom_ids, actuator_ids, base_body_id


def initialize_state(data: Any, init_mode: str) -> None:
    if init_mode == "baseline_demo":
        data.qpos[0:3] = BASELINE_DEMO_BASE_POS
        data.qpos[3:7] = BASELINE_DEMO_BASE_QUAT_WXYZ
        data.qpos[7:19] = BASELINE_DEMO_JOINT_ANGLES
        data.qvel[:] = 0.0
        return
    if init_mode == "rl_scene":
        data.qpos[7:19] = DEFAULT_JOINT_ANGLES
        data.qvel[:] = 0.0
        return
    raise ValueError(f"Unsupported init_mode: {init_mode}")


def set_joint_torque(data: Any, actuator_ids: list[int], torque_log_order: np.ndarray) -> None:
    data.ctrl[:] = 0.0
    for actuator_id, tau in zip(actuator_ids, torque_log_order):
        data.ctrl[actuator_id] = float(tau)


def compute_control_torque(
    go2: Any,
    gait: Any,
    leg_controller: Any,
    mpc_force_world: np.ndarray,
    time_now_s: float,
) -> np.ndarray:
    tau_raw = np.zeros(12, dtype=np.float64)
    for leg in FOOT_NAMES:
        leg_slice = LEG_SLICES[leg]
        output = leg_controller.compute_leg_torque(leg, go2, gait, mpc_force_world[leg_slice], time_now_s)
        tau_raw[leg_slice] = output.tau
    return np.clip(tau_raw, -TAU_LIM, TAU_LIM)


def should_push(args: argparse.Namespace, t: float) -> bool:
    return bool(args.push_force != 0.0 and args.push_start <= t < args.push_start + args.push_duration)


def write_rows(rows: list[dict[str, float | int]], args: argparse.Namespace) -> Path:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    vx_token = str(args.cmd_vx).replace("-", "m").replace(".", "p")
    push_token = f"_push{int(args.push_force)}N" if args.push_force else ""
    out_path = args.out_dir / f"{args.scene}_vx{vx_token}_trial{args.trial:02d}{push_token}.csv"
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=make_fieldnames())
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {out_path}")
    return out_path


def run_rollout(args: argparse.Namespace) -> Path:
    baseline_repo = resolve_path(args.baseline_repo)
    modules = load_baseline_modules(baseline_repo)
    mj = modules["mj"]
    PinGo2Model = modules["PinGo2Model"]
    ComTraj = modules["ComTraj"]
    CentroidalMPC = modules["CentroidalMPC"]
    LegController = modules["LegController"]
    Gait = modules["Gait"]

    scene_file = ROBOT_DATA_DIR / ("scene.xml" if args.scene == "flat" else "scene_terrain.xml")
    if not scene_file.exists():
        raise FileNotFoundError(scene_file)

    model = mj.MjModel.from_xml_path(str(scene_file))
    model.opt.timestep = SIM_DT
    data = mj.MjData(model)
    initialize_state(data, args.init_mode)
    mj.mj_forward(model, data)

    foot_geom_ids, actuator_ids, base_body_id = get_named_ids(mj, model)
    robot_mass = get_robot_mass(model)
    command = np.array([args.cmd_vx, args.cmd_vy, args.cmd_yaw], dtype=np.float64)
    start_xy = np.array(data.qpos[0:2], dtype=np.float64)

    go2 = PinGo2Model()
    update_pin_from_mujoco(go2, data)
    gait = Gait(GAIT_HZ, GAIT_DUTY)
    leg_controller = LegController()
    traj = ComTraj(go2)
    traj.generate_traj(go2, gait, 0.0, args.cmd_vx, args.cmd_vy, Z_POS_DES_BODY, args.cmd_yaw, time_step=MPC_DT)
    mpc = CentroidalMPC(go2, traj)
    u_opt = np.zeros((12, traj.N), dtype=float)

    rows: list[dict[str, float | int]] = []
    any_fall = False
    controller_failed = False
    controller_error = ""
    sim_tau_hold = np.zeros(12, dtype=np.float64)
    log_tau_hold = np.full(12, np.nan, dtype=np.float64)
    ctrl_i = 0
    sim_steps = int(round(args.duration * SIM_HZ))
    push_accum = False

    def control_update(time_now_s: float) -> None:
        nonlocal u_opt, sim_tau_hold, log_tau_hold, ctrl_i
        update_pin_from_mujoco(go2, data)
        if (ctrl_i % STEPS_PER_MPC) == 0:
            traj.generate_traj(
                go2,
                gait,
                time_now_s,
                args.cmd_vx,
                args.cmd_vy,
                Z_POS_DES_BODY,
                args.cmd_yaw,
                time_step=MPC_DT,
            )
            sol = mpc.solve_QP(go2, traj, False)
            n_horizon = traj.N
            w_opt = sol["x"].full().flatten()
            u_opt = w_opt[12 * n_horizon :].reshape((12, n_horizon), order="F")
        sim_tau_hold = compute_control_torque(go2, gait, leg_controller, u_opt[:, 0], time_now_s)
        log_tau_hold = sim_tau_hold.copy()
        ctrl_i += 1

    def sim_loop(viewer: Any | None = None) -> None:
        nonlocal any_fall, controller_failed, controller_error, push_accum
        for k in range(sim_steps):
            time_now_s = float(data.time)
            logged_this_step = False
            if (k % CTRL_DECIM) == 0 and not controller_failed:
                try:
                    control_update(time_now_s)
                except Exception as exc:  # noqa: BLE001
                    controller_failed = True
                    controller_error = f"{type(exc).__name__}: {exc}"
                    any_fall = True
                    print(f"ERROR: controller failed at t={time_now_s:.4f}s: {controller_error}", file=sys.stderr)

            push_flag = should_push(args, time_now_s)
            push_accum = push_accum or push_flag
            data.xfrc_applied[:, :] = 0.0
            if push_flag:
                data.xfrc_applied[base_body_id, 1] = args.push_force

            set_joint_torque(data, actuator_ids, sim_tau_hold)
            mj.mj_step(model, data)

            if ((k + 1) % CTRL_DECIM) == 0 or k == sim_steps - 1:
                roll, pitch, _ = quat_to_rpy(np.array(data.qpos[3:7], dtype=np.float64))
                fall_flag = int(
                    data.qpos[2] < args.min_base_height
                    or abs(roll) > args.max_abs_roll_pitch
                    or abs(pitch) > args.max_abs_roll_pitch
                    or controller_failed
                )
                any_fall = any_fall or bool(fall_flag)
                rows.append(
                    build_row(
                        mj,
                        model,
                        data,
                        command,
                        log_tau_hold,
                        foot_geom_ids,
                        base_body_id,
                        start_xy,
                        int(push_accum),
                        fall_flag,
                        robot_mass,
                    )
                )
                push_accum = False
                logged_this_step = True

            if viewer is not None:
                viewer.cam.lookat = np.array(data.qpos[0:3], dtype=np.float64).tolist()
                viewer.sync()
                time.sleep(max(0.0, SIM_DT))
                if not viewer.is_running():
                    break

            if controller_failed:
                if not logged_this_step:
                    rows.append(
                        build_row(
                            mj,
                            model,
                            data,
                            command,
                            log_tau_hold,
                            foot_geom_ids,
                            base_body_id,
                            start_xy,
                            int(push_accum),
                            1,
                            robot_mass,
                        )
                    )
                    push_accum = False
                break

    if args.headless:
        sim_loop()
    else:
        try:
            import mujoco.viewer as mj_viewer  # noqa: PLC0415
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError("mujoco.viewer is not available; rerun with --headless") from exc
        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 5.0
            viewer.cam.azimuth = 50
            viewer.cam.elevation = -20
            sim_loop(viewer)

    if not rows:
        roll, pitch, _ = quat_to_rpy(np.array(data.qpos[3:7], dtype=np.float64))
        rows.append(
            build_row(
                mj,
                model,
                data,
                command,
                log_tau_hold,
                foot_geom_ids,
                base_body_id,
                start_xy,
                0,
                int(data.qpos[2] < args.min_base_height or abs(roll) > args.max_abs_roll_pitch or abs(pitch) > args.max_abs_roll_pitch),
                robot_mass,
            )
        )

    final_row = rows[-1]
    final_success = int(
        not any_fall
        and final_row["base_z"] >= args.min_base_height
        and abs(float(final_row["roll"])) <= args.max_abs_roll_pitch
        and abs(float(final_row["pitch"])) <= args.max_abs_roll_pitch
        and not controller_failed
    )
    for row in rows:
        row["success_flag"] = final_success

    out_path = write_rows(rows, args)
    if controller_failed:
        error_path = out_path.with_suffix(".error.txt")
        error_path.write_text(controller_error + "\n", encoding="utf-8")
        raise RuntimeError(f"Controller failed; partial CSV written to {out_path}; details in {error_path}")
    return out_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", choices=["flat", "terrain"], required=True)
    parser.add_argument("--cmd_vx", type=float, required=True)
    parser.add_argument("--cmd_vy", type=float, default=0.0)
    parser.add_argument("--cmd_yaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--trial", type=int, required=True)
    parser.add_argument("--out_dir", type=Path, required=True)
    parser.add_argument("--push_force", type=float, default=0.0)
    parser.add_argument("--push_start", type=float, default=5.0)
    parser.add_argument("--push_duration", type=float, default=0.1)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--baseline_repo", type=Path, default=DEFAULT_BASELINE_REPO)
    parser.add_argument("--init_mode", choices=["baseline_demo", "rl_scene"], default="baseline_demo")
    parser.add_argument("--min_base_height", type=float, default=0.18)
    parser.add_argument("--max_abs_roll_pitch", type=float, default=0.8)
    return parser.parse_args()


def main() -> int:
    os.environ.setdefault("PYTHONNOUSERSITE", "1")
    args = parse_args()
    np.random.seed(args.trial)
    run_rollout(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
