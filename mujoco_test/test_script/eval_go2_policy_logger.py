#!/usr/bin/env python3
"""Fixed-command MuJoCo evaluation logger for the Go2 RL policy.

Available fields:
- base pose, body-frame base linear velocity, roll/pitch/yaw, joint position/velocity/torque.
- foot contact flags and per-foot contact-force norms from MuJoCo contacts.
- external push flag, fall flag, final success flag, distance traveled.

Unavailable in this MuJoCo setup:
- full 3D foot force vectors in a stable world frame. The script logs a scalar
  force norm per foot as `foot_force_FL`, `foot_force_FR`, `foot_force_RL`,
  `foot_force_RR` instead of fabricating vector components.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import torch

try:
    import mujoco.viewer
except Exception:  # noqa: BLE001
    mujoco.viewer = None


SCRIPT_DIR = Path(__file__).resolve().parent
MUJOCO_ROOT = SCRIPT_DIR.parent
REPO_ROOT = MUJOCO_ROOT.parent
ROBOT_DATA_DIR = MUJOCO_ROOT / "data" / "go2"
DEFAULT_CHECKPOINT = REPO_ROOT / "legged_gym" / "logs" / "rough_go2" / "TS_re3" / "model_9000.pt"
DEFAULT_ENCODER = MUJOCO_ROOT / "model" / "rsl_rl_teacher_student" / "proprio_encoder" / "proprio_oracle_TS_re3.pth"

sys.path.insert(0, str(MUJOCO_ROOT))
from module.modules import Actor, MLPEncoder  # noqa: E402


FOOT_NAMES = ("FL", "FR", "RL", "RR")
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
P_GAINS = np.full(12, 20.0, dtype=np.float64)
D_GAINS = np.full(12, 0.5, dtype=np.float64)
TORQUE_LIMITS = np.array([20.0, 55.0, 55.0, 20.0, 55.0, 55.0, 20.0, 55.0, 55.0, 20.0, 55.0, 55.0])
ACTION_SCALE = 0.25
BODY_ANG_VEL_SCALE = 0.25
COMMAND_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float64)
JOINT_VEL_SCALE = 0.05


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


def safe_torch_load(path: Path, device: torch.device):
    try:
        return torch.load(path, map_location=device, weights_only=False)
    except TypeError:
        return torch.load(path, map_location=device)


def extract_prefixed_state(state_dict: dict[str, torch.Tensor], prefix: str, strip_to: str | None = None) -> dict[str, torch.Tensor]:
    result = {}
    for key, value in state_dict.items():
        if not key.startswith(prefix):
            continue
        if strip_to is None:
            new_key = key
        else:
            new_key = key.replace(prefix, strip_to, 1)
        result[new_key] = value
    return result


def load_from_checkpoint(path: Path, device: torch.device) -> tuple[torch.nn.Module, torch.nn.Module]:
    payload = safe_torch_load(path, device)
    state_dict = payload.get("model_state_dict", payload)
    actor_state = extract_prefixed_state(state_dict, "actor.", "actor.")
    encoder_state = extract_prefixed_state(state_dict, "proprioceptive_encoder.encoder.", "encoder.")
    if not actor_state or not encoder_state:
        raise RuntimeError(f"Could not find actor/proprioceptive encoder weights in checkpoint {path}")

    actor = Actor(num_obs=77, num_actions=12, hidden_dims=[512, 256, 128]).to(device)
    actor.load_state_dict(actor_state)
    actor.eval()
    encoder = MLPEncoder(input_dim=45 * 15).to(device)
    encoder.load_state_dict(encoder_state)
    encoder.eval()
    return actor, encoder


def load_from_policy_path(policy_path: Path, encoder_path: Path, device: torch.device) -> tuple[torch.nn.Module, torch.nn.Module]:
    try:
        actor = torch.jit.load(str(policy_path), map_location=device)
        actor.eval()
    except RuntimeError:
        actor = Actor(num_obs=77, num_actions=12, hidden_dims=[512, 256, 128]).to(device)
        actor.load_state_dict(safe_torch_load(policy_path, device))
        actor.eval()

    if not encoder_path.exists():
        raise FileNotFoundError(
            f"Encoder weights are required with --policy_path, but {encoder_path} was not found. "
            "Pass --encoder_path or use --checkpoint."
        )
    encoder = MLPEncoder(input_dim=45 * 15).to(device)
    encoder.load_state_dict(safe_torch_load(encoder_path, device))
    encoder.eval()
    return actor, encoder


def infer_action(
    actor: torch.nn.Module,
    encoder: torch.nn.Module,
    obs_history: torch.Tensor,
    observation: np.ndarray,
    device: torch.device,
) -> tuple[np.ndarray, torch.Tensor]:
    obs = torch.from_numpy(observation).float().to(device).unsqueeze(0)
    obs_history = torch.cat((obs_history[:, 45:], obs), dim=-1)
    with torch.inference_mode():
        latent = encoder(obs_history)
        latent = torch.nn.functional.normalize(latent, p=2, dim=-1)
        actor_input = torch.cat((obs, latent), dim=-1)
        action = actor(actor_input)
    return np.clip(action.detach().cpu().numpy().reshape(-1), -6.0, 6.0), obs_history


def compute_observation(data: mujoco.MjData, command: np.ndarray, last_action: np.ndarray) -> np.ndarray:
    body_ang_vel = np.array(data.qvel[3:6], dtype=np.float64)
    gravity_projection = quat_rotate_inverse(np.array(data.qpos[3:7], dtype=np.float64), np.array([0.0, 0.0, -1.0]))
    joint_pos = np.array(data.qpos[7:19], dtype=np.float64)
    joint_vel = np.array(data.qvel[6:18], dtype=np.float64)
    return np.concatenate(
        [
            body_ang_vel * BODY_ANG_VEL_SCALE,
            gravity_projection,
            command * COMMAND_SCALE,
            joint_pos - DEFAULT_JOINT_ANGLES,
            joint_vel * JOINT_VEL_SCALE,
            last_action,
        ]
    )


def compute_torques(data: mujoco.MjData, action: np.ndarray) -> np.ndarray:
    dof_pos = np.array(data.qpos[7:19], dtype=np.float64)
    dof_vel = np.array(data.qvel[6:18], dtype=np.float64)
    torques = P_GAINS * (action * ACTION_SCALE + DEFAULT_JOINT_ANGLES - dof_pos) - D_GAINS * dof_vel
    return np.clip(torques, -TORQUE_LIMITS, TORQUE_LIMITS)


def get_robot_mass(model: mujoco.MjModel) -> float:
    return float(np.sum(model.body_mass[1:]))


def apply_runtime_perturbations(model: mujoco.MjModel, args: argparse.Namespace) -> None:
    """Apply additional-evaluation perturbations without modifying source XML."""
    if args.ground_friction is not None:
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        if floor_id < 0:
            raise RuntimeError("Could not find ground geom named floor for --ground_friction")
        model.geom_friction[floor_id, 0] = float(args.ground_friction)

    if args.base_mass_scale != 1.0:
        base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if base_body_id < 0:
            raise RuntimeError("Could not find body named base_link for --base_mass_scale")
        model.body_mass[base_body_id] *= float(args.base_mass_scale)
        model.body_inertia[base_body_id] *= float(args.base_mass_scale)


def get_foot_contact_forces(model: mujoco.MjModel, data: mujoco.MjData, foot_geom_ids: dict[str, int]) -> tuple[dict[str, int], dict[str, float]]:
    contacts = {name: 0 for name in FOOT_NAMES}
    forces = {name: 0.0 for name in FOOT_NAMES}
    force_buffer = np.zeros(6, dtype=np.float64)
    for idx in range(data.ncon):
        contact = data.contact[idx]
        for foot_name, geom_id in foot_geom_ids.items():
            if contact.geom1 == geom_id or contact.geom2 == geom_id:
                mujoco.mj_contactForce(model, data, idx, force_buffer)
                contacts[foot_name] = 1
                forces[foot_name] += float(np.linalg.norm(force_buffer[:3]))
    return contacts, forces


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


def build_row(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    command: np.ndarray,
    torques: np.ndarray,
    foot_geom_ids: dict[str, int],
    start_xy: np.ndarray,
    push_flag: int,
    fall_flag: int,
    robot_mass: float,
) -> dict[str, float | int]:
    quat = np.array(data.qpos[3:7], dtype=np.float64)
    roll, pitch, yaw = quat_to_rpy(quat)
    base_pos = np.array(data.qpos[0:3], dtype=np.float64)
    base_vel_local = quat_rotate_inverse(quat, np.array(data.qvel[0:3], dtype=np.float64))
    com = np.array(data.subtree_com[1], dtype=np.float64) if model.nbody > 1 else base_pos
    contacts, forces = get_foot_contact_forces(model, data, foot_geom_ids)
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
        row[f"joint_torque_{i + 1}"] = float(torques[i])
    for foot_name in FOOT_NAMES:
        row[f"foot_contact_{foot_name}"] = contacts[foot_name]
        row[f"foot_force_{foot_name}"] = forces[foot_name]
    return row


def run_rollout(args: argparse.Namespace, actor: torch.nn.Module, encoder: torch.nn.Module, device: torch.device) -> Path:
    scene_file = ROBOT_DATA_DIR / ("scene.xml" if args.scene == "flat" else "scene_terrain.xml")
    if not scene_file.exists():
        raise FileNotFoundError(scene_file)

    model = mujoco.MjModel.from_xml_path(str(scene_file))
    model.opt.timestep = args.sim_dt
    apply_runtime_perturbations(model, args)
    data = mujoco.MjData(model)
    mujoco.mj_setConst(model, data)
    data.qpos[7:19] = DEFAULT_JOINT_ANGLES
    mujoco.mj_forward(model, data)

    foot_geom_ids = {name: model.geom(name).id for name in FOOT_NAMES}
    base_body_id = model.body("base_link").id
    robot_mass = get_robot_mass(model)
    command = np.array([args.cmd_vx, args.cmd_vy, args.cmd_yaw], dtype=np.float64)
    last_action = np.zeros(12, dtype=np.float64)
    action = np.zeros(12, dtype=np.float64)
    torques = np.zeros(12, dtype=np.float64)
    obs_history = torch.zeros((1, 45 * 15), dtype=torch.float32, device=device)
    start_xy = np.array(data.qpos[0:2], dtype=np.float64)
    rows: list[dict[str, float | int]] = []
    any_fall = False

    def step_once(viewer=None) -> None:
        nonlocal action, last_action, torques, obs_history, any_fall
        observation = compute_observation(data, command, last_action)
        action, obs_history = infer_action(actor, encoder, obs_history, observation, device)
        last_action = action.copy()
        for _ in range(args.decimation):
            t = float(data.time)
            push_flag = int(args.push_force != 0.0 and args.push_start <= t < args.push_start + args.push_duration)
            data.xfrc_applied[:, :] = 0.0
            if push_flag:
                data.xfrc_applied[base_body_id, 1] = args.push_force
            torques = args.torque_scale * compute_torques(data, action)
            data.ctrl[:12] = torques
            mujoco.mj_step(model, data)

            roll, pitch, _ = quat_to_rpy(np.array(data.qpos[3:7], dtype=np.float64))
            fall_flag = int(data.qpos[2] < args.min_base_height or abs(roll) > args.max_abs_roll_pitch or abs(pitch) > args.max_abs_roll_pitch)
            any_fall = any_fall or bool(fall_flag)
            rows.append(build_row(model, data, command, torques, foot_geom_ids, start_xy, push_flag, fall_flag, robot_mass))

            if viewer is not None:
                viewer.cam.lookat = np.array(data.qpos[0:3], dtype=np.float64).tolist()
                viewer.sync()
                time.sleep(max(0.0, model.opt.timestep))

    if args.headless:
        while data.time < args.duration:
            step_once()
    else:
        if mujoco.viewer is None:
            raise RuntimeError("mujoco.viewer is not available; rerun with --headless")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 5.0
            viewer.cam.azimuth = 50
            viewer.cam.elevation = -20
            while viewer.is_running() and data.time < args.duration:
                step_once(viewer)

    final_success = int(not any_fall and rows and rows[-1]["base_z"] >= args.min_base_height)
    for row in rows:
        row["success_flag"] = final_success

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", choices=["flat", "terrain"], required=True)
    parser.add_argument("--cmd_vx", type=float, required=True)
    parser.add_argument("--cmd_vy", type=float, default=0.0)
    parser.add_argument("--cmd_yaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--trial", type=int, default=0)
    parser.add_argument("--out_dir", type=Path, default=REPO_ROOT / "project_outputs" / "eval_logs")
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--policy_path", type=Path, default=None)
    parser.add_argument("--encoder_path", type=Path, default=DEFAULT_ENCODER)
    parser.add_argument("--push_force", type=float, default=0.0)
    parser.add_argument("--push_start", type=float, default=5.0)
    parser.add_argument("--push_duration", type=float, default=0.1)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--device", type=str, default="auto")
    parser.add_argument("--sim_dt", type=float, default=0.005)
    parser.add_argument("--decimation", type=int, default=4)
    parser.add_argument("--min_base_height", type=float, default=0.18)
    parser.add_argument("--max_abs_roll_pitch", type=float, default=0.8)
    parser.add_argument("--ground_friction", type=float, default=None)
    parser.add_argument("--base_mass_scale", type=float, default=1.0)
    parser.add_argument("--torque_scale", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    np.random.seed(args.trial)
    torch.manual_seed(args.trial)
    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)

    if args.policy_path is not None:
        actor, encoder = load_from_policy_path(args.policy_path, args.encoder_path, device)
        print(f"Loaded policy from {args.policy_path} and encoder from {args.encoder_path}")
    else:
        checkpoint = args.checkpoint if args.checkpoint is not None else DEFAULT_CHECKPOINT
        if not checkpoint.exists():
            raise FileNotFoundError(
                f"No checkpoint was provided and default checkpoint was not found: {checkpoint}"
            )
        actor, encoder = load_from_checkpoint(checkpoint, device)
        print(f"Loaded checkpoint from {checkpoint}")

    run_rollout(args, actor, encoder, device)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
