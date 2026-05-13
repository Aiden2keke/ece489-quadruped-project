#!/usr/bin/env python3
"""Export Go2 RL/PPO configuration summaries for the course report.

This script reads the local legged_gym/rsl_rl config classes directly. It does
not invent training or evaluation results.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MD = REPO_ROOT / "project_outputs" / "rl_config_summary.md"
DEFAULT_CSV = REPO_ROOT / "project_outputs" / "rl_config_summary.csv"


def add_repo_paths() -> None:
    sys.path.insert(0, str(REPO_ROOT / "legged_gym"))
    sys.path.insert(0, str(REPO_ROOT / "rsl_rl"))


def load_module(module_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load {module_name} from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def load_go2_config_module():
    """Load config files without importing legged_gym.envs.__init__.

    The package __init__ imports Isaac Gym task classes, which is unnecessary for
    this offline report exporter and may fail on machines without Isaac Gym.
    """

    add_repo_paths()
    import legged_gym  # noqa: PLC0415

    envs_pkg = types.ModuleType("legged_gym.envs")
    envs_pkg.__path__ = [str(REPO_ROOT / "legged_gym" / "legged_gym" / "envs")]
    sys.modules.setdefault("legged_gym.envs", envs_pkg)

    base_pkg = types.ModuleType("legged_gym.envs.base")
    base_pkg.__path__ = [str(REPO_ROOT / "legged_gym" / "legged_gym" / "envs" / "base")]
    sys.modules.setdefault("legged_gym.envs.base", base_pkg)

    go2_pkg = types.ModuleType("legged_gym.envs.go2")
    go2_pkg.__path__ = [str(REPO_ROOT / "legged_gym" / "legged_gym" / "envs" / "go2")]
    sys.modules.setdefault("legged_gym.envs.go2", go2_pkg)

    load_module(
        "legged_gym.envs.base.base_config",
        REPO_ROOT / "legged_gym" / "legged_gym" / "envs" / "base" / "base_config.py",
    )
    load_module(
        "legged_gym.envs.base.legged_robot_config",
        REPO_ROOT / "legged_gym" / "legged_gym" / "envs" / "base" / "legged_robot_config.py",
    )
    return load_module(
        "legged_gym.envs.go2.go2_config",
        REPO_ROOT / "legged_gym" / "legged_gym" / "envs" / "go2" / "go2_config.py",
    )


def class_to_dict(obj: Any) -> Any:
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    if isinstance(obj, (list, tuple)):
        return [class_to_dict(x) for x in obj]
    if isinstance(obj, dict):
        return {k: class_to_dict(v) for k, v in obj.items()}
    if not hasattr(obj, "__dict__"):
        return obj
    result = {}
    for key in dir(obj):
        if key.startswith("_"):
            continue
        value = getattr(obj, key)
        if callable(value):
            continue
        result[key] = class_to_dict(value)
    return result


def fmt(value: Any) -> str:
    if isinstance(value, dict):
        return "; ".join(f"{k}: {fmt(v)}" for k, v in value.items())
    if isinstance(value, list):
        return "[" + ", ".join(fmt(v) for v in value) + "]"
    return str(value)


def existing_or_note(path: Path) -> str:
    return f"{path} ({'exists' if path.exists() else 'not found'})"


def go2_urdf_mass() -> float | None:
    urdf_path = REPO_ROOT / "legged_gym" / "resources" / "robots" / "go2" / "urdf" / "go2.urdf"
    if not urdf_path.exists():
        return None
    root = ET.parse(urdf_path).getroot()
    masses = [float(m.attrib["value"]) for m in root.iter("mass") if "value" in m.attrib]
    return sum(masses) if masses else None


def build_rows() -> list[tuple[str, str, str]]:
    go2_config = load_go2_config_module()

    flat_cfg = go2_config.GO2FlatCfg()
    flat_train_cfg = go2_config.GO2FlatCfgPPO()
    rough_cfg = go2_config.GO2RoughCfg()
    rough_train_cfg = go2_config.GO2RoughCfgPPO()

    env = class_to_dict(flat_cfg.env)
    control = class_to_dict(flat_cfg.control)
    terrain_flat = class_to_dict(flat_cfg.terrain)
    terrain_rough = class_to_dict(rough_cfg.terrain)
    domain_rand = class_to_dict(flat_cfg.domain_rand)
    rewards = class_to_dict(flat_cfg.rewards)
    ppo = class_to_dict(flat_train_cfg.algorithm)
    policy = class_to_dict(flat_train_cfg.policy)
    runner = class_to_dict(flat_train_cfg.runner)

    sim_dt = flat_cfg.sim.dt
    policy_dt = sim_dt * flat_cfg.control.decimation
    checkpoint_path = REPO_ROOT / "legged_gym" / "logs" / "rough_go2" / "TS_re3" / "model_9000.pt"
    exported_policy_path = (
        REPO_ROOT / "legged_gym" / "logs" / "rough_go2" / "exported" / "policies" / "policy_1.pt"
    )
    go2_mass = go2_urdf_mass()
    critic_obs_dim = env["num_privileged_obs"]
    obs_dim = env["num_observations"]
    latent_dim = policy.get("encoder_latent_dim", 32)

    rows = [
        ("Project", "robot model", "Unitree Go2"),
        ("Project", "robot mass from URDF", f"{go2_mass:.3f} kg" if go2_mass else "not found"),
        ("Project", "simulator/training framework", "Isaac Gym + legged_gym + rsl_rl PPO; MuJoCo used for scripted rollouts/evaluation"),
        ("Project", "task name", "go2 or go2_flat for flat; go2_rough or go2_challenging for rough terrain"),
        ("Environment", "num_envs", fmt(env["num_envs"])),
        ("Environment", "observation dimension", fmt(obs_dim)),
        (
            "Environment",
            "observation components",
            "base angular velocity (3), projected gravity/body orientation (3), velocity command vx/vy/yaw (3), "
            "joint position offsets (12), joint velocities (12), previous actions (12)",
        ),
        ("Environment", "privileged critic observation dimension", fmt(critic_obs_dim)),
        ("Environment", "observation history length", fmt(env["obs_history_length"])),
        ("Environment", "action dimension", fmt(env["num_actions"])),
        ("Environment", "action definition", "desired joint positions: q_des = default_joint_angle + action_scale * action, tracked by joint PD"),
        ("Control", "PD gains", f"stiffness={fmt(control['stiffness'])}; damping={fmt(control['damping'])}"),
        ("Control", "action scale", fmt(control["action_scale"])),
        ("Control", "control decimation", fmt(control["decimation"])),
        ("Control", "sim dt", fmt(sim_dt)),
        ("Control", "policy/control dt", fmt(policy_dt)),
        ("Environment", "episode length", f"{flat_cfg.env.episode_length_s} s"),
        ("PPO", "runner", fmt(runner)),
        ("PPO", "hyperparameters", fmt(ppo)),
        (
            "Network",
            "actor architecture",
            f"input {obs_dim}+{latent_dim}; hidden {fmt(policy['actor_hidden_dims'])}; output 12",
        ),
        (
            "Network",
            "critic architecture",
            f"input {critic_obs_dim}+{latent_dim}; hidden {fmt(policy['critic_hidden_dims'])}; output 1",
        ),
        (
            "Network",
            "proprioceptive encoder",
            f"input {obs_dim}*{env['obs_history_length']}; hidden {fmt(policy.get('encoder_hidden_dims', [512, 256, 128]))}; latent {latent_dim}",
        ),
        (
            "Network",
            "privileged encoder",
            f"input {critic_obs_dim}; hidden {fmt(policy.get('encoder_hidden_dims', [512, 256, 128]))}; latent {latent_dim}",
        ),
        ("Rewards", "reward terms and weights", fmt(rewards["scales"])),
        ("Rewards", "tracking sigma", fmt(rewards["tracking_sigma"])),
        ("Rewards", "base height target", fmt(rewards["base_height_target"])),
        ("Rewards", "foot clearance target", fmt(rewards.get("foot_clearance_target", "not configured"))),
        ("Domain randomization", "friction range", fmt(domain_rand["friction_range"])),
        ("Domain randomization", "base mass added range", fmt(domain_rand["added_mass_range"])),
        ("Domain randomization", "motor strength range", fmt(domain_rand["motor_strength_range"])),
        (
            "Domain randomization",
            "push randomization",
            f"enabled={domain_rand['push_robots']}; interval_s={domain_rand['push_interval_s']}; max_push_vel_xy={domain_rand['max_push_vel_xy']}",
        ),
        (
            "Domain randomization",
            "Kp/Kd randomization",
            f"Kp enabled={domain_rand['randomize_Kp_factor']} range={fmt(domain_rand['Kp_factor_range'])}; "
            f"Kd enabled={domain_rand['randomize_Kd_factor']} range={fmt(domain_rand['Kd_factor_range'])}",
        ),
        (
            "Domain randomization",
            "action delay",
            f"enabled={domain_rand['randomize_action_delay']}; delay_ms_range={fmt(domain_rand['delay_ms_range'])}",
        ),
        ("Terrain", "flat settings", fmt(terrain_flat)),
        ("Terrain", "challenging terrain settings", fmt(terrain_rough)),
        ("Artifacts", "checkpoint path", existing_or_note(checkpoint_path)),
        ("Artifacts", "exported policy path", existing_or_note(exported_policy_path)),
        (
            "Notes",
            "terrain status",
            "Current go2/go2_flat keeps flat training/play. go2_rough/go2_challenging enables legged_gym trimesh rough terrain.",
        ),
        (
            "Notes",
            "MuJoCo evaluation scenes",
            "mujoco_test/data/go2/scene.xml for flat; mujoco_test/data/go2/scene_terrain.xml for terrain.",
        ),
    ]
    return rows


def write_markdown(rows: list[tuple[str, str, str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# RL Configuration Summary",
        "",
        "Generated from local config/code. This file records configuration and artifact paths only; it does not contain fabricated training or evaluation results.",
        "",
        "| Section | Item | Value |",
        "| --- | --- | --- |",
    ]
    for section, key, value in rows:
        safe_value = value.replace("|", "\\|")
        lines.append(f"| {section} | {key} | {safe_value} |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv(rows: list[tuple[str, str, str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["section", "item", "value"])
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--md_out", type=Path, default=DEFAULT_MD)
    parser.add_argument("--csv_out", type=Path, default=DEFAULT_CSV)
    args = parser.parse_args()

    rows = build_rows()
    write_markdown(rows, args.md_out)
    write_csv(rows, args.csv_out)
    print(f"Wrote {args.md_out}")
    print(f"Wrote {args.csv_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
