#!/usr/bin/env python3
import argparse
import csv
import math
import select
import signal
import sys
import time
import warnings
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOG_DIR = PROJECT_ROOT / "real_robot_outputs" / "logs" / "side_logger"
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"

TOPICS = {
    "leg_control_data": "leg_control_data_lcmt",
    "state_estimator_data": "state_estimator_lcmt",
    "rc_command": "rc_command_lcmt",
    "rc_command_data": "rc_command_lcmt",
    "pd_plustau_targets": "pd_tau_targets_lcmt",
}


def _nan():
    return math.nan


def _safe_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _as_list(value, length):
    if value is None:
        return [math.nan] * length
    out = list(value)
    if len(out) < length:
        out += [math.nan] * (length - len(out))
    return out[:length]


def _truthy_int(value):
    return int(bool(value))


class SideLogger:
    def __init__(self, args):
        self.args = args
        self.rows = []
        self.loop_index = 0
        self.start_wall = time.time()
        self.start_monotonic = time.monotonic()
        self.stop_requested = False
        self.topic_seen = {topic: 0 for topic in TOPICS}
        self.decode_warnings = set()

        self.leg = None
        self.state = None
        self.rc = None
        self.rc_topic = ""
        self.pd = None

        self.ctrlmode_left = 0
        self.ctrlmode_right = 0
        self.prev_left_upper_switch = 0
        self.prev_right_upper_switch = 0

    def install_signal_handlers(self):
        def _handler(_signum, _frame):
            self.stop_requested = True

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)

    def handle_leg(self, channel, data, msg_type):
        try:
            self.leg = msg_type.decode(data)
            self.topic_seen[channel] += 1
        except Exception as exc:
            self._warn_decode_once(channel, exc)

    def handle_state(self, channel, data, msg_type):
        try:
            self.state = msg_type.decode(data)
            self.topic_seen[channel] += 1
            self.append_row()
        except Exception as exc:
            self._warn_decode_once(channel, exc)

    def handle_rc(self, channel, data, msg_type):
        try:
            self.rc = msg_type.decode(data)
            self.rc_topic = channel
            self.topic_seen[channel] += 1
            self._update_remote_mode_state()
        except Exception as exc:
            self._warn_decode_once(channel, exc)

    def handle_pd(self, channel, data, msg_type):
        try:
            self.pd = msg_type.decode(data)
            self.topic_seen[channel] += 1
        except Exception as exc:
            self._warn_decode_once(channel, exc)

    def _warn_decode_once(self, channel, exc):
        if channel in self.decode_warnings:
            return
        self.decode_warnings.add(channel)
        warnings.warn(f"Failed to decode LCM topic {channel}: {exc}")

    def _update_remote_mode_state(self):
        if self.rc is None:
            return
        left_upper = _truthy_int(self.rc.left_upper_switch)
        right_upper = _truthy_int(self.rc.right_upper_switch)
        if left_upper and not self.prev_left_upper_switch:
            self.ctrlmode_left = (self.ctrlmode_left + 1) % 3
        if right_upper and not self.prev_right_upper_switch:
            self.ctrlmode_right = (self.ctrlmode_right + 1) % 3
        self.prev_left_upper_switch = left_upper
        self.prev_right_upper_switch = right_upper

    def _estimated_policy_command(self):
        if self.rc is None:
            return math.nan, math.nan, math.nan, "missing_rc_command"

        left_stick = _as_list(self.rc.left_stick, 2)
        right_stick = _as_list(self.rc.right_stick, 2)
        cmd_vx = left_stick[1] * self.args.x_scale
        cmd_vy = 0.0
        cmd_yaw = -right_stick[0] * self.args.yaw_scale

        if self.ctrlmode_left == 1:
            cmd_vy = 0.6 * left_stick[0] * self.args.y_scale

        return cmd_vx, cmd_vy, cmd_yaw, "estimated_from_rc_command"

    def _base_row(self):
        wall = time.time()
        cmd_vx, cmd_vy, cmd_yaw, cmd_source = self._estimated_policy_command()
        return {
            "time": wall - self.start_wall,
            "wall_time": wall,
            "loop_index": self.loop_index,
            "trial_name": self.args.trial_name,
            "experiment_stage": self.args.experiment_stage,
            "command_mode": "side_logger_remote_estimate",
            "logger_type": "lcm_side_logger",
            "cmd_source": cmd_source,
            "cmd_vx": cmd_vx,
            "cmd_vy": cmd_vy,
            "cmd_yaw": cmd_yaw,
            "video_filename_suggested": self.args.video_filename,
            "metadata_path": self.args.metadata_path or "",
            "estop_flag": math.nan,
            "fall_flag": math.nan,
            "manual_stop_flag": math.nan,
            "notes": "",
        }

    def append_row(self):
        row = self._base_row()
        self._add_rc_fields(row)
        self._add_state_fields(row)
        self._add_leg_fields(row)
        self._add_pd_fields(row)
        self.rows.append(row)
        self.loop_index += 1

    def _add_rc_fields(self, row):
        rc = self.rc
        left = _as_list(getattr(rc, "left_stick", None), 2)
        right = _as_list(getattr(rc, "right_stick", None), 2)
        knobs = _as_list(getattr(rc, "knobs", None), 2)
        fields = {
            "rc_topic": self.rc_topic,
            "rc_mode": getattr(rc, "mode", math.nan),
            "rc_left_stick_x": left[0],
            "rc_left_stick_y": left[1],
            "rc_right_stick_x": right[0],
            "rc_right_stick_y": right[1],
            "rc_knob_1": knobs[0],
            "rc_knob_2": knobs[1],
            "rc_left_upper_switch": getattr(rc, "left_upper_switch", math.nan),
            "rc_left_lower_left_switch": getattr(rc, "left_lower_left_switch", math.nan),
            "rc_left_lower_right_switch": getattr(rc, "left_lower_right_switch", math.nan),
            "rc_right_upper_switch": getattr(rc, "right_upper_switch", math.nan),
            "rc_right_lower_left_switch": getattr(rc, "right_lower_left_switch", math.nan),
            "rc_right_lower_right_switch": getattr(rc, "right_lower_right_switch", math.nan),
            "rc_ctrlmode_left": self.ctrlmode_left,
            "rc_ctrlmode_right": self.ctrlmode_right,
        }
        row.update(fields)
        row.update({
            "remote_mode": fields["rc_mode"],
            "remote_left_stick_x": fields["rc_left_stick_x"],
            "remote_left_stick_y": fields["rc_left_stick_y"],
            "remote_right_stick_x": fields["rc_right_stick_x"],
            "remote_right_stick_y": fields["rc_right_stick_y"],
            "remote_left_upper_switch": fields["rc_left_upper_switch"],
            "remote_left_lower_left_switch": fields["rc_left_lower_left_switch"],
            "remote_left_lower_right_switch": fields["rc_left_lower_right_switch"],
            "remote_right_upper_switch": fields["rc_right_upper_switch"],
            "remote_right_lower_left_switch": fields["rc_right_lower_left_switch"],
            "remote_right_lower_right_switch": fields["rc_right_lower_right_switch"],
        })

    def _add_state_fields(self, row):
        state = self.state
        p = _as_list(getattr(state, "p", None), 3)
        v_body = _as_list(getattr(state, "vBody", None), 3)
        rpy = _as_list(getattr(state, "rpy", None), 3)
        omega_body = _as_list(getattr(state, "omegaBody", None), 3)
        contact = _as_list(getattr(state, "contact_estimate", None), 4)
        row.update({
            "base_vx": v_body[0],
            "base_vy": v_body[1],
            "base_yaw_rate": omega_body[2],
            "base_height": p[2],
            "roll": rpy[0],
            "pitch": rpy[1],
            "yaw": rpy[2],
            "angular_vel_x": omega_body[0],
            "angular_vel_y": omega_body[1],
            "angular_vel_z": omega_body[2],
            "state_timestamp_us": getattr(state, "timestamp_us", math.nan),
        })
        for idx, value in enumerate(contact, start=1):
            row[f"foot_force_{idx}"] = value
            row[f"foot_contact_{idx}"] = int(_safe_float(value) > 200.0) if math.isfinite(_safe_float(value)) else math.nan

    def _add_leg_fields(self, row):
        leg = self.leg
        q = _as_list(getattr(leg, "q", None), 12)
        qd = _as_list(getattr(leg, "qd", None), 12)
        tau = _as_list(getattr(leg, "tau_est", None), 12)
        row["leg_timestamp_us"] = getattr(leg, "timestamp_us", math.nan)
        for idx in range(12):
            row[f"joint_pos_{idx + 1}"] = q[idx]
            row[f"joint_vel_{idx + 1}"] = qd[idx]
            row[f"joint_torque_{idx + 1}"] = tau[idx]

    def _add_pd_fields(self, row):
        pd = self.pd
        q_des = _as_list(getattr(pd, "q_des", None), 12)
        qd_des = _as_list(getattr(pd, "qd_des", None), 12)
        tau_ff = _as_list(getattr(pd, "tau_ff", None), 12)
        kp = _as_list(getattr(pd, "kp", None), 12)
        kd = _as_list(getattr(pd, "kd", None), 12)
        contact = _as_list(getattr(pd, "se_contactState", None), 4)
        actual_q = _as_list(getattr(self.leg, "q", None), 12)
        row["pd_target_timestamp_us"] = getattr(pd, "timestamp_us", math.nan)
        for idx in range(12):
            row[f"target_joint_pos_{idx + 1}"] = q_des[idx]
            row[f"target_joint_vel_{idx + 1}"] = qd_des[idx]
            row[f"target_tau_ff_{idx + 1}"] = tau_ff[idx]
            row[f"pd_kp_{idx + 1}"] = kp[idx]
            row[f"pd_kd_{idx + 1}"] = kd[idx]
            if math.isfinite(_safe_float(q_des[idx])) and math.isfinite(_safe_float(actual_q[idx])):
                row[f"joint_tracking_error_{idx + 1}"] = q_des[idx] - actual_q[idx]
            else:
                row[f"joint_tracking_error_{idx + 1}"] = math.nan
        for idx, value in enumerate(contact, start=1):
            row[f"target_contact_{idx}"] = value

    def write_csv(self):
        log_dir = Path(self.args.log_dir).expanduser().resolve()
        log_dir.mkdir(parents=True, exist_ok=True)
        csv_path = log_dir / f"{self.args.trial_name}.csv"
        fieldnames = self._fieldnames()
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"Side logger CSV saved: {csv_path}")
        print(f"Rows saved: {len(self.rows)}")
        self._print_missing_topic_warnings()
        return csv_path

    def _print_missing_topic_warnings(self):
        for topic, count in self.topic_seen.items():
            if count == 0:
                warnings.warn(f"No messages received on topic '{topic}'. Related CSV fields are NaN.")

    @staticmethod
    def _fieldnames():
        names = [
            "time", "wall_time", "loop_index", "trial_name", "experiment_stage",
            "command_mode", "logger_type", "cmd_source", "cmd_vx", "cmd_vy", "cmd_yaw",
            "video_filename_suggested", "metadata_path",
            "estop_flag", "fall_flag", "manual_stop_flag", "notes",
            "rc_topic", "rc_mode", "rc_left_stick_x", "rc_left_stick_y",
            "rc_right_stick_x", "rc_right_stick_y", "rc_knob_1", "rc_knob_2",
            "rc_left_upper_switch", "rc_left_lower_left_switch", "rc_left_lower_right_switch",
            "rc_right_upper_switch", "rc_right_lower_left_switch", "rc_right_lower_right_switch",
            "rc_ctrlmode_left", "rc_ctrlmode_right",
            "remote_mode", "remote_left_stick_x", "remote_left_stick_y",
            "remote_right_stick_x", "remote_right_stick_y",
            "remote_left_upper_switch", "remote_left_lower_left_switch", "remote_left_lower_right_switch",
            "remote_right_upper_switch", "remote_right_lower_left_switch", "remote_right_lower_right_switch",
            "base_vx", "base_vy", "base_yaw_rate", "base_height",
            "roll", "pitch", "yaw", "angular_vel_x", "angular_vel_y", "angular_vel_z",
            "state_timestamp_us", "leg_timestamp_us", "pd_target_timestamp_us",
        ]
        names += [f"foot_force_{idx}" for idx in range(1, 5)]
        names += [f"foot_contact_{idx}" for idx in range(1, 5)]
        names += [f"joint_pos_{idx}" for idx in range(1, 13)]
        names += [f"joint_vel_{idx}" for idx in range(1, 13)]
        names += [f"joint_torque_{idx}" for idx in range(1, 13)]
        names += [f"target_joint_pos_{idx}" for idx in range(1, 13)]
        names += [f"target_joint_vel_{idx}" for idx in range(1, 13)]
        names += [f"target_tau_ff_{idx}" for idx in range(1, 13)]
        names += [f"pd_kp_{idx}" for idx in range(1, 13)]
        names += [f"pd_kd_{idx}" for idx in range(1, 13)]
        names += [f"joint_tracking_error_{idx}" for idx in range(1, 13)]
        names += [f"target_contact_{idx}" for idx in range(1, 5)]
        return names


def _load_runtime_types():
    import lcm
    from go2_gym_deploy.lcm_types.leg_control_data_lcmt import leg_control_data_lcmt
    from go2_gym_deploy.lcm_types.pd_tau_targets_lcmt import pd_tau_targets_lcmt
    from go2_gym_deploy.lcm_types.rc_command_lcmt import rc_command_lcmt
    from go2_gym_deploy.lcm_types.state_estimator_lcmt import state_estimator_lcmt

    return lcm, {
        "leg_control_data": leg_control_data_lcmt,
        "state_estimator_data": state_estimator_lcmt,
        "rc_command": rc_command_lcmt,
        "rc_command_data": rc_command_lcmt,
        "pd_plustau_targets": pd_tau_targets_lcmt,
    }


def run(args):
    logger = SideLogger(args)

    if args.dry_run:
        print("Dry run: side logger will only subscribe and write CSV; it will not publish control commands.")
        print(f"LCM URL: {args.lcm_url}")
        print("Topics:")
        for topic in TOPICS:
            print(f"  - {topic}")
        print(f"Default log_dir: {Path(args.log_dir).expanduser().resolve()}")
        print(f"Trial CSV: {Path(args.log_dir).expanduser().resolve() / (args.trial_name + '.csv')}")
        return None

    lcm_module, msg_types = _load_runtime_types()
    lc = lcm_module.LCM(args.lcm_url)
    lc.subscribe("leg_control_data", lambda c, d: logger.handle_leg(c, d, msg_types[c]))
    lc.subscribe("state_estimator_data", lambda c, d: logger.handle_state(c, d, msg_types[c]))
    lc.subscribe("rc_command", lambda c, d: logger.handle_rc(c, d, msg_types[c]))
    lc.subscribe("rc_command_data", lambda c, d: logger.handle_rc(c, d, msg_types[c]))
    lc.subscribe("pd_plustau_targets", lambda c, d: logger.handle_pd(c, d, msg_types[c]))

    logger.install_signal_handlers()
    duration = max(0.0, float(args.duration))
    end_time = time.monotonic() + duration if duration > 0 else None

    print("LCM side logger started. It subscribes only and sends no control commands.")
    print(f"trial_name={args.trial_name}, stage={args.experiment_stage}, duration={duration if duration > 0 else 'until Ctrl+C'}")
    try:
        while not logger.stop_requested:
            if end_time is not None:
                remaining = end_time - time.monotonic()
                if remaining <= 0:
                    break
                timeout = min(0.2, max(0.0, remaining))
            else:
                timeout = 0.2

            ready, _, _ = select.select([lc.fileno()], [], [], timeout)
            if ready:
                lc.handle()
    finally:
        return logger.write_csv()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Passive LCM side logger for real Unitree Go2 deployment.")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to record. Use 0 to run until Ctrl+C.")
    parser.add_argument("--trial_name", default="real_trial_side_logger")
    parser.add_argument("--experiment_stage", default="remote_control_walk")
    parser.add_argument("--log_dir", default=str(DEFAULT_LOG_DIR))
    parser.add_argument("--video_filename", default="")
    parser.add_argument("--metadata_path", default="")
    parser.add_argument("--lcm_url", default=DEFAULT_LCM_URL)
    parser.add_argument("--dry_run", action="store_true")
    parser.add_argument("--x_scale", type=float, default=0.7,
                        help="Side-estimate scale for rc left_stick_y to cmd_vx; deploy_policy.py default is 0.7.")
    parser.add_argument("--y_scale", type=float, default=1.0,
                        help="Side-estimate scale for lateral command when the left remote mode is lat_vel.")
    parser.add_argument("--yaw_scale", type=float, default=1.0,
                        help="Side-estimate scale for rc right_stick_x to cmd_yaw.")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    run(args)


if __name__ == "__main__":
    main(sys.argv[1:])
