#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "${repo_root}/rover/ros2_ws/install/setup.bash" ]]; then
  # Prefer the workspace overlay when this repo has been built locally.
  # shellcheck disable=SC1091
  set +u
  source "${repo_root}/rover/ros2_ws/install/setup.bash"
  set -u
elif [[ -f "${repo_root}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${repo_root}/install/setup.bash"
  set -u
fi

python3 - "$@" <<'PY'
import argparse
import csv
import math
import signal
import sys
import time

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int8
from trajectory_msgs.msg import JointTrajectory


JOINTS = ["arm_j1", "arm_j2", "arm_j3", "arm_j4", "arm_j5", "arm_j6"]
RAD_TO_DEG = 180.0 / math.pi


def now_s():
    return time.monotonic()


def age_s(stamp):
    if stamp is None:
        return float("inf")
    return max(0.0, now_s() - stamp)


def fmt_age(stamp):
    age = age_s(stamp)
    if not math.isfinite(age):
        return "never"
    return f"{age:5.2f}s"


def values_by_joint(names, values):
    out = {name: 0.0 for name in JOINTS}
    for name, value in zip(names, values):
        if name in out:
            out[name] = float(value)
    return out


def ordered_deg(values):
    return [values.get(name, 0.0) * RAD_TO_DEG for name in JOINTS]


def ordered(values):
    return [values.get(name, 0.0) for name in JOINTS]


def deltas_deg(current, baseline):
    if current is None or baseline is None:
        return [float("nan")] * len(JOINTS)
    return [(current.get(name, 0.0) - baseline.get(name, 0.0)) * RAD_TO_DEG for name in JOINTS]


def fmt_vec(values, width=7, precision=2):
    return " ".join(
        "   n/a" if not math.isfinite(value) else f"{value:{width}.{precision}f}"
        for value in values
    )


def twist_values(msg):
    return [
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.linear.z,
        msg.twist.angular.x,
        msg.twist.angular.y,
        msg.twist.angular.z,
    ]


class ServoJogTrace(Node):
    def __init__(self, args):
        super().__init__("manipulator_servo_jog_trace")
        self.args = args
        self.start = now_s()
        self.last_print = 0.0
        self.latest_joint_cmd = None
        self.latest_joint_cmd_time = None
        self.latest_twist_cmd = None
        self.latest_twist_cmd_time = None
        self.latest_traj = None
        self.latest_traj_time = None
        self.traj_baseline = None
        self.latest_state = None
        self.latest_state_time = None
        self.state_baseline = None
        self.latest_status = None
        self.latest_status_time = None
        self.latest_collision_scale = None
        self.latest_collision_scale_time = None
        self.notes_seen = set()
        self.csv_file = None
        self.csv_writer = None

        if args.csv:
            self.csv_file = open(args.csv, "w", newline="", encoding="utf-8")
            fields = (
                ["elapsed_s", "status", "collision_scale", "joint_cmd_age_s", "twist_cmd_age_s"]
                + [f"joint_cmd_{joint}_deg_s" for joint in JOINTS]
                + ["twist_lin_x", "twist_lin_y", "twist_lin_z", "twist_ang_x", "twist_ang_y", "twist_ang_z"]
                + [f"traj_{joint}_deg" for joint in JOINTS]
                + [f"state_{joint}_deg" for joint in JOINTS]
            )
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fields)
            self.csv_writer.writeheader()

        self.create_subscription(JointJog, "/moveit_servo/delta_joint_cmds", self.joint_cmd_cb, 20)
        self.create_subscription(TwistStamped, "/moveit_servo/delta_twist_cmds", self.twist_cmd_cb, 20)
        self.create_subscription(JointTrajectory, "/manipulator_controller/joint_trajectory", self.traj_cb, 20)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, qos_profile_sensor_data)
        self.create_subscription(Int8, "/moveit_servo/status", self.status_cb, 20)
        self.create_subscription(Float64, "/moveit_servo/collision_velocity_scale", self.collision_scale_cb, 20)
        self.timer = self.create_timer(args.period, self.print_sample)

        print("Tracing manipulator Servo jog path. Press Ctrl-C to stop.")
        print("Columns are in degrees or deg/s unless noted.")
        print("")

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()

    def joint_cmd_cb(self, msg):
        self.latest_joint_cmd = values_by_joint(msg.joint_names, msg.velocities)
        self.latest_joint_cmd_time = now_s()

    def twist_cmd_cb(self, msg):
        self.latest_twist_cmd = twist_values(msg)
        self.latest_twist_cmd_time = now_s()

    def traj_cb(self, msg):
        if not msg.points:
            return
        point = msg.points[-1]
        if len(point.positions) < len(msg.joint_names):
            return
        self.latest_traj = values_by_joint(msg.joint_names, point.positions)
        self.latest_traj_time = now_s()
        if self.traj_baseline is None:
            self.traj_baseline = dict(self.latest_traj)

    def joint_state_cb(self, msg):
        self.latest_state = values_by_joint(msg.name, msg.position)
        self.latest_state_time = now_s()
        if self.state_baseline is None and all(name in msg.name for name in JOINTS):
            self.state_baseline = dict(self.latest_state)

    def status_cb(self, msg):
        self.latest_status = int(msg.data)
        self.latest_status_time = now_s()

    def collision_scale_cb(self, msg):
        self.latest_collision_scale = float(msg.data)
        self.latest_collision_scale_time = now_s()

    def note_once(self, key, text):
        if key in self.notes_seen:
            return
        self.notes_seen.add(key)
        print(f"NOTE: {text}")

    def print_sample(self):
        elapsed = now_s() - self.start
        joint_cmd = self.latest_joint_cmd or {name: 0.0 for name in JOINTS}
        joint_cmd_deg_s = ordered_deg(joint_cmd)
        twist = self.latest_twist_cmd or [0.0] * 6
        traj_delta = deltas_deg(self.latest_traj, self.traj_baseline)
        state_delta = deltas_deg(self.latest_state, self.state_baseline)
        status = "n/a" if self.latest_status is None else str(self.latest_status)
        scale = float("nan") if self.latest_collision_scale is None else self.latest_collision_scale

        joint_recent = age_s(self.latest_joint_cmd_time) <= self.args.recent_window
        twist_recent = age_s(self.latest_twist_cmd_time) <= self.args.recent_window
        twist_norm = math.sqrt(sum(v * v for v in twist))
        non_j2_cmd = any(abs(joint_cmd[name]) > self.args.epsilon for name in ["arm_j1", "arm_j3", "arm_j4", "arm_j5", "arm_j6"])
        j2_only_cmd = joint_recent and abs(joint_cmd["arm_j2"]) > self.args.epsilon and not non_j2_cmd
        extra_traj = any(abs(value) > self.args.epsilon_deg for value in [traj_delta[2], traj_delta[3], traj_delta[4]])
        extra_state = any(abs(value) > self.args.epsilon_deg for value in [state_delta[2], state_delta[3], state_delta[4]])

        if twist_recent and twist_norm > self.args.epsilon:
            self.note_once(
                "twist_active",
                "Twist commands are active. MoveIt Servo Cartesian IK can move several joints while status remains 0.",
            )
        if joint_recent and non_j2_cmd:
            self.note_once(
                "joint_input_not_j2_only",
                "Incoming JointJog contains non-j2 velocities, so coupled output may originate before Servo.",
            )
        if j2_only_cmd and not twist_recent and extra_traj:
            self.note_once(
                "extra_traj",
                "Servo output trajectory changed j3/j4/j5 while input looked j2-only. Focus on Servo smoothing/limits/state.",
            )
        if j2_only_cmd and not twist_recent and extra_state and not extra_traj:
            self.note_once(
                "extra_state",
                "Joint states changed j3/j4/j5 without matching Servo trajectory deltas. Focus on controller/hardware/transmission.",
            )

        print(
            f"t={elapsed:7.2f}s status={status:>3} coll_scale="
            f"{'n/a' if not math.isfinite(scale) else f'{scale:.3f}'} "
            f"joint_age={fmt_age(self.latest_joint_cmd_time)} twist_age={fmt_age(self.latest_twist_cmd_time)} "
            f"traj_age={fmt_age(self.latest_traj_time)} state_age={fmt_age(self.latest_state_time)}"
        )
        print(f"  in_joint:  {fmt_vec(joint_cmd_deg_s)}   [j1 j2 j3 j4 j5 j6]")
        print(f"  in_twist:  {fmt_vec(twist, width=7, precision=3)}   [vx vy vz wx wy wz]")
        print(f"  out_dpos:  {fmt_vec(traj_delta)}   trajectory delta from trace start")
        print(f"  js_dpos:   {fmt_vec(state_delta)}   joint_states delta from trace start")

        if self.csv_writer is not None:
            row = {
                "elapsed_s": elapsed,
                "status": self.latest_status,
                "collision_scale": self.latest_collision_scale,
                "joint_cmd_age_s": age_s(self.latest_joint_cmd_time),
                "twist_cmd_age_s": age_s(self.latest_twist_cmd_time),
                "twist_lin_x": twist[0],
                "twist_lin_y": twist[1],
                "twist_lin_z": twist[2],
                "twist_ang_x": twist[3],
                "twist_ang_y": twist[4],
                "twist_ang_z": twist[5],
            }
            for joint, value in zip(JOINTS, joint_cmd_deg_s):
                row[f"joint_cmd_{joint}_deg_s"] = value
            traj_abs = ordered_deg(self.latest_traj or {})
            state_abs = ordered_deg(self.latest_state or {})
            for joint, value in zip(JOINTS, traj_abs):
                row[f"traj_{joint}_deg"] = value
            for joint, value in zip(JOINTS, state_abs):
                row[f"state_{joint}_deg"] = value
            self.csv_writer.writerow(row)
            self.csv_file.flush()


def main():
    parser = argparse.ArgumentParser(
        description="Trace MoveIt Servo inputs, outputs, and joint states during manipulator jogs."
    )
    parser.add_argument("--period", type=float, default=0.5, help="Print period in seconds.")
    parser.add_argument("--recent-window", type=float, default=0.25, help="Message age considered active.")
    parser.add_argument("--epsilon", type=float, default=1e-4, help="Velocity/twist epsilon in native units.")
    parser.add_argument("--epsilon-deg", type=float, default=0.05, help="Position delta epsilon in degrees.")
    parser.add_argument("--csv", default="", help="Optional CSV path for samples.")
    args = parser.parse_args()

    rclpy.init()
    node = ServoJogTrace(args)

    def stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Stopped.")


if __name__ == "__main__":
    main()
PY
