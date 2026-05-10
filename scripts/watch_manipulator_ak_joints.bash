#!/usr/bin/env bash
set -euo pipefail

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

need_cmd python3
need_cmd candump
need_cmd ip

exec python3 - "$@" <<'PY'
import argparse
import math
import re
import select
import signal
import subprocess
import sys
import time

MOTORS = (101, 102, 103, 104, 105, 106)
JOINT_BY_MOTOR = {
    101: "arm_j1",
    102: "arm_j2",
    103: "arm_j3",
    104: "arm_j4",
    105: "arm_j5",
    106: "arm_j6",
}

# Hardcoded from rover_can.ros2_control.xacro.
REDUCTION_BY_MOTOR = {
    101: -2.0,
    102: -50.0,
    103: 50.0,
    104: 1.0,
    105: -1.0,
    106: -1.0,
}

FRAME_RE = re.compile(
    r"\(([0-9.]+)\).*?\b\S+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+(.*)$"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="scripts/watch_manipulator_ak_joints.bash",
        description=(
            "Continuously watch manipulator AK servo feedback frames and print "
            "estimated joint positions until interrupted."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Assumptions:
  - The manipulator starts at zero pose.
  - The first feedback frame seen for each AK motor is that motor's zero.
  - AK feedback frames use extended CAN ID 0x00002900 | motor_id, DLC 8.
""",
    )
    parser.add_argument("-i", "--iface", default="can0", help="CAN interface")
    parser.add_argument(
        "-r",
        "--rate",
        default=10.0,
        type=float,
        help="terminal refresh rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--raw-changes",
        action="store_true",
        help="print only status payload changes per motor",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="print a new table on every refresh instead of refreshing terminal",
    )
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be greater than zero")
    return args


def require_iface(iface: str) -> None:
    result = subprocess.run(
        ["ip", "link", "show", iface],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(f"CAN interface '{iface}' not found or not accessible.")


def status_filters(iface: str):
    return [f"{iface},{0x00002900 | motor:08X}:1FFFFFFF" for motor in MOTORS]


def signed_i16_be(data: bytes) -> int:
    return int.from_bytes(data, byteorder="big", signed=True)


def decode_payload(payload: bytes):
    p10 = signed_i16_be(payload[0:2])
    v10 = signed_i16_be(payload[2:4])
    c01 = signed_i16_be(payload[4:6])
    temp = int.from_bytes(payload[6:7], byteorder="big", signed=True)
    err = payload[7]
    motor_rad = (p10 / 10.0) * math.pi / 180.0
    motor_vel_rad_s = (v10 * 10.0) * math.pi / 30.0
    current_a = c01 / 100.0
    return motor_rad, motor_vel_rad_s, current_a, temp, err


def parse_frame(line: str):
    match = FRAME_RE.search(line)
    if not match:
        return None

    can_ts = float(match.group(1))
    can_id = int(match.group(2), 16)
    dlc = int(match.group(3))
    if dlc != 8 or (can_id & 0x1FFFFF00) != 0x00002900:
        return None

    motor = can_id & 0xFF
    if motor not in JOINT_BY_MOTOR:
        return None

    byte_text = match.group(4).split()
    if len(byte_text) < 8:
        return None

    try:
        payload = bytes(int(part, 16) for part in byte_text[:8])
    except ValueError:
        return None

    return can_ts, motor, payload


def empty_state():
    return {
        motor: {
            "frames": 0,
            "changes": 0,
            "payload": None,
            "zero": None,
            "raw": None,
            "min_raw": None,
            "max_raw": None,
            "delta": None,
            "joint": None,
            "velocity": None,
            "current": None,
            "temp": None,
            "err": None,
            "last_can_ts": None,
            "integrated_joint": 0.0,
        }
        for motor in MOTORS
    }


def payload_text(payload) -> str:
    if payload is None:
        return "-"
    return " ".join(f"{byte:02X}" for byte in payload)


def fmt(value, precision: int, unit: str = "") -> str:
    if value is None:
        return "-"
    return f"{value:.{precision}f}{unit}"


def print_table(args, state, last_can_ts, last_wall_ts) -> None:
    if not args.no_clear:
        sys.stdout.write("\033[H\033[2J")

    sys.stdout.write(f"Manipulator AK joint positions from {args.iface}\n")
    if last_can_ts is None or last_wall_ts is None:
        sys.stdout.write("Zero reference: first feedback frame per motor. Last frame: none\n\n")
    else:
        now = time.time()
        processed_age = now - last_wall_ts
        can_lag = now - last_can_ts
        sys.stdout.write(
            "Zero reference: first feedback frame per motor. "
            f"Last frame: {last_can_ts:.6f} "
            f"({processed_age:.2f}s processed, {can_lag:.2f}s CAN lag)\n\n"
        )

    header = (
        f"{'Joint':<8} {'Motor':<8} {'Frames':<8} {'Changes':<7} "
        f"{'Payload':<23} {'ZeroDeg':>11} {'RawDeg':>11} "
        f"{'MinRawDeg':>11} {'MaxRawDeg':>11} {'DeltaDeg':>11} "
        f"{'JointRad':>11} {'JointDeg':>11} {'Velocity':>11} "
        f"{'Current':>9} {'Temp':>7} Err\n"
    )
    sys.stdout.write(header)

    for motor in MOTORS:
        s = state[motor]
        joint = JOINT_BY_MOTOR[motor]
        if s["frames"] == 0:
            sys.stdout.write(
                f"{joint:<8} {motor:<8} {0:<8} {'-':<7} {'-':<23} "
                f"{'-':>11} {'-':>11} {'-':>11} {'-':>11} {'-':>11} "
                f"{'-':>11} {'-':>11} {'-':>11} {'-':>9} {'-':>7} NO_FRAMES\n"
            )
            continue

        zero_deg = math.degrees(s["zero"])
        raw_deg = math.degrees(s["raw"])
        min_raw_deg = math.degrees(s["min_raw"])
        max_raw_deg = math.degrees(s["max_raw"])
        delta_deg = math.degrees(s["delta"])
        joint_deg = math.degrees(s["joint"])
        sys.stdout.write(
            f"{joint:<8} {motor:<8} {s['frames']:<8} {s['changes']:<7} "
            f"{payload_text(s['payload']):<23} "
            f"{zero_deg:11.3f} {raw_deg:11.3f} "
            f"{min_raw_deg:11.3f} {max_raw_deg:11.3f} {delta_deg:11.3f} "
            f"{s['joint']:11.7f} {joint_deg:11.4f} {s['velocity']:11.7f} "
            f"{s['current']:8.2f}A {s['temp']:6d}C {s['err']}\n"
        )

    sys.stdout.write("\nPress Ctrl-C to stop.\n")
    sys.stdout.flush()


def update_state(state, can_ts: float, motor: int, payload: bytes) -> None:
    s = state[motor]
    motor_rad, motor_vel_rad_s, current_a, temp, err = decode_payload(payload)

    if s["zero"] is None:
        s["zero"] = motor_rad
        s["min_raw"] = motor_rad
        s["max_raw"] = motor_rad
        s["integrated_joint"] = 0.0

    if s["payload"] is not None and s["payload"] != payload:
        s["changes"] += 1

    reduction = REDUCTION_BY_MOTOR[motor]
    delta = motor_rad - s["zero"]
    joint = delta / reduction
    velocity = motor_vel_rad_s / reduction

    if s["last_can_ts"] is not None:
        dt = can_ts - s["last_can_ts"]
        if 0.0 < dt < 1.0:
            s["integrated_joint"] += velocity * dt

    if abs(delta) < 1e-6 and abs(velocity) > 1e-6:
        joint = s["integrated_joint"]

    s["frames"] += 1
    s["payload"] = payload
    s["raw"] = motor_rad
    s["min_raw"] = min(s["min_raw"], motor_rad)
    s["max_raw"] = max(s["max_raw"], motor_rad)
    s["delta"] = delta
    s["joint"] = joint
    s["velocity"] = velocity
    s["current"] = current_a
    s["temp"] = temp
    s["err"] = err
    s["last_can_ts"] = can_ts


def main() -> int:
    args = parse_args()
    require_iface(args.iface)

    cmd = ["candump", "-ta", *status_filters(args.iface)]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    def stop_proc(*_):
        if proc.poll() is None:
            proc.terminate()

    signal.signal(signal.SIGTERM, stop_proc)

    state = empty_state()
    period = 1.0 / args.rate
    last_print = 0.0
    last_can_ts = None
    last_wall_ts = None

    print(f"Listening for manipulator AK feedback on {args.iface}...")
    if args.raw_changes:
        print("Raw-change mode: printing first payload and later payload changes only.")
    else:
        print(f"Terminal refresh rate: {args.rate:g} Hz")
    print("Press Ctrl-C to stop.")
    sys.stdout.flush()

    try:
        while True:
            if proc.stdout is None:
                break

            ready, _, _ = select.select([proc.stdout], [], [], 0.1)
            if not ready:
                if proc.poll() is not None:
                    break
                now = time.time()
                if not args.raw_changes and now - last_print >= period:
                    last_print = now
                    print_table(args, state, last_can_ts, last_wall_ts)
                continue

            line = proc.stdout.readline()
            if line == "":
                if proc.poll() is not None:
                    break
                continue

            parsed = parse_frame(line)
            if parsed is None:
                continue

            can_ts, motor, payload = parsed
            previous_payload = state[motor]["payload"]
            update_state(state, can_ts, motor, payload)
            last_can_ts = can_ts
            last_wall_ts = time.time()

            if args.raw_changes:
                if previous_payload is None or previous_payload != payload:
                    motor_rad, motor_vel_rad_s, current_a, temp, err = decode_payload(payload)
                    print(
                        f"{can_ts:.6f} motor={motor} joint={JOINT_BY_MOTOR[motor]} "
                        f'payload="{payload_text(payload)}" '
                        f"raw_deg={math.degrees(motor_rad):.3f} "
                        f"motor_vel_rad_s={motor_vel_rad_s:.7f} "
                        f"current_a={current_a:.4f} temp_c={temp} err={err}",
                        flush=True,
                    )
                continue

            now = time.time()
            if now - last_print >= period:
                last_print = now
                print_table(args, state, last_can_ts, last_wall_ts)

    except KeyboardInterrupt:
        pass
    finally:
        stop_proc()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=1.0)
        if not args.raw_changes:
            print_table(args, state, last_can_ts, last_wall_ts)

    print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
PY
