#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/camera_turret_move.bash [options] <x> <y>
  scripts/camera_turret_move.bash [options] <direction> [magnitude]

Move the camera turret by sending classic CAN joystick commands with can-utils.

Arguments:
  x y                    Normalized command values in [-1.0, 1.0].
  direction              left, right, up, down, center.
  magnitude              Direction magnitude in [0.0, 1.0] (default: 0.5).

Options:
  -i, --iface IFACE      CAN interface (default: can0).
  --can-id ID            Standard CAN ID in hex or decimal (default: 0x124).
  -d, --duration SEC     Command duration in seconds (default: 1.0).
  -r, --rate HZ          Publish rate in Hz (default: 20).
  --center               Send a centered command on exit.
  -h, --help             Show this help.

Examples:
  scripts/camera_turret_move.bash right
  scripts/camera_turret_move.bash --iface can0 up 0.8 --duration 2
  scripts/camera_turret_move.bash 0.25 -0.5

If can0 is down or misconfigured, run:
  scripts/can0.bash
EOF
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

is_number() {
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

iface="can0"
can_id="0x124"
duration="1.0"
rate_hz="20"
center_on_exit=0

args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--iface)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      iface="$2"
      shift 2
      ;;
    --can-id)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      can_id="$2"
      shift 2
      ;;
    -d|--duration)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      duration="$2"
      shift 2
      ;;
    -r|--rate)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      rate_hz="$2"
      shift 2
      ;;
    --center)
      center_on_exit=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      args+=("$@")
      break
      ;;
    -*)
      if is_number "$1"; then
        args+=("$1")
        shift
      else
        echo "Unknown option: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
    *)
      args+=("$1")
      shift
      ;;
  esac
done

if [[ ${#args[@]} -lt 1 || ${#args[@]} -gt 2 ]]; then
  usage >&2
  exit 2
fi

if ! is_number "$duration"; then
  echo "Invalid duration: $duration" >&2
  exit 2
fi

if ! is_number "$rate_hz"; then
  echo "Invalid rate: $rate_hz" >&2
  exit 2
fi

need_cmd cansend
need_cmd ip
need_cmd python3

if ! ip link show "${iface}" >/dev/null 2>&1; then
  echo "CAN interface '${iface}' not found or not accessible." >&2
  exit 1
fi

x=""
y=""
case "${args[0]}" in
  left)
    x="-${args[1]:-0.5}"
    y="0.0"
    ;;
  right)
    x="${args[1]:-0.5}"
    y="0.0"
    ;;
  up)
    x="0.0"
    y="${args[1]:-0.5}"
    ;;
  down)
    x="0.0"
    y="-${args[1]:-0.5}"
    ;;
  center)
    x="0.0"
    y="0.0"
    ;;
  *)
    if [[ ${#args[@]} -ne 2 ]]; then
      echo "Expected both x and y, or a direction." >&2
      usage >&2
      exit 2
    fi
    x="${args[0]}"
    y="${args[1]}"
    ;;
esac

validate_unit() {
  local name="$1"
  local value="$2"
  python3 - "$name" "$value" <<'PY'
import math
import sys

name, value_text = sys.argv[1], sys.argv[2]
try:
    value = float(value_text)
except ValueError:
    raise SystemExit(f"Invalid {name}: {value_text}")

if not math.isfinite(value) or value < -1.0 or value > 1.0:
    raise SystemExit(f"Invalid {name}: {value_text} (expected -1.0..1.0)")
PY
}

validate_positive() {
  local name="$1"
  local value="$2"
  python3 - "$name" "$value" <<'PY'
import math
import sys

name, value_text = sys.argv[1], sys.argv[2]
try:
    value = float(value_text)
except ValueError:
    raise SystemExit(f"Invalid {name}: {value_text}")

if not math.isfinite(value) or value <= 0.0:
    raise SystemExit(f"Invalid {name}: {value_text} (expected > 0)")
PY
}

validate_unit x "$x"
validate_unit y "$y"
validate_positive duration "$duration"
validate_positive rate "$rate_hz"

frame_id="$(
  python3 - "$can_id" <<'PY'
import sys

try:
    value = int(sys.argv[1], 0)
except ValueError:
    raise SystemExit(f"Invalid CAN ID: {sys.argv[1]}")

if value < 0 or value > 0x7FF:
    raise SystemExit(f"Invalid CAN ID: {sys.argv[1]} (expected 0x000..0x7ff)")

print(f"{value:03X}")
PY
)"

payload_for_command() {
  python3 - "$1" "$2" <<'PY'
import math
import struct
import sys

def normalized_to_adc(text):
    value = float(text)
    if not math.isfinite(value):
        value = 0.0
    value = max(-1.0, min(1.0, value))
    adc = int(math.floor((((value + 1.0) * 0.5) * 4095.0) + 0.5))
    return max(0, min(4095, adc))

vrx = normalized_to_adc(sys.argv[1])
vry = normalized_to_adc(sys.argv[2])
print(struct.pack("<HH", vrx, vry).hex().upper())
PY
}

command_payload="$(payload_for_command "$x" "$y")"
center_payload="$(payload_for_command 0.0 0.0)"

send_payload() {
  local payload="$1"
  cansend "${iface}" "${frame_id}#${payload}"
}

send_center() {
  if (( center_on_exit )); then
    send_payload "${center_payload}" || true
  fi
}

center() {
  send_center
}

trap center EXIT INT TERM

echo "Sending ${iface} ${frame_id}#${command_payload}: x=${x}, y=${y}, duration=${duration}s, rate=${rate_hz}Hz"
read -r iterations period_sec < <(
  python3 - "$duration" "$rate_hz" <<'PY'
import sys

duration = float(sys.argv[1])
rate_hz = float(sys.argv[2])
iterations = max(1, int(duration * rate_hz + 0.999999))
print(iterations, 1.0 / rate_hz)
PY
)

for ((i = 0; i < iterations; ++i)); do
  if ! send_payload "${command_payload}"; then
    echo "Failed to send classic CAN frame on ${iface}." >&2
    center_on_exit=0
    exit 1
  fi
  sleep "${period_sec}"
done

send_center
trap - EXIT INT TERM
