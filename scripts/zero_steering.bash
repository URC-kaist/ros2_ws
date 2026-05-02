#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/zero_steering.bash [options]

Stores the current physical steering pose as output 0 deg for NoFW steering
actuators. The script disarms each selected node, then sends output encoder
zero capture.

Options:
  -i, --iface IFACE   CAN interface (default: can0)
  -n, --node NODE     Steering node to zero: 1=FR, 2=BR, 3=FL, 4=BL.
                      May be repeated. Default: all steering nodes.
  --yes              Required to actually send frames.
  --dry-run          Print frames without sending them.
  -h, --help         Show this help.

Examples:
  scripts/zero_steering.bash --dry-run
  scripts/zero_steering.bash --yes
  scripts/zero_steering.bash --node 3 --yes
  scripts/zero_steering.bash --iface can1 --yes
EOF
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

iface="can0"
dry_run=0
confirmed=0
nodes=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--iface)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      iface="$2"
      shift 2
      ;;
    -n|--node)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      nodes+=("$2")
      shift 2
      ;;
    --yes)
      confirmed=1
      shift
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ${#nodes[@]} -eq 0 ]]; then
  nodes=(1 2 3 4)
fi

declare -A labels=(
  [1]="S01 FR"
  [2]="S02 BR"
  [3]="S03 FL"
  [4]="S04 BL"
)

for node in "${nodes[@]}"; do
  if [[ ! "$node" =~ ^[0-9]+$ ]] || (( node < 1 || node > 4 )); then
    echo "Invalid steering node: $node (expected 1..4)" >&2
    exit 2
  fi
done

if (( dry_run == 0 )); then
  need_cmd cansend
  if (( confirmed == 0 )); then
    echo "Refusing to zero steering without --yes." >&2
    echo "This stores the current physical AS5600 angle as output 0 deg." >&2
    exit 2
  fi
fi

frame_id() {
  local base="$1"
  local node="$2"
  printf '%03X' "$((base + node))"
}

send_frame() {
  local node="$1"
  local id="$2"
  local data="$3"
  local meaning="$4"
  local frame="${id}#${data}"

  printf '%-7s node=%s %-6s %s\n' "${labels[$node]}" "$node" "$meaning" "$frame"
  if (( dry_run == 0 )); then
    cansend "$iface" "$frame"
    sleep 0.05
  fi
}

echo "CAN interface: $iface"
echo "Selected steering nodes: ${nodes[*]}"
echo

echo "Disarming selected steering nodes..."
for node in "${nodes[@]}"; do
  send_frame "$node" "$(frame_id 0x230 "$node")" "00" "disarm"
done

echo
echo "Capturing current output encoder angle as 0 deg..."
for node in "${nodes[@]}"; do
  send_frame "$node" "$(frame_id 0x280 "$node")" "01" "zero"
done

echo
echo "Done. Verify runtime diagnostic frames 0x5F1..0x5F4:"
echo "  trusted output calibration valid = 1"
echo "  need calibration = 0"
echo "  runtime fault = 0"
