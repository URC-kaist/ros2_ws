#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/check_ak_actuator_status.bash [options]

Capture CAN traffic, then summarize AK servo feedback frames.

Options:
  -i, --iface IFACE      CAN interface (default: can0)
  -d, --duration SEC     Capture duration in seconds (default: 5)
  -m, --motor ID         AK servo motor ID. May be repeated.
                         Default: 101 102 103 104 105 106.
  --raw                  Print matching raw frames after the summary
  -h, --help             Show this help

AK feedback frames use extended CAN ID 0x00002900 | motor_id, DLC 8.
EOF
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

iface="can0"
duration="5"
raw=0
motors=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--iface)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      iface="$2"
      shift 2
      ;;
    -d|--duration)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      duration="$2"
      shift 2
      ;;
    -m|--motor)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      motors+=("$2")
      shift 2
      ;;
    --raw)
      raw=1
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

if [[ ! "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Invalid duration: $duration" >&2
  exit 2
fi

if [[ ${#motors[@]} -eq 0 ]]; then
  motors=(101 102 103 104 105 106)
fi

for motor in "${motors[@]}"; do
  if [[ ! "$motor" =~ ^[0-9]+$ ]] || (( motor < 0 || motor > 255 )); then
    echo "Invalid motor ID: $motor (expected 0..255)" >&2
    exit 2
  fi
done

need_cmd candump
need_cmd timeout
need_cmd perl
need_cmd ip

if ! ip link show "$iface" >/dev/null 2>&1; then
  echo "CAN interface '$iface' not found or not accessible." >&2
  exit 1
fi

tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT

echo "Capturing AK servo feedback on ${iface} for ${duration}s..."
echo "Motors: ${motors[*]}"
echo

set +e
timeout "$duration" candump -ta "$iface" >"$tmp"
rc=$?
set -e

if (( rc != 0 && rc != 124 )); then
  echo "candump failed with exit code $rc" >&2
  exit "$rc"
fi

expected_motors="${motors[*]}"
RAW="$raw" EXPECTED_MOTORS="$expected_motors" perl - "$tmp" <<'PERL'
use strict;
use warnings;

my $path = shift @ARGV;
my %expected = map { $_ => 1 } grep { length } split /\s+/, $ENV{EXPECTED_MOTORS} // "";
my $print_raw = ($ENV{RAW} // 0) ? 1 : 0;
my (%count, %first, %last, %intervals, %sum, %min, %max, %status, %raw_lines);
my $total_frames = 0;
my $extended_frames = 0;
my %extended_ids;

sub note {
  my ($key, $t, $raw_line) = @_;
  $count{$key}++;
  $first{$key} = $t if !exists $first{$key};
  if (exists $last{$key}) {
    my $dt = ($t - $last{$key}) * 1000.0;
    $intervals{$key}++;
    $sum{$key} += $dt;
    $min{$key} = $dt if !exists($min{$key}) || $dt < $min{$key};
    $max{$key} = $dt if !exists($max{$key}) || $dt > $max{$key};
  }
  $last{$key} = $t;
  push @{$raw_lines{$key}}, $raw_line if $print_raw;
}

sub i16_be {
  my $v = ($_[0] << 8) | $_[1];
  $v -= 65536 if $v & 0x8000;
  return $v;
}

sub error_name {
  my ($code) = @_;
  return "OK" if $code == 0;
  return "Over temperature" if $code == 1;
  return "Over current" if $code == 2;
  return "Over voltage" if $code == 3;
  return "Under voltage" if $code == 4;
  return "Encoder fault" if $code == 5;
  return "Phase current unbalance" if $code == 6;
  return "Unknown";
}

open my $fh, "<", $path or die "Cannot read $path: $!\n";
while (my $line = <$fh>) {
  chomp $line;
  next unless $line =~ /\(([0-9.]+)\).*?\bcan\d+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+(.*)$/;
  my ($t, $id_hex, $dlc, $data_text) = ($1 + 0, uc($2), $3 + 0, uc($4));
  $total_frames++;
  if (length($id_hex) > 3) {
    $extended_frames++;
    $extended_ids{$id_hex}++;
  }
  my $id = hex($id_hex);
  next unless ($id & 0x1FFFFF00) == 0x00002900;
  my $motor = $id & 0xFF;
  next if %expected && !$expected{$motor};
  next unless $dlc == 8;
  my @b = map { hex($_) } ($data_text =~ /[0-9A-Fa-f]{2}/g);
  next unless @b >= 8;

  my $p10 = i16_be($b[0], $b[1]);
  my $v10 = i16_be($b[2], $b[3]);
  my $c01 = i16_be($b[4], $b[5]);
  my $temp = $b[6] > 127 ? $b[6] - 256 : $b[6];
  $status{$motor} = {
    position_deg => $p10 / 10.0,
    velocity_rpm => $v10 * 10.0,
    current_a => $c01 / 100.0,
    temp_c => $temp,
    error => $b[7],
  };
  note("ak:$motor", $t, $line);
}
close $fh;

sub period {
  my ($key) = @_;
  return "no frames" if !exists $count{$key};
  my $span = ($last{$key} // 0) - ($first{$key} // 0);
  my $mean = $intervals{$key} ? $sum{$key} / $intervals{$key} : 0;
  return sprintf("count=%d mean=%.1fms min=%.1fms max=%.1fms span=%.1fs",
    $count{$key}, $mean, $min{$key} // 0, $max{$key} // 0, $span);
}

printf "%-7s %-7s %-11s %-11s %-9s %-7s %s\n",
  "Motor", "Frames", "Position", "Velocity", "Current", "Temp", "Error";
my $missing = 0;
for my $motor (sort { $a <=> $b } keys %expected) {
  my $key = "ak:$motor";
  if (!exists $status{$motor}) {
    $missing++;
    printf "%-7s %-7s %-11s %-11s %-9s %-7s %s\n",
      $motor, "0", "-", "-", "-", "-", "NO_FRAMES";
    next;
  }
  my $s = $status{$motor};
  printf "%-7s %-7d %10.1fdeg %10.1frpm %8.2fA %6.0fC %u (%s)\n",
    $motor, $count{$key}, $s->{position_deg}, $s->{velocity_rpm},
    $s->{current_a}, $s->{temp_c}, $s->{error}, error_name($s->{error});
  print "        ", period($key), "\n";
}

if ($missing) {
  print "\nCaptured $total_frames total CAN frame(s), including $extended_frames extended frame(s),\n";
  print "but $missing expected AK motor(s) had no 0x29xx feedback.\n";
  if (%extended_ids) {
    print "Other extended CAN IDs (count):\n";
    my $shown = 0;
    for my $id (sort { $extended_ids{$b} <=> $extended_ids{$a} || hex($a) <=> hex($b) }
                  keys %extended_ids) {
      printf "  0x%s (%d)\n", $id, $extended_ids{$id};
      last if ++$shown >= 12;
    }
  } else {
    print "No extended CAN frames were seen at all.\n";
  }
  print "This is a passive receive-only test. NO_FRAMES can mean:\n";
  print "  - arm power/CAN wiring/termination, bitrate, or motor ID mismatch; or\n";
  print "  - the motor is in MIT/query-response mode instead of Servo Mode with periodic feedback.\n";
  print "The rover AK driver requires Servo Direct Mode and periodic status feedback.\n";
}

if ($print_raw) {
  print "\nRaw matching frames\n";
  for my $key (sort keys %raw_lines) {
    print "[$key]\n";
    print "  $_\n" for @{$raw_lines{$key}};
  }
}
PERL
