#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/nofw_diag_periods.bash [options]

Capture NoFW runtime diagnostic and telemetry frames, then report observed
periods for each expected node.

Options:
  -i, --iface IFACE      CAN interface (default: can0)
  -d, --duration SEC     Capture duration in seconds (default: 10)
  -n, --node NODE        Expected node ID, 1..15. May be repeated.
                         Default: nodes 1..9.
  --raw                  Print raw captured diagnostic/telemetry frames after
                         the summary.
  -h, --help             Show this help.

Examples:
  scripts/nofw_diag_periods.bash
  scripts/nofw_diag_periods.bash --duration 5
  scripts/nofw_diag_periods.bash --node 1 --node 3 --raw
EOF
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

iface="can0"
duration="10"
raw=0
nodes=()

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
    -n|--node)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      nodes+=("$2")
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

if [[ ${#nodes[@]} -eq 0 ]]; then
  nodes=(1 2 3 4 5 6 7 8 9)
fi

for node in "${nodes[@]}"; do
  if [[ ! "$node" =~ ^[0-9]+$ ]] || (( node < 1 || node > 15 )); then
    echo "Invalid node: $node (expected 1..15)" >&2
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

echo "Capturing NoFW diagnostics and telemetry on ${iface} for ${duration}s..."
echo "Filters: ${iface},400:700 ${iface},5F0:7F0"
echo

set +e
timeout "$duration" candump -ta "${iface},400:700" "${iface},5F0:7F0" >"$tmp"
rc=$?
set -e

if (( rc != 0 && rc != 124 )); then
  echo "candump failed with exit code $rc" >&2
  exit "$rc"
fi

expected_nodes="${nodes[*]}"
EXPECTED_NODES="$expected_nodes" perl - "$tmp" <<'PERL'
use strict;
use warnings;

my $path = shift @ARGV;
my @expected = split /\s+/, $ENV{EXPECTED_NODES} // "";

my (%count, %intervals, %sum, %min, %max, %first, %last, %payload, %seen_nodes);
my %family_label = (
  diag => "Diagnostic",
  angle => "Angle",
  velocity => "Velocity",
  limits => "Travel Limits",
  config => "Config",
  other => "Other 0x4xx",
);

open my $fh, "<", $path or die "Cannot read $path: $!\n";
while (my $line = <$fh>) {
  next unless $line =~ /\(([0-9.]+)\).*?\bcan\d+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+(.*)$/;
  my ($t, $id_hex, $dlc, $data) = ($1 + 0, uc($2), $3 + 0, uc($4));
  my $id = hex($id_hex);
  my ($family, $node);

  if ($id >= 0x5F0 && $id <= 0x5FF) {
    next unless $dlc == 8;
    $family = "diag";
    $node = $id - 0x5F0;
  } elsif ($id >= 0x400 && $id <= 0x4FF) {
    my $base = $id & 0x7F0;
    $node = $id & 0x00F;
    if ($base == 0x400) {
      next unless $dlc == 4;
      $family = "angle";
    } elsif ($base == 0x410) {
      next unless $dlc == 4;
      $family = "velocity";
    } elsif ($base == 0x420) {
      next unless $dlc == 8;
      $family = "limits";
    } elsif ($base == 0x430) {
      next unless $dlc == 8;
      $family = "config";
    } else {
      $family = "other";
    }
  } else {
    next;
  }
  next if $node < 1 || $node > 15;

  my $key = "$family:$node";
  $seen_nodes{$node} = 1;
  $count{$key}++;
  $payload{$key} = $data;
  $first{$key} = $t if !exists $first{$key};
  if (exists $last{$key}) {
    my $dt = ($t - $last{$key}) * 1000.0;
    $intervals{$key}++;
    $sum{$key} += $dt;
    $min{$key} = $dt if !exists($min{$key}) || $dt < $min{$key};
    $max{$key} = $dt if !exists($max{$key}) || $dt > $max{$key};
  }
  $last{$key} = $t;
}
close $fh;

sub profile_name {
  return ("VelocityOnly", "As5600", "TmagLut", "DirectInput")[$_[0]] // $_[0];
}

sub mode_name {
  return $_[0] == 1 ? "angle" : $_[0] == 2 ? "velocity" : $_[0];
}

sub i32_le {
  my (@b) = @_;
  my $v = $b[0] | ($b[1] << 8) | ($b[2] << 16) | ($b[3] << 24);
  $v -= 4294967296 if $v & 0x80000000;
  return $v;
}

sub decode_diag {
  my ($data) = @_;
  my @b = map { hex($_) } ($data =~ /[0-9A-Fa-f]{2}/g);
  return "invalid_payload" unless @b == 8;

  my $flags = $b[4];
  my $cal_load = ($flags >> 4) & 0x03;
  my $need_cal = ($b[6] & 0x01) ? 1 : 0;
  my $profile_result = ($b[6] >> 4) & 0x0f;
  my $feedback_required = ($b[7] & 0x01) ? 1 : 0;
  my $armed = ($b[7] & 0x02) ? 1 : 0;

  return sprintf(
    "magic=0x%02X stored=%s active=%s mode=%s vel=%d angle=%d foc_cal=%d out_cal=%d cal_load=%d fault=%d need_cal=%d profile_result=%d feedback_req=%d armed=%d",
    $b[0],
    profile_name($b[1]),
    profile_name($b[2]),
    mode_name($b[3]),
    ($flags & 0x01) ? 1 : 0,
    ($flags & 0x02) ? 1 : 0,
    ($flags & 0x04) ? 1 : 0,
    ($flags & 0x08) ? 1 : 0,
    $cal_load,
    $b[5],
    $need_cal,
    $profile_result,
    $feedback_required,
    $armed
  );
}

sub decode_telemetry {
  my ($family, $data) = @_;
  my @b = map { hex($_) } ($data =~ /[0-9A-Fa-f]{2}/g);

  if ($family eq "angle") {
    return "invalid_payload" unless @b == 4;
    my $mdeg = i32_le(@b);
    return sprintf("angle=%d mdeg %.6f rad", $mdeg, ($mdeg / 1000.0) * 3.141592653589793 / 180.0);
  }

  if ($family eq "velocity") {
    return "invalid_payload" unless @b == 4;
    my $mdeg_s = i32_le(@b);
    return sprintf("velocity=%d mdeg/s %.6f rad/s", $mdeg_s, ($mdeg_s / 1000.0) * 3.141592653589793 / 180.0);
  }

  if ($family eq "limits") {
    return "invalid_payload" unless @b == 8;
    my $min_mdeg = i32_le(@b[0..3]);
    my $max_mdeg = i32_le(@b[4..7]);
    return sprintf("min=%d mdeg max=%d mdeg", $min_mdeg, $max_mdeg);
  }

  if ($family eq "config") {
    return "invalid_payload" unless @b == 8;
    my $gear_ratio_milli = i32_le(@b[0..3]);
    return sprintf(
      "gear=%.3f stored=%s mode=%s vel=%d angle=%d reserved=0x%02X",
      $gear_ratio_milli / 1000.0,
      profile_name($b[4]),
      mode_name($b[5]),
      ($b[6] & 0x01) ? 1 : 0,
      ($b[6] & 0x02) ? 1 : 0,
      $b[7]
    );
  }

  return "raw=$data";
}

sub print_stats_row {
  my ($node, $id_hex, $key, $decoder) = @_;
  if (!$count{$key}) {
    printf "%-4d %-5s %-9s %-10s %-10s %-10s %-10s %s\n",
      $node, $id_hex, "0", "-", "-", "-", "-", "MISSING";
    return;
  }

  my $mean = $intervals{$key} ? $sum{$key} / $intervals{$key} : 0;
  my $span = $last{$key} - $first{$key};
  printf "%-4d %-5s %-9d %-10s %-10s %-10s %-10.3f %s\n",
    $node,
    $id_hex,
    $count{$key},
    $intervals{$key} ? sprintf("%.3f", $mean) : "-",
    $intervals{$key} ? sprintf("%.3f", $min{$key}) : "-",
    $intervals{$key} ? sprintf("%.3f", $max{$key}) : "-",
    $span,
    $decoder->($payload{$key});
}

printf "%-4s %-5s %-9s %-10s %-10s %-10s %-10s %s\n",
  "Node", "ID", "Count", "Mean(ms)", "Min(ms)", "Max(ms)", "Span(s)", "Last diagnostic";
printf "%s\n", "-" x 140;

for my $node (@expected) {
  my $id_hex = sprintf("0x%03X", 0x5F0 + $node);
  print_stats_row($node, $id_hex, "diag:$node", sub { decode_diag($_[0]) });
}

for my $family (qw(angle velocity limits config other)) {
  my @rows;
  for my $key (keys %count) {
    next unless $key =~ /^\Q$family\E:(\d+)$/;
    push @rows, $1;
  }
  my %row_seen = map { $_ => 1 } @rows;
  @rows = sort { $a <=> $b } keys %row_seen;
  next unless @rows;

  print "\n$family_label{$family} telemetry statistics\n";
  printf "%-4s %-5s %-9s %-10s %-10s %-10s %-10s %s\n",
    "Node", "ID", "Count", "Mean(ms)", "Min(ms)", "Max(ms)", "Span(s)", "Last telemetry";
  printf "%s\n", "-" x 140;

  for my $node (@rows) {
    my $base = $family eq "angle" ? 0x400 :
               $family eq "velocity" ? 0x410 :
               $family eq "limits" ? 0x420 :
               $family eq "config" ? 0x430 : 0x400;
    my $id_hex = $family eq "other" ? "0x4xx" : sprintf("0x%03X", $base + $node);
    print_stats_row($node, $id_hex, "$family:$node", sub { decode_telemetry($family, $_[0]) });
  }
}
PERL

echo
ip -details -statistics link show "$iface" | sed -n '/link\/can/,$p'

if (( raw == 1 )); then
  echo
  echo "Raw diagnostic/telemetry frames:"
  cat "$tmp"
fi
