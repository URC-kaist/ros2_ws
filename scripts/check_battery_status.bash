#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/check_battery_status.bash [options]

Capture CAN traffic, then summarize both rover battery monitors as configured
by rover/ros2_ws/src/mr2_rover_description/launch/real.launch.py.

Options:
  -i, --iface IFACE      CAN interface (default: can0)
  -d, --duration SEC     Capture duration in seconds (default: 12)
  --raw                  Print matching raw frames after the summary
  -h, --help             Show this help

Battery frame maps:
  battery_1: summary 0x300, metadata 0x301, cells 0x310-0x314
  battery_2: summary 0x320, metadata 0x321, cells 0x330-0x334
EOF
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

iface="can0"
duration="12"
raw=0

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

echo "Capturing battery status frames on ${iface} for ${duration}s..."
echo "battery_1: 0x300/0x301/0x310-0x314"
echo "battery_2: 0x320/0x321/0x330-0x334"
echo

set +e
timeout "$duration" candump -ta "$iface" >"$tmp"
rc=$?
set -e

if (( rc != 0 && rc != 124 )); then
  echo "candump failed with exit code $rc" >&2
  exit "$rc"
fi

RAW="$raw" perl - "$tmp" <<'PERL'
use strict;
use warnings;

my $path = shift @ARGV;
my $print_raw = ($ENV{RAW} // 0) ? 1 : 0;

my %battery = (
  battery_1 => { summary => 0x300, metadata => 0x301, cell_base => 0x310 },
  battery_2 => { summary => 0x320, metadata => 0x321, cell_base => 0x330 },
);

my (%count, %first, %last, %intervals, %sum, %min, %max, %raw_lines);
my (%summary, %meta, %cells);

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

sub u16_le { return $_[0] | ($_[1] << 8); }
sub s16_le {
  my $v = u16_le($_[0], $_[1]);
  $v -= 65536 if $v & 0x8000;
  return $v;
}

sub decode_summary {
  my ($name, @b) = @_;
  return if @b < 8;
  $summary{$name} = {
    soc => $b[0],
    health => $b[1],
    temp_c => s16_le($b[2], $b[3]) / 10.0,
    voltage_v => u16_le($b[4], $b[5]) / 100.0,
    cycle_low => u16_le($b[6], $b[7]),
  };
}

sub decode_meta {
  my ($name, @b) = @_;
  return if @b < 8;
  $meta{$name} = {
    capacity_mah => u16_le($b[0], $b[1]),
    parallel => $b[2],
    cell_count => $b[3],
    life_cycles => u16_le($b[4], $b[5]),
    cycle_high => u16_le($b[6], $b[7]),
  };
}

sub decode_cells {
  my ($name, @b) = @_;
  return if @b < 8;
  for my $slot (0, 4) {
    my $cell = $b[$slot];
    next if $cell == 0;
    $cells{$name}{$cell} = {
      mv => u16_le($b[$slot + 1], $b[$slot + 2]),
      valid => $b[$slot + 3] ? 1 : 0,
    };
  }
}

open my $fh, "<", $path or die "Cannot read $path: $!\n";
while (my $line = <$fh>) {
  chomp $line;
  next unless $line =~ /\(([0-9.]+)\).*?\bcan\d+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+(.*)$/;
  my ($t, $id_hex, $dlc, $data_text) = ($1 + 0, uc($2), $3 + 0, uc($4));
  my $id = hex($id_hex);
  my @b = map { hex($_) } ($data_text =~ /[0-9A-Fa-f]{2}/g);

  for my $name (keys %battery) {
    my $cfg = $battery{$name};
    if ($id == $cfg->{summary} && $dlc == 8) {
      note("$name:summary", $t, $line);
      decode_summary($name, @b);
    } elsif ($id == $cfg->{metadata} && $dlc == 8) {
      note("$name:metadata", $t, $line);
      decode_meta($name, @b);
    } elsif ($id >= $cfg->{cell_base} && $id <= $cfg->{cell_base} + 4 && $dlc == 8) {
      note("$name:cells", $t, $line);
      decode_cells($name, @b);
    }
  }
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

for my $name (qw(battery_1 battery_2)) {
  my $cfg = $battery{$name};
  print "$name\n";
  printf "  IDs: summary=0x%03X metadata=0x%03X cells=0x%03X-0x%03X\n",
    $cfg->{summary}, $cfg->{metadata}, $cfg->{cell_base}, $cfg->{cell_base} + 4;
  print "  Summary:  ", period("$name:summary"), "\n";
  print "  Metadata: ", period("$name:metadata"), "\n";
  print "  Cells:    ", period("$name:cells"), "\n";
  if (exists $summary{$name}) {
    my $s = $summary{$name};
    printf "  Pack: soc=%u%% health=%u%% temp=%.1fC voltage=%.2fV\n",
      $s->{soc}, $s->{health}, $s->{temp_c}, $s->{voltage_v};
  }
  if (exists $meta{$name}) {
    my $m = $meta{$name};
    my $cycle = ((exists $summary{$name} ? $summary{$name}{cycle_low} : 0) |
                 ($m->{cycle_high} << 16));
    printf "  Meta: capacity=%umAh parallel=%u cells=%u life_cycles=%u firmware_cycle=%u\n",
      $m->{capacity_mah}, $m->{parallel}, $m->{cell_count}, $m->{life_cycles}, $cycle;
  }
  if (exists $cells{$name}) {
    my @parts;
    for my $cell (sort { $a <=> $b } keys %{$cells{$name}}) {
      my $c = $cells{$name}{$cell};
      push @parts, sprintf("C%d=%dmV%s", $cell, $c->{mv}, $c->{valid} ? "" : " invalid");
    }
    print "  Cell voltages: ", join(" ", @parts), "\n";
  }
  print "\n";
}

if ($print_raw) {
  print "Raw matching frames\n";
  for my $key (sort keys %raw_lines) {
    print "[$key]\n";
    print "  $_\n" for @{$raw_lines{$key}};
  }
}
PERL
