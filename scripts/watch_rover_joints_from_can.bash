#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/watch_rover_joints_from_can.bash [options]

Capture CAN traffic with can-utils and display rover joint positions estimated
from raw actuator feedback plus the transmission reductions in rover_can.ros2_control.xacro.

Assumption:
  Start this while the rover is physically at its zero pose. The first feedback
  frame seen for each actuator is used as that actuator's zero reference.

Options:
  -i, --iface IFACE      CAN interface (default: can0)
  -d, --duration SEC     Capture duration in seconds (default: 5)
  --xacro PATH           ros2_control xacro to parse
                          default: ../rover/ros2_ws/src/mr2_rover_description/ros2_control/rover_can.ros2_control.xacro
  --raw                  Print matching raw CAN frames after summaries
  -h, --help             Show this help

Examples:
  scripts/watch_rover_joints_from_can.bash
  scripts/watch_rover_joints_from_can.bash --duration 30
  scripts/watch_rover_joints_from_can.bash --iface can1 --raw
EOF
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
default_xacro="${script_dir}/../rover/ros2_ws/src/mr2_rover_description/ros2_control/rover_can.ros2_control.xacro"

iface="can0"
duration="5"
xacro_path="$default_xacro"
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
    --xacro)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      xacro_path="$2"
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

[[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "Invalid duration: $duration" >&2
  exit 2
}
[[ -f "$xacro_path" ]] || {
  echo "xacro not found: $xacro_path" >&2
  exit 1
}

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

echo "Capturing rover CAN feedback on ${iface} for ${duration}s..."
echo "Zero reference: first feedback frame seen per actuator"
echo "Transmission source: ${xacro_path}"
echo

set +e
timeout "$duration" candump -ta "$iface" >"$tmp"
rc=$?
set -e

if (( rc != 0 && rc != 124 )); then
  echo "candump failed with exit code $rc" >&2
  exit "$rc"
fi

RAW="$raw" perl - "$xacro_path" "$tmp" <<'PERL'
use strict;
use warnings;

my ($xacro, $capture) = @ARGV;
my $print_raw = ($ENV{RAW} // 0) ? 1 : 0;
my $pi = 3.14159265358979323846;
my %profile_name = (0 => "VelocityOnly", 1 => "As5600", 2 => "TmagLut", 3 => "DirectInput");
my %control_mode = (1 => "angle", 2 => "velocity");
my %ak_error = (
  0 => "OK",
  1 => "Over temperature",
  2 => "Over current",
  3 => "Over voltage",
  4 => "Under voltage",
  5 => "Encoder fault",
  6 => "Phase current unbalance",
);

sub trim {
  my ($s) = @_;
  $s =~ s/^\s+|\s+$//g;
  return $s;
}

sub i16_be {
  my $v = ($_[0] << 8) | $_[1];
  $v -= 65536 if $v & 0x8000;
  return $v;
}

sub i32_le {
  my $v = $_[0] | ($_[1] << 8) | ($_[2] << 16) | ($_[3] << 24);
  $v -= 4294967296 if $v & 0x80000000;
  return $v;
}

sub mdeg_to_rad {
  return ($_[0] / 1000.0) * ($pi / 180.0);
}

sub fmt {
  my ($v, $unit, $width, $prec) = @_;
  return sprintf("%${width}s", "-") if !defined $v;
  return sprintf("%${width}.${prec}f%s", $v, $unit);
}

sub note_period {
  my ($h, $key, $t) = @_;
  $h->{$key}{count}++;
  $h->{$key}{first} = $t if !defined $h->{$key}{first};
  if (defined $h->{$key}{last}) {
    my $dt = ($t - $h->{$key}{last}) * 1000.0;
    $h->{$key}{intervals}++;
    $h->{$key}{sum} += $dt;
    $h->{$key}{min} = $dt if !defined($h->{$key}{min}) || $dt < $h->{$key}{min};
    $h->{$key}{max} = $dt if !defined($h->{$key}{max}) || $dt > $h->{$key}{max};
  }
  $h->{$key}{last} = $t;
}

sub period_text {
  my ($p) = @_;
  return "no frames" if !$p || !$p->{count};
  my $mean = $p->{intervals} ? $p->{sum} / $p->{intervals} : 0;
  my $span = ($p->{last} // 0) - ($p->{first} // 0);
  return sprintf("frames=%d mean=%.1fms min=%.1fms max=%.1fms span=%.1fs",
    $p->{count}, $mean, $p->{min} // 0, $p->{max} // 0, $span);
}

my (%joint, %actuator_to_joint, %node_to_actuator, %motor_to_actuator);
my (%simple_joint_reduction, %fourbar_act_reduction, %fourbar_joint_reduction);

open my $xfh, "<", $xacro or die "Cannot read $xacro: $!\n";
my ($cur_joint, $cur_trans, $in_trans, $last_tag, $last_name);
while (my $line = <$xfh>) {
  chomp $line;

  if ($line =~ /<joint\b[^>]*name="([^"]+)"/ && !$in_trans) {
    $cur_joint = $1;
    $joint{$cur_joint}{name} = $cur_joint;
    next;
  }
  if (defined $cur_joint && $line =~ /<param\b[^>]*name="([^"]+)"[^>]*>([^<]*)<\/param>/) {
    my ($k, $v) = ($1, trim($2));
    $joint{$cur_joint}{$k} = $v;
    $node_to_actuator{$v} = $joint{$cur_joint}{actuator} if $k eq "node_id" && defined $joint{$cur_joint}{actuator};
    $motor_to_actuator{$v} = $joint{$cur_joint}{actuator} if $k eq "motor_id" && defined $joint{$cur_joint}{actuator};
    next;
  }
  if (defined $cur_joint && $line =~ /<\/joint>/) {
    $cur_joint = undef;
    next;
  }

  if ($line =~ /<transmission\b[^>]*name="([^"]+)"/) {
    $cur_trans = $1;
    $in_trans = 1;
    next;
  }
  next if !$in_trans;

  if ($line =~ /<actuator\b[^>]*name="([^"]+)"/) {
    $last_tag = "actuator";
    $last_name = $1;
    next;
  }
  if ($line =~ /<joint\b[^>]*name="([^"]+)"/) {
    $last_tag = "joint";
    $last_name = $1;
    next;
  }
  if ($line =~ /<mechanical_reduction>([^<]+)<\/mechanical_reduction>/) {
    my $r = trim($1) + 0.0;
    if ($cur_trans =~ /four_bar/i) {
      if (($last_tag // "") eq "actuator") {
        $fourbar_act_reduction{$last_name} = $r;
      } elsif (($last_tag // "") eq "joint") {
        $fourbar_joint_reduction{$last_name} = $r;
      }
    } elsif (($last_tag // "") eq "joint") {
      $simple_joint_reduction{$last_name} = $r;
    }
    next;
  }
  if ($line =~ /<\/transmission>/) {
    $cur_trans = undef;
    $in_trans = 0;
    $last_tag = undef;
    $last_name = undef;
    next;
  }
}
close $xfh;

for my $j (keys %joint) {
  my $a = $joint{$j}{actuator};
  next if !defined $a;
  $actuator_to_joint{$a} = $j;
}

sub joint_delta_for {
  my ($joint_name, $actuator_name, $delta_actuator) = @_;
  return undef if !defined $delta_actuator;
  if (exists $simple_joint_reduction{$joint_name}) {
    my $r = $simple_joint_reduction{$joint_name};
    return undef if !$r;
    return $delta_actuator / $r;
  }
  if (exists $fourbar_act_reduction{$actuator_name} && exists $fourbar_joint_reduction{$joint_name}) {
    my $ar = $fourbar_act_reduction{$actuator_name};
    my $jr = $fourbar_joint_reduction{$joint_name};
    return undef if !$ar || !$jr;
    return ($delta_actuator / $ar) / $jr;
  }
  return $delta_actuator;
}

my (%out, %ak, %period, %raw);

open my $cfh, "<", $capture or die "Cannot read $capture: $!\n";
while (my $line = <$cfh>) {
  chomp $line;
  next unless $line =~ /\(([0-9.]+)\).*?\bcan\d+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+(.*)$/;
  my ($t, $id_hex, $dlc, $data_text) = ($1 + 0.0, uc($2), $3 + 0, uc($4));
  my $id = hex($id_hex);
  my @b = map { hex($_) } ($data_text =~ /[0-9A-Fa-f]{2}/g);
  next if @b < $dlc;

  if (($id & 0x1FFFFF00) == 0x00002900 && $dlc == 8) {
    my $motor = $id & 0xFF;
    my $act = $motor_to_actuator{$motor} // "motor_$motor";
    my $p_rad = (i16_be($b[0], $b[1]) / 10.0) * ($pi / 180.0);
    my $v_rad_s = (i16_be($b[2], $b[3]) * 10.0) * ($pi / 30.0);
    my $cur_a = i16_be($b[4], $b[5]) / 100.0;
    my $temp_c = $b[6] > 127 ? $b[6] - 256 : $b[6];
    my $err = $b[7];
    $ak{$act}{motor} = $motor;
    $ak{$act}{first_pos} = $p_rad if !defined $ak{$act}{first_pos};
    $ak{$act}{pos} = $p_rad;
    $ak{$act}{vel} = $v_rad_s;
    $ak{$act}{current} = $cur_a;
    $ak{$act}{temp} = $temp_c;
    $ak{$act}{error} = $err;
    note_period(\%period, "ak:$act", $t);
    push @{$raw{"ak:$act"}}, $line if $print_raw;
    next;
  }

  my $std_id = $id & 0x7FF;
  my $node = $std_id & 0x00F;
  my $base = $std_id & 0x7F0;
  next if $node < 1 || $node > 15;
  my $act = $node_to_actuator{$node} // "node_$node";
  $out{$act}{node} = $node;

  if ($base == 0x400 && $dlc >= 4) {
    my $pos = mdeg_to_rad(i32_le(@b[0..3]));
    $out{$act}{first_angle} = $pos if !defined $out{$act}{first_angle};
    $out{$act}{angle} = $pos;
    note_period(\%period, "out:$act:angle", $t);
    push @{$raw{"out:$act:angle"}}, $line if $print_raw;
  } elsif ($base == 0x410 && $dlc >= 4) {
    my $vel = mdeg_to_rad(i32_le(@b[0..3]));
    $out{$act}{velocity} = $vel;
    if (defined $out{$act}{last_velocity_t}) {
      my $dt = $t - $out{$act}{last_velocity_t};
      if ($dt > 0 && $dt < 1.0) {
        $out{$act}{integrated_pos} += ($out{$act}{last_velocity} // 0) * $dt;
      }
    } else {
      $out{$act}{integrated_pos} = 0.0;
    }
    $out{$act}{last_velocity} = $vel;
    $out{$act}{last_velocity_t} = $t;
    note_period(\%period, "out:$act:velocity", $t);
    push @{$raw{"out:$act:velocity"}}, $line if $print_raw;
  } elsif ($base == 0x420 && $dlc >= 8) {
    $out{$act}{limit_min} = mdeg_to_rad(i32_le(@b[0..3]));
    $out{$act}{limit_max} = mdeg_to_rad(i32_le(@b[4..7]));
    note_period(\%period, "out:$act:limits", $t);
    push @{$raw{"out:$act:limits"}}, $line if $print_raw;
  } elsif ($base == 0x430 && $dlc >= 8) {
    $out{$act}{fw_gear_ratio} = i32_le(@b[0..3]) / 1000.0;
    $out{$act}{stored_profile} = $b[4];
    $out{$act}{default_mode} = $b[5];
    $out{$act}{cfg_vel_enabled} = ($b[6] & 0x01) ? 1 : 0;
    $out{$act}{cfg_angle_enabled} = ($b[6] & 0x02) ? 1 : 0;
    note_period(\%period, "out:$act:config", $t);
    push @{$raw{"out:$act:config"}}, $line if $print_raw;
  } elsif ($std_id >= 0x5F1 && $std_id <= 0x5FF && $dlc >= 8) {
    $out{$act}{diag_magic} = $b[0];
    $out{$act}{stored_diag_profile} = $b[1];
    $out{$act}{active_profile} = $b[2];
    $out{$act}{diag_default_mode} = $b[3];
    $out{$act}{diag_vel_enabled} = ($b[4] & 0x01) ? 1 : 0;
    $out{$act}{diag_angle_enabled} = ($b[4] & 0x02) ? 1 : 0;
    $out{$act}{foc_cal_valid} = ($b[4] & 0x04) ? 1 : 0;
    $out{$act}{output_cal_valid} = ($b[4] & 0x08) ? 1 : 0;
    $out{$act}{cal_load} = ($b[4] >> 4) & 0x03;
    $out{$act}{runtime_fault} = $b[5];
    $out{$act}{need_cal} = ($b[6] & 0x01) ? 1 : 0;
    $out{$act}{profile_result} = ($b[6] >> 4) & 0x0F;
    $out{$act}{feedback_required} = ($b[7] & 0x01) ? 1 : 0;
    $out{$act}{armed} = ($b[7] & 0x02) ? 1 : 0;
    note_period(\%period, "out:$act:diag", $t);
    push @{$raw{"out:$act:diag"}}, $line if $print_raw;
  }
}
close $cfh;

print "Joint estimate from CAN feedback\n";
print "--------------------------------\n";
printf "%-24s %-16s %-8s %-10s %-12s %-12s %-12s %s\n",
  "Joint", "Actuator", "Src", "XacroRed", "JointPos", "JointVel", "RawPos", "Notes";

for my $joint_name (sort keys %joint) {
  my $act = $joint{$joint_name}{actuator};
  next if !defined $act;

  my ($src, $raw_pos, $raw_vel, $joint_pos, $joint_vel, $notes);
  $notes = "";

  if (exists $ak{$act}) {
    $src = "AK";
    $raw_pos = $ak{$act}{pos};
    $raw_vel = $ak{$act}{vel};
    my $delta = defined($ak{$act}{pos}) && defined($ak{$act}{first_pos})
      ? $ak{$act}{pos} - $ak{$act}{first_pos}
      : undef;
    $joint_pos = joint_delta_for($joint_name, $act, $delta);
    $joint_vel = joint_delta_for($joint_name, $act, $raw_vel);
    $notes = "motor=$ak{$act}{motor}";
  } elsif (exists $out{$act}) {
    $src = "NoFW";
    if (defined $out{$act}{angle}) {
      $raw_pos = $out{$act}{angle};
      my $delta = $out{$act}{angle} - ($out{$act}{first_angle} // $out{$act}{angle});
      $joint_pos = joint_delta_for($joint_name, $act, $delta);
      $notes = "angle";
    } else {
      $raw_pos = $out{$act}{integrated_pos};
      $joint_pos = joint_delta_for($joint_name, $act, $raw_pos);
      $notes = "velocity-integrated";
    }
    $raw_vel = $out{$act}{velocity};
    $joint_vel = joint_delta_for($joint_name, $act, $raw_vel);
    $notes .= " node=$out{$act}{node}";
  } else {
    $src = "-";
    $notes = "NO_FRAMES";
  }

  my $red = exists($simple_joint_reduction{$joint_name}) ? $simple_joint_reduction{$joint_name}
          : exists($fourbar_joint_reduction{$joint_name}) ? $fourbar_joint_reduction{$joint_name}
          : "-";
  printf "%-24s %-16s %-8s %-10s %12s %12s %12s %s\n",
    $joint_name, $act // "-", $src, $red,
    fmt($joint_pos, "rad", 11, 4),
    fmt($joint_vel, "rad/s", 10, 4),
    fmt($raw_pos, "rad", 11, 4),
    $notes;
}

print "\nNoFW output actuator status\n";
print "--------------------------\n";
printf "%-16s %-4s %-12s %-12s %-10s %-8s %-9s %-9s %-5s %-8s %-8s %-8s\n",
  "Actuator", "Node", "Angle", "Velocity", "FWGear", "Profile", "Mode", "Armed", "Need", "FOCcal", "OUTcal", "Fault";
for my $act (sort keys %out) {
  my $s = $out{$act};
  my $profile = defined $s->{active_profile} ? ($profile_name{$s->{active_profile}} // $s->{active_profile})
              : defined $s->{stored_profile} ? ($profile_name{$s->{stored_profile}} // $s->{stored_profile})
              : "-";
  my $mode = defined $s->{diag_default_mode} ? ($control_mode{$s->{diag_default_mode}} // $s->{diag_default_mode})
           : defined $s->{default_mode} ? ($control_mode{$s->{default_mode}} // $s->{default_mode})
           : "-";
  printf "%-16s %-4s %12s %12s %10s %-8s %-9s %-9s %-5s %-8s %-8s %-8s\n",
    $act,
    $s->{node} // "-",
    fmt($s->{angle}, "rad", 11, 4),
    fmt($s->{velocity}, "rad/s", 10, 4),
    defined($s->{fw_gear_ratio}) ? sprintf("%.3f", $s->{fw_gear_ratio}) : "-",
    $profile,
    $mode,
    defined($s->{armed}) ? $s->{armed} : "-",
    defined($s->{need_cal}) ? $s->{need_cal} : "-",
    defined($s->{foc_cal_valid}) ? $s->{foc_cal_valid} : "-",
    defined($s->{output_cal_valid}) ? $s->{output_cal_valid} : "-",
    defined($s->{runtime_fault}) ? $s->{runtime_fault} : "-";
}

print "\nAK servo status\n";
print "---------------\n";
printf "%-10s %-16s %-12s %-12s %-9s %-7s %s\n",
  "Motor", "Actuator", "RawPos", "RawVel", "Current", "Temp", "Error";
for my $act (sort keys %ak) {
  my $s = $ak{$act};
  my $err_name = $ak_error{$s->{error}} // "Unknown";
  printf "%-10s %-16s %12s %12s %8.2fA %6.0fC %u (%s)\n",
    $s->{motor}, $act,
    fmt($s->{pos}, "rad", 11, 4),
    fmt($s->{vel}, "rad/s", 10, 4),
    $s->{current}, $s->{temp}, $s->{error}, $err_name;
}

print "\nFrame timing\n";
print "------------\n";
for my $key (sort keys %period) {
  print "$key: ", period_text($period{$key}), "\n";
}

if ($print_raw) {
  print "\nRaw matching frames\n";
  print "-------------------\n";
  for my $key (sort keys %raw) {
    print "[$key]\n";
    print "  $_\n" for @{$raw{$key}};
  }
}
PERL
