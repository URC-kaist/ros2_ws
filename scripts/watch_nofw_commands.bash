#!/usr/bin/env bash
set -euo pipefail

iface="${1:-can0}"

if ! command -v candump >/dev/null 2>&1; then
  echo "candump not found. Install can-utils." >&2
  exit 1
fi

if ! ip link show "${iface}" >/dev/null 2>&1; then
  echo "CAN interface '${iface}' not found or not accessible." >&2
  exit 1
fi

echo "Watching NoFW command/diagnostic frames on ${iface}"
echo "  angle cmd: 0x201..0x204"
echo "  velocity cmd: 0x215..0x218"
echo "  profile cmd: 0x221..0x228"
echo "  power cmd: 0x231..0x238"
echo "  diagnostic: 0x5F1..0x5F8"
echo

candump -L "${iface}" | perl -ne '
  next unless /^\(([^)]+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)/;
  my ($ts, $bus, $id_hex, $data) = ($1, $2, uc($3), uc($4));
  my $id = hex($id_hex);
  my $kind = "";
  if ($id >= 0x201 && $id <= 0x204) {
    $kind = "ANGLE_CMD";
  } elsif ($id >= 0x215 && $id <= 0x218) {
    $kind = "VELOCITY_CMD";
  } elsif ($id >= 0x221 && $id <= 0x228) {
    $kind = "PROFILE_CMD";
  } elsif ($id >= 0x231 && $id <= 0x238) {
    $kind = "POWER_CMD";
  } elsif ($id >= 0x5F1 && $id <= 0x5F8) {
    $kind = "DIAG";
  } else {
    next;
  }

  my $extra = "";
  if ($kind eq "POWER_CMD") {
    $extra = $data eq "01" ? " arm" : $data eq "00" ? " disarm" : "";
  } elsif ($kind eq "PROFILE_CMD") {
    $extra = $data eq "00" ? " VelocityOnly" : $data eq "01" ? " As5600" : "";
  } elsif ($kind eq "DIAG" && length($data) == 16) {
    my @b = map { hex($_) } ($data =~ /../g);
    my $armed = ($b[7] & 0x02) ? 1 : 0;
    my $fault = $b[5];
    my $need = ($b[6] & 0x01) ? 1 : 0;
    $extra = " stored=$b[1] active=$b[2] fault=$fault need_cal=$need armed=$armed";
  }
  print "$ts $bus $id_hex $kind#$data$extra\n";
'
