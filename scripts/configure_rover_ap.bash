#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run as root: sudo --preserve-env=MR2_AP_PASSWORD $0" >&2
  exit 1
fi

connection_name=${MR2_AP_CONNECTION:-MR2-Rover-AP}
interface=${MR2_AP_INTERFACE:-wlan0}
ssid=${MR2_AP_SSID:-MR2-Rover}
address=${MR2_DIRECT_AP_CIDR:-192.168.2.102/24}
band=${MR2_AP_BAND:-bg}
password=${MR2_AP_PASSWORD:-}

if [[ -z $password ]]; then
  if [[ ! -t 0 ]]; then
    echo "MR2_AP_PASSWORD is required when stdin is not interactive." >&2
    exit 1
  fi
  read -r -s -p "WPA2 password for ${ssid}: " password
  echo
fi

if (( ${#password} < 8 || ${#password} > 63 )); then
  echo "The WPA2 password must contain 8 to 63 characters." >&2
  exit 1
fi

if ! command -v nmcli >/dev/null 2>&1; then
  echo "nmcli is required. Install and enable NetworkManager first." >&2
  exit 1
fi

if ! nmcli -t -f DEVICE device status | cut -d: -f1 | grep -Fxq "$interface"; then
  echo "NetworkManager interface not found: $interface" >&2
  exit 1
fi

if ! nmcli -t -f NAME connection show | grep -Fxq "$connection_name"; then
  nmcli connection add \
    type wifi \
    ifname "$interface" \
    con-name "$connection_name" \
    autoconnect yes \
    ssid "$ssid"
fi

nmcli connection modify "$connection_name" \
  connection.interface-name "$interface" \
  connection.autoconnect yes \
  802-11-wireless.mode ap \
  802-11-wireless.band "$band" \
  802-11-wireless.ssid "$ssid" \
  802-11-wireless-security.key-mgmt wpa-psk \
  802-11-wireless-security.psk "$password" \
  ipv4.method shared \
  ipv4.addresses "$address" \
  ipv6.method disabled

nmcli connection up "$connection_name"

echo "Rover AP is active: SSID=${ssid}, address=${address}, interface=${interface}"
