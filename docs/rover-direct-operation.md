# Rover-direct operation

This mode removes the separate base-station computer. The Jetson Orin Nano
runs the rover ROS 2 stack, gateway, video relay, dashboard, nginx, and Wi-Fi
access point as native host processes.

Docker is only a laptop development environment. It is not part of the Jetson
runtime.

## Topology

```text
Laptop browser
  |
  | HTTPS / WSS over the Jetson AP (192.168.2.102)
  v
Jetson nginx :443
  |-- /xbee-ws -----------------> gateway 127.0.0.1:8081
  |-- /video/streams,/video-ws -> gateway 127.0.0.1:8081
  `-- /rosbridge-ws -----------> rosbridge 127.0.0.1:9090

gateway /run/mr2/xbee_gateway
  <---- socat PTY pair ---->
rover bridge /run/mr2/xbee_rover

rover cameras -- RTP/H.264 --> 127.0.0.1 UDP ports --> gateway --> browser
```

The direct ROS launch starts real ros2_control, drive/steering, battery
monitoring, MoveIt Servo, the XBEE bridge, rosbridge, and rover video encoding.
It does not start dual GNSS, NTRIP, localization, Nav2/autonomy, science,
panorama, or autonomous perception.

The gateway `rover-direct` profile retains XBEE control/telemetry and video
relay. It disables MAVProxy, base antenna tracking, Rocket M2 polling, and the
base GNSS/RTCM ROS relay.

The dashboard `rover-direct` profile exposes Status, Live Feed, and Arm views.
The persistent drive/steering controls remain available. Multiple browsers may
connect and send commands; this mode does not add an exclusive controller
lease.

## Jetson prerequisites

- ROS 2 Humble and the repository's normal rover hardware dependencies
- Node.js `20.19+` or `22.12+`
- nginx, OpenSSL, rsync, socat, and NetworkManager (`nmcli`)
- GStreamer plus Jetson `nvv4l2h264enc`/`nvvidconv`
- a built rover workspace at `rover/ros2_ws/install/setup.bash`
- configured `can0` and camera udev aliases from the existing rover setup

Build the ROS workspace natively on the Jetson:

```bash
cd rover/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src --rosdistro humble
colcon build --symlink-install
```

Do not use the laptop development container for this build or runtime.

## Configure the Jetson AP

The default AP address is `192.168.2.102/24`, chosen to avoid the existing
`192.168.1.0/24` network. Select the actual Jetson Wi-Fi interface and provide
the WPA2 password outside the repository:

```bash
sudo --preserve-env=MR2_AP_PASSWORD \
  MR2_AP_PASSWORD='replace-with-a-private-password' \
  MR2_AP_INTERFACE=wlan0 \
  ./scripts/configure_rover_ap.bash
```

If `MR2_AP_PASSWORD` is omitted in an interactive terminal, the script prompts
without echoing it. NetworkManager stores the resulting connection as
`MR2-Rover-AP` and brings it up with IPv4 shared mode.

## Install the native services

From the repository root on the Jetson:

```bash
./scripts/install_rover_direct.bash
```

This installs the direct dashboard build, nginx configuration/certificate, and
three systemd units, but does not start or enable the hardware-facing services.
Review CAN and camera readiness first.

Enable at boot without starting now:

```bash
./scripts/install_rover_direct.bash --skip-build --enable
```

Enable and start:

```bash
./scripts/install_rover_direct.bash --skip-build --start
```

The units are:

- `mr2-xbee-sim.service`: owns the local PTY pair
- `mr2-rover-direct.service`: runs `rover_direct.launch.py`
- `mr2-gateway-direct.service`: runs the co-located gateway/video relay

Inspect them with:

```bash
systemctl status mr2-xbee-sim mr2-rover-direct mr2-gateway-direct
journalctl -u mr2-rover-direct -u mr2-gateway-direct -f
```

After joining the `MR2-Rover` Wi-Fi network, open:

```text
https://192.168.2.102
```

The certificate is self-signed. Install/trust it on the operator laptop if
browser warnings are unacceptable.

## Manual native start

For maintenance without systemd, create the PTYs first:

```bash
sudo install -d -o "$USER" -g "$(id -gn)" /run/mr2
socat -d -d \
  pty,raw,echo=0,link=/run/mr2/xbee_rover \
  pty,raw,echo=0,link=/run/mr2/xbee_gateway
```

Then use separate terminals:

```bash
./scripts/run_rover_direct.bash
```

```bash
cd base/gateway
npm start -- \
  --gateway-profile rover-direct \
  --base-xbee-device /run/mr2/xbee_gateway \
  --gateway-host 127.0.0.1
```

The rover launch can create its own PTY pair for a short manual test with
`start_xbee_sim:=true`, but production systemd keeps PTY ownership separate so
the rover and gateway can restart independently.

## Scope and hardware validation

This mode intentionally leaves the existing XBEE protocol, drive command
behavior, arm Servo command path, software E-stop UI, and command deadman
semantics unchanged. Additional safety interlocks and multi-browser command
arbitration are separate work.

Before field operation, validate on the actual Jetson and rover:

- AP association and the fixed `192.168.2.102` address
- browser connections through all four nginx paths
- every configured camera stream and browser H.264 decode
- controller states with `ros2 control list_controllers`
- zero-command CAN startup, then deliberate drive/steering and arm tests
- reconnect behavior after restarting each native service
