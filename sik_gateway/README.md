# MR2 SiK Gateway (Headless Node.js)

This service bridges SiK serial frames to the dashboard using WebSocket.
It implements the MR2 SiK protocol described in `ros2_ws/src/mr2_sik_bridge/README.md`.

## Install

```bash
cd sik_gateway
npm install
```

## Run

```bash
npm start -- --device /dev/ttyUSB0 --baud 57600 --port 8081 --heartbeat-hz 2
```

You can also configure with environment variables:

- `SIK_DEVICE`
- `SIK_BAUD`
- `SIK_WS_PORT`
- `SIK_HEARTBEAT_HZ`

## WebSocket API

Incoming (dashboard -> gateway):
- `cmd_drive` { `linear_x_m_s`, `angular_z_rad_s` } (aliases: `x`, `yaw`)
- `cmd_arm_twist` { `lin_x_m_s`, `lin_y_m_s`, `lin_z_m_s`, `ang_x_rad_s`, `ang_y_rad_s`, `ang_z_rad_s` }
- `heartbeat` { }

Outgoing (gateway -> dashboard):
- `telem_battery` { `total_capacity_mah`, `available_capacity_mah`, `temperature_c` }
- `link_status` { `connected`, `last_rx_ms`, `last_tx_ms` }
