# MR2 Gateway (Headless Node.js)

This service bridges SiK serial frames to the dashboard using WebSocket, and
hosts a small HTTP endpoint for Transitive JWT minting.
It implements the MR2 SiK protocol described in `ros2_ws/src/mr2_sik_bridge/README.md`.

## Install

```bash
cd base_gateway
npm install
```

## Run

```bash
npm start -- --device /dev/ttyUSB0 --baud 57600 --port 8081 --heartbeat-hz 2 --cmd-timeout-ms 500
```

You can also configure with environment variables:

- `SIK_DEVICE`
- `SIK_BAUD`
- `SIK_WS_PORT`
- `SIK_HEARTBEAT_HZ`
- `SIK_CMD_HZ`
- `SIK_CMD_TIMEOUT_MS`

## WebSocket API

Incoming (dashboard -> gateway):
- `cmd_drive` { `linear_x_m_s`, `linear_y_m_s`, `angular_z_rad_s` } (aliases: `x`, `y`, `yaw`)
- `cmd_arm_twist` { `lin_x_m_s`, `lin_y_m_s`, `lin_z_m_s`, `ang_x_rad_s`, `ang_y_rad_s`, `ang_z_rad_s` }
- `heartbeat` { }

Outgoing (gateway -> dashboard):
- `telem_battery` { `total_capacity_mah`, `available_capacity_mah`, `temperature_c`, `pack_voltage_v` }
- `link_status` { `connected`, `last_rx_ms`, `last_tx_ms` }

Link status uses recent heartbeat frames from the ROS bridge; it will report
`connected: false` if no heartbeat is received within the timeout window
(`SIK_LINK_TIMEOUT_MS`, default 2000 ms).

## Transitive token endpoint

The gateway can mint Transitive JWTs for the dashboard at:

- `GET /transitive/token`

Environment variables (required on the gateway host):
- `TRANSITIVE_JWT_SECRET` (required)
- `TRANSITIVE_ID` (default: `unknown`)
- `TRANSITIVE_DEVICE` (default: `unknown`)
- `TRANSITIVE_CAPABILITY` (default: `@transitive-robotics/webrtc-video`)
- `TRANSITIVE_USER_ID` (default: `operator`)
- `TRANSITIVE_VALIDITY` (default: `86400`)

You can also override values per request with query params:
`?id=...&device=...&capability=...&userId=...&validity=...`
