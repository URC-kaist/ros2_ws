# MR2 SiK Bridge

This package provides encoding/decoding utilities for the MR2 SiK serial
protocol and a ROS 2 bridge node that interfaces with the serial link. The
packet helpers focus on framing, CRC verification, and translation between
binary payloads and strongly-typed C++ structs.

## Framing

All packets use a compact binary frame:

- `uint8  magic` = `0xA5`
- `uint8  msg_id`
- `uint8  length` (payload bytes, not including header or CRC)
- `uint8  seq` (wraps at 255)
- `uint8[] payload` (length = `length`)
- `uint16 crc16` (CRC-16/CCITT-FALSE over header+payload)

Notes:
- Endianness: little-endian for all multi-byte fields.
- CRC polynomial: 0x1021, init 0xFFFF, xorout 0x0000, no reflection.
- Frames are fixed-size for a given message type because payload sizes are
  fixed.

## Message IDs

- `0x01` CMD_DRIVE
- `0x02` CMD_ARM_TWIST
- `0x03` HEARTBEAT
- `0x10` TELEM_BATTERY_1
- `0x11` TELEM_BATTERY_2
- `0x20` TELEM_NAV
- `0x30` BASE_SVIN (survey-in ECEF + validity)
- `0x31` BASE_RTCM (raw RTCM byte payload)

## Payloads (ROS-aligned units)

### CMD_DRIVE (msg_id 0x01)
Payload size: 16 bytes

- `uint32 timestamp_ms`
- `float32 linear_x_m_s`
- `float32 linear_y_m_s`
- `float32 angular_z_rad_s`

ROS mapping: `geometry_msgs/Twist`
- `linear.x = linear_x_m_s`
- `linear.y = linear_y_m_s`
- `angular.z = angular_z_rad_s`

Suggested rate: 20–50 Hz
Soft stop: send zeros for linear/angular.

### CMD_ARM_TWIST (msg_id 0x02)
Payload size: 28 bytes

- `uint32 timestamp_ms`
- `float32 lin_x_m_s`
- `float32 lin_y_m_s`
- `float32 lin_z_m_s`
- `float32 ang_x_rad_s`
- `float32 ang_y_rad_s`
- `float32 ang_z_rad_s`

ROS mapping: `geometry_msgs/TwistStamped`
- `twist.linear.{x,y,z}` and `twist.angular.{x,y,z}` use the matching fields.

Suggested rate: 20–50 Hz
Soft stop: send all zeros.

### HEARTBEAT (msg_id 0x03)
Payload size: 4 bytes

- `uint32 timestamp_ms`

Suggested rate: 2–5 Hz
Expected behavior in higher-level code:
- Dashboard sends heartbeats to the bridge; if they stop for the deadman timeout
  window, publish zero drive and arm twist commands continuously until they resume.
- The ROS bridge also emits heartbeats back over the SiK link to indicate the
  bridge is alive (used by the gateway/UI link status).

### TELEM_BATTERY_1 / TELEM_BATTERY_2 (msg_id 0x10 / 0x11)
Payload size: 16 bytes

- `float32 total_capacity_mah`
- `float32 available_capacity_mah`
- `float32 temperature_c`
- `float32 pack_voltage_v`

Derived from existing telemetry:
- `total_capacity_mah = nominal_cell_capacity_mah * parallel_group_count`
- `available_capacity_mah = total_capacity_mah * state_of_charge_pct / 100`
- `temperature_c` rounded or passed through as whole degrees
- `pack_voltage_v = pack_voltage_v`

Battery index is implied by the message ID:
- `0x10` = battery 1
- `0x11` = battery 2

Suggested rate: 1–2 Hz or on change.

### TELEM_NAV (msg_id 0x20)
Payload size: 32 bytes

- `uint32 timestamp_ms`
- `float32 latitude_deg`
- `float32 longitude_deg`
- `float32 altitude_m`
- `float32 heading_deg`
- `float32 cov_x_var`
- `float32 cov_y_var`
- `float32 cov_yaw_var`

Derived from:
- `/gps/filtered` (NavSatFix) for lat/lon/alt
- `/odometry/filtered/global` (Odometry) for heading and covariance

Suggested rate: 1–5 Hz.

### BASE_SVIN (msg_id 0x30)
Payload size: 25 bytes

- `int32 mean_x_cm`, `mean_y_cm`, `mean_z_cm`
- `int8 mean_x_hp`, `mean_y_hp`, `mean_z_hp`
- `uint8 valid`, `uint8 active`
- `uint32 mean_acc_0p1mm`
- `uint32 obs`

Maps directly to `ublox_ubx_msgs/UBXNavSvin` fields and is published on `/base/ubx_nav_svin` on the rover.

### BASE_RTCM (msg_id 0x31)
Payload size: 1 + N bytes

- `uint8 length` (N, 0–254)
- `uint8[N] data` (raw RTCM frame)

Delivered to the rover as `rtcm_msgs/Message` on `/base/rtcm`.

## Library API

Header: `mr2_sik_bridge/packets.hpp`

Key functions:
- `encode_cmd_drive`, `encode_cmd_arm_twist`, `encode_heartbeat`,
  `encode_telem_battery` (battery 1 by default) / `encode_telem_battery` with
  `battery_id`, `encode_telem_nav`
- `decode_frame` (validates magic, size, CRC)
- `decode_cmd_drive`, `decode_cmd_arm_twist`, `decode_heartbeat`,
  `decode_telem_battery`, `decode_telem_nav`

## Behavior Expectations (out of scope for this package)

This package does not implement serial I/O, timeouts, or ROS publishers.
A higher-level SiK bridge node should:
- Enforce heartbeat timeouts and publish zero commands on loss.
- Map decoded drive/arm commands into ROS topics.
- Convert ROS battery telemetry into the compact payload above.
