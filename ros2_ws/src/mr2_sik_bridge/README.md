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
- `0x10` TELEM_BATTERY

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
- If heartbeats stop for the deadman timeout window, publish zero drive and
  arm twist commands continuously until heartbeats resume.

### TELEM_BATTERY (msg_id 0x10)
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

Suggested rate: 1–2 Hz or on change.

## Library API

Header: `mr2_sik_bridge/packets.hpp`

Key functions:
- `encode_cmd_drive`, `encode_cmd_arm_twist`, `encode_heartbeat`,
  `encode_telem_battery`
- `decode_frame` (validates magic, size, CRC)
- `decode_cmd_drive`, `decode_cmd_arm_twist`, `decode_heartbeat`,
  `decode_telem_battery`

## Behavior Expectations (out of scope for this package)

This package does not implement serial I/O, timeouts, or ROS publishers.
A higher-level SiK bridge node should:
- Enforce heartbeat timeouts and publish zero commands on loss.
- Map decoded drive/arm commands into ROS topics.
- Convert ROS battery telemetry into the compact payload above.
