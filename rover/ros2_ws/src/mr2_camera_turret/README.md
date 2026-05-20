# mr2_camera_turret

Classic CAN driver for the STM32 camera turret receiver.

The node subscribes to normalized X/Y turret commands and sends the receiver's
4-byte joystick payload on a standard 11-bit classic CAN frame.

## Quick start

```bash
colcon build --packages-select mr2_can_bus_core mr2_camera_turret
source install/setup.bash
ros2 launch mr2_camera_turret camera_turret_can.launch.py can_iface:=can0
```

Publish a centered command:

```bash
ros2 topic pub --once /camera_turret/command geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"
```

## ROS interface

- Topic: `/camera_turret/command`
- Type: `geometry_msgs/msg/Vector3`
- `x`: X-axis command, normalized to `[-1.0, 1.0]`
- `y`: Y-axis tilt command in radians, clamped to `[-0.3, 0.8]` by default
- `z`: ignored

The node clamps out-of-range inputs and maps command values to 12-bit joystick
ADC values:

| Normalized | ADC |
|------------|-----|
| -1.0       | 0   |
| 0.0        | 2048 |
| 1.0        | 4095 |

## CAN protocol

- CAN ID: `0x124` standard 11-bit
- Frame type: classic CAN 2.0, 500 kbps
- Length: 4 bytes

| Byte | Field |
|------|-------|
| 0    | `vrx` LSB (`y` / tilt command) |
| 1    | `vrx` MSB (`y` / tilt command) |
| 2    | `vry` LSB (`x` / pan command) |
| 3    | `vry` MSB (`x` / pan command) |

## Parameters

- `can_iface` (string, default: `can0`)
- `can_id` (int, default: `0x124`)
- `command_topic` (string, default: `/camera_turret/command`)
- `publish_rate_hz` (double, default: `50.0`)
- `invert_x` (bool, default: `false`)
- `invert_y` (bool, default: `false`)
- `tilt_min_rad` (double, default: `-0.3`)
- `tilt_max_rad` (double, default: `0.8`)

The node keeps sending the latest command while running. On shutdown, it sends
centered X/Y values.
