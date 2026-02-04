# mr2_led

CAN-controlled LED driver for the MR2 NeoPixel firmware. The node exposes a ROS 2
service that sends a single CAN frame to set the LED mode.

## CAN protocol

- **CAN ID:** 0x123 (standard 11-bit)
- **DLC:** 1
- **Data[0]:** mode selector

| Mode name   | Value | LED behavior                         |
|-------------|-------|--------------------------------------|
| OFF         | 0     | All pixels off                       |
| AUTONOMOUS  | 1     | Solid red                            |
| MANUAL      | 2     | Solid blue (teleoperation)           |
| SUCCESS     | 3     | Flashing green (arrived at target)   |

Values above 3 are rejected by the service.

## ROS 2 service

Service type: `mr2_led/srv/SetLedMode`

Request:
- `uint8 mode`

Response:
- `bool success`
- `string message`

Example:

```sh
ros2 service call /set_led_mode mr2_led/srv/SetLedMode "{mode: 1}"
```

## Parameters

- `can_iface` (string, default: `can0`) SocketCAN interface.
- `can_id` (int, default: `0x123`) Standard 11-bit CAN ID.

## Launch

```sh
ros2 launch mr2_led led_can.launch.py
```
