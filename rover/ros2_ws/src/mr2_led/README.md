# mr2_led

CAN-controlled LED driver for the MR2 NeoPixel firmware. The node exposes a ROS 2
service that sends a single CAN frame to set the LED mode.

## Quick start

```bash
source install/setup.bash            # after colcon build
ros2 launch mr2_led led_can.launch.py can_iface:=can0 can_id:=0x123
# In another terminal (sourced):
ros2 service call /set_led_mode mr2_led/srv/SetLedMode "{mode: 2}"
```

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

## Mission status bridge

`mission_status_led_node` subscribes to `/mission_status` and calls
`/set_led_mode` to reflect Mission Master state.

Mapping:

| Condition                                  | LED mode    |
|--------------------------------------------|-------------|
| No `/mission_status` (timeout)             | OFF         |
| `state` = IDLE (0) or COMPLETED (3)        | OFF         |
| `state` = RUNNING (1)                      | AUTONOMOUS  |
| `state` = PAUSED (2)                       | MANUAL      |
| `arrival` = true                           | SUCCESS     |

Parameters:
- `mission_status_topic` (string, default: `/mission_status`)
- `led_service_name` (string, default: `set_led_mode`)
- `status_timeout_sec` (double, default: `1.0`)

Run:

```sh
ros2 run mr2_led mission_status_led_node
```

## Parameters

- `can_iface` (string, default: `can0`) SocketCAN interface.
- `can_id` (int, default: `0x123`) Standard 11-bit CAN ID.

## Launch options

```sh
ros2 launch mr2_led led_can.launch.py can_iface:=can0 can_id:=0x123
```

## Development / build

From the workspace root:

```bash
colcon build --packages-select mr2_led
source install/setup.bash
```

The package depends on `mr2_can_bus_core` for SocketCAN transport and `rosidl_default_generators` for the service type.

## Testing without hardware

Set up a virtual CAN bus and run the node against it:

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
source install/setup.bash
ros2 launch mr2_led led_can.launch.py can_iface:=vcan0
ros2 service call /set_led_mode mr2_led/srv/SetLedMode "{mode: 3}"
```

This lets you exercise the service interface even when no real CAN interface is present.

## Troubleshooting

- `Invalid CAN ID` at startup: ensure `can_id` is within 0–0x7FF (11-bit standard frame).
- `Failed to acquire CAN bus`: confirm the `can_iface` exists and is `UP` (`ip link show can0`).
