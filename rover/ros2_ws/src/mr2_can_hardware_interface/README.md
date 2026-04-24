# mr2_can_hardware_interface notes

## Joint origin modes
Position joints support two behaviors selected from the ros2_control joint parameters:
- `origin_offset` present: boot-origin mode. The first valid actuator position captured after activation becomes the boot reference, and the logical joint state is reported relative to that reference plus `origin_offset`.
- `origin_offset` absent: direct mode. Joint state and commands follow the actuator directly with no startup capture.

Boot-origin mode keeps the actuator at its startup pose until a controller sends a command.

## XBEE simulation PTY pair (overview)
In simulation the launcher creates a PTY pair using `socat`:
- Bridge side: `xbee_sim_device` (default `/tmp/xbee_sim0`)
- External side: `xbee_sim_peer` (default `/tmp/xbee_sim1`)

Connect with e.g. `screen /tmp/xbee_sim1 57600` to interact with the simulated radio.
