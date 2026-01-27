# mr2_can_hardware_interface notes

## Homing topic naming
Homing devices now publish to fixed topic names derived from the joint name. For a joint named `arm_j2`:
- Limit switch state: `can_sensors/arm_j2/limit_switch_state`
- Limit switch fault/watchdog: `can_sensors/arm_j2/limit_switch_state_fault`, `can_sensors/arm_j2/limit_switch_state_watchdog`
- Absolute encoder angle: `can_sensors/arm_j2/absolute_encoder`
- Absolute encoder raw/flags/watchdog: `can_sensors/arm_j2/absolute_encoder_raw`, `can_sensors/arm_j2/absolute_encoder_flags`, `can_sensors/arm_j2/absolute_encoder_watchdog`

Topic overrides (state_topic/fault_topic/etc.) have been removed to keep configurations consistent across robots and demos.

## SiK simulation PTY pair (overview)
In simulation the launcher creates a PTY pair using `socat`:
- Bridge side: `sik_sim_device` (default `/tmp/sik_sim0`)
- External side: `sik_sim_peer` (default `/tmp/sik_sim1`)

Connect with e.g. `screen /tmp/sik_sim1 57600` to interact with the simulated radio.
