# LED and Servo Firmware

Firmware for an STM32 NUCLEO-G431KB that listens on Classic CAN and drives a
NeoPixel strip plus two servo PWM outputs. Incoming CAN frames select an LED
display mode or command joystick-like X/Y servo positions.

## Hardware

- **Board:** NUCLEO-G431KB only (compile-time guard in the sketch).
- **NeoPixel data pin:** `D3` (change `NEOPIXEL_PIN` if needed).
- **LED count:** `256` (change `NUM_PIXELS` to match your strip).
- **X servo pin:** `PA8` / `TIM1_CH1`.
- **Y servo pin:** `PA9` / `TIM1_CH2`.

## CAN configuration

- **Library:** `ACANFD_STM32`
- **Bus type:** Classic CAN (no FD data phase), 500 kbps nominal.
- **Acceptance filter:** Standard IDs `0x123` and `0x124` are accepted into FIFO0.

## CAN protocol

This firmware **receives only**; it does not transmit CAN frames.

### LED control frame

- **CAN ID:** `0x123` (11-bit standard)
- **DLC:** 1+ (requires at least one byte)
- **Data[0]:** mode selector

| Data[0] | Mode             | Effect                                              |
|--------:|------------------|-----------------------------------------------------|
| 0       | Off              | All pixels off                                      |
| 1       | Red              | Solid red (RGB = 65, 0, 0)                          |
| 2       | Blue             | Solid blue (RGB = 0, 0, 65)                         |
| 3       | Green Blink      | Blink green at 500 ms (RGB = 0, 65, 0)              |

Values above 3 are ignored.

### Blink behavior

- Blink mode toggles every 500 ms.
- The blink starts in the "off" state and then alternates on/off.

### Servo control frame

- **CAN ID:** `0x124` (11-bit standard)
- **DLC:** 4+ (extra bytes are ignored)
- **Byte order:** little-endian

```text
DATA: [vrx_L, vrx_H, vry_L, vry_H, ...]
```

| Bytes | Name | Type | Description |
|---|---|---|---|
| `0-1` | `vrx` | `uint16_t` | X-axis command |
| `2-3` | `vry` | `uint16_t` | Y-axis command |

Raw values map to servo angle as:

```cpp
angle = (raw - 2048) * 90.0 / 2048.0;
```

The effective angle is clamped to `-90` to `+90` degrees.

| Raw value | Angle |
|---:|---:|
| `0` | `-90 deg` |
| `1024` | `-45 deg` |
| `2048` | `0 deg` |
| `3072` | `+45 deg` |
| `4096` | `+90 deg` |

Servo output is 50 Hz PWM on `TIM1`:

| Servo | Pin | Angle range | Pulse range |
|---|---|---:|---:|
| X | `PA8` / `TIM1_CH1` | `-90..+90 deg` | `700..2300 us` |
| Y | `PA9` / `TIM1_CH2` | `-90..+90 deg` | `1100..2300 us` |

Examples:

```bash
cansend can0 124#00080008  # center both servos
cansend can0 124#00100008  # X +90 deg, Y center
cansend can0 124#00000000  # X -90 deg, Y -90 deg
cansend can0 124#000C0004  # X +45 deg, Y -45 deg
```

## Runtime behavior

- On boot, the strip is cleared and set to **Green Blink** by default.
- On boot, both servo outputs are started at center position.
- CAN frames are polled continuously; mode changes apply immediately.
- No serial logging is emitted.

## Configuration points

- `NEOPIXEL_PIN`, `NUM_PIXELS`
- `CAN_LED_CONTROL_ID` (default `0x123`)
- `CAN_SERVO_CONTROL_ID` (default `0x124`)
- `BLINK_PERIOD_MS` (default 500 ms)
- Servo pins, pulse limits, and center/range constants in `led_firmware.ino`

## Notes

- If you change a CAN ID, update both the filter and your sender.
- Brightness is set to 255; adjust in `setup()` if needed.
