# LED Firmware (CAN-Controlled NeoPixels)

Firmware for an STM32 NUCLEO-G431KB that listens on CAN and drives a NeoPixel
strip. Incoming CAN frames select a display mode (solid color or green blink).

## Hardware

- **Board:** NUCLEO-G431KB only (compile-time guard in the sketch).
- **NeoPixel data pin:** `D3` (change `NEOPIXEL_PIN` if needed).
- **LED count:** `256` (change `NUM_PIXELS` to match your strip).

## CAN configuration

- **Library:** `ACANFD_STM32`
- **Bus type:** Classic CAN (no FD data phase), 500 kbps nominal.
- **Acceptance filter:** Only standard ID `0x123` is accepted into FIFO0.

## CAN protocol

This firmware **receives only**; it does not transmit CAN frames.

### Control frame

- **CAN ID:** `0x123` (11-bit standard)
- **DLC:** 1+ (requires at least one byte)
- **Data[0]:** mode selector

| Data[0] | Mode             | Effect                                              |
|--------:|------------------|-----------------------------------------------------|
| 0       | Off              | All pixels off                                      |
| 1       | Red              | Solid red (RGB = 50, 0, 0)                           |
| 2       | Blue             | Solid blue (RGB = 0, 0, 50)                          |
| 3       | Green Blink      | Blink green at 500 ms (RGB = 0, 50, 0)               |

Values above 3 are ignored.

### Blink behavior

- Blink mode toggles every 500 ms.
- The blink starts in the "off" state and then alternates on/off.

## Runtime behavior

- On boot, the strip is cleared and set to **Green Blink** by default.
- CAN frames are polled continuously; mode changes apply immediately.
- Serial output at 115200 baud logs FDCAN init status and a brief hint.

## Configuration points

- `NEOPIXEL_PIN`, `NUM_PIXELS`
- `CAN_CONTROL_ID` (default `0x123`)
- `BLINK_PERIOD_MS` (default 500 ms)

## Notes

- If you change the CAN ID, update both the filter and your sender.
- Brightness is set to 255; adjust in `setup()` if needed.
