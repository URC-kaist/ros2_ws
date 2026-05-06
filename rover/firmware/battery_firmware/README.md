# MR2 Misc Firmware

## System overview
The firmware targets an STM32H523 MCU and mediates between a Makita XGT battery
pack and an on-vehicle CAN network. The battery is polled over the single-wire
diagnostic bus using USART2 (configured for half-duplex, inverted 9600 8E1), and
the measured pack data is republished as a set of FDCAN frames on FDCAN1
(classic CAN, 500 kbit/s nominal).

The cooperative state machine inside `Core/Src/battery.c` runs in the foreground
polling loop and performs every step required to wake the pack, issue Makita
commands, parse replies, and broadcast the resulting telemetry.

## Hardware configuration

### MCU resources
- **USART2 (PA2)** – Makita single-wire bus. CubeMX configures PA2 as open-drain
  alternate function with the TX/RX inverters enabled, the TX FIFO at 1/8, and
  half-duplex mode selected. Connect PA2 to the pack data line through the level
  shifting/protection circuitry used previously on USART1. The pin idles high
  so an external pull-up is still required.
- **FDCAN1 (PA11 = RX, PA12 = TX)** – CAN bus interface. The CAN transceiver
  should be powered from 3.3 V (or the appropriate level for your transceiver)
  and wired to the vehicle's CANH/CANL. Bit timing is 500 kbit/s nominal with the
  default CubeMX settings (prescaler = 4, seg1 = 19, seg2 = 5, SJW = 4).
- **Optional logging UART** – `battery_init()` accepts an additional UART handle
  for console output. Leave it `NULL` to disable logs entirely, or pass any
  UART configured for the desired baud rate; the Makita transport always uses
  the first handle.

### Electrical guidance
- Tie MCU ground to pack ground. The single-wire Makita bus expects the STM32 to
  be referenced to the same ground potential as the cells.
- Protect PA2 with an external series resistor, TVS diode, and/or level shifter;
  the pack can swing outside of 0–3.3 V if miswired.
- The CAN lines require the usual 120 Ω termination somewhere on the bus.

## Makita protocol flow

1. **Wake pulse** – A single `0x00` byte is driven on the bus after the idle
   period to wake the pack.
2. **State machine states** – `battery_task()` walks through the following
   states with carefully tuned dwell times:
   - `S_WAKE` issues the wake pulse and waits 70 ms.
   - `S_NUM_CHARGES`, `S_CELL_SIZE`, `S_PARALLEL_CNT`, `S_BATT_HEALTH`,
     `S_CHARGE`, `S_TEMPERATURE`, `S_PACK_VOLT` each send fixed commands defined
     in `CMD_*` arrays to retrieve metadata and aggregate measurements.
   - `S_CELL_VOLTAGES` iterates across all ten cells by adjusting the Makita
     register address (nibble-reversed before TX) and records each cell voltage.
   - `S_COMPLETE` logs summaries (if enabled) and pushes the results to CAN.
3. **UART characteristics** – Transactions use half-duplex 9600 baud, 9-bit word
   length (8 data + even parity), inverted levels, and rely on the HAL to toggle
   TX/RX modes.
4. **Bit reversal and CRC** – Every received byte is bit-reversed to match the
   Makita framing. Two CRC formats are supported: the short `0xCC...0x33` frame
   and the long `0xA5 0xA5` variant. See `check_crc()` for implementation.
5. **Timing** – The driver watches for idle gaps (500 ms for the short frames) to
   decide when a response ends. Each state has a small settle time (50–80 ms) to
   keep the pack happy without blocking the MCU.

## CAN frame protocol

Makita telemetry is republished on FDCAN1 as a fixed set of classic CAN
messages so the rest of the vehicle can remain agnostic to the Makita single
wire link.

### Transport characteristics
- Classical CAN (no FD/BRS) on FDCAN1 at 500 kbit/s nominal, 11-bit identifiers,
  8-byte data fields.
- Each polling cycle ends in `publish_fdcan_state()` and emits the entire
  frame set listed below. The default cadence is one burst every 1 second,
  but it can be adjusted through `s_update_interval_ms`.
- A 32-bit `cycle counter` increments once per burst and is split between the
  summary and metadata frames so consumers can correlate the packets.

### Frame catalogue

| ID    | Name       | Rate\* | Notes                                                             |
|-------|------------|--------|-------------------------------------------------------------------|
| 0x300 | Summary    | 1/`s_update_interval_ms` | High-level SoC, health, temperature, voltage and cycle counter LSBs. |
| 0x301 | Metadata   | same   | Static ratings plus life counters and the cycle counter MSBs.     |
| 0x310 | Cells 1–2  | same   | Two cell reports per frame, ascending cell IDs.                   |
| 0x311 | Cells 3–4  | same   |                                                                   |
| 0x312 | Cells 5–6  | same   |                                                                   |
| 0x313 | Cells 7–8  | same   |                                                                   |
| 0x314 | Cells 9–10 | same   |                                                                   |

\*With the default 1 s interval all frames are emitted once every second.

### Frame layouts

#### Summary (0x300)
- **Byte 0** – State of charge in percent (`u8`). Values above 100 % are
  clipped to 100.
- **Byte 1** – Pack “health” percentage (`u8`) derived from Makita’s
  capacity estimate and the measured parallel count. Values saturate at 100.
- **Bytes 2–3** – Pack temperature as a signed 16-bit integer in
  little-endian tenths of °C (`s16`, `°C × 10`). Example: `0x1E00` = 3.0 °C.
  The firmware bounds the source reading to the ±32 768 range before encoding.
- **Bytes 4–5** – Pack voltage as a little-endian unsigned 16-bit integer in
  centivolts (`u16`, `V × 100`). Rounded to the nearest 0.01 V.
- **Bytes 6–7** – Lower 16 bits of the cycle counter (`cycle[15:0]`), little
  endian. Combine with the metadata frame to recover the full 32-bit value.

#### Metadata (0x301)
- **Bytes 0–1** – Nominal single-cell capacity in milliamp-hours (`u16`
  little endian). The Makita raw value is scaled by 100; for example, a raw
  value of 0x04 becomes bytes `0x90 0x01` (=0x0190=400 mAh per cell).
- **Byte 2** – Parallel group count reported by the pack (`u8`). This tells
  you how many cells are paralleled in each series string.
- **Byte 3** – Cell count (`u8`). Makita XGT packs always report ten series
  cells, but decoding code is flexible if this ever changes.
- **Bytes 4–5** – Lifetime charge cycle count reported by the pack (`u16`
  little endian). This is Makita’s internal counter, not the firmware cycle.
- **Bytes 6–7** – Upper 16 bits of the firmware’s cycle counter
  (`cycle[31:16]`, little endian). Together with the summary frame you can form
  a monotonically increasing 32-bit counter that increments once per CAN burst.

#### Cell voltage frames (0x310–0x314)
Each frame carries two cells with the same structure, so 0x310 contains cells
1 and 2, 0x311 contains 3 and 4, etc. Slots are packed back-to-back:

```
Byte 0 = cell ID (1–10)
Byte 1 = voltage LSB (mV)
Byte 2 = voltage MSB (mV)
Byte 3 = validity flag
Byte 4 = cell ID for the second slot
Byte 5 = voltage LSB
Byte 6 = voltage MSB
Byte 7 = validity flag
```

- Cell IDs are 1-based and always increase with frame ID, so the sequence is
  deterministic even if some readings are marked invalid.
- Voltages are rounded to the nearest millivolt and encoded as unsigned
  little-endian values. Out-of-range readings are saturated to 0–65 535 mV,
  though real packs stay in the 0–5000 mV span.
- The validity flag is set to `1` when a positive voltage was available for the
  cell. When `0`, consumers should treat the slot as stale or absent; the rest
  of the slot may be left at zero by the firmware.

These frames give downstream ECUs all high-level metrics plus per-cell detail
without needing to speak Makita’s single-wire protocol.

## Repository layout
- `Core/` – CubeMX-generated sources plus the Makita driver (`Src/battery.c`,
  `Inc/battery.h`).
- `Drivers/` – STM32 HAL and CMSIS dependencies.
- `cmake/` – Toolchain files and CubeMX/CMake glue.
- `startup_stm32h523xx.s`, `STM32H523xx_*.ld` – Startup code and linker scripts.
- `battery_firmware.ioc` – CubeMX configuration for reference/regeneration.

## Build and flash
Enter the Nix development shell:

```sh
nix develop path:$PWD
```

The project uses CMake presets that pull in `cmake/gcc-arm-none-eabi.cmake`.

```sh
cmake --preset Debug
cmake --build --preset Debug
```

The build emits `build/Debug/battery_firmware.elf`,
`build/Debug/battery_firmware.hex`, and `build/Debug/battery_firmware.bin`.

Flash with probe-rs:

```sh
cmake --build --preset Debug --target flash
```

The flash target uses probe-rs chip name `STM32H523CE`.

## Operation checklist
1. Wire PA2 to the Makita diagnostic pin with the proper voltage conditioning.
2. Attach the CAN transceiver to PA11/PA12 and the CAN backbone.
3. (Optional) Connect a UART to any other port if console logs are desired and
   pass its handle to `battery_init()`.
4. Power the MCU and the battery pack; the firmware will automatically start
   polling every second, publishing the frames described above.

## Customisation
- Adjust polling cadence by editing `s_update_interval_ms` in `battery.c`.
- Add new Makita commands by extending the `CMD_*` array list and inserting a
  new state in `battery_task()`.
- Change CAN identifiers or payload formats by editing the constants and
  `publish_fdcan_state()`.

## Licensing
CubeMX-generated sources keep the STMicroelectronics license headers. Add any
project-specific license text here if necessary.
