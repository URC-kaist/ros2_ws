Transport: UART / USB-Serial (e.g., 115200 8N1).  
Frames are sent/received as **binary**.

---

## 1) Frame Format (Common for Host ↔ Device)

### 1.1 Byte Order
- Multi-byte integers are **little-endian**.

### 1.2 Frame Layout
| Field | Size | Description |
|---|---:|---|
| SOF | 2 | Fixed: `0xAA 0x55` |
| LEN | 1 | Byte length from `SEQ` through `PAYLOAD` (CRC excluded) |
| SEQ | 1 | Sequence number (0–255, incremented by sender) |
| CMD | 1 | Command ID |
| PAYLOAD | N | Command-specific payload (N = LEN - 2) |
| CRC16 | 2 | **CRC-16/CCITT-FALSE**, computed over `LEN..PAYLOAD`, stored little-endian |

> CRC is computed over **from LEN to the end of PAYLOAD** (SOF and CRC16 itself are excluded)

---

## 2) CRC Definition

### CRC-16/CCITT-FALSE
- Polynomial: `0x1021`
- Init: `0xFFFF`
- RefIn/RefOut: `false`
- XorOut: `0x0000`
- Process input bytes **as-is, in order**

CRC storage:
- CRC16 low byte first, high byte next (**little-endian**)  
  Example) CRC = 0xBEEF → `EF BE`

---

## 3) Command Definitions (Host → Device)

### 3.1 CMD = 0x01 : HOMING_START
- Meaning: Start the homing sequence (SEARCH → BACKOFF → APPROACH → RELEASE → READY)
- PAYLOAD: none (0 bytes)
- LEN = 2 (SEQ 1 + CMD 1)

---

### 3.2 CMD = 0x02 : MOVE_TO_RAD
- Meaning: Move to a target angle (rad) relative to the zero reference (position control)
- PAYLOAD:
  - `theta_q` (int32, 4B): **Q16.16 fixed-point** radians
    - `theta_q = round(theta_rad * 65536)`
    - Example) +1.0 rad → 65536 → `0x00010000`
    - Example) -0.5 rad → -32768 → `0xFFFF8000`
- LEN = 6 (SEQ + CMD + 4)
  - Note: Target is clamped to ±90° in firmware.

---

## 4) Response Definitions (Device → Host)

### 4.1 CMD = 0x80 : ACK
- Meaning: The request frame was received/parsed correctly and execution has started
- PAYLOAD:
  - `ack_seq` (1B): SEQ being acknowledged (Host request SEQ)
  - `ack_cmd` (1B): CMD being acknowledged
- LEN = 4

### 4.2 CMD = 0x81 : DONE
- Meaning: Operation completed (homing complete / move complete)
- PAYLOAD:
  - `done_seq` (1B): Original request SEQ
  - `done_cmd` (1B): Original request CMD
- LEN = 4

### 4.3 CMD = 0x82 : ERROR
- Meaning: An error occurred
- PAYLOAD:
  - `err_seq` (1B): Related SEQ (0xFF if unknown)
  - `err_code` (1B): Error code
  - `detail` (2B, uint16): Optional (subcode/diagnostics)
- LEN = 6

#### Suggested error codes
- `0x01` CRC_MISMATCH
- `0x02` BAD_LEN
- `0x03` UNKNOWN_CMD
- `0x11` NOT_HOMED
- `0x12` OUT_OF_RANGE
- `0x13` BUSY
- `0x14` HOME_NOT_FOUND

---

## 5) Example Frames (CRC must be calculated by the rules above)
NOTE: Below examples show the “format” only; actual CRC must be computed using the CRC rules above.

### 5.1 Homing Start (Host SEQ = 0x01)
- Hex:
  - `AA 55 02 01 01  <CRCL> <CRCH>`
  - Meaning: SOF, LEN=2, SEQ=1, CMD=1, CRC

### 5.2 Move to +1.000 rad (Host SEQ = 0x02)
- theta_q = 65536 = 0x00010000 (LE: `00 00 01 00`)
- Hex:
  - `AA 55 06 02 02  00 00 01 00  <CRCL> <CRCH>`

### 5.3 Device ACK for SEQ = 0x02 CMD = 0x02 (Device SEQ is an arbitrary example: 0x10)
- PAYLOAD: ack_seq=02, ack_cmd=02
- Hex:
  - `AA 55 04 10 80  02 02  <CRCL> <CRCH>`

---

## 6) Simple Operational Rules
- The Host sends a request and waits for **ACK (0x80)**.
- When the operation completes, the device sends **DONE (0x81)**.
- On error, the device sends **ERROR (0x82)**.
- Retransmission policy is decided by the Host:
  - If ACK timeout (e.g., 100 ms) → retry 1..N times with the same SEQ (recommended).
- MOVE_TO_RAD is allowed **only after homing**. If not homed, return `ERROR: NOT_HOMED`.

---

## 7) Current Firmware Behavior Notes
- The home switch is used only for homing (not as a limit switch during normal moves).
- Homing sets the zero reference to the switch trigger position plus an offset (currently +0.2 rad).
- Homing search has a capped range of one output shaft revolution with the current gearing.
- Homing direction can be flipped by a single direction constant in the firmware.
- The controller continuously tracks the latest target whenever homed; there is no separate “moving” state.
- Auto-return: after 1 second without any command, the target is set to 0 rad (no ACK/DONE is emitted for auto-return).

---

## 8) Tunable Firmware Constants (arduino.ino)
- `HOME_DIR` = `1` (+1 / -1): homing direction sign.
- `HOME_PWM_FAST` = `60`, `HOME_PWM_SLOW` = `40`: PWM values used during homing search/backoff/approach/release.
- `BACKOFF_COUNTS` = `80`, `HOME_RELEASE_COUNTS` = `120`: encoder counts for backoff and release distances.
- `HOME_OFFSET_COUNTS` = `(0.15 rad)` converted to counts: position assigned when the switch first triggers.
- `HOME_SEARCH_MAX_COUNTS` = one output shaft revolution with gear ratio applied: max travel allowed during search before `ERR_HOME_NOT_FOUND`.
- `MAX_ABS_TARGET_RAD` = `±90 deg` (π/2): command clamp.
- `AUTO_RETURN_IDLE_MS` = `1000`: idle time before auto-return.
- `AUTO_RETURN_RAD` = `0.0`: auto-return target position.
