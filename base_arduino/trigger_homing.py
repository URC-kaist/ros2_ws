#!/usr/bin/env python3
import argparse
import time
import serial


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(seq: int, cmd: int, payload: bytes) -> bytes:
    # LEN counts SEQ + CMD + PAYLOAD
    length = 2 + len(payload)
    crc_input = bytes([length, seq, cmd]) + payload
    crc = crc16_ccitt_false(crc_input)
    return bytes([0xAA, 0x55, length, seq, cmd]) + payload + bytes(
        [crc & 0xFF, (crc >> 8) & 0xFF]
    )


def q16_16(value: float) -> int:
    return int(round(value * 65536.0))


def read_frame(ser: serial.Serial, deadline: float):
    state = "WAIT_AA"
    buf = bytearray()
    length = 0
    need = 0
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        byte = b[0]
        if state == "WAIT_AA":
            if byte == 0xAA:
                state = "WAIT_55"
            continue
        if state == "WAIT_55":
            if byte == 0x55:
                state = "WAIT_LEN"
            else:
                state = "WAIT_AA"
            continue
        if state == "WAIT_LEN":
            length = byte
            if length < 2 or length > 255:
                state = "WAIT_AA"
                continue
            buf = bytearray([length])
            need = length + 2
            state = "WAIT_BODY"
            continue
        if state == "WAIT_BODY":
            buf.append(byte)
            need -= 1
            if need > 0:
                continue
            crc_calc = crc16_ccitt_false(bytes(buf[:-2]))
            crc_rx = buf[-2] | (buf[-1] << 8)
            if crc_calc != crc_rx:
                state = "WAIT_AA"
                continue
            seq = buf[1]
            cmd = buf[2]
            payload = bytes(buf[3:-2])
            return seq, cmd, payload
    return None


def decode_frame(seq: int, cmd: int, payload: bytes) -> str:
    if cmd == 0x80 and len(payload) == 2:
        return f"ACK seq={payload[0]:02x} cmd={payload[1]:02x}"
    if cmd == 0x81 and len(payload) == 2:
        return f"DONE seq={payload[0]:02x} cmd={payload[1]:02x}"
    if cmd == 0x82 and len(payload) == 4:
        detail = payload[2] | (payload[3] << 8)
        return f"ERROR seq={payload[0]:02x} code={payload[1]:02x} detail={detail}"
    return f"FRAME cmd={cmd:02x} seq={seq:02x} payload={payload.hex()}"

def wait_for_done(
    ser: serial.Serial,
    deadline: float,
    target_cmd: int,
    print_frames: bool,
) -> bool:
    while True:
        res = read_frame(ser, deadline)
        if res is None:
            if print_frames:
                print("no response")
            return False
        seq, cmd, payload = res
        if print_frames:
            print(decode_frame(seq, cmd, payload))
        if cmd == 0x81 and len(payload) == 2 and payload[1] == target_cmd:
            return True
        if time.time() >= deadline:
            return False


def periodic_send(
    ser: serial.Serial,
    frame: bytes,
    duration_s: float,
    period_s: float,
    read: bool,
    timeout_s: float,
) -> None:
    if duration_s <= 0 or period_s <= 0:
        return
    end = time.monotonic() + duration_s
    next_send = time.monotonic()
    read_timeout = min(max(timeout_s, 0.0), 0.05)
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_send:
            ser.write(frame)
            ser.flush()
            next_send = now + period_s
        if read:
            res = read_frame(ser, time.time() + read_timeout)
            if res is None:
                continue
            seq, cmd, payload = res
            print(decode_frame(seq, cmd, payload))


def main() -> int:
    ap = argparse.ArgumentParser(description="Send commands to base_arduino")
    ap.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seq", type=lambda x: int(x, 0), default=0x01)
    ap.add_argument("--read", action="store_true", help="Read and decode responses")
    ap.add_argument("--timeout", type=float, default=3.0, help="Read timeout seconds")
    ap.add_argument("--boot-wait", type=float, default=2.0, help="Seconds to wait after opening port")
    ap.add_argument("--raw", action="store_true", help="Dump any raw bytes received")
    ap.add_argument("--keep-secs", type=float, default=0.0, help="Seconds to keep sending after command")
    ap.add_argument("--keep-period", type=float, default=0.5, help="Seconds between keep sends")
    cmd_group = ap.add_mutually_exclusive_group()
    cmd_group.add_argument("--home", action="store_true", help="Send HOMING_START")
    cmd_group.add_argument("--move-rad", type=float, help="Send MOVE_TO_RAD with radians")
    cmd_group.add_argument(
        "--home-then-move",
        type=float,
        help="Send HOMING_START, wait for DONE, then MOVE_TO_RAD with radians",
    )
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        if args.boot_wait > 0:
            time.sleep(args.boot_wait)
        if args.home_then_move is not None:
            home_frame = build_frame(args.seq, 0x01, b"")
            ser.write(home_frame)
            ser.flush()
            deadline = time.time() + args.timeout
            wait_for_done(ser, deadline, 0x01, args.read)

            move_seq = (args.seq + 1) & 0xFF
            q = q16_16(args.home_then_move)
            payload = q.to_bytes(4, byteorder="little", signed=True)
            move_frame = build_frame(move_seq, 0x02, payload)
            ser.write(move_frame)
            ser.flush()
            deadline = time.time() + args.timeout
            if args.read:
                wait_for_done(ser, deadline, 0x02, True)
            elif args.raw:
                data = bytearray()
                while time.time() < deadline:
                    chunk = ser.read(64)
                    if chunk:
                        data.extend(chunk)
                if data:
                    print(data.hex())
                else:
                    print("no response")

            periodic_send(
                ser,
                move_frame,
                args.keep_secs,
                args.keep_period,
                args.read,
                args.timeout,
            )

            print(home_frame.hex())
            print(move_frame.hex())
            return 0

        if args.move_rad is not None:
            cmd = 0x02
            q = q16_16(args.move_rad)
            payload = q.to_bytes(4, byteorder="little", signed=True)
        else:
            cmd = 0x01
            payload = b""

        frame = build_frame(args.seq, cmd, payload)
        ser.write(frame)
        ser.flush()
        if args.read:
            deadline = time.time() + args.timeout
            while True:
                res = read_frame(ser, deadline)
                if res is None:
                    print("no response")
                    break
                seq, cmd, payload = res
                print(decode_frame(seq, cmd, payload))
                if time.time() >= deadline:
                    break
        elif args.raw:
            deadline = time.time() + args.timeout
            data = bytearray()
            while time.time() < deadline:
                chunk = ser.read(64)
                if chunk:
                    data.extend(chunk)
            if data:
                print(data.hex())
            else:
                print("no response")

        periodic_send(
            ser,
            frame,
            args.keep_secs,
            args.keep_period,
            args.read,
            args.timeout,
        )

        print(frame.hex())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
