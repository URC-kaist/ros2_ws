#!/usr/bin/env python3
"""
Hardware-level health check for the MR2 rover.

This script intentionally does not use ROS. It checks:
  - two u-blox GPS USB devices
  - one XBee USB serial adapter
  - five V4L2 camera symlinks
  - one RealSense device
  - eight NoFW CAN actuators
  - two battery monitor CAN status streams
  - optional module-specific hardware

It always runs scripts/can0.bash before checking CAN telemetry.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import stat
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
CAMERAS = ("videoTOP", "videoLEFT", "videoRIGHT", "videoFR", "videoFL")
AUTONOMOUS_CAMERAS = ("videoFRONT",)
MANIPULATOR_CAMERAS = ("videoGRIPPER", "videoARM")
REALSENSE_VIDEO = "videoRGBD"
XBEE_DEVICE = "ttyXBEE"
CAN_IFACE = "can0"
CAN_DURATION_SEC = 10.0
BATTERY_DURATION_SEC = 12.0
AK_DURATION_SEC = 5.0
NOFW_NODES = (1, 2, 3, 4, 5, 6, 7, 8)
AK_MOTOR_IDS = (101, 102, 103, 104, 105, 106)
UBLOX_VENDOR = "1546"
UBLOX_PRODUCT = "01a9"
XBEE_VENDOR = "0403"
XBEE_PRODUCT = "6015"
XBEE_SERIAL = "D30JZD4O"
REALSENSE_VENDOR = "8086"
REALSENSE_PRODUCT = "0b3a"


@dataclass
class CheckResult:
    name: str
    status: str
    detail: str


STATUS_COLORS = {
    "OK": "\033[32m",
    "WARN": "\033[33m",
    "FAIL": "\033[31m",
}
RESET_COLOR = "\033[0m"


@dataclass
class FrameStats:
    count: int = 0
    first: float | None = None
    last: float | None = None
    intervals_ms: list[float] = field(default_factory=list)
    payload: list[int] = field(default_factory=list)

    def add(self, timestamp: float, payload: list[int]) -> None:
        self.count += 1
        self.payload = payload
        if self.first is None:
            self.first = timestamp
        if self.last is not None:
            self.intervals_ms.append((timestamp - self.last) * 1000.0)
        self.last = timestamp

    @property
    def mean_ms(self) -> float | None:
        if not self.intervals_ms:
            return None
        return sum(self.intervals_ms) / len(self.intervals_ms)


def run(
    argv: list[str],
    *,
    check: bool = False,
    timeout: float | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        argv,
        check=check,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
    )


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(errors="replace").strip()
    except OSError:
        return None


def status_join(parts: list[str]) -> str:
    return "; ".join(part for part in parts if part)


def usb_devices(vendor: str, product: str) -> list[Path]:
    devices: list[Path] = []
    root = Path("/sys/bus/usb/devices")
    if not root.exists():
        return devices
    for child in root.iterdir():
        if not child.is_dir():
            continue
        if read_text(child / "idVendor") == vendor and read_text(child / "idProduct") == product:
            devices.append(child)
    return sorted(devices)


def usb_device_for_tty(dev_path: Path) -> Path | None:
    real = Path(os.path.realpath(dev_path))
    sysfs = Path("/sys/class/tty") / real.name
    if not sysfs.exists():
        return None
    device = (sysfs / "device").resolve()
    for parent in (device, *device.parents):
        if (parent / "idVendor").exists() and (parent / "idProduct").exists():
            return parent
    return None


def tty_nodes_for_usb_device(usb_device: Path) -> list[str]:
    nodes: list[str] = []
    for child in usb_device.rglob("tty*"):
        if child.name.startswith("tty") and (child / "dev").exists():
            dev_path = Path("/dev") / child.name
            if dev_path.exists():
                nodes.append(str(dev_path))
    return sorted(set(nodes))


def usb_summary(device: Path) -> str:
    serial = read_text(device / "serial")
    product = read_text(device / "product")
    manufacturer = read_text(device / "manufacturer")
    ttys = tty_nodes_for_usb_device(device)
    parts = [
        device.name,
        f"serial={serial}" if serial else "",
        f"product={product}" if product else "",
        f"manufacturer={manufacturer}" if manufacturer else "",
        f"tty={','.join(ttys)}" if ttys else "",
    ]
    return " ".join(part for part in parts if part)


def video_sysfs_dir(dev_path: Path) -> Path | None:
    real = Path(os.path.realpath(dev_path))
    if not real.name.startswith("video"):
        return None
    sysfs = Path("/sys/class/video4linux") / real.name
    return sysfs if sysfs.exists() else None


def usb_attrs_for_video(dev_path: Path) -> dict[str, str]:
    sysfs = video_sysfs_dir(dev_path)
    if sysfs is None:
        return {}
    device = (sysfs / "device").resolve()
    attrs: dict[str, str] = {}
    for parent in (device, *device.parents):
        vendor = read_text(parent / "idVendor")
        product = read_text(parent / "idProduct")
        if vendor and product:
            attrs["idVendor"] = vendor
            attrs["idProduct"] = product
            for key in ("serial", "product", "manufacturer"):
                value = read_text(parent / key)
                if value:
                    attrs[key] = value
            break
    return attrs


def check_v4l2_query(device: Path) -> tuple[bool | None, str]:
    if shutil.which("v4l2-ctl") is None:
        return None, "v4l2-ctl not installed"
    proc = run(["v4l2-ctl", "-d", str(device), "-D"], timeout=3.0)
    if proc.returncode == 0:
        first_line = next((line.strip() for line in proc.stdout.splitlines() if line.strip()), "")
        return True, first_line
    lines = (proc.stderr or proc.stdout).strip().splitlines()
    return False, lines[-1] if lines else f"exit code {proc.returncode}"


def check_video_device(label: str, device: Path) -> CheckResult:
    if not device.exists():
        return CheckResult(label, "FAIL", f"{device} missing")

    try:
        mode = device.stat().st_mode
    except OSError as exc:
        return CheckResult(label, "FAIL", f"{device} stat failed: {exc}")

    if not stat.S_ISCHR(mode):
        return CheckResult(label, "FAIL", f"{device} exists but is not a character device")

    real = Path(os.path.realpath(device))
    sysfs = video_sysfs_dir(device)
    name = read_text(sysfs / "name") if sysfs else None
    attrs = usb_attrs_for_video(device)
    ok, query_detail = check_v4l2_query(device)

    status = "OK"
    parts = [f"{device}->{real}", f"name={name}" if name else ""]
    if attrs:
        parts.append(f"usb={attrs.get('idVendor')}:{attrs.get('idProduct')}")
        if attrs.get("serial"):
            parts.append(f"serial={attrs['serial']}")
    if ok is False:
        status = "FAIL"
        parts.append(f"v4l2 query failed: {query_detail}")
    elif ok is True:
        parts.append(query_detail)
    else:
        parts.append(query_detail)

    return CheckResult(label, status, status_join(parts))


def check_gps() -> list[CheckResult]:
    devices = usb_devices(UBLOX_VENDOR, UBLOX_PRODUCT)
    if len(devices) < 2:
        detail = f"found {len(devices)} u-blox {UBLOX_VENDOR}:{UBLOX_PRODUCT} USB device(s)"
        if devices:
            detail += ": " + " | ".join(usb_summary(device) for device in devices)
        return [CheckResult("GPS x2", "FAIL", detail)]

    results = [
        CheckResult(
            "GPS x2",
            "OK",
            f"found {len(devices)} u-blox {UBLOX_VENDOR}:{UBLOX_PRODUCT} USB device(s)",
        )
    ]
    for index, device in enumerate(devices[:2], start=1):
        results.append(CheckResult(f"GPS {index}", "OK", usb_summary(device)))
    if len(devices) > 2:
        results.append(CheckResult("GPS extra", "WARN", f"{len(devices) - 2} extra u-blox device(s) present"))
    return results


def check_xbee(device_name: str) -> list[CheckResult]:
    dev_path = Path("/dev") / device_name
    if not dev_path.exists():
        return [CheckResult("XBee", "FAIL", f"{dev_path} missing")]

    try:
        mode = dev_path.stat().st_mode
    except OSError as exc:
        return [CheckResult("XBee", "FAIL", f"{dev_path} stat failed: {exc}")]

    if not stat.S_ISCHR(mode):
        return [CheckResult("XBee", "FAIL", f"{dev_path} exists but is not a character device")]

    real = Path(os.path.realpath(dev_path))
    usb_device = usb_device_for_tty(dev_path)
    if usb_device is None:
        return [CheckResult("XBee", "WARN", f"{dev_path}->{real}; USB parent not found")]

    vendor = read_text(usb_device / "idVendor")
    product = read_text(usb_device / "idProduct")
    serial = read_text(usb_device / "serial")
    parts = [f"{dev_path}->{real}", usb_summary(usb_device)]
    status = "OK"
    issues: list[str] = []
    if vendor != XBEE_VENDOR or product != XBEE_PRODUCT:
        status = "FAIL"
        issues.append(f"usb={vendor}:{product} expected {XBEE_VENDOR}:{XBEE_PRODUCT}")
    if serial and serial != XBEE_SERIAL:
        status = "WARN" if status == "OK" else status
        issues.append(f"serial={serial} expected {XBEE_SERIAL}")
    if issues:
        parts.append(", ".join(issues))
    return [CheckResult("XBee", status, status_join(parts))]


def check_realsense(device_name: str) -> list[CheckResult]:
    results = [check_video_device("RealSense V4L2", Path("/dev") / device_name)]
    devices = usb_devices(REALSENSE_VENDOR, REALSENSE_PRODUCT)
    if devices:
        details = " | ".join(usb_summary(device) for device in devices)
        results.append(CheckResult("RealSense USB", "OK", details))
    else:
        results.append(
            CheckResult(
                "RealSense USB",
                "FAIL",
                f"no USB device with id {REALSENSE_VENDOR}:{REALSENSE_PRODUCT}",
            )
        )

    if shutil.which("rs-enumerate-devices") is not None:
        proc = run(["rs-enumerate-devices", "-s"], timeout=5.0)
        if proc.returncode == 0:
            line = next(
                (
                    line.strip()
                    for line in proc.stdout.splitlines()
                    if line.strip() and " ERROR " not in line
                ),
                "detected by rs-enumerate-devices",
            )
            results.append(CheckResult("RealSense SDK", "OK", line))
        else:
            detail = (proc.stderr or proc.stdout).strip().splitlines()
            results.append(CheckResult("RealSense SDK", "WARN", detail[-1] if detail else "query failed"))
    return results


def parse_can_capture(lines: list[str]) -> dict[tuple[str, int], FrameStats]:
    stats: dict[tuple[str, int], FrameStats] = {}
    pattern = re.compile(
        r"\(([0-9.]+)\)\s+\S+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+((?:[0-9A-Fa-f]{2}\s*)+)"
    )
    for line in lines:
        match = pattern.search(line)
        if not match:
            continue
        timestamp = float(match.group(1))
        can_id = int(match.group(2), 16)
        dlc = int(match.group(3))
        payload = [int(byte, 16) for byte in re.findall(r"[0-9A-Fa-f]{2}", match.group(4))]
        if len(payload) != dlc:
            continue

        family: str | None = None
        node: int | None = None
        if 0x5F0 <= can_id <= 0x5FF and dlc == 8:
            family = "diag"
            node = can_id - 0x5F0
        elif 0x400 <= can_id <= 0x4FF:
            base = can_id & 0x7F0
            node = can_id & 0x00F
            if base == 0x400 and dlc == 4:
                family = "angle"
            elif base == 0x410 and dlc == 4:
                family = "velocity"
            elif base == 0x420 and dlc == 8:
                family = "limits"
            elif base == 0x430 and dlc == 8:
                family = "config"

        if family and node and 1 <= node <= 15:
            stats.setdefault((family, node), FrameStats()).add(timestamp, payload)
    return stats


def profile_name(value: int) -> str:
    return ("VelocityOnly", "As5600", "TmagLut", "DirectInput")[value] if 0 <= value <= 3 else str(value)


def mode_name(value: int) -> str:
    if value == 1:
        return "angle"
    if value == 2:
        return "velocity"
    return str(value)


def decode_diag(payload: list[int]) -> dict[str, int | str]:
    flags = payload[4]
    return {
        "magic": payload[0],
        "stored": profile_name(payload[1]),
        "active": profile_name(payload[2]),
        "mode": mode_name(payload[3]),
        "vel": 1 if flags & 0x01 else 0,
        "angle": 1 if flags & 0x02 else 0,
        "foc_cal": 1 if flags & 0x04 else 0,
        "out_cal": 1 if flags & 0x08 else 0,
        "cal_load": (flags >> 4) & 0x03,
        "fault": payload[5],
        "need_cal": 1 if payload[6] & 0x01 else 0,
        "profile_result": (payload[6] >> 4) & 0x0F,
        "feedback_req": 1 if payload[7] & 0x01 else 0,
        "armed": 1 if payload[7] & 0x02 else 0,
    }


def diag_summary(decoded: dict[str, int | str]) -> str:
    return (
        f"magic=0x{decoded['magic']:02X} stored={decoded['stored']} active={decoded['active']} "
        f"mode={decoded['mode']} vel={decoded['vel']} angle={decoded['angle']} "
        f"foc_cal={decoded['foc_cal']} out_cal={decoded['out_cal']} cal_load={decoded['cal_load']} "
        f"fault={decoded['fault']} need_cal={decoded['need_cal']} "
        f"profile_result={decoded['profile_result']} feedback_req={decoded['feedback_req']} "
        f"armed={decoded['armed']}"
    )


def format_period(stats: FrameStats) -> str:
    mean = stats.mean_ms
    if mean is None:
        return f"count={stats.count} mean=-"
    return f"count={stats.count} mean={mean:.1f}ms min={min(stats.intervals_ms):.1f}ms max={max(stats.intervals_ms):.1f}ms"


def expected_count(duration: float, period_ms: float, fraction: float = 0.65) -> int:
    return max(1, int((duration * 1000.0 / period_ms) * fraction))


def validate_period(
    stats: FrameStats | None,
    duration: float,
    period_ms: float,
    tolerance: float,
) -> tuple[str, str]:
    minimum = expected_count(duration, period_ms)
    if stats is None or stats.count == 0:
        return "FAIL", "missing"
    status = "OK"
    details = [format_period(stats)]
    if stats.count < minimum:
        status = "FAIL"
        details.append(f"expected at least {minimum}")
    mean = stats.mean_ms
    if mean is not None and abs(mean - period_ms) > tolerance:
        status = "WARN" if status == "OK" else status
        details.append(f"nominal {period_ms:.0f}ms")
    return status, ", ".join(details)


def merge_status(first: str, second: str) -> str:
    order = {"OK": 0, "WARN": 1, "FAIL": 2}
    return first if order[first] >= order[second] else second


def check_diag_payload(node: int, stats: FrameStats | None) -> tuple[str, str]:
    if stats is None or not stats.payload:
        return "FAIL", "no diagnostic payload"
    decoded = decode_diag(stats.payload)
    status = "OK"
    issues: list[str] = []

    expected_profile = "As5600" if node <= 4 else "VelocityOnly"
    expected_mode = "angle" if node <= 4 else "velocity"
    expected_angle = 1 if node <= 4 else 0
    expected_feedback = 1 if node <= 4 else 0
    required = {
        "magic": 0xFB,
        "stored": expected_profile,
        "active": expected_profile,
        "mode": expected_mode,
        "vel": 1,
        "angle": expected_angle,
        "foc_cal": 1,
        "out_cal": 1,
        "cal_load": 1,
        "fault": 0,
        "need_cal": 0,
        "profile_result": 0,
        "feedback_req": expected_feedback,
    }
    for key, expected in required.items():
        if decoded[key] != expected:
            status = "FAIL"
            issues.append(f"{key}={decoded[key]} expected {expected}")
    if decoded["armed"] != 0:
        status = merge_status(status, "WARN")
        issues.append(f"armed={decoded['armed']}")

    detail = diag_summary(decoded)
    if issues:
        detail += " (" + ", ".join(issues) + ")"
    return status, detail


def run_can0_setup(iface: str) -> CheckResult:
    script = REPO_ROOT / "scripts" / "can0.bash"
    if not script.exists():
        return CheckResult("CAN setup", "FAIL", f"{script} missing")
    proc = subprocess.run(
        [str(script), iface],
        cwd=str(REPO_ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    detail_lines = (proc.stdout + proc.stderr).strip().splitlines()
    detail = detail_lines[-1] if detail_lines else f"ran {script}"
    return CheckResult("CAN setup", "OK" if proc.returncode == 0 else "FAIL", detail)


def capture_can(iface: str, duration: float) -> tuple[int, list[str], str]:
    if shutil.which("candump") is None:
        return 127, [], "candump not found. Install can-utils."

    argv = ["candump", "-ta", f"{iface},400:700", f"{iface},5F0:7F0"]
    proc = subprocess.Popen(
        argv,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=duration)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            stdout, stderr = proc.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, stderr = proc.communicate(timeout=1.0)
    return proc.returncode if proc.returncode is not None else 0, stdout.splitlines(), stderr.strip()


def check_can(iface: str, duration: float, expected_nodes: tuple[int, ...]) -> list[CheckResult]:
    ip = shutil.which("ip")
    if ip is None:
        return [CheckResult("CAN interface", "FAIL", "ip command not found")]
    proc = run([ip, "link", "show", iface])
    if proc.returncode != 0:
        return [CheckResult("CAN interface", "FAIL", f"{iface} not found")]

    results = [CheckResult("CAN interface", "OK", proc.stdout.strip().splitlines()[0])]
    rc, lines, stderr = capture_can(iface, duration)
    if rc not in (0, -15, -9):
        results.append(CheckResult("CAN capture", "FAIL", stderr or f"candump exit code {rc}"))
        return results
    if not lines:
        results.append(CheckResult("CAN capture", "FAIL", f"no NoFW frames captured on {iface} for {duration:g}s"))
        return results

    stats = parse_can_capture(lines)
    results.append(CheckResult("CAN capture", "OK", f"captured {len(lines)} NoFW frame(s) in {duration:g}s"))

    for node in expected_nodes:
        node_status = "OK"
        details: list[str] = []

        diag_stats = stats.get(("diag", node))
        period_status, period_detail = validate_period(diag_stats, duration, 500.0, 75.0)
        payload_status, payload_detail = check_diag_payload(node, diag_stats)
        node_status = merge_status(node_status, period_status)
        node_status = merge_status(node_status, payload_status)
        details.append(f"diag {period_detail}")
        details.append(payload_detail)

        for family in ("angle", "velocity"):
            family_status, family_detail = validate_period(stats.get((family, node)), duration, 50.0, 15.0)
            node_status = merge_status(node_status, family_status)
            details.append(f"{family} {family_detail}")

        config_status, config_detail = validate_period(stats.get(("config", node)), duration, 500.0, 75.0)
        node_status = merge_status(node_status, config_status)
        details.append(f"config {config_detail}")

        if node <= 4:
            limits_status, limits_detail = validate_period(stats.get(("limits", node)), duration, 500.0, 75.0)
            node_status = merge_status(node_status, limits_status)
            details.append(f"limits {limits_detail}")

        results.append(CheckResult(f"NoFW node {node}", node_status, status_join(details)))

    return results


def parse_battery_status_output(output: str) -> list[CheckResult]:
    results: list[CheckResult] = []
    current_name: str | None = None
    current_lines: list[str] = []

    def flush() -> None:
        if current_name is None:
            return

        status = "OK"
        details: list[str] = []
        found_periods: set[str] = set()
        for line in current_lines:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith(("Summary:", "Metadata:", "Cells:")):
                key, value = stripped.split(":", 1)
                found_periods.add(key)
                value = value.strip()
                if value == "no frames":
                    status = "FAIL"
                details.append(f"{key.lower()} {value}")
            elif stripped.startswith(("Pack:", "Meta:", "Cell voltages:")):
                details.append(stripped)

        missing_periods = [key for key in ("Summary", "Metadata", "Cells") if key not in found_periods]
        if missing_periods:
            status = "FAIL"
            details.append(f"missing {','.join(missing_periods).lower()}")

        results.append(CheckResult(current_name, status, status_join(details)))

    for line in output.splitlines():
        stripped = line.strip()
        if stripped in ("battery_1", "battery_2"):
            flush()
            current_name = stripped
            current_lines = []
        elif current_name is not None:
            current_lines.append(line)
    flush()

    return results


def check_battery_status(iface: str, duration: float) -> list[CheckResult]:
    script = REPO_ROOT / "scripts" / "check_battery_status.bash"
    if not script.exists():
        return [CheckResult("Battery status", "FAIL", f"{script} missing")]

    try:
        proc = subprocess.run(
            [str(script), "--iface", iface, "--duration", f"{duration:g}"],
            cwd=str(REPO_ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=duration + 5.0,
        )
    except subprocess.TimeoutExpired:
        return [CheckResult("Battery status", "FAIL", f"timed out after {duration + 5.0:g}s")]
    if proc.returncode != 0:
        detail_lines = (proc.stderr or proc.stdout).strip().splitlines()
        detail = detail_lines[-1] if detail_lines else f"exit code {proc.returncode}"
        return [CheckResult("Battery status", "FAIL", detail)]

    results = parse_battery_status_output(proc.stdout)
    if not results:
        return [CheckResult("Battery status", "FAIL", "no battery status output parsed")]
    return results


def parse_ak_actuator_status_output(output: str, expected_motors: tuple[int, ...]) -> list[CheckResult]:
    results: list[CheckResult] = []
    seen_motors: set[int] = set()
    last_result: CheckResult | None = None

    for line in output.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        match = re.match(r"^(\d+)\s+(\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(.+)$", stripped)
        if match:
            motor = int(match.group(1))
            if motor not in expected_motors:
                continue
            seen_motors.add(motor)
            frames = int(match.group(2))
            position = match.group(3)
            velocity = match.group(4)
            current = match.group(5)
            temp = match.group(6)
            error = match.group(7)

            status = "OK"
            issues: list[str] = []
            if frames == 0 or error == "NO_FRAMES":
                status = "FAIL"
                issues.append("no frames")
            elif not error.startswith("0 "):
                status = "FAIL"
                issues.append(f"error={error}")

            detail = (
                f"frames={frames} position={position} velocity={velocity} "
                f"current={current} temp={temp} error={error}"
            )
            if issues:
                detail += " (" + ", ".join(issues) + ")"
            last_result = CheckResult(f"AK motor {motor}", status, detail)
            results.append(last_result)
        elif last_result is not None and stripped.startswith("count="):
            last_result.detail = status_join([last_result.detail, stripped])

    for motor in expected_motors:
        if motor not in seen_motors:
            results.append(CheckResult(f"AK motor {motor}", "FAIL", "missing from status output"))

    return results


def check_ak_actuators(iface: str, duration: float, motor_ids: tuple[int, ...]) -> list[CheckResult]:
    script = REPO_ROOT / "scripts" / "check_ak_actuator_status.bash"
    if not script.exists():
        return [CheckResult("AK actuators", "FAIL", f"{script} missing")]

    argv = [str(script), "--iface", iface, "--duration", f"{duration:g}"]
    for motor in motor_ids:
        argv.extend(["--motor", str(motor)])

    try:
        proc = subprocess.run(
            argv,
            cwd=str(REPO_ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=duration + 5.0,
        )
    except subprocess.TimeoutExpired:
        return [CheckResult("AK actuators", "FAIL", f"timed out after {duration + 5.0:g}s")]
    if proc.returncode != 0:
        detail_lines = (proc.stderr or proc.stdout).strip().splitlines()
        detail = detail_lines[-1] if detail_lines else f"exit code {proc.returncode}"
        return [CheckResult("AK actuators", "FAIL", detail)]

    results = parse_ak_actuator_status_output(proc.stdout, motor_ids)
    if not results:
        return [CheckResult("AK actuators", "FAIL", "no AK status output parsed")]
    return results


def print_results(title: str, results: list[CheckResult]) -> None:
    print(f"\n{title}")
    print("-" * len(title))
    for result in results:
        print(f"{color_status(result.status)} {result.name:<18} {result.detail}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run hardware-level health checks for the MR2 rover.",
    )
    parser.add_argument(
        "--science-module",
        action="store_true",
        help="run science module configuration-specific checks",
    )
    parser.add_argument(
        "--manipulator-module",
        action="store_true",
        help="run manipulator module configuration-specific checks",
    )
    parser.add_argument(
        "--autonomous-module",
        action="store_true",
        help="run autonomous module configuration-specific checks",
    )
    return parser.parse_args(argv)


def check_autonomous_module() -> list[CheckResult]:
    return [
        check_video_device(camera_name, Path("/dev") / camera_name)
        for camera_name in AUTONOMOUS_CAMERAS
    ]


def check_manipulator_module(can_ready: bool) -> list[CheckResult]:
    results = [
        check_video_device(camera_name, Path("/dev") / camera_name)
        for camera_name in MANIPULATOR_CAMERAS
    ]
    if can_ready:
        results.extend(check_ak_actuators(CAN_IFACE, AK_DURATION_SEC, AK_MOTOR_IDS))
    else:
        results.append(CheckResult("AK actuators", "FAIL", "skipped because CAN setup failed"))
    return results


def color_enabled() -> bool:
    return sys.stdout.isatty() and os.environ.get("NO_COLOR") is None


def color_status(status: str) -> str:
    marker = f"[{status:4}]"
    if not color_enabled():
        return marker
    color = STATUS_COLORS.get(status)
    return f"{color}{marker}{RESET_COLOR}" if color else marker


def color_summary_label(label: str) -> str:
    if not color_enabled():
        return label
    color = STATUS_COLORS.get(label)
    return f"{color}{label}{RESET_COLOR}" if color else label


def main(argv: list[str] | None = None) -> int:
    raw_args = sys.argv[1:] if argv is None else argv
    args = parse_args(raw_args)

    if os.geteuid() != 0:
        print("Re-running with sudo for CAN setup...")
        os.execvp(
            "sudo",
            ["sudo", sys.executable, str(Path(__file__).resolve()), *raw_args],
        )

    all_results: list[CheckResult] = []
    can_setup_result = run_can0_setup(CAN_IFACE)

    gps_results = check_gps()
    all_results.extend(gps_results)
    print_results("GPS", gps_results)

    xbee_results = check_xbee(XBEE_DEVICE)
    all_results.extend(xbee_results)
    print_results("XBee", xbee_results)

    camera_results = [
        check_video_device(camera_name, Path("/dev") / camera_name)
        for camera_name in CAMERAS
    ]
    all_results.extend(camera_results)
    print_results("Cameras", camera_results)

    realsense_results = check_realsense(REALSENSE_VIDEO)
    all_results.extend(realsense_results)
    print_results("RealSense", realsense_results)

    can_results: list[CheckResult] = [can_setup_result]
    if can_results[-1].status != "FAIL":
        can_results.extend(check_can(CAN_IFACE, CAN_DURATION_SEC, NOFW_NODES))
    all_results.extend(can_results)
    print_results("CAN / NoFW", can_results)

    battery_results: list[CheckResult]
    if can_setup_result.status == "FAIL":
        battery_results = [CheckResult("Battery status", "FAIL", "skipped because CAN setup failed")]
    else:
        battery_results = check_battery_status(CAN_IFACE, BATTERY_DURATION_SEC)
    all_results.extend(battery_results)
    print_results("Battery", battery_results)

    if args.autonomous_module:
        autonomous_results = check_autonomous_module()
        all_results.extend(autonomous_results)
        print_results("Autonomous Module", autonomous_results)

    if args.manipulator_module:
        manipulator_results = check_manipulator_module(can_setup_result.status != "FAIL")
        all_results.extend(manipulator_results)
        print_results("Manipulator Module", manipulator_results)

    fail_count = sum(1 for result in all_results if result.status == "FAIL")
    warn_count = sum(1 for result in all_results if result.status == "WARN")
    ok_count = sum(1 for result in all_results if result.status == "OK")

    print("\nSummary")
    print("-------")
    print(
        f"{color_summary_label('OK')}={ok_count} "
        f"{color_summary_label('WARN')}={warn_count} "
        f"{color_summary_label('FAIL')}={fail_count}"
    )
    return 1 if fail_count else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
        raise SystemExit(130)
