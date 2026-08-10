"""Read-only chronyc tracking parser for rover diagnostics."""

from __future__ import annotations

import re
import subprocess
import time
from typing import Any, Callable, Dict


RunCommand = Callable[..., subprocess.CompletedProcess[str]]


def _number(fields: Dict[str, str], name: str) -> float | None:
    raw = fields.get(name)
    if raw is None:
        return None
    try:
        return float(raw.split()[0])
    except (ValueError, IndexError):
        return None


def parse_chronyc_tracking(
    output: str, sampled_at_epoch_ms: int | None = None
) -> Dict[str, Any]:
    fields: Dict[str, str] = {}
    for line in output.splitlines():
        if ":" not in line:
            continue
        name, value = line.split(":", 1)
        fields[name.strip()] = value.strip()

    leap_status = fields.get("Leap status", "")
    stratum = _number(fields, "Stratum")
    reference_raw = fields.get("Reference ID", "")
    reference_match = re.fullmatch(r"([^\s]+)(?:\s+\((.+)\))?", reference_raw)
    system_match = re.fullmatch(
        r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s+"
        r"seconds\s+(fast|slow)\s+of\s+NTP\s+time",
        fields.get("System time", ""),
        flags=re.IGNORECASE,
    )
    if (
        not leap_status
        or stratum is None
        or reference_match is None
        or system_match is None
    ):
        raise ValueError("chronyc tracking output is missing required fields")

    magnitude = abs(float(system_match.group(1)))
    system_time_offset_s = (
        magnitude if system_match.group(2).lower() == "fast" else -magnitude
    )
    synchronized = leap_status.lower() != "not synchronised" and stratum > 0
    return {
        "schema_version": 1,
        "role": "rover",
        "available": True,
        "synchronized": synchronized,
        "reference_id": reference_match.group(1),
        "reference_name": reference_match.group(2),
        "stratum": int(stratum),
        "system_time_offset_s": system_time_offset_s,
        "last_offset_s": _number(fields, "Last offset"),
        "rms_offset_s": _number(fields, "RMS offset"),
        "root_delay_s": _number(fields, "Root delay"),
        "root_dispersion_s": _number(fields, "Root dispersion"),
        "update_interval_s": _number(fields, "Update interval"),
        "leap_status": leap_status,
        "sampled_at_epoch_ms": sampled_at_epoch_ms
        if sampled_at_epoch_ms is not None
        else time.time_ns() // 1_000_000,
        "error": None,
    }


def unavailable_status(
    error: str, sampled_at_epoch_ms: int | None = None
) -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "role": "rover",
        "available": False,
        "synchronized": False,
        "reference_id": None,
        "reference_name": None,
        "stratum": None,
        "system_time_offset_s": None,
        "last_offset_s": None,
        "rms_offset_s": None,
        "root_delay_s": None,
        "root_dispersion_s": None,
        "update_interval_s": None,
        "leap_status": None,
        "sampled_at_epoch_ms": sampled_at_epoch_ms
        if sampled_at_epoch_ms is not None
        else time.time_ns() // 1_000_000,
        "error": error,
    }


def read_chrony_status(
    runner: RunCommand = subprocess.run, timeout_s: float = 1.5
) -> Dict[str, Any]:
    sampled_at_epoch_ms = time.time_ns() // 1_000_000
    try:
        result = runner(
            ["chronyc", "-n", "tracking"],
            check=True,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        return parse_chronyc_tracking(result.stdout, sampled_at_epoch_ms)
    except FileNotFoundError:
        return unavailable_status("chronyc is not installed", sampled_at_epoch_ms)
    except (subprocess.SubprocessError, ValueError):
        return unavailable_status("chronyc tracking failed", sampled_at_epoch_ms)
