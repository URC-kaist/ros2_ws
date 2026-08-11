"""Validated tcpdump capture and streaming HTTP upload helpers."""

from __future__ import annotations

import http.client
import json
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit


INTERFACE_PATTERN = re.compile(r"^[A-Za-z0-9_.:@-]{1,64}$")


def load_stream_ports(config_path: Path) -> dict[str, int]:
    with config_path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    streams = payload.get("streams") if isinstance(payload, dict) else None
    if not isinstance(streams, list):
        raise ValueError("video config must contain a streams array")
    result: dict[str, int] = {}
    for item in streams:
        stream_id = item.get("stream_id") if isinstance(item, dict) else None
        port = item.get("udp_port") if isinstance(item, dict) else None
        if not isinstance(stream_id, str) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("video config contains an invalid stream")
        result[stream_id] = port
    return result


def build_capture_args(
    tcpdump_binary: str,
    interface: str,
    output_path: Path,
    ports: list[int],
) -> list[str]:
    if not INTERFACE_PATTERN.fullmatch(interface):
        raise ValueError("invalid capture interface")
    if not ports:
        raise ValueError("at least one RTP port is required")
    port_filter = " or ".join(f"dst port {port}" for port in sorted(set(ports)))
    return [
        tcpdump_binary,
        "-i",
        interface,
        "-n",
        "-U",
        "-s",
        "192",
        "-B",
        "4096",
        "--time-stamp-precision=nano",
        "-w",
        str(output_path),
        f"udp and ({port_filter})",
    ]


def stop_process(process: subprocess.Popen[bytes]) -> int:
    if process.poll() is not None:
        return int(process.returncode or 0)
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()


def run_capture(
    command: list[str],
    duration_s: float,
    cancel_event: threading.Event,
    popen=subprocess.Popen,
) -> tuple[int, str, int, int]:
    started_epoch_us = time.time_ns() // 1000
    process = popen(command, stderr=subprocess.PIPE)
    deadline = time.monotonic() + duration_s
    while process.poll() is None and time.monotonic() < deadline and not cancel_event.is_set():
        cancel_event.wait(0.1)
    exit_code = stop_process(process)
    stderr = ""
    if process.stderr is not None:
        stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
    return exit_code, stderr, started_epoch_us, time.time_ns() // 1000


def upload_bytes(url: str, token: str, content_type: str, body: bytes) -> None:
    parsed = urlsplit(url)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError("invalid upload URL")
    connection_type = (
        http.client.HTTPSConnection
        if parsed.scheme == "https"
        else http.client.HTTPConnection
    )
    connection = connection_type(parsed.hostname, parsed.port, timeout=20)
    path = parsed.path or "/"
    if parsed.query:
        path += f"?{parsed.query}"
    try:
        connection.request(
            "PUT",
            path,
            body=body,
            headers={
                "Authorization": f"Bearer {token}",
                "Content-Type": content_type,
                "Content-Length": str(len(body)),
            },
        )
        response = connection.getresponse()
        response.read()
        if response.status < 200 or response.status >= 300:
            raise RuntimeError(f"artifact upload returned HTTP {response.status}")
    finally:
        connection.close()


def upload_file(url: str, token: str, content_type: str, path: Path) -> None:
    parsed = urlsplit(url)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError("invalid upload URL")
    connection_type = (
        http.client.HTTPSConnection
        if parsed.scheme == "https"
        else http.client.HTTPConnection
    )
    connection = connection_type(parsed.hostname, parsed.port, timeout=30)
    request_path = parsed.path or "/"
    size = path.stat().st_size
    try:
        connection.putrequest("PUT", request_path)
        connection.putheader("Authorization", f"Bearer {token}")
        connection.putheader("Content-Type", content_type)
        connection.putheader("Content-Length", str(size))
        connection.endheaders()
        with path.open("rb") as handle:
            while chunk := handle.read(1024 * 1024):
                connection.send(chunk)
        response = connection.getresponse()
        response.read()
        if response.status < 200 or response.status >= 300:
            raise RuntimeError(f"artifact upload returned HTTP {response.status}")
    finally:
        connection.close()
