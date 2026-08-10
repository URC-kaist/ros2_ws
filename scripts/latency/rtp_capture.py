#!/usr/bin/env python3
"""Capture MR2 video RTP headers without modifying the video stream."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VIDEO_CONFIG = (
    REPOSITORY_ROOT
    / "rover/ros2_ws/src/mr2_launch/config/video_streams.json"
)
INTERFACE_PATTERN = re.compile(r"^[A-Za-z0-9_.:@-]{1,64}$")


@dataclass(frozen=True)
class VideoStream:
    stream_id: str
    udp_port: int


def load_video_streams(
    config_path: Path, selected_stream_ids: Sequence[str] | None = None
) -> list[VideoStream]:
    with config_path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)
    raw_streams = raw.get("streams") if isinstance(raw, dict) else None
    if not isinstance(raw_streams, list):
        raise ValueError(f"{config_path} must contain a streams array")

    streams: list[VideoStream] = []
    seen_ids: set[str] = set()
    seen_ports: set[int] = set()
    for index, item in enumerate(raw_streams):
        if not isinstance(item, dict):
            raise ValueError(f"streams[{index}] must be an object")
        stream_id = item.get("stream_id")
        udp_port = item.get("udp_port")
        if not isinstance(stream_id, str) or not stream_id:
            raise ValueError(f"streams[{index}].stream_id must be a non-empty string")
        if not isinstance(udp_port, int) or isinstance(udp_port, bool) or not 1 <= udp_port <= 65535:
            raise ValueError(f"streams[{index}].udp_port must be a valid UDP port")
        if stream_id in seen_ids:
            raise ValueError(f"duplicate stream_id: {stream_id}")
        if udp_port in seen_ports:
            raise ValueError(f"duplicate udp_port: {udp_port}")
        seen_ids.add(stream_id)
        seen_ports.add(udp_port)
        streams.append(VideoStream(stream_id=stream_id, udp_port=udp_port))

    requested = list(selected_stream_ids or [])
    if not requested:
        return streams
    by_id = {stream.stream_id: stream for stream in streams}
    missing = [stream_id for stream_id in requested if stream_id not in by_id]
    if missing:
        raise ValueError(f"unknown stream_id(s): {', '.join(missing)}")
    return [by_id[stream_id] for stream_id in dict.fromkeys(requested)]


def build_capture_filter(streams: Iterable[VideoStream]) -> str:
    ports = sorted({stream.udp_port for stream in streams})
    if not ports:
        raise ValueError("at least one video stream is required")
    port_filter = " or ".join(f"dst port {port}" for port in ports)
    return f"udp and ({port_filter})"


def build_tcpdump_args(
    tcpdump_binary: str,
    interface: str,
    output_path: Path,
    streams: Sequence[VideoStream],
) -> list[str]:
    if not INTERFACE_PATTERN.fullmatch(interface):
        raise ValueError(f"invalid interface name: {interface!r}")
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
        build_capture_filter(streams),
    ]


def metadata_path_for(output_path: Path) -> Path:
    return output_path.with_suffix(f"{output_path.suffix}.metadata.json")


def prepare_output(path: Path, overwrite: bool) -> None:
    if not path.parent.is_dir():
        raise ValueError(f"output directory does not exist: {path.parent}")
    if path.exists():
        if not overwrite:
            raise FileExistsError(f"refusing to overwrite existing file: {path}")
        path.unlink()


def stop_capture(process: subprocess.Popen[bytes]) -> int:
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


def run_capture(args: argparse.Namespace) -> int:
    config_path = Path(args.config).resolve()
    output_path = Path(args.output).resolve()
    sidecar_path = metadata_path_for(output_path)
    streams = load_video_streams(config_path, args.stream_id)
    prepare_output(output_path, args.overwrite)
    prepare_output(sidecar_path, args.overwrite)
    command = build_tcpdump_args(args.tcpdump, args.interface, output_path, streams)

    started_epoch_us = time.time_ns() // 1000
    metadata = {
        "schema_version": 1,
        "kind": "mr2_rtp_capture_metadata",
        "role": args.role,
        "hostname": socket.gethostname(),
        "interface": args.interface,
        "config_path": str(config_path),
        "output_path": str(output_path),
        "streams": [
            {"stream_id": stream.stream_id, "udp_port": stream.udp_port}
            for stream in streams
        ],
        "requested_duration_s": args.duration_s,
        "started_epoch_us": started_epoch_us,
        "finished_epoch_us": None,
        "tcpdump_exit_code": None,
        "tcpdump_command": command,
        "tcpdump_stderr": None,
    }

    print(f"Capturing {args.role} RTP on {args.interface} -> {output_path}")
    print(f"Streams: {', '.join(stream.stream_id for stream in streams)}")
    process: subprocess.Popen[bytes] | None = None
    exit_code = 1
    try:
        process = subprocess.Popen(command, stderr=subprocess.PIPE)
        if args.duration_s is None:
            exit_code = process.wait()
        else:
            try:
                exit_code = process.wait(timeout=args.duration_s)
            except subprocess.TimeoutExpired:
                exit_code = stop_capture(process)
    except KeyboardInterrupt:
        if process is not None:
            exit_code = stop_capture(process)
    finally:
        metadata["finished_epoch_us"] = time.time_ns() // 1000
        metadata["tcpdump_exit_code"] = exit_code
        if process is not None and process.stderr is not None:
            tcpdump_stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
            metadata["tcpdump_stderr"] = tcpdump_stderr
            if tcpdump_stderr:
                print(tcpdump_stderr, file=sys.stderr)
        with sidecar_path.open("x", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2)
            handle.write("\n")

    if exit_code != 0:
        print(f"tcpdump exited with code {exit_code}", file=sys.stderr)
    else:
        print(f"Capture metadata: {sidecar_path}")
    return exit_code


def positive_duration(value: str) -> float:
    duration = float(value)
    if not duration > 0:
        raise argparse.ArgumentTypeError("duration must be greater than zero")
    return duration


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("rover", "base"), required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--stream-id", action="append", default=[])
    parser.add_argument("--duration-s", type=positive_duration)
    parser.add_argument("--config", default=str(DEFAULT_VIDEO_CONFIG))
    parser.add_argument("--tcpdump", default="tcpdump")
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run_capture(args)
    except (FileExistsError, FileNotFoundError, json.JSONDecodeError, OSError, ValueError) as error:
        parser.error(str(error))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
