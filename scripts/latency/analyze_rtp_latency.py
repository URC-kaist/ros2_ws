#!/usr/bin/env python3
"""Match rover/base RTP pcaps and calculate Rocket-link latency statistics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import struct
import sys
from collections import defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import BinaryIO, Iterable, Sequence

try:
    from .rtp_capture import (
        DEFAULT_VIDEO_CONFIG,
        VideoStream,
        load_video_streams,
        metadata_path_for,
    )
except ImportError:
    from rtp_capture import DEFAULT_VIDEO_CONFIG, VideoStream, load_video_streams, metadata_path_for


PCAP_MAGIC = {
    b"\xd4\xc3\xb2\xa1": ("<", "microsecond"),
    b"\xa1\xb2\xc3\xd4": (">", "microsecond"),
    b"\x4d\x3c\xb2\xa1": ("<", "nanosecond"),
    b"\xa1\xb2\x3c\x4d": (">", "nanosecond"),
}
SUPPORTED_LINK_TYPES = {1, 113, 276}
ETHERTYPE_IPV4 = 0x0800
VLAN_ETHERTYPES = {0x8100, 0x88A8, 0x9100}
KERNEL_DROP_PATTERN = re.compile(r"(\d+) packets dropped by kernel")


@dataclass(frozen=True)
class RtpPacket:
    capture_epoch_us: float
    udp_port: int
    ssrc: int
    sequence: int
    rtp_timestamp: int
    marker: bool
    payload_type: int
    udp_payload_bytes: int

    @property
    def key(self) -> tuple[int, int, int, int]:
        return (self.udp_port, self.ssrc, self.sequence, self.rtp_timestamp)


@dataclass(frozen=True)
class MatchedPacket:
    stream_id: str
    udp_port: int
    ssrc: int
    sequence: int
    rtp_timestamp: int
    marker: bool
    rover_capture_epoch_us: float
    base_capture_epoch_us: float
    link_latency_us: float
    udp_payload_bytes: int


def read_exact(handle: BinaryIO, size: int) -> bytes:
    data = handle.read(size)
    if len(data) != size:
        raise ValueError("truncated pcap file")
    return data


def parse_link_layer(packet: bytes, link_type: int) -> tuple[int, int] | None:
    if link_type == 1:
        if len(packet) < 14:
            return None
        ethertype = struct.unpack_from("!H", packet, 12)[0]
        offset = 14
        while ethertype in VLAN_ETHERTYPES:
            if len(packet) < offset + 4:
                return None
            ethertype = struct.unpack_from("!H", packet, offset + 2)[0]
            offset += 4
        return ethertype, offset
    if link_type == 113:
        if len(packet) < 16:
            return None
        return struct.unpack_from("!H", packet, 14)[0], 16
    if link_type == 276:
        if len(packet) < 20:
            return None
        return struct.unpack_from("!H", packet, 0)[0], 20
    return None


def parse_rtp_packet(
    packet: bytes,
    capture_epoch_us: float,
    link_type: int,
    ports: set[int],
    payload_type: int,
) -> RtpPacket | None:
    link = parse_link_layer(packet, link_type)
    if link is None:
        return None
    ethertype, ip_offset = link
    if ethertype != ETHERTYPE_IPV4 or len(packet) < ip_offset + 20:
        return None

    version_ihl = packet[ip_offset]
    if version_ihl >> 4 != 4:
        return None
    ip_header_length = (version_ihl & 0x0F) * 4
    if ip_header_length < 20 or len(packet) < ip_offset + ip_header_length + 8:
        return None
    if packet[ip_offset + 9] != 17:
        return None
    fragment = struct.unpack_from("!H", packet, ip_offset + 6)[0]
    if fragment & 0x1FFF:
        return None

    udp_offset = ip_offset + ip_header_length
    destination_port = struct.unpack_from("!H", packet, udp_offset + 2)[0]
    if destination_port not in ports:
        return None
    udp_length = struct.unpack_from("!H", packet, udp_offset + 4)[0]
    if udp_length < 20:
        return None

    rtp_offset = udp_offset + 8
    if len(packet) < rtp_offset + 12:
        return None
    first = packet[rtp_offset]
    second = packet[rtp_offset + 1]
    if first >> 6 != 2 or second & 0x7F != payload_type:
        return None
    csrc_count = first & 0x0F
    header_length = 12 + csrc_count * 4
    if len(packet) < rtp_offset + header_length:
        return None
    if first & 0x10:
        extension_offset = rtp_offset + header_length
        if len(packet) < extension_offset + 4:
            return None
        extension_words = struct.unpack_from("!H", packet, extension_offset + 2)[0]
        header_length += 4 + extension_words * 4
        if len(packet) < rtp_offset + header_length:
            return None

    sequence, rtp_timestamp, ssrc = struct.unpack_from("!HII", packet, rtp_offset + 2)
    return RtpPacket(
        capture_epoch_us=capture_epoch_us,
        udp_port=destination_port,
        ssrc=ssrc,
        sequence=sequence,
        rtp_timestamp=rtp_timestamp,
        marker=bool(second & 0x80),
        payload_type=second & 0x7F,
        udp_payload_bytes=udp_length - 8,
    )


def read_rtp_pcap(path: Path, ports: set[int], payload_type: int = 96) -> list[RtpPacket]:
    packets: list[RtpPacket] = []
    with path.open("rb") as handle:
        magic = read_exact(handle, 4)
        if magic not in PCAP_MAGIC:
            raise ValueError(f"{path} is not a supported classic pcap file")
        endian, precision = PCAP_MAGIC[magic]
        global_rest = read_exact(handle, 20)
        _, _, _, _, _, link_type = struct.unpack(f"{endian}HHiiii", global_rest)
        if link_type not in SUPPORTED_LINK_TYPES:
            raise ValueError(f"unsupported pcap link type {link_type} in {path}")

        packet_header = struct.Struct(f"{endian}IIII")
        while True:
            raw_header = handle.read(packet_header.size)
            if not raw_header:
                break
            if len(raw_header) != packet_header.size:
                raise ValueError(f"truncated packet header in {path}")
            seconds, fraction, included_length, _original_length = packet_header.unpack(raw_header)
            raw_packet = read_exact(handle, included_length)
            fractional_us = fraction / 1000.0 if precision == "nanosecond" else float(fraction)
            parsed = parse_rtp_packet(
                raw_packet,
                seconds * 1_000_000.0 + fractional_us,
                link_type,
                ports,
                payload_type,
            )
            if parsed is not None:
                packets.append(parsed)
    return packets


def percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def distribution(values: Sequence[float]) -> dict[str, float | None]:
    if not values:
        return {
            "count": 0,
            **{key: None for key in ("min", "mean", "p50", "p95", "p99", "max")},
        }
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values),
    }


def capture_bitrate_bps(packets: Sequence[RtpPacket]) -> float | None:
    if len(packets) < 2:
        return None
    first = min(packet.capture_epoch_us for packet in packets)
    last = max(packet.capture_epoch_us for packet in packets)
    duration_s = (last - first) / 1_000_000.0
    if duration_s <= 0:
        return None
    return sum(packet.udp_payload_bytes for packet in packets) * 8.0 / duration_s


def match_stream_packets(
    stream: VideoStream,
    rover_packets: Sequence[RtpPacket],
    base_packets: Sequence[RtpPacket],
    clock_offset_us: float,
) -> tuple[dict[str, object], list[MatchedPacket]]:
    rover_by_key: dict[tuple[int, int, int, int], list[RtpPacket]] = defaultdict(list)
    base_by_key: dict[tuple[int, int, int, int], list[RtpPacket]] = defaultdict(list)
    for packet in rover_packets:
        rover_by_key[packet.key].append(packet)
    for packet in base_packets:
        base_by_key[packet.key].append(packet)

    matched: list[MatchedPacket] = []
    missing_at_base = 0
    unmatched_at_base = 0
    for key in rover_by_key.keys() | base_by_key.keys():
        rover_group = sorted(rover_by_key.get(key, []), key=lambda packet: packet.capture_epoch_us)
        base_group = sorted(base_by_key.get(key, []), key=lambda packet: packet.capture_epoch_us)
        pair_count = min(len(rover_group), len(base_group))
        missing_at_base += max(0, len(rover_group) - pair_count)
        unmatched_at_base += max(0, len(base_group) - pair_count)
        for rover_packet, base_packet in zip(rover_group[:pair_count], base_group[:pair_count]):
            matched.append(
                MatchedPacket(
                    stream_id=stream.stream_id,
                    udp_port=stream.udp_port,
                    ssrc=rover_packet.ssrc,
                    sequence=rover_packet.sequence,
                    rtp_timestamp=rover_packet.rtp_timestamp,
                    marker=rover_packet.marker,
                    rover_capture_epoch_us=rover_packet.capture_epoch_us,
                    base_capture_epoch_us=base_packet.capture_epoch_us,
                    link_latency_us=(
                        base_packet.capture_epoch_us
                        - rover_packet.capture_epoch_us
                        - clock_offset_us
                    ),
                    udp_payload_bytes=rover_packet.udp_payload_bytes,
                )
            )

    matched.sort(key=lambda packet: packet.rover_capture_epoch_us)
    latencies = [packet.link_latency_us for packet in matched]
    delay_variation = [
        abs(current.link_latency_us - previous.link_latency_us)
        for previous, current in zip(matched, matched[1:])
    ]
    sent = len(rover_packets)
    received = len(base_packets)
    summary: dict[str, object] = {
        "stream_id": stream.stream_id,
        "udp_port": stream.udp_port,
        "ssrcs": sorted({packet.ssrc for packet in rover_packets + base_packets}),
        "rover_packets": sent,
        "base_packets": received,
        "matched_packets": len(matched),
        "missing_at_base_packets": missing_at_base,
        "unmatched_at_base_packets": unmatched_at_base,
        "rover_duplicate_key_packets": sum(
            max(0, len(group) - 1) for group in rover_by_key.values()
        ),
        "base_duplicate_key_packets": sum(
            max(0, len(group) - 1) for group in base_by_key.values()
        ),
        "negative_latency_packets": sum(value < 0 for value in latencies),
        "loss_percent": (missing_at_base / sent * 100.0) if sent else None,
        "rover_offered_bitrate_bps": capture_bitrate_bps(rover_packets),
        "base_delivered_bitrate_bps": capture_bitrate_bps(base_packets),
        "link_latency_us": distribution(latencies),
        "packet_delay_variation_us": distribution(delay_variation),
    }
    return summary, matched


def analyze_captures(
    rover_path: Path,
    base_path: Path,
    streams: Sequence[VideoStream],
    payload_type: int,
    clock_offset_us: float,
    clock_offset_assumed: bool,
) -> tuple[dict[str, object], list[MatchedPacket]]:
    ports = {stream.udp_port for stream in streams}
    rover_packets = read_rtp_pcap(rover_path, ports, payload_type)
    base_packets = read_rtp_pcap(base_path, ports, payload_type)
    stream_summaries: list[dict[str, object]] = []
    samples: list[MatchedPacket] = []
    warnings: list[str] = []
    capture_metadata: dict[str, object] = {}
    for role, path in (("rover", rover_path), ("base", base_path)):
        sidecar_path = metadata_path_for(path)
        if not sidecar_path.is_file():
            capture_metadata[role] = {"metadata_path": None, "kernel_dropped_packets": None}
            continue
        try:
            with sidecar_path.open("r", encoding="utf-8") as handle:
                sidecar = json.load(handle)
            stderr = sidecar.get("tcpdump_stderr", "") if isinstance(sidecar, dict) else ""
            match = KERNEL_DROP_PATTERN.search(stderr) if isinstance(stderr, str) else None
            dropped = int(match.group(1)) if match else None
            capture_metadata[role] = {
                "metadata_path": str(sidecar_path),
                "role": sidecar.get("role") if isinstance(sidecar, dict) else None,
                "hostname": sidecar.get("hostname") if isinstance(sidecar, dict) else None,
                "interface": sidecar.get("interface") if isinstance(sidecar, dict) else None,
                "kernel_dropped_packets": dropped,
            }
            if isinstance(sidecar, dict) and sidecar.get("role") != role:
                warnings.append(f"{role} capture metadata has role={sidecar.get('role')!r}")
            if dropped is not None and dropped > 0:
                warnings.append(
                    f"{role} capture dropped {dropped} packets in the host kernel; loss is unreliable"
                )
        except (OSError, json.JSONDecodeError) as error:
            capture_metadata[role] = {"metadata_path": str(sidecar_path), "error": str(error)}
            warnings.append(f"could not read {role} capture metadata: {error}")
    if clock_offset_assumed:
        warnings.append("clock_offset_us was not supplied; base-rover offset is assumed to be zero")

    for stream in streams:
        rover_stream = [packet for packet in rover_packets if packet.udp_port == stream.udp_port]
        base_stream = [packet for packet in base_packets if packet.udp_port == stream.udp_port]
        summary, matched = match_stream_packets(
            stream, rover_stream, base_stream, clock_offset_us
        )
        stream_summaries.append(summary)
        samples.extend(matched)
        if not matched:
            warnings.append(f"{stream.stream_id}: no matching RTP packets")
        if summary["negative_latency_packets"]:
            warnings.append(
                f"{stream.stream_id}: negative latency samples indicate incorrect clock offset"
            )

    report: dict[str, object] = {
        "schema_version": 1,
        "kind": "mr2_rtp_link_latency_report",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "rover_capture_path": str(rover_path),
        "base_capture_path": str(base_path),
        "payload_type": payload_type,
        "clock_offset_us": clock_offset_us,
        "clock_offset_definition": "base_clock_minus_rover_clock",
        "clock_offset_assumed": clock_offset_assumed,
        "captures": capture_metadata,
        "packet_match_key": ["udp_port", "ssrc", "sequence", "rtp_timestamp"],
        "percentile_method": "linear_interpolation_at_(n-1)*q",
        "aggregate_link_latency_us": distribution(
            [sample.link_latency_us for sample in samples]
        ),
        "warnings": warnings,
        "streams": stream_summaries,
    }
    return report, samples


def load_browser_samples(path: Path) -> list[dict[str, object]]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    values = payload.get("samples") if isinstance(payload, dict) else None
    if not isinstance(values, list):
        raise ValueError("browser samples file must contain a samples array")
    samples: list[dict[str, object]] = []
    required_numbers = (
        "ssrc",
        "rtp_timestamp",
        "marker_sequence",
        "browser_receive_epoch_us",
        "browser_render_epoch_us",
    )
    for index, value in enumerate(values):
        if not isinstance(value, dict) or not isinstance(value.get("stream_id"), str):
            raise ValueError(f"browser sample {index} is invalid")
        if any(not isinstance(value.get(field), (int, float)) for field in required_numbers):
            raise ValueError(f"browser sample {index} has invalid timestamps or RTP key")
        if value["browser_render_epoch_us"] < value["browser_receive_epoch_us"]:
            raise ValueError(f"browser sample {index} renders before it is received")
        samples.append(value)
    return samples


def build_automated_uplink_report(
    link_report: dict[str, object],
    matched_packets: Sequence[MatchedPacket],
    browser_samples: Sequence[dict[str, object]],
    browser_clock_offset_us: float,
    trial_id: str,
    feed_count: int,
) -> dict[str, object]:
    matched_by_key: dict[tuple[str, int, int, int], list[MatchedPacket]] = defaultdict(list)
    for packet in matched_packets:
        if packet.marker:
            matched_by_key[
                (packet.stream_id, packet.ssrc, packet.sequence, packet.rtp_timestamp)
            ].append(packet)
    for values in matched_by_key.values():
        values.sort(key=lambda packet: packet.base_capture_epoch_us)

    browser_by_key: dict[tuple[str, int, int, int], list[dict[str, object]]] = defaultdict(list)
    for sample in browser_samples:
        key = (
            str(sample["stream_id"]),
            int(sample["ssrc"]),
            int(sample["marker_sequence"]),
            int(sample["rtp_timestamp"]),
        )
        browser_by_key[key].append(sample)
    for values in browser_by_key.values():
        values.sort(key=lambda sample: float(sample["browser_receive_epoch_us"]))

    frames_by_stream: dict[str, list[dict[str, float]]] = defaultdict(list)
    duplicate_keys = 0
    for key in matched_by_key.keys() | browser_by_key.keys():
        packet_values = matched_by_key.get(key, [])
        browser_values = browser_by_key.get(key, [])
        pair_count = min(len(packet_values), len(browser_values))
        duplicate_keys += max(0, len(packet_values) - 1) + max(0, len(browser_values) - 1)
        for packet, browser in zip(packet_values[:pair_count], browser_values[:pair_count]):
            browser_receive = float(browser["browser_receive_epoch_us"])
            browser_render = float(browser["browser_render_epoch_us"])
            rover_in_base_clock = packet.rover_capture_epoch_us + float(
                link_report["clock_offset_us"]
            )
            receive_in_base_clock = browser_receive + browser_clock_offset_us
            render_in_base_clock = browser_render + browser_clock_offset_us
            frames_by_stream[packet.stream_id].append(
                {
                    "rocket_m2": packet.base_capture_epoch_us - rover_in_base_clock,
                    "base_to_browser": receive_in_base_clock - packet.base_capture_epoch_us,
                    "decode_render": browser_render - browser_receive,
                    "total": render_in_base_clock - rover_in_base_clock,
                }
            )

    stream_reports = []
    aggregate_frames: list[dict[str, float]] = []
    link_streams = {str(stream["stream_id"]): stream for stream in link_report["streams"]}
    warnings = list(link_report["warnings"])
    if duplicate_keys:
        warnings.append(f"paired {duplicate_keys} duplicate RTP/browser frame keys by time order")
    for stream_id, link_stream in link_streams.items():
        frames = frames_by_stream.get(stream_id, [])
        aggregate_frames.extend(frames)
        stream_warnings: list[str] = []
        if not frames:
            stream_warnings.append("no marker frames matched browser render samples")
        segments = {
            name: distribution([frame[name] for frame in frames])
            for name in ("rocket_m2", "base_to_browser", "decode_render", "total")
        }
        stream_reports.append(
            {
                "stream_id": stream_id,
                "udp_port": link_stream["udp_port"],
                "segments": segments,
                "matched_frames": len(frames),
                "browser_frames": sum(
                    1 for sample in browser_samples if sample["stream_id"] == stream_id
                ),
                "matched_packets": link_stream["matched_packets"],
                "packet_loss_percent": link_stream["loss_percent"],
                "rover_offered_bitrate_bps": link_stream["rover_offered_bitrate_bps"],
                "base_delivered_bitrate_bps": link_stream["base_delivered_bitrate_bps"],
                "warnings": stream_warnings,
            }
        )
        warnings.extend(f"{stream_id}: {warning}" for warning in stream_warnings)

    if not aggregate_frames:
        raise ValueError("no RTP marker frames could be correlated with browser render samples")
    aggregate = {
        name: distribution([frame[name] for frame in aggregate_frames])
        for name in ("rocket_m2", "base_to_browser", "decode_render", "total")
    }
    return {
        "schema_version": 2,
        "kind": "mr2_automated_uplink_latency_report",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "trial_id": trial_id,
        "feed_count": feed_count,
        "stream_ids": [stream["stream_id"] for stream in stream_reports],
        "clock": {
            "base_minus_rover_us": link_report["clock_offset_us"],
            "base_minus_browser_us": browser_clock_offset_us,
        },
        "aggregate": aggregate,
        "streams": stream_reports,
        "captures": link_report["captures"],
        "matched_frames": len(aggregate_frames),
        "browser_frames": len(browser_samples),
        "warnings": warnings,
    }


def prepare_output(path: Path, overwrite: bool) -> None:
    if not path.parent.is_dir():
        raise ValueError(f"output directory does not exist: {path.parent}")
    if path.exists() and not overwrite:
        raise FileExistsError(f"refusing to overwrite existing file: {path}")


def write_report(path: Path, report: dict[str, object], overwrite: bool) -> None:
    prepare_output(path, overwrite)
    mode = "w" if overwrite else "x"
    with path.open(mode, encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")


def write_samples(path: Path, samples: Iterable[MatchedPacket], overwrite: bool) -> None:
    prepare_output(path, overwrite)
    mode = "w" if overwrite else "x"
    with path.open(mode, encoding="utf-8", newline="") as handle:
        fieldnames = list(MatchedPacket.__dataclass_fields__)
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(asdict(sample))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rover", required=True)
    parser.add_argument("--base", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--samples-csv")
    parser.add_argument("--stream-id", action="append", default=[])
    parser.add_argument("--config", default=str(DEFAULT_VIDEO_CONFIG))
    parser.add_argument("--payload-type", type=int, default=96)
    parser.add_argument("--clock-offset-us", type=float)
    parser.add_argument("--browser-samples")
    parser.add_argument("--browser-clock-offset-us", type=float)
    parser.add_argument("--trial-id")
    parser.add_argument("--feed-count", type=int)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def print_summary(report: dict[str, object]) -> None:
    if report["schema_version"] == 2:
        print("Automated Uplink frame latency")
        for stream in report["streams"]:
            latency = stream["segments"]["total"]
            p50 = latency["p50"]
            p50_text = "--" if p50 is None else f"{p50 / 1000.0:.3f} ms"
            print(f"  {stream['stream_id']}: frames={stream['matched_frames']} total_p50={p50_text}")
        for warning in report["warnings"]:
            print(f"warning: {warning}", file=sys.stderr)
        return
    print("Rocket M2 RTP link latency")
    for stream in report["streams"]:
        latency = stream["link_latency_us"]
        p50 = latency["p50"]
        p95 = latency["p95"]
        p50_text = "--" if p50 is None else f"{p50 / 1000.0:.3f} ms"
        p95_text = "--" if p95 is None else f"{p95 / 1000.0:.3f} ms"
        loss = stream["loss_percent"]
        loss_text = "--" if loss is None else f"{loss:.3f}%"
        print(
            f"  {stream['stream_id']}: matched={stream['matched_packets']} "
            f"p50={p50_text} p95={p95_text} loss={loss_text}"
        )
    for warning in report["warnings"]:
        print(f"warning: {warning}", file=sys.stderr)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 0 <= args.payload_type <= 127:
        parser.error("payload type must be between 0 and 127")
    try:
        streams = load_video_streams(Path(args.config).resolve(), args.stream_id)
        clock_offset_assumed = args.clock_offset_us is None
        clock_offset_us = 0.0 if clock_offset_assumed else args.clock_offset_us
        report, samples = analyze_captures(
            Path(args.rover).resolve(),
            Path(args.base).resolve(),
            streams,
            args.payload_type,
            clock_offset_us,
            clock_offset_assumed,
        )
        automated_arguments = (
            args.browser_samples,
            args.browser_clock_offset_us,
            args.trial_id,
            args.feed_count,
        )
        if any(value is not None for value in automated_arguments):
            if any(value is None for value in automated_arguments):
                raise ValueError(
                    "browser samples, browser clock offset, trial ID, and feed count are required together"
                )
            if args.feed_count != len(streams):
                raise ValueError("feed count must match selected streams")
            report = build_automated_uplink_report(
                report,
                samples,
                load_browser_samples(Path(args.browser_samples).resolve()),
                args.browser_clock_offset_us,
                args.trial_id,
                args.feed_count,
            )
        write_report(Path(args.output).resolve(), report, args.overwrite)
        if args.samples_csv:
            write_samples(Path(args.samples_csv).resolve(), samples, args.overwrite)
        print_summary(report)
    except (FileExistsError, FileNotFoundError, json.JSONDecodeError, OSError, ValueError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
