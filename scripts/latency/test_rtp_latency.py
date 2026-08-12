from __future__ import annotations

import json
import struct
import tempfile
import unittest
from pathlib import Path

from scripts.latency.analyze_rtp_latency import (
    analyze_captures,
    build_automated_uplink_report,
    main as analyze_main,
    read_rtp_pcap,
    write_report,
)
from scripts.latency.rtp_capture import (
    VideoStream,
    build_capture_filter,
    build_tcpdump_args,
    load_video_streams,
    metadata_path_for,
)


def make_rtp_packet(
    destination_port: int,
    sequence: int,
    rtp_timestamp: int,
    ssrc: int = 0x10203040,
    payload_type: int = 96,
    vlan: bool = False,
) -> bytes:
    rtp = struct.pack("!BBHII", 0x80, 0x80 | payload_type, sequence, rtp_timestamp, ssrc)
    rtp += b"\x65" + bytes([sequence & 0xFF]) * 20
    udp_length = 8 + len(rtp)
    udp = struct.pack("!HHHH", 40000, destination_port, udp_length, 0) + rtp
    total_length = 20 + len(udp)
    ipv4 = struct.pack(
        "!BBHHHBBHII",
        0x45,
        0,
        total_length,
        sequence,
        0,
        64,
        17,
        0,
        0x0A000001,
        0x0A000002,
    ) + udp
    if vlan:
        ethernet = b"\xaa" * 6 + b"\xbb" * 6 + struct.pack("!H", 0x8100)
        ethernet += struct.pack("!HH", 1, 0x0800)
    else:
        ethernet = b"\xaa" * 6 + b"\xbb" * 6 + struct.pack("!H", 0x0800)
    return ethernet + ipv4


def write_pcap(
    path: Path,
    packets: list[tuple[float, bytes]],
    *,
    nanosecond: bool = True,
) -> None:
    magic = b"\x4d\x3c\xb2\xa1" if nanosecond else b"\xd4\xc3\xb2\xa1"
    with path.open("wb") as handle:
        handle.write(magic)
        handle.write(struct.pack("<HHiiii", 2, 4, 0, 0, 65535, 1))
        for epoch_us, packet in packets:
            seconds = int(epoch_us // 1_000_000)
            remaining_us = epoch_us - seconds * 1_000_000
            fraction = round(remaining_us * (1000 if nanosecond else 1))
            handle.write(struct.pack("<IIII", seconds, fraction, len(packet), len(packet)))
            handle.write(packet)


class CaptureConfigurationTest(unittest.TestCase):
    def test_loads_selected_streams_and_builds_safe_tcpdump_args(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "streams.json"
            config.write_text(
                json.dumps(
                    {
                        "streams": [
                            {"stream_id": "front", "udp_port": 5000},
                            {"stream_id": "rear", "udp_port": 5002},
                        ]
                    }
                ),
                encoding="utf-8",
            )
            streams = load_video_streams(config, ["rear", "front", "rear"])
            self.assertEqual([stream.stream_id for stream in streams], ["rear", "front"])
            self.assertEqual(
                build_capture_filter(streams),
                "udp and (dst port 5000 or dst port 5002)",
            )
            command = build_tcpdump_args("tcpdump", "eth0", root / "capture.pcap", streams)
            self.assertEqual(command[0:3], ["tcpdump", "-i", "eth0"])
            self.assertIn("--time-stamp-precision=nano", command)
            self.assertEqual(command[-1], "udp and (dst port 5000 or dst port 5002)")

    def test_rejects_unknown_stream_and_interface(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "streams.json"
            config.write_text(
                json.dumps({"streams": [{"stream_id": "front", "udp_port": 5000}]}),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "unknown stream_id"):
                load_video_streams(config, ["missing"])
            with self.assertRaisesRegex(ValueError, "invalid interface"):
                build_tcpdump_args(
                    "tcpdump",
                    "eth0;bad",
                    root / "capture.pcap",
                    [VideoStream("front", 5000)],
                )


class RtpPcapAnalysisTest(unittest.TestCase):
    def test_builds_same_frame_uplink_segments_and_direct_total_percentiles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rover = root / "rover.pcap"
            base = root / "base.pcap"
            first = make_rtp_packet(5000, 10, 1000, ssrc=42)
            second = make_rtp_packet(5000, 11, 2000, ssrc=42)
            write_pcap(rover, [(1_000_000, first), (2_000_000, second)])
            write_pcap(base, [(1_001_000, first), (2_009_000, second)])
            link_report, packets = analyze_captures(
                rover,
                base,
                [VideoStream("front", 5000)],
                payload_type=96,
                clock_offset_us=0,
                clock_offset_assumed=False,
            )
            report = build_automated_uplink_report(
                link_report,
                packets,
                [
                    {
                        "stream_id": "front",
                        "ssrc": 42,
                        "marker_sequence": 10,
                        "rtp_timestamp": 1000,
                        "browser_receive_epoch_us": 1_009_000,
                        "browser_render_epoch_us": 1_010_000,
                    },
                    {
                        "stream_id": "front",
                        "ssrc": 42,
                        "marker_sequence": 11,
                        "rtp_timestamp": 2000,
                        "browser_receive_epoch_us": 2_010_000,
                        "browser_render_epoch_us": 2_011_000,
                    },
                ],
                browser_clock_offset_us=0,
                trial_id="trial-1",
                feed_count=1,
            )
            aggregate = report["aggregate"]
            self.assertEqual(report["matched_frames"], 2)
            self.assertEqual(aggregate["rocket_m2"]["p50"], 5000)
            self.assertEqual(aggregate["base_to_browser"]["p50"], 4500)
            self.assertEqual(aggregate["decode_render"]["p50"], 1000)
            self.assertEqual(aggregate["total"]["p50"], 10500)
            self.assertNotEqual(
                aggregate["total"]["p95"],
                aggregate["rocket_m2"]["p95"]
                + aggregate["base_to_browser"]["p95"]
                + aggregate["decode_render"]["p95"],
            )

    def test_matches_fixed_delay_with_clock_offset_and_sequence_wrap(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rover = root / "rover.pcap"
            base = root / "base.pcap"
            sequences = [65534, 65535, 0, 1]
            rover_packets: list[tuple[float, bytes]] = []
            base_packets: list[tuple[float, bytes]] = []
            for index, sequence in enumerate(sequences):
                rover_time = 100_000_000 + index * 10_000
                packet = make_rtp_packet(5000, sequence, 90000 + index * 3000)
                rover_packets.append((rover_time, packet))
                base_packets.append((rover_time + 250 + 5_000, packet))
            write_pcap(rover, rover_packets, nanosecond=True)
            write_pcap(base, base_packets, nanosecond=True)

            report, samples = analyze_captures(
                rover,
                base,
                [VideoStream("front", 5000)],
                payload_type=96,
                clock_offset_us=250,
                clock_offset_assumed=False,
            )
            stream = report["streams"][0]
            self.assertEqual(stream["matched_packets"], 4)
            self.assertEqual(stream["missing_at_base_packets"], 0)
            self.assertEqual(stream["link_latency_us"]["p50"], 5000)
            self.assertEqual(stream["link_latency_us"]["p95"], 5000)
            self.assertEqual(stream["link_latency_us"]["max"], 5000)
            self.assertEqual(report["aggregate_link_latency_us"]["p95"], 5000)
            self.assertEqual([sample.sequence for sample in samples], sequences)

    def test_reports_loss_duplicates_ports_and_negative_latency(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rover = root / "rover.pcap"
            base = root / "base.pcap"
            front_10 = make_rtp_packet(5000, 10, 1000, vlan=True)
            front_11 = make_rtp_packet(5000, 11, 2000)
            rear_20 = make_rtp_packet(5002, 20, 3000)
            ignored = make_rtp_packet(5999, 30, 4000)
            base_only = make_rtp_packet(5000, 99, 9900)
            write_pcap(
                rover,
                [
                    (10_000_000, front_10),
                    (10_010_000, front_11),
                    (10_020_000, rear_20),
                    (10_030_000, ignored),
                ],
                nanosecond=False,
            )
            write_pcap(
                base,
                [
                    (10_004_000, front_10),
                    (10_004_100, front_10),
                    (10_019_000, rear_20),
                    (10_040_000, base_only),
                ],
                nanosecond=False,
            )
            report, _samples = analyze_captures(
                rover,
                base,
                [VideoStream("front", 5000), VideoStream("rear", 5002)],
                payload_type=96,
                clock_offset_us=0,
                clock_offset_assumed=True,
            )
            front, rear = report["streams"]
            self.assertEqual(front["rover_packets"], 2)
            self.assertEqual(front["base_packets"], 3)
            self.assertEqual(front["matched_packets"], 1)
            self.assertEqual(front["missing_at_base_packets"], 1)
            self.assertEqual(front["unmatched_at_base_packets"], 2)
            self.assertEqual(front["base_duplicate_key_packets"], 1)
            self.assertEqual(front["loss_percent"], 50)
            self.assertEqual(rear["negative_latency_packets"], 1)
            self.assertTrue(any("assumed" in warning for warning in report["warnings"]))
            self.assertTrue(any("negative latency" in warning for warning in report["warnings"]))

    def test_surfaces_tcpdump_kernel_drops_from_sidecar(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rover = root / "rover.pcap"
            base = root / "base.pcap"
            packet = make_rtp_packet(5000, 1, 100)
            write_pcap(rover, [(1_000_000, packet)])
            write_pcap(base, [(1_001_000, packet)])
            metadata_path_for(rover).write_text(
                json.dumps(
                    {
                        "role": "rover",
                        "hostname": "rover",
                        "interface": "eth0",
                        "stream_lease_acquired": False,
                        "tcpdump_stderr": "1 packet captured\n3 packets dropped by kernel",
                    }
                ),
                encoding="utf-8",
            )
            report, _samples = analyze_captures(
                rover,
                base,
                [VideoStream("front", 5000)],
                payload_type=96,
                clock_offset_us=0,
                clock_offset_assumed=False,
            )
            self.assertEqual(report["captures"]["rover"]["kernel_dropped_packets"], 3)
            self.assertFalse(
                report["captures"]["rover"]["stream_lease_acquired"]
            )
            self.assertTrue(any("loss is unreliable" in warning for warning in report["warnings"]))
            self.assertTrue(
                any("background link load" in warning for warning in report["warnings"])
            )

    def test_ignores_truncated_and_wrong_payload_type_packets(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.pcap"
            write_pcap(
                path,
                [
                    (1_000_000.125, b"\x00" * 10),
                    (1_001_000.250, make_rtp_packet(5000, 1, 100, payload_type=97)),
                    (1_002_000.375, make_rtp_packet(5000, 2, 200)),
                ],
                nanosecond=True,
            )
            packets = read_rtp_pcap(path, {5000}, payload_type=96)
            self.assertEqual(len(packets), 1)
            self.assertEqual(packets[0].sequence, 2)
            self.assertAlmostEqual(packets[0].capture_epoch_us, 1_002_000.375, places=3)

    def test_refuses_report_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "report.json"
            output.write_text("existing", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                write_report(output, {"schema_version": 1}, overwrite=False)

    def test_analyzer_cli_writes_json_and_packet_csv(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rover = root / "rover.pcap"
            base = root / "base.pcap"
            report_path = root / "report.json"
            samples_path = root / "samples.csv"
            config = root / "streams.json"
            config.write_text(
                json.dumps({"streams": [{"stream_id": "front", "udp_port": 5000}]}),
                encoding="utf-8",
            )
            packet = make_rtp_packet(5000, 7, 700)
            write_pcap(rover, [(5_000_000, packet)])
            write_pcap(base, [(5_003_000, packet)])
            exit_code = analyze_main(
                [
                    "--rover",
                    str(rover),
                    "--base",
                    str(base),
                    "--config",
                    str(config),
                    "--stream-id",
                    "front",
                    "--clock-offset-us",
                    "0",
                    "--output",
                    str(report_path),
                    "--samples-csv",
                    str(samples_path),
                ]
            )
            self.assertEqual(exit_code, 0)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["streams"][0]["link_latency_us"]["p50"], 3000)
            self.assertIn("link_latency_us", samples_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
