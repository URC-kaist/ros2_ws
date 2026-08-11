import json
import tempfile
import unittest
from pathlib import Path

from mr2_latency_diagnostics.capture import build_capture_args, load_stream_ports
from mr2_latency_diagnostics.trial_state import (
    UplinkTrial,
    validate_duration,
    validate_trial_id,
)


class UplinkTrialStateTest(unittest.TestCase):
    def test_terminal_state_ignores_late_transition(self) -> None:
        trial = UplinkTrial("trial-1", ["front"])
        trial.transition("capturing", "Capturing", 0.2)
        trial.cancel()
        trial.transition("completed", "Late completion", 1.0)
        self.assertEqual(trial.phase, "cancelled")

    def test_validation(self) -> None:
        self.assertEqual(validate_trial_id("trial-1"), "trial-1")
        with self.assertRaises(ValueError):
            validate_trial_id("../bad")
        self.assertEqual(validate_duration(15, 60), 15)
        with self.assertRaises(ValueError):
            validate_duration(61, 60)

    def test_capture_filter_contains_only_selected_ports(self) -> None:
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
            ports = load_stream_ports(config)
            command = build_capture_args(
                "tcpdump", "eth0", root / "capture.pcap", [ports["rear"]]
            )
            self.assertEqual(command[-1], "udp and (dst port 5002)")
            self.assertNotIn("5000", command[-1])


if __name__ == "__main__":
    unittest.main()
