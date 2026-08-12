import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock
from unittest.mock import patch

from mr2_latency_diagnostics.capture import (
    build_capture_args,
    load_stream_ports,
    probe_upload_target,
    resolve_capture_interface,
)
from mr2_latency_diagnostics.trial_state import (
    UplinkTrial,
    validate_duration,
    validate_trial_id,
)


class UplinkTrialStateTest(unittest.TestCase):
    def test_trial_records_stream_lease_state(self) -> None:
        no_lease = UplinkTrial("trial-no-lease", ["front"])
        self.assertFalse(no_lease.lease_acquired)
        self.assertTrue(
            UplinkTrial(
                "trial-with-lease", ["front"], lease_acquired=True
            ).lease_acquired
        )

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

    def test_capture_interface_uses_kernel_route_when_not_configured(self) -> None:
        result = type("Result", (), {"stdout": '[{"dev":"eth0"}]'})()
        run = Mock(return_value=result)
        self.assertEqual(resolve_capture_interface("", "192.168.1.101", run), "eth0")
        run.assert_called_once_with(
            ["ip", "-j", "route", "get", "192.168.1.101"],
            check=True,
            capture_output=True,
            text=True,
            timeout=3,
        )

    def test_explicit_capture_interface_does_not_query_route(self) -> None:
        run = Mock()
        self.assertEqual(resolve_capture_interface("enp1s0", "192.168.1.101", run), "enp1s0")
        run.assert_not_called()

    def test_upload_probe_verifies_the_base_trial(self) -> None:
        response = Mock(status=200)
        response.read.return_value = b'{"trial_id":"trial-1"}'
        connection = Mock()
        connection.getresponse.return_value = response
        with patch(
            "mr2_latency_diagnostics.capture.http.client.HTTPConnection",
            return_value=connection,
        ):
            probe_upload_target(
                "http://192.168.1.101/latency/uplink/trials/trial-1",
                "trial-1",
            )
        connection.request.assert_called_once_with(
            "GET",
            "/latency/uplink/trials/trial-1",
            headers={"Accept": "application/json"},
        )
        connection.close.assert_called_once()

    def test_upload_probe_rejects_the_wrong_trial(self) -> None:
        response = Mock(status=200)
        response.read.return_value = b'{"trial_id":"another-trial"}'
        connection = Mock()
        connection.getresponse.return_value = response
        with patch(
            "mr2_latency_diagnostics.capture.http.client.HTTPConnection",
            return_value=connection,
        ):
            with self.assertRaisesRegex(RuntimeError, "wrong trial"):
                probe_upload_target(
                    "http://192.168.1.101/latency/uplink/trials/trial-1",
                    "trial-1",
                )


if __name__ == "__main__":
    unittest.main()
