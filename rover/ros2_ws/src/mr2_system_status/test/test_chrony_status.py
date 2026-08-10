import subprocess
import unittest

from mr2_system_status.chrony_status import (
    parse_chronyc_tracking,
    read_chrony_status,
)


TRACKING = """Reference ID    : C0A8010A (192.168.1.10)
Stratum         : 9
System time     : 0.000250000 seconds fast of NTP time
Last offset     : 0.000020000 seconds
RMS offset      : 0.000030000 seconds
Root delay      : 0.000500000 seconds
Root dispersion : 0.000300000 seconds
Update interval : 4.0 seconds
Leap status     : Normal
"""


class ChronyStatusTest(unittest.TestCase):
    def test_parse_chronyc_tracking(self) -> None:
        status = parse_chronyc_tracking(TRACKING, 1234)
        self.assertIs(status["available"], True)
        self.assertIs(status["synchronized"], True)
        self.assertEqual(status["role"], "rover")
        self.assertEqual(status["reference_id"], "C0A8010A")
        self.assertEqual(status["reference_name"], "192.168.1.10")
        self.assertEqual(status["stratum"], 9)
        self.assertEqual(status["system_time_offset_s"], 0.00025)
        self.assertEqual(status["root_dispersion_s"], 0.0003)
        self.assertEqual(status["sampled_at_epoch_ms"], 1234)

    def test_slow_and_unsynchronized_signs(self) -> None:
        status = parse_chronyc_tracking(
            TRACKING.replace("0.000250000 seconds fast", "0.000125000 seconds slow")
            .replace("Stratum         : 9", "Stratum         : 0")
            .replace("Leap status     : Normal", "Leap status     : Not synchronised")
        )
        self.assertEqual(status["system_time_offset_s"], -0.000125)
        self.assertIs(status["synchronized"], False)

    def test_read_uses_allow_listed_command(self) -> None:
        calls = []

        def runner(command, **kwargs):
            calls.append((command, kwargs))
            return subprocess.CompletedProcess(command, 0, TRACKING, "")

        status = read_chrony_status(runner=runner)
        self.assertIs(status["available"], True)
        self.assertEqual(calls[0][0], ["chronyc", "-n", "tracking"])
        self.assertEqual(calls[0][1]["timeout"], 1.5)

    def test_missing_chronyc_is_structured(self) -> None:
        def runner(*_args, **_kwargs):
            raise FileNotFoundError

        status = read_chrony_status(runner=runner)
        self.assertIs(status["available"], False)
        self.assertEqual(status["error"], "chronyc is not installed")


if __name__ == "__main__":
    unittest.main()
