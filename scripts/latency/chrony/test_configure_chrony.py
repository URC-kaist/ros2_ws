from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path

from scripts.latency.chrony.configure_chrony import (
    install_config,
    render_base_config,
    render_config,
    render_rover_config,
    write_candidate,
)


class ConfigureChronyTest(unittest.TestCase):
    def test_base_is_local_reference_and_only_allows_rover(self) -> None:
        config = render_base_config("192.168.1.10", "192.168.1.20")
        self.assertIn("local stratum 8", config)
        self.assertIn("allow 192.168.1.20/32", config)
        directives = [line for line in config.splitlines() if not line.startswith("#")]
        self.assertFalse(any(line.startswith("server ") for line in directives))
        self.assertFalse(any(line.startswith("pool ") for line in directives))

    def test_rover_uses_only_base(self) -> None:
        config = render_rover_config("192.168.1.10")
        self.assertIn(
            "server 192.168.1.10 iburst prefer trust minpoll 2 maxpoll 4", config
        )
        self.assertIn("makestep 0.1 10", config)
        self.assertNotIn("local stratum", config)
        self.assertEqual(config.count("server "), 1)

    def test_role_and_address_validation(self) -> None:
        with self.assertRaisesRegex(ValueError, "rover-ip"):
            render_config("base", "192.168.1.10", None)
        with self.assertRaisesRegex(ValueError, "only valid"):
            render_config("rover", "192.168.1.10", "192.168.1.20")
        with self.assertRaisesRegex(ValueError, "non-loopback"):
            render_rover_config("127.0.0.1")
        with self.assertRaisesRegex(ValueError, "different"):
            render_base_config("192.168.1.10", "192.168.1.10")

    def test_candidate_refuses_overwrite_without_force(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "chrony.conf"
            write_candidate(path, "first\n", False)
            with self.assertRaises(FileExistsError):
                write_candidate(path, "second\n", False)
            write_candidate(path, "second\n", True)
            self.assertEqual(path.read_text(encoding="utf-8"), "second\n")

    def test_install_validates_backs_up_and_restarts(self) -> None:
        calls: list[list[str]] = []

        def runner(command, **_kwargs):
            calls.append(command)
            return subprocess.CompletedProcess(command, 0, "", "")

        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "chrony.conf"
            target.write_text("old\n", encoding="utf-8")
            backup = install_config(
                "new\n", target=target, runner=runner, require_root=False
            )
            self.assertIsNotNone(backup)
            self.assertEqual(backup.read_text(encoding="utf-8"), "old\n")
            self.assertEqual(target.read_text(encoding="utf-8"), "new\n")
            self.assertEqual(calls[0][0:3], ["chronyd", "-p", "-f"])
            self.assertEqual(calls[1], ["systemctl", "restart", "chrony.service"])

    def test_restart_failure_restores_previous_config(self) -> None:
        calls = 0

        def runner(command, **_kwargs):
            nonlocal calls
            calls += 1
            if command[0] == "systemctl" and calls == 2:
                raise subprocess.CalledProcessError(1, command)
            return subprocess.CompletedProcess(command, 0, "", "")

        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "chrony.conf"
            target.write_text("old\n", encoding="utf-8")
            with self.assertRaises(subprocess.CalledProcessError):
                install_config(
                    "new\n", target=target, runner=runner, require_root=False
                )
            self.assertEqual(target.read_text(encoding="utf-8"), "old\n")


if __name__ == "__main__":
    unittest.main()
