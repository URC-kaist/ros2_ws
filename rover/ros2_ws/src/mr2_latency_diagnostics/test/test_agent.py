import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from mr2_latency_diagnostics.agent_node import UplinkAgent


class UplinkAgentTest(unittest.TestCase):
    def test_prepare_falls_back_when_lease_service_is_unavailable(self) -> None:
        logger = Mock()
        lease_client = Mock()
        lease_client.wait_for_service.return_value = False
        agent = SimpleNamespace(
            _cancel_event=threading.Event(),
            _lease_client=lease_client,
            _lease_service_name="/video_streaming/acquire_stream_lease",
            _lock=threading.RLock(),
            _max_duration_s=60.0,
            _publish_status=Mock(),
            _stream_ports={"front": 5000},
            _trial=None,
            get_logger=lambda: logger,
        )
        request = SimpleNamespace(trial_id="trial-1", stream_ids=["front"])
        response = SimpleNamespace(accepted=False, message="")

        UplinkAgent._prepare(agent, request, response)

        self.assertTrue(response.accepted)
        self.assertFalse(agent._trial.lease_acquired)
        self.assertEqual(agent._trial.stream_ids, ["front"])
        self.assertIn("selected-port capture", response.message)
        logger.warning.assert_called_once()
        agent._publish_status.assert_called_once()


if __name__ == "__main__":
    unittest.main()
