"""ROS node coordinating rover-side stream selection, capture, and upload."""

from __future__ import annotations

import json
import os
import tempfile
import threading
from pathlib import Path
from urllib.parse import urlsplit

import rclpy
from mr2_latency_msgs.msg import UplinkTrialStatus
from mr2_latency_msgs.srv import (
    AcquireVideoStreamLease,
    CancelUplinkTrial,
    PrepareUplinkTrial,
    ReleaseVideoStreamLease,
    StartUplinkTrial,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .capture import (
    build_capture_args,
    load_stream_ports,
    probe_upload_target,
    resolve_capture_interface,
    run_capture,
    upload_bytes,
    upload_file,
)
from .trial_state import UplinkTrial, validate_duration, validate_trial_id


class UplinkAgent(Node):
    """Coordinate one rover capture and any stream lease it acquires."""

    def __init__(self) -> None:
        super().__init__("latency_uplink_agent")
        self._callback_group = ReentrantCallbackGroup()
        self._capture_interface = self.declare_parameter("capture_interface", "").value
        self._video_config_path = Path(
            self.declare_parameter("video_config_path", "").value
        )
        self._artifact_root = Path(
            self.declare_parameter(
                "artifact_directory", "/tmp/mr2-latency-rover"
            ).value
        )
        self._tcpdump_binary = self.declare_parameter(
            "tcpdump_binary", "tcpdump"
        ).value
        self._max_duration_s = float(
            self.declare_parameter("max_capture_duration_s", 60.0).value
        )
        self._lease_service_name = self.declare_parameter(
            "acquire_stream_lease_service",
            "/video_streaming/acquire_stream_lease",
        ).value
        self._release_service_name = self.declare_parameter(
            "release_stream_lease_service",
            "/video_streaming/release_stream_lease",
        ).value

        self._stream_ports = load_stream_ports(self._video_config_path)
        self._artifact_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._trial: UplinkTrial | None = None
        self._cancel_event = threading.Event()
        self._worker: threading.Thread | None = None

        self._lease_client = self.create_client(
            AcquireVideoStreamLease,
            self._lease_service_name,
            callback_group=self._callback_group,
        )
        self._release_client = self.create_client(
            ReleaseVideoStreamLease,
            self._release_service_name,
            callback_group=self._callback_group,
        )
        self._status_pub = self.create_publisher(
            UplinkTrialStatus, "/latency/uplink/status", 10
        )
        self._prepare_service = self.create_service(
            PrepareUplinkTrial,
            "/latency/uplink/prepare",
            self._prepare,
            callback_group=self._callback_group,
        )
        self._start_service = self.create_service(
            StartUplinkTrial,
            "/latency/uplink/start",
            self._start,
            callback_group=self._callback_group,
        )
        self._cancel_service = self.create_service(
            CancelUplinkTrial,
            "/latency/uplink/cancel",
            self._cancel,
            callback_group=self._callback_group,
        )

    def _publish_status(self) -> None:
        with self._lock:
            trial = self._trial
            if trial is None:
                return
            message = UplinkTrialStatus()
            message.trial_id = trial.trial_id
            message.phase = trial.phase
            message.progress = float(trial.progress)
            message.error_code = trial.error_code
            message.message = trial.message
            message.stream_ids = trial.stream_ids
        self._status_pub.publish(message)

    @staticmethod
    def _wait_for_future(future, timeout_s: float):
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout_s):
            raise TimeoutError("ROS service call timed out")
        error = future.exception()
        if error is not None:
            raise error
        return future.result()

    def _prepare(self, request, response):
        try:
            trial_id = validate_trial_id(request.trial_id)
            stream_ids = list(dict.fromkeys(request.stream_ids))
            if not stream_ids:
                raise ValueError("at least one stream is required")
            missing = [item for item in stream_ids if item not in self._stream_ports]
            if missing:
                raise ValueError(f"unknown stream(s): {', '.join(missing)}")
            with self._lock:
                if self._trial is not None and self._trial.phase not in {
                    "completed",
                    "failed",
                    "cancelled",
                }:
                    raise ValueError("another rover uplink trial is active")

            lease_acquired = self._lease_client.wait_for_service(
                timeout_sec=3.0
            )
            if lease_acquired:
                lease_request = AcquireVideoStreamLease.Request()
                lease_request.owner_id = trial_id
                lease_request.stream_ids = stream_ids
                lease_request.lease_timeout_s = self._max_duration_s + 30.0
                lease = self._wait_for_future(
                    self._lease_client.call_async(lease_request), 5.0
                )
                if not lease.accepted:
                    raise RuntimeError(
                        lease.message or "video stream lease was rejected"
                    )
            else:
                self.get_logger().warning(
                    f"Video stream lease service {self._lease_service_name} "
                    "is unavailable; continuing with selected-port capture"
                )

            with self._lock:
                self._trial = UplinkTrial(
                    trial_id=trial_id,
                    stream_ids=stream_ids,
                    lease_acquired=lease_acquired,
                )
                self._cancel_event = threading.Event()
            response.accepted = True
            response.message = (
                "rover uplink trial prepared"
                if lease_acquired
                else "rover uplink trial prepared with selected-port capture"
            )
            self._publish_status()
        except (OSError, RuntimeError, TimeoutError, ValueError) as error:
            response.accepted = False
            response.message = str(error)
        return response

    def _start(self, request, response):
        try:
            duration_s = validate_duration(request.duration_s, self._max_duration_s)
            parsed_url = urlsplit(request.upload_base_url)
            if parsed_url.scheme not in {"http", "https"} or not parsed_url.hostname:
                raise ValueError("invalid upload_base_url")
            if not 16 <= len(request.upload_token) <= 512:
                raise ValueError("invalid upload token")
            with self._lock:
                trial = self._trial
                if trial is None or trial.trial_id != request.trial_id:
                    raise ValueError("trial_id is not prepared")
                if trial.phase != "prepared":
                    raise ValueError(f"trial cannot start from phase {trial.phase}")
                ports = [self._stream_ports[item] for item in trial.stream_ids]
            capture_interface = resolve_capture_interface(
                self._capture_interface, parsed_url.hostname
            )
            command = build_capture_args(
                self._tcpdump_binary,
                capture_interface,
                Path("rover.pcap"),
                ports,
            )
            probe_upload_target(request.upload_base_url.rstrip("/"), trial.trial_id)
            if not self._capture_interface:
                self.get_logger().info(
                    f"Resolved latency capture interface {capture_interface} "
                    f"from route to {parsed_url.hostname}"
                )
            with self._lock:
                trial = self._trial
                if trial is None or trial.trial_id != request.trial_id:
                    raise ValueError("trial_id is not prepared")
                trial.transition("capturing", "Capturing rover RTP", 0.2)
                self._worker = threading.Thread(
                    target=self._run_trial,
                    args=(
                        trial.trial_id,
                        duration_s,
                        request.upload_base_url.rstrip("/"),
                        request.upload_token,
                        capture_interface,
                    ),
                    daemon=True,
                )
                self._worker.start()
            response.accepted = True
            response.message = "rover RTP capture started"
            self._publish_status()
        except (RuntimeError, ValueError) as error:
            response.accepted = False
            response.message = str(error)
        return response

    def _cancel(self, request, response):
        release_now = False
        with self._lock:
            trial = self._trial
            if trial is None or trial.trial_id != request.trial_id:
                response.message = "trial_id is not active"
                return response
            if trial.phase in {"completed", "failed", "cancelled"}:
                response.message = f"trial is already {trial.phase}"
                return response
            release_now = trial.phase == "prepared" and trial.lease_acquired
            trial.cancel()
            self._cancel_event.set()
        if release_now:
            self._release_lease(request.trial_id)
        response.cancelled = True
        response.message = "rover uplink trial cancellation requested"
        self._publish_status()
        return response

    def _run_trial(
        self,
        trial_id: str,
        duration_s: float,
        upload_base_url: str,
        upload_token: str,
        capture_interface: str,
    ) -> None:
        trial_directory = Path(
            tempfile.mkdtemp(prefix="uplink-", dir=self._artifact_root)
        )
        capture_path = trial_directory / "rover.pcap"
        metadata_path = trial_directory / "rover.metadata.json"
        try:
            with self._lock:
                trial = self._trial
                if trial is None or trial.trial_id != trial_id:
                    return
                ports = [self._stream_ports[item] for item in trial.stream_ids]
                stream_ids = list(trial.stream_ids)
                lease_acquired = trial.lease_acquired
            command = build_capture_args(
                self._tcpdump_binary,
                capture_interface,
                capture_path,
                ports,
            )
            exit_code, stderr, started_us, finished_us = run_capture(
                command, duration_s, self._cancel_event
            )
            metadata = {
                "schema_version": 1,
                "kind": "mr2_rtp_capture_metadata",
                "role": "rover",
                "trial_id": trial_id,
                "hostname": os.uname().nodename,
                "interface": capture_interface,
                "stream_ids": stream_ids,
                "stream_lease_acquired": lease_acquired,
                "started_epoch_us": started_us,
                "finished_epoch_us": finished_us,
                "tcpdump_exit_code": exit_code,
                "tcpdump_stderr": stderr,
            }
            metadata_path.write_text(
                json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
            )
            if self._cancel_event.is_set():
                return
            if exit_code != 0:
                raise RuntimeError(f"tcpdump exited with code {exit_code}")

            with self._lock:
                if self._trial is not None:
                    self._trial.transition("uploading", "Uploading rover capture", 0.75)
            self._publish_status()
            upload_bytes(
                f"{upload_base_url}/rover-metadata",
                upload_token,
                "application/json",
                metadata_path.read_bytes(),
            )
            upload_file(
                f"{upload_base_url}/rover-capture",
                upload_token,
                "application/vnd.tcpdump.pcap",
                capture_path,
            )
            with self._lock:
                if self._trial is not None:
                    self._trial.transition(
                        "restoring_streams",
                        (
                            "Restoring video streams"
                            if lease_acquired
                            else "Finalizing selected-port capture"
                        ),
                        0.95,
                    )
            self._publish_status()
        except (OSError, RuntimeError, TimeoutError, ValueError) as error:
            with self._lock:
                if self._trial is not None and not self._cancel_event.is_set():
                    self._trial.fail("rover_capture_failed", str(error))
            self._publish_status()
        finally:
            with self._lock:
                lease_acquired = (
                    self._trial is not None
                    and self._trial.trial_id == trial_id
                    and self._trial.lease_acquired
                )
            released = not lease_acquired or self._release_lease(trial_id)
            with self._lock:
                if self._trial is not None and self._trial.trial_id == trial_id:
                    if self._cancel_event.is_set():
                        self._trial.cancel()
                    elif not released:
                        self._trial.fail(
                            "stream_restore_failed",
                            "Video stream lease could not be released",
                        )
                    elif self._trial.phase == "restoring_streams":
                        self._trial.transition("completed", "Rover upload complete", 1.0)
            self._publish_status()

    def _release_lease(self, trial_id: str) -> bool:
        try:
            if not self._release_client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError("video stream release service is unavailable")
            request = ReleaseVideoStreamLease.Request()
            request.owner_id = trial_id
            response = self._wait_for_future(
                self._release_client.call_async(request), 5.0
            )
            if not response.released:
                raise RuntimeError(response.message or "stream lease release failed")
            return True
        except (RuntimeError, TimeoutError) as error:
            self.get_logger().error(str(error))
            return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UplinkAgent()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
