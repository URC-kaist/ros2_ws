import time
from typing import Any, Dict, Iterable, Tuple

import psutil
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SystemStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("system_status")

        publish_rate_hz = 1.0
        base_topic = "system_status"

        self.cpu_pub = self.create_publisher(DiagnosticArray, f"{base_topic}/cpu", 10)
        self.memory_pub = self.create_publisher(
            DiagnosticArray, f"{base_topic}/memory", 10
        )
        self.swap_pub = self.create_publisher(DiagnosticArray, f"{base_topic}/swap", 10)
        self.disk_pub = self.create_publisher(
            DiagnosticArray, f"{base_topic}/disk", 10
        )
        self.network_pub = self.create_publisher(
            DiagnosticArray, f"{base_topic}/network", 10
        )
        self.temps_pub = self.create_publisher(
            DiagnosticArray, f"{base_topic}/temperatures", 10
        )
        self.load_pub = self.create_publisher(
            DiagnosticArray, f"{base_topic}/load", 10
        )
        self.uptime_pub = self.create_publisher(
            DiagnosticArray, f"{base_topic}/uptime", 10
        )

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish)
        self._temps_ok = True

        # Prime psutil's CPU percent to avoid a misleading first sample.
        psutil.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None, percpu=True)

    def _iter_kv(self, payload: Dict[str, Any]) -> Iterable[Tuple[str, str]]:
        for key, value in payload.items():
            if isinstance(value, (list, tuple)):
                for idx, entry in enumerate(value):
                    yield f"{key}.{idx}", str(entry)
            elif isinstance(value, dict):
                for sub_key, sub_val in value.items():
                    yield f"{key}.{sub_key}", str(sub_val)
            else:
                yield key, str(value)

    def _publish_status(
        self, publisher: DiagnosticArray, name: str, payload: Dict[str, Any]
    ) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = name
        status.message = "OK"
        status.hardware_id = ""
        status.values = [KeyValue(key=k, value=v) for k, v in self._iter_kv(payload)]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        publisher.publish(array)

    def publish(self) -> None:
        cpu_payload = {
            "percent_total": psutil.cpu_percent(interval=None),
            "percent_per_core": psutil.cpu_percent(interval=None, percpu=True),
            "count_logical": psutil.cpu_count(logical=True),
            "count_physical": psutil.cpu_count(logical=False),
        }
        self._publish_status(self.cpu_pub, "cpu", cpu_payload)

        memory_payload = psutil.virtual_memory()._asdict()
        self._publish_status(self.memory_pub, "memory", memory_payload)

        swap_payload = psutil.swap_memory()._asdict()
        self._publish_status(self.swap_pub, "swap", swap_payload)

        disk_payload = psutil.disk_usage("/")._asdict()
        self._publish_status(self.disk_pub, "disk", disk_payload)

        network_payload = psutil.net_io_counters()._asdict()
        self._publish_status(self.network_pub, "network", network_payload)

        temps_payload: Dict[str, Any] = {}
        if self._temps_ok and hasattr(psutil, "sensors_temperatures"):
            try:
                temps = psutil.sensors_temperatures(fahrenheit=False) or {}
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to read temperatures; disabling temperature polling: {exc}"
                )
                self._temps_ok = False
                temps = {}
            for name, entries in temps.items():
                for idx, entry in enumerate(entries):
                    if entry.current is None:
                        continue
                    label = entry.label or f"sensor_{idx}"
                    key = f"{name}.{label}"
                    temps_payload[key] = float(entry.current)
        self._publish_status(self.temps_pub, "temperatures", temps_payload)

        try:
            load_avg = psutil.getloadavg()
        except (AttributeError, OSError):
            load_avg = None
        load_payload = {"load_avg": load_avg}
        self._publish_status(self.load_pub, "load", load_payload)

        uptime_payload = {
            "uptime_sec": time.time() - psutil.boot_time(),
        }
        self._publish_status(self.uptime_pub, "uptime", uptime_payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
