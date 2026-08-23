"""ROS 2 PC-side safety gateway for the Journey 6 split HIL graph."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time

from .core import (
    CommandSafetyGate,
    HealthFrame,
    command_from_mapping,
    command_to_mapping,
)
from .placement import audit_pc_nodes


def _atomic_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _linux_process_snapshot() -> list[str]:
    rows: list[str] = []
    proc = Path("/proc")
    if not proc.is_dir():
        return ["process_snapshot_unavailable_non_linux"]
    for entry in sorted(proc.iterdir(), key=lambda item: item.name):
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except (OSError, PermissionError):
            continue
        if command:
            rows.append(f"{entry.name}\t{command}")
    return rows


def main(args=None) -> None:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Bool, String

    control_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=0.08),
        lifespan=Duration(seconds=0.12),
    )
    health_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    class Journey6HilGateway(Node):
        def __init__(self) -> None:
            super().__init__("journey6_hil_gateway")
            self.declare_parameter("j6_source_id", "j6-algorithm")
            self.declare_parameter("health_timeout_s", 0.25)
            self.declare_parameter("maximum_speed_mps", 2.0)
            self.declare_parameter("maximum_steering_angle_rad", 0.60)
            self.declare_parameter("maximum_acceleration_mps2", 1.5)
            self.declare_parameter("command_check_period_s", 0.02)
            self.declare_parameter("placement_audit_period_s", 1.0)
            self.declare_parameter("evidence_directory", "/evidence")
            self.evidence_directory = Path(
                str(self.get_parameter("evidence_directory").value)
            )
            self.gate = CommandSafetyGate(
                j6_source_id=str(self.get_parameter("j6_source_id").value),
                health_timeout_s=float(self.get_parameter("health_timeout_s").value),
                maximum_speed_mps=float(self.get_parameter("maximum_speed_mps").value),
                maximum_steering_angle_rad=float(
                    self.get_parameter("maximum_steering_angle_rad").value
                ),
                maximum_acceleration_mps2=float(
                    self.get_parameter("maximum_acceleration_mps2").value
                ),
            )
            self.command_publisher = self.create_publisher(
                String, "/hil/vehicle/validated_ackermann_command", control_qos
            )
            self.gateway_health_publisher = self.create_publisher(
                String, "/hil/gateway/health", health_qos
            )
            self.create_subscription(
                String,
                "/hil/vehicle/ackermann_command",
                self._on_command,
                control_qos,
            )
            self.create_subscription(
                String, "/hil/health", self._on_health, health_qos
            )
            self.create_subscription(
                Bool,
                "/hil/safety/estop_request",
                self._on_estop_request,
                control_qos,
            )
            self.create_subscription(
                Bool, "/hil/operator/resume", self._on_operator_resume, health_qos
            )
            self.create_timer(
                float(self.get_parameter("command_check_period_s").value),
                self._publish_gated_command,
            )
            self.create_timer(
                float(self.get_parameter("placement_audit_period_s").value),
                self._audit_placement,
            )
            self._audit_placement()
            self._publish_gated_command()

        def _sim_now_s(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

        def _on_command(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise TypeError("command must be a JSON object")
                self.gate.accept(
                    command_from_mapping(payload),
                    now_sim_s=self._sim_now_s(),
                    now_monotonic_s=time.monotonic(),
                )
            except (KeyError, TypeError, ValueError, RuntimeError, json.JSONDecodeError) as error:
                self.gate.trip("invalid_or_unsafe_command")
                self.get_logger().error(f"Journey 6 command rejected: {error}")
            self._publish_gated_command()

        def _on_health(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise TypeError("health must be a JSON object")
                self.gate.update_health(
                    HealthFrame.from_mapping(payload),
                    received_monotonic_s=time.monotonic(),
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                self.gate.trip("invalid_j6_health")
                self.get_logger().error(f"Journey 6 health rejected: {error}")

        def _on_estop_request(self, message: Bool) -> None:
            if message.data:
                self.gate.set_estop(True)
                self._publish_gated_command()

        def _on_operator_resume(self, message: Bool) -> None:
            if not message.data:
                return
            try:
                self.gate.set_estop(False)
                self.gate.operator_resume(now_monotonic_s=time.monotonic())
            except RuntimeError as error:
                self.get_logger().warning(str(error))

        def _node_names(self) -> list[str]:
            names = []
            for name, namespace in self.get_node_names_and_namespaces():
                if namespace == "/":
                    names.append("/" + name)
                else:
                    names.append(namespace.rstrip("/") + "/" + name)
            return names

        def _audit_placement(self) -> None:
            report = audit_pc_nodes(self._node_names())
            self.gate.set_placement_gate(bool(report["placement_gate_pass"]))
            graph = {
                "schema_version": 1,
                "nodes": sorted(
                    [*report["audited_nodes"], *report["remote_j6_nodes"]]
                ),
                "topics": [
                    {"name": name, "types": types}
                    for name, types in sorted(self.get_topic_names_and_types())
                ],
            }
            _atomic_json(self.evidence_directory / "HIL_NODE_PLACEMENT.json", report)
            _atomic_json(self.evidence_directory / "HIL_ROS_GRAPH.json", graph)
            (self.evidence_directory / "HIL_PC_GATEWAY_PROCESS_LIST.txt").write_text(
                "\n".join(_linux_process_snapshot()) + "\n", encoding="utf-8"
            )
            self._write_authority_evidence()

        def _write_authority_evidence(self) -> None:
            snapshot = self.gate.snapshot(now_monotonic_s=time.monotonic())
            _atomic_json(
                self.evidence_directory / "HIL_COMMAND_AUTHORITY.json", snapshot
            )
            self.gateway_health_publisher.publish(
                String(data=json.dumps(snapshot, sort_keys=True))
            )

        def _publish_gated_command(self) -> None:
            command = self.gate.output(
                now_sim_s=self._sim_now_s(), now_monotonic_s=time.monotonic()
            )
            self.command_publisher.publish(
                String(data=json.dumps(command_to_mapping(command), sort_keys=True))
            )

        def shutdown_zero(self) -> None:
            self.gate.trip("gateway_shutdown")
            self._publish_gated_command()

    rclpy.init(args=args)
    node = Journey6HilGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.shutdown_zero()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
