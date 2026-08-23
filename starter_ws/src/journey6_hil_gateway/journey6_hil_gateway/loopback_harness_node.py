"""Real ROS 2 split-loopback exerciser and evidence collector."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time

from .emulation import (
    FORMAL_STATUS_EVALUATOR_BLOCKER,
    audit_gazebo_sensor_provenance,
    derive_algorithm_host_full_stack_pass,
    evaluate_loopback_report,
    synthetic_sensor_publishers_allowed,
    validate_run_id,
    validate_qos_evidence,
)


def _atomic_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def _read_json(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _set_stamp(message, stamp_s: float) -> None:
    seconds = int(stamp_s)
    message.sec = seconds
    message.nanosec = int(round((stamp_s - seconds) * 1_000_000_000.0))


def main(args=None) -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage

    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    static_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    reliable_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    control_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=0.08),
        lifespan=Duration(seconds=0.12),
    )

    class LoopbackHarness(Node):
        def __init__(self) -> None:
            super().__init__("journey6_loopback_harness", namespace="/pc")
            # Compose passes whole-second durations as an integer ROS argument.
            self.declare_parameter("duration_s", 30)
            self.declare_parameter("sensor_source", "synthetic_transport_probe")
            self.declare_parameter("runtime_backend", "PC_ONNX")
            self.declare_parameter("not_journey6_runtime", True)
            self.declare_parameter("run_id", "")
            self.declare_parameter("evidence_directory", "/evidence")
            self.declare_parameter("apply_network_faults", False)
            self.duration_s = float(self.get_parameter("duration_s").value)
            if self.duration_s < 10.0:
                raise ValueError("loopback harness duration must be at least 10 seconds")
            self.sensor_source = str(self.get_parameter("sensor_source").value)
            if self.sensor_source not in {"synthetic_transport_probe", "gazebo"}:
                raise ValueError("sensor_source must be synthetic_transport_probe or gazebo")
            self.runtime_backend = str(
                self.get_parameter("runtime_backend").value
            )
            self.not_journey6_runtime = bool(
                self.get_parameter("not_journey6_runtime").value
            )
            if self.runtime_backend != "PC_ONNX" or not self.not_journey6_runtime:
                raise ValueError("V2 harness requires PC_ONNX/not_journey6_runtime=true")
            self.run_id = validate_run_id(str(self.get_parameter("run_id").value))
            self.evidence_directory = Path(
                str(self.get_parameter("evidence_directory").value)
            )
            self.apply_network_faults = bool(
                self.get_parameter("apply_network_faults").value
            )
            self.started_monotonic = time.monotonic()
            self.finished = False
            self.events_done: set[str] = set()
            self.fake_planner = None
            self.validated_count = 0
            self.zero_count = 0
            self.nonzero_count = 0
            self.nonzero_sources: set[str] = set()
            self.gateway_reasons: set[str] = set()
            self.zero_during_timeout = False
            self.zero_during_network = False
            self.zero_after_network_before_resume = False
            self.zero_during_blacklist = False
            self.zero_after_stale_replay = False
            self.zero_during_estop = False
            self.nonzero_after_initial_resume = False
            self.nonzero_after_network_resume = False
            self.manual_resume_count = 0
            self.sensor_frames_published = 0

            if synthetic_sensor_publishers_allowed(self.sensor_source):
                self.clock_publisher = self.create_publisher(
                    Clock, "/hil/clock", sensor_qos
                )
                self.color_publisher = self.create_publisher(
                    Image, "/hil/camera/color", sensor_qos
                )
                self.depth_publisher = self.create_publisher(
                    Image, "/hil/camera/depth", sensor_qos
                )
                self.camera_info_publisher = self.create_publisher(
                    CameraInfo, "/hil/camera/camera_info", sensor_qos
                )
                self.tf_publisher = self.create_publisher(
                    TFMessage, "/hil/tf", sensor_qos
                )
                self.tf_static_publisher = self.create_publisher(
                    TFMessage, "/hil/tf_static", static_qos
                )
            self.resume_publisher = self.create_publisher(
                Bool, "/hil/operator/resume", reliable_qos
            )
            self.pause_publisher = self.create_publisher(
                Bool, "/hil/harness/algorithm_pause", reliable_qos
            )
            self.network_publisher = self.create_publisher(
                String, "/hil/harness/network_fault", reliable_qos
            )
            self.estop_publisher = self.create_publisher(
                Bool, "/hil/safety/estop_request", control_qos
            )
            self.spoof_publisher = self.create_publisher(
                String, "/hil/vehicle/ackermann_command", control_qos
            )
            self.create_subscription(
                String,
                "/hil/vehicle/validated_ackermann_command",
                self._on_validated_command,
                control_qos,
            )
            self.create_subscription(
                String, "/hil/gateway/health", self._on_gateway_health, reliable_qos
            )
            self.create_timer(0.05, self._on_tick)
            if synthetic_sensor_publishers_allowed(self.sensor_source):
                self.create_timer(0.10, self._publish_sensor_frame)
                self._publish_static_tf()

        def elapsed(self) -> float:
            return max(0.0, time.monotonic() - self.started_monotonic)

        def _publish_static_tf(self) -> None:
            transform = TransformStamped()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "camera_link"
            transform.transform.rotation.w = 1.0
            self.tf_static_publisher.publish(TFMessage(transforms=[transform]))

        def _publish_sensor_frame(self) -> None:
            if self.sensor_source != "synthetic_transport_probe" or self.finished:
                return
            stamp_s = self.elapsed()
            clock = Clock()
            _set_stamp(clock.clock, stamp_s)
            self.clock_publisher.publish(clock)

            color = Image()
            _set_stamp(color.header.stamp, stamp_s)
            color.header.frame_id = "camera_link"
            color.height = 64
            color.width = 64
            color.encoding = "rgb8"
            color.step = 64 * 3
            color.data = bytes([32, 96, 160]) * (64 * 64)

            depth = Image()
            _set_stamp(depth.header.stamp, stamp_s)
            depth.header.frame_id = "camera_link"
            depth.height = 64
            depth.width = 64
            depth.encoding = "16UC1"
            depth.step = 64 * 2
            depth.data = (1000).to_bytes(2, "little") * (64 * 64)

            camera_info = CameraInfo()
            _set_stamp(camera_info.header.stamp, stamp_s)
            camera_info.header.frame_id = "camera_link"
            camera_info.height = 64
            camera_info.width = 64
            camera_info.k = [50.0, 0.0, 32.0, 0.0, 50.0, 32.0, 0.0, 0.0, 1.0]

            transform = TransformStamped()
            _set_stamp(transform.header.stamp, stamp_s)
            transform.header.frame_id = "map"
            transform.child_frame_id = "base_link"
            transform.transform.rotation.w = 1.0

            self.camera_info_publisher.publish(camera_info)
            self.tf_publisher.publish(TFMessage(transforms=[transform]))
            self.color_publisher.publish(color)
            self.depth_publisher.publish(depth)
            self.sensor_frames_published += 1

        def _publish_resume(self, label: str) -> None:
            self.resume_publisher.publish(Bool(data=True))
            self.manual_resume_count += 1
            self.events_done.add(label)

        def _on_tick(self) -> None:
            elapsed = self.elapsed()
            if elapsed >= self.duration_s:
                self.finished = True
                return
            if elapsed >= 3.0 and "initial_resume" not in self.events_done:
                self._publish_resume("initial_resume")
            if elapsed >= 5.0 and "pause_start" not in self.events_done:
                self.pause_publisher.publish(Bool(data=True))
                self.events_done.add("pause_start")
            if elapsed >= 6.0 and "pause_end" not in self.events_done:
                self.pause_publisher.publish(Bool(data=False))
                self.events_done.add("pause_end")

            network_start = max(8.0, self.duration_s * 0.35)
            if elapsed >= network_start and "network_start" not in self.events_done:
                request = {"profile": "disconnect", "duration_s": 2.0}
                self.network_publisher.publish(
                    String(data=json.dumps(request, sort_keys=True))
                )
                self.events_done.add("network_start")
            if elapsed >= network_start + 3.5 and "network_resume" not in self.events_done:
                self._publish_resume("network_resume")

            blacklist_start = max(13.0, self.duration_s * 0.65)
            if elapsed >= blacklist_start and "blacklist_start" not in self.events_done:
                self.fake_planner = rclpy.create_node("planner_server")
                self.events_done.add("blacklist_start")
            if elapsed >= blacklist_start + 2.0 and "blacklist_end" not in self.events_done:
                if self.fake_planner is not None:
                    self.fake_planner.destroy_node()
                    self.fake_planner = None
                self.events_done.add("blacklist_end")
            if elapsed >= blacklist_start + 3.5 and "blacklist_resume" not in self.events_done:
                self._publish_resume("blacklist_resume")

            stale_start = max(17.0, self.duration_s * 0.80)
            if elapsed >= stale_start and "stale_start" not in self.events_done:
                now = elapsed
                stale = {
                    "stamp_s": now,
                    "sequence": 0,
                    "speed_mps": 0.2,
                    "steering_angle_rad": 0.0,
                    "acceleration_limit_mps2": 0.5,
                    "source_id": "j6-algorithm",
                    "valid_until_s": now + 0.15,
                }
                self.spoof_publisher.publish(
                    String(data=json.dumps(stale, sort_keys=True))
                )
                self.events_done.add("stale_start")
            if elapsed >= stale_start + 1.5 and "stale_resume" not in self.events_done:
                self._publish_resume("stale_resume")

            estop_start = max(20.0, self.duration_s * 0.90)
            if elapsed >= estop_start and "estop_start" not in self.events_done:
                self.estop_publisher.publish(Bool(data=True))
                self.events_done.add("estop_start")
            if elapsed >= estop_start + 1.0 and "estop_resume" not in self.events_done:
                self._publish_resume("estop_resume")

        def _on_gateway_health(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            reason = payload.get("reason")
            if isinstance(reason, str):
                self.gateway_reasons.add(reason)

        def _on_validated_command(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                speed = float(payload["speed_mps"])
                source = str(payload["source_id"])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                return
            self.validated_count += 1
            elapsed = self.elapsed()
            if abs(speed) <= 1.0e-9:
                self.zero_count += 1
                if 5.0 <= elapsed <= 6.5:
                    self.zero_during_timeout = True
                network_start = max(8.0, self.duration_s * 0.35)
                if network_start <= elapsed <= network_start + 2.5:
                    self.zero_during_network = True
                if network_start + 2.0 <= elapsed <= network_start + 3.5:
                    self.zero_after_network_before_resume = True
                blacklist_start = max(13.0, self.duration_s * 0.65)
                if blacklist_start <= elapsed <= blacklist_start + 3.0:
                    self.zero_during_blacklist = True
                stale_start = max(17.0, self.duration_s * 0.80)
                if stale_start <= elapsed <= stale_start + 1.5:
                    self.zero_after_stale_replay = True
                estop_start = max(20.0, self.duration_s * 0.90)
                if estop_start <= elapsed <= estop_start + 1.0:
                    self.zero_during_estop = True
            else:
                self.nonzero_count += 1
                self.nonzero_sources.add(source)
                if elapsed >= 3.0:
                    self.nonzero_after_initial_resume = True
                network_start = max(8.0, self.duration_s * 0.35)
                if elapsed >= network_start + 3.5:
                    self.nonzero_after_network_resume = True

        def build_report(self) -> dict[str, object]:
            duration = self.elapsed()
            algorithm = _read_json(
                self.evidence_directory / "HIL_ALGORITHM_RUNTIME.json"
            )
            placement = _read_json(
                self.evidence_directory / "HIL_NODE_PLACEMENT.json"
            )
            transport_raw = algorithm.get("transport", {})
            if not isinstance(transport_raw, dict):
                transport_raw = {}
            topic_names = {name for name, _ in self.get_topic_names_and_types()}
            forbidden_topics = sorted(
                name
                for name in topic_names
                if name.startswith(("/ground_truth", "/world", "/sealed"))
            )
            qos_evidence_path = self.evidence_directory / "HIL_ROS_QOS_INFO.txt"
            try:
                qos_contract_pass = validate_qos_evidence(
                    qos_evidence_path.read_text(encoding="utf-8")
                )
            except OSError:
                qos_contract_pass = False
            transport = {
                **transport_raw,
                "sensor_source": self.sensor_source,
                "qos_contract_pass": qos_contract_pass,
                "qos_evidence_file": qos_evidence_path.name,
                "tf_received": int(transport_raw.get("tf_count", 0)) > 0,
                "tf_static_received": int(transport_raw.get("tf_static_count", 0)) > 0,
                "image_depth_sync_pass": (
                    int(transport_raw.get("synchronized_pair_count", 0)) >= 10
                    and int(transport_raw.get("rejected_unsynchronized_pair_count", 0)) == 0
                ),
                "ground_truth_topics_observed": forbidden_topics,
            }
            actual_network = (
                algorithm.get("actual_network_fault_applied") is True
                and algorithm.get("actual_network_restore_applied") is True
            )
            safety = {
                "steady_state_pc_duplicate_algorithm_nodes": placement.get(
                    "pc_duplicate_algorithm_nodes"
                ),
                "pc_blacklist_injection_detected": (
                    "pc_duplicate_algorithm_node" in self.gateway_reasons
                ),
                "nonzero_authority_pass": self.nonzero_count > 0
                and self.nonzero_sources == {"j6-algorithm"},
                "command_timeout_safe_stop": self.zero_during_timeout,
                "actual_network_loss_safe_stop": actual_network
                and self.zero_during_network,
                "network_reconnect_requires_manual_resume": (
                    self.zero_after_network_before_resume
                    and self.nonzero_after_network_resume
                ),
                "no_stale_command_replay": self.zero_after_stale_replay,
                "estop_safe_stop": self.zero_during_estop,
                "pc_blacklist_safe_stop": self.zero_during_blacklist,
                "ground_truth_control_violation_count": len(forbidden_topics),
            }
            report: dict[str, object] = {
                "schema_version": 2,
                "runtime_backend": self.runtime_backend,
                "not_journey6_runtime": self.not_journey6_runtime,
                "run_id": self.run_id,
                "formal_attestation_evaluator_available": False,
                "status_evaluator_blocked": FORMAL_STATUS_EVALUATOR_BLOCKER,
                "actual_ros2_processes": True,
                "duration_s": duration,
                "required_duration_s": 1800.0,
                "sensor_source": self.sensor_source,
                "sensor_provenance": audit_gazebo_sensor_provenance(
                    self.evidence_directory / "HIL_GAZEBO_SENSOR_PROVENANCE.json",
                    run_id=self.run_id,
                ),
                "sensor_frames_published": self.sensor_frames_published,
                "validated_command_count": self.validated_count,
                "zero_command_count": self.zero_count,
                "nonzero_command_count": self.nonzero_count,
                "manual_resume_count": self.manual_resume_count,
                "transport": transport,
                "algorithm": algorithm,
                "safety": safety,
                "official_journey6_runtime_evidence": False,
                "official_runtime_attestation": None,
                "complete_node_inventory": {
                    "run_id": self.run_id,
                    "complete": False,
                    "nodes": sorted(self.get_node_names()),
                    "reason": FORMAL_STATUS_EVALUATOR_BLOCKER,
                },
            }
            report["algorithm_host_full_stack_pass"] = (
                derive_algorithm_host_full_stack_pass(report)
            )
            report["statuses"] = evaluate_loopback_report(report)
            return report

    rclpy.init(args=args)
    node = LoopbackHarness()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node.fake_planner is not None:
            node.fake_planner.destroy_node()
        report = node.build_report()
        _atomic_json(
            node.evidence_directory / "J6_LOOPBACK_HIL_EMULATION_REPORT.json",
            report,
        )
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
