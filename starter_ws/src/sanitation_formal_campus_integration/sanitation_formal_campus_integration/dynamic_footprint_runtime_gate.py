"""Live ROS gate for the formal dynamic Nav2 footprint contract.

The gate is intentionally unable to write ``/joint_states`` or an actuator
endpoint. It requires the manager's opt-in, inhibited-only test override and
then reads both real Nav2 costmap outputs back after every fresh request.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid
from pathlib import Path

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .dynamic_footprint_core import (
    load_footprints,
    polygons_exactly_equal,
    run_with_fail_closed_cleanup,
)
from .dynamic_footprint_manager import RUNTIME_TEST_OVERRIDE_TOPIC

INPUT_TOPICS = ("/local_costmap/footprint", "/global_costmap/footprint")
PUBLISHED_TOPICS = (
    "/local_costmap/published_footprint",
    "/global_costmap/published_footprint",
)
INHIBIT_TOPIC = "/manipulation/base_motion_inhibited"
STATUS_TOPIC = "/formal_vehicle/navigation/footprint_status"
SAFETY_STATUS_TOPIC = "/safety/status_json"
MANAGER_NODE = "formal_dynamic_footprint_manager"
SAFETY_NODE = "whole_vehicle_safety_manager"
ROOT_NAMESPACE = "/"
NO_PUBLISH_THREAD_ERROR = "none"
POLYGON_TYPE = "geometry_msgs/msg/Polygon"
POLYGON_STAMPED_TYPE = "geometry_msgs/msg/PolygonStamped"
BOOL_TYPE = "std_msgs/msg/Bool"
STRING_TYPE = "std_msgs/msg/String"
SAFE_BASE_STOP_STATES = {"BASE_COMMAND_STOPPED"}


class DynamicFootprintRuntimeGate(Node):
    """Fail closed unless the graph proves a fresh, inhibited exact readback."""

    def __init__(self, profile_path: Path, timeout_sec: float) -> None:
        super().__init__("formal_dynamic_footprint_runtime_gate")
        self._footprints = load_footprints(profile_path)
        self._timeout_sec = timeout_sec
        self._latest_status: tuple[int, dict[str, object]] | None = None
        self._status_receipts = 0
        self._latest_safety_status: tuple[int, dict[str, object]] | None = None
        self._safety_status_receipts = 0
        self._latest_published: dict[str, tuple[int, PolygonStamped]] = {}
        self._published_receipts = {topic: 0 for topic in PUBLISHED_TOPICS}
        self._override_publisher = self.create_publisher(
            String, RUNTIME_TEST_OVERRIDE_TOPIC, 10
        )
        # The gate only ever asserts inhibition. It never clears it when it
        # exits, so an interrupted gate leaves the physical base safe.
        self._inhibit_publisher = self.create_publisher(Bool, INHIBIT_TOPIC, 10)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(String, SAFETY_STATUS_TOPIC, self._on_safety_status, 10)
        for topic in PUBLISHED_TOPICS:
            self.create_subscription(
                PolygonStamped,
                topic,
                lambda message, topic=topic: self._on_published(topic, message),
                10,
            )

    def _on_status(self, message: String) -> None:
        self._status_receipts += 1
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self._latest_status = None
            return
        if isinstance(payload, dict):
            self._latest_status = (self._status_receipts, payload)
        else:
            self._latest_status = None

    def _on_safety_status(self, message: String) -> None:
        self._safety_status_receipts += 1
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self._latest_safety_status = None
            return
        if isinstance(payload, dict):
            self._latest_safety_status = (self._safety_status_receipts, payload)
        else:
            self._latest_safety_status = None

    def _on_published(self, topic: str, message: PolygonStamped) -> None:
        self._published_receipts[topic] += 1
        self._latest_published[topic] = (self._published_receipts[topic], message)

    @staticmethod
    def _types(node: Node, topic: str) -> set[str]:
        for name, types in node.get_topic_names_and_types():
            if name == topic:
                return set(types)
        return set()

    @staticmethod
    def _has_node(
        endpoints: object,
        node_name: str,
        topic_type: str,
        node_namespace: str = ROOT_NAMESPACE,
    ) -> bool:
        return any(
            endpoint.node_name == node_name
            and endpoint.node_namespace == node_namespace
            and endpoint.topic_type == topic_type
            for endpoint in endpoints
        )

    @staticmethod
    def _only_node(
        endpoints: object,
        node_name: str,
        topic_type: str,
        node_namespace: str = ROOT_NAMESPACE,
    ) -> bool:
        """Reject a same-topic impostor because ROS callbacks lack publisher IDs."""
        rows = list(endpoints)
        return bool(rows) and all(
            endpoint.node_name == node_name
            and endpoint.node_namespace == node_namespace
            and endpoint.topic_type == topic_type
            for endpoint in rows
        )

    def _require_live_graph(self) -> None:
        """Require the named production publishers/subscribers, not just types."""
        deadline = time.monotonic() + self._timeout_sec
        missing: list[str] = []
        while time.monotonic() < deadline:
            missing.clear()
            for input_topic, published_topic in zip(
                INPUT_TOPICS, PUBLISHED_TOPICS, strict=True
            ):
                costmap_node = input_topic.split("/")[1]
                costmap_namespace = f"/{costmap_node}"
                if self._types(self, input_topic) != {POLYGON_TYPE}:
                    missing.append(f"{input_topic}:type")
                else:
                    publishers = self.get_publishers_info_by_topic(input_topic)
                    subscribers = self.get_subscriptions_info_by_topic(input_topic)
                    if not self._only_node(publishers, MANAGER_NODE, POLYGON_TYPE):
                        missing.append(f"{input_topic}:exclusive_manager_publisher")
                    if not self._has_node(
                        subscribers,
                        costmap_node,
                        POLYGON_TYPE,
                        costmap_namespace,
                    ):
                        missing.append(f"{input_topic}:{costmap_node}_subscriber")
                if self._types(self, published_topic) != {POLYGON_STAMPED_TYPE}:
                    missing.append(f"{published_topic}:type")
                elif not self._only_node(
                    self.get_publishers_info_by_topic(published_topic),
                    costmap_node,
                    POLYGON_STAMPED_TYPE,
                    costmap_namespace,
                ):
                    missing.append(f"{published_topic}:{costmap_node}_publisher")

            if self._types(self, STATUS_TOPIC) != {STRING_TYPE} or not self._only_node(
                self.get_publishers_info_by_topic(STATUS_TOPIC), MANAGER_NODE, STRING_TYPE
            ):
                missing.append("status:manager_publisher")
            if self._types(self, SAFETY_STATUS_TOPIC) != {STRING_TYPE} or not self._only_node(
                self.get_publishers_info_by_topic(SAFETY_STATUS_TOPIC),
                SAFETY_NODE,
                STRING_TYPE,
            ):
                missing.append("safety_status:exclusive_safety_manager_publisher")
            if self._types(self, RUNTIME_TEST_OVERRIDE_TOPIC) != {STRING_TYPE} or not self._has_node(
                self.get_subscriptions_info_by_topic(RUNTIME_TEST_OVERRIDE_TOPIC),
                MANAGER_NODE,
                STRING_TYPE,
            ):
                missing.append("runtime_test_override:opt_in_manager_subscriber")
            inhibit_subscribers = self.get_subscriptions_info_by_topic(INHIBIT_TOPIC)
            if not self._has_node(inhibit_subscribers, MANAGER_NODE, BOOL_TYPE):
                missing.append("base_inhibit:footprint_manager_subscriber")
            if not self._has_node(inhibit_subscribers, SAFETY_NODE, BOOL_TYPE):
                missing.append("base_inhibit:independent_safety_subscriber")
            if not missing:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("live ROS graph contract missing: " + ", ".join(missing))

    def _send_override(self, operation: str, nonce: str, profile: str | None = None) -> None:
        payload: dict[str, object] = {
            "schema_version": 1,
            "operation": operation,
            "nonce": nonce,
        }
        if profile is not None:
            payload["requested_profile"] = profile
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._override_publisher.publish(message)

    def _wait_for_override(self, profile: str, nonce: str) -> None:
        """Require messages received after this command and bound to its nonce."""
        baseline_status = self._status_receipts
        baseline_published = dict(self._published_receipts)
        baseline_safety_receipt = self._safety_status_receipts
        baseline_safety_publish_count = -1
        if self._latest_safety_status is not None:
            prior_count = self._latest_safety_status[1].get("status_publish_count")
            if isinstance(prior_count, int):
                baseline_safety_publish_count = prior_count
        baseline_manager_sequence = -1
        if self._latest_status is not None:
            prior_sequence = self._latest_status[1].get("publish_sequence")
            if isinstance(prior_sequence, int):
                baseline_manager_sequence = prior_sequence
        deadline = time.monotonic() + self._timeout_sec
        while time.monotonic() < deadline:
            # Repeat only safe, idempotent messages. Inhibition is asserted
            # before every override and never deasserted by this gate.
            self._inhibit_publisher.publish(Bool(data=True))
            self._send_override("set", nonce, profile)
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_status is None:
                continue
            status_receipt, status = self._latest_status
            manager_sequence = status.get("publish_sequence")
            if (
                status_receipt <= baseline_status
                or not isinstance(manager_sequence, int)
                or manager_sequence <= baseline_manager_sequence
                or status.get("profile") != profile
                or status.get("requested_profile") != profile
                or status.get("reason") != "runtime_test_override"
                or status.get("runtime_test_nonce") != nonce
                or status.get("runtime_test_override_active") is not True
                or status.get("base_motion_inhibited") is not True
                or status.get("navigation_allowed") is not False
                or status.get("motion_authorized") is not False
            ):
                continue
            if self._latest_safety_status is None:
                continue
            safety_receipt, safety_status = self._latest_safety_status
            safety_publish_count = safety_status.get("status_publish_count")
            active_reasons = safety_status.get("active_reasons")
            reason_codes = (
                set(active_reasons.split(",")) if isinstance(active_reasons, str) else set()
            )
            if (
                safety_receipt <= baseline_safety_receipt
                or not isinstance(safety_publish_count, int)
                or safety_publish_count <= baseline_safety_publish_count
                or safety_status.get("state") not in SAFE_BASE_STOP_STATES
                or "manipulator_base_inhibit" not in reason_codes
                or safety_status.get("publish_thread_error") != NO_PUBLISH_THREAD_ERROR
            ):
                continue
            if all(
                topic in self._latest_published
                and self._latest_published[topic][0] > baseline_published[topic]
                and polygons_exactly_equal(
                    self._latest_published[topic][1].polygon.points,
                    self._footprints[profile],
                )
                for topic in PUBLISHED_TOPICS
            ):
                return
        raise RuntimeError(
            "timed out waiting for fresh exact Nav2 published_footprint readback "
            f"for runtime-test override {profile}"
        )

    def _clear_override(self) -> None:
        baseline_status = self._status_receipts
        deadline = time.monotonic() + self._timeout_sec
        nonce = f"clear-{uuid.uuid4()}"
        while time.monotonic() < deadline:
            self._inhibit_publisher.publish(Bool(data=True))
            self._send_override("clear", nonce)
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_status is None:
                continue
            receipt, status = self._latest_status
            if (
                receipt > baseline_status
                and status.get("runtime_test_override_active") is False
                and status.get("runtime_test_nonce") is None
                and status.get("base_motion_inhibited") is True
                and status.get("navigation_allowed") is False
                and status.get("motion_authorized") is False
                and status.get("profile") == status.get("production_profile")
                and status.get("reason") == status.get("production_reason")
            ):
                return
        raise RuntimeError("runtime test override did not clear back to production decision")

    def run(self) -> dict[str, object]:
        self._require_live_graph()

        def exercise_profiles() -> None:
            for profile in ("transport_stowed", "cleaning_deployed", "arm_deployed"):
                self._wait_for_override(profile, str(uuid.uuid4()))

        # A failed test must also not leave a manager selecting a test-only
        # profile. The base stays inhibited regardless of clear outcome. If
        # both phases fail, retain the profile/readback failure as primary and
        # attach the cleanup failure for diagnosis.
        run_with_fail_closed_cleanup(exercise_profiles, self._clear_override)
        return {
            "result": "PASS",
            "runtime_only": True,
            "input_type": POLYGON_TYPE,
            "published_type": POLYGON_STAMPED_TYPE,
            "required_endpoint_namespace": ROOT_NAMESPACE,
            "profiles_read_back": [
                "transport_stowed",
                "cleaning_deployed",
                "arm_deployed",
            ],
            "base_motion_inhibit_independent_safety_subscriber": True,
            "safety_status_fresh_per_override": True,
            "safety_manager_state": "BASE_COMMAND_STOPPED",
            "safety_manager_reason": "manipulator_base_inhibit",
            "test_override_preserves_base_inhibit": True,
            "test_override_never_authorizes_motion": True,
            "fresh_readback_required_per_override": True,
        }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-profile-file", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.timeout_sec <= 0.0:
        raise ValueError("timeout-sec must be positive")
    if not args.motion_profile_file.is_file():
        raise ValueError("motion-profile-file must identify the frozen formal profile")
    rclpy.init()
    node = DynamicFootprintRuntimeGate(args.motion_profile_file, args.timeout_sec)
    try:
        result = node.run()
        encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(encoded, encoding="utf-8")
        sys.stdout.write(encoded)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - console script entry point
    main()
