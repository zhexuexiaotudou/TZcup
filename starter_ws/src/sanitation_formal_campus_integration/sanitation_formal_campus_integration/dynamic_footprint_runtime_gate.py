"""Live ROS gate for the formal dynamic Nav2 footprint contract.

The gate is intentionally unable to write ``/joint_states`` or an actuator
endpoint. It requires the manager's opt-in, inhibited-only test override and
then reads both real Nav2 costmap outputs back after every fresh request.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
import math
from numbers import Real
import sys
import time
import uuid
from pathlib import Path

import rclpy
from geometry_msgs.msg import Polygon, PolygonStamped
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .dynamic_footprint_core import (
    atomic_write_fresh_json,
    blocked_runtime_gate_shape,
    float64_zero_ulp_bound,
    fresh_nonzero_stamp,
    load_footprints,
    load_nav2_footprint_padding,
    load_profile_base_frame,
    padded_rigid_point32_match,
    point32_coordinate_quantization_bound,
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
PUBLISHED_FRAME_BY_TOPIC = {
    "/local_costmap/published_footprint": "odom",
    "/global_costmap/published_footprint": "map",
}
PADDING_NODE_BY_INPUT_TOPIC = {
    "/local_costmap/footprint": "/local_costmap/local_costmap",
    "/global_costmap/footprint": "/global_costmap/global_costmap",
}
PADDING_PARAMETER = "footprint_padding"
ROBOT_BASE_FRAME_PARAMETER = "robot_base_frame"
PUBLISHED_STAMP_MAX_AGE_NS = 2_000_000_000


def _read_footprint_padding_response(
    future: object, declared_padding_m: float
) -> tuple[float, str, float]:
    """Fail closed on the Jazzy GetParameters response boundary."""
    response = future.result()
    if response is None:
        raise ValueError("parameter response missing")
    values = getattr(response, "values", None)
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise ValueError("parameter response values must be a sequence")
    if len(values) != 2:
        raise ValueError("parameter result count")
    try:
        padding_value = parameter_value_to_python(values[0])
        frame = parameter_value_to_python(values[1])
    except Exception as error:
        raise ValueError("parameter value conversion") from error
    if isinstance(padding_value, bool) or not isinstance(padding_value, Real):
        raise ValueError("footprint_padding is not a non-bool numeric value")
    value = float(padding_value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("not finite non-negative")
    if not isinstance(frame, str) or not frame or frame.startswith("/"):
        raise ValueError("robot_base_frame is not a relative frame id")
    declared_bound = point32_coordinate_quantization_bound(declared_padding_m, value)
    if abs(value - declared_padding_m) > declared_bound:
        raise ValueError(
            "does not match declared profile padding within Point32 ULP bound"
        )
    return value, frame, declared_bound


PLANAR_ZERO_ULP_BOUND = float64_zero_ulp_bound()


class DynamicFootprintRuntimeGate(Node):
    """Fail closed unless the graph proves a fresh, inhibited exact readback."""

    def __init__(self, profile_path: Path, timeout_sec: float) -> None:
        super().__init__(
            "formal_dynamic_footprint_runtime_gate",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self._footprints = load_footprints(profile_path)
        self._declared_padding_m = load_nav2_footprint_padding(profile_path)
        self._profile_base_frame = load_profile_base_frame(profile_path)
        self._timeout_sec = timeout_sec
        self._latest_status: tuple[int, dict[str, object]] | None = None
        self._status_receipts = 0
        self._latest_safety_status: tuple[int, dict[str, object]] | None = None
        self._safety_status_receipts = 0
        self._latest_published: dict[str, tuple[int, PolygonStamped]] = {}
        self._published_receipts = {topic: 0 for topic in PUBLISHED_TOPICS}
        self._latest_input: dict[str, tuple[int, Polygon]] = {}
        self._input_receipts = {topic: 0 for topic in INPUT_TOPICS}
        self._padding_m: dict[str, float] = {}
        self._padding_quantization_bound_m = 0.0
        self._robot_base_frame: dict[str, str] = {}
        self._profile_to_robot_base_evidence: dict[str, dict[str, float | str]] = {}
        self._last_quantization_bound_m = 0.0
        self._last_failure_reason = "not_run"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._padding_clients = {
            topic: AsyncParameterClient(self, node_name)
            for topic, node_name in PADDING_NODE_BY_INPUT_TOPIC.items()
        }
        self._override_publisher = self.create_publisher(
            String, RUNTIME_TEST_OVERRIDE_TOPIC, 10
        )
        # The gate only ever asserts inhibition. It never clears it when it
        # exits, so an interrupted gate leaves the physical base safe.
        self._inhibit_publisher = self.create_publisher(Bool, INHIBIT_TOPIC, 10)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(String, SAFETY_STATUS_TOPIC, self._on_safety_status, 10)
        for topic in INPUT_TOPICS:
            self.create_subscription(
                Polygon,
                topic,
                lambda message, topic=topic: self._on_input(topic, message),
                10,
            )
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

    def _on_input(self, topic: str, message: Polygon) -> None:
        self._input_receipts[topic] += 1
        self._latest_input[topic] = (self._input_receipts[topic], message)

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

    @staticmethod
    def _stamp_ns(message: PolygonStamped) -> int:
        stamp = message.header.stamp
        if stamp.sec < 0 or stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
            return 0
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _read_footprint_padding(self) -> None:
        """Read the two live Nav2 padding parameters before any override.

        The gate refuses a missing or malformed value instead of assuming the
        Nav2 default.  That binds the padded geometry to the process that
        actually published it.
        """
        deadline = time.monotonic() + self._timeout_sec
        pending: dict[str, object] = {}
        while time.monotonic() < deadline:
            for topic, client in self._padding_clients.items():
                if topic in self._padding_m or topic in pending:
                    continue
                if client.services_are_ready():
                    pending[topic] = client.get_parameters(
                        [PADDING_PARAMETER, ROBOT_BASE_FRAME_PARAMETER]
                    )
            for topic, future in list(pending.items()):
                if not future.done():
                    continue
                del pending[topic]
                try:
                    value, frame, declared_bound = _read_footprint_padding_response(
                        future, self._declared_padding_m
                    )
                    self._padding_quantization_bound_m = max(
                        self._padding_quantization_bound_m, declared_bound
                    )
                    self._padding_m[topic] = value
                    self._robot_base_frame[topic] = frame
                except Exception as error:
                    self._last_failure_reason = f"{topic}:invalid_footprint_padding:{error}"
                    raise RuntimeError(self._last_failure_reason) from error
            if len(self._padding_m) == len(INPUT_TOPICS):
                local, global_ = (self._padding_m[topic] for topic in INPUT_TOPICS)
                peer_bound = point32_coordinate_quantization_bound(local, global_)
                self._padding_quantization_bound_m = max(
                    self._padding_quantization_bound_m, peer_bound
                )
                if abs(local - global_) > peer_bound:
                    self._last_failure_reason = "live_footprint_padding_mismatch_between_costmaps"
                    raise RuntimeError(self._last_failure_reason)
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = sorted(set(INPUT_TOPICS).difference(self._padding_m))
        self._last_failure_reason = "footprint_padding_unavailable:" + ",".join(missing)
        raise RuntimeError(self._last_failure_reason)

    def _frame_rigid_transform(
        self, input_topic: str, published_topic: str, message: PolygonStamped
    ) -> tuple[float, float, float]:
        expected_frame = PUBLISHED_FRAME_BY_TOPIC[published_topic]
        frame = message.header.frame_id
        if frame != expected_frame:
            raise RuntimeError(
                f"{published_topic}:published_frame_mismatch:{frame!r}!={expected_frame!r}"
            )
        try:
            robot_base_frame = self._robot_base_frame[input_topic]
            profile_to_robot = self._tf_buffer.lookup_transform(
                self._profile_base_frame,
                robot_base_frame,
                Time.from_msg(message.header.stamp),
            )
            profile_translation = profile_to_robot.transform.translation
            profile_rotation = profile_to_robot.transform.rotation
            profile_values = (
                profile_translation.x,
                profile_translation.y,
                profile_translation.z,
                profile_rotation.x,
                profile_rotation.y,
                profile_rotation.z,
                profile_rotation.w,
            )
            if not all(math.isfinite(value) for value in profile_values):
                raise RuntimeError("nonfinite profile-to-robot transform")
            profile_yaw = math.atan2(
                2.0 * profile_rotation.w * profile_rotation.z,
                profile_rotation.w * profile_rotation.w - profile_rotation.z * profile_rotation.z,
            )
            if (
                abs(profile_translation.x) > PLANAR_ZERO_ULP_BOUND
                or abs(profile_translation.y) > PLANAR_ZERO_ULP_BOUND
                or profile_rotation.x != 0.0
                or profile_rotation.y != 0.0
                or abs(profile_yaw) > PLANAR_ZERO_ULP_BOUND
                or profile_rotation.w == 0.0
            ):
                raise RuntimeError("profile and robot base frames are not planar-equivalent")
            self._profile_to_robot_base_evidence[input_topic] = {
                "profile_base_frame": self._profile_base_frame,
                "robot_base_frame": robot_base_frame,
                "translation_x_m": float(profile_translation.x),
                "translation_y_m": float(profile_translation.y),
                "translation_z_m": float(profile_translation.z),
                "yaw_rad": profile_yaw,
                "planar_zero_ulp_bound_m": PLANAR_ZERO_ULP_BOUND,
                "planar_zero_ulp_bound_rad": PLANAR_ZERO_ULP_BOUND,
            }
            transform = self._tf_buffer.lookup_transform(
                expected_frame,
                self._profile_base_frame,
                Time.from_msg(message.header.stamp),
            )
        except (TransformException, KeyError, RuntimeError) as error:
            raise RuntimeError(f"{published_topic}:base_to_costmap_transform_unavailable:{error}") from error
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (translation.x, translation.y, rotation.x, rotation.y, rotation.z, rotation.w)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"{published_topic}:nonfinite_base_to_costmap_transform")
        # A Polygon is planar.  A roll/pitch transform makes a two-dimensional
        # rigid comparison invalid, so reject it rather than silently project.
        if rotation.x != 0.0 or rotation.y != 0.0:
            raise RuntimeError(f"{published_topic}:nonplanar_base_to_costmap_transform")
        norm = rotation.z * rotation.z + rotation.w * rotation.w
        if norm == 0.0:
            raise RuntimeError(f"{published_topic}:invalid_base_to_costmap_rotation")
        yaw = math.atan2(2.0 * rotation.w * rotation.z, rotation.w * rotation.w - rotation.z * rotation.z)
        return float(translation.x), float(translation.y), yaw

    def _fresh_frame_aware_readback(
        self,
        profile: str,
        baseline_input: dict[str, int],
        baseline_published: dict[str, int],
        baseline_published_stamp: dict[str, int],
    ) -> bool:
        now_ros_ns = self.get_clock().now().nanoseconds
        for input_topic, published_topic in zip(INPUT_TOPICS, PUBLISHED_TOPICS, strict=True):
            input_entry = self._latest_input.get(input_topic)
            if input_entry is None or input_entry[0] <= baseline_input[input_topic]:
                self._last_failure_reason = f"{input_topic}:raw_input_not_fresh"
                return False
            if not polygons_exactly_equal(input_entry[1].points, self._footprints[profile]):
                self._last_failure_reason = f"{input_topic}:raw_input_not_exact_profile"
                return False
            published_entry = self._latest_published.get(published_topic)
            if published_entry is None or published_entry[0] <= baseline_published[published_topic]:
                self._last_failure_reason = f"{published_topic}:published_readback_not_fresh"
                return False
            message = published_entry[1]
            fresh, reason = fresh_nonzero_stamp(
                self._stamp_ns(message),
                baseline_published_stamp[published_topic],
                now_ros_ns,
                PUBLISHED_STAMP_MAX_AGE_NS,
            )
            if not fresh:
                self._last_failure_reason = f"{published_topic}:{reason}"
                return False
            try:
                tx, ty, yaw = self._frame_rigid_transform(input_topic, published_topic, message)
                matched, bound, reason = padded_rigid_point32_match(
                    message.polygon.points,
                    self._footprints[profile],
                    self._padding_m[input_topic],
                    tx,
                    ty,
                    yaw,
                )
            except (RuntimeError, ValueError) as error:
                self._last_failure_reason = str(error)
                return False
            self._last_quantization_bound_m = max(self._last_quantization_bound_m, bound)
            if not matched:
                self._last_failure_reason = f"{published_topic}:{reason}"
                return False
        self._last_failure_reason = "ok"
        return True

    @staticmethod
    def _polygon_snapshot(message: Polygon | PolygonStamped | None) -> dict[str, object] | None:
        if message is None:
            return None
        polygon = message if isinstance(message, Polygon) else message.polygon
        snapshot: dict[str, object] = {
            "points": [[float(point.x), float(point.y), float(point.z)] for point in polygon.points]
        }
        if isinstance(message, PolygonStamped):
            snapshot["frame_id"] = message.header.frame_id
            snapshot["stamp_ns"] = DynamicFootprintRuntimeGate._stamp_ns(message)
        return snapshot

    def blocked_result(self, reason: str) -> dict[str, object]:
        """Return diagnostic evidence for every failure path without a PASS shape."""
        self._last_failure_reason = reason
        return {
            "result": "BLOCKED",
            "passed": False,
            "runtime_only": True,
            "reason": reason,
            "last_input": {
                topic: {
                    "receipt": self._latest_input[topic][0] if topic in self._latest_input else 0,
                    "polygon": self._polygon_snapshot(self._latest_input[topic][1])
                    if topic in self._latest_input
                    else None,
                }
                for topic in INPUT_TOPICS
            },
            "last_published": {
                topic: {
                    "receipt": self._latest_published[topic][0]
                    if topic in self._latest_published
                    else 0,
                    "polygon": self._polygon_snapshot(self._latest_published[topic][1])
                    if topic in self._latest_published
                    else None,
                }
                for topic in PUBLISHED_TOPICS
            },
            "last_status": self._latest_status[1] if self._latest_status else None,
            "last_safety": self._latest_safety_status[1] if self._latest_safety_status else None,
            "receipt_counters": {
                "input": self._input_receipts,
                "published": self._published_receipts,
                "status": self._status_receipts,
                "safety": self._safety_status_receipts,
            },
            "footprint_padding_m": self._padding_m,
            "declared_footprint_padding_m": self._declared_padding_m,
            "live_footprint_padding_m": self._padding_m,
            "footprint_padding_quantization_bound_m": self._padding_quantization_bound_m,
            "profile_base_frame": self._profile_base_frame,
            "profile_to_robot_base_planar_equivalence": self._profile_to_robot_base_evidence,
            "robot_base_frame": self._robot_base_frame,
            "point32_quantization_bound_m": self._last_quantization_bound_m,
            "fresh_readback_required_per_override": True,
        }

    def _wait_for_override(self, profile: str, nonce: str) -> None:
        """Require messages received after this command and bound to its nonce."""
        baseline_status = self._status_receipts
        baseline_input = dict(self._input_receipts)
        baseline_published = dict(self._published_receipts)
        baseline_published_stamp: dict[str, int] = {}
        for topic in PUBLISHED_TOPICS:
            entry = self._latest_published.get(topic)
            baseline_published_stamp[topic] = self._stamp_ns(entry[1]) if entry else 0
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
            if self._fresh_frame_aware_readback(
                profile, baseline_input, baseline_published, baseline_published_stamp
            ):
                return
        raise RuntimeError(
            "timed out waiting for fresh exact Nav2 published_footprint readback "
            f"for runtime-test override {profile}: {self._last_failure_reason}"
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
        self._read_footprint_padding()

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
            "passed": True,
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
            "raw_input_exact_per_override": True,
            "published_frame_by_topic": PUBLISHED_FRAME_BY_TOPIC,
            "footprint_padding_m": self._padding_m,
            "declared_footprint_padding_m": self._declared_padding_m,
            "live_footprint_padding_m": self._padding_m,
            "footprint_padding_quantization_bound_m": self._padding_quantization_bound_m,
            "profile_base_frame": self._profile_base_frame,
            "profile_to_robot_base_planar_equivalence": self._profile_to_robot_base_evidence,
            "robot_base_frame": self._robot_base_frame,
            "point32_quantization_bound_m": self._last_quantization_bound_m,
            "published_readback_contract": "ordered_frame_aware_padded_rigid_point32",
            "published_stamp_max_age_sec": PUBLISHED_STAMP_MAX_AGE_NS * 1e-9,
        }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-profile-file", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def _blocked_without_node(reason: str) -> dict[str, object]:
    """Preserve a complete, explicitly unavailable diagnostic before ROS starts."""
    return blocked_runtime_gate_shape(reason, INPUT_TOPICS, PUBLISHED_TOPICS)


def main() -> None:
    args = _parse_args()
    node: DynamicFootprintRuntimeGate | None = None
    initialized = False
    error: BaseException | None = None
    result: dict[str, object] | None = None
    try:
        if args.timeout_sec <= 0.0:
            raise ValueError("timeout-sec must be positive")
        if not args.motion_profile_file.is_file():
            raise ValueError("motion-profile-file must identify the frozen formal profile")
        rclpy.init()
        initialized = True
        node = DynamicFootprintRuntimeGate(args.motion_profile_file, args.timeout_sec)
        result = node.run()
    except BaseException as caught:
        error = caught
        result = (
            node.blocked_result(f"{type(caught).__name__}: {caught}")
            if node is not None
            else _blocked_without_node(f"{type(caught).__name__}: {caught}")
        )
    if node is not None:
        try:
            node.destroy_node()
        except BaseException as caught:
            if error is None:
                error = caught
            result = node.blocked_result(
                f"{type(error).__name__}: {error}; node_destroy:{type(caught).__name__}: {caught}"
            )
    if initialized:
        try:
            rclpy.shutdown()
        except BaseException as caught:
            if error is None:
                error = caught
            if node is not None:
                result = node.blocked_result(
                    f"{type(error).__name__}: {error}; rclpy_shutdown:{type(caught).__name__}: {caught}"
                )
            else:
                result = _blocked_without_node(
                    f"{type(error).__name__}: {error}; rclpy_shutdown:{type(caught).__name__}: {caught}"
                )
    assert result is not None
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    atomic_write_fresh_json(args.output, result)
    sys.stdout.write(encoded)
    if error is not None:
        raise error


if __name__ == "__main__":  # pragma: no cover - console script entry point
    main()
