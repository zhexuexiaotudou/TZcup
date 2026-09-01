#!/usr/bin/env python3
"""Collect fresh, fail-closed sensor and controller evidence for the formal vehicle."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import rclpy
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, NavSatFix, PointCloud2
from std_msgs.msg import Int64MultiArray, String

from formal_vehicle_sensor_runtime_contract import (
    STREAM_CONTRACTS,
    observed_frequency_hz,
    validate_runtime_contract,
)
from formal_runtime_gate_binding import load_binding
from formal_preembedded_sensor_world_binding import validate_preembedded_sensor_world


ROOT = Path(__file__).resolve().parents[1]
EXPANDED_URDF = ROOT / "reports/engineering/formal_competition_vehicle.urdf"
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
DEFAULT_FOV_REPORT = ROOT / "reports/engineering/formal_vehicle_fov_occlusion_report.json"
ACTIVE_CONTROLLERS = {
    "joint_state_broadcaster",
    "arm_controller",
    "gripper_controller",
    "cleaning_controller",
    "storage_controller",
    "service_controller",
}
INACTIVE_CONTROLLERS = {"brush_controller", "recovery_controller"}
TOPICS: dict[str, type] = {
    "/sensors/lidar_2d/scan": LaserScan,
    "/sensors/lidar_3d/points": PointCloud2,
    "/sensors/front_rgbd/depth/image_rect_raw/image": Image,
    "/sensors/front_rgbd/depth/image_rect_raw/depth_image": Image,
    "/sensors/front_rgbd/depth/image_rect_raw/camera_info": CameraInfo,
    "/sensors/front_rgbd/infra1/image_rect_raw": Image,
    "/sensors/front_rgbd/infra1/image_rect_raw/camera_info": CameraInfo,
    "/sensors/front_rgbd/infra2/image_rect_raw": Image,
    "/sensors/front_rgbd/infra2/image_rect_raw/camera_info": CameraInfo,
    "/sensors/wrist_rgbd/depth/image_rect_raw/image": Image,
    "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image": Image,
    "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info": CameraInfo,
    "/sensors/wrist_rgbd/infra1/image_rect_raw": Image,
    "/sensors/wrist_rgbd/infra1/image_rect_raw/camera_info": CameraInfo,
    "/sensors/wrist_rgbd/infra2/image_rect_raw": Image,
    "/sensors/wrist_rgbd/infra2/image_rect_raw/camera_info": CameraInfo,
    "/sensors/rear_left_fisheye/image_raw": Image,
    "/sensors/rear_left_fisheye/camera_info": CameraInfo,
    "/sensors/rear_right_fisheye/image_raw": Image,
    "/sensors/rear_right_fisheye/camera_info": CameraInfo,
    "/sensors/gnss/fix": NavSatFix,
    "/sensors/imu/data": Imu,
    "/formal_vehicle/encoders/a300/counts": Int64MultiArray,
    "/formal_vehicle/encoders/a300/joint_states": JointState,
    "/odom/unfiltered": Odometry,
}

RELIABLE_FRAGMENTED_TOPICS = frozenset(
    {
        "/sensors/lidar_3d/points",
        "/sensors/rear_left_fisheye/image_raw",
        "/sensors/rear_right_fisheye/image_raw",
    }
)
if set(TOPICS) != set(STREAM_CONTRACTS):
    raise RuntimeError("runtime ROS types and dependency-free sensor contracts differ")
CAMERA_INFO_TOPICS = {
    topic for topic, message_type in TOPICS.items() if message_type is CameraInfo
}
FISHEYE_CAMERA_INFO_TOPICS = {
    "/sensors/rear_left_fisheye/camera_info",
    "/sensors/rear_right_fisheye/camera_info",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json_object(path: Path, label: str) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} root must be an object")
    return value


def _snapshot_identity(snapshot_path: Path) -> dict[str, str]:
    snapshot = _json_object(snapshot_path, "snapshot")
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") \
        if isinstance(outputs, dict) else None
    source_hash = snapshot.get("source_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not isinstance(source_hash, str) or not isinstance(urdf_hash, str):
        raise ValueError("snapshot is missing source or expanded-URDF identity")
    return {
        "snapshot_manifest_sha256": _sha256(snapshot_path),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _acceptance_binding(
    session_path: Path, snapshot_path: Path, fov_report_path: Path
) -> dict[str, Any]:
    identity = _snapshot_identity(snapshot_path)
    session = _json_object(session_path, "formal acceptance session")
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    if session.get("snapshot") != identity:
        raise ValueError("formal acceptance session snapshot is not current")
    started_ns = session.get("started_epoch_ns")
    if not isinstance(started_ns, int) or started_ns <= 0 or started_ns > time.time_ns():
        raise ValueError("formal acceptance session start time is invalid")
    if fov_report_path.stat().st_mtime_ns < started_ns:
        raise ValueError("FOV/occlusion report predates the formal acceptance session")
    fov = _json_object(fov_report_path, "FOV/occlusion report")
    if fov.get("status") != "PASSED" or fov.get(
        "all_minimum_clear_fractions_passed"
    ) is not True:
        raise ValueError("FOV/occlusion gate is not passing")
    if fov.get("urdf_sha256") != identity["expanded_urdf_sha256"]:
        raise ValueError("FOV/occlusion report is for another expanded URDF")
    return {
        "session_manifest_sha256": _sha256(session_path),
        "session_manifest_path": str(session_path.resolve()),
        "session_started_epoch_ns": started_ns,
        "status_at_gate_start": session["status"],
        "snapshot": identity,
        "fov_report_path": str(fov_report_path.relative_to(ROOT)),
        "fov_report_sha256": _sha256(fov_report_path),
    }


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_vehicle_sensor_runtime_probe")
        self.samples = {topic: 0 for topic in TOPICS}
        self.metadata: dict[str, dict[str, Any]] = {}
        self.source_stamps_ns: dict[str, list[int]] = {topic: [] for topic in TOPICS}
        self.robot_description: str | None = None
        # High-bandwidth subscriptions are intentionally created only after the
        # controller plane is ready.  KEEP_LAST(1) is sufficient because this
        # gate records metadata and source timestamps, not image payloads.
        # With the launch-side lazy bridge, destroying a completed subscription
        # also stops further GZ -> ROS conversion for that stream.
        self._sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # These three samples are 3.7--6.2 MB each.  BEST_EFFORT loses DDS
        # fragments on the otherwise isolated loopback transport, which makes
        # source timestamps appear sparse even though Gazebo publishes at the
        # configured rate.  Reliable KEEP_LAST(1) preserves the newest complete
        # sample without allowing an unbounded queue.
        self._fragmented_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._topic_subscriptions: dict[str, Any] = {}
        description_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._description_subscription = self.create_subscription(
            String,
            "/robot_description",
            self._description,
            description_qos,
        )
        self.controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

    def start_topic_subscriptions(self) -> None:
        if self._topic_subscriptions:
            raise RuntimeError("sensor subscriptions already started")
        for topic, message_type in TOPICS.items():
            self._topic_subscriptions[topic] = self.create_subscription(
                message_type,
                topic,
                lambda message, name=topic: self._sample(name, message),
                (
                    self._fragmented_sensor_qos
                    if topic in RELIABLE_FRAGMENTED_TOPICS
                    else self._sensor_qos
                ),
            )

    def _topic_ready(self, topic: str) -> bool:
        if self.samples[topic] < 3 or topic not in self.metadata:
            return False
        contract = STREAM_CONTRACTS[topic]
        return "nominal_hz" not in contract or observed_frequency_hz(
            self.source_stamps_ns[topic]
        ) is not None

    def retire_ready_subscriptions(self) -> tuple[str, ...]:
        retired: list[str] = []
        for topic, subscription in tuple(self._topic_subscriptions.items()):
            if not self._topic_ready(topic):
                continue
            self.destroy_subscription(subscription)
            del self._topic_subscriptions[topic]
            retired.append(topic)
        return tuple(retired)

    def pending_topics(self) -> tuple[str, ...]:
        return tuple(sorted(self._topic_subscriptions))

    def _description(self, message: String) -> None:
        self.robot_description = message.data

    def _sample(self, topic: str, message: Any) -> None:
        self.samples[topic] += 1
        header = getattr(message, "header", None)
        row: dict[str, Any] = {
            "frame_id": getattr(header, "frame_id", ""),
        }
        stamp = getattr(header, "stamp", None)
        stamp_ns = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if stamp is not None else 0
        )
        if stamp_ns > 0:
            stamps = self.source_stamps_ns[topic]
            if not stamps or stamps[-1] != stamp_ns:
                stamps.append(stamp_ns)
                if len(stamps) > 512:
                    del stamps[:-512]
        if isinstance(message, Image):
            row.update(width=message.width, height=message.height, encoding=message.encoding)
        elif isinstance(message, CameraInfo):
            row.update(
                width=message.width,
                height=message.height,
                distortion_model=message.distortion_model,
                fx=message.k[0],
                fy=message.k[4],
                cx=message.k[2],
                cy=message.k[5],
                d=list(message.d),
            )
        elif isinstance(message, PointCloud2):
            row.update(width=message.width, height=message.height, point_step=message.point_step)
        elif isinstance(message, LaserScan):
            row.update(
                range_count=len(message.ranges),
                range_min_m=message.range_min,
                range_max_m=message.range_max,
                angle_min_rad=message.angle_min,
                angle_max_rad=message.angle_max,
                angle_increment_rad=message.angle_increment,
            )
        elif isinstance(message, NavSatFix):
            row.update(
                latitude=message.latitude,
                longitude=message.longitude,
                altitude=message.altitude,
            )
        elif isinstance(message, Imu):
            values = (
                message.orientation.x, message.orientation.y,
                message.orientation.z, message.orientation.w,
                message.angular_velocity.x, message.angular_velocity.y,
                message.angular_velocity.z, message.linear_acceleration.x,
                message.linear_acceleration.y, message.linear_acceleration.z,
            )
            row.update(finite_measurement=all(math.isfinite(value) for value in values))
        elif isinstance(message, Int64MultiArray):
            row.update(
                layout_label=(message.layout.dim[0].label if message.layout.dim else ""),
                data_length=len(message.data),
                counts=list(message.data),
            )
        elif isinstance(message, JointState):
            row.update(
                joint_names=list(message.name),
                position=list(message.position),
                velocity=list(message.velocity),
                finite_position_velocity=all(
                    math.isfinite(value)
                    for value in (*message.position, *message.velocity)
                ),
            )
        elif isinstance(message, Odometry):
            row.update(child_frame_id=message.child_frame_id)
        self.metadata[topic] = row

    def observed_hz(self) -> dict[str, float | None]:
        return {
            topic: observed_frequency_hz(stamps)
            for topic, stamps in self.source_stamps_ns.items()
        }

    def controller_states(self) -> dict[str, str]:
        if not self.controllers.wait_for_service(timeout_sec=0.05):
            return {}
        future = self.controllers.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done():
            self.controllers.remove_pending_request(future)
            return {}
        response = future.result()
        if response is None:
            return {}
        return {item.name: item.state for item in response.controller}


def collect(
    output: Path,
    timeout_s: float,
    *,
    session_path: Path = DEFAULT_SESSION,
    snapshot_path: Path = DEFAULT_SNAPSHOT,
    fov_report_path: Path = DEFAULT_FOV_REPORT,
    runtime_binding_path: Path | None = None,
    preembedded_report_path: Path | None = None,
    preembedded_world_path: Path | None = None,
    preembedded_model_pose: str = "0 0 0.005 0 0 0",
) -> dict[str, Any]:
    binding = _acceptance_binding(session_path, snapshot_path, fov_report_path)
    if runtime_binding_path is None:
        raise RuntimeError("formal sensor collector requires a frozen runtime binding")
    runtime_binding = load_binding(runtime_binding_path)
    if runtime_binding["acceptance_session_binding"]["snapshot"] != binding["snapshot"]:
        raise RuntimeError("runtime gate and sensor collector snapshot bindings differ")
    runtime_install_root = Path(
        runtime_binding["runtime_closure_binding"]["runtime_install_root"]
    )
    if preembedded_report_path is None or preembedded_world_path is None:
        raise RuntimeError("formal sensor collector requires a preembedded world binding")
    preembedded_binding = validate_preembedded_sensor_world(
        report_path=preembedded_report_path,
        world_path=preembedded_world_path,
        expanded_urdf_path=EXPANDED_URDF,
        acceptance_session={
            "started_epoch_ns": binding["session_started_epoch_ns"],
            "session_manifest_sha256": binding["session_manifest_sha256"],
        },
        snapshot_identity=binding["snapshot"],
        expected_model_pose=preembedded_model_pose,
        expected_runtime_install_root=runtime_install_root,
    )
    rclpy.init()
    node = Probe()
    states: dict[str, str] = {}
    deadline = time.monotonic() + timeout_s
    controller_plane_ready = False
    try:
        # Do not expose DDS to the raw image / cloud streams while controller
        # startup is still in progress.  This also prevents service responses
        # from being starved by camera callbacks.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
            states = node.controller_states()
            controllers_ready = all(
                states.get(name) == "active" for name in ACTIVE_CONTROLLERS
            ) and all(states.get(name) == "inactive" for name in INACTIVE_CONTROLLERS)
            if controllers_ready and node.robot_description:
                controller_plane_ready = True
                break
        if controller_plane_ready:
            node.start_topic_subscriptions()
            next_progress = time.monotonic()
            while time.monotonic() < deadline and node.pending_topics():
                rclpy.spin_once(node, timeout_sec=0.05)
                retired = node.retire_ready_subscriptions()
                now = time.monotonic()
                if retired or now >= next_progress:
                    print(
                        "sensor_runtime_progress "
                        f"retired={len(retired)} pending={len(node.pending_topics())} "
                        f"pending_topics={','.join(node.pending_topics())}",
                        flush=True,
                    )
                    next_progress = now + 5.0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    description = node.robot_description or ""
    observed_hz = node.observed_hz()
    try:
        description_root = ET.fromstring(description)
        live_wastewater_capacity_values = [
            float(element.text)
            for element in description_root.findall(".//wastewater_capacity_kg")
            if element.text is not None
        ]
    except (ET.ParseError, TypeError, ValueError):
        live_wastewater_capacity_values = []
    expanded_hash = hashlib.sha256(EXPANDED_URDF.read_bytes()).hexdigest()
    camera_info_valid = all(
        node.metadata.get(topic, {}).get("width", 0) > 0
        and node.metadata.get(topic, {}).get("height", 0) > 0
        and node.metadata.get(topic, {}).get("fx", 0.0) > 0.0
        and node.metadata.get(topic, {}).get("fy", 0.0) > 0.0
        for topic in CAMERA_INFO_TOPICS
    )
    expected_fisheye_focal_px = 960.0 / (2.0 * math.sin(2.617994 / 4.0))
    expected_fisheye_distortion = [
        -1.0 / 24.0,
        1.0 / 1920.0,
        -1.0 / 322560.0,
        1.0 / 92897280.0,
    ]
    fisheye_camera_info_valid = all(
        node.metadata.get(topic, {}).get("distortion_model") == "equidistant"
        and node.metadata.get(topic, {}).get("width") == 1920
        and node.metadata.get(topic, {}).get("height") == 1080
        and abs(node.metadata.get(topic, {}).get("fx", 0.0) - expected_fisheye_focal_px)
        < 1.0e-3
        and node.metadata.get(topic, {}).get("cx") == 960.0
        and node.metadata.get(topic, {}).get("cy") == 540.0
        and len(node.metadata.get(topic, {}).get("d", [])) == 4
        and all(
            abs(observed - expected) < 1.0e-12
            for observed, expected in zip(
                node.metadata.get(topic, {}).get("d", []),
                expected_fisheye_distortion,
            )
        )
        for topic in FISHEYE_CAMERA_INFO_TOPICS
    )
    runtime_contract = validate_runtime_contract(
        node.samples, node.metadata, observed_hz
    )
    passed_checks: dict[str, bool | float] = {
        **runtime_contract["passed_checks"],
        "all_camera_info_intrinsics_valid": camera_info_valid,
        "fisheye_camera_info_matches_nominal_equisolid_gazebo_projection": (
            fisheye_camera_info_valid
        ),
        "robot_description_observed": bool(description),
        "a300_plant_loaded_exactly_once": description.count(
            "libA300DrivetrainPlantSystem.so"
        ) == 1,
        "wastewater_capacity_is_8_30_kg": live_wastewater_capacity_values == [8.30],
        "required_controllers_safe": all(
            states.get(name) == "active" for name in ACTIVE_CONTROLLERS
        ) and all(states.get(name) == "inactive" for name in INACTIVE_CONTROLLERS),
        "removed_base_controller_absent": "base_controller" not in states,
        "fov_occlusion_report_passed_and_bound_to_current_urdf": True,
        "formal_acceptance_session_running_and_snapshot_bound": True,
        "preembedded_sensor_world_bound_to_session_snapshot_and_source_urdf": True,
        "controller_plane_ready_before_sensor_subscriptions": controller_plane_ready,
        "all_sensor_subscriptions_retired_after_bounded_evidence": not node.pending_topics(),
    }
    passed = all(bool(value) for value in passed_checks.values())
    report = {
        "report_id": "tzcup_formal_vehicle_headless_runtime_v5",
        "status": (
            "FORMAL_GAZEBO_CONTROL_AND_SENSOR_RUNTIME_PASSED_EXTERNAL_FIDELITY_GATES_PENDING"
            if passed
            else "FORMAL_GAZEBO_CONTROL_AND_SENSOR_RUNTIME_BLOCKED"
        ),
        "passed": passed,
        "passed_checks": {
            **passed_checks,
            "dry_payload_clamp_kg": 1.512,
            "wastewater_payload_clamp_kg": 8.30,
        },
        "sample_counts": node.samples,
        "observed_source_timestamp_hz": observed_hz,
        "observed_interfaces": node.metadata,
        "runtime_sensor_contract": runtime_contract,
        "controller_states": states,
        "session_bound": True,
        # Retain the complete immutable sidecar, rather than only its two
        # convenience projections below.  Final acceptance compares this
        # value byte-for-value with the runner-produced binding.
        "runtime_gate_binding": runtime_binding,
        "acceptance_session_binding": binding,
        "runtime_closure_binding": (
            runtime_binding["runtime_closure_binding"]
        ),
        "preembedded_sensor_world_binding": preembedded_binding,
        "current_source_reconciliation": {
            "expanded_urdf_sha256": expanded_hash,
            "expanded_urdf_path": str(EXPANDED_URDF.relative_to(ROOT)),
            "live_robot_description_sha256": hashlib.sha256(
                description.encode("utf-8")
            ).hexdigest() if description else None,
            "wastewater_capacity_kg": 8.30,
            "live_wastewater_capacity_values_kg": live_wastewater_capacity_values,
            "drivetrain": "A300DrivetrainPlantSystem",
            "odometry_topic": "/odom/unfiltered",
        },
        "claim_boundary": (
            "Fresh session-bound headless Gazebo evidence for exact runtime sensor "
            "frames, source-timestamp cadence, UTM scan limits, nonempty MID360 "
            "points, camera intrinsics, finite GNSS/IMU, A300 four-wheel encoder "
            "feedback, safe controller states and raw plant odometry. The bound "
            "deterministic mesh-ray report proves configured FOV and mount occlusion; "
            "rendered target visibility, physical lens calibration, GNSS multipath, "
            "motion, grasp, cleaning contact and hardware deployment remain separate."
        ),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(output)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--fov-report", type=Path, default=DEFAULT_FOV_REPORT)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--preembedded-report", type=Path, required=True)
    parser.add_argument("--preembedded-world", type=Path, required=True)
    parser.add_argument("--preembedded-model-pose", default="0 0 0.005 0 0 0")
    args = parser.parse_args()
    report = collect(
        args.output,
        args.timeout,
        session_path=args.session,
        snapshot_path=args.snapshot,
        fov_report_path=args.fov_report,
        runtime_binding_path=args.runtime_binding,
        preembedded_report_path=args.preembedded_report,
        preembedded_world_path=args.preembedded_world,
        preembedded_model_pose=args.preembedded_model_pose,
    )
    print(json.dumps(report, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
