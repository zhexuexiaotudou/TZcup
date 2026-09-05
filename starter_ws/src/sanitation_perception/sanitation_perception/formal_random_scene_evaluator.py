"""Live Gazebo-camera evaluator for the formal DOSOD + EdgeSAM product graph.

Generated truth is loaded only in this process.  It is never published and is
used solely for scoring and for an evaluator-side SetEntityPose camera tour.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

from .formal_random_scene_evaluator_core import (
    BoxObservation,
    TruthBox,
    finalize_acceptance,
    load_evaluator_truth,
    match_boxes,
    projection_error_metrics,
    rasterize_dirt_truth,
    segmentation_metrics,
)


RGB_TOPICS = {
    "front": "/sensors/front_rgbd/depth/image_rect_raw/image",
    "wrist": "/sensors/wrist_rgbd/depth/image_rect_raw/image",
    "rear_left": "/sensors/rear_left_fisheye/image_raw",
    "rear_right": "/sensors/rear_right_fisheye/image_raw",
}
DEPTH_TOPICS = {
    "front": "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
    "wrist": "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image",
}
INFO_TOPICS = {
    "front": "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
    "wrist": "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info",
    "rear_left": "/sensors/rear_left_fisheye/camera_info",
    "rear_right": "/sensors/rear_right_fisheye/camera_info",
}


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _transform_matrix(transform) -> np.ndarray:
    translation, rotation = transform.translation, transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    norm = x * x + y * y + z * z + w * w
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid TF quaternion")
    scale = 2.0 / norm
    return np.asarray(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w), translation.x],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w), translation.y],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y), translation.z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _pose2d_error(
    observed: tuple[float, float, float],
    expected: tuple[float, float, float],
) -> tuple[float, float]:
    """Return planar position and wrapped-yaw errors for one named frame."""

    position_error = math.hypot(observed[0] - expected[0], observed[1] - expected[1])
    yaw_delta = observed[2] - expected[2]
    yaw_error = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
    return position_error, yaw_error


def _camera_from_map_at_staging(
    map_from_base: np.ndarray, camera_from_base: np.ndarray
) -> np.ndarray:
    """Freeze evaluator-only camera geometry in the staged scene frame.

    The formal vehicle is held stationary while the evaluator moves only the
    randomized objects.  Those private truth poses are written relative to the
    staging-time map pose, so their 2D image projection must use the matching
    staging-time camera pose.  A later local-odometry drift must remain visible
    to the product map-projection/TF gates, but must not corrupt the independent
    DOSOD pixel-box score.
    """

    map_from_base = np.asarray(map_from_base, dtype=np.float64)
    camera_from_base = np.asarray(camera_from_base, dtype=np.float64)
    if map_from_base.shape != (4, 4) or camera_from_base.shape != (4, 4):
        raise ValueError("staging transforms must be 4x4 matrices")
    if not np.isfinite(map_from_base).all() or not np.isfinite(camera_from_base).all():
        raise ValueError("staging transforms must be finite")
    camera_from_map = camera_from_base @ np.linalg.inv(map_from_base)
    if not np.isfinite(camera_from_map).all():
        raise ValueError("staging camera transform is non-finite")
    return camera_from_map


def _lookup_pose_pair_with_retry(
    lookup,
    spin_once,
    *,
    timeout_s: float,
    monotonic=time.monotonic,
):
    """Return one successful map/odom pose pair within a bounded window.

    The map->odom localization edge and odom->base controller edge can enter a
    fresh tf2 buffer a few samples apart.  A single latest-time lookup can then
    fail with an early-history extrapolation even though the complete chain is
    healthy on the next sample.  Retrying both lookups as one pair avoids that
    startup race without accepting stale transforms or weakening any product
    input/accuracy gate.
    """

    timeout_s = float(timeout_s)
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("pose-pair lookup timeout must be finite and positive")
    deadline = monotonic() + timeout_s
    last_error: Exception | None = None
    while monotonic() < deadline:
        try:
            return lookup("map"), lookup("odom")
        except Exception as exc:  # tf2 exception classes are runtime imports.
            last_error = exc
            spin_once()
    detail = str(last_error) if last_error is not None else "no lookup attempt completed"
    raise RuntimeError(f"latest map/odom pose pair unavailable within {timeout_s:.3f}s: {detail}")


def _project_cube(cube: dict, camera_from_map: np.ndarray, info) -> tuple[TruthBox, float] | None:
    pose = cube["pose"]
    cx, cy = float(pose["x_m"]), float(pose["y_m"])
    edge = float(cube["edge_m"])
    half = edge / 2.0
    center_z = float(pose.get("z_m", half))
    corners = np.asarray(
        [
            [cx + dx, cy + dy, center_z + dz, 1.0]
            for dx in (-half, half)
            for dy in (-half, half)
            for dz in (-half, half)
        ],
        dtype=np.float64,
    ).T
    points = camera_from_map @ corners
    if np.any(points[2] <= 0.15):
        return None
    fx, fy, cx_px, cy_px = float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
    us = fx * points[0] / points[2] + cx_px
    vs = fy * points[1] / points[2] + cy_px
    x1, y1 = max(0.0, float(us.min())), max(0.0, float(vs.min()))
    x2, y2 = min(float(info.width), float(us.max())), min(float(info.height), float(vs.max()))
    if x2 <= x1 or y2 <= y1 or (x2 - x1) < 4.0 or (y2 - y1) < 4.0:
        return None
    center = camera_from_map @ np.asarray([cx, cy, center_z, 1.0])
    return TruthBox(str(cube["object_id"]), "litter_cube", (x1, y1, x2, y2)), float(center[2])


def main() -> None:
    import rclpy
    from cv_bridge import CvBridge
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Pose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from ros_gz_interfaces.srv import SetEntityPose
    from sanitation_perception_interfaces.msg import GarbageTargetArray
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformException, TransformListener
    from vision_msgs.msg import Detection2DArray

    class FormalRandomSceneEvaluator(Node):
        def __init__(self) -> None:
            super().__init__("formal_random_scene_perception_evaluator")
            self.declare_parameter("truth_path", "")
            self.declare_parameter("public_manifest_path", "")
            self.declare_parameter("acceptance_config", "")
            self.declare_parameter("output_path", "")
            self.declare_parameter("diagnostic_frame_path", "")
            self.declare_parameter("world_name", "campus_formal")
            self.declare_parameter("entity_name", "tzcup_formal_sanitation_vehicle")
            self.declare_parameter("startup_timeout_s", 90.0)
            truth_path = str(self.get_parameter("truth_path").value)
            public_manifest_path = Path(
                str(self.get_parameter("public_manifest_path").value)
            )
            config_path = Path(str(self.get_parameter("acceptance_config").value))
            output_path = str(self.get_parameter("output_path").value)
            if (
                not truth_path
                or not public_manifest_path.is_file()
                or not config_path.is_file()
                or not output_path
            ):
                raise RuntimeError(
                    "truth_path, public_manifest_path, acceptance_config and output_path are required"
                )
            self.truth = load_evaluator_truth(truth_path)
            public_manifest = json.loads(
                public_manifest_path.read_text(encoding="utf-8")
            )
            if any(
                forbidden in public_manifest
                for forbidden in ("discrete_cubes", "dirt_patches", "pedestrians")
            ):
                raise RuntimeError("public manifest unexpectedly exposes evaluator entities")
            legacy_source_start = public_manifest.get("vehicle_start_pose_map", {})
            public_start = public_manifest.get("vehicle_start_pose_source_world", {})
            localization_start = public_manifest.get(
                "vehicle_start_pose_localization_map", {}
            )
            try:
                self.public_start_pose = (
                    float(public_start["x_m"]),
                    float(public_start["y_m"]),
                    float(public_start["yaw_rad"]),
                )
                legacy_source_pose = (
                    float(legacy_source_start["x_m"]),
                    float(legacy_source_start["y_m"]),
                    float(legacy_source_start["yaw_rad"]),
                )
                self.localization_start_pose = (
                    float(localization_start["x_m"]),
                    float(localization_start["y_m"]),
                    float(localization_start["yaw_rad"]),
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(
                    "public manifest has no valid source-world/localization fixed start poses"
                ) from exc
            source_position_error, source_yaw_error = _pose2d_error(
                self.public_start_pose, legacy_source_pose
            )
            if source_position_error > 1e-9 or source_yaw_error > 1e-9:
                raise RuntimeError(
                    "public source-world fixed start conflicts with legacy source pose"
                )
            self.source_world_start_contract_error_m = source_position_error
            self.source_world_start_contract_yaw_error_rad = source_yaw_error
            self.config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
            if self.config.get("schema_version") != 1:
                raise RuntimeError("acceptance config schema_version must equal 1")
            boundary = self.config.get("truth_boundary", {})
            if boundary.get("publish_truth_to_ros") is not False or boundary.get("product_truth_input_allowed") is not False:
                raise RuntimeError("acceptance config weakens evaluator truth isolation")
            self.output_path = Path(output_path)
            self.bridge = CvBridge()
            self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.map_message = None
            self.info_by_frame: dict[str, CameraInfo] = {}
            self.depth_by_sensor: dict[str, tuple[float, np.ndarray]] = {}
            self.last_rgb_stamp: dict[str, float] = {}
            self.rgb_topics_seen: set[str] = set()
            self.depth_topics_seen: set[str] = set()
            self.info_topics_seen: set[str] = set()
            self.real_camera_message_count = 0
            self.depth_rgb_skews: list[float] = []
            self.tf_attempts = 0
            self.tf_successes = 0
            self.tf_ages: list[float] = []
            self.diagnostic_truth_flags: list[bool] = []
            self.diagnostic_fail_closed_count = 0
            self.product_detection_message_count = 0
            self.product_mask_message_count = 0
            self.product_target_message_count = 0
            self.evaluated_frame_count = 0
            self.true_positive_count = 0
            self.false_positive_count = 0
            self.visible_truth_ids: set[str] = set()
            self.matched_truth_ids: set[str] = set()
            # Runtime-topic freshness is observed from process start, but
            # accuracy scoring begins only after every randomized entity has
            # been staged and the configured settling interval has elapsed.
            # Otherwise pre-stage or partially staged frames can poison the
            # detection denominator and diagnostic frame selection.
            self.acceptance_sampling_active = False
            self.acceptance_sampling_start_s: float | None = None
            # A product that never publishes Detection2DArray must still be
            # scored as an empty prediction, rather than silently producing
            # zero evaluated frames.  Cache evaluator-only projected truth for
            # each real front-camera frame and match product output by stamp.
            self.front_truth_by_stamp: dict[int, list[TruthBox]] = {}
            self.matched_detection_stamps: set[int] = set()
            self.best_front_diagnostic: tuple[float, np.ndarray, dict] | None = None
            self.staged_front_camera_from_map: np.ndarray | None = None
            self.accumulated_dirty = None
            self.accumulated_observed = None
            self.target_errors_by_uuid: dict[str, float] = {}
            self.target_errors_by_truth: dict[str, float] = {}
            self.target_false_uuid: set[str] = set()
            self.localization_start_error_m: float | None = None
            self.localization_start_yaw_error_rad: float | None = None
            self.localized_map_pose: tuple[float, float, float] | None = None
            self.pose_client = self.create_client(
                SetEntityPose,
                f"/world/{self.get_parameter('world_name').value}/set_pose",
            )

            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            diagnostic_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
            for sensor, topic in RGB_TOPICS.items():
                self.create_subscription(Image, topic, lambda msg, s=sensor, t=topic: self._on_rgb(s, t, msg), qos_profile_sensor_data)
            for sensor, topic in DEPTH_TOPICS.items():
                self.create_subscription(Image, topic, lambda msg, s=sensor, t=topic: self._on_depth(s, t, msg), qos_profile_sensor_data)
            for sensor, topic in INFO_TOPICS.items():
                self.create_subscription(CameraInfo, topic, lambda msg, t=topic: self._on_info(t, msg), qos_profile_sensor_data)
            self.create_subscription(Detection2DArray, "/perception/garbage/detections_2d", self._on_detections, 10)
            self.create_subscription(Image, "/perception/ground_dirt/masks", self._on_mask, 10)
            self.create_subscription(GarbageTargetArray, "/perception/garbage/targets", self._on_targets, 10)
            self.create_subscription(
                DiagnosticArray,
                "/perception/open_vocab/diagnostics",
                self._on_diagnostics,
                diagnostic_qos,
            )

        def _on_map(self, message) -> None:
            self.map_message = message

        def _on_rgb(self, sensor: str, topic: str, message) -> None:
            self.rgb_topics_seen.add(topic)
            self.real_camera_message_count += 1
            stamp = _stamp_seconds(message.header.stamp)
            self.last_rgb_stamp[sensor] = stamp
            if sensor in self.depth_by_sensor:
                self.depth_rgb_skews.append(abs(stamp - self.depth_by_sensor[sensor][0]))
            self.tf_attempts += 1
            try:
                transform = self.tf_buffer.lookup_transform("map", message.header.frame_id, Time(), timeout=Duration(seconds=0.02))
                self.tf_successes += 1
                transform_stamp = _stamp_seconds(transform.header.stamp)
                if transform_stamp > 0.0:
                    self.tf_ages.append(abs(stamp - transform_stamp))
            except TransformException:
                pass
            if (
                sensor == "front"
                and self.acceptance_sampling_active
                and self._stamp_in_acceptance_window(message.header.stamp)
            ):
                truth_boxes = self._truth_boxes(message.header.frame_id, message.header.stamp)
                if truth_boxes:
                    stamp_key = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                    if stamp_key not in self.front_truth_by_stamp:
                        self.front_truth_by_stamp[stamp_key] = truth_boxes
                        self.evaluated_frame_count += 1
                        self.visible_truth_ids.update(item.object_id for item in truth_boxes)
                    largest_area = max(
                        (box.xyxy[2] - box.xyxy[0]) * (box.xyxy[3] - box.xyxy[1])
                        for box in truth_boxes
                    )
                    if self.best_front_diagnostic is None or largest_area > self.best_front_diagnostic[0]:
                        try:
                            rgb = np.asarray(
                                self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8"), dtype=np.uint8
                            ).copy()
                            metadata = {
                                "source_topic": RGB_TOPICS["front"],
                                "source_frame_id": message.header.frame_id,
                                "source_stamp": {
                                    "sec": int(message.header.stamp.sec),
                                    "nanosec": int(message.header.stamp.nanosec),
                                },
                                "real_gazebo_camera_frame": True,
                                "evaluator_truth_overlay_only": True,
                                "scene_staging_world_pose_source": (
                                    "public_episode_manifest_vehicle_start_pose_source_world"
                                ),
                                "truth_boxes_xyxy": [
                                    {"object_id": box.object_id, "class_id": box.class_id, "xyxy": list(box.xyxy)}
                                    for box in truth_boxes
                                ],
                            }
                            self.best_front_diagnostic = (float(largest_area), rgb, metadata)
                        except Exception:
                            pass

        def _on_depth(self, sensor: str, topic: str, message) -> None:
            self.depth_topics_seen.add(topic)
            try:
                array = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
                self.depth_by_sensor[sensor] = (_stamp_seconds(message.header.stamp), np.asarray(array))
            except Exception:
                return

        def _on_info(self, topic: str, message) -> None:
            self.info_topics_seen.add(topic)
            if message.header.frame_id:
                self.info_by_frame[message.header.frame_id] = message

        def _stamp_in_acceptance_window(self, stamp) -> bool:
            return (
                self.acceptance_sampling_start_s is not None
                and _stamp_seconds(stamp) + 1e-9 >= self.acceptance_sampling_start_s
            )

        def _truth_boxes(self, frame_id: str, stamp) -> list[TruthBox]:
            info = self.info_by_frame.get(frame_id)
            if info is None:
                return []
            if (
                frame_id == "front_rgbd_depth_optical_frame"
                and self.staged_front_camera_from_map is not None
            ):
                matrix = self.staged_front_camera_from_map
            else:
                try:
                    # tf2 returns a transform from ``source`` into ``target``.
                    # Cube truth is expressed in map coordinates, while the
                    # pinhole projection below consumes camera-frame points,
                    # so this must be map -> camera (target=frame_id,
                    # source=map). The front-camera acceptance path freezes
                    # this same relation at scene staging below.
                    transform = self.tf_buffer.lookup_transform(
                        frame_id, "map", Time(), timeout=Duration(seconds=0.10)
                    )
                except TransformException:
                    return []
                matrix = _transform_matrix(transform.transform)
            boxes = []
            for cube in self.truth["discrete_cubes"]:
                projected = _project_cube(cube, matrix, info)
                if projected is not None:
                    boxes.append(projected[0])
            return boxes

        def _on_detections(self, message) -> None:
            if not self.acceptance_sampling_active or not self._stamp_in_acceptance_window(
                message.header.stamp
            ):
                return
            if message.header.frame_id != "front_rgbd_depth_optical_frame":
                return
            self.product_detection_message_count += 1
            stamp_key = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            if stamp_key in self.matched_detection_stamps:
                return
            truth_boxes = self.front_truth_by_stamp.get(stamp_key)
            if truth_boxes is None:
                truth_boxes = self._truth_boxes(message.header.frame_id, message.header.stamp)
            if not truth_boxes:
                return
            predictions = []
            for detection in message.detections:
                if not detection.results:
                    continue
                result = detection.results[0].hypothesis
                if result.class_id != "litter_cube":
                    continue
                center = detection.bbox.center.position
                predictions.append(
                    BoxObservation(
                        "litter_cube",
                        float(result.score),
                        (
                            float(center.x - detection.bbox.size_x / 2.0),
                            float(center.y - detection.bbox.size_y / 2.0),
                            float(center.x + detection.bbox.size_x / 2.0),
                            float(center.y + detection.bbox.size_y / 2.0),
                        ),
                    )
                )
            result = match_boxes(predictions, truth_boxes, iou_threshold=float(self.config["metrics"]["cube_box_iou_match"]))
            self.matched_detection_stamps.add(stamp_key)
            self.true_positive_count += int(result["true_positive_count"])
            self.false_positive_count += int(result["false_positive_count"])
            self.matched_truth_ids.update(item["truth_object_id"] for item in result["matches"])

        def _on_mask(self, message) -> None:
            if not self.acceptance_sampling_active or not self._stamp_in_acceptance_window(
                message.header.stamp
            ):
                return
            if self.map_message is None or message.header.frame_id != "map":
                return
            try:
                raster = np.asarray(self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough"), dtype=np.uint8)
            except Exception:
                return
            expected = (int(self.map_message.info.height), int(self.map_message.info.width))
            if raster.shape != expected:
                return
            self.product_mask_message_count += 1
            dirty, observed = raster >= 2, raster > 0
            if self.accumulated_dirty is None:
                self.accumulated_dirty = dirty.copy()
                self.accumulated_observed = observed.copy()
            else:
                self.accumulated_dirty |= dirty
                self.accumulated_observed |= observed

        def _on_targets(self, message) -> None:
            if not self.acceptance_sampling_active or not self._stamp_in_acceptance_window(
                message.header.stamp
            ):
                return
            self.product_target_message_count += 1
            cubes = self.truth["discrete_cubes"]
            for target in message.targets:
                if target.class_id != "litter_cube" or target.target_type != "discrete":
                    continue
                x, y = float(target.map_pose.pose.position.x), float(target.map_pose.pose.position.y)
                candidates = [
                    (math.hypot(x - float(cube["pose"]["x_m"]), y - float(cube["pose"]["y_m"])), str(cube["object_id"]))
                    for cube in cubes
                ]
                distance, truth_id = min(candidates, default=(math.inf, ""))
                uuid = str(target.uuid)
                if distance > 0.75:
                    self.target_false_uuid.add(uuid)
                    continue
                self.target_errors_by_uuid[uuid] = min(distance, self.target_errors_by_uuid.get(uuid, math.inf))
                self.target_errors_by_truth[truth_id] = min(distance, self.target_errors_by_truth.get(truth_id, math.inf))

        def _on_diagnostics(self, message) -> None:
            for status in message.status:
                values = {item.key: item.value for item in status.values}
                if "ground_truth_input_used" in values:
                    try:
                        self.diagnostic_truth_flags.append(bool(json.loads(values["ground_truth_input_used"])))
                    except json.JSONDecodeError:
                        self.diagnostic_truth_flags.append(True)
                if "fail_closed" in values:
                    try:
                        self.diagnostic_fail_closed_count += int(bool(json.loads(values["fail_closed"])))
                    except json.JSONDecodeError:
                        self.diagnostic_fail_closed_count += 1

        def _spin_for(self, seconds: float) -> None:
            deadline = time.monotonic() + seconds
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

        def _set_entity_pose(self, entity_name: str, x: float, y: float, z: float, yaw: float) -> bool:
            request = SetEntityPose.Request()
            request.entity.name = entity_name
            request.pose = Pose()
            request.pose.position.x, request.pose.position.y, request.pose.position.z = x, y, z
            request.pose.orientation.z = math.sin(yaw / 2.0)
            request.pose.orientation.w = math.cos(yaw / 2.0)
            future = self.pose_client.call_async(request)
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            return bool(future.done() and future.result() is not None and future.result().success)

        def _stage_visible_scene(self) -> tuple[int, int]:
            """Move randomized episode objects before sampling; never move the vehicle.

            The episode generator still owns randomized colour, material, shape and
            identity.  The evaluator only places those existing entities in a fixed
            vehicle-relative acceptance ROI.  Updated poses remain private in this
            evaluator and are never published to the product graph.
            """

            # Gazebo SetEntityPose consumes world coordinates. The public
            # episode start is the authoritative world spawn, while the
            # diff-drive odom origin is intentionally reset to zero. Retain a
            # separate localized map-frame pose for evaluator truth. Retry the
            # complete TF pair because its two edges can enter a fresh buffer
            # one sample apart during startup.
            try:
                map_from_base, odom_from_base = _lookup_pose_pair_with_retry(
                    lambda target: self.tf_buffer.lookup_transform(
                        target,
                        "base_link",
                        Time(),
                        timeout=Duration(seconds=0.10),
                    ),
                    lambda: rclpy.spin_once(self, timeout_sec=0.05),
                    timeout_s=float(
                        self.config["staged_scene"].get("pose_pair_timeout_s", 30.0)
                    ),
                )
            except RuntimeError as exc:
                raise RuntimeError(
                    f"cannot stage fixed-vehicle scene without map/odom->base_link: {exc}"
                ) from exc

            def pose2d(transform) -> tuple[float, float, float]:
                q = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                return (
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    yaw,
                )

            map_base_x, map_base_y, map_yaw = pose2d(map_from_base)
            try:
                camera_from_base = self.tf_buffer.lookup_transform(
                    "front_rgbd_depth_optical_frame",
                    "base_link",
                    Time(),
                    timeout=Duration(seconds=0.50),
                )
            except TransformException as exc:
                raise RuntimeError(
                    "cannot stage scene without front-camera/base transform: "
                    f"{exc}"
                ) from exc
            self.staged_front_camera_from_map = _camera_from_map_at_staging(
                _transform_matrix(map_from_base.transform),
                _transform_matrix(camera_from_base.transform),
            )
            # Localization owns map->base_link at z=0, while Gazebo renders
            # the physical camera above the wheel-ground base_footprint. Read
            # that public URDF datum from TF so evaluator projection uses the
            # same physical ground plane as the real Gazebo image.
            try:
                base_from_footprint = self.tf_buffer.lookup_transform(
                    "base_link",
                    "base_footprint",
                    Time(),
                    timeout=Duration(seconds=0.50),
                )
            except TransformException as exc:
                raise RuntimeError(
                    f"cannot stage scene without base_footprint datum: {exc}"
                ) from exc
            self.map_ground_z_m = float(map_from_base.transform.translation.z) + float(
                base_from_footprint.transform.translation.z
            )
            # Odom is required above only as a runtime-completeness check.  A
            # freshly spawned diff-drive controller intentionally starts odom
            # at zero, while Gazebo world and map use the public episode's
            # fixed start coordinate (for example x=-98 m).  Therefore odom
            # must never be misused as a world pose for SetEntityPose.
            pose2d(odom_from_base)
            world_base_x, world_base_y, world_yaw = self.public_start_pose
            self.localized_map_pose = (map_base_x, map_base_y, map_yaw)
            (
                self.localization_start_error_m,
                self.localization_start_yaw_error_rad,
            ) = _pose2d_error(
                self.localized_map_pose, self.localization_start_pose
            )
            maximum_start_error = float(
                self.config["staged_scene"].get(
                    "fixed_start_localization_error_m_max", 0.50
                )
            )
            if self.localization_start_error_m > maximum_start_error:
                raise RuntimeError(
                    "localized fixed start does not match localization-map contract: "
                    f"error={self.localization_start_error_m:.6f}m "
                    f"maximum={maximum_start_error:.6f}m"
                )

            def relative_point(
                base_x: float,
                base_y: float,
                yaw: float,
                forward: float,
                lateral: float,
            ) -> tuple[float, float]:
                return (
                    base_x + forward * math.cos(yaw) - lateral * math.sin(yaw),
                    base_y + forward * math.sin(yaw) + lateral * math.cos(yaw),
                )

            cube_slots = [
                (forward, lateral)
                for forward in (1.15, 1.35, 1.55, 1.75, 1.95)
                for lateral in (-0.54, -0.18, 0.18, 0.54)
            ]
            cube_successes = 0
            for cube, (forward, lateral) in zip(self.truth["discrete_cubes"], cube_slots):
                world_x, world_y = relative_point(
                    world_base_x, world_base_y, world_yaw, forward, lateral
                )
                map_x, map_y = relative_point(
                    map_base_x, map_base_y, map_yaw, forward, lateral
                )
                edge = float(cube["edge_m"])
                if self._set_entity_pose(
                    str(cube["object_id"]), world_x, world_y, edge / 2.0, world_yaw
                ):
                    cube_successes += 1
                    cube["pose"].update(
                        {
                            "x_m": map_x,
                            "y_m": map_y,
                            "z_m": self.map_ground_z_m + edge / 2.0,
                            "yaw_rad": map_yaw,
                        }
                    )

            dirt_slots = [
                (forward, lateral)
                for forward in (2.4, 3.5, 4.6, 5.7, 6.8, 7.9)
                for lateral in (-1.1, 0.0, 1.1)
            ]
            dirt_successes = 0
            for patch, (forward, lateral) in zip(self.truth["dirt_patches"], dirt_slots):
                world_x, world_y = relative_point(
                    world_base_x, world_base_y, world_yaw, forward, lateral
                )
                map_x, map_y = relative_point(
                    map_base_x, map_base_y, map_yaw, forward, lateral
                )
                if self._set_entity_pose(str(patch["object_id"]), world_x, world_y, 0.002, world_yaw):
                    dirt_successes += 1
                    patch["pose"].update({"x_m": map_x, "y_m": map_y, "yaw_rad": map_yaw})
            return cube_successes, dirt_successes

        def execute(self) -> dict:
            timeout = float(self.get_parameter("startup_timeout_s").value)
            if not self.pose_client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError("evaluator SetEntityPose service unavailable")
            deadline = time.monotonic() + timeout
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.map_message is not None and len(self.rgb_topics_seen) == 4 and len(self.info_topics_seen) == 4 and self.diagnostic_truth_flags:
                    break
            missing_startup_inputs = []
            if self.map_message is None:
                missing_startup_inputs.append("public_map")
            if len(self.rgb_topics_seen) < 4:
                missing_startup_inputs.append("four_rgb_topics")
            if len(self.info_topics_seen) < 4:
                missing_startup_inputs.append("four_camera_info_topics")
            if not self.diagnostic_truth_flags:
                missing_startup_inputs.append("product_liveness_diagnostic")
            if missing_startup_inputs:
                raise RuntimeError(
                    "formal perception startup inputs unavailable: "
                    + ", ".join(missing_startup_inputs)
                )
            cube_stage_count, dirt_stage_count = self._stage_visible_scene()
            self.get_logger().info(
                f"fixed-vehicle scene staged cubes={cube_stage_count}/{len(self.truth['discrete_cubes'])} "
                f"dirt={dirt_stage_count}/{len(self.truth['dirt_patches'])}; vehicle teleport prohibited"
            )
            self._spin_for(float(self.config["staged_scene"]["settle_time_s"]))
            self.acceptance_sampling_start_s = self.get_clock().now().nanoseconds * 1e-9
            self.acceptance_sampling_active = True
            self._spin_for(float(self.config["staged_scene"]["sample_time_s"]))
            if self.map_message is None:
                segmentation = {"iou": 0.0, "recall": 0.0, "reason": "public_map_missing"}
            else:
                info = self.map_message.info
                origin = info.origin.position
                truth_raster = rasterize_dirt_truth(
                    self.truth["dirt_patches"],
                    width=int(info.width),
                    height=int(info.height),
                    resolution=float(info.resolution),
                    origin_x=float(origin.x),
                    origin_y=float(origin.y),
                )
                predicted = self.accumulated_dirty if self.accumulated_dirty is not None else np.zeros_like(truth_raster)
                segmentation = segmentation_metrics(predicted, truth_raster)
                segmentation["observed_cell_count"] = int(np.count_nonzero(self.accumulated_observed)) if self.accumulated_observed is not None else 0
            projection = projection_error_metrics(self.target_errors_by_truth.values())
            projection["unique_product_track_count"] = len(self.target_errors_by_uuid)
            projection["matched_truth_cube_count"] = len(self.target_errors_by_truth)
            projection["false_product_track_count"] = len(self.target_false_uuid)
            diagnostics_truth_free = bool(self.diagnostic_truth_flags) and not any(self.diagnostic_truth_flags)
            freshness = {
                "rgb_topic_count": len(self.rgb_topics_seen),
                "depth_topic_count": len(self.depth_topics_seen),
                "camera_info_topic_count": len(self.info_topics_seen),
                "rgb_topics_seen": sorted(self.rgb_topics_seen),
                "depth_topics_seen": sorted(self.depth_topics_seen),
                "camera_info_topics_seen": sorted(self.info_topics_seen),
                "real_camera_message_count": self.real_camera_message_count,
                "depth_rgb_skew_sample_count": len(self.depth_rgb_skews),
                "depth_rgb_skew_max_s": max(self.depth_rgb_skews, default=None),
                "tf_attempt_count": self.tf_attempts,
                "tf_success_count": self.tf_successes,
                "tf_success_ratio": self.tf_successes / self.tf_attempts if self.tf_attempts else 0.0,
                "tf_age_sample_count": len(self.tf_ages),
                "tf_age_max_s": max(self.tf_ages, default=None),
                "diagnostic_ground_truth_input_used": False if diagnostics_truth_free else None,
                "diagnostic_fail_closed_count": self.diagnostic_fail_closed_count,
                "product_detection_message_count": self.product_detection_message_count,
                "product_mask_message_count": self.product_mask_message_count,
                "product_target_message_count": self.product_target_message_count,
                "vehicle_teleport_used": False,
                "scene_staging_world_pose_source": (
                    "public_episode_manifest_vehicle_start_pose_source_world"
                ),
                "localization_start_error_m": self.localization_start_error_m,
                "localization_start_yaw_error_rad": self.localization_start_yaw_error_rad,
                "localized_map_pose": self.localized_map_pose,
                "expected_localization_map_start_pose": self.localization_start_pose,
                "source_world_start_pose_used_for_staging": self.public_start_pose,
                "source_world_start_contract_error_m": self.source_world_start_contract_error_m,
                "source_world_start_contract_yaw_error_rad": self.source_world_start_contract_yaw_error_rad,
                "map_ground_z_m": self.map_ground_z_m,
                "staged_cube_count": cube_stage_count,
                "staged_dirt_patch_count": dirt_stage_count,
            }
            report = finalize_acceptance(
                episode_id=str(self.truth.get("episode_id", "unknown")),
                detection={
                    "true_positive_count": self.true_positive_count,
                    "false_positive_count": self.false_positive_count,
                    "visible_unique_truth_count": len(self.visible_truth_ids),
                    "matched_unique_truth_count": len(self.matched_truth_ids),
                    "evaluated_frame_count": self.evaluated_frame_count,
                },
                segmentation=segmentation,
                projection=projection,
                freshness=freshness,
                thresholds={**self.config["metrics"], **self.config["runtime"]},
            )
            diagnostic_path_raw = str(self.get_parameter("diagnostic_frame_path").value)
            if diagnostic_path_raw and self.best_front_diagnostic is not None:
                import cv2

                diagnostic_path = Path(diagnostic_path_raw)
                diagnostic_path.parent.mkdir(parents=True, exist_ok=True)
                _, rgb, metadata = self.best_front_diagnostic
                if not cv2.imwrite(str(diagnostic_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError(f"failed to save real Gazebo diagnostic frame: {diagnostic_path}")
                metadata["image_path"] = str(diagnostic_path)
                metadata["image_shape_hwc"] = list(rgb.shape)
                diagnostic_path.with_suffix(".json").write_text(
                    json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8"
                )
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            return report

    rclpy.init()
    node = None
    try:
        node = FormalRandomSceneEvaluator()
        report = node.execute()
        print(json.dumps({"output": str(node.output_path), "status": report["status"]}, ensure_ascii=False))
        exit_code = 0 if report["status"] == "PASSED" else 3
    except Exception as exc:
        if node is not None and getattr(node, "output_path", None):
            failure = {
                "schema_version": 1,
                "report_id": "tzcup_formal_random_scene_perception_episode_v1",
                "status": "BLOCKED_RUNTIME",
                "error": str(exc),
                "truth_isolation": {"truth_published_to_ros": False, "truth_used_by_product_control": False},
            }
            node.output_path.parent.mkdir(parents=True, exist_ok=True)
            node.output_path.write_text(json.dumps(failure, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"formal random-scene evaluator failed closed: {exc}")
        exit_code = 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
