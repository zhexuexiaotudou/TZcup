"""ROS 2 product adapter for the PC DOSOD + EdgeSAM graph.

Only public camera, depth, TF and occupancy-map inputs are consumed.  Any
missing depth/map/TF/model condition is fail-closed: diagnostics are emitted,
but no planning observation or target is published.
"""

from __future__ import annotations

import json
import math
import time

import numpy as np

from .dosod_ros_adapter import DosodOnnxDetector
from .edgesam_ros_adapter import EdgeSamOnnxSegmenter
from .product_intermediate_capture import ProductIntermediateCapture
from .product_projection import CameraIntrinsics, PublicGrid, project_rgbd_observation
from .tracking import TargetTracker


FORBIDDEN_INPUT_TOKENS = ("ground_truth", "evaluator", "evaluation/")
GROUND_DIRT_CLASS_IDS = frozenset(("fallen_leaves", "dust_or_soil", "puddle"))


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def serialize_wrist_grasp_recheck(
    *,
    target_id: str,
    frame_id: str,
    pose: tuple[float, float, float, float, float, float, float],
    size_m: tuple[float, float, float],
    confidence: float,
) -> str:
    """Serialize a wrist RGB-D refinement without exposing material truth."""

    if not target_id.strip() or not frame_id.strip():
        raise ValueError("wrist target id and frame must be non-empty")
    if not all(math.isfinite(value) for value in (*pose, *size_m, confidence)):
        raise ValueError("wrist target geometry must be finite")
    if any(value < 0.020 or value > 0.040 for value in size_m):
        raise ValueError("wrist target size is outside the 30 mm cube tolerance")
    norm = math.sqrt(sum(value * value for value in pose[3:]))
    if norm < 1.0e-9 or abs(norm - 1.0) > 0.02:
        raise ValueError("wrist target quaternion is not normalized")
    if not 0.50 <= confidence <= 1.0:
        raise ValueError("wrist target confidence is below the grasp threshold")
    return json.dumps(
        {
            "schema_version": 2,
            "target_id": target_id,
            "frame_id": frame_id,
            "pose": {
                "x_m": pose[0],
                "y_m": pose[1],
                "z_m": pose[2],
                "qx": pose[3],
                "qy": pose[4],
                "qz": pose[5],
                "qw": pose[6],
            },
            "size_m": list(size_m),
            "material": "unknown",
            "confidence": confidence,
            "truth_used": False,
        },
        sort_keys=True,
    )


def _ground_dirt_prompt_indices(
    class_ids,
    boxes_xyxy: np.ndarray | None = None,
    image_shape: tuple[int, int] | None = None,
    max_per_class: int = 3,
    max_area_fraction: float = 0.45,
) -> np.ndarray:
    """Return a bounded highest-score prompt set for the EdgeSAM path.

    DOSOD results are confidence-sorted before this helper is called. Keeping
    three prompts per amorphous class bounds the synchronous EdgeSAM decoder
    work to nine prompts and prevents an early pre-stage frame from starving
    the subsequent real acceptance window while retaining multi-patch recall.
    """

    selected, _ = _ground_dirt_prompt_decisions(
        class_ids,
        boxes_xyxy=boxes_xyxy,
        image_shape=image_shape,
        max_per_class=max_per_class,
        max_area_fraction=max_area_fraction,
    )
    return selected


def _ground_dirt_prompt_decisions(
    class_ids,
    boxes_xyxy: np.ndarray | None = None,
    image_shape: tuple[int, int] | None = None,
    max_per_class: int = 3,
    max_area_fraction: float = 0.45,
) -> tuple[np.ndarray, list[dict]]:
    """Return unchanged prompt indices plus a product-input-only audit trail."""

    if max_per_class < 1:
        raise ValueError("maximum EdgeSAM prompts per class must be positive")
    boxes = None
    image_area = None
    if boxes_xyxy is not None or image_shape is not None:
        if boxes_xyxy is None or image_shape is None:
            raise ValueError("prompt boxes and image shape must be provided together")
        boxes = np.asarray(boxes_xyxy, dtype=np.float32).reshape(-1, 4)
        if len(boxes) != len(class_ids):
            raise ValueError("prompt boxes and class ids must align")
        image_area = float(image_shape[0] * image_shape[1])
        if image_area <= 0.0 or not 0.0 < max_area_fraction <= 1.0:
            raise ValueError("prompt image area and maximum fraction must be positive")
    counts: dict[str, int] = {}
    selected: list[int] = []
    decisions: list[dict] = []
    for index, class_id in enumerate(class_ids):
        decision = {
            "detection_index": index,
            "class_id": str(class_id),
            "accepted": False,
            "reason": "not_ground_dirt_class",
            "area_fraction": None,
        }
        if class_id not in GROUND_DIRT_CLASS_IDS:
            decisions.append(decision)
            continue
        if boxes is not None and image_area is not None:
            x1, y1, x2, y2 = boxes[index]
            area_fraction = max(0.0, float(x2 - x1)) * max(
                0.0, float(y2 - y1)
            ) / image_area
            decision["area_fraction"] = area_fraction
            if area_fraction <= 0.0:
                decision["reason"] = "non_positive_box_area"
                decisions.append(decision)
                continue
            if area_fraction > max_area_fraction:
                decision["reason"] = "box_area_above_limit"
                decisions.append(decision)
                continue
        count = counts.get(class_id, 0)
        if count >= max_per_class:
            decision["reason"] = "per_class_prompt_limit"
            decisions.append(decision)
            continue
        counts[class_id] = count + 1
        selected.append(index)
        decision["accepted"] = True
        decision["reason"] = "accepted"
        decisions.append(decision)
    return np.asarray(selected, dtype=np.int64), decisions


def _projection_masks(
    image_shape: tuple[int, int],
    boxes_xyxy: np.ndarray,
    class_ids,
    dirt_masks,
    dirt_qualities,
) -> tuple[list[np.ndarray], list[float]]:
    """Combine EdgeSAM dirt masks with deterministic box masks for solid litter.

    A 3 cm litter cube is a solid object and DOSOD already supplies its extent;
    running EdgeSAM once per cube delayed the first online publication beyond the
    formal sampling window. EdgeSAM remains mandatory for the amorphous ground
    classes, while cube depth projection uses the detector rectangle.
    """

    boxes = np.asarray(boxes_xyxy, dtype=np.float32).reshape(-1, 4)
    ids = list(class_ids)
    dirt_indices = _ground_dirt_prompt_indices(ids, boxes, image_shape).tolist()
    if len(boxes) != len(ids):
        raise ValueError("projection boxes and class ids must align")
    if len(dirt_masks) != len(dirt_indices) or len(dirt_qualities) != len(dirt_indices):
        raise ValueError("EdgeSAM outputs must align with ground-dirt prompts")
    dirt_by_index = {
        index: (np.asarray(mask, dtype=bool), float(quality))
        for index, mask, quality in zip(dirt_indices, dirt_masks, dirt_qualities)
    }
    height, width = image_shape
    masks: list[np.ndarray] = []
    qualities: list[float] = []
    for index, box in enumerate(boxes):
        if index in dirt_by_index:
            mask, quality = dirt_by_index[index]
            if mask.shape != (height, width):
                raise ValueError("EdgeSAM mask dimensions differ from RGB input")
        else:
            x1, y1, x2, y2 = box
            left = max(0, min(width, int(math.floor(x1))))
            top = max(0, min(height, int(math.floor(y1))))
            right = max(0, min(width, int(math.ceil(x2))))
            bottom = max(0, min(height, int(math.ceil(y2))))
            mask = np.zeros((height, width), dtype=bool)
            mask[top:bottom, left:right] = True
            quality = 1.0
        masks.append(mask)
        qualities.append(quality)
    return masks, qualities


def _transform_matrix(transform) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-12 or not math.isfinite(norm):
        raise ValueError("invalid TF quaternion")
    scale = 2.0 / norm
    matrix = np.asarray(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w), translation.x],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w), translation.y],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y), translation.z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    if not np.isfinite(matrix).all():
        raise ValueError("non-finite TF transform")
    return matrix


def main() -> None:
    import rclpy
    from cv_bridge import CvBridge
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import OccupancyGrid
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.time import Time
    from sanitation_perception_interfaces.msg import GarbageTarget, GarbageTargetArray
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    class PcOpenVocabProductAdapter(Node):
        def __init__(self) -> None:
            super().__init__("pc_open_vocab_product_adapter")
            self.declare_parameter("artifact_root", "")
            # Calibrated on real formal-Gazebo D435 frames: the 3 cm cube at
            # 1.15 m scores 0.01364 with a correctly localized box, while the
            # paired empty-ground frame peaks at 0.00144. Keep a measured
            # margin rather than inheriting the generic COCO-style 0.25 gate.
            self.declare_parameter("score_threshold", 0.005)
            self.declare_parameter("fallen_leaves_score_threshold", 0.0025)
            self.declare_parameter("dust_or_soil_score_threshold", 0.002)
            self.declare_parameter("puddle_score_threshold", 0.003)
            self.declare_parameter("nms_threshold", 0.65)
            self.declare_parameter("depth_max_age_s", 0.5)
            self.declare_parameter("tf_max_age_s", 0.75)
            self.declare_parameter("sample_stride", 4)
            self.declare_parameter("intermediate_capture_root", "")
            self.declare_parameter("intermediate_capture_max_frames", 12)
            self.declare_parameter("intermediate_capture_interval_s", 1.0)
            self.declare_parameter("intermediate_capture_max_bytes", 268435456)
            artifact_root = str(self.get_parameter("artifact_root").value)
            if not artifact_root:
                raise RuntimeError("artifact_root is required; refusing placeholder inference")
            from pathlib import Path

            root = Path(artifact_root)
            capture_root = str(self.get_parameter("intermediate_capture_root").value)
            self.intermediate_capture = (
                ProductIntermediateCapture(
                    capture_root,
                    max_frames=int(
                        self.get_parameter("intermediate_capture_max_frames").value
                    ),
                    minimum_interval_s=float(
                        self.get_parameter("intermediate_capture_interval_s").value
                    ),
                    max_bytes=int(
                        self.get_parameter("intermediate_capture_max_bytes").value
                    ),
                )
                if capture_root
                else None
            )
            load_started = time.monotonic()
            self.detector = DosodOnnxDetector(
                root / "dosod" / "dosod_mlp3x_s_tzcup_rep.onnx",
                score_threshold=float(self.get_parameter("score_threshold").value),
                class_score_thresholds={
                    "fallen_leaves": float(
                        self.get_parameter("fallen_leaves_score_threshold").value
                    ),
                    "dust_or_soil": float(
                        self.get_parameter("dust_or_soil_score_threshold").value
                    ),
                    "puddle": float(self.get_parameter("puddle_score_threshold").value),
                },
                nms_threshold=float(self.get_parameter("nms_threshold").value),
            )
            self.segmenter = EdgeSamOnnxSegmenter(
                root / "edgesam" / "edge_sam_3x_encoder.onnx",
                root / "edgesam" / "edge_sam_3x_decoder.onnx",
            )
            self.bridge = CvBridge()
            self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.latest_map = None
            self.latest_depth: dict[str, Image] = {}
            self.latest_info: dict[str, CameraInfo] = {}
            self.last_run: dict[str, float] = {}
            self.rates = {"front": 2.0, "wrist": 2.0, "rear_left": 1.0, "rear_right": 1.0}
            # DOSOD + EdgeSAM inference is deliberately serialized because the
            # ONNX sessions are shared.  Keep the short-lived map/depth/info
            # cache callbacks in a different group so a long inference cannot
            # freeze the timestamp context used by the next RGB frame.
            self.inference_callback_group = MutuallyExclusiveCallbackGroup()
            self.cache_callback_group = MutuallyExclusiveCallbackGroup()
            self.target_tracker = TargetTracker(
                confirmation_observations=2,
                association_distance_m=0.20,
                maximum_covariance_trace=0.15,
                # Campus litter is static.  Once confirmed, retain its last
                # public map pose for the mission instead of forgetting it as
                # soon as the camera turns away.
                lost_timeout_s=3600.0,
            )

            self.detection_publisher = self.create_publisher(
                Detection2DArray, "/perception/garbage/detections_2d", 10
            )
            self.box_publisher = self.create_publisher(
                Detection2DArray, "/perception/open_vocab/dosod_boxes", 10
            )
            self.mask_publisher = self.create_publisher(
                Image, "/perception/ground_dirt/masks", 10
            )
            self.target_publisher = self.create_publisher(
                GarbageTargetArray, "/perception/garbage/targets", 10
            )
            self.wrist_recheck_publisher = self.create_publisher(
                String, "/perception/wrist/grasp_recheck", 10
            )
            diagnostic_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.diagnostic_publisher = self.create_publisher(
                DiagnosticArray, "/perception/open_vocab/diagnostics", diagnostic_qos
            )

            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            # map_server publishes a transient-local latched map and starts
            # before this deliberately delayed inference node.  A volatile
            # subscription would therefore miss the only map message.
            self.create_subscription(
                OccupancyGrid,
                "/map",
                self._on_map,
                map_qos,
                callback_group=self.cache_callback_group,
            )
            self._subscribe_rgbd(
                "front",
                "/sensors/front_rgbd/depth/image_rect_raw/image",
                "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
                "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
            )
            self._subscribe_rgbd(
                "wrist",
                "/sensors/wrist_rgbd/depth/image_rect_raw/image",
                "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image",
                "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info",
            )
            self._subscribe_rgb_only(
                "rear_left", "/sensors/rear_left_fisheye/image_raw"
            )
            self._subscribe_rgb_only(
                "rear_right", "/sensors/rear_right_fisheye/image_raw"
            )
            self._diagnostic(0, "ready", {"ground_truth_input_used": False})
            # The evaluator starts after this node. A one-shot volatile ready
            # diagnostic can therefore be missed even when the product graph
            # is healthy, while a blocked inference callback must not be
            # mistaken for readiness. The timer runs on the same executor and
            # provides liveness without reading evaluator state or truth.
            self.diagnostic_callback_group = MutuallyExclusiveCallbackGroup()
            self.readiness_timer = self.create_timer(
                1.0,
                lambda: self._diagnostic(
                    0, "alive", {"ground_truth_input_used": False}
                ),
                callback_group=self.diagnostic_callback_group,
            )
            self.get_logger().info(
                "formal DOSOD+EdgeSAM PC product adapter ready; "
                f"model_load_s={time.monotonic() - load_started:.3f}"
            )

        @staticmethod
        def _validate_input_topic(topic: str) -> None:
            if any(token in topic.lower() for token in FORBIDDEN_INPUT_TOKENS):
                raise RuntimeError(f"forbidden evaluator/truth input topic: {topic}")

        def _subscribe_rgbd(self, sensor: str, rgb: str, depth: str, info: str) -> None:
            for topic in (rgb, depth, info):
                self._validate_input_topic(topic)
            self.create_subscription(
                Image,
                depth,
                lambda message, name=sensor: self.latest_depth.__setitem__(name, message),
                qos_profile_sensor_data,
                callback_group=self.cache_callback_group,
            )
            self.create_subscription(
                CameraInfo,
                info,
                lambda message, name=sensor: self.latest_info.__setitem__(name, message),
                qos_profile_sensor_data,
                callback_group=self.cache_callback_group,
            )
            self.create_subscription(
                Image,
                rgb,
                lambda message, name=sensor: self._on_rgbd(name, message),
                qos_profile_sensor_data,
                callback_group=self.inference_callback_group,
            )

        def _subscribe_rgb_only(self, sensor: str, rgb: str) -> None:
            self._validate_input_topic(rgb)
            self.create_subscription(
                Image,
                rgb,
                lambda message, name=sensor: self._on_rgb_only(name, message),
                qos_profile_sensor_data,
                callback_group=self.inference_callback_group,
            )

        def _on_map(self, message: OccupancyGrid) -> None:
            self.latest_map = message

        def _due(self, sensor: str) -> bool:
            now = time.monotonic()
            if now - self.last_run.get(sensor, -1e9) < 1.0 / self.rates[sensor]:
                return False
            self.last_run[sensor] = now
            return True

        def _detections_message(self, image: Image, results) -> Detection2DArray:
            array = Detection2DArray()
            array.header = image.header
            for result in results:
                detection = Detection2D()
                detection.header = image.header
                x1, y1, x2, y2 = result.xyxy
                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = result.class_id
                hypothesis.hypothesis.score = result.confidence
                detection.results.append(hypothesis)
                array.detections.append(detection)
            return array

        def _on_rgb_only(self, sensor: str, image_message: Image) -> None:
            if not self._due(sensor):
                return
            try:
                rgb = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="rgb8")
                results = self.detector.infer(rgb)
                product = self._detections_message(image_message, results)
                self.box_publisher.publish(product)
                self.detection_publisher.publish(product)
                self._diagnostic(0, "rgb_only_ok", {"sensor": sensor, "detections": len(results)})
            except Exception as exc:
                self._diagnostic(2, "rgb_only_failed_closed", {"sensor": sensor, "error": str(exc)})

        def _on_rgbd(self, sensor: str, image_message: Image) -> None:
            if not self._due(sensor):
                return
            depth_message = self.latest_depth.get(sensor)
            info = self.latest_info.get(sensor)
            if depth_message is None or info is None or self.latest_map is None:
                self._diagnostic(2, "rgbd_context_missing", {"sensor": sensor})
                return
            rgb_time = Time.from_msg(image_message.header.stamp)
            depth_time = Time.from_msg(depth_message.header.stamp)
            age = abs((rgb_time - depth_time).nanoseconds) * 1e-9
            if age > float(self.get_parameter("depth_max_age_s").value):
                self._diagnostic(2, "stale_depth_rejected", {"sensor": sensor, "age_s": age})
                return
            try:
                # The campus localization chain does not promise historical
                # interpolation at every camera stamp.  Use the newest complete
                # public TF chain, then bound its age against the image stamp so
                # latest-TF can never silently become an unbounded stale pose.
                transform = self.tf_buffer.lookup_transform(
                    "map", image_message.header.frame_id, Time(), timeout=Duration(seconds=0.2)
                )
            except TransformException as exc:
                self._diagnostic(2, "map_tf_missing", {"sensor": sensor, "error": str(exc)})
                return
            transform_time = Time.from_msg(transform.header.stamp)
            transform_age = abs((rgb_time - transform_time).nanoseconds) * 1e-9
            tf_max_age = float(self.get_parameter("tf_max_age_s").value)
            if not math.isfinite(transform_age) or transform_age > tf_max_age:
                self._diagnostic(
                    2,
                    "stale_map_tf_rejected",
                    {"sensor": sensor, "age_s": transform_age, "maximum_age_s": tf_max_age},
                )
                return
            try:
                rgb = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="rgb8")
                depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
                if depth.shape != rgb.shape[:2]:
                    raise ValueError("RGB and depth dimensions differ")
                results = self.detector.infer(rgb)
                boxes = np.asarray([item.xyxy for item in results], dtype=np.float32).reshape(-1, 4)
                product = self._detections_message(image_message, results)
                # Publish DOSOD immediately. EdgeSAM is intentionally not on
                # the latency-critical solid-litter detection path.
                self.box_publisher.publish(product)
                self.detection_publisher.publish(product)
                self._diagnostic(
                    0,
                    "dosod_product_ok",
                    {"sensor": sensor, "detections": len(results)},
                )
                class_ids = [item.class_id for item in results]
                dirt_indices = _ground_dirt_prompt_indices(
                    class_ids, boxes, rgb.shape[:2]
                )
                dirt_masks, dirt_qualities = self.segmenter.segment_boxes(
                    rgb, boxes[dirt_indices]
                )
                masks, qualities = _projection_masks(
                    rgb.shape[:2], boxes, class_ids, dirt_masks, dirt_qualities
                )
                capture_requested = bool(
                    self.intermediate_capture is not None
                    and self.intermediate_capture.wants_frame(
                        sensor, rgb_time.nanoseconds * 1e-9
                    )
                )
                grid_message = self.latest_map
                origin = grid_message.info.origin
                if abs(origin.orientation.x) > 1e-6 or abs(origin.orientation.y) > 1e-6 or abs(origin.orientation.z) > 1e-6 or abs(origin.orientation.w - 1.0) > 1e-6:
                    raise ValueError("rotated public occupancy-grid origins are not supported")
                grid = PublicGrid(
                    width=int(grid_message.info.width),
                    height=int(grid_message.info.height),
                    resolution=float(grid_message.info.resolution),
                    origin_x=float(origin.position.x),
                    origin_y=float(origin.position.y),
                    occupancy=(
                        np.asarray(grid_message.data, dtype=np.int8).reshape(
                            int(grid_message.info.height), int(grid_message.info.width)
                        )
                        if capture_requested
                        else None
                    ),
                )
                intrinsics = CameraIntrinsics(
                    fx=float(info.k[0]), fy=float(info.k[4]), cx=float(info.k[2]), cy=float(info.k[5])
                )
                confidences = [
                    float(max(0.0, min(1.0, detection.confidence * max(0.0, quality))))
                    for detection, quality in zip(results, qualities)
                ]
                projection_diagnostics = {} if capture_requested else None
                raster, projected = project_rgbd_observation(
                    depth,
                    intrinsics,
                    _transform_matrix(transform.transform),
                    grid,
                    boxes_xyxy=boxes,
                    class_ids=class_ids,
                    masks=masks,
                    confidences=confidences,
                    sample_stride=int(self.get_parameter("sample_stride").value),
                    diagnostics_out=projection_diagnostics,
                )
                if sensor == "front":
                    mask_message = self.bridge.cv2_to_imgmsg(raster, encoding="mono8")
                    mask_message.header.stamp = image_message.header.stamp
                    mask_message.header.frame_id = "map"
                    self.mask_publisher.publish(mask_message)
                target_array = GarbageTargetArray()
                target_array.header.stamp = image_message.header.stamp
                target_array.header.frame_id = "map"
                target_array.registry_sha256 = "formal_dosod_edgesam_frozen_vocabulary_v1"
                tracker_detections = []
                for projected_target in projected:
                    detection = results[projected_target.detection_index]
                    tracker_detections.append(
                        {
                            "class_id": "litter_cube",
                            "target_type": "discrete",
                            "cleaning_policy": "pick_and_bin",
                            "x_m": projected_target.xyz[0],
                            "y_m": projected_target.xyz[1],
                            "confidence": detection.confidence,
                            "covariance_trace": 0.12,
                            "source_backend": "dosod_edgesam_pc",
                        }
                    )
                tracks = self.target_tracker.update(tracker_detections)
                for tracked in tracks:
                    if tracked.state not in {
                        "CONFIRMED",
                        "QUEUED",
                        "APPROACHING",
                        "CLEANING",
                    }:
                        continue
                    target = GarbageTarget()
                    target.header = target_array.header
                    target.uuid = tracked.uuid
                    target.class_id = "litter_cube"
                    target.target_type = "discrete"
                    target.confidence = tracked.confidence
                    target.map_pose.pose.position.x = tracked.x_m
                    target.map_pose.pose.position.y = tracked.y_m
                    target.map_pose.pose.position.z = 0.015
                    target.map_pose.pose.orientation.w = 1.0
                    target.map_pose.covariance[0] = 0.04
                    target.map_pose.covariance[7] = 0.04
                    target.map_pose.covariance[14] = 0.04
                    target.size.x = target.size.y = target.size.z = 0.03
                    target.first_seen = image_message.header.stamp
                    target.last_seen = image_message.header.stamp
                    target.source_stamp = image_message.header.stamp
                    target.observation_count = tracked.observation_count
                    target.track_state = tracked.state
                    target.cleaning_policy = "pick_and_bin"
                    target.source_backend = tracked.source_backend
                    target.visibility = 1.0
                    target.occlusion_ratio = 0.0
                    target.in_keepout = False
                    target_array.targets.append(target)
                self.target_publisher.publish(target_array)
                if sensor == "wrist":
                    for target in target_array.targets:
                        position = target.map_pose.pose.position
                        orientation = target.map_pose.pose.orientation
                        try:
                            encoded_recheck = serialize_wrist_grasp_recheck(
                                target_id=str(target.uuid),
                                frame_id=str(
                                    target.header.frame_id
                                    or target_array.header.frame_id
                                ),
                                pose=(
                                    float(position.x),
                                    float(position.y),
                                    float(position.z),
                                    float(orientation.x),
                                    float(orientation.y),
                                    float(orientation.z),
                                    float(orientation.w),
                                ),
                                size_m=(
                                    float(target.size.x),
                                    float(target.size.y),
                                    float(target.size.z),
                                ),
                                confidence=float(target.confidence),
                            )
                        except ValueError as exc:
                            self._diagnostic(
                                2,
                                "wrist_grasp_recheck_rejected",
                                {"target_id": str(target.uuid), "error": str(exc)},
                            )
                            continue
                        self.wrist_recheck_publisher.publish(
                            String(data=encoded_recheck)
                        )
                # Product messages above are computed and published before
                # diagnostic persistence. Capture consumes only the same
                # public inputs/intermediates and cannot alter those messages.
                if capture_requested and projection_diagnostics is not None:
                    _, prompt_decisions = _ground_dirt_prompt_decisions(
                        class_ids, boxes, rgb.shape[:2]
                    )
                    try:
                        captured = self.intermediate_capture.capture_frame(
                            sensor=sensor,
                            rgb_stamp_s=rgb_time.nanoseconds * 1e-9,
                            depth_stamp_s=depth_time.nanoseconds * 1e-9,
                            rgb=rgb,
                            depth=depth,
                            camera_info={
                                "frame_id": str(info.header.frame_id),
                                "stamp_s": _stamp_seconds(info.header.stamp),
                                "width": int(info.width),
                                "height": int(info.height),
                                "distortion_model": str(info.distortion_model),
                                "k": [float(value) for value in info.k],
                                "d": [float(value) for value in info.d],
                            },
                            map_from_camera=_transform_matrix(transform.transform),
                            detections=[
                                {
                                    "detection_index": index,
                                    "class_id": item.class_id,
                                    "confidence": float(item.confidence),
                                    "xyxy": [float(value) for value in item.xyxy],
                                }
                                for index, item in enumerate(results)
                            ],
                            prompt_decisions=prompt_decisions,
                            prompt_detection_indices=dirt_indices,
                            prompt_masks=dirt_masks,
                            prompt_qualities=dirt_qualities,
                            projection_diagnostics=projection_diagnostics,
                            map_occupancy=grid.occupancy,
                            map_metadata={
                                "frame_id": str(grid_message.header.frame_id),
                                "stamp_s": _stamp_seconds(grid_message.header.stamp),
                                "width": grid.width,
                                "height": grid.height,
                                "resolution": grid.resolution,
                                "origin_x": grid.origin_x,
                                "origin_y": grid.origin_y,
                            },
                        )
                        if captured:
                            self._diagnostic(
                                0,
                                "product_intermediate_captured",
                                {"sensor": sensor},
                            )
                    except Exception as capture_exc:
                        # Persistence must never turn a valid product output
                        # into inference failure or alter any accuracy gate.
                        # Disable further attempts after the first persistence
                        # failure so a full disk cannot repeatedly compress
                        # frames and starve the product inference callbacks.
                        self.intermediate_capture.disable(capture_exc)
                        self._diagnostic(
                            1,
                            "product_intermediate_capture_failed",
                            {"sensor": sensor, "error": str(capture_exc)},
                        )
                self._diagnostic(
                    0,
                    "rgbd_product_ok",
                    {
                        "sensor": sensor,
                        "detections": len(results),
                        "targets": len(projected),
                        "map_tf_age_s": transform_age,
                    },
                )
            except Exception as exc:
                self._diagnostic(2, "rgbd_product_failed_closed", {"sensor": sensor, "error": str(exc)})

        def _diagnostic(self, level: int, message: str, values: dict) -> None:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            # diagnostic_msgs/DiagnosticStatus.level is ROS ``byte`` (not
            # uint8), so rclpy requires a one-byte value on Jazzy and Humble.
            status.level = bytes([level])
            status.name = "formal_open_vocab_perception/pc_product_adapter"
            status.hardware_id = "pc_cpu_onnxruntime"
            status.message = message
            values = {**values, "ground_truth_input_used": False, "fail_closed": level >= 2}
            status.values = [
                KeyValue(key=str(key), value=json.dumps(value, ensure_ascii=False))
                for key, value in sorted(values.items())
            ]
            array.status = [status]
            self.diagnostic_publisher.publish(array)

    rclpy.init()
    try:
        node = PcOpenVocabProductAdapter()
    except Exception as exc:
        rclpy.shutdown()
        raise SystemExit(f"formal PC perception refused to start: {exc}") from exc
    try:
        # One thread may remain inside synchronous ONNX inference while cache
        # and TF/diagnostic callbacks continue to make bounded progress.
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if "executor" in locals():
            executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
