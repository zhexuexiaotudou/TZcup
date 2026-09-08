"""RDK S100P DOSOD + EdgeSAM product adapter.

The official BPU nodes remain the inference owners. This node validates their
``ai_msgs`` results, filters the frozen ground-dirt prompts, and is the only
publisher of planning-facing observations. Missing or stale RGB-D/map/TF
context never becomes an empty-clean observation; the node emits an ERROR
diagnostic and withholds that product output.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

import numpy as np

from .product_projection import CameraIntrinsics, PublicGrid, project_rgbd_observation
from .diagnostic_compat import set_diagnostic_level
from .s100p_product_adapter_core import (
    Detection,
    EdgeSamPromptBatch,
    ExactStampRgbdCache,
    PendingDosodCache,
    PendingEdgeSamCache,
    S100PProductAdapterError,
    decode_edgesam_label_features,
    detections_from_ai_like,
    filter_s100p_product_detections,
    ground_dirt_prompt_batch,
    load_verified_board_artifact_contract,
    requires_edgesam_handoff,
    validate_dosod_source_rois,
    validate_exact_rgbd_projection_binding,
    validate_exact_tf_binding,
    validate_public_map_binding,
)
from .rgb_to_nv12_adapter import (
    S100P_SOURCE_HEIGHT,
    S100P_SOURCE_WIDTH,
)
from .tracking import TargetTracker


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _ai_like_targets(message: Any) -> list[dict[str, Any]]:
    return [
        {
            "type": str(target.type),
            "rois": [
                {
                    "type": str(roi.type),
                    "confidence": float(roi.confidence),
                    "rect": {
                        "x_offset": int(roi.rect.x_offset),
                        "y_offset": int(roi.rect.y_offset),
                        "width": int(roi.rect.width),
                        "height": int(roi.rect.height),
                    },
                }
                for roi in target.rois
            ],
        }
        for target in message.targets
    ]


def _perf_latency_ms(message: Any) -> float | None:
    values = [
        float(row.time_ms_duration)
        for row in message.perfs
        if str(row.type).strip().lower().endswith("_predict_infer")
    ]
    positive = [value for value in values if math.isfinite(value) and value > 0.0]
    return max(positive) if positive else None


def _transform_matrix(transform: Any) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    norm = x * x + y * y + z * z + w * w
    if norm <= 1.0e-12 or not math.isfinite(norm):
        raise ValueError("invalid map TF quaternion")
    scale = 2.0 / norm
    return np.asarray(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w), translation.x],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w), translation.y],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y), translation.z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _resize_mask(
    mask: tuple[bool, ...],
    source_width: int,
    source_height: int,
    width: int,
    height: int,
) -> np.ndarray:
    source = np.asarray(mask, dtype=bool).reshape(source_height, source_width)
    y_index = np.minimum(
        source_height - 1,
        np.floor(np.arange(height) * source_height / height).astype(np.int64),
    )
    x_index = np.minimum(
        source_width - 1,
        np.floor(np.arange(width) * source_width / width).astype(np.int64),
    )
    return source[y_index[:, None], x_index[None, :]]


def main() -> None:
    import rclpy
    from ai_msgs.msg import PerceptionTargets, Roi as AiRoi, Target as AiTarget
    from cv_bridge import CvBridge
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import OccupancyGrid
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
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
    from tf2_ros import Buffer, TransformListener
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    class S100PProductAdapter(Node):
        def __init__(self) -> None:
            super().__init__("open_vocab_product_adapter")
            defaults = {
                "dosod_raw_topic": "/perception/open_vocab/dosod_raw",
                "edgesam_prompts_topic": "/perception/open_vocab/edgesam_prompts",
                "edgesam_raw_topic": "/perception/open_vocab/edgesam_raw",
                "front_rgb_topic": "/sensors/front_rgbd/depth/image_rect_raw/image",
                "front_depth_topic": "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
                "front_camera_info_topic": "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
                "map_topic": "/map",
                "product_detections_topic": "/perception/garbage/detections_2d",
                "product_boxes_topic": "/perception/open_vocab/dosod_boxes",
                "product_masks_topic": "/perception/ground_dirt/masks",
                "product_targets_topic": "/perception/garbage/targets",
                "wrist_recheck_topic": "/perception/wrist/grasp_recheck",
                "diagnostics_topic": "/perception/open_vocab/diagnostics",
                "map_frame": "map",
                "dosod_model_path": "",
                "dosod_vocabulary_path": "",
                "edgesam_encoder_model_path": "",
                "edgesam_decoder_model_path": "",
                "artifact_manifest_path": "",
                "artifact_mode": "formal",
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.declare_parameter("tf_max_age_s", 0.75)
            self.declare_parameter("sample_stride", 4)
            self.declare_parameter("pending_frame_limit", 32)
            self.declare_parameter("pending_dosod_max_age_s", 3.0)
            self.declare_parameter("rgb_frame_cache_limit", 96)
            self.declare_parameter("edgesam_capture_width", 512)
            self.declare_parameter("edgesam_capture_height", 288)

            self._artifact_mode = str(self.get_parameter("artifact_mode").value)
            artifact_kwargs = {
                "artifact_manifest_path": str(self.get_parameter("artifact_manifest_path").value),
                "artifact_paths": {
                    "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm": str(self.get_parameter("dosod_model_path").value),
                    "dosod/tzcup_offline_vocabulary.json": str(self.get_parameter("dosod_vocabulary_path").value),
                    "edgesam/edgesam_encoder_512.hbm": str(self.get_parameter("edgesam_encoder_model_path").value),
                    "edgesam/edgesam_decoder_512.hbm": str(self.get_parameter("edgesam_decoder_model_path").value),
                },
            }
            if self._artifact_mode == "formal":
                artifact_contract = load_verified_board_artifact_contract(**artifact_kwargs)
            elif self._artifact_mode == "development":
                from .s100p_development_artifact_contract import (
                    load_verified_development_artifact_contract,
                )
                artifact_contract = load_verified_development_artifact_contract(**artifact_kwargs)
            else:
                raise S100PProductAdapterError("artifact_mode must be exactly 'formal' or 'development'")
            hashes = artifact_contract.model_hashes
            self._model_hashes = {
                "dosod": hashes["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"],
                "vocabulary": hashes["dosod/tzcup_offline_vocabulary.json"],
                "edgesam_encoder": hashes["edgesam/edgesam_encoder_512.hbm"],
                "edgesam_decoder": hashes["edgesam/edgesam_decoder_512.hbm"],
            }
            self._dosod_emitted_label_map = artifact_contract.emitted_label_to_class_id
            self._artifact_classification = (
                "NON_FORMAL_ABI_DEVELOPMENT"
                if self._artifact_mode == "development"
                else "FORMAL_BOARD_ARTIFACT_CONTRACT"
            )
            self._bridge = CvBridge()
            self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._source_cache = ExactStampRgbdCache(
                int(self.get_parameter("rgb_frame_cache_limit").value)
            )
            self._pending_dosod = PendingDosodCache(
                int(self.get_parameter("pending_frame_limit").value),
                int(float(self.get_parameter("pending_dosod_max_age_s").value) * 1_000_000_000),
            )
            self._pending = PendingEdgeSamCache(
                int(self.get_parameter("pending_frame_limit").value)
            )
            self._last_cache_reject_ns: dict[str, int] = {}
            self._map: OccupancyGrid | None = None
            self._tracker = TargetTracker(
                confirmation_observations=2,
                association_distance_m=0.20,
                maximum_covariance_trace=0.15,
                lost_timeout_s=3600.0,
            )
            self._last_success_diagnostic_ns = 0
            self._dosod_raw_frames = self._product_box_frames = self._product_target_frames = 0
            self._reject_reasons: dict[str, int] = {}

            transient = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            topic = lambda name: str(self.get_parameter(name).value)
            self._prompt_publisher = self.create_publisher(
                PerceptionTargets, topic("edgesam_prompts_topic"), 10
            )
            self._detection_publisher = self.create_publisher(
                Detection2DArray, topic("product_detections_topic"), 10
            )
            self._box_publisher = self.create_publisher(
                Detection2DArray, topic("product_boxes_topic"), 10
            )
            self._mask_publisher = self.create_publisher(
                Image, topic("product_masks_topic"), 10
            )
            self._target_publisher = self.create_publisher(
                GarbageTargetArray, topic("product_targets_topic"), 10
            )
            self._wrist_publisher = self.create_publisher(
                String, topic("wrist_recheck_topic"), 10
            )
            self._diagnostic_publisher = self.create_publisher(
                DiagnosticArray, topic("diagnostics_topic"), transient
            )
            self.create_subscription(
                Image, topic("front_rgb_topic"), self._on_rgb, qos_profile_sensor_data
            )
            self.create_subscription(
                Image,
                topic("front_depth_topic"),
                self._on_depth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                topic("front_camera_info_topic"),
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                OccupancyGrid, topic("map_topic"), self._on_map, transient
            )
            self.create_subscription(
                PerceptionTargets, topic("dosod_raw_topic"), self._on_dosod, 10
            )
            self.create_subscription(
                PerceptionTargets, topic("edgesam_raw_topic"), self._on_edgesam, 10
            )
            self.create_timer(
                0.5,
                self._expire_pending_dosod,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
            self._adapter_diagnostic(
                1 if self._artifact_mode == "development" else 0,
                "development_artifact_mode_non_formal_ready_waiting_for_real_board_inputs"
                if self._artifact_mode == "development"
                else "ready_waiting_for_real_board_inputs",
                {},
            )

        def _on_rgb(self, message: Image) -> None:
            stamp = _stamp_ns(message.header.stamp)
            if (int(message.width), int(message.height)) != (S100P_SOURCE_WIDTH, S100P_SOURCE_HEIGHT) or not message.header.frame_id:
                self._adapter_diagnostic(2, "selected_rgb_shape_or_frame_invalid_fail_closed", {"source_stamp_ns": stamp})
                return
            self._cache_source("rgb", stamp, message)

        def _on_depth(self, message: Image) -> None:
            stamp = _stamp_ns(message.header.stamp)
            self._cache_source("depth", stamp, message)

        def _on_camera_info(self, message: CameraInfo) -> None:
            stamp = _stamp_ns(message.header.stamp)
            self._cache_source("camera_info", stamp, message)

        def _record_reject(self, reason: str) -> None:
            self._reject_reasons[reason] = self._reject_reasons.get(reason, 0) + 1

        def _cache_source(self, kind: str, stamp: int, message: Any) -> None:
            try:
                if kind == "rgb":
                    self._source_cache.put_rgb(stamp, message)
                elif kind == "depth":
                    self._source_cache.put_depth(stamp, message)
                else:
                    self._source_cache.put_camera_info(stamp, message)
            except S100PProductAdapterError as exc:
                self._record_reject("source_stamp_replayed")
                now = time.monotonic_ns()
                previous = self._last_cache_reject_ns.get(kind, 0)
                if now - previous >= 1_000_000_000:
                    self._last_cache_reject_ns[kind] = now
                    self._adapter_diagnostic(
                        2, "source_stamp_replayed_fail_closed",
                        {"source_stamp_ns": stamp, "source_kind": kind, "error": str(exc)},
                    )
                return
            self._try_exact_join(stamp)

        def _try_exact_join(self, stamp: int) -> None:
            self._expire_pending_dosod()
            joined = self._pending_dosod.take_if_ready(stamp, self._source_cache)
            if joined is not None:
                message, (image, depth_message, camera_info) = joined
                self._process_dosod(message, image, depth_message, camera_info)

        def _expire_pending_dosod(self) -> None:
            for stale_stamp in self._pending_dosod.expire(time.monotonic_ns()):
                self._record_reject("dosod_wait_expired")
                self._adapter_diagnostic(
                    2, "dosod_exact_join_expired_fail_closed", {"source_stamp_ns": stale_stamp}
                )

        def _on_map(self, message: OccupancyGrid) -> None:
            origin = message.info.origin
            try:
                validate_public_map_binding(
                    frame_id=str(message.header.frame_id),
                    expected_frame_id=str(self.get_parameter("map_frame").value),
                    width=int(message.info.width),
                    height=int(message.info.height),
                    resolution=float(message.info.resolution),
                    origin_values=(
                        origin.position.x,
                        origin.position.y,
                        origin.position.z,
                        origin.orientation.x,
                        origin.orientation.y,
                        origin.orientation.z,
                        origin.orientation.w,
                    ),
                    data_length=len(message.data),
                )
            except S100PProductAdapterError as exc:
                self._map = None
                self._adapter_diagnostic(2, "public_map_invalid_fail_closed", {"error": str(exc)})
                return
            self._map = message

        def _detection_message(
            self, header: Any, detections: tuple[Detection, ...]
        ) -> Detection2DArray:
            output = Detection2DArray()
            output.header = header
            for row in detections:
                item = Detection2D()
                item.header = header
                item.bbox.center.position.x = row.roi.x_offset + row.roi.width / 2.0
                item.bbox.center.position.y = row.roi.y_offset + row.roi.height / 2.0
                item.bbox.size_x = row.roi.width
                item.bbox.size_y = row.roi.height
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = row.class_id
                hypothesis.hypothesis.score = row.confidence
                item.results.append(hypothesis)
                output.detections.append(item)
            return output

        def _prompt_message(
            self, source: PerceptionTargets, batch: EdgeSamPromptBatch
        ) -> PerceptionTargets:
            output = PerceptionTargets()
            output.header = source.header
            output.fps = source.fps
            output.perfs = source.perfs
            for prompt in batch.prompts:
                target = AiTarget()
                target.type = prompt.class_id
                roi = AiRoi()
                roi.type = prompt.class_id
                roi.rect.x_offset = int(round(prompt.roi.x_offset))
                roi.rect.y_offset = int(round(prompt.roi.y_offset))
                roi.rect.width = int(round(prompt.roi.width))
                roi.rect.height = int(round(prompt.roi.height))
                roi.confidence = float(prompt.confidence)
                target.rois.append(roi)
                output.targets.append(target)
            return output

        def _on_dosod(self, message: PerceptionTargets) -> None:
            latency = _perf_latency_ms(message)
            self._inference_diagnostic("dosod", latency, latency is not None)
            stamp = _stamp_ns(message.header.stamp)
            self._dosod_raw_frames += 1
            try:
                evicted = self._pending_dosod.put(stamp, message, now_ns=time.monotonic_ns())
                for stale_stamp in evicted:
                    self._record_reject("dosod_wait_capacity_evicted")
                    self._adapter_diagnostic(
                        2, "dosod_exact_join_capacity_evicted_fail_closed", {"source_stamp_ns": stale_stamp}
                    )
                self._try_exact_join(stamp)
            except Exception as exc:
                self._record_reject("dosod_wait_rejected")
                self._adapter_diagnostic(
                    2,
                    "dosod_exact_join_rejected_fail_closed",
                    {"source_stamp_ns": stamp, "error": str(exc)},
                )

        def _process_dosod(
            self, message: PerceptionTargets, image: Image, depth_message: Image, camera_info: CameraInfo
        ) -> None:
            stamp = _stamp_ns(message.header.stamp)
            try:
                validate_exact_rgbd_projection_binding(
                    rgb_stamp_ns=stamp,
                    rgb_frame_id=str(image.header.frame_id),
                    rgb_width=int(image.width),
                    rgb_height=int(image.height),
                    depth_stamp_ns=_stamp_ns(depth_message.header.stamp),
                    depth_frame_id=str(depth_message.header.frame_id),
                    depth_width=int(depth_message.width),
                    depth_height=int(depth_message.height),
                    depth_encoding=str(depth_message.encoding),
                    camera_info_stamp_ns=_stamp_ns(camera_info.header.stamp),
                    camera_info_frame_id=str(camera_info.header.frame_id),
                    camera_info_width=int(camera_info.width),
                    camera_info_height=int(camera_info.height),
                    camera_k=camera_info.k,
                )
                detections = detections_from_ai_like(
                    _ai_like_targets(message),
                    emitted_label_to_class_id=self._dosod_emitted_label_map,
                )
                detections = filter_s100p_product_detections(detections)
                detections = validate_dosod_source_rois(
                    detections,
                    source_width=S100P_SOURCE_WIDTH,
                    source_height=S100P_SOURCE_HEIGHT,
                )
                product = self._detection_message(image.header, detections)
                self._box_publisher.publish(product)
                self._detection_publisher.publish(product)
                batch = ground_dirt_prompt_batch(
                    detections,
                    stamp_ns=stamp,
                    image_width=int(image.width),
                    image_height=int(image.height),
                )
                pending = {
                    "detections": detections,
                    "batch": batch,
                    "image": image,
                    "depth": depth_message,
                    "camera_info": camera_info,
                }
                if requires_edgesam_handoff(batch):
                    self._pending.put(stamp, pending)
                    self._prompt_publisher.publish(self._prompt_message(message, batch))
                else:
                    self._adapter_diagnostic(
                        0, "dosod_no_edgesam_prompt_terminal", {"source_stamp_ns": stamp}
                    )
                self._adapter_diagnostic(
                    0,
                    "dosod_product_and_prompts_published",
                    {
                        "source_stamp_ns": stamp,
                        "detections": len(detections),
                        "prompt_count": len(batch.prompts),
                    },
                )
                self._product_box_frames += 1
                if any(row.class_id == "litter_cube" for row in detections):
                    try:
                        self._publish_projected_products(
                            pending,
                            None,
                            publish_mask=False,
                            publish_targets=True,
                        )
                    except Exception as exc:
                        self._adapter_diagnostic(
                            2,
                            "litter_product_failed_closed",
                            {"source_stamp_ns": stamp, "error": str(exc)},
                        )
            except Exception as exc:
                self._record_reject("dosod_processing_failed")
                self._adapter_diagnostic(
                    2,
                    "dosod_product_failed_closed",
                    {"source_stamp_ns": stamp, "error": str(exc)},
                )

        def _edgesam_capture(
            self, message: PerceptionTargets
        ) -> tuple[Any, list[Any]]:
            capture_targets = [
                row for row in message.targets if str(row.type) == "parking_space"
            ]
            prompt_targets = [
                row for row in message.targets if str(row.type) != "parking_space"
            ]
            if len(capture_targets) != 1 or len(capture_targets[0].captures) != 1:
                raise S100PProductAdapterError(
                    "EdgeSAM output must contain exactly one label capture"
                )
            capture = capture_targets[0].captures[0]
            label_counts = [
                int(round(float(row.value)))
                for row in capture_targets[0].attributes
                if str(row.type) == "segmentation_label_count"
            ]
            if label_counts != [len(prompt_targets) + 1]:
                raise S100PProductAdapterError(
                    "EdgeSAM segmentation label count is inconsistent"
                )
            return capture, prompt_targets

        def _on_edgesam(self, message: PerceptionTargets) -> None:
            latency = _perf_latency_ms(message)
            self._inference_diagnostic("edgesam", latency, latency is not None)
            stamp = _stamp_ns(message.header.stamp)
            try:
                if stamp <= 0:
                    raise S100PProductAdapterError("EdgeSAM stamp is invalid")
                pending = self._pending.consume(stamp)
            except S100PProductAdapterError as exc:
                self._record_reject("edgesam_unmatched_or_replayed")
                self._adapter_diagnostic(
                    2,
                    "edgesam_prompt_stamp_unmatched_fail_closed",
                    {"source_stamp_ns": stamp, "error": str(exc)},
                )
                return
            try:
                capture, prompt_targets = self._edgesam_capture(message)
                prompt_rois = []
                prompt_classes = []
                for target in prompt_targets:
                    if len(target.rois) != 1:
                        raise S100PProductAdapterError(
                            "EdgeSAM prompt result must contain one ROI"
                        )
                    roi = target.rois[0]
                    prompt_classes.append(str(target.type))
                    prompt_rois.append(
                        {
                            "type": str(roi.type),
                            "confidence": float(roi.confidence),
                            "rect": {
                                "x_offset": int(roi.rect.x_offset),
                                "y_offset": int(roi.rect.y_offset),
                                "width": int(roi.rect.width),
                                "height": int(roi.rect.height),
                            },
                        }
                    )
                decoded = decode_edgesam_label_features(
                    pending["batch"],
                    output_stamp_ns=stamp,
                    feature_values=capture.features,
                    capture_width=int(capture.img.width),
                    capture_height=int(capture.img.height),
                    expected_capture_width=int(
                        self.get_parameter("edgesam_capture_width").value
                    ),
                    expected_capture_height=int(
                        self.get_parameter("edgesam_capture_height").value
                    ),
                    output_prompt_rois=prompt_rois,
                    output_prompt_class_ids=prompt_classes,
                )
                self._publish_projected_products(
                    pending, decoded, publish_mask=True, publish_targets=False
                )
            except Exception as exc:
                self._record_reject("edgesam_terminal_failure")
                self._adapter_diagnostic(
                    2,
                    "edgesam_product_failed_closed",
                    {"source_stamp_ns": stamp, "error": str(exc)},
                )

        def _publish_projected_products(
            self,
            pending: dict[str, Any],
            decoded: Any | None,
            *,
            publish_mask: bool,
            publish_targets: bool,
        ) -> None:
            image = pending["image"]
            depth_message = pending["depth"]
            info = pending["camera_info"]
            grid_message = self._map
            if depth_message is None or info is None or grid_message is None:
                raise S100PProductAdapterError(
                    "RGB-D, CameraInfo or public map is missing"
                )
            image_stamp = _stamp_ns(image.header.stamp)
            validate_exact_rgbd_projection_binding(
                rgb_stamp_ns=image_stamp,
                rgb_frame_id=str(image.header.frame_id),
                rgb_width=int(image.width),
                rgb_height=int(image.height),
                depth_stamp_ns=_stamp_ns(depth_message.header.stamp),
                depth_frame_id=str(depth_message.header.frame_id),
                depth_width=int(depth_message.width),
                depth_height=int(depth_message.height),
                depth_encoding=str(depth_message.encoding),
                camera_info_stamp_ns=_stamp_ns(info.header.stamp),
                camera_info_frame_id=str(info.header.frame_id),
                camera_info_width=int(info.width),
                camera_info_height=int(info.height),
                camera_k=info.k,
            )
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(image.header.frame_id),
                Time.from_msg(image.header.stamp),
                timeout=Duration(seconds=0.20),
            )
            validate_exact_tf_binding(
                image_stamp_ns=image_stamp,
                transform_stamp_ns=_stamp_ns(transform.header.stamp),
                max_age_s=float(self.get_parameter("tf_max_age_s").value),
            )
            depth = self._bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
            if depth.shape != (int(image.height), int(image.width)):
                raise S100PProductAdapterError("RGB and depth dimensions differ")
            origin = grid_message.info.origin
            if (
                any(
                    abs(value) > 1.0e-6
                    for value in (
                        origin.orientation.x,
                        origin.orientation.y,
                        origin.orientation.z,
                    )
                )
                or abs(origin.orientation.w - 1.0) > 1.0e-6
            ):
                raise S100PProductAdapterError(
                    "rotated public occupancy-grid origin is unsupported"
                )

            detections: tuple[Detection, ...] = pending["detections"]
            dirt_masks = (
                {
                    prompt.source_index: _resize_mask(
                        mask,
                        decoded.image_width,
                        decoded.image_height,
                        int(image.width),
                        int(image.height),
                    )
                    for prompt, mask in zip(decoded.prompts, decoded.masks)
                }
                if decoded is not None
                else {}
            )
            masks = []
            for detection in detections:
                if detection.source_index in dirt_masks:
                    masks.append(dirt_masks[detection.source_index])
                    continue
                mask = np.zeros((int(image.height), int(image.width)), dtype=bool)
                x1, y1, x2, y2 = detection.roi.xyxy
                mask[
                    max(0, int(y1)) : min(int(image.height), int(math.ceil(y2))),
                    max(0, int(x1)) : min(int(image.width), int(math.ceil(x2))),
                ] = True
                masks.append(mask)
            boxes = np.asarray(
                [row.roi.xyxy for row in detections], dtype=np.float32
            ).reshape(-1, 4)
            grid = PublicGrid(
                width=int(grid_message.info.width),
                height=int(grid_message.info.height),
                resolution=float(grid_message.info.resolution),
                origin_x=float(origin.position.x),
                origin_y=float(origin.position.y),
            )
            raster, projected = project_rgbd_observation(
                depth,
                CameraIntrinsics(
                    float(info.k[0]),
                    float(info.k[4]),
                    float(info.k[2]),
                    float(info.k[5]),
                ),
                _transform_matrix(transform.transform),
                grid,
                boxes_xyxy=boxes,
                class_ids=[row.class_id for row in detections],
                masks=masks,
                confidences=[row.confidence for row in detections],
                sample_stride=int(self.get_parameter("sample_stride").value),
            )
            published_mask_cells = 0
            if publish_mask:
                mask_message = self._bridge.cv2_to_imgmsg(raster, encoding="mono8")
                mask_message.header.stamp = image.header.stamp
                mask_message.header.frame_id = str(self.get_parameter("map_frame").value)
                self._mask_publisher.publish(mask_message)
                published_mask_cells = int(np.count_nonzero(raster >= 2))
            target_count = 0
            if publish_targets:
                targets = GarbageTargetArray()
                targets.header.stamp = image.header.stamp
                targets.header.frame_id = str(self.get_parameter("map_frame").value)
                targets.registry_sha256 = self._model_hashes["vocabulary"]
                tracker_input = []
                for row in projected:
                    detection = detections[row.detection_index]
                    tracker_input.append(
                        {
                            "class_id": "litter_cube",
                            "target_type": "discrete",
                            "cleaning_policy": "pick_and_bin",
                            "x_m": row.xyz[0],
                            "y_m": row.xyz[1],
                            "confidence": detection.confidence,
                            "covariance_trace": 0.12,
                            "source_backend": "dosod_s100p_rgbd",
                        }
                    )
                for tracked in self._tracker.update(tracker_input):
                    if tracked.state not in {
                        "CONFIRMED",
                        "QUEUED",
                        "APPROACHING",
                        "CLEANING",
                    }:
                        continue
                    target = GarbageTarget()
                    target.header = targets.header
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
                    target.first_seen = image.header.stamp
                    target.last_seen = image.header.stamp
                    target.source_stamp = image.header.stamp
                    target.observation_count = tracked.observation_count
                    target.track_state = tracked.state
                    target.cleaning_policy = "pick_and_bin"
                    target.source_backend = tracked.source_backend
                    target.visibility = 1.0
                    targets.targets.append(target)
                self._target_publisher.publish(targets)
                target_count = len(targets.targets)
                self._product_target_frames += 1
            self._adapter_diagnostic(
                0,
                "map_products_published",
                {
                    "source_stamp_ns": image_stamp,
                    "mask_published": publish_mask,
                    "published_mask_cells": published_mask_cells,
                    "targets_published": publish_targets,
                    "targets": target_count,
                },
            )

        @staticmethod
        def _value(value: Any) -> str:
            if isinstance(value, bool):
                return "true" if value else "false"
            if isinstance(value, (dict, list, tuple)):
                return json.dumps(value, ensure_ascii=False, sort_keys=True)
            return str(value)

        def _publish_diagnostic(
            self,
            name: str,
            level: int,
            message: str,
            values: dict[str, Any],
        ) -> None:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            set_diagnostic_level(status, level)
            status.name = name
            status.hardware_id = "RDK_S100P_Journey_6P"
            status.message = message
            status.values = [
                KeyValue(key=str(key), value=self._value(value))
                for key, value in sorted(values.items())
            ]
            array.status = [status]
            self._diagnostic_publisher.publish(array)

        def _adapter_diagnostic(
            self, level: int, message: str, values: dict[str, Any]
        ) -> None:
            stamp = values.get("source_stamp_ns")
            if level == 0 and isinstance(stamp, int):
                if stamp - self._last_success_diagnostic_ns < 1_000_000_000:
                    return
                self._last_success_diagnostic_ns = stamp
            self._publish_diagnostic(
                "formal_open_vocab_perception/product_adapter",
                level,
                message,
                {
                    **values,
                    "dosod_raw_frames": self._dosod_raw_frames,
                    "product_box_frames": self._product_box_frames,
                    "product_target_frames": self._product_target_frames,
                    "reject_reasons": self._reject_reasons,
                    "fail_closed": level >= 2,
                    "ground_truth_input_used": False,
                    "artifact_mode": self._artifact_mode,
                    "artifact_classification": self._artifact_classification,
                    "semantic_or_board_acceptance": False if self._artifact_mode == "development" else "not_claimed",
                },
            )

        def _inference_diagnostic(
            self, component: str, latency: float | None, ok: bool
        ) -> None:
            hashes = (
                self._model_hashes["dosod"]
                if component == "dosod"
                else f'{self._model_hashes["edgesam_encoder"]},{self._model_hashes["edgesam_decoder"]}'
            )
            self._publish_diagnostic(
                f"formal_open_vocab_perception/{'hobot_dosod' if component == 'dosod' else 'mono_edgesam'}",
                0 if ok else 2,
                "inference_ok"
                if ok
                else "inference_telemetry_missing_fail_closed",
                {
                    "backend": "bpu",
                    "model_sha256": hashes,
                    "vocabulary_sha256": (
                        self._model_hashes["vocabulary"]
                        if component == "dosod"
                        else ""
                    ),
                    "latency_ms": latency if latency is not None else "",
                    "inference_ok": ok,
                },
            )

    rclpy.init()
    node: S100PProductAdapter | None = None
    try:
        node = S100PProductAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
