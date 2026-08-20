"""ROS 2 lifecycle shell for the fail-closed product perception pipeline.

ROS imports are intentionally local so the contract remains unit-testable on
developer hosts without a ROS installation. Model/postprocess execution is
enabled only by a frozen manifest declaring the supported runtime contract.
"""

from __future__ import annotations

import json
import importlib
import math
from pathlib import Path
import time

from sanitation_perception.action_verifier import (
    ActionVerifierConfig,
    ActionVerdict,
    ProductActionVerifier,
)
from sanitation_perception.camera_frustum_model import CameraFrustumModel
from sanitation_perception.dynamic_trash_map import (
    DynamicTrashMap,
    DynamicTrashMapConfig,
)
from sanitation_perception.frame_synchronizer import (
    LatestFrameScheduler,
    StrictFrameSynchronizer,
)
from sanitation_perception.grid_safety import keepout_clear
from sanitation_perception.lifecycle_health import ProductHealth, WatchdogConfig
from sanitation_perception.observation_model import (
    MapPoseMeasurement,
    TargetObservation,
)
from sanitation_perception.pipeline_manifest import load_pipeline_manifest
from sanitation_perception.trash_map_messages import TargetState


SUPPORTED_RUNTIME_CONTRACT = "fcos_classifier_area_v1"
AREA_CLASS_NAMES = {"leaf_pile", "puddle"}
_CLEANING_EVENT_STATES = {
    "scheduled": TargetState.SCHEDULED,
    "approaching": TargetState.APPROACHING,
    "pre_clean_verify": TargetState.VERIFYING,
    "cleaning": TargetState.CLEANING,
    "post_verify_pending": TargetState.POST_VERIFY,
    "cleaned": TargetState.CLEANED,
    "reclean_queued": TargetState.SCHEDULED,
    "deferred": TargetState.DEFERRED,
    "rejected": TargetState.REJECTED,
}


def load_onnxruntime(import_module=importlib.import_module):
    """Load the runtime at lifecycle configure time so absence is observable."""
    try:
        return import_module("onnxruntime")
    except ImportError as exc:
        raise RuntimeError(
            "onnxruntime unavailable; product perception remains inactive"
        ) from exc


def cleaning_event_target_state(result: str) -> TargetState:
    try:
        return _CLEANING_EVENT_STATES[str(result).strip().lower()]
    except KeyError as exc:
        raise ValueError(f"unsupported product cleaning event: {result}") from exc


def product_target_size(track, discrete_size) -> tuple[float, float, float]:
    """Preserve physical area for AREA post-clean residual verification."""
    values = track.estimated_size_m if track.target_type == "AREA" else discrete_size
    return tuple(float(value) for value in values)


def area_minimum_physical_area_m2(runtime: dict, class_name: str) -> float:
    by_class = runtime.get("minimum_area_region_m2_by_class", {})
    return float(by_class.get(class_name, runtime["minimum_area_region_m2"]))


def stamp_nanoseconds(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def validate_product_runtime_contract(pipeline: dict) -> None:
    runtime = pipeline["runtime"]
    if runtime.get("postprocess_contract") != SUPPORTED_RUNTIME_CONTRACT:
        raise RuntimeError(
            "frozen pipeline does not declare supported postprocess contract "
            f"{SUPPORTED_RUNTIME_CONTRACT}"
        )
    if runtime.get("required_provider") != "CUDAExecutionProvider":
        raise RuntimeError("product x86 runtime requires CUDAExecutionProvider")
    if runtime.get("io_binding_required") is not True:
        raise RuntimeError("product runtime requires ONNX Runtime I/O binding")
    if runtime.get("cpu_fallback_forbidden") is not True:
        raise RuntimeError("silent CPU fallback must remain forbidden")
    if not 1 <= int(runtime.get("maximum_candidates", 0)) <= 100:
        raise RuntimeError("maximum_candidates must be in [1, 100]")
    if not 0.0 < float(runtime.get("minimum_valid_depth_ratio", 0.0)) <= 1.0:
        raise RuntimeError("minimum_valid_depth_ratio must be in (0, 1]")
    if int(runtime.get("minimum_area_region_pixels", 0)) < 3:
        raise RuntimeError("minimum_area_region_pixels must be at least 3")
    if float(runtime.get("minimum_area_region_m2", 0.0)) <= 0.0:
        raise RuntimeError("minimum_area_region_m2 must be positive")
    by_class = runtime.get("minimum_area_region_m2_by_class")
    if by_class is not None:
        if set(by_class) != AREA_CLASS_NAMES:
            raise RuntimeError(
                "minimum_area_region_m2_by_class must define leaf_pile and puddle"
            )
        if any(float(value) <= 0.0 for value in by_class.values()):
            raise RuntimeError(
                "minimum_area_region_m2_by_class values must be positive"
            )
    if float(runtime.get("minimum_rgb_stddev", 0.0)) <= 0.0:
        raise RuntimeError("minimum_rgb_stddev must be positive")
    saturated = float(runtime.get("maximum_dark_or_saturated_fraction", 0.0))
    if not 0.0 < saturated < 1.0:
        raise RuntimeError("maximum_dark_or_saturated_fraction must be in (0, 1)")
    DynamicTrashMapConfig(**runtime["dynamic_trash_map"]).validate()
    ActionVerifierConfig.from_pipeline_manifest(pipeline)
    frustum = CameraFrustumModel(**runtime["camera_frustum"])
    frustum.make_sweep(
        sweep_id="contract",
        mission_id="contract",
        stamp_ns=0,
        camera_frame_id="camera",
        image_frame_id="image",
        camera_x_m=0.0,
        camera_y_m=0.0,
        camera_yaw_rad=0.0,
    )


def optical_forward_yaw(transform_matrix) -> float:
    """Map yaw of the ROS optical +Z viewing axis, not the frame's +X axis."""
    forward_x = float(transform_matrix[0][2])
    forward_y = float(transform_matrix[1][2])
    if math.hypot(forward_x, forward_y) < 1e-9:
        raise RuntimeError("camera optical axis has no usable map-plane projection")
    return math.atan2(forward_y, forward_x)


def track_to_online_observation(
    track,
    *,
    mission_id: str,
    stamp_ns: int,
    camera_frame_id: str,
    image_frame_id: str,
    source_model: str,
) -> TargetObservation:
    return TargetObservation(
        observation_id=f"{track.uuid}:{stamp_ns}",
        mission_id=mission_id,
        stamp_ns=stamp_ns,
        camera_frame_id=camera_frame_id,
        image_frame_id=image_frame_id,
        source_model=source_model,
        source_backend=str(track.source_backend),
        target_type=str(track.target_type).upper(),
        class_probabilities=track.class_posterior,
        confidence=float(track.score_ema),
        map_pose=MapPoseMeasurement(
            x_m=float(track.x_m),
            y_m=float(track.y_m),
            z_m=float(track.z_m),
            covariance_xx=max(float(track.covariance_trace) * 0.5, 1e-9),
            covariance_yy=max(float(track.covariance_trace) * 0.5, 1e-9),
        ),
        bbox_xyxy=track.bbox_xyxy,
        polygon_xy_m=tuple(track.polygon_xy_m),
        estimated_size_m=(
            (float(track.physical_area_m2), 0.0, 0.0)
            if str(track.target_type).upper() == "AREA"
            else (0.0, 0.0, 0.0)
        ),
        in_current_fov=True,
    )


def main() -> None:
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Point32
    import message_filters
    import numpy as np
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from sanitation_perception_interfaces.msg import (
        CleaningEvent,
        GarbageTarget,
        GarbageTargetArray,
    )
    from tf2_ros import Buffer, TransformException, TransformListener

    from sanitation_perception.inference_engine import ProductInferenceEngine
    from sanitation_perception.model_registry import ProductModelRegistry
    from sanitation_perception.performance_monitor import PerformanceConfig, PerformanceMonitor
    from sanitation_perception.product_postprocess import (
        project_area_predictions,
        project_discrete_predictions,
        transform_to_matrix,
    )
    from sanitation_perception.registry import GarbageRegistry
    from sanitation_perception.tracker_v2 import ProductTrackerV2, TrackerV2Config

    default_watchdog = WatchdogConfig(
        camera_stale_ms=500.0,
        maximum_latency_ms=200.0,
        sustained_latency_samples=5,
        maximum_consecutive_tf_errors=3,
        maximum_consecutive_session_errors=2,
    )

    class ProductPerceptionNode(LifecycleNode):
        def __init__(self):
            super().__init__("product_perception")
            self.declare_parameter("pipeline_manifest", "")
            self.declare_parameter("artifact_root", "")
            self.declare_parameter("device_id", 0)
            self.declare_parameter("autostart", True)
            self.declare_parameter("mission_id", "")
            self.declare_parameter("resume_same_mission", False)
            self.declare_parameter("dynamic_map_path", "")
            self.declare_parameter("keepout_mask_topic", "/keepout_filter_mask")
            self.health = ProductHealth(default_watchdog)
            self.last_error = None
            self.pipeline = None
            self.registry = None
            self.engine = None
            self.performance = None
            self.synchronizer = None
            self.scheduler = None
            self.sensor_subscribers = []
            self.bridge = CvBridge()
            self.tracker = None
            self.garbage_registry = None
            self.registry_entries = {}
            self.last_runtime_metrics = None
            self.dynamic_map = None
            self.camera_frustum = None
            self.action_verifier = None
            self.keepout_mask = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.health_publisher = self.create_publisher(
                String, "/perception/product/health", 10
            )
            self.metrics_publisher = self.create_publisher(
                String, "/perception/product/metrics", 10
            )
            self.model_info_publisher = self.create_publisher(
                String, "/perception/product/model_info", 10
            )
            self.target_publisher = self.create_publisher(
                GarbageTargetArray, "/perception/product/targets", 10
            )
            self.observation_publisher = self.create_publisher(
                String, "/perception/product/observations", 10
            )
            self.track_publisher = self.create_publisher(
                String, "/perception/product/tracks", 10
            )
            self.dynamic_map_publisher = self.create_publisher(
                String, "/perception/product/dynamic_trash_map", 10
            )
            self.area_region_publisher = self.create_publisher(
                String, "/perception/product/area_regions", 10
            )
            self.verification_publisher = self.create_publisher(
                String, "/perception/product/action_verdicts", 20
            )
            self.reobserve_publisher = self.create_publisher(
                String, "/perception/product/reobserve_requests", 20
            )
            self.leaf_mask_publisher = self.create_publisher(
                Image, "/perception/product/leaf_mask", 10
            )
            self.puddle_mask_publisher = self.create_publisher(
                Image, "/perception/product/puddle_mask", 10
            )
            self.create_timer(0.25, self._publish_health)
            self.create_timer(0.001, self._consume_latest)
            self.autostart_timer = self.create_timer(1.0, self._autostart)
            latched_grid_qos = QoSProfile(depth=1)
            latched_grid_qos.reliability = ReliabilityPolicy.RELIABLE
            latched_grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("keepout_mask_topic").value),
                self._on_keepout_mask,
                latched_grid_qos,
            )
            self.create_subscription(
                CleaningEvent,
                "/garbage/cleaning_events",
                self._on_cleaning_event,
                20,
            )

        def _on_keepout_mask(self, message) -> None:
            self.keepout_mask = message

        def _on_cleaning_event(self, message) -> None:
            if self.dynamic_map is None:
                return
            source = str(message.source_backend).lower()
            if any(
                token in source
                for token in (
                    "ground_truth",
                    "gazebo_registry",
                    "evaluation_registry",
                )
            ):
                self.last_error = "GT control violation: cleaning event rejected"
                self.health.record_session_error()
                return
            target_uuid = str(message.target_uuid)
            if target_uuid not in self.dynamic_map.targets:
                self.last_error = f"unknown cleaning-event target: {target_uuid}"
                self.health.record_session_error()
                return
            try:
                requested = cleaning_event_target_state(message.result)
                stamp_ns = stamp_nanoseconds(message)
                if stamp_ns <= 0:
                    raise ValueError("cleaning event requires a positive timestamp")
                self.dynamic_map.transition(
                    target_uuid,
                    requested,
                    stamp_ns,
                    f"spot_cleaning_event:{str(message.result).lower()}",
                )
                dynamic_map_path = str(
                    self.get_parameter("dynamic_map_path").value
                ).strip()
                if dynamic_map_path:
                    self.dynamic_map.persist(dynamic_map_path)
                self.last_error = None
            except Exception as exc:
                self.last_error = (
                    f"cleaning event rejected: {type(exc).__name__}: {exc}"
                )
                self.health.record_session_error()

        def _autostart(self) -> None:
            self.autostart_timer.cancel()
            if not bool(self.get_parameter("autostart").value):
                return
            configured = self.trigger_configure()
            if configured and self.health.state == "INACTIVE":
                self.trigger_activate()

        def on_configure(self, _state):
            try:
                ort = load_onnxruntime()
                pipeline_path = Path(
                    str(self.get_parameter("pipeline_manifest").value)
                ).resolve()
                artifact_root = Path(
                    str(self.get_parameter("artifact_root").value)
                ).resolve()
                self.pipeline = load_pipeline_manifest(pipeline_path)
                validate_product_runtime_contract(self.pipeline)
                runtime = self.pipeline["runtime"]
                mission_id = str(self.get_parameter("mission_id").value).strip()
                if not mission_id:
                    raise RuntimeError("mission_id is required; target maps are mission-scoped")
                resume = bool(self.get_parameter("resume_same_mission").value)
                map_path = Path(str(self.get_parameter("dynamic_map_path").value)).resolve()
                if resume:
                    if not map_path.is_file():
                        raise RuntimeError("resume_same_mission requires an existing dynamic_map_path")
                    self.dynamic_map = DynamicTrashMap.resume_same_mission(map_path, mission_id)
                else:
                    self.dynamic_map = DynamicTrashMap.start_new(
                        mission_id,
                        config=DynamicTrashMapConfig(**runtime["dynamic_trash_map"]),
                    )
                self.camera_frustum = CameraFrustumModel(**runtime["camera_frustum"])
                self.health = ProductHealth(
                    WatchdogConfig.from_pipeline_manifest(self.pipeline)
                )
                self.registry = ProductModelRegistry.load(
                    pipeline_path,
                    artifact_root,
                    required_provider=runtime["required_provider"],
                    required_claim="formal",
                )
                self.engine = ProductInferenceEngine(
                    self.registry,
                    ort,
                    device_id=int(self.get_parameter("device_id").value),
                )
                self.engine.warm_up()
                self.performance = PerformanceMonitor(
                    PerformanceConfig.from_pipeline_manifest(self.pipeline)
                )
                self.tracker = ProductTrackerV2(
                    TrackerV2Config.from_pipeline_manifest(self.pipeline)
                )
                self.action_verifier = ProductActionVerifier(
                    ActionVerifierConfig.from_pipeline_manifest(self.pipeline)
                )
                share = Path(get_package_share_directory("sanitation_perception"))
                self.garbage_registry = GarbageRegistry.load(
                    share / "config" / "garbage_registry.yaml"
                )
                self.registry_entries = {
                    entry.class_id: entry
                    for entry in self.garbage_registry.entries.values()
                }
                self.synchronizer = StrictFrameSynchronizer(
                    tolerance_ms=float(runtime["sync_tolerance_ms"]),
                    queue_depth=int(runtime["frame_queue_depth"]),
                )
                self.scheduler = LatestFrameScheduler(
                    queue_depth=int(runtime["frame_queue_depth"])
                )
                topics = (
                    ("rgb", Image, "/camera/color/image_raw"),
                    ("depth", Image, "/camera/depth/image_rect_raw"),
                    ("camera_info", CameraInfo, "/camera/color/camera_info"),
                )
                for stream, message_type, topic in topics:
                    subscriber = message_filters.Subscriber(
                        self, message_type, topic, qos_profile=qos_profile_sensor_data
                    )
                    subscriber.registerCallback(
                        lambda message, stream=stream: self._receive(stream, message)
                    )
                    self.sensor_subscribers.append(subscriber)
                self.health.transition("INACTIVE", "configured_and_warmed")
                self.last_error = None
                self.model_info_publisher.publish(
                    String(data=json.dumps(self.registry.model_info(), sort_keys=True))
                )
                return TransitionCallbackReturn.SUCCESS
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                if self.health.state == "UNCONFIGURED":
                    self.health.transition("ERROR", "configure_failed")
                return TransitionCallbackReturn.FAILURE

        def on_activate(self, state):
            if self.engine is None or self.health.state != "INACTIVE":
                self.last_error = "pipeline was not configured and warmed"
                return TransitionCallbackReturn.FAILURE
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                return result
            self.health.transition("ACTIVE", "lifecycle_activated")
            return TransitionCallbackReturn.SUCCESS

        def on_deactivate(self, state):
            if self.health.state in {"ACTIVE", "DEGRADED"}:
                self.health.transition("INACTIVE", "lifecycle_deactivated")
            return super().on_deactivate(state)

        def on_cleanup(self, _state):
            self.engine = None
            self.registry = None
            self.pipeline = None
            self.tracker = None
            self.garbage_registry = None
            self.registry_entries = {}
            self.dynamic_map = None
            self.camera_frustum = None
            self.action_verifier = None
            self.sensor_subscribers.clear()
            if self.health.state == "INACTIVE":
                self.health.transition("UNCONFIGURED", "cleaned_up")
            elif self.health.state == "ERROR":
                self.health.transition("UNCONFIGURED", "error_cleaned_up")
            return TransitionCallbackReturn.SUCCESS

        def on_error(self, _state):
            if self.health.state != "ERROR":
                self.health.state = "ERROR"
                self.health.reason = "lifecycle_error"
            return TransitionCallbackReturn.SUCCESS

        def _receive(self, stream: str, message) -> None:
            if self.synchronizer is None or self.scheduler is None:
                return
            frame = self.synchronizer.add(stream, stamp_nanoseconds(message), message)
            if frame is not None:
                self.health.record_frame(time.monotonic())
                before = self.scheduler.dropped
                self.scheduler.submit(frame)
                if self.performance is not None:
                    self.performance.record_submission(
                        dropped=self.scheduler.dropped - before
                    )

        def _consume_latest(self) -> None:
            if self.health.state != "ACTIVE" or self.scheduler is None:
                return
            frame = self.scheduler.pop_latest()
            if frame is None:
                return
            try:
                started = time.perf_counter()
                # The RGB timestamp is mandatory. Looking up Time() / latest is forbidden.
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    frame.rgb.payload.header.frame_id,
                    Time.from_msg(frame.rgb.payload.header.stamp),
                )
                self.health.record_tf_success()
                rgb = self.bridge.imgmsg_to_cv2(
                    frame.rgb.payload, desired_encoding="rgb8"
                )
                depth_raw = self.bridge.imgmsg_to_cv2(
                    frame.depth.payload, desired_encoding="passthrough"
                )
                depth_m = np.asarray(depth_raw, dtype=np.float32)
                if np.issubdtype(np.asarray(depth_raw).dtype, np.integer):
                    depth_m *= 0.001
                camera_info = frame.camera_info.payload
                if camera_info.header.frame_id != frame.rgb.payload.header.frame_id:
                    raise RuntimeError("CameraInfo frame does not match RGB frame")
                camera = {
                    "width": int(camera_info.width),
                    "height": int(camera_info.height),
                    "fx": float(camera_info.k[0]),
                    "fy": float(camera_info.k[4]),
                    "cx": float(camera_info.k[2]),
                    "cy": float(camera_info.k[5]),
                    "pixel_sigma": 0.5,
                    "depth_sigma_m": 0.02,
                }
                runtime = self.pipeline["runtime"]
                result = self.engine.run_frame(
                    rgb,
                    depth_m,
                    camera,
                    maximum_candidates=int(runtime["maximum_candidates"]),
                    minimum_valid_depth_ratio=float(
                        runtime["minimum_valid_depth_ratio"]
                    ),
                    minimum_rgb_stddev=float(runtime["minimum_rgb_stddev"]),
                    maximum_dark_or_saturated_fraction=float(
                        runtime["maximum_dark_or_saturated_fraction"]
                    ),
                )
                if (
                    float(result["metrics"]["valid_depth_ratio"])
                    < float(runtime["minimum_valid_depth_ratio"])
                ):
                    raise RuntimeError("valid depth ratio violates pipeline manifest")
                transform_matrix = transform_to_matrix(transform)
                stamp_ns = stamp_nanoseconds(frame.rgb.payload)
                image_frame_id = f"{frame.rgb.payload.header.frame_id}:{stamp_ns}"
                sweep = self.camera_frustum.make_sweep(
                    sweep_id=f"sweep:{stamp_ns}",
                    mission_id=self.dynamic_map.mission_id,
                    stamp_ns=stamp_ns,
                    camera_frame_id=frame.rgb.payload.header.frame_id,
                    image_frame_id=image_frame_id,
                    camera_x_m=float(transform.transform.translation.x),
                    camera_y_m=float(transform.transform.translation.y),
                    camera_yaw_rad=optical_forward_yaw(transform_matrix),
                )
                self.dynamic_map.observed_regions.record(sweep)
                projection_started = time.perf_counter()
                detections = project_discrete_predictions(
                    result["discrete"], depth_m, camera, transform_matrix
                )
                detections.extend(
                    project_area_predictions(
                        result["areas"],
                        depth_m,
                        camera,
                        transform_matrix,
                        minimum_pixels=int(runtime["minimum_area_region_pixels"]),
                        minimum_physical_area_m2=float(
                            runtime["minimum_area_region_m2"]
                        ),
                        minimum_physical_area_m2_by_class={
                            class_name: area_minimum_physical_area_m2(
                                runtime, class_name
                            )
                            for class_name in AREA_CLASS_NAMES
                        },
                    )
                )
                projection_ms = (time.perf_counter() - projection_started) * 1000.0
                tracking_started = time.perf_counter()
                stamp_s = stamp_nanoseconds(frame.rgb.payload) / 1_000_000_000.0
                tracks = self.tracker.update(detections, stamp_s)
                online_observations = []
                for track in tracks:
                    if abs(track.last_seen_s - stamp_s) > 1e-6:
                        continue
                    observation = track_to_online_observation(
                        track,
                        mission_id=self.dynamic_map.mission_id,
                        stamp_ns=stamp_ns,
                        camera_frame_id=frame.rgb.payload.header.frame_id,
                        image_frame_id=image_frame_id,
                        source_model=str(self.pipeline["pipeline_id"]),
                    )
                    target = self.dynamic_map.ingest(observation)
                    verification = None
                    if target is not None:
                        verification = self.action_verifier.evaluate(
                            track, target, depth_valid=True
                        )
                        if target.track_state in {
                            TargetState.CANDIDATE,
                            TargetState.TRACKED,
                            TargetState.OBSERVE_AGAIN,
                            TargetState.CONFIRMED,
                            TargetState.DEFERRED,
                            TargetState.LOST,
                        }:
                            self.dynamic_map.apply_action_verdict(
                                target.uuid,
                                verification.verdict.value,
                                stamp_ns,
                                ",".join(verification.reasons) or "all_checks_passed",
                                reobserve_count=verification.reobserve_count,
                            )
                        track.state = {
                            ActionVerdict.ACCEPT: "VERIFIED",
                            ActionVerdict.OBSERVE_AGAIN: "OBSERVE_AGAIN",
                            ActionVerdict.DEFER: "DEFERRED",
                            ActionVerdict.REJECT: "REJECTED",
                        }[verification.verdict]
                        verification_record = verification.to_record()
                        verification_record["target_uuid"] = target.uuid
                        verification_record["stamp_ns"] = stamp_ns
                        self.verification_publisher.publish(String(data=json.dumps(
                            verification_record, sort_keys=True
                        )))
                        if verification.verdict == ActionVerdict.OBSERVE_AGAIN:
                            self.reobserve_publisher.publish(String(data=json.dumps({
                                "request_id": (
                                    f"{track.uuid}:reobserve:"
                                    f"{verification.reobserve_count}"
                                ),
                                "track_uuid": track.uuid,
                                "target_uuid": target.uuid,
                                "stamp_ns": stamp_ns,
                                "x_m": target.map_x_m,
                                "y_m": target.map_y_m,
                                "covariance_trace": target.covariance_trace,
                                "class_id": target.current_class,
                                "target_size_m": (
                                    math.sqrt(max(target.estimated_size_m[0], 1e-6))
                                    if target.target_type == "AREA"
                                    else min(
                                        self.registry_entries[target.current_class].size_m[:2]
                                    )
                                ),
                                "reobserve_count": verification.reobserve_count,
                                "maximum_reobserve_count": (
                                    self.action_verifier.config.maximum_reobserve_count
                                ),
                                "source_backend": "product_action_verifier",
                                "ground_truth_control_allowed": False,
                            }, sort_keys=True)))
                    online_observations.append({
                        **observation.to_record(),
                        "accepted_target_uuid": target.uuid if target else None,
                        "action_verdict": (
                            verification.verdict.value if verification else None
                        ),
                    })
                self.dynamic_map.expire(stamp_ns)
                tracking_ms = (time.perf_counter() - tracking_started) * 1000.0
                self._publish_masks(frame.rgb.payload, result["areas"])
                self._publish_online_state(online_observations, tracks)
                self._publish_targets(frame.rgb.payload, self.dynamic_map.targets.values())
                end_to_end_ms = (time.perf_counter() - started) * 1000.0
                metrics = {
                    "preprocess": result["metrics"]["preprocess_ms"],
                    "discovery": result["metrics"]["discovery_ms"],
                    "classifier_batch": result["metrics"]["classifier_batch_ms"],
                    "leaf": result["metrics"]["leaf_ms"],
                    "puddle": result["metrics"]["puddle_ms"],
                    "projection": projection_ms,
                    "tracking": tracking_ms,
                    "inference_pipeline": result["metrics"]["inference_pipeline_ms"],
                    "end_to_end": end_to_end_ms,
                }
                self.performance.record_frame(
                    metrics,
                    candidate_count=result["metrics"]["candidate_count"],
                    reject_count=result["metrics"]["rejected_candidate_count"],
                    track_count=len(tracks),
                )
                self.health.record_inference(
                    time.monotonic(), end_to_end_ms
                )
                self.last_runtime_metrics = result["metrics"]
                self.last_error = None
            except TransformException as exc:
                self.last_error = str(exc)
                self.health.record_tf_error()
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                self.health.record_session_error(oom="out of memory" in str(exc).lower())

        def _publish_masks(self, rgb_message, areas: dict) -> None:
            for task, publisher in (
                ("leaf", self.leaf_mask_publisher),
                ("puddle", self.puddle_mask_publisher),
            ):
                mask = np.asarray(areas[task]["mask"], dtype=np.uint8) * 255
                message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
                message.header = rgb_message.header
                publisher.publish(message)

        def _publish_online_state(self, observations, tracks) -> None:
            self.observation_publisher.publish(
                String(data=json.dumps(observations, sort_keys=True))
            )
            self.track_publisher.publish(String(data=json.dumps([
                {
                    "uuid": track.uuid,
                    "state": track.state,
                    "class_posterior": track.class_posterior,
                    "confidence": track.score_ema,
                    "observation_count": track.observation_count,
                    "last_seen_s": track.last_seen_s,
                }
                for track in tracks
            ], sort_keys=True)))
            snapshot = self.dynamic_map.snapshot()
            self.dynamic_map_publisher.publish(
                String(data=json.dumps(snapshot, sort_keys=True))
            )
            self.area_region_publisher.publish(String(data=json.dumps([
                target.to_record()
                for target in self.dynamic_map.targets.values()
                if target.target_type == "AREA"
            ], sort_keys=True)))

        def _publish_targets(self, rgb_message, tracks) -> None:
            message = GarbageTargetArray()
            message.header.stamp = rgb_message.header.stamp
            message.header.frame_id = "map"
            message.registry_sha256 = self.garbage_registry.sha256
            for track in tracks:
                if track.track_state not in {
                    TargetState.CONFIRMED,
                    TargetState.SCHEDULED,
                    TargetState.APPROACHING,
                    TargetState.VERIFYING,
                    TargetState.CLEANING,
                    TargetState.POST_VERIFY,
                    TargetState.DEFERRED,
                }:
                    continue
                entry = self.registry_entries.get(track.current_class)
                if entry is None:
                    continue
                target = GarbageTarget()
                target.header = message.header
                target.uuid = track.uuid
                target.class_id = track.current_class
                target.target_type = entry.target_type
                target.confidence = float(track.confidence)
                target.map_pose.pose.position.x = track.map_x_m
                target.map_pose.pose.position.y = track.map_y_m
                target.map_pose.pose.position.z = track.map_z_m
                target.map_pose.pose.orientation.w = 1.0
                target.map_pose.covariance[0] = track.covariance_xx
                target.map_pose.covariance[1] = track.covariance_xy
                target.map_pose.covariance[6] = track.covariance_xy
                target.map_pose.covariance[7] = track.covariance_yy
                if track.polygon_xy_m:
                    polygon = track.polygon_xy_m
                    xs = [point[0] for point in polygon]
                    ys = [point[1] for point in polygon]
                    size = (max(xs) - min(xs), max(ys) - min(ys), entry.size_m[2])
                else:
                    half_x, half_y = entry.size_m[0] * 0.5, entry.size_m[1] * 0.5
                    polygon = (
                        (track.map_x_m - half_x, track.map_y_m - half_y),
                        (track.map_x_m + half_x, track.map_y_m - half_y),
                        (track.map_x_m + half_x, track.map_y_m + half_y),
                        (track.map_x_m - half_x, track.map_y_m + half_y),
                    )
                    size = entry.size_m
                for x_m, y_m in polygon:
                    target.polygon.points.append(
                        Point32(x=float(x_m), y=float(y_m), z=0.0)
                    )
                # AREA targets carry physical area in size.x so a post-clean
                # observation can compute a real residual ratio. Discrete
                # targets retain their physical XYZ dimensions.
                published_size = product_target_size(track, size)
                target.size.x, target.size.y, target.size.z = (
                    float(published_size[0]),
                    float(published_size[1]),
                    float(published_size[2]),
                )
                target.first_seen = Time(
                    nanoseconds=int(track.first_seen_stamp_ns)
                ).to_msg()
                target.last_seen = Time(
                    nanoseconds=int(track.last_seen_stamp_ns)
                ).to_msg()
                target.observation_count = int(track.observation_count)
                target.track_state = track.track_state.value
                target.cleaning_policy = entry.policy
                target.source_backend = ",".join(track.source_models)
                target.source_stamp = rgb_message.header.stamp
                target.visibility = 1.0
                target.occlusion_ratio = 0.0
                # Missing/out-of-bounds/unknown masks remain fail-closed. Only
                # an explicit zero from Nav2's authoritative keepout layer is
                # eligible for scheduling.
                target.in_keepout = not keepout_clear(
                    self.keepout_mask, track.map_x_m, track.map_y_m
                )
                message.targets.append(target)
            self.target_publisher.publish(message)

        def _publish_health(self) -> None:
            now = time.monotonic()
            snapshot = self.health.snapshot(now)
            snapshot["last_error"] = self.last_error
            snapshot["dynamic_trash_map"] = (
                {
                    "mission_id": self.dynamic_map.mission_id,
                    "active_target_count": self.dynamic_map.count,
                    "preknown_target_coordinates_used": False,
                    "ground_truth_control_allowed": False,
                }
                if self.dynamic_map is not None
                else None
            )
            snapshot["sync"] = (
                {
                    "sync_count": self.synchronizer.sync_count,
                    "sync_reject_count": self.synchronizer.sync_reject_count,
                    "dropped": self.synchronizer.dropped,
                }
                if self.synchronizer is not None
                else None
            )
            snapshot["keepout_mask_received"] = self.keepout_mask is not None
            snapshot["frame_queue"] = (
                {"depth": self.scheduler.depth, "submitted": self.scheduler.submitted,
                 "consumed": self.scheduler.consumed, "dropped": self.scheduler.dropped}
                if self.scheduler is not None
                else None
            )
            self.health_publisher.publish(
                String(data=json.dumps(snapshot, sort_keys=True))
            )
            metrics = self.performance.snapshot(now) if self.performance else None
            self.metrics_publisher.publish(String(data=json.dumps(
                {"acceptance_ready": False, "health": snapshot,
                 "performance": metrics,
                 "last_runtime": self.last_runtime_metrics}, sort_keys=True)))

    rclpy.init()
    node = ProductPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
