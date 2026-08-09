"""ROS 2 lifecycle shell for the fail-closed product perception pipeline.

ROS imports are intentionally local so the contract remains unit-testable on
developer hosts without a ROS installation. Model/postprocess execution is
enabled only by a frozen manifest declaring the supported runtime contract.
"""

from __future__ import annotations

import json
from pathlib import Path
import time

from sanitation_perception.frame_synchronizer import (
    LatestFrameScheduler,
    StrictFrameSynchronizer,
)
from sanitation_perception.lifecycle_health import ProductHealth, WatchdogConfig
from sanitation_perception.pipeline_manifest import load_pipeline_manifest


SUPPORTED_RUNTIME_CONTRACT = "fcos_classifier_area_v1"


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
    if float(runtime.get("minimum_rgb_stddev", 0.0)) <= 0.0:
        raise RuntimeError("minimum_rgb_stddev must be positive")
    saturated = float(runtime.get("maximum_dark_or_saturated_fraction", 0.0))
    if not 0.0 < saturated < 1.0:
        raise RuntimeError("maximum_dark_or_saturated_fraction must be in (0, 1)")


def main() -> None:
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Point32
    import message_filters
    import numpy as np
    import onnxruntime as ort
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from sanitation_perception_interfaces.msg import GarbageTarget, GarbageTargetArray
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
            self.leaf_mask_publisher = self.create_publisher(
                Image, "/perception/product/leaf_mask", 10
            )
            self.puddle_mask_publisher = self.create_publisher(
                Image, "/perception/product/puddle_mask", 10
            )
            self.create_timer(0.25, self._publish_health)
            self.create_timer(0.001, self._consume_latest)
            self.autostart_timer = self.create_timer(1.0, self._autostart)

        def _autostart(self) -> None:
            self.autostart_timer.cancel()
            if not bool(self.get_parameter("autostart").value):
                return
            configured = self.trigger_configure()
            if configured and self.health.state == "INACTIVE":
                self.trigger_activate()

        def on_configure(self, _state):
            try:
                pipeline_path = Path(
                    str(self.get_parameter("pipeline_manifest").value)
                ).resolve()
                artifact_root = Path(
                    str(self.get_parameter("artifact_root").value)
                ).resolve()
                self.pipeline = load_pipeline_manifest(pipeline_path)
                validate_product_runtime_contract(self.pipeline)
                runtime = self.pipeline["runtime"]
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
                    )
                )
                projection_ms = (time.perf_counter() - projection_started) * 1000.0
                tracking_started = time.perf_counter()
                stamp_s = stamp_nanoseconds(frame.rgb.payload) / 1_000_000_000.0
                tracks = self.tracker.update(detections, stamp_s)
                tracking_ms = (time.perf_counter() - tracking_started) * 1000.0
                self._publish_masks(frame.rgb.payload, result["areas"])
                self._publish_targets(frame.rgb.payload, tracks)
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

        def _publish_targets(self, rgb_message, tracks) -> None:
            message = GarbageTargetArray()
            message.header.stamp = rgb_message.header.stamp
            message.header.frame_id = "map"
            message.registry_sha256 = self.garbage_registry.sha256
            for track in tracks:
                if track.state in {"LOST", "REJECTED", "CLEANED"}:
                    continue
                entry = self.registry_entries.get(track.class_id)
                if entry is None:
                    continue
                target = GarbageTarget()
                target.header = message.header
                target.uuid = track.uuid
                target.class_id = track.class_id
                target.target_type = entry.target_type
                target.confidence = float(track.score_ema)
                target.map_pose.pose.position.x = track.x_m
                target.map_pose.pose.position.y = track.y_m
                target.map_pose.pose.position.z = track.z_m
                target.map_pose.pose.orientation.w = 1.0
                target.map_pose.covariance[0] = track.covariance_trace * 0.5
                target.map_pose.covariance[7] = track.covariance_trace * 0.5
                if track.polygon_xy_m:
                    polygon = track.polygon_xy_m
                    xs = [point[0] for point in polygon]
                    ys = [point[1] for point in polygon]
                    size = (max(xs) - min(xs), max(ys) - min(ys), entry.size_m[2])
                else:
                    half_x, half_y = entry.size_m[0] * 0.5, entry.size_m[1] * 0.5
                    polygon = (
                        (track.x_m - half_x, track.y_m - half_y),
                        (track.x_m + half_x, track.y_m - half_y),
                        (track.x_m + half_x, track.y_m + half_y),
                        (track.x_m - half_x, track.y_m + half_y),
                    )
                    size = entry.size_m
                for x_m, y_m in polygon:
                    target.polygon.points.append(
                        Point32(x=float(x_m), y=float(y_m), z=0.0)
                    )
                target.size.x, target.size.y, target.size.z = (
                    float(size[0]), float(size[1]), float(size[2])
                )
                target.first_seen = Time(
                    nanoseconds=int(track.first_seen_s * 1_000_000_000)
                ).to_msg()
                target.last_seen = Time(
                    nanoseconds=int(track.last_seen_s * 1_000_000_000)
                ).to_msg()
                target.observation_count = int(track.observation_count)
                target.track_state = track.state
                target.cleaning_policy = entry.policy
                target.source_backend = track.source_backend
                target.source_stamp = rgb_message.header.stamp
                target.visibility = 1.0
                target.occlusion_ratio = 0.0
                # Keepout membership has no authoritative product map input at
                # P6 yet. Mark unknown as blocked rather than action-safe.
                target.in_keepout = True
                message.targets.append(target)
            self.target_publisher.publish(message)

        def _publish_health(self) -> None:
            now = time.monotonic()
            snapshot = self.health.snapshot(now)
            snapshot["last_error"] = self.last_error
            snapshot["sync"] = (
                {
                    "sync_count": self.synchronizer.sync_count,
                    "sync_reject_count": self.synchronizer.sync_reject_count,
                    "dropped": self.synchronizer.dropped,
                }
                if self.synchronizer is not None
                else None
            )
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
