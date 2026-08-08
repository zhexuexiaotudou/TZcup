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


def main() -> None:
    import message_filters
    import onnxruntime as ort
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from sanitation_perception.inference_engine import ProductInferenceEngine
    from sanitation_perception.model_registry import ProductModelRegistry

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
            self.synchronizer = None
            self.scheduler = None
            self.sensor_subscribers = []
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
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                return result
            if self.engine is None or self.health.state != "INACTIVE":
                self.last_error = "pipeline was not configured and warmed"
                return TransitionCallbackReturn.FAILURE
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
                self.scheduler.submit(frame)

        def _consume_latest(self) -> None:
            if self.health.state != "ACTIVE" or self.scheduler is None:
                return
            frame = self.scheduler.pop_latest()
            if frame is None:
                return
            try:
                # The RGB timestamp is mandatory. Looking up Time() / latest is forbidden.
                self.tf_buffer.lookup_transform(
                    "map",
                    frame.rgb.payload.header.frame_id,
                    Time.from_msg(frame.rgb.payload.header.stamp),
                )
                self.health.record_tf_success()
                # Frozen-model decode/publish is deliberately gated by the declared
                # contract. Its implementation lands with the selected P4 artifacts.
                raise RuntimeError("frozen product postprocessor is not installed")
            except TransformException as exc:
                self.last_error = str(exc)
                self.health.record_tf_error()
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                self.health.record_session_error(oom="out of memory" in str(exc).lower())

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
            self.metrics_publisher.publish(
                String(data=json.dumps({"acceptance_ready": False, **snapshot}, sort_keys=True))
            )

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
