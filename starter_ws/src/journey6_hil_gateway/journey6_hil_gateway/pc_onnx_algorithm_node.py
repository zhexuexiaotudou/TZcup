"""Explicit PC_ONNX algorithm-host probe for split-loopback emulation."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

from .emulation import (
    RuntimeIdentity,
    SensorContractAudit,
    audit_model_qualification_manifest,
    validate_run_id,
)


def _atomic_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def _stamp_s(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) / 1_000_000_000.0


def _runtime_platform() -> dict[str, str | None]:
    values: dict[str, str] = {}
    try:
        for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
            key, separator, value = line.partition("=")
            if separator:
                values[key] = value.strip().strip('"')
    except OSError:
        pass
    return {
        "os_id": values.get("ID"),
        "os_version_id": values.get("VERSION_ID"),
        "ros_distro": os.environ.get("ROS_DISTRO"),
    }


def main(args=None) -> None:
    import numpy as np
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
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

    class PcOnnxAlgorithmHost(Node):
        def __init__(self) -> None:
            super().__init__("pc_onnx_algorithm_host", namespace="/j6")
            self.declare_parameter("runtime_backend", "PC_ONNX")
            self.declare_parameter("not_journey6_runtime", True)
            self.declare_parameter("run_id", "")
            self.declare_parameter("model_path", "")
            self.declare_parameter("model_id", "")
            self.declare_parameter("required_model_id", "d1_littercam_yolov9c")
            self.declare_parameter("expected_sha256", "")
            self.declare_parameter(
                "qualification_manifest_path",
                "/opt/tzcup/models/model_qualification_manifest.json",
            )
            # Retained only for CLI compatibility. Readiness never trusts it.
            self.declare_parameter("model_contract_qualified", False)
            self.declare_parameter("evidence_directory", "/evidence")
            self.declare_parameter("apply_network_faults", False)
            identity = RuntimeIdentity(
                runtime_backend=str(self.get_parameter("runtime_backend").value),
                not_journey6_runtime=bool(
                    self.get_parameter("not_journey6_runtime").value
                ),
            )
            identity.validate()
            if identity.runtime_backend != "PC_ONNX":
                raise ValueError("PC ONNX host refuses non-PC_ONNX runtime_backend")
            self.identity = identity
            self.run_id = validate_run_id(
                str(self.get_parameter("run_id").value)
            )
            self.model_path = Path(str(self.get_parameter("model_path").value))
            self.model_id = str(self.get_parameter("model_id").value)
            self.required_model_id = str(
                self.get_parameter("required_model_id").value
            )
            self.expected_sha256 = str(
                self.get_parameter("expected_sha256").value
            ).lower()
            self.qualification_manifest_path = Path(
                str(self.get_parameter("qualification_manifest_path").value)
            )
            self.evidence_directory = Path(
                str(self.get_parameter("evidence_directory").value)
            )
            self.apply_network_faults = bool(
                self.get_parameter("apply_network_faults").value
            )
            self.audit = SensorContractAudit()
            self.session = None
            self.input_meta = None
            self.model_sha256 = None
            self.model_error = None
            self.inference_count = 0
            self.inference_error_count = 0
            self.command_sequence = 0
            self.health_sequence = 0
            self.paused = False
            self.color_frames: dict[int, Image] = {}
            self.depth_frames: dict[int, Image] = {}
            self.restore_network_at_monotonic: float | None = None
            self.actual_network_fault_applied = False
            self.actual_network_restore_applied = False
            self._load_model()

            self.command_publisher = self.create_publisher(
                String, "/hil/vehicle/ackermann_command", control_qos
            )
            self.health_publisher = self.create_publisher(
                String, "/hil/health", reliable_qos
            )
            self.create_subscription(
                Image, "/hil/camera/color", self._on_color, sensor_qos
            )
            self.create_subscription(
                Image, "/hil/camera/depth", self._on_depth, sensor_qos
            )
            self.create_subscription(
                CameraInfo,
                "/hil/camera/camera_info",
                self._on_camera_info,
                sensor_qos,
            )
            self.create_subscription(TFMessage, "/hil/tf", self._on_tf, sensor_qos)
            self.create_subscription(
                TFMessage, "/hil/tf_static", self._on_tf_static, static_qos
            )
            self.create_subscription(
                Bool, "/hil/harness/algorithm_pause", self._on_pause, reliable_qos
            )
            self.create_subscription(
                String,
                "/hil/harness/network_fault",
                self._on_network_fault,
                reliable_qos,
            )
            self.create_timer(0.10, self._publish_health)
            self.create_timer(0.10, self._network_restore_timer)
            self.create_timer(1.0, self._write_evidence)
            self._write_evidence()

        def _load_model(self) -> None:
            try:
                if not self.model_path.is_file():
                    raise FileNotFoundError(self.model_path)
                actual = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
                if self.expected_sha256 and actual != self.expected_sha256:
                    raise ValueError("PC_ONNX artifact SHA-256 mismatch")
                import onnxruntime as ort

                if "CPUExecutionProvider" not in ort.get_available_providers():
                    raise RuntimeError("CPUExecutionProvider is unavailable")
                session = ort.InferenceSession(
                    str(self.model_path), providers=["CPUExecutionProvider"]
                )
                if session.get_providers() != ["CPUExecutionProvider"]:
                    raise RuntimeError("PC_ONNX provider fallback detected")
                inputs = session.get_inputs()
                if len(inputs) != 1:
                    raise ValueError("loopback probe requires one ONNX image input")
                shape = tuple(inputs[0].shape)
                if len(shape) != 4 or shape[0] != 1 or any(
                    not isinstance(value, int) or value <= 0 for value in shape
                ):
                    raise ValueError("loopback probe requires static NCHW batch-1 input")
                self.session = session
                self.input_meta = inputs[0]
                self.model_sha256 = actual
            except (FileNotFoundError, ImportError, RuntimeError, ValueError) as error:
                self.model_error = f"{type(error).__name__}: {error}"
                self.session = None

        def _on_camera_info(self, message: CameraInfo) -> None:
            self.audit.observe_camera_info(_stamp_s(message.header))

        def _on_tf(self, message: TFMessage) -> None:
            for transform in message.transforms:
                self.audit.observe_tf(_stamp_s(transform.header), static=False)

        def _on_tf_static(self, message: TFMessage) -> None:
            for transform in message.transforms:
                self.audit.observe_tf(_stamp_s(transform.header), static=True)

        def _on_color(self, message: Image) -> None:
            stamp = _stamp_s(message.header)
            self.audit.observe_color(stamp)
            key = int(round(stamp * 1_000_000_000.0))
            self.color_frames[key] = message
            self._try_infer(key)

        def _on_depth(self, message: Image) -> None:
            stamp = _stamp_s(message.header)
            self.audit.observe_depth(stamp)
            key = int(round(stamp * 1_000_000_000.0))
            self.depth_frames[key] = message
            self._try_infer(key)

        def _try_infer(self, key: int) -> None:
            color = self.color_frames.pop(key, None)
            depth = self.depth_frames.pop(key, None)
            if color is None or depth is None:
                if color is not None:
                    self.color_frames[key] = color
                if depth is not None:
                    self.depth_frames[key] = depth
                return
            if self.session is None or self.paused:
                return
            try:
                value = self._preprocess(color)
                self.session.run(None, {self.input_meta.name: value})
                self.inference_count += 1
                self._publish_command(_stamp_s(color.header))
            except (ValueError, RuntimeError) as error:
                self.inference_error_count += 1
                self.model_error = f"inference: {type(error).__name__}: {error}"

        def _preprocess(self, message: Image):
            if message.encoding.lower() not in {"rgb8", "bgr8"}:
                raise ValueError("PC_ONNX probe accepts rgb8/bgr8 only")
            expected = message.height * message.width * 3
            raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
            if raw.size != expected:
                raise ValueError("image payload size does not match dimensions")
            image = raw.reshape(message.height, message.width, 3)
            if message.encoding.lower() == "bgr8":
                image = image[:, :, ::-1]
            _, channels, target_h, target_w = tuple(self.input_meta.shape)
            if channels != 3:
                raise ValueError("PC_ONNX probe requires a three-channel model")
            rows = np.linspace(0, message.height - 1, target_h).astype(np.int64)
            columns = np.linspace(0, message.width - 1, target_w).astype(np.int64)
            resized = image[rows][:, columns]
            tensor = np.transpose(resized, (2, 0, 1))[None]
            if "float" in self.input_meta.type:
                tensor = tensor.astype(np.float32) / 255.0
            elif "uint8" in self.input_meta.type:
                tensor = tensor.astype(np.uint8)
            else:
                raise ValueError(f"unsupported ONNX input type: {self.input_meta.type}")
            return np.ascontiguousarray(tensor)

        def _publish_command(self, stamp_s: float) -> None:
            self.command_sequence += 1
            payload = {
                "stamp_s": stamp_s,
                "sequence": self.command_sequence,
                "speed_mps": 0.10,
                "steering_angle_rad": 0.0,
                "acceleration_limit_mps2": 0.5,
                "source_id": "j6-algorithm",
                "valid_until_s": stamp_s + 0.18,
            }
            self.command_publisher.publish(
                String(data=json.dumps(payload, sort_keys=True))
            )

        def _publish_health(self) -> None:
            self.health_sequence += 1
            payload = {
                "source_id": "j6-algorithm",
                "sequence": self.health_sequence,
                "stamp_s": self.get_clock().now().nanoseconds / 1_000_000_000.0,
                "healthy": self.session is not None and self.model_error is None,
                "runtime_backend": "PC_ONNX",
                "not_journey6_runtime": True,
            }
            self.health_publisher.publish(
                String(data=json.dumps(payload, sort_keys=True))
            )

        def _on_pause(self, message: Bool) -> None:
            self.paused = bool(message.data)

        def _on_network_fault(self, message: String) -> None:
            try:
                request = json.loads(message.data)
                if request.get("profile") != "disconnect":
                    raise ValueError("only bounded disconnect is accepted by the harness")
                duration_s = float(request.get("duration_s", 2.0))
                if not 0.25 <= duration_s <= 30.0:
                    raise ValueError("network fault duration must be within 0.25..30 s")
                if not self.apply_network_faults:
                    raise RuntimeError("actual network fault application is disabled")
                result = subprocess.run(
                    [
                        "python3",
                        "/opt/tzcup/bin/j6_hil_network_faults.py",
                        "disconnect",
                        "--apply",
                        "--evidence",
                        "/evidence/HIL_NETWORK_DISCONNECT.json",
                    ],
                    text=True,
                    capture_output=True,
                    check=False,
                )
                if result.returncode != 0:
                    raise RuntimeError(result.stderr.strip() or result.stdout.strip())
                self.actual_network_fault_applied = True
                self.restore_network_at_monotonic = time.monotonic() + duration_s
            except (TypeError, ValueError, RuntimeError, json.JSONDecodeError) as error:
                self.get_logger().error(f"network fault request rejected: {error}")

        def _network_restore_timer(self) -> None:
            if (
                self.restore_network_at_monotonic is None
                or time.monotonic() < self.restore_network_at_monotonic
            ):
                return
            self.restore_network_at_monotonic = None
            result = subprocess.run(
                [
                    "python3",
                    "/opt/tzcup/bin/j6_hil_network_faults.py",
                    "normal",
                    "--apply",
                    "--evidence",
                    "/evidence/HIL_NETWORK_RESTORE.json",
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.actual_network_restore_applied = result.returncode == 0
            if result.returncode != 0:
                self.get_logger().error(
                    "network restore failed: " + (result.stderr or result.stdout).strip()
                )

        def _write_evidence(self) -> None:
            active_providers = [] if self.session is None else self.session.get_providers()
            qualification = audit_model_qualification_manifest(
                self.qualification_manifest_path,
                model_id=self.model_id,
                model_sha256=self.model_sha256 or "",
                run_id=self.run_id,
            )
            qualification_pass = all(
                (
                    qualification["pt_onnx_parity_pass"] is True,
                    qualification["pc_inference_pass"] is True,
                    qualification["full_stack_pass"] is True,
                )
            )
            payload = {
                "schema_version": 2,
                "runtime_backend": self.identity.runtime_backend,
                "not_journey6_runtime": self.identity.not_journey6_runtime,
                "run_id": self.run_id,
                "platform": _runtime_platform(),
                "model_id": self.model_id,
                "required_model_id": self.required_model_id,
                "required_model_id_match": self.model_id == self.required_model_id,
                "model_contract_qualified": qualification_pass,
                "model_qualification": qualification,
                "model_path": str(self.model_path),
                "model_sha256": self.model_sha256,
                "model_loaded": self.session is not None,
                "provider": active_providers[0] if active_providers else None,
                "fallback_used": len(active_providers) > 1,
                "model_error": self.model_error,
                "inference_count": self.inference_count,
                "inference_error_count": self.inference_error_count,
                "network_faults_requested": self.apply_network_faults,
                "actual_network_fault_applied": self.actual_network_fault_applied,
                "actual_network_restore_applied": self.actual_network_restore_applied,
                "transport": self.audit.snapshot(),
            }
            _atomic_json(
                self.evidence_directory / "HIL_ALGORITHM_RUNTIME.json", payload
            )

    rclpy.init(args=args)
    node = PcOnnxAlgorithmHost()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._write_evidence()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
