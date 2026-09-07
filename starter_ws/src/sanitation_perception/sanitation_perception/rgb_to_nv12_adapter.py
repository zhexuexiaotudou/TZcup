"""Fail-closed, stamp-selected S100P NV12 inputs for DOSOD and EdgeSAM."""

from __future__ import annotations

import time
from typing import Any

import cv2
import numpy as np


class Nv12ConversionError(ValueError):
    """Raised when an image cannot be bound safely to an NV12 frame."""


S100P_SOURCE_WIDTH = 848
S100P_SOURCE_HEIGHT = 480
S100P_SELECTED_PERIOD_NS = 500_000_000


class SourceStampSelector:
    """Deterministically select at most 2 Hz without wall-clock state."""

    def __init__(self) -> None:
        self.last_received_ns = -1
        self.last_selected_ns = -1

    def select(self, stamp_ns: int) -> tuple[bool, str]:
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
            return False, "invalid_stamp"
        if stamp_ns <= self.last_received_ns:
            return False, "duplicate_or_out_of_order"
        self.last_received_ns = stamp_ns
        if self.last_selected_ns >= 0 and stamp_ns - self.last_selected_ns < S100P_SELECTED_PERIOD_NS:
            return False, "rate_limited"
        return True, "selected"

    def commit_selected(self, stamp_ns: int) -> None:
        if stamp_ns != self.last_received_ns:
            raise Nv12ConversionError("selected stamp no longer matches the source stream")
        self.last_selected_ns = stamp_ns


class SuccessDiagnosticThrottle:
    """Throttle only healthy diagnostics against a monotonic local clock."""

    def __init__(self) -> None:
        self._last_success_ns = -1

    def allow(self, level: int, monotonic_ns: int) -> bool:
        if level != 0:
            return True
        if monotonic_ns - self._last_success_ns < 1_000_000_000:
            return False
        self._last_success_ns = monotonic_ns
        return True


def _bgr_from_source(
    data: bytes | bytearray | memoryview,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
) -> np.ndarray:
    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise Nv12ConversionError("NV12 requires positive even width and height")
    normalized = encoding.strip().lower()
    raw = memoryview(data)
    if normalized == "nv12":
        if int(step) != width or len(raw) != width * height * 3 // 2:
            raise Nv12ConversionError("NV12 step or payload size does not match dimensions")
        return cv2.cvtColor(
            np.frombuffer(raw, dtype=np.uint8).reshape(height * 3 // 2, width),
            cv2.COLOR_YUV2BGR_NV12,
        )
    if normalized not in {"rgb8", "bgr8"}:
        raise Nv12ConversionError(f"unsupported source encoding: {encoding}")
    row_step = int(step) if int(step) > 0 else width * 3
    if row_step < width * 3 or len(raw) != row_step * height:
        raise Nv12ConversionError("RGB payload or step does not match dimensions")
    image = np.frombuffer(raw, dtype=np.uint8).reshape(height, row_step)[:, : width * 3]
    image = image.reshape(height, width, 3)
    return image if normalized == "bgr8" else image[:, :, ::-1]


def _bgr_to_nv12(bgr: np.ndarray) -> bytes:
    height, width = bgr.shape[:2]
    i420 = cv2.cvtColor(np.ascontiguousarray(bgr), cv2.COLOR_BGR2YUV_I420).reshape(-1)
    y_size = width * height
    chroma_size = y_size // 4
    u_plane = i420[y_size : y_size + chroma_size].reshape(height // 2, width // 2)
    v_plane = i420[y_size + chroma_size :].reshape(height // 2, width // 2)
    uv_plane = np.empty((height // 2, width), dtype=np.uint8)
    uv_plane[:, 0::2] = u_plane
    uv_plane[:, 1::2] = v_plane
    return i420[:y_size].tobytes() + uv_plane.tobytes()


def image_bytes_to_nv12(
    data: bytes | bytearray | memoryview,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
) -> bytes:
    """Return tightly packed NV12 while preserving the source dimensions."""
    normalized = encoding.strip().lower()
    raw = memoryview(data)
    if normalized == "nv12":
        if width <= 0 or height <= 0 or width % 2 or height % 2:
            raise Nv12ConversionError("NV12 requires positive even width and height")
        if int(step) != width or len(raw) != width * height * 3 // 2:
            raise Nv12ConversionError("NV12 step or payload size does not match dimensions")
        return bytes(raw)
    return _bgr_to_nv12(
        _bgr_from_source(data, width=width, height=height, encoding=encoding, step=step)
    )


def s100p_nv12_pair(
    data: bytes | bytearray | memoryview,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
) -> tuple[bytes, bytes]:
    """Make one packed 848x480 NV12 payload available on both BPU topics.

    The installed official BPU nodes own their respective ResizeNV12 / pyramid
    preprocessing.  Project code forwards only the selected original frame.
    """
    if (width, height) != (S100P_SOURCE_WIDTH, S100P_SOURCE_HEIGHT):
        raise Nv12ConversionError("S100P RGB source must be exactly 848x480")
    payload = image_bytes_to_nv12(
        data, width=width, height=height, encoding=encoding, step=step
    )
    return payload, payload


def main() -> None:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class RgbToNv12Adapter(Node):
        def __init__(self) -> None:
            super().__init__("rgb_to_nv12_adapter")
            self.declare_parameter(
                "input_topic", "/sensors/front_rgbd/depth/image_rect_raw/image"
            )
            self.declare_parameter("dosod_output_topic", "/perception/open_vocab/front_dosod_nv12")
            self.declare_parameter("edgesam_output_topic", "/perception/open_vocab/front_edgesam_nv12")
            self.declare_parameter(
                "diagnostics_topic", "/perception/open_vocab/diagnostics"
            )
            output_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._dosod_publisher = self.create_publisher(Image, str(self.get_parameter("dosod_output_topic").value), output_qos)
            self._edgesam_publisher = self.create_publisher(Image, str(self.get_parameter("edgesam_output_topic").value), output_qos)
            self._selector = SourceStampSelector()
            self._success_diagnostic_throttle = SuccessDiagnosticThrottle()
            self._input_frames = self._selected_frames = self._dosod_frames = self._edgesam_frames = 0
            self._diagnostics = self.create_publisher(
                DiagnosticArray,
                str(self.get_parameter("diagnostics_topic").value),
                10,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("input_topic").value),
                self._on_image,
                qos_profile_sensor_data,
            )

        def _status(self, level: int, message: str, values: dict[str, Any]) -> None:
            if not self._success_diagnostic_throttle.allow(level, time.monotonic_ns()):
                return
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            # diagnostic_msgs/DiagnosticStatus.level is ROS uint8, represented
            # by an integer in rclpy (not a one-byte ``bytes`` payload).
            status.level = int(level)
            status.name = "formal_open_vocab_perception/rgb_to_nv12_adapter"
            status.hardware_id = "RDK_S100P_Journey_6P"
            status.message = message
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in sorted(values.items())
            ]
            array.status = [status]
            self._diagnostics.publish(array)

        def _on_image(self, message: Image) -> None:
            self._input_frames += 1
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            selected, reason = self._selector.select(stamp_ns)
            if not selected:
                if reason != "rate_limited":
                    self._status(2, "source_stamp_rejected_fail_closed", {"source_stamp_ns": stamp_ns, "reason": reason})
                return
            try:
                dosod_payload, edgesam_payload = s100p_nv12_pair(
                    message.data,
                    width=int(message.width),
                    height=int(message.height),
                    encoding=str(message.encoding),
                    step=int(message.step),
                )
                def output(width: int, height: int, payload: bytes) -> Image:
                    item = Image()
                    item.header = message.header
                    item.height, item.width, item.encoding, item.is_bigendian, item.step, item.data = height, width, "nv12", 0, width, payload
                    return item
                self._dosod_publisher.publish(output(S100P_SOURCE_WIDTH, S100P_SOURCE_HEIGHT, dosod_payload))
                self._edgesam_publisher.publish(output(S100P_SOURCE_WIDTH, S100P_SOURCE_HEIGHT, edgesam_payload))
                self._selector.commit_selected(stamp_ns)
                self._selected_frames += 1
                self._dosod_frames += 1
                self._edgesam_frames += 1
                self._status(
                    0,
                    "selected_nv12_pair_published",
                    {
                        "source_stamp_ns": stamp_ns,
                        "input_frames": self._input_frames,
                        "selected_frames": self._selected_frames,
                        "dosod_output_frames": self._dosod_frames,
                        "edgesam_output_frames": self._edgesam_frames,
                        "selector_hz": 2,
                        "dosod_payload_bytes": len(dosod_payload),
                        "edgesam_payload_bytes": len(edgesam_payload),
                        "source_width": S100P_SOURCE_WIDTH,
                        "source_height": S100P_SOURCE_HEIGHT,
                    },
                )
            except Exception as exc:
                self._status(
                    2,
                    "nv12_conversion_failed_closed",
                    {"source_encoding": message.encoding, "error": str(exc)},
                )

    rclpy.init()
    node: RgbToNv12Adapter | None = None
    try:
        node = RgbToNv12Adapter()
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
