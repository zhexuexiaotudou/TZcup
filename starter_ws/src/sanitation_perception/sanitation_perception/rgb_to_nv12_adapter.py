"""Fail-closed ROS image adapter for the S100P EdgeSAM NV12 input contract."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


class Nv12ConversionError(ValueError):
    """Raised when an image cannot be bound safely to an NV12 frame."""


def image_bytes_to_nv12(
    data: bytes | bytearray | memoryview,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
) -> bytes:
    """Return tightly packed NV12 while preserving the source dimensions."""

    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise Nv12ConversionError("NV12 requires positive even width and height")
    normalized = encoding.strip().lower()
    raw = memoryview(data)
    nv12_size = width * height * 3 // 2
    if normalized == "nv12":
        if len(raw) != nv12_size:
            raise Nv12ConversionError("NV12 payload size does not match dimensions")
        return bytes(raw)
    if normalized not in {"rgb8", "bgr8"}:
        raise Nv12ConversionError(f"unsupported source encoding: {encoding}")
    row_step = int(step) if int(step) > 0 else width * 3
    if row_step < width * 3 or len(raw) != row_step * height:
        raise Nv12ConversionError("RGB payload or step does not match dimensions")
    rows = np.frombuffer(raw, dtype=np.uint8).reshape(height, row_step)
    image = rows[:, : width * 3].reshape(height, width, 3)
    bgr = image if normalized == "bgr8" else image[:, :, ::-1]
    i420 = cv2.cvtColor(np.ascontiguousarray(bgr), cv2.COLOR_BGR2YUV_I420).reshape(-1)
    y_size = width * height
    chroma_size = y_size // 4
    y_plane = i420[:y_size]
    u_plane = i420[y_size : y_size + chroma_size].reshape(height // 2, width // 2)
    v_plane = i420[y_size + chroma_size :].reshape(height // 2, width // 2)
    uv_plane = np.empty((height // 2, width), dtype=np.uint8)
    uv_plane[:, 0::2] = u_plane
    uv_plane[:, 1::2] = v_plane
    return y_plane.tobytes() + uv_plane.tobytes()


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
            self.declare_parameter(
                "output_topic", "/perception/open_vocab/front_nv12"
            )
            self.declare_parameter(
                "diagnostics_topic", "/perception/open_vocab/diagnostics"
            )
            output_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._publisher = self.create_publisher(
                Image, str(self.get_parameter("output_topic").value), output_qos
            )
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
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.level = bytes([level])
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
            try:
                payload = image_bytes_to_nv12(
                    message.data,
                    width=int(message.width),
                    height=int(message.height),
                    encoding=str(message.encoding),
                    step=int(message.step),
                )
                output = Image()
                output.header = message.header
                output.height = message.height
                output.width = message.width
                output.encoding = "nv12"
                output.is_bigendian = 0
                output.step = message.width
                output.data = payload
                self._publisher.publish(output)
                self._status(
                    0,
                    "nv12_frame_published",
                    {
                        "source_encoding": message.encoding,
                        "width": message.width,
                        "height": message.height,
                        "payload_bytes": len(payload),
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
