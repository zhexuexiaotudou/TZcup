import struct
import zlib
from pathlib import Path

from sanitation_hmi.ros_adapter import encode_image_png


ROOT = Path(__file__).resolve().parents[1]


class Image:
    width = 2
    height = 1
    encoding = "bgr8"
    step = 6
    data = bytes([0, 0, 255, 0, 255, 0])


def test_standard_library_png_encoder_converts_bgr_to_rgb():
    png = encode_image_png(Image())
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    offset = 8
    chunks = {}
    while offset < len(png):
        length = struct.unpack(">I", png[offset : offset + 4])[0]
        kind = png[offset + 4 : offset + 8]
        chunks.setdefault(kind, b"")
        chunks[kind] += png[offset + 8 : offset + 8 + length]
        offset += length + 12
    raw = zlib.decompress(chunks[b"IDAT"])
    assert raw == bytes([0, 255, 0, 0, 0, 255, 0])


def test_clock_subscription_uses_simulation_clock_qos():
    source = (ROOT / "sanitation_hmi/ros_adapter.py").read_text(encoding="utf-8")
    assert "from rclpy.qos import qos_profile_sensor_data" in source
    assert "qos_profile_sensor_data," in source
