"""Tests for the formal RGB/BGR to NV12 image boundary."""

import numpy as np
import pytest

from sanitation_perception.rgb_to_nv12_adapter import (
    Nv12ConversionError,
    image_bytes_to_nv12,
)


def test_bgr_and_rgb_convert_to_tightly_packed_nv12():
    bgr = np.zeros((2, 4, 3), dtype=np.uint8)
    bgr[:, :, 2] = 255
    bgr_nv12 = image_bytes_to_nv12(
        bgr.tobytes(), width=4, height=2, encoding="bgr8", step=12
    )
    rgb_nv12 = image_bytes_to_nv12(
        bgr[:, :, ::-1].tobytes(), width=4, height=2, encoding="rgb8", step=12
    )
    assert len(bgr_nv12) == 12
    assert bgr_nv12 == rgb_nv12


def test_nv12_passthrough_is_exact_and_fail_closed_on_bad_input():
    source = bytes(range(12))
    assert image_bytes_to_nv12(
        source, width=4, height=2, encoding="nv12", step=4
    ) == source
    with pytest.raises(Nv12ConversionError, match="even"):
        image_bytes_to_nv12(source, width=3, height=2, encoding="nv12", step=3)
    with pytest.raises(Nv12ConversionError, match="unsupported"):
        image_bytes_to_nv12(bytes(24), width=4, height=2, encoding="mono8", step=4)
    with pytest.raises(Nv12ConversionError, match="payload"):
        image_bytes_to_nv12(bytes(10), width=4, height=2, encoding="bgr8", step=12)
