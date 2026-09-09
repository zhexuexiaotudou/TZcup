import numpy as np
import pytest

from sanitation_perception.journey6_nv12 import letterbox_rgb, nv12_to_rgb, rgb_to_nv12


def test_letterbox_is_deterministic_and_restores_boxes():
    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    output, geometry = letterbox_rgb(rgb, (640, 640))
    assert output.shape == (640, 640, 3)
    assert geometry.pad_top == 80
    restored = geometry.restore_xyxy((20.0, 100.0, 220.0, 300.0))
    assert restored == pytest.approx((10.0, 10.0, 110.0, 110.0))


def test_nv12_contract_has_expected_plane_and_roundtrip_error():
    x = np.arange(0, 64, dtype=np.uint8)
    rgb = np.stack(np.meshgrid(x, x), axis=-1)
    rgb = np.concatenate((rgb, rgb[..., :1]), axis=2)
    nv12 = rgb_to_nv12(rgb, matrix="bt601", value_range="limited")
    assert nv12.shape == (96, 64)
    restored = nv12_to_rgb(nv12, width=64, height=64)
    assert restored.shape == rgb.shape
    assert float(np.abs(restored.astype(np.int16) - rgb.astype(np.int16)).mean()) < 5.0


def test_nv12_rejects_odd_dimensions_and_unknown_contract():
    with pytest.raises(ValueError, match="even"):
        rgb_to_nv12(np.zeros((3, 4, 3), dtype=np.uint8))
    with pytest.raises(ValueError, match="unsupported"):
        rgb_to_nv12(np.zeros((4, 4, 3), dtype=np.uint8), matrix="guess")
