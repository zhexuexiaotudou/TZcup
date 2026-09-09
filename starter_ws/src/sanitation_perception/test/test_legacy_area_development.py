import numpy as np
import pytest

from sanitation_perception.legacy_area_development import (
    decode_legacy_area,
    preprocess_legacy_area,
)


def test_legacy_area_preprocess_reconstructs_seven_channels():
    rgb = np.zeros((24, 32, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    depth = np.full((24, 32), 2.0, dtype=np.float32)
    tensor = preprocess_legacy_area(rgb, depth)
    assert tensor.shape == (1, 7, 384, 512)
    assert tensor.dtype == np.float32
    assert tensor.flags.c_contiguous


def test_legacy_area_heads_are_leaf_and_puddle_not_boundary():
    logits = np.full((1, 2, 384, 512), -10.0, dtype=np.float32)
    logits[0, 1, 100:110, 100:110] = 10.0
    decoded = decode_legacy_area(logits)
    assert not decoded["leaf_pile"]["mask"].any()
    assert decoded["puddle"]["mask"].sum() == 100


def test_legacy_area_rejects_bad_shapes():
    with pytest.raises(ValueError, match="output shape"):
        decode_legacy_area(np.zeros((1, 1, 384, 512), dtype=np.float32))


def test_legacy_area_rejects_nonfinite_logits():
    logits = np.zeros((1, 2, 384, 512), dtype=np.float32)
    logits[0, 0, 0, 0] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        decode_legacy_area(logits)
