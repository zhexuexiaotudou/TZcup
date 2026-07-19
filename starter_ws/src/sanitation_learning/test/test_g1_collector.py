import numpy as np
import pytest

from sanitation_learning.g1_collector import _decode_label_map, _decode_semantic_map


def test_gazebo_semantic_repeated_channel_decode():
    encoded = np.array([[[0, 0, 0], [1, 1, 1], [5, 5, 5]]], dtype=np.uint8)
    assert _decode_semantic_map(encoded).tolist() == [[0, 1, 5]]


def test_instance_map_keeps_full_24_bit_identity():
    encoded = np.array([[[3, 1, 0], [5, 4, 3]]], dtype=np.uint8)
    assert _decode_label_map(encoded).tolist() == [[259, 197637]]


def test_semantic_decode_rejects_colored_map():
    with pytest.raises(ValueError, match="repeated-channel"):
        _decode_semantic_map(np.array([[[1, 2, 3]]], dtype=np.uint8))
