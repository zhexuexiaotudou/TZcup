import numpy as np
import pytest

from sanitation_perception.preprocessing import preprocess_rgb, resize_labels


def test_preprocess_rgb_preserves_rgb_order_and_contract():
    image = np.zeros((96, 128, 3), dtype=np.uint8)
    image[0, 0] = (255, 64, 0)
    tensor = preprocess_rgb(image)
    assert tensor.shape == (1, 3, 96, 128)
    assert tensor.dtype == np.float32
    assert tensor[0, :, 0, 0].tolist() == pytest.approx([1.0, 64.0 / 255.0, 0.0])
    assert tensor.flags.c_contiguous


def test_preprocess_rejects_ambiguous_input():
    with pytest.raises(ValueError, match="uint8"):
        preprocess_rgb(np.zeros((96, 128, 3), dtype=np.float32))
    with pytest.raises(ValueError, match="HxWx3"):
        preprocess_rgb(np.zeros((96, 128), dtype=np.uint8))


def test_label_resize_uses_nearest_neighbor():
    labels = np.array([[0, 5], [2, 3]], dtype=np.uint8)
    resized = resize_labels(labels, 4, 4)
    assert set(np.unique(resized)) == {0, 2, 3, 5}
