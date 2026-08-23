import numpy as np
import pytest

from j6_tensor_parity import compare_outputs


def test_tensor_parity_reports_every_output_node():
    report = compare_outputs(
        {"boxes": np.ones((1, 4)), "scores": np.array([[0.2, 0.8]])},
        {"boxes": np.ones((1, 4)), "scores": np.array([[0.2, 0.8]])},
    )
    assert report["all_nodes_pass"] is True
    assert set(report["nodes"]) == {"boxes", "scores"}


def test_tensor_parity_rejects_shape_or_node_substitution():
    with pytest.raises(ValueError, match="names differ"):
        compare_outputs({"a": np.ones(1)}, {"b": np.ones(1)})
    with pytest.raises(ValueError, match="shape differs"):
        compare_outputs({"a": np.ones(1)}, {"a": np.ones(2)})


def test_tensor_parity_fails_large_direction_error():
    report = compare_outputs(
        {"a": np.array([1.0, 0.0])},
        {"a": np.array([0.0, 1.0])},
    )
    assert report["all_nodes_pass"] is False
