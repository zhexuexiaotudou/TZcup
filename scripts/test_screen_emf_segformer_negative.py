from pathlib import Path

import numpy as np
import pytest

import screen_emf_segformer_negative as screening


def test_prediction_summary_preserves_native_classes_without_target_mapping():
    native = np.array([[21, 21, 4], [6, 17, 17]], dtype=np.int16)
    labels = {index: f"class_{index}" for index in range(150)}
    labels.update({4: "tree", 17: "plant", 21: "water"})

    result = screening.summarize_prediction(native, labels)

    assert result["grid_hw"] == [2, 3]
    assert result["relevant_native_classes"]["water"] == {
        "class_id": 21,
        "pixel_count": 2,
        "pixel_fraction": 2 / 6,
    }
    assert result["relevant_native_classes"]["plant"]["pixel_count"] == 2
    assert "leaf_pile" not in result["relevant_native_classes"]
    assert "puddle" not in result["relevant_native_classes"]


@pytest.mark.parametrize(
    "native",
    [
        np.zeros((1, 1, 1), dtype=np.int16),
        np.array([[-1]], dtype=np.int16),
        np.array([[150]], dtype=np.int16),
        np.array([[1.0]], dtype=np.float32),
    ],
)
def test_prediction_summary_rejects_invalid_native_output(native):
    with pytest.raises(ValueError, match="prediction"):
        screening.summarize_prediction(
            native, {index: str(index) for index in range(150)}
        )


def test_artifact_contract_rejects_missing_files(tmp_path: Path):
    with pytest.raises(ValueError, match="missing"):
        screening.verify_artifacts(tmp_path)
