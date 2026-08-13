from pathlib import Path

import train_trcrv10_identifiability as train


def test_only_protocol_bounded_models_and_views() -> None:
    assert train.MODELS == ("convnext_tiny", "resnet18")
    assert train.VIEWS == ("tight", "context")
    source = Path(train.__file__).read_text(encoding="utf-8")
    assert "ConvNeXt_Tiny_Weights.IMAGENET1K_V1" in source
    assert "ResNet18_Weights.IMAGENET1K_V1" in source
    assert "requires CUDA" in source


def test_metrics_are_per_class_and_macro() -> None:
    result = train.metrics([0, 0, 1, 1, 2, 2], [0, 1, 1, 1, 2, 0])
    assert set(result) == {"macro_f1", "per_class", "confusion"}
    assert set(result["per_class"]) == set(train.CLASSES)
    assert result["confusion"] == [[1, 1, 0], [0, 2, 0], [1, 0, 1]]


def test_training_result_is_bound_to_dataset_and_sealed_boundaries() -> None:
    source = Path(train.__file__).read_text(encoding="utf-8")
    for field in (
        "dataset_manifest_sha256", "train_samples", "holdout_samples",
        "by_size_support", "recommended_100_frames_per_class_met", "scene_seeds",
        "production_runtime_eligible", "G10_DEV_VAL_SEALED_read", "VAL_NEW_read", "G5_V2_read",
    ):
        assert field in source
