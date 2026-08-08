from __future__ import annotations

import sys
from pathlib import Path

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_pretrained import (  # noqa: E402
    BACKBONE_SPECS,
    BackboneSpec,
    FROM_SCRATCH_CONTROL_LABEL,
    PRETRAINED_REQUIRED,
    from_scratch_control_record,
    pretrained_backbone_spec,
    provenance_record,
    verify_pretrained_weights,
)


def test_pretrained_is_required() -> None:
    assert PRETRAINED_REQUIRED is True


def test_specs_use_official_enum_api() -> None:
    resnet = pretrained_backbone_spec("resnet18")
    assert resnet.weight_enum == "ResNet18_Weights.IMAGENET1K_V1"
    assert "download.pytorch.org" in resnet.source_url
    assert "BSD" in resnet.license_ref
    assert resnet.architecture == "resnet18"
    deeplab = pretrained_backbone_spec("deeplabv3_resnet50")
    assert "COCO_WITH_VOC_LABELS_V1" in deeplab.weight_enum
    classifier = pretrained_backbone_spec("mobilenet_v3_small")
    assert classifier.weight_enum == (
        "MobileNet_V3_Small_Weights.IMAGENET1K_V1"
    )
    with pytest.raises(ValueError):
        pretrained_backbone_spec("mobilenet_unknown")


def test_verify_fails_closed_without_cache_and_network(tmp_path) -> None:
    with pytest.raises(RuntimeError, match="official pretrained weights are required"):
        verify_pretrained_weights("resnet18")
    with pytest.raises(RuntimeError, match="official pretrained weights are required"):
        verify_pretrained_weights("resnet18", allow_network=False)


def test_provenance_record_hashes_acquired_cache(tmp_path, monkeypatch) -> None:
    cache = tmp_path / "resnet18-f37072fd.pth"
    cache.write_bytes(b"pretrained-weights")
    import hashlib

    expected = hashlib.sha256(cache.read_bytes()).hexdigest()
    original = BACKBONE_SPECS["resnet18"]
    monkeypatch.setitem(
        BACKBONE_SPECS,
        "resnet18",
        BackboneSpec(
            architecture=original.architecture,
            weight_enum=original.weight_enum,
            source_url=original.source_url,
            license_ref=original.license_ref,
            torchvision_min_version=original.torchvision_min_version,
            expected_sha256=expected,
        ),
    )
    record = provenance_record(
        "resnet18",
        cache_path=cache,
        torchvision_version="0.20.0",
    )
    assert record["pretrained"] is True
    assert record["weight_enum"] == "ResNet18_Weights.IMAGENET1K_V1"
    assert record["sha256"] == provenance_record(
        "resnet18",
        cache_path=cache,
        torchvision_version="0.20.0",
    )["sha256"]
    assert record["cache_path"] == str(cache)
    assert record["product_ready"] is False


def test_provenance_rejects_wrong_official_weight_hash(tmp_path) -> None:
    cache = tmp_path / "resnet18-f37072fd.pth"
    cache.write_bytes(b"tampered")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        provenance_record("resnet18", cache_path=cache)


def test_missing_cache_file_fails_closed(tmp_path) -> None:
    with pytest.raises(FileNotFoundError, match="cache file not found"):
        provenance_record(
            "resnet18",
            cache_path=tmp_path / "missing.pth",
        )


def test_from_scratch_control_is_labelled_ablation_only() -> None:
    record = from_scratch_control_record("resnet18")
    assert record["from_scratch_control"] is True
    assert record["labelled_ablation"] is True
    assert record["product_ready"] is False
    assert record["status"] == "ablation_only"
    assert FROM_SCRATCH_CONTROL_LABEL == "from_scratch_control"


def test_backbone_specs_cover_supported_architectures() -> None:
    assert set(BACKBONE_SPECS) == {
        "resnet18",
        "deeplabv3_resnet50",
        "mobilenet_v3_small",
    }
