"""Pretrained-backbone provenance contracts.

Production candidates must acquire official torchvision weights through the
enum API (``weights=ResNet18_Weights.IMAGENET1K_V1``) and never silently fall
back to ``pretrained=False``.  Provenance records carry the exact weight enum,
source URL, license, architecture and, when a cache file is available, the
SHA-256 of the acquired artifact.  ``from_scratch_control`` is an explicitly
labelled ablation that can never produce product-ready status.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import urlparse


PRETRAINED_REQUIRED = True
FROM_SCRATCH_CONTROL_LABEL = "from_scratch_control"


@dataclass(frozen=True)
class BackboneSpec:
    architecture: str
    weight_enum: str
    source_url: str
    license_ref: str
    torchvision_min_version: str
    expected_sha256: str | None = None


BACKBONE_SPECS = {
    "resnet18": BackboneSpec(
        architecture="resnet18",
        weight_enum="ResNet18_Weights.IMAGENET1K_V1",
        source_url="https://download.pytorch.org/models/resnet18-f37072fd.pth",
        license_ref="BSD-3-Clause (PyTorch ImageNet weights)",
        torchvision_min_version="0.13",
        expected_sha256=(
            "f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec"
        ),
    ),
    "deeplabv3_resnet50": BackboneSpec(
        architecture="deeplabv3_resnet50",
        weight_enum=(
            "DeepLabV3_ResNet50_Weights.COCO_WITH_VOC_LABELS_V1"
        ),
        source_url=(
            "https://download.pytorch.org/models/"
            "deeplabv3_resnet50_coco-cd0a2569.pth"
        ),
        license_ref=(
            "BSD-3-Clause / COCO dataset license (see torchvision model zoo)"
        ),
        torchvision_min_version="0.13",
        expected_sha256=(
            "cd0a25694c4a0f7106b38f4938bf90a874f2f241cc410b8f63c7024399538f06"
        ),
    ),
    "mobilenet_v3_small": BackboneSpec(
        architecture="mobilenet_v3_small",
        weight_enum="MobileNet_V3_Small_Weights.IMAGENET1K_V1",
        source_url=(
            "https://download.pytorch.org/models/"
            "mobilenet_v3_small-047dcff4.pth"
        ),
        license_ref="BSD-3-Clause (PyTorch ImageNet weights)",
        torchvision_min_version="0.13",
        expected_sha256=(
            "047dcff4addef86ea5bc2eff13c9614dc11f47ab1160d0a71a25e7db994f4e1f"
        ),
    ),
}


def pretrained_backbone_spec(architecture: str) -> BackboneSpec:
    try:
        return BACKBONE_SPECS[architecture]
    except KeyError as exc:
        raise ValueError(
            f"unknown pretrained backbone architecture {architecture!r}"
        ) from exc


def provenance_record(
    architecture: str,
    *,
    cache_path=None,
    from_scratch_control: bool = False,
    torchvision_version: str | None = None,
) -> dict:
    """Build the provenance record for a backbone acquisition.

    When ``cache_path`` points at an acquired weight file its SHA-256 is
    computed and recorded.  If an expected hash is pinned in the spec and the
    cache file does not match, this raises fail-closed.
    """
    if from_scratch_control:
        return {
            "pretrained": False,
            "from_scratch_control": True,
            "labelled_ablation": True,
            "product_ready": False,
            "status": "ablation_only",
            "architecture": architecture,
            "torchvision_version": torchvision_version,
        }
    spec = pretrained_backbone_spec(architecture)
    record: dict = {
        "pretrained": True,
        "weight_enum": spec.weight_enum,
        "source_url": spec.source_url,
        "license": spec.license_ref,
        "architecture": spec.architecture,
        "torchvision_min_version": spec.torchvision_min_version,
        "torchvision_version": torchvision_version,
        "cache_path": None,
        "sha256": None,
        "from_scratch_control": False,
        "product_ready": False,
    }
    if cache_path is not None:
        cache = Path(cache_path)
        if not cache.is_file():
            raise FileNotFoundError(
                f"pretrained weight cache file not found: {cache}"
            )
        record["cache_path"] = str(cache)
        record["sha256"] = _file_sha256(cache)
        if spec.expected_sha256 is not None and (
            record["sha256"] != spec.expected_sha256
        ):
            raise ValueError(
                f"pretrained weight SHA-256 mismatch for {architecture}: "
                f"{record['sha256']}"
            )
    return record


def verify_pretrained_weights(
    architecture: str,
    *,
    cache_path=None,
    allow_network: bool = False,
    torchvision_version: str | None = None,
) -> dict:
    """Fail-closed verification of official pretrained weights.

    Unit tests never download: without a cache file and with
    ``allow_network=False`` this raises, guaranteeing that tests exercise the
    fail-closed path only.
    """
    if cache_path is None:
        raise RuntimeError(
            "official pretrained weights are required "
            f"({architecture}); an acquired cache file is required for "
            "exact SHA-256 verification"
        )
    return provenance_record(
        architecture,
        cache_path=cache_path,
        torchvision_version=torchvision_version,
    )


def torchvision_cache_path(architecture: str) -> Path:
    """Return the official torch-hub cache path for a weight enum artifact."""
    import torch

    spec = pretrained_backbone_spec(architecture)
    filename = Path(urlparse(spec.source_url).path).name
    if not filename:
        raise ValueError(
            f"official weight URL has no filename: {spec.source_url}"
        )
    return Path(torch.hub.get_dir()) / "checkpoints" / filename


def _file_sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def from_scratch_control_record(architecture: str) -> dict:
    """Explicit ablation record; never product-ready."""
    return provenance_record(
        architecture, from_scratch_control=True
    )


__all__ = [
    "BACKBONE_SPECS",
    "BackboneSpec",
    "FROM_SCRATCH_CONTROL_LABEL",
    "PRETRAINED_REQUIRED",
    "from_scratch_control_record",
    "pretrained_backbone_spec",
    "provenance_record",
    "verify_pretrained_weights",
    "torchvision_cache_path",
]
