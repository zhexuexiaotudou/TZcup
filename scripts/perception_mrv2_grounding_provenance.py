#!/usr/bin/env python3
"""Verify the newly available official Grounding DINO Swin-T checkpoint."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import urllib.request

try:
    import torch
except ImportError:
    torch = None


OFFICIAL_REPOSITORY = "https://github.com/IDEA-Research/GroundingDINO"
OFFICIAL_RELEASE_API = "https://api.github.com/repos/IDEA-Research/GroundingDINO/releases/tags/v0.1.0-alpha"
OFFICIAL_CHECKPOINT_URL = "https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth"
OFFICIAL_CHECKPOINT_NAME = "groundingdino_swint_ogc.pth"
OFFICIAL_CHECKPOINT_BYTES = 693_997_677
RELEASE_TAG_COMMIT = "ddedf74b250249e0ae81f3781cbf98b3b4d3cb88"
RUNTIME_SOURCE_COMMIT = "856dde20aee659246248e20734ef9ba5214f5e44"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json_url(url: str) -> tuple[dict, dict]:
    request = urllib.request.Request(url, headers={"User-Agent": "Codex-MRV2-Provenance"})
    with urllib.request.urlopen(request, timeout=30) as response:
        headers = {key.lower(): value for key, value in response.headers.items()}
        return json.load(response), {"status": response.status, "url": response.url, "headers": headers}


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    if torch is None:
        raise RuntimeError("checkpoint structure verification requires PyTorch")
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--runtime-source-archive", type=Path, required=True)
    parser.add_argument("--release-source-archive", type=Path, required=True)
    parser.add_argument("--runtime-source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    release, http = read_json_url(OFFICIAL_RELEASE_API)
    assets = [item for item in release.get("assets", ()) if item.get("name") == OFFICIAL_CHECKPOINT_NAME]
    official_asset = assets[0] if len(assets) == 1 else None
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    model = checkpoint.get("model") if isinstance(checkpoint, dict) else None
    license_path = args.runtime_source_root / "LICENSE"
    license_text = license_path.read_text(encoding="utf-8") if license_path.is_file() else ""
    gates = {
        "single_matching_official_release_asset": official_asset is not None,
        "official_release_asset_url_match": bool(official_asset and official_asset.get("browser_download_url") == OFFICIAL_CHECKPOINT_URL),
        "official_release_asset_size_match": bool(official_asset and int(official_asset.get("size", -1)) == OFFICIAL_CHECKPOINT_BYTES),
        "local_filename_match": args.checkpoint.name == OFFICIAL_CHECKPOINT_NAME,
        "local_size_match": args.checkpoint.stat().st_size == OFFICIAL_CHECKPOINT_BYTES,
        "checkpoint_is_model_state_dict": isinstance(model, dict) and len(model) >= 900,
        "checkpoint_tensor_prefix_match": isinstance(model, dict) and any(str(key).startswith("module.transformer") for key in model),
        "runtime_source_license_apache_2_0": "Apache License" in license_text and "Version 2.0" in license_text,
        "runtime_source_archive_nonempty": args.runtime_source_archive.stat().st_size > 0,
        "release_source_archive_nonempty": args.release_source_archive.stat().st_size > 0,
    }
    report = {
        "schema_version": 1,
        "stage": "MRV2-05-GROUNDING-DINO-PROVENANCE",
        "official": {
            "repository": OFFICIAL_REPOSITORY,
            "release_tag": "v0.1.0-alpha",
            "release_tag_commit": RELEASE_TAG_COMMIT,
            "runtime_source_commit": RUNTIME_SOURCE_COMMIT,
            "checkpoint_url": OFFICIAL_CHECKPOINT_URL,
            "release_asset": {
                key: official_asset.get(key) if official_asset else None
                for key in ("id", "name", "size", "browser_download_url", "created_at", "updated_at", "content_type")
            },
            "release_api_http": http,
        },
        "local_checkpoint": {
            "path": args.checkpoint.as_posix(),
            "bytes": args.checkpoint.stat().st_size,
            "sha256": sha256(args.checkpoint),
            "top_level_keys": sorted(checkpoint) if isinstance(checkpoint, dict) else [],
            "model_tensor_count": len(model) if isinstance(model, dict) else 0,
            "first_model_keys": list(model)[:10] if isinstance(model, dict) else [],
        },
        "source_archives": {
            "release_tag": {"path": args.release_source_archive.as_posix(), "bytes": args.release_source_archive.stat().st_size, "sha256": sha256(args.release_source_archive)},
            "runtime_source": {"path": args.runtime_source_archive.as_posix(), "bytes": args.runtime_source_archive.stat().st_size, "sha256": sha256(args.runtime_source_archive)},
            "license_path": license_path.as_posix(),
            "license_sha256": sha256(license_path) if license_path.is_file() else None,
        },
        "gates": gates,
        "GROUNDING_DINO_OFFICIAL_CHECKPOINT_PROVENANCE_VERIFIED": all(gates.values()),
        "checkpoint_publisher_digest_available": bool(official_asset and official_asset.get("digest")),
        "redistribution_status": "reference_only_pending_explicit_checkpoint_license_statement",
        "shipped_in_product": False,
        "benchmark_executed": False,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }
    write_json(args.output, report)
    return 0 if report["GROUNDING_DINO_OFFICIAL_CHECKPOINT_PROVENANCE_VERIFIED"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
