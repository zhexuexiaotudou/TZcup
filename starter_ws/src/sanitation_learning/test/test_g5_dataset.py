from __future__ import annotations

import json
from pathlib import Path

from sanitation_learning.g4_assets import (
    CLASS_ORDER, REQUIRED_PAPER_TAXONOMIES, load_g4_asset_registry,
)
from sanitation_learning.g4_scene import negative_only_rule
from sanitation_learning.g5_dataset import (
    G5_WORLD_PROFILES, SEALED_SPLIT, _dataset_tree_digest, derive_g5_registry,
    finalize_g5_dataset, generate_g5_worlds,
)


PACKAGE = Path(__file__).resolve().parents[1]
REGISTRY = PACKAGE / "config" / "g4_asset_registry.yaml"
XACRO = (
    PACKAGE.parents[0]
    / "sanitation_vehicle_description"
    / "urdf"
    / "sanitation_vehicle.urdf.xacro"
)


def test_g5_registry_uses_only_new_sealed_asset_ids() -> None:
    development = load_g4_asset_registry(REGISTRY)
    sealed = derive_g5_registry(development)
    development_ids = {
        item["id"]
        for class_id in CLASS_ORDER
        for item in development["classes"][class_id]
    } | {item["id"] for item in development["hard_negatives"]}
    sealed_targets = {
        item["id"]
        for class_id in CLASS_ORDER
        for item in sealed["classes"][class_id]
    }
    sealed_negatives = {item["id"] for item in sealed["hard_negatives"]}
    assert not development_ids & (sealed_targets | sealed_negatives)
    assert all(
        item["split_eligibility"] == [SEALED_SPLIT]
        for class_id in CLASS_ORDER
        for item in sealed["classes"][class_id]
    )
    assert set(REQUIRED_PAPER_TAXONOMIES) <= {
        item["taxonomy"] for item in sealed["hard_negatives"]
    }


def test_g5_world_generator_has_four_new_world_hashes(tmp_path: Path) -> None:
    manifest = generate_g5_worlds(
        REGISTRY, tmp_path / "models", XACRO, tmp_path / "worlds"
    )
    assert len(manifest["worlds"]) == 4 == len(G5_WORLD_PROFILES)
    assert len({item["world_id"] for item in manifest["worlds"]}) == 4
    assert len({item["sha256"] for item in manifest["worlds"]}) == 4
    assert all(item["split_eligibility"] == [SEALED_SPLIT] for item in manifest["worlds"])
    assert manifest["development_access_forbidden"] is True
    generated = json.loads(
        (tmp_path / "models" / "g5_generated_asset_manifest.json").read_text()
    )
    assert generated["dataset_id"] == SEALED_SPLIT


def test_g5_negative_only_prior_is_supported_without_becoming_development_data() -> None:
    hits = sum(negative_only_rule(SEALED_SPLIT, index) for index in range(25))
    assert hits == 7


def test_g5_dataset_tree_digest_binds_path_content_and_size(tmp_path: Path) -> None:
    payload = tmp_path / "scenes" / "scene_5000" / "rgb.npy"
    payload.parent.mkdir(parents=True)
    payload.write_bytes(b"first")
    first = _dataset_tree_digest(tmp_path)
    assert first["file_count"] == 1
    assert first["total_bytes"] == 5
    payload.write_bytes(b"other")
    second = _dataset_tree_digest(tmp_path)
    assert second["sha256"] != first["sha256"]


def test_g5_finalize_rejects_world_file_hash_drift(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    manifest = generate_g5_worlds(REGISTRY, root / "models", XACRO, root / "worlds")
    world_path = root / "worlds" / manifest["worlds"][0]["path"]
    world_path.write_text(world_path.read_text() + "\n<!-- tampered -->\n")
    development = tmp_path / "development_worlds.json"
    development.write_text(
        json.dumps({"worlds": [], "assets": [], "negative_assets": []})
    )
    qa = finalize_g5_dataset(root, tmp_path / "qa", development)
    assert qa["gates"]["world_file_hashes_match_manifest"] is False
    assert qa["G5_dataset_gate_pass"] is False
    assert qa["world_file_mismatches"][0]["world_id"] == manifest["worlds"][0]["world_id"]
