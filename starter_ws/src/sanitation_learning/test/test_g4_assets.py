from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_assets import (
    CLASS_ORDER,
    HARD_NEGATIVE_MIN_COUNTS,
    REQUIRED_PAPER_TAXONOMIES,
    TARGET_MIN_COUNTS,
    g4_registry_summary,
    load_g4_asset_registry,
    write_g4_assets,
)


ROOT = Path(__file__).resolve().parents[2]
REGISTRY = (
    ROOT
    / "sanitation_learning"
    / "config"
    / "g4_asset_registry.yaml"
)
GENERATOR = ROOT.parents[1] / "scripts" / "generate_g4_asset_registry.py"


def _find_repository_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "scripts" / "auto05r_g4_capture_all.sh").is_file():
            return candidate
    raise RuntimeError("could not locate repository root")


REPO = _find_repository_root()


def test_registry_counts_and_metadata():
    registry = load_g4_asset_registry(REGISTRY)
    summary = g4_registry_summary(registry)
    assert summary["target_variant_total"] == 166
    assert summary["hard_negative_family_total"] == 84
    for split in ("train", "val", "test"):
        assert (
            summary["hard_negative_family_counts_by_split"][split]
            >= HARD_NEGATIVE_MIN_COUNTS[split]
        )
    required_target = {
        "id",
        "geometry_family",
        "material_family",
        "texture_family",
        "palette",
        "split_eligibility",
        "source",
        "license",
        "sha256",
    }
    required_negative = required_target - {"palette"}
    for class_id in CLASS_ORDER:
        counts = summary["target_variant_counts_by_class"][class_id]
        assert counts["train"] >= TARGET_MIN_COUNTS[class_id]["train"]
        assert counts["val"] >= TARGET_MIN_COUNTS[class_id]["val"]
        assert counts["test"] >= TARGET_MIN_COUNTS[class_id]["test"]
        for variant in registry["classes"][class_id]:
            assert required_target.issubset(variant)
            assert variant["split_eligibility"] in (["train"], ["val"], ["test"])
    for negative in registry["hard_negatives"]:
        assert required_negative.issubset(negative)
        assert negative["taxonomy"]
    train_taxonomies = {
        negative["taxonomy"]
        for negative in registry["hard_negatives"]
        if negative["split_eligibility"] == ["train"]
    }
    assert set(REQUIRED_PAPER_TAXONOMIES).issubset(train_taxonomies)
    leaf_attributes = set(summary["area_attributes_by_class"]["leaf_pile"])
    puddle_attributes = set(summary["area_attributes_by_class"]["puddle"])
    assert {
        "thickness_high",
        "thickness_low",
        "density_sparse",
        "density_dense",
        "condition_wet",
        "condition_dry",
        "leaf_shape_round",
        "leaf_shape_pointed",
        "leaf_shape_lobed",
        "shadow_coverage",
        "partial_occlusion",
        "background_leaf_discrimination",
    }.issubset(leaf_attributes)
    assert {
        "contour_irregular",
        "contour_round",
        "reflectivity_shallow",
        "reflectivity_deep",
        "ground_asphalt",
        "ground_concrete",
        "ground_paving",
        "ground_soil",
        "shadow_coverage",
        "wet_non_puddle",
        "specular_highlight",
        "partial_occlusion",
        "boundary_blur",
    }.issubset(puddle_attributes)


def test_write_g4_assets_produces_sdf_png_and_sha(tmp_path):
    summary = write_g4_assets(REGISTRY, tmp_path)
    assert len(summary["generated_assets"]) == 250
    manifest = json.loads(
        (tmp_path / "g4_generated_asset_manifest.json").read_text(encoding="utf-8")
    )
    assert len(manifest["generated_assets"]) == 250
    first = manifest["generated_assets"][0]
    model_dir = tmp_path / first["asset_id"]
    assert (model_dir / "model.sdf").is_file()
    assert (model_dir / "model.config").is_file()
    textures = list((model_dir / "textures").glob("*.png"))
    assert len(textures) == 1
    sdf = (model_dir / "model.sdf").read_text(encoding="utf-8")
    assert "albedo_map" in sdf
    assert "gz-sim-label-system" in sdf
    assert first["texture_png_sha256"] == (
        hashlib.sha256(textures[0].read_bytes()).hexdigest()
    )
    distinct_textures = {
        item["texture_png_sha256"] for item in manifest["generated_assets"]
    }
    assert len(distinct_textures) >= 100


def test_write_g4_assets_is_deterministic(tmp_path):
    first = write_g4_assets(REGISTRY, tmp_path / "a")
    second = write_g4_assets(REGISTRY, tmp_path / "b")
    assert json.dumps(first, sort_keys=True) == json.dumps(second, sort_keys=True)
    assert first["generated_assets"][0]["texture_png_sha256"] == second[
        "generated_assets"
    ][0]["texture_png_sha256"]


def test_generator_matches_committed_registry_bytes(tmp_path):
    output = tmp_path / "g4_asset_registry.yaml"
    subprocess.run(
        [sys.executable, str(GENERATOR), "--output", str(output)],
        check=True,
        cwd=REPO,
    )
    assert output.read_bytes() == REGISTRY.read_bytes()
    assert yaml.safe_load(output.read_text(encoding="utf-8")) == yaml.safe_load(
        REGISTRY.read_text(encoding="utf-8")
    )


def test_legacy_g3_not_used_as_new_selection_set():
    contract = yaml.safe_load(
        (
            ROOT
            / "sanitation_learning"
            / "config"
            / "auto05r_factorized_diagnostics.yaml"
        ).read_text(encoding="utf-8")
    )
    assert contract["legacy_g3_test_used_as_selection"] is False
    g4_contract = yaml.safe_load(
        (
            ROOT
            / "sanitation_learning"
            / "config"
            / "auto05r_g4_contract.yaml"
        ).read_text(encoding="utf-8")
    )
    assert g4_contract["data_contract"]["legacy_g3_test_used_as_selection"] is False
    assert g4_contract["data_contract"]["test_used_for_model_selection"] is False
    assert g4_contract["data_contract"]["G4_dataset_gate_pass"] is False
