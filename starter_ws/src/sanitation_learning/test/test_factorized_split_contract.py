from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from sanitation_learning.factorized_diagnostics import (
    REQUIRED_METRICS,
    diagnosis_ids_for_row,
    load_diagnostic_config,
    validate_factorized_metrics_report,
)


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "auto05r_factorized_diagnostics.yaml"
)


def test_config_defines_d1_to_d6_and_keeps_legacy_test_out_of_selection() -> None:
    config = load_diagnostic_config(CONFIG)
    assert set(config["diagnoses"]) == {"D1", "D2", "D3", "D4", "D5", "D6"}
    assert config["legacy_g3_test_used_as_selection"] is False
    for diagnosis_id in ("D1", "D2", "D3", "D4", "D5", "D6"):
        assert config["diagnoses"][diagnosis_id]["name"]
        assert config["diagnoses"][diagnosis_id]["description"]
        assert config["diagnoses"][diagnosis_id]["filter"]
        for metric in REQUIRED_METRICS:
            assert metric in config["diagnoses"][diagnosis_id]["required_metrics"]


def test_load_rejects_legacy_test_as_selection() -> None:
    config = deepcopy(load_diagnostic_config(CONFIG))
    config["legacy_g3_test_used_as_selection"] = True
    with pytest.raises(ValueError, match="legacy_g3_test_used_as_selection"):
        _validate_config_round_trip(config)


def _validate_config_round_trip(config: dict) -> None:
    path = Path(__file__).parent / "_round_trip.yaml"
    import yaml

    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    try:
        return load_diagnostic_config(path)
    finally:
        path.unlink(missing_ok=True)


def test_diagnosis_ids_for_empty_seen_metadata() -> None:
    config = load_diagnostic_config(CONFIG)
    row = {
        "world_id": "world_x",
        "asset_family": "asset_x",
        "geometry_family": "geometry_x",
        "material_id": "material_x",
        "lighting_family": "lighting_x",
        "trajectory_id": "trajectory_x",
        "negative_only": True,
        "negative_asset_family": "negative_x",
    }
    assert sorted(diagnosis_ids_for_row(row, config)) == ["D5", "D6"]


def test_diagnosis_ids_with_seen_metadata() -> None:
    config = deepcopy(load_diagnostic_config(CONFIG))
    config["seen_metadata"]["seen_world_ids"] = ["world_a"]
    config["seen_metadata"]["seen_asset_families"] = ["asset_a"]
    config["seen_metadata"]["seen_material_ids"] = ["material_a"]
    config["seen_metadata"]["seen_geometry_families"] = ["geometry_a"]
    config["seen_metadata"]["seen_lighting_families"] = ["lighting_a"]
    config["seen_metadata"]["seen_trajectory_ids"] = ["trajectory_a"]
    config["seen_metadata"]["seen_negative_asset_families"] = ["negative_a"]

    same_world_unseen_asset = {
        "world_id": "world_a",
        "asset_family": "asset_b",
        "geometry_family": "geometry_a",
        "material_id": "material_a",
        "lighting_family": "lighting_a",
        "trajectory_id": "trajectory_a",
        "negative_only": False,
        "negative_asset_family": None,
    }
    assert diagnosis_ids_for_row(same_world_unseen_asset, config) == ["D1"]

    unseen_world_seen_asset = {
        "world_id": "world_b",
        "asset_family": "asset_a",
        "geometry_family": "geometry_a",
        "material_id": "material_a",
        "lighting_family": "lighting_a",
        "trajectory_id": "trajectory_a",
        "negative_only": False,
        "negative_asset_family": None,
    }
    assert diagnosis_ids_for_row(unseen_world_seen_asset, config) == ["D2"]

    unseen_material_seen_geometry = {
        "world_id": "world_a",
        "asset_family": "asset_a",
        "geometry_family": "geometry_a",
        "material_id": "material_b",
        "lighting_family": "lighting_a",
        "trajectory_id": "trajectory_a",
        "negative_only": False,
        "negative_asset_family": None,
    }
    assert diagnosis_ids_for_row(unseen_material_seen_geometry, config) == ["D3"]

    unseen_lighting_seen_asset = {
        "world_id": "world_a",
        "asset_family": "asset_a",
        "geometry_family": "geometry_a",
        "material_id": "material_a",
        "lighting_family": "lighting_b",
        "trajectory_id": "trajectory_a",
        "negative_only": False,
        "negative_asset_family": None,
    }
    assert diagnosis_ids_for_row(unseen_lighting_seen_asset, config) == ["D4"]

    unseen_negative_asset = {
        "world_id": "world_a",
        "asset_family": "asset_a",
        "geometry_family": "geometry_a",
        "material_id": "material_a",
        "lighting_family": "lighting_a",
        "trajectory_id": "trajectory_a",
        "negative_only": True,
        "negative_asset_family": "negative_b",
    }
    assert diagnosis_ids_for_row(unseen_negative_asset, config) == ["D5"]

    full_unseen = {
        "world_id": "world_b",
        "asset_family": "asset_b",
        "geometry_family": "geometry_b",
        "material_id": "material_b",
        "lighting_family": "lighting_b",
        "trajectory_id": "trajectory_b",
        "negative_only": True,
        "negative_asset_family": "negative_b",
    }
    assert diagnosis_ids_for_row(full_unseen, config) == ["D5", "D6"]


def test_validate_factorized_metrics_report_accepts_valid_report() -> None:
    config = load_diagnostic_config(CONFIG)
    entry = {metric: 0.5 for metric in REQUIRED_METRICS}
    entry["negative_fp_per_frame"] = 0.0
    report = {"diagnoses": {diagnosis_id: dict(entry) for diagnosis_id in config["diagnoses"]}}
    assert validate_factorized_metrics_report(report, config) == []


def test_validate_factorized_metrics_report_detects_missing_and_bad_values() -> None:
    config = load_diagnostic_config(CONFIG)
    entry = {metric: 0.5 for metric in REQUIRED_METRICS}
    entry["negative_fp_per_frame"] = 0.0
    report = {"diagnoses": {diagnosis_id: dict(entry) for diagnosis_id in config["diagnoses"]}}
    report["diagnoses"]["D1"].pop("leaf_iou")
    report["diagnoses"]["D2"]["ap50"] = "high"
    report["diagnoses"]["D3"]["macro_f1"] = None  # explicit null is allowed
    errors = validate_factorized_metrics_report(report, config)
    assert any("D1" in error and "leaf_iou" in error for error in errors)
    assert any("D2" in error and "ap50" in error for error in errors)
    assert not any("D3" in error for error in errors)


def test_validate_factorized_metrics_report_rejects_missing_diagnosis() -> None:
    config = load_diagnostic_config(CONFIG)
    errors = validate_factorized_metrics_report({"diagnoses": {}}, config)
    assert any("D1" in error for error in errors)
