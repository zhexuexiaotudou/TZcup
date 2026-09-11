from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import formal_a12_execution_registry as registry  # noqa: E402


def test_registry_covers_the_fixed_18x10_matrix_with_generator_derived_seeds() -> None:
    value = registry.build_registry(ROOT)
    registry.validate_registry(value, ROOT)
    rows = value["entries"]
    assert len(rows) == 180
    assert len({(row["scenario_id"], row["seed"]) for row in rows}) == 180
    assert len({row["mission_id"] for row in rows}) == 180
    assert len({row["mission_group_id"] for row in rows}) == 30
    first = next(row for row in rows if row["scenario_id"] == "mapping" and row["seed"] == 0)
    assert first["split"] == "hidden"
    assert first["generator_dirt_seed"] == registry.derived_seed(73012026, "hidden", first["map_index"], first["mission_index"], "dirt")
    assert first["source_config_hashes"]["starter_ws/src/sanitation_campus_scenario/sanitation_campus_scenario/generator.py"]


def test_registry_rejects_matrix_collisions_and_source_drift() -> None:
    value = registry.build_registry(ROOT)
    collision = copy.deepcopy(value)
    collision["entries"][1] = copy.deepcopy(collision["entries"][0])
    with pytest.raises(registry.RegistryError, match="collision"):
        registry.validate_registry(collision, ROOT)

    drift = copy.deepcopy(value)
    drift["entries"][0]["source_config_hashes"]["scripts/run_formal_single_episode_cleaning_mission.sh"] = "0" * 64
    with pytest.raises(registry.RegistryError, match="drift"):
        registry.validate_registry(drift, ROOT)


def test_every_feature_is_machine_readable_blocked_until_product_demo_has_an_injection() -> None:
    rows = registry.build_registry(ROOT)["entries"]
    status_by_feature = {
        scenario: {row["expected_injection"]["status"] for row in rows if row["scenario_id"] == scenario}
        for scenario in registry.FEATURE_AUDIT
    }
    assert all(statuses == {"BLOCKED_MISSING_A12_PRODUCT_INJECTION"} for statuses in status_by_feature.values())
    assert any(row["expected_injection"]["existing_component_entrypoints"] for row in rows if row["scenario_id"] == "dynamic_avoidance")
    assert not next(row for row in rows if row["scenario_id"] == "timed_trajectory")["expected_injection"]["existing_component_entrypoints"]


def test_no_replace_writer_and_a12_preflight_refuse_before_reading_legacy_episode(tmp_path: Path) -> None:
    output = tmp_path / "a12_execution_manifest.json"
    registry._write_json_no_replace(output, {"schema": "fixture"})
    assert json.loads(output.read_text(encoding="utf-8"))["schema"] == "fixture"
    with pytest.raises(registry.RegistryError, match="refusing to overwrite"):
        registry._write_json_no_replace(output, {"schema": "fixture"})

    value = registry.build_registry(ROOT)
    with pytest.raises(registry.RegistryError, match="blocked before operator_start"):
        registry.write_execution_manifest(
            registry=value,
            repository_root=ROOT,
            run_root=tmp_path,
            output=tmp_path / "would_not_exist.json",
            scenario_id="mapping",
            seed=0,
            episode_manifest=tmp_path / "missing-episode.json",
            evaluator_manifest=tmp_path / "missing-evaluator.json",
        )
    assert not (tmp_path / "would_not_exist.json").exists()
