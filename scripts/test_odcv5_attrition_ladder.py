from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _module():
    path = ROOT / "scripts" / "audit_odcv5_online_attrition.py"
    spec = importlib.util.spec_from_file_location("odcv5_attrition", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _payload() -> dict:
    frame = {
        "visible": True,
        "actionable_window": True,
        "observation_created": True,
        "action_detection": True,
        "correct_action_detection": True,
        "model_score": 0.9,
        "visible_bbox_short_side_px": 12.0,
        "distance_m": 1.2,
        "depth_valid_ratio": 1.0,
    }
    encounter = {
        "target_id": "can-1",
        "class_name": "metal_can",
        "scene_seed": 4,
        "world_id": "wet_world",
        "world_xyz_m": [1.0, 2.0, 0.0],
        "occlusion_bucket": "partial",
        "visible_fraction_bucket": "partial",
        "ever_in_camera_frustum": True,
        "entered_actionable_window": True,
        "frames": [frame],
    }
    product = {
        "uuid": "map-1",
        "class_name": "metal_can",
        "x_m": 1.01,
        "y_m": 2.01,
        "ever_confirmed": True,
        "transitions": [{"to": "TRACKED"}, {"to": "CONFIRMED"}],
    }
    return {
        "source_commit": "a" * 40,
        "G5_SEALED_FINAL_read": False,
        "routes": {
            "D1-B": {
                "encounters": [encounter],
                "product_map": {"missions": [{"scene_seed": 4, "product_targets": [product]}]},
            }
        },
    }


def test_attrition_keeps_scheduler_unknown_for_legacy_evidence(tmp_path: Path):
    module = _module()
    source = tmp_path / "benchmark.json"
    source.write_text("{}", encoding="utf-8")
    overall, by_class, by_domain, root_cause = module.build_reports(
        _payload(), route="D1-B", input_path=source
    )
    target = overall["targets"][0]
    assert target["stage_pass"]["DYNAMIC_MAP_CONFIRMED"] is True
    assert target["stage_pass"]["SCHEDULER_ACTIONABLE"] is None
    assert overall["summary"]["stages"]["SCHEDULER_ACTIONABLE"]["count_unknown"] == 1
    assert by_class["groups"]["metal_can"]["target_count"] == 1
    assert "size:small_lt18px" in by_domain["groups"]
    assert root_cause["training_allowed"] is False
    assert root_cause["unresolved_legacy_trace_gap_counts"]["SCHEDULER_REJECT"] == 1


def test_attrition_is_conditional_and_does_not_count_upstream_loss_twice(tmp_path: Path):
    module = _module()
    payload = _payload()
    payload["routes"]["D1-B"]["encounters"][0]["frames"][0].update(
        observation_created=False,
        action_detection=False,
        correct_action_detection=False,
    )
    payload["routes"]["D1-B"]["product_map"]["missions"][0]["product_targets"] = []
    source = tmp_path / "benchmark.json"
    source.write_text("{}", encoding="utf-8")
    overall, _, _, root_cause = module.build_reports(payload, route="D1-B", input_path=source)
    stages = overall["summary"]["stages"]
    assert stages["NATIVE_DETECTOR_OBSERVATION"]["count_lost"] == 1
    assert stages["NATIVE_DETECTOR_ACTION_THRESHOLD"]["count_in"] == 0
    assert overall["targets"][0]["root_cause"]["status"] == "UNRESOLVED_LEGACY_TRACE_GAP"
    assert root_cause["unresolved_legacy_trace_gap_counts"][
        "DETECTOR_NO_PROPOSAL+DETECTOR_BOX_IOU_FAIL"
    ] == 1


def test_wrong_class_nearby_product_cannot_resurrect_target(tmp_path: Path):
    module = _module()
    payload = _payload()
    encounter = payload["routes"]["D1-B"]["encounters"][0]
    encounter["frames"][0]["correct_action_detection"] = False
    product = payload["routes"]["D1-B"]["product_map"]["missions"][0]["product_targets"][0]
    product["class_name"] = "paper_litter"
    source = tmp_path / "benchmark.json"
    source.write_text("{}", encoding="utf-8")
    overall, _, _, root_cause = module.build_reports(payload, route="D1-B", input_path=source)
    target = overall["targets"][0]
    assert target["stage_pass"]["CORRECT_CLASS"] is False
    assert target["stage_pass"]["PROJECTION_SUCCESS"] is False
    assert target["stage_pass"]["DYNAMIC_MAP_CONFIRMED"] is False
    assert target["root_cause"]["taxonomy"] == "DETECTOR_WRONG_CLASS"
    assert root_cause["primary_directly_attributed_loss"] == "DETECTOR_WRONG_CLASS"


def test_sealed_final_input_is_rejected(tmp_path: Path):
    module = _module()
    payload = _payload()
    payload["G5_SEALED_FINAL_read"] = True
    source = tmp_path / "benchmark.json"
    source.write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError, match="sealed-final"):
        module.build_reports(payload, route="D1-B", input_path=source)


def test_existing_ladder_can_recover_root_cause_without_missing_raw_report(tmp_path: Path):
    module = _module()
    source = tmp_path / "source.json"
    source.write_text("{}", encoding="utf-8")
    overall, _, _, expected = module.build_reports(_payload(), route="D1-B", input_path=source)
    for record in overall["targets"]:
        record.pop("root_cause")
    records = overall["targets"]
    for record in records:
        record["root_cause"] = module._root_cause(record["stage_pass"])
    shared = {key: overall[key] for key in (
        "schema_version", "protocol", "stage", "source_commit", "route", "input",
        "GT_used_by_product_pipeline", "GT_used_only_by_attrition_evaluator",
        "G5_SEALED_FINAL_read", "legacy_scheduler_target_attribution_available",
        "scheduler_unknown_is_not_a_pass",
    )}
    recovered = module._root_cause_decision(records, shared)
    assert recovered["decision"] == expected["decision"]
    assert recovered["training_allowed"] is False
