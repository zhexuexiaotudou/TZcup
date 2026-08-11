from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def _load_benchmark():
    pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    path = ROOT / "scripts" / "perception_oprv3_moving_benchmark.py"
    spec = importlib.util.spec_from_file_location("oprv3_moving_benchmark", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_capture_projection_reconstructs_optical_axis_without_gt(tmp_path: Path):
    benchmark = _load_benchmark()
    camera = tmp_path / "camera.json"
    camera.write_text(
        '{"k":[300,0,320,0,300,240,0,0,1]}', encoding="utf-8"
    )
    transform = tmp_path / "tf.json"
    transform.write_text(
        '{"base_to_camera_xyz_m":[0.36,0.0,0.66]}', encoding="utf-8"
    )
    row = {
        "camera_path": camera,
        "tf_path": transform,
        "capture_record": {
            "vehicle_xy_m": [1.0, 2.0],
            "vehicle_yaw_rad": 0.0,
        },
    }
    calibration, matrix = benchmark.camera_projection_inputs(
        row, camera_pitch_down_rad=0.0
    )
    assert calibration["fx"] == 300.0
    assert matrix[:3, 3] == pytest.approx([1.36, 2.0, 0.66])
    assert matrix[:3, 2] == pytest.approx([1.0, 0.0, 0.0])
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
    _calibration, pitched = benchmark.camera_projection_inputs(
        row, camera_pitch_down_rad=np.deg2rad(50.0)
    )
    assert pitched[:3, 2] == pytest.approx(
        [np.cos(np.deg2rad(50.0)), 0.0, -np.sin(np.deg2rad(50.0))]
    )
    assert np.linalg.det(pitched[:3, :3]) == pytest.approx(1.0)


def test_nearest_gt_is_evaluator_only_and_distance_bounded():
    benchmark = _load_benchmark()
    targets = [
        {"target_id": "a", "world_xyz_m": [1.0, 2.0, 0.0]},
        {"target_id": "b", "world_xyz_m": [4.0, 5.0, 0.0]},
    ]
    target, distance = benchmark._nearest_gt(1.1, 2.0, targets, 0.5)
    assert target["target_id"] == "a"
    assert distance == pytest.approx(0.1)
    target, distance = benchmark._nearest_gt(2.0, 2.0, targets, 0.5)
    assert target is None
    assert distance == pytest.approx(1.0)


def test_product_map_scores_all_targets_that_entered_camera_frustum():
    benchmark = _load_benchmark()
    encounters = [
        {
            "target_id": "actionable",
            "scene_seed": 2,
            "ever_in_camera_frustum": True,
            "entered_actionable_window": True,
        },
        {
            "target_id": "visible_but_not_actionable",
            "scene_seed": 2,
            "ever_in_camera_frustum": True,
            "entered_actionable_window": False,
        },
        {
            "target_id": "never_visible",
            "scene_seed": 2,
            "ever_in_camera_frustum": False,
            "entered_actionable_window": False,
        },
    ]
    actionable_groups, map_scorable_groups = (
        benchmark._partition_product_map_encounters(encounters)
    )
    actionable = actionable_groups[2]
    map_scorable = map_scorable_groups[2]
    assert [item["target_id"] for item in actionable] == ["actionable"]
    assert [item["target_id"] for item in map_scorable] == [
        "actionable",
        "visible_but_not_actionable",
    ]


def test_area_morphology_is_frozen_and_unknown_values_fail():
    benchmark = _load_benchmark()
    mask = np.zeros((9, 9), dtype=bool)
    mask[2:7, 2:7] = True
    mask[0, 0] = True
    opened = benchmark.apply_area_morphology(mask, "open3")
    assert not opened[0, 0]
    assert opened[4, 4]
    with pytest.raises(ValueError, match="unsupported frozen"):
        benchmark.apply_area_morphology(mask, "future_operator")


def test_area_gate_binds_selected_postprocess_to_exact_checkpoints(tmp_path: Path):
    benchmark = _load_benchmark()
    leaf = tmp_path / "leaf.pt"
    puddle = tmp_path / "puddle.pt"
    leaf.write_bytes(b"leaf")
    puddle.write_bytes(b"puddle")
    gate = tmp_path / "OPRV3_AREA_GATE.json"
    payload = {
        "protocol": "OPRV3-06",
        "OPRV3_06_AREA_PASS": True,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "models": {
            "leaf": {"sha256": benchmark.sha256(leaf)},
            "puddle": {"sha256": benchmark.sha256(puddle)},
        },
        "selected_config": {
            "by_class": {
                "leaf_pile": {"threshold": 0.8, "morphology": "open3"},
                "puddle": {"threshold": 0.6, "morphology": "close3"},
            }
        },
    }
    gate.write_text(json.dumps(payload), encoding="utf-8")
    configs, provenance = benchmark.load_area_gate(
        gate, leaf_checkpoint=leaf, puddle_checkpoint=puddle
    )
    assert configs == {
        "leaf_pile": {"threshold": 0.8, "morphology": "open3"},
        "puddle": {"threshold": 0.6, "morphology": "close3"},
    }
    assert provenance["OPRV3_06_AREA_PASS"] is True
    leaf.write_bytes(b"changed")
    with pytest.raises(RuntimeError, match="leaf hash mismatch"):
        benchmark.load_area_gate(
            gate, leaf_checkpoint=leaf, puddle_checkpoint=puddle
        )


def test_runtime_source_commit_injection_is_strict(monkeypatch):
    benchmark = _load_benchmark()
    monkeypatch.setenv("TZCUP_SOURCE_COMMIT", "A" * 40)
    assert benchmark.repository_commit() == "a" * 40
    monkeypatch.setenv("TZCUP_SOURCE_COMMIT", "not-a-commit")
    with pytest.raises(RuntimeError, match="40-character"):
        benchmark.repository_commit()


def test_scheduler_uses_fresh_online_target_and_frozen_safe_boundary():
    benchmark = _load_benchmark()
    manifest = {
        "runtime": {
            "dynamic_trash_map": {
                "association_distance_m": 0.30,
                "confirmation_observations": 3,
                "confirmation_class_posterior": 0.70,
                "confirmation_confidence": 0.60,
                "maximum_covariance_trace": 0.03,
                "lost_after_s": 1.0,
                "reject_after_s": 5.0,
                "maximum_observation_history": 64,
            }
        }
    }
    config = benchmark.DynamicTrashMapConfig(
        **manifest["runtime"]["dynamic_trash_map"]
    )
    dynamic_map = benchmark.DynamicTrashMap.start_new("mission", config=config)

    class Target:
        uuid = "target"
        track_state = benchmark.TargetState.CONFIRMED
        task_state = benchmark.TargetState.CONFIRMED
        transitions = []
        confidence = 0.95
        observation_count = 4
        map_x_m = 0.30
        map_y_m = 0.0
        covariance_trace = 0.01
        source_models = ["MRV2-A-oprv3-development"]
        current_class = "metal_can"

    target = Target()
    dynamic_map.targets[target.uuid] = target
    decision = benchmark.schedule_current_target(
        benchmark.CleaningTaskScheduler(),
        dynamic_map,
        target,
        {
            "frame_index": 20,
            "timestamp_ns": 2_000_000_000,
            "vehicle_xy_m": [0.0, 0.0],
        },
    )
    assert decision["action"] == "CLEAN_NOW"
    assert decision["fresh_online_observation"] is True
    assert decision["coverage_safe_boundary"] is True
    assert target.track_state == benchmark.TargetState.SCHEDULED
