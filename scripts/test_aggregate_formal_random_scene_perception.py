import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import aggregate_formal_random_scene_perception as perception  # noqa: E402

aggregate = perception.aggregate


def _report(episode_id: str, status: str = "PASSED") -> dict:
    return {
        "report_id": "tzcup_formal_random_scene_perception_episode_v1",
        "episode_id": episode_id,
        "status": status,
        "litter_cube_detection": {"precision": 0.9, "recall": 0.85, "f1": 0.87},
        "ground_dirt_segmentation": {"iou": 0.7, "recall": 0.9},
        "map_projection": {"rmse_m": 0.1, "p95_m": 0.2, "false_product_track_count": 0},
        "sensor_runtime": {"real_camera_message_count": 10},
        "truth_isolation": {
            "truth_published_to_ros": False,
            "truth_used_by_product_control": False,
            "synthetic_offline_image_used": False,
        },
    }


def _formal_episode_id(index: int) -> str:
    return f"val-map-{index % 8:03d}-mission-{index // 8:03d}"


def test_aggregate_requires_full_validation_matrix_for_formal_pc_evidence(tmp_path: Path):
    (tmp_path / "product_source_manifest.sha256").write_text(
        "a" * 64 + "  pc_open_vocab_adapter.py\n", encoding="utf-8"
    )
    paths = []
    for index in range(30):
        path = tmp_path / f"episode-{index}.json"
        path.write_text(json.dumps(_report(_formal_episode_id(index))), encoding="utf-8")
        paths.append(path)
    report = aggregate(paths, 30)
    assert report["status"] == "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_PASSED"
    assert report["gates"]["minimum_disjoint_episode_count"]
    assert report["gates"]["all_required_validation_maps_covered"]
    assert report["gates"]["minimum_episodes_per_validation_map"]
    assert report["statistical_scope"]["episode_count_by_validation_map"]["val-map-000"] == 4
    assert report["statistical_scope"]["smoke_eligible_for_final_product_evidence"] is False
    assert report["claim_boundary"]["s100_board_accepted"] is False
    assert report["claim_boundary"]["real_world_accuracy_claimed"] is False
    assert len(report["episodes"][0]["report_sha256"]) == 64
    assert "path" not in report["episodes"][0]
    assert report["episodes"][0]["litter_cube_detection"]["f1"] == 0.87
    assert report["episodes"][0]["artifact_evidence"][
        "product_source_manifest_entries"
    ] == ["a" * 64 + "  pc_open_vocab_adapter.py"]


def test_aggregate_blocks_smoke_scale_input_even_when_the_caller_requests_three(tmp_path: Path):
    paths = []
    for index in range(3):
        path = tmp_path / f"episode-{index}.json"
        path.write_text(json.dumps(_report(_formal_episode_id(index))), encoding="utf-8")
        paths.append(path)
    report = aggregate(paths, 3)
    assert report["status"] == "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_BLOCKED"
    assert report["minimum_episode_count"] == 30
    assert report["requested_minimum_episode_count"] == 3
    assert not report["gates"]["minimum_disjoint_episode_count"]


def test_aggregate_blocks_duplicate_or_failed_episode(tmp_path: Path):
    paths = []
    for index in range(30):
        path = tmp_path / f"episode-{index}.json"
        status = "BLOCKED_ACCURACY_OR_RUNTIME" if index == 29 else "PASSED"
        episode_id = "val-map-000-mission-000" if index < 2 else _formal_episode_id(index)
        path.write_text(json.dumps(_report(episode_id, status)), encoding="utf-8")
        paths.append(path)
    report = aggregate(paths, 30)
    assert report["status"] == "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_BLOCKED"
    assert report["duplicate_episode_ids"] == ["val-map-000-mission-000"]
    assert not report["gates"]["all_episode_metric_gates_passed"]


def test_aggregate_blocks_formal_matrix_that_collapses_to_one_validation_map(tmp_path: Path):
    paths = []
    for index in range(30):
        path = tmp_path / f"episode-{index}.json"
        path.write_text(
            json.dumps(_report(f"val-map-000-mission-{index:03d}")), encoding="utf-8"
        )
        paths.append(path)
    report = aggregate(paths, 30)
    assert report["status"] == "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_BLOCKED"
    assert not report["gates"]["all_required_validation_maps_covered"]


def test_bound_matrix_report_preserves_existing_binding_and_writes_canonical_sidecar(
    tmp_path: Path, monkeypatch
):
    binding = {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        },
    }
    monkeypatch.setattr(perception, "load_binding", lambda _: binding)
    output = tmp_path / "formal_random_scene_perception_acceptance.json"
    report = {"status": "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_PASSED"}

    perception.write_bound_report(output, report, tmp_path / "existing-binding.json")

    sidecar = output.with_name(output.name + ".runtime_binding.json")
    assert json.loads(sidecar.read_text(encoding="utf-8")) == binding
    persisted = json.loads(output.read_text(encoding="utf-8"))
    assert persisted["runtime_gate_binding"] == binding
    assert persisted["acceptance_session_binding"] == binding["acceptance_session_binding"]
    assert persisted["runtime_closure_binding"] == binding["runtime_closure_binding"]


def test_bound_matrix_report_fails_closed_when_existing_binding_is_invalid(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setattr(
        perception,
        "load_binding",
        lambda _: (_ for _ in ()).throw(perception.RuntimeGateError("invalid binding")),
    )
    output = tmp_path / "formal_random_scene_perception_acceptance.json"
    with pytest.raises(perception.RuntimeGateError, match="invalid binding"):
        perception.write_bound_report(output, {"status": "PASSED"}, tmp_path / "missing.json")
    assert not output.exists()
    assert not output.with_name(output.name + ".runtime_binding.json").exists()
