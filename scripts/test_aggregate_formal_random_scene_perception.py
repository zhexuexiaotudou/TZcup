import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import aggregate_formal_random_scene_perception as perception  # noqa: E402

aggregate = perception.aggregate


def _report(episode_id: str, status: str = "PASSED") -> dict:
    report = perception.finalize_acceptance(
        episode_id=episode_id,
        detection={"true_positive_count": 90, "false_positive_count": 0,
                   "visible_unique_truth_count": 100, "matched_unique_truth_count": 90,
                   "evaluated_frame_count": 100},
        segmentation={"iou": 0.7, "recall": 0.9, "truth_cell_count": 100},
        projection={"sample_count": 10, "rmse_m": 0.1, "p95_m": 0.2,
                    "false_product_track_count": 0},
        freshness={"real_camera_message_count": 10, "rgb_topic_count": 4,
                   "depth_topic_count": 2, "camera_info_topic_count": 4,
                   "depth_rgb_skew_max_s": 0.1, "tf_success_ratio": 1.0,
                   "tf_age_max_s": 0.1, "diagnostic_ground_truth_input_used": False,
                   "product_detection_message_count": 10, "product_mask_message_count": 10,
                   "product_target_message_count": 10},
    )
    if status != "PASSED":
        report["sensor_runtime"]["product_mask_message_count"] = 0
        report = perception.finalize_acceptance(
            episode_id=episode_id, detection=report["litter_cube_detection"],
            segmentation=report["ground_dirt_segmentation"], projection=report["map_projection"],
            freshness=report["sensor_runtime"],
        )
    return report


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
    assert report["episodes"][0]["litter_cube_detection"]["f1"] == pytest.approx(2 * 0.9 / 1.9)
    assert report["episodes"][0]["artifact_evidence"][
        "product_source_manifest_entries"
    ] == ["a" * 64 + "  pc_open_vocab_adapter.py"]


def test_aggregate_blocks_matrix_when_one_episode_uses_truth_in_product_perception(tmp_path: Path):
    paths = []
    for index in range(30):
        report = _report(_formal_episode_id(index))
        if index == 0:
            report["truth_isolation"]["truth_used_by_product_perception"] = True
        path = tmp_path / f"episode-{index}.json"
        path.write_text(json.dumps(report), encoding="utf-8")
        paths.append(path)
    result = aggregate(paths, 30)
    assert result["status"] == "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_BLOCKED"
    assert not result["gates"]["truth_isolation_passed"]


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


@pytest.mark.parametrize("mutation", [
    lambda r: r.pop("metric_checks"),
    lambda r: r.pop("litter_cube_detection"),
    lambda r: r["thresholds"].update(cube_precision_min=0.0),
    lambda r: r["ground_dirt_segmentation"].update(iou=0.0),
    lambda r: r["map_projection"].update(rmse_m=float("nan")),
    lambda r: r["sensor_runtime"].update(product_mask_message_count=0),
    lambda r: r["litter_cube_detection"].update(precision=0.01),
])
def test_aggregate_rejects_stale_pass_or_incomplete_evidence(tmp_path, mutation):
    paths = []
    for index in range(30):
        row = _report(_formal_episode_id(index))
        if index == 0:
            mutation(row)
        path = tmp_path / f"{index}.json"
        path.write_text(json.dumps(row), encoding="utf-8")
        paths.append(path)
    report = aggregate(paths, 30)
    assert report["status"].endswith("BLOCKED")
    assert not report["gates"]["all_episode_reports_well_formed"]
    assert report["input_errors"]


@pytest.mark.parametrize("root", [None, [], 1, "PASSED"])
def test_non_object_episode_is_reported_as_input_error(tmp_path, root):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(root), encoding="utf-8")
    report = aggregate([path], 30)
    assert report["input_errors"]


def test_legitimate_negative_map_coordinates_remain_valid():
    report = _report("val-map-000-mission-000")
    report["sensor_runtime"].update({
        "localized_map_pose": {"x": -1.0, "y": -2.0, "yaw": -0.5},
        "expected_localization_map_start_pose": [-1.0, -2.0, -0.5],
        "source_world_start_pose_used_for_staging": {"x": -10.0, "y": -20.0},
        "map_ground_z_m": -0.1651,
    })
    perception._validate_episode(report)


def test_live_evaluator_config_matches_frozen_aggregate_thresholds():
    import yaml
    config = yaml.safe_load((ROOT / "starter_ws/src/sanitation_perception/config/formal_random_scene_acceptance.yaml").read_text(encoding="utf-8"))
    effective = {**perception.DEFAULT_THRESHOLDS, **config["metrics"], **config["runtime"]}
    assert all(effective[name] == value for name, value in perception.DEFAULT_THRESHOLDS.items())
    assert config["minimum_episode_count"] == perception.DEFAULT_THRESHOLDS["minimum_episode_count"]
