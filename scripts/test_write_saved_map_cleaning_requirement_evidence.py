import importlib.util
import json
import sys
from pathlib import Path


SCRIPT = Path(__file__).with_name("write_saved_map_cleaning_requirement_evidence.py")
SPEC = importlib.util.spec_from_file_location("requirement_evidence", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def test_requirement_evidence_binds_one_identity_and_records_missing(tmp_path, monkeypatch):
    episode = tmp_path / "episode.json"
    binding = tmp_path / "binding.json"
    restart = tmp_path / "restart.json"
    runtime = tmp_path / "runtime.json"
    coverage = tmp_path / "coverage.json"
    route = tmp_path / "route.json"
    output = tmp_path / "requirement_evidence.json"
    manifest = tmp_path / "runtime_artifact_manifest.json"
    _write(episode, {"episode_id": "ep-1"})
    _write(binding, {"acceptance_session_binding": {"session_manifest_sha256": "a" * 64}})
    _write(restart, {"cleaning_start_wall_time": "2026-09-11T00:00:00+00:00"})
    _write(runtime, {"amcl_pose_sample_count": 2, "nav2_action_ready": True, "coverage_first_brush_enabled_monotonic_s": 1.0, "brush_enabled_distance_m": 3.0, "trajectory_total_distance_m": 4.0, "coverage_action_terminal_passed": True, "estimated_coverage_fraction": 0.95})
    _write(coverage, {"success": True, "return_home": {"success": True, "session_id": "a" * 64, "runtime_id": "run-1", "episode_id": "ep-1", "goal_frame_id": "map", "final_cmd_vel_zero": True, "brush_control_released": True, "coverage_control_released": True}})
    _write(route, {"passed": True, "realized_lane_spacing_m": 1.0, "recommended_max_lane_spacing_m": 1.056, "disconnected_turn_rows": []})
    monkeypatch.setattr(sys, "argv", ["evidence", "--episode-manifest", str(episode), "--runtime-binding", str(binding), "--runtime-id", "run-1", "--restart-record", str(restart), "--cleaning-runtime", str(runtime), "--coverage-report", str(coverage), "--route-sanity", str(route), "--output", str(output), "--artifact-manifest", str(manifest)])
    assert MODULE.main() == 0
    value = json.loads(output.read_text())
    assert {row["status"] for row in value["requirements"]} == {"observed"}
    assert {row["session_id"] for row in value["requirements"]} == {"a" * 64}
    assert {row["runtime_id"] for row in value["requirements"]} == {"run-1"}
    assert {row["episode_id"] for row in value["requirements"]} == {"ep-1"}
    assert json.loads(manifest.read_text())["artifacts_sha256"][str(output)]
