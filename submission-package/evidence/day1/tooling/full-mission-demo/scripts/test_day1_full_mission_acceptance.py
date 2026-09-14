from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import jsonschema
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from day1_full_mission_acceptance import (  # noqa: E402
    RECEIPT_NAME,
    assemble,
)
import day1_full_mission_acceptance as acceptance  # noqa: E402


SCHEMA_PATH = (
    ROOT / "schemas/day1_full_mission_acceptance_receipt.schema.json"
)
SCHEMA_EXAMPLE_PATH = (
    ROOT
    / "schemas/examples/day1_full_mission_acceptance_receipt.fixture.json"
)


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _make_fixture(tmp_path: Path) -> dict:
    run_dir = tmp_path / "run"
    video_dir = tmp_path / "video"
    output_dir = tmp_path / "out"
    support = run_dir / "support"
    support.mkdir(parents=True, exist_ok=True)
    video_dir.mkdir(parents=True, exist_ok=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    mission_id = "fixture-full-mission"
    run_id = "fixture-run"

    stages = [
        {
            "id": "startup",
            "status": "COMPLETED",
            "start_sim_sec": 0.0,
            "end_sim_sec": 10.0,
        },
        {
            "id": "coverage",
            "status": "COMPLETED",
            "start_sim_sec": 10.0,
            "end_sim_sec": 390.0,
        },
        {
            "id": "return_home",
            "status": "COMPLETED",
            "start_sim_sec": 390.0,
            "end_sim_sec": 450.0,
        },
        {
            "id": "shutdown",
            "status": "COMPLETED",
            "start_sim_sec": 450.0,
            "end_sim_sec": 480.0,
        },
    ]
    _write_json(
        run_dir / "mission_timeline.json",
        {
            "schema_version": 1,
            "mission_id": mission_id,
            "run_id": run_id,
            "duration_basis": "full_mission_sim_seconds",
            "sim_start_sec": 0.0,
            "sim_end_sec": 480.0,
            "sim_duration_sec": 480.0,
            "wall_duration_sec": 60.0,
            "rtf": 8.0,
            "terminal_state": "COMPLETED",
            "stages": stages,
        },
    )
    _write_json(
        run_dir / "coverage_report.json",
        {
            "schema_version": 2,
            "mission_id": mission_id,
            "success": True,
            "full_execution_success": True,
            "coverage_quality_success": True,
            "safety_success": True,
            "localization_success": True,
            "ground_truth_used_for_control": False,
            "brush_disabled_on_exit": True,
            "collision_count": 0,
            "keepout_violation_sample_count": 0,
            "planned_metrics": {"coverage_rate": 1.0},
            "empirical_metrics": {
                "coverage_rate": 0.99,
                "covered_area_m2": 500.0,
                "cleanable_area_m2": 505.0,
                "actual_path_length_m": 800.0,
                "actual_duration_sec": 380.0,
                "brush_enabled_distance_m": 780.0,
                "brush_state_transitions": 4,
            },
        },
    )
    (run_dir / "coverage_trajectory.csv").write_text(
        "sim_time_sec,x_m,y_m,brush_enabled\n"
        "0.0,0.0,0.0,false\n"
        "1.0,0.1,0.0,true\n"
        "2.0,0.2,0.0,true\n",
        encoding="utf-8",
    )
    (run_dir / "coverage_config.yaml").write_text(
        yaml.safe_dump(
            {
                "outer_polygon": [[0, 0], [10, 0], [10, 10], [0, 10]],
                "empirical_coverage_threshold": 0.98,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    _write_json(
        run_dir / "bounded_coverage_result.json",
        {
            "schema_version": 1,
            "artifact_kind": "day1_bounded_saved_map_coverage_result",
            "terminal_state": "COMPLETED",
            "valid_bounded_mission": True,
            "mission_id": mission_id,
        },
    )
    _write_json(
        run_dir / "cleaning_bridge.json",
        {
            "permit_observed": True,
            "lift_requested": True,
            "all_three_ready_sample_count": 10,
            "maximum_roller_velocity_rad_s": 12.0,
            "brush_disabled_on_exit": True,
            "dirt_system_disabled_on_exit": True,
            "cleaned_cell_delta": 100,
            "cleaned_area_delta_m2": 8.0,
            "initial": {
                "cell_count": 1800,
                "cleaned_cell_count": 0,
            },
            "terminal": {
                "cell_count": 1800,
                "cleaned_cell_count": 100,
                "cleaned_fraction": 0.055,
            },
        },
    )
    safety_evidence = support / "emergency_stop.json"
    collision_evidence = support / "collision_monitor.json"
    safety_evidence.write_text('{"fixture":true}\n', encoding="utf-8")
    collision_evidence.write_text('{"fixture":true}\n', encoding="utf-8")
    _write_json(
        run_dir / "safety_receipt.json",
        {
            "schema_version": 1,
            "mission_id": mission_id,
            "collision_count": 0,
            "keepout_violation_count": 0,
            "emergency_stop": {
                "exercised": True,
                "latency_sec": 0.49,
                "final_linear_mps": 0.0,
                "final_angular_radps": 0.0,
            },
            "evidence": [
                {
                    "role": "emergency_stop",
                    "path": "support/emergency_stop.json",
                    "sha256": _sha256(safety_evidence),
                },
                {
                    "role": "collision_monitor",
                    "path": "support/collision_monitor.json",
                    "sha256": _sha256(collision_evidence),
                },
            ],
        },
    )
    _write_json(
        run_dir / "return_home_receipt.json",
        {
            "schema_version": 1,
            "mission_id": mission_id,
            "requested": True,
            "started": True,
            "completed": True,
            "started_sim_sec": 390.0,
            "completed_sim_sec": 450.0,
            "position_error_m": 0.04,
            "yaw_error_rad": 0.01,
            "brush_disabled_during_return": True,
        },
    )
    _write_json(
        run_dir / "final_state.json",
        {
            "mission_id": mission_id,
            "terminal_state": "COMPLETED",
            "mode": "IDLE",
            "motion_zero": True,
            "brush_enabled": False,
            "dirt_system_enabled": False,
            "emergency_stop_active": False,
            "timestamp_sim_sec": 480.0,
        },
    )

    bag_dir = run_dir / "bag"
    bag_dir.mkdir()
    mcap = bag_dir / "bag_0.mcap"
    magic = b"\x89MCAP0\r\n"
    mcap.write_bytes(magic + b"fixture-payload" * 4 + magic)
    metadata = {
        "rosbag2_bagfile_information": {
            "message_count": len(_required_topics()) * 2,
            "duration": {"nanoseconds": 480_000_000_000},
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": 2,
                }
                for topic in sorted(_required_topics())
            ],
        }
    }
    (bag_dir / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding="utf-8",
    )

    completeness_files = {}
    for role in (
        "source_index",
        "build_receipt",
        "test_receipt",
        "component_register",
        "power_budget",
        "board_runtime",
    ):
        path = support / f"{role}.json"
        path.write_text(
            json.dumps({"fixture_only": True, "role": role}) + "\n",
            encoding="utf-8",
        )
        completeness_files[role] = path
    _write_json(
        run_dir / "hardware_software_manifest.json",
        {
            "schema_version": 1,
            "software": {
                "status": "COMPLETE",
                "revision": "fixture-revision",
                "evidence": [
                    {
                        "role": role,
                        "path": f"support/{role}.json",
                        "sha256": _sha256(completeness_files[role]),
                    }
                    for role in (
                        "source_index",
                        "build_receipt",
                        "test_receipt",
                    )
                ],
            },
            "hardware": {
                "status": "COMPLETE",
                "platform": "fixture-platform",
                "evidence": [
                    {
                        "role": role,
                        "path": f"support/{role}.json",
                        "sha256": _sha256(completeness_files[role]),
                    }
                    for role in (
                        "component_register",
                        "power_budget",
                        "board_runtime",
                    )
                ],
            },
        },
    )

    video_path = video_dir / "demo.mp4"
    video_path.write_bytes(b"fixture-video" * 100)
    video_probe = {
        "format": {
            "duration": "480.0",
            "size": str(video_path.stat().st_size),
            "format_name": "mov,mp4,m4a,3gp,3g2,mj2",
        },
        "streams": [
            {
                "codec_type": "video",
                "codec_name": "h264",
                "width": 1920,
                "height": 1080,
            }
        ],
    }
    _write_json(
        video_dir / "video_manifest.json",
        {
            "schema_version": 1,
            "run_id": run_id,
            "mission_id": mission_id,
            "file": video_path.name,
            "file_sha256": _sha256(video_path),
            "duration_sec": 480.0,
            "width": 1920,
            "height": 1080,
            "codec_name": "h264",
            "contains_operation_footage": True,
            "stitched": False,
        },
    )
    return {
        "run_dir": run_dir,
        "video_dir": video_dir,
        "output_dir": output_dir,
        "video_probe": video_probe,
    }


def _required_topics() -> set[str]:
    return {
        "/clock",
        "/ground_truth/odom",
        "/odom",
        "/amcl_pose",
        "/localization/fused_pose",
        "/scan",
        "/cmd_vel_nav",
        "/cmd_vel_gate",
        "/base_controller/cmd_vel",
        "/joint_states",
        "/safety/status",
        "/brush_enabled",
        "/coverage/state",
        "/coverage/component_state",
        "/coverage/current_path",
        "/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json",
    }


def _assemble(fixture: dict) -> dict:
    return assemble(
        run_dir=fixture["run_dir"],
        video_dir=fixture["video_dir"],
        output_dir=fixture["output_dir"],
        video_probe=fixture["video_probe"],
        fixture_only=True,
    )


def _gate(receipt: dict, gate_id: str) -> dict:
    return next(row for row in receipt["gates"] if row["id"] == gate_id)


def test_complete_fixture_passes_and_matches_schema(tmp_path: Path) -> None:
    fixture = _make_fixture(tmp_path)
    receipt = _assemble(fixture)

    assert receipt["status"] == "PASS"
    assert receipt["machine_gate_pass"] is True
    assert receipt["fixture_only"] is True
    assert receipt["efficiency"]["effective_cleaning_efficiency_m2_h"] == 3750.0
    jsonschema.validate(
        receipt,
        json.loads(SCHEMA_PATH.read_text(encoding="utf-8")),
    )
    assert (fixture["output_dir"] / "demo_storyboard.md").is_file()
    assert (fixture["output_dir"] / "demo_recording_checklist.md").is_file()
    assert (fixture["output_dir"] / "demo_shot_index.csv").is_file()


def test_checked_in_schema_example_is_explicitly_fixture_only() -> None:
    payload = json.loads(SCHEMA_EXAMPLE_PATH.read_text(encoding="utf-8"))

    assert payload["fixture_only"] is True
    assert "fixture" in payload["run"]["mission_id"]
    jsonschema.validate(
        payload,
        json.loads(SCHEMA_PATH.read_text(encoding="utf-8")),
    )


def test_missing_return_home_receipt_fails_closed(tmp_path: Path) -> None:
    fixture = _make_fixture(tmp_path)
    (fixture["run_dir"] / "return_home_receipt.json").unlink()

    receipt = _assemble(fixture)

    assert receipt["status"] == "FAIL"
    assert _gate(receipt, "return_home_receipt_present")["passed"] is False


def test_efficiency_ignores_wall_clock_and_coverage_probe_short_window(
    tmp_path: Path,
) -> None:
    fixture = _make_fixture(tmp_path)
    timeline_path = fixture["run_dir"] / "mission_timeline.json"
    timeline = json.loads(timeline_path.read_text(encoding="utf-8"))
    timeline["wall_duration_sec"] = 120.0
    timeline["rtf"] = 4.0
    _write_json(timeline_path, timeline)
    report_path = fixture["run_dir"] / "coverage_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["empirical_metrics"]["actual_duration_sec"] = 10.0
    _write_json(report_path, report)

    receipt = _assemble(fixture)

    assert receipt["status"] == "PASS"
    assert receipt["efficiency"]["full_mission_sim_duration_sec"] == 480.0
    assert receipt["efficiency"]["wall_duration_sec"] == 120.0
    assert receipt["efficiency"]["effective_cleaning_efficiency_m2_h"] == 3750.0
    assert receipt["efficiency"]["ten_second_steady_window_used"] is False


def test_short_steady_window_cannot_become_full_mission_duration(
    tmp_path: Path,
) -> None:
    fixture = _make_fixture(tmp_path)
    timeline_path = fixture["run_dir"] / "mission_timeline.json"
    timeline = json.loads(timeline_path.read_text(encoding="utf-8"))
    timeline["duration_basis"] = "steady_window_10s"
    timeline["sim_start_sec"] = 0.0
    timeline["sim_end_sec"] = 10.0
    timeline["sim_duration_sec"] = 10.0
    timeline["wall_duration_sec"] = 10.0
    timeline["rtf"] = 1.0
    timeline["stages"] = [
        {
            "id": "startup",
            "status": "COMPLETED",
            "start_sim_sec": 0.0,
            "end_sim_sec": 2.0,
        },
        {
            "id": "coverage",
            "status": "COMPLETED",
            "start_sim_sec": 2.0,
            "end_sim_sec": 6.0,
        },
        {
            "id": "return_home",
            "status": "COMPLETED",
            "start_sim_sec": 6.0,
            "end_sim_sec": 8.0,
        },
        {
            "id": "shutdown",
            "status": "COMPLETED",
            "start_sim_sec": 8.0,
            "end_sim_sec": 10.0,
        },
    ]
    _write_json(timeline_path, timeline)

    receipt = _assemble(fixture)

    assert receipt["status"] == "FAIL"
    assert receipt["efficiency"]["effective_cleaning_efficiency_m2_h"] is None
    assert (
        _gate(receipt, "timeline_full_mission_duration_basis")["passed"]
        is False
    )


def test_video_outside_five_to_ten_minutes_fails(tmp_path: Path) -> None:
    fixture = _make_fixture(tmp_path)
    fixture["video_probe"]["format"]["duration"] = "240.0"
    manifest_path = fixture["video_dir"] / "video_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["duration_sec"] = 240.0
    _write_json(manifest_path, manifest)

    receipt = _assemble(fixture)

    assert receipt["status"] == "FAIL"
    assert _gate(receipt, "video_duration_5_to_10_min")["passed"] is False


def test_inconsistent_dirt_delta_fails(tmp_path: Path) -> None:
    fixture = _make_fixture(tmp_path)
    cleaning_path = fixture["run_dir"] / "cleaning_bridge.json"
    cleaning = json.loads(cleaning_path.read_text(encoding="utf-8"))
    cleaning["terminal"]["cleaned_cell_count"] = 99
    _write_json(cleaning_path, cleaning)

    receipt = _assemble(fixture)

    assert receipt["status"] == "FAIL"
    assert (
        _gate(receipt, "dirt_delta_accounting_consistent")["passed"]
        is False
    )


def test_mcap_missing_required_topic_fails(tmp_path: Path) -> None:
    fixture = _make_fixture(tmp_path)
    metadata_path = fixture["run_dir"] / "bag" / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    rows = metadata["rosbag2_bagfile_information"][
        "topics_with_message_count"
    ]
    metadata["rosbag2_bagfile_information"][
        "topics_with_message_count"
    ] = [
        row
        for row in rows
        if row["topic_metadata"]["name"] != "/brush_enabled"
    ]
    metadata_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding="utf-8",
    )

    receipt = _assemble(fixture)

    assert receipt["status"] == "FAIL"
    assert "/brush_enabled" in receipt["mcap"]["missing_topics"]
    assert _gate(receipt, "mcap_required_topics")["passed"] is False


def test_hardware_completeness_blocker_cannot_pass(tmp_path: Path) -> None:
    fixture = _make_fixture(tmp_path)
    manifest_path = fixture["run_dir"] / "hardware_software_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["hardware"]["status"] = "BLOCKED_MECHANICAL_ELECTRICAL_INTEGRATION"
    _write_json(manifest_path, manifest)

    receipt = _assemble(fixture)

    assert receipt["status"] == "FAIL"
    assert (
        _gate(receipt, "hardware_completeness_declared")["passed"]
        is False
    )


def test_cli_writes_pass_receipt(
    tmp_path: Path, monkeypatch
) -> None:
    fixture = _make_fixture(tmp_path)
    monkeypatch.setattr(
        acceptance,
        "_probe_video",
        lambda _path, _ffprobe_bin: fixture["video_probe"],
    )

    exit_code = acceptance.main(
        [
            "--run-dir",
            str(fixture["run_dir"]),
            "--video-dir",
            str(fixture["video_dir"]),
            "--output-dir",
            str(fixture["output_dir"]),
        ]
    )

    receipt_path = fixture["output_dir"] / RECEIPT_NAME
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    assert exit_code == 0
    assert receipt["status"] == "PASS"
