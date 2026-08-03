import json
from pathlib import Path

import yaml

from visual_demo_summary import assemble


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_visual_demo_summary_passes_with_coverage_dashboard_bag_and_video(tmp_path):
    _write_json(
        tmp_path / "coverage_report.json",
        {
            "success": True,
            "full_execution_success": True,
            "safety_success": True,
            "mission_id": "demo",
            "component_count": 17,
            "empirical_metrics": {"coverage_rate": 0.93},
            "collision_count": 0,
            "keepout_violation_sample_count": 0,
        },
    )
    _write_json(
        tmp_path / "dashboard_telemetry.json",
        {
            "status": "COMPLETED",
            "topics_seen": [
                "/brush_enabled",
                "/cmd_vel",
                "/coverage/component_state",
                "/coverage/current_path",
                "/coverage/evaluation_sample",
                "/coverage/state",
                "/emergency_stop",
                "/localization/fused_pose",
            ],
            "claim_boundary": {
                "ground_truth_usage": "evaluation_and_visualization_only"
            },
        },
    )
    bag = tmp_path / "visual_demo_bag"
    bag.mkdir()
    required = [
        "/cmd_vel",
        "/coverage/component_state",
        "/coverage/current_path",
        "/coverage/evaluation_sample",
        "/coverage/state",
        "/localization/fused_pose",
        "/scan",
        "/tf",
    ]
    metadata = {
        "rosbag2_bagfile_information": {
            "message_count": 100,
            "duration": {"nanoseconds": 1_000_000_000},
            "topics_with_message_count": [
                {"topic_metadata": {"name": topic}} for topic in required
            ],
        }
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(metadata), encoding="utf-8"
    )
    (tmp_path / "visual_demo.mp4").write_bytes(b"0" * 100_000)
    (tmp_path / "visual_demo_frame.png").write_bytes(b"png")
    _write_json(
        tmp_path / "gazebo_cleaning_telemetry.json",
        {"targets_cleaned": 10, "targets_total": 10},
    )

    report = assemble(
        tmp_path,
        coverage_exit_code=0,
        mcap_required=True,
        video_mode="on",
        camera_follow_requested=True,
        targets_required=True,
    )
    assert report["machine_gate_pass"] is True
    assert report["coverage"]["empirical_coverage_rate"] == 0.93
    assert report["mcap"]["message_count"] == 100
    assert report["claim_boundary"]["learned_perception_pass"] is False
    assert report["cleaning_targets"]["complete"] is True


def test_visual_demo_summary_fails_closed_when_targets_are_incomplete(tmp_path):
    _write_json(
        tmp_path / "coverage_report.json",
        {
            "success": True,
            "full_execution_success": True,
            "safety_success": True,
            "collision_count": 0,
            "keepout_violation_sample_count": 0,
        },
    )
    _write_json(
        tmp_path / "dashboard_telemetry.json",
        {
            "status": "COMPLETED",
            "topics_seen": [
                "/brush_enabled", "/cmd_vel", "/coverage/component_state",
                "/coverage/current_path", "/coverage/evaluation_sample",
                "/coverage/state", "/emergency_stop", "/localization/fused_pose",
            ],
            "claim_boundary": {
                "ground_truth_usage": "evaluation_and_visualization_only"
            },
        },
    )
    _write_json(
        tmp_path / "gazebo_cleaning_telemetry.json",
        {"targets_cleaned": 9, "targets_total": 10},
    )

    report = assemble(
        tmp_path,
        coverage_exit_code=0,
        mcap_required=False,
        video_mode="off",
        camera_follow_requested=True,
        targets_required=True,
    )

    assert report["machine_gate_pass"] is False
    assert report["checks"]["target_requirement_satisfied"] is False


def test_visual_demo_summary_fails_closed_when_mission_did_not_complete(tmp_path):
    _write_json(
        tmp_path / "coverage_report.json",
        {"success": False, "full_execution_success": False},
    )
    report = assemble(
        tmp_path,
        coverage_exit_code=2,
        mcap_required=False,
        video_mode="off",
        camera_follow_requested=False,
    )
    assert report["machine_gate_pass"] is False
    assert report["status"] == "FAIL"
    assert report["checks"]["coverage_full_execution_success"] is False


def test_visual_demo_summary_requires_camera_follow(tmp_path):
    _write_json(
        tmp_path / "coverage_report.json",
        {
            "success": True,
            "full_execution_success": True,
            "safety_success": True,
            "collision_count": 0,
            "keepout_violation_sample_count": 0,
        },
    )
    _write_json(
        tmp_path / "dashboard_telemetry.json",
        {
            "status": "COMPLETED",
            "topics_seen": [
                "/brush_enabled",
                "/cmd_vel",
                "/coverage/component_state",
                "/coverage/current_path",
                "/coverage/evaluation_sample",
                "/coverage/state",
                "/emergency_stop",
                "/localization/fused_pose",
            ],
            "claim_boundary": {
                "ground_truth_usage": "evaluation_and_visualization_only"
            },
        },
    )

    report = assemble(
        tmp_path,
        coverage_exit_code=0,
        mcap_required=False,
        video_mode="off",
        camera_follow_requested=False,
    )

    assert report["machine_gate_pass"] is False
    assert report["checks"]["camera_follow_requested"] is False


def test_visual_demo_summary_does_not_require_camera_follow_when_headless(tmp_path):
    _write_json(
        tmp_path / "coverage_report.json",
        {
            "success": True,
            "full_execution_success": True,
            "safety_success": True,
            "collision_count": 0,
            "keepout_violation_sample_count": 0,
        },
    )
    _write_json(
        tmp_path / "dashboard_telemetry.json",
        {
            "status": "COMPLETED",
            "topics_seen": [
                "/brush_enabled", "/cmd_vel", "/coverage/component_state",
                "/coverage/current_path", "/coverage/evaluation_sample",
                "/coverage/state", "/emergency_stop", "/localization/fused_pose",
            ],
            "claim_boundary": {
                "ground_truth_usage": "evaluation_and_visualization_only"
            },
        },
    )

    report = assemble(
        tmp_path,
        coverage_exit_code=0,
        mcap_required=False,
        video_mode="off",
        camera_follow_requested=False,
        camera_follow_required=False,
    )

    assert report["machine_gate_pass"] is True
    assert report["camera_follow"]["required"] is False
