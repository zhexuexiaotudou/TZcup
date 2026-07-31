#!/usr/bin/env python3
"""Assemble an auditable AUTO-17 visual-demo acceptance summary."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import yaml


def _load_json(path: Path) -> dict | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _video_evidence(path: Path) -> dict:
    exists = path.is_file()
    size = path.stat().st_size if exists else 0
    return {
        "path": path.name,
        "exists": exists,
        "bytes": size,
        "nonempty": size >= 100_000,
    }


def _mcap_evidence(directory: Path) -> dict:
    metadata_path = directory / "metadata.yaml"
    if not metadata_path.is_file():
        return {
            "path": directory.name,
            "metadata_present": False,
            "message_count": 0,
            "duration_ns": 0,
            "topics": [],
        }
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = metadata.get("rosbag2_bagfile_information", {})
    topics = [
        row.get("topic_metadata", {}).get("name")
        for row in info.get("topics_with_message_count", [])
    ]
    return {
        "path": directory.name,
        "metadata_present": True,
        "message_count": int(info.get("message_count", 0)),
        "duration_ns": int((info.get("duration") or {}).get("nanoseconds", 0)),
        "topics": sorted(topic for topic in topics if topic),
    }


def assemble(
    output_dir: Path,
    *,
    coverage_exit_code: int,
    mcap_required: bool,
    video_mode: str,
    camera_follow_requested: bool,
) -> dict:
    coverage = _load_json(output_dir / "coverage_report.json")
    dashboard = _load_json(output_dir / "dashboard_telemetry.json")
    mcap = _mcap_evidence(output_dir / "visual_demo_bag")
    video = _video_evidence(output_dir / "visual_demo.mp4")
    screenshot = output_dir / "visual_demo_frame.png"
    coverage_success = bool(
        coverage
        and coverage.get("success") is True
        and coverage.get("full_execution_success") is True
    )
    dashboard_completed = bool(
        dashboard and dashboard.get("status") == "COMPLETED"
    )
    dashboard_required_topics = {
        "/brush_enabled",
        "/cmd_vel",
        "/coverage/component_state",
        "/coverage/current_path",
        "/coverage/evaluation_sample",
        "/coverage/state",
        "/emergency_stop",
        "/localization/fused_pose",
    }
    dashboard_topics = set(dashboard.get("topics_seen", [])) if dashboard else set()
    required_topics = {
        "/cmd_vel",
        "/coverage/component_state",
        "/coverage/current_path",
        "/coverage/evaluation_sample",
        "/coverage/state",
        "/localization/fused_pose",
        "/scan",
        "/tf",
    }
    recorded_topics = set(mcap["topics"])
    checks = {
        "coverage_process_exit_zero": coverage_exit_code == 0,
        "coverage_full_execution_success": coverage_success,
        "coverage_safety_success": bool(
            coverage and coverage.get("safety_success") is True
        ),
        "coverage_zero_collision_and_keepout_violations": bool(
            coverage
            and coverage.get("collision_count") == 0
            and coverage.get("keepout_violation_sample_count") == 0
        ),
        "dashboard_completed_snapshot_present": dashboard_completed,
        "dashboard_required_topics_seen": dashboard_required_topics <= dashboard_topics,
        "dashboard_truth_boundary_explicit": bool(
            dashboard
            and dashboard.get("claim_boundary", {}).get("ground_truth_usage")
            == "evaluation_and_visualization_only"
        ),
        "mcap_requirement_satisfied": (
            not mcap_required
            or (
                mcap["metadata_present"]
                and mcap["message_count"] > 0
                and required_topics <= recorded_topics
            )
        ),
        "video_requirement_satisfied": (
            video_mode != "on" or video["nonempty"]
        ),
        "video_frame_present_when_recorded": (
            not video["exists"]
            or (screenshot.is_file() and screenshot.stat().st_size > 0)
        ),
        "camera_follow_requested": bool(camera_follow_requested),
    }
    gate_checks = checks
    return {
        "schema_version": 1,
        "stage": "AUTO-17",
        "title": "visual demonstration layer",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if all(gate_checks.values()) else "FAIL",
        "machine_gate_pass": all(gate_checks.values()),
        "checks": checks,
        "coverage": {
            "exit_code": coverage_exit_code,
            "report_present": coverage is not None,
            "success": coverage_success,
            "mission_id": coverage.get("mission_id") if coverage else None,
            "component_count": coverage.get("component_count") if coverage else None,
            "empirical_coverage_rate": (
                (coverage.get("empirical_metrics") or {}).get("coverage_rate")
                if coverage
                else None
            ),
            "collision_count": (
                coverage.get("collision_count") if coverage else None
            ),
            "keepout_violation_count": (
                coverage.get("keepout_violation_sample_count")
                if coverage
                else None
            ),
        },
        "dashboard": {
            "snapshot_present": dashboard is not None,
            "status": dashboard.get("status") if dashboard else None,
            "topics_seen": dashboard.get("topics_seen", []) if dashboard else [],
        },
        "mcap": mcap,
        "video": {
            **video,
            "mode": video_mode,
            "representative_frame": (
                screenshot.name if screenshot.is_file() else None
            ),
        },
        "camera_follow": {
            "requested": camera_follow_requested,
            "transport_topic": "/gui/track",
            "target": "sanitation_vehicle",
        },
        "claim_boundary": {
            "source_level": "LIVE_GAZEBO_NAVIGATION_COVERAGE_DEMO",
            "learned_perception_pass": False,
            "real_domain_pass": False,
            "j6_runtime_pass": False,
            "simulation_competition_matrix_pass": False,
            "note": (
                "AUTO-17 improves observability and repeatable demonstration; "
                "it does not change AUTO-05/06/07/08/13/14/15 status."
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--coverage-exit-code", type=int, required=True)
    parser.add_argument("--mcap-required", action="store_true")
    parser.add_argument(
        "--video-mode", choices=("on", "off", "auto"), default="auto"
    )
    parser.add_argument("--camera-follow-requested", action="store_true")
    args = parser.parse_args()
    report = assemble(
        args.output_dir,
        coverage_exit_code=args.coverage_exit_code,
        mcap_required=args.mcap_required,
        video_mode=args.video_mode,
        camera_follow_requested=args.camera_follow_requested,
    )
    output = args.output_dir / "acceptance_summary.json"
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["machine_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
