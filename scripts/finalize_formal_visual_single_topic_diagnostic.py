#!/usr/bin/env python3
"""Summarize the bounded stages of a single-camera transport diagnostic."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(_text(path))
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def summarize(root: Path, child_result: int) -> dict[str, object]:
    prepared = _json(root / "single_topic_world_report.json")
    gz_info = _text(root / "gz_topic_info.txt")
    ros_info = _text(root / "ros_topic_info.txt")
    gz_sample = _json(root / "gz_sample_metadata.json")
    ros_width = _text(root / "ros_width.txt").strip()
    transport_maps = _json(root / "transport_process_maps.json")
    stages = {
        "single_camera_world": (
            prepared.get("passed") is True
            and prepared.get("remaining_total_sensor_count") == 1
        ),
        "gazebo_and_bridge_transport_libraries_bound": (
            transport_maps.get("passed") is True
            and bool(transport_maps.get("cross_process_checks"))
            and all(transport_maps.get("cross_process_checks", {}).values())
        ),
        "gazebo_publisher_and_bridge_subscriber_discovered": (
            "gz.msgs.Image" in gz_info
            and "Publisher" in gz_info
            and "Subscriber" in gz_info
        ),
        "direct_gazebo_image_received": (
            gz_sample.get("passed") is True
            and gz_sample.get("width") == 1600
            and gz_sample.get("height") == 1000
            and gz_sample.get("expected_uncompressed_data_bytes_from_step")
            == 4_800_000
        ),
        "ros_bridge_and_diagnostic_subscriber_discovered": (
            "Publisher count: 1" in ros_info
            and "Subscription count: 1" in ros_info
        ),
        "ros_image_received": ros_width == "1600",
        "child_process_completed": child_result == 0,
    }
    passed = all(stages.values())
    first_failed = next((name for name, ok in stages.items() if not ok), None)
    return {
        "report_id": "tzcup_formal_visual_single_topic_diagnostic_v1",
        "status": (
            "FORMAL_VISUAL_SINGLE_TOPIC_DIAGNOSTIC_PASSED"
            if passed
            else "FORMAL_VISUAL_SINGLE_TOPIC_DIAGNOSTIC_FAILED"
        ),
        "passed": passed,
        "child_result": child_result,
        "first_failed_stage": first_failed,
        "stages": stages,
        "transport_process_maps": transport_maps,
        "direct_gazebo_sample": gz_sample,
        "ros_width": ros_width,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--child-result", type=int, required=True)
    args = parser.parse_args()
    report_path = args.output_root / "diagnostic_report.json"
    if report_path.exists() or report_path.is_symlink():
        raise RuntimeError(f"refusing stale diagnostic report: {report_path}")
    report = summarize(args.output_root, args.child_result)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(report["status"])
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
