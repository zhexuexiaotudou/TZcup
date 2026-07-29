#!/usr/bin/env python3
"""Audit AUTO-02 MCAP topic coverage and reproduce formal mission metrics."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


REQUIRED_TOPICS = {
    "/brush_enabled",
    "/cmd_vel",
    "/cmd_vel_gate",
    "/collision_monitor_state",
    "/coverage/component_state",
    "/coverage/current_path",
    "/coverage/diagnostics",
    "/coverage/evaluation_sample",
    "/coverage/state",
    "/ground_truth/odom",
    "/localization/fused_pose",
    "/scan",
    "/speed_limit",
    "/tf",
    "/tf_static",
}


def relative_delta(actual: float, expected: float) -> float:
    denominator = max(abs(expected), 1.0e-12)
    return abs(actual - expected) / denominator


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--coverage-report", type=Path, required=True)
    parser.add_argument("--replay-state", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-emergency-stop", action="store_true")
    args = parser.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sanitation_coverage.metrics import (
        empirical_swept_metrics,
        summarize_distances,
        synchronized_xy_errors,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    fused = []
    truth = []
    brush_points = []
    evaluation_stamps = []
    brush_enabled = False
    coverage_states = []
    while reader.has_next():
        topic, data, _received_stamp = reader.read_next()
        if topic not in {
            "/brush_enabled",
            "/coverage/evaluation_sample",
            "/ground_truth/odom",
            "/localization/fused_pose",
            "/coverage/state",
        }:
            continue
        message = deserialize_message(data, get_message(topic_types[topic]))
        if topic == "/brush_enabled":
            brush_enabled = bool(message.data)
            continue
        if topic == "/coverage/evaluation_sample":
            sample = json.loads(message.data)
            evaluation_stamps.append(float(sample["stamp_sec"]))
            if sample["brush_enabled"]:
                brush_points.append(
                    (
                        float(sample["stamp_sec"]),
                        float(sample["base_x_m"])
                        + 0.55 * math.cos(float(sample["yaw_rad"])),
                        float(sample["base_y_m"])
                        + 0.55 * math.sin(float(sample["yaw_rad"])),
                    )
                )
            continue
        if topic == "/coverage/state":
            coverage_states.append(message.data)
            continue
        stamp = message.header.stamp
        time_sec = stamp.sec + stamp.nanosec * 1.0e-9
        pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
        sample = (time_sec, pose.position.x, pose.position.y)
        if (
            topic == "/ground_truth/odom"
            and brush_enabled
            and "/coverage/evaluation_sample" not in topic_types
        ):
            orientation = pose.orientation
            siny_cosp = 2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            )
            cosy_cosp = 1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            )
            yaw = math.atan2(siny_cosp, cosy_cosp)
            brush_points.append(
                (
                    time_sec,
                    pose.position.x + 0.55 * math.cos(yaw),
                    pose.position.y + 0.55 * math.sin(yaw),
                )
            )
        (truth if topic == "/ground_truth/odom" else fused).append(sample)

    if evaluation_stamps:
        evaluation_start = min(evaluation_stamps)
        evaluation_end = max(evaluation_stamps)
        fused = [
            sample
            for sample in fused
            if evaluation_start <= sample[0] <= evaluation_end
        ]
        truth = [
            sample
            for sample in truth
            if evaluation_start <= sample[0] <= evaluation_end
        ]
    else:
        evaluation_start = None
        evaluation_end = None
    errors, sync_errors, dropped = synchronized_xy_errors(fused, truth)
    reproduced = summarize_distances(errors)
    source = json.loads(args.coverage_report.read_text(encoding="utf-8"))
    expected_rmse = float(
        source["localization_regression_during_coverage"]["rmse_m"]
    )
    reproduced_rmse = reproduced["rmse_m"]
    delta = (
        relative_delta(float(reproduced_rmse), expected_rmse)
        if reproduced_rmse is not None
        else None
    )
    mission_geometry = source["mission_geometry"]
    reproduced_empirical = empirical_swept_metrics(
        mission_geometry["cleanable_outer_polygon"],
        brush_points,
        float(source["operation_width_m"]),
        resolution=float(source["empirical_metrics"]["resolution_m"]),
        exclusion_polygons=mission_geometry[
            "cleanable_exclusion_polygons"
        ],
    )
    expected_coverage = float(source["empirical_metrics"]["coverage_rate"])
    reproduced_coverage = float(reproduced_empirical["coverage_rate"])
    coverage_delta = relative_delta(reproduced_coverage, expected_coverage)
    required_topics = set(REQUIRED_TOPICS)
    if args.require_emergency_stop:
        required_topics.add("/emergency_stop")
    missing = sorted(required_topics - set(topic_types))
    replay_text = (
        args.replay_state.read_text(encoding="utf-8", errors="replace")
        if args.replay_state.is_file()
        else ""
    )
    checks = {
        "mcap_metadata_readable": (args.bag / "metadata.yaml").is_file(),
        "required_topics_present_100_percent": not missing,
        "coverage_state_recorded": bool(coverage_states),
        "coverage_terminal_state_recorded": any(
            state in {"COMPLETED", "FAILED", "RECOVERY"}
            for state in coverage_states
        ),
        "coverage_state_replay_observed": bool(replay_text.strip()),
        "localization_samples_reproduced": bool(errors),
        "localization_rmse_metric_delta_at_most_1_percent": (
            delta is not None and delta <= 0.01
        ),
        "empirical_coverage_metric_delta_at_most_1_percent": (
            brush_points and coverage_delta <= 0.01
        ),
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-02",
        "bag": str(args.bag),
        "required_topic_count": len(required_topics),
        "present_required_topic_count": len(required_topics) - len(missing),
        "missing_required_topics": missing,
        "coverage_state_sample_count": len(coverage_states),
        "coverage_terminal_state_observed": any(
            state in {"COMPLETED", "FAILED", "RECOVERY"}
            for state in coverage_states
        ),
        "localization": {
            "source_rmse_m": expected_rmse,
            "reproduced_rmse_m": reproduced_rmse,
            "relative_delta": delta,
            "estimate_sample_count": len(fused),
            "truth_sample_count": len(truth),
            "matched_sample_count": len(errors),
            "dropped_estimate_count": dropped,
            "sync_error_sample_count": len(sync_errors),
            "evaluation_window_start_sec": evaluation_start,
            "evaluation_window_end_sec": evaluation_end,
        },
        "empirical_coverage": {
            "source_coverage_rate": expected_coverage,
            "reproduced_coverage_rate": reproduced_coverage,
            "relative_delta": coverage_delta,
            "brush_on_sample_count": len(brush_points),
        },
        "checks": checks,
        "replay_gate_pass": all(checks.values()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["replay_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
