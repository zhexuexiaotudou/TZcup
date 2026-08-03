#!/usr/bin/env python3
"""Audit and reconstruct one completed coverage mission from retained MCAP."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REQUIRED_TOPICS = {
    "/tf",
    "/tf_static",
    "/cmd_vel",
    "/brush_enabled",
    "/localization/fused_pose",
    "/ground_truth/odom",
    "/coverage/state",
    "/coverage/component_state",
    "/coverage/full_plan",
    "/coverage/planned_swaths",
    "/coverage/planned_connectors",
    "/coverage/current_component_path",
    "/coverage/actual_cleaning_trajectory",
    "/coverage/actual_transit_trajectory",
}


def summarize_replay(
    *,
    topic_counts: dict[str, int],
    states: list[str],
    component_payloads: list[dict],
    brush_values: list[bool],
    first_timestamp_ns: int | None,
    last_timestamp_ns: int | None,
    play_exit_code: int,
) -> dict:
    available = {topic for topic, count in topic_counts.items() if count > 0}
    missing = sorted(REQUIRED_TOPICS - available)
    component_ids = [
        str(payload["component_id"])
        for payload in component_payloads
        if payload.get("component_id") is not None
    ]
    ordered_unique_ids = list(dict.fromkeys(component_ids))
    terminal_states = [state for state in states if state in {"COMPLETED", "STOPPED", "FAILED"}]
    duration_ns = (
        max(0, last_timestamp_ns - first_timestamp_ns)
        if first_timestamp_ns is not None and last_timestamp_ns is not None
        else 0
    )
    gates = {
        "bag_has_messages_and_duration": sum(topic_counts.values()) > 0 and duration_ns > 0,
        "all_required_topics_present": not missing,
        "semantic_plan_present": topic_counts.get("/coverage/full_plan", 0) >= 1,
        "component_timeline_reconstructable": bool(ordered_unique_ids),
        "completed_terminal_state_present": "COMPLETED" in terminal_states,
        "brush_on_and_off_observed": True in brush_values and False in brush_values,
        "ros2_bag_play_succeeded": play_exit_code == 0,
    }
    return {
        "schema": "tzcup.coverage_mcap_replay.v1",
        "bag_readable": True,
        "message_count": sum(topic_counts.values()),
        "duration_ns": duration_ns,
        "required_topic_count": len(REQUIRED_TOPICS),
        "missing_required_topics": missing,
        "topic_message_counts": dict(sorted(topic_counts.items())),
        "state_sequence": states,
        "terminal_states": terminal_states,
        "component_event_count": len(component_payloads),
        "ordered_component_ids": ordered_unique_ids,
        "brush_transitions_observed": len(brush_values),
        "ros2_bag_play_exit_code": play_exit_code,
        "gates": gates,
        "pass": all(gates.values()),
    }


def read_bag(path: Path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_name) for topic, type_name in topic_types.items()
    }
    counts: dict[str, int] = {}
    states: list[str] = []
    component_payloads: list[dict] = []
    brush_values: list[bool] = []
    first_timestamp = None
    last_timestamp = None
    last_brush = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        first_timestamp = timestamp if first_timestamp is None else first_timestamp
        last_timestamp = timestamp
        if topic not in {
            "/coverage/state", "/coverage/component_state", "/brush_enabled"
        }:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == "/coverage/state":
            value = str(message.data)
            if not states or states[-1] != value:
                states.append(value)
        elif topic == "/coverage/component_state":
            try:
                component_payloads.append(json.loads(message.data))
            except json.JSONDecodeError:
                component_payloads.append({"invalid_json": True})
        else:
            value = bool(message.data)
            if last_brush is None or last_brush != value:
                brush_values.append(value)
                last_brush = value
    return (
        counts, states, component_payloads, brush_values,
        first_timestamp, last_timestamp,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--play-exit-code", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    values = read_bag(args.bag)
    report = summarize_replay(
        topic_counts=values[0], states=values[1], component_payloads=values[2],
        brush_values=values[3], first_timestamp_ns=values[4],
        last_timestamp_ns=values[5], play_exit_code=args.play_exit_code,
    )
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()
