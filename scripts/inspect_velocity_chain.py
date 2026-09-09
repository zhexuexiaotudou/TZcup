#!/usr/bin/env python3
"""Summarize commanded and observed linear speeds in a completed ROS MCAP."""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Iterable
from pathlib import Path
from typing import Any


VELOCITY_TOPICS = (
    "/cmd_vel",
    "/cmd_vel_gate",
    "/wheel/odom_raw",
    "/ground_truth/odom",
)
BRUSH_TOPIC = "/brush_enabled"


def _percentile(sorted_values: list[float], fraction: float) -> float | None:
    """Return a linearly interpolated percentile for already sorted values."""
    if not sorted_values:
        return None
    index = (len(sorted_values) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return sorted_values[lower]
    weight = index - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def summarize(samples: Iterable[float]) -> dict[str, float | int | None]:
    """Summarize finite absolute linear-velocity samples without ROS imports."""
    values = sorted(
        abs(float(sample)) for sample in samples if math.isfinite(float(sample))
    )
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "max": values[-1],
    }


def _linear_x(message: Any) -> float:
    """Extract ``linear.x`` from Twist or Odometry-shaped ROS messages."""
    if hasattr(message, "linear"):
        return float(message.linear.x)
    return float(message.twist.twist.linear.x)


def inspect_bag(bag: Path) -> dict[str, Any]:
    """Read one MCAP with ROS runtime dependencies loaded only on execution."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    tracked_topics = set(VELOCITY_TOPICS) | {BRUSH_TOPIC}
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in tracked_topics & topic_types.keys()
    }
    all_samples = {topic: [] for topic in VELOCITY_TOPICS}
    brush_on_samples = {topic: [] for topic in VELOCITY_TOPICS}
    brush_enabled = False
    brush_sample_count = 0

    while reader.has_next():
        topic, data, _timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == BRUSH_TOPIC:
            brush_enabled = bool(message.data)
            brush_sample_count += 1
            continue
        speed = _linear_x(message)
        all_samples[topic].append(speed)
        if brush_enabled:
            brush_on_samples[topic].append(speed)

    return {
        "schema": "tzcup.velocity_chain_inspection.v1",
        "bag": str(bag),
        "brush_enabled_sample_count": brush_sample_count,
        "velocity_topics": {
            topic: {
                "message_type": topic_types.get(topic),
                "all_samples": summarize(all_samples[topic]),
                "brush_on_samples": summarize(brush_on_samples[topic]),
            }
            for topic in VELOCITY_TOPICS
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True, help="Completed rosbag2 MCAP directory")
    parser.add_argument("--output", type=Path, help="Optional JSON report path")
    args = parser.parse_args()

    report = inspect_bag(args.bag)
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
