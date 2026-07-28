#!/usr/bin/env python3
"""Capture selected parameters from the AUTO-01 G2 collision monitor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


COLLISION_PARAMETERS = [
    "polygons",
    "source_timeout",
    "cmd_vel_in_topic",
    "cmd_vel_out_topic",
    "FootprintApproach.footprint_topic",
    "FootprintApproach.action_type",
    "observation_sources",
    "scan.topic",
    "ground_cloud.topic",
    "ground_cloud.min_height",
    "ground_cloud.max_height",
]

FILTER_PARAMETERS = [
    "input_topic",
    "output_topic",
    "output_frame",
    "mask_min_xyz_m",
    "mask_max_xyz_m",
    "sampling_stride",
]


def get_parameters(node, remote: str, names: list[str]) -> dict:
    client = AsyncParameterClient(node, remote)
    if not client.wait_for_services(timeout_sec=20.0):
        raise RuntimeError(f"parameter service unavailable: {remote}")
    future = client.get_parameters(names)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"parameter request failed: {remote}")
    response_values = future.result().values
    if len(response_values) != len(names):
        raise RuntimeError(
            f"parameter response length mismatch for {remote}: "
            f"requested={len(names)} returned={len(response_values)}"
        )
    return dict(
        zip(
            names,
            [parameter_value_to_python(value) for value in response_values],
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("auto01_g2_parameter_capture")
    try:
        selected = get_parameters(
            node, "/collision_monitor", COLLISION_PARAMETERS
        )
        filter_selected = get_parameters(
            node, "/pointcloud_self_filter", FILTER_PARAMETERS
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = {
        "schema_version": 1,
        "node": "/collision_monitor",
        "parameters": {
            "polygons": selected["polygons"],
            "source_timeout": selected["source_timeout"],
            "cmd_vel_in_topic": selected["cmd_vel_in_topic"],
            "cmd_vel_out_topic": selected["cmd_vel_out_topic"],
            "FootprintApproach": {
                "footprint_topic": selected[
                    "FootprintApproach.footprint_topic"
                ],
                "action_type": selected["FootprintApproach.action_type"],
            },
            "observation_sources": selected["observation_sources"],
            "scan": {"topic": selected["scan.topic"]},
            "ground_cloud": {
                "topic": selected["ground_cloud.topic"],
                "min_height": selected["ground_cloud.min_height"],
                "max_height": selected["ground_cloud.max_height"],
            },
        },
        "pointcloud_self_filter": {
            "node": "/pointcloud_self_filter",
            "parameters": {
                "input_topic": filter_selected["input_topic"],
                "output_topic": filter_selected["output_topic"],
                "output_frame": filter_selected["output_frame"],
                "mask_min_xyz_m": filter_selected["mask_min_xyz_m"],
                "mask_max_xyz_m": filter_selected["mask_max_xyz_m"],
                "sampling_stride": filter_selected["sampling_stride"],
            },
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
