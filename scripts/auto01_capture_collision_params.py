#!/usr/bin/env python3
"""Capture selected parameters from both AUTO-01 collision-monitor stages."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


def get_parameters(node, remote_node: str, names: list[str]) -> dict:
    client = AsyncParameterClient(node, remote_node)
    if not client.wait_for_services(timeout_sec=20.0):
        raise RuntimeError(f"parameter service unavailable: {remote_node}")
    future = client.get_parameters(names)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"parameter request failed: {remote_node}")
    values = future.result().values
    if len(values) != len(names):
        raise RuntimeError(
            f"parameter response length mismatch for {remote_node}: "
            f"requested={len(names)} returned={len(values)}"
        )
    return {
        name: parameter_value_to_python(value)
        for name, value in zip(names, values)
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("auto01_collision_parameter_capture")
    try:
        high = get_parameters(
            node,
            "/collision_monitor",
            [
                "polygons",
                "source_timeout",
                "HighOverhangStop.enabled",
                "HighOverhangStop.action_type",
                "HighOverhangStop.min_points",
                "HighOverhangStop.points",
                "observation_sources",
                "high_overhang_cloud.topic",
                "high_overhang_cloud.min_height",
                "high_overhang_cloud.max_height",
            ],
        )
        ground = get_parameters(
            node,
            "/ground_collision_monitor",
            [
                "polygons",
                "source_timeout",
                "FootprintApproach.footprint_topic",
                "observation_sources",
                "scan.topic",
                "ground_cloud.topic",
                "ground_cloud.min_height",
                "ground_cloud.max_height",
                "cmd_vel_out_topic",
            ],
        )
        scan_filter = get_parameters(
            node,
            "/scan_self_filter",
            [
                "input_topic",
                "output_topic",
                "laser_origin_x_m",
                "mask_min_x_m",
                "mask_max_x_m",
                "mask_min_y_m",
                "mask_max_y_m",
            ],
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = {
        "schema_version": 1,
        "high_monitor": {
            "node": "/collision_monitor",
            "parameters": {
                "polygons": high["polygons"],
                "source_timeout": high["source_timeout"],
                "HighOverhangStop": {
                    "enabled": high["HighOverhangStop.enabled"],
                    "action_type": high["HighOverhangStop.action_type"],
                    "min_points": high["HighOverhangStop.min_points"],
                    "points": high["HighOverhangStop.points"],
                },
                "observation_sources": high["observation_sources"],
                "high_overhang_cloud": {
                    "topic": high["high_overhang_cloud.topic"],
                    "min_height": high["high_overhang_cloud.min_height"],
                    "max_height": high["high_overhang_cloud.max_height"],
                },
            },
        },
        "ground_monitor": {
            "node": "/ground_collision_monitor",
            "parameters": {
                "polygons": ground["polygons"],
                "source_timeout": ground["source_timeout"],
                "FootprintApproach": {
                    "footprint_topic": ground["FootprintApproach.footprint_topic"]
                },
                "observation_sources": ground["observation_sources"],
                "scan": {"topic": ground["scan.topic"]},
                "ground_cloud": {
                    "topic": ground["ground_cloud.topic"],
                    "min_height": ground["ground_cloud.min_height"],
                    "max_height": ground["ground_cloud.max_height"],
                },
                "cmd_vel_out_topic": ground["cmd_vel_out_topic"],
            },
        },
        "scan_self_filter": {
            "node": "/scan_self_filter",
            "parameters": scan_filter,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
