#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml


def _parameters(document: dict) -> dict:
    if not document:
        return {}
    node = next(iter(document.values()))
    return node.get("ros__parameters", {})


def _polygon(value) -> list[list[float]]:
    if isinstance(value, str):
        value = json.loads(value)
    return [[float(axis) for axis in point] for point in value]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    expected = _polygon(profile["footprint_xy_m"])
    local = _parameters(yaml.safe_load((args.trial / "runtime_local_costmap_params.yaml").read_text(encoding="utf-8")))
    global_ = _parameters(yaml.safe_load((args.trial / "runtime_global_costmap_params.yaml").read_text(encoding="utf-8")))
    nav2 = yaml.safe_load((args.trial / "nav2_stage5br6w_v4.yaml").read_text(encoding="utf-8"))
    collision = nav2["collision_monitor"]["ros__parameters"]
    coverage = json.loads((args.trial / "coverage_report.json").read_text(encoding="utf-8"))
    expected_radius = max(math.hypot(*point) for point in expected)
    checks = {
        "local_costmap_same_candidate_footprint": _polygon(local.get("footprint", [])) == expected,
        "global_costmap_same_candidate_footprint": _polygon(global_.get("footprint", [])) == expected,
        "collision_monitor_consumes_local_published_footprint": collision.get("FootprintApproach", {}).get("footprint_topic") == "/local_costmap/published_footprint",
        "coverage_mission_geometry_same_candidate_radius": abs(float(coverage["mission_geometry"]["footprint_radius_m"]) - expected_radius) <= 1e-9,
        "coverage_profile_recorded": coverage.get("mission_geometry", {}).get("configured_headland_width_m") == float(profile["coverage_overrides"]["headland_width_m"]),
        "local_published_footprint_observed": (args.trial / "runtime_local_published_footprint.yaml").stat().st_size > 0,
        "global_published_footprint_observed": (args.trial / "runtime_global_published_footprint.yaml").stat().st_size > 0,
    }
    report = {
        "schema_version": 1,
        "stage": "Stage5BR6W",
        "profile": profile["profile"],
        "expected_footprint_xy_m": expected,
        "checks": checks,
        "all_runtime_consumers_same_candidate_footprint": all(checks.values()),
        "production_default_unchanged": profile["production_default_unchanged"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if report["all_runtime_consumers_same_candidate_footprint"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
