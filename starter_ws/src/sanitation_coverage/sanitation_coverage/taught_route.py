"""Fail-closed, hash-addressed taught-route contract and semantic compiler."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml

from .coverage_components import ComponentType, CoverageComponent
from .coverage_plan import CoveragePlan
from .metrics import point_in_cleanable_area


ALLOWED_DIRECTIONS = {"FORWARD", "BIDIRECTIONAL"}


def canonical_route_hash(payload: dict[str, Any]) -> str:
    canonical = dict(payload)
    canonical.pop("sha256", None)
    encoded = json.dumps(
        canonical, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def seal_taught_route(payload: dict[str, Any]) -> dict[str, Any]:
    sealed = dict(payload)
    sealed["sha256"] = canonical_route_hash(sealed)
    return sealed


def _pose(payload: Any, index: int) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError(f"pose {index} must be an object")
    required = {"x", "y", "yaw", "speed_limit_mps", "brush_enabled"}
    missing = sorted(required - payload.keys())
    if missing:
        raise ValueError(f"pose {index} missing fields: {missing}")
    values = {key: float(payload[key]) for key in ("x", "y", "yaw", "speed_limit_mps")}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError(f"pose {index} contains non-finite values")
    if values["speed_limit_mps"] <= 0.0:
        raise ValueError(f"pose {index} speed_limit_mps must be positive")
    if not isinstance(payload["brush_enabled"], bool):
        raise ValueError(f"pose {index} brush_enabled must be boolean")
    return {**values, "brush_enabled": payload["brush_enabled"]}


def compile_taught_route(
    payload: dict[str, Any], safe_polygon, exclusion_polygons=()
) -> CoveragePlan:
    required = {
        "route_id", "version", "frame_id", "poses", "allowed_direction",
        "no_clean_sections", "interaction_points", "recovery_points", "sha256",
    }
    missing = sorted(required - payload.keys())
    if missing:
        raise ValueError(f"taught route missing fields: {missing}")
    expected_hash = str(payload["sha256"]).removeprefix("sha256:")
    if len(expected_hash) != 64 or expected_hash != canonical_route_hash(payload):
        raise ValueError("taught route sha256 mismatch")
    direction = str(payload["allowed_direction"])
    if direction not in ALLOWED_DIRECTIONS:
        raise ValueError(f"unsupported allowed_direction: {direction}")
    poses = [_pose(item, index) for index, item in enumerate(payload["poses"])]
    if len(poses) < 2:
        raise ValueError("taught route requires at least two poses")
    for index, pose in enumerate(poses):
        if not point_in_cleanable_area(
            pose["x"], pose["y"], safe_polygon, exclusion_polygons
        ):
            raise ValueError(f"pose {index} leaves the footprint-safe polygon")

    no_clean_edges: set[int] = set()
    for item in payload["no_clean_sections"]:
        start, end = int(item["start_index"]), int(item["end_index"])
        if start < 0 or end > len(poses) - 1 or start >= end:
            raise ValueError("invalid no-clean section index range")
        no_clean_edges.update(range(start, end))
    for index in no_clean_edges:
        if poses[index]["brush_enabled"] or poses[index + 1]["brush_enabled"]:
            raise ValueError("no-clean section contains a brush-enabled pose")

    for field in ("interaction_points", "recovery_points"):
        for item in payload[field]:
            pose_index = int(item["pose_index"])
            if not 0 <= pose_index < len(poses):
                raise ValueError(f"{field} pose_index is outside the route")

    components = []
    for index, (start, end) in enumerate(zip(poses, poses[1:])):
        brush = bool(start["brush_enabled"] and end["brush_enabled"])
        kind = ComponentType.SWATH if brush else ComponentType.TRANSIT
        speed = min(start["speed_limit_mps"], end["speed_limit_mps"])
        components.append(CoverageComponent(
            component_id=f"taught-{index:03d}",
            kind=kind,
            points=((start["x"], start["y"]), (end["x"], end["y"])),
            brush_enabled=brush,
            speed_profile="TAUGHT_CLEAN" if brush else "TAUGHT_TRANSIT",
            metadata={
                "source_pose_index": index,
                "target_pose_index": index + 1,
                "speed_limit_mps": speed,
                "allowed_direction": direction,
                "collision_checked": True,
                "executor": "Nav2 FollowPath",
            },
        ))
    return CoveragePlan(
        mission_id=str(payload["route_id"]),
        frame_id=str(payload["frame_id"]),
        components=tuple(components),
        route_mode="TAUGHT_ROUTE",
        metadata={
            "route_id": str(payload["route_id"]),
            "route_version": str(payload["version"]),
            "route_sha256": expected_hash,
            "allowed_direction": direction,
            "interaction_points": list(payload["interaction_points"]),
            "recovery_points": list(payload["recovery_points"]),
            "offline_generated": True,
            "runtime_collision_check_required": True,
        },
    )


def load_taught_route(path: str | Path, safe_polygon, exclusion_polygons=()) -> CoveragePlan:
    route_path = Path(path)
    payload = yaml.safe_load(route_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("taught route file must contain an object")
    return compile_taught_route(payload, safe_polygon, exclusion_polygons)
