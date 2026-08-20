#!/usr/bin/env python3
"""Build reload routes and adjudicate the formal 20,000 m2 mapping chain."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import math
from pathlib import Path

import yaml


def _read_json(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def _read_json_fail_closed(path: Path, errors: dict[str, str]) -> dict:
    try:
        return _read_json(path)
    except (FileNotFoundError, json.JSONDecodeError, TypeError, ValueError) as error:
        errors[str(path)] = str(error)
        return {}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def select_reload_waypoints(
    exploration: dict,
    *,
    initial_xy=(0.0, 0.0),
    minimum_separation_m=15.0,
    maximum_waypoints=5,
) -> list[list[float]]:
    candidates = []
    for row in exploration.get("goals", []):
        if row.get("succeeded") is not True:
            continue
        point = (
            float(row["world_x_m"]),
            float(row["world_y_m"]),
            float(row.get("yaw_rad", 0.0)),
        )
        if all(math.isfinite(value) for value in point):
            candidates.append(point)
    selected = []
    anchors = [(float(initial_xy[0]), float(initial_xy[1]))]
    remaining = list(candidates)
    while remaining and len(selected) < int(maximum_waypoints):
        ranked = sorted(
            remaining,
            key=lambda point: min(
                math.hypot(point[0] - x, point[1] - y) for x, y in anchors
            ),
            reverse=True,
        )
        point = ranked[0]
        separation = min(
            math.hypot(point[0] - x, point[1] - y) for x, y in anchors
        )
        if separation < float(minimum_separation_m):
            break
        selected.append([point[0], point[1], point[2]])
        anchors.append((point[0], point[1]))
        remaining.remove(point)
    return selected


def build_route(args) -> int:
    exploration = _read_json(args.exploration)
    waypoints = select_reload_waypoints(
        exploration,
        initial_xy=(args.initial_x, args.initial_y),
        minimum_separation_m=args.minimum_separation_m,
        maximum_waypoints=args.maximum_waypoints,
    )
    if len(waypoints) < args.minimum_waypoints:
        raise ValueError(
            f"only {len(waypoints)} separated successful frontier goals; "
            f"need {args.minimum_waypoints}"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(waypoints, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0


def _map_image_path(map_yaml: Path) -> Path:
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    return (map_yaml.parent / str(metadata["image"])).resolve()


def evaluate(args) -> int:
    input_errors: dict[str, str] = {}
    inputs = {
        "exploration": _read_json_fail_closed(args.exploration, input_errors),
        "map_quality": _read_json_fail_closed(args.map_quality, input_errors),
        "map_geometry": _read_json_fail_closed(args.map_geometry, input_errors),
        "mapping_tf": _read_json_fail_closed(args.mapping_tf, input_errors),
        "reload_tf": _read_json_fail_closed(args.reload_tf, input_errors),
        "navigation": _read_json_fail_closed(args.navigation, input_errors),
        "processes": _read_json_fail_closed(args.processes, input_errors),
    }
    try:
        map_image = _map_image_path(args.map_yaml)
    except (FileNotFoundError, KeyError, TypeError, yaml.YAMLError) as error:
        input_errors[str(args.map_yaml)] = str(error)
        map_image = args.map_yaml.with_suffix(".missing-image")
    artifact_paths = {
        "map_yaml": args.map_yaml,
        "map_image": map_image,
        "posegraph": args.posegraph,
        "posegraph_data": args.posegraph_data,
        "reload_route": args.reload_route,
    }
    artifacts = {
        name: {
            "path": str(path),
            "exists": path.is_file(),
            "size_bytes": path.stat().st_size if path.is_file() else 0,
            "sha256": _sha256(path) if path.is_file() else None,
        }
        for name, path in artifact_paths.items()
    }
    artifacts_complete = all(
        row["exists"] and row["size_bytes"] > 0 for row in artifacts.values()
    )
    exploration = inputs["exploration"]
    quality = inputs["map_quality"]
    geometry = inputs["map_geometry"]
    processes = inputs["processes"]
    mapping_area_m2 = min(
        float(exploration.get("mapping_area_m2") or 0.0),
        float(quality.get("known_area_m2") or 0.0),
    )
    boundary_rmse = geometry.get("boundary_rmse_m")
    ghost_ratio = geometry.get("loop_ghosting_ratio")
    topology_failures = {
        "registration_failed": not bool(
            geometry.get("rigid_alignment", {}).get("optimizer_success")
        ),
        "boundary_rmse_exceeded": (
            boundary_rmse is None or float(boundary_rmse) > args.max_boundary_rmse_m
        ),
        "loop_ghosting_exceeded": (
            ghost_ratio is None or float(ghost_ratio) > args.max_loop_ghosting_ratio
        ),
    }
    topology_damage_count = sum(topology_failures.values())
    coordinate_frame_break_count = sum(
        int(inputs[name].get("coordinate_frame_break_count", 1))
        for name in ("mapping_tf", "reload_tf")
    )
    process_codes = processes.get("exit_codes", {})
    required_zero_codes = (
        "exploration", "map_save", "posegraph_serialize",
        "map_quality", "map_geometry", "route_build", "navigation",
    )
    process_pass = all(process_codes.get(name) == 0 for name in required_zero_codes)
    reproducibility = processes.get("reproducibility", {})
    config_hashes = reproducibility.get("config_sha256", {})
    reproducibility_pass = bool(
        re.fullmatch(r"[0-9a-f]{40,64}", str(reproducibility.get("source_commit", "")))
        and reproducibility.get("source_dirty") is False
        and isinstance(reproducibility.get("seed"), int)
        and str(reproducibility.get("command", "")).strip()
        and str(reproducibility.get("ros_distro", "")).strip()
        and isinstance(config_hashes, dict)
        and len(config_hashes) >= 5
        and all(re.fullmatch(r"[0-9a-f]{64}", str(value)) for value in config_hashes.values())
    )
    formal_scope = bool(processes.get("formal_scope"))
    restart_pass = bool(processes.get("restart_completed"))
    sensor_provenance = processes.get("sensor_provenance", {})
    positioning_graph_audits = sensor_provenance.get("runtime_graph_audits", {})
    positioning_provenance_pass = bool(
        sensor_provenance.get("positioning")
        == "gazebo_dual_navsat_rtk_plus_wheel_imu_plus_scan_matching"
        and sensor_provenance.get("gazebo_dual_navsat_sensor_pair") is True
        and sensor_provenance.get("gazebo_truth_to_gnss_sensor_model") is False
        and sensor_provenance.get("all_runtime_graph_audits_pass") is True
        and sensor_provenance.get("ground_truth_ros_subscription_in_positioning")
        is False
        and sensor_provenance.get("oracle_pose_topic_to_controller") is False
        and set(positioning_graph_audits) == {"mapping", "reload"}
        and all(
            isinstance(audit, dict) and audit.get("pass") is True
            for audit in positioning_graph_audits.values()
        )
    )
    reload_navigation_pass = bool(
        artifacts_complete
        and restart_pass
        and inputs["reload_tf"].get("continuous") is True
        and inputs["navigation"].get("success") is True
        and int(inputs["navigation"].get("waypoint_count", 0)) >= 3
    )
    checks = {
        "formal_scope": formal_scope,
        "continuous_mapping_pass": exploration.get("success") is True,
        "mapping_area_at_least_20000m2": mapping_area_m2 >= 20_000.0,
        "saved_map_quality_pass": quality.get("slam_quality_pass") is True,
        "artifacts_complete": artifacts_complete,
        "restart_completed": restart_pass,
        "reload_relocalize_navigation_pass": reload_navigation_pass,
        "coordinate_frame_break_count_zero": coordinate_frame_break_count == 0,
        "topology_damage_count_zero": topology_damage_count == 0,
        "all_processes_exit_zero": process_pass,
        "reproducibility_bound": reproducibility_pass,
        "ground_truth_not_used_for_control": (
            exploration.get("ground_truth_used_for_control") is False
            and positioning_provenance_pass
        ),
    }
    success = all(checks.values())
    smoke_chain_checks = {
        name: value
        for name, value in checks.items()
        if name not in {
            "formal_scope",
            "mapping_area_at_least_20000m2",
            "topology_damage_count_zero",
        }
    }
    smoke_chain_pass = (not formal_scope) and all(smoke_chain_checks.values())
    report = {
        "schema_version": 1,
        "stage": "PRODUCT-MAPPING-20000M2",
        "success": success,
        "smoke_chain_pass": smoke_chain_pass,
        "SIMULATION_PRODUCT_COMPLETE": False,
        "formal_scope": formal_scope,
        "mapping_area_m2": mapping_area_m2,
        "map_save_load_relocalize_navigation_pass": reload_navigation_pass,
        "coordinate_frame_break_count": coordinate_frame_break_count,
        "topology_damage_count": topology_damage_count,
        "topology_thresholds": {
            "maximum_boundary_rmse_m": args.max_boundary_rmse_m,
            "maximum_loop_ghosting_ratio": args.max_loop_ghosting_ratio,
        },
        "topology_failures": topology_failures,
        "checks": checks,
        "smoke_chain_checks": smoke_chain_checks,
        "artifacts": artifacts,
        "input_reports": {name: str(path) for name, path in {
            "exploration": args.exploration,
            "map_quality": args.map_quality,
            "map_geometry": args.map_geometry,
            "mapping_tf": args.mapping_tf,
            "reload_tf": args.reload_tf,
            "navigation": args.navigation,
            "processes": args.processes,
        }.items()},
        "claim_boundary": (
            "This report adjudicates product gate B mapping only; it never "
            "sets full simulation product completion."
        ),
        "input_errors": input_errors,
    }
    if args.output.exists() and not args.allow_overwrite:
        raise FileExistsError(f"refusing to overwrite {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(args.output)
    return 0 if success or smoke_chain_pass else 2


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser()
    subparsers = root.add_subparsers(dest="command", required=True)
    route = subparsers.add_parser("build-route")
    route.add_argument("--exploration", required=True, type=Path)
    route.add_argument("--output", required=True, type=Path)
    route.add_argument("--initial-x", type=float, default=0.0)
    route.add_argument("--initial-y", type=float, default=0.0)
    route.add_argument("--minimum-separation-m", type=float, default=15.0)
    route.add_argument("--minimum-waypoints", type=int, default=3)
    route.add_argument("--maximum-waypoints", type=int, default=5)
    route.set_defaults(handler=build_route)
    adjudicate = subparsers.add_parser("evaluate")
    for name in (
        "exploration", "map_quality", "map_geometry", "mapping_tf",
        "reload_tf", "navigation", "processes",
    ):
        adjudicate.add_argument(f"--{name.replace('_', '-')}", required=True, type=Path)
    adjudicate.add_argument("--map-yaml", required=True, type=Path)
    adjudicate.add_argument("--posegraph", required=True, type=Path)
    adjudicate.add_argument("--posegraph-data", required=True, type=Path)
    adjudicate.add_argument("--reload-route", required=True, type=Path)
    adjudicate.add_argument("--output", required=True, type=Path)
    adjudicate.add_argument("--max-boundary-rmse-m", type=float, default=0.15)
    adjudicate.add_argument("--max-loop-ghosting-ratio", type=float, default=0.02)
    adjudicate.add_argument("--allow-overwrite", action="store_true")
    adjudicate.set_defaults(handler=evaluate)
    return root


def main() -> int:
    args = parser().parse_args()
    try:
        return args.handler(args)
    except (FileNotFoundError, FileExistsError, KeyError, TypeError, ValueError) as error:
        print(f"product mapping acceptance error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
