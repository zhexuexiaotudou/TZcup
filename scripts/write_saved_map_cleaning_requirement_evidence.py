#!/usr/bin/env python3
"""Write fail-closed localization and coverage requirement evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime
from pathlib import Path


def _read(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _sha256(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def _epoch_ns(value: object, fallback: Path) -> int:
    if isinstance(value, str):
        try:
            return int(datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp() * 1e9)
        except ValueError:
            pass
    return fallback.stat().st_mtime_ns if fallback.exists() else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--runtime-id", required=True)
    parser.add_argument("--restart-record", type=Path, required=True)
    parser.add_argument("--cleaning-runtime", type=Path, required=True)
    parser.add_argument("--coverage-report", type=Path, required=True)
    parser.add_argument("--route-sanity", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--artifact-manifest", type=Path, required=True)
    args = parser.parse_args()
    episode = _read(args.episode_manifest)
    binding = _read(args.runtime_binding)
    restart = _read(args.restart_record)
    runtime = _read(args.cleaning_runtime)
    coverage = _read(args.coverage_report)
    route = _read(args.route_sanity)
    episode_id = episode.get("episode_id")
    session_id = binding.get("acceptance_session_binding", {}).get("session_manifest_sha256")
    if not isinstance(episode_id, str) or not episode_id or not isinstance(session_id, str) or len(session_id) != 64:
        raise SystemExit("requirement evidence cannot bind episode/session identity")
    start_ns = _epoch_ns(restart.get("cleaning_start_wall_time"), args.restart_record)
    end_ns = args.cleaning_runtime.stat().st_mtime_ns if args.cleaning_runtime.exists() else start_ns
    identity = {"session_id": session_id, "runtime_id": args.runtime_id, "episode_id": episode_id}
    runtime_sha = _sha256(args.cleaning_runtime)
    coverage_sha = _sha256(args.coverage_report)
    route_sha = _sha256(args.route_sanity)
    return_home = coverage.get("return_home") if isinstance(coverage.get("return_home"), dict) else {}

    def row(requirement_id: str, observed: bool, source: str, artifact: Path, digest: str | None, metrics: dict, reason: str | None = None) -> dict:
        return {
            "requirement_id": requirement_id,
            "status": "observed" if observed else "missing",
            "start_epoch_ns": start_ns,
            "end_epoch_ns": end_ns,
            "source": source,
            "closure": "saved_map_cleaning_runtime",
            "artifact_path": str(artifact),
            "artifact_sha256": digest,
            "metrics": metrics,
            "missing_reason": reason if not observed else None,
            **identity,
        }

    localization = bool(runtime.get("amcl_pose_sample_count", 0) > 0 and runtime.get("nav2_action_ready") is True)
    started = bool(runtime.get("coverage_first_brush_enabled_monotonic_s") is not None)
    progress = bool(runtime.get("brush_enabled_distance_m", 0.0) > 0 and runtime.get("trajectory_total_distance_m", 0.0) > 0)
    home = bool(
        return_home.get("success") is True
        and return_home.get("session_id") == session_id
        and return_home.get("runtime_id") == args.runtime_id
        and return_home.get("episode_id") == episode_id
        and return_home.get("goal_frame_id") == "map"
        and return_home.get("final_cmd_vel_zero") is True
        and return_home.get("brush_control_released") is True
        and return_home.get("coverage_control_released") is True
    )
    completed = bool(runtime.get("coverage_action_terminal_passed") is True and coverage.get("success") is True and home)
    evidence = {
        "schema_version": 1,
        "identity": identity,
        "requirements": [
            row("localization_navigation", localization, "collect_formal_map_lifecycle_runtime", args.cleaning_runtime, runtime_sha, {"amcl_pose_sample_count": runtime.get("amcl_pose_sample_count", 0), "nav2_action_ready": runtime.get("nav2_action_ready")}, "AMCL pose or Nav2 readiness not observed"),
            row("coverage_start", started, "collect_formal_map_lifecycle_runtime", args.cleaning_runtime, runtime_sha, {"coverage_started_event": runtime.get("coverage_started_event"), "first_brush_enabled_monotonic_s": runtime.get("coverage_first_brush_enabled_monotonic_s")}, "brush-enabled coverage start not observed"),
            row("coverage_progress", progress, "collect_formal_map_lifecycle_runtime", args.cleaning_runtime, runtime_sha, {"brush_enabled_distance_m": runtime.get("brush_enabled_distance_m", 0.0), "trajectory_total_distance_m": runtime.get("trajectory_total_distance_m", 0.0), "coverage_ratio": runtime.get("estimated_coverage_fraction")}, "nonzero brush-enabled trajectory not observed"),
            row("coverage_complete", completed, "coverage_execution_and_route_sanity", args.coverage_report, coverage_sha, {"coverage_action_terminal_passed": runtime.get("coverage_action_terminal_passed"), "coverage_success": coverage.get("success"), "coverage_ratio": runtime.get("estimated_coverage_fraction"), "route_spacing_m": route.get("realized_lane_spacing_m"), "route_spacing_limit_m": route.get("recommended_max_lane_spacing_m"), "free_space_turn_continuity": not bool(route.get("disconnected_turn_rows")), "geofence_consumer_bundle_valid": route.get("passed") is True, "route_sanity_artifact_path": str(args.route_sanity), "route_sanity_artifact_sha256": route_sha, "return_home_observed": home}, "coverage terminal success and return-home closure not observed"),
            row("return_home", home, "formal_saved_map_coverage_executor", args.coverage_report, coverage_sha, return_home, "Nav2 home goal/result, pose tolerance, final stop, or control release not observed"),
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    artifacts = {str(path): digest for path, digest in ((args.cleaning_runtime, runtime_sha), (args.coverage_report, coverage_sha), (args.route_sanity, route_sha), (args.output, _sha256(args.output))) if digest is not None}
    args.artifact_manifest.write_text(json.dumps({"schema_version": 1, "identity": identity, "artifacts_sha256": artifacts}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
