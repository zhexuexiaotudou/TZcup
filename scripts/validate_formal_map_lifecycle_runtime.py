#!/usr/bin/env python3
"""Fail-closed aggregator for first-map then saved-map cleaning evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import yaml

from formal_runtime_gate_binding import RuntimeGateError, load_binding


PASS_STATUS = "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED"
BLOCKED_STATUS = "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_BLOCKED"
ROOT = Path(__file__).resolve().parents[1]
FORMAL_CAMPUS_PACKAGE = ROOT / "starter_ws/src/sanitation_formal_campus_integration"
FORMAL_SPEED_PROFILES = (
    ROOT / "config/high_fidelity_vehicle/formal_operation_speed_profiles.yaml"
)

if str(FORMAL_CAMPUS_PACKAGE) not in sys.path:
    sys.path.insert(0, str(FORMAL_CAMPUS_PACKAGE))

from sanitation_formal_campus_integration.saved_map_coverage_core import (
    DRY_CLEANING_SPEED_PROFILE,
    MAPPING_SAFE_SPEED_PROFILE,
    SavedMapCoverageError,
    load_formal_operation_speed_profile,
)
from sanitation_formal_campus_integration.runtime_evidence_core import (
    COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S,
    EXPECTED_COMMAND_TOPIC_PUBLISHER,
)


def _json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _atomic_write_json(path: Path, value: dict) -> None:
    """Write one retained acceptance object without exposing a partial file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(path.name + f".pending.{os.getpid()}")
    try:
        pending.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        pending.replace(path)
    finally:
        if pending.exists():
            pending.unlink()


def _embed_runtime_binding(report: dict, runtime_binding_path: Path) -> dict:
    """Attach the exact pre-runtime binding; never reconstruct a projection."""
    binding = load_binding(runtime_binding_path)
    report["runtime_gate_binding"] = binding
    report["acceptance_session_binding"] = binding["acceptance_session_binding"]
    report["runtime_closure_binding"] = binding["runtime_closure_binding"]
    return binding


def write_bound_report(output: Path, report: dict, runtime_binding_path: Path) -> None:
    """Publish the canonical report and its identical binding sidecar in order."""
    binding = _embed_runtime_binding(report, runtime_binding_path)
    sidecar = output.with_name(output.name + ".runtime_binding.json")
    # The acceptance orchestrator rejects a sidecar newer than its report.
    _atomic_write_json(sidecar, binding)
    _atomic_write_json(output, report)


def _hashes_valid(root: Path, manifest: dict) -> bool:
    occupancy_name = manifest.get("occupancy_map")
    if occupancy_name != "occupancy.yaml":
        return False
    try:
        metadata = yaml.safe_load(
            (root / occupancy_name).read_text(encoding="utf-8")
        )
    except (OSError, yaml.YAMLError):
        return False
    image_name = metadata.get("image") if isinstance(metadata, dict) else None
    if (
        image_name != "occupancy.pgm"
        or Path(image_name).is_absolute()
        or Path(image_name).name != image_name
        or "/" in image_name
        or "\\" in image_name
    ):
        return False
    required = {
        occupancy_name,
        image_name,
        "mission_geometry.yaml",
        "materialization_contract.yaml",
        "geofence_keepout.yaml",
        "geofence_keepout.pgm",
        "neutral_speed.yaml",
        "neutral_speed.pgm",
    }
    hashes = manifest.get("sha256")
    if not isinstance(hashes, dict) or set(hashes) != required:
        return False
    resolved_root = root.resolve()
    return all(
        isinstance(expected, str)
        and len(expected) == 64
        and Path(name).name == name
        and "/" not in name
        and "\\" not in name
        and not (root / name).is_symlink()
        and (root / name).is_file()
        and (root / name).resolve().parent == resolved_root
        and hashlib.sha256((root / name).read_bytes()).hexdigest() == expected
        for name, expected in hashes.items()
    )


def _report_matches_speed_profile(report: object, expected_profile: object) -> bool:
    """Bind a coverage report to the selected, source-owned speed profile."""
    if not isinstance(report, dict):
        return False
    if report.get("operation_speed_profile") != getattr(expected_profile, "name", None):
        return False
    speed = report.get("maximum_linear_speed_mps")
    if isinstance(speed, bool) or not isinstance(speed, (int, float)):
        return False
    return math.isclose(
        float(speed),
        float(getattr(expected_profile, "maximum_linear_speed_mps", math.nan)),
        abs_tol=1e-12,
    )


def validate(
    map_root: Path,
    mapping_runtime: Path,
    cleaning_runtime: Path,
    *,
    speed_profiles_path: Path = FORMAL_SPEED_PROFILES,
) -> dict:
    manifest = _json(map_root / "map_lifecycle_manifest.json")
    mapping = _json(mapping_runtime)
    cleaning = _json(cleaning_runtime)
    try:
        mapping_speed_profile = load_formal_operation_speed_profile(
            speed_profiles_path, MAPPING_SAFE_SPEED_PROFILE
        )
        dry_cleaning_speed_profile = load_formal_operation_speed_profile(
            speed_profiles_path, DRY_CLEANING_SPEED_PROFILE
        )
    except SavedMapCoverageError:
        mapping_speed_profile = dry_cleaning_speed_profile = None
    try:
        observed_fraction = float(manifest.get("observed_fraction", 0.0))
        quality_threshold = float(manifest.get("quality_threshold", 0.0))
        stable_samples = int(manifest.get("stable_gate_samples", 0))
    except (TypeError, ValueError):
        observed_fraction = quality_threshold = math.nan
        stable_samples = 0
    checks = {
        "quality_gated_map_manifest": (
            manifest.get("schema_version") == 1
            and manifest.get("status") == "ready_for_localization_cleaning"
            and math.isfinite(observed_fraction)
            and math.isfinite(quality_threshold)
            and quality_threshold >= 0.95
            and observed_fraction >= quality_threshold
            and stable_samples >= 3
            and manifest.get("fixed_start_verified") is True
            and manifest.get("gnss_mapping_reference_observed") is True
            and manifest.get("mapping_pose_source")
            == "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"
            and manifest.get("world_truth_used_for_control") is False
            and manifest.get("mapping_ignored_dirt") is True
            and _hashes_valid(map_root, manifest)
        ),
        "mapping_runtime_passed": (
            mapping.get("passed") is True
            and mapping.get("truth_used_for_control") is False
            and mapping.get("collision_monitor_nodes") == ["/collision_monitor"]
            and mapping.get("cmd_vel_gate_publishers") == ["/collision_monitor"]
            and mapping.get("base_command_publishers") == [
                "/whole_vehicle_safety_manager"
            ]
            and mapping.get("command_topic_publishers") == {
                topic: [publisher]
                for topic, publisher in EXPECTED_COMMAND_TOPIC_PUBLISHER.items()
            }
            and mapping.get("command_chain_publishers_attributed") is True
            and mapping.get("odom_tf_publisher_count") == 1
            and float(mapping.get("odom_tf_min_rate_hz", 0.0)) >= 10.0
            and mapping.get("slam_map_observed") is True
            and mapping.get("slam_odom_failures_after_ready") == 0
            # The 0.10 m lower bound is the pre-existing collector runtime
            # contract; this validator adds the command-chain attribution and
            # freshness evidence without changing its physical threshold.
            and float(mapping.get("odom_displacement_m", 0.0)) >= 0.10
            and mapping.get("filtered_scan_sample_count", 0) > 0
            and mapping.get("collision_monitor_state_sample_count", 0) > 0
            and mapping.get("command_chain_live") is True
            and mapping.get("command_chain_first_nonzero_ordered") is True
            and mapping.get("command_chain_receipt_reorder_tolerance_s")
            == COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S
            and mapping.get("active_command_chain_window_definition")
            == "status_after_first_nonzero_base_output_with_base_command_enabled_true"
            and mapping.get("active_command_chain_safety_sample_count", 0) > 0
            and mapping.get("active_command_chain_command_timeout_count") == 0
        ),
        # Mapping remains governed by the source-owned mapping_safe profile.
        # Do not inherit the dry cleaning speed into its separate runtime.
        "mapping_safe_profile_retains_0_45_m_s": (
            mapping_speed_profile is not None
            and math.isclose(
                mapping_speed_profile.maximum_linear_speed_mps, 0.45, abs_tol=1e-12
            )
        ),
        "saved_map_cleaning_runtime_passed": (
            cleaning.get("passed") is True
            and cleaning.get("truth_used_for_control") is False
            and cleaning.get("localization_backend") == "amcl"
            and cleaning.get("saved_map_sha256_verified") is True
            and cleaning.get("world_derived_map_fallback") is False
            and cleaning.get("collision_monitor_node_count") == 1
            and cleaning.get("cmd_vel_gate_publisher_count") == 1
            and cleaning.get("cleaning_stack_ready") is True
            and cleaning.get("coverage_server_ready") is True
            and cleaning.get("hard_restart_verified") is True
            and cleaning.get("coverage_action_terminal_passed") is True
            and cleaning.get("coverage_state", {}).get("state") == "COMPLETED"
            and cleaning.get("coverage_execution_report", {}).get("success") is True
            and cleaning.get("coverage_execution_report", {}).get(
                "terminal_state"
            ) == "COMPLETED"
            and cleaning.get("coverage_execution_report", {}).get(
                "ground_truth_used_for_control"
            ) is False
            and cleaning.get("coverage_execution_report", {}).get(
                "operation_width_m"
            ) == 1.32
            and dry_cleaning_speed_profile is not None
            and _report_matches_speed_profile(
                cleaning.get("coverage_execution_report"), dry_cleaning_speed_profile
            )
            and int(cleaning.get("coverage_execution_report", {}).get(
                "planned_swath_count", 0
            )) > 0
            and cleaning.get("coverage_execution_report", {}).get(
                "completed_swath_count"
            ) == cleaning.get("coverage_execution_report", {}).get(
                "planned_swath_count"
            )
            and float(cleaning.get("trajectory_total_distance_m", 0.0)) > 0.0
            and float(cleaning.get("brush_enabled_distance_m", 0.0)) > 0.0
            and int(cleaning.get("brush_state_sample_count", 0)) >= 2
            and int(cleaning.get("brush_state_transitions", 0)) >= 2
            and cleaning.get("brush_state_source")
            == "/brush_enabled_product_runtime"
            and cleaning.get("brush_disabled_on_exit") is True
            and float(cleaning.get("estimated_coverage_fraction", 0.0)) >= 0.95
        ),
    }
    passed = all(checks.values())
    return {
        "schema_version": 1,
        "status": PASS_STATUS if passed else BLOCKED_STATUS,
        "passed": passed,
        "checks": checks,
        "blockers": [name for name, value in checks.items() if not value],
        "truth_used_for_control": False,
        "operation_speed_profiles": {
            "mapping_safe": (
                mapping_speed_profile.maximum_linear_speed_mps
                if mapping_speed_profile is not None
                else None
            ),
            "dry_cleaning": (
                dry_cleaning_speed_profile.maximum_linear_speed_mps
                if dry_cleaning_speed_profile is not None
                else None
            ),
        },
        "claim_boundary": (
            "PASS requires a real 200x100 SLAM exploration, >=95% observed "
            "map save and a separate saved-map AMCL cleaning runtime."
        ),
        "evidence": {
            "map_root": str(map_root),
            "mapping_runtime": str(mapping_runtime),
            "cleaning_runtime": str(cleaning_runtime),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-root", required=True, type=Path)
    parser.add_argument("--mapping-runtime", required=True, type=Path)
    parser.add_argument("--cleaning-runtime", required=True, type=Path)
    parser.add_argument("--runtime-binding", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = validate(args.map_root, args.mapping_runtime, args.cleaning_runtime)
        write_bound_report(args.output, report, args.runtime_binding)
    except (OSError, RuntimeGateError, TypeError, ValueError, KeyError) as exc:
        print(f"FORMAL_MAP_LIFECYCLE_RUNTIME_BINDING_BLOCKED: {exc}", file=sys.stderr)
        return 2
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
