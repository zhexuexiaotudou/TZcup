#!/usr/bin/env python3
"""Combine the two independent formal water-recovery runtime episodes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import load_binding


NORMAL_REQUIRED_CHECKS = (
    "finite_initial_ground_water",
    "all_physical_proxy_conditions_seen",
    "squeegee_blade_has_ground_contact_during_recovery",
    "brush_disks_have_ground_contact_during_recovery",
    "vehicle_physically_advanced",
    "nozzle_covered_all_24_water_columns",
    "recovery_rate_at_least_0_95",
    "ground_to_tank_mass_error_at_most_0_01",
    "dynamic_payload_applied_matches_tank",
    "visual_water_fraction_matches_ground_state",
    "side_brush_motor_samples_present",
    "side_brush_steady_current_within_0_75_a_continuous_rating",
    "side_brush_over_rating_contiguous_at_most_1_s",
    "side_brush_peak_temperature_below_60_c",
    "side_brush_steady_p05_speed_ratio_at_least_0_80",
    "side_brush_direction_matches_command",
    "side_brush_low_speed_contiguous_at_most_1_s",
    "side_brush_fault_free_throughout_normal_pass",
    "side_brush_telemetry_fields_finite",
    "central_roller_motor_samples_present",
    "central_roller_steady_current_within_0_75_a_continuous_rating",
    "central_roller_over_rating_contiguous_at_most_1_s",
    "central_roller_peak_temperature_below_60_c",
    "central_roller_steady_p05_speed_ratio_at_least_0_80",
    "central_roller_direction_matches_command",
    "central_roller_low_speed_contiguous_at_most_1_s",
    "central_roller_fault_free_throughout_normal_pass",
    "central_roller_telemetry_fields_finite",
)
FULL_REQUIRED_CHECKS = (
    "tank_reaches_full",
    "water_remains_when_tank_full",
    "full_tank_stops_ground_removal",
    "full_tank_stops_flow",
    "full_case_mass_error_at_most_0_01",
    "dynamic_payload_applied_matches_full_tank",
    "active_recovery_blocks_service_drain",
    "service_drain_reduces_tank_mass",
    "service_drain_stationary_interlock_permitted",
    "service_drain_reports_removed_volume",
    "service_drain_closes_and_updates_payload",
    "side_brush_fault_free_throughout_full_case",
    "central_roller_fault_free_throughout_full_case",
)
TYPED_DIAG_REQUIRED_CHECKS = (
    "all_snapshots_parse_as_63_finite_values",
    "first_frame_below_0_5_s",
    "maximum_gap_at_most_75_ms",
    "no_burst_gap_below_20_ms",
    "physics_revision_advances",
    "physics_revision_never_moves_backwards",
    "physics_revision_stagnation_below_0_75_s",
    "rate_18_to_22_hz",
    "raw_trace_contains_every_received_frame",
    "ros_status_json_has_zero_publishers",
    "steady_samples_not_physics_stale",
    "telemetry_sequence_strictly_increasing",
)
TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS = (
    "gazebo_topic_type_is_double_v",
    "launch_log_audit_passed",
    "ros_topic_type_is_float64_multi_array",
    "ros_typed_topic_has_one_publisher",
    "zero_nodeshared_publish_errors",
    "zero_topic_tagged_publish_failures",
)
TYPED_CRITICAL_MANIFEST_REQUIRED_PATHS = {
    "scripts/collect_formal_typed_cleaning_motor_diagnostic.py",
    "scripts/run_formal_typed_cleaning_motor_diagnostic.sh",
    "patches/upstream/gz_transport13/0001-publish-retry-current-frame-on-eintr.patch",
    "patches/upstream/gz_transport13/manifest.json",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorVectorBridge.cc",
    "starter_ws/src/sanitation_gazebo_control/src/WaterEvaluationBridge.cc",
    "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",
}


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _regular_file_matches(path_value: object, expected_sha256: object) -> bool:
    """Return whether a report-owned path is still its recorded regular file."""

    if not isinstance(path_value, str) or not isinstance(expected_sha256, str):
        return False
    path = Path(path_value)
    return (
        path.is_file()
        and not path.is_symlink()
        and _sha256(path) == expected_sha256
    )


def _preembedded_world_binding_valid(
    binding: object,
    *,
    source_binding: dict[str, Any],
    acceptance_session_binding: dict[str, Any],
    runtime_binding: dict[str, Any],
) -> bool:
    """Recheck the scenario's contact-world identity before publishing water PASS."""

    if not isinstance(binding, dict):
        return False
    runtime_closure = runtime_binding.get("runtime_closure_binding")
    if not isinstance(runtime_closure, dict):
        return False
    if (
        binding.get("spawn_mode") != "preembedded_before_gazebo_sensors_system"
        or binding.get("model_name") != "tzcup_formal_sanitation_vehicle"
        or not isinstance(binding.get("sensor_count"), int)
        or binding["sensor_count"] <= 0
        or binding.get("snapshot") != source_binding
        or binding.get("source_urdf_sha256")
        != source_binding.get("expanded_urdf_sha256")
        or binding.get("acceptance_session_sha256")
        != acceptance_session_binding.get("session_manifest_sha256")
        or binding.get("runtime_install_root")
        != runtime_closure.get("runtime_install_root")
    ):
        return False
    return all(
        (
            _regular_file_matches(
                binding.get("preembedded_report_path"),
                binding.get("preembedded_report_sha256"),
            ),
            _regular_file_matches(
                binding.get("preembedded_world_path"),
                binding.get("preembedded_world_sha256"),
            ),
            _regular_file_matches(
                binding.get("source_world_path"), binding.get("source_world_sha256")
            ),
            _regular_file_matches(
                binding.get("source_urdf_path"), binding.get("source_urdf_sha256")
            ),
            _regular_file_matches(
                binding.get("controller_config_path"),
                binding.get("controller_config_sha256"),
            ),
        )
    )


def _load_jsonl_objects(path: Path) -> list[dict[str, Any]] | None:
    """Read the same strict JSONL shape required by the final orchestrator."""

    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            return None
        if not isinstance(row, dict):
            return None
        rows.append(row)
    return rows


def combine(
    normal_path: Path,
    full_path: Path,
    normal_surface_path: Path,
    full_surface_path: Path,
    typed_diag_path: Path | None = None,
    raw_trace_path: Path | None = None,
    typed_runner_path: Path | None = None,
    typed_collector_path: Path | None = None,
    critical_source_manifest_path: Path | None = None,
    typed_cleaning_telemetry_source_sha256: str | None = None,
    runtime_binding_path: Path | None = None,
) -> dict[str, Any]:
    normal = _load(normal_path)
    full = _load(full_path)
    normal_surface = _load(normal_surface_path)
    full_surface = _load(full_surface_path)
    # Programmatic unit callers may exercise the pure scenario aggregation
    # without a live formal session.  The CLI used to publish canonical
    # evidence requires --runtime-binding, so it never takes this path.
    runtime_binding_valid = runtime_binding_path is None
    preembedded_world_bindings_valid = runtime_binding_path is None
    runtime_binding: dict[str, Any] | None = None
    source_binding: dict[str, Any] | None = None
    acceptance_session_binding: dict[str, Any] | None = None
    if runtime_binding_path is not None and runtime_binding_path.is_file():
        try:
            runtime_binding = load_binding(runtime_binding_path)
            candidate_session = runtime_binding.get("acceptance_session_binding")
            if isinstance(candidate_session, dict) and isinstance(
                candidate_session.get("snapshot"), dict
            ):
                acceptance_session_binding = candidate_session
                source_binding = candidate_session["snapshot"]
                runtime_binding_valid = (
                    normal.get("source_binding") == source_binding
                    and full.get("source_binding") == source_binding
                    and normal.get("acceptance_session_binding")
                    == acceptance_session_binding
                    and full.get("acceptance_session_binding")
                    == acceptance_session_binding
                    and normal.get("runtime_gate_binding") == runtime_binding
                    and full.get("runtime_gate_binding") == runtime_binding
                )
                preembedded_world_bindings_valid = (
                    _preembedded_world_binding_valid(
                        normal.get("preembedded_sensor_world_binding"),
                        source_binding=source_binding,
                        acceptance_session_binding=acceptance_session_binding,
                        runtime_binding=runtime_binding,
                    )
                    and _preembedded_world_binding_valid(
                        full.get("preembedded_sensor_world_binding"),
                        source_binding=source_binding,
                        acceptance_session_binding=acceptance_session_binding,
                        runtime_binding=runtime_binding,
                    )
                )
        except (OSError, ValueError, RuntimeError):
            runtime_binding_valid = False
            preembedded_world_bindings_valid = False
    scenario_names_valid = (
        normal.get("scenario") == "normal_recovery"
        and full.get("scenario") == "full_tank_fail_closed"
    )
    normal_checks = normal.get("checks", {})
    full_checks = full.get("checks", {})
    normal_physics_checks_valid = all(
        normal_checks.get(name) is True for name in NORMAL_REQUIRED_CHECKS
    )
    full_interlock_checks_valid = all(
        full_checks.get(name) is True for name in FULL_REQUIRED_CHECKS
    )
    side_brush_expanded_sdf_surface_valid = (
        normal_surface.get("status")
        == "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED"
        and full_surface.get("status")
        == "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED"
        and normal_surface.get("expanded_sdf_sha256")
        == full_surface.get("expanded_sdf_sha256")
        and normal_surface.get("expanded_sdf_sha256") is not None
        and normal_surface.get("schema_version") == 2
        and full_surface.get("schema_version") == 2
        and normal_surface.get("central_roller", {}).get("collision")
        == "central_roller_link_collision"
        and full_surface.get("central_roller", {}).get("collision")
        == "central_roller_link_collision"
        and normal_surface.get("central_roller", {}).get("radius_m") == 0.100
        and full_surface.get("central_roller", {}).get("radius_m") == 0.100
        and normal_surface.get("central_roller", {}).get("length_m") == 0.620
        and full_surface.get("central_roller", {}).get("length_m") == 0.620
        and normal_surface.get("central_roller", {}).get("surface", {}).get("mu")
        == 0.08
        and normal_surface.get("central_roller", {}).get("surface", {}).get("mu2")
        == 0.08
        and full_surface.get("central_roller", {}).get("surface", {}).get("mu")
        == 0.08
        and full_surface.get("central_roller", {}).get("surface", {}).get("mu2")
        == 0.08
    )
    typed_paths = (
        typed_diag_path,
        raw_trace_path,
        typed_runner_path,
        typed_collector_path,
        critical_source_manifest_path,
    )
    typed_transport_valid = False
    typed_transport: dict[str, Any] = {
        "contract": {
            "ros_type": "std_msgs/msg/Float64MultiArray",
            "gz_type": "gz.msgs.Double_V",
            "snapshot_length": 63,
            "status_transport": "gazebo_only_diagnostic",
        }
    }
    if all(
        path is not None and path.is_file() and not path.is_symlink()
        for path in typed_paths
    ):
        resolved = [path.resolve() for path in typed_paths if path is not None]
        typed_diag = _load(typed_diag_path)
        source_manifest = _load(critical_source_manifest_path)
        raw_trace_rows = _load_jsonl_objects(raw_trace_path)
        raw_trace_count = len(raw_trace_rows) if raw_trace_rows is not None else 0
        diag_checks = typed_diag.get("checks", {})
        diag_metrics = typed_diag.get("metrics", {})
        recorded_trace_count = diag_metrics.get("raw_trace_frame_count", -1)
        digest_valid = bool(
            typed_cleaning_telemetry_source_sha256
            and len(typed_cleaning_telemetry_source_sha256) == 64
            and all(
                character in "0123456789abcdef"
                for character in typed_cleaning_telemetry_source_sha256
            )
        )
        transport_audit = typed_diag.get("transport_audit")
        transport_checks = (
            transport_audit.get("checks", {})
            if isinstance(transport_audit, dict)
            else {}
        )
        transport_files_valid = False
        if isinstance(transport_audit, dict):
            audit_file_fields = (
                ("launch_log", "launch_log_sha256"),
                ("launch_audit_json", "launch_audit_sha256"),
                ("gazebo_topic_info", "gazebo_topic_info_sha256"),
                ("ros_topic_info", "ros_topic_info_sha256"),
            )
            audit_paths: list[Path] = []
            for path_key, hash_key in audit_file_fields:
                raw_path = transport_audit.get(path_key)
                expected_hash = transport_audit.get(hash_key)
                if not isinstance(raw_path, str) or not isinstance(expected_hash, str):
                    audit_paths = []
                    break
                candidate = Path(raw_path)
                if (
                    candidate.is_symlink()
                    or not candidate.is_file()
                    or candidate.resolve().parent != typed_diag_path.resolve().parent
                    or _sha256(candidate) != expected_hash
                ):
                    audit_paths = []
                    break
                audit_paths.append(candidate.resolve())
            transport_files_valid = (
                len(audit_paths) == len(audit_file_fields)
                and len(set(audit_paths)) == len(audit_paths)
            )
        critical_rows = source_manifest.get("critical_files")
        critical_paths = {
            row.get("path")
            for row in critical_rows
            if isinstance(row, dict) and isinstance(row.get("path"), str)
        } if isinstance(critical_rows, list) else set()
        critical_hashes_valid = bool(critical_rows) and all(
            isinstance(row, dict)
            and isinstance(row.get("source_sha256"), str)
            and len(row["source_sha256"]) == 64
            and all(character in "0123456789abcdef" for character in row["source_sha256"])
            and row.get("source_matches_frozen_copy", True) is True
            for row in critical_rows
        )
        typed_transport_valid = (
            len(set(resolved)) == len(resolved)
            and typed_diag.get("status") == "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED"
            and typed_diag.get("passed") is True
            and set(diag_checks) == set(TYPED_DIAG_REQUIRED_CHECKS)
            and all(diag_checks.get(name) is True for name in TYPED_DIAG_REQUIRED_CHECKS)
            and raw_trace_rows is not None
            and raw_trace_count > 0
            and isinstance(recorded_trace_count, int)
            and raw_trace_count == recorded_trace_count
            and source_manifest.get("schema_version") == 1
            and source_manifest.get("source_package_files_match_frozen_copy") is True
            and source_manifest.get("install_symlink_count") == 0
            and TYPED_CRITICAL_MANIFEST_REQUIRED_PATHS <= critical_paths
            and critical_hashes_valid
            and isinstance(transport_audit, dict)
            and transport_audit.get("passed") is True
            and set(transport_checks) == set(TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS)
            and all(
                transport_checks.get(name) is True
                for name in TYPED_TRANSPORT_AUDIT_REQUIRED_CHECKS
            )
            and transport_audit.get("node_shared_publish_errors") == []
            and transport_audit.get("topic_tagged_publish_failures") == []
            and transport_files_valid
            and digest_valid
        )
        typed_transport.update(
            typed_diag_json=str(typed_diag_path.resolve()),
            typed_diag_sha256=_sha256(typed_diag_path),
            raw_trace_jsonl=str(raw_trace_path.resolve()),
            raw_trace_sha256=_sha256(raw_trace_path),
            runner_script=str(typed_runner_path.resolve()),
            runner_sha256=_sha256(typed_runner_path),
            collector_script=str(typed_collector_path.resolve()),
            collector_sha256=_sha256(typed_collector_path),
            critical_source_manifest_json=str(critical_source_manifest_path.resolve()),
            critical_source_manifest_sha256=_sha256(critical_source_manifest_path),
            typed_cleaning_telemetry_source_sha256=typed_cleaning_telemetry_source_sha256,
        )
    passed = (
        bool(normal.get("passed"))
        and bool(full.get("passed"))
        and scenario_names_valid
        and normal_physics_checks_valid
        and full_interlock_checks_valid
        and side_brush_expanded_sdf_surface_valid
        and typed_transport_valid
        and runtime_binding_valid
        and preembedded_world_bindings_valid
    )
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_water_recovery_acceptance_v1",
        "status": (
            "FORMAL_WATER_RECOVERY_ACCEPTANCE_PASSED" if passed else "FAILED"
        ),
        "passed": passed,
        "checks": {
            "normal_recovery_passed": bool(normal.get("passed")),
            "full_tank_fail_closed_passed": bool(full.get("passed")),
            "scenario_names_valid": scenario_names_valid,
            "normal_physics_checks_valid": normal_physics_checks_valid,
            "full_interlock_checks_valid": full_interlock_checks_valid,
            "side_brush_expanded_sdf_surface_valid": side_brush_expanded_sdf_surface_valid,
            "typed_transport_evidence_valid": typed_transport_valid,
            "frozen_runtime_session_binding_valid": runtime_binding_valid,
            "preembedded_sensor_world_bindings_valid": preembedded_world_bindings_valid,
        },
        "summary": {
            "normal_initial_ground_volume_l": normal.get("metrics", {}).get(
                "initial_ground_volume_l"
            ),
            "normal_final_ground_volume_l": normal.get("metrics", {}).get(
                "final_ground_volume_l"
            ),
            "normal_ground_removed_l": normal.get("metrics", {}).get(
                "ground_removed_l"
            ),
            "normal_tank_mass_gain_kg": normal.get("metrics", {}).get(
                "tank_mass_gain_kg"
            ),
            "normal_dynamic_payload_applied_mass_kg": normal.get(
                "metrics", {}
            ).get("dynamic_payload_applied_mass_kg"),
            "normal_recovery_rate": normal.get("metrics", {}).get("recovery_rate"),
            "normal_mass_balance_error_fraction": normal.get("metrics", {}).get(
                "mass_balance_error_fraction"
            ),
            "normal_nozzle_covered_column_count": normal.get("metrics", {}).get(
                "nozzle_covered_column_count"
            ),
            "normal_ready_duty_cycle": normal.get("metrics", {}).get(
                "all_conditions_ready_duty_cycle"
            ),
            "full_tank_mass_kg": full.get("at_full", {}).get("tank_mass_kg"),
            "full_post_stop_ground_delta_l": full.get("metrics", {}).get(
                "post_full_ground_delta_l"
            ),
            "full_terminal_flow_l_min": full.get("terminal", {}).get(
                "flow_l_min"
            ),
            "full_remaining_ground_volume_l": full.get("metrics", {}).get(
                "remaining_ground_volume_l"
            ),
            "active_recovery_drain_requested_open": full.get(
                "active_recovery_drain_interlock_terminal", {}
            ).get("service_drain_requested_open"),
            "active_recovery_actual_drain_open": full.get(
                "active_recovery_drain_interlock_terminal", {}
            ).get("service_drain_open"),
            "service_drained_volume_l": full.get("metrics", {}).get(
                "service_drained_volume_l"
            ),
            "after_drain_tank_mass_kg": full.get(
                "after_service_drain", {}
            ).get("tank_mass_kg"),
            "after_drain_dynamic_payload_applied_mass_kg": full.get(
                "metrics", {}
            ).get("dynamic_payload_applied_mass_kg"),
        },
        "evidence": {
            "normal_json": str(normal_path.resolve()),
            "normal_sha256": _sha256(normal_path),
            "full_json": str(full_path.resolve()),
            "full_sha256": _sha256(full_path),
            "normal_side_brush_surface_json": str(normal_surface_path.resolve()),
            "normal_side_brush_surface_sha256": _sha256(normal_surface_path),
            "full_side_brush_surface_json": str(full_surface_path.resolve()),
            "full_side_brush_surface_sha256": _sha256(full_surface_path),
            "expanded_side_brush_sdf_sha256": normal_surface.get(
                "expanded_sdf_sha256"
            ),
            "typed_transport": typed_transport,
            "preembedded_sensor_world_bindings": {
                "normal": normal.get("preembedded_sensor_world_binding"),
                "full": full.get("preembedded_sensor_world_binding"),
            },
            "runtime_binding_json": (
                str(runtime_binding_path.resolve())
                if runtime_binding_path is not None and runtime_binding_path.is_file()
                else None
            ),
            "runtime_binding_sha256": (
                _sha256(runtime_binding_path)
                if runtime_binding_path is not None and runtime_binding_path.is_file()
                else None
            ),
        },
        "source_binding": source_binding,
        "acceptance_session_binding": acceptance_session_binding,
        "runtime_gate_binding": runtime_binding,
        "claim_boundary": (
            "Gazebo L1 sparse 2.5-D finite-water proxy with physical actuator, "
            "geometry, pump-flow, tank-mass and visual-state coupling; this does "
            "not claim CFD, spray, foam, slosh, or wet-surface material dynamics."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--normal", type=Path, required=True)
    parser.add_argument("--full", type=Path, required=True)
    parser.add_argument("--normal-side-brush-surface", type=Path, required=True)
    parser.add_argument("--full-side-brush-surface", type=Path, required=True)
    parser.add_argument("--typed-diag", type=Path, required=True)
    parser.add_argument("--typed-raw-trace", type=Path, required=True)
    parser.add_argument("--typed-runner", type=Path, required=True)
    parser.add_argument("--typed-collector", type=Path, required=True)
    parser.add_argument("--critical-source-manifest", type=Path, required=True)
    parser.add_argument("--typed-cleaning-telemetry-source-sha256", required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = combine(
        args.normal,
        args.full,
        args.normal_side_brush_surface,
        args.full_side_brush_surface,
        args.typed_diag,
        args.typed_raw_trace,
        args.typed_runner,
        args.typed_collector,
        args.critical_source_manifest,
        args.typed_cleaning_telemetry_source_sha256,
        args.runtime_binding,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
