#!/usr/bin/env python3
"""Fail-closed full-mission acceptance wrapper for the Day-1 competition run.

The wrapper consumes one bounded-coverage run directory and, optionally, one
demo-video directory.  It never promotes standalone coverage evidence into a
full-mission or official-efficiency claim.  Every section must be backed by
same-run machine evidence before the aggregate receipt can be PASS.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any, Iterable

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DEMO_PLAN = REPO_ROOT / "config/day1_full_mission_demo_plan.json"
RECEIPT_NAME = "full_mission_acceptance_receipt.json"
ARTIFACT_KIND = "day1_full_mission_acceptance_receipt"
SCHEMA_VERSION = 1
EFFICIENCY_THRESHOLD_M2_H = 3500.0
FULL_MISSION_BASIS = "full_mission_sim_seconds"
REQUIRED_STAGE_IDS = ("startup", "coverage", "return_home", "shutdown")
MCAP_REQUIRED_TOPICS = {
    "/clock",
    "/ground_truth/odom",
    "/odom",
    "/amcl_pose",
    "/localization/fused_pose",
    "/scan",
    "/cmd_vel_nav",
    "/cmd_vel_gate",
    "/base_controller/cmd_vel",
    "/joint_states",
    "/safety/status",
    "/brush_enabled",
    "/coverage/state",
    "/coverage/component_state",
    "/coverage/current_path",
    "/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json",
}
SOFTWARE_REQUIRED_ROLES = {"source_index", "build_receipt", "test_receipt"}
HARDWARE_REQUIRED_ROLES = {
    "component_register",
    "power_budget",
    "board_runtime",
}

EXPECTED_GATE_IDS = (
    "timeline_present",
    "timeline_schema",
    "timeline_terminal_completed",
    "timeline_full_mission_duration_basis",
    "timeline_required_stages_completed",
    "timeline_sim_clock_consistent",
    "coverage_report_present",
    "coverage_schema_v2",
    "coverage_success_flags",
    "coverage_trajectory_nonempty",
    "coverage_threshold_declared",
    "coverage_rate_meets_threshold",
    "coverage_result_valid",
    "brush_telemetry_present",
    "brush_safety_permit",
    "brush_ready_and_spinning",
    "brush_disabled_on_exit",
    "dirt_delta_positive",
    "dirt_delta_accounting_consistent",
    "safety_receipt_present",
    "coverage_safety_zero_violation",
    "emergency_stop_within_1s",
    "safety_evidence_hashed",
    "return_home_receipt_present",
    "return_home_completed",
    "return_home_timeline_consistent",
    "return_home_pose_within_tolerance",
    "return_home_brush_off",
    "final_state_present",
    "final_state_clean_and_idle",
    "efficiency_full_mission_duration",
    "efficiency_rtf_separated",
    "efficiency_threshold_met",
    "video_dir_present",
    "video_file_present",
    "video_probe_succeeded",
    "video_duration_5_to_10_min",
    "video_resolution_and_codec",
    "video_manifest_matches",
    "video_contains_operation_footage",
    "mcap_directory_present",
    "mcap_metadata_present",
    "mcap_payload_present",
    "mcap_duration_covers_mission",
    "mcap_required_topics",
    "hardware_software_manifest_present",
    "software_completeness_declared",
    "hardware_completeness_declared",
    "completeness_evidence_hashed",
    "demo_kit_plan_loaded",
    "demo_kit_files_written",
)


class AcceptanceError(RuntimeError):
    """Raised when the wrapper cannot produce a trustworthy receipt."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _finite_number(value: object) -> float | None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        return None
    return float(value)


def _integer(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _numeric_value(value: object) -> float | None:
    if isinstance(value, str):
        try:
            value = float(value)
        except ValueError:
            return None
    return _finite_number(value)


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _read_yaml(path: Path) -> dict[str, Any] | None:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return None
    return value if isinstance(value, dict) else None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative(path: Path, base: Path) -> str:
    try:
        return path.resolve().relative_to(base.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def _file_evidence(path: Path, base: Path) -> dict[str, Any]:
    evidence: dict[str, Any] = {
        "path": _relative(path, base),
        "exists": path.is_file(),
        "bytes": path.stat().st_size if path.is_file() else 0,
        "sha256": None,
    }
    if evidence["exists"]:
        evidence["sha256"] = _sha256(path)
    return evidence


def _add_gate(
    gates: list[dict[str, Any]],
    section: str,
    gate_id: str,
    passed: bool,
    message: str,
    evidence: dict[str, Any] | None = None,
) -> None:
    gates.append(
        {
            "id": gate_id,
            "section": section,
            "passed": bool(passed),
            "status": "PASS" if passed else "FAIL",
            "message": message,
            "evidence": evidence or {},
        }
    )


def _section_status(gates: Iterable[dict[str, Any]], section: str) -> str:
    rows = [row for row in gates if row["section"] == section]
    return "PASS" if rows and all(row["passed"] for row in rows) else "FAIL"


def _timeline_payload(
    run_dir: Path, gates: list[dict[str, Any]]
) -> tuple[dict[str, Any], dict[str, Any]]:
    path = run_dir / "mission_timeline.json"
    exists = path.is_file()
    payload = _read_json(path) if exists else None
    payload = payload or {}
    _add_gate(
        gates,
        "timeline",
        "timeline_present",
        exists and payload != {},
        "mission_timeline.json must exist and be a JSON object",
        _file_evidence(path, run_dir),
    )
    _add_gate(
        gates,
        "timeline",
        "timeline_schema",
        _integer(payload.get("schema_version")) == 1
        and isinstance(payload.get("mission_id"), str)
        and bool(payload.get("mission_id")),
        "timeline schema_version must be 1 and mission_id must be nonempty",
    )
    _add_gate(
        gates,
        "timeline",
        "timeline_terminal_completed",
        payload.get("terminal_state") == "COMPLETED",
        "the full mission must end in terminal_state COMPLETED",
        {"terminal_state": payload.get("terminal_state")},
    )

    sim_start = _finite_number(payload.get("sim_start_sec"))
    sim_end = _finite_number(payload.get("sim_end_sec"))
    sim_duration = _finite_number(payload.get("sim_duration_sec"))
    wall_duration = _finite_number(payload.get("wall_duration_sec"))
    rtf = _finite_number(payload.get("rtf"))
    basis = payload.get("duration_basis")
    basis_ok = (
        basis == FULL_MISSION_BASIS
        and sim_start is not None
        and sim_end is not None
        and sim_duration is not None
        and wall_duration is not None
        and sim_end > sim_start
        and sim_duration > 0.0
        and wall_duration > 0.0
        and math.isclose(sim_duration, sim_end - sim_start, rel_tol=0.0, abs_tol=1e-6)
    )
    _add_gate(
        gates,
        "timeline",
        "timeline_full_mission_duration_basis",
        basis_ok,
        "duration_basis must be full_mission_sim_seconds with a positive sim clock span",
        {
            "duration_basis": basis,
            "sim_start_sec": sim_start,
            "sim_end_sec": sim_end,
            "sim_duration_sec": sim_duration,
            "wall_duration_sec": wall_duration,
        },
    )

    stages = payload.get("stages")
    valid_stages: list[dict[str, Any]] = []
    stages_schema_ok = isinstance(stages, list) and bool(stages)
    stage_sum = 0.0
    previous_end: float | None = None
    if stages_schema_ok:
        for row in stages:
            if not isinstance(row, dict):
                stages_schema_ok = False
                break
            stage_id = row.get("id")
            status = row.get("status")
            start = _finite_number(row.get("start_sim_sec"))
            end = _finite_number(row.get("end_sim_sec"))
            if (
                not isinstance(stage_id, str)
                or not stage_id
                or status not in {"COMPLETED", "FAILED", "SKIPPED"}
                or start is None
                or end is None
                or end < start
            ):
                stages_schema_ok = False
                break
            if previous_end is not None and start + 1e-6 < previous_end:
                stages_schema_ok = False
                break
            previous_end = end
            stage_sum += end - start
            valid_stages.append(row)
    stage_ids = {row.get("id") for row in valid_stages}
    required_stages_ok = (
        stages_schema_ok
        and set(REQUIRED_STAGE_IDS) <= stage_ids
        and all(
            row.get("status") == "COMPLETED"
            for row in valid_stages
            if row.get("id") in REQUIRED_STAGE_IDS
        )
    )
    _add_gate(
        gates,
        "timeline",
        "timeline_required_stages_completed",
        required_stages_ok,
        "startup, coverage, return_home and shutdown stages must all complete",
        {
            "required_stage_ids": list(REQUIRED_STAGE_IDS),
            "observed_stage_ids": sorted(
                stage_id for stage_id in stage_ids if isinstance(stage_id, str)
            ),
        },
    )
    clock_ok = (
        basis_ok
        and stages_schema_ok
        and sim_duration is not None
        and math.isclose(stage_sum, sim_duration, rel_tol=0.0, abs_tol=1e-3)
        and valid_stages
        and math.isclose(
            float(valid_stages[0]["start_sim_sec"]),
            float(sim_start),
            rel_tol=0.0,
            abs_tol=1e-3,
        )
        and math.isclose(
            float(valid_stages[-1]["end_sim_sec"]),
            float(sim_end),
            rel_tol=0.0,
            abs_tol=1e-3,
        )
    )
    _add_gate(
        gates,
        "timeline",
        "timeline_sim_clock_consistent",
        clock_ok,
        "stage intervals must be ordered and cover the declared sim clock span",
        {"stage_duration_sum_sec": stage_sum},
    )
    stages_by_id = {
        row["id"]: row
        for row in valid_stages
        if isinstance(row.get("id"), str)
    }
    normalized = {
        "schema_version": payload.get("schema_version"),
        "mission_id": payload.get("mission_id"),
        "run_id": payload.get("run_id"),
        "duration_basis": basis,
        "sim_start_sec": sim_start,
        "sim_end_sec": sim_end,
        "sim_duration_sec": sim_duration,
        "wall_duration_sec": wall_duration,
        "rtf": rtf,
        "terminal_state": payload.get("terminal_state"),
        "stages": valid_stages,
        "stage_by_id": stages_by_id,
    }
    return normalized, {"present": exists}


def _coverage_payload(
    run_dir: Path, gates: list[dict[str, Any]]
) -> dict[str, Any]:
    report_path = run_dir / "coverage_report.json"
    trajectory_path = run_dir / "coverage_trajectory.csv"
    result_path = run_dir / "bounded_coverage_result.json"
    config_path = run_dir / "coverage_config.yaml"
    report = _read_json(report_path) if report_path.is_file() else None
    report = report or {}
    result = _read_json(result_path) if result_path.is_file() else None
    result = result or {}
    config = _read_yaml(config_path) if config_path.is_file() else None
    config = config or {}
    empirical = report.get("empirical_metrics")
    empirical = empirical if isinstance(empirical, dict) else {}
    planned = report.get("planned_metrics")
    planned = planned if isinstance(planned, dict) else {}

    _add_gate(
        gates,
        "coverage",
        "coverage_report_present",
        bool(report),
        "coverage_report.json must exist and be a JSON object",
        _file_evidence(report_path, run_dir),
    )
    _add_gate(
        gates,
        "coverage",
        "coverage_schema_v2",
        _integer(report.get("schema_version")) == 2,
        "coverage report schema_version must be 2",
        {"schema_version": report.get("schema_version")},
    )
    success_flags = (
        report.get("success") is True
        and report.get("full_execution_success") is True
        and report.get("coverage_quality_success") is True
        and report.get("safety_success") is True
        and report.get("localization_success") is True
        and report.get("ground_truth_used_for_control") is False
    )
    _add_gate(
        gates,
        "coverage",
        "coverage_success_flags",
        success_flags,
        "coverage success, execution, quality, safety and localization gates must all pass with GT kept out of control",
        {
            "success": report.get("success"),
            "full_execution_success": report.get("full_execution_success"),
            "coverage_quality_success": report.get("coverage_quality_success"),
            "safety_success": report.get("safety_success"),
            "localization_success": report.get("localization_success"),
            "ground_truth_used_for_control": report.get(
                "ground_truth_used_for_control"
            ),
        },
    )
    trajectory_row_count = 0
    try:
        with trajectory_path.open("r", newline="", encoding="utf-8") as stream:
            trajectory_row_count = sum(1 for _ in csv.DictReader(stream))
    except OSError:
        trajectory_row_count = 0
    _add_gate(
        gates,
        "coverage",
        "coverage_trajectory_nonempty",
        trajectory_row_count > 0,
        "coverage trajectory must contain at least one data row",
        {
            **_file_evidence(trajectory_path, run_dir),
            "row_count": trajectory_row_count,
        },
    )
    threshold = _finite_number(config.get("empirical_coverage_threshold"))
    _add_gate(
        gates,
        "coverage",
        "coverage_threshold_declared",
        threshold is not None and 0.0 < threshold <= 1.0,
        "coverage_config.yaml must declare a valid empirical_coverage_threshold",
        {
            "path": _relative(config_path, run_dir),
            "empirical_coverage_threshold": threshold,
        },
    )
    coverage_rate = _finite_number(empirical.get("coverage_rate"))
    _add_gate(
        gates,
        "coverage",
        "coverage_rate_meets_threshold",
        coverage_rate is not None
        and threshold is not None
        and coverage_rate >= threshold,
        "empirical coverage rate must meet the threshold copied from the run config",
        {
            "coverage_rate": coverage_rate,
            "threshold": threshold,
        },
    )
    result_valid = (
        _integer(result.get("schema_version")) == 1
        and result.get("artifact_kind")
        == "day1_bounded_saved_map_coverage_result"
        and result.get("valid_bounded_mission") is True
        and result.get("terminal_state") == "COMPLETED"
    )
    _add_gate(
        gates,
        "coverage",
        "coverage_result_valid",
        result_valid,
        "bounded_coverage_result.json must independently mark the coverage mission complete",
        _file_evidence(result_path, run_dir),
    )
    return {
        "report": report,
        "result": result,
        "coverage_rate": coverage_rate,
        "covered_area_m2": _finite_number(empirical.get("covered_area_m2")),
        "cleanable_area_m2": _finite_number(
            empirical.get("cleanable_area_m2")
        ),
        "actual_path_length_m": _finite_number(
            empirical.get("actual_path_length_m")
        ),
        "coverage_probe_duration_sec": _finite_number(
            empirical.get("actual_duration_sec")
        ),
        "brush_enabled_distance_m": _finite_number(
            empirical.get("brush_enabled_distance_m")
        ),
        "brush_state_transitions": _integer(
            empirical.get("brush_state_transitions")
        ),
        "planned_coverage_fraction": _finite_number(
            planned.get("coverage_rate")
        ),
        "trajectory_row_count": trajectory_row_count,
    }


def _brush_and_dirt_payloads(
    run_dir: Path, gates: list[dict[str, Any]]
) -> tuple[dict[str, Any], dict[str, Any]]:
    path = run_dir / "cleaning_bridge.json"
    cleaning = _read_json(path) if path.is_file() else None
    cleaning = cleaning or {}
    initial = cleaning.get("initial")
    initial = initial if isinstance(initial, dict) else {}
    terminal = cleaning.get("terminal")
    terminal = terminal if isinstance(terminal, dict) else {}

    _add_gate(
        gates,
        "brush",
        "brush_telemetry_present",
        bool(cleaning),
        "cleaning_bridge.json must contain same-run brush telemetry",
        _file_evidence(path, run_dir),
    )
    permit_ok = (
        cleaning.get("permit_observed") is True
        and cleaning.get("lift_requested") is True
    )
    _add_gate(
        gates,
        "brush",
        "brush_safety_permit",
        permit_ok,
        "cleaning permit and lift request must be observed",
        {
            "permit_observed": cleaning.get("permit_observed"),
            "lift_requested": cleaning.get("lift_requested"),
        },
    )
    ready_count = _integer(cleaning.get("all_three_ready_sample_count"))
    max_roller = _finite_number(cleaning.get("maximum_roller_velocity_rad_s"))
    _add_gate(
        gates,
        "brush",
        "brush_ready_and_spinning",
        ready_count is not None
        and ready_count > 0
        and max_roller is not None
        and max_roller > 0.0,
        "all three cleaning tools must be ready and the roller must spin",
        {
            "all_three_ready_sample_count": ready_count,
            "maximum_roller_velocity_rad_s": max_roller,
        },
    )
    disabled_ok = (
        cleaning.get("brush_disabled_on_exit") is True
        and cleaning.get("dirt_system_disabled_on_exit") is True
    )
    _add_gate(
        gates,
        "brush",
        "brush_disabled_on_exit",
        disabled_ok,
        "brush and dirt systems must be disabled on exit",
        {
            "brush_disabled_on_exit": cleaning.get("brush_disabled_on_exit"),
            "dirt_system_disabled_on_exit": cleaning.get(
                "dirt_system_disabled_on_exit"
            ),
        },
    )

    delta_cells = _integer(cleaning.get("cleaned_cell_delta"))
    delta_area = _finite_number(cleaning.get("cleaned_area_delta_m2"))
    _add_gate(
        gates,
        "dirt_delta",
        "dirt_delta_positive",
        delta_cells is not None
        and delta_cells > 0
        and delta_area is not None
        and delta_area > 0.0,
        "ground-dirt delta must contain both positive cell and area changes",
        {
            "cleaned_cell_delta": delta_cells,
            "cleaned_area_delta_m2": delta_area,
        },
    )
    initial_cells = _integer(initial.get("cell_count"))
    terminal_cells = _integer(terminal.get("cell_count"))
    initial_cleaned = _integer(initial.get("cleaned_cell_count"))
    terminal_cleaned = _integer(terminal.get("cleaned_cell_count"))
    accounting_ok = (
        initial_cells is not None
        and initial_cells > 0
        and terminal_cells == initial_cells
        and initial_cleaned is not None
        and terminal_cleaned is not None
        and delta_cells is not None
        and terminal_cleaned - initial_cleaned == delta_cells
    )
    _add_gate(
        gates,
        "dirt_delta",
        "dirt_delta_accounting_consistent",
        accounting_ok,
        "terminal minus initial cleaned cells must equal cleaned_cell_delta",
        {
            "initial_cell_count": initial_cells,
            "terminal_cell_count": terminal_cells,
            "initial_cleaned_cell_count": initial_cleaned,
            "terminal_cleaned_cell_count": terminal_cleaned,
            "cleaned_cell_delta": delta_cells,
        },
    )
    return (
        {
            "permit_observed": cleaning.get("permit_observed"),
            "lift_requested": cleaning.get("lift_requested"),
            "all_three_ready_sample_count": ready_count,
            "maximum_roller_velocity_rad_s": max_roller,
            "brush_disabled_on_exit": cleaning.get("brush_disabled_on_exit"),
            "dirt_system_disabled_on_exit": cleaning.get(
                "dirt_system_disabled_on_exit"
            ),
        },
        {
            "initial_cleaned_cell_count": initial_cleaned,
            "terminal_cleaned_cell_count": terminal_cleaned,
            "cleaned_cell_delta": delta_cells,
            "cleaned_area_delta_m2": delta_area,
            "initial_cell_count": initial_cells,
            "terminal_cell_count": terminal_cells,
            "terminal_cleaned_fraction": _finite_number(
                terminal.get("cleaned_fraction")
            ),
        },
    )


def _evidence_list_ok(
    rows: object,
    *,
    base_dir: Path,
    required_roles: set[str],
) -> tuple[bool, list[dict[str, Any]]]:
    if not isinstance(rows, list):
        return False, []
    normalized: list[dict[str, Any]] = []
    roles: set[str] = set()
    all_ok = True
    for row in rows:
        if not isinstance(row, dict):
            all_ok = False
            continue
        role = row.get("role")
        raw_path = row.get("path")
        declared_hash = row.get("sha256")
        if not isinstance(role, str) or not isinstance(raw_path, str):
            all_ok = False
            continue
        path = Path(raw_path)
        if not path.is_absolute():
            path = base_dir / path
        file_ok = (
            path.is_file()
            and isinstance(declared_hash, str)
            and len(declared_hash) == 64
            and _sha256(path) == declared_hash.lower()
        )
        roles.add(role)
        all_ok = all_ok and file_ok
        normalized.append(
            {
                "role": role,
                "path": _relative(path, base_dir),
                "exists": path.is_file(),
                "sha256": _sha256(path) if path.is_file() else None,
                "declared_sha256": declared_hash,
                "hash_match": file_ok,
            }
        )
    return all_ok and required_roles <= roles, normalized


def _safety_payload(
    run_dir: Path,
    coverage: dict[str, Any],
    gates: list[dict[str, Any]],
) -> dict[str, Any]:
    path = run_dir / "safety_receipt.json"
    safety = _read_json(path) if path.is_file() else None
    safety = safety or {}
    report = coverage.get("report", {})
    _add_gate(
        gates,
        "safety",
        "safety_receipt_present",
        bool(safety),
        "safety_receipt.json must exist and be a JSON object",
        _file_evidence(path, run_dir),
    )
    coverage_collision = _integer(report.get("collision_count"))
    coverage_keepout = _integer(report.get("keepout_violation_sample_count"))
    safety_collision = _integer(safety.get("collision_count"))
    safety_keepout = _integer(safety.get("keepout_violation_count"))
    zero_violation = (
        report.get("safety_success") is True
        and coverage_collision == 0
        and coverage_keepout == 0
        and safety_collision == 0
        and safety_keepout == 0
    )
    _add_gate(
        gates,
        "safety",
        "coverage_safety_zero_violation",
        zero_violation,
        "coverage and safety receipts must both report zero collisions and keepout violations",
        {
            "coverage_collision_count": coverage_collision,
            "coverage_keepout_violation_count": coverage_keepout,
            "safety_collision_count": safety_collision,
            "safety_keepout_violation_count": safety_keepout,
        },
    )
    emergency = safety.get("emergency_stop")
    emergency = emergency if isinstance(emergency, dict) else {}
    latency = _finite_number(emergency.get("latency_sec"))
    final_linear = _finite_number(emergency.get("final_linear_mps"))
    final_angular = _finite_number(emergency.get("final_angular_radps"))
    estop_ok = (
        emergency.get("exercised") is True
        and latency is not None
        and 0.0 <= latency <= 1.0
        and final_linear is not None
        and abs(final_linear) <= 1e-6
        and final_angular is not None
        and abs(final_angular) <= 1e-6
    )
    _add_gate(
        gates,
        "safety",
        "emergency_stop_within_1s",
        estop_ok,
        "emergency-stop latency must be <= 1 s and both final velocities must be zero",
        {
            "exercised": emergency.get("exercised"),
            "latency_sec": latency,
            "final_linear_mps": final_linear,
            "final_angular_radps": final_angular,
        },
    )
    hashed_ok, evidence = _evidence_list_ok(
        safety.get("evidence"),
        base_dir=run_dir,
        required_roles={"emergency_stop", "collision_monitor"},
    )
    _add_gate(
        gates,
        "safety",
        "safety_evidence_hashed",
        hashed_ok,
        "safety evidence must retain emergency-stop and collision-monitor files with matching SHA-256 hashes",
        {"evidence": evidence},
    )
    return {
        "collision_count": safety_collision,
        "keepout_violation_count": safety_keepout,
        "emergency_stop": {
            "exercised": emergency.get("exercised"),
            "latency_sec": latency,
            "final_linear_mps": final_linear,
            "final_angular_radps": final_angular,
        },
        "evidence": evidence,
    }


def _return_home_payload(
    run_dir: Path,
    timeline: dict[str, Any],
    gates: list[dict[str, Any]],
) -> dict[str, Any]:
    path = run_dir / "return_home_receipt.json"
    receipt = _read_json(path) if path.is_file() else None
    receipt = receipt or {}
    _add_gate(
        gates,
        "return_home",
        "return_home_receipt_present",
        bool(receipt),
        "return_home_receipt.json must exist and be a JSON object",
        _file_evidence(path, run_dir),
    )
    completed = (
        receipt.get("requested") is True
        and receipt.get("started") is True
        and receipt.get("completed") is True
    )
    _add_gate(
        gates,
        "return_home",
        "return_home_completed",
        completed,
        "return-home must be requested, started and completed",
        {
            "requested": receipt.get("requested"),
            "started": receipt.get("started"),
            "completed": receipt.get("completed"),
        },
    )
    started = _finite_number(receipt.get("started_sim_sec"))
    finished = _finite_number(receipt.get("completed_sim_sec"))
    stage = timeline.get("stage_by_id", {}).get("return_home")
    stage = stage if isinstance(stage, dict) else {}
    stage_start = _finite_number(stage.get("start_sim_sec"))
    stage_end = _finite_number(stage.get("end_sim_sec"))
    timeline_ok = (
        started is not None
        and finished is not None
        and finished >= started
        and stage_start is not None
        and stage_end is not None
        and started >= stage_start - 1e-3
        and finished <= stage_end + 1e-3
    )
    _add_gate(
        gates,
        "return_home",
        "return_home_timeline_consistent",
        timeline_ok,
        "return-home receipt interval must be contained in the return_home timeline stage",
        {
            "receipt_started_sim_sec": started,
            "receipt_completed_sim_sec": finished,
            "stage_start_sim_sec": stage_start,
            "stage_end_sim_sec": stage_end,
        },
    )
    position_error = _finite_number(receipt.get("position_error_m"))
    yaw_error = _finite_number(receipt.get("yaw_error_rad"))
    pose_ok = (
        position_error is not None
        and position_error <= 0.10
        and yaw_error is not None
        and yaw_error <= 0.05
    )
    _add_gate(
        gates,
        "return_home",
        "return_home_pose_within_tolerance",
        pose_ok,
        "return-home position and yaw errors must be within 0.10 m and 0.05 rad",
        {
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
        },
    )
    _add_gate(
        gates,
        "return_home",
        "return_home_brush_off",
        receipt.get("brush_disabled_during_return") is True,
        "brush must remain disabled during return-home",
        {
            "brush_disabled_during_return": receipt.get(
                "brush_disabled_during_return"
            )
        },
    )
    return {
        "requested": receipt.get("requested"),
        "started": receipt.get("started"),
        "completed": receipt.get("completed"),
        "started_sim_sec": started,
        "completed_sim_sec": finished,
        "position_error_m": position_error,
        "yaw_error_rad": yaw_error,
        "brush_disabled_during_return": receipt.get(
            "brush_disabled_during_return"
        ),
    }


def _final_state_payload(
    run_dir: Path, gates: list[dict[str, Any]]
) -> dict[str, Any]:
    path = run_dir / "final_state.json"
    state = _read_json(path) if path.is_file() else None
    state = state or {}
    _add_gate(
        gates,
        "final_state",
        "final_state_present",
        bool(state),
        "final_state.json must exist and be a JSON object",
        _file_evidence(path, run_dir),
    )
    clean_idle = (
        state.get("terminal_state") == "COMPLETED"
        and state.get("mode") == "IDLE"
        and state.get("motion_zero") is True
        and state.get("brush_enabled") is False
        and state.get("dirt_system_enabled") is False
        and state.get("emergency_stop_active") is False
    )
    _add_gate(
        gates,
        "final_state",
        "final_state_clean_and_idle",
        clean_idle,
        "final state must be COMPLETED, IDLE, motion-zero and all cleaning/safety actuators disengaged",
        state,
    )
    return {
        "terminal_state": state.get("terminal_state"),
        "mode": state.get("mode"),
        "motion_zero": state.get("motion_zero"),
        "brush_enabled": state.get("brush_enabled"),
        "dirt_system_enabled": state.get("dirt_system_enabled"),
        "emergency_stop_active": state.get("emergency_stop_active"),
        "timestamp_sim_sec": _finite_number(state.get("timestamp_sim_sec")),
    }


def _probe_video(path: Path, ffprobe_bin: str | None) -> dict[str, Any]:
    executable = ffprobe_bin or shutil.which("ffprobe")
    if not executable:
        raise AcceptanceError("ffprobe not found")
    command = [
        executable,
        "-v",
        "error",
        "-show_entries",
        "format=duration,size,format_name",
        "-show_entries",
        "stream=codec_type,codec_name,width,height",
        "-of",
        "json",
        str(path),
    ]
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if completed.returncode != 0:
        raise AcceptanceError(
            f"ffprobe failed with exit code {completed.returncode}: "
            f"{completed.stderr.strip()}"
        )
    payload = json.loads(completed.stdout)
    if not isinstance(payload, dict):
        raise AcceptanceError("ffprobe output is not a JSON object")
    return payload


def _video_payload(
    video_dir: Path | None,
    *,
    run_id: str,
    mission_id: str | None,
    min_seconds: float,
    max_seconds: float,
    min_width: int,
    min_height: int,
    ffprobe_bin: str | None,
    video_probe: dict[str, Any] | None,
    gates: list[dict[str, Any]],
) -> dict[str, Any]:
    _add_gate(
        gates,
        "video",
        "video_dir_present",
        video_dir is not None and video_dir.is_dir(),
        "a same-run demo video directory must be supplied",
        {"path": str(video_dir) if video_dir else None},
    )
    manifest_path = video_dir / "video_manifest.json" if video_dir else None
    manifest = (
        _read_json(manifest_path)
        if manifest_path is not None and manifest_path.is_file()
        else None
    )
    manifest = manifest or {}
    declared_name = manifest.get("file")
    candidates: list[Path] = []
    if video_dir is not None and video_dir.is_dir():
        if isinstance(declared_name, str) and (video_dir / declared_name).is_file():
            candidates = [video_dir / declared_name]
        else:
            candidates = sorted(
                path
                for path in video_dir.glob("*.mp4")
                if path.is_file()
            )
    video_path = candidates[0] if len(candidates) == 1 else None
    _add_gate(
        gates,
        "video",
        "video_file_present",
        video_path is not None,
        "exactly one MP4 video must be identifiable",
        {
            "candidates": [path.name for path in candidates],
            "declared_file": declared_name,
        },
    )
    probe: dict[str, Any] = {}
    probe_error: str | None = None
    if video_path is not None:
        try:
            probe = video_probe or _probe_video(video_path, ffprobe_bin)
        except (AcceptanceError, OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
            probe_error = str(exc)
    _add_gate(
        gates,
        "video",
        "video_probe_succeeded",
        bool(probe),
        "ffprobe must successfully inspect the supplied video",
        {"error": probe_error},
    )
    format_info = probe.get("format")
    format_info = format_info if isinstance(format_info, dict) else {}
    duration = _numeric_value(format_info.get("duration"))
    streams = probe.get("streams")
    streams = streams if isinstance(streams, list) else []
    video_stream = next(
        (
            row
            for row in streams
            if isinstance(row, dict) and row.get("codec_type") == "video"
        ),
        {},
    )
    width = _integer(video_stream.get("width"))
    height = _integer(video_stream.get("height"))
    codec_name = video_stream.get("codec_name")
    duration_ok = (
        duration is not None
        and min_seconds <= duration <= max_seconds
    )
    _add_gate(
        gates,
        "video",
        "video_duration_5_to_10_min",
        duration_ok,
        "video duration must be between the configured 5 and 10 minute bounds",
        {
            "duration_sec": duration,
            "minimum_sec": min_seconds,
            "maximum_sec": max_seconds,
        },
    )
    resolution_ok = (
        width is not None
        and width >= min_width
        and height is not None
        and height >= min_height
        and codec_name in {"h264", "hevc"}
    )
    _add_gate(
        gates,
        "video",
        "video_resolution_and_codec",
        resolution_ok,
        "video must use H.264/H.265 at or above the configured resolution",
        {
            "width": width,
            "height": height,
            "codec_name": codec_name,
        },
    )
    actual_hash = _sha256(video_path) if video_path and video_path.is_file() else None
    manifest_run_id = manifest.get("run_id")
    manifest_mission_id = manifest.get("mission_id")
    manifest_hash = manifest.get("file_sha256")
    manifest_duration = _finite_number(manifest.get("duration_sec"))
    manifest_width = _integer(manifest.get("width"))
    manifest_height = _integer(manifest.get("height"))
    manifest_codec = manifest.get("codec_name")
    manifest_ok = (
        _integer(manifest.get("schema_version")) == 1
        and isinstance(manifest_run_id, str)
        and manifest_run_id == run_id
        and isinstance(manifest_mission_id, str)
        and mission_id is not None
        and manifest_mission_id == mission_id
        and isinstance(manifest_hash, str)
        and actual_hash is not None
        and manifest_hash.lower() == actual_hash
        and duration is not None
        and manifest_duration is not None
        and math.isclose(duration, manifest_duration, rel_tol=1e-3, abs_tol=0.05)
        and width == manifest_width
        and height == manifest_height
        and codec_name == manifest_codec
    )
    _add_gate(
        gates,
        "video",
        "video_manifest_matches",
        manifest_ok,
        "video manifest must bind the file hash, run, mission, duration, resolution and codec to the probed media",
        {
            "manifest": _file_evidence(manifest_path, video_dir)
            if manifest_path is not None
            else {"exists": False},
            "actual_sha256": actual_hash,
            "declared_sha256": manifest_hash,
            "manifest_run_id": manifest_run_id,
            "expected_run_id": run_id,
            "manifest_mission_id": manifest_mission_id,
            "expected_mission_id": mission_id,
        },
    )
    footage_ok = (
        manifest.get("contains_operation_footage") is True
        and manifest.get("stitched") is False
    )
    _add_gate(
        gates,
        "video",
        "video_contains_operation_footage",
        footage_ok,
        "video must contain continuous operation footage and must not be a stitched placeholder",
        {
            "contains_operation_footage": manifest.get(
                "contains_operation_footage"
            ),
            "stitched": manifest.get("stitched"),
        },
    )
    return {
        "path": _relative(video_path, video_dir)
        if video_path is not None and video_dir is not None
        else None,
        "bytes": video_path.stat().st_size if video_path and video_path.is_file() else 0,
        "sha256": actual_hash,
        "duration_sec": duration,
        "width": width,
        "height": height,
        "codec_name": codec_name,
        "manifest": _relative(manifest_path, video_dir)
        if manifest_path is not None and video_dir is not None
        else None,
        "contains_operation_footage": manifest.get(
            "contains_operation_footage"
        ),
        "stitched": manifest.get("stitched"),
    }


def _mcap_payload(
    run_dir: Path,
    *,
    sim_duration: float | None,
    gates: list[dict[str, Any]],
) -> dict[str, Any]:
    bag_dir = run_dir / "bag"
    if not bag_dir.is_dir():
        candidates = sorted(
            path.parent
            for path in run_dir.rglob("metadata.yaml")
            if path.parent.is_dir()
        )
        bag_dir = candidates[0] if len(candidates) == 1 else Path()
    _add_gate(
        gates,
        "mcap",
        "mcap_directory_present",
        bool(str(bag_dir)) and bag_dir.is_dir(),
        "a ROS 2 bag directory must exist inside the same run",
        {"path": _relative(bag_dir, run_dir) if bag_dir.is_dir() else None},
    )
    metadata_path = bag_dir / "metadata.yaml" if bag_dir.is_dir() else Path()
    metadata = _read_yaml(metadata_path) if metadata_path.is_file() else None
    metadata = metadata or {}
    info = metadata.get("rosbag2_bagfile_information")
    info = info if isinstance(info, dict) else {}
    _add_gate(
        gates,
        "mcap",
        "mcap_metadata_present",
        bool(info),
        "bag/metadata.yaml must contain rosbag2_bagfile_information",
        _file_evidence(metadata_path, run_dir),
    )
    mcap_files = sorted(bag_dir.glob("*.mcap")) if bag_dir.is_dir() else []
    payloads: list[dict[str, Any]] = []
    payload_ok = bool(mcap_files)
    for path in mcap_files:
        row = _file_evidence(path, run_dir)
        magic = b""
        trailer = b""
        if path.is_file():
            with path.open("rb") as stream:
                magic = stream.read(8)
                if row["bytes"] >= 8:
                    stream.seek(-8, 2)
                    trailer = stream.read(8)
        row["mcap_magic"] = magic == b"\x89MCAP0\r\n"
        row["mcap_trailer"] = trailer == b"\x89MCAP0\r\n"
        payload_ok = payload_ok and row["bytes"] > 0 and row["mcap_magic"] and row["mcap_trailer"]
        payloads.append(row)
    _add_gate(
        gates,
        "mcap",
        "mcap_payload_present",
        payload_ok,
        "at least one nonempty MCAP payload with valid leading and trailing magic must exist",
        {"files": payloads},
    )
    duration_ns = _integer((info.get("duration") or {}).get("nanoseconds"))
    duration_sec = duration_ns / 1e9 if duration_ns is not None else None
    duration_ok = (
        duration_sec is not None
        and duration_sec > 0.0
        and sim_duration is not None
        and duration_sec >= 0.95 * sim_duration
    )
    _add_gate(
        gates,
        "mcap",
        "mcap_duration_covers_mission",
        duration_ok,
        "MCAP duration must cover at least 95 percent of the full-mission sim duration",
        {
            "mcap_duration_sec": duration_sec,
            "full_mission_sim_duration_sec": sim_duration,
        },
    )
    topic_counts: dict[str, int] = {}
    topic_rows = info.get("topics_with_message_count")
    if isinstance(topic_rows, list):
        for row in topic_rows:
            if not isinstance(row, dict):
                continue
            metadata_row = row.get("topic_metadata")
            metadata_row = metadata_row if isinstance(metadata_row, dict) else {}
            name = metadata_row.get("name")
            count = _integer(row.get("message_count"))
            if isinstance(name, str) and count is not None:
                topic_counts[name] = count
    missing_topics = sorted(
        topic for topic in MCAP_REQUIRED_TOPICS if topic_counts.get(topic, 0) <= 0
    )
    _add_gate(
        gates,
        "mcap",
        "mcap_required_topics",
        not missing_topics,
        "MCAP metadata must prove message coverage for every required mission topic",
        {
            "required_topics": sorted(MCAP_REQUIRED_TOPICS),
            "missing_topics": missing_topics,
            "topic_message_counts": topic_counts,
        },
    )
    return {
        "bag_path": _relative(bag_dir, run_dir) if bag_dir.is_dir() else None,
        "message_count": _integer(info.get("message_count")),
        "duration_sec": duration_sec,
        "topic_count": len(topic_counts),
        "required_topic_count": len(MCAP_REQUIRED_TOPICS),
        "missing_topics": missing_topics,
        "files": payloads,
    }


def _hardware_software_payload(
    run_dir: Path, gates: list[dict[str, Any]]
) -> dict[str, Any]:
    path = run_dir / "hardware_software_manifest.json"
    manifest = _read_json(path) if path.is_file() else None
    manifest = manifest or {}
    _add_gate(
        gates,
        "hardware_software_completeness",
        "hardware_software_manifest_present",
        bool(manifest),
        "hardware_software_manifest.json must exist and be a JSON object",
        _file_evidence(path, run_dir),
    )
    software = manifest.get("software")
    software = software if isinstance(software, dict) else {}
    hardware = manifest.get("hardware")
    hardware = hardware if isinstance(hardware, dict) else {}
    software_roles_ok, software_evidence = _evidence_list_ok(
        software.get("evidence"),
        base_dir=run_dir,
        required_roles=SOFTWARE_REQUIRED_ROLES,
    )
    software_ok = (
        software.get("status") == "COMPLETE"
        and isinstance(software.get("revision"), str)
        and bool(software.get("revision"))
        and software_roles_ok
    )
    _add_gate(
        gates,
        "hardware_software_completeness",
        "software_completeness_declared",
        software_ok,
        "software completeness requires source-index, build and test evidence with hashes",
        {
            "status": software.get("status"),
            "revision": software.get("revision"),
            "evidence": software_evidence,
        },
    )
    hardware_roles_ok, hardware_evidence = _evidence_list_ok(
        hardware.get("evidence"),
        base_dir=run_dir,
        required_roles=HARDWARE_REQUIRED_ROLES,
    )
    hardware_ok = (
        hardware.get("status") == "COMPLETE"
        and isinstance(hardware.get("platform"), str)
        and bool(hardware.get("platform"))
        and hardware_roles_ok
    )
    _add_gate(
        gates,
        "hardware_software_completeness",
        "hardware_completeness_declared",
        hardware_ok,
        "hardware completeness requires component-register, power-budget and board-runtime evidence with hashes",
        {
            "status": hardware.get("status"),
            "platform": hardware.get("platform"),
            "evidence": hardware_evidence,
        },
    )
    _add_gate(
        gates,
        "hardware_software_completeness",
        "completeness_evidence_hashed",
        software_roles_ok and hardware_roles_ok,
        "all completeness evidence files must exist and match declared SHA-256 hashes",
        {},
    )
    return {
        "software": {
            "status": software.get("status"),
            "revision": software.get("revision"),
            "evidence": software_evidence,
        },
        "hardware": {
            "status": hardware.get("status"),
            "platform": hardware.get("platform"),
            "evidence": hardware_evidence,
        },
        "claim_boundary": (
            "Completeness receipt packaging only; judge-facing hardware/software "
            "completeness remains subject to source evidence review."
        ),
    }


def _efficiency_payload(
    timeline: dict[str, Any],
    coverage: dict[str, Any],
    gates: list[dict[str, Any]],
) -> dict[str, Any]:
    basis = timeline.get("duration_basis")
    sim_duration = _finite_number(timeline.get("sim_duration_sec"))
    wall_duration = _finite_number(timeline.get("wall_duration_sec"))
    covered_area = _finite_number(coverage.get("covered_area_m2"))
    path_length = _finite_number(coverage.get("actual_path_length_m"))
    brush_distance = _finite_number(coverage.get("brush_enabled_distance_m"))
    coverage_stage = timeline.get("stage_by_id", {}).get("coverage")
    coverage_stage = coverage_stage if isinstance(coverage_stage, dict) else {}
    coverage_stage_duration = None
    if (
        _finite_number(coverage_stage.get("end_sim_sec")) is not None
        and _finite_number(coverage_stage.get("start_sim_sec")) is not None
    ):
        coverage_stage_duration = (
            float(coverage_stage["end_sim_sec"])
            - float(coverage_stage["start_sim_sec"])
        )
    duration_ok = (
        basis == FULL_MISSION_BASIS
        and sim_duration is not None
        and sim_duration > 0.0
        and covered_area is not None
        and covered_area > 0.0
    )
    efficiency = (
        round(covered_area / sim_duration * 3600.0, 6)
        if duration_ok and covered_area is not None and sim_duration is not None
        else None
    )
    coverage_stage_efficiency = (
        round(covered_area / coverage_stage_duration * 3600.0, 6)
        if duration_ok
        and covered_area is not None
        and coverage_stage_duration is not None
        and coverage_stage_duration > 0.0
        else None
    )
    _add_gate(
        gates,
        "efficiency",
        "efficiency_full_mission_duration",
        duration_ok,
        "effective efficiency denominator must be the full-mission sim duration",
        {
            "formula": "effective_cleaned_area_union_m2 / full_mission_sim_duration_sec * 3600",
            "duration_basis": basis,
            "full_mission_sim_duration_sec": sim_duration,
            "covered_area_m2": covered_area,
        },
    )
    declared_rtf = _finite_number(timeline.get("rtf"))
    computed_rtf = (
        sim_duration / wall_duration
        if sim_duration is not None
        and wall_duration is not None
        and wall_duration > 0.0
        else None
    )
    rtf_ok = (
        computed_rtf is not None
        and computed_rtf > 0.0
        and (
            declared_rtf is None
            or math.isclose(declared_rtf, computed_rtf, rel_tol=0.01, abs_tol=1e-6)
        )
    )
    _add_gate(
        gates,
        "efficiency",
        "efficiency_rtf_separated",
        rtf_ok,
        "wall duration and real-time factor must be reported separately and remain consistent",
        {
            "wall_duration_sec": wall_duration,
            "declared_rtf": declared_rtf,
            "computed_rtf": computed_rtf,
            "wall_clock_used_for_efficiency": False,
        },
    )
    threshold_ok = (
        efficiency is not None
        and efficiency >= EFFICIENCY_THRESHOLD_M2_H
    )
    _add_gate(
        gates,
        "efficiency",
        "efficiency_threshold_met",
        threshold_ok,
        "full-mission effective cleaning efficiency must be at least 3500 m2/h",
        {
            "effective_cleaning_efficiency_m2_h": efficiency,
            "threshold_m2_h": EFFICIENCY_THRESHOLD_M2_H,
        },
    )
    average_width = (
        covered_area / brush_distance
        if covered_area is not None
        and brush_distance is not None
        and brush_distance > 0.0
        else None
    )
    average_speed = (
        path_length / sim_duration
        if path_length is not None
        and sim_duration is not None
        and sim_duration > 0.0
        else None
    )
    return {
        "effective_cleaned_area_union_m2": covered_area,
        "full_mission_sim_duration_sec": sim_duration,
        "wall_duration_sec": wall_duration,
        "real_time_factor": computed_rtf,
        "effective_cleaning_efficiency_m2_h": efficiency,
        "coverage_stage_duration_sec": coverage_stage_duration,
        "coverage_stage_efficiency_m2_h": coverage_stage_efficiency,
        "average_effective_width_m": average_width,
        "average_path_speed_mps": average_speed,
        "threshold_m2_h": EFFICIENCY_THRESHOLD_M2_H,
        "threshold_met": threshold_ok,
        "duration_source": FULL_MISSION_BASIS,
        "wall_clock_used_for_efficiency": False,
        "ten_second_steady_window_used": False,
    }


def _load_demo_plan(
    plan_path: Path, gates: list[dict[str, Any]]
) -> dict[str, Any]:
    plan = _read_json(plan_path)
    plan = plan or {}
    shots = plan.get("shots")
    checklist = plan.get("recording_checklist")
    valid = (
        _integer(plan.get("schema_version")) == 1
        and plan.get("artifact_kind") == "day1_full_mission_demo_plan"
        and _finite_number(plan.get("target_duration_sec")) is not None
        and 300.0 <= float(plan["target_duration_sec"]) <= 600.0
        and isinstance(shots, list)
        and bool(shots)
        and isinstance(checklist, list)
        and bool(checklist)
    )
    if valid:
        previous_end = 0.0
        for shot in shots:
            if not isinstance(shot, dict):
                valid = False
                break
            start = _finite_number(shot.get("start_sec"))
            end = _finite_number(shot.get("end_sec"))
            if (
                start is None
                or end is None
                or start != previous_end
                or end < start
                or not isinstance(shot.get("narration"), str)
                or not shot.get("narration")
            ):
                valid = False
                break
            previous_end = end
        if valid and previous_end != float(plan["target_duration_sec"]):
            valid = False
    _add_gate(
        gates,
        "demo_kit",
        "demo_kit_plan_loaded",
        valid,
        "demo plan must cover 300-600 seconds with contiguous shots and a checklist",
        _file_evidence(plan_path, REPO_ROOT),
    )
    return plan if valid else {}


def _write_demo_kit(
    output_dir: Path,
    plan: dict[str, Any],
    gates: list[dict[str, Any]],
) -> dict[str, Any]:
    if not plan:
        _add_gate(
            gates,
            "demo_kit",
            "demo_kit_files_written",
            False,
            "demo kit cannot be written without a valid plan",
            {},
        )
        return {}
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        storyboard_json = output_dir / "demo_storyboard.json"
        storyboard_md = output_dir / "demo_storyboard.md"
        checklist_json = output_dir / "demo_recording_checklist.json"
        checklist_md = output_dir / "demo_recording_checklist.md"
        shot_index_csv = output_dir / "demo_shot_index.csv"
        storyboard_json.write_text(
            json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        checklist_json.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "artifact_kind": "day1_full_mission_demo_recording_checklist",
                    "items": plan["recording_checklist"],
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        lines = [
            f"# {plan['title']}",
            "",
            f"- Target duration: {plan['target_duration_sec']} s",
            "- All footage, numbers and terminal states must come from one run.",
            "- Do not substitute old runs, static slides or rendered-only animation.",
            "",
        ]
        for shot in plan["shots"]:
            lines.extend(
                [
                    f"## {shot['id']} | {shot['start_sec']}-{shot['end_sec']} s | {shot['title']}",
                    "",
                    shot["screen"],
                    "",
                    f"Narration: {shot['narration']}",
                    "",
                    f"Evidence: {', '.join(shot['evidence'])}",
                    "",
                    f"Acceptance: {shot['acceptance']}",
                    "",
                ]
            )
        storyboard_md.write_text("\n".join(lines), encoding="utf-8")
        checklist_lines = [
            "# Demo recording checklist",
            "",
        ]
        for item in plan["recording_checklist"]:
            checklist_lines.append(
                f"- [ ] {item['id']} {item['item']} Evidence: {item['evidence']}"
            )
        checklist_lines.append("")
        checklist_md.write_text("\n".join(checklist_lines), encoding="utf-8")
        with shot_index_csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=(
                    "id",
                    "start_sec",
                    "end_sec",
                    "title",
                    "screen",
                    "narration",
                    "evidence",
                    "acceptance",
                ),
            )
            writer.writeheader()
            for shot in plan["shots"]:
                writer.writerow(
                    {
                        **shot,
                        "evidence": "; ".join(shot["evidence"]),
                    }
                )
    except (OSError, KeyError, TypeError, ValueError) as exc:
        _add_gate(
            gates,
            "demo_kit",
            "demo_kit_files_written",
            False,
            f"failed to write demo kit: {exc}",
            {},
        )
        return {}
    files = {
        "demo_storyboard_json": "demo_storyboard.json",
        "demo_storyboard_markdown": "demo_storyboard.md",
        "demo_recording_checklist_json": "demo_recording_checklist.json",
        "demo_recording_checklist_markdown": "demo_recording_checklist.md",
        "demo_shot_index_csv": "demo_shot_index.csv",
    }
    _add_gate(
        gates,
        "demo_kit",
        "demo_kit_files_written",
        all((output_dir / name).is_file() for name in files.values()),
        "storyboard, checklist and shot-index files must be written",
        files,
    )
    return files


def _blocked_receipt(run_dir: Path, error: str) -> dict[str, Any]:
    now = _utc_now()
    return {
        "schema_version": SCHEMA_VERSION,
        "artifact_kind": ARTIFACT_KIND,
        "generated_at_utc": now,
        "fixture_only": False,
        "status": "FAIL",
        "machine_gate_pass": False,
        "claim_boundary": {
            "fixture_only": False,
            "official_recognition_claim": False,
            "old_run_reuse_allowed": False,
            "note": "Fail-closed wrapper error; no competition claim is made.",
        },
        "run": {"path": str(run_dir), "run_id": None, "mission_id": None},
        "timeline": {"status": "FAIL"},
        "efficiency": {"status": "FAIL"},
        "coverage": {"status": "FAIL"},
        "brush": {"status": "FAIL"},
        "dirt_delta": {"status": "FAIL"},
        "safety": {"status": "FAIL"},
        "return_home": {"status": "FAIL"},
        "final_state": {"status": "FAIL"},
        "video": {"status": "FAIL"},
        "mcap": {"status": "FAIL"},
        "hardware_software_completeness": {"status": "FAIL"},
        "demo_kit": {"status": "FAIL"},
        "gates": [],
        "blockers": [error],
    }


def assemble(
    *,
    run_dir: Path,
    video_dir: Path | None,
    output_dir: Path,
    demo_plan_path: Path = DEFAULT_DEMO_PLAN,
    min_video_seconds: float = 300.0,
    max_video_seconds: float = 600.0,
    min_video_width: int = 1280,
    min_video_height: int = 720,
    ffprobe_bin: str | None = None,
    video_probe: dict[str, Any] | None = None,
    fixture_only: bool = False,
) -> dict[str, Any]:
    run_dir = run_dir.resolve()
    video_dir = video_dir.resolve() if video_dir is not None else None
    output_dir = output_dir.resolve()
    gates: list[dict[str, Any]] = []

    timeline, timeline_meta = _timeline_payload(run_dir, gates)
    coverage = _coverage_payload(run_dir, gates)
    brush, dirt_delta = _brush_and_dirt_payloads(run_dir, gates)
    safety = _safety_payload(run_dir, coverage, gates)
    return_home = _return_home_payload(run_dir, timeline, gates)
    final_state = _final_state_payload(run_dir, gates)
    efficiency = _efficiency_payload(timeline, coverage, gates)

    mission_id = timeline.get("mission_id")
    if not isinstance(mission_id, str):
        coverage_mission_id = coverage.get("report", {}).get("mission_id")
        mission_id = coverage_mission_id if isinstance(coverage_mission_id, str) else None
    run_id = timeline.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        run_id = mission_id or run_dir.name

    video = _video_payload(
        video_dir,
        run_id=run_id,
        mission_id=mission_id,
        min_seconds=min_video_seconds,
        max_seconds=max_video_seconds,
        min_width=min_video_width,
        min_height=min_video_height,
        ffprobe_bin=ffprobe_bin,
        video_probe=video_probe,
        gates=gates,
    )
    mcap = _mcap_payload(
        run_dir,
        sim_duration=_finite_number(timeline.get("sim_duration_sec")),
        gates=gates,
    )
    completeness = _hardware_software_payload(run_dir, gates)
    demo_plan = _load_demo_plan(demo_plan_path, gates)
    demo_kit = _write_demo_kit(output_dir, demo_plan, gates)

    existing_gate_ids = {row["id"] for row in gates}
    for gate_id in EXPECTED_GATE_IDS:
        if gate_id not in existing_gate_ids:
            _add_gate(
                gates,
                "contract",
                gate_id,
                False,
                "expected fail-closed gate was not evaluated",
                {},
            )
    duplicate_ids = sorted(
        gate_id
        for gate_id in existing_gate_ids
        if sum(1 for row in gates if row["id"] == gate_id) > 1
    )
    if duplicate_ids:
        _add_gate(
            gates,
            "contract",
            "duplicate_gate_ids",
            False,
            "gate identifiers must be unique",
            {"duplicate_ids": duplicate_ids},
        )

    machine_gate_pass = bool(gates) and all(row["passed"] for row in gates)
    blockers = [
        row["id"]
        for row in gates
        if not row["passed"]
    ]
    sections = {
        "timeline": timeline,
        "efficiency": efficiency,
        "coverage": coverage,
        "brush": brush,
        "dirt_delta": dirt_delta,
        "safety": safety,
        "return_home": return_home,
        "final_state": final_state,
        "video": video,
        "mcap": mcap,
        "hardware_software_completeness": completeness,
        "demo_kit": {"files": demo_kit},
    }
    for section, payload in sections.items():
        payload["status"] = _section_status(gates, section)

    return {
        "schema_version": SCHEMA_VERSION,
        "artifact_kind": ARTIFACT_KIND,
        "generated_at_utc": _utc_now(),
        "fixture_only": fixture_only,
        "status": "PASS" if machine_gate_pass else "FAIL",
        "machine_gate_pass": machine_gate_pass,
        "claim_boundary": {
            "fixture_only": fixture_only,
            "official_recognition_claim": False,
            "old_run_reuse_allowed": False,
            "coverage_alone_is_full_mission": False,
            "wall_clock_is_efficiency_denominator": False,
            "note": (
                "This receipt packages same-run evidence for review. It does not "
                "replace judge scoring or convert a bounded coverage run into an "
                "official full-mission claim."
            ),
        },
        "run": {
            "path": str(run_dir),
            "run_id": run_id,
            "mission_id": mission_id,
            "timeline_file_present": timeline_meta.get("present", False),
        },
        "timeline": timeline,
        "efficiency": efficiency,
        "coverage": {
            key: value
            for key, value in coverage.items()
            if key not in {"report", "result"}
        },
        "coverage_report": coverage.get("report", {}),
        "bounded_coverage_result": coverage.get("result", {}),
        "brush": brush,
        "dirt_delta": dirt_delta,
        "safety": safety,
        "return_home": return_home,
        "final_state": final_state,
        "video": video,
        "mcap": mcap,
        "hardware_software_completeness": completeness,
        "demo_kit": {
            **sections["demo_kit"],
            "target_duration_sec": demo_plan.get("target_duration_sec"),
            "shot_count": len(demo_plan.get("shots", [])),
        },
        "gates": gates,
        "blockers": blockers,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--video-dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--demo-plan", type=Path, default=DEFAULT_DEMO_PLAN)
    parser.add_argument("--ffprobe", default=None)
    parser.add_argument("--min-video-seconds", type=float, default=300.0)
    parser.add_argument("--max-video-seconds", type=float, default=600.0)
    parser.add_argument("--min-video-width", type=int, default=1280)
    parser.add_argument("--min-video-height", type=int, default=720)
    args = parser.parse_args(argv)

    if not args.run_dir.is_dir():
        print(f"run directory does not exist: {args.run_dir}", file=sys.stderr)
        return 2
    args.output_dir.mkdir(parents=True, exist_ok=True)
    receipt_path = args.output_dir / RECEIPT_NAME
    if receipt_path.exists():
        print(f"refusing to overwrite existing receipt: {receipt_path}", file=sys.stderr)
        return 2
    try:
        receipt = assemble(
            run_dir=args.run_dir,
            video_dir=args.video_dir,
            output_dir=args.output_dir,
            demo_plan_path=args.demo_plan,
            min_video_seconds=args.min_video_seconds,
            max_video_seconds=args.max_video_seconds,
            min_video_width=args.min_video_width,
            min_video_height=args.min_video_height,
            ffprobe_bin=args.ffprobe,
        )
    except Exception as exc:  # pragma: no cover - defensive CLI boundary
        receipt = _blocked_receipt(args.run_dir, f"{type(exc).__name__}: {exc}")
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(str(receipt_path))
    return 0 if receipt.get("machine_gate_pass") is True else 2


if __name__ == "__main__":
    raise SystemExit(main())
