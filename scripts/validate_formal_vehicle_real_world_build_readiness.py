#!/usr/bin/env python3
"""Validate the fail-closed real-world build-readiness declaration."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml

from generate_formal_vehicle_snapshot import SnapshotError, verify_snapshot


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/high_fidelity_vehicle/real_world_build_readiness.yaml"
NOT_READY = "NOT_READY_FOR_FABRICATION_OR_REAL_WORLD_ACCEPTANCE"
REQUIRED_BLOCKERS = {
    "geometry_assembly",
    "manufacturable_cad_drawings_tolerances",
    "structural_strength_stability",
    "mass_payload",
    "power_harness_thermal",
    "waterproofing_emc_safety",
    "control_sensor_calibration",
    "gazebo",
    "s100",
    "contest_closed_loop",
}
REQUIRED_EVIDENCE_STATES = {
    "last_expanded_urdf": "current_snapshot_output",
    "source_snapshot": "current_verified_snapshot",
    "component_register": "current_snapshot_output",
    "fov_occlusion": "current_static_analysis",
    "inertia_swept_volume": "current_static_analysis",
    "vehicle_runtime": "missing",
    "gazebo_closed_loop": "missing",
    "s100_live": "missing",
}
EXPECTED_SUBSYSTEM_MASSES_KG = {
    "a300_base_drive": 78.500000,
    "arm_gripper_wrist": 40.342576,
    "bodywork_service_safety": 12.280004,
    "cleaning_recovery": 11.125000,
    "storage_waste": 10.362001,
    "sensors_mast": 3.948001,
    "power_compute": 3.450001,
}
MASS_TOLERANCE_KG = 1.0e-6
SHA256_HEX_LENGTH = 64
FOV_PASSED = "PASSED"
INERTIA_PASSED = "PRODUCTION_ANCHORS_PASSED_WITH_RAW_JOINT_SPACE_EXCLUSION_REGIONS"


def _mapping(value: Any, label: str, errors: list[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        errors.append(f"{label} must be a mapping")
        return {}
    return value


def _number(value: Any, label: str, errors: list[str]) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append(f"{label} must be a finite number")
        return None
    result = float(value)
    if not math.isfinite(result):
        errors.append(f"{label} must be a finite number")
        return None
    return result


def _root_file(root: Path, relative: Any, label: str, errors: list[str]) -> Path | None:
    if not isinstance(relative, str) or not relative:
        errors.append(f"{label} must be a non-empty repository-relative path")
        return None
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError:
        errors.append(f"{label} must not escape the repository root")
        return None
    if not candidate.is_file():
        errors.append(f"{label} is missing")
        return None
    return candidate


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _bound_hash(path: Path | None, expected: Any, label: str, errors: list[str]) -> str | None:
    if not isinstance(expected, str) or len(expected) != SHA256_HEX_LENGTH:
        errors.append(f"{label} must be a SHA-256 hex digest")
        return None
    try:
        int(expected, 16)
    except ValueError:
        errors.append(f"{label} must be a SHA-256 hex digest")
        return None
    if path is None:
        return None
    actual = _sha256(path)
    if actual != expected:
        errors.append(f"{label} does not match the current file")
    return actual


def _json_payload(path: Path | None, label: str, errors: list[str]) -> dict[str, Any]:
    if path is None:
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        errors.append(f"{label} is not readable JSON: {exc}")
        return {}
    return _mapping(payload, label, errors)


def _verify_current_snapshot(
    binding: dict[str, Any], root: Path, errors: list[str]
) -> dict[str, Any] | None:
    manifest_path = _root_file(root, binding.get("path"), "current_snapshot.path", errors)
    _bound_hash(manifest_path, binding.get("sha256"), "current_snapshot.sha256", errors)
    if manifest_path is None:
        return None
    try:
        manifest = verify_snapshot(root, manifest_path=manifest_path)
    except SnapshotError as exc:
        errors.append(f"current source snapshot verification failed: {exc}")
        return None
    if manifest.get("source_inventory_sha256") != binding.get("source_inventory_sha256"):
        errors.append("current_snapshot.source_inventory_sha256 does not match the verified current snapshot")
    if manifest.get("output_inventory_sha256") != binding.get("output_inventory_sha256"):
        errors.append("current_snapshot.output_inventory_sha256 does not match the verified current snapshot")
    return manifest


def _verify_snapshot_output(
    evidence_name: str,
    row: dict[str, Any],
    manifest: dict[str, Any] | None,
    root: Path,
    errors: list[str],
) -> Path | None:
    path = _root_file(root, row.get("path"), f"evidence_paths.{evidence_name}.path", errors)
    actual_hash = _bound_hash(path, row.get("sha256"), f"evidence_paths.{evidence_name}.sha256", errors)
    if manifest is None or not isinstance(row.get("path"), str):
        return path
    outputs = manifest.get("outputs")
    entry = outputs.get(row["path"]) if isinstance(outputs, dict) else None
    if not isinstance(entry, dict):
        errors.append(f"evidence_paths.{evidence_name}.path is not an output of the verified snapshot")
    elif entry.get("sha256") != row.get("sha256") or entry.get("sha256") != actual_hash:
        errors.append(f"evidence_paths.{evidence_name} hash does not match the verified snapshot output")
    return path


def _verify_static_fov(
    row: dict[str, Any],
    expected_urdf_sha256: Any,
    expected_layout_sha256: Any,
    root: Path,
    errors: list[str],
) -> None:
    path = _root_file(root, row.get("path"), "evidence_paths.fov_occlusion.path", errors)
    _bound_hash(path, row.get("sha256"), "evidence_paths.fov_occlusion.sha256", errors)
    report = _json_payload(path, "fov_occlusion report", errors)
    if report.get("status") != FOV_PASSED or report.get("all_minimum_clear_fractions_passed") is not True:
        errors.append("fov_occlusion must record a passed static FOV audit")
    if row.get("urdf_sha256") != expected_urdf_sha256 or report.get("urdf_sha256") != expected_urdf_sha256:
        errors.append("fov_occlusion URDF hash is not bound to the current expanded URDF")
    if row.get("layout_sha256") != expected_layout_sha256 or report.get("layout_sha256") != expected_layout_sha256:
        errors.append("fov_occlusion layout hash is not bound to the current frozen layout")
    validator_path = _root_file(root, row.get("validator_path"), "evidence_paths.fov_occlusion.validator_path", errors)
    actual_validator_hash = _bound_hash(
        validator_path, row.get("validator_sha256"), "evidence_paths.fov_occlusion.validator_sha256", errors
    )
    if report.get("validator") != row.get("validator_path") or report.get("validator_sha256") != actual_validator_hash:
        errors.append("fov_occlusion validator provenance is not current")


def _verify_static_inertia(
    row: dict[str, Any],
    expected_urdf_sha256: Any,
    expected_layout_sha256: Any,
    root: Path,
    errors: list[str],
) -> None:
    path = _root_file(root, row.get("path"), "evidence_paths.inertia_swept_volume.path", errors)
    _bound_hash(path, row.get("sha256"), "evidence_paths.inertia_swept_volume.sha256", errors)
    report = _json_payload(path, "inertia_swept_volume report", errors)
    if report.get("status") != INERTIA_PASSED:
        errors.append("inertia_swept_volume must record the accepted static inertia/collision status")
    inputs = _mapping(report.get("inputs"), "inertia_swept_volume report.inputs", errors)
    if row.get("urdf_sha256") != expected_urdf_sha256 or inputs.get("expanded_urdf_sha256") != expected_urdf_sha256:
        errors.append("inertia_swept_volume URDF hash is not bound to the current expanded URDF")
    if row.get("layout_sha256") != expected_layout_sha256 or inputs.get("layout_sha256") != expected_layout_sha256:
        errors.append("inertia_swept_volume layout hash is not bound to the current frozen layout")
    scanner_path = _root_file(root, row.get("scanner_path"), "evidence_paths.inertia_swept_volume.scanner_path", errors)
    actual_scanner_hash = _bound_hash(
        scanner_path, row.get("scanner_sha256"), "evidence_paths.inertia_swept_volume.scanner_sha256", errors
    )
    if inputs.get("scanner") != row.get("scanner_path") or inputs.get("scanner_sha256") != actual_scanner_hash:
        errors.append("inertia_swept_volume scanner provenance is not current")


def validate(config_path: Path = DEFAULT_CONFIG, *, root: Path = ROOT) -> dict[str, Any]:
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        return {"valid": False, "errors": [f"cannot read readiness config: {exc}"]}

    errors: list[str] = []
    config = _mapping(payload, "readiness config", errors)
    if config.get("schema_version") != 2 or isinstance(config.get("schema_version"), bool):
        errors.append("schema_version must equal integer 2")
    if config.get("status") != NOT_READY:
        errors.append(f"status must equal {NOT_READY}")
    if type(config.get("ready")) is not bool or config.get("ready") is not False:
        errors.append("ready must be boolean false")

    artifact = _mapping(config.get("last_expanded_artifact"), "last_expanded_artifact", errors)
    if artifact.get("links") != 196 or isinstance(artifact.get("links"), bool):
        errors.append("last_expanded_artifact.links must equal integer 196")
    if artifact.get("joints") != 195 or isinstance(artifact.get("joints"), bool):
        errors.append("last_expanded_artifact.joints must equal integer 195")
    expanded_mass = _number(artifact.get("mass_kg"), "last_expanded_artifact.mass_kg", errors)
    if expanded_mass is not None and not math.isclose(expanded_mass, 160.007583, abs_tol=1.0e-6):
        errors.append("last_expanded_artifact.mass_kg does not match the declared artifact")
    if artifact.get("source_snapshot_state") != "current_verified_snapshot":
        errors.append("last expanded artifact must be marked current_verified_snapshot")

    snapshot_binding = _mapping(config.get("current_snapshot"), "current_snapshot", errors)
    snapshot = _verify_current_snapshot(snapshot_binding, root, errors)

    budget = _mapping(config.get("mass_budget"), "mass_budget", errors)
    design = _number(budget.get("a300_design_load_kg"), "mass_budget.a300_design_load_kg", errors)
    base = _number(budget.get("a300_base_mass_kg"), "mass_budget.a300_base_mass_kg", errors)
    dry = _number(budget.get("maximum_dry_payload_kg"), "mass_budget.maximum_dry_payload_kg", errors)
    wastewater = _number(budget.get("maximum_wastewater_payload_kg"), "mass_budget.maximum_wastewater_payload_kg", errors)
    reserve_each = _number(budget.get("dynamic_payload_reserve_each_kg"), "mass_budget.dynamic_payload_reserve_each_kg", errors)
    reserve_count = budget.get("dynamic_payload_reserve_state_count")
    if isinstance(reserve_count, bool) or not isinstance(reserve_count, int) or reserve_count != 2:
        errors.append("mass_budget.dynamic_payload_reserve_state_count must equal integer 2")
        reserve_count = None
    declared_physical_margin = _number(budget.get("expected_physical_replacement_margin_kg"), "mass_budget.expected_physical_replacement_margin_kg", errors)
    declared_conservative_margin = _number(budget.get("expected_conservative_remaining_margin_kg"), "mass_budget.expected_conservative_remaining_margin_kg", errors)
    computed_physical_margin = None
    computed_conservative_margin = None
    if None not in (design, expanded_mass, base, dry, wastewater):
        computed_conservative_margin = design - (expanded_mass - base + dry + wastewater)
        if declared_conservative_margin is None or not math.isclose(declared_conservative_margin, computed_conservative_margin, abs_tol=1.0e-6):
            errors.append("mass_budget.expected_conservative_remaining_margin_kg formula mismatch")
        if not math.isclose(computed_conservative_margin, 0.030417, abs_tol=1.0e-6):
            errors.append("conservative mass budget must recompute to 0.030417 kg")
    if None not in (design, expanded_mass, base, dry, wastewater, reserve_each, reserve_count):
        computed_physical_margin = design - (
            (expanded_mass - reserve_each * reserve_count) - base + dry + wastewater
        )
        if declared_physical_margin is None or not math.isclose(declared_physical_margin, computed_physical_margin, abs_tol=1.0e-6):
            errors.append("mass_budget.expected_physical_replacement_margin_kg formula mismatch")
        if not math.isclose(computed_physical_margin, 0.032417, abs_tol=1.0e-6):
            errors.append("physical replacement mass budget must recompute to 0.032417 kg")

    subsystem = _mapping(budget.get("subsystem_mass_breakdown_kg"), "mass_budget.subsystem_mass_breakdown_kg", errors)
    if set(subsystem) != set(EXPECTED_SUBSYSTEM_MASSES_KG):
        errors.append("mass_budget.subsystem_mass_breakdown_kg must contain exactly every audited subsystem")
    subsystem_total = 0.0
    subsystem_values_valid = True
    for name, expected in EXPECTED_SUBSYSTEM_MASSES_KG.items():
        value = _number(subsystem.get(name), f"mass_budget.subsystem_mass_breakdown_kg.{name}", errors)
        if value is None:
            subsystem_values_valid = False
            continue
        subsystem_total += value
        if not math.isclose(value, expected, abs_tol=MASS_TOLERANCE_KG):
            errors.append(f"mass_budget.subsystem_mass_breakdown_kg.{name} does not match the audited link aggregation")
    declared_subsystem_total = _number(budget.get("subsystem_mass_total_kg"), "mass_budget.subsystem_mass_total_kg", errors)
    if subsystem_values_valid and declared_subsystem_total is not None:
        if not math.isclose(declared_subsystem_total, subsystem_total, abs_tol=MASS_TOLERANCE_KG):
            errors.append("mass_budget.subsystem_mass_total_kg formula mismatch")
        if expanded_mass is not None and not math.isclose(declared_subsystem_total, expanded_mass, abs_tol=MASS_TOLERANCE_KG):
            errors.append("mass_budget.subsystem_mass_total_kg must equal last_expanded_artifact.mass_kg")

    margin_targets = _mapping(budget.get("manufacturing_margin_targets"), "mass_budget.manufacturing_margin_targets", errors)
    minimum_margin_target = _number(margin_targets.get("minimum_margin_kg"), "mass_budget.manufacturing_margin_targets.minimum_margin_kg", errors)
    recommended_margin_target = _number(margin_targets.get("recommended_margin_kg"), "mass_budget.manufacturing_margin_targets.recommended_margin_kg", errors)
    declared_minimum_reduction = _number(margin_targets.get("conservative_required_reduction_minimum_kg"), "mass_budget.manufacturing_margin_targets.conservative_required_reduction_minimum_kg", errors)
    declared_recommended_reduction = _number(margin_targets.get("conservative_required_reduction_recommended_kg"), "mass_budget.manufacturing_margin_targets.conservative_required_reduction_recommended_kg", errors)
    if minimum_margin_target is not None and not math.isclose(minimum_margin_target, 5.0, abs_tol=MASS_TOLERANCE_KG):
        errors.append("mass_budget.manufacturing_margin_targets.minimum_margin_kg must equal 5.0")
    if recommended_margin_target is not None and not math.isclose(recommended_margin_target, 10.0, abs_tol=MASS_TOLERANCE_KG):
        errors.append("mass_budget.manufacturing_margin_targets.recommended_margin_kg must equal 10.0")
    if margin_targets.get("mass_credit_policy") != "no_credit_until_physical_redesign_weighing_and_recomputed_inertia":
        errors.append("mass_budget.manufacturing_margin_targets.mass_credit_policy must preserve physical-evidence gating")
    if type(margin_targets.get("direct_urdf_mass_edit_is_not_a_mitigation")) is not bool or margin_targets.get("direct_urdf_mass_edit_is_not_a_mitigation") is not True:
        errors.append("mass_budget.manufacturing_margin_targets.direct_urdf_mass_edit_is_not_a_mitigation must be boolean true")
    computed_minimum_reduction = None
    computed_recommended_reduction = None
    if computed_conservative_margin is not None and minimum_margin_target is not None:
        computed_minimum_reduction = minimum_margin_target - computed_conservative_margin
        if declared_minimum_reduction is None or not math.isclose(declared_minimum_reduction, computed_minimum_reduction, abs_tol=MASS_TOLERANCE_KG):
            errors.append("mass_budget.manufacturing_margin_targets.conservative_required_reduction_minimum_kg formula mismatch")
    if computed_conservative_margin is not None and recommended_margin_target is not None:
        computed_recommended_reduction = recommended_margin_target - computed_conservative_margin
        if declared_recommended_reduction is None or not math.isclose(declared_recommended_reduction, computed_recommended_reduction, abs_tol=MASS_TOLERANCE_KG):
            errors.append("mass_budget.manufacturing_margin_targets.conservative_required_reduction_recommended_kg formula mismatch")

    mitigation = _mapping(budget.get("mitigation"), "mass_budget.mitigation", errors)
    if mitigation.get("status") != "design_required_not_credited":
        errors.append("mass_budget.mitigation.status must equal design_required_not_credited")
    candidates = mitigation.get("candidates")
    if not isinstance(candidates, list) or not candidates or not all(isinstance(candidate, str) and candidate for candidate in candidates):
        errors.append("mass_budget.mitigation.candidates must be a non-empty list of candidate identifiers")

    platform = _mapping(budget.get("platform_reselection"), "mass_budget.platform_reselection", errors)
    factor = _number(platform.get("design_payload_factor"), "mass_budget.platform_reselection.design_payload_factor", errors)
    rated_for_minimum = _number(platform.get("rated_payload_minimum_for_5kg_margin_kg"), "mass_budget.platform_reselection.rated_payload_minimum_for_5kg_margin_kg", errors)
    rated_for_recommended = _number(platform.get("rated_payload_minimum_for_10kg_margin_kg"), "mass_budget.platform_reselection.rated_payload_minimum_for_10kg_margin_kg", errors)
    if factor is not None and not math.isclose(factor, 0.9, abs_tol=MASS_TOLERANCE_KG):
        errors.append("mass_budget.platform_reselection.design_payload_factor must equal 0.9")
    if None not in (factor, minimum_margin_target, recommended_margin_target, design):
        conservative_payload = design - computed_conservative_margin if computed_conservative_margin is not None else None
        if conservative_payload is not None:
            expected_rated_for_minimum = (conservative_payload + minimum_margin_target) / factor
            expected_rated_for_recommended = (conservative_payload + recommended_margin_target) / factor
            if rated_for_minimum is None or not math.isclose(rated_for_minimum, expected_rated_for_minimum, abs_tol=0.01):
                errors.append("mass_budget.platform_reselection.rated_payload_minimum_for_5kg_margin_kg formula mismatch")
            if rated_for_recommended is None or not math.isclose(rated_for_recommended, expected_rated_for_recommended, abs_tol=0.01):
                errors.append("mass_budget.platform_reselection.rated_payload_minimum_for_10kg_margin_kg formula mismatch")
    if type(platform.get("dynamic_stability_must_pass_independently")) is not bool or platform.get("dynamic_stability_must_pass_independently") is not True:
        errors.append("mass_budget.platform_reselection.dynamic_stability_must_pass_independently must be boolean true")

    throughput = _mapping(config.get("throughput_budget"), "throughput_budget", errors)
    theoretical = _number(throughput.get("theoretical_cleaning_m2_h"), "throughput_budget.theoretical_cleaning_m2_h", errors)
    competition = _number(throughput.get("competition_minimum_m2_h"), "throughput_budget.competition_minimum_m2_h", errors)
    declared_throughput_margin = _number(throughput.get("expected_margin_m2_h"), "throughput_budget.expected_margin_m2_h", errors)
    computed_throughput_margin = None
    if None not in (theoretical, competition):
        computed_throughput_margin = theoretical - competition
        if declared_throughput_margin is None or not math.isclose(declared_throughput_margin, computed_throughput_margin, abs_tol=1.0e-6):
            errors.append("throughput_budget.expected_margin_m2_h formula mismatch")
        if not math.isclose(computed_throughput_margin, 64.0, abs_tol=1.0e-6):
            errors.append("throughput budget must recompute to 64 m2/h")

    blockers = _mapping(config.get("blocking_categories"), "blocking_categories", errors)
    if set(blockers) != REQUIRED_BLOCKERS:
        errors.append("blocking_categories must contain exactly every required blocker category")
    elif any(value != "blocked" for value in blockers.values()):
        errors.append("every blocking category must remain blocked")

    evidence = _mapping(config.get("evidence_paths"), "evidence_paths", errors)
    if set(evidence) != set(REQUIRED_EVIDENCE_STATES):
        errors.append("evidence_paths must contain exactly every required evidence key")
    else:
        for name, expected_state in REQUIRED_EVIDENCE_STATES.items():
            row = _mapping(evidence.get(name), f"evidence_paths.{name}", errors)
            relative = row.get("path")
            state = row.get("state")
            if not isinstance(relative, str) or not relative:
                errors.append(f"evidence_paths.{name}.path must be a non-empty string")
                continue
            if state != expected_state:
                errors.append(f"evidence_paths.{name}.state must equal {expected_state}")
                continue
            if expected_state == "missing":
                candidate = (root / relative).resolve()
                try:
                    candidate.relative_to(root.resolve())
                except ValueError:
                    errors.append(f"evidence_paths.{name}.path must not escape the repository root")
                else:
                    if candidate.is_file():
                        errors.append(f"evidence_paths.{name} is declared missing but exists")
                continue

            path = _root_file(root, relative, f"evidence_paths.{name}.path", errors)
            if path is None:
                continue

            if name == "source_snapshot":
                if relative != snapshot_binding.get("path") or row.get("sha256") != snapshot_binding.get("sha256"):
                    errors.append("source_snapshot evidence must equal the current_snapshot binding")
                _bound_hash(path, row.get("sha256"), "evidence_paths.source_snapshot.sha256", errors)
            elif name == "last_expanded_urdf":
                _verify_snapshot_output(name, row, snapshot, root, errors)
                if relative != artifact.get("path") or row.get("sha256") != artifact.get("sha256"):
                    errors.append("last_expanded_artifact must equal the current snapshot-expanded URDF evidence")
            elif name == "component_register":
                component_path = _verify_snapshot_output(name, row, snapshot, root, errors)
                component = _json_payload(component_path, "component_register report", errors)
                if component.get("urdf_sha256") != artifact.get("sha256"):
                    errors.append("component_register is not bound to the current expanded URDF")
            elif name == "fov_occlusion":
                layout_sha256 = snapshot_binding.get("source_inventory_sha256")
                if isinstance(snapshot, dict):
                    source_inventory = snapshot.get("source_inventory")
                    if isinstance(source_inventory, dict):
                        layout_entry = source_inventory.get("config/high_fidelity_vehicle/formal_vehicle_layout.yaml")
                        if isinstance(layout_entry, dict):
                            layout_sha256 = layout_entry.get("sha256")
                _verify_static_fov(row, artifact.get("sha256"), layout_sha256, root, errors)
            elif name == "inertia_swept_volume":
                layout_sha256 = snapshot_binding.get("source_inventory_sha256")
                if isinstance(snapshot, dict):
                    source_inventory = snapshot.get("source_inventory")
                    if isinstance(source_inventory, dict):
                        layout_entry = source_inventory.get("config/high_fidelity_vehicle/formal_vehicle_layout.yaml")
                        if isinstance(layout_entry, dict):
                            layout_sha256 = layout_entry.get("sha256")
                _verify_static_inertia(row, artifact.get("sha256"), layout_sha256, root, errors)

    return {
        "schema_version": 2,
        "status": NOT_READY,
        "ready": False,
        "valid": not errors,
        "errors": errors,
        "computed": {
            "physical_replacement_margin_kg": computed_physical_margin,
            "conservative_remaining_margin_kg": computed_conservative_margin,
            "subsystem_mass_total_kg": subsystem_total if subsystem_values_valid else None,
            "conservative_required_reduction_minimum_kg": computed_minimum_reduction,
            "conservative_required_reduction_recommended_kg": computed_recommended_reduction,
            "throughput_margin_m2_h": computed_throughput_margin,
            "current_snapshot_verified": snapshot is not None,
            "current_expanded_urdf_sha256": artifact.get("sha256"),
        },
        "claim_boundary": "Validation binds current static digital evidence to a verified frozen snapshot while keeping every manufacturing, hardware, runtime, and real-world gate fail-closed; it is not fabrication or real-world acceptance.",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(args.config, root=args.root)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    raise SystemExit(0 if result["valid"] else 2)


if __name__ == "__main__":
    main()
