#!/usr/bin/env python3
"""Fail-closed Windows-native serial STEP export for the 105 mapped components.

This command is deliberately dormant while any design-input contract is pending.
It first checks the contract and the small CadQuery STEP roundtrip *without
importing CadQuery*.  Only a future released contract can reach the one-at-a-
time build/re-import/export loop.  It never reads, converts, or derives a STEP
from an STL/mesh.  There is intentionally no preview mode in this revision.
"""

from __future__ import annotations

import argparse
import ast
import gc
import hashlib
import importlib.util
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping


CONTRACT_RELATIVE = Path("config/high_fidelity_vehicle/native_cadquery_serial_export_contract.json")
RELEASED_STATUS = "native_export_released"
MESH_STEP_MARKERS = ("FACETED_BREP", "TRIANGULATED_FACE_SET", "TESSELLATED")
BREP_STEP_MARKERS = ("MANIFOLD_SOLID_BREP", "ADVANCED_BREP_SHAPE_REPRESENTATION")
CHECKPOINT_NAME = "export-checkpoint.json"


class ExportBlocked(RuntimeError):
    """A release condition failed before loading the CAD kernel."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ExportBlocked(f"JSON root must be an object: {path}")
    return value


def load_contract(root: Path) -> dict[str, Any]:
    return load_json(root / CONTRACT_RELATIVE)


def require_work_directory(root: Path, output: Path) -> Path:
    work = (root / ".work").resolve()
    candidate = output.resolve()
    if candidate == work or work not in candidate.parents:
        raise ExportBlocked(f"output must stay below {work}: {candidate}")
    return candidate


def component_ids(batch: Mapping[str, Any], contract: Mapping[str, Any]) -> tuple[str, ...]:
    adapter = batch.get("adapter")
    if adapter == "fifth_part":
        return tuple(str(row["part_id"]) for row in contract["parts"])
    if adapter == "sixth_item":
        return tuple(str(row["id"]) for row in contract["items"])
    if adapter == "seventh_mapping":
        return tuple(str(row["manifest_part_id"]) for row in contract["part_mappings"])
    if adapter == "eighth_power_distribution":
        return (str(contract["part_id"]),)
    return ()


def _source_is_lazy_and_mesh_free(source: Path) -> None:
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [alias.name for alias in node.names]
            if any(name == "cadquery" or name.startswith("cadquery.") for name in names):
                raise ExportBlocked(f"top-level CadQuery import is forbidden: {source}")
    forbidden = ("importMesh", "import_mesh", "mesh_to_step")
    source_text = source.read_text(encoding="utf-8")
    if any(marker in source_text for marker in forbidden):
        raise ExportBlocked(f"mesh-derived export marker found in {source}")


def _has_open_pending_input(value: Any) -> bool:
    if isinstance(value, Mapping):
        for key, nested in value.items():
            if isinstance(key, str) and key.startswith("pending_") and nested:
                return True
            if key == "must_not_exist_yet" and nested is True:
                return True
            if _has_open_pending_input(nested):
                return True
    elif isinstance(value, list):
        return any(_has_open_pending_input(item) for item in value)
    return False


def _verify_source_manifest_bindings(
    source_path: Path,
    contract_path: Path,
    manifest_path: Path,
    source_relative: str,
    contract_relative: str,
    *,
    expected_status: str,
) -> dict[str, Any]:
    """Require the source manifest to bind the exact executable inputs by SHA."""

    manifest = load_json(manifest_path)
    if manifest.get("status") != expected_status:
        raise ExportBlocked(
            f"source manifest status is not {expected_status}: {manifest_path}"
        )
    source_binding = next(
        (
            item
            for item in manifest.get("source_files", [])
            if isinstance(item, Mapping) and item.get("path") == source_relative
        ),
        None,
    )
    contract_binding = next(
        (
            item
            for item in manifest.get("design_inputs", [])
            if isinstance(item, Mapping) and item.get("path") == contract_relative
        ),
        None,
    )
    if not isinstance(source_binding, Mapping) or source_binding.get(
        "sha256"
    ) != sha256(source_path):
        raise ExportBlocked(f"source SHA binding mismatch: {source_relative}")
    if not isinstance(contract_binding, Mapping) or contract_binding.get(
        "sha256"
    ) != sha256(contract_path):
        raise ExportBlocked(f"contract SHA binding mismatch: {contract_relative}")
    return manifest


def audit_serial_export_contract(
    root: Path, serial_contract: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """Audit the dormant serial route without importing CadQuery or source modules.

    Release authorization intentionally fails first while the design inputs are
    pending.  This independent audit still proves that the dormant route binds
    eight SHA-256 source/contract manifests and exactly 105 unique component
    identifiers.  It must remain useful before any release gate is opened.
    """

    resolved_root = root.resolve()
    errors: list[str] = []
    batches_report: list[dict[str, Any]] = []
    component_ids_seen: list[str] = []
    pending_batch_count = 0
    source_digest_bindings_valid = True
    try:
        contract = dict(
            load_contract(resolved_root)
            if serial_contract is None
            else serial_contract
        )
    except Exception as exc:
        return {
            "schema_version": 1,
            "report_id": "tzcup_native_cadquery_serial_export_contract_audit_v1",
            "status": "STATIC_SERIAL_EXPORT_CONTRACT_INVALID",
            "contract_structurally_valid": False,
            "formal_export_ready": False,
            "native_cad_delivery_accepted": False,
            "cadquery_imported": False,
            "source_modules_loaded": False,
            "errors": [f"contract_load_failed:{type(exc).__name__}:{exc}"],
            "blockers": ["SERIAL_EXPORT_CONTRACT_INVALID"],
        }

    expected_roles = (
        ("first", "provenance_only", 0, None),
        ("second", "provenance_only", 0, None),
        ("third", "provenance_only", 0, None),
        ("fourth", "provenance_only", 0, None),
        ("fifth_bodywork_per_part", "component_addressable", 47, "fifth_part"),
        ("sixth_cleaning_per_part", "component_addressable", 23, "sixth_item"),
        (
            "seventh_storage_service_per_part",
            "component_addressable",
            34,
            "seventh_mapping",
        ),
        (
            "eighth_power_distribution_single_part",
            "component_addressable",
            1,
            "eighth_power_distribution",
        ),
    )
    formal = contract.get("formal_export")
    if not isinstance(formal, Mapping):
        errors.append("formal_export_missing_or_invalid")
    else:
        if formal.get("execution") != "strictly_serial":
            errors.append("formal_export_not_strictly_serial")
        if formal.get("minimum_free_physical_memory_mib") != 4096:
            errors.append("formal_export_memory_gate_not_4096_mib")
        prohibited = set(formal.get("prohibited_methods", []))
        if not {
            "mesh_to_step_conversion",
            "mesh_import_as_native_brep_substitute",
            "faceted_or_tessellated_step_export",
            "WSL",
            "Docker",
            "Gazebo",
        }.issubset(prohibited):
            errors.append("formal_export_prohibited_methods_incomplete")
    preview = contract.get("preview")
    if not isinstance(preview, Mapping) or preview.get("implemented") is not False:
        errors.append("preview_boundary_not_fail_closed")
    if contract.get("status") != "design_input_pending_native_export":
        errors.append("serial_contract_status_not_pending_design_input")
    batches = contract.get("source_batches")
    if not isinstance(batches, list) or len(batches) != len(expected_roles):
        errors.append("source_batch_count_not_eight")
        batches = []

    for index, expected in enumerate(expected_roles):
        if index >= len(batches) or not isinstance(batches[index], Mapping):
            errors.append(f"source_batch_{index}_missing_or_invalid")
            continue
        batch = batches[index]
        batch_id, expected_role, expected_count, expected_adapter = expected
        row_errors: list[str] = []
        for key in ("source", "contract", "source_manifest"):
            relative = batch.get(key)
            if not isinstance(relative, str):
                row_errors.append(f"{key}_path_missing")
                continue
            candidate = (resolved_root / relative).resolve()
            if resolved_root != candidate and resolved_root not in candidate.parents:
                row_errors.append(f"{key}_path_escapes_repository")
            elif not candidate.is_file():
                row_errors.append(f"{key}_file_missing")
        if batch.get("batch_id") != batch_id:
            row_errors.append("batch_id_mismatch")
        if batch.get("role") != expected_role:
            row_errors.append("role_mismatch")
        if batch.get("component_export_count") != expected_count:
            row_errors.append("component_export_count_mismatch")
        if batch.get("adapter") != expected_adapter:
            row_errors.append("adapter_mismatch")
        if expected_role == "component_addressable" and batch.get(
            "requires_native_validator"
        ) is not True:
            row_errors.append("native_release_validator_not_required")

        actual_ids: tuple[str, ...] = ()
        source_path = resolved_root / str(batch.get("source", ""))
        contract_path = resolved_root / str(batch.get("contract", ""))
        manifest_path = resolved_root / str(batch.get("source_manifest", ""))
        if not row_errors:
            try:
                _source_is_lazy_and_mesh_free(source_path)
                batch_contract = load_json(contract_path)
                source_manifest = _verify_source_manifest_bindings(
                    source_path,
                    contract_path,
                    manifest_path,
                    str(batch["source"]),
                    str(batch["contract"]),
                    expected_status="design_input_pending_native_export",
                )
                actual_ids = component_ids(batch, batch_contract)
                if len(actual_ids) != expected_count or len(set(actual_ids)) != len(
                    actual_ids
                ):
                    row_errors.append("actual_component_ids_do_not_match_count")
                if batch_contract.get("status") != "design_input_pending_native_export":
                    row_errors.append("batch_contract_status_not_pending")
                else:
                    pending_batch_count += 1
                if source_manifest.get("status") != "design_input_pending_native_export":
                    row_errors.append("source_manifest_status_not_pending")
            except Exception as exc:
                if "SHA binding mismatch" in str(exc):
                    source_digest_bindings_valid = False
                row_errors.append(f"static_batch_audit_failed:{type(exc).__name__}:{exc}")
        component_ids_seen.extend(actual_ids)
        errors.extend(f"{batch_id}:{error}" for error in row_errors)
        batches_report.append(
            {
                "batch_id": batch_id,
                "role": expected_role,
                "component_count": len(actual_ids),
                "static_integrity_valid": not row_errors,
                "errors": row_errors,
            }
        )

    if len(component_ids_seen) != 105 or len(set(component_ids_seen)) != 105:
        errors.append("component_addressable_ids_not_exactly_105_unique")
    if contract.get("expected_source_batch_count") != 8:
        errors.append("declared_expected_source_batch_count_not_eight")
    if contract.get("expected_component_addressable_count") != 105:
        errors.append("declared_expected_component_count_not_105")
    valid = not errors
    return {
        "schema_version": 1,
        "report_id": "tzcup_native_cadquery_serial_export_contract_audit_v1",
        "status": (
            "STATIC_SERIAL_EXPORT_CONTRACT_VALID_NATIVE_EXPORT_BLOCKED"
            if valid
            else "STATIC_SERIAL_EXPORT_CONTRACT_INVALID"
        ),
        "contract_structurally_valid": valid,
        "source_batch_count": len(batches),
        "provenance_only_batch_count": sum(
            row.get("role") == "provenance_only" for row in batches_report
        ),
        "component_addressable_batch_count": sum(
            row.get("role") == "component_addressable" for row in batches_report
        ),
        "component_addressable_count": len(component_ids_seen),
        "component_ids_unique": len(set(component_ids_seen)) == 105,
        "pending_batch_contract_count": pending_batch_count,
        "source_digest_bindings_valid": source_digest_bindings_valid,
        "minimum_free_physical_memory_mib": 4096,
        "execution": "strictly_serial",
        "formal_export_ready": False,
        "native_cad_delivery_accepted": False,
        "cadquery_imported": False,
        "source_modules_loaded": False,
        "batches": batches_report,
        "errors": errors,
        "blockers": (
            [
                "SERIAL_EXPORT_CONTRACT_PENDING_RELEASE",
                "SOURCE_BATCH_CONTRACTS_PENDING_RELEASE",
                "CADQUERY_ROUNDTRIP_AND_MEMORY_PREFLIGHT_NOT_EXECUTED",
            ]
            if valid
            else ["SERIAL_EXPORT_CONTRACT_INVALID"]
        ),
    }


def require_windows_native_resources(root: Path, serial_contract: Mapping[str, Any]) -> None:
    """Apply the same >=4096 MiB gate for direct Python invocation."""

    preflight_path = Path(__file__).with_name("cadquery_windows_preflight.py")
    spec = importlib.util.spec_from_file_location("native_serial_windows_preflight", preflight_path)
    if spec is None or spec.loader is None:
        raise ExportBlocked("cannot load Windows-native CadQuery preflight")
    preflight = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preflight)
    report = preflight.build_report(root)
    expected = int(serial_contract["formal_export"]["minimum_free_physical_memory_mib"])
    if report["minimums"]["free_physical_memory_mib"] != expected:
        raise ExportBlocked("serial export and Windows preflight memory requirements disagree")
    if not report["bootstrap_permitted"]:
        raise ExportBlocked("Windows-native resource preflight is blocked; no CadQuery import is permitted")


def check_release_authorization(
    root: Path, serial_contract: Mapping[str, Any], roundtrip_report: Path | None = None
) -> list[dict[str, Any]]:
    """Validate every release prerequisite before any CadQuery import occurs."""

    if serial_contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("serial export contract is design-input pending; formal export is forbidden")
    if serial_contract.get("preview", {}).get("implemented"):
        raise ExportBlocked("preview must never share the formal export contract")
    if roundtrip_report is not None:
        require_work_directory(root, roundtrip_report.parent)
        report = load_json(roundtrip_report)
        if report.get("test_name") != serial_contract["formal_export"]["requires_roundtrip_report"] or report.get("outcome") != "passed":
            raise ExportBlocked("the required minimal CadQuery B-rep STEP roundtrip has not passed")

    batches = serial_contract.get("source_batches")
    if not isinstance(batches, list) or len(batches) != serial_contract.get("expected_source_batch_count"):
        raise ExportBlocked("serial contract must list exactly the expected source batches")
    source_records: list[dict[str, Any]] = []
    total = 0
    for batch in batches:
        if not isinstance(batch, dict):
            raise ExportBlocked("source batch must be an object")
        source = root / str(batch["source"])
        batch_contract_path = root / str(batch["contract"])
        manifest = root / str(batch["source_manifest"])
        if not source.is_file() or not batch_contract_path.is_file() or not manifest.is_file():
            raise ExportBlocked(f"missing source/contract/manifest for {batch.get('batch_id')}")
        _source_is_lazy_and_mesh_free(source)
        batch_contract = load_json(batch_contract_path)
        if batch_contract.get("status") != RELEASED_STATUS:
            raise ExportBlocked(f"{batch['batch_id']} contract is not released")
        if _has_open_pending_input(batch_contract):
            raise ExportBlocked(f"{batch['batch_id']} retains pending release inputs")
        _verify_source_manifest_bindings(
            source,
            batch_contract_path,
            manifest,
            str(batch["source"]),
            str(batch["contract"]),
            expected_status=RELEASED_STATUS,
        )
        expected = int(batch.get("component_export_count", 0))
        ids = component_ids(batch, batch_contract)
        if len(ids) != expected or len(set(ids)) != len(ids):
            raise ExportBlocked(f"{batch['batch_id']} component mapping does not match its export count")
        if batch.get("requires_native_validator"):
            module = _load_source_module(root, str(batch["source"]), str(batch["batch_id"]))
            validator = getattr(module, "validate_release_authorization", None)
            if not callable(validator):
                raise ExportBlocked(f"{batch['batch_id']} lacks its required source release validator")
            validator(batch_contract)
        total += len(ids)
        source_records.append({
            "batch_id": batch["batch_id"], "source": str(batch["source"]), "source_sha256": sha256(source),
            "contract": str(batch["contract"]), "contract_sha256": sha256(batch_contract_path),
            "source_manifest": str(batch["source_manifest"]), "source_manifest_sha256": sha256(manifest),
            "component_ids": list(ids), "adapter": batch.get("adapter"),
        })
    if total != serial_contract.get("expected_component_addressable_count"):
        raise ExportBlocked(f"expected {serial_contract.get('expected_component_addressable_count')} mapped components, found {total}")
    return source_records


def _load_source_module(root: Path, source_relative: str, batch_id: str) -> Any:
    path = root / source_relative
    spec = importlib.util.spec_from_file_location(f"native_serial_{batch_id}", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load source module {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _build_component(module: Any, adapter: str, cq: Any, batch_contract: Mapping[str, Any], component_id: str) -> Any:
    if adapter == "fifth_part":
        return module.build_design_input_part(cq, batch_contract, component_id)
    if adapter in {"sixth_item", "seventh_mapping"}:
        return module.build_design_input_shape(cq, batch_contract, component_id)
    if adapter == "eighth_power_distribution":
        return module.build_power_distribution_box(cq, batch_contract)
    raise RuntimeError(f"unsupported serial-export adapter: {adapter}")


def _step_header_ok(step_path: Path) -> bool:
    """Stream the complete STEP file so late tessellation markers cannot hide."""

    markers = ("ISO-10303-21", *BREP_STEP_MARKERS, *MESH_STEP_MARKERS)
    overlap = max(len(marker) for marker in markers) - 1
    carry = b""
    saw_iso = False
    saw_brep = False
    saw_mesh = False
    with step_path.open("rb") as stream:
        while True:
            chunk = stream.read(262_144)
            if not chunk:
                break
            upper = (carry + chunk).upper()
            saw_iso = saw_iso or b"ISO-10303-21" in upper
            saw_brep = saw_brep or any(marker.encode("ascii") in upper for marker in BREP_STEP_MARKERS)
            saw_mesh = saw_mesh or any(marker.encode("ascii") in upper for marker in MESH_STEP_MARKERS)
            carry = upper[-overlap:]
    return saw_iso and saw_brep and not saw_mesh


def _topology(shape: Any) -> dict[str, int]:
    return {name: len(getattr(shape, method)()) for name, method in {"solids": "Solids", "faces": "Faces", "edges": "Edges", "vertices": "Vertices"}.items()}


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rendered = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _source_records_sha256(sources: list[dict[str, Any]]) -> str:
    return hashlib.sha256(
        json.dumps(sources, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode(
            "utf-8"
        )
    ).hexdigest()


def _component_plan(sources: list[dict[str, Any]]) -> list[tuple[str, str]]:
    return [
        (str(source["batch_id"]), str(component_id))
        for source in sources
        for component_id in source["component_ids"]
    ]


def _component_step_relative(batch_id: str, component_id: str) -> str:
    return f"components/{batch_id}__{component_id}.step"


def _checkpoint_payload(
    sources: list[dict[str, Any]],
    completed: list[dict[str, Any]],
    *,
    state: str,
    error: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "tzcup_native_cadquery_serial_export_checkpoint",
        "state": state,
        "source_records_sha256": _source_records_sha256(sources),
        "expected_component_count": len(_component_plan(sources)),
        "completed": completed,
    }
    if error is not None:
        payload["last_error"] = dict(error)
    return payload


def _completed_checkpoint_rows(
    staging: Path, sources: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Read only a complete, hash-bound prefix of a retained failed export."""

    checkpoint_path = staging / CHECKPOINT_NAME
    try:
        checkpoint = load_json(checkpoint_path)
    except Exception as exc:
        raise ExportBlocked(f"cannot resume without a valid checkpoint: {exc}") from exc
    if (
        checkpoint.get("schema_version") != 1
        or checkpoint.get("kind")
        != "tzcup_native_cadquery_serial_export_checkpoint"
        or checkpoint.get("state") not in {"running", "failed"}
        or checkpoint.get("source_records_sha256") != _source_records_sha256(sources)
        or checkpoint.get("expected_component_count") != len(_component_plan(sources))
        or not isinstance(checkpoint.get("completed"), list)
    ):
        raise ExportBlocked("resume checkpoint does not bind the current released source records")
    rows: list[dict[str, Any]] = []
    plan = _component_plan(sources)
    for index, row in enumerate(checkpoint["completed"]):
        if not isinstance(row, dict) or index >= len(plan):
            raise ExportBlocked("resume checkpoint has an invalid completed-component row")
        batch_id, component_id = plan[index]
        expected_relative = _component_step_relative(batch_id, component_id)
        if (
            row.get("outcome") != "passed"
            or row.get("batch_id") != batch_id
            or row.get("component_id") != component_id
            or row.get("step") != expected_relative
            or not isinstance(row.get("step_sha256"), str)
            or not isinstance(row.get("topology"), dict)
        ):
            raise ExportBlocked("resume checkpoint is not the exact serial component prefix")
        step_path = staging / expected_relative
        try:
            step_path.resolve().relative_to(staging.resolve())
        except ValueError as exc:
            raise ExportBlocked("resume component STEP escapes staging") from exc
        if (
            step_path.is_symlink()
            or not step_path.is_file()
            or sha256(step_path) != row["step_sha256"]
            or not _step_header_ok(step_path)
        ):
            raise ExportBlocked(f"resume component STEP is missing, changed, linked, or invalid: {expected_relative}")
        rows.append(row)
    return rows


def _prepare_staging(
    output: Path, sources: list[dict[str, Any]], *, resume: bool
) -> tuple[Path, list[dict[str, Any]]]:
    """Create or authenticate the one retained incomplete export directory."""

    if output.exists() or output.is_symlink():
        raise ExportBlocked(f"refusing to overwrite prior export directory: {output}")
    staging = output.parent / f".{output.name}.incomplete"
    if staging.exists() or staging.is_symlink():
        if staging.is_symlink() or not staging.is_dir():
            raise ExportBlocked(f"incomplete export path is not a safe directory: {staging}")
        if not resume:
            raise ExportBlocked(f"unfinished export retained at {staging}; rerun with --resume")
        completed = _completed_checkpoint_rows(staging, sources)
        _write_json(
            staging / CHECKPOINT_NAME,
            _checkpoint_payload(sources, completed, state="running"),
        )
        return staging, completed
    if resume:
        raise ExportBlocked(f"cannot resume because no retained incomplete export exists: {staging}")
    staging.mkdir(parents=False)
    _write_json(
        staging / CHECKPOINT_NAME, _checkpoint_payload(sources, [], state="running")
    )
    return staging, []


def _add_reimported_component(
    assembly: Any, shapes: list[Any], batch_id: str, component_id: str
) -> None:
    for index, shape in enumerate(shapes):
        if not shape.isValid():
            raise RuntimeError("STEP re-import produced an invalid B-rep")
        suffix = "" if len(shapes) == 1 else f"__shape_{index}"
        assembly.add(shape, name=f"{batch_id}__{component_id}{suffix}")


def export_released_components(
    root: Path,
    output: Path,
    sources: list[dict[str, Any]],
    serial_contract: Mapping[str, Any],
    *,
    resume: bool = False,
) -> Path:
    """Build one component at a time and retain a hash-bound resume checkpoint."""

    output.parent.mkdir(parents=True, exist_ok=True)
    staging, receipt_rows = _prepare_staging(output, sources, resume=resume)
    completed = {(row["batch_id"], row["component_id"]) for row in receipt_rows}
    completed_rows = {
        (row["batch_id"], row["component_id"]): row for row in receipt_rows
    }
    logs = staging / "component-logs"
    try:
        require_windows_native_resources(root, serial_contract)
        import cadquery as cq  # guarded by check_release_authorization

        assembly = cq.Assembly(name="native_component_addressable_assembly")
        for source_record in sources:
            # The builder and temporary imported shapes for one source batch are
            # released before the next batch.  The assembly is the intentional
            # final aggregate and is guarded again at every batch boundary.
            require_windows_native_resources(root, serial_contract)
            for component_id in source_record["component_ids"]:
                key = (source_record["batch_id"], component_id)
                if key not in completed:
                    continue
                row = completed_rows[key]
                step_path = staging / str(row["step"])
                reimported = cq.importers.importStep(str(step_path)).vals()
                if not reimported:
                    raise RuntimeError(f"resume STEP has no B-rep payload: {step_path}")
                _add_reimported_component(
                    assembly,
                    reimported,
                    str(source_record["batch_id"]),
                    str(component_id),
                )
                del reimported
                gc.collect()
            module = _load_source_module(
                root, source_record["source"], source_record["batch_id"]
            )
            batch_contract = load_json(root / source_record["contract"])
            for component_id in source_record["component_ids"]:
                key = (source_record["batch_id"], component_id)
                if key in completed:
                    continue
                log_path = logs / f"{source_record['batch_id']}__{component_id}.json"
                step_path = staging / _component_step_relative(
                    str(source_record["batch_id"]), str(component_id)
                )
                try:
                    step_path.parent.mkdir(parents=True, exist_ok=True)
                    if step_path.is_symlink():
                        raise RuntimeError("refusing a symbolic-link component STEP path")
                    if step_path.exists():
                        step_path.unlink()
                    built = _build_component(
                        module,
                        str(source_record["adapter"]),
                        cq,
                        batch_contract,
                        component_id,
                    )
                    if isinstance(built, dict):
                        part_assembly = cq.Assembly(name=component_id)
                        topology = {"assembly_members": len(built)}
                        for member_name, member_shape in built.items():
                            member_shape = (
                                member_shape.val()
                                if hasattr(member_shape, "val")
                                else member_shape
                            )
                            if not member_shape.isValid():
                                raise RuntimeError(f"{member_name} B-rep is invalid")
                            part_assembly.add(member_shape, name=str(member_name))
                        part_assembly.save(str(step_path))
                        del part_assembly
                    else:
                        shape = built.val() if hasattr(built, "val") else built
                        if not shape.isValid():
                            raise RuntimeError("built B-rep is invalid")
                        topology = _topology(shape)
                        cq.exporters.export(shape, str(step_path))
                        del shape
                    del built
                    reimported = cq.importers.importStep(str(step_path)).vals()
                    if not reimported:
                        raise RuntimeError("STEP re-import produced no B-rep")
                    _add_reimported_component(
                        assembly,
                        reimported,
                        str(source_record["batch_id"]),
                        str(component_id),
                    )
                    if not _step_header_ok(step_path):
                        raise RuntimeError(
                            "STEP lacks non-faceted B-rep header evidence"
                        )
                    row = {
                        "outcome": "passed",
                        "batch_id": source_record["batch_id"],
                        "component_id": component_id,
                        "step": str(step_path.relative_to(staging)),
                        "step_sha256": sha256(step_path),
                        "topology": topology,
                    }
                    _write_json(log_path, row)
                    receipt_rows.append(row)
                    completed.add(key)
                    _write_json(
                        staging / CHECKPOINT_NAME,
                        _checkpoint_payload(
                            sources, receipt_rows, state="running"
                        ),
                    )
                    del reimported
                    gc.collect()
                except Exception as exc:
                    _write_json(
                        log_path,
                        {
                            "outcome": "failed",
                            "batch_id": source_record["batch_id"],
                            "component_id": component_id,
                            "error": {
                                "type": type(exc).__name__,
                                "message": str(exc),
                            },
                        },
                    )
                    raise RuntimeError(
                        f"stopped after retaining component log: {log_path}"
                    ) from exc
            del module, batch_contract
            gc.collect()
        assembly_path = staging / "native_component_addressable_assembly.step"
        assembly.save(str(assembly_path))
        if not _step_header_ok(assembly_path):
            raise RuntimeError(
                "component-addressable assembly lacks non-faceted B-rep header evidence"
            )
        assembled = cq.importers.importStep(str(assembly_path)).vals()
        if not assembled or any(not shape.isValid() for shape in assembled):
            raise RuntimeError(
                "component-addressable assembly STEP re-import is invalid"
            )
        receipt = {
            "schema_version": 1,
            "outcome": "passed",
            "native_readiness_accepted": False,
            "scope": (
                "Native geometric export receipt only; it does not close "
                "manufacturing, electrical, thermal, EMC, runtime or "
                "acceptance pending gates."
            ),
            "component_count": len(receipt_rows),
            "components": receipt_rows,
            "sources": sources,
            "assembly": {"step": assembly_path.name, "sha256": sha256(assembly_path)},
        }
        _write_json(staging / "sha256-receipt.json", receipt)
        _write_json(
            staging / CHECKPOINT_NAME,
            _checkpoint_payload(sources, receipt_rows, state="complete"),
        )
        staging.replace(output)
        return output / "sha256-receipt.json"
    except Exception as exc:
        _write_json(
            staging / CHECKPOINT_NAME,
            _checkpoint_payload(
                sources,
                receipt_rows,
                state="failed",
                error={"type": type(exc).__name__, "message": str(exc)},
            ),
        )
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--roundtrip-report", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--preflight-only", action="store_true", help="check formal gates without CadQuery, venv or exports")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="resume only the retained hash-bound .incomplete export directory",
    )
    args = parser.parse_args(argv)
    root = args.repo_root.resolve()
    try:
        if args.preflight_only:
            sources = check_release_authorization(root, load_contract(root))
            print(json.dumps({"outcome": "release_preflight_passed", "source_batches": len(sources)}, ensure_ascii=False))
            return 0
        if args.roundtrip_report is None or args.output_dir is None:
            parser.error("--roundtrip-report and --output-dir are required unless --preflight-only is used")
        output = require_work_directory(root, args.output_dir)
        contract = load_contract(root)
        require_windows_native_resources(root, contract)
        sources = check_release_authorization(root, contract, args.roundtrip_report.resolve())
        receipt = export_released_components(
            root, output, sources, contract, resume=args.resume
        )
        print(json.dumps({"outcome": "passed", "receipt": str(receipt)}, ensure_ascii=False))
        return 0
    except Exception as exc:
        print(json.dumps({"outcome": "blocked_or_failed", "error": {"type": type(exc).__name__, "message": str(exc)}}, ensure_ascii=False))
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
