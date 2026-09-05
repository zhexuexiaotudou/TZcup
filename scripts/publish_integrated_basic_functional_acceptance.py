#!/usr/bin/env python3
"""Validate one unique integrated run manifest and publish its fixed gate summary."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import RuntimeGateError, load_binding


SCENARIOS = ("mobility", "water_normal", "water_full", "manipulation")
MATERIAL_MASS_KG = {
    "paperboard": 0.0189,
    "PP": 0.0243,
    "PET": 0.03726,
    "aluminum": 0.0729,
}


class PublishError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PublishError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise PublishError(f"JSON root must be an object: {path}")
    return value


def snapshot_binding(path: Path) -> dict[str, str]:
    snapshot = json_object(path)
    output = snapshot.get("outputs", {}).get(
        "reports/engineering/formal_competition_vehicle.urdf"
    )
    if not isinstance(output, dict) or not isinstance(output.get("sha256"), str):
        raise PublishError("snapshot manifest has no expanded formal vehicle URDF hash")
    return {
        "snapshot_manifest": str(path.resolve()),
        "snapshot_manifest_sha256": sha256_file(path),
        "expanded_urdf_sha256": output["sha256"],
    }


def runtime_snapshot_identity(path: Path) -> dict[str, str]:
    snapshot = json_object(path)
    output = snapshot.get("outputs", {}).get(
        "reports/engineering/formal_competition_vehicle.urdf"
    )
    source_hash = snapshot.get("source_inventory_sha256")
    if (
        not isinstance(output, dict)
        or not isinstance(output.get("sha256"), str)
        or not isinstance(source_hash, str)
    ):
        raise PublishError("snapshot manifest is incomplete for runtime binding")
    return {
        "snapshot_manifest_sha256": sha256_file(path),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": output["sha256"],
    }


def verify_runtime_binding(
    snapshot_path: Path,
    session_path: Path,
    closure_path: Path,
    install_root: Path,
    sidecar: Path,
) -> dict[str, Any]:
    snapshot = runtime_snapshot_identity(snapshot_path)
    session = json_object(session_path)
    if (
        session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or session.get("snapshot") != snapshot
    ):
        raise PublishError("current formal session is not bound to the snapshot")
    try:
        binding = load_binding(sidecar)
    except RuntimeGateError as exc:
        raise PublishError(f"runtime binding is invalid: {exc}") from exc
    bound_session = binding["acceptance_session_binding"]
    if (
        bound_session.get("snapshot") != snapshot
        or bound_session.get("session_started_epoch_ns") != session.get("started_epoch_ns")
        or bound_session.get("session_manifest") != str(session_path.resolve())
        or bound_session.get("session_manifest_sha256") != sha256_file(session_path)
        or bound_session.get("snapshot_current_source_verified") is not True
    ):
        raise PublishError("runtime binding is not bound to the current snapshot/session")
    closure = json_object(closure_path)
    bound_closure = binding["runtime_closure_binding"]
    if (
        bound_closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or bound_closure.get("manifest") != str(closure_path.resolve())
        or bound_closure.get("manifest_sha256") != sha256_file(closure_path)
        or bound_closure.get("closure_sha256") != closure.get("closure_sha256")
        or bound_closure.get("runtime_install_root") != str(install_root.resolve())
        or bound_closure.get("symbolic_link_count") != 0
    ):
        raise PublishError("runtime binding is not bound to the current frozen closure")
    return binding


def build_summary(
    manifest_path: Path,
    snapshot_path: Path,
    runtime_gate_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    manifest = json_object(manifest_path)
    run_id = manifest.get("run_id")
    if manifest_path.name != "integrated_acceptance_manifest.json":
        raise PublishError("source must be the immutable per-run integrated acceptance manifest")
    if not isinstance(run_id, str) or not run_id or manifest_path.parent.name != run_id:
        raise PublishError("manifest path is not uniquely namespaced by its recorded run_id")
    if not (
        manifest.get("schema_version") == 1
        and manifest.get("report_id") == "tzcup_integrated_basic_functional_acceptance_v1"
        and manifest.get("status") == "INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_PASSED"
        and manifest.get("passed") is True
        and manifest.get("source_bound") is True
    ):
        raise PublishError("integrated run manifest is not a passing source-bound v1 report")

    invocations = manifest.get("scenario_invocations")
    results = manifest.get("scenario_results")
    if not isinstance(invocations, dict) or set(invocations) != set(SCENARIOS):
        raise PublishError("integrated run has an incomplete scenario invocation set")
    if not isinstance(results, dict) or set(results) != set(SCENARIOS):
        raise PublishError("integrated run has an incomplete scenario result set")
    for name in SCENARIOS:
        if results[name].get("passed") is not True:
            raise PublishError(f"scenario is not explicitly passing: {name}")
        result_path = Path(str(invocations[name].get("result", "")))
        if not result_path.is_file():
            raise PublishError(f"recorded scenario result is missing: {name}")
        if sha256_file(result_path) != invocations[name].get("result_sha256"):
            raise PublishError(f"recorded scenario result changed after aggregation: {name}")

    material = manifest.get("material_contract", {}).get("material")
    expected_mass = MATERIAL_MASS_KG.get(str(material))
    if expected_mass is None:
        raise PublishError(f"unsupported integrated material contract: {material}")
    if float(manifest["material_contract"].get("expected_mass_kg", -1.0)) != expected_mass:
        raise PublishError("integrated material mass contract is inconsistent")
    manipulation = results["manipulation"]
    if (
        manipulation.get("material") != material
        or abs(float(manipulation.get("cube", {}).get("mass_kg", -1.0)) - expected_mass)
        > 1.0e-8
    ):
        raise PublishError("manipulation result does not match the run material contract")
    if (
        runtime_gate_binding is not None
        and manifest.get("runtime_gate_binding") != runtime_gate_binding
    ):
        raise PublishError("integrated manifest runtime binding differs from the final sidecar")

    summary = {
        "schema_version": 1,
        "report_id": "tzcup_integrated_basic_functional_acceptance_summary_v1",
        "status": "INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_PASSED",
        "passed": True,
        "run_id": run_id,
        "source_manifest": {
            "path": str(manifest_path.resolve()),
            "sha256": sha256_file(manifest_path),
        },
        "source_binding": snapshot_binding(snapshot_path),
        "material_contract": manifest["material_contract"],
        "scenario_statuses": {
            name: results[name].get("status") for name in SCENARIOS
        },
        "claim_boundary": (
            "This fixed-path summary is a validated publication of exactly one unique, "
            "source-bound integrated run manifest. The immutable per-run manifest and its "
            "recorded scenario files remain the evidence authority."
        ),
    }
    if runtime_gate_binding is not None:
        summary["runtime_gate_binding"] = runtime_gate_binding
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--snapshot-manifest", type=Path, required=True)
    parser.add_argument("--session-status", type=Path, required=True)
    parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--runtime-install-root", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.manifest.resolve() == args.output.resolve():
            raise PublishError("fixed summary must not overwrite the immutable run manifest")
        if args.output.exists():
            raise PublishError("canonical summary already exists")
        expected_sidecar = args.output.with_name(args.output.name + ".runtime_binding.json")
        if args.runtime_binding.resolve() != expected_sidecar.resolve():
            raise PublishError("runtime binding must be the canonical summary sibling sidecar")
        binding = verify_runtime_binding(
            args.snapshot_manifest,
            args.session_status,
            args.runtime_closure,
            args.runtime_install_root,
            args.runtime_binding,
        )
        summary = build_summary(args.manifest, args.snapshot_manifest, binding)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + f".pending.{os.getpid()}")
        temporary.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        temporary.replace(args.output)
    except (PublishError, OSError, TypeError, ValueError) as exc:
        print(f"INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_PUBLISH_FAILED: {exc}")
        return 1
    print("INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_SUMMARY_PUBLISHED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
