from __future__ import annotations

import json
from pathlib import Path

import pytest

import publish_integrated_basic_functional_acceptance as publisher


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def evidence(tmp_path: Path, material: str = "PP") -> tuple[Path, Path]:
    run_id = "formal_unique_run"
    run_dir = tmp_path / run_id
    invocations = {}
    results = {}
    for name in publisher.SCENARIOS:
        result_path = run_dir / f"{name}.json"
        result = {"passed": True, "status": f"PASS_{name}"}
        if name == "manipulation":
            result.update(
                material=material,
                cube={"mass_kg": publisher.MATERIAL_MASS_KG[material]},
            )
        write_json(result_path, result)
        invocations[name] = {
            "result": str(result_path.resolve()),
            "result_sha256": publisher.sha256_file(result_path),
        }
        results[name] = result
    manifest = run_dir / "integrated_acceptance_manifest.json"
    write_json(
        manifest,
        {
            "schema_version": 1,
            "report_id": "tzcup_integrated_basic_functional_acceptance_v1",
            "status": "INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_PASSED",
            "passed": True,
            "source_bound": True,
            "run_id": run_id,
            "material_contract": {
                "material": material,
                "cube_edge_m": 0.03,
                "expected_mass_kg": publisher.MATERIAL_MASS_KG[material],
            },
            "scenario_invocations": invocations,
            "scenario_results": results,
        },
    )
    snapshot = tmp_path / "snapshot.json"
    write_json(
        snapshot,
        {
            "outputs": {
                "reports/engineering/formal_competition_vehicle.urdf": {
                    "sha256": "a" * 64
                }
            }
        },
    )
    return manifest, snapshot


def test_unique_manifest_publishes_fixed_session_bindable_summary(tmp_path: Path) -> None:
    manifest, snapshot = evidence(tmp_path, "aluminum")
    summary = publisher.build_summary(manifest, snapshot)
    assert summary["status"] == "INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_PASSED"
    assert summary["run_id"] == "formal_unique_run"
    assert summary["source_manifest"]["sha256"] == publisher.sha256_file(manifest)
    assert summary["source_binding"]["expanded_urdf_sha256"] == "a" * 64
    assert summary["material_contract"]["material"] == "aluminum"


def test_publisher_rejects_non_unique_or_tampered_run_evidence(tmp_path: Path) -> None:
    manifest, snapshot = evidence(tmp_path)
    wrong = tmp_path / "integrated_acceptance_manifest.json"
    wrong.write_bytes(manifest.read_bytes())
    with pytest.raises(publisher.PublishError, match="namespaced"):
        publisher.build_summary(wrong, snapshot)

    payload = json.loads(manifest.read_text(encoding="utf-8"))
    manipulation = Path(payload["scenario_invocations"]["manipulation"]["result"])
    manipulation.write_text("{}", encoding="utf-8")
    with pytest.raises(publisher.PublishError, match="changed after aggregation"):
        publisher.build_summary(manifest, snapshot)


def test_publisher_rejects_material_mass_drift(tmp_path: Path) -> None:
    manifest, snapshot = evidence(tmp_path, "paperboard")
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    payload["material_contract"]["expected_mass_kg"] = 0.03726
    write_json(manifest, payload)
    with pytest.raises(publisher.PublishError, match="mass contract"):
        publisher.build_summary(manifest, snapshot)


def test_runtime_binding_is_rechecked_and_embedded_in_canonical_summary(
    tmp_path: Path,
) -> None:
    manifest, snapshot = evidence(tmp_path)
    snapshot_payload = json.loads(snapshot.read_text(encoding="utf-8"))
    snapshot_payload["source_inventory_sha256"] = "b" * 64
    write_json(snapshot, snapshot_payload)
    identity = publisher.runtime_snapshot_identity(snapshot)
    session = tmp_path / "session.json"
    write_json(
        session,
        {
            "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "started_epoch_ns": 123,
            "snapshot": identity,
        },
    )
    closure = tmp_path / "closure.json"
    write_json(closure, {"closure_sha256": "c" * 64})
    install_root = tmp_path / "runtime/install"
    install_root.mkdir(parents=True)
    sidecar = tmp_path / "summary.json.runtime_binding.json"
    binding = {
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "snapshot": identity,
            "session_started_epoch_ns": 123,
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "session_manifest": str(session.resolve()),
            "session_manifest_sha256": publisher.sha256_file(session),
            "snapshot_current_source_verified": True,
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "manifest": str(closure.resolve()),
            "manifest_sha256": publisher.sha256_file(closure),
            "closure_sha256": "c" * 64,
            "runtime_install_root": str(install_root.resolve()),
            "symbolic_link_count": 0,
        },
    }
    write_json(sidecar, binding)
    verified = publisher.verify_runtime_binding(
        snapshot, session, closure, install_root, sidecar
    )
    manifest_payload = json.loads(manifest.read_text(encoding="utf-8"))
    manifest_payload["runtime_gate_binding"] = binding
    write_json(manifest, manifest_payload)
    summary = publisher.build_summary(manifest, snapshot, verified)
    assert summary["runtime_gate_binding"] == binding

    binding["acceptance_session_binding"]["snapshot"]["source_inventory_sha256"] = "d" * 64
    write_json(sidecar, binding)
    with pytest.raises(publisher.PublishError, match="current snapshot/session"):
        publisher.verify_runtime_binding(snapshot, session, closure, install_root, sidecar)
