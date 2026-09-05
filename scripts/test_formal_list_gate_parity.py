from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import pytest
import yaml

import run_formal_final_acceptance as orchestration
from formal_acceptance_session import finalize, start
from validate_formal_functional_acceptance_contract import audit


def _snapshot(root: Path) -> Path:
    urdf = root / "reports/engineering/formal_competition_vehicle.urdf"
    urdf.parent.mkdir(parents=True, exist_ok=True)
    urdf.write_text('<robot name="parity"/>\n', encoding="utf-8")
    snapshot = root / "reports/engineering/formal_vehicle_snapshot_manifest.json"
    snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "parity-source",
                "source_inventory": {},
                "outputs": {
                    str(urdf.relative_to(root)).replace("\\", "/"): {
                        "sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
                        "size_bytes": urdf.stat().st_size,
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    return snapshot


def _contract(root: Path) -> Path:
    contract = {
        "acceptance_session": {
            "path": "artifacts/session.json",
            "snapshot_manifest": "reports/engineering/formal_vehicle_snapshot_manifest.json",
            "accepted_statuses": ["FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"],
        },
        "functional_positions": {"trajectory": ["trajectory_gate"]},
        "mission_level_gates": ["trajectory_gate"],
        "evidence_gates": {
            "trajectory_gate": {
                "path": "artifacts/trajectory.json",
                "success_statuses": ["PASS"],
                "session_bound": True,
                "required_list_item_values": {
                    "episodes": {"formal_success": True}
                },
                "required_list_item_minimums": {
                    "episodes": {"observed_fraction": 0.95}
                },
                "required_list_item_maximums": {
                    "episodes": {"task_to_full_coverage_ratio": 1.0}
                },
            }
        },
    }
    target = root / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    return target


def _context(root: Path) -> orchestration.Context:
    return orchestration.Context(
        root=root,
        runtime_ws=root / ".work/runtime",
        integrated_build_manifest=root / ".work/build_manifest.json",
        perception_artifacts=root / ".work/perception",
        onnx_pythonpath=root / ".work/onnx",
        run_root=root / ".work/run",
        base_domain=90,
        episode_count=30,
        session=root / "artifacts/session.json",
        snapshot=root / "reports/engineering/formal_vehicle_snapshot_manifest.json",
    )


@pytest.mark.parametrize(
    ("tampered_field", "tampered_value"),
    [
        ("formal_success", 1),
        ("observed_fraction", float("inf")),
        ("task_to_full_coverage_ratio", float("-inf")),
    ],
)
def test_list_item_contract_rejects_invalid_values_in_all_consumers(
    tmp_path: Path, tampered_field: str, tampered_value: object
) -> None:
    snapshot = _snapshot(tmp_path)
    contract_path = _contract(tmp_path)
    register_path = tmp_path / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
    register_path.write_text(
        yaml.safe_dump({"functional_positions": [{"id": "trajectory"}]}),
        encoding="utf-8",
    )
    session = tmp_path / "artifacts/session.json"
    start(snapshot, session)
    time.sleep(0.002)
    evidence = tmp_path / "artifacts/trajectory.json"
    episode = {
        "formal_success": True,
        "observed_fraction": 0.96,
        "task_to_full_coverage_ratio": 0.99,
    }
    episode[tampered_field] = tampered_value
    evidence.write_text(
        json.dumps({"status": "PASS", "episodes": [episode]}),
        encoding="utf-8",
    )

    sealed = finalize(contract_path, snapshot, session, tmp_path)
    assert sealed["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert "list item value mismatch" in sealed["failures"]["trajectory_gate"]

    aggregate = audit(contract_path, register_path, root=tmp_path)
    assert aggregate["gate_results"]["trajectory_gate"]["state"] == "failed"
    assert "evidence list item values do not match" in aggregate["gate_results"][
        "trajectory_gate"
    ]["error"]

    with pytest.raises(orchestration.OrchestrationError, match="list-item mismatch"):
        orchestration._validate_gate(_context(tmp_path), "trajectory_gate", 0)
