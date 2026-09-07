from __future__ import annotations

import json
import hashlib
from pathlib import Path

from auto15_competition_matrix import build_matrix
from validate_product_acceptance_contract import (
    ProductAcceptanceContractError,
    load_contract,
    validate_auto15_execution_evidence,
    validate_static_contract,
)


ROOT = Path(__file__).resolve().parents[1]


def test_matrix_is_complete_and_fail_closed() -> None:
    state = json.loads(
        (ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(
            encoding="utf-8"
        )
    )
    matrix = build_matrix(state)
    assert matrix["scenario_count"] == 18
    assert matrix["static_contract_pass"] is True
    assert matrix["required_unique_execution_count"] == 180
    assert len({item for row in matrix["scenarios"] for item in row["required_execution_ids"]}) == 180
    assert matrix["simulation_competition_matrix_pass"] is False
    assert matrix["runtime_execution_evidence_pass"] is False
    assert not any(matrix["product_runtime_states"].values())
    assert matrix["executed_integrated_missions"] == 0
    assert "AUTO-08" in matrix["blocking_dependencies"]
    assert all(
        row["integrated_execution_status"] == "NOT_EXECUTED"
        for row in matrix["scenarios"]
    )


def test_passing_components_are_not_promoted_to_integrated_results() -> None:
    state = json.loads(
        (ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(
            encoding="utf-8"
        )
    )
    matrix = build_matrix(state)
    app = next(row for row in matrix["scenarios"] if row["scenario_id"] == "app")
    assert app["component_evidence_status"] == "AVAILABLE"
    assert app["formal_mission_count"] == 0
    assert app["video_count"] == 0
    assert app["mcap_count"] == 0


def _receipt(path: Path) -> dict[str, str]:
    path.write_bytes(b"evidence")
    return {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def test_execution_evidence_requires_every_unique_receipt_group_and_artifact(tmp_path: Path) -> None:
    contract = load_contract()
    static = validate_static_contract(contract)
    executions = []
    for index, execution_id in enumerate(static["required_execution_ids"]):
        scenario_id, seed = execution_id.split(":seed-")
        executions.append(
            {
                "scenario_id": scenario_id,
                "seed": int(seed),
                "mission_group_id": f"group-{index % 30}",
                "status": "PASS",
                "video": _receipt(tmp_path / f"{index}.mp4"),
                "mcap": _receipt(tmp_path / f"{index}.mcap"),
            }
        )
    result = validate_auto15_execution_evidence(contract, {"executions": executions}, tmp_path)
    assert result["execution_evidence_pass"] is True
    assert result["retained_execution_count"] == 180
    assert result["mission_group_count"] == 30
    assert not any(result["product_runtime_states"].values())
    state = json.loads(
        (ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(
            encoding="utf-8"
        )
    )
    matrix = build_matrix(state, {"executions": executions}, tmp_path)
    assert matrix["runtime_execution_evidence_pass"] is True
    assert matrix["simulation_competition_matrix_pass"] is False
    assert not any(matrix["product_runtime_states"].values())

    executions[-1]["video"] = executions[0]["video"]
    try:
        validate_auto15_execution_evidence(contract, {"executions": executions}, tmp_path)
    except ProductAcceptanceContractError:
        pass
    else:
        raise AssertionError("shared video evidence must not satisfy two executions")

    executions[-1]["video"] = _receipt(tmp_path / "replacement.mp4")
    executions[-1]["mcap"] = {"path": "missing.mcap", "sha256": "0" * 64}
    try:
        validate_auto15_execution_evidence(contract, {"executions": executions}, tmp_path)
    except ProductAcceptanceContractError:
        pass
    else:
        raise AssertionError("missing constituent MCAP evidence must fail closed")
