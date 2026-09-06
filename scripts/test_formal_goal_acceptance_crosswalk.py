import ast
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CROSSWALK = ROOT / "config/high_fidelity_vehicle/formal_goal_acceptance_crosswalk.json"
CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
RUNNER = ROOT / "scripts/run_formal_final_acceptance.py"


def _runner_step_ids() -> set[str]:
    tree = ast.parse(RUNNER.read_text(encoding="utf-8"))
    result: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "StepSpec":
            continue
        if node.args and isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
            result.add(node.args[0].value)
    return result


def test_crosswalk_covers_current_goal_runner_and_gate_contract() -> None:
    crosswalk = json.loads(CROSSWALK.read_text(encoding="utf-8"))
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    requirements = crosswalk["requirements"]

    assert crosswalk["crosswalk_id"] == "tzcup_formal_goal_acceptance_crosswalk_v1"
    assert [item["id"] for item in requirements] == [f"A{i:02d}" for i in range(1, 22)]
    assert set(crosswalk["status_dimensions"]) == {
        "functional_system_complete",
        "formal_orchestration_complete",
        "competition_simulation_pass",
        "simulation_product_complete",
        "s100_compute_accepted",
        "product_integration_ready",
        "product_field_ready",
    }
    assert not any(crosswalk["status_dimensions"].values())

    mapped_steps = {step for item in requirements for step in item["step_ids"]}
    mapped_gates = {gate for item in requirements for gate in item["evidence_gates"]}
    assert mapped_steps == _runner_step_ids()
    assert mapped_gates == set(contract["evidence_gates"])
    assert all(item["state"] != "PASS" for item in requirements)


def test_crosswalk_keeps_external_and_historical_claims_fail_closed() -> None:
    payload = json.loads(CROSSWALK.read_text(encoding="utf-8"))
    by_id = {item["id"]: item for item in payload["requirements"]}

    assert by_id["A16"]["state"] == "HISTORICAL_COMPONENT_PASS_NOT_FORMAL_INTEGRATED"
    assert by_id["A17"]["state"] == "BLOCKED"
    assert by_id["A18"]["state"] == "BLOCKED_EXTERNAL_INPUT_AND_ARTIFACTS"
    assert by_id["A19"]["state"] == "CONTRACT_MAPPING_OPEN"
    assert by_id["A20"]["state"] == "CONTRACT_MAPPING_OPEN"
    assert by_id["A20"]["evidence_gates"] == []
    assert by_id["A21"]["state"] == "BLOCKED_EXTERNAL_INPUT_AND_ARTIFACTS"
    assert by_id["A21"]["evidence_gates"] == ["s100_live_runtime"]
