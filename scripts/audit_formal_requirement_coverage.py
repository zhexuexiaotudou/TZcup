#!/usr/bin/env python3
"""Generate the fail-closed formal requirement coverage and runtime-gap register.

This Windows/Python-only audit reads declared source evidence and the frozen
formal acceptance contract.  It never launches a shell, WSL, Gazebo, ROS or
CAD tooling, and cannot promote source matches to runtime acceptance.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REGISTER = ROOT / "config/high_fidelity_vehicle/formal_requirement_coverage_gap_register.json"
CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
REPORT_ID = "tzcup_formal_requirement_coverage_runtime_gap_audit_v1"
VALID_ITEM_STATUS = {"STATIC_MODEL_CONTROL_GATE_DECLARED", "STATIC_GAP"}


def _read(root: Path, relative: str) -> str:
    try:
        return (root / relative).read_text(encoding="utf-8")
    except FileNotFoundError:
        return ""


def _evidence(root: Path, entry: list[str]) -> dict[str, object]:
    relative, token = entry
    text = _read(root, relative)
    line = next((n for n, value in enumerate(text.splitlines(), 1) if token in value), None)
    return {"path": relative, "token": token, "line": line, "present": line is not None}


def _gate_declared(contract_text: str, gate: str) -> bool:
    return re.search(rf"(?m)^  {re.escape(gate)}:\s*$", contract_text) is not None


def audit(
    root: Path = ROOT,
    *,
    session_relative_path: str = SESSION.relative_to(ROOT).as_posix(),
) -> dict[str, object]:
    root = root.resolve()
    register = json.loads((root / REGISTER.relative_to(ROOT)).read_text(encoding="utf-8"))
    contract_text = (root / CONTRACT.relative_to(ROOT)).read_text(encoding="utf-8")
    session_available = True
    try:
        session = json.loads((root / session_relative_path).read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        session = {}
        session_available = False
    session_status = session.get("status") if session_available else "MISSING_CURRENT_SESSION"
    session_evidence = session.get("evidence")
    session_evidence = session_evidence if isinstance(session_evidence, dict) else {}
    items: list[dict[str, object]] = []
    for requirement in register["requirements"]:
        documentation = [_evidence(root, entry) for entry in requirement["documentation"]]
        model = [_evidence(root, entry) for entry in requirement["model"]]
        control = [_evidence(root, entry) for entry in requirement["control"]]
        gates = [
            {"gate": gate, "declared_in_contract": _gate_declared(contract_text, gate)}
            for gate in requirement["formal_runtime_gates"]
        ]
        static_complete = all(item["present"] for item in documentation + model + control) and all(
            item["declared_in_contract"] for item in gates
        )
        missing = []
        if not all(item["present"] for item in documentation):
            missing.append("DOCUMENTATION_EVIDENCE_MISSING")
        if not all(item["present"] for item in model):
            missing.append("MODEL_EVIDENCE_MISSING")
        if not all(item["present"] for item in control):
            missing.append("CONTROL_EVIDENCE_MISSING")
        if not all(item["declared_in_contract"] for item in gates):
            missing.append("FORMAL_RUNTIME_GATE_DECLARATION_MISSING")
        runtime_missing = [gate["gate"] for gate in gates if gate["gate"] not in session_evidence]
        items.append(
            {
                "id": requirement["id"],
                "family": requirement["family"],
                "requirement": requirement["requirement"],
                "documentation": documentation,
                "model": model,
                "control": control,
                "formal_runtime_gates": gates,
                "static_coverage_status": "STATIC_MODEL_CONTROL_GATE_DECLARED" if static_complete else "STATIC_GAP",
                "static_gap_codes": missing,
                "runtime_evidence_status": "MISSING_CURRENT_SESSION_GATE_EVIDENCE" if runtime_missing else "NOT_ACCEPTED_BY_STATIC_AUDIT",
                "missing_current_session_gates": runtime_missing,
                "runtime_accepted": False,
            }
        )
    static_gaps = [item["id"] for item in items if item["static_coverage_status"] == "STATIC_GAP"]
    runtime_blocked = [item["id"] for item in items if item["missing_current_session_gates"]]
    return {
        "report_id": REPORT_ID,
        "register_id": register["register_id"],
        "audit_mode": "static_source_and_contract_only",
        "execution_prohibited": ["WSL", "bash.exe", "Gazebo", "ROS runtime", "CadQuery", "FreeCAD"],
        "claim_boundary": register["claim_boundary"],
        "acceptance_session": {
            "path": Path(session_relative_path).as_posix(),
            "available": session_available,
            "status": session_status,
            "evidence_key_count": len(session_evidence),
        },
        "items": items,
        "requirement_count": len(items),
        "static_complete_count": len(items) - len(static_gaps),
        "static_gap_items": static_gaps,
        "runtime_blocked_items": runtime_blocked,
        "runtime_accepted": False,
        "status": "FORMAL_REQUIREMENT_COVERAGE_STATIC_GAPS_AND_RUNTIME_BLOCKED" if static_gaps else "FORMAL_REQUIREMENT_COVERAGE_STATIC_COMPLETE_RUNTIME_BLOCKED",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/engineering/formal_requirement_coverage_gap_register.json")
    args = parser.parse_args()
    report = audit(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{report['status']}: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
