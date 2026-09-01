#!/usr/bin/env python3
"""Validate fail-closed semantics of the formal requirement coverage register."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from audit_formal_requirement_coverage import REPORT_ID, VALID_ITEM_STATUS


def validate(report: object) -> None:
    if not isinstance(report, dict) or report.get("report_id") != REPORT_ID:
        raise ValueError("not a formal requirement coverage report")
    if report.get("audit_mode") != "static_source_and_contract_only":
        raise ValueError("coverage report must remain static-only")
    if report.get("runtime_accepted") is not False:
        raise ValueError("static coverage must never accept runtime")
    if not isinstance(report.get("claim_boundary"), str) or not report["claim_boundary"]:
        raise ValueError("claim boundary is required")
    if set(report.get("execution_prohibited", [])) < {"WSL", "bash.exe", "Gazebo", "CadQuery", "FreeCAD"}:
        raise ValueError("prohibited execution boundary weakened")
    session = report.get("acceptance_session")
    if not isinstance(session, dict) or not isinstance(session.get("evidence_key_count"), int):
        raise ValueError("acceptance session conclusion is required")
    items = report.get("items")
    if not isinstance(items, list) or not items:
        raise ValueError("items must be a non-empty list")
    ids: set[str] = set()
    static_gaps: list[str] = []
    runtime_blocked: list[str] = []
    for item in items:
        if not isinstance(item, dict) or not isinstance(item.get("id"), str) or item["id"] in ids:
            raise ValueError("items need unique ids")
        ids.add(item["id"])
        if item.get("static_coverage_status") not in VALID_ITEM_STATUS:
            raise ValueError(f"invalid static status: {item['id']}")
        for group in ("documentation", "model", "control"):
            evidence = item.get(group)
            if not isinstance(evidence, list) or not evidence or not all(isinstance(row, dict) and isinstance(row.get("present"), bool) for row in evidence):
                raise ValueError(f"{item['id']} lacks {group} evidence")
        gates = item.get("formal_runtime_gates")
        if not isinstance(gates, list) or not gates or not all(isinstance(gate, dict) and isinstance(gate.get("gate"), str) and isinstance(gate.get("declared_in_contract"), bool) for gate in gates):
            raise ValueError(f"{item['id']} lacks formal runtime gates")
        if item.get("runtime_accepted") is not False:
            raise ValueError(f"{item['id']} wrongly accepts runtime")
        missing = item.get("missing_current_session_gates")
        if not isinstance(missing, list):
            raise ValueError(f"{item['id']} lacks current-session gate conclusion")
        if item["static_coverage_status"] == "STATIC_GAP":
            static_gaps.append(item["id"])
        if missing:
            runtime_blocked.append(item["id"])
    if report.get("requirement_count") != len(items) or report.get("static_complete_count") != len(items) - len(static_gaps):
        raise ValueError("requirement counts disagree")
    if report.get("static_gap_items") != static_gaps or report.get("runtime_blocked_items") != runtime_blocked:
        raise ValueError("gap summary disagrees with items")
    expected = "FORMAL_REQUIREMENT_COVERAGE_STATIC_GAPS_AND_RUNTIME_BLOCKED" if static_gaps else "FORMAL_REQUIREMENT_COVERAGE_STATIC_COMPLETE_RUNTIME_BLOCKED"
    if report.get("status") != expected:
        raise ValueError("top-level status disagrees with static gaps")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    validate(json.loads(args.report.read_text(encoding="utf-8")))
    print(f"valid formal requirement coverage report: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
