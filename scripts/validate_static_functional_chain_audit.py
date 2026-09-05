#!/usr/bin/env python3
"""Validate the schema and fail-closed semantics of a static chain audit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from generate_static_functional_chain_audit import (
    REPORT_ID,
    REQUIRED_ITEM_IDS,
    SUPPLEMENTAL_ITEM_IDS,
    VALID_STATUSES,
)
from validate_s100p_mechanical_electrical_evidence import BLOCKED_STATUS as S100P_BLOCKED_STATUS


def validate(report: object, *, require_static_closed: bool = False) -> None:
    if not isinstance(report, dict) or report.get("report_id") != REPORT_ID:
        raise ValueError("not a static functional-chain audit report")
    if report.get("audit_mode") != "static_source_only":
        raise ValueError("audit mode must remain static_source_only")
    if report.get("status_scope") != "core_13_item_legacy_static_functional_chain_only":
        raise ValueError("status_scope must prevent core status from being misread as the expanded scope")
    items = report.get("items")
    if not isinstance(items, list) or not items:
        raise ValueError("items must be a non-empty list")
    ids: list[str] = []
    seen_ids: set[str] = set()
    blocked: list[str] = []
    for item in items:
        if not isinstance(item, dict) or not isinstance(item.get("id"), str):
            raise ValueError("each item requires a string id")
        if item["id"] in seen_ids:
            raise ValueError(f"duplicate item id: {item['id']}")
        seen_ids.add(item["id"])
        ids.append(item["id"])
        if item.get("status") not in VALID_STATUSES:
            raise ValueError(f"invalid item status: {item['id']}")
        checks = item.get("checks")
        if not isinstance(checks, list) or not checks:
            raise ValueError(f"item {item['id']} lacks source checks")
        for check in checks:
            evidence = check.get("evidence") if isinstance(check, dict) else None
            if not isinstance(evidence, dict) or not isinstance(evidence.get("path"), str):
                raise ValueError(f"item {item['id']} has invalid evidence")
        if not isinstance(item.get("physical_semantics"), str) or not item["physical_semantics"]:
            raise ValueError(f"item {item['id']} lacks a physical-semantics assessment")
        if not isinstance(item.get("placeholder_indicators"), list) or not all(
            isinstance(indicator, str) and indicator
            for indicator in item["placeholder_indicators"]
        ):
            raise ValueError(f"item {item['id']} has invalid placeholder indicators")
        if item["status"] == "BLOCKED":
            if not isinstance(item.get("blocked_reason"), str) or not item["blocked_reason"]:
                raise ValueError(f"blocked item {item['id']} lacks a reason")
            blocked.append(item["id"])
    if tuple(ids) != REQUIRED_ITEM_IDS:
        raise ValueError("audit must contain exactly the required 13 functional-chain items")
    if report.get("required_item_count") != len(REQUIRED_ITEM_IDS):
        raise ValueError("required_item_count must remain 13")
    if report.get("static_closed_count") != len(ids) - len(blocked):
        raise ValueError("static_closed_count disagrees with item statuses")
    if report.get("runtime_accepted") is not False:
        raise ValueError("static functional-chain audit must not accept runtime")
    if report.get("fresh_gazebo_runtime_required") is not True:
        raise ValueError("static functional-chain audit must require fresh Gazebo runtime evidence")
    if report.get("blocked_items") != blocked:
        raise ValueError("blocked_items must exactly match blocked item ids")
    expected_status = "BLOCKED" if blocked else "STATIC_CLOSED"
    if report.get("status") != expected_status:
        raise ValueError("top-level status disagrees with blocked items")
    supplemental = report.get("supplemental_items")
    if not isinstance(supplemental, list) or tuple(
        item.get("id") if isinstance(item, dict) else None for item in supplemental
    ) != SUPPLEMENTAL_ITEM_IDS:
        raise ValueError("supplemental_items must contain the declared expanded-scope items")
    supplemental_blocked: list[str] = []
    for item in supplemental:
        if not isinstance(item, dict) or item.get("status") not in VALID_STATUSES:
            raise ValueError("supplemental item has invalid status")
        if not isinstance(item.get("physical_semantics"), str) or not item["physical_semantics"]:
            raise ValueError("supplemental item lacks a physical-semantics assessment")
        if not isinstance(item.get("placeholder_indicators"), list):
            raise ValueError("supplemental item has invalid placeholder indicators")
        if item["status"] == "BLOCKED":
            if not isinstance(item.get("blocked_reason"), str) or not item["blocked_reason"]:
                raise ValueError("blocked supplemental item lacks a reason")
            supplemental_blocked.append(item["id"])
    if report.get("supplemental_blocked_items") != supplemental_blocked:
        raise ValueError("supplemental_blocked_items disagrees with supplemental statuses")
    if report.get("expanded_required_item_count") != len(REQUIRED_ITEM_IDS) + len(SUPPLEMENTAL_ITEM_IDS):
        raise ValueError("expanded_required_item_count disagrees with the fixed audit scope")
    if report.get("expanded_static_closed_count") != len(ids) - len(blocked) + len(supplemental) - len(supplemental_blocked):
        raise ValueError("expanded_static_closed_count disagrees with item statuses")
    expected_expanded_status = "BLOCKED" if blocked or supplemental_blocked else "STATIC_CLOSED"
    if report.get("expanded_scope_status") != expected_expanded_status:
        raise ValueError("expanded_scope_status disagrees with expanded item statuses")
    if report.get("expanded_scope_runtime_accepted") is not False:
        raise ValueError("expanded static scope must not accept runtime")
    if not isinstance(report.get("expanded_scope_boundary"), str) or not report["expanded_scope_boundary"]:
        raise ValueError("expanded scope requires an explicit runtime boundary")
    s100p_evidence = report.get("s100p_mechanical_electrical_evidence")
    if not isinstance(s100p_evidence, dict) or s100p_evidence.get("validator") != "validate_s100p_mechanical_electrical_evidence.py":
        raise ValueError("static audit must cite the S100P evidence validator conclusion")
    if s100p_evidence.get("validator_complete") is not True or s100p_evidence.get("status") != S100P_BLOCKED_STATUS:
        raise ValueError("S100P evidence conclusion must be validated and remain BLOCKED")
    acceptance = s100p_evidence.get("acceptance")
    if not isinstance(acceptance, dict) or any(acceptance.get(key) is not False for key in ("urdf_update_authorized", "mechanical_installation_accepted", "electrical_installation_accepted", "runtime_accepted")):
        raise ValueError("S100P evidence conclusion must not accept URDF, installation or runtime")
    blockers = s100p_evidence.get("blocked_gates")
    if not isinstance(blockers, list) or not {"mounting_hole_pattern_and_datums", "board_mass_and_center_of_mass", "connector_coordinates_keepouts_and_cable_service_space", "enclosure_thermal_derating_and_temperature_measurement", "installed_power_on_and_runtime_validation"} <= set(blockers):
        raise ValueError("S100P evidence conclusion lacks critical unresolved gates")
    if require_static_closed and blocked:
        raise ValueError("audit remains BLOCKED: " + ", ".join(blocked))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--require-static-closed", action="store_true")
    args = parser.parse_args()
    validate(json.loads(args.report.read_text(encoding="utf-8")), require_static_closed=args.require_static_closed)
    print(f"valid static audit: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
