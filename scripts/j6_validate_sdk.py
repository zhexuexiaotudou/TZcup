#!/usr/bin/env python3
"""Validate a Journey 6 SDK discovery report and optional toolchain lock."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


FORBIDDEN = ("rdk", "s100", "s100p", "s600")
ALLOWED_MARCHES = ("auto", "nash-e", "nash-m", "nash-p")


def validate(discovery: dict, lock: dict | None = None) -> dict:
    failures: list[str] = []
    if discovery.get("target_family") != "journey6":
        failures.append("target_family_must_be_journey6")
    accepted = discovery.get("accepted_sdk_roots", [])
    if not accepted:
        failures.append("journey6_official_sdk_missing")
    candidate_text = json.dumps(discovery.get("candidates", []), sort_keys=True).lower()
    accepted_rows = [row for row in discovery.get("candidates", []) if row.get("accepted")]
    if any(marker in json.dumps(row).lower() for row in accepted_rows for marker in FORBIDDEN):
        failures.append("accepted_sdk_contains_forbidden_rdk_or_s100_marker")
    if lock is not None:
        if lock.get("target_family") != "journey6":
            failures.append("toolchain_lock_family_mismatch")
        if lock.get("target_sku") != "auto":
            failures.append("toolchain_lock_sku_must_remain_auto_until_inventory")
        march = lock.get("target_march", "auto")
        if march not in ALLOWED_MARCHES:
            failures.append("unsupported_or_unknown_march")
        source = json.dumps(lock.get("source", {}), sort_keys=True).lower()
        if any(marker in source for marker in FORBIDDEN):
            failures.append("toolchain_lock_references_forbidden_rdk_or_s100_package")
    ready = not failures
    return {
        "schema_version": 1,
        "target_family": "journey6",
        "status": "ready" if ready else "blocked_external",
        "sdk_validation_pass": ready,
        "failures": failures,
        "observed_candidate_count": len(discovery.get("candidates", [])),
        "accepted_sdk_count": len(accepted),
        "discovery_contains_forbidden_text": any(marker in candidate_text for marker in FORBIDDEN),
        "truth_boundary": "Validation is not model compile, x86 runtime, or real-board evidence.",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--discovery", "--inventory", dest="discovery", required=True)
    parser.add_argument("--toolchain-lock")
    parser.add_argument("--output", default="J6_SDK_VALIDATION.json")
    args = parser.parse_args()
    discovery = json.loads(Path(args.discovery).read_text(encoding="utf-8"))
    lock = yaml.safe_load(Path(args.toolchain_lock).read_text(encoding="utf-8")) if args.toolchain_lock else None
    report = validate(discovery, lock)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.suffix.lower() in (".yaml", ".yml"):
        output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    else:
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["sdk_validation_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
