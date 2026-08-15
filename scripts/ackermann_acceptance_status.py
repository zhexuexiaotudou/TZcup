#!/usr/bin/env python3
"""Derive fail-closed Ackermann gate status from measured evidence files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


GATES = (
    "ACKERMANN_MODEL_PASS",
    "ACKERMANN_PHYSICS_PASS",
    "ACKERMANN_ODOMETRY_PASS",
    "ACKERMANN_LOCALIZATION_PASS",
    "ACKERMANN_NAV2_PASS",
    "ACKERMANN_COVERAGE_PASS",
    "ACKERMANN_DYNAMIC_PASS",
    "ACKERMANN_ESTOP_PASS",
    "ACKERMANN_REPLAY_PASS",
)


def derive_status(evidence_dir: Path) -> dict:
    gates = {}
    evidence = {}
    first_blocker = None
    for gate in GATES:
        path = evidence_dir / f"{gate.lower()}.json"
        passed = False
        reason = "evidence_missing"
        if path.is_file():
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                passed = payload.get("passed") is True
                reason = payload.get("first_failure") or (
                    "passed" if passed else "evidence_did_not_pass"
                )
            except (OSError, json.JSONDecodeError) as error:
                reason = f"evidence_unreadable:{type(error).__name__}"
        gates[gate] = passed
        evidence[gate] = {"path": path.name, "reason": reason}
        if not passed and first_blocker is None:
            first_blocker = gate
    default_ready = all(gates.values())
    return {
        "schema_version": 1,
        **gates,
        "ACKERMANN_DEFAULT_PROFILE_READY": default_ready,
        "first_blocking_layer": first_blocker,
        "evidence": evidence,
        "fail_closed": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    status = derive_status(args.evidence_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(status, indent=2))
    return 0 if status["ACKERMANN_DEFAULT_PROFILE_READY"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
