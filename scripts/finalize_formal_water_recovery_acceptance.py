#!/usr/bin/env python3
"""Combine the two independent formal water-recovery runtime episodes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def combine(normal_path: Path, full_path: Path) -> dict[str, Any]:
    normal = _load(normal_path)
    full = _load(full_path)
    scenario_names_valid = (
        normal.get("scenario") == "normal_recovery"
        and full.get("scenario") == "full_tank_fail_closed"
    )
    passed = bool(normal.get("passed")) and bool(full.get("passed")) and scenario_names_valid
    return {
        "schema_version": 1,
        "status": (
            "FORMAL_WATER_RECOVERY_ACCEPTANCE_PASSED" if passed else "FAILED"
        ),
        "passed": passed,
        "checks": {
            "normal_recovery_passed": bool(normal.get("passed")),
            "full_tank_fail_closed_passed": bool(full.get("passed")),
            "scenario_names_valid": scenario_names_valid,
        },
        "summary": {
            "normal_recovery_rate": normal.get("metrics", {}).get("recovery_rate"),
            "normal_mass_balance_error_fraction": normal.get("metrics", {}).get(
                "mass_balance_error_fraction"
            ),
            "normal_nozzle_covered_column_count": normal.get("metrics", {}).get(
                "nozzle_covered_column_count"
            ),
            "normal_ready_duty_cycle": normal.get("metrics", {}).get(
                "all_conditions_ready_duty_cycle"
            ),
            "full_tank_mass_kg": full.get("at_full", {}).get("tank_mass_kg"),
            "full_post_stop_ground_delta_l": full.get("metrics", {}).get(
                "post_full_ground_delta_l"
            ),
        },
        "evidence": {
            "normal_json": str(normal_path.resolve()),
            "normal_sha256": _sha256(normal_path),
            "full_json": str(full_path.resolve()),
            "full_sha256": _sha256(full_path),
        },
        "claim_boundary": (
            "Gazebo L1 sparse 2.5-D finite-water proxy with physical actuator, "
            "geometry, pump-flow, tank-mass and visual-state coupling; this does "
            "not claim CFD, spray, foam, slosh, or wet-surface material dynamics."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--normal", type=Path, required=True)
    parser.add_argument("--full", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = combine(args.normal, args.full)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
