#!/usr/bin/env python3
"""Fail fast when the simulation and production hardware contract drifts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def _positive(mapping: dict, key: str, errors: list[str], prefix: str) -> None:
    value = mapping.get(key)
    if not isinstance(value, (int, float)) or value <= 0:
        errors.append(f"{prefix}.{key} must be positive")


def validate_contract(contract: dict) -> list[str]:
    errors: list[str] = []
    if contract.get("coordinate_convention", {}).get("units") != "SI":
        errors.append("coordinate_convention.units must be SI")
    velocity = contract.get("command_interfaces", {}).get("velocity", {})
    for key in ("minimum_rate_hz", "timeout_sec"):
        _positive(velocity, key, errors, "command_interfaces.velocity")
    limits = velocity.get("limits", {})
    for key in ("linear_x_mps", "angular_z_radps"):
        bounds = limits.get(key)
        if (
            not isinstance(bounds, list)
            or len(bounds) != 2
            or not all(isinstance(item, (int, float)) for item in bounds)
            or bounds[0] >= 0
            or bounds[1] <= 0
        ):
            errors.append(f"command_interfaces.velocity.limits.{key} must cross zero")
    for name, sensor in contract.get("sensor_interfaces", {}).items():
        _positive(sensor, "minimum_rate_hz", errors, f"sensor_interfaces.{name}")
        _positive(sensor, "freshness_timeout_sec", errors, f"sensor_interfaces.{name}")
        if not str(sensor.get("topic", "")).startswith("/"):
            errors.append(f"sensor_interfaces.{name}.topic must be absolute")
    safety = contract.get("safety_contract", {})
    for key in ("maximum_command_stop_latency_sec", "maximum_brush_stop_latency_sec"):
        _positive(safety, key, errors, "safety_contract")
    if not safety.get("require_hardware_estop_chain"):
        errors.append("safety_contract.require_hardware_estop_chain must be true")
    if contract.get("time_contract", {}).get("production_uses_sim_time") is not False:
        errors.append("time_contract.production_uses_sim_time must be false")
    for gate in ("sil", "hil", "field"):
        if gate not in contract.get("acceptance_gates", {}):
            errors.append(f"acceptance_gates.{gate} is required")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("contract", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    contract = yaml.safe_load(args.contract.read_text(encoding="utf-8"))
    errors = validate_contract(contract)
    report = {
        "schema": "tzcup.hardware_interface_contract_validation.v1",
        "contract": str(args.contract),
        "valid": not errors,
        "errors": errors,
    }
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
