#!/usr/bin/env python3
"""Fail-closed static validator for the body/sensor/power fourth B-rep batch."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from validate_native_brep_pending_batch import validate_batch


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_RELATIVE_PATH = Path("config/high_fidelity_vehicle/native_brep_body_sensor_power_fourth_batch_contract.json")
MANIFEST_RELATIVE_PATH = Path("starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch_source_manifest.json")
SOURCE_RELATIVE_PATH = Path("starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch.py")
EXPECTED_IDS = ("bodywork_access_set", "sensor_mast_and_installation_brackets", "compute_and_control_cabinet_mounting", "power_distribution_mounting_enclosures")


def validate(root: Path = REPOSITORY_ROOT, contract_path: Path | None = None, manifest_path: Path | None = None, source_path: Path | None = None) -> dict[str, Any]:
    return validate_batch(root, contract_relative=CONTRACT_RELATIVE_PATH, manifest_relative=MANIFEST_RELATIVE_PATH, source_relative=SOURCE_RELATIVE_PATH, document_id="tzcup_native_brep_body_sensor_power_fourth_batch_parametric_contract_v1", manifest_id="tzcup_native_brep_body_sensor_power_fourth_batch_cadquery_source_manifest_v1", expected_ids=EXPECTED_IDS, summary_count_key="fourth_batch_component_count", require_shared_assembly_step=False, contract_path=contract_path, manifest_path=manifest_path, source_path=source_path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=REPOSITORY_ROOT)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = validate(args.root, args.contract, args.manifest, args.source)
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
