#!/usr/bin/env python3
"""Validate the fail-closed S100P installation/power evidence contract offline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / "config/high_fidelity_vehicle/s100p_mechanical_electrical_evidence.json"
RECORD_ID = "tzcup_s100p_mechanical_electrical_evidence_v1"
BLOCKED_STATUS = "BLOCKED_MECHANICAL_ELECTRICAL_INTEGRATION"
OFFICIAL_URLS = {
    "https://d-robotics.github.io/rdk_doc/rdk_s/Quick_start/hardware_introduction/rdk_s100/",
    "https://archive.d-robotics.cc/downloads/hardware/rdk_s100/rdk_s100/",
}
SOURCE_EVIDENCE_LEVELS = {
    "OFFICIAL_PRIMARY_PUBLIC_DOCUMENTATION",
    "OFFICIAL_PRIMARY_PUBLIC_DIRECTORY",
    "LOCAL_READ_ONLY_IDENTITY_ARTIFACT",
}
REQUIRED_BLOCKERS = {
    "exact_owned_board_dimensions_and_boundary",
    "mounting_hole_pattern_and_datums",
    "board_mass_and_center_of_mass",
    "connector_coordinates_keepouts_and_cable_service_space",
    "heatsink_fan_and_airflow_envelope",
    "j1_pinout_polarity_and_real_harness",
    "dc_dc_fuse_wire_gauge_grounding_and_transient_protection",
    "enclosure_thermal_derating_and_temperature_measurement",
    "installed_power_on_and_runtime_validation",
}


def validate(payload: object, root: Path = ROOT) -> None:
    if not isinstance(payload, dict) or payload.get("schema_version") != 1 or payload.get("record_id") != RECORD_ID:
        raise ValueError("invalid S100P mechanical/electrical evidence record")
    if payload.get("status") != BLOCKED_STATUS:
        raise ValueError("S100P installation/power evidence must remain blocked")
    policy = payload.get("open_source_only_policy")
    if not isinstance(policy, dict) or policy.get("required") is not True:
        raise ValueError("open-source-only policy is required")
    if policy.get("restricted_step_action") != "DO_NOT_DOWNLOAD_DO_NOT_COMMIT_DO_NOT_DERIVE_DO_NOT_DECLARE_REUSABLE":
        raise ValueError("restricted official STEP must remain prohibited")
    sources = payload.get("sources")
    if not isinstance(sources, list):
        raise ValueError("sources must be a list")
    source_by_id = {source.get("id"): source for source in sources if isinstance(source, dict)}
    if len(source_by_id) != len(sources) or any(
        source.get("evidence_level") not in SOURCE_EVIDENCE_LEVELS
        for source in source_by_id.values()
    ):
        raise ValueError("each source must have a recognized evidence level")
    official_urls = {source.get("url") for source in sources if isinstance(source, dict) and isinstance(source.get("url"), str)}
    if not OFFICIAL_URLS <= official_urls:
        raise ValueError("both required official primary URLs must be recorded")
    identity = source_by_id.get("local_read_only_board_identity")
    if not isinstance(identity, dict) or not isinstance(identity.get("path"), str) or not (root / identity["path"]).is_file():
        raise ValueError("retained local read-only identity artifact is required")
    items = payload.get("evidence_items")
    if not isinstance(items, list):
        raise ValueError("evidence_items must be a list")
    by_id = {item.get("id"): item for item in items if isinstance(item, dict)}
    if len(by_id) != len(items):
        raise ValueError("evidence item IDs must be unique")
    expected = {
        "board_model": "KS1P75Y",
        "soc": "S100P",
        "cpu_frequency": 2.0,
        "memory_capacity": 24,
        "ai_compute": 128,
        "acrylic_enclosure_nominal_dimensions": [120, 121, 51],
        "dc_input_voltage_range": [12, 20],
        "j1_rated_input": [20, 10],
        "typical_input_power": [12, 5.5, 70],
        "maximum_input_power": [20, 7.5, 150],
        "operating_temperature_range": [0, 45],
        "connector_model_table": "Official documentation publishes a connector-model table.",
        "local_board_identity": "D-Robotics RDK S100P V1P0",
    }
    for item_id, value in expected.items():
        item = by_id.get(item_id)
        if not isinstance(item, dict) or item.get("value") != value:
            raise ValueError(f"missing or changed official/local evidence item: {item_id}")
        if item.get("can_freeze_urdf") is not False:
            raise ValueError(f"{item_id} must not authorize a URDF freeze")
        source_id = item.get("source_id")
        if source_id not in source_by_id:
            raise ValueError(f"{item_id} has no recorded source")
        if item.get("evidence_level") != source_by_id[source_id].get("evidence_level"):
            raise ValueError(f"{item_id} evidence level must match its recorded source")
        if not isinstance(item.get("urdf_disposition"), str) or not item["urdf_disposition"]:
            raise ValueError(f"{item_id} must state its URDF disposition")
    if any(item.get("can_freeze_urdf") is not False for item in by_id.values()):
        raise ValueError("no evidence item may authorize a URDF freeze")
    geometry = by_id["acrylic_enclosure_nominal_dimensions"]
    if set(geometry.get("blocked_by", [])) < {
        "bare_board_boundary",
        "mounting_hole_pattern",
        "connector_keepout",
        "heatsink_fan_envelope",
    }:
        raise ValueError("published enclosure dimensions must retain all mechanical blockers")
    connector = by_id["connector_model_table"]
    if set(connector.get("blocked_by", [])) < {
        "connector_coordinates",
        "connector_keepout",
        "j1_pinout_and_polarity",
        "cable_bend_and_service_space",
    }:
        raise ValueError("connector model names must retain coordinate and harness blockers")
    blockers = set(payload.get("blocked_gates", []))
    if not REQUIRED_BLOCKERS <= blockers:
        raise ValueError("critical installation/power blockers must remain explicit")
    acceptance = payload.get("acceptance")
    if not isinstance(acceptance, dict) or any(acceptance.get(key) is not False for key in ("urdf_update_authorized", "mechanical_installation_accepted", "electrical_installation_accepted", "runtime_accepted")):
        raise ValueError("S100P evidence contract must not accept URDF, installation or runtime")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--evidence", type=Path, default=DEFAULT)
    args = parser.parse_args()
    validate(json.loads(args.evidence.read_text(encoding="utf-8")), args.root.resolve())
    print(f"valid fail-closed S100P mechanical/electrical evidence: {args.evidence}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
