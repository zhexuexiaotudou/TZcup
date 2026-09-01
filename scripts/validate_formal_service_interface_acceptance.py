#!/usr/bin/env python3
"""Aggregate fail-closed physical service-interface episode evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from formal_runtime_gate_binding import load_binding


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_SCENARIOS = {
    "charge_allow",
    "charge_reject_no_contact",
    "charge_reject_door_closed",
    "charge_reject_lock_open",
    "drain_allow",
    "drain_reject_no_contact",
    "drain_reject_cap_closed",
    "mutual_interlock_charge_wins",
}
EXPECTED_WASTEWATER_CAPACITY_KG = 8.30
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
FORBIDDEN_SUBSCRIPTION_FRAGMENTS = (
    "/world/",
    "/ground_truth",
    "/truth/",
    "/pose/info",
)
HASHED_SOURCES = (
    "starter_ws/src/sanitation_service_acceptance/models/formal_service_station.sdf",
    (
        "starter_ws/src/sanitation_service_acceptance/launch/"
        "formal_service_acceptance.launch.py"
    ),
    "starter_ws/src/sanitation_service_acceptance/sanitation_service_acceptance/acceptance_core.py",
    "starter_ws/src/sanitation_service_acceptance/sanitation_service_acceptance/collector.py",
    (
        "starter_ws/src/sanitation_power_system/sanitation_power_system/"
        "charge_interface_manager.py"
    ),
    "starter_ws/src/sanitation_safety/sanitation_safety/service_drain_manager.py",
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/power_service_hardware.xacro",
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro",
    "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro",
)

GENERAL_GATES = {
    "joint_state_observed",
    "full_tank_sensor_observed",
    "initial_8_30kg_capacity_reaches_full_sensor",
    "charge_raw_bridge_unique",
    "drain_raw_bridge_unique",
    "a300_bms_state_observed",
    "tank_mass_observed",
    "service_drained_volume_observed",
    "no_world_truth_consumed",
}
CHARGE_GATES = {
    "charge_contact_matches_scenario",
    "charge_door_matches_command",
    "charge_lock_matches_command",
    "charge_enable_matches_expected",
    "charge_connected_matches_expected",
    "charge_power_matches_expected",
    "traction_is_inhibited_during_charge_request",
    "battery_soc_matches_charge_result",
}
DRAIN_GATES = {
    "drain_contact_matches_scenario",
    "drain_cap_matches_command",
    "drain_permit_matches_expected",
    "drain_valve_matches_expected",
    "drain_plant_command_matches_expected",
}


def _source_binding(snapshot_path: Path) -> dict[str, str]:
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    source_hash = snapshot.get("source_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise ValueError("snapshot has no expanded URDF sha256")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, object], dict[str, object]]:
    """Reject service-interface evidence detached from the current formal session."""

    source_binding = _source_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    binding = load_binding(binding_path)
    bound_session = binding.get("acceptance_session_binding")
    if not isinstance(bound_session, dict):
        raise ValueError("runtime binding has no acceptance-session binding")
    if bound_session.get("snapshot") != source_binding:
        raise ValueError("runtime binding snapshot differs from service-interface source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
    ):
        raise ValueError("runtime binding session differs from service-interface session")
    return binding, bound_session


def required_gate_names(scenario: str) -> set[str]:
    required = set(GENERAL_GATES)
    if scenario.startswith("charge_") or scenario == "mutual_interlock_charge_wins":
        required.update(CHARGE_GATES)
    if scenario.startswith("drain_") or scenario == "mutual_interlock_charge_wins":
        required.update(DRAIN_GATES)
        if scenario == "drain_allow":
            required.update(
                {
                    "drain_removes_measured_tank_mass",
                    "drain_reports_measured_removed_volume",
                    "drain_mass_conservation_within_0_02kg",
                }
            )
        else:
            required.update(
                {
                    "rejected_drain_preserves_tank_mass",
                    "rejected_drain_reports_zero_removed_volume",
                }
            )
    return required


def _capacity_contract() -> dict[str, object]:
    vehicle = (ROOT / HASHED_SOURCES[-1]).read_text(encoding="utf-8")
    storage = (ROOT / HASHED_SOURCES[-2]).read_text(encoding="utf-8")
    required_vehicle_fragments = (
        "<wastewater_capacity_kg>8.30</wastewater_capacity_kg>",
        "<tank_capacity_kg>8.30</tank_capacity_kg>",
    )
    storage_clamp = "max(min(float(wastewater_load_mass_kg), 8.30), 0.001)"
    gates = {
        "dynamic_payload_capacity_kg_is_8_30": required_vehicle_fragments[0]
        in vehicle,
        "water_recovery_capacity_kg_is_8_30": required_vehicle_fragments[1]
        in vehicle,
        "expanded_storage_payload_clamp_kg_is_8_30": storage_clamp in storage,
    }
    return {
        "expected_wastewater_capacity_kg": EXPECTED_WASTEWATER_CAPACITY_KG,
        "gates": gates,
        "status": "PASS" if all(gates.values()) else "FAIL_CLOSED",
    }


def aggregate(
    episodes_dir: Path,
    runtime_binding_path: Path | None = None,
    *,
    snapshot_path: Path | None = None,
    session_path: Path | None = None,
) -> dict:
    episodes: dict[str, dict] = {}
    errors: list[str] = []
    for path in sorted(episodes_dir.glob("*.json")):
        try:
            episode = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            errors.append(f"invalid episode {path.name}: {error}")
            continue
        scenario = episode.get("scenario")
        if scenario not in EXPECTED_SCENARIOS:
            errors.append(f"unexpected scenario in {path.name}: {scenario}")
            continue
        if scenario in episodes:
            errors.append(f"duplicate scenario: {scenario}")
            continue
        subscriptions = episode.get("subscription_topics", [])
        forbidden = [
            topic
            for topic in subscriptions
            if any(fragment in topic for fragment in FORBIDDEN_SUBSCRIPTION_FRAGMENTS)
        ]
        if forbidden:
            errors.append(f"{scenario} consumes forbidden world truth: {forbidden}")
        if episode.get("schema") != "tzcup.formal_service_interface_episode.v1":
            errors.append(f"{scenario} has an invalid episode schema")
        if episode.get("result") != "PASS":
            errors.append(f"{scenario} did not pass")
        gates = episode.get("gates")
        if not isinstance(gates, dict) or not all(gates.values()):
            errors.append(f"{scenario} has an open gate")
        else:
            missing_gates = sorted(required_gate_names(scenario) - gates.keys())
            if missing_gates:
                errors.append(f"{scenario} is missing required gates: {missing_gates}")
        if episode.get("wastewater_capacity_kg") != EXPECTED_WASTEWATER_CAPACITY_KG:
            errors.append(f"{scenario} does not assert 8.30 kg wastewater capacity")
        episodes[scenario] = episode
    missing = sorted(EXPECTED_SCENARIOS - episodes.keys())
    if missing:
        errors.append(f"missing scenarios: {missing}")
    capacity_contract = _capacity_contract()
    if capacity_contract["status"] != "PASS":
        errors.append("8.30 kg wastewater capacity source contract is open")
    hashes = {
        relative: hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
        for relative in HASHED_SOURCES
    }
    result = {
        "schema": "tzcup.formal_service_interface_acceptance.v1",
        "status": "PASS" if not errors else "FAIL_CLOSED",
        "scenario_count": len(episodes),
        "expected_scenarios": sorted(EXPECTED_SCENARIOS),
        "episodes": episodes,
        "wastewater_capacity_contract": capacity_contract,
        "errors": errors,
        "source_sha256": hashes,
        "truth_boundary": (
            "Physical fixture Contacts and product JointState/status only; no "
            "product node or evaluator gate consumes world truth."
        ),
    }
    if runtime_binding_path is not None:
        if snapshot_path is None or session_path is None:
            raise ValueError("snapshot and session are required with runtime binding")
        binding, acceptance_session_binding = _bound_runtime_evidence(
            snapshot_path, session_path, runtime_binding_path
        )
        result["runtime_gate_binding"] = binding
        result["acceptance_session_binding"] = acceptance_session_binding
        result["runtime_closure_binding"] = binding["runtime_closure_binding"]
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    args = parser.parse_args()
    result = aggregate(
        args.episodes_dir,
        args.runtime_binding,
        snapshot_path=args.snapshot,
        session_path=args.session,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
