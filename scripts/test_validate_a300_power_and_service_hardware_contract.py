from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_a300_power_and_service_hardware_contract import (
    A300PowerServiceContractError,
    DEFAULT_CONTRACT,
    validate,
)


def _mutated_contract(tmp_path: Path, mutate) -> Path:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    mutate(contract)
    output = tmp_path / "a300-power-service-mutated.yaml"
    output.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    return output


def test_committed_power_and_service_hardware_is_integrated_and_valid() -> None:
    result = validate()
    assert result == {
        "contract_id": "a300_power_charge_and_drain_hardware_preintegration_v1",
        "status": "A300_POWER_AND_SERVICE_HARDWARE_INTEGRATED_STATIC_VALID",
        "battery_pack_count": 2,
        "inferred_pack_mass_kg": 7.5,
        "explicit_battery_mass_kg": 15.0,
        "a300_curb_mass_kg": 78.5,
        "charge_link_count": 4,
        "drain_link_count": 6,
        "runtime_integrated": True,
    }


def test_pack_mass_inference_cannot_be_relabelled_official(tmp_path: Path) -> None:
    contract = _mutated_contract(
        tmp_path,
        lambda data: data["truth_boundary"]["derived_not_published_single_pack_mass"].update(
            {"source_class": "official_single_pack_mass"}
        ),
    )
    with pytest.raises(A300PowerServiceContractError, match="engineering inference"):
        validate(contract)


def test_battery_transfer_must_preserve_curb_mass(tmp_path: Path) -> None:
    contract = _mutated_contract(
        tmp_path,
        lambda data: data["a300_curb_mass_transfer"].update(
            {"explicit_two_battery_assemblies_kg": 30.0}
        ),
    )
    with pytest.raises(A300PowerServiceContractError, match="78.5 kg"):
        validate(contract)


def test_battery_envelopes_cannot_overlap(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["battery_and_bms_design"]["assemblies"][1]["xyz_m"] = [-0.08, 0.095, 0.075]

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300PowerServiceContractError, match="battery envelopes overlap"):
        validate(contract)


def test_charge_lock_must_be_physical_prismatic_joint(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        lock = next(
            item for item in data["charge_interface_design"]["links"]
            if item["name"] == "charge_connector_lock_link"
        )
        lock["joint_type"] = "fixed"

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300PowerServiceContractError, match="physical bounded prismatic"):
        validate(contract)


def test_charge_plug_cannot_revert_to_synthetic_boolean(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        interfaces = data["charge_interface_design"]["implemented_interfaces"]
        interfaces[0] = {
            "topic": "/formal_vehicle/power/charge_plug_present",
            "type": "std_msgs/msg/Bool",
            "direction": "subscription",
        }

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300PowerServiceContractError, match="raw Contacts"):
        validate(contract)


def test_drain_hose_cannot_revert_to_free_boolean(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        interfaces = data["wastewater_drain_valve_design"]["implemented_interfaces"]
        interfaces[0]["type"] = "std_msgs/msg/Bool"

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300PowerServiceContractError, match="raw Contacts"):
        validate(contract)


def test_drain_valve_must_fail_closed(tmp_path: Path) -> None:
    contract = _mutated_contract(
        tmp_path,
        lambda data: data["wastewater_drain_valve_design"].update(
            {"fail_safe_position_rad": 1.570796327}
        ),
    )
    with pytest.raises(A300PowerServiceContractError, match="fail-safe position"):
        validate(contract)


def test_drain_mass_must_equal_physical_link_sum(tmp_path: Path) -> None:
    contract = _mutated_contract(
        tmp_path,
        lambda data: data["wastewater_drain_valve_design"].update(
            {"assembly_mass_kg": 1.1}
        ),
    )
    with pytest.raises(A300PowerServiceContractError, match="link masses"):
        validate(contract)
