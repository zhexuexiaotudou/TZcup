from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_formal_mechanical_interface_datums import (
    DEFAULT_CROSSWALK,
    MechanicalDatumValidationError,
    validate,
)


def _crosswalk_copy(tmp_path: Path) -> dict:
    return yaml.safe_load(DEFAULT_CROSSWALK.read_text(encoding="utf-8"))


def _write_crosswalk(tmp_path: Path, data: dict) -> Path:
    output = tmp_path / "mechanical-interface-datums.yaml"
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return output


def _interface(data: dict, interface_id: str) -> dict:
    return next(item for item in data["interface_crosswalk"] if item["interface_id"] == interface_id)


def test_committed_crosswalk_is_snapshot_bound_and_not_a_manufacturing_release() -> None:
    result = validate()
    assert result["status"] == "STATIC_DERIVED_SNAPSHOT_BOUND_NOT_MANUFACTURING_RELEASE"
    assert result["manufacturing_release"] is False
    assert result["coordinate_reference"] == "base_footprint"
    assert result["nominal_joint_configuration"] == "zero"
    assert result["datum_count"] == 16
    assert result["interface_count"] == 7
    assert set(result["checked_interfaces"]) == {
        "chassis_top_plate_to_arm_base",
        "chassis_top_plate_to_sensor_tower",
        "chassis_to_cleaning_head",
        "chassis_top_plate_to_dry_bin",
        "chassis_top_plate_to_wastewater_tank",
        "chassis_to_charge_receptacle",
        "wastewater_tank_to_drain_hose",
    }
    assert set(result["source_snapshot_sha256"]) == {
        "config/high_fidelity_vehicle/formal_vehicle_layout.yaml",
        "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml",
        "reports/engineering/formal_competition_vehicle.urdf",
    }
    assert result["observed_datums"]["arm_base_mechanical"]["xyz_m"] == [0.1, -0.2, 0.4791]
    assert result["observed_datums"]["charge_receptacle_endpoint"]["xyz_m"] == [0.25, 0.402, 0.4831]
    assert result["observed_datums"]["wastewater_drain_hose_endpoint"]["xyz_m"] == [-0.555, -0.305, 0.4801]


def test_snapshot_hash_drift_fails_before_interface_evaluation(tmp_path: Path) -> None:
    data = _crosswalk_copy(tmp_path)
    data["source_snapshot"]["inputs"][0]["sha256"] = "0" * 64
    with pytest.raises(MechanicalDatumValidationError, match="snapshot hash mismatch"):
        validate(_write_crosswalk(tmp_path, data))


def test_datum_coordinate_drift_fails_closed(tmp_path: Path) -> None:
    data = _crosswalk_copy(tmp_path)
    roller = next(item for item in data["datum_catalog"] if item["datum_id"] == "cleaning_roller_axis")
    roller["expected_root_pose"]["xyz_m"][0] += 0.002
    with pytest.raises(MechanicalDatumValidationError, match="cleaning_roller_axis FK position error"):
        validate(_write_crosswalk(tmp_path, data))


def test_component_register_interface_binding_drift_fails_closed(tmp_path: Path) -> None:
    data = _crosswalk_copy(tmp_path)
    arm = _interface(data, "chassis_top_plate_to_arm_base")
    arm["mechanical_subassembly"]["expected"]["controller"] = "unexpected_controller"
    with pytest.raises(MechanicalDatumValidationError, match="mechanical subassembly.controller"):
        validate(_write_crosswalk(tmp_path, data))


def test_crosswalk_cannot_be_marked_as_manufacturing_release(tmp_path: Path) -> None:
    data = _crosswalk_copy(tmp_path)
    data["manufacturing_release"]["released"] = True
    with pytest.raises(MechanicalDatumValidationError, match="outside manufacturing release"):
        validate(_write_crosswalk(tmp_path, data))
