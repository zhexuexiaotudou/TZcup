from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_a300_drivetrain_realism_contract import (
    A300DrivetrainContractError,
    DEFAULT_CONTRACT,
    validate,
)


def _mutated_contract(tmp_path: Path, mutate) -> Path:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    mutate(contract)
    output = tmp_path / "a300-mutated.yaml"
    output.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    return output


def test_committed_a300_drivetrain_integration_is_static_valid() -> None:
    result = validate()
    assert result["status"] == "A300_DRIVETRAIN_VEHICLE_INTEGRATION_STATIC_VALID"
    assert result["upstream_commit"] == "b0f6d920422ad302372a1c65e31d61648da884ed"
    assert result["mesh_count"] == 5
    assert result["published_mass_kg"] == 78.5
    assert result["allocated_mass_kg"] == 78.5
    assert result["runtime_integrated"] is True
    assert result["runtime_revalidation_pending"] is True


def test_invented_suspension_compliance_fails_closed(tmp_path: Path) -> None:
    contract = _mutated_contract(
        tmp_path,
        lambda data: data["future_rigid_body_chain"].update(
            {"no_compliance_joint_allowed": False}
        ),
    )
    with pytest.raises(A300DrivetrainContractError, match="suspension compliance"):
        validate(contract)


def test_unpublished_mass_cannot_be_relabelled_official(tmp_path: Path) -> None:
    contract = _mutated_contract(
        tmp_path,
        lambda data: data["mass_partition"].update({"source_class": "official_public"}),
    )
    with pytest.raises(A300DrivetrainContractError, match="engineering allocations"):
        validate(contract)


def test_mass_partition_must_preserve_published_total(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["mass_partition"]["allocations_kg"]["four_motor_assemblies"] += 1.0

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300DrivetrainContractError, match="conserve"):
        validate(contract)


def test_public_electrical_boundary_drift_fails_closed(tmp_path: Path) -> None:
    contract = _mutated_contract(
        tmp_path,
        lambda data: data["published_platform_boundaries"].update(
            {"continuous_battery_current_a": 80.0}
        ),
    )
    with pytest.raises(A300DrivetrainContractError, match="continuous_battery_current_a"):
        validate(contract)


def test_mesh_hash_drift_fails_closed(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["vendored_meshes"][0]["sha256"] = "0" * 64

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300DrivetrainContractError, match="mesh hash mismatch"):
        validate(contract)


def test_contract_cannot_revert_to_preintegration_state(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["plant_model"]["integration_boundary"]["loaded_by_current_vehicle"] = False

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300DrivetrainContractError, match="runtime wiring"):
        validate(contract)


def test_selected_odom_authority_cannot_drift(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        data["plant_model"]["integration_boundary"]["final_single_authority_plan"][
            "selected_odometry_publisher"
        ] = "/base_controller"

    contract = _mutated_contract(tmp_path, mutate)
    with pytest.raises(A300DrivetrainContractError, match="selected_odometry_publisher"):
        validate(contract)
