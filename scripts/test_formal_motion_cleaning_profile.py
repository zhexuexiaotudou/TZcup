from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_formal_motion_cleaning_profile import (
    CLEANING_XACRO,
    CONTROL_XACRO,
    PROFILE,
    FormalMotionCleaningProfileError,
    validate_profile,
)


def _mutated_profile(tmp_path: Path, mutate) -> Path:
    data = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    mutate(data)
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def test_formal_motion_cleaning_profile_matches_source_geometry() -> None:
    result = validate_profile()
    assert result["status"] == (
        "FORMAL_MOTION_CLEANING_PROFILE_SOURCE_CHECKS_PASSED_RUNTIME_CONTACT_PENDING"
    )
    assert result["wheel_radius_m"] == pytest.approx(0.1651)
    assert result["control_wheel_radius_m"] == pytest.approx(0.1625)
    assert result["planning_kinematic_constraint"] == (
        "curvature_limited_reference_path_for_skid_steer"
    )
    assert result["physical_steering_claim"] is False
    assert result["runtime_tracking_status"] == (
        "pending_skid_steer_tracking_validation"
    )
    assert result["arm_transport_start_positions_rad"] == {
        "shoulder_pan_joint": -1.0,
        "shoulder_lift_joint": -1.0,
        "elbow_joint": 1.8,
        "wrist_1_joint": -1.5,
        "wrist_2_joint": -1.55,
        "wrist_3_joint": 0.25,
    }
    assert result["working_pose"] == {
        "lift_m": 0.100,
        "squeegee_float_m": 0.0,
        "squeegee_pitch_rad": 0.0,
    }
    assert result["lift_coordinate_semantics"] == {
        "zero_position": "transport_safe_raised",
        "positive_direction": "downward",
        "transport_lift_m": 0.0,
        "work_lift_m": 0.100,
        "zero_pose_minimum_ground_clearance_m": pytest.approx(0.100),
    }
    assert result["expected_geometry_z_m"] == pytest.approx(
        {
            "side_brush_lowest": 0.0,
            "central_roller_lowest": 0.0,
            "squeegee_lowest": 0.0,
            "suction_nozzle_lowest": 0.005,
        },
        abs=1e-9,
    )
    assert result["physical_ground_contact_status"] == (
        "pending_runtime_contact_and_normal_force_validation"
    )


def test_rejects_passive_squeegee_exported_as_motor(tmp_path: Path) -> None:
    path = tmp_path / "control_interfaces.xacro"
    source = CONTROL_XACRO.read_text(encoding="utf-8")
    changed = source.replace(
        '      <xacro:hf_position_joint name="dry_deposit_gate_joint"',
        '      <xacro:hf_position_joint name="squeegee_float_joint" lower="-0.015" upper="0.015" velocity="0.10" effort="250.0"/>\n'
        '      <xacro:hf_position_joint name="dry_deposit_gate_joint"',
    )
    assert changed != source
    path.write_text(changed, encoding="utf-8")
    with pytest.raises(FormalMotionCleaningProfileError, match="passive joints cannot"):
        validate_profile(control_path=path)


def test_rejects_cleaning_lift_solver_release_clearance_as_a_commandable_stroke(
    tmp_path: Path,
) -> None:
    path = tmp_path / "control_interfaces.xacro"
    source = CONTROL_XACRO.read_text(encoding="utf-8")
    changed = source.replace(
        'name="cleaning_lift_joint" lower="0.0" upper="0.100"',
        'name="cleaning_lift_joint" lower="0.0" upper="0.10002"',
    )
    assert changed != source
    path.write_text(changed, encoding="utf-8")
    with pytest.raises(
        FormalMotionCleaningProfileError,
        match="commandable product stroke",
    ):
        validate_profile(control_path=path)


def test_rejects_gazebo_arm_start_pose_that_differs_from_transport_contract(tmp_path: Path) -> None:
    path = tmp_path / "control_interfaces.xacro"
    source = CONTROL_XACRO.read_text(encoding="utf-8")
    changed = source.replace(
        'name="shoulder_pan_joint" lower="${-2*pi}" upper="${2*pi}" velocity="3.14" effort="150.0" initial_position="-1.0"',
        'name="shoulder_pan_joint" lower="${-2*pi}" upper="${2*pi}" velocity="3.14" effort="150.0" initial_position="0.0"',
    )
    assert changed != source
    path.write_text(changed, encoding="utf-8")
    with pytest.raises(
        FormalMotionCleaningProfileError,
        match="shoulder_pan_joint initial/transport position differs",
    ):
        validate_profile(control_path=path)


def test_rejects_unverified_physical_contact_claim(tmp_path: Path) -> None:
    path = _mutated_profile(
        tmp_path,
        lambda data: data["claim_boundary"].update(
            {"physical_ground_contact_status": "passed"}
        ),
    )
    with pytest.raises(FormalMotionCleaningProfileError, match="must keep physical ground contact pending"):
        validate_profile(profile_path=path)


def test_rejects_named_side_brush_sweep_collision_that_drops_sdf_surface(
    tmp_path: Path,
) -> None:
    path = tmp_path / "cleaning_mechanism.xacro"
    source = CLEANING_XACRO.read_text(encoding="utf-8")
    changed = source.replace("      <collision>\n        <origin xyz=\"0 0 -0.065\"/>", "      <collision name=\"bristle_sweep_collision\">\n        <origin xyz=\"0 0 -0.065\"/>", 1)
    assert changed != source
    path.write_text(changed, encoding="utf-8")
    with pytest.raises(FormalMotionCleaningProfileError, match="must remain unnamed"):
        validate_profile(cleaning_path=path)


def test_rejects_virtual_ackermann_as_physical_steering(tmp_path: Path) -> None:
    path = _mutated_profile(
        tmp_path,
        lambda data: data["drive"]["virtual_ackermann_constraint"].update(
            {"represents_physical_steering": True}
        ),
    )
    with pytest.raises(FormalMotionCleaningProfileError, match="cannot claim physical steering"):
        validate_profile(profile_path=path)


def test_rejects_noncanonical_skid_steer_reference_constraint(tmp_path: Path) -> None:
    path = _mutated_profile(
        tmp_path,
        lambda data: data["drive"].update(
            {"canonical_planning_kinematic_constraint": "virtual_ackermann"}
        ),
    )
    with pytest.raises(FormalMotionCleaningProfileError, match="canonical name"):
        validate_profile(profile_path=path)
