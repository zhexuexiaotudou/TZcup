from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from validate_formal_vehicle_urdf import (
    DEFAULT_CONTRACT,
    DEFAULT_LAYOUT,
    FormalVehicleValidationError,
    validate_expanded_urdf,
    validate_layout,
)


ROOT = Path(__file__).resolve().parents[1]


def _write_layout(tmp_path: Path, mutate) -> Path:
    layout = yaml.safe_load(DEFAULT_LAYOUT.read_text(encoding="utf-8"))
    mutate(layout)
    path = tmp_path / "layout.yaml"
    path.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")
    return path


def _inertial(mass: float = 1.0) -> str:
    return f"""
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="{mass}"/>
        <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
      </inertial>"""


def _expanded_fixture(tmp_path: Path, *, mutate: str = "") -> Path:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    layout = yaml.safe_load(DEFAULT_LAYOUT.read_text(encoding="utf-8"))
    links = list(contract["frame_contract"]["required_frames"])
    for virtual_frame in layout["validation_policy"]["inertial_exempt_virtual_frames"]:
        if virtual_frame not in links:
            links.append(virtual_frame)
    for installation_frame in layout["installation_frames"]:
        if installation_frame not in links:
            links.append(installation_frame)
    required_joint_specs = list(contract["joint_contract"])
    joint_children = {item["name"].replace("_joint", "_link") for item in required_joint_specs}
    # These joint child / parent frames are implementation details, so add them
    # to the minimal expanded-URDF fixture explicitly.
    for child in (
        "dry_bin_lid_link",
        "wastewater_lid_link",
        "cleaning_lift_carriage_link",
        "dry_deposit_hopper_link",
        "dry_deposit_gate_link",
        "dry_bin_latch_base_link",
        "dry_bin_latch_link",
        "wastewater_lid_latch_base_link",
        "wastewater_lid_latch_link",
    ):
        if child not in links:
            links.append(child)

    link_xml = "\n".join(f'  <link name="{name}">{_inertial()}</link>' for name in links)
    for virtual_frame in layout["validation_policy"]["inertial_exempt_virtual_frames"]:
        link_xml = link_xml.replace(
            f'  <link name="{virtual_frame}">{_inertial()}</link>',
            f'  <link name="{virtual_frame}"/>',
        )
    used_children: set[str] = set()
    joint_xml: list[str] = []
    joint_by_name = {item["name"]: item for item in required_joint_specs}
    explicit_children = {
        "left_side_brush_joint": "left_side_brush_link",
        "right_side_brush_joint": "right_side_brush_link",
        "central_roller_joint": "central_roller_link",
        "cleaning_lift_joint": "cleaning_lift_carriage_link",
        "squeegee_pitch_joint": "squeegee_link",
        "squeegee_float_joint": "suction_nozzle_link",
        "dry_bin_lid_joint": "dry_bin_lid_link",
        "dry_bin_latch_joint": "dry_bin_latch_link",
        "dry_deposit_gate_joint": "dry_deposit_gate_link",
        "wastewater_lid_joint": "wastewater_lid_link",
        "wastewater_lid_latch_joint": "wastewater_lid_latch_link",
    }
    explicit_parents = {
        "left_side_brush_joint": "base_footprint",
        "right_side_brush_joint": "base_footprint",
        "central_roller_joint": "base_footprint",
        "cleaning_lift_joint": "base_footprint",
        "squeegee_pitch_joint": "base_footprint",
        "squeegee_float_joint": "squeegee_link",
        "dry_bin_lid_joint": "dry_bin_link",
        "dry_bin_latch_joint": "dry_bin_latch_base_link",
        "dry_deposit_gate_joint": "dry_deposit_hopper_link",
        "wastewater_lid_joint": "wastewater_tank_link",
        "wastewater_lid_latch_joint": "wastewater_lid_latch_base_link",
    }
    for name, spec in joint_by_name.items():
        child = explicit_children[name]
        parent = explicit_parents[name]
        used_children.add(child)
        kind = spec["type"]
        if child in layout["installation_frames"]:
            child_pose = layout["installation_frames"][child]
            child_xyz = list(child_pose["xyz_m"])
            child_rpy = list(child_pose["rpy_rad"])
            if parent in layout["installation_frames"]:
                parent_xyz = layout["installation_frames"][parent]["xyz_m"]
                # Fixture parents used here have zero rotation, so subtraction
                # is sufficient and keeps the fixture readable.
                child_xyz = [child_xyz[axis] - parent_xyz[axis] for axis in range(3)]
        else:
            child_xyz = [0.0, 0.0, 0.0]
            child_rpy = [0.0, 0.0, 0.0]
        origin = ' '.join(str(value) for value in child_xyz)
        rpy = ' '.join(str(value) for value in child_rpy)
        if kind == "continuous":
            limit = '<limit effort="20" velocity="30"/>'
        elif kind == "prismatic":
            limit = '<limit lower="-0.015" upper="0.10" effort="500" velocity="0.2"/>'
        else:
            limit = '<limit lower="-0.18" upper="1.58" effort="50" velocity="1"/>'
        joint_xml.append(
            f'  <joint name="{name}" type="{kind}"><parent link="{parent}"/>'
            f'<child link="{child}"/><origin xyz="{origin}" rpy="{rpy}"/>'
            f'<axis xyz="0 0 1"/>{limit}</joint>'
        )

    # Connect all remaining links into a single tree without multiple parents.
    for index, child in enumerate(links):
        if child == "base_footprint" or child in used_children:
            continue
        parent = "base_footprint"
        if child in layout["installation_frames"]:
            pose = layout["installation_frames"][child]
            xyz = ' '.join(str(value) for value in pose["xyz_m"])
            rpy = ' '.join(str(value) for value in pose["rpy_rad"])
        else:
            xyz = "0 0 0"
            rpy = "0 0 0"
        joint_xml.append(
            f'  <joint name="fixture_fixed_{index}" type="fixed"><parent link="{parent}"/>'
            f'<child link="{child}"/><origin xyz="{xyz}" rpy="{rpy}"/></joint>'
        )
    raw = (
        '<robot name="tzcup_formal_sanitation_vehicle">\n'
        + link_xml
        + "\n"
        + "\n".join(joint_xml)
        + "\n</robot>\n"
    )
    if mutate:
        raw = raw.replace(*mutate.split("|||", 1))
    path = tmp_path / "expanded.urdf"
    path.write_text(raw, encoding="utf-8")
    return path


def test_committed_layout_report_is_deterministic_and_fail_closed() -> None:
    result = validate_layout()
    report = json.loads(
        (ROOT / "reports" / "engineering" / "formal_vehicle_layout_report.json").read_text(encoding="utf-8")
    )
    assert result == report
    assert result["status"] == (
        "LAYOUT_CONTRACT_VALID_RESOLVED_ENGINEERING_GATES_WITH_DECLARED_EXTERNAL_PENDING"
    )
    assert result["dry_bin_usable_l"] >= 40.0
    assert result["preliminary_wastewater"]["final_capacity_frozen"] is True
    assert result["preliminary_wastewater"]["cog_limit_l"] == pytest.approx(14.0)
    assert result["preliminary_wastewater"]["usable_l"] == pytest.approx(8.30)
    assert "full_inertia_and_cog_scan" not in result["pending_external_gates"]
    assert "exact_s100p_board_envelope_and_mounting_pattern" in result["pending_external_gates"]
    assert result["compute_hardware"]["provisional_reference_dimensions_m"] == [0.121, 0.120, 0.0524]
    assert result["compute_hardware"]["modeled_collision_dimensions_m"] == [0.121, 0.120, 0.0524]
    assert result["compute_hardware"]["external_envelope_frozen"] is False


def test_committed_expanded_urdf_report_is_deterministic() -> None:
    urdf = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"
    result = validate_expanded_urdf(urdf)
    report = json.loads(
        (ROOT / "reports" / "engineering" / "formal_vehicle_urdf_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert result == report
    # Keep this exact so adding or dropping a physical subassembly cannot
    # silently change the frozen product model.  The explicit A300 drivetrain
    # modules, encoders, S100 mount and service hardware bring the current
    # expanded vehicle to 196 links.
    assert result["urdf_validation"]["link_count"] == 196
    assert result["urdf_validation"]["joint_count"] == 195
    assert result["urdf_validation"]["total_empty_vehicle_mass_kg"] == pytest.approx(160.007583)
    assert result["urdf_validation"]["static_frame_pose_consistency"]["checked_count"] == 42


def test_runtime_report_is_evidence_backed_and_fail_closed() -> None:
    report = json.loads(
        (ROOT / "reports" / "engineering" / "formal_vehicle_runtime_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert report["status"] == "FORMAL_GAZEBO_CONTROL_AND_SENSOR_RUNTIME_PASSED_EXTERNAL_FIDELITY_GATES_PENDING"
    assert all(report["passed_checks"].values())
    assert report["passed_checks"]["dry_payload_clamp_kg"] == 1.512
    assert report["passed_checks"]["wastewater_payload_clamp_kg"] == 8.30
    assert report["report_id"] == "tzcup_formal_vehicle_headless_runtime_v4"
    assert report["passed_checks"]["a300_plant_loaded_exactly_once"] is True
    assert report["passed_checks"]["removed_base_controller_absent"] is True
    assert report["passed_checks"]["raw_plant_odometry_frames_correct"] is True
    assert report["passed_checks"]["all_camera_info_intrinsics_valid"] is True
    assert report["controller_states"] == {
        "arm_controller": "active",
        "brush_controller": "inactive",
        "cleaning_controller": "active",
        "gripper_controller": "active",
        "joint_state_broadcaster": "active",
        "recovery_controller": "inactive",
        "service_controller": "active",
        "storage_controller": "active",
    }
    assert all(count > 0 for count in report["sample_counts"].values())
    assert {
        "/sensors/front_rgbd/depth/image_rect_raw/image",
        "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
        "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
        "/sensors/wrist_rgbd/depth/image_rect_raw/image",
        "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image",
        "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info",
        "/sensors/rear_left_fisheye/image_raw",
        "/sensors/rear_left_fisheye/camera_info",
        "/sensors/rear_right_fisheye/image_raw",
        "/sensors/rear_right_fisheye/camera_info",
    } <= set(report["sample_counts"])
    expected_hash = hashlib.sha256(
        (ROOT / "reports/engineering/formal_competition_vehicle.urdf").read_bytes()
    ).hexdigest()
    assert report["current_source_reconciliation"]["expanded_urdf_sha256"] == expected_hash


def test_minimal_expanded_urdf_fixture_passes_deterministic_checks(tmp_path: Path) -> None:
    result = validate_expanded_urdf(_expanded_fixture(tmp_path))
    assert result["urdf_validation"]["passed"] is True
    assert result["urdf_validation"]["all_physical_links_have_positive_mass_and_positive_definite_physical_inertia"] is True
    assert result["urdf_validation"]["massless_virtual_frames"] == ["base_footprint", "ur5e_base_link"]
    assert result["urdf_validation"]["static_frame_pose_consistency"]["checked_count"] == 38
    assert result["status"] == "FORMAL_URDF_DETERMINISTIC_CHECKS_PASSED_EXTERNAL_GATES_PENDING"
    assert result["pending_external_gates"]


def test_rejects_missing_link_inertial(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    raw = urdf.read_text(encoding="utf-8").replace(
        f'<link name="imu_link">{_inertial()}</link>',
        '<link name="imu_link"><!-- inertial removed --></link>',
    )
    urdf.write_text(raw, encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="link imu_link has no inertial"):
        validate_expanded_urdf(urdf)


def test_rejects_inertial_on_virtual_root(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    raw = urdf.read_text(encoding="utf-8").replace(
        '<link name="base_footprint"/>',
        f'<link name="base_footprint">{_inertial()}</link>',
    )
    urdf.write_text(raw, encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="virtual frame base_footprint must not carry inertial"):
        validate_expanded_urdf(urdf)


def test_rejects_non_positive_definite_inertia(tmp_path: Path) -> None:
    urdf = _expanded_fixture(
        tmp_path,
        mutate='ixx="0.02" ixy="0"|||ixx="0" ixy="0"',
    )
    with pytest.raises(FormalVehicleValidationError, match="not positive definite"):
        validate_expanded_urdf(urdf)


def test_rejects_duplicate_link_name(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    raw = urdf.read_text(encoding="utf-8").replace(
        "</robot>", f'<link name="imu_link">{_inertial()}</link></robot>'
    )
    urdf.write_text(raw, encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="duplicate link names"):
        validate_expanded_urdf(urdf)


def test_rejects_symbolic_placeholder_in_expanded_urdf(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    urdf.write_text(urdf.read_text(encoding="utf-8").replace('value="1.0"', 'value="${mass}"', 1), encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="symbolic placeholder"):
        validate_expanded_urdf(urdf)


def test_ignores_symbolic_xacro_examples_inside_xml_comments(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    raw = urdf.read_text(encoding="utf-8")
    urdf.write_text(
        raw.replace(
            "<robot ",
            "<!-- Documented Xacro example: ${side}_collision -->\n<robot ",
            1,
        ),
        encoding="utf-8",
    )
    result = validate_expanded_urdf(urdf)
    assert result["status"] == (
        "FORMAL_URDF_DETERMINISTIC_CHECKS_PASSED_EXTERNAL_GATES_PENDING"
    )


def test_rejects_missing_required_joint(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    raw = urdf.read_text(encoding="utf-8")
    start = raw.index('  <joint name="central_roller_joint"')
    end = raw.index("</joint>", start) + len("</joint>")
    urdf.write_text(raw[:start] + raw[end:], encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="required URDF joints missing"):
        validate_expanded_urdf(urdf)


def test_rejects_inverted_joint_limit(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    raw = urdf.read_text(encoding="utf-8").replace(
        'lower="-0.18" upper="1.58"', 'lower="1.58" upper="-0.18"', 1
    )
    urdf.write_text(raw, encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="inverted or empty limits"):
        validate_expanded_urdf(urdf)


def test_rejects_static_frame_position_drift(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    raw = urdf.read_text(encoding="utf-8").replace(
        'xyz="0.535 0.0 1.1621"', 'xyz="0.555 0.0 1.1621"', 1
    )
    urdf.write_text(raw, encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="installation frame lidar_2d_link differs"):
        validate_expanded_urdf(urdf)


def test_rejects_static_frame_rotation_drift(tmp_path: Path) -> None:
    urdf = _expanded_fixture(tmp_path)
    marker = 'xyz="0.535 0.0 1.1621" rpy="0.0 0.0 0.0"'
    raw = urdf.read_text(encoding="utf-8").replace(
        marker, 'xyz="0.535 0.0 1.1621" rpy="0.0 0.0 0.10"', 1
    )
    assert raw != urdf.read_text(encoding="utf-8")
    urdf.write_text(raw, encoding="utf-8")
    with pytest.raises(FormalVehicleValidationError, match="installation frame lidar_2d_link differs"):
        validate_expanded_urdf(urdf)


def test_rejects_dry_bin_below_40_l(tmp_path: Path) -> None:
    path = _write_layout(tmp_path, lambda data: data["storage"]["dry_bin"].update({"usable_volume_l": 39.0}))
    with pytest.raises(FormalVehicleValidationError, match="dry-bin usable volume"):
        validate_layout(path)


def test_rejects_cleaning_width_below_requirement(tmp_path: Path) -> None:
    path = _write_layout(
        tmp_path,
        lambda data: data["cleaning_geometry"].update({"effective_working_width_m": 0.59}),
    )
    with pytest.raises(FormalVehicleValidationError, match="below 0.6 m"):
        validate_layout(path)


def test_rejects_false_final_wastewater_capacity_claim(tmp_path: Path) -> None:
    def mutate(data):
        data["storage"]["wastewater_tank"]["cog_limit_l"] = 10.0
        data["storage"]["wastewater_tank"]["final_capacity_pending"] = False

    path = _write_layout(tmp_path, mutate)
    with pytest.raises(FormalVehicleValidationError, match="does not match"):
        validate_layout(path)


def test_rejects_failed_approximate_fov_gate(tmp_path: Path) -> None:
    def mutate(data):
        data["sensor_layout"][0]["approximate_clear_fraction"] = 0.20

    path = _write_layout(tmp_path, mutate)
    with pytest.raises(FormalVehicleValidationError, match="fails the approximate FOV"):
        validate_layout(path)
