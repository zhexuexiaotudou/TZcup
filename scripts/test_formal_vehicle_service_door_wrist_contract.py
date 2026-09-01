#!/usr/bin/env python3
"""Source-level checks for service-door and wrist RGB-D mechanics."""

from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description"
BODYWORK = DESCRIPTION / "urdf" / "high_fidelity" / "bodywork.xacro"
SENSORS = DESCRIPTION / "urdf" / "high_fidelity" / "sensor_suite.xacro"
CONTROL = DESCRIPTION / "urdf" / "high_fidelity" / "control_interfaces.xacro"
REGISTER = ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_component_register.yaml"
GENERATOR = DESCRIPTION / "cad" / "formal_vehicle" / "generate_product_bodywork_meshes.py"


DOORS = (
    "bodywork_power_service_door",
    "bodywork_compute_service_door",
    "bodywork_wet_service_door",
    "bodywork_rear_dry_service_door",
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_four_service_doors_are_limited_latched_mechanisms() -> None:
    bodywork = _read(BODYWORK)
    control = _read(CONTROL)

    assert bodywork.count("<xacro:hf_service_door name=") == 4
    assert "hf_fixed_body_panel" not in bodywork
    assert "service_door_hinge_barrel.stl" in bodywork
    assert "service_door_rotary_latch.stl" in bodywork
    assert 'name="${name}_hinge_joint" type="revolute"' in bodywork
    assert 'name="${name}_latch_joint" type="revolute"' in bodywork
    assert '<axis xyz="0 0 1"/>' in bodywork
    for door in DOORS:
        assert f'name="{door}"' in bodywork
        assert f'name="{door}_hinge_joint"' in control
        assert f'name="{door}_latch_joint"' in control


def test_service_door_cad_is_hinge_local_and_has_hardware() -> None:
    generator = _read(GENERATOR)

    assert "def extruded_polygon_yz" in generator
    for mesh in (
        "power_service_door.stl",
        "compute_service_door.stl",
        "wet_service_door.stl",
        "rear_dry_service_door.stl",
        "service_door_hinge_barrel.stl",
        "service_door_rotary_latch.stl",
    ):
        assert f'"{mesh}"' in generator


def test_wrist_bracket_camera_and_optical_frame_are_separate_links() -> None:
    sensors = _read(SENSORS)

    assert '<link name="${name}_mount_link">' in sensors
    assert '<joint name="${name}_bracket_joint" type="fixed">' in sensors
    assert '<child link="${name}_mount_link"/>' in sensors
    assert '<joint name="${name}_mount_joint" type="fixed">' in sensors
    assert '<parent link="${name}_mount_link"/><child link="${name}_link"/>' in sensors
    assert 'name="${name}_depth_optical_frame" parent="${name}_link"' in sensors
    assert 'name="wrist_rgbd" parent="${wrist_parent}"' in sensors
    assert 'mount_bracket="true"' in sensors
    assert "Every load-carrying bracket member" in sensors
    assert '<mass value="0.236709"/>' in sensors


def test_rear_fisheye_uses_native_equisolid_projection() -> None:
    sensors = _read(SENSORS)

    assert 'type="wideanglecamera"' in sensors
    assert sensors.count('type="wideanglecamera"') == 1
    assert "<type>equisolid_angle</type>" in sensors
    assert "<scale_to_hfov>true</scale_to_hfov>" in sensors
    assert "theta^9 Taylor representation" in sensors


def test_register_matches_mechanical_parent_truth() -> None:
    register = yaml.safe_load(_read(REGISTER))
    subassemblies = {
        item["id"]: item for item in register["mechanical_subassemblies"]
    }

    assert subassemblies["dry_storage"]["parent_link"] == "storage_system_mount_link"
    assert subassemblies["wastewater_storage"]["parent_link"] == "storage_system_mount_link"
    assert subassemblies["wrist_rgbd_installation"]["parent_link"] == "tool0"
    assert subassemblies["wrist_rgbd_installation"]["root_link"] == "wrist_rgbd_mount_link"
    assert subassemblies["wrist_rgbd_installation"]["pregrasp_depth_fov_clear_fraction"] >= 0.95
    assert subassemblies["wrist_rgbd_installation"]["pregrasp_cube_visible_fraction"] >= 0.95
    assert subassemblies["wrist_rgbd_installation"]["bracket_mass_kg"] == 0.236709
    service = subassemblies["bodywork_service_access"]
    assert service["parent_link"] == "base_link"
    assert len(service["root_links"]) == 4
    assert set(service["passive_joints"]) == {
        f"{door}_{suffix}" for door in DOORS for suffix in ("hinge_joint", "latch_joint")
    }


def test_register_exposes_unlock_then_open_service_semantics() -> None:
    register = yaml.safe_load(_read(REGISTER))
    position = next(
        item for item in register["functional_positions"] if item["id"] == "bodywork_service_access"
    )

    assert position["actuation"] == "passive_service_interlock"
    assert position["lock_state"] == "all_latches_zero_rad"
    assert position["service_sequence"] == "unlock_corresponding_latch_then_open_hinge_within_limit"
    assert len(position["components"]) == 12
