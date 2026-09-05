from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
STORAGE_XACRO = (
    ROOT
    / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro"
)
CONTROL_XACRO = (
    ROOT
    / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro"
)
REGISTER = ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
MESH_ROOT = (
    ROOT / "starter_ws/src/sanitation_vehicle_description/meshes/project/storage"
)


def _joint(root: ET.Element, name: str) -> ET.Element:
    joint = root.find(f".//joint[@name='{name}']")
    assert joint is not None, name
    return joint


def _endpoint(joint: ET.Element, tag: str) -> str:
    endpoint = joint.find(tag)
    assert endpoint is not None
    return endpoint.attrib["link"]


def test_dry_and_wet_lids_have_explicit_manual_over_center_latches() -> None:
    root = ET.parse(STORAGE_XACRO).getroot()
    cases = {
        "dry_bin": ("dry_bin_link", "dry_bin_lid_link"),
        "wastewater_lid": ("wastewater_tank_link", "wastewater_lid_link"),
    }
    for prefix, (tank, lid) in cases.items():
        latch = _joint(root, f"{prefix}_latch_joint")
        assert latch.attrib["type"] == "revolute"
        assert _endpoint(latch, "parent") == f"{prefix}_latch_base_link"
        assert _endpoint(latch, "child") == f"{prefix}_latch_link"
        assert latch.find("axis").attrib["xyz"] == "0 1 0"
        limit = latch.find("limit")
        assert limit is not None
        assert float(limit.attrib["lower"]) == 0.0
        assert float(limit.attrib["upper"]) == 1.221730476

        base = _joint(root, f"{prefix}_latch_base_joint")
        assert base.attrib["type"] == "fixed"
        assert _endpoint(base, "parent") == tank
        assert _endpoint(base, "child") == f"{prefix}_latch_base_link"

        keeper = _joint(root, f"{prefix}_latch_keeper_joint")
        assert keeper.attrib["type"] == "fixed"
        assert _endpoint(keeper, "parent") == lid
        assert _endpoint(keeper, "child") == f"{prefix}_latch_keeper_link"


def test_storage_service_joints_are_state_only_not_fake_powered_actuators() -> None:
    root = ET.parse(CONTROL_XACRO).getroot()
    source = CONTROL_XACRO.read_text(encoding="utf-8")
    for name in (
        "dry_bin_lid_joint",
        "dry_bin_latch_joint",
        "wastewater_lid_joint",
        "wastewater_lid_latch_joint",
    ):
        assert f'<xacro:hf_state_only_joint name="{name}"/>' in source
        assert f'<xacro:hf_position_joint name="{name}"' not in source
    assert root is not None
    assert (
        '<xacro:hf_position_joint name="wastewater_drain_valve_joint" '
        'lower="0.0" upper="${pi/2.0}" velocity="1.2" effort="25.0" '
        'initial_position="0.00002"/>'
    ) in source


def test_storage_register_binds_latches_presence_sensor_and_existing_drain() -> None:
    data = yaml.safe_load(REGISTER.read_text(encoding="utf-8"))
    positions = {item["id"]: item for item in data["functional_positions"]}
    assemblies = {item["id"]: item for item in data["mechanical_subassemblies"]}

    dry_deposition = positions["dry_deposition"]
    assert dry_deposition["presence_sensor_link"] == "dry_deposit_presence_sensor_link"
    assert dry_deposition["presence_measurement"] == "physical_chute_contact_event"
    assert "dry_deposit_contact" in dry_deposition["required_topic_contracts"]
    presence = {item["id"]: item for item in dry_deposition["components"]}[
        "dry_deposit_presence_sensor"
    ]
    assert presence["link"] == "dry_deposit_presence_sensor_link"
    assert presence["joint"] == "dry_deposit_presence_sensor_joint"

    for position_id, latch_joint in (
        ("dry_storage", "dry_bin_latch_joint"),
        ("wet_storage", "wastewater_lid_latch_joint"),
    ):
        storage = positions[position_id]
        assert latch_joint in storage["required_passive_joints"]
        assert storage["lock_state"] == "lid_zero_rad_and_latch_zero_rad"
        assert storage["service_sequence"] == (
            "rotate_latch_to_released_stop_then_open_lid"
        )
    assert positions["wet_storage"]["maintenance_drain_position"] == "wastewater_drain"
    assert positions["wastewater_drain"]["required_joints"] == [
        "wastewater_drain_valve_joint"
    ]
    assert assemblies["dry_storage"]["passive_joints"] == [
        "dry_bin_lid_joint",
        "dry_bin_latch_joint",
    ]
    assert assemblies["wastewater_storage"]["passive_joints"] == [
        "wastewater_lid_joint",
        "wastewater_lid_latch_joint",
    ]
    assert assemblies["robot_deposition_port"]["required_topic_contracts"] == [
        "dry_deposit_contact"
    ]


def test_toggle_latch_meshes_are_deterministic_registered_assets() -> None:
    manifest = (
        ROOT / "starter_ws/src/sanitation_vehicle_description/meshes/MANIFEST.sha256"
    ).read_text(encoding="utf-8")
    for name in (
        "storage_toggle_latch_base.stl",
        "storage_toggle_latch_handle.stl",
        "storage_toggle_latch_keeper.stl",
    ):
        path = MESH_ROOT / name
        assert path.is_file()
        assert path.stat().st_size > 84
        assert f"project/storage/{name}" in manifest
