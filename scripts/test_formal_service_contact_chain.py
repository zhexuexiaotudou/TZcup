from __future__ import annotations

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description"
POWER = ROOT / "starter_ws" / "src" / "sanitation_power_system"
SAFETY = ROOT / "starter_ws" / "src" / "sanitation_safety"
SERVICE_XACRO = DESCRIPTION / "urdf" / "high_fidelity" / "power_service_hardware.xacro"
CONTROL_XACRO = DESCRIPTION / "urdf" / "high_fidelity" / "control_interfaces.xacro"
LAUNCH = DESCRIPTION / "launch" / "formal_vehicle_sim.launch.py"
NATIVE_CONTACT_BRIDGE = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_gazebo_control"
    / "src"
    / "FormalContactEvaluationNativeBridge.cc"
)
MANAGER = POWER / "sanitation_power_system" / "charge_interface_manager.py"
SIM_INPUTS = SAFETY / "sanitation_safety" / "simulation_safety_inputs.py"
REGISTER = ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_component_register.yaml"


def _gazebo_reference(root: ET.Element, link: str) -> ET.Element:
    match = root.find(f".//gazebo[@reference='{link}']")
    assert match is not None, link
    return match


def test_service_fittings_have_named_collisions_and_contact_sensors() -> None:
    root = ET.parse(SERVICE_XACRO).getroot()
    expected = {
        "charge_receptacle_link": (
            "charge_receptacle_contact_collision",
            "charge_receptacle_contact_sensor",
            "/formal_vehicle/gazebo/charge_receptacle/contact",
            "base_footprint_fixed_joint_lump__charge_receptacle_contact_collision_collision_49",
        ),
        "wastewater_drain_coupling_link": (
            "wastewater_drain_coupling_contact_collision",
            "wastewater_drain_coupling_contact_sensor",
            "/formal_vehicle/gazebo/wastewater_drain_coupling/contact",
            "base_footprint_fixed_joint_lump__wastewater_drain_coupling_contact_collision_collision_120",
        ),
    }
    for link_name, (collision_name, sensor_name, topic, expanded_collision) in expected.items():
        link = root.find(f".//link[@name='{link_name}']")
        assert link is not None
        assert link.find(f"collision[@name='{collision_name}']") is not None
        gazebo = _gazebo_reference(root, link_name)
        assert gazebo.findtext("preserveFixedJoint") == "true"
        sensor = gazebo.find(f"sensor[@name='{sensor_name}']")
        assert sensor is not None
        assert sensor.get("type") == "contact"
        assert sensor.findtext("topic") is None
        assert sensor.findtext("contact/topic") == topic
        assert sensor.findtext("always_on") == "true"
        assert float(sensor.findtext("update_rate", "0")) >= 50.0
        # The fixed service links are folded into base_footprint by sdformat;
        # this must stay aligned with the final expanded-SDF collision name.
        assert sensor.findtext("contact/collision") == expanded_collision


def test_default_launch_uses_one_native_contact_bridge_per_product_raw_topic() -> None:
    native_source = NATIVE_CONTACT_BRIDGE.read_text(encoding="utf-8")
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    expected = {
        "charge_receptacle_contact_bridge": (
            "charge_receptacle",
            "/formal_vehicle/gazebo/charge_receptacle/contact",
            "/formal_vehicle/service/raw/charge_plug_contact"
        ),
        "wastewater_drain_contact_bridge": (
            "wastewater_drain",
            "/formal_vehicle/gazebo/wastewater_drain_coupling/contact",
            "/formal_vehicle/service/raw/drain_hose_contact"
        ),
    }
    contact_nodes = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            continue
        if call.func.id != "Node":
            continue
        keywords = {item.arg: item.value for item in call.keywords if item.arg}
        name = keywords.get("name")
        if not isinstance(name, ast.Constant) or name.value not in expected:
            continue
        contact_nodes[name.value] = keywords

    assert set(contact_nodes) == set(expected)
    assert "GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>" in native_source
    all_remaps = []
    for name, (group, gazebo_topic, ros_topic) in expected.items():
        keywords = contact_nodes[name]
        assert ast.literal_eval(keywords["package"]) == "sanitation_gazebo_control"
        assert ast.literal_eval(keywords["executable"]) == "formal_contact_evaluation_native_bridge"
        assert ast.literal_eval(keywords["parameters"]) == [{"endpoint_group": group}]
        remappings = ast.literal_eval(keywords["remappings"])
        assert remappings == [(gazebo_topic, ros_topic)]
        assert f'"{gazebo_topic}"' in native_source
        all_remaps.extend(remappings)
        assert [source for source, target in all_remaps if target == ros_topic] == [gazebo_topic]


def test_every_parameter_bridge_has_a_unique_stable_node_name() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"), filename=str(LAUNCH))
    names: list[str] = []
    bridge_count = 0
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "Node":
            continue
        keywords = {item.arg: item.value for item in node.keywords if item.arg}
        package = keywords.get("package")
        if not isinstance(package, ast.Constant) or package.value != "ros_gz_bridge":
            continue
        bridge_count += 1
        name = keywords.get("name")
        assert isinstance(name, ast.Constant) and isinstance(name.value, str)
        names.append(name.value)
    # Service Contacts use two native bridges; retain a guard for the remaining
    # ros_gz parameter bridges without requiring the retired service bridges.
    assert bridge_count >= 5
    assert len(names) == len(set(names))


def test_charge_manager_uses_nonempty_contact_and_sensor_qos() -> None:
    source = MANAGER.read_text(encoding="utf-8")
    assert "from ros_gz_interfaces.msg import Contacts" in source
    assert "from rclpy.qos import qos_profile_sensor_data" in source
    assert '"/formal_vehicle/service/raw/charge_plug_contact"' in source
    assert 'self._values["plug_present"] = bool(message.contacts)' in source
    assert 'self._times["plug_present"] = time.monotonic()' in source
    assert 'plug_present = bool(self._values["plug_present"] and plug_contact_fresh)' in source
    assert '"plug_contact_fresh": plug_contact_fresh' in source
    assert "/formal_vehicle/power/charge_plug_present" not in source
    assert "/formal_vehicle/power/charge_plug_present" not in SIM_INPUTS.read_text(
        encoding="utf-8"
    )


def test_contact_message_dependency_and_passive_joint_state_interfaces_exist() -> None:
    package = ET.parse(POWER / "package.xml").getroot()
    exec_dependencies = {item.text for item in package.findall("exec_depend")}
    assert "ros_gz_interfaces" in exec_dependencies
    control = CONTROL_XACRO.read_text(encoding="utf-8")
    for joint in (
        "charge_port_door_hinge_joint",
        "charge_connector_lock_joint",
        "wastewater_drain_service_cap_joint",
    ):
        assert f'<xacro:hf_state_only_joint name="{joint}"/>' in control


def test_component_register_uses_contact_contracts_not_synthetic_plug_boolean() -> None:
    register = yaml.safe_load(REGISTER.read_text(encoding="utf-8"))
    contracts = register["topic_contracts"]
    assert "charge_plug_present" not in contracts
    assert contracts["charge_receptacle_contact"] == {
        "transport": "ros_native",
        "direction": "subscription",
        "ros_topic": "/formal_vehicle/service/raw/charge_plug_contact",
        "ros_type": "ros_gz_interfaces/msg/Contacts",
        "source_path": "starter_ws/src/sanitation_power_system/sanitation_power_system/charge_interface_manager.py",
    }
    assert contracts["wastewater_drain_hose_contact"]["ros_topic"] == (
        "/formal_vehicle/service/raw/drain_hose_contact"
    )
