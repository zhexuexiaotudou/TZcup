from __future__ import annotations

"""Fail-closed source/launch contract for the nine formal-water native bridges."""

import ast
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from generate_formal_vehicle_snapshot import authoritative_source_paths


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
CONTROL = ROOT / "starter_ws/src/sanitation_gazebo_control"


@dataclass(frozen=True, order=True)
class Endpoint:
    source: str
    group: str | None
    direction: str
    gazebo_topic: str
    ros_type: str
    gazebo_type: str


# This is the former, independently fixed parameter_bridge contract.  The
# final field is the ROS name after the former launch remapping has applied.
# Do not derive this table from the current launch: it is the migration oracle.
RAW_OLD_CONTRACT = """
cleaning_actuator_scalar_bridge|R2G|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/lift_position|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/lift_position
cleaning_actuator_scalar_bridge|R2G|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/enable|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/enable
cleaning_actuator_scalar_bridge|R2G|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/reset_faults|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/reset_faults
cleaning_actuator_scalar_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active
cleaning_actuator_scalar_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/total_current_a|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/total_current_a
cleaning_actuator_scalar_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/total_power_w|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/cleaning_motors/total_power_w
formal_auxiliary_bridge|R2G|/formal_vehicle/lighting/work_lights_on|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/lighting/work_lights_on
formal_auxiliary_bridge|R2G|/formal_vehicle/lighting/tail_lights_on|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/lighting/tail_lights_on
formal_auxiliary_bridge|R2G|/formal_vehicle/lighting/warning_lights_on|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/lighting/warning_lights_on
formal_auxiliary_bridge|R2G|/formal_vehicle/simulation/command/emergency_stop|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/simulation/command/emergency_stop
formal_auxiliary_bridge|R2G|/formal_vehicle/simulation/command/emergency_stop_plunger_pressed|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/simulation/command/emergency_stop_plunger_pressed
formal_auxiliary_bridge|R2G|/formal_vehicle/simulation/command/emergency_stop_reset|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/simulation/command/emergency_stop_reset
formal_auxiliary_bridge|R2G|/formal_vehicle/power/branches/safety/enabled|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/power/branches/safety/enabled
formal_auxiliary_bridge|R2G|/formal_vehicle/simulation/command/main_power|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/simulation/command/main_power
formal_auxiliary_bridge|R2G|/formal_vehicle/power/main_contactor_command|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/power/main_contactor_command
formal_auxiliary_bridge|G2R|/emergency_stop|std_msgs/msg/Bool|gz.msgs.Boolean|/emergency_stop
formal_auxiliary_bridge|G2R|/formal_vehicle/power/main_isolator_closed|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/power/main_isolator_closed
formal_auxiliary_bridge|G2R|/formal_vehicle/power/main_contactor_closed|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/power/main_contactor_closed
formal_auxiliary_bridge|G2R|/formal_vehicle/lighting/work_lights_applied|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/lighting/work_lights_applied
formal_auxiliary_bridge|G2R|/formal_vehicle/lighting/tail_lights_applied|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/lighting/tail_lights_applied
formal_auxiliary_bridge|G2R|/formal_vehicle/lighting/warning_lights_applied|std_msgs/msg/Bool|gz.msgs.Boolean|/formal_vehicle/lighting/warning_lights_applied
formal_squeegee_evaluation_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_position_m|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_position_m
formal_squeegee_evaluation_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_velocity_m_s|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_velocity_m_s
formal_squeegee_evaluation_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_force_n|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_force_n
formal_squeegee_evaluation_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_position_rad|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_position_rad
formal_squeegee_evaluation_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_velocity_rad_s|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_velocity_rad_s
formal_squeegee_evaluation_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_torque_nm|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_torque_nm
formal_squeegee_evaluation_bridge|G2R|/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/squeegee_link/sensor/squeegee_blade_ground_contact/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/cleaning/squeegee/contact
formal_brush_contact_evaluation_bridge|G2R|/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/left_side_brush_link/sensor/left_side_brush_ground_contact/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/cleaning/left_side_brush/contact
formal_brush_contact_evaluation_bridge|G2R|/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/right_side_brush_link/sensor/right_side_brush_ground_contact/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/cleaning/right_side_brush/contact
formal_brush_contact_evaluation_bridge|G2R|/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/central_roller_link/sensor/central_roller_ground_contact/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/cleaning/central_roller/contact
charge_receptacle_contact_bridge|G2R|/formal_vehicle/gazebo/charge_receptacle/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/formal_vehicle/service/raw/charge_plug_contact
wastewater_drain_contact_bridge|G2R|/formal_vehicle/gazebo/wastewater_drain_coupling/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/formal_vehicle/service/raw/drain_hose_contact
formal_vehicle_product_bridge|G2R|/sensors/lidar_2d/scan|sensor_msgs/msg/LaserScan|gz.msgs.LaserScan|/sensors/lidar_2d/scan
formal_vehicle_product_bridge|G2R|/sensors/gnss/fix|sensor_msgs/msg/NavSatFix|gz.msgs.NavSat|/sensors/gnss/fix
formal_vehicle_product_bridge|G2R|/sensors/imu/data|sensor_msgs/msg/Imu|gz.msgs.IMU|/sensors/imu/data
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg/applied|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg/applied
formal_vehicle_product_bridge|R2G|/model/tzcup_formal_sanitation_vehicle/water_recovery/command/enable|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/command/enable
formal_vehicle_product_bridge|R2G|/model/tzcup_formal_sanitation_vehicle/water_recovery/command/service_drain_open|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/command/service_drain_open
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_mass_kg|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_mass_kg
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_level_fraction|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_level_fraction
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/flow_l_min|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/flow_l_min
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/recovered_volume_l|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/recovered_volume_l
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_full|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_full
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_flow_l_min|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_flow_l_min
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_tank_level_fraction|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_tank_level_fraction
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_differential_pressure_kpa|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_differential_pressure_kpa
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/pump_current_a|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/pump_current_a
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_low_probe_wet|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_low_probe_wet
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_high_probe_wet|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_high_probe_wet
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_protection_active|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_protection_active
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_open|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_open
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_permitted|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_permitted
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drained_volume_l|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drained_volume_l
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/dry_bin/fill_level_fraction|std_msgs/msg/Float64|gz.msgs.Double|/model/tzcup_formal_sanitation_vehicle/dry_bin/fill_level_fraction
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/dry_bin/full|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/dry_bin/full
formal_vehicle_product_bridge|G2R|/model/tzcup_formal_sanitation_vehicle/dry_bin/sensor_ready|std_msgs/msg/Bool|gz.msgs.Boolean|/model/tzcup_formal_sanitation_vehicle/dry_bin/sensor_ready
formal_vehicle_product_bridge|G2R|/cleaning/suction_nozzle/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/cleaning/suction_nozzle/contact
formal_vehicle_product_bridge|G2R|/storage/dry_deposit/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/storage/dry_deposit/contact
front_bumper_contact_bridge|G2R|/safety/front_bumper/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/formal_vehicle/simulation/raw/front_bumper/contact
rear_bumper_contact_bridge|G2R|/safety/rear_bumper/contact|ros_gz_interfaces/msg/Contacts|gz.msgs.Contacts|/formal_vehicle/simulation/raw/rear_bumper/contact
""".strip()

NODE_TO_NATIVE = {
    "formal_vehicle_product_bridge": (
        "formal_vehicle_product_native_bridge", "src/FormalVehicleProductNativeBridge.cc", None
    ),
    "cleaning_actuator_scalar_bridge": (
        "cleaning_actuator_scalar_native_bridge", "src/CleaningActuatorScalarNativeBridge.cc", None
    ),
    "formal_auxiliary_bridge": (
        "formal_auxiliary_native_bridge", "src/FormalAuxiliaryNativeBridge.cc", None
    ),
    "formal_squeegee_evaluation_bridge": (
        "formal_contact_evaluation_native_bridge", "src/FormalContactEvaluationNativeBridge.cc", "squeegee"
    ),
    "formal_brush_contact_evaluation_bridge": (
        "formal_contact_evaluation_native_bridge", "src/FormalContactEvaluationNativeBridge.cc", "brushes"
    ),
    "charge_receptacle_contact_bridge": (
        "formal_contact_evaluation_native_bridge", "src/FormalContactEvaluationNativeBridge.cc", "charge_receptacle"
    ),
    "wastewater_drain_contact_bridge": (
        "formal_contact_evaluation_native_bridge", "src/FormalContactEvaluationNativeBridge.cc", "wastewater_drain"
    ),
    "front_bumper_contact_bridge": (
        "formal_contact_evaluation_native_bridge", "src/FormalContactEvaluationNativeBridge.cc", "front_bumper"
    ),
    "rear_bumper_contact_bridge": (
        "formal_contact_evaluation_native_bridge", "src/FormalContactEvaluationNativeBridge.cc", "rear_bumper"
    ),
}

CPP_TYPES = {
    "std_msgs::msg::Bool": "std_msgs/msg/Bool",
    "std_msgs::msg::Float64": "std_msgs/msg/Float64",
    "sensor_msgs::msg::LaserScan": "sensor_msgs/msg/LaserScan",
    "sensor_msgs::msg::NavSatFix": "sensor_msgs/msg/NavSatFix",
    "sensor_msgs::msg::Imu": "sensor_msgs/msg/Imu",
    "ros_gz_interfaces::msg::Contacts": "ros_gz_interfaces/msg/Contacts",
    "gz::msgs::Boolean": "gz.msgs.Boolean",
    "gz::msgs::Double": "gz.msgs.Double",
    "gz::msgs::LaserScan": "gz.msgs.LaserScan",
    "gz::msgs::NavSat": "gz.msgs.NavSat",
    "gz::msgs::IMU": "gz.msgs.IMU",
    "gz::msgs::Contacts": "gz.msgs.Contacts",
}

DECLARATION = re.compile(
    r"constexpr\s+(?P<kind>GroupedGazeboToRosEndpoint|GazeboToRosEndpoint|RosToGazeboEndpoint)"
    r"<\s*(?P<ros>[\w:]+)\s*,\s*(?P<gz>[\w:]+)\s*>\s*"
    r"(?P<symbol>\w+)\s*\{\s*(?:\"(?P<group>[^\"]+)\"\s*,\s*)?"
    r"\"(?P<topic>[^\"]+)\"\s*\};",
    re.DOTALL,
)


def _old_rows() -> tuple[tuple[str, str, str, str, str, str], ...]:
    return tuple(tuple(line.split("|")) for line in RAW_OLD_CONTRACT.splitlines())


def _expected_endpoints() -> set[Endpoint]:
    endpoints = set()
    for node, direction, topic, ros_type, gazebo_type, _ in _old_rows():
        _, source, group = NODE_TO_NATIVE[node]
        endpoints.add(Endpoint(source, group, direction, topic, ros_type, gazebo_type))
    return endpoints


def _native_endpoints() -> tuple[set[Endpoint], dict[str, int]]:
    actual: set[Endpoint] = set()
    symbol_counts: dict[str, int] = {}
    for source in sorted({item[1] for item in NODE_TO_NATIVE.values()}):
        raw = (CONTROL / source).read_text(encoding="utf-8")
        for match in DECLARATION.finditer(raw):
            kind = match["kind"]
            group = match["group"]
            assert (kind == "GroupedGazeboToRosEndpoint") == (group is not None), (
                source,
                match["symbol"],
            )
            direction = "R2G" if kind == "RosToGazeboEndpoint" else "G2R"
            actual.add(
                Endpoint(
                    source,
                    group,
                    direction,
                    match["topic"],
                    CPP_TYPES[match["ros"]],
                    CPP_TYPES[match["gz"]],
                )
            )
            symbol_counts[f"{source}:{match['symbol']}"] = len(
                re.findall(rf"\b{re.escape(match['symbol'])}\b", raw)
            )
    return actual, symbol_counts


def _literal_keyword(call: ast.Call, name: str):
    for keyword in call.keywords:
        if keyword.arg == name:
            return ast.literal_eval(keyword.value)
    return None


def _launch_nodes() -> dict[str, list[dict[str, object]]]:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    nodes: dict[str, list[dict[str, object]]] = {}
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == "Node"):
            continue
        name = _literal_keyword(call, "name")
        if name in NODE_TO_NATIVE:
            nodes.setdefault(name, []).append(
                {
                    "package": _literal_keyword(call, "package"),
                    "executable": _literal_keyword(call, "executable"),
                    "parameters": _literal_keyword(call, "parameters"),
                    "remappings": tuple(_literal_keyword(call, "remappings") or ()),
                }
            )
    return nodes


def test_fixed_former_parameter_bridge_contract_has_61_typed_endpoints() -> None:
    rows = _old_rows()
    assert len(rows) == 61
    assert Counter(row[1] for row in rows) == {"G2R": 47, "R2G": 14}
    assert len({row[:5] for row in rows}) == 61


def test_native_typed_constexpr_contract_exactly_replaces_all_61_endpoints() -> None:
    actual, symbol_counts = _native_endpoints()
    expected = _expected_endpoints()
    assert actual == expected
    assert len(actual) == 61
    assert Counter(endpoint.direction for endpoint in actual) == {"G2R": 47, "R2G": 14}
    assert Counter(endpoint.group for endpoint in actual if endpoint.group) == {
        "squeegee": 7,
        "brushes": 3,
        "charge_receptacle": 1,
        "wastewater_drain": 1,
        "front_bumper": 1,
        "rear_bumper": 1,
    }
    # A declaration that the executable never configures is not a bridge.
    assert all(count >= 2 for count in symbol_counts.values())


def test_launch_keeps_the_six_contact_instances_groups_and_all_eight_remaps() -> None:
    nodes = _launch_nodes()
    assert set(nodes) == set(NODE_TO_NATIVE)
    for name, (executable, _, group) in NODE_TO_NATIVE.items():
        assert len(nodes[name]) == 1
        node = nodes[name][0]
        assert node["package"] == "sanitation_gazebo_control"
        assert node["executable"] == executable
        assert node["parameters"] == ([{"endpoint_group": group}] if group else None)

    actual_remaps = {
        (name, source, target)
        for name, records in nodes.items()
        for source, target in records[0]["remappings"]
    }
    expected_remaps = {
        (node, topic, ros_target)
        for node, direction, topic, _, _, ros_target in _old_rows()
        if direction == "G2R" and topic != ros_target
    }
    assert len(expected_remaps) == 8
    assert actual_remaps == expected_remaps


def test_launch_has_no_legacy_parameter_bridge_for_the_nine_migrated_nodes() -> None:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    legacy = []
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == "Node"):
            continue
        name = _literal_keyword(call, "name")
        if name in NODE_TO_NATIVE and _literal_keyword(call, "executable") == "parameter_bridge":
            legacy.append(name)
    assert legacy == []


def test_all_four_native_executables_are_built_installed_and_packaged() -> None:
    cmake = (CONTROL / "CMakeLists.txt").read_text(encoding="utf-8")
    expected_sources = {
        "formal_vehicle_product_native_bridge": "src/FormalVehicleProductNativeBridge.cc",
        "cleaning_actuator_scalar_native_bridge": "src/CleaningActuatorScalarNativeBridge.cc",
        "formal_auxiliary_native_bridge": "src/FormalAuxiliaryNativeBridge.cc",
        "formal_contact_evaluation_native_bridge": "src/FormalContactEvaluationNativeBridge.cc",
    }
    for executable, source in expected_sources.items():
        add = re.search(
            rf"add_executable\(\s*{re.escape(executable)}\s+(?P<source>[^\s)]+)\s*\)",
            cmake,
            re.DOTALL,
        )
        assert add and add["source"] == source
        assert re.search(
            rf"install\(TARGETS\s+{re.escape(executable)}\s+RUNTIME DESTINATION lib/\$\{{PROJECT_NAME\}}\s*\)",
            cmake,
            re.DOTALL,
        )
        dependencies = re.search(
            rf"ament_target_dependencies\(\s*{re.escape(executable)}(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        )
        assert dependencies and {"rclcpp", "ros_gz_bridge", "std_msgs"} <= set(
            dependencies["body"].split()
        )
    package = ET.parse(CONTROL / "package.xml").getroot()
    depends = {element.text for element in package.findall("depend")}
    assert {"rclcpp", "ros_gz_bridge", "ros_gz_interfaces", "std_msgs"} <= depends


def test_snapshot_authoritative_inventory_tracks_every_new_native_bridge_input() -> None:
    sources = set(authoritative_source_paths(ROOT))
    assert {
        Path("starter_ws/src/sanitation_gazebo_control/src/FormalVehicleProductNativeBridge.cc"),
        Path("starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorScalarNativeBridge.cc"),
        Path("starter_ws/src/sanitation_gazebo_control/src/FormalAuxiliaryNativeBridge.cc"),
        Path("starter_ws/src/sanitation_gazebo_control/src/FormalContactEvaluationNativeBridge.cc"),
        Path("starter_ws/src/sanitation_gazebo_control/include/sanitation_gazebo_control/NativeBridgeSupport.hh"),
        Path("starter_ws/src/sanitation_gazebo_control/CMakeLists.txt"),
        Path("starter_ws/src/sanitation_gazebo_control/package.xml"),
    } <= sources
