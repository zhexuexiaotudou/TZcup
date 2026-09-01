from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws/src/sanitation_safety"


def test_component_register_contains_every_auxiliary_product_datum():
    register = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml").read_text(
            encoding="utf-8"
        )
    )
    entries = {entry["id"]: entry for entry in register["functional_positions"]}
    expected = {
        "fused_power_distribution": "power_distribution_box_link",
        "isolated_low_voltage_power": "isolated_dc_dc_module_link",
        "hardwired_safety_enable": "safety_relay_link",
        "charge_interface": "charge_port_housing_link",
        "emergency_stop": "emergency_stop_housing_link",
        "warning_and_work_lighting": "warning_and_work_lighting_datum_link",
    }
    for component, link in expected.items():
        assert entries[component]["link"] == link


def test_simulation_node_consumes_physical_estop_and_publishes_other_safety_inputs():
    source = (PACKAGE / "sanitation_safety/simulation_safety_inputs.py").read_text(
        encoding="utf-8"
    )
    for topic in (
        "/emergency_stop",
        "/safety/relay_enabled",
        "/safety/control_heartbeat",
        "/formal_vehicle/power/battery_state",
        "/formal_vehicle/auxiliary/status_json",
        "/formal_vehicle/simulation/raw/front_bumper/contact",
        "/formal_vehicle/simulation/raw/rear_bumper/contact",
        "/safety/front_bumper/contact",
        "/safety/rear_bumper/contact",
    ):
        assert topic in source
    assert "/evaluation/" not in source
    assert '"battery_voltage_v": self._battery_voltage_v' in source
    assert '"battery_state_fresh": battery_fresh' in source
    assert 'create_publisher(Bool, "/emergency_stop"' not in source
    assert 'Bool, "/emergency_stop", self._on_estop_state' in source


def test_console_entry_point_is_installed():
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert "simulation_safety_inputs = " in setup
    assert "sanitation_safety.simulation_safety_inputs:main" in setup


def test_formal_launch_exposes_fail_closed_simulation_input_opt_in():
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert '"start_simulation_safety_inputs"' in launch
    assert 'executable="simulation_safety_inputs"' in launch
    assert '"simulation_initial_estop_active"' in launch
    assert 'default_value="true"' in launch
    assert '"use_sim_time": False' in launch
    assert 'name="formal_auxiliary_bridge"' in launch
    assert '"/emergency_stop@std_msgs/msg/Bool[gz.msgs.Boolean"' in launch
    assert '"/formal_vehicle/simulation/raw/front_bumper/contact"' in launch
    assert '"/formal_vehicle/simulation/raw/rear_bumper/contact"' in launch
