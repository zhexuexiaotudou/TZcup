from __future__ import annotations

from pathlib import Path

import yaml

from validate_formal_vehicle_component_register import _python_topic_endpoints


ROOT = Path(__file__).resolve().parents[1]
REGISTER = ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_component_register.yaml"
POWER_ROOT = ROOT / "starter_ws" / "src" / "sanitation_power_system"
SAFETY_ROOT = ROOT / "starter_ws" / "src" / "sanitation_safety"
BMS_NODE = POWER_ROOT / "sanitation_power_system" / "a300_bms_node.py"
CHARGE_MANAGER = POWER_ROOT / "sanitation_power_system" / "charge_interface_manager.py"
SIMULATION_INPUTS = SAFETY_ROOT / "sanitation_safety" / "simulation_safety_inputs.py"


def test_power_and_charge_product_topics_have_one_declared_writer() -> None:
    register = yaml.safe_load(REGISTER.read_text(encoding="utf-8"))
    contracts = register["topic_contracts"]
    expected = {
        "battery_state": ("a300_bms_simulator", BMS_NODE),
        "battery_soc": ("a300_bms_simulator", BMS_NODE),
        "bms_fault": ("a300_bms_simulator", BMS_NODE),
        "bms_status": ("a300_bms_simulator", BMS_NODE),
        "traction_permitted": ("a300_bms_simulator", BMS_NODE),
        "charge_requested": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "main_power_requested": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "load_power_request": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "charge_enable": ("charge_interface_manager", CHARGE_MANAGER),
        "charge_connected": ("charge_interface_manager", CHARGE_MANAGER),
        "charge_power_request": ("charge_interface_manager", CHARGE_MANAGER),
        "charge_status": ("charge_interface_manager", CHARGE_MANAGER),
        "auxiliary_power_status": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "safety_power_branch": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "low_voltage_power_branch": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "high_power_branch": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "work_lights_state": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "tail_lights_state": ("simulation_safety_inputs", SIMULATION_INPUTS),
        "warning_lights_state": ("simulation_safety_inputs", SIMULATION_INPUTS),
    }
    for contract_id, (writer_node, source) in expected.items():
        contract = contracts[contract_id]
        assert contract["transport"] == "ros_native", contract_id
        assert contract["direction"] == "publisher", contract_id
        assert contract["single_writer"] is True, contract_id
        assert contract["writer_node"] == writer_node, contract_id
        assert ("publisher", contract["ros_topic"], contract["ros_type"].rsplit("/", 1)[-1]) in (
            _python_topic_endpoints(source)
        ), contract_id


def test_simulation_input_adapter_does_not_publish_manager_owned_charge_state() -> None:
    endpoints = _python_topic_endpoints(SIMULATION_INPUTS)
    assert (
        "subscription",
        "/formal_vehicle/power/charge_connected",
        "Bool",
    ) in endpoints
    assert (
        "publisher",
        "/formal_vehicle/power/charge_connected",
        "Bool",
    ) not in endpoints
    assert all(endpoint[1] != "/formal_vehicle/power/charge_plug_present" for endpoint in endpoints)


def test_charge_plug_presence_comes_only_from_real_contact_messages() -> None:
    manager_endpoints = _python_topic_endpoints(CHARGE_MANAGER)
    assert (
        "subscription",
        "/formal_vehicle/service/raw/charge_plug_contact",
        "Contacts",
    ) in manager_endpoints
    source = CHARGE_MANAGER.read_text(encoding="utf-8")
    assert 'self._values["plug_present"] = bool(message.contacts)' in source
    assert "/formal_vehicle/power/charge_plug_present" not in source
    simulation_source = SIMULATION_INPUTS.read_text(encoding="utf-8")
    assert "/formal_vehicle/power/charge_plug_present" not in simulation_source
    assert "_on_charge_plug_present" not in simulation_source


def test_charge_manager_has_a_complete_shared_freshness_gate() -> None:
    source = CHARGE_MANAGER.read_text(encoding="utf-8")
    for key in (
        "requested",
        "plug_present",
        "traction_permitted",
        "emergency_stop_active",
        "main_power_requested",
        "bms_fault",
        "bms",
        "joint",
        "odom",
    ):
        assert f'"{key}"' in source
    assert 'self.declare_parameter("input_timeout_sec", 0.25)' in source
    assert "required = tuple(self._times)" in source
    assert "telemetry_fresh = all(" in source
    assert "0.0 <= now - self._times[key] <= timeout" in source


def test_documented_power_boundaries_and_ownership_are_current() -> None:
    power_readme = (POWER_ROOT / "README.md").read_text(encoding="utf-8")
    safety_readme = (SAFETY_ROOT / "README.md").read_text(encoding="utf-8")
    combined = power_readme + safety_readme
    assert "1024 Wh" in combined
    assert "650 W" in combined
    assert "6 kWh" not in combined
    assert "1 kW" not in combined
    assert "a300_bms_simulator is the sole writer" in power_readme
    assert "charge_interface_manager is the sole writer" in power_readme
    assert "RUNTIME_REVALIDATION_PENDING" in power_readme
    assert "RUNTIME_REVALIDATION_PENDING" in safety_readme


def test_bms_charge_request_has_an_explicit_watchdog() -> None:
    source = BMS_NODE.read_text(encoding="utf-8")
    assert "charge_request_timeout_sec" in source
    assert "charge_request_fresh" in source
    assert "self._charge_request_w if charge_request_fresh else 0.0" in source
    assert 'for name in ("publish_period_sec", "charge_request_timeout_sec")' in source
    assert "if not math.isfinite(value) or value <= 0.0:" in source


def test_auxiliary_charge_connected_has_an_explicit_watchdog() -> None:
    source = SIMULATION_INPUTS.read_text(encoding="utf-8")
    assert "charge_connected_timeout_sec" in source
    assert "charge_connected_fresh" in source
    assert "if not charge_connected_fresh:" in source
    assert "self._charge_connected = False" in source
    for parameter in (
        "publish_period_sec",
        "operator_command_timeout_sec",
        "bumper_contact_latch_sec",
        "battery_state_timeout_sec",
        "charge_connected_timeout_sec",
    ):
        assert f'"{parameter}"' in source
    assert "if not math.isfinite(value) or value <= 0.0:" in source


def test_charge_manager_rejects_non_finite_or_non_positive_parameters() -> None:
    source = CHARGE_MANAGER.read_text(encoding="utf-8")
    assert '"input_timeout_sec"' in source
    assert '"rated_charge_power_w"' in source
    assert "if not math.isfinite(value) or value <= 0.0:" in source
    assert 'raise ValueError(f"{name} must be finite and positive")' in source
