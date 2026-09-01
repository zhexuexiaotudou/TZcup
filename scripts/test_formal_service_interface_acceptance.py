from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from validate_formal_service_interface_acceptance import (
    EXPECTED_SCENARIOS,
    EXPECTED_WASTEWATER_CAPACITY_KG,
    aggregate,
    required_gate_names,
)


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_service_acceptance"
STATION = PACKAGE / "models" / "formal_service_station.sdf"
LAUNCH = PACKAGE / "launch" / "formal_service_acceptance.launch.py"
COLLECTOR = PACKAGE / "sanitation_service_acceptance" / "collector.py"
RUNNER = ROOT / "scripts" / "run_formal_service_interface_acceptance.sh"
POWER_XACRO = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "urdf"
    / "high_fidelity"
    / "power_service_hardware.xacro"
)
VEHICLE_XACRO = POWER_XACRO.parents[1] / "formal_competition_vehicle.urdf.xacro"


def test_station_contains_only_physical_fixture_collisions() -> None:
    root = ET.parse(STATION).getroot()
    model = root.find("model")
    assert model is not None and model.findtext("static") == "true"
    collisions = {
        collision.get("name")
        for collision in model.findall("link/collision")
    }
    assert collisions == {
        "charge_plug_fixture_collision",
        "wastewater_hose_fixture_collision",
    }
    assert not model.findall(".//plugin")
    assert not model.findall(".//sensor")


def test_evaluation_joint_plugins_are_default_off_and_isolated() -> None:
    vehicle = VEHICLE_XACRO.read_text(encoding="utf-8")
    power = POWER_XACRO.read_text(encoding="utf-8")
    assert '<xacro:arg name="service_acceptance_interfaces" default="false"/>' in vehicle
    assert 'service_acceptance_interfaces="$(arg service_acceptance_interfaces)"' in vehicle
    assert '<xacro:if value="${service_acceptance_interfaces}">' in power
    assert power.count("gz-sim-joint-position-controller-system") == 3
    for topic in (
        "/formal_vehicle/evaluation/service/charge_door_position_rad",
        "/formal_vehicle/evaluation/service/charge_lock_position_m",
        "/formal_vehicle/evaluation/service/drain_cap_position_rad",
    ):
        assert topic in power


def test_acceptance_launch_uses_only_ros_to_gz_evaluation_commands() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    assert "formal_vehicle_sim.launch.py" in source
    assert "formal_service_station.sdf" in source
    assert source.count("@std_msgs/msg/Float64]gz.msgs.Double") == 3
    assert "@std_msgs/msg/Float64[gz.msgs.Double" not in source
    assert "name='service_acceptance_joint_command_bridge'" in source
    assert "OnProcessExit" in source and "Shutdown" in source


def test_collector_has_no_world_truth_or_boolean_contact_input() -> None:
    source = COLLECTOR.read_text(encoding="utf-8")
    assert "/formal_vehicle/service/raw/charge_plug_contact" in source
    assert "/formal_vehicle/service/raw/drain_hose_contact" in source
    assert "ros_gz_interfaces.msg import Contacts" in source
    assert "/formal_vehicle/power/charge_plug_present" not in source
    assert "sensor_msgs.msg import BatteryState, JointState" in source
    assert "/formal_vehicle/power/traction_permitted" in source
    assert "service_drained_volume_l" in source
    assert "tank_mass_kg" in source
    for forbidden in ("/world/", "/ground_truth", "/truth/", "/pose/info"):
        assert forbidden not in source


def test_runner_executes_every_scenario_with_fresh_vehicle_model() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    for scenario in EXPECTED_SCENARIOS:
        assert scenario in source
    assert "service_acceptance_interfaces:=true" in source
    assert "wastewater_load_mass_kg:=8.30" in source
    assert "FORMAL_SERVICE_SETUP" in source
    assert "formal service overlay is missing" in source
    assert 'station_x_offset="4.0"' in source
    assert "validate_formal_service_interface_acceptance.py" in source
    assert "--check --output \"${snapshot}\"" in source
    assert "--snapshot \"${snapshot}\" --session \"${session}\"" in source


def test_acceptance_launch_explicitly_starts_safety_and_a300_power_simulators() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    assert "'start_simulation_safety_inputs': 'true'" in source
    assert "'start_power_system_simulators': 'true'" in source


def _write_episode(root: Path, scenario: str) -> None:
    (root / f"{scenario}.json").write_text(
        json.dumps(
            {
                "schema": "tzcup.formal_service_interface_episode.v1",
                "scenario": scenario,
                "result": "PASS",
                "gates": {name: True for name in required_gate_names(scenario)},
                "wastewater_capacity_kg": EXPECTED_WASTEWATER_CAPACITY_KG,
                "subscription_topics": [
                    "/formal_vehicle/service/raw/charge_plug_contact",
                    "/joint_states",
                ],
            }
        ),
        encoding="utf-8",
    )


def test_aggregate_requires_all_scenarios_and_rejects_truth_topics(tmp_path: Path) -> None:
    for scenario in EXPECTED_SCENARIOS:
        _write_episode(tmp_path, scenario)
    assert aggregate(tmp_path)["status"] == "PASS"
    missing = tmp_path / "charge_allow.json"
    missing.unlink()
    assert aggregate(tmp_path)["status"] == "FAIL_CLOSED"
    _write_episode(tmp_path, "charge_allow")
    episode = json.loads(missing.read_text(encoding="utf-8"))
    episode["subscription_topics"].append("/world/formal/pose/info")
    missing.write_text(json.dumps(episode), encoding="utf-8")
    result = aggregate(tmp_path)
    assert result["status"] == "FAIL_CLOSED"
    assert any("forbidden world truth" in error for error in result["errors"])


def test_aggregate_rejects_pass_label_with_missing_physical_gate(tmp_path: Path) -> None:
    for scenario in EXPECTED_SCENARIOS:
        _write_episode(tmp_path, scenario)
    path = tmp_path / "drain_allow.json"
    episode = json.loads(path.read_text(encoding="utf-8"))
    del episode["gates"]["drain_mass_conservation_within_0_02kg"]
    path.write_text(json.dumps(episode), encoding="utf-8")
    result = aggregate(tmp_path)
    assert result["status"] == "FAIL_CLOSED"
    assert any("missing required gates" in error for error in result["errors"])


def test_aggregate_freezes_final_8_30kg_capacity_contract(tmp_path: Path) -> None:
    for scenario in EXPECTED_SCENARIOS:
        _write_episode(tmp_path, scenario)
    result = aggregate(tmp_path)
    assert result["wastewater_capacity_contract"] == {
        "expected_wastewater_capacity_kg": 8.30,
        "gates": {
            "dynamic_payload_capacity_kg_is_8_30": True,
            "water_recovery_capacity_kg_is_8_30": True,
            "expanded_storage_payload_clamp_kg_is_8_30": True,
        },
        "status": "PASS",
    }

    episode_path = tmp_path / "drain_allow.json"
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    episode["wastewater_capacity_kg"] = 8.493
    episode_path.write_text(json.dumps(episode), encoding="utf-8")
    result = aggregate(tmp_path)
    assert result["status"] == "FAIL_CLOSED"
    assert any("8.30 kg wastewater capacity" in error for error in result["errors"])
