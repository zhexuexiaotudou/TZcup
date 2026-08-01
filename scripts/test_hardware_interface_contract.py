from pathlib import Path
import runpy

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "starter_ws/src/sanitation_tasks/config/hardware_interface_contract.yaml"
FAULTS = ROOT / "starter_ws/src/sanitation_tasks/config/sim2real_fault_profiles.yaml"


def validator():
    return runpy.run_path(str(ROOT / "scripts/validate_hardware_interface_contract.py"))


def test_production_hardware_contract_is_valid() -> None:
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    assert validator()["validate_contract"](contract) == []


def test_fault_profiles_cover_transport_surface_and_actuator_degradation() -> None:
    profiles = yaml.safe_load(FAULTS.read_text(encoding="utf-8"))["profiles"]
    assert {"nominal", "transport_stress", "wet_surface", "degraded_drive"} <= profiles.keys()
    assert profiles["transport_stress"]["sensor_latency_ms"] >= 100
    assert profiles["wet_surface"]["wheel_slip_ratio"] >= 0.15
    assert profiles["degraded_drive"]["actuator_gain"] <= 0.65


def test_contract_rejects_sim_time_in_production() -> None:
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    contract["time_contract"]["production_uses_sim_time"] = True
    errors = validator()["validate_contract"](contract)
    assert "time_contract.production_uses_sim_time must be false" in errors
