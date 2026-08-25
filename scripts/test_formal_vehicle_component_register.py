from __future__ import annotations

from pathlib import Path
import json

import pytest
import yaml

from validate_formal_vehicle_component_register import (
    DEFAULT_REGISTER,
    ComponentRegisterError,
    validate,
)


def test_committed_component_register_matches_expanded_urdf() -> None:
    result = validate()
    assert result["status"] == "COMPONENT_REGISTER_AND_MECHANICAL_LOAD_PATHS_VALID"
    assert result["sensor_installation_count"] == 8
    assert result["mechanical_subassembly_count"] >= 11
    assert result["functional_position_count"] >= 35
    assert result["top_protrusion_name"] == "modular_sensor_tower"


def _mutated_register(tmp_path: Path, mutate) -> Path:
    register = yaml.safe_load(DEFAULT_REGISTER.read_text(encoding="utf-8"))
    mutate(register)
    output = tmp_path / "mutated-register.yaml"
    output.write_text(yaml.safe_dump(register, sort_keys=False), encoding="utf-8")
    return output


def test_function_position_missing_topic_fails_closed(tmp_path: Path) -> None:
    register = _mutated_register(
        tmp_path,
        lambda data: data["functional_positions"][0].update(
            {"required_topics": ["/not/a/real/formal_vehicle/topic"]}
        ),
    )
    with pytest.raises(ComponentRegisterError, match="required_topic missing"):
        validate(register_path=register)


def test_function_position_uncommanded_joint_fails_closed(tmp_path: Path) -> None:
    def mutate(data: dict) -> None:
        pumping = next(item for item in data["functional_positions"] if item["id"] == "water_pumping")
        pumping["interface"] = "storage_controller"

    register = _mutated_register(tmp_path, mutate)
    with pytest.raises(ComponentRegisterError, match="does not command"):
        validate(register_path=register)


def test_committed_function_position_runtime_report_closes_all_actuators() -> None:
    report = json.loads(
        (
            DEFAULT_REGISTER.parents[2]
            / "reports"
            / "engineering"
            / "formal_function_positions_runtime_report.json"
        ).read_text(encoding="utf-8")
    )
    assert report["status"] == "FORMAL_CLEANING_STORAGE_AND_RECOVERY_ACTUATORS_PASSED"
    assert report["controller_count"] == 4
    assert report["actuated_joint_count"] == 10
    assert report["failures"] == []
    assert len(report["measured"]) == 10
    assert all(measurement["passed"] for measurement in report["measured"].values())
    assert report["contact_topic_publishers"] == {
        "/cleaning/suction_nozzle/contact": 1,
        "/storage/dry_deposit/contact": 1,
        "/safety/front_bumper/contact": 1,
        "/safety/rear_bumper/contact": 1,
    }
