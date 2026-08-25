from __future__ import annotations

from validate_formal_vehicle_component_register import validate


def test_committed_component_register_matches_expanded_urdf() -> None:
    result = validate()
    assert result["status"] == "COMPONENT_REGISTER_AND_MECHANICAL_LOAD_PATHS_VALID"
    assert result["sensor_installation_count"] == 8
    assert result["mechanical_subassembly_count"] >= 8
    assert result["top_protrusion_name"] == "modular_sensor_tower"
