from check_formal_water_preoperational_readiness import (
    ACTIVE_CONTROLLERS,
    INACTIVE_CONTROLLERS,
    REQUIRED_NODES,
    SERVICE_DRAIN_MANAGER_NODE,
    controller_contract_checks,
    controller_states_from_response,
)
from types import SimpleNamespace


def test_parses_semantic_controller_states() -> None:
    states = {name: "active" for name in ACTIVE_CONTROLLERS}
    states.update({name: "inactive" for name in INACTIVE_CONTROLLERS})
    assert all(controller_contract_checks(states).values())


def test_missing_or_wrong_state_cannot_satisfy_contract() -> None:
    checks = controller_contract_checks({"brush_controller": "active"})
    assert checks["main_six_controllers_active"] is False
    assert checks["brush_and_recovery_configured_inactive"] is False


def test_controller_service_response_is_mapped_by_exact_name_and_state() -> None:
    response = SimpleNamespace(
        controller=[
            SimpleNamespace(name="brush_controller", state="inactive"),
            SimpleNamespace(name="joint_state_broadcaster", state="active"),
        ]
    )
    assert controller_states_from_response(response) == {
        "brush_controller": "inactive",
        "joint_state_broadcaster": "active",
    }


def test_water_preflight_requires_exclusive_evaluator_drain_ownership() -> None:
    assert REQUIRED_NODES == {"/whole_vehicle_safety_manager"}
    assert SERVICE_DRAIN_MANAGER_NODE == "/service_drain_safety_manager"
