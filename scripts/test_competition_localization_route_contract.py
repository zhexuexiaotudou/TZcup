from pathlib import Path


SCRIPT = Path(__file__).with_name("competition_localization_route.py").read_text()


def test_goal_is_gated_on_active_nav2_lifecycle_and_permit():
    assert (
        'LIFECYCLE_NODES = ("bt_navigator", "controller_server", "planner_server")'
        in SCRIPT
    )
    assert 'state["permit"]' in SCRIPT
    assert "action.server_is_ready()" in SCRIPT
    assert "lifecycle_states[name] == ACTIVE_STATE" in SCRIPT
    assert "motion_start = now" in SCRIPT
    assert (
        'readiness_error = "nav2_lifecycle_action_or_permit_timeout"'
        in SCRIPT
    )
    assert SCRIPT.index("goal = NavigateToPose.Goal()") > SCRIPT.index(
        "if ready():"
    )


def test_route_protocol_records_readiness_contract_without_ground_truth():
    assert '"requires_lifecycle_active_nodes": list(LIFECYCLE_NODES)' in SCRIPT
    assert '"requires_action_server_ready": True' in SCRIPT
    assert '"requires_actuator_permit": True' in SCRIPT
    assert '"readiness_timeout_s": args.prepare_seconds' in SCRIPT
