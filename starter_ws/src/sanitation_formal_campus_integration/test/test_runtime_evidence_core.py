from sanitation_formal_campus_integration.runtime_evidence_core import (
    COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S,
    COMMAND_CHAIN_TOPICS,
    EXPECTED_COMMAND_TOPIC_PUBLISHER,
    first_nonzero_chain_is_ordered,
)


def _times(*values: float) -> dict[str, float]:
    return dict(zip(COMMAND_CHAIN_TOPICS, values, strict=True))


def test_command_chain_accepts_monotonic_first_nonzero_receipts():
    assert first_nonzero_chain_is_ordered(_times(1.0, 1.1, 1.2, 1.3))


def test_command_chain_allows_only_bounded_cross_topic_callback_reordering():
    tolerance = COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S
    assert first_nonzero_chain_is_ordered(_times(1.0, 1.0 - tolerance, 1.1, 1.2))
    assert not first_nonzero_chain_is_ordered(
        _times(1.0, 1.0 - tolerance - 0.001, 1.1, 1.2)
    )


def test_command_chain_rejects_missing_or_nonfinite_first_receipts():
    missing = _times(1.0, 1.1, 1.2, 1.3)
    missing["/cmd_vel_gate"] = None  # type: ignore[assignment]
    assert not first_nonzero_chain_is_ordered(missing)
    assert not first_nonzero_chain_is_ordered(_times(1.0, 1.1, float("nan"), 1.3))


def test_command_chain_publisher_contract_names_every_production_stage():
    assert EXPECTED_COMMAND_TOPIC_PUBLISHER == {
        "/cmd_vel_nav": "/controller_server",
        "/cmd_vel_smoothed": "/velocity_smoother",
        "/cmd_vel_gate": "/collision_monitor",
        "/base_controller/cmd_vel": "/whole_vehicle_safety_manager",
    }
