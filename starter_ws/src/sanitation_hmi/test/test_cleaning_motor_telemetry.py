import pytest

from sanitation_hmi.cleaning_motor_telemetry import (
    decode_cleaning_motor_snapshot,
)


def healthy_snapshot():
    values = [1.0, 7.0, 3.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    for index in range(5):
        values.extend(
            [0.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 1.0, 0.0, 0.0, float(index == 3)]
        )
    return values


def test_decodes_atomic_cleaning_motor_snapshot_for_dashboard_diagnostics():
    decoded = decode_cleaning_motor_snapshot(healthy_snapshot())

    assert decoded["telemetry_sequence"] == 7
    assert decoded["physics_update_sequence"] == 3
    assert decoded["command_fresh"] is True
    assert decoded["fault_active"] is False
    assert [motor["name"] for motor in decoded["motors"]] == [
        "left_side_brush",
        "right_side_brush",
        "central_roller",
        "cleaning_lift",
        "recovery_pump",
    ]


def test_rejects_truncated_or_internally_inconsistent_fault_snapshot():
    with pytest.raises(ValueError, match="63 finite"):
        decode_cleaning_motor_snapshot(healthy_snapshot()[:-1])

    inconsistent = healthy_snapshot()
    inconsistent[5] = 1.0
    with pytest.raises(ValueError, match="aggregate"):
        decode_cleaning_motor_snapshot(inconsistent)
