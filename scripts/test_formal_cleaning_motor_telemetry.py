import math

import pytest

from formal_cleaning_motor_telemetry import (
    CleaningMotorTelemetryError,
    HEADER_COUNT,
    VALUE_COUNT,
    decode_cleaning_motor_telemetry,
    update_physics_revision_watchdog,
)


def valid_values() -> list[float]:
    values = [0.0] * VALUE_COUNT
    values[:8] = [1.0, 42.0, 100.0, 0.0, 1.0, 0.0, 1.5, 36.0]
    for index in range(5):
        base = HEADER_COUNT + index * 11
        values[base : base + 11] = [
            (-1.0 if index == 1 else 1.0) * 8.0,
            0.1 * index,
            (-1.0 if index == 1 else 1.0) * 7.8,
            0.3,
            26.0,
            7.2,
            0.2,
            14.66,
            0.0,
            0.0,
            1.0 if index == 3 else 0.0,
        ]
    return values


def test_decodes_fixed_atomic_snapshot() -> None:
    decoded = decode_cleaning_motor_telemetry(valid_values())
    assert decoded["telemetry_sequence"] == 42
    assert decoded["physics_update_sequence"] == 100
    assert decoded["physics_update_stale"] is False
    assert decoded["command_fresh"] is True
    assert decoded["fault_active"] is False
    motors = decoded["motors"]
    assert motors[0]["name"] == "left_side_brush"
    assert motors[0]["command"] == 8.0
    assert motors[1]["measured_speed"] == -7.8
    assert motors[3]["control_mode"] == "position"


def test_decodes_early_subscription_fail_closed_startup_snapshot() -> None:
    values = [0.0] * VALUE_COUNT
    values[:8] = [1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0]
    values[HEADER_COUNT + 3 * 11 + 10] = 1.0

    decoded = decode_cleaning_motor_telemetry(values)

    assert decoded["physics_update_sequence"] == 0
    assert decoded["physics_update_stale"] is True
    assert decoded["command_fresh"] is False
    assert decoded["fault_active"] is True
    assert decoded["motors"][3]["control_mode"] == "position"
    assert all(float(motor["current_a"]) == 0.0 for motor in decoded["motors"])


def test_revision_watchdog_starts_only_after_first_healthy_physics_frame() -> None:
    sequence = None
    advanced = None
    for now_s in (0.0, 0.8, 2.0):
        sequence, advanced = update_physics_revision_watchdog(
            sequence=0,
            physics_stale=True,
            last_sequence=sequence,
            last_advance_s=advanced,
            now_s=now_s,
        )
        assert sequence is None
        assert advanced is None

    sequence, advanced = update_physics_revision_watchdog(
        sequence=1,
        physics_stale=False,
        last_sequence=sequence,
        last_advance_s=advanced,
        now_s=3.0,
    )
    assert (sequence, advanced) == (1, 3.0)
    assert update_physics_revision_watchdog(
        sequence=1,
        physics_stale=False,
        last_sequence=sequence,
        last_advance_s=advanced,
        now_s=3.749,
    ) == (1, 3.0)
    with pytest.raises(ValueError, match="stalled"):
        update_physics_revision_watchdog(
            sequence=1,
            physics_stale=False,
            last_sequence=sequence,
            last_advance_s=advanced,
            now_s=3.75,
        )


@pytest.mark.parametrize(
    ("mutator", "pattern"),
    [
        (lambda values: values.pop(), "length"),
        (lambda values: values.__setitem__(0, 2.0), "unsupported"),
        (lambda values: values.__setitem__(1, 1.5), "integer"),
        (lambda values: values.__setitem__(3, 2.0), "0 or 1"),
        (lambda values: values.__setitem__(5, math.nan), "non-finite"),
        (lambda values: values.__setitem__(HEADER_COUNT + 9, 9.0), "enum"),
        (lambda values: values.__setitem__(HEADER_COUNT + 8, 1.0), "disagrees"),
        (
            lambda values: values.__setitem__(HEADER_COUNT + 3 * 11 + 10, 0.0),
            "position actuator",
        ),
        (lambda values: values.__setitem__(6, 99.0), "total current"),
        (lambda values: values.__setitem__(7, 99.0), "total power"),
        (lambda values: values.__setitem__(5, 1.0), "aggregate fault"),
        (lambda values: values.__setitem__(6, -1.0), "non-negative"),
        (lambda values: values.__setitem__(HEADER_COUNT + 3, -1.0), "non-negative"),
        (lambda values: values.__setitem__(HEADER_COUNT + 4, 500.0), "temperature"),
        (lambda values: values.__setitem__(HEADER_COUNT + 7, 0.0), "speed_limit"),
        (lambda values: values.__setitem__(2, 0.0), "fail-closed startup"),
    ],
)
def test_rejects_schema_regressions(mutator, pattern: str) -> None:
    values = valid_values()
    mutator(values)
    with pytest.raises(CleaningMotorTelemetryError, match=pattern):
        decode_cleaning_motor_telemetry(values)
