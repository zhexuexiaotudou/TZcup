import pytest

from sanitation_safety.dry_speed_qualification_core import (
    DEFAULT_MAX_LINEAR_VELOCITY_MPS,
    DRY_CLEANING_SPEED_PROFILE,
    ISOLATED_SAME_MAP_DRY_STATE,
    DrySpeedQualificationState,
)


def qualified_state():
    state = DrySpeedQualificationState(
        configured_max_linear_velocity_mps=1.0,
        mission_mode="cleaning",
        operation_speed_profile=DRY_CLEANING_SPEED_PROFILE,
        qualification_state=ISOLATED_SAME_MAP_DRY_STATE,
    )
    state.set_qualification_active(True, 10.0)
    state.set_dry_brush_active(True, 10.0)
    return state


def test_qualified_dry_brush_is_the_only_one_mps_state():
    assert qualified_state().effective_max_linear_velocity_mps(
        now=10.1, pump_output=(0.0,)
    ) == 1.0


@pytest.mark.parametrize(
    "mode,profile,state,brush,pump,now",
    [
        ("mapping", DRY_CLEANING_SPEED_PROFILE, ISOLATED_SAME_MAP_DRY_STATE, (1.0, 0.0, 0.0), (0.0,), 10.1),
        ("cleaning", "mapping_safe", ISOLATED_SAME_MAP_DRY_STATE, (1.0, 0.0, 0.0), (0.0,), 10.1),
        ("cleaning", DRY_CLEANING_SPEED_PROFILE, "none", (1.0, 0.0, 0.0), (0.0,), 10.1),
        ("cleaning", DRY_CLEANING_SPEED_PROFILE, ISOLATED_SAME_MAP_DRY_STATE, (0.0, 0.0, 0.0), (0.0,), 10.1),
        ("cleaning", DRY_CLEANING_SPEED_PROFILE, ISOLATED_SAME_MAP_DRY_STATE, (1.0, 0.0, 0.0), (1.0,), 10.1),
        ("cleaning", DRY_CLEANING_SPEED_PROFILE, ISOLATED_SAME_MAP_DRY_STATE, (1.0, 0.0, 0.0), (0.0,), 10.26),
    ],
)
def test_any_mode_wet_brush_off_or_stale_heartbeat_reverts_to_default(
    mode, profile, state, brush, pump, now
):
    candidate = qualified_state()
    candidate.mission_mode = mode
    candidate.operation_speed_profile = profile
    candidate.qualification_state = state
    candidate.set_dry_brush_active(any(brush), 10.0)
    assert candidate.effective_max_linear_velocity_mps(
        now=now, pump_output=pump
    ) == DEFAULT_MAX_LINEAR_VELOCITY_MPS


@pytest.mark.parametrize("pump", [(), (0.0, 0.0), (float("nan"),), (True,), ("zero",)])
def test_missing_or_malformed_pump_readback_reverts_to_default(pump):
    assert qualified_state().effective_max_linear_velocity_mps(
        now=10.1, pump_output=pump
    ) == DEFAULT_MAX_LINEAR_VELOCITY_MPS
