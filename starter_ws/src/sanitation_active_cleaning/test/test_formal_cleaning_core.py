import pytest

from sanitation_active_cleaning.formal_cleaning_core import FormalCleaningCore


@pytest.fixture
def core():
    return FormalCleaningCore(
        request_timeout_sec=1.0,
        safety_timeout_sec=0.5,
        joint_state_timeout_sec=0.5,
        work_lift_m=0.100,
        transport_lift_m=0.000,
        lift_tolerance_m=0.001,
    )


def evaluate(core, **changes):
    values = {
        "now": 5.0,
        "requested": True,
        "request_stamp": 5.0,
        "permitted": True,
        "safety_stamp": 5.0,
        "lift_position_m": 0.100,
        "joint_state_stamp": 5.0,
    }
    values.update(changes)
    return core.evaluate(**values)


def test_cleaning_only_activates_after_fresh_work_pose(core):
    deploying = evaluate(core, lift_position_m=0.0)
    assert deploying.phase == "DEPLOYING"
    assert not deploying.active
    assert deploying.target_lift_m == pytest.approx(0.100)

    cleaning = evaluate(core)
    assert cleaning.phase == "CLEANING"
    assert cleaning.active
    assert cleaning.work_pose_reached


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"permitted": False}, "safety_not_permitted"),
        ({"safety_stamp": 4.49}, "safety_stale"),
    ],
)
def test_safety_inhibit_has_no_motion_target_and_zero_actuators(core, changes, reason):
    decision = evaluate(core, **changes)
    assert decision.phase == "SAFE_INHIBIT"
    assert not decision.active
    assert decision.target_lift_m is None
    assert decision.reason == reason


def test_request_watchdog_stows_while_safety_remains_valid(core):
    decision = evaluate(core, request_stamp=3.99)
    assert decision.phase == "TRANSPORT"
    assert not decision.active
    assert decision.target_lift_m == pytest.approx(0.0)
    assert decision.reason == "request_stale"


def test_stale_joint_state_never_starts_brush_pump_or_water(core):
    decision = evaluate(core, joint_state_stamp=4.49)
    assert decision.phase == "DEPLOYING"
    assert not decision.active
    assert decision.reason == "joint_state_stale"


def test_non_finite_parameters_are_rejected():
    with pytest.raises(ValueError):
        FormalCleaningCore(
            request_timeout_sec=1.0,
            safety_timeout_sec=0.5,
            joint_state_timeout_sec=0.5,
            work_lift_m=float("nan"),
            transport_lift_m=0.0,
            lift_tolerance_m=0.001,
        )
