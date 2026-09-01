from __future__ import annotations

import math

import pytest

from sanitation_safety.service_drain_core import REQUIRED_INPUTS, ServiceDrainCore


def _permit_all(core: ServiceDrainCore, now: float = 1.0) -> None:
    for name in REQUIRED_INPUTS:
        core.update(name, True, now)


def test_all_fresh_true_inputs_open_both_physical_and_recovery_paths() -> None:
    core = ServiceDrainCore()
    _permit_all(core)
    decision = core.evaluate(1.1)
    assert decision.permitted is True
    assert decision.target_position_rad == pytest.approx(math.pi / 2)
    assert decision.water_recovery_drain_open is True
    assert decision.reasons == ()


@pytest.mark.parametrize(
    "name",
    [
        "request_open",
        "stationary",
        "cleaning_stopped",
        "pump_stopped",
        "cap_open",
        "hose_connected",
        "tank_valid",
        "safety_permit",
        "power_available",
    ],
)
def test_any_false_condition_closes_joint_and_recovery(name: str) -> None:
    core = ServiceDrainCore()
    _permit_all(core)
    core.update(name, False, 1.05)
    decision = core.evaluate(1.1)
    assert decision.permitted is False
    assert decision.target_position_rad == 0.0
    assert decision.water_recovery_drain_open is False
    assert f"{name}_false" in decision.reasons


def test_any_stale_input_closes_joint_and_recovery() -> None:
    core = ServiceDrainCore(input_timeout_s=0.25)
    _permit_all(core)
    core.update("hose_connected", True, 0.8)
    decision = core.evaluate(1.1)
    assert decision.target_position_rad == 0.0
    assert decision.water_recovery_drain_open is False
    assert "hose_connected_stale" in decision.reasons


def test_missing_inputs_fail_closed_on_startup() -> None:
    decision = ServiceDrainCore().evaluate(0.0)
    assert not decision.permitted
    assert len(decision.reasons) == len(REQUIRED_INPUTS)
    assert all(reason.endswith("_stale") for reason in decision.reasons)


def test_backward_or_invalid_clock_fails_closed() -> None:
    core = ServiceDrainCore()
    _permit_all(core, now=2.0)
    assert "request_open_stale" in core.evaluate(1.0).reasons
    assert core.evaluate(float("nan")).reasons == ("invalid_now",)


def test_unknown_input_and_invalid_configuration_are_rejected() -> None:
    core = ServiceDrainCore()
    with pytest.raises(KeyError, match="unknown"):
        core.update("not_a_condition", True, 0.0)
    with pytest.raises(ValueError, match="input_timeout_s"):
        ServiceDrainCore(input_timeout_s=0.0)
    with pytest.raises(ValueError, match="open_position_rad"):
        ServiceDrainCore(open_position_rad=float("nan"))
