from __future__ import annotations

from dataclasses import replace
import math

import pytest

from sanitation_power_system.charge_interface_core import ChargeInterfaceInputs
from sanitation_power_system.charge_interface_core import evaluate_charge_interface


def healthy() -> ChargeInterfaceInputs:
    return ChargeInterfaceInputs(
        requested=True,
        plug_present=True,
        lock_engaged=True,
        door_open=True,
        stationary=True,
        traction_permitted=False,
        emergency_stop_active=True,
        main_power_requested=False,
        bms_fault=False,
        telemetry_fresh=True,
    )


def test_all_physical_and_electrical_interlocks_permit_rated_charge() -> None:
    decision = evaluate_charge_interface(healthy())
    assert decision.enabled
    assert decision.charge_power_request_w == 650.0
    assert decision.reasons == ()


def test_each_missing_guard_fails_closed() -> None:
    mutations = {
        "requested": False,
        "plug_present": False,
        "lock_engaged": False,
        "door_open": False,
        "stationary": False,
        "traction_permitted": True,
        "emergency_stop_active": False,
        "main_power_requested": True,
        "bms_fault": True,
        "telemetry_fresh": False,
    }
    for field, value in mutations.items():
        decision = evaluate_charge_interface(replace(healthy(), **{field: value}))
        assert not decision.enabled, field
        assert decision.charge_power_request_w == 0.0
        assert decision.reasons, field


@pytest.mark.parametrize("invalid", [0.0, -1.0, math.nan, math.inf])
def test_charge_power_boundary_must_be_finite_and_positive(invalid: float) -> None:
    with pytest.raises(ValueError, match="finite and positive"):
        evaluate_charge_interface(healthy(), rated_charge_power_w=invalid)
