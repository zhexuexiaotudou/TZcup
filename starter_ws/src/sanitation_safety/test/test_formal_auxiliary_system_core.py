import math

import pytest

from sanitation_safety.formal_auxiliary_system_core import FormalAuxiliarySystemCore
from sanitation_safety.formal_auxiliary_system_core import PRODUCT_BINDINGS


def test_power_up_estop_is_a_safe_product_state_with_warning_lights():
    state = FormalAuxiliarySystemCore().step(
        elapsed_sec=0.0,
        emergency_stop_active=True,
        main_power_requested=True,
        charge_connected_requested=False,
        work_lights_requested=True,
        tail_lights_requested=True,
        warning_lights_requested=False,
    )

    assert not state.relay_enabled
    assert not state.high_power_branch_enabled
    assert state.low_voltage_branch_enabled
    assert not state.work_lights_on
    assert state.tail_lights_on
    assert state.warning_lights_on
    assert "emergency_stop_active" in state.active_reasons


def test_released_estop_allows_requested_product_branches_and_lights():
    state = FormalAuxiliarySystemCore().step(
        elapsed_sec=0.0,
        emergency_stop_active=False,
        main_power_requested=True,
        charge_connected_requested=False,
        work_lights_requested=True,
        tail_lights_requested=True,
        warning_lights_requested=True,
    )

    assert state.low_voltage_branch_enabled
    assert state.relay_enabled
    assert state.high_power_branch_enabled
    assert state.work_lights_on
    assert state.tail_lights_on
    assert state.warning_lights_on


def test_charge_interlock_requires_estop_and_main_power_off_then_increases_soc():
    core = FormalAuxiliarySystemCore(battery_soc=0.5)
    rejected = core.step(
        elapsed_sec=10.0,
        emergency_stop_active=False,
        main_power_requested=True,
        charge_connected_requested=True,
        work_lights_requested=False,
        tail_lights_requested=False,
        warning_lights_requested=False,
    )
    assert not rejected.charge_connected
    assert "charge_interlock_rejected" in rejected.active_reasons

    before = core.battery_soc
    charging = core.step(
        elapsed_sec=360.0,
        emergency_stop_active=True,
        main_power_requested=False,
        charge_connected_requested=True,
        work_lights_requested=False,
        tail_lights_requested=False,
        warning_lights_requested=False,
    )
    assert charging.charge_connected
    assert not charging.relay_enabled
    assert charging.net_battery_power_kw < 0.0
    assert charging.battery_soc > before


def test_depleted_simulation_battery_drops_relay_and_every_powered_output():
    state = FormalAuxiliarySystemCore(battery_soc=0.0).step(
        elapsed_sec=0.0,
        emergency_stop_active=True,
        main_power_requested=True,
        charge_connected_requested=True,
        work_lights_requested=True,
        tail_lights_requested=True,
        warning_lights_requested=True,
    )

    assert not state.relay_enabled
    assert not state.safety_branch_enabled
    assert not state.low_voltage_branch_enabled
    assert not state.high_power_branch_enabled
    assert not state.work_lights_on
    assert not state.tail_lights_on
    assert not state.warning_lights_on


def test_main_power_off_never_closes_the_actuator_safety_relay():
    state = FormalAuxiliarySystemCore().step(
        elapsed_sec=0.0,
        emergency_stop_active=False,
        main_power_requested=False,
        charge_connected_requested=False,
        work_lights_requested=False,
        tail_lights_requested=False,
        warning_lights_requested=False,
    )
    assert state.safety_branch_enabled
    assert not state.relay_enabled
    assert not state.high_power_branch_enabled


def test_physical_isolator_and_contactor_are_both_required_for_applied_power():
    core = FormalAuxiliarySystemCore()
    isolator_open = core.step(
        elapsed_sec=0.0,
        emergency_stop_active=False,
        main_power_requested=True,
        charge_connected_requested=False,
        work_lights_requested=True,
        tail_lights_requested=True,
        warning_lights_requested=False,
        main_isolator_closed=False,
        main_contactor_closed=False,
    )
    assert not isolator_open.relay_command_enabled
    assert not isolator_open.low_voltage_branch_enabled
    assert "main_isolator_open_or_feedback_stale" in isolator_open.active_reasons

    contactor_open = core.step(
        elapsed_sec=0.0,
        emergency_stop_active=False,
        main_power_requested=True,
        charge_connected_requested=False,
        work_lights_requested=True,
        tail_lights_requested=True,
        warning_lights_requested=False,
        main_isolator_closed=True,
        main_contactor_closed=False,
    )
    assert contactor_open.relay_command_enabled
    assert not contactor_open.relay_enabled
    assert not contactor_open.high_power_branch_enabled
    assert "main_contactor_open_or_feedback_stale" in contactor_open.active_reasons

    applied = core.step(
        elapsed_sec=0.0,
        emergency_stop_active=False,
        main_power_requested=True,
        charge_connected_requested=False,
        work_lights_requested=True,
        tail_lights_requested=True,
        warning_lights_requested=False,
        main_isolator_closed=True,
        main_contactor_closed=True,
    )
    assert applied.relay_command_enabled
    assert applied.relay_enabled
    assert applied.high_power_branch_enabled


def test_every_auxiliary_function_has_an_existing_model_binding_name():
    assert set(PRODUCT_BINDINGS) == {
        "work_lights",
        "tail_lights",
        "four_corner_warning_lights",
        "charge_interface",
        "fused_power_distribution",
        "isolated_low_voltage_power",
        "safety_relay",
        "main_power_isolator",
        "main_power_contactor",
        "emergency_stop",
    }
    assert all(binding["link"] and binding["datum"] for binding in PRODUCT_BINDINGS.values())


@pytest.mark.parametrize("elapsed", [-1.0, math.nan, math.inf])
def test_invalid_elapsed_time_is_rejected(elapsed):
    with pytest.raises(ValueError, match="elapsed_sec"):
        FormalAuxiliarySystemCore().step(
            elapsed_sec=elapsed,
            emergency_stop_active=True,
            main_power_requested=False,
            charge_connected_requested=False,
            work_lights_requested=False,
            tail_lights_requested=False,
            warning_lights_requested=False,
        )
