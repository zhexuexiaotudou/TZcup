import math
from pathlib import Path

import pytest

from sanitation_power_system.a300_bms_core import (
    A300BatteryCore,
    A300BatteryParameters,
)


def test_published_pack_boundaries_are_energy_consistent():
    parameters = A300BatteryParameters()
    parameters.validate()
    assert parameters.nominal_voltage_v == 25.6
    assert parameters.design_capacity_ah == 40.0
    assert parameters.design_energy_wh == 1024.0
    assert parameters.continuous_discharge_current_a == 60.0
    assert parameters.circuit_breaker_current_a == 100.0
    assert parameters.maximum_supported_charge_power_w == 650.0
    assert parameters.maximum_supported_charge_current_a == 23.5


def test_discharge_conserves_energy_and_uses_negative_battery_state_current():
    core = A300BatteryCore(initial_soc=0.5)
    state = core.step(
        elapsed_sec=3600.0,
        requested_load_power_w=512.0,
        requested_charge_power_w=0.0,
        emergency_stop=False,
        main_power_requested=True,
    )
    assert state.soc == pytest.approx(0.0)
    assert state.current_a < 0.0
    assert state.load_power_delivered_w == pytest.approx(512.0)


def test_continuous_current_limit_clamps_load_without_tripping_breaker():
    core = A300BatteryCore(initial_soc=1.0)
    state = core.step(
        elapsed_sec=1.0,
        requested_load_power_w=2000.0,
        requested_charge_power_w=0.0,
        emergency_stop=False,
        main_power_requested=True,
    )
    assert state.discharge_limited
    assert not state.breaker_latched
    assert state.load_power_delivered_w <= 29.2 * 60.0


def test_breaker_latches_above_100a_and_only_service_state_can_reset():
    core = A300BatteryCore(initial_soc=0.5)
    state = core.step(
        elapsed_sec=0.1,
        requested_load_power_w=4000.0,
        requested_charge_power_w=0.0,
        emergency_stop=False,
        main_power_requested=True,
    )
    assert state.breaker_latched
    assert state.load_power_delivered_w == 0.0
    assert not core.reset_breaker(emergency_stop=False, main_power_requested=True)
    assert core.reset_breaker(emergency_stop=True, main_power_requested=False)


def test_charge_requires_estop_main_power_off_and_valid_temperature():
    core = A300BatteryCore(initial_soc=0.5)
    rejected = core.step(
        elapsed_sec=10.0,
        requested_load_power_w=0.0,
        requested_charge_power_w=650.0,
        emergency_stop=False,
        main_power_requested=True,
    )
    assert not rejected.charging
    accepted = core.step(
        elapsed_sec=10.0,
        requested_load_power_w=0.0,
        requested_charge_power_w=650.0,
        emergency_stop=True,
        main_power_requested=False,
    )
    assert accepted.charging
    assert accepted.current_a > 0.0
    assert accepted.charge_power_accepted_w <= 650.0


@pytest.mark.parametrize("value", [math.nan, math.inf, -1.0])
def test_invalid_step_inputs_fail_closed(value):
    core = A300BatteryCore()
    with pytest.raises(ValueError):
        core.step(
            elapsed_sec=value,
            requested_load_power_w=0.0,
            requested_charge_power_w=0.0,
            emergency_stop=True,
            main_power_requested=False,
        )


def test_safety_publish_cadence_is_steady_but_energy_elapsed_uses_ros_clock():
    source = (
        Path(__file__).resolve().parents[1]
        / "sanitation_power_system/a300_bms_node.py"
    ).read_text(encoding="utf-8")
    assert "Clock(clock_type=ClockType.STEADY_TIME)" in source
    assert "clock=self._scheduler_clock" in source
    assert "now_ns = self.get_clock().now().nanoseconds" in source
    assert "elapsed_sec = max(0.0, (now_ns - self._last_clock_ns) * 1e-9)" in source
