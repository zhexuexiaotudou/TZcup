"""Deterministic A300 40 Ah pack and BMS state model."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class A300BatteryParameters:
    """Published electrical boundaries plus explicit engineering assumptions."""

    nominal_voltage_v: float = 25.6
    design_capacity_ah: float = 40.0
    design_energy_wh: float = 1024.0
    continuous_discharge_current_a: float = 60.0
    circuit_breaker_current_a: float = 100.0
    maximum_supported_charge_power_w: float = 650.0
    maximum_supported_charge_current_a: float = 23.5
    charging_minimum_temperature_c: float = 0.0
    charging_maximum_temperature_c: float = 40.0
    engineering_empty_voltage_v: float = 22.4
    engineering_full_voltage_v: float = 29.2
    engineering_internal_resistance_ohm: float = 0.025

    def validate(self) -> None:
        values = vars(self)
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("battery parameters must be finite")
        positive = (
            self.nominal_voltage_v,
            self.design_capacity_ah,
            self.design_energy_wh,
            self.continuous_discharge_current_a,
            self.circuit_breaker_current_a,
            self.maximum_supported_charge_power_w,
            self.maximum_supported_charge_current_a,
            self.engineering_empty_voltage_v,
            self.engineering_full_voltage_v,
            self.engineering_internal_resistance_ohm,
        )
        if not all(value > 0.0 for value in positive):
            raise ValueError("battery electrical boundaries must be positive")
        if self.circuit_breaker_current_a <= self.continuous_discharge_current_a:
            raise ValueError("breaker current must exceed continuous current")
        if self.engineering_full_voltage_v <= self.engineering_empty_voltage_v:
            raise ValueError("full voltage must exceed empty voltage")
        if self.charging_maximum_temperature_c <= self.charging_minimum_temperature_c:
            raise ValueError("charging temperature range is invalid")
        if abs(
            self.nominal_voltage_v * self.design_capacity_ah
            - self.design_energy_wh
        ) > 1e-6:
            raise ValueError("voltage, amp-hours and watt-hours disagree")


@dataclass(frozen=True)
class A300BatteryState:
    soc: float
    voltage_v: float
    current_a: float
    charge_ah: float
    temperature_c: float
    charging: bool
    discharge_limited: bool
    breaker_latched: bool
    load_power_delivered_w: float
    charge_power_accepted_w: float
    reasons: tuple[str, ...]


class A300BatteryCore:
    """Energy-conserving pack model with BMS and service-charge interlocks."""

    def __init__(
        self,
        *,
        parameters: A300BatteryParameters | None = None,
        initial_soc: float = 0.8,
        initial_temperature_c: float = 25.0,
    ) -> None:
        self.parameters = parameters or A300BatteryParameters()
        self.parameters.validate()
        if not math.isfinite(initial_soc) or not 0.0 <= initial_soc <= 1.0:
            raise ValueError("initial_soc must be within [0, 1]")
        if not math.isfinite(initial_temperature_c):
            raise ValueError("initial temperature must be finite")
        self.soc = initial_soc
        self.temperature_c = initial_temperature_c
        self.breaker_latched = False

    def reset_breaker(self, *, emergency_stop: bool, main_power_requested: bool) -> bool:
        """Reset only in the stationary service-power state."""

        if emergency_stop and not main_power_requested:
            self.breaker_latched = False
            return True
        return False

    def step(
        self,
        *,
        elapsed_sec: float,
        requested_load_power_w: float,
        requested_charge_power_w: float,
        emergency_stop: bool,
        main_power_requested: bool,
    ) -> A300BatteryState:
        values = (elapsed_sec, requested_load_power_w, requested_charge_power_w)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("battery step inputs must be finite")
        if elapsed_sec < 0.0 or requested_load_power_w < 0.0 or requested_charge_power_w < 0.0:
            raise ValueError("elapsed time and requested power must be non-negative")

        p = self.parameters
        reasons: list[str] = []
        open_circuit_voltage = p.engineering_empty_voltage_v + self.soc * (
            p.engineering_full_voltage_v - p.engineering_empty_voltage_v
        )
        requested_current = requested_load_power_w / max(open_circuit_voltage, 1e-9)
        if requested_current > p.circuit_breaker_current_a:
            self.breaker_latched = True
            reasons.append("circuit_breaker_overcurrent_latched")

        continuous_power_limit = open_circuit_voltage * p.continuous_discharge_current_a
        available_energy = self.soc * p.design_energy_wh
        energy_limited_power = available_energy * 3600.0 / elapsed_sec if elapsed_sec > 0.0 else math.inf
        delivered_load = min(requested_load_power_w, continuous_power_limit, energy_limited_power)
        discharge_limited = delivered_load + 1e-9 < requested_load_power_w
        if discharge_limited:
            reasons.append("continuous_discharge_or_energy_limit")
        if self.breaker_latched or not main_power_requested:
            delivered_load = 0.0
            if self.breaker_latched:
                reasons.append("breaker_open")

        charging_temperature_ok = (
            p.charging_minimum_temperature_c
            <= self.temperature_c
            <= p.charging_maximum_temperature_c
        )
        charging_interlock_ok = emergency_stop and not main_power_requested
        accepted_charge = 0.0
        if requested_charge_power_w > 0.0:
            if not charging_interlock_ok:
                reasons.append("charge_motion_interlock_rejected")
            elif not charging_temperature_ok:
                reasons.append("charge_temperature_interlock_rejected")
            elif self.soc >= 1.0:
                reasons.append("battery_full")
            else:
                charge_limit = min(
                    p.maximum_supported_charge_power_w,
                    open_circuit_voltage * p.maximum_supported_charge_current_a,
                )
                remaining_energy = (1.0 - self.soc) * p.design_energy_wh
                remaining_limited_power = (
                    remaining_energy * 3600.0 / elapsed_sec
                    if elapsed_sec > 0.0
                    else charge_limit
                )
                accepted_charge = min(
                    requested_charge_power_w, charge_limit, remaining_limited_power
                )
                if accepted_charge + 1e-9 < requested_charge_power_w:
                    reasons.append("charger_or_capacity_limit")

        delta_wh = (accepted_charge - delivered_load) * elapsed_sec / 3600.0
        self.soc = min(1.0, max(0.0, self.soc + delta_wh / p.design_energy_wh))
        current_a = accepted_charge / max(open_circuit_voltage, 1e-9)
        current_a -= delivered_load / max(open_circuit_voltage, 1e-9)
        terminal_voltage = open_circuit_voltage + current_a * (
            p.engineering_internal_resistance_ohm
        )
        return A300BatteryState(
            soc=self.soc,
            voltage_v=terminal_voltage,
            current_a=current_a,
            charge_ah=self.soc * p.design_capacity_ah,
            temperature_c=self.temperature_c,
            charging=accepted_charge > 0.0,
            discharge_limited=discharge_limited,
            breaker_latched=self.breaker_latched,
            load_power_delivered_w=delivered_load,
            charge_power_accepted_w=accepted_charge,
            reasons=tuple(dict.fromkeys(reasons)),
        )
