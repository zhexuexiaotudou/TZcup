"""Pure simulation model for formal-vehicle auxiliary product interfaces."""

from __future__ import annotations

import math
from dataclasses import dataclass


PRODUCT_BINDINGS = {
    "work_lights": {
        "link": "bodywork_lighting_link",
        "datum": "warning_and_work_lighting_datum_link",
    },
    "tail_lights": {
        "link": "bodywork_lighting_link",
        "datum": "warning_and_work_lighting_datum_link",
    },
    "four_corner_warning_lights": {
        "link": "bodywork_lighting_link",
        "datum": "warning_and_work_lighting_datum_link",
    },
    "charge_interface": {
        "link": "charge_interface_datum_link",
        "datum": "charge_interface_datum_link",
    },
    "fused_power_distribution": {
        "link": "power_distribution_box_link",
        "datum": "power_distribution_box_link",
    },
    "isolated_low_voltage_power": {
        "link": "isolated_dc_dc_module_link",
        "datum": "isolated_dc_dc_module_link",
    },
    "safety_relay": {
        "link": "safety_relay_link",
        "datum": "safety_relay_link",
    },
    "main_power_isolator": {
        "link": "main_power_isolator_handle_link",
        "datum": "main_power_isolator_housing_link",
    },
    "main_power_contactor": {
        "link": "main_power_contactor_armature_link",
        "datum": "main_power_contactor_housing_link",
    },
    "emergency_stop": {
        "link": "emergency_stop_datum_link",
        "datum": "emergency_stop_datum_link",
    },
}


@dataclass(frozen=True)
class AuxiliaryState:
    """One product-facing simulation state snapshot."""

    battery_soc: float
    emergency_stop_active: bool
    main_isolator_closed: bool
    main_contactor_closed: bool
    relay_command_enabled: bool
    relay_enabled: bool
    charge_connected: bool
    safety_branch_enabled: bool
    low_voltage_branch_enabled: bool
    high_power_branch_enabled: bool
    work_lights_on: bool
    tail_lights_on: bool
    warning_lights_on: bool
    net_battery_power_kw: float
    active_reasons: tuple[str, ...]


@dataclass
class FormalAuxiliarySystemCore:
    """Energy-domain simulation without an assumed A300 battery voltage."""

    battery_soc: float = 0.8
    simulation_battery_capacity_kwh: float = 1.024
    simulation_charge_power_kw: float = 0.650
    simulation_low_voltage_load_kw: float = 0.08
    simulation_high_power_idle_load_kw: float = 0.12
    simulation_lighting_load_kw: float = 0.04
    safety_soc_floor: float = 0.02
    low_voltage_soc_floor: float = 0.05
    high_power_soc_floor: float = 0.10

    def __post_init__(self) -> None:
        positive = {
            "simulation_battery_capacity_kwh": self.simulation_battery_capacity_kwh,
            "simulation_charge_power_kw": self.simulation_charge_power_kw,
            "simulation_low_voltage_load_kw": self.simulation_low_voltage_load_kw,
            "simulation_high_power_idle_load_kw": self.simulation_high_power_idle_load_kw,
            "simulation_lighting_load_kw": self.simulation_lighting_load_kw,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if not math.isfinite(self.battery_soc) or not 0.0 <= self.battery_soc <= 1.0:
            raise ValueError("battery_soc must be within [0, 1]")
        floors = (
            self.safety_soc_floor,
            self.low_voltage_soc_floor,
            self.high_power_soc_floor,
        )
        if not 0.0 <= floors[0] < floors[1] < floors[2] < 1.0:
            raise ValueError("SOC floors must be strictly increasing within [0, 1)")

    def step(
        self,
        *,
        elapsed_sec: float,
        emergency_stop_active: bool,
        main_power_requested: bool,
        charge_connected_requested: bool,
        work_lights_requested: bool,
        tail_lights_requested: bool,
        warning_lights_requested: bool,
        main_isolator_closed: bool | None = None,
        main_contactor_closed: bool | None = None,
    ) -> AuxiliaryState:
        """Advance SOC and return a fail-closed product state."""

        if not math.isfinite(elapsed_sec) or elapsed_sec < 0.0:
            raise ValueError("elapsed_sec must be finite and non-negative")

        reasons: list[str] = []
        safety_power = self.battery_soc > self.safety_soc_floor
        if not safety_power:
            reasons.append("safety_branch_undervoltage_simulation")
        charge_connected = bool(
            charge_connected_requested
            and emergency_stop_active
            and not main_power_requested
            and safety_power
            and self.battery_soc < 1.0
        )
        if charge_connected_requested and not charge_connected:
            reasons.append("charge_interlock_rejected")

        # Backward-compatible None is reserved for pure-core callers.  The
        # simulation node always supplies fresh physical feedback and treats a
        # missing sample as open; an operator request alone can never energize
        # an applied branch.
        isolator_closed = bool(
            main_power_requested
            if main_isolator_closed is None
            else main_isolator_closed
        )
        contactor_closed = bool(
            False if main_contactor_closed is None and main_isolator_closed is not None
            else (
                main_power_requested
                if main_contactor_closed is None
                else main_contactor_closed
            )
        )
        effective_main_power = bool(main_power_requested and isolator_closed)
        if main_power_requested and not isolator_closed:
            reasons.append("main_isolator_open_or_feedback_stale")

        low_voltage = bool(
            effective_main_power
            and safety_power
            and self.battery_soc > self.low_voltage_soc_floor
            and not charge_connected
        )
        # This is the safety relay's actuator-enable contact, not merely the
        # presence of safety-branch supply.  It opens for E-stop, main-power
        # off and service charging so the vehicle manager cannot permit motion
        # while the simulated high-power bus is unavailable.
        relay_command_enabled = bool(
            safety_power
            and effective_main_power
            and not emergency_stop_active
            and not charge_connected
        )
        relay_enabled = bool(relay_command_enabled and contactor_closed)
        high_power = bool(
            relay_enabled
            and self.battery_soc > self.high_power_soc_floor
        )
        if emergency_stop_active:
            reasons.append("emergency_stop_active")
        if main_power_requested and not high_power:
            reasons.append("high_power_branch_inhibited")
        if relay_command_enabled and not contactor_closed:
            reasons.append("main_contactor_open_or_feedback_stale")

        work_lights = bool(low_voltage and not emergency_stop_active and work_lights_requested)
        tail_lights = bool(low_voltage and tail_lights_requested)
        warning_lights = bool(
            safety_power and (emergency_stop_active or warning_lights_requested)
        )

        load_kw = 0.0
        if low_voltage:
            load_kw += self.simulation_low_voltage_load_kw
        if high_power:
            load_kw += self.simulation_high_power_idle_load_kw
        if work_lights or tail_lights or warning_lights:
            load_kw += self.simulation_lighting_load_kw
        net_battery_power_kw = (
            -self.simulation_charge_power_kw if charge_connected else load_kw
        )
        delta_soc = (
            net_battery_power_kw
            * elapsed_sec
            / 3600.0
            / self.simulation_battery_capacity_kwh
        )
        self.battery_soc = max(0.0, min(1.0, self.battery_soc - delta_soc))

        return AuxiliaryState(
            battery_soc=self.battery_soc,
            emergency_stop_active=bool(emergency_stop_active),
            main_isolator_closed=isolator_closed,
            main_contactor_closed=contactor_closed,
            relay_command_enabled=relay_command_enabled,
            relay_enabled=relay_enabled,
            charge_connected=charge_connected,
            safety_branch_enabled=safety_power,
            low_voltage_branch_enabled=low_voltage,
            high_power_branch_enabled=high_power,
            work_lights_on=work_lights,
            tail_lights_on=tail_lights,
            warning_lights_on=warning_lights,
            net_battery_power_kw=net_battery_power_kw,
            active_reasons=tuple(reasons),
        )
