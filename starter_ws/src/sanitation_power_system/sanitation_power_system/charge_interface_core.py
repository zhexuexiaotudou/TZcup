"""Fail-closed service-charge interlock independent of ROS and Gazebo."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ChargeInterfaceInputs:
    requested: bool
    plug_present: bool
    lock_engaged: bool
    door_open: bool
    stationary: bool
    traction_permitted: bool
    emergency_stop_active: bool
    main_power_requested: bool
    bms_fault: bool
    telemetry_fresh: bool


@dataclass(frozen=True)
class ChargeInterfaceDecision:
    enabled: bool
    charge_power_request_w: float
    reasons: tuple[str, ...]


def evaluate_charge_interface(
    inputs: ChargeInterfaceInputs, *, rated_charge_power_w: float = 650.0
) -> ChargeInterfaceDecision:
    """Permit charging only while every physical and electrical guard passes."""

    if not math.isfinite(rated_charge_power_w) or rated_charge_power_w <= 0.0:
        raise ValueError("rated_charge_power_w must be finite and positive")
    checks = (
        (inputs.telemetry_fresh, "telemetry_stale"),
        (inputs.requested, "charge_not_requested"),
        (inputs.plug_present, "plug_absent"),
        (inputs.lock_engaged, "connector_lock_not_engaged"),
        (inputs.door_open, "charge_door_not_open"),
        (inputs.stationary, "vehicle_not_stationary"),
        (not inputs.traction_permitted, "traction_not_inhibited"),
        (inputs.emergency_stop_active, "emergency_stop_not_active"),
        (not inputs.main_power_requested, "main_power_requested"),
        (not inputs.bms_fault, "bms_fault"),
    )
    reasons = tuple(reason for passed, reason in checks if not passed)
    enabled = not reasons
    return ChargeInterfaceDecision(
        enabled=enabled,
        charge_power_request_w=rated_charge_power_w if enabled else 0.0,
        reasons=reasons,
    )
