"""Fail-closed safety decision for the physical wastewater service drain."""

from __future__ import annotations

from dataclasses import dataclass
import math


REQUIRED_INPUTS = (
    "request_open",
    "stationary",
    "cleaning_stopped",
    "pump_stopped",
    "cap_open",
    "hose_connected",
    "tank_valid",
    "safety_permit",
    "power_available",
)


@dataclass(frozen=True)
class ServiceDrainDecision:
    permitted: bool
    target_position_rad: float
    water_recovery_drain_open: bool
    reasons: tuple[str, ...]


@dataclass
class _TimedBool:
    value: bool = False
    updated_s: float | None = None


class ServiceDrainCore:
    """Require every physical and safety condition to be fresh and true."""

    def __init__(self, *, input_timeout_s: float = 0.25, open_position_rad: float = math.pi / 2) -> None:
        if not math.isfinite(input_timeout_s) or input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s must be finite and positive")
        if not math.isfinite(open_position_rad) or open_position_rad <= 0.0:
            raise ValueError("open_position_rad must be finite and positive")
        self.input_timeout_s = float(input_timeout_s)
        self.open_position_rad = float(open_position_rad)
        self._inputs = {name: _TimedBool() for name in REQUIRED_INPUTS}

    def update(self, name: str, value: bool, now_s: float) -> None:
        if name not in self._inputs:
            raise KeyError(f"unknown service-drain input: {name}")
        if not math.isfinite(now_s):
            raise ValueError("service-drain input timestamp must be finite")
        self._inputs[name] = _TimedBool(bool(value), float(now_s))

    def evaluate(self, now_s: float) -> ServiceDrainDecision:
        reasons: list[str] = []
        if not math.isfinite(now_s):
            return ServiceDrainDecision(False, 0.0, False, ("invalid_now",))
        for name in REQUIRED_INPUTS:
            signal = self._inputs[name]
            age = None if signal.updated_s is None else now_s - signal.updated_s
            if age is None or not math.isfinite(age) or age < 0.0 or age > self.input_timeout_s:
                reasons.append(f"{name}_stale")
            elif not signal.value:
                reasons.append(f"{name}_false")
        permitted = not reasons
        return ServiceDrainDecision(
            permitted=permitted,
            target_position_rad=self.open_position_rad if permitted else 0.0,
            water_recovery_drain_open=permitted,
            reasons=tuple(reasons),
        )

