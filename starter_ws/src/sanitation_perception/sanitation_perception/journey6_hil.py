"""Fail-closed command authority for split PC/Journey 6 HIL."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class AckermannHilCommand:
    stamp_s: float
    sequence: int
    speed_mps: float
    steering_angle_rad: float
    acceleration_limit_mps2: float
    source_id: str
    valid_until_s: float

    def validate(self) -> None:
        values = (
            self.stamp_s,
            self.speed_mps,
            self.steering_angle_rad,
            self.acceleration_limit_mps2,
            self.valid_until_s,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError("HIL command contains non-finite values")
        if self.sequence < 0 or not self.source_id:
            raise ValueError("HIL command sequence and source are required")
        if self.valid_until_s <= self.stamp_s:
            raise ValueError("HIL command validity window is empty")
        if self.acceleration_limit_mps2 <= 0.0:
            raise ValueError("HIL acceleration limit must be positive")


class HilCommandAuthority:
    """Accept fresh commands from one J6 source and latch safe-stop on link loss."""

    def __init__(self, *, source_id: str, maximum_future_skew_s: float = 0.05) -> None:
        if not source_id:
            raise ValueError("Journey 6 command source_id is required")
        self.source_id = source_id
        self.maximum_future_skew_s = float(maximum_future_skew_s)
        self.last_command: AckermannHilCommand | None = None
        self.last_sequence = -1
        self.last_stamp_s = -math.inf
        self.link_connected = True
        self.resume_authorized = True

    def accept(self, command: AckermannHilCommand, *, now_s: float) -> None:
        command.validate()
        if command.source_id != self.source_id:
            raise ValueError("non-Journey6 command authority rejected")
        if not self.link_connected or not self.resume_authorized:
            raise RuntimeError("HIL link requires an explicit operator resume")
        if command.sequence <= self.last_sequence or command.stamp_s <= self.last_stamp_s:
            raise ValueError("stale or replayed HIL command rejected")
        if command.stamp_s > now_s + self.maximum_future_skew_s:
            raise ValueError("HIL command timestamp is in the future")
        if command.valid_until_s <= now_s:
            raise ValueError("expired HIL command rejected")
        self.last_command = command
        self.last_sequence = command.sequence
        self.last_stamp_s = command.stamp_s

    def network_lost(self) -> None:
        self.link_connected = False
        self.resume_authorized = False
        self.last_command = None

    def network_restored(self) -> None:
        self.link_connected = True

    def operator_resume(self) -> None:
        if not self.link_connected:
            raise RuntimeError("cannot resume while HIL link is disconnected")
        self.resume_authorized = True
        self.last_command = None

    def output(self, *, now_s: float) -> AckermannHilCommand | None:
        command = self.last_command
        if (
            command is None
            or not self.link_connected
            or not self.resume_authorized
            or now_s >= command.valid_until_s
        ):
            return None
        return replace(command)


__all__ = ["AckermannHilCommand", "HilCommandAuthority"]
