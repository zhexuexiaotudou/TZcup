"""ROS-independent safety core for the split PC/Journey 6 HIL gateway."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping

from sanitation_perception.journey6_hil import (
    AckermannHilCommand,
    HilCommandAuthority,
)


@dataclass(frozen=True)
class HealthFrame:
    source_id: str
    sequence: int
    stamp_s: float
    healthy: bool

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> "HealthFrame":
        frame = cls(
            source_id=str(payload["source_id"]),
            sequence=int(payload["sequence"]),
            stamp_s=float(payload["stamp_s"]),
            healthy=payload["healthy"] is True,
        )
        if not frame.source_id or frame.sequence < 0 or not math.isfinite(frame.stamp_s):
            raise ValueError("invalid J6 health frame")
        return frame


def command_from_mapping(payload: Mapping[str, object]) -> AckermannHilCommand:
    return AckermannHilCommand(
        stamp_s=float(payload["stamp_s"]),
        sequence=int(payload["sequence"]),
        speed_mps=float(payload["speed_mps"]),
        steering_angle_rad=float(payload["steering_angle_rad"]),
        acceleration_limit_mps2=float(payload["acceleration_limit_mps2"]),
        source_id=str(payload["source_id"]),
        valid_until_s=float(payload["valid_until_s"]),
    )


def command_to_mapping(command: AckermannHilCommand) -> dict[str, object]:
    return asdict(command)


class CommandSafetyGate:
    """Preserve J6 command provenance while allowing the PC to add zero only.

    The gate intentionally reuses ``sanitation_perception.journey6_hil`` for
    timestamp, sequence, authority, timeout, and reconnect semantics.  This
    wrapper adds physical limits, J6 health, E-stop, PC placement, and a
    startup latch.  No planning or replacement non-zero command exists here.
    """

    def __init__(
        self,
        *,
        j6_source_id: str = "j6-algorithm",
        health_timeout_s: float = 0.25,
        maximum_speed_mps: float = 2.0,
        maximum_steering_angle_rad: float = 0.60,
        maximum_acceleration_mps2: float = 1.5,
        zero_lifespan_s: float = 0.08,
    ) -> None:
        numeric = (
            health_timeout_s,
            maximum_speed_mps,
            maximum_steering_angle_rad,
            maximum_acceleration_mps2,
            zero_lifespan_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in numeric):
            raise ValueError("HIL safety limits must be finite and positive")
        self.j6_source_id = j6_source_id
        self.health_timeout_s = float(health_timeout_s)
        self.maximum_speed_mps = float(maximum_speed_mps)
        self.maximum_steering_angle_rad = float(maximum_steering_angle_rad)
        self.maximum_acceleration_mps2 = float(maximum_acceleration_mps2)
        self.zero_lifespan_s = float(zero_lifespan_s)
        self.authority = HilCommandAuthority(source_id=j6_source_id)
        self.authority.network_lost()
        self.authority.network_restored()
        self.last_health_sequence = -1
        self.last_health_stamp_s = -math.inf
        self.last_health_monotonic_s: float | None = None
        self.health_reported_healthy = False
        self.estop_active = True
        self.placement_gate_pass = False
        self.last_sim_time_s = -math.inf
        self.last_reason = "startup_operator_resume_required"

    def update_health(
        self,
        frame: HealthFrame,
        *,
        received_monotonic_s: float,
    ) -> None:
        if frame.source_id != self.j6_source_id:
            self.trip("non_j6_health_source")
            raise ValueError("health source is not the configured Journey 6 host")
        if frame.sequence <= self.last_health_sequence or frame.stamp_s <= self.last_health_stamp_s:
            self.trip("stale_or_replayed_health")
            raise ValueError("stale or replayed J6 health frame")
        if not math.isfinite(received_monotonic_s):
            self.trip("invalid_health_receive_time")
            raise ValueError("health receive time is invalid")
        self.last_health_sequence = frame.sequence
        self.last_health_stamp_s = frame.stamp_s
        self.last_health_monotonic_s = float(received_monotonic_s)
        self.health_reported_healthy = frame.healthy
        if not frame.healthy:
            self.trip("j6_unhealthy")

    def set_placement_gate(self, passed: bool) -> None:
        self.placement_gate_pass = bool(passed)
        if not self.placement_gate_pass:
            self.trip("pc_duplicate_algorithm_node")

    def set_estop(self, active: bool) -> None:
        self.estop_active = bool(active)
        if self.estop_active:
            self.trip("estop_active")

    def health_fresh(self, *, now_monotonic_s: float) -> bool:
        return (
            self.health_reported_healthy
            and self.last_health_monotonic_s is not None
            and 0.0 <= now_monotonic_s - self.last_health_monotonic_s <= self.health_timeout_s
        )

    def operator_resume(self, *, now_monotonic_s: float) -> None:
        blockers = []
        if self.estop_active:
            blockers.append("estop_active")
        if not self.placement_gate_pass:
            blockers.append("pc_placement_not_clean")
        if not self.health_fresh(now_monotonic_s=now_monotonic_s):
            blockers.append("j6_health_not_fresh")
        if blockers:
            self.last_reason = "+".join(blockers)
            raise RuntimeError("operator resume rejected: " + ", ".join(blockers))
        self.authority.network_restored()
        self.authority.operator_resume()
        self.last_reason = "waiting_for_new_j6_command"

    def accept(self, command: AckermannHilCommand, *, now_sim_s: float, now_monotonic_s: float) -> None:
        if now_sim_s < self.last_sim_time_s:
            self.trip("simulation_clock_rollback")
            raise ValueError("simulation clock rollback rejected")
        self.last_sim_time_s = float(now_sim_s)
        if not self.health_fresh(now_monotonic_s=now_monotonic_s):
            self.trip("j6_health_timeout")
            raise RuntimeError("J6 health is stale")
        if self.estop_active or not self.placement_gate_pass:
            self.trip("safety_interlock_active")
            raise RuntimeError("HIL safety interlock is active")
        if abs(command.speed_mps) > self.maximum_speed_mps:
            self.trip("speed_limit_exceeded")
            raise ValueError("HIL speed exceeds the PC physical safety limit")
        if abs(command.steering_angle_rad) > self.maximum_steering_angle_rad:
            self.trip("steering_limit_exceeded")
            raise ValueError("HIL steering exceeds the PC physical safety limit")
        if command.acceleration_limit_mps2 > self.maximum_acceleration_mps2:
            self.trip("acceleration_limit_exceeded")
            raise ValueError("HIL acceleration exceeds the PC physical safety limit")
        self.authority.accept(command, now_s=now_sim_s)
        self.last_reason = "j6_command_accepted"

    def output(self, *, now_sim_s: float, now_monotonic_s: float) -> AckermannHilCommand:
        if now_sim_s < self.last_sim_time_s:
            self.trip("simulation_clock_rollback")
        self.last_sim_time_s = max(self.last_sim_time_s, float(now_sim_s))
        if not self.health_fresh(now_monotonic_s=now_monotonic_s):
            self.trip("j6_health_timeout")
        command = self.authority.output(now_s=now_sim_s)
        if command is None or self.estop_active or not self.placement_gate_pass:
            if command is None and self.last_reason == "j6_command_accepted":
                self.last_reason = "command_timeout"
            return self.safe_zero(now_sim_s=now_sim_s)
        return command

    def trip(self, reason: str) -> None:
        self.authority.network_lost()
        self.last_reason = str(reason)

    def safe_zero(self, *, now_sim_s: float) -> AckermannHilCommand:
        return AckermannHilCommand(
            stamp_s=float(now_sim_s),
            sequence=max(0, self.authority.last_sequence),
            speed_mps=0.0,
            steering_angle_rad=0.0,
            acceleration_limit_mps2=self.maximum_acceleration_mps2,
            source_id="pc-safety-gate-zero-only",
            valid_until_s=float(now_sim_s) + self.zero_lifespan_s,
        )

    def snapshot(self, *, now_monotonic_s: float) -> dict[str, object]:
        active = self.authority.last_command
        return {
            "schema_version": 1,
            "j6_source_id": self.j6_source_id,
            "nonzero_command_authority": self.j6_source_id,
            "pc_command_capability": "zero_only_safety_gate",
            "last_sequence": self.authority.last_sequence,
            "last_command_source": None if active is None else active.source_id,
            "j6_health_fresh": self.health_fresh(now_monotonic_s=now_monotonic_s),
            "j6_health_reported_healthy": self.health_reported_healthy,
            "estop_active": self.estop_active,
            "pc_placement_gate_pass": self.placement_gate_pass,
            "resume_authorized": self.authority.resume_authorized,
            "link_connected": self.authority.link_connected,
            "safe_stop_active": self.authority.output(now_s=self.last_sim_time_s) is None,
            "reason": self.last_reason,
        }


__all__ = [
    "CommandSafetyGate",
    "HealthFrame",
    "command_from_mapping",
    "command_to_mapping",
]
