"""ROS-independent fail-safe velocity-gate state."""

from dataclasses import dataclass


@dataclass
class VelocityGateState:
    emergency_stopped: bool = False
    command_timeout_sec: float = 0.5
    last_command_monotonic: float | None = None
    max_linear_velocity: float = 0.45
    max_angular_velocity: float = 0.35

    def output(self, linear_x: float, angular_z: float, now: float):
        timed_out = (
            self.last_command_monotonic is None
            or now - self.last_command_monotonic > self.command_timeout_sec
        )
        if self.emergency_stopped or timed_out:
            return 0.0, 0.0
        return (
            max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x)),
            max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z)),
        )
