"""ROS-independent state machine for safely deploying the cleaning mechanism."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CleaningDecision:
    phase: str
    active: bool
    target_lift_m: float | None
    request_fresh: bool
    safety_fresh: bool
    joint_state_fresh: bool
    work_pose_reached: bool
    reason: str


class FormalCleaningCore:
    """Fail closed until the safety permit and physical work pose are fresh."""

    def __init__(
        self,
        *,
        request_timeout_sec: float,
        safety_timeout_sec: float,
        joint_state_timeout_sec: float,
        work_lift_m: float,
        transport_lift_m: float,
        lift_tolerance_m: float,
    ) -> None:
        values = (
            request_timeout_sec,
            safety_timeout_sec,
            joint_state_timeout_sec,
            work_lift_m,
            transport_lift_m,
            lift_tolerance_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("cleaning state-machine parameters must be finite")
        if min(request_timeout_sec, safety_timeout_sec, joint_state_timeout_sec) <= 0:
            raise ValueError("watchdog timeouts must be positive")
        if lift_tolerance_m <= 0:
            raise ValueError("lift tolerance must be positive")
        self.request_timeout_sec = request_timeout_sec
        self.safety_timeout_sec = safety_timeout_sec
        self.joint_state_timeout_sec = joint_state_timeout_sec
        self.work_lift_m = work_lift_m
        self.transport_lift_m = transport_lift_m
        self.lift_tolerance_m = lift_tolerance_m

    @staticmethod
    def _fresh(stamp: float | None, *, now: float, timeout: float) -> bool:
        return stamp is not None and 0.0 <= now - stamp <= timeout

    def evaluate(
        self,
        *,
        now: float,
        requested: bool,
        request_stamp: float | None,
        permitted: bool,
        safety_stamp: float | None,
        lift_position_m: float | None,
        joint_state_stamp: float | None,
    ) -> CleaningDecision:
        request_fresh = self._fresh(
            request_stamp, now=now, timeout=self.request_timeout_sec
        )
        safety_fresh = self._fresh(
            safety_stamp, now=now, timeout=self.safety_timeout_sec
        )
        joint_state_fresh = self._fresh(
            joint_state_stamp, now=now, timeout=self.joint_state_timeout_sec
        )
        valid_lift = lift_position_m is not None and math.isfinite(lift_position_m)
        work_pose_reached = bool(
            joint_state_fresh
            and valid_lift
            and abs(float(lift_position_m) - self.work_lift_m)
            <= self.lift_tolerance_m
        )

        if not permitted or not safety_fresh:
            reason = "safety_not_permitted" if safety_fresh else "safety_stale"
            return CleaningDecision(
                phase="SAFE_INHIBIT",
                active=False,
                target_lift_m=None,
                request_fresh=request_fresh,
                safety_fresh=safety_fresh,
                joint_state_fresh=joint_state_fresh,
                work_pose_reached=work_pose_reached,
                reason=reason,
            )

        if not requested or not request_fresh:
            reason = "cleaning_not_requested" if request_fresh else "request_stale"
            return CleaningDecision(
                phase="TRANSPORT",
                active=False,
                target_lift_m=self.transport_lift_m,
                request_fresh=request_fresh,
                safety_fresh=safety_fresh,
                joint_state_fresh=joint_state_fresh,
                work_pose_reached=work_pose_reached,
                reason=reason,
            )

        if not work_pose_reached:
            return CleaningDecision(
                phase="DEPLOYING",
                active=False,
                target_lift_m=self.work_lift_m,
                request_fresh=request_fresh,
                safety_fresh=safety_fresh,
                joint_state_fresh=joint_state_fresh,
                work_pose_reached=False,
                reason=(
                    "joint_state_stale"
                    if not joint_state_fresh
                    else "work_pose_not_reached"
                ),
            )

        return CleaningDecision(
            phase="CLEANING",
            active=True,
            target_lift_m=self.work_lift_m,
            request_fresh=True,
            safety_fresh=True,
            joint_state_fresh=True,
            work_pose_reached=True,
            reason="cleaning_enabled",
        )
