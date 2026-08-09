"""Auditable pause/resume boundary between Coverage and spot cleaning."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CoverageBridgeState:
    coverage_state: str = "RUNNING"
    active_target_uuid: str | None = None
    pause_count: int = 0
    resume_count: int = 0


class PerceptionCoverageBridge:
    def __init__(self):
        self.state = CoverageBridgeState()
        self.timeline: list[dict] = []

    def request_safe_pause(self, target_uuid: str, *, allowed: bool, stamp_ns: int) -> bool:
        if not allowed or self.state.coverage_state != "RUNNING":
            self.timeline.append({
                "stamp_ns": stamp_ns,
                "event": "pause_rejected",
                "target_uuid": target_uuid,
            })
            return False
        self.state.coverage_state = "PAUSED_FOR_SPOT_CLEAN"
        self.state.active_target_uuid = target_uuid
        self.state.pause_count += 1
        self.timeline.append({"stamp_ns": stamp_ns, "event": "coverage_paused", "target_uuid": target_uuid})
        return True

    def resume(self, target_uuid: str, *, safety_clear: bool, brush_enabled: bool, stamp_ns: int) -> bool:
        if (
            self.state.coverage_state != "PAUSED_FOR_SPOT_CLEAN"
            or self.state.active_target_uuid != target_uuid
            or not safety_clear
            or brush_enabled
        ):
            self.timeline.append({"stamp_ns": stamp_ns, "event": "resume_rejected", "target_uuid": target_uuid})
            return False
        self.state.coverage_state = "RUNNING"
        self.state.active_target_uuid = None
        self.state.resume_count += 1
        self.timeline.append({"stamp_ns": stamp_ns, "event": "coverage_resumed", "target_uuid": target_uuid})
        return True
