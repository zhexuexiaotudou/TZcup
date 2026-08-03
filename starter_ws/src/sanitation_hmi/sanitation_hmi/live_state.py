"""Thread-safe mission telemetry model used by the AUTO-17 live dashboard."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import math
from threading import Lock
import time


TERMINAL_STATES = {"COMPLETED", "FAILED", "CANCELED"}
ACTIVE_COMPONENT_STATES = {
    "EXECUTING_SWATH", "EXECUTING_TURN", "EXECUTING_ROTATE",
    "EXECUTING_SHIFT", "EXECUTING_BACKUP", "EXECUTING_BYPASS",
    "REPAIR_SWATH", "REPAIR_TRANSIT",
}


def _bounded_points(points, maximum: int = 320) -> list[list[float]]:
    normalized = [[float(point[0]), float(point[1])] for point in points]
    if len(normalized) <= maximum:
        return normalized
    stride = max(1, math.ceil(len(normalized) / maximum))
    sampled = normalized[::stride]
    if sampled[-1] != normalized[-1]:
        sampled.append(normalized[-1])
    return sampled


class LiveMissionState:
    """Keep a compact, browser-safe snapshot of the current demo mission."""

    def __init__(
        self,
        *,
        expected_components: int = 17,
        mission_id: str = "demo_coverage_001",
        geometry: dict | None = None,
        clock=time.monotonic,
    ) -> None:
        self._lock = Lock()
        self._clock = clock
        self._started_monotonic = clock()
        self._last_update_monotonic = self._started_monotonic
        self._state = "BOOTING"
        self._details: dict = {}
        self._expected_components = max(1, int(expected_components))
        self._mission_id = str(mission_id)
        self._geometry = deepcopy(geometry or {})
        self._seen_components: list[str] = []
        self._current_component: str | None = None
        self._estimated_pose: list[float] | None = None
        self._evaluation_pose: list[float] | None = None
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._brush_enabled = False
        self._emergency_stop = False
        self._planned_path: list[list[float]] = []
        self._trajectory: deque[list[float]] = deque(maxlen=1200)
        self._cleaned_trajectory: deque[list[float]] = deque(maxlen=1200)
        self._events: deque[dict] = deque(maxlen=16)
        self._topics_seen: set[str] = set()
        self._append_event("BOOTING", "等待 ROS 2 / Gazebo / Nav2 就绪")

    def _touch(self) -> None:
        self._last_update_monotonic = self._clock()

    def _append_event(self, state: str, label: str) -> None:
        now = self._clock()
        self._events.append(
            {
                "state": str(state),
                "label": str(label),
                "elapsed_sec": round(now - self._started_monotonic, 1),
            }
        )

    def mark_topic(self, topic: str) -> None:
        with self._lock:
            self._topics_seen.add(str(topic))
            self._touch()

    def update_state(self, state: str, details: dict | None = None) -> None:
        normalized = str(state or "UNKNOWN")
        with self._lock:
            if normalized != self._state:
                if normalized in TERMINAL_STATES and self._current_component:
                    if self._current_component not in self._seen_components:
                        self._seen_components.append(self._current_component)
                self._append_event(normalized, self._event_label(normalized, details))
            self._state = normalized
            self._details = deepcopy(details or {})
            self._topics_seen.add("/coverage/state")
            self._touch()

    def update_component(self, payload: dict) -> None:
        with self._lock:
            expected = payload.get("expected_components")
            if expected is not None and int(expected) > 0:
                self._expected_components = int(expected)
            kind = payload.get("kind")
            index = payload.get("index")
            state = str(payload.get("state") or self._state)
            if state in ACTIVE_COMPONENT_STATES and kind is not None and index is not None:
                key = str(payload.get("component_id") or f"{kind}:{int(index)}")
                if key != self._current_component:
                    if (
                        self._current_component
                        and self._current_component not in self._seen_components
                    ):
                        self._seen_components.append(self._current_component)
                    self._current_component = key
                    self._append_event(
                        state,
                        f"{'清扫带' if kind == 'swath' else '转弯'} {int(index) + 1}",
                    )
            self._details = deepcopy(payload)
            self._state = state
            self._topics_seen.add("/coverage/component_state")
            self._touch()

    def update_estimated_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._estimated_pose = [float(x), float(y), float(yaw)]
            self._topics_seen.add("/localization/fused_pose")
            self._touch()

    def update_evaluation_sample(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        brush_enabled: bool,
        coverage_state: str | None = None,
    ) -> None:
        point = [float(x), float(y)]
        with self._lock:
            self._evaluation_pose = [point[0], point[1], float(yaw)]
            if not self._trajectory or math.dist(self._trajectory[-1], point) >= 0.025:
                self._trajectory.append(point)
            if brush_enabled and (
                not self._cleaned_trajectory
                or math.dist(self._cleaned_trajectory[-1], point) >= 0.025
            ):
                self._cleaned_trajectory.append(point)
            self._brush_enabled = bool(brush_enabled)
            if coverage_state and self._state not in TERMINAL_STATES:
                self._state = str(coverage_state)
            self._topics_seen.add("/coverage/evaluation_sample")
            self._touch()

    def update_velocity(self, linear: float, angular: float) -> None:
        with self._lock:
            self._linear_speed = float(linear)
            self._angular_speed = float(angular)
            self._topics_seen.add("/cmd_vel")
            self._touch()

    def update_brush(self, enabled: bool) -> None:
        with self._lock:
            self._brush_enabled = bool(enabled)
            self._topics_seen.add("/brush_enabled")
            self._touch()

    def update_emergency_stop(self, enabled: bool) -> None:
        with self._lock:
            self._emergency_stop = bool(enabled)
            self._topics_seen.add("/emergency_stop")
            self._touch()

    def update_planned_path(self, points) -> None:
        with self._lock:
            self._planned_path = _bounded_points(points)
            self._topics_seen.add("/coverage/current_path")
            self._touch()

    def snapshot(self) -> dict:
        with self._lock:
            now = self._clock()
            completed = len(self._seen_components)
            if self._state in TERMINAL_STATES and self._current_component:
                completed = min(self._expected_components, completed)
            active_number = min(
                self._expected_components,
                completed + (1 if self._current_component else 0),
            )
            return {
                "schema_version": 1,
                "mission_id": self._mission_id,
                "status": self._state,
                "terminal": self._state in TERMINAL_STATES,
                "elapsed_sec": round(now - self._started_monotonic, 1),
                "last_update_age_sec": round(now - self._last_update_monotonic, 2),
                "progress": {
                    "completed_components": completed,
                    "active_component_number": active_number,
                    "expected_components": self._expected_components,
                    "ratio": round(
                        min(1.0, completed / self._expected_components), 4
                    ),
                    "current_component": self._current_component,
                },
                "vehicle": {
                    "estimated_pose_map": deepcopy(self._estimated_pose),
                    "evaluation_only_pose_map": deepcopy(self._evaluation_pose),
                    "linear_speed_m_s": round(self._linear_speed, 4),
                    "angular_speed_rad_s": round(self._angular_speed, 4),
                },
                "cleaning": {
                    "brush_enabled": self._brush_enabled,
                    "emergency_stop": self._emergency_stop,
                },
                "visualization": {
                    "planned_path": deepcopy(self._planned_path),
                    "evaluation_only_trajectory": list(self._trajectory),
                    "evaluation_only_cleaned_trajectory": list(
                        self._cleaned_trajectory
                    ),
                    "geometry": deepcopy(self._geometry),
                },
                "events": list(self._events),
                "topics_seen": sorted(self._topics_seen),
                "details": deepcopy(self._details),
                "claim_boundary": {
                    "source_level": "LIVE_GAZEBO_NAVIGATION_COVERAGE_DEMO",
                    "ground_truth_usage": "evaluation_and_visualization_only",
                    "learned_perception_pass": False,
                    "real_domain_pass": False,
                    "j6_runtime_pass": False,
                    "competition_matrix_pass": False,
                },
            }

    @staticmethod
    def _event_label(state: str, details: dict | None) -> str:
        labels = {
            "PLANNING": "生成覆盖路径",
            "TRANSIT_PREFLIGHT": "检查起点可达性",
            "TRANSIT": "驶向清扫起点",
            "ALIGNING": "对齐首条清扫带",
            "EXECUTING_SWATH": "执行清扫带",
            "EXECUTING_TURN": "执行转弯",
            "RECOVERY": "导航恢复",
            "COMPLETED": "任务完成",
            "FAILED": "任务失败",
            "CANCELED": "任务取消",
        }
        label = labels.get(state, state)
        if details and details.get("error"):
            label = f"{label}：{details['error']}"
        return label
