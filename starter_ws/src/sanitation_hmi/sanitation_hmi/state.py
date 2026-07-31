"""Thread-safe, source-aware state for the human visualization console."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
import threading
import time
from typing import Any


SOURCE_TIMEOUTS = {
    "clock": 3.0,
    "odom": 2.0,
    "slam_map": 10.0,
    "planned_path": 5.0,
    "camera": 3.0,
    "gazebo_overview": 3.0,
    "perception": 5.0,
    "safety": 3.0,
}


def _now() -> float:
    return time.time()


@dataclass
class VisualizationState:
    """Single source of truth shared by ROS callbacks and HTTP handlers."""

    reference: dict[str, Any] = field(default_factory=dict)
    lock: threading.RLock = field(default_factory=threading.RLock)
    updated_at: dict[str, float] = field(default_factory=dict)
    source_errors: dict[str, str] = field(default_factory=dict)
    vehicle: dict[str, Any] = field(
        default_factory=lambda: {"x": None, "y": None, "yaw": None, "speed_m_s": None}
    )
    slam_map: dict[str, Any] | None = None
    planned_path: list[list[float]] = field(default_factory=list)
    local_path: list[list[float]] = field(default_factory=list)
    trajectory: deque[list[float]] = field(default_factory=lambda: deque(maxlen=3000))
    predictions: list[dict[str, Any]] = field(default_factory=list)
    truth: list[dict[str, Any]] = field(default_factory=list)
    cleaned_targets: set[str] = field(default_factory=set)
    events: deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=160))
    coverage_state: str = "数据不可用"
    coverage_metrics: dict[str, Any] = field(
        default_factory=lambda: {
            "planned_ratio": None,
            "actual_ratio": None,
            "missed_ratio": None,
            "repeat_ratio": None,
            "basis": "unavailable",
        }
    )
    spot_state: str = "数据不可用"
    brush_enabled: bool | None = None
    emergency_stop: bool | None = None
    mode: str = "live"
    replay: dict[str, Any] | None = None
    images: dict[str, tuple[bytes, float]] = field(default_factory=dict)
    started_at: float = field(default_factory=_now)

    def touch(self, source: str, *, error: str | None = None, at: float | None = None) -> None:
        with self.lock:
            self.updated_at[source] = _now() if at is None else float(at)
            if error:
                self.source_errors[source] = error
            else:
                self.source_errors.pop(source, None)

    def add_event(
        self,
        event_type: str,
        title: str,
        detail: str,
        *,
        severity: str = "info",
        source: str = "runtime",
        at: float | None = None,
    ) -> None:
        with self.lock:
            self.events.append(
                {
                    "at": _now() if at is None else float(at),
                    "type": event_type,
                    "title": title,
                    "detail": detail,
                    "severity": severity,
                    "source": source,
                }
            )

    def update_vehicle(self, x: float, y: float, yaw: float, speed_m_s: float) -> None:
        with self.lock:
            self.vehicle = {
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
                "speed_m_s": float(speed_m_s),
            }
            stamp = _now()
            self.updated_at["odom"] = stamp
            sample = [float(x), float(y), float(yaw), stamp, bool(self.brush_enabled)]
            if not self.trajectory or self.trajectory[-1][:3] != sample[:3]:
                self.trajectory.append(sample)

    def update_map(
        self,
        *,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: list[int],
    ) -> None:
        expected = int(width) * int(height)
        if expected <= 0 or len(data) != expected:
            self.touch("slam_map", error="invalid_occupancy_grid")
            return
        with self.lock:
            self.slam_map = {
                "width": int(width),
                "height": int(height),
                "resolution": float(resolution),
                "origin": [float(origin_x), float(origin_y)],
                "data": [int(value) for value in data],
            }
            self.updated_at["slam_map"] = _now()
            self.source_errors.pop("slam_map", None)

    def set_path(self, name: str, points: list[list[float]]) -> None:
        normalized = [[float(point[0]), float(point[1])] for point in points]
        with self.lock:
            if name == "planned_path":
                self.planned_path = normalized
            elif name == "local_path":
                self.local_path = normalized
            else:
                raise ValueError(f"unknown path: {name}")
            self.updated_at[name] = _now()

    def set_targets(self, source: str, targets: list[dict[str, Any]]) -> None:
        with self.lock:
            if source == "perception":
                self.predictions = deepcopy(targets)
            elif source == "truth":
                self.truth = deepcopy(targets)
            else:
                raise ValueError(f"unknown target source: {source}")
            self.updated_at[source] = _now()

    def set_image(self, name: str, png: bytes) -> None:
        if not png.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("only encoded PNG images are accepted")
        with self.lock:
            stamp = _now()
            self.images[name] = (bytes(png), stamp)
            self.updated_at[name] = stamp

    def get_image(self, name: str) -> bytes | None:
        with self.lock:
            value = self.images.get(name)
            return value[0] if value else None

    def set_replay(self, replay: dict[str, Any] | None) -> None:
        with self.lock:
            self.replay = deepcopy(replay)

    def _source_status(self, now: float) -> dict[str, dict[str, Any]]:
        names = sorted(set(SOURCE_TIMEOUTS) | set(self.updated_at) | set(self.source_errors))
        result: dict[str, dict[str, Any]] = {}
        for name in names:
            last = self.updated_at.get(name)
            age = None if last is None else max(0.0, now - last)
            timeout = SOURCE_TIMEOUTS.get(name, 10.0)
            error = self.source_errors.get(name)
            if error:
                status = "error"
            elif last is None:
                status = "unavailable"
            elif age is not None and age > timeout:
                status = "stale"
            else:
                status = "live"
            result[name] = {
                "status": status,
                "age_s": None if age is None else round(age, 2),
                "error": error,
            }
        return result

    def snapshot(
        self, *, now: float | None = None, include_replay_samples: bool = False
    ) -> dict[str, Any]:
        with self.lock:
            current = _now() if now is None else float(now)
            sources = self._source_status(current)
            critical = ("odom", "slam_map", "safety")
            critical_states = [sources[name]["status"] for name in critical]
            if all(status == "live" for status in critical_states):
                system_status = "ready"
            elif any(status == "live" for status in critical_states):
                system_status = "degraded"
            else:
                system_status = "offline"
            replay = deepcopy(self.replay) or self._session_replay()
            replay_payload = replay
            if replay is not None and not include_replay_samples:
                replay_payload = {
                    key: deepcopy(value)
                    for key, value in replay.items()
                    if key != "samples"
                }
                replay_payload["sample_count"] = len(replay.get("samples", []))
            return {
                "schema_version": 1,
                "generated_at": current,
                "mode": self.mode,
                "system_status": system_status,
                "scene": self.reference.get("scene", {}),
                "reference": deepcopy(self.reference),
                "sources": sources,
                "vehicle": deepcopy(self.vehicle),
                "slam_map": deepcopy(self.slam_map),
                "planned_path": deepcopy(self.planned_path),
                "local_path": deepcopy(self.local_path),
                "trajectory": list(self.trajectory),
                "targets": {
                    "truth": deepcopy(self.truth),
                    "predictions": deepcopy(self.predictions),
                    "cleaned": sorted(self.cleaned_targets),
                },
                "mission": {
                    "coverage_state": self.coverage_state,
                    "coverage_metrics": deepcopy(self.coverage_metrics),
                    "spot_state": self.spot_state,
                    "brush_enabled": self.brush_enabled,
                    "next_action": self._next_action(system_status),
                },
                "safety": {
                    "emergency_stop": self.emergency_stop,
                    "status": (
                        "emergency_stopped"
                        if self.emergency_stop is True
                        else "ready"
                        if self.emergency_stop is False and sources["safety"]["status"] == "live"
                        else "unknown"
                    ),
                },
                "events": list(reversed(self.events)),
                "replay": replay_payload,
                "capabilities": {
                    "monitoring": True,
                    "emergency_stop": sources["safety"]["status"] == "live",
                    "task_dispatch": False,
                    "replay": replay is not None,
                    "export": True,
                },
            }

    def _session_replay(self) -> dict[str, Any] | None:
        if len(self.trajectory) < 2:
            return None
        return {
            "id": "current_session_odom",
            "label": "当前会话 ROS 里程计记录",
            "source": "/odom received by sanitation_hmi_adapter",
            "mode_label": "历史回放",
            "samples": [
                {
                    "x": row[0],
                    "y": row[1],
                    "yaw": row[2],
                    "t": row[3],
                    "brush": row[4],
                }
                for row in self.trajectory
            ],
            "sample_count_original": len(self.trajectory),
            "success": None,
            "execution_boundary": "仅回放本次服务进程收到的 ROS 里程计，不推断任务成功",
            "planned_metrics": None,
            "empirical_metrics": None,
            "warning": (
                "这是当前会话的真实 ROS 里程计历史，不是实时运行。"
                "未收到刷盘状态时不得解释为已清扫。"
            ),
        }

    def _next_action(self, system_status: str) -> str:
        if self.emergency_stop is True:
            return "等待人工确认后解除急停"
        if system_status == "offline":
            return "等待 ROS 数据连接"
        if system_status == "degraded":
            return "检查缺失或过期的数据源"
        state = self.coverage_state.upper()
        if "RUN" in state or "FOLLOW" in state:
            return "沿规划路径继续清扫"
        if "PAUSE" in state:
            return "等待恢复任务"
        return "等待安全任务编排器下发任务"
