"""Semantic coverage-plan components shared by planning, execution and UI."""

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any


class ComponentType(str, Enum):
    TRANSIT = "TRANSIT"
    SWATH = "SWATH"
    ROTATE = "ROTATE"
    SHIFT = "SHIFT"
    BACKUP = "BACKUP"
    OBSTACLE_BYPASS = "OBSTACLE_BYPASS"
    REPAIR_SWATH = "REPAIR_SWATH"
    RETURN_HOME = "RETURN_HOME"
    # Ackermann profile semantics. FORWARD/REVERSE are collision-checked
    # connector arcs executed through FollowPath with correct headings;
    # CUSP_STOP is a single-point standstill between direction changes;
    # DEFERRED_SWATH records a swath left to a later pass rather than cheating
    # it with an impossible maneuver.
    FORWARD = "FORWARD"
    REVERSE = "REVERSE"
    CUSP_STOP = "CUSP_STOP"
    DEFERRED_SWATH = "DEFERRED_SWATH"


CLEANING_COMPONENTS = {ComponentType.SWATH, ComponentType.REPAIR_SWATH}


def connector_handoff_replan_decision(
    current_pose: tuple[float, float, float],
    nominal_start_pose: tuple[float, float, float],
    *,
    max_position_error_m: float = 0.75,
    max_heading_error_rad: float = 0.35,
) -> dict[str, float | bool]:
    """Decide whether a stale static connector needs a live replan.

    The decision uses localization only; simulator ground truth is
    deliberately absent from the execution path.
    """
    position_error_m = math.dist(current_pose[:2], nominal_start_pose[:2])
    heading_error_rad = abs(math.atan2(
        math.sin(float(current_pose[2]) - float(nominal_start_pose[2])),
        math.cos(float(current_pose[2]) - float(nominal_start_pose[2])),
    ))
    requires_replan = (
        position_error_m > float(max_position_error_m)
        or heading_error_rad > float(max_heading_error_rad)
    )
    return {
        "requires_replan": requires_replan,
        "position_error_m": position_error_m,
        "heading_error_rad": heading_error_rad,
        "max_position_error_m": float(max_position_error_m),
        "max_heading_error_rad": float(max_heading_error_rad),
    }


def _point(value: Any) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("component points must be [x, y]")
    point = (float(value[0]), float(value[1]))
    if not all(math.isfinite(item) for item in point):
        raise ValueError("component points must be finite")
    return point


@dataclass(frozen=True)
class CoverageComponent:
    component_id: str
    kind: ComponentType
    points: tuple[tuple[float, float], ...]
    brush_enabled: bool = False
    speed_profile: str = "TRANSIT"
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", ComponentType(self.kind))
        object.__setattr__(self, "points", tuple(_point(point) for point in self.points))
        if not self.component_id:
            raise ValueError("component_id is required")
        single_point_kinds = {ComponentType.ROTATE, ComponentType.CUSP_STOP}
        if self.kind not in single_point_kinds and len(self.points) < 2:
            raise ValueError(f"{self.kind.value} requires at least two points")
        if self.kind in single_point_kinds and len(self.points) < 1:
            raise ValueError(f"{self.kind.value} requires an anchor point")
        expected_brush = self.kind in CLEANING_COMPONENTS
        if bool(self.brush_enabled) != expected_brush:
            raise ValueError(
                f"brush_enabled must be {expected_brush} for {self.kind.value}"
            )

    @property
    def length_m(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))

    def to_dict(self) -> dict[str, Any]:
        speed_mps = self.metadata.get("speed_limit_mps")
        expected_duration = (
            self.length_m / float(speed_mps)
            if speed_mps is not None and float(speed_mps) > 0.0 else None
        )
        return {
            "component_id": self.component_id,
            "kind": self.kind.value,
            "component_type": self.kind.value,
            "points": [list(point) for point in self.points],
            "path": [list(point) for point in self.points],
            "start_pose": list(self.points[0]),
            "end_pose": list(self.points[-1]),
            "brush_enabled": self.brush_enabled,
            "speed_profile": self.speed_profile,
            "length_m": self.length_m,
            "expected_length_m": self.length_m,
            "expected_duration_s": expected_duration,
            "collision_checked": bool(self.metadata.get("collision_checked", False)),
            "source_swath_id": self.metadata.get("source_swath_id"),
            "target_swath_id": self.metadata.get("target_swath_id"),
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CoverageComponent":
        return cls(
            component_id=str(payload["component_id"]),
            kind=ComponentType(payload["kind"]),
            points=tuple(payload["points"]),
            brush_enabled=bool(payload.get("brush_enabled", False)),
            speed_profile=str(payload.get("speed_profile", "TRANSIT")),
            metadata=dict(payload.get("metadata", {})),
        )
