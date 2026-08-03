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


CLEANING_COMPONENTS = {ComponentType.SWATH, ComponentType.REPAIR_SWATH}


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
        if self.kind not in {ComponentType.ROTATE} and len(self.points) < 2:
            raise ValueError(f"{self.kind.value} requires at least two points")
        if self.kind == ComponentType.ROTATE and len(self.points) < 1:
            raise ValueError("ROTATE requires an anchor point")
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
