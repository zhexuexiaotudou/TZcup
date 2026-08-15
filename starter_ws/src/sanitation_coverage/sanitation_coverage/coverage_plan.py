"""Versioned full-plan contract for coverage missions."""

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from .coverage_components import CoverageComponent


SCHEMA = "tzcup.coverage_plan.v1"


@dataclass(frozen=True)
class CoveragePlan:
    mission_id: str
    frame_id: str
    components: tuple[CoverageComponent, ...]
    route_mode: str = "AREA_FILL"
    planner_profile: str = "SKID_STEER_OPTIMIZED"
    generated_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.mission_id or not self.frame_id:
            raise ValueError("mission_id and frame_id are required")
        allowed_modes = {"AREA_FILL", "TAUGHT_ROUTE", "POINT_CLEAN"}
        if self.route_mode not in allowed_modes:
            raise ValueError(f"unsupported route_mode: {self.route_mode}")
        ids = [component.component_id for component in self.components]
        if len(ids) != len(set(ids)):
            raise ValueError("component_id values must be unique")

    @property
    def total_length_m(self) -> float:
        return sum(component.length_m for component in self.components)

    def to_dict(self) -> dict[str, Any]:
        serialized = [component.to_dict() for component in self.components]
        main_swaths = [item for item in serialized if item["kind"] == "SWATH"]
        connectors = [
            item for item in serialized
            if item["kind"] in {
                "TRANSIT", "ROTATE", "SHIFT", "BACKUP", "OBSTACLE_BYPASS",
                "RETURN_HOME", "FORWARD", "REVERSE", "CUSP_STOP",
            }
        ]
        repairs = [item for item in serialized if item["kind"] == "REPAIR_SWATH"]
        return {
            "schema": SCHEMA,
            "mission_id": self.mission_id,
            "frame_id": self.frame_id,
            "route_mode": self.route_mode,
            "planner_profile": self.planner_profile,
            "generated_at": self.generated_at,
            "component_count": len(self.components),
            "total_length_m": self.total_length_m,
            "components": serialized,
            "ordered_components": serialized,
            "main_swaths": main_swaths,
            "connectors": connectors,
            "repair_components": repairs,
            "planned_metrics": self.metadata.get("planned_metrics", {}),
            "selection_cost": self.metadata.get("selection_cost"),
            "cleanable_polygon": self.metadata.get("cleanable_polygon", []),
            "swath_angle_rad": self.metadata.get("swath_angle_rad"),
            "operation_width_m": self.metadata.get("operation_width_m"),
            "swath_spacing_m": self.metadata.get("swath_spacing_m"),
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CoveragePlan":
        if payload.get("schema") != SCHEMA:
            raise ValueError(f"unsupported coverage plan schema: {payload.get('schema')}")
        return cls(
            mission_id=str(payload["mission_id"]),
            frame_id=str(payload["frame_id"]),
            route_mode=str(payload.get("route_mode", "AREA_FILL")),
            planner_profile=str(payload.get("planner_profile", "SKID_STEER_OPTIMIZED")),
            generated_at=str(payload.get("generated_at", "")),
            components=tuple(
                CoverageComponent.from_dict(item) for item in payload["components"]
            ),
            metadata=dict(payload.get("metadata", {})),
        )
