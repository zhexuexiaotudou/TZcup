"""Pure helpers for semantic cleaning telemetry."""


SCHEMA = "tzcup.gazebo_cleaning_telemetry.v2"


def classify_motion_state(state: str, brush_enabled: bool) -> str:
    state = str(state).upper()
    if state.startswith("REPAIR_") or state == "REPAIR_SWATH":
        return "repair"
    if brush_enabled and state == "EXECUTING_SWATH":
        return "cleaning"
    return "transit"


def decimate_xy(points, limit=240):
    if limit <= 0:
        raise ValueError("limit must be positive")
    step = max(1, len(points) // limit)
    return [[float(point.x), float(point.y)] for point in points[::step]][:limit + 1]


def validate_telemetry_v2(payload):
    if payload.get("schema") != SCHEMA:
        raise ValueError("telemetry schema must be v2")
    paths = payload.get("paths")
    required = {
        "planned_swaths", "planned_connectors", "planned_repairs",
        "current_component", "actual_cleaning", "actual_transit", "actual_repair",
    }
    if not isinstance(paths, dict) or not required.issubset(paths):
        raise ValueError("semantic path layers are incomplete")
    return True
