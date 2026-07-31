from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "dashboard_telemetry_frames.py"
SPEC = importlib.util.spec_from_file_location("dashboard_telemetry_frames", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_render_frame_contains_live_geometry_and_status() -> None:
    telemetry = {
        "status": "EXECUTING_SWATH",
        "elapsed_sec": 42.5,
        "progress": {
            "completed_components": 5,
            "expected_components": 17,
            "ratio": 5 / 17,
            "current_component": "swath:2",
        },
        "vehicle": {
            "linear_speed_m_s": 0.3,
            "estimated_pose_map": [0.5, -1.2, 0.2],
            "evaluation_only_pose_map": [0.52, -1.18, 0.21],
        },
        "cleaning": {"brush_enabled": True, "emergency_stop": False},
        "visualization": {
            "planned_path": [[-1.0, -1.0], [1.0, -1.0]],
            "evaluation_only_trajectory": [[-1.0, -1.1], [0.5, -1.2]],
            "evaluation_only_cleaned_trajectory": [[-1.0, -1.1], [0.4, -1.2]],
            "geometry": {
                "outer_polygon": [[-2, -3], [2, -3], [2, 3], [-2, 3]],
                "keepout_polygons": [[[-0.2, 0.2], [0.8, 0.2], [0.8, 1.2], [-0.2, 1.2]]],
            },
        },
    }

    frame = MODULE.render_frame(telemetry, 12)

    assert frame.size == (1600, 900)
    assert frame.mode == "RGB"
    colors = frame.resize((32, 18)).getcolors(maxcolors=1024)
    assert colors is not None and len(colors) > 8
