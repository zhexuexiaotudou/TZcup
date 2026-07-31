#!/usr/bin/env python3
"""Audit the browser supervision console without inventing missing runtime evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "starter_ws" / "src" / "sanitation_hmi" / "web"


def _read_url(base_url: str, path: str) -> tuple[int, bytes, str]:
    try:
        with urlopen(base_url.rstrip("/") + path, timeout=5) as response:
            return response.status, response.read(), response.headers.get_content_type()
    except HTTPError as exc:
        return exc.code, exc.read(), exc.headers.get_content_type()


def audit(base_url: str | None = None) -> dict:
    html = (WEB / "index.html").read_text(encoding="utf-8")
    css = (WEB / "app.css").read_text(encoding="utf-8")
    javascript = (WEB / "app.js").read_text(encoding="utf-8")
    gateway_source = (
        ROOT / "starter_ws" / "src" / "sanitation_hmi" / "sanitation_hmi" / "gateway.py"
    ).read_text(encoding="utf-8")
    adapter_source = (
        ROOT / "starter_ws" / "src" / "sanitation_hmi" / "sanitation_hmi" / "ros_adapter.py"
    ).read_text(encoding="utf-8")
    checks = {
        "map_is_primary_surface": 'id="map-canvas"' in html and "二维作业地图" in html,
        "reference_slam_compare_available": all(
            marker in html for marker in ('data-map-view="reference"', 'data-map-view="slam"', 'data-map-view="compare"')
        ),
        "truth_and_prediction_legends_separate": "仿真参考真值" in html and "感知预测" in html,
        "planned_and_actual_legends_separate": "规划结果" in html and "实际执行" in html,
        "gazebo_and_camera_panels_present": "/world_overview/image" in html and "/camera/color/image_raw" in html,
        "judge_learning_engineering_modes_present": all(mode in html for mode in ("评委", "学习", "工程")),
        "layer_controls_implemented": "layer-controls" in html and "layerNames" in javascript,
        "map_pan_zoom_fit_implemented": "pointerdown" in javascript and 'addEventListener("wheel"' in javascript and "fitMap" in javascript,
        "coverage_colors_derived_from_brush_trajectory": "drawCoverage" in javascript and "row[4]" in javascript,
        "replay_and_export_present": "/api/v1/replay" in javascript and "/api/v1/export" in html,
        "source_health_and_truth_boundary_present": "source-health-list" in html and "当前事实边界" in html,
        "task_controls_marked_fail_closed": (
            "data-needs-dispatch" in html
            and "dispatch_rejected" in gateway_source
            and "safe_task_orchestrator_unavailable" in adapter_source
        ),
        "no_browser_direct_cmd_vel": "/cmd_vel" not in html + javascript,
        "responsive_desktop_and_narrow_layouts": "@media (max-width: 900px)" in css and "100dvh" in css,
        "hidden_state_not_overridden": "[hidden] { display: none !important; }" in css,
        "reduced_motion_supported": "prefers-reduced-motion" in css,
    }
    runtime = {
        "checks_run": bool(base_url),
        "reachable": False,
        "state": None,
        "sources": {},
        "visual_monitoring_ready": False,
        "emergency_stop_connected": False,
        "task_execution_connected": False,
    }
    if base_url:
        try:
            health_status, health_body, _ = _read_url(base_url, "/healthz")
            state_status, state_body, content_type = _read_url(base_url, "/api/v1/state")
            css_status, _, css_type = _read_url(base_url, "/static/app.css")
            js_status, _, js_type = _read_url(base_url, "/static/app.js")
            health = json.loads(health_body)
            state = json.loads(state_body)
            runtime["reachable"] = health_status == state_status == css_status == js_status == 200
            runtime["content_contract"] = (
                health.get("status") == "ok"
                and content_type == "application/json"
                and css_type == "text/css"
                and js_type in {"text/javascript", "application/javascript"}
            )
            runtime["state"] = state.get("system_status")
            runtime["sources"] = {
                name: state.get("sources", {}).get(name, {}).get("status", "unavailable")
                for name in ("odom", "slam_map", "camera", "gazebo_overview", "safety")
            }
            runtime["visual_monitoring_ready"] = all(
                runtime["sources"][name] == "live"
                for name in ("odom", "slam_map", "camera", "gazebo_overview")
            )
            capabilities = state.get("capabilities", {})
            runtime["emergency_stop_connected"] = bool(capabilities.get("emergency_stop"))
            runtime["task_execution_connected"] = bool(capabilities.get("task_dispatch"))
            runtime["reference_and_slam_sources_separate"] = (
                "reference" in state and "slam_map" in state and state["reference"] is not state["slam_map"]
            )
        except (OSError, URLError, ValueError, json.JSONDecodeError) as exc:
            runtime["error"] = f"{exc.__class__.__name__}: {exc}"

    software_ready = all(checks.values())
    human_ready = bool(
        software_ready
        and runtime["reachable"]
        and runtime.get("content_contract")
        and runtime["visual_monitoring_ready"]
        and runtime["emergency_stop_connected"]
        and runtime["task_execution_connected"]
    )
    blockers = []
    if not software_ready:
        blockers.append("software_contract_failed")
    if not runtime["checks_run"]:
        blockers.append("live_runtime_not_checked")
    elif not runtime["visual_monitoring_ready"]:
        blockers.append("required_live_visual_sources_not_ready")
    if runtime["checks_run"] and not runtime["emergency_stop_connected"]:
        blockers.append("emergency_stop_interface_not_connected")
    if runtime["checks_run"] and not runtime["task_execution_connected"]:
        blockers.append("safe_task_orchestrator_not_connected")
    return {
        "schema_version": 1,
        "evidence_level": "SOFTWARE_CONTRACT_AND_OPTIONAL_LIVE_ROS_AUDIT",
        "software_checks": checks,
        "software_contract_pass": software_ready,
        "runtime": runtime,
        "human_visualization_ready": human_ready,
        "blockers": blockers,
        "truth_boundary": (
            "human_visualization_ready requires live odom, SLAM, camera, Gazebo overview, "
            "emergency stop, and safe task execution. Static UI checks cannot substitute for them."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.url)
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if report["software_contract_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
