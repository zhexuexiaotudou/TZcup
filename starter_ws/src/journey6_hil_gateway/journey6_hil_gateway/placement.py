"""ROS graph placement policy for the PC side of split Journey 6 HIL."""

from __future__ import annotations

from collections.abc import Iterable
import re


FORBIDDEN_PC_NODE_CLASSES: dict[str, tuple[re.Pattern[str], ...]] = {
    "product_perception": (
        re.compile(r"(^|/)(product_)?perception(_node)?$"),
        re.compile(r"(^|/)(detector|classifier|action_verifier)(_|$)"),
        re.compile(r"(^|/)dynamic_trash_map(_|$)"),
    ),
    "planning_and_navigation": (
        re.compile(r"(^|/)(planner|controller|behavior)_server$"),
        re.compile(r"(^|/)(bt_navigator|waypoint_follower|nav2)(_|$)"),
    ),
    "coverage_and_cleaning_intelligence": (
        re.compile(r"(^|/)(coverage|spot_clean|reobserve)(_|$)"),
        re.compile(r"(^|/)(cleaning_task_scheduler|product_orchestrator)$"),
    ),
    "oracle_or_ground_truth": (
        re.compile(r"(^|/)(garbage_)?oracle(_|$)"),
        re.compile(r"(^|/)ground_truth(_|$)"),
    ),
    "duplicate_control": (
        re.compile(r"(^|/)(ackermann|diff_drive)_controller$"),
        re.compile(r"(^|/)velocity_smoother$"),
    ),
}

ALLOWED_PC_NODE_PATTERNS = (
    re.compile(r"(^|/)(j6_)?hil(_|/|$)"),
    re.compile(r"(^|/)(gazebo|gz_|robot_state_publisher)(_|$)"),
    re.compile(r"(^|/)(evaluator|evidence|recorder|rosbag)(_|$)"),
    re.compile(r"(^|/)(sensor|camera|lidar|imu|gnss|wheel_odom)(_|$)"),
    re.compile(r"(^|/)(actuator|collision|emergency_stop)(_|$)"),
)


def _canonical_node_name(name: str) -> str:
    value = "/" + str(name).strip().strip("/").lower()
    return re.sub(r"/+", "/", value)


def audit_pc_nodes(node_names: Iterable[str]) -> dict[str, object]:
    """Return a machine-evaluable fail-closed PC node placement report."""
    graph_nodes = sorted(
        {_canonical_node_name(name) for name in node_names if str(name).strip()}
    )
    j6_nodes = [
        node for node in graph_nodes if node == "/j6" or node.startswith("/j6/")
    ]
    nodes = [node for node in graph_nodes if node not in j6_nodes]
    violations: list[dict[str, str]] = []
    for node in nodes:
        if any(pattern.search(node) for pattern in ALLOWED_PC_NODE_PATTERNS):
            continue
        for category, patterns in FORBIDDEN_PC_NODE_CLASSES.items():
            if any(pattern.search(node) for pattern in patterns):
                violations.append({"node": node, "category": category})
                break
    return {
        "schema_version": 1,
        "host_role": "pc_sensor_plant",
        "audited_nodes": nodes,
        "remote_j6_nodes": j6_nodes,
        "j6_namespace": "/j6",
        "violations": violations,
        "pc_duplicate_algorithm_nodes": len(violations),
        "placement_gate_pass": not violations,
    }


__all__ = ["FORBIDDEN_PC_NODE_CLASSES", "audit_pc_nodes"]
