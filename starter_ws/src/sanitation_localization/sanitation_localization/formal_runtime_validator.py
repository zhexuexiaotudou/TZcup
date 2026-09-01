"""Fail-closed validation of formal localization runtime evidence.

This module is deliberately ROS-independent so recorded evidence remains
auditable on a development PC and in CI. It never reads simulation truth.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable


SCHEMA_VERSION = 1
VALID_MODES = {"mapping", "cleaning"}
FORBIDDEN_TOPIC_PREFIXES = (
    "/world",
    "/gazebo",
    "/model",
    "/ground_truth",
    "/truth",
    "/simulation/reference",
)


def _normalized_node(value: str) -> str:
    return "/" + value.strip("/")


def _node_matches(actual: str, expected: str) -> bool:
    actual_normalized = _normalized_node(actual)
    expected_normalized = _normalized_node(expected)
    if "/" in expected.strip("/"):
        return actual_normalized == expected_normalized
    return actual_normalized.rsplit("/", 1)[-1] == expected_normalized[1:]


def _topic(report: dict[str, Any], name: str) -> dict[str, Any]:
    return report.get("topics", {}).get(name, {})


def _nodes(endpoints: Iterable[dict[str, Any]]) -> set[str]:
    return {
        str(endpoint.get("node", ""))
        for endpoint in endpoints
        if endpoint.get("node")
    }


def _matching_nodes(nodes: Iterable[str], expected: str) -> set[str]:
    return {node for node in nodes if _node_matches(node, expected)}


def _authority_nodes(
    report: dict[str, Any], by_gid: dict[str, Any]
) -> tuple[set[str], set[str]]:
    registry = report.get("endpoint_registry", {})
    known: set[str] = set()
    unknown: set[str] = set()
    for gid, count in by_gid.items():
        if int(count) <= 0:
            continue
        endpoint = registry.get(gid)
        if endpoint and endpoint.get("node"):
            known.add(str(endpoint["node"]))
        else:
            unknown.add(gid)
    return known, unknown


def validate_runtime_report(
    report: dict[str, Any],
    *,
    minimum_messages: int = 3,
    minimum_tf_messages: int = 3,
    local_ekf_node: str = "local_ekf",
    global_ekf_node: str = "global_ekf",
    slam_node: str = "slam_toolbox",
    amcl_node: str = "amcl",
    navsat_node: str = "navsat_transform",
) -> dict[str, Any]:
    """Validate one collector report and return a durable acceptance record."""
    checks: list[dict[str, Any]] = []

    def check(check_id: str, passed: bool, evidence: Any) -> None:
        checks.append(
            {"id": check_id, "passed": bool(passed), "evidence": evidence}
        )

    mode = str(report.get("mode", ""))
    check(
        "schema_and_mode",
        report.get("schema_version") == SCHEMA_VERSION and mode in VALID_MODES,
        {"schema_version": report.get("schema_version"), "mode": mode},
    )

    isolation = report.get("collector_contract", {})
    subscribed_topics = [
        str(value) for value in isolation.get("subscribed_topics", [])
    ]
    forbidden_seen = sorted(
        topic
        for topic in subscribed_topics
        if any(topic.startswith(prefix) for prefix in FORBIDDEN_TOPIC_PREFIXES)
    )
    check(
        "no_world_truth_input",
        isolation.get("world_truth_used") is False and not forbidden_seen,
        {
            "world_truth_used": isolation.get("world_truth_used"),
            "forbidden_subscriptions": forbidden_seen,
        },
    )

    graph_nodes = {str(value) for value in report.get("graph_nodes", [])}
    odom = _topic(report, "/odom")
    odom_publishers = _nodes(odom.get("publishers", []))
    odom_authorities, odom_unknown = _authority_nodes(
        report, odom.get("messages_by_gid", {})
    )
    odom_all_owners = odom_publishers | odom_authorities
    odom_wrong_owners = sorted(
        node
        for node in odom_all_owners
        if not _node_matches(node, local_ekf_node)
    )
    check(
        "local_ekf_unique_odom_publisher",
        bool(_matching_nodes(odom_all_owners, local_ekf_node))
        and not odom_wrong_owners
        and not odom_unknown,
        {
            "owners": sorted(odom_all_owners),
            "wrong_owners": odom_wrong_owners,
            "unknown_gids": sorted(odom_unknown),
        },
    )
    check(
        "local_ekf_odom_output_active",
        int(odom.get("message_count", 0)) >= minimum_messages,
        {"message_count": int(odom.get("message_count", 0))},
    )

    for topic_name, check_id in (
        ("/odom/unfiltered", "local_ekf_receives_wheel_odom"),
        ("/imu/data", "local_ekf_receives_imu"),
    ):
        topic = _topic(report, topic_name)
        subscribers = _nodes(topic.get("subscriptions", []))
        check(
            check_id,
            int(topic.get("message_count", 0)) >= minimum_messages
            and bool(_matching_nodes(subscribers, local_ekf_node)),
            {
                "message_count": int(topic.get("message_count", 0)),
                "subscribers": sorted(subscribers),
            },
        )

    map_odom = report.get("tf_edges", {}).get("map->odom", {})
    tf_authorities, tf_unknown = _authority_nodes(
        report, map_odom.get("messages_by_gid", {})
    )
    expected_tf_node = slam_node if mode == "mapping" else global_ekf_node
    wrong_tf_owners = sorted(
        node
        for node in tf_authorities
        if not _node_matches(node, expected_tf_node)
    )
    check(
        "map_to_odom_unique_authority",
        int(map_odom.get("message_count", 0)) >= minimum_tf_messages
        and bool(_matching_nodes(tf_authorities, expected_tf_node))
        and not wrong_tf_owners
        and not tf_unknown,
        {
            "expected": expected_tf_node,
            "message_count": int(map_odom.get("message_count", 0)),
            "authorities": sorted(tf_authorities),
            "wrong_authorities": wrong_tf_owners,
            "unknown_gids": sorted(tf_unknown),
        },
    )

    if mode == "mapping":
        global_nodes = _matching_nodes(graph_nodes, global_ekf_node)
        check(
            "mapping_global_ekf_absent",
            not global_nodes,
            {"matching_nodes": sorted(global_nodes)},
        )
    elif mode == "cleaning":
        slam_nodes = _matching_nodes(graph_nodes, slam_node)
        check(
            "cleaning_slam_toolbox_absent",
            not slam_nodes,
            {"matching_nodes": sorted(slam_nodes)},
        )

        fused = _topic(report, "/localization/fused_odom")
        fused_publishers = _nodes(fused.get("publishers", []))
        fused_authorities, fused_unknown = _authority_nodes(
            report, fused.get("messages_by_gid", {})
        )
        fused_owners = fused_publishers | fused_authorities
        fused_wrong = sorted(
            node
            for node in fused_owners
            if not _node_matches(node, global_ekf_node)
        )
        check(
            "global_ekf_fused_odom_output",
            int(fused.get("message_count", 0)) >= minimum_messages
            and bool(_matching_nodes(fused_owners, global_ekf_node))
            and not fused_wrong
            and not fused_unknown,
            {
                "message_count": int(fused.get("message_count", 0)),
                "owners": sorted(fused_owners),
                "wrong_owners": fused_wrong,
                "unknown_gids": sorted(fused_unknown),
            },
        )

        for topic_name, check_id in (
            ("/odom", "global_ekf_receives_local_velocity"),
            ("/amcl_pose", "global_ekf_receives_amcl"),
            ("/odometry/gps", "global_ekf_receives_gnss_odometry"),
        ):
            topic = _topic(report, topic_name)
            subscribers = _nodes(topic.get("subscriptions", []))
            check(
                check_id,
                int(topic.get("message_count", 0)) >= minimum_messages
                and bool(_matching_nodes(subscribers, global_ekf_node)),
                {
                    "message_count": int(topic.get("message_count", 0)),
                    "subscribers": sorted(subscribers),
                },
            )

        gps = _topic(report, "/odometry/gps")
        gps_publishers = _nodes(gps.get("publishers", []))
        gnss = _topic(report, "/gnss/fix")
        gnss_subscribers = _nodes(gnss.get("subscriptions", []))
        check(
            "navsat_transform_uses_live_gnss",
            int(gnss.get("message_count", 0)) >= minimum_messages
            and int(gps.get("message_count", 0)) >= minimum_messages
            and bool(_matching_nodes(gnss_subscribers, navsat_node))
            and bool(_matching_nodes(gps_publishers, navsat_node)),
            {
                "gnss_message_count": int(gnss.get("message_count", 0)),
                "gps_odometry_message_count": int(gps.get("message_count", 0)),
                "gnss_subscribers": sorted(gnss_subscribers),
                "gps_publishers": sorted(gps_publishers),
            },
        )

        amcl = _topic(report, "/amcl_pose")
        amcl_publishers = _nodes(amcl.get("publishers", []))
        check(
            "amcl_pose_source_active",
            int(amcl.get("message_count", 0)) >= minimum_messages
            and bool(_matching_nodes(amcl_publishers, amcl_node)),
            {
                "message_count": int(amcl.get("message_count", 0)),
                "publishers": sorted(amcl_publishers),
            },
        )

    passed = all(item["passed"] for item in checks)
    return {
        "schema_version": SCHEMA_VERSION,
        "mode": mode,
        "status": "PASS" if passed else "BLOCKED",
        "passed": passed,
        "checks": checks,
        "summary": {
            "passed_checks": sum(item["passed"] for item in checks),
            "total_checks": len(checks),
        },
    }


def main(argv: list[str] | None = None) -> int:
    """Validate a collector JSON document and emit a fail-closed record."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--minimum-messages", type=int, default=3)
    parser.add_argument("--minimum-tf-messages", type=int, default=3)
    args = parser.parse_args(argv)

    report = json.loads(args.input.read_text(encoding="utf-8"))
    result = validate_runtime_report(
        report,
        minimum_messages=args.minimum_messages,
        minimum_tf_messages=args.minimum_tf_messages,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
