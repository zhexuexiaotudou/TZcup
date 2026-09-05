#!/usr/bin/env python3
"""Wait for a bounded ROS graph contract and atomically retain the result."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile
import time


def _parse_topic_requirement(value: str) -> tuple[str, str]:
    name, separator, message_type = value.partition("=")
    if not separator or not name.startswith("/") or not message_type:
        raise argparse.ArgumentTypeError("topic must be /name=package/msg/Type")
    return name, message_type


def _fully_qualified_node(name: str, namespace: str) -> str:
    if name.startswith("/"):
        return name
    namespace = namespace.rstrip("/")
    return f"{namespace}/{name}" if namespace else f"/{name}"


def graph_contract_status(
    topic_types: dict[str, list[str]],
    nodes: list[tuple[str, str]],
    required_topics: list[tuple[str, str]],
    required_nodes: list[str],
) -> dict:
    observed_nodes = sorted(
        {_fully_qualified_node(name, namespace) for name, namespace in nodes}
    )
    missing_topics = [
        {"name": name, "type": message_type}
        for name, message_type in required_topics
        if message_type not in topic_types.get(name, [])
    ]
    missing_nodes = sorted(set(required_nodes) - set(observed_nodes))
    return {
        "ready": not missing_topics and not missing_nodes,
        "missing_topics": missing_topics,
        "missing_nodes": missing_nodes,
        "required_topic_types_observed": {
            name: sorted(topic_types.get(name, [])) for name, _ in required_topics
        },
        "required_nodes_observed": sorted(set(required_nodes) & set(observed_nodes)),
    }


def _process_alive(pid: int | None) -> bool:
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _write_atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            directory_fd = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError:
            pass
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("--pid", type=int)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", action="append", type=_parse_topic_requirement, default=[])
    parser.add_argument("--node", action="append", default=[])
    args = parser.parse_args()
    if args.timeout <= 0.0 or not (args.topic or args.node):
        parser.error("positive timeout and at least one topic or node are required")
    for node in args.node:
        if not node.startswith("/"):
            parser.error("node names must be fully qualified")

    import rclpy
    from rclpy.node import Node

    started = time.monotonic()
    deadline = started + args.timeout
    rclpy.init(args=None)
    probe = Node("formal_ros_graph_readiness_probe")
    status = graph_contract_status({}, [], args.topic, args.node)
    pid_alive = _process_alive(args.pid)
    try:
        while rclpy.ok() and pid_alive and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.20)
            topic_types = dict(probe.get_topic_names_and_types())
            nodes = list(probe.get_node_names_and_namespaces())
            status = graph_contract_status(topic_types, nodes, args.topic, args.node)
            if status["ready"]:
                break
            pid_alive = _process_alive(args.pid)
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    pid_alive = _process_alive(args.pid)
    elapsed = time.monotonic() - started
    report = {
        "schema_version": 1,
        "status": "READY" if status["ready"] and pid_alive else "BLOCKED",
        "elapsed_wall_s": elapsed,
        "supervised_pid": args.pid,
        "supervised_pid_alive": pid_alive,
        "required_topics": [
            {"name": name, "type": message_type} for name, message_type in args.topic
        ],
        "required_nodes": args.node,
        **status,
    }
    _write_atomic_json(args.output, report)
    return 0 if report["status"] == "READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
