#!/usr/bin/env python3
"""Fail-closed acceptance validator for the live map->odom stabilizer run."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


FORBIDDEN_NODE_TOKENS = (
    "/ground_truth",
    "/world",
    "/gazebo",
    "/model",
    "/simulation/reference",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain a JSON object")
    return value


def load_status_yaml(path: Path) -> dict[str, Any]:
    documents = [
        value
        for value in yaml.safe_load_all(path.read_text(encoding="utf-8"))
        if value is not None
    ]
    document = documents[0] if documents else None
    if not isinstance(document, dict) or not isinstance(document.get("data"), str):
        raise ValueError("map_odom_stabilizer status capture is malformed")
    status = json.loads(document["data"])
    if not isinstance(status, dict):
        raise ValueError("map_odom_stabilizer status payload is malformed")
    return status


def _node_authority(authority: dict[str, Any], owner: str) -> dict[str, Any]:
    edge = authority.get("tf_edges", {}).get("map->odom", {})
    by_gid = {
        gid: int(count)
        for gid, count in edge.get("messages_by_gid", {}).items()
        if int(count) > 0
    }
    registry = authority.get("endpoint_registry", {})
    nodes = [registry.get(gid, {}).get("node") for gid in by_gid]
    raw = authority.get("topics", {}).get("/localization/raw_map_odom", {})
    raw_publishers = {
        endpoint.get("node") for endpoint in raw.get("publishers", [])
    }
    raw_subscribers = {
        endpoint.get("node") for endpoint in raw.get("subscriptions", [])
    }
    non_recorder_raw_subscribers = {
        node
        for node in raw_subscribers
        if node and not node.rstrip("/").endswith("/rosbag2_recorder")
        and node.rstrip("/") != "/rosbag2_recorder"
    }
    graph_nodes = set(authority.get("graph_nodes", []))
    passed = (
        len(by_gid) == 1
        and nodes == [owner]
        and int(edge.get("message_count", 0)) >= 100
        and raw_publishers == {"/global_ekf"}
        and non_recorder_raw_subscribers == {owner}
        and int(raw.get("message_count", 0)) >= 100
        and owner in graph_nodes
    )
    return {
        "passed": passed,
        "owner": owner,
        "tf_gid_count": len(by_gid),
        "tf_nodes": nodes,
        "tf_message_count": int(edge.get("message_count", 0)),
        "raw_map_odom_publishers": sorted(raw_publishers),
        "raw_map_odom_subscribers": sorted(raw_subscribers),
        "raw_map_odom_message_count": int(raw.get("message_count", 0)),
        "graph_nodes": sorted(graph_nodes),
    }


def _parameter_gate(effective: dict[str, Any]) -> dict[str, Any]:
    nodes = effective.get("nodes", {})
    stabilizer = nodes.get("/map_odom_stabilizer", {})
    expected = {
        "tau_sec": 1.5,
        "max_filter_dt_sec": 0.1,
        "max_gap_sec": 0.5,
        "input_tf_topic": "/localization/raw_map_odom",
    }
    matches = {
        name: stabilizer.get(name, {}).get("actual") == value
        for name, value in expected.items()
    }
    passed = (
        effective.get("schema_version") == 1
        and effective.get("all_expected") is True
        and all(matches.values())
    )
    return {"passed": passed, "expected": expected, "matches": matches}


def _focus_gate(focus: dict[str, Any]) -> dict[str, Any]:
    accuracy = focus.get("accuracy", {})
    samples = int(accuracy.get("samples", 0) or 0)
    values = {
        key: accuracy.get(key)
        for key in ("rmse_m", "p95_m", "max_m")
    }
    passed = (
        focus.get("status") == "PASS"
        and samples >= 100
        and all(
            value is not None and float(value) <= 0.05
            for value in values.values()
        )
    )
    return {"passed": passed, "samples": samples, "metrics": values}


def _status_gate(status: dict[str, Any]) -> dict[str, Any]:
    passed = (
        status.get("status") == "READY"
        and status.get("blocked_reason") is None
        and status.get("uses_ground_truth") is False
        and status.get("uses_future") is False
        and int(status.get("accepted_updates", 0)) >= 100
        and int(status.get("published_tf_messages", 0)) >= 100
        and int(status.get("rejected_updates", 0)) == 0
        and float(status.get("tau_sec", 0.0)) == 1.5
    )
    return {"passed": passed, "status": status}


def _truth_isolation_gate(node_info_text: str) -> dict[str, Any]:
    lowered = node_info_text.lower()
    forbidden = [token for token in FORBIDDEN_NODE_TOKENS if token in lowered]
    passed = (
        "/map_odom_stabilizer/status" in node_info_text
        and "/localization/raw_map_odom" in node_info_text
        and "/tf" in node_info_text
        and not forbidden
    )
    return {"passed": passed, "forbidden_tokens": forbidden}


def _resource_gate(resource: dict[str, Any]) -> dict[str, Any]:
    passed = (
        resource.get("status") == "RELEASED"
        and resource.get("gazebo_process_remaining") is False
        and resource.get("ros_runtime_process_remaining") is False
        and resource.get("formal_lock_available") is True
    )
    return {"passed": passed, "resource": resource}


def _driver_gate(driver_rc_text: str) -> dict[str, Any]:
    try:
        returncode = int(driver_rc_text.strip())
    except ValueError:
        returncode = None
    return {"passed": returncode == 0, "returncode": returncode}


def validate(run_dir: Path) -> dict[str, Any]:
    inputs = {
        name: run_dir / filename
        for name, filename in {
            "effective_parameters": "effective_parameters.json",
            "authority": "tf_authority.json",
            "focus": "localization_focus.json",
            "status": "map_odom_stabilizer.status.yaml",
            "node_info": "map_odom_stabilizer.node.txt",
            "resource": "resource_release.json",
            "driver_rc": "driver.rc",
        }.items()
    }
    missing = [name for name, path in inputs.items() if not path.is_file()]
    if missing:
        raise ValueError("missing required live evidence: " + ",".join(missing))
    effective = load_json(inputs["effective_parameters"])
    authority = load_json(inputs["authority"])
    focus = load_json(inputs["focus"])
    status = load_status_yaml(inputs["status"])
    node_info = inputs["node_info"].read_text(encoding="utf-8")
    resource = load_json(inputs["resource"])
    driver_rc_text = inputs["driver_rc"].read_text(encoding="utf-8")
    gates = {
        "parameters": _parameter_gate(effective),
        "authority": _node_authority(authority, "/map_odom_stabilizer"),
        "focus": _focus_gate(focus),
        "runtime_status": _status_gate(status),
        "truth_isolation": _truth_isolation_gate(node_info),
        "resource_release": _resource_gate(resource),
        "driver": _driver_gate(driver_rc_text),
    }
    passed = all(gate["passed"] for gate in gates.values())
    return {
        "schema_version": 1,
        "status": "LIVE_CANDIDATE_PASS" if passed else "FAIL",
        "passed": passed,
        "scope": "one bounded same-session live replay with causal map->odom stabilizer",
        "official_boundary": (
            "The stabilizer result is live runtime evidence for the strict "
            "max<=50 mm interpretation. Official text remains ambiguous about "
            "which reported statistic controls acceptance."
        ),
        "gates": gates,
        "evidence_hashes": {
            str(path): sha256_file(path) for path in inputs.values()
        },
        "control_truth_boundary": "no ground-truth subscription in stabilizer or control path",
        "rollback": {
            "launch_default": "map_odom_stabilizer:=false",
            "commit": "47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc",
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise SystemExit(f"fresh output required: {args.output}")
    try:
        receipt = validate(args.run_dir.resolve())
    except Exception as error:
        receipt = {
            "schema_version": 1,
            "status": "FAIL",
            "passed": False,
            "failure_type": type(error).__name__,
            "failure": str(error),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return 0 if receipt["status"] == "LIVE_CANDIDATE_PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
