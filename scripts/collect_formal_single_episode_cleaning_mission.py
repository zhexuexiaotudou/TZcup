#!/usr/bin/env python3
"""Collect one fail-closed cleaning mission from one live ROS / Gazebo run.

The collector is observation-only. It freezes every file and directory passed
to the product launch, records initial and terminal evaluator states, keeps the
complete per-target grasp evidence, queries live product parameters, and audits
the executable ROS graph for subscriptions to evaluator truth.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from pathlib import Path
from typing import Any


PRODUCT_TOPICS = {
    "planner": "/active_cleaning/planner_status",
    "mission_complete": "/active_cleaning/mission_complete",
    "trajectory": "/active_cleaning/trajectory",
    "grasp_result": "/active_cleaning/grasp_result",
    "odometry": "/odom",
}
EVALUATOR_TOPICS = {
    "ground_dirt": "/evaluation/single_episode/ground_dirt/status_json",
    "water": "/evaluation/single_episode/water_recovery/status_json",
    "dry_bin": "/evaluation/single_episode/dry_bin/status_json",
    "pedestrians": "/scenario/environment/pedestrian_driver/status",
    "collision": "/collision_monitor_state",
    "front_bumper": "/formal_vehicle/simulation/raw/front_bumper/contact",
    "rear_bumper": "/formal_vehicle/simulation/raw/rear_bumper/contact",
}
CONTROL_PROHIBITED_TRUTH_TOPICS = tuple(
    EVALUATOR_TOPICS[key] for key in ("ground_dirt", "water", "dry_bin", "pedestrians")
)
RUNTIME_PARAMETER_CONTRACT = {
    "/formal_active_cleaning_policy_planner": (
        "policy_checkpoint", "episode_seed", "maximum_task_distance_m"
    ),
    "/pc_open_vocab_product_adapter": ("artifact_root",),
    "/formal_map_lifecycle_manager": ("mode", "episode_manifest", "artifact_directory"),
}
MULTISITE_INTERFACES = {
    "dosod": {
        "name": "/perception/open_vocab/dosod_boxes",
        "type": "vision_msgs/msg/Detection2DArray",
        "interface_kind": "topic",
        "observed_topic": "/perception/open_vocab/dosod_boxes",
    },
    "edgesam": {
        "name": "/perception/ground_dirt/masks",
        "type": "sensor_msgs/msg/Image",
        "interface_kind": "topic",
        "observed_topic": "/perception/ground_dirt/masks",
    },
    "nav2": {
        "name": "/follow_path",
        "type": "nav2_msgs/action/FollowPath",
        "interface_kind": "action",
        "observed_topic": "/follow_path/_action/status",
    },
    "dynamic_pedestrians": {
        "name": "/scenario/environment/pedestrian_driver/status",
        "type": "std_msgs/msg/String",
        "interface_kind": "topic",
        "observed_topic": "/scenario/environment/pedestrian_driver/status",
    },
    "cleaning_actuator": {
        "name": "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/motor_current_a",
        "type": "std_msgs/msg/Float64MultiArray",
        "interface_kind": "topic",
        "observed_topic": "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/motor_current_a",
    },
}
REQUIRED_RUNTIME_NODES = {
    "/formal_active_cleaning_policy_planner",
    "/formal_physical_grasp_executor",
    "/pc_open_vocab_product_adapter",
    "/formal_map_lifecycle_manager",
    "/formal_product_demo_operator_gate",
    "/collision_monitor",
}


class CollectionError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_descriptor(path: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    if resolved.is_symlink() or not resolved.is_file():
        raise CollectionError(f"input is not a regular file: {path}")
    return {
        "kind": "file",
        "path": str(resolved),
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def directory_descriptor(path: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    if resolved.is_symlink() or not resolved.is_dir():
        raise CollectionError(f"input is not a regular directory: {path}")
    rows: list[dict[str, Any]] = []
    for item in sorted(
        resolved.rglob("*"), key=lambda value: value.relative_to(resolved).as_posix()
    ):
        if item.is_symlink():
            raise CollectionError(f"symlink prohibited in frozen input directory: {item}")
        if item.is_dir():
            continue
        if not item.is_file():
            raise CollectionError(f"non-regular entry in frozen input directory: {item}")
        rows.append({
            "relative_path": item.relative_to(resolved).as_posix(),
            "size_bytes": item.stat().st_size,
            "sha256": sha256_file(item),
        })
    if not rows:
        raise CollectionError(f"frozen input directory is empty: {path}")
    canonical = json.dumps(rows, ensure_ascii=False, separators=(",", ":"), sort_keys=True)
    return {
        "kind": "directory",
        "path": str(resolved),
        "file_count": len(rows),
        "total_bytes": sum(row["size_bytes"] for row in rows),
        "tree_sha256": hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
        "files": rows,
    }


def build_input_binding(args: argparse.Namespace) -> dict[str, Any]:
    files = {
        "episode_manifest": args.episode_manifest,
        "evaluator_episode_manifest": args.evaluator_episode_manifest,
        "evaluator_ground_truth": args.evaluator_ground_truth,
        "world": args.world,
        "pedestrian_schedule": args.pedestrian_schedule,
        "session_status": args.session_status,
        "same_map_baseline": args.same_map_baseline,
        "policy_checkpoint": args.policy_checkpoint,
        "runtime_binding": args.runtime_binding,
    }
    directories = {
        "saved_map": args.saved_map,
        "perception_artifacts": args.perception_artifacts,
    }
    return {
        "schema_version": 1,
        "artifact_kind": "single_episode_immutable_input_binding",
        "artifacts": {
            **{name: file_descriptor(Path(path)) for name, path in files.items()},
            **{name: directory_descriptor(Path(path)) for name, path in directories.items()},
        },
    }


def verify_input_binding(binding: dict[str, Any]) -> None:
    if binding.get("artifact_kind") != "single_episode_immutable_input_binding":
        raise CollectionError("invalid immutable input binding")
    artifacts = binding.get("artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        raise CollectionError("immutable input artifact ledger missing")
    for name, expected in artifacts.items():
        if not isinstance(expected, dict):
            raise CollectionError(f"invalid input descriptor: {name}")
        path = Path(str(expected.get("path", "")))
        if expected.get("kind") == "file":
            actual = file_descriptor(path)
        elif expected.get("kind") == "directory":
            actual = directory_descriptor(path)
        else:
            raise CollectionError(f"invalid input descriptor kind: {name}")
        if actual != expected:
            raise CollectionError(f"input changed after pre-launch freeze: {name}")


def parse_diagnostic(message: Any, *, expected_name: str) -> dict[str, Any] | None:
    for row in message.status:
        if row.name == expected_name:
            values: dict[str, Any] = {item.key: item.value for item in row.values}
            values.update({
                "diagnostic_name": row.name,
                "hardware_id": row.hardware_id,
                "level": int(row.level),
                "state": row.message,
            })
            return values
    return None


def source_row(
    metric: str, topic: str, source_class: str,
    identity: dict[str, Any], count: int,
) -> dict[str, Any]:
    return {
        "metric": metric,
        "topic": topic,
        "source_class": source_class,
        **{key: identity[key] for key in (
            "session_id", "episode_id", "episode_seed", "gazebo_process_id",
            "runtime_id", "ros_domain_id", "gz_partition",
        )},
        "sample_count": count,
    }


def _parameter_value(value: Any) -> Any:
    return {
        1: value.bool_value,
        2: value.integer_value,
        3: value.double_value,
        4: value.string_value,
        5: list(value.byte_array_value),
        6: list(value.bool_array_value),
        7: list(value.integer_array_value),
        8: list(value.double_array_value),
        9: list(value.string_array_value),
    }.get(int(value.type))


def _full_node_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip("/")
    return f"{namespace}/{name}" if namespace else f"/{name}"


def _add_input_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--episode-manifest", required=True, type=Path)
    parser.add_argument("--evaluator-episode-manifest", required=True, type=Path)
    parser.add_argument("--evaluator-ground-truth", required=True, type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--pedestrian-schedule", required=True, type=Path)
    parser.add_argument("--session-status", required=True, type=Path)
    parser.add_argument("--same-map-baseline", required=True, type=Path)
    parser.add_argument("--policy-checkpoint", required=True, type=Path)
    parser.add_argument("--runtime-binding", required=True, type=Path)
    parser.add_argument("--saved-map", required=True, type=Path)
    parser.add_argument("--perception-artifacts", required=True, type=Path)


def main() -> int:
    if "--prepare-input-binding" in __import__("sys").argv:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--prepare-input-binding", required=True, type=Path)
        _add_input_arguments(parser)
        args = parser.parse_args()
        result = build_input_binding(args)
        args.prepare_input_binding.parent.mkdir(parents=True, exist_ok=True)
        args.prepare_input_binding.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return 0

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--episode-seed", required=True, type=int)
    parser.add_argument("--runtime-id", required=True)
    parser.add_argument("--gazebo-process-id", required=True, type=int)
    parser.add_argument("--session-start-epoch-ns", required=True, type=int)
    parser.add_argument("--input-binding", required=True, type=Path)
    parser.add_argument("--ready-file", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=21600.0)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--multisite-topic-observations", type=Path,
        help="optional canonical read-only product-interface observation record",
    )
    _add_input_arguments(parser)
    args = parser.parse_args()

    binding = json.loads(args.input_binding.read_text(encoding="utf-8"))
    if not isinstance(binding, dict):
        raise SystemExit("input binding root must be an object")
    verify_input_binding(binding)
    truth = json.loads(args.evaluator_ground_truth.read_text(encoding="utf-8"))
    cubes = truth.get("discrete_cubes") if isinstance(truth, dict) else None
    if not isinstance(cubes, list) or not cubes:
        raise SystemExit("evaluator truth has no discrete cube identity ledger")
    expected_cube_ids = {
        str(row.get("object_id", "")) for row in cubes if isinstance(row, dict)
    }
    if len(expected_cube_ids) != len(cubes) or "" in expected_cube_ids:
        raise SystemExit("evaluator truth cube IDs are missing or duplicated")

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry, Path as NavPath
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
    )
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Float64MultiArray, String
    from vision_msgs.msg import Detection2DArray

    identity = {
        "session_id": args.session_id,
        "episode_id": args.episode_id,
        "episode_seed": args.episode_seed,
        "runtime_id": args.runtime_id,
        "gazebo_process_id": args.gazebo_process_id,
        "session_start_epoch_ns": args.session_start_epoch_ns,
        "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
        "gz_partition": os.environ.get("GZ_PARTITION", ""),
    }
    if not identity["gz_partition"]:
        raise SystemExit("GZ_PARTITION is required for a uniquely bound live episode")

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("formal_single_episode_cleaning_collector")
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.started = time.monotonic()
            self.done = False
            self.complete = False
            self.planner: dict[str, Any] = {}
            self.first: dict[str, dict[str, Any]] = {}
            self.latest: dict[str, dict[str, Any]] = {}
            self.counts = {key: 0 for key in (*PRODUCT_TOPICS, *EVALUATOR_TOPICS)}
            self.path_count = 0
            self.trajectory_evidence: list[dict[str, Any]] = []
            self.planner_status_samples: list[dict[str, Any]] = []
            self.grasp_results: list[dict[str, Any]] = []
            self.collision_count = 0
            self.intervention_count = 0
            self.return_started_seen = False
            self.return_start_state: dict[str, Any] | None = None
            self.odom_points: list[tuple[float, float]] = []
            self.operator_start_received = False
            self.mission_start_odom_index: int | None = None
            self.return_start_odom_index: int | None = None
            self.runtime_parameters: dict[str, dict[str, Any]] = {}
            self.parameter_futures: dict[str, Any] = {}
            self.parameter_clients: dict[str, Any] = {}
            self.runtime_graph: dict[str, Any] = {}
            self.ready_written = False
            self.multisite_counts = {role: 0 for role in MULTISITE_INTERFACES}
            self.multisite_nav2_goal_succeeded = False
            self.create_subscription(
                DiagnosticArray, PRODUCT_TOPICS["planner"], self.on_planner, latched
            )
            self.create_subscription(
                Bool, PRODUCT_TOPICS["mission_complete"], self.on_complete, latched
            )
            self.create_subscription(NavPath, PRODUCT_TOPICS["trajectory"], self.on_path, 20)
            self.create_subscription(String, PRODUCT_TOPICS["grasp_result"], self.on_grasp, 50)
            self.create_subscription(Odometry, PRODUCT_TOPICS["odometry"], self.on_odom, 50)
            self.create_subscription(
                Bool, "/product_demo/operator_start", self.on_operator_start, 10
            )
            for key in ("ground_dirt", "water", "dry_bin", "pedestrians"):
                self.create_subscription(
                    String, EVALUATOR_TOPICS[key],
                    lambda msg, k=key: self.on_json(k, msg), 20,
                )
            self.create_subscription(
                CollisionMonitorState, EVALUATOR_TOPICS["collision"], self.on_collision, 20
            )
            self.create_subscription(
                Contacts, EVALUATOR_TOPICS["front_bumper"],
                lambda msg: self.on_contact("front_bumper", msg), qos_profile_sensor_data,
            )
            self.create_subscription(
                Contacts, EVALUATOR_TOPICS["rear_bumper"],
                lambda msg: self.on_contact("rear_bumper", msg), qos_profile_sensor_data,
            )
            if args.multisite_topic_observations is not None:
                self.create_subscription(Detection2DArray, MULTISITE_INTERFACES["dosod"]["observed_topic"], lambda msg: self.on_multisite("dosod"), 20)
                self.create_subscription(Image, MULTISITE_INTERFACES["edgesam"]["observed_topic"], lambda msg: self.on_multisite("edgesam"), 20)
                self.create_subscription(GoalStatusArray, MULTISITE_INTERFACES["nav2"]["observed_topic"], self.on_nav2_status, 20)
                self.create_subscription(String, MULTISITE_INTERFACES["dynamic_pedestrians"]["observed_topic"], lambda msg: self.on_multisite("dynamic_pedestrians"), 20)
                self.create_subscription(Float64MultiArray, MULTISITE_INTERFACES["cleaning_actuator"]["observed_topic"], lambda msg: self.on_multisite("cleaning_actuator"), 20)
            for node_name, names in RUNTIME_PARAMETER_CONTRACT.items():
                client = self.create_client(GetParameters, f"{node_name}/get_parameters")
                self.parameter_clients[node_name] = (client, names)
            self.create_timer(0.2, self.tick)
            self.create_timer(1.0, self.audit_runtime)

        def on_planner(self, msg: Any) -> None:
            self.counts["planner"] += 1
            row = parse_diagnostic(
                msg, expected_name="formal_active_cleaning_policy_planner"
            )
            if row is not None:
                self.planner = row
                self.planner_status_samples.append(
                    {**row, "collector_received_epoch_ns": time.time_ns()}
                )
                if row.get("returning_home") == "true" and not self.return_started_seen:
                    self.return_started_seen = True
                    self.return_start_odom_index = max(0, len(self.odom_points) - 1)
                    self.return_start_state = {
                        "planner_status": dict(row),
                        "evaluator": {
                            key: dict(self.latest.get(key, {}))
                            for key in ("ground_dirt", "water", "dry_bin")
                        },
                        "successful_grasp_target_ids": sorted({
                            str(item.get("target_id")) for item in self.grasp_results
                            if item.get("verified_in_bin") is True
                        }),
                    }

        def on_complete(self, msg: Any) -> None:
            self.counts["mission_complete"] += 1
            self.complete = self.complete or bool(msg.data)

        def on_path(self, msg: Any) -> None:
            self.counts["trajectory"] += 1
            self.path_count += 1
            points = [
                [float(pose.pose.position.x), float(pose.pose.position.y)]
                for pose in msg.poses
            ]
            self.trajectory_evidence.append(
                {
                    "collector_received_epoch_ns": time.time_ns(),
                    "frame_id": str(msg.header.frame_id),
                    "pose_count": len(points),
                    "trajectory_xy_m": points,
                }
            )

        def on_operator_start(self, msg: Any) -> None:
            if bool(msg.data) and not self.operator_start_received:
                self.operator_start_received = True
                self.mission_start_odom_index = max(0, len(self.odom_points) - 1)

        def on_grasp(self, msg: Any) -> None:
            self.counts["grasp_result"] += 1
            try:
                row = json.loads(msg.data)
                if isinstance(row, dict):
                    row["collector_received_epoch_ns"] = time.time_ns()
                    self.grasp_results.append(row)
            except (json.JSONDecodeError, TypeError):
                pass

        def on_odom(self, msg: Any) -> None:
            self.counts["odometry"] += 1
            self.odom_points.append(
                (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
            )

        def on_json(self, key: str, msg: Any) -> None:
            self.counts[key] += 1
            try:
                row = json.loads(msg.data)
                if isinstance(row, dict):
                    self.first.setdefault(key, dict(row))
                    self.latest[key] = row
                    if key == "pedestrians":
                        self.collision_count = max(
                            self.collision_count, int(row.get("collision_count", 0))
                        )
            except (json.JSONDecodeError, TypeError, ValueError):
                pass

        def on_collision(self, msg: Any) -> None:
            self.counts["collision"] += 1
            if int(msg.action_type) != 0:
                self.intervention_count += 1

        def on_contact(self, key: str, msg: Any) -> None:
            self.counts[key] += 1
            if msg.contacts:
                self.collision_count += 1

        def on_multisite(self, role: str) -> None:
            self.multisite_counts[role] += 1

        def on_nav2_status(self, msg: Any) -> None:
            self.on_multisite("nav2")
            self.multisite_nav2_goal_succeeded = self.multisite_nav2_goal_succeeded or any(
                int(row.status) == GoalStatus.STATUS_SUCCEEDED for row in msg.status_list
            )

        def _parameter_done(
            self, node_name: str, names: tuple[str, ...], future: Any
        ) -> None:
            try:
                response = future.result()
                if len(response.values) != len(names):
                    return
                self.runtime_parameters[node_name] = {
                    name: _parameter_value(value)
                    for name, value in zip(names, response.values)
                }
            except Exception:
                return

        def audit_runtime(self) -> None:
            for node_name, (client, names) in self.parameter_clients.items():
                if node_name in self.runtime_parameters or node_name in self.parameter_futures:
                    continue
                if client.service_is_ready():
                    request = GetParameters.Request(names=list(names))
                    future = client.call_async(request)
                    self.parameter_futures[node_name] = future
                    future.add_done_callback(
                        lambda done, n=node_name, p=names:
                        self._parameter_done(n, p, done)
                    )
            nodes = sorted({
                _full_node_name(name, namespace)
                for name, namespace in self.get_node_names_and_namespaces()
            })
            subscribers: dict[str, list[str]] = {}
            for topic in CONTROL_PROHIBITED_TRUTH_TOPICS:
                subscribers[topic] = sorted({
                    _full_node_name(info.node_name, info.node_namespace)
                    for info in self.get_subscriptions_info_by_topic(topic)
                })
            self.runtime_graph = {
                "observed_epoch_ns": time.time_ns(),
                "nodes": nodes,
                "required_nodes": sorted(REQUIRED_RUNTIME_NODES),
                "required_nodes_present": REQUIRED_RUNTIME_NODES.issubset(nodes),
                "control_prohibited_truth_topic_subscribers": subscribers,
            }

        def multisite_observations(self) -> dict[str, Any]:
            action_servers = {
                name: sorted(types)
                for name, types in self.get_action_server_names_and_types()
            }
            interfaces: dict[str, dict[str, Any]] = {}
            for role, contract in MULTISITE_INTERFACES.items():
                topic = contract["observed_topic"]
                publishers = sorted({
                    _full_node_name(info.node_name, info.node_namespace)
                    for info in self.get_publishers_info_by_topic(topic)
                })
                action_ready = (
                    role != "nav2"
                    or action_servers.get(contract["name"]) == [contract["type"]]
                )
                interfaces[role] = {
                    "name": contract["name"], "type": contract["type"],
                    "interface_kind": contract["interface_kind"],
                    "live_observed": self.multisite_counts[role] > 0 and bool(publishers) and action_ready,
                    "message_count": self.multisite_counts[role],
                    "publisher_nodes": publishers,
                    "goal_succeeded": self.multisite_nav2_goal_succeeded if role == "nav2" else None,
                }
            return {
                "schema_version": 1,
                "artifact_kind": "formal_multisite_live_product_interface_observations",
                "collected_epoch_ns": time.time_ns(),
                "interfaces": interfaces,
            }

        def is_ready(self) -> bool:
            initial_ready = all(
                key in self.first for key in ("ground_dirt", "water", "dry_bin", "pedestrians")
            )
            parameters_ready = set(self.runtime_parameters) == set(RUNTIME_PARAMETER_CONTRACT)
            graph_ready = self.runtime_graph.get("required_nodes_present") is True
            subscribers = self.runtime_graph.get(
                "control_prohibited_truth_topic_subscribers", {}
            )
            truth_boundary_ready = (
                len(subscribers) == len(CONTROL_PROHIBITED_TRUTH_TOPICS)
                and all(
                    rows == ["/formal_single_episode_cleaning_collector"]
                    for rows in subscribers.values()
                )
            )
            return (
                initial_ready and parameters_ready and graph_ready
                and truth_boundary_ready and bool(self.planner) and bool(self.odom_points)
            )

        def tick(self) -> None:
            if self.is_ready() and not self.ready_written:
                ready_payload = json.dumps({
                    "schema_version": 1,
                    "artifact_kind": "single_episode_collector_ready",
                    "run_identity": identity,
                    "created_epoch_ns": time.time_ns(),
                }, indent=2, sort_keys=True) + "\n"
                temporary = args.ready_file.with_suffix(args.ready_file.suffix + ".tmp")
                temporary.write_text(ready_payload, encoding="utf-8")
                temporary.replace(args.ready_file)
                self.ready_written = True
            if self.complete and self.planner.get("state") == "COMPLETE":
                self.done = True
            elif time.monotonic() - self.started >= args.timeout:
                self.done = True

        def report(self) -> dict[str, Any]:
            verify_input_binding(binding)
            # Re-sample the graph at the terminal boundary; readiness alone
            # must not hide a product node that exited or subscribed later.
            self.audit_runtime()
            if args.multisite_topic_observations is not None:
                observations = self.multisite_observations()
                args.multisite_topic_observations.parent.mkdir(parents=True, exist_ok=True)
                if args.multisite_topic_observations.exists():
                    raise CollectionError("refusing to overwrite multisite topic observations")
                args.multisite_topic_observations.write_text(
                    json.dumps(observations, indent=2, sort_keys=True) + "\n", encoding="utf-8"
                )
            sources = [
                source_row(k, v, "product", identity, self.counts[k])
                for k, v in PRODUCT_TOPICS.items()
            ]
            sources += [
                source_row(k, v, "evaluator_truth", identity, self.counts[k])
                for k, v in EVALUATOR_TOPICS.items()
            ]
            successful = sorted({
                str(row.get("target_id")) for row in self.grasp_results
                if row.get("verified_in_bin") is True and row.get("target_id")
            })
            mission_start = self.mission_start_odom_index
            return_start = self.return_start_odom_index
            task_points = (
                self.odom_points[mission_start : return_start + 1]
                if mission_start is not None and return_start is not None
                else []
            )
            return_points = (
                self.odom_points[return_start:]
                if return_start is not None
                else []
            )
            return {
                "schema_version": 2,
                "artifact_kind": "single_live_episode_raw_collection",
                "created_epoch_ns": time.time_ns(),
                "run_identity": identity,
                "input_binding": {
                    "path": str(args.input_binding.resolve()),
                    "sha256": sha256_file(args.input_binding),
                    "artifacts": binding["artifacts"],
                },
                "metric_sources": sources,
                "runtime_graph": self.runtime_graph,
                "runtime_parameters": self.runtime_parameters,
                "product": {
                    "planner_status": self.planner,
                    "mission_complete": self.complete,
                    "trajectory_publish_count": self.path_count,
                    "trajectory_evidence": self.trajectory_evidence,
                    "planner_status_samples": self.planner_status_samples,
                    "grasp_results": self.grasp_results,
                    "successful_grasp_target_ids": successful,
                    "odom_sample_count": len(self.odom_points),
                    "operator_start_received": self.operator_start_received,
                    "task_odom_trajectory_xy_m": [list(point) for point in task_points],
                    "return_odom_trajectory_xy_m": [list(point) for point in return_points],
                    "return_started_seen": self.return_started_seen,
                    "return_start_state": self.return_start_state,
                },
                "evaluator": {
                    "initial": self.first,
                    "terminal": self.latest,
                    "collision_count": self.collision_count,
                    "collision_monitor_intervention_count": self.intervention_count,
                },
                "collector_ready_before_operator_start": self.ready_written,
                "timed_out": not self.complete,
            }

    args.ready_file.parent.mkdir(parents=True, exist_ok=True)
    if args.ready_file.exists():
        raise SystemExit(f"refusing to reuse collector readiness evidence: {args.ready_file}")
    rclpy.init()
    node = Collector()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if (
        report["product"]["mission_complete"]
        and report["collector_ready_before_operator_start"]
    ) else 3


if __name__ == "__main__":
    raise SystemExit(main())
