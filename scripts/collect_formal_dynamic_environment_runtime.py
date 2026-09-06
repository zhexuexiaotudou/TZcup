#!/usr/bin/env python3
"""Collect evaluator-only pedestrian and contact truth without controlling the robot."""

from __future__ import annotations

import argparse
from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError as FutureTimeout
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


FORMAL_WALKER_RADIUS_M = 0.25
FORMAL_WALKER_PAIR_CLEARANCE_M = 0.50
MAXIMUM_POSE_AGE_S = 1.0
MINIMUM_COMPLETE_WALKER_POSE_SAMPLES = 2
DEFAULT_POSE_POLL_TIMEOUT_S = 5.0
NATIVE_POSE_POLL_INTERVAL_S = 0.5


def _atomic_write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def _load_schedule_walker_ids(
    schedule_path: Path,
) -> tuple[tuple[str, ...], dict[str, float], str, str]:
    """Load this run's public driver identities, never a hard-coded fixture set."""

    if not schedule_path.is_file() or schedule_path.is_symlink():
        raise ValueError("pedestrian schedule must be a regular file")
    try:
        schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
        pedestrians = schedule["pedestrians"]
        world_name = str(schedule["world_name"])
    except (OSError, json.JSONDecodeError, KeyError, TypeError) as exc:
        raise ValueError("pedestrian schedule is unreadable") from exc
    if not isinstance(pedestrians, list) or len(pedestrians) != 8:
        raise ValueError("formal dynamic collector requires exactly eight scheduled walkers")
    try:
        identities = tuple(str(row["object_id"]) for row in pedestrians)
        radii = tuple(float(row["radius_m"]) for row in pedestrians)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("pedestrian schedule has invalid walker identities") from exc
    if (
        any(not identity for identity in identities)
        or len(set(identities)) != len(identities)
        or any(not math.isfinite(radius) or radius <= 0.0 for radius in radii)
        or not world_name
    ):
        raise ValueError("pedestrian schedule has invalid unique walkers or radii")
    return (
        identities,
        dict(zip(identities, radii, strict=True)),
        hashlib.sha256(schedule_path.read_bytes()).hexdigest(),
        world_name,
    )


def _braced_blocks(message: str, label: str) -> list[str]:
    """Return complete protobuf blocks with an exact label, preserving order."""

    pattern = re.compile(rf"(?m)^\s*{re.escape(label)}\s*\{{")
    blocks: list[str] = []
    for match in pattern.finditer(message):
        depth = 0
        for index in range(match.end() - 1, len(message)):
            if message[index] == "{":
                depth += 1
            elif message[index] == "}":
                depth -= 1
                if depth == 0:
                    blocks.append(message[match.start() : index + 1])
                    break
        else:
            raise ValueError(f"Gazebo Pose_V {label} block is unterminated")
    return blocks


def _protobuf_scalar(block: str, field: str, default: float = 0.0) -> float:
    match = re.search(
        rf"^\s*{re.escape(field)}:\s*([-+0-9.eE]+)\s*$", block, re.MULTILINE
    )
    value = default if match is None else float(match.group(1))
    if not math.isfinite(value):
        raise ValueError(f"Gazebo Pose_V {field} is not finite")
    return value


def parse_live_walker_pose_frame(
    message: str, expected_ids: tuple[str, ...]
) -> tuple[int, dict[str, tuple[float, float]]]:
    """Parse one native Pose_V sample; do not infer a pose from the schedule."""

    headers = _braced_blocks(message, "header")
    if len(headers) != 1:
        raise ValueError("Gazebo Pose_V must contain exactly one top-level header")
    stamps = _braced_blocks(headers[0], "stamp")
    if len(stamps) != 1:
        raise ValueError("Gazebo Pose_V header lacks one simulation stamp")
    # Gazebo protobuf text omits scalar fields at their default zero value.
    seconds = _protobuf_scalar(stamps[0], "sec", 0.0)
    nanoseconds = _protobuf_scalar(stamps[0], "nsec", 0.0)
    if seconds < 0.0 or nanoseconds < 0.0 or nanoseconds >= 1_000_000_000:
        raise ValueError("Gazebo Pose_V simulation stamp is invalid")
    stamp_ns = int(seconds) * 1_000_000_000 + int(nanoseconds)
    if stamp_ns <= 0:
        raise ValueError("Gazebo Pose_V simulation stamp is zero")

    positions: dict[str, tuple[float, float]] = {}
    walker_like_names: set[str] = set()
    for block in _braced_blocks(message, "pose"):
        name_match = re.search(r'^\s*name:\s*"([^"]*)"\s*$', block, re.MULTILINE)
        if name_match is None or not name_match.group(1):
            raise ValueError("Gazebo Pose_V contains an empty or missing pose name")
        name = name_match.group(1)
        if "walker" in name.lower() or "pedestrian" in name.lower():
            walker_like_names.add(name)
        if name not in expected_ids:
            continue
        if name in positions:
            raise ValueError("Gazebo Pose_V contains a duplicate walker")
        position_blocks = _braced_blocks(block, "position")
        if len(position_blocks) != 1:
            raise ValueError("Gazebo Pose_V walker has no unique position")
        positions[name] = (
            _protobuf_scalar(position_blocks[0], "x"),
            _protobuf_scalar(position_blocks[0], "y"),
        )
    if walker_like_names != set(expected_ids):
        raise ValueError("Gazebo Pose_V walker identities do not match the schedule")
    if set(positions) != set(expected_ids):
        raise ValueError("Gazebo Pose_V does not contain exactly all scheduled walkers")
    return stamp_ns, positions


def _gazebo_executable() -> str:
    executable = shutil.which("gz")
    if executable is None:
        vendor = Path("/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz")
        if vendor.exists():
            executable = str(vendor)
    if executable is None:
        raise RuntimeError("Gazebo CLI not found")
    return executable


def _await_pose_future(
    future: Future[tuple[int, dict[str, tuple[float, float]], str]],
    *,
    timeout_s: float,
) -> tuple[str, tuple[int, dict[str, tuple[float, float]], str] | None]:
    """Bounded shutdown harvest classification; callers retain fail-closed state."""

    try:
        return "sample", future.result(timeout=timeout_s)
    except FutureTimeout:
        return "timeout", None
    except subprocess.TimeoutExpired:
        return "timeout", None
    except (OSError, RuntimeError, subprocess.SubprocessError):
        return "transport_error", None
    except ValueError:
        return "invalid", None
    except Exception:
        return "transport_error", None


def _pair_key(left: str, right: str) -> str:
    return "|".join(sorted((left, right)))


class WalkerProximityAccumulator:
    """Pure evaluator-only aggregation for complete live walker pose frames."""

    def __init__(self, walker_radii_m: dict[str, float]) -> None:
        self.walker_radii_m = walker_radii_m
        self.walker_ids = tuple(walker_radii_m)
        self.complete_frame_count = 0
        self.invalid_frame_count = 0
        self.stale_frame_count = 0
        self.transport_error_count = 0
        self.transport_timeout_count = 0
        self.last_complete_source_stamp_ns: int | None = None
        self.last_complete_receipt_monotonic: float | None = None
        self.maximum_complete_frame_gap_ns = 0
        self.raw_pose_frame_sha256: list[str] = []
        self.minimum_distances_m = {
            _pair_key(left, right): math.inf
            for index, left in enumerate(self.walker_ids)
            for right in self.walker_ids[index + 1 :]
        }

    def observe(
        self,
        *,
        poses: dict[str, tuple[float, float]],
        source_stamp_ns: int,
        receipt_sim_time_ns: int,
        receipt_monotonic: float,
        raw_frame_sha256: str,
    ) -> None:
        """Accept one complete, fresh, finite Gazebo truth frame or retain failure."""

        if set(poses) != set(self.walker_ids) or not all(
            math.isfinite(coordinate)
            for position in poses.values()
            for coordinate in position
        ):
            self.invalid_frame_count += 1
            return
        if (
            source_stamp_ns <= 0
            or receipt_sim_time_ns <= 0
            or not math.isfinite(receipt_monotonic)
        ):
            self.stale_frame_count += 1
            return
        age_ns = receipt_sim_time_ns - source_stamp_ns
        if age_ns < 0 or age_ns > int(MAXIMUM_POSE_AGE_S * 1_000_000_000):
            self.stale_frame_count += 1
            return
        if self.last_complete_source_stamp_ns is not None:
            gap_ns = source_stamp_ns - self.last_complete_source_stamp_ns
            if gap_ns <= 0:
                self.stale_frame_count += 1
                return
            self.maximum_complete_frame_gap_ns = max(
                self.maximum_complete_frame_gap_ns, gap_ns
            )
        self.last_complete_source_stamp_ns = source_stamp_ns
        self.last_complete_receipt_monotonic = receipt_monotonic
        self.raw_pose_frame_sha256.append(raw_frame_sha256)
        self.complete_frame_count += 1
        for index, left in enumerate(self.walker_ids):
            for right in self.walker_ids[index + 1 :]:
                distance = math.dist(poses[left], poses[right])
                key = _pair_key(left, right)
                self.minimum_distances_m[key] = min(
                    self.minimum_distances_m[key], distance
                )

    def report(self, *, window_end_monotonic: float, pose_source_topic: str) -> dict:
        source_fresh_at_end = (
            self.last_complete_receipt_monotonic is not None
            and 0.0
            <= window_end_monotonic - self.last_complete_receipt_monotonic
            <= MAXIMUM_POSE_AGE_S
        )
        sampling_complete = (
            self.complete_frame_count >= MINIMUM_COMPLETE_WALKER_POSE_SAMPLES
            and self.maximum_complete_frame_gap_ns
            <= int(MAXIMUM_POSE_AGE_S * 1_000_000_000)
            and source_fresh_at_end
        )
        source_trusted = (
            self.invalid_frame_count == 0
            and self.stale_frame_count == 0
            and self.transport_error_count == 0
            and sampling_complete
        )
        finite_distances = {
            key: value
            for key, value in self.minimum_distances_m.items()
            if math.isfinite(value)
        }
        violations = [
            {"walker_pair": key, "minimum_center_distance_m": distance}
            for key, distance in finite_distances.items()
            if distance <= self._pair_threshold_m(key)
        ]
        radius_contract = all(
            math.isclose(radius, FORMAL_WALKER_RADIUS_M, abs_tol=1e-9)
            for radius in self.walker_radii_m.values()
        )
        thresholds = {
            key: self._pair_threshold_m(key) for key in self.minimum_distances_m
        }
        threshold_contract = (
            len(thresholds) == 28
            and all(
                math.isclose(
                    threshold, FORMAL_WALKER_PAIR_CLEARANCE_M, abs_tol=1e-9
                )
                for threshold in thresholds.values()
            )
        )
        peer_gate_passed = (
            source_trusted
            and radius_contract
            and threshold_contract
            and not violations
            and len(finite_distances) == len(self.minimum_distances_m)
        )
        return {
            "pose_source_topic": pose_source_topic,
            "pose_source_type": "native gz.msgs.Pose_V via gz topic -e -n 1",
            "pose_source_native_gazebo_read": True,
            "native_pose_poll_interval_s": NATIVE_POSE_POLL_INTERVAL_S,
            "native_pose_transport_timeout_policy": "count_and_fail_closed",
            "pose_source_schedule_bound_walker_ids": list(self.walker_ids),
            "walker_radius_m_by_id": self.walker_radii_m,
            "formal_walker_radius_contract_all_0_25_m": radius_contract,
            "walker_pair_clearance_threshold_m_by_pair": thresholds,
            "formal_walker_pair_threshold_contract_28x_0_50_m": threshold_contract,
            "pose_source_is_live_gazebo_truth": source_trusted,
            "walker_pose_complete_frame_count": self.complete_frame_count,
            "walker_pose_invalid_frame_count": self.invalid_frame_count,
            "walker_pose_stale_frame_count": self.stale_frame_count,
            "native_pose_transport_error_count": self.transport_error_count,
            "native_pose_transport_timeout_count": self.transport_timeout_count,
            "walker_pose_source_fresh_at_window_end": source_fresh_at_end,
            "raw_pose_frame_sha256": self.raw_pose_frame_sha256,
            "walker_pose_maximum_complete_frame_gap_s": (
                self.maximum_complete_frame_gap_ns / 1_000_000_000
            ),
            "minimum_walker_center_distance_m_by_pair": finite_distances,
            "walker_center_distance_violations_lte_0_50_m": violations,
            "walker_center_distance_violation_count": len(violations),
            "walker_pose_sampling_sufficient": sampling_complete,
            "walker_peer_gate_passed": peer_gate_passed,
        }

    def record_transport_error(self, *, timed_out: bool) -> None:
        self.transport_error_count += 1
        self.transport_timeout_count += int(timed_out)

    def _pair_threshold_m(self, pair: str) -> float:
        left, right = pair.split("|", 1)
        return self.walker_radii_m[left] + self.walker_radii_m[right]


class EnvironmentTruthCollector(Node):
    """Observe environment truth in a process with no command publishers/actions."""

    def __init__(
        self,
        *,
        timeout_s: float,
        pedestrian_schedule: Path,
        pose_poll_timeout_s: float,
    ) -> None:
        super().__init__(
            "formal_dynamic_environment_truth_collector",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.timeout_s = timeout_s
        self.started = time.monotonic()
        self.done = False
        self.status_samples: list[dict] = []
        self.active_pedestrian_count = 0
        self.status_error_count = 0
        self.front_contact_sample_count = 0
        self.rear_contact_sample_count = 0
        self.front_collision_sample_count = 0
        self.rear_collision_sample_count = 0
        self.pose_poll_timeout_s = pose_poll_timeout_s
        self.walker_ids, self.walker_radii_m, self.schedule_sha256, self.world_name = (
            _load_schedule_walker_ids(
                pedestrian_schedule
            )
        )
        self.walker_proximity = WalkerProximityAccumulator(self.walker_radii_m)
        self._pose_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="formal_dynamic_pose_reader"
        )
        self._pose_future: Future[tuple[int, dict[str, tuple[float, float]], str]] | None = None
        self._accepting_pose_polls = True
        self._closed = False
        self.create_subscription(
            String,
            "/scenario/environment/pedestrian_driver/status",
            self._pedestrian_status,
            20,
        )
        self.create_subscription(
            Contacts,
            "/safety/front_bumper/contact",
            self._front_contact,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Contacts,
            "/safety/rear_bumper/contact",
            self._rear_contact,
            qos_profile_sensor_data,
        )
        # A separate native Gazebo read prevents lossy Pose_V-to-TF conversion
        # from erasing names and stamps.  The schedule supplies identities and
        # radii only; it never supplies, interpolates, or predicts live poses.
        self.create_timer(NATIVE_POSE_POLL_INTERVAL_S, self._poll_live_walker_poses)
        self.create_timer(0.1, self._tick)

    def _pedestrian_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            self.status_error_count += 1
            return
        state = value.get("state")
        if isinstance(state, str) and state.startswith("ERROR_"):
            self.status_error_count += 1
        elapsed = value.get("schedule_elapsed_s")
        if state != "ACTIVE" or not isinstance(elapsed, (int, float)):
            return
        count = value.get("pedestrian_count")
        if not isinstance(count, int):
            self.status_error_count += 1
            return
        self.active_pedestrian_count = count
        self.status_samples.append(
            {
                "observation_ros_time_ns": self.get_clock().now().nanoseconds,
                "schedule_elapsed_s": float(elapsed),
                "pedestrian_count": count,
            }
        )

    def _front_contact(self, message: Contacts) -> None:
        self.front_contact_sample_count += 1
        self.front_collision_sample_count += int(bool(message.contacts))

    def _rear_contact(self, message: Contacts) -> None:
        self.rear_contact_sample_count += 1
        self.rear_collision_sample_count += int(bool(message.contacts))

    def _poll_live_walker_poses(self) -> None:
        if self._pose_future is not None:
            if not self._pose_future.done():
                return
            self._harvest_pose_future(wait_timeout_s=0.0)
        if self._accepting_pose_polls and self._pose_future is None:
            self._pose_future = self._pose_executor.submit(
                self._read_live_walker_pose_frame
            )

    def _harvest_pose_future(self, *, wait_timeout_s: float) -> None:
        """Consume one native read with a bounded wait and fail-closed accounting."""
        future = self._pose_future
        if future is None:
            return
        outcome, sample = _await_pose_future(future, timeout_s=wait_timeout_s)
        if outcome == "sample":
            assert sample is not None
            stamp_ns, poses, frame_sha256 = sample
            try:
                self.walker_proximity.observe(
                    poses=poses,
                    source_stamp_ns=stamp_ns,
                    receipt_sim_time_ns=self.get_clock().now().nanoseconds,
                    receipt_monotonic=time.monotonic(),
                    raw_frame_sha256=frame_sha256,
                )
            except ValueError:
                self.walker_proximity.invalid_frame_count += 1
        elif outcome == "timeout":
            self.walker_proximity.record_transport_error(timed_out=True)
        elif outcome == "transport_error":
            self.walker_proximity.record_transport_error(timed_out=False)
        else:
            self.walker_proximity.invalid_frame_count += 1
        self._pose_future = None

    def _read_live_walker_pose_frame(
        self,
    ) -> tuple[int, dict[str, tuple[float, float]], str]:
        """Read precisely one complete native Gazebo Pose_V message off-thread."""
        try:
            result = subprocess.run(
                [
                    _gazebo_executable(),
                    "topic",
                    "-e",
                    "-t",
                    f"/world/{self.world_name}/pose/info",
                    "-n",
                    "1",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=self.pose_poll_timeout_s,
            )
            raw_frame = result.stdout
            stamp_ns, poses = parse_live_walker_pose_frame(
                raw_frame, self.walker_ids
            )
            return stamp_ns, poses, hashlib.sha256(raw_frame.encode("utf-8")).hexdigest()
        except (OSError, RuntimeError, subprocess.SubprocessError, ValueError):
            raise

    def _tick(self) -> None:
        if time.monotonic() - self.started >= self.timeout_s:
            self.done = True
            self._accepting_pose_polls = False

    def telemetry(self) -> dict:
        native_pose_topic = f"/world/{self.world_name}/pose/info"
        proximity = self.walker_proximity.report(
            window_end_monotonic=time.monotonic(),
            pose_source_topic=native_pose_topic,
        )
        return {
            "schema_version": 1,
            "collector_role": "evaluator_only_no_robot_control",
            "pedestrian_schedule_sha256": self.schedule_sha256,
            "gazebo_native_pose_topic": native_pose_topic,
            "active_pedestrian_count": self.active_pedestrian_count,
            "pedestrian_status_samples": self.status_samples,
            "pedestrian_status_error_count": self.status_error_count,
            "collision_count": (
                self.front_collision_sample_count + self.rear_collision_sample_count
            ),
            "topic_sample_counts": {
                "/scenario/environment/pedestrian_driver/status": len(
                    self.status_samples
                ),
                "/safety/front_bumper/contact": self.front_contact_sample_count,
                "/safety/rear_bumper/contact": self.rear_contact_sample_count,
                native_pose_topic: (
                    self.walker_proximity.complete_frame_count
                ),
            },
            "evaluator_truth_topics_subscribed": [
                "/scenario/environment/pedestrian_driver/status",
                "/safety/front_bumper/contact",
                "/safety/rear_bumper/contact",
            ],
            "evaluator_native_gazebo_topics_read": [
                native_pose_topic
            ],
            "control_topics_published": [],
            "product_actions_created": [],
            **proximity,
        }

    def close(self) -> None:
        """Stop submissions, then boundedly harvest the in-flight native read."""
        if self._closed:
            return
        self._closed = True
        self._accepting_pose_polls = False
        self._harvest_pose_future(wait_timeout_s=self.pose_poll_timeout_s)
        # A stuck CLI process must not make shutdown unbounded.  Its already
        # recorded timeout leaves the peer gate fail-closed.
        self._pose_executor.shutdown(wait=False, cancel_futures=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=330.0)
    parser.add_argument("--pedestrian-schedule", type=Path, required=True)
    parser.add_argument(
        "--pose-poll-timeout", type=float, default=DEFAULT_POSE_POLL_TIMEOUT_S
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        raise ValueError("timeout must be positive")
    if args.pose_poll_timeout <= 0.0:
        raise ValueError("pose poll timeout must be positive")
    rclpy.init()
    node = EnvironmentTruthCollector(
        timeout_s=args.timeout,
        pedestrian_schedule=args.pedestrian_schedule,
        pose_poll_timeout_s=args.pose_poll_timeout,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        value = node.telemetry()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    _atomic_write_json(args.output, value)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
