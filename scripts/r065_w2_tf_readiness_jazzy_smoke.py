#!/usr/bin/env python3
"""Jazzy-only no-Gazebo smoke for the R065 W2 one-shot TF helper.

It proves the helper accepts an advancing simulated clock plus an exact dynamic
map->base_footprint transform, rejects a clock-only graph with no TF, and does
not add a /tf publisher.  It is a schema smoke only, not W2 runtime evidence.
"""

from __future__ import annotations

import json
import os
import argparse
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def _publish_clock(node, publisher, nanoseconds: int) -> None:
    from rosgraph_msgs.msg import Clock

    message = Clock()
    message.clock.sec = nanoseconds // 1_000_000_000
    message.clock.nanosec = nanoseconds % 1_000_000_000
    publisher.publish(message)
    node.get_logger().debug("clock published")


def _publish_tf(broadcaster, nanoseconds: int) -> None:
    from geometry_msgs.msg import TransformStamped

    message = TransformStamped()
    message.header.frame_id = "map"
    message.child_frame_id = "base_footprint"
    message.header.stamp.sec = nanoseconds // 1_000_000_000
    message.header.stamp.nanosec = nanoseconds % 1_000_000_000
    message.transform.rotation.w = 1.0
    broadcaster.sendTransform(message)


def _run_helper(helper: Path, output: Path, timeout_sec: float) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [sys.executable, str(helper), "--output", str(output), "--timeout-sec", str(timeout_sec)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env={**os.environ, "PYTHONUNBUFFERED": "1"},
    )


def _drive_fixture_until_terminal(
    fixture, clock_publisher, broadcaster, process: subprocess.Popen[str], *, publish_tf: bool,
    deadline_sec: float,
) -> tuple[str, str, list[str], dict[str, object]]:
    """Keep the exact fixture alive through DDS discovery, bounded by monotonic time."""
    import rclpy

    deadline = time.monotonic() + deadline_sec
    started = time.monotonic()
    stamp = 1_000_000_000
    observed_tf_publishers: set[str] = set()
    observed_clock_subscribers: set[str] = set()
    first_helper_clock_endpoint_after_sec: float | None = None
    last_diagnostics: dict[str, object] = {}
    while process.poll() is None and time.monotonic() < deadline:
        stamp += 50_000_000
        _publish_clock(fixture, clock_publisher, stamp)
        if publish_tf:
            _publish_tf(broadcaster, stamp)
        rclpy.spin_once(fixture, timeout_sec=0.01)
        observed_tf_publishers.update(info.node_name for info in fixture.get_publishers_info_by_topic("/tf"))
        clock_endpoint_names = {
            info.node_name for info in fixture.get_subscriptions_info_by_topic("/clock")
        }
        observed_clock_subscribers.update(clock_endpoint_names)
        if "r065_w2_tf_readiness" in clock_endpoint_names and first_helper_clock_endpoint_after_sec is None:
            first_helper_clock_endpoint_after_sec = time.monotonic() - started
        last_diagnostics = {
            "fixture_use_sim_time": bool(fixture.get_parameter("use_sim_time").value),
            "fixture_clock_ns": fixture.get_clock().now().nanoseconds,
            "clock_publisher_subscription_count": clock_publisher.get_subscription_count(),
            "clock_subscription_endpoints": [
                {
                    "node_name": info.node_name,
                    "node_namespace": info.node_namespace,
                    "topic_type": info.topic_type,
                    "qos": str(info.qos_profile),
                }
                for info in fixture.get_subscriptions_info_by_topic("/clock")
            ],
            "observed_clock_subscriber_node_names": sorted(observed_clock_subscribers),
            "first_helper_clock_endpoint_after_sec": first_helper_clock_endpoint_after_sec,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
            "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", ""),
        }
        time.sleep(0.02)
    if process.poll() is None:
        process.terminate()  # Exact helper child only; never broad process cleanup.
        stdout, stderr = process.communicate(timeout=3.0)
        raise RuntimeError(f"helper exceeded bounded fixture deadline; stdout={stdout} stderr={stderr}")
    stdout, stderr = process.communicate(timeout=3.0)
    return stdout, stderr, sorted(observed_tf_publishers), last_diagnostics


def main() -> int:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from tf2_ros import TransformBroadcaster

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--helper-timeout-sec", type=float, default=3.0)
    parser.add_argument("--fixture-deadline-sec", type=float, default=5.0)
    args = parser.parse_args()
    helper = Path(__file__).with_name("r065_w2_tf_readiness.py")
    if not helper.is_file():
        raise SystemExit("helper missing")
    rclpy.init()
    fixture = rclpy.create_node(
        "r065_w2_tf_readiness_fixture",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    clock_publisher = fixture.create_publisher(
        Clock, "/clock", QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    )
    broadcaster = TransformBroadcaster(fixture)
    try:
        project_work = helper.parent.parent / ".work"
        project_work.mkdir(exist_ok=True)
        root = Path(tempfile.mkdtemp(prefix="r065-w2-tf-smoke-", dir=project_work))
        try:
            # Clock-only graph: this is specifically the missing-TF negative case.
            missing_output = root / "missing_tf.json"
            missing = _run_helper(helper, missing_output, args.helper_timeout_sec)
            missing_stdout, missing_stderr, missing_publishers, missing_diagnostics = _drive_fixture_until_terminal(
                fixture, clock_publisher, broadcaster, missing, publish_tf=False,
                deadline_sec=args.fixture_deadline_sec
            )
            missing_report = json.loads(missing_output.read_text(encoding="utf-8"))
            (root / "missing_tf.stdout").write_text(missing_stdout, encoding="utf-8")
            (root / "missing_tf.stderr").write_text(missing_stderr, encoding="utf-8")
            if missing.returncode == 0 or missing_report.get("passed") is not False:
                raise RuntimeError(f"missing TF unexpectedly passed: {missing_stdout} {missing_stderr}")
            if missing_report.get("reason") != "map_to_base_footprint_tf_unavailable":
                raise RuntimeError(
                    f"missing TF has wrong reason: {missing_report}; fixture={missing_diagnostics}"
                )
            if missing_publishers != ["r065_w2_tf_readiness_fixture"]:
                raise RuntimeError(f"missing-TF helper added /tf publisher: {missing_publishers}")

            success_output = root / "success.json"
            success = _run_helper(helper, success_output, args.helper_timeout_sec)
            success_stdout, success_stderr, success_publishers, success_diagnostics = _drive_fixture_until_terminal(
                fixture, clock_publisher, broadcaster, success, publish_tf=True,
                deadline_sec=args.fixture_deadline_sec
            )
            success_report = json.loads(success_output.read_text(encoding="utf-8"))
            (root / "success.stdout").write_text(success_stdout, encoding="utf-8")
            (root / "success.stderr").write_text(success_stderr, encoding="utf-8")
            if success.returncode != 0 or success_report.get("passed") is not True:
                raise RuntimeError(f"valid TF failed: {success_stdout} {success_stderr} {success_report}")
            if success_report.get("tf_or_actuator_control_interfaces_created") is not False:
                raise RuntimeError(f"helper self-reported a write interface: {success_report}")
            if success_publishers != ["r065_w2_tf_readiness_fixture"]:
                raise RuntimeError(f"helper unexpectedly owns /tf publisher: {success_publishers}")
            print(json.dumps({
                "report_id": "r065_w2_tf_readiness_jazzy_schema_smoke_v1",
                "passed": True,
                "valid_tf_helper_rc": success.returncode,
                "missing_tf_helper_rc": missing.returncode,
                "helper_tf_publishers": success_publishers,
                "missing_tf_fixture_diagnostics": missing_diagnostics,
                "success_fixture_diagnostics": success_diagnostics,
            }, sort_keys=True))
        except BaseException:
            print(f"R065 W2 Jazzy smoke artifacts retained: {root}", file=sys.stderr)
            raise
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
