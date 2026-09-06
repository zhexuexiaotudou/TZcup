"""Contract tests for evaluator-only native Gazebo walker-pose collection."""

from __future__ import annotations

import ast
from concurrent.futures import Future, TimeoutError as FutureTimeout
import math
from pathlib import Path
import re
import subprocess

import pytest


ROOT = Path(__file__).parents[1]
COLLECTOR = ROOT / "scripts" / "collect_formal_dynamic_environment_runtime.py"
VALIDATOR = ROOT / "scripts" / "validate_formal_dynamic_obstacle_avoidance.py"


def _definitions(*names: str) -> dict:
    tree = ast.parse(COLLECTOR.read_text(encoding="utf-8"))
    wanted = [
        node
        for node in tree.body
        if isinstance(node, (ast.Assign, ast.FunctionDef, ast.ClassDef))
        and (
            isinstance(node, ast.Assign)
            or getattr(node, "name", "") in names
        )
    ]
    namespace = {
        "Future": Future,
        "FutureTimeout": FutureTimeout,
        "math": math,
        "re": re,
        "subprocess": subprocess,
    }
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=wanted, type_ignores=[])),
            str(COLLECTOR),
            "exec",
        ),
        namespace,
    )
    return namespace


def _native_pose_frame(
    names: list[str], *, stamp_sec: int = 1, stamp_nsec: int = 0
) -> str:
    poses = "\n".join(
        f'''pose {{
  name: "{name}"
  position {{
    x: {index + 1}.0
    y: 0.0
    z: 0.0
  }}
}}'''
        for index, name in enumerate(names)
    )
    return f'''header {{
  stamp {{
    sec: {stamp_sec}
    nsec: {stamp_nsec}
  }}
}}
{poses}
'''


def test_collector_uses_one_native_pose_v_read_off_thread_without_tf() -> None:
    source = COLLECTOR.read_text(encoding="utf-8")
    tree = ast.parse(source)
    assert '"topic",' in source and '"-e",' in source and '"-n",' in source
    assert 'f"/world/{self.world_name}/pose/info"' in source
    assert "ThreadPoolExecutor" in source and "_read_live_walker_pose_frame" in source
    assert "TFMessage" not in source
    assert "create_publisher" not in source
    assert '"native_pose_transport_timeout_policy": "count_and_fail_closed"' in source
    assert "evaluator_native_gazebo_topics_read" in source
    assert 'parameter_overrides=[Parameter("use_sim_time", value=True)]' in source
    assert "_atomic_write_json(args.output, value)" in source
    assert ".pending." in source and "pending.replace(path)" in source
    assert not any(
        isinstance(node, ast.ImportFrom) and node.module == "rclpy.action"
        for node in ast.walk(tree)
    )


def test_shutdown_harvestes_completed_pose_read_before_telemetry() -> None:
    source = COLLECTOR.read_text(encoding="utf-8")
    assert "def _harvest_pose_future" in source
    assert "self._accepting_pose_polls = False" in source
    assert "self._harvest_pose_future(wait_timeout_s=self.pose_poll_timeout_s)" in source
    assert "self._pose_executor.shutdown(wait=False, cancel_futures=True)" in source
    assert source.index("node.close()") < source.index("value = node.telemetry()")


def test_shutdown_future_harvest_classifies_success_failure_and_timeout() -> None:
    await_future = _definitions("_await_pose_future")["_await_pose_future"]

    completed: Future = Future()
    sample = (1, {"walker_hash_0": (0.0, 0.0)}, "a" * 64)
    completed.set_result(sample)
    assert await_future(completed, timeout_s=0.0) == ("sample", sample)

    failed: Future = Future()
    failed.set_exception(RuntimeError("native read failed"))
    assert await_future(failed, timeout_s=0.0) == ("transport_error", None)

    pending: Future = Future()
    assert await_future(pending, timeout_s=0.0) == ("timeout", None)

    cli_timeout: Future = Future()
    cli_timeout.set_exception(subprocess.TimeoutExpired(["gz", "topic"], 0.8))
    assert await_future(cli_timeout, timeout_s=0.0) == ("timeout", None)


def test_native_pose_parser_accepts_complete_real_shape() -> None:
    parser = _definitions("_braced_blocks", "_protobuf_scalar", "parse_live_walker_pose_frame")[
        "parse_live_walker_pose_frame"
    ]
    expected = tuple(f"walker_hash_{index}" for index in range(8))
    stamp, poses = parser(_native_pose_frame(list(expected), stamp_sec=12, stamp_nsec=5), expected)
    assert stamp == 12_000_000_005
    assert poses["walker_hash_0"] == (1.0, 0.0)
    assert set(poses) == set(expected)


def test_native_pose_parser_accepts_protobuf_default_omitted_scalars() -> None:
    parser = _definitions("_braced_blocks", "_protobuf_scalar", "parse_live_walker_pose_frame")[
        "parse_live_walker_pose_frame"
    ]
    expected = tuple(f"walker_hash_{index}" for index in range(8))
    frame = _native_pose_frame(list(expected), stamp_sec=0, stamp_nsec=5)
    frame = frame.replace("    sec: 0\n", "", 1)
    frame = frame.replace("    x: 1.0\n    y: 0.0\n", "", 1)
    stamp, poses = parser(frame, expected)
    assert stamp == 5
    assert poses["walker_hash_0"] == (0.0, 0.0)


@pytest.mark.parametrize(
    ("frame", "error"),
    [
        (_native_pose_frame([""] + [f"walker_hash_{i}" for i in range(1, 8)]), "empty"),
        (_native_pose_frame(["walker_hash_0", "walker_hash_0"] + [f"walker_hash_{i}" for i in range(2, 8)]), "duplicate"),
        (_native_pose_frame([f"walker_hash_{i}" for i in range(7)]), "identities"),
        (_native_pose_frame([f"walker_hash_{i}" for i in range(7)] + ["walker_hash_unknown"]), "identities"),
        (_native_pose_frame([f"walker_hash_{i}" for i in range(8)], stamp_sec=0), "zero"),
    ],
)
def test_native_pose_parser_fails_closed_on_invalid_identity_or_stamp(
    frame: str, error: str
) -> None:
    parser = _definitions("_braced_blocks", "_protobuf_scalar", "parse_live_walker_pose_frame")[
        "parse_live_walker_pose_frame"
    ]
    expected = tuple(f"walker_hash_{index}" for index in range(8))
    with pytest.raises(ValueError, match=error):
        parser(frame, expected)


def test_proximity_gate_rejects_simulation_stamp_rollback() -> None:
    accumulator = _definitions("_pair_key", "WalkerProximityAccumulator")[
        "WalkerProximityAccumulator"
    ]({f"walker_hash_{index}": 0.25 for index in range(8)})
    poses = {f"walker_hash_{index}": (float(index), 0.0) for index in range(8)}
    accumulator.observe(
        poses=poses,
        source_stamp_ns=2_000_000_000,
        receipt_sim_time_ns=2_000_000_000,
        receipt_monotonic=1.0,
        raw_frame_sha256="a" * 64,
    )
    accumulator.observe(
        poses=poses,
        source_stamp_ns=1_500_000_000,
        receipt_sim_time_ns=2_000_000_000,
        receipt_monotonic=1.1,
        raw_frame_sha256="b" * 64,
    )
    report = accumulator.report(
        window_end_monotonic=1.1,
        pose_source_topic="/world/campus_formal/pose/info",
    )
    assert report["walker_pose_stale_frame_count"] == 1
    assert report["walker_peer_gate_passed"] is False


def test_validator_keeps_environment_telemetry_evaluator_only() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    assert "attach_evaluator_dynamic_proximity" in source
    assert "environment_truth_collector" in source
    assert "evaluator_truth_process_isolated" in source


def test_runtime_runner_has_no_lossy_pose_v_tf_bridge() -> None:
    source = (ROOT / "scripts" / "run_formal_dynamic_obstacle_avoidance.sh").read_text(
        encoding="utf-8"
    )
    assert "walker_pose_bridge" not in source
    assert "tf2_msgs/msg/TFMessage" not in source
    assert "/evaluation/formal_dynamic/walker_pose" not in source
    assert '--pedestrian-schedule "${runtime_schedule}"' in source
