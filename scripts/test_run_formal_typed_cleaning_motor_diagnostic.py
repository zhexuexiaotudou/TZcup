from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path

from collect_formal_typed_cleaning_motor_diagnostic import EXPECTED_CHECKS, finalize


ROOT = Path(__file__).resolve().parents[1]


def _write_inputs(tmp_path: Path) -> tuple[Path, Path, Path, Path, Path]:
    collector = tmp_path / "collector.json"
    launch = tmp_path / "launch.log"
    audit = tmp_path / "audit.json"
    gz_info = tmp_path / "gz_info.txt"
    ros_info = tmp_path / "ros_info.txt"
    collector.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "status": "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED",
                "passed": True,
                "checks": {name: True for name in EXPECTED_CHECKS},
                "metrics": {"raw_trace_frame_count": 200},
            }
        ),
        encoding="utf-8",
    )
    launch.write_text("healthy runtime\n", encoding="utf-8")
    audit.write_text(json.dumps({"passed": True}), encoding="utf-8")
    gz_info.write_text("Publishers [Address, Message Type]:\nfoo, gz.msgs.Double_V\n", encoding="utf-8")
    ros_info.write_text(
        "Type: std_msgs/msg/Float64MultiArray\nPublisher count: 1\n",
        encoding="utf-8",
    )
    return collector, launch, audit, gz_info, ros_info


def test_runner_requires_fresh_merged_runtime_lock_and_memory_guards() -> None:
    source = (ROOT / "scripts/run_formal_typed_cleaning_motor_diagnostic.sh").read_text(
        encoding="utf-8"
    )
    assert "FORMAL_WATER_TYPED_RUNTIME_WS:?" in source
    assert "FORMAL_WATER_TYPED_OUTPUT_DIR:?" in source
    assert "FORMAL_WATER_CRITICAL_SOURCE_MANIFEST:?" in source
    assert '[[ ! -e "${output_dir}" ]]' in source
    assert 'find "${runtime_ws}/install" -type l' in source
    assert 'install/sanitation_gazebo_control' in source
    assert "formal_runtime_configure" in source
    assert "formal_runtime_memory_preflight" in source
    assert "formal_runtime_start_memory_watchdog" in source
    assert "formal_runtime_stop_memory_watchdog" in source
    assert source.count('"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch') == 1
    assert "--symlink-install" not in source
    assert "source_sha256" in source
    assert "source_matches_frozen_copy" in source


def test_runner_checks_live_types_and_strict_publish_audit() -> None:
    source = (ROOT / "scripts/run_formal_typed_cleaning_motor_diagnostic.sh").read_text(
        encoding="utf-8"
    )
    assert 'gz topic -i -t "${typed_topic}"' in source
    assert 'ros2 topic info -v "${typed_topic}"' in source
    assert "audit_formal_water_launch_log.py" in source
    assert "--finalize" in source
    assert "raw_frames.jsonl" in source
    assert "typed_diag.json" in source


def test_typed_runner_disables_only_shutdown_unstable_optional_bridges() -> None:
    runner = (ROOT / "scripts/run_formal_typed_cleaning_motor_diagnostic.sh").read_text(
        encoding="utf-8"
    )
    launch_path = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    )
    launch = ast.parse(launch_path.read_text(encoding="utf-8"))
    names = {
        node.args[0].value: next(
            value.value.value
            for value in node.keywords
            if value.arg == "default_value" and isinstance(value.value, ast.Constant)
        )
        for node in ast.walk(launch)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value
        in {"start_product_bridge", "start_cleaning_actuator_scalar_bridge"}
    }
    assert names == {
        "start_product_bridge": "true",
        "start_cleaning_actuator_scalar_bridge": "true",
    }
    assert "start_product_bridge:=false" in runner
    assert "start_cleaning_actuator_scalar_bridge:=false" in runner
    for bridge, switch in (
        ("formal_vehicle_product_bridge", "start_product_bridge"),
        ("cleaning_actuator_scalar_bridge", "start_cleaning_actuator_scalar_bridge"),
    ):
        node = next(
            node
            for node in ast.walk(launch)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "Node"
            and any(
                key.arg == "name"
                and isinstance(key.value, ast.Constant)
                and key.value.value == bridge
                for key in node.keywords
            )
        )
        condition = next(key.value for key in node.keywords if key.arg == "condition")
        assert isinstance(condition, ast.Call)
        assert isinstance(condition.func, ast.Name) and condition.func.id == "IfCondition"
        assert isinstance(condition.args[0], ast.Name) and condition.args[0].id == switch


def test_finalize_binds_zero_error_transport_and_input_hashes(tmp_path: Path) -> None:
    inputs = _write_inputs(tmp_path)
    output = tmp_path / "typed_diag.json"
    assert finalize(*inputs, output) == 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["passed"] is True
    transport = report["transport_audit"]
    assert transport["checks"]["zero_nodeshared_publish_errors"] is True
    assert transport["checks"]["zero_topic_tagged_publish_failures"] is True
    assert transport["node_shared_publish_errors"] == []
    assert transport["topic_tagged_publish_failures"] == []
    for path_key, hash_key in (
        ("launch_log", "launch_log_sha256"),
        ("launch_audit_json", "launch_audit_sha256"),
        ("gazebo_topic_info", "gazebo_topic_info_sha256"),
        ("ros_topic_info", "ros_topic_info_sha256"),
    ):
        path = Path(transport[path_key])
        assert hashlib.sha256(path.read_bytes()).hexdigest() == transport[hash_key]


def test_finalize_fails_on_nodeshared_or_tagged_publish_failure(tmp_path: Path) -> None:
    inputs = list(_write_inputs(tmp_path))
    inputs[1].write_text(
        "NodeShared::Publish() Error: Interrupted system call\n"
        "gz_publish_failed topic=/typed count=1\n",
        encoding="utf-8",
    )
    output = tmp_path / "typed_diag.json"
    assert finalize(*inputs, output) == 1
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["passed"] is False
    assert report["transport_audit"]["checks"]["zero_nodeshared_publish_errors"] is False
    assert report["transport_audit"]["checks"]["zero_topic_tagged_publish_failures"] is False


def test_finalize_refuses_stale_output(tmp_path: Path) -> None:
    inputs = _write_inputs(tmp_path)
    output = tmp_path / "typed_diag.json"
    output.write_text("stale\n", encoding="utf-8")
    try:
        finalize(*inputs, output)
    except ValueError as error:
        assert "refusing stale output" in str(error)
    else:
        raise AssertionError("expected stale-output refusal")
