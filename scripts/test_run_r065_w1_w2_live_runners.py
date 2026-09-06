from pathlib import Path
import importlib.util

import pytest


ROOT = Path(__file__).resolve().parents[1]
W1 = ROOT / "scripts" / "run_r065_w1_dynamic_footprint_live.sh"
W2 = ROOT / "scripts" / "run_r065_w2_moveit_ground_live.sh"
COLLECTOR = ROOT / "scripts" / "collect_r065_w2_live_grasp_request.py"


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _collector():
    spec = importlib.util.spec_from_file_location("r065_w2_collector", COLLECTOR)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _target() -> dict:
    return {
        "uuid": "pc-track-01", "frame_id": "map", "source_stamp_ns": 123,
        "header_stamp_ns": 123,
        "source_backend": "dosod_edgesam_pc", "target_type": "discrete",
        "track_state": "CONFIRMED", "confidence": 0.8,
        "pose": {"x_m": 0.3, "y_m": -0.95, "z_m": 0.015, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
        "size_m": [0.03, 0.03, 0.03],
    }


def _request() -> str:
    return (
        '{"schema_version":2,"target_id":"pc-track-01","frame_id":"map",'
        '"pose":{"x_m":0.3,"y_m":-0.95,"z_m":0.015,"qx":0.0,"qy":0.0,"qz":0.0,"qw":1.0},'
        '"size_m":[0.03,0.03,0.03],"material":"unknown","confidence":0.8,"truth_used":false}'
    )


def test_w1_is_fixed_source_bound_map_lifecycle_live_gate() -> None:
    source = _source(W1)
    assert "set +u\nsource /opt/ros/jazzy/setup.bash\nset -u" in source
    assert 'realpath --no-symlinks -e "${run_root_arg}"' in source
    assert '[[ "${raw_run_root}" == "${run_root}" ]]' in source
    assert 'set +u\nsource "${runtime_ws}/install/setup.bash"\nset -u' in source
    assert 'source "${repo_root}/scripts/run_formal_runtime_isolation.sh"' in source
    assert 'source "${repo_root}/scripts/formal_source_bound_preflight.sh"' in source
    assert 'formal_source_bound_preflight \\' in source
    assert 'formal_source_bound_verify_overlay "${runtime_ws}/install"' in source
    assert 'runtime_root="${run_root}/w1_runtime"' in source
    assert 'output="${run_root}/w1.json"' in source
    assert 'runtime_binding="${run_root}/w1.runtime_binding.json"' in source
    assert 'formal_campus_map_lifecycle.launch.py' in source
    assert 'ready="false"' in source and 'ready="true"' in source
    assert '[[ "${ready}" == "true" ]]' in source
    assert 'mission_mode:=mapping' in source
    assert 'enable_dynamic_footprint_runtime_test_override:=true' in source
    assert 'start_pedestrians:=false start_coverage:=false' in source
    assert 'ros2 run sanitation_formal_campus_integration' in source
    assert 'formal-dynamic-footprint-runtime-gate' in source
    assert '--output "${output}"' in source
    assert 'formal_runtime_cleanup_groups "${GZ_PARTITION}" "${launch_pid}"' in source
    assert 'cleanup_evidence' in source
    assert 'R065 W1 refuses an inherited Gazebo partition' in source
    assert 'ros2 topic pub' not in source
    assert 'joint_states' not in source
    assert source.index('formal_source_bound_preflight') < source.index('ros2 launch')


def test_w2_uses_real_campus_localization_and_machine_readable_gate_stdout() -> None:
    source = _source(W2)
    assert "set +u\nsource /opt/ros/jazzy/setup.bash\nset -u" in source
    assert 'realpath --no-symlinks -e "${run_root_arg}"' in source
    assert '[[ "${raw_run_root}" == "${run_root}" ]]' in source
    assert 'set +u\nsource "${runtime_ws}/install/setup.bash"\nset -u' in source
    assert 'source "${repo_root}/scripts/run_formal_runtime_isolation.sh"' in source
    assert 'source "${repo_root}/scripts/formal_source_bound_preflight.sh"' in source
    assert 'runtime_root="${run_root}/w2_runtime"' in source
    assert '[[ "$#" -eq 1 ]]' in source
    assert 'runtime_binding="${run_root}/w2.runtime_binding.json"' in source
    assert 'request_json_path="${run_root}/w2_request.json"' in source
    assert 'request_provenance="${run_root}/w2_request_provenance.json"' in source
    assert 'formal_campus.launch.py' in source
    assert 'localization_backend:=amcl' in source
    assert 'start_pedestrians:=false start_navigation:=false start_coverage:=false' in source
    assert 'ros2 run tf2_ros tf2_echo map base_footprint' in source
    assert 'ros2 run sanitation_manipulation' in source
    assert 'moveit_ground_runtime_gate --ros-args' in source
    assert 'formal_pc_open_vocab.launch.py artifact_root:="${perception_artifact_root}"' in source
    assert 'for key in ("perception_artifact_root", "onnx_pythonpath")' in source
    assert 'root / "onnxruntime" / "__init__.py"' in source
    assert 'export PYTHONPATH="${onnx_pythonpath}:${PYTHONPATH:-}"' in source
    assert 'collect_r065_w2_live_grasp_request.py' in source
    assert '-p allow_ground_removal_test:=true' in source
    assert 'json.loads(raw)' in source
    assert '"request_json:=${request_parameter}"' in source
    assert 'executor_or_controller_commands_sent": False' in source
    assert 'truth_used_for_control": False' in source
    assert 'cat "${gate_json}"' in source
    assert '>"${gate_json}"' in source
    assert 'source_bound_preflight.stdout' in source
    assert 'source_bound_overlay.stdout' in source
    assert 'ros2 topic pub' not in source
    assert 'static_transform_publisher' not in source
    assert 'formal_active_cleaning' not in source
    assert 'JointState' not in source
    assert 'joint_states' not in source
    assert 'R065 W2 refuses an inherited Gazebo partition' in source
    assert source.index('formal_source_bound_preflight') < source.index('ros2 launch')


def test_w2_collector_accepts_only_same_fresh_product_target_and_recheck() -> None:
    collector = _collector()
    request, stamp = collector.request_from_pair(_target(), _request())
    assert request["target_id"] == "pc-track-01"
    assert stamp == 123


def test_w2_collector_rejects_tampered_or_missing_product_pair() -> None:
    collector = _collector()
    with pytest.raises(collector.CaptureError, match="UUID"):
        collector.request_from_pair(_target(), _request().replace("pc-track-01", "other", 1))
    with pytest.raises(collector.CaptureError, match="not JSON"):
        collector.request_from_pair(_target(), "")


def test_w2_collector_requires_fresh_nonfuture_simulation_time_target() -> None:
    collector = _collector()
    assert collector.source_age_s(now_ros_ns=2_000_000_000, source_stamp_ns=1_500_000_000) == 0.5
    with pytest.raises(collector.CaptureError, match="future-dated"):
        collector.source_age_s(now_ros_ns=1_000_000_000, source_stamp_ns=1_000_000_001)
    with pytest.raises(collector.CaptureError, match="stale"):
        collector.source_age_s(now_ros_ns=3_000_000_001, source_stamp_ns=2_000_000_000)


def test_w2_recheck_recomputes_freshness_at_its_own_ros_clock() -> None:
    collector = _collector()
    # A target that was fresh when its array arrived must still be rejected if
    # it has aged out before the matching wrist recheck arrives.
    assert collector.source_age_s(now_ros_ns=1_900_000_000, source_stamp_ns=1_000_000_000) == 0.9
    with pytest.raises(collector.CaptureError, match="stale"):
        collector.source_age_s(
            now_ros_ns=2_000_000_001, source_stamp_ns=1_000_000_000
        )
    source = _source(COLLECTOR)
    recheck = source[source.index("def on_recheck"):source.index("rclpy.init()")]
    assert "capture_ros_time_ns = self.get_clock().now().nanoseconds" in recheck
    assert "now_ros_ns=capture_ros_time_ns" in recheck
    assert "source_stamp_ns=source_stamp_ns" in recheck


def test_w2_collector_requires_the_sole_named_product_publisher() -> None:
    collector = _collector()
    good = {
        collector.TARGET_TOPIC: [{"node_name": collector.PRODUCT_NODE, "node_namespace": "/", "topic_type": collector.TARGET_TYPE}],
        collector.RECHECK_TOPIC: [{"node_name": collector.PRODUCT_NODE, "node_namespace": "/", "topic_type": collector.RECHECK_TYPE}],
    }
    assert collector.publisher_contract(good)
    bad = {key: list(value) for key, value in good.items()}
    bad[collector.TARGET_TOPIC][0] = {"node_name": "unknown", "node_namespace": "/", "topic_type": collector.TARGET_TYPE}
    assert not collector.publisher_contract(bad)


def test_w2_collector_binds_both_frozen_perception_roots(tmp_path) -> None:
    collector = _collector()
    artifacts = tmp_path / "artifacts"
    onnx = tmp_path / "onnx"
    artifacts.mkdir()
    (onnx / "onnxruntime").mkdir(parents=True)
    (onnx / "onnxruntime" / "__init__.py").write_text("", encoding="utf-8")
    actual = collector.closure_perception_roots({
        "closure": {
            "perception_artifact_root": str(artifacts),
            "onnx_pythonpath": str(onnx),
        }
    })
    assert actual == (artifacts.resolve(), onnx.resolve())
    (onnx / "onnxruntime" / "__init__.py").unlink()
    with pytest.raises(collector.CaptureError, match="onnxruntime"):
        collector.closure_perception_roots({
            "closure": {
                "perception_artifact_root": str(artifacts),
                "onnx_pythonpath": str(onnx),
            }
        })


def test_w2_collector_is_read_only_and_never_subscribes_to_truth_or_actions() -> None:
    source = _source(COLLECTOR)
    assert source.count("create_subscription(") == 2
    assert "create_publisher(" not in source
    assert "ActionClient" not in source
    assert "create_client(" not in source
    assert 'Parameter("use_sim_time", value=True)' in source
    assert "capture_ros_time_ns" in source
    assert "source_age_s" in source
    assert '"onnx_pythonpath": str(onnx_pythonpath)' in source
    assert "ground_truth" not in source
    assert '"/evaluation/' not in source
