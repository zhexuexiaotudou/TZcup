from pathlib import Path

from finalize_formal_gz_native_sensor_smoke import finalize


ROOT = Path(__file__).resolve().parents[1]


def _probe(root: Path, name: str, status: int, byte_count: int) -> None:
    (root / f"{name}.status").write_text(f"{status}\n", encoding="utf-8")
    (root / f"{name}.bytes").write_text(f"{byte_count}\n", encoding="utf-8")


def test_finalizer_classifies_render_lifecycle_block_without_formal_claim(tmp_path: Path) -> None:
    _probe(tmp_path, "imu", 0, 100)
    _probe(tmp_path, "utm30_gpu_lidar", 124, 0)
    _probe(tmp_path, "front_rgbd_depth", 124, 0)
    report = finalize(tmp_path, launch_alive=True)
    assert report["status"] == "DYNAMIC_SPAWN_RENDER_SENSOR_LIFECYCLE_BLOCKED"
    assert report["passed"] is False
    assert report["formal_eligible"] is False
    assert report["transport_plane"] == "gazebo_native_without_high_bandwidth_ros_bridges"


def test_finalizer_requires_all_three_native_messages(tmp_path: Path) -> None:
    for name in ("imu", "utm30_gpu_lidar", "front_rgbd_depth"):
        _probe(tmp_path, name, 0, 64)
    report = finalize(tmp_path, launch_alive=True)
    assert report["status"] == "GZ_NATIVE_DYNAMIC_SENSOR_SMOKE_PASSED"
    assert report["passed"] is True


def test_finalizer_never_leaves_passed_evidence_after_watchdog_breach(
    tmp_path: Path,
) -> None:
    for name in ("imu", "utm30_gpu_lidar", "front_rgbd_depth"):
        _probe(tmp_path, name, 0, 64)
    report = finalize(tmp_path, launch_alive=True, memory_watchdog_result=86)
    assert report["status"] == "MEMORY_WATCHDOG_BREACHED"
    assert report["passed"] is False
    persisted = (tmp_path / "gz_native_sensor_smoke.json").read_text(encoding="utf-8")
    assert '"passed": false' in persisted


def test_runner_is_bounded_memory_guarded_and_disables_ros_payload_bridges() -> None:
    source = (ROOT / "scripts/run_formal_gz_native_dynamic_sensor_smoke.sh").read_text(
        encoding="utf-8"
    )
    assert "formal_runtime_memory_preflight" in source
    assert "formal_runtime_start_memory_watchdog" in source
    assert "FORMAL_RUNTIME_MEMORY_WATCHDOG_RESULT" in source
    assert "start_high_bandwidth_sensor_bridges:=false" in source
    assert "high_bandwidth_sensor_runtime:=true" in source
    assert source.count("gz topic -e") == 1
    assert "probe imu /sensors/imu/data" in source
    assert "probe utm30_gpu_lidar /sensors/lidar_2d/scan" in source
    assert "probe front_rgbd_depth /sensors/front_rgbd/depth/image_rect_raw/image" in source
    assert "collect_formal_dynamic_sensor_diagnostic.py" not in source
    assert "frozen diagnostic runtime is stale for" in source
    assert "urdf/high_fidelity/manipulator_stack.xacro" in source
    assert "urdf/high_fidelity/sensor_suite.xacro" in source
    assert "worlds/formal_vehicle_validation.sdf" in source
    assert "vehicle package resolves outside the requested frozen runtime" in source
    assert source.index("set +u\nsource \"${runtime_setup}\"") < source.index(
        "package_share=\"$(ros2 pkg prefix --share sanitation_vehicle_description)\""
    )
    assert source.index("formal_runtime_stop_memory_watchdog") < source.index(
        "finalize_formal_gz_native_sensor_smoke.py"
    )
    assert '--memory-watchdog-result "${watchdog_result}"' in source


def test_formal_launch_separates_sensor_generation_from_high_bandwidth_bridges() -> None:
    source = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'LaunchConfiguration(\n        "start_high_bandwidth_sensor_bridges"' in source
    assert '"start_high_bandwidth_sensor_bridges",' in source
    assert "high_bandwidth_bridges_enabled = PythonExpression(" in source
    assert source.count("condition=IfCondition(high_bandwidth_bridges_enabled)") == 2
