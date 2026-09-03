import hashlib
import json
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from formal_preembedded_sensor_world_binding import (
    PreembeddedWorldBindingError,
    validate_preembedded_sensor_world,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(tmp_path: Path) -> tuple[Path, Path, Path, Path, dict, dict]:
    urdf = tmp_path / "vehicle.urdf"
    world = tmp_path / "world.sdf"
    report = tmp_path / "world.json"
    session = tmp_path / "session.json"
    install_root = (tmp_path / "install").resolve()
    controller = (
        install_root
        / "share/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
    )
    controller.parent.mkdir(parents=True)
    controller.write_text("controller_manager: {}\n", encoding="utf-8")
    urdf.write_text(
        "<robot name='vehicle'><link name='base'/><gazebo reference='base'><sensor name='scan' "
        "type='gpu_lidar'/></gazebo></robot>", encoding="utf-8"
    )
    world.write_text(
        "<sdf version='1.11'><world name='w'><model name='ground_plane'><static>true</static>"
        "<link name='ground'/></model><model name='vehicle'>"
        "<plugin filename='gz_ros2_control-system' "
        "name='gz_ros2_control::GazeboSimROS2ControlPlugin'><parameters>"
        f"{controller}</parameters></plugin></model></world></sdf>\n",
        encoding="utf-8",
    )
    session.write_text("{}\n", encoding="utf-8")
    snapshot = {
        "snapshot_manifest_sha256": "a" * 64,
        "source_inventory_sha256": "b" * 64,
        "expanded_urdf_sha256": _sha256(urdf),
    }
    acceptance = {
        "started_epoch_ns": 1,
        "path": str(session),
        "session_manifest_sha256": _sha256(session),
    }
    report.write_text(json.dumps({
        "status": "FORMAL_PREEMBEDDED_SENSOR_WORLD_READY", "passed": True,
        "formal_eligible": True,
        "spawn_mode": "preembedded_before_gazebo_sensors_system",
        "output_world": str(world.resolve()), "output_world_sha256": _sha256(world),
        "vehicle_urdf": str(urdf.resolve()), "vehicle_urdf_sha256": _sha256(urdf),
        "sensor_count": 1, "model_initial_pose": "0 0 0.005 0 0 0",
        "model_name": "vehicle",
        "source_world": str(world.resolve()), "source_world_sha256": _sha256(world),
        "controller_runtime_binding": {
            "runtime_install_root": str(install_root),
            "resolved_controller_config": str(controller),
            "controller_config_relative_to_install": (
                "share/sanitation_vehicle_description/"
                "config/formal_vehicle_controllers.yaml"
            ),
            "controller_config_sha256": _sha256(controller),
        },
    }) + "\n", encoding="utf-8")
    return report, world, urdf, install_root, acceptance, snapshot


def test_binds_paths_hashes_sensor_count_pose_and_session_snapshot(tmp_path: Path):
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    binding = validate_preembedded_sensor_world(
        report_path=report, world_path=world, expanded_urdf_path=urdf,
        acceptance_session=acceptance, snapshot_identity=snapshot,
        expected_model_pose="0 0 0.005 0 0 0",
        expected_runtime_install_root=install_root,
    )
    assert binding["preembedded_world_sha256"] == _sha256(world)
    assert binding["sensor_count"] == 1
    assert binding["spawn_mode"] == "preembedded_before_gazebo_sensors_system"
    assert binding["snapshot"] == snapshot


def test_rejects_preembedded_evidence_that_predates_the_acceptance_session(
    tmp_path: Path,
) -> None:
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    acceptance["started_epoch_ns"] = time.time_ns()
    stale_ns = acceptance["started_epoch_ns"] - 1_000_000
    os.utime(report, ns=(stale_ns, stale_ns))
    os.utime(world, ns=(stale_ns, stale_ns))

    with pytest.raises(PreembeddedWorldBindingError, match="predates acceptance session"):
        validate_preembedded_sensor_world(
            report_path=report,
            world_path=world,
            expanded_urdf_path=urdf,
            acceptance_session=acceptance,
            snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )


@pytest.mark.parametrize("field,value,error", [
    ("formal_eligible", False, "not ready"),
    ("sensor_count", 2, "sensor count mismatch"),
    ("spawn_mode", "dynamic", "wrong spawn mode"),
    ("model_initial_pose", "0 0 0 0 0 0", "initial pose"),
    ("vehicle_urdf_sha256", "0" * 64, "URDF SHA-256"),
])
def test_rejects_report_contract_drift(tmp_path: Path, field: str, value: object, error: str):
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload[field] = value
    report.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(PreembeddedWorldBindingError, match=error):
        validate_preembedded_sensor_world(
            report_path=report, world_path=world, expanded_urdf_path=urdf,
            acceptance_session=acceptance, snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )


def test_rejects_snapshot_urdf_drift_and_wrong_world_path(tmp_path: Path):
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    snapshot["expanded_urdf_sha256"] = "f" * 64
    with pytest.raises(PreembeddedWorldBindingError, match="snapshot does not match"):
        validate_preembedded_sensor_world(
            report_path=report, world_path=world, expanded_urdf_path=urdf,
            acceptance_session=acceptance, snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )


def test_rejects_source_world_hash_drift(tmp_path: Path):
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["source_world_sha256"] = "0" * 64
    report.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(PreembeddedWorldBindingError, match="source-world SHA-256"):
        validate_preembedded_sensor_world(
            report_path=report, world_path=world, expanded_urdf_path=urdf,
            acceptance_session=acceptance, snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )
    snapshot["expanded_urdf_sha256"] = _sha256(urdf)
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["output_world"] = str(tmp_path / "other.sdf")
    report.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(PreembeddedWorldBindingError, match="another world"):
        validate_preembedded_sensor_world(
            report_path=report, world_path=world, expanded_urdf_path=urdf,
            acceptance_session=acceptance, snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )


def test_rejects_control_plugin_on_static_asset_instead_of_formal_vehicle(
    tmp_path: Path,
) -> None:
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))
    root = ET.parse(world)
    world_node = root.getroot().find("world")
    assert world_node is not None
    vehicle = world_node.find("model[@name='vehicle']")
    ground = world_node.find("model[@name='ground_plane']")
    assert vehicle is not None and ground is not None
    plugin = vehicle.find("plugin")
    assert plugin is not None
    vehicle.remove(plugin)
    ground.append(plugin)
    root.write(world, encoding="unicode")
    payload["output_world_sha256"] = _sha256(world)
    payload["source_world_sha256"] = _sha256(world)
    report.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(PreembeddedWorldBindingError, match="complete bound gz_ros2_control"):
        validate_preembedded_sensor_world(
            report_path=report,
            world_path=world,
            expanded_urdf_path=urdf,
            acceptance_session=acceptance,
            snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )


def test_rejects_competing_gz_ros2_control_authority(tmp_path: Path) -> None:
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))
    root = ET.parse(world)
    vehicle = root.getroot().find("world/model[@name='vehicle']")
    assert vehicle is not None
    vehicle.append(
        ET.fromstring(
            "<plugin filename='gz_ros2_control-system' name='other_control_plugin'>"
            "<parameters>other.yaml</parameters></plugin>"
        )
    )
    root.write(world, encoding="unicode")
    payload["output_world_sha256"] = _sha256(world)
    payload["source_world_sha256"] = _sha256(world)
    report.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(PreembeddedWorldBindingError, match="no competing control authority"):
        validate_preembedded_sensor_world(
            report_path=report,
            world_path=world,
            expanded_urdf_path=urdf,
            acceptance_session=acceptance,
            snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )


@pytest.mark.parametrize(
    "mutation", ["controller_hash", "controller_root", "relative_controller_root", "world_parameter"]
)
def test_rejects_controller_runtime_binding_drift(tmp_path: Path, mutation: str):
    report, world, urdf, install_root, acceptance, snapshot = _fixture(tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))
    if mutation == "controller_hash":
        payload["controller_runtime_binding"]["controller_config_sha256"] = "0" * 64
    elif mutation == "controller_root":
        payload["controller_runtime_binding"]["runtime_install_root"] = str(
            tmp_path / "other-install"
        )
    elif mutation == "relative_controller_root":
        payload["controller_runtime_binding"]["runtime_install_root"] = "install"
    else:
        world.write_text(
            world.read_text(encoding="utf-8").replace(
                "formal_vehicle_controllers.yaml", "other_controllers.yaml"
            ),
            encoding="utf-8",
        )
        payload["output_world_sha256"] = _sha256(world)
        payload["source_world_sha256"] = _sha256(world)
    report.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(PreembeddedWorldBindingError):
        validate_preembedded_sensor_world(
            report_path=report,
            world_path=world,
            expanded_urdf_path=urdf,
            acceptance_session=acceptance,
            snapshot_identity=snapshot,
            expected_model_pose="0 0 0.005 0 0 0",
            expected_runtime_install_root=install_root,
        )
