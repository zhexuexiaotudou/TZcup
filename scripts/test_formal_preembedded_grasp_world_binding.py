import hashlib
import json
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from formal_preembedded_grasp_world_binding import (
    CUBE_INITIAL_POSE,
    VEHICLE_INITIAL_POSE,
    PreembeddedGraspWorldBindingError,
    validate_preembedded_grasp_world,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(tmp_path: Path) -> dict[str, object]:
    tmp_path.mkdir(parents=True, exist_ok=True)
    install = tmp_path / "install"
    (install / "setup.bash").parent.mkdir(parents=True)
    (install / "setup.bash").write_text("# frozen overlay\n", encoding="utf-8")
    controller = install / "share/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
    controller.parent.mkdir(parents=True)
    controller.write_text("controller_manager: {}\n", encoding="utf-8")
    source_world = install / "share/sanitation_manipulation/worlds/formal_cube_manipulation.sdf"
    source_world.parent.mkdir(parents=True)
    source_world.write_text("<sdf version='1.11'><world name='formal_cube_manipulation'/></sdf>\n", encoding="utf-8")

    report = tmp_path / "grasp.preembedded_grasp_world.json"
    world = tmp_path / "grasp.preembedded_grasp_world.sdf"
    vehicle = tmp_path / "grasp.preembedded_vehicle.urdf"
    cube = tmp_path / "grasp.preembedded_cube.urdf"
    session = tmp_path / "session.json"
    session.write_text("{}\n", encoding="utf-8")
    acceptance = {
        "started_epoch_ns": 1,
        "session_manifest_sha256": _sha256(session),
    }
    vehicle.write_text("<robot name='tzcup_formal_sanitation_vehicle'/>\n", encoding="utf-8")
    cube.write_text("<robot name='material_cube'/>\n", encoding="utf-8")
    world.write_text(
        "<sdf version='1.11'><world name='formal_cube_manipulation'>"
        "<model name='tzcup_formal_sanitation_vehicle'><pose>"
        f"{VEHICLE_INITIAL_POSE}</pose><plugin filename='gz_ros2_control-system' "
        "name='gz_ros2_control::GazeboSimROS2ControlPlugin'><parameters>"
        f"{controller}</parameters></plugin></model>"
        f"<model name='material_cube'><pose>{CUBE_INITIAL_POSE}</pose></model>"
        "</world></sdf>\n",
        encoding="utf-8",
    )
    snapshot = {
        "snapshot_manifest_sha256": "a" * 64,
        "source_inventory_sha256": "b" * 64,
        "expanded_urdf_sha256": "c" * 64,
    }
    report.write_text(
        json.dumps(
            {
                "status": "FORMAL_PREEMBEDDED_SENSOR_WORLD_READY",
                "passed": True,
                "formal_eligible": True,
                "spawn_mode": "preembedded_before_gazebo_sensors_system",
                "output_world": str(world.resolve()),
                "output_world_sha256": _sha256(world),
                "vehicle_urdf": str(vehicle.resolve()),
                "vehicle_urdf_sha256": _sha256(vehicle),
                "model_name": "tzcup_formal_sanitation_vehicle",
                "model_initial_pose": VEHICLE_INITIAL_POSE,
                "source_world": str(source_world.resolve()),
                "source_world_sha256": _sha256(source_world),
                "controller_runtime_binding": {
                    "runtime_install_root": str(install.resolve()),
                    "resolved_controller_config": str(controller.resolve()),
                    "controller_config_relative_to_install": (
                        "share/sanitation_vehicle_description/"
                        "config/formal_vehicle_controllers.yaml"
                    ),
                    "controller_config_sha256": _sha256(controller),
                },
                "additional_model": {
                    "urdf": str(cube.resolve()),
                    "urdf_sha256": _sha256(cube),
                    "model_name": "material_cube",
                    "model_initial_pose": CUBE_INITIAL_POSE,
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return {
        "report": report,
        "world": world,
        "vehicle": vehicle,
        "cube": cube,
        "source_world": source_world,
        "install": install,
        "acceptance": acceptance,
        "snapshot": snapshot,
    }


def _validate(paths: dict[str, object]) -> dict[str, object]:
    return validate_preembedded_grasp_world(
        report_path=paths["report"],  # type: ignore[arg-type]
        world_path=paths["world"],  # type: ignore[arg-type]
        vehicle_urdf_path=paths["vehicle"],  # type: ignore[arg-type]
        cube_urdf_path=paths["cube"],  # type: ignore[arg-type]
        source_world_path=paths["source_world"],  # type: ignore[arg-type]
        acceptance_session=paths["acceptance"],  # type: ignore[arg-type]
        snapshot_identity=paths["snapshot"],  # type: ignore[arg-type]
        expected_runtime_install_root=paths["install"],  # type: ignore[arg-type]
    )


def _report(paths: dict[str, object]) -> dict:
    return json.loads(Path(paths["report"]).read_text(encoding="utf-8"))


def _write_report(paths: dict[str, object], payload: dict) -> None:
    Path(paths["report"]).write_text(json.dumps(payload) + "\n", encoding="utf-8")


def test_binds_all_four_fresh_regular_artifacts_and_contract_fields(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    binding = _validate(paths)
    assert binding["vehicle_initial_pose"] == VEHICLE_INITIAL_POSE
    assert tuple(map(float, str(binding["cube_initial_pose"]).split())) == tuple(
        map(float, CUBE_INITIAL_POSE.split())
    )
    assert binding["spawn_mode"] == "preembedded_before_gazebo_sensors_system"
    assert binding["snapshot"] == paths["snapshot"]


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (lambda payload: payload.update(spawn_mode="dynamic"), "spawn/model contract"),
        (lambda payload: payload.update(vehicle_urdf="/wrong/vehicle.urdf"), "vehicle URDF path"),
        (lambda payload: payload["additional_model"].update(urdf_sha256="0" * 64), "cube URDF SHA-256"),
        (lambda payload: payload.update(model_initial_pose="0 0 0 0 0 0"), "vehicle initial pose"),
        (lambda payload: payload["additional_model"].update(model_initial_pose="0 0 0 0 0 0"), "cube initial pose"),
    ),
)
def test_rejects_report_spawn_path_hash_and_pose_drift(
    tmp_path: Path, mutate, message: str
) -> None:
    paths = _fixture(tmp_path)
    payload = _report(paths)
    mutate(payload)
    _write_report(paths, payload)
    with pytest.raises(PreembeddedGraspWorldBindingError, match=message):
        _validate(paths)


def test_rejects_world_pose_or_duplicate_model_even_when_report_hash_is_updated(
    tmp_path: Path,
) -> None:
    paths = _fixture(tmp_path)
    root = ET.parse(Path(paths["world"]))
    world = root.getroot().find("world")
    assert world is not None
    cube = world.find("model[@name='material_cube']")
    assert cube is not None
    cube.find("pose").text = "0 0 0.017 0 0 0"
    root.write(Path(paths["world"]), encoding="unicode")
    payload = _report(paths)
    payload["output_world_sha256"] = _sha256(Path(paths["world"]))
    _write_report(paths, payload)
    with pytest.raises(PreembeddedGraspWorldBindingError, match="world cube pose"):
        _validate(paths)

    paths = _fixture(tmp_path / "duplicate")
    root = ET.parse(Path(paths["world"]))
    world = root.getroot().find("world")
    assert world is not None
    world.append(ET.fromstring("<model name='material_cube'><pose>0.300 -0.950 0.017 0 0 0</pose></model>"))
    root.write(Path(paths["world"]), encoding="unicode")
    payload = _report(paths)
    payload["output_world_sha256"] = _sha256(Path(paths["world"]))
    _write_report(paths, payload)
    with pytest.raises(PreembeddedGraspWorldBindingError, match="exactly one vehicle and one material cube"):
        _validate(paths)


def test_rejects_stale_and_symbolic_auxiliary_evidence(tmp_path: Path) -> None:
    paths = _fixture(tmp_path)
    session_started = time.time_ns()
    paths["acceptance"]["started_epoch_ns"] = session_started  # type: ignore[index]
    for key in ("report", "world", "vehicle"):
        os.utime(Path(paths[key]), ns=(session_started + 1, session_started + 1))
    stale_ns = session_started - 1
    os.utime(Path(paths["cube"]), ns=(stale_ns, stale_ns))
    with pytest.raises(PreembeddedGraspWorldBindingError, match="cube URDF predates"):
        _validate(paths)

    paths = _fixture(tmp_path / "symlink")
    replacement = Path(paths["report"]).with_name("replacement.json")
    replacement.write_text(Path(paths["report"]).read_text(encoding="utf-8"), encoding="utf-8")
    Path(paths["report"]).unlink()
    try:
        Path(paths["report"]).symlink_to(replacement)
    except OSError as error:
        pytest.skip(f"symbolic links are unavailable: {error}")
    with pytest.raises(PreembeddedGraspWorldBindingError, match="regular non-symbolic"):
        _validate(paths)
