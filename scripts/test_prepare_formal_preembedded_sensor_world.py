from pathlib import Path
import argparse
import importlib.util
import json
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "prepare_formal_preembedded_sensor_world",
    ROOT / "scripts/prepare_formal_preembedded_sensor_world.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_restores_urdf_sensor_reference_link_and_local_pose():
    urdf = ET.fromstring(
        """<robot name='fixture'>
        <link name='base'/><link name='wrist'/>
        <gazebo reference='wrist'><sensor name='wrist_camera' type='camera'>
          <pose>0.1 0 0 0 0 0</pose></sensor></gazebo>
        <gazebo reference='base'><sensor name='scan' type='gpu_lidar'/></gazebo>
        </robot>"""
    )
    converted = ET.fromstring(
        """<model name='fixture'><link name='base'>
        <sensor name='wrist_camera' type='camera'><pose>9 9 9 0 0 0</pose></sensor>
        <sensor name='scan' type='gpu_lidar'><pose>8 8 8 0 0 0</pose></sensor>
        </link><link name='wrist'/></model>"""
    )
    restored = MODULE.restore_sensor_attachments(
        converted, MODULE.sensor_attachment_contract(urdf), urdf
    )
    wrist = converted.find("link[@name='wrist']/sensor[@name='wrist_camera']")
    scan = converted.find("link[@name='base']/sensor[@name='scan']")
    assert wrist is not None and wrist.findtext("pose") == "0.1 0 0 0 0 0"
    assert scan is not None and scan.findtext("pose") == "0 0 0 0 0 0"
    assert {row["sensor"] for row in restored} == {"scan", "wrist_camera"}


def test_reconstructs_fixed_sensor_holder_when_sdformat_reduces_reference_link():
    urdf = ET.fromstring(
        """<robot name='fixture'><link name='base'/><link name='camera_link'/>
        <joint name='camera_mount' type='fixed'><parent link='base'/><child link='camera_link'/></joint>
        <gazebo reference='camera_link'><sensor name='camera' type='camera'>
        <pose>0.1 0 0 0 0 0</pose></sensor></gazebo></robot>"""
    )
    converted = ET.fromstring(
        """<model name='fixture'><link name='base'><sensor name='camera' type='camera'>
        <pose>1.1 2 3 0 0 0</pose></sensor></link></model>"""
    )
    restored = MODULE.restore_sensor_attachments(
        converted, MODULE.sensor_attachment_contract(urdf), urdf
    )
    holder = converted.find("link[@name='camera_link']")
    assert holder is not None and holder.findtext("pose") == "1 2 3 0 0 0"
    assert holder.find("sensor[@name='camera']") is not None
    joint = converted.find("joint[@name='formal_sensor_attachment_camera_link']")
    assert joint is not None and joint.findtext("parent") == "base"
    assert restored[0]["attachment_status"] == "restored_reconstructed_fixed_reference_link"


def test_build_can_make_single_sensor_source_diagnostic(tmp_path: Path):
    world = tmp_path / "world.sdf"
    urdf = tmp_path / "vehicle.urdf"
    world.write_text("<sdf version='1.11'><world name='w'/></sdf>", encoding="utf-8")
    urdf.write_text(
        "<robot name='fixture'><link name='base'/><gazebo reference='base'>"
        "<sensor name='camera' type='camera'/><sensor name='scan' type='gpu_lidar'/>"
        "</gazebo></robot>", encoding="utf-8"
    )
    tree, rows, _ = MODULE.build_preembedded_world(
        world,
        "<sdf version='1.11'><model name='fixture'><link name='base'>"
        "<sensor name='camera' type='camera'/><sensor name='scan' type='gpu_lidar'/>"
        "</link></model></sdf>",
        urdf,
        only_sensors={"scan"},
    )
    assert [row["sensor"] for row in rows] == ["scan"]
    assert [sensor.get("name") for sensor in tree.getroot().findall(".//sensor")] == ["scan"]


def test_build_diagnostic_raw_layout_omits_reconstructed_attachment_joint(tmp_path: Path):
    world = tmp_path / "world.sdf"
    urdf = tmp_path / "vehicle.urdf"
    world.write_text("<sdf version='1.11'><world name='w'/></sdf>", encoding="utf-8")
    urdf.write_text(
        "<robot name='fixture'><link name='base'/><link name='camera_link'/>"
        "<joint name='camera_mount' type='fixed'><parent link='base'/>"
        "<child link='camera_link'/></joint><gazebo reference='camera_link'>"
        "<sensor name='camera' type='camera'/></gazebo></robot>",
        encoding="utf-8",
    )

    tree, rows, model = MODULE.build_preembedded_world(
        world,
        "<sdf version='1.11'><model name='fixture'><link name='base'>"
        "<sensor name='camera' type='camera'/></link></model></sdf>",
        urdf,
        restore_attachments=False,
    )

    assert rows == []
    assert model.find("joint[@name='formal_sensor_attachment_camera_link']") is None
    assert tree.getroot().find("world/model/link/sensor[@name='camera']") is not None


def test_preembedded_model_pose_defaults_to_dynamic_create_contact_settling_clearance(tmp_path: Path):
    world = tmp_path / "world.sdf"
    urdf = tmp_path / "vehicle.urdf"
    world.write_text("<sdf version='1.11'><world name='w'/></sdf>", encoding="utf-8")
    urdf.write_text(
        "<robot name='fixture'><link name='base'/><gazebo reference='base'>"
        "<sensor name='scan' type='gpu_lidar'/></gazebo></robot>", encoding="utf-8"
    )
    tree, _, _ = MODULE.build_preembedded_world(
        world,
        "<sdf version='1.11'><model name='fixture'><link name='base'>"
        "<sensor name='scan' type='gpu_lidar'/></link></model></sdf>",
        urdf,
    )
    assert tree.getroot().findtext("world/model/pose") == "0 0 0.005 0 0 0"


def _runtime_controller_fixture(tmp_path: Path) -> tuple[Path, Path]:
    install_root = (tmp_path / "install").resolve()
    controller = install_root / MODULE.CONTROLLER_RUNTIME_RELATIVE
    controller.parent.mkdir(parents=True)
    controller.write_text("controller_manager: {}\n", encoding="utf-8")
    return install_root, controller


def _converted_model_with_control_plugin() -> ET.Element:
    return ET.fromstring(
        "<model name='fixture'><plugin filename='gz_ros2_control-system' "
        "name='gz_ros2_control::GazeboSimROS2ControlPlugin'><parameters>"
        f"{MODULE.CANONICAL_CONTROLLER_URI}</parameters></plugin></model>"
    )


def test_binds_controller_parameters_to_exact_frozen_runtime_artifact(tmp_path: Path):
    install_root, controller = _runtime_controller_fixture(tmp_path)
    model = _converted_model_with_control_plugin()

    binding = MODULE.bind_controller_parameters(model, controller, install_root)

    assert model.findtext("plugin/parameters") == str(controller)
    assert binding["portable_source_uri"] == MODULE.CANONICAL_CONTROLLER_URI
    assert binding["resolved_controller_config"] == str(controller)
    assert binding["controller_config_sha256"] == MODULE._sha256(controller)


@pytest.mark.parametrize("bad_path_kind", ["relative", "outside", "missing"])
def test_controller_binding_rejects_nonruntime_paths(tmp_path: Path, bad_path_kind: str):
    install_root, controller = _runtime_controller_fixture(tmp_path)
    if bad_path_kind == "relative":
        candidate = Path("relative/controllers.yaml")
    elif bad_path_kind == "outside":
        candidate = tmp_path / "outside.yaml"
        candidate.write_text("controller_manager: {}\n", encoding="utf-8")
    else:
        candidate = controller.with_name("missing.yaml")

    with pytest.raises(MODULE.PreparationError):
        MODULE.bind_controller_parameters(
            _converted_model_with_control_plugin(), candidate, install_root
        )


@pytest.mark.parametrize(
    "mutation", ["missing_plugin", "duplicate_plugin", "competing_plugin", "wrong_uri"]
)
def test_controller_binding_rejects_plugin_contract_drift(tmp_path: Path, mutation: str):
    install_root, controller = _runtime_controller_fixture(tmp_path)
    model = _converted_model_with_control_plugin()
    if mutation == "missing_plugin":
        model.remove(model.find("plugin"))
    elif mutation == "duplicate_plugin":
        model.append(ET.fromstring(ET.tostring(model.find("plugin"), encoding="unicode")))
    elif mutation == "competing_plugin":
        model.append(
            ET.fromstring(
                "<plugin filename='gz_ros2_control-system' "
                "name='other_control_plugin'><parameters>ignored.yaml</parameters>"
                "</plugin>"
            )
        )
    else:
        model.find("plugin/parameters").text = "package://other/controllers.yaml"

    with pytest.raises(MODULE.PreparationError):
        MODULE.bind_controller_parameters(model, controller, install_root)


def test_run_rewrites_package_uri_and_records_controller_binding(tmp_path: Path, monkeypatch):
    source_world = tmp_path / "world.sdf"
    urdf = tmp_path / "vehicle.urdf"
    output_world = tmp_path / "prepared.sdf"
    output_report = tmp_path / "prepared.json"
    install_root, controller = _runtime_controller_fixture(tmp_path)
    source_world.write_text(
        "<sdf version='1.11'><world name='w'>"
        "<model name='preexisting_static_asset'><static>true</static>"
        "<link name='asset'/></model></world></sdf>",
        encoding="utf-8",
    )
    urdf.write_text(
        "<robot name='fixture'><link name='base'/><gazebo reference='base'>"
        "<sensor name='scan' type='gpu_lidar'/></gazebo></robot>",
        encoding="utf-8",
    )
    converted = (
        "<sdf version='1.11'><model name='fixture'><link name='base'>"
        "<sensor name='scan' type='gpu_lidar'/></link>"
        "<plugin filename='gz_ros2_control-system' "
        "name='gz_ros2_control::GazeboSimROS2ControlPlugin'><parameters>"
        f"{MODULE.CANONICAL_CONTROLLER_URI}</parameters></plugin></model></sdf>"
    )
    monkeypatch.setattr(MODULE, "convert_urdf", lambda _gz, _urdf: converted)
    args = argparse.Namespace(
        source_world=str(source_world),
        vehicle_urdf=str(urdf),
        output_world=str(output_world),
        report=str(output_report),
        controller_config=str(controller),
        runtime_install_root=str(install_root),
        gz="gz",
        only_sensor=[],
        model_pose="0 0 0.005 0 0 0",
    )

    report = MODULE.run(args)
    world_text = output_world.read_text(encoding="utf-8")
    written_report = json.loads(output_report.read_text(encoding="utf-8"))

    assert MODULE.CANONICAL_CONTROLLER_URI not in world_text
    assert str(controller) in world_text
    assert "preexisting_static_asset" in world_text
    assert report["controller_runtime_binding"] == written_report["controller_runtime_binding"]
    assert written_report["controller_runtime_binding"]["controller_config_sha256"] == MODULE._sha256(controller)


def test_run_marks_raw_layout_diagnostic_as_not_formal(tmp_path: Path, monkeypatch):
    source_world = tmp_path / "world.sdf"
    urdf = tmp_path / "vehicle.urdf"
    output_world = tmp_path / "prepared.sdf"
    output_report = tmp_path / "prepared.json"
    install_root, controller = _runtime_controller_fixture(tmp_path)
    source_world.write_text(
        "<sdf version='1.11'><world name='w'/></sdf>", encoding="utf-8"
    )
    urdf.write_text(
        "<robot name='fixture'><link name='base'/><gazebo reference='base'>"
        "<sensor name='scan' type='gpu_lidar'/></gazebo></robot>",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        MODULE,
        "convert_urdf",
        lambda _gz, _urdf: (
            "<sdf version='1.11'><model name='fixture'><link name='base'>"
            "<sensor name='scan' type='gpu_lidar'/></link>"
            "<plugin filename='gz_ros2_control-system' "
            "name='gz_ros2_control::GazeboSimROS2ControlPlugin'><parameters>"
            f"{MODULE.CANONICAL_CONTROLLER_URI}</parameters></plugin></model></sdf>"
        ),
    )
    args = argparse.Namespace(
        source_world=str(source_world), vehicle_urdf=str(urdf),
        output_world=str(output_world), report=str(output_report),
        controller_config=str(controller), runtime_install_root=str(install_root),
        gz="gz", only_sensor=[], model_pose="0 0 0.005 0 0 0",
        diagnostic_skip_attachment_restoration=True,
    )

    report = MODULE.run(args)

    assert report["status"] == "DIAGNOSTIC_NOT_FORMAL_PREEMBEDDED_WORLD"
    assert report["formal_eligible"] is False
    assert report["diagnostic_skip_attachment_restoration"] is True
    assert report["sensor_count"] == 1


def test_build_preembedded_world_rejects_converted_sensor_omission(tmp_path: Path):
    world = tmp_path / "world.sdf"
    urdf = tmp_path / "vehicle.urdf"
    world.write_text("<sdf version='1.11'><world name='w'/></sdf>", encoding="utf-8")
    urdf.write_text(
        "<robot name='fixture'><link name='base'/><gazebo reference='base'>"
        "<sensor name='required' type='camera'/></gazebo></robot>",
        encoding="utf-8",
    )
    try:
        MODULE.build_preembedded_world(
            world,
            "<sdf version='1.11'><model name='fixture'><link name='base'/></model></sdf>",
            urdf,
        )
    except MODULE.PreparationError as error:
        assert "omitted formal sensors" in str(error)
    else:
        raise AssertionError("converted model without required sensor was accepted")


@pytest.mark.parametrize(
    "urdf_name,converted,error",
    [
        ("fixture", "<sdf version='1.11'/>", "exactly one direct converted model"),
        (
            "fixture",
            "<sdf version='1.11'><model name='fixture'/><model name='other'/></sdf>",
            "exactly one direct converted model",
        ),
        (
            "fixture",
            "<sdf version='1.11'><model name='other'><link name='base'>"
            "<sensor name='scan' type='gpu_lidar'/></link></model></sdf>",
            "converted model name differs",
        ),
        (
            "",
            "<sdf version='1.11'><model name='fixture'><link name='base'>"
            "<sensor name='scan' type='gpu_lidar'/></link></model></sdf>",
            "robot name is missing",
        ),
    ],
)
def test_build_rejects_ambiguous_or_unbound_model_identity(
    tmp_path: Path, urdf_name: str, converted: str, error: str
) -> None:
    world = tmp_path / "world.sdf"
    urdf = tmp_path / "vehicle.urdf"
    world.write_text("<sdf version='1.11'><world name='w'/></sdf>", encoding="utf-8")
    name_attribute = f" name='{urdf_name}'" if urdf_name else ""
    urdf.write_text(
        f"<robot{name_attribute}><link name='base'/><gazebo reference='base'>"
        "<sensor name='scan' type='gpu_lidar'/></gazebo></robot>",
        encoding="utf-8",
    )
    with pytest.raises(MODULE.PreparationError, match=error):
        MODULE.build_preembedded_world(world, converted, urdf)


def test_production_urdf_declares_every_render_sensor_on_a_reference_link():
    root = ET.parse(ROOT / "reports/engineering/formal_competition_vehicle.urdf").getroot()
    attachments = MODULE.sensor_attachment_contract(root)
    assert {"utm30lx", "mid360", "front_rgbd_d435_rgbd", "wrist_rgbd_d435_rgbd"} <= set(attachments)
    assert attachments["utm30lx"][0] == "lidar_2d_link"
    assert attachments["mid360"][0] == "lidar_3d_link"
    assert attachments["wrist_rgbd_d435_rgbd"][0] == "wrist_rgbd_link"
    assert attachments["utm30lx"][2] == "gpu_lidar"


def test_production_expanded_urdf_has_one_complete_control_authority():
    root = ET.parse(ROOT / "reports/engineering/formal_competition_vehicle.urdf").getroot()
    candidates = [
        plugin
        for plugin in root.findall("gazebo/plugin")
        if plugin.get("filename") == MODULE.FORMAL_CONTROLLER_FILENAME
        or plugin.get("name") == MODULE.FORMAL_CONTROLLER_NAME
    ]
    assert len(candidates) == 1
    plugin = candidates[0]
    assert plugin.get("filename") == MODULE.FORMAL_CONTROLLER_FILENAME
    assert plugin.get("name") == MODULE.FORMAL_CONTROLLER_NAME
    assert [item.text for item in plugin.findall("parameters")] == [
        MODULE.CANONICAL_CONTROLLER_URI
    ]
