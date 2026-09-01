from __future__ import annotations

from pathlib import Path

import pytest

from generate_formal_vehicle_snapshot import (
    CANONICAL_CONTROLLER_URI,
    ENTRYPOINT,
    SnapshotError,
    _canonical_digest,
    _exclusive_generation_lock,
    _inventory,
    _validate_expanded_urdf_paths,
    _validate_sim_gripper_authority,
    authoritative_source_paths,
    generate_snapshot,
    verify_snapshot,
)


ROOT = Path(__file__).resolve().parents[1]


def test_committed_formal_vehicle_snapshot_manifest_is_current() -> None:
    manifest = verify_snapshot(ROOT)
    assert manifest["profile"]["entrypoint"] == ENTRYPOINT.as_posix()
    assert manifest["profile"]["use_sim"] is True
    assert (
        manifest["profile"]["gripper_linkage_mode"]
        == "gazebo_compliant_effort_plugin_no_urdf_mimic"
    )
    assert manifest["profile"]["xacro_version"] != "unreported"


def test_authoritative_inventory_covers_all_formal_xacros_and_contracts() -> None:
    sources = set(authoritative_source_paths(ROOT))
    xacro_root = ROOT / ENTRYPOINT.parent / "high_fidelity"
    expected_xacros = {path.relative_to(ROOT) for path in xacro_root.glob("*.xacro")}
    assert expected_xacros
    assert expected_xacros <= sources
    assert ENTRYPOINT in sources
    assert Path("config/high_fidelity_vehicle/formal_vehicle_layout.yaml") in sources
    assert Path("config/high_fidelity_vehicle/formal_vehicle_component_register.yaml") in sources
    assert Path("scripts/generate_formal_vehicle_snapshot.py") in sources
    assert Path("scripts/formal_gripper_linkage_contract.py") in sources
    mesh_root = ROOT / "starter_ws/src/sanitation_vehicle_description/meshes"
    expected_meshes = {
        path.relative_to(ROOT)
        for path in mesh_root.rglob("*")
        if path.is_file() and path.suffix.lower() in {".dae", ".png", ".stl"}
    }
    assert expected_meshes
    assert expected_meshes <= sources


def test_inventory_digest_detects_source_or_output_mutation(tmp_path: Path) -> None:
    relative = Path("source.xacro")
    (tmp_path / relative).write_text("first\n", encoding="utf-8")
    before = _inventory(tmp_path, (relative,))
    (tmp_path / relative).write_text("second\n", encoding="utf-8")
    after = _inventory(tmp_path, (relative,))
    assert before != after
    assert _canonical_digest(before) != _canonical_digest(after)


def test_path_gate_preserves_sim_plugin_and_accepts_canonical_uri() -> None:
    raw = (
        '<robot><gazebo><plugin filename="gz_ros2_control-system">'
        f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>"
        "</plugin></gazebo></robot>\n"
    )
    validated = _validate_expanded_urdf_paths(raw, ROOT)
    assert "gz_ros2_control-system" in validated
    assert f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>" in validated


def test_sim_gripper_authority_rejects_duplicate_or_missing_writer() -> None:
    followers = "".join(
        f'<joint name="{name}" type="revolute"><parent link="base"/>'
        f'<child link="{name}_link"/></joint>'
        for name in (
            "robotiq_85_right_knuckle_joint",
            "robotiq_85_left_inner_knuckle_joint",
            "robotiq_85_right_inner_knuckle_joint",
            "robotiq_85_left_finger_tip_joint",
            "robotiq_85_right_finger_tip_joint",
        )
    )
    without_plugin = f"<robot>{followers}</robot>"
    with pytest.raises(SnapshotError, match="exactly one"):
        _validate_sim_gripper_authority(without_plugin)

    duplicate = without_plugin.replace(
        "</joint>",
        '<mimic joint="robotiq_85_left_knuckle_joint"/></joint>',
        1,
    ).replace(
        "</robot>",
        '<gazebo><plugin filename="libGripperMimicEffortSystem.so"/></gazebo></robot>',
    )
    with pytest.raises(SnapshotError, match="duplicate URDF mimic"):
        _validate_sim_gripper_authority(duplicate)


@pytest.mark.parametrize("path", ["/mnt/f/work/formal_vehicle_controllers.yaml", "C:/work/formal_vehicle_controllers.yaml"])
def test_path_gate_rejects_machine_specific_controller_path(path: str) -> None:
    raw = f"<robot><gazebo><plugin><parameters>{path}</parameters></plugin></gazebo></robot>"
    with pytest.raises(SnapshotError, match="canonical controller parameter URI"):
        _validate_expanded_urdf_paths(raw, ROOT)


@pytest.mark.parametrize(
    "path",
    [
        "/tmp/foreign.stl",
        "/opt/foreign.stl",
        "file:///tmp/foreign.stl",
        r"C:\work\foreign.stl",
        r"\\server\share\foreign.stl",
    ],
)
def test_path_gate_rejects_absolute_mesh_reference(path: str) -> None:
    raw = (
        '<robot><gazebo><plugin filename="gz_ros2_control-system">'
        f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>"
        "</plugin></gazebo><link name=\"base\"><visual><geometry>"
        f'<mesh filename="{path}"/>'
        "</geometry></visual></link></robot>"
    )
    with pytest.raises(SnapshotError, match="absolute filesystem reference"):
        _validate_expanded_urdf_paths(raw, ROOT)


def test_path_gate_allows_absolute_ros_topics() -> None:
    raw = (
        '<robot><gazebo><plugin filename="gz_ros2_control-system">'
        f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>"
        "</plugin><sensor><topic>/sensors/lidar/scan</topic></sensor>"
        "</gazebo></robot>"
    )
    assert "/sensors/lidar/scan" in _validate_expanded_urdf_paths(raw, ROOT)


def test_path_gate_allows_absolute_ros_topic_prefixes() -> None:
    raw = (
        '<robot><gazebo><plugin filename="gz_ros2_control-system">'
        f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>"
        "</plugin><plugin>"
        "<telemetry_topic_prefix>/model/vehicle/squeegee</telemetry_topic_prefix>"
        "</plugin></gazebo></robot>"
    )
    assert "/model/vehicle/squeegee" in _validate_expanded_urdf_paths(raw, ROOT)


def test_path_gate_rejects_package_uri_parent_traversal() -> None:
    traversal = (
        "package://sanitation_vehicle_description/"
        "../sanitation_gazebo_control/package.xml"
    )
    raw = (
        '<robot><gazebo><plugin filename="gz_ros2_control-system">'
        f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>"
        "</plugin></gazebo><link name=\"base\"><visual><geometry>"
        f'<mesh filename="{traversal}"/>'
        "</geometry></visual></link></robot>"
    )
    with pytest.raises(SnapshotError, match="non-canonical package URI"):
        _validate_expanded_urdf_paths(raw, ROOT)


def test_generation_fails_closed_without_xacro(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("generate_formal_vehicle_snapshot.shutil.which", lambda _: None)
    with pytest.raises(SnapshotError, match="xacro is unavailable"):
        generate_snapshot(ROOT)


def test_snapshot_generation_lock_is_exclusive_and_released(tmp_path: Path) -> None:
    with _exclusive_generation_lock(tmp_path):
        with pytest.raises(SnapshotError, match="already locked"):
            with _exclusive_generation_lock(tmp_path):
                pass
    with _exclusive_generation_lock(tmp_path):
        assert (tmp_path / "reports/engineering/.formal_vehicle_snapshot.lock").is_file()
    assert not (tmp_path / "reports/engineering/.formal_vehicle_snapshot.lock").exists()
