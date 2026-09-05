from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
import re

import pytest

from formal_gripper_linkage_contract import (
    FormalGripperLinkageError,
    GRIPPER_FOLLOWER_RELATIONS,
    GRIPPER_MASTER_JOINT,
    resolve_mimic_relations,
)
from scan_formal_vehicle_inertia_and_swept_volume import Model, ScanError
from validate_formal_fov_occlusion import UrdfModel


ROOT = Path(__file__).resolve().parents[1]
SIM_URDF = ROOT / "reports/engineering/formal_competition_vehicle.urdf"


def _formal_relations(include_mimics: bool) -> dict[str, tuple[str, float, float] | None]:
    relations: dict[str, tuple[str, float, float] | None] = {
        GRIPPER_MASTER_JOINT: None
    }
    relations.update(
        {
            name: relation if include_mimics else None
            for name, relation in GRIPPER_FOLLOWER_RELATIONS.items()
        }
    )
    return relations


def test_contract_accepts_exact_planning_graph_and_synthesizes_sim_graph() -> None:
    assert {
        name: relation
        for name, relation in resolve_mimic_relations(
            _formal_relations(include_mimics=True)
        ).items()
        if name in GRIPPER_FOLLOWER_RELATIONS
    } == GRIPPER_FOLLOWER_RELATIONS
    assert resolve_mimic_relations(
        _formal_relations(include_mimics=False)
    ) == GRIPPER_FOLLOWER_RELATIONS


def test_contract_rejects_partial_and_contradictory_formal_graphs() -> None:
    partial = _formal_relations(include_mimics=False)
    partial["robotiq_85_right_knuckle_joint"] = GRIPPER_FOLLOWER_RELATIONS[
        "robotiq_85_right_knuckle_joint"
    ]
    with pytest.raises(FormalGripperLinkageError, match="partial"):
        resolve_mimic_relations(partial)

    contradictory = _formal_relations(include_mimics=True)
    contradictory["robotiq_85_right_knuckle_joint"] = (
        GRIPPER_MASTER_JOINT,
        1.0,
        0.0,
    )
    with pytest.raises(FormalGripperLinkageError, match="expected"):
        resolve_mimic_relations(contradictory)


def test_canonical_sim_urdf_omits_metadata_but_static_tools_restore_physics() -> None:
    root = ET.parse(SIM_URDF).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert all(joints[name].find("mimic") is None for name in GRIPPER_FOLLOWER_RELATIONS)

    fov_model = UrdfModel(SIM_URDF)
    fov_by_name = {
        fov_model.joint_names_by_child[child]: joint
        for child, joint in fov_model.joints.items()
    }
    scan_model = Model(SIM_URDF)
    for name, expected in GRIPPER_FOLLOWER_RELATIONS.items():
        assert fov_by_name[name].mimic == expected
        scan_joint = scan_model.joints[name]
        assert (
            scan_joint.mimic_joint,
            scan_joint.mimic_multiplier,
            scan_joint.mimic_offset,
        ) == expected


def test_python_contract_matches_gazebo_effort_plugin_arrays() -> None:
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/GripperMimicEffortSystem.cc"
    ).read_text(encoding="utf-8")
    names_match = re.search(
        r"kFollowerJointNames\s*\{(?P<body>.*?)\};", source, re.DOTALL
    )
    multipliers_match = re.search(
        r"kFollowerMultipliers\s*\{(?P<body>.*?)\};", source, re.DOTALL
    )
    assert names_match is not None
    assert multipliers_match is not None
    names = re.findall(r'"([^"]+)"', names_match.group("body"))
    multipliers = [
        float(value)
        for value in re.findall(r"[-+]?\d+(?:\.\d+)?", multipliers_match.group("body"))
    ]
    expected_names = list(GRIPPER_FOLLOWER_RELATIONS)
    expected_multipliers = [
        GRIPPER_FOLLOWER_RELATIONS[name][1] for name in expected_names
    ]
    assert names == expected_names
    assert multipliers == expected_multipliers


def test_static_models_reject_duplicate_joint_names_before_contract_resolution(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "duplicate.urdf"
    urdf.write_text(
        "<robot name='duplicate'>"
        "<link name='base_footprint'/><link name='first'/><link name='second'/>"
        "<joint name='duplicate_joint' type='fixed'><parent link='base_footprint'/><child link='first'/></joint>"
        "<joint name='duplicate_joint' type='fixed'><parent link='base_footprint'/><child link='second'/></joint>"
        "</robot>",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="duplicate joint names"):
        UrdfModel(urdf)
    with pytest.raises(ScanError, match="duplicate joint names"):
        Model(urdf)
