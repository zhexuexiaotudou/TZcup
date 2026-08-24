from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO = PACKAGE_ROOT / "urdf" / "placeholder_mobile_manipulator.urdf.xacro"
SRDF = PACKAGE_ROOT / "config" / "placeholder_mobile_manipulator.srdf"
PROFILE = PACKAGE_ROOT / "config" / "placeholder_profile.yaml"


def test_placeholder_xacro_is_well_formed_and_prominently_non_authoritative():
    root = ET.parse(XACRO).getroot()
    text = XACRO.read_text(encoding="utf-8")
    assert root.attrib["name"] == "placeholder_mobile_manipulator"
    assert "PLACEHOLDER ONLY" in text
    assert "real-robot" in text
    for argument in (
        "base_length",
        "base_width",
        "arm_mount_x",
        "arm_link_1",
        "arm_link_6",
        "gripper_opening",
        "bin_length",
        "bin_width",
        "bin_depth",
        "wrist_stereo_baseline",
    ):
        assert f'name="{argument}"' in text


def test_placeholder_description_contains_required_frames_and_six_axis_chain():
    text = XACRO.read_text(encoding="utf-8")
    for name in (
        "single_lidar_link",
        "mid360_link",
        "front_rgbd_link",
        "rear_left_fisheye_link",
        "rear_right_fisheye_link",
        "wrist_stereo_left_optical_frame",
        "wrist_stereo_right_optical_frame",
        "rear_collection_bin_link",
        "bin_drop_frame",
    ):
        assert name in text
    for index in range(1, 7):
        assert f'index="{index}"' in text
    for wheel in ("front_left", "front_right", "rear_left", "rear_right"):
        assert f'name="{wheel}"' in text
    assert text.count('collision name="bin_') == 5


def test_placeholder_srdf_and_profile_preserve_evidence_boundary():
    semantic = ET.parse(SRDF).getroot()
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    assert semantic.attrib["name"] == "placeholder_mobile_manipulator"
    assert profile["placeholder"] is True
    assert profile["evidence_authority"] is False
    assert profile["vehicle"]["body_size_m"][:2] == [0.60, 0.40]
    assert profile["object"]["edge_m"] == 0.030
    assert profile["object"]["max_targets_per_episode"] == 20
    assert profile["bin"]["internal_size_m"] == [0.20, 0.20, 0.10]
    assert profile["manipulator"]["max_grasp_attempts"] == 2
    assert "measured_urdf" in profile["replacement_gate"]["requires"]
