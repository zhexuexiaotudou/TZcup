from pathlib import Path

from sanitation_formal_campus_integration.dynamic_footprint_core import (
    ARM_JOINTS,
    ARM_STOWED,
    load_footprints,
    select_profile,
)


ROOT = Path(__file__).resolve().parents[4]


def _stowed() -> dict[str, float]:
    return dict(zip(ARM_JOINTS, ARM_STOWED, strict=True))


def test_fail_closed_until_all_arm_joints_are_known() -> None:
    assert select_profile({}, False) == "arm_deployed"


def test_arm_inhibit_or_joint_motion_selects_arm_envelope() -> None:
    joints = _stowed()
    assert select_profile(joints, True) == "arm_deployed"
    joints["elbow_joint"] += 0.2
    assert select_profile(joints, False) == "arm_deployed"


def test_cleaning_and_transport_profiles_follow_lift_position() -> None:
    joints = _stowed()
    joints["cleaning_lift_joint"] = 0.100
    assert select_profile(joints, False) == "cleaning_deployed"
    joints["cleaning_lift_joint"] = 0.00
    assert select_profile(joints, False) == "transport_stowed"


def test_formal_profile_defines_all_runtime_envelopes() -> None:
    profile = load_footprints(
        ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
    )
    assert set(profile) == {"transport_stowed", "cleaning_deployed", "arm_deployed"}
    assert max(y for _, y in profile["cleaning_deployed"]) > max(
        y for _, y in profile["transport_stowed"]
    )
