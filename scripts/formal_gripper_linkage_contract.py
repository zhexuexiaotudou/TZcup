"""Shared physical-linkage contract for the formal Robotiq 2F-85 model.

The Gazebo description intentionally omits URDF ``mimic`` tags because the
project-owned compliant effort plugin is the sole physical writer for the five
passive joints.  Static geometry tools still need the same deterministic
one-actuator kinematics.  This module keeps those two representations bound to
one fail-closed contract instead of duplicating unvalidated fallback values.
"""

from __future__ import annotations

from collections.abc import Mapping


GRIPPER_MASTER_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_FOLLOWER_RELATIONS: dict[str, tuple[str, float, float]] = {
    "robotiq_85_right_knuckle_joint": (GRIPPER_MASTER_JOINT, -1.0, 0.0),
    "robotiq_85_left_inner_knuckle_joint": (GRIPPER_MASTER_JOINT, 1.0, 0.0),
    "robotiq_85_right_inner_knuckle_joint": (GRIPPER_MASTER_JOINT, -1.0, 0.0),
    "robotiq_85_left_finger_tip_joint": (GRIPPER_MASTER_JOINT, -1.0, 0.0),
    "robotiq_85_right_finger_tip_joint": (GRIPPER_MASTER_JOINT, 1.0, 0.0),
}


class FormalGripperLinkageError(ValueError):
    """Raised when a formal gripper is incomplete or contradicts its linkage."""


def resolve_mimic_relations(
    explicit_relations: Mapping[str, tuple[str, float, float] | None],
) -> dict[str, tuple[str, float, float]]:
    """Return explicit mimics plus the validated formal physical linkage.

    A generic URDF without any formal 2F-85 joint passes through unchanged.  If
    any formal joint is present, the master and all five followers are required.
    The description must then contain either all five exact URDF mimics
    (planning / real description) or none of them (Gazebo physical description).
    A partial or contradictory graph is rejected.
    """

    resolved = {
        name: relation
        for name, relation in explicit_relations.items()
        if relation is not None
    }
    formal_names = {GRIPPER_MASTER_JOINT, *GRIPPER_FOLLOWER_RELATIONS}
    present = formal_names.intersection(explicit_relations)
    if not present:
        return resolved
    missing = sorted(formal_names.difference(explicit_relations))
    if missing:
        raise FormalGripperLinkageError(
            "formal Robotiq linkage is incomplete; missing joints: "
            + ", ".join(missing)
        )
    if explicit_relations[GRIPPER_MASTER_JOINT] is not None:
        raise FormalGripperLinkageError("formal Robotiq master joint cannot mimic another joint")

    explicit_followers = {
        name for name in GRIPPER_FOLLOWER_RELATIONS if explicit_relations[name] is not None
    }
    if explicit_followers and explicit_followers != set(GRIPPER_FOLLOWER_RELATIONS):
        missing_mimics = sorted(set(GRIPPER_FOLLOWER_RELATIONS) - explicit_followers)
        raise FormalGripperLinkageError(
            "formal Robotiq mimic graph is partial; missing relations: "
            + ", ".join(missing_mimics)
        )
    for name, expected in GRIPPER_FOLLOWER_RELATIONS.items():
        actual = explicit_relations[name]
        if actual is not None and actual != expected:
            raise FormalGripperLinkageError(
                f"formal Robotiq mimic relation for {name} is {actual}, expected {expected}"
            )
        resolved[name] = expected
    return resolved
