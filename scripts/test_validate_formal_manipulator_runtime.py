import ast
import hashlib
import json
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = ROOT / "scripts" / "validate_formal_manipulator_runtime.py"


def _binding_helpers() -> tuple[object, object]:
    """Load only the pure binding helpers without importing ROS on Windows."""

    tree = ast.parse(VALIDATOR.read_text(encoding="utf-8"), filename=str(VALIDATOR))
    helpers = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {"snapshot_binding", "bound_runtime_evidence"}
    ]
    namespace: dict[str, object] = {
        "Path": Path,
        "hashlib": hashlib,
        "json": json,
        "time": time,
    }
    exec(compile(ast.Module(body=helpers, type_ignores=[]), str(VALIDATOR), "exec"), namespace)
    return namespace["snapshot_binding"], namespace["bound_runtime_evidence"]


def _joint_state_measurement_helper() -> object:
    """Load the pure JointState alignment helper without importing ROS on Windows."""

    tree = ast.parse(VALIDATOR.read_text(encoding="utf-8"), filename=str(VALIDATOR))
    helpers = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "aligned_joint_state_measurements"
    ]
    namespace: dict[str, object] = {"math": __import__("math")}
    exec(compile(ast.Module(body=helpers, type_ignores=[]), str(VALIDATOR), "exec"), namespace)
    return namespace["aligned_joint_state_measurements"]


def test_validator_fails_closed_on_session_snapshot_and_runtime_binding_drift() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    assert "from formal_runtime_gate_binding import load_binding" in source
    assert "formal acceptance session must be RUNNING" in source
    assert "runtime binding snapshot differs from manipulator source binding" in source
    assert "runtime binding session differs from manipulator session" in source
    assert "runtime binding timestamp is outside the active acceptance session" in source
    assert "runtime binding file timestamp is outside the active acceptance session" in source
    assert "runtime binding closure has invalid" in source
    assert "runtime binding closure contains symbolic links" in source
    assert 'parser.add_argument("--runtime-binding", type=Path, required=True)' in source
    assert "runtime_gate_binding" in source
    assert "runtime_identity" in source


def test_validator_requires_live_evidence_for_all_five_gripper_followers() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    for follower in (
        "robotiq_85_right_knuckle_joint",
        "robotiq_85_left_inner_knuckle_joint",
        "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint",
        "robotiq_85_right_finger_tip_joint",
    ):
        assert follower in source
    assert "def follower_runtime_evidence" in source
    assert "tracked_joints = ARM_JOINTS + [GRIPPER_JOINT] + list(MIMIC_RELATIONS)" in source
    assert "LIVE_JOINT_STATE_TRACKED" in source
    assert "mimic follower lacks live joint-state evidence" in source
    assert "mimic follower runtime tracking failed" in source
    assert "JointState arrays must align with name" in source
    assert "velocity_range_rad_s" in source
    assert "peak_abs_velocity_rad_s" in source
    assert "peak_abs_effort_nm" in source
    assert "max_noncontact_tracking_error_rad" in source
    assert "FOLLOWER_EFFORT_LIMIT_NM = 12.0" in source
    assert "FOLLOWER_EFFORT_TOLERANCE_NM = 0.05" in source
    assert "all_followers_observed_effort_within_limit" in source
    assert "JointForceCmd" in source
    assert "measured JointState observations" in source
    assert '"mimic_follower_runtime_evidence": follower_evidence' in source
    assert "live /joint_states from the running Gazebo controller graph" in source
    assert "static-URDF substitute" in source
    assert "for follower motion" in source


def test_joint_state_alignment_collects_all_fields_by_name_and_fails_closed() -> None:
    align = _joint_state_measurement_helper()
    observed = align(  # type: ignore[operator]
        ["joint_b", "joint_a"],
        [0.2, -0.1],
        [1.5, -2.0],
        [3.0, -4.0],
    )
    assert observed == {
        "joint_b": {"position_rad": 0.2, "velocity_rad_s": 1.5, "effort_nm": 3.0},
        "joint_a": {"position_rad": -0.1, "velocity_rad_s": -2.0, "effort_nm": -4.0},
    }
    for positions, velocities, efforts in (
        ([], [0.0], [0.0]),
        ([0.0], [], [0.0]),
        ([0.0], [0.0], []),
    ):
        with pytest.raises(ValueError, match="align with name"):
            align(["joint_a"], positions, velocities, efforts)  # type: ignore[operator]
    with pytest.raises(ValueError, match="duplicate"):
        align(["joint_a", "joint_a"], [0.0, 0.1], [0.0, 0.0], [0.0, 0.0])  # type: ignore[operator]


def test_binding_helper_rejects_a_sidecar_from_another_snapshot(tmp_path: Path) -> None:
    snapshot_binding, bound_runtime_evidence = _binding_helpers()
    snapshot = tmp_path / "snapshot.json"
    session = tmp_path / "session.json"
    sidecar = tmp_path / "runtime_binding.json"
    snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "a" * 64,
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {"sha256": "b" * 64}
                },
            }
        ),
        encoding="utf-8",
    )
    started_epoch_ns = time.time_ns() - 1_000_000_000
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_epoch_ns,
            }
        ),
        encoding="utf-8",
    )
    sidecar.write_text("{}", encoding="utf-8")
    source = snapshot_binding(snapshot)  # type: ignore[operator]
    session_sha = hashlib.sha256(session.read_bytes()).hexdigest()
    matching_binding = {
        "verified_epoch_ns": time.time_ns(),
        "acceptance_session_binding": {
            "snapshot": source,
            "session_manifest_sha256": session_sha,
            "session_started_epoch_ns": started_epoch_ns,
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "runtime_install_root": "/frozen/runtime/install",
            "manifest_sha256": "d" * 64,
            "closure_sha256": "e" * 64,
            "symbolic_link_count": 0,
        },
    }

    namespace_binding = bound_runtime_evidence.__globals__  # type: ignore[attr-defined]
    namespace_binding["load_binding"] = lambda _: matching_binding
    _, _, observed = bound_runtime_evidence(snapshot, session, sidecar)  # type: ignore[operator]
    assert observed is matching_binding

    mismatched = json.loads(json.dumps(matching_binding))
    mismatched["acceptance_session_binding"]["snapshot"]["expanded_urdf_sha256"] = "c" * 64
    namespace_binding["load_binding"] = lambda _: mismatched
    with pytest.raises(ValueError, match="snapshot differs"):
        bound_runtime_evidence(snapshot, session, sidecar)  # type: ignore[operator]
