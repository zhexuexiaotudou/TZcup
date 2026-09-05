from __future__ import annotations

import hashlib
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from validate_formal_fov_occlusion import (
    ARM_JOINTS,
    ARM_POSES,
    SENSORS,
    _aabb,
    _build_bvh,
    _inside_fov,
    _nearest,
    _stl,
    compact_report,
    _portable_evidence_path,
    validate,
)


ROOT = Path(__file__).resolve().parents[1]
LAYOUT = ROOT / "config/high_fidelity_vehicle/formal_vehicle_layout.yaml"
SENSOR_XACRO = ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/sensor_suite.xacro"
CUBE_RUNTIME = ROOT / "scripts/validate_formal_cube_pick_place_runtime.py"
SWEEP_SCANNER = ROOT / "scripts/scan_formal_vehicle_inertia_and_swept_volume.py"
MOTION_PROFILE = ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"


def test_parallel_ray_aabb_is_stable_on_box_face() -> None:
    lower = np.asarray((0.0, 0.0, 0.0))
    upper = np.asarray((1.0, 1.0, 1.0))
    assert _aabb(np.asarray((0.0, 0.5, -1.0)), np.asarray((0.0, 0.0, 1.0)), lower, upper, 3.0)
    assert not _aabb(np.asarray((-0.1, 0.5, -1.0)), np.asarray((0.0, 0.0, 1.0)), lower, upper, 3.0)


def test_ray_edge_noise_is_conservatively_blocked_but_bounded() -> None:
    triangles = np.asarray([[(0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (0.0, 1.0, 1.0)]])
    owners = np.asarray(["bracket"], dtype=object)
    bvh = _build_bvh(triangles)
    origin = np.zeros(3)

    for offset in (-5e-10, 5e-10):
        hit, _ = _nearest(origin, np.asarray((offset, 0.25, 1.0)), 2.0, triangles, owners, bvh, set())
        assert hit == 0
    hit, _ = _nearest(origin, np.asarray((-2e-9, 0.25, 1.0)), 2.0, triangles, owners, bvh, set())
    assert hit is None


def test_near_equal_hits_use_stable_owner_but_real_nearer_hit_wins() -> None:
    triangle = np.asarray([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
    origin = np.asarray((0.25, 0.25, 0.0))
    direction = np.asarray((0.0, 0.0, 1.0))

    for order in (("zeta", 1.0), ("alpha", 1.0 + 5e-10)), (("alpha", 1.0 + 5e-10), ("zeta", 1.0)):
        triangles = np.asarray([triangle + (0.0, 0.0, height) for _, height in order])
        owners = np.asarray([owner for owner, _ in order], dtype=object)
        hit, distance = _nearest(origin, direction, 2.0, triangles, owners, _build_bvh(triangles), set())
        assert owners[hit] == "alpha"
        assert distance == pytest.approx(1.0)

    triangles = np.asarray([triangle + (0.0, 0.0, 1.0), triangle + (0.0, 0.0, 1.0 + 2e-9)])
    owners = np.asarray(["zeta", "alpha"], dtype=object)
    hit, _ = _nearest(origin, direction, 2.0, triangles, owners, _build_bvh(triangles), set())
    assert owners[hit] == "zeta"


def test_stl_parser_returns_digest_from_the_same_read(tmp_path: Path) -> None:
    raw = b"""solid one\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid one\n"""
    path = tmp_path / "one_triangle.stl"
    path.write_bytes(raw)

    triangles, digest = _stl(path)

    assert triangles.shape == (1, 3, 3)
    assert digest == hashlib.sha256(raw).hexdigest()


def test_camera_and_lidar_fov_boundaries_are_fail_closed() -> None:
    camera = SENSORS["front_d435_depth"]
    assert _inside_fov(np.asarray((0.0, 0.0, 1.0)), camera)
    assert not _inside_fov(np.asarray((math.tan(math.radians(44.0)), 0.0, 1.0)), camera)
    lidar = SENSORS["utm30lx"]
    assert _inside_fov(np.asarray((1.0, 0.0, 0.0)), lidar)
    assert not _inside_fov(np.asarray((-1.0, 0.0, 0.0)), lidar)


def test_arm_pose_set_matches_physical_runtime_and_sweep_gate() -> None:
    runtime = CUBE_RUNTIME.read_text(encoding="utf-8")
    sweep = SWEEP_SCANNER.read_text(encoding="utf-8")
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))["arm_task_poses"]
    assert profile["joint_names"] == list(ARM_JOINTS)
    for pose_name, source_name in (("pregrasp", "PREGRASP"), ("pick", "PICK"), ("deposit", "DEPOSIT")):
        values = [ARM_POSES[pose_name][name] for name in ARM_JOINTS]
        assert profile[f"{pose_name}_rad"] == pytest.approx(values)
        literal = "[" + ", ".join(str(value) for value in values) + "]"
        assert source_name in runtime
        for value in values:
            assert f"{value}" in runtime
    transport = [ARM_POSES["transport"][name] for name in ARM_JOINTS]
    assert profile["transport_rad"] == pytest.approx(transport)
    for value in transport:
        assert f"{value}" in sweep
    assert ARM_POSES["deposit"]["dry_deposit_gate_joint"] == pytest.approx(1.05)
    assert profile["deposit_auxiliary_joint_positions"]["dry_deposit_gate_joint_rad"] == pytest.approx(1.05)
    control = (ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro").read_text(encoding="utf-8")
    for joint_name, value in zip(ARM_JOINTS, transport):
        assert f'name="{joint_name}"' in control
        assert f'initial_position="{value}"' in control


def test_layout_uses_exact_tool0_wrist_datum_and_downward_rear_axes() -> None:
    layout = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    utm = next(item for item in layout["sensor_layout"] if item["id"] == "utm30lx")
    assert utm["xyz_m"] == pytest.approx([0.535, 0.0, 1.1621])
    assert utm["approximate_clear_fraction"] >= utm["minimum_clear_fraction"]
    front = next(item for item in layout["sensor_layout"] if item["id"] == "front_d435_depth")
    assert front["forward_xyz"] == pytest.approx(
        [math.cos(math.radians(25.0)), 0.0, -math.sin(math.radians(25.0))],
        abs=1e-6,
    )
    wrist = next(item for item in layout["sensor_layout"] if item["id"] == "wrist_d435_depth")
    assert wrist["coordinate_reference"] == "tool0"
    assert wrist["xyz_m"] == pytest.approx([0.140, 0.0, 0.025])
    assert wrist["approximate_clear_fraction"] == pytest.approx(0.966946779)
    assert wrist["mount_clearance_contract"] == "rear_plane_dogleg_all_metal_x_le_minus_0.0125_m"
    for sensor_id in ("rear_left_fisheye", "rear_right_fisheye"):
        sensor = next(item for item in layout["sensor_layout"] if item["id"] == sensor_id)
        assert sensor["forward_xyz"][2] < -0.08
    source = SENSOR_XACRO.read_text(encoding="utf-8")
    assert '<origin xyz="0.060 0 0.695"/>' in source
    assert '<origin xyz="0.565 0 0.449" rpy="0 0.436332313 0"/>' in source
    assert source.count('optical_rpy="-1.832595714594 -0.000035461162 -1.570996889205"') == 2


def test_wrist_bracket_stays_behind_depth_frustum_and_mass_is_preserved() -> None:
    source = SENSOR_XACRO.read_text(encoding="utf-8")
    assert "Every load-carrying bracket member" in source
    for origin in (
        '-0.016 0.010 0.027', '-0.020 0.004 0.029', '-0.024 0.070 0.060',
        '-0.024 0.140 0.049', '-0.044 0.124 0.049',
    ):
        assert f'xyz="{origin}"' in source
    assert '<mass value="0.236709"/>' in source
    assert SENSORS["wrist_d435_depth"]["ignore"] == {"wrist_rgbd_link"}


def test_fisheye_parameter_audit_requires_native_equisolid_projection() -> None:
    validator = (ROOT / "scripts/validate_formal_fov_occlusion.py").read_text(encoding="utf-8")
    assert validator.count('"type": "wideanglecamera"') == 2
    assert validator.count('"lens_type": "equisolid_angle"') == 2
    assert '_text(element, "camera/lens/scale_to_hfov") == "true"' in validator
    assert 'camera/lens/cutoff_angle' in validator


def test_compact_report_preserves_gate_counts_without_raw_target_rows() -> None:
    pose = {"clear_fraction": 1.0, "worst_blocked_rays": [{"ray_index": 1}]}
    report = {
        "sensor_results": {"sensor": {"required_pose_results": {"p": dict(pose)}, "all_pose_results": {"p": dict(pose)}}},
        "functional_zone_coverage": {
            "front_ground_observation": {"pose_results": {"p": {"targets": [{"visible": True}, {"visible": False}]}}},
            "rear_fisheye_safety_perimeter": {"pose_results": {"p": {"left_targets": [{"visible": True}], "right_targets": [{"visible": False}]}}},
            "wrist_pregrasp_cube": {"targets": [{"visible": True, "in_fov": True, "line_of_sight_clear": True}]},
            "wrist_deposit_aperture": {"targets": [{"visible": False, "in_fov": False, "line_of_sight_clear": False}]},
        },
    }
    compact = compact_report(report)
    assert compact["report_form"] == "compact_scored_summary"
    assert compact["functional_zone_coverage"]["front_ground_observation"]["pose_results"]["p"] == {
        "target_count": 2,
        "visible_count": 1,
    }
    assert "worst_blocked_rays" not in compact["sensor_results"]["sensor"]["all_pose_results"]["p"]


def test_compact_report_rounds_machine_precision_only() -> None:
    report = {
        "sensor_results": {"sensor": {"required_pose_results": {"p": {}}, "all_pose_results": {"p": {}}, "origin_world_m": [0.12345678901234]}},
        "functional_zone_coverage": {
            "front_ground_observation": {"pose_results": {"p": {"targets": []}}},
            "rear_fisheye_safety_perimeter": {"pose_results": {"p": {"left_targets": [], "right_targets": []}}},
            "wrist_pregrasp_cube": {"targets": []},
            "wrist_deposit_aperture": {"targets": []},
        },
    }
    shifted = {
        **report,
        "sensor_results": {"sensor": {**report["sensor_results"]["sensor"], "origin_world_m": [0.12345678901235]}},
    }
    assert compact_report(report) == compact_report(shifted)


def test_startup_pose_regression_detects_legacy_zero_arm_occlusion() -> None:
    report = validate(ROOT / "reports/engineering/formal_competition_vehicle.urdf", LAYOUT)
    gate = report["startup_arm_pose_visibility_regression"]
    assert gate["passed"]
    assert gate["legacy_zero_pose_clear_fraction"] < 0.95
    assert gate["configured_transport_pose_clear_fraction"] >= 0.95
    assert any("ur5e" in name or "robotiq" in name or "wrist_rgbd" in name for name in gate["legacy_zero_pose_occluders"])

def test_default_urdf_evidence_path_is_repository_relative() -> None:
    assert _portable_evidence_path(ROOT / "reports/engineering/formal_competition_vehicle.urdf") == (
        "reports/engineering/formal_competition_vehicle.urdf"
    )
