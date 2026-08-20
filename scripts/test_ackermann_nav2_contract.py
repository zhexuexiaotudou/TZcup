#!/usr/bin/env python3
"""Fast contract tests for the Ackermann Nav2 profile."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_coverage"))

import pytest
import xml.etree.ElementTree as ET
import yaml

from ackermann_profile_validation import resolve_profiles
from sanitation_coverage import ackermann_model
from sanitation_coverage.ackermann_connector import plan_forward_dubins_path
from sanitation_coverage.metrics import split_path_at_curvature_reversals


NAV2_ACKERMANN = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_navigation"
    / "config"
    / "nav2_ackermann.yaml"
)


def _load() -> dict:
    return yaml.safe_load(NAV2_ACKERMANN.read_text(encoding="utf-8"))


@pytest.mark.parametrize(
    ("start_pose", "goal_pose"),
    [
        ((-81.0, 8.0, math.pi), (-81.0, 14.0, math.pi / 2.0)),
        ((81.0, 28.0, 0.0), (81.0, 34.0, math.pi / 2.0)),
        ((-81.0, 38.0, math.pi), (-81.0, 32.0, -math.pi / 2.0)),
        ((81.0, -8.0, 0.0), (81.0, -14.0, -math.pi / 2.0)),
    ],
    ids=("west-north", "east-north", "west-south", "east-south"),
)
def test_formal_boundary_lane_shift_has_forward_dubins_solution(
    start_pose, goal_pose
):
    apron = [(-98.5, -48.5), (98.5, -48.5), (98.5, 48.5), (-98.5, 48.5)]
    path = plan_forward_dubins_path(start_pose, goal_pose, apron, [])
    assert path is not None
    assert len(path) >= 100
    assert path[0] == start_pose
    assert path[-1] == goal_pose
    primitives = split_path_at_curvature_reversals(
        [(pose[0], pose[1]) for pose in path],
        [pose[2] for pose in path],
    )
    assert [len(points) for points, _ in primitives] == [56, 50, 11]
    assert min(pose[0] for pose in path) >= -98.5
    assert max(pose[0] for pose in path) <= 98.5
    assert min(pose[1] for pose in path) >= -48.5
    assert max(pose[1] for pose in path) <= 48.5
    for start, end in zip(path, path[1:]):
        projection = (
            (end[0] - start[0]) * math.cos(start[2])
            + (end[1] - start[1]) * math.sin(start[2])
        )
        assert projection >= -1.0e-6


def test_hybrid_reeds_shepp_planner_contract():
    nav2 = _load()
    planners = nav2["planner_server"]["ros__parameters"]
    assert planners["planner_plugins"] == ["GridBased", "GridBasedForward"]
    planner = planners["GridBased"]
    assert planner["plugin"] == "nav2_smac_planner::SmacPlannerHybrid"
    assert planner["motion_model_for_search"] == "REEDS_SHEPP"
    assert int(planner["angle_quantization_bins"]) == 72
    assert planner["allow_unknown"] is False
    assert float(planner["minimum_turning_radius"]) == pytest.approx(
        ackermann_model.minimum_radius_m(), abs=1e-3
    )
    forward = planners["GridBasedForward"]
    assert forward["plugin"] == "nav2_smac_planner::SmacPlannerHybrid"
    assert forward["motion_model_for_search"] == "DUBIN"
    assert float(forward["minimum_turning_radius"]) == pytest.approx(
        ackermann_model.minimum_radius_m(), abs=1e-3
    )


def test_coverage_launch_maps_ackermann_profile_to_physical_width():
    launch = (
        ROOT
        / "starter_ws/src/sanitation_coverage/launch/coverage.launch.py"
    ).read_text(encoding="utf-8")
    assert "'autonomous_navigation_profile_v1'" in launch
    assert '"1.32 if \'"' in launch


def test_rpp_controllers_have_explicit_direction_and_no_rotate():
    nav2 = _load()
    controllers = nav2["controller_server"]["ros__parameters"]
    assert controllers["goal_checker_plugins"] == [
        "goal_checker",
        "frontier_goal_checker",
        "cusp_goal_checker",
        "primitive_goal_checker",
        "connector_goal_checker", "swath_exit_goal_checker",
    ]
    assert float(
        controllers["primitive_goal_checker"]["yaw_goal_tolerance"]
    ) == 1.0
    for name in (
        "FollowPath", "FrontierPath", "DubinsPath", "ReversePath",
        "CleanPath", "RepairPath",
    ):
        controller = controllers[name]
        assert (
            controller["plugin"]
            == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        )
        assert controller["use_rotate_to_heading"] is False
        assert controller["use_collision_detection"] is True
        assert controller["use_regulated_linear_velocity_scaling"] is True
        assert controller["use_cost_regulated_linear_velocity_scaling"] is True
        assert float(controller["regulated_linear_scaling_min_radius"]) == pytest.approx(
            ackermann_model.minimum_radius_m(), abs=1e-3
        )
    assert float(controllers["FollowPath"]["desired_linear_vel"]) == 0.45
    assert float(controllers["FollowPath"]["lookahead_dist"]) <= 0.25
    assert controllers["FollowPath"]["use_velocity_scaled_lookahead_dist"] is False
    assert float(controllers["FollowPath"]["max_robot_pose_search_dist"]) <= 1.0
    assert float(controllers["FollowPath"]["regulated_linear_scaling_min_speed"]) == 0.20
    frontier = controllers["FrontierPath"]
    assert float(frontier["lookahead_dist"]) == pytest.approx(1.0)
    assert float(frontier["min_lookahead_dist"]) == pytest.approx(1.0)
    assert float(frontier["max_lookahead_dist"]) == pytest.approx(1.0)
    assert frontier["use_velocity_scaled_lookahead_dist"] is False
    assert frontier["allow_reversing"] is True
    assert float(controllers["DubinsPath"]["max_robot_pose_search_dist"]) == 2.0
    connector = controllers["ConnectorPath"]
    assert connector["plugin"] == "nav2_mppi_controller::MPPIController"
    assert connector["motion_model"] == "Ackermann"
    assert float(connector["AckermannConstraints"]["min_turning_r"]) == 1.429
    assert float(connector["vx_max"]) == 0.25
    assert float(connector["vx_min"]) == 0.0
    assert float(connector["wz_max"]) == 0.25
    assert float(connector["model_dt"]) == pytest.approx(1.0 / 15.0)
    assert connector["PathAlignCritic"]["use_path_orientations"] is True
    assert int(connector["PathAngleCritic"]["mode"]) == 2
    assert "PreferForwardCritic" in connector["critics"]
    assert connector["PreferForwardCritic"]["enabled"] is True
    assert float(controllers["DubinsPath"]["desired_linear_vel"]) == 0.25
    assert float(controllers["DubinsPath"]["regulated_linear_scaling_min_speed"]) == 0.10
    assert controllers["FollowPath"]["allow_reversing"] is True
    assert controllers["ReversePath"]["allow_reversing"] is True
    for name in ("DubinsPath", "CleanPath", "RepairPath"):
        assert controllers[name]["allow_reversing"] is False
    assert float(controllers["ReversePath"]["desired_linear_vel"]) == 0.20
    assert float(controllers["ReversePath"]["lookahead_dist"]) == 0.50
    assert float(controllers["ReversePath"]["max_robot_pose_search_dist"]) <= 1.0
    clean_speed = float(controllers["CleanPath"]["desired_linear_vel"])
    assert clean_speed == 1.00
    assert float(controllers["CleanPath"]["lookahead_dist"]) == 1.20
    assert controllers["CleanPath"]["use_velocity_scaled_lookahead_dist"] is False
    assert float(controllers["CleanPath"]["min_lookahead_dist"]) == 1.20
    assert float(controllers["CleanPath"]["max_lookahead_dist"]) == 1.20
    assert ackermann_model.OPERATION_WIDTH_M * clean_speed * 3600.0 > 3500.0


def test_no_spin_behavior_plugins_and_no_spin_bt():
    nav2 = _load()
    behavior = nav2["behavior_server"]["ros__parameters"]
    plugins = behavior["behavior_plugins"]
    assert "spin" not in plugins
    assert set(plugins) == {"backup", "drive_on_heading", "wait"}
    bt_navigator = nav2["bt_navigator"]["ros__parameters"]
    for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
        tree = bt_navigator[key]
        assert tree.startswith("__ACKERMANN_NAV_")
        tree_path = (
            ROOT / "starter_ws" / "src" / "sanitation_navigation"
            / "behavior_trees"
            / (
                "navigate_to_pose_ackermann.xml"
                if key == "default_nav_to_pose_bt_xml"
                else "navigate_through_poses_ackermann.xml"
            )
        )
        assert tree_path.is_file()
        tree_text = tree_path.read_text(encoding="utf-8").lower()
        tree_root = ET.fromstring(tree_text)
        assert not [node for node in tree_root.iter() if node.tag.lower() == "spin"]
        assert 'default_planner="gridbased"' in tree_text
        assert "backup" in tree_text
        assert "clearentirecostmap" in tree_text
        if key == "default_nav_to_pose_bt_xml":
            assert "pipelinesequence" in tree_text
            assert "ratecontroller" in tree_text

    launch_text = (
        ROOT / "starter_ws/src/sanitation_navigation/launch/navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert "ReplaceString" in launch_text
    assert "__ACKERMANN_NAV_TO_POSE_BT__" in launch_text
    assert "__ACKERMANN_NAV_THROUGH_POSES_BT__" in launch_text


def test_frontier_bt_uses_intermediate_ackermann_goal_checker_only():
    tree_dir = (
        ROOT / "starter_ws" / "src" / "sanitation_navigation" / "behavior_trees"
    )
    frontier = (tree_dir / "navigate_to_pose_ackermann_frontier.xml").read_text(
        encoding="utf-8"
    )
    strict = (tree_dir / "navigate_to_pose_ackermann.xml").read_text(
        encoding="utf-8"
    )
    assert 'goal_checker_id="frontier_goal_checker"' in frontier
    assert 'controller_id="FrontierPath"' in frontier
    assert "ControllerSelector" not in frontier
    assert 'goal_checker_id="goal_checker"' in strict
    assert "PipelineSequence" in frontier
    assert '<RateController hz="1.0">' in frontier
    assert "SequenceWithMemory" not in frontier
    assert "<Spin" not in frontier


def test_honest_footprint_in_costmaps():
    nav2 = _load()
    expected = (
        "[[0.82, 0.66], [0.82, -0.66], "
        "[-0.575, -0.66], [-0.575, 0.66]]"
    )
    for name in ("local_costmap", "global_costmap"):
        costmap = nav2[name][name]["ros__parameters"]
        footprint = costmap["footprint"]
        assert footprint == expected
        assert float(costmap["inflation_layer"]["inflation_radius"]) >= 1.20
    assert ackermann_model.honest_footprint_polygon() == [
        [pytest.approx(0.82), 0.66],
        [pytest.approx(0.82), -0.66],
        [-0.575, -0.66],
        [-0.575, 0.66],
    ]


def test_collision_monitor_uses_vehicle_local_static_footprint():
    nav2 = _load()
    monitor = nav2["collision_monitor"]["ros__parameters"]
    footprint = monitor["FootprintApproach"]
    assert "footprint_topic" not in footprint
    assert footprint["points"] == (
        "[[0.82, 0.66], [0.82, -0.66], "
        "[-0.575, -0.66], [-0.575, 0.66]]"
    )
    assert monitor["cmd_vel_out_topic"] == "/cmd_vel_gate"


def test_velocity_smoother_forbids_in_place_yaw_request():
    nav2 = _load()
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"] == [1.00, 0.0, 0.70]
    assert smoother["min_velocity"] == [-0.30, 0.0, -0.70]
    assert smoother["max_accel"] == [1.00, 0.0, 1.20]


def test_nonholonomic_goal_tolerance_is_bounded_but_not_point_turn_strict():
    controller = _load()["controller_server"]["ros__parameters"]
    goal = controller["goal_checker"]
    assert goal["stateful"] is False
    assert 0.20 <= float(goal["xy_goal_tolerance"]) <= 0.25
    assert 0.25 <= float(goal["yaw_goal_tolerance"]) <= 0.30
    cusp = controller["cusp_goal_checker"]
    assert cusp["stateful"] is False
    assert controller["goal_checker_plugins"] == [
        "goal_checker", "frontier_goal_checker", "cusp_goal_checker", "primitive_goal_checker",
        "connector_goal_checker", "swath_exit_goal_checker",
    ]
    frontier = controller["frontier_goal_checker"]
    assert float(frontier["xy_goal_tolerance"]) == 0.25
    assert float(frontier["yaw_goal_tolerance"]) == pytest.approx(math.pi)
    primitive = controller["primitive_goal_checker"]
    assert primitive["stateful"] is False
    assert float(primitive["xy_goal_tolerance"]) <= 0.40
    assert float(primitive["yaw_goal_tolerance"]) <= 1.00
    connector = controller["connector_goal_checker"]
    assert connector["stateful"] is False
    assert float(connector["xy_goal_tolerance"]) <= 0.40
    assert float(connector["yaw_goal_tolerance"]) <= 0.50
    swath_exit = controller["swath_exit_goal_checker"]
    assert float(swath_exit["xy_goal_tolerance"]) <= 0.30
    assert float(swath_exit["yaw_goal_tolerance"]) <= 1.00
    assert float(cusp["xy_goal_tolerance"]) == pytest.approx(0.25)
    assert float(cusp["yaw_goal_tolerance"]) == pytest.approx(0.35)


def test_profile_compatibility_fails_closed():
    with pytest.raises(ValueError):
        resolve_profiles("ackermann", "optimized")
    with pytest.raises(ValueError):
        resolve_profiles("ackermann", "legacy")
    with pytest.raises(ValueError):
        resolve_profiles("skid_steer_legacy", "ackermann")
    with pytest.raises(ValueError):
        resolve_profiles("hovercraft", "legacy")
    ack = resolve_profiles("ackermann", "ackermann")
    assert ack.nav2_params == "nav2_ackermann.yaml"
    assert ack.ekf_config == "ekf_ackermann.yaml"
    assert ack.wheel_odom_input == "/wheel/odom_raw"
    legacy = resolve_profiles("skid_steer_legacy", "optimized")
    assert legacy.wheel_odom_input == "/odom/unfiltered"


def test_visual_launcher_preserves_ackermann_controller_limits():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(
        encoding="utf-8"
    )
    assert '"${MAP_SIZE}" "${DRIVE_MODEL}"' in launcher
    assert 'if drive_model != "ackermann":' in launcher
    assert 'assert follow["use_rotate_to_heading"] is False' in launcher
    assert 'assert follow["allow_reversing"] is False' in launcher
    assert '["ReversePath"]["allow_reversing"] is True' in launcher
    assert 'map_file="${map_root}/sanitation_test_map.yaml"' in launcher


def test_coverage_goal_cold_start_retry_is_bounded():
    probe = (
        ROOT
        / "starter_ws/src/sanitation_coverage/sanitation_coverage/coverage_probe.py"
    ).read_text(encoding="utf-8")
    assert "coverage_goal_attempts = 3" in probe
    assert '"attempts": coverage_goal_attempts' in probe


def test_ackermann_connector_timeout_matches_tight_curve_floor():
    probe = (
        ROOT
        / "starter_ws/src/sanitation_coverage/sanitation_coverage/coverage_probe.py"
    ).read_text(encoding="utf-8")
    assert 'self.speed_limits_mps["REVERSE"]' in probe
    assert 'self.speed_limits_mps["FORWARD"]' in probe
    assert 'self.speed_limits_mps["CLEAN"]' in probe
    assert 'is_ackermann_connector = (' in probe
    assert 'self.ackermann_profile_active and component.get("kind") == "FORWARD"' in probe
    assert 'or is_ackermann_connector' in probe
    assert "path_poses[0] = (" not in probe


def test_follow_path_operational_windows_ignore_truncated_feedback_distance():
    probe = (
        ROOT
        / "starter_ws/src/sanitation_coverage/sanitation_coverage/coverage_probe.py"
    ).read_text(encoding="utf-8")
    assert "if travelled is not None and not action_path_points:" in probe
    assert "projected_path_progress(" in probe


def test_ackermann_swath_leadin_and_repair_entry_are_kinematic():
    mission = yaml.safe_load((
        ROOT
        / "starter_ws/src/sanitation_tasks/config/competition_ackermann_demo_area.yaml"
    ).read_text(encoding="utf-8"))
    assert float(mission["swath_endpoint_extension_m"]) == 2.2
    xs = [float(point[0]) for point in mission["outer_polygon"]]
    ys = [float(point[1]) for point in mission["outer_polygon"]]
    assert max(xs) - min(xs) == pytest.approx(15.6)
    assert max(ys) - min(ys) == pytest.approx(10.0)
    coverage = yaml.safe_load((
        ROOT
        / "starter_ws/src/sanitation_coverage/config/coverage_ackermann.yaml"
    ).read_text(encoding="utf-8"))
    assert float(
        coverage["coverage_server"]["ros__parameters"]["operation_width"]
    ) == pytest.approx(float(mission["planning_swath_spacing_m"]))
    assert float(mission["operation_width_m"]) == pytest.approx(
        ackermann_model.OPERATION_WIDTH_M
    )
    nominal_overlap = 1.0 - (
        float(mission["planning_swath_spacing_m"])
        / float(mission["operation_width_m"])
    )
    assert nominal_overlap <= 0.20
    assert float(mission["empirical_repeat_rate_threshold"]) <= 0.20
    probe = (
        ROOT
        / "starter_ws/src/sanitation_coverage/sanitation_coverage/coverage_probe.py"
    ).read_text(encoding="utf-8")
    assert "if self.ackermann_profile_active:" in probe
    assert 'entry_result = self._navigate_to({' in probe


def test_ackermann_transit_reuses_reeds_shepp_costmap_preflight_before_dubins():
    probe = (
        ROOT
        / "starter_ws/src/sanitation_coverage/sanitation_coverage/coverage_probe.py"
    ).read_text(encoding="utf-8")
    assert 'planner_id="GridBased",' in probe
    preflight_assignment = probe.index("plan = precomputed_plan")
    geometric_fallback = probe.index("forward_poses = plan_forward_dubins_path(")
    assert preflight_assignment < geometric_fallback
    assert '"planner_id": planner_id' in probe


def test_legacy_nav2_profile_cannot_satisfy_ackermann_contract():
    base = (
        ROOT / "starter_ws" / "src" / "sanitation_navigation" / "config" / "nav2.yaml"
    )
    nav2 = yaml.safe_load(base.read_text(encoding="utf-8"))
    planner = nav2["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_navfn_planner::NavfnPlanner"
    behaviors = nav2["behavior_server"]["ros__parameters"]["behavior_plugins"]
    assert "spin" in behaviors
    follow = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["use_rotate_to_heading"] is True
    assert follow["allow_reversing"] is False


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
