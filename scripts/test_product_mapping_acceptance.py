import argparse
import json
import math

from PIL import Image
import pytest

from scripts.product_mapping_acceptance import evaluate, select_reload_waypoints


def _write_json(path, payload):
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_reload_route_uses_spatially_separated_successful_frontiers():
    exploration = {
        "goals": [
            {"world_x_m": 20, "world_y_m": 0, "yaw_rad": 0, "succeeded": True},
            {"world_x_m": 21, "world_y_m": 0, "yaw_rad": 0, "succeeded": True},
            {"world_x_m": -20, "world_y_m": 0, "yaw_rad": 3.14, "succeeded": True},
            {"world_x_m": 0, "world_y_m": 20, "yaw_rad": 1.57, "succeeded": True},
            {"world_x_m": 0, "world_y_m": -20, "yaw_rad": -1.57, "succeeded": False},
        ]
    }
    route = select_reload_waypoints(
        exploration, minimum_separation_m=15.0, maximum_waypoints=5
    )
    assert len(route) == 3
    assert [21.0, 0.0, 0.0] in route or [20.0, 0.0, 0.0] in route
    assert [-20.0, 0.0, 3.14] in route
    assert [0.0, 20.0, 1.57] in route


def test_reload_route_respects_physical_travel_budget():
    exploration = {
        "goals": [
            {"world_x_m": x, "world_y_m": y, "yaw_rad": 0.0, "succeeded": True}
            for x, y in (
                (90, -45), (-90, -45), (90, 45), (-90, 45),
                (50, 0), (50, 30), (20, 30), (-20, 30), (-50, 0),
            )
        ]
    }

    route = select_reload_waypoints(
        exploration,
        initial_xy=(0.0, 0.0),
        minimum_separation_m=20.0,
        minimum_waypoints=3,
        maximum_waypoints=5,
        maximum_route_length_m=225.0,
    )

    assert len(route) >= 3
    current = (0.0, 0.0)
    length = 0.0
    for waypoint in route:
        length += ((waypoint[0] - current[0]) ** 2 + (waypoint[1] - current[1]) ** 2) ** 0.5
        current = waypoint[:2]
    assert length <= 225.0
    for index, waypoint in enumerate(route):
        anchors = [[0.0, 0.0], *route[:index]]
        assert min(
            ((waypoint[0] - anchor[0]) ** 2 + (waypoint[1] - anchor[1]) ** 2) ** 0.5
            for anchor in anchors
        ) >= 20.0
    current = [0.0, 0.0]
    for waypoint in route:
        assert waypoint[2] == pytest.approx(
            math.atan2(waypoint[1] - current[1], waypoint[0] - current[0])
        )
        current = waypoint[:2]


def _evaluation_args(tmp_path, *, formal_scope=True):
    image = tmp_path / "map.pgm"
    Image.new("L", (2, 2), color=254).save(image)
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(
        "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    route = tmp_path / "route.json"
    route.write_text("[[1, 0, 0], [2, 0, 0], [3, 0, 0]]\n", encoding="utf-8")
    posegraph = tmp_path / "map.posegraph"
    posegraph_data = tmp_path / "map.data"
    posegraph.write_bytes(b"posegraph")
    posegraph_data.write_bytes(b"data")
    paths = {
        "exploration": _write_json(tmp_path / "exploration.json", {
            "success": True,
            "mapping_area_m2": 20_050.0,
            "ground_truth_used_for_control": False,
        }),
        "map_quality": _write_json(tmp_path / "quality.json", {
            "known_area_m2": 20_020.0,
            "slam_quality_pass": True,
        }),
        "map_geometry": _write_json(tmp_path / "geometry.json", {
            "boundary_rmse_m": 0.10,
            "visible_truth_boundary_recall": 0.96,
            "loop_ghosting_ratio": 0.01,
            "rigid_alignment": {"optimizer_success": True},
        }),
        "mapping_tf": _write_json(tmp_path / "mapping_tf.json", {
            "continuous": True,
            "coordinate_frame_break_count": 0,
            "diagnostic_transform_jump_count": 0,
        }),
        "reload_tf": _write_json(tmp_path / "reload_tf.json", {
            "continuous": True,
            "coordinate_frame_break_count": 0,
            "diagnostic_transform_jump_count": 0,
        }),
        "navigation": _write_json(tmp_path / "navigation.json", {
            "success": True,
            "waypoint_count": 3,
        }),
        "processes": _write_json(tmp_path / "processes.json", {
            "formal_scope": formal_scope,
            "restart_completed": True,
            "runtime_shutdown": {
                "all_started_service_groups_clean": True,
                "missing_shutdown_records": [],
                "started_service_groups": ["mapping_sim"],
                "records": {
                    "mapping_sim": {
                        "clean": True,
                        "wrapper_exit_code": 0,
                        "residual_process_present": False,
                        "signal_stage": "sigint",
                    }
                },
            },
            "sensor_provenance": {
                "positioning": (
                    "gazebo_dual_navsat_rtk_plus_wheel_imu_plus_scan_matching"
                ),
                "gazebo_dual_navsat_sensor_pair": True,
                "gazebo_truth_to_gnss_sensor_model": False,
                "runtime_graph_audits": {
                    phase: {"pass": True} for phase in ("mapping", "reload")
                },
                "all_runtime_graph_audits_pass": True,
                "ground_truth_ros_subscription_in_positioning": False,
                "oracle_pose_topic_to_controller": False,
            },
            "reproducibility": {
                "source_commit": "a" * 40,
                "source_dirty": False,
                "seed": 2028,
                "command": "run_product_mapping_acceptance.sh --seed 2028",
                "ros_distro": "jazzy",
                "config_sha256": {
                    f"config-{index}": "b" * 64 for index in range(5)
                },
            },
            "exit_codes": {
                name: 0 for name in (
                    "exploration", "map_save", "posegraph_serialize",
                    "map_quality", "map_geometry", "route_build", "navigation",
                )
            },
        }),
    }
    return argparse.Namespace(
        **paths,
        map_yaml=map_yaml,
        posegraph=posegraph,
        posegraph_data=posegraph_data,
        reload_route=route,
        output=tmp_path / "final.json",
        max_boundary_rmse_m=0.15,
        min_visible_boundary_recall=0.95,
        max_loop_ghosting_ratio=0.02,
        allow_overwrite=False,
    )


def test_mapping_adjudicator_requires_complete_formal_chain(tmp_path):
    args = _evaluation_args(tmp_path)
    assert evaluate(args) == 0
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["success"] is True
    assert report["mapping_area_m2"] == 20_020.0
    assert report["SIMULATION_PRODUCT_COMPLETE"] is False
    assert all(row["sha256"] for row in report["artifacts"].values())


def test_mapping_adjudicator_never_promotes_smoke_scope(tmp_path):
    args = _evaluation_args(tmp_path, formal_scope=False)
    assert evaluate(args) == 0
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["success"] is False
    assert report["smoke_chain_pass"] is True
    assert report["checks"]["formal_scope"] is False
    assert "formal_scope" not in report["smoke_chain_checks"]
    assert "mapping_area_at_least_20000m2" not in report["smoke_chain_checks"]
    assert "topology_damage_count_zero" not in report["smoke_chain_checks"]


def test_mapping_adjudicator_rejects_missing_visible_obstacle_surfaces(tmp_path):
    args = _evaluation_args(tmp_path)
    geometry = json.loads(args.map_geometry.read_text(encoding="utf-8"))
    geometry["visible_truth_boundary_recall"] = 0.949
    _write_json(args.map_geometry, geometry)

    assert evaluate(args) == 2
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["topology_failures"][
        "visible_boundary_recall_below_minimum"
    ] is True
    assert report["checks"]["topology_damage_count_zero"] is False


def test_mapping_adjudicator_writes_fail_closed_report_for_missing_phase2(tmp_path):
    args = _evaluation_args(tmp_path)
    args.reload_tf.unlink()
    args.navigation.unlink()
    args.reload_route.unlink()
    assert evaluate(args) == 2
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["success"] is False
    assert report["checks"]["artifacts_complete"] is False
    assert report["checks"]["reload_relocalize_navigation_pass"] is False
    assert str(args.reload_tf) in report["input_errors"]
    assert str(args.navigation) in report["input_errors"]


def test_mapping_adjudicator_rejects_self_declared_or_incomplete_gt_isolation(tmp_path):
    args = _evaluation_args(tmp_path)
    processes = json.loads(args.processes.read_text(encoding="utf-8"))
    processes["sensor_provenance"]["runtime_graph_audits"].pop("reload")
    processes["sensor_provenance"]["all_runtime_graph_audits_pass"] = True
    _write_json(args.processes, processes)
    assert evaluate(args) == 2
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["checks"]["ground_truth_not_used_for_control"] is False

    args.output.unlink()
    processes["sensor_provenance"]["runtime_graph_audits"]["reload"] = {
        "pass": False,
        "ground_truth_subscription_present": True,
    }
    processes["sensor_provenance"][
        "ground_truth_ros_subscription_in_positioning"
    ] = True
    _write_json(args.processes, processes)
    assert evaluate(args) == 2
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["checks"]["ground_truth_not_used_for_control"] is False


def test_mapping_adjudicator_rejects_unclean_runtime_shutdown(tmp_path):
    args = _evaluation_args(tmp_path)
    processes = json.loads(args.processes.read_text(encoding="utf-8"))
    processes["runtime_shutdown"] = {
        "all_started_service_groups_clean": False,
        "missing_shutdown_records": ["mapping_sim"],
        "records": {},
    }
    _write_json(args.processes, processes)
    assert evaluate(args) == 2
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["checks"]["runtime_shutdown_clean"] is False


def test_mapping_adjudicator_rejects_transform_owner_jumps(tmp_path):
    args = _evaluation_args(tmp_path)
    reload_tf = json.loads(args.reload_tf.read_text(encoding="utf-8"))
    reload_tf["diagnostic_transform_jump_count"] = 1
    _write_json(args.reload_tf, reload_tf)

    assert evaluate(args) == 2
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["transform_jump_count"] == 1
    assert report["checks"]["coordinate_frame_break_count_zero"] is False


def test_mapping_adjudicator_recomputes_shutdown_truth_from_records(tmp_path):
    args = _evaluation_args(tmp_path)
    processes = json.loads(args.processes.read_text(encoding="utf-8"))
    processes["runtime_shutdown"]["records"]["mapping_sim"]["clean"] = False
    processes["runtime_shutdown"]["all_started_service_groups_clean"] = True
    _write_json(args.processes, processes)
    assert evaluate(args) == 2
    report = json.loads(args.output.read_text(encoding="utf-8"))
    assert report["checks"]["runtime_shutdown_clean"] is False
