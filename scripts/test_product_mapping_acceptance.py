import argparse
import json

from PIL import Image

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
            "loop_ghosting_ratio": 0.01,
            "rigid_alignment": {"optimizer_success": True},
        }),
        "mapping_tf": _write_json(tmp_path / "mapping_tf.json", {
            "continuous": True,
            "coordinate_frame_break_count": 0,
        }),
        "reload_tf": _write_json(tmp_path / "reload_tf.json", {
            "continuous": True,
            "coordinate_frame_break_count": 0,
        }),
        "navigation": _write_json(tmp_path / "navigation.json", {
            "success": True,
            "waypoint_count": 3,
        }),
        "processes": _write_json(tmp_path / "processes.json", {
            "formal_scope": formal_scope,
            "restart_completed": True,
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
