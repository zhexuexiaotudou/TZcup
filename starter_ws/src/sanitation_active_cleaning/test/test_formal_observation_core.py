from pathlib import Path

import yaml
import pytest

from sanitation_active_cleaning.formal_observation_core import (
    FormalObservationBridgeCore,
    ProductTargetObservation,
    PublicPlanningMap,
    FormalObservationError,
    observation_stamp_reason,
)


def _public_map(tmp_path: Path) -> PublicPlanningMap:
    # PGM is top-down: top-left occupied, all other cells free.
    (tmp_path / "occupancy.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([0, 255, 255, 255]))
    (tmp_path / "occupancy.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "occupancy.pgm",
                "resolution": 1.0,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "mission_geometry.yaml").write_text(
        yaml.safe_dump(
            {
                "frame_id": "map",
                "outer_polygon": [[0, 0], [2, 0], [2, 2], [0, 2]],
                "keepout_polygons": [],
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "materialization_contract.yaml").write_text(
        yaml.safe_dump(
            {"evaluator_truth_used": False, "dirt_truth_used": False}
        ),
        encoding="utf-8",
    )
    return PublicPlanningMap.load(
        tmp_path / "occupancy.yaml",
        tmp_path / "mission_geometry.yaml",
        tmp_path / "materialization_contract.yaml",
    )


def test_public_map_and_projected_mask_create_belief_without_hidden_inputs(tmp_path):
    planning_map = _public_map(tmp_path)
    assert planning_map.traversable == (True, True, False, True)
    core = FormalObservationBridgeCore(planning_map, min_target_confidence=0.5)
    update = core.update_projected_mask(
        frame_id="map",
        width=2,
        height=2,
        encoding="mono8",
        step=2,
        data=bytes([1, 255, 255, 1]),
    )
    assert update.accepted is True
    assert update.observed_cells == 3
    assert update.dirty_cells == 1
    assert core.occupancy_grid_values() == (0, 100, -1, 0)
    belief = core.belief_snapshot()
    assert belief.observed == (True, True, False, True)
    assert belief.known_ground_dirt[1] == 1.0


def test_mask_contract_rejects_camera_frame_dimensions_and_noncontiguous_rows(tmp_path):
    core = FormalObservationBridgeCore(_public_map(tmp_path), min_target_confidence=0.5)
    kwargs = {
        "frame_id": "map",
        "width": 2,
        "height": 2,
        "encoding": "mono8",
        "step": 2,
        "data": bytes([1, 1, 1, 1]),
    }
    assert not core.update_projected_mask(**{**kwargs, "frame_id": "camera"}).accepted
    assert not core.update_projected_mask(**{**kwargs, "width": 1}).accepted
    assert not core.update_projected_mask(**{**kwargs, "step": 3}).accepted


def test_target_filter_accepts_only_finite_product_targets_in_public_free_map(tmp_path):
    core = FormalObservationBridgeCore(_public_map(tmp_path), min_target_confidence=0.5)
    common = {
        "confidence": 0.9,
        "source_backend": "dosod_edgesam",
        "track_state": "CONFIRMED",
        "in_keepout": False,
    }
    accepted = core.replace_targets(
        [
            ProductTargetObservation("good", 0.5, 0.5, **common),
            ProductTargetObservation(
                "blocked_source", 1.5, 0.5, **{**common, "source_backend": "ground_truth"}
            ),
            ProductTargetObservation(
                "low_confidence", 1.5, 0.5, **{**common, "confidence": 0.2}
            ),
            ProductTargetObservation("occupied", 0.5, 1.5, **common),
            ProductTargetObservation(
                "keepout_flag", 1.5, 0.5, **{**common, "in_keepout": True}
            ),
        ]
    )
    assert [target.target_id for target in accepted] == ["good"]
    assert [target.target_id for target in core.belief_snapshot().known_targets] == [
        "good"
    ]


def test_tentative_or_lost_targets_never_enter_the_planner_queue(tmp_path):
    core = FormalObservationBridgeCore(_public_map(tmp_path), min_target_confidence=0.5)
    common = {
        "confidence": 0.99,
        "source_backend": "dosod_edgesam_pc",
        "in_keepout": False,
    }
    accepted = core.replace_targets(
        [
            ProductTargetObservation(
                "tentative", 0.5, 0.5, track_state="TENTATIVE", **common
            ),
            ProductTargetObservation(
                "lost", 0.5, 0.5, track_state="LOST", **common
            ),
        ]
    )
    assert accepted == ()
    assert core.belief_snapshot().known_targets == ()


def test_duplicate_uuid_cannot_substitute_a_rejected_target(tmp_path):
    core = FormalObservationBridgeCore(_public_map(tmp_path), min_target_confidence=0.5)
    good = ProductTargetObservation("same", 0.5, 0.5, 0.9, "dosod_edgesam",
                                    "CONFIRMED", False)
    bad = ProductTargetObservation("same", 1.5, 0.5, 0.9, "ground_truth",
                                   "CONFIRMED", False)
    assert core.replace_targets([good, bad]) == ()
    assert core.replace_targets([bad, good]) == ()
    assert core.belief_snapshot().known_targets == ()


def test_cleaned_product_track_is_not_queued_for_cleaning_again(tmp_path):
    core = FormalObservationBridgeCore(_public_map(tmp_path), min_target_confidence=0.5)
    core.replace_targets([ProductTargetObservation(
        "done", 0.5, 0.5, 0.9, "dosod_edgesam", "CLEANED", False)])
    assert core.belief_snapshot().known_targets[0].cleared


@pytest.mark.parametrize("stamp,now,previous,reason", [
    (0, 10_000_000_000, None, "invalid_source_stamp"),
    (11_000_000_000, 10_000_000_000, None, "future_source_stamp"),
    (8_000_000_000, 10_000_000_000, None, "stale_source_stamp"),
    (9_000_000_000, 10_000_000_000, 9_000_000_000, "replayed_source_stamp"),
    (9_000_000_000, 10_000_000_000, 9_100_000_000, "replayed_source_stamp"),
    (9_000_000_000, 10_000_000_000, 8_900_000_000, "accepted"),
])
def test_source_time_not_receive_time_controls_readiness(stamp, now, previous, reason):
    assert observation_stamp_reason(stamp, now, 1.5, previous) == reason


def test_unknown_occupancy_is_not_traversable(tmp_path):
    _public_map(tmp_path)
    (tmp_path / "occupancy.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([0, 128, 255, 255]))
    result = PublicPlanningMap.load(tmp_path / "occupancy.yaml",
                                    tmp_path / "mission_geometry.yaml",
                                    tmp_path / "materialization_contract.yaml")
    assert result.traversable == (True, True, False, False)


def test_rotated_map_cannot_silently_change_target_coordinates(tmp_path):
    _public_map(tmp_path)
    path = tmp_path / "occupancy.yaml"
    metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    metadata["origin"][2] = 0.5
    path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
    with pytest.raises(FormalObservationError, match="rotated"):
        PublicPlanningMap.load(path, tmp_path / "mission_geometry.yaml",
                               tmp_path / "materialization_contract.yaml")


def test_callback_wiring_rejects_stale_frame_and_duplicate_targets(tmp_path):
    # Exercise the actual callback bodies with bounded message doubles. This
    # verifies wiring without claiming ROS message/schema or middleware coverage.
    import ast
    import time
    from types import SimpleNamespace as NS
    import sanitation_active_cleaning.formal_observation_bridge as bridge

    module = ast.parse(Path(bridge.__file__).read_text(encoding="utf-8"))
    node_class = next(node for node in ast.walk(module)
                      if isinstance(node, ast.ClassDef)
                      and node.name == "FormalObservationBridge")
    namespace = dict(Node=object, Image=object, GarbageTargetArray=NS,
                     ProductTargetObservation=ProductTargetObservation,
                     observation_stamp_reason=observation_stamp_reason, time=time)
    exec(compile(ast.Module(body=[node_class], type_ignores=[]),
                 bridge.__file__, "exec"), namespace)
    node = namespace["FormalObservationBridge"].__new__(namespace["FormalObservationBridge"])
    node._core = FormalObservationBridgeCore(_public_map(tmp_path), min_target_confidence=0.5)
    node._max_age = 1.5
    node._last_mask_stamp = node._last_targets_stamp = None
    node._last_mask_time = node._last_targets_time = 123.0
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    node._publish_status = lambda: None
    published = []
    node._targets_publisher = NS(publish=published.append)
    header = lambda sec, frame="map": NS(frame_id=frame, stamp=NS(sec=sec, nanosec=0))
    node._on_mask(NS(header=header(1)))
    assert node._last_mask_time is None
    assert node._last_mask_reason == "stale_source_stamp"
    node._on_targets(NS(header=header(10, "camera"), targets=[]))
    assert node._last_targets_time is None
    assert node._last_targets_reason == "frame_mismatch"

    def target(backend):
        return NS(header=header(10), uuid="duplicate", confidence=0.9,
                  source_backend=backend, track_state="CONFIRMED", in_keepout=False,
                  source_stamp=header(10).stamp, last_seen=header(10).stamp,
                  map_pose=NS(pose=NS(position=NS(x=0.5, y=0.5))))

    node._on_targets(NS(header=header(10), registry_sha256="digest",
                        targets=[target("dosod_edgesam"), target("ground_truth")]))
    assert len(published) == 1
    assert published[0].targets == []
    assert node._core.belief_snapshot().known_targets == ()
    node._on_targets(NS(header=header(10), targets=[]))
    assert node._last_targets_time is None
    assert node._last_targets_reason == "replayed_source_stamp"
