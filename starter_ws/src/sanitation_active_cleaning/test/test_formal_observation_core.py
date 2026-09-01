from pathlib import Path

import yaml

from sanitation_active_cleaning.formal_observation_core import (
    FormalObservationBridgeCore,
    ProductTargetObservation,
    PublicPlanningMap,
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
