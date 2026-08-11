from dataclasses import replace

from sanitation_perception.dynamic_trash_map import DynamicTrashMap, DynamicTrashMapConfig
from sanitation_perception.trash_map_messages import TargetState

from online_map_test_support import observation, record_sweep


def test_multiframe_confirmation_and_removal_expiry():
    dynamic_map = DynamicTrashMap.start_new(
        "mission-a",
        config=DynamicTrashMapConfig(lost_after_s=1.0, reject_after_s=3.0),
    )
    assert dynamic_map.count == 0

    target = None
    for stamp in (1_000_000_000, 1_100_000_000, 1_200_000_000):
        record_sweep(dynamic_map, stamp)
        target = dynamic_map.ingest(observation(dynamic_map, stamp))

    assert target is not None
    assert target.track_state == TargetState.CONFIRMED
    assert target.observation_count == 3
    assert dynamic_map.count == 1

    dynamic_map.expire(2_300_000_000)
    assert target.track_state == TargetState.LOST
    record_sweep(dynamic_map, 4_300_000_000)
    dynamic_map.expire(4_300_000_000)
    assert target.track_state == TargetState.REJECTED
    assert dynamic_map.count == 0


def test_lost_target_is_retained_until_its_location_is_reobserved():
    dynamic_map = DynamicTrashMap.start_new(
        "mission-out-of-view",
        config=DynamicTrashMapConfig(lost_after_s=1.0, reject_after_s=3.0),
    )
    stamp = 1_000_000_000
    record_sweep(dynamic_map, stamp)
    target = dynamic_map.ingest(observation(dynamic_map, stamp))
    dynamic_map.expire(2_100_000_000)
    assert target.track_state == TargetState.LOST

    record_sweep(dynamic_map, 5_000_000_000, yaw_rad=3.141592653589793)
    dynamic_map.expire(5_000_000_000)
    assert target.track_state == TargetState.LOST


def test_area_association_allows_centroid_motion_within_region_scale():
    dynamic_map = DynamicTrashMap.start_new("mission-area-association")
    stamp = 1_000_000_000
    record_sweep(dynamic_map, stamp)
    first = dynamic_map.ingest(
        replace(
            observation(dynamic_map, stamp, x_m=2.0),
            target_type="AREA",
            class_probabilities={"puddle": 0.92, "background": 0.08},
            polygon_xy_m=((1.8, -0.2), (2.2, -0.2), (2.0, 0.2)),
        )
    )
    next_stamp = 1_100_000_000
    record_sweep(dynamic_map, next_stamp)
    second = dynamic_map.ingest(
        replace(
            observation(dynamic_map, next_stamp, x_m=2.4),
            target_type="AREA",
            class_probabilities={"puddle": 0.92, "background": 0.08},
            polygon_xy_m=((2.2, -0.2), (2.6, -0.2), (2.4, 0.2)),
        )
    )
    assert second.uuid == first.uuid
    assert dynamic_map.count == 1


def test_ground_truth_ingress_is_rejected():
    dynamic_map = DynamicTrashMap.start_new("mission-gt")
    stamp = 1_000_000_000
    record_sweep(dynamic_map, stamp)
    candidate = observation(dynamic_map, stamp, source_backend="ground_truth")
    try:
        dynamic_map.ingest(candidate)
    except ValueError as exc:
        assert "ground-truth" in str(exc)
    else:
        raise AssertionError("ground-truth observation entered the product map")
    assert dynamic_map.count == 0


def test_area_target_requires_the_longer_temporal_confirmation_window():
    dynamic_map = DynamicTrashMap.start_new("mission-area")
    target = None
    for index in range(5):
        stamp = 1_000_000_000 + index * 100_000_000
        record_sweep(dynamic_map, stamp)
        target = dynamic_map.ingest(
            replace(
                observation(dynamic_map, stamp),
                target_type="AREA",
                polygon_xy_m=((1.9, -0.1), (2.1, -0.1), (2.0, 0.1)),
            )
        )
    assert target is not None
    assert target.track_state == TargetState.TRACKED

    stamp = 1_500_000_000
    record_sweep(dynamic_map, stamp)
    target = dynamic_map.ingest(
        replace(
            observation(dynamic_map, stamp),
            target_type="AREA",
            polygon_xy_m=((1.9, -0.1), (2.1, -0.1), (2.0, 0.1)),
        )
    )
    assert target.track_state == TargetState.CONFIRMED


def test_leaf_area_uses_the_class_aware_four_frame_window():
    dynamic_map = DynamicTrashMap.start_new("mission-leaf")
    target = None
    for index in range(4):
        stamp = 2_000_000_000 + index * 100_000_000
        record_sweep(dynamic_map, stamp)
        target = dynamic_map.ingest(
            replace(
                observation(dynamic_map, stamp),
                target_type="AREA",
                class_probabilities={"leaf_pile": 0.92, "background": 0.08},
                polygon_xy_m=((1.9, -0.1), (2.1, -0.1), (2.0, 0.1)),
            )
        )
    assert target is not None
    assert target.current_class == "leaf_pile"
    assert target.track_state == TargetState.CONFIRMED
