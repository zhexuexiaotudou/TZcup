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
    dynamic_map.expire(4_300_000_000)
    assert target.track_state == TargetState.REJECTED
    assert dynamic_map.count == 0


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
