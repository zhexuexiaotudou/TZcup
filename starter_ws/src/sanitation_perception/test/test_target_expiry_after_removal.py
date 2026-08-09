from sanitation_perception.dynamic_trash_map import DynamicTrashMap, DynamicTrashMapConfig
from sanitation_perception.trash_map_messages import TargetState

from online_map_test_support import observation, record_sweep


def test_target_expires_after_removal_without_navigation_action():
    dynamic_map = DynamicTrashMap.start_new(
        "mission-removal",
        config=DynamicTrashMapConfig(lost_after_s=0.5, reject_after_s=1.0),
    )
    stamp = 1_000_000_000
    record_sweep(dynamic_map, stamp)
    target = dynamic_map.ingest(observation(dynamic_map, stamp))
    dynamic_map.expire(1_600_000_000)
    assert target.track_state == TargetState.LOST
    dynamic_map.expire(2_100_000_000)
    assert target.track_state == TargetState.REJECTED
    assert dynamic_map.count == 0
