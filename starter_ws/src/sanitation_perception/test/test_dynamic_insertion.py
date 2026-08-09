from sanitation_perception.dynamic_trash_map import DynamicTrashMap

from online_map_test_support import observation, record_sweep


def test_late_inserted_target_does_not_exist_before_first_camera_observation():
    dynamic_map = DynamicTrashMap.start_new("mission-late-insertion")
    for stamp in (0, 10_000_000_000, 29_000_000_000):
        record_sweep(dynamic_map, stamp)
        assert dynamic_map.count == 0

    inserted_at = 30_000_000_000
    record_sweep(dynamic_map, inserted_at)
    target = dynamic_map.ingest(observation(dynamic_map, inserted_at))
    assert target is not None
    assert target.first_seen_stamp_ns == inserted_at
    assert dynamic_map.count == 1
