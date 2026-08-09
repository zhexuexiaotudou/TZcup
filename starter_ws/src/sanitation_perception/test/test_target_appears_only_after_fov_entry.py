from sanitation_perception.dynamic_trash_map import DynamicTrashMap

from online_map_test_support import observation, record_sweep


def test_target_appears_only_after_fov_entry():
    dynamic_map = DynamicTrashMap.start_new("mission-fov-entry")
    before = observation(dynamic_map, 1_000_000_000)
    assert dynamic_map.ingest(before) is None
    assert dynamic_map.count == 0
    record_sweep(dynamic_map, 2_000_000_000)
    assert dynamic_map.ingest(observation(dynamic_map, 2_000_000_000)) is not None
    assert dynamic_map.count == 1
