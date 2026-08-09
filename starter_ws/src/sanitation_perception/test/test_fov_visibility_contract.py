import math

from sanitation_perception.dynamic_trash_map import DynamicTrashMap

from online_map_test_support import observation, record_sweep


def test_target_is_created_only_with_matching_current_fov_proof():
    dynamic_map = DynamicTrashMap.start_new("mission-fov")
    stamp = 1_000_000_000

    pre_fov = observation(dynamic_map, stamp, x_m=2.0, y_m=0.0)
    assert dynamic_map.ingest(pre_fov) is None
    assert dynamic_map.count == 0
    assert dynamic_map.rejected_observations[-1]["reason"] == "no_matching_current_fov_proof"

    record_sweep(dynamic_map, stamp, yaw_rad=math.pi)
    assert dynamic_map.ingest(pre_fov) is None
    assert dynamic_map.count == 0

    visible_stamp = 2_000_000_000
    record_sweep(dynamic_map, visible_stamp, yaw_rad=0.0)
    target = dynamic_map.ingest(observation(dynamic_map, visible_stamp))
    assert target is not None
    assert dynamic_map.count == 1
    assert target.first_seen_stamp_ns == visible_stamp


def test_fov_proof_must_match_image_and_timestamp():
    dynamic_map = DynamicTrashMap.start_new("mission-frame-contract")
    stamp = 1_000_000_000
    record_sweep(dynamic_map, stamp, image_frame_id="rgb-authoritative")
    mismatched = observation(dynamic_map, stamp, image_frame_id="rgb-other")
    assert dynamic_map.ingest(mismatched) is None
    assert dynamic_map.count == 0
