from sanitation_perception.dynamic_trash_map import DynamicTrashMap

from online_map_test_support import observation, record_sweep


def test_inverse_covariance_weighted_pose_fusion_preserves_history():
    dynamic_map = DynamicTrashMap.start_new("mission-fusion")
    first_stamp = 1_000_000_000
    second_stamp = 1_100_000_000
    record_sweep(dynamic_map, first_stamp)
    first = dynamic_map.ingest(
        observation(dynamic_map, first_stamp, x_m=2.0, covariance=0.04)
    )
    record_sweep(dynamic_map, second_stamp)
    fused = dynamic_map.ingest(
        observation(dynamic_map, second_stamp, x_m=2.2, covariance=0.01)
    )

    assert fused is first
    assert abs(fused.map_x_m - 2.16) < 1e-9
    assert fused.covariance_trace < 0.01
    assert fused.observation_count == 2
    assert len(fused.image_history) == 2
    assert len(dynamic_map.observation_log) == 2


def test_class_evidence_accumulates_without_overwriting_prior_observation():
    dynamic_map = DynamicTrashMap.start_new("mission-posterior")
    for stamp in (1_000_000_000, 1_100_000_000, 1_200_000_000):
        record_sweep(dynamic_map, stamp)
        target = dynamic_map.ingest(observation(dynamic_map, stamp))
    assert target.current_class == "plastic_bottle"
    assert target.class_posterior["plastic_bottle"] > 0.99
    assert len(target.image_history) == 3
