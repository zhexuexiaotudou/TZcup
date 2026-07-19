import numpy as np

from sanitation_learning.g2_metrics import measure_instances, normalized_domain_metrics, summarize_instance_rows


def test_instance_metrics_are_not_class_aggregates_and_zero_pixels_are_not_visible():
    semantic = np.zeros((8, 10), dtype=np.uint8)
    instances = np.zeros_like(semantic, dtype=np.uint32)
    depth = np.full_like(semantic, 2.0, dtype=np.float32)
    semantic[1:4, 2:6] = 1
    instances[1:4, 2:6] = 101
    semantic[5:7, 7:9] = 1
    instances[5:7, 7:9] = 102
    rows = measure_instances(semantic, instances, depth, [
        {"runtime_instance_id": 101, "class_id": "plastic_bottle", "unoccluded_reference_area_px": 16},
        {"runtime_instance_id": 102, "class_id": "plastic_bottle"},
        {"runtime_instance_id": 103, "class_id": "plastic_bottle"},
    ])
    assert [row["mask_area_px"] for row in rows] == [12, 4, 0]
    assert rows[0]["bbox_xywh_px"] == [2, 1, 4, 3]
    assert rows[0]["bbox_shortest_side_px"] == 3
    assert rows[2]["visibility"] == "not_visible"
    summary = summarize_instance_rows(rows)
    assert summary["visibility"]["plastic_bottle"] == {"visible": 2, "not_visible": 1}


def test_cross_world_is_null_for_one_world():
    metrics = normalized_domain_metrics(
        same_world={"macro_f1": 0.7}, cross_material={"macro_f1": 0.6},
        cross_world={"macro_f1": 0.5}, distinct_world_count=1,
    )
    assert metrics["cross_world"] is None
    assert metrics["cross_world_eligibility"] is False
