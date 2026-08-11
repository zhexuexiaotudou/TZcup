import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "oprv3_sealed", ROOT / "scripts/perception_oprv3_sealed_final.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_sealed_policy_is_fail_closed():
    policy = {
        "gates": {
            "recall": {"metric": "object.recall", "operator": "ge", "threshold": 0.95},
            "false_rate": {"metric": "online.false_rate", "operator": "le", "threshold": 0.01},
            "pre_fov": {"metric": "online.pre_fov", "operator": "eq", "threshold": 0},
        }
    }
    passed = MODULE.evaluate_policy(
        policy, {"object": {"recall": 0.96}, "online": {"false_rate": 0.01, "pre_fov": 0}}
    )
    assert passed["pass"] is True
    failed = MODULE.evaluate_policy(
        policy, {"object": {"recall": 0.94}, "online": {"false_rate": 0.011, "pre_fov": 1}}
    )
    assert failed["pass"] is False
    assert set(failed["gates"]) == {"recall", "false_rate", "pre_fov"}


def test_area_aggregation_uses_pixel_totals_and_negative_frames():
    rows = [
        {"scene_seed": 1, "frame_index": 0, "negative_only": False},
        {"scene_seed": 2, "frame_index": 0, "negative_only": True},
    ]
    item = {
        "intersection_pixels": 8, "union_pixels": 10,
        "boundary_intersection_pixels": 4, "boundary_union_pixels": 6,
        "has_area_candidate": False,
    }
    negative = {**item, "intersection_pixels": 0, "union_pixels": 0, "has_area_candidate": True}
    areas = {
        (1, 0): {"leaf_pile": item, "puddle": item},
        (2, 0): {"leaf_pile": negative, "puddle": {**negative, "has_area_candidate": False}},
    }
    metrics = MODULE.area_metrics(rows, areas)
    assert metrics["iou_by_class"]["leaf_pile"] == 0.8
    assert metrics["boundary_f1"] == 0.8
    assert metrics["negative_area_fp_per_frame"] == 1.0
