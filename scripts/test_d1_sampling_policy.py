from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from prepare_ddrv4_d1 import EXPOSURE_RATIOS, sampled_exposures


def test_d1_sampling_policy_meets_exact_exposure_floor():
    frames, instances = [], {}
    categories = (
        ("metal", [{"class_id": "metal_can", "bbox_short_side_px": 25}]),
        ("negative", []),
        ("small", [{"class_id": "paper_litter", "bbox_short_side_px": 12}]),
        ("general", [{"class_id": "plastic_bottle", "bbox_short_side_px": 25}]),
    )
    for index, (_, objects) in enumerate(categories):
        row = {"scene_seed": index, "frame_index": 0, "negative_only": not objects, "rgb_path": f"rgb/{index}.png", "world_id": "g7v4_test"}
        frames.append(row); instances[(index, 0)] = objects
    selected, audit = sampled_exposures(frames, instances, total=2000)
    assert len(selected) == 2000
    assert audit["ratios"] == EXPOSURE_RATIOS
    assert audit["counts"] == {"metal_can_targeted": 500, "negative_only": 500, "small_object_positive": 400, "general_positive": 600}
    assert audit["G6_used"] is False and audit["G5_used"] is False and audit["G5_V2_used"] is False
