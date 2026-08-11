import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g7_detector_dataset import G7Plan, NEGATIVE_TAXONOMIES, SPLITS, build_g7_dataset


def test_g7_negative_taxonomy_has_all_sixteen_categories():
    assert len(NEGATIVE_TAXONOMIES) == 16
    assert {"wet_road", "specular_road", "oil_like_patch", "lane_marking", "manhole", "metal_reflection", "plastic_like_clutter", "dark_debris_like_non_target"} <= set(NEGATIVE_TAXONOMIES)


def test_negative_rotation_is_driven_by_negative_index_not_frame_stride(tmp_path):
    plan = G7Plan(
        frames_by_split={name: 4 for name in SPLITS},
        frames_per_scene=1,
        full_negative_target=16,
        formal=False,
    )
    root = tmp_path / "g7-negative-cycle"
    build_g7_dataset(root, plan)
    report = json.loads(
        (root / "reports/G7_NEGATIVE_TAXONOMY.json").read_text(encoding="utf-8")
    )
    counts = report["taxonomy_counts_by_split"]
    assert all(sum(by_split.values()) == 1 for by_split in counts.values())
