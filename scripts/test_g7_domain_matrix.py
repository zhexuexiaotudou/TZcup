from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g7_detector_dataset import BOTTLE_DOMAINS, CRITICAL_METAL_DOMAINS, METAL_DOMAINS, PAPER_DOMAINS


def test_g7_domain_matrix_covers_contract():
    assert len(METAL_DOMAINS) >= 18
    assert {"silver_highly_reflective", "dark_can", "wet_road_reflection", "deep_shadow", "cluttered_roadside"} <= CRITICAL_METAL_DOMAINS
    assert {"clear_bottle", "semi_transparent_bottle", "specular_bottle", "small_distant", "wet_road"} <= set(BOTTLE_DOMAINS)
    assert {"white_paper", "folded", "partially_occluded", "thin_elongated", "similar_to_road_paint"} <= set(PAPER_DOMAINS)
