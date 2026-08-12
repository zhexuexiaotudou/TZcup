from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def test_route_b_uses_fixed_holdout_and_background_verifier():
 s=(ROOT/'scripts/build_rgdrv8_route_b_crops.py').read_text(); assert "HOLDOUT_proposals_fixed_once" in s; assert "VAL_NEW_read':False" in s
 t=(ROOT/'scripts/train_rgdrv8_route_b_verifier.py').read_text(); assert 'CandidateCropClassifier' in t; assert 'background_specificity' in t; assert "ROUTE_B_VERIFIER_HOLDOUT_PASS" in t
