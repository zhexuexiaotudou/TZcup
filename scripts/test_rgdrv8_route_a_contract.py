from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def test_route_a_train_is_ga1_warm_started_and_val_closed():
    s=(ROOT/'scripts/train_rgdrv8_route_a.py').read_text(); assert "cfg.load_from=str(a.checkpoint)" in s; assert "'VAL_NEW_read':False" in s; assert "official_mmdetection_v3.3.0_rtmdet_s" in s
def test_route_a_selection_is_holdout_only_and_constraint_aware():
    s=(ROOT/'scripts/select_rgdrv8_route_a.py').read_text(); assert "'selection_data':'HOLDOUT_NEW_ONLY'" in s; assert "wrong_actionable_rate" in s; assert "small_eventual_correct_recall" in s
