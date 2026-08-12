from pathlib import Path


def test_crv6_static_script_freezes_holdout_before_reading_val():
    source = (Path(__file__).parent / "screen_crv6_reconstituted.py").read_text(encoding="utf-8")
    assert source.index("atomic_json(selection_path, selection)") < source.index('load_truth(args.prepared / "val.json"')
    assert '"G7_static_VAL_used_for_selection": False' in source
    assert '"G7_static_VAL_role": "NON_GATING_HISTORICAL_REGRESSION"' in source


def test_crv6_candidate_never_relabels_historical_hash():
    source = (Path(__file__).parent / "screen_crv6_reconstituted.py").read_text(encoding="utf-8")
    assert '"candidate_id": "D1B_RECON_R1"' in source
    assert '"historical_D1B_checkpoint_impersonated": False' in source
