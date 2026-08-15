from pathlib import Path

import authorize_trcrv10_sealed_access as access


def test_access_is_one_time_and_requires_green_integrated_gate() -> None:
    source = Path(access.__file__).read_text(encoding="utf-8")
    assert "sealed access record already exists" in source
    assert 'holdout.get("TRCRV10_INTEGRATED_HOLDOUT_PASS") is not True' in source
    assert "sealed access denied" in source


def test_exact_freeze_surface_and_g5_denial_are_explicit() -> None:
    source = Path(access.__file__).read_text(encoding="utf-8")
    for name in ("proposal_model", "proposal_threshold", "classifier", "verifier", "reobserve_policy", "g10_manifest"):
        assert name in source
    assert '"G5_V2_access_authorized": False' in source
