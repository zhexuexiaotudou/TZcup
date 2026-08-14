from pathlib import Path


def test_finalizer_enforces_stop_condition_b_and_sealed_boundary():
    source = Path(__file__).with_name("finalize_crcrv11.py").read_text(encoding="utf-8")
    assert "B_R1_R2_R3_ALL_FAILED" in source
    assert "stop condition B requires explicit R1/R2/R3 failures" in source
    assert "sealed boundary changed before stop condition B" in source
    assert '"SIMULATION_PRODUCT_COMPLETE": False' in source
    assert '"release_bundle_created": False' in source
