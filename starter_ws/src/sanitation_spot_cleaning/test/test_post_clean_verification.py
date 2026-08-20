from sanitation_spot_cleaning.post_clean_verification import (
    PostCleanVerifier,
    VerificationOutcome,
)


def test_discrete_target_requires_multiple_absent_frames():
    verifier = PostCleanVerifier()
    verifier.begin("bottle", target_type="DISCRETE", clean_attempt=1)
    assert verifier.observe_discrete("bottle", detected=False, confidence=0.0) == VerificationOutcome.CONTINUE_OBSERVING
    assert verifier.observe_discrete("bottle", detected=False, confidence=0.0) == VerificationOutcome.CONTINUE_OBSERVING
    assert verifier.observe_discrete("bottle", detected=False, confidence=0.0) == VerificationOutcome.CLEANED
    assert verifier.finalize("bottle") == VerificationOutcome.CLEANED


def test_residual_triggers_one_reclean_then_manual_attention():
    verifier = PostCleanVerifier()
    verifier.begin("paper", target_type="DISCRETE", clean_attempt=1)
    verifier.observe_discrete("paper", detected=True, confidence=0.8)
    assert verifier.finalize("paper") == VerificationOutcome.RECLEAN
    verifier.begin("paper", target_type="DISCRETE", clean_attempt=2)
    verifier.observe_discrete("paper", detected=True, confidence=0.8)
    assert verifier.finalize("paper") == VerificationOutcome.MANUAL_ATTENTION


def test_area_target_requires_ninety_percent_reduction():
    verifier = PostCleanVerifier()
    verifier.begin("leaf", target_type="AREA", clean_attempt=1, area_before_m2=1.0)
    assert verifier.observe_area("leaf", area_after_m2=0.11) == VerificationOutcome.CONTINUE_OBSERVING
    assert verifier.finalize("leaf") == VerificationOutcome.RECLEAN
    verifier.begin("leaf", target_type="AREA", clean_attempt=2, area_before_m2=0.11)
    assert verifier.observe_area("leaf", area_after_m2=0.01) == VerificationOutcome.CLEANED
