from pathlib import Path


SOURCE = Path(__file__).with_name("finalize_crv6.py").read_text(encoding="utf-8")


def test_finalizer_requires_commit_bound_online_evidence():
    assert 'online.get("source_commit") != args.source_commit' in SOURCE
    assert 'online.get("candidate_sha256") != candidate_hash' in SOURCE


def test_finalizer_cannot_create_release_when_x86_gate_is_blocked():
    assert '"release_created": False' in SOURCE
    assert '"G5_V2_read": False' in SOURCE
    assert '"MODEL_FREEZE_X86_CREATED": False' in SOURCE
    assert '"PR_READY_ALLOWED": False' in SOURCE


def test_finalizer_emits_all_mandatory_crv6_outputs():
    for name in (
        "PERCEPTION_CRV6_FINAL_STATUS.json",
        "PERCEPTION_CRV6_FINAL_BLOCKERS.json",
        "PERCEPTION_CRV6_EVIDENCE_INDEX.md",
        "PERCEPTION_CRV6_MODEL_REGISTRY.json",
        "PERCEPTION_CRV6_RELEASE_MANIFEST.json",
        "PERCEPTION_CRV6_THIRD_PARTY_NOTICES.md",
        "CHECKPOINT_RECONSTITUTION_V6_REPORT.md",
    ):
        assert name in SOURCE
