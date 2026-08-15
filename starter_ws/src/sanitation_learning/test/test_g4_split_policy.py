from __future__ import annotations

import sys
from pathlib import Path

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_split_policy import (  # noqa: E402
    DEVELOPMENT_ROLES,
    LEGACY_DIAGNOSTIC_ROLE,
    SEALED_FINAL_ROLE,
    assert_development_rows,
    canonical_role_name,
    is_development_role,
    partition_rows,
    screening_decision,
)


def test_test_role_renamed_with_mandatory_warning() -> None:
    with pytest.warns(UserWarning, match="legacy_G4_D6_diagnostic"):
        role = canonical_role_name("test")
    assert role == LEGACY_DIAGNOSTIC_ROLE
    assert is_development_role("test") is False


def test_development_roles_are_recognized() -> None:
    for role in DEVELOPMENT_ROLES:
        assert canonical_role_name(role) == role
        assert is_development_role(role) is True
    assert canonical_role_name(SEALED_FINAL_ROLE) == SEALED_FINAL_ROLE
    assert is_development_role(SEALED_FINAL_ROLE) is False


def test_unknown_split_role_fails_closed() -> None:
    with pytest.raises(ValueError, match="unknown G4 split role"):
        canonical_role_name("TESTING")
    with pytest.raises(ValueError, match="undeclared split role"):
        assert_development_rows([{"split": "TRAIN"}], "training")


def test_partition_rows_renames_test_and_rejects_unknown() -> None:
    rows = [
        {"split": "train"},
        {"split": "train_world_holdout"},
        {"split": "val"},
        {"split": "test"},
        {"split": "G5_SEALED_FINAL"},
    ]
    with pytest.warns(UserWarning, match="legacy_G4_D6_diagnostic"):
        partitioned = partition_rows(rows)
    assert len(partitioned["train"]) == 1
    assert len(partitioned["train_world_holdout"]) == 1
    assert len(partitioned["val"]) == 1
    assert len(partitioned[LEGACY_DIAGNOSTIC_ROLE]) == 1
    assert len(partitioned[SEALED_FINAL_ROLE]) == 1
    with pytest.raises(ValueError):
        partition_rows([{"split": "bogus"}])


def test_development_guard_rejects_legacy_and_sealed() -> None:
    with pytest.raises(ValueError, match="legacy_G4_D6_diagnostic"):
        assert_development_rows(
            [{"split": LEGACY_DIAGNOSTIC_ROLE}], "training"
        )
    with pytest.raises(ValueError, match="G5_SEALED_FINAL"):
        assert_development_rows(
            [{"split": SEALED_FINAL_ROLE}], "threshold selection"
        )
    with pytest.raises(ValueError, match="legacy_G4_D6_diagnostic"):
        assert_development_rows([{"split": "test"}], "hard-negative mining")
    assert_development_rows(
        [{"split": "train"}, {"split": "val"}], "training"
    )


def test_screening_decision_invariant_to_legacy_diagnostic_metrics() -> None:
    development_gates = {
        "in_domain_candidate_recall_ge": True,
        "cross_world_macro_f1_ge": False,
        "onnx_task_specific_parity_pass": True,
    }
    legacy_variants = (
        {"legacy_macro_f1": True, "legacy_parity": True},
        {"legacy_macro_f1": False, "legacy_parity": False},
        {},
    )
    decisions = [
        screening_decision(
            dict(development_gates), legacy_gates=variant
        )
        for variant in legacy_variants
    ]
    assert len({decision["AUTO_05R_PASS"] for decision in decisions}) == 1
    assert decisions[0]["AUTO_05R_PASS"] is False
    for decision in decisions:
        assert decision["legacy_G4_D6_diagnostic_included_in_decision"] is False
        assert decision["G5_sealed_final_included_in_decision"] is False
    passing = screening_decision(
        {key: True for key in development_gates}
    )
    assert passing["AUTO_05R_PASS"] is True
    changed_legacy = screening_decision(
        {key: True for key in development_gates},
        legacy_gates={"legacy_macro_f1": False},
    )
    assert changed_legacy["AUTO_05R_PASS"] is True


def test_screening_decision_requires_real_gates() -> None:
    decision = screening_decision({})
    assert decision["AUTO_05R_PASS"] is False
    assert decision["AUTO_05R_BLOCKED"] is True
