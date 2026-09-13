import json
from pathlib import Path

import pytest

from verify_competition_perception_replay_evidence import (
    ARTIFACT_PATH,
    DOCUMENT_PATH,
    ReplayEvidenceError,
    verify_evidence,
)


ROOT = Path(__file__).resolve().parents[1]


def _write_evidence(root: Path, policy_status: str, policy_metrics: dict | None) -> None:
    artifact = {
        "status": "RAW_REPLAY_RETAINED_POLICY_NOT_RUN",
        "raw_results": {
            "stage5a_controlled_fixture": {
                "totals": {"tp": 33, "fp": 50, "fn": 43},
                "evidence": {"sha256": "a" * 64},
            },
            "stage5b_original_model_replay": {
                "totals": {"tp": 0, "fp": 654, "fn": 76},
                "evidence": {"sha256": "b" * 64},
            },
        },
        "policy_results": {
            "status": policy_status,
            "metrics": policy_metrics,
        },
    }
    artifact_path = root / ARTIFACT_PATH
    artifact_path.parent.mkdir(parents=True)
    artifact_path.write_text(json.dumps(artifact), encoding="utf-8")

    document_path = root / DOCUMENT_PATH
    document_path.parent.mkdir(parents=True, exist_ok=True)
    policy_value = "NOT_RUN" if policy_metrics is None else "1 / 2 / 3"
    document_path.write_text(
        "## 2026-09-14 离线根因复核\n\n"
        f"| Stage5A controlled profile | policy | {policy_value} |\n",
        encoding="utf-8",
    )


def test_checked_in_replay_evidence_is_consistent():
    result = verify_evidence(ROOT)
    assert result["stage5a_raw_totals"] == {"tp": 33, "fp": 50, "fn": 43}
    assert result["stage5b_raw_totals"] == {"tp": 0, "fp": 654, "fn": 76}
    assert result["policy_status"] == "NOT_RUN"


def test_numeric_policy_claim_is_rejected_when_policy_was_not_run(tmp_path):
    _write_evidence(tmp_path, "NOT_RUN", None)
    document_path = tmp_path / DOCUMENT_PATH
    document_path.write_text(
        document_path.read_text(encoding="utf-8").replace(
            "NOT_RUN", "1 / 2 / 3", 1
        ),
        encoding="utf-8",
    )
    with pytest.raises(ReplayEvidenceError, match="numeric policy metrics"):
        verify_evidence(tmp_path)


def test_measured_policy_claim_requires_metrics(tmp_path):
    _write_evidence(tmp_path, "MEASURED_NOT_OFFICIAL_ACCEPTANCE", None)
    with pytest.raises(ReplayEvidenceError, match="requires metrics"):
        verify_evidence(tmp_path)
