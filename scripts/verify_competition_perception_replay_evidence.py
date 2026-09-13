#!/usr/bin/env python3
"""Fail closed when replay evidence and policy claims disagree."""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


ARTIFACT_PATH = Path("artifacts/perception_replay_20260914_review/replay_status.json")
DOCUMENT_PATH = Path("docs/competition-perception-score.md")
POLICY_SECTION = "## 2026-09-14 离线根因复核"
MEASURED_POLICY_STATUSES = {"MEASURED", "MEASURED_NOT_OFFICIAL_ACCEPTANCE"}
NUMERIC_POLICY_CLAIM = re.compile(r"\b\d+\s*/\s*\d+\s*/\s*\d+\b")


class ReplayEvidenceError(ValueError):
    pass


def _load_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReplayEvidenceError(f"cannot read replay evidence: {path}") from exc
    if not isinstance(value, dict):
        raise ReplayEvidenceError(f"replay evidence must be a JSON object: {path}")
    return value


def _totals(record: dict, label: str) -> dict[str, int]:
    totals = record.get("totals")
    if not isinstance(totals, dict):
        raise ReplayEvidenceError(f"{label} totals are missing")
    result = {}
    for key in ("tp", "fp", "fn"):
        value = totals.get(key)
        if type(value) is not int or value < 0:
            raise ReplayEvidenceError(f"{label}.{key} must be a non-negative integer")
        result[key] = value
    return result


def _verify_sha256(record: dict, label: str) -> None:
    evidence = record.get("evidence")
    if not isinstance(evidence, dict):
        raise ReplayEvidenceError(f"{label} evidence metadata is missing")
    sha256 = evidence.get("sha256")
    if not isinstance(sha256, str) or re.fullmatch(r"[0-9a-f]{64}", sha256) is None:
        raise ReplayEvidenceError(f"{label} evidence sha256 is invalid")


def verify_evidence(root: str | Path) -> dict:
    root = Path(root).resolve()
    artifact = _load_json(root / ARTIFACT_PATH)
    raw_results = artifact.get("raw_results")
    if not isinstance(raw_results, dict):
        raise ReplayEvidenceError("raw_results must be an object")

    expected_raw = {
        "stage5a_controlled_fixture": {"tp": 33, "fp": 50, "fn": 43},
        "stage5b_original_model_replay": {"tp": 0, "fp": 654, "fn": 76},
    }
    for label, expected in expected_raw.items():
        record = raw_results.get(label)
        if not isinstance(record, dict):
            raise ReplayEvidenceError(f"missing raw replay record: {label}")
        _verify_sha256(record, label)
        if _totals(record, label) != expected:
            raise ReplayEvidenceError(f"{label} raw totals changed")

    policy = artifact.get("policy_results")
    if not isinstance(policy, dict):
        raise ReplayEvidenceError("policy_results must be an object")
    status = policy.get("status")
    metrics = policy.get("metrics")
    if status in MEASURED_POLICY_STATUSES:
        if not isinstance(metrics, dict):
            raise ReplayEvidenceError("measured policy result requires metrics")
        _totals(metrics, "policy")
    else:
        if metrics is not None:
            raise ReplayEvidenceError("unmeasured policy result must use null metrics")

        document = (root / DOCUMENT_PATH).read_text(encoding="utf-8")
        if POLICY_SECTION not in document:
            raise ReplayEvidenceError("replay document section is missing")
        replay_section = document.split(POLICY_SECTION, 1)[1]
        policy_lines = [
            line
            for line in replay_section.splitlines()
            if line.lstrip().startswith("|") and "policy" in line.lower()
        ]
        if not policy_lines:
            raise ReplayEvidenceError("replay document must expose the policy status")
        for line in policy_lines:
            if NUMERIC_POLICY_CLAIM.search(line) and "not_run" not in line.lower():
                raise ReplayEvidenceError(
                    "numeric policy metrics are claimed while policy replay is not measured"
                )

    return {
        "status": artifact.get("status"),
        "policy_status": status,
        "stage5a_raw_totals": expected_raw["stage5a_controlled_fixture"],
        "stage5b_raw_totals": expected_raw["stage5b_original_model_replay"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    print(json.dumps(verify_evidence(args.root), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
