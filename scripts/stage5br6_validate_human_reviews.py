from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
LEARNING = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING))

from sanitation_learning.human_review_handoff import validate_completed_response  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Integrity-check two human-completed Stage5BR6 responses without editing them.")
    parser.add_argument("--handoff-manifest", required=True)
    parser.add_argument("--reviewer-a", required=True)
    parser.add_argument("--reviewer-b", required=True)
    args = parser.parse_args()
    manifest = json.loads(Path(args.handoff_manifest).read_text(encoding="utf-8"))
    paths = [Path(args.reviewer_a), Path(args.reviewer_b)]
    responses = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
    for response, package in zip(responses, manifest["reviewer_packages"], strict=True):
        validate_completed_response(response, package)
    if responses[0]["reviewer_pseudonym"] == responses[1]["reviewer_pseudonym"]:
        raise ValueError("reviewer pseudonyms must be different")
    if responses[0]["package_id"] == responses[1]["package_id"]:
        raise ValueError("reviewers must use different packages")
    print(json.dumps({
        "manual_audit_integrity_pass": True,
        "response_sha256": [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths],
        "responses_modified": False,
        "scoring_executed": False,
    }, indent=2))


if __name__ == "__main__":
    main()
