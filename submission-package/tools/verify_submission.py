#!/usr/bin/env python3
"""Run the non-runtime pre-submission audit and write a machine-readable receipt."""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

try:
    from pypdf import PdfReader
except ImportError:  # pragma: no cover - dependency is documented in the README
    PdfReader = None


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "docs" / "技术方案报告.pdf"
VIDEO_REPORT = ROOT / "video" / "validation" / "video-package-validation.json"
CODE_MANIFEST = ROOT / "code" / "CODE_MANIFEST.sha256"
SCORE = ROOT / "metrics" / "score-estimate.json"
READINESS = ROOT / "metrics" / "final-readiness.json"
EVIDENCE_REPORT = ROOT / "metrics" / "submission-verification.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_command(command: list[str]) -> tuple[int, str, str]:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    return completed.returncode, completed.stdout, completed.stderr


def check_report() -> dict[str, object]:
    if not REPORT.is_file():
        return {"status": "FAIL", "error": "report missing"}
    if PdfReader is None:
        return {"status": "FAIL", "error": "pypdf unavailable"}
    pages = len(PdfReader(str(REPORT)).pages)
    return {
        "status": "PASS" if 20 <= pages <= 50 else "FAIL",
        "pages": pages,
        "bytes": REPORT.stat().st_size,
        "sha256": sha256(REPORT),
    }


def check_video() -> dict[str, object]:
    if not VIDEO_REPORT.is_file():
        return {"status": "FAIL", "error": "video validation report missing"}
    payload = json.loads(VIDEO_REPORT.read_text(encoding="utf-8"))
    media = payload.get("details", {}).get("media", {})
    passed = (
        payload.get("status") == "PASS"
        and all(value == "PASS" for value in payload.get("checks", {}).values())
        and media.get("duration_s") == 300.0
        and media.get("frame_count") == 9000
        and media.get("width") == 1920
        and media.get("height") == 1080
    )
    return {
        "status": "PASS" if passed else "FAIL",
        "report": VIDEO_REPORT.relative_to(ROOT).as_posix(),
        "media": media,
    }


def check_code() -> dict[str, object]:
    if not CODE_MANIFEST.is_file():
        return {"status": "FAIL", "error": "code manifest missing"}
    code_root = CODE_MANIFEST.parent
    errors: list[str] = []
    count = 0
    pattern = re.compile(r"^(?P<hash>[0-9a-f]{64})  (?P<path>.+)$")
    for line in CODE_MANIFEST.read_text(encoding="utf-8").splitlines():
        match = pattern.fullmatch(line)
        if not match:
            continue
        count += 1
        path = code_root / match.group("path")
        if not path.is_file():
            errors.append(f"missing {match.group('path')}")
        elif sha256(path) != match.group("hash"):
            errors.append(f"hash {match.group('path')}")

    test_runner = code_root / "run_offline_tests.py"
    test_code, test_out, test_err = run_command([sys.executable, str(test_runner)])
    if test_code != 0:
        errors.append(f"offline tests failed: {test_err.strip() or test_out.strip()}")
    return {
        "status": "PASS" if not errors else "FAIL",
        "manifest_entries": count,
        "offline_tests": "PASS" if test_code == 0 else "FAIL",
        "errors": errors,
    }


def check_evidence_index() -> dict[str, object]:
    code, stdout, stderr = run_command(
        [sys.executable, str(ROOT / "tools" / "verify_evidence_index.py")]
    )
    if code != 0:
        return {"status": "FAIL", "error": stderr.strip() or stdout.strip()}
    return {"status": "PASS", **json.loads(stdout)}


def check_metrics() -> dict[str, object]:
    score = json.loads(SCORE.read_text(encoding="utf-8"))
    readiness = json.loads(READINESS.read_text(encoding="utf-8"))
    passed = (
        score.get("official_score_claimed") == 0
        and readiness.get("technical_material_status") == "READY_FOR_REVIEW"
        and readiness.get("ready_for_external_submission") is False
    )
    return {
        "status": "PASS" if passed else "FAIL",
        "score_estimate": score.get("scenarios"),
        "external_submission_blocked": not readiness.get("ready_for_external_submission"),
    }


def main() -> int:
    checks = {
        "report_pdf": check_report(),
        "video_package": check_video(),
        "code_package": check_code(),
        "evidence_index": check_evidence_index(),
        "score_and_readiness": check_metrics(),
    }
    status = "PASS" if all(item.get("status") == "PASS" for item in checks.values()) else "FAIL"
    payload = {
        "schema_version": 1,
        "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "status": status,
        "checks": checks,
    }
    EVIDENCE_REPORT.parent.mkdir(parents=True, exist_ok=True)
    EVIDENCE_REPORT.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
