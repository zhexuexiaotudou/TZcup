#!/usr/bin/env python3
"""Static verification for the nontechnical score-closure package."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import sys
from pathlib import Path


REQUIRED_FILES = (
    ".gitattributes",
    "README.md",
    "official-score-closure.md",
    "operator-usability.md",
    "maintenance-three-step.md",
    "reliability-case.md",
    "social-benefit.md",
    "system-completeness.md",
    "demo-video-production.md",
    "industrialization-readiness.md",
    "ip-research-boundary.md",
    "invention-disclosure-template.md",
    "paper-outline.md",
    "data/official-score-items.json",
    "data/closure-status.json",
    "data/maintenance-procedure.json",
    "data/reliability-event-dictionary.json",
    "data/operator-certification-template.csv",
    "data/demo-storyboard.csv",
    "data/demo-acceptance-checklist.csv",
    "data/social-benefit-scenarios.json",
    "data/industrialization-gates.csv",
    "data/ip-boundary.json",
    "tools/operator_certification.py",
    "tools/validate_maintenance_procedure.py",
    "tools/reliability_qualification.py",
    "tools/social_benefit_model.py",
    "tools/validate_demo_media.py",
    "tools/build_supplement_pdf.py",
    "pdf/TZcup-nontechnical-score-closure.pdf",
)

EXPECTED_STATUSES = {
    "APP-USE-TRAINING": "PLANNED_NOT_MEASURED",
    "APP-USE-MAINTENANCE": "DESIGN_ONLY_NOT_PHYSICALLY_MEASURED",
    "APP-USE-RELIABILITY": "INSTRUMENTED_NOT_MEASURED",
    "APP-VALUE-SOCIAL": "MODEL_ONLY_NOT_FIELD_MEASURED",
    "COMP-HW-SW": "PARTIAL",
    "COMP-DEMO": "KIT_ONLY_VIDEO_NOT_MEASURED",
    "COMP-DOCUMENT": "DELIVERED",
    "BONUS-INDUSTRIALIZATION": "REVIEW_READY_NOT_FIELD_VALIDATED",
    "BONUS-IP-PAPER": "NOT_ELIGIBLE_NO_FILING_OR_PUBLICATION",
}


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_module(relative: str):
    root = package_root()
    path = root / relative
    name = path.stem
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def validate_storyboard(rows: list[dict[str, str]], errors: list[str]) -> float:
    if not rows:
        errors.append("demo storyboard has no rows")
        return 0.0
    cursor = 0
    for index, row in enumerate(rows, start=1):
        try:
            start = int(row["start_s"])
            end = int(row["end_s"])
            duration = int(row["duration_s"])
        except (KeyError, ValueError):
            errors.append(f"storyboard row {index}: invalid timing")
            continue
        if start != cursor or end - start != duration or duration <= 0:
            errors.append(f"storyboard row {index}: timing is not contiguous")
        cursor = end
    if not 300 <= cursor <= 600:
        errors.append(f"storyboard total duration must be 300-600s; got {cursor}")
    return float(cursor)


def pdf_metadata(path: Path) -> dict[str, object]:
    metadata: dict[str, object] = {
        "exists": path.is_file(),
        "bytes": path.stat().st_size if path.is_file() else 0,
    }
    try:
        from pypdf import PdfReader
    except ImportError:
        metadata["pypdf_available"] = False
        metadata["pages"] = None
        return metadata
    reader = PdfReader(str(path))
    metadata["pypdf_available"] = True
    metadata["pages"] = len(reader.pages)
    metadata["encrypted"] = reader.is_encrypted
    return metadata


def build_report() -> tuple[dict[str, object], list[str]]:
    root = package_root()
    errors: list[str] = []
    missing = [relative for relative in REQUIRED_FILES if not (root / relative).is_file()]
    if missing:
        errors.append(f"missing required files: {missing}")
        return {"status": "FAIL", "errors": errors}, errors

    official = json.loads((root / "data/official-score-items.json").read_text(encoding="utf-8"))
    closure = json.loads((root / "data/closure-status.json").read_text(encoding="utf-8"))
    official_items = official.get("items", [])
    official_by_id = {item.get("id"): item for item in official_items}
    if set(official_by_id) != set(EXPECTED_STATUSES):
        errors.append("official score item IDs do not match the frozen closure scope")
    for item_id, expected_status in EXPECTED_STATUSES.items():
        if official_by_id.get(item_id, {}).get("status") != expected_status:
            errors.append(f"{item_id}: official status is not fail-closed")
        if closure.get("items", {}).get(item_id, {}).get("status") != expected_status:
            errors.append(f"{item_id}: closure status differs from official matrix")
        if closure.get("items", {}).get(item_id, {}).get("official_points_claimed") != 0.0:
            errors.append(f"{item_id}: package claims official points without measurement")

    maintenance = json.loads(
        (root / "data/maintenance-procedure.json").read_text(encoding="utf-8")
    )
    maintenance_module = load_module("tools/validate_maintenance_procedure.py")
    errors.extend(
        f"maintenance: {error}" for error in maintenance_module.validate(maintenance)
    )

    reliability = json.loads(
        (root / "data/reliability-event-dictionary.json").read_text(encoding="utf-8")
    )
    reliability_module = load_module("tools/reliability_qualification.py")
    plan = reliability_module.qualification_plan()
    expected_minimums = {
        "zero_primary_failures": 299,
        "one_primary_failure": 473,
        "two_primary_failures": 628,
    }
    if plan["minimum_vehicle_days"] != expected_minimums:
        errors.append("reliability qualification sample sizes changed unexpectedly")
    qualification = reliability.get("qualification", {})
    dictionary_keys = {
        "zero_primary_failures": "zero_fault_min_vehicle_days",
        "one_primary_failure": "one_fault_min_vehicle_days",
        "two_primary_failures": "two_fault_min_vehicle_days",
    }
    for key, value in expected_minimums.items():
        dictionary_key = dictionary_keys[key]
        if qualification.get(dictionary_key) != value:
            errors.append(f"reliability dictionary mismatch for {dictionary_key}")

    social = json.loads(
        (root / "data/social-benefit-scenarios.json").read_text(encoding="utf-8")
    )
    social_module = load_module("tools/social_benefit_model.py")
    social_report = social_module.evaluate_document(social)
    if social_report["status"] != "MODEL_ONLY_NOT_FIELD_MEASURED":
        errors.append("social-benefit model overstates its evidence status")
    if any(item["project_result_claimable"] for item in social_report["scenarios"]):
        errors.append("illustrative social-benefit scenario is marked claimable")

    storyboard = read_csv(root / "data/demo-storyboard.csv")
    storyboard_duration = validate_storyboard(storyboard, errors)
    checklist = read_csv(root / "data/demo-acceptance-checklist.csv")
    required_checklist = {
        "DUR-01",
        "RES-01",
        "FPS-01",
        "CODE-01",
        "AUD-01",
        "TASK-01",
        "CLEAN-01",
        "SAFE-01",
        "RESULT-01",
        "VERSION-01",
        "TRUTH-01",
    }
    if not required_checklist.issubset({row.get("id") for row in checklist}):
        errors.append("demo acceptance checklist misses a required gate")

    gates = read_csv(root / "data/industrialization-gates.csv")
    if {row.get("id") for row in gates} != {"G0", "G1", "G2", "G3", "G4", "G5"}:
        errors.append("industrialization gates must contain G0 through G5")
    premature_pass = [
        row.get("id")
        for row in gates
        if row.get("id") != "G0" and "PASS" in row.get("status", "")
    ]
    if premature_pass:
        errors.append(f"industrialization gates pass without field evidence: {premature_pass}")

    ip = json.loads((root / "data/ip-boundary.json").read_text(encoding="utf-8"))
    if ip.get("status") != "NOT_ELIGIBLE_NO_FILING_OR_PUBLICATION":
        errors.append("IP status must remain ineligible without filing or publication")
    if ip.get("official_points_claimed") != 0.0:
        errors.append("IP package claims official points without a filing or publication")

    pdf = pdf_metadata(root / "pdf/TZcup-nontechnical-score-closure.pdf")
    if not pdf["exists"] or int(pdf["bytes"]) < 10_000:
        errors.append("supplement PDF is missing or unexpectedly small")
    if pdf.get("pages") is not None and int(pdf["pages"]) < 10:
        errors.append("supplement PDF must have at least 10 pages")

    hashes = {
        relative: sha256(root / relative)
        for relative in REQUIRED_FILES
        if relative != "pdf/TZcup-nontechnical-score-closure.pdf"
    }
    report = {
        "status": "PASS" if not errors else "FAIL",
        "package_version": closure.get("package_version"),
        "scope": closure.get("scope"),
        "required_files_checked": len(REQUIRED_FILES),
        "official_items_checked": len(official_items),
        "official_points_claimed": 0.0,
        "storyboard_duration_s": storyboard_duration,
        "demo_checklist_items": len(checklist),
        "industrialization_gates": len(gates),
        "reliability_plan": plan["minimum_vehicle_days"],
        "pdf": pdf,
        "supporting_file_hashes": hashes,
        "errors": errors,
    }
    return report, errors


def write_manifest(root: Path) -> int:
    files = sorted(
        path
        for path in root.rglob("*")
        if path.is_file()
        and "evidence" not in path.relative_to(root).parts
        and "__pycache__" not in path.parts
        and path.name != "MANIFEST.sha256"
    )
    lines = [
        f"{sha256(path)}  {path.relative_to(root).as_posix()}"
        for path in files
    ]
    (root / "MANIFEST.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(files)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    root = package_root()
    report, errors = build_report()
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.write and not errors:
        (root / "evidence").mkdir(exist_ok=True)
        (root / "evidence" / "verification-report.json").write_text(
            payload, encoding="utf-8"
        )
        report["manifest_files"] = write_manifest(root)
        payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        (root / "evidence" / "verification-report.json").write_text(
            payload, encoding="utf-8"
        )
    print(payload, end="")
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
