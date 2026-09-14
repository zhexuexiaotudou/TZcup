#!/usr/bin/env python3
"""Create closure-bound capture inputs and invoke the offline perception scorer."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from rescore_product_capture_random_scene import _inventory_digest, _sha256, rescore


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture-root", "public-manifest", "evaluator-truth", "session-status", "runtime-binding", "binding-output", "report-output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--projection-evidence", type=Path)
    args = parser.parse_args()
    if len(args.source_commit) != 40 or any(c not in "0123456789abcdef" for c in args.source_commit):
        raise SystemExit("source commit must be a lowercase 40-hex SHA")
    public, truth, session, runtime = map(_load, (args.public_manifest, args.evaluator_truth, args.session_status, args.runtime_binding))
    if public.get("episode_id") != truth.get("episode_id") or public.get("map_id") != truth.get("map_id"):
        raise SystemExit("public/evaluator identity mismatch")
    binding = {"schema_version": 1, "episode_id": public["episode_id"], "map_id": public["map_id"], "source_commit": args.source_commit,
        "capture_manifest_sha256": _inventory_digest(args.capture_root), "public_manifest_sha256": _sha256(args.public_manifest), "evaluator_truth_sha256": _sha256(args.evaluator_truth),
        "acceptance_session_binding": {"path": str(args.session_status.resolve()), "sha256": _sha256(args.session_status), "started_epoch_ns": session.get("started_epoch_ns")},
        "runtime_closure_binding": {"path": str(args.runtime_binding.resolve()), "sha256": _sha256(args.runtime_binding), "closure": runtime.get("runtime_closure_binding")},
        "product_truth_input_used": False}
    args.binding_output.parent.mkdir(parents=True, exist_ok=True)
    args.binding_output.write_text(json.dumps(binding, indent=2) + "\n", encoding="utf-8")
    report = rescore(args.capture_root, args.public_manifest, args.evaluator_truth, args.binding_output, args.projection_evidence)
    args.report_output.parent.mkdir(parents=True, exist_ok=True)
    args.report_output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "report": str(args.report_output)}))
    return 0 if report["status"] == "RESCORED_OFFLINE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
