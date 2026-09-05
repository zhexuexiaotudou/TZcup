#!/usr/bin/env python3
"""Offline validator for one retained live RDK S100P collector artifact."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from jsonschema import Draft202012Validator, exceptions as jsonschema_exceptions

from formal_s100_live_acceptance_core import (
    active_session_identity,
    build_final_report,
    json_object,
    runtime_closure_binding,
    snapshot_identity,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RAW_SCHEMA = ROOT / "config/high_fidelity_vehicle/formal_s100_live_runtime.schema.json"
DEFAULT_FINAL_SCHEMA = ROOT / "config/high_fidelity_vehicle/formal_s100_live_acceptance.schema.json"


def validate_schema(payload: dict, schema_path: Path) -> list[str]:
    schema = json_object(schema_path)
    Draft202012Validator.check_schema(schema)
    return [
        f"{'.'.join(str(item) for item in error.absolute_path) or '<root>'}: {error.message}"
        for error in sorted(Draft202012Validator(schema).iter_errors(payload), key=lambda error: list(error.absolute_path))
    ]


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--acceptance-session", type=Path, required=True)
    parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-schema", type=Path, default=DEFAULT_RAW_SCHEMA)
    parser.add_argument("--final-schema", type=Path, default=DEFAULT_FINAL_SCHEMA)
    args = parser.parse_args()
    if args.output.exists():
        print(json.dumps({"status": "INVALID", "error": f"refusing to overwrite retained report: {args.output}"}, indent=2))
        return 2
    try:
        raw = json_object(args.raw)
        raw_schema_failures = validate_schema(raw, args.raw_schema)
        identity = snapshot_identity(args.snapshot)
        closure = runtime_closure_binding(args.runtime_closure)
        report = build_final_report(
            raw,
            identity,
            args.raw,
            active_session_identity(args.acceptance_session, identity, closure),
            closure,
        )
        if raw_schema_failures:
            report["passed"] = False
            report["status"] = "FORMAL_RDK_S100_LIVE_PRODUCT_RUNTIME_BLOCKED"
            report["blockers"] = [f"raw schema: {row}" for row in raw_schema_failures] + report["blockers"]
        final_schema_failures = validate_schema(report, args.final_schema)
        if final_schema_failures:
            raise ValueError("final report schema failure: " + "; ".join(final_schema_failures))
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError, jsonschema_exceptions.SchemaError) as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)}, indent=2))
        return 2
    atomic_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
