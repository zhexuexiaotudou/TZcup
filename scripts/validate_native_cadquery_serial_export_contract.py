#!/usr/bin/env python3
"""Validate the dormant 8-batch/105-part CadQuery serial export contract.

This static validator never imports CadQuery, loads a source module, creates a
STEP file, or evaluates the current Windows runtime/memory gate.  A successful
exit proves only that the fail-closed export route is internally bound and
remains blocked pending release evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from run_native_cadquery_serial_export import (
    CONTRACT_RELATIVE,
    audit_serial_export_contract,
    load_json,
)


ROOT = Path(__file__).resolve().parents[1]


def validate(
    root: Path = ROOT, contract_path: Path | None = None
) -> dict[str, object]:
    resolved_root = root.resolve()
    resolved_contract = contract_path or (resolved_root / CONTRACT_RELATIVE)
    if not resolved_contract.is_absolute():
        resolved_contract = resolved_root / resolved_contract
    try:
        contract = load_json(resolved_contract)
    except Exception:
        contract = {}
    return audit_serial_export_contract(resolved_root, contract)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = validate(args.root, args.contract)
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        output = args.output
        if not output.is_absolute():
            output = args.root.resolve() / output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if report.get("contract_structurally_valid") is True else 2


if __name__ == "__main__":
    raise SystemExit(main())
