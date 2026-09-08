#!/usr/bin/env python3
"""Fail closed on physical DOSOD HBM output layout from ``hbrt4-disas --json``."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dosod_hbm_abi_contract import validate_hbrt4_disas
from hbm_evidence_common import normal_file, sha256_file


def validate(path: Path) -> dict[str, object]:
    normal_file(path, "hbrt4_disas_json")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("hbrt4_disas_json_invalid") from exc
    validate_hbrt4_disas(value)
    return {"status": "HBM_ABI_VERIFIED", "disas_json_sha256": sha256_file(path)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--disas-json", required=True, type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(validate(args.disas_json), indent=2))
    except ValueError as exc:
        print(f"hbm_abi_blocked:{exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
