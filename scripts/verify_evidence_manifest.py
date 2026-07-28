#!/usr/bin/env python3
"""Verify byte counts, SHA-256 values and coverage of a compact evidence dir."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from autonomous_runner import verify_manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_dir", type=Path)
    args = parser.parse_args()
    errors = verify_manifest(args.evidence_dir.resolve())
    print(json.dumps({"valid": not errors, "errors": errors}, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
