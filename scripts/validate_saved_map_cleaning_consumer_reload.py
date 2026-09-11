#!/usr/bin/env python3
"""Fail closed before a hard-restarted saved-map cleaning consumer starts."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws/src/sanitation_formal_campus_integration"
sys.path.insert(0, str(PACKAGE))

from sanitation_formal_campus_integration.map_lifecycle_core import (
    MapLifecycleError,
    load_campus_map_contract,
    validate_mapping_handoff_record,
    validate_saved_map_cleaning_consumer_bundle,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-root", required=True, type=Path)
    parser.add_argument("--episode-manifest", required=True, type=Path)
    parser.add_argument("--mapping-handoff-record", required=True, type=Path)
    args = parser.parse_args()
    try:
        if args.mapping_handoff_record.resolve() != (args.map_root / "mapping_handoff_record.json").resolve():
            raise MapLifecycleError("consumer reload must use the sealed mapping handoff record")
        validate_mapping_handoff_record(args.map_root)
        validate_saved_map_cleaning_consumer_bundle(
            args.map_root, load_campus_map_contract(args.episode_manifest)
        )
    except (MapLifecycleError, OSError, ValueError) as exc:
        print(f"SAVED_MAP_CLEANING_CONSUMER_BLOCKED: {exc}", file=sys.stderr)
        return 2
    print("SAVED_MAP_CLEANING_CONSUMER_READY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
