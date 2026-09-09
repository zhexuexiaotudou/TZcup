#!/usr/bin/env python3
"""Fail-closed structural checks for a completed rosbag2 MCAP directory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


MCAP_MAGIC = b"\x89MCAP0\r\n"
_FOOTER_RECORD_SIZE = 1 + 8 + 20 + len(MCAP_MAGIC)


def _has_mcap_footer(path: Path) -> bool:
    """Return whether *path* has both MCAP magics and the terminal footer."""
    try:
        if path.stat().st_size < len(MCAP_MAGIC) + _FOOTER_RECORD_SIZE:
            return False
        with path.open("rb") as stream:
            if stream.read(len(MCAP_MAGIC)) != MCAP_MAGIC:
                return False
            stream.seek(-_FOOTER_RECORD_SIZE, 2)
            footer = stream.read(_FOOTER_RECORD_SIZE)
    except OSError:
        return False
    return (
        footer[:1] == b"\x02"
        and int.from_bytes(footer[1:9], "little") == 20
        and footer[-len(MCAP_MAGIC):] == MCAP_MAGIC
    )


def inspect_mcap_bag(directory: Path) -> dict:
    """Return evidence whose ``sealed`` field is true only for a complete bag."""
    metadata_path = directory / "metadata.yaml"
    evidence = {
        "path": directory.name,
        "metadata_present": metadata_path.is_file(),
        "metadata_valid": False,
        "message_count": 0,
        "duration_ns": 0,
        "topics": [],
        "mcap_files": [],
        "footer_complete": False,
        "sealed": False,
    }
    if not metadata_path.is_file():
        return evidence

    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        info = metadata["rosbag2_bagfile_information"]
        if not isinstance(info, dict):
            return evidence
        evidence["message_count"] = int(info.get("message_count", 0))
        evidence["duration_ns"] = int((info.get("duration") or {}).get("nanoseconds", 0))
        evidence["topics"] = sorted(
            row.get("topic_metadata", {}).get("name")
            for row in info.get("topics_with_message_count", [])
            if isinstance(row, dict) and row.get("topic_metadata", {}).get("name")
        )
        relative_paths = info.get("relative_file_paths") or []
        mcap_paths = [directory / name for name in relative_paths if str(name).endswith(".mcap")]
        if not mcap_paths:
            mcap_paths = sorted(directory.glob("*.mcap"))
    except (OSError, TypeError, ValueError, yaml.YAMLError, KeyError):
        return evidence

    evidence["metadata_valid"] = True
    evidence["mcap_files"] = [path.name for path in mcap_paths]
    evidence["footer_complete"] = bool(mcap_paths) and all(
        _has_mcap_footer(path) for path in mcap_paths
    )
    evidence["sealed"] = bool(
        evidence["metadata_valid"] and evidence["footer_complete"]
    )
    return evidence


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--require-sealed", action="store_true")
    args = parser.parse_args()
    evidence = inspect_mcap_bag(args.bag)
    print(json.dumps(evidence, ensure_ascii=False, indent=2))
    return 0 if not args.require_sealed or evidence["sealed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
