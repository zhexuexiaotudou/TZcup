#!/usr/bin/env python3
"""Discover an official Journey 6 SDK without treating look-alike SDKs as valid.

Discovery is intentionally read-only and fail-closed.  A package whose name or
metadata mentions RDK/S100/S100P is reported as rejected even when it contains
similarly named Horizon tools.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
from typing import Iterable


J6_MARKERS = ("journey6", "journey_6", "journey-6", "j6 openexplorer")
FORBIDDEN_MARKERS = ("rdk", "s100", "s100p", "s600")
TOOL_MARKERS = (
    "hb_compile",
    "hb_model_info",
    "hb_verifier",
    "hbdk4_compiler",
    "horizon_tc_ui",
    "hmct",
)


def _normalized(value: str) -> str:
    return re.sub(r"\s+", " ", value.lower().replace("\\", "/"))


def _probe_text(root: Path) -> str:
    snippets = [str(root)]
    if root.is_dir():
        for name in ("README", "README.md", "README-EN", "VERSION", "version.txt"):
            candidate = root / name
            if candidate.is_file():
                try:
                    snippets.append(candidate.read_text(encoding="utf-8", errors="ignore")[:65536])
                except OSError:
                    pass
    return _normalized("\n".join(snippets))


def inspect_candidate(root: Path) -> dict:
    resolved = root.expanduser().resolve()
    exists = resolved.exists()
    text = _probe_text(resolved) if exists else _normalized(str(resolved))
    forbidden = sorted(marker for marker in FORBIDDEN_MARKERS if marker in text)
    family_evidence = sorted(marker for marker in J6_MARKERS if marker in text)
    tools: dict[str, list[str]] = {marker: [] for marker in TOOL_MARKERS}
    if exists and resolved.is_dir():
        for path in resolved.rglob("*"):
            if not path.is_file():
                continue
            lowered = path.name.lower().replace("-", "_")
            for marker in TOOL_MARKERS:
                if marker in lowered:
                    tools[marker].append(str(path.relative_to(resolved)).replace("\\", "/"))
    tool_count = sum(bool(paths) for paths in tools.values())
    accepted = bool(exists and family_evidence and tool_count and not forbidden)
    reasons: list[str] = []
    if not exists:
        reasons.append("path_missing")
    if exists and not family_evidence:
        reasons.append("journey6_family_evidence_missing")
    if exists and not tool_count:
        reasons.append("tool_markers_missing")
    if forbidden:
        reasons.append("forbidden_rdk_or_s100_family_marker")
    return {
        "path": str(resolved),
        "exists": exists,
        "journey6_family_evidence": family_evidence,
        "forbidden_family_evidence": forbidden,
        "tool_markers": tools,
        "accepted": accepted,
        "reasons": reasons,
    }


def discover(roots: Iterable[Path]) -> dict:
    candidates = [inspect_candidate(root) for root in roots]
    accepted = [row for row in candidates if row["accepted"]]
    return {
        "schema_version": 1,
        "target_family": "journey6",
        "target_sku": "auto",
        "target_march": "auto",
        "status": "ready" if accepted else "blocked_external",
        "accepted_sdk_roots": [row["path"] for row in accepted],
        "candidates": candidates,
        "blockers": [] if accepted else [
            {
                "code": "journey6_official_sdk_missing",
                "required": "Official Journey 6 OpenExplorer/OE release package",
                "status": "blocked_external",
            }
        ],
        "truth_boundary": (
            "Discovery does not prove x86 simulation, model compilation, HBM verification, "
            "or board runtime acceptance. RDK S100/S100P/S600 packages are rejected."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", action="append", default=[], help="SDK root to inspect")
    parser.add_argument("--output", default="J6_SDK_DISCOVERY.json")
    args = parser.parse_args()
    configured = list(args.root)
    for env_name in ("J6_SDK_ROOT", "JOURNEY6_SDK_ROOT", "J6_OE_ROOT"):
        value = os.environ.get(env_name)
        if value:
            configured.append(value)
    report = discover(Path(value) for value in dict.fromkeys(configured))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "ready" else 2


if __name__ == "__main__":
    raise SystemExit(main())
