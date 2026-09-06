#!/usr/bin/env python3
"""Turn one live safety-status JSON sample into immutable speed evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-cap", type=float, required=True)
    parser.add_argument("--expected-profile", required=True)
    parser.add_argument("--expected-state", required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("refusing to overwrite safety-manager readback")
    try:
        status = json.loads(args.raw.read_text(encoding="utf-8").strip())
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SystemExit(f"invalid safety status sample: {exc}") from exc
    if not isinstance(status, dict):
        raise SystemExit("safety status sample must be a JSON object")
    cap = status.get("effective_max_linear_velocity_mps")
    if isinstance(cap, bool) or not isinstance(cap, (int, float)) or not math.isclose(
        float(cap), args.expected_cap, abs_tol=1.0e-12
    ):
        raise SystemExit("safety manager effective cap differs from the required live cap")
    if status.get("operation_speed_profile") != args.expected_profile:
        raise SystemExit("safety manager profile readback differs from launch scope")
    if status.get("speed_qualification_state") != args.expected_state:
        raise SystemExit("safety manager qualification state differs from launch scope")
    payload = {
        "schema_version": 1,
        "captured_epoch_ns": time.time_ns(),
        "raw_status_sha256": hashlib.sha256(args.raw.read_bytes()).hexdigest(),
        "effective_max_linear_velocity_mps": float(cap),
        "operation_speed_profile": status["operation_speed_profile"],
        "speed_qualification_state": status["speed_qualification_state"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
