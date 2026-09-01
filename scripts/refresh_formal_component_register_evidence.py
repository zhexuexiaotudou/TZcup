#!/usr/bin/env python3
"""Re-run the component-register gate without changing snapshot bytes."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from generate_formal_vehicle_snapshot import _json_bytes
from validate_formal_vehicle_component_register import validate


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "reports/engineering/formal_competition_vehicle.urdf"
DEFAULT_OUTPUT = ROOT / "reports/engineering/formal_vehicle_component_register_report.json"


def refresh(urdf: Path, output: Path) -> dict[str, object]:
    """Validate the frozen URDF and atomically publish canonical JSON evidence."""
    result = validate(urdf_path=urdf)
    payload = _json_bytes(result)
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    pending.write_bytes(payload)
    pending.replace(output)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = refresh(args.urdf, args.output)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
