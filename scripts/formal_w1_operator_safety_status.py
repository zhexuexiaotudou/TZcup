#!/usr/bin/env python3
"""Fail-closed parser for the exact ``ros2 topic echo --once`` String envelope."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml


def load_status(path: Path) -> dict[str, object]:
    documents = list(yaml.safe_load_all(path.read_text(encoding="utf-8")))
    populated = [value for value in documents if value is not None]
    if len(populated) != 1:
        raise ValueError("expected exactly one populated YAML document")
    envelope = populated[0]
    if not isinstance(envelope, dict) or set(envelope) != {"data"}:
        raise ValueError("expected one std_msgs/msg/String data field")
    encoded = envelope["data"]
    if not isinstance(encoded, str):
        raise ValueError("safety String data field is not text")
    value = json.loads(encoded)
    if not isinstance(value, dict):
        raise ValueError("safety JSON payload is not an object")
    return value


def count(value: dict[str, object]) -> int:
    result = value.get("status_publish_count")
    if not isinstance(result, int) or isinstance(result, bool) or result < 0:
        raise ValueError("invalid safety status_publish_count")
    return result


def main() -> None:
    if len(sys.argv) < 3:
        raise SystemExit(2)
    action, path = sys.argv[1], Path(sys.argv[2])
    value = load_status(path)
    if action == "json":
        print(json.dumps(value, sort_keys=True))
        return
    if action == "count":
        print(count(value))
        return
    if action == "require" and len(sys.argv) == 6:
        expected_state, required_reason, previous = sys.argv[3:]
        if value.get("state") != expected_state:
            raise SystemExit(3)
        reasons = value.get("active_reasons")
        if not isinstance(reasons, str) or required_reason not in reasons.split(","):
            raise SystemExit(3)
        if count(value) <= int(previous):
            raise SystemExit(3)
        return
    raise SystemExit(2)


if __name__ == "__main__":
    main()
