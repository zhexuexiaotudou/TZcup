#!/usr/bin/env python3
"""Validate autonomous registry, state and generated run-plan invariants."""

from __future__ import annotations

import json

from autonomous_runner import PLAN_PATH, STATE_PATH, build_plan, load_json, load_registry, validate_registry, validate_state


def main() -> int:
    registry = load_registry()
    state = load_json(STATE_PATH)
    errors = validate_registry(registry) + validate_state(state, registry)
    if load_json(PLAN_PATH) != build_plan(registry):
        errors.append("AUTONOMOUS_RUN_PLAN.json differs from the registry-derived plan")
    print(json.dumps({"valid": not errors, "errors": errors}, ensure_ascii=False, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
