"""Read board runtime identities from one already-sourced shell; never installs."""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
from typing import Any, Callable


PYTHON_IMPORTS = ("numpy", "cv2", "yaml", "rclpy", "cv_bridge", "ai_msgs", "sensor_msgs", "vision_msgs", "tf2_ros", "sanitation_perception")


def collect_python_imports(importer: Callable[[str], Any] = importlib.import_module) -> dict[str, dict[str, str]]:
    """Return exact module version/path, failing before a partial receipt exists."""
    rows: dict[str, dict[str, str]] = {}
    for name in PYTHON_IMPORTS:
        module = importer(name)
        path = getattr(module, "__file__", None)
        version = getattr(module, "__version__", None)
        if not isinstance(path, str) or not path.startswith("/") or not isinstance(version, str) or not version:
            raise RuntimeError(f"sourced Python import lacks version/path: {name}")
        rows[name] = {"version": version, "module_path": path}
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sourced-shell-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.sourced_shell_id.strip() or args.output.exists():
        raise SystemExit("require a non-empty sourced shell id and a fresh output path")
    imports = collect_python_imports()
    for row in imports.values():
        row["sourced_shell_id"] = args.sourced_shell_id
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"schema_version": 1, "sourced_shell_id": args.sourced_shell_id, "python_imports": imports}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
