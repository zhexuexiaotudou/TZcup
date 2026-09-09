"""Read board runtime identities from one already-sourced shell; never installs."""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
from pathlib import Path
from typing import Any, Callable, Mapping


PYTHON_IMPORTS = ("numpy", "cv2", "yaml", "rclpy", "cv_bridge", "ai_msgs", "sensor_msgs", "vision_msgs", "tf2_ros", "sanitation_perception")


def collect_python_imports(
    importer: Callable[[str], Any] = importlib.import_module,
    distribution_version: Callable[[str], str] = importlib.metadata.version,
    distribution_packages: Callable[[], Mapping[str, list[str]]] = importlib.metadata.packages_distributions,
) -> dict[str, dict[str, str | None]]:
    """Return exact module version/path, failing before a partial receipt exists."""
    rows: dict[str, dict[str, str | None]] = {}
    for name in PYTHON_IMPORTS:
        module = importer(name)
        path = getattr(module, "__file__", None)
        if not isinstance(path, str) or not path.startswith("/"):
            raise RuntimeError(f"sourced Python import lacks absolute path: {name}")
        try:
            version = distribution_version(name)
        except importlib.metadata.PackageNotFoundError:
            version = None
            for distribution in distribution_packages().get(name, ()):
                try:
                    version = distribution_version(distribution)
                    break
                except importlib.metadata.PackageNotFoundError:
                    continue
            if version is None:
                version = getattr(module, "__version__", None)
        if version is not None and (not isinstance(version, str) or not version):
            raise RuntimeError(f"sourced Python import has invalid version: {name}")
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
