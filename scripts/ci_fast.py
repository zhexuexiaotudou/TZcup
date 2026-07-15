#!/usr/bin/env python3
"""Fast, ROS-independent validation used by the 开发工作流 CI gate."""

from __future__ import annotations

import compileall
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROOT / "starter_ws" / "src"


def require_project_files() -> None:
    required = (
        ROOT / "AGENTS.md",
        ROOT / "README.md",
        ROOT / "README_FIRST.md",
        ROOT / "PROJECT_SPEC.md",
        ROOT / "STAGE_GATES.md",
        ROOT / "docs" / "development-workflow.md",
        ROOT / ".github" / "workflows" / "development-workflow.yml",
    )
    missing = [str(path.relative_to(ROOT)) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(f"missing required project files: {', '.join(missing)}")


def validate_python() -> None:
    targets = (SOURCE_ROOT, ROOT / "scripts")
    failed = [str(path) for path in targets if not compileall.compile_dir(path, quiet=1)]
    if failed:
        raise RuntimeError(f"Python compilation failed under: {', '.join(failed)}")


def validate_structured_files() -> None:
    for path in sorted(SOURCE_ROOT.rglob("*.json")):
        json.loads(path.read_text(encoding="utf-8"))

    for pattern in ("*.yaml", "*.yml"):
        for path in sorted(SOURCE_ROOT.rglob(pattern)):
            yaml.safe_load(path.read_text(encoding="utf-8"))

    xml_patterns = ("package.xml", "*.xacro", "*.sdf", "*.urdf")
    seen: set[Path] = set()
    for pattern in xml_patterns:
        for path in sorted(SOURCE_ROOT.rglob(pattern)):
            if path not in seen:
                ET.parse(path)
                seen.add(path)


def run_ros_independent_tests() -> None:
    coverage_package = SOURCE_ROOT / "sanitation_coverage"
    sys.path.insert(0, str(coverage_package))
    test_path = coverage_package / "test" / "test_metrics.py"
    result = pytest.main(["-q", str(test_path)])
    if result != pytest.ExitCode.OK:
        raise RuntimeError(f"ROS-independent pytest gate failed with exit code {int(result)}")


def main() -> int:
    require_project_files()
    validate_python()
    validate_structured_files()
    run_ros_independent_tests()
    print("development workflow fast validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
