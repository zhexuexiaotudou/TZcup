from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from auto16_release import blockers_from_state, evidence_index


ROOT = Path(__file__).resolve().parents[1]


def test_final_blockers_include_required_truth_boundaries() -> None:
    state = json.loads(
        (ROOT / "AUTONOMOUS_STATE.json").read_text(encoding="utf-8")
    )
    blockers = {item["stage"] for item in blockers_from_state(state)}
    assert {"AUTO-05", "AUTO-08", "AUTO-13", "AUTO-14", "AUTO-15"} <= blockers


def test_evidence_index_does_not_promote_blocked_stages() -> None:
    state = json.loads(
        (ROOT / "AUTONOMOUS_STATE.json").read_text(encoding="utf-8")
    )
    index = evidence_index(state)
    assert "| AUTO-15 | BLOCKED |" in index
    assert "不把低等级证据外推" in index


def test_all_ament_python_packages_export_the_build_type() -> None:
    for package_xml in sorted((ROOT / "starter_ws/src").glob("*/package.xml")):
        package = ET.parse(package_xml).getroot()
        buildtools = {
            node.text for node in package.findall("buildtool_depend")
        }
        if "ament_python" not in buildtools:
            continue
        export = package.find("export")
        assert export is not None, package_xml
        assert export.findtext("build_type") == "ament_python", package_xml


def test_all_ament_python_packages_declare_pytest_discovery() -> None:
    for setup_py in sorted((ROOT / "starter_ws/src").glob("*/setup.py")):
        payload = setup_py.read_text(encoding="utf-8")
        assert 'tests_require=["pytest"]' in payload, setup_py
