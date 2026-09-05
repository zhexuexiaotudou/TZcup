from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from validate_s100p_mechanical_electrical_evidence import DEFAULT, ROOT, validate


def test_current_evidence_is_valid_and_fail_closed() -> None:
    payload = json.loads(DEFAULT.read_text(encoding="utf-8"))
    validate(payload, ROOT)
    assert payload["status"] == "BLOCKED_MECHANICAL_ELECTRICAL_INTEGRATION"
    assert payload["acceptance"] == {
        "urdf_update_authorized": False,
        "mechanical_installation_accepted": False,
        "electrical_installation_accepted": False,
        "runtime_accepted": False,
    }


def test_cannot_promote_official_dimensions_to_urdf_freeze() -> None:
    payload = json.loads(DEFAULT.read_text(encoding="utf-8"))
    item = next(item for item in payload["evidence_items"] if item["id"] == "acrylic_enclosure_nominal_dimensions")
    item["can_freeze_urdf"] = True
    with pytest.raises(ValueError, match="must not authorize a URDF freeze"):
        validate(payload, ROOT)


def test_evidence_level_cannot_be_detached_from_its_source() -> None:
    payload = json.loads(DEFAULT.read_text(encoding="utf-8"))
    item = next(item for item in payload["evidence_items"] if item["id"] == "j1_rated_input")
    item["evidence_level"] = "LOCAL_READ_ONLY_IDENTITY_ARTIFACT"
    with pytest.raises(ValueError, match="evidence level must match"):
        validate(payload, ROOT)


def test_cannot_drop_power_on_or_unmeasured_interface_blockers() -> None:
    payload = json.loads(DEFAULT.read_text(encoding="utf-8"))
    payload["blocked_gates"].remove("installed_power_on_and_runtime_validation")
    with pytest.raises(ValueError, match="critical installation/power blockers"):
        validate(payload, ROOT)

    payload = json.loads(DEFAULT.read_text(encoding="utf-8"))
    item = next(item for item in payload["evidence_items"] if item["id"] == "connector_model_table")
    item["blocked_by"].remove("connector_coordinates")
    with pytest.raises(ValueError, match="connector model names"):
        validate(payload, ROOT)


def test_cli_is_offline_and_machine_readable() -> None:
    completed = subprocess.run(
        [sys.executable, str(ROOT / "scripts/validate_s100p_mechanical_electrical_evidence.py"), "--root", str(ROOT), "--evidence", str(DEFAULT)],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "valid fail-closed" in completed.stdout
