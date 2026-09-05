import hashlib
import json
from pathlib import Path

from refresh_formal_component_register_evidence import refresh


ROOT = Path(__file__).resolve().parents[1]


def test_refresh_matches_the_frozen_snapshot_inventory(tmp_path: Path) -> None:
    output = tmp_path / "component.json"
    result = refresh(
        ROOT / "reports/engineering/formal_competition_vehicle.urdf", output
    )
    manifest = json.loads(
        (ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    expected = manifest["outputs"][
        "reports/engineering/formal_vehicle_component_register_report.json"
    ]
    assert result["status"] == "COMPONENT_REGISTER_URDF_FK_AND_INTERFACES_VALID"
    assert hashlib.sha256(output.read_bytes()).hexdigest() == expected["sha256"]
    assert output.stat().st_size == expected["size_bytes"]
