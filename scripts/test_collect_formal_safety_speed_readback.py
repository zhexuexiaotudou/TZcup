import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COLLECTOR = ROOT / "scripts" / "collect_formal_safety_speed_readback.py"


def _run(tmp_path: Path, status: dict, expected: str = "1.0") -> subprocess.CompletedProcess[str]:
    raw = tmp_path / "status.json"
    output = tmp_path / "readback.json"
    raw.write_text(json.dumps(status), encoding="utf-8")
    return subprocess.run(
        [
            sys.executable, str(COLLECTOR), "--raw", str(raw), "--output", str(output),
            "--expected-cap", expected,
            "--expected-profile", "dry_cleaning_competition_candidate",
            "--expected-state", "isolated_same_map_dry_coverage",
        ],
        text=True,
        capture_output=True,
        check=False,
    )


def test_collector_accepts_only_live_exact_speed_scope(tmp_path: Path) -> None:
    result = _run(tmp_path, {
        "effective_max_linear_velocity_mps": 1.0,
        "operation_speed_profile": "dry_cleaning_competition_candidate",
        "speed_qualification_state": "isolated_same_map_dry_coverage",
    })
    assert result.returncode == 0, result.stderr
    assert json.loads((tmp_path / "readback.json").read_text())[
        "effective_max_linear_velocity_mps"
    ] == 1.0


def test_collector_rejects_hand_written_or_unscoped_cap(tmp_path: Path) -> None:
    result = _run(tmp_path, {
        "effective_max_linear_velocity_mps": 1.0,
        "operation_speed_profile": "dry_cleaning_competition_candidate",
        "speed_qualification_state": "none",
    })
    assert result.returncode != 0
    assert not (tmp_path / "readback.json").exists()
