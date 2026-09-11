import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "prepare_competition_sim_only_evidence.py"


def test_help_requires_machine_generated_probes():
    result = subprocess.run([sys.executable, str(SCRIPT), "--help"], check=True, capture_output=True, text=True)
    for token in ("--repo", "--board-probe", "--remote-probe", "--output"):
        assert token in result.stdout
    assert "--remote-head" not in result.stdout
    assert "--board-serial" not in result.stdout


def test_profile_separates_evidence_readiness_from_official_acceptance():
    profile = json.loads((ROOT / "config" / "competition_sim_only_v1.json").read_text(encoding="utf-8"))
    assert profile["claim"] == "COMPETITION_SIM_ONLY_EVIDENCE_READY"
    assert "not organizer recognition" in profile["claim_boundary"]
    assert profile["video_required_for_current_execution"] is False
    assert set(profile["board_algorithm_modules"]) == {"perception", "mapping_localization", "planning_decision", "control"}


def test_prepare_rejects_non_probe_json(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    subprocess.run(["git", "init", str(repo)], check=True, capture_output=True)
    subprocess.run(["git", "-C", str(repo), "config", "user.email", "test@example.invalid"], check=True)
    subprocess.run(["git", "-C", str(repo), "config", "user.name", "Test"], check=True)
    (repo / "seed.txt").write_text("seed\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(repo), "add", "seed.txt"], check=True)
    subprocess.run(["git", "-C", str(repo), "commit", "-m", "seed"], check=True, capture_output=True)
    fake = tmp_path / "fake.json"
    fake.write_text("{}\n", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--repo", str(repo), "--board-probe", str(fake),
         "--remote-probe", str(fake), "--output", str(tmp_path / "out")],
        capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "wrong probe identity" in result.stderr
