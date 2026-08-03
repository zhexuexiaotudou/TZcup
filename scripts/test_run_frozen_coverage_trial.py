from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_frozen_trial_is_isolated_and_uses_the_real_small_mission():
    runner = (ROOT / "scripts/run_frozen_coverage_trial.ps1").read_text(
        encoding="utf-8"
    )
    assert '"ROS_DOMAIN_ID=42"' in runner
    assert '"GZ_PARTITION=$partition"' in runner
    assert '"--map-size", "small"' in runner
    assert '"--no-gui"' in runner
    assert '"--timeout", "300"' in runner
    assert "coverage_optimizer_$Tag" in runner
