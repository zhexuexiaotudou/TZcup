from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "formal_dry_speed_requalification_token.py"
SPEC = importlib.util.spec_from_file_location("dry_speed_token", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
PROFILE = ROOT / "config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml"


def test_token_is_root_and_stage_bound_and_rejects_another_cap(tmp_path: Path) -> None:
    run_root = tmp_path / "fresh"
    run_root.mkdir()
    token_path = run_root / "token.json"
    token = MODULE.create(
        profile_path=PROFILE, run_root=run_root, output=token_path,
        stage_id="speed_0_70_mps",
    )
    assert token["nonce"]
    assert MODULE.validate(
        profile_path=PROFILE, run_root=run_root, token_path=token_path, requested_cap=0.70
    )["nonce"] == token["nonce"]
    with pytest.raises(ValueError, match="requested non-default cap"):
        MODULE.validate(
            profile_path=PROFILE, run_root=run_root, token_path=token_path, requested_cap=0.8
        )
    other_root = tmp_path / "other"
    other_root.mkdir()
    with pytest.raises(ValueError, match="does not bind this requalification run root"):
        MODULE.validate(
            profile_path=PROFILE, run_root=other_root, token_path=token_path, requested_cap=0.70
        )
