import json
from pathlib import Path
import sys

import yaml


SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

from build_journey6_bundle import build, validate_profile
from j6_board_inventory import collect
from j6_discover_sdk import discover
from j6_validate_sdk import validate


def test_discovery_requires_journey6_evidence_and_rejects_s100(tmp_path):
    valid = tmp_path / "official-journey6-openexplorer"
    valid.mkdir()
    (valid / "hb_compile").write_text("", encoding="utf-8")
    rejected = tmp_path / "journey6-s100-lookalike"
    rejected.mkdir()
    (rejected / "hb_compile").write_text("", encoding="utf-8")
    report = discover([valid, rejected])
    assert report["status"] == "ready"
    assert report["accepted_sdk_roots"] == [str(valid.resolve())]
    assert rejected.resolve().as_posix() not in report["accepted_sdk_roots"]
    assert report["candidates"][1]["reasons"] == ["forbidden_rdk_or_s100_family_marker"]


def test_missing_sdk_is_blocked_external():
    report = discover([])
    result = validate(report)
    assert report["status"] == "blocked_external"
    assert result["sdk_validation_pass"] is False
    assert "journey6_official_sdk_missing" in result["failures"]


def test_toolchain_lock_rejects_wrong_family_source():
    discovery = {
        "target_family": "journey6",
        "accepted_sdk_roots": ["/opt/journey6"],
        "candidates": [],
    }
    lock = {
        "target_family": "journey6",
        "target_sku": "auto",
        "target_march": "nash-e",
        "source": {"package_name": "rdk-s100-openexplorer"},
    }
    result = validate(discovery, lock)
    assert result["status"] == "blocked_external"
    assert "toolchain_lock_references_forbidden_rdk_or_s100_package" in result["failures"]


def test_committed_profiles_are_inventory_gated_and_valid():
    profiles = ROOT / "deploy" / "journey6" / "board_bundle" / "profiles"
    observed = {}
    for path in profiles.glob("*.yaml"):
        profile = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert validate_profile(profile) == []
        observed[profile["profile_id"]] = profile["target_march"]
    assert observed == {
        "journey6_generic": "auto",
        "journey6_nash_e": "nash-e",
        "journey6_nash_m": "nash-m",
        "journey6_nash_p": "nash-p",
    }


def test_bundle_builder_materializes_locked_blocked_skeleton(tmp_path):
    output = tmp_path / "bundle"
    report = build(ROOT / "deploy" / "journey6" / "board_bundle", output, ROOT)
    assert report["status"] == "blocked_external"
    assert report["bundle_ready"] is False
    assert (output / "SHA256SUMS").is_file()
    assert (output / "scripts" / "j6_board_inventory.py").is_file()
    manifest = json.loads((output / "bundle_manifest.json").read_text(encoding="utf-8"))
    assert manifest["target_family"] == "journey6"
    assert manifest["target_sku"] == manifest["target_march"] == "auto"
    for line in (output / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        digest, relative = line.split("  ", 1)
        import hashlib
        assert hashlib.sha256((output / relative).read_bytes()).hexdigest() == digest


def test_non_journey6_host_inventory_never_claims_ready():
    report = collect()
    if not report["board"]["journey6_identity_evidence"]:
        assert report["status"] == "blocked_external"
        assert "journey6_board_identity_not_confirmed" in report["blockers"]


def test_deployment_entrypoints_are_explicit_and_rollback_capable():
    shell = (ROOT / "scripts" / "deploy_journey6.sh").read_text(encoding="utf-8")
    powershell = (ROOT / "scripts" / "deploy_journey6.ps1").read_text(encoding="utf-8")
    installer = (ROOT / "deploy" / "journey6" / "board_bundle" / "scripts" / "install_candidate.sh").read_text(encoding="utf-8")
    assert "--execute" in shell and "[switch]$Execute" in powershell
    assert "sha256sum -c SHA256SUMS" in installer
    assert "last-known-good" in installer
    assert "sanity/warmup/parity command unresolved" in installer
    assert "mv -Tf" in installer
    assert 'write_evidence "rolled_back"' in installer
