from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _module():
    path = ROOT / "scripts/audit_crv6_checkpoint_recovery.py"
    spec = importlib.util.spec_from_file_location("crv6_recovery", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_missing_historical_checkpoint_selects_r1_when_initialization_exists(tmp_path: Path):
    module = _module()
    initialization = tmp_path / "initial.pth"
    initialization.write_bytes(b"initialization")
    original = module.INITIALIZATION_SHA256
    module.INITIALIZATION_SHA256 = module.sha256(initialization)
    try:
        report = module.build_report([tmp_path], repository="unused", distribution="unused", include_external=False)
    finally:
        module.INITIALIZATION_SHA256 = original
    assert report["exact_checkpoint_recovered"] is False
    assert report["HISTORICAL_D1B_CHECKPOINT_LOST"] is True
    assert report["D1_B_initialization"]["available"] is True
    assert report["next_route"] == "R1"
    assert report["recovery_search_closed"] is True


def test_exact_checkpoint_is_identified_by_bytes_not_filename(tmp_path: Path):
    module = _module()
    candidate = tmp_path / "unrelated-name.pth"
    candidate.write_bytes(b"historical")
    original = module.TARGET_SHA256
    module.TARGET_SHA256 = module.sha256(candidate)
    try:
        report = module.build_report([tmp_path], repository="unused", distribution="unused", include_external=False)
    finally:
        module.TARGET_SHA256 = original
    assert report["exact_checkpoint_recovered"] is True
    assert report["HISTORICAL_D1B_CHECKPOINT_LOST"] is False
    assert report["next_route"] == "HISTORICAL_RECOVERED"


def test_exact_checkpoint_can_be_found_inside_zip(tmp_path: Path):
    import zipfile

    module = _module()
    archive = tmp_path / "old-results.zip"
    with zipfile.ZipFile(archive, "w") as bundle:
        bundle.writestr("nested/renamed.pth", b"historical-in-archive")
    original = module.TARGET_SHA256
    module.TARGET_SHA256 = module.hashlib.sha256(b"historical-in-archive").hexdigest()
    try:
        report = module.build_report([tmp_path], repository="unused", distribution="unused", include_external=False)
    finally:
        module.TARGET_SHA256 = original
    assert report["exact_checkpoint_recovered"] is True
    assert report["exact_checkpoint_path"].endswith("old-results.zip::nested/renamed.pth")
