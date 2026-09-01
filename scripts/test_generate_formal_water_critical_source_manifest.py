from __future__ import annotations

import json
from pathlib import Path

from generate_formal_water_critical_source_manifest import CRITICAL_FILES, generate


ROOT = Path(__file__).resolve().parents[1]


def test_typed_and_gz_transport_patch_sources_are_critical() -> None:
    assert "scripts/run_formal_typed_cleaning_motor_diagnostic.sh" in CRITICAL_FILES
    assert "scripts/collect_formal_typed_cleaning_motor_diagnostic.py" in CRITICAL_FILES
    assert (
        "patches/upstream/gz_transport13/0001-publish-retry-current-frame-on-eintr.patch"
        in CRITICAL_FILES
    )
    assert "patches/upstream/gz_transport13/manifest.json" in CRITICAL_FILES


def test_generate_binds_source_and_regular_copy_install(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    workspace = tmp_path / "workspace"
    (workspace / "src").mkdir(parents=True)
    (workspace / "install").mkdir(parents=True)
    (workspace / "INSTALL_SYMLINKS.txt").write_text("", encoding="utf-8")
    for relative in CRITICAL_FILES:
        source = repo / relative
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(relative + "\n", encoding="utf-8")
        if relative.startswith("starter_ws/"):
            frozen = workspace / relative.removeprefix("starter_ws/")
            frozen.parent.mkdir(parents=True, exist_ok=True)
            frozen.write_text(relative + "\n", encoding="utf-8")
    report = generate(repo, workspace)
    assert report["source_package_files_match_frozen_copy"] is True
    assert report["frozen_source_symlink_count"] == 0
    assert report["install_symlink_report_matches_scan"] is True
    assert report["install_symlink_count"] == 0
    paths = {row["path"] for row in report["critical_files"]}
    assert paths == set(CRITICAL_FILES)


def test_generator_cli_refuses_stale_output_contract_is_present() -> None:
    source = (ROOT / "scripts/generate_formal_water_critical_source_manifest.py").read_text(
        encoding="utf-8"
    )
    assert "if args.output.exists() or args.output.is_symlink():" in source
    assert "refusing stale output" in source
    assert ".pending." in source


def test_generate_rejects_inaccurate_install_symlink_report(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    workspace = tmp_path / "workspace"
    (workspace / "src").mkdir(parents=True)
    (workspace / "install").mkdir(parents=True)
    (workspace / "INSTALL_SYMLINKS.txt").write_text(
        "lib/not-actually-a-link.so\n", encoding="utf-8"
    )
    for relative in CRITICAL_FILES:
        source = repo / relative
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(relative + "\n", encoding="utf-8")
        if relative.startswith("starter_ws/"):
            frozen = workspace / relative.removeprefix("starter_ws/")
            frozen.parent.mkdir(parents=True, exist_ok=True)
            frozen.write_text(relative + "\n", encoding="utf-8")
    try:
        generate(repo, workspace)
    except ValueError as exc:
        assert "does not match the merged install tree" in str(exc)
    else:  # pragma: no cover - fail-closed assertion
        raise AssertionError("inaccurate INSTALL_SYMLINKS.txt was accepted")
