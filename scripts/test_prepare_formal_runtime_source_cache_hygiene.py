#!/usr/bin/env python3
"""Static safeguards for the fixed-scope Windows cache hygiene prebuild gate."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "prepare_formal_runtime_source_cache_hygiene.ps1"


def test_hygiene_script_is_fixed_to_the_runtime_source_root() -> None:
    text = SCRIPT.read_text(encoding="utf-8")

    assert "[switch]$Execute" in text
    assert 'Join-Path $repositoryRoot "starter_ws\\src"' in text
    assert "Assert-FixedRepositoryRoot" in text
    assert "Assert-PathWithinSourceRoot" in text
    assert "git -C $RepositoryRoot ls-files --full-name" in text
    assert "[AllowEmptyCollection()][object[]]$Candidates" in text


def test_hygiene_script_limits_targets_and_rejects_reparse_points() -> None:
    text = SCRIPT.read_text(encoding="utf-8")

    assert '$safeEntry.Name -ceq "__pycache__"' in text
    assert '$safeEntry.Name -ceq ".pytest_cache"' in text
    assert '$safeEntry.Extension -ceq ".pyc"' in text
    assert 'if ($parentName -cne "__pycache__")' in text
    assert "Test-ReparsePoint" in text
    assert "Assert-NoReparseDescendants" in text
    assert "Refusing reparse point or symlink" in text


def test_hygiene_script_only_removes_items_inside_explicit_execute_branch() -> None:
    text = SCRIPT.read_text(encoding="utf-8")

    assert "if ($Execute) {" in text
    assert "Remove-Item -LiteralPath $item.FullName -Recurse -Force -ErrorAction Stop" in text
    assert "Remove-Item -LiteralPath $item.FullName -Force -ErrorAction Stop" in text
    assert "deleted_manifest" in text
    assert "FORMAL_RUNTIME_SOURCE_CACHE_HYGIENE_DRY_RUN_READY" in text
