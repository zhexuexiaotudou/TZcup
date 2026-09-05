from __future__ import annotations

import json
import subprocess
from pathlib import Path

from audit_locked_repository import audit_repository


ROOT = Path(__file__).resolve().parents[1]


def _git(repo: Path, *arguments: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo), *arguments], text=True
    ).strip()


def _fixture(tmp_path: Path) -> tuple[Path, Path, str]:
    repo = tmp_path / "linorobot2"
    repo.mkdir()
    subprocess.run(["git", "init", "-q", str(repo)], check=True)
    subprocess.run(
        ["git", "-C", str(repo), "config", "user.name", "Test"], check=True
    )
    subprocess.run(
        ["git", "-C", str(repo), "config", "user.email", "test@example.invalid"],
        check=True,
    )
    (repo / "tracked.txt").write_text("locked\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(repo), "add", "tracked.txt"], check=True)
    subprocess.run(["git", "-C", str(repo), "commit", "-qm", "locked"], check=True)
    url = "https://github.com/linorobot/linorobot2.git"
    subprocess.run(["git", "-C", str(repo), "remote", "add", "origin", url], check=True)
    lock_file = tmp_path / "locked_revisions.json"
    lock_file.write_text(
        json.dumps(
            {
                "repositories": {
                    "linorobot2": {
                        "commit": _git(repo, "rev-parse", "HEAD"),
                        "url": url,
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    return repo, lock_file, url


def test_locked_repository_audit_accepts_exact_clean_checkout(tmp_path: Path) -> None:
    repo, lock_file, _ = _fixture(tmp_path)
    report = audit_repository(repo, lock_file, "linorobot2")
    assert report["verified"] is True
    assert report["working_tree_clean"] is True


def test_locked_repository_audit_rejects_dirty_or_untracked_checkout(
    tmp_path: Path,
) -> None:
    repo, lock_file, _ = _fixture(tmp_path)
    (repo / "tracked.txt").write_text("changed\n", encoding="utf-8")
    (repo / "untracked.txt").write_text("new\n", encoding="utf-8")
    report = audit_repository(repo, lock_file, "linorobot2")
    assert report["verified"] is False
    assert report["working_tree_clean"] is False
    assert len(report["status_porcelain"]) == 2


def test_locked_repository_audit_rejects_commit_or_origin_drift(tmp_path: Path) -> None:
    repo, lock_file, _ = _fixture(tmp_path)
    (repo / "tracked.txt").write_text("next\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(repo), "add", "tracked.txt"], check=True)
    subprocess.run(["git", "-C", str(repo), "commit", "-qm", "drift"], check=True)
    subprocess.run(
        ["git", "-C", str(repo), "remote", "set-url", "origin", "https://example.invalid/drift.git"],
        check=True,
    )
    report = audit_repository(repo, lock_file, "linorobot2")
    assert report["verified"] is False
    assert any("commit mismatch" in error for error in report["errors"])
    assert any("origin mismatch" in error for error in report["errors"])


def test_stage1_audits_linorobot2_before_and_after_builds() -> None:
    stage1 = (ROOT / "scripts/stage1_ci.sh").read_text(encoding="utf-8")
    assert 'audit_locked_repository linorobot2 before' in stage1
    assert 'audit_locked_repository linorobot2 after' in stage1
    assert 'linorobot2_state_before.json' in stage1
    assert 'linorobot2_state_after.json' in stage1
