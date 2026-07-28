#!/usr/bin/env python3
"""Guarded GitHub PR/merge adapter used by the autonomous controller."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(argv: list[str], *, capture: bool = False) -> str:
    result = subprocess.run(argv, cwd=ROOT, check=True, text=True, capture_output=capture)
    return result.stdout.strip() if capture else ""


def require_clean_task_branch() -> str:
    branch = run(["git", "branch", "--show-current"], capture=True)
    if not branch.startswith(("agent/", "codex/")):
        raise RuntimeError(f"refusing automation from non-task branch: {branch}")
    if run(["git", "status", "--porcelain"], capture=True):
        raise RuntimeError("refusing automation with a dirty worktree")
    return branch


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("push", "open-pr", "merge"))
    parser.add_argument("--title")
    parser.add_argument("--body-file", type=Path)
    parser.add_argument("--pr", type=int)
    parser.add_argument("--execute", action="store_true", help="required mutation guard")
    args = parser.parse_args()
    branch = require_clean_task_branch()
    if not args.execute:
        print(json.dumps({"dry_run": True, "action": args.action, "branch": branch}))
        return 0
    if args.action == "push":
        run(["git", "push", "-u", "origin", branch])
    elif args.action == "open-pr":
        if not args.title or not args.body_file:
            parser.error("open-pr requires --title and --body-file")
        run(["gh", "pr", "create", "--base", "main", "--head", branch, "--title", args.title, "--body-file", str(args.body_file)])
    else:
        if args.pr is None:
            parser.error("merge requires --pr")
        checks = run(["gh", "pr", "checks", str(args.pr), "--required"], capture=True)
        if "fail" in checks.lower() or "pending" in checks.lower():
            raise RuntimeError("required PR checks are not green")
        run(["gh", "pr", "merge", str(args.pr), "--merge", "--delete-branch=false"])
    print(json.dumps({"ok": True, "action": args.action, "branch": branch}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
