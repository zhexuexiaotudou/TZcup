#!/usr/bin/env python3
"""Fail when tracked files contain common credential signatures."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PATTERNS = {
    "private_key": re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    "github_token": re.compile(rb"\bgh[opsu]_[A-Za-z0-9_]{30,}\b"),
    "aws_access_key": re.compile(rb"\bAKIA[0-9A-Z]{16}\b"),
}


def main() -> int:
    tracked = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=ROOT,
        check=True,
        capture_output=True,
    ).stdout.split(b"\0")
    findings: list[str] = []
    for raw in tracked:
        if not raw:
            continue
        relative = raw.decode("utf-8", errors="surrogateescape")
        path = ROOT / relative
        if not path.is_file() or path.stat().st_size > 10 * 1024 * 1024:
            continue
        payload = path.read_bytes()
        for name, pattern in PATTERNS.items():
            if pattern.search(payload):
                findings.append(f"{relative}: {name}")
    if findings:
        print("\n".join(findings))
        return 1
    print(f"secret scan passed ({len(tracked) - 1} candidate paths checked)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
