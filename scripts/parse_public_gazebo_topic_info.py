"""Strictly validate one ROS 2 topic-info verbose publisher block."""
from __future__ import annotations

from pathlib import Path
import re
import sys

_BLOCK = re.compile(
    r"^Type: (?P<type>[^\r\n]+)\r?$\n"
    r"^Publisher count: 1\r?$\n"
    r"^Publisher GID:\r?$\n"
    r"^[ \t]*(?P<gid>[0-9a-fA-F]{32})[ \t]*\r?$\n"
    r"^Node name: (?P<node>[^\r\n]+)\r?$\n"
    r"^Node namespace: (?P<namespace>[^\r\n]+)\r?$",
    re.MULTILINE,
)

def parse(path: Path, expected_type: str, owner: str) -> None:
    text = path.read_text(encoding="utf-8")
    matches = list(_BLOCK.finditer(text))
    if len(matches) != 1 or text.count("Publisher GID:") != 1:
        raise ValueError("publisher_count_or_blocks_invalid")
    row = matches[0].groupdict()
    if row["type"] != expected_type or row["node"] != owner or row["namespace"] != "/":
        raise ValueError("publisher_identity_invalid")
    if set(row["gid"].lower()) == {"0"}:
        raise ValueError("publisher_gid_invalid")

if __name__ == "__main__":
    try:
        parse(Path(sys.argv[1]), sys.argv[2], sys.argv[3])
    except (IndexError, ValueError) as exc:
        raise SystemExit(str(exc))
