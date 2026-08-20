"""Best-effort atomic snapshot writes across Linux and mounted Windows paths."""

from __future__ import annotations

from pathlib import Path
import time


def write_text_snapshot(
    target: Path,
    content: str,
    *,
    max_attempts: int = 5,
    retry_delay_sec: float = 0.02,
) -> bool:
    """Atomically replace *target*, retrying transient Windows share locks."""
    attempts = max(1, int(max_attempts))
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    for attempt in range(attempts):
        try:
            temporary.replace(target)
            return True
        except PermissionError:
            if attempt + 1 >= attempts:
                return False
            time.sleep(float(retry_delay_sec) * (attempt + 1))
    return False
