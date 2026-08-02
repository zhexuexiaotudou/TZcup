#!/usr/bin/env python3
"""Create a Gazebo GUI config whose camera is fully operator-controlled."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


CAMERA_TRACKING_PLUGIN = re.compile(
    r"\s*<plugin\s+filename=\"CameraTracking\"[^>]*>.*?</plugin>\s*",
    flags=re.DOTALL,
)


def remove_camera_tracking(config: str) -> str:
    """Remove exactly one CameraTracking plugin and retain valid line breaks."""
    updated, count = CAMERA_TRACKING_PLUGIN.subn("\n", config)
    if count != 1:
        raise ValueError(f"expected exactly one CameraTracking plugin, found {count}")
    return updated.rstrip() + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source = args.input.read_text(encoding="utf-8")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(remove_camera_tracking(source), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
