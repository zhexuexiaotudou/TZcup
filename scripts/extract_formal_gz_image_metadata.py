#!/usr/bin/env python3
"""Reduce one gz.msgs.Image text sample to bounded diagnostic metadata."""

from __future__ import annotations

import json
import re
import sys


def extract(text: str) -> dict[str, object]:
    def integer(name: str) -> int:
        match = re.search(rf"(?m)^\s*{re.escape(name)}:\s*(\d+)\s*$", text)
        if match is None:
            raise ValueError(f"gz image sample is missing {name}")
        return int(match.group(1))

    width = integer("width")
    height = integer("height")
    step = integer("step")
    pixel = re.search(r"(?m)^\s*pixel_format_type:\s*([^\s]+)\s*$", text)
    data = re.search(r'(?m)^\s*data:\s*"(.+)"\s*$', text)
    if pixel is None:
        raise ValueError("gz image sample is missing pixel_format_type")
    if data is None:
        raise ValueError("gz image sample has no non-empty data field")
    return {
        "report_id": "tzcup_formal_gz_image_sample_metadata_v1",
        "status": "FORMAL_GZ_IMAGE_SAMPLE_RECEIVED",
        "passed": True,
        "width": width,
        "height": height,
        "step": step,
        "pixel_format_type": pixel.group(1),
        "expected_uncompressed_data_bytes_from_step": step * height,
        "data_field_present_and_nonempty": True,
    }


def main() -> int:
    try:
        payload = extract(sys.stdin.read())
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
