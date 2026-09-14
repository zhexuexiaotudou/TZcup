from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from validate_demo_media import extract_media_info, parse_fraction, validate_media  # noqa: E402


class DemoMediaValidatorTests(unittest.TestCase):
    def test_fraction_parser(self) -> None:
        self.assertEqual(parse_fraction("30000/1001"), 30000 / 1001)
        self.assertEqual(parse_fraction("0/0"), 0.0)

    def test_valid_media_shape(self) -> None:
        probe = {
            "streams": [
                {
                    "codec_type": "video",
                    "codec_name": "h264",
                    "width": 1920,
                    "height": 1080,
                    "avg_frame_rate": "30/1",
                },
                {"codec_type": "audio", "codec_name": "aac"},
            ],
            "format": {"duration": "570.0"},
        }
        info = extract_media_info(probe)
        self.assertEqual(validate_media(info), [])

    def test_partial_clip_fails_duration_and_audio(self) -> None:
        probe = {
            "streams": [
                {
                    "codec_type": "video",
                    "codec_name": "h264",
                    "width": 1920,
                    "height": 1080,
                    "avg_frame_rate": "30/1",
                }
            ],
            "format": {"duration": "6.333333"},
        }
        errors = validate_media(extract_media_info(probe))
        self.assertTrue(any("duration" in error for error in errors))
        self.assertTrue(any("audio" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
