from __future__ import annotations

import csv
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from verify_package import EXPECTED_STATUSES, validate_storyboard  # noqa: E402


class VerifyPackageTests(unittest.TestCase):
    def test_storyboard_is_contiguous(self) -> None:
        with (ROOT / "data/demo-storyboard.csv").open(
            encoding="utf-8-sig", newline=""
        ) as stream:
            rows = list(csv.DictReader(stream))
        errors: list[str] = []
        self.assertEqual(validate_storyboard(rows, errors), 570.0)
        self.assertEqual(errors, [])

    def test_scope_is_fail_closed(self) -> None:
        self.assertEqual(EXPECTED_STATUSES["APP-USE-TRAINING"], "PLANNED_NOT_MEASURED")
        self.assertEqual(
            EXPECTED_STATUSES["APP-USE-RELIABILITY"],
            "INSTRUMENTED_NOT_MEASURED",
        )
        self.assertEqual(
            EXPECTED_STATUSES["BONUS-IP-PAPER"],
            "NOT_ELIGIBLE_NO_FILING_OR_PUBLICATION",
        )


if __name__ == "__main__":
    unittest.main()
