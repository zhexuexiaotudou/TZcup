from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from reliability_qualification import (  # noqa: E402
    binomial_cdf,
    clopper_pearson_upper_bound,
    evaluate_vehicle_days,
    minimum_days_for_failures,
    required_zero_failure_days,
)


class ReliabilityQualificationTests(unittest.TestCase):
    def test_cdf_is_monotonic(self) -> None:
        self.assertLess(binomial_cdf(0, 299, 0.005), binomial_cdf(1, 299, 0.005))
        self.assertAlmostEqual(binomial_cdf(10, 10, 0.3), 1.0, places=12)

    def test_zero_failure_qualification(self) -> None:
        self.assertEqual(required_zero_failure_days(0.95, 0.01), 299)
        self.assertLess(clopper_pearson_upper_bound(0, 299, 0.95), 0.01)
        self.assertGreater(clopper_pearson_upper_bound(0, 298, 0.95), 0.01)

    def test_failure_count_sample_sizes(self) -> None:
        self.assertEqual(minimum_days_for_failures(1), 473)
        self.assertEqual(minimum_days_for_failures(2), 628)

    def test_vehicle_day_ledger(self) -> None:
        report = evaluate_vehicle_days(
            [
                {
                    "vehicle_id": "v1",
                    "service_day": "2026-09-01",
                    "primary_fault": "false",
                },
                {
                    "vehicle_id": "v1",
                    "service_day": "2026-09-01",
                    "primary_fault": "false",
                },
                {
                    "vehicle_id": "v2",
                    "service_day": "2026-09-01",
                    "primary_fault": "true",
                },
            ]
        )
        self.assertEqual(report["vehicle_days"], 2)
        self.assertEqual(report["primary_failures"], 1)
        self.assertFalse(report["target_met"])


if __name__ == "__main__":
    unittest.main()
