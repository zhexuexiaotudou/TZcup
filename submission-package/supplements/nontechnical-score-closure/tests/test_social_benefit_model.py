from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from social_benefit_model import evaluate_document, evaluate_scenario  # noqa: E402


class SocialBenefitModelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.document = json.loads(
            (ROOT / "data/social-benefit-scenarios.json").read_text(encoding="utf-8")
        )

    def test_document_remains_non_claimable(self) -> None:
        report = evaluate_document(self.document)
        self.assertEqual(report["status"], "MODEL_ONLY_NOT_FIELD_MEASURED")
        self.assertEqual(report["official_points_claimed"], 0.0)
        self.assertTrue(
            all(not item["project_result_claimable"] for item in report["scenarios"])
        )

    def test_base_scenario_arithmetic(self) -> None:
        result = evaluate_scenario(self.document["scenarios"][1])
        self.assertAlmostEqual(result["baseline_person_hours"], 23.0)
        self.assertAlmostEqual(result["autonomous_person_hours"], 9.0)
        self.assertAlmostEqual(result["person_hour_reduction"], 14.0)
        self.assertAlmostEqual(result["energy_delta_kwh"], 6.0)
        self.assertFalse(result["quality_claimable"])

    def test_negative_input_is_rejected(self) -> None:
        scenario = dict(self.document["scenarios"][0])
        scenario["vehicle_kwh_per_day"] = -1
        with self.assertRaises(ValueError):
            evaluate_scenario(scenario)


if __name__ == "__main__":
    unittest.main()
