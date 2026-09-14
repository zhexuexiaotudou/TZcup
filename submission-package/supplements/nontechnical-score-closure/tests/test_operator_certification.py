from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from operator_certification import evaluate  # noqa: E402


def row(item: str, category: str, rating: str = "A", critical: str = "false") -> dict[str, str]:
    return {
        "operator_id": "OP-01",
        "day": "3",
        "item_id": item,
        "category": category,
        "rating": rating,
        "prompt_count": "0",
        "critical_error": critical,
        "evidence_path": "evidence/op-01",
    }


class OperatorCertificationTests(unittest.TestCase):
    def test_empty_ledger_is_not_measured(self) -> None:
        self.assertEqual(evaluate([])["status"], "NOT_MEASURED")

    def test_passing_ledger(self) -> None:
        report = evaluate(
            [
                row("SAFE-1", "safety"),
                row("SAFE-2", "safety"),
                row("OPS-1", "operation"),
                row("OPS-2", "operation", rating="B", critical="false"),
            ]
        )
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["operation_ab_rate"], 1.0)

    def test_safety_error_fails(self) -> None:
        report = evaluate(
            [
                row("SAFE-1", "safety", critical="true"),
                row("OPS-1", "operation"),
            ]
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["safety_failures"], ["OP-01:SAFE-1"])


if __name__ == "__main__":
    unittest.main()
