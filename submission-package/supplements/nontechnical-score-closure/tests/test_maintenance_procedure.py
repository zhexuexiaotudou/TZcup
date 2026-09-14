from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from validate_maintenance_procedure import validate  # noqa: E402


class MaintenanceProcedureTests(unittest.TestCase):
    def setUp(self) -> None:
        self.procedure = json.loads(
            (ROOT / "data/maintenance-procedure.json").read_text(encoding="utf-8")
        )

    def test_frozen_procedure_passes(self) -> None:
        self.assertEqual(validate(self.procedure), [])

    def test_extra_step_is_rejected(self) -> None:
        candidate = copy.deepcopy(self.procedure)
        candidate["procedures"][0]["steps"].append({"number": 4, "action": "extra"})
        self.assertTrue(validate(candidate))

    def test_professional_tool_is_rejected(self) -> None:
        candidate = copy.deepcopy(self.procedure)
        candidate["required_tools"] = ["wrench"]
        self.assertTrue(validate(candidate))

    def test_physical_pass_without_measurement_is_rejected(self) -> None:
        candidate = copy.deepcopy(self.procedure)
        candidate["physical_acceptance"][0]["status"] = "PASS"
        self.assertTrue(validate(candidate))


if __name__ == "__main__":
    unittest.main()
