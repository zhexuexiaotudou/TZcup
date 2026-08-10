from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from perception_mrv2_finalize import build_status  # noqa: E402


def test_route_exhaustion_is_fail_closed_and_never_unlocks_freeze():
    a = {"static_decision": {"static_gate_pass": False}}
    b = {"MRV2_B_DETECTOR_PASS": False}
    c = {"static_decision": {"static_gate_pass": False}}
    grounding = {"GROUNDING_DINO_REFERENCE_STATIC_PASS": False}
    status = build_status(a, b, c, grounding, source={"tree": "a" * 40})
    assert status["routes"]["routes_exhausted"] is True
    assert status["MRV2_X86_STATIC_PASS"] is False
    assert status["MODEL_BLOCKED_INTERNAL"] is True
    assert status["MODEL_FREEZE_X86_created"] is False
    assert status["G5_SEALED_FINAL_read"] is False
    assert status["PRODUCT_X86_PERCEPTION_READY"] is False
