import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = (
    ROOT
    / "artifacts/perception_product_20260809T151411Z/x2/X2_EXTERNAL_ASSET_BLOCKED.json"
)


def test_x2_external_asset_blocker_is_fail_closed_and_does_not_claim_model_failure():
    payload = json.loads(EVIDENCE.read_text(encoding="utf-8"))

    assert payload["decision"] == "BLOCKED_EXTERNAL_NETWORK_ASSET"
    assert len(payload["attempts"]) == 3
    assert payload["external_cache"]["observed_size_bytes"] == 0
    assert payload["external_cache"]["valid_checkpoint"] is False
    assert payload["execution_boundary"]["checkpoint_loaded"] is False
    assert payload["execution_boundary"]["model_inference_started"] is False
    assert payload["execution_boundary"]["performance_failure_claimed"] is False
    assert payload["fixed_gate_changes"] == []
    assert payload["G5_SEALED_FINAL_read"] is False
    assert payload["PERCEPTION_ONLINE_X86_DEV_PASS"] is False
