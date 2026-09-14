import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = ROOT / "artifacts/perception_gpu_recovery_20260914"


def test_gpu_recovery_receipt_binds_model_and_frozen_metrics():
    receipt = json.loads((ARTIFACT_ROOT / "receipt.json").read_text(encoding="utf-8"))
    model = ARTIFACT_ROOT / receipt["model"]["path"]
    assert hashlib.sha256(model.read_bytes()).hexdigest() == receipt["model"]["sha256"]
    assert receipt["raw_replay"]["baseline_tp_fp_fn"] == [33, 50, 43]
    assert receipt["raw_replay"]["after_tp_fp_fn"] == [41, 0, 35]
    assert receipt["policy_replay"]["baseline_tp_fp_fn"] == [0, 0, 76]
    assert receipt["policy_replay"]["after_tp_fp_fn"] == [33, 0, 43]
    assert receipt["dataset_audit"]["training_or_validation_use_of_holdout"] is False
    assert receipt["gpu_inference_parity"]["model_sha256"] == receipt["model"]["sha256"]
    assert receipt["gpu_inference_parity"]["argmax_agreement"] == 1.0
    assert receipt["gpu_inference_parity"]["onnx_cpu_vs_torch_gpu_max_abs_diff"] <= 1e-4
    assert receipt["claim_boundary"]["official_competition_R01"] == "NOT_MEASURED"
    assert receipt["claim_boundary"]["ninety_five_percent_claim_supported"] is False


def test_gpu_recovery_failure_ledger_is_retained():
    failures = json.loads(
        (ARTIFACT_ROOT / "failure_ledger.json").read_text(encoding="utf-8")
    )
    assert len(failures["failures"]) >= 6
    assert all(
        item.get("resolution") or item.get("fallback")
        for item in failures["failures"]
    )
