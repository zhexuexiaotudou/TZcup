import hashlib
import json
from pathlib import Path

from perception_mrv2_baseline import asset, emit


def test_asset_rejects_zero_byte_checkpoint(tmp_path: Path):
    path = tmp_path / "checkpoint.pth"
    path.write_bytes(b"")
    record = asset(path, "test")
    assert record["present"] is True
    assert record["bytes"] == 0
    assert record["sha256"] is None
    assert record["valid_nonempty_file"] is False


def test_emit_hashes_exact_lf_git_ready_bytes(tmp_path: Path):
    emit(tmp_path, {"BASELINE.json": {"NEW_MODEL_RECOVERY_V2_AUTHORIZED": True}})
    payload = json.loads((tmp_path / "artifact_manifest.json").read_text(encoding="utf-8"))
    record = payload["files"][0]
    data = (tmp_path / record["path"]).read_bytes()
    assert b"\r\n" not in data
    assert record["sha256"] == hashlib.sha256(data).hexdigest()
