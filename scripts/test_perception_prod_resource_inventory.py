import hashlib
from pathlib import Path

from perception_prod_resource_inventory import emit, sha256


def test_sha256_streams_exact_bytes(tmp_path: Path):
    payload = b"tzcup-perception-prod-00\n"
    path = tmp_path / "asset.bin"
    path.write_bytes(payload)
    assert sha256(path) == hashlib.sha256(payload).hexdigest()


def test_emit_writes_inventory_and_manifest(tmp_path: Path):
    payloads = {f"item_{index}.json": {"index": index} for index in range(8)}
    emit(tmp_path, payloads)
    assert all((tmp_path / name).is_file() for name in payloads)
    manifest = (tmp_path / "artifact_manifest.json").read_text(encoding="utf-8")
    assert '"all_inventory_files_present": true' in manifest
    assert manifest.count('"sha256"') == 8
