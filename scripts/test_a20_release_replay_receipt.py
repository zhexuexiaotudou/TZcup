import json
import os
from pathlib import Path

import pytest

import a20_release_replay_receipt as receipt_module
from a20_release_replay_receipt import (
    BLOCKER,
    _open_bound_input,
    _open_root_directory,
    _output_in_root,
    _read_bound_json,
    _regular_in_root,
    _write_fresh_output,
    validate_receipt,
)


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json"


def test_a20_is_blocked_without_a_canonical_formal_product_replay_producer() -> None:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    report = validate_receipt({"handwritten": "PASS"})

    assert contract["canonical_product_replay_producer"] is None
    assert report["valid"] is False
    assert report["status"] == "A20_RECEIPT_STATIC_BLOCKED"
    assert report["errors"] == [BLOCKER]
    assert report["release_runtime_pass"] is False


def test_cli_paths_reject_escape_symlink_and_existing_output(tmp_path: Path) -> None:
    root = tmp_path / "root"
    root.mkdir()
    receipt = root / "receipt.json"
    receipt.write_text("{}", encoding="utf-8")
    assert _regular_in_root(root, receipt, "receipt") == receipt

    with pytest.raises(ValueError, match="escapes"):
        _regular_in_root(root, tmp_path / "outside.json", "receipt")
    with pytest.raises(ValueError, match="escapes"):
        _regular_in_root(root, root / ".." / "outside.json", "receipt")
    with pytest.raises(ValueError, match="fresh"):
        _output_in_root(root, receipt)
    try:
        link = root / "link.json"
        link.symlink_to(receipt)
    except OSError as exc:  # pragma: no cover - Windows without symlink privilege
        pytest.skip(f"symbolic links unavailable: {exc}")
    with pytest.raises(ValueError, match="symbolic-link"):
        _regular_in_root(root, link, "receipt")
    directory = root / "real"
    directory.mkdir()
    (directory / "nested.json").write_text("{}", encoding="utf-8")
    linked_directory = root / "linked"
    linked_directory.symlink_to(directory, target_is_directory=True)
    with pytest.raises(ValueError, match="symbolic-link ancestor"):
        _regular_in_root(root, linked_directory / "nested.json", "receipt")


def test_secure_output_rejects_pending_symlink_and_commit_race(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "root"
    root.mkdir()
    output = root / "result.json"
    outside = tmp_path / "outside.json"
    pending = root / ".result.json.pending.fixed"
    try:
        pending.symlink_to(outside)
    except OSError as exc:  # pragma: no cover - Windows without symlink privilege
        pytest.skip(f"symbolic links unavailable: {exc}")
    root_descriptor = _open_root_directory(root)
    try:
        with pytest.raises(FileExistsError):
            _write_fresh_output(root, root_descriptor, output, {"blocked": True}, token="fixed")
    finally:
        os.close(root_descriptor)
    assert not outside.exists()
    assert pending.is_symlink()
    pending.unlink()

    real_link = os.link

    def create_target_then_link(*args, **kwargs):
        output.write_text("attacker", encoding="utf-8")
        return real_link(*args, **kwargs)

    monkeypatch.setattr(receipt_module.os, "link", create_target_then_link)
    root_descriptor = _open_root_directory(root)
    try:
        with pytest.raises(ValueError, match="appeared during commit"):
            _write_fresh_output(root, root_descriptor, output, {"blocked": True}, token="race")
    finally:
        os.close(root_descriptor)
    assert output.read_text(encoding="utf-8") == "attacker"


@pytest.mark.skipif(os.name == "nt", reason="secure dir_fd traversal is POSIX-only")
def test_bound_input_rejects_mutation_after_read(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    receipt = tmp_path / "receipt.json"
    receipt.write_text('{"receipt": "before"}', encoding="utf-8")
    root_descriptor = _open_root_directory(tmp_path)
    try:
        descriptor, identity = _open_bound_input(tmp_path, root_descriptor, receipt)
    finally:
        os.close(root_descriptor)
    actual_fstat = os.fstat

    def mutate_before_fstat(fd: int):
        receipt.write_text('{"receipt": "after after after"}', encoding="utf-8")
        return actual_fstat(fd)

    monkeypatch.setattr(receipt_module.os, "fstat", mutate_before_fstat)
    with pytest.raises(ValueError, match="changed while being read"):
        _read_bound_json(descriptor, identity)
