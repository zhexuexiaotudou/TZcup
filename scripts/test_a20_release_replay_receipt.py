import json
from pathlib import Path

import pytest

from a20_release_replay_receipt import BLOCKER, _output_in_root, _regular_in_root, validate_receipt


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
    with pytest.raises(ValueError, match="fresh"):
        _output_in_root(root, receipt)
    try:
        link = root / "link.json"
        link.symlink_to(receipt)
    except OSError as exc:  # pragma: no cover - Windows without symlink privilege
        pytest.skip(f"symbolic links unavailable: {exc}")
    with pytest.raises(ValueError, match="non-symlink"):
        _regular_in_root(root, link, "receipt")
