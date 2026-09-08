from __future__ import annotations

import json
import os
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

from auto15_competition_matrix import build_matrix, write_new
import validate_product_acceptance_contract as validation
from validate_product_acceptance_contract import (
    ProductAcceptanceContractError,
    ROOT,
    load_contract,
    validate_auto15_execution_evidence,
    validate_static_contract,
)


def _must_block(payload: dict, tmp_path: Path) -> None:
    try:
        validate_auto15_execution_evidence(load_contract(), payload, tmp_path)
    except ProductAcceptanceContractError as exc:
        assert "BLOCKED_NO_CANONICAL_PRODUCER" in str(exc)
        return
    raise AssertionError("unwired AUTO-15 receipt intake must never pass")


def test_matrix_preserves_full_18x10_and_30_group_requirements_but_is_blocked() -> None:
    state = json.loads((ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(encoding="utf-8"))
    matrix = build_matrix(state)
    assert matrix["status"] == "BLOCKED"
    assert matrix["required_unique_execution_count"] == 180
    assert matrix["required_integrated_missions"] == 30
    assert matrix["formal_video_count"] == matrix["formal_mcap_count"] == 0
    assert matrix["runtime_execution_evidence_pass"] is False
    assert not any(matrix["product_runtime_states"].values())


def test_fake_media_provenance_reuse_and_missing_mapping_cannot_pass(tmp_path: Path) -> None:
    fake = {
        "provenance": {"source_commit": "a" * 40},
        "executions": [{"video": b"\x00\x00\x00\x18ftypisom" + b"padding", "mcap": b"\x89MCAP0\r\n"}],
        "mission_groups": [],
    }
    _must_block(fake, tmp_path)  # fake MP4/23-byte MCAP and forged provenance
    _must_block({"executions": [fake["executions"][0], fake["executions"][0]], "mission_groups": []}, tmp_path)  # reuse
    _must_block({"executions": [], "mission_groups": [{"mission_group_id": "g", "members": []}]}, tmp_path)  # missing 18x10->group map
    state = json.loads((ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json").read_text(encoding="utf-8"))
    try:
        build_matrix(state, fake, tmp_path)
    except ProductAcceptanceContractError:
        pass
    else:
        raise AssertionError("matrix must not promote an arbitrary receipt ledger")


def test_alternate_source_and_toctou_paths_are_not_accepted(tmp_path: Path) -> None:
    source = tmp_path / "a12.md"
    source.write_text((ROOT / "docs" / "a12-product-acceptance-specification.md").read_text(encoding="utf-8"), encoding="utf-8")
    try:
        validate_static_contract(load_contract(), source)
    except ProductAcceptanceContractError:
        pass
    else:
        raise AssertionError("untracked alternate authority source must fail")
    drifted_contract = deepcopy(load_contract())
    drifted_contract["authoritative_specification"]["canonical_content_sha256"] = "0" * 64
    try:
        validate_static_contract(drifted_contract)
    except ProductAcceptanceContractError:
        pass
    else:
        raise AssertionError("A12 source digest drift must fail")
    target = tmp_path / "receipt.json"
    target.write_text("{}", encoding="utf-8")
    alias = tmp_path / "alias.json"
    try:
        os.symlink(target, alias)
    except OSError:
        return
    try:
        validation._sealed_regular_bytes(alias, "symlinked receipt")
    except ProductAcceptanceContractError:
        pass
    else:
        raise AssertionError("symlink input must fail")
    replacement = tmp_path / "replacement.json"
    replacement.write_text('{"replaced":true}', encoding="utf-8")
    original_read, replaced = validation.os.read, False

    def replace_during_read(fd: int, size: int) -> bytes:
        nonlocal replaced
        chunk = original_read(fd, size)
        if chunk and not replaced:
            replaced = True
            os.replace(replacement, target)
        return chunk

    with patch.object(validation.os, "read", side_effect=replace_during_read):
        try:
            validation._sealed_regular_bytes(target, "racing receipt")
        except ProductAcceptanceContractError:
            pass
        else:
            raise AssertionError("descriptor/path replacement must fail")
    _must_block({"executions": [{"receipt": {"path": "alias.json"}}], "mission_groups": []}, tmp_path)


def test_evidence_output_is_fresh_and_never_replaced(tmp_path: Path) -> None:
    artifact = tmp_path / "receipt.json"
    write_new(artifact, b"first")
    assert artifact.read_bytes() == b"first"
    try:
        write_new(artifact, b"replacement")
    except FileExistsError:
        pass
    else:
        raise AssertionError("retained output must not be overwritten")
    assert artifact.read_bytes() == b"first"
