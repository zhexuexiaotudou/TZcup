from __future__ import annotations

import json
from pathlib import Path

from generate_perception_product_status import EVIDENCE_KEYS, generate


def _write(path: Path, field_path: tuple[str, ...], value: bool) -> None:
    payload: dict = {}
    target = payload
    for key in field_path[:-1]:
        target[key] = {}
        target = target[key]
    target[field_path[-1]] = value
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_status_generator_is_fail_closed_and_hashes_evidence(tmp_path: Path):
    evidence = {}
    for name, field_path in EVIDENCE_KEYS.items():
        path = tmp_path / f"{name}.json"
        _write(path, field_path, name != "field")
        evidence[name] = path
    output = tmp_path / "output"
    registry = tmp_path / "registry.json"
    registry.write_text(json.dumps({"models": {"detector": {"sha256": "c" * 64}}}))
    release = tmp_path / "release.json"
    release.write_text(json.dumps({"release_id": "formal-v1", "source_commit": "a" * 40}))
    result = generate(
        output,
        evidence,
        source_commit="a" * 40,
        external_blockers={"field"},
        model_registry_path=registry,
        release_manifest_path=release,
    )
    assert result["statuses"]["PRODUCT_SIM_PERCEPTION_READY"] is True
    assert result["statuses"]["PRODUCT_X86_RUNTIME_READY"] is True
    assert result["statuses"]["PRODUCT_J6_TOOLCHAIN_READY"] is True
    assert result["statuses"]["PRODUCT_J6_BOARD_READY"] is True
    assert result["statuses"]["PRODUCT_FIELD_READY"] is False
    blockers = json.loads(
        (output / "PERCEPTION_PRODUCT_BLOCKERS.json").read_text()
    )
    assert blockers["external_only"] is True
    assert blockers["blockers"] == [
        {
            "gate": "field",
            "classification": "external_resource",
            "evidence_status": "failed",
        }
    ]
    assert len(
        json.loads(
            (output / "PERCEPTION_RELEASE_MANIFEST.json").read_text()
        )["status_sha256"]
    ) == 64
    assert json.loads(
        (output / "PERCEPTION_RELEASE_MANIFEST.json").read_text()
    )["release_id"] == "formal-v1"
    assert json.loads(
        (output / "PERCEPTION_MODEL_REGISTRY.json").read_text()
    )["models"]["detector"]["sha256"] == "c" * 64


def test_missing_evidence_never_passes_a_product_status(tmp_path: Path):
    result = generate(
        tmp_path / "output",
        {},
        source_commit="b" * 40,
        external_blockers={"j6_board", "field"},
    )
    assert not any(result["statuses"].values())
