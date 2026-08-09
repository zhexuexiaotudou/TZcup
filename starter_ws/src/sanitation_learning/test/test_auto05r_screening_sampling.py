from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[4]


def _load_screening():
    pytest.importorskip("torch")
    path = ROOT / "scripts" / "auto05r_screening.py"
    spec = importlib.util.spec_from_file_location("auto05r_screening", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _row(index: int, world: str = "world") -> dict:
    return {
        "world_id": world,
        "scene_seed": index // 10,
        "frame_index": index,
        "split": "train",
        "negative_only": False,
    }


def test_small_object_frames_are_retained_before_stratified_fill() -> None:
    screening = _load_screening()
    rows = [_row(index) for index in range(20)]
    small_keys = {
        screening._row_identity(rows[index]) for index in (2, 5, 9)
    }
    selected = screening._prioritized_discovery_row_sample(
        rows, small_keys, 8, seed=17
    )
    selected_keys = {screening._row_identity(row) for row in selected}
    assert len(selected) == 8
    assert small_keys <= selected_keys
    assert len(selected_keys) == len(selected)


def test_small_object_sampling_is_deterministic_when_over_capacity() -> None:
    screening = _load_screening()
    rows = [_row(index, world=f"world_{index % 2}") for index in range(12)]
    small_keys = {screening._row_identity(row) for row in rows}
    first = screening._prioritized_discovery_row_sample(
        rows, small_keys, 5, seed=19
    )
    second = screening._prioritized_discovery_row_sample(
        rows, small_keys, 5, seed=19
    )
    assert [screening._row_identity(row) for row in first] == [
        screening._row_identity(row) for row in second
    ]


def _write_qualified_reuse_fixture(
    source_dir: Path,
    *,
    qa_sha256: str = "formal-qa",
    ineligible_task: str | None = None,
) -> None:
    selections = {}
    training = {}
    for task in ("classifier", "leaf", "puddle"):
        selection = {
            "selected": task != ineligible_task,
            "selected_epoch": 3 if task != ineligible_task else None,
            "validation_metrics": {"validation_threshold_product_eligible": True},
        }
        selections[task] = selection
        training[task] = {"selection": selection}
        (source_dir / f"{task}.pt").write_bytes(task.encode("ascii"))
    report = {
        "student_route": {"dataset_qa_sha256": qa_sha256},
        "selection": selections,
        "training": training,
    }
    (source_dir / "auto05r_screening_report.json").write_text(
        json.dumps(report), encoding="utf-8"
    )


def test_qualified_task_reuse_accepts_only_same_qa_eligible_tasks(
    tmp_path: Path,
) -> None:
    screening = _load_screening()
    _write_qualified_reuse_fixture(tmp_path)
    _, provenance = screening._load_qualified_task_reuse_report(
        tmp_path, "formal-qa", ("classifier", "leaf", "puddle")
    )
    assert set(provenance["tasks"]) == {"classifier", "leaf", "puddle"}
    assert all(
        len(task["checkpoint_sha256"]) == 64
        for task in provenance["tasks"].values()
    )


@pytest.mark.parametrize(
    ("qa_sha256", "ineligible_task", "match"),
    (
        ("other-qa", None, "different formal G4 QA"),
        ("formal-qa", "leaf", "not product eligible"),
    ),
)
def test_qualified_task_reuse_rejects_wrong_qa_or_ineligible_task(
    tmp_path: Path,
    qa_sha256: str,
    ineligible_task: str | None,
    match: str,
) -> None:
    screening = _load_screening()
    _write_qualified_reuse_fixture(
        tmp_path, qa_sha256=qa_sha256, ineligible_task=ineligible_task
    )
    with pytest.raises(RuntimeError, match=match):
        screening._load_qualified_task_reuse_report(
            tmp_path, "formal-qa", ("classifier", "leaf", "puddle")
        )


def test_reused_checkpoint_must_be_marked_training_complete(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    screening = _load_screening()
    (tmp_path / "classifier.pt").write_bytes(b"incomplete")
    monkeypatch.setattr(
        screening.torch,
        "load",
        lambda *args, **kwargs: {
            "checkpoint_status": "training_interrupted",
            "state_dict": {"weight": object()},
        },
    )
    with pytest.raises(RuntimeError, match="not marked training_complete"):
        screening._load_reused_model(
            "classifier",
            tmp_path,
            tmp_path / "output",
            screening.torch.device("cpu"),
        )
