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


def test_area_negative_sampling_covers_taxonomy_before_repeat() -> None:
    screening = _load_screening()
    rows = []
    for index, taxonomy in enumerate(
        ("wet_non_puddle", "reflection", "shadow_edge", "road_paint")
    ):
        for repeat in range(3):
            row = _row(index * 10 + repeat, world=f"world_{index}")
            row["negative_only"] = True
            row["taxonomies"] = [taxonomy]
            rows.append(row)
    selected = screening._taxonomy_balanced_negative_sample(rows, 4, seed=31)
    assert {row["taxonomies"][0] for row in selected} == {
        "wet_non_puddle",
        "reflection",
        "shadow_edge",
        "road_paint",
    }
    assert len({screening._row_identity(row) for row in selected}) == 4


def test_scene_metadata_adds_ground_and_lighting_taxonomy() -> None:
    screening = _load_screening()
    row = _row(0)
    tagged = screening._tag_rows_with_scene_metadata(
        [row],
        {
            0: {
                "objects": [
                    {"taxonomy": "reflection", "semantic_label": 0}
                ],
                "ground_material_executed_by_world": "wet_dark_asphalt",
                "lighting_executed_by_world": "low_sun_glare",
            }
        },
    )[0]
    assert tagged["taxonomies"] == [
        "ground:wet_dark_asphalt",
        "lighting:low_sun_glare",
        "reflection",
    ]


def test_area_backbone_freeze_keeps_decoder_and_boundary_trainable() -> None:
    screening = _load_screening()

    class FakeArea(screening.torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.deeplab = screening.torch.nn.Module()
            self.deeplab.backbone = screening.torch.nn.Linear(2, 2)
            self.deeplab.classifier = screening.torch.nn.Sequential(
                screening.torch.nn.Linear(2, 2),
                screening.torch.nn.BatchNorm1d(2),
                screening.torch.nn.Linear(2, 1),
            )
            self.geometry_stem = screening.torch.nn.Linear(2, 2)
            self.boundary_head = screening.torch.nn.Linear(2, 1)

    model = FakeArea()
    report = screening._freeze_area_backbone(model)
    states = {name: value.requires_grad for name, value in model.named_parameters()}
    assert not states["deeplab.backbone.weight"]
    assert not states["geometry_stem.weight"]
    assert states["deeplab.classifier.0.weight"]
    assert not states["deeplab.classifier.1.weight"]
    assert states["deeplab.classifier.2.weight"]
    assert states["boundary_head.weight"]
    assert report["frozen_parameter_tensors"] == 4
    assert report["trainable_parameter_tensors"] == 8
    assert report["frozen_batch_norm_modules"] == 1
    assert model.force_batch_norm_eval


def test_area_refiner_only_freezes_every_base_parameter() -> None:
    screening = _load_screening()

    class FakeRefinedArea(screening.torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.base = screening.torch.nn.Linear(2, 2)
            self.highres_refiner = screening.torch.nn.Sequential(
                screening.torch.nn.Linear(2, 2),
                screening.torch.nn.ReLU(),
                screening.torch.nn.Linear(2, 1),
            )

    model = FakeRefinedArea()
    report = screening._freeze_area_refiner_only(model)
    states = {name: value.requires_grad for name, value in model.named_parameters()}
    assert not states["base.weight"]
    assert not states["base.bias"]
    assert states["highres_refiner.0.weight"]
    assert states["highres_refiner.2.weight"]
    assert report["frozen_parameter_tensors"] == 2
    assert report["trainable_parameter_tensors"] == 4


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


def test_selection_fingerprint_ignores_report_checkpoint_metadata_drift() -> None:
    screening = _load_screening()
    common = {
        "selected_epoch": 5,
        "selection_score": 0.97,
        "tie_breaker_score": 0.05,
        "validation_metrics": {"validation_macro_f1": 0.97},
        "violated_constraints": [],
    }
    report_selection = {"selected": True, **common}
    checkpoint_selection = {
        **common,
        "product_eligible": True,
        "status": "constraint_feasible",
    }
    assert screening._selection_fingerprint(
        report_selection
    ) == screening._selection_fingerprint(checkpoint_selection)
