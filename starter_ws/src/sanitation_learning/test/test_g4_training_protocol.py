from __future__ import annotations

from collections import Counter
from pathlib import Path
import random
import sys

import numpy as np
import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_training import (  # noqa: E402
    BalancedBatchSampler,
    DEFAULT_BATCH_PROPORTIONS,
    HardNegativeMining,
    MicroOverfitGate,
    Trainer,
    _allocate_targets,
    load_training_protocol,
    row_group_membership,
)


REPO = Path(__file__).resolve().parents[4]
PROTOCOL = (
    REPO
    / "starter_ws"
    / "src"
    / "sanitation_learning"
    / "config"
    / "auto05r_training_protocol.yaml"
)


def _make_rows(count: int) -> list[dict]:
    rows = []
    for index in range(count):
        negative_only = index % 5 < 2
        labels = set()
        if not negative_only:
            if index % 3 == 0:
                labels.add("plastic_bottle")
            elif index % 3 == 1:
                labels.add("metal_can")
            else:
                labels.add("paper_litter")
            if index % 4 == 0:
                labels.add("leaf_pile")
            if index % 7 == 0:
                labels.add("puddle")
        rows.append(
            {
                "index": index,
                "negative_only": negative_only,
                "paper_like_hard_negative": negative_only and index % 10 == 0,
                "labels": sorted(labels),
            }
        )
    return rows


def test_protocol_frozen_contract() -> None:
    payload = load_training_protocol(PROTOCOL)
    for name in ("discovery", "classifier", "leaf", "puddle"):
        assert isinstance(payload["models"][name]["seed"], int)
    assert payload["models"]["discovery"]["input_shape"] == [1, 3, 512, 384]
    assert abs(
        sum(float(value) for value in payload["batch_proportions"].values())
        - 1.0
    ) < 1e-9
    assert 0.0 < payload["ema_decay"] < 1.0
    assert payload["early_stopping_patience"] >= 1
    assert payload["optimizer"]["name"] == "AdamW"
    assert payload["scheduler"]["name"] == "CosineAnnealingLR"
    selection = payload["model_selection"]
    assert set(selection["allowed_splits"]) <= {"train", "val"}
    assert set(selection["allowed_diagnostics"]) == {
        "D1",
        "D2",
        "D3",
        "D4",
        "D5",
    }
    assert selection["test_split_readable_during_training"] is False
    assert selection["hard_negative_mining_from_test"] is False
    assert set(payload["micro_overfit"]["sample_counts"]) == {
        "discovery_frames",
        "classifier_crops",
        "leaf_frames",
        "puddle_frames",
    }
    assert set(payload["micro_overfit"]["gates"]) == {
        "discovery_recall_min",
        "negative_fp_per_frame_max",
        "classifier_macro_f1_min",
        "paper_precision_min",
        "leaf_iou_min",
        "puddle_iou_min",
    }


def test_row_group_membership() -> None:
    membership = row_group_membership(
        {
            "negative_only": True,
            "paper_like_hard_negative": True,
            "labels": [],
        }
    )
    assert membership["negative_only"] is True
    assert membership["paper_like_hard_negative"] is True
    assert membership["positive"] is False
    membership = row_group_membership(
        {"negative_only": False, "labels": [4, "puddle"]}
    )
    assert membership["leaf_pile"] is True
    assert membership["puddle"] is True
    assert membership["positive"] is True


def test_balanced_batch_sampler_rejects_bad_proportions() -> None:
    rows = _make_rows(20)
    with pytest.raises(ValueError):
        BalancedBatchSampler(
            rows, batch_size=8, proportions={"positive": 0.5}
        )
    with pytest.raises(ValueError):
        BalancedBatchSampler([], batch_size=8)


def test_balanced_batch_sampler_group_proportions() -> None:
    rows = _make_rows(200)
    sampler = BalancedBatchSampler(rows, batch_size=16, seed=123)
    targets = _allocate_targets(sampler.proportions, sampler.batch_size)
    assert sum(targets.values()) == sampler.batch_size
    for group, target in targets.items():
        assert (
            abs(target / sampler.batch_size - sampler.proportions[group])
            <= 1 / sampler.batch_size + 1e-9
        )
    sampled: list[list[int]] = []
    for _ in range(8):
        sampled.extend(list(iter(sampler)))
    seen_rows: set[int] = set()
    for batch in sampled:
        assert len(batch) == sampler.batch_size
        assert len(set(batch)) == len(batch)  # no duplicates inside a batch
        seen_rows.update(batch)
        for group in sampler.groups:
            if targets[group] >= 1:
                assert any(
                    row_group_membership(rows[index])[group]
                    for index in batch
                ), f"group {group} missing from a batch"
    assert seen_rows == set(range(len(rows)))


def test_balanced_sampler_does_not_repeat_tiny_negative_set() -> None:
    rows = []
    for index in range(20):
        if index < 4:
            rows.append(
                {
                    "index": index,
                    "negative_only": True,
                    "paper_like_hard_negative": False,
                    "labels": [],
                }
            )
        elif index < 7:
            rows.append(
                {
                    "index": index,
                    "negative_only": True,
                    "paper_like_hard_negative": True,
                    "labels": [],
                }
            )
        else:
            rows.append(
                {
                    "index": index,
                    "negative_only": False,
                    "paper_like_hard_negative": False,
                    "labels": ["plastic_bottle"],
                }
            )
    sampler = BalancedBatchSampler(rows, batch_size=8, seed=5)
    appearances = Counter()
    paper_appearances = Counter()
    for _ in range(60):
        for batch in sampler:
            for index in batch:
                if index < 4:
                    appearances[index] += 1
                elif index < 7:
                    paper_appearances[index] += 1
    assert set(appearances) == {0, 1, 2, 3}
    assert 50 <= min(appearances.values()) <= max(appearances.values()) <= 130
    assert len(paper_appearances) == 3


def test_trainer_best_checkpoint_never_reads_test(tmp_path) -> None:
    torch = pytest.importorskip("torch")
    from torch.utils.data import DataLoader, TensorDataset

    class TinyModel(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.linear = torch.nn.Linear(4, 1)

        def forward(self, x):
            return self.linear(x)

    torch.manual_seed(0)
    model = TinyModel()
    train = TensorDataset(torch.randn(16, 4), torch.randn(16, 1))
    val = TensorDataset(torch.randn(8, 4), torch.randn(8, 1))
    train_loader = DataLoader(train, batch_size=4)
    val_loader = DataLoader(val, batch_size=4)

    def loss_fn(outputs, targets):
        return ((outputs - targets) ** 2).mean()

    def metric_fn(model, loader):
        model.eval()
        total = 0.0
        count = 0
        with torch.no_grad():
            for inputs, targets in loader:
                total += float(loss_fn(model(inputs), targets).item())
                count += 1
        return {"validation_loss": total / max(count, 1)}

    config = {
        "seed": 7,
        "epochs": 3,
        "ema_decay": 0.9,
        "early_stopping_patience": 2,
        "amp": False,
        "learning_rate": 0.01,
        "weight_decay": 0.0,
        "test_split_readable_during_training": False,
        "hard_negative_mining_from_test": False,
    }
    trainer = Trainer(model, config)
    checkpoint = tmp_path / "best.pt"
    report = trainer.fit(
        train_loader,
        val_loader,
        loss_fn=loss_fn,
        metric_fn=metric_fn,
        metric_key="validation_loss",
        maximize=False,
        checkpoint_path=checkpoint,
    )
    assert report["best_epoch"] >= 1
    assert len(report["curves"]) >= 1
    assert report["curves"][0]["epoch"] == 1
    assert checkpoint.is_file()
    assert report["test_split_readable_during_training"] is False
    assert report["status"] == "scaffold_fit_completed"

    class TestDataset(TensorDataset):
        split = "test"

    test_loader = DataLoader(TestDataset(val.tensors[0], val.tensors[1]))
    with pytest.raises(ValueError):
        trainer.fit(
            train_loader,
            test_loader,
            loss_fn=loss_fn,
            metric_fn=metric_fn,
            metric_key="validation_loss",
        )


def _frame(index: int, split: str, boxes=()) -> dict:
    return {
        "index": index,
        "split": split,
        "boxes": [list(box) for box in boxes],
    }


def _fp_output(height: int = 8, width: int = 8) -> dict:
    objectness = np.zeros((1, height, width), np.float32)
    objectness[0, 6, 6] = 0.99
    offset = np.zeros((2, height, width), np.float32)
    bbox_size = np.ones((2, height, width), np.float32)
    return {
        "objectness": objectness,
        "offset": offset,
        "bbox_size": bbox_size,
    }


def _fp_logit_output(height: int = 8, width: int = 8) -> dict:
    objectness = np.full((1, height, width), -2.0, np.float32)
    objectness[0, 6, 6] = 2.0
    offset = np.zeros((2, height, width), np.float32)
    bbox_size = np.ones((2, height, width), np.float32)
    return {
        "objectness_logits": objectness,
        "offset": offset,
        "bbox_size": bbox_size,
    }


def test_hard_negative_mining_rejects_test_split() -> None:
    miner = HardNegativeMining(max_rounds=3, top_k=2, seed=1)
    frames = [
        _frame(0, "train"),
        _frame(1, "val"),
        _frame(2, "test"),
    ]
    with pytest.raises(ValueError):
        miner.mine(frames, lambda frame: _fp_output())
    frames_missing_split = [_frame(0, "train"), {"index": 1, "boxes": []}]
    with pytest.raises(ValueError):
        miner.mine(frames_missing_split, lambda frame: _fp_output())


def test_hard_negative_mining_only_mines_train_val_background() -> None:
    miner = HardNegativeMining(max_rounds=3, top_k=2, seed=1)
    frames = [_frame(0, "train"), _frame(1, "train"), _frame(2, "val")]
    result = miner.mine(frames, lambda frame: _fp_output())
    assert result["test_frames_seen"] == 0
    assert result["train_frames_seen"] == 2
    assert result["val_frames_seen"] == 1
    assert 1 <= len(result["rounds"]) <= 3
    assert set(result["mined_frame_indices"]) <= {0, 1, 2}


def test_hard_negative_mining_sigmoids_objectness_logits() -> None:
    miner = HardNegativeMining(max_rounds=1, top_k=2, seed=1)
    frames = [_frame(11, "val")]
    result = miner.mine(frames, lambda frame: _fp_logit_output())
    assert 11 in result["mined_frame_indices"]
    assert len(result["rounds"]) == 1


def test_hard_negative_mining_ignores_gt_matched_detection() -> None:
    miner = HardNegativeMining(max_rounds=2, top_k=2, seed=1)
    frames = [
        _frame(10, "train", boxes=[(22.0, 22.0, 26.0, 26.0)]),
        _frame(11, "val"),
    ]
    result = miner.mine(frames, lambda frame: _fp_output())
    assert 10 not in result["mined_frame_indices"]
    assert 11 in result["mined_frame_indices"]


def test_hard_negative_mining_round_cap() -> None:
    with pytest.raises(ValueError):
        HardNegativeMining(max_rounds=4)
    miner = HardNegativeMining(max_rounds=3, top_k=1)
    frames = [_frame(0, "train")] * 10
    result = miner.mine(frames, lambda frame: _fp_output())
    assert len(result["rounds"]) <= 3


def test_micro_overfit_gate_pass_and_fail() -> None:
    protocol = load_training_protocol(PROTOCOL)
    gate = MicroOverfitGate(protocol["micro_overfit"]["gates"])
    passing = {
        "discovery_recall": 0.95,
        "negative_fp_per_frame": 0.05,
        "classifier_macro_f1": 0.97,
        "paper_precision": 0.93,
        "leaf_iou": 0.75,
        "puddle_iou": 0.72,
    }
    assert gate.evaluate(passing)["pass"] is True
    failing = dict(passing)
    failing["leaf_iou"] = 0.3
    assert gate.evaluate(failing)["pass"] is False
    assert gate.evaluate(failing)["gates"]["leaf_iou"]["passed"] is False
    assert gate.evaluate({"discovery_recall": 0.95})["pass"] is False
