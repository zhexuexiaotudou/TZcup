"""AUTO-05R-2/3 training protocol: balanced batching, trainer, mining, gates.

This module is scaffolding for the G4 training contract:

- ``BalancedBatchSampler`` samples batches by configured proportions across
  positive / negative-only / paper-like hard-negative / discrete classes /
  leaf / puddle buckets.  It cycles the full bucket before reshuffling so a
  small negative set is never repeatedly re-sampled like a weighted sampler.
- ``Trainer`` runs epochs with per-epoch validation, best-checkpoint
  selection on train/validation only, EMA, early stopping, AMP and
  deterministic seeding; the test split is rejected at the API level.
- ``HardNegativeMining`` collects top false positives from train/val
  background only (max 3 rounds) and refuses any test-split frame.
- ``MicroOverfitGate`` evaluates the frozen micro-overfit thresholds and
  returns ``pass=false`` unless every gate (and every metric) is present.

PyTorch is only required by ``Trainer``; all other components are pure NumPy
and stay testable in the fast CI.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import math
import os
import random
from pathlib import Path
from typing import Callable, Iterable, Iterator, Sequence

import numpy as np
import yaml

from .auto04_contract import box_iou, decode_centernet_outputs
from .g4_models import CLASSIFIER_CLASSES
from .g4_split_policy import (
    LEGACY_DIAGNOSTIC_ROLE,
    SEALED_FINAL_ROLE,
    assert_development_rows,
)


_ID_TO_NAME = {
    1: "plastic_bottle",
    2: "metal_can",
    3: "paper_litter",
    4: "leaf_pile",
    5: "puddle",
}

DEFAULT_BATCH_PROPORTIONS = {
    "positive": 0.30,
    "negative_only": 0.20,
    "paper_like_hard_negative": 0.10,
    "plastic_bottle": 0.10,
    "metal_can": 0.10,
    "paper_litter": 0.10,
    "leaf_pile": 0.05,
    "puddle": 0.05,
}

REQUIRED_MODELS = ("discovery", "classifier", "leaf", "puddle")
ALLOWED_SELECTION_SPLITS = frozenset(
    {"train", "train_world_holdout", "val"}
)
REQUIRED_SELECTION_DIAGNOSTICS = frozenset({"D1", "D2", "D3", "D4", "D5"})


def _torch():
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for the G4 Trainer") from exc
    return torch


def row_group_membership(row: dict) -> dict[str, bool]:
    """Classify one row into the sampler's (possibly overlapping) buckets."""
    raw_labels = row.get("labels", row.get("class_ids", ()))
    labels: set[str] = set()
    for value in raw_labels:
        if isinstance(value, str):
            labels.add(value)
        elif isinstance(value, int):
            name = _ID_TO_NAME.get(value)
            if name is not None:
                labels.add(name)
    negative_only = bool(row.get("negative_only", False))
    paper_like = bool(row.get("paper_like_hard_negative", False))
    return {
        "positive": not negative_only,
        "negative_only": negative_only,
        "paper_like_hard_negative": negative_only and paper_like,
        "plastic_bottle": "plastic_bottle" in labels,
        "metal_can": "metal_can" in labels,
        "paper_litter": "paper_litter" in labels,
        "leaf_pile": "leaf_pile" in labels,
        "puddle": "puddle" in labels,
    }


def _allocate_targets(proportions: dict[str, float], batch_size: int) -> dict[str, int]:
    names = list(proportions)
    raw = [proportions[name] * batch_size for name in names]
    targets = [int(math.floor(value)) for value in raw]
    remainder = batch_size - sum(targets)
    order = sorted(
        range(len(names)),
        key=lambda index: (raw[index] - targets[index], index),
        reverse=True,
    )
    for position in range(remainder):
        targets[order[position % len(order)]] += 1
    return dict(zip(names, targets))


class _CyclicBucket:
    """Deterministic cycling sampler for one bucket (no weighted repeats)."""

    def __init__(self, indices: list[int], rng: random.Random):
        self.indices = list(indices)
        self.rng = rng
        self.order: list[int] = []
        self.cursor = 0

    def take(self, count: int) -> list[int]:
        picked: list[int] = []
        while len(picked) < count and self.indices:
            if self.cursor >= len(self.order):
                self.order = list(self.indices)
                self.rng.shuffle(self.order)
                self.cursor = 0
            picked.append(self.order[self.cursor])
            self.cursor += 1
        return picked

    def reset(self, rng: random.Random) -> None:
        self.rng = rng
        self.order = []
        self.cursor = 0


class BalancedBatchSampler:
    """Sample batches by configured group proportions with full-bucket cycling."""

    def __init__(
        self,
        rows: Sequence[dict],
        batch_size: int,
        proportions: dict[str, float] | None = None,
        seed: int = 0,
    ):
        if batch_size < 1:
            raise ValueError("batch_size must be positive")
        self.batch_size = int(batch_size)
        self.seed = int(seed)
        self.proportions = dict(
            DEFAULT_BATCH_PROPORTIONS if proportions is None else proportions
        )
        total = sum(float(value) for value in self.proportions.values())
        if abs(total - 1.0) > 1e-6:
            raise ValueError("batch proportions must sum to 1")
        if not self.proportions:
            raise ValueError("at least one batch group is required")
        self.rows = list(rows)
        if not self.rows:
            raise ValueError("empty dataset")
        memberships = [row_group_membership(row) for row in self.rows]
        self._buckets = {
            group: [
                index
                for index, membership in enumerate(memberships)
                if membership[group]
            ]
            for group in self.proportions
        }
        self.groups = tuple(self.proportions)
        self._rng = random.Random(self.seed)
        self._buckets_cyclic = {
            group: _CyclicBucket(indices, self._rng)
            for group, indices in self._buckets.items()
        }
        self._pool = _CyclicBucket(list(range(len(self.rows))), self._rng)
        self._epoch = 0

    def __len__(self) -> int:
        return max(1, math.ceil(len(self.rows) / self.batch_size))

    def __iter__(self) -> Iterator[list[int]]:
        self._epoch += 1
        epoch_rng = random.Random(self.seed + self._epoch * 7919)
        for bucket in self._buckets_cyclic.values():
            bucket.reset(epoch_rng)
        self._pool.reset(epoch_rng)
        for _ in range(len(self)):
            yield self._batch()

    def _batch(self) -> list[int]:
        targets = _allocate_targets(self.proportions, self.batch_size)
        chosen: list[int] = []
        seen: set[int] = set()
        for group, target in targets.items():
            for index in self._buckets_cyclic[group].take(target):
                if index not in seen:
                    seen.add(index)
                    chosen.append(index)
        while len(chosen) < self.batch_size:
            index = self._pool.take(1)[0]
            if index not in seen:
                seen.add(index)
                chosen.append(index)
        return chosen


def load_training_protocol(path) -> dict:
    """Load and validate the frozen ``auto05r_training_protocol.yaml``."""
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    errors: list[str] = []
    if payload.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if payload.get("stage") != "AUTO-05R":
        errors.append("stage must be AUTO-05R")
    models = payload.get("models", {})
    for name in REQUIRED_MODELS:
        model = models.get(name)
        if not isinstance(model, dict) or not isinstance(
            model.get("seed"), int
        ):
            errors.append(f"models.{name}.seed must be an integer")
    micro = payload.get("micro_overfit", {})
    if not isinstance(micro.get("sample_counts"), dict):
        errors.append("micro_overfit.sample_counts required")
    gates = micro.get("gates")
    if not isinstance(gates, dict) or not gates:
        errors.append("micro_overfit.gates required")
    proportions = payload.get("batch_proportions", {})
    if not isinstance(proportions, dict) or not proportions:
        errors.append("batch_proportions required")
    elif abs(sum(float(value) for value in proportions.values()) - 1.0) > 1e-6:
        errors.append("batch_proportions must sum to 1")
    if not isinstance(payload.get("optimizer"), dict) or not isinstance(
        payload.get("scheduler"), dict
    ):
        errors.append("optimizer and scheduler must be present")
    ema = payload.get("ema_decay")
    patience = payload.get("early_stopping_patience")
    if not isinstance(ema, (int, float)) or not 0.0 < float(ema) < 1.0:
        errors.append("ema_decay must be in (0, 1)")
    if not isinstance(patience, int) or patience < 1:
        errors.append("early_stopping_patience must be a positive integer")
    selection = payload.get("model_selection", {})
    splits = set(selection.get("allowed_splits", []))
    if not splits or not splits <= ALLOWED_SELECTION_SPLITS:
        errors.append(
            "model_selection.allowed_splits must be a subset of {train, val}"
        )
    diagnostics = set(selection.get("allowed_diagnostics", []))
    if diagnostics != REQUIRED_SELECTION_DIAGNOSTICS:
        errors.append(
            "model_selection.allowed_diagnostics must be exactly D1-D5"
        )
    if selection.get("test_split_readable_during_training") is not False:
        errors.append(
            "model_selection.test_split_readable_during_training must be false"
        )
    if selection.get("hard_negative_mining_from_test") is not False:
        errors.append(
            "model_selection.hard_negative_mining_from_test must be false"
        )
    if selection.get("legacy_G4_D6_diagnostic_readable") is not False:
        errors.append(
            "model_selection.legacy_G4_D6_diagnostic_readable must be false"
        )
    if selection.get("G5_sealed_final_readable") is not False:
        errors.append(
            "model_selection.G5_sealed_final_readable must be false"
        )
    if selection.get("constraint_aware") is not True:
        errors.append("model_selection.constraint_aware must be true")
    if selection.get("load_best") is not True:
        errors.append("model_selection.load_best must be true")
    if not isinstance(selection.get("positive_early_stopping_patience"), int) or (
        selection.get("positive_early_stopping_patience", 0) < 1
    ):
        errors.append(
            "model_selection.positive_early_stopping_patience must be a "
            "positive integer"
        )
    if errors:
        raise ValueError(
            "invalid AUTO-05R training protocol: " + "; ".join(errors)
        )
    return payload


class Trainer:
    """Epoch trainer with validation, best checkpoint, EMA, early stopping.

    ``fit`` accepts train/val loaders only.  A dataset that exposes the test
    split, the contaminated legacy diagnostic or the sealed G5 final set is
    rejected with ``ValueError`` before any training step.
    """

    def __init__(
        self,
        model,
        config: dict | None = None,
        *,
        device=None,
        seed: int | None = None,
        output_dir=None,
    ):
        torch = _torch()
        self.model = model
        self.config = dict(config or {})
        if self.config.get("test_split_readable_during_training") is True:
            raise ValueError(
                "test_split_readable_during_training must be false"
            )
        if self.config.get("hard_negative_mining_from_test") is True:
            raise ValueError("hard_negative_mining_from_test must be false")
        self.seed = int(
            self.config.get("seed", 20260805) if seed is None else seed
        )
        self.epochs = int(self.config.get("epochs", 1))
        self.ema_decay = float(self.config.get("ema_decay", 0.0))
        self.patience = int(self.config.get("early_stopping_patience", 0))
        self.amp = bool(self.config.get("amp", False))
        self.output_dir = Path(output_dir) if output_dir else None
        self.device = (
            torch.device(device)
            if device is not None
            else torch.device(
                "cuda" if torch.cuda.is_available() else "cpu"
            )
        )
        self.model = self.model.to(self.device)
        learning_rate = float(self.config.get("learning_rate", 1e-3))
        weight_decay = float(self.config.get("weight_decay", 0.0))
        self.optimizer = torch.optim.AdamW(
            self.model.parameters(),
            lr=learning_rate,
            weight_decay=weight_decay,
        )
        self.scheduler = self._build_scheduler()
        self.ema_state = None
        self._non_ema_state = None
        self.best_metric = None
        self.best_epoch = None
        self.best_state = None
        self.curves: list[dict] = []
        self.epochs_without_improvement = 0
        self.early_stopped = False

    def _build_scheduler(self):
        torch = _torch()
        scheduler_cfg = self.config.get("scheduler") or {}
        name = scheduler_cfg.get("name")
        if not name:
            return None
        if name == "CosineAnnealingLR":
            return torch.optim.lr_scheduler.CosineAnnealingLR(
                self.optimizer,
                T_max=int(scheduler_cfg.get("t_max", self.epochs)),
                eta_min=float(scheduler_cfg.get("eta_min", 0.0)),
            )
        if name == "StepLR":
            return torch.optim.lr_scheduler.StepLR(
                self.optimizer,
                step_size=int(scheduler_cfg.get("step_size", 10)),
                gamma=float(scheduler_cfg.get("gamma", 0.5)),
            )
        raise ValueError(f"unsupported scheduler {name}")

    def _apply_seed(self) -> None:
        torch = _torch()
        os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")
        random.seed(self.seed)
        np.random.seed(self.seed)
        torch.manual_seed(self.seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(self.seed)
        try:
            torch.use_deterministic_algorithms(True, warn_only=True)
            torch.backends.cudnn.deterministic = True
            torch.backends.cudnn.benchmark = False
        except Exception:
            pass

    @staticmethod
    def _assert_no_test_dataset(dataset) -> None:
        split = getattr(dataset, "split", None)
        splits = getattr(dataset, "splits", None)
        forbidden = {
            "test",
            LEGACY_DIAGNOSTIC_ROLE,
            SEALED_FINAL_ROLE,
        }
        if split in forbidden or (
            splits is not None and forbidden.intersection(set(splits))
        ):
            raise ValueError(
                "Trainer refuses datasets exposing the test split, "
                "legacy_G4_D6_diagnostic or G5_SEALED_FINAL "
                "(all development-unreadable)"
            )

    def _update_ema(self) -> None:
        if not 0.0 < self.ema_decay < 1.0:
            return
        with _torch().no_grad():
            if self.ema_state is None:
                self.ema_state = {
                    key: value.detach().clone()
                    for key, value in self.model.state_dict().items()
                }
            else:
                for key, value in self.model.state_dict().items():
                    self.ema_state[key] = (
                        self.ema_decay * self.ema_state[key]
                        + (1.0 - self.ema_decay) * value.detach()
                    )

    def _apply_ema_weights(self) -> None:
        self._non_ema_state = {
            key: value.detach().clone()
            for key, value in self.model.state_dict().items()
        }
        self.model.load_state_dict(
            {key: value.clone() for key, value in self.ema_state.items()},
            strict=False,
        )

    def _restore_weights(self) -> None:
        if self._non_ema_state is not None:
            self.model.load_state_dict(self._non_ema_state)
            self._non_ema_state = None

    def _run_epoch(self, train_loader, epoch: int, loss_fn) -> float:
        torch = _torch()
        self.model.train()
        total = 0.0
        steps = 0
        amp_enabled = self.amp and torch.cuda.is_available()
        scaler = torch.amp.GradScaler("cuda", enabled=amp_enabled)
        for inputs, targets in train_loader:
            inputs = inputs.to(self.device)
            targets = targets.to(self.device)
            self.optimizer.zero_grad(set_to_none=True)
            with torch.autocast(
                device_type="cuda",
                enabled=amp_enabled,
                dtype=torch.float16,
            ):
                outputs = self.model(inputs)
                loss = loss_fn(outputs, targets)
            scaler.scale(loss).backward()
            scaler.step(self.optimizer)
            scaler.update()
            self._update_ema()
            total += float(loss.detach().cpu())
            steps += 1
        return total / max(steps, 1)

    def validate(
        self,
        val_loader=None,
        *,
        metric_fn=None,
        metric_key: str = "validation_loss",
    ) -> dict:
        """Validate with EMA weights when EMA is enabled."""
        if val_loader is None:
            raise ValueError("validate requires val_loader")
        self._assert_no_test_dataset(val_loader.dataset)
        if metric_fn is None:
            raise TypeError("metric_fn is required for validation")
        ema_applied = False
        if self.ema_state is not None:
            self._apply_ema_weights()
            ema_applied = True
        try:
            self.model.eval()
            metrics = metric_fn(self.model, self._device_loader(val_loader))
        finally:
            if ema_applied:
                self._restore_weights()
        if metric_key not in metrics:
            raise ValueError(
                f"metric_fn must return {metric_key!r}, got {sorted(metrics)}"
            )
        return metrics

    def _device_loader(self, loader):
        """Yield validation batches on the configured model device."""

        def move(value):
            if hasattr(value, "to"):
                return value.to(self.device)
            if isinstance(value, dict):
                return {key: move(item) for key, item in value.items()}
            if isinstance(value, tuple):
                return tuple(move(item) for item in value)
            if isinstance(value, list):
                return [move(item) for item in value]
            return value

        for batch in loader:
            yield move(batch)

    def _default_metric_fn(self, loss_fn):
        device = self.device

        def metric_fn(model, loader) -> dict:
            torch = _torch()
            model.eval()
            total = 0.0
            count = 0
            with torch.no_grad():
                for inputs, targets in loader:
                    inputs = inputs.to(device)
                    targets = targets.to(device)
                    total += float(loss_fn(model(inputs), targets).item())
                    count += 1
            return {"validation_loss": total / max(count, 1)}

        return metric_fn

    def fit(
        self,
        train_loader=None,
        val_loader=None,
        *,
        loss_fn=None,
        metric_fn=None,
        metric_key: str = "validation_loss",
        maximize: bool = False,
        checkpoint_path=None,
    ) -> dict:
        """Train with train/val only; returns the full curve and best info."""
        torch = _torch()
        if train_loader is None or val_loader is None:
            raise ValueError(
                "Trainer.fit requires train_loader and val_loader; "
                "the test split is never accepted"
            )
        self._assert_no_test_dataset(train_loader.dataset)
        self._assert_no_test_dataset(val_loader.dataset)
        if loss_fn is None:
            raise TypeError("loss_fn is required")
        metric_fn = metric_fn or self._default_metric_fn(loss_fn)
        self._apply_seed()
        checkpoint = Path(checkpoint_path) if checkpoint_path is not None else None
        if checkpoint is not None:
            checkpoint.parent.mkdir(parents=True, exist_ok=True)
        for epoch in range(1, self.epochs + 1):
            training_loss = self._run_epoch(train_loader, epoch, loss_fn)
            metrics = self.validate(
                val_loader, metric_fn=metric_fn, metric_key=metric_key
            )
            curve = {"epoch": epoch, "training_loss": training_loss, **metrics}
            self.curves.append(curve)
            if self.scheduler is not None:
                self.scheduler.step()
            value = float(metrics[metric_key])
            improved = self.best_metric is None or (
                value > self.best_metric
                if maximize
                else value < self.best_metric
            )
            if improved:
                self.best_metric = value
                self.best_epoch = epoch
                self.best_state = {
                    key: tensor.detach().cpu().clone()
                    for key, tensor in self.model.state_dict().items()
                }
                if checkpoint is not None:
                    torch.save(
                        {
                            "best_epoch": epoch,
                            "best_metric": value,
                            "state_dict": self.best_state,
                            "ema_state": (
                                {
                                    key: tensor.detach().cpu().clone()
                                    for key, tensor in self.ema_state.items()
                                }
                                if self.ema_state is not None
                                else None
                            ),
                            "seed": self.seed,
                        },
                        checkpoint,
                    )
                self.epochs_without_improvement = 0
            else:
                self.epochs_without_improvement += 1
            if (
                self.patience > 0
                and self.epochs_without_improvement >= self.patience
            ):
                self.early_stopped = True
                break
        return {
            "curves": self.curves,
            "best_epoch": self.best_epoch,
            "best_metric": self.best_metric,
            "early_stopped": self.early_stopped,
            "best_checkpoint_path": (
                str(checkpoint) if checkpoint is not None else None
            ),
            "test_split_readable_during_training": False,
            "status": "scaffold_fit_completed",
        }


class HardNegativeMining:
    """Top-false-positive collection from train/val background only.

    ``max_rounds`` is capped at 3.  Any frame with ``split == "test"`` raises
    ``ValueError`` before mining, so the G4 final test can never be read.
    """

    def __init__(
        self,
        decode_fn: Callable | None = None,
        *,
        max_rounds: int = 3,
        stride: int = 4,
        score_threshold: float = 0.5,
        nms_iou_threshold: float = 0.5,
        fp_iou_threshold: float = 0.3,
        top_k: int = 64,
        seed: int = 0,
    ):
        max_rounds = int(max_rounds)
        if not 1 <= max_rounds <= 3:
            raise ValueError("HardNegativeMining supports at most 3 rounds")
        self.max_rounds = max_rounds
        self.stride = int(stride)
        self.score_threshold = float(score_threshold)
        self.nms_iou_threshold = float(nms_iou_threshold)
        self.fp_iou_threshold = float(fp_iou_threshold)
        self.top_k = int(top_k)
        self.seed = int(seed)
        self.decode_fn = decode_fn or self._default_decode

    def _default_decode(self, outputs):
        if isinstance(outputs, dict):
            objectness_logits = outputs.get("objectness_logits")
            if objectness_logits is not None:
                objectness = 1.0 / (
                    1.0 + np.exp(-np.asarray(objectness_logits, dtype=np.float32))
                )
            else:
                objectness = np.asarray(outputs["objectness"], dtype=np.float32)
            offset = outputs["offset"]
            size = outputs.get("bbox_size")
            if size is None:
                size = outputs["size"]
        else:
            objectness = np.asarray(outputs[0], dtype=np.float32)
            offset, size = outputs[1], outputs[2]
        return decode_centernet_outputs(
            objectness,
            np.asarray(offset),
            np.asarray(size),
            stride=self.stride,
            score_threshold=self.score_threshold,
            nms_iou_threshold=self.nms_iou_threshold,
        )

    def mine(self, frames: Sequence[dict], infer_fn: Callable) -> dict:
        frames = list(frames)
        test_frames = [
            frame
            for frame in frames
            if str(frame.get("split", "")).lower() == "test"
        ]
        forbidden = [
            frame
            for frame in frames
            if str(frame.get("split", "")) in (
                "test",
                LEGACY_DIAGNOSTIC_ROLE,
                SEALED_FINAL_ROLE,
            )
        ]
        if test_frames or forbidden:
            raise ValueError(
                "hard-negative mining must never read the contaminated legacy "
                "diagnostic, the G4 final test or G5_SEALED_FINAL; "
                f"found {len(test_frames) + len(forbidden)} forbidden frames"
            )
        missing_split = [
            frame for frame in frames if "split" not in frame
        ]
        if missing_split:
            raise ValueError(
                "every frame passed to hard-negative mining must declare its "
                "split so test frames cannot be mined accidentally"
            )
        rounds: list[dict] = []
        mined_by_round: list[list[dict]] = []
        for round_index in range(1, self.max_rounds + 1):
            candidates: list[dict] = []
            for frame in frames:
                detections = self.decode_fn(infer_fn(frame))
                gt_boxes = [tuple(value) for value in frame.get("boxes", ())]
                false_positives = [
                    det
                    for det in detections
                    if not any(
                        box_iou(det.bbox_xyxy, gt_box) >= self.fp_iou_threshold
                        for gt_box in gt_boxes
                    )
                ]
                for det in false_positives:
                    candidates.append(
                        {
                            "frame_index": int(frame["index"]),
                            "score": det.score,
                            "bbox_xyxy": list(det.bbox_xyxy),
                        }
                    )
            candidates.sort(
                key=lambda item: (item["score"], -item["frame_index"]),
                reverse=True,
            )
            selected = candidates[: self.top_k]
            if not selected:
                break
            mined_by_round.append(selected)
            rounds.append(
                {
                    "round": round_index,
                    "candidate_count": len(candidates),
                    "mined_count": len(selected),
                    "top_scores": [item["score"] for item in selected],
                }
            )
        mined_indices = sorted(
            {
                int(item["frame_index"])
                for candidates in mined_by_round
                for item in candidates
            }
        )
        return {
            "rounds": rounds,
            "mined_frame_indices": mined_indices,
            "test_frames_seen": 0,
            "train_frames_seen": sum(
                1 for frame in frames if frame.get("split") == "train"
            ),
            "val_frames_seen": sum(
                1 for frame in frames if frame.get("split") == "val"
            ),
            "max_rounds": self.max_rounds,
        }


GATE_SPECS = (
    ("discovery_recall", "discovery_recall_min", "ge"),
    ("discovery_ap50", "discovery_ap50_min", "ge"),
    ("discovery_precision", "discovery_precision_min", "ge"),
    (
        "discovery_false_proposals_per_frame",
        "discovery_false_proposals_per_frame_max",
        "le",
    ),
    ("negative_fp_per_frame", "negative_fp_per_frame_max", "le"),
    ("classifier_macro_f1", "classifier_macro_f1_min", "ge"),
    ("paper_precision", "paper_precision_min", "ge"),
    (
        "classifier_background_specificity",
        "classifier_background_specificity_min",
        "ge",
    ),
    (
        "classifier_hard_negative_specificity",
        "classifier_hard_negative_specificity_min",
        "ge",
    ),
    ("leaf_iou", "leaf_iou_min", "ge"),
    ("leaf_boundary_f1", "leaf_boundary_f1_min", "ge"),
    (
        "leaf_negative_fp_per_frame",
        "leaf_negative_fp_per_frame_max",
        "le",
    ),
    ("puddle_iou", "puddle_iou_min", "ge"),
    ("puddle_boundary_f1", "puddle_boundary_f1_min", "ge"),
    (
        "puddle_negative_fp_per_frame",
        "puddle_negative_fp_per_frame_max",
        "le",
    ),
)


class MicroOverfitGate:
    """Evaluate the frozen capacity-only micro-overfit thresholds.

    Micro gates prove fitting capacity on a tiny train subset only; they are
    explicitly not development screening and never imply a product claim.
    """

    def __init__(self, thresholds: dict):
        self.thresholds = {
            key: float(value) for key, value in dict(thresholds).items()
        }

    def evaluate(self, metrics: dict) -> dict:
        gates: dict[str, dict] = {}
        missing_thresholds: list[str] = []
        for gate_name, threshold_key, operator in GATE_SPECS:
            if threshold_key not in self.thresholds:
                missing_thresholds.append(threshold_key)
                gates[gate_name] = {
                    "passed": False,
                    "reason": "missing_threshold",
                }
                continue
            threshold = self.thresholds[threshold_key]
            if gate_name not in metrics:
                gates[gate_name] = {
                    "passed": False,
                    "value": None,
                    "threshold": threshold,
                    "reason": "missing_metric",
                }
                continue
            value = float(metrics[gate_name])
            passed = (
                value >= threshold if operator == "ge" else value <= threshold
            )
            gates[gate_name] = {
                "passed": bool(passed),
                "value": value,
                "threshold": threshold,
            }
        passed = (
            all(item["passed"] for item in gates.values())
            and not missing_thresholds
        )
        return {
            "pass": bool(passed),
            "micro_overfit_pass": bool(passed),
            "gate_kind": "capacity_only",
            "screening_claim_allowed": False,
            "product_claim_allowed": False,
            "gates": gates,
            "missing_thresholds": missing_thresholds,
            "status": "passed" if passed else "failed",
        }


__all__ = [
    "ALLOWED_SELECTION_SPLITS",
    "BalancedBatchSampler",
    "DEFAULT_BATCH_PROPORTIONS",
    "GATE_SPECS",
    "HardNegativeMining",
    "MicroOverfitGate",
    "REQUIRED_MODELS",
    "REQUIRED_SELECTION_DIAGNOSTICS",
    "Trainer",
    "load_training_protocol",
    "row_group_membership",
]
