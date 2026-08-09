"""Training loops for the AUTO-05R-2/3 G4 model families."""

from __future__ import annotations

import math
from pathlib import Path
import random
import time
from typing import Callable

import numpy as np
import torch
from torch.utils.data import DataLoader, Sampler

from .g4_data import (
    G4AreaDataset,
    G4ClassifierDataset,
    G4DiscoveryCropDataset,
    G4DiscoveryDataset,
)
from .g4_losses import area_loss, classifier_loss, discovery_loss
from .g4_models import build_g4_model
from .g4_selection import ConstraintAwareSelector
from .g4_split_policy import assert_development_rows


SEED = 20260806


class _AreaBalancedSampler(Sampler):
    """Deterministic 50/50 positive/negative sampling for area micro runs."""

    def __init__(
        self,
        rows: list[dict],
        batch_size: int,
        seed: int,
    ):
        super().__init__()
        self.rows = list(rows)
        self.batch_size = int(batch_size)
        self.seed = int(seed)
        self.positive = [
            index
            for index, row in enumerate(self.rows)
            if not row.get("negative_only")
        ]
        self.negative = [
            index
            for index, row in enumerate(self.rows)
            if row.get("negative_only")
        ]
        if not self.positive or not self.negative:
            raise ValueError("area training requires positive and negative rows")

    def __len__(self) -> int:
        return max(1, math.ceil(len(self.rows) / self.batch_size))

    def __iter__(self):
        rng = random.Random(self.seed)
        positive = list(self.positive)
        negative = list(self.negative)
        rng.shuffle(positive)
        rng.shuffle(negative)
        positive_index = 0
        negative_index = 0
        for _ in range(len(self)):
            batch: list[int] = []
            for slot in range(self.batch_size):
                use_positive = slot % 2 == 0
                if use_positive:
                    batch.append(positive[positive_index % len(positive)])
                    positive_index += 1
                else:
                    batch.append(negative[negative_index % len(negative)])
                    negative_index += 1
            yield batch


def _seed(seed: int) -> None:
    import random

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    try:
        torch.use_deterministic_algorithms(True, warn_only=True)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
    except Exception:
        pass


def _move_batch(batch, device):
    inputs = batch[0].to(device)
    targets = []
    for item in batch[1:]:
        if isinstance(item, dict):
            targets.append(
                {key: value.to(device) for key, value in item.items()}
            )
        else:
            targets.append(item.to(device))
    return inputs, targets


def _default_metric_fn(loss_fn: Callable):
    def metric_fn(model, loader, device) -> dict:
        model.eval()
        total = 0.0
        steps = 0
        with torch.no_grad():
            for batch in loader:
                inputs, targets = _move_batch(batch, device)
                outputs = model(inputs)
                loss = loss_fn(outputs, *targets)
                total += float(loss.detach().cpu())
                steps += 1
        return {"validation_loss": total / max(steps, 1)}

    return metric_fn


def fit_model(
    model,
    train_loader,
    val_loader,
    *,
    loss_fn,
    device,
    epochs: int = 40,
    learning_rate: float = 5e-4,
    weight_decay: float = 1e-4,
    ema_decay: float = 0.999,
    early_stopping_patience: int = 8,
    amp: bool = True,
    seed: int = SEED,
    checkpoint_path: str | Path | None = None,
    metric_key: str = "validation_loss",
    maximize: bool = False,
    load_best: bool = True,
    selector: ConstraintAwareSelector | None = None,
    validation_metric_fn: Callable | None = None,
) -> tuple[torch.nn.Module, dict]:
    _seed(seed)
    model = model.to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=learning_rate, weight_decay=weight_decay
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(epochs, 1), eta_min=learning_rate * 0.05
    )
    metric_fn = validation_metric_fn or _default_metric_fn(loss_fn)
    ema_state = None
    ema_updates = 0
    non_ema_state = None
    best_metric = None
    best_epoch = None
    best_state = None
    best_selection: dict | None = None
    curves: list[dict] = []
    epochs_without_improvement = 0
    early_stopped = False
    started = time.perf_counter()
    use_amp = bool(amp and torch.cuda.is_available())
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)
    if selector is not None and val_loader is None:
        raise ValueError(
            "constraint-aware selection requires a validation loader"
        )

    def update_ema() -> None:
        nonlocal ema_state, ema_updates
        if not 0.0 < ema_decay < 1.0:
            return
        ema_updates += 1
        with torch.no_grad():
            if ema_state is None:
                ema_state = {
                    key: value.detach().clone()
                    for key, value in model.state_dict().items()
                }
            else:
                effective_decay = min(
                    float(ema_decay),
                    float(1 + ema_updates) / float(10 + ema_updates),
                )
                for key, value in model.state_dict().items():
                    if torch.is_floating_point(value):
                        ema_state[key] = (
                            effective_decay * ema_state[key]
                            + (1.0 - effective_decay) * value.detach()
                        )
                    else:
                        ema_state[key] = value.detach().clone()

    def apply_ema() -> None:
        nonlocal non_ema_state
        if ema_state is None:
            return
        non_ema_state = {
            key: value.detach().clone()
            for key, value in model.state_dict().items()
        }
        model.load_state_dict(
            {key: value.clone() for key, value in ema_state.items()},
            strict=False,
        )

    def restore_non_ema() -> None:
        nonlocal non_ema_state
        if non_ema_state is None:
            return
        model.load_state_dict(non_ema_state)
        non_ema_state = None

    for epoch in range(1, epochs + 1):
        model.train()
        losses: list[float] = []
        for batch in train_loader:
            inputs, targets = _move_batch(batch, device)
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(
                device_type="cuda", enabled=use_amp, dtype=torch.float16
            ):
                outputs = model(inputs)
                loss = loss_fn(outputs, *targets)
            scaler.scale(loss).backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            scaler.step(optimizer)
            scaler.update()
            update_ema()
            losses.append(float(loss.detach().cpu()))
        training_loss = float(np.mean(losses)) if losses else 0.0
        curve: dict = {"epoch": epoch, "training_loss": training_loss}
        if val_loader is not None:
            ema_applied = False
            if ema_state is not None:
                apply_ema()
                ema_applied = True
            try:
                metrics = metric_fn(model, val_loader, device)
                evaluated_state = {
                    key: value.detach().cpu().clone()
                    for key, value in (
                        ema_state if ema_applied else model.state_dict()
                    ).items()
                }
            finally:
                if ema_applied:
                    restore_non_ema()
            curve.update(metrics)
            value = float(metrics[metric_key])
        else:
            value = training_loss
        curves.append(curve)
        if scheduler is not None:
            scheduler.step()
        selected = False
        if selector is not None:
            verdict = selector.consider(epoch, metrics)
            selected = bool(verdict["checkpoint_selected"])
            curve["selection"] = {
                "selected": bool(verdict["selected"]),
                "checkpoint_selected": selected,
                "product_eligible": bool(verdict["product_eligible"]),
                "selection_score": verdict["selection_score"],
                "selected_epoch": verdict["selected_epoch"],
                "violated_constraints": verdict["violated_constraints"],
            }
            if selected:
                best_metric = value
                best_epoch = epoch
                best_selection = selector.checkpoint_best()
        else:
            improved = best_metric is None or (
                value > best_metric if maximize else value < best_metric
            )
            selected = improved
        if selected:
            best_metric = value
            best_epoch = epoch
            best_state = evaluated_state if val_loader is not None else {
                key: value.detach().cpu().clone()
                for key, value in model.state_dict().items()
            }
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1
        if (
            early_stopping_patience > 0
            and epochs_without_improvement >= early_stopping_patience
        ):
            early_stopped = True
            break
    if best_state is not None and load_best:
        model.load_state_dict(best_state)
    checkpoint_path_obj = Path(checkpoint_path) if checkpoint_path else None
    if checkpoint_path_obj is not None:
        checkpoint_path_obj.parent.mkdir(parents=True, exist_ok=True)
        torch.save(
            {
                "best_epoch": best_epoch,
                "best_metric": best_metric,
                "state_dict": best_state,
                "ema_state": ema_state,
                "seed": seed,
                "selection": best_selection,
            },
            checkpoint_path_obj,
        )
    selection_summary = None
    if selector is not None:
        selection_summary = selector.best()
    report = {
        "epochs": len(curves),
        "curves": curves,
        "best_epoch": best_epoch,
        "best_metric": best_metric,
        "selection": selection_summary or best_selection,
        "early_stopped": early_stopped,
        "duration_s": time.perf_counter() - started,
        "device": str(device),
        "amp": use_amp,
        "ema_decay_target": ema_decay,
        "ema_warmup": "min(target,(1+updates)/(10+updates))",
        "ema_updates": ema_updates,
    }
    return model, report


def train_discovery(
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    device,
    epochs: int = 40,
    batch_size: int = 8,
    learning_rate: float = 5e-4,
    seed: int = 20260805,
    val_rows: list[dict] | None = None,
    checkpoint_path=None,
    early_stopping_patience: int = 8,
    load_best: bool = True,
    selector: ConstraintAwareSelector | None = None,
    validation_metric_fn: Callable | None = None,
    model=None,
    objectness_variant: str = "L2_independent_ohem",
) -> tuple[torch.nn.Module, dict]:
    assert_development_rows(rows, "discovery training")
    if val_rows is not None:
        assert_development_rows(val_rows, "discovery validation")
    dataset = G4DiscoveryDataset(rows, instances_by_key, augment=True, seed=seed)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=0,
        generator=torch.Generator().manual_seed(seed),
    )
    val_loader = None
    if val_rows:
        val_dataset = G4DiscoveryDataset(
            val_rows, instances_by_key, augment=False, seed=seed
        )
        val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
    if model is None:
        model = build_g4_model("discovery")
    loss_fn = lambda outputs, targets: discovery_loss(
        outputs,
        targets,
        objectness_variant=objectness_variant,
    )["total"]
    model, report = fit_model(
        model,
        loader,
        val_loader,
        loss_fn=loss_fn,
        device=device,
        epochs=epochs,
        learning_rate=learning_rate,
        seed=seed,
        checkpoint_path=checkpoint_path,
        early_stopping_patience=early_stopping_patience,
        load_best=load_best,
        selector=selector,
        validation_metric_fn=validation_metric_fn,
    )
    return model, report


def train_discovery_crop(
    samples: list[dict],
    *,
    device,
    epochs: int = 200,
    batch_size: int = 8,
    learning_rate: float = 4e-4,
    seed: int = 20260805,
    checkpoint_path=None,
    early_stopping_patience: int = 0,
    load_best: bool = False,
    augment: bool = True,
) -> tuple[torch.nn.Module, dict]:
    dataset = G4DiscoveryCropDataset(samples, augment=augment, seed=seed)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=0,
        generator=torch.Generator().manual_seed(seed),
    )
    model = build_g4_model("discovery")
    loss_fn = lambda outputs, targets: discovery_loss(outputs, targets)["total"]
    model, report = fit_model(
        model,
        loader,
        None,
        loss_fn=loss_fn,
        device=device,
        epochs=epochs,
        learning_rate=learning_rate,
        seed=seed,
        checkpoint_path=checkpoint_path,
        early_stopping_patience=early_stopping_patience,
        load_best=load_best,
    )
    return model, report


def train_classifier(
    samples: list[dict],
    *,
    device,
    epochs: int = 40,
    batch_size: int = 16,
    learning_rate: float = 8e-4,
    seed: int = 20260806,
    val_samples: list[dict] | None = None,
    checkpoint_path=None,
    early_stopping_patience: int = 8,
    load_best: bool = True,
    cache_crops: bool = False,
    selector: ConstraintAwareSelector | None = None,
    validation_metric_fn: Callable | None = None,
) -> tuple[torch.nn.Module, dict]:
    assert_development_rows(samples, "classifier training")
    if val_samples is not None:
        assert_development_rows(val_samples, "classifier validation")
    dataset = G4ClassifierDataset(
        samples,
        augment=True,
        seed=seed,
        cache_crops=cache_crops,
    )
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=0,
        generator=torch.Generator().manual_seed(seed),
    )
    val_loader = None
    if val_samples:
        val_loader = DataLoader(
            G4ClassifierDataset(val_samples, augment=False, seed=seed),
            batch_size=batch_size,
            shuffle=False,
        )
    model = build_g4_model("classifier")
    model, report = fit_model(
        model,
        loader,
        val_loader,
        loss_fn=classifier_loss,
        device=device,
        epochs=epochs,
        learning_rate=learning_rate,
        seed=seed,
        checkpoint_path=checkpoint_path,
        early_stopping_patience=early_stopping_patience,
        load_best=load_best,
        selector=selector,
        validation_metric_fn=validation_metric_fn,
    )
    return model, report


def train_area(
    task: str,
    rows: list[dict],
    *,
    device,
    epochs: int = 40,
    batch_size: int = 4,
    learning_rate: float = 4e-4,
    seed: int = 20260807,
    val_rows: list[dict] | None = None,
    checkpoint_path=None,
    early_stopping_patience: int = 8,
    load_best: bool = True,
    cache_frames: bool = False,
    crop_mode: str = "full",
    selector: ConstraintAwareSelector | None = None,
    validation_metric_fn: Callable | None = None,
) -> tuple[torch.nn.Module, dict]:
    if task not in ("leaf", "puddle"):
        raise ValueError(f"unknown area task {task}")
    assert_development_rows(rows, f"{task} training")
    if val_rows is not None:
        assert_development_rows(val_rows, f"{task} validation")
    channel = 0 if task == "leaf" else 1
    dataset = G4AreaDataset(
        rows,
        augment=True,
        seed=seed + channel,
        channel=channel,
        cache_frames=cache_frames,
        crop_mode=crop_mode,
    )
    loader = DataLoader(
        dataset,
        batch_sampler=_AreaBalancedSampler(rows, batch_size, seed + channel),
        num_workers=0,
    )
    val_loader = None
    if val_rows:
        val_loader = DataLoader(
            G4AreaDataset(
                val_rows,
                augment=False,
                seed=seed + channel,
                channel=channel,
                cache_frames=cache_frames,
                crop_mode="full",
            ),
            batch_size=batch_size,
            shuffle=False,
        )
    model = build_g4_model(task)
    loss_fn = lambda outputs, targets, boundaries: area_loss(
        outputs, targets, boundaries
    )["total"]
    model, report = fit_model(
        model,
        loader,
        val_loader,
        loss_fn=loss_fn,
        device=device,
        epochs=epochs,
        learning_rate=learning_rate,
        seed=seed + channel,
        checkpoint_path=checkpoint_path,
        early_stopping_patience=early_stopping_patience,
        load_best=load_best,
        selector=selector,
        validation_metric_fn=validation_metric_fn,
    )
    return model, report


__all__ = [
    "fit_model",
    "train_area",
    "train_classifier",
    "train_discovery",
    "train_discovery_crop",
]
