from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import random
import time

import cv2
import numpy as np
import yaml

from .assets import load_asset_registry
from .rendered import CLASS_ORDER, generate_frame


def _torch():
    try:
        import torch
        from torch import nn
    except ImportError as exc:
        raise RuntimeError("PyTorch is required inside the Stage5B training image") from exc
    return torch, nn


def build_model(candidate_id: str):
    torch, nn = _torch()
    if candidate_id == "candidate_a_pixel":
        return nn.Conv2d(3, len(CLASS_ORDER), kernel_size=1, bias=True)
    if candidate_id == "candidate_b_context":
        return nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=3, padding=1, bias=True),
            nn.ReLU(inplace=False),
            nn.Conv2d(24, 32, kernel_size=5, padding=4, dilation=2, bias=True),
            nn.ReLU(inplace=False),
            nn.Conv2d(32, 48, kernel_size=5, padding=6, dilation=3, bias=True),
            nn.ReLU(inplace=False),
            nn.Conv2d(48, 48, kernel_size=5, padding=8, dilation=4, bias=True),
            nn.ReLU(inplace=False),
            nn.Conv2d(48, 48, kernel_size=3, padding=6, dilation=6, bias=True),
            nn.ReLU(inplace=False),
            nn.Conv2d(48, len(CLASS_ORDER), kernel_size=1, bias=True),
        )
    raise ValueError(f"unknown candidate {candidate_id}")


class RenderedTorchDataset:
    def __init__(self, registry: dict, scene_seeds: list[int], frames_per_scene: int, augment: bool = False):
        self.registry = registry
        self.keys = [(seed, frame) for seed in scene_seeds for frame in range(frames_per_scene)]
        self.augment = augment

    def __len__(self):
        return len(self.keys)

    def __getitem__(self, index):
        torch, _ = _torch()
        seed, frame_index = self.keys[index]
        frame = generate_frame(seed, frame_index, self.registry)
        image_rgb = frame.image_rgb.copy()
        if self.augment:
            selector = (seed * 13 + frame_index * 7) % 20
            if selector in {0, 1, 2}:
                gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
                image_rgb = np.repeat(gray[:, :, None], 3, axis=2)
            elif selector in {3, 4, 5}:
                image_rgb = image_rgb[:, :, [2, 0, 1]]
            elif selector == 6:
                gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
                edges = cv2.Laplacian(gray, cv2.CV_8U, ksize=3)
                image_rgb = np.repeat(edges[:, :, None], 3, axis=2)
            elif selector == 7:
                gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
                edges = cv2.Canny(gray, 35, 110)
                image_rgb = np.repeat((255 - edges)[:, :, None], 3, axis=2)
            elif selector in {8, 9}:
                image_rgb = np.clip(image_rgb.astype(np.float32) * 0.55 + 12, 0, 255).astype(np.uint8)
        image = torch.from_numpy(np.transpose(image_rgb.astype(np.float32) / 255.0, (2, 0, 1)).copy())
        target = torch.from_numpy(frame.semantic_labels.astype(np.int64).copy())
        return image, target


def _macro_metrics(confusion: np.ndarray) -> dict:
    per_class = {}
    for class_index, class_id in enumerate(CLASS_ORDER[1:], 1):
        tp = int(confusion[class_index, class_index])
        fp = int(confusion[:, class_index].sum() - tp)
        fn = int(confusion[class_index, :].sum() - tp)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[class_id] = {"precision": precision, "recall": recall, "f1": f1}
    return {
        "per_class": per_class,
        "macro_precision": float(np.mean([item["precision"] for item in per_class.values()])),
        "macro_recall": float(np.mean([item["recall"] for item in per_class.values()])),
        "macro_f1": float(np.mean([item["f1"] for item in per_class.values()])),
    }


def _validate(model, loader, device) -> dict:
    torch, _ = _torch()
    confusion = np.zeros((len(CLASS_ORDER), len(CLASS_ORDER)), dtype=np.int64)
    model.eval()
    with torch.no_grad():
        for images, targets in loader:
            predictions = model(images.to(device)).argmax(dim=1).cpu().numpy()
            truth = targets.numpy()
            flat = truth.reshape(-1) * len(CLASS_ORDER) + predictions.reshape(-1)
            confusion += np.bincount(flat, minlength=len(CLASS_ORDER) ** 2).reshape(len(CLASS_ORDER), len(CLASS_ORDER))
    return {**_macro_metrics(confusion), "confusion_matrix": confusion.tolist()}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def train_candidates(registry_path: str | Path, config_path: str | Path, output: str | Path) -> dict:
    torch, _ = _torch()
    registry = load_asset_registry(registry_path)
    config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    seed = int(config["seed"])
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    try:
        torch.use_deterministic_algorithms(True, warn_only=True)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
    except Exception:
        pass
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    train_seeds = [value for value in range(500) if value % 10 <= 6][: int(config["training_scene_seeds"])]
    val_seeds = [value for value in range(500) if value % 10 == 7][: int(config["validation_scene_seeds"])]
    train_data = RenderedTorchDataset(registry, train_seeds, int(config["frames_per_scene"]) if "frames_per_scene" in config else 10, augment=True)
    val_data = RenderedTorchDataset(registry, val_seeds, int(config["frames_per_scene"]) if "frames_per_scene" in config else 10)
    train_loader = torch.utils.data.DataLoader(train_data, batch_size=int(config["batch_size"]), shuffle=True, num_workers=0, generator=torch.Generator().manual_seed(seed))
    val_loader = torch.utils.data.DataLoader(val_data, batch_size=int(config["batch_size"]), shuffle=False, num_workers=0)
    class_weights = torch.tensor([0.15, 2.5, 2.5, 2.5, 2.0, 2.0], dtype=torch.float32, device=device)
    candidate_reports = []
    for candidate in config["candidates"]:
        candidate_id = candidate["id"]
        if candidate.get("architecture") == "deferred":
            candidate_reports.append({"candidate_id": candidate_id, "trained": False, "status": "deferred_due_to_onnx_and_j6_operator_risk"})
            continue
        model = build_model(candidate_id).to(device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=float(config["learning_rate"]), weight_decay=float(config["weight_decay"]))
        loss_fn = torch.nn.CrossEntropyLoss(weight=class_weights)
        curves = []
        best_f1 = -1.0
        best_state = None
        started = time.perf_counter()
        candidate_epochs = 5 if candidate_id == "candidate_a_pixel" else int(config["epochs"])
        for epoch in range(candidate_epochs):
            model.train(); losses = []
            for images, targets in train_loader:
                images = images.to(device); targets = targets.to(device)
                optimizer.zero_grad(set_to_none=True)
                logits = model(images)
                probabilities = torch.softmax(logits, dim=1)
                one_hot = torch.nn.functional.one_hot(targets, num_classes=len(CLASS_ORDER)).permute(0, 3, 1, 2).float()
                intersection = (probabilities[:, 1:] * one_hot[:, 1:]).sum(dim=(0, 2, 3))
                denominator = probabilities[:, 1:].sum(dim=(0, 2, 3)) + one_hot[:, 1:].sum(dim=(0, 2, 3))
                dice_loss = 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
                loss = loss_fn(logits, targets) + 0.50 * dice_loss
                loss.backward(); optimizer.step(); losses.append(float(loss.detach().cpu()))
            validation = _validate(model, val_loader, device)
            curves.append({"epoch": epoch + 1, "training_loss": float(np.mean(losses)), "validation_macro_f1": validation["macro_f1"], "validation_macro_precision": validation["macro_precision"], "validation_macro_recall": validation["macro_recall"]})
            if validation["macro_f1"] > best_f1:
                best_f1 = validation["macro_f1"]
                best_state = {name: tensor.detach().cpu().clone() for name, tensor in model.state_dict().items()}
        model.load_state_dict(best_state)
        checkpoint = output / f"{candidate_id}_best.pt"
        torch.save({"candidate_id": candidate_id, "state_dict": best_state, "training_seed": seed, "class_order": CLASS_ORDER}, checkpoint)
        last_checkpoint = output / f"{candidate_id}_last.pt"
        torch.save({"candidate_id": candidate_id, "state_dict": {name: tensor.detach().cpu() for name, tensor in model.state_dict().items()}, "training_seed": seed, "class_order": CLASS_ORDER}, last_checkpoint)
        onnx_path = output / f"{candidate_id}.onnx"
        dummy = torch.zeros(tuple(int(value) for value in config["input_shape"]), device=device)
        model.eval()
        torch.onnx.export(model, dummy, onnx_path, input_names=["images"], output_names=["logits"], opset_version=int(config["opset"]), do_constant_folding=True, dynamic_axes=None)
        import onnx
        onnx_model = onnx.load(onnx_path); onnx.checker.check_model(onnx_model)
        operators = {}
        for node in onnx_model.graph.node:
            operators[node.op_type] = operators.get(node.op_type, 0) + 1
        final_validation = _validate(model, val_loader, device)
        report = {"candidate_id": candidate_id, "trained": True, "weight_source": "gradient_descent_cross_entropy_plus_dice", "handwritten_color_weights": False, "training_seed": seed, "training_scene_count": len(train_seeds), "training_frame_count": len(train_data), "validation_scene_count": len(val_seeds), "validation_frame_count": len(val_data), "epochs": candidate_epochs, "device": str(device), "duration_sec": time.perf_counter() - started, "parameter_count": sum(parameter.numel() for parameter in model.parameters()), "onnx_path": onnx_path.name, "onnx_sha256": _sha256(onnx_path), "onnx_bytes": onnx_path.stat().st_size, "operators": operators, "validation": final_validation, "curves": curves}
        candidate_reports.append(report)
        (output / f"{candidate_id}_training_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    trained = [item for item in candidate_reports if item.get("trained")]
    selected = max(trained, key=lambda item: (item["validation"]["macro_f1"], -item["onnx_bytes"]))
    selected_path = output / selected["onnx_path"]
    final_path = output / "stage5b_learned_perception.onnx"
    final_path.write_bytes(selected_path.read_bytes())
    environment = {"python": platform.python_version(), "platform": platform.platform(), "torch": torch.__version__, "cuda_available": torch.cuda.is_available(), "cuda_version": torch.version.cuda, "device": str(device), "deterministic_algorithms": "warn_only_due_to_cuda_nll_loss2d", "cudnn_deterministic": True, "cudnn_benchmark": False, "onnx": __import__("onnx").__version__, "opencv": cv2.__version__, "numpy": np.__version__}
    report = {"schema_version": 1, "stage": "Stage5B", "selection_basis": ["validation_macro_f1", "model_size", "basic_onnx_operators", "j6_compatibility_risk"], "candidates": candidate_reports, "selected_candidate": selected["candidate_id"], "selected_model": final_path.name, "selected_model_sha256": _sha256(final_path), "training_config": config, "environment_lock": environment, "code_commit": os.environ.get("STAGE5B_CODE_COMMIT", "working_tree_precommit"), "test_split_used_for_selection": False}
    (output / "model_selection_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (output / "environment_lock.json").write_text(json.dumps(environment, indent=2) + "\n", encoding="utf-8")
    model_card = {"schema_version": 1, "model_id": "stage5b_learned_perception_v1", "architecture": selected["candidate_id"], "weight_source": "trained_from_random_initialization_on_D1_procedural_rendered", "handwritten_color_weights": False, "input_shape": config["input_shape"], "class_order": list(CLASS_ORDER), "opset": config["opset"], "domain": "D1_procedural_rendered", "license": "Apache-2.0", "intended_use": "ROS/Gazebo research and Stage5B evaluation", "limitations": ["not evaluated on D2 real data", "procedural D1 is not equivalent to real Gazebo camera rendering", "no J6 board evidence", "no competition accuracy claim"]}
    (output / "model_card.json").write_text(json.dumps(model_card, indent=2) + "\n", encoding="utf-8")
    return report
