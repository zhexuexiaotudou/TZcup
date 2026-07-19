from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import yaml

from .assets import load_asset_registry
from .evaluation import _stress
from .models import _torch, build_model
from .rendered import CLASS_ORDER, generate_frame


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _confusion(truth: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    flat = truth.reshape(-1) * len(CLASS_ORDER) + prediction.reshape(-1)
    return np.bincount(flat, minlength=len(CLASS_ORDER) ** 2).reshape(
        len(CLASS_ORDER), len(CLASS_ORDER)
    )


def _metrics(confusion: np.ndarray) -> dict:
    per_class = {}
    for index, class_id in enumerate(CLASS_ORDER):
        tp = int(confusion[index, index])
        fp = int(confusion[:, index].sum() - tp)
        fn = int(confusion[index, :].sum() - tp)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2.0 * precision * recall / max(precision + recall, 1e-12)
        per_class[class_id] = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "iou": tp / max(tp + fp + fn, 1),
            "support_pixels": int(confusion[index].sum()),
            "predicted_pixels": int(confusion[:, index].sum()),
        }
    foreground = [per_class[name] for name in CLASS_ORDER[1:]]
    return {
        "per_class": per_class,
        "macro_precision": float(np.mean([item["precision"] for item in foreground])),
        "macro_recall": float(np.mean([item["recall"] for item in foreground])),
        "macro_f1": float(np.mean([item["f1"] for item in foreground])),
        "foreground_miou": float(np.mean([item["iou"] for item in foreground])),
        "confusion_matrix": confusion.tolist(),
    }


def _tensor(image_rgb: np.ndarray):
    torch, _ = _torch()
    array = np.transpose(image_rgb.astype(np.float32) / 255.0, (2, 0, 1))
    return torch.from_numpy(np.ascontiguousarray(array))


def _dataset(registry: dict, keys: list[tuple[int, int]]):
    torch, _ = _torch()
    frames = [generate_frame(seed, frame_index, registry) for seed, frame_index in keys]
    images = torch.stack([_tensor(frame.image_rgb) for frame in frames])
    targets = torch.from_numpy(
        np.stack([frame.semantic_labels for frame in frames]).astype(np.int64)
    )
    return frames, images, targets


def _evaluate(model, images, targets, device) -> tuple[dict, np.ndarray, np.ndarray]:
    model.eval()
    torch, _ = _torch()
    with torch.no_grad():
        logits = model(images.to(device)).cpu()
    predictions = logits.argmax(1).numpy().astype(np.uint8)
    confusion = np.zeros((len(CLASS_ORDER), len(CLASS_ORDER)), dtype=np.int64)
    for truth, prediction in zip(targets.numpy(), predictions):
        confusion += _confusion(truth, prediction)
    return _metrics(confusion), logits.numpy(), predictions


def _save_overlay(path: Path, frame, prediction: np.ndarray) -> None:
    palette = np.asarray(
        [(30, 30, 30), (64, 180, 255), (220, 100, 70),
         (245, 220, 80), (80, 190, 90), (60, 120, 230)], dtype=np.uint8
    )
    canvas = np.concatenate(
        (frame.image_rgb, palette[frame.semantic_labels], palette[prediction]), axis=1
    )
    cv2.imwrite(str(path), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))


def _class_weights(targets, device):
    torch, _ = _torch()
    counts = torch.bincount(targets.reshape(-1), minlength=len(CLASS_ORDER)).float()
    frequencies = counts / counts.sum()
    weights = 1.0 / torch.sqrt(torch.clamp(frequencies, min=1e-6))
    weights = weights / weights[1:].mean()
    weights[0] = 0.12
    return weights.to(device), counts.long().tolist()


def _loss(logits, targets, weights):
    torch, _ = _torch()
    ce = torch.nn.functional.cross_entropy(logits, targets, weight=weights)
    probabilities = torch.softmax(logits, dim=1)
    one_hot = torch.nn.functional.one_hot(
        targets, num_classes=len(CLASS_ORDER)
    ).permute(0, 3, 1, 2).float()
    intersection = (probabilities[:, 1:] * one_hot[:, 1:]).sum((0, 2, 3))
    denominator = probabilities[:, 1:].sum((0, 2, 3)) + one_hot[:, 1:].sum((0, 2, 3))
    dice = 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
    return ce + 0.75 * dice, ce, dice


def _train_micro(registry: dict, config: dict, output: Path) -> tuple[dict, Path]:
    torch, _ = _torch()
    seed = int(config["seed"])
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    keys = [
        (int(scene_seed), frame_index)
        for scene_seed in config["micro_scene_seeds"]
        for frame_index in range(int(config["micro_frames_per_scene"]))
    ]
    frames, images, targets = _dataset(registry, keys)
    class_weights, class_histogram = _class_weights(targets, device)
    model = build_model("stage5br_micro_unet").to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=float(config["learning_rate"]),
        weight_decay=float(config["weight_decay"])
    )
    loader = torch.utils.data.DataLoader(
        torch.utils.data.TensorDataset(images, targets),
        batch_size=int(config["batch_size"]), shuffle=True, num_workers=0,
        generator=torch.Generator().manual_seed(seed)
    )
    epochs = []
    best = None
    best_state = None
    started = time.perf_counter()
    for epoch in range(1, int(config["maximum_epochs"]) + 1):
        model.train()
        losses, ce_losses, dice_losses, gradient_norms = [], [], [], []
        for batch_images, batch_targets in loader:
            batch_images, batch_targets = batch_images.to(device), batch_targets.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(batch_images)
            loss, ce, dice = _loss(logits, batch_targets, class_weights)
            loss.backward()
            gradient_norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 20.0)
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
            ce_losses.append(float(ce.detach().cpu()))
            dice_losses.append(float(dice.detach().cpu()))
            gradient_norms.append(float(gradient_norm.detach().cpu()))
        metrics, logits_np, predictions = _evaluate(model, images, targets, device)
        epochs.append({
            "epoch": epoch, "loss": float(np.mean(losses)),
            "cross_entropy": float(np.mean(ce_losses)),
            "dice_loss": float(np.mean(dice_losses)),
            "gradient_norm": float(np.mean(gradient_norms)),
            "learning_rate": float(optimizer.param_groups[0]["lr"]),
            "macro_f1": metrics["macro_f1"],
            "foreground_miou": metrics["foreground_miou"],
            "per_class_iou": {name: metrics["per_class"][name]["iou"] for name in CLASS_ORDER[1:]},
            "logits_min": float(logits_np.min()), "logits_max": float(logits_np.max()),
            "logits_mean": float(logits_np.mean()), "logits_std": float(logits_np.std()),
        })
        score = (metrics["macro_f1"], metrics["foreground_miou"])
        if best is None or score > best:
            best = score
            best_state = {name: value.detach().cpu().clone() for name, value in model.state_dict().items()}
        if (metrics["macro_f1"] >= float(config["micro_macro_f1_gate"])
                and metrics["foreground_miou"] >= float(config["micro_miou_gate"])):
            break
    model.load_state_dict(best_state)
    final_metrics, final_logits, predictions = _evaluate(model, images, targets, device)
    checkpoint = output / "stage5br_micro_overfit.pt"
    torch.save({"architecture": "stage5br_micro_unet", "state_dict": best_state,
                "class_order": CLASS_ORDER, "seed": seed, "keys": keys}, checkpoint)
    for index in range(min(4, len(frames))):
        _save_overlay(output / f"micro_overlay_{index:02d}.png", frames[index], predictions[index])
    histogram_counts, histogram_edges = np.histogram(final_logits, bins=64)
    hard_negatives = sorted({item["negative_id"] for frame in frames for item in frame.negatives})
    report = {
        "schema_version": 1, "phase": "Stage5BR Phase A1 micro-overfit",
        "architecture": "stage5br_micro_unet", "seed": seed,
        "sample_count": len(frames), "scene_frame_keys": keys,
        "class_order": list(CLASS_ORDER),
        "all_five_target_classes_present": all(count > 0 for count in class_histogram[1:]),
        "hard_negative_types": hard_negatives, "hard_negative_type_count": len(hard_negatives),
        "at_least_four_hard_negatives_present": len(hard_negatives) >= 4,
        "class_histogram_pixels": dict(zip(CLASS_ORDER, class_histogram)),
        "class_weights": dict(zip(CLASS_ORDER, class_weights.detach().cpu().tolist())),
        "epochs_executed": len(epochs), "duration_sec": time.perf_counter() - started,
        "device": str(device), "curves": epochs, "final_metrics": final_metrics,
        "logits_histogram": {"counts": histogram_counts.tolist(), "bin_edges": histogram_edges.tolist()},
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "micro_overfit_pass": (
            final_metrics["macro_f1"] >= float(config["micro_macro_f1_gate"])
            and final_metrics["foreground_miou"] >= float(config["micro_miou_gate"])
            and all(count > 0 for count in class_histogram[1:]) and len(hard_negatives) >= 4
        ),
    }
    _write_json(output / "micro_overfit_report.json", report)
    return report, checkpoint


def _pipeline_parity(registry: dict, config: dict, checkpoint: Path, output: Path) -> dict:
    import onnx
    import onnxruntime as ort
    from sanitation_perception.preprocessing import preprocess_rgb, resize_labels

    torch, _ = _torch()
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    model = build_model(payload["architecture"])
    model.load_state_dict(payload["state_dict"])
    model.eval()
    frames = [generate_frame(seed, frame, registry) for seed, frame in payload["keys"][:4]]
    tensors = np.concatenate([preprocess_rgb(frame.image_rgb) for frame in frames])
    training_tensors = np.stack([
        np.transpose(frame.image_rgb.astype(np.float32) / 255.0, (2, 0, 1))
        for frame in frames
    ])
    reconstruction_error = float(np.max(np.abs(tensors - training_tensors)))
    with torch.no_grad():
        pytorch_logits = model(torch.from_numpy(tensors)).numpy()
    onnx_path = output / "stage5br_micro_overfit.onnx"
    torch.onnx.export(
        model, torch.from_numpy(tensors[:1]), onnx_path,
        input_names=["images"], output_names=["logits"],
        opset_version=int(config["opset"]),
        dynamic_axes={"images": {0: "batch"}, "logits": {0: "batch"}},
        do_constant_folding=True,
    )
    checked = onnx.load(onnx_path)
    onnx.checker.check_model(checked)
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    onnx_logits = session.run(["logits"], {"images": tensors})[0]
    max_error = float(np.max(np.abs(pytorch_logits - onnx_logits)))
    agreement = float(np.mean(pytorch_logits.argmax(1) == onnx_logits.argmax(1)))
    labels_round_trip = all(
        np.array_equal(frame.semantic_labels, resize_labels(
            frame.semantic_labels, frame.semantic_labels.shape[1], frame.semantic_labels.shape[0]
        )) for frame in frames
    )
    operators = {}
    for node in checked.graph.node:
        operators[node.op_type] = operators.get(node.op_type, 0) + 1
    report = {
        "schema_version": 1, "phase": "Stage5BR Phase A2 pipeline parity",
        "raw_label_to_training_target_exact": True,
        "raw_rgb_to_training_tensor_max_error": reconstruction_error,
        "checkpoint_load_exact": all(
            torch.equal(payload["state_dict"][name], value)
            for name, value in model.state_dict().items()
        ),
        "pytorch_onnx_max_logit_error": max_error,
        "pytorch_onnx_argmax_agreement": agreement,
        "class_order": list(CLASS_ORDER),
        "checkpoint_class_order": list(payload["class_order"]),
        "class_order_identical": tuple(payload["class_order"]) == CLASS_ORDER,
        "rgb_bgr_contract": "cv_bridge desired_encoding=rgb8; preprocess_rgb requires RGB",
        "normalization": "float32 divide by 255.0", "resize": [128, 96],
        "mask_interpolation": "nearest", "label_resize_round_trip_exact": labels_round_trip,
        "onnx_operators": operators,
        "onnx_sha256": hashlib.sha256(onnx_path.read_bytes()).hexdigest(),
        "pipeline_parity_pass": (
            reconstruction_error == 0.0
            and max_error <= float(config["pipeline_max_logit_error_gate"])
            and agreement >= float(config["pipeline_argmax_agreement_gate"])
            and tuple(payload["class_order"]) == CLASS_ORDER and labels_round_trip
        ),
    }
    _write_json(output / "pipeline_parity_report.json", report)
    return report


def _forced_registry(registry: dict, layer: str) -> dict:
    result = copy.deepcopy(registry)
    contract = result["split_contract"]
    train_indices = list(contract["train_variant_indices"])
    train_worlds = list(contract["train_worlds"])
    if layer == "same_assets_unseen_seeds":
        contract["val_variant_indices"] = train_indices
        contract["val_worlds"] = train_worlds
    elif layer == "unseen_textures":
        unseen_index = int(contract["val_variant_indices"][0])
        for spec in result["classes"].values():
            unseen_texture = spec["variants"][unseen_index]["texture"]
            for index in train_indices:
                spec["variants"][index]["texture"] = unseen_texture
        contract["val_variant_indices"] = train_indices
        contract["val_worlds"] = train_worlds
    elif layer == "unseen_assets":
        contract["val_worlds"] = train_worlds
    elif layer != "unseen_world":
        raise ValueError(layer)
    return result


def _diagnose_logits(logits: np.ndarray, targets, predictions: np.ndarray) -> dict:
    shifted = logits - logits.max(axis=1, keepdims=True)
    probability = np.exp(shifted)
    probability /= probability.sum(axis=1, keepdims=True)
    entropy = -(probability * np.log(np.clip(probability, 1e-12, 1.0))).sum(axis=1)
    confidence = probability.max(axis=1)
    truth = targets.numpy()
    per_class_loss = {}
    for index, class_id in enumerate(CLASS_ORDER):
        selected = truth == index
        values = -np.log(np.clip(probability[:, index][selected], 1e-12, 1.0))
        per_class_loss[class_id] = float(values.mean()) if values.size else None
    fp_sources = {}
    for predicted_index, class_id in enumerate(CLASS_ORDER[1:], 1):
        sources = np.bincount(truth[predictions == predicted_index], minlength=len(CLASS_ORDER))
        fp_sources[class_id] = {
            CLASS_ORDER[index]: int(count) for index, count in enumerate(sources)
            if index != predicted_index and count
        }
    return {
        "truth_pixel_histogram": {name: int((truth == index).sum()) for index, name in enumerate(CLASS_ORDER)},
        "prediction_pixel_histogram": {name: int((predictions == index).sum()) for index, name in enumerate(CLASS_ORDER)},
        "mean_entropy": float(entropy.mean()), "mean_confidence": float(confidence.mean()),
        "per_class_cross_entropy": per_class_loss, "false_positive_truth_sources": fp_sources,
    }


def _split_ladder(registry: dict, config: dict, checkpoint: Path, output: Path) -> tuple[dict, dict]:
    torch, _ = _torch()
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    model = build_model(payload["architecture"])
    model.load_state_dict(payload["state_dict"])
    model.to(torch.device("cpu"))
    count = int(config["ladder_scene_count"])
    frames_per_scene = int(config["ladder_frames_per_scene"])
    val_seeds = [value for value in range(2000) if value % 10 == 7][:count]
    layers = [("train", registry, payload["keys"])]
    for name in ("same_assets_unseen_seeds", "unseen_textures", "unseen_assets", "unseen_world"):
        keys = [(seed, frame) for seed in val_seeds for frame in range(frames_per_scene)]
        layers.append((name, _forced_registry(registry, name), keys))
    results, collapse_details = [], {}
    for name, layer_registry, keys in layers:
        _, images, targets = _dataset(layer_registry, keys)
        metrics, logits, predictions = _evaluate(model, images, targets, torch.device("cpu"))
        results.append({"layer": name, "sample_count": len(keys), **metrics})
        collapse_details[name] = _diagnose_logits(logits, targets, predictions)
    stress_registry = _forced_registry(registry, "unseen_world")
    stress_keys = [(seed, frame) for seed in val_seeds for frame in range(frames_per_scene)]
    stress_frames, _, stress_targets = _dataset(stress_registry, stress_keys)
    stress_metrics = []
    for stress_name in (
        "grayscale", "hue_shift", "color_permutation", "exposure_extremes",
        "background_color_swap", "texture_only", "shape_only", "blue_patch_puddle_confuser",
    ):
        transformed = np.stack([_stress(frame.image_rgb, stress_name) for frame in stress_frames])
        tensor = torch.from_numpy(np.ascontiguousarray(
            np.transpose(transformed.astype(np.float32) / 255.0, (0, 3, 1, 2))
        ))
        metrics, _, _ = _evaluate(model, tensor, stress_targets, torch.device("cpu"))
        stress_metrics.append({"stress": stress_name, **metrics})
    first_collapse = next(
        (item["layer"] for item in results[1:] if item["macro_f1"] < 0.70), None
    )
    report = {
        "schema_version": 1, "phase": "Stage5BR Phase A3 split ladder",
        "layers": results, "color_stress": stress_metrics,
        "first_collapse_layer_below_macro_f1_0_70": first_collapse,
        "test_split_used_for_model_selection": False,
    }
    collapse = {
        "schema_version": 1, "phase": "Stage5BR Phase A4 collapse audit",
        "layers": collapse_details,
        "interpretation_boundary": "diagnostic P1 procedural screening only; not a G1 accuracy claim",
    }
    _write_json(output / "split_ladder_report.json", report)
    _write_json(output / "collapse_audit_report.json", collapse)
    return report, collapse


def run(registry_path: str | Path, config_path: str | Path, output_path: str | Path) -> dict:
    output = Path(output_path)
    output.mkdir(parents=True, exist_ok=True)
    registry = load_asset_registry(registry_path)
    config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    micro, checkpoint = _train_micro(registry, config, output)
    parity = _pipeline_parity(registry, config, checkpoint, output)
    ladder, _ = _split_ladder(registry, config, checkpoint, output)
    report = {
        "schema_version": 1, "stage": "Stage5BR training-chain self-audit",
        "micro_overfit_pass": micro["micro_overfit_pass"],
        "pipeline_parity_pass": parity["pipeline_parity_pass"],
        "first_collapse_layer": ladder["first_collapse_layer_below_macro_f1_0_70"],
        "phase_a_pass": micro["micro_overfit_pass"] and parity["pipeline_parity_pass"],
        "P1_only": True, "G1_accuracy_claim": False,
        "READY_FOR_GPT_REVIEW_STAGE5B": False, "READY_FOR_STAGE5C": False,
    }
    _write_json(output / "stage5br_phase_a_summary.json", report)
    return report


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    report = run(args.registry, args.config, args.output)
    print(json.dumps(report, indent=2))
    if not report["phase_a_pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
