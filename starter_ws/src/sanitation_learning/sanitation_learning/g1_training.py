from __future__ import annotations

import hashlib
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import yaml

from .evaluation import _stress
from .models import _torch, build_model
from .rendered import CLASS_ORDER
from .stage5br_audit import _confusion, _metrics, _save_overlay, _write_json


def _load_records(root: Path):
    manifest = json.loads((root / "g1_dataset_manifest.json").read_text(encoding="utf-8"))
    scene_split = {
        int(seed): split for split, seeds in manifest["split_scene_seeds"].items() for seed in seeds
    }
    records = []
    for record in manifest["records"]:
        seed = int(record["scene_seed"])
        base = root / "scenes" / f"scene_{seed:04d}"
        records.append({
            "scene_seed": seed, "split": scene_split[seed],
            "image": base / record["paths"]["image"],
            "semantic": base / record["paths"]["semantic"],
        })
    return manifest, records


class G1Dataset:
    def __init__(self, records: list[dict], augment: bool, seed: int):
        torch, _ = _torch()
        self.images = []
        self.targets = []
        for record in records:
            bgr = cv2.imread(str(record["image"]), cv2.IMREAD_COLOR)
            self.images.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            self.targets.append(np.load(record["semantic"], allow_pickle=False).astype(np.int64))
        self.augment = augment
        self.seed = seed
        self.epoch = 0
        self.torch = torch

    def __len__(self):
        return len(self.images)

    def set_epoch(self, epoch: int):
        self.epoch = int(epoch)

    def __getitem__(self, index):
        image = self.images[index].copy()
        target = self.targets[index].copy()
        if self.augment:
            rng = np.random.default_rng(self.seed + self.epoch * 100003 + index * 97)
            if rng.random() < 0.5:
                image, target = image[:, ::-1], target[:, ::-1]
            if rng.random() < 0.35:
                image, target = image[::-1], target[::-1]
            rotations = int(rng.integers(0, 4))
            if rotations:
                image = np.rot90(image, rotations).copy()
                target = np.rot90(target, rotations).copy()
                image = cv2.resize(image, (128, 96), interpolation=cv2.INTER_AREA)
                target = cv2.resize(target.astype(np.uint8), (128, 96), interpolation=cv2.INTER_NEAREST).astype(np.int64)
            image = np.clip(
                image.astype(np.float32) * rng.uniform(0.65, 1.35)
                + rng.uniform(-18, 18), 0, 255
            ).astype(np.uint8)
            color_mode = rng.random()
            if color_mode < 0.12:
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                image = np.repeat(gray[:, :, None], 3, axis=2)
            elif color_mode < 0.24:
                image = image[:, :, rng.permutation(3)]
            elif color_mode < 0.34:
                hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
                hsv[:, :, 0] = (hsv[:, :, 0].astype(np.int16) + int(rng.integers(25, 155))) % 180
                image = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        tensor = np.ascontiguousarray(np.transpose(image.astype(np.float32) / 255.0, (2, 0, 1)))
        return self.torch.from_numpy(tensor), self.torch.from_numpy(np.ascontiguousarray(target))


def _evaluate(model, dataset: G1Dataset, device, batch_size: int) -> tuple[dict, np.ndarray, np.ndarray]:
    torch, _ = _torch()
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=0)
    confusion = np.zeros((len(CLASS_ORDER), len(CLASS_ORDER)), dtype=np.int64)
    all_logits, all_predictions = [], []
    model.eval()
    with torch.no_grad():
        for images, targets in loader:
            logits = model(images.to(device)).cpu().numpy()
            predictions = logits.argmax(1).astype(np.uint8)
            for truth, prediction in zip(targets.numpy(), predictions):
                confusion += _confusion(truth, prediction)
            all_logits.append(logits)
            all_predictions.append(predictions)
    metrics = _metrics(confusion)
    metrics["leaf_puddle_miou"] = float(np.mean([
        metrics["per_class"]["leaf_pile"]["iou"], metrics["per_class"]["puddle"]["iou"]
    ]))
    return metrics, np.concatenate(all_logits), np.concatenate(all_predictions)


def _color_stress(model, dataset: G1Dataset, device, batch_size: int) -> dict:
    torch, _ = _torch()
    names = (
        "grayscale", "hue_shift", "color_permutation", "exposure_extremes",
        "background_color_swap", "texture_only", "shape_only", "blue_patch_puddle_confuser",
    )
    results = []
    for name in names:
        confusion = np.zeros((len(CLASS_ORDER), len(CLASS_ORDER)), dtype=np.int64)
        for start in range(0, len(dataset), batch_size):
            images = np.stack([_stress(dataset.images[index], name) for index in range(start, min(start + batch_size, len(dataset)))])
            targets = np.stack(dataset.targets[start:start + batch_size])
            tensor = torch.from_numpy(np.ascontiguousarray(np.transpose(images.astype(np.float32) / 255.0, (0, 3, 1, 2))))
            with torch.no_grad():
                predictions = model(tensor.to(device)).argmax(1).cpu().numpy().astype(np.uint8)
            for truth, prediction in zip(targets, predictions):
                confusion += _confusion(truth, prediction)
        results.append({"stress": name, **_metrics(confusion)})
    return {"cases": results, "aggregate_macro_f1": float(np.mean([item["macro_f1"] for item in results]))}


def _loss(logits, targets, weights):
    torch, _ = _torch()
    ce = torch.nn.functional.cross_entropy(logits, targets, weight=weights, reduction="none")
    probabilities = torch.softmax(logits, dim=1)
    true_probability = probabilities.gather(1, targets[:, None]).squeeze(1)
    focal = (((1.0 - true_probability) ** 1.5) * ce).mean()
    one_hot = torch.nn.functional.one_hot(targets, len(CLASS_ORDER)).permute(0, 3, 1, 2).float()
    intersection = (probabilities[:, 1:] * one_hot[:, 1:]).sum((0, 2, 3))
    denominator = probabilities[:, 1:].sum((0, 2, 3)) + one_hot[:, 1:].sum((0, 2, 3))
    dice = 1.0 - ((2 * intersection + 1) / (denominator + 1)).mean()
    return focal + dice, focal, dice


def train(dataset_root: str | Path, config_path: str | Path, output_path: str | Path) -> dict:
    torch, _ = _torch()
    root, output = Path(dataset_root), Path(output_path)
    output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    manifest, records = _load_records(root)
    train_scene_ids = sorted(manifest["split_scene_seeds"]["train"])
    in_domain_val_ids = set(train_scene_ids[-5:])
    optimization_ids = set(train_scene_ids[:-5])
    train_records = [record for record in records if record["scene_seed"] in optimization_ids]
    in_domain_records = [record for record in records if record["scene_seed"] in in_domain_val_ids]
    cross_records = [record for record in records if record["split"] == "val"]
    test_records = [record for record in records if record["split"] == "test"]
    seed = int(config["seed"])
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    if torch.cuda.is_available(): torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    train_data = G1Dataset(train_records, True, seed)
    in_domain_data = G1Dataset(in_domain_records, False, seed)
    cross_data = G1Dataset(cross_records, False, seed)
    test_data = G1Dataset(test_records, False, seed)
    all_targets = torch.from_numpy(np.stack(train_data.targets))
    counts = torch.bincount(all_targets.reshape(-1), minlength=len(CLASS_ORDER)).float()
    weights = (counts.sum() / torch.clamp(counts * len(CLASS_ORDER), min=1.0)).pow(0.65)
    weights = torch.clamp(weights, 0.1, 20.0); weights[0] = 0.06; weights = weights.to(device)
    model = build_model("stage5br_g1_unet").to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=float(config["learning_rate"]), weight_decay=float(config["weight_decay"]))
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=int(config["maximum_epochs"]), eta_min=2e-5)
    loader = torch.utils.data.DataLoader(
        train_data, batch_size=int(config["batch_size"]), shuffle=True, num_workers=0,
        generator=torch.Generator().manual_seed(seed)
    )
    curves, best_state, best_score, stale = [], None, None, 0
    started = time.perf_counter()
    for epoch in range(1, int(config["maximum_epochs"]) + 1):
        train_data.set_epoch(epoch); model.train(); losses = []; focals = []; dices = []; gradients = []
        for images, targets in loader:
            images, targets = images.to(device), targets.to(device)
            optimizer.zero_grad(set_to_none=True); logits = model(images)
            loss, focal, dice = _loss(logits, targets, weights)
            loss.backward(); gradient = torch.nn.utils.clip_grad_norm_(model.parameters(), 20.0)
            optimizer.step()
            losses.append(float(loss.detach().cpu())); focals.append(float(focal.detach().cpu()))
            dices.append(float(dice.detach().cpu())); gradients.append(float(gradient.detach().cpu()))
        scheduler.step()
        if epoch % int(config["validation_interval_epochs"]) != 0 and epoch != int(config["maximum_epochs"]):
            continue
        in_domain, _, _ = _evaluate(model, in_domain_data, device, int(config["batch_size"]))
        cross, _, _ = _evaluate(model, cross_data, device, int(config["batch_size"]))
        score = (cross["macro_f1"], cross["leaf_puddle_miou"], in_domain["macro_f1"])
        curves.append({
            "epoch": epoch, "training_loss": float(np.mean(losses)),
            "focal_loss": float(np.mean(focals)), "dice_loss": float(np.mean(dices)),
            "gradient_norm": float(np.mean(gradients)), "learning_rate": optimizer.param_groups[0]["lr"],
            "in_domain_macro_f1": in_domain["macro_f1"], "in_domain_leaf_puddle_miou": in_domain["leaf_puddle_miou"],
            "cross_asset_world_macro_f1": cross["macro_f1"], "cross_asset_world_leaf_puddle_miou": cross["leaf_puddle_miou"],
        })
        if best_score is None or score > best_score:
            best_score = score; stale = 0
            best_state = {name: value.detach().cpu().clone() for name, value in model.state_dict().items()}
        else:
            stale += 1
        if stale >= int(config["patience_intervals"]):
            break
    model.load_state_dict(best_state)
    in_domain, _, in_predictions = _evaluate(model, in_domain_data, device, int(config["batch_size"]))
    cross, _, cross_predictions = _evaluate(model, cross_data, device, int(config["batch_size"]))
    color = _color_stress(model, cross_data, device, int(config["batch_size"]))
    test, _, _ = _evaluate(model, test_data, device, int(config["batch_size"]))
    checkpoint = output / "stage5br_g1_baseline.pt"
    torch.save({"architecture": "stage5br_g1_unet", "state_dict": best_state, "class_order": CLASS_ORDER, "seed": seed}, checkpoint)
    onnx_path = output / "stage5br_g1_baseline.onnx"
    model.cpu().eval()
    torch.onnx.export(model, torch.zeros((1, 3, 96, 128)), onnx_path, input_names=["images"], output_names=["logits"], opset_version=int(config["opset"]), do_constant_folding=True)
    for index in range(min(4, len(in_domain_data))):
        frame = type("Frame", (), {"image_rgb": in_domain_data.images[index], "semantic_labels": in_domain_data.targets[index]})
        _save_overlay(output / f"g1_in_domain_overlay_{index:02d}.png", frame, in_predictions[index])
    for index in range(min(4, len(cross_data))):
        frame = type("Frame", (), {"image_rgb": cross_data.images[index], "semantic_labels": cross_data.targets[index]})
        _save_overlay(output / f"g1_cross_asset_overlay_{index:02d}.png", frame, cross_predictions[index])
    screening_pass = all([
        in_domain["macro_f1"] >= float(config["in_domain_macro_f1_gate"]),
        cross["leaf_puddle_miou"] >= float(config["leaf_puddle_miou_gate"]),
        cross["macro_f1"] >= float(config["cross_asset_world_macro_f1_gate"]),
        color["aggregate_macro_f1"] >= float(config["color_stress_macro_f1_gate"]),
    ])
    report = {
        "schema_version": 1, "stage": "Stage5BR G1 model recovery screening",
        "weight_source": "random_initialization_gradient_training_on_G1_actual_gazebo_camera",
        "dataset_id": manifest["dataset_id"], "training_scene_ids": sorted(optimization_ids),
        "in_domain_validation_scene_ids": sorted(in_domain_val_ids),
        "cross_asset_world_validation_scene_ids": manifest["split_scene_seeds"]["val"],
        "held_out_test_scene_ids": manifest["split_scene_seeds"]["test"],
        "test_split_used_for_model_selection": False,
        "class_pixel_counts": dict(zip(CLASS_ORDER, counts.long().tolist())),
        "class_weights": dict(zip(CLASS_ORDER, weights.detach().cpu().tolist())),
        "device": str(device), "duration_sec": time.perf_counter() - started,
        "epochs_executed": curves[-1]["epoch"], "curves": curves,
        "in_domain": in_domain, "cross_asset_world": cross,
        "color_stress": color, "held_out_test_after_selection": test,
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "onnx_sha256": hashlib.sha256(onnx_path.read_bytes()).hexdigest(),
        "screening_gates": {
            "in_domain_macro_f1": float(config["in_domain_macro_f1_gate"]),
            "leaf_puddle_miou": float(config["leaf_puddle_miou_gate"]),
            "cross_asset_world_macro_f1": float(config["cross_asset_world_macro_f1_gate"]),
            "color_stress_macro_f1": float(config["color_stress_macro_f1_gate"]),
        },
        "G1_model_screening_pass": screening_pass,
        "READY_FOR_GPT_REVIEW_STAGE5B": False, "READY_FOR_STAGE5C": False,
    }
    _write_json(output / "g1_model_screening_report.json", report)
    return report


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    report = train(args.dataset, args.config, args.output)
    print(json.dumps({
        "G1_model_screening_pass": report["G1_model_screening_pass"],
        "in_domain_macro_f1": report["in_domain"]["macro_f1"],
        "cross_asset_world_macro_f1": report["cross_asset_world"]["macro_f1"],
        "cross_asset_world_leaf_puddle_miou": report["cross_asset_world"]["leaf_puddle_miou"],
        "color_stress_macro_f1": report["color_stress"]["aggregate_macro_f1"],
    }, indent=2))
    if not report["G1_model_screening_pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
