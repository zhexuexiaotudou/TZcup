#!/usr/bin/env python3
"""Container worker for inspecting, exporting, and auditing pinned D1 weights."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np


D1_PT_SHA256 = "1cf60873661811f51cd84fb6aafb403646b67d2add57c4851b0be48ebdff2873"
D1_ONNX_SHA256 = "01c72cdbcd08b6fd91c9a56a065f19837bffd67cca175a75b39e295c3afc01f5"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_pinned_checkpoint(path: Path) -> None:
    actual = sha256(path)
    if actual != D1_PT_SHA256:
        raise RuntimeError(f"D1 checkpoint SHA mismatch: {actual}")


def load_checkpoint(path: Path, source: Path):
    sys.path.insert(0, str(source))
    import torch

    require_pinned_checkpoint(path)
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict):
        raise RuntimeError(f"unexpected checkpoint type: {type(checkpoint).__name__}")
    model = checkpoint.get("ema") or checkpoint.get("model")
    if model is None:
        raise RuntimeError("checkpoint has neither ema nor model")
    return torch, checkpoint, model


def normalize_names(names: Any) -> list[str]:
    if isinstance(names, dict):
        keys = sorted(names, key=lambda value: int(value))
        if [int(key) for key in keys] != list(range(len(keys))):
            raise RuntimeError("class-name dictionary indices are not contiguous")
        return [str(names[key]) for key in keys]
    if isinstance(names, (list, tuple)):
        return [str(value) for value in names]
    raise RuntimeError(f"unsupported class names type: {type(names).__name__}")


def inspect_checkpoint(checkpoint_path: Path, source: Path) -> dict[str, Any]:
    torch, checkpoint, model = load_checkpoint(checkpoint_path, source)
    names = normalize_names(getattr(model, "names", None))
    yaml_config = getattr(model, "yaml", {})
    matching_configs: list[str] = []
    if isinstance(yaml_config, dict):
        import yaml

        checkpoint_structure = dict(yaml_config)
        checkpoint_structure.pop("nc", None)
        checkpoint_structure.pop("yaml_file", None)
        for candidate in sorted((source / "models" / "detect").glob("*.yaml")):
            payload = yaml.safe_load(candidate.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                payload.pop("nc", None)
                payload.pop("yaml_file", None)
                if payload == checkpoint_structure:
                    matching_configs.append(str(candidate.relative_to(source)))
    detection_head = (
        yaml_config.get("head", [])[-1][2]
        if isinstance(yaml_config, dict) and yaml_config.get("head")
        else None
    )
    return {
        "schema_version": 1,
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": sha256(checkpoint_path),
        "checkpoint_keys": sorted(str(key) for key in checkpoint),
        "checkpoint_epoch": checkpoint.get("epoch"),
        "checkpoint_date": checkpoint.get("date"),
        "checkpoint_git": checkpoint.get("git"),
        "model_python_class": f"{type(model).__module__}.{type(model).__name__}",
        "model_yaml": yaml_config,
        "architecture_from_model": yaml_config.get("yaml_file")
        if isinstance(yaml_config, dict)
        else None,
        "matching_official_architecture_configs": matching_configs,
        "detection_head_from_model_yaml": detection_head,
        "class_count": len(names),
        "class_names": names,
        "stride": [int(value) for value in getattr(model, "stride", [])],
        "parameter_count": sum(parameter.numel() for parameter in model.parameters()),
        "parameter_dtype": str(next(model.parameters()).dtype),
        "torch_version": torch.__version__,
        "python_version": sys.version,
    }


def prepare_model(model: Any) -> Any:
    import torch

    model = model.float().eval()
    for parameter in model.parameters():
        parameter.requires_grad = False
    for module in model.modules():
        if hasattr(module, "inplace"):
            module.inplace = False
        if hasattr(module, "dynamic"):
            module.dynamic = False
        if hasattr(module, "export"):
            module.export = True
    return model.to(torch.device("cpu"))


def flatten_outputs(value: Any) -> tuple[Any, ...]:
    import torch

    if isinstance(value, torch.Tensor):
        return (value,)
    if isinstance(value, (list, tuple)):
        flattened: list[Any] = []
        for item in value:
            flattened.extend(flatten_outputs(item))
        return tuple(flattened)
    raise RuntimeError(f"unsupported model output type: {type(value).__name__}")


class ExportWrapper:
    """Keep only tensor outputs while preserving the official model graph."""

    def __new__(cls, model: Any):
        import torch

        class Wrapper(torch.nn.Module):
            def __init__(self, wrapped: Any) -> None:
                super().__init__()
                self.wrapped = wrapped

            def forward(self, images):
                outputs = flatten_outputs(self.wrapped(images))
                return outputs[0] if len(outputs) == 1 else outputs

        return Wrapper(model)


def export_direct(
    checkpoint_path: Path,
    source: Path,
    output: Path,
    *,
    reconstruct: bool,
) -> dict[str, Any]:
    torch, _checkpoint, loaded = load_checkpoint(checkpoint_path, source)
    model = loaded
    if reconstruct:
        from models.yolo import Model

        yaml_config = getattr(loaded, "yaml", None)
        names = normalize_names(getattr(loaded, "names", None))
        if not isinstance(yaml_config, dict):
            raise RuntimeError("checkpoint model has no reconstructable yaml dictionary")
        model = Model(yaml_config, ch=3, nc=len(names))
        state = loaded.float().state_dict()
        result = model.load_state_dict(state, strict=True)
        if result.missing_keys or result.unexpected_keys:
            raise RuntimeError(
                f"state reconstruction mismatch missing={result.missing_keys} "
                f"unexpected={result.unexpected_keys}"
            )
        model.names = names
    model = prepare_model(model)
    wrapped = ExportWrapper(model).eval()
    sample = torch.zeros((1, 3, 640, 640), dtype=torch.float32)
    with torch.inference_mode():
        outputs = flatten_outputs(wrapped(sample))
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        wrapped,
        sample,
        str(output),
        export_params=True,
        opset_version=17,
        do_constant_folding=True,
        input_names=["images"],
        output_names=[f"output{index}" for index in range(len(outputs))],
        dynamic_axes=None,
    )
    import onnx

    graph = onnx.load(str(output))
    for key, value in {
        "names": normalize_names(getattr(model, "names", None)),
        "stride": [int(item) for item in getattr(model, "stride", [])],
        "source_checkpoint_sha256": D1_PT_SHA256,
        "export_route": "E3_reconstruct" if reconstruct else "E2_direct",
    }.items():
        metadata = graph.metadata_props.add()
        metadata.key = key
        metadata.value = repr(value) if not isinstance(value, str) else value
    onnx.save(graph, str(output))
    return {
        "route": "E3" if reconstruct else "E2",
        "output": str(output),
        "output_shapes": [list(value.shape) for value in outputs],
    }


def tensor_shape(value_info: Any) -> list[int | str]:
    result: list[int | str] = []
    for dim in value_info.type.tensor_type.shape.dim:
        if dim.dim_value > 0:
            result.append(int(dim.dim_value))
        elif dim.dim_param:
            result.append(str(dim.dim_param))
        else:
            result.append("dynamic")
    return result


def parse_metadata(value: str) -> Any:
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        return value


def audit_onnx(path: Path) -> dict[str, Any]:
    import onnx

    model = onnx.load(str(path), load_external_data=False)
    onnx.checker.check_model(model)
    inferred = onnx.shape_inference.infer_shapes(model)
    initializers = {item.name for item in inferred.graph.initializer}
    inputs = [
        {"name": item.name, "shape": tensor_shape(item)}
        for item in inferred.graph.input
        if item.name not in initializers
    ]
    outputs = [
        {"name": item.name, "shape": tensor_shape(item)}
        for item in inferred.graph.output
    ]
    operators = Counter(
        f"{node.domain or 'ai.onnx'}::{node.op_type}"
        for node in inferred.graph.node
    )
    custom = sorted(
        name
        for name in operators
        if name.split("::", 1)[0] not in {"ai.onnx", "ai.onnx.ml"}
    )
    initializer_types = Counter(
        onnx.TensorProto.DataType.Name(item.data_type)
        for item in inferred.graph.initializer
    )
    metadata = {item.key: parse_metadata(item.value) for item in inferred.metadata_props}
    names = normalize_names(metadata.get("names")) if "names" in metadata else []
    static_input = inputs == [{"name": "images", "shape": [1, 3, 640, 640]}]
    opsets = {item.domain or "ai.onnx": int(item.version) for item in inferred.opset_import}
    embedded_nms = any(name.endswith("::NonMaxSuppression") for name in operators)
    fp16 = initializer_types.get("FLOAT16", 0) > 0
    dual_detect_outputs = outputs == [
        {"name": "output0", "shape": [1, 14, 8400]},
        {"name": "1774", "shape": [1, 14, 8400]},
    ]
    contract_pass = (
        static_input
        and opsets.get("ai.onnx") == 17
        and not embedded_nms
        and not custom
        and not fp16
        and len(names) == 10
        and dual_detect_outputs
    )
    return {
        "schema_version": 1,
        "model": str(path),
        "sha256": sha256(path),
        "size_bytes": path.stat().st_size,
        "onnx_checker_pass": True,
        "shape_inference_pass": True,
        "inputs": inputs,
        "outputs": outputs,
        "opsets": opsets,
        "ir_version": int(inferred.ir_version),
        "operator_inventory": dict(sorted(operators.items())),
        "custom_operators": custom,
        "embedded_nms": embedded_nms,
        "initializer_data_types": dict(sorted(initializer_types.items())),
        "fp16_initializer_present": fp16,
        "metadata": metadata,
        "class_names": names,
        "dual_detect_outputs_pass": dual_detect_outputs,
        "canonical_contract_pass": contract_pass,
    }


def parity_check(
    checkpoint_path: Path,
    source: Path,
    onnx_path: Path,
    selection_path: Path,
    development_root: Path,
) -> dict[str, Any]:
    sys.path.insert(0, str(source))
    import cv2
    import onnxruntime as ort
    import torch
    from models.experimental import attempt_load
    from utils.augmentations import letterbox

    require_pinned_checkpoint(checkpoint_path)
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    images = selection.get("images", [])
    if len(images) < 100:
        raise RuntimeError(f"parity requires at least 100 development images, got {len(images)}")
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = attempt_load(str(checkpoint_path), device=device, inplace=False, fuse=True)
    model = prepare_model(model).to(device)
    for module in model.modules():
        if hasattr(module, "export"):
            module.export = True
    providers = [
        provider
        for provider in ("CUDAExecutionProvider", "CPUExecutionProvider")
        if provider in ort.get_available_providers()
    ]
    session = ort.InferenceSession(str(onnx_path), providers=providers)
    input_name = session.get_inputs()[0].name
    output_names = [item.name for item in session.get_outputs()]
    output_metrics: list[list[dict[str, float]]] = [
        [] for _ in output_names
    ]
    image_records: list[dict[str, Any]] = []
    for item in images[:100]:
        relative = Path(item["relative_path"])
        image_path = (development_root / relative).resolve()
        if not image_path.is_relative_to(development_root.resolve()):
            raise RuntimeError(f"development image escapes root: {relative}")
        if sha256(image_path) != item["sha256"]:
            raise RuntimeError(f"development image SHA mismatch: {relative}")
        original = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if original is None:
            raise RuntimeError(f"cannot read development image: {relative}")
        prepared = letterbox(original, (640, 640), stride=32, auto=False)[0]
        prepared = prepared[:, :, ::-1].transpose(2, 0, 1)
        prepared = np.ascontiguousarray(prepared, dtype=np.float32) / 255.0
        batch = prepared[None]
        with torch.inference_mode():
            pt_outputs = flatten_outputs(model(torch.from_numpy(batch).to(device)))
        ort_outputs = session.run(output_names, {input_name: batch})
        if len(pt_outputs) != len(ort_outputs):
            raise RuntimeError(
                f"output count mismatch PT={len(pt_outputs)} ONNX={len(ort_outputs)}"
            )
        per_image: list[dict[str, Any]] = []
        for index, (pt_value, ort_value) in enumerate(zip(pt_outputs, ort_outputs)):
            pt_array = pt_value.detach().float().cpu().numpy()
            if pt_array.shape != ort_value.shape:
                raise RuntimeError(
                    f"output {index} shape mismatch PT={pt_array.shape} ONNX={ort_value.shape}"
                )
            difference = np.abs(pt_array - ort_value)
            box_difference = difference[:, :4, :]
            class_difference = difference[:, 4:, :]
            pt_flat = pt_array.reshape(-1).astype(np.float64)
            ort_flat = ort_value.reshape(-1).astype(np.float64)
            denominator = float(np.linalg.norm(pt_flat) * np.linalg.norm(ort_flat))
            cosine = float(np.dot(pt_flat, ort_flat) / denominator) if denominator else 1.0
            metrics = {
                "max_abs_error": float(difference.max(initial=0.0)),
                "mean_abs_error": float(difference.mean()),
                "max_box_abs_error_px": float(box_difference.max(initial=0.0)),
                "max_class_score_abs_error": float(
                    class_difference.max(initial=0.0)
                ),
                "cosine_similarity": cosine,
            }
            output_metrics[index].append(metrics)
            per_image.append({"output": output_names[index], **metrics})
        image_records.append(
            {
                "image_id": item["image_id"],
                "relative_path": item["relative_path"],
                "sha256": item["sha256"],
                "outputs": per_image,
            }
        )
    aggregate: list[dict[str, Any]] = []
    for name, values in zip(output_names, output_metrics):
        aggregate.append(
            {
                "output": name,
                "image_count": len(values),
                "max_abs_error": max(value["max_abs_error"] for value in values),
                "mean_abs_error": float(
                    np.mean([value["mean_abs_error"] for value in values])
                ),
                "minimum_cosine_similarity": min(
                    value["cosine_similarity"] for value in values
                ),
                "max_box_abs_error_px": max(
                    value["max_box_abs_error_px"] for value in values
                ),
                "max_class_score_abs_error": max(
                    value["max_class_score_abs_error"] for value in values
                ),
            }
        )
    parity_pass = all(
        value["max_box_abs_error_px"] <= 1.0
        and value["max_class_score_abs_error"] <= 1e-4
        and value["minimum_cosine_similarity"] >= 0.99999
        for value in aggregate
    )
    return {
        "schema_version": 1,
        "checkpoint_sha256": D1_PT_SHA256,
        "onnx_sha256": sha256(onnx_path),
        "selection_manifest_sha256": sha256(selection_path),
        "image_count": len(image_records),
        "source_split": "train",
        "providers": session.get_providers(),
        "device": str(device),
        "outputs": aggregate,
        "thresholds": {
            "max_box_abs_error_px_at_most": 1.0,
            "max_class_score_abs_error_at_most": 1e-4,
            "minimum_cosine_similarity_at_least": 0.99999,
        },
        "parity_pass": parity_pass,
        "images": image_records,
    }


def infer_development(
    onnx_path: Path,
    selection_path: Path,
    development_root: Path,
) -> dict[str, Any]:
    """Run the canonical D1 primary head on a hashed TRAIN-only manifest."""
    import cv2
    import onnxruntime as ort
    import torch
    from utils.augmentations import letterbox
    from utils.general import non_max_suppression, scale_boxes

    actual_onnx_sha = sha256(onnx_path)
    if actual_onnx_sha != D1_ONNX_SHA256:
        raise RuntimeError(f"D1 canonical ONNX SHA mismatch: {actual_onnx_sha}")
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    images = selection.get("images", [])
    if not images:
        raise RuntimeError("development selection contains no images")
    providers = [
        provider
        for provider in ("CUDAExecutionProvider", "CPUExecutionProvider")
        if provider in ort.get_available_providers()
    ]
    session = ort.InferenceSession(str(onnx_path), providers=providers)
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or list(inputs[0].shape) != [1, 3, 640, 640]:
        raise RuntimeError(f"unexpected canonical input contract: {inputs}")
    if [item.name for item in outputs] != ["output0", "1774"]:
        raise RuntimeError(
            f"unexpected DualDDetect output names: {[item.name for item in outputs]}"
        )
    if [list(item.shape) for item in outputs] != [[1, 14, 8400], [1, 14, 8400]]:
        raise RuntimeError(
            f"unexpected DualDDetect output shapes: {[item.shape for item in outputs]}"
        )
    input_name = inputs[0].name
    source_to_target = {1: 1, 2: 2, 7: 3}
    records: list[dict[str, Any]] = []
    global_class_max = np.zeros(10, dtype=np.float32)
    top1_class_counts = Counter()
    for item in images:
        relative = Path(item["relative_path"])
        image_path = (development_root / relative).resolve()
        if not image_path.is_relative_to(development_root.resolve()):
            raise RuntimeError(f"development image escapes root: {relative}")
        if sha256(image_path) != item["sha256"]:
            raise RuntimeError(f"development image SHA mismatch: {relative}")
        original = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if original is None:
            raise RuntimeError(f"cannot read development image: {relative}")
        prepared = letterbox(original, (640, 640), stride=32, auto=False)[0]
        prepared = prepared[:, :, ::-1].transpose(2, 0, 1)
        batch = np.ascontiguousarray(prepared, dtype=np.float32)[None] / 255.0
        ort_outputs = session.run(["output0", "1774"], {input_name: batch})
        class_scores = ort_outputs[1][0, 4:, :]
        global_class_max = np.maximum(global_class_max, class_scores.max(axis=1))
        top1_class_counts.update(
            int(value) for value in class_scores.argmax(axis=0).tolist()
        )
        primary = torch.from_numpy(ort_outputs[1])
        detections = non_max_suppression(
            primary,
            conf_thres=0.001,
            iou_thres=0.45,
            classes=None,
            agnostic=False,
            max_det=300,
        )[0]
        if len(detections):
            detections[:, :4] = scale_boxes(
                prepared.shape[:2], detections[:, :4], original.shape
            )
        predictions = []
        for row in detections.detach().cpu().numpy():
            source_class = int(row[5])
            predictions.append(
                {
                    "bbox_xyxy": [float(value) for value in row[:4]],
                    "confidence": float(row[4]),
                    "source_class_index": source_class,
                    "target_category_id": source_to_target.get(source_class),
                }
            )
        records.append(
            {
                "image_id": int(item["image_id"]),
                "relative_path": item["relative_path"],
                "sha256": item["sha256"],
                "predictions": predictions,
            }
        )
    return {
        "schema_version": 1,
        "development_only": True,
        "source_split": "train",
        "onnx_sha256": actual_onnx_sha,
        "selection_manifest_sha256": sha256(selection_path),
        "providers": session.get_providers(),
        "preprocessing": {
            "letterbox": [640, 640],
            "auto": False,
            "stride": 32,
            "color": "BGR_to_RGB",
            "normalization": "uint8_div_255_float32",
        },
        "output_contract": {
            "detection_head": "DualDDetect",
            "primary_output_index": 1,
            "primary_output_name": "1774",
            "auxiliary_output_index": 0,
            "auxiliary_output_name": "output0",
            "official_runtime_reference": "detect_dual.py: pred = pred[0][1]",
        },
        "nms": {
            "minimum_confidence": 0.001,
            "iou_threshold": 0.45,
            "embedded_in_onnx": False,
            "source_class_filter": None,
            "max_detections": 300,
        },
        "raw_score_diagnostics": {
            "maximum_score_by_source_class": {
                str(index): float(value)
                for index, value in enumerate(global_class_max.tolist())
            },
            "top1_candidate_count_by_source_class": {
                str(index): int(top1_class_counts.get(index, 0))
                for index in range(10)
            },
        },
        "source_to_target_category": {
            "1": {"source_name": "plastic_bottle", "target_id": 1},
            "2": {"source_name": "drinks_can", "target_id": 2},
            "7": {"source_name": "paper_waste", "target_id": 3},
        },
        "image_count": len(records),
        "images": records,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    inspect = sub.add_parser("inspect")
    inspect.add_argument("--checkpoint", type=Path, required=True)
    inspect.add_argument("--source", type=Path, required=True)
    inspect.add_argument("--output", type=Path, required=True)
    export = sub.add_parser("export")
    export.add_argument("--checkpoint", type=Path, required=True)
    export.add_argument("--source", type=Path, required=True)
    export.add_argument("--output", type=Path, required=True)
    export.add_argument("--reconstruct", action="store_true")
    audit = sub.add_parser("audit")
    audit.add_argument("--model", type=Path, required=True)
    audit.add_argument("--output", type=Path, required=True)
    parity = sub.add_parser("parity")
    parity.add_argument("--checkpoint", type=Path, required=True)
    parity.add_argument("--source", type=Path, required=True)
    parity.add_argument("--model", type=Path, required=True)
    parity.add_argument("--selection", type=Path, required=True)
    parity.add_argument("--development-root", type=Path, required=True)
    parity.add_argument("--output", type=Path, required=True)
    infer = sub.add_parser("infer-development")
    infer.add_argument("--source", type=Path, required=True)
    infer.add_argument("--model", type=Path, required=True)
    infer.add_argument("--selection", type=Path, required=True)
    infer.add_argument("--development-root", type=Path, required=True)
    infer.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "inspect":
        payload = inspect_checkpoint(args.checkpoint, args.source)
    elif args.command == "export":
        payload = export_direct(
            args.checkpoint, args.source, args.output, reconstruct=args.reconstruct
        )
    elif args.command == "audit":
        payload = audit_onnx(args.model)
    elif args.command == "parity":
        payload = parity_check(
            args.checkpoint,
            args.source,
            args.model,
            args.selection,
            args.development_root,
        )
    else:
        sys.path.insert(0, str(args.source))
        payload = infer_development(
            args.model,
            args.selection,
            args.development_root,
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(json.dumps(payload, indent=2, sort_keys=True))
    passed = payload.get("canonical_contract_pass", payload.get("parity_pass", True))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
