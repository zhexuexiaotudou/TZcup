from __future__ import annotations

from pathlib import Path

import numpy as np

from .synthetic import CLASS_COLORS_RGB, HEIGHT, WIDTH


def build_color_prototype_model(output_path: str | Path) -> Path:
    try:
        import onnx
        from onnx import TensorProto, helper, numpy_helper
    except ImportError as exc:
        raise RuntimeError("onnx package is required to build the Stage5A model") from exc
    prototypes = CLASS_COLORS_RGB.astype(np.float32) / 255.0
    weights = (2.0 * prototypes).reshape(len(prototypes), 3, 1, 1)
    bias = -np.square(prototypes).sum(axis=1)
    graph = helper.make_graph(
        [helper.make_node("Conv", ["images", "weights", "bias"], ["logits"], kernel_shape=[1, 1])],
        "stage5a_synthetic_color_prototype",
        [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, HEIGHT, WIDTH])],
        [helper.make_tensor_value_info("logits", TensorProto.FLOAT, [1, len(prototypes), HEIGHT, WIDTH])],
        [numpy_helper.from_array(weights, "weights"), numpy_helper.from_array(bias, "bias")],
    )
    model = helper.make_model(
        graph,
        producer_name="tzcup-stage5a",
        opset_imports=[helper.make_opsetid("", 13)],
    )
    model.ir_version = min(model.ir_version, 10)
    onnx.checker.check_model(model)
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, path)
    return path


def infer_labels(session, image_rgb: np.ndarray) -> tuple[np.ndarray, float]:
    import time

    tensor = np.transpose(image_rgb.astype(np.float32) / 255.0, (2, 0, 1))[None, ...]
    start = time.perf_counter()
    logits = session.run(["logits"], {"images": tensor})[0]
    latency_ms = (time.perf_counter() - start) * 1000.0
    return np.argmax(logits[0], axis=0).astype(np.uint8), latency_ms
