from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil


def run_preflight(model_path: str | Path, calibration_manifest: str | Path | None, output: str | Path) -> dict:
    import onnx
    model_path = Path(model_path)
    model = onnx.load(model_path); onnx.checker.check_model(model)
    input_shape = [dimension.dim_value for dimension in model.graph.input[0].type.tensor_type.shape.dim]
    operators = {}
    for node in model.graph.node:
        key = f"{node.domain or 'ai.onnx'}::{node.op_type}"
        operators[key] = operators.get(key, 0) + 1
    tools = {name: shutil.which(name) for name in ("hb_mapper", "hbdk", "horizon_tc_ui", "hb_model_modifier")}
    toolchain_available = any(tools.values())
    calibration = None
    if calibration_manifest and Path(calibration_manifest).is_file():
        calibration = json.loads(Path(calibration_manifest).read_text(encoding="utf-8"))
    fixed_shape = all(isinstance(value, int) and value > 0 for value in input_shape)
    report = {"schema_version": 1, "stage": "Stage5B", "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(), "onnx_checker_pass": True, "fixed_input_shape": input_shape, "fixed_shape_pass": fixed_shape, "opset": max(item.version for item in model.opset_import), "operator_inventory": operators, "custom_operator_count": sum(count for name, count in operators.items() if not name.startswith("ai.onnx::")), "calibration_manifest_present": calibration is not None, "calibration_frame_count": calibration.get("frame_count") if calibration else None, "tool_discovery": tools, "j6_toolchain_available": toolchain_available, "j6_model_precheck_pass": False, "conversion_dry_run_executed": False, "quantization_dry_run_executed": False, "j6_runtime_pass": False, "j6_board_fps": None, "failure_reason": None if toolchain_available else "official Horizon J6 toolchain not found; fail-closed"}
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True); output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
