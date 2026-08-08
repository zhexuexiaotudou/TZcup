# Task ID

`TZCUP-PERCEPTION-RECOVERY-20260805-01`

<!-- HYBRID_TASK_METADATA_BEGIN
{
  "task_id": "TZCUP-PERCEPTION-RECOVERY-20260805-01",
  "validation_commands": [
    {
      "executable": "git",
      "arguments": ["diff", "--check"],
      "working_directory": ".",
      "timeout_seconds": 180,
      "permission_pattern": "git diff --check"
    },
    {
      "executable": "py",
      "arguments": ["-3", "scripts/ci_fast.py"],
      "working_directory": ".",
      "timeout_seconds": 600,
      "permission_pattern": "py -3 scripts/ci_fast.py"
    },
    {
      "executable": "py",
      "arguments": ["-3", "-m", "pytest", "-q",
        "starter_ws/src/sanitation_learning/test/test_native_to_model_scale_contract.py",
        "starter_ws/src/sanitation_learning/test/test_small_object_bucket_scale.py",
        "starter_ws/src/sanitation_learning/test/test_machine_evaluable_scale.py",
        "starter_ws/src/sanitation_learning/test/test_factorized_split_contract.py",
        "starter_ws/src/sanitation_perception/test/test_pipeline_manifest.py",
        "starter_ws/src/sanitation_perception/test/test_backends.py"],
      "working_directory": ".",
      "timeout_seconds": 300,
      "permission_pattern": "py -3 -m pytest targeted perception recovery tests"
    }
  ]
}
HYBRID_TASK_METADATA_END -->

# Objective

在当前干净工作树 `F:\Project\TZcup-perception-recovery`（分支 `deepseek/perception-recovery`，基线 `d321c23`）完成 `AUTO-05R-0`：评测尺度合同、因子化诊断矩阵、正式模型 manifest v2 与 runtime backend fail-closed 合同。只修合同和评估/运行时基础设施，不训练新模型、不生成正式 ONNX、不采集 G4、不改历史证据。

# Relevant context

仓库远端 `main` 为 `d321c23beed1fbba1c8d66457f19195c18cc18b6`。历史事实必须保留：

- `AUTO-04=PASS`，但只是小样本 micro-overfit，不能外推。
- `AUTO-05=BLOCKED`；G3 数据门通过，三次 screening 失败，最佳 Attempt 3 仍有 discovery recall、in-domain/cross-world F1、leaf/puddle IoU、color/material stress 等门失败。
- `AUTO-06/07/08` 保持 dependency blocked。
- 不得把旧 G3 test 当新模型选参集；不得把 `model_manifest.yaml` 直接覆盖；不得改动 `AUTONOMOUS_STATE.json`、`FINAL_AUTONOMOUS_STATUS.json`、`FINAL_BLOCKER_REGISTER.json`、`artifacts/autonomous_auto04_20260730_evidence/`、`artifacts/autonomous_auto05_20260730_evidence/`。

必须先阅读的关键文件：

```text
README.md
README_FIRST.md
PROJECT_SPEC.md
STAGE_GATES.md
AUTONOMOUS_STATE.json
docs/progress.md
docs/auto04-micro-overfit.md
docs/auto05-g3-screening.md
docs/auto05-attempts.md
scripts/auto05_screening.py
scripts/ci_fast.py
starter_ws/src/sanitation_learning/sanitation_learning/auto04_contract.py
starter_ws/src/sanitation_learning/sanitation_learning/g3_scene.py
starter_ws/src/sanitation_learning/config/auto05_g3_screening.yaml
starter_ws/src/sanitation_perception/config/model_manifest.yaml
starter_ws/src/sanitation_perception/config/preprocess_spec.yaml
starter_ws/src/sanitation_perception/config/postprocess_spec.yaml
starter_ws/src/sanitation_perception/sanitation_perception/backends.py
starter_ws/src/sanitation_perception/sanitation_perception/preprocessing.py
starter_ws/src/sanitation_perception/sanitation_perception/perception_node.py
starter_ws/src/sanitation_perception/sanitation_perception/projection.py
starter_ws/src/sanitation_perception/sanitation_perception/tracking.py
```

# Current architecture

当前 `scripts/auto05_screening.py` 的 `instance_boxes` 只返回原图 `bbox_xyxy/short_side/mask_area`，`detector_raw_predictions` 把 bbox 缩放到模型输入尺度后仍用原图 `short_side/mask_area` 判断 machine-evaluable 和 small object，尺度口径混用。

当前 `model_manifest.yaml` 是单文件 legacy synthetic manifest；`backends.py` 对 `onnxruntime` 硬编码 `synthetic_only=True`，不校验 manifest、SHA 或模型状态。当前 `perception_node.py` 是 128x96 六类 segmentation legacy synthetic node；本任务不重写它，只建立未来正式运行链所需合同。

当前 tracker 使用同类关联、`confidence=min(old,new)`、固定 `0.80` confirmation。本任务只做合同记录和后续 v2 的接口占位，不重写 tracker 行为。

# Requirements

## 1. AUTO-05R-0 inventory

创建目录 `artifacts/perception_recovery_inventory/`，生成 8 个合法 JSON：

```text
repository_baseline.json
current_dataset_contract.json
current_model_architecture.json
current_metric_contract.json
current_runtime_contract.json
current_manifest_contract.json
current_tracker_contract.json
current_failure_analysis.json
```

要求：

- 内容来自仓库实际文件和证据，不写未验证的指标；无法得到精确值就写 `null` 并给 `source`。
- `repository_baseline.json` 记录 `commit=d321c23beed1fbba1c8d66457f19195c18cc18b6`、分支、remote、schema_version、UTC 时间、`historical_auto05_status=BLOCKED`。
- `current_failure_analysis.json` 必须覆盖提示词中的 12 类根因，并保持历史 `AUTO-05=BLOCKED`。
- 每个 JSON 顶层至少包含 `schema_version`、`generated_at_utc`、`source_files`、`facts`/`metrics`/`contract` 等结构化字段。

## 2. Metric scale contract

新增 `starter_ws/src/sanitation_learning/sanitation_learning/metric_scale.py`，提供：

- `MetricScale` 枚举：`NATIVE_SCALE`、`MODEL_INPUT_SCALE`。
- `InstanceScaleRecord` dataclass，包含：
  `native_bbox_xyxy`、`native_short_side_px`、`native_mask_area_px`、`model_bbox_xyxy`、`model_short_side_px`、`model_mask_area_px`。
- `derive_model_scale_record(record, source_size, target_size, model_mask=None)`：由 native 记录生成 model-input 记录；若提供 model_mask，则 `model_mask_area_px` 必须从 `cv2.resize(mask, (target_width, target_height), interpolation=cv2.INTER_NEAREST)` 统计，不得用 native mask area 面积缩放代替。
- `machine_evaluable_bucket(record, scale=MetricScale.NATIVE_SCALE, min_short_side_px=8.0, min_mask_area_px=20.0)`。
- `small_object_bucket(record, scale=MetricScale.NATIVE_SCALE, max_short_side_px=18.0)`。
- `assert_scale_fields_present(record)`：缺失任一 scale 字段时抛 `ValueError`。

重构 `scripts/auto05_screening.py`：

- `instance_boxes` 返回 native 字段 `native_bbox_xyxy/native_short_side_px/native_mask_area_px`，并保留旧键 `bbox_xyxy/short_side/mask_area` 作为 native 别名。
- `instance_boxes` 的每条记录包含 `mask`（布尔 NumPy 数组）供 model scale mask area 精确重算；不得在后续 metrics 中依赖旧键。
- `detector_raw_predictions` 使用 `derive_model_scale_record` 生成 model scale 字段，并把匹配用的 `bbox_xyxy` 明确指向 model 尺度。
- `detector_metrics` 的 machine-evaluable 与 small-object 判断必须显式使用 `MetricScale.NATIVE_SCALE` 字段，并在返回值中加入：
  `machine_evaluable_scale="native"`、`small_object_scale="native"`、`scale_contract_version=1`。
- 不得改变 AUTO-05 冻结阈值和门禁数值。

## 3. Factorized diagnostics contract

新增：

```text
starter_ws/src/sanitation_learning/config/auto05r_factorized_diagnostics.yaml
starter_ws/src/sanitation_learning/sanitation_learning/factorized_diagnostics.py
```

配置必须定义 `D1` 到 `D6`：

```text
D1: same world / unseen asset
D2: unseen world / seen asset
D3: unseen material / seen geometry
D4: unseen lighting / seen asset
D5: unseen negative assets
D6: full unseen world+asset+negative+trajectory
```

`factorized_diagnostics.py` 至少提供：

- `load_diagnostic_config(path)`：校验 D1-D6、必填字段和 `legacy_g3_test_used_as_selection=false`。
- `diagnosis_ids_for_row(row, config)`：根据 row 的 `world_id`、`asset_family`、`material_id`、`lighting_family`、`negative_only`、`trajectory_id` 等元数据返回适用 diagnosis id。
- `validate_factorized_metrics_report(report, config)`：校验每个 diagnosis 包含 `macro_f1`、`per_class_precision`、`per_class_recall`、`per_class_f1`、`ap50`、`ap50_95`、`negative_fp_per_frame`、`discovery_recall`、`leaf_iou`、`puddle_iou`。

不得伪造实际模型指标；实现合同和校验函数即可。

## 4. Manifest v2 and backend fail-closed

保留旧文件，新增 legacy copy：

```text
starter_ws/src/sanitation_perception/config/model_manifest.yaml
starter_ws/src/sanitation_perception/config/model_manifest_legacy_synthetic.yaml
```

新增：

```text
starter_ws/src/sanitation_perception/config/detector_manifest.yaml
starter_ws/src/sanitation_perception/config/classifier_manifest.yaml
starter_ws/src/sanitation_perception/config/leaf_segmenter_manifest.yaml
starter_ws/src/sanitation_perception/config/puddle_segmenter_manifest.yaml
starter_ws/src/sanitation_perception/config/perception_pipeline_manifest.yaml
starter_ws/src/sanitation_perception/sanitation_perception/pipeline_manifest.py
```

每个 model manifest 至少包含：

```text
schema_version
model_id
artifact
artifact_sha256
framework
opset
license
weight_source
pretraining_source
input names/shapes/dtypes
normalization
output names/shapes
class_order
thresholds
NMS
provider_compatibility
screening_pass
formal_pass
live_pass
synthetic_only
competition_claim_allowed
```

当前没有正式模型，所有新 model manifest 必须把 `artifact` 和 `artifact_sha256` 置为 `null`，状态字段为 `false`，并允许被 validator 识别为 `not_available`；不得声称模型已训练或已过门。`model_manifest_legacy_synthetic.yaml` 必须与旧 `model_manifest.yaml` 内容和语义一致。

`pipeline_manifest.py` 提供：

- `load_model_manifest(path)` / `load_pipeline_manifest(path)`。
- `validate_model_manifest(manifest, artifact_root=None)`：必填字段、YAML 类型、阈值有限、状态类型、`artifact` 非空时校验文件存在且 SHA-256 匹配。
- `backend_eligibility(manifest)`：返回 screening/formal/live/competition 状态布尔值。

重构 `starter_ws/src/sanitation_perception/sanitation_perception/backends.py`：

- `onnxruntime` 不再天然等于 `synthetic_only`。
- `select_backend(...)` 支持 `manifest_path` 和 `required_claim`（例如 `"screening"`/`"formal"`/`"live"`）。
- 缺 manifest、manifest 无效、artifact SHA 不匹配、或所需状态不足时抛 `BackendUnavailable`。
- `BackendSelection` 增加 `synthetic_only`、`screening_pass`、`formal_pass`、`live_pass`、`competition_claim_allowed`。
- 保持 `ground_truth` 为 evaluation-only、`mock` 为 test-only、`horizon_j6` 需要 toolchain+runtime。

## 5. Tests and CI

新增测试：

```text
starter_ws/src/sanitation_learning/test/test_native_to_model_scale_contract.py
starter_ws/src/sanitation_learning/test/test_small_object_bucket_scale.py
starter_ws/src/sanitation_learning/test/test_machine_evaluable_scale.py
starter_ws/src/sanitation_learning/test/test_factorized_split_contract.py
starter_ws/src/sanitation_perception/test/test_pipeline_manifest.py
```

更新 `starter_ws/src/sanitation_perception/test/test_backends.py` 以覆盖 manifest 缺失、SHA 不匹配、状态不足、状态足够成功四种情况。

把上述新测试路径加入 `scripts/ci_fast.py` 的 `test_paths`，确保快速 CI 真正运行它们。

## 6. Docs

- 新增 `docs/auto05r-0-recovery.md`，说明 AUTO-05R-0 已建立哪些合同、旧 G3 继续作为 legacy benchmark、新 G4 尚未采集、AUTO-05 历史仍为 BLOCKED。
- 在 `docs/progress.md` 顶部添加一个简洁的 `## 2026-08-05：AUTO-05R-0 感知恢复合同` 小节，保留历史事实，不降低门槛。
- 同步根目录中文 `README.md` 的“当前状态”表：`学习感知` 行可改为 `恢复推进中`，但必须同时写明 `AUTO-05=BLOCKED` 历史状态和 AUTO-06/07/08 未通过。README 不得新增 `## AUTO-` 或 `## Stage` 进度标题，保持不超过 180 行。

# Explicit non-goals

- 不训练、不微调、不导出 detector/classifier/area ONNX。
- 不采集或伪造 G4 数据。
- 不改 `AUTONOMOUS_STATE.json`、`FINAL_AUTONOMOUS_STATUS.json`、`FINAL_BLOCKER_REGISTER.json` 或历史 evidence 目录。
- 不改 `AUTO-05` 冻结门槛，不把 paper 合并为 background，不把旧 G3 test 当新模型选择集。
- 不重写 `perception_node.py` 为正式多模型运行链；该工作属于 AUTO-07R。
- 不实现 tracker v2 行为；只记录当前 tracker 合同到 inventory。
- 不改 hybrid workflow、`.agent` 之外的 workflow 配置。
- 不提交 `.agent/` 或任何原始数据/checkpoint/rosbag。

# Existing patterns to follow

- 保持 `scripts/auto05_screening.py` 现有 CLI 和输出结构兼容；新增字段不能破坏 `finalize_auto05.py` 读取。
- 保持现有 `backends.py` 的 `BackendUnavailable` 语义和现有参数默认值兼容性，只扩展新参数。
- YAML/JSON 使用仓库现有风格；Python 使用 `from __future__ import annotations`。
- 新测试放入现有包 `test/` 目录，并加入 `ci_fast.py`。

# Validation commands

见 metadata；执行：

```powershell
git diff --check
py -3 scripts/ci_fast.py
py -3 -m pytest -q `
  starter_ws/src/sanitation_learning/test/test_native_to_model_scale_contract.py `
  starter_ws/src/sanitation_learning/test/test_small_object_bucket_scale.py `
  starter_ws/src/sanitation_learning/test/test_machine_evaluable_scale.py `
  starter_ws/src/sanitation_learning/test/test_factorized_split_contract.py `
  starter_ws/src/sanitation_perception/test/test_pipeline_manifest.py `
  starter_ws/src/sanitation_perception/test/test_backends.py
```

# Acceptance criteria

- `artifacts/perception_recovery_inventory/` 下 8 个 JSON 均存在、可解析、包含要求字段，且不把 AUTO-05 写成 PASS。
- `metric_scale.py` 与 `auto05_screening.py` 中 machine-evaluable/small-object 判断显式声明 `native` scale，新增测试通过。
- `auto05r_factorized_diagnostics.yaml` 包含 D1-D6，`factorized_diagnostics.py` 可校验报告 schema，新增测试通过。
- manifest v2 文件齐全；`pipeline_manifest.py` 对缺失 manifest、SHA 不匹配、状态不足 fail-closed；`test_pipeline_manifest.py` 和更新后的 `test_backends.py` 通过。
- `git diff --check` 通过。
- `py -3 scripts/ci_fast.py` 通过，且快速 CI 包含新增测试。
- README、docs/progress.md、docs/auto05r-0-recovery.md 已同步；README 仍符合仓库 CI 卫生规则。

# Required completion report

报告必须列出：

- changed files 与新增文件；
- 每个 inventory JSON 的来源和关键结论；
- metric scale 重构的具体函数与测试结果；
- manifest v2 与 backend fail-closed 的设计；
- 验证命令输出摘要；
- 尚未执行项：G4 采集、模型训练、screening、formal/live/spot-clean；
- 残余风险：正式模型不存在，`competition_claim_allowed` 必须保持 false。

# Stop conditions

只有任务超出用户授权、无法安全完成、需要另一个 AI、或必须修改 hybrid workflow 本身时才停止。
