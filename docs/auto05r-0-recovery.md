# AUTO-05R-0 感知恢复合同

## 目的与范围

AUTO-05R-0 只建立感知恢复的第一批合同与评估/运行时基础设施，不训练模型、不导出正式 ONNX、不采集 G4，也不改写任何历史证据。目标是为后续 AUTO-05R 的正式训练、G4 数据与 AUTO-06/07 正式链提供可机器校验的尺度口径、诊断矩阵、模型 manifest 与 backend 准入合同。

## 已建立的合同

### 1. 评测尺度合同

- `metric_scale.py` 定义 `MetricScale.NATIVE_SCALE / MODEL_INPUT_SCALE`、`InstanceScaleRecord` 六字段记录、`derive_model_scale_record` 转换、machine-evaluable 与 small-object 分箱，以及 `assert_scale_fields_present` 缺字段即抛错。
- model-input mask area 必须来自 `cv2.resize(mask, INTER_NEAREST)` 的精确像素统计；提供 mask 时禁止用 native mask area 面积缩放代替。
- `scripts/auto05_screening.py`：`instance_boxes` 返回 `native_bbox_xyxy/native_short_side_px/native_mask_area_px` 与旧键别名，并携带布尔 `mask`；`detector_raw_predictions` 用 `derive_model_scale_record` 生成 model scale 字段并把匹配用 `bbox_xyxy` 明确指向 model 尺度；`detector_metrics` 的 machine-evaluable 与 small-object 判断显式使用 native scale，返回值带 `machine_evaluable_scale="native"`、`small_object_scale="native"`、`scale_contract_version=1`。
- AUTO-05 冻结阈值与门禁数值不变：machine-evaluable 短边 ≥8 px / mask ≥20 px、small-object 短边 <18 px，detector/area 门限与 `auto05_g3_screening.yaml` 保持一致。

### 2. 因子化诊断矩阵

- `auto05r_factorized_diagnostics.yaml` 定义 D1-D6：
  - D1 same world / unseen asset
  - D2 unseen world / seen asset
  - D3 unseen material / seen geometry
  - D4 unseen lighting / seen asset
  - D5 unseen negative assets
  - D6 full unseen world+asset+negative+trajectory
- `factorized_diagnostics.py` 提供配置加载（强制 `legacy_g3_test_used_as_selection=false`）、行级诊断匹配和报告 schema 校验（macro F1、逐类 precision/recall/F1、AP50/AP50-95、negative FP/frame、discovery recall、leaf/puddle IoU）。
- 本任务不产生任何模型指标；报告校验函数只校验结构，不伪造数值。

### 3. 模型 manifest v2

- 保留旧 `model_manifest.yaml` 并新增内容一致的 `model_manifest_legacy_synthetic.yaml` 副本。
- 新增 detector / classifier / leaf_segmenter / puddle_segmenter 四个模型 manifest 与 `perception_pipeline_manifest.yaml`。
- 当前没有正式模型：所有 v2 manifest 的 `artifact` 与 `artifact_sha256` 为 `null`，`screening_pass/formal_pass/live_pass` 为 `false`，`competition_claim_allowed` 为 `false`；validator 将其识别为 `not_available`。
- `pipeline_manifest.py` 提供加载、校验（必填字段、YAML 类型、有限阈值、布尔状态、artifact 存在与 SHA-256 匹配）与 `backend_eligibility`。

### 4. runtime backend fail-closed

- `backends.py` 的 onnxruntime 不再天然等于 synthetic_only；`select_backend` 新增 `manifest_path`、`required_claim`（screening/formal/live/competition）与 `artifact_root`。
- 缺 manifest、manifest 无效、artifact SHA 不匹配或所需状态不足均抛 `BackendUnavailable`；`BackendSelection` 增加 `synthetic_only/screening_pass/formal_pass/live_pass/competition_claim_allowed`。
- `ground_truth` 仍为 evaluation-only，`mock` 仍为 test-only，`horizon_j6` 仍需要 toolchain+runtime。
- 不重写 `perception_node.py`；正式多模型运行链属于 AUTO-07R，tracker 行为也不改动，仅记录合同。

### 5. 仓库现状 inventory

`artifacts/perception_recovery_inventory/` 下 8 个 JSON 记录仓库基线、G3 数据集合同、模型架构、指标合同、运行时合同、manifest 合同、tracker 合同与 12 类根因分析。所有数值来自仓库实际文件与证据，无法得到的精确值保留 `null` 并给出来源；AUTO-05 历史状态保持 `BLOCKED`。

## 历史事实（不变）

- `AUTO-04=PASS`：只证明小样本 Gazebo micro train-set capacity，不能外推跨世界、真实域、J6 或竞赛感知。
- `AUTO-05=BLOCKED`：G3 数据门通过；三次有界 screening 未通过全部冻结门，最佳 Attempt 3 仍有 discovery recall、in-domain/cross-world F1、leaf/puddle IoU、macro mIoU 与 color/material stress 共 7 个门失败。
- `AUTO-06/07/08=BLOCKED`（dependency）：依赖 AUTO-05，未启动。
- 不得把旧 G3 test 当作新模型选参集；G3 继续作为 legacy benchmark。

## G4 与后续

- G4 尚未采集：worlds、frames、split 元数据保持 `null`，不得伪造或外推。
- 正式模型不存在：训练、导出、screening、formal/live/spot-clean 均未执行。
- 任何后续模型只有在其 manifest artifact 存在、SHA-256 匹配且对应 claim 状态通过时才能被 backend 选择。

## 验证

快速 CI 与针对性 pytest 覆盖尺度合同、small-object/machine-evaluable 尺度、D1-D6 报告校验、manifest 校验与 backend fail-closed；全部通过后本任务才能提交。
