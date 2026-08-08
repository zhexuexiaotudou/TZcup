# AUTO-05R P0 可信基础（Trustworthiness Foundation）

状态（2026-08-09）：**P0 基础设施已实现；未训练任何新产品模型；未创建
G5 数据集；未通过 AUTO-05R / P4 / P5 / formal / live / J6 / 真实现场门。**

## 已实现（P0-1 .. P0-12）

1. **P0-1 翻转几何修复**：`g4_geometry.py` 提供 native↔model 与水平翻转
   统一工具，`G4DiscoveryDataset` 的 flip 分支从实际帧形状和
   `DISCOVERY_MODEL_SIZE` 动态生成 bbox，硬编码 `384/512` 已删除；往返误差
   ≤0.5 px（`test_g4_data.py`，含角落、非方形、翻转一次/两次、随机 1000
   boxes）。
2. **P0-2 分裂角色策略**：`g4_split_policy.py` 定义 development-readable
   （`train` / `train_world_holdout` / `val` / `D1`-`D5`）；旧 `test` 在报告
   与 CLI 中一律改名为 `legacy_G4_D6_diagnostic` 并强制告警；训练/阈值/
   checkpoint 选择/困难负样本挖掘/screening 判定一律拒绝读取。回归测试证明
   修改 legacy 指标不会改变 screening 决策。
3. **P0-3 密封 G5 合约**：`g4_sealed_final.py` 要求 ≥4 未见 worlds、
   ≥100 scenes、≥1000 frames、目标/困难负样本资产均未见，且必须存在
   `MODEL_FREEZE.json` 才能解锁；freeze 必须携带真实 P4 通过证据、四模型
   artifact/预训练权重哈希、ONNX 合同和冻结 evaluator 哈希；
   `SealedFinalGate` 原子记录首次访问并拒绝重跑。one-shot CLI 不存在
   open-only 或外部预制 metrics 入口，只能在记录访问后运行哈希匹配的冻结
   evaluator。测试只使用临时合成元数据，不创建真实 final 数据集。
4. **P0-4 最佳 checkpoint 训练**：`scripts/auto05r_screening.py` 对
   discovery/classifier/leaf/puddle 均启用每 epoch 验证、EMA、正早停耐心、
   checkpoint 持久化与 `load_best=True`；classifier 验证使用独立的
   train-world holdout 样本集；legacy/G5 数据绝不进入训练函数。
5. **P0-5 约束感知选择**：`g4_selection.py` 的 `ConstraintAwareSelector`
   先满足硬 FP/specificity 约束再最大化任务目标，持久化 selected epoch、
   选择分数、违反约束与验证指标；若当前没有 epoch 满足硬门，只保留按约束
   违反距离排序的 `diagnostic_fallback_only` checkpoint，明确不可获得产品资格。
6. **P0-6 task-specific ONNX parity**：`g4_onnx_parity.py` 提供 discovery
   decoded 候选数/框/分数一致、classifier top-1 与最大概率误差、
   segmenter 二值掩码 IoU/像素一致与 boundary 掩码一致；`assert_onnx_contract`
   强制固定形状、opset 17、operator inventory 与零 custom ops。
7. **P0-7 语义修正**：screening 报告显式区分 in-domain
   （`train_world_holdout`）与 cross-world（`val`）；任何指标不再把 legacy
   集合称为 `test` 或 `final`。
8. **P0-8 真实门槛与固定阈值**：`perception_p4_screening_policy.yaml` 与
   `perception_p5_final_policy.yaml` 为唯一权威 P4/P5 阈值；same-color
   specificity 按 taxonomy 真实计算，D1-D5 逐项真实推理并输出报告，legacy D6 仅
   诊断（不参与判定），G5 单独门控且当前 `not_evaluated`；缺失指标一律
   fail-closed。
9. **P0-9 冻结/清单**：`g4_manifest.py` 从不可变 frozen model config 生成
   artifact manifest（预处理/后处理、阈值、class map、输入输出形状、算子
   清单、artifact SHA-256、config hash、freeze id/timestamp、溯源、验收
   状态），字段或哈希缺失/不匹配即拒绝。
10. **P0-10 预训练溯源**：`g4_pretrained.py` 使用官方 torchvision 权重枚举
    （ResNet18 discovery、MobileNetV3-small classifier、DeepLabV3-ResNet50
    area），记录 source URL、enum、license、架构与缓存文件完整 SHA-256；
    生产候选获取/校验失败即 fail-closed；
    `from_scratch_control` 仅作为标注消融，永不产生 product-ready 状态。
11. **P0-11 G4 数据门紧凑证据**：`artifacts/auto05r_g4_data_gate/` 提交
    schemas、哈希、计数、split/world/asset registries 与既有
    `G4_dataset_gate_pass` 决策；证据同时明确该门只证明采集/标注质量，旧 test
    后来被两次历史 screening 读取并已污染。生成器
    `scripts/auto05r_g4_data_gate_evidence.py` 确定性可复现，原始帧/包/模型
    二进制一律不入库。P2 后续新增 manifest—像素一致性审计并推翻了旧门，
    当前紧凑证据已更新为 `G4_dataset_gate_pass=false`；详见
    `docs/auto05r-p2-data-integrity-recovery.md`。
12. **P0-12 micro 门加强**：discovery 增加 AP50/precision/FP rate；
    classifier 增加 background/hard-negative specificity；area 增加 boundary
    F1 与负样本帧 FP；报告显式标记 `gate_kind=capacity_only`，不得冒充
    screening 或产品 claim。

## 当前真实边界

- 无新产品模型被训练或冻结；`MODEL_FREEZE.json` 不存在。
- legacy G4 `test` 是受污染诊断证据（`legacy_G4_D6_diagnostic`），只可用于
  非门控诊断。
- G5 sealed final 尚未创建，任何 `G5_SEALED_FINAL` 指标均为 `not_evaluated`。
- `AUTO_05R_PASS`、`P4_SCREENING_PASS`、`P5_FINAL_PASS`、formal、live、
  J6 与真实现场 claim 全部保持 false。

## 2026-08-09 验证证据

- Windows fast CI：`379 passed, 12 skipped`；skip 来自该 Python 环境缺少
  torch/onnx，未作为模型验收。
- CUDA 容器（Torch 2.5.1+cu124、torchvision 0.20.1+cu124、ONNX 1.17、
  ORT 1.20.1、RTX 4080 Laptop）：模型/ONNX 定向测试 `24 passed, 0 skipped`；
  全量 fast CI `391 passed, 0 skipped`。
- 官方权重完整 SHA-256、镜像 ID 与命令结果见
  `artifacts/auto05r_p0_evidence/P0_VALIDATION.json`。

## 验证入口

```powershell
py -3 -m pytest starter_ws/src/sanitation_learning/test/test_g4_data.py starter_ws/src/sanitation_learning/test/test_g4_training_protocol.py starter_ws/src/sanitation_learning/test/test_g4_models.py -q
py -3 scripts/ci_fast.py
```
