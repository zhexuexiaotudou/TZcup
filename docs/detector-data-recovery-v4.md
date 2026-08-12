# Detector Data Recovery V4

Detector Data Recovery V4（DDRV4）用于恢复学习检测器，并以 fail-closed 方式决定是否允许进入运动相机、冻结模型和产品验收链。它不覆盖旧实验结论，也不把静态指标等同于在线产品能力。

## 协议边界

- 只允许新建、隔离的 `G7_DETECTOR_DEVELOPMENT` 数据参与 detector 开发；
- G6 只用于历史回归或对照，不参与 checkpoint、阈值、增强或路线选择；
- 旧 `G5_SEALED_FINAL` 永久拒绝，`G5_V2_SEALED_FINAL` 在有效 freeze 前禁止读取；
- Area 沿用已验证的软件结果，除非存在可复现的集成缺陷，不与 detector 一起重新调参；
- 静态候选必须先通过运动相机在线质量和性能门，才能解锁后续验证。

运行时约束由 `sanitation_learning.ddrv4_boundary` 执行，CI 覆盖数据隔离、G5 拒绝和 G6 禁止选模等规则。

## 数据与候选

G7 数据生成入口：

```powershell
py -3 scripts/build_ddrv4_g7.py --output <EMPTY_EXTERNAL_DIRECTORY>
```

有效数据包包含 3200 帧、320 场、13 个世界和 2810 个实例，覆盖 metal、bottle、paper、小目标以及 16 类全负样本。独立审计确认像素、元数据和实例一致，跨 split pHash duplicate 为 0。

在固定 G7-only 协议下，当前候选为 D1-B，阈值 `0.53`。静态验证结果：

| 指标 | 结果 |
|---|---:|
| Recall | 0.9778 |
| Precision | 0.9778 |
| Macro-F1 | 0.9777 |
| Metal recall | 0.9467 |
| Small-object recall | 0.9467 |
| FP/frame | 0.0167 |

静态门通过只允许进入在线开发验证，不代表产品可用。

## 当前阻塞

运动相机兼容回归覆盖 24 个 mission、2160 帧。当前主要结果为：

| 指标 | 结果 |
|---|---:|
| Eventual recall | 0.3898 |
| Metal recall | 0.1053 |
| Paper recall | 0.7000 |
| Small-object recall | 0.3529 |
| Wrong-actionable rate | 0.3510 |
| Product map precision | 0.2111 |
| Product map coverage | 0.1900 |

该回归缺少完整 G7 moving pack 中的 behind-FOV、转弯、遮挡和反光任务，因此既不能判定在线通过，也不能作为最终产品裁决。性能回放在 300 帧、10 Hz 输入上得到 `9.9974 Hz`、P95 `155.83 ms`、掉帧率 `0`，严格 `>=10 Hz` 门仍失败。

当前结论保持：

- `MODEL_BLOCKED_INTERNAL=true`
- `DDRV4_X86_DEV_PASS=false`
- `PRODUCT_X86_PERCEPTION_READY=false`
- freeze、G5_V2、30-seed、Spot Cleaning、soak、release、J6 和 field 全部锁定

## 证据与复现

紧凑证据位于 [`artifacts/detector_data_recovery_v4_20260811T134117Z/`](../artifacts/detector_data_recovery_v4_20260811T134117Z/)，其中：

- `g7/`：数据规模与独立审计摘要；
- `diagnostics/`：旧模型失败分类；
- `d1/`：静态候选选择和验证；
- `online_dev/`：运动相机兼容回归；
- `performance/`：哈希绑定的性能回放；
- `final/`：最终状态、阻塞项、模型注册和 evidence index。

原始训练数据、逐帧输出和失败尝试不进入 Git。历史过程由 Git 与 PR 保留，本页只维护当前有效协议、结果和解锁条件。
