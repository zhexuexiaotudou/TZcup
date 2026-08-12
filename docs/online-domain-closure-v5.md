# ONLINE-DOMAIN-CLOSURE-V5

ONLINE-DOMAIN-CLOSURE-V5（ODCV5）是 DDRV4 D1-B 静态通过后的在线域闭合协议。它不改写 A1/A2/A3、X1/X2/X3、MRV2、OPR-A/B/C、旧 G5 或 DDRV4 的历史结论，也不重新打开 D2/D3。G6 Area 保持冻结，G5_V2 在 x86 freeze 前保持拒绝访问。

## 当前起点

- DDRV4 D1-B：官方 MMDetection 3.3.0 RTMDet-s，checkpoint SHA-256 `481374d4839e72f05fff0d6d2f6135bc7d715d5c2faf84e75d7d97ca3fc6a361`，action threshold `0.53`。
- G7 静态 VAL recall / precision / macro-F1 均约 `0.978`，具备继续在线闭合的资格。
- 既有 24 mission / 2160 frame 在线回归中，59 个 actionable 离散目标的 eventual recall 为 `0.3898`，correct-class recall 为 `0.1111`，product-map precision / coverage 为 `0.2111 / 0.19`。
- 既有 moving 数据缺 behind-FOV、turn、occlusion 和 reflection，不能冒充完整 G7-MOVING 开发门。
- 300 frame CUDA 产品回放 P95 为 `155.83 ms`、drop 为 `0`，但 10 Hz pacing 测得 `9.9974 Hz`；后续必须用 unpaced capacity 与至少 15 Hz 实时源重建有余量的性能门。

## 顺序与边界

执行顺序为：损失阶梯、golden-frame runtime parity、完整 G7-MOVING development pack、native detector moving gate、必要时同架构 bounded adaptation、projection/tracker/map recovery、完整在线开发门、性能余量门、x86 freeze、G5_V2 one-shot、30-seed map、30-seed Spot Cleaning/post-clean、2h soak/fault/replay/x86 release、J6、真实 RGB-D field、competition/neat-freak/PR。

在 parity 完成前禁止训练。只有 native D1-B 在独立 G7-MOVING VAL 失败，才允许同一 RTMDet-s 架构的 M1/M2 moving-domain adaptation。任何缺失证据均保持 unknown/false，不用总体 mission 计数推断逐目标通过，不用 GT 坐标生成产品 target，不四舍五入性能结果，不伪造 30-seed、soak、replay、J6、field 或外部同步。

## ODCV5-00 损失阶梯

入口：

```powershell
py -3 scripts/audit_odcv5_online_attrition.py `
  --benchmark F:\Project\TZcup\.workspace\artifacts\TZcup-ddrv4-online-dev-v6\base\DDRV4_BASE_MOVING.json `
  --route D1-B `
  --output-dir F:\Project\TZcup\.workspace\artifacts\TZcup-odcv5-20260812\attrition
```

输出 `ODCV5_ATTRITION_LADDER.json`、`ODCV5_ATTRITION_BY_CLASS.json` 和 `ODCV5_ATTRITION_BY_DOMAIN.json`。阶梯为严格单调链：下游附近的假目标不能让已经在 detector/class/depth 丢失的 GT 重新进入链。旧 DDRV4 报告没有逐目标 scheduler attribution，因此 `SCHEDULER_ACTIONABLE` 明确为 `UNKNOWN_LEGACY_EVIDENCE_GAP`，不是通过。

旧回放可审计结果显示，主要离散损失先发生在 action threshold 和 correct-class：59 个 actionable 目标中 7 个在 observation 阶段丢失；余下 52 个中 29 个未达到 0.53；余下 23 个中 13 个类别不正确。10 个正确类别且 depth-valid 的目标中，只有 3 个能与 0.50 m 内同类产品 map target 关联。旧格式不能把后一个复合损失进一步可靠拆成 projection、tracker 与 map 各自责任，因此下一步必须写入逐帧 trace 并执行 ODCV5-01 parity，不能据此开始训练。

## ODCV5-01 Golden-frame parity

`build_odcv5_golden_manifest.py` 从完整 capture 中按 RGB/depth/semantic/instance/camera/TF 哈希选择 100 positive + 50 negative 帧，选择与模型输出无关。当前 manifest 覆盖三类、small、turn、behind-FOV、occlusion、reflection、wet road、shadow、road marking、clutter、dark/bright pavement，RGB 精确重复为 0。

`audit_odcv5_golden_parity.py` 比较 `P0_NATIVE`、`P1_ADAPTER`、`P2_PRODUCT` 的 checkpoint、class map、BGR、resize/keep-ratio/pad、mean/std、observation/action threshold、NMS/top-K、decoded class/score/bbox，并要求 valid-depth correct detection 的 projection success 不低于 `0.98`。任何 pipeline trace 或精确 checkpoint 缺失都会 fail-closed。

当前 D1-B 历史路径已不存在，宿主、旧容器和已构建镜像中也未找到 SHA-256 为 `481374...a361` 的 checkpoint。因而 P0/P1/P2 尚未执行，`ODCV5_01_PASS=false`，`RUNTIME_CONTRACT_BUG` 仍未知而不是 false；恢复精确 checkpoint 字节前禁止训练和后续 moving gate。
