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

输出 `ODCV5_ATTRITION_LADDER.json`、`ODCV5_ATTRITION_BY_CLASS.json`、`ODCV5_ATTRITION_BY_DOMAIN.json` 和 `ODCV5_ROOT_CAUSE_DECISION.json`。阶梯为严格单调链：下游附近的假目标不能让已经在 detector/class/depth 丢失的 GT 重新进入链。旧 DDRV4 报告没有逐目标 scheduler attribution，因此 `SCHEDULER_ACTIONABLE` 明确为 `UNKNOWN_LEGACY_EVIDENCE_GAP`，不是通过。

旧回放可审计结果显示，主要离散损失先发生在 action threshold 和 correct-class：59 个 actionable 目标中 7 个在 observation 阶段丢失；余下 52 个中 29 个未达到 0.53；余下 23 个中 13 个类别不正确。10 个正确类别且 depth-valid 的目标中，只有 3 个能与 0.50 m 内同类产品 map target 关联。根因决策精确归因 `DETECTOR_SCORE_LOW=29`、`DETECTOR_WRONG_CLASS=13`、`OUTSIDE_ACTIONABLE_WINDOW=1`。旧格式不保留完整 proposal 列表，因而 7 个 observation miss 只能标为 `DETECTOR_NO_PROPOSAL`/`DETECTOR_BOX_IOU_FAIL` 未决；另 7 个复合损失不能可靠拆成 projection、tracker 与 map 各自责任。缺口保持显式 unknown，不能据此开始训练。

## ODCV5-01 Golden-frame parity

`build_odcv5_golden_manifest.py` 从完整 capture 中按 RGB/depth/semantic/instance/camera/TF 哈希选择 100 positive + 50 negative 帧，选择与模型输出无关。当前 manifest 覆盖三类、small、turn、behind-FOV、occlusion、reflection、wet road、shadow、road marking、clutter、dark/bright pavement，RGB 精确重复为 0。

`audit_odcv5_golden_parity.py` 比较 `P0_NATIVE`、`P1_ADAPTER`、`P2_PRODUCT` 的 checkpoint、class map、BGR、resize/keep-ratio/pad、mean/std、observation/action threshold、NMS/top-K、decoded class/score/bbox，并要求 valid-depth correct detection 的 projection success 不低于 `0.98`。任何 pipeline trace 或精确 checkpoint 缺失都会 fail-closed。

当前 D1-B 历史路径已不存在，宿主、旧容器和已构建镜像中也未找到 SHA-256 为 `481374...a361` 的 checkpoint。因而 P0/P1/P2 尚未执行，`ODCV5_01_PASS=false`，`RUNTIME_CONTRACT_BUG` 仍未知而不是 false；恢复精确 checkpoint 字节前禁止训练和 ODCV5-03 detector moving gate，但不阻止不读取模型的 ODCV5-02 数据构建。

## ODCV5-02 G7-MOVING Development Pack

入口：

```powershell
py -3 scripts/build_odcv5_g7_moving.py `
  --output F:\Project\TZcup\.workspace\artifacts\TZcup-odcv5-20260812\g7-moving-development
```

生成器从空目录建立严格开发用途的 `MOVING_TRAIN`、`MOVING_HOLDOUT` 和 `MOVING_VAL`，正式规模为 `30/10/15` missions、每 mission 18 帧，共 990 帧。每帧落盘同步 RGB、depth、CameraInfo、timestamp、vehicle pose/TF、semantic GT 和 instance GT；GT 只存在于 evaluator manifest，产品输入 manifest 不包含 class、GT 坐标或 instance ID。

`G7_MOVING_QA.json` 要求所有 mission 完整、帧数精确、sensor timestamp 差为 0、world/seed split 隔离、RGB exact 与跨 split pHash 重复为 0，并要求每个 split 覆盖 straight、behind-FOV、turning、occlusion/reappearance、reflection、wet/dark/bright road、shadow、road paint、clutter、small/distant、dynamic insertion/removal 和 negative-only mission。当前正式包 `required_coverage_complete=true`、`G7_MOVING_PASS=true`；TRAIN 每类 58 个、VAL 每类 28 个 actionable encounter，VAL 每类 14 个 first-visible `<18 px` encounter。

该结果只完成数据与 QA 门，不代表 D1-B moving detector 通过。由于 ODCV5-01 仍 fail-closed，ODCV5-03 未执行，训练仍禁止。
