# OPRV3 在线优先感知恢复

OPRV3 是 PR #90 上独立于 X1/X2/X3 与 MRV2-A/B/C 的新协议。历史静态失败保持原样，`MODEL_BLOCKED_INTERNAL=true` 也不会因测量口径变化而被改写。新协议先回答移动清扫车在目标进入安全可行动窗口后，能否在错过清扫机会前完成发现、分类、跟踪和地图定位；旧的单帧、`<18 px`、AP、FP 与 area 指标继续作为诊断并完整报告。

## 测量边界

- `ObservableTargetEncounter` 和可见/遮挡/深度状态只由独立 evaluator 使用；生产 observation 不含 GT identity 或 GT 坐标。
- `ActionableObservationWindow` 在看任何 OPRV3 移动模型结果前，从已冻结的 AUTO-05R 相机、15 Hz、Nav2 清扫速度/减速度、控制延迟、刷盘前向偏置、Spot Cleaning 三次确认规则和 G4 真实物理尺寸推导。
- 低置信度 observation 只能进入多帧跟踪；observation、track confirmation 与 clean action 使用严格递增的独立阈值。
- 所有 GT target 都必须落入 `never_in_camera_frustum`、`occluded_entirely`、`visible_but_never_actionable` 或 `entered_actionable_window`，不得按模型结果事后缩小分母。

## 门槛来源

[`oprv3_gate_provenance.yaml`](../starter_ws/src/sanitation_learning/config/oprv3_gate_provenance.yaml) 将门槛分为 `OFFICIAL_GATE`、`INTERNAL_DIAGNOSTIC_GATE` 和 `ONLINE_PRODUCT_GATE`。2026-08-10 已核验[地平线官方赛题页](https://developer.horizon.auto/competition/848127658035142656)（赛题编号 DG-202604）：垃圾识别准确率要求不低于 95%，同时公开了清扫效率、定位精度、清扫宽度和建图面积要求。官方页面没有给出 accuracy 的混淆矩阵定义，因此开发门采用更保守映射：全产品目标 precision、recall、F1 均需不低于 0.95；`<18 px recall >= 0.70` 仍只作为内部诊断门，不冒充官方规则。正式提交绑定证据完成前 `COMPETITION_PERCEPTION_PASS` 保持 false。

## 阶段边界

OPRV3-00/01 的代码和解析测试只建立门槛溯源、事件 schema 与解析几何。公式审计不能替代 Gazebo 移动车辆实测；在至少每类 20 个目标的经验探针完成前，不允许声称 OPRV3-01 通过。现有 MRV2-C/MRV2-A/X3 只有完成不少于 20 个移动任务的开发矩阵后，才可决定直接进入在线集成或启动 G6/新模型恢复。

## OPRV3-01 移动相机经验探针

开发采集使用 AUTO-05R 产品相机姿态、`0.65 m/s` 指令速度、90 帧/任务、四路 RGB/depth/semantic/instance 严格同戳和 50 ms 内里程计匹配。采集回调在运动阶段采用有界内存缓冲，停车后再持久化，避免把 PNG/NPY 写盘速度误当相机节奏。通过集包含 24 条任务、2160 帧、4 个 world，其中 20 条正向任务为每类恰好 20 个 GT 目标，另有 4 条 negative-only 任务。全部通过任务均为 90/90 帧，最大传感器/里程计偏差 50 ms，实测速率中位数为 `0.65 m/s`。

严格四路 GT 的有效采样率为 `1.6425–8.4810 Hz`、中位数 `5.0898 Hz`，低于相机标称 15 Hz。100 个正向 GT 中 99 个取得至少 3 个实际可行动采样帧；1 个 metal-can 只有 1–2 个窗口采样，单列为 `insufficient_sampled_actionable_frames`，不计作模型漏检，也不从 `all_gt_targets` 删除。湿地长批次在共享 GPU 负载下多次墙钟超时，失败报告完整保留；同参数单场景湿地 smoke 曾 90/90 通过，只用于反射能力诊断，不冒充正式覆盖。

## OPRV3-02 现有模型前向开发矩阵

MRV2-C、MRV2-A 和 X3 在完全相同的 24 条任务上运行，G5 与 legacy D6 均未读取。GT 只在离线 evaluator 中构造窗口和匹配；生产模型只接收图像输入。冻结阈值分别来自各自 TRAIN holdout，低置信观察不会直接触发动作。

| route | eventual detection | eventual correct class | three-frame confirmation | actionable miss | wrong actionable rate |
|---|---:|---:|---:|---:|---:|
| MRV2-C | 1.0000 | 1.0000 | 0.9495 | 0.0000 | 0.01187 |
| MRV2-A | 1.0000 | 1.0000 | 0.9798 | 0.0000 | 0.00967 |
| X3 | 0.9899 | 0.9899 | 0.9495 | 0.0101 | 0.00458 |

MRV2-A 是当前前向开发候选：核心 eventual recall/class 与错误率满足 OPRV3-02 数值门；MRV2-C 的错误率超过 `0.01`，X3 漏掉一个 eligible metal-can。该矩阵仍不能通过完整 OPRV3-02，因为当前任务没有后方入视野、转弯、显式遮挡和正式反射覆盖，也没有完成在线地图/area 正式门。故不创建 freeze，不读取 sealed final，不启动 30-seed 或 Spot Cleaning 正式门，`MODEL_BLOCKED_INTERNAL=true` 保持不变。

原始外部证据：

- `F:\Project\TZcup\.workspace\artifacts\TZcup-oprv3-moving-benchmark-v1\OPRV3_MOVING_BENCHMARK.json`，SHA256 `4b0368dfe2ef4b9c4abd2fb2b997c12d088fb1279d95bb1af60e5c3024233298`。
- `F:\Project\TZcup\.workspace\artifacts\TZcup-oprv3-moving-benchmark-v1\PIXEL_DISTANCE_EMPIRICAL_REPORT.json`，SHA256 `1ae9660897117da3e23b486f58e3ba61930e1cc4259c78b6ed9755303586e6d9`。

## OPRV3-02 特殊覆盖恢复

采集器增加了显式 frame-counted SE(2) 运动剖面：普通直行仍必须满足邻帧平移门；转弯任务允许以独立的最小 yaw 变化接纳原地旋转帧，并在每条记录中持久化车辆 yaw、命令阶段、线速度和角速度。覆盖判定同时要求场景预声明与实测 GT 事实，不读取模型分数：转弯入视野要求目标先不可见、累计 yaw 后才可见；遮挡要求预声明的遮挡者与目标 GT 框产生实际重叠；反射要求湿地场景的完整通过 capture report。

转弯入视野任务为 90/90 帧，最大传感器/里程计偏差 9 ms，车辆最大绝对 yaw 变化 `3.0730 rad`。五类目标均由不可见进入视野；预冻结行动窗口中的 2/2 目标 eventual detection、正确分类和三帧确认均为 `1.0`，错误可行动预测为 0。显式遮挡任务同为 90/90 帧、最大偏差 10 ms；metal-can 与 paper-litter 的 GT 框产生实测重叠，MRV2-A 对 5/5 eligible 目标三项 recall 均为 `1.0`，错误可行动预测为 `0/42`。

反射覆盖采用物理场景事实而不是纯元数据标签：v6 在 `wet_dark_asphalt + overcast_diffuse` 世界中用“直行—短弧线—斜向直行”持续运动剖面取得 90/90 帧，最大同步偏差 9 ms、累计 yaw `0.4549 rad`、有效采样 `4.4800 Hz`。MRV2-A ONNX/CUDA 对 5/5 eligible 目标 eventual detection、分类和三帧确认均为 `1.0`，错误可行动预测为 0。此前直行容量不足、转弯卡滞和 48/90 等失败采集全部保留在仓库外，不覆盖 v6 证据。

24 条基础任务与三条独立特殊覆盖任务合计保留 115 个 GT；114 个进入预冻结窗口，MRV2-A eventual detection/classification 为 `114/114`、三帧确认为 `111/114=0.9737`、错误可行动预测为 `9/1113=0.00809`、漏清扫机会为 0。全部九类 moving coverage 均有 GT 实证，因此 `OPRV3_01_pass=true`、`OPRV3_02_pass=true`，下一阶段为 OPRV3-07。两条 moving 核心通过不能替代 Map/Track、Area 正式 mIoU/boundary/negative-area、4080 性能、30-seed、Spot Cleaning 或最终部署门；在 OPRV3-07 完成前，`MODEL_BLOCKED_INTERNAL=true`、`OPRV3_X86_DEV_PASS=false`。紧凑证据见 `artifacts/online_first_recovery_v3_20260810T042843Z/moving_dev/OPRV3_SPECIAL_COVERAGE_MATRIX.json`，完整外部报告保留在 `F:\Project\TZcup\.workspace\artifacts\TZcup-oprv3-special-benchmark-v1\`。

## OPRV3-07 产品开发门审计

`scripts/perception_oprv3_product_dev_gate.py` 以 fail-closed 方式合并 moving、Area、产品地图和性能证据；没有正式产物的指标一律为 `null/false`，不由 smoke 或公式推导代替。产品地图离线评分明确区分两个 GT 分母：检测 recall 只统计进入预冻结行动窗口的目标，地图 precision 则允许匹配所有真实进入相机视野的目标，避免把窗口外的正确在线发现误判为假阳性；生产流水线始终不接收 GT identity 或坐标。

ONNX/CUDA 诊断已对转弯、显式遮挡、湿地反射和动态移除各跑一条 90 帧真实 Gazebo 任务。四组产品 precision、地图 localization coverage、ID consistency 均为 `1.0`；false-confirmed、duplicate、fragmentation、pre-FOV creation、wrong-class clean action 均为 0，RMSE 分别为 `0.0470/0.0501/0.0372/0.0618 m`；动态移除后的 stale clean action 为 0。地图生命周期由“固定时长盲删”改为“位置被后续相机重新扫过且目标仍缺失才拒绝”，AREA 目标采用独立的 `0.50 m` 区域尺度关联以容纳视角变化引起的质心漂移。

commit `7aeb3fe502ac...` 的首轮正式基础矩阵完成 24 个任务、2160 帧，检测侧 eventual detection/classification 为 `99/99`、确认 `97/99`、错误可行动率 `9/931=0.009667`，但产品地图只匹配 `91/100`，coverage `0.91 < 0.95`，所以如实失败。8 个缺口来自 leaf：零下限投影诊断显示真实 leaf 物理面积中位数 `0.038421 m²`、最大 `0.044605 m²`，统一 `0.05 m²` 下限会系统性删除真实目标；远端杂散区域中位数仅 `0.001040 m²`。产品契约据此改为 leaf `0.02 m²`、puddle `0.05 m²`，仍分别要求四/六帧确认，并将实际 AREA 物理面积传入在线地图。该修复必须从新 commit 完整重算，当前仍不将 `OPRV3_X86_DEV_PASS` 置为 true。

commit `d52dfa20f0df...` 重算后，产品地图聚合达到 `116/120` coverage、`116/118` precision、RMSE `0.052033 m`、ID consistency `0.996218`，重复/碎片均为 `1/120`；300 帧 10.2 Hz 压测处理 `298/300`，effective `10.1249 Hz`、P95 `190.551 ms`、drop `0.006667`。但两个假确认令 false-actionable rate 为 `0.016949 > 0.01`，OPRV3-07 仍按实失败。seed 1 的额外 paper 目标距 leaf AREA 质心 `0.0227 m` 且点位实际位于四点 polygon 内；tracker 因而新增窄化规则：AREA 与 DISCRETE 不合并，只有离散点被 AREA polygon 包含时才抑制重复。无 polygon 或相邻但不包含的目标仍独立。seed 1 未提交真实 ONNX 重放得到 5 GT/5 产品目标，precision、coverage、ID 均为 `1.0`，false/duplicate/fragmentation 均为 0；仍需新 commit 全量重算后才能改变总门状态。

首次 Area 段直接使用未读取 G5/D6 的 MRV2-A 固定跨世界结果：leaf IoU `0.9118` 与 macro mIoU `0.8170` 通过，puddle IoU `0.7222 < 0.80`、boundary F1 `0.6388 < 0.75`、negative-area FP/frame `0.1304 > 0.05` 失败，因此严格路由到 OPRV3-06 Area 恢复。

## OPRV3-06 有界 Area 恢复与固定审计

恢复实现保持离散检测器不变，只训练 Area 分支：Area 训练改用 scene holdout 后完整 TRAIN 池，负样本先按对象 taxonomy、ground 与 lighting 做确定性均衡，再分层补足；从既有 `training_complete` DeepLab checkpoint warm start，冻结 backbone、geometry stem 与 BatchNorm，只更新 decoder/semantic/boundary heads。压缩帧缓存以 `float16` RGB 和 `uint8` target/boundary 保存并在 batch 边界恢复 `float32`，降低全量训练内存；RGB 读取仅允许三次有界重试，错误仍 fail-closed。v14 延续有界恢复并导出 leaf/puddle ONNX；离散检测器和 Area 三个 ONNX 会话均要求 `CUDAExecutionProvider`，CUDA 激活失败时禁止静默回退。此前 v1-v13 的依赖、OOM、并发 CUDA、BatchNorm、吞吐和未达门结果均保留在仓库外，不冒充成功训练。

`scripts/perception_mrv2_audit.py --area-only` 只在 VAL 扫描阈值/形态学，锁定 leaf `0.75 + open_close3`、puddle `0.70 + close3` 后原样评估 VAL 与 D1-D5；G5 sealed final 和 legacy D6 均未读取。`scripts/perception_oprv3_area_gate.py` 再按像素交并总量聚合，v14 结果为 leaf IoU `0.934790`、puddle IoU `0.965574`、macro mIoU `0.950182`、boundary F1 `0.767164`、negative-area FP/frame `8/390=0.020513`，五项全部通过。

完整固定门位于仓库外 `F:\Project\TZcup\.workspace\artifacts\TZcup-perception-product-runtime\oprv3-area-audit-v14\OPRV3_AREA_GATE.json`，leaf/puddle checkpoint SHA-256 分别为 `dc308d72...`、`f59a3b46...`。该 Area 门已由下述全量 OPRV3-07 重算绑定，不再是当前阻塞项。

## OPRV3-07 全量 x86 产品开发门结果

提交 `7053ff879b926e089714870cdb97126bb241b31a` 从 24 条基础任务、转弯、遮挡、反光和动态移除证据完整重算。Moving 聚合为 27 条任务、111 个 eligible 目标，eventual detection/classification 均为 `1.0`、确认率 `0.981982`、错误可行动率 `9/1085=0.008295`。产品地图聚合为 28 条任务、120 个 eligible 目标，matched `116`、confirmed `117`，precision `0.991453`、coverage `0.966667`、RMSE `0.052207 m`、ID consistency `1.0`，duplicate、fragmentation、pre-FOV creation、wrong-class clean action 与 removed-target stale action 均为 0。

正式性能门在 RTX 4080 Laptop GPU、`CUDAExecutionProvider` 和三个 hash-bound ONNX 会话上输入 300 帧，处理 297 帧，effective `10.083979 Hz`、端到端 P95 `160.792 ms`、drop `0.01`，且真实执行 detector、leaf、puddle、tracker、DynamicTrashMap 与 CleaningTaskScheduler。与 OPRV3-06 v14 合并后，官方对象指标保守映射 precision/recall/F1 为 `0.991453/0.966667/0.978903`，OPRV3-07 六个分区全部通过：`OPRV3_X86_DEV_PASS=true`、`MODEL_BLOCKED_INTERNAL=false`、`freeze_allowed=true`。正式门目录为 `F:\Project\TZcup\.workspace\artifacts\TZcup-oprv3-formal-7053ff8\gates\`；G5 sealed final 和 legacy D6 在开发门期间均未读取。

## OPRV3-08 x86 冻结契约

`scripts/perception_oprv3_freeze.py` 不训练、不推理且不改模型输出。它要求所有正式报告绑定同一完整开发 commit，逐项核验 detector/leaf/puddle checkpoint 与 ONNX 哈希、CUDA 性能、产品 pipeline 配置、第三方通知、资产许可证以及不可变 `repo@sha256` 容器；失败时不留下半成品目录。成功后原子生成 `MODEL_FREEZE_X86.json`、`PERCEPTION_X86_FREEZE_MANIFEST.json`、`PERCEPTION_X86_DEPENDENCY_LOCK.json`、`perception_pipeline_x86_frozen.yaml` 与 `SHA256SUMS`。开发证据 commit 与冻结工具 revision 分开记录，避免用后续工具或文档提交冒充重评测。冻结只授权一次 sealed-final 访问，不代表 sealed final、30-seed、Spot Cleaning、soak、J6 或现场门已经通过。

冻结同时绑定 `scripts/perception_oprv3_sealed_final.py`、`perception_oprv3_sealed_final_policy.yaml`、开发前冻结几何和 development world manifest。OPRV3-09 在数据访问前只校验 freeze、模型/ONNX、CUDA provider、manifest、世界/资产隔离与所有哈希；随后用原子 `O_EXCL` 写入唯一访问记录，才允许读取 G5 像素。一次评测同时执行静态 precision/recall/F1、AP、Area 像素/边界统计，以及移动相机 eventual discovery、产品 tracker、DynamicTrashMap、scheduler 和 pre-FOV/false-actionable 审计。无论通过或失败都写不可重跑结果；访问后的异常也视为访问已消耗，禁止借故查看数据后调参。

## OPRV3-09 G5 one-shot 结果

`G5_SEALED_FINAL` 定稿 QA 对 4 个未见世界、100 场、1000 帧全部通过：100 条独立移动序列、72 个静态目标场景和 28 个动态干扰场景满足形式门；世界、目标和 hard-negative 资产与 development 零重叠，pose reset、manifest-pixel、可见性、四传感器同步、camera 与 TF 均为 100%，跨世界 exact/pHash 重复为 0。freeze `oprv3-x86-7053ff8-v2-20260811` 的唯一访问记录于 `2026-08-11T04:32:16Z` 原子创建，三个 ONNX 会话首 provider 均为 CUDA。

最终结果为 `SEALED_FINAL_PASS=false`。通过项包括 object precision `1.0`、false-actionable `0.007126`、pre-FOV `0`、online small eventual recall `0.947368`、AP50:95 `0.770267` 和 negative-area FP/frame `0`。失败项包括 object recall/F1 `0.588889/0.741259`、minimum per-class recall `0.733459`、AP50 `0.784368`、online eventual detection/class `0.780980`，以及 leaf/puddle/macro IoU `0.218898/0.291642/0.255270`、boundary F1 `0.291522`。完整结果 SHA-256 为 `f9f1dc52...`；紧凑证据见 `artifacts/online_first_recovery_v3_20260810T042843Z/oprv3_09/OPRV3_SEALED_FINAL_SUMMARY.json`。本 G5 永久禁止复测、hard-negative mining 或参数选择；当前 freeze 被 sealed final 阻断，OPRV3-10 及之后所有产品门不得启动。下一候选必须只使用 development 数据，重新走 OPR-A/Area、OPRV3-07/08，并使用全新 `G5_V2_SEALED_FINAL`。

## G6 后三条有界 detector 路线

G6 独立审计通过后，OPR-A 的 class-agnostic small specialist 达到 `0.9885` VAL recall，但既有 classifier 与三轮有界适配均不能同时满足召回和背景 specificity。OPR-B 使用 Torchvision Faster R-CNN R50-FPN-v2 与显式 P2 8/12/16 像素锚点；holdout recall/precision 为 `0.9788/0.9519`，隔离 VAL 降至 `0.8109/0.9743`，其中 bottle/metal recall 仅 `0.7723/0.6732`。

最后一条 OPR-C 在执行前锁定 Apache-2.0 的 MMDetection v3.3.0 RTMDet-s 和官方 checkpoint `387a891e...`。2400 帧、10 个 fit world 的 8-epoch 微调耗时 569 秒，按独立 TRAIN-world holdout COCO mAP 选定 epoch 7；固定阈值 `0.30` 的 holdout recall/precision 为 `0.9859/0.9777`，但首次隔离 VAL 为 `0.9075/0.8843`、unmatched FP/frame `0.11875`，bottle/metal/paper recall 为 `0.9286/0.8277/0.9663`。因此 `OPR_A_PASS=false`、`OPR_B_PASS=false`、`OPR_C_PASS=false`，禁止扩展 OPR-D/E/F，离散 detector 继续 `MODEL_BLOCKED_INTERNAL=true`。最小下一研究需求是新的独立材质/光照开发包，而不是追加 GPU 或读取任一 sealed G5；OPRV3-06 Area 作为独立分支继续执行。完整训练与 checkpoint 留在仓库外，紧凑证据见 `artifacts/online_first_recovery_v3_20260810T042843Z/opr_b/` 和 `opr_c/`。

## G6 Area 独立恢复

新 Area 候选只读取 G6 development 数据。共享轻量编码器在高分辨率 decoder feature 上分别生成 leaf/puddle semantic 与 boundary heads，并加入 taxonomy hard-negative 损失；10 个 TRAIN world 用于拟合，第 11 个 TRAIN world 专用于 checkpoint 与后处理选择。selected epoch 4 的 holdout leaf/puddle IoU 为 `0.9136/0.9892`、boundary F1 `0.8273`、negative-area FP/frame `0`。formal-v1/v2 的训练后导出分别被过严的 raw-logit 和 probability-only 容差挡下，失败目录完整保留；formal-v3 没有追加训练，以 probability error ≤`0.005`、semantic/boundary 二值一致率 ≥`0.99999`、custom op `0` 的解码相关合同完成两个固定输入 ONNX，semantic mask agreement 均为 `1.0`。

锁定 leaf `0.70 + no morphology + 0.0005 m²`、puddle `0.40 + no morphology + 0.001 m²` 后，首次评估 G6 VAL 与 D1-D5 共 2800 帧。像素总量聚合 leaf/puddle/macro IoU 为 `0.955664/0.990752/0.973208`，boundary F1 `0.847253`，10 类已标注 hard-negative 区域的 actionable FP/frame 为 `2/2800=0.000714`，D4 为 `1/400=0.0025`，故五项 `OPRV3_06_AREA_PASS=true`。最小物理面积与有效深度约束已实际进入静态门；三帧两命中的已配准 temporal filter 已实现并测试，但 G6 scene 内各帧随机生成，不能冒充时序实证。Area 通过不能抵消 detector A/B/C 失败，整体仍 `OPRV3_X86_DEV_PASS=false`、`MODEL_BLOCKED_INTERNAL=true`，禁止 freeze 或读取 G5_V2。紧凑证据见 `artifacts/online_first_recovery_v3_20260810T042843Z/oprv3_06_g6/OPRV3_G6_AREA_SUMMARY.json`。
