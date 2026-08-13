# TZcup 智慧环卫无人清扫车

TZcup 是一个基于 ROS 2 与 Gazebo 的智慧环卫无人清扫车工程，覆盖车辆与园区数字孪生、自主定位导航、全覆盖清扫、垃圾感知、定点清扫、安全控制、人机界面和可审计验收。

项目面向 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic，支持在 Windows WSLg 中运行可视化演示，也支持通过 Docker 执行无头验证。

## 核心能力

- 四轮差速清扫车、刷盘、传感器与结构化园区场景建模；
- SLAM、融合定位、Nav2 导航、keepout 与安全速度控制；
- 基于 OpenNav Coverage / Fields2Cover 的全覆盖路径规划与补扫；
- RGB-D 垃圾检测、跟踪、地图融合和定点清扫任务调度；
- Gazebo、RViz 与浏览器看板组成的调试和监督界面；
- 多场景回归、证据清单、模型注册、回放与发布检查工具。

## 系统边界

正式任务启动时只预载道路、可清扫区域、静态障碍和安全约束，不预载垃圾坐标。垃圾目标必须由车载 RGB-D 在车辆运动中发现，经多帧确认、时间戳 TF 投影和动态地图融合后，才能进入清扫调度；Gazebo 真值仅供独立评测使用。

当前仿真、导航、覆盖清扫、安全链和可视化演示可运行。学习感知仍处于 fail-closed 验证阶段，尚未取得完整在线质量门、J6 实板和真实场地产品验收，因此本仓库不能被表述为已经完成实车产品部署。详细边界见 [当前状态](docs/current-status.md) 和 [Detector Data Recovery V4](docs/detector-data-recovery-v4.md)。

## 快速体验

在 Windows PowerShell 的仓库根目录运行完整可视化演示：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1 -Video on
```

只启动 Gazebo 清扫演示：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1
```

界面中的橙色外框表示任务范围，青绿色区域表示实际可清扫范围；规划、实际轨迹与已清扫区域使用不同图层显示。

首次安装、WSLg 配置、ROS 工作空间构建和其他启动方式见 [README_FIRST.md](README_FIRST.md)。

## 开发与验证

快速执行不依赖 ROS 的仓库检查：

```powershell
py -3 scripts/ci_fast.py
```

ROS、Gazebo、导航或运行时变更还需要执行对应的 Stage 验收，不能用快速检查代替。分支、PR、CI、证据和交付约束见 [开发工作流](docs/development-workflow.md) 与 [证据策略](docs/artifact-policy.md)。

## 目录结构

| 路径 | 内容 |
|---|---|
| `starter_ws/src/` | 自研 ROS 2 功能包 |
| `scripts/` | 安装、构建、启动、训练、评测与证据工具 |
| `config/` | 系统、任务和验收配置 |
| `docs/` | 架构、操作、验证协议与当前状态 |
| `artifacts/` | 适合纳入 Git 的紧凑验收证据 |

## 文档入口

- [项目规格](PROJECT_SPEC.md)
- [当前状态](docs/current-status.md)
- [环境与启动](README_FIRST.md)
- [操作指南](docs/operator-guide.md)
- [车辆模型](docs/vehicle-model-guide.md)
- [覆盖路径优化](docs/coverage-path-optimization.md)
- [数字孪生场景](docs/gazebo-digital-twin-scene.md)
- [工业化与 Sim2Real](docs/industrialization-and-sim2real.md)
- [故障回滚](docs/rollback.md)

## 许可证

项目代码及第三方组件的使用边界见 [LICENSE.md](LICENSE.md)。

## CRV6 感知恢复进展

`CHECKPOINT-RECONSTITUTION-V6` 已关闭历史 D1-B checkpoint 的最后一次有界恢复搜索：历史 SHA-256 `481374...a361` 的字节仍未找回，历史 DDRV4 D1 通过事实不改写。当前按 R1 使用已审计的 D1-B 初始化、冻结 G7 static TRAIN/HOLDOUT 边界和相同六轮协议重构出新候选；新候选必须以自己的 SHA 和 provenance 标识，不得冒充历史 D1-B。

CRV6 工具链覆盖恢复审计、R1 provenance、static VAL 非门禁回归、golden native/runtime parity、G7-MOVING HOLDOUT/VAL 原生门、有界 MA1 moving-domain adaptation，以及严格分离 discrete/area/combined 指标的真实 Gazebo 在线审计。MA1 在独立 G7-MOVING VAL 通过，但真实 Gazebo 24-mission 回放的离散 detector/map 门未通过；现有 G6 Area 的 boundary F1 和 negative FP/frame 也未达到 CRV6 阈值。因此 `MODEL_BLOCKED_INTERNAL=true`，禁止 freeze、读取 G5_V2 或声明产品就绪。大型数据、checkpoint 和逐帧 trace 保存在仓库外 evidence 根目录。

后续 GOCV7 工具可在不读取 G5_V2 或正式 30-seed 数据的前提下，对代表性真实 Gazebo mission 执行 native MMDetection、产品 adapter 与完整 detector 入口的同帧 trace，并准备 world 隔离、哈希去重的 development-only GA1 TRAIN/HOLDOUT 数据。只有这些开发门及下游在线门全部通过后，才允许进入 x86 freeze 与一次性 sealed-final 验收。

GA1 路由严格限定为一次由 MA1 warm start 的真实 Gazebo TRAIN 微调，checkpoint 和 action threshold 只由 world 隔离的 GA1 HOLDOUT 选择；训练、选择报告同时绑定源码 commit、容器镜像 digest、checkpoint/config SHA，并显式证明既有 24-mission、G5、G5_V2 与正式 30-seed 在冻结前均未读取。现有 G6 Area 候选的跨世界固定门已达到 macro mIoU `0.973208`、boundary F1 `0.847253`、negative actionable FP/frame `0.000714`，GOCV7 不再启动 Area backbone 或额外 boundary-head 训练，后续仅允许验证固定 G6 模型的产品运行时接入与在线链路。

正式 24-mission benchmark 只接受 `GOCV7_GA1_HOLDOUT_ONLY` 且已经通过的哈希绑定 selection；任何 selection 失败、checkpoint 不一致或在阈值冻结前读取正式回放的记录都会 fail-closed。

GA1 数据准备要求 24 个固定 development seed 全部存在、每任务至少 20 个同步帧且总量不少于提示词要求的 300 帧；这一门槛允许低实时倍率 Gazebo 世界在保留超时失败证据后缩短单任务采集，但禁止缺 world、缺 seed、跨 split 重复或用不足样本冒充通过。

DDRV4、CRV6 与 GOCV7 三类 HOLDOUT selection 由同一正式 benchmark 入口兼容校验；旧路线的哈希、VAL 未读边界保持不变，GA1 另外要求 HOLDOUT gate 通过且正式 24-mission 在阈值冻结前未读取。GA1 精确 RGB 哈希重复在同一 split 内确定性保留首帧并审计丢弃，任何 TRAIN/HOLDOUT 跨 split 重复仍立即失败。

GA1 HOLDOUT 的 actionable precision 与产品链一致：正确匹配到冻结行动距离之外、随后会被深度投影范围门拒绝的可见目标不计为动作或误动作；范围内正确匹配与未匹配/错类预测才进入 precision/wrong-actionable 统计。阈值只在预先固定的 `0.05–0.95` 网格内选择。

若唯一 GA1 fine-tune 与有界阈值修复仍不能通过 HOLDOUT，GOCV7 必须在读取正式 24-mission 前停止，生成六个强制 BLOCKED 最终文件；性能、freeze、G5_V2、30-seed、Spot Cleaning、soak、MCAP、release、Ready/Merge 与部署保持锁定。

`REAL-GAZEBO-DETECTOR-RECOVERY-V8` 在上述 fail-closed 基线上授权最多三条有限 detector 恢复路线。第一阶段只对已消费的 GA1 development HOLDOUT 做逐目标失败审计，固定使用 GOCV7 阈值且不得调参或读取新 VAL、G5_V2 与正式 30-seed；只有回答 small miss、false actionable 背景类型和最大错报类别后，才能构建 world/seed/asset 隔离的 G8 real-Gazebo development pack。最终仍以真实 detector、tracker/map、在线性能、冻结、sealed final、30-seed、清扫、soak、回放和 x86 release 全门通过为仿真产品完成条件。

G8 的显式 detector 采集模式在每个正任务中放置三类各 4 个独立 Gazebo 实体，并使用 instance camera 的稳定实体标签逐个计数；旧 G4 默认仍保持每类 1 个目标。G8 的 15-mission 训练周期固定 5 个 negative-only，10-mission HOLDOUT/VAL 周期固定 3 个 negative-only，从而用真实 encounter 数满足配额，禁止把同一目标的多帧重复观察冒充多个 encounter。

G8 自动域矩阵只在显式开关下启用，正任务按冻结序列执行普通接近、转弯/后入视野、遮挡、动态移除和动态插入；动态插入/移除均由 Gazebo pose 服务在冻结帧触发并写入 capture report，任一事件未执行都会使采集 fail-closed。反射任务只能在实际 wet world/material 上运行，不能仅靠任务标签声明覆盖。

G8 wet/specular 数据由可复跑派生器从对应 split 的独立 base world 生成：派生过程修改 Gazebo world id、地面 PBR roughness 与低角度高光灯，重新计算 SDF/manifest SHA，并逐字节校验地面纹理资源闭包；训练、HOLDOUT、VAL 各自使用其 split 内不同 base world，禁止跨 split 复用同一 world 或资产。

G8 数据准备器独立重读每个 real-Gazebo 任务的 RGB、semantic/instance mask、TF 与 capture report，按稳定 instance id 统计真实 encounter 和首次可见尺寸，物化三份封闭 COCO 索引，并 fail-closed 检查任务/负样本配额、域角色、四传感器同步以及 world/seed/asset/RGB/pHash 跨 split 零重叠；帧数建议值不能替代 encounter 硬门槛。

派生 wet world 运行时把只读 world/纹理根与只读模型资产根分别挂载，并在启动前验证 `models/` 存在；这使每个 wet SDF 保持独立 SHA 和 split 身份，同时继续使用同一 split 的原生 Gazebo 模型资产，而不会把模型复制进派生证据目录。

若独立像素审计发现跨 split 的 pHash 冲突，只允许按 `split:world:seed` 隔离整个任务并把原 manifest/capture SHA 写入 split manifest；禁止删除单帧来伪造任务完整性，隔离后全部配额和泄漏门必须从零重算。

Route A 先物化 `LEGACY_GA1_TRAIN + G8_TRAIN_NEW` 的 TRAIN-only 源池，再用 SHA 绑定的 GA1 checkpoint 以低阈值挖掘至少 2000 个 proposal hard-negative crops；冻结 exposure 为 small 25%、metal 20%、general 25%、hard-negative 30%，训练阶段不读取 HOLDOUT_NEW、VAL_NEW 或 G5_V2，HOLDOUT_NEW 只用于 checkpoint 与阈值选择。

Route A 保持官方 MMDetection RTMDet-s 和 640x480 产品输入，从 GA1 checkpoint warm-start；每个 epoch checkpoint 与全局阈值只在 HOLDOUT_NEW 上按 eventual correct-class recall、small eventual recall、actionable precision 和 wrong-actionable 硬约束联合选择，选择冻结前不得读取 VAL_NEW。

Route B 只在 Route A 的高召回候选无法满足 wrong-actionable 门时启用：固定生成 TRAIN/HOLDOUT proposals，使用已有官方权重 MobileNetV3-Small 四分类 crop verifier（含 background），要求 TRAIN unique positive/background crops 各至少 3000；HOLDOUT proposals 只生成一次，不回流 detector，VAL 仍保持未读直到组合策略冻结。

Route B verifier 从每类固定抽取 4000 个 unique TRAIN crops 并将 TRAIN/HOLDOUT 图像各预载一次，规避 Docker Desktop 小文件挂载的重复 I/O 阻塞；不改变 16-epoch、四类平衡、全量 HOLDOUT threshold sweep 或任何精度门槛。

Route C 是有限 detector 恢复的最后路线：保留 Route B 已固定的 proposal 坐标与标签，使用产品可部署的 `square_crop(scale=6, minimum_side=64)` 增加地面上下文并强化同一个 MobileNetV3-Small verifier 的 hard-negative 能力；HOLDOUT 只用于冻结，VAL_NEW 在候选冻结前保持未读。

Route C 的 contextual crop 必须回溯 Route A 的 `source_train.json`（`G8 TRAIN_NEW + legacy GA1 development`）和固定 `holdout.json`，以保持与 Route B proposal 的 image-id/source-pool 一致；不得把仅含 G8 的 `prepared_final_v2/fit.json` 误作完整 TRAIN_COMBINED 索引。

RGDRV8 specialist 使用 G8 校准相机的固定 3×3 overlapping ground-mask tiles（原生 320×240、统一放大到 640×480）。G8 的远地面目标可投影至图像顶边，因此旧 G6 仅覆盖下 75% 的六块 ROI 不适用于 G8；九块 ROI 覆盖完整 640×480 cleanable image mask，且生产选区函数不接收 GT。

RGDRV8 严格执行有限路线停止条件：仅当 A/B/C 都有 HOLDOUT 失败证据时，发布 `MODEL_BLOCKED_INTERNAL_REAL_GAZEBO_DETECTOR=true`、`SIMULATION_PRODUCT_COMPLETE=false` 和 `NEXT_ARCHITECTURE_RESEARCH_REQUIRED.json`；此状态下保持 VAL_NEW/G5_V2 未读，并将 tracker/map、在线任务、性能、freeze、30-seed、清扫、soak/replay 与 x86 release 明确标为 dependency-blocked，而不是用未执行结果冒充产品完成。

TGARV9 在不改写上述失败事实的前提下，将单帧 detector P/R/F1/AP 诊断、track 级确认、产品 actionable target 与实际清扫动作分层审计；任何 temporal/geometry 恢复都必须先通过全新 G9 真实 Gazebo HOLDOUT，生产输入仅限 RGB、depth、CameraInfo 与 TF，TargetTube 真值只进入独立 evaluator。T2/T3 的有界训练逐轮保留 checkpoint，并在训练结束后由统一选型器对全部 checkpoint 运行相同 G9 与三种冻结 temporal/geometry policy，禁止把 HOLDOUT 反馈写回训练。最终 wrong confirmed actionable 仍要求不高于 1%，错误或虚假目标清扫仍必须为零。

T2 使用哈希绑定的 MMDetection v3.3.0 官方 DINO 4-scale R50 improved 权重；选型先满足全部 track-level 产品硬门，再以 detector AP50:95 作为次级优化量，并报告逐类 P/R/F1 与 `<18 / 18–32 / >32 px` 召回。只有 G9 产品门通过后才允许执行 300–500 帧、batch=1、CUDA AMP 的 `>=5 Hz` 部署性预筛；预筛失败直接进入最后一条 T3，不得读取 `VAL_NEW`。

最终 evidence index 以 `RGDRV8_GA1_FAILURE_TAXONOMY.json` 作为 GA1 failure-forensics 主记录，并单独保留 confusion、score 和 size/domain 辅助矩阵；发布器在所有必需外部证据存在且三条路线状态确认为失败后才生成 final 目录。
