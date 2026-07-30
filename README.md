# TZcup 无人清扫车仿真项目

## AUTO-12 清扫效率重构候选（2026-07-30）

AUTO-12 已建立 opt-in `auto12_efficiency_v1` 候选：展开作业宽度 `1.32 m`、清扫速度 `1.0 m/s`、加/减速度 `1.0/1.2 m/s²`，物理刷组、清扫 footprint、碰撞几何、Nav2 costmap、Coverage、动力学与能耗配置使用同一参数源；既有 `0.65 m` 生产默认保持不变。候选将通过时间步进动力学与 `0.10 m` 栅格清扫矩阵验证，证据等级严格限定为 `OFFLINE_TIME_STEP_DYNAMICS_AND_RASTER_SIMULATION`，不冒充 Gazebo 或实车实测。

## AUTO-11 20,000 m² 大地图与定时任务（2026-07-30）

AUTO-11 已通过 200 m × 100 m、0.1 m 分辨率的 20,000 m² 大地图与定时任务正式门：20 个 zone/submap 可序列化重载，10 条轨迹逐条 XY RMSE 均小于 `0.05 m`（最大 `0.03004 m`），丢失恢复率 `0.95`、TF continuity `0.99998`；5 次全覆盖和 20 次定时任务的 zone accuracy `1.0`、boundary/collision 均为 `0`、中断恢复率 `0.96`。GT 来自独立仿真 world-state，估计器使用另一随机观测模型，未做里程计自比较。`AUTO-11=PASS`；紧凑证据见 [`artifacts/autonomous_auto11_20260730_evidence/`](artifacts/autonomous_auto11_20260730_evidence/)，说明见 [`docs/auto11-large-map-scheduling.md`](docs/auto11-large-map-scheduling.md)。证据等级是 `OFFLINE_LARGE_MAP_SIMULATION`，不冒充 Gazebo、真实车辆或 J6 板端。

## AUTO-10 APP / speech / constrained task DSL（2026-07-30）

AUTO-10 已实现并通过独立的本地 APP/API、鉴权授权、幂等提交、GPU 语音识别和 fail-closed 任务 DSL 正式门。288 个真实 HTTP/API/UI 用例中合法成功率和非法拒绝率均为 `1.0`，P95 `16.03 ms`；1200 个双语/歧义/冲突/不安全 DSL 用例的语义分解、工具选择和参数准确率均为 `1.0`，危险执行率和直接执行器访问均为 `0`；500 条 Windows TTS→四档噪声/三类混响→GPU Whisper 用例覆盖三音色、三语速和中英文，意图准确率 `0.9911`、危险命令拒绝率 `1.0`、端到端 P95 `171.94 ms`。真实浏览器另验证了桌面与 390 px 窄屏布局、“紧急停止”安全 DSL 和直接电机指令拒绝。`AUTO-10=PASS`；紧凑证据见 [`artifacts/autonomous_auto10_20260730_evidence/`](artifacts/autonomous_auto10_20260730_evidence/)，说明见 [`docs/auto10-multimodal-hmi.md`](docs/auto10-multimodal-hmi.md)。该结论不代表车辆执行、真实域或 J6 板端通过。

## AUTO-13 真实域工具链与资源发现（2026-07-30）

独立 AUTO-13 lane 已实现显式同意的相机/视频采集、落盘前隐私模糊、棋盘格标定、带 SHA 的数据接入和离散/区域/定位统一评测。程序化 fixture 已验证软件合同，但不计为真实域证据。资源发现识别到 1 个本机 Integrated Camera 和仓库内 249 个图像文件，但没有任何满足至少 20 个真实 scene/1000 frame、五类完整、hard-negative、标定和独立 map localization GT 的可审计 manifest。因此 `AUTO-13=BLOCKED_EXTERNAL`、`REAL_DOMAIN_BLOCKED_EXTERNAL=true`、`REAL_DOMAIN_PASS=false`；相机存在不等于真实 GT 数据就绪。紧凑证据见 [`artifacts/autonomous_auto13_20260730_evidence/`](artifacts/autonomous_auto13_20260730_evidence/)，说明见 [`docs/auto13-real-domain.md`](docs/auto13-real-domain.md) 与 [`docs/real-domain-annotation-protocol.md`](docs/real-domain-annotation-protocol.md)。

## AUTO-04 双模型 micro-overfit 机器门通过（2026-07-30）

AUTO-04 已落地真正的 object-level anchor-free detector 与独立 leaf/puddle area segmenter。detector 直接输出三类中心 heatmap、中心 offset 和 bbox 尺寸，经 confidence-ranked decode 与 class-wise NMS 得到目标框，不使用语义分割连通域冒充 detector；area head 独立训练与导出。第二轮正式 GPU 运行通过全部冻结机器门：detector AP50 `0.99670`、三类 recall 均为 `1.0`、negative-only FP/frame 为 `0`，ONNX 最大数值误差 `1.1444e-05` 且 decoded agreement 为 `1.0`；leaf/puddle IoU 分别为 `0.98106/0.96914`、macro mIoU `0.97510`、negative-only area FP/frame 为 `0`，ONNX 最大误差 `9.1553e-05`、argmax agreement 为 `1.0`。第一轮失败仍作为 prior attempt 保留，紧凑证据见 [`artifacts/autonomous_auto04_20260730_evidence/`](artifacts/autonomous_auto04_20260730_evidence/)。`AUTO-04=PASS`，自主状态已推进到 AUTO-05；本结论仅证明 Gazebo micro train-set capacity，不代表跨世界、真实域、J6 或竞赛感知通过。实现说明见 [`docs/auto04-micro-overfit.md`](docs/auto04-micro-overfit.md)。

## AUTO-03 Oracle 主动观察闭环正式矩阵通过（2026-07-30）

AUTO-03 已实现不依赖观察位姿真值的主动观察任务链：带噪候选进入队列后，车辆在 Coverage 组件边界暂停，经观察位姿采样、`ComputePathToPose` 预检和 `NavigateToPose` 靠近，完成同步图像捕获与 evaluation-only 机器可判定评测，再返回边界并恢复覆盖。Oracle 只发布带噪平面位置、协方差、时间戳、通用类别/尺度以及 false/stale 状态；planner、Nav2、控制器和执行器均不得订阅语义 GT，也不得由 Oracle 设置车辆位姿。

正式确定性矩阵为 6 个 G2 Gazebo 世界、60 个 scene、250 条 trial，并逐世界记录 19 个必需话题的 MCAP 后实际重放审计。首轮完整矩阵只有预测目标短边相对误差 P95 `0.31173` 未达到 `≤0.30`，其余任务、安全、中心投影和成本门均通过；该轮已作为失败尝试保留。短边模型只用 A–D 世界拟合，E–F 留出验证后重新执行完整矩阵：路径预检、可达导航、机器可判定和 Coverage 恢复均为 100%，碰撞、keepout 和 GT 控制违规均为 0；170 个投影样本的中心误差 P50/P95 为 `7.89398/18.43392 px`，短边相对误差 P50/P95 为 `0.09270/0.29771`，中位额外时间 `19.502 s`、吞吐损失 `19.598%`。六个 MCAP 均达到 19/19 话题覆盖且重放审计通过。紧凑证据见 [`artifacts/autonomous_auto03_20260729_evidence/`](artifacts/autonomous_auto03_20260729_evidence/)，自主状态已推进到 AUTO-04；实现说明见 [`docs/auto03-oracle-active-observation.md`](docs/auto03-oracle-active-observation.md)。

## 本机 Ubuntu 24.04 WSLg 图形验收通过（2026-07-29）

当前 Windows 主机已新增完全位于本地磁盘的 `TZcup-Ubuntu-24.04` WSL2 发行版，并在 Ubuntu 24.04.4、ROS 2 Jazzy、Gazebo Sim 8.11.0 与 WSLg 中完成真实图形复核。Gazebo 三维场景和 Nav2 默认 RViz 布局均实际渲染；D3D12 renderer 为 NVIDIA GeForce RTX 4080 Laptop GPU，OpenGL 4.6、硬件加速为 `yes`。当前 `main@11ee369` 的全工作空间结果为 `449 tests / 0 errors / 0 failures / 49 skipped`，运行中 smoke check 为 `11/11` 必需 topic、`missing_topics=[]`、`success=true`。

紧凑机器证据见 [`artifacts/wslg_gui_20260729_evidence/`](artifacts/wslg_gui_20260729_evidence/)，本机运行说明见 [`docs/compatibility.md`](docs/compatibility.md)。这补足的是本机 WSLg 图形运行与基础 ROS topic 验收，不改变 AUTO-02 算法结论，也不代表真人审计、真实车辆、真实域、J6 或竞赛效率门通过。

## AUTO-02 完整导航回归与配置冻结通过（2026-07-29）

AUTO-01 选出的 opt-in `G2-C3 / V5_retracted` 已在 Docker 中完成 ROS 2 Jazzy + Gazebo Harmonic 全矩阵回归，并冻结为 [`autonomous_navigation_profile_v1.yaml`](starter_ws/src/sanitation_navigation/config/autonomous_navigation_profile_v1.yaml)。静态五个 seed 均完成 `17/17` 组件，经验覆盖率为 `0.92733–0.94467`、计划覆盖率均为 `0.986`、定位 XY RMSE 为 `0.03153–0.04050 m`，碰撞、keepout 违规和刷盘状态违规均为 `0`；五次冷启动均在 `24–25 s` 内达到完整 lifecycle/TF/参数就绪。

动态门完成 `20/20` 有效交互且碰撞为 `0`，最小观测分离距离 `0.60439 m`，高于配置硬阈值 `0.12 m`；禁入区违规为 `0`，限速区均速 `0.28613 m/s`，低于含容差上限 `0.3135 m/s`；急停 `30/30`，P95/最大停止延迟为 `0.13994/0.14063 s`，停止后输出持续为零。五个静态 bag 和一个动态 bag 的 MCAP 元数据、必需主题、任务状态回放及经验覆盖率重算全部通过，覆盖率相对误差均为 `0`。紧凑证据见 [`artifacts/autonomous_auto02_20260729_evidence/`](artifacts/autonomous_auto02_20260729_evidence/)，复现说明见 [`docs/auto02-full-regression.md`](docs/auto02-full-regression.md)；自主状态已推进到 AUTO-03。

本结论仅是机器仿真回归与配置冻结，不改变 production 默认 profile，不代表真人审计、真实车辆、真实域、J6 板端或最终竞赛指标通过。历史 Stage4W–Stage5BR6W 证据和两项 Stage5BR6-A 人工 false 标志保持不变。

## AUTO-01 几何与低矮障碍闭环通过（2026-07-29）

自主主线已选择 opt-in `G2-C3`：相机采用机械回收态 `V5_retracted`，保持 Stage4W 的 `0.40 m × 0.36 m` 导航 footprint 不变；向下 RGB-D 点云先转换到 `base_footprint`，只剔除已知车体包围盒内的自点，再与原始 LiDAR 一起送入单级 Collision Monitor。生产默认相机和默认 footprint 均未改变。G1 的三个配置、G2-C1 与 G2-C2 均按各自首个失败层保留为拒绝证据，没有用降低门槛覆盖失败事实。实现与复现说明见 [`docs/auto01-geometry.md`](docs/auto01-geometry.md)。

机器门结果为：冷启动 3/3，Nav2 参数服务 25/25/26 秒就绪；seed0 完整覆盖 17/17，经验覆盖率 `0.932`、碰撞 `0`、keepout 违规 `0`、定位 RMSE `0.03391 m`、swath 冲突 `0`，MCAP 回放通过；低障碍 `30/30`、高障碍 `30/30`，碰撞 `0`、height-classification false-safe `0`。[PR #30](https://github.com/zhexuexiaotudou/TZcup/pull/30) 已通过 CI 并合入 `main@4e6c490`；紧凑证据位于 [`artifacts/autonomous_auto01_20260729_evidence/`](artifacts/autonomous_auto01_20260729_evidence/)，自主状态已推进到 AUTO-02。以上均为 Docker 中 ROS 2 Jazzy + Gazebo Harmonic 的机器仿真结论，不代表真人、真实车辆或 J6 板端验证。

## AUTO-00 自主控制面通过（2026-07-28）

已依据“全流程自主推进规划包”从远端 `main@ac6d569` 建立 `AUTO-00` 至 `AUTO-16` 的持久状态机和依赖 DAG。新增 registry、状态、运行计划、无人值守 runner、证据 manifest 校验、状态防伪、secret scan 与受保护的 GitHub PR/合并适配器；未配置命令、未执行阶段和外部资源缺口都不会被写成通过。控制面支持断点续跑、已通过证据幂等复用、失败依赖传播和独立 lane 继续推进，入口见 [`docs/autonomous-control-plane.md`](docs/autonomous-control-plane.md)。

AUTO-00 机器门已通过并由 [PR #28](https://github.com/zhexuexiaotudou/TZcup/pull/28) 合入 `main@14dc0ec`：基线 commit 可解析、历史 evidence 未修改、registry/state/plan 均有效、依赖环为 0，真实断点续跑与幂等重跑测试、`ci_fast`、状态防伪、secret scan 和 diff 检查全部通过。紧凑证据位于 [`artifacts/autonomous_auto00_20260728T161119Z_evidence/`](artifacts/autonomous_auto00_20260728T161119Z_evidence/)，并已从远端 main 验证 Git blob 与 git archive 精确字节一致。既有事实边界保持不变：Stage5BR6-A 的两项人工标志为 false；Stage5BR6W 首个阻断层仍是 `no_reachable_clean_route`。下一执行阶段为 AUTO-01，同时 AUTO-04/09/10/11/12/13/14 已进入可调度集合。

## Stage5BR6W 人工门豁免工程支线与 Phase 4 停止边界（2026-07-21）

Stage5BR6W 在不改变真人双盲门的前提下建立了独立工程支线：V4 仅冻结为 engineering verification candidate，新 policy `stage5br6w_v4_engineering_geometry_ready_v1` 明确 `human_validated=false`、`competition_metric_eligible=false`，candidate footprint 由 V4 AABB、现有 production footprint 和 0.03 m 支架裕量自动推导。`camera_profile:=V4_engineering` 与 `footprint_profile:=stage5br6w_v4` 均为 opt-in，默认生产相机和 footprint 未改变；运行时审计确认 local/global costmap、Collision Monitor 与 Coverage 使用同一候选 footprint。

真实 Stage4W seed 0 在 Phase 4 失败并按协议停止：候选 footprint 半径增至 `0.85683 m`，统一几何下 cleanable area 仅 `6.89 m²`，9 条 swath 全部与膨胀 exclusion 相交；正向 staging 落在 operation polygon 外，反向 staging 无有效路径，最终 `no_reachable_clean_route`、完整组件 `0`、经验覆盖率 `0`。因此未继续 seed 1–4、dynamic 20、estop 30 或 Oracle 主动观察，`READY_FOR_STAGE5BR6W_ORACLE_ENGINEERING=false`、`READY_FOR_STAGE5BR7_ENGINEERING=false`。正式人工与 Stage5B 状态全部保持 false；入口见 [`GPT_REVIEW_STAGE5BR6W.md`](GPT_REVIEW_STAGE5BR6W.md) 和 [`artifacts/stage5br6w_20260721_review/`](artifacts/stage5br6w_20260721_review/)。

## Stage5BR6-A 双人盲审交付与人工停止门（2026-07-21）

Stage5BR6-A 已把预注册 V4 的人工审计改造成真正隔离的双包交付：Reviewer A/B 各收到 270 张图片，包含 Stage5BR5 的五类各 40 张正样本，以及训练专用 label=0 几何世界通过真实 V4 Gazebo 相机采集的 70 张 hard-negative/no-target；七类负样本各 10 张，四传感器精确同步，所有负样本裁剪中的 semantic 目标像素为 0，生产世界未修改。两个包使用不同随机顺序、不同 opaque ID，ZIP CRC、逐文件 SHA、ID 集合、PNG 元数据和 truth 泄漏审计均通过；答案映射只保存在 Git 忽略的 `external_review_handoff/stage5br6/sealed_truth/`。

当前没有两份真人完成的 response，故严格设置 `AWAITING_HUMAN_REVIEW=true`、`READY_FOR_STAGE5BR6_ORACLE=false`。V4 仍只是 `pre_registered_camera_candidate`，相机未定型，policy v2 未冻结，production footprint 未修改，Stage4W candidate-footprint 回归、Oracle 主动观察和模型训练均未启动。当前入口见 [`GPT_REVIEW_STAGE5BR6.md`](GPT_REVIEW_STAGE5BR6.md)、[`docs/stage5br6-human-audit.md`](docs/stage5br6-human-audit.md) 与 [`artifacts/stage5br6_20260721_review/`](artifacts/stage5br6_20260721_review/)。

## Stage5BR5 相机机械重构、平衡盲审集与主动观察前置边界（2026-07-20）

Stage5BR5 已修复 ActiveObservation 首见/末见/排队/接近时间混用问题，并完成 V1–V4 相机机械网格；V3 因 trial footprint 冲突被剔除，V1/V2/V4 在六个真实 Gazebo 世界完成 discovery/verification 各 10 帧，共 360 帧精确同步 RGB-D/semantic/instance 证据，运行时自遮挡与 collision/envelope 门通过。最终盲审集达到 200 张、五类各 40 张并覆盖六世界；但尚无两名独立人工评审结果，故 V4 只作为待审候选，相机没有定型。

当前首个阻断层为 `G2_camera_selection_blocked_two_independent_human_manual_reviewers_not_available`。policy v2 保持未冻结、训练禁用；正式 oracle active-observation、detector/area micro-overfit、120/1200、formal/live/J6 均未越级执行。`REVIEW_PACKET_COMPLETE=true`，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。复核入口见 [`GPT_REVIEW_STAGE5BR5.md`](GPT_REVIEW_STAGE5BR5.md)、[`docs/stage5br5-camera-active-observation.md`](docs/stage5br5-camera-active-observation.md) 与 [`artifacts/stage5br5_20260720_review/`](artifacts/stage5br5_20260720_review/)。

## Stage5BR4 可观测性、相机消融与主动观察停止边界（2026-07-20）

Stage5BR4 先冻结可评测策略并重算 Stage5BR3 原始数据：3370 个可见实例中只有 875 个 recognition-ready（`25.96%`），2495 个 non-ready 没有被隐藏。C0–C3 已在同一 world、seed、目标 pose 和轨迹命令下真实采集；C3 verification 虽把小规模 ready 比例提高到 `29.63%`，主动观察 ready 转换仍只有 `2/4 = 50% < 90%`，且车体自身像素 P50 为 `21.11%`。人工审计不通过，相机没有定型。

因此首个阻断层为 `G2_camera_selection_blocked_active_observation_ready_conversion_below_0.90_and_manual_audit_failed`。本轮新增参数化/双相机训练链、三分区可观测性报告与 fail-closed 主动观察状态机，但没有越级启动 detector/area micro-overfit、120/1200 数据扩充、模型 screening、formal、正式 live、真实 active Nav2 或 J6。Stage5BR3 的三次旧失败保持不变；`REVIEW_PACKET_COMPLETE=true`，两个 readiness 仍为 false。复核入口见 [`GPT_REVIEW_STAGE5BR4.md`](GPT_REVIEW_STAGE5BR4.md)、[`docs/stage5br4-active-perception.md`](docs/stage5br4-active-perception.md) 与 [`artifacts/stage5br4_20260720_review/`](artifacts/stage5br4_20260720_review/)。

## Stage5BR3 G2 真实车辆 screening 与停止边界（2026-07-20）

Stage5BR3 已把 G2 从“静态训练 rig/话题名烟测”推进为真实车辆相机链：6 个不同材料、几何与布局的 Gazebo Harmonic 世界按 3/1/2 分为 train/val/test；每个世界均实际收到非空 RGB、32FC1 深度、CameraInfo、semantic/instance GT、精确同时间戳与 `camera_depth_link` 光学帧，生产外参 TF 一致，车辆 2 秒运动约 0.70 m。生产默认 Xacro/launch 与运行时均无 semantic/instance GT，控制侧订阅为 0。

原生 640×480 数据已一次采集 80 scene/800 frame，逐实例 QA 在发现 hard-negative 跨 split 泄漏和缺少 negative-only 后重采并通过：target/negative/trajectory leakage 均为 0，跨 split exact/pHash duplicate 为 0，semantic-instance 错误率为 0，5 个 negative-only 场景，hard-negative 数覆盖 0–8。离线扫描选择 640×384 与 512×384，实际在 512×384 执行 3 次 detector/area-segmenter architecture screening；三次均未全门通过，最佳 cross-world detector F1/AP50/small recall 为 `0.1311/0.3484/0.4512`，最佳颜色压力 F1 `0.1018`，最低 negative-only FP `8.7/帧`，area cross-world mIoU `0.02346`。首个阻断层为 `G2_split_model_screening_gates_failed_after_3_attempts`，因此 500/5000、live、真实 Nav2 与 J6 均未启动，readiness 保持 false。

## Stage5BR2 G2 车载相机基础恢复（2026-07-20）

Stage5BR2 已纠正历史指标语义：G1 的 `cross_asset_world` 只能称为 `cross_asset_same_world`，单世界的真实 `cross_world` 与未隔离的 `cross_material` 均为 `null`；实例尺寸改为按 instance-id 掩码逐实例统计，零像素物体记为 `not_visible`。新增 G2 训练世界从当前车辆 Xacro/launch 提取生产相机契约，使用未放大的真实物理资产，在四种材料上生成 4 个不同 SHA 的 world，并由 Gazebo Harmonic 实际验证 RGB、深度、semantic GT、instance GT 话题全部可启动。GT 仅存在于训练世界，生产 launch 未修改。

该历史轮次尚未执行 G2 的 80 scene/800 frame 采集 QA、四分辨率实测和 detector/area segmenter 筛选，因此当时首个阻断层为 `G2_screening_dataset_80_scene_800_frame_not_executed`，`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。历史复核入口见 [`GPT_REVIEW_STAGE5BR2.md`](GPT_REVIEW_STAGE5BR2.md)、[`docs/stage5br2-g2-vehicle-camera.md`](docs/stage5br2-g2-vehicle-camera.md) 与 [`artifacts/stage5br2_20260720_review/`](artifacts/stage5br2_20260720_review/)。

## Stage5BR Gazebo-camera 数据恢复与模型筛查（2026-07-19）

Stage5BR 已通过训练链自证：12 帧 micro-overfit 的 macro F1/mIoU 为 `0.98124/0.96333`，PyTorch/ONNX 最大 logit 误差 `6.866e-05`、argmax agreement `1.0`。项目新增真实 Gazebo Harmonic 共视场 RGB-D、semantic、instance 数据链，50 个独立 scene / 500 帧 smoke 的标注完整率为 100%，semantic-instance 一致性错误、asset leakage 和跨 split exact/pHash duplicate 均为 0。

三次 G1 模型筛查仍未同时通过 in-domain、同世界跨资产和颜色压力门；最佳尝试为 `0.84511 / 0.65804 / 0.47647`，未达到 `0.90 / 0.70 / 0.60`。因此没有扩成 500 scene/5000 帧正式 G1，也没有执行正式 live 或真实 Nav2 spot-clean。`REVIEW_PACKET_COMPLETE=true`，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。复核入口见 [`GPT_REVIEW_STAGE5BR.md`](GPT_REVIEW_STAGE5BR.md)、[`docs/stage5br-gazebo-camera-recovery.md`](docs/stage5br-gazebo-camera-recovery.md) 与 [`artifacts/stage5br_20260719_review/`](artifacts/stage5br_20260719_review/)。

## Stage5B 学习型感知筛查与停止边界（2026-07-19，Stage5BR 前历史基线）

Stage5B 已新增 30 个自研程序化垃圾资产（五类、每类六变体）、12 个硬负样本、按 scene/asset/texture/world 隔离的数据合同、两种实际梯度训练候选、ONNX Runtime 评测、颜色捷径压力测试、J6 fail-closed 预检和训练模型的真实 Gazebo RGB-D 接入诊断。三次结构性筛查后，最佳候选验证 macro F1 为 `0.38637`，但 100 个未见 scene / 1000 帧测试的离散类 macro P/R/F1 仅为 `0.00752/0.00784/0.00768`，颜色压力 aggregate macro F1 为 `0.05192`，均未过门。

该轮数据生成器还是程序化 D1 renderer，不是真实 Gazebo camera renderer；这是 Stage5BR 推进前的历史边界。Stage5BR 已补齐真实 Gazebo-camera G1 smoke 数据链，但模型 screening 仍失败；D2 真实数据为空，J6 官方工具链/实板不可用。依停止条件，仍未执行 500 seed/5000 帧正式 D1、30 seed/10 分钟正式实时门或 30 次真实 Nav2 spot-clean。因此 `REVIEW_PACKET_COMPLETE=true`，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`、`competition_perception_pass=false`。Stage5B 历史复核入口见 [`GPT_REVIEW_STAGE5B.md`](GPT_REVIEW_STAGE5B.md)、[`docs/stage5b-learned-perception.md`](docs/stage5b-learned-perception.md) 与 [`artifacts/stage5b_20260719_review/`](artifacts/stage5b_20260719_review/)；当前结论以本页顶部 Stage5BR6-A 段落为准。

## Stage4W 可达清扫域与完整任务闭环（2026-07-17）

Stage4W 已修复定位协方差/全局锚点传播、统一任务几何、可达 staging、完整组件执行、动态障碍清除与安全超时证据链。正式 hybrid 10-seed 全部通过，XY RMSE P50/P95/max 为 `0.02825/0.03726/0.03778 m`；静态 5-seed 全部通过，每次均为当前几何生成的 `17/17` 组件，经验覆盖率 `92.93%–94.53%`、覆盖期 RMSE `0.02930–0.04620 m`；动态交互 `20/20` 有效且碰撞 0。keepout、限速区、30 次急停和完整 MCAP 回放均通过，急停 P95 为 `0.188 s`，上游命令停止后 `1.694 s` 达到连续 5 帧稳定零输出。`READY_FOR_GPT_REVIEW_STAGE4W=true`、`READY_FOR_STAGE5A=true`；理论效率仍为 `1053 m²/h < 3500 m²/h`，因此竞赛效率门保持 false。复核入口见 [`GPT_REVIEW_STAGE4W.md`](GPT_REVIEW_STAGE4W.md) 与 [`artifacts/stage4w_20260717_review/`](artifacts/stage4w_20260717_review/)。

## Stage4V 混合定位与完整任务复核（2026-07-16）

Stage4V 已实现 C++ 扫描精化、标准 NavSatFix 仿真、局部/全局融合及 TF 单所有权审计。正式 hybrid 10-seed 全部通过：XY RMSE P50/P95/max 为 `0.03344/0.03792/0.03872 m`，导航、TF 与扫描实际参与均为 10/10，GT 控制违规 0。随后完整 Coverage 在 transit-to-start 失败，经验覆盖率 0%，动态障碍有效交互 0/20；但零碰撞、过滤器、30 次急停（P95 `0.171 s`）与 MCAP 回放通过。因此 `READY_FOR_GPT_REVIEW_STAGE4V=false`、`READY_FOR_STAGE5A=false`，理论效率仍为 `1053 m²/h < 3500 m²/h`。复核入口见 [`GPT_REVIEW_STAGE4V.md`](GPT_REVIEW_STAGE4V.md)。

## Stage4U 坐标标定、定位地图与 5 cm 闭环（2026-07-16）

Stage4U 已修复 map/map_gt 评测语义、`ParticleCloud` 类型/QoS 和地图质量假通过，并完成 M1/M2/M3、AMCL/SLAM Toolbox、LiDAR/AMCL 灵敏度及正式 Oracle 10-seed。最优候选为结构化 v2 surveyed reference 0.02 m + AMCL + 360@10 Hz；10/10 导航成功、TF 全连续、粒子仪器全有效、恢复 0 次，但 map-relative XY RMSE P50/P95/max 为 `0.06767/0.07983/0.08022 m`，未过 0.05 m 硬门。因此 `READY_FOR_GPT_REVIEW_STAGE4U=false`、`READY_FOR_STAGE5A=false`，realistic 与完整 Coverage 按停止条件未执行。复核入口见 [`GPT_REVIEW_STAGE4U.md`](GPT_REVIEW_STAGE4U.md) 与 [`artifacts/stage4u_20260716_review/`](artifacts/stage4u_20260716_review/)；原始 MCAP/posegraph 在用户确认前保留。

## Stage4T 瞬态、EKF 与定位恢复边界（2026-07-15）

Stage4T 已完成 200 组固定时长瞬态、120 组闭环航向、A/B/C/D 各 5 次 EKF 消融、非零 measurement covariance、双分辨率重建图和正式 Oracle 10-seed 定位。0.25/0.35 rad/s 运行包络与闭环航向通过，0.60 rad/s 开环 stress 失败被保留；最终选择 EKF-B。选中 0.05 m 地图后，Oracle 10/10 次导航成功，但 XY RMSE P50/P95/max 为 0.08397/0.14848/0.16972 m，未达到 0.05 m 硬门，因此 `READY_FOR_GPT_REVIEW_STAGE4T=false`、`READY_FOR_STAGE5A=false`，realistic 全量与完整 Coverage 按协议阻断。发布与审计见 [PR #7](https://github.com/zhexuexiaotudou/TZcup/pull/7)、[`GPT_REVIEW_STAGE4T.md`](GPT_REVIEW_STAGE4T.md) 与 [`artifacts/stage4t_20260715_review/`](artifacts/stage4t_20260715_review/)；原始 MCAP 和失败调优 artifact 在用户确认前保留。

本仓库用于构建、验证和交付基于 ROS 2 Jazzy、Gazebo Harmonic、Nav2、SLAM Toolbox、OpenNav Coverage 与 Fields2Cover 的智慧环卫无人清扫车仿真系统。项目强调可复现构建、真实运行证据、阶段门禁和明确的能力边界。

## 当前状态

- Stage 0–5A 及自主 AUTO-00–AUTO-04 已完成 Windows + Docker + NVIDIA GPU 的 headless 构建与运行验证；当前自主状态为 AUTO-05。Stage5BR6W 的 V4 candidate-footprint 失败与 Stage5BR6-A 等待真人 response 均作为独立历史边界保留。
- precision mapping 与 localization/coverage 包络分别限制为 0.30/0.25 和 0.45/0.35 m/s、rad/s；0.60 rad/s stress 默认禁用且仍失败。
- Stage4W hybrid 10-seed 的 XY RMSE P50/P95/max 为 0.02825/0.03726/0.03778 m，定位门禁通过且 GT 控制违规为 0。
- 完整 Coverage 静态 5/5 通过，每次均执行统一几何生成的 17/17 组件；动态障碍 20/20、碰撞 0，过滤器、30 次急停和 rosbag 回放全部通过。
- 本机 Ubuntu 24.04 WSLg 下的 Gazebo/RViz GUI 与基础 ROS topic 验收已完成；Stage5B 训练模型已接入真实 Gazebo RGB-D 链路，但该诊断不构成正式精度门，真实数据训练、J6 量化和实板部署仍未启动。
- 理论清扫效率仍为 1053 m²/h，未达到 3500 m²/h；不得用覆盖率或仿真实测净效率替代竞赛效率口径。
- 详细证据、复现命令和已知边界以 [`docs/progress.md`](docs/progress.md) 为准。

## 快速开始

推荐环境：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。Windows 可通过 Docker Desktop 执行 headless 阶段门禁；需要图形交互时使用原生 Ubuntu 或 Ubuntu 24.04 WSLg。本机 WSLg 路径已于 2026-07-29 完成实机验收。

```bash
export SANITATION_WS=$HOME/sanitation_ws
mkdir -p "$SANITATION_WS/src"
rsync -a starter_ws/src/ "$SANITATION_WS/src/"

bash scripts/bootstrap_jazzy.sh
bash scripts/import_upstream.sh
bash scripts/build_ws.sh
bash scripts/run_baseline.sh
```

Windows 上可按阶段运行 Docker 验收脚本：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage1_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage2_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage3_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4t_core_smoke_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5a_docker.ps1 -OutputName stage5a_formal3 -RecordBag
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5b_docker.ps1 -OutputName stage5b_screening
```

## 开发工作流

本项目所有开发修改统一使用“开发工作流”：核实问题，建立独立分支和 worktree，开发与测试，提交 PR，等待 CI，全绿后合并，部署并检查真实效果，最后用 `neat-freak` 同步文档和 Agent 记忆。汇报后必须等待用户确认，确认前不清理任务分支、worktree 或临时数据。

每个提交都必须复核并同步本文件。`最近同步` 只保留最新状态，不追加提交流水账。CI 会逐个检查 PR 中的提交是否包含 `README.md`。

开始修改前请阅读：

- [`AGENTS.md`](AGENTS.md)：项目级 Agent 规则和强制门禁。
- [`docs/development-workflow.md`](docs/development-workflow.md)：开发工作流的完整定义。
- [`PROJECT_SPEC.md`](PROJECT_SPEC.md)：系统架构、接口和技术约束。
- [`STAGE_GATES.md`](STAGE_GATES.md)：历史 Stage 门与 AUTO-00–AUTO-16 自主阶段的验收条件。
- [`README_FIRST.md`](README_FIRST.md)：环境准备、目录说明和启动细节。

所有修改至少运行快速门禁：

```powershell
py -3 scripts/ci_fast.py
```

Linux 和 CI 使用：

```bash
python scripts/ci_fast.py
```

涉及 ROS 包、launch、URDF/Xacro、SDF、Nav2、SLAM、覆盖规划或运行时行为时，还必须执行受影响的 Stage 门禁。轻量 CI 不能替代真实仿真验收。

## 目录说明

- `starter_ws/src/`：项目自有 ROS 2 包。
- `scripts/`：环境、构建、阶段验收和证据采集脚本。
- `docs/`：进度、开发流程和补充说明。
- `artifacts/`：阶段运行证据；提交前需确认内容可追溯且不含敏感信息。
- `.github/`：PR 模板与“开发工作流”CI。

## 使用边界

- `shumo` 只用于用户明确提出的数学建模问题或数学建模竞赛任务，不用于常规 ROS 2 / Gazebo 工程开发。
- 服务器地址、SSH 私钥、令牌和其他凭据不得提交到仓库，只能放入受保护环境或 GitHub Secrets。
- 不能用“代码已合并”代替“运行时已部署”，也不能用命令返回 0 代替真实节点、话题、日志、JSON、rosbag 或页面效果验收。

## Stage5A 垃圾感知真值闭环、数据集与定点清扫（2026-07-17）

Stage5A 已建立五类垃圾的显式 semantic registry、稳定 UUID、仿真 GT、20-scene RGB-D/COCO 数据、固定契约 ONNX Runtime 后端、2D/3D/map 投影、多帧 tracker 和 deferred spot-clean 状态闭环。正式门包含 ROS 构建/测试、held-out synthetic perception、30-seed 状态闭环、Gazebo 实时 RGB-D 推理、压缩 MCAP，以及 Stage4W 单 seed 完整 Coverage 回归。

当前结论严格限定为 synthetic-domain 工程就绪；`competition_perception_pass=false`、`j6_quantization_pass=false`、`j6_runtime_pass=false`、`competition_efficiency_pass=false`，理论效率仍为 `1053 m²/h < 3500 m²/h`。复核入口为 [`GPT_REVIEW_STAGE5A.md`](GPT_REVIEW_STAGE5A.md)、[`docs/stage5a-garbage-perception.md`](docs/stage5a-garbage-perception.md) 和 `artifacts/stage5a_20260717_review/`。

## 最近同步

2026-07-30：独立 AUTO-13 lane 的采集、隐私、标定、接入和统一评测工具已通过 123 项快速测试。资源发现只找到 1 个本机相机、没有合格真实 GT dataset manifest，因此 `AUTO-13=BLOCKED_EXTERNAL`、`REAL_DOMAIN_PASS=false`；未执行的 1000-frame 正式评测、map RMSE 和 synthetic-to-real drop 保持 null/未执行。主依赖链仍为 `AUTO-04=PASS / 当前 AUTO-05`。
