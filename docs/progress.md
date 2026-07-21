# 项目推进记录

## Stage5BR6W：人工门豁免工程支线（2026-07-21）

状态：完成 Phase 0–3 和 observation planner 加固；真实 Phase 4 seed 0 失败后按协议停止，未进入 Oracle。

已实现独立 engineering waiver，保留 `AWAITING_HUMAN_REVIEW=true` 与全部正式 false 状态；Reviewer A/B 包和 sealed truth 未修改。V4 只冻结为 engineering verification candidate；工程 policy 使用新 ID 与 SHA，明确 `human_validated=false`、`competition_metric_eligible=false`。candidate footprint 由 V4 AABB、production footprint 与 0.03 m 安全裕量推导，并通过双 opt-in profile 接入实际 V4 相机、local/global costmap、Collision Monitor、Coverage mission geometry 和 planner。运行时 footprint audit 全部通过，production 默认未改变。

Observation planner 已增加完整 camera SE(3)、V4 侧向偏置、实际 CameraInfo、整多边形边界/keepout 相交、global costmap footprint cost、位姿相关 target/self overlap、ROI/short-side、路径长度/转向/clearance 代价；工程输入缺失和无可行 pose 均 fail-closed。快速门为 73/73，针对性 planner 测试为 6/6。

真实 Stage4W seed 0 使用 candidate footprint 半径 `0.856825 m`、headland `1.35 m`。运行时 local/global costmap 与 Coverage 加载同一多边形；规划本身成功且计划覆盖率 `0.96226`，但 cleanable area 仅 `6.89 m²`，9 条 swath 全部与膨胀 exclusion 相交。正向 staging 可达却位于 operation polygon 外，反向 staging footprint cost 99 且 `NO_VALID_PATH`，最终 `no_reachable_clean_route`、组件 0、经验覆盖率 0。碰撞/keepout 为 0、刷盘最终关闭、定位 RMSE `0.04333 m`，但不能补偿完整任务失败；Coverage state 未进入 bag，replay=false。

停止边界：static 仅执行 1/5 且 0/1 通过；dynamic 0/20、estop 0/30、Oracle 0 world/0 scene/0 candidate。`READY_FOR_STAGE5BR6W_ORACLE_ENGINEERING=false`、`READY_FOR_STAGE5BR7_ENGINEERING=false`，正式人工与 Stage5B readiness 全部保持 false。紧凑证据见 `artifacts/stage5br6w_20260721_review/`，完整失败日志与 bag 保留在本机 `artifacts/stage5br6w_20260721_runtime/footprint_regression_retry2/static/seed_0/`。

## Stage5BR6-A：双人盲审交付准备

状态：交付包已准备，等待两名独立真人评审；后续门禁未启动。

已完成：

- 将 Stage5BR5 预注册候选固定为 V4，但保持 `camera_selected=false`。
- 保留五类各 40 张的 200 个正样本，并从 Stage5BR6 训练专用 label=0 几何世界通过真实 V4 Gazebo RGB/depth/semantic/instance 精确同步链采集 70 个 no-target/hard-negative；同色非垃圾、瓶/罐形障碍、非积水湿地面、阴影、非目标落叶背景、车辆自身结构和裁剪边界伪影各 10 张，生产世界未修改。
- 每个负样本裁剪均以 semantic mask 检查，目标像素总数为 0。
- 生成 Reviewer A/B 两个独立 ZIP；随机顺序、opaque ID 与 package ID 均不同，无 Git 路径、world、camera 或 truth 映射泄漏。
- ZIP CRC、逐文件 SHA、PNG 元数据、response 模板空值和 sample ID 集合校验通过。
- 提供真人回收完整性校验脚本；脚本不会生成、补全或修正人工回答。

当前边界：

- `AWAITING_HUMAN_REVIEW=true`，真人 response 为 `0/2`。
- `READY_FOR_STAGE5BR6_ORACLE=false`、`READY_FOR_GPT_REVIEW_STAGE5BR6=false`、`READY_FOR_STAGE5BR7=false`。
- V4 相机契约与 policy v2 未冻结；production footprint 未修改。
- candidate-footprint Stage4W 回归、Oracle 主动观察、detector/area model 训练和 J6 均未执行。

证据：

- `artifacts/stage5br6_20260721_review/stage5br6_status.json`
- `artifacts/stage5br6_20260721_review/human_handoff_manifest_redacted.json`
- `artifacts/stage5br6_20260721_review/human_handoff_integrity_report.json`
- 本机 Git 忽略交付目录：`external_review_handoff/stage5br6/`

## Stage5BR5：相机机械重构、平衡盲审与主动观察基础（2026-07-20）

ActiveObservation 已把 `first_seen_s`、`last_seen_s`、排队、preflight、approach、动态 deadline 与最近 observation 时间分开；重复 discovery 刷新末见时间，sensor stale 和 queue timeout 独立，空间合并允许模型 ID 变化，旧记录可迁移。几何 planner 以 cleanable/keepout、footprint clearance、协方差、预期像素/ROI、自遮挡、视角、路径长度和转向代价选择候选；ROS 2 wrapper 实际调用 `/compute_path_to_pose` 且不使用 GT 输出位姿。

机械网格中 V1/V2/V4 通过，V3 因旋转相机 AABB 超出 trial footprint 剔除。V1/V2/V4 各在 6 world × 2 role × 10 frame 完成真实 Gazebo 采集，共 360 帧，精确四传感器时间戳、自像素 P95、target/self overlap 与物理门通过。v2 ready fraction 为 `0.13450/0.13636/0.30508`；这些只是 view-level 结果，不是主动观察闭环转换。

盲审数据经多轮固定 seed 补样后达到 200 张、五类各 40 张并覆盖六世界。当前没有两名独立人工评审，manual accuracy、Cohen kappa 与 self-occlusion failure 均为 null，相机没有选择，policy v2 为未冻结且 training disabled 的草案。首个阻断层为 `G2_camera_selection_blocked_two_independent_human_manual_reviewers_not_available`。正式 oracle active-observation、detector/area micro-overfit、120/1200、formal/live/J6 按门禁未执行；Stage5BR4 和更早结论未改写。

回归已重新执行：`ci_fast` 68/68；`sanitation_learning` 与 `sanitation_spot_cleaning` colcon build 通过、29 tests/0 failures；Stage5A 为 30/30 spot-clean、119 帧 live、GT control violation 0 且 rosbag 已录制；Stage4W seed0 为 17/17 组件、经验覆盖率 `0.944`、定位 RMSE `0.03737 m`、碰撞/keepout/brush violation 0 且 replay 通过；生产默认运行时 GT 隔离通过。

## Stage5BR4：可观测性、相机消融与主动观察（2026-07-20）

状态：复核材料完整，Stage5B/Stage5C readiness 均为 false。C0 原始 3370 可见实例只有 875 个满足冻结的 recognition-ready；C0–C3 五段真实采集均完成 10/10 同步帧。C3 verification 的 ready 比例为 29.63%，但 discovery non-ready 到 verification ready 的实例转换仅 50%，低于 90% 门，且 self-pixel P50 为 21.11%。人工可辨识审计失败，首个阻断层为 `G2_camera_selection_blocked_active_observation_ready_conversion_below_0.90_and_manual_audit_failed`。

已实现相机 mount 参数化、默认关闭的物理 C3 verification RGB-D/GT、冻结策略哈希、all-visible/ready/non-ready 报告，以及包含去重、路径/keepout/footprint/visibility preflight、stale/timeout/最大接近和代价记录的主动观察状态机。生产默认隔离通过。因相机选择失败，真正 detector、area 模型、micro-overfit、120/1200、screening、formal、正式 live、真实 active Nav2 和 J6 均未执行；Stage5BR3 三次旧模型结果未改写。

## Stage5BR3：真实车辆 G2 数据、逐实例 QA 与 split-model screening

Stage5BR3 将 `artifacts/stage5br2_*_review/**` 改为 binary，避免 Git blob 与 Windows 导出包发生 LF/CRLF 证据字节漂移；同时废弃独立静态相机 rig，训练 GT 传感器只在显式 `enable_training_gt:=true` 时挂到生产车辆 `camera_link`，生产默认渲染和运行时均无 semantic/instance GT。

最终 G2 有 6 个不同 SHA、材料和几何布局的世界，按 3 train / 1 val / 2 test 隔离。六世界真实消息门全部通过：640×480 RGB/depth/semantic/instance 非空、CameraInfo 有效、四传感器精确同时间戳、光学帧统一为 `camera_depth_link`、深度为 32FC1 且有限值处于 0.3–100 m、base→camera 外参为 `[0.53, 0, 0.22] m`，实际车辆 2 秒移动约 0.70 m。

原生数据一次采集 80 scene/800 frame、约 2.225 GB。第一次 QA 因 12 个 hard-negative 资产跨 split 复用且 negative-only 场景数为 0 而失败；将 hard negatives 固定拆为 8/2/2 并强制 5 个 negative-only 种子后重采。第二次 QA 为 80/800、标注完整率 100%、target/negative/trajectory leakage 0、跨 split exact/pHash duplicate 0、semantic-instance 错误率 0，hard-negative 数覆盖 0–8，最终通过。

四档离线扫描选择 640×384 与 512×384；实际在 512×384 执行 3 次 split-model 尝试。最佳 detector cross-world F1/AP50/AP50:95/small recall 为 `0.1311/0.3484/0.1075/0.4512`，最佳颜色压力 F1 `0.1018`，最低 negative-only FP `8.7/帧`；area cross-world mIoU `0.02346`。未达到 screening 门，故停在 `G2_split_model_screening_gates_failed_after_3_attempts`。没有执行 500/5000、live、真实 Nav2、真实域、J6 或竞赛效率门；`1053 m²/h < 3500 m²/h` 不变。

回归方面，`ci_fast.py` 68 项、`sanitation_learning` 11 项和三包 colcon build/test 通过；Stage5A 为 30/30 spot-clean、132 帧 live 且 GT control violation 0；Stage4W seed 0 完成 17/17 组件、覆盖率 `0.93533`、定位 RMSE `0.03572 m`、碰撞/keepout/brush violation 均为 0。完整运行日志留在本地，紧凑机器摘要为 `artifacts/stage5br3_20260720_review/stage5br3_regression_summary.json`。

## Stage5BR2：G2 车载相机基础恢复与 fail-closed 边界

- 从当前车辆 Xacro 提取 `camera_link` 相对 `base_link` 外参 `[0.53, 0, 0.22] m`、`camera_depth_link`、640×480、水平 FOV `1.50098 rad`、15 Hz，并校验 `sim.launch.py` 的生产 ROS 话题映射。
- 建立四个 G2 世界与 2/1/1 train/val/test world-isolated split；4 个 world SHA 和 4 种材料均不同，资产为项目自制 Apache-2.0 程序化几何、scale 1.0。
- Gazebo Harmonic 逐世界实际启动通过，RGB、深度、semantic GT、instance GT 四类话题齐全；GT 为 training-only，生产 launch 未修改。
- 修正指标语义：历史 G1 `cross_asset_world` 规范化为 `cross_asset_same_world`，单世界 `cross_world=null`；新增逐 instance-id bbox、最短边、mask area、距离、遮挡和 `not_visible` 统计。
- ROS-independent 快速门通过：68 tests。当前尚未采集 G2 80 scene/800 frame，故分辨率实测、detector/area segmenter、500/5000、live、真实 Nav2 和 J6 均未执行。
- 首个阻断层：`G2_screening_dataset_80_scene_800_frame_not_executed`；`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。
- 证据：`GPT_REVIEW_STAGE5BR2.md`、`artifacts/stage5br2_20260720_review/`、`docs/stage5br2-g2-vehicle-camera.md`。

## Stage5BR：Gazebo-camera 数据恢复、训练链审计与泛化修复

状态：G1 数据 smoke 通过，学习模型 screening 失败并按停止条件冻结。

已完成：

- 12 帧 micro-overfit 达到 macro F1 `0.98124`、foreground mIoU `0.96333`。
- PyTorch/ONNX/ROS parity 达到最大 logit error `6.866e-05`、argmax agreement `1.0`。
- 新增 Gazebo Label system、共视场 RGB-D/semantic/instance cameras、scene/lighting 随机化和 exact timestamp collector。
- G1 50 scene/500 frame：annotation completeness `1.0`、label consistency error `0`、asset leakage `0`、跨 split exact/pHash duplicate `0/0`。
- 三次 G1 model screening 均失败；最佳 cross asset/world F1 为 `0.65804`，最佳 color stress F1 为 `0.47647`。
- Stage5A 回归通过：30/30 synthetic spot-clean，live 186 帧、MCAP true、GT control violation 0。
- Stage4W seed 0 回归通过：17/17、coverage `0.936`、RMSE `0.03260 m`、零碰撞/keepout/brush violation。

停止边界：

- 不生成 500 scene/5000 frame formal G1；
- 不运行 30 seed/10 min formal live；
- 不运行真实 Nav2 spot-clean；
- R1、J6 实板、竞赛感知和 `1053 < 3500 m²/h` 效率门保持 false。

证据：`GPT_REVIEW_STAGE5BR.md`、`docs/stage5br-gazebo-camera-recovery.md`、`artifacts/stage5br_20260719_review/`。

## Stage5B：学习型感知、域隔离与颜色捷径筛查

状态：已形成可复核的失败边界，未通过 Stage5B，未进入 Stage5C。紧凑证据包完整，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。

已完成：

- 新增 `sanitation_learning`，含五类各六变体、12 个硬负样本、许可清单、scene/asset/texture/world 隔离、RGB-D/semantic/instance/map-pose/COCO 生成、标注 QA、训练、ONNX 评测、颜色压力和 J6 预检。
- 候选 A 为已训练 1×1 Conv 基线；候选 B 为 137,078 参数、6 Conv + 5 ReLU 的上下文模型；候选 C 因 ONNX/J6 算子风险明确 deferred。选择未使用测试集。
- 三次结构性筛查后冻结：最佳验证 macro F1 `0.38637`，100 个未见 scene / 1000 帧离散 macro P/R/F1 `0.00752/0.00784/0.00768`，leaf/puddle IoU `0.00376/0.2494`，颜色压力 aggregate macro F1 `0.05192`；map RMSE `0.09731 m` 是唯一主要精度通过项。
- 修正评测命名：无置信度排序 PR 曲线时，`ap50`/`ap50_95` 为 null；实际 IoU 匹配分数使用独立字段，禁止冒充 AP。
- 训练模型真实接入 Gazebo RGB-D/TF/ONNX Runtime 链，处理 161 帧并发布分割与 map targets，且 `ground_truth_input_used=false`；该运行只作为接口诊断，正式 30 seed/10 分钟门为 false。
- 回归通过：`py scripts/ci_fast.py` 为 57 passed；Stage5A 固定颜色离线/30 次状态评测/实时 Gazebo 通过；Stage4W seed 0 为 17/17、经验覆盖率 94.2%、碰撞/keepout/刷盘违规 0。

停止边界：

- 当前 D1 数据是程序化 renderer，不是 Gazebo camera 实际渲染；500 seed/5000 帧正式集未执行。
- 颜色捷径和未见泛化失败后，按规划包停止条件不执行 30-seed 正式实时门与 30 次真实 Nav2 spot-clean，避免用运行可达性替代精度。
- D2 无授权真实数据；J6 官方工具链、转换/量化和实板 FPS 均无证据；理论效率 `1053 m²/h < 3500 m²/h`。

复核入口：`GPT_REVIEW_STAGE5B.md`、`docs/stage5b-learned-perception.md` 与 `artifacts/stage5b_20260719_review/`。原始三次筛查、Docker workspace、数据卷与 rosbag 在用户确认前保留本机。

## Stage5A：垃圾感知真值闭环、数据集与定点清扫

状态：正式实现已覆盖 registry、GT、20-scene 数据、ONNX Runtime、RGB-D 到 map、多帧 tracker、30-seed synthetic task-state E2E 和 Stage4W 回归。紧凑复核目录的 9 个机器 gate 全部通过，`READY_FOR_GPT_REVIEW_STAGE5A=true`、`READY_FOR_STAGE5B=true`。

已验证边界：Stage5A 仅为 synthetic-domain 工程证据。30-seed 状态闭环不等于 30 次真实车辆/Nav2 定点任务；J6 工具链/量化/运行、真实数据精度、原生 GUI、实车、机械臂与竞赛效率仍未通过。详细复现与结果见 `docs/stage5a-garbage-perception.md` 和 `GPT_REVIEW_STAGE5A.md`。

## Stage4W：可达清扫域、完整覆盖与动态交互闭环

状态：Stage4W 正式门禁全部通过并已作为 Stage5A 回归基线；当前阶段状态见上方 Stage5A 条目。

已完成：

- 修复 GNSS 协方差建模、refined/GNSS 有界权重和全局锚点随局部里程计传播；正式 hybrid 10-seed 为 10/10，XY RMSE P50/P95/max `0.02825/0.03726/0.03778 m`，导航、TF 单所有者、扫描精化参与均为 10/10，GT 控制违规 0。
- 建立唯一 mission geometry：outer、headland、keepout、显式 exclusion、world→map 固定障碍、footprint 和安全裕量共同编译。当前生成 9 swath + 8 turn = 17 组件；Stage4V 的固定 23 组件来自旧几何，Stage4W 标记为不适用。
- 同时预规划正/反 staging，等待全局 costmap 覆盖候选点，核对 cost/keepout/speed mask 与 footprint clearance，再以明确 approach yaw 执行 transit 和 brush-off 稠密 entry。
- 为 NavigateToPose、ComputePathToPose 和 FollowPath 使用各自动作错误码语义；FollowPath 104 被正确识别为 `PATIENCE_EXCEEDED`。Nav2 使用 `PoseProgressChecker`，controller 有界容忍 5 s。
- 动态障碍通过持久 ROS–Gazebo SetEntityPose 服务桥横穿；同一组件注入间距至少 0.5 m。局部/全局 obstacle layer 启用无限量程清障，消除障碍移走后的旧标记。
- 正式静态 5-seed 全部通过：每次 17/17、经验覆盖率 `92.93%–94.53%`、覆盖期 RMSE `0.02930–0.04620 m`、碰撞/keepout/刷盘违规均为 0、刷盘最终关闭、回放 5/5。
- 正式动态任务通过：20/20 有效交互、碰撞 0，完整任务 17/17、覆盖率 93.53%、覆盖期 RMSE 0.03014 m；keepout 违规 0、限速区平均 0.288 m/s。
- 30 次急停全部归零与释放恢复，P95 `0.188 s`；停止上游命令后 `1.694 s` 达到连续 5 帧稳定零输出。动态 MCAP 完整回放通过。

边界：

- `READY_FOR_GPT_REVIEW_STAGE4W=true`、`READY_FOR_STAGE5A=true` 只表示 Stage4W 技术门满足；Stage5A 已在后续独立阶段实施并保留新的合成域边界。
- 竞赛理论效率仍为 `1053 m²/h < 3500 m²/h`，`competition_efficiency_pass=false`；不得以经验覆盖率替换效率门。
- 垃圾感知训练、J6 量化、实板部署和原生 Ubuntu/WSLg GUI 验收未执行。
- 紧凑证据位于 `artifacts/stage4w_20260717_review/`；原始 MCAP、筛查和失败诊断在用户确认前保留本机。

复核入口：`GPT_REVIEW_STAGE4W.md`、`artifacts/stage4w_20260717_review/stage4w_summary.json` 与 `MANIFEST.json`。

## Stage4V：混合定位与完整任务复核

状态：正式混合定位门禁通过，完整 Coverage 门禁失败；未进入 Stage5A。

新增 `sanitation_scan_refiner`、`sanitation_gnss_sim`、混合全局融合器和 TF 所有权审计。正式 10-seed 的 XY RMSE P50/P95/max 为 `0.033438/0.037916/0.038717 m`；定位、导航、TF 单所有者及扫描参与均为 10/10，GT 控制违规 0。完整任务随后真实运行：规划覆盖率 97.5%，但 transit-to-start 超时/终止，完整执行 false、经验覆盖率 0%；动态障碍有效交互 0/20、碰撞 0；keepout 违规 0、速度区通过；30 次急停 P95 `0.1705 s`；MCAP 融合位姿回放通过。理论效率 `1053 m²/h` 未达 `3500 m²/h`。最终 `READY_FOR_GPT_REVIEW_STAGE4V=false`、`READY_FOR_STAGE5A=false`。

证据入口：`GPT_REVIEW_STAGE4V.md`、`artifacts/stage4v_20260716_review/`；原始 10-seed、Coverage 和 MCAP 在用户确认前保留。

## Stage4U：坐标标定、定位地图与 5 cm 定位闭环

状态：达到可复核失败边界；未通过 Stage4U，未进入 Stage5A。

已完成：显式冻结 SE(2) 坐标标定；map-relative/地理配准/absolute 误差解耦；Jazzy `nav2_msgs/msg/ParticleCloud` 与 best-effort QoS 修复；加权粒子统计；地图生成/基础质量/定位几何三级门；M1/M2/M3 与 AMCL/SLAM Toolbox 对照；360@10 与 720@20、AMCL profile 灵敏度；结构化 v2 场景；正式串行 Oracle 10-seed。

正式最优候选为结构化 v2、0.02 m surveyed reference、AMCL 精度 profile、360@10 Hz。10/10 seed 完整，10/10 导航成功，TF 全连续，粒子仪器全有效，恢复 0 次；XY RMSE P50/P95/max 为 `0.067669/0.079833/0.080218 m`，worst 为 seed 7。首个真实失败层仍是 `oracle_localization_pass`。

边界：M2 posegraph 已序列化，但没有执行独立离线优化/重渲染；M3 是定位参考图，不是 SLAM 建图成绩；realistic、完整 Coverage、动态障碍和急停按停止条件未执行；理论效率仍为 `1053 m²/h`，`READY_FOR_GPT_REVIEW_STAGE4U=false`、`READY_FOR_STAGE5A=false`。

复核入口：`GPT_REVIEW_STAGE4U.md`、`artifacts/stage4u_20260716_review/stage4u_summary.json`、`oracle_10seed_compact.json` 与 `MANIFEST.json`。

发布与合并后验证：[PR #9](https://github.com/zhexuexiaotudou/TZcup/pull/9) 的 `fast-validation` 通过，已按 merge-commit 策略合入 `main@efd5e34cbb3c8ba1016118c63a6e35402704e787`。远端 main tree `00f2b33c5866025421bc5e9bea224945b58eafbd` 与本地验证树一致；合并树真实 Gazebo core smoke 再验通过 covariance 与 operational envelope，MCAP 17.5 MiB、48,255 条消息且元数据可读。回滚点为 `de5106cdaf0948888c0225a1076cad790280efa3`。

## Stage4T：转向瞬态、EKF 融合与定位恢复

状态：到达可复核失败边界；未通过 Stage4T，未进入 Stage5A。

已完成：

- 固定时长瞬态 `200/200`、闭环航向 `120/120`；实际 `/cmd_vel` 积分、完整逐 trial 指标和重复性均保留，GT 控制违规为 0。
- precision/coverage 运行包络真实输出越界为 0；0.60 rad/s stress 失败原样保留且默认禁用。
- 原始全零 covariance topic 保留；项目 measurement adapter 发布非零 YAML 化 wheel/IMU covariance，真实 core smoke 通过。
- A/B/C/D 各 5 次同动作集消融完成，选择 EKF-B；可选 chassis yaw-rate controller 记录为 `not_needed`。
- 0.05/0.02 m 地图均以 selected EKF 自动闭环路线重建，0.05 m 质量门通过并选中；SDF 刚体配准几何指标、overlay、keepout/speed masks 和建图 MCAP 均保留。
- Oracle 正式 10-seed 达到 10/10 导航成功、TF 全连续、粒子退化 0，但 XY RMSE P50/P95/max 为 `0.08397/0.14848/0.16972 m`，超过 `0.05 m` 硬门。

当前边界：

- 第一真实失败层为 `oracle_localization_pass`，根因指向 SLAM 地图的非刚性几何误差与稀疏场景 AMCL 匹配精度。
- 按 Stage4T 停止条件，没有执行 realistic 全量 10-seed、完整 Coverage、20-seed 动态障碍、30 次急停或完整任务 rosbag replay。
- `READY_FOR_GPT_REVIEW_STAGE4T=false`，`READY_FOR_STAGE5A=false`；`competition_efficiency_pass=false`，理论效率仍为 `1053 m²/h`。

复核入口：

- `GPT_REVIEW_STAGE4T.md`
- `artifacts/stage4t_20260715_review/stage4t_summary.json`
- `artifacts/stage4t_20260715_review/MANIFEST.json`

发布与合并后验证：

- [PR #7](https://github.com/zhexuexiaotudou/TZcup/pull/7) 的最新 `fast-validation` 已通过，随后按仓库 merge-commit 策略合入 `main@2412300192d6f4204e0049e55c06ba69353377ba`；回滚点为 `b7734801d775740dccf6ce16a12f6e739b2e8136`。
- 远端 main tree `cc9698b3167b37999592613db73f3e08af79cbcc` 已在独立部署副本中执行真实 Gazebo core smoke：covariance 与运行包络均通过，实际速度越界为 0，MCAP 为 17.9 MiB、49,437 条消息且元数据可读。
- 合并后 core smoke 不改变 Stage4T 停止结论：0.60 rad/s stress 仍失败，完整瞬态/EKF/地图/Oracle 证据继续以复核目录为准。

## Stage4S：运动模型标定与定位闭环

状态：已到达可复核失败边界，未通过 Stage4S，未进入 Stage5A。

已完成：

- 新增模型级 Gazebo `OdometryPublisher` 真值源 `/ground_truth/model_odom_raw`，严格校验 `world` 与 `sanitation_vehicle/base_footprint`，移除生产路径对匿名 `Pose_V.transforms[0]` 的依赖。
- 通过出生点、静止 20 s、前进 1 m、正负 90°、world→map_gt 变换和实体稳定性自证。
- 建立使用仿真时钟、无障碍专用世界的 13 段开环实验台，并记录命令、关节、raw odom、IMU、EKF、真值、TF、段标记和完整 MCAP。
- 解耦 physical 与 DiffDrive 参数，完成轮半径 5 点、轮距 9 点粗细网格；选择 `drive_wheel_radius=0.14 m`、`drive_wheel_separation=1.22 m`。
- 完成 5 点摩擦/WheelSlip 最小网格。降低横向摩擦或启用 WheelSlip 均显著恶化高速转向，默认接触为网格最优。

当前边界：

- 首个失败层为 `layer_1_body_command_tracking`。
- 5 m 直线、低速正反整圈和四个圆弧半径通过；高速 `0.60 rad/s` 正转整圈车体 yaw 误差为 `19.1825°`，门槛为 `≤18°`。
- raw wheel odom 与 IMU 初步门槛通过，但不能跳过 Layer 1 直接做 EKF 消融。
- Stage4S-5 至 Stage4S-9 未执行；`READY_FOR_GPT_REVIEW_STAGE4S=false`、`READY_FOR_STAGE5A=false`。
- 垃圾感知训练、J6 量化和实板部署均未开始。

复核入口：

- `GPT_REVIEW_STAGE4S.md`
- `artifacts/stage4s_20260715_review/stage4s_summary.json`
- `artifacts/stage4s_20260715_review/manifest.sha256`

## Stage 0：预检与基线锁定

状态：已通过（容器 headless 预检门）。

已完成：

- 将用户提供的完整推进包扁平导入仓库根目录；原工作区为空，没有覆盖既有成果。
- 校验 `MANIFEST.json` 的 35 个条目均存在且字节数一致。
- 初始化 Git，并以独立基线提交保存原始推进包。
- 完成 Windows 宿主、GPU、磁盘、WSL、Docker 与本机 ROS 工具 inventory。
- 实时核查并锁定 Linorobot2、OpenNav Coverage 和 Fields2Cover 上游版本。
- 修复预检脚本，使其输出结构化检查、阻塞项、告警、命令路径与原始探针结果。
- 修复第三方导入策略：使用精确 commit、拒绝覆盖 dirty checkout、校验最终 SHA。
- 构建 `tzcup/sanitation-jazzy:stage0` 验证镜像并执行预检；脚本返回 0。
- 验证 Ubuntu 24.04.4、ROS 2 Jazzy、Gazebo Sim 8.11.0、`ros_gz`、colcon、rosdep、vcs 全部可用。
- 验证 `ros-jazzy-fields2cover` 2.0.0 二进制包可安装。

当前边界：

- Windows 宿主不满足直接运行 ROS 2 Jazzy/Gazebo Harmonic 的要求。
- Docker 可作为 Ubuntu 24.04/Jazzy headless 构建通道；GUI 与动力学证据仍需 Ubuntu 24.04 原生或 WSLg。

证据：

- `artifacts/preflight.json`
- `artifacts/stage0_20260714_*/preflight.json`
- `artifacts/stage0_20260714_*/preflight_run.log`
- `artifacts/stage0_20260714_*/host_inventory.json`

复现命令：

```powershell
docker desktop start
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_docker_preflight.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\collect_stage0_evidence.ps1
```

## Stage 1：工作空间可重复构建

状态：已通过。

已完成：

- 在全新隔离工作空间中导入 starter 包、Linorobot2 和 OpenNav Coverage。
- 完成 rosdep 安装；`micro_ros_agent` 仅用于真实硬件路径且 Jazzy rosdep 无对应键，因此在仿真构建中显式跳过。
- 连续执行两次 `colcon build --symlink-install` 和两次测试。
- 增加 `sanitation_tasks` 的项目自有 pytest，验证冒烟检查所需的运动、传感器、相机与 TF topic 集合。
- 上游 `linorobot2_gazebo` 没有 pytest 用例（pytest code 5），因此从测试 lane 明确排除；上游 CMake `xmllint` 依赖在线 ROS schema，改由离线 XML well-formedness 检查覆盖。其余上游 lint、GTest 和项目测试均执行。
- 两次测试结果均为 275 tests、0 errors、0 failures、44 skipped；跳过项来自 cppcheck 对当前 2.13.0 慢版本的上游保护逻辑。
- 构建前后第三方仓库 SHA 一致且 `dirty_files=0`。

证据：

- `artifacts/stage1_20260714_154523/stage1_summary.json`
- `artifacts/stage1_20260714_154523/build_1.log`
- `artifacts/stage1_20260714_154523/build_2.log`
- `artifacts/stage1_20260714_154523/test_results.txt`
- `artifacts/stage1_20260714_154523/third_party_status_after.txt`

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage1_docker.ps1
```

## Stage 2：车辆 URDF、场景与运行闭环

状态：headless GPU 验收已通过；GUI 截图仍需原生 Ubuntu 24.04 或 WSLg 复核。

已完成：

- 重写本项目 `sim.launch.py`，在 Jazzy 上以字符串参数加载 `robot_description`，组合 Gazebo server、可选 GUI、实体生成、ROS-Gazebo bridge、EKF 与命令超时保护。
- 建立参数化 4WD 清扫车：0.65 m 清扫 footprint、40 L 尘箱、四轮、双刷、LiDAR、RGB-D、IMU 与 `arm_mount_link`。
- 移除上游模型级重复 Sensors system，消除同一场景被创建两次导致的 Ogre2 重复材质和崩溃。
- 使用 Gazebo Harmonic Ogre2 headless rendering 和 Docker NVIDIA GPU passthrough 实际运行仿真。
- 静态验证 URDF、由 URDF 转换的 SDF 和场景 SDF。
- 新增运行探针，订阅时钟、TF、双路里程计、关节、IMU、LiDAR、RGB、深度和点云，并发送 5 秒速度指令验证实际动力学位移。
- Stage 2 实测 12/12 类话题均有消息；车辆位移 1.18725 m，阈值 0.01 m；仿真在证据采集期间保持存活。
- 给 launch 清理增加有上限的 INT/TERM/KILL 阶梯，避免 Gazebo 子进程造成 CI 假卡死。

证据：

- `artifacts/stage2_20260714_163402/stage2_summary.json`
- `artifacts/stage2_20260714_163402/runtime_probe.json`
- `artifacts/stage2_20260714_163402/simulation.log`
- `artifacts/stage2_20260714_163402/nodes.txt`
- `artifacts/stage2_20260714_163402/topics.txt`
- `artifacts/stage2_20260714_163402/gz_topics.txt`

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage2_docker.ps1
```

## Stage 3：SLAM、定位、Nav2 与安全

状态：运行门已通过；定位精度仍是进入 Stage 4 前必须显式携带的风险。

已完成：

- 新增 `sanitation_navigation`，提供 SLAM Toolbox、地图保存、AMCL/Nav2、Regulated Pure Pursuit、车辆 footprint、keepout filter 与 speed filter 配置。
- 解决 Gazebo LiDAR 作用域帧与 URDF `laser` 帧不一致的问题，SLAM 能持续消费真实 `/scan`。
- 实际生成并保存 194×64、0.05 m/px 的 SLAM 地图。
- 新增 `sanitation_safety` 高优先级速度门：Nav2 统一输出到 `/cmd_vel_nav`，仅速度门可向车辆发布 `/cmd_vel`。
- 实际执行 10 点 `NavigateThroughPoses`，action 状态为 `SUCCEEDED`，并记录 node/topic/action/service、TF、AMCL、里程计与 rosbag。
- 隔离验证急停：正常指令放行、急停归零、释放后恢复、上游失联 0.5 秒后归零全部通过。
- 构建与新增测试通过；导航包 lint、XML 和 3 个速度门单元测试均通过。

证据与边界：

- `artifacts/stage3_20260714_172155/stage3_summary.json`
- `artifacts/stage3_20260714_172155/navigation_probe.json`
- `artifacts/stage3_20260714_172155/safety_probe.json`
- `artifacts/stage3_20260714_172155/slam_map.yaml`
- `artifacts/stage3_20260714_172155/navigation_bag/metadata.yaml`
- action 虽成功，但终点 AMCL 与里程计平面距离相差 1.806 m，且 controller 日志出现 2 次 progress failure；该结果只能证明导航闭环可运行，不能证明定位精度达标。

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage3_docker.ps1
```

## Stage 4：覆盖规划、指标与受控执行交接

状态：评审门已通过；项目按主提示词停在 Stage 4，不进入感知训练或 J6 量化。

已完成：

- 新增 `sanitation_coverage`，集成 OpenNav Coverage 与 Fields2Cover，使用 Boustrophedon 路由、Dubins 转弯和 0.65 m 作业宽度。
- 对 16 m × 8 m、128 m² 示例区域生成 12 条作业带、11 个转弯和 2140 个稠密 Nav2 路径点；总路径长度 213.494 m。
- 以 0.10 m 栅格审计计划覆盖：覆盖 124.80 m²，覆盖率 97.5%，漏扫率 2.5%，重复率 2.492%。这些是规划几何指标，不是实车经验覆盖率。
- 发现并兼容 OpenNav `PathComponents` 中退化的 swath end point；兼容层只用相邻 turn 首点及最终路径点重建端点，原始与修复后数据均写入证据。
- 根据 AMCL 当前位姿选择完整覆盖路径的最近点，从 2140 点计划中截取 180 点执行窗交给 Nav2；action 被接受并持续执行，20 秒内里程计位移 7.393 m，随后主动取消。
- 清扫刷在执行窗内开启、退出时关闭；完整路径的作业带/转弯刷控计划记录为 12 个开启段和 11 个关闭段。
- 记录 coverage server、Nav2、Gazebo、node/topic/action/service、rosbag、完整路径 JSON 与指标 JSON；Stage 4 新增测试 3/3 通过，累计 293 tests、0 errors、0 failures、44 skipped。

证据与边界：

- `artifacts/stage4_20260714_174914/stage4_summary.json`
- `artifacts/stage4_20260714_174914/coverage_metrics.json`
- `artifacts/stage4_20260714_174914/coverage_path.json`
- `artifacts/stage4_20260714_174914/coverage_bag/metadata.yaml`
- 受 Stage 3 终点定位差 1.806 m 影响，只执行与取消局部路径窗以验证接口和物理运动；97.5% 覆盖率不能解释为完整覆盖任务已经实跑完成。
- 当前主机没有 Ubuntu 24.04/WSLg 图形环境，因此没有伪造 Gazebo/RViz GUI 截图；headless Ogre2、ROS 图谱、JSON 与 rosbag 是本轮可复核证据。

复现命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4_docker.ps1
```

评审边界：优先修正定位一致性并完整回放覆盖任务；是否进入感知与 J6 阶段由人工评审后另行决定。
