# AUTO-03 Oracle 主动观察闭环

## 结论

AUTO-03 的最终六世界正式结果由
`artifacts/autonomous_auto03_20260729_evidence/auto03_acceptance_report.json`
自动生成；本页中的最终指标只从该报告同步，不从 README 反推。六世界运行与逐世界
MCAP 重放均已完成，核心汇总的全部机器硬门通过；紧凑证据已从实现提交
`731d069abe1384ee427cf2909c8b9605e70911b0` 生成并通过逐文件 manifest 校验，
自主状态已推进到 AUTO-04。

发布闭环已完成：[PR #35](https://github.com/zhexuexiaotudou/TZcup/pull/35) 的
`fast-validation` 通过后 squash 合入 `main@c49122113583cf17015989740239128f6341ec41`，
主分支 CI run `30488608555` 通过；远端 main 中 92 个 manifest 管理文件与对应
Git blob 逐字节一致。该阶段没有常驻服务，部署门为 `not_applicable`，回滚点为
`82c85c0`。

## 目标与真值边界

本阶段只验证“已有候选后，车辆能否安全地主动靠近并获得机器可判定图像”，不把
Oracle 成绩解释为学习感知成绩。唯一读取场景真值的
`auto03_oracle_scene_source` 只能向执行侧发布：

- 候选 ID 和通用类别；
- 带噪声的平面 XY；
- 协方差、时间戳和通用目标尺度；
- 假候选与失联候选。

它不能输出观察位姿、路径或成功状态，不能设置车辆位姿。`auto03_machine_ready_evaluator`
是唯一订阅语义 GT 的评测节点；节点图审计会拒绝 planner、Nav2、控制节点或
`auto03_observation_executive` 对 GT topic 的任何订阅。production 默认仍为
`enable_training_gt:=false`，GT bridge 和自车标签只在本阶段的 opt-in 评测启动中出现。

## 实际任务链

每条候选都真实执行以下链路：

```text
Coverage component boundary
→ queue / fail-closed preflight
→ observation-pose sampling
→ ComputePathToPose
→ NavigateToPose
→ synchronized RGB + evaluation-only semantic capture
→ machine-ready evaluation
→ return to the saved boundary pose
→ Coverage resume
```

不可达/keepout 候选在规划前终止；stale 候选按时间戳拒绝；假候选可以完成观察，
但必须以 REJECTED 结束且不得发出清扫命令。导航失败会立即进入
`UNREACHABLE/navigate_to_pose_failed`，不会继续保持为可被下一候选空间合并的活动任务。

## 相机、投影与搜索 ROI

`AUTO03_corner` 是 opt-in 验证相机，不改变 production 相机，也不改变冻结的
`0.40 m × 0.36 m` 导航 footprint。相机位于 `(0.32, 0.28, 0.66) m`，
俯角 `35°`、偏航 `45°`；其旋转后包络仍位于冻结 footprint 内。

投影使用实际 `CameraInfo`、到达后的 TF 位姿、二维中心仿射标定和通用类别尺度。
目标尺寸预测框与候选搜索 ROI 分开记录：尺寸误差始终按未扩大的预测目标短边计算；
搜索 ROI 额外保留 `15 px` 标定/候选不确定度裕量，用于判断实际目标中心是否落入
可搜索区域。这样不会通过放大尺寸预测来降低短边误差。

规划投影与到达后捕获投影被显式解耦。规划侧保持冻结的保守标定，避免投影校准改变
观察位姿分布；捕获侧只以原始投影中心、基础短边和 Oracle 已允许的通用目标尺度为
特征，预测到达后的目标短边。首轮完整矩阵以 A–D 世界为训练集、E–F 世界为留出集
重新拟合捕获侧短边系数：训练 P95 从 `0.28985` 降至 `0.26288`，留出集 P95
从 `0.33187` 降至 `0.29862`，六世界离线回放 P95 为 `0.27639`。该离线结果只用于
决定是否值得重跑，不能替代新的六世界正式运行。

## 数据和回放

确定性矩阵包含 6 个 G2 Gazebo 世界、60 个 scene、250 条 trial：

- 有效目标 200 条，五类各 40 条；
- 其中 reachable 170 条、unreachable/keepout 30 条；
- false/no-target 30 条；
- stale/dropout 20 条。

每个世界都独立冷启动 Gazebo、定位、Nav2、Coverage 和主动观察节点。启动器先确认
`map_server=active` 且 `/map` 有发布者，再启动 Nav2，防止 StaticLayer 停留在
临时 `5 m × 5 m` 代价地图。所有世界串行执行，避免多个 Gazebo 实例争用 CPU
污染控制成功率。

每个世界记录 MCAP，覆盖候选、观察位姿、规划/导航 action status、捕获、评测、
Coverage 状态、刷盘、定位、里程计、TF 和速度。`auto03_replay_audit.py` 从 MCAP
反序列化全部 trial result，核对候选顺序与唯一性，重新计算同一套指标并要求最大差值
不超过 1%；随后对任务/刷盘 topic 做 remap 的实际 `ros2 bag play`。

最终正式矩阵的路径预检、可达导航、机器可判定、Coverage 恢复均为 100%；不可达、
false 和 stale 候选均 100% fail-closed，碰撞、keepout 和 GT 控制违规均为 0。
170 个可投影样本的中心误差 P50/P95 为 `7.89398/18.43392 px`，短边相对误差
P50/P95 为 `0.09270/0.29771`，中心落入搜索 ROI 和预测/实际 ready 一致率均为
100%。每个已确认目标的中位额外距离/时间为 `0.000128 m/19.502 s`，按 AUTO-02
实测基线计算的吞吐损失为 `19.598%`。六个世界的 MCAP 均达到 19/19 必需话题覆盖，
消息级指标重算最大差值为 0，实际 replay 均启动成功。

## 失败尝试保留

- 原 V5 相机出现 24.79% 自车像素、81.65 px 中心误差和返回超时。
- corner 相机的 35°/90° 偏航尝试分别留下自遮挡或导航不稳定，最终选择 45°。
- 两个 Gazebo 世界并行使世界 B 导航降至 `26/34`，因此正式矩阵改为串行。
- 首轮世界 A 冷启动时 map server 生命周期响应超时，StaticLayer 没收到地图；
  新增地图就绪门后重跑通过。
- 首轮 MCAP 固定等待漏录 action status 和 1 条 trial；现在 Oracle 延迟到 recorder
  明确订阅全部 topic 后才开始。
- 未带不确定度的投影框在世界 A 部分样本只有 `5/6` 中心命中；目标尺寸框和搜索
  ROI 拆分后，端到端烟测为 `5/5`。
- 单次 NavigateToPose 在三个样本中已接近目标位置但最终朝向未收敛；现在先完整取消
  近目标 action，再提交“当前位置 + 目标朝向”的第二个 Nav2 goal，原失败区间定向
  重验为 `6/6`，没有用 set-pose 或直接 `/cmd_vel` 代替导航。
- 首轮完整六世界矩阵的 170 个可投影样本中，中心误差 P50/P95 为
  `8.53655/20.16039 px`，但目标短边相对误差 P95 为 `0.31173`，高于 `0.30`
  硬门。该轮连同六个 MCAP 和重放审计完整保留；随后只校准到达后捕获投影的类别短边
  模型，规划投影与导航参数未改变。

失败原始日志留在 Git 忽略的 AUTO-03 raw/formal/smoke 目录，紧凑证据中的
`attempt_ledger.json` 与 `raw_metric_index.json` 提供索引。

## 复现

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File `
  .\scripts\run_auto03_matrix_docker.ps1 `
  -OutputName autonomous_auto03_formal_20260729 `
  -SkipMatrixGeneration -SkipBuild

python .\scripts\finalize_auto03.py `
  --raw-root .\artifacts\autonomous_auto03_formal_20260729 `
  --output .\artifacts\autonomous_auto03_20260729_evidence
```

完整原始 MCAP 和日志只保留在本机；Git 只提交带逐文件 SHA-256 manifest 的紧凑证据。

## 声明边界

AUTO-03 即使通过，也只代表 Docker 中 ROS 2 Jazzy + Gazebo Harmonic 的 Oracle
几何/任务链机器验收。它不代表学习感知、真人审计、真实车辆、真实域、J6 板端、
3500 m²/h 效率或最终竞赛指标通过；Stage5BR6-A 的两项人工 false 标志保持不变。
