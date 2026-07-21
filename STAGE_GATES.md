# Codex 阶段推进与验收门

## Stage5BR6-A 双人盲审交付门（当前）

- V4 只作为预注册候选，`camera_selected=false`。
- Reviewer A/B 各 270 张：Stage5BR5 正样本 200 张，真实 Gazebo no-target/hard-negative 70 张。
- 七类负样本各 10 张，semantic 目标像素总数为 0；两个包的顺序、opaque ID 和 package ID 独立。
- ZIP CRC、逐文件 SHA、sample ID 集合、PNG 元数据和 truth 泄漏审计通过。
- 两名独立真人 response：`0/2`，因此 `AWAITING_HUMAN_REVIEW=true`、`READY_FOR_STAGE5BR6_ORACLE=false`。
- V4/policy v2 冻结、candidate footprint、Stage4W footprint 回归、Oracle 主动观察和模型训练均未执行。
- 当前 `REVIEW_PACKET_COMPLETE=true`、`READY_FOR_GPT_REVIEW_STAGE5BR6=false`、`READY_FOR_STAGE5BR7=false`、`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。

## Stage5BR5 相机选择与主动观察前置门（历史）

- ActiveObservation 时间语义与回归：通过；16 项受影响测试覆盖 refresh、stale、长队列等待、空间合并、两次接近、动态 timeout、unreachable、迁移和 Coverage resume。
- V1–V4 机械网格：通过执行；V1/V2/V4 可行，V3 因 trial footprint 冲突剔除，production footprint 未修改。
- 六世界运行时相机门：V1/V2/V4 均通过；36 次 capture、360 帧精确同步证据，自像素 P95 和 target/self overlap 满足 `<=0.05`。
- 平衡盲审数据门：通过；200 张、五类各 40 张、覆盖六世界。
- 两名独立人工评审门：未执行，保持 false；accuracy/kappa/self-occlusion 指标为 null，脚本不替代评审者。
- 相机选择、policy v2 冻结和正式 oracle active-observation：被人工门阻断，均未执行。
- detector/area micro-overfit、120/1200、500/5000、live、真实 30 次 Nav2 spot-clean、真实域与 J6：按停止条件未执行。
- 当前 `REVIEW_PACKET_COMPLETE=true`、`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。

## Stage5BR4 前置恢复门（当前）

- 在任何新模型训练前冻结 `perception_evaluability_policy.yaml`，同时报告 all-visible、recognition-ready、non-ready。
- C0–C3 必须使用相同 world、asset、scene seed、目标 pose 与车辆轨迹做真实运行消融；相机配置需同时通过可辨识性、主动观察 ready conversion `>=0.90` 和安装/遮挡审计。
- 当前 C3 conversion 为 `0.50`，人工审计失败，相机未选定；因此 detector/area micro-overfit、120/1200 和后续 screening 被阻断。
- `REVIEW_PACKET_COMPLETE=true` 不改变 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。

## Stage5BR3：G2 真实车辆数据与 split-model screening

- 证据字节修复、六世界真实车辆相机运行时契约、生产 GT 隔离：通过。
- G2 80 scene/800 frame 原生采集与逐实例 QA：一次失败后修复并重采，通过。
- 四档分辨率扫描：通过，选择 640×384 与 512×384；模型实际筛查使用 512×384。
- detector + area segmenter 三次 architecture screening：失败；所有 screening 门未同时通过。
- 停止条件已执行：500/5000、live、真实 Nav2、真实域与 J6 均未启动。
- Stage5BR2 的 16 个证据文件四表面逐字节审计通过，紧凑复核包完整：`REVIEW_PACKET_COMPLETE=true`。
- 曝光、白平衡、噪声、模糊和动态障碍请求没有原生逐项施加证据，不得外推为已验证的数据增强。
- 当前 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。

## Stage5BR：Gazebo-camera 数据恢复与模型筛查

状态：训练链与 G1 smoke 通过，模型恢复 screening 失败，正式 Stage5B 阻断。

- Phase A：micro-overfit 与 PyTorch/ONNX/ROS preprocessing parity 通过。
- G1 smoke：50 scene/500 frame actual Gazebo camera，annotation/sync/split QA 通过。
- 模型 screening：三次均未同时达到 in-domain、跨资产/世界、leaf/puddle 和颜色压力门。
- 停止条件：未执行 500 scene/5000 frame formal G1、30 seed/10 min live、真实 Nav2 spot-clean 或 J6。
- 复核状态：`REVIEW_PACKET_COMPLETE=true`，`READY_FOR_GPT_REVIEW_STAGE5B=false`，`READY_FOR_STAGE5C=false`。

## Stage5B 正式门禁与 Stage5BR 前历史结果（2026-07-19）

正式通过需要：D0 Stage5A 回归、D1 真实 Gazebo-camera RGB-D 的 500 seed/5000 帧独立数据、学习模型未见测试、颜色捷径压力、30 seed/至少 10 分钟实时 Gazebo、30 次真实 Nav2 spot-clean、D2 状态披露和 J6 fail-closed 预检全部满足规划包阈值。只有全部通过时才可设置 `READY_FOR_GPT_REVIEW_STAGE5B=true` 与 `READY_FOR_STAGE5C=true`。

Stage5BR 前历史结果：D0 与 Stage4W seed 0 回归通过；两种候选确为梯度训练模型，第三种因 ONNX/J6 算子风险未训练。三次结构性筛查后，100 个未见 scene / 1000 帧离散 macro P/R/F1 为 `0.00752/0.00784/0.00768`，leaf/puddle IoU 为 `0.00376/0.2494`，颜色压力 aggregate macro F1 为 `0.05192`；仅 map RMSE `0.09731 m` 通过。该轮数据域为 `D1_procedural_rendered_not_gazebo_camera`。Stage5BR 已补齐真实 Gazebo-camera G1 smoke 数据链，但新的模型筛查仍未过门，因此仍未启动 500/5000 正式集与后续正式 E2E。

该历史轮次结论：`REVIEW_PACKET_COMPLETE=true` 仅表示失败证据可审计；`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`、`competition_perception_pass=false`、`j6_runtime_pass=false`、`competition_efficiency_pass=false`。当时第一阻塞层为 `G1_model_recovery_in_domain_cross_asset_world_and_color_stress`；真实 G1 smoke 数据链已通过，不再把“完全没有 Gazebo-camera pipeline”列为该轮阻断。

## Stage5A 当前门禁（2026-07-17）

只有 Stage4W 最小回归、GT registry/遮挡、held-out synthetic perception、30-seed spot-clean、Gazebo 实时 RGB-D/ONNX/非空 2D-3D-map 输出和正式 rosbag 全部通过时，才可设置 `READY_FOR_GPT_REVIEW_STAGE5A=true` 与 `READY_FOR_STAGE5B=true`。机器门以 `artifacts/stage5a_20260717_review/stage5a_summary.json` 为准。

正式结果：上述 9 个机器 gate 全部为 true，故 `READY_FOR_GPT_REVIEW_STAGE5A=true`、`READY_FOR_STAGE5B=true`。该 Ready 仅表示 synthetic-domain 内部工程门通过。无真实数据和 J6 实板证据时保持 `competition_perception_pass=false`、`j6_quantization_pass=false`、`j6_runtime_pass=false`；理论效率 `1053 m²/h` 未达 `3500 m²/h`，故 `competition_efficiency_pass=false`。

## Stage4W 回归基线（2026-07-17）

- Hybrid 定位回归：通过（10/10 完整、导航成功、TF 单所有者、扫描精化参与；GT 控制违规 0）。
- 定位 XY RMSE：通过（P50/P95/max `0.02825/0.03726/0.03778 m`，每 seed ≤0.05 m）。
- 静态 Coverage：通过（5/5；每 seed 为统一几何生成的 17/17 组件；经验覆盖率 `92.93%–94.53%`）。
- 动态障碍：通过（20/20 有效交互、碰撞 0、最大恢复时间小于 1 s）。
- keepout/speed filter：通过（keepout 违规 0，限速区平均 `0.288 m/s`）。
- 急停与失联安全：通过（30/30，P95 `0.188 s`；停止上游命令后 `1.694 s` 稳定归零）。
- MCAP 回放：静态 5/5 与动态完整回放通过。
- 历史 Stage4V 固定 23 组件检查：不适用；统一 headland/cutout 几何当前生成 9 swath + 8 turn = 17 组件，门禁要求全部生成组件成功。
- 竞赛效率：失败（`0.65 × 0.45 × 3600 = 1053 m²/h < 3500 m²/h`）。
- `READY_FOR_GPT_REVIEW_STAGE4W=true`；`READY_FOR_STAGE5A=true`。进入感知/J6/实板前仍需 GPT/人工复核。

## Stage4U 当前门禁（2026-07-16）

- Oracle 10-seed 有效性：通过（10/10 完整、导航成功、TF 连续、粒子仪器有效）。
- Oracle map-relative XY RMSE ≤ 0.05 m：失败（P50/P95/max `0.06767/0.07983/0.08022 m`）。
- Realistic 10-seed：未执行；前置 Oracle 硬门失败。
- 完整 Coverage/动态障碍/急停/replay：未执行；前置 realistic 硬门未满足。
- `READY_FOR_GPT_REVIEW_STAGE4U=false`；`READY_FOR_STAGE5A=false`。

## Stage 0：预检与基线锁定

### 任务

- 识别操作系统、ROS 2、Gazebo、GPU、磁盘和显示环境；
- 确认 Ubuntu 24.04 + Jazzy + Harmonic；
- 检查本包文件；
- 建立 `docs/progress.md` 和 `artifacts/`；
- 锁定第三方 commit。

### 验收

- `scripts/check_env.sh` 返回 0；
- 生成 `artifacts/preflight.json`；
- 输出版本矩阵和已知风险。

## Stage 1：工作空间可重复构建

### 任务

- 导入 starter 包和第三方仓库；
- `rosdep install`；
- 修复依赖、包清单和安装规则；
- 增加 CI/headless 构建脚本。

### 验收

- 全新 shell 中 `colcon build --symlink-install` 成功；
- 连续执行两次构建均成功；
- `colcon test` 无关键失败；
- 第三方源码无本地修改。

## Stage 2：车辆和场景启动

### 任务

- 修复/完善 `sanitation_vehicle.urdf.xacro`；
- 修复/完善 SDF 场景；
- 一键 launch；
- 校验 TF、传感器和 `/cmd_vel`；
- 添加 headless 参数。

### 验收

- Gazebo 不崩溃，实时率可接受；
- 车辆落地稳定，不弹飞、不沉降；
- 键盘可控；
- `/scan`、相机、IMU、里程计、TF 均存在；
- 冒烟检查 JSON 全部通过。

## Stage 3：SLAM、定位与导航

### 任务

- 接入 SLAM Toolbox；
- 保存地图；
- 接入 Nav2/AMCL；
- 配置 footprint、速度、加速度、costmap；
- keepout zone 和 emergency stop 速度门控。

### 验收

- 建图成功；
- 地图加载后可完成 10 个点位导航；
- 无静态碰撞；
- 定位误差脚本可输出 RMSE/P95；
- 急停延迟可测量。

## Stage 4：区域全覆盖任务

### 任务

- 接入 OpenNav Coverage/Fields2Cover；
- 读取 `demo_area.yaml`；
- 生成覆盖路径；
- 通过 Nav2 跟踪；
- 清扫 footprint 随轨迹累积；
- 输出覆盖率、漏扫率、重复率、效率。

### 验收

- 单个命令启动完整覆盖任务；
- 目标区覆盖率达到项目阶段阈值（初始 ≥90%，后续优化）；
- 任务中断后可恢复或明确失败；
- 输出 `coverage_report.json` 和轨迹图；
- 形成 3–5 分钟基础演示素材。

## 第一 GPT 复核门

完成 Stage 4 后停止继续扩展，提交：

- 最终文件树；
- commit SHA；
- 构建日志；
- 一键运行命令；
- 关键 topic/node/action 列表；
- URDF 检查结果；
- 场景截图；
- 轨迹、地图和覆盖结果；
- 所有 JSON 指标；
- 已知问题；
- 下一阶段的 3 个可选路线。

不要在没有复核的情况下直接进入大规模感知训练、J6 量化或机械臂抓取。

## Stage 5：复核后计划

- 垃圾检测/定位与仿真真值；
- 动态行人和移动障碍；
- J6 量化推理节点；
- 自然语言任务分解；
- 机械臂抓取；
- 多车集群；
- 20,000 m² headless 压测。

### Stage5BR6：双人盲审、V4 冻结与 Oracle 主动观察

Stage5BR6 分为两个不可越级的阶段。A 阶段只生成两个互相独立、无 truth 泄漏的人工盲审包；没有两份完整真人 response 时必须设置 `AWAITING_HUMAN_REVIEW=true` 并停止。B 阶段首先校验 package ID/SHA、独立性、完整字段、时间和 sample ID 集合，人工门通过后才允许冻结 V4/policy v2、生成 candidate footprint、重跑 Stage4W 和执行多世界 Oracle 主动观察。

Stage5BR6-A 的当前结论为：两个 270 张盲审包已生成，正样本 200、真实 Gazebo hard-negative/no-target 70；人工 response 为 0/2，故 `READY_FOR_STAGE5BR6_ORACLE=false`、`READY_FOR_GPT_REVIEW_STAGE5BR6=false`、`READY_FOR_STAGE5BR7=false`。脚本、LLM 或 truth mapping 不得代替真人作答。
