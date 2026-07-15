# Codex 主提示词：Stage 4R 修复、完整实跑与 Stage 5A 感知接口

你现在继续推进当前无人清扫车 ROS 2 仿真仓库。不要重新创建项目，不要覆盖 Stage 0–4 的历史证据。

本轮目标是：

1. 修正 Stage 4 复核中发现的证据和坐标系问题；
2. 完成真正的 SLAM 探索、同帧定位评测和完整区域覆盖实跑；
3. 验证禁行、限速、动态障碍、恢复和急停时延；
4. 输出可复核的真实场景图、轨迹、JSON 和完整 rosbag；
5. Stage 4R 全部通过后，才允许建立 Stage 5A 垃圾感知接口和仿真真值链；
6. 到下一次 GPT 复核门停止，不进行 J6 实板部署或虚假量化结论。

## 一、先阅读和保护现有成果

先完整阅读：

- `GPT_REVIEW_STAGE4.md`
- `docs/progress.md`
- `PROJECT_SPEC.md`
- `COMPETITION_REQUIREMENTS.md`
- `EVALUATION_PLAN.md`
- `starter_ws/src/` 全部项目自有代码
- `artifacts/stage2_*`
- `artifacts/stage3_*`
- `artifacts/stage4_*`

然后创建新分支：

```bash
git checkout -b stage4r-empirical-coverage
```

禁止删除或改写历史 artifact。新证据写入：

```text
artifacts/stage4r_<timestamp>/
artifacts/stage5a_<timestamp>/
```

第三方仓库继续锁定 commit，不直接修改上游源码。

## 二、必须接受的独立复核结论

### 1. 当前 1.806 m 不是合法定位误差

现有代码把：

- `/amcl_pose` 的 `map` 坐标；
- `/odom` 的 `odom` 坐标；

直接相减。禁止继续使用这个计算。

`amcl_covariance_trace_xy` 也不是位置误差。

必须把估计位姿和 Gazebo ground truth 转换到同一坐标系、同一时间基准后再计算。

### 2. 当前 SLAM 只保存了初始局部地图

不能继续把“启动 SLAM 后收到第一帧 map”作为建图成功。必须先执行自动探索或预定义建图路线。

### 3. 当前 Coverage 只运行了 180/2140 点

不能把 97.5% 解释为实际覆盖率。现有 20 秒 bounded handoff 只保留为接口测试，不得作为完整任务证据。

### 4. 当前 `success` 判据过弱

FollowPath 被接受并开始运动，不等于完整任务完成。必须重构状态和报告字段。

### 5. 当前效率未达到比赛目标

`0.584279 m²/s = 2103.4 m²/h`，低于 3500 m²/h。必须按正确单位报告并给出设计差距。

## 三、Stage 4R-1：坐标系、真值和定位误差

### 任务

1. 建立 Gazebo ground truth 输出：
   - 优先桥接 `/world/sanitation_test_world/dynamic_pose/info` 或等价模型 pose topic；
   - 提取 `sanitation_vehicle` 的世界位姿；
   - 输出标准 ROS 2 pose/odometry topic，例如 `/ground_truth/odom`；
   - 明确 frame 为 `world` 或 `map_gt`。

2. 统一坐标系：
   - 设计并记录 `world -> map` 对齐关系；
   - 推荐让车辆初始 world pose、SLAM map origin 和 AMCL initial pose有明确的一致关系；
   - 不得直接相减 `map` 与 `odom` 坐标；
   - 所有误差计算前通过 TF 转换到同一 frame。

3. 时间同步：
   - 使用 ROS simulation time；
   - 对 ground truth 和 AMCL/EKF 做近似时间同步或插值；
   - 报告实际同步误差。

4. 新增 `localization_evaluator`：
   - 输出轨迹；
   - XY error；
   - yaw error；
   - RMSE；
   - P50/P95/最大值；
   - 样本数；
   - 丢弃的未同步样本数；
   - 误差随时间曲线。

5. 重新解释 Stage 3：
   - 删除“AMCL 与 odom 坐标差就是定位误差”的逻辑；
   - 保留旧 artifact，但在新报告中明确旧指标无效。

### 验收

- ground truth topic 持续有数据；
- estimate 和 truth 均被转换到同一 frame；
- 自动单元测试验证不同 frame 不能直接比较；
- 运行至少一条闭合路线；
- 生成 `localization_report.json` 和轨迹/误差 PNG；
- `competition_localization_pass` 按 RMSE ≤0.05 m 判定；
- 未达到时保持 false，不能调宽口径。

## 四、Stage 4R-2：有效 SLAM 建图

### 任务

1. 启动 Gazebo + SLAM Toolbox；
2. 在保存地图前，自动执行建图路线：
   - 覆盖直路、转弯、狭窄通道和主要障碍；
   - 可以使用安全的预定义速度序列或 waypoint route；
   - 不依赖人工键盘。
3. 记录：
   - 机器人实际轨迹；
   - map 更新次数；
   - 已知自由/占据/未知 cell 数；
   - 已知面积；
   - 地图边界跨度；
   - map resolution。
4. 保存地图并将其用于后续导航，而不是继续使用 1.0 m/cell 的手工空白地图。
5. 地图必须是 0.05 m/cell 或更精细。
6. 提供地图质量检查脚本，拒绝：
   - 只收到一帧；
   - 已知面积过小；
   - 几乎全未知；
   - 地图尺寸明显不足。

### 初始验收阈值

- 地图分辨率 ≤0.05 m/cell；
- x 已知跨度 ≥20 m；
- y 已知跨度 ≥10 m；
- 已知面积 ≥150 m²；
- 保存后可以被 map_server 成功加载；
- 自动生成 `slam_quality_report.json`；
- 输出地图 PNG 和建图轨迹图。

阈值不适合当前世界时，可以基于场景尺寸提出有证据的调整，但不得退回“收到一帧即可”。

## 五、Stage 4R-3：完整 Coverage 组件化执行

### 设计要求

不要再把完整 2140 点路径截取最近的 180 点。

采用组件化执行：

1. 从机器人当前位姿 `NavigateToPose` 到第一条作业带起点；
2. 按 OpenNav Coverage 返回的有序 route component 执行；
3. 每条 swath：
   - 发送对应 FollowPath；
   - `brush_enabled=true`；
   - 等待终态 `SUCCEEDED`；
4. 每个 turn：
   - 发送对应 FollowPath；
   - `brush_enabled=false`；
   - 等待终态 `SUCCEEDED`；
5. 某段失败时：
   - 记录错误码；
   - 执行明确的 Nav2 recovery；
   - 有上限重试；
   - 超过上限则整个任务失败；
6. 最后一段成功后，关闭刷盘并返回任务结果。

如果 RPP 对重复平行线发生全局最近点剪枝，分段 FollowPath 是主解决方案。不得通过跳到全路径中间规避。

### 报告字段重构

至少拆分：

```json
{
  "planning_success": false,
  "transit_to_start_success": false,
  "full_execution_success": false,
  "empirical_coverage_success": false,
  "safety_success": false,
  "competition_efficiency_pass": false,
  "success": false
}
```

总 `success=true` 必须同时满足：

- 完整 route 所有 component 有终态；
- 无未处理失败；
- 刷盘最后处于关闭；
- 有真实轨迹；
- 实际覆盖指标成功生成。

不能再用 `accepted && execution_started` 作为总成功。

### 真实恢复计数

禁止硬编码 `recovery_count=0`。从：

- Navigate/FollowPath feedback；
- BT recovery 事件；
- 控制器/规划器错误；
- 本项目重试状态机；

真实统计并写出每次恢复原因。

## 六、Stage 4R-4：实际清扫覆盖率

### 任务

根据真实执行轨迹计算扫掠区域：

- 采样 `map -> cleaning_footprint_link`；
- 或将 ground truth 底盘位姿转换到 map；
- 使用 0.65 m 作业宽度；
- 对轨迹进行栅格扫掠或 polygon union；
- 只在 `brush_enabled=true` 时累计清扫面积；
- 转弯关刷时不能计入清扫面积。

输出：

- `actual_covered_area_m2`
- `actual_coverage_rate`
- `actual_miss_rate`
- `actual_repeat_rate`
- `actual_path_length_m`
- `actual_duration_s`
- `actual_efficiency_m2_h`
- `collisions`
- `recoveries`
- 每段 swath/turn 结果

必须同时保留：

- planned coverage；
- empirical coverage；

二者不能混用。

### 基线验收

- 完整任务 terminal success；
- 实际覆盖率 ≥90%；
- 无碰撞；
- 刷盘状态切换数量与 route component 对应；
- 退出后刷盘关闭；
- 生成 `empirical_coverage_report.json`；
- 生成 planned path、actual path、cleaned raster 叠加图；
- rosbag 覆盖完整任务，而不是 20 秒窗口。

## 七、Stage 4R-5：效率与赛题差距

报告单位必须包含 m²/h。

分别计算：

1. 理论直线极限：
   `operation_width × nominal_speed × 3600`
2. Fields2Cover 规划估计；
3. Gazebo 实际执行；
4. 扣除转弯、恢复、避障后的净效率。

执行参数扫描：

- 作业宽度：0.65、0.8、1.0、1.2 m；
- 作业速度：0.45、0.8、1.0、1.2、1.5 m/s；
- 记录理论效率和仿真可行性；
- 不允许仅修改 URDF 数字后宣称机械上可实现。

输出：

- 当前方案与 3500 m²/h 的差距；
- 达标所需的最小有效速度；
- 达标所需的清扫宽度；
- 车辆稳定性和安全性的影响；
- `competition_efficiency_pass`。

当前结果约 2103.4 m²/h，不得写成达标。

## 八、Stage 4R-6：禁行、限速、动态障碍和急停

### Keepout

- 单独创建 keepout mask；
- 设置明确禁行多边形；
- 规划目标故意穿过禁行区；
- 验证实际轨迹没有进入；
- 输出最小边界距离和越界次数。

### Speed zone

- 单独创建 speed mask；
- 设置明确限速区；
- 比较进入前、区内、离开后的速度；
- 输出速度限制执行率。

### 动态障碍

- 增加可重复的行人或移动箱体；
- 在覆盖任务中横穿；
- 至少执行 20 次不同随机种子；
- 统计碰撞、最小距离、任务恢复和完成率。

### 急停

修改 safety probe，记录：

- estop publish monotonic timestamp；
- 第一条零速度输出 timestamp；
- 每次响应时间；
- P50/P95/max；
- 至少 30 次试验。

验收：

- P95 ≤1.0 s；
- 最大值单独报告；
- 释放后恢复；
- 上游超时归零；
- 节点退出不再出现重复 shutdown traceback。

## 九、Stage 4R-7：可视化、rosbag 和打包

### Headless 可视化

当前没有 Gazebo GUI时，增加一个 world overview RGB camera：

- 固定在场景上方或斜上方；
- 能看到清扫车、主要道路和障碍；
- 从真实 Gazebo 渲染 topic 保存 PNG；
- 至少输出：
  - 初始场景；
  - 建图过程；
  - Coverage 中段；
  - 动态障碍避让；
  - 最终轨迹。

这些图必须来自仿真渲染，不是手工绘图。

### rosbag

- 保留整个 bag 目录；
- ZIP 必须包含 `metadata.yaml` 和所有 MCAP；
- 自动执行 `ros2 bag info`；
- 记录消息数、时长、topic；
- 验证可以回放关键 topic。

### Manifest

重新生成完整 `MANIFEST.json`：

- 包含当前所有需要提交的文件；
- 包含 SHA-256；
- 不再沿用最初 35 文件的旧 manifest。

### 图谱

在所有 lifecycle 节点 active 后采集：

- nodes；
- topics；
- actions；
- services；
- TF tree。

## 十、Stage 4R 复核门

完成后生成：

```text
GPT_REVIEW_STAGE4R.md
```

必须包含：

- commit SHA；
- 构建和测试；
- ground truth 与定位误差；
- 有效 SLAM 地图；
- 完整 Coverage terminal 结果；
- planned 与 empirical coverage；
- brush component 切换；
- 恢复次数；
- 碰撞；
- keepout；
- speed zone；
- 动态障碍；
- 急停时延；
- m²/h 效率；
- 真实 Gazebo 场景 PNG；
- 完整 rosbag info；
- P0/P1/P2 问题。

只有以下关键条件同时满足时：

```text
READY_FOR_GPT_REVIEW_STAGE4R=true
```

- 同帧定位评测存在；
- 有效 SLAM 地图存在；
- 完整 Coverage 执行完成；
- empirical coverage ≥90%；
- 无碰撞；
- 急停 P95 ≤1s；
- 完整 rosbag 可复现；
- 关键图形证据存在。

是否达到 3500 m²/h 单独作为 `competition_efficiency_pass`，不得通过伪造数据影响 Stage 4R 的软件闭环判定。

## 十一、Stage 5A：仅在 Stage 4R 通过后执行

若且仅若 Stage 4R 通过，继续建立垃圾感知仿真接口，但暂不进行 J6 实板部署。

### 创建包

- `sanitation_perception_interfaces`
- `sanitation_perception`
- `sanitation_ground_truth`
- `sanitation_dataset`

### 固定 ROS 接口

输入：

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_rect_raw`

输出：

- `/garbage/detections_2d`，`vision_msgs/Detection2DArray`
- `/garbage/targets`，`geometry_msgs/PoseArray`
- `/perception/diagnostics`

推理后端抽象：

```text
mock
ground_truth
onnxruntime
horizon_j6_placeholder
```

`horizon_j6_placeholder` 只定义接口、配置和模型元数据，不得在没有工具链和板卡证据时宣称部署成功。

### 地瓜兼容约束

- 上层任务节点不能依赖 CUDA/PyTorch 类型；
- 模型入口统一 ONNX；
- 预处理、后处理和 ROS 消息独立；
- 支持模拟推理延迟；
- 支持 FPS 限制；
- 支持丢帧统计；
- 记录模型输入尺寸、归一化、类别表和版本；
- 预留 J6 量化模型路径和 backend factory。

### 仿真真值和数据集

- 从 Gazebo 垃圾模型名称和世界 pose 生成 ground truth；
- 生成 2D/3D 标注；
- 保存 RGB、深度、相机内参、机器人 pose 和垃圾 pose；
- 随机化垃圾类别、位置、姿态和光照；
- 输出数据集 manifest；
- 至少生成一个小规模可复现实例集。

### Stage 5A 复核门

生成：

```text
GPT_REVIEW_STAGE5A.md
```

包含：

- 接口定义；
- ground truth 流；
- mock/ground_truth backend；
- 数据集样例；
- latency/FPS 模拟；
- J6 backend 边界；
- 下一步模型选择和量化计划。

完成后停止，不训练大模型，不虚构 J6 性能。

设置：

```text
READY_FOR_GPT_REVIEW_STAGE5A=true
```

或 false，并说明阻塞项。
