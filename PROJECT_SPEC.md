# TZcup 系统技术规范

## 1. 产品目标

TZcup 是面向园区智慧环卫的 ROS 2 无人清扫车工程。系统在结构化道路与安全约束内完成定位、导航、覆盖清扫、垃圾发现、目标融合、定点清扫和人工监督，并以可回放证据证明每个能力边界。

默认运行环境为 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。Windows WSLg 用于本地可视化，Docker 用于无头验证；二者都不替代真实车辆、J6 实板或现场验收。

## 2. 核心原则

### 2.1 空地图启动

正式任务只加载道路、可清扫区域、静态障碍、keepout 和安全约束，不加载垃圾坐标。`DynamicTrashMap` 必须为空启动，垃圾目标只能由车载 RGB-D 当前观测产生。

### 2.2 真值隔离

Gazebo world state、semantic/instance 图和 `/ground_truth/*` 只进入独立评测节点，不得进入生产检测、跟踪、地图、调度、导航、控制或安全决策。缺少生产观测时系统必须报告 unavailable 或 blocked，不能用真值补写。

### 2.3 安全优先

Safety Perception、急停、Collision Monitor、keepout 与速度限制对所有清扫动作拥有最高优先级。任何输入缺失、时间戳过期、TF 不可用、路径不可达或状态冲突都必须 fail closed。

### 2.4 证据分级

语法检查、单元测试、离线回放、Gazebo 运行、J6 实板和真实场地是不同证据等级。低等级证据不能推导高等级产品结论；当前结论统一见 [`docs/current-status.md`](docs/current-status.md)。

## 3. 系统架构

| 子系统 | 职责 | 主要边界 |
|---|---|---|
| 车辆与场景 | 车体、刷盘、传感器、园区和动态障碍 | 模型几何、坐标系和碰撞体必须一致 |
| 定位与导航 | SLAM、融合定位、Nav2、TF 和安全速度控制 | 估计与仿真真值隔离 |
| 覆盖清扫 | 区域分解、swath、转弯、补扫、刷盘状态 | 规划区域、实际轨迹和清扫足迹分层 |
| 学习感知 | 离散垃圾 detector、leaf/puddle area heads | detector 与 area 模型独立训练、评测和发布 |
| 动态垃圾地图 | 多帧跟踪、RGB-D 投影、地图融合与衰减 | 只接收生产观测，不接收 evaluation truth |
| 任务编排 | Coverage、目标确认、定点清扫、暂停与恢复 | 只有可达、稳定目标进入执行队列 |
| 人机监督 | Gazebo、RViz、浏览器看板、审计导出 | 界面不能绕过任务编排直接控制执行器 |
| 评测与发布 | 指标、回放、证据清单、模型注册和回滚 | 结论绑定代码、配置、模型与数据哈希 |

## 4. 感知与数据合同

### 4.1 输入

- RGB、depth、CameraInfo 和 TF 必须使用同一传感时刻；
- 相机内外参与图像分辨率必须写入数据 manifest；
- 目标、world、asset、trajectory 和相邻帧按组隔离，test 不参与选模；
- 原始数据、逐帧输出和第三方 SDK 留在 Git 外，仓库只保存紧凑证据。

### 4.2 模型

- metal、bottle、paper 使用直接 object detector；禁止把 segmentation connected-components 冒充 detector；
- leaf pile 与 puddle 使用独立 area segmentation heads；
- checkpoint、阈值、预处理、算子、许可和逐文件 SHA-256 必须注册；
- PyTorch、ONNX、GPU provider 和目标运行时的预处理与输出语义必须一致。

### 4.3 在线链路

检测结果经置信度过滤、NMS、多帧跟踪、RGB-D 三维投影和地图融合后形成 actionable 目标。在线验收同时检查 recall、precision、wrong-actionable、地图质量、频率、延迟和掉帧；静态离线成绩不能单独解锁产品发布。

完整数据模型见 [`DYNAMIC_TRASH_MAP_SPEC.md`](DYNAMIC_TRASH_MAP_SPEC.md)，当前恢复协议见 [`docs/detector-data-recovery-v4.md`](docs/detector-data-recovery-v4.md)。

## 5. 导航、覆盖与安全合同

- local/global costmap、Collision Monitor、Coverage 几何和路径预检使用同一车辆 footprint；
- LiDAR 与自车过滤后的 RGB-D 点云进入障碍链，未过滤点云不得直接进入控制；
- Coverage 必须区分规划路径、实际轨迹、清扫轨迹、连接段、补扫段和刷盘状态；
- keepout 违规、碰撞、急停后非零速度和刷盘状态违规均为硬失败；
- 评测必须使用独立时间窗和真值重算覆盖率、定位误差与安全指标。

覆盖规划详见 [`docs/coverage-path-optimization.md`](docs/coverage-path-optimization.md)，车辆几何详见 [`docs/vehicle-model-guide.md`](docs/vehicle-model-guide.md)。

## 6. 人机监督合同

浏览器和 RViz 只显示可追溯的实时来源，每个来源独立标记 `live/stale/error/unavailable`。参考地图、SLAM 地图、规划结果、实际轨迹、仿真真值、感知预测和清扫覆盖不得互相代替。

任务 API 使用 token、角色、严格 schema、幂等键和受限 DSL。直接 `/cmd_vel`、电机或关节请求一律拒绝；任务编排器缺失时 Coverage、暂停、恢复和返航返回安全错误。看板实现边界见 [`docs/human-visualization.md`](docs/human-visualization.md)。

## 7. J6 与真实场地

- J6 只使用授权的 D-Robotics 工具链，记录工具、模型、校准集和编译产物哈希；
- 工具可启动不等于模型已成功量化，模型编译成功不等于实板运行通过；
- 实板验收至少覆盖精度一致性、实时性、稳定性、功耗和温度；
- 真实域指标只接受获授权的真实 RGB-D、相机标定、独立标注和地图真值；Gazebo、fixture 或伪标签不得设置真实域通过。

资源不足时应保留采集、标定、接入、标注、隐私和评测工具，并明确报告外部阻塞。

## 8. 主要接口

| 接口 | 含义 |
|---|---|
| `/cmd_vel` | 经过安全链后的底盘速度命令 |
| `/scan`、RGB-D topics、`/imu`、`/odom` | 车辆传感与里程计 |
| `/map`、`/tf`、`/tf_static` | 地图和坐标变换 |
| `/coverage/state`、`/coverage/current_path` | Coverage 任务状态与路径 |
| `/brush_enabled`、`/emergency_stop` | 清扫与安全状态 |
| perception detections / tracks | 生产感知候选与多帧跟踪 |
| dynamic trash map | 地图级目标、置信度、状态和来源 |
| `/ground_truth/*` | evaluation-only 真值，禁止生产订阅 |

接口名或消息结构变化时，必须同步 launch、配置、测试、操作文档和验收门。

## 9. 工程与发布约束

- 配置、模型、数据和证据必须可追溯到精确 commit 与 SHA-256；
- 正式运行使用冻结 profile，实验配置必须显式 opt-in；
- 仓库保存源码、配置、许可和紧凑 evidence，不保存可再生的大型中间产物；
- 发布包只能从已合并且 CI 全绿的精确 `origin/main` 生成，并提供 manifest、SBOM、许可、操作和回滚说明；
- 任何 Ready/Pass 字段只能由对应硬门生成，不能人工提升或从局部 smoke 推断。

详细验收条件见 [`STAGE_GATES.md`](STAGE_GATES.md)，开发与交付流程见 [`docs/development-workflow.md`](docs/development-workflow.md)。
