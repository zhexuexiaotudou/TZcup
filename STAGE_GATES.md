# TZcup 验收门

本文件定义稳定的验收条件，不记录某次运行成绩。当前结果见 [`docs/current-status.md`](docs/current-status.md)，机器证据见 `artifacts/` 和最终状态 JSON。

## 通用规则

- 所有正式结果绑定精确 commit、配置、模型、数据和证据哈希；
- test、sealed final 和 evaluation truth 不参与选模、阈值选择或生产控制；
- 前置门失败时停止下游执行，未运行指标保持 `null` 或明确的 `not_executed`；
- 语法、smoke、micro、离线、仿真、实板和现场证据不得互相冒充；
- 任一碰撞、keepout、真值泄漏、许可问题或不可解释的数据泄漏均为硬失败。

## G0：仓库与环境

通过条件：

- `py -3 scripts/ci_fast.py` 通过；
- Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 及所需依赖可验证；
- 新环境可重复构建，`colcon test` 无关键失败；
- README、当前状态、项目规范、门禁和许可引用有效；
- 原始 rosbag、数据集、SDK、缓存和密钥未进入 Git。

## G1：车辆、场景与传感器

通过条件：

- 车辆稳定落地，碰撞体、惯量、轮距、刷盘和 footprint 一致；
- `/scan`、RGB-D、CameraInfo、IMU、odom 与 TF 完整且时间语义一致；
- Gazebo world、可清扫区域、keepout、动态障碍和材质可重复加载；
- 生产启动不挂载或订阅 evaluation-only 真值；
- headless 与可视化入口均能完成真实启动检查。

## G2：定位、导航与基础安全

通过条件：

- 地图加载、SLAM/定位和 Nav2 lifecycle 正常；
- 正式轨迹定位 XY RMSE 不高于 `0.05 m`；
- footprint 在 local/global costmap、Collision Monitor 和路径预检中一致；
- 静态与动态障碍零碰撞，keepout 违规为 0；
- 急停至少 30 次，P95 不高于 `1.0 s`、最大不高于 `1.5 s`，停止后速度持续为零且刷盘关闭；
- 关键 topic、TF、状态和任务终态可由 rosbag 重放重算。

## G3：覆盖清扫

通过条件：

- 所有由当前几何生成的 swath、turn、connector 和 repair 组件成功或明确 fail closed；
- 计划覆盖率不低于 `0.95`，经验覆盖率不低于 `0.90`；
- 清扫区、keepout、规划路径、实际轨迹和清扫足迹独立记录；
- 碰撞、边界违规、刷盘状态违规为 0；
- 动态干预后任务可恢复，回放重算与报告相对误差不高于 `1%`；
- 效率按真实刷幅、速度、转弯、重复与停顿计算，不以理论上限冒充实测。

## G4：感知数据与静态模型

通过条件：

- 数据来自授权域，RGB/depth/CameraInfo/TF/annotation 同步完整；
- world、asset、trajectory、相邻帧和 exact/pHash 跨 split 零泄漏；
- detector 与 area heads 独立训练、评测、导出和注册；
- 阈值只由 development/validation 数据选择，test 与 sealed final 不参与；
- 指标覆盖逐类 recall/precision、macro-F1、small-object、negative FP、area IoU 和 PyTorch/ONNX parity；
- checkpoint、预处理、阈值、模型许可、算子与 SHA-256 完整。

静态门只允许进入在线开发，不产生产品 Ready。

## G5：运动相机在线感知

正式任务需覆盖直行、转弯、behind-FOV、遮挡、反光、不同距离/尺寸、负样本和动态背景，并检查：

- eventual recall、逐类 recall 和 small-object recall；
- wrong-actionable、重复目标、地图 precision/coverage 与目标衰减；
- detector、tracker、RGB-D 投影、动态地图和调度队列的真实串联；
- 输入频率、端到端 P95、掉帧和 GPU/CPU provider；
- 运行输入、模型、配置和输出与证据哈希绑定。

所有冻结阈值必须同时通过。缺失正式任务类型时只能报告开发回归，不能报告在线产品通过。

## G6：综合仿真产品链

前置条件为 G2、G3、G5 全部通过。正式矩阵至少覆盖：

- Coverage 主任务空垃圾地图启动；
- 车载感知发现、跟踪、三维投影和动态地图融合；
- 可达目标确认、Spot Cleaning、Coverage 恢复和返航；
- 动态障碍、急停、传感器 stale、TF 缺失与任务恢复；
- 多 seed、长时 soak、完整视频/rosbag 和独立重算。

任何使用 Gazebo 真值参与生产决策的运行无效。

## G7：J6 工具链与实板

工具链门要求授权版本、环境、模型、校准集、编译命令、产物和 SHA 可追溯，并完成量化精度回归。实板门额外要求：

- 正式输入链真实运行；
- 精度与 x86 冻结候选满足一致性阈值；
- 频率、P95、内存、功耗和温度通过；
- 至少 30 分钟稳定运行，无未解释崩溃、掉帧或 provider 回退。

没有实体板时 `J6_RUNTIME_PASS` 必须为 false，实板指标保持 `null`。

## G8：真实场地

通过条件：

- 已获得采集授权、隐私处理、相机标定和数据 manifest；
- 至少 20 个真实 scene、1000 帧、目标类别完整、hard-negative 和独立地图真值；
- 同地点、同轨迹和连续帧按组隔离；
- 离散 macro-F1 不低于 `0.90`，逐类 recall 不低于 `0.85`；
- area mIoU 不低于 `0.75`，negative specificity 不低于 `0.95`；
- 地图定位 RMSE 不高于 `0.15 m`，synthetic-to-real F1 drop 不高于 `0.10`；
- 现场关键路径、安全事件和回滚均完成真人验收。

Gazebo、程序化 fixture、无标注公开图像或模型伪标签不能通过本门。

## G9：发布与产品验收

通过条件：

- PR CI 与受影响门全绿，精确修订已合并到远端 `main`；
- 发布包从该修订生成，含 manifest、SBOM、许可、配置、操作和回滚说明；
- 部署目标、修订、时间、健康检查和回滚点有记录；
- 真实部署后的关键用户路径、节点、话题、日志和状态通过；
- `neat-freak` 已使代码、README、规范、当前状态和项目规则一致。

只有 G6、G7、G8 中产品声明所需的门均通过时，才能设置对应产品 Ready。软件完整不等于竞赛矩阵、J6 实板或真实场地通过。

## 运行入口映射

| 改动范围 | 最低验证入口 |
|---|---|
| 通用代码与文档 | `py -3 scripts/ci_fast.py` |
| 车辆、传感器、SDF/Xacro | Stage 2 Docker/ROS 门 |
| SLAM、Nav2、安全 | Stage 3 与受影响安全回归 |
| Coverage | Stage 4 / Stage4W 回归 |
| 学习感知 | 对应数据 QA、静态、在线和性能脚本 |
| 页面与看板 | 真实渲染、API 和窄屏检查 |
| J6 / field | 授权环境中的实板或现场协议 |

快速检查永远不能替代受影响的真实运行门。
