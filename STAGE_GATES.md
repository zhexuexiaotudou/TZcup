# Codex 阶段推进与验收门

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
