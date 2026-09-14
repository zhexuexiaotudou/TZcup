# TZcup 软件仿真源码包

本目录是 DG-202604 项目的选交代码与算法材料。内容来自已封存的 ROS 2 Jazzy / Gazebo Harmonic
仿真工程，包含车辆描述、仿真启动、建图与定位配置、Nav2、覆盖规划、安全门、任务探针、
离线地图重建和视频生成脚本。

## 目录

| 路径 | 内容 |
|---|---|
| ``ros2_ws/src/sanitation_vehicle_description`` | 四轮车辆 URDF、传感器、刷盘、尘箱和机械臂安装接口 |
| ``ros2_ws/src/sanitation_bringup`` | Gazebo、ROS-GZ bridge、EKF、真值和车辆启动 |
| ``ros2_ws/src/sanitation_navigation`` | SLAM Toolbox、AMCL、Nav2、速度/禁行区滤波配置 |
| ``ros2_ws/src/sanitation_coverage`` | OpenNav Coverage、Fields2Cover、Boustrophedon/Dubins 路径和指标计算 |
| ``ros2_ws/src/sanitation_safety`` | 命令超时、急停和速度门 |
| ``ros2_ws/src/sanitation_tasks`` | 运行时、导航、定位、建图和任务分解探针 |
| ``project_scripts`` | 离线射线建图、阶段验收、证据渲染和 5 分钟视频生成脚本 |
| ``verification`` | 视频包和证据索引校验器 |

## 环境

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12+
- Nav2、SLAM Toolbox、robot_localization、OpenNav Coverage、Fields2Cover

依赖与第三方许可证汇总见提交包的 ``dependencies/``，不在本代码目录重复复制。

## 构建与测试

在 ROS 2 环境并完成依赖安装后：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

不依赖 ROS 运行时的离线源码测试：

```bash
python3 run_offline_tests.py
```

## 主要入口

```bash
ros2 launch sanitation_bringup sim.launch.py gui:=false
ros2 launch sanitation_navigation slam.launch.py rviz:=false
ros2 launch sanitation_navigation navigation.launch.py rviz:=false
ros2 launch sanitation_coverage coverage.launch.py
```

脚本中的 stage CI 入口保留完整节点、topic、action、bag 和指标采集流程。离线地图重建可从冻结
SDF 和已知位姿生成二维地图与面积核验报告：

```bash
python3 project_scripts/offline_raycast_mapping.py --help
```

## 核心算法

- 覆盖规划：Boustrophedon 作业带、Dubins 转弯、0.65 m 作业宽度和清扫刷启停计划。
- 定位评测：同时间窗严格配对，分别报告 RMSE、P95、max，不隐藏严格 max 失败。
- 安全控制：速度命令门、超时归零、物理状态急停和停止保持检查。
- 任务分解：冻结 UTF-8 转写到确定性 DSL，开发集和内部冻结 holdout 分开报告。
- 离线地图：冻结 SDF 几何、已知位姿、全圆测距和面积/质量门；明确不是 Gazebo live SLAM。
- 视频生成：从成功分项素材按操作链重剪，生成旁白、字幕、章节、操作说明和媒体校验回执。
- 定位 live：`competition_localization_route.py` 与 stabilizer harness 在
  `map->odom` 长时间缺失时提前 fail-closed，避免再次空耗完整 420 秒窗口。

## 数据集与证据

受控感知夹具、合成评测、运行 receipt、地图面积核验和视频证据已放在提交包的
``evidence/`` 与 ``video/``。原始大 MCAP、ONNX 权重、Gazebo 构建目录和临时产物不复制到
代码包；对应的来源提交、哈希和边界记录保留在 ``evidence/index.json``。

## 状态边界

本代码包证明源码与算法可审阅、可构建，不把离线重建写成 live SLAM，不把单次路线写成长期
成功率，不把受控感知候选写成官方 ≥95% 识别，也不替代 S100P 板端或实车验证。