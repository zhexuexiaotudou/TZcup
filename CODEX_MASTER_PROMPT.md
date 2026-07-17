# 给 Codex 的主提示词：无人清扫车仿真环境与覆盖任务

> 历史入口：本文定义最初 Stage 0–4 推进和第一复核门。当前项目已推进到 Stage5A；继续开发前以根目录 `README.md`、`STAGE_GATES.md`、`docs/progress.md` 和最新 `GPT_REVIEW_STAGE*.md` 为准，不得按本文末尾的 Stage4 停止条件回退当前状态。

你现在是本项目的 ROS 2 / Gazebo / Nav2 主工程师。请在当前仓库中直接推进，不要只给建议。

## 一、项目背景

我们参加题目“面向智慧环卫场景的国产系统无人清扫车关键技术攻关”。最终采用“仿真 + 实车”方案。当前任务只聚焦仿真主线，但必须从一开始保持 sim-to-real 接口一致，并为地平线 J6 感知模型部署预留边界。

赛题核心包括：

- 建图、定位、循迹、区域全覆盖；
- 垃圾识别、定点清扫、自动清扫；
- 动态避障、边界防护、故障急停；
- 轻度积水和落叶堆场景；
- 后续多模态交互、大模型任务分解、抓取；
- 定位精度、覆盖效率、识别率、避障成功率、急停响应等指标。

## 二、你必须先阅读

按顺序完整阅读：

1. `README_FIRST.md`
2. `PROJECT_SPEC.md`
3. `COMPETITION_REQUIREMENTS.md`
4. `THIRD_PARTY_SELECTION.md`
5. `STAGE_GATES.md`
6. `starter_ws/src/` 下全部代码
7. `scripts/` 下全部脚本

然后检查当前仓库是否已包含用户此前的代码。不得覆盖用户已有成果；先做 inventory 和差异分析。

## 三、确定的技术路线

主路线：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Linorobot2 `jazzy` 作为 4WD 底盘和传感器/导航基线
- Nav2 + SLAM Toolbox + robot_localization
- OpenNav Coverage + Fields2Cover 作为区域全覆盖任务套件
- 本项目代码全部放在 `sanitation_*` 包中
- 不直接修改第三方仓库

如果目标机器并非 Ubuntu 24.04/Jazzy，不要盲目安装混合版本。先输出兼容性结论和迁移成本，再选择：
- 建议迁移到 24.04/Jazzy/Harmonic；
- 或在明确理由下使用 22.04/Humble/Fortress 分支。

## 四、工作方式

1. 直接执行命令、修改文件、构建和测试。
2. 不要因为普通编译错误、依赖缺失、参数选择而停下来询问。
3. 对可逆的小决策自行选择最稳妥方案，并记录。
4. 只有涉及硬件真实尺寸、底盘运动学类型、许可证风险或可能破坏已有成果时才暂停。
5. 每完成一个 Stage：
   - 更新 `docs/progress.md`；
   - 写清修改文件；
   - 记录命令和结果；
   - 生成 `artifacts/stageN_<timestamp>/`；
   - 提交一个语义明确的 git commit。
6. 禁止伪造运行成功、截图、指标或 rosbag。
7. 禁止只修改 README 而不落地代码。
8. 禁止为了“先跑起来”删除安全、TF、时间同步或评测要求。
9. 所有脚本必须幂等或明确说明非幂等原因。

## 五、按大步推进

### Stage 0：预检

完成：

- OS、ROS、Gazebo、GPU、显示、磁盘、Python、colcon、rosdep、vcs 检查；
- 创建 `scripts/check_env.sh` 或修复现有脚本；
- 输出 `artifacts/preflight.json`；
- 记录第三方实际分支和 commit；
- 检查 `jazzy-v2`、`v1.2.1-devel` 和 `main` 哪个 OpenNav Coverage 分支与当前 Jazzy/Nav2 能稳定构建。

通过条件：预检脚本返回 0，或者以明确错误码说明唯一阻塞项。

### Stage 1：可重复构建

完成：

- 创建/修复 `$SANITATION_WS`；
- 导入 `starter_ws/src`；
- 拉取第三方仓库；
- `rosdep install`；
- 修复 package.xml、CMakeLists、setup.py、安装规则；
- `colcon build --symlink-install`；
- `colcon test`；
- 增加 `scripts/build_ws.sh` 和 headless CI 入口。

通过条件：干净 shell 中两次连续构建成功，第三方仓库无本地修改。

### Stage 2：车辆 URDF 与场景

完成并实际运行：

- 4WD 参数化清扫车；
- 0.65 m 清扫宽度；
- 40 L 尘箱；
- LiDAR、RGB-D、IMU；
- `arm_mount_link`；
- 道路、路缘、窄通道、垃圾、落叶堆、低摩擦积水区和障碍物；
- `gui:=true/false`；
- 一键 `sim.launch.py`；
- 键盘控制；
- topic/TF 冒烟检查。

重点检查：

- 车辆不弹飞、不穿地；
- 轮胎接触和惯量合理；
- TF 无环、无重复 frame；
- 时间均使用 `use_sim_time`；
- 相机、雷达和里程计频率合理；
- 运行时不依赖在线下载模型。

通过条件：`sanitation_smoke_check` 全部通过，并保存截图、topic 列表、TF 和日志。

### Stage 3：SLAM、定位、Nav2 和安全

完成：

- SLAM Toolbox 建图 launch；
- 地图保存；
- AMCL/Nav2 导航 launch；
- 正确 robot footprint；
- 适合 4WD 清扫车的速度、加速度和 controller 参数；
- keepout filter；
- speed limit zone；
- emergency stop 高优先级速度门控；
- 10 点导航自动测试；
- 定位误差记录框架。

优先策略：

- 4WD skid-steer 初期使用 Nav2 常规 2D planner + Regulated Pure Pursuit 或 MPPI；
- 不要误用 Ackermann 最小转弯半径模型；
- 如果真实底盘最终确认 Ackermann，再新增车型和 Smac Hybrid-A*/State Lattice 配置，不要破坏当前 4WD 基线。

通过条件：自动完成 10 点导航，无碰撞；急停测试可输出延迟。

### Stage 4：区域全覆盖与清扫指标

完成：

- 集成 OpenNav Coverage + Fields2Cover；
- 从 `sanitation_tasks/config/demo_area.yaml` 读取目标多边形；
- operation width = 0.65 m；
- 生成 Boustrophedon 基线；
- 将 coverage path 交给 Nav2 跟踪；
- 转弯时允许关闭刷盘，直线 swath 时打开；
- 记录清扫 footprint 的扫掠区域；
- 计算：
  - 覆盖率；
  - 漏扫率；
  - 重复清扫率；
  - 路径长度；
  - 总时间；
  - 有效清扫效率；
  - 恢复次数；
- 支持中途注入静态障碍后重新规划或明确恢复；
- 输出 `coverage_report.json`、轨迹图和 rosbag。

如果 OpenNav Coverage 在 Jazzy 版本发生不可快速修复的依赖冲突：

1. 保留集成分支和完整错误证据；
2. 实现一个项目内最小 Boustrophedon 生成器作为 fallback；
3. 输出标准 `nav_msgs/Path`；
4. 保持未来可替换回 OpenNav Coverage 的抽象接口；
5. 不得因此跳过评测。

## 六、必须达到的第一复核门

完成 Stage 4 后停止，不要继续做大规模感知模型训练或 J6 量化。输出：

1. `GPT_REVIEW_STAGE4.md`
2. 完整 git diff 概要
3. 当前 commit SHA
4. 最终文件树
5. 从零安装和运行命令
6. 构建/测试结果
7. 车辆和场景截图
8. RViz 地图和轨迹截图
9. topic、node、action、service 清单
10. TF 树
11. coverage JSON
12. 定位和急停初步 JSON
13. rosbag 路径
14. 已知问题，按 P0/P1/P2 排序
15. 下一阶段三个方案：
    - A：先做垃圾感知 + J6；
    - B：先做动态避障 + 安全；
    - C：先做任务分解 + APP/语音。

最后明确写出：`READY_FOR_GPT_REVIEW_STAGE4=true` 或 `false`。只有所有关键证据真实存在时才能写 `true`。
