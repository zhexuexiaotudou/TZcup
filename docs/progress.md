# 项目推进记录

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
