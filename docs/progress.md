# 项目推进记录

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

下一步：进入 Stage 4，先建立可审计的 Boustrophedon 路径与覆盖指标，再决定是否允许真实 Nav2 覆盖任务受当前 AMCL 漂移影响。
