# GPT Stage 4 复核包

## 结论

Stage 0–4 的代码、构建、headless GPU 仿真、SLAM/Nav2、安全门、OpenNav Coverage/Fields2Cover 规划、指标 JSON 和 rosbag 均已落地。Stage 4 生成 12 条作业带、11 个转弯、2140 个路径点，并完成 Nav2 180 点受控执行窗；20 秒内里程计位移 7.393 m，退出时关闭刷盘。

当前只能把 97.5% 解释为规划几何覆盖率。Stage 3 的 AMCL/里程计终点差为 1.806 m，完整覆盖任务、障碍注入恢复和 GUI 截图尚未验收，因此本轮停止在 Stage 4 复核门，不进入感知训练或 J6 量化。

## 1. Git 基线、提交与 diff 概要

- 原始推进包基线：`2ce9c97ba04a4d809a3efd6fdfe1dfa184f2af8e`
- Stage 0：`76bfefcafc8eacf45f70be6d376cad5ab2940ace`
- Stage 1：`a27cdb888e1d94e06ef06e4a6d441ebea6a2a54f`
- Stage 2：`c87533b50a5355e481d72f5365009cc44d1bea93`
- Stage 3：`7b3806f1c0710d1065663188940513735cea1ae2`
- Stage 4 运行证据提交：`73d980ca758e59a5721f6303bc96082bd937016c`
- 最终当前提交以 `git rev-parse HEAD` 为准；本复核文档将在 Stage 4 证据提交之后单独提交。

相对原始推进包：134 个文件变化，新增 24,564 行、删除 127 行。主要差异为可重复 Docker/Jazzy 门禁、真实仿真与 ROS 探针、导航与安全包、覆盖规划包、分阶段 JSON/日志/rosbag 证据。完整概要命令：

```powershell
git diff --stat 2ce9c97..HEAD
git log --oneline --decorate 2ce9c97..HEAD
```

## 2. 最终文件树

```text
TZcup/
├─ docker/
│  └─ Dockerfile.jazzy
├─ docs/
│  ├─ compatibility.md
│  └─ progress.md
├─ repos/
│  ├─ locked_revisions.json
│  └─ simulation.repos
├─ scripts/
│  ├─ check_env.sh
│  ├─ import_upstream.sh
│  ├─ build_ws.sh
│  ├─ run_docker_preflight.ps1
│  ├─ run_stage1_docker.ps1
│  ├─ run_stage2_docker.ps1
│  ├─ run_stage3_docker.ps1
│  ├─ run_stage4_docker.ps1
│  ├─ stage1_ci.sh
│  ├─ stage2_ci.sh
│  ├─ stage3_ci.sh
│  ├─ stage4_ci.sh
│  └─ render_stage4_evidence.py
├─ starter_ws/src/
│  ├─ sanitation_bringup/{launch,config,package.xml,CMakeLists.txt}
│  ├─ sanitation_vehicle_description/{launch,rviz,urdf,package.xml,CMakeLists.txt}
│  ├─ sanitation_worlds/{worlds,package.xml,CMakeLists.txt}
│  ├─ sanitation_tasks/{config,resource,sanitation_tasks,test,package.xml,setup.*}
│  ├─ sanitation_navigation/{config,launch,maps,package.xml,CMakeLists.txt}
│  ├─ sanitation_safety/{resource,sanitation_safety,test,package.xml,setup.*}
│  └─ sanitation_coverage/{config,launch,resource,sanitation_coverage,test,package.xml,setup.*}
├─ artifacts/
│  ├─ preflight.json
│  ├─ stage0_20260714_223858/{host_inventory.json,preflight.json,preflight_run.log}
│  ├─ stage1_20260714_154523/{build_*.log,test_results.txt,stage1_summary.json,...}
│  ├─ stage2_20260714_163402/{runtime_probe.json,stage2_summary.json,topics.txt,...}
│  ├─ stage3_20260714_172155/{navigation_probe.json,safety_probe.json,slam_map.*,tf_static.txt,navigation_bag,...}
│  └─ stage4_20260714_174914/{coverage_report.json,coverage_path.json,coverage_plan.png,slam_map.png,coverage_bag,...}
├─ README_FIRST.md
├─ PROJECT_SPEC.md
├─ COMPETITION_REQUIREMENTS.md
├─ THIRD_PARTY_SELECTION.md
├─ STAGE_GATES.md
└─ GPT_REVIEW_STAGE4.md
```

可用 `git ls-files` 输出无省略的逐文件清单。

## 3. 从零安装、构建与运行

当前 Windows 主机没有原生 ROS 2 Jazzy，已验证通道是 Docker Desktop + NVIDIA GPU：

```powershell
docker desktop start
docker build -f .\docker\Dockerfile.jazzy -t tzcup/sanitation-jazzy:stage0 .

powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_docker_preflight.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\collect_stage0_evidence.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage1_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage2_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage3_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4_docker.ps1
```

Stage 1 创建隔离工作区、导入锁定的 Linorobot2/OpenNav Coverage、执行 rosdep 和两次构建测试；后续脚本复用最新 Stage 1 工作区。原生 Ubuntu 24.04/Jazzy 的导入方式见 `README_FIRST.md`。

## 4. 构建和测试结果

- Stage 0：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 8.11、GPU passthrough、Fields2Cover 2.0 预检通过。
- Stage 1：两次干净构建与测试通过；第三方 SHA 未改变且无 dirty 文件。
- Stage 2：12/12 运行话题有数据，车辆位移 1.18725 m，Gazebo 进程保持存活。
- Stage 3：10 点 `NavigateThroughPoses` 返回 `SUCCEEDED`；安全门正常、急停、恢复、超时归零测试通过。
- Stage 4：3 个覆盖包测试通过；累计 293 tests、0 errors、0 failures、44 skipped。跳过项均来自上游 cppcheck 慢版本保护。

测试明细：`artifacts/stage4_20260714_174914/test_results.txt`。

## 5. Stage 4 覆盖结果

| 指标 | 结果 | 证据口径 |
|---|---:|---|
| 目标面积 | 128.00 m² | demo 多边形 |
| 计划覆盖面积 | 124.80 m² | 0.10 m 栅格化 swath |
| 覆盖率 | 97.5% | 计划几何，不是经验覆盖 |
| 漏扫率 | 2.5% | 计划几何 |
| 重复清扫率 | 2.492% | 计划几何 |
| 路径长度 | 213.494 m | 完整稠密 Dubins 路径 |
| 估计总时间 | 213.597 s | Fields2Cover task time/速度估计 |
| 有效清扫效率 | 0.584 m²/s | 计划覆盖面积/估计时间 |
| 恢复次数 | 0 | 本轮未注入障碍 |
| Nav2 交接 | 180/2140 点 | AMCL 最近点起始、20 s 有界执行 |
| 有界执行位移 | 7.393 m | `/odom` |

机器可读文件：

- `artifacts/stage4_20260714_174914/coverage_report.json`
- `artifacts/stage4_20260714_174914/coverage_path.json`
- `artifacts/stage4_20260714_174914/stage4_summary.json`

OpenNav 当前 `PathComponents.swaths[*].end` 会退化到 start；兼容层利用下一 turn 首点和最终 nav path 点重建端点，并在报告中设置 `swath_endpoint_compatibility_repair=true`，原始端点也保留在 `coverage_path.json`。

## 6. 图形证据与截图边界

- 轨迹图：`artifacts/stage4_20260714_174914/coverage_plan.png`
- SLAM 地图渲染：`artifacts/stage4_20260714_174914/slam_map.png`
- 原始 SLAM 地图：`artifacts/stage3_20260714_172155/slam_map.pgm`

这两张 PNG 是从机器证据生成的 headless 复核图，不是 Gazebo/RViz GUI 截图。当前 Windows 主机没有 Ubuntu 24.04/WSLg 图形通道，因此车辆外观、完整场景和 RViz GUI 截图缺失；未伪造截图。

## 7. ROS 图谱、TF 与 rosbag

- Stage 4 actions：`artifacts/stage4_20260714_174914/actions.txt`
- Stage 4 topics：`artifacts/stage4_20260714_174914/topics.txt`
- Stage 4 services：`artifacts/stage4_20260714_174914/services.txt`
- Stage 4 nodes 快照：`artifacts/stage4_20260714_174914/nodes.txt`
- Stage 3 完整 nodes：`artifacts/stage3_20260714_172155/nodes.txt`
- Stage 3 TF 静态变换：`artifacts/stage3_20260714_172155/tf_static.txt`

运行时主链：

```text
map --AMCL--> odom --EKF/Gazebo odometry--> base_footprint --> base_link
                                                        ├─ laser
                                                        ├─ camera_link --> camera_depth_link
                                                        ├─ imu_link
                                                        ├─ left_brush_link / right_brush_link
                                                        ├─ cleaning_footprint_link
                                                        ├─ dust_bin_link
                                                        └─ arm_mount_link
```

Stage 4 节点采样只记录到两个生命周期管理节点，完整导航节点清单使用 Stage 3 同配置运行证据补足；action/topic/service 与 coverage 生命周期节点均在 Stage 4 留证。

rosbag：

- Stage 3 导航：`artifacts/stage3_20260714_172155/navigation_bag/navigation_bag_0.mcap`
- Stage 4 覆盖：`artifacts/stage4_20260714_174914/coverage_bag/coverage_bag_0.mcap`
- Stage 4 bag 共 3,646 条消息，覆盖 `/odom`、`/cmd_vel`、`/amcl_pose`、`/brush_enabled` 和 coverage visualization topic。

## 8. 定位与急停 JSON

- 定位/10 点导航：`artifacts/stage3_20260714_172155/navigation_probe.json`
- 急停速度门：`artifacts/stage3_20260714_172155/safety_probe.json`
- Stage 3 汇总：`artifacts/stage3_20260714_172155/stage3_summary.json`

急停探针验证正常指令放行、急停归零、释放恢复、0.5 秒上游超时归零。当前没有硬实时延迟分布，只能确认功能与采样结果。

## 9. 已知问题

### P0

- Stage 3 终点 AMCL 与里程计平面位置相差 1.806 m，且控制器日志有 2 次 progress failure。该问题阻塞完整覆盖实跑与经验覆盖率验收。

### P1

- 缺少原生 Ubuntu/WSLg 下的车辆、场景和 RViz GUI 截图，现有 PNG 不能替代 GUI 验收。
- Stage 4 只执行 180 点受控路径窗，没有完成 2140 点整场任务；静态障碍中途注入、重规划/恢复和无碰撞结论尚未实测。
- SLAM 地图只有 194×64、0.05 m/px，已保存但可观测区域有限，需要在定位修复后重新建图。

### P2

- OpenNav swath end point 退化需要项目兼容层；升级上游后应删除兼容并回归测试。
- Stage 4 `nodes.txt` ROS graph 瞬时采样不完整；复核时应在所有生命周期节点 active 后重复采集。
- Gazebo/Fast DDS 清理阶段仍可能出现共享内存或 shutdown 噪声，不影响本轮门禁结果，但应在演示脚本中继续收敛。

## 10. 下一阶段三个方案

- A：先做垃圾感知 + J6。优点是尽早形成国产部署链；风险是当前定位/覆盖 P0 会干扰数据采集与端到端指标。
- B：先做动态避障 + 安全。优点是直接补齐障碍注入、恢复、边界防护和急停时延证据；仍建议先用一个短迭代修复定位一致性。
- C：先做任务分解 + APP/语音。优点是较快形成产品交互演示；风险是底层覆盖可靠性不足会让上层任务成功率失真。

建议人工评审先安排“定位一致性 + 完整覆盖回放”修复门，再在 A/B/C 中优先选择 B；本轮不自行越过 Stage 4。

READY_FOR_GPT_REVIEW_STAGE4=false
