# 正式车全过程录制与回放接入

主动清扫的控制恢复规则见[执行与恢复合同](active-cleaning-execution-recovery.md)。本页说明运行证据的接入与尚未满足的正式封存条件；实现存在不等于全场演示已通过。

## 同一运行的采集链

`run_formal_single_episode_cleaning_mission.sh` 在操作员启动之前建立独立采集进程组，等待真实相机、录像工作进程以及MCAP订阅和文件增长。采集器只允许自身与固定 `/a12_trusted_gt_recorder` 订阅评测里程计，并核验录制器的实际PID/PGID；产品控制节点不能订阅评测真值。

正式产品话题为：

| 用途 | 话题 | 类型 |
|---|---|---|
| 检测框 | `/perception/open_vocab/dosod_boxes` | `vision_msgs/msg/Detection2DArray` |
| 目标 | `/perception/garbage/targets` | `sanitation_perception_interfaces/msg/GarbageTargetArray` |
| 任务状态 | `/active_cleaning/planner_status` | `diagnostic_msgs/msg/DiagnosticArray` |
| 物理抓投结果 | `/active_cleaning/grasp_result` | `std_msgs/msg/String` |
| 安全链之后的刷盘命令 | `/brush_controller/commands` | `std_msgs/msg/Float64MultiArray` |
| 定位估计 / 评测真值 | `/localization/fused_odom` / `/ground_truth/odom` | `nav_msgs/msg/Odometry` |

旧小车的 `/spot_clean/state` 和 `/garbage/cleaning_events` 不能用来证明正式车执行过任务。视频源为前向RGB-D传感器的RGB伴随流 `/sensors/front_rgbd/depth/image_rect_raw/image`，编码器检查真实图像编码、独立源时间戳、帧率、丢帧和首末帧时效；15 fps输出不表示真实源有15 Hz。

采集窗口由实际操作员启动和首次任务完成消息界定。刷盘、估计位姿和真值按消息时间戳窗口筛选，定位配对使用有界分块排序，覆盖率仅使用通过安全链的完整刷盘工作命令。MCAP回放必须运行真实播放器，并检查播放前后输入哈希，不能用阅读metadata替代播放。

## 评测坐标与兼容性

Gazebo模型里程计来自模型世界位姿，且保留源时间戳；实现依据可核对[Gazebo OdometryPublisher](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)。评测变换由冻结episode的起始位姿求逆得到，不能用启动时的临时位置覆盖。评测TF与协方差输出位于 `/evaluation/*`，不接入控制TF。

正式场景显式启用 `enable_evaluation_odometry` 和 `require_episode_identity`，并传入episode哈希。单项底盘、清扫机构等独立整车测试默认不启动新增ROS真值桥；旧Stage适配器保留原有显式变换使用方式。

冻结闭包新增 `sanitation_tasks` 包与ROS CLI文件身份，总包数为20。构建后在已source ROS与安装环境的shell中，先设置：

```bash
export FORMAL_ROS2_EXECUTABLE="$(command -v ros2)"
```

该路径必须是绝对路径下的普通文件，闭包绑定其SHA-256；后续正式编排将同一身份传给回放器。再按[正式编排](formal-final-acceptance-orchestration.md)依次record、verify、verify-recorded和preflight。新增包和车辆插件要求全新构建，不能把这些源码复制进旧install。

## 当前必须保留的未完成状态

- MP4观察报告 `A12_VIDEO_OBSERVED` 仅证明录像采集，不代表比赛矩阵通过。
- 采集supervisor目前保留 `A12_CAPTURE_SUPERVISOR_BLOCKED`：A12 source-metrics/raw-capture正式封存器尚未接齐。该状态不会伪造正式receipt；普通单episode仍以原collector和aggregate为准。
- `--a12-scenario` 与 `--a12-seed` 同时给出时，180条执行身份必须通过登记器。当前18种场景缺少产品入口中的对应注入证明，因此登记器在启动前拒绝；不能拿同一种综合清扫场景重命名180次。
- 0.45 m/s低速原始运行不能封成3500 m²/h正式全覆盖基线。速度重鉴定、完整建图/重启/清扫长跑、A12矩阵、A19长稳和S100实板证据须分别验收。

交接时必须携带源码版本、闭包、原始失败日志、MCAP和视频哈希。只有完整当前运行的证据齐全后，才可以报告“全过程demo完成”；测试通过与上述待验状态分开记录。
