# 产品运行时基础架构

本页描述产品控制链的代码真相与不变量。A–P 验收只验证这条链，不能替代链本身。

## 三平面边界

| 平面 | 权限 | 禁止事项 |
|---|---|---|
| Safety Plane | E-stop、Collision Monitor、Nav2 costmap/keepout、底盘速度门 | 接收感知节点对安全功能的关闭或绕过请求 |
| Autonomy Plane | 定位、建图、Nav2、Coverage、任务暂停/恢复 | 使用垃圾 GT、绕过 Safety 直接运动 |
| Cleaning Intelligence | 在线感知、Tracking、ActionVerifier、DynamicTrashMap、重观察、点清洁、后验 | 直接写底盘速度、把分类或执行器成功直接解释为 CLEANED |

Cleaning Intelligence 只通过 Nav2 action 移动，通过 Coverage service 请求安全暂停/恢复，并在每次动作前重新读取 Safety Plane 的权威状态。

Safety Plane 由单一 `safety_authority` 持有 `/emergency_stop`。它上电默认急停并以 10 Hz 发布权威心跳；HMI 只能向 `/safety/operator_estop_command` 提交请求。`product_supervisor` 独立汇总扫描、定位、Coverage、相机、感知、点清扫和重观察心跳：运动平面缺失、超时或定位协方差异常会触发并锁存 E-stop，监督心跳缺失时人工清除请求必须被拒绝；监督进程由产品入口自动重启，但健康恢复不会自动解除已经锁存的急停。感知/清扫平面故障只进入 `DEGRADED` 并禁止不安全清扫，Safety/Nav2 仍保持可用。速度门、点清洁和主动重观察都要求权威心跳新鲜，权威节点消失或心跳超过 0.5 s 时分别自动输出零速度、关闭滚刷或取消运动。

## 在线目标链

```text
同步 RGB + Depth + CameraInfo + timestamped TF
  -> class-agnostic proposal / area segmentation
  -> RGB-D map projection
  -> class-agnostic association and persistent track
  -> independent ActionVerifier
     -> OBSERVE_AGAIN (最多 2 次)
     -> DEFER / REJECT
     -> ACCEPT -> CONFIRMED
  -> DynamicTrashMap
  -> product spot-clean scheduler
```

`ProductTrackerV2` 只能产生 `READY_FOR_VERIFICATION`，不能产生产品级 `CONFIRMED`。`DynamicTrashMap` 的观察融合也只能建立 `TRACKED`；只有独立 `ProductActionVerifier` 的显式 `ACCEPT` verdict 能进入 `CONFIRMED`。

ActionVerifier 固定检查：可行动类别、class confidence、background/unknown、多帧一致性、有效深度投影、投影协方差、track persistence、track/map 一致性，以及启用时的多视角分离。任何输入缺失均失效关闭；GT、registry 或 evaluation source 在入口拒绝。

## 主动重观察

`OBSERVE_AGAIN` 通过以下实际接口执行：

```text
/perception/product/reobserve_requests
  -> engineering observation-pose planner
  -> Nav2 ComputePathToPose
  -> Coverage /pause + PAUSED acknowledgement
  -> Nav2 NavigateToPose
  -> fresh ActionVerifier verdict
  -> Coverage /resume + original-state acknowledgement
```

观察位姿按实体相机安装、完整车辆 footprint、静态 cleaning boundary、实时 global costmap 和 keepout mask规划。请求没有路径、定位过期、协方差过大、E-stop、Collision Monitor 非 clear、目标/footprint 不安全或没有新 verdict 时均 DEFER；最多执行两次。

## 点清洁与后验

只有 `CONFIRMED` 目标可进入：

```text
Nav2 path precheck
  -> Coverage safe pause acknowledgement
  -> Nav2 approach (brush centre 对准目标)
  -> Pre-Clean Verification
  -> brush actuation
  -> Post-Clean Verification
  -> Coverage resume acknowledgement
```

Pre-Clean 会重新检查目标仍存在、identity/class/persistence/covariance、感知健康、定位健康、E-stop、Collision Monitor、keepout、完整 footprint 和路径。刷盘只在 Coverage 已确认暂停且 Nav2 到达后开启。

`/brush_enabled` 是共享执行器命令。Coverage 是常态所有者；Spot Cleaning 只在持有当前任务时发布，并在任务结束前显式发布一次 `false` 后释放所有权。Spot Cleaning 空闲时保持静默，禁止周期性 `false` 覆盖 Coverage 的清扫命令。

执行器成功只进入 `POST_CLEAN_VERIFY`：

- 离散垃圾必须在目标位置重新进入真实在线 camera frustum 后连续 3 帧未检出；
- Area 必须以物理面积比较，`remaining_area / before_area <= 0.10`；
- Area 最多重清一次；
- 失败或证据超时一律 DEFER，并在刷盘关闭后恢复 Coverage。

## 启动边界

完整产品仿真入口为 `sanitation_product_bringup/launch/product_simulation.launch.py`，组件级入口为 `sanitation_bringup/launch/product_cleaning.launch.py`。完整入口固定 Ackermann、1.32 m 车辆/刷宽、上电急停、Coverage 人工启动、生产相机、训练 GT 关闭，并要求生产 Coverage 控制器的 GT 订阅关闭。

产品定位不再由 ROS 侧 GT 派生。车辆 Xacro 在车体纵轴前后各布置一只 Gazebo NavSat，物理基线固定为 0.80 m；两个 `gz.msgs.NavSat` 经 `gps_msgs/msg/GPSFix` 桥接后由 `dual_navsat_adapter` 做同历元配对、基线合理性检查和确定性 RTK 误差/延迟/丢包注入，输出标准 `/gnss/fix`、`/gnss/heading` 与 `/gnss/velocity`。局部 `/odom` 只由 wheel/IMU EKF 发布。`hybrid_global_fuser` 把 WGS84 投影先变换到任务 map 坐标，再用 wheel/IMU 传播延迟的 RTK 位置与双天线航向；它是产品 `/localization/fused_pose` 和 `map→odom` 的唯一权威。Nav2 的 `external` 定位后端只保留 map server，不再启动 AMCL，因此不会形成双发布者或双 TF 权威。适配器和融合器异常退出时由产品入口重启，但安全监督仍必须因定位缺失/协方差降级重新锁存 E-stop；恢复不得自动开车。

监督节点以持续的 `map→odom` TF 作为全局定位运行心跳，同时保留位姿协方差判定，避免把静止时低频位姿事件误判为定位失联。入口默认 headless，以 `ROS_DOMAIN_ID` 派生 `GZ_PARTITION`/`IGN_PARTITION`；产品世界用 5 ms 步长与 200 Hz 更新上限形成 1× 时间基准，防止遗留或并发 Gazebo 世界向本次任务串入时钟和传感器数据。调用者必须显式提供：

- 冻结的 `pipeline_manifest`；
- 哈希匹配的 `artifact_root`；
- 非空 `mission_id`；
- mission-scoped `dynamic_map_path`；
- 静态 `cleanable_polygon_json`。
- 不含 `cleaning_targets` 的 `mission_config`、地图/keepout/speed map 与 Ackermann Nav2 参数；
- 输出目录和非空 HMI `operator_token`。

产品 HMI 不订阅 `/garbage/ground_truth`，Coverage 产品实例不订阅 `/ground_truth/odom`，launch 不启动垃圾 oracle，原始 model odometry 与 world dynamic pose 也不桥接到产品 ROS 图；这些真值桥仅在显式 `enable_evaluation_gt=true` 的独立评估模式存在。仓库自带 perception manifests 是不可激活的 placeholder；没有正式模型、哈希、CUDA provider 与支持的 postprocess contract 时，生命周期配置必须失败。

## 尚不能据此宣称的状态

代码合同和单元测试不等于实时产品证据。只有冻结模型实际激活、ROS build/test、完整 Gazebo 链、30-seed、性能、soak、fault、replay、sealed final 与 release 全部通过固定 V1 合同后，才能把 `SIMULATION_PRODUCT_COMPLETE` 设为 `true`。
