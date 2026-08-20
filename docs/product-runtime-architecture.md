# 产品运行时基础架构

本页描述产品控制链的代码真相与不变量。A–P 验收只验证这条链，不能替代链本身。

## 三平面边界

| 平面 | 权限 | 禁止事项 |
|---|---|---|
| Safety Plane | E-stop、Collision Monitor、Nav2 costmap/keepout、底盘速度门 | 接收感知节点对安全功能的关闭或绕过请求 |
| Autonomy Plane | 定位、建图、Nav2、Coverage、任务暂停/恢复 | 使用垃圾 GT、绕过 Safety 直接运动 |
| Cleaning Intelligence | 在线感知、Tracking、ActionVerifier、DynamicTrashMap、重观察、点清洁、后验 | 直接写底盘速度、把分类或执行器成功直接解释为 CLEANED |

Cleaning Intelligence 只通过 Nav2 action 移动，通过 Coverage service 请求安全暂停/恢复，并在每次动作前重新读取 Safety Plane 的权威状态。

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

产品组件入口为 `sanitation_bringup/launch/product_cleaning.launch.py`。调用者必须显式提供：

- 冻结的 `pipeline_manifest`；
- 哈希匹配的 `artifact_root`；
- 非空 `mission_id`；
- mission-scoped `dynamic_map_path`；
- 静态 `cleanable_polygon_json`。

该 launch 不启动 `sanitation_ground_truth` 或任何垃圾 oracle。仓库自带 perception manifests 是不可激活的 placeholder；没有正式模型、哈希、CUDA provider 与支持的 postprocess contract 时，生命周期配置必须失败。

## 尚不能据此宣称的状态

代码合同和单元测试不等于实时产品证据。只有冻结模型实际激活、ROS build/test、完整 Gazebo 链、30-seed、性能、soak、fault、replay、sealed final 与 release 全部通过固定 V1 合同后，才能把 `SIMULATION_PRODUCT_COMPLETE` 设为 `true`。
