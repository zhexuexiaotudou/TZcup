# AUTO-05R P13 测试工程

产品感知软件的故障注入门按 fail-closed 语义执行。以下项目已由真实代码路径的自动
测试覆盖，但这些软件测试不替代冻结模型下的 ROS/Gazebo live 验收。

| 故障 | 可执行证据 | 期望状态 |
|---|---|---|
| RGB delayed | `test_missing_or_delayed_sensor_stream_never_yields_a_frame[rgb]` | 不产出同步帧 |
| depth delayed | `test_missing_or_delayed_sensor_stream_never_yields_a_frame[depth]` | 不产出同步帧 |
| CameraInfo missing/delayed | `test_missing_or_delayed_sensor_stream_never_yields_a_frame[camera_info]` | 不产出同步帧 |
| TF missing | `test_camera_tf_latency_and_session_faults_fail_closed` | DEGRADED，禁止 spot-clean |
| depth NaN | `test_product_engine_runs_discovery_classifier_and_both_area_models` | 图像质量门拒绝 |
| GPU provider unavailable | `test_missing_cuda_or_dynamic_shape_is_rejected` | session 不创建 |
| model hash mismatch | `test_registry_fails_closed_on_corruption_or_missing_threshold[corrupt-SHA-256]` | registry 不加载 |
| corrupt ONNX | `test_corrupt_onnx_session_creation_fails_closed` | session 不创建 |
| candidate flood | `test_candidate_flood_is_bounded_before_single_classifier_batch` | top-16 后单批推理 |
| camera freeze | `test_camera_tf_latency_and_session_faults_fail_closed` | DEGRADED，禁止 spot-clean |
| OOM simulation | `test_oom_enters_error_and_snapshot_blocks_spot_clean` | ERROR，禁止 spot-clean |
| model reload failure | `test_failed_warmup_does_not_switch_active_release` | 保留旧原子指针 |

仍未通过的 P13 正式门：冻结模型产品节点 10 次冷启动 `10/10 ACTIVE`，以及至少 5 个
formal MCAP 的真实 replay，离线重算指标差 `<=1%`。在 P4/P5 freeze 和 learned-live
证据产生前，这两项必须保持 false。

当前故障路径聚焦回归在 Windows 与正式 CUDA 镜像中均为 `22 passed`；全仓库快速门
为 `466 passed / 23 skipped`。容器只读源码产生的 pytest cache 警告不影响结果。
