# AUTO-05R P6 产品运行时软件合同

当前 P6 先完成不依赖冻结模型的运行时内核，产品 ROS lifecycle 节点与真实
ORT CUDA session 仍必须在 P4/P5 模型冻结后做 live 验收。

已实现：

- RGB、depth、CameraInfo 的 20 ms 硬同步门；每路缓存最多 2 帧；
- latest-frame-wins 调度，推理落后时丢弃旧帧而不形成积压；
- tracker v2 的 class-agnostic 空间关联、图像 IoU/map 距离/时间门、类别
  后验累积、置信 EMA、稳定 UUID、临时遮挡恢复和重复抑制；
- 低置信轨迹只能保持 `TENTATIVE` 或进入 `DEFERRED`，不能直接驱动清扫；
- camera stale、TF 连续错误、session 连续错误、OOM、持续超时的 watchdog，
  非 `ACTIVE` 状态一律设置 `perception_spot_clean_allowed=false`；
- leaf/puddle 的每个预测连通区独立提取 contour，并用该帧 depth、CameraInfo
  与外部传入的时间戳 TF 投影为 map polygon、物理面积、置信度和协方差；无效
  depth 直接丢弃，禁止使用 registry 固定矩形替代预测边界；
- 上述全部阈值由 `perception_pipeline_manifest.yaml` 提供，缺失、越界或队列
  大于 2 时 manifest 加载直接失败。
- 产品 model registry 以 `model_id + version + sha256` 唯一标识四模型，并在
  session 创建前验证 artifact 存在性/哈希、正式 claim、provider 兼容性和
  非空阈值；当前 placeholder 因 artifact 为空会按设计拒绝启动。
- ORT CUDA session 内核固定 `CUDAExecutionProvider` 为首 provider，调用
  `disable_fallback()`，按 manifest 固定 shape 预分配 CUDA OrtValue，使用
  `bind_ortvalue_input/output + run_with_iobinding`；warm-up profile 中只要出现
  CPU/unassigned node 就拒绝进入产品运行。实现依据当前 ONNX Runtime 官方
  CUDA EP 与 Python I/O Binding API。

仍未通过：

- 正式 `LifecycleNode` 的 configure/activate/deactivate/error 全链；
- 四个冻结模型上的真实 ORT CUDA session/warm-up/profile 验收；
- RGB stamp 的 ROS TF 查询、完整推理与多实例 polygon ROS 发布；
- ROS diagnostics topic、fault injection、10 次冷启动及 learned-live。

这些状态保持 false，不能因为纯 Python 内核测试通过而提升 P6/P7/live 门。
