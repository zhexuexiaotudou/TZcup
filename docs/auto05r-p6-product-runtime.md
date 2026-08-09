# AUTO-05R P6 产品运行时软件合同

当前 P6 已补齐冻结模型的预处理、解码、投影、跟踪和 ROS publisher 代码；产品
ROS lifecycle 节点与真实 ORT CUDA session 仍必须在 P4/P5 模型冻结后做 live 验收。

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
- 产品 Lifecycle 入口已建立：configure 阶段完成 pipeline/model manifest、artifact
  hash、正式 claim、CUDA provider、I/O Binding、预分配与 warm-up 审计，任一失败
  都不进入 ACTIVE；同步队列仍由 20 ms/深度 2 合同控制，TF 强制使用 RGB stamp。
- 产品推理不再停在 postprocessor 占位异常：discovery 按 P3/P4/P5 解码并在图外做
  local maximum/NMS，候选按冻结 top-16 上限进入固定 batch-16 crop classifier，
  不足部分 padding 后每帧只执行一次 classifier I/O-binding run；leaf/puddle 分别构建
  与训练一致的 10 通道 RGB-D/ground-geometry 输入并输出原生分辨率 mask。
- 离散实例仅从预测 bbox 内的有效 depth 投影；每个 leaf/puddle 连通区分别从预测
  contour 生成 map polygon、物理面积、置信度与协方差，再统一进入 class-agnostic
  tracker v2。area 不使用 registry rectangle。低置信 track 保持 TENTATIVE/DEFERRED。
- 自动生成的正式 manifest 现在显式声明 `fcos_classifier_area_v1`，并绑定 freeze 的
  preprocess/postprocess hash；registry 在创建 session 前重算并核对这些合同。没有
  P5 formal claim 的 manifest 仍不能启动产品节点。
- ROS 产品节点发布预测 leaf/puddle mask 和 `GarbageTargetArray`，同时将 preprocess、
  discovery、classifier、leaf、puddle、projection、tracking 和 end-to-end 延迟写入
  metrics。keepout 权威输入尚未接入，因此所有目标暂时按 blocked 标记，禁止误触发。
- 已建立不可变 release 的原子激活指针；新版本只有在独立 stage 中完成 registry
  校验和 inactive warm-up 后才能切换，失败保持旧指针，显式 rollback 可恢复上一版本。
- 产品容器、Compose、build/run/healthcheck 和 release packaging 已加入；当前正式模型
  为空，因此真实 release 构建按设计失败关闭，不能先产出空壳产品包。
- P7 性能监控按 manifest 固定记录 preprocess、discovery、classifier batch、leaf、
  puddle、projection、tracking、inference pipeline 与 end-to-end 的 P50/P95，以及
  effective Hz、drop rate、候选/拒绝/轨迹数、CPU/GPU memory；门限为推理 P95
  `≤150 ms`、端到端 P95 `≤200 ms`、`≥10 Hz`、drop `≤1%`。两小时 soak
  审计同时要求零 crash/deadlock/意外 reload/TF stale storm、队列深度 `≤2`、
  内存增长 `≤5%`。当前无冻结模型，指标样本不足时门按设计为 false。

仍未通过：

- 四个冻结模型上的真实 ORT CUDA session/warm-up/profile 验收；
- RGB stamp 的 ROS TF 查询、完整推理与多实例 polygon 的真实 Gazebo 验收；
- ROS diagnostics topic、fault injection、10 次冷启动及 learned-live。
- 冻结模型下的真实性能门和连续两小时 Gazebo soak。

这些状态保持 false，不能因为纯 Python 内核测试通过而提升 P6/P7/live 门。

当前软件验证证据：Windows 快速门 `460 passed / 23 skipped`；正式 CUDA 镜像中的
产品 runtime/manifest/packaging 聚焦回归 `30 passed`；ROS 2 Jazzy 容器实际构建
`sanitation_perception_interfaces` 与 `sanitation_perception` 两包成功，随后 colcon
测试 `65 passed / 0 failed`。以上只证明
实现和安装链成立，不能替代冻结模型上的 provider profile、Gazebo live 或 soak。
