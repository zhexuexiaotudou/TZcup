# Product capture 随机场离线复评分

`scripts/rescore_product_capture_random_scene.py` 将 PR178 的
`ProductIntermediateCapture` 输出与单独保管的 evaluator truth 进行只读复评分。它
不加载模型、不启动 ROS/Gazebo、不修改 capture，也不会把 truth 传回产品路径。输出
固定为 `eligible_as_formal_product_acceptance=false`。

## 所需输入

命令要求四个互不混合的输入：

- `--capture-root`：产品侧 `frames/frame-NNNN/{arrays.npz,metadata.json,manifest.json}`
  与 `maps/<content-sha256>/`；逐文件和 map content 哈希均重新核验。
- `--public-manifest`：同一场景的 `public/episode_manifest.json`。
- `--evaluator-truth`：单独访问的 `evaluator/ground_truth.json`。它必须在
  `/evaluation/` namespace 且 `control_use_prohibited=true`。
- `--capture-binding`：由运行收集器在 capture 完成后写入的 JSON。必须包含
  episode/map identity、40 位 source commit、capture inventory、public/truth 的
  SHA-256，以及 `acceptance_session_binding` 与 `runtime_closure_binding`；还必须
  明确 `product_truth_input_used=false`。

capture 格式本身刻意不保存 truth、episode、session 或 closure。这不是缺陷：这些值
只能由独立绑定文件提供并逐项核验，避免 evaluator truth 回流进产品数据。

```powershell
py -3 scripts/rescore_product_capture_random_scene.py `
  --capture-root "$env:TZCUP_CAPTURE_ROOT" `
  --public-manifest "$env:TZCUP_PUBLIC_EPISODE_MANIFEST" `
  --evaluator-truth "$env:TZCUP_EVALUATOR_GROUND_TRUTH" `
  --capture-binding "$env:TZCUP_CAPTURE_BINDING" `
  --projection-evidence "$env:TZCUP_EVALUATOR_PROJECTION_EVIDENCE" `
  --output .workspace/evidence/product-capture-random-scene-rescore.json
```

没有 `frame-NNNN` 时工具输出 `BLOCKED/no_product_capture_frames`。缺少、篡改或错配
的 manifest、RGB-D/CameraInfo/TF/map hash、source/session/closure、episode/map
identity 或 truth isolation 都会保持 `BLOCKED`。

## 可用指标与边界

工具对每个保存帧将产品 litter cube 2D 检测与 evaluator-only cube 的相机投影匹配，
输出聚合 Precision、Recall、F1 及漏检 object ID、误检索引。它按 leaf、dust、puddle
三个 class 将保存的产品 raster 与 evaluator dirt patch 栅格比对，输出 IoU、Recall 和
漏检面积（格数乘保存地图分辨率平方）。缺失 class raster 会单独列为不可用，而不是
作为零预测或零真值。

PR178 capture 没有保存产品 target 的稳定 ID 或 map 坐标。因此没有
`--projection-evidence` 时，旧 capture 仍可完成其他离线指标，但
`map_projection.rmse_m/p95_m` 会是 `null`，原因固定为
`projection_evidence_not_supplied_for_legacy_capture`，绝不宣称通过。

需要投影误差时，可从评估侧提供独立 `--projection-evidence`。它是
`/evaluation/` namespace、`control_use_prohibited=true` 的 schema v1 JSON，必须与
capture binding 的 inventory SHA-256、source commit、acceptance session、runtime
closure 完全相同。每个 sample 必须给出唯一的产品 `target_uuid`、非空
`track_identity`、capture frame 和 detection index、与保存 RGB-D 完全一致且严格递增的
观测时间、产品 map point，以及 evaluator-only 的 `evaluator_object_id`。工具会重新从
capture 重投影 detection，要求所得 map point 与 sample 完全一致，再用 evaluator truth
中的同一 object ID 计算 RMSE/P95/max。缺 UUID、重复 UUID、跨 session/closure、相机或
map frame 不匹配、时间倒退、被修改的 capture hash、无可重投影目标或无法关联 evaluator
object 都使整个复评分保持 `BLOCKED`。

该 sidecar 只在产品运行结束后由评估侧生成；不能被产品控制、模型输入或
`ProductIntermediateCapture` 写入。它保留 PR178 capture 的既有 schema，并为有真实采集
的运行提供受哈希和坐标链约束的可选投影误差路径。

本轮没有真实 capture，故实际状态仍是 `no_product_capture_frames`。fixture 仅验证格式、
哈希、绑定、truth isolation 与 fail-closed 行为，绝不构成真实识别、定位、部署或正式
感知验收证据。
