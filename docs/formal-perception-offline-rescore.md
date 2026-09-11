# 随机场感知离线复评分

`scripts/rescore_formal_random_scene_perception_offline.py` 只消费保存的 r7
诊断文件，不启动 ROS、Gazebo 或模型推理，不改变提示词、权重和阈值。输出始终
`eligible_as_formal_product_acceptance=false`。最佳单帧方块分数不能替代完整 episode
或 30 次、8 张 validation 地图的正式矩阵。

## 输入与命令

每个 `--episode-root` 必须包含：

- `dosod_raw_diagnostic.json`：真实相机图像 SHA-256、evaluator-only 声明、固定
  `0.005` 后处理 detections；显式空数组有效，缺少该档不是零检测。
- `best_front_frame.json`：848×480×3 图像形状和 20 个 staged cube 标识。
- `best_front_frame.png`：与原诊断绑定的保存图像。
- `perception_acceptance.json`：episode 标识、地污混淆计数及投影摘要。
- `tf2_echo_base_link_front_rgbd_depth_optical_frame.txt`：保存的相机变换。

在源码根目录设置实际记录目录后执行（Windows 使用 `py -3`，Linux 使用 `python`）：

```powershell
py -3 scripts/rescore_formal_random_scene_perception_offline.py --episode-root "$env:TZCUP_SAVED_EPISODE" --output .workspace/evidence/perception-offline-rescore.json
```

可重复 `--episode-root` 汇总不同记录；重复路径会拒绝。该入口采用 r7 固定相机内参、
20 个槽位和 staging frame，不能直接用于任意新场景或移动相机记录。新主线若保留相同
诊断合同即可消费；`product_intermediate_capture` 的 NPZ 回放格式不是此入口的替代输入。

## 可复算范围

| 指标 | 来源与边界 |
|---|---|
| litter_cube Precision / Recall / F1 | 保存单帧预测与重建的 staging 像素框匹配；IoU=0.50、score=0.005 |
| 地污 IoU / Precision / Recall | 保存的 intersection、union、prediction、truth 计数；必须为非负整数且满足并集恒等式 |
| 漏检样本 | 方块 object ID、按置信度排序的误检索引、漏检/误检地污格数 |
| 地污漏检面积 m² | 原摘要未绑定栅格分辨率，输出 null；不能由格数猜面积 |
| leaves / soil / puddle 逐类分数 | 原记录只有合并地污计数，没有分类真值掩膜，无法恢复 |
| 地图投影误差 | 仅转录原 episode 摘要并明确未复算；重算需保存产品 track 与 truth 配对 |

所有五个输入均有 SHA-256 追溯。文件摘要只标识消费字节，不证明当前
source/session/runtime 绑定，不能将历史诊断贴上新会话标识作为当前正式证据。

## 评分修复与验收边界（2026-09-11）

原矩阵聚合仅相信 episode 的 `status=PASSED`，缺少指标的 30 份报告也能通过。
聚合现复用评分核心重算门禁，检查冻结阈值、指标/计数、metric_checks 和 blocked_checks
一致性。缺项、陈旧 PASS 和非法数值会报告输入错误，不能提升为正式通过。

评分核心拒绝空地污真值、无穷或越界地污比率、负投影误差，以及匹配数超过可见真值
或 TP 的计数。空集合 IoU 的数学定义保留，但空真值不再证明本场地污感知通过。
离线地污能力判断同时要求 IoU≥0.65 和 Recall≥0.85，不能只凭 IoU 宣称达标。

本轮只读检查主工作区 `.workspace/evidence` 和 `.workspace/artifacts` 未定位到上述
完整原始文件集，真实复评分仍为 **BLOCKED：缺少可访问的原始 episode 目录**。
运行负责人已确认：没有已核实的同 episode r7 目录；本轮 native 工作只到
`water_recovery`，未运行 perception，也没有本轮产品采集帧（`no_product_capture_frames`）。
合成单测明确只验证文件读取、异常拒绝和算术，不提供真实 Precision/Recall 或部署证据。
实时 evaluator 也使用该核心；fresh ROS/Gazebo 感知运行门仍需在批准的运行窗口验证。

运行负责人在下一份 fresh source/install/closure、RUNNING session 和 snapshot
全部一致后使用既有入口，不更换正在执行的源码：

```bash
FORMAL_PERCEPTION_OUTPUT_ROOT="$fresh_episode_output" \
FORMAL_PERCEPTION_FINAL_ARTIFACT="$fresh_matrix_report" \
bash scripts/run_formal_random_scene_perception.sh
```

上述两个路径必须不存在；调用前还须提供 `FORMAL_VEHICLE_RUNTIME_WS`、
`FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST`、`FORMAL_ACCEPTANCE_SESSION` 和
`FORMAL_VEHICLE_SNAPSHOT_MANIFEST`。依赖为已批准的 Jazzy/Harmonic 环境、冻结模型与
ONNX Runtime，30 次完整 validation episode；只运行 `--help` 或 pytest 不满足此门。

回归命令：

```powershell
py -3 scripts/ci_fast.py
py -3 -m pytest -q scripts/test_rescore_formal_random_scene_perception_offline.py scripts/test_aggregate_formal_random_scene_perception.py starter_ws/src/sanitation_perception/test/test_formal_random_scene_evaluator_core.py
```

README 的整体能力边界保持不变。没有启动竞争仿真、覆盖原始证据或修改持久记忆；
任务分支和 worktree 保留到用户确认后再清理。
