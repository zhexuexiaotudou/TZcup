# AUTO-05 G3 多世界数据与 screening

## 当前状态

AUTO-05 已完成 8 个真实 Gazebo 原生采集世界、4/2/2 world split、120 scene/1200 frame，并通过数据 QA。三次有界 direct detector / 独立 RGB-D leaf/puddle area heads screening 均未通过全部冻结门；最佳 Attempt 3 仍有 7 个门失败。因此 `AUTO-05=BLOCKED`，AUTO-06/07/08 不得启动。

## G4 预注册恢复边界

G4 不是第四次 V3 调参。它只允许一个冻结配置，具体合同在
`starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml`：检测仍由
direct anchor-free center/offset/bbox head 输出，增加轻量 FPN、quality 和 GIoU，
并把 leaf/puddle 改为独立 binary heads。阈值和 checkpoint 只可用 validation
world 选择；test 只可运行一次。任何失败继续保持 `AUTO-05=BLOCKED`。

G4 G3 重采只能通过 `scripts/run_auto05_g4_capture_runtime.sh` 从 fresh combined
runtime 运行。AutoDL 容器没有嵌套 Docker 权限时，只允许它做这一步原生 G3
采集；不得复用 historical Stage1/Docker capture overlay，也不得与其它 Gazebo 链并行。wrapper 在启动 Gazebo 前复用正式 runtime gate：它
校验当前 HEAD/tree、canonical snapshot、verify-recorded closure 与同一个 RUNNING
session，并登记 ROS domain/GZ partition；共享 formal Gazebo lock 拒绝并行世界。
每个 G3 scene 都实际 `gz remove` 旧 vehicle 后再 `ros_gz_sim create` 新 vehicle，
并保留 `vehicle_reset.json`，不把“停车/teleport”叙述成隔离重置。

同机且有 Docker 的环境仍可由 `scripts/run_auto05_g4_pipeline.sh` 依序执行
gate-bound capture、dataset QA、唯一冻结 G4 screening、review-gated finalizer。
AutoDL 与 OCI 环境分离时，先在 AutoDL 执行以下 `export`；归档以仓库相对逻辑
身份记录全部普通文件 SHA-256、归档 SHA-256、精确 source HEAD/tree、runtime
binding、verified closure 和 RUNNING session。再在同一 HEAD/tree、干净且已有
同源、结构合法 image receipt 的 OCI 环境执行 `import`，它拒绝覆盖、链接/特殊文件、篡改、
source/session/closure 不匹配和 replay/synthetic 替换。最后以
`AUTO05_G4_IMPORTED_HANDOFF=1` 调用 pipeline；这条路径只做 QA/screening/finalize，
不会重跑采集；实际 OCI image ID 的 Docker inspect 仍在后续 pipeline 入口执行。QA/split/leakage、contract、runtime binding、test-consumed lock 与
ONNX 文件内容仍会交叉绑定。任何非零结果均不得提升 AUTO-06/07/08。
交接脚本只支持原生 Linux/WSL checkout；Windows 宿主必须进入项目 WSL/Linux
checkout 后再运行，不能通过 NTFS junction/reparse fallback 导入。

第一次长采集在第 20 个 scene 暴露车辆跨 scene 继承速度/姿态的问题：该 scene 碰撞扰动后只完成 8/10 帧。首轮 19 个完成 scene、失败 scene 和日志独立保留；采集器已改为每个 scene 先删除并重新生成车辆，再执行随机化和同步捕获。修复不降低 10/10 帧或相邻位移门，正式 G3 数据从空目录重采。

## G3 数据合同

- 8 个世界分别使用不同 material、layout、lighting 和 SDF SHA，分配为 train/val/test `4/2/2`；
- 每世界固定 15 scene、每 scene 10 个原生 `640×480` 同步 RGB/depth/semantic/instance frame；
- 每个 val/test 世界固定 5 个 negative-only scene，即每个 held-out 世界至少 50 个纯负样本 frame；
- target variant、hard-negative asset、world 和 trajectory 按 split 隔离，test 不参与阈值或模型选择；
- 每类每场景 0–3 个实例，覆盖无目标、缺类、多实例、实际重叠、0.5–8 m、不同材质/光照、same-color negatives 和主动接近前后角色；
- 动态 hard-negative 不是 manifest 中的请求：采集器在每个同步帧后通过 Gazebo `set_pose_vector` 实际移动对象，并在 `capture_report.json` 保存逐帧位姿。

## 模型与门禁

离散目标继续使用 direct anchor-free center/offset/bbox detector，不以 segmentation connected-components 生成检测结果。RGB-D area model 为两个独立 binary heads。阈值只在两个 validation world 上选择；两个 test world 只做冻结后的最终 screening。

正式门同时要求 discovery recall/false candidates、in-domain 与 cross-world macro F1、small-object recall、negative-only FP、leaf/puddle IoU、color/material stress、same-color-negative specificity 和 ONNX 数值一致性全部通过。通过仍只表示 native Gazebo G3 离线 screening，不代表 AUTO-06 formal、live、真实域、J6 或最终竞赛感知。

## 复现

```bash
# AutoDL: after a fresh final combined runtime/session has completed native G3.
python3 scripts/auto05_g4_cross_host_handoff.py export \
  --repo "$PWD" --archive /root/autodl-tmp/auto05-g4-native-g3.tar.gz

# Clean same-source OCI checkout: build the receipt outside the import root.
AUTO05_G4_IMAGE_ROOT="$PWD/.work/auto05-g4-image" \
  bash scripts/build_auto05_g4_screening_image.sh
python3 scripts/auto05_g4_cross_host_handoff.py import \
  --repo "$PWD" --archive /transfer/auto05-g4-native-g3.tar.gz \
  --receipt /transfer/auto05-g4-native-g3.tar.gz.receipt.json \
  --oci-receipt "$PWD/.work/auto05-g4-image/evidence/screening_image.json"
AUTO05_G4_IMPORTED_HANDOFF=1 bash scripts/run_auto05_g4_pipeline.sh
```
