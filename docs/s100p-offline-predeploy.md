# S100P 离线预部署审计（采集暂停期）

本文件和 `scripts/validate_s100p_offline_predeploy.py` 只在 Windows 本机读取
repository、`artifact_manifest.json` 与已归档的历史 JSON。整个工作保持：

```text
operation_boundary=no_board_copy_no_ssh_no_node_start_no_data_collection
```

它不复制制品、不连 SSH、不安装依赖、不调用 `ros2 launch`、不启动节点，也不恢复任何
数据采集。它不修改 `run_formal_final_acceptance.py` 或其他 central acceptance 入口。

它只覆盖离线准备项，不是最终部署许可。最终决策必须由
[S100P 最终部署前统一硬门](s100p-final-predeploy-audit.md) 给出；该门还要求同一 PC/session/
runtime closure identity 下的 HBM、payload、overlay、依赖及热/功耗真实 receipt。

## 当前 dry-run 结论

运行：

```powershell
py -3 scripts/validate_s100p_offline_predeploy.py `
  --artifact-root .work/formal_perception_assets
```

当前预期为 `BLOCKED`，且是正确的 fail-closed 结果。已验证的本地准备项是：

该 validator 的回归测试已纳入 `scripts/ci_fast.py`。CI 只接受它在缺失项目
DOSOD HBM 时明确返回 `BLOCKED` 并保留 `formal_board_acceptance: false`；不会把
`PREDEPLOY_READY_NOT_DEPLOYED`、`BLOCKED` 或任何草稿状态写成板端部署/运行时通过。

| 范围 | 本地审计结果 | 板端含义 |
| --- | --- | --- |
| 项目 DOSOD HBM | 阻断：缺文件、SHA-256、字节数及运行时 manifest 行 | 不能传输或启动 |
| 项目四类词表 | SHA-256/字节数匹配 | 仍未部署 |
| EdgeSAM-512 encoder/decoder HBM | SHA-256/字节数匹配 | 仍未部署 |
| 类别映射、overlay 清单、启动参数记录 | SHA-256/字节数与路径合同匹配 | 仍未构建/复制 overlay |
| overlay 源包 | `sanitation_perception_interfaces`、`sanitation_perception` 源目录及 `package.xml` 名称匹配 | 不表示板上有该 overlay |
| launch 合同 | 四个节点、模型绝对路径参数、RGB-D/CameraInfo/map/TF 输入和产品输出合同静态匹配 | 不表示节点曾启动 |

项目四类 DOSOD HBM 是唯一已确认的**制品闭包**缺口。它不能由官方 COCO-80 DOSOD HBM、
官方词表、PC ONNX 或旧 smoke 替代。生成前仍先满足
`config/dosod_s100p_hbm_compile_contract.json` 的项目 ONNX、冻结词表、校准、官方 recipe 和
编译器身份合同；生成后的 HBM 还必须拥有真实编译 receipt、Nash parity 和量化回归，才可回填
bundle 清单。

## 已审计路径、包与 ROS 接口

`config/s100p_product_artifact_bundle.json` 只派生未来目标路径，不执行拷贝：

| 来源 | 未来目标根 | 内容 |
| --- | --- | --- |
| `--artifact-root`（当前 `.work/formal_perception_assets`） | `/opt/tzcup/s100p/artifacts` | `artifact_manifest.json`、DOSOD HBM/词表、EdgeSAM HBM |
| repository 根目录 | `/opt/tzcup/s100p/overlay` | 类别映射、overlay package inventory、board launch parameter record |

未来 overlay 清单固定项目包 `sanitation_perception_interfaces`、`sanitation_perception`，并要求
板端基础包 `hobot_dosod`、`mono_edgesam`、`ai_msgs`、`vision_msgs`、`diagnostic_msgs`、
`cv_bridge`、`tf2_ros`。离线检查只验证源码清单；并不声称当前板上已有项目 overlay。

静态 launch 图为 `rgb_to_nv12_adapter → hobot_dosod / mono_edgesam →
open_vocab_product_adapter`。输入是前向 RGB、depth、CameraInfo、`/map` 和 TF；DOSOD 与
EdgeSAM 内部边界是 `ai_msgs/PerceptionTargets`。产品输出固定为：

- `/perception/garbage/detections_2d` (`vision_msgs/Detection2DArray`)
- `/perception/ground_dirt/masks` (`sensor_msgs/Image`)
- `/perception/garbage/targets` (`sanitation_perception_interfaces/GarbageTargetArray`)
- `/perception/open_vocab/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

该图不包含 `/cmd_vel`、刷盘、泵、机械臂或其他执行器话题；任何未来真实 I/O 都仍须遵守
[S100P 板端优先边界](s100p-board-first-execution-boundary.md) 和
[实车 bring-up 合同](s100p-real-hardware-bring-up-contract.md)。

## 历史 G0 与 smoke 的严格口径

`artifacts/s100p_hardware_bringup/g0_20260831T001506Z_6226f4c5/G0/g0_read_only_inventory.json`
是历史只读参考：它记录到 RDK S100P 身份、`aarch64`、TROS Humble、BPU runtime、
`hobot_dosod`/`mono_edgesam` 等基础包，且明确无 ROS publish/service、CAN/GPIO 或执行器访问。
同一份清单还显示当时项目 overlay 及项目模型均为 `ABSENT`；设备枚举没有视频、串口或 CAN
节点。因此 G0 只能说明计算板基线，不能说明当前资源、项目安装或传感器就绪。

`artifacts/formal_s100p_board_smoke_20260830.json` 记录官方参考 DOSOD BPU、RGB→NV12 和五
节点图的历史 smoke。其自身的 `formal_acceptance=false`，且缺少项目四类 HBM/词表、真实
RGB-D/TF/map、非空产品输出与 1800 秒稳定性。validator 会将二者显式标为历史参考，绝不接受为
正式运行门。

G0 的七个历史基础包仅用于确认当时的只读基线；它不必覆盖未来完整 package.xml 依赖闭包。
完整闭包只能由最终统一硬门要求的 `runtime_dependencies_receipt.json` 证明，不能因 G0 缺少尚未
安装的 Python/ROS 依赖而被错误地当成历史证据损坏。

## 资源门、未来命令计划与回滚

正式采集恢复后，资源门仍以 [正式 S100P live acceptance](formal-s100-live-acceptance.md) 为准：
同一 session/snapshot 中连续 `1800 s`，BPU 后端，DOSOD `>=2 Hz`、EdgeSAM `>=1 Hz`、两者
P95 `<=1000 ms`、最低可用内存 `>=5%`、峰值温度 `<=85 °C`，零节点消失/重启/推理失败，并有
非空产品 detection/mask/target。历史 G0 的约 19 GB `MemAvailable` 只是一时快照，不是当前
资源 pass，也没有温度或 1800 秒数据。

未来由现场负责人另行授权后的命令顺序是：冻结 source/model/session/closure 并重跑 dry-run；
在维护窗口传输已验 SHA 的**版本化**制品和 overlay，保留上一版本根；重新核验真实板身份、包、
模型、词表、传感器与资源头寸；再按正式 collector 完成 1800 秒无执行器证据，回传 raw JSON 并
在冻结 worktree 离线验证。本次没有执行其中任何步骤。

本次回滚为 `not_applicable_no_board_state_changed`。未来若受权变更后任一门失败，只停止已批准
的产品图、保留日志，恢复已经记录 SHA 的上一版本化 artifact/overlay 根；不发送执行器命令。身份、
包、hash 或资源门不一致时退回 G0/G1 只读状态，正式 acceptance 保持 blocked。
