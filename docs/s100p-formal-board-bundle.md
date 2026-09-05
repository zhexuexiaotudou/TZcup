# S100P 正式快照绑定板端 Bundle（静态、不可部署）

`config/s100p_formal_board_bundle_manifest.json` 是一个可复制的清单文件，不是可复制的板端 payload，也不授权向 RDK S100P 传输文件。它将当前正式车辆快照、S100P 离线预部署 product bundle、DOSOD 编译合同、EdgeSAM profile、ROS 2 launch、overlay 包合同和项目 adapter 源码逐项以路径、长度及 SHA-256 固定下来。

验证入口：

```powershell
py -3 scripts/validate_s100p_formal_board_bundle.py --allow-blocked-exit-zero
py -3 scripts/validate_s100p_formal_board_bundle.py --allow-blocked-exit-zero `
  --output reports/engineering/s100p_formal_board_bundle_audit.json
py -3 -m pytest scripts/test_validate_s100p_formal_board_bundle.py -q
```

验证器只读取本地 JSON/文本，并以 1 MiB 分块计算 SHA-256；不会 SSH、复制、安装依赖、启动 ROS 节点、访问 BPU、采集数据，或生成 DOSOD/EdgeSAM 模型与 HBM。默认退出码为 2，`--allow-blocked-exit-zero` 仅方便检查清单，绝不改变其 `BLOCKED` 结论。

当前静态链路能证明：`required_board_payload_roles` 恰有四项，并逐项锁定 `asset_key`、`target_relative_path`、`required=true` 与 `source_receipt_required=true`；验证器会将这四项分别比对 product artifact bundle 的 artifact-root 目标路径、board launch parameter record 的绝对参数记录，以及 launch 文件中实际传给 DOSOD/EdgeSAM 的参数。product bundle 内部记录的 overlay 清单长度/SHA 也必须与真实 overlay 合同一致。`sanitation_perception/package.xml` 的全部直接 `exec_depend` 会与 overlay 合同逐项核对：`sanitation_perception` 与 `sanitation_perception_interfaces` 由 overlay 提供，其余 ROS/Python 运行包以及 launch 所需的 `hobot_dosod`、`mono_edgesam` 被明确列为板端基础包豁免；这只是未安装、未验证的静态依赖闭包，不是依赖已就绪的声明。若任一绑定源缺失、不可读或哈希/长度被篡改，验证器返回结构化 `BLOCKED` JSON，而不抛出异常；且这些源与正式 snapshot 的冻结身份共同被记录。

它不能证明、也不声称已经部署：DOSOD 项目 HBM 缺失/未哈希（编译合同为 `HBM_NOT_PRODUCED`）；四个板端模型 payload 没有全部 receipt；TZcup overlay 未构建/未安装/未验证；板端 ROS 依赖与启动未验证；没有热/供电测量，也没有板端运行证据。因此 `ready_to_deploy` 和 `board_runtime_accepted` 都固定为 `false`。正式 snapshot 的本次绑定是冻结身份，不是一次 live source revalidation，更不是运行时验收。
