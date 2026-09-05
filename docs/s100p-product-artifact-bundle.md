# S100P 产品制品准备包（不部署）

`config/s100p_product_artifact_bundle.json` 是 RDK S100P / Journey 6P /
`nash-m` 的本地准备清单，而不是部署脚本。它固定四个正式项目角色：DOSOD HBM、DOSOD
词表、EdgeSAM-512 encoder HBM 和 decoder HBM；同时绑定正式类别映射、项目 overlay/包
清单及板端启动参数记录。

所有源路径均相对于明确的本地根目录：模型与运行时 `artifact_manifest.json` 相对于
`--artifact-root`，项目配置相对于 `--repository-root`。板端路径只由清单中的
`/opt/tzcup/s100p/artifacts` 和 `/opt/tzcup/s100p/overlay` 加相对路径派生并写入报告；验证器
不会复制该文件、建立 SSH、安装依赖、启动 ROS 节点或恢复数据采集。

运行本地预检：

```powershell
py -3 scripts/validate_s100p_product_artifact_bundle.py `
  --artifact-root .work/formal_perception_assets
```

该命令的默认返回码为非零，直到每个正式文件存在、大小和 SHA-256 匹配，并且运行时
`artifact_manifest.json` 也含有四个相同的 `rdk_s100` / `nash-m` 行。当前清单故意保持
`PREPARATION_ONLY_BLOCKED`：现有本地正式运行时清单没有 DOSOD HBM 行，不能用 EdgeSAM
HBM、ONNX 文件或官方 COCO-80 示例替代。即使未来本地制品齐全，该检查的成功状态也只是
`PREPARED_NOT_DEPLOYED`，不构成板端运行、实物 I/O 或执行器放行。

项目四类别 DOSOD HBM 生成前还必须通过
`config/dosod_s100p_hbm_compile_contract.json` 与
`scripts/validate_dosod_s100p_hbm_compile_contract.py`。该门把准确的四类 ONNX 图合同、冻结词表、
reparameterization 输入、官方 S100 配方、至少 500 个独立校准张量及实时 OE 3.7.0 编译器身份
绑定为一个 `compile_plan_sha256`。它只授权后续 ONNX/toolchain preflight，不产生或接受 HBM；
只有真实 `hb_compile` receipt、HBM SHA/大小、x86 Nash parity、量化指标回归和板端运行门完成后，
才能回填本 bundle。官方 COCO-80 HBM 不能作为该链的正例。

需要把失败报告供其他本地审计读取时，可显式传入 `--report <local-path>`；它只写该本地 JSON
报告。`--allow-blocked-exit-zero` 仅适用于读取 BLOCKED 报告的编排，不会改变报告状态。
