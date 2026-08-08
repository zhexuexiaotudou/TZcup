# AUTO-05R P11 Horizon J6 工具链边界

## 当前官方基线

2026-08-09 联网复核 Horizon 官方 OpenExplorer 文档，文档站当前版本为 `3.9.0`。
官方 PTQ 合同要求 ONNX opset `10–19`、IR version `≤9`；J6 PTQ 不在转换阶段
替换输入 layout，因此 layout 必须在导出前冻结。J6E/M 的 `NonMaxSuppression`、
`NonZero`、`TopK` 等不应进入本项目 BPU 模型图，候选筛选与 NMS 保持图外。

官方流程以 `hb_compile` 做模型检查与 PTQ/编译，以 `hb_verifier` 做模型阶段间
一致性验证；march 必须按真实芯片选择，例如 `nash-e/nash-m/nash-h/nash-p`。

## 本机证据与边界

本机存在官方 OE `3.7.0` S100/S600 离线包，archive SHA-256 与预登记值一致，
其中 `hbdk4_compiler 4.7.5`、`hmct 2.6.5`、`horizon_tc_ui 3.5.3` wheels 齐全。
但这不是当前 `3.9.0` 工具链，也尚无冻结感知 ONNX 可做 PTQ/compile，因此：

- `PRODUCT_J6_TOOLCHAIN_READY=false`；
- `PRODUCT_J6_BOARD_READY=false`；
- board FPS、温度、功耗保持 `null`；
- 3.7.0 包发现不能替代 3.9.0 获取、正式 `hb_compile`、量化精度门或实板证据。

仓库预检现强制固定 batch=1、opset/IR 合同、无 custom op、校准帧不少于 1000，
并提供保守 J6E/M BPU operator profile。最终仍以当前官方 `hb_compile` 日志为准。

官方资料：

- <https://doc.oe.horizon.auto/>
- <https://doc.oe.horizon.auto/3.9.0/guide/env_install/software_installation.html>
- <https://doc.oe.horizon.auto/en/guide/appendix/supported_op_list/operator_support/onnx_operator_support_j6em.html>
