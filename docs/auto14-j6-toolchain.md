# AUTO-14：Horizon J6 官方工具链与板端

## 最终状态

已从 D-Robotics 官方源取得 OpenExplorer `3.7.0` 的 S100/S600 包：

- 下载字节数：`2,846,971,999`
- SHA-256：
  `DE90DA5CF58879A0883BB47856232514C3CC30E368D8864911BD05E267229C5B`
- `hbdk4_compiler 4.7.5`
- `hmct 2.6.5`
- `horizon_tc_ui 3.5.3`

官方 wheel 已安装到隔离的 CUDA 12.4/cuDNN Ubuntu 22.04 环境，
`hb_compile --help` 实际返回成功。仓库交付了可复现的包哈希/版本发现器、
环境准备脚本、ONNX checker/shape/operator/custom-op/calibration 预检、
官方 `nash-e` compile config 生成器，以及验证 HBM SHA、固定 batch=1 和
输入/输出合同的 runtime adapter。

## 尚未通过的门

AUTO-06 正式模型和至少 500 个 calibration frame 尚未生成，因此本轮没有
执行正式量化与 compile，也没有测量量化后离散 F1、区域 mIoU 或定位误差
drop。本机 USB/PCIe 设备发现没有 J6/S100/S600 实体板卡，故 30 分钟板端
稳定性、FPS、温度、功耗和端到端延迟均无数据。

AUTO-14 已达到本轮可执行停止边界，当前只能设置：

```text
OFFICIAL_TOOLCHAIN_PACKAGE_READY=true
AUTO-14=BLOCKED
first_blocking_layer=dependency_AUTO-06_formal_model_not_selected
J6_TOOLCHAIN_PASS=false
J6_RUNTIME_PASS=false
board FPS/temperature/power=null
```

官方包发现证据在
[`artifacts/autonomous_auto14_20260730_evidence/`](../artifacts/autonomous_auto14_20260730_evidence/)。
`hb_compile` 可启动不等于任一项目模型已经完成量化/编译。

## 下一步

AUTO-06 产出冻结 ONNX 后，对 detector 与 area model 分别执行：

```text
auto14_onnx_preflight.py
hb_compile -c <generated-config>
x86 Nash runtime parity
quantized metric regression
```

只有两模型编译成功、unsupported/custom op 为 0、无静默 CPU fallback 且
量化 drop 达门后，才可把 `J6_TOOLCHAIN_PASS` 改为 true。板卡仍缺失时，
`J6_RUNTIME_PASS` 必须保持 false。
