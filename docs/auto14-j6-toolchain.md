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
历史 `nash-e` compile config 生成器现已由 DOSOD 上游 S100 配方约束替换；正式预检固定
`nash-m`、`rgb/NCHW -> nv12/NHWC`、`data_scale=1/255`、float32 NCHW 校准张量，
并验证 HBM SHA、固定 batch=1 和
输入/输出合同的 runtime adapter。

## 尚未通过的门

AUTO-06 正式模型和至少 500 个 calibration frame 尚未生成，因此本轮没有
执行正式量化与 compile，也没有测量量化后离散 F1、区域 mIoU 或定位误差
drop。此处“无实体板卡”是 2026-07-30 的历史探测结论；当时本机 USB/PCIe 设备发现
没有 J6/S100/S600 实体板卡，故 30 分钟板端稳定性、FPS、温度、功耗和端到端延迟均无数据。

2026-08-30 更新：真实 RDK S100P 已连接，官方参考 DOSOD/EdgeSAM、项目 overlay、
RGB→NV12 桥和 BPU/ROS 图 smoke 已完成。该结果不改变 AUTO-06 项目四类模型仍未产生的
阻塞，也不代替项目模型、非空产品输出与 1800 秒正式板端验收。

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

当前四类 DOSOD ONNX 已冻结，但项目专属 HBM 仍缺失。恢复受控 OpenExplorer 环境后先执行：

```bash
python3 scripts/collect_dosod_s100p_compiler_identity.py \
  --toolchain-discovery artifacts/autonomous_auto14_20260730_evidence/toolchain_discovery.json \
  --output .work/dosod_s100p_compiler_identity.json
python3 scripts/validate_dosod_s100p_hbm_compile_contract.py \
  --repository-root . \
  --artifact-root .work/formal_perception_assets \
  --upstream-root .work/perception_upstreams/dosod_pc \
  --calibration-dir <frozen_float32_nchw_calibration_dir> \
  --compiler-identity .work/dosod_s100p_compiler_identity.json
python3 scripts/auto14_onnx_preflight.py \
  --model .work/formal_perception_assets/dosod/dosod_mlp3x_s_tzcup_rep.onnx \
  --calibration-dir <frozen_float32_nchw_calibration_dir> \
  --output-dir .work/dosod_s100p_compile \
  --model-name dosod_mlp3x_s_tzcup_rep \
  --repository-root . \
  --artifact-root .work/formal_perception_assets \
  --upstream-root .work/perception_upstreams/dosod_pc \
  --compiler-identity .work/dosod_s100p_compiler_identity.json \
  --march nash-m --jobs 1
hb_compile -c .work/dosod_s100p_compile/dosod_mlp3x_s_tzcup_rep_config.yaml
x86 Nash runtime parity
quantized metric regression
```

冻结合同在 `config/dosod_s100p_hbm_compile_contract.json`。它锁定 ONNX SHA/字节数、
`images -> scores, boxes` 的静态 `float32` 形状、四类最后一维、opset 11、无 NMS、词表顺序、
reparameterization 输入、上游 S100 配方、OE 3.7.0 版本和预期 HBM 路径。预检只把 manifest
逐项登记、形状严格等于 `[1,3,640,640]`、dtype 为 float32、值域为 `[0,1]` 的 `.npy`
计入 500 样本门；未登记文件、重复源/张量哈希、软链接或验证集重叠都会 fail-closed。

2026-08-31 的现存数据只读盘点得到 **0 个合格校准样本**：已有 14 张 Gazebo 诊断/验证帧会
污染正式 `val` 门，其余图片是裁剪图、报告图、上游 demo 或缺少原始帧/授权。数据采集仍处于
用户要求的暂停状态，因此当前真实审计只允许报告
`live_compiler_identity_missing` 与 `calibration_directory_missing`，且不得生成 compile YAML/HBM。

只有项目 DOSOD 与选定 EdgeSAM 角色均完成制品闭环、unsupported/custom op 为 0、无静默 CPU fallback 且
量化 drop 达门后，才可把 `J6_TOOLCHAIN_PASS` 改为 true。当前 S100P 虽已连接，但项目四角色
制品、非空产品输出和 1800 秒稳定性门未通过前，`J6_RUNTIME_PASS` 必须保持 false。
