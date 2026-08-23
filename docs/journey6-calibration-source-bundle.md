# Journey 6 校准与源码部署包

## 当前状态

本链只接受明确列入 inventory 的非 sealed 开发数据，不递归发现或自动借用历史验证集。当前只读盘点范围仅包括：

```text
g10/smoke_train_world0
g10/smoke_positive_train_world0
```

盘点结果为 `471` 个 RGB PNG 候选、`0` 个名称或路径含 ROI/crop 的文件。候选文件尚未形成逐文件 SHA-256、用途角色与分层元数据，因此不能计作正式校准记录：

```text
J6_CALIBRATION_PACK_READY=false
detector_frame=0/1000 audited records
second_pass_roi=0/1000 audited records
candidate_train_rgb_png=471
candidate_roi_or_crop=0
```

`G5_V2`、`SEALED_FINAL`、`DEV_VAL` 在路径、split、source ID 或 source bundle 引用中均失效关闭。工具不会读取命中这些 token 的 payload，也不能通过把它们加入 allowlist 绕过检查。

当前 reference-only source bundle 已真实解析 development-only Area ONNX 引用：

```text
path=.workspace/models/j6f2/area/auto05_rgbd_area_segmenter.onnx
sha256=82a408f17c81f0aebe68debcb5385eccde859308f59fe8ab2e8bcff72414b3eb
```

该引用的存在和结构检查不代表当前 Area 功能验收、Journey 6 转换或运行通过。D1 detector canonical ONNX、C++ graph-external postprocess 与真实 TRAIN golden tensor lock 已具备；模型选择/发布许可、正式校准、nash profile matrix 与官方 Journey 6 工具链仍未通过，所以：

```text
J6_SOURCE_DEPLOYMENT_BUNDLE_READY=false
```

## 校准 record 合同

输入由 source config 与 JSON/JSONL record inventory 组成。每条 record 必须包含：

```json
{
  "relative_path": "detector_frame/frame_000001.png",
  "role": "detector_frame",
  "split": "calibration_train",
  "sha256": "<64 lowercase hex>",
  "strata": {
    "target_class": "plastic_bottle",
    "scene": "asphalt_campus",
    "lighting": "day",
    "distance_bucket": "far"
  }
}
```

允许的 role 只有 `detector_frame` 与 `second_pass_roi`。生产默认门槛分别为至少 1000 条；重复路径、越出 data root、缺文件、SHA 不匹配、缺任一 strata 或分布不足都会使状态为 `blocked_external`。

source config 还冻结以下预处理合同：

- RGB 输入；
- 偶数宽高，当前模板为 `640 × 640`；
- 保持宽高比的居中 letterbox、bilinear、pad value `114`；
- NV12、BT.601、limited range、UV chroma、宽高 2-byte alignment。

模板位于 `deploy/journey6/source_bundle/calibration_source.template.yaml`。先对 record inventory 文件计算 SHA-256 并写入 `source.record_inventory_sha256`，再运行：

```powershell
py -3 scripts/j6_calibration_manifest.py `
  --source-config C:\tzcup-j6\calibration_source.yaml `
  --records C:\tzcup-j6\calibration_records.jsonl `
  --data-root C:\tzcup-j6\calibration_data `
  --output-dir C:\tzcup-j6\calibration-evidence
```

输出固定为：

```text
J6_CALIBRATION_MANIFEST.json
J6_CALIBRATION_DISTRIBUTION.json
J6_CALIBRATION_SHA256SUMS
```

缺少任一门槛时仍生成带 blocker 的证据并返回退出码 2，不得把生成文件本身解释为校准通过。当前失效关闭快照位于 `deploy/journey6/source_bundle/evidence/`。

## Source bundle 合同

source bundle 只生成引用锁，不复制大 payload。必需引用包括：

- detector、classifier、Area canonical ONNX；
- 冻结 model lock 与 release-clear license audit；
- calibration manifest、distribution、checksums；
- NV12、Python/C++ postprocess 与 golden tensor lock；
- nash-e、nash-m、nash-p profiles 与 Journey 6 toolchain lock；
- board runtime/source、install、health、rollback 与 HIL config。

所有 component 固定为 `copy_policy=reference_only`。SDK archive、HBM、BC、HBO 和普通 archive 不能作为 source component；目录内嵌这些 payload 也会被拒绝。ONNX 与 golden tensor payload 仅在原位置计算 SHA 并写入 manifest，不复制进 Git 或生成目录。

运行：

```powershell
py -3 scripts/build_journey6_source_bundle.py `
  --output-dir .workspace\evidence\j6f2\source_bundle
```

当前命令真实返回退出码 2，生成：

```text
J6_SOURCE_BUNDLE_MANIFEST.json
J6_SOURCE_BUNDLE_SHA256SUMS
J6_SOURCE_BUNDLE_STATUS.json
```

当前外部证据 manifest SHA-256 为 `ef6c8572c471f3906e97c6f77518560b6ec04567f475b80b58c812cc45d62ea8`，checksums SHA-256 为 `4707fe4805e61cdd57350a2eee0a7916bbe8a4ff4b73c9303f482f15d1f14476`。D1 canonical ONNX、Area ONNX 和真实 TRAIN 图生成的 golden tensor lock 已有引用锁；由于模型未冻结、发布许可未闭合、1000+ 校准包不足、nash profile/toolchain 未经官方 SDK 验证，source bundle 仍为 `blocked_external`。只有全部 source prerequisite 真实存在、语义门通过并完成引用锁时，`J6_SOURCE_DEPLOYMENT_BUNDLE_READY` 才能改为 `true`；它仍不等于 HBM 编译、x86 runtime、板端部署或产品验收通过。
