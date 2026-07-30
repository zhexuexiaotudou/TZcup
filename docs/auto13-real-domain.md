# AUTO-13 真实域机器评测

## 当前状态

AUTO-13 已实现显式同意的相机/视频采集、落盘前隐私区域模糊、棋盘格标定、数据接入检查和离散/区域/定位统一评测器。工具已用程序化 fixture 验证，但 fixture 只验证软件合同，不计为真实域证据。

正式资源要求至少 20 个真实 scene/1000 frame、五类完整 GT、hard-negative、相机标定和独立 map localization ground truth。资源发现完成前保持 `AUTO-13=PENDING`；若本机和仓库不存在满足要求的可审计真实集，则按规划包设置 `REAL_DOMAIN_BLOCKED_EXTERNAL=true`、`REAL_DOMAIN_PASS=false`，并继续其他独立阶段。

## 工具入口

```powershell
py -3 scripts/auto13_real_domain.py capture `
  --source 0 --frames 1000 --output <private-output> `
  --privacy-regions <privacy_regions.json> --consent

py -3 scripts/auto13_real_domain.py calibrate `
  --images <chessboard-dir> --square-size-m 0.024 `
  --output <calibration.json>

py -3 scripts/auto13_real_domain.py ingest `
  --capture-manifest <capture_manifest.json> `
  --annotations <annotations.json> `
  --calibration <calibration.json> `
  --output <real_dataset_manifest.json>

py -3 scripts/auto13_real_domain.py evaluate `
  --ground-truth <ground_truth.json> `
  --predictions <predictions.json> `
  --output <real_domain_metrics.json>
```

采集和标注细则见 `docs/real-domain-annotation-protocol.md`。真实帧默认保存在仓库外，不应提交到 Git。
