# AUTO-13 真实域机器评测

## 当前状态

AUTO-13 已实现显式同意的相机/视频采集、落盘前隐私区域模糊、棋盘格标定、数据接入检查和离散/区域/定位统一评测器。工具已用程序化 fixture 验证，但 fixture 只验证软件合同，不计为真实域证据。

正式资源要求至少 20 个真实 scene/1000 frame、五类完整 GT、hard-negative、相机标定和独立 map localization ground truth。资源发现识别到 1 个 Integrated Camera 和仓库内 249 个图像文件，但合格真实 dataset manifest 为 0；相机存在和无标注图像均不能代替完整 GT。因此 `AUTO-13=BLOCKED_EXTERNAL`、`REAL_DOMAIN_BLOCKED_EXTERNAL=true`、`REAL_DOMAIN_PASS=false`。1000-frame 正式评测、map localization RMSE 和 synthetic-to-real drop 未执行，指标保持 null。紧凑证据位于 `artifacts/autonomous_auto13_20260730_evidence/`。

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
