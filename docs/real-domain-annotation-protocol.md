# 真实域采集与标注协议

## 采集与隐私

真实图像采集必须取得场地和被摄人员授权，并在调用 `capture` 子命令时显式传入 `--consent`。原始相机或视频源不得自动上传；仓库只保存工具、协议和紧凑指标，不保存含个人信息的原始帧。人脸、车牌或其他可识别区域必须在 `privacy_regions.json` 中登记并在落盘前模糊，输出 PNG 不保留 EXIF。

每个 scene 使用稳定 `scene_id`，每帧使用稳定 `frame_id`、绝对时间戳、标定文件 SHA 和采集设备描述。正式集至少包含 20 个 scene/1000 frame、五类完整覆盖和 hard-negative；同一连续轨迹不得拆到不同 split。

## 标注结构

annotation JSON 顶层包含 `schema_version` 和 `frames`。每帧至少包含：

```json
{
  "frame_id": "frame_000001",
  "scene_id": "site_a_route_01",
  "hard_negative": false,
  "instances": [
    {
      "class_id": "plastic_bottle",
      "bbox_xyxy": [100, 120, 150, 210],
      "occluded": false
    }
  ],
  "area_masks": {
    "leaf_pile": "masks/frame_000001_leaf.npy",
    "puddle": "masks/frame_000001_puddle.npy"
  }
}
```

离散类为 `plastic_bottle`、`metal_can`、`paper_litter`；区域类为 `leaf_pile`、`puddle`。区域 mask 必须与 RGB 同尺寸。hard-negative 帧可无目标，但必须记录其难例类型。无法确定的对象标为 ignore，不强迫归类。

## 质量与独立性

- 两轮标注：初标与独立复核；分歧保留记录，不能由模型预测自动覆盖；
- 随机抽取至少 10% 帧核验 bbox、mask、类别和 hard-negative；
- 标注者不得看到 test 预测分数或 synthetic truth；
- 标定应使用至少 12 张清晰棋盘格图像，保存相机矩阵、畸变和重投影误差；
- map localization 必须由独立可测位姿系统、fiducial 或其他可审计真值获得，不得用里程计自比较；
- 没有完整 GT 时只能标记资源待补，不能设置 `REAL_DOMAIN_PASS=true`。

## RGB-D 与独立摆位记录

产品采集使用 `scripts/real_rgbd_capture.py capture` 同步保存 RGB、depth 与
CameraInfo，时间差硬门为 20 ms、队列深度为 2，并继续要求显式 `--consent`。
普通笔记本 RGB 摄像头不满足此合同。

每个可测目标需在 placement JSON 中记录 `frame_id/object_id/class_id`、map 坐标
`position_map_m`、`measurement_method`、`uncertainty_m` 和
`independent_of_perception=true`。允许的方法为 fiducial、surveyed fixture、
total station 或 motion capture；不确定度必须不高于 0.05 m。使用
`validate-placement` 子命令机器校验，模型预测和车辆自身里程计不能充当独立真值。

产品级 RGB-D 采集使用 `scripts/real_rgbd_capture.py capture`。每帧必须同时保存 RGB、
depth、CameraInfo 与指定时刻的 `map -> camera` TF；任一 TF 不可用即 fail-closed。
`--privacy-regions` 指定的区域在 RGB 落盘前模糊，不能依赖采集后的补处理。独立 placement
可用 `create-placement` 逐条录入，再用 `validate-placement` 校验；只有 fiducial、surveyed
fixture、total station 或 motion capture 等独立方法可通过。
