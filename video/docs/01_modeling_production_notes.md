# 第一分钟建模展示制作说明

## 成片目标

- 时长：精确 `60.0 s`
- 画幅：`1920 x 1080`
- 帧率：`30 fps`
- 编码：`H.264 / yuv420p`
- 语言：中文标签

## 镜头内容

1. 整车建模开场
2. 前后双视角整车校核
3. 六轮底盘与模块化车身
4. 双侧刷、中央滚刷、吸口与 600 mm 连续覆盖
5. UR5e + Robotiq 2F-85 机械臂单元
6. 3D LiDAR、RGB-D、RTK / IMU 多源感知塔
7. 40 L 干尘箱与 8.3 L 独立污水箱
8. 基于真实整车布局渲染的模块分解示意
9. 重新组合与关键参数

## 真实素材边界

本片只使用以下整车建模渲染：

- `formal_vehicle_product_preview.png`
- `formal_front_left.png`
- `formal_rear_right.png`
- `formal_sensor_tower.png`
- `formal_top_cleaning.png`

模块分件镜头是二维版式分解和局部放大，画面明确标注“模块分解示意”“基于整车布局渲染”，不声明存在新的 CAD 爆炸图。

## 重新生成

```powershell
py -3 scripts\render_day1_modeling_video.py
```

脚本会生成 MP4、ffprobe 信息、完整解码记录、关键帧、校验 JSON 和 SHA-256 清单。
