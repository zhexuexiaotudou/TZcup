# 模型与资产许可证

## 项目模型

AUTO-04 与 AUTO-05 证据中的 ONNX 模型由本项目脚本在自建 Gazebo
合成数据上训练，没有下载或微调第三方预训练权重。模型随
`sanitation_learning` 包按 Apache-2.0 交付。其指标只适用于相应证据等级。

## Gazebo 与车辆资产

正式整车的 A300、UR5e、Robotiq 2F-85 和部分传感器外壳采用已锁定版本、允许
再分发的上游 mesh；每个来源的 commit、用途、限制和许可证登记在
`starter_ws/src/sanitation_vehicle_description/meshes/vendor/SOURCES.yaml`，许可证
原文随各自目录保留。项目自建的清扫、回收、分仓、安装支架和防护外壳 mesh
由 `cad/formal_vehicle/` 或 `scripts/generate_*mesh*.py` 中的参数化源生成并按
Apache-2.0 交付。Gazebo Fuel、网页图片、照片、音视频或来源不明资产不进入仓库。

## 外部依赖

- `linorobot2@b96aa42fbfa4390a77e0aab90935fe55d66d04ba`：Apache-2.0；
- `opennav_coverage@224118081c4c8de651f1db621053ab873b08f13d`：Apache-2.0；
- `clearpath_common@b0f6d920422ad302372a1c65e31d61648da884ed`：BSD-3-Clause；正式A300描述候选；
- `clearpath_simulator@ee098ad6f67b4e35d77841ed6f004b8f86cd77e4`：BSD-3-Clause；ROS 2 Jazzy/Gazebo Harmonic候选；
- `Universal_Robots_ROS2_Description@39242984dc8d1fff9584c922c17c69c58df3591d`：BSD-3-Clause；UR5e描述；
- `robotiq/ros@3ab3befccaa10468f80803ba687105c9d224d567`：BSD-3-Clause；2F-85描述和驱动参考；
- `robotnik_sensors@11852f4f14d6e0a561117396187f7674ed33ab2b`：BSD-3-Clause；UTM-30LX、MID-360、D435、u-blox与VectorNav描述候选；
- ROS 2 Jazzy、Gazebo Harmonic、Nav2、MoveIt2、PyTorch、ONNX Runtime
  等按各自上游许可证安装，不复制到最终 ZIP；
- D-Robotics OpenExplorer 3.7.0 的 2.85 GB 官方包不进入 Git 或最终 ZIP，
  仅记录官方来源、版本和 SHA；使用时需遵守厂商 SDK 条款。

当前清单中的未知许可证数量为 0。外部 mesh 不能因被复制进 Apache-2.0 包而被
重新许可；其原许可证继续生效。若未来加入外部模型、mesh、纹理、字体或数据集，
必须先登记来源、版本、许可证和 SHA，才能进入发布包。

## 高保真整车的厂商数据表边界

Arducam、Pololu、Actuonix、Jabsco/Xylem、Hokuyo、Livox、Intel、u-blox、
VectorNav、Universal Robots和D-Robotics页面目前只作为型号、尺寸、质量、
FOV、功率或性能参数来源。除非单个下载物另附明确允许修改和再分发的许可，
仓库不复制其网页图片、PDF、STEP/IGES或其他厂商资产。正式刷盘、刮吸、箱体、
支架和无开放网格的外壳将根据公开尺寸由项目重新建模，并记录修改与哈希。
