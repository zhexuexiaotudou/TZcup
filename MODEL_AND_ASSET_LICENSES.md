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
- ROBOTIS DYNAMIXEL XW540-T260-R 官方 e-Manual：仅作为投放闸门执行器的公开尺寸、质量、12 V 瞬时堵转转矩和 IP68 参数来源；仓库不复制厂商 CAD，外壳、安装耳和输出盘由项目参数化重建；
- `D-Robotics-AI-Lab/DOSOD@c50129b5badf6ed7bb85e692ab493d8bdb58da6a`：GPL-3.0；PC侧重参数化、ONNX导出与参考推理源码；
- `chongzhou96/EdgeSAM@d24d99671f41a9c0003061248bded64a481e9059`：S-Lab License 1.0，仅允许非商业使用和再分发；本竞赛研发可作为PC基线，任何商业化实车用途需另行取得许可；
- `D-Robotics/hobot_dosod@c5b585204cd84585af1830b00da0c66c1383305e`：Apache-2.0仓库；含S100参考HBM和TROS节点，模型权利边界仍以锁定仓库随附文件为准；
- `D-Robotics/mono_edgesam@a24b1cb29ad10cfd86802a3731cf26ba621a406a`：Apache-2.0仓库；含S100 EdgeSAM参考HBM和TROS节点，模型权利边界仍以锁定仓库随附文件为准；
- D-Robotics RDK S100（KS1E55Y/S100E）官方板卡指南 v1.2：仅用于临时参考包络；实际目标为用户已连接的 RDK S100P V1P0，精确外形、孔位、质量与接口空间必须以实物测量锁定，不能沿用旧板尺寸；
  121 x 120 x 52.4 mm 外形包络、12 GB和80 TOPS型号信息；官方公开STEP仅作
  本地尺寸核查，因下载页未明确授予再分发许可，不复制或转换后提交。仓库内
  `s100_board_reference.stl`是严格限制于公开外形包络内的项目自建外观参考。
  来源：[官方板卡指南](https://github.com/D-Robotics/rdk_s_doc/blob/main/docs/01_Quick_start/01_hardware_introduction/01_rdk_s100/01_rdk_s100.md)、
  [官方下载页](https://github.com/D-Robotics/rdk_s_doc/blob/main/docs/01_Quick_start/download.md)；
- ROS 2 Jazzy、Gazebo Harmonic、Nav2、MoveIt2、PyTorch、ONNX Runtime
  等按各自上游许可证安装，不复制到最终 ZIP；
- D-Robotics OpenExplorer 3.7.0 的 2.85 GB 官方包不进入 Git 或最终 ZIP，
  仅记录官方来源、版本和 SHA；使用时需遵守厂商 SDK 条款。

当前清单中的未知许可证数量为 0。外部 mesh 不能因被复制进 Apache-2.0 包而被
重新许可；其原许可证继续生效。若未来加入外部模型、mesh、纹理、字体或数据集，
必须先登记来源、版本、许可证和 SHA，才能进入发布包。

## 高保真整车的厂商数据表边界

Arducam、Pololu、Actuonix、Jabsco/Xylem、ROBOTIS、Hokuyo、Livox、Intel、u-blox、
VectorNav、Universal Robots和D-Robotics页面目前只作为型号、尺寸、质量、
FOV、功率或性能参数来源。除非单个下载物另附明确允许修改和再分发的许可，
仓库不复制其网页图片、PDF、STEP/IGES或其他厂商资产。正式刷盘、刮吸、箱体、
支架和无开放网格的外壳将根据公开尺寸由项目重新建模，并记录修改与哈希。
