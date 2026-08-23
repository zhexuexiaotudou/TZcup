# 模型与资产许可证

## 项目模型

AUTO-04 与 AUTO-05 证据中的 ONNX 模型由本项目脚本在自建 Gazebo
合成数据上训练，没有下载或微调第三方预训练权重。模型随
`sanitation_learning` 包按 Apache-2.0 交付。其指标只适用于相应证据等级。

## Gazebo 与车辆资产

仓库中的车辆 URDF/Xacro、机械臂、障碍物、垃圾物体、材质、地图与世界均由
项目内参数化代码或文本资源构建，没有提交 Gazebo Fuel 模型、外部 mesh、
照片、音视频或其他来源不明的二进制资产。这些资源随所属
`sanitation_vehicle_description`、`sanitation_worlds`、`sanitation_navigation`
包按 Apache-2.0 交付。

## 外部依赖

- `linorobot2@b96aa42fbfa4390a77e0aab90935fe55d66d04ba`：Apache-2.0；
- `opennav_coverage@224118081c4c8de651f1db621053ab873b08f13d`：Apache-2.0；
- ROS 2 Jazzy、Gazebo Harmonic、Nav2、MoveIt2、PyTorch、ONNX Runtime
  等按各自上游许可证安装，不复制到最终 ZIP；
- D-Robotics OpenExplorer 3.7.0 的 2.85 GB 官方包不进入 Git 或最终 ZIP，
  仅记录官方来源、版本和 SHA；使用时需遵守厂商 SDK 条款。

当前清单中的未知许可证数量为 0。若未来加入外部模型、mesh、纹理、字体或
数据集，必须先登记来源、版本、许可证和 SHA，才能进入发布包。

## Journey 6 预训练候选

Journey 6 PC-first 路线登记了四个候选来源，但当前均未进入项目 release：
D1/C2 缺少可直接审计的 ONNX；D2/C1 已完成实际下载、SHA 与 ONNX 静态审计，
但 D2 产物类别合同与模型卡不符，C1 的 ONNX Ultralytics AGPL 元数据与模型卡
Apache-2.0 声明冲突，架构、训练代码、数据集与权重再分发条款尚未闭合。
Git 不携带这些权重。
详细边界见 `docs/journey6-license-boundary.md`。

Journey 6 OpenExplorer、HUCP/DNN Runtime、BSP、sysroot、AIBenchmark 与官方
sanity HBM 只使用板卡或官方 SDK 随附版本，不进入 Git 或项目 release ZIP；
RDK S100/S100P 包不能替代它们。
