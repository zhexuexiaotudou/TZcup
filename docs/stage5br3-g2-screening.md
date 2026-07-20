# Stage5BR3：G2 真实车辆数据与 split-model screening

## 结论

Stage5BR3 已把 G2 相机链从独立静态训练 rig 修正为实际 `sanitation_vehicle` 的生产相机外参与光学帧。六个不同材料、几何和布局的 Gazebo Harmonic 世界按 3/1/2 分配给 train/val/test，逐世界运行时契约与生产 GT 隔离通过。80 scene/800 frame 原生数据在第一次 QA 失败后完成修复、全量重采和复核；四档离线分辨率扫描完成；detector 与 area segmenter 在 512×384 执行三次 architecture screening 后仍未过门，因此严格停止。

当前第一个阻断层为 `G2_split_model_screening_gates_failed_after_3_attempts`。`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。这轮没有启动 500 scene/5000 frame 正式集、30-seed/10-min live、30 次真实 Nav2 spot-clean、真实域评估或 J6 工具链。

## 运行时相机契约

- 训练 GT 传感器只有显式 `enable_training_gt:=true` 时才挂载在真实车辆 `camera_link`；生产默认 Xacro 不包含 semantic/instance 传感器。
- 六世界实际接收非空 640×480 RGB、`32FC1` 深度、semantic、instance、CameraInfo；四图像消息使用完全相同时间戳和 `camera_depth_link` 光学帧。
- `base_link` 到相机外参为 `[0.53, 0, 0.22] m`；车辆在两秒观测窗内实际移动约 0.70 m。
- 每个世界使用独立容器、ROS domain 和 Gazebo/bridge/collector 进程，报告绑定最终 world SHA，避免沿用旧进程或旧话题。
- 生产运行时 topic 审计中 semantic/instance GT 为 0，控制侧 GT 订阅为 0。

## 数据与 QA

原生数据保留在本机 `F:/Project/TZcup-stage5br3-data/g2_screening_native`，约 2.225 GB，不放入 Git 复核包。首次完整 QA 揭示 12 个 hard-negative 资产跨 split 复用且没有 negative-only 场景；失败报告被原样保留。随后将 hard negatives 固定拆为 8/2/2，强制 5 个 negative-only seed，并从头重采 80/800。

最终 QA：80 scene、800 frame、标注完整率 100%；target/negative/trajectory leakage 均为 0；跨 split exact/pHash duplicate 均为 0；semantic-instance 错误率为 0；每场景相邻帧位移满足 0.25 m 门；hard-negative 数量覆盖 0–8。

场景清单记录曝光、白平衡、噪声、模糊和动态障碍请求；本轮可直接证明的是不同世界的原生灯光、材质、几何、布局和车辆运动。没有把“请求参数”冒充已在 Gazebo 原生图像上逐项施加的证据，这是后续数据工程仍需补强的边界。

## 分辨率与模型筛选

一次原生采集后离线评估 256×192、384×288、512×384、640×384。根据离散目标短边、bbox 和 mask-area 分位数选择 640×384 与 512×384，模型实跑固定为 512×384。

三次尝试均为独立随机初始化的 split models：离散类 detector 输出置信度排序 bbox 并计算 AP50/AP50:95/F1/small recall/negative-only FP；leaf/puddle 使用 area segmenter 并计算 foreground mIoU。最佳观测值为 detector cross-world F1 `0.1311`、AP50 `0.3484`、AP50:95 `0.1075`、small recall `0.4512`、color-stress F1 `0.1018`，最低 same-color negative FP `8.7/frame`，area cross-world mIoU `0.02346`。这些值远未同时达到固定门限，第三次后停止。

每次报告同时保存训练曲线、模型参数量、固定 shape ONNX、算子清单、模型字节、CPU latency，以及 GPU 可用时的训练峰值显存。测试 split 没有参与选择。

## 复现入口

- 世界生成与静态契约：`py -m pytest starter_ws/src/sanitation_learning/test/test_gazebo_g2.py`
- 六世界运行时：`bash scripts/stage5br3_runtime_contract.sh`
- 单场景采集：`bash scripts/stage5br3_capture_world.sh ...`
- QA：`py scripts/stage5br3_finalize_dataset.py ...`
- 分辨率扫描：`py scripts/stage5br3_resolution_scan.py ...`
- 模型筛选：`py scripts/stage5br3_train_split_models.py ... --attempt 1|2|3`
- 生产隔离：`bash scripts/stage5br3_production_isolation.sh`

紧凑证据位于 `artifacts/stage5br3_20260720_review/`。原始数据、probe 数据、失败采集和工作区在用户确认前保留。
