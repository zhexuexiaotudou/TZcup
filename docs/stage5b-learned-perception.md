# Stage5B 学习型感知复现与失败边界

## 结论

本轮完成了可训练、可导出 ONNX、可接入 ROS/Gazebo 的学习型感知骨架，但没有通过 Stage5B。失败不是工具链无法运行，而是 D1 数据域不满足“Gazebo camera 实际渲染”，且模型在未见测试和颜色压力下严重失效。按规划包停机条件，正式 500 seed/5000 帧、30 seed/10 分钟实时门和 30 次真实 Nav2 spot-clean 均未执行。

## 数据与模型

`sanitation_learning/config/asset_registry.yaml` 定义五个目标类别、每类六个程序化变体和 12 个硬负样本；调色板跨类复用，train/val/test 分别使用变体 0–3、4、5，并隔离 world 与 texture。数据生成保存 RGB、depth、semantic、instance、map pose、COCO、split hash 与标注 QA，但 manifest 明确写入 `gazebo_camera_rendered=false`。

训练固定 seed 2051。候选 A 是 24 参数的已训练像素基线；候选 B 是 137,078 参数的上下文卷积网络；候选 C 因 ONNX/J6 风险 deferred。候选 B 由验证集选出，测试集未参与选择，导出模型 SHA-256 为 `6858863df0588f33083779f15c87a37e7f280d06d8241c37363277d4c9ecc328`。

## 三次筛查与冻结原因

第一次筛查的 test 离散 macro F1 约 `0.00185`；第二次加入更大感受野、CE+Dice 和通道/边缘增强后为 `0.0166`；第三次使用 RF51 上下文网络与形态学后，验证 macro F1 达 `0.38637`，但 test 离散 macro P/R/F1 下降到 `0.00752/0.00784/0.00768`。颜色压力 aggregate macro F1 为 `0.05192`，远低于 `0.85`；leaf/puddle IoU 也仅 `0.00376/0.2494`。继续围绕程序化生成器调参只会增加对错误域的拟合风险，因此在第三次结构性尝试后冻结。

评测没有保留可形成 precision-recall 曲线的置信度排序检测，因此 AP 字段明确为 null；`instance_match_score_at_iou50` 和 `mean_instance_match_score_iou50_95` 只是 IoU 匹配 Jaccard 分数。

## 运行与回归

诊断运行采用 `stage5b_perception.launch.py`，模型经 ONNX Runtime 处理真实 Gazebo RGB-D/TF 161 帧，发布分割与 map targets，`ground_truth_input_used=false`。该结果只证明数据通路和运行接口可达，不证明分类正确；正式实时门保持 false。

Stage5A 固定颜色基线全量回归通过；Stage4W seed 0 完整任务 17/17、经验覆盖率 94.2%、碰撞和 keepout 违规 0；快速测试 57 项通过。由此可将失败定位在 Stage5B 数据/学习泛化层，而非上一阶段主线回归。

## 复现

```powershell
py scripts/ci_fast.py
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5b_docker.ps1 -OutputName stage5b_screening
py scripts/stage5b_finalize.py
```

`-FormalDataset` 只扩大当前程序化数据，不能把它升级为 Gazebo-camera D1；在真正 renderer 完成且筛查门通过前不应运行它来宣称正式门。

## 下一步最小闭环

先实现 Gazebo 内逐场景 spawn、相机稳定/同步采集、semantic/instance 真值和可重放 manifest 的 D1 pipeline，再以冻结 test world/mesh/texture/seed 重训和评测。只有颜色压力、离散类与区域类门全部通过，才恢复 30 seed/10 分钟实时与真实 Nav2 spot-clean。D2、J6 和效率继续独立 fail-closed。
