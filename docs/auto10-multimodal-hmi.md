# AUTO-10 多模态任务入口

AUTO-10 把用户输入与机器人执行面隔开。APP 和语音只产生受限任务 DSL，HTTP 网关本身不发布 ROS 运动话题，也不直接调用关节或电机。

## 数据流

1. 本地 APP 通过 bearer token 和 idempotency key 提交单个 `command`。
2. 网关完成鉴权、角色授权、请求 schema 和重复提交校验。
3. 语言解析器把中英文命令转换为固定字段 DSL。
4. DSL validator 确认所有工具属于 allowlist，且 `direct_actuator_access=false`。
5. 本阶段只返回 `execution_dispatched=false` 的已验证任务；后续集成必须再经过任务编排器和安全监督器。

允许的工具为 coverage、spot-clean、schedule、pause、resume、return-home、status 和 emergency-stop。包含 `/cmd_vel`、joint、motor、电机或关节等直接执行意图的输入立即拒绝。

## 正式评测

- APP/API/UI：启动真实本地 HTTP 服务并发送 270 个合法/非法请求，再覆盖鉴权、授权、幂等冲突和 13 个 UI contract，共 288 cases；
- speech：Windows System.Speech 生成 500 个 PCM16 单声道样本，覆盖三音色、三语速、中英文，再施加四档噪声和 dry/short-room/long-room 三类混响；`faster-whisper small` 在 CUDA 12.4.1 cuDNN 容器内转写，转写文本仅通过有限域命令词典做 transcript-only 归一化；
- DSL：1200 个 normal、synonym、bilingual、ASR-noisy、missing、conflict 和 unsafe cases；预期标签由静态 case generator 固定，不从被测输出反推。

正式结果是 APP P95 `16.03 ms`、speech intent accuracy `0.9911` / P95 `171.94 ms`、DSL 三项准确率 `1.0`，危险命令拒绝和歧义 fail-closed 均满足门槛。逐 case/audio 原始数据不提交 Git，紧凑证据中的 `raw_metric_index.json` 记录外部路径、大小和 SHA。

## 边界

本门证明本机任务入口、机器合成语音链路和受限 DSL 的行为，不证明真实环境 ASR、云端生成式 LLM、车辆任务执行、真实域感知或 J6 板端性能。浏览器 QA 中发现并修复的问题保留在 attempt ledger，不把最初渲染失败写成通过。
