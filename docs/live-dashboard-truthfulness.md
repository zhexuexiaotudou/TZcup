# 全流程看板：过期、断联与组件终态

此补丁依赖 PR #169 的 `demo.html` 与 `LiveMissionState`，基线为
`132d4024d473aa085e6a1f94224f1f255f59485b`。它不替代经典监督台 `/` 的轨迹回放。

`/final_demo/state` 过期后，即使最后一次消息包含
`formal_product_acceptance=true`，页面也必须显示“当前验收未知”，
并把阶段标注为“最后观测”。HTTP 失联会撤销当前速度、位姿、刷盘、安全、
进度、图像和路径；保留的阶段身份不能作为当前状态或验收证明。
重新收到遥测后才恢复显示。

覆盖任务组件的 `FAILED`、`CANCELED` 分别计入
`progress.failed_components`、`progress.canceled_components`，不计成功完成。
重复终态和两个 ROS 状态话题的到达顺序不应重复计数。

专项命令：

```powershell
$env:PYTHONPATH = "$PWD/starter_ws/src/sanitation_hmi"
py -3 -m pytest starter_ws/src/sanitation_hmi/test/test_live_state.py
node --test starter_ws/src/sanitation_hmi/test/test_demo_disconnect.cjs
```

本任务使用独立静态 HTTP 页面与显式 UI 夹具验证真实浏览器渲染，
包括 1920×1080、390×844、过期验收 true、HTTP 503、恢复。
这些截图不是 ROS / Gazebo / 正式 session 证据。
实际运行入口继续使用 PR #169 的既有启动流程；不得为截图重启共享仿真。

正式交付仍要求 PR #169 先进入 main，本补丁随后重定向到 main 并重验 CI，
再经已批准的 HMI overlay 部署与真实遥测复验。当前未获得共享运行窗口，
真实运行门保持 blocked，不能仅将补丁并入任务分支就声称已部署。
