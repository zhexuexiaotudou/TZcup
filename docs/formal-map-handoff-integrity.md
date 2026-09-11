# 首次建图到独立 AMCL 清扫的交接校验

本页描述产品交接的源码门。离线回归通过不代表真实 SLAM 探索、AMCL 重定位或全覆盖运行通过；正式验收仍需当前 source/session 下的完整运行证据。

## 交接链

`run_formal_first_map_dynamic_prerequisite.sh` 在建图运行通过后写入 `mapping_handoff_record.json`，退出清理完成后再记录进程组停止与清理时间。交接记录绑定地图 manifest、mapping runtime 和 `runtime_gate_binding.json` 三份文件的 SHA-256。

`run_formal_saved_map_cleaning_lifecycle.sh` 在启动 Gazebo/AMCL 之前校验保存地图及交接，并比较建图和清扫的 session/source/runtime closure 身份。只有 mapping preflight、完成、清理和 cleaning preflight 的时间顺序有效，且记录的 mapping 进程已经停止，才启动独立清扫进程。

`hard_restart_record_valid` 读取交接原文，要求 schema、完成状态、成功退出、进程停止、PID 和时间字段有效，再核对清扫记录继承的字段及全部哈希。PID/计数不能以布尔值冒充整数；时间必须含时区。缺失或损坏记录返回拒绝。此函数证明证据一致性，真实运行身份由现有 runtime binding 校验负责。

最终 `validate_formal_map_lifecycle_runtime.py` 重验交接文件与绑定，不能仅凭 `hard_restart_verified=true` 接受已经漂移的产物。旧记录缺少新的 binding 哈希时应重新运行建图与交接；不得补写历史记录来声称当前会话通过。

## 地图稳定观测

同一地图消息重复执行定时评估不能算作多次观测。稳定计数必须来自不同、时间戳递增的有效地图，地图坐标系为 `map`，占据数据及原点有效，且地图、里程计和 GNSS 输入保持新鲜。观测率下限为 95%，连续稳定样本至少为 3；参数不得把这两项门槛调低。

## 可执行验收

在已安装依赖的项目根目录执行：

```powershell
$env:PYTHONPATH = 'starter_ws/src/sanitation_formal_campus_integration;scripts'
py -3 -m pytest starter_ws/src/sanitation_formal_campus_integration/test/test_map_handoff_integrity.py starter_ws/src/sanitation_formal_campus_integration/test/test_map_lifecycle_freshness.py scripts/test_validate_formal_map_lifecycle_runtime.py -q
py -3 scripts/ci_fast.py
```

真实运行产物的最终验收沿用既有入口：

```bash
python3 scripts/validate_formal_map_lifecycle_runtime.py \
  --map-root "$MAP_ROOT" \
  --mapping-runtime "$MAP_ROOT/mapping_runtime.json" \
  --cleaning-runtime "$CLEANING_RUNTIME" \
  --runtime-binding "$CLEANING_BINDING" \
  --output "$NEW_ACCEPTANCE_OUTPUT"
```

输出路径必须是本轮保留证据路径。当前正式运行资源由全场验收任务统一协调，不得并发启动另一套 Gazebo 或复用旧会话作为通过证据。运行前先完成对应 Stage 3 构建/运行门，保留实际地图、话题、TF、进程退出及硬重启记录；失败证据与回滚版本同样保留。
