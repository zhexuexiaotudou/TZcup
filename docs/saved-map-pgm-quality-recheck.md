# 保存地图 PGM 质量复核

固化地图的 `observed_fraction` 不再是自行声明的准入依据。建图保存完成后，系统按保存的 `occupancy.yaml` 与 `occupancy.pgm` 重建三值占据栅格，并在正式 geofence 内重新计算已观测单元数、总单元数和观测率。

仅接受二进制 `P5` PGM、`trinary` 模式、零 yaw 原点、`0 < resolution <= 0.10 m`、合法 `free_thresh < occupied_thresh` 和 `negate` 为 `0` 或 `1`。等于两侧阈值的像素按 unknown 处理；unknown 不计入观测率。保存产生的 manifest 必须同时记录重算的比例、已观测单元数和场地单元数，并与再次读取的 PGM 完全一致。

清扫启动入口和最终 map-lifecycle 聚合器都调用同一保存地图验证函数。修改 PGM、YAML、几何或 manifest 后即使重新计算文件哈希，也不能把旧观测率用于通过 95% 门。

离线验证：

```powershell
$env:PYTHONPATH = 'starter_ws/src/sanitation_formal_campus_integration'
py -3 -m pytest starter_ws/src/sanitation_formal_campus_integration/test/test_map_lifecycle_core.py scripts/test_validate_formal_map_lifecycle_runtime.py -q
```

该检查只验证保存产物和准入逻辑。它不替代 fresh 200 m × 100 m SLAM 建图、独立硬重启 AMCL 清扫或 Stage 3 的真实运行证据。
