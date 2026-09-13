# Day-1：20,000 m² 建图可行性核算

## 结论

当前工作树内没有任何 PGM/YAML 地图满足 `known_area_m2 = free + occupied >= 20,000 m²`。
当前实测短期探针仍为 `1402.175 m² / FAIL`。历史提交证明旧产品链曾在真实正式运行中越过面积门，
但那些运行没有通过完整 Mapping Gate，且原始 PGM/NPZ 不在本工作树内，不能把历史文档值直接算作
当前可复现地图。因此 Day-1 的准确结论是：

- **面积门可行性有历史证据**：不是物理上不可达。
- **当前分支可复现性为 FAIL**：没有当前工作树内达到 20,000 m² 的原始栅格。
- **正式结论仍为 NO-GO/PENDING**：不得把合成图、局部地图或历史面积数字宣称为当前 PASS。

## 离线核算工具

新增 [`scripts/verify_map_area.py`](../scripts/verify_map_area.py)，只读取 PGM/YAML，不启动
Gazebo、ROS 或 S100P。核算规则与 ROS 地图语义一致：

1. `mode` 必须为 `trinary`。
2. 像素 `205` 始终作为 unknown，不计入面积。
3. `occupied` 和 `free` 由 `occupied_thresh`、`free_thresh` 和 `negate` 判定。
4. `known_cells = occupied_cells + free_cells`。
5. `known_area_m2 = known_cells * resolution^2`。
6. 输出同时记录 YAML/PGM SHA-256、尺寸、各类格数、面积、缺口和达到门限所需各向同性线性缩放。

当前工作树的精确输出保存在
[`reports/mapping/day1_map_area_verification.json`](../reports/mapping/day1_map_area_verification.json)：

```powershell
py -3 scripts/verify_map_area.py `
  --root . `
  --minimum-area-m2 20000 `
  --output reports/mapping/day1_map_area_verification.json
```

门禁模式使用 `--require-pass`；没有任何地图达到门限时返回退出码 `2`。

## 当前工作树地图

| PGM/YAML | 分辨率 | 栅格尺寸 | free + occupied | known area | 判定 |
|---|---:|---:|---:|---:|---|
| `starter_ws/src/sanitation_navigation/maps/sanitation_test_map.yaml` | 1.0 m | 40×25 | 1000 | 1000.0 m² | 合成导航测试夹具，不计建图 |
| `starter_ws/src/sanitation_navigation/maps/stage4v_surveyed_reference.yaml` | 0.02 m | 1661×829 | 1,376,969 | 550.7876 m² | SDF 可见几何生成的定位参考；`mapping_evidence=false` |
| `artifacts/stage4t_20260715_review/selected_map.yaml` | 0.05 m | 662×325 | 61,349 | 153.37250000000003 m² | 当前工作树内真实保存的 Stage4T SLAM 地图 |

两个 `stage4v_filters/*.yaml` 为 `mode: scale` 的 keepout/speed mask，已由工具明确排除，不能把其中
的自由或占用像素当作建图面积。

因此，当前工作树内**可独立复算的真实 SLAM 地图**中，最大的是 Stage4T
`artifacts/stage4t_20260715_review/selected_map.pgm`，面积为
`153.37250000000003 m²`，缺口 `19846.6275 m²`。

## 历史提交中的精确地图

所有 Git ref 中共找到两个已删除历史路径下的真实 SLAM PGM/YAML 对；使用同一工具离线提取后得到：

| 来源提交 | 地图路径 | 分辨率 | 栅格尺寸 | free + occupied | known area | SHA-256（PGM） |
|---|---|---:|---:|---:|---:|---|
| `413b6ebfb16d40e00a820c1dcf8cb5c87c90e566` | `artifacts/stage4r_20260715_093931/stage4r_slam_map.yaml` | 0.05 m | 817×663 | 177,127 | 442.8175000000001 m² | `9dc9ce1855e5063f3340db01e7e0d7b7834405d6be75c160534cf3e7480b1ab2` |
| `7b3806f1c0710d1065663188940513735cea1ae2` | `artifacts/stage3_20260714_172155/slam_map.yaml` | 0.05 m | 194×64 | 20 | 0.05000000000000001 m² | `547bdba570073cf566bc53270a987b44086e57b31ca7b0e8f27c8d27a0b28aae` |

历史提交中最接近 20,000 m² 的**已提交真实栅格**是 Stage4R 的 `442.8175000000001 m²`，仍需
`19557.1825 m²`，各向同性线性放大 `6.720515854336513` 倍。

## 历史 20,000 m² 运行证据与边界

### 20,075.20 m² 正式尝试

提交 [`a400781048f571f90572213d8cb25fdd13803f93`](https://github.com/zhexuexiaotudou/TZcup/commit/a400781048f571f90572213d8cb25fdd13803f93)
（`fix(mapping): require dense in-bounds observation`）的记录说明，产品链 seed `2028` 的真实正式运行达到：

- 总已知面积 `20,075.20 m²`。
- 边界包络覆盖率 `100%`。
- 导航 `511/515` 成功。
- 地图/位姿图保存、全新 Gazebo 重启、重定位和 `3/3` Nav2 航点完成。
- 可见边界召回 `0.93628 < 0.95`。
- 目标矩形内部已知覆盖率 `92.35% < 95%`。
- 主要漏边为 `building_north_2`。

该地图**满足面积数字**，但未满足完整 Mapping Gate，因此只能记录为
`AREA_ONLY_HISTORICAL_PASS / MAPPING_GATE_FAIL`，不能写成当前地图通过。

### 20,011 m² 旧运行

同一提交的 README 记录另一次无诊断覆盖建图达到原始 `20,011 m²`，保存栅格复算
`20,017.68 m²`，可见边界召回 `0.9598`。该次运行因旧 runner 的僵尸进程误判提前终止，
没有形成正式 PASS；其地图也没有作为原始 PGM/NPZ 进入当前工作树。

### 为什么不能把历史数字当作当前原始证据

- `a400781` 不是当前 `f1f2282` 的祖先；相关地图由临时运行目录和旧产品链产生。
- 当前 Git 全历史只包含 Stage3、Stage4R、Stage4T 和测试/参考图，不包含 `20,075.20 m²`
  run 的 PGM/NPZ。
- `docs/competition-mapping-probe.md` 明确说明 `1402.175 m²` 探针的 NPZ/PGM 由实际地图生成，
  但该证据包不在本工作树，不能在隔离范围内重新逐格核算。

## 当前 1402.175 m² 探针的放大与运行时间

从 `1402.175 m²` 到 `20,000 m²`：

| 指标 | 精确值 |
|---|---:|
| 面积缺口 | `18597.825 m²` |
| 面积倍率 | `14.263554834453616` |
| 各向同性线性尺度 | `3.7767121725720134` |
| 实测 RTF（封存探针） | `0.124420` |
| 后 120 s 增长率外推剩余墙钟时间 | `107.43977469670712 min` |
| 后 60 s 增长率外推剩余墙钟时间 | `226.25091240875912 min` |
| 早段平均增长率外推剩余墙钟时间 | `59.20988538681948 min` |
| 配置期望 20–35 仿真分钟对应墙钟时间 | `160.74586079408454–281.305256389648 min` |

现有候选路线 `source_world_route_m` 长度 `436.24880949681335 m`；按 `0.45 m/s` 和实测 RTF
仅理想运动就需要约 `16.15736331469679` 仿真分钟，即约 `129.86146370918496` 墙钟分钟。
`29 m` 扫描半径下的无遮挡理论下界路线长度为 `299.27449272984455 m`，但该下界不包含遮挡、
自滤波、frontier 失败和定位问题，不能作为完成证明。

上述时间都是线性估计，不是完成保证。探针后段增长已从早段约 `5.235 m²/s` 降到后 120 s
`2.885 m²/s`、后 60 s `1.370 m²/s`，说明地图出现了明显的收益递减。

## 当前 blocker

1. 当前工作树没有 20,000 m² 级原始 PGM/YAML，不能离线证明面积门。
2. 5 分钟探针只到 `1402.175 m²`，且证据包与 PGM 不在本工作树。
3. 面积增长持续减速，不能用早段平均增长率作为完成承诺。
4. 历史 20,075.20 m² 运行失败于 `building_north_2` 边界召回和矩形内部已知覆盖率，说明
   “面积超过 20,000”不足以证明完整建图完成。
5. 当前没有授权启动 Gazebo/ROS/S100P，因此不能在本任务内产生新的正式长跑。

## 最短有效下一次运行

比当前 5 分钟探针更短的运行不能建立正式面积门。下一次有效动作必须是完整 first-map
lifecycle，使用新鲜 episode、冻结 runtime workspace 和新鲜输出根目录，完成保存后立即用本工具
离线核算：

```bash
export FORMAL_VEHICLE_RUNTIME_WS=/path/to/frozen/runtime-ws
export FORMAL_DYNAMIC_EPISODE_ROOT=/path/to/fresh/formal/episode
export FORMAL_DYNAMIC_SAVED_MAP_ROOT=/path/to/fresh/saved-map-output
export FORMAL_MAPPING_TIMEOUT_S=21600

bash scripts/run_formal_first_map_dynamic_prerequisite.sh

python3 scripts/verify_map_area.py \
  --root "${FORMAL_DYNAMIC_SAVED_MAP_ROOT}" \
  --map-yaml "${FORMAL_DYNAMIC_SAVED_MAP_ROOT}/occupancy.yaml" \
  --minimum-area-m2 20000 \
  --require-pass
```

通过该命令才算面积门；完整 Mapping Gate 还必须同时满足边界包络 `1.0`、目标矩形内部已知覆盖率
`>= 0.95` 和拓扑召回 `>= 0.95`。
