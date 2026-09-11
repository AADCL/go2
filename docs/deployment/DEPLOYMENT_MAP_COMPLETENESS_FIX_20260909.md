# 机器狗 1 地图完整度修复部署记录

部署日期：2026-09-09
设备：机器狗 1，`192.168.50.110`
工作空间：`/home/nvidia/go2_nav_ws`
修复地图：`lab_202609091010`

## 1. 结论

本次问题不是雷达或 FAST-LIO 掉线，也不是 PCD/PGM 文件损坏。问题由两级过度
筛选叠加造成：

1. 动态建图对 5 cm 细体素沿用过长的静态确认窗口，大量真实墙面/地面点因
   MID360 的正常端点抖动而一直停留在候选态或被清除。
2. 地形导出直接用稀疏的重建地面生成二维地图，没有完整保留已经验证的传统
   occupancy 投影基底。
3. 地形完整度用四邻域统计稀疏激光采样，同一连续地面中仅对角相邻的采样被错误
   分成多个组件。
4. 已行走自由证据只采用地面追踪成功的轨迹段，实际走过但暂时缺少地面回波的区域
   仍被保留为 unknown。

修复后，正式地图已知覆盖由 `2,218/62,220 = 3.56%` 恢复为
`18,803/62,220 = 30.22%`，且全部 `18,609` 个传统 occupancy 基线已知格均被
保留。源 `public_map.pcd`、轨迹和外参未修改。

## 2. 动态建图修复

`go2_map_builder` 继续使用 20 cm 父体素进行保守静态确认，但 5 cm 子体素使用独立
短确认窗，以容忍约 2-3 cm 的激光端点抖动：

```yaml
hit_probability: 0.65
miss_probability: 0.40
min_hit_scans: 12
min_observation_span: 2.0
min_hit_ratio: 0.60
fine_min_hit_scans: 6
fine_min_observation_span: 0.50
fine_min_hit_ratio: 0.50
min_candidate_clear_miss_scans: 4
min_candidate_clear_miss_span: 0.30
min_clear_miss_scans: 12
min_clear_miss_span: 1.20
ray_endpoint_margin: 0.25
candidate_timeout: 6.0
```

细体素自身确认后立即缓存代表点，但只有同 generation 的父体素确认后才能进入
`/static_scan` 和最终 PCD。父体素后确认时会标记地图 dirty，避免此前缓存的真实
细点永远无法进入快照。动态点仍需经过父体素门限，动态障碍过滤功能没有取消。

## 3. 二维地图与地形导出修复

`run_go2 export-map <map_name>` 对新地形地图改为两阶段事务：

1. 传统 occupancy exporter 先写隐藏的 PGM/YAML 基线。
2. 地形 exporter 读取该基线，只在全部地形资产、尺寸、参数和 SHA256 校验通过后
   原子提交正式地图。

失败时正式 `map.pgm`/`map.yaml` 不变。PGM/YAML 自身也使用成对提交和失败回滚。

地形质量统计改为八邻域采样连通，仅用于非修改式完整度检测；主地面传播仍为四
邻域，原有受高度约束的补洞逻辑不变，不会借对角线绕过台阶。`60%` 门限没有降低。

完整、连续记录的机器人轨迹作为直接实走证据，可将 unknown 补为 free；它不能
清除基线 occupied，地形障碍最后覆盖。相邻轨迹间隔大于 `0.50 m` 时不会连接，
地面高程和坡度仍只来自通过 PMF、连续性和局部平面校验的地面。

## 4. 当前地图重放结果

输入：

```text
public_map.pcd:        73,321 points
traversed_path_map:    189 points
trajectory max gap:    0.109 m
map geometry:          244 x 255 @ 0.05 m
base_link-to-floor:    0.335 m
```

严格生产参数结果：

```text
trajectory trace:                 175/189 = 92.6%    required >= 80%
PMF observation reconstruction:             31.8%    required >= 30%
largest 8-connected ground component:       69.3%    required >= 60%
ground / legacy baseline-free coverage:     12.6%    required >= 10%
legacy known cells preserved:       18,609/18,609
driven corridor known coverage:       1,939/1,939 = 100%
reconstructed ground cells:                    1,893
terrain obstacle cells:                           212
terrain asset checksums:                           15
```

二维地图变化：

| 指标 | 修复前 | 修复后 |
|---|---:|---:|
| occupied | 473 | 3,957 |
| free | 1,745 | 14,846 |
| unknown | 60,002 | 43,417 |
| known coverage | 3.56% | 30.22% |
| 最大八连通 known 区占比 | 73.13% | 87.43% |

正式地图 export id：`repair-formal-terrain-20260909-01`。

## 5. 编译与测试

```text
go2_mapping + go2_terrain build: PASS
mapping unit tests:              25 PASS
terrain model tests:             44 PASS
terrain algorithm tests:         31 PASS
terrain validator tests:         13 PASS
catkin_test_results:              0 errors, 0 failures, 0 skipped
validate_workspace.sh:            PASS
isolated full-map export:         PASS
formal full-map transaction:      PASS
```

编译中仍会显示系统 PCL/VTK 可选工具缺失警告；它们不属于本次修改，未造成编译或
测试失败。

## 6. 当前运行状态

部署期间未发送速度、目标或 Enable 请求，也未重启 ROS。检查结束时：

```text
/go2/control/enabled: False
navigation PID:        5658
navigation start:      2026-09-09 10:18:20 CST
active ROS master:     11311 only
temporary export files: none
```

当前进程已经把旧地图加载到内存，因此 RViz 在这一次运行中不会自动变成修复后的
地图。正常停止当前导航并重新启动后，才会加载新的磁盘资产。

## 7. 下次验证步骤

1. 在当前导航终端按 `Ctrl+C` 正常退出，不要直接断电。
2. 保持机器狗 Disable，重新启动：

   ```bash
   RVIZ=true run_go2 navigation lab_202609091010 --real
   ```

3. 在 RViz 中确认 Fixed Frame 为 `map`，检查完整二维边界、三维地图、TF 和全局
   坡度层；此时先不要 Enable。
4. 确认定位稳定、`/localization/ok=true`、地形健康后，再由现场人员执行：

   ```bash
   run_go2 enable
   ```

5. 后续新建地图使用原命令即可。重点观察最终 `public_map.pcd` 点数和导出日志中的
   五项质量指标；任何指标不达标都会保留上一版正式地图。

## 8. 备份与回滚

主备份：

```text
/home/nvidia/go2_nav_ws/backups/codex_20260909_map_completeness_fix/
/home/nvidia/go2_nav_ws/backups/codex_20260909_terrain_connectivity_fix/
```

原始问题地图完整备份：

```text
/home/nvidia/go2_nav_ws/backups/codex_20260909_terrain_connectivity_fix/formal_map_before_repair.tgz
SHA256: 2502da0af7e7cd415c1ab04a1fdb584fb51d344d687e56bb766e7a34c486cfff
```

回滚必须先正常停止 mapping/navigation，再恢复对应归档并重新编译受影响包。不要在
运行中的 map_server、terrain layer 或 map builder 上覆盖回滚文件。

## 9. 已知边界

- 当前 PCD 已在旧参数下生成，只有 73,321 点；修复无法恢复已经被旧动态过滤器删除
  的三维点。二维导航地图已通过保守基线恢复，未来新建地图才会体现新的点保留参数。
- 全部轨迹作为 free 证据后，`minimum_trajectory_corridor_known_ratio` 主要成为合并后
  一致性检查；核心地形质量仍由轨迹追踪率、PMF 观测率、地面连通率和基线自由区
  覆盖率约束。
- `>0.50 m` 的轨迹跳变不会画连接段，但跳变两端仍保留点位圆盘。当前地图无此类
  跳变；后续可将该条件提升为显式导出失败规则并补充边界单元测试。
