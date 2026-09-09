# 机器狗 1 动态建图与坡度规划部署报告

## 1. 部署信息

- 设备：机器狗 1，`nvidia@192.168.50.110`
- 工作空间：`/home/nvidia/go2_nav_ws`
- 完成时间：`2026-09-08T20:38:31Z`（北京时间 2026-09-09）
- 参考基线：WheelTech V5.1，提交 `4a240019f3776e82f0de1b55029339d056e03bcc`
- Patchwork++ ROS1：提交 `f8c070bf2774b2f3ef622644a511bdfe3f2f27bb`
- Patchwork++ 许可证：GPL-3.0；Go2 新增包保留各自许可证声明
- 最终部署归档 SHA256：
  `258e39cd3b8e93ce33622b57aee763fdd3dad219c2c38e1293e39ec4563ba305`

系统新增依赖：

```text
ros-noetic-jsk-recognition-msgs 1.2.19-1focal.20250520.060210
```

## 2. 已完成范围

- `go2_map_builder` 已改为时间同步的六自由度射线原点、三维 DDA、
  贝叶斯静态晋升/动态清除、粗细体素 generation 绑定、轨迹输出和 30 秒异步快照。
- 新增 `go2_terrain`，提供离线地面重建、天花板排除、六层 2.5D 地形资产、
  校验和、全局 `Go2TerrainLayer` 和实时地面/普通障碍分割。
- 全局 costmap 顺序为 `StaticLayer -> Go2TerrainLayer -> InflationLayer`；
  局部 costmap 只有 `ObstacleLayer -> InflationLayer`，不加载坡度层。
- 坡度 `<=8 deg` 零附加代价，`8-30 deg` 为 15-80 软代价，连续至少四格
  `>30 deg` 为全局致命代价，横向扩展 `0.20 m`。
- 运行时 Terrain Guard 使用同一个近场连通地面分量完成覆盖率、坡面兜底、
  高度拟合和 clearing，禁止不同地面块拼接健康证据。
- Terrain Guard 需要连续 5 个健康帧开门；只对几何缺帧容忍最多 3 帧且不超过
  `0.25 s`。点云超时、低频、处理超时、拟合失败和离地高度越界立即关闭。
- `go2_navigation_supervisor` 独占公开 `/move_base` Action 和简单目标入口；
  未就绪目标不会进入内部 move_base。地形失效和控制失效会取消目标并输出零速度。
- 新旧地图导出均使用本次唯一 export ID。新地形地图由元数据和 SHA256 提交；
  历史二维地图最后原子写入 `.go2_legacy_export_receipt`。
- 新地图只有完整且校验通过的 `terrain_2p5d.yaml` 才启用新链；历史地图保持
  legacy 行为。声明新格式但资产损坏时拒绝启动，不能静默降级。

## 3. 编译与自动测试

```text
catkin_make -j1: PASS
validate_workspace.sh: PASS
catkin_make run_tests -j1: PASS
Summary: 183 tests, 0 errors, 0 failures, 0 skipped
```

覆盖范围包括 DDA、外参射线原点、里程计插值、贝叶斯晋升/清除、体素 generation、
快照事务、地面连通、墙/台阶/天花板排除、10/20/30/35 度坡、坡度代价、
TerrainLayer 合并、物理高度门、双地面分量隔离和健康迟滞。

构建日志中的 VTK 可选程序和 PCL 可选 I/O 提示为系统已有警告；所有生产目标均已
成功链接，不影响本部署所用功能。

## 4. 无运动实机验证

全过程没有调用 Enable、发布 Goal 或发送速度命令，也没有启动真实 SDK bridge。

### 4.1 实时感知

35 秒采样、排除 5 秒预热后：

```text
地形帧：300
状态样本：450
点云输出频率中位数：10.002 Hz
处理延迟：1.62-5.53 ms，中位数 4.60 ms
连通地面面积：2.05-3.33 m2
近场支撑面积：0.88-1.13 m2
覆盖扇区：4/8
回调错误：0
```

机器狗测试时处于低姿态。MID360 估计离地 `0.251-0.271 m`，低于允许的
`0.43-0.59 m`；450/450 个稳定状态均为
`sensor_height_below_minimum`，`/terrain/healthy` 全程 False，分类为
`low_posture_fail_closed`。这证明低姿态不会误放行；标准站立姿态仍需现场复验。

### 4.2 地图导出

合成平地正例：

```text
181 x 141 @ 0.05 m
base_link 到地面：0.350 m
export_id：synthetic-final-20260909
15 个资产 SHA256：全部通过
```

历史导出正例回执为 `legacy-final-20260909`。缺失 PCD 时底层节点 code 1，
不生成回执；正式 `run_go2 export-map` 将 ROS1 的 roslaunch 零状态误差转换为
退出码 5，不能把旧文件误报为新结果。

### 4.3 启动兼容性

```text
新地形地图 mock 导航：PASS，control_enabled=false
历史地图 lab_202609081650 mock 导航：PASS，control_enabled=False
```

新模式确认全局层包含 TerrainLayer、局部层不包含 TerrainLayer、真实 SDK 未运行；
legacy 模式确认 Patchwork++、Terrain Guard 和 TerrainLayer 均不加载。

## 5. 保持不变项

网络：

```text
eth0  192.168.123.99/24  Go2 底盘
eth1  192.168.1.50/24    MID360
wlan0 192.168.50.110/24  管理网络
```

参数：

```text
地图导出 obstacle_inflation_m: 0.03 m
全局 costmap inflation_radius: 0.10 m
局部 costmap inflation_radius: 0.10 m
```

旧地图 `lab_202609081650`：

```text
public_map.pcd a689cdd9b63cec90f7d8beab6f9b7332cfe7c6d6e0399fe4a9abef05fdb9b974
map.pgm       86b6f4515e40e592766b52ac2aa654fd20e3233b51833275d3f28c81f0a02162
map.yaml      8e7448151fad984c4d3f5ac90ea2cea82191d9c1303acf39bb5f4353ff0660cd
```

外参文件：

```text
src/go2_core/config/extrinsics.yaml
231348cee35c7d0a64277a7b52fcaee529731918ad419dea65d92742b2115439
```

未修改控制速度、footprint、TEB、SDK bridge 步态/Enable 逻辑或网络配置。

## 6. 关键源码哈希

```text
run_go2                                             64375463a38a7576a123b32fb189cb086ccd66e833f8790d5e268d62bf86571e
src/go2_mapping/src/map_builder.cpp                 c657667e421d0bf6ba1c7f669fec75fd743498480f1831b2bc36be1431bf6d16
src/go2_mapping/src/occupancy_map_exporter.cpp      cf003e3e6b22604b7068a7fa7e98b21b5e41b0abae0529603fc93e950d77e514
src/go2_terrain/src/terrain_guard.cpp               5f54832d2ac32fb5465615981ca636990d4a84d34b077953219aa4115085bc9c
src/go2_terrain/src/terrain_exporter.cpp            090788c71805653d78b7fae7fe1d8aff82fc0f0b7d268a88e1f9686c7925864c
src/go2_terrain/src/go2_terrain_layer.cpp           0c7d7a4022a27f92b24788bf380331cb08a797cd9e45819b66c916bfaa5b9a8a
src/go2_navigation/src/navigation_supervisor.cpp    023d0a8381611caf7e3d6abc98106904c9f4bf991e4d03127ee752c5d9b71716
src/go2_control/src/velocity_shaper.cpp             eb6813f65cc58c8022062bbd70397631dc0f1667a814fe38a40f6955df23bdb4
```

## 7. 日志与回滚

部署和验证日志：

```text
/home/nvidia/go2_nav_ws/deployment_logs/terrain_20260909
```

部署前原始备份：

```text
/home/nvidia/go2_nav_ws/backups/codex_20260909_dynamic_terrain
```

最终同步前备份：

```text
/home/nvidia/go2_nav_ws/backups/codex_20260909_pre_final_sync/source_before_final.tgz
SHA256 a0de2bd32f1c0c166d9c4b66f8d25a2bfc3fe9478e2a81f483bac79988ba1412
```

发生异常时先停止 ROS 栈并保持 SDK Disable，再从备份恢复源码并重新执行
`./build_workspace.sh`。旧地图、外参和网络不依赖本次新增资产。

## 8. 现场剩余验收

1. 让机器狗进入标准站立姿态，启动新地图的 mock 导航，确认
   `estimated_sensor_height_m=0.43-0.59`、`ground_plane_fit_status=valid`、
   `/terrain/healthy=true` 稳定至少 10 秒。
2. 新建测试地图，让人员横穿后离开，确认最终 PCD/PGM 不保留人体轮廓，固定墙体和
   细障碍不丢失。
3. 在真实坡道采集新地图，复核天花板排除、坡面连续性和全局低坡优先规划。
4. 只有现场确认环境安全后才执行 `run_go2 navigation <map> --real` 和手动 Enable。

详细启动、导出、诊断和故障处理见 `TERRAIN_OPTIMIZATION_GUIDE.md` 与
`STARTUP_GUIDE.md`。
