# GO2 V2.0.0 发布说明

发布日期：2026-09-09

## 发布范围

本版本以机器狗 1 `/home/nvidia/go2_nav_ws` 的已验证状态为准，在 GitHub 原 `main` 历史之上新增提交并创建 `v2.0.0` 标签，不改写或删除 V1 历史。

主要新增和修复：

- 两级贝叶斯动态/静态点云建图、精确三维 DDA 和时间戳匹配的六自由度雷达射线原点。
- `public_map.pcd`、`traversed_path_map.pcd` 与 `mapping_snapshot.sha256` 原子保存契约。
- 离线地面/坡度/粗糙度/台阶/代价/置信度六层地图，以及完整性和 SHA256 校验。
- 全局 `StaticLayer -> Go2TerrainLayer -> InflationLayer`，局部保持 `ObstacleLayer -> InflationLayer`。
- Patchwork++ 当前帧地面分割、天花板排除、连续坡面局部放行和 Terrain Guard 健康门。
- Supervisor 对公开 Goal/action 的准入、取消、零速和 stale-goal 保护。
- velocity shaper 与 Unitree SDK2 bridge 的速度连续性、步态响应、电池/BMS/关节/IMU 诊断。
- 全局与局部 costmap 膨胀均为用户确认的 `0.10 m`，保留真实 `0.70 m x 0.31 m` footprint。
- 地形导出的基底自由区覆盖门限由 `10%` 调整为 `8%`；其他轨迹、拟合、连通、走廊和基底保持门不变。
- Terrain Guard 最少地面扇区由 4 调整为机器狗 1 实测适用的 2；输入断流、低频、平面质量和传感器高度等硬门不变。

## 兼容性

- 无 `terrain_2p5d.yaml` 的历史地图使用 legacy 导航配置。
- 带 `.go2_terrain_mapping_v1` 的新地图必须完成地形导出，否则导航拒绝启动。
- `terrain_2p5d.yaml` 存在时，任一资产缺失、尺寸不符或校验失败都会拒绝导航，不允许静默降级。
- 外参继续使用 `src/go2_core/config/extrinsics.yaml`，本版本未改变机器狗 1 的雷达安装参数。

## 已验证状态

- 动态建图、地图保存和占据/地形导出已经在机器狗 1 完成端侧测试。
- 最终门限下，随版本发布的 `lab_202609091725` 已通过地形资产校验；早期试验地图保留在端侧工作区，不纳入发布包。
- `lab_202609091725` 已完成重定位、首个 Goal 到达和多 Goal 运行；Terrain Guard 扇区误判问题已由实测采样定位并修正。
- 发布过程不包含 `build/`、`devel/`、ROS 日志、bag、备份、临时认证文件或旧工作树。

## 回滚

GitHub 历史提交不变。需要回滚源码时可检出 V1 的 `bb3e652`；机器狗 1 上本轮配置备份仍位于 `/home/nvidia/go2_nav_ws/backups/`。回滚后必须重新编译并重新启动相应流程，不能在运行中混用两个版本的节点。
