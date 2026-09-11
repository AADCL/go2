# 机器狗 1 动态建图与坡度规划说明

本文对应 `/home/nvidia/go2_nav_ws` 的地形优化版本。原有 FAST-LIO、NDT、
GlobalPlanner、TEB、TF 主链、机器人外参、footprint、速度整形、SDK bridge
及 Enable 逻辑均未替换。

## 1. 兼容规则

- 旧地图没有 `terrain_2p5d.yaml`，自动使用 `legacy` 模式。
- 通过本版本 `run_go2 mapping` 新建的地图含
  `.go2_terrain_mapping_v1` 标记，必须成功执行 `run_go2 export-map` 后才能导航。
- `terrain_2p5d.yaml` 是地形资产的提交标记。只要它存在，任何文件缺失、尺寸不符、
  越界路径或 SHA256 不一致都会使导航拒绝启动，不能静默降级。
- `lab_202609081650` 及其原有 PCD/PGM/YAML 不补生成地形层，也不在部署时改写。

## 2. 数据链

建图：

```text
/cloud_registered_odom + /odom_robot(6DoF, 按时间戳插值)
  -> go2_map_builder
  -> 动态贝叶斯确认/清除
  -> public_map.pcd

/odom_nav(平面轨迹，每移动 0.05 m 记录)
  -> traversed_path_map.pcd
```

地图导出：

```text
public_map.pcd + traversed_path_map.pcd
  -> 稳定二维占据图基底（临时文件，不直接覆盖正式地图）
  -> 保守地面种子、连续局部平面生长、地面/障碍重分类
  -> 连续地面可纠正绝对 Z 投影造成的坡面伪障碍；轨迹补洞只填 unknown
  -> 真实障碍最后覆盖
  -> map.pgm + map.yaml + terrain_2p5d.*
```

新地图导航：

```text
map.pgm -> StaticLayer -> Go2TerrainLayer -> InflationLayer (全局)

/cloud_registered_base -> terrain_sensor(重力对齐、位于 MID360 原点)
  -> Patchwork++ -> 普通障碍/射线清除点
  -> ObstacleLayer -> InflationLayer (局部)
```

`public_map.pcd`、FAST-LIO 和 NDT 始终使用完整点云，地面分割只服务于局部
costmap，避免改变重定位输入。

## 3. 固定参数

### 3.1 外参与网络

- `base_link -> lidar_link`: `[0.187, 0.0, 0.16] m`,
  `[-0.1, 39.0, 0.0] deg`。
- `terrain_sensor` 位于实际雷达原点，姿态与重力对齐，不继承雷达的 39 度俯仰。
- Patchwork++ 名义传感器高度 `0.51 m`。运行时在雷达周围 `1.50 m` 内对最大
  四连通地面分量进行 Huber 平面拟合，按坡面在雷达原点处的截距估计实际离地高度；
  只有 `0.43-0.59 m` 才允许健康门打开。低姿态、侧倒、地面样本退化或拟合残差
  过大都会保持 Disable，不能通过增大固定高度容差绕过。
- `eth0=192.168.123.99/24` 连接 GO2 底盘。
- `eth1=192.168.1.50/24` 连接 MID360。
- `wlan0=192.168.50.110/24` 为管理网络。

### 3.2 动态建图

| 参数 | 值 |
|---|---:|
| 粗/细体素 | `0.20 / 0.05 m` |
| `p_hit / p_miss` | `0.65 / 0.40` |
| 占据/清除阈值 | `0.75 / 0.35` |
| 静态晋升 | 粗体素至少 12 次/2 s/60%；细体素至少 6 次/0.5 s/50%，公开仍要求父粗体素已确认 |
| 未确认候选清除 | 至少 4 次连续自由射线、持续 0.30 s |
| 已确认静态删除 | 至少 12 次连续自由射线、持续 1.20 s |
| DDA 射线 | stride 2，最远 20 m，端点保护 0.25 m |
| 候选超时 | `6 s` |
| 自动保存 | `30 s`，退出时再保存 |

射线原点由同一时间戳的 `/odom_robot` 六自由度位姿和保留的
`base_link -> lidar_link` 外参计算；`/odom_nav` 只记录行走轨迹。生产配置默认
不发布 `/go2_mapping/dynamic_points`。保存由单一后台写线程串行执行；
`mapping_snapshot.sha256` 最后提交并同时校验两个 PCD，保存失败、容量越界或
正在写入时不会留下可供导出的伪完整快照。

### 3.3 离线地形和代价

- 地图分辨率 `0.05 m`。
- 自动估计 `base_link` 到地面距离，仅接受 `0.20-0.55 m`。
- 连续地面生长允许到 `35 deg`，并阻止墙、孤立边缘、台阶链和断开天花板进入地面。
- 正常连续追踪可沿坡面累计升降；长缺测后的重捕获必须通过 PMF，且相对初始地面
  高差不超过 `0.65 m`，避免把约 `0.8 m` 的桌面/平台或 `2.55 m` 天花板当成地面。
- 普通障碍相对局部地面高度为 `0.05-1.50 m`，更高点不写入二维障碍。
- 已行走自由走廊半宽 `0.18 m`。全部记录到的机器人位置都可将 unknown 补为
  free，但不能清除已有障碍；相邻轨迹间距超过 `0.50 m` 时不连接，真实地形障碍
  最后覆盖自由证据。地面追踪是否成功只决定高程和坡度重建，不影响这项实走证据。
- 地图导出障碍膨胀参数保持 `0.03 m`。与旧导出器一致，它在 `0.05 m`
  栅格上四舍五入为一个四邻域格，实际栅格外扩为 `0.05 m`；全局和局部
  costmap 膨胀均保持 `0.10 m`。
- 所有地图先使用已经验证的 occupancy exporter 生成二维基底；新地图把基底写入
  隐藏的事务临时文件，地形 exporter 校验全部资产后才提交正式地图。基底中所有
  known 单元都保持 known；连续重建地面可把绝对 Z 投影造成的坡面伪障碍重分类为
  free，轨迹补洞只能填充 unknown，不能单独清除已有障碍；地形障碍最后覆盖。
- 地形提交前要求轨迹地面追踪率至少 80%、PMF 可采观测重建率至少 30%，且最大
  地面八连通采样分量至少占已重建地面的 60%。这里只允许相邻对角栅格参与非修改式
  完整度统计；主地面传播仍严格使用四连通，原有受高度约束的补洞逻辑不变。重建地面
  还必须至少覆盖基底 free 的
  8%，输出 known 至少覆盖已行走走廊的 95%，且基底 known 不能变回 unknown。
  任一门槛不满足时保留上一版正式地图。
- 坡度 `<=8 deg` 不加代价，`8-30 deg` 从 15 到 80 线性增加。
- 连续至少四格 `>30 deg` 为全局致命代价，横向扩展 `0.20 m`。

局部 costmap 不加载坡度、高程、台阶或粗糙度层。对于 Patchwork++ 误分到
非地面的连续 `8-70 deg` 平面，额外使用连接地面起点、至少三格单调连续性、
二维 Huber 平面拟合和残差阈值进行兜底，避免把 `10/20/30 deg` 坡误标为局部
硬障碍；墙、箱体和规则楼梯仍保留为 marking 障碍。
因此未进入静态地图的陡坡没有局部坡度保护；全局 `30 deg` 阈值是唯一坡度禁行依据。

## 4. 编译

首次部署会安装系统包：

```bash
sudo apt-get install ros-noetic-jsk-recognition-msgs
```

随后执行：

```bash
cd /home/nvidia/go2_nav_ws
./build_workspace.sh
```

脚本在可用空间少于 `5 GB` 时停止，不自动删除 `/home/nvidia/.ros/log`。
编译固定使用系统安装的 JSK 消息包，不构建 Patchwork++ 仓库内嵌的重复副本。
Patchwork++ 只编译在线 Go2 包装节点，不编译其演示程序。

部署或改动算法后执行完整回归（普通参数调整无需每次重复）：

```bash
cd /home/nvidia/go2_nav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests -j1
catkin_test_results --verbose
```

只有测试结果为零失败时才进入不发运动指令的运行时验收。

## 5. 第一次流程：建图和地形导出

地图名必须是新的，脚本会拒绝覆盖已有 PCD/PGM/YAML：

```bash
run_go2 mapping terrain_test_01
```

等待 FAST-LIO 初始化完成后再遥控行走。应覆盖坡道两侧、墙角和窄通道；动态障碍
测试时让人员横穿后离开。建图完成前保存：

```bash
run_go2 save-map
```

确认服务成功后在 launch 窗口按 `Ctrl+C`。正常退出还会原子保存一次。检查：

```bash
ls -lh ~/go2_nav_ws/maps/terrain_test_01/public_map.pcd
ls -lh ~/go2_nav_ws/maps/terrain_test_01/traversed_path_map.pcd
```

导出：

```bash
run_go2 export-map terrain_test_01
```

建图、导出和导航共用排他锁；导出前必须先正常停止建图 launch。脚本也会检查
残留 ROS 节点，发现另一套栈仍在运行时直接拒绝，而不会结束非本命令启动的进程。
每次导出都会生成唯一 ID：新地形地图把 ID 写入受校验和保护的元数据，历史地图则
最后原子写入 `.go2_legacy_export_receipt`。脚本只接受与本次 ID 完全相同的结果，
因此不会把上一次残留的地图文件误判成本次导出成功。

导出先在隐藏临时文件中生成稳定二维基底，再在同目录的临时 staging 中生成并校验
完整地形资产，最后才依次替换结果，并把 `terrain_2p5d.yaml` 最后写入。地面高度
估计越界、轨迹/地面完整性不足、点云不足、网格不一致或校验失败在提交前不会修改
旧结果，并返回非零状态。提交阶段若设备掉电，可能留下新旧资产
混合，但 metadata-last 和 SHA256 校验会让导航失败关闭；恢复供电后应重新执行导出，
不能删除元数据强制降级。导出开始和提交前都会复核
`mapping_snapshot.sha256`，建图源文件在导出途中发生变化时拒绝提交。

主要输出：

```text
map.pgm / map.yaml
terrain_2p5d.yaml
terrain_elevation.f32 / terrain_slope.f32
terrain_roughness.f32 / terrain_step.f32
terrain_cost.u8 / terrain_confidence.u8
terrain_ground.pcd / terrain_obstacles.pcd
terrain_preview.ppm / terrain_checksums.sha256
```

手工复核资产：

```bash
source /opt/ros/noetic/setup.bash
source ~/go2_nav_ws/devel/setup.bash
rosrun go2_terrain validate_terrain_map.py \
  --map-dir ~/go2_nav_ws/maps/terrain_test_01
```

## 6. 第二次流程：重定位和导航

先进行不发真机控制的检查：

```bash
run_go2 navigation terrain_test_01
```

启动输出必须显示 `terrain navigation`。检查：

```bash
run_go2 status
rosnode list | grep -E 'terrain|patchwork|move_base'
rostopic hz /terrain/patchwork_ground
rostopic echo -n 1 /terrain/status
rosparam get /move_base/global_costmap/plugins
rosparam get /move_base/local_costmap/plugins
```

定位健康后执行：

```bash
run_go2 reset-navigation
run_go2 enable
```

这里会自动选择 mock bridge，只放行 ROS 内部速度链，不连接或控制底盘。确认
`/navigation/ready: true` 后可发送测试目标；完成后执行 `run_go2 disable`。

全局顺序应为 `StaticLayer -> Go2TerrainLayer -> InflationLayer`；局部只能看到
`ObstacleLayer -> InflationLayer`。确认 `/map_cloud`、`/map_2d`、稳定的
`map -> odom` 和 costmap 后按 `Ctrl+C`。

现场确认安全后才能启动真机模式：

```bash
run_go2 navigation terrain_test_01 --real
run_go2 status
run_go2 enable
```

真实运动测试仍由现场人员执行。本次部署/编译过程不会自动 Enable 或发送导航目标。
`go2_navigation_supervisor` 独占公开的 `/move_base` Action 和
`/move_base_simple/goal`；真实规划器使用内部 `/move_base_internal` Action。
两种目标都只有在定位、控制及地形健康门全部就绪时才会转发，并由同一 Action
客户端原子替换旧目标，因此未就绪目标无法绕过入口门控，旧取消也不会误杀新目标。

旧地图示例：

```bash
run_go2 navigation lab_202609081650 --real
```

启动输出必须显示 `legacy navigation`，且不会启动 Patchwork++ 或
`Go2TerrainLayer`。

## 7. 运行时诊断

地形模式重点话题：

| Topic | 含义 |
|---|---|
| `/cloud_registered_terrain` | 重力对齐且原点位于 MID360 的输入点云 |
| `/terrain/patchwork_ground` | Patchwork++ 地面点 |
| `/terrain/patchwork_nonground` | Patchwork++ 非地面点 |
| `/terrain/obstacle_points` | 局部 costmap 普通障碍 marking 输入 |
| `/terrain/clearing_points` | 局部 costmap 射线 clearing 输入 |
| `/terrain/status` | 频率、地面比例、点数、处理耗时和健康状态 |

地面分割目标频率至少 `8 Hz`。若持续低于此值，先检查 CPU、输入点数和 TF，不能
通过增大订阅队列掩盖积压。所有实时订阅/发布队列均为 1。健康门还要求单个连续
地面分量至少 `0.60 m²`、机器人 `1.0 m` 近场内至少 `0.18 m²`，并覆盖
8 个方向扇区中的至少 4 个；这是机器狗 1 的 39 度前倾 MID360 实测可见范围，
地面高度、连通面积、近场面积、每扇区至少两格、频率和超时保护均仍有效。
覆盖率、离地高度拟合、坡面兜底和 clearing 必须来自同一个近场连通地面分量，
不能把远处正常高度地面与近处错误表面拼接成健康证据。健康门首次打开需要连续
5 个完整健康帧；打开后仅对几何证据短缺容忍最多 3 帧且不超过 `0.25 s`。
点云超时、输出低频、处理超时、平面残差超限和明确的雷达离地高度越界都会立即
关闭健康门；重新恢复仍需 5 个健康帧，并要求速度链收到新的命令。
稀疏、单侧、低姿态或脚下无可信参考的地面不会放行导航。
terrain 模式下 velocity shaper 还独立检查同一健康心跳；false、超过 `0.75 s`
未更新或启动后尚无心跳时，会立即清除目标/当前速度并持续发布零。该门不依赖
`/navigation/ready`，不会与 Enable 形成循环；legacy 地图不启用此附加门。

## 8. 失败处理与回滚

- 导航提示地形校验失败：不要删除 `terrain_2p5d.yaml` 强制降级，应重新执行导出并
  查明缺失或 checksum 错误。
- 地面高度不在 `0.20-0.55 m`：检查轨迹起点、初始站姿和外参，不使用默认高度继续。
- 天花板进入 PGM：检查诊断 PCD 和局部地面连通性，避免直接提高全局 Z 阈值。
- Patchwork 无输出：检查 `/cloud_registered_base`、`terrain_sensor` TF 和传感器高度。
- 任意控制异常：先 `run_go2 disable`，再停止 launch。

部署备份位于工作空间 `backups/` 下带 UTC 时间戳的目录。回滚时停止所有 GO2 launch，
恢复备份文件、删除本次新增的 `src/go2_terrain` 和
`src/third_party/patchworkpp`，然后重新执行 `./build_workspace.sh`。地图和网络配置不在
自动回滚范围内，且本次部署不会修改它们。
