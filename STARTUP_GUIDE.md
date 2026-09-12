# GO2 Mid-360 建图与自主导航启动说明

本文档对应端侧工作空间：

```text
/home/nvidia/go2_nav_ws
```

旧工程 `/home/nvidia/go2_mid360_nav` 已在 V2.0.0 发布并验证后从机器狗 1 清理；需要对照或回退时使用 Git 历史和 `v2.0.0` 标签。新工作空间采用一个 catkin 工作空间、七个自研功能包和一个 `third_party` 目录；正常操作不需要逐窗口手动 `source`，也不再需要依次打开十几个 ROS 节点窗口。动态建图、2.5D 地形导出和坡度规划的完整规则见 `TERRAIN_OPTIMIZATION_GUIDE.md`。

## 1. 运行前安全要求

真机导航前必须满足以下条件：

1. GO2 周围留出足够空地，操作员随时可使用遥控器或急停。
2. 首次验证必须先用 mock 模式，不向机器人发送运动指令。
3. real SDK 启动后默认仍处于 disabled；确认定位、地图、代价地图和目标均正确后，才执行 `enable`。
4. 任何异常先执行 `run_go2 disable`，再终止主 launch。
5. 启动 FAST-LIO 时机器人必须静止，等待初始姿态归一化完成后再移动。
6. 执行 `enable` 前，先用遥控器或 App 让 GO2 正常站立。V2.1.1 在确认 `mcf` 高层控制器后显式请求 `ClassicWalk(true)`，成功回复后放行 Move；不自动选择控制器，也不调用 `StandUp()`、`BalanceStand()` 或 `FreeAvoid()`。经典步态请求成功不代表已通过物理步态反馈确认，首次升级仍需现场观察迈步与爬坡效果。
7. 电池 SOC 低于 25% 时 bridge 拒绝使能。步态测试建议充至至少 40%～50%，避免低电压影响动态表现。
8. 地形地图导航还会从实时连通地面估计 MID360 到局部坡面的实际高度。标准工作姿态
   启动/恢复连续 5 帧应在 `0.43-0.59 m`；已健康运行时使用 2 cm 滞回，即
   `0.41-0.61 m`。蹲伏时 `/terrain/healthy=false` 仍是正确保护，先正常站立。

## 2. 工程结构

```text
go2_nav_ws/
├── src/
│   ├── go2_core/          # TF、里程计和点云坐标适配
│   ├── go2_mapping/       # 三维地图累积和二维占据栅格导出
│   ├── go2_terrain/       # 离线 2.5D 地形、全局坡度层和实时地面分割
│   ├── go2_localization/  # 地图加载、NDT-OMP、定位质量保护
│   ├── go2_navigation/    # map_server、move_base、TEB、导航监督
│   ├── go2_control/       # 速度整形、超时保护、Unitree SDK2 bridge
│   ├── go2_bringup/       # 两条完整流程的总 launch 和系统状态监控
│   └── third_party/
│       ├── FAST_LIO/
│       ├── livox_ros_driver2/
│       ├── Livox-SDK2/
│       ├── Unitree_SDK2/
│       └── patchworkpp/
├── maps/<map_name>/       # 每张地图独立保存
├── run_go2                # 统一操作入口，内部自动 source
├── build_workspace.sh     # 编译与静态校验
└── validate_workspace.sh  # 包和 launch 静态检查
```

## 3. 保留的机器人外参与运动边界

外参文件：`src/go2_core/config/extrinsics.yaml`。

- `base_link -> lidar_link`：x=0.187 m，y=0，z=0.16 m，roll=-0.1°，pitch=39.0°，yaw=0°。
- FAST-LIO 内部 LiDAR/IMU 平移：`[-0.011, -0.02329, 0.04412]` m，旋转矩阵为单位阵。
- `base_link -> camera_link`：x=0.35 m，y=0，z=0.10 m；D435i 当前不参与定位和导航。
- GO2 footprint：0.70 m × 0.31 m。
- 当前配置上限：前进 vx≤0.60 m/s，倒车速度≤0.18 m/s，vy=0，|wz|≤0.80 rad/s；这次步态修复未改变这些既有速度参数。
- 允许规划器受限倒车。速度整形器持续前进目标下限为 0.30 m/s，倒车为 0.12 m/s，前进加速度上限为 0.60 m/s²；SDK bridge 不二次硬抬前进速度。原地转向整形下限为 0.50 rad/s，并保留换向滞回；SDK 转向死区为 0.01 rad/s，最小有效目标为 0.04 rad/s。

这些参数是从旧工程迁移并明确固化的当前值；Mid-360 安装状态仍标记为 `provisional_preserved`，以后重新标定时只需更新唯一的外参文件和 FAST-LIO 内部外参。

## 4. 一次性准备与编译

登录端侧：

```bash
ssh nvidia@192.168.50.110
cd /home/nvidia/go2_nav_ws
```

首次部署或修改源码后执行：

```bash
./build_workspace.sh
```

脚本会自动加载 ROS Noetic、以单任务方式编译以避免 Jetson 内存压力，并执行 package/launch 静态检查。编译成功后，日常启动不要再手动运行 `source /opt/ros/noetic/setup.bash` 或 `source devel/setup.bash`；`run_go2` 会自动处理。

编译脚本还会在 `/home/nvidia/.local/bin/run_go2` 建立用户级命令入口。该目录已经位于 nvidia 用户的 `PATH` 中，因此在 `~/Desktop` 或其他任意目录应直接执行：

```bash
run_go2 reset-navigation
run_go2 status
```

不要写成 `./run_go2`；前缀 `./` 的含义是“只在当前目录寻找这个文件”。只有当前目录正好是 `/home/nvidia/go2_nav_ws` 时，`./run_go2` 才成立。

`mapping` 和 `navigation` 启动前会检查 ROS master 中是否已有 LiDAR、FAST-LIO、TF、定位、move_base 或 real SDK bridge。发现旧工程或另一套 GO2 链仍在运行时会拒绝启动，并列出冲突节点；先回到原 launch 窗口按 `Ctrl+C`，确认冲突节点消失后再重试。该保护不会自动终止未知进程。

单独复查工作空间：

```bash
./validate_workspace.sh
```

## 5. 网络检查

当前约定：

| 设备 | 端侧网卡 | 端侧 IP | 设备 IP |
|---|---|---|---|
| Livox Mid-360 | eth1 | 192.168.1.50 | 192.168.1.191 |
| GO2 EDU | eth0 | 192.168.123.99 | GO2 默认 192.168.123.x 网段 |
| 管理网络 | wlan0 | 192.168.50.110 | 操作电脑所在网段 |

检查：

```bash
ip -br addr show eth0
ip -br addr show eth1
ping -c 3 192.168.1.191
```

如果网卡名发生变化，不要直接启用真机控制；先修改 `go2_core/config/robot.yaml` 以及导航启动时传给 SDK bridge 的网卡参数。

## 6. 第一次流程：三维建图

### 6.1 启动完整建图链

给地图取一个只包含字母、数字、下划线或短横线的名字，例如 `lab_20260901`：

```bash
cd /home/nvidia/go2_nav_ws
run_go2 mapping lab_20260901
```

该命令在一个终端中统一启动：

```text
Livox driver
  -> FAST-LIO
  -> 里程计/TF/点云适配
  -> GO2 三维地图累积器
  -> 系统状态监控
```

如果端侧有图形界面并希望同时启动 RViz：

```bash
RVIZ=true run_go2 mapping lab_20260901
```

### 6.2 启动后检查

另开一个终端只做短命令即可，不需要手动 source：

```bash
cd /home/nvidia/go2_nav_ws
run_go2 status
```

也可做详细检查：

```bash
rostopic hz /livox/lidar
rostopic hz /lio/odometry
rostopic hz /cloud_registered_odom
rostopic echo -n 1 /odom_nav
rosrun tf tf_echo odom base_link
```

正常情况下，机器人静止初始化后 `/odom_nav` 与 `odom -> base_link` 连续输出；建图输入点云位于 `odom`，不会再把 `base_link` 点云误当作世界坐标累积。

### 6.3 采集地图

- 启动后的前几秒保持机器人静止。
- 低速、平稳地遍历区域，尽量形成闭环。
- 人员可短时横穿以验证动态过滤，但不要长时间遮住固定结构。
- 对走廊、门口和转弯处适当重复经过，以增加结构约束。

### 6.4 保存三维地图

保持建图主 launch 运行，在第二个终端执行：

```bash
cd /home/nvidia/go2_nav_ws
run_go2 save-map
```

应生成：

```text
/home/nvidia/go2_nav_ws/maps/lab_20260901/public_map.pcd
/home/nvidia/go2_nav_ws/maps/lab_20260901/traversed_path_map.pcd
```

确认文件后，可在建图主终端按 `Ctrl-C` 正常退出：

```bash
ls -lh /home/nvidia/go2_nav_ws/maps/lab_20260901/{public_map.pcd,traversed_path_map.pcd}
```

### 6.5 导出二维导航地图

```bash
cd /home/nvidia/go2_nav_ws
run_go2 export-map lab_20260901
```

应在同一目录生成：

```text
public_map.pcd   # NDT 三维定位地图
map.pgm          # move_base 二维占据地图
map.yaml         # map_server 元数据
terrain_2p5d.yaml
terrain_{elevation,slope,roughness,step}.f32
terrain_{cost,confidence}.u8
```

无地形资料的旧地图继续使用 `src/go2_mapping/config/occupancy.yaml`。新地图先用该投影确定尺寸和原点，再按 `src/go2_terrain/config/terrain_export.yaml` 重建连续地面，统一生成 PGM 和六层地形资产；不会继承绝对 Z 投影中的坡面伪障碍。实测墙体和台阶覆盖自由证据，缺少可靠参考的区域保持未知。新增 `terrain_quality.yaml` 检查轨迹的地面、自由和起点连通比例；提交前校验失败保留上一版地图，正常提交 I/O 错误会回滚。已有 revision 1 地图仍可导航，不会自动重新导出。完整规则见 `TERRAIN_OPTIMIZATION_GUIDE.md` 和 `docs/TERRAIN_EXPORT_REVISION2_20260910.md`。

## 7. 第二次流程：重定位与自主导航

### 7.1 强制先做 mock 验证

```bash
cd /home/nvidia/go2_nav_ws
run_go2 navigation lab_20260901
```

此模式会启动完整定位和规划链，但控制末端是 mock，不会调用 Unitree SDK2。一个终端内包含：

```text
Livox + FAST-LIO + core
  -> 三维地图加载 + NDT-OMP + localization guard
  -> map_server + move_base + TEB
  -> navigation supervisor
  -> velocity shaper + mock SDK bridge
```

### 7.2 在 RViz 设置初始位姿

可在有桌面的端侧使用：

```bash
RVIZ=true run_go2 navigation lab_20260901
```

RViz 的 Fixed Frame 必须设为 `map`。等待 `/map_cloud`、`/map_2d` 和实时点云出现后：

1. 选择 “2D Pose Estimate”。
2. 在地图中点击机器人真实位置并拖动箭头指向机器人朝向。
3. 该消息必须发布到 `/initialpose`，frame 必须是 `map`。
4. NDT 首次匹配通过后，临时 identity `map -> odom` 会更新为真实变换。
5. localization guard 需要连续 5 次正常结果才将 `/localization/ok` 置为 true。

启动后 NDT 会先发布临时的 identity `map -> odom`，使 RViz 能立即选择
`map` 并显示地图。此时实时点云尚未完成全局对齐，`/localization/ok` 仍为
false，机器人不能被启用。发送 `/initialpose` 且 NDT 接受后，同一个节点会
把临时 TF 替换为真实 `map -> odom`；随后执行 `reset-navigation` 清理初始化
期间的代价地图数据。

检查：

```bash
rostopic echo -n 1 /localization/ndt_score
rostopic echo -n 1 /localization/ndt_ok
rostopic echo -n 1 /localization/ok
rostopic echo -n 1 /navigation/ready
rosrun tf tf_echo map base_link
```

此时先确认 `/localization/ok: true`。SDK bridge（包括 mock）启动时默认
disabled，因此执行下一节的 reset/enable 前，`/navigation/ready: false` 是正常状态。

### 7.3 mock 模式发送目标

先清除初始化期间的目标和 costmap，再放行 mock 后端：

```bash
run_go2 reset-navigation
run_go2 enable
```

`run_go2` 会自动发现当前唯一的 `/go2_sdk_bridge_mock/enable` 服务；mock 只记录
`/cmd_vel_safe`，不会加载 Unitree SDK 或向底盘发送数据。若 real 和 mock 服务同时
存在，命令会拒绝 Enable。确认 `/navigation/ready: true` 后再发送目标。

RViz Fixed Frame 保持 `map`，使用 “2D Nav Goal” 发布目标。观察：

```bash
rostopic echo /move_base/status
rostopic echo /move_base/NavfnROS/plan
rostopic echo /cmd_vel_nav
rostopic echo /cmd_vel_safe
```

确认全局路径不穿墙、局部代价地图障碍合理、TEB 不产生倒车速度、`/cmd_vel_safe` 平滑且超时归零。
检查完成后执行 `run_go2 disable`，再在主 launch 窗口按 `Ctrl+C`。

### 7.4 清除旧目标和旧控制状态

每次切换实车前、重新定位后或怀疑残留目标时执行：

```bash
cd /home/nvidia/go2_nav_ws
run_go2 reset-navigation
```

该命令会同时：

- 取消 move_base 当前及历史活动目标；
- 立即向规划控制输出零速度；
- 清空 global/local costmap。

新架构不再使用 EGO trajectory，因此不存在 `/planning/pos_cmd`、`/ego/goal` 或旧 EGO 轨迹的残留执行问题。

### 7.5 启动真机链

结束 mock launch 后，以 real 模式重新启动：

```bash
cd /home/nvidia/go2_nav_ws
run_go2 navigation lab_202609021334 --real
```

注意：SDK bridge 启动后默认 disabled。它会立即发布底盘状态，但不会在启动时切换运动模式，也不会让机器人动作。重新完成初始位姿和定位检查，然后按顺序执行：

```bash
run_go2 reset-navigation
run_go2 status
run_go2 chassis-status
run_go2 enable
```

`enable` 只有在以下步骤全部成功后才会真正放行速度：

1. 丢弃启用前缓存的所有速度；
2. 检查定位健康、底盘电池 SOC 不低于 25%，并确认两路 DDS 遥测在 0.50 秒内更新；
3. 通过 Unitree MotionSwitcher 查询当前控制模式；
4. 只读确认当前模式是本机固件实际注册的 `mcf`，不自动切换控制器；
5. 发送零速 `Move(0,0,0)`，再显式调用 `ClassicWalk(true)`（API 2049），要求成功回复；
6. 等待 0.30 秒，再次确认控制器仍为 `mcf` 且没有可见的模式接管后才置为 enabled。后续速度仍使用 `Move(vx, vy, vyaw)`，不周期性重发步态切换。

如果任一步失败，bridge 会保持 disabled，并在终端明确打印失败原因；此时不要重复发布目标，也不要连续反复执行 `enable`。把启动终端中的错误信息保存下来排查。

成功日志应包含类似内容：

```text
ClassicWalk(true), API 2049, acknowledged by MCF. Firmware gait feedback is unavailable; acknowledgement is not a measured gait confirmation.
Unitree motion controller confirmed [mcf]
GO2 Move preparation complete: controller=mcf gait_policy=classic_mcf.
REAL GO2 motion bridge ENABLED: classic_mcf.
```

随后再检查：

```bash
rostopic echo -n 1 /go2/diagnostics
```

其中应看到 `gait_mode=classic_mcf`、`classic_sdk_result=0`、`classic_request_accepted=true`、`allow_motion_mode_switch=false`、`required_motion_mode=mcf`、`active_motion_mode=mcf`、`motion_enabled=true`、`last_move_sdk_result=0` 和 `last_gait_error` 为空。`classic_feedback_verified=false` 表示本机固件缺少可验证的步态反馈，不表示 API 请求失败。此时再现场测试一个 0.5～1.0 m 的直线目标。需要停止时：

```bash
run_go2 disable
```

`disable` 会立即发送 `StopMove` 并锁住后续运动命令；不能以关闭终端代替安全停机。

`enable` 会主动丢弃启用前缓存的所有速度，再准备经典步态；部分失败路径会调用 `StopMove`。正确顺序仍是“reset → enable → 发布新目标”。发生可见的外部步态/姿态/控制器 API 接管时，bridge 停止继续发送 Move，需要人工重新 Enable。详细证据与测试步骤见 [经典步态与坡道误停修复](docs/DEPLOYMENT_CLASSIC_GAIT_20260911.md)。

## 8. TF 与 Topic 正式约定

### 8.1 TF 主链

```text
map -> odom -> base_link -> lidar_link
                         -> camera_link
```

- `map -> odom`：由 `go2_ndt_localizer` 唯一发布；初始化前为临时 identity，
  NDT 成功后切换为真实定位结果。临时阶段 `/localization/ok=false`。
- `odom -> base_link`：由 `go2_pose_adapter` 根据 FAST-LIO 输出发布。
- `base_link -> lidar_link`：由 `go2_tf_manager` 按保留外参发布。
- FAST-LIO 内部帧使用 `lio_odom` 和 `body_lio`，不允许再以 `world` 进入正式导航接口。

### 8.2 关键 Topic

| Topic | 类型/用途 | Frame |
|---|---|---|
| `/lio/odometry` | FAST-LIO 原始里程计 | `lio_odom`/`body_lio` |
| `/odom_nav` | 统一导航里程计 | `odom` -> `base_link` |
| `/cloud_registered_base` | 局部避障点云 | `base_link` |
| `/cloud_registered_odom` | 三维地图累积点云 | `odom` |
| `/map_cloud` | NDT 三维地图 | `map` |
| `/map_2d` | move_base 占据地图 | `map` |
| `/initialpose` | 人工初始位姿 | `map` |
| `/localization/pose` | NDT 全局位姿 | `map` |
| `/move_base_simple/goal` | RViz 导航目标 | `map` |
| `/cmd_vel_nav` | move_base/TEB 原始控制 | 无 frame |
| `/cmd_vel_safe` | 限幅、限加速度和门控后的控制 | 无 frame |
| `/go2/battery_state` | ROS 标准电池状态，电压、电流、SOC、温度、有效单体电压 | `base_link` |
| `/go2/state/bms` | Unitree BMS 原始字段，包含状态、循环次数和 15 个保留单体槽位 | `base_link` |
| `/go2/imu` | Unitree 底盘 IMU | `base_link` |
| `/go2/joint_states` | 12 个腿部关节位置、速度和估算力矩 | `base_link` |
| `/go2/state/low_state` | 完整 `rt/lowstate`：IMU、20 电机、BMS、足端力、遥控器、风扇、电源等 | `base_link` |
| `/go2/state/sport_mode` | 完整 `rt/sportmodestate`：mode、gait type、抬脚高度、机身速度、足端状态等 | `base_link` |
| `/go2/diagnostics` | DDS 新鲜度、使能状态、当前 gait type 和电池摘要 | 无 frame |

正式活动链中没有 `/global_path -> /sparse_waypoints -> /ego/goal -> /planning/pos_cmd`，也不需要 Goal Bridge。原 EGO 链只保留在 Git 历史中作为参考；若从历史版本恢复，不应与新 move_base/TEB 链同时启动。

## 9. 保护逻辑

- NDT 检查收敛状态、fitness、矩阵有限性、单次位置/角度跳变和匹配超时。
- localization guard 对全局位姿做第二层连续性检查，连续正常后才允许导航。
- navigation supervisor 在定位从正常变为异常时取消目标并发送零速度。
- velocity shaper 允许受限倒车、禁止横移，限制速度/加速度，并在 0.5 s 无新命令时归零。
- real SDK bridge 再次检查定位状态和命令超时；启动默认 disabled。
- real SDK bridge 在每次 `enable` 前确认已注册的 `mcf` 并显式请求 `ClassicWalk(true)`，不自动选择控制器。此前 `normal`、`sport_mode` 和 `ai` 返回 `7004`，不再使用这些名称试错。
- 无有效迈步响应持续至当前 6 秒门限时，bridge 会 disabled 并发送 `StopMove`；诊断中 `no_step_response=true`。纯转向仍要求卸载证据，短零速脉冲不能重置监测窗口。
- Bridge 在 enabled 期间，无目标、TEB 零速和命令超时均发送 `Move(0,0,0)`。SDK Move 频率保持 200 Hz，速度整形器为 20 Hz。显式 disable、定位丢失、no-step 或节点退出仍发送 `StopMove`；外部模式/姿态 API 接管时停止继续发 Move，避免覆盖人工接管。

### 9.1 查看 GO2 底盘与电池状态

真机导航启动后，状态采集不依赖运动使能；bridge 保持 disabled 时也可查看：

```bash
run_go2 chassis-status

# 或逐个查看完整话题
rostopic echo -n 1 /go2/battery_state
rostopic echo -n 1 /go2/state/bms
rostopic echo -n 1 /go2/state/sport_mode
rostopic echo -n 1 /go2/state/low_state
rostopic echo -n 1 /go2/diagnostics
```

常用的精简查询：

```bash
rostopic echo /go2/battery_state/percentage
rostopic echo /go2/state/sport_mode/gait_type
rostopic echo /go2/state/sport_mode/foot_raise_height
rostopic hz /go2/state/low_state
```

`/go2/battery_state/percentage` 的范围是 0.0–1.0；例如 `0.57` 表示 57%。`/go2/state/bms/current_ma` 保留 SDK 原始毫安值，`/go2/battery_state/current` 已换算为安培。当前电池只有 8 个有效单体，标准电池话题会过滤 DDS 中预留的零值槽位，完整 15 槽原始数组仍保留在 `/go2/state/bms/cell_voltage_mv`。

## 10. 常见故障排查

### 找不到地图

```bash
ls -lh /home/nvidia/go2_nav_ws/maps/<map_name>/
```

定位需要 `public_map.pcd`，move_base 需要 `map.yaml` 和 `map.pgm`，三者地图名必须一致。

### 没有 `/livox/lidar`

检查 eth1 地址、Mid-360 电源、设备 IP 和 `livox_ros_driver2` 配置。不要在同时运行旧、新两个 Livox driver 的情况下排查，否则会产生端口占用或重复 publisher。

### 没有 `/odom_nav`

先看 `/lio/odometry`。若 FAST-LIO 有输出但 `/odom_nav` 没有，检查启动阶段机器人是否静止，以及 `go2_pose_adapter` 日志。

### 初始位姿发出后定位仍为 false

- 确认 `/initialpose.header.frame_id` 是 `map`。
- 确认 `/map_cloud` 已加载且非空。
- 初始位置和朝向需足够接近真实值。
- 查看 `/localization/ndt_score`、`translation_jump`、`rotation_jump`。
- 不要为了“让它变绿”直接放宽阈值；先确认地图、外参和点云 frame。

### move_base 没有输出速度

检查 `/navigation/ready`、目标 frame、costmap 数据与 move_base 状态。定位保护未通过时无速度是正确行为。

地形地图还要检查：

```bash
rostopic echo -n 1 /terrain/status
```

若提示 `estimated MID360 height ... is below minimum`，查看估计高度和活动门限，
结合实际站姿、安装与坡道过渡检查原因。V2.1.1 启动下限为 0.43 m，已健康运行时
为 0.41 m；不能把所有高度越界都判成蹲伏。若平面样本、几何或残差不合格，检查
雷达遮挡和附近地面；停止输入后地形保护仍会关闭。

### real bridge 无法连接 GO2

检查 eth0 是否为 `192.168.123.99`、是否只启动一个 Unitree SDK2 bridge，以及机器人是否在正确工作模式。连接恢复前保持 disabled。

### 机器人抽搐或速度突变

先 disable，记录 `/cmd_vel_nav`、`/cmd_vel_safe`、`/go2/state/low_state` 和 `/go2/diagnostics`。确认 `gait_mode=classic_mcf`、`classic_sdk_result=0`、`allow_motion_mode_switch=false`、`active_motion_mode=mcf`、`last_move_sdk_result=0` 和 `no_step_response=false`。本机正常运动时 `mode/gait_type/foot_raise_height` 也可能全为 0，不能据此判断经典步态未切换；应结合关节运动、四足受力和现场观察。当前最大前进速度 0.60 m/s，持续步行目标下限 0.30 m/s，倒车上限 0.18 m/s；这些是既有配置，V2.1.1 未调速。若坡道目标被取消，同时检查 `/terrain/status` 的拟合高度、活动门限和输入新鲜度。

### 隔离导航链测试官方底盘 Move

只有在 bridge 已 disabled 且整个 navigation launch 已按 `Ctrl+C` 退出后，才允许执行：

```bash
run_go2 chassis-self-test
```

该程序不使用 MotionSwitcher、ClassicWalk、FreeAvoid、TEB 或速度整形。它先调用 `StandUp()`，再严格按 Unitree GO2 官方示例的 5 ms 周期连续发送 1 秒 `Move(0.30,0,0)`，最后无条件发送零速和 `StopMove()`。测试约前进 0.3 m，必须清空前方场地并握住遥控器。程序会打印 SDK 返回码、最大关节速度、最小足端力和是否出现足端卸载；如果官方测试也只摆动身体，则问题位于底盘固件、运动服务或控制权，而不是 ROS 导航。

当前固件上，`StandUp()` 后 1 秒直发测试会出现 RPC 返回 0 但不迈步。若机器人已经通过遥控器/App 正常站立，可执行对照项：

```bash
run_go2 chassis-standing-test
```

该对照项跳过 `StandUp()`，其余速度、频率、持续时间和停止保护完全相同。历史现场结果为最大关节速度 `6.67 rad/s`、足端力降至 0，证明直接 `Move` 可以迈步，但不能确认当时使用经典步态。V2.1.1 的正式 bridge 会在 Move 准备阶段额外请求经典步态；该旧对照工具不能代替新版 Enable 的验收。两项测试只允许在完整 navigation launch 已退出、前方场地清空时执行。

### 稍远目标未到终点就停止

先检查目标状态：

```bash
rostopic echo -n 1 /move_base/status
```

如果状态为 `4` 且文本包含 `Robot is oscillating`，这不是正常到达。当前
GO2 参数使用 12 s 的振荡观察窗口和 0.08 m 的进展距离，并修正了 TEB
不允许 `max_vel_x_backwards<=penalty_epsilon` 的约束。硬件边界仍禁止倒车；
如果仍然中止，应录制 `/cmd_vel_nav`、`/cmd_vel_safe`、`/odom_nav` 和局部
轨迹，区分局部代价地图阻塞与底盘未执行命令。

### 次日连续目标回归测试

机器人开机后先用遥控器或 App 正常站立，并确认遥控器可以让四足正常迈步。随后按第 7 节启动真机导航、完成 `2D Pose Estimate`，确认 `run_go2 status` 为 `True / GOOD`，再执行：

```bash
run_go2 reset-navigation
run_go2 enable
rostopic echo -n 1 /go2/diagnostics
```

诊断应显示 `motion_enabled=true`、`localization_ok=true`、`gait_mode=classic_mcf`、`classic_sdk_result=0`、`classic_request_accepted=true`、`active_motion_mode=mcf` 和 `no_step_response=false`。按以下顺序测试，每一步都等待 `/move_base/status` 给出结果：

1. 发布前方 0.5～0.8 m、朝向基本不变的直线目标；
2. 第一目标完成后等待 3～5 秒，不重新 enable，发布第二个前方 0.5～0.8 m 目标，验证 `Move(0,0,0)` 保持控制态后可以连续起步；
3. 发布 0.8～1.2 m、包含约 15～30 度转向的目标，验证小角速度不再被死区清零；
4. 检查最终状态。`status=3` 才是成功，`status=4` 表示中止，不能当作到达。

```bash
rostopic echo -n 1 /move_base/status
rostopic echo -n 1 /go2/diagnostics
```

测试期间出现不迈步、持续摇摆或路径异常时立即执行 `run_go2 disable`。不要在 Bridge 自动 disabled 后继续发布 goal；应先保存主启动终端日志和上述两个状态，再决定是否重新 enable。

## 11. 参数调整位置

| 内容 | 文件 |
|---|---|
| 机器人外参 | `src/go2_core/config/extrinsics.yaml` |
| footprint、速度上限、网络约定 | `src/go2_core/config/robot.yaml` |
| FAST-LIO | `src/go2_bringup/config/fast_lio_mid360.yaml` |
| 三维建图 | `src/go2_mapping/config/mapper.yaml` |
| PCD 到 PGM | `src/go2_mapping/config/occupancy.yaml` |
| 新地图地形重建/坡度代价 | `src/go2_terrain/config/terrain_export.yaml` |
| 实时地面分割 | `src/go2_terrain/config/patchworkpp_go2.yaml`、`terrain_guard_go2.yaml` |
| NDT | `src/go2_localization/config/localization.yaml` |
| 定位保护 | `src/go2_localization/config/guard.yaml` |
| costmap/move_base/TEB | `src/go2_navigation/config/` |
| 速度整形 | `src/go2_control/config/control.yaml` |

修改后重新执行 `./build_workspace.sh`。YAML 参数通常无需重新编译，但完整验证可以及时发现 launch 或依赖问题。

## 12. 停机顺序

真机导航：

```bash
run_go2 disable
run_go2 reset-navigation
```

确认机器人停止后，在主 launch 终端按 `Ctrl-C`。建图则先 `save-map` 并确认 PCD 文件，再停止主 launch。

## 13. 当前验证边界

工作空间的“编译成功”表示源码、链接依赖、package 发现和 launch 静态解析均通过。由于重组期间未让现场机器人实际运动，首次运行仍必须依次完成：传感器检查、mock 导航、低速近目标真机测试、再扩大场地。NDT 阈值、二维地图高度切片和 TEB 权重属于现场地图相关参数，需依据 rosbag 和实测日志收敛。
