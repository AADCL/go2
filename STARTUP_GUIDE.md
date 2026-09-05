# GO2 Mid-360 建图与自主导航启动说明

本文档对应端侧工作空间：

```text
/home/nvidia/go2_nav_ws
```

旧工程 `/home/nvidia/go2_mid360_nav` 未被修改，可用于对照和回退。新工作空间采用一个 catkin 工作空间、六个自研功能包和一个 `third_party` 目录；正常操作不需要逐窗口手动 `source`，也不再需要依次打开十几个 ROS 节点窗口。

## 1. 运行前安全要求

真机导航前必须满足以下条件：

1. GO2 周围留出足够空地，操作员随时可使用遥控器或急停。
2. 首次验证必须先用 mock 模式，不向机器人发送运动指令。
3. real SDK 启动后默认仍处于 disabled；确认定位、地图、代价地图和目标均正确后，才执行 `enable`。
4. 任何异常先执行 `run_go2 disable`，再终止主 launch。
5. 启动 FAST-LIO 时机器人必须静止，等待初始姿态归一化完成后再移动。
6. 执行 `enable` 前，必须先用遥控器或 App 让 GO2 正常站立并确认四足可正常迈步。`enable` 只读确认本机固件正在使用 `mcf` 高层控制器，不调用 MotionSwitcher、`StandUp()`、`BalanceStand()`、`ClassicWalk()` 或 `FreeAvoid()`；现场对照测试已证明这些姿态/步态切换会使当前固件进入暂时不执行 `Move` 的状态。
7. 电池 SOC 低于 25% 时 bridge 拒绝使能。步态测试建议充至至少 40%～50%，避免低电压影响动态表现。

## 2. 工程结构

```text
go2_nav_ws/
├── src/
│   ├── go2_core/          # TF、里程计和点云坐标适配
│   ├── go2_mapping/       # 三维地图累积和二维占据栅格导出
│   ├── go2_localization/  # 地图加载、NDT-OMP、定位质量保护
│   ├── go2_navigation/    # map_server、move_base、TEB、导航监督
│   ├── go2_control/       # 速度整形、超时保护、Unitree SDK2 bridge
│   ├── go2_bringup/       # 两条完整流程的总 launch 和系统状态监控
│   └── third_party/
│       ├── FAST_LIO/
│       ├── livox_ros_driver2/
│       ├── Livox-SDK2/
│       └── Unitree_SDK2/
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
- 当前室内边界：vx≤0.60 m/s，vy=0，|wz|≤0.80 rad/s。没有取消硬件边界，避免局部规划异常时向底盘发送无界速度。
- TEB 允许最高 0.18 m/s 的短距离受控倒车；速度整形器将持续前进目标保持在至少 0.30 m/s、持续倒车目标保持在至少 0.12 m/s，并按 0.60 m/s² 的加速度限制平滑升降。转向死区为 0.01 rad/s，纯转向输出下限为 0.50 rad/s，SDK bridge 不再二次硬抬前进速度。

这些参数是从旧工程迁移并明确固化的当前值；Mid-360 安装状态仍标记为 `provisional_preserved`，以后重新标定时只需更新唯一的外参文件和 FAST-LIO 内部外参。

## 4. 一次性准备与编译

登录端侧：

```bash
ssh nvidia@192.168.50.100
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
| Livox Mid-360 | eth0 | 192.168.1.50 | 192.168.1.191 |
| GO2 EDU | eth1 | 192.168.123.199 | GO2 默认 192.168.123.x 网段 |

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
- 避免人员长时间紧贴机器人或大面积动态遮挡。
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
```

确认文件后，可在建图主终端按 `Ctrl-C` 正常退出：

```bash
ls -lh /home/nvidia/go2_nav_ws/maps/lab_20260901/public_map.pcd
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
```

导出参数位于 `src/go2_mapping/config/occupancy.yaml`。若地面或桌面被错误投影成障碍，应先调整高度切片参数并重新导出，不要直接修改 PGM 掩盖问题。

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

只有 `/localization/ok: true` 且 `/navigation/ready: true` 时才允许进入目标测试。

### 7.3 mock 模式发送目标

RViz Fixed Frame 保持 `map`，使用 “2D Nav Goal” 发布目标。观察：

```bash
rostopic echo /move_base/status
rostopic echo /move_base/NavfnROS/plan
rostopic echo /cmd_vel_nav
rostopic echo /cmd_vel_safe
```

确认全局路径不穿墙、局部代价地图障碍合理、TEB 只在必要时产生受限倒车速度、`/cmd_vel_safe` 平滑且超时归零。

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
2. 检查底盘电池 SOC 不低于 25%；
3. 通过 Unitree MotionSwitcher 查询当前控制模式；
4. 只读确认当前模式是本机固件实际注册的 `mcf`，不自动切换控制器；
5. 不调用任何姿态或 gait 切换 API，保持操作者已经验证可行走的底盘状态；
6. 发送一次零速 `Move(0,0,0)`，再次确认控制器仍为 `mcf` 后才置为 enabled。后续速度使用 `Move(vx, vy, vyaw)`。

如果任一步失败，bridge 会保持 disabled，并在终端明确打印失败原因；此时不要重复发布目标，也不要连续反复执行 `enable`。把启动终端中的错误信息保存下来排查。

成功日志应包含类似内容：

```text
Unitree motion controller confirmed [mcf]
GO2 direct Move control armed: controller=mcf; no posture or gait transition API was called
REAL GO2 motion bridge ENABLED for direct Move control in mcf
```

随后再检查：

```bash
rostopic echo -n 1 /go2/diagnostics
```

其中应看到 `gait_mode=direct_mcf`、`allow_motion_mode_switch=false`、`required_motion_mode=mcf`、`active_motion_mode=mcf`、`motion_enabled=true`、`last_move_sdk_result=0` 和 `last_gait_error` 为空。此时才发送一个 0.5～1.0 m、方向明确、周围无障碍的直线测试目标。需要停止时：

```bash
run_go2 disable
```

`disable` 会立即发送 `StopMove` 并锁住后续运动命令；不能以关闭终端代替安全停机。

`enable` 会主动丢弃启用前缓存的所有速度，但不会调用 `StopMove` 或姿态切换 API；它以 `Move(0,0,0)` 进入并保持已站立控制态。因此正确顺序必须是“reset → enable → 发布新目标”；启用之前产生的速度不会被执行。

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

正式活动链中没有 `/global_path -> /sparse_waypoints -> /ego/goal -> /planning/pos_cmd`，也不需要 Goal Bridge。原 EGO 链保留在旧工程中作为历史参考，不应与新 move_base/TEB 链同时启动。

## 9. 保护逻辑

- NDT 检查收敛状态、fitness、矩阵有限性、单次位置/角度跳变和匹配超时。
- localization guard 对全局位姿做第二层连续性检查，连续正常后才允许导航。
- navigation supervisor 只在定位与底盘控制均就绪时转发新目标；任一状态失效时取消目标并发送零速度，未就绪期间的新目标直接拒绝。
- velocity shaper 禁止横移，仅允许最高 0.18 m/s 的受控倒车，限制速度/加速度，并在 0.5 s 无新命令时归零。
- real SDK bridge 再次检查定位状态和命令超时；启动默认 disabled。
- real SDK bridge 在每次成功 `enable` 前只读确认实际控制器为本机已注册的 `mcf`，但保持机器人已经站立且经遥控器验证的运动状态，不再调用姿态或 gait 切换 API。现场返回码 `7004` 已证明 `normal`、`sport_mode` 和 `ai` 都不是这台固件接受的选择名，因此默认禁止自动切换控制器。
- 非零速度持续 6 秒后，bridge 会同时检查 12 个关节的运动速度和四足是否出现卸载。SDK 若返回成功但没有真实迈步响应，bridge 会自动 disabled 并发送 `StopMove`；诊断中 `no_step_response=true`。
- Bridge 在 enabled 期间，无目标、TEB 零速和命令超时均持续发送 `Move(0,0,0)`，不再反复退出和重入底盘运动状态机。只有显式 disable、定位丢失、no-step watchdog 或节点退出才发送 `StopMove`。SDK 控制频率为 200 Hz，并保留纯转向方向锁定和零脉冲释放滞回。

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

检查 eth0 地址、Mid-360 电源、设备 IP 和 `livox_ros_driver2` 配置。不要在同时运行旧、新两个 Livox driver 的情况下排查，否则会产生端口占用或重复 publisher。

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

### real bridge 无法连接 GO2

检查 eth1 是否为 `192.168.123.199`、是否只启动一个 Unitree SDK2 bridge，以及机器人是否在正确工作模式。连接恢复前保持 disabled。

### 机器人抽搐或速度突变

先 disable，记录 `/cmd_vel_nav`、`/cmd_vel_safe`、`/go2/state/low_state` 和 `/go2/diagnostics`。确认 `gait_mode=direct_mcf`、`allow_motion_mode_switch=false`、`active_motion_mode=mcf`、`last_move_sdk_result=0` 和 `no_step_response=false`。本机 `/go2/state/sport_mode` 在不同姿态阶段报告过 `error_code=100` 和 `1002`；该字段没有随 SDK 提供枚举，不能把它单独解释为成功或失败，也不能只用 `gait_type` 和抬脚高度判断是否迈步，应以 `low_state` 的关节速度和四足受力为准。新链最大前进速度为 0.60 m/s，最低持续步行目标为 0.30 m/s，并由速度整形器平滑升降；若 `/cmd_vel_nav` 本身振荡，应调 TEB/代价地图，而不是继续移除底盘安全边界。

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

该对照项跳过 `StandUp()`，其余速度、频率、持续时间和停止保护完全相同。现场结果为最大关节速度 `6.67 rad/s`、足端力降至 0，确认直接 `Move` 可以形成完整步态；因此正式 bridge 采用相同的“已站立直接控制”路径。两项测试都会同步记录遥控器活动和 SportModeState，且只允许在完整 navigation launch 已退出、前方场地清空时执行。

### 稍远目标未到终点就停止

先检查目标状态：

```bash
rostopic echo -n 1 /move_base/status
```

如果状态为 `4` 且文本包含 `Robot is oscillating`，这不是正常到达。当前
GO2 参数使用 20 s 的振荡观察窗口和 0.08 m 的进展距离，并修正了 TEB
不允许 `max_vel_x_backwards<=penalty_epsilon` 的约束。当前仅允许最高 0.18 m/s 的受控倒车；
如果仍然中止，应录制 `/cmd_vel_nav`、`/cmd_vel_safe`、`/odom_nav` 和局部
轨迹，区分局部代价地图阻塞与底盘未执行命令。

### 次日连续目标回归测试

机器人开机后先用遥控器或 App 正常站立，并确认遥控器可以让四足正常迈步。随后按第 7 节启动真机导航、完成 `2D Pose Estimate`，确认 `run_go2 status` 为 `True / GOOD`，再执行：

```bash
run_go2 reset-navigation
run_go2 enable
rostopic echo -n 1 /go2/diagnostics
```

诊断必须显示 `motion_enabled=true`、`localization_ok=true`、`gait_mode=direct_mcf`、`active_motion_mode=mcf` 和 `no_step_response=false`。按以下顺序测试，每一步都等待 `/move_base/status` 给出结果：

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
