<p align="center">
  <img src="docs/assets/aadcl-logo.png" alt="AADCL" width="96">
</p>

<h1 align="center">GO2 · 三维建图与自主导航</h1>

<p align="center">Livox Mid-360 · FAST-LIO · NDT-OMP · move_base/TEB · Unitree SDK2</p>

<p align="center">
  <img alt="版本" src="https://img.shields.io/badge/version-2.1.2-1677ff">
  <img alt="ROS" src="https://img.shields.io/badge/ROS-Noetic-22314E">
  <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-20.04-E95420">
  <img alt="C++" src="https://img.shields.io/badge/C%2B%2B-14-00599C">
</p>

本仓库是 Unitree GO2 EDU 与 Livox Mid-360 的 ROS Noetic 端侧工作空间，提供一条命令启动的动态过滤三维建图，以及基于保存地图的重定位、全局坡度规划、局部实时避障和真机控制。自研代码集中在七个 `go2_*` 功能包中，机器人外参由单一配置文件管理。

**快速入口：** [完整启动手册](STARTUP_GUIDE.md) · [地形优化说明](TERRAIN_OPTIMIZATION_GUIDE.md) · [遥控接管与目标恢复](docs/DEPLOYMENT_MANUAL_RESUME_20260912.md) · [V2.1.2 发布说明](docs/RELEASE_NOTES_V2.1.2.md) · [第三方版本](THIRD_PARTY.md)

> V2.1.2 对应机器狗 1 已部署、编译并由用户现场反馈正常的遥控交接修复。普通摇杆介入时暂停自动控制并保留目标，回中稳定 1 秒后通过健康与控制检查，重新规划并继续原目标。修复 `SwitchJoystick(1027)` 被误判为步态切换的问题，接入实际 LowState 遥控数据；29 项相关测试通过。原启动命令、速度、外参和地图算法保留，历史版本继续可用。固件仍未提供可靠的物理步态反馈，接口成功应答不等于步态测量确认。

## 核心能力

| 能力 | 实现 |
| --- | --- |
| 三维建图 | Mid-360、FAST-LIO、六自由度射线原点、两级贝叶斯动态过滤和 PCD 原子快照 |
| 二维/地形地图 | PGM/YAML 与 elevation、slope、roughness、step、cost、confidence 六层 2.5D 资产 |
| 重定位 | NDT-OMP 匹配、`map -> odom` 唯一发布、位姿跳变限制和定位健康检测 |
| 路径规划 | `move_base`、GlobalPlanner、全局坡度代价、TEB、Patchwork++ 局部实时避障 |
| 真机控制 | Enable 请求经典步态、遥控暂停与目标续走、连续速度整形、命令超时与步态响应监测 |
| 安全门控 | 定位与底盘状态联合准入、旧目标清理、失效停车、电量门槛和诊断输出 |
| 一键启动 | `run_go2` 自动加载 ROS 和工作空间，统一管理建图、导航、地图与底盘状态 |

## 系统链路

第一次建图：

```text
Livox Mid-360 -> FAST-LIO -> timestamped 6DoF ray origin
                              -> Bayesian static/dynamic mapper
                              -> public_map.pcd + traversed_path_map.pcd
                              -> validated occupancy + terrain exporter
                              -> map.pgm/yaml + terrain_2p5d assets
```

第二次重定位与导航：

```text
Livox Mid-360 -> FAST-LIO -> odom -> base_link
                                  |
public_map.pcd -> NDT-OMP --------+-> map -> odom
                                  |
map.pgm -> StaticLayer -> Go2TerrainLayer -> GlobalPlanner
live cloud -> Patchwork++ -> Terrain Guard -> local ObstacleLayer
GlobalPlanner -> TEB -> velocity shaper -> Unitree SDK2 bridge -> GO2
```

`go2_navigation_supervisor` 截获 RViz 的 `/move_base_simple/goal` 和公开 action。只有定位、底盘控制、实时地形链和内部 `move_base` 同时就绪时，目标才会转发到内部 action server。普通遥控介入时保留用户目标、暂停内部规划执行；回中稳定后清理 costmap 并重规划续走，期间的新目标替换保留目标。显式取消、Disable、姿态接管或健康故障仍终止自动续走。

## 目录结构

```text
go2/
├── src/
│   ├── go2_core/          # TF、外参、里程计和点云坐标适配
│   ├── go2_mapping/       # 三维地图累积与二维占据栅格导出
│   ├── go2_localization/  # 地图加载、NDT-OMP 与定位健康保护
│   ├── go2_navigation/    # move_base、GlobalPlanner、TEB 与目标监督
│   ├── go2_control/       # 速度整形、SDK2 bridge、底盘状态与诊断
│   ├── go2_bringup/       # 建图和导航总 launch、系统状态监控
│   ├── go2_terrain/       # 离线地形重建、全局坡度层和实时地面/障碍分类
│   └── third_party/       # FAST-LIO、Livox、Unitree SDK2 与 Patchwork++
├── maps/<map_name>/       # 历史地图及当前 PCD/PGM/2.5D 验证地图
├── run_go2                # 统一操作入口
├── build_workspace.sh     # 编译并执行静态检查
├── validate_workspace.sh  # package 与 launch 检查
└── STARTUP_GUIDE.md       # 详细现场操作手册
```

七个自研 ROS 包均位于 `src/`。重组前的重复功能包未发布到活动仓库，避免形成重复包、重复 TF 或重复 publisher；旧 `/home/nvidia/go2_mid360_nav` 工作树已退出运行链并在 V2.0.0 发布后清理，历史源码由 Git 提交保留。

## 环境要求

- NVIDIA Jetson，AArch64。
- Ubuntu 20.04、ROS Noetic、Python 3、C++14。
- PCL、Eigen3、OpenMP、Boost。
- ROS Navigation Stack、GlobalPlanner、TEB Local Planner、map_server。
- Livox Mid-360 及对应 Livox ROS Driver 2/Livox SDK2。
- Unitree GO2 EDU 及 AArch64 Unitree SDK2。

记录的第三方基线：

| 组件 | 记录版本 |
| --- | --- |
| FAST-LIO | 基于 `251c328d1f51a958a18cceb7055d52c46a815f44` 的 GO2 快照 |
| Livox ROS Driver 2 | `4a1def929e5b59c7a8122d19fce6efba581ce9f7` |
| Livox SDK2 | 与上述驱动配套的端侧快照 |
| Unitree SDK2 | `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` |

不要直接替换 SDK 版本。`go2_control` 链接 Unitree SDK2 中的 AArch64 静态库和 CycloneDDS 动态库，升级前必须检查 ABI、固件模式与真实机器人测试结果。

## 部署与编译

工作空间默认部署路径为：

```text
/home/nvidia/go2_nav_ws
```

完整仓库已包含 `src/third_party` 中记录的五个第三方源码快照。克隆后安装 ROS 依赖并编译：

```bash
cd /home/nvidia/go2_nav_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./build_workspace.sh
```

编译脚本使用单任务构建以降低 Jetson 内存压力，并在成功后安装用户级 `run_go2` 命令。日常操作不需要在每个窗口手动 source。

## 快速开始

### 第一次：三维建图

```bash
run_go2 mapping lab01
```

需要 RViz 时：

```bash
RVIZ=true run_go2 mapping lab01
```

机器人静止完成 FAST-LIO 初始化后开始采集。完成后依次保存三维地图并导出二维地图与地形资产：

```bash
run_go2 save-map
# 在建图终端按 Ctrl+C，等待保存并退出后再导出：
run_go2 export-map lab01
```

输出目录：

```text
maps/lab01/public_map.pcd
maps/lab01/traversed_path_map.pcd
maps/lab01/map.pgm
maps/lab01/map.yaml
maps/lab01/terrain_2p5d.yaml
```

### 第二次：重定位与自主导航

首次检查必须先使用 mock bridge：

```bash
RVIZ=true run_go2 navigation lab01
run_go2 status
```

地图、TF、点云和定位确认正确后停止 mock launch，再启动真机 bridge：

```bash
RVIZ=true run_go2 navigation lab01 --real
run_go2 status
run_go2 enable
```

`enable` 会先取消旧目标、发送零速度并清理 costmap，再请求 SDK bridge 使能。命令同时核验服务返回和 `/go2/control/enabled`，失败时不会显示误导性的成功提示。使能成功后，在 RViz 中发布新的 `2D Nav Goal`。

导航中用遥控器前进或转向后，放开摇杆并保持回中至少 1 秒；检查通过后会继续原目标，无需再执行 `enable`。回中确认后还需等待控制器应答与重新规划。显式 `reset-navigation` 或 `enable` 仍按原流程清理旧目标；详情见[遥控接管与目标恢复](docs/DEPLOYMENT_MANUAL_RESUME_20260912.md)。

随时停止底盘控制：

```bash
run_go2 disable
```

## TF 与 Topic 约定

| 数据 | Frame/关系 |
| --- | --- |
| 全局地图与导航目标 | `map` |
| FAST-LIO 连续里程计 | `odom` |
| 机器人机体 | `base_link` |
| 地面投影 | `base_footprint` |
| 激光雷达 | `lidar_link` |
| 全局 TF 链 | `map -> odom -> base_link -> lidar_link` |
| FAST-LIO 内部世界帧 | `lio_odom`，不直接作为公共导航 Fixed Frame |

RViz 的 Fixed Frame 必须设为 `map`。

| Topic | 用途 | Frame |
| --- | --- | --- |
| `/lio/odometry` | FAST-LIO 原始里程计 | `lio_odom` |
| `/odom_nav` | 导航里程计 | `odom` |
| `/cloud_registered_base` | NDT 输入点云 | `base_link` |
| `/cloud_registered_odom` | 建图点云 | `odom` |
| `/map_cloud` | 三维重定位地图 | `map` |
| `/map_2d` | 二维导航地图 | `map` |
| `/localization/pose` | NDT 全局位姿 | `map` |
| `/localization/ok` | 定位健康门控 | 无 frame |
| `/move_base_simple/goal` | RViz 原始目标 | `map` |
| `/navigation/ready` | 定位、控制、地形链与内部规划器联合就绪状态 | 无 frame |
| `/cmd_vel_nav` | TEB 输出 | 无 frame |
| `/cmd_vel_safe` | 整形与限幅后速度 | 无 frame |

## 外参与导航边界

唯一机器人外参文件为 [`src/go2_core/config/extrinsics.yaml`](src/go2_core/config/extrinsics.yaml)。当前保留的 Mid-360 安装外参：

```text
base_link -> lidar_link
x = 0.187 m, y = 0.0 m, z = 0.16 m
roll = -0.1 deg, pitch = 39.0 deg, yaw = 0.0 deg
```

该标定状态为 `provisional_preserved_from_go2_mid360_nav`。重新标定前不要在 launch 或源码中增加第二份静态 TF。

当前主要导航边界：

| 参数 | 当前值 |
| --- | ---: |
| 机器人 footprint | `0.70 m x 0.31 m` |
| footprint padding | `0.03 m` |
| 地图导出障碍膨胀 | `0.03 m` |
| 全局 costmap 膨胀 | `0.10 m` |
| 局部 costmap 膨胀 | `0.10 m` |
| TEB 最小障碍距离 | `0.10 m` |
| TEB inflation distance | `0.25 m` |
| 最大前进速度 | `0.60 m/s` |
| 最大倒车速度 | `0.18 m/s` |
| 最大转向角速度 | `0.80 rad/s` |

`0.10 m` costmap 膨胀用于约 `0.64 m` 窄通道测试，安全余量有限。不要通过缩小 footprint 获得虚假的可通行路径；应以现场实测净宽、点云噪声和机器狗姿态为准。

## 安全要求

- 真机测试前保证周围环境安全，操作员能够立即遥控或执行 `run_go2 disable`。
- real SDK bridge 启动后默认 disabled；必须先检查定位、地图和诊断，再手动 enable。
- 电量低于 `25%` 时拒绝使能，步态测试建议充至 `40%` 以上。
- 定位丢失或控制失效时目标会被取消；恢复后必须重新 enable 并发布新目标。
- 若从 Git 历史恢复旧工程，不要与本工作空间并行启动；两套 TF、点云或 SDK publisher 会造成不可预测行为。
- 不要在未核对固件控制模式时调用 MotionSwitcher、ClassicWalk 或其他模式切换接口。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [STARTUP_GUIDE.md](STARTUP_GUIDE.md) | 建图、地图导出、重定位、导航、真机测试与故障排查 |
| [TERRAIN_OPTIMIZATION_GUIDE.md](TERRAIN_OPTIMIZATION_GUIDE.md) | 动态建图、地形导出、全局坡度与局部地面分类 |
| [V2.1.0 发布说明](docs/RELEASE_NOTES_V2.1.0.md) | 通用坡面/墙体导出修复、回归结果与兼容范围 |
| [V2.1.1 发布说明](docs/RELEASE_NOTES_V2.1.1.md) | 经典步态请求、高度滞回和隔离 bag 回放验证 |
| [V2.1.2 发布说明](docs/RELEASE_NOTES_V2.1.2.md) | 遥控保留目标、回中续走与经典步态确认修复 |
| [遥控接管与目标恢复](docs/DEPLOYMENT_MANUAL_RESUME_20260912.md) | 真实数据来源、状态交接、测试及回滚 |
| [经典步态修复与验收](docs/DEPLOYMENT_CLASSIC_GAIT_20260911.md) | 日志根因、诊断字段、原流程测试和回滚 |
| [通用导出修复](docs/TERRAIN_EXPORT_REVISION2_20260910.md) | revision 2 参数、质量检查、备份与回滚 |
| [WheelTech 算法对比](docs/WHEELTECH_ALGORITHM_COMPARISON_20260909.md) | 两套系统在算法和安全架构上的共同点、差异与后续建议 |
| [V2.0.1 发布说明](docs/RELEASE_NOTES_V2.0.1.md) | 最终地形门限的工作空间校验补丁 |
| [V2.0.0 发布说明](docs/RELEASE_NOTES_V2.0.0.md) | 本版本范围、验证状态、兼容与回滚说明 |
| [THIRD_PARTY.md](THIRD_PARTY.md) | 第三方来源和记录修订版本 |
| [go2_core/config](src/go2_core/config) | 机器人外参、frame 与网络配置 |
| [go2_navigation/config](src/go2_navigation/config) | costmap、GlobalPlanner、TEB 和 move_base 参数 |
| [go2_control/config](src/go2_control/config) | 速度整形与底盘控制参数 |

## 源码完整性

本仓库保存机器狗 1 `go2_nav_ws` 的源码、配置和地图快照，包含：

- 七个自研 ROS 功能包及其配置、launch、消息、插件和工具脚本；
- `FAST_LIO`、`livox_ros_driver2`、`Livox-SDK2`、`Unitree_SDK2`、Patchwork++ 的固定快照；
- 六组历史地图、`lab_202609081650` 兼容地图、室内地形地图及两张重新导出的室外 `real_20260910*` 地图；
- 一键启动、构建、验证脚本和完整现场操作手册。

为保持仓库可复现且干净，Catkin 生成目录 `build/`、`devel/`、运行日志、rosbag、临时暂存目录和本地备份未纳入版本控制。这些均为构建或运行产物，不属于项目源码。第三方快照的来源与记录修订见 [`THIRD_PARTY.md`](THIRD_PARTY.md)。

历史地图按端侧原样保存，不代表均可直接导航：`lab_202609091010`、`lab_202609091450`、`lab_202609091625` 的旧地形门限与当前契约不符，使用前须重新导出并检查。已通过本次正式资产校验的是两张室外 revision 2 图及 `lab_202609091725` revision 1 图，详见发布说明。

---

本项目由 AADCL 维护。原有自研包声明 BSD-3-Clause；第三方组件遵循各自许可证。V2.1.0 离线表面重建包含 WheelTech 代码适配，其上游许可证字段尚为 `TODO`，不能将该部分笼统视为 BSD；具体来源与许可边界见 [THIRD_PARTY.md](THIRD_PARTY.md) 及 [地形包说明](src/go2_terrain/THIRD_PARTY.md)。
