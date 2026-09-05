<p align="center">
  <img src="docs/assets/aadcl-logo.png" alt="AADCL" width="96">
</p>

<h1 align="center">GO2 · 三维建图与自主导航</h1>

<p align="center">Livox Mid-360 · FAST-LIO · NDT-OMP · move_base/TEB · Unitree SDK2</p>

<p align="center">
  <img alt="版本" src="https://img.shields.io/badge/version-1.0.0-1677ff">
  <img alt="ROS" src="https://img.shields.io/badge/ROS-Noetic-22314E">
  <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-20.04-E95420">
  <img alt="C++" src="https://img.shields.io/badge/C%2B%2B-14-00599C">
</p>

本仓库是 Unitree GO2 EDU 与 Livox Mid-360 的 ROS Noetic 端侧工作空间，提供一条命令启动的三维建图，以及基于保存地图的重定位、自主规划和真机控制。自研代码集中在六个 `go2_*` 功能包中，机器人外参由单一配置文件管理。

**快速入口：** [完整启动手册](STARTUP_GUIDE.md) · [第三方版本](THIRD_PARTY.md) · [外参配置](src/go2_core/config/extrinsics.yaml) · [运动参数](src/go2_control/config/control.yaml) · [导航参数](src/go2_navigation/config)

> 当前提交由 2026-09-03 端侧审计及部署缓存整理。六个自研包和当前二维地图已包含；端侧离线期间无法取得的第三方源码与三维 `public_map.pcd` 未伪造补齐，详见[源码完整性](#源码完整性)。

## 核心能力

| 能力 | 实现 |
| --- | --- |
| 三维建图 | Mid-360 驱动、FAST-LIO、静态点云筛选与累积、PCD 原子保存 |
| 二维地图 | 从三维 PCD 导出 PGM/YAML，占据、自由与未知区域分离 |
| 重定位 | NDT-OMP 匹配、`map -> odom` 唯一发布、位姿跳变限制和定位健康检测 |
| 路径规划 | `move_base`、GlobalPlanner、TEB、本地实时点云避障和窄通道参数 |
| 真机控制 | 连续速度整形、Unitree SDK2 SportClient、命令超时、步态响应监测 |
| 安全门控 | 定位与底盘状态联合准入、旧目标清理、失效停车、电量门槛和诊断输出 |
| 一键启动 | `run_go2` 自动加载 ROS 和工作空间，统一管理建图、导航、地图与底盘状态 |

## 系统链路

第一次建图：

```text
Livox Mid-360
  -> FAST-LIO
  -> pose/cloud adapters
  -> static map accumulator
  -> public_map.pcd
  -> occupancy exporter
  -> map.pgm + map.yaml
```

第二次重定位与导航：

```text
Livox Mid-360 -> FAST-LIO -> odom -> base_link
                                  |
public_map.pcd -> NDT-OMP --------+-> map -> odom
                                  |
map.pgm -> GlobalPlanner -> TEB -> velocity shaper
                                  -> navigation supervisor
                                  -> Unitree SDK2 bridge -> GO2
```

`go2_navigation_supervisor` 截获 RViz 的 `/move_base_simple/goal`。只有定位有效且底盘控制已使能时，目标才会转发到 `/move_base/validated_goal`；状态丢失时会取消目标并发送零速度，重新使能后必须发布新目标。

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
│   └── third_party/       # FAST-LIO、Livox driver 与 SDK2（需补齐）
├── maps/<map_name>/       # 每张地图的 PCD、PGM 和 YAML
├── run_go2                # 统一操作入口
├── build_workspace.sh     # 编译并执行静态检查
├── validate_workspace.sh  # package 与 launch 检查
└── STARTUP_GUIDE.md       # 详细现场操作手册
```

六个自研 ROS 包均为 `1.0.0`。重组前的重复功能包未发布到活动仓库，避免形成重复包、重复 TF 或重复 publisher；它们只保留在本地离线归档中用于追溯。

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

先恢复 `src/third_party` 中记录的四个第三方源码目录，再安装 ROS 依赖并编译：

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

机器人静止完成 FAST-LIO 初始化后开始采集。完成后依次保存三维地图并导出二维地图：

```bash
run_go2 save-map
run_go2 export-map lab01
```

输出目录：

```text
maps/lab01/public_map.pcd
maps/lab01/map.pgm
maps/lab01/map.yaml
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
| `/move_base/validated_goal` | 监督器放行目标 | `map` |
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
- 不要并行启动旧工程和本工作空间，两套 TF、点云或 SDK publisher 会造成不可预测行为。
- 不要在未核对固件控制模式时调用 MotionSwitcher、ClassicWalk 或其他模式切换接口。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [STARTUP_GUIDE.md](STARTUP_GUIDE.md) | 建图、地图导出、重定位、导航、真机测试与故障排查 |
| [THIRD_PARTY.md](THIRD_PARTY.md) | 第三方来源和记录修订版本 |
| [go2_core/config](src/go2_core/config) | 机器人外参、frame 与网络配置 |
| [go2_navigation/config](src/go2_navigation/config) | costmap、GlobalPlanner、TEB 和 move_base 参数 |
| [go2_control/config](src/go2_control/config) | 速度整形与底盘控制参数 |

## 源码完整性

本仓库中的自研功能包、启动脚本、二维地图和迁移参考代码来自此前对 `/home/nvidia/go2_nav_ws` 的读取与已验证部署。生成本仓库时机器人不在线，以下内容尚未包含：

- `src/third_party/FAST_LIO`
- `src/third_party/livox_ros_driver2`
- `src/third_party/Livox-SDK2`
- `src/third_party/Unitree_SDK2`
- `maps/lab_202609031555/public_map.pcd`
- 端侧其他地图、rosbag、ROS 日志及构建产物

因此，干净 clone 在补齐上述依赖前不能完成真机编译。这一限制不会通过下载任意最新版依赖来掩盖；机器人重新联网后应按 `THIRD_PARTY.md` 的记录版本补齐并重新验证。

---

本项目由 AADCL 维护。各自研 ROS 包在 `package.xml` 中声明 BSD-3-Clause；第三方组件遵循各自许可证。
