# 机器狗 1：经典步态与坡道导航中断修复

日期：2026-09-11。设备：`nvidia@192.168.50.110`。
工程：`/home/nvidia/go2_nav_ws`。本次在机器人本机完成编译。

## 1. 结论与证据

本次发现两个独立问题，不能把录制中所有停止都解释成底盘没有切换步态。

### 原底盘代码没有请求经典步态

原配置为 `gait_mode=direct_mcf`。Enable 只执行控制器检查和 `Move(0,0,0)`，没有调用 `ClassicWalk(true)`。`mcf` 是控制器名称，不是经典步态的确认信息。因此，平地能走不能证明已选择适合本次坡道测试的经典步态。

Unitree SDK2 的 `ClassicWalk(bool)` 对应 Sport API **2049**，参数是布尔值 `data`。新代码在 Enable 内显式调用该接口，不重新使用此前失败的 `SelectMode(ai)`、`SelectMode(sport_mode)` 或旧 `SwitchGait`。

参考：[Unitree 官方 SportClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/sport/sport_client.py)、[官方 ROS2 发布记录中的弃用接口](https://github.com/unitreerobotics/unitree_ros2/releases)。本机链接现有 Unitree SDK2，未更换 SDK 版本或控制通道。

### Bag 中后两个目标被地形高度保护取消

录制文件：`2026-09-11-11-15-39.bag`，时长约 72.34 秒。
下表时间为相对 bag 开始的秒数。

| 目标 | 下发时间 | 结果 | 直接原因 |
|---|---:|---|---|
| 第 1 个 | 35.610 | 42.238 到达 | 正常 |
| 第 2 个 | 44.668 | 51.669 到达 | 正常 |
| 第 3 个 | 54.850 | 58.749 起被取消 | 地形拟合 MID360 高度 0.429955 m，略低于 0.43 m |
| 第 4 个 | 63.397 | 66.747 起被取消 | 地形拟合高度约 0.4216 m，再次触发下限 |

这些取消时刻，定位保持正常、SDK bridge 仍然 Enable，记录的 Move 返回值为 0，电池约 65–67%。地形状态先变为不健康，导航监督器随后取消目标并输出零速度。并非 GlobalPlanner 没有规划，也不是这份录制证明 SDK Move 报错。

注意：SDK 中 Move 的发送成功不等于已测量到足端执行成功。录制期间正常平地运动时也一直是 `mode=0`、`gait_type=0`、`foot_raise_height=0`，这些字段在本机 MCF 固件上无法用于步态确认。只读检查的两路 DDS 状态均如此；GetState 查询虽然返回 0，但内容为空。录制没有呈现可验证的遥控器经典步态切换事件，不能据此量化手动切换前后的爬坡效果。

## 2. 已部署改动

### Enable 显式选择经典步态

- 默认策略改为 `classic_mcf`，仍使用现有 MCF 控制器。
- 每次 Enable：保留定位和电池检查，增加当前 DDS 遥测新鲜度检查，发送零速度，再请求 `ClassicWalk(true)`，等待 0.30 秒，然后复核控制器。
- 请求失败时 Enable 返回明确 SDK 错误码，并保持 Disable；不静默退回其他步态，不反复切换控制器。
- Enable 成功后维持原来的 Move 发送方式、速度整形与频率，不周期性调用步态切换接口干扰迈步。
- 观察 Sport/MotionSwitcher DDS 请求；运行中如果看到改变步态、姿态或控制器的竞争请求，则失效本次经典步态许可并停止继续发送 Move，由人工重新 Enable。对于远端 Damp/StandDown 等接管，不额外发送姿态指令覆盖它。
- 接收侧并不能观察所有固件内部或物理遥控器行为，因此这是对可见 API 竞争的检测，不能声称物理步态得到完整闭环保证。
- 原 `direct_mcf` 保留为显式回滚选项，不在失败时自动使用。

新增诊断字段：`classic_api_id`、`classic_sdk_result`、`classic_request_accepted`、`classic_feedback_verified`、`last_classic_request_age_sec`、`mode_override_api`。

`classic_request_accepted=true` 表示最近的经典步态请求收到成功回复；**`classic_feedback_verified=false` 是当前固件反馈能力的说明，不等于请求失败**。现场实际抬腿和爬坡仍需验证。

### 地形高度采用有限滞回

配置文件：`src/go2_terrain/config/terrain_guard_go2.yaml`。

```yaml
health:
  min_sensor_height_m: 0.43
  max_sensor_height_m: 0.59
  sensor_height_hysteresis_m: 0.02
```

- 启动及故障恢复：仍需连续 5 帧在 **0.43–0.59 m** 范围内，且所有原健康检查通过。
- 已经健康运行：高度允许 **0.41–0.61 m**，容纳腿部起伏及坡道过渡处局部平面估计的小偏差。
- 超出运行范围仍立即关闭；一旦关闭，重新开启仍使用严格范围。
- 未放宽平面 RMSE、连通地面支持、点云超时和频率检查；未改变地面/障碍分类或局部坡度策略。
- 参数限定在 0–0.03 m，设为 0 可恢复原高度门限行为。

## 3. 验证结果

- `catkin_make --pkg go2_control go2_terrain -j2` 成功。
- 两包测试全部通过：经典步态 5 项、地形模型 48 项、地形算法 31 项、离线地面/导出事务 12 项、地图验证 Python 16 项，共 **112 个测试用例**。catkin 汇总工具对部分 gtest 的父子 XML 节点重复计数，因此不采用其简单合计作为用例数量。
- `bash validate_workspace.sh` 成功，包括建图、旧地图导航、地形地图导航的 launch 展开与参数检查。
- 在独立 ROS master `127.0.0.1:11461` 同时回放旧版及新版 TerrainGuard。只发布 `/terrain/patchwork_ground`、`/terrain/patchwork_nonground` 和模拟时钟，没有 SDK、底盘驱动、导航或目标/速度发布器。
- 旧版复现约 58.8 秒、66.8 秒及后续的高度保护关闭；新版完成启动后，到点云结束前没有再次关闭地形许可。两版看到相同最低拟合高度约 **0.420934 m**。
- 点云停止后新版仍报 `terrain input stale` 并关闭健康许可，超时保护有效。
- 外参、`run_go2`、导航配置文件 SHA256 检查一致。没有改网络、地图、footprint、TEB、速度限制及地图转换流程。

此回放验证了停止原因及地形检查修复；**未自动 Enable 或发出真机运动指令，不能代替实际经典步态与爬坡验收**。

## 4. 下次现场测试：原命令不变

机器人恢复正常站立，确认周围安全后，仍按原流程运行：

```bash
cd /home/nvidia/go2_nav_ws
run_go2 navigation <你的地图名> --real
# 在 RViz 完成重定位，等待定位及地形状态健康。
run_go2 status
run_go2 enable
run_go2 chassis-status
```

本次结束时没有启动真实导航，下一次启动会加载新二进制和配置。若之后已有旧实例常驻，先正常停止该实例再按上述命令启动。

1. Enable 应返回 `success: True`，消息包含 `classic_mcf`。日志应出现 `ClassicWalk(true), API 2049, acknowledged by MCF`。
2. 如返回失败，保存完整错误码。不要用旧步态重试运动来掩盖接口不支持问题；需要根据本机回复继续排查。
3. 先连续发布两个平地目标，再测试短距离上坡目标，现场观察四足是否实际迈步。首次验收不要同时用遥控器改步态，以便单独验证新 Enable。
4. 成功后再测试坡道中途停车及再次发布目标，确认无需遥控器重新选择经典步态。
5. 如果再次停车，先查看 `/terrain/status` 的 `ground_plane_fit_status`、`estimated_sensor_height_m`、`active_minimum_sensor_height_m`、`input_age_sec`，以及 `/go2/diagnostics` 中经典步态结果和控制状态，区分目标取消、控制 Disable 和底盘实际执行不足。

可在原终端中按需查看（不改变启动要求）：

```bash
rostopic echo -n 1 /go2/diagnostics
rostopic echo -n 1 /terrain/status
```

若需要再次录制，使用原有 rosbag 方式即可。只有新版录制与现场足端表现相结合，才能进一步验证该固件对 API 2049 的实际动作效果。

## 5. 文件与回滚

备份目录：

```text
/home/nvidia/go2_nav_ws/backups/classic_gait_20260911_031339
```

包含原两包源码、原 SDK bridge 和 TerrainGuard 二进制、源码/二进制/保持不变文件的校验清单。机器人时钟与电脑有偏差，文件名时间仅用于定位本次备份；本次未更改系统时间。

本次变更文件：

```text
src/go2_control/CMakeLists.txt
src/go2_control/include/go2_control/classic_gait.hpp          # 新增
src/go2_control/src/sdk_bridge.cpp
src/go2_control/launch/control.launch
src/go2_control/test/classic_gait_test.cpp                   # 新增
src/go2_terrain/include/go2_terrain/terrain_model.hpp
src/go2_terrain/src/terrain_guard.cpp
src/go2_terrain/config/terrain_guard_go2.yaml
src/go2_terrain/test/terrain_model_test.cpp
```

回滚前停止导航及控制。在尚未进行其他源码修改的情况下，可恢复两包旧源码后重新编译：

```bash
cd /home/nvidia/go2_nav_ws
tar -xzf backups/classic_gait_20260911_031339/source_before.tar.gz
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make --pkg go2_control go2_terrain -j2
```

新增的两个头文件/测试文件即使留在目录中，也不会被恢复后的旧 CMake/源码引用；无需删除整个工程。若未来已有其他修改，先比较差异再回滚，避免覆盖之后的工作。

分析、构建和回放日志：`experiments/classic_gait_20260911/`。本次没有推送 Git、没有更改地图文件，也没有自动运行实机运动测试。
