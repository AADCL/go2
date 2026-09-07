# GO2 EDU + Mid-360 新设备部署记录

目标设备：`unitree@192.168.50.111`

工作空间：`/home/unitree/go2_nav_ws`
部署日期：2026-09-06

## 设备专用网络

| 用途 | 接口 | 主机地址 | 设备地址 |
|---|---|---|---|
| GO2 底盘 DDS | `go2dds`（挂接 `eth1`） | `192.168.123.18/24` | `192.168.123.161` |
| SSH/外部网络 | `eth1` | `192.168.50.111/24` | 管理电脑 |
| Livox Mid-360（USB Ethernet） | `eth0` | `192.168.1.50/24`，网关 `192.168.1.1` | `192.168.1.168`（编号 30668） |

本机 USB Ethernet 已确认是 `eth0`，专用于 Mid-360。所有真实 SDK
启动均默认绑定 `go2dds`；如设备网卡命名改变，可临时使用
`GO2_INTERFACE=<接口名> run_go2 ...` 覆盖。

`eth1` 同时承载管理网和机器狗底盘物理链路。不要直接把 DDS 地址作为
`eth1` 的第二地址，否则 CycloneDDS 可能仍选择 `192.168.50.111` 作为源地址。
部署时使用独立 macvlan：

```bash
sudo nmcli connection add type macvlan \
  con-name go2-dds ifname go2dds dev eth1 mode bridge \
  ipv4.method manual ipv4.addresses 192.168.123.18/24 \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect yes
sudo nmcli connection up go2-dds
```

检查：

```bash
ip -br address show go2dds
ping -c 2 192.168.123.161
```

工程编译会为 Unitree SDK 随附的 CycloneDDS 库生成 `.so.0` 运行时链接，
防止误加载 `/usr/local/lib` 中不兼容的 DDS 库。

## 保持不变的机器人外参

`base_link -> lidar_link`：

- x = `0.187 m`
- y = `0.0 m`
- z = `0.16 m`
- roll = `-0.1 deg`
- pitch = `39.0 deg`
- yaw = `0.0 deg`

这是机器人安装外参。Livox 驱动 JSON 中的雷达内部外参保持 0，避免重复旋转。

## 一次性安装

```bash
cd /home/unitree/go2_nav_ws
chmod +x install_dependencies.sh build_workspace.sh validate_workspace.sh run_go2
./install_dependencies.sh
./build_workspace.sh
```

成功后 `run_go2` 会安装为 `/home/unitree/.local/bin/run_go2`。重新登录终端；
若命令仍找不到，执行：

```bash
export PATH="$HOME/.local/bin:$PATH"
```

## 第一次：手动遥控建图

1. 让机器狗站稳，确认 Mid-360 已上电。
2. 在终端启动建图（地图名可替换）：

   ```bash
   run_go2 mapping site01
   ```

3. 新开终端检查传感器与 LIO：

   ```bash
   rostopic hz /livox/lidar
   rostopic hz /livox/imu
   rostopic hz /lio/odometry
   run_go2 status
   ```

4. 使用原厂遥控器低速行走，闭环经过需要导航的区域。建图过程中不要运行真实
   SDK bridge，不要同时启动第二套 Livox/FAST-LIO。
5. 回到起点附近并停止，然后保存三维地图：

   ```bash
   run_go2 save-map
   ```

6. 停止建图主进程后生成二维占据地图：

   ```bash
   run_go2 export-map site01
   ```

7. 确认以下三个文件存在且非空：

   ```bash
   ls -lh /home/unitree/go2_nav_ws/maps/site01/{public_map.pcd,map.pgm,map.yaml}
   ```


## 第二次：重定位与自主导航

1. 把机器狗放在建图起点附近，保持静止启动：

   ```bash
   RVIZ=true run_go2 navigation site01 --real
   ```


2. 在 RViz 中使用 `map` 作为 Fixed Frame；先发布 `2D Pose Estimate`，确认点云、
   二维地图和机器人姿态重合。
3. 检查状态，只有定位和控制条件均正常时才继续：

   ```bash
   run_go2 status
   ```

4. 清除旧目标并使能底盘：

   ```bash
   run_go2 enable
   ```

5. 在 RViz 发布一个全新的 `2D Nav Goal`。旧目标不会在重新使能后自动恢复。
6. 结束时先禁用，再停止主 launch：

   ```bash
   run_go2 disable
   ```

## 安全与诊断

- 第一次真机测试先用近距离、直线目标，保证周围有人可以按急停。
- `navigation ... --real` 启动后默认 disabled；不要绕过 `run_go2 enable`。
- 目标规划成功但不运动时先看 `run_go2 status`，重点检查
  `/localization/ok`、`/go2/control/enabled` 与 `/navigation/ready`。
- 底盘电池、关节、足端力和运动状态：`run_go2 chassis-status`。
- 雷达无数据时依次检查 `ip addr show eth0`、`ping 192.168.1.168`、
  `rostopic hz /livox/lidar` 和 Livox 节点日志。
- 本工程的全局/局部 costmap 膨胀半径均为 `0.10 m`；机器人 footprint 与
  `0.03 m` padding 未缩小。
