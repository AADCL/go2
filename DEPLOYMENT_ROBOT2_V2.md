# Robot 2 地形感知导航 V2 部署与验收

本文只适用于第二条 GO2（Jetson Orin Nano）。算法基线为 `v2.0.1`
（`7d01bb1`），仅叠加第二条狗的网络、设备和工作空间路径配置。

## 固定配置

| 对象 | 配置 |
| --- | --- |
| Jetson 管理地址 | `unitree@192.168.50.111` |
| 工作空间 | `/home/unitree/go2_nav_ws` |
| Mid-360 接口 / Jetson 地址 | `eth0` / `192.168.1.50` |
| Mid-360 地址 | `192.168.1.168` |
| GO2 DDS 接口 / Jetson 地址 | `go2dds` / `192.168.123.18` |

`eth0` 只连接 Mid-360；`go2dds` 只承载 Unitree SDK2。不要把底盘控制接口改成
`eth0`，也不要把第一条狗的 Livox 地址带到本机。

## 安全部署

部署时先在独立目录编译，旧工作空间和地图均保留：

```bash
source /opt/ros/noetic/setup.bash
cd /home/unitree/go2_nav_ws_v2_staging
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DROS_EDITION=ROS1 -DCATKIN_ENABLE_TESTING=ON -j1
catkin_make run_tests -j1
catkin_test_results build/test_results
./validate_workspace.sh
```

切换前必须确认没有 ROS 导航、建图或底盘进程。保留旧目录作为带时间戳的备份，
再把 staging 改名到正式路径；由于 catkin 会记录绝对路径，切换后移走 staging
阶段的 `build` 和 `devel`，并在正式路径重新编译。

## 非运动验证

先用 mock 控制链检查，禁止执行 `run_go2 enable`，也不要在 RViz 下发目标：

```bash
cd /home/unitree/go2_nav_ws
./run_go2 status
RVIZ=false ./run_go2 mapping robot2_smoke
```

确认 Mid-360、IMU、FAST-LIO、里程计、注册点云和系统状态都有数据后停止建图。
随后可用已有地图进行 mock 导航检查：

```bash
RVIZ=false ./run_go2 navigation <已有地图名>
./run_go2 status
```

已有旧地图没有 `terrain_2p5d.yaml` 时会自动使用兼容模式，不影响原有二维导航。

## 新地图与四项优化

```bash
RVIZ=true ./run_go2 mapping <新地图名>
./run_go2 save-map
./run_go2 export-map <新地图名>
```

新流程包含动态点云过滤、地面/天花板抑制以及 elevation、slope、roughness、
step、cost、confidence 地形资产。导航时坡度代价只加载到 global costmap；local
costmap 不加载坡度层，仅使用实时地面分割后的障碍与清除点云。

真机验收必须有人看护、场地清空并保持急停可用。先检查定位和规划，再启动 `--real`
并显式 `enable`；从低速短距离开始，最后再测试动态障碍和斜坡。

## 回滚

如切换后编译或非运动验证失败，停止所有 ROS 进程，把当前目录改为失败快照，再将
部署前的时间戳备份恢复为 `/home/unitree/go2_nav_ws`。不要删除失败快照或旧地图，
待定位原因后再处理。恢复后在旧工作空间执行一次原有的构建/状态检查。

