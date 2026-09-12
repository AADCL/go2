# 第二条狗：实时地形检查放宽与复测

设备 `unitree@192.168.50.111`，工程 `/home/unitree/go2_nav_ws`。
用户要求先放宽实时地形检查，以减少右转时的瞬时误停。

## 修改

| 检查项 | 原值 | 当前值 |
| --- | --- | --- |
| 拟合的 MID360 离地高度下限 | 0.43 m | 0.35 m |
| 最小连通地面面积 | 0.60 m² | 0.40 m² |
| 恢复放行所需连续合格帧 | 5 | 3 |
| 连续几何证据不足容忍帧数 | 3 | 5 |
| 几何证据不足最长容忍时间 | 0.25 s | 0.50 s |
| 高度单项越界容忍 | 无 | 距最后完整合格帧最多 0.25 s |

高度容忍仅用于检查已放行、拟合数值有限、当前输入点数、地面点数、连通面积、近场支持、扇区覆盖和处理时限都合格的情况。异常帧不能刷新最后合格时间，也不能打开已经关闭的检查。独立的一次性到期定时器保证停止输入时仍执行到期检查；回调繁忙时仍受 ROS 调度延迟影响。

近场半径仍为 1.20 m，近场面积下限仍为 0.18 m²，扇区数仍为 2；高度上限仍为 0.59 m，平面 RMSE 上限仍为 0.04 m，输入超时仍为 0.60 s，输出最低频率仍为 8 Hz。没有修改地图、地面拟合算法、障碍物过滤、定位检查、TEB、速度限制或 SDK 步态处理。

0.35 m 是本轮根据录包放宽的测试门槛，不代表新的机械标定值，也不保证所有低姿态都能被识别。持续异常仍可能取消目标；本次未改变目标取消策略。

## 验证结果

ARM64 编译成功，48 项地形模型测试通过，`validate_workspace.sh` 包和 launch 校验通过。校验脚本中的旧门槛断言同步更新为本次配置。

完整节点回放使用独立 `http://localhost:11321` ROS master，仅发布测试点云，没有发布真实速度、目标或使能请求。统计从前两秒启动阶段之后开始，以下均为诊断采样数，不是点云帧数。

| 数据 | 原配置 | 最终配置 |
| --- | --- | --- |
| 右转卡住朝向的 25 秒录包 | 285/342 放行，57 次不放行 | 342/342 放行，其中 2 次采样使用高度容忍 |
| 先前正常站立录包 | 本轮未重跑旧配置 | 477/477 放行 |
| 先前低姿态录包 | 本轮未重跑旧配置 | 279/279 不放行，高度约 0.231–0.261 m |

完整节点合成测试先输入 0.50 m 正常地面，再切到 0.30 m：持续低高度时，距最后合格帧 0.251208 s 关闭；只输入一次异常后停止点云时，在 0.250137 s 关闭。两种情况均未自行重新放行。

卡住录包采自停车后保留的朝向，不包含首次取消目标前的完整运动过程。因此回放证明能减少该朝向下的地形检查误停，尚未证明实际右转已经完全解决。

## 生效和实机复测

部署时未重启现场导航；已运行节点继续使用旧参数和旧程序。需要先在停稳后退出原导航，再用原指令启动：

```bash
cd /home/unitree/go2_nav_ws
RVIZ=true run_go2 navigation lab_202609101805 --real
```

地图名按实际使用的地图替换。启动完成后，用遥控器重新选经典步态；退出原程序的 StopMove 路径仍可能影响步态。定位就绪后，新终端执行：

```bash
cd /home/unitree/go2_nav_ws
run_go2 enable
```

确认运行配置：

```bash
rosparam get /go2_terrain_guard/health/min_sensor_height_m
rosparam get /go2_terrain_guard/health/height_outlier_hold_sec
```

应分别显示 `0.35` 和 `0.25`。重新发布目标，复测原来的直行和右转。若再次停车，保留位置和朝向，读取 `/terrain/status` 和导航日志，避免连续点目标、重复 enable 导致原因混在一起。

## 备份、证据与回退

本轮目录：`/home/unitree/go2_archive/changes/terrain_relaxed_checks_20260911_qpsWES/`。

- `before.tar.gz`：修改前地形配置、源码、头文件、测试、地形节点二进制及回放工具。归档比对通过；SHA256 `a838b4c047e7868a427a112371b478cfca7b00704218cc29cc443310fa92a344`。
- `validation-before.tar.gz`：修改前工作区校验脚本，归档比对通过。
- `stalled_before.json`、`stalled_final.json`、`standing_final.json`、`low_final.json`：完整节点回放结果。
- `height_deadline.json`：持续异常和停止输入的到期检查。
- `build-final.log`、`tests-final.log`、`validate-final.log`：编译与验证记录。

回退时先停稳并退出导航，然后：

```bash
tar -xmzf /home/unitree/go2_archive/changes/terrain_relaxed_checks_20260911_qpsWES/before.tar.gz -C /home/unitree/go2_nav_ws
tar -xmzf /home/unitree/go2_archive/changes/terrain_relaxed_checks_20260911_qpsWES/validation-before.tar.gz -C /home/unitree/go2_nav_ws
```

备份含原节点二进制，可随后重新启动原导航。新增的离线检查工具和本说明可保留，不影响运行。地图和其他历史备份均未改动。
