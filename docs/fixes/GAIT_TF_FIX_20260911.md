# 第二条狗：普通停车步态与 TF 发布修复

部署目录：`/home/unitree/go2_nav_ws`，设备 `unitree@192.168.50.111`。

## 本次依据

用户复现中，enable 和首次行走没有出现已知反馈变化；路径转换发生约 8 ms 的 TF 未来外推错误后，速度归零，桥接发送 API 1003 StopMove 并收到成功响应。约 38 ms 后反馈 error_code 从 2010 变为 100，用户随后确认经典步态变成默认步态。

error_code 不是已确认的官方步态枚举。StopMove 是强关联线索，是否为固件触发原因以及替代停车是否保持经典，仍须实机复测。

## 修改

- 普通零速度：发送 `Move(0,0,0)`，最多约 20 Hz。至少发送 0.30 秒，且收到连续 0.20 秒的停稳反馈后结束发送。停稳条件为平面速度不超过 0.05 m/s、底盘 yaw_speed 和 IMU 转速均不超过 0.10 rad/s，反馈年龄不超过 0.15 秒。计时使用单调时钟，避免系统校时影响停车期限。
- 零速度发送失败，或 1 秒内无法确认停稳：禁用控制、清空旧速度并回退到 StopMove。正常停稳不禁用控制，无须每个目标重新 enable。
- 定位丢失、命令超时、显式 disable 和退出仍使用 StopMove。失败的 StopMove 不再被标记为完成；定时器重试，确认前不执行新的速度命令。故障或退出仍可能改变步态，本修改不是固件步态锁。
- TF：NDT 主回调与 TF 定时发布使用独立队列，20 Hz 发布最后接受的 map→odom 修正；短锁保护修正赋值和发布，NDT 匹配不持锁。保留 0.10 秒预发布时间，以及原有定位有效性检查。没有延长健康超时或修改 TEB、地图、雷达参数。

## 验证

- ARM64 真 SDK 桥接和 NDT 定位节点编译成功。
- `bash validate_workspace.sh`：包、launch 入口及地图导出配置检查通过。
- `python3 tools/test_bridge_idle_policy.py`：用假 SDK 执行生产停车、控制循环和定位回调，验证被动等待、正常停稳、过期/无效反馈、发送失败、故障停车重试及命令超时。
- `python3 tools/test_localization_tf_queue.py`：独立 localhost ROS master；模拟主回调阻塞 0.35 秒。新队列 248 次查询、0 次失败。
- 同一测试 `--baseline`：旧共享队列 249 次查询、169 次失败，证明能复现该类故障。
- 上述测试均未对真实底盘发送动作。仿真测试中的运动命令仅发给假 SDK。

## 实机复测

部署时现有导航进程继续运行旧程序；新程序在重启导航后加载。保留单独启动的 roscore，在导航终端按 Ctrl+C，再运行：

```bash
cd /home/unitree/go2_nav_ws
RVIZ=true run_go2 navigation lab_202609101805 --real
```

地图名仍是任意已建图名称，以上只是本次测试使用的地图。

1. 等定位就绪，需要时设置 2D Pose Estimate。
2. 在导航启动完成后，用遥控器重新选择经典步态。退出旧程序时仍会调用 StopMove，因此应在重启后再选择。
3. 新终端运行 `run_go2 enable`。
4. 在 RViz 发布短距离目标。观察移动、停车，以及到达目标或中途暂停时的步态。
5. 停下后等 3 秒，再手动转圈检查。确认停稳期间不要同时用遥控器要求运动，否则会干扰停稳判定。

诊断 `/go2/diagnostics` 应包含 `normal_stop_policy=zero_move_with_standstill_check`。日志正常应先出现 `Normal navigation stop: sending Move(0,0,0)`，再出现 `standstill confirmed`；若出现 `Normal stop failed`，记录原因并检查底盘状态，不要把它当成正常停车。

下一次采集应包含 SDK 请求/响应、sport 反馈、rosout、cmd_vel_nav/safe、定位/地形状态和 `/move_base_internal/status`。前一轮采集已经停止，须重新启动后再复测。

## 备份与回退

本次修改前已保存源码、控制/定位配置及旧可执行文件：

`/home/unitree/go2_archive/changes/robot2_before_gait_tf_fix_20260911_IQl9vT/before.tar.gz`

SHA256：`066cc273340275d7d0c764df8d4f0ae50f2fdaa92f4c1d7af61b77e2c75fcbba`。

备份与源文件比较返回 0。运行中的旧进程不自动重启；需要回退时，先退出导航，再将该归档恢复到 `/home/unitree/go2_nav_ws`，最后按原指令重新启动。归档不包含也不覆盖地图。

原始复现证据：`/home/unitree/go2_archive/tests/gait_capture_20260911_7Rv5eh/navigation.bag` 与 `sdk-events.log`。
