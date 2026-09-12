# 机器狗 1：遥控介入后保留目标并恢复导航

设备：`nvidia@192.168.50.110`，工程：`/home/nvidia/go2_nav_ws`。本次修改控制桥和导航监督节点，不修改地图、外参、网络、速度、footprint、TEB 或启动命令。

## 日志证据与原因

原导航日志：`/home/nvidia/.ros/log/5ac8af58-add4-11f1-b6ae-3c6d664ded41/rosout.log`。

- `1789126333.886339`：第一次 `ClassicWalk(true)` 得到 SDK 成功应答，随后控制使能。
- `1789126475.565042`：旧程序将外部 API `1027` 记为步态/姿态变化，永久 Disable；导航监督节点随后取消目标。
- 随后的多个目标因为 `control_enabled=false` 被拒绝。之后 `run_go2 enable` 的零速调用成功，但 `ClassicWalk(true)` 返回 `-1`。
- 官方 SDK 定义表明 API 1027 是 `SwitchJoystick(bool)`。它控制遥控路由，不是步态选择；旧程序没有区分其参数，也没有把普通摇杆介入建模为可恢复的暂停。
- `-1` 不是已确认的“已经处于经典步态”状态。本次不会将其当作成功。现有固件的 gait/mode 字段无法证明真实步态，GetState 也未返回有效步态反馈。
- 同一日志更早有地形健康短暂失败，这是另一个会停止导航的条件。读取现场状态时地形与定位正常，本次没有放宽其阈值。

## 新行为

1. 初始启动仍为 Disable，按原方式完成定位并执行 `run_go2 enable`。
2. 普通遥控前进/转向时，停止自动 SDK 速度输出，不持续发送零速或 StopMove 与遥控争抢。
3. 保留原导航目标与公开 `/move_base` Action。内部 `/move_base_internal` 的当前执行按确切 goal ID 暂停，以免遥控期间发生规划超时；对用户目标不报告取消、失败或完成。
4. 遥控回中且按键释放稳定 1 秒，定位、地形、内部规划服务就绪时，清理 costmap，复核控制器，再按当前位置重新规划并继续保留目标。遥控期间发布新目标会替换保留目标。
5. 仅有摇杆操作且未观察到步态/姿态变更时，保留此前已成功应答的 ClassicWalk 选择，不重复切换步态。
6. 显式 Enable 仍重新请求经典步态。仅当第一次返回通用拒绝 `-1` 时，最多尝试一次 `ClassicWalk(false)` → 等待 0.3 秒 → `ClassicWalk(true)`，每一步都必须成功。通信、API、lease 等其他错误不执行此恢复；摇杆、按键或外部姿态变化会中断准备过程。
7. 明确的 Disable、取消、reset、姿态/步态按键、控制器切换、定位/地形故障会终止自动续走。不会因为恢复健康而擅自恢复被终止的任务。
8. SDK 服务等待使用后台调用，监督节点继续接收健康、取消和替换目标；不会因为等待 SDK 而误判健康心跳超时。取消旧内部目标使用确切 ID，避免延迟 cancel-all 取消新目标。

## 实际遥控数据源

本机只读采样 5 秒：`rt/wirelesscontroller` 和 `rt/wirelesscontroller_unprocessed` 均为 0 条，`rt/lowstate` 为 2477 条，`rt/sportmodestate` 为 1496 条。因此正式实现从现有 LowState 订阅提取 `wireless_remote[40]`。

字节布局依据本机 Unitree SDK2 的 `example/state_machine/gamepad.hpp`：2 字节头、16 位按键、lx/rx/ry/L2/ly 浮点数。使用复制解码，避免 ARM 未对齐访问；校验头、数值和范围。回中阈值为各摇杆绝对值不超过 0.12，底盘遥控数据年龄不超过 0.5 秒。

这验证的是底盘 DDS 携带的遥控数据，不等同于固件提供了独立的遥控器无线链路在线证明。

新增状态话题 `/go2/control/state`：`disabled`、`enabled`、`manual_override`、`manual_ready`。原 `/go2/control/enabled` 保留兼容；新监督节点使用同一个有序状态话题区分普通暂停和 Disable，避免不同话题到达顺序造成误取消。

新增内部服务 `/go2_sdk_bridge_real/resume_after_manual`，只能恢复一次真实的、可恢复的遥控接管；不能从初始 Disable 或故障状态直接使能。

## 现场测试

本次没有运行真实 SDK 控制节点，没有调用真机 Enable、ClassicWalk、运动目标或姿态服务。需现场按原流程启动后确认实际表现。

```bash
cd /home/nvidia/go2_nav_ws
run_go2 navigation lab_202609121350 --real
```

使用当前需要的地图名；完成原本的重定位操作后在另一终端运行：

```bash
run_go2 enable
```

发布一个目标，行走中用遥控短暂前进/转向，然后回中并放开按键。预期约 1 秒回中确认，再加上控制器应答与重新规划的耗时后继续原目标；普通遥控结束不需要再次 Enable，也不需要重新发布目标。

遥控暂停中再发布另一个目标，预期恢复后执行新目标。随后测试显式取消或 Disable，预期不会自动续走。

状态观察（需要已经 source 工作空间）：

```bash
source /home/nvidia/go2_nav_ws/devel/setup.bash
rostopic echo /go2/control/state
```

如恢复被拒绝，保留当时日志与 `/go2/diagnostics`，不要通过重复切换步态来掩盖错误。日志会分别报告首次 ClassicWalk、关闭、再次开启的 SDK 返回码。

注意：原 `run_go2 enable` 仍先执行 reset 清理旧目标，这是显式重新使能流程。要保留遥控前的目标，应让普通遥控自动恢复，不额外执行 reset/enable。

## 文件、验证与回滚

原件备份：`/home/nvidia/go2_nav_ws/backups/manual_resume_20260911T115909Z`。备份名来自设备时钟，本次未修改系统时间。

修改文件：

- `src/go2_control/src/sdk_bridge.cpp`
- `src/go2_control/include/go2_control/classic_gait.hpp`
- `src/go2_control/include/go2_control/manual_control.hpp`
- `src/go2_control/test/classic_gait_test.cpp`
- `src/go2_navigation/src/navigation_supervisor.cpp`
- `src/go2_navigation/CMakeLists.txt`、`package.xml`
- `src/go2_navigation/test/manual_resume.test`、`manual_resume_test.py`

编译、测试、校验记录位于 `experiments/reenable_20260912/`。测试使用独立 ROS master、模拟 Action 与服务，不加载真实 SDK 节点。覆盖遥控暂停/恢复、目标替换、取消、reset、定位/地形故障、SDK 拒绝、慢 SDK 回复和旧消息到达顺序。

通过 16 个控制单元测试和 13 个监督节点集成测试。catkin 汇总中的 32/14 包含 XML testsuite 聚合或 rostest 外层结果，独立用例数为 29。准备过程中重复复核遥控恢复权限；同时发生的停止/姿态请求会撤销经典步态确认和自动恢复资格。

回滚先关闭导航，再从备份恢复两个包并编译。新增加的文件不会被旧代码引用，保留也不会影响回滚运行：

```bash
cd /home/nvidia/go2_nav_ws
tar xzf backups/manual_resume_20260911T115909Z/original_sources.tar.gz
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make --pkg go2_control go2_navigation -j2 -l2
```

源代码与运行接口测试不能替代真实固件的步态和遥控交接验收；最终真机效果以本节现场测试为准。
