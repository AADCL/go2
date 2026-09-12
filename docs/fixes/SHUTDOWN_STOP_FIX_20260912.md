# Nano：正常关闭导航时保留当前步态

## 原因与修复

导航过程中的普通零速停车此前已改为 `Move(0,0,0)` 并检查停稳，但 `Go2SdkBridgeReal` 析构函数仍无条件发送 `StopMove()`。即使狗已经停稳，退出导航仍会追加 API 1003；它与本机经典步态变为默认步态的实测现象相关。

RViz 在 `navigation.launch` 中不是 required 节点，也没有直接调用 SDK。单独关闭 RViz 不等于停止导航，已有目标可能继续执行。

新的退出流程：

- 已无待完成的桥接运动或已完成停车：不新增 Move、StopMove 或步态请求，不干扰遥控器。
- 尚有待完成的运动：关闭使能、清空缓存速度，重复发送零 Move，使用退出后新到的 DDS 速度反馈检查停稳。最少观察 0.30 秒，连续静止至少 0.20 秒；线速度不超过 0.05 m/s，转向速度不超过 0.10 rad/s，反馈新鲜度不超过 0.15 秒。
- 一秒内不能确认停稳，或零 Move 发送失败：仍用 StopMove 兜底。未确认的故障停车最多再重试两次，每次 SDK 响应超时为一秒。最终失败打印 `STOP UNCONFIRMED`，不会声称已停稳。
- SIGINT/Ctrl+C、SIGTERM、终端挂断 SIGHUP 的处理函数只设标记，停车在主线程执行。ROS shutdown 后也能执行，等待不依赖 ROS 定时器或模拟时间。退出标记设置后，排队的控制与定位回调不再发送动作。
- DDS 订阅保留到停车结束，再关闭订阅，避免回调访问正在销毁的成员。

显式 `disable`、定位丢失、运动命令超时、无迈步响应及已有故障停车的处理不变，仍可能触发 StopMove。本次不承诺任何异常下都保持步态。SIGKILL、断电和进程崩溃不能保证执行退出函数。

## 验证状态

已在第二条狗完成 ARM64 bridge 编译、Linux 回归测试及工作空间 package/launch 检查。模拟 SDK 与可控时间回归覆盖静止退出、运动中退出、旧反馈、持续运动、异常转向反馈、零 Move 失败、有限故障重试、重复退出、ROS shutdown 及 SIGINT/SIGTERM/SIGHUP 后的回调拦截。测试不连接机器人，不产生真实 DDS 运动请求。

```bash
python3 tools/test_bridge_idle_policy.py
```

另用 `tools/check_bridge_shutdown_lifecycle.py` 验证实际编译程序的生命周期：在独立、仅有 lo 的 Linux 网络命名空间内启动 ROS master 和 disabled bridge，分别发送 SIGINT、SIGTERM、SIGHUP 和 ROS XML-RPC shutdown。四项均正常退出，退出耗时分别为 0.164、0.214、0.214、0.268 秒，均记录 `already idle; no new SDK stop request`。网络命名空间无法访问底盘；该脚本发现任何非 lo 网卡都会拒绝运行。隔离环境中查询不到 Unitree 服务版本属于预期现象。

ROS 主动 shutdown 会先关闭 rosconsole，因此最终退出结果改用标准错误输出，保证即使 ROS 已退出，`STOP UNCONFIRMED` 等提示仍可记录。以上实际进程测试验证静止退出；运动中停止的回归使用模拟 SDK，真实步态与运动停车仍需现场验证。

## 部署与备份

已增量部署到第二条狗 `/home/unitree/go2_nav_ws`。修改前为 `227823b`，工作区干净且无导航进程；其地面检查优化完整保留。运行代码仅修改 SDK bridge，地图、雷达地址、规划和地形参数未改。部署过程中未启用底盘，也未启动连接底盘的实机导航。

备份与证据目录：`/home/unitree/go2_archive/changes/shutdown_stop_fix_20260912_PGf5TR/`。

- `before.bundle`：修改前完整 Git 历史，已通过 bundle 校验。
- `before.tar.gz`：原 bridge 源码、回归脚本及旧 ARM64 bridge 程序，已逐项与原文件比对。SHA256：`66393149a2e3bd32c4299dc27ca8d97bc124de99cd825f903c7fb53e6e7b3c6c`。
- `build-final.log`、`tests-final.log`、`validate-final.log`：最终编译、回归及工作空间检查。
- `lifecycle-final/`：四种实际进程退出方式的日志。

新可执行文件为 `devel/lib/go2_control/go2_sdk_bridge_real_node`，SHA256：`f5672aee74c7c4a25c333a7ad1e8b38b437b1f741e4760985a0de15eba5fab1b`。下一次正常启动导航即加载此文件。

需要回退时，先停稳、退出导航并保留任何新的本地改动，再切回原 `codex/nano-ground-check-20260911` 分支，从上述归档恢复旧 bridge 二进制。旧分支及全部地图保留。

## 实机复测

更新后的导航沿用原启动命令与地图名。遥控器选经典步态，完成一个短目标并停稳后执行 `run_go2 reset-navigation`，确认停止，再在导航终端按 Ctrl+C。观察 bridge 的 `already idle` 或 `standstill confirmed` 日志，并用遥控器确认步态。

首次对照不要先执行 `disable`，否则其保留的 StopMove 会混淆退出测试；异常时仍立即用遥控器或 `run_go2 disable` 停车。运动中退出、SIGTERM、终端关闭还需在可立即接管的空地单独验证，并同步记录 API 1003/1008、速度反馈及人工步态观察。`mode` 与 `gait_type` 不能单独区分本机已观察到的两种步态。
