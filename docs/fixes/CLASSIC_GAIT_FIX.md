# 步态与 enable：机器人响应 -1 的处理

## 当前默认流程

用遥控器或 APP 选好经典步态，再按原命令启动导航、定位、enable。
默认 `gait_policy=preserve_current`：启动、enable 和每段运动开始都不发送
切步态请求。它保留操作者选择，不会验证或承诺固件在 Move 时始终处于
经典步态，也不能将 `mcf`、`sport_mode=0` 或 `gait_type=0` 当作经典步态证明。

空闲时不高频发送 `Move(0,0,0)`，运动结束执行 StopMove。正常到点、
规划失败及定位短暂丢失后恢复，可以重新点目标，不必重复 enable；
定位丢失取消旧目标并清空缓存速度，不自动恢复旧任务。
显式 disable 和无运动响应保护仍有效。

## -1 的来源

已核对机器人实际链接的 ARM64 `libunitree_sdk2.a`：

* `SportClient::ClassicWalk(bool)` 使用 API 2049，参数为布尔 data。
* `Client::CheckApi` 未注册时返回 3103；发送失败 3102；超时 3104；
  响应 API 不匹配 3105。
* 正常收到匹配响应后，`ClientBase::Call` 返回
  `Response.header.status.code`。这次 -1 来自机器人响应。
* 官方 ClassicWalk 封装丢弃 Response.data，错误定义未给出 -1 的具体语义。
  因此不能将 -1 解释为“已经是经典步态”，也不能直接视作成功。

此前每次 enable 和运动起步都强制请求经典步态，会让机器人拒绝请求时
连带阻止导航。现在默认不改变操作者步态，消除这条不必要的依赖。

## 显式请求与诊断

仍保留 `request_classic_once` 策略，供确实需要自动请求的受控测试：
在启动前设置 `GO2_GAIT_POLICY=request_classic_once`。它只在 enable 时
请求一次，失败保持禁用，没有自动重试或默默降级。启动后切换策略需
重启导航进程。日常使用无需设置此变量。

显式请求用与原封装相同的已注册 API 及 JSON 参数，并保留响应正文。
`/go2/diagnostics` 增加 `gait_policy`、`last_classic_request_result`、
`last_classic_response`；enable 错误包含返回码和正文。传输错误仍按
SDK 返回码识别，机器人不给正文时明确保留为空。

实时地形健康门仍可能阻止导航，与经典步态请求是独立条件。
修改经过构建和无硬件回归验证，不代表已经验证固件持续行走中的步态。
