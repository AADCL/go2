# 文档索引

日常建图与导航请先看根目录的 [启动手册](../STARTUP_GUIDE.md)。本目录按用途归档，历史记录保留当时的机器人编号、路径、参数和验证结论。

## 当前 Nano 版本

- 分支：`unitree-orin-nano`；端侧工作空间：`/home/unitree/go2_nav_ws`。
- 已实测快照：`nano-tested-20260911`（`70f5cbe`）。本开发分支 `codex/nano-shutdown-stop-20260912` 新增正常退出停车修复，尚待实机部署及步态复测；地图、地形参数和启动命令未改动。
- 修改前快照：`nano-before-20260911`。已有标签和 Git 提交历史保留。
- 更晚的地面支持检查优化位于独立开发分支 `codex/nano-ground-check-20260911`，未随本次文档整理合入。
- 地图名称是用户自定义参数，例如 `lab_202609101805`；实际使用时替换为对应地图目录名。

建议先阅读以下说明，了解当前代码相对早期部署的变化：

| 文档 | 内容 |
| --- | --- |
| [完整启动手册](../STARTUP_GUIDE.md) | 建图、保存、导出、定位、导航和使能 |
| [步态与 TF 修复](fixes/GAIT_TF_FIX_20260911.md) | 普通停车与经典步态保持的修复记录 |
| [正常退出停车修复](fixes/SHUTDOWN_STOP_FIX_20260912.md) | 退出时的停车确认、退出信号处理与故障兜底 |
| [实时地形检查调整](fixes/TERRAIN_RELAXED_CHECKS_20260911.md) | 当前测试配置、短时高度容忍、验证和回退 |
| [二维地图导出更新](deployment/DEPLOYMENT_ROBOT2_V210.md) | 第二条狗的 V2.1.0 导出部署与验证 |
| [第三方来源](../THIRD_PARTY.md) | 算法、驱动与 SDK 的来源和版本 |

修复记录中的“待复测”“未重启”等描述是记录编写时的状态；它们不表示机器人此刻正在运行什么。现场状态需通过 `run_go2 status` 和实际启动参数确认。

## 部署记录 · deployment

- [Robot 2 初始地形导航 V2 部署](deployment/DEPLOYMENT_ROBOT2_V2.md)
- [Robot 2 V2.1.0 地图导出更新](deployment/DEPLOYMENT_ROBOT2_V210.md)
- [Robot 1 动态建图与坡度规划部署](deployment/DEPLOYMENT_TERRAIN_20260909.md)
- [Robot 1 地图完整度修复部署](deployment/DEPLOYMENT_MAP_COMPLETENESS_FIX_20260909.md)
- [Robot 1 地形导出失败修复部署](deployment/DEPLOYMENT_TERRAIN_EXPORT_FIX_20260909.md)

## 修复记录 · fixes

同一主题的多份记录对应不同阶段。当前实时地形测试门槛以 2026-09-11 的“实时地形检查调整”及仓库配置为准，早期记录用于追溯原因。

- [步态请求返回 -1 与 enable](fixes/CLASSIC_GAIT_FIX.md)
- [普通停车步态与 TF 发布](fixes/GAIT_TF_FIX_20260911.md)
- [右转停车与地面覆盖检查](fixes/TERRAIN_COVERAGE_FIX_20260911.md)
- [实时地面高度拟合](fixes/TERRAIN_HEIGHT_FIX_20260911.md)
- [实时地形检查调整与复测](fixes/TERRAIN_RELAXED_CHECKS_20260911.md)
- [Robot 1 通用地形导出修复](fixes/TERRAIN_EXPORT_REVISION2_20260910.md)

## 专题与参考

- [地形优化说明](guides/TERRAIN_OPTIMIZATION_GUIDE.md)：动态建图、离线地形导出和全局坡度规划；基于机器狗 1 的历史说明，执行示例时应按 Nano 路径和当前参数调整。
- [WheelTech 算法对比](reference/WHEELTECH_ALGORITHM_COMPARISON_20260909.md)：算法来源、系统差异与后续建议。
- [历史设计与实施计划](superpowers/)：研发过程记录，不作为现场启动操作入口。
- `assets/`：README 等文档使用的图片资源。

## 历史发布 · releases

- [V2.0.0](releases/RELEASE_NOTES_V2.0.0.md)
- [V2.0.1](releases/RELEASE_NOTES_V2.0.1.md)
- [V2.1.0](releases/RELEASE_NOTES_V2.1.0.md)

以上发布说明保留当时的机器狗 1 基线；当前 Nano 分支还包含后续适配与修复，不应只按旧版本号判断功能。
