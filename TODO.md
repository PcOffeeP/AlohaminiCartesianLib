# 目标与待办

本文件只记**当前活跃的未完成事项**。设计取舍记在 `docs/ADR.md`，架构导览见 `docs/ARCHITECTURE.md`，现场数字记在 `docs/field-results.md`，已关闭与低优先级边角工程债归档在 `docs/archive/engineering-debt-archive-2026-11.md`。

状态只有三种：`未开始` / `进行中` / `受阻`。

## 1 整体目标

把 AlohaMini 2 Pro 从「遥操作主臂牵着走」改成「PC 上 Python 库直接按笛卡尔目标下发」，底层复用厂商发送链与硬件保护。

- **完成判据**：`docs/acceptance-baseline.md` 的 **V01–V13** 实机证据列全部填实（逐条统计以 `docs/acceptance-baseline.md` §2.3 为准）。

## 2 阶段总览

| 阶段 | 目标 | 完成判据 | 状态 |
| --- | --- | --- | --- |
| A 单臂闭环与标定 | 完成新代际校验、`signs`/`zeros` 标定、`move_to` 真机闭环与卷尺验证 | 新代际 V12 过、E8 标定落地、E7 `move_to` 真机跑通 + 卷尺误差落毫米级 | 进行中（V01 单关节追踪已按 D20 通过；剩余项等下一轮现场窗口） |
| B 双臂独立验证 | 右臂复现 A 阶段结论，并证明左右臂与夹爪互不串值 | V02 实机证据填实 | 未开始（前置：A 阶段收口） |
| C 整机协同验收 | 底盘、升降与双臂分阶段互锁、停止恢复及超时保护真机验证 | V03 / V04 / V05 / V07 / V08 实机证据填实 | 未开始（前置：B 阶段收口） |
| D 库可交付与发版 | 开源合规、CLI 工具、文档导览与 `v0.8.0` 发版门禁收口 | `LICENSE`/`NOTICE`、示例、CLI、实机证据满足 D19 发版红线 | 进行中（离线软件、许可、V13 与 CLI 已完成；`v0.8.0` 发版等实机证据） |

## 3 下一轮现场窗口待办（Phase 3，按执行顺序）

> 现场操作均须单独拿到执行授权，并按 `docs/field-ops/README.md` 固定链推进；本清单不构成执行授权。

| 顺序 | 待办事项 | 完成判据 | 影响 | 当前阻塞 |
| --- | --- | --- | --- | --- |
| 1 | 核验 `tools/calib.py fetch` 真实 SSH 抓取路径与现役代际（`4a80c0fd4421`）包络 | 实机抓回文件与 Pi 原件 `sha256` 一致；核实 `arm_right_shoulder_lift` 1143 tick 行程决定记入接受名单或重标定 | V12 前置、D23 | 需现场连机 + torque 释放态推臂授权 |
| 2 | 新代际（`4a80c0fd4421`）重取 v2 指纹并重跑 V12 单位一致性核对 | `capture_robot_fingerprint.py` 取得新代际 v2 指纹，`tools/verify_unit_consistency.py` 18/18 差异 0、exit `0` | V12 实机证据（现为「待重取」） | 需现场连接运行中的 Host |
| 3 | 执行 E8 标定产出左右臂 `signs` / `zeros` 映射（`tools/calibrate_kinematics.py`） | 左右臂各产出一份 `signs` / `zeros` JSON 并通过 FK 姿态自检 | 解除 E7 `move_to` 实机验证阻塞 | 需现场人工摆臂 + Host 读数（见 `docs/field-ops/runbook/4-experiments.md#exp-e8`） |
| 4 | 执行 E7 `ArmHandle.move_to` 真机闭环与 `+Z` 9 cm 卷尺实测（`tools/run_cartesian_move.py`） | ① `--limits-only` 过；② `+Z` 9 cm 闭环完成且确认无 D24 前的腕部翻转/步长死锁；③ 卷尺实测与指令偏差验证 URDF 杆长 | 笛卡尔入口实机可用性、D21、D24 | 前置：第 3 项 `signs`/`zeros` 到位 + 现场人守断电开关（见 `docs/field-ops/runbook/4-experiments.md#exp-e7`） |
| 5 | 完成右臂受控试动与双臂/夹爪独立性验证 | 动左/右一侧时对侧观测与动作键零串值；现役左夹爪（1465 tick）与右夹爪开合验证通过 | V02 实机证据、B 阶段 | 前置：A 阶段收口 |
| 6 | 完成底盘/升降/臂分阶段互锁、停止恢复与反馈超时真机验证 | 跨阶段请求被拒且当拍零发送；`stop` / `resume` / 断流超时实机留痕 | V03 / V04 / V05 / V07 / V08、C 阶段 | 前置：B 阶段收口 |

## 4 剩余裁决与外部依赖项

> 原挂于本节的 **V06 过流保护实机取证** 与 **V10 拒绝子项收口** 已由 `docs/ADR.md` D26 正式裁决收口，并同步更新 `docs/acceptance-baseline.md`。

| 事项 | 需决定的问题 / 完成条件 | 影响 | 状态 |
| --- | --- | --- | --- |
| `docs/field-ops/` 手册串口/核对命令现场复核 | 下一轮现场顺手逐条核对剩余设备路径与 grep 命令，纠正不一致处 | 现场手册可执行性 | 未开始 |
