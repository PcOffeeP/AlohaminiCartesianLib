# 离线调查：左夹爪标定行程 51 tick

> 2026-11 离线调查：左夹爪标定行程 51 tick 的查证过程与结论。决策见 `docs/ADR.md` D14；数字见 `docs/field-results.md`。**本节不是现场采集**：没有连机器人，没有新增任何测量。

## 离线核对（无新采集）· 左右夹爪行程差异

**这一节不是现场采集**：没有连机器人，没有新增任何测量。数字来自两份既有证据文件的复算，加两端源码的静态核对（本库 `src/` 与厂商 fork `lerobot_alohamini` 只读）。

数据源：

- 标定文件 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/calibration-AlohaMiniRobot.json`（18 个电机）
- 真机指纹 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/capture/fingerprint-final-20261025.json`（`robot_metadata.motors` 与 `observation_state`）
- 寄存器只读转储 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/stdout/reg_read_1792923639.log`（左臂 ID 1/2/3/6，**不含夹爪 ID7**）

### 结论

左夹爪 `arm_left_gripper` 的 `range` 只有 51 tick（`range_0_100` 下 4.48° 行程），判为**异常**；根因**未确认**。

- 配置口径已排除：两侧夹爪同为 `sts3250`、`drive_mode` 同为 0、`normalization` 同为 `range_0_100`、`resolution` 同为 4096。
- 数据传递已排除：标定文件与指纹 metadata 在 18 个电机 × 5 个字段（`id`/`drive_mode`/`homing_offset`/`range_min`/`range_max`）上逐项比对，0 差异。
- 数值位置指向标定时刻：左夹爪 `range` 中心 2046.5，等于半圈归零的目标位 2047（`feetech.py:287`，`half_turn_homings = pos - int(4095/2)`）。即标定采集窗口内该夹爪只在归零位上下 ±26 tick 内抖动。

### 左右夹爪逐项对照

| 字段 | 左 `arm_left_gripper` | 右 `arm_right_gripper` | 判读 |
| --- | --- | --- | --- |
| `id` | 7 | 7 | 相同 |
| `model` | `sts3250` | `sts3250` | 相同 |
| `drive_mode` | 0 | 0 | 相同 |
| `normalization` | `range_0_100` | `range_0_100` | 相同 |
| `resolution` | 4096 | 4096 | 相同 |
| `homing_offset` | 1571 | -1964 | 不同，但只决定「哪一处物理位置读成 2047」，不解释行程差 |
| `range_min` | 2021 | 2010 | 相差 11 tick |
| `range_max` | 2072 | 3479 | 相差 1407 tick |
| `range` 宽度 | **51 tick** | **1469 tick** | 右是左的 28.8 倍 |
| `deg_per_unit`（`detect.py:298`） | 0.04484 | 1.29143 | 换算公式 `((range_max-range_min)/span)*360/(resolution-1)`，`span` 取 100（`capability/detect.py:16-18`） |
| 满 `0..100` 对应的角度行程 | **4.48°** | **129.14°** | 同型电机差 28.8 倍 |
| `range` 中心 | 2046.5 | 2744.5 | 左夹爪整段落在归零位 2047 附近 |
| 采集时观测值 `observation_state` | `arm_left_gripper.pos` = 45.098 | `arm_right_gripper.pos` = 2.859 | 均落在各自 `[range_min, range_max]` 内，归一化本身无异常 |

### 18 个电机 `range` 宽度全表（按宽度升序）

| 电机 | id | 型号 | `range_min`-`range_max` | 宽度 | `normalization` | 满量程对应角度 | 判读 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `arm_left_gripper` | 7 | sts3250 | 2021-2072 | **51** | `range_0_100` | 4.48° | **唯一离群** |
| `arm_right_gripper` | 7 | sts3250 | 2010-3479 | 1469 | `range_0_100` | 129.14° | 同型参照 |
| `arm_right_wrist_yaw` | 5 | sts3250 | 1053-2879 | 1826 | `range_m100_100` | 160.53° | 正常 |
| `arm_left_wrist_yaw` | 5 | sts3250 | 1114-3070 | 1956 | `range_m100_100` | 171.96° | 正常 |
| `arm_left_wrist_flex` | 4 | sts3250 | 934-3049 | 2115 | `range_m100_100` | 185.93° | 正常 |
| `arm_right_wrist_flex` | 4 | sts3250 | 969-3091 | 2122 | `range_m100_100` | 186.55° | 正常 |
| `arm_right_elbow_flex` | 3 | sts3095 | 965-3105 | 2140 | `range_m100_100` | 188.13° | 正常 |
| `arm_left_elbow_flex` | 3 | sts3095 | 947-3109 | 2162 | `range_m100_100` | 190.07° | 正常 |
| `arm_left_shoulder_lift` | 2 | sts3095 | 930-3229 | 2299 | `range_m100_100` | 202.11° | 正常 |
| `arm_right_shoulder_lift` | 2 | sts3095 | 939-3258 | 2319 | `range_m100_100` | 203.87° | 正常 |
| `arm_right_shoulder_pan` | 1 | sts3250 | 1247-3743 | 2496 | `range_m100_100` | 219.43° | 正常 |
| `arm_left_shoulder_pan` | 1 | sts3250 | 627-3327 | 2700 | `range_m100_100` | 237.36° | 正常 |
| `arm_left_wrist_roll` | 6 | sts3250 | 0-4095 | 4095 | `range_m100_100` | 360° | 未采真实行程，硬写默认值 |
| `arm_right_wrist_roll` | 6 | sts3250 | 0-4095 | 4095 | `range_m100_100` | 360° | 同上 |
| `base_left_wheel` | 8 | sts3250 | 0-4095 | 4095 | `range_m100_100` | 360° | 按设计硬写默认值 |
| `base_back_wheel` | 9 | sts3250 | 0-4095 | 4095 | `range_m100_100` | 360° | 同上 |
| `base_right_wheel` | 10 | sts3250 | 0-4095 | 4095 | `range_m100_100` | 360° | 同上 |
| `lift_axis` | 11 | sts3095 | 0-4095 | 4095 | `degrees` | 360°（`degrees` 下 `deg_per_unit` 恒为 1.0，角度列只是刻度等价） | 同上 |

读法：「真实行程」一栏只看宽度落在 1469–2700 之间的 11 台电机。宽度 4095 的 6 台不是采到的行程，是厂商标定里直接写死的 `0..4095`（`alohamini.py:441-453` 三只底盘轮 + `arm_left_wrist_roll`，`alohamini.py:466-479` 对 `arm_right_wrist_roll`）。在这 11 台真实行程里，左夹爪 51 tick 比第二窄的（右夹爪 1469）小 28.8 倍，比最窄的非夹爪关节（`arm_right_wrist_yaw` 1826）小 35.8 倍。离群的是这一台夹爪，不是夹爪这一类。

### 逐条查证

| 问题 | 结论 | 依据 |
| --- | --- | --- |
| 夹爪在标定协议里的行程预期是多少 | 代码里**没有**给夹爪设过角度预期，只给了一个归一化区间 `[0, 100]`；区间宽度由 `range` 反推，因此「预期行程」就是「标定采到的 `range`」本身，没有独立校验 | 本库 `control/commands.py:103-104` 只校验 `0 <= position <= 100`；`control/api.py:130-136` 把开映射到 `100.0`、合映射到 `0.0`；`capability/detect.py:16-18` 定义 `range_0_100 → span 100`、`range_m100_100 → span 200`、`degrees → 1.0` |
| `normalization` 分类各意味着什么 | `range_0_100` 表示线上值 = `(raw - range_min)/(range_max - range_min) * 100`，即 0 对应 `range_min`、100 对应 `range_max`；`range_m100_100` 同理但映射到 -100..100 | 厂商 `motors_bus.py` 的 `_normalize`（`range_0_100` 分支）与 `capability/detect.py:195-197` 的表 |
| 4.48° 对两指夹爪说得通吗 | 说不通。同型电机的右夹爪同一机构、同一 `drive_mode`，标定采到 129.14°；左夹爪只有它的 3.5%。具体开口毫米数**未采集**（缺连杆长度/丝杆导程参数），这里不给毫米估算 | 对照表；无毫米数据的部分标为未采集 |
| `drive_mode` 两侧是否相同 | 相同，都是 0。不同的话会在 `_normalize`/`_unnormalize` 里对 0..100 做 `100 - norm` 的镜像，表现为开合方向相反，与行程宽度无关 | 对照表；厂商 `motors_bus.py:872-874`（`range_0_100` 分支的 `100 - norm if drive_mode`） |
| 「标定时夹爪处于几乎闭合位」是否合理解释 | 是当前最合理解释，且与数值吻合：vendor 标定把夹爪放进 `unknown_left` / `unknown_right`，靠人工开合、边采边记 min/max；采到的左夹爪只在归零位 ±26 tick 内，说明采集窗口内它基本没离开过标定起始姿态。至于「没开」是因为人没去开、还是机械不允许开，**未确认** | `alohamini.py:444`（`unknown_left` 含 `arm_left_gripper`、排除项只有三只底盘轮与 `arm_left_wrist_roll`）、`:450`（`record_ranges_of_motion`）、`:476`（右臂同流程）；`motors_bus.py:802-861` 采样实现 |
| 采样不足能否解释 51 tick | 不成立。`record_ranges_of_motion` 是紧凑 `while` 循环里反复 `sync_read("Present_Position")`（`motors_bus.py:825-843`），中间没有 sleep；人工开合一次至少跨数百个采样周期。该循环的**真实采样周期未在真机总线上实测**，这一句标为未确认，但方向上不支持「采样漏点」解释 | `motors_bus.py:802-861` |
| 标定流程本身有没有兜底 | 有一条：`min == max` 会抛 `ValueError`（`motors_bus.py:857-860`）。本次 `51 ≠ 0` 所以没触发。也就是说，一个「动了几 tick」的夹爪能顺利通过标定校验并写盘，流程只能拦住完全没动的情形 | `motors_bus.py:857-860` |
| 这个 `range` 会落到硬件上吗 | 会。`write_calibration` 把 `range_min`/`range_max` 写进伺服的 `Min_Position_Limit`/`Max_Position_Limit`（位置寄存器 9 / 11）。左臂 ID 1/2/3/6 的实测寄存器值与标定文件逐项相同，说明这条写入链在本机上确实生效。**ID7 未读**，所以「左夹爪寄存器也是 2021/2072」是推断，未直接观测 | 厂商 `feetech.py:268-273`（写）、`:247-251`（读）、`tables.py:50-51`（寄存器地址）；`reg_read_1792923639.log` 中 ID1 的 `Min_Position_Limit=627`、`Max_Position_Limit=3327` 与标定文件一致 |
| 指令超出行程会怎样 | 两端都会静默截断，不报错。本库把夹爪位置钳在 `[0, 100]`；厂商 `_normalize` 把读数钳在 `[range_min, range_max]`；`_unnormalize` 把指令钳在区间内。若 ID7 的固件限位真是 2021/2072，伺服本身也拒绝走出这个窗口 | 本库 `control/commands.py:103-104`；厂商 `motors_bus.py:864-870`（`bounded_val`）与 `_unnormalize` |

### 可能原因排序（都不算已确认）

| 排序 | 可能原因 | 支持点 | 缺口 / 反证 |
| --- | --- | --- | --- |
| 1 | 标定流程／操作层：标定窗口内左夹爪没被开合 | 采到的 `range` 中心等于归零位 2047、宽度只有 51 tick；厂商流程靠人工开合采集，`51 ≠ 0` 所以过了校验 | 没有标定会话的输出记录或录屏；ID7 寄存器未读 |
| 2 | 机械层：左夹爪被结构／线束／绑带限住，物理上张不开 | 同样能解释「±26 tick 抖动」；若真被限住，标定时人工开合也不会有读数变化 | 无任何机械侧证据；未到现场 |
| 3 | 数据层：标定值写进 ID7 失败或写错 | 理论上可能，但 18 电机 × 5 字段 0 差异，且同总线上 ID1/2/3/6 寄存器与文件完全一致 | **ID7 在本次转储里没被读**，无法排除；需要现场只读补测 |
| 4 | 机构差异：左夹爪减速比／传动与右不同 | 无 | 两侧同为 `sts3250`、`drive_mode` 同为 0，无支持证据 |

### 影响与控制侧要注意的

- 左夹爪在本库侧只有 4.48° 可用行程：`open()`（100）与 `close()`（0）命令物理上分不开夹爪，而链路不会报错（见上「指令超出行程会怎样」）。这是**静默失效**，比报错更需要记住。
- 若将来改成按角度下发夹爪：要让左夹爪张开 100°，按 `deg_per_unit = 0.04484` 需 2231 个归一化单位，远超 `[0, 100]`，必然被钳到端点。夹爪下发继续用 0..100，不要换角度。
- 不影响 A 阶段：夹爪不是运动学关节（`kinematics/joint_adapter.py:54-55`、`:64-65` 明确 gripper 不进关节向量）。
- 不影响 Gate F：`range_max > range_min`、换算字段齐全，第二段不会因这件事拒绝。既有测试对这一条没有预期变化。

现场只读确认步骤见 `docs/field-ops/runbook/3-health.md#sec-4-2` 第 4.2 节「左夹爪 ID7 只读确认」。

