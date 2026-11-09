# 现场验证结果台账

按时间倒序。每行一条结论，数字可在「证据文件」里核对。证据文件都在仓库外的证据目录：

- 2026-11-08 节：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261107/`（目录名按授权所取，实际 UTC 日期 11-08）
- 2026-11-08 节的第二段（路 A 复核）：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261108/`（目录名与实际 UTC 日期一致，与上一行的 `s4-20261107/` 不同）
- 2026-11-06 节：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/`
- 2026-11-05 节：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261104/`
- 2026-10-25 及更早：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/`（索引 [evidence/s4-20261025/README.md](file:///Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/README.md)）

下表「证据文件」列给的是**该节所属目录**下的相对路径（各节首段的「条件」一句已写明本节用哪个目录）。逐条命令、判据在 [docs/field-ops/README.md](field-ops/README.md)，退出码表在 [docs/field-ops/50-experiments.md](field-ops/50-experiments.md) §10，本文件只记拿到了什么。

## 2026-11-08（现场验收日）· S4 现场窗口（第 4 次）

条件：AlohaMini 2 Pro（`robot_model=alohamini2pro`），18 个电机，Pi 上 lerobot `0.6.1`，用户在机器人旁且手边可按断电开关，单臂、空场、无负载；本轮**有运动命令**（`--anchor command`，左右 `shoulder_pan` 各 5 步 × 1°）。STAMP `20261108T125734Z`；Host 起 `12:58:42Z`（盘上首次留痕）、停 `13:08:29Z`（`kill -INT`）。证据目录（仓库外）为 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261107/`，下表「证据文件」列都是这个目录下的相对路径。

**目录命名说明**：目录名 `s4-20261107` 是按授权所取，机器 `date -u` 是 `2026-11-08`，因此目录内 STAMP 与全部时刻都是 11-08 UTC。目录名不改，本节按真实 UTC 日期记账（`stdout/scope-append-20261108T125734Z.md` 偏差 ①）。

### 结论一览

| 结论 | 关键数字 | 条件 | 证据文件 |
| --- | --- | --- | --- |
| **肩部 anchor=command 跟随成立**：左右各 5 步 × 1°，达成率均 `77.3626%` | 左 `arm_left_shoulder_pan` 实测 `14.681319° → 18.549451°`，净走 `3.868132°`（指令 `5.0°`），`(18.549451 − 14.681319) / 5.0 = 77.3626%`；右 `arm_right_shoulder_pan` 实测 `−14.065934° → −10.197802°`，净走 `3.868132°`，同 `77.3626%`。两侧 `steps_completed=5`、`abort_reason=None`、`.exit` 均 `0` | 单关节 `--mode tracking --anchor command --delta-deg 1.0 --steps 5 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0`；单臂、空场、无负载、用户在旁 | `tracking/tracking-2.5-{left,right}-anchor-command-20261108T125734Z.csv`、`stdout/7.1-tracking-{left,right}-shoulder-anchor-command-20261108T125734Z.{log,exit}` |
| **对照 11-06 `--anchor present` 同轴 10 步，提升约 5–7 倍** | `77.3626% / 15.82% = 4.89`（左）、`77.3626% / 11.43% = 6.77`（右） | 两轮只差 `--anchor`（与步数 10→5）；11-06 同为 settle 2 s 窗、容差 0.05° | 对照数字见本文件 2026-11-06 节「双肩 pan 达成率」行；本轮见上 |
| **「lead 累积破静摩擦」机理真机确证** | stdout 首两行原文 `note: anchor=command: each step anchors on the previous target, so the lead accumulates across steps`；5 步累计 lead `5.0°`，与 11-06「1° 单步在 15 s 长窗里肩只走 `17.582%`」互补 | 同上 | `stdout/7.1-tracking-left-shoulder-anchor-command-20261108T125734Z.log`（第 2 行）、`stdout/7.1-tracking-right-shoulder-anchor-command-20261108T125734Z.log`（第 2 行） |
| **§2.5 补证判据三项全过 → 补证状态：通过（anchor=command 形态）** | ① 运动帧 `requested_target == accepted_target`：左 `6258/6258`（100.00%）、右 `6174/6174`（100.00%）；② 每关节达成率 `77.3626% ≥ 70%`；③ 左右同值，无一侧不过 | ① 只数 `requested_target` 与 `accepted_target` 双非空的行；② 达成率 = 净行程 ÷ 指令行程 | `tracking/tracking-2.5-{left,right}-anchor-command-20261108T125734Z-safety.csv`、主 CSV、`stdout/7.1-tracking-*.{log,exit}` |
| **边界实录：步末残余未收敛，V01 主表「残余 ≤2 tick」支仍不过** | 两侧每步 `settled=False`、`note=settle budget exhausted`；步末 `error_deg` 均 `1.131868°`（`12.9 tick`），远大于门槛 `2 tick`（`0.1758°`） | 同上 | 主 CSV 第 5 步（`step_index=4`）行 |
| 收尾 hold 生效：§2.5 第 4 证据项以寄存器净行程 + CSV 旁证 | 左 `shoulder_pan` 寄存器 `Goal_Position == Present_Position = 2187`；运动前指纹 `2144` → 断电前 `2187`，净 `+43` tick；同期 CSV 净走 `3.868132°` = `+44` tick（`1 tick = 360/4096 = 0.087890625°`），相差 1 tick 来自寄存器读数与 CSV 末帧不同时刻 | 运动前指纹 `13:03:53Z`；断电前指纹 `13:08:18Z`（停 Host 前）；寄存器只读 `13:09:12Z` | `capture/fingerprint-20261108T125734Z.json`、`capture/fingerprint-postpower-20261108T125734Z.json`、`stdout/4.1c-postpower-diff-20261108T125734Z.txt`、`stdout/4.2-all18-left-20261108T125734Z.log` |
| B1 标定字段：`CALIB_FIELD_DIFFS: 0` | 与 11-06 收工基线 `fingerprint-postpower-20261106T070942Z.json`（`captured_utc=2026-11-06T07:10:31+00:00`）逐字段比，标定字段差异 `0` | 只读两份指纹 JSON，不连设备 | `stdout/4.1c-postpower-diff-20261108T125734Z.txt`（A 组）、`stdout/4.1c-selfcheck-20261108T125734Z.txt`（自检两组计数均 0） |
| B1 位置组：12 只超 3 tick，全部归因 11-06 六轮记录在案的运动 | `POSITION_MOTORS_OVER_3_TICK: 12`；超阈中最大四只：`arm_left_elbow_flex −999`、`arm_right_elbow_flex −864`、`arm_right_wrist_roll +467`、`arm_left_shoulder_lift +392`；两把夹爪 `ok`（`−1` / `+0`）。唯左 `shoulder_pan` 观测 `−30` tick，其中 `−11.4` tick 锚定 11-06 `slowvsstall` 那轮未 settle 的单步（该单步行程未实测，**归因算式未在证据目录留原文，属现场报告转述、未验证标注**） | 该基线采于 11-06 `07:10:31Z`，之后有 07:19–09:06Z 六轮记录在案的运动，故 B 组差值只能归因、不作「断电滑移」判据 | `stdout/4.1c-postpower-diff-20261108T125734Z.txt`（B 组）、本文件 2026-11-06 节「到场窗口（有人在场）· 肩部定性」 |
| 新基线换账 | 运动前基线 `capture/fingerprint-20261108T125734Z.json`（`captured_utc=2026-11-08T13:03:53+00:00`，`schema_version=2`，`lerobot_version="0.6.1"`）；断电前指纹 `capture/fingerprint-postpower-20261108T125734Z.json`（`captured_utc=2026-11-08T13:08:18+00:00`） | capture 在 Pi 侧跑（指纹 `host=127.0.0.1`），`scp` 取回；主环境无 `lerobot` 时 Mac 侧那步必然 exit `3`（`stdout/8-fingerprint-before-poweroff-*.exit` 原文 `3`） | `capture/fingerprint-20261108T125734Z.json`、`capture/fingerprint-postpower-20261108T125734Z.json`、`stdout/4.1-capture-*`、`stdout/8-fingerprint-before-poweroff-*` |
| B3 全 18 电机静态体检（停 Host 后只读） | 18/18 `Model_Number` 与配置型号对上（`sts3250=2825`、`sts3095=2569`，无 `sts3215`）；`Status` 全 `0`；`Present_Voltage` 原始值 min `116` / max `119`（源码侧已核：地址 62、1 字节、无符号、全链无换算，读到什么就是什么；`11.6–11.9 V` 用 `0.1 V/单位` 的比例算，该比例来自 Feetech 手册、源码不提供，故换算值只作相对比较，不得当标定或实测证据）；18 只 `Torque_Enable` 全 `0`（与 `kill -INT` 释放扭矩一致） | 左总线 11 只 + 右总线 7 只，两份日志均 `EXIT_CODE: 0` | `stdout/4.2-all18-{left,right}-20261108T125734Z.{log,exit,cmd,py}` |
| Host 收尾路径与 11-06 一致 | `kill -INT 1985`、`kill_exit=0`；`pgrep`/`ss` 无 LISTEN；Host 日志末两行 `Keyboard interrupt received. Exiting...` / `Shutting down AlohaMini Host.` | `13:08:29Z` 停，`13:08:31Z`、`13:08:43Z` 两次复核 | `stdout/8-host-stop-20261108T125734Z.log`、`stdout/8.1-host-stopped-20261108T125734Z.log` |

### 路 A：V01 残余支与达成率分母复核（STAMP 20261108T141936Z）

条件：与本节上一段同一台机（`robot_model=alohamini2pro`、18 个电机、Pi 上 lerobot `0.6.1`）；用户在机器人旁，可按断电开关；本段**有运动命令**：左右 `shoulder_pan` 各跑一轮 `--mode tracking --anchor command --delta-deg 1.0 --steps 5 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 15`（settle 窗 15 s、容差 `0.05°`）。STAMP `20261108T141936Z`。证据目录（仓库外）为 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261108/`，本节表格「证据文件」列都是这个目录下的相对路径。

#### 表 1：B1 位置组首判「断电无滑移」

基线 = 上一场停 Host 前的断电前指纹 `s4-20261107/capture/fingerprint-postpower-20261108T125734Z.json`（`captured_utc=2026-11-08T13:08:18+00:00`）；今天 = 本场运动前的开机后指纹 `capture/fingerprint-postpower-20261108T141936Z.4.1c-preserved.json`（`captured_utc=2026-11-08T14:20:35+00:00`）。判据形态：两次上电之间隔了一整次断电，期间没有运动，位置若没滑移即成立——这是**第一份不受「期间有运动」污染的 B1 判据**（11-08 上一场那份的基线之后有 11-06 六轮运动，只能归因、不能作判据）。

| 电机 | `baseline_tick` | `today_tick` | `delta`（tick） |
| --- | --- | --- | --- |
| `arm_left_elbow_flex` | `2097` | `2097` | `+0` |
| `arm_left_gripper` | `2047` | `2047` | `+0` |
| `arm_left_shoulder_lift` | `1323` | `1323` | `+0` |
| `arm_left_shoulder_pan` | `2187` | `2186` | `-1` |
| `arm_left_wrist_flex` | `3048` | `3048` | `+0` |
| `arm_left_wrist_roll` | `1906` | `1906` | `+0` |
| `arm_left_wrist_yaw` | `2096` | `2096` | `+0` |
| `arm_right_elbow_flex` | `2222` | `2222` | `+0` |
| `arm_right_gripper` | `2010` | `2010` | `+0` |
| `arm_right_shoulder_lift` | `1034` | `1034` | `+0` |
| `arm_right_shoulder_pan` | `2378` | `2378` | `+0` |
| `arm_right_wrist_flex` | `3090` | `3090` | `+0` |
| `arm_right_wrist_roll` | `2231` | `2231` | `+0` |
| `arm_right_wrist_yaw` | `2019` | `2019` | `+0` |

结果：`CALIB_FIELD_DIFFS: 0`；14 只位置电机 `delta` 全部 `+0`，**只有** `arm_left_shoulder_pan` 为 `-1` tick；`POSITION_MOTORS_OVER_3_TICK: 0`。同一份文件的 C 组（`lift_axis.height_mm: 0.0 -> 0.0` 等）原样比、不折 tick。证据文件 `stdout/4.1c-postpower-diff-20261108T141936Z.txt`。

#### 表 2：15 s 窗实测（路 A 主跑）

| 侧 | 关节 | 净行程 | 步末残余 `deg` | 残余 tick | `settled` | `rise_90` 非空步数 | 达成率（分母 `abs(delta)×steps = 5.0`） | 达成率（旧分母 `Σ(cmd−baseline)`） | 按 D20 判定 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 左 | `arm_left_shoulder_pan` | `3.956044°` | `1.043956°` | `11.88` | `0/5` | `3/5` | `79.1209%` | `43.0108%`（分母 `9.197802`） | 过 |
| 右 | `arm_right_shoulder_pan` | `4.131868°` | `0.868132°` | `9.88` | `0/5` | `4/5` | `82.6374%` | `48.6417%`（分母 `8.494505`） | 过 |

两侧 `steps_completed=5`、`abort_reason=None`、`.exit` 均 `0`；`settled=False` 的 `note` 全为 `settle budget exhausted`。换算常数 `1 tick = 360/4096° = 0.087890625°`；本轮 stdout 给的 `1 unit`：左 `1.1868131868131868 deg`、右 `1.0971428571428572 deg`。运行量：左 `commands=3009 observations=3008 over 75.289 s`、右 `commands=2957 observations=2956 over 75.287 s`。证据文件 `tracking/tracking-v01residual-{left,right}-anchor-command-20261108T141936Z.csv`、`stdout/7.1-tracking-{left,right}-shoulder-v01residual-20261108T141936Z.{log,exit}`、`stdout/7.2-analysis-{left,right}-20261108T141936Z.txt`。

#### 表 3：sidecar 判读

| 侧 | 数据行 | 帧数 | 运动窗口（`sample_sequence >= 3`）`target_source` | `requested_target == accepted_target` | `hold_active` | `current_ma` max | `>= 20 mA` 样本 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 左 | `54144` | `3008`（每帧 18 行） | `3007` 帧全 `command` | `42098/42098 = 100.0000%` | 全 `False`（`54144/54144`） | `104.0` | `700` |
| 右 | `53208` | `2956`（每帧 18 行） | `2955` 帧全 `command` | `41384/41384 = 100.0000%` | 全 `False` | `71.5` | `153` |

`requested == accepted` 一列数的是 `requested_target` 与 `accepted_target` 双非空的行（其余是底盘三轮与升降轴，Host 侧本就没有目标表项）。右侧 `sample_sequence=2` 那一帧为上一轮遗留的 `watchdog`（不计入本轮；左侧同一帧为 `none`）。两侧判读都落入 `docs/field-ops/40-gates-params.md` §7.2 结论①：**目标已被 Host 采纳，残余在 Host 下游的伺服／机械静摩擦侧**。证据文件 `tracking/tracking-v01residual-{left,right}-anchor-command-20261108T141936Z-safety.csv`、`stdout/7.2-analysis-{left,right}-20261108T141936Z.txt`。

**机理与结论**：厂商写 `P_Coefficient=16` / `I_Coefficient=0`（`alohamini.py:537`/`:539` 左、`:564`/`:565` 右），没有积分项，位置环只能靠力平衡收尾，稳态残余必然留下、延长 settle 窗也消不掉。残余只有 `0.87–1.04°`（`9.88–11.88 tick`），两侧同量级，**「动作慢 / 跟不上」这个解释被排除**。残余支的处置定为分级口径，见 `docs/ADR.md` D20。

#### 指纹三份

| 文件 | `captured_utc` | 用途 |
| --- | --- | --- |
| `capture/fingerprint-20261108T141936Z.json` | `2026-11-08T14:20:21+00:00` | 本场开机后 §4.1 过程指纹 |
| `capture/fingerprint-postpower-20261108T141936Z.4.1c-preserved.json` | `2026-11-08T14:20:35+00:00` | §4.1c 对比用的那一份（表 1 的「今天」） |
| `capture/fingerprint-postpower-20261108T141936Z.json` | `2026-11-08T14:28:12+00:00` | 运动后、停 Host 前的收尾指纹 |

**同名冲突如实记**：收尾那份（`14:28:12`）与 §4.1c 用的是同一个文件名 `fingerprint-postpower-20261108T141936Z.json`，把 `14:20:35` 那份覆盖了，靠 `.4.1c-preserved.json` 另存才留下。因此 `4.1c-postpower-diff-20261108T141936Z.txt` 头部记的 `today` 路径现存内容是 `14:28:12` 那份，表 1 对比实际用的字节在 `.4.1c-preserved.json` 里（该 txt 头部记的 `captured_utc=2026-11-08T14:20:35+00:00` 与之对应）。同名冲突的处置与 `TODO.md` 已登记的那条一致，本场再次发生。收尾指纹的事实行见 `stdout/8-fingerprint-facts-20261108T141936Z.log`：`robot_model alohamini2pro`、`lerobot_version 0.6.1`、`observation_state_keys 18`、`captured_utc 2026-11-08T14:28:12+00:00`。

**收尾留痕**：`14:28:31Z` 执行 `kill -INT 2758`、`kill_int_exit=0`（`stdout/8-host-stop-20261108T141936Z.log`）；`14:28:40Z` 复核：`pgrep -af "alohamini_host|lerobot"` 只命中复核命令自身那一行（自匹配，无宿主进程），`ss -ltnp` 配 `grep -E ":5555|:5556"` 零命中（无监听）；`---` 之后是 Host 日志末三行原文 `WARNING:root:Host command watchdog expired; stopping motion` / `Keyboard interrupt received. Exiting...` / `Shutting down AlohaMini Host.`（`stdout/8.1-host-stopped-20261108T141936Z.log`）。

### 未取到 / 未验证

| 项目 | 状态 |
| --- | --- |
| V01 主表「步末残余 ≤2 tick」支 | **已按 `docs/ADR.md` D20 处置**：残余支改为按关节类别分级（肩类 `≤2°`、其余 `≤2 tick`），达成率分母统一 `abs(delta) × steps`；11-08 路 A 双侧按新口径全支过，V01 收口为「通过」。口径出处 `docs/ADR.md` D20。 |
| 左 `shoulder_pan` 位置差 `−11.4` tick 的归因算式 | **未验证**。证据目录只有 delta 列（`4.1c-postpower-diff-*.txt`），没有逐只归因表；`−11.4` 是现场报告转述，未落原文 |
| `Present_Voltage` 的 0.1 V/单位 | **源码侧已核**：地址 62、1 字节、无符号、全链无换算（vendor fork `ec65318f`：`src/lerobot/motors/feetech/tables.py:86` / `:140` 均为 `(62, 1)`，编码表 `:210-219` 无该项，`src/lerobot/motors/motors_bus.py:1014-1025` 只做地址取址 + 1 字节读 + 符号解码 + 位置归一化，`src/lerobot/robots/alohamini/` 内零命中）；**但「0.1 V/单位」这个比例源码不提供**，来自 Feetech 手册，故 `116–119` → `11.6–11.9 V` 只在手册口径下成立，换算后的电压值不得当标定或实测证据使用，读数只作相对比较 |
| 两项安全前置（手臂低位、断电开关在手边）在 B2 开跑前的复核 | **未复核**。只有 P1 阶段 `stdout/scope-20261108T125734Z.md` 的用户口述记录（非仪器证据）；`stdout/7.1-checklist-20261108T125734Z.md` 第 6、7 条同口径 |
| `rise_90_time_s` | 仍不全：左 `count=2`、右 `count=3`，其余步为空（与 11-06 的探针口径一致，未修） |
| 手册指纹同名冲突 | **已手工绕过**：同一 STAMP 下 §4.1c 指纹与收尾断电前指纹同名，§4.1c 那份另存 `capture/fingerprint-postpower-20261108T125734Z.4.1c-preserved.json`（与 `13:00:29Z` 那份逐字节相同）。手册口径本轮已补（见 `TODO.md` §4） |

## 2026-11-06（远程会话）· S4 现场窗口（第 3 次）

条件：AlohaMini 2 Pro（`robot_model=alohamini2pro`），18 个电机，Pi 上 lerobot `0.6.1`，远程 SSH 会话；**无人在机器人旁、无物理急停操作者**（scope 偏离，见 `stdout/scope-20261106T062506Z.md`），全程 0 条运动命令——§7.1b 走拒绝路径、§4.4 走 `dry_run=True`。STAMP `20261106T062506Z`；Host 起 `06:44:41Z` / 停 `06:45:45Z`。证据目录（仓库外）为 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/`，下表「证据文件」列都是这个目录下的相对路径。

### 结论一览

| 结论 | 关键数字 | 条件 | 证据文件 |
| --- | --- | --- | --- |
| 全 18 电机静态体检：18/18 在线、612/612 读取零失败 | 18 个电机 × 34 项寄存器 = 612 次读取，`'ok': False` 0 条；左组 11 只（左臂 7 + 底盘 3 + 升降 1）、右组 7 只，两份 `.exit` 均 `0` | 停 Host 后只读，走 `/dev/am_arm_follower_left` 与 `/dev/am_arm_follower_right` | `stdout/4.2-all18-left-20261106T062506Z.log`、`stdout/4.2-all18-right-20261106T062506Z.log`（各带 `.cmd` / `.exit` / `.py`） |
| ID8–11（底盘 3 轮 + 升降轴）首次实机确认在线 | `base_left_wheel` / `base_back_wheel` / `base_right_wheel` / `lift_axis` 的 `Model_Number` = `2825` / `2825` / `2825` / `2569`（2825 = sts3250、2569 = sts3095） | 同上；底盘三轮 `Operating_Mode=1`，升降轴 `Operating_Mode=0` | 同上 |
| 标定寄存器三项 54/54 与 2026-11-05 基线全等 | `Min_Position_Limit` / `Max_Position_Limit` / `Homing_Offset` × 18 电机 = 54 项，逐项与 `fingerprint-20261105T063600Z.json` 的 `range_min` / `range_max` / `homing_offset` 相同，差异 0 项；指纹 JSON 侧 8 字段 × 18 电机的全量 diff 也是 `CALIB_FIELD_DIFFS: 0` | 只读；`Drive_Mode` 不在 34 项寄存器清单里，寄存器侧只能比这三项（见「未取到 / 未验证」） | `stdout/4.1c-postpower-diff-20261106T064429Z.txt`、`stdout/4.2-all18-*.log` |
| 左夹爪 Min/Max 仍是 `2021` / `2072`（51 tick 未变） | 寄存器读数与原基线逐项相同，`docs/ADR.md` D14「本轮不重标定」的依据延续 | 同上 | `stdout/4.2-all18-left-20261106T062506Z.log`、`capture/fingerprint-postpower-20261106T064429Z.json` |
| 电压读数：18 电机原始值 min `106` / mean `107.83` / max `109`（≈10.6–10.9 V） | 每单位 0.1 V 是厂商手册推导、源码不提供该系数（源码侧已核：地址 62、1 字节、无符号、全链无换算），故 `10.6 V` 判据线与本节电压读数都只在手册口径下成立，读数只作相对比较，换算后的电压值不得当标定或实测证据使用；最小 `106` 恰卡在判据线 10.6 V 上（判据是「任一 < 10.6 V 才需充电后再取基准」），故不判需充电 | 只读寄存器 `Present_Voltage`；本轮未单独测母线电压 | `stdout/4.2-all18-left-20261106T062506Z.log`、`stdout/4.2-all18-right-20261106T062506Z.log` |
| 温度读数：min `20` / max `29` ℃ | `arm_right_wrist_yaw` 单点 `29` ℃，其余 17 只在 20–23 ℃；单点偏高的成因未验证 | 同上；读数时机器人静止、Host 已停 | 同上 |
| 上电自复位升降轴两轮都到位 | 会话 1 峰值 `Present_Current=2658.5 mA`、会话 2 峰值 `2015.0 mA`，两轮日志均有 `Lift axis homed to 0mm.` | vendor Host 启动时自动执行，不是本库命令 | `stdout/6.1a-hostlog-20261106T064429Z.log`（会话 2）；会话 1 原文是 Pi 上 `/tmp/site-host-20261106T063251Z.log`，未取回证据目录，数字见 `stdout/scope-20261106T062506Z.md` |
| postpower 指纹齐全 | `robot_metadata.schema_version=2`、18 个电机、`captured_utc=2026-11-06T06:45:05+00:00`、`lerobot_version="0.6.1"`、`observation_state` 18 键、`sample_count=5` | capture 在 Pi 侧跑（指纹 `host` 字段是 `127.0.0.1`），`scp` 取回 | `capture/fingerprint-postpower-20261106T064429Z.json`、`stdout/4.1c-capture-check-20261106T064429Z.log`、`stdout/4.1c-capture-20261106T064429Z.*` |
| 位置比对：8 只臂电机超 3 tick，**不可归因** | 超阈 8 只：`arm_right_wrist_roll` −303、`arm_left_shoulder_pan` +200、`arm_left_wrist_roll` +91、`arm_right_wrist_yaw` −83、`arm_right_shoulder_pan` −39、`arm_left_wrist_yaw` +29、`arm_right_elbow_flex` −9、`arm_left_wrist_flex` +6；其余 6 只 ≤ 3 tick（其中 `arm_right_wrist_flex` 恰为 +3）。`lift_axis.height_mm` 由 −0.095947265625 变 `0.0`（Host 自复位的结果） | 基线 `fingerprint-20261105T063600Z.json` 采于 2026-11-05 06:43:10Z，**早于那轮试动**，差值里混着试动位移，不能当断电滑移证据 | `stdout/4.1c-postpower-diff-20261106T064429Z.txt` |
| **基准换账**：旧基线作废，`fingerprint-postpower-20261106T064429Z.json` 升为新基线 | 新基线 `captured_utc=2026-11-06T06:45:05+00:00`；旧基线 `fingerprint-20261105T063600Z.json` 跨过了 11-05 那轮试动，不能再作「断电前位置」基准。**再换一次**：2026-11-06 07:00Z 后换电，现行基线改为 `fingerprint-postpower-20261106T070942Z.json`（`captured_utc=2026-11-06T07:10:31+00:00`），`064429Z` 跨过换电、同样再作废为**位置**基准 | — | `capture/fingerprint-postpower-20261106T064429Z.json`、`stdout/4.1c-postpower-diff-20261106T064429Z.txt`、`capture/fingerprint-postpower-20261106T070942Z.json`、`stdout/scope-append-20261106T070942Z.md` |
| §7.1b 反向拒绝：exit `6` 但落在 **Gate B**，不是 Gate C | stderr 原文四行：厂商客户端 `ERROR:root:Error processing observation data, serving last observation: Host robot_model does not match the client configuration`，随后本库 `Gate B refused: 'arm_left_shoulder_pan.pos' is not part of the connected state schema.`（`available: forward, wrist_right`）+ `Check --joint and --robot-model against the host side; nothing was commanded.`；零命令、无 CSV（`ls` 报 `No such file or directory`） | `--robot-model alohamini2` 故意写错，宿主报 `alohamini2pro`；厂商客户端先把观测 schema 降级成只剩相机键，本库的键存在性检查 Gate B 先拦，Gate C 文案在真机链路上不可达 | `stdout/7.1b-gatec-reverse-20261106T064429Z.{cmd,log,exit}`、`stdout/7.1b-no-csv-20261106T064429Z.*` |
| 本库 dry-run 真机首跑通过 | `exit 0`；观测 `21` 键；`metadata schema_version=2`、motor 条目 `18`；`send_action` 调用 `0` 次；升降轴 `lift.move_by(delta_mm=1.0)` 在 priming 阶段被拒，报文 `MotionNotEnabled: motion requires a kinematic adapter; connect() was called without one` | `examples/robot_dry_run.py` 注入厂商 `AlohaMiniClient`，`dry_run=True`、无 adapter；vendor 快照 `/tmp/lerobot-vendor-6aee1f81/src` | `stdout/4.4-robot-dry-run-20261106T064429Z.{cmd,log,exit}` |
| 串口符号链接与 `configs/site.yaml` 一致 | `/dev/am_arm_follower_left -> ttyACM0`、`/dev/am_arm_follower_right -> ttyACM1`，正对 `site.yaml` 的 `left_port` / `right_port` | 只读 `ls -l` | `stdout/3.7b2-serial-links-20261106T062506Z.log`、`stdout/3.7b-serial-pi-20261106T062506Z.log` |
| Pi 磁盘占用 84%，与上一轮持平 | `/dev/mmcblk0p2` 29G、已用 23G、可用 4.6G、`Use%` 84% | 只读 `df -h /` | `stdout/3.p0-df-20261106T062506Z.log` |

### 到场窗口（有人在场）· 肩部定性

条件：同一台 AlohaMini 2 Pro（`robot_model=alohamini2pro`，18 个电机，左总线 11 只 / 右总线 7 只），**用户本人在机器人旁**（手臂低位支撑，逐轮口头授权），**可下运动命令**、**左总线可直连只读**（停 Host 后打开 `/dev/am_arm_follower_left`）、**Host 可启停**（多数轮次 `kill -INT`，M1 两轮为 `kill -9`）。运动轮次全部为单关节 `--mode tracking` 步进，参数 `--control-hz 50.0`、`--settle-tolerance-deg 0.05`。授权记录逐轮在 `stdout/scope-20261106T062506Z.md` 与 `stdout/scope-append-*.md` 里。

窗口以证据里的时间戳为准：换电后第一次 Host 启动 `07:10:20Z`（`stdout/6.1a-host-pid-20261106T070942Z.log` 的首次 `date -u`），第一条运动命令的客户端 `07:19:38Z` 启动，最后一次运动命令 `09:06:35Z` 起跑（STAMP），最后一次 Host 停机 `09:08:01Z`（`stdout/4.1-host-int-20261106T090635Z.log`）。即 **07:10Z–09:08Z**，同一 UTC 日期内。

本小节证据全部在 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/` 这一个目录下，「证据文件」列给该目录内的相对路径。

| 结论 | 关键数字 | 条件 | 证据文件 |
| --- | --- | --- | --- |
| 命令链路最终清白：sidecar（贴着主 CSV 写的 safety 快照，逐帧记 `requested_target` 与 `accepted_target`）运动帧 `requested == accepted` 全 100% | 左肩 pan `902/902`（`m1r3-pan` 轮）、左肩反向 `800/800`（`m2-pan` 轮）、腕 `909/909`（`m1r3-wrist`）、右肩 `804/804`（`final-rightpan`）、Punch 试修后左肩 `906/906`（`final-punchtest`）；严格相等与容差 1e-9 两个口径同值，最大 \|diff\| `0.0` | 五轮单关节步进，客户端正常跑完或按授权停 Host | `stdout/10.5-m1r3-analysis-pan-20261106T074620Z.txt`（第 C 节）、`stdout/10.5-m1r3-analysis-wrist-20261106T074049Z.txt`（第 C 节）、`stdout/10.5-m2r1-verdict-20261106T075623Z.md`（1.1 表）、`stdout/10.5-rightpan-judge-20261106T084419Z.txt`（第 1 节）、`stdout/10.5-punchtest-judge-20261106T085403Z.txt`（第 1 节）、`stdout/scope-append-final-20261107-20261106T085908Z.md` |
| 双肩 pan 达成率（10 步 × 1°、settle 2 s 窗、容差 0.05°） | 左肩正向 **15.82%**（18.0 / 113.7527 tick；第二口径 17.0 tick = 14.95%）、左肩反向 **36.92%**（42.0 tick；第二口径 38.0 tick = 33.41%）、右肩 **11.43%**（13.0 tick；第二口径 11.0 tick = 9.67%）；三轮 10/10 步 `settled=False`、`note` 全 `settle budget exhausted` | 左臂 / 右臂 `shoulder_pan` 单关节，`--delta-deg ±1.0 --steps 10`；正向、反向、右肩各一轮 | `stdout/10.5-m2r1-verdict-20261106T075623Z.md`（0 节与 1.3 表）、`stdout/10.5-rightpan-judge-20261106T084419Z.txt`（第 3 节）、`stdout/10.5-m1r3-analysis-pan-20261106T074620Z.txt` |
| 腕部同参数对照：自身命令量全走完，但命令量本身少 1.375 tick | 腕走 `90.0/90.0` tick（`measured_tick / requested_tick = 1.000000`）；**每步命令量只有 10.0 tick（0.879120879 度），而 `--delta-deg` 给的是 1.0**——实测每步下发量就等于 0.879°，与 `--delta-deg 1.0` 不符，**原记「换算来源未定位」已更正**：换算无 bug，差额是默认 `--anchor present` 这一口径的 artifact——每步目标是「上一条实测反馈 + 1.0°」，腕每步实测落后目标 0.120879°（1.375 tick）且不累积，故相邻两条目标间距 = `1.0 − 0.120879 = 0.879120879°`（折 10.0 tick）；同一 `--delta-deg` 下肩每步仍是 11.375 tick。结论与三处独立读数（`cmd_units` 差 / `cmd_deg` 差 / `measured_deg` 差）见 `TODO.md`「腕与肩的 deg→units 换算不自洽」那条的最终结论；**不作「腕 100% 证明链路正常」的证据** | 腕 `arm_left_wrist_roll`，`--delta-deg 1.0 --steps 10`，`deg_per_unit=1.8000000000000003` | `stdout/10.5-m1r3-analysis-wrist-20261106T074049Z.txt`（第 F 节差分表、第 H 节逐行表） |
| 单步 15 s 长窗：腕走完 87.912%，肩只走 17.582% | 腕 `10.0/11.375` tick = **87.912%**，残余 1.375 tick；肩 `2.0/11.375` tick = **17.582%**，残余 **9.375 tick**；两轴 `settled=False`、`note=settle budget exhausted` | `--steps 1 --delta-deg 1.0 --settle-timeout-s 15`，同参数两轴各一次 | `stdout/slow-vs-stall-judge-20261106T090635Z.txt`（第 3、7 节） |
| 失速判据成立：1° 的 lead 破不了肩轴静摩擦 | 11.375 tick（1°）的目标在 15 s 长窗里只走 2.0 tick，残余 9.375 tick 未收敛 | 同上 | `stdout/slow-vs-stall-judge-20261106T090635Z.txt`（第 7 节） |
| 推压测试推翻「机械卡死」 | 300 样本、150 s、2 Hz：`Torque_Enable` 300/300 恒 `1`、`Status` 恒 `0`、`Moving` 恒 `0`；推压段（20–120 s，200 样本）`Present_Position` 在 `2113–2148` 往复，span 35 tick、单步最大 23 tick，负载峰 `+72 / −68`；松手后 60 样本全停在 `2124`、span 0 | 严格只读，Host 状态未变，`/dev/am_arm_follower_left` 直连 | `stdout/push-test-20261106T081803Z-verdict.md`（表一、表二）、`stdout/push-test-20261106T081803Z.log`（第 313–314 行 `samples=300 Torque_Enable values seen=[1]`、`Present_Position n=300 … span=35`） |
| 用户断电手感正常 | **用户现场口述，未取仪器证据**（无日志、无传感器读数） | 用户在场，断电后手动掰臂 | 无（口述） |
| 配置层排除：`configure()` 之后两轴目标寄存器逐项同值 | 两轴 × 14 项目标寄存器 `14/14` 同值、差异 `0` 项；两轴 × 15 项上下文寄存器 `15 × 2 = 30/30` 有值，其中 3 项是动态读数（`Present_Position` 2100 / 1765、`Present_Voltage` 121 / 118、`Present_Temperature` 26 / 25，不计入配置差异），其余 12 项同值；`Phase` 两轴同为 `12`（`0x0C`）；读数 `FAIL` 0、`"ok": false` 0 条；两遍采样 28/28 格稳定 | Host `08:35:03Z` 起、`08:35:51Z` `kill -INT` 停，读数窗口 `08:36:10Z–08:36:13Z`，严格只读 | `stdout/rt-regs-verdict-20261106T083449Z.md`（0 节、第 1、2 节）、`stdout/rt-regs-20261106T083449Z.log` |
| Punch=8 试修无改善（**本项目首次寄存器写入**） | 写 `Minimum_Startup_Force=8` 后左肩 **14.95%**（17.0 tick）对基线 **15.82%**（18.0 tick），差 **−0.88 个百分点 / −1.0 tick**；已写回 `0`，读回 `0`、新连接独立复核 `0`、`match:true`、两次写入均 exit `0` | Host 已停、串口无引用、`lock=0`、`torque_enable=0`；授权限「一次写 8 + 一次写回 0」 | `stdout/scope-append-final-20261107-20261106T085908Z.md`、`stdout/07-write-8-20261106T085234Z.log`、`stdout/07-write-0-20261106T085740Z.log`、`stdout/08-read-recheck-20261106T085234Z.log`、`stdout/08-read-recheck-20261106T085740Z.log` |
| 右肩 11.43% 落「类别问题（非个体退化）」 | 右肩单轮 11.43%（13.0 tick），与左肩同参数 15.82%（18.0 tick）同量级；两轴 10/10 步均 `settled=False` | 右臂 `shoulder_pan`，与左肩同一参数集 | `stdout/10.5-rightpan-judge-20261106T084419Z.txt`（第 3 节）、`stdout/10.5-m2r1-verdict-20261106T075623Z.md` |
| **最终定性：命令形态问题** | tracking 把每步 lead 钳在 1° = 11.375 tick（`MAX_STEP_DELTA_DEG = 1.0`，每步以当前反馈为基线重新加 1°，误差不累积）；这个量级腕部够用，肩轴静摩擦破不了。机理 = 误差幅度对比静摩擦 / 负载阈值。判定依据：同一链路、同一 sidecar 上腕走 87.912% 而肩走 17.582%（上两行数字） | 代码侧：`benchmarks/alohamini_io/measure_latency.py:3149` 每步重读 `baseline_units`、`:3167` 再 `+delta_deg`、`:504` 的 `MAX_STEP_DELTA_DEG = 1.0` | `benchmarks/alohamini_io/measure_latency.py`（约 `:3149-3168`、`:504`）；数字见本表「单步 15 s 长窗」与「失速判据」两行 |
| 修复路径：软件侧解耦 lead 锚点 | 给 `measure_latency` 加 lead 锚点旋钮（`--anchor {present,command}` 或 `--lead-deg`），让误差可累积；修完重跑肩部 10 步 × 1° 验证 | 结论 / 计划；后于 2026-11-08 落地并现场验证（见本文 2026-11-08 节） | `docs/field-results.md` 2026-11-08 节 |
| `Torque_Enable` 的**现场行为**已定（写入者仍为外部来源） | 现场数据两条：① `kill -9` 之后左总线 11 只电机全 `Torque_Enable=1`（`stdout/10.5-m1-regs-final-20261106T073052Z.log` 第 17、52、87、122、157、192、227、262、297、332、367 行）；② `kill -INT` 之后两轴 `Torque_Enable=0`（`stdout/rt-regs-verdict-20261106T083449Z.md` 第 1.2 节；该轮读数会话 `disconnect(disable_torque=False)`，未碰 `Torque_Enable`）。`SIGINT` 一侧的机理：Host 走 `except KeyboardInterrupt` → `finally` → `robot.disconnect()`，`disconnect()` 调 `left_bus.disconnect(self.config.disable_torque_on_disconnect)`，该字段默认 `True`，即释放扭矩；`kill -9` / `SIGTERM` 不给收尾机会、扭矩保持带电，`SIGTERM` 一支本轮无单独读数（未验证）。源码行号按提交取，逐条两支：`alohamini_host.py:548-556`（`ec65318f`；`6aee1f81` 为 `:612-620`）、`alohamini.py:1251-1253`（`ec65318f`；`6aee1f81` 为 `:1284-1286`）、`config_alohamini.py:56`（两支该行相同，偏移 0）。残留：写 `Goal_Position` 时固件自置 `Torque_Enable=1` 只有**外部来源**（厂商手册与社区帖两处，原文不在本证据目录），写入者仍未定 | Host 已停后只读；写路径未做本体实测 | 现场：`stdout/10.5-m1-regs-final-20261106T073052Z.log`、`stdout/rt-regs-verdict-20261106T083449Z.md`（第 1.2 节）、`stdout/rt-regs-host-verify-20261106T083449Z.log`（末两行 `Keyboard interrupt received. Exiting...` / `Shutting down AlohaMini Host.`）；写入者一侧：**外部来源，非本轮实测** |
| 看门狗会在 1 s 内把 `Goal_Position` 改写成保持位（机制） | Host 在最后一条合法命令之后超过 `watchdog_timeout_ms` 仍没收到新命令时，把 `last_observation` 的 `*.pos` 当保持位重新下发（`{**hold_action, "x.vel": 0.0, …}`），并把 `target_source` 置 `watchdog`；此时读到的 `Goal − Present` ≈ 1 tick，远小于 11 tick，分支③ 会假命中。现场告警原文两行 `WARNING:root:Host command watchdog expired; stopping motion` 在 `stdout/site-host-20261106T074049Z.log` 第 16–17 行，**属 07:40:49Z 起的那个 Host 会话**（装的是 07:40 腕部轮与 07:46 pan 轮），不是 07:26 那轮的证据 | 默认超时 `watchdog_timeout_ms: int = 1000`（`config_alohamini.py:110`）；判定式与保持位构造 `alohamini_host.py:348-362`（`ec65318f` 只读核对；`6aee1f81` 为 `:411-425`）、`hold_action` 取 `last_observation` 的 `.pos`、`target_source = "watchdog"` 在 `:357`（`6aee1f81` 为 `:420`） | `stdout/site-host-20261106T074049Z.log`（第 16–17 行） |
| sidecar 首帧的 `target_source=watchdog` 是上一轮遗留，不是本轮客户端先死的证据 | 两轮 sidecar 各有且仅有 18 行 `watchdog`（18 只电机 × 1 帧），全部 `step_index=0`、`sample_sequence=2`，而各轮第一条命令帧在 `sample_sequence=3`：`tracking/m1r3-pan-20261106T074620Z-safety.csv`、`tracking/slowvsstall-pan-20261106T090635Z-safety.csv`。其余 5 份 sidecar（`m1r3-wrist`、`m2-pan`、`slowvsstall-wrist`、`final-rightpan`、`final-punchtest`）`watchdog` 帧数全为 0，首帧都是 `none` | `target_source` 是 Host 侧的持久状态：只在收到合法命令时置 `command`（`alohamini_host.py:330`）、看门狗触发时置 `watchdog`（`:357`）、监督纠偏时置 `protection`（`:373`），新客户端连上本身不改它；上一轮客户端结束（正常退出或被 kill）后 1 s 看门狗即触发，于是「上一轮刚结束、Host 仍在跑」的会话里，新客户端第一条数据帧就是 `watchdog`，早于它自己的第一条命令（本地 fork `ec65318f` 只读核对） | `tracking/m1r3-pan-20261106T074620Z-safety.csv`、`tracking/slowvsstall-pan-20261106T090635Z-safety.csv`、`tracking/m1r3-wrist-20261106T074049Z-safety.csv`、`tracking/m2-pan-20261106T075413Z-safety.csv`、`tracking/slowvsstall-wrist-20261106T090635Z-safety.csv`、`tracking/final-rightpan-20261106T084419Z-safety.csv`、`tracking/final-punchtest-20261106T085403Z-safety.csv` |
| `achieved_command_hz=28.4` 未复现（归因未取证） | 2026-11-05 `pan` 那一轮实测 28.4 Hz（同轮另两次 43.7 / 42.8）；今日同参数七轮实测 `39.0`（`7.1-tracking`）、`39.0`（`10.5-m2`）、`44.0`（`10.5-m1r3-pan`）、`44.3`（`10.5-m1r3-wrist`）、`44.3`（`6.1e-punchtest`）、`45.1`（`2.1-wrist`）、`45.2`（`3.1-pan`），28.4 Hz 未复现，归因当时会话负载，未单独立项取证 | 同参数 `--control-hz 50.0`，请求 50.0 Hz | `evidence/s4-20261104/stdout/7.1-shoulder-pan-20261105T063600Z.log:10`；今日七条见 `stdout/7.1-tracking-20261106T084419Z.log`、`stdout/10.5-m2-20261106T075413Z.log`、`stdout/10.5-m1r3-pan-20261106T074049Z.log`、`stdout/10.5-m1r3-wrist-20261106T074049Z.log`、`stdout/6.1e-punchtest-leftpan-20261106T085403Z.log`、`stdout/2.1-wrist-20261106T090635Z.log`、`stdout/3.1-pan-20261106T090635Z.log`（各第 10 行，按上述顺序依次为 `39.0` / `39.0` / `44.0` / `44.3` / `44.3` / `45.1` / `45.2`，已逐条核对原文） |
| §4.5 Pi 未提交文件调查：判「记录 reflog」 | `stash list` 空（0 字节）、`status --porcelain` 空、`reflog --date=iso` 两行：`2026-10-25 09:17:30` `checkout: moving from master to p1a-merged`、`09:22:01` fast-forward merge 到 `6aee1f81`。据此判第二支（记录 reflog）：具体是哪一步删除不可定位。「6 个未提交改动」这个数字出自 2026-10-25 节对 legacy 备份 `ec65318f` 的描述，本轮未复核具体文件名 | 只读 SSH，一次会话 | `stdout/4.5a-stash-20261106T064822Z.log`、`stdout/4.5b-reflog-20261106T064822Z.log`、`stdout/4.5c-status-20261106T064822Z.log`、`stdout/4.5-ssh-check-20261106T064822Z.log` |

本小节未取到 / 未验证：

| 项目 | 状态 |
| --- | --- |
| 07:26 那轮 M1 `Goal − Present = 1 tick` 的成因 | **未确定**。读数 `Goal=2156` / `Present=2155`（ID1），看门狗改写是候选之一（机制见上表「看门狗会在 1 s 内把 `Goal_Position` 改写成保持位」）。该轮没有 Host 日志原件、没有 CSV、没有 sidecar（`tracking/` 下无任何 `tracking-10.5-m1-20261106T072606Z*`），该轮判读自己写明「快照本身无法区分『从未写入』与『已写入且已到位』」（`stdout/10.5-m1-verdict-20261106T072606Z.md`「有效性质疑」段）；该轮操作顺序是客户端先 `kill -9`、Host 7 s 后才 `kill -9`，不符合 M1「Host 死在客户端之前」的主判据（`docs/field-ops/50-experiments.md` §10.5 M1） |
| 断电后手臂失力手感 | **用户现场口述，未取仪器证据**。无日志、无传感器读数，不进结论 |
| `Minimum_Startup_Force` 的 `configure()` 持久性 | 未取证。下次 Host 启动后该寄存器是否被 vendor `configure()` 覆盖回 0，未测（`stdout/scope-append-final-20261107-20261106T085908Z.md` 的「未验证 / 限制」） |
| 腕部 1.375 tick 稳态残余的成因 | 未查。15 s 长窗仍未收敛（`stdout/slow-vs-stall-judge-20261106T090635Z.txt`），与肩的 9.375 tick 不同源，成因未定位 |
| `rise_90_time_s` | 仍全空。本轮所有 tracking CSV 该列 10/10 行为空；`slow-vs-stall` 判读给出的机制是「残余 > 0.1 × 命令量，没有帧越过 90% 线」（第 5.4 节），未修探针 |
| `Torque_Enable=1` 由固件在写 `Goal_Position` 时自置 | 目前只有**外部来源**（厂商手册 + 社区帖两处），未在本体上单独验证。现场数据只能证明「`kill -9` 后仍为 1」「SIGINT 后为 0」 |
| vendor 遥操作「不做 lead 钳制、误差可累积」 | 本轮未跑 teleop oracle，无记录。用户口述 vendor 遥操作同链路能带动肩部，未取仪器证据 |

### 未取到 / 未验证

| 项目 | 状态 |
| --- | --- |
| `Drive_Mode` 的寄存器侧核对 | 未取到。`Drive_Mode` 不在 §4.2 脚本读的 34 项寄存器清单里，寄存器层「标定四字段」只能比三项（`Min_Position_Limit` / `Max_Position_Limit` / `Homing_Offset`）；`drive_mode` 只由指纹 JSON 侧比对（8 字段 diff 为 0） |
| `base_back_wheel` 一次 `Moving=1` 瞬时扰动 | 成因未验证。左组日志里 `base_back_wheel` 读到 `Moving=1`、`Present_Velocity=-50`，同一轮其余电机 `Moving=0`；后段 `pass2` 复读全为 `Moving=0`。未重测、未定位 |
| `arm_right_wrist_yaw` 温度 `29 ℃`（其余 17 只 20–23 ℃） | 成因未验证。单点读数，未复测，未查该轴负载与驱动配置 |
| 第一次起 Host 的本地 ssh 挂 600 s | 根因未定位。会话 1（`06:32:51Z` 起）本地 ssh 调用 600 s 未返回，而远端 Host 已就绪；上层中止后 `06:43:31Z` 执行 `kill 2340`（`kill_exit=0`），复查无 LISTEN、无残留。会话 2 改用「本地 ssh 后台 + 轮询端口 + 杀本地 ssh」，`06:44:41Z` 一次起通（python PID `2517`） |
| Gate C 的真机专属证据 | 不可达。真机链路上厂商客户端先降级观测 schema，本库 Gate B 先拒；Gate C 文案只在离线测试里成立（见 `docs/field-ops/40-gates-params.md` §7.1b）。V10 的 Gate C 专属实机子项是否接受 Gate B 替代，待裁决 |
| 会话 1 的 Host 日志原件 | 未取回证据目录。`/tmp/site-host-20261106T063251Z.log` 的 `2658.5 mA` 与 `Lift axis homed to 0mm.` 两行只经 `stdout/scope-20261106T062506Z.md` 转述 |

## 2026-11-05（远程会话）· S4 现场窗口（第 2 次）

条件：AlohaMini 2 Pro（`robot_model=alohamini2pro`），18 个电机，Pi 运行时 HEAD `6aee1f81`，Pi 上 lerobot `0.6.1`，远程 SSH 会话（无人在机器人现场），本库 HEAD `b61a61f`；证据目录（仓库外）为 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261104/`，下表「证据文件」列都是这个目录下的相对路径。

### 结论一览

| 结论 | 关键数字 | 条件 | 证据文件 |
| --- | --- | --- | --- |
| 连通与基线指纹齐全，Pi 上 lerobot 版本首次实采 | Pi HEAD `6aee1f81`；lerobot `0.6.1`；指纹 `robot_metadata.schema_version=2`、18 个电机、motors 8 字段与 2026-10-25 零差异（已做两版 JSON `robot_metadata.motors` 子树全量 diff：18 只电机 × 8 字段 = 144 条叶子路径，0 差异；报告 `stdout/4.1-motors-diff-20261105.txt`）；指纹新键 `lerobot_version="0.6.1"`；手册 §4.3 三键实采 `observation_request_window=3`、`watchdog_timeout_ms=1000`、`max_loop_freq_hz=50`，与 `configs/site.yaml` 一致 | SSH 通；`--lerobot-version` 机制首次实战；capture 在 Pi 侧跑，指纹 `host` 字段是 `127.0.0.1`；§4.3 三项取自源码默认值且命令行无覆盖 | `capture/fingerprint-20261105T063600Z.json`、`stdout/4.1b-lerobot-version-20261105T063600Z.log`、`stdout/4.1b-probe-20261105T063600Z.log`、`stdout/4.1-capture-pi-1-20261105T063600Z.cmd`、`configs/site-verified-20261105T063600Z.yaml` |
| 左夹爪 ID7 寄存器只读确认命中判据，固件限位已按标定值生效 | 左臂 ID7 `Min_Position_Limit=2021` / `Max_Position_Limit=2072`；右臂 ID7 对照 `2010` / `3479`；两只 `Torque_Enable=0`、`Lock=1`；两份日志 `EXIT_CODE: 0` | 停 Host 后只读。51 tick 的成因在标定值，`docs/ADR.md` D14「本轮不重标定」维持。本轮 `Torque_Enable=0` 与 2026-10-25 读到的 ID1/2/3/6 的 `Torque_Enable=1` **不可比**：本轮测的是夹爪 ID7，上次测的是臂关节，不是同一批电机的状态变化 | `stdout/4.2-gripper-regs-20261105T063600Z.log`、`stdout/4.2-gripper-regs-right-20261105T063600Z.log` |
| 两份 YAML 经 `load_settings(check_devices=False)` 读通 | `stdout/4.3e-load-check-20261105T063600Z.exit` 为 `0`；`.log` 打出 `AlohaMiniSettings`，其中 `observation_request_window=3`、`watchdog_timeout_ms=1000`、`max_loop_freq_hz=50` | 该步 `.cmd` 用 `configs/template.yaml` + `configs/site.yaml` 调 `load_settings(..., check_devices=False)` | `configs/template.yaml`、`configs/site.yaml`、`stdout/4.3e-load-check-20261105T063600Z.{cmd,log,exit}` |
| 零运动测量三次各 500 行，跨机 RTT 达成，真实网段与回环同量级，PC 侧 50 Hz 命令节奏有实测支持 | 部署形态（请求 30 Hz、窗口 3）`achieved_rate_hz=30.1`；回环 RTT 形态（请求 1000 Hz、窗口 1）`achieved_rate_hz=49.8`、`net_rtt_estimate_ms` p50 `6.188`；跨机（Mac → Pi，`host=10.34.170.98`）`achieved_rate_hz=48.0`、`net_rtt_estimate_ms` p50 `6.010`、p95 `9.105`、`host_timing_missing=0/500` | 跨机那条首次 exit `9`（`it publishes no latest_request_timing`：隔离环境装的是 vendor wheel，不含该计时面，首跑不写 CSV，产物是 `stdout/5.3-rtt-net-cross-20261105T063600Z.log`，`.exit` 为 `9`）；重跑把 `PYTHONPATH` 指向本地 fork `6aee1f81` 快照 `/tmp/lerobot-vendor-6aee1f81/src` 后 exit 0，CSV 实际名带 `-2` | `rtt/rtt-deploy-20261105T063600Z.csv`、`rtt/rtt-net-20261105T063600Z.csv`、`rtt/rtt-net-cross-20261105T063600Z-2.csv`、`stdout/5.1-rtt-deploy-*`、`stdout/5.2-rtt-net-*`、`stdout/5.3-rtt-net-cross-*`、`stdout/5.3-rtt-net-cross-2-*`、`stdout/5.3b-clone-*`、`stdout/5.3b-verify-src-*` |
| 静止电流基线 18/18 电机全覆盖，读数只有两档 | 18/18 电机 × 100 帧（5 Hz、20 s、`frame_errors=0`）；取值只有 `0` 与 `+6.50 mA`（`+6.5` = 1 个 raw 刻度 × 6.5）；全程恒 0 的 7 只：`arm_left_gripper`、`arm_left_wrist_yaw`、`arm_right_gripper`、`arm_right_wrist_flex`、`arm_right_wrist_roll`、`base_left_wheel`、`base_right_wheel`（`flat0_by_abs_le_0.5mA_count=7`） | 取数路径是 `latest_safety_status["currents_ma"]`，不是 Host 的 `[Currents]` 日志行 | `stdout/6.1c-obs-idle-20261105T063600Z.log`（末尾有 `# per_motor_table` 与 `# summary`）、`.cmd`、`.py` |
| 运动窗口电流量级不足判读门槛，臂总线电流不能判负载 | 腕部 run 的运动窗口 1638 个（帧×电机）样本，最大 `6.5 mA`，`≥20 mA` 样本 0 个；`pan` / `lift` 两次 run 的窗口内最大 `19.5 mA`，取值只有 `0` / `6.5` / `13.0` / `19.5` 四档 | 判读门槛 `20 mA`；`verdict:` 行为「臂总线电流不能判负载」，后续所有依赖电流的判断一律降级为「无电流证据」 | `stdout/6.2b-analysis-20261105T063600Z.txt`、`stdout/7.2-analysis-20261105T063600Z.txt` |
| 腕部 sidecar 真机首跑成功，逐帧 18 电机齐全、`hold_active` 全 False | `exit 0`；`frame_count=179`、`rows_per_frame_distribution={18: 179}`（179 帧每帧 18 行，`total_rows=3222`）；`motor_empty_rows=0`、`current_ma_empty_rows=0`；`current_ma` 取值只有 `0.0` 与 `6.5` 两档；`hold_active_values={'False': 3222}`；`target_source_per_frame={'none': 1, 'command': 178}`，帧内不一致 `0` | 腕部那一轮（`7.1-wrist-try2`）的 sidecar 旁路采集，与主 CSV 同窗口（`tracking/tracking-wrist-20261105T063600Z-safety.csv`，275522 字节） | `stdout/7.1-wrist-sidecar-try2-20261105T063600Z.log`（数字源）、`.cmd`、`.exit` |
| 达成率三轴只有腕部接近指令，肩部两轴与 2026-10-25 相比一致变差 | 腕部 87.91%（净 `1.7582°` ÷ 指令 `2.0000°`，`error_deg=0.1209`，与 2026-10-25 逐字相同）；`arm_left_shoulder_pan` **4.40%**（净 `0.0879°`）；`arm_left_shoulder_lift` **0.00%**（净 `0°`）；三轴 `settled=False` | 左臂单关节，三次 run 各 2 步、每步 1°；本轮容差 `0.05°` 严于上次 `0.15°`，`note` 全为 `settle budget exhausted`；三者与 2026-10-25 的 87.9% / 8.8% / 13.2% 相比，肩部两轴变差。限定：两轮差值只有 0.5–1.5 tick（亚 LSB 到 1.5 个计数，1 tick = 0.0879°），腕部达成率与残余两轮逐字相同，且本窗口容差 `0.05°` 小于 1 LSB（0.0879°），`settled=False` 是这一算术后果——变差是实测事实，但不能当硬件劣化证据 | `tracking/tracking-wrist-20261105T063600Z.csv`、`tracking/tracking-pan-20261105T063600Z.csv`、`tracking/tracking-lift-20261105T063600Z.csv`、`stdout/7.1-wrist-try2-*`、`stdout/7.1-shoulder-pan-*`、`stdout/7.1-shoulder-lift-*`、`stdout/6.2b-analysis-*`、`stdout/7.2-analysis-*` |
| 停滞不在 PC→Host 值传递段；Host→总线写入段未观测；候选根因含关节扭矩未使能（`alohamini.py:545` `enable_torque()` 被注释、`:520` 先 `disable_torque()`，**本段行号按提交 `ec65318f`**）、目标编码取整、伺服保护锁存、力矩不足、机械，未定性（分析脚本机械判为 ②，判读链见下） | 两轴（`pan`、`lift`）步内 `requested_target == accepted_target` 零例外（`requested!=accepted rows=0`）；`accepted_target` 变化点与主 CSV 的 `cmd_units` 逐位字符相同（`req==cmd True`、`acc==cmd True`）；运动帧 `target_source` 全为 `command`。`requested_targets` / `accepted_targets` 在本版 vendor 上分别是 `_arm_goal_positions` 与 `_arm_sent_positions`（`alohamini.py:987-988`，**本段行号按提交 `ec65318f`**），后者在 `sync_write` 调用之后无回读即 `update`（`:919`，同上基准），故「零例外」只说明 `_limit_*_by_current` 未钳制值，不构成总线写入证据 | `hold_active` 在 Pi `6aee1f81` 上结构上恒 False（**本段行号按 Pi 运行时 `6aee1f81`**）：`grep _joint_hold_goal[` 共命中 **7** 行（`git grep -n _joint_hold_goal 6aee1f81 -- src/lerobot/robots/alohamini/alohamini.py` 计数），全部是定义 / 导出 / 读用法，无赋值写入点——初始化 `:298`、导出 `:335` / `:1018`、get 与 in 判断 `:1109` / `:1121`、下标读 `:1161`、pop `:1164`，故不作判据 | `stdout/7.2-analysis-20261105T063600Z.txt`、`stdout/7.2-joint-hold-write-*`、`stdout/7.2-joint-hold-writes-20261105T063600Z.log`、`stdout/7.2b-host-tracking-*`、`stdout/host-log-20261105T063600Z.log`、`tracking/tracking-pan-20261105T063600Z-safety.csv`、`tracking/tracking-lift-20261105T063600Z-safety.csv` |
| 日志里没有 `[Currents]` 与 `[HOST TRACKING]` 的原因是厂商日志级别，不是代码没走到 | 整份 Host 日志 `[Currents]` 行 `lines 0` / `elements 0`；`[HOST TRACKING]` 计数 `0`；厂商 client 里 `logging.basicConfig(...)` 的 `level=logging.INFO` 那一行被注释掉（`lerobot/robots/alohamini/alohamini_client.py:39`），Host 的 root logger 停在 WARNING | 只读核对源码；未拿到改 vendor 的授权，改 vendor 无解；已有一条不依赖它的取数路径（见「静止电流基线」一行） | `stdout/6.1d-currents-src-*`、`stdout/6.1d-host-loglevel-*`、`stdout/6.1b-currents-dist-20261105T063600Z.log`、`stdout/7.2b-host-tracking-count-20261105T063600Z.log` |
| 手册 §13 的一条补救假设被推翻 | 三轴 `rise_90_ms count=0 p50=nan`，三份 tracking CSV 的 `rise_90_time_s` 列全空；容差取到 `0.05°`（小于 `0.1°`）仍然空 | 报告自带说明：要 90% 交叉需残余 ≤ `0.1 × delta` 且发生在 settle 循环结束之前，实际没发生；原因未定位（见「未取到 / 未验证」） | 三次 `stdout/7.1-*.log` 的 `rise_90_ms count=0 p50=nan` 行与 `note:` 行、三份 `tracking/*.csv` |
| M0 离线复算：1° 在总线上是 11 tick，换算链清白 | 1° ÷ 0.0879°/tick（= 360/(4096-1)）= 11.3766 tick，M0 记 11.375 tick，vendor `motors_bus.py:895` 的 `int()` 向零截断得 11 tick；每步取整损失 0.375 tick（< 1 tick 上界），命令是绝对量故不累加；腕部正对照 10/10 成立（期望 11.375 tick/步、实测 10 tick/步，`meas-goal` 恒 −1 tick 而非累积） | **离线复算，非现场采集**：不连机器人、不新增测量；输入是指纹 `capture/fingerprint-20261105T063600Z.json` 与 `tracking/` 六份 CSV 的复算，加本库 `src/` 与 vendor fork `6aee1f81` 的静态核对 | `stdout/M0-tick-recompute-20261105.txt`（含 `.py`） |
| 收尾 Host 已停，机器人断电未执行 | `kill 2096`、`kill_exit=0`、停机时刻 `2026-11-05T07:08:54Z`；排除自匹配后 `pgrep` 无输出、`ss_grep_exit=1`（5555 / 5556 无监听） | 无远程可控电源通道；手册 §8.1 要求断电前先把手臂放到低位，机器人仍带电、位置环通电，差人工一步 | `stdout/8.2a-host-stop-20261105T063600Z.{cmd,log}`、`stdout/8.2b-host-verify-*`、`stdout/8.3c-power-state-*`、`stdout/8.1-host-stop-20261105T063600Z.md`、`stdout/scope-20261104T023000Z.md` 的「收尾记录」段 |

§7.2 判读链（`pan` / `lift` 两轮）：分析脚本机械判为 ②（目标被底层接管），前导帧（`sample_sequence=2`，早于第一条命令帧 `sample_sequence=3`）已排除；实质结论 ①：停滞不在 PC→Host 值传递段。判读细节与证据行号见 `stdout/7.2-analysis-20261105T063600Z.txt`。腕部那一轮同一位置的前导帧是 `none` 而非 `watchdog`。

### 肩部停滞：排除法结论

| 编号 | 根因 | 状态 | 一句依据 |
| --- | --- | --- | --- |
| A | 目标编码取整（归一化→tick 往返取整把 1° 命令吃掉） | 已排除 | M0 离线复算：期望步长 11.375 tick→`int()` 后 11 tick，实测步长 0–1 tick，取整损失 0.375 tick < 1 tick 且命令是绝对量不累加（`stdout/M0-tick-recompute-20261105.txt` 的 (c)、(e7)） |
| B | PC→Host 值传递段（目标在 Host 端被改写或钳制） | 已排除 | 两轴步内 `requested_target == accepted_target` 零例外，`accepted_target` 变化点与主 CSV 的 `cmd_units` 逐位字符相同，运动帧 `target_source` 全为 `command`；sidecar 旁证 req/acc/cmd 逐字符相同（wrist 177/178、pan 116/119、lift 175/176 条非空样本） |
| C | 伺服寄存器配置与限流钳制 | 已排除 | 2026-10-25 停 Host 后只读：`Torque_Limit` 1000、`Max_Torque_Limit` 1000、`P`/`I`/`D` = 16/0/32、`Operating_Mode` 0，四台逐项相同（`stdout/reg_read_1792923639.log`，152 次读零失败）；且「`requested == accepted` 零例外」只说明 `_limit_*_by_current` 未钳制值 |
| D | 关节扭矩未使能 | **未排除（被弱化，不作定论）** | 源码依据：`configure()` 在 `alohamini.py:520` 调 `left_bus.disable_torque()`、`:545` 的 `enable_torque()` 被注释、`disconnect` 默认 `disable_torque_on_disconnect=True`（`config_alohamini.py:56`）；但不是「全包无调用点」，且与「腕部同总线达成 87.91% 且两轮逐字相同」不相容；由 `docs/field-ops/50-experiments.md` §10.5 的 M1 分支 ① 覆盖 |
| E | 伺服保护锁存 | 未排除 | 无本窗口的寄存器侧观测；`Status` 全 0 只在 2026-10-25 停 Host 后读到 |
| F | 力矩不足 | 无证据（降级为「无电流证据」） | 臂总线电流不能判负载：运动窗口最大 `19.5 mA`，低于 `20 mA`（3 个 raw 刻度）门槛 |
| G | 机械 | 未排除 | 未到现场，无齿隙 / 死区 / 连杆测量 |
| — | 总线写静默失败 | 未排除（零观测） | `sync_write` 无 ACK；`requested_targets` / `accepted_targets` 在本版 vendor 上分别是 `_arm_goal_positions` 与 `_arm_sent_positions`（`alohamini.py:987-988`），后者在 `:919` 于 `sync_write` 调用之后、无回读即 `update`，故不携带总线写入信息 |

弱证据一条：实测组合 `{Torque_Enable=1, Lock=0}` 与 vendor 两条写路径都不相容——`feetech.py:293` 写 `{0, 0}`、`:304` 写 `{1, 1}`，本库 `src/` 不写这两个寄存器；该组合与「`Lock` 写静默失败」相容，但这是**未验证的假说**，不是结论。

未排除者合计：D / E / G + 总线写静默失败；F 无证据。定性动作（teleop oracle → M1 → M2）见 `docs/field-ops/50-experiments.md` §10.5，均需有人到场。

### 未取到 / 未验证

| 项目 | 状态 |
| --- | --- |
| 三轴一致变差（87.9 → 87.91 持平、8.8 → 4.40、13.2 → 0.00）的原因 | 未定位。电压、温度、机械三项都没有测量。差值只有 0.5–1.5 tick（1 tick = 0.0879°），不足以当硬件劣化证据 |
| Host 日志 3 行 `WARNING:root:Host command watchdog expired; stopping motion` 与三次 run 的一一对应 | 假说。计数与三次 run 吻合（3 行对 3 次，客户端断开后的超时），未验证 |
| 7 只恒 0 mA 电机（`arm_left_gripper`、`arm_left_wrist_yaw`、`arm_right_gripper`、`arm_right_wrist_flex`、`arm_right_wrist_roll`、`base_left_wheel`、`base_right_wheel`）是否正常 | 未验证。这 7 只全程读数恒 0，是否等于「没有电流」未验证；本窗口已判定电流不能当负载证据 |
| 机器人断电 | 未执行。待人工到现场，手册 §8.1 要求断电前先把手臂放到低位 |
| `rise_90_time_s` 恒空的原因 | 未定位。容差已取到 `0.05°`，手册原假设（容差 < 0.1° 可补上这一列）被推翻 |
| 右臂试动数据 | 未取。`tracking/` 六份文件（三份 tracking + 三份 safety）全为 `arm_left_*` |
| 相机帧数据 | 未取。`capture/` 内无图像文件，`robot_metadata.cameras` 只有 `forward` / `wrist_right` 两项（口径同 2026-10-25 节，仍是未取） |

### 读法

| 口径 | 怎么算 |
| --- | --- |
| STAMP 双标签映射 | 第 1 步产物用 STAMP `20261104T023000Z`（会话初始化时写下的约定值，与实测 `date -u` 相差约 28 小时），第 2 步起用实测 STAMP `20261105T063600Z`；映射与顺序偏离见 `stdout/scope-20261104T023000Z.md` 的「执行中补记」段，读文件名时不要当成两次会话 |
| 达成率 | 沿用 2026-10-25 节同一口径（净行程 ÷ 指令行程），本文不重复推导 |
| 电流 | 落盘值已是 mA（`raw × 6.5` 截断）：`6.5` = 1 个 raw 刻度，`20 mA` = 3 个刻度即判读门槛。本窗口运动窗口最大 `19.5 mA`，仍不足 3 个刻度，所以「不能判负载」；早前按 `[Currents]` 日志统计的口径在本窗口用不上（该日志一行都没打印，见「结论一览」的机理一行）。量级对照：同总线（`lift_axis` 属 `left_bus_motors`，`alohamini.py:215`）的升降轴单电机 `read` 曾读到 `2639.0 mA`（2026-10-25，`evidence/s4-20261025/stdout/v2verify-host-20261025.log` 的 `[lift_axis.home]` 行），比本窗口大两个数量级；但那走单电机 `read` 路径，11 电机 `sync_read` 路径没有大电流正对照，电流读取链路的大电流可信性仍未验证。**若将来要判负载，需外接电流表**（臂总线读数不足以承担）。 |
| `target_source` 前导帧 | 每轮第一帧出现在第一条命令帧之前，那一帧的 `target_source`（`none` 或 `watchdog`）不属运动阶段，判三类结论时要排除 |
| `settled` 跨轮不可比 | 2026-10-25 容差 `0.15°`、本窗口 `0.05°`，同一关节 `settled=False` 的成因不同，比 `settled` 要同时比容差 |
| 时间基准 | 全部 UTC（`date -u`）；本机时为 UTC+8，钟点比日志数字大 8 小时 |

## 离线核对（无新采集）· 左右夹爪行程差异

- 左夹爪 `arm_left_gripper` 标定 `range` 只有 **51 tick**（`range_0_100` 下 **4.48°** 行程），判为异常；右夹爪同型对照 **1469 tick**（129.14°），右是左的 28.8 倍。
- 固件限位已实测确认（2026-11-05，只读）：左 ID7 `Min_Position_Limit=2021` / `Max_Position_Limit=2072`，右 ID7 对照 `2010` / `3479`，与标定文件逐项一致，故 51 tick 的成因在标定值，不在写盘链路。仍开放的是「标定时为什么只采到 51 tick」（人工开合是否发生 / 机械是否被限住），未排查。
- 完整查证过程（18 电机 `range` 全表、9 条逐条查证、可能原因排序、影响分析）见 `docs/investigations/gripper-range-51-tick.md`。**这一节不是现场采集**。

## 2026-10-25 · S4 现场窗口

条件：AlohaMini 2 Pro（`robot_model=alohamini2pro`），18 个电机；本库 v0.7.0；Pi 侧运行时 HEAD `6aee1f81`（升级前为 `ec65318f`）。

### 结论一览

| 结论 | 关键数字 | 条件 | 证据文件 |
| --- | --- | --- | --- |
| Pi 运行时已发 metadata v2 | `robot_metadata.schema_version=2`，18 个电机，`homing_offset` 与 `resolution` 无缺失 | 升级到 `6aee1f81` 后 | `capture/fingerprint-final-20261025.json`（7075 B） |
| 标定跨断电保持 | 18 个电机 × 8 个字段 = 144 个字段 0 差异 | 换电池前后各采一次 | `capture/site-fingerprint-battery2.json` 与 `capture/fingerprint-final-20261025.json` |
| 腕部跟随达标 | 净行程 `+1.758°`／指令 `2.0°`，达成率 87.9%，两步 `settled=True`，误差 `0.121°` | 左臂 `wrist_roll`，每步 1°，2 步 | `tracking/modeB-wrist-20261025.csv` |
| 肩部 pan 未跟随 | 5.9%（首次，1°×3 步）、8.8%（1°×3 步与 1°×2 步各一次），全部 `settled=False` | 左臂 `shoulder_pan`，每步 1° | `tracking/modeB-first-20261025.csv`、`tracking/modeB-shoulder-20261025.csv`、`tracking/modeB-pan2-20261025.csv` |
| 肩部 lift 未跟随 | 达成率 13.2%，两步全 `settled=False` | 左臂 `shoulder_lift`，1°×2 步 | `tracking/modeB-lift-20261025.csv` |
| 五次试动均未中途中止 | `note` 列只有 `ok` 与 `settle budget exhausted`；首次 report 记 `steps_completed=3`、`abort_reason=None`、`hold_issued=yes` | 左臂单关节，五次运行 | 五份 `tracking/*.csv`、`stdout/modeB-first-20261025.stdout`（stderr 0 字节） |
| 伺服寄存器配置查不出肩部与腕部的差别 | 152 次读全部成功、零失败；`Torque_Limit` 1000、`Max_Torque_Limit` 1000、`P`/`I`/`D` = 16/0/32、`Operating_Mode` 0，四台逐项相同 | 停 Host 后只读，左臂 ID 1/2/3/6 | `stdout/reg_read_1792923639.log` |
| 扭矩处于开启态，写入者未识别 | `Torque_Enable=1`、`Lock=0`；`Status` 全 0，温度 22–27 ℃、电压 10.6–10.8 V | 停 Host 后只读 | 同上 |
| 臂总线电流读得到，在静止与轻载下量级极小 | `[Currents]` 整份日志 1278 行、11502 个元素：值分布 `0` 11056 个、`6` 444 个、`13` 2 个，**最大 13 mA**（= 2 个 raw 刻度）；打印值**已经是 mA**（Host 侧按 `int(raw × 6.5)` 截断），不是 raw 刻度 | 全日志落在静止 / 轻载窗口；运动中的量级未验证 | `stdout/v2verify-host-20261025.log` |
| 回环 RTT | `net_rtt_est` p50 6.18 ms、p95 6.45 ms；`achieved_rate_hz=49.8`（请求 1000 Hz）；`host_timing_missing=0/100` | Pi 本机回环（`host=127.0.0.1`） | `rtt/rtt-loopback-merged-20261025.csv`、`stdout/3.2-rtt-20261025T094446Z.log` |
| Gate F 第一段有真实拒绝记录 | exit `6`、零命令；报文 `Gate F refused: the host metadata schema_version is 1, expected 2.` | v1 宿主，升级前 | `stdout/2.2b-gateF-attempt-20261025T074158Z.log` |
| 阶段 0 到场核对产物齐备 | `stdout/0.*`：Pi 提交、`schema_version` 行、Host 源码读数与端口默认值 `5555` / `5556` | 到场后 SSH 只读核对 | `stdout/0.2-pi-commit-20261025T070057Z.log`、`stdout/0.3-pi-schema-20261025T070057Z.log`、`stdout/0.22b-port-defaults-20261025T070856Z.log` |
| P0 关闭：运行时已是 metadata v2 | 完全跑通路径 `/home/pi5/lerobot_alohamini`，HEAD `6aee1f81d503be98972d605ed32189221a674607`（`main` 与 `codex/alohamini-cartesian-teleop` 的合并提交链；升级前 UTC 07:00 前后 HEAD `ec65318f`）；旧部署备份 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`（`ec65318f` + 6 个未提交改动，原样保留）；升级后厂商测试 `60 passed`（该次测试的原始输出未入证据目录），`import lerobot` 落点 `/home/pi5/lerobot_alohamini/src/lerobot/__init__.py` | 关闭判据：`capture` exit `0` 且 JSON 里 `robot_metadata.schema_version=2`；UTC 09:33:54 的采指纹已在这版上完成 | `capture/fingerprint-final-20261025.json`、`stdout/0.11b-lerobot-install-20261025T070856Z.log`、`stdout/5.4-pi-lerobot-env-20261025T074005Z.log` |
| 阶段 1 指纹重采与 codex 原分支逐字段一致 | `sample_count=5`；逐字段对比 `field diffs: 0`（该对比的原始报告未入证据目录） | 升级到 `6aee1f81` 后重采终版指纹 | `capture/fingerprint-final-20261025.json` |
| 阶段 1 回环 RTT 的未四舍五入分位数 | `request_age_ms` p50 `20.109`；`net_rtt_estimate_ms` p50 `6.176`；`host_timing_missing=0/100` | Pi 本机回环（`host=127.0.0.1`），请求 1000 Hz、100 样本 | `rtt/rtt-loopback-merged-20261025.csv` |
| 阶段 4 配置样例的三键取自现场 Host 源码默认值 | `observation_request_window=3`、`watchdog_timeout_ms=1000`、`max_loop_freq_hz=50`，出处 Pi `config_alohamini.py:98/110/113`；入库样例 `configs/template.yaml`、`configs/site.yaml`（`load_settings` 读通由 `tests/test_examples_and_configs.py` 契约 B 覆盖），命令与日志 `stdout/4.*` 齐备 | 只读读 Host 源码，未改任何参数 | `stdout/4.3-host-config-20261025T070057Z.log`、`stdout/4.3b-config-args-20261025T070203Z.log` |

### 未取到的项目

| 项目 | 原因 |
| --- | --- |
| 跨机 RTT | 只在 Pi 上做过回环测量，`host=127.0.0.1`；PC 到 Pi 的真实网段未测 |
| Gate F 第二段拒绝记录 | v2 宿主 18/18 电机字段齐全（`stdout/5.1-probe21-20261025T094457Z.log`），第二段结构上不会触发 |
| 右臂试动数据 | 本轮只动左臂，`tracking/` 五份 CSV 全为 `arm_left_*` |
| 相机帧数据 | 采集时返回零帧，目录内无任何图像文件；`capture/*.json` 的 `robot_metadata.cameras` 只有 2 项，「4 路相机全零帧」这一条在证据目录里没有直接记录 |
| 运行后的回弹量 | 试动结束后未继续采样该轴，`tracking/` 里没有运行之后的记录 |
| 运动窗口内的电流量级 | 现有 `[Currents]` 最大 13 mA（2 个 raw 刻度），全落在静止与轻载窗口；运动中的读数未采集 |

### 读法

| 口径 | 怎么算 |
| --- | --- |
| 达成率 | 净行程 ÷ 指令行程。净行程 = 末步 `measured_deg` − 首步 `baseline_deg`；指令行程 = Σ（每步 `cmd_deg` − 该步 `baseline_deg`），不用「步数 × 1°」。例：`1.7582 ÷ 2.0000 = 87.9%` |
| 停稳 | `settled=True` 指该步在 settle 预算内稳住（本轮容差 0.15°）；`note` 为 `settle budget exhausted` 即预算用完仍未稳 |
| 1 tick | 0.0879°（`360/(4096-1)`）。腕部每步约 10 tick；肩部在 2026-10-25 窗口每步约 1 tick，在 2026-11-05 窗口 `pan` 整轮 2 步共 1 tick（0.5 tick/步）、`lift` 整轮 0 tick |
| `tracking/` CSV 列 | 14 列含义见 `docs/field-ops/60-artifacts.md` 第 11 节。要点：`cmd_deg` 是该步的绝对目标角，`baseline_deg` 是该步起点的实测角，`measured_deg` 是该步收尾的实测角 |
| 回环 RTT 的读法 | `net_rtt_est_s` 是扣掉 host 采样、相位、排队与预取等待之后的估计值，不是纯往返时延；`host=127.0.0.1` 表示这是 Pi 本机回环，不能当网络延迟读 |
| 电流的单位、粒度与计数 | `[Currents]` 打印值已是 mA（Host 侧 `int(raw × 6.5)` 截断）：`6` 是 1 个 raw 刻度，`13` 是 2 个刻度。同一份日志里升降轴 `Present_Current` 读到 `71.5` / `221.0` / `936.0` / `2639.0` mA，说明电流读取本身正常；局限在臂总线静止与轻载下只有 0–2 个 raw 刻度。**运动中的量级未验证**，不要拿它反推负载。计数须按整份文件复算：`grep -c '\[Currents\]'` 给 1278、元素 11502；旧版本 `docs/field-ops/30-host-ops.md` 第 6 节按前缀统计留下的「1020 行、9180 个元素」已作废，那组数只到本文件第 1035 行（约前 50 KB），比整份文件少 258 行，按前缀统计会偏小；现该文第 6 节与本文都按整份文件复算，引用时不要混用。判读门槛见 `docs/field-ops/30-host-ops.md` 第 6.2 节 |
| 时间基准 | 证据文件名与日志时间戳都由 `date -u` 生成，是 UTC。本文件正文里的日期（如 `2026-10-25`）也是 UTC 日期。本机时间是 UTC+8，钟点比日志数字大 8 小时：UTC 07:00 对应本地 15:00，UTC 09:33 对应本地 17:33。Pi 的系统时区在证据目录内没有 `timedatectl` 记录 |
