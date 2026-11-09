# S4 现场执行手册（`docs/field-ops/`）

这个文件夹是 S4 现场执行手册：一次现场窗口怎么从到场核对做到收尾。手册按原节号顺序拆成 7 份子文档，节号沿用原单文件，一位未重排。

历史曾用名：原为单文件 `docs/s4-field-smoke.md`，拆分后节号与内容未改。

缩写与人话对照，第一次读先看这张表；其余术语在用到的小节里给一句解释。

| 术语 | 一句人话 |
| --- | --- |
| S4 | 把本库接到真机上跑一轮最小验证的阶段，手册原名里的 `smoke` 就是这个意思 |
| 现场窗口 | 一次完整到场作业：从 SSH 连通做到断电收尾 |
| `signs` / `zeros` | 关节角与电机刻度之间的两个映射参数：符号管方向，零点管偏置 |
| Gate D | 运行时门禁之一：决定这次测量能不能发命令的前置检查 |
| `move_to` | 库的笛卡尔入口：给末端一个目标位姿，由 IK（逆运动学，把末端位姿算成六个关节角）解出来再下发 |
| sidecar | 与主 CSV 同一窗口写出的 safety 快照（文件名是 `<主CSV名>-safety.csv`），逐帧记 `requested_target` 与 `accepted_target` |
| tick | 电机位置读数的最小刻度，`1 tick = 0.087890625°` |

## 1 工作流总览（一次现场窗口的固定链）

编号即执行顺序，每步一行；命令与判据在各步指向的小节里，本节不重复。

1. **§2 变量块与 `run` 函数**：同一 shell 粘一次，`$S4DIR` / `$STAMP` 由 `date -u` 定死，后续命令都假定这几步已完成。→ `00-opening.md` §2
2. **§3 到场核对**：网络、Host 进程、端口、工具、设备路径逐条过；其中 §3.9 授权记录落盘（五项填满）、§3.10 人工确认项逐条勾。→ `00-opening.md` §3
3. **两条安全前置口述确认**：就在 §3.9 记 scope 时当面口述并落盘——手臂低位或有承托；能按断电开关的人在机器人旁。→ `00-opening.md` §3.9
4. **§10.8 `signs` / `zeros` 标定（分段做，不是整段做完才起 Host）**：摆参考位形在 **torque 释放态**下做（人手工摆臂）→ §6.1 起 Host 后用 capture 读数（§4.1）→ 算出 `signs` / `zeros` 后 §8.1 停 Host；只读观测档与运动档分离，硬前置与判据见 §10.8「安全前置」。→ `50-experiments.md` §10.8，其中 §4.1 见 `10-health-baseline.md`、§6.1 与 §8.1 见 `30-host-ops.md`
5. **§6.1 起 Host**：按三步法起，停 Host 用的 PID 取 `6.1a-host-pid` 里 `ps` 打出的那个（不是 `echo pid=$!`）。→ `30-host-ops.md` §6.1
6. **§10.6 Gate D**：按该节的唯一触发路径取数；等不到就记「未触发」，不另造注入路径。→ `50-experiments.md` §10.6
7. **§10.7 `move_to` 与卷尺实测**：①a / ② / ③ 的前置是 §10.8 产出的两份 JSON；④ 卷尺实测缺量具就记「未测」，不用目测替。→ `50-experiments.md` §10.7
8. **收尾 §8.1**：停 Host ≠ 失力——先把手臂放到低位，再断电。→ `30-host-ops.md` §8.1

两条安全前置（手臂低位或有承托、断电开关在手且有人在旁）必须在 §3.9 记 scope 时口述确认并落盘，缺一条即停。

上面这条链是固定的：开场（§2/§3）→ `signs` / `zeros` 标定（§10.8）→ 起 Host（§6.1）→ 实验（§10.6 Gate D、§10.7 `move_to` 与卷尺）→ 收尾（§8.1）。

**任何一步缺前置就停在那一步。** 前置清单是 `00-opening.md` §1 的 P0–P5 加上各步所在小节的「安全前置」；不允许跳过一步往下做，也不允许手写或猜出一个缺失的值来补齐前置。

## 2 路由：每份文件管什么

### 2.1 设计参考

口径与判据的出处；现场遇到「这条要不要判」「数字算得对不对」，查这四份。

| 文件 | 管什么 |
| --- | --- |
| `docs/ADR.md` | 决策记录 D1–D21：设计取舍与已定口径（如 D14 左夹爪本轮不重标定、D18 sidecar 单独成表、D20 残余支分级、D21 URDF 零实测） |
| `docs/acceptance-baseline.md` | 验收基线 V01–V13：一条判据一行，含实机证据列 |
| `docs/library-guide.md` | 接口细节：配置覆盖、能力探测与单位换算、控制层、公开 API、约束与机器守卫 |
| `docs/background.md` | 目标、已实现能力与实机现状 |

### 2.2 程序子文档（本文件夹）

节号范围一位未改，与拆分前一致。

| 文件 | 负责的原节号 |
| --- | --- |
| `00-opening.md` | 原 H1 前言、§0、§1、§2、§3 |
| `10-health-baseline.md` | §4 |
| `20-toolchain.md` | §5 |
| `30-host-ops.md` | §6、§8 |
| `40-gates-params.md` | §7、§9、§12、附录 A |
| `50-experiments.md` | §10、§10.5–§10.8 |
| `60-artifacts.md` | §11、§13 |
| `completed-work.md` | 本文件夹的「已执行工作」：结论速览、时间线、数字入口；不含命令与判据 |

### 2.3 证据根

原件都在 `~/pi-workspace/alohamini-cartesian-teleop/evidence/`，目录内 `README.md` 写命名约定。按窗口分目录：

| 目录 | 对应场次 |
| --- | --- |
| `s4-20261025/` | 2026-10-25 及更早 |
| `s4-20261104/` | 2026-11-05 节（会话 STAMP 从 `20261104T023000Z` 起） |
| `s4-20261106/` | 2026-11-06 节 |
| `s4-20261107/` | 2026-11-08 节第一段（目录名按授权所取，目录内 STAMP 与全部时刻是 11-08 UTC） |
| `s4-20261108/` | 2026-11-08 节第二段（路 A 复核） |

### 2.4 台账

`docs/field-results.md`：现场数字与结论，按时间倒序。每个日期节给「结论 / 关键数字 / 条件 / 证据文件」四列，「证据文件」列是该节所属证据目录下的相对路径。

### 2.5 节号 → 文件对照

在任何一份子文档里看到「见第 12 节」这类指针，按这张表查文件。本文件夹所有 `§N` 指针沿用原单文件的节号，写到别的文件时补上文件名前缀。

| 节号 | 文件 |
| --- | --- |
| §0–§3 | `00-opening.md` |
| §4 | `10-health-baseline.md` |
| §5 | `20-toolchain.md` |
| §6、§8 | `30-host-ops.md` |
| §7、§9、§12、附录 A | `40-gates-params.md` |
| §10、§10.5–§10.8 | `50-experiments.md` |
| §11、§13 | `60-artifacts.md` |

## 3 安全底线与已确证事实速查

这两个小节照原文移入，小节标题与字母编号未改。唯一的改动是 `### A.` 的出处列：指针补了文件名前缀，节号一位未改。

### C. 安全底线

1. **Host 只 `SIGINT`（`kill -INT`），不 `SIGTERM`、不 `kill -9`（除 §10.5 的 M1 / M2 取证）** → §8.1、§10.5
2. **`--anchor command` 与 `move_to` 必须有能按断电开关的人在机器人旁**；远程无人在场只跑 `--anchor present` → §7.1
3. **sidecar 是主证据，寄存器尸检不可信**（无 sidecar 时靠时刻先后与 Host 日志有无 `Host command watchdog expired`） → §10.5

### A. 已确证事实速查

| 事实 | 一句话 | 出处 |
| --- | --- | --- |
| 网络与 IP | `PI_HOST` 是唯一需现场填的值，上次记录 `10.34.170.98`；连不上按四步处置：`ping` → mDNS `pi5.local` → 问现场要 IPv4 → 路由器 DHCP 列表找 `pi5` | 00-opening.md §2「变量块」与「SSH 连接检查与连不上时的处置」、00-opening.md §3.1 |
| Host 启动三步法 | ① 本地后台跑 ssh（远端 `setsid nohup … &`）→ ② 轮询 5556 端口最多 90 s → ③ 端口起来后杀本地 ssh | 30-host-ops.md §6.1 |
| 停 Host 用的 PID | 取 `6.1a-host-pid` 里 `ps` 打出的那个，**不要**用 `echo pid=$!`（`setsid` 自己占掉一个 PID；2026-11-06 实测 `2516` vs `2517`） | 30-host-ops.md §6.1 |
| Host 启动命令带 `python -u` | 无缓冲，日志实时落 `/tmp/site-host-$STAMP.log` | 30-host-ops.md §6.1 |
| 串口别名 | `/dev/am_arm_follower_left` → `ttyACM0`、`/dev/am_arm_follower_right` → `ttyACM1`；`/dev/serial/by-id/` 的 `usb-1a86_…` 名字**不能**代替这两个别名 | 10-health-baseline.md §4.2、00-opening.md §3.7 |
| 当前基线指纹 | `capture/fingerprint-20261108T141936Z.json`（`captured_utc=2026-11-08T14:20:21+00:00`）、`capture/fingerprint-postpower-20261108T141936Z.4.1c-preserved.json`（`…T14:20:35+00:00`）、收尾 `capture/fingerprint-postpower-20261108T141936Z.json`（`…T14:28:12+00:00`，与前者同名冲突的那份） | `docs/field-results.md` 2026-11-08 节「指纹三份」；原件 `~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261108/capture/fingerprint-*.json` |
| 断电无滑移首判 | 第一份不受「期间有运动」污染的 B1 判据：14 只位置电机 `delta` 全 `+0`，唯 `arm_left_shoulder_pan` 为 `-1` tick；`CALIB_FIELD_DIFFS: 0`、`POSITION_MOTORS_OVER_3_TICK: 0` | `docs/field-results.md` 2026-11-08 节表 1 |
| 肩部全案结论 | `--anchor command` 5 步 × 1° 达成率左 `77.3626%`、路 A 重跑左 `79.1209%` / 右 `82.6374%`；步末残余 `0.87–1.04°`（`9.88–11.88` tick）两侧同量级 → 目标已被 Host 采纳，残余在 Host 下游伺服／机械静摩擦；处置按分级口径（肩类 `≤2°`、其余 `≤2 tick`） | `docs/field-results.md` 2026-11-06 与 2026-11-08 节；`docs/ADR.md` D20 |
| 臂总线电流不能判负载 | 2026-11-05 那轮静止基线与运动窗口读数全部低于 `20 mA` 门槛（`≥20 mA` 样本 0 个）；**2026-11-08 sidecar 已采到非零电流**（左 max `104.0 mA`、`≥20 mA` 样本 `700`；右 `71.5 mA` / `153` 样本）——「电流不可判负载」结论仅限 11-05 口径，**待按 sidecar 数据重裁** | 30-host-ops.md §6.2；`docs/field-results.md` 2026-11-05 节与 2026-11-08 节 |
| 电池电压口径 | `Present_Voltage` 原始值 min `116` / max `119`（地址 62、1 字节、无符号、全链无换算，源码侧已核）；`11.6–11.9 V` 用 `0.1 V/单位` 的比例算，该比例来自 Feetech 手册、源码不提供，故换算值只作相对比较，**不得**当标定或实测证据 | `docs/field-results.md` 2026-11-08 节 B3 行；10-health-baseline.md §4.2「满电电压基准的判据」 |
| `Torque_Enable` | 写 `Goal_Position` 时固件会自置 `1`（外部来源、非本轮实测）；只有 `SIGINT`（`kill -INT`）会释放扭矩，`kill -9` / `SIGTERM` 不释放 | 30-host-ops.md §8.1；`docs/field-results.md` 2026-11-06 节 |
| kill 纪律 | M1 / M2 取运动中的扭矩状态只许 `kill -9`，**禁用 `SIGTERM`**（干净退出走 `disable_torque`，读到的是退出后状态，读数作废） | 50-experiments.md §10.5 |
| sidecar 是肩部判读主证据 | `tracking-*-safety.csv` 的 `target_source`、`requested_target == accepted_target`、`hold_active`、`current_ma`；`[HOST TRACKING]` 与 `[Currents]` 因 vendor 日志级别压制不打印，不可依赖 | 30-host-ops.md §6.1、40-gates-params.md §7.2、60-artifacts.md §11；`docs/ADR.md` D18 |
| sidecar 解析细节 | 文件是 CRLF 行尾，`tr -d '\r'` 不能省；`$9` 是 `target_source`、`$2` 是 `sample_sequence` | 50-experiments.md §10.5、60-artifacts.md §11 |
| 寄存器尸检的可信边界 | 客户端先死／无 CSV／无 sidecar 时，判读只靠 Host `kill` 与客户端收尾的时刻先后 + Host 日志有无 `Host command watchdog expired`；单看寄存器读数不可信 | 50-experiments.md §10.5 |
| scope 模板五项 | 时间 UTC / 授权人 / 授权范围 / 允许的动作 / 不允许的动作，落 `stdout/scope-$STAMP.md`；不过 `run`（heredoc 正文不进 `.cmd`） | 00-opening.md §3.9 |
| URDF 来源与零实测 | 派生自厂商上游 `am-arm200pro.urdf`（commit `5b03af1ef6c80595bd988e0137cf8e7c35811eb6`，sha256 `c38669d46d2bf804c112a6d8ce926074c8ff65ba70917503e09b77c75e285030`），`sources.json` 自报 `hardware_verified: false`，本库无杆长实测 | `docs/library-guide.md` §3.1；`docs/ADR.md` D21 |
| `signs` / `zeros` 契约 | 正向 `q_kinematic_deg = sign × motor_deg + zero`；反向 `motor_deg = (q − zero) / sign`；`sign` 只许 `±1`，库内无默认值，不注入就构造不出适配器 | 50-experiments.md §10.8 |
| §4.1c 两组判据 | A 组（标定字段）任一不等 = 事故，停；B 组位置阈值 `3 tick`（本项目自定上界，非厂商值） | 10-health-baseline.md §4.1c |
| 换算常数 | `1 tick = 360/4096° = 0.087890625°` | 10-health-baseline.md §4.1c；`docs/field-results.md` 2026-11-08 节 |
| 左夹爪只有 51 tick | `2021..2072`，`range_0_100` 下 `4.48°`；D14 已定本轮不重标定，只读确认 | 10-health-baseline.md §4.2；`docs/ADR.md` D14 |
| 未标定前的可执行性 | §10.7 的 ①a / ② / ③ 缺 `signs` / `zeros` 直接按 `15` 退出，**不得**手写或猜 | 50-experiments.md §10.7、50-experiments.md §10.8 |
| 时间基准 | Pi 时区 `Europe/London`，日志时间等于 UTC，本地是 UTC+8；`$S4DIR` / `$STAMP` 按 `date -u` 生成 | 00-opening.md §2 |
| 产物路径纪律 | 一律显式传 `--out` / `--csv`：默认文件名时间戳只到秒，同一秒内跑两次会静默覆盖 | 00-opening.md §3.8 |
