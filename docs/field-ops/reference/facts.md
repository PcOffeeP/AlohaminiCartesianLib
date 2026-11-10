# 已确证事实与术语速查（原 README §3.A 与术语表）

本文件管什么：两件事——第一次读手册先认的七个术语，以及「这件事上次已经查清了吗」的 24 条已确证事实（一条事实一句人话一个出处）。事实的出处列都指向现行文件里的锚点，点开即到。

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

## A. 已确证事实速查

| 事实 | 一句话 | 出处 |
| --- | --- | --- |
| 网络与 IP | `PI_HOST` 是唯一需现场填的值，上次记录 `10.34.170.98`；连不上按四步处置：`ping` → mDNS `pi5.local` → 问现场要 IPv4 → 路由器 DHCP 列表找 `pi5` | [§2](../runbook/1-prepare.md#sec-2)「变量块」与「SSH 连接检查与连不上时的处置」、[§3.1](../runbook/1-prepare.md#sec-3) |
| Host 启动三步法 | ① 本地后台跑 ssh（远端 `setsid nohup … &`）→ ② 轮询 5556 端口最多 90 s → ③ 端口起来后杀本地 ssh | [§6.1](../runbook/2-host.md#host-start) |
| 停 Host 用的 PID | 取 `6.1a-host-pid` 里 `ps` 打出的那个，**不要**用 `echo pid=$!`（`setsid` 自己占掉一个 PID；2026-11-06 实测 `2516` vs `2517`） | [§6.1](../runbook/2-host.md#host-start) |
| Host 启动命令带 `python -u` | 无缓冲，日志实时落 `/tmp/site-host-$STAMP.log` | [§6.1](../runbook/2-host.md#host-start) |
| 串口别名 | `/dev/am_arm_follower_left` → `ttyACM0`、`/dev/am_arm_follower_right` → `ttyACM1`；`/dev/serial/by-id/` 的 `usb-1a86_…` 名字**不能**代替这两个别名 | [§4.2](../runbook/3-health.md#sec-4-2)、[§3.7](../runbook/1-prepare.md#sec-3) |
| 当前基线指纹 | `capture/fingerprint-20261108T141936Z.json`（`captured_utc=2026-11-08T14:20:21+00:00`）、`capture/fingerprint-postpower-20261108T141936Z.4.1c-preserved.json`（`…T14:20:35+00:00`）、收尾 `capture/fingerprint-postpower-20261108T141936Z.json`（`…T14:28:12+00:00`，与前者同名冲突的那份） | `docs/field-results.md` 2026-11-08 节「指纹三份」；原件 `~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261108/capture/fingerprint-*.json` |
| 断电无滑移首判 | 第一份不受「期间有运动」污染的 B1 判据：14 只位置电机 `delta` 全 `+0`，唯 `arm_left_shoulder_pan` 为 `-1` tick；`CALIB_FIELD_DIFFS: 0`、`POSITION_MOTORS_OVER_3_TICK: 0` | `docs/field-results.md` 2026-11-08 节表 1 |
| 肩部全案结论 | `--anchor command` 5 步 × 1° 达成率左 `77.3626%`、路 A 重跑左 `79.1209%` / 右 `82.6374%`；步末残余 `0.87–1.04°`（`9.88–11.88` tick）两侧同量级 → 目标已被 Host 采纳，残余在 Host 下游伺服／机械静摩擦；处置按分级口径（肩类 `≤2°`、其余 `≤2 tick`） | `docs/field-results.md` 2026-11-06 与 2026-11-08 节；`docs/ADR.md` D20 |
| 臂总线电流不能判负载 | 2026-11-05 那轮静止基线与运动窗口读数全部低于 `20 mA` 门槛（`≥20 mA` 样本 0 个）；**2026-11-08 sidecar 已采到非零电流**（左 max `104.0 mA`、`≥20 mA` 样本 `700`；右 `71.5 mA` / `153` 样本）——「电流不可判负载」结论仅限 11-05 口径，**待按 sidecar 数据重裁** | [§6.2](../runbook/4-experiments.md#exp-e2)；`docs/field-results.md` 2026-11-05 节与 2026-11-08 节 |
| 电池电压口径 | `Present_Voltage` 原始值 min `116` / max `119`（地址 62、1 字节、无符号、全链无换算，源码侧已核）；`11.6–11.9 V` 用 `0.1 V/单位` 的比例算，该比例来自 Feetech 手册、源码不提供，故换算值只作相对比较，**不得**当标定或实测证据 | `docs/field-results.md` 2026-11-08 节 B3 行；[§4.2](../runbook/3-health.md#sec-4-2)「满电电压基准的判据」 |
| `Torque_Enable` | 写 `Goal_Position` 时固件会自置 `1`（外部来源、非本轮实测）；只有 `SIGINT`（`kill -INT`）会释放扭矩，`kill -9` / `SIGTERM` 不释放 | [§8.1](../runbook/2-host.md#sec-8-1)；`docs/field-results.md` 2026-11-06 节 |
| kill 纪律 | M1 / M2 取运动中的扭矩状态只许 `kill -9`，**禁用 `SIGTERM`**（干净退出走 `disable_torque`，读到的是退出后状态，读数作废） | [§10.5](../runbook/4-experiments.md#exp-e5) |
| sidecar 是肩部判读主证据 | `tracking-*-safety.csv` 的 `target_source`、`requested_target == accepted_target`、`hold_active`、`current_ma`；`[HOST TRACKING]` 与 `[Currents]` 因 vendor 日志级别压制不打印，不可依赖 | [§6.1](../runbook/2-host.md#host-start)、[§7.2](../runbook/4-experiments.md#exp-e3)、[§11](artifacts.md#sec-11)；`docs/ADR.md` D18 |
| sidecar 解析细节 | 文件是 CRLF 行尾，`tr -d '\r'` 不能省；`$9` 是 `target_source`、`$2` 是 `sample_sequence` | [§10.5](../runbook/4-experiments.md#exp-e5)、[§11](artifacts.md#sec-11) |
| 寄存器尸检的可信边界 | 客户端先死／无 CSV／无 sidecar 时，判读只靠 Host `kill` 与客户端收尾的时刻先后 + Host 日志有无 `Host command watchdog expired`；单看寄存器读数不可信 | [§10.5](../runbook/4-experiments.md#exp-e5) |
| scope 模板五项 | 时间 UTC / 授权人 / 授权范围 / 允许的动作 / 不允许的动作，落 `stdout/scope-$STAMP.md`；不过 `run`（heredoc 正文不进 `.cmd`） | [§3.9](../runbook/1-prepare.md#sec-3) |
| URDF 来源与零实测 | 派生自厂商上游 `am-arm200pro.urdf`（commit `5b03af1ef6c80595bd988e0137cf8e7c35811eb6`，sha256 `c38669d46d2bf804c112a6d8ce926074c8ff65ba70917503e09b77c75e285030`），`sources.json` 自报 `hardware_verified: false`，本库无杆长实测 | `docs/library-guide.md` §3.1；`docs/ADR.md` D21 |
| `signs` / `zeros` 契约 | 正向 `q_kinematic_deg = sign × motor_deg + zero`；反向 `motor_deg = (q − zero) / sign`；`sign` 只许 `±1`，库内无默认值，不注入就构造不出适配器 | [§10.8](../runbook/4-experiments.md#exp-e8) |
| §4.1c 两组判据 | A 组（标定字段）任一不等 = 事故，停；B 组位置阈值 `3 tick`（本项目自定上界，非厂商值） | [§4.1c](../runbook/3-health.md#sec-4-1c) |
| 换算常数 | `1 tick = 360/4096° = 0.087890625°` | [§4.1c](../runbook/3-health.md#sec-4-1c)；`docs/field-results.md` 2026-11-08 节 |
| 左夹爪只有 51 tick | `2021..2072`，`range_0_100` 下 `4.48°`；D14 已定本轮不重标定，只读确认 | [§4.2](../runbook/3-health.md#sec-4-2)；`docs/ADR.md` D14 |
| 未标定前的可执行性 | §10.7 的 ①a / ② / ③ 缺 `signs` / `zeros` 直接按 `15` 退出，**不得**手写或猜 | [§10.7](../runbook/4-experiments.md#exp-e7)、[§10.8](../runbook/4-experiments.md#exp-e8) |
| 时间基准 | Pi 时区 `Europe/London`，日志时间等于 UTC，本地是 UTC+8；`$S4DIR` / `$STAMP` 按 `date -u` 生成 | [§2](../runbook/1-prepare.md#sec-2) |
| 产物路径纪律 | 一律显式传 `--out` / `--csv`：默认文件名时间戳只到秒，同一秒内跑两次会静默覆盖 | [§3.8](../runbook/1-prepare.md#sec-3) |
