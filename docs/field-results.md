# 现场验证结果台账

按时间倒序。每行一条结论，数字可在「证据文件」里核对。证据文件都在仓库外的证据目录：

- 2026-11-05 节：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261104/`
- 2026-10-25 及更早：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/`（索引 [evidence/s4-20261025/README.md](file:///Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/README.md)）

下表「证据文件」列给的是**该节所属目录**下的相对路径（各节首段的「条件」一句已写明本节用哪个目录）。逐条命令、判据与退出码表在 [docs/s4-field-smoke.md](s4-field-smoke.md)，本文件只记拿到了什么。

## 2026-11-05（远程会话）· S4 现场窗口（第 2 次）

条件：AlohaMini 2 Pro（`robot_model=alohamini2pro`），18 个电机，Pi 运行时 HEAD `6aee1f81`，Pi 上 lerobot `0.6.1`，远程 SSH 会话（无人在机器人现场），本库 HEAD `b61a61f`；证据目录（仓库外）为 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261104/`，下表「证据文件」列都是这个目录下的相对路径。

### 结论一览

| 结论 | 关键数字 | 条件 | 证据文件 |
| --- | --- | --- | --- |
| 连通与基线指纹齐全，Pi 上 lerobot 版本首次实采 | Pi HEAD `6aee1f81`；lerobot `0.6.1`；指纹 `robot_metadata.schema_version=2`、18 个电机、motors 8 字段与 2026-10-25 零差异（已做两版 JSON `robot_metadata.motors` 子树全量 diff：18 只电机 × 8 字段 = 144 条叶子路径，0 差异；报告 `stdout/4.1-motors-diff-20261105.txt`）；指纹新键 `lerobot_version="0.6.1"`；手册 §4.3 三键实采 `observation_request_window=3`、`watchdog_timeout_ms=1000`、`max_loop_freq_hz=50`，与 `configs/site.yaml` 一致 | SSH 通；`--lerobot-version` 机制首次实战；capture 在 Pi 侧跑，指纹 `host` 字段是 `127.0.0.1`；§4.3 三项取自源码默认值且命令行无覆盖 | `capture/fingerprint-20261105T063600Z.json`、`stdout/4.1b-lerobot-version-20261105T063600Z.log`、`stdout/4.1b-probe-20261105T063600Z.log`、`stdout/4.1-capture-pi-1-20261105T063600Z.cmd`、`configs/site-verified-20261105T063600Z.yaml` |
| 左夹爪 ID7 寄存器只读确认命中判据，固件限位已按标定值生效 | 左臂 ID7 `Min_Position_Limit=2021` / `Max_Position_Limit=2072`；右臂 ID7 对照 `2010` / `3479`；两只 `Torque_Enable=0`、`Lock=1`；两份日志 `EXIT_CODE: 0` | 停 Host 后只读。51 tick 的成因在标定值，`docs/ADR.md` D14「本轮不重标定」维持。本轮 `Torque_Enable=0` 与 2026-10-25 读到的 ID1/2/3/6 的 `Torque_Enable=1` **不可比**：本轮测的是夹爪 ID7，上次测的是臂关节，不是同一批电机的状态变化 | `stdout/4.2-gripper-regs-20261105T063600Z.log`、`stdout/4.2-gripper-regs-right-20261105T063600Z.log` |
| 零运动测量三次各 500 行，跨机 RTT 达成，真实网段与回环同量级，PC 侧 50 Hz 命令节奏有实测支持 | 部署形态（请求 30 Hz、窗口 3）`achieved_rate_hz=30.1`；回环 RTT 形态（请求 1000 Hz、窗口 1）`achieved_rate_hz=49.8`、`net_rtt_estimate_ms` p50 `6.188`；跨机（Mac → Pi，`host=10.34.170.98`）`achieved_rate_hz=48.0`、`net_rtt_estimate_ms` p50 `6.010`、p95 `9.105`、`host_timing_missing=0/500` | 跨机那条首次 exit `9`（`it publishes no latest_request_timing`：隔离环境装的是 vendor wheel，不含该计时面，首跑不写 CSV，产物是 `stdout/5.3-rtt-net-cross-20261105T063600Z.log`，`.exit` 为 `9`）；重跑把 `PYTHONPATH` 指向本地 fork `6aee1f81` 快照 `/tmp/lerobot-vendor-6aee1f81/src` 后 exit 0，CSV 实际名带 `-2` | `rtt/rtt-deploy-20261105T063600Z.csv`、`rtt/rtt-net-20261105T063600Z.csv`、`rtt/rtt-net-cross-20261105T063600Z-2.csv`、`stdout/5.1-rtt-deploy-*`、`stdout/5.2-rtt-net-*`、`stdout/5.3-rtt-net-cross-*`、`stdout/5.3-rtt-net-cross-2-*`、`stdout/5.3b-clone-*`、`stdout/5.3b-verify-src-*` |
| 静止电流基线 18/18 电机全覆盖，读数只有两档 | 18/18 电机 × 100 帧（5 Hz、20 s、`frame_errors=0`）；取值只有 `0` 与 `+6.50 mA`（`+6.5` = 1 个 raw 刻度 × 6.5）；全程恒 0 的 7 只：`arm_left_gripper`、`arm_left_wrist_yaw`、`arm_right_gripper`、`arm_right_wrist_flex`、`arm_right_wrist_roll`、`base_left_wheel`、`base_right_wheel`（`flat0_by_abs_le_0.5mA_count=7`） | 取数路径是 `latest_safety_status["currents_ma"]`，不是 Host 的 `[Currents]` 日志行 | `stdout/6.1c-obs-idle-20261105T063600Z.log`（末尾有 `# per_motor_table` 与 `# summary`）、`.cmd`、`.py` |
| 运动窗口电流量级不足判读门槛，臂总线电流不能判负载 | 腕部 run 的运动窗口 1638 个（帧×电机）样本，最大 `6.5 mA`，`≥20 mA` 样本 0 个；`pan` / `lift` 两次 run 的窗口内最大 `19.5 mA`，取值只有 `0` / `6.5` / `13.0` / `19.5` 四档 | 判读门槛 `20 mA`；`verdict:` 行为「臂总线电流不能判负载」，后续所有依赖电流的判断一律降级为「无电流证据」 | `stdout/6.2b-analysis-20261105T063600Z.txt`、`stdout/7.2-analysis-20261105T063600Z.txt` |
| 达成率三轴只有腕部接近指令，肩部两轴与 2026-10-25 相比一致变差 | 腕部 87.91%（净 `1.7582°` ÷ 指令 `2.0000°`，`error_deg=0.1209`，与 2026-10-25 逐字相同）；`arm_left_shoulder_pan` **4.40%**（净 `0.0879°`）；`arm_left_shoulder_lift` **0.00%**（净 `0°`）；三轴 `settled=False` | 左臂单关节，三次 run 各 2 步、每步 1°；本轮容差 `0.05°` 严于上次 `0.15°`，`note` 全为 `settle budget exhausted`；三者与 2026-10-25 的 87.9% / 8.8% / 13.2% 相比，肩部两轴变差。限定：两轮差值只有 0.5–1.5 tick（亚 LSB 到 1.5 个计数，1 tick = 0.0879°），腕部达成率与残余两轮逐字相同，且本窗口容差 `0.05°` 小于 1 LSB（0.0879°），`settled=False` 是这一算术后果——变差是实测事实，但不能当硬件劣化证据 | `tracking/tracking-wrist-20261105T063600Z.csv`、`tracking/tracking-pan-20261105T063600Z.csv`、`tracking/tracking-lift-20261105T063600Z.csv`、`stdout/7.1-wrist-try2-*`、`stdout/7.1-shoulder-pan-*`、`stdout/7.1-shoulder-lift-*`、`stdout/6.2b-analysis-*`、`stdout/7.2-analysis-*` |
| 停滞不在 PC→Host 值传递段；Host→总线写入段未观测；候选根因含关节扭矩未使能（`alohamini.py:545` `enable_torque()` 被注释、`:520` 先 `disable_torque()`）、目标编码取整、伺服保护锁存、力矩不足、机械，未定性（分析脚本机械判为 ②，判读链见下） | 两轴（`pan`、`lift`）步内 `requested_target == accepted_target` 零例外（`requested!=accepted rows=0`）；`accepted_target` 变化点与主 CSV 的 `cmd_units` 逐位字符相同（`req==cmd True`、`acc==cmd True`）；运动帧 `target_source` 全为 `command`。`requested_targets` / `accepted_targets` 在本版 vendor 上分别是 `_arm_goal_positions` 与 `_arm_sent_positions`（`alohamini.py:987-988`），后者在 `sync_write` 调用之后无回读即 `update`（`:919`），故「零例外」只说明 `_limit_*_by_current` 未钳制值，不构成总线写入证据 | `hold_active` 在 Pi `6aee1f81` 上结构上恒 False：`grep _joint_hold_goal[` 共命中 7 行，全部是定义 / 导出 / 读用法，无赋值写入点——初始化 `:298`、导出 `:335` / `:1018`、get 与 in 判断 `:1109` / `:1121`、下标读 `:1161`、pop `:1164`，故不作判据 | `stdout/7.2-analysis-20261105T063600Z.txt`、`stdout/7.2-joint-hold-write-*`、`stdout/7.2-joint-hold-writes-20261105T063600Z.log`、`stdout/7.2b-host-tracking-*`、`stdout/host-log-20261105T063600Z.log`、`tracking/tracking-pan-20261105T063600Z-safety.csv`、`tracking/tracking-lift-20261105T063600Z-safety.csv` |
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
| D | 关节扭矩未使能 | **未排除（被弱化，不作定论）** | 源码依据：`configure()` 在 `alohamini.py:520` 调 `left_bus.disable_torque()`、`:545` 的 `enable_torque()` 被注释、`disconnect` 默认 `disable_torque_on_disconnect=True`（`config_alohamini.py:68`）；但不是「全包无调用点」，且与「腕部同总线达成 87.91% 且两轮逐字相同」不相容；由 `docs/s4-field-smoke.md` §10.5 的 M1 分支 ① 覆盖 |
| E | 伺服保护锁存 | 未排除 | 无本窗口的寄存器侧观测；`Status` 全 0 只在 2026-10-25 停 Host 后读到 |
| F | 力矩不足 | 无证据（降级为「无电流证据」） | 臂总线电流不能判负载：运动窗口最大 `19.5 mA`，低于 `20 mA`（3 个 raw 刻度）门槛 |
| G | 机械 | 未排除 | 未到现场，无齿隙 / 死区 / 连杆测量 |
| — | 总线写静默失败 | 未排除（零观测） | `sync_write` 无 ACK；`requested_targets` / `accepted_targets` 在本版 vendor 上分别是 `_arm_goal_positions` 与 `_arm_sent_positions`（`alohamini.py:987-988`），后者在 `:919` 于 `sync_write` 调用之后、无回读即 `update`，故不携带总线写入信息 |

弱证据一条：实测组合 `{Torque_Enable=1, Lock=0}` 与 vendor 两条写路径都不相容——`feetech.py:293` 写 `{0, 0}`、`:304` 写 `{1, 1}`，本库 `src/` 不写这两个寄存器；该组合与「`Lock` 写静默失败」相容，但这是**未验证的假说**，不是结论。

未排除者合计：D / E / G + 总线写静默失败；F 无证据。定性动作（teleop oracle → M1 → M2）见 `docs/s4-field-smoke.md` §10.5，均需有人到场。

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
| `tracking/` CSV 列 | 14 列含义见 `docs/s4-field-smoke.md` 第 11 节。要点：`cmd_deg` 是该步的绝对目标角，`baseline_deg` 是该步起点的实测角，`measured_deg` 是该步收尾的实测角 |
| 回环 RTT 的读法 | `net_rtt_est_s` 是扣掉 host 采样、相位、排队与预取等待之后的估计值，不是纯往返时延；`host=127.0.0.1` 表示这是 Pi 本机回环，不能当网络延迟读 |
| 电流的单位、粒度与计数 | `[Currents]` 打印值已是 mA（Host 侧 `int(raw × 6.5)` 截断）：`6` 是 1 个 raw 刻度，`13` 是 2 个刻度。同一份日志里升降轴 `Present_Current` 读到 `71.5` / `221.0` / `936.0` / `2639.0` mA，说明电流读取本身正常；局限在臂总线静止与轻载下只有 0–2 个 raw 刻度。**运动中的量级未验证**，不要拿它反推负载。计数须按整份文件复算：`grep -c '\[Currents\]'` 给 1278、元素 11502；旧版本 `docs/s4-field-smoke.md` 第 6 节按前缀统计留下的「1020 行、9180 个元素」已作废，那组数只到本文件第 1035 行（约前 50 KB），比整份文件少 258 行，按前缀统计会偏小；现该文第 6 节与本文都按整份文件复算，引用时不要混用。判读门槛见 `docs/s4-field-smoke.md` 第 6.2 节 |
| 时间基准 | 证据文件名与日志时间戳都由 `date -u` 生成，是 UTC。本文件正文里的日期（如 `2026-10-25`）也是 UTC 日期。本机时间是 UTC+8，钟点比日志数字大 8 小时：UTC 07:00 对应本地 15:00，UTC 09:33 对应本地 17:33。Pi 的系统时区在证据目录内没有 `timedatectl` 记录 |
