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
| 达成率三轴只有腕部接近指令，肩部两轴与 2026-10-25 相比一致变差 | 腕部 87.91%（净 `1.7582°` ÷ 指令 `2.0000°`，`error_deg=0.1209`，与 2026-10-25 逐字相同）；`arm_left_shoulder_pan` **4.40%**（净 `0.0879°`）；`arm_left_shoulder_lift` **0.00%**（净 `0°`）；三轴 `settled=False` | 左臂单关节，三次 run 各 2 步、每步 1°；本轮容差 `0.05°` 严于上次 `0.15°`，`note` 全为 `settle budget exhausted`；三者与 2026-10-25 的 87.9% / 8.8% / 13.2% 相比，肩部两轴变差 | `tracking/tracking-wrist-20261105T063600Z.csv`、`tracking/tracking-pan-20261105T063600Z.csv`、`tracking/tracking-lift-20261105T063600Z.csv`、`stdout/7.1-wrist-try2-*`、`stdout/7.1-shoulder-pan-*`、`stdout/7.1-shoulder-lift-*`、`stdout/6.2b-analysis-*`、`stdout/7.2-analysis-*` |
| 目标已被 Host 接受，停滞发生在伺服 / 机械侧（三条判据的实质结论是 ①，分析脚本机械判为 ②，判读链见下） | 两轴（`pan`、`lift`）步内 `requested_target == accepted_target` 零例外（`requested!=accepted rows=0`）；`accepted_target` 变化点与主 CSV 的 `cmd_units` 逐位字符相同（`req==cmd True`、`acc==cmd True`）；运动帧 `target_source` 全为 `command` | `hold_active` 在 Pi `6aee1f81` 上结构上恒 False：`grep _joint_hold_goal[` 共命中 7 行，全部是定义 / 导出 / 读用法，无赋值写入点——初始化 `:298`、导出 `:335` / `:1018`、get 与 in 判断 `:1109` / `:1121`、下标读 `:1161`、pop `:1164`，故不作判据 | `stdout/7.2-analysis-20261105T063600Z.txt`、`stdout/7.2-joint-hold-write-*`、`stdout/7.2-joint-hold-writes-20261105T063600Z.log`、`stdout/7.2b-host-tracking-*`、`stdout/host-log-20261105T063600Z.log`、`tracking/tracking-pan-20261105T063600Z-safety.csv`、`tracking/tracking-lift-20261105T063600Z-safety.csv` |
| 日志里没有 `[Currents]` 与 `[HOST TRACKING]` 的原因是厂商日志级别，不是代码没走到 | 整份 Host 日志 `[Currents]` 行 `lines 0` / `elements 0`；`[HOST TRACKING]` 计数 `0`；厂商 client 里 `logging.basicConfig(...)` 的 `level=logging.INFO` 那一行被注释掉（`lerobot/robots/alohamini/alohamini_client.py:39`），Host 的 root logger 停在 WARNING | 只读核对源码；未拿到改 vendor 的授权，改 vendor 无解；已有一条不依赖它的取数路径（见「静止电流基线」一行） | `stdout/6.1d-currents-src-*`、`stdout/6.1d-host-loglevel-*`、`stdout/6.1b-currents-dist-20261105T063600Z.log`、`stdout/7.2b-host-tracking-count-20261105T063600Z.log` |
| 手册 §13 的一条补救假设被推翻 | 三轴 `rise_90_ms count=0 p50=nan`，三份 tracking CSV 的 `rise_90_time_s` 列全空；容差取到 `0.05°`（小于 `0.1°`）仍然空 | 报告自带说明：要 90% 交叉需残余 ≤ `0.1 × delta` 且发生在 settle 循环结束之前，实际没发生；原因未定位（见「未取到 / 未验证」） | 三次 `stdout/7.1-*.log` 的 `rise_90_ms count=0 p50=nan` 行与 `note:` 行、三份 `tracking/*.csv` |
| 收尾 Host 已停，机器人断电未执行 | `kill 2096`、`kill_exit=0`、停机时刻 `2026-11-05T07:08:54Z`；排除自匹配后 `pgrep` 无输出、`ss_grep_exit=1`（5555 / 5556 无监听） | 无远程可控电源通道；手册 §8.1 要求断电前先把手臂放到低位，机器人仍带电、位置环通电，差人工一步 | `stdout/8.2a-host-stop-20261105T063600Z.{cmd,log}`、`stdout/8.2b-host-verify-*`、`stdout/8.3c-power-state-*`、`stdout/8.1-host-stop-20261105T063600Z.md`、`stdout/scope-20261104T023000Z.md` 的「收尾记录」段 |

§7.2 判读链（`pan` / `lift` 两轮）：

| 步骤 | 内容 |
| --- | --- |
| 脚本机械判为 ② | 规则 2 要求「存在 `target_source` 为 `watchdog` / `protection` 的帧」，且这些帧早于第一条命令帧。两轴各命中 1 帧（`sample_sequence=2`），脚本输出 `VERDICT: ② 目标被底层接管` |
| 排除理由 | 该帧是第一帧，早于第一条命令帧（`sample_sequence=3`），早于任何命令发出，不属运动阶段，已裁决排除；腕部那一轮同一位置的前导帧是 `none` 而不是 `watchdog` |
| 实质结论 ① | 两轴步内 `requested!=accepted rows=0`，`accepted_target` 变化点与主 CSV `cmd_units` 逐位相同，运动帧 `target_source` 全为 `command`：目标已被 Host 接受，停滞发生在伺服 / 机械侧 |

### 未取到 / 未验证

| 项目 | 状态 |
| --- | --- |
| 三轴一致变差（87.9 → 87.91 持平、8.8 → 4.40、13.2 → 0.00）的原因 | 未定位。电压、温度、机械三项都没有测量 |
| Host 日志 3 行 `WARNING:root:Host command watchdog expired; stopping motion` 与三次 run 的一一对应 | 假说。计数与三次 run 吻合（3 行对 3 次，客户端断开后的超时），未验证 |
| 7 只恒 0 mA 电机（`arm_left_gripper`、`arm_left_wrist_yaw`、`arm_right_gripper`、`arm_right_wrist_flex`、`arm_right_wrist_roll`、`base_left_wheel`、`base_right_wheel`）是否正常 | 未验证。这 7 只全程读数恒 0，是否等于「没有电流」未验证；本窗口已判定电流不能当负载证据 |
| 机器人断电 | 未执行。待人工到现场，手册 §8.1 要求断电前先把手臂放到低位 |
| `rise_90_time_s` 恒空的原因 | 未定位。容差已取到 `0.05°`，手册原假设（容差 < 0.1° 可补上这一列）被推翻 |
| 右臂试动数据 | 未取。`tracking/` 六份文件（三份 tracking + 三份 safety）全为 `arm_left_*` |
| 相机帧数据 | 未取。`capture/` 内无图像文件，`robot_metadata.cameras` 只有 `forward` / `wrist_right` 两项（口径同 2026-10-25 节，仍是未取） |

### 读法

| 口径 | 怎么算 |
| --- | --- |
| STAMP 双标签映射 | 本窗口目录有两套时间戳。`20261104T023000Z` 是第 1 步按会话初始化时写下的**约定值**，只出现在 `stdout/0.9-probe-*`、`stdout/3.*`、`stdout/scope-20261104T023000Z.md` 这一批第 1 步产物上；`20261105T063600Z` 是从第 2 步起的**实测值**（`date -u` 实测 UTC 为 `2026-11-05T06:32Z`，与约定值相差约 28 小时），绝大多数产物用它。两者的映射与顺序偏离记在 `stdout/scope-20261104T023000Z.md` 的「执行中补记」段，读文件名时不要当成两次会话 |
| 达成率 | 沿用 2026-10-25 节同一口径（净行程 ÷ 指令行程），本文不重复推导 |
| 电流 | 落盘值已是 mA（`raw × 6.5` 截断）：`6.5` = 1 个 raw 刻度，`20 mA` = 3 个刻度即判读门槛。本窗口运动窗口最大 `19.5 mA`，仍不足 3 个刻度，所以「不能判负载」。早前按 `[Currents]` 日志统计的口径在本窗口用不上：该日志一行都没打印（见「结论一览」的机理一行） |
| `target_source` 前导帧 | 每轮第一帧出现在第一条命令帧之前，那一帧的 `target_source`（`none` 或 `watchdog`）不属运动阶段，判三类结论时要排除 |
| `settled` 跨轮不可比 | 2026-10-25 容差 `0.15°`、本窗口 `0.05°`，同一关节 `settled=False` 的成因不同，比 `settled` 要同时比容差 |
| 执行顺序 | 手册 §4 原假定 Host 常驻；本窗口 Host 未运行，顺序改为 §4.2 →（起 Host）→ §4.1b → §4.1 → §4.3 → §5 → §6 → §7 → §8，全程只起停 Host 一次（出处：`stdout/scope-20261104T023000Z.md` 的「执行中补记」） |
| 时间基准 | 全部 UTC（`date -u`）；本机时为 UTC+8，钟点比日志数字大 8 小时 |

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

现场只读确认步骤见 `docs/s4-field-smoke.md` 第 4.2 节「左夹爪 ID7 只读确认」。

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
| 1 tick | 0.0879°。腕部每步约 10 tick，肩部每步约 1 tick |
| `tracking/` CSV 列 | 14 列含义见 `docs/s4-field-smoke.md` 第 11 节。要点：`cmd_deg` 是该步的绝对目标角，`baseline_deg` 是该步起点的实测角，`measured_deg` 是该步收尾的实测角 |
| 回环 RTT 的读法 | `net_rtt_est_s` 是扣掉 host 采样、相位、排队与预取等待之后的估计值，不是纯往返时延；`host=127.0.0.1` 表示这是 Pi 本机回环，不能当网络延迟读 |
| 电流的单位、粒度与计数 | `[Currents]` 打印值已是 mA（Host 侧 `int(raw × 6.5)` 截断）：`6` 是 1 个 raw 刻度，`13` 是 2 个刻度。同一份日志里升降轴 `Present_Current` 读到 `71.5` / `221.0` / `936.0` / `2639.0` mA，说明电流读取本身正常；局限在臂总线静止与轻载下只有 0–2 个 raw 刻度。**运动中的量级未验证**，不要拿它反推负载。计数须按整份文件复算：`grep -c '\[Currents\]'` 给 1278、元素 11502；旧版本 `docs/s4-field-smoke.md` 第 6 节按前缀统计留下的「1020 行、9180 个元素」已作废，那组数只到本文件第 1035 行（约前 50 KB），比整份文件少 258 行，按前缀统计会偏小；现该文第 6 节与本文都按整份文件复算，引用时不要混用。判读门槛见 `docs/s4-field-smoke.md` 第 6.2 节 |
| 时间基准 | 证据文件名与日志时间戳都由 `date -u` 生成，是 UTC。本文件正文里的日期（如 `2026-10-25`）也是 UTC 日期。本机时间是 UTC+8，钟点比日志数字大 8 小时：UTC 07:00 对应本地 15:00，UTC 09:33 对应本地 17:33。Pi 的系统时区在证据目录内没有 `timedatectl` 记录 |
