## 11 产物与归档

证据目录按 `AGENTS.md` §6 表格「实测数据目录」一行的路径，落在 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/` 下。第 2 节的变量块按 UTC 日期建子目录，`$S4DIR` 就是它：

```
$S4DIR/                       # = ~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-YYYYMMDD
├── capture/
├── rtt/
├── tracking/
├── configs/
└── stdout/
```

| 目录 | 放什么 |
| --- | --- |
| `capture/` | 第 4.1 节的 `fingerprint-$STAMP.json`、第 4.1c 节的 `fingerprint-postpower-$STAMP.json`（断电后指纹）；失败时这里为空，失败运行的原文在 `stdout/4.1-capture-*` |
| `rtt/` | 第 5 节的三份 CSV：`rtt-deploy-$STAMP.csv`、`rtt-net-$STAMP.csv`、`rtt-net-cross-$STAMP.csv` |
| `tracking/` | 第 7.1 节与第 6.2 节的 `tracking-*.csv`、各自 sidecar `*-safety.csv` 与 report 原文 |
| `configs/` | 第 4.3 节的 `template.yaml` 与 `site.yaml` |
| `stdout/` | 每条命令的三件套 `<步骤号>-$STAMP.cmd`、`.log`、`.exit`；`scope-*` 授权记录；第 6、8 节的记录 |

只有这五个子目录。新的产物种类不改目录结构，按上表归位。

2026-10-25 那次窗口实际取到了什么、数字是多少、每个数字对应哪个文件，已汇总到 [docs/field-results.md](field-results.md)。要查现场结果先看那份台账；本文只管怎么执行。

### rtt 模式 CSV：18 列

一行是一个被接受的观测请求。

| 列 | 含义 |
| --- | --- |
| `sample_index` | 行序号 |
| `token` | 该观测请求的 token |
| `observation_sequence` | 客户端的观测序号，只有增长才计一行 |
| `sent_monotonic_s` | 请求发出时刻，客户端单调时钟 |
| `received_monotonic_s` | 响应到达时刻，客户端单调时钟 |
| `age_s` | 请求年龄（端到端，不是纯网络往返） |
| `host_sample_duration_s` | 宿主状态采样耗时 |
| `phase_bound_s` | 请求在宿主主循环里等待相位的上界 |
| `phase_mean_s` | 同上，均值 |
| `queue_bound_s` | 同窗口其他在飞请求排队项的上界 |
| `queue_mean_s` | 同上，均值 |
| `prefetch_bound_s` | 窗口预取等待的上界 |
| `prefetch_mean_s` | 同上，均值 |
| `host_timing_missing` | 该帧取不到宿主采样耗时时记 `True`，否则记 `False` |
| `net_rtt_est_s` | 网络往返点估计（已减去上面各项） |
| `net_rtt_low_s` | 估计下界 |
| `net_rtt_high_s` | 估计上界 |
| `state_sample_unix_ns` | 该样本的宿主侧 unix 纳秒时间戳；缺失时留空，不填 0 |

### tracking 模式 CSV：14 列

一行是一个步进。

| 列 | 含义 |
| --- | --- |
| `step_index` | 步序号，从 0 起 |
| `t_send_monotonic_s` | 本步首条命令的发出时刻 |
| `received_monotonic_s` | 越过 90% 那一帧的到达时刻 |
| `cmd_deg` | 目标角（度） |
| `cmd_units` | 目标值，线上归一化单位 |
| `deg_per_unit` | 该关节 1 个归一化单位等于多少度 |
| `cmd_echo_deg` | 客户端回显的目标角（度），客户端截断时与 `cmd_deg` 不同 |
| `baseline_deg` | 本步起始实测角（度） |
| `measured_deg` | 本步结束实测角（度） |
| `error_deg` | `cmd_deg - measured_deg` |
| `rise_90_time_s` | 90% 上升时间；容差设置不当会为空 |
| `overshoot_deg` | 超调量（度） |
| `settled` | 是否在容差内稳住 |
| `note` | `ok`、`settle budget exhausted`、`interrupted` 或 `aborted: <原因>` |

### tracking 模式 sidecar CSV：safety 快照（9 列）

`--mode tracking` 除主 CSV 外还会写第二份文件：safety 快照 sidecar（贴着主文件写的副产物）。主 CSV 仍是 14 列、一行一步；sidecar 是长表，**每帧每电机一行**。安全面为何单独成册而不进 14 列，见 `docs/ADR.md` D18。

**文件名规则**（`safety_csv_path`，`benchmarks/alohamini_io/measure_latency.py:1289`）：与主 CSV 同目录，主文件名去掉尾部 `.csv` 再加 `-safety.csv`。

| 主 CSV | 对应 sidecar |
| --- | --- |
| `tracking-20261025T000000Z.csv` | `tracking-20261025T000000Z-safety.csv` |
| `a.b.csv` | `a.b-safety.csv`（只去尾部 `.csv`，名字里的点保留） |
| `notes`（名字不以 `.csv` 结尾） | `notes-safety.csv` |

**产生条件**：只在 `--mode tracking` 产生；`--mode rtt` 不产生（该模式不读 `latest_safety_status`）。

**9 列**（`SAFETY_CSV_FIELDS`，`measure_latency.py:626`，顺序固定）：

| 列 | 含义 |
| --- | --- |
| `step_index` | 该帧落在第几步，从 0 起，与主 CSV 的 `step_index` 同义 |
| `sample_sequence` | 该帧的 `observation_sequence`；**同一 `sample_sequence` 的行属同一时刻** |
| `t_monotonic_s` | 该帧的时间戳（客户端单调时钟，秒）。优先取到达时刻 `latest_request_timing["received_monotonic_s"]`；取不到则回落到本次观测的轮询时钟读数。**Pi 上厂商客户端不发布 `latest_request_timing`（只有 `latest_host_timing`，见 `docs/ADR.md` D12），所以现场这一列是轮询时刻，不是到达时刻** |
| `motor` | 电机名（裸名，如 `arm_left_shoulder_pan`）。帧里读不到电机表时这一格为空 |
| `current_ma` | 该电机电流，单位 mA（Host 侧 `raw × 6.5`，`alohamini_host.py:300-301`）。sidecar 没有单位列，单位固定 mA |
| `requested_target` | 厂商帧 `requested_targets` 的该电机项，即 `_arm_goal_positions`（源码阅读：`alohamini.py:987`，该表在 `:895-896` 写入）。写入点在 `ensure_safe_goal_position`（`alohamini.py:872-896`）**之后**、电流限幅（`_limit_gripper_goal_by_current` / `_limit_joint_goal_by_current`，`:897-905`）**之前**。单位是 action 键的归一化单位，不是度。被电流限幅钳过时与本行的 `accepted_target` 不相等；被 `max_relative_target` 相对限幅改过时两列都是改后值、可能仍相等 |
| `accepted_target` | Host 实际采纳的目标，单位同上（`_arm_sent_positions`，`alohamini.py:988`） |
| `hold_active` | 该电机是否在 Host 的 hold 表 `joint_holds` 里。**盘上写法是 Python 字面量 `True` / `False`**（`write_csv` 把布尔原样交给 `csv.writer`，落盘是 `str(True)` 的文本），不是 `1`/`0`，也不是空/非空；`joint_holds` 不可读（不是容器）时这一格为空。下游按字符串解析时按 `True` / `False` 这两个词判，不要用真值判断（`"False"` 是非空字符串，按真值判会读反）。钉住的 vendor（`ec65318f`）上恒 `False`，是否可用先按第 7.2 节核实 |
| `target_source` | **整帧属性**：该帧的目标由哪一层给出，取值 `none` / `command` / `watchdog` / `protection`（`alohamini_host.py` 行 261、330、357、373、392）。同一 `sample_sequence` 的每一行同值；缺失或不在四值之内（包括空串、未知字符串）时这一格为空——空格是「这帧没说」，`none` 是「这帧说没有层产出目标」，两者不混读 |

**长表语义**：一行 = 一个（帧, 电机）。同一 `sample_sequence` 的若干行属同一时刻，组内 `step_index` 与 `t_monotonic_s` 相同。行按帧到达顺序排列，组内按帧自己列电机的顺序。一帧 18 个电机就是 18 行。

**只记新帧**：sidecar 复用 tracking 既有的陈旧检测——只有 `observation_sequence` 增长的那次观测才记行，缓存重复帧（陈旧帧）不重复记；`_run_tracking_mode` 在正式跑之前的那次 preflight 观测（读序号取基线）也不计行。所以「帧数」= 主循环里真正增长的帧数。

**与主 CSV 的 join 盲区**：帧在 `_observe` 里记，主 CSV 的行却要到步首帧读完、baseline 判定通过、命令发出之前才建。所以某步在**步首**就因 baseline 缺失 / 非有限或反馈陈旧而中止时，主 CSV 里**没有**该 `step_index` 的行，sidecar 里**却有**该步的帧。按 `step_index` join 会把这些帧丢掉；按 `sample_sequence` 对齐更稳。对齐不上主 CSV 行的帧，读法是「这一步没走成，但它经过了几个帧」。

**空单元格纪律**：`current_ma` / `requested_target` / `accepted_target` 只在能转成有限数值时才写数值，否则留空（CSV 里是空单元格）。`bool` 不算数值（`True` 不能写成 `1 mA`）。缺值**不补 0**：`0 mA` 读起来像「测到的零电流」、`0` 目标读起来像「Host 采纳过 0」，两者都是伪观测。

**降级：能力缺失与内容不完整**（两个**降级**分支互斥，所以降级最多产生一条 stderr 警告；退出码不变；sidecar 照写）：

| 分支 | 触发条件 | stderr | 后果 |
| --- | --- | --- | --- |
| A 能力缺失 | 客户端根本没有 `latest_safety_status` 属性 | 一条，点名缺的属性，并写明 sidecar 路径与哪些列会空 | sidecar 每帧一行，`motor` 与三个数值列全空，只剩坐标列有值 |
| B 内容不完整 | 属性存在，但本次运行**至少一帧**没有可读的 `currents_ma` 电机表 | 一条，点名 `latest_safety_status`，并带可机读计数 `frames=<int>, frames_with_motor_table=<int>` | 这些帧的 `motor` 与三个数值列全空 |
| 全健康 | 属性存在且每一帧都带电机表 | 不告警 | 正常 |

计数取自 sidecar 自己的行（`_safety_frame_counts`），警告与文件不会互相矛盾：`frames` 是本次记录的帧数，`frames_with_motor_table` 是其中至少带一个电机名的帧数，差值是空 `motor` 帧数。判据是**整轮计数**而不是「开局探一次」：只探一次的话，4 帧里坏 3 帧会静默通过。

sidecar 写盘失败（`OSError`）也只打一条自己的警告，**退出码不变**：主 CSV 才是主产物，判据顺序（hold 失败 > 写失败 > 中止）只看主 CSV。**这几件事不要混**：「最多一条」只约束降级本身（两个分支互斥）；写盘失败是另一件事实。一次运行**合法可出现三条 stderr**：① safety 面降级警告（分支 A 或 B，最多一条）② timing 面回落点名（sidecar 的 `t_monotonic_s` 或 90% 交叉点的到达时刻回落到轮询时钟，整轮一条；Pi 上 vendor 不发布 `latest_request_timing`，所以只要这轮至少记录到一帧就会打）③ sidecar 写盘失败警告。回落的点名讲的是 `latest_request_timing`、不含 `latest_safety_status`，落点也是独立的一条 `if`，与 safety 降级那一对互斥分支互不抑制。回落点名是「这一列是什么」的说明，**不是故障**，不要按故障处理。

**怎么用 sidecar 判肩部**：见第 7.2 节，那里给的是现场执行顺序与三类结论的判据。

## 13 验收标准

每条一个判据，不写「看起来正常」。

- [ ] 第 1 节 P0–P5 逐条有结论；P0 有 `capture` 产物为证（未勾：P1 / P4 / P5 因**无人在机器人现场**未判；P0 已关闭，P2 / P3 已判）
- [x] `capture` exit 0，JSON 11 个顶层键齐全，`robot_metadata.schema_version` 为 `2`，且逐个电机的 8 字段覆盖情况有记录（2026-11-05 远程会话已满足，证据：`capture/fingerprint-20261105T063600Z.json`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] JSON 里 `lerobot_version` 非 `null`，且与第 4.1b-lerobot-version 节的日志里的 `$PI_LEROBOT_VERSION` 逐字一致（2026-11-05 远程会话已满足，证据：`capture/fingerprint-20261105T063600Z.json`、`stdout/4.1b-lerobot-version-20261105T063600Z.log`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] 第 4.1b 节探测输出在盘；Gate F 按「未触发」记录，理由与原始输出一并落盘，没有为凑记录改动任何文件（2026-11-05 远程会话已满足，证据：`stdout/4.1b-probe-20261105T063600Z.log`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] 第 4.2 节 ID7 的 `Min_Position_Limit` / `Max_Position_Limit` 原文在盘，并写明是否等于 `2021` / `2072`；`Torque_Enable` 只记录未改动（2026-11-05 远程会话已满足，证据：`stdout/4.2-gripper-regs-20261105T063600Z.log`、`stdout/4.2-gripper-regs-right-20261105T063600Z.log`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] 第 4.3 节两份 YAML 落盘，字段名与 `config/overlay.py` 一致，`load_settings(check_devices=False)` exit 0（2026-11-05 远程会话已满足，证据：`configs/template.yaml`、`configs/site.yaml`、`stdout/4.3e-load-check-20261105T063600Z.exit`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] 第 5 节三条 rtt 各 exit 0、各 500 样本、CSV 各有数据行；`achieved_rate_hz` 与请求率的差距有记录；跨机那条给出 `net_rtt_estimate_s` p50 并与 `6.176 ms` 并列（2026-11-05 远程会话已满足跨机那条，证据：`rtt/rtt-net-cross-20261105T063600Z-2.csv`、`stdout/5.3-rtt-net-cross-2-*`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] 第 6 节两段电流读数在盘（主路径 `latest_safety_status["currents_ma"]`，范式见 `stdout/6.1c-obs-idle-$STAMP.py`；`[Currents]` 日志路径当前不可依赖，见 §6.1），并写出结论：臂总线电流可判读，或不能判负载（2026-11-05 远程会话已满足，证据：`stdout/6.2b-analysis-20261105T063600Z.txt`、`stdout/7.2-analysis-20261105T063600Z.txt`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] 第 7.1 节若执行：每步的 `error_deg`、`overshoot_deg`、`settled` 齐全；`rise_90_time_s` 为空时写明原因；收尾 `hold_issued` 在报告里（2026-11-06 到场窗口已满足：10 步的 `error_deg` / `overshoot_deg` / `settled` 与 `hold_issued=yes` 在 `stdout/10.5-m1r3-pan-20261106T074049Z.log:5`、`stdout/10.5-m1r3-pan-20261106T074049Z.log:11`，逐行表在 `tracking/m1r3-pan-20261106T074620Z.csv`；`rise_90_time_s` 为空的原因「残余 > 0.1 × 命令量、没有帧越过 90% 线」由 `stdout/slow-vs-stall-judge-20261106T090635Z.txt` 第 5.4 节逐轴核成立（腕收尾残余 0.1209° 对 0.100°、`pan` 0.8242° 对 0.100°），数字见 `docs/field-results.md` 2026-11-06 节。探针本身未改、以及「收敛之后会不会有 90% 交叉」仍未观测，这两件记在 `TODO.md` 的 `rise_90_time_s` 工程债行）
- [x] 第 7.2 节：`target_source` 分布、`requested_target` 与 `accepted_target` 的比较、`current_ma` 走势三项有结论，且落到三类之一；`_joint_hold_goal` 有无写入点的 grep 原文在盘（2026-11-05 远程会话已满足，证据：`stdout/7.2-analysis-20261105T063600Z.txt`、`stdout/7.2-joint-hold-write-*`，数字见 `docs/field-results.md` 2026-11-05 节）
- [x] sidecar 的四个数字有记录：每帧行数、`motor` 空行占比、`current_ma` 分布、`hold_active` 全列取值（safety sidecar 此前只有离线测试覆盖，真机首跑按这四项记）（2026-11-05 远程会话已满足，证据：`stdout/7.1-wrist-sidecar-try2-20261105T063600Z.log`，四项数字尚未在台账逐项登记；同一轮主表见已登记的 `stdout/7.1-wrist-try2-*`）
- [ ] 第 8 节：断电与上电有记录，Host 停启有 PID 与两条命令原文，停机与重启时间都是 UTC（未勾：Host 停启有记录且时刻是 UTC（PID `1866`，`stdout/6.1a-host-pid-20261106T070942Z.log`；停机 `2026-11-06T09:08:01Z` + `kill_int_exit=0`，`stdout/4.1-host-int-20261106T090635Z.log`），但「断电与上电有记录」只有用户口述的换电（`stdout/scope-append-20261106T070942Z.md` ①），无仪器证据；断电前「把手臂放到低位」也没有现场记录）
- [ ] `$S4DIR/stdout/` 里每步都有 `.cmd`、`.log`、`.exit` 三件套（未勾：缺 `stdout/6.1a-host-start-20261105T063600Z.exit`；6.1a 其余四个子步骤 `currents-peek` / `host-log-1` / `host-procs-1` / `wait-ports-1` 三件套齐全）
- [x] 第 4.1c 节：`fingerprint-postpower-*.json` 在盘，`CALIB_FIELD_DIFFS: 0`，位置组 8 只 `arm_*` 超 3 tick，已判**不可归因**（基线跨过试动，见 `docs/field-results.md` 2026-11-06 节），基线已换
- [x] 第 4.2 节「全 18 电机静态体检」：左 11 / 右 7 两份日志都 `EXIT_CODE: 0`，18 个 `Model_Number` 与 `MODEL_NUMBER_TABLE` 相符，`Present_Voltage` 的 min / mean / max 有记录（已执行）
- [x] 第 4.4 节：`examples/robot_dry_run.py` exit 0，stdout 有 `send_action 调用次数: 0 次`（已执行）
- [x] 第 4.5 节：`stash list` / `reflog` / `status --porcelain` 三份原文在盘，并落到「记录哈希 / 记录 reflog / 确认丢失」三支之一（2026-11-06 远程会话已满足，落「记录 reflog」支：`stash list` 与 `status --porcelain` 两份均 0 字节，`reflog --date=iso` 两行——`2026-10-25 09:17:30 checkout: moving from master to p1a-merged`、`09:22:01` fast-forward 到 `6aee1f81`；证据：`stdout/4.5a-stash-20261106T064822Z.log`、`stdout/4.5b-reflog-20261106T064822Z.log`、`stdout/4.5c-status-20261106T064822Z.log`，数字见 `docs/field-results.md` 2026-11-06 节）
- [x] 第 7.1b 节：模型名故意写错的命令 exit 6 或 3 有 stderr 原文、无 CSV、命令行回显三件套，并写明落到哪一支（已执行（落 Gate B））
- [ ] 第 10.5 节：teleop oracle 与 M1 / M2 各有一份读数记录，且每条结论都落到 §10.5 的分支之一（未勾：M1 / M2 在 2026-11-06 已有读数并落了分支——M1（`stdout/10.5-m1-verdict-20261106T072606Z.md`）字面命中分支③，其成因判为**疑似**看门狗改写（该轮无 CSV、无 sidecar、无 Host 日志原件，成因未定，见 `docs/field-results.md` 2026-11-06 节「未取到 / 未验证」）；M2 落在 §10.5 M2 判据表新增的「介于两档之间」档（`stdout/10.5-m2r1-verdict-20261106T075623Z.md`）；**teleop oracle 本轮没跑**，只有用户口述 vendor 遥操作能带动，无仪器证据，故三项不同时满足）

打勾依据是 2026-11-05 远程会话与 2026-11-06 两段会话（远程会话 + 有人到场的到场窗口）的产物；勾选不等于该项已收口到验收基线，V 编号的实机证据仍按 `docs/acceptance-baseline.md` 填。

判据里两条与数据形态有关的注意：

- 2026-10-25 五份 tracking CSV 的 `received_monotonic_s` 与 `rise_90_time_s` 全空，同因：没有帧记到 90% 交点。**2026-11-05 用 `--settle-tolerance-deg 0.05`（小于 0.1°）跑三轴，`rise_90_ms count=0 p50=nan` 仍然为空，「容差取小就能补上这一列」的假设不成立**；三份 CSV 的 `rise_90_time_s` 仍全空。报告自带的说明是：要记一次 90% 交叉，需残余 ≤ `0.1 × delta` 且发生在 settle 循环结束之前，实测没发生。2026-11-06 的 `slow-vs-stall` 判读把成因核到「残余 > 0.1 × 命令量、无帧越过 90% 线」（`stdout/slow-vs-stall-judge-20261106T090635Z.txt` 第 5.4 节，逐轴两行「成立」）；探针未改、以及「收敛之后会不会有 90% 交叉」仍未观测，这两件记在 `TODO.md` 的 `rise_90_time_s` 工程债行。
- Mode B 只在 2026-10-25 跑过五次，都是左臂单关节、每步 1°。右臂、多关节联动、更大的单步行程都未跑过。

