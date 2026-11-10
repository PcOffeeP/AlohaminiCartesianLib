# 产物、归档与 CSV 列义（原 §11）

本文件管什么：一次运行的产物落在哪个子目录、主 CSV 与 sidecar 各列是什么意思、证据根怎么按场次分目录、台账去哪查。

<a id="sec-11"></a>
## 11 产物与归档（原 §11）

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

<a id="csv-rtt-18"></a>
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

<a id="csv-tracking-14"></a>
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

<a id="sidecar-csv"></a>
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


## 证据根（原 README §2.3）

原件都在 `~/pi-workspace/alohamini-cartesian-teleop/evidence/`，目录内 `README.md` 写命名约定。按窗口分目录：

| 目录 | 对应场次 |
| --- | --- |
| `s4-20261025/` | 2026-10-25 及更早 |
| `s4-20261104/` | 2026-11-05 节（会话 STAMP 从 `20261104T023000Z` 起） |
| `s4-20261106/` | 2026-11-06 节 |
| `s4-20261107/` | 2026-11-08 节第一段（目录名按授权所取，目录内 STAMP 与全部时刻是 11-08 UTC） |
| `s4-20261108/` | 2026-11-08 节第二段（路 A 复核） |

## 台账（原 README §2.4）

`docs/field-results.md`：现场数字与结论，按时间倒序。每个日期节给「结论 / 关键数字 / 条件 / 证据文件」四列，「证据文件」列是该节所属证据目录下的相对路径。
