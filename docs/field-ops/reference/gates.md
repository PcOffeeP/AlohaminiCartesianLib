# 运行时门禁与退出码（原 §10、§9 门禁相关行、§7.1 的 Gate A–F 表）

本文件管什么：一次测量能不能发命令的那六道运行时门禁（Gate A–F，发命令前的检查）各判什么、不通过时走哪条路，以及退出码怎么读——每个码什么意思、`2` 与 `9` 的全部触发点落在哪。拿到非 0 码先查这里的退出码表，再按 [troubleshooting.md](troubleshooting.md#sec-12) 处置。

<a id="sec-10"></a>
<a id="exit-codes"></a>
## 10 退出码对照表（原 §10）

**一码多义**：同一个码在不同脚本里含义不同，读码先看是哪个脚本。已知四例：`6` 在 `capture` 里表示宿主 metadata 不可用、主动不写盘，在 `measure` 里表示运动门禁拒绝、零命令；`1` 在 `calib_health` 里是 `CALIB_NEW_UNREGISTERED`（**非阻断**，新代际未登记），在 `calib_register` 里是结构校验不合格（阻断，不写盘），在 `calib_fetch` 里是「有 `IMPLAUSIBLE` 且未声明理由」（提醒、不阻断）；`3` 在 `calib_health` 里是 `CALIB_SUSPECT`（可疑，需人核对），在 `calib_register` 里是输入文件不可解析；`4` 在 `calib_health` 里是 `CALIB_CORRUPT`（文件本身坏了），在 `calib_fetch` 里是「**不可继续**」（遥操数据集 / 结构自洽失败 / 接受名单坏 / 远端读不到 / ssh 超时），两个 `4` 不是同一件事；`calib_health` 传了 `--session-anchor` 时还有第五个码义：`CALIB_SESSION_DRIFT`（锚点代际与文件不符）也是 `4`。

下表里 `calib_health` = `tools/calib_health.py`，`calib_register` = `tools/calib_register.py`，`calib_fetch` = `tools/calib_fetch.py`。三个工具的四态与错误分类的完整口径分别写在各自脚本的模块 docstring 里。

| 脚本 | 码 | 含义 |
| --- | --- | --- |
| capture / measure | 2 | 参数解析失败，一律 `2`，不经过任何 `EXIT_*` 常量；触发点全部列在表下「退出码 2 的全部触发点」小节 |
| capture | 0 | payload 已写盘 |
| capture | 3 | 建客户端、`connect()` 或观测传输失败 |
| capture | 4 | `--timeout-s` 到期仍不够 `--samples` |
| capture | 5 | 写盘失败（`OSError`、`bytes`、`NaN`/`Inf`） |
| capture | 6 | 宿主 metadata 不可用：为空、`schema_version` 不等于 2、或 `motors` 非空映射不成立 |
| measure | 0 | 跑完且 CSV 已写 |
| measure | 1 | 有证据在盘上的提前停止（命令被拒 / 超时 / 反馈失联 / Ctrl-C） |
| measure | 3 | 连接失败 |
| measure | 5 | CSV 写不出 |
| measure | 6 | 代码 Gate A–F 拒绝 |
| measure | 7 | rtt 模式样本不足 |
| measure | 8 | 收尾 hold 被拒，停命令没到 |
| measure | 9 | 客户端不发布 rtt 模式所测量的能力（`latest_request_timing`），或发布了但内容填不满。三种形态都返回 `9`、都不写 CSV，形态与触发点见下表 |
| calib_health | 0 | `CALIB_KNOWN`：这个 `calib_id` 已登记，照常用 |
| calib_health | 1 | `CALIB_NEW_UNREGISTERED`：结构通过但未登记，**非阻断**，按提示跑 `calib_register.py` 登记 |
| calib_health | 2 | 参数错误（既没给 `--calib-file` 也没给 `--fingerprint`），或 `--registry` 指向的登记表不可读（四态之一都给不出来） |
| calib_health | 3 | `CALIB_SUSPECT`：能解析但不像正常标定（结构校验不过；或与已登记最新代际比是 `BROAD` / `REGION_INCONSISTENT`，**或 `id` / `drive_mode` 跨代际变动**），需人核对 |
| calib_health | 4 | `CALIB_CORRUPT`：文件不存在、JSON 不可解析、顶层形状认不出、缺字段、值为 `null`、`range_min >= range_max`。**例外**：传了 `--session-anchor` 且锚点与文件 `calib_id` 不符时，同一个 `4` 是第五态 `CALIB_SESSION_DRIFT`，stdout 配合 `SESSION_ANCHOR: MISMATCH <anchor> -> <current>`（含义见下节） |
| calib_register | 0 | 成功（含默认 dry-run，含「该代际已登记，未改动」） |
| calib_register | 1 | 结构校验不合格（电机数、缺字段、取值非法），不写盘 |
| calib_register | 2 | 参数错误（既没给 `--from-file` 也没给 `--from-fingerprint`） |
| calib_register | 3 | 输入文件不可解析（读不了、JSON 坏、认不出的顶层形状） |
| calib_register | 4 | 登记表不可读或不可写 |
| calib_fetch | 0 | 通过：包络汇总不是 `IMPLAUSIBLE` / `WRONG_DATASET`，或已用 `--allow-implausible "理由"` 留痕；含 `--dry-run`（不写盘） |
| calib_fetch | 1 | 有 `IMPLAUSIBLE` 且未声明理由：**提醒、不阻断**；给了 `--out` 才落盘（在门禁之后） |
| calib_fetch | 2 | 用法错误（`--from-file` 与 `--host` 都没给或同时给；`--allow-implausible` 缺 `--out`） |
| calib_fetch | 4 | **不可继续**：ssh 抓取失败或超时（含 `-o BatchMode=yes` 下需要口令 / host key 确认）、`--kind teleop`、输入不是合法 JSON、顶层形状认不出、结构自洽失败、`WRONG_DATASET`（遥操臂）、接受名单坏、登记表读写失败、`--out` 写不了、留痕写不出来 |

`capture_robot_fingerprint.py` 不产生 `9`：缺 `latest_request_timing` 时它继续采集并把计时字段记 `null`。

### calib_fetch 的两条红线

1. **远端读不到不回落本地缓存。** ssh 非零退出、起不了 ssh、stdout 为空、超时，一律退出 `4`，不拿盘上旧文件冒充「刚抓的标定」。ssh 调用带 `-o BatchMode=yes -o ConnectTimeout=20` 与 20 秒 `subprocess.run(timeout=)`（常量 `SSH_TIMEOUT_S`，现场网络慢可调这一处），任一处超时都落 `4`。
2. **退出 `4` 一律不留标定快照。** 门禁顺序是「`--kind teleop` → 结构自洽 → 数据集形状」，写盘顺序是「留痕 → 登记 → 快照」；快照只在门禁全过之后才写。

### calib_health 的两个开关（不改既有四态与退出码）

| 开关 | 作用 | 对 stdout / 退出码的影响 |
| --- | --- | --- |
| `--accepted <path>` | 包络接受名单（默认仓库内 `configs/calib-envelope-accepted.yaml`），**默认就跑**包络判定 | 只追加 `CALIB_MOTOR:` / `CALIB_ENVELOPE:` / `CALIB_ENVELOPE_IMPLAUSIBLE:` 行；状态行 `CALIB_STATE:` 与退出码不变 |
| `--session-anchor <calib_id>` | 执法「会话内不可变」：锚点与文件 `calib_id` 不符即阻断 | 不符时 stdout 多出 `SESSION_ANCHOR: MISMATCH <anchor> -> <current>` 与第五态 `CALIB_STATE: CALIB_SESSION_DRIFT`，退出码 `4`；相符时不打额外行，结论与退出码与不传时一致 |

<a id="exit-code-2"></a>
### 退出码 2 的全部触发点

参数解析失败，一律 `2`，不经过任何 `EXIT_*` 常量；所有判定都在打开任何 socket 与调用 `client_factory(...)` 之前完成。`measure`（rtt 与 tracking 共用同一个 `parse_args`，rtt 也查这些旋钮）：argparse 自身产生的用法错误 —— 未知选项、缺少 `--host`、`--mode` 不在 `rtt`/`tracking`、`--anchor` 不在 `{present,command}`（stderr 原文 `measure_latency.py: error: argument --anchor: invalid choice: 'bogus' (choose from present, command)`，`--mode rtt` 接受该参数并忽略）、任一 `type=` 转换失败（`--port` / `--samples` / `--request-window` / `--steps` 非整数，或任一 `type=float` 选项不是可解析的数值文本；`--csv` 的 `type=Path` 对命令行字符串不会抛，不在此列）；整数下界 3 项 —— `--samples < 1`、`--request-window < 1`、`--steps <= 0`；「有限且带符号」同查 7 项 —— `--rate-hz`（有限且 `> 0`）、`--control-hz`（有限且 `>= 1`）、`--host-loop-hz`（有限且 `> 0`）、`--timeout-s`（有限且 `> 0`）、`--settle-timeout-s`（有限且 `> 0`）、`--settle-tolerance-deg`（有限且 `>= 0`，`0` 合法即最严格设置）、`--delta-deg`（有限且**非零**，`0` 与 `-0.0` 都落 `2`，负值合法；判据 `measure_latency.py:1798`），非有限值（`nan` / `inf`，含 `float()` 后变成 `inf` 的 `1e400`）与不满足符号条件的值都落 `2`；交叉校验 1 项 —— `--settle-tolerance-deg >= abs(resolve_delta_deg(args))`（与钳到 1.0° 幅度上限之后的**有效 delta 的绝对值**比较，`measure_latency.py:1812-1817`；实测 stderr 原文 `must be below the effective --delta-deg magnitude`）。`capture` 的数值校验只有 2 项 —— `--samples < 1`、`--timeout-s` 有限且 `> 0`（不查 `--rate-hz` / `--control-hz` 之类），另加同一形状的 argparse 用法错误（未知选项、缺少 `--host`、`--port` 非整数）。以上为 `capture` / `measure` 的 `2` 的全部触发点。`tools/calib_health.py` / `tools/calib_register.py` 也会返回 `2`，但只有两类：① **参数 / 用法错误**——两个输入参数**都没给**（「必须给 `--calib-file` 或 `--fingerprint` 之一」/「必须给 `--from-file` 或 `--from-fingerprint` 之一」）、两个输入参数**同时给**（argparse 互斥组报 `not allowed with argument`）、以及 argparse 自身的用法错误（未知选项、缺参数值等，两个脚本同形）；② **登记表不可读或不可写**（`calib_health` 把不可读的 `--registry` 也算在这里）。这两类不经过上面这些旋钮校验，逐条口径见两个脚本的模块 docstring 与 §10 表。`tools/calib_fetch.py` 的 `2` 只有两类触发点（共三种情形：`--from-file` 与 `--host` 都没给、两者同时给、`--allow-implausible` 缺 `--out`）；它不把登记表读写失败算在 `2`，那一类落 `4`。这个 `2` 不与 `1` / `4` 合并：`2` 是调用方式错（还没做任何判定），与 `1`（判定结果：有 `IMPLAUSIBLE`）和 `4`（判定结果：不可继续）不同层，故不合并。幅度钳位本身不产生 `2`：超过 `1.0°` 的 `--delta-deg` 由 `resolve_delta_deg` 静默钳（`measure_latency.py:1784-1793` 的注释写明这是有意的），只有 `0` 才拒。合法预算在运行中到期不走 `2`：`capture` 落 `4`，`measure` 的 rtt 落 `7`、tracking 落 `1`

<a id="exit-code-9"></a>
### 退出码 `9` 的三种形态

| 形态 | 何时触发 | 是否发过请求 |
| --- | --- | --- |
| 属性不存在 | `connect()` 返回后、窗口预热之前 | 没发过 |
| 属性存在但从未填充（整轮是空 dict，`observation_sequence` 不增长） | 窗口预热之后，采样循环放弃那一刻才判，要等满 `--timeout-s` | 发过 1 次 state-only 预热，并至少观测 1 帧 |
| 半填充（缺 `token` / `sent_monotonic_s` / `received_monotonic_s` / `age_s` 任一） | 写行之前逐帧复查，`observation_sequence` 每次增长都查一次 | 已发过请求，并在该帧拒绝 |

三种都不写 CSV，连表头都不写。stderr 分得开：第一种是 `it publishes no latest_request_timing`，后两种是 `it publishes latest_request_timing but never fills it completely`，并跟一句 `missing key(s): …` 点名缺的键。只看退出码分不出这三种，要读 stderr。

退出码同时写在每步的 `$S4DIR/stdout/<步骤号>-$STAMP.exit` 里，查表前先看这个文件。

<a id="gate-a"></a>
<a id="gate-b"></a>
<a id="gate-c"></a>
<a id="gate-d"></a>
<a id="gate-e"></a>
<a id="gate-f"></a>
本文里的 Gate A–F 指 `benchmarks/alohamini_io/measure_latency.py` 里的运行时门禁，即发命令前的六道检查：

| 代码 Gate | 判什么 | 不通过时 |
| --- | --- | --- |
| A | `--enable-motion` 与 `--joint` 两个参数都给了 | exit 6，在建客户端之前就拒，零网络零命令 |
| B | 目标关节出现在首帧观测里 | exit 6 |
| C | 宿主 metadata 的 `robot_model` 非空，且与 `--robot-model` 相等 | exit 6 |
| D | 该关节基线反馈是有限数 | exit 6 |
| E | `delta_deg != 0` 且 `steps >= 1`，且 `\|delta_deg\| × steps <= 20.0`（**负值合法**，负号表示朝反方向走；判总量按绝对值，`benchmarks/alohamini_io/measure_latency.py:2405`、`:2414-2418`） | exit 6 |
| F | ① metadata `schema_version` 必须等于 `2`；`True` 与字符串 `"2"` 被拒，**浮点 `2.0` 实现上会被放行**；② 该关节的 deg-per-unit 能从 v2 校准表算出 | exit 6，零命令 |

Gate F 第①条的判断式在 `benchmarks/alohamini_io/measure_latency.py`，原文 `isinstance(schema_version, bool) or schema_version != HOST_METADATA_SCHEMA_VERSION`（`HOST_METADATA_SCHEMA_VERSION = 2`）。`2.0 != 2` 在 Python 里为 `False`，所以浮点 `2.0` 与整数 `2` 同路放行；`"2" != 2` 为 `True`，字符串被拒。

代码 Gate A 在 `main` 里、建客户端之前执行；B–F 在 `_tracking_preflight` 里、第一条 `send_action` 之前执行。B–F 的顺序是 B、C、D、E、F。

<a id="sec-9-gates"></a>
## 9 已发现问题与已定方案（原 §9 门禁相关行）

| 问题 | 现场怎么处理 | 判据 | 产物 |
| --- | --- | --- | --- |
| P0 曾广播 v1（已关闭） | 只核对，不回退、不重跑 v1 拒绝路径 | `capture` exit 0 且 `robot_metadata.schema_version == 2` | `capture/fingerprint-*.json` |
| Gate F 第二段不可达 | 不制造记录，只记「未触发」；禁止改标定文件造记录 | 无拒绝记录即为正确结果 | `stdout/4.1b-probe-*`（历史同类产物 `stdout/5.1-probe21-*`） |

§9 其余行在 [troubleshooting.md#sec-9](troubleshooting.md#sec-9)。
