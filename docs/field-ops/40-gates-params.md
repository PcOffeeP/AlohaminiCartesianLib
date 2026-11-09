## 7 受控试动（唯一动机器人的节）

### 7.1 开跑勾选单与命令

开跑前逐条勾选，勾不全不开跑：

- [ ] 现场授权到手，授权范围写明（哪个关节、多少度、谁在旁），已落进第 3.9 节的 `scope-*`
- [ ] 单臂：另一臂不参与本次运动
- [ ] 空场：人物离场
- [ ] 无负载：末端无负载
- [ ] 行程按「1° × 更多步」规划：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:504`）钳的是每步**幅度**、不钳符号——`resolve_delta_deg`（`:2118-2126`）返回 `min(max(delta, -1.0), 1.0)`，所以 `--delta-deg 2.0` 按 `1.0` 执行、`--delta-deg -2.0` 按 `-1.0` 执行、`--delta-deg -1.0` 原样执行。stdout 同时打印实际值与请求值，原文形如 `delta_deg=1.000 (requested 2.000)`。要更大总行程只能加 `--steps`，加大 `--delta-deg` 的幅度不产生更大位移
- [ ] **手臂低位或有承托**：抬高的手臂在 lead 一次性释放时会被带走，幅度不可控
- [ ] **有能按断电开关的人在机器人旁**：远程且无人在场时**禁止**跑 `--anchor command`（此时只能跑 `--anchor present`）
- [ ] 断电开关在手边，可随时按下

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

命令：

```bash
run 7.1-tracking env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 5 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-$STAMP.csv"
```

解释器与 `PYTHONPATH` 的来由同第 5.3 节：主环境没装 `lerobot`，`uv run --no-sync python` 会以 exit `3` 失败（2026-11-05 实测）。

参数与理由：

| 参数 | 建议值 | 为什么 |
| --- | --- | --- |
| `--mode tracking` | 必填 | 不写就落回 `rtt`，只采观测 |
| `--enable-motion` | 必填 | 代码 Gate A 的第一个条件 |
| `--joint` | `arm_left_shoulder_pan.pos`（先左臂单关节） | 代码 Gate A 的第二个条件；`.pos` 后缀可省 |
| `--delta-deg` | `1.0` | 幅度被硬性钳到 `1.0°`（常量 `MAX_STEP_DELTA_DEG`，`measure_latency.py:504`），**符号保留**：`-1.0` 按 `-1.0` 执行、`-2.0` 钳成 `-1.0`；`0` 是用法错误（exit 2）。负值即朝反方向 |
| `--anchor` | `present`（默认；验证误差累积形态时用 `command`） | 两模式只差目标锚在哪：`present` 每步重读实测位置、把目标放在「实测加 `delta_deg`」上，lead 只有一步、不跨步累积；`command` 锚在上一步发出的 target（首步落回实测），lead 跨步累积。指标受影响：`command` 的一步可能补掉跨了几步的 lead，`rise_90_time_s` 提前记满、`overshoot_deg` 变成（行程 − 一步 delta）、`error_deg` 停在累积 lead 上，所以这三个量与 `present` 不能横向比较。CSV 与 sidecar 没有锚点列，只能靠 stdout 那行 `note: anchor=command: …` 区分模式，归档时 stdout 必须与 CSV 一起存。物理风险：`command` 下 Gate E 仍按 `\|delta\| × steps <= 20.0` 判，`--steps 20 --delta-deg 1.0` 合法、lead 可积到 20°，静摩擦一旦破开，整段 lead 可能一次性释放；首跑用 3–5 步 |
| `--steps` | 先 `5`，成功后再说 | `\|delta\| × steps` 必须不超过 20.0°；默认 20 步、每步 1.0° 正好顶在上限；`--delta-deg` 传负值时上限按绝对值算，符号不影响上限 |
| `--control-hz` | `50.0`（默认） | 两条命令最小间隔 1/50 s = 20 ms；必须有限且不小于 1，`inf`/`nan` 被拒 |
| `--settle-tolerance-deg` | `0.05`（本次实测；容差小不是 `rise_90` 有值的充分条件） | 默认 `0.15`。2026-11-05 用 `0.05`（小于 `0.1 × delta`，即小于 `0.1°`）跑三轴，`rise_90_ms` 仍报 `count=0`、`p50=nan`：原先「容差不小于 `0.1 × delta` 时 settle 循环先退、于是 `rise_90_ms` 报 `count=0`」的推论被实测推翻，容差取小也补不上这一列 |
| `--settle-timeout-s` | `2.0` | 默认 `1.0` s，20 ms 命令节奏下每步预算偏紧 |
| `--timeout-s` | `60` | 默认 `30` s 对多步加 settle 偏紧，中途超时会以 exit 1 结束 |

`--delta-deg` 的幅度上限与 stdout 表现：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:504`），`resolve_delta_deg`（`:2118-2126`）的实现是 `min(max(delta, -MAX_STEP_DELTA_DEG), MAX_STEP_DELTA_DEG)`——**钳的是幅度，不钳符号**。请求 `--delta-deg 2.0` 时按 1.0 执行，请求 `-2.0` 时按 -1.0 执行，请求 `-1.0` 时原样执行。report 首部把实际值与请求值都打印出来，原文形如 `delta_deg=1.000 (requested 2.000)`（打印在 `measure_latency.py:3369-3370`）。要凑更大总行程，只能改成「1° × 更多步」，加大 `--delta-deg` 的幅度无效。

横幅里那句 `note: settle tolerance <X> deg >= 0.1 * delta (<Y> deg)`（`measure_latency.py:3427-3431`）有一处既有文案瑕疵：标签写的是 `0.1 * delta`，括号里的 `<Y>` 打印的却是 **`delta` 本身**。判读时按 `delta` 本身读那个数，不要把它当成「0.1 × delta 的下限值」。该瑕疵已登记在 `TODO.md`。

负值合法这条与退出码的关系：`--delta-deg 0` 才是用法错误（exit 2，stderr 原文 `--delta-deg must be a finite and non-zero number (got 0.0); negative steps are legal, a zero step commands nothing`，判据在 `measure_latency.py:1798`）；`--delta-deg -1` 一路过到 connect，本地实测主环境无 `lerobot` 时以 exit 3 结束，不是 2。

关于 `rise_90_time_s` 为空：源码的说法是这一列 can be empty，即**可能**为空。是否为空取决于该关节是否在同一次轮询里越过 90% 线；容差取小未必空，取大也未必不空。

运动学事实：

- 方向：默认 `--anchor present` 下每步目标是当前实测基线加 `delta_deg`；`--anchor command` 下每步目标是上一步发出的 target 加 `delta_deg`，锚是命令值而不是实测值，因此每步的 lead 跨步累积。`--delta-deg` 允许负值（只有零被拒），负值即朝反方向，同一轮内各步符号相同，所以每步朝同一方向累积。
- 脚本不检查关节限位。源码里没有任何对 URDF 或 adapter 限位的比较。方向与累计行程是否安全，只能靠人。
- 反馈失联保护：连续 3 次 `observation_sequence` 不增长就中止（exit 1）。中止前仍会发收尾 hold。
- 收尾 hold 从 `finally` 发，正常结束、命令被拒、反馈死、Ctrl-C 都会发。只有一次 `*.pos` 都没测到时才不发。
- hold 是就地冻结：把实测 `*.pos` 当目标，三轴速度钉 0，反馈带高度就回写实测高度。报告里只写 `hold_issued`，不写「sent」，因为 action socket 是 PUSH/CONFLATE 无确认。
- exit 8 表示 hold 被拒，也就是停命令没到。这一条按「机器可能仍在动」处置：先断电，再排查。处置见第 8.1 节与第 12 节。

一处未核实：电机名到关节观测键（如 `arm_left_shoulder_pan.pos`）的对应关系本文没有核实过。`--joint` 写错时由代码 Gate B 拒绝（exit 6、零命令），不会动机器人。

产物：`tracking/tracking-$STAMP.csv`、其 sidecar `tracking-$STAMP-safety.csv`、report 原文与 `stdout/7.1-tracking-*`。

### 7.1b Gate C 反向拒绝（模型名故意写错）

目的：拿一次「宿主与 `--robot-model` 不一致 → 不动机器人」的记录，补 V10 的反向证据。

```bash
run 7.1b-gatec-reverse env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --robot-model alohamini2 --delta-deg 1.0 --steps 1 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-7.1b-$STAMP.csv"
```

`--robot-model alohamini2` 是故意写的错值：宿主报的是 `alohamini2pro`（`configs/site.yaml:12`）。两个值都是 vendor 的合法 SKU（vendor 那份型号表 `model_specs.py:65-80`，`ec65318f` 里 `alohamini2` 与 `alohamini2pro` 两段），且两侧的关节键名集合相同（同文件 `model_specs.py:33-50`，`am-follower-6dof` 与 `am-follower-6dof-hd` 两个关节元组逐项同名），按代码顺序看 Gate B（键在不在）不会先拒——**2026-11-06 实测推翻这一预判**，理由见本节「实测结果」。

#### 实测结果（2026-11-06）：真机落 Gate B，不是 Gate C

原先按代码顺序预判的两支，实测只成立一支，且与预判相反：exit `6`，但拦下来的是 **Gate B**（键在不在），不是 Gate C（模型名）。

| 分支 | 现象 | 结论与记录 |
| --- | --- | --- |
| ① 真机主支（2026-11-06 实测） | exit **6**；stderr 原文四行：厂商客户端先打 `ERROR:root:Error processing observation data, serving last observation: Host robot_model does not match the client configuration`，随后本库打 `Gate B refused: 'arm_left_shoulder_pan.pos' is not part of the connected state schema.`（`available: forward, wrist_right`）与 `Check --joint and --robot-model against the host side; nothing was commanded.`；无 CSV（`ls` 报 `No such file or directory`） | 记 Gate B 反向拒绝成立：厂商客户端先按 `robot_model` 把观测 schema 降级成只剩相机键，本库的键存在性检查先拦下，命令一次没下发 |
| ② 不可达 | Gate C 文案 `Gate C refused: the host reports robot_model='alohamini2pro' but --robot-model='alohamini2' was requested.`（`benchmarks/alohamini_io/measure_latency.py:2392-2396`） | **真机链路不可达**：Gate B 在它之前就拒了。Gate C 的零命令拒绝只能由离线测试锁定（见下） |

机理：厂商客户端解每一帧观测时拿 metadata 的 `robot_model` 与自己的配置比对，不一致就打一条 ERROR 并退回上一帧观测（`serving last observation`），此时状态 schema 里已没有关节键；本库的键存在性检查（Gate B）按 `--joint` 找不到键，直接以 `6` 退出，Gate C 的模型名比对根本没走到。**这是厂商行为，不改 vendor，也不为凑 Gate C 去改本库代码。** Gate B 同样是有效的「零命令」证据。

原文此处曾写「两侧关节键名集合相同，所以 Gate B（键在不在）不会先拒」——2026-11-06 实测推翻：厂商先在客户端侧把 schema 降级，键集合随之改变，Gate B 反而先拒。V10 的 Gate C 专属实机子项在真机上不可达，是否接受 Gate B 替代由上层裁决（已登记进 `TODO.md`）。

Gate C 本身的零命令拒绝由离线测试锁住（现场拿不到就以它们为据）：`tests/benchmarks/test_measure_latency.py::test_main_should_refuse_tracking_when_the_host_reports_another_robot_model`（退出码 `6`、`sent_actions == []`、不写 CSV）、`::test_main_should_refuse_tracking_when_the_host_cannot_name_its_model[empty_metadata|no_robot_model|null_robot_model]`（宿主报不出型号时 fail closed）。

#### 取证三件套

真机上只可能落到第 ① 支（Gate B）；第 ② 支（Gate C 专属文案）在真机链路不可达，只由离线测试锁定。现场这三件照样取：

1. **stderr 原文**（`.log` 里那两行）；
2. **无 CSV 产物**：`run 7.1b-no-csv ls -l "$S4DIR/tracking/tracking-7.1b-$STAMP.csv"`，期望 `No such file or directory`；
3. **命令行回显**：`.cmd` 文件里那条完整命令。

「零命令」在现场的可取证形态就是这三件；V10 的 `send_action` 计数只能在离线测试里锁（现场没有计数钩子，`--mode tracking` 的真 client 也不暴露下发计数）。

产物：`stdout/7.1b-gatec-reverse-$STAMP.{cmd,log,exit}`、`stdout/7.1b-no-csv-$STAMP.*`。

### 7.2 肩部 hold 与目标接受判定

试动后立刻取 sidecar 与 Host 追踪行，不要隔一轮再取。

背景：2026-10-25 五次运行都是左臂单关节、每步 1°：腕部 `arm_left_wrist_roll` 达成 87.9%（两步 `settled=True`），肩部 `arm_left_shoulder_pan` 与 `arm_left_shoulder_lift` 5.9%–13.2%、全部 `settled=False`。完整数字见 `docs/field-results.md`。本步要判的是：这两轴的停滞发生在 Host 侧还是 Host 下游。

达成率口径（`docs/ADR.md` D20）：分母统一取命令行给的名义指令行程 `abs(--delta-deg) × --steps`；实测净行程 = 末步 `measured_deg` − 首步 `baseline_deg`；达成率 = 实测净行程 ÷ 名义指令行程。

为什么不再用 `Σ(cmd_deg − baseline_deg)`：`--anchor command` 下每步的 `cmd_deg − baseline_deg` 含跨步累积的 lead，逐步求和会把同一段行程重复计入（2026-11-08 路 A：同一份数据用该分母得左 `43.0108%` / 右 `48.6417%`，用名义分母得左 `79.1209%` / 右 `82.6374%`）。

**例外**：`--anchor present` 且逐步实际命令增量与 `--delta-deg` 不等时（腕部已记录的形态：每步实际只下 `0.879120879°`，`delta=1.0°`），分母改用 `Σ(cmd_deg − baseline_deg)`，并在记录里写明这一次用的是哪个分母。原因见 `docs/field-results.md` 2026-11-06 节「腕部同参数对照」与 `TODO.md` 的「腕与肩的 deg→units 换算不自洽」条。

残余门槛同样见 D20：肩类（`shoulder_pan` / `shoulder_lift`）按 `≤2°` 判，其余轴按 `≤2 tick（0.1758°）` 判。

主依据三项，全部来自 sidecar（列定义见第 11 节）：

1. `target_source`（末列，整帧属性）：取值 `none` / `command` / `watchdog` / `protection`，点名该帧的目标由哪一层给出。同一 `sample_sequence` 的每一行同值。
2. `requested_target` 与 `accepted_target`：同一行内两者不相等，说明 Host 收到的目标与它真正采纳的目标不同，即目标在半路被改过（电流限幅是两列之间那一道）。
3. `current_ma` 逐帧走势：按 `sample_sequence` 递增、固定一个 `motor` 取序列，与主 CSV 同一步的 `measured_deg − baseline_deg` 对照。判据是「持续非零且随步进不回落」。

结论只有三类，现场必须落到其中一类并写明：

| 结论 | 判据 | 记录怎么写 |
| --- | --- | --- |
| ① 目标被接受，停滞在伺服或机械侧 | 全步 `target_source=command`，且同一步内 `requested_target == accepted_target` 全段成立 | 写明「目标已被 Host 采纳」；电流走势作为旁证，并注明电流判读是否可用（第 6.2 节的结论） |
| ② 目标被底层接管 | 任一步出现 `target_source` 为 `protection` 或 `watchdog` | 写明出现该值的 `step_index` 与帧数。早于该轮第一条命令帧的那一帧不算：它记的是上一轮遗留的 `watchdog`（上一轮客户端结束后 Host 看门狗触发过，`target_source` 一直保持到下一个合法命令），与本轮无关，见 §10.5 M1 的「首帧 `target_source=watchdog`」段与 `docs/field-results.md` 2026-11-06 节 |
| ③ 目标未及时接受 | `accepted_target` 相对 `requested_target` 滞后于同一序列的目标（同一行不相等，或目标要到后续帧才出现） | 写明首次出现差异的 `sample_sequence` |

必须现场核实的一条：`hold_active` 在这版固件上是否恒 `False`。用 grep 取原文：

```bash
run 7.2-joint-hold-write ssh "$PI" 'grep -n "_joint_hold_goal\[" ~/lerobot_alohamini/src/lerobot/robots/alohamini/alohamini.py'
```

判法：

- 命中的只有读用法（`self._joint_hold_goal[...]` 在等号右边，或 `dict(...)`、`.get(...)`、`.pop(...)`）→ 没有写入点，`hold_active` 恒 `False`，**不作判据**。出处写进记录。
- 命中里出现写用法（等号左边是 `self._joint_hold_goal[...]`）→ 有写入点，`hold_active` 可作判据，按 sidecar 的 `True` / `False` 文本判读。

本地 fork（HEAD `ec65318f`）的读法结论是「无写入点」，行号清单见 `docs/ADR.md` D18 与 `docs/field-results.md` 2026-11-05 节。Pi 上跑的是合并后的 `6aee1f81`，是否相同以本步输出为准，不要沿用本地结论。

旁证是 Host stdout 的 `[HOST TRACKING][<joint>]` 行，行内含 `target= / command= / measured= / error= / current=`。从 Host 日志里 grep 该前缀，把原文落盘：

```bash
run 7.2b-host-tracking ssh "$PI" 'HOSTLOG=/tmp/site-host-<STAMP>.log; grep "\[HOST TRACKING\]" "$HOSTLOG" | grep "shoulder_pan\|shoulder_lift" | tail -30'
```

判据：三类结论落到一类；`hold_active` 可用与否有 grep 原文为证；`[HOST TRACKING]` 行原文在盘。

产物：sidecar CSV（`tracking/tracking-$STAMP-safety.csv`）、`stdout/7.1-tracking-*`、`stdout/7.2-joint-hold-write-*`、`stdout/7.2b-host-tracking-*`。

## 9 已发现问题与已定方案

一条问题一行，写的是「到现场怎么处理」；发现经过不放这张表。

| 问题 | 现场怎么处理 | 判据 | 产物 |
| --- | --- | --- | --- |
| P0 曾广播 v1（已关闭） | 只核对，不回退、不重跑 v1 拒绝路径 | `capture` exit 0 且 `robot_metadata.schema_version == 2` | `capture/fingerprint-*.json` |
| Gate F 第二段不可达 | 不制造记录，只记「未触发」；禁止改标定文件造记录 | 无拒绝记录即为正确结果 | `stdout/4.1b-probe-*`（历史同类产物 `stdout/5.1-probe21-*`） |
| 肩部两轴不跟随 | 试动后取 sidecar 与 `[HOST TRACKING]`（后者因日志级别压制不打，改用 sidecar，见第 6.1 节）：判据三项里 `requested == accepted` 零例外、运动帧 `target_source` 全 `command`，结论落到「① 目标被接受，停滞在伺服或机械侧」 | 达成率 `≥50%`（分母 `abs(delta) × steps`）且该步步末残余按关节类别分级：肩类 `≤2°`、其余轴 `≤2 tick（0.1758°）`；若用 `settled=True` 口径，容差必须 ≥1 LSB（0.0879°）（口径见 `docs/ADR.md` D20 与 `docs/acceptance-baseline.md` V01 行）。若两轴仍不跟随，按 §10.5 的 teleop oracle → M1 → M2 定性，结论落到硬件不可修时按 `docs/acceptance-baseline.md` §2.1 记「受阻」 | sidecar CSV + `stdout/7.2-*` |
| 已判：臂总线电流不能判负载 | 静止基线与运动窗口的读数全部低于 20 mA 门槛（数字见 `docs/field-results.md` 2026-11-05 节），结论：臂总线电流不能判负载，依赖电流的判断降级为「无电流证据」 | 已判：门槛 20 mA（3 个 raw 刻度），不能判读 | `stdout/6.1*-`、`stdout/6.2*-`、`stdout/6.2b-analysis-*`、`stdout/7.2-analysis-*` |
| `Torque_Enable=1` 写入者未识别 | 只记录，不主动改扭矩寄存器 | 寄存器读数原文落盘 | `stdout/4.2-*` |
| 左夹爪 51 tick（D14 暂不重标定） | 只读 ID7 限位 | 是否 `2021` / `2072` | `stdout/4.2-*` |
| 跨机 RTT（2026-11-05 已测） | 已执行：隔离环境 + fork 快照 `PYTHONPATH` 跑一次 `--mode rtt`（命令见 §5.3） | 已测：exit 0、500 行，`net_rtt_estimate_s` p50 与 Pi 回环同量级（数字见 `docs/field-results.md` 2026-11-05 节） | `rtt/rtt-net-cross-20261105T063600Z-2.csv`、`stdout/5.3-rtt-net-cross-2-*` |
| 4 路相机零帧 | 采集时把 stderr 落盘 | 有零帧原文或无非空帧 | `stdout/` + `capture/*.json` |

两条禁令，原文照办：

- 不要为了制造 F 的拒绝去改标定文件。
- 不要为了造记录去改标定文件或伪造数据。

两条现场判读要点：

- Pi 上 `measure --mode rtt` 返回 `9`、`capture` 的计时字段为 `null`，都是该 vendor 缺 `latest_request_timing` 时的设计行为，不当故障排查。升级后 HEAD `6aee1f81` 已提供该面。`9` 的三种形态见第 10 节。
- `hold_active` 的作用范围见第 7.2 节：先用 `7.2-joint-hold-write` 确认有无写入点，没有就不作判据。

## 12 失败与中止处置

| 现象 | 判据 | 处置 |
| --- | --- | --- |
| exit 6（capture） | stderr 含 `schema_version=` | 记录原文，核对第 1 节 P0，不要重试 |
| exit 6（measure） | 拒绝发生在首帧后 | 记录是哪道代码 Gate（第 7.1 节表），写进 `"$S4DIR/stdout/"` |
| exit 8 | 报告里 hold 被拒 | 先断电，再排查；按「仍在动」处置，见第 8.1 节 |
| exit 1 加反馈失联 | 连续 3 次序号不涨 | 检查网络与 Host 负载 |
| 网络中断 | `ping` 丢包 | 重跑该步，不复用旧产物 |

停 Host 不等于失力、断电时序与 exit 8 的处置，见第 8.1 节。

## 附录 A：两个脚本的完整参数表

### capture（6 个参数加 `-h`）

| 参数 | 必填 | 默认值 | 约束 | 影响什么 |
| --- | --- | --- | --- | --- |
| `--host` | 是 | 无 | 无 | 连哪台 Pi |
| `--port` | 否 | `5556` | 无 | 宿主观测端口 |
| `--robot-model` | 否 | `alohamini2pro` | 无 | 传给客户端的模型名；代码 Gate C 拿它与宿主 metadata 的 `robot_model` 比对，取值按第 4.1 节 JSON 里的值填 |
| `--samples` | 否 | `5` | 不小于 1（否则参数解析失败 exit 2） | 要采到的观测请求数 |
| `--timeout-s` | 否 | `30.0` | 必须有限且大于 0（否则参数解析失败 exit 2）；预算合法但到期仍不够样本落 exit 4 | 采样本的总时间预算，也是 `capture` 唯一的停机条件 |
| `--out` | 否 | `artifacts/alohamini-io/fingerprint-<UTC>.json`，解析参数时算时间戳 | 父目录自动创建 | JSON 落盘位置 |

### measure（18 个参数加 `-h`）

| 参数 | 必填 | 默认值 | 约束 | 影响什么 |
| --- | --- | --- | --- | --- |
| `--host` | 是 | 无 | 无 | 连哪台 Pi |
| `--port` | 否 | `5556` | 无 | 宿主观测端口 |
| `--robot-model` | 否 | `alohamini2pro` | 无 | 代码 Gate C 的比对对象 |
| `--mode` | 否 | `rtt` | `rtt` 或 `tracking` | 跑哪套测量；`tracking` 会真动机器人 |
| `--rate-hz` | 否 | `30.0` | 必须有限且大于 0（否则参数解析失败 exit 2）；小于 200 打警告 | 观测采样率，与 `--control-hz` 独立 |
| `--control-hz` | 否 | `50.0` | 必须有限且不小于 1（否则 exit 2） | 命令最小间隔的倒数 |
| `--samples` | 否 | `500` | 不小于 1（否则 exit 2），仅 rtt 用 | 要采到的观测帧数 |
| `--timeout-s` | 否 | `30.0` | 必须有限且大于 0（否则参数解析失败 exit 2）；预算合法但在运行中到期不是 `2`：rtt 落 exit 7（样本不足）、tracking 落 exit 1（提前停止） | 整次运行的时间预算 |
| `--csv` | 否 | `artifacts/alohamini-io/<mode>-<UTC>.csv` | 无 | CSV 落盘位置 |
| `--request-window` | 否 | `1` | 不小于 1（否则 exit 2） | 在飞观测请求数；1 是 RTT 形态，3 是部署形态 |
| `--host-loop-hz` | 否 | `50.0` | 必须有限且大于 0（否则 exit 2） | 宿主主循环频率，决定相位项与排队项 |
| `--enable-motion` | 否 | 开关，默认关 | 无 | 代码 Gate A 的第一个条件 |
| `--joint` | 否 | 无 | tracking 模式实质必填 | 要步进的关节；代码 Gate A 的第二个条件 |
| `--delta-deg` | 否 | `1.0` | 必须有限且**非零**（`0` 落 exit 2）；幅度被钳到 `±1.0`、符号保留（`measure_latency.py:504`、`:2118-2126`） | 每步目标增量（度）；负值即朝反方向 |
| `--steps` | 否 | `20` | 不小于 1（否则 exit 2） | 步数；`\|delta\| × steps` 受 20.0° 上限约束（`measure_latency.py:2414-2418`） |
| `--settle-timeout-s` | 否 | `1.0` | 必须有限且大于 0（否则 exit 2） | 每步的稳定判定预算 |
| `--settle-tolerance-deg` | 否 | `0.15` | 必须有限且不小于 0（`0` 合法）；且必须小于 clamp 后有效 delta 的**绝对值**；两者违规均 exit 2（`measure_latency.py:1812-1817`） | 稳定判定的容差（度） |
| `--anchor` | 否 | `present` | 只能取 `present` / `command`（其它值参数解析失败 exit 2）；只在 `--mode tracking` 生效，`--mode rtt` 接受该参数并忽略 | 每步目标的锚点：`present` 锚实测位置，lead 只有一步；`command` 锚上一步发出的 target（首步回落实测），lead 跨步累积 |

`measure` 的 `--help` 用的是 `RawDescriptionHelpFormatter`，不打印默认值。上表默认值来自源码常量。
