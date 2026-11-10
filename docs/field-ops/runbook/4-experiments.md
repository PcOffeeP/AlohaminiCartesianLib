# 4 实验卡 E1–E8（原 §5、§6、§7、§10.5–§10.8）

本文件管什么：八张按顺序走的实验卡（E1→E2→E3→E4→E5→E6→E7→E8），每张给命令、判据、前置与产物指针。

<a id="exp-e1"></a>
## E1 零运动测量（原 §5）

| 字段 | 内容 |
| --- | --- |
| 目的 | 只采样观测、不构造任何运动指令的零运动测量：部署形态与 RTT 形态各一条命令 |
| 前置 | 见 [§2](1-prepare.md#sec-2) 的变量块与 `run` 函数；只读 SSH 的授权见 [§0](1-prepare.md#sec-0) 的授权表 |
| 前置卡 | 无（开场 1-prepare.md） |
| 后继卡 | E2 |
| 命令 | 见本卡「5.1 部署形态（请求 30 Hz、窗口 3）」与「5.2 RTT 形态（请求 1000 Hz、窗口 1）」两小节 |
| 判据 | 见本卡 5.1 与 5.2 各自的「判据」行 |
| 预期剧本 | 本卡未单列 |
| 失败处置 | 见 [§12](../reference/troubleshooting.md#sec-12) 与 [§10](../reference/gates.md#exit-codes) 的退出码对照 |
| 产物 | 见本卡「产物」一行 / [§11](../reference/artifacts.md#sec-11) |

三条命令都不构造任何运动指令：`measure --mode rtt` 只采样观测。三条都过 `run`：stdout 与 stderr 合并落到 `$S4DIR/stdout/<步骤号>-$STAMP.log`，退出码落到同名 `.exit`。

<a id="sec-5-1"></a>
### 5.1 部署形态（请求 30 Hz、窗口 3）

```bash
run 5.1-rtt-deploy env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 30 --request-window 3 --samples 500 --csv "$S4DIR/rtt/rtt-deploy-$STAMP.csv"
```

判据：exit 0，`$S4DIR/rtt/rtt-deploy-$STAMP.csv` 写出且有数据行。返回 `3` 是连接问题，按 [§12](../reference/troubleshooting.md#sec-12) 处置。

解释器不用 `uv run --no-sync python`：主环境没装 `lerobot`，那样写会以 exit `3`、`No module named 'lerobot'` 失败（2026-11-05 实测）。隔离环境与 `PYTHONPATH` 快照的取法见 [§5.3](../reference/env-rebuild.md#sec-5-3) 的「解释器与 `PYTHONPATH`」小节。

<a id="sec-5-2"></a>
### 5.2 RTT 形态（请求 1000 Hz、窗口 1）

```bash
run 5.2-rtt-net env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 --csv "$S4DIR/rtt/rtt-net-$STAMP.csv"
```

解释器与 `PYTHONPATH` 的来由同 5.1（见 [§5.3](../reference/env-rebuild.md#sec-5-3)）；`uv run --no-sync python` 在主环境会以 exit `3` 失败，不是代码问题。

判据：`exit 0` 且 CSV 有数据行。

若返回 `9`：该时点这份 vendor 的 client 不发布 `latest_request_timing`，rtt 无法测量。这是设计行为，按 [§10](../reference/gates.md#exit-codes) 记录，不当故障、不重试改参数。升级后 HEAD `6aee1f81` 提供该面，实测 exit 0。

在 Pi 上执行时，必须把 `PYTHONPATH` 指向被测代码，否则会 import 到已安装的旧版本。采集脚本的 `default_client_factory()` 直接 `import lerobot`，不经过本库 adapter。不设 `PYTHONPATH` 时有两种表现：指纹计时字段全 `null`，或 `measure --mode rtt` 返回退出码 `9`。两种都看着像代码缺陷，实际原因是跑错了代码。

两条读法必须先定下来：

- `--rate-hz` 低于 200 时，`age_s` 的主体是响应在客户端 socket 里等下一次 poll 的空闲时间，所以它测的是「端到端观测滞后」，不能当网络往返时间读。要测 RTT 必须用 `--rate-hz 1000`。脚本在低于 200 Hz 时自己打警告；打了警告不等于失败，但它封住了这一组数值当 RTT 读的用法。
- `--request-window 1` 是 RTT 形态（无预取等待可减）；`--request-window 3` 是部署形态（控制栈实际用的窗口）。两者不可混为一谈，也不可互相替代。

`--samples` 默认是 `500`，本文照样写出来，便于现场比对命令原文。参考值（2026-10-22）：部署形态组 `achieved_rate_hz=30.1`（请求 30 Hz）；RTT 形态组 `achieved_rate_hz=49.0`（请求 1000 Hz）。实际到达率受 Host 主循环节拍限制。

产物：`rtt/rtt-deploy-$STAMP.csv`、`rtt/rtt-net-$STAMP.csv`，以及 `stdout/5.1-rtt-deploy-*`、`stdout/5.2-rtt-net-*`。

[§5.3](../reference/env-rebuild.md#sec-5-3) 隔离环境与 `PYTHONPATH` 重建 8 步见 `../reference/env-rebuild.md#sec-5-3`。

<a id="exp-e2"></a>
## E2 量级验证：臂总线电流能不能判读（原 §6）

| 字段 | 内容 |
| --- | --- |
| 目的 | 给出「臂总线电流能不能用来判负载」这条前置事实（取自原节首段） |
| 前置 | 见本卡「6.2 短运动电流量级」的前置条件行；启停 Host 的前置与三步法见 [§6.1](2-host.md#host-start) |
| 前置卡 | E1 |
| 后继卡 | E3 |
| 命令 | 见本卡「主路径」「支路」「6.2 短运动电流量级」三小节；启 Host 的三步法在 [§6.1](2-host.md#host-start) |
| 判据 | 见本卡「主路径」的判据行与「6.2」的结论表 |
| 预期剧本 | 见本卡 6.1 的「基线参考值」与 6.2 的「实测参考」两段 |
| 失败处置 | [§12](../reference/troubleshooting.md#sec-12) |
| 产物 | 见本卡两处「产物」一行 / [§11](../reference/artifacts.md#sec-11) |

本节要给出一条前置事实：臂总线电流能不能用来判负载。后面所有依赖电流的判断（[§7.2](#exp-e3)）都以这个结论为准。取数主路径是客户端的 `latest_safety_status["currents_ma"]`；`[Currents]` 这条日志在当前 vendor 的日志级别下不打印（见 [§6.1](2-host.md#host-start)）。

先固定三件已核对的事：

- 打印值已经是 mA，不是 raw 刻度：Host 侧按 `int(raw × 6.5)` 截断后打印。
- 电流读取本身正常：同一轮 Host 日志里升降轴 `Present_Current` 读到 `71.5 / 221.0 / 936.0 / 2639.0 mA`（`stdout/v2verify-host-20261025.log`）。
- 局限在臂总线：2026-10-25 整份日志的 `[Currents]` 共 1278 行、11502 个元素（元素分布：`0` 11056 个、`6` 444 个、`13` 2 个），最大值 `13 mA`，等于 2 个 raw 刻度。静止与轻载下只有 0–2 个刻度，运动中的量级见 [§6.2](#exp-e2)（2026-11-05 实测）。计数须按整份日志复算，按前缀统计会偏小。

`[Currents]` 数组下标与电机 ID 的映射**没有文档**：禁止按位置推断是哪个电机。要判具体电机的电流，用 sidecar 的 `current_ma` 列（带键名，见 [§11](../reference/artifacts.md#sec-11)）。厂商快照里的键叫 `currents_ma`，写进 sidecar 的列叫 `current_ma`，查代码时别把这两个名字互换。

#### 主路径：`latest_safety_status["currents_ma"]`（2026-11-05 起）

用一个小脚本连上客户端的观测面，逐帧读 18 个电机的 `currents_ma` 并汇总。脚本范式是 `stdout/6.1c-obs-idle-$STAMP.py`，只调 `connect()`、`get_observation(include_cameras=False)`、`disconnect()`，不发任何动作。本次参数 5 Hz × 20 s = 100 帧。运行方式与 [§4.2](3-health.md#sec-4-2) 同形（脚本投到 Pi 的 `/tmp`、用 Pi 侧 venv 解释器跑，`scp` 需「向 Pi 写文件」授权）：

```bash
run 6.1c-obs-idle ssh "$PI" 'cd /home/pi5 && PYTHONPATH=/home/pi5/lerobot_alohamini/src ~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/6.1c-obs-idle-$STAMP.py'
```

产物：`stdout/6.1c-obs-idle-$STAMP.{cmd,log}`。日志末尾自动打两段汇总：`# per_motor_table`（逐电机 `n` / `min_mA` / `max_mA` / `mean_mA`）与 `# summary`（全局最大、`motors_seen`、恒 0 电机清单）。

判据：`.exit` 为 `0`；`motors_seen=18`；`# per_motor_table` 有 18 行且每行 `n` 相同。

基线参考值（2026-11-05 实测，18/18 电机 × 100 帧）：读数只有 `0` 与 `+6.50 mA` 两个值，7 只恒 0（`arm_left_gripper`、`arm_left_wrist_yaw`、`arm_right_gripper`、`arm_right_wrist_flex`、`arm_right_wrist_roll`、`base_left_wheel`、`base_right_wheel`），全局最大 `+6.50 mA`。

#### 支路：`[Currents]` grep（当前 vendor 日志级别压制，不可依赖）

```bash
run 6.1b-currents-idle ssh "$PI" 'sleep 30; HOSTLOG=/tmp/site-host-<启动该 Host 时的 STAMP>.log; grep -c "\[Currents\]" "$HOSTLOG"; grep "\[Currents\]" "$HOSTLOG" | tail -3'
```

日志路径里的 `<启动该 Host 时的 STAMP>` 按实际文件名填，同一会话里就是上一条命令里的 `$STAMP`。

这一支**不可依赖**：2026-11-05 实测整份 Host 日志的 `[Currents]` 行数为 0（`stdout/6.1b-currents-dist-*` 打出 `lines 0` / `elements 0`），原因是厂商 client 里 `logging.basicConfig(... level=logging.INFO)` 的 level 那行被注释、root logger 停在 WARNING；同类 INFO 行（`[HOST TRACKING]`）同样不打印（`stdout/7.2b-host-tracking-count-*` 计数 `0`）。所以「有 `[Currents]` 原文」不再作判据，取数走上面的主路径。

产物：`stdout/6.1a-host-start-*`（若启了 Host）、`stdout/6.1c-obs-idle-*`、`stdout/6.1b-currents-idle-*`（支路，当前 vendor 下通常为空）。

### 6.2 短运动电流量级（原 §6.2）

前置条件：6.1 的基线已在盘；第 [§7.1](#exp-e3) 的勾选单已过（至少「授权」与「断电开关在手」两条）；运动授权在手。

跑 2 步单关节步进：

```bash
run 6.2-currents-move env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 2 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-6.2-$STAMP.csv"
```

解释器与 [§5.3](../reference/env-rebuild.md#sec-5-3) 的「解释器与 `PYTHONPATH`」小节一致；写成 `uv run --no-sync python` 会以 exit `3`、`No module named 'lerobot'` 失败。

跑完取数走 sidecar 主路径：按 [§6.1](2-host.md#host-start) 的脚本范式（`stdout/6.1c-obs-idle-$STAMP.py`）统计运动窗口内的 `current_ma`，或直接分析本次 tracking 的 sidecar `tracking/tracking-6.2-$STAMP-safety.csv`。要统计三项：运动窗口内 `current_ma` 的最大值、取值分布、`≥20 mA` 的样本数。

`[Currents]` 的 grep 支保留为旁证（当前 vendor 日志级别压制，整份日志 0 行，不可依赖）：

```bash
run 6.2b-currents-move ssh "$PI" 'HOSTLOG=/tmp/site-host-<STAMP>.log; grep "\[Currents\]" "$HOSTLOG" | tail -20'
```

判据（必须写出结论，不写「看起来正常」）：

| 运动窗口内 sidecar `current_ma` 的读数 | 结论 | 后续怎么用 |
| --- | --- | --- |
| 出现 ≥3 个 raw 刻度（≥20 mA） | 臂总线电流可判读 | 7.2 的电流走势可以当负载旁证用 |
| 仍是 0–2 个刻度 | 臂总线电流不能判负载 | 后续所有依赖电流的判断降级为「无电流证据」，不要拿它下结论 |

`13 mA` 是 2 个 raw 刻度，`20 mA` 对应 3 个刻度，判读门槛就卡在这一档。

实测参考（2026-11-05，同一参数集）：腕部 run 的运动窗口 1638 个（帧×电机）样本，最大 `6.5 mA`，`≥20 mA` 样本 0 个；`pan` / `lift` 两次 run 窗口内最大 `19.5 mA`，取值只有 `0` / `6.5` / `13.0` / `19.5` → 结论「臂总线电流不能判负载」。

两点必须一起写进记录：打印值是 mA，不是 raw 刻度；`[Currents]` 的数组下标与电机 ID 没有映射文档，禁止按位置推断是哪个电机。要判具体电机，用 sidecar 的 `current_ma`（带键名，列定义见 [§11](../reference/artifacts.md#sec-11)）。

判据另一半：主 CSV 与 sidecar 都写出。产物：`tracking/tracking-6.2-$STAMP.csv` 与其 sidecar、`stdout/6.2-currents-move-*`、`stdout/6.2b-currents-move-*`、`stdout/6.2b-analysis-*`、`stdout/7.2-analysis-*`。

<a id="exp-e3"></a>
## E3 受控试动：唯一动机器人的节（原 §7.1、§7.2）

| 字段 | 内容 |
| --- | --- |
| 目的 | 单关节受控步进：开跑勾选单、单关节步进、肩部目标接受判定（原 7.1 与 7.2 两节） |
| 前置 | 见本卡「7.1 开跑勾选单与命令」的勾选单与参数表 |
| 前置卡 | E2 |
| 后继卡 | E4 |
| 命令 | 见本卡「7.1 开跑勾选单与命令」 |
| 判据 | 见本卡「7.2 肩部 hold 与目标接受判定」的结论表与判据行 |
| 预期剧本 | 本卡未单列 |
| 失败处置 | 见 [§12](../reference/troubleshooting.md#sec-12)（exit 8 的处置同本卡 7.1 末条） |
| 产物 | 见本卡两处「产物」一行 / [§11](../reference/artifacts.md#sec-11) |

### 7.1 开跑勾选单与命令

开跑前逐条勾选，勾不全不开跑：

- [ ] 现场授权到手，授权范围写明（哪个关节、多少度、谁在旁），已落进 [§3.9](1-prepare.md#sec-3) 的 `scope-*`
- [ ] 单臂：另一臂不参与本次运动
- [ ] 空场：人物离场
- [ ] 无负载：末端无负载
- [ ] 行程按「1° × 更多步」规划：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:504`）钳的是每步**幅度**、不钳符号——`resolve_delta_deg`（`:2118-2126`）返回 `min(max(delta, -1.0), 1.0)`，所以 `--delta-deg 2.0` 按 `1.0` 执行、`--delta-deg -2.0` 按 `-1.0` 执行、`--delta-deg -1.0` 原样执行。stdout 同时打印实际值与请求值，原文形如 `delta_deg=1.000 (requested 2.000)`。要更大总行程只能加 `--steps`，加大 `--delta-deg` 的幅度不产生更大位移
- [ ] **手臂低位或有承托**：抬高的手臂在 lead 一次性释放时会被带走，幅度不可控
- [ ] **有能按断电开关的人在机器人旁**：远程且无人在场时**禁止**跑 `--anchor command`（此时只能跑 `--anchor present`）
- [ ] 断电开关在手边，可随时按下

六道代码 Gate A–F 的定义表在 [reference/gates.md#gate-a](../reference/gates.md#gate-a)（原 §7.1 的 Gate 表）；本节按那张表执行。

命令：

```bash
run 7.1-tracking env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 5 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-$STAMP.csv"
```

解释器与 `PYTHONPATH` 的来由同 [§5.3](../reference/env-rebuild.md#sec-5-3)：主环境没装 `lerobot`，`uv run --no-sync python` 会以 exit `3` 失败（2026-11-05 实测）。

参数与理由：

| 参数 | 建议值 | 为什么 |
| --- | --- | --- |
| `--mode tracking` | 必填 | 不写就落回 `rtt`，只采观测 |
| `--enable-motion` | 必填 | 代码 [Gate A](../reference/gates.md#gate-a) 的第一个条件 |
| `--joint` | `arm_left_shoulder_pan.pos`（先左臂单关节） | 代码 [Gate A](../reference/gates.md#gate-a) 的第二个条件；`.pos` 后缀可省 |
| `--delta-deg` | `1.0` | 幅度被硬性钳到 `1.0°`（常量 `MAX_STEP_DELTA_DEG`，`measure_latency.py:504`），**符号保留**：`-1.0` 按 `-1.0` 执行、`-2.0` 钳成 `-1.0`；`0` 是用法错误（exit 2）。负值即朝反方向 |
| `--anchor` | `present`（默认；验证误差累积形态时用 `command`） | 两模式只差目标锚在哪：`present` 每步重读实测位置、把目标放在「实测加 `delta_deg`」上，lead 只有一步、不跨步累积；`command` 锚在上一步发出的 target（首步落回实测），lead 跨步累积。指标受影响：`command` 的一步可能补掉跨了几步的 lead，`rise_90_time_s` 提前记满、`overshoot_deg` 变成（行程 − 一步 delta）、`error_deg` 停在累积 lead 上，所以这三个量与 `present` 不能横向比较。CSV 与 sidecar 没有锚点列，只能靠 stdout 那行 `note: anchor=command: …` 区分模式，归档时 stdout 必须与 CSV 一起存。物理风险：`command` 下 Gate E 仍按 `\|delta\| × steps <= 20.0` 判，`--steps 20 --delta-deg 1.0` 合法、lead 可积到 20°，静摩擦一旦破开，整段 lead 可能一次性释放；首跑用 3–5 步 |
| `--steps` | 先 `5`，成功后再说 | `\|delta\| × steps` 必须不超过 20.0°；默认 20 步、每步 1.0° 正好顶在上限；`--delta-deg` 传负值时上限按绝对值算，符号不影响上限 |
| `--control-hz` | `50.0`（默认） | 两条命令最小间隔 1/50 s = 20 ms；必须有限且不小于 1，`inf`/`nan` 被拒 |
| `--settle-tolerance-deg` | `0.05`（本次实测；容差小不是 `rise_90` 有值的充分条件） | 默认 `0.15`。2026-11-05 用 `0.05`（小于 `0.1 × delta`，即小于 `0.1°`）跑三轴，`rise_90_ms` 仍报 `count=0`、`p50=nan`：原先「容差不小于 `0.1 × delta` 时 settle 循环先退、于是 `rise_90_ms` 报 `count=0`」的推论被实测推翻，容差取小也补不上这一列 |
| `--settle-timeout-s` | `2.0` | 默认 `1.0` s，20 ms 命令节奏下每步预算偏紧 |
| `--timeout-s` | `60` | 默认 `30` s 对多步加 settle 偏紧，中途超时会以 exit 1 结束 |

本表只讲建议值与理由；每个参数的**默认值与合法性约束**以 [附录 A：两个脚本的完整参数表](../reference/params.md#param-capture) 为单一来源，本表不重复。

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
- exit 8 表示 hold 被拒，也就是停命令没到。这一条按「机器可能仍在动」处置：先断电，再排查。处置见 [§8.1](2-host.md#sec-8-1) 与 [§12](../reference/troubleshooting.md#sec-12)。

一处未核实：电机名到关节观测键（如 `arm_left_shoulder_pan.pos`）的对应关系本文没有核实过。`--joint` 写错时由代码 [Gate B](../reference/gates.md#gate-b) 拒绝（exit 6、零命令），不会动机器人。

产物：`tracking/tracking-$STAMP.csv`、其 sidecar `tracking-$STAMP-safety.csv`、report 原文与 `stdout/7.1-tracking-*`。

### 7.2 肩部 hold 与目标接受判定

试动后立刻取 sidecar 与 Host 追踪行，不要隔一轮再取。

背景：2026-10-25 五次运行都是左臂单关节、每步 1°：腕部 `arm_left_wrist_roll` 达成 87.9%（两步 `settled=True`），肩部 `arm_left_shoulder_pan` 与 `arm_left_shoulder_lift` 5.9%–13.2%、全部 `settled=False`。完整数字见 `docs/field-results.md`。本步要判的是：这两轴的停滞发生在 Host 侧还是 Host 下游。

达成率口径（`docs/ADR.md` D20）：分母统一取命令行给的名义指令行程 `abs(--delta-deg) × --steps`；实测净行程 = 末步 `measured_deg` − 首步 `baseline_deg`；达成率 = 实测净行程 ÷ 名义指令行程。

为什么不再用 `Σ(cmd_deg − baseline_deg)`：`--anchor command` 下每步的 `cmd_deg − baseline_deg` 含跨步累积的 lead，逐步求和会把同一段行程重复计入（2026-11-08 路 A：同一份数据用该分母得左 `43.0108%` / 右 `48.6417%`，用名义分母得左 `79.1209%` / 右 `82.6374%`）。

**例外**：`--anchor present` 且逐步实际命令增量与 `--delta-deg` 不等时（腕部已记录的形态：每步实际只下 `0.879120879°`，`delta=1.0°`），分母改用 `Σ(cmd_deg − baseline_deg)`，并在记录里写明这一次用的是哪个分母。原因见 `docs/field-results.md` 2026-11-06 节「腕部同参数对照」与 `TODO.md` 的「腕与肩的 deg→units 换算不自洽」条。

残余门槛同样见 D20：肩类（`shoulder_pan` / `shoulder_lift`）按 `≤2°` 判，其余轴按 `≤2 tick（0.1758°）` 判。

主依据三项，全部来自 sidecar（列定义见 [§11](../reference/artifacts.md#sec-11)）：

1. `target_source`（末列，整帧属性）：取值 `none` / `command` / `watchdog` / `protection`，点名该帧的目标由哪一层给出。同一 `sample_sequence` 的每一行同值。
2. `requested_target` 与 `accepted_target`：同一行内两者不相等，说明 Host 收到的目标与它真正采纳的目标不同，即目标在半路被改过（电流限幅是两列之间那一道）。
3. `current_ma` 逐帧走势：按 `sample_sequence` 递增、固定一个 `motor` 取序列，与主 CSV 同一步的 `measured_deg − baseline_deg` 对照。判据是「持续非零且随步进不回落」。

结论只有三类，现场必须落到其中一类并写明：

| 结论 | 判据 | 记录怎么写 |
| --- | --- | --- |
| ① 目标被接受，停滞在伺服或机械侧 | 全步 `target_source=command`，且同一步内 `requested_target == accepted_target` 全段成立 | 写明「目标已被 Host 采纳」；电流走势作为旁证，并注明电流判读是否可用（[§6.2](#exp-e2) 的结论） |
| ② 目标被底层接管 | 任一步出现 `target_source` 为 `protection` 或 `watchdog` | 写明出现该值的 `step_index` 与帧数。早于该轮第一条命令帧的那一帧不算：它记的是上一轮遗留的 `watchdog`（上一轮客户端结束后 Host 看门狗触发过，`target_source` 一直保持到下一个合法命令），与本轮无关，见 [§10.5](#exp-e5) M1 的「首帧 `target_source=watchdog`」段与 `docs/field-results.md` 2026-11-06 节 |
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

<a id="exp-e4"></a>
## E4 Gate C 反向拒绝（原 §7.1b）

| 字段 | 内容 |
| --- | --- |
| 目的 | 拿一次「模型名不一致 → 不动机器人」的反向拒绝记录（原节首段） |
| 前置 | 见本卡「目的」一段；开跑前置同 [§7.1](#exp-e3) 的勾选单（本卡零下发） |
| 前置卡 | E3（并列反向取证） |
| 后继卡 | E5 |
| 命令 | 见本卡开头的命令块（「目的」一段之后） |
| 判据 | 见本卡「取证三件套」小节 |
| 预期剧本 | 见本卡「实测结果（2026-11-06）：真机落 Gate B，不是 Gate C」小节 |
| 失败处置 | [§12](../reference/troubleshooting.md#sec-12) |
| 产物 | 见本卡「产物」一行 / [§11](../reference/artifacts.md#sec-11) |


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

<a id="sec-10-5"></a>
<a id="exp-e5"></a>
## E5 肩部定性专项（原 §10.5）

| 字段 | 内容 |
| --- | --- |
| 目的 | 肩部停滞发生在 Host 侧还是 Host 下游：teleop oracle、M1、M2 三段走法 |
| 前置 | 见本卡「M1（有人在场）」的硬前置段；开跑勾选单见 [§7.1](#exp-e3) |
| 前置卡 | E4 |
| 后继卡 | E6 |
| 命令 | 见本卡「teleop oracle（有人最先做，成本最低）」「M1（有人在场）」「M2（有人在场）」三小节 |
| 判据 | 见本卡 M1 / M2 两张判据表与「产物名与留痕要求」小节 |
| 预期剧本 | 本卡未单列 |
| 失败处置 | 见 [§12](../reference/troubleshooting.md#sec-12) 与 [§8.1](2-host.md#sec-8-1)（强杀后扭矩保持） |
| 产物 | 见本卡「产物名与留痕要求」小节 / [§11](../reference/artifacts.md#sec-11) |

本节接 [§7.2](#exp-e3)：那里已把停滞定位到「不在 PC→Host 值传递段」，Host→总线写入段与伺服 / 机械侧仍无观测。本节是要有人到机器人跟前才能做的跟进动作；2026-11-05 远程会话（无人在机器人现场）只完成了 M0。

复现遥操作那种误差累积形态用 `--anchor command`，这也是现场验证肩部静摩擦能否被累积 lead 破开的用法。[§7.1](#exp-e3) 参数表给的默认值是 `present`（每步重锚实测、lead 只有一步），两者构成对照。首跑用 3–5 步，且 `command` 模式下的 `rise_90_time_s` / `overshoot_deg` / `error_deg` 不与 `present` 横向比较。

### 背景：M0 已离线排除换算链

M0 已离线排除换算链（1° = 11 tick；台账与数字见 `docs/field-results.md` 2026-11-05 节的 M0 行，原始报告 `stdout/M0-tick-recompute-20261105.txt`）；本节阈值按 11 tick/度 取。

本节数字除 M0 两项（`1° = 11.375 tick`、腕部正对照 10/10，属离线复算）外，均出自 `docs/field-results.md` 的 2026-11-05 节。

### teleop oracle（有人最先做，成本最低）

做法：用厂商遥操作（leader 主臂，即手持的遥操作主臂）动 `arm_left_shoulder_pan` 与 `arm_left_shoulder_lift`，全程不经过本库。

启动方式：厂商遥操作的入口与命令仓库里查得到，写死在下面。**时序关键是 Host 保持运行**——vendor 文档原文 `Run these on the PC / DGX client after the host is running.`（`docs/alohamini/commands.md:210`），遥操作是 PC 侧经 ZMQ 连 Host 的远端客户端（`examples/alohamini/teleoperate_bi.py:96-119` 建 `AlohaMiniClientConfig(remote_ip=…)` + `AlohaMiniClient`），它碰的是**主臂**串口，不抢从臂总线。所以**不要停 Host**。

在 PC（Mac）另开一个终端，按 vendor 文档的原样命令跑（`docs/alohamini/commands.md:224-234`）：

```bash
conda activate bot
cd /Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini
python examples/alohamini/teleoperate_bi.py \
  --robot.remote_ip "$PI_HOST" \
  --robot.robot_model alohamini2pro \
  --teleop.id am_leader_bi \
  --teleop.arm_profile am-leader-6dof \
  --fps 50 \
  --camera-fps 30
```

三条必须对上的口径：

- `--robot.robot_model` 必须与宿主实报值一致（本机 `configs/site.yaml:12` 是 `alohamini2pro`）；写错会撞厂商客户端自己的模型校验（`alohamini_client.py:417-420` 抛 `ValueError: Host robot_model does not match the client configuration`），见 [§7.1b](#exp-e4) 的同一条机理。
- `--teleop.arm_profile am-leader-6dof` 是 6 自由度主臂档；只支持 `so-arm-5dof` / `am-leader-6dof` 两个取值（`examples/alohamini/teleoperate_bi.py:55-63`）。
- 主臂串口在 vendor 脚本的**提交口径**里是写死的常量，不按平台取、也不可覆盖：`left_arm_config.port="/dev/am_arm_leader_left"`（`examples/alohamini/teleoperate_bi.py:90`）、`right_arm_config.port="/dev/am_arm_leader_right"`（`:94`），两支提交 `ec65318f` 与 `6aee1f81` 上逐字相同；`--teleop.left_port` / `--teleop.right_port` 在这两支上不存在（全仓 `git grep` 零命中）。**但本地 fork 的工作区不是这样**：那 6 个未提交改动已给 `examples/alohamini/teleoperate_bi.py` 加上 `--teleop.left_port`（`:57`）与 `--teleop.right_port`（`:63`），默认值按平台取（`:59` / `:65`：macOS 是 `/dev/tty.usbmodem…`、其它平台是 `/dev/am_arm_leader_*`），并把写死的端口改成读参数（`:103` / `:107`）。现场用哪份检出决定有没有这两个参数：先 `git -C <fork> status --porcelain` 看——输出 6 行（6 个未提交改动在）就是工作区口径，有两个参数且默认按平台取；输出 0 行（干净检出）就是提交口径，没有这两个参数。两种情形都不要自己造端口参数：按提交口径现场只能按写死的取值确认设备是否在位：`ls -l /dev/am_arm_leader_left /dev/am_arm_leader_right`，两条都在才谈 teleop；不在位就记「未测」。

冲突与失败：若遥操作连不上 Host、或报端口被占用，把 stderr 原文记下来，先查 Host 是否真的在跑（[§3.4](1-prepare.md#sec-3) 的 5555 / 5556 `LISTEN`）与 `remote_ip` 是否写对；仍不通就记「未测」，**不要改 vendor 代码、不要猜命令**。

物理前提（人工确认项）：**主臂手柄（leader）需在现场**、且需先按 vendor 流程标定过（`docs/alohamini/commands.md:149-157` 说明 Pi 侧的 `alohamini_calibrate` 标的是从臂、不是 Mac 侧主臂）。手柄不在场，整条 teleop 不可执行，直接跳到 M1。

判据：位移 **> 20 tick**（≈ 1.76°）算「能动」。读数取动前 / 动后各一次 `Present_Position`，只读，范式同 [§4.2](3-health.md#sec-4-2)。

二分结论：

| teleop 读数 | 结论 | 下一步 |
| --- | --- | --- |
| 能动（> 20 tick） | 问题在本库的命令语义 | 继续 M1 / M2 |
| 不能动（≤ 20 tick） | 这两轴的伺服 / 机械侧本身不动，**本库路径完全脱身** | 不再往本库代码里找，转伺服 / 机械侧 |

留痕：`stdout/10.5-teleop-$STAMP.{cmd,log,exit}` 三件套 + 一段读数记录（动前 / 动后的 `Present_Position`）。

### M1（有人在场）：`--steps 10` 整轮 10° + `kill -9` 后读寄存器

硬前置（先满足再动手）：**把手臂放到低位，或有人承托**。`kill -9` 之后扭矩保持，但**没有看门狗**——Host 死了不会再兜住下垂，手臂可能重力下垂。风险同 [§8.1](2-host.md#sec-8-1)。

步骤：

1. 发一条整轮命令：`--steps 10` + `--delta-deg 1.0`，即 10 步 × 1°，走的是 [§7.1](#exp-e3) 已跑通的连续路径。关节取 `arm_left_shoulder_pan` 或 `arm_left_shoulder_lift`。**不要试图用 `--delta-deg 10` 一次下发 10°**：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:504`）把幅度钳回 1.0°，`--delta-deg 10` 只会按 1.0° 执行。

   ```bash
   run 10.5-m1 env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 10 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-10.5-m1-$STAMP.csv"
   ```

2. 另开一个终端，在命令起跑后按固定墙钟偏移强杀 Host：

   ```bash
   run 10.5-m1-kill ssh "$PI" 'sleep 3; kill -9 <PID>; echo kill_exit=$?'
   ```

   `<PID>` 取 [§3.2](1-prepare.md#sec-3) 的 `3.2-host-procs` 读到的 Host 进程号。为什么用固定墙钟而不是「等 `.log` 里出现某一行」：`--mode tracking` 在跑动期间不打印逐步进度（`benchmarks/alohamini_io/measure_latency.py` 全文无 `flush=True`，跑动期只有 `:3418-3421` 的横幅，明细要等 `:3555` 的报告），而 [§2](1-prepare.md#sec-2) 的 `run` 把 stdout 重定向进 `.log` 并在命令返回后才 `cat`，所以 `.log` 不是活的监视面。时间预算：`--steps 10`、每步 settle 上限 `2.0 s`，整轮最长约 `20 s`，`sleep 3` 时一定还在跑。若现场改小了 `--settle-timeout-s`，把 `sleep` 按同一比例改小，保持「不超过整轮预算的 1/3」。

3. 立刻读寄存器（Host 已死，串口空闲），读法沿用 [§4.2](3-health.md#sec-4-2) 的只读脚本范式（同一套 `run` 包装、同一套脚本副本落 `$S4DIR/stdout/` 的纪律）。

   ```bash
   run 10.5-m1-regs ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-gripper-regs-$STAMP.py"
   ```

   这份脚本的 `MOTORS` 含 ID1 / ID2 / ID3 / ID6 / ID7，六个要读的寄存器都在其 `REQUIRED_REGS` + `EXTRA_REGS` 里；若 [§4.2](3-health.md#sec-4-2) 当时投的是 18 电机版，用 `/tmp/4.2-all18-left-$STAMP.py`。

**禁用 `SIGTERM`，只用 `kill -9`**：Host 干净退出会走 `disable_torque`（失力），之后读到的 `Torque_Enable` 是退出后的状态，不是运动中的状态——读数是假的。`kill -9` 不给进程处理机会，扭矩保持，读到的才是运动中的状态。

要读的六个寄存器（左总线 ID1 / ID2 / ID6 各一遍）：`Torque_Enable`、`Goal_Position`、`Present_Position`、`Status`、`Present_Temperature`、`Present_Voltage`。

阈值（以下这一组数是默认 `--anchor present` 的口径）：`--steps 10` 整轮 10° 时，每步目标是「当前反馈 + 1°」（约 `measure_latency.py:3149-3168` 每步都重读 `baseline_units` 再 `+delta_deg`），前一步没走到位的部分不带进下一步，因此**最后一步的 `Goal_Position − Present_Position` ≈ 11 tick**（1° ÷ 0.0879°/tick = 11.375，M0 取整 11 tick），**不是累积 110 tick**。`--anchor command` 是另一条下发路径：目标锚在上一步发出的 target 上、lead 跨步累积，同一个读数变成 `steps × \|delta\|` 量级（`--steps 10 --delta-deg 1.0` 时约 110 tick）。读寄存器前先确认这次 run 用的哪个锚点：stdout 有 `note: anchor=command` 那行就是 `command`，没有这行就是默认 `present`（这行只在 `command` 模式打）。

**看门狗会改写 `Goal_Position`，让分支③ 假命中。** Host 在最后一条合法命令之后超过 `watchdog_timeout_ms`（默认 1000 ms）仍没收到新命令时，会把 `last_observation` 的 `*.pos` 当保持位重新下发，并把 `target_source` 置 `watchdog`。此时读到的 `Goal_Position − Present_Position` ≈ 1 tick，远小于 11 tick，**分支③「写没到总线」会假命中**。机理依据：判定式与保持位构造（`hold_action` 取 `last_observation` 的 `.pos`）在 `alohamini_host.py:348-362`（`ec65318f`；`6aee1f81` 为 `:411-425`）、`:357`（`ec65318f`；`6aee1f81` 为 `:420`）置 `target_source`；默认超时 `1000` 在 `config_alohamini.py:110`（两支该行相同，偏移 0）。现场告警原文两行 `WARNING:root:Host command watchdog expired; stopping motion` 在 `stdout/site-host-20261106T074049Z.log` 第 16–17 行，**属 07:40:49Z 起的那个 Host 会话**（装的是 07:40 腕部轮与 07:46 pan 轮），不是「客户端先死」的专用证据。

**首帧 `target_source=watchdog` 不代表本轮客户端先死。** `target_source` 是 Host 侧的持久状态：只在收到合法命令时置 `command`（`alohamini_host.py:330`）、看门狗触发时置 `watchdog`（`:357`）、监督纠偏时置 `protection`（`:373`），新客户端连上本身不改它。上一轮客户端结束（正常退出或被 kill）后 1 s 看门狗即触发，于是「上一轮客户端刚结束、Host 仍在跑」的会话里，新客户端连上的**第一条数据帧**就是 `watchdog`，早于它自己的第一条命令。2026-11-06 实测：`tracking/m1r3-pan-20261106T074620Z-safety.csv` 与 `tracking/slowvsstall-pan-20261106T090635Z-safety.csv` 各有且仅有 18 行 `watchdog`（18 只电机 × 1 帧），全部 `step_index=0`、`sample_sequence=2`，而各轮第一条命令帧在 `sample_sequence=3`；其余 5 份 sidecar（`m1r3-wrist`、`m2-pan`、`slowvsstall-wrist`、`final-rightpan`、`final-punchtest`）`watchdog` 帧数全为 0，首帧都是 `none`。

可执行的三步（读寄存器之前做，做完再决定这一轮能不能用于分支③）：

1. **主判据：时刻先后。** Host 的 `kill` 时刻必须早于客户端的收尾时刻（Host 死时客户端还在跑）。照 `stdout/10.5-m1-verdict-20261106T072606Z.md` 的「起止 UTC 时刻」表格式，把客户端 `kill -9` 与 Host `kill -9` 各自一行 UTC 时刻单列记下。Host 死在客户端之后（客户端先退出或先被 `kill`）→ 该轮作废。2026-11-06 的 07:26 那轮就是客户端先 `kill -9`（`07:26:48Z`）、Host 7 s 后才 `kill -9`（`07:26:55Z`），不符合本判据。
2. **sidecar 检查只在该轮确实有 sidecar 时做。** 有 `tracking/tracking-10.5-m1-$STAMP-safety.csv` 时，只看**最后一帧 `command` 之后**是否出现 `watchdog`，不是「是否出现过」：

   ```bash
   CSV=$S4DIR/tracking/tracking-10.5-m1-$STAMP-safety.csv
   ls -l "$CSV"                     # 无此文件 = 本轮无 sidecar，跳到第 3 步
   LASTCMD=$(tr -d '\r' < "$CSV" | awk -F, 'NR>1 && $9=="command" && $2+0>m+0 {m=$2} END{print m}')
   echo "last command frame: sample_sequence=$LASTCMD"
   tr -d '\r' < "$CSV" | awk -F, -v last="$LASTCMD" 'NR>1 && $9=="watchdog" && $2+0>last+0 {print $2}' | sort -un
   ```

   判据：最后那条 `awk` 空输出 → 过了这一关；打印出任何 `sample_sequence` → 说明该客户端当时还在采样、却已 ≥1 s 没有发出被接受的命令（那几帧的 `target_source` 是 Host 的保持位改写），该轮读数**不能**用于分支③，改判「未落到分支」并重跑。两条实现要点：sidecar 是 CRLF 行尾，`tr -d '\r'` 不能省（不删 `\r` 时 `$9` 永远不等于 `watchdog`，检查会静默「通过」）；`$9` 是 `target_source`、`$2` 是 `sample_sequence`（列义见 [§11](../reference/artifacts.md#sec-11)）。

   这一检查管不了「客户端先死」：sidecar 只记客户端收到的帧，客户端一断采样就不再落行（本仓库 `benchmarks/alohamini_io/measure_latency.py` 的 `_observe` 按新帧计快照，约 `:3120`），所以那种情形在本轮 sidecar 里根本看不到 `watchdog`——主判据只能靠第 1 步。首帧 `watchdog` 之所以也不算：它在第一条命令之前，是上一轮遗留（见上一段），与本轮客户端死活无关。
3. **没有 sidecar 时以 Host 日志 + 时刻先后为准。** 按本节的跑动中 `kill -9` 流程（先杀 Host）时，客户端多半在收尾前就断了，`tracking/tracking-10.5-m1-$STAMP.csv` 与 sidecar 通常都不落盘——2026-11-06 的 07:26 那轮即是：`tracking/` 下没有任何 `tracking-10.5-m1-20261106T072606Z*`，该轮判读自记「无 CSV、无每步日志」（`stdout/10.5-m1-verdict-20261106T072606Z.md`「有效性质疑」段）。这种情况下第 2 步无从执行，判定只靠第 1 步的时刻先后，并查 Host 日志里有无 `Host command watchdog expired` 行（有则该轮视为目标被改写，不得当「写没到总线」的证据）。

三分支判据：

| 分支 | 读数形态 | 结论 | 记录怎么写 |
| --- | --- | --- | --- |
| ① | `Torque_Enable=0` | 该轴在运动过程中未使能（未使能路径） | 写明「未使能」，附 `Torque_Enable` 原文与读的电机 ID |
| ② | `Goal_Position` 前进量达到 **≈ 11 tick**，而 `Present_Position` 不动 | 伺服**收到了目标但不响应**，对应候选根因「伺服保护锁存 / 力矩不足 / 机械」 | 三个数（起点 `Present_Position`、`Goal_Position`、`Present_Position`）逐值抄原文，算出前进量 |
| ③ | `Goal_Position − Present_Position` 远小于 **11 tick**（两个读数近似相等） | 目标**根本没写进总线**（软件侧 / 总线写静默失败） | 写明「写没到总线」，附两个寄存器的原文；**先过上面三步检查**（尤其第 2 步：最后一帧 `command` 之后不得出现 `watchdog`；无 sidecar 时靠第 1 步时刻先后 + Host 日志），否则本轮读数不得用于本分支 |

留痕：`stdout/10.5-m1-$STAMP.{cmd,log,exit}`、`stdout/10.5-m1-kill-$STAMP.{cmd,log,exit}`、`stdout/10.5-m1-regs-$STAMP.log`；寄存器日志里要有六个寄存器的逐值原文与读的电机 ID 清单。

### M2（有人在场）：同轴反向 `--delta-deg -1 --steps 10`

做法：与 M1 逐项镜像，只改两处：

- 加 `--delta-deg -1`（负值自 `321c348` 起合法，幅度仍钳 1°，符号在钳位后保留：`measure_latency.py:2118-2126`）；
- 产物名换成 `10.5-m2-*`。

其余全同：同一关节、`--steps 10`、`--control-hz 50.0`、`--settle-tolerance-deg 0.05`、`--settle-timeout-s 2.0`、`--timeout-s 60`、同样在跑动中对 Host `kill -9`、同样读六个寄存器（ID1 / ID2 / ID6）、同样先满足「手臂放低位或有人承托」的硬前置。

```bash
run 10.5-m2 env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg -1 --steps 10 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-10.5-m2-$STAMP.csv"
```

判据表（M1 与 M2 两轮合起来读；`11 tick` 的阈值对两轮同样适用，只是方向相反）：

| 读数形态 | 结论 |
| --- | --- |
| 正向 0 / 反向正常 | **单向机械**问题（棘轮、单向卡阻类） |
| 双向都 0 | **阈值型**（静摩擦 / 死区阈值把 11 tick 级别的目标吃掉了） |
| 双向都按比例（例如都只走 1–2 tick） | **力矩不足** |
| 双向都动、但**都低于命令量**（例如正向 1.8 tick/步、反向 4.2 tick/步，反向/正向 ≈ 2.33 倍） | **阈值型为主、叠加方向不对称**：既不是「单向机械」（另一向也不达标），也不是严格的「双向都 0 的阈值型」；记录时给出两向的 tick/步与比值，并注明「**不能据此判单向机械阻挡**」 |
| 正向卡死（M1 走不动）+ **反向能动**（M2 位移明显大于 0） | **单向机械阻挡**：缆线绷紧、结构顶住、行程限制三类之一；定性到此完成，转机械检查 |

最后一行是 **2026-11-05 之后项目方提出的机械假设，不是已验证结论**：它的意思是「同一台电机、同一个命令语义，反向能走而正向不走时，优先怀疑单方向的机械阻挡」。要判它成立，M1 与 M2 两轮的寄存器原文都得在盘，且 M2 的位移超过 3 tick（不是 1–2 tick 的抖动量）。

2026-11-06 实测落在新增的「介于两档之间」那一档：左肩正向 15.82%（18.0 / 113.7527 tick，10 步合计 1.8 tick/步）、左肩反向 36.92%（42.0 tick，4.2 tick/步，反向/正向 2.33 倍）、右肩正向 11.43%（13.0 tick），三轮 10/10 步 `settled=False`。判读见 `stdout/10.5-m2r1-verdict-20261106T075623Z.md`（第 1.4、2.2 节）、`stdout/10.5-rightpan-judge-20261106T084419Z.txt`（第 3 节）。这一档只写「阈值型为主、叠加方向不对称」，不得据此判单向机械阻挡。

留痕：`stdout/10.5-m2-$STAMP.{cmd,log,exit}`、`stdout/10.5-m2-kill-$STAMP.*`、`stdout/10.5-m2-regs-$STAMP.log` + 一份主 CSV（沿用 `--mode tracking` 的 tracking CSV 命名，见 [§11](../reference/artifacts.md#sec-11)）。

### 产物名与留痕要求

- 所有产物落 `$S4DIR/stdout/` 与 `$S4DIR/tracking/`，命名 `10.5-<子项>-<STAMP>.<ext>`（子项取 `teleop` / `m1` / `m1-kill` / `m1-regs` / `m2` / `m2-kill` / `m2-regs`）。
- 每步必须有 `.cmd` / `.log` / `.exit` 三件套，写法同 [§2](1-prepare.md#sec-2) 的 `run` 包装。
- 每条结论必须落到本节的分支之一（teleop 的二分、M1 的三分支、M2 的三种形态），并把对应的寄存器原文或 CSV 行号写进记录；没落到分支的记录不算结论。
- 现场若拿到与本文不符的输出形态，按实际输出修正并回写到本小节（沿用本文档原有的回写约定）。

<a id="sec-10-6"></a>
<a id="exp-e6"></a>
## E6 ik 通路与 Gate D（原 §10.6）

| 字段 | 内容 |
| --- | --- |
| 目的 | 回答 `measure_latency.py` 有没有 ik 模式，以及 Gate D 在真机上唯一可能的触发形态 |
| 前置 | 见本卡「安全：本节不动电机」一小节（本卡不动电机） |
| 前置卡 | E5 |
| 后继卡 | E7 |
| 命令 | 本卡无现场命令，见本卡「ik 通路怎么验：不在 `measure_latency.py` 里」一小节 |
| 判据 | 见本卡「Gate D 的定义、唯一触发路径与三处限制」与「现场确认项」两小节 |
| 预期剧本 | 本卡未单列 |
| 失败处置 | [§12](../reference/troubleshooting.md#sec-12) |
| 产物 | 见本卡「现场确认项」表 / [§11](../reference/artifacts.md#sec-11) |

本节是跑法预写，一次真机记录都还没有，判据与预期值未经现场校验。

本节接 [§10.5](#exp-e5) 之后，回答两件事：`measure_latency.py` 里有没有「ik 模式」（结论：没有），以及 Gate D（baseline 反馈缺失或非有限即拒绝）在真机上唯一可能的触发形态、取证边界与替代证据。

### 本轮 brief 的假设与代码不符：`--mode` 只有两个取值

本次写稿的 brief（任务说明）原本假设脚本里有 `ik` 模式。核对源码后没有：`--mode` 的取值集合是写死的二元组（源码原文）：

```python
    parser.add_argument(
        "--mode",
        choices=("rtt", "tracking"),
        default=DEFAULT_MODE,
        help="rtt: request-age only, zero motion; tracking: real robot motion",
    )
```

（`benchmarks/alohamini_io/measure_latency.py:1591-1596`，取值集合就在 `benchmarks/alohamini_io/measure_latency.py:1593`）

| `--mode` 取值 | 一句话人话 | 动机器人 | 走哪套门禁 |
| --- | --- | --- | --- |
| `rtt`（默认） | 只测请求年龄 | 否 | 无运动门禁，Gate A–F 不执行 |
| `tracking` | 单关节阶跃响应 | 是 | Gate A–F（[§7.1](#exp-e3) 的表） |
| `ik`（**不存在**） | — | — | 传进来即参数解析失败，退出码 `2` |

附录 A 的 measure 参数表里 `--mode` 一行也只写 `rtt` 或 `tracking`，与源码一致。结论：「`measure_latency.py --mode ik`」这种写法不存在；现场照 brief 字面去试，拿到的是 argparse 的用法错误（退出码 `2`，不连设备、不开 socket，触发点清单见 [§10](../reference/gates.md#exit-codes) 的「退出码 2 的全部触发点」）。

### Gate D 的定义、唯一触发路径与三处限制

Gate D 判的是 baseline（基线，即第一帧观测里该关节的实测值）是不是一个有限数。判断式与文案（源码原文）：

```python
    if _finite_or_none(observation.get(joint_key)) is None:
        return None, (
            f"Gate D refused: baseline feedback for {joint_key!r} is missing or non-finite; "
            "nothing was commanded."
        )
```

（`benchmarks/alohamini_io/measure_latency.py:2399-2403`；文案本身在 `benchmarks/alohamini_io/measure_latency.py:2401`）模板是 `Gate D refused: baseline feedback for {joint_key!r} is missing or non-finite; nothing was commanded.`，`{joint_key!r}` 会带单引号，现场 stderr 形如：

```
Gate D refused: baseline feedback for 'arm_left_shoulder_pan.pos' is missing or non-finite; nothing was commanded.
```

三处限制，缺一条都触发不到这道门：

| # | 限制 | 依据 |
| --- | --- | --- |
| 1 | 只在 `--mode tracking`：`_tracking_preflight` 只由 tracking 路径调用一次，`rtt` 路径不调 | 定义在 `benchmarks/alohamini_io/measure_latency.py:2353`，唯一调用点在 `benchmarks/alohamini_io/measure_latency.py:3450` |
| 2 | 该关节键必须**在首帧观测里存在**；键不在观测里时，更早的 Gate B 先拒，走不到 D | `benchmarks/alohamini_io/measure_latency.py:2373-2379` |
| 3 | 值必须**不可用**：`float()` 抛 `TypeError` / `ValueError`，或 `not math.isfinite(...)`，都判「缺失或非有限」 | `_finite_or_none` 定义在 `benchmarks/alohamini_io/measure_latency.py:726` |

三种观测形态各落哪道门：

| 首帧观测里 `joint_key` | 值 | 落哪道门 | 退出码 |
| --- | --- | --- | --- |
| 不存在 | — | Gate B（`joint_key not in observation`） | `6` |
| 存在 | 有限数 | 过 D，继续 E / F | — |
| 存在 | `nan` / `inf` / 非数值文本 | **Gate D** | `6` |
| 存在 | `None` | **Gate D**（`float(None)` 抛 `TypeError`） | `6` |

退出码常量是 `EXIT_MOTION_GATE_REFUSED = 6`（`benchmarks/alohamini_io/measure_latency.py:472`），与 [§10](../reference/gates.md#exit-codes) 的「measure 6 = 代码 Gate A–F 拒绝」同一格。

离线复现（本轮实测，一次性探针落 `/tmp`，不入库）：把步进关节的反馈改成 `nan`（键仍在观测里；元数据照 `_TrackingFake` 的形状：`schema_version` 2 加该关节的 `range_min` / `range_max` / `resolution`）、跑一次 `--mode tracking`。

探针原文（整段存成 `/tmp/probe_gate_d.py`，在仓库根执行；下文输出与这份原文同源）：

```python
#!/usr/bin/env python3
"""Gate D 离线复现探针（一次性，落 /tmp，不入库；在仓库根执行）。"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import tempfile
from pathlib import Path

SCRIPT = Path("benchmarks/alohamini_io/measure_latency.py").resolve()
spec = importlib.util.spec_from_file_location("bench_measure_latency", SCRIPT)
latency = importlib.util.module_from_spec(spec)
spec.loader.exec_module(latency)


class NanBaselineFake:
    """步进关节的键仍在观测里，但基线反馈是 nan。"""

    def __init__(self) -> None:
        self.sent_actions: list[dict] = []
        self.latest_robot_metadata = {
            "schema_version": 2,
            "robot_model": "alohamini2pro",
            "motors": {
                "arm_left_shoulder_pan": {"range_min": 1000, "range_max": 3000, "resolution": 4096}
            },
        }

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def get_observation(self, include_cameras: bool = True) -> dict:
        return {"arm_left_shoulder_pan.pos": float("nan")}

    def send_action(self, action: dict) -> dict:
        self.sent_actions.append(dict(action))
        return dict(action)


def main() -> int:
    """跑一次 --mode tracking，打印退出码、发送次数、CSV 与 stderr。"""
    client = NanBaselineFake()
    csv_path = Path(tempfile.mkdtemp()) / "gate-d.csv"
    argv = [
        "--host",
        "127.0.0.1",
        "--mode",
        "tracking",
        "--enable-motion",
        "--joint",
        "arm_left_shoulder_pan",
        "--steps",
        "1",
        "--csv",
        str(csv_path),
    ]
    stderr = io.StringIO()
    with contextlib.redirect_stderr(stderr), contextlib.redirect_stdout(io.StringIO()):
        code = latency.main(argv, client_factory=lambda host, port, model, window: client)
    print(f"exit code: {code}")
    print(f"EXIT_MOTION_GATE_REFUSED: {latency.EXIT_MOTION_GATE_REFUSED}")
    print(f"send_action 次数: {len(client.sent_actions)}")
    print(f"csv 是否落盘: {csv_path.exists()}")
    print("stderr 原文:")
    print(stderr.getvalue().strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

命令与输出原文：

```bash
uv run --no-sync python /tmp/probe_gate_d.py
```

```
exit code: 6
EXIT_MOTION_GATE_REFUSED: 6
send_action 次数: 0
csv 是否落盘: False
stderr 原文:
Gate D refused: baseline feedback for 'arm_left_shoulder_pan.pos' is missing or non-finite; nothing was commanded.
```

这组数字是**离线**结果（假 client、无设备、无网络），不是现场形态。

### 怎么造成 baseline 缺失：没有现成注入路径

要问的是：能不能人为制造 baseline 缺失，好在现场「顺便」把 Gate D 取到。先查代码有没有注入或降级路径：

```bash
grep -n "os.environ\|getenv" benchmarks/alohamini_io/measure_latency.py
grep -n "dry.run\|dry_run" benchmarks/alohamini_io/measure_latency.py
```

两条都零命中（无输出、`exit 1`）。逐项核对：

| 可能的注入路径 | 是否存在 | 依据 |
| --- | --- | --- |
| 一个 `ik` 或 `gate-d` 之类的模式开关 | 无 | `--mode` 只有 `rtt` / `tracking`（`benchmarks/alohamini_io/measure_latency.py:1593`） |
| 环境变量降级 | 无 | 全文件 `os.environ` / `getenv` 零命中 |
| `--dry-run` 之类的干跑开关 | 无 | 全文件 `dry_run` 零命中 |
| 把 `--joint` 指向不存在的键 | 有，但落 Gate B，不是 D | `benchmarks/alohamini_io/measure_latency.py:2373-2379` |

结论：现场唯一合法触发方式是真机 Host 返回该关节的非有限反馈，**不可人为制造**。因此这条门禁的现场形态**只能被动等待**；不得为了触发它去改代码、改脚本或注入假值。等到就记原文（stderr 逐字、退出码、有无 CSV），等不到就写「未触发」，不得写成「通过」或「失败」。

离线证据现状（本轮核查，修正 brief 的一处说法）：仓库里**没有**专门用例断言 Gate D 的拒绝文案。

```bash
grep -rn "Gate D" tests/ --include=*.py
grep -rn "baseline feedback" tests/ --include=*.py
```

第一条命中两处、均为注释或 docstring、无断言锁定该文案：`tests/benchmarks/test_measure_latency.py:33` 是模块 docstring 把 Gate D 列在「覆盖对象」清单里，`tests/test_arm_ik.py:1247` 是一条行内注释（提到「Gate D 2.0 deg metric」）；第二条零命中。所以「Gate D 的拒绝文案已由离线用例锁定」这一说法本轮核查**不成立**，记为遗留项：要锁定得补一条用例（`_TrackingFake` 的该关节取 `nan`，断言退出码 `6` + `send_action` 为 0 + stderr 含 `Gate D`），本轮不改 `tests/`。对照：Gate B / C / E / F 都有专门用例（同文件里的 `test_main_should_refuse_tracking_when_the_joint_is_not_in_the_state_schema`、`test_main_should_refuse_tracking_when_the_host_reports_another_robot_model` 等）。

顺带澄清一处易混：同一套措辞在运行中的另一条路径也出现——每步取基线失败时的中止理由是 `baseline feedback for {joint_key} is missing or non-finite`（`benchmarks/alohamini_io/measure_latency.py:3151`），那属运行时中止（退出码 `1`），不是 Gate D 的 `6`，两者不得互相替代。

### ik 通路怎么验：不在 `measure_latency.py` 里

`measure_latency.py` 全文没有 IK：没有 `placo`、没有 `ArmIK`、没有正向运动学。它的每步目标是「锚点角度 + `--delta-deg`」的算术（`benchmarks/alohamini_io/measure_latency.py:3155-3168`），一次只动一个关节。

「笛卡尔目标 → IK → 六关节角 → 下发」这一段在库侧：`ArmHandle.move_to`（`src/alohamini_cartesian/control/api.py:214`）。要验这一段只能走库 API，跑法见 [§10.7](#exp-e7)。两条通路的对照：

| 项 | `measure_latency --mode tracking` | `ArmHandle.move_to` |
| --- | --- | --- |
| 输入 | 单关节名 + 每步角度增量 | 笛卡尔目标 `(x, y, z)`，姿态恒为单位矩阵 |
| 解算 | 无 IK，直接加减角度 | 每周期 `ArmIK.solve`（`placo`）解六关节 |
| 目标关节数 | 1 | 6 |
| 门禁来源 | `benchmarks/alohamini_io/measure_latency.py` 的 Gate A–F | 库内 step gate、lead 钳位、`JOINT_LIMIT`、follow-error 监督、`move_to` 超时 |
| 产物 | `tracking-*.csv` 与其 sidecar | 调用方自己留痕（[§10.7](#exp-e7) 的脚本写 JSON） |

**两套门禁不同名、不同源，不得互相替代为证据**：tracking 的记录证明不了 `move_to` 的门禁，`move_to` 的记录也证明不了 Gate A–F。

### 安全：本节不动电机

| 动作 | 是否动机器人 | 前置 |
| --- | --- | --- |
| 离线 IK / 预检求解（含 [§10.7](#exp-e7) 的 `--dry-run`） | 否，纯数值 | 无 |
| `measure_latency --mode rtt` | 否 | 现场网络与登录授权 |
| `measure_latency --mode tracking` | **是** | 运动档：`--enable-motion` + [§7.1](#exp-e3) 的开跑勾选单 + 现场运动授权 |
| `ArmHandle.move_to` 真跑 | **是** | 见 [§10.7](#exp-e7) 的「安全前置」 |

本节自身不引入任何运动命令；若现场为取证 Gate D 而跑 `--mode tracking`，按 [§7.1](#exp-e3) 的既有前置与授权走，不另立口径。

### 现场确认项（[§10.6](#exp-e6)）

| # | 确认项 | 怎么确认 | 确认不了怎么办 |
| --- | --- | --- | --- |
| 1 | 不要照 brief 字面试 `--mode ik` | 只要拿到退出码 `2` 就是预设形态，不是故障 | 记「未试」；不要改脚本加模式 |
| 2 | Gate D 的现场形态只能被动等待（无注入路径） | 现场 stderr 出现 `Gate D refused:` 即触发 | 等不到就记「未触发」，判据不因此作废 |
| 3 | Gate D 文案没有离线用例锁定（遗留项） | `grep -rn "Gate D" tests/ --include=*.py` 命中两处，均为注释或 docstring，无断言 | 不在现场补测；交回仓库侧处理 |
| 4 | 真触发到 Gate D 时证据要齐 | stderr 原文 + 退出码 + 是否落 CSV，三件一起存 | 缺任一件则该次不算证据 |
| 5 | tracking 的记录不能当 `move_to` 的证据 | 对照本节「ik 通路怎么验」的表 | 需要 `move_to` 证据时按 [§10.7](#exp-e7) 跑 |

<a id="sec-10-7"></a>
<a id="exp-e7"></a>
## E7 move_to 与笛卡尔路径（原 §10.7）

| 字段 | 内容 |
| --- | --- |
| 目的 | 把库 API `move_to` 跑起来的完整跑法：内嵌驱动脚本、命令、判据与预期值 |
| 前置 | 见本卡「硬前置（先满足再动手）」段、「环境前置：两个 venv 都不够用」与「安全前置（真实运动档）」 |
| 前置卡 | E6 |
| 后继卡 | E8 |
| 命令 | 见本卡「命令（①b 逐字可抄；①a/②/③ 待前置落地）」小节 |
| 判据 | 见本卡「判据表」与「预期值表（两种形态）」两小节 |
| 预期剧本 | 见本卡「预期值表（两种形态）」与「现场最可能的形态」段 |
| 失败处置 | 见 [§12](../reference/troubleshooting.md#sec-12) 与本卡「判据表」的退出码一节 |
| 产物 | 见本卡「命令」小节末段的产物约定 / [§11](../reference/artifacts.md#sec-11) |

本节是跑法预写，一次真机记录都还没有，判据与预期值未经现场校验。

本节接 [§10.6](#exp-e6)：ik 通路只有库 API 这一条路，所以这里给的是「怎么把库 API 跑起来」的完整跑法，而不是某个现成脚本的参数表。

**硬前置（先满足再动手）：本节 ①a / ② / ③ 三条命令的前置当前不满足。** 这三条都强制 `--signs-json` / `--zeros-json`，而 `signs`（六关节 ±1）/ `zeros`（六关节零点，度）的**产出流程见 [§10.8](#exp-e8)（预写，未真机执行）；未标定前 ①a / ② / ③ 不可执行**。仓库侧现状照旧：`tools/` 只有 `verify_unit_consistency.py`（离线核对指纹与标定的单位口径），`benchmarks/` 下两个脚本不产出它们，仓库内的 `.py` 文件对 `signs-json` / `zeros-json` 零命中（`grep -rn "signs-json\|zeros-json" --include=*.py .` 无输出，只有本手册提到这两个选项名）；`examples/robot_dry_run.py:46` 自述「`JointKinematicAdapter` 需要每侧六关节完整的 `signs`/`zeros`，现场没有标定值」。因此：

| 档 | 可执行性 | 依据 |
| --- | --- | --- |
| ①b（纯离线 `--dry-run`） | **可执行**：只用 URDF 与 seed，不读 `signs` / `zeros` | `_run_dry` 只调 `_build_ik` / `_urdf_limits_deg`，不建 adapter、不建 client |
| ①a / ② / ③ | **暂不可执行**：缺 `signs` / `zeros` 就直接按 `15` 退出 | `_load_side_json` 的 `path is None` 分支 |

`signs` / `zeros` 的产出流程已在 [§10.8](#exp-e8) 预写、待现场执行并回写；在这之前这三条记为「**暂不可执行**」，且**不得为了让命令跑起来去手写或猜 `signs` / `zeros`**（那是伪标定，违反 `AGENTS.md` 第 3 节的证据纪律）。本节其余内容（判据表、预期值表、安全前置、现场确认项）仍按预写保留，等 [§10.8](#exp-e8) 现场执行并回写后照原样执行。

### 调用途径判定：仓库里没有现成入口

| 候选入口 | 有什么 | 能不能下发 `move_to` |
| --- | --- | --- |
| `benchmarks/alohamini_io/measure_latency.py` | `--mode rtt` / `--mode tracking` | 不能：两个模式都不碰 IK（[§10.6](#exp-e6)） |
| `tools/` | 只有 `verify_unit_consistency.py`（离线核对指纹与标定的单位口径，只读两个 JSON） | 不能 |
| `examples/minimal_dry_run.py` | 内存假 transport 演示 `move_by` 全链路 | 不能：不连设备 |
| `examples/robot_dry_run.py` | 真 client 注入 + 零发送 dry-run | 不能：不注入 adapter 与 IK 工厂，命令在状态机之前就被拒 |
| `connect(...)` + `ArmHandle.move_to` | 唯一真下发路径 | 能，但仓库里没有调用方脚本 |

结论：最小改动路径 = 本节内嵌的 Python 驱动脚本，落 `/tmp`（照 [§4.2](3-health.md#sec-4-2)「脚本副本落 `$S4DIR/stdout/`」的同一套纪律，不新建仓库文件）。写法照 `examples/robot_dry_run.py` 的姿势：厂商客户端由 `client_factory` 注入，`lerobot` 的 import 只发生在最内层工厂函数体内。「`src/` 不许 import `lerobot`」那条边界只约束 `src/`；驱动脚本作为手册内嵌代码，按 `examples/` 的既有写法注入。

### 环境前置：两个 venv 都不够用（本轮实测）

| 环境 | `lerobot` | `placo` | 能跑哪档 |
| --- | --- | --- | --- |
| 主 `.venv`（`uv run --no-sync python`） | 无 | 有（0.10.1） | ①b 的 `--dry-run`（纯离线） |
| `.venvs/alohamini-lerobot` | `0.6.1` | **无** | 跑不了 ①a / ② / ③（IK 要 `placo`） |

本轮实测命令与输出：

```bash
uv run --no-sync python -c "import importlib.util as u; print('lerobot:', u.find_spec('lerobot'))"
.venvs/alohamini-lerobot/bin/python -c "import placo"
```

```
lerobot: None
ModuleNotFoundError: No module named 'placo'
```

补 `placo` 的两条路，任选一条并留痕（`stdout/10.7-env-*`）：

| 路 | 命令 | 风险与判据 |
| --- | --- | --- |
| A（首选） | `uv pip install --python .venvs/alohamini-lerobot/bin/python "placo==0.10.1"` | 可能挪 `numpy`（该环境现为 `2.2.6`，主 `.venv` 是 `2.3.5`）。装前装后各存一份 `uv pip list`；判据：`import lerobot` 与 `import placo` 同时成功且差异只在预期内，否则回退 |
| B（隔离） | `cp -a .venvs/alohamini-lerobot .venvs/s4-move-to` 后在副本里装 `placo` | vendor 环境零改动；代价是多一份同量级副本（[§5.3](../reference/env-rebuild.md#sec-5-3) 记录过 875 M 这一量级） |

两条都不改仓库文件，也不改 Pi 上的部署。

### 驱动脚本草案（内嵌，不新建文件）

把下面整段存成 `/tmp/s10-7-move-to.py`，后面三条命令都用它。三档：`--dry-run`（纯离线）、`--limits-only`（只读限位窗口）、默认（真跑，必须显式给 `--enable-motion`）。

**留存一份脚本本体**：`/tmp` 不归档，证据目录里只留得住输出、留不住脚本。第一次跑本节前先把它另存一份到证据目录（与 [§4.2](3-health.md#sec-4-2) / [§4.2c](3-health.md#sec-4-2) 的脚本副本纪律一致）：

```bash
mkdir -p "$S4DIR/stdout" && cp -p /tmp/s10-7-move-to.py "$S4DIR/stdout/10.7-script-$STAMP.py"
```

此后每次改了 `/tmp` 里的脚本都要重新覆盖这一份，使 `10.7-script-$STAMP.py` 与当次 `10.7-*.json` 产物同源。判据：该文件在盘，且与 `/tmp/s10-7-move-to.py` 的 `sha256` 相同（`shasum -a 256` 两份对比，两个字串一致）。

- 为什么不造假标定：`signs` / `zeros` 必须来自真实标定，缺文件就按 `15` 退出，脚本不补默认值（见 `AGENTS.md` 第 3 节的证据纪律）。
- 为什么 `--dry-run` 是「构造上零发送」：它不建客户端、不开 socket、不进 `connect`，只做 IK 与预检算术。
- 预检那段是 `_precheck_straight_path`（`src/alohamini_cartesian/control/api.py:158-201`）的**复算**（同样的 1 mm 采样、同样的软姿态权重），不是调用库内私有函数。

```python
#!/usr/bin/env python3
"""§10.7 驱动脚本草案（预写，未真机执行）。

手册 docs/field-ops/runbook/4-experiments.md §10.7 内嵌这份脚本，仓库里不新建文件。三种跑法：

  --dry-run       纯离线：读 URDF 限位、解 IK、逐毫米预检；不建客户端、零网络、零发送
  --limits-only   只读：连真机读 metadata，打印两侧有效限位窗口；dry_run=True，零发送
  默认（真跑）     笛卡尔移动：按 --points 分点调用 ArmHandle.move_by

退出码：0 成功；2 参数解析失败（argparse 自身，或本脚本对 --points / --dz / --timeout-s 的
入参校验，同 §10 既有口径）；10 预检/IK 拒绝；
11 move_to 超时；12 反馈失联；13 运动被阻断；14 运动未启用，或缺 `lerobot`（`ImportError`）；
15 前置缺失（`--signs-json` / `--zeros-json`）。URDF 路径错与 `RuntimeUnavailable` 本脚本
不接，见手册 (a) 表下的两条补充。
10 以上为本节新增、脚本自定义，与既有 0–9 不冲突。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

from alohamini_cartesian.control import (
    FeedbackLost,
    MotionBlocked,
    MotionNotEnabled,
    MotionTimeout,
    OutOfRange,
    connect,
)
from alohamini_cartesian.control.arm_ik import ArmIK, ArmKinematicProfile
from alohamini_cartesian.kinematics.joint_adapter import (
    JOINT_ORDER,
    URDF_JOINT_NAMES,
    JointKinematicAdapter,
)

_EXIT_OK = 0
_EXIT_USAGE = 2
_EXIT_PRECHECK = 10
_EXIT_TIMEOUT = 11
_EXIT_FEEDBACK = 12
_EXIT_BLOCKED = 13
_EXIT_NOT_ENABLED = 14
_EXIT_PREREQ = 15

_URDF_REL = "src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf"
_TARGET_FRAME = "left_Fixed_Jaw"
# 离线核算用的 seed（tests/test_arm_ik.py 的 SOLVE_SEED_Q）；这是假设，不是真机位姿。
_OFFLINE_SEED_DEG = (10.0, -20.0, 30.0, -40.0, 20.0, -10.0)
_ORIENTATION_WEIGHT = 0.01
_PRECHECK_STEP_M = 0.001


def _error(message: str) -> None:
    """把一条说明打到 stderr。"""
    print(message, file=sys.stderr)


def _prereq_error(message: str) -> None:
    """前置缺失：打 stderr 并以 15 退出（不猜、不补默认值）。"""
    _error(message)
    raise SystemExit(_EXIT_PREREQ)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="§10.7 move_to 驱动脚本（预写，未真机执行）")
    parser.add_argument("--host", default="127.0.0.1", help="机器人 Pi 的 IP/主机名")
    parser.add_argument("--robot-model", default="alohamini2pro", help="必须与 Host 侧一致")
    parser.add_argument("--observation-request-window", type=int, default=3)
    parser.add_argument(
        "--side",
        default="left",
        choices=("left", "right"),
        help="哪侧臂；右臂复用左臂 URDF 是库的假设（joint_adapter.py:76-82），不得用左臂值顶替右臂",
    )
    parser.add_argument("--dz", type=float, default=0.09, help="总位移（米），默认 0.09")
    parser.add_argument("--points", type=int, default=1, help="分几点走：1 = 单次 9 cm")
    parser.add_argument("--timeout-s", type=float, default=10.0, help="每次 move_by 的超时")
    parser.add_argument("--urdf", default=_URDF_REL, help="URDF 路径（相对仓库根）")
    parser.add_argument("--q-seed-deg", type=float, nargs=6, default=None)
    parser.add_argument("--signs-json", default=None, help="六关节 sign 的 JSON 文件")
    parser.add_argument("--zeros-json", default=None, help="六关节 zero 的 JSON 文件")
    parser.add_argument("--out", required=True, help="产物 JSON 路径（显式给）")
    parser.add_argument("--dry-run", action="store_true", help="纯离线 IK 与预检，零发送")
    parser.add_argument("--limits-only", action="store_true", help="只读打印真机限位窗口")
    parser.add_argument("--enable-motion", action="store_true", help="真跑必须显式打开")
    return parser.parse_args(argv)


def _urdf_limits_deg(urdf_path: Path) -> dict[str, tuple[float, float]]:
    """从 URDF 读六关节限位（弧度转度），键用 URDF 关节名，与 ArmIK.solve 同口径。"""
    root = ET.parse(urdf_path).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name is None or limit is None:
            continue
        lower = math.degrees(float(limit.get("lower")))
        upper = math.degrees(float(limit.get("upper")))
        limits[name] = (lower, upper)
    return limits


def _build_ik(urdf_path: Path) -> ArmIK:
    """按 am_arm200 左臂模型建 IK 缝（右臂复用同一份左臂模型）。"""
    profile = ArmKinematicProfile(
        urdf_path=str(urdf_path),
        target_frame=_TARGET_FRAME,
        joint_names=tuple(URDF_JOINT_NAMES[joint] for joint in JOINT_ORDER),
    )
    return ArmIK(profile)


def _min_margin_deg(q_deg: np.ndarray, limits: dict[str, tuple[float, float]]) -> tuple[float, str]:
    """返回「最小限位裕度」与那个关节名；裕度 = min(hi - q, q - lo)。"""
    worst, worst_joint = float("inf"), ""
    for index, joint in enumerate(JOINT_ORDER):
        lo, hi = limits[URDF_JOINT_NAMES[joint]]
        value = float(q_deg[index])
        margin = min(hi - value, value - lo)
        if margin < worst:
            worst, worst_joint = margin, joint
    return worst, worst_joint


def _solve_row(
    ik: ArmIK, seed: np.ndarray, target_pose: np.ndarray, limits: dict[str, tuple[float, float]]
) -> dict[str, Any]:
    """解一个目标位姿，返回可 JSON 化的行。"""
    result = ik.solve(
        q_current_deg=seed,
        target_pose=target_pose,
        joint_limits_deg=limits,
        orientation_weight=_ORIENTATION_WEIGHT,
    )
    q = np.asarray(result.q_deg, dtype=float)
    margin, margin_joint = _min_margin_deg(q, limits)
    return {
        "ok": bool(result.ok),
        "reason": result.reason,
        "residual_m": float(result.position_error_m),
        "orientation_error_deg": float(result.orientation_error_deg),
        "q_deg": [float(value) for value in q],
        "min_margin_deg": margin,
        "margin_joint": margin_joint,
    }


def _target(seed_translation: np.ndarray, dz: float) -> np.ndarray:
    """move_to 构造的目标：单位姿态 + 平移量（姿态不是「保持当前姿态」）。"""
    target = np.eye(4, dtype=float)
    target[:3, 3] = np.asarray(seed_translation, dtype=float) + np.array([0.0, 0.0, float(dz)])
    return target


def _run_dry(
    seed: np.ndarray, urdf_path: Path, dz: float, points: int
) -> tuple[int, dict[str, Any]]:
    """纯离线：解整段 9 cm 目标与分点表，并按 1 mm 采样复算预检（不 import 私有函数）。"""
    ik = _build_ik(urdf_path)
    limits = _urdf_limits_deg(urdf_path)
    start = np.asarray(ik.fk(seed)[:3, 3], dtype=float)
    target = _target(start, dz)
    report: dict[str, Any] = {
        "seed_q_deg": [float(v) for v in seed],
        "seed_fk_xyz_m": [float(v) for v in start],
        "target_xyz_m": [float(v) for v in target[:3, 3]],
        "dz_m": float(dz),
        "urdf_limits_deg": {name: [lo, hi] for name, (lo, hi) in limits.items()},
        "single": _solve_row(ik, seed, target, limits),
    }

    end = np.asarray(target[:3, 3], dtype=float)
    delta = end - start
    distance = float(np.linalg.norm(delta))
    n = max(1, math.ceil(distance / _PRECHECK_STEP_M))
    first_failure = None
    for i in range(1, n + 1):
        waypoint = np.eye(4, dtype=float)
        waypoint[0:3, 0:3] = target[0:3, 0:3]
        waypoint[:3, 3] = start + (i / n) * delta
        row = _solve_row(ik, seed, waypoint, limits)
        if first_failure is None and not row["ok"]:
            first_failure = {"i": i, "n": n, **row}
    report["precheck"] = {
        "n": n,
        "step_m": _PRECHECK_STEP_M,
        "distance_m": distance,
        "first_failure": first_failure,
    }

    waypoints = []
    for i in range(1, points + 1):
        row = _solve_row(ik, seed, _target(start, dz * i / points), limits)
        waypoints.append({"i": i, "t": i / points, **row})
    report["waypoints"] = waypoints

    print(f"[离线] URDF = {urdf_path}")
    print(f"[离线] seed 关节角（度）= {report['seed_q_deg']}")
    print(f"[离线] seed FK 末端（米）= {report['seed_fk_xyz_m']}")
    print(f"[离线] 总位移 {dz:.4f} m，目标点（米）= {report['target_xyz_m']}")
    single = report["single"]
    print(
        f"[离线] 整段目标：ok={single['ok']} reason={single['reason']} "
        f"residual_m={single['residual_m']:.6f} 姿态误差={single['orientation_error_deg']:.2f}° "
        f"最小裕度={single['min_margin_deg']:.2f}°（{single['margin_joint']}）"
    )
    print(f"[离线] 整段目标 q_deg = {[round(v, 4) for v in single['q_deg']]}")
    pre = report["precheck"]
    if pre["first_failure"] is None:
        print(f"[离线] 预检 {pre['n']} 点（1 mm 步长）全部通过")
    else:
        failure = pre["first_failure"]
        print(
            f"[离线] 预检第 {failure['i']}/{failure['n']} 点起失败："
            f"reason={failure['reason']} residual_m={failure['residual_m']:.6f}"
        )
    for row in waypoints:
        print(
            f"[离线] 点 {row['i']}/{points} t={row['t']:.4f} ok={row['ok']} "
            f"reason={row['reason']} residual_m={row['residual_m']:.6f} "
            f"最小裕度={row['min_margin_deg']:.2f}°（{row['margin_joint']}）"
        )
    if not single["ok"] or pre["first_failure"] is not None:
        return _EXIT_PRECHECK, report
    return _EXIT_OK, report


def _load_side_json(path: str | None, field: str) -> dict[str, float]:
    """读一个「关节名 → 数值」的 JSON（六关节必须齐全，缺一个就拒，不补默认值）。"""
    if path is None:
        _prereq_error(f"缺少 --{field}s-json：{field} 必须来自真实标定，脚本不造假值")
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    missing = [joint for joint in JOINT_ORDER if joint not in data]
    if missing:
        _prereq_error(f"{path} 缺关节 {missing}")
    return {joint: float(data[joint]) for joint in JOINT_ORDER}


def _build_adapter(
    signs: dict[str, float], zeros: dict[str, float], urdf_path: Path
) -> JointKinematicAdapter:
    """建关节换算适配器：signs/zeros 来自标定，URDF 限位按侧注入。"""
    urdf = _urdf_limits_deg(urdf_path)
    limits_by_side = {
        side: {joint: urdf[URDF_JOINT_NAMES[joint]] for joint in JOINT_ORDER}
        for side in ("left", "right")
    }
    return JointKinematicAdapter(
        signs={side: dict(signs) for side in ("left", "right")},
        zeros={side: dict(zeros) for side in ("left", "right")},
        urdf_limits=limits_by_side,
    )


def _build_client_factory(robot_model: str, window: int):
    """厂商客户端的惰性工厂：lerobot 的 import 只发生在最内层工厂体内。"""

    def factory(host: str, **kwargs: Any) -> Any:
        """按 host 造一个已配置、未连接的 AlohaMiniClient。"""
        from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient
        from lerobot.robots.alohamini.config_alohamini import AlohaMiniClientConfig

        config = AlohaMiniClientConfig(
            remote_ip=host, robot_model=robot_model, observation_request_window=window, **kwargs
        )
        return AlohaMiniClient(config)

    return factory


def _print_limits(session: Any, adapter: JointKinematicAdapter, side: str) -> dict[str, Any]:
    """只读：打印有效限位窗口与 metadata 原文，不发任何动作。"""
    feedback = session.client.get_observation(include_cameras=False)
    metadata = feedback.get("_robot_metadata") or getattr(
        session.client, "latest_robot_metadata", None
    )
    if metadata is None:
        raise FeedbackLost("feedback carries no _robot_metadata")
    motors = metadata.get("motors") or {}
    raw = {
        key: {
            field: motors[key].get(field)
            for field in ("normalization", "range_min", "range_max", "resolution")
        }
        for key in sorted(motors)
        if str(key).startswith("arm_")
    }
    windows: dict[str, Any] = {}
    for known in ("left", "right"):
        try:
            windows[known] = {
                joint: list(window)
                for joint, window in adapter.effective_limits_deg(known, metadata).items()
            }
        except Exception as exc:  # 该侧 metadata 不全就如实记下，不补值
            windows[known] = {"error": f"{type(exc).__name__}: {exc}"}
    print(f"[只读] side={side} 有效限位窗口（度）：")
    for joint, window in windows[side].items():
        print(f"[只读]   {joint}: {window}")
    return {"motor_metadata": raw, "effective_limits_deg": windows}


def _run_limits(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    """只读档：真机 metadata → 限位窗口，dry_run=True，零发送。"""
    urdf_path = Path(args.urdf)
    adapter = _build_adapter(
        _load_side_json(args.signs_json, "sign"),
        _load_side_json(args.zeros_json, "zero"),
        urdf_path,
    )
    session = connect(
        args.host,
        dry_run=True,
        client_factory=_build_client_factory(args.robot_model, args.observation_request_window),
        # 库契约是 ik_factory(side)：两侧复用同一份左臂 URDF，故忽略 side
        ik_factory=lambda _side: _build_ik(urdf_path),
        adapter=adapter,
        clock=time.monotonic,
        sleeper=time.sleep,
    )
    try:
        report = _print_limits(session, adapter, args.side)
    finally:
        session.disconnect()
    return _EXIT_OK, report


def _run_motion(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    """真跑档：先打印限位窗口，再按 --points 分点 move_by，逐点记 residual 与步数。"""
    if not args.enable_motion:
        _error("真跑必须显式给 --enable-motion")
        return _EXIT_NOT_ENABLED, {}
    urdf_path = Path(args.urdf)
    adapter = _build_adapter(
        _load_side_json(args.signs_json, "sign"),
        _load_side_json(args.zeros_json, "zero"),
        urdf_path,
    )
    session = connect(
        args.host,
        dry_run=False,
        enable_motion=True,
        client_factory=_build_client_factory(args.robot_model, args.observation_request_window),
        # 库契约是 ik_factory(side)：两侧复用同一份左臂 URDF，故忽略 side
        ik_factory=lambda _side: _build_ik(urdf_path),
        adapter=adapter,
        clock=time.monotonic,
        sleeper=time.sleep,
        move_timeout_s=args.timeout_s,
    )
    report: dict[str, Any] = {"dz_m": args.dz, "points": args.points, "moves": []}
    code = _EXIT_OK
    try:
        report.update(_print_limits(session, adapter, args.side))
        arm = session.left if args.side == "left" else session.right
        step_m = args.dz / args.points
        for i in range(1, args.points + 1):
            started = time.monotonic()
            try:
                move = arm.move_by(dz=step_m, timeout_s=args.timeout_s)
            except (MotionTimeout, OutOfRange, FeedbackLost, MotionBlocked) as exc:
                if isinstance(exc, MotionTimeout):
                    finish = _EXIT_TIMEOUT
                elif isinstance(exc, OutOfRange):
                    finish = _EXIT_PRECHECK
                elif isinstance(exc, FeedbackLost):
                    finish = _EXIT_FEEDBACK
                else:
                    finish = _EXIT_BLOCKED
                row = {
                    "i": i,
                    "ok": False,
                    "error": type(exc).__name__,
                    "message": str(exc),
                    "elapsed_s": time.monotonic() - started,
                }
                if isinstance(exc, MotionTimeout):
                    row["moved_m"] = getattr(exc, "moved", None)
                report["moves"].append(row)
                print(f"[真跑] 点 {i}/{args.points} {type(exc).__name__}: {exc}")
                code = finish
                break
            row = {
                "i": i,
                "ok": True,
                "residual_m": float(move.residual_m),
                "steps": len(move.steps),
                "elapsed_s": time.monotonic() - started,
            }
            report["moves"].append(row)
            print(
                f"[真跑] 点 {i}/{args.points} 到位 residual_m={row['residual_m']:.4f} "
                f"steps={row['steps']} 耗时={row['elapsed_s']:.2f} s"
            )
    finally:
        session.disconnect()
    return code, report


def main(argv: list[str] | None = None) -> int:
    """入口：按 --dry-run / --limits-only / 默认三档分流。"""
    args = _parse_args(argv)
    if args.points < 1 or not (math.isfinite(args.dz) and math.isfinite(args.timeout_s)):
        _error("--points 必须 >= 1，--dz 与 --timeout-s 必须有限")
        return _EXIT_USAGE

    report: dict[str, Any] = {"argv": sys.argv[1:], "mode": ""}
    try:
        if args.dry_run:
            if args.q_seed_deg is None:
                print(
                    "[离线] 未给 --q-seed-deg，使用离线核算的假设 seed "
                    f"{list(_OFFLINE_SEED_DEG)}（不是真机位姿）"
                )
            seed = np.asarray(
                _OFFLINE_SEED_DEG if args.q_seed_deg is None else args.q_seed_deg, dtype=float
            )
            report["mode"] = "dry-run"
            code, dry = _run_dry(seed, Path(args.urdf), args.dz, args.points)
            report["dry_run"] = dry
        elif args.limits_only:
            report["mode"] = "limits-only"
            code, limits = _run_limits(args)
            report["limits_only"] = limits
        else:
            report["mode"] = "motion"
            code, motion = _run_motion(args)
            report["motion"] = motion
    except MotionNotEnabled as exc:
        _error(f"MotionNotEnabled: {exc}")
        code = _EXIT_NOT_ENABLED
    except ImportError as exc:
        _error(f"ImportError: {exc}（真机档需要 vendor 环境里的 lerobot，见 §10.7 前置）")
        code = _EXIT_NOT_ENABLED
    except FeedbackLost as exc:
        _error(f"FeedbackLost: {exc}")
        code = _EXIT_FEEDBACK

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    report["exit_code"] = code
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[产物] {args.out}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
```

脚本用到的库入口：`connect`（`src/alohamini_cartesian/control/api.py:969`）、`ArmHandle.move_to`（`src/alohamini_cartesian/control/api.py:214`）、`ArmHandle.move_by`（`src/alohamini_cartesian/control/api.py:440`）、`ArmIK.solve`（`src/alohamini_cartesian/control/arm_ik.py:280`）、`JointKinematicAdapter`（`src/alohamini_cartesian/kinematics/joint_adapter.py:127`）。

### 命令（①b 逐字可抄；①a/②/③ 待前置落地）

Host 起法见 [§6.1](2-host.md#host-start)（本节不重启 Host：只要它已在跑）。产物一律显式 `--out`，照 [§3.8](1-prepare.md#sec-3) 的路径纪律。

产物与输入都落 `$S4DIR/stdout/`（`scope-*` 同目录；[§11](../reference/artifacts.md#sec-11) 的五个子目录不变，本节不新增目录）。`signs` / `zeros` 两份 JSON 是**输入**，不是本节产物：它们是六关节的 sign（±1）与 zero（度）映射，只能来自真实标定，**产出流程见 [§10.8](#exp-e8)（预写，未真机执行；本仓库无产出脚本）**，来源见本节「现场确认项」第 6 条；缺文件时脚本按 `15` 退出、不跑、不造假值。

三条真机档命令的 `PYTHONPATH` 比 [§10.5](#exp-e5) 多一截 `:…/src`：本库没装进 vendor 环境（本轮实测 `find_spec('alohamini_cartesian')` 在该环境返回 `None`），而这三档都要 import `alohamini_cartesian`；`measure_latency.py` 只用标准库，所以 [§7.1](#exp-e3) / [§10.5](#exp-e5) 的那些命令不需要这一截。

**①a 只读：真机限位窗口（零发送）**

做什么：连 Host 读一帧观测与 metadata，打印两侧「有效限位窗口」——标定刻度窗口与 URDF 限位的交集（换算口径见 `JointKinematicAdapter.effective_limits_deg`，`src/alohamini_cartesian/kinematics/joint_adapter.py:443`）。
为什么先做：`move_to` 的限位门用的是**标定窗口**，不是 URDF 的 `±179.9087°`（URDF 只是乐观上界），离线推不出真机窗口。

```bash
run 10.7-limits env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src:/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python /tmp/s10-7-move-to.py --host "$PI_HOST" --limits-only --signs-json "$S4DIR/stdout/10.7-signs-$STAMP.json" --zeros-json "$S4DIR/stdout/10.7-zeros-$STAMP.json" --out "$S4DIR/stdout/10.7-limits-$STAMP.json"
```

判据：退出码 `0`；日志里含 `wrist_roll` 的那一行（打印键是 URDF 名 `left_wrist_roll` / `right_wrist_roll`）把 `-163.7°` 包住；两侧六个关节都有窗口（出现 `error:` 即该侧 metadata 不全）。产物：`stdout/10.7-limits-$STAMP.json`，含每台臂电机的 `normalization` / `range_min` / `range_max` / `resolution` 原文。

**①b 离线：IK 解与预检（零网络、零发送）**

```bash
run 10.7-dryrun uv run --no-sync python /tmp/s10-7-move-to.py --dry-run --points 7 --out "$S4DIR/stdout/10.7-dryrun-$STAMP.json"
```

判据：退出码 `0`（整段目标 `ok=True` 且 90 点预检全过）；`整段目标 q_deg` 与本节「预期值表」identity 那行一致。不传 `--q-seed-deg` 时脚本用离线假设 seed（会打印一行提示，明写「不是真机位姿」）；要对齐真机，显式传真机实测的六关节角。产物：`stdout/10.7-dryrun-$STAMP.json`。

**② 9 cm 单次 `move_to`**

标签叫 `move_to`，脚本实际调 `ArmHandle.move_by`（内部算出增量目标后转给 `move_to`，`src/alohamini_cartesian/control/api.py:440` 的文档串写明「The resulting target is then passed to :meth:`move_to`」，`move_to` 在 `:214`）——**`move_by` → `move_to` 同一入口**，两者语义等价。

```bash
run 10.7-single env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src:/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python /tmp/s10-7-move-to.py --host "$PI_HOST" --enable-motion --points 1 --timeout-s 10.0 --signs-json "$S4DIR/stdout/10.7-signs-$STAMP.json" --zeros-json "$S4DIR/stdout/10.7-zeros-$STAMP.json" --out "$S4DIR/stdout/10.7-single-$STAMP.json"
```

**③ 7 点路径**

```bash
run 10.7-path env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src:/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python /tmp/s10-7-move-to.py --host "$PI_HOST" --enable-motion --points 7 --timeout-s 10.0 --signs-json "$S4DIR/stdout/10.7-signs-$STAMP.json" --zeros-json "$S4DIR/stdout/10.7-zeros-$STAMP.json" --out "$S4DIR/stdout/10.7-path-$STAMP.json"
```

②③ 的判据见下面的判据表、安全前置与预期值表；最可能先落到「超时 + `moves` 无成功行」，那是预期形态之一，读法见判据表 (b) 的说明。

**④ 卷尺实测（URDF 锚定，必做）**

② 跑完、`moves` 出现成功行之后就地量一次：量的是「指令 9 cm 到底是不是 9 cm」。这是唯一能发现杆长错的办法——FK/IK 全部锚在 `src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf`（杆长表见 `docs/library-guide.md` §3.1），该 URDF 取自厂商上游、本库零实测（决策见 `docs/ADR.md` D21）。**必做**，不因 ② 跑顺就跳过；③ 只跑路径，不量。

量测口径（末端夹爪尖，URDF 末端连杆 `left_Fixed_Jaw` / `right_Fixed_Jaw`；量哪一侧就在记录里写明哪一侧，`--side` 默认 `left`）：

| 项 | 口径 |
| --- | --- |
| 量什么 | 末端夹爪尖沿竖直方向（基座系 `+Z`）的实际**位移标量**，不是总位移：横向漂移会把这个数放大，横向分量另记一格，不并入 |
| 量点 | 夹爪固定指的针尖（`Fixed_Jaw` 那一侧，不是运动指） |
| 基准点标记 | 跑 ② **之前**先在量点旁立一条钢卷尺或 30 cm 钢直尺（最小刻度 1 mm），用美纹纸把尺固定住；把量点起点在尺上的投影刻度描在美纹纸上。**起点必须落成实物标记**，不得事后凭记忆或凭眼睛补一个起点 |
| 量测方向 | 沿基座系 `+Z`（默认即竖直向上）。立尺时用垂线或角尺校一次竖直，尺面与位移方向平行，读数取量点投影到尺面的刻度 |
| 读数精度 | 读到 1 mm；同一人、同一把尺、同一基准点重复 3 次取中位；3 次之间极差 > 2 mm 视为基准点漂了，重贴美纹纸重标后再量 |
| 什么时候量 | ② 的成功行出现后，等手臂静止 2 s 再量（等 `steady`，避免在动的时候读） |
| 不能量的时候 | ② 落成「超时 + `moves` 无成功行」这类预期形态时**没有位移可量**：记「未测：② 未达成动作，原因「异常类名」」，**不得**拿 FK 预测值回填、不得估读 |

三条数必须同源（同一次 ② 的产物 `stdout/10.7-single-$STAMP.json` + 命令原文 `stdout/10.7-single-$STAMP.cmd`）：

| 项 | 取值 | 来源 |
| --- | --- | --- |
| 指令值 | `90 mm`（`--dz` 默认 `0.09`，② 为 `--points 1`） | `stdout/10.7-single-$STAMP.cmd` 的命令原文 |
| 预测值（参考，不作判据） | `90 mm − residual_m × 1000`，即「指令量减去同一 URDF 下解算的残差」 | `stdout/10.7-single-$STAMP.json` 的 `moves[0].residual_m` |
| 实测值 | 终点刻度 − 起点刻度（mm） | 现场手记，落 `stdout/10.7-tape-$STAMP.md` |
| 差值 | 实测值 − 指令值（mm） | 现场算完写进同一份 `stdout/10.7-tape-$STAMP.md` |

预测值那一行要按它本来的身份读：它由**同一份 URDF** 算出来，与指令值同源，**不是**独立证据；判据只比「实测值 vs 指令值」，预测值只用来解释差值落在哪。

判据：

| 情形 | 口径 | 怎么办 |
| --- | --- | --- |
| 通过 | 绝对值（实测值 − 指令值）≤ 1 mm，即毫米级以内 | 按 [§11](../reference/artifacts.md#sec-11) 归档：`stdout/10.7-tape-$STAMP.md` 与 `10.7-single-$STAMP.json`、`.cmd` 同批存 |
| 不通过 | 差值超 1 mm | 记下读数与差值后**停在这一步**，不改容差、不改判据；`docs/ADR.md` D21 的重新评估条件①（卷尺实测偏差超毫米级）就此触发：回头修 `kinematics.urdf` 的杆长参数，再重跑本节 ② 与 ④ |
| 无效 | 量法不成立（起点未标实物、3 次极差 > 2 mm、未等静止、尺未校竖直） | 记「无效，重做」，**不得**记为通过 |

口径边界（写清，免得被当成更强的证据读）：

- 毫米级是**量法粗口径**：卷尺、单臂、单轴，只覆盖 `+Z` 一个方向，只量末端位置不量姿态（`docs/ADR.md` D21「已知代价」第 1 条）。
- 右臂复用左臂 URDF 是库的假设，本项测的是哪一侧只对哪一侧成立，不能推到另一侧（同 D21「已知代价」第 2 条）。
- 本项通过之前，FK/IK 只作「同一份几何下解算自洽」使用，不作物理精度承诺（D21 决定第 4 条）。
- 本项对应 `TODO.md` §4 的「URDF 杆长零实测」一行；本节只写跑法，不改 `TODO.md` 状态，拿到记录后另行回填。

### 判据表

**(a) 退出码**：`10` 以上本节新增、脚本自定义，与 [§10](../reference/gates.md#exit-codes) 的 `0`–`9` 不冲突。

| 码 | 含义 | 对应异常 / 来源 | 是否可能已下发 |
| --- | --- | --- | --- |
| `0` | 到位（离线档为判据全过） | — | 离线档为零发送 |
| `2` | 参数解析失败 | argparse 自身，或本脚本对 `--points` / `--dz` / `--timeout-s` 的入参校验（`_EXIT_USAGE`），同 [§10](../reference/gates.md#exit-codes) 既有口径 | 否 |
| `10` | 预检 / IK 拒绝 | `OutOfRange`（`src/alohamini_cartesian/control/api.py:199-201`） | 否 |
| `11` | `move_to` 超时 | `MotionTimeout`（`src/alohamini_cartesian/control/api.py:299-302`） | 可能（视 step gate 是否放行过某些周期） |
| `12` | 反馈失联 | `FeedbackLost`（`src/alohamini_cartesian/control/api.py:324-327`） | 可能 |
| `13` | 运动被阻断 | `MotionBlocked`，含 adapter 连续 3 次失败（`src/alohamini_cartesian/control/api.py:387-391`） | 可能 |
| `14` | 运动未启用，或缺 `lerobot` | `MotionNotEnabled`（`src/alohamini_cartesian/control/api.py:1019-1020`）、`ImportError`（vendor 环境无 `lerobot`） | 否 |
| `15` | 前置缺失（`--signs-json` / `--zeros-json` 未给或缺关节） | 脚本自身（`_load_side_json` → `_prereq_error`） | 否 |

两条**不在本表**的逃逸形态（脚本没写 `try/except`，按 Python 默认行为）：

| 形态 | 实际发生什么 | 现场怎么认 |
| --- | --- | --- |
| URDF 路径错（`--urdf` 指错） | 未捕获的 `FileNotFoundError` → traceback + 退出码 **`1`**（`_urdf_limits_deg` 的 `ET.parse`，或 `ArmIK` 构造时报 `URDF not found`） | stderr 有 traceback；**不要**当成 `15` 前置缺失 |
| 未注入 `client_factory`（`RuntimeUnavailable`） | 同样 traceback + 退出码 **`1`**（`src/alohamini_cartesian/control/api.py:962`） | 本脚本总是注入，正常路径不会出现 |

**(b) stdout 与异常特征串**（逐条 grep 过源码原文，不确认的不写）

| 特征串 | 源码位置 | 何时出现 | 现场怎么读 |
| --- | --- | --- | --- |
| `move_to timed out after ` | `src/alohamini_cartesian/control/api.py:300` | 到 `timeout_s` 仍未满足到达条件 | 最常见形态；配合 JSON 里 `moves[].moved_m` 一起读 |
| `no new feedback for ` | `src/alohamini_cartesian/control/api.py:325` | 超过 `feedback_timeout_s` 没有新样本 | 链路问题，不是运动问题 |
| `precheck failed at waypoint ` | `src/alohamini_cartesian/control/api.py:200` | 预检发现不可达或越限 | IK 不可达 → 零下发 |
| `adapter failed (` | `src/alohamini_cartesian/control/api.py:391` | 控制器连续 3 个周期报 `ADAPTER_FAILED` | 先查 `signs` / `zeros` 与 metadata |
| `STEP_LIMIT` / `JOINT_LIMIT` / `FOLLOW_TIMEOUT` / `RESIDUAL` / `IK_FAILED` | `src/alohamini_cartesian/control/controller.py:48-57` | 控制器每周期判 | **不会出现在 stdout**：`move_to` 只把 `ADAPTER_FAILED` 转成异常，其余 `error_code` 不向外传（`src/alohamini_cartesian/control/api.py:387-391`），现场只能从「超时 + 无成功行」反推 |
| `MotionTimeout` / `OutOfRange` / `FeedbackLost` | 异常类（`src/alohamini_cartesian/control/api.py:75-97`） | 见 (a) | 脚本把类名写进 JSON 的 `moves[].error` |
| `MotionBlocked` | 异常类（`src/alohamini_cartesian/control/commands.py:52`） | 见 (a) | 同上：`moves[].error` 写类名 |

**(c) 默认值与到门口径**

| 参数 | 默认值 | 出处 |
| --- | --- | --- |
| `move_timeout_s` | `10.0` s | `src/alohamini_cartesian/control/api.py:608`、`src/alohamini_cartesian/control/api.py:981` |
| `feedback_timeout_s` | `1.0` s | `src/alohamini_cartesian/control/api.py:610`、`src/alohamini_cartesian/control/api.py:982` |
| `arrival_tolerance_m` | `0.005` m | `src/alohamini_cartesian/control/api.py:607`、`src/alohamini_cartesian/control/api.py:980` |
| `precheck_step_m` | `0.001` m | `src/alohamini_cartesian/control/api.py:51`、`src/alohamini_cartesian/control/api.py:609` |
| `control_period_s` | `1/30` s | `src/alohamini_cartesian/control/api.py:606` |
| 到达判定 | `residual <= 0.005` 且 `settled` 且 `valid` | `src/alohamini_cartesian/control/api.py:408-414` |
| step gate（关节步长门禁） | `2.0°` / 周期 | `src/alohamini_cartesian/control/controller.py:133`、`src/alohamini_cartesian/control/controller.py:535-543` |
| lead 钳位 | `30.0°` | `src/alohamini_cartesian/control/api.py:668-670` |
| follow-error 监督 | 容差 `1.0°`、超时 `1.0` s | `src/alohamini_cartesian/control/api.py:668-670`、`src/alohamini_cartesian/control/controller.py:464-480` |

**(d) Gate A–F 在 `move_to` 路径上不生效**

Gate A–F 是 `measure_latency.py` 里的运行时门禁（[§7.1](#exp-e3) 的表），`move_to` 一个都不经过：两者在不同文件、不同入口。`move_to` 路径上的替代物：

| Gate A–F | `move_to` 路径上的对应物 | 依据 |
| --- | --- | --- |
| A 运动开关 + 关节名 | `connect(dry_run=False)` 必须同时 `enable_motion=True`；没有 adapter 时运动命令抛 `MotionNotEnabled` | `src/alohamini_cartesian/control/api.py:1019-1020` |
| B 关节在首帧观测里 | adapter 要求六个关节键齐全且有限，缺一个就抛；`move_to` 把它吞成 `q_current=None`，随后预检被跳过、控制器逐周期报 `ADAPTER_FAILED` | `src/alohamini_cartesian/control/api.py:336-345` |
| C `robot_model` 比对 | **无对应物**：本库不拿宿主 `robot_model` 与任何入参比对 | `grep -rn validate_robot_model src/` 只命中 `model_specs.py` 里的定义与一处内部调用，运动路径上没有调用点 |
| D 基线反馈有限 | 无同名门；非有限反馈在 adapter 的 `observation_to_kinematic_deg` 内就抛：缺键与非有限各一条 `ValueError` → 走 B 行那条路 | `src/alohamini_cartesian/kinematics/joint_adapter.py:511-515`（缺键在 `:511-512`、非有限在 `:514-515`；函数入口 `:469`） |
| E 步长 / 行程上限 | step gate `2.0°` / 周期 + IK 结果的 `JOINT_LIMIT` | `src/alohamini_cartesian/control/controller.py:535-543`、`src/alohamini_cartesian/control/arm_ik.py:471` |
| F 单位换算可用 | adapter 的 metadata 校验分两段：① `schema_version` 必须是整数 `2`；② 六个关节条目必须齐全（逐关节逐个取，缺一个就抛） | ① `src/alohamini_cartesian/kinematics/joint_adapter.py:275-287`（判据在 `:282-283`）；② `effective_limits_deg` 的 `for joint in JOINT_ORDER` 循环 `src/alohamini_cartesian/kinematics/joint_adapter.py:462-467`（循环头在 `:462`），缺条目由 `_motor_entry` 抛，`src/alohamini_cartesian/kinematics/joint_adapter.py:342-343` |

### 安全前置（真实运动档）

真跑前逐条勾选，勾不全不开跑。本节只列 `move_to` 额外的几条，其余与 [§7.1](#exp-e3) 的开跑勾选单并行使用：

- [ ] 现场运动授权到手，写明「哪一侧臂、沿 +Z 9 cm、谁在旁、断电开关在谁手上」，落进 [§3.9](1-prepare.md#sec-3) 的 `scope-*`
- [ ] **手臂低位或有承托**：`move_to` 的姿态目标是单位矩阵（不是保持当前姿态），末端姿态会朝 identity 大幅转
- [ ] 有能按断电开关的人站在机器人旁；远程无人在场时**不跑** ② 与 ③
- [ ] 空场、无负载、另一臂不参与
- [ ] 先跑 ①a（只读）与 ①b（离线 dry-run），两者都过再谈真跑
- [ ] ①a 打印的 `wrist_roll` 窗口覆盖 `-163.7°`；不覆盖就**停**（见 `wrist_roll` 那条）
- [ ] 行程几何核验：工作区范围**不在** `site.yaml`。该文件的键集合由 `src/alohamini_cartesian/config/overlay.py:40` 的 `SETTINGS_FIELDS` 定死（只有 `robot_model`、两个端口、`cameras`、三个运行时参数），未知键直接 `ConfigError`（`src/alohamini_cartesian/config/overlay.py:171-173`），所以离线无法据配置判断目标是否在位形空间内；离线只能给 URDF 几何 + IK 数值收敛，物理上会不会撞桌撞物只能现场目视
- [ ] `wrist_roll ≈ -163.7°` 单独判一次：identity 姿态形态下 7 个点都要求该关节跑到 `-163.6…-163.7°`；离线只有 URDF 的 `±179.9087°` 可用，**真机真实限位来自标定窗口**，离线拿不到。若真机窗口更窄，第一次真跑会直接落 `JOINT_LIMIT`（IK 结果里判，`src/alohamini_cartesian/control/arm_ik.py:471`），所以第一次真跑前必须先只读取一次真机限位窗口（①a）

### 点位从哪来：不是 `site.yaml` 的数值

| 项 | 事实 |
| --- | --- |
| 起点 | **真机当前末端位姿**：`move_by` 每周期用反馈换算关节角、再算 FK，把增量加在实测位姿上（`src/alohamini_cartesian/control/api.py:467-476`） |
| 方向与幅度 | 沿 `+Z` 位移 `0.09 m` 等分 7 点（每点 `0.012857 m`） |
| `+Z` 的方向语义 | `move_by` 的增量先按 `session._r_body_to_arm[side]` 旋转再加到实测位姿；该矩阵默认是单位矩阵（`src/alohamini_cartesian/control/api.py:644-650`），所以默认就是「基座系 +Z」。`site.yaml` 里没有任何坐标系或工作区字段，机身实际朝向与基座系是否同向只能现场确认 |
| 离线核算用的起点 | 假设 seed 关节角 `(10, -20, 30, -40, 20, -10)°`（`tests/test_arm_ik.py:682` 的 `SOLVE_UNREACHABLE_SEED_Q`，经 `tests/test_arm_ik.py:918` 的 `SOLVE_SEED_Q` 引用），FK 末端 `(-0.054681, -0.184418, 0.285184) m`，距基座原点 `0.343991 m`。**这是假设，不是真机位姿** |
| 姿态 | `move_to` 构造的目标是 `np.eye(4)` 再只写平移分量（`src/alohamini_cartesian/control/api.py:267-270`）→ 下发 **identity 姿态**，不是「保持当前姿态」 |

轴的选择理由：`+Z` 与「基座原点 → seed 末端」方向的夹角是六个基座系轴里最小的（34.0°），且 URDF 已剥掉 collision mesh、`placo` 无自碰撞与桌面约束，向上位移在物理上最不容易撞桌。**这是选择理由，不是安全证明。**

### 预期值表（两种形态）

`move_to` 实际用 identity 形态，**现场预期看 identity 那一组**；姿态保持那组只作对照，说明「换成保持姿态会好很多」。本节的脚本只产出 identity 那组（目标姿态写死单位矩阵）。

单点（`+Z` 9 cm）：

| 形态 | `ok` | 残差 (m) | 姿态误差 | 最小限位裕度 | 最紧关节 | 六关节角（度） |
| --- | --- | --- | --- | --- | --- | --- |
| 姿态保持（对照，非 `move_to` 形态） | True | 0.000692 | 0.0° | 115.54° | `elbow_flex` | `[10.0001, -34.1125, 64.3720, -19.7404, 19.9999, -9.9999]` |
| **identity（`move_to` 实际形态）** | True | **0.000913** | 0.02° | **16.23°** | **`wrist_roll`** | `[16.3162, -48.7143, 61.4876, -77.2437, -0.0134, -163.6820]` |

其它轴向 9 cm（identity 形态）：

| 轴向 | `ok` | 残差 (m) | 最小裕度 | 备注 |
| --- | --- | --- | --- | --- |
| `+X` | **False** | 0.144103 | — | `RESIDUAL`；`shoulder_pan` 与 `wrist_yaw` 顶在 `+179.9087°` 面；预检第 78 点起连续失败 13 点 |
| `-X` | True | 0.002478 | 37.96° | — |
| `+Y` | True | 0.000254 | 29.50° | — |
| `-Y` | True | 0.002168 | 11.11° | 裕度最小的一侧 |
| `-Z` | True | 0.004311 | 17.29° | 残差占 `residual_limit_m = 0.005` 的 86% |
| `+Z` | True | 0.000913 | 16.23° | 本节选的轴 |

7 点路径（沿 `+Z` 等分 9 cm，`t = i/7`；identity 形态）：

| 点 | t | `ok` | 残差 (m) | 最小裕度 | 最紧关节 | 六关节角（度） |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0.1429 | True | 0.001153 | 16.29° | `wrist_roll` | `[16.3844, -48.2625, 38.7114, -99.4800, -0.0281, -163.6230]` |
| 2 | 0.2857 | True | 0.001099 | 16.27° | `wrist_roll` | `[16.3628, -47.3334, 41.8074, -95.4861, -0.0233, -163.6414]` |
| 3 | 0.4286 | True | 0.001047 | 16.25° | `wrist_roll` | `[16.3470, -46.8508, 45.1971, -91.6352, -0.0198, -163.6550]` |
| 4 | 0.5714 | True | 0.001000 | 16.24° | `wrist_roll` | `[16.3354, -46.7784, 48.8662, -87.9083, -0.0173, -163.6650]` |
| 5 | 0.7143 | True | 0.000961 | 16.24° | `wrist_roll` | `[16.3269, -47.0828, 52.8057, -84.2833, -0.0155, -163.6725]` |
| 6 | 0.8571 | True | 0.000931 | 16.23° | `wrist_roll` | `[16.3206, -47.7355, 57.0118, -80.7365, -0.0143, -163.6780]` |
| 7 | 1.0000 | True | 0.000913 | 16.23° | `wrist_roll` | `[16.3162, -48.7143, 61.4876, -77.2437, -0.0134, -163.6820]` |

姿态保持形态的 7 点（对照）：7/7 收敛，最大残差 `0.004427 m`（第 4 点，占 `residual_limit_m = 0.005` 的 89%），最小裕度 `115.54°`（出现在第 7 点、关节 `elbow_flex`，与上面的姿态保持单点行同值：路径越往后越紧，终点最紧；7 点逐个为 `143.03°` / `141.32°` / `137.02°` / `132.72°` / `126.26°` / `121.00°` / `115.54°`）。

生产预检网格（1 mm 步长、`n = 90`）：`+Z` 0 点失败；`+X` 第 78 点起连续失败 13 点、终点 `ok=False`。

占位与假设要标清：seed 位姿是**假设**；URDF 的 `±179.9087°` 是**乐观上界**；真机窗口只能由 ①a 取。

**现场最可能的形态（离线可推、未实测）**：identity 形态下第一个点就要把 `wrist_roll` 从 seed 的 `-10°` 跨到 `-163.6°`，跨 `153.6°`；而 step gate 上限是 `2.0°` / 周期（`src/alohamini_cartesian/control/controller.py:133`、`src/alohamini_cartesian/control/controller.py:535-543`）。若真机位姿接近这个假设 seed，②③ 会在第一个周期就被 step gate 拒发、逐周期重复，最后走满 `--timeout-s` 抛 `MotionTimeout`（`src/alohamini_cartesian/control/api.py:300`），JSON 里 `moves` 没有成功行——**这是预期形态，不是故障**。反过来，若真机位姿与假设差很远（例如 `wrist_roll` 本来就在 `-163°` 附近），第一次真跑可能真的动起来。两种结果都要按本节留痕，并按「现场确认项」第 8 条判。

### 现场确认项（[§10.7](#exp-e7)）

| # | 确认项 | 怎么确认 | 确认不了怎么办 |
| --- | --- | --- | --- |
| 1 | 真机当前位姿（六关节角与末端 xyz） | ①a 的 metadata 与 ①b 的 seed 对照；或按 [§4.4](3-health.md#sec-4-4) 的只读观测取 | 拿到后替换 `--q-seed-deg`，离线表才算对真机成立 |
| 2 | 真机真实关节限位窗口（尤其 `wrist_roll` 是否覆盖 `-163.7°`） | ①a 打印的 `effective_limits_deg` | 不覆盖就停，不跑 ②③；记为「形态不可达」 |
| 3 | 工作区范围 | 配置里没有（`site.yaml` 七个键，见 `src/alohamini_cartesian/config/overlay.py:40`） | 只能现场目视与询问；离线不给结论 |
| 4 | `+Z` 向上 9 cm 物理上是否撞物 / 撞桌 | 目视；注意 `placo` 模型已剥掉 collision mesh、无自碰撞与桌面约束 | 有碰撞风险就停，或减小 `--dz` |
| 5 | 9 cm 在 `2.0°` / 周期 step gate 下要走多少周期、能否在 `10 s` 内到位 | 用 ①b 的最大单关节跨度 ÷ `2.0°` × `1/30 s` 估算（identity 形态的 `wrist_roll` 约 `153.6°` → 约 77 个周期 ≈ `2.6 s`，前提是第一步就能被接受） | 估算不能替实测；现场以 `moves[].elapsed_s` 与是否超时为准 |
| 6 | `signs` / `zeros` 的产出流程（**流程见 [§10.8](#exp-e8)，预写未执行；未标定前阻塞 ①a / ② / ③**） | 无离线推导法：只能来自真实标定，仓库里没有产出脚本，产出流程是 [§10.8](#exp-e8) 的预写跑法（见本节节首硬前置）；另：右臂复用同一份左臂 URDF 模型（`src/alohamini_cartesian/kinematics/joint_adapter.py:76-82` 的注释写明这是库的假设） | 产出流程现场执行并回写前，这三档记为「暂不可执行」；**不得**手写或猜这两份 JSON，也不得用左臂值顶替右臂；→ 见 [§10.8](#exp-e8) |
| 7 | Gate D 的现场触发形态 | 见 [§10.6](#exp-e6) 的「现场确认项」 | 同 [§10.6](#exp-e6)：等不到就记「未触发」 |
| 8 | 7 点路径每周期关节步长是否触发 step gate 拒发 | 看 `moves` 是否出现「超时且无成功行」 | 触发了即预期形态；要真正走完需另议分点策略，不在本节范围内 |
| 9 | 卷尺在手（钢卷尺或 30 cm 钢直尺，最小刻度 1 mm）+ 美纹纸 + 垂线或角尺 | 开跑前查实物在场：④ 卷尺实测靠这三样，缺一样判据不成立 | 缺就**不记录 ④**，记「未测：量具不在场」，不得用目测或 FK 预测值代替 |

<a id="sec-10-8"></a>
<a id="exp-e8"></a>
## E8 signs / zeros 标定流程（原 §10.8）

| 字段 | 内容 |
| --- | --- |
| 目的 | 把 `signs`（六关节 ±1）与 `zeros`（六关节零点，度）从「摆一个能用人眼判的姿势」产出来 |
| 前置 | 见本卡「zeros 现场标定」的读数前置与「安全前置」 |
| 前置卡 | E7 |
| 后继卡 | 收尾 5-wrapup.md |
| 命令 | 见本卡「zeros 现场标定」与「sign 判定」两小节 |
| 判据 | 见本卡「自检（注入后跑 FK）」「sign 判定」「现场确认项」三小节 |
| 预期剧本 | 本卡未单列 |
| 失败处置 | [§12](../reference/troubleshooting.md#sec-12) |
| 产物 | 见本卡「产出物与 JSON 形态」小节 / [§11](../reference/artifacts.md#sec-11) |

本节一次真机都没执行过，所有数值都是离线 URDF 计算或公式推导，判据未经现场校验；执行后按实际回写。

本节只解决一件事：`signs`（六关节 ±1）与 `zeros`（六关节零点，度）这两份映射怎么从「摆一个能用人眼判的姿势」产出来。它对应 [§10.7](#exp-e7) 现场确认项第 6 条所指的产出流程，**不代表这两份文件已经存在**：没有现场记录之前，本节只是跑法预写，[§10.7](#exp-e7) 的 ①a / ② / ③ 仍记「暂不可执行」。

### 产出物与 JSON 形态

每侧一份标定文件，六关节齐全：

| 产物 | 路径 | 内容 |
| --- | --- | --- |
| 左臂标定文件 | `$S4DIR/stdout/10.8-calib-left-$STAMP.json` | `side` + `signs` + `zeros` |
| 右臂标定文件 | `$S4DIR/stdout/10.8-calib-right-$STAMP.json` | 同上，`side` 为 `right` |
| 拆分后的 signs（喂 `--signs-json`） | `$S4DIR/stdout/10.8-signs-left-$STAMP.json` / `…-right-…` | 扁平的「关节名 → ±1」 |
| 拆分后的 zeros（喂 `--zeros-json`） | `$S4DIR/stdout/10.8-zeros-left-$STAMP.json` / `…-right-…` | 扁平的「关节名 → 度」 |
| 自检记录 | `$S4DIR/stdout/10.8-selfcheck-$STAMP.md` | 参考位形、读数、算出的 sign/zero、FK 核对结果 |

标定文件形态（**下面每个数字都是占位，示例形态，非实测值**；不得照抄，也不得把它当标定结果）：

```json
{
  "side": "left",
  "signs": {
    "shoulder_pan": 1,
    "shoulder_lift": -1,
    "elbow_flex": 1,
    "wrist_flex": -1,
    "wrist_yaw": 1,
    "wrist_roll": -1
  },
  "zeros": {
    "shoulder_pan": -12.3,
    "shoulder_lift": 41.0,
    "elbow_flex": -8.5,
    "wrist_flex": 3.2,
    "wrist_yaw": -67.4,
    "wrist_roll": 0.0
  }
}
```

适配器（决定这两份文件怎么被消费）的契约：

| 项 | 契约 | 出处 |
| --- | --- | --- |
| 正向（观测 → 运动学角） | `q_kinematic_deg = sign × motor_deg + zero` | `src/alohamini_cartesian/kinematics/joint_adapter.py:26` |
| 反向（运动学角 → 命令） | `motor_deg = (q_kinematic_deg − zero) / sign`，实现写作 `sign × (q − zero)` | `src/alohamini_cartesian/kinematics/joint_adapter.py:31`、`src/alohamini_cartesian/kinematics/joint_adapter.py:635` |
| `sign` 取值 | 只许 `+1` / `-1`；其他数值、`bool`、字符串在构造时 fail-fast | `src/alohamini_cartesian/kinematics/joint_adapter.py:206` |
| `zero` 取值 | 有限数、单位度；在正向式里是**最后一步的加法项**，反向式先减 zero 再乘 sign。非有限在构造时 fail-fast | `src/alohamini_cartesian/kinematics/joint_adapter.py:212` |
| 组织形态 | `side -> joint -> 值`，两侧各注入一份，构造时传入 | `src/alohamini_cartesian/kinematics/joint_adapter.py:142` |
| 库内默认值 | **没有**。不注入就构造不出适配器；本库不提供「猜一份」的退路 | `src/alohamini_cartesian/kinematics/joint_adapter.py:147` |
| 缺文件时现场行为 | [§10.7](#exp-e7) 驱动脚本以 `15` 退出（`_load_side_json` 的前置拒绝，六关节缺一也走这条） | `docs/field-ops/runbook/4-experiments.md:1064`、`docs/field-ops/runbook/4-experiments.md:1386` |
| 双向消费 | 同一份映射同时用于观测（units→deg）与命令（deg→units），命令区间闸门也用它 | `src/alohamini_cartesian/kinematics/joint_adapter.py:510`、`src/alohamini_cartesian/kinematics/joint_adapter.py:617`、`src/alohamini_cartesian/kinematics/joint_adapter.py:443` |
| 关节名 | 六关节固定顺序 `shoulder_pan` / `shoulder_lift` / `elbow_flex` / `wrist_flex` / `wrist_yaw` / `wrist_roll` | `src/alohamini_cartesian/kinematics/joint_adapter.py:67` |
| 右臂关节名 | 电机键是 `arm_right_<joint>.pos`，**URDF 关节名仍是 `left_*`**（右臂复用左臂模型，不是「右臂 = 左臂」的值假设） | `src/alohamini_cartesian/kinematics/joint_adapter.py:76` |

与 firmware 字段的区别（两条不同层的换算，**不得混算**）：

| 项 | 适配器的 `signs` / `zeros`（本节产出的两份文件） | firmware 的 `homing_offset` / `drive_mode` |
| --- | --- | --- |
| 量纲 | `sign` 无量纲（±1）；`zero` 单位**度** | `homing_offset` 单位 **tick**；`drive_mode` 是 `0` / `1` 的反向开关 |
| 谁消费 | 本库 `JointKinematicAdapter`，构造时注入 | 厂商 `MotorsBus` 的归一化 / 反归一化链 |
| 本库是否重放 | 不适用（本库自己的映射） | `homing_offset` **从不重放**（源码原文见 `src/alohamini_cartesian/kinematics/joint_adapter.py:51`）；`drive_mode` 只在 `range_m100_100` 模式里按厂商公式取反 |
| 混算禁令 | 不得把 `homing_offset`（tick）直接当 `zero`（度）；不得用 `drive_mode` 反推 `sign`。两者所在的换算层不同，单位也不同，混用会得到量级都错的映射 | 同左 |

拆分与侧别（脚本侧的坑，必须按这条传）：

```bash
jq '.signs' "$S4DIR/stdout/10.8-calib-left-$STAMP.json" > "$S4DIR/stdout/10.8-signs-left-$STAMP.json"
jq '.zeros' "$S4DIR/stdout/10.8-calib-left-$STAMP.json" > "$S4DIR/stdout/10.8-zeros-left-$STAMP.json"
jq -e 'keys | length == 6' "$S4DIR/stdout/10.8-signs-left-$STAMP.json"
jq -e 'keys | length == 6' "$S4DIR/stdout/10.8-zeros-left-$STAMP.json"
```

- 拆分后必须是**扁平**的「关节名 → 数值」；`_load_side_json` 读的就是扁平形态（[§10.8](#exp-e8) 内嵌那份自检脚本），多包一层 `signs` / `zeros` 外层键会被当成「缺关节」并以 `15` 退出。
- 现场没有 `jq` 时用别的工具拆也行，判据只有一条：拆出来的文件是扁平六键，值类型是数值。本节只定产物形态，不定工具。
- [§10.7](#exp-e7) 驱动脚本的 `_build_adapter`（`docs/field-ops/runbook/4-experiments.md:1083`）把**同一份** `signs` / `zeros` 同时注入 `left` 与 `right` —— 这是脚本的简化。所以**右臂真跑必须传右臂自己的文件**：`--signs-json …/10.8-signs-right-$STAMP.json --zeros-json …/10.8-zeros-right-$STAMP.json`。**不得**把左臂文件传给右臂命令，也不得靠这次简化让右臂「借用」左臂值。

### 参考位形（离线 URDF FK 计算，未经实测）

做法：`uv run --no-sync python`（主 `.venv` 有 `placo` 0.10.1）加载 `src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf`（只读），关节名与 `ArmIK.fk` 同口径、末端取 `left_Fixed_Jaw` 原点。计算脚本落 `/tmp`，不落仓。下面的数都是**离线 URDF FK 计算，未经实测**。

全零位形（六关节全 0°）：

| link | x (m) | y (m) | z (m) |
| --- | --- | --- | --- |
| `left_Base` | 0.000000 | 0.000000 | 0.000000 |
| `left_Rotation_Pitch` | 0.000000 | 0.000000 | +0.062556 |
| `left_Upper_Arm` | 0.000000 | −0.029035 | +0.120779 |
| `left_Lower_Arm` | 0.000000 | +0.099479 | +0.161950 |
| `left_Wrist_Pitch_Roll` | 0.000000 | −0.098949 | +0.158745 |
| `left_wrist_yaw` | −0.000966 | −0.127079 | +0.159661 |
| `left_Fixed_Jaw`（末端） | −0.000966 | −0.188071 | +0.135422 |

零位形下各杆段（段 = 相邻两关节轴原点连线）：

| 段 | 长度 (m) | 单位方向 | 最接近的世界轴 |
| --- | --- | --- | --- |
| 大臂段（`left_Upper_Arm` → `left_Lower_Arm`） | 0.134948 | (0.000004, 0.952325, 0.305086) | `+Y`，偏 17.76° |
| 小臂段（`left_Lower_Arm` → `left_Wrist_Pitch_Roll`） | 0.198454 | (0.000000, −0.999870, −0.016148) | `−Y`，偏 0.93° |
| 腕段（`left_Wrist_Pitch_Roll` → `left_wrist_yaw`） | 0.028161 | (−0.034316, −0.998882, 0.032517) | `−Y`，偏 2.71° |
| 夹爪段（`left_wrist_yaw` → `left_Fixed_Jaw`） | 0.065632 | (−0.000001, −0.929307, −0.369307) | `−Y`，偏 21.67° |

零位形末端距 `left_Base` 原点 0.231756 m。**零位形不适合当参考位形**：大臂段本身就与 `+Y` 差 17.76°（URDF 的 elbow origin 带 −0.041171 m 的 x 分量），肉眼判「正不正」判不准。

候选姿势（腕三轴全 0；候选即扫描 `shoulder_lift` / `elbow_flex` 找「杆段贴世界轴」的整数角）：

| 候选 | q_target（`pan`, `lift`, `elbow`, `w_flex`, `w_yaw`, `w_roll`）（度） | 大臂段贴轴 | 小臂段贴轴 | 末端 (x, y, z) m | 距基座原点 m | 判读 |
| --- | --- | --- | --- | --- | --- | --- |
| C0 零位形 | 0, 0, 0, 0, 0, 0 | `+Y` 偏 17.76° | `−Y` 偏 0.93° | (−0.000966, −0.188071, +0.135422) | 0.231756 | 大臂不贴轴，不选 |
| C1 整臂竖直向上伸直 | 0, −72, +163, 0, 0, 0 | `+Z` 偏 0.24° | `+Z` 偏 0.07° | (−0.000965, −0.049983, +0.543696) | 0.545989 | 两段同轴，只剩一个竖直判据（绕轴自旋判不出），且 163° 贴 ±180° 限位面 |
| C2 大臂竖直 + 小臂水平朝 `+Y` | 0, −72, −107, 0, 0, 0 | `+Z` 偏 0.24° | `+Y` 偏 0.07° | (−0.000966, +0.259492, +0.277231) | 0.379729 | 两条判据各一次；`−Y` 侧有障碍时的备选 |
| **C2b 大臂竖直 + 小臂水平朝 `−Y`（选定）** | **0, −72, +73, 0, 0, 0** | `+Z` 偏 0.24° | `−Y` 偏 0.07° | **(−0.000966, −0.316448, +0.234221)** | **0.393700** | 两条判据各一次，且小臂朝模型零位末端所在的一侧（离机身本体远） |
| C3 大臂竖直向下 + 小臂水平朝 `+Y` | 0, +108, +73, 0, 0, 0 | `−Z` 偏 0.24° | `+Y` 偏 0.07° | (−0.000967, +0.258378, +0.007337) | 0.258483 | 末端只高出基座平面 7 mm，桌面 / 底座碰撞风险，不选 |
| C4 整臂竖直向下伸直 | 0, +108, +162, 0, 0, 0 | `−Z` 偏 0.24° | `−Z` 偏 0.93° | (−0.000968, −0.003065, −0.301719) | 0.301736 | 末端在基座平面下 0.302 m，物理上不成立，不选 |

选 C2b 的理由：两条杆段各贴一条世界轴（偏 0.24° / 0.07°），一条垂线校竖直、一把角尺校水平就够；末端相对基座的关系简单（上方 0.234 m、正前方 0.316 m、侧向 0.001 m），便于后面的量级核对。C1 / C4 用同轴判据只能判「两者共线且竖直」，判不出绕轴自旋；C3 的末端高度不可接受。

C2b 的逐关节目标角与目视判据（**目标角是 URDF 运动学角，不是电机角**；电机角要现场读）：

| 关节（URDF 名 / 手册关节名） | q_target（度） | 目视判据（用杆长关系描述） |
| --- | --- | --- |
| `left_shoulder_pan` / `shoulder_pan` | 0 | 整个手臂落在机身中线所在的竖直平面内：从正上方看，小臂指向「正前方」，不左右偏 |
| `left_shoulder_lift` / `shoulder_lift` | −72 | **大臂段**（shoulder_lift 轴 → elbow 轴，长 **0.135 m**）沿重力反方向竖直向上：垂线贴在大臂侧面，上下两端都贴住即竖直 |
| `left_elbow_flex` / `elbow_flex` | +73 | **小臂段**（elbow 轴 → wrist_flex 轴，长 **0.198 m**）水平：角尺一边贴小臂、另一边竖直（与上一条的垂线平行），两段成 90° |
| `left_wrist_flex` / `wrist_flex` | 0 | 腕段（0.028 m）与小臂同轴（离线 FK 下偏 3.47°）：目视「腕部跟管状小臂一条线」 |
| `left_wrist_yaw_joint` / `wrist_yaw` | 0 | 夹爪沿小臂延长线朝前：离线 FK 下夹爪 link 的 z 轴指 `−Y`，与水平小臂同向 |
| `left_wrist_roll` / `wrist_roll` | 0 | 夹爪不自旋：离线 FK 下夹爪 link 的 x 轴指 `−X`，即张口左右开。**该轴位置判据完全退化**（+1° 步末端位移 0.000 mm，见下节灵敏度表），只能看张口朝向 |

C2b 的离线 FK 结果：末端 `left_Fixed_Jaw` = **(−0.000966, −0.316448, +0.234221) m**，距 `left_Base` 原点 **0.393700 m**（离线 URDF FK 计算，未经实测）。杆长口径：大臂段 0.134948 m、小臂段 0.198454 m，与 `docs/library-guide.md` §3.1 的关键杆长表同源。

两条口径边界，别读过头：

- 这些位置值只说明「同一份 URDF 下自洽」。URDF 杆长**零实测**（`TODO.md` §4 那行），所以它们不是物理精度承诺。
- `shoulder_lift = −72°` / `elbow_flex = +73°` 是否在真机可达窗口内，离线判不了（URDF 的 ±180° 只是乐观上界），属「现场确认项（[§10.8](#exp-e8)）」第 1 条。

### zeros 现场标定

前置：手臂已按上节的 C2b 摆到位（摆姿方式与安全见「安全前置」），本轮授权含只读观测。

| 步 | 做什么 | 判据 / 依据 |
| --- | --- | --- |
| 1 | 摆姿：把六关节摆到 C2b 的形状（大臂竖直向上、小臂水平朝正前方、腕部顺直） | 上节逐关节目视判据；摆完人不要碰 |
| 2 | 读数：按 [§4.1](3-health.md#sec-4-1) 采一份 `capture` 指纹，`observation_state` 里有每个电机的 `<motor>.pos`（**归一化值**，键形如 `arm_left_shoulder_pan.pos`） | [§4.1](3-health.md#sec-4-1) 的判据（exit 0、`robot_metadata.schema_version == 2`） |
| 3 | 归一化 → tick：按 [§4.1c](3-health.md#sec-4-1c) 的同一条换算 `int(((v + 100) / 200) × (max − min) + min)`，`range_0_100` 用 `int((v / 100) × (max − min) + min)`；`max` / `min` 取该电机 metadata 的 `range_max` / `range_min` | `docs/field-ops/runbook/3-health.md:179`、`docs/field-ops/runbook/3-health.md:273` |
| 4 | tick → 度：`motor_deg = (tick − mid) × 360 / (resolution − 1)`，`mid = (range_min + range_max) / 2` | 这是库观测侧的原式（`src/alohamini_cartesian/kinematics/joint_adapter.py:530`）。口算口径 **1 tick ≈ 0.0879° = 360/4096**（用 `resolution − 1 = 4095` 时 360/4095 = 0.08792°，差 0.02%，判读时忽略） |
| 5 | 算 zero：`zero = q_target − sign × motor_deg_read`，每关节一次 | 适配器正向式的移项（`src/alohamini_cartesian/kinematics/joint_adapter.py:26`） |

读数链一句话版：**归一化值 → tick（[§4.1c](3-health.md#sec-4-1c) 同式）→ 度（1 tick ≈ 0.0879° = 360/4096）→ 代入 `zero = q_target − sign × motor_deg_read`**。

填表（本次为空白模板，不是实测值；按 `AGENTS.md` §3 缺失口径留空，**不得**填 0）：

| 关节 | q_target（度，C2b） | motor_deg_read（度） | sign | zero（度） |
| --- | --- | --- | --- | --- |
| `shoulder_pan` | 0 | （未测） | （未定） | （未测） |
| `shoulder_lift` | −72 | （未测） | （未定） | （未测） |
| `elbow_flex` | +73 | （未测） | （未定） | （未测） |
| `wrist_flex` | 0 | （未测） | （未定） | （未测） |
| `wrist_yaw` | 0 | （未测） | （未定） | （未测） |
| `wrist_roll` | 0 | （未测） | （未定） | （未测） |

**sign 未定之前这张表算不出来**，所以顺序是：先摆姿 + 读数（本节）→ 再定 sign（下一节）→ 再回填 zero。读数与摆姿必须同一次：姿势一动，`motor_deg_read` 就作废。

为什么不能先假定 sign：

- **现场确认项：左臂 sign 现值在证据目录与 configs 中均无记录。** 已查 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/` 下 `s4-20261106/configs`、`s4-20261107/configs`、`s4-20261108/configs` 三个目录**全空**，全目录 `grep -E "sign|zero"` **无命中**。所以没有可继承的起始值，也没有历史对照可以「先验一下」。
- 因此：**首次标定时先用 ±1° 单步法定 sign**（下一节），不得凭「机械上应该同向」之类的推断先填 ±1。

### sign 判定

做法：在参考位形（或任何安全位形）上，单轴发 **+1°** 小步，看**该轴自身的实测角**是增还是减——增则 `sign = +1`，减则 `sign = −1`（符号相对 URDF 正方向）。

命令形态沿用 [§7.1](#exp-e3) / [§10.5](#exp-e5) 已验证过的 `measure_latency.py --mode tracking`（`--joint <电机键> --delta-deg 1.0 --steps 1`，其余参数与 [§7.1](#exp-e3) 那张参数表一致；不要照 [§10.5](#exp-e5) 的 M1/M2 那样跑 10 步，本步只要一个方向信号）。看的是 tracking CSV 里该轴 `measured_deg` 相对基线是增还是减（[§11](../reference/artifacts.md#sec-11) 的 tracking CSV 列口径），不是肉眼看位移。

单轴 +1° 的末端位移（离线 FK，C2b 位形）——决定「为什么不能靠肉眼判方向」：

| 关节 | +1° 时末端原点位移 (mm) | +1° 时末端姿态转角 (度) |
| --- | --- | --- |
| `shoulder_pan` | 5.523 | 1.000 |
| `shoulder_lift` | 5.393 | 1.000 |
| `elbow_flex` | 5.040 | 1.000 |
| `wrist_flex` | 1.608 | 1.000 |
| `wrist_yaw` | 1.065 | 1.000 |
| `wrist_roll` | **0.000** | 1.000 |

`wrist_roll` 的 1° 步在末端位置上量不出来（0.000 mm，主轴穿过末端原点），若只看位移会把「动了」误判成「没动」甚至误判成「反向」。所以六轴的判据统一为**读该轴观测角的增减**，不读末端位移。

填表（本次为空白模板，不是实测值）：

| # | 关节 | 实测方向（增 / 减） | sign（+1 / −1） | 复测是否一致（第二次独立读数） |
| --- | --- | --- | --- | --- |
| 1 | `shoulder_pan` | （未测） | （未定） | （未测） |
| 2 | `shoulder_lift` | （未测） | （未定） | （未测） |
| 3 | `elbow_flex` | （未测） | （未定） | （未测） |
| 4 | `wrist_flex` | （未测） | （未定） | （未测） |
| 5 | `wrist_yaw` | （未测） | （未定） | （未测） |
| 6 | `wrist_roll` | （未测） | （未定） | （未测） |

与 zeros 的交叉校验（sign 判错时 zero 会系统性偏「两倍 `motor_deg_read`」，用它当自检）：

- 记 `m = motor_deg_read`（同一姿势、同一轴）。sign 取 `+1` 时 `zero = q_target − m`；取 `−1` 时 `zero = q_target + m`。两者之差恒为 `2m`。
- 于是：**同一姿势下按两种候选 sign 各算一次 zero，差值必须恰好等于 `2 × motor_deg_read`**（数值上应吻合到 0.01° 量级）。不吻合说明两次用的不是同一姿势的读数（或 read 值抄错），先回去重采，不要先改 sign。
- 算例（占位数字，示例形态）：若 `q_target = −72°`、`m = +41.0°`，则 `sign = +1` 给 `zero = −113.0°`、`sign = −1` 给 `zero = −31.0°`，差 `82.0° = 2 × 41.0°`。
- 反向读法：若现场最后算出的 zero 大得离谱（例如某个轴的 zero 绝对值超过 180°），先怀疑 sign 判反了，而不是先怀疑读数。

### 自检（注入后跑 FK）

两道闸，都过才算自检完成：

| 闸 | 做什么 | 判据 |
| --- | --- | --- |
| (a) 构造闸 | 用拆好的四份文件喂 [§10.7](#exp-e7) 的 ①a（`--limits-only`，只读、零发送） | 退出码 `0`，两侧六个关节都有窗口。sign 非 ±1、六关节缺一、文件没拆分，都在这一步暴露（`15` 或缺窗口） |
| (b) 数值闸 | 同一姿势的读数 + 标定文件 → 按适配器观测侧公式算 `q_kinematic_deg`（`src/alohamini_cartesian/kinematics/joint_adapter.py:534`）→ 与 C2b 的 `q_target` 比 → 再用 URDF 的 FK 算末端位置 | `q_kinematic_deg` 与 `q_target` 之差 ≤ 3°（目视量级）；末端位置落进下表口径 |

末端量级核对表（期望值 = C2b 的离线 FK；量具是卷尺 / 角尺 / 目视，**不是毫米级判据**）：

| 项 | 期望值（离线 FK，未经实测） | 允许偏差口径 | 不符时先查什么 |
| --- | --- | --- | --- |
| 末端高度（基座系 `+Z`） | +0.234 m | ±0.05 m 量级 | 先查大臂段是否真竖直（C2b 判据 2）；再查 `shoulder_lift` / `elbow_flex` 的 zero 是否用同一姿势的读数算的 |
| 末端水平距离（正前方） | 0.316 m | ±0.05 m 量级 | 先查小臂段是否真水平；再查 `elbow_flex` 的 sign 是否判反（判反会让 zero 偏 `2m`） |
| 末端侧向偏移（左右） | 0.001 m（≈ 0） | ≤ 0.03 m | 基本只由 `shoulder_pan` 的 zero 决定：pan 零位偏了，整条臂绕竖轴转，侧向最先偏离 |
| 末端距基座原点 | 0.394 m | ±0.05 m 量级 | 上面三项都对但这一项不对，说明量点不是 `left_Fixed_Jaw`（末端连杆原点），先统一量点 |

±0.05 m 这个口径的来由：离线算「六关节同时偏 +3°」→ 末端位移 32.78 mm、「同时偏 −3°」→ 32.70 mm。目视判姿到位的精度就是几度量级，所以核对口径只能是分米里的半格，**不是毫米级**。毫米级判据是 [§10.7](#exp-e7) 的 ④ 卷尺实测项，本表替代不了它。

本项通过只说明「sign / zero 与参考位形自洽、量级对得上」，不构成 FK 绝对精度证据：几何仍锚在零实测的 URDF 上。

### 右臂流程

右臂把「参考位形 → zeros → sign 判定 → 自检」四步**独立**跑一遍：独立摆姿（右臂自己摆到 C2b 的形状）、独立读数（`arm_right_*` 键）、独立定 sign（六轴各一次单步）、独立算 zero、独立自检。

| 允许 | 禁止 |
| --- | --- |
| 参考位形用同一套目标角（C2b 是 URDF 几何，两侧同模型） | 直接把左臂的 `signs` / `zeros` 抄进右臂文件 |
| 用 sign 判定法实测确认某轴与左臂一致后，沿用该轴的左臂值 | 以「同型号机械、应该一样」为理由沿用，不附实测 |
| 右臂沿用左臂的 URDF 模型（库的设计，`src/alohamini_cartesian/kinematics/joint_adapter.py:76`） | 把「模型复用」当成「标定值复用」——两者不是一回事 |

例外条款只有一条：若某轴右臂实测 sign 与左臂一致，可以在右臂标定文件里写同值，但**证据里必须写明「实测一致」并附两次独立读数**（左臂一次、右臂一次，各自带 STAMP 与电机键）。没有这两份读数就沿用 = 伪标定。

侧别相关的三处必须对上，缺一处就把左臂文件送进了右臂命令：

| 位置 | 右臂取值 |
| --- | --- |
| 标定文件 `side` 字段 | `"right"` |
| 拆分文件名 | `10.8-signs-right-$STAMP.json` / `10.8-zeros-right-$STAMP.json` |
| [§10.7](#exp-e7) 命令参数 | `--signs-json` / `--zeros-json` 指向上面那两份，必要时再显式给 `--side right` |

### 安全前置

| 项 | 要求 | 出处 |
| --- | --- | --- |
| Host 启停 | 按 [§6.1](2-host.md#host-start) 的三步法启、按 [§8.1](2-host.md#sec-8-1) 停；**停 Host 不等于失力**，摆姿前先确认扭矩状态 | [§6.1](2-host.md#host-start)、[§8.1](2-host.md#sec-8-1) |
| 单轴单次 ≤ 2° | sign 判定用 **1°** 小步（`--delta-deg 1.0 --steps 1`），且全程不超过 step gate 的 `max_joint_step_deg = 2.0` | `src/alohamini_cartesian/control/controller.py:133` |
| 手臂低位或有承托 | C2b 把大臂抬到竖直向上，摆姿与保持期间**必须有承托或人手不离开**；判为不安全时改用 C2（小臂朝另一侧）或改在低位判 sign，参考位形的目视判据随之重算 | [§7.1](#exp-e3) 开跑勾选单 |
| 断电开关在人手上 | sign 判定会真实下发命令，属运动档；远程无人在场**不跑**「sign 判定」那一节 | [§7.1](#exp-e3) 开跑勾选单 |
| 授权范围 | 摆姿 + 读数属只读观测档（[§4.1](3-health.md#sec-4-1) 的 `capture`）；sign 判定属运动档，需现场运动授权 | [§0](1-prepare.md#sec-0) 的授权表 |
| 摆姿本身 | 摆姿要动关节（释放扭矩后手摆，或人手扶着摆），使能瞬间按当前位置闭环，可能跳一下；`Torque_Enable=1` 的写入者至今未定位，所以摆姿期间人手不离开、末端不放重物 | `TODO.md` §3.1「识别 `Torque_Enable=1` 的写入者」那行 |

### 现场确认项（[§10.8](#exp-e8)）

| # | 确认项 | 怎么确认 | 确认不了怎么办 |
| --- | --- | --- | --- |
| 1 | 参考位形 C2b 在该侧真能摆到位（`shoulder_lift = −72°` / `elbow_flex = +73°` 是否在可达窗口内） | 摆姿时目视逐条对「参考位形」那一节的判据；摆不动或顶住就是不可达 | 改 C2（小臂朝 `+Y`）或整体降级为「低位 L 形」并重算该位形的 FK 与表格；不可达那一侧记「位形不可达」，不得硬掰 |
| 2 | 摆好的姿势有没有碰撞 / 缆线绷紧 | 目视 + 手摸线束；末端与机身、桌面、升降柱留出可见间隙 | 有碰撞风险就换位形或换侧，不记录本次标定 |
| 3 | 读数命令可用性（[§4.1](3-health.md#sec-4-1) 的 `capture` 在本次环境能跑通） | `capture` exit 0、`robot_metadata.schema_version == 2`、`observation_state` 里六关节键齐全 | 跑不通就记「本次无读数」，**不得**用上一次姿势的旧读数顶替（姿势已变，读数作废） |
| 4 | 左臂 sign 起始值无记录，必须用单步法 | 证据目录 `s4-20261106` / `s4-20261107` / `s4-20261108` 三份 `configs` 全空，全目录按 `sign` / `zero` 两个词 grep 无命中（本节已查） | 没有起始值这件事本身不是阻塞：按「sign 判定」那一节六轴各做一次单步法即可；**不得**先填一个「看起来合理」的 sign |
| 5 | sign 与 zero 是否互相自洽 | 同一姿势下按两种候选 sign 各算一次 zero，差值应恰好 `2 × motor_deg_read`（「sign 判定」那一节的交叉校验） | 不吻合就重采这一轴的读数，不要改 sign 去迁就 |
| 6 | 标定文件的拆分与侧别对不对 | 拆出的两份是扁平六键（用 `jq` 求键数为 6）；`side` 字段与文件名、`--signs-json` / `--zeros-json` 的指向三者一致 | 任一处不一致就重拆，不带着这个疑点往下跑 [§10.7](#exp-e7) 的 ①a |
| 7 | 与 firmware `homing_offset` / `drive_mode` 不得混用 | 本节文件里的 `zero` 是**度**、`sign` 是 ±1；不写、不读 `homing_offset` / `drive_mode`，也不拿它们当对照值 | 若现场发现某轴 `drive_mode` 不是 0，先停：那会改变归一化读数的符号（[§4.1c](3-health.md#sec-4-1c) 已写明），读数链要重新界定才能用 |
| 8 | 右臂不得复制左臂值 | 右臂文件从右臂自己的读数与单步法来；例外条款要求「实测一致」+ 两次独立读数 | 拿不出两次独立读数就重做右臂的 sign 判定，不得沿用 |
| 9 | 本节不产出任何 V 编号的实机证据 | 本节产出的是两份标定文件 + 自检记录；V01 的 `move_to` 实机证据仍要按 [§10.7](#exp-e7) 跑 | 记「标定已有、`move_to` 未跑」，别把标定记录当 `move_to` 证据 |
