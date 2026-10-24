# S4 现场实施：真机 smoke

这份文档给一次现场窗口的执行顺序、每条命令、每步判据与产物。拿着笔记本到机器人现场照着做，不因缺信息临时决策。

## 0 这份文档怎么用

- 先读第 1 节与第 2 节，再动手。第 1 节消除两套 Gate 的撞名，第 2 节给出不满足就不能开始的前置条件。
- 执行顺序固定：阶段 0 到场核对 → 阶段 1 零运动测量 → 阶段 2 归档 Gate 记录 → 阶段 3 制造一次拒绝记录 → 阶段 4 受控试动 → 阶段 5 配置样例。
- 拿到非 0 退出码看第 9 节与第 12 节；交回前按第 11 节逐条勾选。

一页总览：

| 阶段 | 时长 | 是否动机器人 | 产物 | 退出码判据 |
| --- | --- | --- | --- | --- |
| 0 到场核对 | 10 分钟 | 否 | 勾选清单、两条 `--help` 的退出码 | 两条 `--help` 均 exit 0 |
| 1 零运动测量与指纹 | 本文未设预算 | 否 | `capture/fingerprint-<UTC>.json`、`rtt/rtt-net-<UTC>.csv`、`rtt/rtt-deploy-<UTC>.csv` | 三次运行均 exit 0 |
| 2 归档 Gate A–E | 本文未设预算 | 否 | `gates/` 下 10 个 `<side>-gate-<letter>.json` | 校验器 `ok` 为 `true` 且 `records_checked` 为 `10` |
| 3 制造代码 Gate F 拒绝 | 本文未设预算 | 否 | `stdout/` 下 stderr 原文与命令原文 | exit 6，零命令 |
| 4 Mode B 受控试动 | 本文未设预算 | 是，单臂单关节 | `tracking/tracking-<UTC>.csv` 与 report 原文 | exit 0；exit 8 按第 12 节处置 |
| 5 配置样例采集 | 本文未设预算 | 否 | 模板 YAML 与现场 YAML 各一份 | 两份文件能一起被 `load_settings` 读通，不抛 `ConfigError` |

只有阶段 0 有时间预算，来源是本次任务的约定。其余阶段的实际耗时记进 `stdout/` 存档，本文不设预算。

## 1 开跑前必须知道的一件事：两套 Gate 撞名

「Gate」这个词在本项目里指两套完全不同的东西，字母还撞车。

| 说法 | 指什么 | 在哪 | 数量 |
| --- | --- | --- | --- |
| 代码时的「Gate A–F」 | `measure_latency.py` 内的运行时门禁（发命令前的检查） | `benchmarks/alohamini_io/measure_latency.py` | 6 个，A–F |
| 归档时的「Gate A–E」 | 验收里程碑：几何、映射、FK 点检、离线 IK、受控试动 | 定义只在旧仓库 `docs/alohamini-control/`，本库无 | 5 个，A–E |

两套字母撞车，同一封信里说「Gate A」会产生歧义。本文一律写成「代码 Gate A」与「归档 Gate A」。

代码 Gate 全清单，这就是现场要过的六道：

| 代码 Gate | 判什么 | 不通过时 |
| --- | --- | --- |
| A | `--enable-motion` 与 `--joint` 两个参数都给了 | exit 6，在建客户端之前就拒，零网络零命令 |
| B | 目标关节出现在首帧观测里 | exit 6 |
| C | 宿主 metadata 的 `robot_model` 非空，且与 `--robot-model` 相等 | exit 6 |
| D | 该关节基线反馈是有限数 | exit 6 |
| E | `delta_deg > 0` 且 `steps >= 1`，且 `delta_deg × steps <= 20.0` | exit 6 |
| F | ① metadata `schema_version` 是整数 `2`（`True` 与字符串 `"2"` 都拒）；② 该关节的 deg-per-unit 能从 v2 校准表算出 | exit 6，零命令 |

代码 Gate A 在 `main` 里、建客户端之前执行；B–F 在 `_tracking_preflight` 里、第一条 `send_action` 之前执行。B–F 的顺序是 B、C、D、E、F。

## 2 硬前置条件（不满足就别开始）

| 编号 | 条件 | 怎么确认 | 不满足怎么办 |
| --- | --- | --- | --- |
| P0 | 宿主广播的 `robot_metadata.schema_version` 必须是整数 2 | 跑 `capture` 一次，exit 0 且 JSON 里 `robot_metadata.schema_version` 是 `2` | 见下面「P0 阻塞」小节 |
| P1 | 真机 AlohaMini 2 Pro 在场，物理断电开关可用 | 目视加手测 | 停止 |
| P2 | Pi 上 Host 进程常驻 | Pi 上 `ps` 与端口 5555/5556 在听 | 起 Host |
| P3 | PC 与 Pi 同网段 | `ping` 一次 | 修网络 |
| P4 | 现场授权（谁授权、授权范围、谁在旁） | 口头确认，把结论记进归档 Gate 记录的 `operator` 字段 | 停止 |
| P5 | 轨迹空间足够：Mode B 累计行程可达 20° | 卷尺与目视确认该关节正向 20° 无干涉 | 减小 `--steps` |

### P0 阻塞

上一次现场（证据目录 `field-run-summary.md` 第 4 节）记录：`capture` 退出码 6，报错原文 `invalid robot metadata: schema_version=1, expected 2.`。这说明 Pi 上的 Host 仍在广播 `schema_version: 1`。

厂商代码里该值硬编码为 `2`（旧仓库 `src/lerobot/robots/alohamini/alohamini_host.py:181`，原文 `"schema_version": 2,`）。它写在 metadata 字典字面量里，没有配置项，也没有常量名。Pi 广播 1 说明 Pi 上的 vendor 代码是更旧的提交。

宿主仍是 v1 时，后果是确定的：

- `capture` 必 exit 6，不写盘。
- 代码 Gate F 必然拒绝，`--mode tracking` 跑不起来，一条命令都发不出。

结论：本窗口能否做受控试动，取决于 P0 是否先解决。解决方式是把 Pi 上 `/home/pi5/lerobot_alohamini` 更新到与本机 vendor 提交一致的版本。这一步涉及远端 git 操作，需要单独授权。

本文档不提供升级步骤：两仓库文档里找不到 `schema_version` 从 1 升到 2 的步骤。

## 3 阶段 0：到场核对（10 分钟）

先定义本窗口的证据目录，后面所有命令都用它：

```bash
S4DIR=~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-<UTC日期>
```

`<UTC日期>` 写成 `YYYYMMDD`，例：日期 2026-10-22 写作 `s4-20261022`，与既有证据文件 `rtt-net-20261022-162919.csv` 同一命名法。后面的 `<UTC>` 是时分秒时间戳，形式 `YYYYmmddTHHMMSSZ`，每条命令自己填实际值。

逐条勾选：

- [ ] 断电开关位置已知，手能一次按到
- [ ] 机器人已上电
- [ ] Pi 上 Host 已起（记下 PID）
- [ ] 5555 与 5556 端口在听
- [ ] `uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --help` exit 0
- [ ] `uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --help` exit 0
- [ ] 证据目录已建：`mkdir -p "$S4DIR"/{capture,rtt,tracking,gates,configs,stdout}`
- [ ] 每条命令都显式传 `--out` 或 `--csv`

默认文件名的坑：两个脚本的默认产物目录是 `artifacts/alohamini-io/`，相对当前工作目录解析，不以仓库根为基准。文件名里的时间戳只到秒（格式 `YYYYmmddTHHMMSSZ`），同一秒内跑两次会静默覆盖前一份（`write_text` 直接盖写）。结论：一律显式传 `--out` / `--csv`，不要依赖默认文件名。

`--host` 用 Pi 的地址。上一次现场记录的地址是 `10.34.170.98`（来源 `field-run-summary.md` 第 1 行「远端」）；本次先用 `ping` 确认该地址仍是这台 Pi，再往下做。

## 4 阶段 1：零运动测量与指纹（不动机器人）

三条命令都不构造任何运动指令：`capture` 只请求观测，`measure --mode rtt` 只采样观测。

| 步骤 | 命令 | 判据 | 产物 |
| --- | --- | --- | --- |
| 1.1 | `uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host 10.34.170.98 --out "$S4DIR/capture/fingerprint-<UTC>.json"` | exit 0，JSON 写出 | `fingerprint-<UTC>.json` |
| 1.2 | `uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host 10.34.170.98 --mode rtt --rate-hz 1000 --request-window 1 --samples 500 --csv "$S4DIR/rtt/rtt-net-<UTC>.csv"` | exit 0 | `rtt-net-<UTC>.csv` |
| 1.3 | `uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host 10.34.170.98 --mode rtt --rate-hz 30 --request-window 3 --samples 500 --csv "$S4DIR/rtt/rtt-deploy-<UTC>.csv"` | exit 0 | `rtt-deploy-<UTC>.csv` |

1.1 的默认值：`--samples 5`、`--timeout-s 30.0`、`--port 5556`。1.2 与 1.3 的 `--samples` 默认是 `500`，本文照样写出来，便于现场比对命令原文。

每条命令的 stdout 与 stderr 各存一份到 `"$S4DIR/stdout/"`，文件名带步骤号与退出码。

两条读法必须先定下来：

- `--rate-hz` 低于 200 时，`age_s` 的主体是响应在客户端 socket 里等下一次 poll 的空闲时间，所以它测的是「端到端观测滞后」，不能当网络往返时间读。要测 RTT 必须用 `--rate-hz 1000`，上一次现场就是这么跑的。脚本在低于 200 Hz 时自己打警告，警告原文见 `field-run-summary.md` 第 5b 节；打了警告不等于失败，但它封住了这一组数值当 RTT 读的用法。
- `--request-window 1` 是 RTT 形态（无预取等待可减）；`--request-window 3` 是部署形态（控制栈实际用的窗口）。两者不可混为一谈，也不可互相替代。

参考值（上一次现场，2026-10-22）：1.2 组 `achieved_rate_hz=49.0`（请求 1000 Hz），1.3 组 `achieved_rate_hz=30.1`（请求 30 Hz）。来源 `field-run-summary.md` 第 5 节。实际到达率受 Host 主循环节拍限制。

### capture 的 JSON 有 10 个顶层键

| 键 | 含义 |
| --- | --- |
| `schema_version` | payload 格式版本，恒为 `1`（源码常量 `PAYLOAD_SCHEMA_VERSION = 1`） |
| `captured_utc` | 抓取时刻，ISO 8601，秒精度 |
| `host` | `--host` 传入的地址 |
| `requested_robot_model` | `--robot-model` 传入的模型名 |
| `read_only` | 恒为 `true` |
| `sample_count` | `samples` 的实际行数 |
| `samples` | 一行一个被接受的观测请求，字段 `observation_sequence`、`age_s`、`sent_monotonic_s`、`received_monotonic_s` |
| `robot_metadata` | 宿主 metadata 原样拷贝 |
| `control_status` | 最后一帧观测带的宿主控制状态帧，宿主没发时为 `null` |
| `observation_state` | 连接后取到的一帧 state-only 观测，只保留有限数值 |

### 这个 JSON 里有两个 `schema_version`

- 顶层 `schema_version` 是 `1`，它表示 payload 格式版本，永远不是 2。
- 要等于 2 的是 `robot_metadata.schema_version`。

现场最容易把顶层那个 `1` 当成失败原因。脚本只校验 `robot_metadata.schema_version == 2` 与 `motors` 非空映射，不校验每个电机的 8 个字段是否齐全。8 字段是 `id`、`model`、`normalization`、`drive_mode`、`range_min`、`range_max`、`homing_offset`、`resolution`，齐不齐由宿主决定，现场逐电机记录实际覆盖情况。

## 5 阶段 2：归档 Gate A–E 记录（不动机器人时填）

本库没有归档 Gate 的模板与校验器：`docs/` 下只有决策记录、迁移归档与本文件。格式定义、模板、校验器都在旧仓库：

- 格式与检查项：`/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop/docs/alohamini-control/gates/README.md`
- 模板：同目录 `_template.json`
- 校验器：`/Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop/tools/alohamini_gate_record.py`

下面 11 条是现场照着填的全部规则。

1. 文件命名：`<side>-gate-<letter>.json`。`side` 取 `left`/`right`，`letter` 取 `A`–`E`。一臂一 Gate 一个文件，一臂五个文件，两臂共 10 个。
2. 模板全文见本文附录 A，照抄后改名。
3. `checks` 是数组，每项 `{id, description, expected, actual, result}`，`result` 只能取 `pass`/`fail`。`status=not_started` 时 `checks` 必须为空数组；`status=passed` 时 `checks` 非空、每项 `result` 为 `pass`，`evidence` 也非空。
4. `evidence` 是数组，每项 `{kind, path, note}`。`kind` 只能取 `file`/`command_output`/`measurement`。`path` 非空，且不能是绝对路径：写相对仓库根的路径，或以 `~/` 开头。
5. `profile_fingerprint` 三个子键齐全：`robot_model`、`robot_id`、`calibration_sha256`（64 位十六进制）。
6. `calibration_sha256` 现场这样算：

   ```bash
   shasum -a 256 ~/.cache/huggingface/lerobot/calibration/robots/alohamini/AlohaMiniRobot.json
   ```

7. `recorded_at` 必须 ISO 8601 且偏移为 0。`Z`、`+00:00`、`+0000`、`z` 都可；带非零偏移或无时区会被拒。
8. 校验命令与判据：

   ```bash
   python /Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini-cartesian-teleop/tools/alohamini_gate_record.py validate "$S4DIR/gates/" --json
   ```

   要求 `ok` 为 `true` 且 `records_checked` 为 `10`。小于 10 视为归档不完整。校验器只用标准库，本机任何 Python 都能跑，不需要 `lerobot`，也不改任何文件。
9. 跨 Gate 规则：同一侧归档 Gate E 为 `passed` 要求 A–D 均存在且均 `passed`，否则校验器 exit 3。
10. `calibration_sha256` 变化即该侧此前记录作废。三种情况会触发：重标定、换电机、改 `robot_id`。作废后须重跑重录。
11. 每道 Gate 该填哪些检查项，见本文附录 B 的 A1–A3 / B1–B4 / C1–C3 / D1–D3 / E1–E3。

红线：Gate 记录是准入证据，不是安全保证。记录通过不代表该侧可未经授权试动。

## 6 阶段 3：制造一次代码 Gate F 的真实拒绝记录

代码 Gate F 在第一条 `send_action` 之前执行，所以它的拒绝路径结构上零命令，可以安全地制造。

做法：宿主仍是 `schema_version=1` 时，跑

```bash
uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host 10.34.170.98 --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --steps 1 --csv "$S4DIR/tracking/f-gate-refusal-<UTC>.csv"
```

代码 Gate A–E 都会过，拒绝点落在 F 的版本判断，得到 exit 6、零命令。拒绝时脚本不生成任何 CSV，命令里的 `--csv` 只是为了把命令写完整：跑之前不传它，默认路径同样用不上。

宿主已经是 v2 时，改用「换算不可解析的关节」触发 F 的第二段。前提是先确认该关节真的不可换算，否则命令会真发出去。确认不了就不做这一步，宁缺记录不冒风险。

记录要求：保存 stderr 原文、命令原文、退出码三项，写进对应归档 Gate 记录的 `evidence`，`kind` 取 `command_output`。

不要为了制造 F 的拒绝去改标定文件。

## 7 阶段 4：Mode B 受控试动（唯一动机器人的阶段）

开跑前再确认，逐条勾选：

- [ ] 现场授权到手，授权范围写明（哪个关节、多少度、谁在旁）
- [ ] 单臂：另一臂不参与本次运动
- [ ] 空场：人物离场
- [ ] 无负载：末端无负载
- [ ] 断电开关在手边，可随时按下
- [ ] 该臂归档 Gate A–D 均 `passed`（这是归档 Gate 的准入规则，与代码 Gate 无关）

命令：

```bash
uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host 10.34.170.98 --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 5 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-<UTC>.csv"
```

参数与理由：

| 参数 | 建议值 | 为什么 |
| --- | --- | --- |
| `--mode tracking` | 必填 | 不写就落回 `rtt`，只采观测 |
| `--enable-motion` | 必填 | 代码 Gate A 的第一个条件 |
| `--joint` | `arm_left_shoulder_pan.pos`（先左臂单关节） | 代码 Gate A 的第二个条件；`.pos` 后缀可省 |
| `--delta-deg` | `1.0` | 会被硬性 clamp 到 `1.0°`（常量 `MAX_STEP_DELTA_DEG`），传更大也只发 1.0 |
| `--steps` | 先 `5`，成功后再说 | `delta × steps` 必须不超过 20.0°；默认 20 步正好顶在上限 |
| `--control-hz` | `50.0`（默认） | 两条命令最小间隔 1/50 s = 20 ms；必须有限且不小于 1，`inf`/`nan` 被拒 |
| `--settle-tolerance-deg` | `0.05` | 默认 `0.15` 会让 90% 上升时间测不到：容差不小于 `0.1 × delta`（即 0.1°）时，settle 循环的退出条件比 90% 越线更宽松，`rise_90_ms` 报 `count=0`、`p50=nan`。要测上升时间必须取小于 0.1° 的值 |
| `--settle-timeout-s` | `2.0` | 默认 `1.0` s，20 ms 命令节奏下每步预算偏紧 |
| `--timeout-s` | `60` | 默认 `30` s 对多步加 settle 偏紧，中途超时会以 exit 1 结束 |

运动学事实：

- 方向：每步目标是当前实测基线加 `delta_deg`，`delta_deg` 恒正，所以每步朝同一方向累积。
- 脚本不检查关节限位。源码里没有任何对 URDF 或 adapter 限位的比较。方向与累计行程是否安全，只能靠人。
- 反馈失联保护：连续 3 次 `observation_sequence` 不增长就中止（exit 1）。中止前仍会发收尾 hold。
- 收尾 hold 从 `finally` 发，正常结束、命令被拒、反馈死、Ctrl-C 都会发。只有一次 `*.pos` 都没测到时才不发。
- hold 是就地冻结：把实测 `*.pos` 当目标，三轴速度钉 0，反馈带高度就回写实测高度。报告里只写 `hold_issued`，不写「sent」，因为 action socket 是 PUSH/CONFLATE 无确认。

exit 8 表示 hold 被拒，也就是停命令没到。这一条按「机器可能仍在动」处置：先断电，再排查。判定与处置见第 12 节。

## 8 阶段 5：配置样例采集（两份 YAML）

按 D2 的「模板文件 → 现场文件」两层产出：一份模板（不含现场敏感取值的通用默认），一份现场（真实端口、相机节点、超时与频率）。取值只能来自现场实测。

字段名以 `src/alohamini_cartesian/config/overlay.py` 为准：

| 字段 | 类型 | 校验规则 | 库内默认值 |
| --- | --- | --- | --- |
| `robot_model` | 非空字符串 | 空串抛 `ConfigError` | `alohamini1` |
| `left_port` | 非空字符串 | 空串抛 `ConfigError` | `/dev/ttyUSB0` |
| `right_port` | 非空字符串 | 空串抛 `ConfigError` | `/dev/ttyUSB1` |
| `cameras` | 映射，名到字符串 | 非映射或值非字符串抛 `ConfigError` | `{}` |
| `observation_request_window` | 整数 | 必须是 `int`（`bool` 被拒），不小于 1 | `1` |
| `watchdog_timeout_ms` | 整数 | 必须是 `int`（`bool` 被拒），不小于 1 | `500` |
| `max_loop_freq_hz` | 整数 | 必须是 `int`（`bool` 被拒），不小于 1 | `50` |

未知键抛 `ConfigError`，不静默忽略。

模板文件写到 `"$S4DIR/configs/template.yaml"`。下面每个值都取自 `overlay.py` 的 `AlohaMiniSettings` 默认值，可以直接落盘：

```yaml
robot_model: alohamini1
left_port: /dev/ttyUSB0
right_port: /dev/ttyUSB1
cameras: {}
observation_request_window: 1
watchdog_timeout_ms: 500
max_loop_freq_hz: 50
```

现场文件写到 `"$S4DIR/configs/site.yaml"`，只写与模板不同的字段，字段名同上。数值本文不给，逐字段从现场读：

| 字段 | 现场去哪读 |
| --- | --- |
| `robot_model` | 阶段 1.1 产物的 `robot_metadata.robot_model` |
| `left_port` / `right_port` | 现场 `ls -l /dev/serial/by-id/` 输出里对应的两个串口设备节点 |
| `cameras` | 现场 `ls -l /dev/video*` 输出，加实际相机接法 |
| `observation_request_window` | 宿主实际生效的观测请求窗口 |
| `watchdog_timeout_ms` | 宿主实际生效的看门狗超时 |
| `max_loop_freq_hz` | 宿主实际生效的主循环频率 |

判据：两份一起能被读通。命令：

```bash
uv run --no-sync python -c "from pathlib import Path; from alohamini_cartesian.config.overlay import load_settings; print(load_settings(template=Path('$S4DIR/configs/template.yaml'), site=Path('$S4DIR/configs/site.yaml')))"
```

现场易错点：本库 config overlay 的 `watchdog_timeout_ms` 默认是 `500`（`src/alohamini_cartesian/config/overlay.py` 的 `AlohaMiniSettings`），厂商 `config_alohamini.py` 默认是 `1000`（旧仓库 `src/lerobot/robots/alohamini/config_alohamini.py:102`）。现场核对宿主实参时不要把两个数字混用，样例里写实际生效的那个值。

现场易错点：`robot_model` 的两处默认值不一致。本库 `src/alohamini_cartesian/config/overlay.py` 的 `AlohaMiniSettings.robot_model` 默认值是 `alohamini1`（`overlay.py:51`）；两个现场脚本的 `DEFAULT_ROBOT_MODEL` 都是 `alohamini2pro`（`benchmarks/alohamini_io/capture_robot_fingerprint.py:76`、`benchmarks/alohamini_io/measure_latency.py:360`）。现场机型是 AlohaMini 2 Pro，而 `measure_latency.py` 的代码 Gate C 要求宿主 metadata 的 `robot_model` 与 `--robot-model` 相等（`measure_latency.py:1437`）。所以现场 YAML 必须显式写 `robot_model: alohamini2pro`，不能用模板里的 `alohamini1`，否则配置表达的意思与真机不符。

模板随包分发的落点（`configs/` 是否进仓库）不在本次窗口决定。两份文件本次都落在证据目录。

## 9 退出码对照表

**一码两义**：`6` 在 `capture` 里表示宿主 metadata 不可用、主动不写盘；在 `measure` 里表示运动门禁拒绝、零命令。现场拿到 6，先看是哪个脚本。

| 脚本 | 码 | 含义 |
| --- | --- | --- |
| 两者 | 2 | 参数解析失败（`--samples < 1`、`--rate-hz <= 0`、`--control-hz` 非有限或小于 1、`--delta-deg <= 0`、`--settle-tolerance-deg` 不小于有效 delta 等），不经过任何 `EXIT_*` 常量 |
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

## 10 产物与归档

证据目录按 `AGENTS.md` 第 8 节约定落在 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/` 下。本次建一个 `s4-<UTC日期>/` 子目录：

```
/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/
└── s4-<UTC日期>/
    ├── capture/
    ├── rtt/
    ├── tracking/
    ├── gates/
    ├── configs/
    └── stdout/
```

| 目录 | 放什么 |
| --- | --- |
| `capture/` | 阶段 1.1 的 `fingerprint-<UTC>.json`；P0 未解决时放失败运行的 stderr 原文，文件名带退出码 |
| `rtt/` | 阶段 1.2 与 1.3 的两份 CSV（`rtt-net-*`、`rtt-deploy-*`） |
| `tracking/` | 阶段 4 的 `tracking-<UTC>.csv` 与 report 原文 |
| `gates/` | 阶段 2 的 10 个 `<side>-gate-<letter>.json` |
| `configs/` | 阶段 5 的 `template.yaml` 与 `site.yaml` |
| `stdout/` | 每条命令的 stdout、stderr 原文、退出码、命令原文；阶段 3 的拒绝记录也放这里 |

`gates/` 里每份记录的 `evidence[].path` 写成 `~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-<UTC日期>/...` 形式，不要写 `/Users/...` 绝对路径，校验器拒绝绝对路径。

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

## 11 验收标准

每条一个判据，不写「看起来正常」。

- [ ] 宿主 `robot_metadata.schema_version` 取到整数 `2`，且有 `capture` 产物为证（P0 关闭）
- [ ] `capture` exit 0，JSON 10 个顶层键齐全，且 `robot_metadata.motors` 里逐个电机的 8 字段齐备情况有记录
- [ ] 两组 rtt 各 500 样本、各 exit 0，且报告里 `achieved_rate_hz` 与请求率的差距有记录
- [ ] 代码 Gate F 至少有一次真实拒绝记录（stderr 原文 + 命令原文 + 退出码）
- [ ] Mode B 若执行：每步的 `error_deg`、`overshoot_deg`、`settled` 齐全；`rise_90_time_s` 非空；若为空，说明是容差设置问题并写明原因；收尾 `hold_issued` 在报告里
- [ ] 10 份归档 Gate 记录齐备，校验器 `ok` 为 `true` 且 `records_checked` 为 `10`
- [ ] 两份 YAML 样例（模板 + 现场）落盘，字段名与 `config/overlay.py` 一致，且能被 `load_settings` 读通

## 12 失败与中止处置

| 现象 | 判据 | 处置 |
| --- | --- | --- |
| exit 6（capture） | stderr 含 `schema_version=` | 记录原文，转 P0 分支，不要重试 |
| exit 6（measure） | 拒绝发生在首帧后 | 记录是哪道代码 Gate，写进归档记录 |
| exit 8 | 报告里 hold 被拒 | 先断电，再排查；按「仍在动」处置 |
| exit 1 加反馈失联 | 连续 3 次序号不涨 | 检查网络与 Host 负载 |
| 网络中断 | `ping` 丢包 | 重跑该阶段，不复用旧产物 |

## 13 已知缺口与未验证项

- Mode B 从未在真机跑过。
- 收尾 hold 的实际效果无证据。
- exit 8 的真实触发路径无记录。
- Action socket 无确认，`hold_issued` 不等于送达。
- 脚本不检查关节限位。
- 本库没有归档 Gate 的模板与校验器，只有旧仓库有。
- `schema_version` 升到 2 的步骤两仓库文档都没有。

## 14 待裁决事项

两条，均未定，不阻塞现场零运动阶段。

1. 归档 Gate 的模板与校验器是否从旧仓库迁入本库？校验器只用标准库，可以直接搬；模板 21 行。不迁就只能引用旧仓库路径。
2. 证据目录是否长期固定在 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/`？还是本库内另立 `evidence/`？

## 附录 A：Gate 归档模板原文

来源：旧仓库 `docs/alohamini-control/gates/_template.json`。21 行，一字未改。

```json
{
  "_comment": "Gate 归档记录模板。复制为 <side>-gate-<letter>.json 后填写，格式说明见 docs/alohamini-control/gates/README.md。以 _ 开头的键被校验器忽略。",
  "_comment_placeholders": "必须替换的占位：gate（A–E）、side（left/right）、profile_fingerprint.robot_model 与 profile_fingerprint.robot_id（此处填的是 Pi 现场默认值，现场不同须改）、profile_fingerprint.calibration_sha256（64 个 0 是占位，换成实测 64 位十六进制度值）、recorded_at（1970-01-01T00:00:00Z 是占位，换成 ISO 8601 UTC 实际时刻）、operator。",
  "_comment_rules": "status=not_started 允许 checks 与 evidence 为空，本模板可原样通过结构校验。status=passed 要求 checks 非空且全部 result 为 pass，evidence 非空。同一侧 Gate E 为 passed 要求 A–D 均已 passed。",
  "_comment_checks": "checks 元素形状：{id, description, expected, actual, result}。id 用 A1、B2 式短标识；expected 写可判定值；actual 写实测值；result 取 pass 或 fail。示例见 README 第 8 节。",
  "_comment_evidence": "evidence 元素形状：{kind, path, note}。kind 取 file、command_output、measurement；path 写相对仓库根的路径或以 ~/ 开头的现场路径。",
  "schema_version": 1,
  "gate": "A",
  "side": "left",
  "status": "not_started",
  "profile_fingerprint": {
    "robot_model": "alohamini2pro",
    "robot_id": "AlohaMiniRobot",
    "calibration_sha256": "0000000000000000000000000000000000000000000000000000000000000000"
  },
  "recorded_at": "1970-01-01T00:00:00Z",
  "operator": "",
  "checks": [],
  "evidence": [],
  "notes": ""
}
```

这里的 `schema_version` 是 `1`（归档格式版本），与宿主 metadata 的 `2` 是两套东西。

## 附录 B：Gate A–E 检查项清单

来源：旧仓库 `docs/alohamini-control/gates/README.md` 第 8 节。逐字照抄。

归档 Gate A 几何：

| id | description | expected / actual 写法 |
| --- | --- | --- |
| `A1` | 六关节中心与轴向与派生模型一致 | expected 写「与 `am_arm200/kinematics.urdf` 一致」；actual 写逐关节比对结论 |
| `A2` | 末端 frame 为 `left_Fixed_Jaw` / `right_Fixed_Jaw` 输出系 | expected 写 frame 名；actual 写实际取到的 frame 名 |
| `A3` | 固定变换（肩底座原点）与安装位置一致 | expected 写三轴偏移量（米）；actual 写实测值 |

归档 Gate B 映射：

| id | description | expected / actual 写法 |
| --- | --- | --- |
| `B1` | 逐关节 `sign` 与实机方向一致 | expected 写每个关节的符号；actual 写实测方向表结论（逐关节 pass 或 fail） |
| `B2` | 逐关节 `zero` 与实机零位一致 | expected 写零位定义；actual 写实测零位偏差（度） |
| `B3` | 标定指纹录入齐全 | expected 写「18 个电机齐全，8 个字段齐全」；actual 写实际覆盖数 |
| `B4` | 标定 JSON 与 EEPROM 双存储对照 | expected 写「`range_min`、`range_max`、`homing_offset` 逐电机相等」；actual 写不一致的电机清单，无则写「0 个不一致」 |

归档 Gate C FK 点检：

| id | description | expected / actual 写法 |
| --- | --- | --- |
| `C1` | 不少于 5 个参考姿态覆盖工作空间 | expected 写「5 个姿态」；actual 写实际姿态数与姿态说明 |
| `C2` | 理论 FK 与实测位置误差在门限内 | expected 写门限（毫米）；actual 写各姿态误差（毫米） |
| `C3` | 符号错误的排除：各轴单关节小角度正向运动方向与模型同号 | expected 写「六关节同号」；actual 写异常关节清单 |

归档 Gate D 离线 IK：

| id | description | expected / actual 写法 |
| --- | --- | --- |
| `D1` | 安全姿态下 ±X/±Y/±Z 小步求解连续 | expected 写步数与每步最大关节变化（度）；actual 写实测值 |
| `D2` | 位置残差在门限内 | expected 写「残差不超过 5.0 mm」；actual 写 max_residual（米） |
| `D3` | 求解限位用 adapter 交集限位而非 URDF 占位限位 | expected 写交集限位来源；actual 写实际生效的限位区间 |

归档 Gate E 受控试动：

| id | description | expected / actual 写法 |
| --- | --- | --- |
| `E1` | 试动条件：单臂、空场、无负载 | expected 写「另一臂不参与、人物离场、末端无负载」；actual 写现场确认结论 |
| `E2` | 极低速小步运动，方向与指令一致，单次总行程不超过 20 度 | expected 写步长与总行程上限；actual 写实测方向与行程 |
| `E3` | 保护触发：触发后停止及时，停止后不自动续动 | expected 写触发条件与停发时限；actual 写触发记录与停止耗时 |

## 附录 C：两个脚本的完整参数表

### capture（6 个参数加 `-h`）

| 参数 | 必填 | 默认值 | 约束 | 影响什么 |
| --- | --- | --- | --- | --- |
| `--host` | 是 | 无 | 无 | 连哪台 Pi |
| `--port` | 否 | `5556` | 无 | 宿主观测端口 |
| `--robot-model` | 否 | `alohamini2pro` | 无 | 传给客户端的模型名，须与宿主一致 |
| `--samples` | 否 | `5` | 小于 1 报参数错误 exit 2 | 要采到的观测请求数 |
| `--timeout-s` | 否 | `30.0` | 无 | 采样本的总时间预算 |
| `--out` | 否 | `artifacts/alohamini-io/fingerprint-<UTC>.json`，导入时算时间戳 | 父目录自动创建 | JSON 落盘位置 |

### measure（17 个参数加 `-h`）

| 参数 | 必填 | 默认值 | 约束 | 影响什么 |
| --- | --- | --- | --- | --- |
| `--host` | 是 | 无 | 无 | 连哪台 Pi |
| `--port` | 否 | `5556` | 无 | 宿主观测端口 |
| `--robot-model` | 否 | `alohamini2pro` | 无 | 代码 Gate C 的比对对象 |
| `--mode` | 否 | `rtt` | `rtt` 或 `tracking` | 跑哪套测量；`tracking` 会真动机器人 |
| `--rate-hz` | 否 | `30.0` | 必须大于 0；小于 200 打警告 | 观测采样率，与 `--control-hz` 独立 |
| `--control-hz` | 否 | `50.0` | 必须有限且不小于 1 | 命令最小间隔的倒数 |
| `--samples` | 否 | `500` | 不小于 1，仅 rtt 用 | 要采到的观测帧数 |
| `--timeout-s` | 否 | `30.0` | 无 | 整次运行的时间预算 |
| `--csv` | 否 | `artifacts/alohamini-io/<mode>-<UTC>.csv` | 无 | CSV 落盘位置 |
| `--request-window` | 否 | `1` | 不小于 1 | 在飞观测请求数；1 是 RTT 形态，3 是部署形态 |
| `--host-loop-hz` | 否 | `50.0` | 大于 0 | 宿主主循环频率，决定相位项与排队项 |
| `--enable-motion` | 否 | 开关，默认关 | 无 | 代码 Gate A 的第一个条件 |
| `--joint` | 否 | 无 | tracking 模式实质必填 | 要步进的关节；代码 Gate A 的第二个条件 |
| `--delta-deg` | 否 | `1.0` | 大于 0；实际被 clamp 到 `1.0` | 每步目标增量（度） |
| `--steps` | 否 | `20` | 不小于 1 | 步数；与 `--delta-deg` 相乘受 20.0° 上限约束 |
| `--settle-timeout-s` | 否 | `1.0` | 大于 0 | 每步的稳定判定预算 |
| `--settle-tolerance-deg` | 否 | `0.15` | 不小于 0，且必须小于有效 delta | 稳定判定的容差（度） |

`measure` 的 `--help` 用的是 `RawDescriptionHelpFormatter`，不打印默认值。上表默认值来自源码常量。
