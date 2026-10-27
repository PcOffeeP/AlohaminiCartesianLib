# S4 现场实施：真机 smoke

这份文档给一次现场窗口的执行顺序、每条命令、每步判据与产物。到现场后由 Agent 通过 SSH 连上 Pi，照着本文档逐条取数与执行，不因缺信息临时决策。

本手册的 Pi 侧命令在 2026-10-25 对 `pi5@10.34.170.98` 执行过一轮（第 2、4、8 节的部分条目，每步在 `$S4DIR/stdout/` 下留有成组的 `.cmd`/`.log`/`.exit`）；其余条目仍未实测。首次在现场执行未实测条目时如有偏差，按实际输出修正并回写到本文档。

## 0 这份文档怎么用

- 先读第 2 节（硬前置条件）与第 1 节（代码 Gate 清单），再动手。
- 第 3 节给出变量块与 `run` 留痕函数。其余各节的命令都假定这两块已在当前 shell 里执行过。
- 执行顺序固定：阶段 0 到场核对 → 阶段 1 零运动测量 → 阶段 2 制造拒绝记录 → 阶段 3 受控试动 → 阶段 4 配置样例。
- 阶段 2 与阶段 3 互斥，按宿主版本二选一，见第 6 节。
- 拿到非 0 退出码看第 9 节与第 12 节；交回前按第 11 节逐条勾选。

一页总览：

| 阶段 | 时长 | 是否动机器人 | 产物 | 退出码判据 |
| --- | --- | --- | --- | --- |
| 0 到场核对 | 10 分钟 | 否 | `$S4DIR/stdout/0.*` | 两条 `--help` 均 exit 0 |
| 1 零运动测量与指纹 | 本文未设预算 | 否 | `capture/fingerprint-$STAMP.json`、`rtt/rtt-net-$STAMP.csv`、`rtt/rtt-deploy-$STAMP.csv` | 三次运行均 exit 0；宿主为 v1 时 1.1 必 exit 6，见第 6 节 |
| 2 制造代码 Gate F 拒绝 | 本文未设预算 | 否 | `$S4DIR/stdout/2.*` | exit 6，零命令 |
| 3 Mode B 受控试动 | 本文未设预算 | 是，单臂单关节 | `tracking/tracking-$STAMP.csv` 与 report 原文 | exit 0；exit 8 按第 12 节处置 |
| 4 配置样例采集 | 本文未设预算 | 否 | `configs/template.yaml` 与 `configs/site.yaml` | 两份文件能一起被 `load_settings` 读通，不抛 `ConfigError` |

只有阶段 0 有时间预算，来源是本次任务的约定。其余阶段的实际耗时记进 `$S4DIR/stdout/` 存档，本文不设预算。

## 1 代码 Gate A–F 清单（现场要过的六道检查）

本文里的 Gate A–F 指 `benchmarks/alohamini_io/measure_latency.py` 里的运行时门禁，即发命令前的六道检查。

| 代码 Gate | 判什么 | 不通过时 |
| --- | --- | --- |
| A | `--enable-motion` 与 `--joint` 两个参数都给了 | exit 6，在建客户端之前就拒，零网络零命令 |
| B | 目标关节出现在首帧观测里 | exit 6 |
| C | 宿主 metadata 的 `robot_model` 非空，且与 `--robot-model` 相等 | exit 6 |
| D | 该关节基线反馈是有限数 | exit 6 |
| E | `delta_deg > 0` 且 `steps >= 1`，且 `delta_deg × steps <= 20.0` | exit 6 |
| F | ① metadata `schema_version` 必须等于 `2`；`True` 与字符串 `"2"` 被拒，**浮点 `2.0` 实现上会被放行**；② 该关节的 deg-per-unit 能从 v2 校准表算出 | exit 6，零命令 |

Gate F 第①条的判断式在 `benchmarks/alohamini_io/measure_latency.py`，原文 `isinstance(schema_version, bool) or schema_version != HOST_METADATA_SCHEMA_VERSION`（`HOST_METADATA_SCHEMA_VERSION = 2`）。`2.0 != 2` 在 Python 里为 `False`，所以浮点 `2.0` 与整数 `2` 同路放行；`"2" != 2` 为 `True`，字符串被拒。

代码 Gate A 在 `main` 里、建客户端之前执行；B–F 在 `_tracking_preflight` 里、第一条 `send_action` 之前执行。B–F 的顺序是 B、C、D、E、F。

## 2 硬前置条件（不满足就别开始）

| 编号 | 条件 | 怎么确认 | 不满足怎么办 |
| --- | --- | --- | --- |
| P0 | 宿主广播的 `robot_metadata.schema_version` 必须是整数 2 | 跑阶段 1.1 的 `capture`，exit 0 且 JSON 里 `robot_metadata.schema_version` 是 `2`；到场前可先用本节的两条 SSH 命令读 Pi 上的代码预判 | 见下面「P0 阻塞」小节 |
| P1 | 真机 AlohaMini 2 Pro 在场，物理断电开关可用 | 目视加手测 | 停止 |
| P2 | Pi 上 Host 进程常驻 | 阶段 0 的 `0.5-host-procs` 与 `0.6-ports` | 起 Host |
| P3 | PC 与 Pi 同网段 | 阶段 0 的 `0.1-ping` | 修网络 |
| P4 | 现场授权（谁授权、授权范围、谁在旁） | 口头确认，把结论写进 `"$S4DIR/stdout/"` 的现场记录（时间、人员、授权范围） | 停止 |
| P5 | 轨迹空间足够：Mode B 累计行程可达 20° | 卷尺与目视确认该关节正向 20° 无干涉 | 减小 `--steps` |

### P0 阻塞

判断顺序分两层：先用 SSH 读 Pi 上的 vendor 代码做预判，再决定本窗口做哪几个阶段；最终判据仍是 `capture` 的实测结果。

**已实测记录（2026-10-25 UTC 07:00 前后，Pi `10.34.170.98`，来源 `stdout/0.2-pi-commit-20261025T070057Z.log`）**：该时点 vendor 仓库 `/home/pi5/lerobot_alohamini` 的 HEAD 是 `ec65318f`（当日 UTC 09:33 之后已换成 `6aee1f81`，见后文「已关闭」段），`src/lerobot/robots/alohamini/alohamini_host.py` 第 129 行为 `"schema_version": 1,`。即 **P0 未通过，且是源码级的 1**，不是「代码 2、运行时 1」的组合。该 Pi 的 git 工作区另有 6 个未提交改动（`docs/alohamini/commands.md`、`examples/alohamini/{calibrate_bi,record_bi,teleoperate_bi}.py`、`src/lerobot/robots/alohamini/config_alohamini.py`、`tests/robots/test_alohamini_ros_compat.py`），未清理、未触碰。第 8 节三个配置字段的源码默认值取自这份工作区文件。

同一日运行时实证（2026-10-25T07:41Z）：在 Pi 上执行阶段 2 的 Gate F 命令，门禁拒绝，报文原文为

```text
Gate F refused: the host metadata schema_version is 1, expected 2.
  The calibration table under motors.<motor> would come from another metadata layout, so its
  fields (range_min / range_max / resolution / normalization) and every degree figure derived
  from them are unverified; nothing was commanded.
```

退出码 `6`，全程零运动命令。至此 P0 由「源码级为 1」升级为「运行时实测为 1」，两级证据一致。

**2026-10-25 当日已关闭，该时点 HEAD 为 `6aee1f81`**：Pi 上的运行时升级到含 metadata v2 的版本，完全跑通的路径 `/home/pi5/lerobot_alohamini`，HEAD `6aee1f81d503be98972d605ed32189221a674607`（UTC 09:33:54 的采指纹已在这版上完成，见 `capture/fingerprint-merged-6aee1f81.json` 的 `captured_utc`；升级前 UTC 07:00 前后的 HEAD 是 `ec65318f`）。旧部署原样备份在 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`（`ec65318f` 加 6 个未提交改动）。关闭判据：`capture` exit 0，`robot_metadata.schema_version` 为 `2`，18 个电机的 `homing_offset` 与 `resolution` 全部有值；改名后厂商测试 `60 passed`。上面那段 v1 记录保留为历史，下文 P0 判断规则与分支照旧适用。

下面两条命令用到第 3 节定义的 `PI` 与 `run`。先执行第 3 节的变量块与函数定义，再回到这里。

```bash
run 0.2-pi-commit ssh "$PI" 'git -C /home/pi5/lerobot_alohamini rev-parse HEAD'
run 0.3-pi-schema ssh "$PI" 'grep -n "schema_version" ~/lerobot_alohamini/src/lerobot/robots/alohamini/alohamini_host.py'
```

判断规则：

- 只认 `build_robot_metadata` 函数体里的那一行。`camera_stream.py` 的 `CAMERA_STREAM_SCHEMA_VERSION` 是摄像头的另一套版本号，不要拿它判断 P0。按目录 grep 会把这个常量一起捞出来（实测多一行 `camera_stream.py:74: "schema_version": CAMERA_STREAM_SCHEMA_VERSION,`），所以 `0.3` 只查单文件。
- `0.3` 的输出里，`build_robot_metadata` 那段是 `"schema_version": 2,` → P0 视为可通过，继续做阶段 1 与阶段 3。
- `0.3` 读到 `1` → P0 不通过，按下面「P0 未解决」分支执行。本窗口只做阶段 0、阶段 1 的 1.2 与 1.3、阶段 2 的 v1 路径、阶段 4。
- 这两条只是**读代码**，不等于运行时真的广播 `2`。代码硬编码 `2` 而运行时广播 `1` 的组合在原理上存在（Pi 上的 Host 可能从别的路径启动、或用的是更旧的镜像）。最终仍以阶段 1.1 的 `capture` exit 0 与 JSON 里 `robot_metadata.schema_version` 的值为准。
- 命令路径以实际为准：先用 `0.2` 拿到提交记录，再用下面这条确认文件存在：

```bash
run 0.4-pi-ls ssh "$PI" 'ls /home/pi5/lerobot_alohamini/src/lerobot/robots/alohamini/'
```

路径不符时按实际路径调整 `0.3` 后重跑，把实际路径与该次输出记进 `$S4DIR/stdout/`（`run` 已自动留下 `.cmd`、`.log`、`.exit`）。

要看 `0.3` 命中行前后的上下文时，先定位再取，不要写死区间：

```bash
run 0.3b-schema-context ssh "$PI" 'f=~/lerobot_alohamini/src/lerobot/robots/alohamini/alohamini_host.py; n=$(grep -n "\"schema_version\"" "$f" | head -1 | cut -d: -f1); sed -n "$((n-25)),$((n+15))p" "$f"'
```

厂商文件行号随提交变化，一律以 grep 命中的行号为准，不要记死行号。2026-10-25 实测：`build_robot_metadata` 在约 111 行，`"schema_version"` 在 129 行；按固定区间 `sed -n "160,200p"` 取到的是 `main()` 的 argparse，不是这个函数。

本小节产物：`$S4DIR/stdout/` 下的 `0.2-pi-commit-*`、`0.3-pi-schema-*`、`0.3b-schema-context-*`、`0.4-pi-ls-*`。

**本机无法预先确认，只能到现场实测。** `robot_metadata.schema_version` 由 Pi 上的 Host 进程广播，取值取决于 Pi 上的 vendor 代码，要 SSH 连到 Pi 才看得到；本机（未连 Pi）做不到。

上一次现场（证据目录 `field-run-summary.md` 第 4 节）记录：`capture` 退出码 6，报错原文 `invalid robot metadata: schema_version=1, expected 2.`。这说明 Pi 上的 Host 仍在广播 `schema_version: 1`。

厂商代码里该值硬编码为 `2`（旧仓库 `src/lerobot/robots/alohamini/alohamini_host.py:181`，原文 `"schema_version": 2,`）。它写在 metadata 字典字面量里，没有配置项，也没有常量名。Pi 广播 1 说明 Pi 上的 vendor 代码是更旧的提交。

宿主仍是 v1 时，后果是确定的：

- `capture` 必 exit 6，且不写盘。
- 代码 Gate F 必然拒绝，`--mode tracking` 一条命令都发不出。

结论：本窗口能否做受控试动，取决于 P0 是否先解决。解除方式是把 Pi 上 `/home/pi5/lerobot_alohamini` 更新到与本机 vendor 提交一致的版本。这一步涉及远端 git 操作，需要单独授权。

本文档不提供升级步骤：两仓库文档里找不到 `schema_version` 从 1 升到 2 的步骤。

## 3 远程与本地分工

现场凡是 Pi 侧的信息（Host 进程、端口、vendor 代码、配置实际取值），统一用 `ssh` 取，不靠口头转述。理由有两条：口头转述没有落盘证据，出错时无法回溯；`run` 函数会把命令原文、输出原文、退出码一起写进 `$S4DIR/stdout/`，事后能复核。

### 变量块

先建一个会话内的变量块。粘进当前 shell 一次，后面所有命令都用它：

```bash
export PI_HOST=<现场确认的 Pi 地址>
export PI="pi5@$PI_HOST"
export S4DIR=~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-$(date -u +%Y%m%d)
export STAMP=$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "$S4DIR"/{capture,rtt,tracking,configs,stdout}
```

- `PI_HOST` 是唯一需要现场填的值。上次现场记录的是 `10.34.170.98`（来源 `field-run-summary.md` 第 1 行「远端」），先按这个值试，连接检查见本节末尾。
- `S4DIR` 按 UTC 日期建，形式 `s4-20261022`，与既有证据文件 `rtt-net-20261022-162919.csv` 同一命名法。
- `STAMP` 是本次会话的时间戳，形式 `YYYYmmddTHHMMSSZ`。正文里凡出现 `$STAMP` 的地方，指的都是这一个值；同一次会话内的全部产物共用它，不再手填。

### 时间基准：日志里的时间是 UTC

Pi 的系统时区设成 `Europe/London`，日志显示的时间等于 UTC。本地时间是 UTC+8，钟点比日志里的数字大 8 小时：日志里的 `09:5x` 对应本地 `17:5x`。

该结论来自现场 `date` 与 `timedatectl` 的屏幕输出，未落盘到证据目录；证据目录内唯一可核的时间依据是 `date -u` 生成的 UTC 时间戳。

因此本文凡涉及现场时间，都写明是 UTC 还是本地时间。`$S4DIR` 与 `$STAMP` 按 UTC 生成（`date -u`），产物文件名里的时间也一律是 UTC。

### 在哪跑

| 命令类别 | 在哪跑 | 形式 |
| --- | --- | --- |
| 脚本运行（`capture_robot_fingerprint.py`、`measure_latency.py`） | Mac | `run 1.1-capture uv run --no-sync python …` |
| 产物文件（JSON、CSV、YAML） | Mac，落 `$S4DIR` | 由 `--out` / `--csv` 指定 |
| Pi 上的 Host 进程与端口（PID、5555、5556） | Pi | `run 0.5-host-procs ssh "$PI" '…'` |
| Pi 上的 vendor 代码（提交、`schema_version` 行、字段默认值） | Pi | `run 0.2-pi-commit ssh "$PI" '…'` |
| 配置取值（`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz` 的实际生效值） | Pi 取原值，写进 Mac 上的 YAML | `run 4.1`–`4.3`，见第 8 节 |

### `run` 包装函数

后面每条要留痕的命令都过这个函数。逐字照抄，粘进同一个 shell：

```bash
# 用法：run <步骤号> <命令...>
# 产物：$S4DIR/stdout/<步骤号>-<STAMP>.{cmd,log,exit}
run() {
  local step="$1"; shift
  local base="$S4DIR/stdout/$step-$STAMP"
  printf '%s\n' "$*" > "$base.cmd"
  "$@" >"$base.log" 2>&1
  local code=$?
  echo "$code" > "$base.exit"
  cat "$base.log"
  echo "[exit $code] $base"
  return $code
}
```

每次调用产出三个文件：`.cmd` 是命令原文，`.log` 是 stdout 与 stderr 的合并原文，`.exit` 是退出码。

为什么不用 `| tee`：管道会把 `$?` 变成 `tee` 的退出码，拿到的永远是 `tee` 的 0，与「拿非 0 退出码看第 9 节」直接冲突。这个函数先把输出落盘再 `cat` 回显，退出码取自被调命令本身。

`.cmd` 只记录命令骨架：`$*` 展开会吃掉引号，heredoc 正文也不在里面。需要回放时以 `.log` 与本节正文里的命令原文为准，`.cmd` 只用来对齐步骤号。

用法示例，三条：

```bash
# 本地脚本
run 1.1-capture uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --out "$S4DIR/capture/fingerprint-$STAMP.json"

# Pi 侧命令
run 0.5-host-procs ssh "$PI" 'pgrep -af "alohamini_host|lerobot"'

# 失败用例：验证留痕链路本身
run 0.9-probe false
```

第三条应返回 1，并写出 `$S4DIR/stdout/0.9-probe-$STAMP.exit`，内容为 `1`。写出来的不是 `1`，先修 `S4DIR` 与函数定义，再往下做。

步骤号的第一位是阶段号，不是本文节号：阶段 0 用 `0.*`，阶段 1 用 `1.*`，阶段 4 用 `4.*`。

### SSH 连接检查与连不上时的处置

```bash
run 0.0-ssh-check ssh -o ConnectTimeout=5 "$PI" 'echo ok'
```

判据：`.log` 里是 `ok`，`.exit` 是 `0`。

连不上时按顺序处置，每一步都过 `run`：

1. 先确认网络层是否通：

```bash
run 0.1-ping ping -c 3 "$PI_HOST"
```

全丢包是网络问题，有回包但 `ssh` 失败是 ssh 层问题。

2. 试 mDNS 名：

```bash
run 0.0b-ssh-mdns ssh -o ConnectTimeout=5 "pi5@pi5.local" 'echo ok'
```

通了就把地址改成 `pi5.local`：

```bash
export PI_HOST=pi5.local
export PI="pi5@$PI_HOST"
```

然后重跑 `0.0-ssh-check`。

3. 问现场要地址：问清 Pi 当前的 IPv4 地址，按第 2 步的方式改 `PI_HOST` 与 `PI` 后重跑 `0.0-ssh-check`。
4. 现场答不上来时，在路由器管理页查 DHCP 列表，找主机名 `pi5` 或对应 MAC 的地址。

本节产物：`$S4DIR/stdout/0.0-ssh-check-*`、`0.0b-ssh-mdns-*`、`0.9-probe-*`。

## 4 阶段 0：到场核对（10 分钟）

`$S4DIR` 与 `$STAMP` 由第 3 节的变量块给出，本节不重复定义。证据目录已由那个块建好，本节不再新建。

按顺序执行：

1. 网络可达：

```bash
run 0.1-ping ping -c 3 "$PI_HOST"
```

判据：3 个包全回，丢包 `0%`。

2. Host 进程在跑，记下 PID：

```bash
run 0.5-host-procs ssh "$PI" 'pgrep -af "alohamini_host|lerobot"'
```

判据：`.log` 里至少一行，行首有 PID。没有输出说明 Host 没起，先起 Host 再往下。

`pgrep` 默认按扩展正则解释，`|` 直接写，不要转义。写成 `\|` 时 ERE 把反斜杠加竖线当字面量，有进程也可能匹配不到。

3. 工具存在性自证（放在 `0.6` 之前跑）：

```bash
run 0.4b-tools ssh "$PI" 'command -v ss netstat; echo "---"; ss -ltn | head -20'
```

这一条只回答一个问题：`ss` / `netstat` 在不在。`command -v` 打出路径即存在；`ss -ltn` 那一段不管有没有端口，都能看出工具本身能不能跑。

4. 5555 与 5556 在听：

```bash
run 0.6-ports ssh "$PI" 'ss -ltnp | grep -E ":5555|:5556"'
```

Pi 上没有 `ss` 时（`.log` 里出现 `ss: command not found`，退出码非 0）退回 `netstat`：

```bash
run 0.6b-ports-netstat ssh "$PI" 'netstat -ltnp | grep -E ":5555|:5556"'
```

判据：两个端口各至少一行 `LISTEN`。

`0.6` 空输出有两种可能：工具缺失，或没有监听。`0.4b-tools` 用来区分这两种。**空输出本身不构成「Host 未运行」的证明**，要结合 `0.5` 的进程检查一起判。2026-10-25 实测：`ss` 与 `netstat` 在 Pi 上都存在（`/usr/bin/ss`、`/usr/bin/netstat`），`0.6` 两条输出为空是因为确实没有 LISTEN 的 5555/5556。该结论来自 2026-10-25 的独立复核（`command -v` 与 `ss -ltn`），不是通过 `0.4b-tools` 步骤本身取得——`0.4b` 是为后续现场补的规范化步骤。

5. Pi 地址核对（IP 变更时用）：

```bash
run 0.10-pi-ip ssh "$PI" 'hostname -I'
```

这一条需要先能连上。连不上时依次试 mDNS 名 `pi5.local`（命令见第 3 节）、问现场要地址、在路由器管理页查 DHCP 列表。

6. 两个脚本的参数能解析：

```bash
run 0.7-capture-help uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --help
run 0.8-measure-help uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --help
```

判据：两条均 `.exit` 为 `0`。

人工确认项（没有机器判据）：

- [ ] 断电开关位置已知，手能一次按到。这一条只能人工确认，没有机器判据。
- [ ] 机器人已上电。这一条只能人工确认，没有机器判据。
- [ ] 单臂、空场、无负载三项按第 7 节在阶段 3 开跑前再确认一次。
- [ ] 每条产出文件的命令都显式传了 `--out` 或 `--csv`。`run` 写的 `.cmd/.log/.exit` 是留痕，不算阶段产物。

默认文件名的坑：两个脚本的默认产物目录是 `artifacts/alohamini-io/`，相对当前工作目录解析，不以仓库根为基准。文件名里的时间戳只到秒（格式 `YYYYmmddTHHMMSSZ`），同一秒内跑两次会静默覆盖前一份（`write_text` 直接盖写）。结论：一律显式传 `--out` / `--csv`，不要依赖默认文件名。

`--host` 一律写 `"$PI_HOST"`。上一次现场记录的地址是 `10.34.170.98`（来源 `field-run-summary.md` 第 1 行「远端」）。本次先用 `0.1-ping` 确认可达，再用 `0.10-pi-ip` 的输出确认这个地址就是这台 Pi，然后往下做。

本阶段产物：`$S4DIR/stdout/` 下的 `0.1-ping-*`、`0.5-host-procs-*`、`0.4b-tools-*`、`0.6-ports-*`（或 `0.6b-ports-netstat-*`）、`0.7-capture-help-*`、`0.8-measure-help-*`、`0.10-pi-ip-*`。

## 5 阶段 1：零运动测量与指纹（不动机器人）

三条命令都不构造任何运动指令：`capture` 只请求观测，`measure --mode rtt` 只采样观测。三条都过 `run`，stdout 与 stderr 合并落到 `$S4DIR/stdout/<步骤号>-$STAMP.log`，退出码落到同名 `.exit`。

1.1 指纹：

```bash
run 1.1-capture uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --out "$S4DIR/capture/fingerprint-$STAMP.json"
```

判据：exit 0，`$S4DIR/capture/fingerprint-$STAMP.json` 写出。宿主为 v1 时这一条必 exit 6 且不写盘，见第 6 节。

1.2 RTT 形态（请求 1000 Hz、窗口 1）：

```bash
run 1.2-rtt-net uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 --csv "$S4DIR/rtt/rtt-net-$STAMP.csv"
```

判据：`exit 0` 且 CSV 有数据行。若返回 `9`，说明该时点这份 vendor 的 client 不发布 `latest_request_timing`（升级前 HEAD `ec65318f` 属此情形），rtt 无法测量——这是设计行为，按第 13 节记录，不当故障、不重试改参数。升级后 HEAD `6aee1f81` 提供该面，`3.2-rtt` 实测 exit 0。若返回 `3`，是连接问题，按第 12 节处置。

在 Pi 上执行时必须把 `PYTHONPATH` 指向被测代码，否则会 import 到已安装的旧版本。采集脚本的 `default_client_factory()` 直接 `import lerobot`，不经过本库 adapter；不设 `PYTHONPATH` 时表现为指纹计时字段全 `null`，或 `measure --mode rtt` 返回退出码 `9`。这两类现象看着像代码缺陷，实际是跑错了代码。退出码 `9` 就是这样在真机上首次触发的，这条不再只有单元测试背书。

1.3 部署形态（请求 30 Hz、窗口 3）：

```bash
run 1.3-rtt-deploy uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 30 --request-window 3 --samples 500 --csv "$S4DIR/rtt/rtt-deploy-$STAMP.csv"
```

判据：exit 0，`$S4DIR/rtt/rtt-deploy-$STAMP.csv` 写出。

1.1 的默认值：`--samples 5`、`--timeout-s 30.0`、`--port 5556`。1.2 与 1.3 的 `--samples` 默认是 `500`，本文照样写出来，便于现场比对命令原文。

两条读法必须先定下来：

- `--rate-hz` 低于 200 时，`age_s` 的主体是响应在客户端 socket 里等下一次 poll 的空闲时间，所以它测的是「端到端观测滞后」，不能当网络往返时间读。要测 RTT 必须用 `--rate-hz 1000`，上一次现场就是这么跑的。脚本在低于 200 Hz 时自己打警告，警告原文见 `field-run-summary.md` 第 5b 节；打了警告不等于失败，但它封住了这一组数值当 RTT 读的用法。
- `--request-window 1` 是 RTT 形态（无预取等待可减）；`--request-window 3` 是部署形态（控制栈实际用的窗口）。两者不可混为一谈，也不可互相替代。

参考值（上一次现场，2026-10-22）：1.2 组 `achieved_rate_hz=49.0`（请求 1000 Hz），1.3 组 `achieved_rate_hz=30.1`（请求 30 Hz）。来源 `field-run-summary.md` 第 5 节。实际到达率受 Host 主循环节拍限制。

本阶段产物：`capture/fingerprint-$STAMP.json`、`rtt/rtt-net-$STAMP.csv`、`rtt/rtt-deploy-$STAMP.csv`，以及 `stdout/1.1-capture-*`、`stdout/1.2-rtt-net-*`、`stdout/1.3-rtt-deploy-*`。

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

实测（2026-10-25 UTC 07:00 前后，该时点 Pi HEAD 为 `ec65318f`）：`build_robot_metadata` 给每个电机只写 6 个键——`id`、`model`、`normalization`、`drive_mode`、`range_min`、`range_max`，**没有 `homing_offset`，也没有 `resolution`**。`robot_metadata` 顶层是 `schema_version`、`robot_model`、`motors`、`cameras` 四项。v2 宿主（同日 UTC 09:33 之后的提交 `6aee1f81`）上 18 个电机的这两个字段全部有值，无 `null`。文档其它地方提到的「8 个字段」是代码里 `validate_metadata` 认识的完整字段集，不是 Pi 当前实际发出的字段集。

### 跨机 RTT 的前置：PC 侧隔离环境

跨机 RTT 指从 PC 到 Pi 走真实网段的往返时延（白话：一个请求发出去到响应回来，在路上花掉的时间）。现有两份 rtt CSV 都是 Pi 本机回环（`host=127.0.0.1`），量不到这一段，缺口见第 13 节。要跑跨机那一次，解释器里必须有 vendor 客户端 `lerobot.robots.alohamini.alohamini_client.AlohaMiniClient`；而本库主环境不含 `lerobot` —— `adapter/protocol.py` 是全库唯一允许 import `lerobot` 的地方（AGENTS.md §4），主环境还要按 ADR D8 保持 numpy 2.3.x。`benchmarks/alohamini_io/measure_latency.py` 的 `default_client_factory()`（`:1123`，惰性 `import` 在 `:1129`–`:1130`）在运行期才 import 该客户端，所以 `--help` 与单元测试都不需要它，真正跑 `--mode rtt` 才需要。

做法：在仓库内 `.venvs/` 下另建一个隔离环境（`.venvs/` 已加进 `.gitignore`），装厂商客户端的冻结快照，主环境不动。2026-10-25 实测结论：环境建好、4 条 import / 版本检查与 1 条 `--help` 均通过、主环境未污染；**仍未做任何 `connect()`**，真实跨机 RTT 仍未测。

**环境事实**

| 项 | 值 |
| --- | --- |
| 环境路径 | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot`（仓库内 `.venvs/` 下，已被 `.gitignore` 的 `.venvs/` 规则忽略，不进 `git status`） |
| 解释器 | `Python 3.13.13`（`pyvenv.cfg` 的 `home = /opt/miniconda3/bin`） |
| 与系统 site-packages 隔离 | `include-system-site-packages = false` |
| `lerobot` 版本 | `0.6.1`（厂商本地副本 `pyproject.toml` 的 `version`；同一文件写 `requires-python = ">=3.12"`） |
| 已装包数 | `pip freeze` 输出 44 行 |
| 整目录体积 | `du -sh` = 881M |
| numpy | `2.2.6`。`lerobot` 核心依赖钉 `numpy>=2.0.0,<2.3.0`，隔离环境解析到 2.2.6；主环境是 2.3.5，两者 site-packages 完全隔离 |
| 额外补装 | `pyzmq 27.2.0`（`site-packages/zmq` 6.1M），原因见下面安装第 3 步 |
| 安装形态 | 非 editable：从 `/tmp/lerobot-src-probe` 源码快照构建出的 `/tmp/lerobot-wheel/lerobot-0.6.1-py3-none-any.whl` 安装；也可按下文第 2 步从源码快照现构建 |

**下载体积实测**（`pip download` 落盘后按字节统计，不是估值）

| 项 | 值 |
| --- | --- |
| 总量 | 197.77 MB / 43 包 |
| 拆分 | 42 个依赖 wheel 196.05 MB（205,577,393 B）+ `lerobot` 本体 wheel 1.72 MB（1,804,000 B） |
| 前三大 | `torch 2.11.0` 76.87 MB、`cmake 4.1.3` 46.96 MB、`opencv-python-headless 4.13.0.92` 44.10 MB；三者合计 168 / 196 MB |
| CUDA | 无任何 `nvidia-*` 包，无 CUDA 轮子；平台标签只有 `macosx_*` 与 `py3-none-any` |
| Python 约束 | `lerobot >=3.12`、`torch >=3.10`、`torchvision >=3.10,!=3.14.1`、`numpy >=3.10`、`opencv >=3.6`，均无上界；所选解释器 CPython 3.13.13 在范围内 |

**安装命令原文**（分步）

```bash
# 1) 建环境（仓库内 `.venvs/`，已被 `.gitignore` 忽略，Python >= 3.12）
python3 -m venv /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot
# 2) 从厂商本地副本的 /tmp 快照装，绝不 pip install -e 厂商仓库本体
#    （否则 .egg-info 会落到 /Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini 里）
rsync -a --exclude='.git' --exclude='.venv' --exclude='*.egg-info' \
  /Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini/ /tmp/lerobot-src-probe/
/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/pip install --find-links /tmp/lerobot-wheels /tmp/lerobot-src-probe
# 3) 唯一需要补的包：lerobot 只把 pyzmq 列为可选 extra，而 alohamini 客户端 __init__ 里 import zmq
/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/pip install "pyzmq>=26.2.1,<28.0.0"
```

本次（2026 环境迁移重建）实际执行的是纯离线预构建 wheel 安装：`.../bin/pip install --no-index --find-links /tmp/lerobot-wheel --find-links /tmp/lerobot-wheels lerobot==0.6.1`，退出码 `0`，43 个包全部来自缓存。`pyzmq` 那一步由 pip 本地 HTTP 缓存命中（`Using cached pyzmq-27.2.0-cp312-abi3-macosx_10_15_universal2.whl`），无下载行。

第 2 步的依赖走本地 `--find-links` 与 pip 缓存：主安装输出里 `grep -c '^Downloading'` = 0。第 3 步是这套环境**唯一**联网的一步：`lerobot` 把 `pyzmq` 只列在可选 extra 里（`pyzmq-dep = ["pyzmq>=26.2.1,<28.0.0"]`），核心依赖不含，而 `alohamini_client.py` 在 `__init__` 里惰性 `import zmq`。

**验证命令原文**（4 条 import / 版本检查加 1 条 `--help`；实测输出照抄）

| 检查 | 命令 | 实测输出 |
| --- | --- | --- |
| 版本 | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python -c "import lerobot; print(lerobot.__version__)"` | `0.6.1` |
| 版本区间 | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python -c "from importlib.metadata import version; from packaging.specifiers import SpecifierSet; v = version('lerobot'); print('version', v); print('in range', SpecifierSet('>=0.6.1,<0.7').contains(v))"` | `version 0.6.1`、`in range True` |
| 目标接口 | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python -c "from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient; from lerobot.robots.alohamini.config_alohamini import AlohaMiniClientConfig; print('client import ok', AlohaMiniClient, AlohaMiniClientConfig)"` | `client import ok <class 'lerobot.robots.alohamini.alohamini_client.AlohaMiniClient'> <class 'lerobot.robots.alohamini.config_alohamini.AlohaMiniClientConfig'>` |
| 第三方面 | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python -c "import zmq, cv2, numpy; print('zmq', zmq.__version__, 'cv2', cv2.__version__, 'numpy', numpy.__version__)"` | `zmq 27.2.0 cv2 4.13.0 numpy 2.2.6` |
| CLI | 仓库根下 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --help` | exit `0`，459 行 usage |

**主环境未污染，厂商仓库未被写入**（装前与装后各取一次，同一台 Mac）

| 检查 | 装前 | 装后 |
| --- | --- | --- |
| numpy | `2.3.5` | `2.3.5` |
| pytest 汇总 | `1275 passed, 2 skipped`（`8.92s`） | `1275 passed, 2 skipped`（`9.18s`） |
| `uv pip list` 行数 | 49 | 49 |
| 两次清单的 `diff` | — | 空 |
| `git -C <厂商本地副本> status --porcelain` | 6 行 `M` | 逐字相同，无新增 `??` |

厂商仓库那 6 行改动在本次工作**开始前**就是脏的（含 `src/lerobot/robots/alohamini/config_alohamini.py`），不是这次装环境造成的。

**跑跨机 RTT 的完整命令原文**（在仓库根下执行；`$PI_HOST`、`$S4DIR`、`$STAMP` 一律用第 3 节变量块的定义，这里不另立变量）

```bash
# 零运动 RTT 形态：请求 1000 Hz、窗口 1（与 Pi 本机那两次同口径，才可比）
/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py \
  --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 \
  --csv "$S4DIR/rtt/rtt-net-$STAMP.csv"
```

这条命令与本节 1.2 的 `1.2-rtt-net` 的唯一差别是解释器：把 `uv run --no-sync python` 换成 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python`，因为主环境没有 `lerobot`。参数口径（`--mode rtt`、`--rate-hz 1000`、`--request-window 1`、`--samples 500`）必须与本机回环那两次一致，否则两组数字不可比。要按第 3 节留痕，把整条命令套进 `run` 即可（本文未给这一步编号）。`--csv` 名字与 1.2 相同：同一次会话里两条都跑会互相覆盖，跨机这一条要单独留存就先在独立的 `$STAMP` 会话里跑。

判据：`exit 0` 且 CSV 有数据行。返回 `9` 或 `3` 的处置与 1.2 相同，见本节 1.2 的判据段与第 12 节。

**已知限制**（逐条，不要合并读）

1. 这套环境是**冻结快照**：装的是构建出的 wheel，厂商仓库后续改动不会反映进来，要更新得重装。
2. 装进去的是 `/tmp/lerobot-src-probe`（厂商工作树的 rsync 快照；本次重建用的是由该快照构建的 `/tmp/lerobot-wheel/lerobot-0.6.1-py3-none-any.whl`，同源，结论不变），而厂商工作树当时有 6 个未提交改动 —— 所以这个环境里的 `AlohaMiniClientConfig` 反映的是工作树状态，不是某个提交。
3. 只能在 PC 侧（macOS arm64）跑。Pi 是 Linux aarch64，需要另一套 wheel，这个环境整体不可搬移。
4. 只验到 import 与 `--help`，**未做任何 `connect()`**。真实跨机 RTT 仍未测，第 13 节那条缺口不因本节而关闭。
5. 第 3 步的 `pyzmq` 来自 PyPI（唯一联网步骤）。要完全离线复现，需先把 pyzmq wheel 补进 `/tmp/lerobot-wheels`。

**不要**把 `lerobot` 装进本仓库的 `.venv`：它的核心依赖钉 `numpy>=2.0.0,<2.3.0`，会把主环境的 numpy 从 2.3.5 降级，冲撞 `docs/ADR.md` D8（AGENTS.md §7 已决：主环境停在 numpy 2.3.x）。

## 6 阶段 2：制造一次代码 Gate F 的真实拒绝记录

**互斥声明**：阶段 2 依赖宿主仍是 v1，才用版本判断触发 Gate F 的第一段；阶段 3 要求宿主为 v2，Gate F 的第一段才过得去。同一窗口只能做其中一个，按第 2 节 P0 的判断二选一。

| 宿主 `robot_metadata.schema_version` | 能做的阶段 | 做不了的阶段 |
| --- | --- | --- |
| `1`（v1） | 阶段 0、阶段 1 的 1.2 与 1.3、阶段 2 的 v1 路径、阶段 4 | 阶段 1.1 的 `capture`（exit 6）、阶段 3 |
| `2`（v2） | 阶段 0、阶段 1 全部、阶段 2 的 v2 路径、阶段 3、阶段 4 | 阶段 2 的 v1 路径（版本判断不再拒绝） |

### v1 路径：用版本判断触发

代码 Gate F 在第一条 `send_action` 之前执行，所以它的拒绝路径结构上零命令，可以安全地制造。

2026-10-25 已实际执行本条路径并取得拒绝记录，命中 Gate F 第一段（版本判断），报文见第 2 节实测记录。

步骤号与证据文件的对照：本文的 `2.0-refusal-v1` 对应证据文件 `stdout/2.2b-gateF-attempt-20261025T074158Z.{cmd,log,exit}`（exit `6`、零命令）；`2.1-probe` 未按本节命令运行，同一项检查实际由 `stdout/5.1-probe21-20261025T094457Z.log` 完成，输出为 18 个电机逐项 `missing=[] bad=[]`。

宿主仍是 `schema_version=1` 时，跑

```bash
run 2.0-refusal-v1 uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --steps 1 --csv "$S4DIR/tracking/f-gate-refusal-$STAMP.csv"
```

代码 Gate 按 A、B、C、D、E、F 的顺序执行，A 到 E 会先跑。它们是否全过取决于 Pi 上 metadata 的实际内容（`robot_model` 是否非空且与 `--robot-model` 相等、关节名是否在首帧观测里、基线是否有限），本机无法预先验证。A 到 E 全过之后，拒绝点落在 F 的版本判断，得到 exit 6、零命令。

命令带 `--csv` 只为统一留痕与路径完整：拒绝时脚本不写任何文件，这个路径不会产生文件。判据是 `.exit` 内容为 `6`，不看有没有 CSV。

记录要求：`.cmd`、`.log`、`.exit` 三个文件由 `run` 自动写出，对应命令原文、stderr 原文、退出码三项。

不要为了制造 F 的拒绝去改标定文件。

### v2 路径：先探校准字段是否齐全

宿主已经是 v2 时，F 的第一段放行，第二段（该关节的 deg-per-unit 能否从 v2 校准表算出）成为唯一的拒绝点。先读阶段 1.1 的 `capture` JSON，逐电机看校准字段是否齐全：

```bash
run 2.1-probe python - <<'PY'
import json, os
from pathlib import Path
s4 = Path(os.environ["S4DIR"])
paths = sorted(s4.glob("capture/fingerprint-*.json"))
assert paths, "先跑阶段 1.1，拿到 fingerprint JSON"
meta = json.loads(paths[-1].read_text())["robot_metadata"]
motors = meta.get("motors") or {}
print(f"motors: {len(motors)}")
for name, fields in sorted(motors.items()):
    missing = [k for k in ("normalization", "drive_mode", "range_min", "range_max", "homing_offset", "resolution")
               if fields.get(k) is None]
    bad = []
    try:
        if fields.get("range_max") is not None and fields.get("range_min") is not None \
           and float(fields["range_max"]) <= float(fields["range_min"]):
            bad.append("range_max<=range_min")
        if fields.get("resolution") is not None and float(fields["resolution"]) <= 1.0:
            bad.append("resolution<=1")
    except (TypeError, ValueError):
        bad.append("non-numeric")
    print(f"{name}: missing={missing} bad={bad}")
PY
```

判断规则：

- 只有当某个电机的 `missing` 或 `bad` 非空时，它对应的关节才会在 Gate F 第二段被拒。
- v1 宿主（2026-10-25 UTC 07:00 前后的提交 `ec65318f`）上，`build_robot_metadata` 给每个电机只写 6 个键（`id`、`model`、`normalization`、`drive_mode`、`range_min`、`range_max`），没有 `homing_offset`，也没有 `resolution`；那时即便把 `schema_version` 改成 2，`2.1-probe` 的输出也会是每个电机 `missing=['homing_offset','resolution']`，第二段对所有关节都成立。
- v2 宿主（同日 UTC 09:33 之后的提交 `6aee1f81`）上 18/18 电机字段齐全，`2.1-probe` 不会报 `missing`，Gate F 第二段不会被触发。这条拒绝路径在 v2 宿主上未验证。
- 记录证据时必须写明命中的是哪一段。两种拒绝要分开：`schema_version` 不是 2 → Gate F 第一段拒绝；换算字段不全 → Gate F 第二段拒绝。
- 若全部电机都齐全（其他厂商版本可能如此），本窗口触发不了 F 的第二段。此时只记录「v2 宿主下 Gate F 未触发」这一事实，理由写进 `TODO.md` 与 `$S4DIR/stdout/`（`5.1-probe21-*` 已在盘上）。**不要为了造记录去改标定文件或伪造数据。**
- 电机名到关节观测键（如 `arm_left_shoulder_pan.pos`）的对应关系本文未核实。现场用该电机的名字加 `--joint` 试一次，命令骨架如下，`--joint` 的取值按 `2.1` 输出里 `missing` 或 `bad` 非空的电机名改：

```bash
run 2.2-refusal-v2 uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --steps 1 --csv "$S4DIR/tracking/f-gate-refusal-$STAMP.csv"
```

`--joint` 写错时由代码 Gate B 拒绝（exit 6，零命令），不会动机器人。

本阶段产物：`$S4DIR/stdout/` 下的 `2.0-refusal-v1-*` 或 `2.1-probe-*`、`2.2-refusal-v2-*`。两种路径都不写 `tracking/` 里的 CSV（拒绝路径零命令、不写盘）。

## 7 阶段 3：Mode B 受控试动（唯一动机器人的阶段）

本节只能在宿主为 v2 的窗口做，与阶段 2 的 v1 路径互斥，见第 6 节。

开跑前再确认，逐条勾选：

- [ ] 现场授权到手，授权范围写明（哪个关节、多少度、谁在旁）
- [ ] 单臂：另一臂不参与本次运动
- [ ] 空场：人物离场
- [ ] 无负载：末端无负载
- [ ] 行程按「1° × 更多步」规划：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:392`）把每步压回 `1.0°`，请求更大值无效；stdout 会同时打印两者，原文 `delta_deg=1.000 (requested 2.000)`。要更大总行程只能加 `--steps`，加大 `--delta-deg` 不产生更大位移
- [ ] 断电开关在手边，可随时按下

命令：

```bash
run 3.1-tracking uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 5 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-$STAMP.csv"
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
| `--settle-tolerance-deg` | `0.05` | 默认 `0.15` 可能让 90% 上升时间测不到：容差不小于 `0.1 × delta`（即 0.1°）时，settle 循环的退出条件比 90% 越线更宽松，于是 `rise_90_ms` 报 `count=0`、`p50=nan`。要测上升时间必须取小于 0.1° 的值 |
| `--settle-timeout-s` | `2.0` | 默认 `1.0` s，20 ms 命令节奏下每步预算偏紧 |
| `--timeout-s` | `60` | 默认 `30` s 对多步加 settle 偏紧，中途超时会以 exit 1 结束 |

`--delta-deg` 的硬上限与 stdout 表现：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:392`，`resolve_delta_deg` 用 `min(delta_deg, MAX_STEP_DELTA_DEG)`）。请求 `--delta-deg 2.0` 时实际按 1.0 执行，report 首部把两者都打印出来，原文形如 `delta_deg=1.000 (requested 2.000)`（格式在 `measure_latency.py:2341-2342`）。要凑更大总行程，只能改成「1° × 更多步」，加大 `--delta-deg` 无效。

关于 `rise_90_time_s` 为空：源码的说法是这一列 can be empty，即**可能**为空。是否为空取决于该关节是否在同一次轮询里越过 90% 线，不是「容差取小了就一定空、取大了就一定不空」。

运动学事实：

- 方向：每步目标是当前实测基线加 `delta_deg`，`delta_deg` 恒正，所以每步朝同一方向累积。
- 脚本不检查关节限位。源码里没有任何对 URDF 或 adapter 限位的比较。方向与累计行程是否安全，只能靠人。
- 反馈失联保护：连续 3 次 `observation_sequence` 不增长就中止（exit 1）。中止前仍会发收尾 hold。
- 收尾 hold 从 `finally` 发，正常结束、命令被拒、反馈死、Ctrl-C 都会发。只有一次 `*.pos` 都没测到时才不发。
- hold 是就地冻结：把实测 `*.pos` 当目标，三轴速度钉 0，反馈带高度就回写实测高度。报告里只写 `hold_issued`，不写「sent」，因为 action socket 是 PUSH/CONFLATE 无确认。

exit 8 表示 hold 被拒，也就是停命令没到。这一条按「机器可能仍在动」处置：先断电，再排查。判定与处置见第 12 节。

### 2026-10-25 真机试动实测（五次运行）

条件：左臂、单关节、每步 1°、场地清空、断电开关在手。证据目录 `tracking/` 下有五份 CSV，一份对应一次运行，下表按 `t_send_monotonic_s` 的先后排列。

时间基准：本小节与第 13 节引用的现场时间取自 Pi 日志，均为 UTC。本地时间为 UTC+8，钟点比日志数字大 8 小时，见第 3 节「时间基准」。

达成率口径：指令行程 = Σ（每步 `cmd_deg` − 该步 `baseline_deg`）；实测净行程 = 末步 `measured_deg` − 首步 `baseline_deg`；达成率 = 实测净行程 ÷ 指令行程。`cmd_deg` 是每步的绝对目标角，各步起点不同，分母按逐步差求和，不用「步数 × 1°」。例：腕部 1.7582° ÷ 2.0000° = 87.9%；首次试动 0.1758° ÷ 3.0000° = 5.9%。

| 次序 | 关节（电机 ID） | 指令行程 | 实测净行程 | 达成率 | 停稳 | 数据文件 |
| --- | --- | --- | --- | --- | --- | --- |
| 1（首次试动） | `arm_left_shoulder_pan`(1) | 3.0°（1°×3 步） | +0.176° | 5.9% | 三步 `settled=False`，误差 0.824° / 1.000° / 1.000° | `tracking/modeB-first-20261025.csv` |
| 2 | `arm_left_wrist_roll`(6) | 2.0°（1°×2 步） | +1.758° | 87.9% | 两步 `settled=True`，误差 0.121° | `tracking/modeB-wrist-20261025.csv` |
| 3 | `arm_left_shoulder_pan`(1) | 3.0°（1°×3 步） | +0.264° | 8.8% | 三步 `settled=False`，误差 0.824° / 1.000° / 0.912° | `tracking/modeB-shoulder-20261025.csv` |
| 4 | `arm_left_shoulder_lift`(2) | 2.0°（1°×2 步） | +0.264° | 13.2% | 两步 `settled=False`，误差 0.912° / 0.824° | `tracking/modeB-lift-20261025.csv` |
| 5 | `arm_left_shoulder_pan`(1) | 2.0°（1°×2 步） | +0.176° | 8.8% | 两步 `settled=False`，误差 0.912° / 0.912° | `tracking/modeB-pan2-20261025.csv` |

读数要点：

- 位移都是 1 tick 的整数倍（1 tick = 0.0879°）：腕部每步约 10 tick，肩部每步约 1 tick。
- 第 1 行与第 3 行命令相同：同一关节、同为 1°×3 步，净行程却是 +0.176° 与 +0.264°，相差 0.088°，正好 1 tick。同一命令重复两次的落点已经不同，单次达成率不能当稳定指标读。
- 运行结束后的回弹量未采集：`tracking/` 里没有运行之后的采样，证据目录也没有对应记录。要量化回弹，需在试动结束后继续读该轴并落盘。
- 五份 CSV 的 `note` 列只有 `ok` 与 `settle budget exhausted` 两种值，没有 `aborted:`，即每次运行都把 `--steps` 走满、没有中途中止。首次试动的 report 原文（`stdout/modeB-first-20261025.stdout`）与此一致：`steps_completed=3`、`abort_reason=None`、`hold_issued=yes`，同次 stderr 0 字节。现场记录这几次运行的退出码均为 0；证据目录里只有首次试动留了 report 与 stderr，没有逐次 `.exit` 文件。
- `rise_90_time_s` 全部为空，原因是 `--settle-tolerance-deg` 默认 0.15 不小于 0.1 × 步长，90% 交点不会先被判定（脚本自带说明）。

本轮只动左臂，右臂未参与，对侧串值未验证。

本阶段产物：`tracking/tracking-$STAMP.csv` 与 `stdout/3.1-tracking-*`。

## 8 阶段 4：配置样例采集（两份 YAML）

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

### 现场取值：三个数值字段用命令取

`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz` 三个字段的取值在 Pi 上，下面五条命令都在 Pi 侧跑：

```bash
run 4.1-host-args ssh "$PI" 'pgrep -af alohamini'
run 4.2-host-cmdline ssh "$PI" 'pid=$(pgrep -f alohamini_host.py | head -1); echo "pid=$pid"; [ -n "$pid" ] && ps -o pid,cmd -p "$pid" && tr "\0" " " < /proc/$pid/cmdline; echo'
run 4.3-host-config ssh "$PI" 'grep -rnE --include="*.py" --exclude-dir=__pycache__ "observation_request_window|watchdog_timeout_ms|max_loop_freq_hz" ~/lerobot_alohamini/src/lerobot/robots/alohamini/ | head -20'
run 4.3b-config-args ssh "$PI" 'grep -n "add_argument" ~/lerobot_alohamini/src/lerobot/robots/alohamini/alohamini_host.py'
run 4.3c-config-classes ssh "$PI" 'sed -n "95,145p" ~/lerobot_alohamini/src/lerobot/robots/alohamini/config_alohamini.py'
```

`4.2` 的两条配套判断规则：

- `ps -o cmd` 的输出里必须出现 `python`，否则说明 Host 未运行、取到的是别的东西。此时该项记为「未取到」，不要当作实际启动参数。
- 不要用 `pgrep -f alohamini`：2026-10-25 实测它会匹配到 ssh 远端那条 `bash -c …` 自己（当时 `pid=2060`），`/proc/$pid/cmdline` 读到的是探针自己的命令。必须写 `alohamini_host.py`。

`4.3` 的 `--include="*.py" --exclude-dir=__pycache__` 不能去掉：按目录 grep 会命中 `__pycache__` 的 `.pyc`，输出里多出 `grep: …: binary file matches` 并占掉 `head -20` 的名额（2026-10-25 实测混进 3 行）。

`4.3b` 用来确认这三个字段有没有命令行入口。实测只有 5 处 `add_argument`（`--robot_model`、`--no_follower`、`--profile_timing`、`--camera-stream`、`--camera-stream-port`），都不涉及这三个字段。

`4.3c` 用来看清默认值属于哪个 dataclass。实测 `AlohaMiniHostConfig` 与 `AlohaMiniClientConfig` 各有一个 `observation_request_window`（`:110` 与 `:135`），取 Host 那个。

取值规则：

- 先用 `4.2` 看实际启动命令行里有没有覆盖。当前 Pi 的 vendor 版本没有这三项的命令行入口，因此实际生效值等于 `AlohaMiniHostConfig` 的源码默认值，用 `4.3` 取。
- 将来厂商版本若新增入口，以 `4.2` 的实际输出为准。
- 写进现场 YAML 的必须是「实际生效的那个值」，并在 `$S4DIR/stdout/` 里留下取得它的命令（`4.x-*.cmd`）与输出（`4.x-*.log`）。

已实测取值来路（2026-10-25 UTC 07:00 前后，该时点 Pi HEAD 为 `ec65318f`，Host 未运行）。本表只记「这些数字怎么来的」：**数字本身以 `configs/site.yaml` 为准**，本节不再复制一份当权威。2026-10-25 实测到的三个数字记在 `configs/site.yaml` 的注释里（每个字段一行，写明取自哪个 `config_alohamini.py` 行号）；后续现场改配置改 `configs/site.yaml`，不要回来改本表。

| 字段 | 取值出处 | 依据（来路） |
| --- | --- | --- |
| `observation_request_window` | 见 `configs/site.yaml` 的 `observation_request_window` | `config_alohamini.py:110`（`AlohaMiniHostConfig` 源码默认）；该项无命令行入口 |
| `watchdog_timeout_ms` | 见 `configs/site.yaml` 的 `watchdog_timeout_ms` | `config_alohamini.py:122` |
| `max_loop_freq_hz` | 见 `configs/site.yaml` 的 `max_loop_freq_hz` | `config_alohamini.py:125` |

Host 未运行，所以这三个值是**源码默认**，不是「运行中进程的实际生效值」。`config_alohamini.py` 在 Pi 工作区是未提交修改状态，取值以工作区文件为准，与当时 HEAD `ec65318f` 的差异未核对。启动 Host 后应重跑 `4.2`/`4.3` 复核。

同日 Host 启动后复核：Host 实际启动命令为 `/home/pi5/miniconda3/envs/lerobot_alohamini/bin/python -m lerobot.robots.alohamini.alohamini_host --robot_model alohamini2pro`，未包含任何覆盖这三个字段的参数，与本节「源码默认即实际生效值」的判断一致。

两份 YAML 的分工：

| 文件 | 是什么 | 三个数值字段 |
| --- | --- | --- |
| `configs/template.yaml` | 随库分发的通用默认，换机器可原样使用 | `1` / `500` / `50`，取自 `src/alohamini_cartesian/config/overlay.py` 的 `AlohaMiniSettings` 默认值 |
| `configs/site.yaml` | 本机现场值，即 Host 进程实际生效的那份 | 本节现场数字的唯一出口，见文件内注释 |

覆盖顺序：默认值 → template → site → 环境变量（`ALOHAMINI_*`），由 `overlay.py` 的 `load_settings` 实现。两份一起加载的契约由 `tests/test_examples_and_configs.py` 的 `test_template_and_site_load_together_and_the_three_fields_follow_site_yaml` 锁定：三个字段的值等于直接从 `configs/site.yaml` 原文读出来的值。所以本节不再另记一份数字——上面那张表保留的是来路，不是取值。

另外四个字段的取值：

| 字段 | 取法 |
| --- | --- |
| `robot_model` | 阶段 1.1 产物 `$S4DIR/capture/fingerprint-$STAMP.json` 里 `robot_metadata.robot_model` 的值 |
| `left_port` / `right_port` | `run 4.5-serial ls -l /dev/serial/by-id/`；Pi 一侧同时取一份 `run 4.5b-serial-pi ssh "$PI" 'ls -l /dev/serial/by-id/'` 做对照（判据见下） |
| `cameras` | `run 4.6-video ls -l /dev/video*`；Pi 一侧取 `4.6b-video-pi` 做对照（命令见下），再加实际相机接法（判据见下） |

Pi 一侧的相机节点对照命令：

```bash
run 4.6b-video-pi ssh "$PI" 'ls -l /dev/video* 2>/dev/null | head -20'
```

`4.5b` 与 `4.6` 的判据不能按 `.exit` 是否为 0 判成功。`ls` 在无匹配时返回 2；2026-10-25 实测 Pi 上没有 `/dev/ttyUSB*`，末条 `ls` 无匹配使整条命令 exit 2，而 `by-id` 与 `ttyACM` 的输出完全正常。判据是：看 `.log` 里有没有 `/dev/ttyACM*` 行；`4.6` 同理，看有没有 `/dev/video*` 行。`.exit` 为 2 但 `.log` 里有该有的行，这一条算取到。

标定文件核对（换机或重装后要确认标定文件还在）：

```bash
run 4.7-calib ssh "$PI" 'ls -l ~/.cache/huggingface/lerobot/calibration/robots/*/ 2>/dev/null; find ~ -maxdepth 5 -name "*.json" -path "*calibration*" 2>/dev/null | head -20'
```

判据：输出里能找到这次要用的标定文件，并记下大小。2026-10-25 实测：Pi 上有 `robots/alohamini/AlohaMiniRobot.json`（2880 B）与 lekiwi 的两个，另有 `/home/pi5/b02-b03-evidence/calibration-alohamini2pro.json`。

设备分工（2026-10-25 实测）：本机（macOS）没有 `/dev/serial/by-id/`、`/dev/ttyACM*`、`/dev/video*`；Pi 上有 `ttyACM0`/`ttyACM1`（两块 `usb-1a86` 单串口，`5B14029263` 与 `5B14111703`）与 20 个 `/dev/video*` 节点。本机那两条命令在没接机械臂的 Mac 上取不到，不能据此判「设备缺失」。

设备路径填的是**将来运行本库那台机器**上的路径，不是「随便哪台机器都能用」的路径：

- 真正部署时本库跑在接了机械臂的那台机器上（本项目是 Pi），YAML 里写 Pi 侧路径（`/dev/ttyACM0`、`/dev/ttyACM1`、`/dev/video*`），在这台机器上 `load_settings(check_devices=True)` 才成立。
- 若只在 Mac 上做配置结构校验，用 `load_settings(check_devices=False)`（这是默认值），只验键名、类型、取值，**不验设备存在性**。此时设备存在性的证据来自 Pi 侧的 `4.5b`/`4.6b` 输出，不要因为本机 `ls` 为空就判「设备缺失」。
- `check_devices=True` 在 Mac 上必然失败（本机没有这些 `/dev/` 节点），这不是配置写错，是跑错机器。

判据：两份一起能被读通。本次现场在 Mac 上跑结构校验，显式写 `check_devices=False`：

```bash
run 4.4-load-check uv run --no-sync python -c "from pathlib import Path; from alohamini_cartesian.config.overlay import load_settings; print(load_settings(template=Path('$S4DIR/configs/template.yaml'), site=Path('$S4DIR/configs/site.yaml'), check_devices=False))"
```

判据：`.exit` 为 `0`，`.log` 里打印出合并后的 `AlohaMiniSettings`。

设备存在性校验需要把两份 YAML 拿到接了机械臂的 Pi 上跑，本次现场不做。命令留在这里备查：

```bash
run 4.4b-load-check-deploy uv run --no-sync python -c "from pathlib import Path; from alohamini_cartesian.config.overlay import load_settings; print(load_settings(template=Path('configs/template.yaml'), site=Path('configs/site.yaml'), check_devices=True))"
```

这一条在 Pi 上部署后跑（本次现场不做）。命令里的 `configs/template.yaml`、`configs/site.yaml` 是占位路径，实际以 Pi 上落点为准；前提是 Pi 已装本库。本次实际跑的是 `4.4-load-check`，`4.4b` 未跑。

现场易错点：本库 config overlay 的 `watchdog_timeout_ms` 默认是 `500`（`src/alohamini_cartesian/config/overlay.py` 的 `AlohaMiniSettings`），厂商 `config_alohamini.py` 的默认是 `1000`（不写行号：行号随厂商提交变化，2026-10-25 实测在 Pi 上约 122 行）。现场核对宿主实参时不要把两个数字混用，样例里写实际生效的那个值。以 `4.2` 与 `4.3` 的实测为准，这两个默认值都不作为依据。

现场易错点：`robot_model` 的两处默认值不一致。本库 `src/alohamini_cartesian/config/overlay.py` 的 `AlohaMiniSettings.robot_model` 默认值是 `alohamini1`（`overlay.py:51`）；两个现场脚本的 `DEFAULT_ROBOT_MODEL` 都是 `alohamini2pro`（`benchmarks/alohamini_io/capture_robot_fingerprint.py:76`、`benchmarks/alohamini_io/measure_latency.py:360`）。现场机型是 AlohaMini 2 Pro，而 `measure_latency.py` 的代码 Gate C 要求宿主 metadata 的 `robot_model` 与 `--robot-model` 相等（`measure_latency.py:1437`）。所以现场 YAML 必须显式写 `robot_model: alohamini2pro`，不能用模板里的 `alohamini1`，否则配置表达的意思与真机不符。

两份配置样例已入库：`configs/template.yaml` 与 `configs/site.yaml`。现场值见 `configs/site.yaml`（来路见本节），读取验证用 `load_settings(template=..., site=...)`。

本阶段产物：`configs/template.yaml`、`configs/site.yaml`，以及 `stdout/4.1-host-args-*`、`4.2-host-cmdline-*`、`4.3-host-config-*`、`4.3b-config-args-*`、`4.3c-config-classes-*`、`4.4-load-check-*`、`4.5-serial-*`、`4.5b-serial-pi-*`、`4.6-video-*`、`4.6b-video-pi-*`、`4.7-calib-*`。`4.4b-load-check-deploy` 本次未跑，无产物。

## 9 退出码对照表

**一码两义**：`6` 在 `capture` 里表示宿主 metadata 不可用、主动不写盘；在 `measure` 里表示运动门禁拒绝、零命令。现场拿到 6，先看是哪个脚本。

| 脚本 | 码 | 含义 |
| --- | --- | --- |
| 两者 | 2 | 参数解析失败，一律 `2`，不经过任何 `EXIT_*` 常量；所有判定都在打开任何 socket 与调用 `client_factory(...)` 之前完成。`measure`（rtt 与 tracking 共用同一个 `parse_args`，rtt 也查这些旋钮）：argparse 自身产生的用法错误 —— 未知选项、缺少 `--host`、`--mode` 不在 `rtt`/`tracking`、任一 `type=` 转换失败（`--port` / `--samples` / `--request-window` / `--steps` 非整数，或任一 `type=float` 选项不是可解析的数值文本；`--csv` 的 `type=Path` 对命令行字符串不会抛，不在此列）；整数下界 3 项 —— `--samples < 1`、`--request-window < 1`、`--steps <= 0`；「有限且带符号」同查 7 项 —— `--rate-hz`（有限且 `> 0`）、`--control-hz`（有限且 `>= 1`）、`--host-loop-hz`（有限且 `> 0`）、`--timeout-s`（有限且 `> 0`）、`--settle-timeout-s`（有限且 `> 0`）、`--settle-tolerance-deg`（有限且 `>= 0`，`0` 合法即最严格设置）、`--delta-deg`（有限且 `> 0`），非有限值（`nan` / `inf`，含 `float()` 后变成 `inf` 的 `1e400`）与不满足符号条件的值都落 `2`；交叉校验 1 项 —— `--settle-tolerance-deg >= resolve_delta_deg(args)`（与钳到 1.0° 上限之后的有效 delta 比较）。`capture` 的数值校验只有 2 项 —— `--samples < 1`、`--timeout-s` 有限且 `> 0`（不查 `--rate-hz` / `--control-hz` 之类），另加同一形状的 argparse 用法错误（未知选项、缺少 `--host`、`--port` 非整数）。以上为 `2` 的全部触发点。合法预算在运行中到期不走 `2`：`capture` 落 `4`，`measure` 的 rtt 落 `7`、tracking 落 `1` |
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
| measure | 9 | 客户端不发布 rtt 模式所测量的能力（`latest_request_timing`），或发布了但内容填不满。三种形态都返回 `9`、都不写 CSV，形态与触发点见表下小表 |

`capture_robot_fingerprint.py` 不产生 `9`：缺 `latest_request_timing` 时它继续采集并把计时字段记 `null`，见第 13 节。

### 退出码 `9` 的三种形态

| 形态 | 何时触发 | 是否发过请求 |
| --- | --- | --- |
| 属性不存在 | `connect()` 返回后、窗口预热之前 | 没发过 |
| 属性存在但从未填充（整轮是空 dict，`observation_sequence` 不增长） | 窗口预热之后，采样循环放弃那一刻才判，要等满 `--timeout-s` | 发过 1 次 state-only 预热，并至少观测 1 帧 |
| 半填充（缺 `token` / `sent_monotonic_s` / `received_monotonic_s` / `age_s` 任一） | 写行之前逐帧复查，`observation_sequence` 每次增长都查一次 | 已发过请求，并在该帧拒绝 |

三种都不写 CSV，连表头都不写。stderr 分得开：第一种是 `it publishes no latest_request_timing`，后两种是 `it publishes latest_request_timing but never fills it completely`，并跟一句 `missing key(s): …` 点名缺的键。只看退出码分不出这三种，要读 stderr。

退出码同时写在每步的 `$S4DIR/stdout/<步骤号>-$STAMP.exit` 里，查表前先看这个文件。

## 10 产物与归档

证据目录按 `AGENTS.md` 第 8 节约定落在 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/` 下。第 3 节的变量块按 UTC 日期建子目录，`$S4DIR` 就是它：

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
| `capture/` | 阶段 1.1 的 `fingerprint-$STAMP.json`；P0 未解决时这里为空，失败运行的原文在 `stdout/1.1-capture-*` |
| `rtt/` | 阶段 1.2 与 1.3 的两份 CSV（`rtt-net-$STAMP.csv`、`rtt-deploy-$STAMP.csv`） |
| `tracking/` | 阶段 3 的 `tracking-$STAMP.csv` 与 report 原文 |
| `configs/` | 阶段 4 的 `template.yaml` 与 `site.yaml` |
| `stdout/` | 每条命令的三件套 `<步骤号>-$STAMP.cmd`、`.log`、`.exit`；阶段 2 的拒绝记录也在这里 |

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

## 11 验收标准

每条一个判据，不写「看起来正常」。

- [ ] 宿主 `robot_metadata.schema_version` 取到整数 `2`，且有 `capture` 产物为证（P0 关闭）
- [ ] `capture` exit 0，JSON 10 个顶层键齐全，且 `robot_metadata.motors` 里逐个电机的 8 字段齐备情况有记录
- [ ] 两组 rtt 各 500 样本、各 exit 0，且报告里 `achieved_rate_hz` 与请求率的差距有记录
- [ ] 代码 Gate F 至少有一次真实拒绝记录（stderr 原文 + 命令原文 + 退出码）；本窗口触发不了 F 的第二段时，改为记录「v2 宿主下 Gate F 未触发」、`2.1-probe` 的输出与理由。当前 Pi 上第二段可达（换算字段不全），所以本窗口应按第二段记；只有在电机字段齐全的宿主机上才落回「未触发」分支
- [ ] Mode B 若执行：每步的 `error_deg`、`overshoot_deg`、`settled` 齐全；`rise_90_time_s` 非空；若为空，说明原因（容差设置或命中同一轮询），并写明是哪一种；收尾 `hold_issued` 在报告里
- [ ] 阶段 2 与阶段 3 只做了第 6 节对照表允许的那一个，另一个没做并写明了原因
- [ ] 两份 YAML 样例（模板 + 现场）落盘，字段名与 `config/overlay.py` 一致，且能被 `load_settings` 读通
- [ ] `$S4DIR/stdout/` 里每步都有 `.cmd`、`.log`、`.exit` 三件套

## 12 失败与中止处置

| 现象 | 判据 | 处置 |
| --- | --- | --- |
| exit 6（capture） | stderr 含 `schema_version=` | 记录原文，转 P0 分支，不要重试 |
| exit 6（measure） | 拒绝发生在首帧后 | 记录是哪道代码 Gate，写进 `"$S4DIR/stdout/"` |
| exit 8 | 报告里 hold 被拒 | 先断电，再排查；按「仍在动」处置 |
| exit 1 加反馈失联 | 连续 3 次序号不涨 | 检查网络与 Host 负载 |
| 网络中断 | `ping` 丢包 | 重跑该阶段，不复用旧产物 |

### 停 Host 不等于失力

停 Host 之后手臂仍然带电保持。2026-10-25 停 Host 后读到的寄存器是 `Torque_Enable=1`、`Lock=0`，位置环还在通电，人手推就是顶着位置环（读数完整表见第 13 节）。断电才失力。

所以收工若要断电，先把手臂放到低位再断电，避开断电瞬间的下落。这条只约束断电时序，不改变任何一条退出码判据。

## 13 已知缺口与未验证项

Pi 侧命令的实测范围（一条命令算一项）：

| 已实测（2026-10-25） | 未实测 |
| --- | --- |
| `0.0-ssh-check`、`0.10-pi-ip`、`0.2-pi-commit`、`0.3-pi-schema`、`0.3b-schema-context`、`0.5-host-procs`、`0.6-ports`、`0.6b-ports-netstat`、`1.1-capture`、`1.2`、`1.3`、阶段 2 的 Gate B 探测与 Gate F 拒绝（`2.0-refusal-v1`）、`4.1-host-args`、`4.2-host-cmdline`、`4.3-host-config`、`4.3b-config-args`、`4.3c-config-classes`、`4.5-serial`、`4.5b-serial-pi`、`4.6-video`、`4.6b-video-pi`、`4.7-calib`、阶段 3 的 `3.1-tracking`（五次运行） | `0.4b-tools`、`0.4-pi-ls`、`2.1-probe`、`4.4-load-check` |

步骤号与证据文件的对照：`2.0-refusal-v1` → `stdout/2.2b-gateF-attempt-20261025T074158Z.{cmd,log,exit}`（exit `6`、零命令）；`2.1-probe` 未按本文命令运行，同一项检查由 `stdout/5.1-probe21-20261025T094457Z.log` 完成，输出为 18 个电机逐项 `missing=[] bad=[]`。

新增实测项的备注：

- `1.1-capture`、`1.2`、`1.3`：已在 v2 宿主上实测通过（RTT 为 Pi 本机回环）。`1.1-capture` exit 0，`robot_metadata.schema_version` 为 `2`，18 个电机字段齐全；`1.2` / `1.3` exit 0，请求率受 Host 50 Hz 主循环封顶，`1.2` 组 `achieved_rate_hz=49.8`（请求 1000 Hz）。早先一次在 Pi 侧以 conda python 执行 `1.1-capture` 时退出码 `3`（该时点 vendor 为升级前的 `ec65318f`，不提供 `latest_request_timing`；升级后的 `6aee1f81` 提供该面），未写 JSON；该现象已由 v0.7.0 的能力探测改为显式中止或记 `null`。
- 阶段 2 的 Gate B 探测：命中，列出可用关节。
- 阶段 2 的 Gate F 拒绝（`2.0-refusal-v1`）：命中第一段，`schema_version` 为 1。
- `2.1-probe`：本身未跑。v2 宿主下 18/18 电机字段齐全，Gate F 第二段不会被触发，探测结论会是「不拒」。第二段的拒绝路径仍未验证，见本节末尾。
- `4.2-host-cmdline` 与 `4.3-host-config` 的「启动 Host 后复核」已做：Host 启动命令里没有覆盖 `observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz` 这三个字段的参数。
- 阶段 3 的命令（`3.1-tracking`）：已跑五次，五份 CSV 的 `note` 列只有 `ok` 与 `settle budget exhausted`，无 `aborted`；首次试动的 report 记 `steps_completed=3`、`abort_reason=None`、`hold_issued=yes`，同次 stderr 0 字节。只有腕部达成率高：`arm_left_wrist_roll` 净行程 +1.758°/2.0°（87.9%），两步 `settled=True`，误差 0.121°。肩部未跟随：首次试动 `arm_left_shoulder_pan` 5.9%，随后同一轴 8.8%（1°×3 步）与 8.8%（1°×2 步），`arm_left_shoulder_lift` 13.2%，全部 `settled=False`。原因定位中。数据见第 7 节。

未实测不等于不可用，但首次执行时要格外留意输出形态与本文档描述是否一致。

- Mode B 只在 2026-10-25 跑过五次，都是左臂单关节、每步 1°。右臂、多关节联动、更大的单步行程都未跑过，见第 7 节实测表。
- `tracking` CSV 的 `received_monotonic_s` 一列五份全空。按第 10 节的列定义，该列记的是本步越过 90% 那一帧的到达时刻；五份的 `rise_90_time_s` 也全空，说明没有帧记到 90% 交点，该列随之空。两者同因这一条是推论，未单独验证。后果是算不出「命令发出到反馈到达」的时延：`t_send_monotonic_s` 有值、到达时刻没有，本轮只能算位置与误差。要补这一列，先把 `--settle-tolerance-deg` 取到小于 0.1 × delta（即小于 0.1°）让交点出现，再确认脚本在该帧写入到达时刻。
- Gate F 第二段（换算字段不全时的拒绝）在 v2 宿主上未触发、未验证：现在 18/18 字段齐全，第二段不会拒。要验它需构造换算字段不全的宿主，或注入测试。
- 4 路相机在采集时全返回零帧（stderr 出现 `Frame is None`），另有两次 ZMQ 请求超时。该条来自现场采集时的观察，证据目录内没有零帧记录，也没有任何图像文件，**这一条在目录内没有直接证据**；下次采集时应把该 stderr 落盘。关节与 metadata 正常，所以未阻塞零运动阶段，但带图像的用例会拿到空帧。未排查。
- RTT 数字是 Pi 本机回环，不能当网络延迟读。
- 收尾 hold 的实际效果无证据。
- exit 8 的真实触发路径无记录。
- Action socket 无确认，`hold_issued` 不等于送达。
- 脚本不检查关节限位。
- `schema_version` 升到 2 的步骤两仓库文档都没有。
- 电机名到关节观测键的对应关系未核实，见第 6 节。
- 运动过程中的实时寄存器跃迁未采集：本节「电流字段不可用，扭矩状态为开，写入者未识别」里那 152 次读全部发生在停 Host 之后，是静止态。`Present_Load`、`Present_Current`、`Status` 在步进过程中怎么变，没有数据。
- Host 侧的安全门禁是否扣住了肩部目标未验证。要判它得看停滞检测、`joint_hold_events`、`requested_targets` / `accepted_targets` 在步进时的表现，这些都需要重启 Host 复现。

### 电流字段不可用，扭矩状态为开，写入者未识别

四条现场事实并排，反馈侧与手感侧互相矛盾，都保留，不做取舍：

| 事实 | 内容 |
| --- | --- |
| 反馈侧 | 18 个电机的 `currents_ma` 全为 `0.0`，偶见 `6.5`（等于 1 个最小刻度） |
| 手感侧 | 手推手臂有明显阻力、推不动 |
| 命令侧 | `arm_left_wrist_roll` 能跟随命令走出 +1.758°/2.0°（87.9%） |
| 源码侧 | 厂商代码 `alohamini.py` 的 `configure()` 无条件调用 `disable_torque()`，两处 `enable_torque()` 是注释掉的（`alohamini.py:545`、`:567`），git 全历史里从未生效 |

2026-10-25 停 Host 后只读伺服寄存器，读左臂 ID 1（`shoulder_pan`）、2（`shoulder_lift`）、3（`elbow_flex`）、6（`wrist_roll`）：34 个寄存器 × 4 电机，加二次采样，共 152 次读，全部成功，零失败。

| 寄存器 | ID1 | ID2 | ID3 | ID6 | 判读 |
| --- | --- | --- | --- | --- | --- |
| `Torque_Limit` | 1000 | 1000 | 1000 | 1000 | 满量程，未被压低 |
| `Max_Torque_Limit` | 1000 | 1000 | 1000 | 1000 | 同上 |
| `P` / `I` / `D` | 16 / 0 / 32 | 16 / 0 / 32 | 16 / 0 / 32 | 16 / 0 / 32 | 四台完全相同，未被改 |
| `Operating_Mode` | 0 | 0 | 0 | 0 | 位置伺服 |
| `Torque_Enable` | 1 | 1 | 1 | 1 | 扭矩开着 |
| `Lock` | 0 | 0 | 0 | 0 | 未锁 |
| `Protection_Current` | 310 | 500 | 500 | 310 | 按型号分组（sts3250=310、sts3095=500），不按「动得动 / 动不了」分组 |
| `Present_Load` | 0 | 0 | 0 | 0 | 静止态为 0 |
| `Present_Position` = `Goal_Position` | 一致 | 一致 | 一致 | 一致 | 四台都停在目标位 |
| `Status` | 0 | 0 | 0 | 0 | 无报警 |
| 温度 / 电压 | 22–27 ℃ / 10.6–10.8 V | — | — | — | 正常 |

结论四条：

- `currents_ma` 仍不可用作负载或扭矩判据。静止态 `currents_ma` 与寄存器 `Present_Load` 都是 0，说明静止时确实无负载；这不能反推运动过程中的负载。
- 扭矩状态已由寄存器读数为**开着**：四台都是 `Torque_Enable=1`、`Lock=0`。这与「手推不动」一致：位置模式下扭矩开着，人手推就是顶着位置环。
- 肩部与腕部在寄存器配置上找不出差别。`Torque_Limit`、`Max_Torque_Limit`、`P`/`I`/`D`、`Operating_Mode` 四项四台逐项相同；有差异的五项全部随型号变化，没有任何一项按达成率分组：`Protection_Current`（310 / 500）、`Minimum_Startup_Force`（0 / 8）、`CW/CCW_Dead_Zone`（0 / 2）、`Protection_Time`（200 / 250）、`Unloading_Condition`（45 / 44）。ID1 与 ID6 同属 sts3250 一组（例如 `Protection_Current` 都是 310），而这两台的达成率分别是 8.8% 与 87.9%。因此寄存器配置差异解释不了达成率差异。
- `{Torque_Enable=1, Lock=0}` 这个组合不是本库与厂商 fork 的常规写路径能留下的状态。厂商 `feetech.py` 的 `disable_torque()` 写 `{Torque_Enable=0, Lock=0}`，`enable_torque()` 写 `{1, 1}`（`lerobot_alohamini/src/lerobot/motors/feetech/feetech.py:293`、`:304`），两条都产生不出 `{1, 0}`；本库 `src/` 里没有对这两个寄存器的写入。**写入者身份未识别。** 源码侧 `configure()` 无条件调 `disable_torque()` 预测扭矩应为 0，实测为 1，两者冲突：是这次运行路径上该调用没走到，还是之后被别的写入者覆盖，两点都未验证。

### Pi 上的 client 只发布部分计时面

这条观测要按 vendor 版本分两段读，时点不能混。

**升级前**（2026-10-25 UTC 07:00 前后，HEAD `ec65318f`；`stdout/1.1b-capture-pi-20261025T074158Z.log` 与该轮 `1.2b` 的报错都在这个时点）：`AlohaMiniClient` 提供 `latest_host_timing`，但**不提供** `latest_request_timing`，`measure_latency.py --mode rtt` 返回 `9`。该版本里能拿到的是 `latest_safety_status`，与 `latest_control_status` 不是同一帧，结构也不同（`version` / `control_owner` / `host_session_id` / `command_watchdog_timeout_s`），不可互相替代。

**升级后**（HEAD `6aee1f81`）：提供 `latest_request_timing`，`measure_latency.py --mode rtt` 实测 exit 0，证据 `evidence/s4-20261025/stdout/3.2-rtt-20261025T094446Z.exit` 内容为 `exit=0`。

对仍缺该面的旧 vendor，下面的探测行为依然有效。

两个基准脚本由此产生的行为（v0.7.0 起）：

| 脚本 | 缺什么 | 行为 |
| --- | --- | --- |
| `measure_latency.py --mode rtt` | `latest_request_timing` | 连上后立即拒绝，退出码 `9`，stderr 说明缺哪个属性、为什么不测量、建议换发布该面的 vendor 版本；**不写 CSV**，连表头都不写 |
| `measure_latency.py --mode tracking` | 不读该面 | 不受影响 |
| `capture_robot_fingerprint.py` | `latest_request_timing` | 继续采集；每条样本的 `age_s` / `sent_monotonic_s` / `received_monotonic_s` 记为 `null`，stderr 警告一次（写明「缺失而非零」） |
| `capture_robot_fingerprint.py` | `latest_control_status` | `control_status` 记为 `null`，不回退到 `latest_safety_status` |

payload 的 10 个顶层键与 `schema_version = 1` 不变；缺能力不改变 capture 的退出码语义（metadata 不合法仍是 6，写盘失败仍是 5，正常仍是 0）。

判读要点：在这份 vendor 上 `measure --mode rtt` 返回 `9` 是**设计行为，不是故障**；`capture` 里出现 `null` 计时字段同理。看到它们不要当成通信问题排查。

为什么不静默回退：若用 `getattr` 把缺失的 `latest_request_timing` 当空值处理，rtt CSV 的 `token` / `sent_monotonic_s` / `received_monotonic_s` / `age_s` / `net_rtt_*` 会全部变空或 NaN，而 `host_timing_missing` 仍为 `False`，等于谎报主机计时存在。缺能力时应显式中止并说明，不做静默降级。

另一条相关事实：`pyproject.toml` 对 `lerobot` 只声明名字、不声明版本或提交（ADR D1 有意如此），因此没有机制能在安装期拦住这类接口不匹配；两个脚本的测试夹具同时设了 `latest_request_timing` 与 `latest_host_timing`，单元测试也发现不了——这个缺口是靠真机跑出来的。

### 左夹爪标定行程只有 51 tick（离线推算发现，待现场只读确认）

离线核对（见 `docs/field-results.md`「离线核对（无新采集）· 左右夹爪行程差异」）发现：`arm_left_gripper` 的 `range` 是 2021-2072，只有 51 tick（`range_0_100` 下 4.48°），而 `arm_right_gripper` 是 2010-3479，1469 tick（129.14°）。两侧同为 `sts3250`、`drive_mode` 同为 0。根因未确认。

下次现场用**只读**方式确认三件事，不动机器人、不发位置指令：

1. 左夹爪 ID7 的固件限位是否已被写成 2021 / 2072。参照 `stdout/reg_read_1792923639.py` 的写法，在 `MOTORS` 里给左臂总线加上夹爪，其余不动：

   ```python
   PORT = "/dev/am_arm_follower_left"
   MOTORS = {
       "shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
       "shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
       "elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
       "wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
       "gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),  # 新增
   }
   ```

   要看的寄存器：`Min_Position_Limit`、`Max_Position_Limit`、`Present_Position`、`Goal_Position`、`Homing_Offset`、`Torque_Enable`、`Operating_Mode`、`Model_Number`、`Status`。

   判读：`Min/Max_Position_Limit` 等于 2021/2072 → 固件限位已生效，左夹爪被锁在 51 tick 内，问题在标定值而非传递；读出别的值 → 问题在写盘或写盘之后的覆盖。

2. 右夹爪 ID7 用同一脚本、换 `PORT = "/dev/am_arm_follower_right"` 与右臂电机表读一遍，作为同型对照（也顺便补上本轮 `reg_read` 没覆盖的 ID4/5/7）。

3. 断电前后各读一次 `Min_Position_Limit` / `Max_Position_Limit`，确认这两个值跨断电保持（本轮已知 18 电机 × 8 字段跨换电池 0 差异，但那是 metadata 侧，不是寄存器侧）。

**以下一条不是只读，需另行授权后再做**：把扭矩关掉（`disable_torque`）、用手慢慢开合左夹爪，同时流式读 `Present_Position`，看读数变化范围。这一步会改 `Torque_Enable`/`Lock`（厂商 `feetech.py:293`、`:304` 写 `{0,0}` 与 `{1,1}`），而本轮已知当前状态是 `{1, 0}`（写入者未识别），贸然改会破坏「扭矩处于开启态」这条现场事实的可追溯性。要做的话先记录改前读数、改后立刻复原并记录。

采样要求：三件事都要落 `stdout/` 日志与 `.cmd` 文件，按第 3 节 `run` 包装函数的约定命名，退出码与命令原文一起存。

## 14 待裁决事项

无。原「证据目录是否长期固定」一条已定：维持外部目录，见 `AGENTS.md` 第 8 节。

## 附录 A：两个脚本的完整参数表

### capture（6 个参数加 `-h`）

| 参数 | 必填 | 默认值 | 约束 | 影响什么 |
| --- | --- | --- | --- | --- |
| `--host` | 是 | 无 | 无 | 连哪台 Pi |
| `--port` | 否 | `5556` | 无 | 宿主观测端口 |
| `--robot-model` | 否 | `alohamini2pro` | 无 | 传给客户端的模型名；代码 Gate C 拿它与宿主 metadata 的 `robot_model` 比对，取值按阶段 1.1 JSON 里的值填 |
| `--samples` | 否 | `5` | 不小于 1（否则参数解析失败 exit 2） | 要采到的观测请求数 |
| `--timeout-s` | 否 | `30.0` | 必须有限且大于 0（否则参数解析失败 exit 2）；预算合法但到期仍不够样本落 exit 4 | 采样本的总时间预算，也是 `capture` 唯一的停机条件 |
| `--out` | 否 | `artifacts/alohamini-io/fingerprint-<UTC>.json`，解析参数时算时间戳 | 父目录自动创建 | JSON 落盘位置 |

### measure（17 个参数加 `-h`）

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
| `--delta-deg` | 否 | `1.0` | 必须有限且大于 0（否则 exit 2）；实际被 clamp 到 `1.0` | 每步目标增量（度） |
| `--steps` | 否 | `20` | 不小于 1（否则 exit 2） | 步数；与 `--delta-deg` 相乘受 20.0° 上限约束 |
| `--settle-timeout-s` | 否 | `1.0` | 必须有限且大于 0（否则 exit 2） | 每步的稳定判定预算 |
| `--settle-tolerance-deg` | 否 | `0.15` | 必须有限且不小于 0（`0` 合法）；且必须小于 clamp 后的有效 delta；两者违规均 exit 2 | 稳定判定的容差（度） |

`measure` 的 `--help` 用的是 `RawDescriptionHelpFormatter`，不打印默认值。上表默认值来自源码常量。
