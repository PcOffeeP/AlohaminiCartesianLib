# 1 开场准备（原 §0–§3、§4.3）

本文件管什么：开场到第一次取数之前要过的门——怎么用这本手册、硬前置 P0–P5、变量块与留痕函数、到场核对，以及部署取值核对。

## S4 现场执行手册：真机 smoke

这份文档是一次现场窗口的执行手册，从到场核对写到收尾，每一节给命令、判据与产物。现场由 Agent 通过 SSH 连上 Pi，照着逐条取数、执行，不靠临时决策补缺失的信息。

实测边界：第 [§3](#sec-3)、[§4.1](3-health.md#sec-4-1)、[§4.3](#sec-4-3)、[§5](4-experiments.md#exp-e1)、[§7.1](4-experiments.md#exp-e3) 节的命令在 2026-10-25 对 `pi5@10.34.170.98` 跑过一轮，产物在证据目录 `evidence/s4-20261025/`，数字汇总见 `docs/field-results.md`。第 [§4.2](3-health.md#sec-4-2)、[§5.3](../reference/env-rebuild.md#sec-5-3)、[§6.1](2-host.md#host-start)、[§6.2](4-experiments.md#exp-e2)、[§7.2](4-experiments.md#exp-e3) 节原定为本轮新增、未在现场执行过；2026-11-05 的现场会话（远程 SSH，Pi 运行时 HEAD `6aee1f81`、lerobot `0.6.1`）已把这五节与 [§4.1](3-health.md#sec-4-1) 的 Pi 侧 capture 跑过一轮，命令与本文原写法不同之处已在对应小节写明，数字汇总见 `docs/field-results.md` 的 2026-11-05 节。首跑若输出形态与本文不符，按实际输出修正，并回写到本文档。2026-11-05 之后新增的 [§4.1c](3-health.md#sec-4-1c)、[§4.2](3-health.md#sec-4-2) 的「全 18 电机静态体检」、[§4.4](3-health.md#sec-4-4)、[§7.1b](4-experiments.md#exp-e4) 已在 2026-11-06 的现场窗口跑过（证据目录 `evidence/s4-20261106/`，数字汇总见 `docs/field-results.md` 的 2026-11-06 节）；[§4.5](../completed-work.md#appendix-b) 只跑过 ssh 连通性检查，`stash list` / `reflog` / `status --porcelain` 三份原文未取；改写后的 [§10.5](4-experiments.md#exp-e5) 仍未在任何现场跑过：命令形式在本地验过（`--help`、`--delta-deg` 的负值与零值行为、位置比对脚本的自检与变异自检），现场输出形态以首次执行为准。**[§10.6](4-experiments.md#exp-e6) 与 [§10.7](4-experiments.md#exp-e7) 同样一次现场都没跑过**：[§10.6](4-experiments.md#exp-e6) 只有一次离线复现（假 client，无设备无网络），[§10.7](4-experiments.md#exp-e7) 只有 ①b 的离线 `--dry-run` 在本机验过。

<a id="sec-0"></a>
## 0 这份文档怎么用

执行顺序固定，按下面四步走：

1. 先读 [§1](#sec-1)，把 P0–P5 逐条判完。判不了的条件不要跳过：`P1`–`P5` 任一不满足，就停在那一步。
2. 在同一个 shell 里粘 [§2](#sec-2) 的变量块与 `run` 函数，再跑一次 `0.9-probe false` 自证留痕链路。后面所有命令都假定这几步已经做完。
3. 按 [§3](#sec-3) 到 [§8](5-wrapup.md#sec-8-2) 的顺序执行。每节开头写前置条件，每步命令之后写判据与产物。
4. 拿到非 0 退出码先查 [§10](../reference/gates.md#exit-codes)；要处置查 [§12](../reference/troubleshooting.md#sec-12)；查已发现的问题查 [§9](../reference/troubleshooting.md#sec-9)；交回前按 [§13](5-wrapup.md#sec-13) 逐条勾选。

步骤号说明：`run` 的步骤号就是本文的分节号（[§3.4](#sec-3) 用 `3.4-ports`，[§4.3](#sec-4-3) 用 `4.3a-host-args`），产物名里的步骤号也是它。历史证据文件仍沿用重排前的旧步骤号命名（`0.*`、`1.*`、`3.1-tracking` 等），文中引用原件时按原文件名写。

新步骤号 `4.1b-lerobot-version` 在执行顺序上**先于 4.1**（它是 4.1 的输入），与后文的 `4.1b-probe`（校准字段齐全性探测）是两件事，只共用 `4.1b` 前缀，产物文件名不冲突。

`4.1c` / `4.4` / `4.5` / `7.1b` 同样是插入式编号，现有分节号一个都没重排：`4.1c` 跟着 `4.1`、`4.1b` 走（断电期间位置比对），`4.4` / `4.5` 接在 `4.3` 之后仍属「[§4](3-health.md) 只读确认」，`7.1b` 是 `7.1` 的反向拒绝取证。

一页总览：

| 节 | 内容 | 时长 | 是否动机器人 | 主要产物 | 完成判据 |
| --- | --- | --- | --- | --- | --- |
| 3 到场核对 | 网络、Host 进程、端口、工具、设备路径、授权留痕 | 10 分钟 | 否 | `stdout/3.*`、`stdout/scope-*` | 两条 `--help` 均 exit 0；5555 与 5556 各至少一行 `LISTEN` |
| 4 只读确认 | capture 指纹、断电期间位置比对（4.1c）、校准字段探测、左夹爪 ID7 寄存器与全 18 电机静态体检、部署取值、库真机 dry-run（4.4）、Pi 未提交文件调查（4.5） | 本文未设预算 | 否 | `capture/fingerprint-$STAMP.json`、`stdout/4.2-gripper-regs-*`、`configs/*.yaml`、`capture/fingerprint-postpower-*.json`、`stdout/4.2-all18-*`、`stdout/4.4-*`、`stdout/4.5*` | `capture` exit 0 且 `robot_metadata.schema_version` 为 `2`；`load_settings(check_devices=False)` 读通 |
| 5 零运动测量 | 部署形态 rtt、RTT 形态 rtt、跨机 rtt | 本文未设预算 | 否 | `rtt/` 三份 CSV | 三条均 exit 0 且各有数据行 |
| 6 量级验证 | 静止电流基线、短运动电流量级 | 本文未设预算 | 6.2 走 2 步 | `stdout/6.1*`、`stdout/6.2*` | 两段 `[Currents]` 原文与计数在盘，并写出「电流能不能判读」的结论 |
| 7 受控试动 | 开跑勾选单、单关节步进、肩部目标接受判定 | 本文未设预算 | 是，单臂单关节 | `tracking/tracking-$STAMP.csv`、其 sidecar、`stdout/7.2-*` | exit 0；exit 8 按 [§12](../reference/troubleshooting.md#sec-12) 处置 |
| 8 收尾 | 断电时序、Host 停启留痕、产物归档 | 本文未设预算 | 否 | `stdout/8.*` | 每步三件套齐全，Host 停启有记录 |

只有到场核对有预算（10 分钟），依据是本次任务的约定。其余各节不设预算，实际耗时记进 `$S4DIR/stdout/` 存档。

哪些步骤要现场授权：

| 动作 | 在哪节 | 需要哪类授权 | 没有授权时 |
| --- | --- | --- | --- |
| SSH 到 Pi 执行只读命令 | 第 [§3](#sec-3)、[§4.1](3-health.md#sec-4-1)、[§4.2](3-health.md#sec-4-2)、[§4.3](#sec-4-3)、[§4.4](3-health.md#sec-4-4)、[§4.5](../completed-work.md#appendix-b)、[§5.3](../reference/env-rebuild.md#sec-5-3)、[§7.1b](4-experiments.md#exp-e4)、[§10.5](4-experiments.md#exp-e5) 节 | 现场网络与登录授权 | 不能开工，停在 [§1](#sec-1) |
| 向 Pi 写文件（把寄存器脚本投到 `/tmp`） | [§4.2](3-health.md#sec-4-2) | 向 Pi 写文件的授权 | 改为在 Pi 上手工输入同一脚本，输入方式需现场确认 |
| 启停 Host 进程 | 第 [§6.1](2-host.md#host-start)、[§6.2](4-experiments.md#exp-e2)、[§7](4-experiments.md#exp-e3)、[§10.5](4-experiments.md#exp-e5) 节 | 现场授权（会中断机器人的常驻主程序） | 只在 Host 已运行时取数，不重启 |
| `kill -9` 强杀 Host 进程（M1 / M2 的读数就靠它） | [§10.5](4-experiments.md#exp-e5) | 现场授权，且必须是「强杀、不做失力」这一类；同一行要写明谁承托手臂 | 停，只做 teleop oracle |
| 真机受控试动 | 第 [§6.2](4-experiments.md#exp-e2)、[§7.1](4-experiments.md#exp-e3) 节 | 明确的运动授权：哪个关节、多少度、谁在旁、断电开关在手 | 停，只做到 [§6.1](2-host.md#host-start) |
| 真机受控试动 | [§10.7](4-experiments.md#exp-e7)（②③） | 运动授权 + 断电开关在人手上 | 停，只做 ①a/①b |
| 断电与上电 | 第 [§3](#sec-3)、[§8](2-host.md#sec-8-1) 节 | 现场授权 | 停止 |
| 跨机 RTT（PC → Pi 真实网段） | [§5.3](../reference/env-rebuild.md#sec-5-3) | 网络访问授权 | 只记「未测」 |

改 Pi 上的仓库、标定文件或服务配置都不在本手册范围内，需单独授权。

<a id="sec-1"></a>
## 1 硬前置条件

| 编号 | 条件 | 怎么确认 | 不满足怎么办 |
| --- | --- | --- | --- |
| P0 | 宿主广播的 `robot_metadata.schema_version` 是整数 2（已完成，只核对） | [§4.1](3-health.md#sec-4-1) 的 `capture` exit 0，且 JSON 里 `robot_metadata.schema_version` 为 `2` | 停止，不回退代码、不重跑 v1 拒绝路径 |
| P1 | 真机 AlohaMini 2 Pro 在场，物理断电开关可用 | 目视加手测 | 停止 |
| P2 | Pi 上 Host 进程常驻 | [§3](#sec-3) 的 `3.2-host-procs` 与 `3.4-ports` | 按 [§6.1](2-host.md#host-start) 的命令骨架起 Host |
| P3 | PC 与 Pi 同网段 | [§2](#sec-2) SSH 连接检查的 `3.1a-ssh-check` 与 `3.1-ping` | 修网络 |
| P4 | 现场授权（谁授权、授权范围、谁在旁） | 口头确认后落盘，见 [§3](#sec-3)「授权记录」 | 停止 |
| P5 | 轨迹空间足够：单次试动累计行程可达 20° | 卷尺与目视确认该关节正向 20° 无干涉 | 减小 `--steps` |

P0 结论：已关闭。Pi 上的运行时升级到含 metadata v2 的版本，HEAD 为 `6aee1f81`，`capture` exit 0 且 `robot_metadata.schema_version` 为 `2`；旧部署原样备份在 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`（`ec65318f` 加 6 个未提交改动）。现场只做核对。

<a id="sec-2"></a>
## 2 变量块、留痕函数、时间基准

Pi 侧的信息（Host 进程、端口、vendor 代码、配置实际取值）一律用 `ssh` 取，不靠口头转述。理由有两条：口头转述没有落盘证据，出错无法回溯；`run` 函数把命令原文、输出原文、退出码一起写进 `$S4DIR/stdout/`，事后能复核。

### 变量块

先建一个会话内的变量块，粘进当前 shell 一次，后面所有命令都用它：

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
| 脚本运行（`capture_robot_fingerprint.py`、`measure_latency.py`） | Mac | `run 4.1-capture uv run --no-sync python …` |
| 产物文件（JSON、CSV、YAML） | Mac，落 `$S4DIR` | 由 `--out` / `--csv` 指定 |
| Pi 上的 Host 进程与端口（PID、5555、5556） | Pi | `run 3.2-host-procs ssh "$PI" '…'` |
| Pi 上的 vendor 代码（提交、`schema_version` 行、字段默认值） | Pi | `run 4.3b-host-config ssh "$PI" '…'` |
| 寄存器的只读读取 | Pi（Host 已停，串口空闲） | `run 4.2-gripper-regs ssh "$PI" '…'`，见 [§4.2](3-health.md#sec-4-2) |
| 配置取值（`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz` 的实际生效值） | Pi 取原值，写进 Mac 上的 YAML | `run 4.3a-host-args`–`4.3d-config-classes`，见 [§4.3](#sec-4-3) |

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

步骤号即分节号。历史证据文件用的是重排前的旧步骤号，引用原件时按原文件名写。

为什么不用 `| tee`：管道会把 `$?` 变成 `tee` 的退出码，拿到的永远是 `tee` 的 0，与「拿非 0 退出码看 [§10](../reference/gates.md#exit-codes)」直接冲突。这个函数先把输出落盘再 `cat` 回显，退出码取自被调命令本身。

`.cmd` 只记录命令骨架：`$*` 展开会吃掉引号，heredoc 正文也不在里面。需要回放时以 `.log` 与本节正文里的命令原文为准，`.cmd` 只用来对齐步骤号。

用法示例，三条：

```bash
# 本地脚本
run 4.1-capture uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --out "$S4DIR/capture/fingerprint-$STAMP.json"

# Pi 侧命令
run 3.2-host-procs ssh "$PI" 'pgrep -af "alohamini_host|lerobot"'

# 失败用例：验证留痕链路本身
run 0.9-probe false
```

第三条应返回 1，并写出 `$S4DIR/stdout/0.9-probe-$STAMP.exit`，内容为 `1`。写出来的不是 `1`，说明留痕链路没接好：先修 `S4DIR` 与函数定义，再往下做。

### SSH 连接检查与连不上时的处置

```bash
run 3.1a-ssh-check ssh -o ConnectTimeout=5 "$PI" 'echo ok'
```

判据：`.log` 里是 `ok`，`.exit` 是 `0`。

连不上时按顺序处置，每一步都过 `run`：

1. 先确认网络层是否通：

```bash
run 3.1-ping ping -c 3 "$PI_HOST"
```

全丢包是网络问题，有回包但 `ssh` 失败是 ssh 层问题。

2. 试 mDNS 名：

```bash
run 3.1b-ssh-mdns ssh -o ConnectTimeout=5 "pi5@pi5.local" 'echo ok'
```

通了就把地址改成 `pi5.local`：

```bash
export PI_HOST=pi5.local
export PI="pi5@$PI_HOST"
```

然后重跑 `3.1a-ssh-check`。

3. 问现场要地址：拿到 Pi 当前的 IPv4 地址，按第 2 步的方式改 `PI_HOST` 与 `PI`，再重跑 `3.1a-ssh-check`。
4. 现场答不上来时，在路由器管理页查 DHCP 列表，找主机名 `pi5` 或对应 MAC 的地址。

本节产物：`$S4DIR/stdout/3.1a-ssh-check-*`、`3.1b-ssh-mdns-*`、`3.1-ping-*`、`0.9-probe-*`。

<a id="sec-3"></a>
## 3 到场核对

`$S4DIR` 与 `$STAMP` 由 [§2](#sec-2) 的变量块给出，本节不重复定义；证据目录也已由那个块建好，本节不再新建。

### 3.1 网络可达

```bash
run 3.1-ping ping -c 3 "$PI_HOST"
```

判据：3 个包全回，丢包 `0%`。全丢包是网络问题；有回包但 `ssh` 失败是 ssh 层问题，按 [§2](#sec-2) 的四步处置。

### 3.2 Host 进程在跑

```bash
run 3.2-host-procs ssh "$PI" 'pgrep -af "alohamini_host|lerobot"'
```

判据：`.log` 里至少一行，行首有 PID。没有输出说明 Host 没起：按 [§6.1](2-host.md#host-start) 的命令骨架起 Host，再往下做。

`pgrep` 默认按扩展正则解释，`|` 直接写，不要转义。写成 `\|` 时 ERE 把反斜杠加竖线当字面量，有进程也可能匹配不到。

### 3.3 工具存在性自证

```bash
run 3.3-tools ssh "$PI" 'command -v ss netstat; echo "---"; ss -ltn | head -20'
```

这一条只回答一个问题：`ss` / `netstat` 在不在。`command -v` 打出路径即存在；`ss -ltn` 那一段不看端口，只看工具本身能不能跑。

### 3.4 5555 与 5556 在听

```bash
run 3.4-ports ssh "$PI" 'ss -ltnp | grep -E ":5555|:5556"'
```

Pi 上没有 `ss` 时（`.log` 里出现 `ss: command not found`，退出码非 0）退回 `netstat`：

```bash
run 3.4b-ports-netstat ssh "$PI" 'netstat -ltnp | grep -E ":5555|:5556"'
```

判据：两个端口各至少一行 `LISTEN`。

`3.4` 空输出有两种可能：工具缺失，或没有监听。`3.3-tools` 用来区分这两种。**空输出本身不构成「Host 未运行」的证明**，要结合 `3.2` 的进程检查一起判。2026-10-25 实测：`ss` 与 `netstat` 在 Pi 上都存在（`/usr/bin/ss`、`/usr/bin/netstat`），`3.4` 两条输出为空是因为确实没有 LISTEN 的 5555/5556。该结论由 2026-10-25 的独立复核（`command -v` 与 `ss -ltn`）得出，`3.3-tools` 这一步本身没有取到它。

### 3.5 Pi 地址核对

```bash
run 3.5-pi-ip ssh "$PI" 'hostname -I'
```

判据：输出里的 IPv4 与 `$PI_HOST` 一致。不一致就按 [§2](#sec-2) 的方式改 `PI_HOST` 与 `PI`，重跑 `3.1a-ssh-check`。

`--host` 一律写 `"$PI_HOST"`。上一次现场记录的地址是 `10.34.170.98`（来源 `field-run-summary.md` 第 1 行「远端」）。

### 3.6 两个脚本的参数能解析

```bash
run 3.6-capture-help uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --help
run 3.6b-measure-help uv run --no-sync python benchmarks/alohamini_io/measure_latency.py --help
```

判据：两条均 `.exit` 为 `0`。

### 3.7 设备路径（只读）

串口与相机节点，本机与 Pi 各取一份：

```bash
run 3.7-serial ls -l /dev/serial/by-id/
run 3.7b-serial-pi ssh "$PI" 'ls -l /dev/serial/by-id/'
run 3.7c-video ls -l /dev/video*
run 3.7d-video-pi ssh "$PI" 'ls -l /dev/video* 2>/dev/null | head -20'
```

`3.7b` 与 `3.7c` 的判据不能只看 `.exit` 是否为 0。`ls` 在无匹配时返回 2；2026-10-25 实测 Pi 上没有 `/dev/ttyUSB*`，末条 `ls` 无匹配使整条命令 exit 2，而 `by-id` 与 `ttyACM` 的输出完全正常。判据是看 `.log`：`3.7b` 要有 `/dev/ttyACM*` 行，`3.7c` 要有 `/dev/video*` 行。`.exit` 为 2 但 `.log` 里有该有的行，这一条算取到。

标定文件核对（换机或重装后要确认标定文件还在）：

```bash
run 3.7e-calib ssh "$PI" 'ls -l ~/.cache/huggingface/lerobot/calibration/robots/*/ 2>/dev/null; find ~ -maxdepth 5 -name "*.json" -path "*calibration*" 2>/dev/null | head -20'
```

判据：输出里能找到这次要用的标定文件，并记下大小。2026-10-25 实测：Pi 上有 `robots/alohamini/AlohaMiniRobot.json`（2880 B）与 lekiwi 的两个，另有 `/home/pi5/b02-b03-evidence/calibration-alohamini2pro.json`。

设备分工（2026-10-25 实测）：本机（macOS）没有 `/dev/serial/by-id/`、`/dev/ttyACM*`、`/dev/video*`；Pi 上有 `ttyACM0`/`ttyACM1`（两块 `usb-1a86` 单串口，`5B14029263` 与 `5B14111703`）与 20 个 `/dev/video*` 节点。本机那两条命令在没接机械臂的 Mac 上取不到，不能据此判「设备缺失」。

### 3.8 产物路径纪律：一律显式传 `--out` / `--csv`

默认文件名有两处要当心：两个脚本的默认产物目录是 `artifacts/alohamini-io/`，相对当前工作目录解析，不以仓库根为基准；文件名里的时间戳只到秒（格式 `YYYYmmddTHHMMSSZ`），同一秒内跑两次会静默覆盖前一份（`write_text` 直接盖写）。所以一律显式传 `--out` / `--csv`，不依赖默认文件名。

`run` 写的 `.cmd/.log/.exit` 是留痕，不算本节产物。

### 3.9 授权记录落盘

授权范围要写成文字留档，否则事后无法核对「当时到底允许了什么」。写一个本地文件，把五项填满：

```bash
cat > "$S4DIR/stdout/scope-$STAMP.md" <<'EOF'
- 时间（UTC）：
- 授权人：
- 授权范围（关节、度数、谁在旁）：
- 允许的动作：只读 / 启停 Host / 试动 / 断电
- 不允许的动作：
EOF
```

这一条不过 `run`：`run` 的 `.cmd` 只记命令骨架、不含 heredoc 正文（见 [§2](#sec-2)），所以授权记录单独成文件。

判据：文件在盘，五项都有内容。产物：`stdout/scope-$STAMP.md`。

### 3.10 人工确认项

没有机器判据，只能人工确认：

- [ ] 断电开关位置已知，手能一次按到。
- [ ] 机器人已上电。
- [ ] 单臂、空场、无负载三项在 [§7.1](4-experiments.md#exp-e3) 开跑前再确认一次。

本节产物：`$S4DIR/stdout/` 下的 `3.1-ping-*`、`3.2-host-procs-*`、`3.3-tools-*`、`3.4-ports-*`（或 `3.4b-ports-netstat-*`）、`3.5-pi-ip-*`、`3.6-capture-help-*`、`3.6b-measure-help-*`、`3.7-serial-*`、`3.7b-serial-pi-*`、`3.7b2-serial-links-*`、`3.7c-video-*`、`3.7d-video-pi-*`、`3.7e-calib-*`、`scope-*`。

<a id="sec-4-3"></a>
### 4.3 部署取值核对

前置条件：Host 在跑（要读它的启动命令行）；不动机器人。

#### 三个数值字段用命令取

下面五条都在 Pi 侧跑：

```bash
run 4.3a-host-args ssh "$PI" 'pgrep -af alohamini'
run 4.3a2-host-cmdline ssh "$PI" 'pid=$(pgrep -f alohamini_host.py | head -1); echo "pid=$pid"; [ -n "$pid" ] && ps -o pid,cmd -p "$pid" && tr "\0" " " < /proc/$pid/cmdline; echo'
run 4.3b-host-config ssh "$PI" 'grep -rnE --include="*.py" --exclude-dir=__pycache__ "observation_request_window|watchdog_timeout_ms|max_loop_freq_hz" ~/lerobot_alohamini/src/lerobot/robots/alohamini/ | head -20'
run 4.3c-config-args ssh "$PI" 'grep -n "add_argument" ~/lerobot_alohamini/src/lerobot/robots/alohamini/alohamini_host.py'
run 4.3d-config-classes ssh "$PI" 'sed -n "95,145p" ~/lerobot_alohamini/src/lerobot/robots/alohamini/config_alohamini.py'
```

`4.3a2` 的两条配套判断规则：

- `ps -o cmd` 的输出里必须出现 `python`，否则说明 Host 未运行、取到的是别的东西。此时该项记为「未取到」，不要当作实际启动参数。
- 不要用 `pgrep -f alohamini`：2026-10-25 实测它会匹配到 ssh 远端那条 `bash -c …` 自己（当时 `pid=2060`），`/proc/$pid/cmdline` 读到的是探针自己的命令。必须写 `alohamini_host.py`。

`4.3b` 的 `--include="*.py" --exclude-dir=__pycache__` 不能去掉：按目录 grep 会命中 `__pycache__` 的 `.pyc`，输出里多出 `grep: …: binary file matches` 并占掉 `head -20` 的名额（2026-10-25 实测混进 3 行）。

`4.3c` 用来确认这三个字段有没有命令行入口。实测只有 5 处 `add_argument`（`--robot_model`、`--no_follower`、`--profile_timing`、`--camera-stream`、`--camera-stream-port`），都不涉及这三个字段。

`4.3d` 用来看清默认值属于哪个 dataclass。实测 `AlohaMiniHostConfig` 与 `AlohaMiniClientConfig` 各有一个 `observation_request_window`（`config_alohamini.py:98` 与 `:123`），取 Host 那个。

取值规则：

- 先用 `4.3a2` 看实际启动命令行里有没有覆盖。当前 Pi 的 vendor 版本没有这三项的命令行入口，因此实际生效值等于 `AlohaMiniHostConfig` 的源码默认值，用 `4.3b` 取。
- 将来厂商版本若新增入口，以 `4.3a2` 的实际输出为准。
- 写进现场 YAML 的必须是「实际生效的那个值」，并在 `$S4DIR/stdout/` 里留下取得它的命令（`4.3a`–`4.3e-*.cmd`）与输出（`4.3a`–`4.3e-*.log`）。

2026-10-25 结论：Host 实际启动命令为 `/home/pi5/miniconda3/envs/lerobot_alohamini/bin/python -m lerobot.robots.alohamini.alohamini_host --robot_model alohamini2pro`，不含任何覆盖这三个字段的参数，所以源码默认值就是实际生效值。

#### 两份 YAML 的字段与分工

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

| 文件 | 是什么 | 三个数值字段 |
| --- | --- | --- |
| `configs/template.yaml` | 随库分发的通用默认，换机器可原样使用 | `1` / `500` / `50`，取自 `src/alohamini_cartesian/config/overlay.py` 的 `AlohaMiniSettings` 默认值 |
| `configs/site.yaml` | 本机现场值，即 Host 进程实际生效的那份 | 数字本身以 `configs/site.yaml` 为准，本节不复制一份当权威 |

`configs/site.yaml` 的注释里写「见 `docs/field-ops/` 下讲 §4.3 那份文档」（旧文件名对照见 [reference/mapping.md](../reference/mapping.md)），指的就是本节。

三个数值字段的来路（只记「这些数字怎么来的」，取值以 `configs/site.yaml` 为准）：

| 字段 | 依据（来路） |
| --- | --- |
| `observation_request_window` | `config_alohamini.py:98`（`AlohaMiniHostConfig` 源码默认）；该项无命令行入口 |
| `watchdog_timeout_ms` | `config_alohamini.py:110` |
| `max_loop_freq_hz` | `config_alohamini.py:113` |

另外四个字段的取法：

| 字段 | 取法 |
| --- | --- |
| `robot_model` | [§4.1](3-health.md#sec-4-1) 产物 `$S4DIR/capture/fingerprint-$STAMP.json` 里 `robot_metadata.robot_model` 的值 |
| `left_port` / `right_port` | [§3.7](#sec-3) 的 `3.7b-serial-pi` 输出（看 `/dev/ttyACM*` 行） |
| `cameras` | [§3.7](#sec-3) 的 `3.7d-video-pi` 输出，再加实际相机接法 |

覆盖顺序：默认值 → template → site → 环境变量（`ALOHAMINI_*`），由 `overlay.py` 的 `load_settings` 实现。两份一起加载的契约由 `tests/test_examples_and_configs.py` 的 `test_template_and_site_load_together_and_the_three_fields_follow_site_yaml` 锁定：三个字段的值等于直接从 `configs/site.yaml` 原文读出来的值。

#### 读取校验

```bash
run 4.3e-load-check uv run --no-sync python -c "from pathlib import Path; from alohamini_cartesian.config.overlay import load_settings; print(load_settings(template=Path('$S4DIR/configs/template.yaml'), site=Path('$S4DIR/configs/site.yaml'), check_devices=False))"
```

判据：`.exit` 为 `0`，`.log` 里打印出合并后的 `AlohaMiniSettings`。

设备路径填的是**将来运行本库那台机器**上的路径，换一台机器未必通用：

- 真正部署时本库跑在接了机械臂的那台机器上（本项目是 Pi），YAML 里写 Pi 侧路径（`/dev/ttyACM0`、`/dev/ttyACM1`、`/dev/video*`），在这台机器上 `load_settings(check_devices=True)` 才成立。
- 若只在 Mac 上做配置结构校验，用 `load_settings(check_devices=False)`（这是默认值），只验键名、类型、取值，**不验设备存在性**。此时设备存在性的证据来自 Pi 侧的 `3.7b`/`3.7d` 输出，不要因为本机 `ls` 为空就判「设备缺失」。
- `check_devices=True` 在 Mac 上必然失败（本机没有这些 `/dev/` 节点），这不是配置写错，是跑错机器。

`check_devices=True` 这一支本轮未跑：它需要在接了机械臂的 Pi 上跑，且 Pi 上要装了本库，命令未在本轮验证。

#### 两个现场易错点

`watchdog_timeout_ms` 两处默认值不同：本库 `src/alohamini_cartesian/config/overlay.py` 的 `AlohaMiniSettings` 默认 `500`，厂商 `config_alohamini.py` 默认 `1000`（不写死行号：行号随厂商提交变化，2026-10-25 在 Pi 上约 122 行）。核对宿主实参时不要混用，样例里写实际生效的那个值。

`robot_model` 两处默认值不一致：本库 `overlay.py:51` 是 `alohamini1`；两个现场脚本的 `DEFAULT_ROBOT_MODEL` 都是 `alohamini2pro`（`benchmarks/alohamini_io/capture_robot_fingerprint.py:96`、`benchmarks/alohamini_io/measure_latency.py:537`）。现场机型是 AlohaMini 2 Pro，而代码 Gate C 要求宿主 metadata 的 `robot_model` 与 `--robot-model` 相等（比对在 `measure_latency.py:2382-2396`）。现场 YAML 必须显式写 `robot_model: alohamini2pro`，不能用模板里的 `alohamini1`，否则配置表达的意思与真机不符。

本节产物：`configs/template.yaml`、`configs/site.yaml`，`stdout/4.3a-host-args-*`、`4.3a2-host-cmdline-*`、`4.3b-host-config-*`、`4.3c-config-args-*`、`4.3d-config-classes-*`、`4.3e-load-check-*`，以及 [§4.1](3-health.md#sec-4-1)、[§4.1b](3-health.md#sec-4-1b)、[§4.2](3-health.md#sec-4-2) 的产物。

