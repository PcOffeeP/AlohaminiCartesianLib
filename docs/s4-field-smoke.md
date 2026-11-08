# S4 现场执行手册：真机 smoke

这份文档是一次现场窗口的执行手册，从到场核对写到收尾，每一节给命令、判据与产物。现场由 Agent 通过 SSH 连上 Pi，照着逐条取数、执行，不靠临时决策补缺失的信息。

实测边界：第 3、4.1、4.3、5、7.1 节的命令在 2026-10-25 对 `pi5@10.34.170.98` 跑过一轮，产物在证据目录 `evidence/s4-20261025/`，数字汇总见 `docs/field-results.md`。第 4.2、5.3、6.1、6.2、7.2 节原定为本轮新增、未在现场执行过；2026-11-05 的现场会话（远程 SSH，Pi 运行时 HEAD `6aee1f81`、lerobot `0.6.1`）已把这五节与 §4.1 的 Pi 侧 capture 跑过一轮，命令与本文原写法不同之处已在对应小节写明，数字汇总见 `docs/field-results.md` 的 2026-11-05 节。首跑若输出形态与本文不符，按实际输出修正，并回写到本文档。2026-11-05 之后新增的 §4.1c、§4.2 的「全 18 电机静态体检」、§4.4、§7.1b 已在 2026-11-06 的现场窗口跑过（证据目录 `evidence/s4-20261106/`，数字汇总见 `docs/field-results.md` 的 2026-11-06 节）；§4.5 只跑过 ssh 连通性检查，`stash list` / `reflog` / `status --porcelain` 三份原文未取；改写后的 §10.5 仍未在任何现场跑过：命令形式在本地验过（`--help`、`--delta-deg` 的负值与零值行为、位置比对脚本的自检与变异自检），现场输出形态以首次执行为准。

## 0 这份文档怎么用

执行顺序固定，按下面四步走：

1. 先读第 1 节，把 P0–P5 逐条判完。判不了的条件不要跳过：`P1`–`P5` 任一不满足，就停在那一步。
2. 在同一个 shell 里粘第 2 节的变量块与 `run` 函数，再跑一次 `0.9-probe false` 自证留痕链路。后面所有命令都假定这几步已经做完。
3. 按第 3 节到第 8 节的顺序执行。每节开头写前置条件，每步命令之后写判据与产物。
4. 拿到非 0 退出码先查第 10 节；要处置查第 12 节；查已发现的问题查第 9 节；交回前按第 13 节逐条勾选。

步骤号说明：`run` 的步骤号就是本文的分节号（§3.4 用 `3.4-ports`，§4.3 用 `4.3a-host-args`），产物名里的步骤号也是它。历史证据文件仍沿用重排前的旧步骤号命名（`0.*`、`1.*`、`3.1-tracking` 等），文中引用原件时按原文件名写。

新步骤号 `4.1b-lerobot-version` 在执行顺序上**先于 4.1**（它是 4.1 的输入），与后文的 `4.1b-probe`（校准字段齐全性探测）是两件事，只共用 `4.1b` 前缀，产物文件名不冲突。

`4.1c` / `4.4` / `4.5` / `7.1b` 同样是插入式编号，现有分节号一个都没重排：`4.1c` 跟着 `4.1`、`4.1b` 走（断电期间位置比对），`4.4` / `4.5` 接在 `4.3` 之后仍属「§4 只读确认」，`7.1b` 是 `7.1` 的反向拒绝取证。

一页总览：

| 节 | 内容 | 时长 | 是否动机器人 | 主要产物 | 完成判据 |
| --- | --- | --- | --- | --- | --- |
| 3 到场核对 | 网络、Host 进程、端口、工具、设备路径、授权留痕 | 10 分钟 | 否 | `stdout/3.*`、`stdout/scope-*` | 两条 `--help` 均 exit 0；5555 与 5556 各至少一行 `LISTEN` |
| 4 只读确认 | capture 指纹、断电期间位置比对（4.1c）、校准字段探测、左夹爪 ID7 寄存器与全 18 电机静态体检、部署取值、库真机 dry-run（4.4）、Pi 未提交文件调查（4.5） | 本文未设预算 | 否 | `capture/fingerprint-$STAMP.json`、`stdout/4.2-gripper-regs-*`、`configs/*.yaml`、`capture/fingerprint-postpower-*.json`、`stdout/4.2-all18-*`、`stdout/4.4-*`、`stdout/4.5*` | `capture` exit 0 且 `robot_metadata.schema_version` 为 `2`；`load_settings(check_devices=False)` 读通 |
| 5 零运动测量 | 部署形态 rtt、RTT 形态 rtt、跨机 rtt | 本文未设预算 | 否 | `rtt/` 三份 CSV | 三条均 exit 0 且各有数据行 |
| 6 量级验证 | 静止电流基线、短运动电流量级 | 本文未设预算 | 6.2 走 2 步 | `stdout/6.1*`、`stdout/6.2*` | 两段 `[Currents]` 原文与计数在盘，并写出「电流能不能判读」的结论 |
| 7 受控试动 | 开跑勾选单、单关节步进、肩部目标接受判定 | 本文未设预算 | 是，单臂单关节 | `tracking/tracking-$STAMP.csv`、其 sidecar、`stdout/7.2-*` | exit 0；exit 8 按第 12 节处置 |
| 8 收尾 | 断电时序、Host 停启留痕、产物归档 | 本文未设预算 | 否 | `stdout/8.*` | 每步三件套齐全，Host 停启有记录 |

只有到场核对有预算（10 分钟），依据是本次任务的约定。其余各节不设预算，实际耗时记进 `$S4DIR/stdout/` 存档。

哪些步骤要现场授权：

| 动作 | 在哪节 | 需要哪类授权 | 没有授权时 |
| --- | --- | --- | --- |
| SSH 到 Pi 执行只读命令 | 第 3、4.1、4.2、4.3、4.4、4.5、5.3、7.1b、10.5 节 | 现场网络与登录授权 | 不能开工，停在第 1 节 |
| 向 Pi 写文件（把寄存器脚本投到 `/tmp`） | 第 4.2 节 | 向 Pi 写文件的授权 | 改为在 Pi 上手工输入同一脚本，输入方式需现场确认 |
| 启停 Host 进程 | 第 6.1、6.2、7、10.5 节 | 现场授权（会中断机器人的常驻主程序） | 只在 Host 已运行时取数，不重启 |
| `kill -9` 强杀 Host 进程（M1 / M2 的读数就靠它） | 第 10.5 节 | 现场授权，且必须是「强杀、不做失力」这一类；同一行要写明谁承托手臂 | 停，只做 teleop oracle |
| 真机受控试动 | 第 6.2、7.1 节 | 明确的运动授权：哪个关节、多少度、谁在旁、断电开关在手 | 停，只做到第 6.1 节 |
| 断电与上电 | 第 3、8 节 | 现场授权 | 停止 |
| 跨机 RTT（PC → Pi 真实网段） | 第 5.3 节 | 网络访问授权 | 只记「未测」 |

改 Pi 上的仓库、标定文件或服务配置都不在本手册范围内，需单独授权。

## 1 硬前置条件

| 编号 | 条件 | 怎么确认 | 不满足怎么办 |
| --- | --- | --- | --- |
| P0 | 宿主广播的 `robot_metadata.schema_version` 是整数 2（已完成，只核对） | 第 4.1 节的 `capture` exit 0，且 JSON 里 `robot_metadata.schema_version` 为 `2` | 停止，不回退代码、不重跑 v1 拒绝路径 |
| P1 | 真机 AlohaMini 2 Pro 在场，物理断电开关可用 | 目视加手测 | 停止 |
| P2 | Pi 上 Host 进程常驻 | 第 3 节的 `3.2-host-procs` 与 `3.4-ports` | 按第 6.1 节的命令骨架起 Host |
| P3 | PC 与 Pi 同网段 | 第 2 节 SSH 连接检查的 `3.1a-ssh-check` 与 `3.1-ping` | 修网络 |
| P4 | 现场授权（谁授权、授权范围、谁在旁） | 口头确认后落盘，见第 3 节「授权记录」 | 停止 |
| P5 | 轨迹空间足够：单次试动累计行程可达 20° | 卷尺与目视确认该关节正向 20° 无干涉 | 减小 `--steps` |

P0 结论：已关闭。Pi 上的运行时升级到含 metadata v2 的版本，HEAD 为 `6aee1f81`，`capture` exit 0 且 `robot_metadata.schema_version` 为 `2`；旧部署原样备份在 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`（`ec65318f` 加 6 个未提交改动）。现场只做核对。

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
| 寄存器的只读读取 | Pi（Host 已停，串口空闲） | `run 4.2-gripper-regs ssh "$PI" '…'`，见第 4.2 节 |
| 配置取值（`observation_request_window`、`watchdog_timeout_ms`、`max_loop_freq_hz` 的实际生效值） | Pi 取原值，写进 Mac 上的 YAML | `run 4.3a-host-args`–`4.3d-config-classes`，见第 4.3 节 |

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

为什么不用 `| tee`：管道会把 `$?` 变成 `tee` 的退出码，拿到的永远是 `tee` 的 0，与「拿非 0 退出码看第 10 节」直接冲突。这个函数先把输出落盘再 `cat` 回显，退出码取自被调命令本身。

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

## 3 到场核对

`$S4DIR` 与 `$STAMP` 由第 2 节的变量块给出，本节不重复定义；证据目录也已由那个块建好，本节不再新建。

### 3.1 网络可达

```bash
run 3.1-ping ping -c 3 "$PI_HOST"
```

判据：3 个包全回，丢包 `0%`。全丢包是网络问题；有回包但 `ssh` 失败是 ssh 层问题，按第 2 节的四步处置。

### 3.2 Host 进程在跑

```bash
run 3.2-host-procs ssh "$PI" 'pgrep -af "alohamini_host|lerobot"'
```

判据：`.log` 里至少一行，行首有 PID。没有输出说明 Host 没起：按第 6.1 节的命令骨架起 Host，再往下做。

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

判据：输出里的 IPv4 与 `$PI_HOST` 一致。不一致就按第 2 节的方式改 `PI_HOST` 与 `PI`，重跑 `3.1a-ssh-check`。

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

这一条不过 `run`：`run` 的 `.cmd` 只记命令骨架、不含 heredoc 正文（见第 2 节），所以授权记录单独成文件。

判据：文件在盘，五项都有内容。产物：`stdout/scope-$STAMP.md`。

### 3.10 人工确认项

没有机器判据，只能人工确认：

- [ ] 断电开关位置已知，手能一次按到。
- [ ] 机器人已上电。
- [ ] 单臂、空场、无负载三项在第 7.1 节开跑前再确认一次。

本节产物：`$S4DIR/stdout/` 下的 `3.1-ping-*`、`3.2-host-procs-*`、`3.3-tools-*`、`3.4-ports-*`（或 `3.4b-ports-netstat-*`）、`3.5-pi-ip-*`、`3.6-capture-help-*`、`3.6b-measure-help-*`、`3.7-serial-*`、`3.7b-serial-pi-*`、`3.7b2-serial-links-*`、`3.7c-video-*`、`3.7d-video-pi-*`、`3.7e-calib-*`、`scope-*`。

## 4 只读确认（不动机器人）

本节全部是只读动作：4.1 只请求观测，4.1b 只读 JSON，4.2 只读伺服寄存器（Host 已停、串口空闲），4.3 只在 Pi 上读代码与配置。

### 4.1b-lerobot-version 取 Pi 上 lerobot 版本（只读，先于 4.1 执行）

4.1 的指纹 JSON 要记 Pi 端 Host 用的 lerobot 版本；Mac 上没装 lerobot，所以先从 Pi 取，4.1 时把取到的值传进去。

```bash
run 4.1b-lerobot-version ssh "$PI" '~/miniconda3/envs/lerobot_alohamini/bin/python -c "import importlib.metadata; print(importlib.metadata.version(\"lerobot\"))"'
```

把取到的值收进 shell 变量，下一步 4.1 要引用：

```bash
export PI_LEROBOT_VERSION=$(tr -d '[:space:]' < "$S4DIR/stdout/4.1b-lerobot-version-$STAMP.log")
echo "PI_LEROBOT_VERSION=$PI_LEROBOT_VERSION"
```

判据：`.log` 里只有一行形如 `0.6.1` 的版本号，`$PI_LEROBOT_VERSION` 非空。若出现 `Traceback` / `PackageNotFoundError` / 空行，说明该解释器没装 lerobot 或路径变了 → 记「未取到」，**不要**手填版本号，4.1 就**不要**传 `--lerobot-version`（JSON 里会记 `null`，那是诚实的缺失记录）。

产物：`stdout/4.1b-lerobot-version-$STAMP.{cmd,log,exit}`。

### 4.1 capture 指纹

```bash
run 4.1-capture uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --lerobot-version "$PI_LEROBOT_VERSION" --out "$S4DIR/capture/fingerprint-$STAMP.json"
```

判据：exit 0，`$S4DIR/capture/fingerprint-$STAMP.json` 写出，JSON 里 `robot_metadata.schema_version` 为 `2`。P0 就靠这一条判定。

第二条判据：JSON 里 `lerobot_version` 非 `null`，且与 `4.1b-lerobot-version` 取到的 `$PI_LEROBOT_VERSION` **逐字一致**。

`4.1` 的默认值：`--samples 5`、`--timeout-s 30.0`、`--port 5556`；`--lerobot-version` 不给时回落到本机已装的 `lerobot`，本机也没有就记 `null`；空串会被拒绝（exit 2）。

产物：`capture/fingerprint-$STAMP.json`、`stdout/4.1-capture-*`。

#### capture 在 Pi 侧跑（2026-11-05 实战做法）

本次实战里 capture 是**在 Pi 上跑的**（Mac 主环境没有 `lerobot`），因此 JSON 的 `host` 字段记的是 `127.0.0.1`。命令形态是「脚本经 stdin 投递 + Pi 侧 venv 解释器 + `PYTHONPATH=/home/pi5/lerobot_alohamini/src`」，产物落 Pi 的 `/tmp`，再用 `scp` 取回 `$S4DIR/capture/`。命令原文（路径按现场替换）：

```bash
ssh "$PI" 'cd /tmp && PYTHONPATH=/home/pi5/lerobot_alohamini/src ~/miniconda3/envs/lerobot_alohamini/bin/python - --host 127.0.0.1 --lerobot-version 0.6.1 --samples 5 --timeout-s 30.0 --out /tmp/fingerprint-$STAMP.json' < benchmarks/alohamini_io/capture_robot_fingerprint.py
```

判据补充：`host` 字段是 `127.0.0.1` 时，说明 capture 在被测机本机回环上跑，不是 Mac 到 Pi 的跨机路径——不要当成网络路径写进记录。

#### capture 的 JSON 有 11 个顶层键

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
| `lerobot_version` | Pi 端 Host 用的 lerobot 版本（`--lerobot-version` 原样传入）；取不到时为 `null`，不伪造 |

#### 这个 JSON 里有两个 `schema_version`

- 顶层 `schema_version` 是 `1`，它表示 payload 格式版本，永远不是 2。
- 要等于 2 的是 `robot_metadata.schema_version`。

现场最常见的误判是把顶层那个 `1` 当成失败原因。脚本只校验 `robot_metadata.schema_version == 2` 与 `motors` 非空映射，不校验每个电机的 8 个字段是否齐全。8 字段是 `id`、`model`、`normalization`、`drive_mode`、`range_min`、`range_max`、`homing_offset`、`resolution`，齐不齐由宿主决定，现场逐电机记录实际覆盖情况。

v2 宿主（HEAD `6aee1f81`）上 18 个电机的 `homing_offset` 与 `resolution` 全部有值，无 `null`。升级前的 v1 宿主给每个电机只写 6 个键，没有这两个字段。

### 4.1b 校准字段齐全性探测（只读）

先读 4.1 的 JSON，逐电机看校准字段是否齐全。这条不动机器人，只读盘上的文件：

```bash
run 4.1b-probe python - <<'PY'
import json, os
from pathlib import Path
s4 = Path(os.environ["S4DIR"])
paths = sorted(s4.glob("capture/fingerprint-*.json"))
assert paths, "先跑 4.1，拿到 fingerprint JSON"
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

- 只有当某个电机的 `missing` 或 `bad` 非空时，它对应的关节才会在代码 Gate F 第二段被拒。
- v2 宿主上 18/18 电机字段齐全，Gate F 第二段不会被触发。此时只记录「v2 宿主下 Gate F 未触发」这一事实，理由写进 `$S4DIR/stdout/`。
- 记录证据时必须写明命中的是哪一段。两种拒绝要分开：`schema_version` 不是 2 → Gate F 第一段拒绝；换算字段不全 → Gate F 第二段拒绝。

产物：`stdout/4.1b-probe-*`。历史同类产物是 `stdout/5.1-probe21-20261025T094457Z.log`，输出为 18 个电机逐项 `missing=[] bad=[]`，即第二段未触发。

### 4.1c 断电期间移位检查：标定字段与位置分开比对（只读）

什么时候跑：**上一轮收工断电 → 本轮上电、Host 起来之后**，第一件只读动作就是这一节。它回答两个互不混同的问题：

1. 标定有没有被改过（`Min_Position_Limit` / `Max_Position_Limit` / `Homing_Offset` / `Drive_Mode` 一类）。任一不等 = **事故**，不是滑移。
2. 断电期间关节有没有滑移（位置变了、标定没变）。

前置：第 1 节 P0–P5 全过；Host 常驻（`capture` 要连 5556）；本轮授权含只读与启停 Host。本节不动机器人。

#### 第一步：采一份新指纹

沿用第 4.1 节的 `capture`，产物名换掉，避免覆盖基线：

```bash
run 4.1c-capture uv run --no-sync python benchmarks/alohamini_io/capture_robot_fingerprint.py --host "$PI_HOST" --out "$S4DIR/capture/fingerprint-postpower-$STAMP.json"
```

命名口径：过程指纹固定 `fingerprint-$STAMP.json`，收尾断电前固定 `fingerprint-postpower-$STAMP.json`，两者后缀不同、互不覆盖；同一 STAMP 下还要采 §4.1c 的一份时，另存 `fingerprint-postpower-$STAMP.4.1c-preserved.json` 并在台账点名（2026-11-08 现场即此情形）。

判据：exit 0，`robot_metadata.schema_version` 为 `2`。**本节故意不传 `--lerobot-version`**：那个变量由 §4.1b-lerobot-version 导出，而本节常被当成「上电后第一件事」跑；传空串会被 `capture_robot_fingerprint.py:310-315` 以 exit 2 拒（原文 `--lerobot-version must not be empty`）。本节只判 `schema_version`，不判版本字段，所以整条标志省掉最稳；`lerobot_version` 记成 `null` 是诚实缺失，不是失败（口径同 §4.1）。
若主环境没有 `lerobot` 而以 **exit 3** 结束，改用第 4.1 节「capture 在 Pi 侧跑」的同一形态（Pi 侧解释器 + `PYTHONPATH=/home/pi5/lerobot_alohamini/src`），产物再用 `scp` 取回 `$S4DIR/capture/`。拿到 exit 2 说明命令本身写错了（多半是给了一个空变量），先修命令，别当成设备问题。

#### 第二步：一行能抄的比对命令

比对基准是**上一次收工断电前**采的现场指纹，现指向 `~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/capture/fingerprint-postpower-20261106T070942Z.json`（`captured_utc=2026-11-06T07:10:31+00:00`，`robot_metadata.schema_version=2`，18 个电机（左 11 + 右 7），`lerobot_version="0.6.1"`，`host=127.0.0.1`，`sample_count=5`；同一 STAMP 的产物另带 `stdout/4.1c-capture-20261106T070942Z.*` 与 `stdout/4.1c-capture-scp-20261106T070942Z.*`）。上一份 `fingerprint-postpower-20261106T064429Z.json`（`captured_utc=2026-11-06T06:45:05+00:00`）**已作废为位置基准**：2026-11-06 07:00Z 后用户换新电池并重新上电（`stdout/scope-append-20261106T070942Z.md` ①），位置跨过换电不再连续，它只剩标定字段可作历史对照；换电那一轮因此**判不了「断电期间移位」**，这份新基线服务的是下一轮断电。**基准必须采于上一次收工断电前，且此后没有未记录的运动**——否则位置组差值里混着那些运动，超阈也归不了因：2026-11-05 的旧指纹 `evidence/s4-20261104/capture/fingerprint-20261105T063600Z.json` 采于那轮试动之前、跨过了试动，位置组 8 只电机超 3 tick 却无法归因，已作废为**位置**基准（标定字段仍可作历史对照）。做法沿用 2026-11-05 已验证的 `robot_metadata.motors` 子树 diff 思路，但**把标定与位置拆成两段**，判据不同。

**该基线的位置组（B 组）只作归因，不作判据**：`fingerprint-postpower-20261106T070942Z.json`（`captured_utc=2026-11-06T07:10:31Z`）之后另有 07:19–09:06Z 六轮记录在案的运动，所以它的 B 组位置差值只能归到「记录在案的运动」（并据此识别未记录的运动），**不能当「断电期间滑移」的判据**；新基准在本次上电后重采。A 组（标定字段）不受影响，仍可判「标定有没有被改过」。

先把脚本落到 `$S4DIR/stdout/` 并在同一份指纹上自检一次（两侧同一文件，期望两组计数都是 0）：

```bash
cat > "$S4DIR/stdout/4.1c-postpower-diff.py" <<'PY'
#!/usr/bin/env python3
"""§4.1c 断电期间移位检查：标定字段与当前位置分开比对。只读两个指纹 JSON，不连设备。"""
import json
import sys
from pathlib import Path

CALIB_FIELDS = (
    "id", "model", "normalization", "range_min", "range_max",
    "homing_offset", "drive_mode", "resolution",
)
ABSENT = "<absent>"
SPEED_KEYS = ("x.vel", "y.vel", "theta.vel")


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def to_tick(norm, motor):
    """drive_mode=0 情形下与厂商 _unnormalize 同式的换算（motors_bus.py:892-902）。

    厂商实现还有 drive_mode 取反与 ±100 / 0..100 钳制两步，本脚本没有；本机 18 个电机的
    drive_mode 全是 0（指纹可核），所以数值等价。若某轴 drive_mode 变成 1，B 组的 Δ 符号会反
    ——那种变更由 A 组（drive_mode 字段）先拦下来。
    """
    lo, hi = motor["range_min"], motor["range_max"]
    if motor["normalization"] == "range_0_100":
        return int((norm / 100.0) * (hi - lo) + lo)
    return int(((norm + 100.0) / 200.0) * (hi - lo) + lo)


def main(argv):
    if len(argv) != 4:
        print("usage: 4.1c-postpower-diff.py <baseline.json> <today.json> <out.txt>", file=sys.stderr)
        return 2
    base, today, out = load(argv[1]), load(argv[2]), Path(argv[3])
    bm = base["robot_metadata"]["motors"]
    tm = today["robot_metadata"]["motors"]
    bs = base.get("observation_state") or {}
    ts = today.get("observation_state") or {}

    lines = [
        f"# baseline: {argv[1]}  captured_utc={base.get('captured_utc')}",
        f"# today   : {argv[2]}  captured_utc={today.get('captured_utc')}",
        "# rule: calibration fields compared verbatim; positions compared in tick, threshold 3 tick",
    ]

    lines.append("")
    lines.append("== A. 标定字段（任一不等 = 事故，停） ==")
    calib_diffs = 0
    for name in sorted(set(bm) | set(tm)):
        old, new = bm.get(name) or {}, tm.get(name) or {}
        rows = [
            f"{field}: {old.get(field, ABSENT)} -> {new.get(field, ABSENT)}"
            for field in CALIB_FIELDS
            if old.get(field, ABSENT) != new.get(field, ABSENT)
        ]
        if rows:
            calib_diffs += len(rows)
            lines.append(f"{name}:")
            lines.extend(f"  {row}" for row in rows)
    lines.append(f"CALIB_FIELD_DIFFS: {calib_diffs}")

    lines.append("")
    lines.append("== B. 当前位置（tick；阈值 3 tick） ==")
    lines.append(f"{'motor':<26} | {'baseline_tick':>13} | {'today_tick':>10} | {'delta':>6} | verdict")
    pos_over = 0
    for name in sorted(bm):
        key = f"{name}.pos"
        if key not in bs or key not in ts:
            continue
        try:
            old_tick = to_tick(float(bs[key]), bm[name])
            new_tick = to_tick(float(ts[key]), tm.get(name) or bm[name])
        except (KeyError, TypeError, ValueError) as exc:
            lines.append(f"{name:<26} | 换算失败 {type(exc).__name__}: {exc}")
            continue
        delta = new_tick - old_tick
        if abs(delta) > 3:
            pos_over += 1
        lines.append(
            f"{name:<26} | {old_tick:>13} | {new_tick:>10} | {delta:>+6} | "
            f"{'>3 tick，记台账' if abs(delta) > 3 else 'ok'}"
        )
    lines.append(f"POSITION_MOTORS_OVER_3_TICK: {pos_over}")

    lines.append("")
    lines.append("== C. 不在 .pos 口径里的量（原样比，不折 tick） ==")
    for key in ("lift_axis.height_mm", *SPEED_KEYS):
        if key in bs or key in ts:
            lines.append(f"{key}: {bs.get(key, ABSENT)} -> {ts.get(key, ABSENT)}")

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {out}")
    print(f"CALIB_FIELD_DIFFS: {calib_diffs}")
    print(f"POSITION_MOTORS_OVER_3_TICK: {pos_over}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
PY

export BASEFP=~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/capture/fingerprint-postpower-20261106T070942Z.json
run 4.1c-selfcheck uv run --no-sync python "$S4DIR/stdout/4.1c-postpower-diff.py" "$BASEFP" "$BASEFP" "$S4DIR/stdout/4.1c-selfcheck-$STAMP.txt"
```

判据（自检）：exit 0，输出 `CALIB_FIELD_DIFFS: 0` 与 `POSITION_MOTORS_OVER_3_TICK: 0`。自检不过说明脚本本身有问题，先修脚本再比真数据（本脚本的两组计数在本地各做过一次自检与一次变异自检：改 `arm_left_gripper.homing_offset` +1 只进 A 组记 1 条；把 `arm_left_shoulder_pan.pos` 加 0.5 只进 B 组并算成 `+6 tick`）。

比对真数据：

```bash
run 4.1c-postpower-diff uv run --no-sync python "$S4DIR/stdout/4.1c-postpower-diff.py" "$BASEFP" "$S4DIR/capture/fingerprint-postpower-$STAMP.json" "$S4DIR/stdout/4.1c-postpower-diff-$STAMP.txt"
```

#### 比对键与判据

| 组 | 比对什么 | 来源字段 | 判据 | 落空时怎么记 |
| --- | --- | --- | --- | --- |
| A1 标定 | `range_min`、`range_max`、`homing_offset`、`drive_mode`（= 寄存器 `Min_Position_Limit` / `Max_Position_Limit` / `Homing_Offset` / `Drive_Mode`） | `robot_metadata.motors.<motor>.*` | **任一不等 → 停**，标定变动属事故，禁止继续试动、禁止改标定文件去「对齐」 | 记录电机名与字段、两侧原值 |
| A2 拓扑 | `id`、`model`、`normalization`、`resolution` | `robot_metadata.motors.<motor>.*` | 任一不等 → 记录并核对 vendor 升级记录（换型号 / 换分辨率是升级的正常结果，但必须能解释） | 记录电机名与字段、两侧原值，并附 vendor 升级记录原文 |
| B 位置 | 14 个 `arm_*_<joint>.pos` 的 tick 值（用厂商 `_unnormalize` 的同一条公式换算：`range_m100_100` 用 `int(((v+100)/200)*(max-min)+min)`，`range_0_100` 用 `int((v/100)*(max-min)+min)`，`motors_bus.py:892-902`） | `observation_state` | 每电机 \|Δtick\| ≤ **3** → 记「断电无滑移」；> 3 → 把电机名、基准 tick、本次 tick、差值写进台账。3 tick 是本项目自定的「非滑移」上界，不是厂商值：两侧各自 `int()` 截断各带 ≤1 tick 误差，余 1 tick 作为机械回差余量 | 只列超出 3 tick 的电机 |
| C 其它 | `lift_axis.height_mm`（mm，不折 tick）、底盘三轴 `x.vel` / `y.vel` / `theta.vel`（原样比） | `observation_state` | 原样记录；本轮不设阈值 | 记两侧原值 |

三条口径要点，写记录时必须带上：

- **Punch 不在指纹里**。Feetech 的 Punch（起步力，寄存器名 `Minimum_Startup_Force`）只由 §4.2 的 `EXTRA_REGS` 读到，`robot_metadata.motors` 没有这个字段。要比 Punch，先跑 §4.2 并把两轮读数并列，不能拿指纹冒充。
- **B 组只覆盖 14 个臂电机**。指纹 `observation_state` 的观测键里 `lift_axis` 是 `lift_axis.height_mm`（进 C 组），底盘三轴只有速度键、没有位置键（本机指纹里 `base_left_wheel` / `base_back_wheel` / `base_right_wheel` 在 `observation_state` 里没有键）。所以「18 电机位置比对」这个说法要写清是「14 个臂电机按 tick 比 + 升降轴按 mm 比 + 底盘三轴只有速度」。
- **左夹爪的 tick 分辨率极粗**：`range_0_100` 下 `range_min..range_max` 是 `2021..2072`（51 tick），归一化值每变 1 单位约 0.51 tick，小位移会在 tick 上被吞掉。夹爪读数只能当「有没有大位移」看，不能当精细滑移证据。

产物：`capture/fingerprint-postpower-$STAMP.json`、`stdout/4.1c-postpower-diff.py`、`stdout/4.1c-selfcheck-$STAMP.txt`、`stdout/4.1c-postpower-diff-$STAMP.txt`，以及每步的 `.cmd` / `.log` / `.exit` 三件套。


### 4.2 左夹爪 ID7 与全 18 电机只读确认

前置条件：第 1 节 P0–P5 全过；按本节「现场确认项」表的 `3.7b2-serial-links` 确认两个串口别名在（`/dev/am_arm_follower_left` → `ttyACM0`、`/dev/am_arm_follower_right` → `ttyACM1`）；Host 已停（串口被 Host 占着时读不到）；授权记录里写明了「向 Pi 写文件」与「停 Host」。

为什么别名要单独核：第 3.7 节的 `ls -l /dev/serial/by-id/` 只列出 by-id 名（2026-11-06 实测 `usb-1a86_USB_Single_Serial_5B14029263-if00 → ../../ttyACM0`、`…5B14111703-if00 → ../../ttyACM1`，`stdout/3.7b-serial-pi-20261106T062506Z.log`），从这份列表核不出 `am_arm_follower_*` 这两个别名是否存在、指向哪个口——两套名字不能互相代替。

背景一句：离线核对（`docs/field-results.md`「离线核对（无新采集）· 左右夹爪行程差异」）发现 `arm_left_gripper` 的 `range` 是 2021-2072，只有 51 tick（`range_0_100` 下 4.48°），而 `arm_right_gripper` 是 2010-3479，1469 tick（129.14°）。两侧同为 `sts3250`、`drive_mode` 同为 0，根因未确认。`docs/ADR.md` D14 已定本轮不重标定，本步只做只读确认，判成因落在哪一侧。

做法：以证据目录里的 `stdout/reg_read_1792923639.py` 为模板，在 `MOTORS` 里给左臂总线加上夹爪，其余不动：

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

2026-11-05 的实战做法：把探测目标从 `shoulder_pan`（ID1）扩到 `gripper`（ID7），`MOTORS` 里加 `"gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100)`，并把存活探测（`probe` 与波特率重试）换成对 ID7 读 `Model_Number`；左右两臂各跑一次（两个脚本，右臂脚本的 `PORT` 换成右臂总线别名）。

执行方式有三件事，其中两件已定值，一件仍需现场确认：

1. 投递方式：有「向 Pi 写文件」授权就走下面的 scp 支；没有授权就走 stdin 支（不向 Pi 落盘）。
2. 解释器：`/home/pi5/miniconda3/envs/lerobot_alohamini/bin/python`，即 Host 自己用的解释器（第 6.1 节启动命令原文，上次现场已验证存在）。
3. 端口别名：脚本里 `PORT = "/dev/am_arm_follower_left"` 是别名，原脚本注释写它指向 `ttyACM0`。第 3.7 节的输出若显示别名与实际节点不同，按实际改。

把改好的脚本放到 Pi 的 `/tmp/` 属于向 Pi 写文件，需在第 3.9 节的授权记录里写明。

本脚本 2026-11-05 已在真机执行两次：左臂、右臂各一次，两次都是 `EXIT_CODE: 0`（`stdout/4.2-gripper-regs-*`、`stdout/4.2-gripper-regs-right-*`）。实测读数：左臂 ID7 `Min_Position_Limit = 2021` / `Max_Position_Limit = 2072`，命中判据第一支（固件限位生效，成因在标定值，D14 维持不重标定）；右臂 ID7 `2010` / `3479`；两只都是 `Torque_Enable=0`、`Lock=1`。可比性限定：本轮 `Torque_Enable=0` 与 2026-10-25 读到的 ID1/2/3/6 `Torque_Enable=1` 不可比——测量对象不同（本轮读的是夹爪 ID7）。

#### 第一步：在 Mac 上建脚本

只在本地写文件，不涉及 Pi：

```bash
cat > "$S4DIR/stdout/4.2-gripper-regs-$STAMP.py" <<'PY'
# -*- coding: utf-8 -*-
"""READ-ONLY servo register dump for alohamini left arm (ttyACM0).

This script only reads registers. It issues no servo commands other than the
read instruction. Torque state is left exactly as found.
"""
from __future__ import annotations

import json
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.feetech.tables import MODEL_NUMBER_TABLE
from lerobot.motors.motors_bus import Motor, MotorNormMode

PORT = "/dev/am_arm_follower_left"   # -> ttyACM0 (left arm). ttyACM1 (right) is never opened.
BAUDRATE = 1_000_000
WAIT_S = 0.5

MOTORS = {
    "shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),  # 新增：左臂夹爪 ID7
}

REQUIRED_REGS = [
    "Model_Number", "Firmware_Major_Version", "Firmware_Minor_Version",
    "Present_Load", "Torque_Limit", "Max_Torque_Limit",
    "P_Coefficient", "I_Coefficient", "D_Coefficient",
    "Goal_Position", "Present_Position", "Torque_Enable",
    "Operating_Mode", "Protection_Current",
]

EXTRA_REGS = [
    "Present_Velocity", "Present_Current", "Present_Temperature", "Present_Voltage",
    "Status", "Moving", "Minimum_Startup_Force", "CW_Dead_Zone", "CCW_Dead_Zone",
    "Overload_Torque", "Protective_Torque", "Protection_Time", "Unloading_Condition",
    "Max_Temperature_Limit", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit",
    "Goal_Velocity", "Lock", "Velocity_closed_loop_P_proportional_coefficient",
]

DYNAMIC_REGS = ["Present_Position", "Present_Load", "Present_Velocity", "Moving"]

ALL_REGS = REQUIRED_REGS + EXTRA_REGS

results = {}   # name -> {reg: {"ok": bool, "value": int|None, "err_type":..., "err":...}}
pass2 = {}


def read_one(bus, name, reg):
    try:
        v = bus.read(reg, name, normalize=False, num_retry=0)
        return {"ok": True, "value": v, "err_type": None, "err": None}
    except Exception as exc:  # noqa: BLE001 - one bad register must not abort the sweep
        return {"ok": False, "value": None, "err_type": type(exc).__name__, "err": str(exc)}


def main():
    print("READ-ONLY register sweep -- target:", PORT, "baud:", BAUDRATE, flush=True)
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=None)
    bus.connect(handshake=False)          # openPort() only; no servo packet emitted
    print("connected:", bus.is_connected, flush=True)
    bus.set_baudrate(BAUDRATE)            # local UART setting only, no servo packet

    # quick liveness probe on ID1 only
    probe = read_one(bus, "shoulder_pan", "Model_Number")
    print("probe shoulder_pan Model_Number ->", probe, flush=True)
    if not probe["ok"]:
        for br in (1_000_000, 500_000, 115_200):
            bus.set_baudrate(br)
            p2 = read_one(bus, "shoulder_pan", "Model_Number")
            print(f"retry baud={br} ->", p2, flush=True)
            if p2["ok"]:
                break

    for name in MOTORS:
        results[name] = {}
        m = MOTORS[name]
        print(f"--- ID{m.id} {name} (configured model={m.model}) ---", flush=True)
        for reg in ALL_REGS:
            r = read_one(bus, name, reg)
            results[name][reg] = r
            if r["ok"]:
                extra = ""
                if reg == "Model_Number":
                    extra = "  (MODEL_NUMBER_TABLE sts3215=777 sts3250=2825 sts3095=2569)"
                print(f"  {reg:52s} = {r['value']}{extra}", flush=True)
            else:
                print(f"  {reg:52s} = FAIL  {r['err_type']}: {r['err']}", flush=True)

    print(f"--- wait {WAIT_S}s, sample #2 ---", flush=True)
    time.sleep(WAIT_S)
    for name in MOTORS:
        pass2[name] = {}
        print(f"--- ID{MOTORS[name].id} {name} ---", flush=True)
        for reg in DYNAMIC_REGS:
            r = read_one(bus, name, reg)
            pass2[name][reg] = r
            print(f"  {reg:52s} = {r['value'] if r['ok'] else 'FAIL  ' + str(r['err_type']) + ': ' + str(r['err'])}", flush=True)

    # disconnect without touching torque: the keyword below DISABLES the torque-off path
    bus.disconnect(disable_torque=False)
    print("after disconnect: bus.is_connected =", bus.is_connected, flush=True)

    print("=== JSON ===", flush=True)
    print(json.dumps({
        "port": PORT,
        "baudrate": BAUDRATE,
        "wait_s": WAIT_S,
        "motors": {n: {"id": m.id, "configured_model": m.model} for n, m in MOTORS.items()},
        "required_regs": REQUIRED_REGS,
        "extra_regs": EXTRA_REGS,
        "pass1": results,
        "pass2": pass2,
        "model_number_table": MODEL_NUMBER_TABLE,
        "is_connected_after_disconnect": bus.is_connected,
    }, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    except BaseException as exc:  # noqa: BLE001
        print("FATAL:", type(exc).__name__, exc, flush=True)
        rc = 2
    print("EXIT_CODE:", rc, flush=True)
    sys.exit(rc)

PY
```

#### 第二步：投递到 Pi 的 `/tmp`（**需「向 Pi 写文件」授权**）

```bash
run 4.2-scp scp "$S4DIR/stdout/4.2-gripper-regs-$STAMP.py" "$PI:/tmp/4.2-gripper-regs-$STAMP.py"
```

#### 第三步：在 Pi 上跑（与 Host 同环境解释器）

```bash
run 4.2-gripper-regs ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-gripper-regs-$STAMP.py"
```

#### 没有「向 Pi 写文件」授权时的替代（方案需现场确认）

```bash
run 4.2-gripper-regs ssh "$PI" '~/miniconda3/envs/lerobot_alohamini/bin/python -' < "$S4DIR/stdout/4.2-gripper-regs-$STAMP.py"
```

脚本经 stdin 送入，不向 Pi 落盘，因此不需要写盘授权；若现场不允许 stdin 送脚本，改为在 Pi 上手工输入同一脚本。

本步只读、不写扭矩寄存器。

判据（二选一，记录里必须写明走了哪一支）：

| 读数 | 结论 | 现场怎么处理 |
| --- | --- | --- |
| `Min_Position_Limit` / `Max_Position_Limit` 等于 `2021` / `2072` | 固件限位已生效，左夹爪被锁在 51 tick 内，成因在标定值 | 记为「已确认，不处理」。D14 已定本轮不重标定 |
| 读出别的值 | 问题在写盘或写盘之后的覆盖 | 记入待办清单，本窗口不再动标定文件 |

同一轮再读一次 `Torque_Enable` 与 `Operating_Mode`，只记录，不改。

参照读数（2026-10-25 停 Host 后读左臂 ID 1/2/3/6）：`Torque_Enable=1`、`Lock=0`、`Operating_Mode=0`、`Status=0`、`Torque_Limit` 与 `Max_Torque_Limit` 都是 1000、`P`/`I`/`D` 都是 16/0/32，原文在 `stdout/reg_read_1792923639.log`。这组状态**写入者未识别**，所以本步只读、不写扭矩寄存器。

产物：`stdout/4.2-gripper-regs-$STAMP.{cmd,log,exit}` 与脚本副本 `stdout/4.2-gripper-regs-$STAMP.py`。

#### 全 18 电机静态体检（左臂 + 右臂 + 底盘 3 轮 + 升降轴）

上一小节只读左夹爪 ID7。这一小节把同一套只读脚本扩到全部 18 个电机，一轮拿齐静态读数（`Present_Voltage` / `Present_Temperature` / `Status` / 限位 / 偏置），供后续所有「先查电压温度再谈运动」的判据使用。

**寄存器口径：34 项 = `REQUIRED_REGS` 14 项 + `EXTRA_REGS` 20 项**（本节脚本的清单，2026-11-05 现场核实口径）。文档里其它口径的项数一律以这一条为准。

**总线与 ID 分配**（出处是 vendor 源码 + `configs/site.yaml` 的已实测设备路径，**不是现场逐台复核**；见后面的现场确认项）：

| 总线（`PORT`） | 电机名 | ID | model | 归一化 | 出处 |
| --- | --- | --- | --- | --- | --- |
| 左 `/dev/am_arm_follower_left`（→ `ttyACM0`） | `arm_left_shoulder_pan` … `arm_left_gripper` | 1–7 | 1 `sts3250`、2/3 `sts3095`、4/5/6 `sts3250`、7 `sts3250` | 1–6 `range_m100_100`、7 `range_0_100` | vendor `alohamini.py:155-163`（`am-follower-6dof-hd` 档）+ `configs/site.yaml:15` |
| 左（**同一条总线**） | `base_left_wheel` / `base_back_wheel` / `base_right_wheel` | 8 / 9 / 10 | `sts3250` | `range_m100_100` | vendor `alohamini.py:209-215`（`left_bus_motors` 里同时有 `arm_left_*` 与 `base_*`），`model_specs.py:73-79` 的 `base_motor` |
| 左（**同一条总线**） | `lift_axis` | 11 | `sts3095` | `degrees` | 同上，`lift_motor` |
| 右 `/dev/am_arm_follower_right`（→ `ttyACM1`） | `arm_right_*` | 1–7 | 同左臂 | 同左臂 | `configs/site.yaml:16` + 同一份 `am-follower-6dof-hd` 档 |

一句话人话：底盘三个轮子和升降轴**不在独立总线上**，它们挂在左臂那条总线（`left_port`）上——vendor 把 `arm_left_*`、`base_*`、`lift_axis` 放进同一个 `left_bus_motors` 字典，再传给 `FeetechMotorsBus(port=self.config.left_port, ...)`（`alohamini.py:209-222`）。左总线 11 个电机、右总线 7 个，合起来 18 个。

脚本改动只有 `MOTORS` 一段（其余——`REQUIRED_REGS` / `EXTRA_REGS` / 存活探测 / 第二遍 `pass2` / `disconnect(disable_torque=False)`——一个字不改）。两段按总线各跑一次：

左总线（11 个电机）：

```python
PORT = "/dev/am_arm_follower_left"
MOTORS = {
    "arm_left_shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_left_elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_left_wrist_flex": Motor(4, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_wrist_yaw": Motor(5, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_left_gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),
    "base_left_wheel": Motor(8, "sts3250", MotorNormMode.RANGE_M100_100),
    "base_back_wheel": Motor(9, "sts3250", MotorNormMode.RANGE_M100_100),
    "base_right_wheel": Motor(10, "sts3250", MotorNormMode.RANGE_M100_100),
    "lift_axis": Motor(11, "sts3095", MotorNormMode.DEGREES),
}
```

右总线（7 个电机）：

```python
PORT = "/dev/am_arm_follower_right"
MOTORS = {
    "arm_right_shoulder_pan": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_shoulder_lift": Motor(2, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_right_elbow_flex": Motor(3, "sts3095", MotorNormMode.RANGE_M100_100),
    "arm_right_wrist_flex": Motor(4, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_wrist_yaw": Motor(5, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_wrist_roll": Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "arm_right_gripper": Motor(7, "sts3250", MotorNormMode.RANGE_0_100),
}
```

同时把存活探测的**两处**（probe 与波特率重试循环里那两次 `read_one`）都从 `"shoulder_pan"` 改成 `"arm_left_shoulder_pan"`（右总线那份对应改成 `"arm_right_shoulder_pan"`）：电机名改了，探测键必须跟着改，否则第一行就 `KeyError`。

跑法照 §4.2 第一到第三步（Mac 上建脚本 → `scp` 或 stdin 投递 → 用 Host 同环境解释器跑），产物名分开：

投递沿用 §4.2 第二步的两条路径之一（有「向 Pi 写文件」授权就 `scp`，没有就走 stdin）：

```bash
run 4.2-all18-scp scp "$S4DIR/stdout/4.2-all18-left-$STAMP.py" "$PI:/tmp/4.2-all18-left-$STAMP.py"
run 4.2-all18b-scp scp "$S4DIR/stdout/4.2-all18-right-$STAMP.py" "$PI:/tmp/4.2-all18-right-$STAMP.py"
```

```bash
run 4.2-all18-left ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-all18-left-$STAMP.py"
run 4.2-all18-right ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-all18-right-$STAMP.py"
```

判据：

| 判据 | 值 |
| --- | --- |
| 两份日志都以 `EXIT_CODE: 0` 结束 | 必需 |
| 每条总线每个电机都读到 `Model_Number` | 左 11/11、右 7/7；对照 `MODEL_NUMBER_TABLE`（`sts3215=777`、`sts3250=2825`、`sts3095=2569`） |
| 18 个电机的 `Present_Voltage` 全部回读 | 记 min / mean / max 三个数 |
| `Status` 全 0 | 非 0 就把该电机与非 0 值单独记下 |

**满电电压基准的判据**：记录 18 个电机 `Present_Voltage` 的 min / mean / max；**任一 < 10.6 V → 记「需充电后再取基准」**，本窗口不开始任何运动类步骤。10.6 V 是 **2026-10-25 台账的历史观测**（`docs/field-results.md`「扭矩处于开启态」一行记到温度 22–27 ℃、电压 10.6–10.8 V），**不是厂商规格，也不是本机型下限**；充没充电、仓内有没有充电步骤可写，仓库里都没有，所以这一条只给判据不给流程。

现场确认项（到场先填，填之前不要改脚本硬编码）：

| 项 | 怎么确认 | 填什么 |
| --- | --- | --- |
| 左 / 右总线别名的实际指向（复核） | `run 3.7b2-serial-links ssh "$PI" 'ls -l /dev/am_arm_follower_left /dev/am_arm_follower_right'` | 判据：两个别名都存在且分别指向 `ttyACM0` / `ttyACM1`，与 `configs/site.yaml` 的 `left_port` / `right_port` 一致。2026-11-06 实测 `left → ttyACM0`、`right → ttyACM1`（`stdout/3.7b2-serial-links-20261106T062506Z.log`）；换机、重插、换 USB 口后需复核 |
| 左总线上 ID8–11 是否在线 | 本节左总线那份日志里 `base_*` / `lift_axis` 的 `Model_Number` 是否回读到 | 读到 = 与源码一致；读不到 = 记现象原文，暂按「这台的底盘 / 升降接在别处」追查，不改脚本去猜端口 |
| vendor Host 启动输出里的端口 | Host 启动日志原文（§6.1 的启动命令骨架） | 若出现别的串口名，把原文抄进记录 |
| 底盘三轴与升降是否带电 | 本节读到的 `Torque_Enable` | 三个轮子与 `lift_axis` 各自的 `Torque_Enable` 值 |

产物：`stdout/4.2-all18-left-$STAMP.{cmd,log,exit}`、`stdout/4.2-all18-right-$STAMP.{cmd,log,exit}` 与两份脚本副本 `stdout/4.2-all18-*-$STAMP.py`。

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

`4.3d` 用来看清默认值属于哪个 dataclass。实测 `AlohaMiniHostConfig` 与 `AlohaMiniClientConfig` 各有一个 `observation_request_window`（`:110` 与 `:135`），取 Host 那个。

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

`configs/site.yaml` 的注释里写「见 `docs/s4-field-smoke.md` 第 4.3 节」，指的就是本节。

三个数值字段的来路（只记「这些数字怎么来的」，取值以 `configs/site.yaml` 为准）：

| 字段 | 依据（来路） |
| --- | --- |
| `observation_request_window` | `config_alohamini.py:110`（`AlohaMiniHostConfig` 源码默认）；该项无命令行入口 |
| `watchdog_timeout_ms` | `config_alohamini.py:122` |
| `max_loop_freq_hz` | `config_alohamini.py:125` |

另外四个字段的取法：

| 字段 | 取法 |
| --- | --- |
| `robot_model` | 第 4.1 节产物 `$S4DIR/capture/fingerprint-$STAMP.json` 里 `robot_metadata.robot_model` 的值 |
| `left_port` / `right_port` | 第 3.7 节的 `3.7b-serial-pi` 输出（看 `/dev/ttyACM*` 行） |
| `cameras` | 第 3.7 节的 `3.7d-video-pi` 输出，再加实际相机接法 |

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

`robot_model` 两处默认值不一致：本库 `overlay.py:51` 是 `alohamini1`；两个现场脚本的 `DEFAULT_ROBOT_MODEL` 都是 `alohamini2pro`（`benchmarks/alohamini_io/capture_robot_fingerprint.py:90`、`benchmarks/alohamini_io/measure_latency.py:537`）。现场机型是 AlohaMini 2 Pro，而代码 Gate C 要求宿主 metadata 的 `robot_model` 与 `--robot-model` 相等（比对在 `measure_latency.py:2382-2396`）。现场 YAML 必须显式写 `robot_model: alohamini2pro`，不能用模板里的 `alohamini1`，否则配置表达的意思与真机不符。

本节产物：`configs/template.yaml`、`configs/site.yaml`，`stdout/4.3a-host-args-*`、`4.3a2-host-cmdline-*`、`4.3b-host-config-*`、`4.3c-config-args-*`、`4.3d-config-classes-*`、`4.3e-load-check-*`，以及第 4.1、4.1b、4.2 节的产物。

### 4.4 库真机 dry-run（只读观测，零下发）

前置：Host 常驻（§3.4 的 5555 / 5556 在听）；本轮授权含「只读观测」；vendor 快照已在 `/tmp`（取法见 §5.3 的两行 `git clone` + `rev-parse HEAD`，应等于 Pi 运行时的 HEAD `6aee1f81`）。

这条命令验的是本库自己的 `connect(dry_run=True)` 真 client 路径：用厂商 `AlohaMiniClient` 注入本库，读观测、对升降轴下一条命令、断开——**`send_action` 一次都不该发生**。

```bash
run 4.4-robot-dry-run env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src:/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python examples/robot_dry_run.py --host "$PI_HOST"
```

`PYTHONPATH` 两段都必需：第一段是 vendor 快照（主环境与隔离环境都没有 `latest_request_timing` 那版 vendor 代码），第二段是本库 `src/`（本库没装进 vendor 环境，见 `examples/robot_dry_run.py:17-20` 的自述）。在仓库根下跑，`src` 写绝对路径以免依赖 cwd。

在 Pi 本机上跑时省掉 `--host`（脚本默认 `127.0.0.1`）；本手册的 `run` 在 Mac 上跑，所以要显式给 `"$PI_HOST"`。

判据（三件，缺一不算通过）：

| 判据 | 看什么 |
| --- | --- |
| exit 0 | `stdout/4.4-robot-dry-run-$STAMP.exit` 为 `0` |
| 零下发 | stdout 原文 `send_action 调用次数: 0 次发送（dry_run=True，期望 0）`（`examples/robot_dry_run.py:431`） |
| 读到了真观测 | stdout 有 `host=... ：读一帧观测，键 N 个` 与 metadata 行（`schema_version=2`、motor 条目 18） |

**这份脚本没有报告 JSON**：它只往 stdout 打印，`dry_run` 不落成字段，也没有观测帧数计数。「`dry_run` 为真」的现场可取证形态就是上面那句原文 + exit 0，不要把「没有 JSON」记成失败。

相机一句：脚本自身两次取帧都走 `include_cameras=False`（先 `prime_observation_request_window(include_cameras=False)`，再 `get_observation(include_cameras=False)`），但库内 `LiftHandle.height()`（`src/alohamini_cartesian/control/api.py:494`）、`move_by` 内部的 `height()`、以及 `_prime_state_machine()` 的首帧（`control/api.py:845`）共 3 次调用**无参** `get_observation()`，落回厂商默认 `include_cameras=True`——真机分支仍会多发 3 次相机帧请求。这是已知形态（`examples/robot_dry_run.py:51-59` 写明），不是本次新增的缺陷；证据在 Host 侧收到的请求，脚本输出里没有帧数。

预期结果：升降轴那条命令在 priming 阶段的 adapter 门禁被拒（`MotionNotEnabled`），打印拒绝原因后继续走完。**命令被拒属预期，不是失败**（`examples/robot_dry_run.py:44-49`）；失败指的是 exit 非 0，或 `send_action` 计数不为 0。

另一种预期形态：脚本先调 `session.lift.height()`，若反馈失联会先抛 `FeedbackLost`（`examples/robot_dry_run.py:344-346` 自己写明这一点），**这不等于 adapter 门禁拒绝**，按 `FeedbackLost` 记（查 Host 是否在跑、观测是否在涨），不要记成「门禁按预期拒绝」。

产物：`stdout/4.4-robot-dry-run-$STAMP.{cmd,log,exit}`。

### 4.5 Pi 未提交文件调查（只读）

背景一句：「未提交文件」指 Pi 现役目录 `/home/pi5/lerobot_alohamini` 里没进 git 的现场文件（配置、标定、临时脚本）。2026-10-25 的旧部署备份 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025` 是「`ec65318f` 加 6 个未提交改动」，那 6 个文件是否还在，只能到现役目录里查。

对象：`/home/pi5/lerobot_alohamini`（现役目录，不是 legacy 备份）。本节只读，不 checkout、不 stash pop、不 reset。

```bash
run 4.5a-stash ssh "$PI" 'cd ~/lerobot_alohamini && git stash list'
run 4.5b-reflog ssh "$PI" 'cd ~/lerobot_alohamini && git reflog --date=iso | head -20'
run 4.5c-status ssh "$PI" 'cd ~/lerobot_alohamini && git status --porcelain'
```

判据（三支，落到哪支写哪支）：

| 现象 | 结论 | 记录怎么写 |
| --- | --- | --- |
| `git stash list` 里能认出那 6 条现场文件 | 文件还在，只是被 stash 了 | 记 `stash@{n}` 的哈希与条目原文；不 pop、不改工作树 |
| stash 为空，`git reflog` 里有 `checkout` / `reset` 痕迹 | 可能是升级时被切走 | 记那几行 reflog 原文（含时间） |
| stash 为空、reflog 也没有 | **确认丢失** | 记「确认丢失，以 2026-11-05 台账为准」，**不追**；台账数字见 `docs/field-results.md` |

`git status --porcelain` 的输出本身就是第三份证据：与本轮的改动预期逐行对一遍，把非空行原文抄进记录（现役目录不该有我们这轮产生的东西，我们没有向 Pi 写代码）。

产物：`stdout/4.5a-stash-$STAMP.*`、`stdout/4.5b-reflog-$STAMP.*`、`stdout/4.5c-status-$STAMP.*`（各三件套）。

## 5 零运动测量（不动机器人）

三条命令都不构造任何运动指令：`measure --mode rtt` 只采样观测。三条都过 `run`：stdout 与 stderr 合并落到 `$S4DIR/stdout/<步骤号>-$STAMP.log`，退出码落到同名 `.exit`。

### 5.1 部署形态（请求 30 Hz、窗口 3）

```bash
run 5.1-rtt-deploy env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 30 --request-window 3 --samples 500 --csv "$S4DIR/rtt/rtt-deploy-$STAMP.csv"
```

判据：exit 0，`$S4DIR/rtt/rtt-deploy-$STAMP.csv` 写出且有数据行。返回 `3` 是连接问题，按第 12 节处置。

解释器不用 `uv run --no-sync python`：主环境没装 `lerobot`，那样写会以 exit `3`、`No module named 'lerobot'` 失败（2026-11-05 实测）。隔离环境与 `PYTHONPATH` 快照的取法见第 5.3 节的「解释器与 `PYTHONPATH`」小节。

### 5.2 RTT 形态（请求 1000 Hz、窗口 1）

```bash
run 5.2-rtt-net env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 --csv "$S4DIR/rtt/rtt-net-$STAMP.csv"
```

解释器与 `PYTHONPATH` 的来由同 5.1（见第 5.3 节）；`uv run --no-sync python` 在主环境会以 exit `3` 失败，不是代码问题。

判据：`exit 0` 且 CSV 有数据行。

若返回 `9`：该时点这份 vendor 的 client 不发布 `latest_request_timing`，rtt 无法测量。这是设计行为，按第 10 节记录，不当故障、不重试改参数。升级后 HEAD `6aee1f81` 提供该面，实测 exit 0。

在 Pi 上执行时，必须把 `PYTHONPATH` 指向被测代码，否则会 import 到已安装的旧版本。采集脚本的 `default_client_factory()` 直接 `import lerobot`，不经过本库 adapter。不设 `PYTHONPATH` 时有两种表现：指纹计时字段全 `null`，或 `measure --mode rtt` 返回退出码 `9`。两种都看着像代码缺陷，实际原因是跑错了代码。

两条读法必须先定下来：

- `--rate-hz` 低于 200 时，`age_s` 的主体是响应在客户端 socket 里等下一次 poll 的空闲时间，所以它测的是「端到端观测滞后」，不能当网络往返时间读。要测 RTT 必须用 `--rate-hz 1000`。脚本在低于 200 Hz 时自己打警告；打了警告不等于失败，但它封住了这一组数值当 RTT 读的用法。
- `--request-window 1` 是 RTT 形态（无预取等待可减）；`--request-window 3` 是部署形态（控制栈实际用的窗口）。两者不可混为一谈，也不可互相替代。

`--samples` 默认是 `500`，本文照样写出来，便于现场比对命令原文。参考值（2026-10-22）：部署形态组 `achieved_rate_hz=30.1`（请求 30 Hz）；RTT 形态组 `achieved_rate_hz=49.0`（请求 1000 Hz）。实际到达率受 Host 主循环节拍限制。

产物：`rtt/rtt-deploy-$STAMP.csv`、`rtt/rtt-net-$STAMP.csv`，以及 `stdout/5.1-rtt-deploy-*`、`stdout/5.2-rtt-net-*`。

### 5.3 跨机 RTT（PC → Pi 真实网段）

背景一句：2026-10-25 时的两份 rtt CSV 均为 Pi 本机回环（`host=127.0.0.1`），量不到 PC 到 Pi 这一段；2026-11-05 已补跨机实测（结果见 §9 与 `docs/field-results.md`）。跨机这一次必须换解释器：用仓库内的隔离环境 `.venvs/alohamini-lerobot/bin/python`，因为主环境没有 `lerobot`，且按 `docs/ADR.md` D8 主环境要停在 numpy 2.3.x。

前置条件：第 0 节的网络访问授权；隔离环境已建（环境事实见本节末尾）。

命令原文（在仓库根下执行；`$PI_HOST`、`$S4DIR`、`$STAMP` 用第 2 节变量块的定义）：

```bash
# 零运动 RTT 形态：请求 1000 Hz、窗口 1（与 Pi 本机那两次同口径，才可比）
/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py \
  --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 \
  --csv "$S4DIR/rtt/rtt-net-cross-$STAMP.csv"
```

这条命令与 5.2 的唯一差别是解释器：把 `uv run --no-sync python` 换成 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python`，因为主环境没有 `lerobot`。参数口径（`--mode rtt`、`--rate-hz 1000`、`--request-window 1`、`--samples 500`）必须与 Pi 本机那两次一致，否则两组数字不可比。

`--csv` 必须改名成 `rtt-net-cross-$STAMP.csv`：与 5.2 同名会在同一次会话里互相覆盖。

要留痕就把整条命令套进 `run`，步骤号 `5.3-rtt-net-cross`：

```bash
run 5.3-rtt-net-cross /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode rtt --rate-hz 1000 --request-window 1 --samples 500 --csv "$S4DIR/rtt/rtt-net-cross-$STAMP.csv"
```

判据：`exit 0` 且 `rtt/rtt-net-cross-$STAMP.csv` 有数据行。把该 CSV 里 `net_rtt_estimate_s` 的 p50 与 Pi 回环的 `6.176 ms` 并列写进现场记录（`docs/background.md` 记 `net_rtt_estimate_ms` p50 6.176；原始 CSV 是 `rtt/rtt-loopback-merged-20261025.csv`），两者之差就是 PC 到 Pi 这一段网段的开销。返回 `9` 或 `3` 的处置与 5.2 相同。

#### 解释器与 `PYTHONPATH`（2026-11-05 实测）

两条硬事实：

- 主环境不装 `lerobot`，直接 `uv run --no-sync python benchmarks/alohamini_io/measure_latency.py …` 会以 exit `3`、`No module named 'lerobot'` 失败。
- 隔离环境装的是构建出的 vendor wheel，不含 `latest_request_timing`；不把 `PYTHONPATH` 指向 fork 快照时，rtt 会以 exit `9` 结束（实测见 `stdout/5.3-rtt-net-cross-*`；重跑 `5.3-rtt-net-cross-2` 把 `PYTHONPATH` 指向 `6aee1f81` 快照后 exit 0）。

所以本节与 5.1、5.2、7.1 的测量命令都写成 `env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src .venvs/alohamini-lerobot/bin/python …`。快照取法：把只读 fork 克隆到 `/tmp`，不改 fork 本身。

```bash
git clone --branch p1a-merged --quiet /Users/pcoffeep/AgentPlayground/active-projects/lerobot_alohamini /tmp/lerobot-vendor-6aee1f81
git -C /tmp/lerobot-vendor-6aee1f81 rev-parse HEAD
```

第二条应等于 Pi 运行时的 HEAD `6aee1f81`（本次实测输出 `6aee1f81d503be98972d605ed32189221a674607`）。

环境事实：

| 项 | 值 |
| --- | --- |
| 环境路径 | `/Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot`（仓库内 `.venvs/`，已被 `.gitignore` 忽略，不进 `git status`） |
| 解释器 | Python 3.13.13 |
| `lerobot` 版本 | `0.6.1` |
| numpy | `2.2.6`（主环境 2.3.5，两边 site-packages 完全隔离） |
| 整目录体积 | `du -sh` = 881M |

已知限制：

1. 冻结快照：装的是构建出的 wheel，厂商仓库后续改动不会反映进来，要更新得重装。
2. 只能在 PC 侧（macOS arm64）跑。Pi 是 Linux aarch64，需要另一套 wheel，这个环境整体不可搬移。
3. 这套环境只验到 import 与 `--help`，**未做任何 `connect()`**。5.3 是它的第一次真连，输出形态以首次现场为准。
4. 不要把 `lerobot` 装进本仓库的 `.venv`：它的核心依赖钉 `numpy>=2.0.0,<2.3.0`，会把主环境的 numpy 从 2.3.5 降级，冲撞 `docs/ADR.md` D8。

产物：`rtt/rtt-net-cross-$STAMP.csv`、`stdout/5.3-rtt-net-cross-*`。

## 6 量级验证：臂总线电流能不能判读

本节要给出一条前置事实：臂总线电流能不能用来判负载。后面所有依赖电流的判断（第 7.2 节）都以这个结论为准。取数主路径是客户端的 `latest_safety_status["currents_ma"]`；`[Currents]` 这条日志在当前 vendor 的日志级别下不打印（见第 6.1 节）。

先固定三件已核对的事：

- 打印值已经是 mA，不是 raw 刻度：Host 侧按 `int(raw × 6.5)` 截断后打印。
- 电流读取本身正常：同一轮 Host 日志里升降轴 `Present_Current` 读到 `71.5 / 221.0 / 936.0 / 2639.0 mA`（`stdout/v2verify-host-20261025.log`）。
- 局限在臂总线：2026-10-25 整份日志的 `[Currents]` 共 1278 行、11502 个元素（元素分布：`0` 11056 个、`6` 444 个、`13` 2 个），最大值 `13 mA`，等于 2 个 raw 刻度。静止与轻载下只有 0–2 个刻度，运动中的量级见第 6.2 节（2026-11-05 实测）。计数须按整份日志复算，按前缀统计会偏小。

`[Currents]` 数组下标与电机 ID 的映射**没有文档**：禁止按位置推断是哪个电机。要判具体电机的电流，用 sidecar 的 `current_ma` 列（带键名，见第 11 节）。厂商快照里的键叫 `currents_ma`，写进 sidecar 的列叫 `current_ma`，查代码时别把这两个名字互换。

### 6.1 静止电流基线

前置条件：Host 在跑（`3.2-host-procs` 有一行）；机器人上电、手臂不动；启停 Host 在授权范围内（第 0 节的表）。

Host 未运行时按下面这条启动。**2026-11-06 实测：原来的「一次 ssh 前台拉起、等它回话」写法有风险——本地 ssh 会挂住 600 s 不返回，而远端 Host 其实已经起来了**（会话 1：`06:32:51Z` 启动、python PID `2340`，本地 ssh 一直不返回，空转 10 分 40 秒后由上层中止，再在 `06:43:31Z` 远端 `kill 2340`（`kill_exit=0`）收场，复查无 LISTEN、无残留；挂住的根因未定位）。改成三步写法：

```bash
# ① 本地后台跑起 Host 命令：远端照旧 setsid nohup 脱离，本地 ssh 放后台
ssh "$PI" "cd /home/pi5/lerobot_alohamini && PYTHONPATH=/home/pi5/lerobot_alohamini/src setsid nohup /home/pi5/miniconda3/envs/lerobot_alohamini/bin/python -u -m lerobot.robots.alohamini.alohamini_host --robot_model alohamini2pro </dev/null >/tmp/site-host-$STAMP.log 2>&1 & echo pid=\$!" >"$S4DIR/stdout/6.1a-host-start-$STAMP.log" 2>&1 &
LOCAL_SSH_PID=$!

# ② 轮询 5556 监听（判据是端口起来，不是等本地 ssh 返回）：最多等 90 s、每 3 s 探一次
for _ in $(seq 1 30); do
  ssh -o ConnectTimeout=5 "$PI" 'ss -ltn | grep -q ":5556" && echo ready=1' | tee -a "$S4DIR/stdout/6.1a-host-start-$STAMP.log" | grep -q ready=1 && break
  sleep 3
done
run 6.1a-host-pid ssh "$PI" 'date -u +%Y-%m-%dT%H:%M:%SZ; ps -eo pid,args | grep alohamini_host | grep -v "bash -c" | grep -v grep; ss -ltnp | grep -E ":5555|:5556"'

# ③ 端口起来后杀本地 ssh；远端 Host 已 setsid 脱离，不受影响
kill "$LOCAL_SSH_PID" 2>/dev/null
```

判据：`6.1a-host-start-$STAMP.log` 里有 `pid=<数字>` 与 `ready=1`；`6.1a-host-pid-$STAMP.log` 里 `ps` 那行是 `python … -m lerobot.robots.alohamini.alohamini_host`，`ss` 给出 `:5555` 与 `:5556` 各一行 `LISTEN`（2026-11-06 实测原文：`pid=2516`、`ready=1`，`ps` 打出 PID `2517`，两个端口各一行 `LISTEN`）。

**停 Host 用的 PID 要取 `6.1a-host-pid` 里 `ps` 打出的那个，不要用 `echo pid=$!` 的**：2026-11-06 实测 `echo pid=$!` 打 `2516`，实际 python 是 `2517`（`setsid` 自己占掉一个 PID），照 `2516` 去 kill 打不中。

原写法的风险就是上面那次事故：本地 ssh 不返回时没有超时，现场会白等，而远端 Host 已在跑、升降轴已自复位，容易误判成「没起来」再启一次。

启起来后等 30 s，再取基线。

#### 主路径：`latest_safety_status["currents_ma"]`（2026-11-05 起）

用一个小脚本连上客户端的观测面，逐帧读 18 个电机的 `currents_ma` 并汇总。脚本范式是 `stdout/6.1c-obs-idle-$STAMP.py`，只调 `connect()`、`get_observation(include_cameras=False)`、`disconnect()`，不发任何动作。本次参数 5 Hz × 20 s = 100 帧。运行方式与 §4.2 同形（脚本投到 Pi 的 `/tmp`、用 Pi 侧 venv 解释器跑，`scp` 需「向 Pi 写文件」授权）：

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

### 6.2 短运动电流量级

前置条件：6.1 的基线已在盘；第 7.1 节的勾选单已过（至少「授权」与「断电开关在手」两条）；运动授权在手。

跑 2 步单关节步进：

```bash
run 6.2-currents-move env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 2 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-6.2-$STAMP.csv"
```

解释器与 §5.3 的「解释器与 `PYTHONPATH`」小节一致；写成 `uv run --no-sync python` 会以 exit `3`、`No module named 'lerobot'` 失败。

跑完取数走 sidecar 主路径：按 §6.1 的脚本范式（`stdout/6.1c-obs-idle-$STAMP.py`）统计运动窗口内的 `current_ma`，或直接分析本次 tracking 的 sidecar `tracking/tracking-6.2-$STAMP-safety.csv`。要统计三项：运动窗口内 `current_ma` 的最大值、取值分布、`≥20 mA` 的样本数。

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

两点必须一起写进记录：打印值是 mA，不是 raw 刻度；`[Currents]` 的数组下标与电机 ID 没有映射文档，禁止按位置推断是哪个电机。要判具体电机，用 sidecar 的 `current_ma`（带键名，列定义见第 11 节）。

判据另一半：主 CSV 与 sidecar 都写出。产物：`tracking/tracking-6.2-$STAMP.csv` 与其 sidecar、`stdout/6.2-currents-move-*`、`stdout/6.2b-currents-move-*`、`stdout/6.2b-analysis-*`、`stdout/7.2-analysis-*`。

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
| `--delta-deg` | `1.0` | 幅度被硬性钳到 `1.0°`（常量 `MAX_STEP_DELTA_DEG`，`:504`），**符号保留**：`-1.0` 按 `-1.0` 执行、`-2.0` 钳成 `-1.0`；`0` 是用法错误（exit 2）。负值即朝反方向 |
| `--anchor` | `present`（默认；验证误差累积形态时用 `command`） | 两模式只差目标锚在哪：`present` 每步重读实测位置、把目标放在「实测加 `delta_deg`」上，lead 只有一步、不跨步累积；`command` 锚在上一步发出的 target（首步落回实测），lead 跨步累积。指标受影响：`command` 的一步可能补掉跨了几步的 lead，`rise_90_time_s` 提前记满、`overshoot_deg` 变成（行程 − 一步 delta）、`error_deg` 停在累积 lead 上，所以这三个量与 `present` 不能横向比较。CSV 与 sidecar 没有锚点列，只能靠 stdout 那行 `note: anchor=command: …` 区分模式，归档时 stdout 必须与 CSV 一起存。物理风险：`command` 下 Gate E 仍按 `\|delta\| × steps <= 20.0` 判，`--steps 20 --delta-deg 1.0` 合法、lead 可积到 20°，静摩擦一旦破开，整段 lead 可能一次性释放；首跑用 3–5 步 |
| `--steps` | 先 `5`，成功后再说 | `\|delta\| × steps` 必须不超过 20.0°；默认 20 步、每步 1.0° 正好顶在上限；`--delta-deg` 传负值时上限按绝对值算，符号不影响上限 |
| `--control-hz` | `50.0`（默认） | 两条命令最小间隔 1/50 s = 20 ms；必须有限且不小于 1，`inf`/`nan` 被拒 |
| `--settle-tolerance-deg` | `0.05`（本次实测；容差小不是 `rise_90` 有值的充分条件） | 默认 `0.15`。2026-11-05 用 `0.05`（小于 `0.1 × delta`，即小于 `0.1°`）跑三轴，`rise_90_ms` 仍报 `count=0`、`p50=nan`：原先「容差不小于 `0.1 × delta` 时 settle 循环先退、于是 `rise_90_ms` 报 `count=0`」的推论被实测推翻，容差取小也补不上这一列 |
| `--settle-timeout-s` | `2.0` | 默认 `1.0` s，20 ms 命令节奏下每步预算偏紧 |
| `--timeout-s` | `60` | 默认 `30` s 对多步加 settle 偏紧，中途超时会以 exit 1 结束 |

`--delta-deg` 的幅度上限与 stdout 表现：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:504`），`resolve_delta_deg`（`:2118-2126`）的实现是 `min(max(delta, -MAX_STEP_DELTA_DEG), MAX_STEP_DELTA_DEG)`——**钳的是幅度，不钳符号**。请求 `--delta-deg 2.0` 时按 1.0 执行，请求 `-2.0` 时按 -1.0 执行，请求 `-1.0` 时原样执行。report 首部把实际值与请求值都打印出来，原文形如 `delta_deg=1.000 (requested 2.000)`（打印在 `measure_latency.py:3369-3370`）。要凑更大总行程，只能改成「1° × 更多步」，加大 `--delta-deg` 的幅度无效。

横幅里那句 `note: settle tolerance <X> deg >= 0.1 * delta (<Y> deg)`（`:3427-3431`）有一处既有文案瑕疵：标签写的是 `0.1 * delta`，括号里的 `<Y>` 打印的却是 **`delta` 本身**。判读时按 `delta` 本身读那个数，不要把它当成「0.1 × delta 的下限值」。该瑕疵已登记在 `TODO.md`。

负值合法这条与退出码的关系：`--delta-deg 0` 才是用法错误（exit 2，stderr 原文 `--delta-deg must be a finite and non-zero number (got 0.0); negative steps are legal, a zero step commands nothing`，判据在 `:1798`）；`--delta-deg -1` 一路过到 connect，本地实测主环境无 `lerobot` 时以 exit 3 结束，不是 2。

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

`--robot-model alohamini2` 是故意写的错值：宿主报的是 `alohamini2pro`（`configs/site.yaml:12`）。两个值都是 vendor 的合法 SKU（`model_specs.py:61-79`），且两侧的关节键名集合相同（`model_specs.py:35-44`），按代码顺序看 Gate B（键在不在）不会先拒——**2026-11-06 实测推翻这一预判**，理由见本节「实测结果」。

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

达成率口径：指令行程 = Σ（每步 `cmd_deg` − 该步 `baseline_deg`）；实测净行程 = 末步 `measured_deg` − 首步 `baseline_deg`；达成率 = 实测净行程 ÷ 指令行程。`cmd_deg` 是每步的绝对目标角，各步起点不同，分母按逐步差求和，不用「步数 × 1°」。

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

## 8 收尾

### 8.1 停 Host 不等于失力

停 Host 之后手臂仍然带电保持。2026-10-25 停 Host 后读到的寄存器是 `Torque_Enable=1`、`Lock=0`，位置环还在通电，人手推就是顶着位置环。断电才失力。（2026-10-25 那次用的信号未记录；若是 `kill -9` 则与本段相容，若是 `kill -INT` 则与实测冲突，需复核。）

**只有 `SIGINT` 会释放扭矩。** Host 收到 `SIGINT`（`kill -INT`）走 `except KeyboardInterrupt` → `finally` → `robot.disconnect()`，`disconnect()` 调 `left_bus.disconnect(self.config.disable_torque_on_disconnect)`，该字段默认 `True`，即释放扭矩。源码（本地 fork `ec65318f`；Pi 副本 `6aee1f81` 行号差 +33）：`alohamini_host.py:548-556`、`alohamini.py:1251-1253`、`config_alohamini.py:68`。现场：2026-11-06 实测 `kill -INT` 之后左总线两轴（ID1 / ID6）`Torque_Enable=0`（`stdout/rt-regs-verdict-20261106T083449Z.md` 第 1.2 节），Host 日志末两行 `Keyboard interrupt received. Exiting...` / `Shutting down AlohaMini Host.`（`stdout/rt-regs-host-verify-20261106T083449Z.log` 末两行）。`kill -9` 与 `SIGTERM` 不给进程收尾机会，扭矩保持带电：2026-11-06 实测 `kill -9` 后左总线 11 只电机 `Torque_Enable` 全 `1`（`stdout/10.5-m1-regs-final-20261106T073052Z.log`，11 个 `Torque_Enable = 1` 行）。`SIGTERM` 那一支本轮没有单独读数，是按「不给收尾机会」推的，**未验证**。

另一条是**外部来源、非本轮实测**：写 `Goal_Position` 时固件会自置 `Torque_Enable=1`（厂商手册与社区帖两处，原文不在证据目录）。因此「读到的 `Torque_Enable=1`」本身不足以断定是谁写的。

所以收工若要断电，先把手臂放到低位再断电，避开断电瞬间的下落。这条只约束断电时序，不改变任何一条退出码判据。

若本次出现过 exit 8（hold 被拒、停命令没到），按「机器可能仍在动」处置：先断电，再排查。

### 8.2 Host 停启留痕

- 停 Host 用第 3.2 节 `3.2-host-procs` 取到的 PID 执行 `kill`。2026-11-05 实测的命令原文与判据：

```bash
run 8.2a-host-stop ssh "$PI" 'date -u +%Y-%m-%dT%H:%M:%SZ; kill <PID>; echo kill_exit=$?'
```

本次 `<PID>` 为 `2096`，`.log` 为 `2026-11-05T07:08:54Z` 与 `kill_exit=0`（`stdout/8.2a-host-stop-*`）。
- 停后 5 秒内验证进程与端口都已消失：

```bash
run 8.2b-host-verify ssh "$PI" 'pgrep -af "alohamini_host|lerobot"; ss -ltnp | grep -E ":5555|:5556"'
```

判据：`pgrep` 无输出、`ss` 那一段无 `LISTEN`（`stdout/8.2b-host-verify-*`）。`pgrep` 会匹配到 ssh 远端那条 `bash -c …` 自己（本次多出 `pid=3112` 一行），判据按「除该行以外无输出」读。
- 重启 Host 按第 6.1 节的命令骨架，日志路径一并写进同一份记录。
- 记下停机与重启的时间（UTC），供日志对齐。

判据：一份记录里有 PID、停命令原文、重启命令原文、两个 UTC 时间。产物：`stdout/8.1-host-stop-*`、`stdout/8.2a-host-stop-*`、`stdout/8.2b-host-verify-*`。

停 Host 不等于失力：手臂仍带电保持，见第 8.1 节。2026-11-05 的事实是 Host 停后机器人**未断电**——现场没有远程电源通道，需人工先把手臂放到低位再断电，断电仍待人工。

### 8.3 产物归档与交回

- `$S4DIR` 就在本机证据目录下（`~/pi-workspace/alohamini-cartesian-teleop/evidence/`），不需要回传。
- 若某步在 Pi 上生成了文件（寄存器脚本副本、Host 日志），取回方式需现场确认：`scp` 需要网络授权。替代做法是在 Pi 上把原文 `cat` 出来，落进 `stdout/*.log`。
- 交回前按第 13 节逐条勾选，把未完成项与「需现场确认」的条目列给调度方。数字入 `docs/field-results.md` 需另一次任务授权，本手册不写台账。

## 9 已发现问题与已定方案

一条问题一行，写的是「到现场怎么处理」；发现经过不放这张表。

| 问题 | 现场怎么处理 | 判据 | 产物 |
| --- | --- | --- | --- |
| P0 曾广播 v1（已关闭） | 只核对，不回退、不重跑 v1 拒绝路径 | `capture` exit 0 且 `robot_metadata.schema_version == 2` | `capture/fingerprint-*.json` |
| Gate F 第二段不可达 | 不制造记录，只记「未触发」；禁止改标定文件造记录 | 无拒绝记录即为正确结果 | `stdout/4.1b-probe-*`（历史同类产物 `stdout/5.1-probe21-*`） |
| 肩部两轴不跟随 | 试动后取 sidecar 与 `[HOST TRACKING]`（后者因日志级别压制不打，改用 sidecar，见第 6.1 节）：判据三项里 `requested == accepted` 零例外、运动帧 `target_source` 全 `command`，结论落到「① 目标被接受，停滞在伺服或机械侧」 | 达成率 ≥50% 且该步步末残余 ≤2 tick（0.1758°）；若用 `settled=True` 口径，容差必须 ≥1 LSB（0.0879°）（理由见 `docs/acceptance-baseline.md` V01 行）。若两轴仍不跟随，按 §10.5 的 teleop oracle → M1 → M2 定性，结论落到硬件不可修时按 `docs/acceptance-baseline.md` §2.1 记「受阻」 | sidecar CSV + `stdout/7.2-*` |
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

## 10 退出码对照表

**一码两义**：`6` 在 `capture` 里表示宿主 metadata 不可用、主动不写盘；在 `measure` 里表示运动门禁拒绝、零命令。现场拿到 6，先看是哪个脚本。

| 脚本 | 码 | 含义 |
| --- | --- | --- |
| 两者 | 2 | 参数解析失败，一律 `2`，不经过任何 `EXIT_*` 常量；触发点全部列在表下「退出码 2 的全部触发点」小节 |
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

`capture_robot_fingerprint.py` 不产生 `9`：缺 `latest_request_timing` 时它继续采集并把计时字段记 `null`。

### 退出码 2 的全部触发点

参数解析失败，一律 `2`，不经过任何 `EXIT_*` 常量；所有判定都在打开任何 socket 与调用 `client_factory(...)` 之前完成。`measure`（rtt 与 tracking 共用同一个 `parse_args`，rtt 也查这些旋钮）：argparse 自身产生的用法错误 —— 未知选项、缺少 `--host`、`--mode` 不在 `rtt`/`tracking`、`--anchor` 不在 `{present,command}`（stderr 原文 `measure_latency.py: error: argument --anchor: invalid choice: 'bogus' (choose from present, command)`，`--mode rtt` 接受该参数并忽略）、任一 `type=` 转换失败（`--port` / `--samples` / `--request-window` / `--steps` 非整数，或任一 `type=float` 选项不是可解析的数值文本；`--csv` 的 `type=Path` 对命令行字符串不会抛，不在此列）；整数下界 3 项 —— `--samples < 1`、`--request-window < 1`、`--steps <= 0`；「有限且带符号」同查 7 项 —— `--rate-hz`（有限且 `> 0`）、`--control-hz`（有限且 `>= 1`）、`--host-loop-hz`（有限且 `> 0`）、`--timeout-s`（有限且 `> 0`）、`--settle-timeout-s`（有限且 `> 0`）、`--settle-tolerance-deg`（有限且 `>= 0`，`0` 合法即最严格设置）、`--delta-deg`（有限且**非零**，`0` 与 `-0.0` 都落 `2`，负值合法；判据 `measure_latency.py:1798`），非有限值（`nan` / `inf`，含 `float()` 后变成 `inf` 的 `1e400`）与不满足符号条件的值都落 `2`；交叉校验 1 项 —— `--settle-tolerance-deg >= abs(resolve_delta_deg(args))`（与钳到 1.0° 幅度上限之后的**有效 delta 的绝对值**比较，`measure_latency.py:1812-1817`；实测 stderr 原文 `must be below the effective --delta-deg magnitude`）。`capture` 的数值校验只有 2 项 —— `--samples < 1`、`--timeout-s` 有限且 `> 0`（不查 `--rate-hz` / `--control-hz` 之类），另加同一形状的 argparse 用法错误（未知选项、缺少 `--host`、`--port` 非整数）。以上为 `2` 的全部触发点。幅度钳位本身不产生 `2`：超过 `1.0°` 的 `--delta-deg` 由 `resolve_delta_deg` 静默钳（`measure_latency.py:1784-1793` 的注释写明这是有意的），只有 `0` 才拒。合法预算在运行中到期不走 `2`：`capture` 落 `4`，`measure` 的 rtt 落 `7`、tracking 落 `1`

### 退出码 `9` 的三种形态

| 形态 | 何时触发 | 是否发过请求 |
| --- | --- | --- |
| 属性不存在 | `connect()` 返回后、窗口预热之前 | 没发过 |
| 属性存在但从未填充（整轮是空 dict，`observation_sequence` 不增长） | 窗口预热之后，采样循环放弃那一刻才判，要等满 `--timeout-s` | 发过 1 次 state-only 预热，并至少观测 1 帧 |
| 半填充（缺 `token` / `sent_monotonic_s` / `received_monotonic_s` / `age_s` 任一） | 写行之前逐帧复查，`observation_sequence` 每次增长都查一次 | 已发过请求，并在该帧拒绝 |

三种都不写 CSV，连表头都不写。stderr 分得开：第一种是 `it publishes no latest_request_timing`，后两种是 `it publishes latest_request_timing but never fills it completely`，并跟一句 `missing key(s): …` 点名缺的键。只看退出码分不出这三种，要读 stderr。

退出码同时写在每步的 `$S4DIR/stdout/<步骤号>-$STAMP.exit` 里，查表前先看这个文件。

## 10.5 肩部定性专项（M0 已离线复算并立台账；teleop oracle / M1 / M2 走法已写死，待有人到场）

本节接第 7.2 节：那里已把停滞定位到「不在 PC→Host 值传递段」，Host→总线写入段与伺服 / 机械侧仍无观测。本节是要有人到机器人跟前才能做的跟进动作；2026-11-05 远程会话（无人在机器人现场）只完成了 M0。

复现遥操作那种误差累积形态用 `--anchor command`，这也是现场验证肩部静摩擦能否被累积 lead 破开的用法。§7.1 参数表给的默认值是 `present`（每步重锚实测、lead 只有一步），两者构成对照。首跑用 3–5 步，且 `command` 模式下的 `rise_90_time_s` / `overshoot_deg` / `error_deg` 不与 `present` 横向比较。

### 背景：M0 已离线排除换算链

M0 已离线排除换算链（1° = 11 tick；台账与数字见 `docs/field-results.md` 2026-11-05 节的 M0 行，原始报告 `stdout/M0-tick-recompute-20261105.txt`）；本节阈值按 11 tick/度 取。

本节数字除 M0 两项（`1° = 11.375 tick`、腕部正对照 10/10，属离线复算）外，均出自 `docs/field-results.md` 的 2026-11-05 节。

### teleop oracle（有人最先做，成本最低）

做法：用厂商遥操作（leader 主臂，即手持的遥操作主臂）动 `arm_left_shoulder_pan` 与 `arm_left_shoulder_lift`，全程不经过本库。

启动方式：厂商遥操作的入口与命令仓库里查得到，写死在下面。**时序关键是 Host 保持运行**——vendor 文档原文 `Run these on the PC / DGX client after the host is running.`（`docs/alohamini/commands.md:263`），遥操作是 PC 侧经 ZMQ 连 Host 的远端客户端（`examples/alohamini/teleoperate_bi.py:96-119` 建 `AlohaMiniClientConfig(remote_ip=…)` + `AlohaMiniClient`），它碰的是**主臂**串口，不抢从臂总线。所以**不要停 Host**。

在 PC（Mac）另开一个终端，按 vendor 文档的原样命令跑（`docs/alohamini/commands.md:267-276`）：

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

- `--robot.robot_model` 必须与宿主实报值一致（本机 `configs/site.yaml:12` 是 `alohamini2pro`）；写错会撞厂商客户端自己的模型校验（`alohamini_client.py:417-420` 抛 `ValueError: Host robot_model does not match the client configuration`），见第 7.1b 节的同一条机理。
- `--teleop.arm_profile am-leader-6dof` 是 6 自由度主臂档；只支持 `so-arm-5dof` / `am-leader-6dof` 两个取值（`examples/alohamini/teleoperate_bi.py:68-79`）。
- 主臂串口默认值按平台取（`teleoperate_bi.py:56-66`）：macOS 是 `/dev/tty.usbmodem*`、Linux 是 `/dev/am_arm_leader_left` / `/dev/am_arm_leader_right`。现场用 `ls /dev/tty.usbmodem*`（或对应平台的 `/dev/*leader*`）确认后，用 `--teleop.left_port` / `--teleop.right_port` 显式指定，不要依赖默认值。

冲突与失败：若遥操作连不上 Host、或报端口被占用，把 stderr 原文记下来，先查 Host 是否真的在跑（§3.4 的 5555 / 5556 `LISTEN`）与 `remote_ip` 是否写对；仍不通就记「未测」，**不要改 vendor 代码、不要猜命令**。

物理前提（人工确认项）：**主臂手柄（leader）需在现场**、且需先按 vendor 流程标定过（`docs/alohamini/commands.md:288-289` 说明 Pi 侧的 `alohamini_calibrate` 标的是从臂、不是 Mac 侧主臂）。手柄不在场，整条 teleop 不可执行，直接跳到 M1。

判据：位移 **> 20 tick**（≈ 1.76°）算「能动」。读数取动前 / 动后各一次 `Present_Position`，只读，范式同第 4.2 节。

二分结论：

| teleop 读数 | 结论 | 下一步 |
| --- | --- | --- |
| 能动（> 20 tick） | 问题在本库的命令语义 | 继续 M1 / M2 |
| 不能动（≤ 20 tick） | 这两轴的伺服 / 机械侧本身不动，**本库路径完全脱身** | 不再往本库代码里找，转伺服 / 机械侧 |

留痕：`stdout/10.5-teleop-$STAMP.{cmd,log,exit}` 三件套 + 一段读数记录（动前 / 动后的 `Present_Position`）。

### M1（有人在场）：`--steps 10` 整轮 10° + `kill -9` 后读寄存器

硬前置（先满足再动手）：**把手臂放到低位，或有人承托**。`kill -9` 之后扭矩保持，但**没有看门狗**——Host 死了不会再兜住下垂，手臂可能重力下垂。风险同第 8.1 节。

步骤：

1. 发一条整轮命令：`--steps 10` + `--delta-deg 1.0`，即 10 步 × 1°，走的是第 7.1 节已跑通的连续路径。关节取 `arm_left_shoulder_pan` 或 `arm_left_shoulder_lift`。**不要试图用 `--delta-deg 10` 一次下发 10°**：常量 `MAX_STEP_DELTA_DEG = 1.0`（`benchmarks/alohamini_io/measure_latency.py:504`）把幅度钳回 1.0°，`--delta-deg 10` 只会按 1.0° 执行。

   ```bash
   run 10.5-m1 env PYTHONPATH=/tmp/lerobot-vendor-6aee1f81/src /Users/pcoffeep/pi-workspace/alohamini-cartesian-lib/.venvs/alohamini-lerobot/bin/python benchmarks/alohamini_io/measure_latency.py --host "$PI_HOST" --mode tracking --enable-motion --joint arm_left_shoulder_pan.pos --delta-deg 1.0 --steps 10 --control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 --csv "$S4DIR/tracking/tracking-10.5-m1-$STAMP.csv"
   ```

2. 另开一个终端，在命令起跑后按固定墙钟偏移强杀 Host：

   ```bash
   run 10.5-m1-kill ssh "$PI" 'sleep 3; kill -9 <PID>; echo kill_exit=$?'
   ```

   `<PID>` 取第 3.2 节 `3.2-host-procs` 读到的 Host 进程号。为什么用固定墙钟而不是「等 `.log` 里出现某一行」：`--mode tracking` 在跑动期间不打印逐步进度（`benchmarks/alohamini_io/measure_latency.py` 全文无 `flush=True`，跑动期只有 `:3418-3421` 的横幅，明细要等 `:3555` 的报告），而第 2 节的 `run` 把 stdout 重定向进 `.log` 并在命令返回后才 `cat`，所以 `.log` 不是活的监视面。时间预算：`--steps 10`、每步 settle 上限 `2.0 s`，整轮最长约 `20 s`，`sleep 3` 时一定还在跑。若现场改小了 `--settle-timeout-s`，把 `sleep` 按同一比例改小，保持「不超过整轮预算的 1/3」。

3. 立刻读寄存器（Host 已死，串口空闲），读法沿用第 4.2 节的只读脚本范式（同一套 `run` 包装、同一套脚本副本落 `$S4DIR/stdout/` 的纪律）。

   ```bash
   run 10.5-m1-regs ssh "$PI" "~/miniconda3/envs/lerobot_alohamini/bin/python /tmp/4.2-gripper-regs-$STAMP.py"
   ```

   这份脚本的 `MOTORS` 含 ID1 / ID2 / ID3 / ID6 / ID7，六个要读的寄存器都在其 `REQUIRED_REGS` + `EXTRA_REGS` 里；若 §4.2 当时投的是 18 电机版，用 `/tmp/4.2-all18-left-$STAMP.py`。

**禁用 `SIGTERM`，只用 `kill -9`**：Host 干净退出会走 `disable_torque`（失力），之后读到的 `Torque_Enable` 是退出后的状态，不是运动中的状态——读数是假的。`kill -9` 不给进程处理机会，扭矩保持，读到的才是运动中的状态。

要读的六个寄存器（左总线 ID1 / ID2 / ID6 各一遍）：`Torque_Enable`、`Goal_Position`、`Present_Position`、`Status`、`Present_Temperature`、`Present_Voltage`。

阈值（以下这一组数是默认 `--anchor present` 的口径）：`--steps 10` 整轮 10° 时，每步目标是「当前反馈 + 1°」（约 `measure_latency.py:3149-3168` 每步都重读 `baseline_units` 再 `+delta_deg`），前一步没走到位的部分不带进下一步，因此**最后一步的 `Goal_Position − Present_Position` ≈ 11 tick**（1° ÷ 0.0879°/tick = 11.375，M0 取整 11 tick），**不是累积 110 tick**。`--anchor command` 是另一条下发路径：目标锚在上一步发出的 target 上、lead 跨步累积，同一个读数变成 `steps × \|delta\|` 量级（`--steps 10 --delta-deg 1.0` 时约 110 tick）。读寄存器前先确认这次 run 用的哪个锚点：stdout 有 `note: anchor=command` 那行就是 `command`，没有这行就是默认 `present`（这行只在 `command` 模式打）。

**看门狗会改写 `Goal_Position`，让分支③ 假命中。** Host 在最后一条合法命令之后超过 `watchdog_timeout_ms`（默认 1000 ms）仍没收到新命令时，会把 `last_observation` 的 `*.pos` 当保持位重新下发，并把 `target_source` 置 `watchdog`。此时读到的 `Goal_Position − Present_Position` ≈ 1 tick，远小于 11 tick，**分支③「写没到总线」会假命中**。机理依据：判定式与保持位构造（`hold_action` 取 `last_observation` 的 `.pos`）在 `alohamini_host.py:348-362`、`:357` 置 `target_source`；默认超时 `1000` 在 `config_alohamini.py:122`（本地 fork `ec65318f` 只读核对；Pi 副本 `6aee1f81` 行号差 +33）。现场告警原文两行 `WARNING:root:Host command watchdog expired; stopping motion` 在 `stdout/site-host-20261106T074049Z.log` 第 16–17 行，**属 07:40:49Z 起的那个 Host 会话**（装的是 07:40 腕部轮与 07:46 pan 轮），不是「客户端先死」的专用证据。

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

   判据：最后那条 `awk` 空输出 → 过了这一关；打印出任何 `sample_sequence` → 说明该客户端当时还在采样、却已 ≥1 s 没有发出被接受的命令（那几帧的 `target_source` 是 Host 的保持位改写），该轮读数**不能**用于分支③，改判「未落到分支」并重跑。两条实现要点：sidecar 是 CRLF 行尾，`tr -d '\r'` 不能省（不删 `\r` 时 `$9` 永远不等于 `watchdog`，检查会静默「通过」）；`$9` 是 `target_source`、`$2` 是 `sample_sequence`（列义见第 11 节）。

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

留痕：`stdout/10.5-m2-$STAMP.{cmd,log,exit}`、`stdout/10.5-m2-kill-$STAMP.*`、`stdout/10.5-m2-regs-$STAMP.log` + 一份主 CSV（沿用 `--mode tracking` 的 tracking CSV 命名，见第 11 节）。

### 产物名与留痕要求

- 所有产物落 `$S4DIR/stdout/` 与 `$S4DIR/tracking/`，命名 `10.5-<子项>-<STAMP>.<ext>`（子项取 `teleop` / `m1` / `m1-kill` / `m1-regs` / `m2` / `m2-kill` / `m2-regs`）。
- 每步必须有 `.cmd` / `.log` / `.exit` 三件套，写法同第 2 节的 `run` 包装。
- 每条结论必须落到本节的分支之一（teleop 的二分、M1 的三分支、M2 的三种形态），并把对应的寄存器原文或 CSV 行号写进记录；没落到分支的记录不算结论。
- 现场若拿到与本文不符的输出形态，按实际输出修正并回写到本小节（沿用本文档原有的回写约定）。

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

**9 列**（`SAFETY_CSV_FIELDS`，`:626`，顺序固定）：

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

## 12 失败与中止处置

| 现象 | 判据 | 处置 |
| --- | --- | --- |
| exit 6（capture） | stderr 含 `schema_version=` | 记录原文，核对第 1 节 P0，不要重试 |
| exit 6（measure） | 拒绝发生在首帧后 | 记录是哪道代码 Gate（第 7.1 节表），写进 `"$S4DIR/stdout/"` |
| exit 8 | 报告里 hold 被拒 | 先断电，再排查；按「仍在动」处置，见第 8.1 节 |
| exit 1 加反馈失联 | 连续 3 次序号不涨 | 检查网络与 Host 负载 |
| 网络中断 | `ping` 丢包 | 重跑该步，不复用旧产物 |

停 Host 不等于失力、断电时序与 exit 8 的处置，见第 8.1 节。

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
- [x] 第 7.1 节若执行：每步的 `error_deg`、`overshoot_deg`、`settled` 齐全；`rise_90_time_s` 为空时写明原因；收尾 `hold_issued` 在报告里（2026-11-06 到场窗口已满足：10 步的 `error_deg` / `overshoot_deg` / `settled` 与 `hold_issued=yes` 在 `stdout/10.5-m1r3-pan-20261106T074049Z.log:5`、`:11`，逐行表在 `tracking/m1r3-pan-20261106T074620Z.csv`；`rise_90_time_s` 为空的原因「残余 > 0.1 × 命令量、没有帧越过 90% 线」由 `stdout/slow-vs-stall-judge-20261106T090635Z.txt` 第 5.4 节逐轴核成立（腕收尾残余 0.1209° 对 0.100°、`pan` 0.8242° 对 0.100°），数字见 `docs/field-results.md` 2026-11-06 节。探针本身未改、以及「收敛之后会不会有 90% 交叉」仍未观测，这两件记在 `TODO.md` 的 `rise_90_time_s` 工程债行）
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
