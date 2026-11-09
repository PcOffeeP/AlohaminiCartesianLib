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

判据：`6.1a-host-start-$STAMP.log` 里有 `pid=<数字>` 与 `ready=1`；`6.1a-host-pid-$STAMP.log` 里 `ps` 那行是 `python … -m lerobot.robots.alohamini.alohamini_host`，`ss` 给出 `5555` 与 `5556` 两个端口各一行 `LISTEN`（2026-11-06 实测原文：`pid=2516`、`ready=1`，`ps` 打出 PID `2517`，两个端口各一行 `LISTEN`）。

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

## 8 收尾

### 8.1 停 Host 不等于失力

停 Host 之后手臂仍然带电保持。2026-10-25 停 Host 后读到的寄存器是 `Torque_Enable=1`、`Lock=0`，位置环还在通电，人手推就是顶着位置环。断电才失力。（2026-10-25 那次用的信号未记录；若是 `kill -9` 则与本段相容，若是 `kill -INT` 则与实测冲突，需复核。）

**只有 `SIGINT` 会释放扭矩。** Host 收到 `SIGINT`（`kill -INT`）走 `except KeyboardInterrupt` → `finally` → `robot.disconnect()`，`disconnect()` 调 `left_bus.disconnect(self.config.disable_torque_on_disconnect)`，该字段默认 `True`，即释放扭矩。源码（行号按提交取，逐条两支）：`alohamini_host.py:548-556`（`ec65318f`；`6aee1f81` 为 `:612-620`）、`alohamini.py:1251-1253`（`ec65318f`；`6aee1f81` 为 `:1284-1286`）、`config_alohamini.py:56`（两支该行相同，偏移 0）。现场：2026-11-06 实测 `kill -INT` 之后左总线两轴（ID1 / ID6）`Torque_Enable=0`（`stdout/rt-regs-verdict-20261106T083449Z.md` 第 1.2 节），Host 日志末两行 `Keyboard interrupt received. Exiting...` / `Shutting down AlohaMini Host.`（`stdout/rt-regs-host-verify-20261106T083449Z.log` 末两行）。`kill -9` 与 `SIGTERM` 不给进程收尾机会，扭矩保持带电：2026-11-06 实测 `kill -9` 后左总线 11 只电机 `Torque_Enable` 全 `1`（`stdout/10.5-m1-regs-final-20261106T073052Z.log`，11 个 `Torque_Enable = 1` 行）。`SIGTERM` 那一支本轮没有单独读数，是按「不给收尾机会」推的，**未验证**。

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

