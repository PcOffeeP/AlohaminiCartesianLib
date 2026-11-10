# 2 Host 起停与扭矩纪律（原 §6.1 程序部分、§8.1）

本文件管什么：Host 怎么起（静止电流基线的三步法）、怎么停，以及「停 Host 不等于失力」这条扭矩纪律。

<a id="host-start"></a>
### 6.1 静止电流基线

前置条件：Host 在跑（`3.2-host-procs` 有一行）；机器人上电、手臂不动；启停 Host 在授权范围内（[§0](1-prepare.md#sec-0) 的表）。

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

<a id="sec-8-1"></a>
### 8.1 停 Host 不等于失力

停 Host 之后手臂仍然带电保持。2026-10-25 停 Host 后读到的寄存器是 `Torque_Enable=1`、`Lock=0`，位置环还在通电，人手推就是顶着位置环。断电才失力。（2026-10-25 那次用的信号未记录；若是 `kill -9` 则与本段相容，若是 `kill -INT` 则与实测冲突，需复核。）

**只有 `SIGINT` 会释放扭矩。** Host 收到 `SIGINT`（`kill -INT`）走 `except KeyboardInterrupt` → `finally` → `robot.disconnect()`，`disconnect()` 调 `left_bus.disconnect(self.config.disable_torque_on_disconnect)`，该字段默认 `True`，即释放扭矩。源码（行号按提交取，逐条两支）：`alohamini_host.py:548-556`（`ec65318f`；`6aee1f81` 为 `:612-620`）、`alohamini.py:1251-1253`（`ec65318f`；`6aee1f81` 为 `:1284-1286`）、`config_alohamini.py:56`（两支该行相同，偏移 0）。现场：2026-11-06 实测 `kill -INT` 之后左总线两轴（ID1 / ID6）`Torque_Enable=0`（`stdout/rt-regs-verdict-20261106T083449Z.md` 第 1.2 节），Host 日志末两行 `Keyboard interrupt received. Exiting...` / `Shutting down AlohaMini Host.`（`stdout/rt-regs-host-verify-20261106T083449Z.log` 末两行）。`kill -9` 与 `SIGTERM` 不给进程收尾机会，扭矩保持带电：2026-11-06 实测 `kill -9` 后左总线 11 只电机 `Torque_Enable` 全 `1`（`stdout/10.5-m1-regs-final-20261106T073052Z.log`，11 个 `Torque_Enable = 1` 行）。`SIGTERM` 那一支本轮没有单独读数，是按「不给收尾机会」推的，**未验证**。

另一条是**外部来源、非本轮实测**：写 `Goal_Position` 时固件会自置 `Torque_Enable=1`（厂商手册与社区帖两处，原文不在证据目录）。因此「读到的 `Torque_Enable=1`」本身不足以断定是谁写的。

所以收工若要断电，先把手臂放到低位再断电，避开断电瞬间的下落。这条只约束断电时序，不改变任何一条退出码判据。

若本次出现过 exit 8（hold 被拒、停命令没到），按「机器可能仍在动」处置：先断电，再排查。

