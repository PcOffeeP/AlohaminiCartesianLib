# 5 收尾与验收（原 §8.2、§8.3、§13）

本文件管什么：收尾——Host 停启留痕、产物归档与交回，以及交回前要逐条过的验收判据清单。

## 8 收尾

<a id="sec-8-2"></a>
### 8.2 Host 停启留痕

- 停 Host 用 [§3.2](1-prepare.md#sec-3) 的 `3.2-host-procs` 取到的 PID 执行 `kill`。2026-11-05 实测的命令原文与判据：

```bash
run 8.2a-host-stop ssh "$PI" 'date -u +%Y-%m-%dT%H:%M:%SZ; kill <PID>; echo kill_exit=$?'
```

本次 `<PID>` 为 `2096`，`.log` 为 `2026-11-05T07:08:54Z` 与 `kill_exit=0`（`stdout/8.2a-host-stop-*`）。
- 停后 5 秒内验证进程与端口都已消失：

```bash
run 8.2b-host-verify ssh "$PI" 'pgrep -af "alohamini_host|lerobot"; ss -ltnp | grep -E ":5555|:5556"'
```

判据：`pgrep` 无输出、`ss` 那一段无 `LISTEN`（`stdout/8.2b-host-verify-*`）。`pgrep` 会匹配到 ssh 远端那条 `bash -c …` 自己（本次多出 `pid=3112` 一行），判据按「除该行以外无输出」读。
- 重启 Host 按 [§6.1](2-host.md#host-start) 的命令骨架，日志路径一并写进同一份记录。
- 记下停机与重启的时间（UTC），供日志对齐。

判据：一份记录里有 PID、停命令原文、重启命令原文、两个 UTC 时间。产物：`stdout/8.1-host-stop-*`、`stdout/8.2a-host-stop-*`、`stdout/8.2b-host-verify-*`。

停 Host 不等于失力：手臂仍带电保持，见 [§8.1](2-host.md#sec-8-1)。2026-11-05 的事实是 Host 停后机器人**未断电**——现场没有远程电源通道，需人工先把手臂放到低位再断电，断电仍待人工。

<a id="sec-8-3"></a>
### 8.3 产物归档与交回

- `$S4DIR` 就在本机证据目录下（`~/pi-workspace/alohamini-cartesian-teleop/evidence/`），不需要回传。
- 若某步在 Pi 上生成了文件（寄存器脚本副本、Host 日志），取回方式需现场确认：`scp` 需要网络授权。替代做法是在 Pi 上把原文 `cat` 出来，落进 `stdout/*.log`。
- 交回前按 [§13](#sec-13) 逐条勾选，把未完成项与「需现场确认」的条目列给调度方。数字入 `docs/field-results.md` 需另一次任务授权，本手册不写台账。

<a id="sec-13"></a>
### 验收判据清单

逐条判据原文与历史勾选状态见 [completed-work.md 附录A（原 §13 验收标准，冻结存档）](../completed-work.md#appendix-a)。

- [§13 第 1 条](../completed-work.md#sec-13-1)
- [§13 第 2 条](../completed-work.md#sec-13-2)
- [§13 第 3 条](../completed-work.md#sec-13-3)
- [§13 第 4 条](../completed-work.md#sec-13-4)
- [§13 第 5 条](../completed-work.md#sec-13-5)
- [§13 第 6 条](../completed-work.md#sec-13-6)
- [§13 第 7 条](../completed-work.md#sec-13-7)
- [§13 第 8 条](../completed-work.md#sec-13-8)
- [§13 第 9 条](../completed-work.md#sec-13-9)
- [§13 第 10 条](../completed-work.md#sec-13-10)
- [§13 第 11 条](../completed-work.md#sec-13-11)
- [§13 第 12 条](../completed-work.md#sec-13-12)
- [§13 第 13 条](../completed-work.md#sec-13-13)
- [§13 第 14 条](../completed-work.md#sec-13-14)
- [§13 第 15 条](../completed-work.md#sec-13-15)
- [§13 第 16 条](../completed-work.md#sec-13-16)
- [§13 第 17 条](../completed-work.md#sec-13-17)
- [§13 第 18 条](../completed-work.md#sec-13-18)
- [§13 第 19 条](../completed-work.md#sec-13-19)

