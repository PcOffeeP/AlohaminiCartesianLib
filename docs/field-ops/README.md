# S4 现场执行手册（`docs/field-ops/`）

历史曾用名：原为单文件 `docs/s4-field-smoke.md`，拆分后节号与内容未改。本轮起按读者旅程重组：`runbook/`（按现场顺序的五份执行手册）+ `reference/`（查表查数）；旧文件名对照见 `reference/mapping.md`。

## 一页现状

1. 这是 S4 现场执行手册，按「读者旅程」组织：一次现场窗口从到场核对做到收尾，按顺序走 `runbook/` 的五份。
2. 现场顺序 = [runbook/1-prepare.md](runbook/1-prepare.md) → [2-host.md](runbook/2-host.md) → [3-health.md](runbook/3-health.md) → [4-experiments.md](runbook/4-experiments.md) → [5-wrapup.md](runbook/5-wrapup.md)。
3. 查表与查数去 [reference/](reference/)；做过什么、下一轮做什么去 [completed-work.md](completed-work.md) 与 [next-mission.md](next-mission.md)。

## 1 工作流总览（一次现场窗口的固定链）

编号即执行顺序，每步一行；命令与判据在各步指向的小节里，本节不重复。

1. **§2 变量块与 `run` 函数**：同一 shell 粘一次，`$S4DIR` / `$STAMP` 由 `date -u` 定死，后续命令都假定这几步已完成。→ [1-prepare.md §2](runbook/1-prepare.md#sec-2)
2. **§3 到场核对**：网络、Host 进程、端口、工具、设备路径逐条过；其中 §3.9 授权记录落盘（五项填满）、§3.10 人工确认项逐条勾。→ [1-prepare.md §3](runbook/1-prepare.md#sec-3)
3. **两条安全前置口述确认**：就在 §3.9 记 scope 时当面口述并落盘——手臂低位或有承托；能按断电开关的人在机器人旁。→ [1-prepare.md §3.9](runbook/1-prepare.md#sec-3)
4. **§10.8 `signs` / `zeros` 标定（分段做，不是整段做完才起 Host）**：摆参考位形在 **torque 释放态**下做（人手工摆臂）→ [§6.1](runbook/2-host.md#host-start) 起 Host 后用 capture 读数（[§4.1](runbook/3-health.md#sec-4-1)）→ 算出 `signs` / `zeros` 后 [§8.1](runbook/2-host.md#sec-8-1) 停 Host；只读观测档与运动档分离，硬前置与判据见 [§10.8](runbook/4-experiments.md#exp-e8)「安全前置」。→ [4-experiments.md §10.8](runbook/4-experiments.md#exp-e8)，其中 §4.1 见 [3-health.md](runbook/3-health.md)、§6.1 与 §8.1 见 [2-host.md](runbook/2-host.md)
5. **§6.1 起 Host**：按三步法起，停 Host 用的 PID 取 `6.1a-host-pid` 里 `ps` 打出的那个（不是 `echo pid=$!`）。→ [2-host.md §6.1](runbook/2-host.md#host-start)
6. **§10.6 Gate D**：按该节的唯一触发路径取数；等不到就记「未触发」，不另造注入路径。→ [4-experiments.md §10.6](runbook/4-experiments.md#exp-e6)
7. **§10.7 `move_to` 与卷尺实测**：①a / ② / ③ 的前置是 §10.8 产出的两份 JSON；④ 卷尺实测缺量具就记「未测」，不用目测替。→ [4-experiments.md §10.7](runbook/4-experiments.md#exp-e7)
8. **收尾 §8.1**：停 Host ≠ 失力——先把手臂放到低位，再断电。→ [2-host.md §8.1](runbook/2-host.md#sec-8-1)

两条安全前置（手臂低位或有承托、断电开关在手且有人在旁）必须在 §3.9 记 scope 时口述确认并落盘，缺一条即停。

上面这条链是固定的：开场（[§2](runbook/1-prepare.md#sec-2)/[§3](runbook/1-prepare.md#sec-3)）→ `signs` / `zeros` 标定（[§10.8](runbook/4-experiments.md#exp-e8)）→ 起 Host（[§6.1](runbook/2-host.md#host-start)）→ 实验（[§10.6](runbook/4-experiments.md#exp-e6) Gate D、[§10.7](runbook/4-experiments.md#exp-e7) `move_to` 与卷尺）→ 收尾（[§8.1](runbook/2-host.md#sec-8-1)）。

**任何一步缺前置就停在那一步。** 前置清单是 [1-prepare.md §1](runbook/1-prepare.md#sec-1) 的 P0–P5 加上各步所在小节的「安全前置」；不允许跳过一步往下做，也不允许手写或猜出一个缺失的值来补齐前置。

## 2 安全底线

1. **Host 只 `SIGINT`（`kill -INT`），不 `SIGTERM`、不 `kill -9`（除 §10.5 的 M1 / M2 取证）** → [§8.1](runbook/2-host.md#sec-8-1)、[§10.5](runbook/4-experiments.md#exp-e5)
2. **`--anchor command` 与 `move_to` 必须有能按断电开关的人在机器人旁**；远程无人在场只跑 `--anchor present` → [§7.1](runbook/4-experiments.md#exp-e3)
3. **sidecar 是主证据，寄存器尸检不可信**（无 sidecar 时靠时刻先后与 Host 日志有无 `Host command watchdog expired`） → [§10.5](runbook/4-experiments.md#exp-e5)

## 3 路由：三个入口

| 我现在的目的 | 去哪 |
| --- | --- |
| 今天要上现场 | 从 [runbook/1-prepare.md](runbook/1-prepare.md) 起，按上面 8 步链走 |
| 查数字、判据、参数、退出码 | [reference/](reference/)，逐份见下表 |
| 看过什么、下一轮做什么 | [completed-work.md](completed-work.md)（含附录A/B 冻结存档）、[next-mission.md](next-mission.md) |

| reference/ 里的文件 | 管什么 |
| --- | --- |
| [facts.md](reference/facts.md) | 已确证事实速查与术语表：一条事实一句人话一个出处 |
| [gates.md](reference/gates.md) | 运行时门禁 Gate A–F 与退出码对照（`2`、`9` 的全部触发点） |
| [params.md](reference/params.md) | 两个脚本的完整参数表：默认值与合法性约束的单一来源 |
| [artifacts.md](reference/artifacts.md) | 产物与归档、主 CSV 与 sidecar 列义、证据根按场次分目录 |
| [troubleshooting.md](reference/troubleshooting.md) | 失败与中止处置、§9 已发现问题里除门禁两行之外各行 |
| [env-rebuild.md](reference/env-rebuild.md) | 隔离环境与 `PYTHONPATH` 重建 8 步 |
| [mapping.md](reference/mapping.md) | 旧文件名 / 原节号 → 新路径对照（过渡期临时表，一轮现场后删） |

### 口径出处（原 §2.1 设计参考）

口径与判据的出处；现场遇到「这条要不要判」「数字算得对不对」，查这四份。

| 文件 | 管什么 |
| --- | --- |
| `docs/ADR.md` | 决策记录 D1–D21：设计取舍与已定口径（如 D14 左夹爪本轮不重标定、D18 sidecar 单独成表、D20 残余支分级、D21 URDF 零实测） |
| `docs/acceptance-baseline.md` | 验收基线 V01–V13：一条判据一行，含实机证据列 |
| `docs/library-guide.md` | 接口细节：配置覆盖、能力探测与单位换算、控制层、公开 API、约束与机器守卫 |
| `docs/background.md` | 目标、已实现能力与实机现状 |
