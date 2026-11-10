# 旧路径 → 新路径对照（过渡期，一轮现场后删）

本文件是重组过渡期的临时对照表，一轮现场跑完后删除。看到旧文件名或旧节号，先在这里查新位置；新写的文档一律直接链锚点，不再引用旧文件名。

## 表 1 旧文件名 → 新位置

| 旧文件 | 新位置 |
| --- | --- |
| `00-opening.md` | §0–§3 见 [runbook/1-prepare.md](../runbook/1-prepare.md)；§4.3 见同文件 `#sec-4-3` |
| `10-health-baseline.md` | §4.1–§4.4 见 [runbook/3-health.md](../runbook/3-health.md)；§4.3 见 [runbook/1-prepare.md#sec-4-3](../runbook/1-prepare.md#sec-4-3)；§4.5 见 [completed-work.md#appendix-b](../completed-work.md#appendix-b) |
| `20-toolchain.md` | §5.1/§5.2 见 [runbook/4-experiments.md#exp-e1](../runbook/4-experiments.md#exp-e1)；§5.3 见 [env-rebuild.md#sec-5-3](env-rebuild.md#sec-5-3) |
| `30-host-ops.md` | §6 见 [runbook/4-experiments.md#exp-e2](../runbook/4-experiments.md#exp-e2)；§6.1 程序部分见 [runbook/2-host.md#host-start](../runbook/2-host.md#host-start)；§8.1 见 [runbook/2-host.md#sec-8-1](../runbook/2-host.md#sec-8-1)；§8.2/§8.3 见 [runbook/5-wrapup.md](../runbook/5-wrapup.md) |
| `40-gates-params.md` | §7.1/§7.2 见 [runbook/4-experiments.md#exp-e3](../runbook/4-experiments.md#exp-e3)；§7.1b 见 [runbook/4-experiments.md#exp-e4](../runbook/4-experiments.md#exp-e4)；§9 见 [troubleshooting.md#sec-9](troubleshooting.md#sec-9) 与 [gates.md#sec-9-gates](gates.md#sec-9-gates)；§12 见 [troubleshooting.md#sec-12](troubleshooting.md#sec-12)；附录A 见 [params.md](params.md) |
| `50-experiments.md` | §10 见 [gates.md#sec-10](gates.md#sec-10)；§10.5–§10.8 见 [runbook/4-experiments.md#exp-e5](../runbook/4-experiments.md#exp-e5)…[#exp-e8](../runbook/4-experiments.md#exp-e8) |
| `60-artifacts.md` | §11 见 [artifacts.md#sec-11](artifacts.md#sec-11)；§13 见 [completed-work.md#appendix-a](../completed-work.md#appendix-a) |

## 表 2 原节号 → 新路径#锚点

旧表（节号 → 文件）整块保留成「旧文件」一列，右侧补新路径与锚点。

| 节号 | 旧文件 | 新路径#锚点 |
| --- | --- | --- |
| §0 | `00-opening.md` | [runbook/1-prepare.md#sec-0](../runbook/1-prepare.md#sec-0) |
| §1 | `00-opening.md` | [runbook/1-prepare.md#sec-1](../runbook/1-prepare.md#sec-1) |
| §2 | `00-opening.md` | [runbook/1-prepare.md#sec-2](../runbook/1-prepare.md#sec-2) |
| §3 | `00-opening.md` | [runbook/1-prepare.md#sec-3](../runbook/1-prepare.md#sec-3) |
| §4 | `10-health-baseline.md` | [runbook/3-health.md](../runbook/3-health.md)（§4.1–§4.4） |
| §4.1 | `10-health-baseline.md` | [runbook/3-health.md#sec-4-1](../runbook/3-health.md#sec-4-1) |
| §4.1b-lerobot-version | `10-health-baseline.md` | [runbook/3-health.md#sec-4-1b-lerobot](../runbook/3-health.md#sec-4-1b-lerobot) |
| §4.1b | `10-health-baseline.md` | [runbook/3-health.md#sec-4-1b](../runbook/3-health.md#sec-4-1b) |
| §4.1c | `10-health-baseline.md` | [runbook/3-health.md#sec-4-1c](../runbook/3-health.md#sec-4-1c) |
| §4.2 | `10-health-baseline.md` | [runbook/3-health.md#sec-4-2](../runbook/3-health.md#sec-4-2) |
| §4.3 | `10-health-baseline.md` | [runbook/1-prepare.md#sec-4-3](../runbook/1-prepare.md#sec-4-3) |
| §4.4 | `10-health-baseline.md` | [runbook/3-health.md#sec-4-4](../runbook/3-health.md#sec-4-4) |
| §4.5 | `10-health-baseline.md` | [completed-work.md#appendix-b](../completed-work.md#appendix-b) |
| §5 | `20-toolchain.md` | [runbook/4-experiments.md#exp-e1](../runbook/4-experiments.md#exp-e1) |
| §5.1 | `20-toolchain.md` | [runbook/4-experiments.md#sec-5-1](../runbook/4-experiments.md#sec-5-1) |
| §5.2 | `20-toolchain.md` | [runbook/4-experiments.md#sec-5-2](../runbook/4-experiments.md#sec-5-2) |
| §5.3 | `20-toolchain.md` | [env-rebuild.md#sec-5-3](env-rebuild.md#sec-5-3) |
| §6 | `30-host-ops.md` | [runbook/4-experiments.md#exp-e2](../runbook/4-experiments.md#exp-e2) |
| §6.1 | `30-host-ops.md` | 程序部分 [runbook/2-host.md#host-start](../runbook/2-host.md#host-start)、取数与判读 [runbook/4-experiments.md#exp-e2](../runbook/4-experiments.md#exp-e2) |
| §6.2 | `30-host-ops.md` | [runbook/4-experiments.md#exp-e2](../runbook/4-experiments.md#exp-e2) |
| §7 | `40-gates-params.md` | [runbook/4-experiments.md#exp-e3](../runbook/4-experiments.md#exp-e3) |
| §7.1 | `40-gates-params.md` | [runbook/4-experiments.md#exp-e3](../runbook/4-experiments.md#exp-e3)；Gate A–F 表见 [gates.md#gate-a](gates.md#gate-a) |
| §7.1b | `40-gates-params.md` | [runbook/4-experiments.md#exp-e4](../runbook/4-experiments.md#exp-e4) |
| §7.2 | `40-gates-params.md` | [runbook/4-experiments.md#exp-e3](../runbook/4-experiments.md#exp-e3) |
| §8 | `30-host-ops.md` | [runbook/2-host.md#sec-8-1](../runbook/2-host.md#sec-8-1) 与 [runbook/5-wrapup.md](../runbook/5-wrapup.md) |
| §8.1 | `30-host-ops.md` | [runbook/2-host.md#sec-8-1](../runbook/2-host.md#sec-8-1) |
| §8.2 | `30-host-ops.md` | [runbook/5-wrapup.md#sec-8-2](../runbook/5-wrapup.md#sec-8-2) |
| §8.3 | `30-host-ops.md` | [runbook/5-wrapup.md#sec-8-3](../runbook/5-wrapup.md#sec-8-3) |
| §9 | `40-gates-params.md` | 门禁两行 [gates.md#sec-9-gates](gates.md#sec-9-gates)；其余行 [troubleshooting.md#sec-9](troubleshooting.md#sec-9) |
| §10 | `50-experiments.md` | [gates.md#sec-10](gates.md#sec-10)（退出码表另见 [#exit-codes](gates.md#exit-codes)） |
| §10.5 | `50-experiments.md` | [runbook/4-experiments.md#exp-e5](../runbook/4-experiments.md#exp-e5) |
| §10.6 | `50-experiments.md` | [runbook/4-experiments.md#exp-e6](../runbook/4-experiments.md#exp-e6) |
| §10.7 | `50-experiments.md` | [runbook/4-experiments.md#exp-e7](../runbook/4-experiments.md#exp-e7) |
| §10.8 | `50-experiments.md` | [runbook/4-experiments.md#exp-e8](../runbook/4-experiments.md#exp-e8) |
| §11 | `60-artifacts.md` | [artifacts.md#sec-11](artifacts.md#sec-11) |
| §12 | `40-gates-params.md` | [troubleshooting.md#sec-12](troubleshooting.md#sec-12) |
| §13 | `60-artifacts.md` | [completed-work.md#appendix-a](../completed-work.md#appendix-a) |
| 附录 A | `40-gates-params.md` | [params.md#param-capture](params.md#param-capture)（`measure` 表见 [#param-measure](params.md#param-measure)） |
| Gate A–F | `40-gates-params.md` §7.1 | [gates.md#gate-a](gates.md#gate-a)，逐道 [#gate-b](gates.md#gate-b) / [#gate-c](gates.md#gate-c) / [#gate-d](gates.md#gate-d) / [#gate-e](gates.md#gate-e) / [#gate-f](gates.md#gate-f) |

## 表 3 旧 README §2.2 程序子文档表（历史对照，已作废，见上表）

| 文件 | 负责的原节号 |
| --- | --- |
| `00-opening.md` | 原 H1 前言、§0、§1、§2、§3 |
| `10-health-baseline.md` | §4 |
| `20-toolchain.md` | §5 |
| `30-host-ops.md` | §6、§8 |
| `40-gates-params.md` | §7、§9、§12、附录 A |
| `50-experiments.md` | §10、§10.5–§10.8 |
| `60-artifacts.md` | §11、§13 |
| `completed-work.md` | 本文件夹的「已执行工作」：结论速览、时间线、数字入口；不含命令与判据 |
| `next-mission.md`（下一轮任务） | — |

## 本轮需要人工解释的三处处置

1. **§9 按行拆开**：门禁相关的两行（P0 曾广播 v1、Gate F 第二段不可达）进 [gates.md#sec-9-gates](gates.md#sec-9-gates)，其余各行与原表头进 [troubleshooting.md#sec-9](troubleshooting.md#sec-9)。两处都保留原节号与列头，单元格文字未改。
2. **§13 整块冻结进 [completed-work.md 附录A](../completed-work.md#appendix-a)**；[runbook/5-wrapup.md#sec-13](../runbook/5-wrapup.md#sec-13) 只留逐条指针清单，不再复制判据原文。
3. **附录 A 参数表是「默认值 / 合法性约束」的单一来源**（[params.md](params.md)）；§7.1「参数与理由」表只讲建议值与理由（[runbook/4-experiments.md#exp-e3](../runbook/4-experiments.md#exp-e3)）。两处都没删表，只是分工写清。
