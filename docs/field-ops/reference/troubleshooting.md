# 失败、中止与现场处置（原 §12、§9 处置行）

本文件管什么：现场出现非 0 退出码、链路断、命令被拒、机器可能仍在动时的处置顺序，以及 §9「已发现问题与已定方案」里除门禁两行之外的各行（到现场怎么处理、判据、产物）。

<a id="sec-12"></a>
## 12 失败与中止处置（原 §12）

| 现象 | 判据 | 处置 |
| --- | --- | --- |
| exit 6（capture） | stderr 含 `schema_version=` | 记录原文，核对第 1 节 P0，不要重试 |
| exit 6（measure） | 拒绝发生在首帧后 | 记录是哪道代码 Gate（第 7.1 节表），写进 `"$S4DIR/stdout/"` |
| exit 8 | 报告里 hold 被拒 | 先断电，再排查；按「仍在动」处置，见第 8.1 节 |
| exit 1 加反馈失联 | 连续 3 次序号不涨 | 检查网络与 Host 负载 |
| 网络中断 | `ping` 丢包 | 重跑该步，不复用旧产物 |

停 Host 不等于失力、断电时序与 exit 8 的处置，见第 8.1 节。


<a id="sec-9"></a>
## 9 已发现问题与已定方案（原 §9 其余行）

一条问题一行，写的是「到现场怎么处理」；发现经过不放这张表。

| 问题 | 现场怎么处理 | 判据 | 产物 |
| --- | --- | --- | --- |
| 肩部两轴不跟随 | 试动后取 sidecar 与 `[HOST TRACKING]`（后者因日志级别压制不打，改用 sidecar，见第 6.1 节）：判据三项里 `requested == accepted` 零例外、运动帧 `target_source` 全 `command`，结论落到「① 目标被接受，停滞在伺服或机械侧」 | 达成率 `≥50%`（分母 `abs(delta) × steps`）且该步步末残余按关节类别分级：肩类 `≤2°`、其余轴 `≤2 tick（0.1758°）`；若用 `settled=True` 口径，容差必须 ≥1 LSB（0.0879°）（口径见 `docs/ADR.md` D20 与 `docs/acceptance-baseline.md` V01 行）。若两轴仍不跟随，按 §10.5 的 teleop oracle → M1 → M2 定性，结论落到硬件不可修时按 `docs/acceptance-baseline.md` §2.1 记「受阻」 | sidecar CSV + `stdout/7.2-*` |
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

门禁相关的两行（P0、Gate F 第二段）在 [gates.md#sec-9-gates](gates.md#sec-9-gates)。
