# 现场验证结果台账

按时间倒序。每行一条结论，数字可在「证据文件」里核对。证据文件都在仓库外的证据目录：

- 根目录（绝对路径）：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/`
- 索引：[evidence/s4-20261025/README.md](file:///Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/README.md)

下表「证据文件」列给的是这个目录下的相对路径。逐条命令、判据与退出码表在 [docs/s4-field-smoke.md](s4-field-smoke.md)，本文件只记拿到了什么。

## 2026-10-25 · S4 现场窗口

条件：AlohaMini 2 Pro（`robot_model=alohamini2pro`），18 个电机；本库 v0.7.0；Pi 侧运行时 HEAD `6aee1f81`（升级前为 `ec65318f`）。

### 结论一览

| 结论 | 关键数字 | 条件 | 证据文件 |
| --- | --- | --- | --- |
| Pi 运行时已发 metadata v2 | `robot_metadata.schema_version=2`，18 个电机，`homing_offset` 与 `resolution` 无缺失 | 升级到 `6aee1f81` 后 | `capture/fingerprint-final-20261025.json`（7075 B） |
| 标定跨断电保持 | 18 个电机 × 8 个字段 = 144 个字段 0 差异 | 换电池前后各采一次 | `capture/site-fingerprint-battery2.json` 与 `capture/fingerprint-final-20261025.json` |
| 腕部跟随达标 | 净行程 `+1.758°`／指令 `2.0°`，达成率 87.9%，两步 `settled=True`，误差 `0.121°` | 左臂 `wrist_roll`，每步 1°，2 步 | `tracking/modeB-wrist-20261025.csv` |
| 肩部 pan 未跟随 | 5.9%（首次，1°×3 步）、8.8%（1°×3 步与 1°×2 步各一次），全部 `settled=False` | 左臂 `shoulder_pan`，每步 1° | `tracking/modeB-first-20261025.csv`、`tracking/modeB-shoulder-20261025.csv`、`tracking/modeB-pan2-20261025.csv` |
| 肩部 lift 未跟随 | 达成率 13.2%，两步全 `settled=False` | 左臂 `shoulder_lift`，1°×2 步 | `tracking/modeB-lift-20261025.csv` |
| 五次试动均未中途中止 | `note` 列只有 `ok` 与 `settle budget exhausted`；首次 report 记 `steps_completed=3`、`abort_reason=None`、`hold_issued=yes` | 左臂单关节，五次运行 | 五份 `tracking/*.csv`、`stdout/modeB-first-20261025.stdout`（stderr 0 字节） |
| 伺服寄存器配置查不出肩部与腕部的差别 | 152 次读全部成功、零失败；`Torque_Limit` 1000、`Max_Torque_Limit` 1000、`P`/`I`/`D` = 16/0/32、`Operating_Mode` 0，四台逐项相同 | 停 Host 后只读，左臂 ID 1/2/3/6 | `stdout/reg_read_1792923639.log` |
| 扭矩处于开启态，写入者未识别 | `Torque_Enable=1`、`Lock=0`；`Status` 全 0，温度 22–27 ℃、电压 10.6–10.8 V | 停 Host 后只读 | 同上 |
| 回环 RTT | `net_rtt_est` p50 6.18 ms、p95 6.45 ms；`achieved_rate_hz=49.8`（请求 1000 Hz）；`host_timing_missing=0/100` | Pi 本机回环（`host=127.0.0.1`） | `rtt/rtt-loopback-merged-20261025.csv`、`stdout/3.2-rtt-20261025T094446Z.log` |
| Gate F 第一段有真实拒绝记录 | exit `6`、零命令；报文 `Gate F refused: the host metadata schema_version is 1, expected 2.` | v1 宿主，升级前 | `stdout/2.2b-gateF-attempt-20261025T074158Z.log` |

### 未取到的项目

| 项目 | 原因 |
| --- | --- |
| 跨机 RTT | 只在 Pi 上做过回环测量，`host=127.0.0.1`；PC 到 Pi 的真实网段未测 |
| Gate F 第二段拒绝记录 | v2 宿主 18/18 电机字段齐全（`stdout/5.1-probe21-20261025T094457Z.log`），第二段结构上不会触发 |
| 右臂试动数据 | 本轮只动左臂，`tracking/` 五份 CSV 全为 `arm_left_*` |
| 相机帧数据 | 采集时返回零帧，目录内无任何图像文件；`capture/*.json` 的 `robot_metadata.cameras` 只有 2 项，「4 路相机全零帧」这一条在证据目录里没有直接记录 |
| 运行后的回弹量 | 试动结束后未继续采样该轴，`tracking/` 里没有运行之后的记录 |

### 读法

| 口径 | 怎么算 |
| --- | --- |
| 达成率 | 净行程 ÷ 指令行程。净行程 = 末步 `measured_deg` − 首步 `baseline_deg`；指令行程 = Σ（每步 `cmd_deg` − 该步 `baseline_deg`），不用「步数 × 1°」。例：`1.7582 ÷ 2.0000 = 87.9%` |
| 停稳 | `settled=True` 指该步在 settle 预算内稳住（本轮容差 0.15°）；`note` 为 `settle budget exhausted` 即预算用完仍未稳 |
| 1 tick | 0.0879°。腕部每步约 10 tick，肩部每步约 1 tick |
| `tracking/` CSV 列 | 14 列含义见 `docs/s4-field-smoke.md` 第 10 节。要点：`cmd_deg` 是该步的绝对目标角，`baseline_deg` 是该步起点的实测角，`measured_deg` 是该步收尾的实测角 |
| 回环 RTT 的读法 | `net_rtt_est_s` 是扣掉 host 采样、相位、排队与预取等待之后的估计值，不是纯往返时延；`host=127.0.0.1` 表示这是 Pi 本机回环，不能当网络延迟读 |
| 时间基准 | 证据文件名与日志时间戳都由 `date -u` 生成，是 UTC。本文件正文里的日期（如 `2026-10-25`）也是 UTC 日期。本机时间是 UTC+8，钟点比日志数字大 8 小时：UTC 07:00 对应本地 15:00，UTC 09:33 对应本地 17:33。Pi 的系统时区在证据目录内没有 `timedatectl` 记录 |
