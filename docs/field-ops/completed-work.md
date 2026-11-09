# 已执行工作（现场）

本文件只回答一件事：到 2026-11-08 为止，现场到底拿到了什么、结论是什么。执行命令、判据与产物归档不在这里，在 `00-opening.md` … `60-artifacts.md`。

## 1 最新结论速览

一条一行。数字与判据以被指的小节为准，本节不复述数字表。「路 A」指 2026-11-08 第二段那次复核（STAMP `20261108T141936Z`）。

| # | 结论 | 出处 |
| --- | --- | --- |
| 1 | 链路清白（target 生成到 Host 采纳这一段没有夹带）：sidecar 的运动帧里 `requested_target == accepted_target` 全 100%，五轮逐轮清点无例外，「PC→Host 值传递段」排除 | `docs/field-results.md` 2026-11-06 节「命令链路最终清白」行、2026-11-05 节「肩部停滞：排除法结论」B 行 |
| 2 | 肩部全案按 `docs/ADR.md` D20 关闭：目标已被 Host 采纳，残余（命令目标与实际停住位置之间的差）落在 Host 下游的伺服／机械静摩擦；残余门槛改按关节类别分级，肩类 `≤2°`、其余 `≤2 tick` | `docs/ADR.md` D20；`docs/field-results.md` 2026-11-08 节「路 A」表 3 |
| 3 | 断电无滑移首判成立：两份指纹之间隔了一整次断电、期间无运动，14 只位置电机 `delta` 全 `+0`，唯一例外是 `arm_left_shoulder_pan` 的 `-1` tick | `docs/field-results.md` 2026-11-08 节「路 A」表 1 |
| 4 | 标定字段稳定：`CALIB_FIELD_DIFFS: 0`，跨断电、跨场次都没有标定字段差异（该计数是标定字段差异的条数） | `docs/field-results.md` 2026-11-08 节 B1 行与「路 A」表 1、2026-11-06 节 B1 行 |
| 5 | URDF 杆长零实测，几何验收等卷尺：FK/IK 锚在厂商上游几何上，本库没有任何杆长实测，收口方式定为现场 `+Z` 9 cm 指令对卷尺读数 | `docs/ADR.md` D21；`docs/field-ops/50-experiments.md` §10.7 的 ④；`TODO.md` §4「URDF 杆长零实测」行 |
| 6 | `signs` / `zeros` 待现场标定：§10.8 是预写跑法，两份映射 JSON 还没有现场产出，缺文件时 §10.7 的 ①a / ② / ③ 按 `15` 退出 | `docs/field-ops/50-experiments.md` §10.8、§10.7；`TODO.md` §4「signs/zeros 标定流程」行 |
| 7 | 臂总线电流不能判负载这一结论只限 2026-11-05 口径：11-08 的 sidecar 已采到非零电流，判据待按 sidecar 数据重裁 | `docs/field-results.md` 2026-11-05 节「运动窗口电流量级不足判读门槛」行、2026-11-08 节「路 A」表 3；`TODO.md` §3.1 第一行 |
| 8 | 左夹爪 51 tick 按 D14 本轮不重标定：限位读数与标定文件逐项一致，成因在标定值，不在写盘链路 | `docs/ADR.md` D14；`docs/field-results.md`「离线核对（无新采集）· 左右夹爪行程差异」小节 |

## 2 时间线

每步只写「问题 → 动作 → 结论」各一句，不展开。

| 日期 | 问题 | 动作 | 结论 | 出处 |
| --- | --- | --- | --- | --- |
| 2026-10-25 | 左臂单关节 1° 步进时腕部能跟上，肩部两轴不跟随 | 左臂单关节五次试动，停 Host 后只读伺服寄存器 | 跟随差异只在肩轴：腕部达成率 87.9%、肩 pan 5.9% 与 8.8%、肩 lift 13.2%；四台电机 152 次寄存器读零失败且逐项相同，查不出差别 | `docs/field-results.md` 2026-10-25 节；`~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261025/` |
| 2026-11-04 / 2026-11-05 | 原定未执行的那几节没有现场数据 | 远程 SSH 会话（无人在机器人旁）补跑 §4.1 的 Pi 侧 capture 与 §4.2、§5.3、§6.1、§6.2、§7.2 | 指纹与 lerobot 版本采到、跨机 RTT 达成、三轴试动里腕 87.91% 而肩 pan 4.40% / lift 0.00%；停滞不在 PC→Host 值传递段；臂总线电流不能判负载 | `docs/field-results.md` 2026-11-05 节（会话 STAMP 从 `20261104T023000Z` 起，见该节「读法」表第一行）；`~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261104/` |
| 2026-11-06 | 肩部停滞的根因未定（候选含扭矩未使能、目标编码取整、伺服保护锁存、力矩不足、机械、总线写静默失败） | 到场窗口逐类排除：推压测试、配置层寄存器对照、五轮 sidecar 查 `requested` 与 `accepted`、1° 单步 15 s 长窗 | 机械卡死被推压测试推翻、配置层与链路两段排除；失速判据成立（1° 的 lead 在 15 s 内只走 2.0 tick）；根因方向定为 tracking 每步把 lead 钳在 1°、误差不累积 | `docs/field-results.md` 2026-11-06 节「到场窗口（有人在场）· 肩部定性」（候选清单见 2026-11-05 节「肩部停滞：排除法结论」）；`~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261106/` |
| 2026-11-07 | 台账没有这一天的记录，当日要回答的问题无法从现有文件取到 | 台账未见记录：`docs/field-results.md` 没有 2026-11-07 节，`docs/ADR.md` 也没有「A2 生产路径」相关条目 | 本行不写结论；可查的相邻记录是修复路径的计划与落地两端 | `docs/field-results.md` 2026-11-06 节「修复路径：软件侧解耦 lead 锚点」行、2026-11-08 节 |
| 2026-11-08 | lead 锚点解耦（`--anchor command`）能不能把肩轴带动 | 左右 `shoulder_pan` 各 5 步 × 1°（`--anchor command`、settle 2 s 窗），再按路 A 用 15 s 窗双侧重跑 | 达成率 5 步 × 1° 两侧同为 77.3626%，路 A 重跑左 79.1209% / 右 82.6374%，步末残余 0.87–1.04°；残余支按 `docs/ADR.md` D20 定案，V01 收口为「通过」 | `docs/field-results.md` 2026-11-08 节（含「路 A」小节）；`~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261107/`、`~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261108/` |

台账缺口：`docs/field-results.md` 的日期节只有 2026-10-25、2026-11-05、2026-11-06、2026-11-08 四天，另有一节「离线核对（无新采集）」。没有 2026-11-04 与 2026-11-07 的独立日期节：11-04 是 11-05 那场会话的起始 STAMP，11-07 无任何记录。

## 3 关键数字只给指针

本节不复制数字表。查数字走三处入口：

1. `docs/field-results.md` 各日期节：每节给「结论 / 关键数字 / 条件 / 证据文件」四列，按时间倒序。
2. `docs/acceptance-baseline.md` V01–V13：一条判据一行，含实机证据列，并附现状汇总。
3. 证据根目录 `~/pi-workspace/alohamini-cartesian-teleop/evidence/` 下的原件，目录内 `README.md` 写命名约定。
