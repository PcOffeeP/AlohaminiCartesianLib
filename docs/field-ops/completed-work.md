# 已执行工作（现场）

本文件只回答一件事：到 2026-11-08 为止，现场到底拿到了什么、结论是什么。执行命令、判据与产物归档不在这里，在 `runbook/` 与 `reference/` 下（旧文件名对照见 `reference/mapping.md`）。

## 1 最新结论速览

一条一行。数字与判据以被指的小节为准，本节不复述数字表。「路 A」指 2026-11-08 第二段那次复核（STAMP `20261108T141936Z`）。

| # | 结论 | 出处 |
| --- | --- | --- |
| 1 | 链路清白（target 生成到 Host 采纳这一段没有夹带）：sidecar 的运动帧里 `requested_target == accepted_target` 全 100%，五轮逐轮清点无例外，「PC→Host 值传递段」排除 | `docs/field-results.md` 2026-11-06 节「命令链路最终清白」行、2026-11-05 节「肩部停滞：排除法结论」B 行 |
| 2 | 肩部全案按 `docs/ADR.md` D20 关闭：目标已被 Host 采纳，残余（命令目标与实际停住位置之间的差）落在 Host 下游的伺服／机械静摩擦；残余门槛改按关节类别分级，肩类 `≤2°`、其余 `≤2 tick` | `docs/ADR.md` D20；`docs/field-results.md` 2026-11-08 节「路 A」表 3 |
| 3 | 断电无滑移首判成立：两份指纹之间隔了一整次断电、期间无运动，14 只位置电机 `delta` 全 `+0`，唯一例外是 `arm_left_shoulder_pan` 的 `-1` tick | `docs/field-results.md` 2026-11-08 节「路 A」表 1 |
| 4 | 标定字段稳定：`CALIB_FIELD_DIFFS: 0`，跨断电、跨场次都没有标定字段差异（该计数是标定字段差异的条数） | `docs/field-results.md` 2026-11-08 节 B1 行与「路 A」表 1、2026-11-06 节 B1 行 |
| 5 | URDF 杆长零实测，几何验收等卷尺：FK/IK 锚在厂商上游几何上，本库没有任何杆长实测，收口方式定为现场 `+Z` 9 cm 指令对卷尺读数 | `docs/ADR.md` D21；[§10.7](runbook/4-experiments.md#exp-e7) 的 ④；`TODO.md` §4「URDF 杆长零实测」行 |
| 6 | `signs` / `zeros` 待现场标定：§10.8 是预写跑法，两份映射 JSON 还没有现场产出，缺文件时 §10.7 的 ①a / ② / ③ 按 `15` 退出 | [§10.8](runbook/4-experiments.md#exp-e8)、[§10.7](runbook/4-experiments.md#exp-e7)；`TODO.md` §4「signs/zeros 标定流程」行 |
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

<a id="appendix-a"></a>
## 附录A：原 §13 验收标准（历史勾选存档，冻结）

本节冻结：勾选状态与其中的证据文件名是历史记录，随窗口推进只在新窗口里追加，不回改。

## 13 验收标准（原 §13）

每条一个判据，不写「看起来正常」。

- [ ] 第 1 节 P0–P5 逐条有结论；P0 有 `capture` 产物为证（未勾：P1 / P4 / P5 因**无人在机器人现场**未判；P0 已关闭，P2 / P3 已判） <a id="sec-13-1"></a>
- [x] `capture` exit 0，JSON 11 个顶层键齐全，`robot_metadata.schema_version` 为 `2`，且逐个电机的 8 字段覆盖情况有记录（2026-11-05 远程会话已满足，证据：`capture/fingerprint-20261105T063600Z.json`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-2"></a>
- [x] JSON 里 `lerobot_version` 非 `null`，且与第 4.1b-lerobot-version 节的日志里的 `$PI_LEROBOT_VERSION` 逐字一致（2026-11-05 远程会话已满足，证据：`capture/fingerprint-20261105T063600Z.json`、`stdout/4.1b-lerobot-version-20261105T063600Z.log`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-3"></a>
- [x] 第 4.1b 节探测输出在盘；Gate F 按「未触发」记录，理由与原始输出一并落盘，没有为凑记录改动任何文件（2026-11-05 远程会话已满足，证据：`stdout/4.1b-probe-20261105T063600Z.log`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-4"></a>
- [x] 第 4.2 节 ID7 的 `Min_Position_Limit` / `Max_Position_Limit` 原文在盘，并写明是否等于 `2021` / `2072`；`Torque_Enable` 只记录未改动（2026-11-05 远程会话已满足，证据：`stdout/4.2-gripper-regs-20261105T063600Z.log`、`stdout/4.2-gripper-regs-right-20261105T063600Z.log`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-5"></a>
- [x] 第 4.3 节两份 YAML 落盘，字段名与 `config/overlay.py` 一致，`load_settings(check_devices=False)` exit 0（2026-11-05 远程会话已满足，证据：`configs/template.yaml`、`configs/site.yaml`、`stdout/4.3e-load-check-20261105T063600Z.exit`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-6"></a>
- [x] 第 5 节三条 rtt 各 exit 0、各 500 样本、CSV 各有数据行；`achieved_rate_hz` 与请求率的差距有记录；跨机那条给出 `net_rtt_estimate_s` p50 并与 `6.176 ms` 并列（2026-11-05 远程会话已满足跨机那条，证据：`rtt/rtt-net-cross-20261105T063600Z-2.csv`、`stdout/5.3-rtt-net-cross-2-*`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-7"></a>
- [x] 第 6 节两段电流读数在盘（主路径 `latest_safety_status["currents_ma"]`，范式见 `stdout/6.1c-obs-idle-$STAMP.py`；`[Currents]` 日志路径当前不可依赖，见 §6.1），并写出结论：臂总线电流可判读，或不能判负载（2026-11-05 远程会话已满足，证据：`stdout/6.2b-analysis-20261105T063600Z.txt`、`stdout/7.2-analysis-20261105T063600Z.txt`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-8"></a>
- [x] 第 7.1 节若执行：每步的 `error_deg`、`overshoot_deg`、`settled` 齐全；`rise_90_time_s` 为空时写明原因；收尾 `hold_issued` 在报告里（2026-11-06 到场窗口已满足：10 步的 `error_deg` / `overshoot_deg` / `settled` 与 `hold_issued=yes` 在 `stdout/10.5-m1r3-pan-20261106T074049Z.log:5`、`stdout/10.5-m1r3-pan-20261106T074049Z.log:11`，逐行表在 `tracking/m1r3-pan-20261106T074620Z.csv`；`rise_90_time_s` 为空的原因「残余 > 0.1 × 命令量、没有帧越过 90% 线」由 `stdout/slow-vs-stall-judge-20261106T090635Z.txt` 第 5.4 节逐轴核成立（腕收尾残余 0.1209° 对 0.100°、`pan` 0.8242° 对 0.100°），数字见 `docs/field-results.md` 2026-11-06 节。探针本身未改、以及「收敛之后会不会有 90% 交叉」仍未观测，这两件记在 `TODO.md` 的 `rise_90_time_s` 工程债行） <a id="sec-13-9"></a>
- [x] 第 7.2 节：`target_source` 分布、`requested_target` 与 `accepted_target` 的比较、`current_ma` 走势三项有结论，且落到三类之一；`_joint_hold_goal` 有无写入点的 grep 原文在盘（2026-11-05 远程会话已满足，证据：`stdout/7.2-analysis-20261105T063600Z.txt`、`stdout/7.2-joint-hold-write-*`，数字见 `docs/field-results.md` 2026-11-05 节） <a id="sec-13-10"></a>
- [x] sidecar 的四个数字有记录：每帧行数、`motor` 空行占比、`current_ma` 分布、`hold_active` 全列取值（safety sidecar 此前只有离线测试覆盖，真机首跑按这四项记）（2026-11-05 远程会话已满足，证据：`stdout/7.1-wrist-sidecar-try2-20261105T063600Z.log`，四项数字尚未在台账逐项登记；同一轮主表见已登记的 `stdout/7.1-wrist-try2-*`） <a id="sec-13-11"></a>
- [ ] 第 8 节：断电与上电有记录，Host 停启有 PID 与两条命令原文，停机与重启时间都是 UTC（未勾：Host 停启有记录且时刻是 UTC（PID `1866`，`stdout/6.1a-host-pid-20261106T070942Z.log`；停机 `2026-11-06T09:08:01Z` + `kill_int_exit=0`，`stdout/4.1-host-int-20261106T090635Z.log`），但「断电与上电有记录」只有用户口述的换电（`stdout/scope-append-20261106T070942Z.md` ①），无仪器证据；断电前「把手臂放到低位」也没有现场记录） <a id="sec-13-12"></a>
- [ ] `$S4DIR/stdout/` 里每步都有 `.cmd`、`.log`、`.exit` 三件套（未勾：缺 `stdout/6.1a-host-start-20261105T063600Z.exit`；6.1a 其余四个子步骤 `currents-peek` / `host-log-1` / `host-procs-1` / `wait-ports-1` 三件套齐全） <a id="sec-13-13"></a>
- [x] 第 4.1c 节：`fingerprint-postpower-*.json` 在盘，`CALIB_FIELD_DIFFS: 0`，位置组 8 只 `arm_*` 超 3 tick，已判**不可归因**（基线跨过试动，见 `docs/field-results.md` 2026-11-06 节），基线已换 <a id="sec-13-14"></a>
- [x] 第 4.2 节「全 18 电机静态体检」：左 11 / 右 7 两份日志都 `EXIT_CODE: 0`，18 个 `Model_Number` 与 `MODEL_NUMBER_TABLE` 相符，`Present_Voltage` 的 min / mean / max 有记录（已执行） <a id="sec-13-15"></a>
- [x] 第 4.4 节：`examples/robot_dry_run.py` exit 0，stdout 有 `send_action 调用次数: 0 次`（已执行） <a id="sec-13-16"></a>
- [x] 第 4.5 节：`stash list` / `reflog` / `status --porcelain` 三份原文在盘，并落到「记录哈希 / 记录 reflog / 确认丢失」三支之一（2026-11-06 远程会话已满足，落「记录 reflog」支：`stash list` 与 `status --porcelain` 两份均 0 字节，`reflog --date=iso` 两行——`2026-10-25 09:17:30 checkout: moving from master to p1a-merged`、`09:22:01` fast-forward 到 `6aee1f81`；证据：`stdout/4.5a-stash-20261106T064822Z.log`、`stdout/4.5b-reflog-20261106T064822Z.log`、`stdout/4.5c-status-20261106T064822Z.log`，数字见 `docs/field-results.md` 2026-11-06 节） <a id="sec-13-17"></a>
- [x] 第 7.1b 节：模型名故意写错的命令 exit 6 或 3 有 stderr 原文、无 CSV、命令行回显三件套，并写明落到哪一支（已执行（落 Gate B）） <a id="sec-13-18"></a>
- [ ] 第 10.5 节：teleop oracle 与 M1 / M2 各有一份读数记录，且每条结论都落到 §10.5 的分支之一（未勾：M1 / M2 在 2026-11-06 已有读数并落了分支——M1（`stdout/10.5-m1-verdict-20261106T072606Z.md`）字面命中分支③，其成因判为**疑似**看门狗改写（该轮无 CSV、无 sidecar、无 Host 日志原件，成因未定，见 `docs/field-results.md` 2026-11-06 节「未取到 / 未验证」）；M2 落在 §10.5 M2 判据表新增的「介于两档之间」档（`stdout/10.5-m2r1-verdict-20261106T075623Z.md`）；**teleop oracle 本轮没跑**，只有用户口述 vendor 遥操作能带动，无仪器证据，故三项不同时满足） <a id="sec-13-19"></a>

打勾依据是 2026-11-05 远程会话与 2026-11-06 两段会话（远程会话 + 有人到场的到场窗口）的产物；勾选不等于该项已收口到验收基线，V 编号的实机证据仍按 `docs/acceptance-baseline.md` 填。

判据里两条与数据形态有关的注意：

- 2026-10-25 五份 tracking CSV 的 `received_monotonic_s` 与 `rise_90_time_s` 全空，同因：没有帧记到 90% 交点。**2026-11-05 用 `--settle-tolerance-deg 0.05`（小于 0.1°）跑三轴，`rise_90_ms count=0 p50=nan` 仍然为空，「容差取小就能补上这一列」的假设不成立**；三份 CSV 的 `rise_90_time_s` 仍全空。报告自带的说明是：要记一次 90% 交叉，需残余 ≤ `0.1 × delta` 且发生在 settle 循环结束之前，实测没发生。2026-11-06 的 `slow-vs-stall` 判读把成因核到「残余 > 0.1 × 命令量、无帧越过 90% 线」（`stdout/slow-vs-stall-judge-20261106T090635Z.txt` 第 5.4 节，逐轴两行「成立」）；探针未改、以及「收敛之后会不会有 90% 交叉」仍未观测，这两件记在 `TODO.md` 的 `rise_90_time_s` 工程债行。
- Mode B 只在 2026-10-25 跑过五次，都是左臂单关节、每步 1°。右臂、多关节联动、更大的单步行程都未跑过。


<a id="appendix-b"></a>
## 附录B：原 §4.5 Pi 未提交文件调查

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

