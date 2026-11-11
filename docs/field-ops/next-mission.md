# 下一轮现场日任务书（第 5 次现场 · 2026-11-10 改版）

本文是**任务书**，不是通用手册：只写这一轮现场做什么、按什么顺序做、某一项不过怎么办。跑法、命令块、CSV 列义与判据数值表的原文都在各子文档里，本文只给指针与「判据是什么」的一句话摘要，不复制细节。

读者是这次去现场的人。行前先过 §7 的前置清单，并把 T0（[§3.2](#mission-t0)）在到现场之前做完；到场照 §6 的执行顺序走。

**本版是 2026-11-10 改版**：标定被完整重写、供电改为分开、旧部署已清理三件事已核实，三者的由来与「变了 / 没变」对照见 [§0](#mission-sec-0)。T1 / T2 / T3 三张任务卡与既有细节（跑法、判据、退出码、锚点链接）一条未删。

**不确定性标注**：本文引用的三节跑法（[§10.6](runbook/4-experiments.md#exp-e6)、§10.7、§10.8）都还是预写、未真机执行；本文的判据摘要同样未经现场校验，第一次执行后按实际回写。

<a id="mission-sec-0"></a>
## 0 本版修订的由来（2026-11-10 改版）

2026-11-10 发生了三件影响本轮的事。前两件只核实到「文件已重写」「用户告知」这一级，第三件是用户本人的操作。本版把它们写进任务书，并据此加了两件事：离线可做的 T0（标定基线处置）与现场先做的 S1（行程复核）。

| # | 变化 | 核实到哪一级 | 证据 / 指针 |
| --- | --- | --- | --- |
| 1 | 标定被完整重写 | pi 侧文件已核：现役 `~/.cache/huggingface/lerobot/calibration/robots/alohamini/AlohaMiniRobot.json` 2882 字节、sha256 `d6594b999eb8533932adf20b6b5be413768c521f9f082109454cd323e183dfcf`、mtime 与 ctime 同为 `2026-11-10 11:58:15 +0000`、同 inode 原地覆写，pi 上无备份（归档副本在仓库外 `Cartesianlib-workbench/calibration/AlohaMiniRobot.json`）；旧基线 2880 字节、sha256 `6d089d3879b538a806dd5008dba66cde1bdb5a819aa7c4987becd0bf6b6e4c56`。18 个电机里 **14 个手臂电机变动**，`base_left_wheel` / `base_back_wheel` / `base_right_wheel` / `lift_axis` 四个完全未动，`id` 与 `drive_mode` 全 18 项未变 | 逐字段 14 项变动表的全文在 `/Users/pcoffeep/pi-workspace/Cartesianlib-workbench/notes/2026-11-10-recalibration.md`（**尚未落 `docs/`**，本文只抄四个变动最大的关节，见 [§4](#mission-limit-risk)） |
| 2 | 供电改为分开（pi 与机械臂不再共电池） | **用户告知的物理事实，软件无法验证**；手上只有空载采样的旁证，见 [§5](#mission-power) | 2026-11-10 13:18–13:20 GMT 空载采样：`vcgencmd get_throttled` 为 `0x0`、`EXT5V_V` 为 5.02098 V / 5.01294 V、dmesg 与 `journalctl -k` 各 711 行内欠压／降频关键词命中 0 次、无 over-current、本 boot 无 USB disconnect／reset |
| 3 | 旧部署 `lerobot_alohamini.legacy-752fd799-20261015` 被清理 | 用户本人的操作已确认（2026-11-10 13:09）；另一份 `lerobot_alohamini.legacy-ec65318f-20261025`（6.3 G）仍在 | 同日只读侦察 |

### 变了 / 没变

| 项 | 变了吗 | 依据 |
| --- | --- | --- |
| 现役标定文件 | 变了：2880 → 2882 字节，sha256 由 `6d089d3879b538a806dd5008dba66cde1bdb5a819aa7c4987becd0bf6b6e4c56` 换成 `d6594b999eb8533932adf20b6b5be413768c521f9f082109454cd323e183dfcf` | 上表第 1 行 |
| 左夹爪行程 | 变了：`2021..2072`（51 tick / 4.5°）→ `2018..3483`（1465 tick / 128.8°），与右侧 1479 tick 基本一致 | 同上；`docs/investigations/gripper-range-51-tick.md` 的前提据此失效，该文件已按此关闭（状态块与「结论」段追加，原文保留） |
| 右肩 lift 行程 | 变了：`939..3258`（2319 tick / 203.8°）→ `905..2048`（1143 tick / 100.5°） | 同上；风险见 [§4](#mission-limit-risk) |
| 供电 | 变了：pi 与机械臂不再共电池（用户告知） | [§5](#mission-power) |
| 控制代码 | 没变：厂商仓 `/home/pi5/lerobot_alohamini`（分支 `p1a-merged`、HEAD `6aee1f81`）唯一未提交改动是把三个摄像头（backward / chest / wrist_left）从注释改为启用，**无任何数值常量变化** | 2026-11-10 只读侦察 |
| PID 与 stall 常量 | 没变：`P_Coefficient=16`、`I_Coefficient=0`、`_JOINT_STALL_MIN_COMMAND_ERROR_DEG=2.0`、`arm_acceleration=100`、`lift_target_speed_mm_s=150.0`、`disable_torque_on_disconnect=True` 均未改 | 同上（`config_alohamini.py` 逐项核过） |
| `signs` / `zeros` | 没变：仍完全缺失，T1 仍是从零产出 | `TODO.md` §4「signs/zeros 标定流程」行；`completed-work.md` §1 第 6 条 |
| 达成率口径 | 没变：仍是增量比值 `Δmeasured / Δcommanded`；标定零点变化在差分里一阶抵消，故 2026-11-08 的 `79.1209%` / `82.6374%` 与本轮可比 | `docs/field-results.md` 2026-11-08 节；[§5](#mission-power) 的挂起项 |

同一次只读侦察（2026-11-10，未动机器人）另有一条要先知道：Host 进程**未运行**，无任何 lerobot / python 进程，无自定义 systemd 服务；`/dev/ttyACM0`（SN `5B14029263`）与 `/dev/ttyACM1`（SN `5B14111703`）都在、开机 4 秒内枚举成功、本 boot 无掉线重连；根分区 `/dev/mmcblk0p2` 29 G 已用 23 G、**剩 4.8 G（83%）**；`ssh pi5@10.34.170.98` 免密可通，`pi5.local` 解析失败，`10.34.159.106` 与 `10.168.1.100` 不可达。`/dev/am_arm_follower_left` / `_right` 两个 udev 别名**本次未查**，已列进 §7 前置清单。

## 1 一句话目标

完成笛卡尔控制层的三项真机验证——T1 `signs`/`zeros` 标定（关节角与电机刻度之间的两个映射参数：`sign` 管方向、`zero` 管零位偏置）、T2 Gate D（运行时门禁之一：首帧观测里该关节的基线实测值缺失或非有限就拒绝，不发命令）、T3 `move_to`（库的笛卡尔入口：给末端一个目标位姿，由 IK 逆运动学解成六个关节角再下发）加卷尺实测——让「Python 库按笛卡尔目标控制手臂到指定坐标」这条链拿到数字背书。

三项不是一条依赖链上的三站：**T1 与 T0 是 T3 的两道硬前置**——第一道 T1 产出 T3 要的两份标定文件，第二道 T0 把新标定代际登记进基线（[§3.2](#mission-t0)）；T0 只需本地文件、可离线做完，所以不占现场窗口。T2 与 T3 之间**没有依赖**——Gate A–F 在 `move_to` 路径上不生效，两者只是共用同一次起 Host 的窗口，收尾同一次断电。

## 2 为什么是现在

已有证据按层排开，空白只剩最上面一层。

| 层 | 已有什么证据 | 还缺什么 | 出处 |
| --- | --- | --- | --- |
| 链路层（PC→Host 值传递） | 链路清白：sidecar（与主 CSV 同一窗口写出的 safety 快照）的运动帧里 `requested_target == accepted_target` 全 100% | — | `completed-work.md` §1 第 1 条 |
| 关节层 | 肩部全案按分级口径关闭：目标已被 Host 采纳，残余落在 Host 下游伺服／机械静摩擦；门槛肩类 `≤2°`、其余 `≤2 tick`（电机位置读数的最小刻度，`1 tick = 0.087890625°`） | — | `completed-work.md` §1 第 2 条；`docs/ADR.md` D20 |
| 标定字段 | 跨断电、跨场次 `CALIB_FIELD_DIFFS: 0`（标定字段差异的条数为 0）（该结论仅限于 2026-10-16–2026-10-22 两份同代际快照之间；2026-11-10 重标定后基线变更，不跨代际成立——见 [§0](#mission-sec-0)） | — | `completed-work.md` §1 第 4 条 |
| 笛卡尔层 | 只有离线用例 | **唯一空白**：无 `signs`/`zeros` 现场产出、无 Gate D 现场形态、无 `move_to` 真机记录、无杆长实测 | `docs/ADR.md` D21；`TODO.md` §4「IK → `ArmHandle.move_to` 的实机验证」「URDF 杆长零实测」两行；`completed-work.md` §1 第 5、6 条 |

最后一行展开成三件事，正好对应本文的 T1–T3。三项同属「无现场记录」这一类空白，是并列关系，不是一条依赖链：T2 与 T3 之间**没有依赖**——Gate A–F 在 `move_to` 路径上不生效（[§10.7](runbook/4-experiments.md#exp-e7) 的 (d) 表；`TODO.md` §4「Gate D 的现场形态」行明写「不阻塞 `move_to` 实机验证」），两者只是共用同一次起 Host 的窗口。需要先说清的三条：

- 缺 T1 则 T3 不可执行：`move_to` 的 ①a / ② / ③ 强制 `--signs-json` / `--zeros-json`，缺文件即按 `15` 退出，不得手写或猜。
- 缺 T0 则 T3 同样不可执行：新标定代际没登记，T3 的 ④ 与「限位交集」类结论就没有可比基线（见 [§3.2](#mission-t0)）。
- 缺 T3 的 ④（卷尺）则几何验收没有数字：URDF 杆长取自厂商上游、零实测，几何验收的收口方式就是 ④ 的 `+Z` 9 cm 指令对卷尺读数（`docs/ADR.md` D21）。

判据口径提醒：`docs/ADR.md` D21 决定 2 写的是「实测 vs **FK 预测**之差毫米级以内」，而 [§10.7](runbook/4-experiments.md#exp-e7) ④ 现场判的是「实测 vs **指令 `90 mm`** ≤ 1 mm」。**本轮按 §10.7 判**；D21 那句只作为「几何验收的收口方式与重新评估条件」的出处。

## 3 任务表

### 3.1 总表

| 编号 | 任务 | 一句话目的 | 指针 | 依赖 | 预计耗时 |
| --- | --- | --- | --- | --- | --- |
| T0 | 标定基线处置（**本地执行，不需现场、不需开机**） | 把新标定代际登记进记录，并写明三处受影响基线怎么处置 | [§3.2](#mission-t0) | 无（只要归档副本在本地） | 本文未设预算（离线） |
| S1 | 行程复核（现场步骤，不是 T 编号任务） | 核对被压窄的两个关节实际能推到哪 | [§3.3](#mission-s1) | torque 释放态；人守断电开关；推臂授权 | 本文未设预算 |
| T1 | `signs` / `zeros` 标定（解锁项，最先做） | 产出 T3 硬前置的四份拆分文件 | [§10.8](runbook/4-experiments.md#exp-e8) | 开场 §2 / §3 先做完 | 估算 2–3 h（手册无此数字） |
| T2 | Gate D 捕获 | 拿到或记下这道门禁的现场形态 | [§10.6](runbook/4-experiments.md#exp-e6) | Host 在跑 | 估算 10 分钟（手册无此数字） |
| T3 | `move_to` + 卷尺 | 笛卡尔入口的第一条真机记录，并量出 9 cm 到底几毫米 | [§10.7](runbook/4-experiments.md#exp-e7) | T1 的两份拆分文件（第一道）；T0 的新代际登记（第二道）；①a 只读窗口 | 估算 1 h（手册无此数字） |

耗时一列全是**估算**或未设预算，手册不给数字；按实际记录回填。依赖一列只记硬前置：**T1 与 T0 都是 T3 的硬前置**——T1 缺 `signs` / `zeros` 则 ①a / ② / ③ 按 `15` 退出，T0 不过则 T3 降级为「不可执行」（[§6](#mission-order)）；T2 与 T3 之间没有依赖，只是共用同一次起 Host 的窗口。S1 不是硬前置，但它的结论会改变 T3 的选点（见 S1 的「不过怎么办」）。

<a id="mission-t0"></a>
### 3.2 T0 标定基线处置（本地执行，不需现场、不需开机）

| 项 | 内容 |
| --- | --- |
| 目的 | 把 2026-11-10 的新标定定成「已登记的一代」，并写明三处被旧基线顶着的产物各自怎么处置——不登记，本轮所有「限位交集 / 单位口径 / 标定没被改过」类结论都没有可比基线 |
| 现状 | 新标定 sha256 与旧基线不同（`d6594b99…` 对 `6d089d38…`，2882 对 2880 字节，14 个手臂电机字段变动）。三处会因这次**预期变更**而报不一致：① `docs/acceptance-baseline.md` V12 的实机证据（`tools/verify_unit_consistency.py` 比对 18/18、差异 0 个，而其中的指纹是 2026-10-25 的）；② `completed-work.md` §1 第 4 条的 `CALIB_FIELD_DIFFS: 0`；③ [§4.1c](runbook/3-health.md#sec-4-1c) 的 A 组判据（已按代际断开：同代际内标定字段任一不等即判事故，跨代际先判态）——拿旧代际基线跑本节第二步会量出这 14 个电机的差异，但**不再断言事故**：先跑 `tools/calib_health.py` 判态，现役代际已登记、正常应输出 `CALIB_STATE: CALIB_KNOWN`；若输出 `CALIB_NEW_UNREGISTERED`（非阻断）则按下面「不过怎么办」的登记流程收尾，不记为事故 |
| 要交付的东西 | 一份「新代际已登记」的记录，四项不可缺：① 登记键 `calib_id`（代际身份，只由 `id` / `drive_mode` / `homing_offset` / `range_min` / `range_max` 五个字段决定，现役为 `4a80c0fd4421`），随附 `file_sha256` 与 `byte_size` 作**溯源辅助**（`d6594b999eb8533932adf20b6b5be413768c521f9f082109454cd323e183dfcf` / 2882 字节）——**身份判定用 `calib_id`，不用文件哈希**，同一个标定值换个字节格式就是另一个文件哈希；② 18 个电机 × 5 个字段逐项值（`id` / `drive_mode` / `homing_offset` / `range_min` / `range_max`，即 `calib_id` 的输入，与 `calibration/identity.py` 的 `CANONICAL_FIELDS` 同口径）；③ 登记时间（UTC）；④ 登记人。数据源是归档副本 `Cartesianlib-workbench/calibration/AlohaMiniRobot.json` 与现役文件 `pi-live-20261110`，两者都已登记在 `configs/calib-registry.yaml`；**不需要连 pi** |
| 三处受影响基线怎么处置 | ① **V12**：`docs/acceptance-baseline.md` 的 V12 行已按代际分写、旧代际证据标注「新代际下失效」，处置是用新指纹重跑 `tools/verify_unit_consistency.py` 并把结果记进产物；再改该文件判据语义**需另一次任务授权**。② **`CALIB_FIELD_DIFFS: 0`**：结论改写为按代际分段（本文档已在 `completed-work.md` §1 第 4 条追加「不跨代际成立」的限定，只加限定、未改原句；分段改写同样需授权）。③ **§4.1c A 组**：判据已按代际断开（比「同一代际内」而不是「跨代际恒等」，见 `docs/ADR.md` D22）；在新代际登记完成前，跨代际的比对结论不具判事故效力；登记后的比对基准是「新代际第一份指纹」 |
| 产物 | `$S4DIR/stdout/calib-gen2-registry-$STAMP.md`。T0 可离线做，产物仍与现场证据同根，便于与 §4.1c 的比对产物并列。内容 = 引用登记表里的那一条（`calib_id` / `source` / `first_seen` / `file_sha256` / `byte_size` / `acknowledged_by`）＋ `tools/calib_health.py` 的健康检查结论原文（`CALIB_STATE` 行与退出码） |
| 不过怎么办 | 可执行判据（不再靠目测）：① 跑 `uv run --no-sync python tools/calib_health.py --calib-file <标定文件>`（`--registry` 省略时默认就是仓库内 `configs/calib-registry.yaml`），现役标定必须输出 `CALIB_STATE: CALIB_KNOWN` 且退出码 `0`；② 输出 `CALIB_NEW_UNREGISTERED`（退出码 `1`）**不算过**——那是「新代际还没登记」的非阻断警告，须先跑 `uv run --no-sync python tools/calib_register.py --from-file <标定文件> --write` 登记（再跑 ① 应变 `CALIB_KNOWN`）；③ `CALIB_SUSPECT`（退出码 `3`）/ `CALIB_CORRUPT`（退出码 `4`）一律不过，按事故处置，**不得改标定文件去「对齐」**。四项缺一、或三处处置没写 → T0 不过，**T3 降级为「不可执行，记录原因」**（依赖见 [§6](#mission-order)）。不得只记一个 `file_sha256` 就算登记，也不得把「已看到新标定」当成「已登记」 |
| 已落地的做法（ADR D22） | 原「彻底做法」那一行已落地，见 `docs/ADR.md` D22。`src/alohamini_cartesian/calibration/` 子包（`identity` / `registry` / `limits`）已实现，`configs/calib-registry.yaml` 已登记两条真实代际（`2905048c28a2`、`4a80c0fd4421`），`tools/calib_health.py` 给四态健康检查（`CALIB_KNOWN` / `CALIB_NEW_UNREGISTERED` / `CALIB_SUSPECT` / `CALIB_CORRUPT`），`tools/calib_register.py` 负责登记。三层限位模块**已实现但未接入控制路径**（D22 决定 7：前置是现场实测层，`configs/limits-observed.yaml` 仍是空模板）。本卡现在只剩「补登记人」（`acknowledged_by` 仍是 `null`）＋ 上面那行「三处受影响基线怎么处置」 |

<a id="mission-s1"></a>
### 3.3 S1 行程复核（现场步骤，排在开场落 scope 之后、T1 之前）

| 项 | 内容 |
| --- | --- |
| 为什么单列 | 重标定把两个关节的 range 压窄了：`arm_right_shoulder_lift` 只剩 1143 tick（100.5°，原 2319 tick / 203.8°），`arm_left_shoulder_pan` 少 414 tick（2700 → 2286，237.3° → 200.9°）。标定 range 记的是「标定时人工扫过的范围」，不等于机械极限，**可能偏窄**；偏窄的话，T3 的可达空间是被标定切掉的，不是被机械切掉的 |
| 前置 | ① **torque 释放态**：取得方式照 [§8.1](runbook/2-host.md#sec-8-1)，**只有 `SIGINT`（`kill -INT`）释放扭矩**；开场时 Host 若未运行，**不得假定已释放**，按「先用 `kill -INT` 停一次」处理。② 有人守在断电开关旁。③ 授权里含「人手工推臂」这一类动作——这是运动档 |
| 做法 | 人对 `arm_right_shoulder_lift`、`arm_left_shoulder_pan` 各推到两端物理极限，每端停住后读一次**原始位置 tick**，并记「是否撞到机械限位」。读数用 [§4.2](runbook/3-health.md#sec-4-2) 的 Pi 侧寄存器只读模板（Host 已停、串口空闲）：读 `Present_Position`，并同时读 `Min_Position_Limit` / `Max_Position_Limit` 核对它们是否仍等于标定的 `range_min` / `range_max`（旧代际 2026-11-05 核过相等，新代际未复核） |
| 判据 | 实测极限落在标定 `range_min..range_max` **之外** = 「标定未扫满」；落在**之内** = 复核通过，T3 按现限位继续 |
| 不过怎么办 | 判到「标定未扫满」：在 T3 之前**重标**（重标会再换一次代际，回来要补一次 T0 登记），或按实测极限修正限位（改配置需另一次授权）；两者都没做，T3 不跑。读数取不到（串口被占、Host 停不掉、人推不动）：记「未测：原因」，T3 的目标点改由 ①a 只读窗口的 `effective_limits_deg` 实测确认可达后再谈真跑，并在 scope 与本轮报告里写明「限位未复核」 |
| 边界 | S1 **不是** T1 的替代：它不产出 `signs` / `zeros`，也不判符号与零点，只回答「可用行程有没有被标定切掉」 |

### 3.4 T1 §10.8 `signs` / `zeros` 标定（解锁项，最先做）

| 项 | 内容 |
| --- | --- |
| 目的 | 产出左右各一份标定文件并拆成扁平六键的 `signs` / `zeros`，让 T3 的 ①a / ② / ③ 从「不可执行」变成「可执行」 |
| 前置 | ① 摆姿在 **torque 释放态**（扭矩已释放，人手工摆臂）；② 摆姿与保持期间人手不离开；③ Host 启停按 [§6.1](runbook/2-host.md#host-start) / §8.1——**停 Host ≠ 失力**：要拿到释放态，停 Host 必须用 `kill -INT`，**只有 `SIGINT`（终端中断信号，即 `kill -INT`）会释放扭矩**，`kill -9` / `SIGTERM` 只停进程、手臂仍通电保持（[§8.1](runbook/2-host.md#sec-8-1)、README「安全底线」第 1 条）；④ `sign` 判定属运动档，要有现场运动授权 |
| 做法 | 指针：[§10.8](runbook/4-experiments.md#exp-e8) 全节（参考位形、zeros、sign 判定、自检、右臂流程、安全前置）。一句话：把臂摆成参考位形 C2b `(0, −72, +73, 0, 0, 0)`°（大臂竖直向上、小臂水平朝正前方；**这组角是 URDF 运动学角，不是电机角**，电机角现场读），读一次数算 `zeros`，再用 1° 单步判六轴 `sign`，最后注入文件跑自检 |
| 判据 | 全部落 `$S4DIR/stdout/`：左右各一份标定文件（2）＋ 拆分后扁平六键 `signs` / `zeros` 四份（4）＋ 自检 md（每侧一份；文件名无侧别后缀，右臂那份需另命名避免同 `$STAMP` 覆盖）——只做左臂共 7 份。自检两道闸都算判据：**构造闸**＝用拆好的四份文件喂 §10.7 的 ①a（`--limits-only`，只读、零发送），判退出码 `0` 且两侧六个关节都有窗口；**数值闸**＝`q_kinematic_deg` 与 `q_target` 差 ≤ 3°，末端量级核对口径 ±0.05 m（量级核对，不是毫米级判据）。①a 要连 Host，所以 T1 段内的可执行次序是「摆姿（释放态）→ 起 Host 读数 → 自检 → 停 Host」 |
| 不过怎么办 | `sign` 判定按 1° 单步法**重做单轴**（不整段重做）；仍不过就把现场值先落当次证据目录。写进 `TODO.md` §4 对应行**需另一次任务授权**（`AGENTS.md` §4「写入需授权」；记录要求本身不构成写入授权），未授权就只在报告里列「待记录事项」。**不得猜值**，**不得把左臂值当右臂值**（沿用某轴须「实测一致」＋两侧各自独立读数） |

一条硬要求单独提：**拆分后必须是扁平六键**，多包一层外层键会被当成「缺关节」并按 `15` 退出。

### 3.5 T2 Gate D 捕获

| 项 | 内容 |
| --- | --- |
| 目的 | 取到这道运行时门禁的现场形态；取不到也要留下「未触发」这个结论 |
| 前置 | Host 在跑；走 `--mode tracking`（Gate D 的唯一触发路径）；其余按 [§10.6](runbook/4-experiments.md#exp-e6) 的既有前置与 [§7.1](runbook/4-experiments.md#exp-e3) 的开跑勾选单，不另立口径 |
| 做法 | **被动等待**。Gate D 无注入路径：现场唯一合法触发是真机 Host 返回该关节的非有限反馈，不得为触发它改代码、改脚本或注入假值。指针：[§10.6](runbook/4-experiments.md#exp-e6) |
| 判据 | 两种结论都合法。捕获到：stderr 原文（形如 `Gate D refused: …`）＋ 退出码 `6` ＋ 是否落 CSV，三件一起存；未触发：记「未触发」 |
| 不过怎么办 | **不重来、不另造注入路径**。等不到就记「未触发」，判据不因此作废 |

耗时：**估算** 10 分钟（手册无此数字）。

### 3.6 T3 §10.7 `move_to` + 卷尺

**预期剧本，开跑前先读这一段。** `move_to` 下发的是 identity 姿态（单位矩阵，不是「保持当前姿态」），末端姿态会朝 identity 大幅转。identity 形态下 `wrist_roll` 要一路转到约 `-163.7°`；这个跨度 `153.6°` 的起点是**假设 seed `−10°`**，[§10.7](runbook/4-experiments.md#exp-e7)「点位从哪来」表写明「这是假设，不是真机位姿」。**两支都要按预期留痕，都不是故障**：

- 若真机位姿接近该 seed：跨 `153.6°` 这种量级的单周期解会被 step gate（每周期允许的关节步长上限，`2.0°` / 周期）逐周期拒发，最后走满超时抛 `MotionTimeout`（退出码 `11`，`moves` 里没有成功行）。
- 若真机 `wrist_roll` 本来就在 `-163°` 附近：第一次真跑**可能真的动起来**。

两支的留痕口径同源（[§10.7](runbook/4-experiments.md#exp-e7) 末尾「现场最可能的形态」段）：按预期形态留痕，不翻修代码、不改限值。

| 项 | 内容 |
| --- | --- |
| 目的 | 笛卡尔入口的第一条真机记录：① IK 解出的关节目标真的下发，② `move_to` 的成功与失败两条路径各留一条现场记录；若出现成功行，就地卷尺量一次 |
| 前置 | ① T1 产出的 `--signs-json` / `--zeros-json`（右臂用右臂自己的文件）；② 先跑 ①a 只读取限窗口，确认 `wrist_roll` 覆盖所需角度；③ 手臂低位或有承托；④ 能按断电开关的人在机器人旁；⑤ 卷尺 ＋ 美纹纸 ＋ 垂线或角尺在场；⑥ 空场、无负载、另一臂不参与；⑦ ①a（只读）与 ①b（离线 dry-run）两者都过再谈真跑（⑥⑦ 出自 [§10.7](runbook/4-experiments.md#exp-e7)「安全前置」） |
| 做法 | 指针：[§10.7](runbook/4-experiments.md#exp-e7)。一句话：①a 只读取限窗口 → ①b 离线 dry-run（零网络、零发送）→ ② 9 cm 单次 →（出现成功行才）③ 7 点路径 → ④ 卷尺实测（**必做**，不因 ② 跑顺就跳过；③ 只跑路径不量） |
| 判据 | 出现成功行：就地卷尺实测，实测与 `90 mm` 之差的绝对值 ≤ 1 mm 为过。量的是末端夹爪尖沿基座系 `+Z` 的位移标量，起点必须在跑 ② 之前立成实物标记；读数 1 mm，重复 3 次取中位 |
| 不过怎么办 | 差值超 1 mm：**停在 ④**，不改容差、不改判据，并触发 `docs/ADR.md` D21 的重新评估条件①（卷尺实测偏差超毫米级）。量具不在场：记「未测」，**不得**用目测或 FK 预测值回填。② 落成「超时 + `moves` 无成功行」时**没有位移可量**：记「未测：② 未达成动作，原因「异常类名」」，**不得**用 FK 预测值回填（[§10.7](runbook/4-experiments.md#exp-e7) 卷尺表「不能量的时候」行） |

判据只比「实测 vs 指令」，判据形式写作 `|实测 − 90 mm| ≤ 1 mm`；JSON 里同一份 URDF 算出的残差只用来解释差值落在哪，不是独立证据。口径与 `docs/ADR.md` D21 决定 2 不同，两处不要混读：D21 写「实测 vs **FK 预测**之差毫米级以内」，本节按 [§10.7](runbook/4-experiments.md#exp-e7) ④ 判「实测 vs **指令 `90 mm`** ≤ 1 mm」；D21 那句只作为「几何验收的收口方式与重新评估条件」的出处。

<a id="mission-limit-risk"></a>
## 4 限位与行程风险

限位交集 = URDF 限位 ∩ 标定 range（换算成度）；库侧的 `effective_limits_deg` 就是这个交集，URDF 里的 `±3.14` rad 不作真实行程的依据（`docs/library-guide.md:166`）。标定 range 记的是「标定时人工扫过的范围」，不等于机械极限，所以 range 一变，交集的取值来源就跟着变。

四个变动最大的关节（旧 2026-10-22 基线 → 新 2026-11-10 基线；4096 tick = 360°，1 tick ≈ 0.0879°）：

| 电机 | homing_offset | range_min..range_max | 行程宽度 tick（度） |
| --- | --- | --- | --- |
| `arm_left_gripper` | 1571 → 1570 | 2021..2072 → 2018..3483 | **51 → 1465**（4.5° → 128.8°） |
| `arm_right_shoulder_lift` | -818 → -784 | 939..3258 → 905..2048 | **2319 → 1143**（203.8° → 100.5°） |
| `arm_left_shoulder_pan` | -1359 → -1398 | 627..3327 → 670..2956 | 2700 → 2286（237.3° → 200.9°） |
| `arm_right_shoulder_pan` | -1988 → -1837 | 1247..3743 → 1231..3462 | 2496 → 2231（219.4° → 196.1°） |

另有 10 个变动较小的电机（如 `arm_left_elbow_flex` 的 homing_offset -1322 → -1170、`arm_right_gripper` 的 -1964 → -2012、`arm_left_wrist_roll` 的 -266 → -502），完整 14 项表在 `/Users/pcoffeep/pi-workspace/Cartesianlib-workbench/notes/2026-11-10-recalibration.md`，本文不抄全。

读法三条：

- **风险在被压窄的两只**：`arm_right_shoulder_lift` 的上界从 3258 掉到 2048（少 1210 tick ≈ 106.4°），`arm_left_shoulder_pan` 的上界从 3327 掉到 2956（少 371 tick ≈ 32.6°）。凡 T3 的目标点落在这些被切掉的方向上，会在 ①a 只读窗口或 IK 的限位检查里被拒。**T3 的目标点一律先过 ①a 的 `effective_limits_deg` 再谈真跑，不按旧 range 挑点。**
- **放宽的那只不改 T3 选点**：`arm_left_gripper` 51 → 1465 tick 是异常消失（现与右侧 1479 tick 基本一致）；夹爪有自己的入口（`GripperHandle`），`ArmHandle.move_to` 只收 `x` / `y` / `z` 三个量，所以它不进六轴限位。它改的是 V12 单位口径核对的输入（18 个电机全比），处置见 T0。
- **不猜方向**：「上界被切掉」对应哪个物理方向，本轮没有现场实测，不写方向假说；以 S1 的实测极限为准。

<a id="mission-power"></a>
## 5 供电变更与度量口径

结论先写：**本轮度量口径不变，不得因为「供电已改」就放宽任何判据。**

供电状态：pi 与机械臂不再共电池——**用户告知的物理事实，软件无法验证**。

可观测旁证（2026-11-10 13:18–13:20 GMT 空载采样，两条读数各一次）：

| 观测 | 读数 |
| --- | --- |
| `vcgencmd get_throttled` | `0x0` |
| `EXT5V_V` | 5.02098 V / 5.01294 V |
| 欠压／降频关键词 | dmesg 与 `journalctl -k` 各 711 行内命中 0 次 |
| over-current | 无 |
| USB | 本 boot 无 disconnect／reset |

三条限制，缺一条就不能把这组读数当「改造见效」：

1. **空载**：采样时 load 0.00、无 python 进程，没跑过任何运动；带负载时的电压与瞬时电流一次未测。
2. **pi 侧没有舵机母线遥测**：`/sys/class/power_supply/` 为空，Pi 只看得到自己的 5 V 输入，看不到机械臂母线的电压／电流。
3. **历史日志不可追**：`/var/log/syslog*` 与 `kern.log*` 不存在，`journalctl` 只有本 boot 一个 boot，改造前后的日志对照做不出来。

所以**不能说「供电改造已见效」**，也不得据此改任何阈值：T1 自检的 3°、T3 的 `|实测 − 90 mm| ≤ 1 mm`、以及任何引用残余门槛的判据全部照旧。

挂起项（下一轮，不在本轮）：**供电贡献对照实验**——用新标定 + 现供电重跑 2026-11-08 那个「5 步 × 1° 肩部测试」（`--anchor command`、settle 2 s 窗），与 `79.1209%` / `82.6374%` 比。可比的理由是口径没变：达成率是增量比值 `Δmeasured / Δcommanded`，标定零点变化在差分里一阶抵消，所以换了代际后这两个数字仍可作对照；对照之外不加新判据。

<a id="mission-order"></a>
## 6 执行顺序与依赖

| 序 | 步骤 | 指针 |
| --- | --- | --- |
| 0 | T0 标定基线处置：**本地、离线，不需现场、不需开机**，行前做完（四项登记 ＋ 三处处置口径） | [§3.2](#mission-t0) |
| 1 | 开场 §2 / §3：变量块（`$S4DIR` / `$STAMP`）、到场核对、scope（授权记录，五项落 `stdout/scope-$STAMP.md`）落盘、两条安全前置当面口述并写进 scope | [§2](runbook/1-prepare.md#sec-2)、§3（§3.9） |
| 2 | S1 行程复核：torque 释放态下推两个被压窄的关节到两端，读 `Present_Position` 与两个限位寄存器；**同一释放态接着做序 3**，不重复起停 Host | [§3.3](#mission-s1)；[§4.2](runbook/3-health.md#sec-4-2)；[§8.1](runbook/2-host.md#sec-8-1) |
| 3 | T1 摆姿与读数：**摆姿在 torque 释放态 → 起 Host 读数**（分段做，不是整段做完才起 Host；自检的 ①a 要连 Host，这里不停 Host） | [§10.8](runbook/4-experiments.md#exp-e8)；[§6.1](runbook/2-host.md#host-start) |
| 4 | 自检（T1 收尾）：构造闸＝用拆好的四份文件喂 ①a（`--limits-only`，只读、零发送），判退出码 `0` 且两侧六个关节都有窗口；数值闸＝`q_kinematic_deg` 与 `q_target` 差 ≤ 3°、末端量级核对 ±0.05 m。两道都过再停 Host | [§10.8](runbook/4-experiments.md#exp-e8)「自检」表 |
| 5 | 停 Host：**必须 `kill -INT`**（只有 `SIGINT` 释放扭矩；`kill -9` / `SIGTERM` 手臂仍通电保持） | [§8.1](runbook/2-host.md#sec-8-1)；README「安全底线」第 1 条 |
| 6 | 起 Host | [§6.1](runbook/2-host.md#host-start) |
| 7 | T2 Gate D | [§10.6](runbook/4-experiments.md#exp-e6) |
| 8 | T3 `move_to`：①a → ①b → ② → ③ → ④ 卷尺 | [§10.7](runbook/4-experiments.md#exp-e7) |
| 9 | 收尾 §8.1（先把手臂放到低位，再断电） | [§8.1](runbook/2-host.md#sec-8-1) |

依赖一句：**T1 不过（缺 `signs` / `zeros`，或缺其中一关节）则 T3 降级为「不可执行，记录原因」**——`§10.7` 的硬前置就是这两份文件，缺时脚本按 `15` 退出，**不得手写或猜**。**T0 也不过同样阻断 T3**：登记记录四项缺一、或三处处置没写，T3 一样降级为「不可执行，记录原因」，因为 T3 的 ④ 与「限位交集」类结论都要挂在已登记的标定代际上；T0 可离线完成，所以不占现场窗口。序 7 与序 8 之间**没有依赖**：Gate A–F 在 `move_to` 路径上不生效（[§10.7](runbook/4-experiments.md#exp-e7) 的 (d) 表），T2 与 T3 只是共用同一次起 Host 的窗口。S1 不是 T3 的第三道硬前置，但它判到「标定未扫满」时按 [§3.3](#mission-s1) 的处置走，处置未完成期间 T3 不跑。

## 7 用户侧前置清单

| 要什么 | 为什么 | 缺了怎么办 |
| --- | --- | --- |
| 上电 | 手臂带电才有力矩与位置读数 | 不上电就一步都不跑 |
| 手臂低位或有承托 | T3 的姿态目标是 identity，末端会朝 identity 大幅转；T1 的 C2b 要把大臂抬到竖直向上 | 摆不到安全姿态就换参考位形（§10.8 的 C2）或换侧；两侧都做不到就当天不做 T1 |
| 网络（`PI_HOST` 现场填） | SSH 与 Host 端口都靠它 | 连不上按 [§2](runbook/1-prepare.md#sec-2) 的四步处置走（上次可用的地址是 `10.34.170.98`，2026-11-10 复测免密可通） |
| 开机后先确认两个 udev 别名 | 2026-11-10 的只读侦察只看到 `ttyACM0` / `ttyACM1` 与 `/dev/serial/by-id/` 下的 `usb-1a86_…` 名，**`/dev/am_arm_follower_left` / `_right` 当前是否存在本次未查**；两套名字不能互相代替，别名缺失会让指向别名的脚本读不到口 | 按 [§3.7](runbook/1-prepare.md#sec-3) 的只读口径核：手册 §3.7 正文只有 `3.7` / `3.7b` / `3.7c` / `3.7d` / `3.7e` 五条 `run` 命令（`ttyACM*` 与 `by-id` 的核对在 `3.7b`），**别名一项此前没有命令块**，本条是本轮新点的第一项核对，用这一条只读命令：`ssh "$PI" 'ls -l /dev/am_arm_follower_* /dev/ttyACM*'`，产物存 `$S4DIR/stdout/`；判据看 `.log`（`ls` 无匹配时 exit 2，不看 `.exit`）。别名不在就改用 `ttyACM0` / `ttyACM1`，并在 scope 里写明换了路径 |
| pi 根分区的写入位置 | 2026-11-10 实测根分区 `/dev/mmcblk0p2` 29 G 已用 23 G、**剩 4.8 G（83%）**；证据与产物都落这台机器 | 产证据前先 `df -h ~` 确认余量，落盘位置写进 scope；余量不够先腾位置（清旧证据需授权） |
| 卷尺（钢卷尺或 30 cm 钢直尺，1 mm 刻度）＋ 美纹纸 ＋ 垂线或角尺 | T3 的 ④ 是几何验收唯一一条判据；这三样缺一样，该判据不成立 | 记「未测：量具不在场」，④ 不得用目测或 FK 预测值代替 |
| 断电开关旁有人 | `sign` 判定与 T3 都会真下发命令 | 远程无人在场就不跑运动档，只跑只读与离线档 |
| 运动授权，写明「哪一侧臂、沿 `+Z` 9 cm、谁在旁、开关在谁手上」，落 `scope-*`；S1 的人手工推臂同样要写明 | 事后能核对当时到底允许了什么 | 没有授权就不开跑，也不许事后补记 |
| 两条安全前置（手臂低位或有承托、能按断电开关的人在机器人旁）当面口述并写进 scope | 缺一条即停，是现场的硬门 | 没落盘就等于没确认 |

## 8 不在本轮

| 项 | 口径 |
| --- | --- |
| 遥操作对照（teleop oracle） | 本轮不列入判据与执行顺序；若现场有人力富余，可最先试它（[§10.5](runbook/4-experiments.md#exp-e5) 该小节标题的原文口径是「有人最先做，成本最低」），但它不替代 T1–T3，也不构成任何 V 编号证据 |
| 右臂全量验证 | 时间富余才做。右臂必须按 [§10.8](runbook/4-experiments.md#exp-e8) 的右臂流程独立跑：独立摆姿、独立读数、独立定 `sign`。**右臂不得复制左臂值**，沿用某轴须「实测一致」＋两次独立读数；标定文件的 `side` 字段、拆分文件名、`--signs-json` / `--zeros-json` 指向三处侧别必须对上 |
| 其余 | **不扩范围**：不新写脚本、不改库代码、不改判据、不追加验收条目 |

## 9 产出与收口

| 项 | 怎么做 |
| --- | --- |
| 证据落盘 | `~/pi-workspace/alohamini-cartesian-teleop/evidence/s4-<日期>/`，一次现场一个窗口目录；目录内 `README.md` 写命名约定 |
| scope | 按 [§3.9](runbook/1-prepare.md#sec-3) 追加，不覆盖原有记录 |
| 台账 | 现场值先落当次证据目录；回来在 `docs/field-results.md` 写新日期节（结论 / 关键数字 / 条件 / 证据文件四列）**需另一次任务授权** |
| 待办 | 关闭 `TODO.md` §4 里对应的行同样**需另一次任务授权**；未授权就只在报告里列「待记录事项」。叙事惯例照 `completed-work.md` §1 的结论表与 §2 的时间线 |
| 未测项 | 未测就写「未测」并点名缺什么，不留空、不填替代值 |

授权前提一行：**记录要求本身不构成写入授权**（`AGENTS.md` §4「写入需授权」，§8「文档里的命令不构成执行授权」；[§8.3](runbook/5-wrapup.md#sec-8-3) 也写明「数字入 `docs/field-results.md` 需另一次任务授权」）。
