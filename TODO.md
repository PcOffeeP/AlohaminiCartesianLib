# 目标与待办

本文件只记接下来要做的事。做完的事项不留在这里：设计取舍写在 `docs/ADR.md`，架构与现状写在 `docs/background.md`，现场取到的数字写在 `docs/field-results.md`。

更新约定：状态只有 `未开始` / `进行中` / `受阻` 三种。每条小目标写清判据，并在「影响」列写明它牵动哪些 V 编号或哪个阶段的完成判据。

## 1 整体目标

把 AlohaMini 2 Pro 从「遥操作主臂牵着走」改成「PC 上一段 Python 代码按笛卡尔目标下发」，底层仍走现有发送链、单位与硬件保护。

完成判据：`docs/acceptance-baseline.md` 的 V01–V13 全部「实机证据」列填实。

当前：软件侧与实机侧的逐条统计见 `docs/acceptance-baseline.md` §2.3「现状汇总」，本文不复制条数。

## 2 阶段目标

| 阶段 | 目标 | 完成判据 | 状态 |
| --- | --- | --- | --- |
| A 单臂受控试动跑通 | 左臂六关节在 PC 侧命令下稳定跟随 | 每关节达成率 ≥50%，且该步步末残余 ≤2 tick（0.1758°）；若用 settled 口径，容差 ≥1 LSB（0.0879°） | 受阻（缺 `docs/s4-field-smoke.md` §10.5 的 teleop / M1，需人到场） |
| B 双臂受控试动 | 右臂复制 A 的结论，并验证对侧无串值 | 实机证据填实，映射见 `docs/acceptance-baseline.md` §2.4 | 未开始：前置是 A 收口 |
| C 整机协同验收 | 臂 / 底盘 / 升降分阶段互锁与协同 | 实机证据填实，映射见 `docs/acceptance-baseline.md` §2.4 | 未开始 |
| D 库可交付 | 别人拿到库能跑通示例、配置、依赖声明明确 | 示例可跑、两份 YAML 读通、`lerobot` 依赖声明落地 | 进行中：示例与两份 YAML 已跑通；依赖声明已落地为 `lerobot>=0.6.1,<0.7`（取舍见 `docs/ADR.md` D16，检查模块 `adapter/lerobot_version.py`） |

阶段完成判据与 V 编号的对应关系以 `docs/acceptance-baseline.md` §2.4 为准，本表不复述。

## 3 小目标

### 3.1 A 阶段（当前重点）

| 目标 | 判据 | 影响 | 阻塞 |
| --- | --- | --- | --- |
| 定位 `shoulder_pan` / `shoulder_lift` 不跟随命令的原因。实测 `shoulder_pan` 整轮 2 步共走 1 tick（0.0879°）、`shoulder_lift` 整轮 0 tick；腕部每步约 10 tick | 拿到这两轴一次达标记录（每关节达成率 ≥50%，且该步步末残余 ≤2 tick（0.1758°）；若用 settled 口径，容差 ≥1 LSB（0.0879°））；或经 M1 / teleop 定性为硬件不可修 → A 阶段记**受阻**（不是完成）、V01 保持未取，按臂 / 关节资格档案降级推进 B（口径见 `docs/acceptance-baseline.md` §2.1 与 `docs/ADR.md` D19 红线 4） | A 阶段完成判据；V01 实机证据 | 排除法结论见 `docs/field-results.md` 2026-11-05 节「肩部停滞：排除法结论」：A 编码取整、B PC→Host 值传递段、C 寄存器配置与限流钳制均已排除。残留未验证：D 关节扭矩未使能（弱化，不作定论）/ E 伺服保护锁存 / G 机械 / 总线写静默失败；F 力矩不足无电流证据。下一步：先 teleop oracle（位移 >20 tick 算能动，不能动即本库路径完全脱身），再 M1（10° 单步 + `kill -9` 后读左总线 `Torque_Enable` / `Goal_Position` / `Present_Position` / `Status` / `Present_Temperature` / `Present_Voltage`）；两步均需有人到场，明细见 `docs/s4-field-smoke.md` §10.5。
| 运动中采集 `Present_Load` / `Present_Current` / `Status` 的跃迁 | 用 `measure --mode tracking` 自带的 `<主CSV>-safety.csv` 回答同一问题（判据按 sidecar 重写）：① `current_ma` 逐帧走势——同一步内连续 ≥ 2 帧高于静止态基线，且该步 `measured_deg − baseline_deg` < 1 tick（0.0879°）；② `requested_target` 与 `accepted_target` 在同一步内是否相等（不等即目标被钳制）；`hold_active` 恒 `False`（vendor `_joint_hold_goal` 无写入点，`alohamini.py:298` 初始化、`:985`/`:1076`/`:1088`/`:1128`/`:1131` 只读或 pop），不可作判据。退路：寄存器侧读数与 Host 侧 `requested_targets` / `accepted_targets` 对照 | A 阶段完成判据；V01 实机证据 | 下一步：按 sidecar 自带的 `<主CSV>-safety.csv` 跑一遍本行判据（逐帧逐电机，9 列）。残留未验证：电流判据已因「臂总线电流不能判负载」降级为「无电流证据」。现场数据与结论见 `docs/field-results.md` 2026-11-05 节；列义与判读见 `docs/s4-field-smoke.md` 第 11 节。
| 识别 `Torque_Enable=1` 的写入者 | 定位到具体写入者与调用时机，或在文档里写明追查范围与未识别结论 | A 阶段完成判据；V01；V06 的扭矩前提 | 残留未验证：`Torque_Enable=1` 的写入者未识别。实测组合 `{Torque_Enable=1, Lock=0}` 与 vendor 两条常规写路径都不相容，与「`Lock` 写静默失败」相容，但这是**未验证的假说**，不是结论。逐条依据与行号见 `docs/field-results.md` 2026-11-05 节「肩部停滞：排除法结论」的弱证据段。
| M1 / M2 现场入口已就绪，但没有现场数字 | 拿到 M1、M2 两轮的寄存器读数原文（六寄存器 × ID1 / ID2 / ID6）并落到 §10.5 的分支之一 | A 阶段定性结论；V01 | 入口已落地（`321c348`：`--delta-deg` 支持负值，幅度仍钳 1°）；走法已在 `docs/s4-field-smoke.md` §10.5 写死（M1 `--steps 10`、M2 `--delta-deg -1 --steps 10`）。**无现场数字，不算验收证据** |

### 3.2 B 阶段

| 目标 | 判据 | 影响 | 阻塞 |
| --- | --- | --- | --- |
| 右臂做受控试动，复制 A 的结论，并验证对侧无串值 | V02 实机证据填实：动一侧时另一侧的观测值与动作键不变，同一份记录里能读到两侧各自的键 | B 阶段完成判据；V02 | 2026-10-25 只动左臂，`tracking/` 五份 CSV 全为 `arm_left_*`；右臂未测 |

### 3.3 C 阶段

| 目标 | 判据 | 影响 | 阻塞 |
| --- | --- | --- | --- |
| 底盘与升降做受控动作，并验证与手臂的阶段互锁 | V03、V07 实机证据填实：跨阶段请求被拒且当拍 `send_action` 为 0，释放并静止后可恢复 | C 阶段完成判据；V03、V07 | 本轮未做过底盘与升降的任何受控动作 |

### 3.4 D 阶段

| 目标 | 判据 | 影响 | 阻塞 |
| --- | --- | --- | --- |
| 决定版本兼容性检查的运行时调用点 | 决定是否在 `connect` / 适配层入口前调用 `assert_lerobot_version_supported()`，结论写进 `docs/ADR.md` 或直接落地并补测试 | D 阶段完成判据（「接口不匹配能暴露」目前只在测试期成立）；ADR D16 已知代价第三条 | 无调用点：`src/` 内没有人 import `adapter/lerobot_version.py`，只有 `tests/test_lerobot_version.py` 直接调用 |

## 4 跨阶段的工程债

与单个阶段无关、但影响最终验收可信度的事项。

| 事项 | 判据 | 影响 | 阻塞 |
| --- | --- | --- | --- |
| sidecar 在「写盘阶段被 Ctrl-C」的分支（`_write_safety_sidecar(..., after_interrupt=True)`）无测试覆盖 | 补一条测试锁住该分支：中断后再按一次 Ctrl-C 不抛出，sidecar 落盘或明确不落盘 | 中断时 sidecar 的可信度；ADR D18 第 9 条（写盘失败只警告、退出码不变）的可复核性 | 未开始：目前只有正常路径与 `OSError` 路径有测试 |
| tracking 报告（`_format_tracking_report`）不打印 sidecar 路径与行数，operator 只能自己按主 CSV 名推 `<主文件名>-safety.csv`、自己数行 | 决定是否在报告里加一行 sidecar 路径与行数（可同带空 `motor` 帧数）：加则补测试；不加则在 `docs/s4-field-smoke.md` 写明推法与数法（已写） | 现场归档与复盘效率；`docs/s4-field-smoke.md` 第 11 节「tracking 模式 sidecar CSV：safety 快照（9 列）」小节 | 未开始：现有报告字段被既有测试锁定，加字段需同步改断言 |
| Gate F 第二段拒绝（换算字段不全）在 v2 宿主上未验证 | 拿到一份第二段拒绝记录：命令原文、stderr 原文、退出码 `6`、`send_action` 为 0；或注入换算字段不全的 metadata 并在测试里锁定拒绝 | V10 实机证据 | 离线端到端拒绝路径已锁定：测试 `test_main_should_refuse_tracking_when_the_conversion_fields_are_incomplete[missing_resolution]` / `[missing_range_min]` / `[range_max_at_range_min]`，锁住退出码 `6` + `send_action` 为 0 + stderr 属换算字段而非 `schema_version`；另有 `test_main_should_still_measure_rtt_when_the_conversion_fields_are_incomplete` 记录 rtt 不做该检查。实机 v2 宿主上的拒绝记录仍未取得，故不可删该行；v2 宿主 18/18 电机字段齐全，第二段结构上不会触发，不得为造记录改标定文件或伪造数据 |
| 4 路相机采集时全返回零帧（`Frame is None`），另有两次 ZMQ 请求超时 | 查清零帧原因（相机节点、权限、采集进程占用），并拿到一次非空帧的采集记录 | 带图像的用例。（原先这里还写着「V11 帧平滑回退」，那条链接是错的：V11 第三款说的是 metadata 帧序列的处理，与相机零帧无关，本轮去掉） | 未排查。关节与 metadata 正常，未阻塞零运动阶段。证据目录里没有零帧或无图像的直接记录，`capture/*.json` 的 `cameras` 只有 2 项 |
| rtt 逐帧能力探测还有两个未验证前提：① 探测假定 `observation_sequence` 增长与 `latest_request_timing` 填充在同一次 `get_observation` 里原子完成（若后台线程先加序号再写计时面，逐帧严格探测会偶发误拒）；② 空 / 半填充拒绝路径的形态「先 1 次 state-only prime + 至少 1 帧观测才拒」只有审查实测 | 原子性有一条结论：现场观测，或把「逐帧严格、误拒可重跑」这个取舍写进 `docs/ADR.md` D12 已知代价；空 / 半填充拒绝路径的 prime 次数（1 次）与至少 1 帧观测写入测试断言 | rtt 产出的可信度（退出码 `9` 与空值行）；V01、V10 现场证据的可复核性 | 代码层面判据已由测试锁定：`test_main_should_refuse_rtt_when_the_timing_face_is_never_filled`、`test_main_should_measure_rtt_when_the_timing_face_is_filled`、`test_main_should_refuse_rtt_with_the_absent_wording_when_the_face_is_missing`，锁住的是「属性存在但恒为空 dict → 退出码 `9` + 不写 CSV」。剩余未验证：真实 vendor 客户端是否会出现「存在但为空」的时序，只能现场确认。现实现为逐帧严格：每次 `observation_sequence` 增长都在写行前复查一次，中途变空也拒（测试 `test_main_should_refuse_rtt_when_the_timing_face_empties_mid_run`）。首帧为空、次帧才填会被判 `9`，属有意取舍（测试 `test_main_should_still_refuse_rtt_when_the_timing_face_fills_from_the_second_frame`）：真实 vendor 客户端是否出现该形态未验证，若现场出现合法客户端被拒，需重新论证取舍。另有两项未验证：① 逐帧探测隐含假定「`observation_sequence` 增长与 `latest_request_timing` 填充在同一次 `get_observation` 里原子完成」；若真实客户端由后台线程先加 `sequence` 再写计时面，逐帧严格探测会偶发误拒，该原子性从未验证。② 空计时拒绝路径的形态「先 1 次 state-only prime + 观测 1 帧才拒」目前只有审查实测、没有测试锁定（prime 次数无断言） |
| 计时面「值形态」判据已收口（离线）：键在、值不可用的形态现与缺键同拒 —— 负 `age_s`、`bool` 时间戳/年龄、非 `str` 或空白 `token`、`float()` 溢出（`10**400`）、`repr()` 抛异常；真机 vendor 是否出现这些形态未验证 | 现场拿到一次「值不可用即拒」的记录，或写明该形态不可达 | rtt 产出的可信度（退出码 `9` 与空值行）；V01、V10 现场证据的可复核性 | 已修（离线）：判据函数 `_timing_key_problem`（`benchmarks/alohamini_io/measure_latency.py:675`）与入口 `missing_request_timing_keys`（`:1193`）已收口，锁定测试 `test_main_should_refuse_rtt_when_a_timing_value_is_not_usable[...]`。残留未验证：真机 vendor 是否出现这些值形态，无现场证据。
| 「`observation_sequence` 冻结 + 计时面为空 → `9`」的判读时点：中途冻结已有测试锁定，真机冻结形态未观测 | 现场观测到一次冻结形态（空面或齐全面）并记录退出码与等待时长 | 两种缺陷叠加时诊断会被「样本不足」掩盖；CPU 空转 | 已修（离线）：中途冻结由 `test_collect_rtt_samples_should_lock_the_mid_run_freeze_behaviour` 锁定（`InsufficientSamples`、`collected=3`、保留 3 行）。残留未验证：① 真机冻结形态无现场证据；② `9` 仍要等满 `--timeout-s` 才给出，期间的提前提示没有。
| 非有限 / 非正的 `--rate-hz` 与 `--timeout-s`、以及直接调用者传非有限 `rate_hz` 仍会空转：`--rate-hz` 曾只查 `<= 0`（`inf`、`nan`、`1e400` 全放行，`period_s` 退化成 `0.0`），`--timeout-s` 曾完全不校验（`nan` / `inf` 让 `deadline_s` 永不到达，`0` / 负值让预算一开局就过期） | CLI 上非有限或非正的 `--rate-hz` / `--timeout-s` 一律退出码 `2`（argparse 用法错误，不开 socket）且 stderr 出现选项名；直接调用 `collect_rtt_samples(rate_hz=nan)` 时 0 进展等待不低于 1 ms，100 ms 预算下 `get_observation` 不超过 200 次；任何有限速率下 0 进展等待都不小于 1 ms（`--rate-hz 1e9`、50 ms 预算下轮询数十次） | rtt 空转防护的两个旁门；退出码表 0/1/3/5/6/7/8/9 不受影响，新增触发点属 `2`（argparse 用法错误，本就不在该表内） | 已修（离线）：非有限或非正的 `--rate-hz` / `--timeout-s` 一律退出码 `2`（argparse 用法错误，不开 socket），0 进展等待改用 `idle_pause_s`（`:1502`，地板 `1e-3`），锁定测试 `test_parse_args_should_reject_a_non_finite_rate_hz[nan\|inf\|overflow_to_inf]`。残留未验证：① 只挡 CLI，脚本配真实 vendor client 的组合从未跑过；② 「`--timeout-s 0|-1` → 退出码 `7`」的 CLI 路径本轮消失，同形态只剩直接调用 `collect_rtt_samples(timeout_s=0)` 可达，无测试锁定。
| 四个数值旋钮的有限性校验只挡在 CLI：直接调用者传非有限值仍按回落口径解释；`host_loop_hz=inf` 下相位 / 排队项仍算成 `0.0` | 已按 `--rate-hz` / `--timeout-s` 的形状给 `--host-loop-hz` / `--settle-timeout-s` / `--settle-tolerance-deg` / `--delta-deg` 补 `not (math.isfinite(x) and ...)`（退出码 `2`、stderr 出现选项名、均在打开任何 socket 之前）；`--host-loop-hz` 的回落归一只剩一套：`_host_loop_hz_or_upper_bound`（`benchmarks/alohamini_io/measure_latency.py`，约 `:941`），`net_rtt_estimate_s` / `_queue_terms_s` / `_prefetch_terms_s` / `_format_rtt_report` 四个调用点全部走它，直接调用者传 `nan` / `0` / 负值时报告与 CSV 的 `queued_window` 与 `queue_bound_*` / `prefetch_bound_*` 一致 | rtt / tracking 两路剩下的空转与静默偏置旁门；退出码表不受影响（用法错误 `2`） | 已修（离线）：`--host-loop-hz` / `--settle-timeout-s` / `--settle-tolerance-deg` / `--delta-deg` 补有限性校验（退出码 `2`），回落归一只剩 `_host_loop_hz_or_upper_bound`，锁定测试 `test_rtt_report_should_print_the_same_queue_terms_as_the_rows[queue_gate_open\|queue_gate_closed]`。残留未验证：① 只挡 CLI，无真机证据；② 直接调用传 `host_loop_hz=inf` 时相位 / 排队项仍算 `0.0`（有意保留）；③ 报告对直接调用者的 `nan` 仍打印 `nan`。
| `capture_robot_fingerprint.py` 的 `--timeout-s` 校验只挡在 CLI：直接调用 `collect_samples(timeout_s=nan)` 仍「不终止 + 空转」 | 已按 `measure_latency.py` 的 `--timeout-s` 形状补 `not (math.isfinite(x) and x > 0.0)` → 用法错误 `2`、stderr 出现 `--timeout-s`，发生在任何 `client_factory(...)` / `connect()` 之前；退出码表 0/3/4/5/6 不变 | 采集脚本没有别的停机条件；与 rtt 侧同一类旁门 | 已修（离线）：`--timeout-s` 补 `not (math.isfinite(x) and x > 0.0)` → 用法错误 `2`、stderr 出现 `--timeout-s`，发生在任何 `client_factory(...)` / `connect()` 之前，锁定测试 `test_parse_args_should_reject_a_nonsense_timeout_s[nan\|inf\|overflow_to_inf\|zero\|negative]`。残留未验证：① 无真机证据；② 直接调用 `collect_samples(timeout_s=nan)` 仍不终止且空转，该函数无护栏、无测试锁定。
| `achieved_rate_hz` 的分母口径天然偏高 `N/(N-1)`：`elapsed_s` 从 `_run_rtt_mode` 入口起算（`benchmarks/alohamini_io/measure_latency.py:1939`），第 0 行不花一个周期 | 定下报告口径（是否改用 `(N-1)` 个周期）；偏置的适用范围已写入 `docs/ADR.md` D12 | `achieved_rate_hz` 与 `--rate-hz` 的可比性 | 既有口径，本轮未改，无测试锁定。实测（请求 200 Hz）：3 行报 264.2 Hz（1.32×，理论上限 1.50×）、5 行 231.4 Hz（1.16×，理论 1.25×）、21 行 206.5 Hz（1.03×，理论 1.05×）；500 行时偏置约 +0.2% |
| 冻结尾部的轮询代价：落后超过一个周期后退回整周期等待，但冻结后的首个周期内仍按 `IDLE_POLL_FLOOR_S`（1 ms）轮询；周期短于地板的速率（`--rate-hz 1e6` → 1 µs）整段按地板轮询 | 现场/真机确认对死链的 `get_observation` 负载可接受 | rtt 死链时的传输负载 | 离线实测（3 行后冻结，睡眠驱动时钟）：默认 30 Hz / 30 s 933 次（修前 29,903）、100 Hz / 1 s 109 次（上界 220，修前 973）、100 Hz / 100 ms 20 次（上界 40，修前 73）、1e6 Hz / 50 ms 53 次（每次 1 ms 地板，增长阶段的 3 次为 1 µs）；无现场证据 |
| 残留的退出码 `3` 逃逸面：坏客户端数据仍可能被报成「链路坏」 | 逐项要么接住归 `9`，要么写明不可达 | 退出码 `9` 与 `3` 的可分性（谁该修客户端、谁该查网） | 本轮实测（离线）：`__float__` 抛非 `ArithmeticError`（如 `RuntimeError`）→ exit `3`（`observation failed: float boom`）；`str` 子类的 `__str__` 抛异常 → exit `3`（`str boom`）；自定义 `Mapping` 的 `__contains__` / `__getitem__` 抛异常 → exit `3`（`contains boom`）。`bytes.decode` 不可达（`_request_token_text` 用 `errors="replace"`）。与基线同形，本轮未修 |
| 计时面值为数值字符串（`sent_monotonic_s="1000.0"`）经 `float()` 视为可用：有意取舍（`_request_row` 写进 CSV 的是数字），但与「须有限数值」的类型读法不同 | 现场确认 vendor 是否会用 str 传戳 | 判据的类型严格性口径（「值形态」判据是否要收紧到「是数值类型」） | 有意为之，无测试锁定：`tests/benchmarks/test_measure_latency.py:5276-5284` 以注释形式记为「looks wrong and is accepted on purpose」 |
| 拒绝文案的子句标题仍是 `missing key(s):`，而条目可能是 `age_s (negative value (-0.5))`（值坏而非缺失） | 定下降级方案（例如标题保留、在 `, so ` 之后补一句说明） | 运维读 stderr 时的可理解性 | 标题被既有测试锁住（`tests/benchmarks/test_measure_latency.py:4467` 的 `_MISSING_KEYS_MARKER = "missing key(s):"`），不得改；本轮未动文案骨架 |
| 本库没有内置编码器分辨率表：`resolve_deg_per_unit(model_resolution=...)` 的表由调用方注入，`src/` 内无 `MODEL_RESOLUTION` 常量，`kinematics/model_specs.py` 的 `ROBOT_SPECS` 只有型号/导程/轮半径 | 在 `src/` 内提供内置型号→分辨率表，或在 `docs/ADR.md` 写明「`resolution` 只来自 v2 指纹」这条前提并用测试锁定只有标定文件时的 `ConversionUnavailable` 路径 | V12 与 V10 第二段的换算前提 | 未修：目前靠 v2 指纹自带的 `resolution` 覆盖 18/18 电机，只有标定文件时无法算出 `deg_per_unit`（抛 `ConversionUnavailable`） |
| 左夹爪标定 `range` 只有 51 tick（`range_0_100` 下 4.48° 行程），离线核对判为异常；成因已定位在标定值：左 ID7 `Min_Position_Limit`/`Max_Position_Limit` 实测 `2021`/`2072`，右 ID7 同型对照 `2010`/`3479`，与标定文件逐项一致；`docs/ADR.md` D14 决定本轮不重标定 | 现场只读确认左夹爪 ID7 限位 + 右夹爪同型对照（脚本与判读见 `docs/s4-field-smoke.md` 第 4.2 节）已满足。后续若要收口：重标定后 `range` 宽度与右夹爪同量级，或查明「标定时为什么只采到 51 tick」（人工开合是否发生 / 机械是否被限住） | 左夹爪可用行程（`open()` 与 `close()` 物理上分不开，且不报错，静默失效）；V02 右臂试动时夹爪行程是否符合预期 | 完整查证过程（18 电机 `range` 全表、9 条逐条查证、原因排序、影响分析）见 `docs/investigations/gripper-range-51-tick.md`；台账结论见 `docs/field-results.md`「离线核对（无新采集）· 左右夹爪行程差异」小节；暂不重标定（D14） |
| v1 整环从未跑过：Pi 在 2026-10-25 升级前广播 `schema_version = 1`，本库未对该版本跑过整环；升级后广播 `2`，该场景只剩离线用例覆盖。帧平滑回退已按 `docs/ADR.md` D15 定为不做，不再列入待办 | 若要取 v1 整环记录，需一台仍在广播 `schema_version = 1` 的宿主，跑一次零运动会话并留存命令原文与观测；在此之前维持离线用例覆盖 | V11 实机证据 | 现场 Pi 已升级到广播 `2`，v1 宿主不可得；离线侧由 `tests/test_capability.py` 的 v1 用例覆盖（解析、换算回落、无跨帧平滑） |
| V09 的进程外零发送证据只覆盖一条路径：`examples/minimal_dry_run.py` 的臂 `move_by` + `disconnect`（`tests/test_examples_and_configs.py` 契约 A / D 用子进程 stdout 的「0 次发送」取数）。夹爪、升降、底盘、`connect` 只有进程内断言（假 client 的 `sends` 计数） | 若要进程外覆盖这几条路径，扩一个跑夹爪 / 升降 / 底盘的 dry-run 示例并复用契约 D 的 AST 守卫；否则维持 `docs/acceptance-baseline.md` V09 行的覆盖广度说明 | V09 判据的可复核强度（判据本身已满足：原文只要求示例 stdout 报 `0 次发送` 且会话读过观测） | 未做。缺口是覆盖广度，不是判据不达标 |
| `rise_90_time_s` 探针在真机恒空：三轴（`wrist_roll` / `shoulder_pan` / `shoulder_lift`）在容差 `0.05°`（小于 0.1°）下仍全空，手册原「容差 <0.1° 应能补上」的补救假设失效 | 改脚本的 90% 交叉探针，或改口径（明确不测上升时间） | V01 的 `rise_90` 一列长期无数据 | 未开工，无真机可用数据；原因未定位 |
| `arm_left_shoulder_pan` 的 `achieved_command_hz=28.4`（请求 `50.0`，两轮同参数另两次分别 43.7 / 42.8） | 查清为何 pan 那次命令节奏掉到 28.4 Hz | 50 Hz 命令节奏在单关节试动中并非稳定达成 | 未查因，未验证是否为 Host 主循环抢占或该轴反馈慢 |
| 三轴一致变差 vs 2026-10-25（pan 8.8%→4.40%、lift 13.2%→0.00%、腕部持平 87.9%→87.91%） | 下次现场先测母线电压与电机温度、再重复同一参数集 | A 阶段完成判据；V01 | 本次未测电压 / 温度，原因未定位（电流被判不能当负载证据，无旁证） |
| `test_char_1_*` 的 parity 夹具依赖秒级时间窗：`tests/test_control_api_lerobot_parity.py` 的 `_make_command_client()` 直接把 `client._feedback_requested_at = time.monotonic()`（`:85`），夹具注释写明这是 vendor `:890` 的 `control_feedback_valid` 窗口，依赖 1 s watchdog 窗口 | 让该夹具不依赖真实时钟（注入固定时间戳或缩短断言路径），或在慢机上重复跑 50 次无偶发红 | parity 守卫的可信度：慢 CI 上夹具构造与断言之间跨过窗口会偶发红 | 未开始：现状只在当前这台机器上跑过，未做慢机重复跑 |
| `_StubZmq` 缺 `Again`：`tests/test_control_api_lerobot_parity.py:43` 附近的桩只桩了 `NOBLOCK` 与 `ZMQError`，没有 `Again` | 在桩上补 `Again`（或明确记录「vendor 该路径不需要 `Again`」并给出源码位置） | 潜在掩盖：vendor 代码若有 `except zmq.Again` 分支，桩会抛 `AttributeError` 或把该分支静默跳过 | 未开始：现状能跑通，缺口在覆盖形态 |
| parity 守卫只在 vendor 环境存在：两个 parity 文件在主环境（无 `lerobot`）整体跳过，`pytest tests` 的 `2 skipped` 不覆盖它们 | 在主环境给出可复核的跳过原因说明 + 在 vendor 环境（`.venvs/alohamini-lerobot` + `PYTHONPATH=<fork 快照>/src`）跑一次并留痕；或明确记录该守卫只在 vendor 环境成立、主环境不覆盖 | 主环境全绿不代表差分对照跑过；发版判断与测试数读法的可信度 | 未开始：主环境无 `lerobot`，两个文件在收集期整体跳过，主环境侧尚未产出可复核的跳过原因说明 |
| tracking 横幅文案自相矛盾：`benchmarks/alohamini_io/measure_latency.py:3350-3353` 那句 `note: settle tolerance … deg >= 0.1 * delta ({…} deg)`，标签写的是 `0.1 * delta`，括号里打印的却是 `delta` 本身 | 改文案：括号里打印 `0.1 * abs(delta)`，或把标签改成 `delta`；改它会改正向运行的 stdout，需确认两个既有断言（`tests/benchmarks/test_measure_latency.py:3330` 查 `note: settle tolerance` 前缀与 `>= 0.1 * delta` 子串、`:3418` 查不出现该子串）仍成立 | 正向运行 stdout 的可读性；`docs/s4-field-smoke.md` §7.1 对横幅的转述 | 未修：既有瑕疵。`--delta-deg 1.0` 时打印 `(1.000 deg)`，而该句真正想说「0.1 × delta = 0.100 deg」。`docs/s4-field-smoke.md` §7.1 已写明按 `delta` 本身读那个数 |
