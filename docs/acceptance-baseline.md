# 验收基线 V01–V13

这份文档给出 13 条准入基线，逐条写清要求、软件证据、实机证据与通过判据。基线回答一个问题：这项能力现在算不算可用。

每格的软件证据都点到具体测试文件与用例名，打开文件能核对到同名函数。没有对应测试的地方直接写「无对应测试」，不写含糊的「已覆盖」。实机证据按现状填写，未取就是「未取」。本文出现的现场日期与钟点是 UTC，本地时间为 UTC+8。

## 1 主表

| 编号 | 要求（一句话） | 软件证据（本库现状） | 实机证据（现状） | 判据（什么算过） |
| --- | --- | --- | --- | --- |
| V01 | 脱离遥操作主臂，纯 PC 侧控制 | 无「主臂」专项用例，本库代码里没有 leader 分支。间接证据两条：`tests/test_examples_and_configs.py` 契约 A 的 5 条用例（`test_example_script_exits_zero`、`test_example_script_reports_zero_send_action_calls` 等）用 PC 侧假 transport 把会话整环跑通；`tests/test_control_api.py::test_default_client_factory_without_runtime_raises_runtime_unavailable` 钉住运行时只能由调用方注入 | 部分。2026-10-25 真机受控试动（Mode B，左臂单关节、每步 1°，证据目录 `tracking/` 下五份 CSV）：腕部 `arm_left_wrist_roll` 经 PC 侧命令走出净行程 `+1.758°`（指令 2.0°），达成率 `87.9%`，两步 `settled=True`，误差 `0.121°`；肩部未跟随：首次试动 `arm_left_shoulder_pan`（1°×3 步）`5.9%`，随后同一轴 `8.8%`（1°×3 步）与 `8.8%`（1°×2 步），`arm_left_shoulder_lift` `13.2%`，全部 `settled=False`，原因排查中。达成率 = 净行程 ÷ 指令行程，口径见 `docs/s4-field-smoke.md` 第 7 节。判据里「不接主臂的 `connect` 成功记录」本轮未单独留存 | `uv run --no-sync python -m pytest tests/test_examples_and_configs.py` 全绿，示例 exit 0 且 stdout 含 `0 次发送`；真机侧：每关节**达成率 ≥50%，且该步步末残余 ≤2 tick（2 × 0.0879° = 0.1758°）**；若改用 `settled=True` 口径，则**容差必须 ≥1 LSB（0.0879°）**；另需一次不接主臂的 `connect` 成功记录。判据如此限定的理由：容差 `0.05°` 比 1 LSB 还小，任一步都判不成 `settled`，旧判据「达成率 ≥50% 且 `settled=True`」在这种容差下不可达，问题出在判据本身而不是硬件。实测数字：`docs/field-results.md` 的 2026-11-05 节「达成率三轴只有腕部接近指令」一行。**11-08 补证**：`--anchor command` 5 步 × 1° 双侧达成率均 `77.36%`（见 §2.5.1），但步末残余 `12.9 tick`、`settled=False`——「残余 ≤2 tick 或 settled」支仍未满足，A 阶段维持受阻；V01 状态字不变。 |
| V02 | 双臂与夹爪独立控制，无对侧串值 | `tests/test_control_api.py::test_red_b3_left_move_sends_only_left_arm_keys_and_never_solves_right_ik`、`::test_red_b3_right_move_sends_only_right_arm_keys_and_never_solves_left_ik`、`::test_red_b19_right_gripper_is_side_isolated`；`tests/test_kinematics_adapter.py::test_t_b1_interference_keys_gripper_base_other_arm_do_not_leak`、`::test_t_b8_d_corrupt_other_side_entries_do_not_affect_clean_side` | 未取。2026-10-25 本轮只动左臂，右臂未测，对侧串值未验证。夹爪侧另受左夹爪行程 4.48° 限制（`docs/field-results.md`「离线核对」节），重标定前不足以验证夹爪控制，见 `docs/ADR.md` D14 | 上述用例通过；真机侧动左臂时右臂的观测值与动作键不变，同一份记录里能读到两侧各自的键 |
| V03 | 机械臂与底盘/升降分阶段互锁 | `tests/test_control_commands.py::test_b21_active_arm_cross_stage_base_request_blocks`、`::test_b22_active_base_lift_cross_stage_arm_request_blocks`；`tests/test_control_controller.py::test_t_c6_1_active_arm_receiving_base_target_blocks_with_zero_send`、`::test_t_c6_3_active_base_lift_receiving_arm_target_blocks_with_zero_send`、`::test_t_c6_4_interlock_is_recoverable_via_state_machine_release_and_settle`；`tests/test_control_api.py::test_red_b17_base_drive_refused_during_arm_motion`、`::test_red_b17_lift_move_to_refused_during_arm_motion` | 未取 | 跨阶段请求被拒，当拍 `send_action` 为 0；释放并等到静止后可恢复 |
| V04 | 停止优先与条件恢复 | `tests/test_control_commands.py::test_b31_stop_command_targets_standby_from_every_state`、`::test_b32_same_tick_motion_and_stop_resolves_to_standby`、`::test_b33_motion_after_stop_command_is_refused_until_resume`、`::test_b91_stop_dominates_resume_and_motion_in_same_tick_batch`、`::test_b92_resume_command_in_standby_runs_unlock_gate` | 未取 | 上述用例通过。语义是：停止命令在任一状态把状态机带到 standby；同一拍内运动与停止并存时结果为 standby；未执行 resume 前再接运动命令被拒 |
| V05 | 反馈异常拦截与时效守护 | `tests/test_control_api.py::test_red_b14_stale_feedback_aborts_move_with_feedback_lost`、`::test_red_b8_missing_settled_valid_is_fail_closed_not_arrival`、`::test_red_c3_b9_frozen_stream_mid_motion_is_not_arrival`；`tests/test_control_commands.py::test_b52_sample_id_regression_from_active_faults`、`::test_t_c12_sample_id_regression_from_non_active_state_faults`；`tests/test_control_controller.py::test_t_c8_1_follow_error_timeout_terminates_with_zero_send`；`tests/benchmarks/test_measure_latency.py::test_run_tracking_steps_should_abort_after_the_feedback_freezes_mid_run` | 未取。2026-10-25 本轮未构造陈旧或冻结反馈场景 | 陈旧或冻结反馈触发 `FeedbackLost` 或 fault，当拍零下发；脚本侧连续 3 次序号不涨判失联并中止（`docs/s4-field-smoke.md` 第 12 节） |
| V06 | 复用底层过流保护并终止目标累积 | `tests/test_control_controller.py::test_t_c4_1_hold_terminates_forward_step_and_reports_overcurrent_hold`、`::test_t_c4_2_hold_clears_target_accumulation_prevents_windup`、`::test_t_c10_4_hold_persists_never_restores_forward_motion`。过流判定本身在厂商固件与运行时一侧，本库只接收 hold 结果，这一段无对应测试 | 未取。取数方法**未定**：现约束下（不改 vendor、无负载场景）不可达，维持「未取」。理由：臂总线电流判读门槛未达到（2026-11-05 运动窗口最大 `19.5 mA`，判读门槛 `20 mA`），且 `[Currents]` 日志在厂商默认日志级别下不打印。若将来要验，需**外接电流表**，或由厂商提供**过流事件接口** | 注入 hold 后前进步终止、目标累积清零、hold 期间不再恢复前进；真机侧另需一次过流事件的记录 |
| V07 | 启动不额外规划机械臂回位 | `tests/test_control_api.py::test_red_b4_connect_does_not_home_or_solve_ik`、`::test_red_b4_first_arm_action_is_zero_displacement_hold`、`::test_red_b4_connect_dry_run_stays_silent` | 未取 | `connect(...)` 不发动作、不调 IK；会话的第一条手臂动作是零位移 hold |
| V08 | 姿态软约束 + 位置残差硬门禁 | `tests/test_arm_ik.py::test_t_ik_6_1_solve_exposes_default_orientation_weight_of_0_01`、`::test_t_ik_6_2_explicit_soft_orientation_weight_keeps_position_and_records_error`、`::test_t_ik_19_4_within_limits_with_excessive_residual_reports_residual`；`tests/test_control_controller.py::test_t_c2_2_ok_true_residual_boundary_is_inclusive`、`::test_t_c2_6_residual_limit_comes_from_profile_not_hardcoded`；`tests/test_control_api.py::test_red_b6_midpath_blocked_point_is_prechecked_before_any_send` | 未取 | 残差超过 `profile.residual_limit_m` 时报 `RESIDUAL` 且零下发；姿态权重默认 0.01，位置优先于姿态 |
| V09 | dry-run 严格零发送 | `tests/test_control_api.py::test_red_b2_move_to_dry_run_returns_step_diagnostics_and_never_sends`、`::test_red_c6_dry_run_move_by_still_reads_and_reports_feedback_lost`、`::test_red_c3_b8_dry_run_move_by_available_in_production_form`、`::test_red_c7_dry_run_gripper_never_sends`、`::test_red_c7_dry_run_lift_never_sends`、`::test_red_c7_dry_run_base_drive_and_stop_never_send`、`::test_red_c7_dry_run_disconnect_never_sends`、`::test_red_b4_connect_dry_run_stays_silent`；`tests/test_control_controller.py::test_t_c3_4_dry_run_full_lifecycle_never_sends`；`tests/test_examples_and_configs.py` 契约 A 与契约 D 用子进程 stdout 的 `0 次发送` 取数，不靠源码常量断言。覆盖广度如实说明：进程内断言覆盖臂 `move_to`/`move_by`、夹爪 open/close/set、升降 `move_to`/`move_by`、底盘 `drive`/`stop`、`disconnect`、`connect`（不发动作也不求解 IK）；进程外（子进程 stdout 取数）只有 `examples/minimal_dry_run.py` 一条路径，覆盖臂 `move_by` + `disconnect`，夹爪 / 升降 / 底盘没有进程外零发送证据 | 不适用。口径已结算：V09 判的是 dry-run 下 `send_action` 的调用次数，而 dry-run 的定义就是不向 Host 发动作；实机只能验证「发了动作对端怎么反应」，验证不了「没发」，所以这一类要求没有实机侧证据可取。本条以软件证据为唯一判据，不计入「未取」 | 上述用例通过；示例子进程 stdout 报告 `send_action` 调用次数为 0，且同一会话确实读过观测 |
| V10 | 两道 Gate 资格门禁 | `tests/benchmarks/test_measure_latency.py::test_main_should_refuse_tracking_when_the_host_reports_another_robot_model`（Gate C）、`::test_main_should_refuse_tracking_when_the_metadata_schema_is_not_two`（Gate F 第一段）、`::test_main_should_refuse_tracking_when_the_joint_units_cannot_be_resolved`（Gate F 第二段）；`tests/benchmarks/test_capture_robot_fingerprint.py::test_main_should_reject_invalid_metadata`（指纹侧同一判据） | 部分。已取得 Gate F 第一段的真实拒绝记录（v1 宿主，退出码 `6`、零命令），报文原文 `Gate F refused: the host metadata schema_version is 1, expected 2.`，见 `docs/s4-field-smoke.md` 第 2 节与第 6 节。v2 宿主下第二段（换算字段不全时的拒绝）未触发、未验证：18/18 电机字段齐全，第二段不会拒。拒绝路径实机证据已取：2026-11-06 真机 exit 6 落 Gate B（厂商先降级 schema），零命令 + 无产物；Gate C 专属文案由离线测试锁定，真机链路不可达。机器人侧按臂归档的 Gate A–E 机制本库没有，由本基线替代 | 上述用例通过；真机侧一条拒绝记录要同时含命令原文、stderr 原文与退出码 `6`，且 `send_action` 为 0 |
| V11 | 既有系统兼容（v1 帧解析 + 换算回落；帧平滑回退不做，见 `docs/ADR.md` D15） | `tests/test_capability.py::test_detect_parses_v1_metadata_and_records_missing_field_notes`、`::test_detect_fills_missing_motor_fields_with_none`、`::test_schema1_notes_still_name_missing_fields` 锁 v1 帧的解析与缺字段降级；`::test_v11_v1_frame_parses_and_conversion_falls_back_by_field`、`::test_v11_v1_frame_with_full_fields_converts_from_metadata_like_v2`、`::test_v11_consecutive_frames_are_parsed_independently_no_smoothing`、`::test_v11_no_frame_smoothing_primitive_in_the_package` 锁 v1 帧的换算回落与 capability 层无跨帧平滑。**本库无帧平滑回退实现**：`src/alohamini_cartesian/capability/` 内既没有 EMA / 低通 / slew 限速，也没有「保持上一帧」的缓存，`detect_from_observation` 与 `resolve_deg_per_unit` 都是逐帧无状态；`::test_v11_no_frame_smoothing_primitive_in_the_package` 是词边界正则扫全包的源码级守卫，当前零命中。缺口性质因此从「无测试」更正为「无实现」，并已由 `docs/ADR.md` D15 定为不做。v1 帧真正拿到的兼容是两件事：schema 1 的 metadata 能被收下（缺字段填 `None` + `notes` 点名），换算按字段三级回落（metadata → calibration-file → model-table），缺到无处可落时抛 `ConversionUnavailable` 而不猜 4096。两条边界：① 控制层有每帧关节步长门禁（默认 `2.0°`/周期）与领先夹紧，那是安全门禁与截断，不是帧序列平滑，也不按 metadata 版本切换，不属本款（实现说明见 `docs/library-guide.md` §3）；② 关节换算适配器要求 v2 metadata，v1 metadata 不得使能（`tests/test_kinematics_adapter.py::test_t_b7_d_schema_version_missing_or_not_two_rejected`），所以 v1 帧的兼容止于能力探测与单位换算，进不了笛卡尔控制回路 | 未取。Pi 在 2026-10-25 升级前广播 `schema_version = 1`，正好是这条要兼容的版本，但本库没有对该版本跑过整环；升级后 Pi 广播 `2`（P0 已关闭），这条要兼容的 v1 场景现在只能靠离线用例覆盖。 | ① schema 1 的 metadata 解析不抛错，`homing_offset` 与 `resolution` 为 `None`，`notes` 点名缺失字段；② 换算按字段三级回落（metadata → calibration-file → model-table），缺到无处可落时抛 `ConversionUnavailable` 而不猜 4096；③ capability 层无跨帧平滑（`::test_v11_no_frame_smoothing_primitive_in_the_package` 词边界正则扫全包零命中，`::test_v11_consecutive_frames_are_parsed_independently_no_smoothing` 锁定跨帧无残留）。第三款按「不做」收口，不是待补实现。原文第三款「帧平滑回退这条要补用例之后才可判」的两种读法已由 D15 收口：① 兼容旧系统发来的帧 → 现状已覆盖解析与换算；② 回退到旧的平滑路径 → 本库不提供这条路径，也不打算提供。本行仍为「部分」：软件侧三款可判，实机侧未取（见本行实机证据列），不得据此写成已完成或通过 |
| V12 | 运动学单位与限位交集准确 | `tests/test_kinematics_adapter.py::test_t_b6_b_one_sided_narrowing_bounds_from_urdf_and_cal`、`::test_t_b6_c_empty_intersection_rejects_every_conversion`、`::test_t_b6_e_zero_width_urdf_limit_rejects_every_conversion`、`::test_t_b6_h_bus_silent_clamp_pre_rejected_with_oracle`；`tests/test_capability.py::test_resolve_uses_metadata_fields_first`、`::test_resolve_falls_back_to_calibration_file`、`::test_resolve_span_by_normalization`；`tests/test_control_controller.py::test_t_c11_1_solve_contract_seed_shape_and_adapter_intersection_limits`、`::test_t_c11_2_solve_limits_track_injected_adapter_windows_not_placeholder` | 已核对（离线，盘上真机数据，不是新的现场动作）。材料：`schema_version = 2` 的 metadata 指纹 `capture/fingerprint-final-20261025.json`（7075 B，18 个电机 8 字段齐备、无 `null`，见 `docs/s4-field-smoke.md` 第 5 与第 13 节）与现场标定 `/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/calibration-AlohaMiniRobot.json`（18 个电机，见同文第 3.7 节 `3.7e-calib`）。核对工具 `tools/verify_unit_consistency.py`：18/18 个电机两侧 `range_min`/`range_max` 完全相等，`homing_offset`/`id`/`drive_mode` 亦逐项相等，差异 0 个，退出码 `0`。`resolution` 全部 4096，`deg_per_unit` 落在 0.044835（`arm_left_gripper`）–1.800000（`arm_left_wrist_roll` 等满 range 电机）deg/单位（如 `arm_left_shoulder_pan` 1.186813、`arm_right_shoulder_pan` 1.097143、`arm_left_elbow_flex` 0.950330）；`delta_deg` 列全为 `0.000` 是两侧 `range` 相等的算术后果（行程只由 range 与 `resolution` 算出，range 相等即必然相等），不是第二重证据。本次核对的是两侧 `range` 与 `id`/`drive_mode`/`homing_offset` 一致，**不含 URDF 限位交集**；`deg_per_unit` 的**绝对**正确性无法用两份离线文件互证（`resolution` 只有指纹一个来源），本条只证明「两侧单位口径一致」。`lift_axis` 的 `normalization` 是 `degrees`，按库口径 `deg_per_unit` 恒为 1.0、range 只是占位（`detect.py:204-206`）。同时更正上一版的一句错述：`resolution` 不在标定文件里，也**不**来自 `kinematics/model_specs.py`（该文件只有 `ROBOT_SPECS` 的型号/导程/轮半径，没有编码器分辨率表）；本库里 `resolution` 来自 v2 指纹的 `robot_metadata.motors[*].resolution`，而 `resolve_deg_per_unit(model_resolution=...)` 的型号表由调用方注入、`src/` 内无内置表 | 上述用例通过；真机侧需一份 `schema_version = 2` 的 fingerprint JSON，逐电机 8 字段齐备，且用它算出的 `deg_per_unit` 与标定文件的 range 一致（2026-10-27 已按要求用 `tools/verify_unit_consistency.py` 复核，见实机证据列） |
| V13 | IK 独立解耦、实时无卡顿 | `tests/test_arm_ik.py::test_t_ik_12_1_module_imports_with_placo_absent`、`::test_t_ik_12_5_module_has_no_client_host_or_network_imports`、`::test_t_ik_13_1_success_result_exposes_sane_timing`；`tests/benchmarks/test_benchmark_arm_ik.py::test_t_ik_16_2_budget_judgement_accepts_values_within_the_closed_bounds`、`::test_t_ik_16_3_budget_judgement_rejects_marginal_overrun_on_either_axis`、`::test_t_ik_14_4_step_p99_is_finite_positive_and_exceeds_solve_p99` | 未取（取数形态已定义）。取数形态：在目标部署 PC（Mac）上跑 `benchmarks/alohamini_ik` 的离线基准，与预算常数比（`solve_p99_ms` / `step_p99_ms` 落在预算内即 `within_budget=True`），记录里写明**该 PC 的 CPU 型号**与基准命令原文 | `control/arm_ik.py` 不 import 客户端、Host、网络模块；离线基准的 `solve_p99_ms` 与 `step_p99_ms` 落在预算常数内，`within_budget` 返回 True |

## 2 使用说明

### 2.1 这份基线替代什么

替代旧仓库那份「按臂归档的 Gate A–E」准入记录。

为什么替代：Gate A–E 是机器人运行时一侧的逐臂检查项，事实来源在厂商运行时与现场标定。本库是产品侧，只通过 `adapter/protocol.py` 两个 `Protocol` 与对端接触，拿不到也不需要那些归档。原来用 Gate 归档来判断「能不能放出这一步」，现在改用这份基线，把每条要求对到本库可复现的软件证据和现场能取到的实机证据上。

注意区分两处同名的 Gate：本文说的 Gate A–E 指旧归档；`docs/s4-field-smoke.md` 第 1 节那六道 Gate A–F 是 `benchmarks/alohamini_io/measure_latency.py` 里的运行时门禁，是代码里的检查，不是归档。

**Gate A–E 归档为什么不作准入记录（已决）**：Gate 系统连同设计规范 D018 定下的归档 JSON 格式（`docs/alohamini-control/gates/<side>-gate-<letter>.json`，`schema_version` 1，`tools/alohamini_gate_record.py` 校验）已删除。归档工具不再存在，归档格式无法产出，所以 Gate A–E 不能作为准入记录使用，由本基线（V01–V13）替代。这是已决结论，出处 `docs/ADR.md` D17，不是暂定项。

**本基线未覆盖的缺口：按臂资格档案**：V01–V13 里没有「哪条臂已通过受控试动」的按臂档案。现状是左臂部分通过、右臂完全未测。左臂的数字见 V01 行的原值：`arm_left_wrist_roll` 达成 `87.9%`、`settled=True`；`arm_left_shoulder_pan` `5.9%` 与 `8.8%`、`arm_left_shoulder_lift` `13.2%`，全部 `settled=False`；右臂未测（V02 行同样记「未取」）。若将来需要按臂资格判定，应另立条目，而不是复活 Gate 归档格式。这个缺口不占 V 编号，也不预留占位编号。

**A 阶段的降级出口（受阻 ≠ 完成）**：「受阻」= **当前无法推进**（A 阶段这里是肩部 `arm_left_shoulder_pan` / `arm_left_shoulder_lift` 两轴未跟随），记录时必须写明缺哪个动作——现在是缺 `docs/s4-field-smoke.md` §10.5 的 teleop oracle 与 M1（需人员到场）。经 M1 / teleop 定性为**硬件不可修**之后**仍记「受阻」**（不改成别的状态），并追加「按臂 / 关节资格档案降级推进 B」这条出口：已取得跟随证据的关节按已取记录进档案，未跟随的关节资格为空，不继承、不推断。两种情形都**不得记「完成」**，V01 的实机证据保持「未取」、**不因降级而算通过**。这条口径的取舍与红线见 `docs/ADR.md` D19（红线 4）。它替代旧的逃逸口——旧写法允许把「在手册里写明结构性原因」当作 A 阶段的交差方式，这不成立。

### 2.2 什么时候算关闭

一条算关闭，要同时满足三件事：

1. 软件证据列填的是可点开核对的测试文件名与用例名，且这些用例在本机实跑通过。
2. 实机证据列填的是可复查的记录：命令原文、输出原文、退出码、产物路径，缺一项不算。例外：实机证据列写「不适用」的条目（目前只有 V09）按已填处理——那类要求的方向决定它没有实机侧证据可取，判据全在软件侧，理由写在对应行的实机格与 2.3 的说明里。
3. 判据列写成可执行的形式，任何人照做能复现同一结论。写「看起来正常」「基本可用」的判据不算关闭。

两列都填实之后，把表中该行的实机证据从「未取」改成记录路径，并在本节汇总里改数。

### 2.3 现状汇总

逐条统计，与本文件第 1 节一致：

| 维度 | 有证据 | 部分 | 不适用 | 未取 | 合计 |
| --- | --- | --- | --- | --- | --- |
| 软件侧 | 10 条（V02、V03、V04、V05、V07、V08、V09、V10、V12、V13） | 3 条（V01、V06、V11） | 0 条 | 0 条 | 13 |
| 实机侧 | 1 条（V12） | 2 条（V01、V10） | 1 条（V09） | 9 条 | 13 |

「部分」的含义：有专项用例或现场记录，但只覆盖该条要求的一部分，缺口在对应格子里写明了。

「不适用」的含义：这一条要求的方向决定它没有实机侧证据可取，不是「还没取」。目前只有 V09 属于这一类（理由写在 V09 的实机格）。实机侧「未取」从 10 条降为 9 条是这一条改分类的结果，不是新拿到了现场记录。

V11 仍是「部分」，缺口已由 `docs/ADR.md` D15 收口——帧平滑回退不做，软件侧判据为「v1 能解析 + 换算可回落 + 缺字段填 `None` 并点名」，三款均有用例锁定；仍算「部分」的原因是**实机侧未取**：本库没有对 v1 宿主跑过整环，Pi 升级后只广播 schema 2，这条只能靠离线用例覆盖。

实机侧已取的三条中 V12 已全实，V01、V10 仍只是部分：

- V01 取到的是腕部一个关节的跟随证据（`arm_left_wrist_roll` 达成 `87.9%`，两步 `settled=True`）。肩部两轴未跟随（首次试动 `arm_left_shoulder_pan` `5.9%`，随后同一轴 `8.8%`、`arm_left_shoulder_lift` `13.2%`，全部 `settled=False`），原因排查中；判据里「不接主臂的 `connect` 成功记录」本轮未单独留存。补证跑法与判据见 §2.5。
- V10 取到的是 Gate F 第一段的拒绝记录（v1 宿主，退出码 `6`、零命令）。v2 宿主下第二段（换算字段不全时的拒绝）未触发、未验证，理由是 18/18 电机字段齐全，第二段不会拒。拒绝路径实机证据已取：2026-11-06 真机 exit 6 落 Gate B（厂商先降级 schema），零命令 + 无产物；Gate C 专属文案由离线测试锁定，真机链路不可达。
- V12 已核对（离线，盘上真机数据）：`tools/verify_unit_consistency.py` 逐电机比对 18/18 两侧 `range_min`/`range_max` 完全相等（`homing_offset`/`id`/`drive_mode` 亦逐项相等），差异 0 个、退出码 `0`；`resolution` 全部 4096，`delta_deg` 列全为 `0.000`（它是两侧 range 相等的算术后果，不是第二重证据）。判据里「用它算出的 `deg_per_unit` 与标定文件 range 一致」这一条据此关闭。限定：本条只证明「两侧单位口径一致」，不含 URDF 限位交集；`deg_per_unit` 的绝对正确性无法用两份离线文件互证（`resolution` 只有指纹一个来源）。

P0 已于 2026-10-25 关闭，该时点 HEAD 为 `6aee1f81d503be98972d605ed32189221a674607`（UTC 09:33:54 的采指纹已在这版上完成；升级前 UTC 07:00 前后的 HEAD 是 `ec65318f`）：Pi 上完全跑通的路径 `/home/pi5/lerobot_alohamini`，旧部署备份在 `/home/pi5/lerobot_alohamini.legacy-ec65318f-20261025`。

### 2.4 跨阶段验收项

V04（停止优先与条件恢复）、V05（反馈异常拦截与时效守护）、V06（复用底层过流保护）、V08（姿态软约束 + 位置残差硬门禁）是**跨阶段验收项**，不专属 A–D 任一阶段；A–D 的阶段完成判据只引用 V01–V03 与 V07（A→V01、B→V02、C→V03 / V07，D 行不引用 V 编号），其余条目按第 1 节各自结算，不参与阶段完成判据。

理由：这四条判的是「出事时怎么处置」的横切能力（停机、反馈失效、过流、残差异常），每次运动都可能碰到，不是某个阶段的一次性里程碑。把它们计入某阶段的完成判据，会让阶段完成被一条与阶段目标无关的条目挡住；反过来，也不能因它们未取而判某阶段未完成。它们的实机证据仍按第 1 节各自填写，与阶段推进脱钩。

### 2.5 V01 的实机证据补充：验证跑法（预写）

V01 现在的实机证据只有一个腕部关节的跟随（`arm_left_wrist_roll` 达成 `87.9%`，两步 `settled=True`），肩部两轴未跟随，见 §2.3。本节写补证的跑法，**不改变 V01 当前状态**：跑完并拿到本节末尾的产物之后，才按 §2.2 把 V01 的实机证据列改成记录路径、并改 §2.3 的汇总数。

**硬前置（缺一不跑）**：

- **手臂须低位或有承托。** `--anchor command` 下 lead 跨步累积，一旦破静摩擦，整段 lead 可能一次性释放，抬高的手臂被一次性带走，幅度不可控。
- **必须有能按断电开关的人在机器人旁。** 远程且无人在场时**禁止**跑 `--anchor command`——没人能随时断电，一次性释放没有兜底。

**跑法**：机器人上电 → 按 `docs/s4-field-smoke.md` §6.1 起 Host（判据 `pid=` 与 `ready=1`）→ 左右 `shoulder_pan` 各跑一轮。关节选项是 `--joint`，取 `arm_left_shoulder_pan` / `arm_right_shoulder_pan`。以下只是**参数片段**，解释器 / `--host` / 脚本路径等前缀照 §7.1 `run 7.1-tracking` 那一行，只换关节并新增 `--anchor command`：

```bash
--mode tracking --enable-motion --anchor command --delta-deg 1.0 --steps 5 \
--control-hz 50.0 --settle-tolerance-deg 0.05 --settle-timeout-s 2.0 --timeout-s 60 \
--csv "$S4DIR/tracking/tracking-2.5-<side>-anchor-command-$STAMP.csv"
```

首跑 **3–5 步**（`--steps 5`）以安全为先；无异常后再考虑加步数。

**`--anchor command` 的用意**：锚点从上一条**实测反馈**换成上一条**发出的 target**，误差跨步累积，肩轴才有机会破静摩擦。默认的 `--anchor present` 下每步目标都是「实测 + 1°」，走不到的部分不带进下一步。

**通过判据**：

| 判据 | 门槛 | 口径与来源 |
| --- | --- | --- |
| ① 运动帧 `requested_target == accepted_target` 占比 | **100%** | 沿用既有口径，2026-11-06 五轮 sidecar 运动帧 `requested == accepted` 全 100%（`902/902`、`800/800`、`909/909`、`804/804`、`906/906`） |
| ② 每关节达成率 | **≥ 70%** | 基线（`--anchor present` 形态）取 `docs/field-results.md` 2026-11-06 节前几行：肩 `shoulder_pan` 10 步 × 1°（settle 2 s 窗、容差 0.05°）为左肩正向 `15.82%`、左肩反向 `36.92%`、右肩 `11.43%`（第二口径分别为 `14.95%` / `33.41%` / `9.67%`）；单步 15 s 长窗下肩 `17.582%`。取 70% 作保守门槛：明显高于基线、又不要求一步到位 |
| ③ 左右任一侧不过 | 整轮不过 → 回查 | 先看 `current_ma` 与步末残余，再决定重跑或转排除法 |

达成率口径沿用 `docs/s4-field-smoke.md` 第 7 节（净行程 ÷ 指令行程）。`--anchor command` 下指令行程按 `abs(delta_deg) × steps` 算成立；`--anchor present` 下相邻 target 的间距会被跟随误差吃掉，不能用 `delta_deg × steps` 当分母（见 `TODO.md` §4 的换算条目）。

上表基线的三个百分数是**10 步 × 1°、settle 2 s 窗、容差 0.05°** 那一场的口径（`docs/field-results.md:45`），与单步 15 s 长窗的 `17.582%`（`:47`）不是同一个窗口，不要混成一个区间。对端量级参照只有口述：用户口述 vendor 遥操作同链路能带动肩部，量级约 100%（**口述，未取仪器证据**，见 `docs/field-results.md` 2026-11-06 节；本轮未跑 teleop oracle）。

`70%` 与 A 阶段的 **≥50%** 不同源，不是同一场的同一门槛：A 阶段判据是 `--anchor present` + 10 步、门槛 `≥50%` 且步末残余 ≤2 tick（`TODO.md` §3.1、本文件 V01 行），本节是 `--anchor command` + 5 步、门槛 `≥70%`。

**证据清单**：

| 产物 | 内容 | 用来证明什么 |
| --- | --- | --- |
| 主 CSV（左右各一份） | 每步的 `cmd_deg` / `measured_deg` / `error_deg` / `settled` | 判据② 的达成率与步末残余 |
| `<主CSV名>-safety.csv` sidecar | 每帧的 `requested_target` / `accepted_target` / `current_ma` | 判据①，以及不过时的回查材料 |
| stdout note | `note: anchor=command` 那行 | 自证本轮锚点，避免与 `present` 的读数混比 |
| scope 记录 | 步末 `Goal_Position − Present_Position` | 与 stdout 的 target 对账 |

跑完拿到上表所列产物才改 V01 的实机证据列，否则 V01 维持现状。

#### 2.5.1 补证结果（2026-11-08 实测，`anchor=command` 形态）

| 判据 | 门槛 | 实测 | 结论 |
| --- | --- | --- | --- |
| ① 运动帧 `requested_target == accepted_target` 占比 | `100%` | 左 `6258/6258`（`100.00%`）、右 `6174/6174`（`100.00%`） | 过 |
| ② 每关节达成率 | `≥ 70%` | 左 `77.3626%`、右 `77.3626%`（各净走 `3.868132°` / 指令 `5.0°`） | 过 |
| ③ 左右任一侧不过 → 整轮不过 | — | 左右同值，无一侧不过 | 过 |

证据目录（仓库外）`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/s4-20261107/`，STAMP `20261108T125734Z`：

| 产物 | 路径 |
| --- | --- |
| 主 CSV（左右各一份） | `tracking/tracking-2.5-{left,right}-anchor-command-20261108T125734Z.csv` |
| sidecar | `tracking/tracking-2.5-{left,right}-anchor-command-20261108T125734Z-safety.csv` |
| stdout | `stdout/7.1-tracking-{left,right}-shoulder-anchor-command-20261108T125734Z.{log,exit}`（含 `note: anchor=command`、`steps_completed=5`、exit `0`） |
| scope | `stdout/scope-20261108T125734Z.md`、`stdout/7.1-checklist-20261108T125734Z.md`、`stdout/scope-append-20261108T125734Z.md` |

**V01 §2.5 补证状态：通过（`anchor=command` 形态）。**

边界（不改主表门槛）：两侧每步 `settled=False`、步末残余 `1.131868°`（`12.9 tick`），远大于主表的 `2 tick`。本轮满足的是本节门槛（达成率 ≥70%），**不是**主表「达成率 ≥50% 且步末残余 ≤2 tick」这一条；主表门槛是否调整见 `TODO.md` §4「V01 主表残余支」。
