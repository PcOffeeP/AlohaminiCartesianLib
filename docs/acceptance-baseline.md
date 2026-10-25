# 验收基线 V01–V13

这份文档给出 13 条准入基线，逐条写清要求、软件证据、实机证据与通过判据。基线回答一个问题：这项能力现在算不算可用。

每格的软件证据都点到具体测试文件与用例名，打开文件能核对到同名函数。没有对应测试的地方直接写「无对应测试」，不写含糊的「已覆盖」。实机证据按现状填写，未取就是「未取」。

## 1 主表

| 编号 | 要求（一句话） | 软件证据（本库现状） | 实机证据（现状） | 判据（什么算过） |
| --- | --- | --- | --- | --- |
| V01 | 脱离遥操作主臂，纯 PC 侧控制 | 无「主臂」专项用例，本库代码里没有 leader 分支。间接证据两条：`tests/test_examples_and_configs.py` 契约 A 的 5 条用例（`test_example_script_exits_zero`、`test_example_script_reports_zero_send_action_calls` 等）用 PC 侧假 transport 把会话整环跑通；`tests/test_control_api.py::test_default_client_factory_without_runtime_raises_runtime_unavailable` 钉住运行时只能由调用方注入 | 未取 | `uv run --no-sync python -m pytest tests/test_examples_and_configs.py` 全绿，示例 exit 0 且 stdout 含 `0 次发送`；真机侧另需一次不接主臂的 `connect` 成功记录 |
| V02 | 双臂与夹爪独立控制，无对侧串值 | `tests/test_control_api.py::test_red_b3_left_move_sends_only_left_arm_keys_and_never_solves_right_ik`、`::test_red_b3_right_move_sends_only_right_arm_keys_and_never_solves_left_ik`、`::test_red_b19_right_gripper_is_side_isolated`；`tests/test_kinematics_adapter.py::test_t_b1_interference_keys_gripper_base_other_arm_do_not_leak`、`::test_t_b8_d_corrupt_other_side_entries_do_not_affect_clean_side` | 未取 | 上述用例通过；真机侧动左臂时右臂的观测值与动作键不变，同一份记录里能读到两侧各自的键 |
| V03 | 机械臂与底盘/升降分阶段互锁 | `tests/test_control_commands.py::test_b21_active_arm_cross_stage_base_request_blocks`、`::test_b22_active_base_lift_cross_stage_arm_request_blocks`；`tests/test_control_controller.py::test_t_c6_1_active_arm_receiving_base_target_blocks_with_zero_send`、`::test_t_c6_3_active_base_lift_receiving_arm_target_blocks_with_zero_send`、`::test_t_c6_4_interlock_is_recoverable_via_state_machine_release_and_settle`；`tests/test_control_api.py::test_red_b17_base_drive_refused_during_arm_motion`、`::test_red_b17_lift_move_to_refused_during_arm_motion` | 未取 | 跨阶段请求被拒，当拍 `send_action` 为 0；释放并等到静止后可恢复 |
| V04 | 停止优先与条件恢复 | `tests/test_control_commands.py::test_b31_stop_command_targets_standby_from_every_state`、`::test_b32_same_tick_motion_and_stop_resolves_to_standby`、`::test_b33_motion_after_stop_command_is_refused_until_resume`、`::test_b91_stop_dominates_resume_and_motion_in_same_tick_batch`、`::test_b92_resume_command_in_standby_runs_unlock_gate` | 未取 | 上述用例通过。语义是：停止命令在任一状态把状态机带到 standby；同一拍内运动与停止并存时结果为 standby；未执行 resume 前再接运动命令被拒 |
| V05 | 反馈异常拦截与时效守护 | `tests/test_control_api.py::test_red_b14_stale_feedback_aborts_move_with_feedback_lost`、`::test_red_b8_missing_settled_valid_is_fail_closed_not_arrival`、`::test_red_c3_b9_frozen_stream_mid_motion_is_not_arrival`；`tests/test_control_commands.py::test_b52_sample_id_regression_from_active_faults`、`::test_t_c12_sample_id_regression_from_non_active_state_faults`；`tests/test_control_controller.py::test_t_c8_1_follow_error_timeout_terminates_with_zero_send`；`tests/benchmarks/test_measure_latency.py::test_run_tracking_steps_should_abort_after_the_feedback_freezes_mid_run` | 未取 | 陈旧或冻结反馈触发 `FeedbackLost` 或 fault，当拍零下发；脚本侧连续 3 次序号不涨判失联并中止（`docs/s4-field-smoke.md` 第 12 节） |
| V06 | 复用底层过流保护并终止目标累积 | `tests/test_control_controller.py::test_t_c4_1_hold_terminates_forward_step_and_reports_overcurrent_hold`、`::test_t_c4_2_hold_clears_target_accumulation_prevents_windup`、`::test_t_c10_4_hold_persists_never_restores_forward_motion`。过流判定本身在厂商固件与运行时一侧，本库只接收 hold 结果，这一段无对应测试 | 未取 | 注入 hold 后前进步终止、目标累积清零、hold 期间不再恢复前进；真机侧另需一次过流事件的记录 |
| V07 | 启动不额外规划机械臂回位 | `tests/test_control_api.py::test_red_b4_connect_does_not_home_or_solve_ik`、`::test_red_b4_first_arm_action_is_zero_displacement_hold`、`::test_red_b4_connect_dry_run_stays_silent` | 未取 | `connect(...)` 不发动作、不调 IK；会话的第一条手臂动作是零位移 hold |
| V08 | 姿态软约束 + 位置残差硬门禁 | `tests/test_arm_ik.py::test_t_ik_6_1_solve_exposes_default_orientation_weight_of_0_01`、`::test_t_ik_6_2_explicit_soft_orientation_weight_keeps_position_and_records_error`、`::test_t_ik_19_4_within_limits_with_excessive_residual_reports_residual`；`tests/test_control_controller.py::test_t_c2_2_ok_true_residual_boundary_is_inclusive`、`::test_t_c2_6_residual_limit_comes_from_profile_not_hardcoded`；`tests/test_control_api.py::test_red_b6_midpath_blocked_point_is_prechecked_before_any_send` | 未取 | 残差超过 `profile.residual_limit_m` 时报 `RESIDUAL` 且零下发；姿态权重默认 0.01，位置优先于姿态 |
| V09 | dry-run 严格零发送 | `tests/test_control_api.py::test_red_b2_move_to_dry_run_returns_step_diagnostics_and_never_sends`、`::test_red_c7_dry_run_gripper_never_sends`、`::test_red_c7_dry_run_lift_never_sends`、`::test_red_c7_dry_run_base_drive_and_stop_never_send`、`::test_red_c7_dry_run_disconnect_never_sends`；`tests/test_control_controller.py::test_t_c3_4_dry_run_full_lifecycle_never_sends`；`tests/test_examples_and_configs.py` 契约 A 与契约 D 用子进程 stdout 的 `0 次发送` 取数，不靠源码常量断言 | 未取（dry-run 不碰设备，这一类没有实机侧证据） | 上述用例通过；示例子进程 stdout 报告 `send_action` 调用次数为 0，且同一会话确实读过观测 |
| V10 | 两道 Gate 资格门禁 | `tests/benchmarks/test_measure_latency.py::test_main_should_refuse_tracking_when_the_host_reports_another_robot_model`（Gate C）、`::test_main_should_refuse_tracking_when_the_metadata_schema_is_not_two`（Gate F 第一段）、`::test_main_should_refuse_tracking_when_the_joint_units_cannot_be_resolved`（Gate F 第二段）；`tests/benchmarks/test_capture_robot_fingerprint.py::test_main_should_reject_invalid_metadata`（指纹侧同一判据） | 部分。Pi 上已取得一次代码 Gate F 的真实拒绝，报文原文 `Gate F refused: the host metadata schema_version is 1, expected 2.`，见 `docs/s4-field-smoke.md` 第 2 节与第 6 节。机器人侧按臂归档的 Gate A–E 机制本库没有，由本基线替代 | 上述用例通过；真机侧一条拒绝记录要同时含命令原文、stderr 原文与退出码 `6`，且 `send_action` 为 0 |
| V11 | 既有系统兼容（v1 帧平滑回退） | `tests/test_capability.py::test_detect_parses_v1_metadata_and_records_missing_field_notes`、`::test_detect_fills_missing_motor_fields_with_none`、`::test_schema1_notes_still_name_missing_fields`。schema 1 的解析与缺字段降级有测试；帧平滑回退无对应测试 | 未取。Pi 当前 vendor 广播 `schema_version = 1`，正好是这条要兼容的版本，但本库没有对该版本跑过整环 | schema 1 的 metadata 解析不抛错，`homing_offset` 与 `resolution` 为 `None`，`notes` 点名缺失字段；帧平滑回退这条要补用例之后才可判 |
| V12 | 运动学单位与限位交集准确 | `tests/test_kinematics_adapter.py::test_t_b6_b_one_sided_narrowing_bounds_from_urdf_and_cal`、`::test_t_b6_c_empty_intersection_rejects_every_conversion`、`::test_t_b6_e_zero_width_urdf_limit_rejects_every_conversion`、`::test_t_b6_h_bus_silent_clamp_pre_rejected_with_oracle`；`tests/test_capability.py::test_resolve_uses_metadata_fields_first`、`::test_resolve_falls_back_to_calibration_file`、`::test_resolve_span_by_normalization`；`tests/test_control_controller.py::test_t_c11_1_solve_contract_seed_shape_and_adapter_intersection_limits`、`::test_t_c11_2_solve_limits_track_injected_adapter_windows_not_placeholder` | 部分。现场标定文件已取得：`/Users/pcoffeep/pi-workspace/alohamini-cartesian-teleop/evidence/calibration-AlohaMiniRobot.json`，18 个电机每个含 `id`、`drive_mode`、`homing_offset`、`range_min`、`range_max`；`resolution` 不在该文件里，来自 `kinematics/model_specs.py` 的型号表。取得过程见 `docs/s4-field-smoke.md` 第 8 节 `4.7-calib`。`schema_version = 2` 的 metadata 指纹未取，`1.1-capture` 没有写出 JSON，见同文第 5 与第 13 节 | 上述用例通过；真机侧需一份 `schema_version = 2` 的 fingerprint JSON，逐电机 8 字段齐备，且用它算出的 `deg_per_unit` 与标定文件的 range 一致 |
| V13 | IK 独立解耦、实时无卡顿 | `tests/test_arm_ik.py::test_t_ik_12_1_module_imports_with_placo_absent`、`::test_t_ik_12_5_module_has_no_client_host_or_network_imports`、`::test_t_ik_13_1_success_result_exposes_sane_timing`；`tests/benchmarks/test_benchmark_arm_ik.py::test_t_ik_16_2_budget_judgement_accepts_values_within_the_closed_bounds`、`::test_t_ik_16_3_budget_judgement_rejects_marginal_overrun_on_either_axis`、`::test_t_ik_14_4_step_p99_is_finite_positive_and_exceeds_solve_p99` | 未取 | `control/arm_ik.py` 不 import 客户端、Host、网络模块；离线基准的 `solve_p99_ms` 与 `step_p99_ms` 落在预算常数内，`within_budget` 返回 True |

## 2 使用说明

### 2.1 这份基线替代什么

替代旧仓库那份「按臂归档的 Gate A–E」准入记录。

为什么替代：Gate A–E 是机器人运行时一侧的逐臂检查项，事实来源在厂商运行时与现场标定。本库是产品侧，只通过 `adapter/protocol.py` 两个 `Protocol` 与对端接触，拿不到也不需要那些归档。原来用 Gate 归档来判断「能不能放出这一步」，现在改用这份基线，把每条要求对到本库可复现的软件证据和现场能取到的实机证据上。

注意区分两处同名的 Gate：本文说的 Gate A–E 指旧归档；`docs/s4-field-smoke.md` 第 1 节那六道 Gate A–F 是 `benchmarks/alohamini_io/measure_latency.py` 里的运行时门禁，是代码里的检查，不是归档。

### 2.2 什么时候算关闭

一条算关闭，要同时满足三件事：

1. 软件证据列填的是可点开核对的测试文件名与用例名，且这些用例在本机实跑通过。
2. 实机证据列填的是可复查的记录：命令原文、输出原文、退出码、产物路径，缺一项不算。
3. 判据列写成可执行的形式，任何人照做能复现同一结论。写「看起来正常」「基本可用」的判据不算关闭。

两列都填实之后，把表中该行的实机证据从「未取」改成记录路径，并在本节汇总里改数。

### 2.3 现状汇总

逐条统计，与本文件第 1 节一致：

| 维度 | 有证据 | 部分 | 未取 | 合计 |
| --- | --- | --- | --- | --- |
| 软件侧 | 10 条（V02、V03、V04、V05、V07、V08、V09、V10、V12、V13） | 3 条（V01、V06、V11） | 0 条 | 13 |
| 实机侧 | 0 条 | 2 条（V10、V12） | 11 条 | 13 |

「部分」的含义：有专项用例或现场记录，但只覆盖该条要求的一部分，缺口在对应格子里写明了。

软件侧整体快照：`uv run --no-sync python -m pytest tests` 报 `1074 passed, 2 skipped`。跳过的 2 条是两个 `lerobot` parity 文件（`tests/test_control_api_lerobot_parity.py`、`tests/test_kinematics_lerobot_parity.py`），原因是本机没装 `lerobot`。这 2 条跳过不等于全部通过。

实机侧已取的两条都只是部分：

- V10 取到的是 Gate F 拒绝路径，Gate C 在真机上的表现未取。
- V12 取到的是标定文件，metadata 侧的 `schema_version = 2` 指纹未取，P0 仍阻塞。
