#!/usr/bin/env python

"""AlohaMini 笛卡尔控制器 (CartesianController) 的行为测试。

从上游 lerobot 仓库搬运的 T-C1..T-C18 用例集: 步长/残差门禁、dry_run、阶段互锁、
过流 hold 退让、领先夹紧、跟随超时、错误词表、安全间隔 deadline、真 ArmIK 集成。

ArmIK 数值用例依赖可选依赖 placo, 缺失时跳过。
"""

# Behavior: 实施包 P4-B cycle 1 —— T-C1 步长门禁 (design §4「运动连续性」,
# 控制层内核 controller.py). 模块
# `alohamini_cartesian.control.controller` 尚不存在, 所以每个用例在 helper 内
# lazy import (与 tests/robots/test_alohamini_control_commands.py、
# tests/robots/test_alohamini_arm_ik.py 相同): 模块缺失时逐例 ImportError, 而不是整
# 模块 collection 失败.
#
# Required behavior (acceptance, 本 cycle 冻结):
# - 在 READY+ACTIVE(ARM) 状态下调用 `CartesianController.tick(feedback, target)`:
#   * ScriptedIK 返回相对参考解 <= max_joint_step_deg 的解 -> 接受, 经 adapter 转成
#     底层动作后恰发送一次; CycleOutcome.sent=True, error_code=None;
#   * 超限 -> 拒绝: 不 clamp、不发送, sent=False, action=None,
#     error_code=ControlError.STEP_LIMIT;
#   * 边界: 恰好 max_joint_step_deg 通过, 略大 (2.0 通过 / 2.0001 拒绝);
# - 步长语义: 参考解为「上一被接受命令」的关节角; 首个周期尚无被接受命令时参考为当
#   周期反馈 (经 adapter 转 kinematic q) —— 否则「首周期大跳变」无法被判定;
# - 被拒绝的 IK 解不得污染参考 (design §4 失败隔离): 下一周期仍与上一「被接受」解比较;
# - dry_run=True 时 sender 调用数恒为 0;
# - feedback = 原始 observation dict (含 `_robot_metadata`) + `_control_status`;
#   反馈经 adapter 转 kinematic q 作为 IK seed.
#
# 本 binding 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - 直接转发 IK 输出, 不经 adapter 转换 (T-C1.8: 非单位 adapter, 发送动作必须等于独立
#   算出的 affine `sign*(q-zero)`; 直接转发 q 或忽略 sign/zero 都会 fail);
# - 超限时 clamp 到步长上限并继续发送 (T-C1.7: 断言 sender 调用数为 0 且 action is None);
# - 硬编码 2.0 门限, 忽略注入的 max_joint_step_deg (T-C1.6: 5.0 门限下 3.0 跳变必须
#   接受; 0.5 门限下 0.5001 必须拒绝);
# - 每个周期都拿反馈(而非上一被接受命令)当参考 (T-C1.4);
# - 被拒绝的解覆盖参考 (T-C1.5);
# - 不校验状态机 ACTIVE 就发送 (T-C1.10).
#
# 冻结 seam (相对任务已裁定 seam 的细化, 待主 Agent 固化):
#   module : alohamini_cartesian.control.controller
#   class  : CartesianController(
#                state_machine=ControlStateMachine, arm_ik=<solve 协议替身>,
#                adapter=JointKinematicAdapter, side="left"|"right",
#                clock=Callable[[], float], sender=Callable[[Mapping], None],
#                dry_run=bool, max_joint_step_deg=2.0, max_joint_lead_deg=<必填>,
#                follow_error_tolerance_deg=<必填>, follow_error_timeout_s=<必填>)
#   entry  : .tick(feedback: Mapping, target) -> CycleOutcome
#   data   : CycleOutcome = frozen dataclass {sent: bool,
#                action: Mapping[str, float] | None, error_code: ControlError | None,
#                detail: str | None}
#   enum   : ControlError (controller 层), 至少含 STEP_LIMIT; 接受路径 error_code=None
#   feedback: `{arm_<side>_<joint>.pos, ..., "_robot_metadata": v2, "_control_status": {...}}`
#             —— 控制器从 feedback 取 metadata 做 adapter 转换 (见报告 R1/R2)
#   `arm_ik.solve(...)` 以位置或关键字参数接收 (q_seed, target_pose, joint_limits_deg[,...]);
#   本 cycle 不约束 joint_limits_deg 的来源 (adapter 无公开 limits 访问器, 见报告 R3).
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件. 期望值为冻结
# 字面量, 不从被测实现回读.

import dataclasses
from enum import Enum
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

# 冻结的运动学关节顺序 (与 joint_kinematic_adapter.JOINT_ORDER 相同, 在此作为字面量).
JOINT_ORDER: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)

CTRL_SIDE = "left"
CTRL_MODEL = "am_arm200"

# 非单位 affine (sign 混合 ±1, zero 逐关节互异且非 0), 使「直接转发 IK q」与「经
# adapter 转换」在数值上可区分. 期望发送动作在测试内独立算出, 不从 adapter 回读.
CTRL_SIGNS: dict[str, float] = {
    joint: (1.0 if i % 2 == 0 else -1.0) for i, joint in enumerate(JOINT_ORDER)
}
CTRL_ZEROS: dict[str, float] = {joint: float(10 * (i + 1)) for i, joint in enumerate(JOINT_ORDER)}

# 冻结的「当前位姿」kinematic 关节角 (deg). 无 urdf_limits 注入时 adapter 在 degrees
# 模式下不限幅, 因此大跳变也不会被 adapter 自身拒绝, 门禁只能来自被测控制器.
BASE_Q: tuple[float, ...] = (10.0, -20.0, 30.0, -40.0, 50.0, -60.0)

# 冻结目标位姿 (4x4). 本 cycle 只验证门禁, 不约束 target 语义.
TARGET = np.eye(4)


# ---------------------------------------------------------------------------
# Lazy seam loader: 缺模块时逐例 ImportError.
# ---------------------------------------------------------------------------
def load_controller() -> SimpleNamespace:
    """Import the P4-B controller seam lazily so a missing module fails per case."""
    from alohamini_cartesian.control.controller import (
        CartesianController,
        ControlError,
        CycleOutcome,
    )

    return SimpleNamespace(
        CartesianController=CartesianController,
        ControlError=ControlError,
        CycleOutcome=CycleOutcome,
    )


# ---------------------------------------------------------------------------
# 冻结的 test doubles / fixtures (无真实时钟、无 IO、无随机).
# ---------------------------------------------------------------------------
class FakeClock:
    """Injectable clock; time only moves when a test advances it explicitly."""

    def __init__(self, start: float = 0.0) -> None:
        """初始化注入时钟及其起始时刻。"""
        self._now = start

    def __call__(self) -> float:
        """返回当前注入时间。"""
        return self._now

    def advance(self, seconds: float) -> None:
        """把注入时间显式推进 seconds 秒。"""
        self._now += seconds


class RecordingSender:
    """sender 替身: 记录每次动作快照 (拷贝, 后续修改不影响证据)."""

    def __init__(self) -> None:
        """初始化空的调用记录。"""
        self.calls: list[dict] = []

    def __call__(self, action: dict[str, float]) -> None:
        """记录一次动作快照（拷贝，后续修改不影响证据）。"""
        self.calls.append(dict(action))


class ScriptedIK:
    """arm_ik 协议替身: 记录 solve 入参, 按脚本顺序返回预置 IKResult.

    调用形态对位置/关键字参数都容忍 (positional 或
    ``q_seed_deg``/``q_current_deg`` 关键字), 因为 P4-A 的 `ArmIK.solve` 与设计文
    档对该参数命名不一致. 脚本耗尽即 AssertionError, 使「多算一次」也可见.

    P4-B cycle 2 追加: 暴露 ``profile`` (与真实 ``ArmIK`` 一致), 使控制器可从
    ``arm_ik.profile.residual_limit_m`` 读取残差门限 (acceptance: 「position_error_m
    > profile.residual_limit_m」). 未显式传入时保持与设计默认值一致的 0.005.
    """

    def __init__(self, results: object, *, residual_limit_m: float = 0.005) -> None:
        """初始化脚本队列与 profile 残差门限。"""
        self._results = list(results)
        self.calls: list[dict] = []
        # Mirrors the real ArmIK surface (`self.profile = <ArmKinematicProfile>`).
        self.profile = SimpleNamespace(residual_limit_m=float(residual_limit_m))

    def solve(self, *args: object, **kwargs: object) -> object:
        """记录入参，并按脚本顺序返回下一个 IKResult。"""
        self.calls.append({"args": tuple(args), "kwargs": dict(kwargs)})
        if not self._results:
            raise AssertionError("ScriptedIK exhausted: controller solved more than scripted")
        return self._results.pop(0)


# ---------------------------------------------------------------------------
# Frozen expectation helpers.
# ---------------------------------------------------------------------------
def expected_action(q_deg: object) -> dict[str, float]:
    """Independent affine expectation: motor = sign * (q - zero) (DEGREES mode)."""
    q = np.asarray(q_deg, dtype=float)
    return {
        f"arm_{CTRL_SIDE}_{joint}.pos": CTRL_SIGNS[joint] * (q[i] - CTRL_ZEROS[joint])
        for i, joint in enumerate(JOINT_ORDER)
    }


def raw_forward(q_deg: object) -> dict[str, float]:
    """The WRONG 'forward the IK q verbatim' output (kept to prove discrimination)."""
    q = np.asarray(q_deg, dtype=float)
    return {f"arm_{CTRL_SIDE}_{joint}.pos": float(q[i]) for i, joint in enumerate(JOINT_ORDER)}


def q_delta(delta: float, index: int = 0) -> np.ndarray:
    """DEGREES 空间基准关节角，仅第 index 个关节偏移 delta。"""
    q = np.array(BASE_Q, dtype=float)
    q[index] += delta
    return q


def ik_ok(q_deg: object, *, position_error_m: float = 0.0) -> object:
    """冻结的成功 IK 结果 (ok=True)，数值由给定关节角独立构造。"""
    from alohamini_cartesian.control.arm_ik import IKResult

    return IKResult(
        q_deg=np.asarray(q_deg, dtype=float),
        ok=True,
        reason=None,
        position_error_m=position_error_m,
        orientation_error_deg=0.0,
        iterations=2,
        solve_time_s=0.0002,
    )


def ik_fail(reason: str, q_deg: object = None, *, position_error_m: float = float("inf")) -> object:
    """Frozen IK failure result (ok=False with an explicit design reason)."""
    from alohamini_cartesian.control.arm_ik import IKResult

    q = np.array(BASE_Q, dtype=float) if q_deg is None else np.asarray(q_deg, dtype=float)
    return IKResult(
        q_deg=q,
        ok=False,
        reason=reason,
        position_error_m=position_error_m,
        orientation_error_deg=0.0,
        iterations=1,
        solve_time_s=0.0002,
    )


def make_adapter(side: str = CTRL_SIDE) -> object:
    """接入冻结非单位 affine 的真实 JointKinematicAdapter。"""
    from alohamini_cartesian.kinematics.joint_adapter import JointKinematicAdapter

    return JointKinematicAdapter(
        signs={side: dict(CTRL_SIGNS)},
        zeros={side: dict(CTRL_ZEROS)},
    )


def make_metadata(side: str = CTRL_SIDE) -> dict:
    """v2 元数据：degrees 模式下的量程容得下冻结关节角。"""
    motors = {}
    for i, joint in enumerate(JOINT_ORDER):
        motors[f"arm_{side}_{joint}"] = {
            "id": i + 1,
            "model": f"syn-ctrl-{i + 1}",
            "normalization": "degrees",
            "drive_mode": 1 if i % 2 else 0,
            "range_min": 41 + i,
            "range_max": 4001 + i,
            "homing_offset": 100 * (i + 1) - 500,
            "resolution": 6399 + i,
        }
    return {"schema_version": 2, "robot_model": CTRL_MODEL, "motors": motors}


def observation_for_q(q_deg: object, side: str = CTRL_SIDE) -> dict[str, float]:
    """Motor-degree observation whose adapter forward map yields exactly q_deg."""
    q = np.asarray(q_deg, dtype=float)
    return {
        f"arm_{side}_{joint}.pos": CTRL_SIGNS[joint] * (q[i] - CTRL_ZEROS[joint])
        for i, joint in enumerate(JOINT_ORDER)
    }


def make_feedback(q_deg: object, *, side: str = CTRL_SIDE, sample_id: int = 1) -> dict:
    """Feedback = raw observation dict + `_robot_metadata` + `_control_status`."""
    feedback = observation_for_q(q_deg, side)
    feedback["_robot_metadata"] = make_metadata(side)
    feedback["_control_status"] = {
        "schema_version": 1,
        "host_session_id": "test-session",
        "sample_sequence": sample_id,
        "joint_holds": {},
        "gripper_holds": {},
    }
    return feedback


def make_controller(
    results: object,
    *,
    step_deg: float = 2.0,
    dry_run: bool = False,
    active: bool = True,
    side: str = CTRL_SIDE,
    max_joint_step_kwarg: bool = True,
    stage: str = "arm",
    residual_limit_m: float = 0.005,
) -> SimpleNamespace:
    """构造被测控制器: 状态机推进到 READY (+ACTIVE/ARM|BASE_LIFT), 注入全部替身与参数.

    ``stage`` 控制进入 ACTIVE 时激活的阶段: ``"arm"`` (ArmCommand) 或
    ``"base"`` (零速 BaseCommand, 对应 D014 底盘停车语义涉及处).
    ``residual_limit_m`` 注入到 arm_ik 替身的 ``profile`` 上, 用于验证残差门限
    来自 profile 而非硬编码 5 mm.
    """
    from alohamini_cartesian.control.commands import (
        ArmCommand,
        BaseCommand,
        ControlStateMachine,
        Side,
    )

    mod = load_controller()
    adapter = make_adapter(side)
    ik = ScriptedIK(results, residual_limit_m=residual_limit_m)
    sender = RecordingSender()

    machine = ControlStateMachine(settled_samples=1)
    machine.on_feedback(1, settled=True)
    machine.resume()
    if active:
        if stage == "arm":
            side_enum = Side.LEFT if side == "left" else Side.RIGHT
            machine.handle(ArmCommand(side=side_enum, dx=0.01, dy=0.0, dz=0.0))
        elif stage == "base":
            # 零时长 + 零速度 = 纯数据形式的底盘停车/静止请求 (D017 合法), 进入
            # ACTIVE(BASE_LIFT) 以验证控制器的阶段互锁发送门禁.
            machine.handle(BaseCommand(x=0.0, y=0.0, turn=0.0, seconds=0.0))
        else:
            raise ValueError(f"unknown stage {stage!r}")

    kwargs = {
        "state_machine": machine,
        "arm_ik": ik,
        "adapter": adapter,
        "side": side,
        "clock": FakeClock(),
        "sender": sender,
        "dry_run": dry_run,
        "max_joint_lead_deg": 30.0,
        "follow_error_tolerance_deg": 1.0,
        "follow_error_timeout_s": 1.0,
    }
    if max_joint_step_kwarg:
        kwargs["max_joint_step_deg"] = step_deg

    controller = mod.CartesianController(**kwargs)
    return SimpleNamespace(
        controller=controller,
        ik=ik,
        sender=sender,
        adapter=adapter,
        machine=machine,
        mod=mod,
    )


def extract_seed(call: dict) -> np.ndarray:
    """Recover the IK seed argument regardless of positional/keyword calling style."""
    args = call["args"]
    kwargs = call["kwargs"]
    if args:
        return np.asarray(args[0], dtype=float)
    for key in ("q_seed_deg", "q_current_deg", "q_deg", "seed_deg"):
        if key in kwargs:
            return np.asarray(kwargs[key], dtype=float)
    raise AssertionError(f"no IK seed found in solve call: {call}")


def assert_action_close(actual: dict[str, float], expected: dict[str, float]) -> None:
    """断言发送动作与独立算出的 affine 期望完全一致。"""
    assert set(actual) == set(expected)
    for key, value in expected.items():
        assert actual[key] == pytest.approx(value, abs=1e-9), f"key {key!r}"


# ---------------------------------------------------------------------------
# T-C1.0 — frozen seam: CycleOutcome shape, ControlError vocabulary, constructibility.
# ---------------------------------------------------------------------------
def test_t_c1_0_frozen_cycle_outcome_and_error_vocabulary_and_construction() -> None:
    """冻结 seam：CycleOutcome 形状、ControlError 词表与可构造性。"""
    mod = load_controller()

    assert dataclasses.is_dataclass(mod.CycleOutcome)
    assert {f.name for f in dataclasses.fields(mod.CycleOutcome)} == {
        "sent",
        "action",
        "error_code",
        "detail",
    }
    outcome = mod.CycleOutcome(sent=False, action=None, error_code=None, detail=None)
    with pytest.raises(dataclasses.FrozenInstanceError):
        outcome.sent = True  # type: ignore[misc]

    assert issubclass(mod.ControlError, Enum)
    assert "STEP_LIMIT" in mod.ControlError.__members__

    # 完整 seam 可构造, 且 max_joint_step_deg 有默认 (未显式传入).
    ctx = make_controller([ik_ok(BASE_Q)], max_joint_step_kwarg=False)
    assert ctx.controller is not None


# ---------------------------------------------------------------------------
# T-C1.1 — 合法小步解 (<= 门限) 被接受, 恰发送一次, 动作经 adapter 转换.
# ---------------------------------------------------------------------------
def test_t_c1_1_subthreshold_solution_is_accepted_and_sent_through_adapter() -> None:
    """合法小步解被接受，恰发送一次，且动作经 adapter 转换。"""
    q_sol = q_delta(1.0)
    ctx = make_controller([ik_ok(q_sol)])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is True
    assert outcome.error_code is None
    assert outcome.action is not None
    assert_action_close(outcome.action, expected_action(q_sol))

    assert len(ctx.sender.calls) == 1
    assert_action_close(ctx.sender.calls[0], expected_action(q_sol))

    # 反馈经 adapter 转 kinematic q 作为 IK seed (首周期).
    assert len(ctx.ik.calls) == 1
    assert extract_seed(ctx.ik.calls[0]) == pytest.approx(np.asarray(BASE_Q), abs=1e-9)


# ---------------------------------------------------------------------------
# T-C1.2 — 首周期大跳变解被拒绝, 零发送.
# ---------------------------------------------------------------------------
def test_t_c1_2_first_tick_big_jump_is_rejected_with_zero_send() -> None:
    """首周期大跳变解被拒绝，且零发送。"""
    ctx = make_controller([ik_ok(q_delta(50.0))])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code is ctx.mod.ControlError.STEP_LIMIT
    assert ctx.sender.calls == []
    assert len(ctx.ik.calls) == 1


# ---------------------------------------------------------------------------
# T-C1.3 — 边界: 恰好 2.0° 通过, 2.0001° 拒绝 (含下方安全区).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("delta", "accepted"),
    [(1.9999, True), (2.0, True), (2.0001, False), (2.5, False)],
)
def test_t_c1_3_threshold_boundary_is_inclusive(delta: float, accepted: bool) -> None:
    """步长边界含等号：恰好 2.0° 通过，2.0001° 拒绝。"""
    ctx = make_controller([ik_ok(q_delta(delta))])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    if accepted:
        assert outcome.sent is True
        assert outcome.error_code is None
        assert len(ctx.sender.calls) == 1
    else:
        assert outcome.sent is False
        assert outcome.action is None
        assert outcome.error_code is ctx.mod.ControlError.STEP_LIMIT
        assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C1.4 — 参考是上一「被接受命令」, 不是每周期反馈.
# 若实现拿反馈当参考, 第 2 周期相对反馈 3.0° > 2.0° 会被误拒 -> 发送数变 1.
# ---------------------------------------------------------------------------
def test_t_c1_4_reference_is_last_accepted_command_not_feedback() -> None:
    """步长参考是上一被接受命令，而非当周期反馈。"""
    ctx = make_controller([ik_ok(q_delta(1.0)), ik_ok(q_delta(3.0)), ik_ok(q_delta(5.0001))])
    feedback = make_feedback(BASE_Q)  # 反馈恒在 BASE_Q (不随接受命令变化)

    o1 = ctx.controller.tick(feedback, TARGET)
    o2 = ctx.controller.tick(feedback, TARGET)
    o3 = ctx.controller.tick(feedback, TARGET)

    assert (o1.sent, o2.sent, o3.sent) == (True, True, False)
    assert o3.error_code is ctx.mod.ControlError.STEP_LIMIT
    assert len(ctx.sender.calls) == 2
    assert len(ctx.ik.calls) == 3


# ---------------------------------------------------------------------------
# T-C1.5 — 被拒绝的解不污染参考 (design §4 失败隔离).
# 若实现用被拒解更新参考, 第 3 周期相对 base+50 的 46° 会被误拒.
# ---------------------------------------------------------------------------
def test_t_c1_5_rejected_solution_does_not_pollute_reference() -> None:
    """被拒绝的解不得污染步长参考（失败隔离）。"""
    ctx = make_controller([ik_ok(q_delta(2.0)), ik_ok(q_delta(50.0)), ik_ok(q_delta(4.0))])
    feedback = make_feedback(BASE_Q)

    o1 = ctx.controller.tick(feedback, TARGET)
    o2 = ctx.controller.tick(feedback, TARGET)
    o3 = ctx.controller.tick(feedback, TARGET)

    assert (o1.sent, o2.sent, o3.sent) == (True, False, True)
    assert o2.error_code is ctx.mod.ControlError.STEP_LIMIT
    assert len(ctx.sender.calls) == 2


# ---------------------------------------------------------------------------
# T-C1.6 — 门限来自注入参数, 不是硬编码 2.0.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("step_deg", "delta", "accepted"),
    [(5.0, 3.0, True), (2.0, 3.0, False), (0.5, 0.5, True), (0.5, 0.5001, False)],
)
def test_t_c1_6_threshold_is_injected_not_hardcoded(
    step_deg: float, delta: float, accepted: bool
) -> None:
    """步长门限来自注入参数，不是硬编码 2.0。"""
    ctx = make_controller([ik_ok(q_delta(delta))], step_deg=step_deg)

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is accepted
    if accepted:
        assert outcome.error_code is None
    else:
        assert outcome.error_code is ctx.mod.ControlError.STEP_LIMIT


# ---------------------------------------------------------------------------
# T-C1.7 — 超限是拒绝而不是 clamp: 不允许发送任何(哪怕夹紧过的)动作.
# ---------------------------------------------------------------------------
def test_t_c1_7_over_limit_is_rejected_not_clamped() -> None:
    """超限是拒绝而不是 clamp：不得发送任何被夹紧的动作。"""
    ctx = make_controller([ik_ok(q_delta(50.0))])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert ctx.sender.calls == []
    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code is ctx.mod.ControlError.STEP_LIMIT
    assert outcome.detail is None or isinstance(outcome.detail, str)


# ---------------------------------------------------------------------------
# T-C1.8 — 接受路径发送的是 adapter 输出, 不是原始 IK q (direct-forward 辨识).
# ---------------------------------------------------------------------------
def test_t_c1_8_sent_action_is_adapter_output_not_raw_ik_q() -> None:
    """接受路径发送 adapter 输出，而非原始 IK q。"""
    q_sol = q_delta(1.0)
    ctx = make_controller([ik_ok(q_sol)])

    ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    expected = expected_action(q_sol)
    wrong = raw_forward(q_sol)
    assert expected != wrong  # adapter 必须真的改变数值 (非单位 affine)
    assert_action_close(ctx.sender.calls[0], expected)
    assert ctx.sender.calls[0] != wrong


# ---------------------------------------------------------------------------
# T-C1.9 — dry_run: 决策仍为接受 (action 给出拟发送动作), 但 sender 调用数恒 0.
# ---------------------------------------------------------------------------
def test_t_c1_9_dry_run_never_calls_sender() -> None:
    """dry_run 下决策仍为接受，但 sender 调用数恒为 0。"""
    q_sol = q_delta(1.0)
    ctx = make_controller([ik_ok(q_sol)], dry_run=True)

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert ctx.sender.calls == []
    assert outcome.sent is False
    assert outcome.error_code is None
    assert outcome.action is not None
    assert_action_close(outcome.action, expected_action(q_sol))


# ---------------------------------------------------------------------------
# T-C1.10 — 非 ACTIVE (仅 READY) 时不得发送任何动作.
# ---------------------------------------------------------------------------
def test_t_c1_10_no_send_when_state_machine_is_not_active() -> None:
    """状态机仅 READY 未 ACTIVE 时不得发送任何动作。"""
    ctx = make_controller([ik_ok(q_delta(1.0))], active=False)

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert ctx.sender.calls == []
    assert outcome.sent is False
    assert outcome.error_code is not None
    assert outcome.error_code is not ctx.mod.ControlError.STEP_LIMIT


# ===========================================================================
# P4-B cycle 2 —— T-C2 残差门限 + T-C3 dry-run 发送门禁 (appended; existing 17 kept)
# ===========================================================================
#
# Required behavior (acceptance, 本 cycle 冻结):
# - T-C2 IK 失败门禁: `arm_ik.solve` 返回 ok=False (reason ∈ RESIDUAL/JOINT_LIMIT/
#   NON_FINITE) -> 拒绝、零发送、action=None, 且 error_code 映射该 IK reason
#   (ControlError 需提供与 reason 同名成员: RESIDUAL/JOINT_LIMIT/NON_FINITE);
# - T-C2 ok=True 残差门禁: position_error_m > profile.residual_limit_m -> 同样拒绝
#   (error_code=RESIDUAL, 零发送); 恰好 5.0 mm (默认 profile) 必须通过;
# - T-C2 被拒周期不得污染步长参考 (与 T-C1.5 的失败隔离一致);
# - T-C2 残差门限取自 arm_ik.profile.residual_limit_m (非硬编码 5 mm).
# - T-C3 dry-run 全路径: dry_run=True 下完整生命周期 (进入 ACTIVE -> 多周期运动 ->
#   stop -> 退出) sender 调用恒 0; stop/退出/非 ARM 阶段 (BASE_LIFT, D014 底盘停车
#   语义涉及处) 也一律不发机械臂动作.
#
# 冻结子接缝 (acceptance 字面「profile.residual_limit_m」; 待主 Agent 固化):
#   - 残差门限读取自 `arm_ik.profile.residual_limit_m` (与真实 ArmIK 一致,
#     ArmKinematicProfile.residual_limit_m 默认 0.005); 测试替身 ScriptedIK 暴露
#     `profile` 以注入自定义门限.
#   - ControlError 成员名与 IKResult.reason 同名 (RESIDUAL/JOINT_LIMIT/NON_FINITE).
#
# 本 cycle 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - 把所有 IK 失败坍缩为通用 IK_FAILED, 忽略 reason (T-C2.1);
# - 只信 ArmIK 的 ok 标志, 不做控制器侧 ok=True 残差复检 (T-C2.2/T-C2.4/T-C2.6);
# - 用被拒 (残差/IK 失败) 的解覆盖步长参考 (T-C2.4);
# - 硬编码 0.005 门限, 忽略 profile 注入值 (T-C2.6);
# - 发送门禁只看 state==ACTIVE, 不校验 active_stage==ARM (T-C3.5);
# - dry_run 仅在首周期生效, 后续生命周期/stop 路径泄漏发送 (T-C3.1/T-C3.4).
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.

RESIDUAL_LIMIT_M = 0.005  # 与 ArmKinematicProfile 默认值 / design 5 mm 门限一致


# ---------------------------------------------------------------------------
# T-C2.1 — ok=False 的 IK 失败: 拒绝、零发送, error_code 映射 IK reason.
# 若实现坍缩为通用 IK_FAILED (或忽略 reason), error_code.name != reason.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("reason", ("RESIDUAL", "JOINT_LIMIT", "NON_FINITE"))
def test_t_c2_1_ik_failure_reason_maps_to_error_code(reason: str) -> None:
    """IK 失败时拒绝、零发送，error_code 映射 IK reason。"""
    ctx = make_controller([ik_fail(reason, q_delta(0.5))])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is False
    assert outcome.action is None
    assert ctx.sender.calls == []
    assert outcome.error_code is not None
    assert outcome.error_code is not ctx.mod.ControlError.STEP_LIMIT
    # error_code 必须能区分 RESIDUAL / JOINT_LIMIT / NON_FINITE, 而非统一 IK_FAILED.
    assert outcome.error_code.name == reason, (outcome.error_code, reason)
    assert len(ctx.ik.calls) == 1


# ---------------------------------------------------------------------------
# T-C2.2 — ok=True 但 position_error_m 超残差门限 -> 拒绝; 恰好 5.0 mm 通过.
# 若控制器不做残差复检, 超限解会被当作接受并发送 (RED).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("residual_m", "accepted"),
    [
        (0.0, True),
        (0.0049999, True),
        (0.005, True),  # 边界: 恰好 5.0 mm 必须通过 (inclusive)
        (0.0050001, False),
        (0.006, False),
    ],
)
def test_t_c2_2_ok_true_residual_boundary_is_inclusive(residual_m: float, accepted: bool) -> None:
    """残差边界含等号：恰好 5.0 mm 通过，略超即拒绝。"""
    q_sol = q_delta(1.0)
    ctx = make_controller([ik_ok(q_sol, position_error_m=residual_m)])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    if accepted:
        assert outcome.sent is True
        assert outcome.error_code is None
        assert_action_close(ctx.sender.calls[0], expected_action(q_sol))
    else:
        assert outcome.sent is False
        assert outcome.action is None
        assert outcome.error_code is ctx.mod.ControlError.RESIDUAL
        assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C2.4 — 被拒周期不得污染步长参考 (残差 / IK 失败两种来源).
# S1=+1.9 deg 被接受; S2=+3.9 deg 距 S1 恰 2.0 deg (步长门限 inclusive, 只有残差/
# IK 门禁能拒它); S3=0.0 deg 距 S1 1.9 deg (通过) 但距 S2 3.9 deg (若参考被污染则拒).
# 实现若用被拒的 S2 覆盖参考, o3 会变成拒绝.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("failing", ["ik_false", "high_residual"])
def test_t_c2_4_rejected_cycle_does_not_pollute_reference(failing: str) -> None:
    """被拒周期（残差或 IK 失败）不得污染步长参考。"""
    s1 = q_delta(1.9)
    s2 = q_delta(3.9)
    s3 = q_delta(0.0)
    failing_result = (
        ik_fail("RESIDUAL", s2)
        if failing == "ik_false"
        else ik_ok(s2, position_error_m=RESIDUAL_LIMIT_M + 0.001)
    )
    ctx = make_controller([ik_ok(s1), failing_result, ik_ok(s3)])
    feedback = make_feedback(BASE_Q)

    o1 = ctx.controller.tick(feedback, TARGET)
    o2 = ctx.controller.tick(feedback, TARGET)
    o3 = ctx.controller.tick(feedback, TARGET)

    assert o1.sent is True
    assert o2.sent is False
    assert o2.error_code is not None
    assert o2.error_code is not ctx.mod.ControlError.STEP_LIMIT
    assert o3.sent is True
    assert o3.error_code is None
    assert len(ctx.sender.calls) == 2


# ---------------------------------------------------------------------------
# T-C2.6 — 残差门限取自 profile, 非硬编码 5 mm.
# 注入 10 mm 门限: 8 mm 必须通过, 12 mm 必须拒绝. 硬编码 5 mm 会错拒 8 mm.
# ---------------------------------------------------------------------------
def test_t_c2_6_residual_limit_comes_from_profile_not_hardcoded() -> None:
    """残差门限取自 arm_ik.profile，而非硬编码 5 mm。"""
    q_sol = q_delta(1.0)
    injected_limit = 0.01  # 10 mm, 区别于默认 5 mm

    ctx_pass = make_controller(
        [ik_ok(q_sol, position_error_m=0.008)], residual_limit_m=injected_limit
    )
    o_pass = ctx_pass.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o_pass.sent is True
    assert o_pass.error_code is None
    assert len(ctx_pass.sender.calls) == 1

    ctx_fail = make_controller(
        [ik_ok(q_sol, position_error_m=0.012)], residual_limit_m=injected_limit
    )
    o_fail = ctx_fail.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o_fail.sent is False
    assert o_fail.error_code is ctx_fail.mod.ControlError.RESIDUAL
    assert ctx_fail.sender.calls == []


# ---------------------------------------------------------------------------
# T-C3.1 — dry_run 多周期运动: 每周期决策接受 (action 非空) 但 sender 恒 0.
# 若 dry_run 只在首周期生效, 第 2/3 周期会泄漏发送.
# ---------------------------------------------------------------------------
def test_t_c3_1_dry_run_multi_cycle_motion_never_sends() -> None:
    """dry_run 多周期运动：每周期接受但 sender 恒 0。"""
    ctx = make_controller(
        [ik_ok(q_delta(0.5)), ik_ok(q_delta(1.0)), ik_ok(q_delta(1.5))], dry_run=True
    )
    feedback = make_feedback(BASE_Q)

    outcomes = [ctx.controller.tick(feedback, TARGET) for _ in range(3)]

    assert ctx.sender.calls == []
    assert all(o.sent is False for o in outcomes)
    assert all(o.action is not None for o in outcomes)
    assert all(o.error_code is None for o in outcomes)
    assert len(ctx.ik.calls) == 3


# ---------------------------------------------------------------------------
# T-C3.2 — dry_run 下 stop 路径: 进入 ACTIVE 运动后 StopCommand -> STANDBY, 后续
# tick 不得再发送也不得再触发 IK.
# ---------------------------------------------------------------------------
def test_t_c3_2_dry_run_stop_path_never_sends() -> None:
    """dry_run 下 stop 路径：进入 STANDBY 后不再发送也不再求解。"""
    from alohamini_cartesian.control.commands import StopCommand

    ctx = make_controller([ik_ok(q_delta(0.5))], dry_run=True)
    feedback = make_feedback(BASE_Q)

    o_active = ctx.controller.tick(feedback, TARGET)
    ctx.machine.handle(StopCommand())
    o_stopped = ctx.controller.tick(feedback, TARGET)

    assert ctx.sender.calls == []
    assert o_active.sent is False
    assert o_stopped.sent is False
    assert o_stopped.error_code is not None
    assert o_stopped.action is None
    assert len(ctx.ik.calls) == 1  # stop 后不得再求解 / 发送


# ---------------------------------------------------------------------------
# T-C3.3 — dry_run 下退出路径: ReleaseCommand -> SETTLING, 后续 tick 不发送.
# ---------------------------------------------------------------------------
def test_t_c3_3_dry_run_settling_release_path_never_sends() -> None:
    """dry_run 下 release 路径：进入 SETTLING 后不再发送。"""
    from alohamini_cartesian.control.commands import ReleaseCommand

    ctx = make_controller([ik_ok(q_delta(0.5))], dry_run=True)
    feedback = make_feedback(BASE_Q)

    ctx.controller.tick(feedback, TARGET)
    ctx.machine.handle(ReleaseCommand())
    outcome = ctx.controller.tick(feedback, TARGET)

    assert ctx.sender.calls == []
    assert outcome.sent is False
    assert outcome.error_code is not None
    assert len(ctx.ik.calls) == 1


# ---------------------------------------------------------------------------
# T-C3.4 — dry_run 全生命周期: 进入 ACTIVE -> 多周期运动 -> release -> stop, 每步
# 断言 sender 恒 0. 捕获「dry_run 仅首周期生效 / stop 路径泄漏」的实现.
# ---------------------------------------------------------------------------
def test_t_c3_4_dry_run_full_lifecycle_never_sends() -> None:
    """dry_run 全生命周期（运动→release→stop）sender 恒 0。"""
    from alohamini_cartesian.control.commands import ReleaseCommand, StopCommand

    ctx = make_controller(
        [ik_ok(q_delta(0.5)), ik_ok(q_delta(1.0)), ik_ok(q_delta(1.5))], dry_run=True
    )
    feedback = make_feedback(BASE_Q)

    for _ in range(3):
        ctx.controller.tick(feedback, TARGET)
        assert ctx.sender.calls == []

    ctx.machine.handle(ReleaseCommand())  # ACTIVE -> SETTLING
    ctx.controller.tick(feedback, TARGET)
    assert ctx.sender.calls == []

    ctx.machine.on_feedback(99, settled=True)  # SETTLING -> READY (settled_samples=1)
    ctx.machine.handle(StopCommand())  # READY -> STANDBY
    ctx.controller.tick(feedback, TARGET)
    assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C3.5 — 阶段互锁 (D014 底盘停车语义涉及处): ACTIVE 但 active_stage=BASE_LIFT
# 时, tick 不得求解/发送机械臂动作. 若门禁只看 state==ACTIVE (不校验 stage), 会误发.
# ---------------------------------------------------------------------------
def test_t_c3_5_active_base_lift_stage_never_sends_arm_action() -> None:
    """ACTIVE(BASE_LIFT) 阶段不得求解或发送机械臂动作。"""
    from alohamini_cartesian.control.commands import ControlState, Stage

    ctx = make_controller([ik_ok(q_delta(0.5))], stage="base")

    assert ctx.machine.state is ControlState.ACTIVE
    assert ctx.machine.active_stage is Stage.BASE_LIFT

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert ctx.sender.calls == []
    assert outcome.sent is False
    assert outcome.error_code is not None
    assert len(ctx.ik.calls) == 0  # 底盘阶段不得为机械臂求解


# ===========================================================================
# P4-B cycle 3 —— T-C4 过流 hold 终止 + T-C10 有界反向退让 (appended; 既有 33 例未动)
# ===========================================================================
#
# Required behavior (acceptance, 本 cycle 冻结):
# - T-C4 过流 hold 终止: 当周期 feedback 的 `_control_status.joint_holds` 含本臂某关节
#   hold 时, 控制器立即终止该周期前进步进 (绝不沿受限方向发运动), 清除目标累积
#   (`_last_accepted_q` -> None, 下一周期步长门禁从当周期反馈起算, 防 windup),
#   且 `CycleOutcome.error_code is ControlError.OVERCURRENT_HOLD`;
# - T-C10 有界反向退让: hold 态下被 hold 关节只允许沿「释放方向」(blocked_direction
#   的反方向, 经 adapter sign 映射到运动学空间) 的反向小步, 幅度 ∈ (0, bound],
#   bound 复用注入的 `max_joint_step_deg`; hold 持续期间绝不自动恢复正向, hold 消失后
#   恢复正常接受路径.
#
# 冻结子接缝 (本 cycle 设计并冻结, 待主 Agent 固化):
#   - 释放方向: hold entry 的 `blocked_direction`(电机空间 ±1) 取反后在 **运动学空间**
#     映射为 `release_kinematic_dir = -blocked_direction * adapter_sign(side, joint)`,
#     其中 `adapter_sign` 是 adapter 同一 side/joint 的 affine sign (motor =
#     sign*(q-zero)); sign=+1 时与「电机符号取反」一致, sign=-1 时相反 (T-C10.2 辨识).
#   - 退让上界: 复用构造参数 `max_joint_step_deg` (不新增 retreat_step_deg), 退让关节
#     命令 = 当周期反馈 q 沿释放方向移动 delta 度, 0 < |delta| <= max_joint_step_deg;
#     非 hold 关节定格在当周期反馈 (不转发前进解)。
#   - hold 周期返回 `sent=True` + `action` + `error_code=OVERCURRENT_HOLD` (退让即该
#     周期的安全动作); `dry_run=True` 时不调用 sender。
#   - `blocked_direction is None` (方向未知) 时 fail-safe: 被 hold 关节不发生任何运动。
#   - 只识别本 side 的 `joint_holds`; 另一侧关节 hold 与 `gripper_holds` 不影响本控制器。
#
# 本 cycle 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - 忽略 hold, 照常下发 IK 前进步进 (T-C4.1 / T-C10.1 / T-C10.4: 受限方向 delta>0);
# - 检测到 hold 但不清除 `_last_accepted_q`, windup 继续累积 (T-C4.2: 第 3 周期被误拒);
# - 双向自由运动 / 只按 blocked_direction 电机符号退让而忽略 adapter sign (T-C10.2);
# - 退让幅度超上界 (硬编码 2.0 / 未受 max_joint_step_deg 约束) (T-C10.1 / T-C10.3);
# - 把任意 `_control_status` hold (另一侧 / 夹爪) 当作本臂限制 (T-C4.3);
# - dry_run 下 hold 退让泄漏发送 (T-C10.6);
# - 方向未知时猜测方向乱动, 或 hold 永久锁死不再恢复 (T-C10.7 / T-C10.5).
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.

CTRL_HOLD_REASON = "overcurrent_hold"


def make_hold_entry(
    *, goal: float = 0.0, blocked_direction: float | None, release_margin: float = 1.0
) -> dict:
    """B3 `_control_status` hold entry (host shape): blocked_direction 为电机空间符号 (±1/None)."""
    return {
        "reason": CTRL_HOLD_REASON,
        "goal": float(goal),
        "blocked_direction": blocked_direction,
        "release_margin": float(release_margin),
    }


def feedback_with_holds(
    q_deg: object,
    *,
    joint_holds: dict | None = None,
    gripper_holds: dict | None = None,
    side: str = CTRL_SIDE,
    sample_id: int = 1,
) -> dict:
    """Feedback (raw observation + metadata + `_control_status`) with injected holds."""
    feedback = make_feedback(q_deg, side=side, sample_id=sample_id)
    feedback["_control_status"]["joint_holds"] = dict(joint_holds or {})
    feedback["_control_status"]["gripper_holds"] = dict(gripper_holds or {})
    return feedback


def q_from_action(action: dict[str, float], side: str = CTRL_SIDE) -> np.ndarray:
    """Invert the frozen affine independently: q = sign * motor + zero."""
    return np.array(
        [
            CTRL_SIGNS[joint] * float(action[f"arm_{side}_{joint}.pos"]) + CTRL_ZEROS[joint]
            for joint in JOINT_ORDER
        ],
        dtype=float,
    )


def shoulder_pan_hold(blocked_direction: float | None = 1.0, *, goal: float = BASE_Q[0]) -> dict:
    """A single joint_holds entry for joint 0 (sign=+1) on the controlled side."""
    return {
        "arm_left_shoulder_pan": make_hold_entry(goal=goal, blocked_direction=blocked_direction)
    }


# ---------------------------------------------------------------------------
# T-C4.0 — 冻结词汇: ControlError 提供 OVERCURRENT_HOLD 成员.
# ---------------------------------------------------------------------------
def test_t_c4_0_overcurrent_hold_error_vocabulary_exists() -> None:
    """冻结词汇：ControlError 提供 OVERCURRENT_HOLD 成员。"""
    mod = load_controller()
    assert "OVERCURRENT_HOLD" in mod.ControlError.__members__


# ---------------------------------------------------------------------------
# T-C4.1 — hold 当周期终止前进步进: 不沿受限方向发送, 报 OVERCURRENT_HOLD.
# 若实现忽略 hold, 会把 IK 的 +1.9° 前进解原样发出 (error_code=None, delta>0).
# ---------------------------------------------------------------------------
def test_t_c4_1_hold_terminates_forward_step_and_reports_overcurrent_hold() -> None:
    """Hold 当周期终止前进步进，并报 OVERCURRENT_HOLD。"""
    forward = q_delta(1.9, index=0)
    ctx = make_controller([ik_ok(forward)])
    fb = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0))

    outcome = ctx.controller.tick(fb, TARGET)

    assert outcome.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD
    assert outcome.sent is True
    assert outcome.action is not None
    assert len(ctx.sender.calls) == 1

    q_cmd = q_from_action(ctx.sender.calls[0])
    # 受限方向 = motor +1 (sign=+1 -> 关节 +1): 命令 delta 必须 <= 0.
    assert q_cmd[0] - BASE_Q[0] <= 0.0
    assert q_cmd[0] != pytest.approx(float(forward[0]), abs=1e-6)  # 不是直接转发前进解
    # 非 hold 关节定格在当周期反馈, 不随 IK 前进.
    for index in (1, 2, 3, 4, 5):
        assert q_cmd[index] == pytest.approx(BASE_Q[index], abs=1e-9)


# ---------------------------------------------------------------------------
# T-C4.2 — hold 清除目标累积 (防 windup): hold 后下一周期的步长参考从反馈起算.
# 周期1 接受 +2.0 (参考=+2.0); 周期2 hold; 周期3 解 -2.0 距反馈恰 2.0 (inclusive)
# 但距未清除的 +2.0 参考为 4.0. 若参考未清除, 周期3 被误拒 (sent=False).
# 脚本对「hold 周期是否调用 IK」两种实现都稳健 (多余脚本项被丢弃, 不假设调用次数).
# ---------------------------------------------------------------------------
def test_t_c4_2_hold_clears_target_accumulation_prevents_windup() -> None:
    """Hold 清除目标累积，防止 windup。"""
    ctx = make_controller(
        [ik_ok(q_delta(2.0, index=0)), ik_ok(q_delta(-2.0, index=0)), ik_ok(q_delta(-2.0, index=0))]
    )

    o1 = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o1.sent is True
    assert o1.error_code is None

    fb_hold = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0))
    o2 = ctx.controller.tick(fb_hold, TARGET)
    assert o2.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD

    o3 = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o3.sent is True, "hold 后目标累积未清除: 步长参考仍停在 hold 前, windup"
    assert o3.error_code is None


# ---------------------------------------------------------------------------
# T-C4.3 — 只有本 side 的 joint_holds 生效; 另一侧关节 hold 与 gripper_holds 不限制本臂.
# 若实现把任意 `_control_status` hold 当作本臂限制, 这里会被误报 OVERCURRENT_HOLD.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("joint_holds", "gripper_holds"),
    [
        ({"arm_right_shoulder_pan": make_hold_entry(goal=0.0, blocked_direction=1.0)}, {}),
        ({}, {"gripper_pan_left": make_hold_entry(goal=50.0, blocked_direction=1.0)}),
    ],
)
def test_t_c4_3_holds_for_other_side_or_gripper_do_not_block_arm(
    joint_holds: dict, gripper_holds: dict
) -> None:
    """另一侧关节 hold 与夹爪 hold 不限制本臂运动。"""
    q_sol = q_delta(1.0, index=0)
    ctx = make_controller([ik_ok(q_sol)])
    fb = feedback_with_holds(BASE_Q, joint_holds=joint_holds, gripper_holds=gripper_holds)

    outcome = ctx.controller.tick(fb, TARGET)

    assert outcome.sent is True
    assert outcome.error_code is None
    assert len(ctx.sender.calls) == 1
    assert_action_close(ctx.sender.calls[0], expected_action(q_sol))


# ---------------------------------------------------------------------------
# T-C10.1 — 退让沿释放方向且有界: blocked_direction=+1 -> 关节沿 -1 退让, 幅度
# <= 注入 max_joint_step_deg (此处 0.5). 硬编码 2.0 或未受约束会超幅.
# ---------------------------------------------------------------------------
def test_t_c10_1_bounded_retreat_along_release_direction() -> None:
    """Hold 退让沿释放方向且有界（不超过注入的步长）。"""
    ctx = make_controller([ik_ok(q_delta(1.9, index=0))], step_deg=0.5)
    fb = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0))

    outcome = ctx.controller.tick(fb, TARGET)

    assert outcome.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD
    assert outcome.sent is True
    delta = q_from_action(ctx.sender.calls[0])[0] - BASE_Q[0]
    assert delta < 0.0, "hold 关节未沿释放方向退让"
    assert abs(delta) <= 0.5 + 1e-9, "退让幅度超过 max_joint_step_deg"


# ---------------------------------------------------------------------------
# T-C10.2 — 释放方向必须经 adapter sign 映射 (运动学), 不是直接取反电机符号.
# sign=-1 的关节 (shoulder_lift): blocked_direction=+1 -> 运动学释放方向 = +1.
# 只按电机符号退让的实现会在该关节上反向 (delta 符号错误).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("joint_index", "blocked_direction", "expected_kinematic_sign"),
    [(0, 1.0, -1.0), (0, -1.0, 1.0), (1, 1.0, 1.0), (1, -1.0, -1.0)],
)
def test_t_c10_2_retreat_direction_respects_adapter_sign(
    joint_index: int, blocked_direction: float, expected_kinematic_sign: float
) -> None:
    """退让方向经 adapter sign 映射，而非照搬电机符号。"""
    joint = JOINT_ORDER[joint_index]
    ctx = make_controller([ik_ok(q_delta(1.9, index=joint_index))], step_deg=1.0)
    fb = feedback_with_holds(
        BASE_Q,
        joint_holds={
            f"arm_left_{joint}": make_hold_entry(
                goal=BASE_Q[joint_index], blocked_direction=blocked_direction
            )
        },
    )

    ctx.controller.tick(fb, TARGET)

    delta = q_from_action(ctx.sender.calls[0])[joint_index] - BASE_Q[joint_index]
    assert np.sign(delta) == expected_kinematic_sign, (joint, blocked_direction, delta)
    assert abs(delta) <= 1.0 + 1e-9


# ---------------------------------------------------------------------------
# T-C10.3 — 退让上界随注入的 max_joint_step_deg 变化 (非硬编码).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("step_deg", [0.25, 0.5, 1.0])
def test_t_c10_3_retreat_bound_tracks_max_joint_step(step_deg: float) -> None:
    """退让上界随注入的 max_joint_step_deg 变化，非硬编码。"""
    ctx = make_controller([ik_ok(q_delta(1.9, index=0))], step_deg=step_deg)
    fb = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0))

    ctx.controller.tick(fb, TARGET)

    delta = q_from_action(ctx.sender.calls[0])[0] - BASE_Q[0]
    assert delta < 0.0
    assert abs(delta) <= step_deg + 1e-9, f"退让 {abs(delta)} 超过注入上界 {step_deg}"


# ---------------------------------------------------------------------------
# T-C10.4 — hold 持续期间绝不恢复正向 (双向自由运动辨识): IK 每周期都想 +1.9° 前进,
# 控制器连续 3 周期都只能退让; 且退让不累积放大 (每周期相对反馈有界).
# 允许双向自由运动的实现会在这些周期发送 +1.9° (delta>0).
# ---------------------------------------------------------------------------
def test_t_c10_4_hold_persists_never_restores_forward_motion() -> None:
    """Hold 持续期间绝不恢复正向运动，且退让不累积放大。"""
    ctx = make_controller([ik_ok(q_delta(1.9, index=0)) for _ in range(3)])
    fb = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0))

    outcomes = [ctx.controller.tick(fb, TARGET) for _ in range(3)]

    assert all(o.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD for o in outcomes)
    assert len(ctx.sender.calls) == 3
    for call in ctx.sender.calls:
        delta = q_from_action(call)[0] - BASE_Q[0]
        assert delta <= 0.0, "hold 持续期间沿受限方向前进了"
        assert abs(delta) <= 2.0 + 1e-9, "退让幅度跨周期累积放大"


# ---------------------------------------------------------------------------
# T-C10.5 — hold 消失后不再锁死: 控制器恢复正常接受路径 (error_code=None).
# 若实现把 hold 状态永久锁存, hold 消失后仍报 OVERCURRENT_HOLD.
# 全 0 解对「退让参考」语义保持中立 (不假设清除到反馈 or 退让点).
# ---------------------------------------------------------------------------
def test_t_c10_5_hold_does_not_latch_after_clearing() -> None:
    """Hold 消失后不锁死，控制器恢复正常接受路径。"""
    ctx = make_controller([ik_ok(q_delta(0.0, index=0)) for _ in range(3)])

    o_hold = ctx.controller.tick(
        feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0)), TARGET
    )
    assert o_hold.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD

    o_clear = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o_clear.error_code is None
    assert o_clear.sent is True


# ---------------------------------------------------------------------------
# T-C10.6 — dry_run: hold 退让决策可见 (action 非空) 但 sender 调用恒 0.
# ---------------------------------------------------------------------------
def test_t_c10_6_dry_run_hold_retreat_never_sends() -> None:
    """dry_run 下 hold 退让决策可见，但 sender 恒 0。"""
    ctx = make_controller([ik_ok(q_delta(1.9, index=0))], dry_run=True)
    fb = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0))

    outcome = ctx.controller.tick(fb, TARGET)

    assert ctx.sender.calls == []
    assert outcome.sent is False
    assert outcome.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD
    assert outcome.action is not None
    delta = q_from_action(outcome.action)[0] - BASE_Q[0]
    assert delta <= 0.0


# ---------------------------------------------------------------------------
# T-C10.7 — blocked_direction 未知 (None) 时 fail-safe: 被 hold 关节不发生任何运动.
# 若实现猜测方向 (例如默认 +1) 会沿某个方向乱动.
# ---------------------------------------------------------------------------
def test_t_c10_7_unknown_blocked_direction_is_failsafe_no_motion() -> None:
    """blocked_direction 未知时 fail-safe：被 hold 关节不动。"""
    ctx = make_controller([ik_ok(q_delta(1.9, index=0))])
    fb = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(None))

    outcome = ctx.controller.tick(fb, TARGET)

    assert outcome.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD
    if outcome.action is not None:
        delta = q_from_action(outcome.action)[0] - BASE_Q[0]
        assert delta == pytest.approx(0.0, abs=1e-9), "方向未知时不得猜测退让方向"


# ===========================================================================
# P4-B cycle 4 —— T-C5 底盘显式停车 + T-C6 控制器级阶段互锁
# (appended; 既有 50 例未动)
# ===========================================================================
#
# Required behavior (acceptance, 本 cycle 冻结):
# - T-C5 (D014): 同一 CartesianController 实例在 ACTIVE(BASE_LIFT) 下, tick 收到底盘类
#   目标 (BaseCommand) 或本周期无新目标 (None, 「间隙」) 时:
#     * 底盘运动命令 (x/y/turn 任一非零) 正常透传: 经 sender 恰发一次
#       `{"x.vel": x, "y.vel": y, "theta.vel": turn}`, sent=True, error_code=None;
#     * 显式零速 (BaseCommand(0,0,0,...)) 与「间隙」(None) 都必须经 sender 显式发送
#       `{"x.vel":0,"y.vel":0,"theta.vel":0}` (D014: 不依赖 watchdog 兜底);
#     * 停止/退出路径 (StopCommand->STANDBY, ReleaseCommand->SETTLING) 之后的底盘 tick
#       仍必须显式发送全零速度 (不得因「非 ACTIVE(BASE_LIFT)」而静默跳过);
#     * dry_run=True 时底盘全路径 sender 调用恒 0 (action 仍给出拟发送值);
#     * 底盘路径绝不求解 IK, action 键恰为 alohamini 底盘动作键 x.vel/y.vel/theta.vel.
# - T-C6 (V03/D003, P2 裁定语义): 控制器把运动类命令交由状态机裁决, 跨阶段冲突时状态机
#   置 BLOCKED 且抛 MotionBlocked, 且**零发送**:
#     * ACTIVE(ARM) + tick 收到 BASE_LIFT 类目标 (含显式零速) -> BLOCKED + raise;
#     * ACTIVE(BASE_LIFT) + tick 收到 ARM 类命令 -> BLOCKED + raise (对称反向);
#     * 互锁后经 release + 停稳窗口 -> READY 的恢复路径为状态机既有行为, 且可再次驱动.
#
# 冻结 seam (本 cycle 设计并冻结, 待主 Agent 固化; 相对既有 50 例的 `tick(feedback,
# target)` 签名保持向后兼容):
#   - `CartesianController.tick(feedback, target)` 以 target 的**类型**分派底盘/机械臂:
#       target: BaseCommand                     -> 底盘 (BASE_LIFT) 路径
#       target: None                            -> 底盘「间隙」(无新目标) 路径
#       target: ArmCommand                      -> 机械臂命令路径 (仅用于阶段互锁裁决)
#       target: numpy.ndarray (4x4 pose) 及其它 -> 既有机械臂 pose 路径 (行为不变)
#   - 底盘路径 (BaseCommand/None):
#       * machine ACTIVE 时命令先交 `machine.handle(target)` 裁决:
#           - 同阶段 (BASE_LIFT) -> no-op, 继续发送;
#           - 跨阶段 (ARM)       -> state=BLOCKED, 抛 MotionBlocked, 不发送;
#         ACTIVE(BASE_LIFT) 下发送 `{"x.vel":x,"y.vel":y,"theta.vel":turn}`
#         (target is None -> 全零);
#       * machine 非 ACTIVE (STANDBY/READY/SETTLING/BLOCKED/FAULT) 时不交状态机
#         (避免误激活/误抛), 一律显式发送全零速度 (停止/间隙/退出安全路径);
#       * machine ACTIVE(ARM) 且 target is None -> NOT_ACTIVE, 不发送.
#   - 机械臂命令路径 (ArmCommand): 仅把命令交 `machine.handle` 做阶段裁决; 跨阶段 ->
#     BLOCKED + raise, 零发送. (同阶段语义不在本 cycle 范围.)
#   - 底盘动作键冻结为 alohamini 既有动作空间 `x.vel` / `y.vel` / `theta.vel`
#     (alohamini.py:819-820, alohamini_host.py:401-403, replay_bi.py:126).
#
# 本 cycle 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - 底盘目标被机械臂路径吞掉: ACTIVE(ARM) 下收到底盘目标仍照常求解/发送
#   (T-C6.1/T-C6.2: 会真的发出机械臂动作 -> sender 非空且未抛 MotionBlocked);
# - 「只切状态不发零速」: 显式零速/间隙/停止/退出时不发全零 (T-C5.2/T-C5.3/T-C5.4/T-C5.5);
# - 只按 nonzero 请求发送, 间隙时沿用上一速度 (T-C5.3);
# - 底盘路径漏做 dry_run 拦截 (T-C5.6/T-C5.7);
# - 底盘路径误走 IK 或误用 `.pos` 键 (T-C5.1/T-C5.8);
# - 互锁只置 BLOCKED 不抛出, 或先发送再互锁 (T-C6.1/T-C6.2/T-C6.3);
# - 反向只锁单向 (T-C6.3).
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.
# ===========================================================================

BASE_ACTION_ZERO: dict[str, float] = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


def base_action(x: float, y: float, turn: float) -> dict[str, float]:
    """Frozen expectation for the alohamini base action dict (identity mapping)."""
    return {"x.vel": float(x), "y.vel": float(y), "theta.vel": float(turn)}


def base_command(
    x: float = 0.0, y: float = 0.0, turn: float = 0.0, *, seconds: float = 1.0
) -> object:
    """构造带显式速度分量与时长参数的 BaseCommand。"""
    from alohamini_cartesian.control.commands import BaseCommand

    return BaseCommand(x=x, y=y, turn=turn, seconds=seconds)


def base_stop_command() -> object:
    """D014/D017 explicit zero-velocity base command (pure-data base stop)."""
    from alohamini_cartesian.control.commands import BaseCommand

    return BaseCommand(x=0.0, y=0.0, turn=0.0, seconds=0.0)


def arm_command(
    *, dx: float = 0.01, dy: float = 0.0, dz: float = 0.0, side: str = "left"
) -> object:
    """构造指定侧的小幅平移 ArmCommand。"""
    from alohamini_cartesian.control.commands import ArmCommand, Side

    return ArmCommand(side=Side.LEFT if side == "left" else Side.RIGHT, dx=dx, dy=dy, dz=dz)


def make_base_controller(*, dry_run: bool = False) -> SimpleNamespace:
    """Controller in ACTIVE(BASE_LIFT); empty IK script makes any base-path solve fail."""
    return make_controller([], dry_run=dry_run, stage="base")


def assert_base_action_close(actual: dict[str, float], expected: dict[str, float]) -> None:
    """断言底盘动作与冻结期望完全一致。"""
    assert set(actual) == set(expected)
    for key, value in expected.items():
        assert actual[key] == pytest.approx(value, abs=1e-12), f"key {key!r}"


# ---------------------------------------------------------------------------
# T-C5.1 — 底盘运动命令在 ACTIVE(BASE_LIFT) 下正常透传 (x/y/turn -> x.vel/y.vel/theta.vel).
# 若实现不建底盘路径, 这里会返回 NOT_ACTIVE 且零发送.
# ---------------------------------------------------------------------------
def test_t_c5_1_base_drive_command_passes_through() -> None:
    """底盘运动命令在 ACTIVE(BASE_LIFT) 下透传为 x/y/theta.vel。"""
    ctx = make_base_controller()

    outcome = ctx.controller.tick(
        make_feedback(BASE_Q), base_command(x=0.15, y=-0.08, turn=12.0, seconds=1.0)
    )

    expected = base_action(0.15, -0.08, 12.0)
    assert outcome.sent is True
    assert outcome.error_code is None
    assert outcome.action is not None
    assert_base_action_close(outcome.action, expected)
    assert len(ctx.sender.calls) == 1
    assert_base_action_close(ctx.sender.calls[0], expected)
    assert ctx.ik.calls == []  # 底盘路径绝不为机械臂求解


# ---------------------------------------------------------------------------
# T-C5.2 — 显式零速 BaseCommand 仍必须经 sender 显式发送全零速度 (D014).
# 若实现把零速当「无操作」跳过发送, sender 为空 -> RED.
# ---------------------------------------------------------------------------
def test_t_c5_2_base_explicit_zero_command_sends_zero_velocity() -> None:
    """显式零速底盘命令仍须经 sender 发送全零速度。"""
    ctx = make_base_controller()

    outcome = ctx.controller.tick(make_feedback(BASE_Q), base_stop_command())

    assert outcome.sent is True
    assert outcome.error_code is None
    assert len(ctx.sender.calls) == 1
    assert_base_action_close(ctx.sender.calls[0], BASE_ACTION_ZERO)


# ---------------------------------------------------------------------------
# T-C5.3 — 「间隙」(本周期无新目标, target=None) 必须显式发送全零速度, 不得沿用上一速度.
# 若实现只在 nonzero 请求时发送, 第 2 次调用不发零 -> RED.
# ---------------------------------------------------------------------------
def test_t_c5_3_base_gap_without_request_sends_zero_velocity() -> None:
    """无新目标的间隙周期必须显式发送全零速度。"""
    ctx = make_base_controller()
    fb = make_feedback(BASE_Q)

    ctx.controller.tick(fb, base_command(x=0.2, y=0.0, turn=0.0, seconds=1.0))
    gap = ctx.controller.tick(fb, None)

    assert gap.sent is True
    assert gap.error_code is None
    assert len(ctx.sender.calls) == 2
    assert_base_action_close(ctx.sender.calls[1], BASE_ACTION_ZERO)


# ---------------------------------------------------------------------------
# T-C5.4 — 停止路径: 驱动后 StopCommand -> STANDBY, 随后的底盘 tick 仍须显式发全零.
# 「只切状态不发零速」的实现会在此零发送 -> RED.
# ---------------------------------------------------------------------------
def test_t_c5_4_base_stop_path_sends_explicit_zero_velocity() -> None:
    """停止路径：切到 STANDBY 后底盘 tick 仍发全零速度。"""
    from alohamini_cartesian.control.commands import ControlState, StopCommand

    ctx = make_base_controller()
    fb = make_feedback(BASE_Q)
    ctx.controller.tick(fb, base_command(x=0.2, y=0.0, turn=0.0, seconds=1.0))

    ctx.machine.handle(StopCommand())
    assert ctx.machine.state is ControlState.STANDBY

    outcome = ctx.controller.tick(fb, base_stop_command())

    assert outcome.sent is True
    assert outcome.error_code is None
    assert_base_action_close(ctx.sender.calls[-1], BASE_ACTION_ZERO)


# ---------------------------------------------------------------------------
# T-C5.5 — 退出路径: 驱动后 ReleaseCommand -> SETTLING, 随后无新目标的底盘 tick 仍须显式发全零.
# ---------------------------------------------------------------------------
def test_t_c5_5_base_exit_path_sends_explicit_zero_velocity() -> None:
    """退出路径：release 后无新目标的底盘 tick 仍发全零速度。"""
    from alohamini_cartesian.control.commands import ControlState, ReleaseCommand

    ctx = make_base_controller()
    fb = make_feedback(BASE_Q)
    ctx.controller.tick(fb, base_command(x=0.2, y=0.0, turn=0.0, seconds=1.0))

    ctx.machine.handle(ReleaseCommand())
    assert ctx.machine.state is ControlState.SETTLING

    outcome = ctx.controller.tick(fb, None)

    assert outcome.sent is True
    assert outcome.error_code is None
    assert_base_action_close(ctx.sender.calls[-1], BASE_ACTION_ZERO)


# ---------------------------------------------------------------------------
# T-C5.6 — dry_run: 底盘驱动/显式零速/间隙三路径 sender 恒 0, 但 action 仍给出拟发送值.
# 若 dry_run 只在机械臂路径生效, 底盘路径会泄漏发送 -> RED.
# ---------------------------------------------------------------------------
def test_t_c5_6_dry_run_base_drive_zero_and_gap_never_send() -> None:
    """dry_run 下底盘驱动、零速与间隙三路径 sender 恒 0。"""
    ctx = make_base_controller(dry_run=True)
    fb = make_feedback(BASE_Q)

    o_drive = ctx.controller.tick(fb, base_command(x=0.2, y=0.0, turn=0.0, seconds=1.0))
    o_zero = ctx.controller.tick(fb, base_stop_command())
    o_gap = ctx.controller.tick(fb, None)

    assert ctx.sender.calls == []
    for outcome in (o_drive, o_zero, o_gap):
        assert outcome.sent is False
        assert outcome.error_code is None
        assert outcome.action is not None
    assert_base_action_close(o_drive.action, base_action(0.2, 0.0, 0.0))
    assert_base_action_close(o_zero.action, BASE_ACTION_ZERO)
    assert_base_action_close(o_gap.action, BASE_ACTION_ZERO)


# ---------------------------------------------------------------------------
# T-C5.7 — dry_run: 停止/退出路径 sender 恒 0.
# ---------------------------------------------------------------------------
def test_t_c5_7_dry_run_base_stop_and_exit_paths_never_send() -> None:
    """dry_run 下底盘 stop 与 exit 路径 sender 恒 0。"""
    from alohamini_cartesian.control.commands import ReleaseCommand, StopCommand

    fb = make_feedback(BASE_Q)

    ctx_stop = make_base_controller(dry_run=True)
    ctx_stop.controller.tick(fb, base_command(x=0.2, y=0.0, turn=0.0, seconds=1.0))
    ctx_stop.machine.handle(StopCommand())
    o_stop = ctx_stop.controller.tick(fb, None)

    ctx_exit = make_base_controller(dry_run=True)
    ctx_exit.controller.tick(fb, base_command(x=0.2, y=0.0, turn=0.0, seconds=1.0))
    ctx_exit.machine.handle(ReleaseCommand())
    o_exit = ctx_exit.controller.tick(fb, None)

    assert ctx_stop.sender.calls == []
    assert ctx_exit.sender.calls == []
    for outcome in (o_stop, o_exit):
        assert outcome.sent is False
        assert outcome.error_code is None
        assert outcome.action is not None
        assert_base_action_close(outcome.action, BASE_ACTION_ZERO)


# ---------------------------------------------------------------------------
# T-C5.8 — 底盘动作键恰为 alohamini 既有动作空间 x.vel/y.vel/theta.vel, 且为恒等映射.
# 若误用 `.pos` 键或改写数值, 集合/数值断言失败 -> RED.
# ---------------------------------------------------------------------------
def test_t_c5_8_base_action_uses_alohamini_base_velocity_keys() -> None:
    """底盘动作键恰为 x.vel/y.vel/theta.vel，且为恒等映射。"""
    ctx = make_base_controller()

    ctx.controller.tick(make_feedback(BASE_Q), base_command(x=1.5, y=-2.5, turn=3.5, seconds=1.0))

    assert len(ctx.sender.calls) == 1
    assert set(ctx.sender.calls[0]) == {"x.vel", "y.vel", "theta.vel"}
    assert_base_action_close(ctx.sender.calls[0], base_action(1.5, -2.5, 3.5))


# ---------------------------------------------------------------------------
# T-C5.9 — 底盘未激活 (STANDBY) 时绝不放行驱动速度: 驱动请求被显式全零取代.
# 若实现无视状态透传底盘驱动, 会发出 x.vel=0.9 -> RED; 若实现完全无底盘路径, sent=False -> RED.
# ---------------------------------------------------------------------------
def test_t_c5_9_base_drive_is_not_sent_when_base_is_not_active() -> None:
    """底盘未激活时驱动请求被显式全零速度取代。"""
    from alohamini_cartesian.control.commands import ControlState, StopCommand

    ctx = make_base_controller()
    ctx.machine.handle(StopCommand())
    assert ctx.machine.state is ControlState.STANDBY

    outcome = ctx.controller.tick(
        make_feedback(BASE_Q), base_command(x=0.9, y=0.0, turn=0.0, seconds=1.0)
    )

    assert outcome.sent is True
    assert all(call == BASE_ACTION_ZERO for call in ctx.sender.calls)
    assert_base_action_close(ctx.sender.calls[-1], BASE_ACTION_ZERO)
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C6.1 — ACTIVE(ARM) + tick 收到 BASE_LIFT 类目标: 状态机置 BLOCKED 且抛 MotionBlocked,
# 零发送. 现状 (机械臂路径吞掉底盘目标) 会真的求解并发出机械臂动作 (未抛, sender 非空).
# ---------------------------------------------------------------------------
def test_t_c6_1_active_arm_receiving_base_target_blocks_with_zero_send() -> None:
    """ACTIVE(ARM) 收到底盘目标：锁存 BLOCKED、抛出、零发送。"""
    from alohamini_cartesian.control.commands import ControlState, MotionBlocked, Stage

    ctx = make_controller([ik_ok(q_delta(0.5))], stage="arm")
    assert ctx.machine.state is ControlState.ACTIVE
    assert ctx.machine.active_stage is Stage.ARM

    with pytest.raises(MotionBlocked):
        ctx.controller.tick(
            make_feedback(BASE_Q), base_command(x=0.1, y=0.0, turn=0.0, seconds=1.0)
        )

    assert ctx.machine.state is ControlState.BLOCKED
    assert ctx.machine.active_stage is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C6.2 — 显式零速 BaseCommand 仍是 BASE_LIFT 类目标, ACTIVE(ARM) 下不得当作无害空操作.
# ---------------------------------------------------------------------------
def test_t_c6_2_active_arm_receiving_explicit_zero_base_target_still_blocks() -> None:
    """显式零速底盘目标在 ACTIVE(ARM) 下同样触发互锁。"""
    from alohamini_cartesian.control.commands import ControlState, MotionBlocked

    ctx = make_controller([ik_ok(q_delta(0.5))], stage="arm")

    with pytest.raises(MotionBlocked):
        ctx.controller.tick(make_feedback(BASE_Q), base_stop_command())

    assert ctx.machine.state is ControlState.BLOCKED
    assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C6.3 — 对称反向: ACTIVE(BASE_LIFT) + tick 收到 ARM 类命令 -> BLOCKED + raise, 零发送.
# 现状返回 NOT_ACTIVE 且不抛 -> RED.
# ---------------------------------------------------------------------------
def test_t_c6_3_active_base_lift_receiving_arm_target_blocks_with_zero_send() -> None:
    """对称反向：ACTIVE(BASE_LIFT) 收到机械臂命令即互锁且零发送。"""
    from alohamini_cartesian.control.commands import ControlState, MotionBlocked, Stage

    ctx = make_base_controller()
    assert ctx.machine.active_stage is Stage.BASE_LIFT

    with pytest.raises(MotionBlocked):
        ctx.controller.tick(make_feedback(BASE_Q), arm_command(dx=0.01))

    assert ctx.machine.state is ControlState.BLOCKED
    assert ctx.machine.active_stage is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C6.4 — 互锁可恢复: BLOCKED + release + 停稳窗口 -> READY (状态机既有行为),
# 恢复后可重新激活底盘并再次正常透传.
# ---------------------------------------------------------------------------
def test_t_c6_4_interlock_is_recoverable_via_state_machine_release_and_settle() -> None:
    """互锁可经 release 加停稳窗口恢复，并重新激活底盘。"""
    from alohamini_cartesian.control.commands import (
        BaseCommand,
        ControlState,
        MotionBlocked,
        ReleaseCommand,
        Stage,
    )

    ctx = make_base_controller()
    with pytest.raises(MotionBlocked):
        ctx.controller.tick(make_feedback(BASE_Q), arm_command(dx=0.01))
    assert ctx.machine.state is ControlState.BLOCKED

    ctx.machine.handle(ReleaseCommand())
    ctx.machine.on_feedback(2, settled=True)  # settled_samples=1 -> READY
    assert ctx.machine.state is ControlState.READY

    ctx.machine.handle(BaseCommand(x=0.3, y=0.0, turn=0.0, seconds=1.0))
    assert ctx.machine.active_stage is Stage.BASE_LIFT
    outcome = ctx.controller.tick(make_feedback(BASE_Q), base_command(x=0.3))
    assert outcome.sent is True
    assert_base_action_close(ctx.sender.calls[-1], base_action(0.3, 0.0, 0.0))


# ===========================================================================
# P4-B cycle 5 —— T-C7 领先量夹紧 + T-C8 跟随误差超时 + T-C9 命令路由 fail-closed
# (appended; 既有 63 例未动)
#
# Required behavior (acceptance, 本 cycle 冻结):
# - T-C7 领先量夹紧: 被接受的关节目标相对**当周期实测反馈**的领先 ≤ 注入参数
#   `max_joint_lead_deg`; 超限部分**有界夹紧** (clamp 到 `feedback ± lead`, 不是拒绝);
#   逐关节独立; 恰好 lead 不夹 (inclusive); 夹紧发生可观测 (接受路径 error_code=None,
#   通过 `CycleOutcome.detail` 非空字符串标记); dry-run 下 action 仍为夹紧后动作但零发送.
# - T-C8 跟随误差超时 (FakeClock): 每个 ACTIVE(ARM) arm-pose 周期评估跟踪误差
#   `err = max_j |q_feedback[j] - q_last_dispatched[j]|`; `err > follow_error_tolerance_deg`
#   持续**严格超过** `follow_error_timeout_s` (以 clock 秒计, 非周期计数) -> 终止步进:
#   sent=False, action=None, error_code=FOLLOW_TIMEOUT, 零发送; 误差回落后计时重置;
#   未到超时正常推进 (接受并发送).
# - T-C9 命令路由 fail-closed: 显式按 **target 类型** 路由, 不信任 else 兜底:
#   * `GripperCommand` -> ARM 类: ACTIVE(ARM) 下同阶段无操作 (零发送、不解 IK);
#     ACTIVE(BASE_LIFT) 下跨阶段 -> 状态机 BLOCKED + 抛 MotionBlocked、零发送;
#   * `LiftCommand` -> BASE_LIFT 类: ACTIVE(BASE_LIFT) 下同阶段无操作 (零发送、不解 IK);
#     ACTIVE(ARM) 下跨阶段 -> 状态机 BLOCKED + 抛 MotionBlocked、零发送;
#   * 未识别 target 类型 (非 BaseCommand/None/ArmCommand/GripperCommand/LiftCommand/4x4 pose)
#     -> 显式拒绝 `error_code=UNSUPPORTED_TARGET`、零发送、不解 IK、不抛异常. 现状 (else
#     落到 `_tick_arm_pose`) 会在 ACTIVE(ARM) 下真的求解并发出机械臂动作.
#
# 冻结 seam (本 cycle 设计并冻结, 待主 Agent 固化; 相对既有 63 例保持向后兼容):
#   - 领先夹紧在 step 门禁通过的接受路径上执行; 夹紧后的 q 经 adapter 转 action 并发送,
#     夹紧后的 q 也是下一周期 step/跟随监督的参照 (即命令的实际目标).
#   - 可观测性: `CycleOutcome` 字段集已被 T-C1.0 冻结 ({sent, action, error_code, detail}),
#     因此夹紧标记只能经 `detail`: 发生夹紧 -> detail 为非空字符串; 未夹紧的接受周期
#     detail 保持 None.
#   - 跟随监督的 `q_last_dispatched` = 最近一次**被接受**命令的 kinematic q (即
#     `_last_accepted_q`); 首个周期 (尚无被接受命令) 不评估. 误差在 kinematic 空间比较
#     (反馈已由 adapter 转 kinematic q).
#   - 超时为严格大于 (`> follow_error_timeout_s`); 计时起点是**首次观测到**
#     `err > tolerance` 的那个周期的 clock 时刻 (不是构造时刻, 也不是周期计数).
#   - `ControlError` 新增成员 `FOLLOW_TIMEOUT` 与 `UNSUPPORTED_TARGET`.
#
# 本 cycle 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - 无领先夹紧, IK 领先解直通发送 (T-C7.1/T-C7.3/T-C7.5/T-C7.6/T-C7.7);
# - 夹紧相对「上一被接受命令」而非实测反馈 (T-C7.7);
# - 夹紧按整段 step 向量等比缩放而非逐关节独立夹紧 (T-C7.4);
# - 硬编码 lead 门限, 忽略注入的 `max_joint_lead_deg` (T-C7.5);
# - 恰好 lead 被误夹 (T-C7.2 边界 inclusive);
# - 跟随超时用周期计数冒充 (T-C8.4 多周期短时间不触发; T-C8.1 少周期大时间触发);
# - 误差回落后计时未重置 (T-C8.2);
# - 未到超时就终止, 或每周期重置计时导致永不触发 (T-C8.1/T-C8.3);
# - 超时仍发送前进解 (T-C8.1: 断言 sender 调用数不增);
# - 未识别 target 静默落入 arm-pose 路径并发送 (T-C9.5);
# - Gripper/Lift 未做阶段互锁裁决 (T-C9.1..T-C9.4).
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.
# ===========================================================================


def make_supervision_controller(
    results: object,
    *,
    step_deg: float = 2.0,
    lead_deg: float = 30.0,
    follow_tolerance_deg: float = 1.0,
    follow_timeout_s: float = 1.0,
    dry_run: bool = False,
    side: str = CTRL_SIDE,
    residual_limit_m: float = 0.005,
) -> SimpleNamespace:
    """构造被测控制器并**暴露 FakeClock**, 以注入 T-C7/T-C8 的 lead/跟随参数.

    与既有 `make_controller` 相同的替身与状态机推进 (READY -> ACTIVE(ARM)), 额外把
    `FakeClock` 放进返回值, 供 T-C8 用显式时间推进驱动超时.
    """
    from alohamini_cartesian.control.commands import (
        ArmCommand,
        ControlStateMachine,
        Side,
    )

    mod = load_controller()
    adapter = make_adapter(side)
    ik = ScriptedIK(results, residual_limit_m=residual_limit_m)
    sender = RecordingSender()
    clock = FakeClock()

    machine = ControlStateMachine(settled_samples=1)
    machine.on_feedback(1, settled=True)
    machine.resume()
    side_enum = Side.LEFT if side == "left" else Side.RIGHT
    machine.handle(ArmCommand(side=side_enum, dx=0.01, dy=0.0, dz=0.0))

    controller = mod.CartesianController(
        state_machine=machine,
        arm_ik=ik,
        adapter=adapter,
        side=side,
        clock=clock,
        sender=sender,
        dry_run=dry_run,
        max_joint_step_deg=step_deg,
        max_joint_lead_deg=lead_deg,
        follow_error_tolerance_deg=follow_tolerance_deg,
        follow_error_timeout_s=follow_timeout_s,
    )
    return SimpleNamespace(
        controller=controller,
        ik=ik,
        sender=sender,
        adapter=adapter,
        machine=machine,
        clock=clock,
        mod=mod,
    )


# ---------------------------------------------------------------------------
# T-C7.1 — 正方向超领先被有界夹紧到 feedback+lead, 且可观测 (detail 非空).
# 无夹紧实现会直通发送 IK 的 +1.5° 解 -> action 数值不符.
# ---------------------------------------------------------------------------
def test_t_c7_1_lead_clamp_bounds_positive_overshoot_and_is_observable() -> None:
    """正方向超领先被夹紧到 feedback+lead，且 detail 可观测。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(1.5, index=0))],
        step_deg=2.0,
        lead_deg=1.0,
        follow_tolerance_deg=100.0,
        follow_timeout_s=100.0,
    )

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    clamped = q_delta(1.0, index=0)
    assert outcome.sent is True
    assert outcome.error_code is None
    assert outcome.action is not None
    assert_action_close(outcome.action, expected_action(clamped))
    assert len(ctx.sender.calls) == 1
    assert_action_close(ctx.sender.calls[0], expected_action(clamped))
    assert outcome.detail is not None and isinstance(outcome.detail, str), "夹紧未可观测"


# ---------------------------------------------------------------------------
# T-C7.2 — 边界: 恰好 lead 不夹 (inclusive), detail 保持 None.
# 把边界当超限 (>=) 误夹的实现会发送被夹紧解 -> RED.
# ---------------------------------------------------------------------------
def test_t_c7_2_lead_boundary_exact_is_not_clamped() -> None:
    """恰好等于 lead 不夹紧，detail 保持 None。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(1.0, index=0))],
        step_deg=2.0,
        lead_deg=1.0,
        follow_tolerance_deg=100.0,
        follow_timeout_s=100.0,
    )

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is True
    assert outcome.error_code is None
    assert_action_close(outcome.action, expected_action(q_delta(1.0, index=0)))
    assert outcome.detail is None, "恰好 lead 不应产生夹紧标记"


# ---------------------------------------------------------------------------
# T-C7.3 — 反方向 (滞后) 超领先同样被夹紧到 feedback-lead.
# 只处理正方向或取绝对值符号错误的实现会发出 -1.5° 或 +1.0° -> RED.
# ---------------------------------------------------------------------------
def test_t_c7_3_lead_clamp_bounds_negative_overshoot() -> None:
    """负方向超领先同样被夹紧到 feedback-lead。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(-1.5, index=0))],
        step_deg=2.0,
        lead_deg=1.0,
        follow_tolerance_deg=100.0,
        follow_timeout_s=100.0,
    )

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    clamped = q_delta(-1.0, index=0)
    assert outcome.sent is True
    assert outcome.error_code is None
    assert_action_close(outcome.action, expected_action(clamped))
    assert outcome.detail is not None


# ---------------------------------------------------------------------------
# T-C7.4 — 夹紧逐关节独立: 仅超 lead 的关节被夹, 其它关节保持 IK 解.
# 整段等比缩放 / 只夹单关节的实现会让未超限关节数值改变 -> RED.
# ---------------------------------------------------------------------------
def test_t_c7_4_lead_clamp_is_per_joint() -> None:
    """夹紧逐关节独立：仅超限关节被夹，其余保持 IK 解。"""
    q_sol = np.array(BASE_Q, dtype=float)
    q_sol[0] += 3.0  # 超 lead (1.0)
    q_sol[1] += 0.5  # 未超 lead
    expected = np.array(BASE_Q, dtype=float)
    expected[0] += 1.0
    expected[1] += 0.5

    ctx = make_supervision_controller(
        [ik_ok(q_sol)],
        step_deg=5.0,
        lead_deg=1.0,
        follow_tolerance_deg=100.0,
        follow_timeout_s=100.0,
    )

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is True
    assert outcome.error_code is None
    assert_action_close(outcome.action, expected_action(expected))
    assert outcome.detail is not None


# ---------------------------------------------------------------------------
# T-C7.5 — 夹紧上界随注入的 max_joint_lead_deg 变化 (非硬编码).
# 硬编码 lead 的实现会在 (0.5,1.5) / (2.0,1.5) 组合上数值不符 -> RED.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("lead_deg", "delta", "expected_delta"),
    [(0.5, 1.5, 0.5), (2.0, 1.5, 1.5), (2.0, 3.0, 2.0)],
)
def test_t_c7_5_lead_bound_tracks_injected_parameter(
    lead_deg: float, delta: float, expected_delta: float
) -> None:
    """夹紧上界随注入的 max_joint_lead_deg 变化，非硬编码。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(delta, index=0))],
        step_deg=10.0,
        lead_deg=lead_deg,
        follow_tolerance_deg=100.0,
        follow_timeout_s=100.0,
    )

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is True
    assert outcome.error_code is None
    assert_action_close(outcome.action, expected_action(q_delta(expected_delta, index=0)))


# ---------------------------------------------------------------------------
# T-C7.6 — dry_run: 夹紧决策可见 (action = 夹紧后动作, detail 标记) 但 sender 恒 0.
# ---------------------------------------------------------------------------
def test_t_c7_6_dry_run_lead_clamp_is_visible_without_send() -> None:
    """dry_run 下夹紧决策可见（action/detail），但 sender 恒 0。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(1.5, index=0))],
        step_deg=2.0,
        lead_deg=1.0,
        follow_tolerance_deg=100.0,
        follow_timeout_s=100.0,
        dry_run=True,
    )

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert ctx.sender.calls == []
    assert outcome.sent is False
    assert outcome.error_code is None
    assert_action_close(outcome.action, expected_action(q_delta(1.0, index=0)))
    assert outcome.detail is not None


# ---------------------------------------------------------------------------
# T-C7.7 — 夹紧基准是「当周期实测反馈」而非「上一被接受命令」.
# 周期1 接受 +0.5 (参考=+0.5); 周期2 反馈仍为 BASE_Q, IK 解 +1.5 → 相对反馈领先 1.5>lead
# -> 夹到 feedback+1.0. 若按参考 (+0.5) 夹紧则 1.5-0.5=1.0 == lead 不夹, 会发出 +1.5 -> RED.
# ---------------------------------------------------------------------------
def test_t_c7_7_lead_clamp_is_relative_to_feedback_not_reference() -> None:
    """夹紧基准是当周期反馈，而非上一被接受命令。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5, index=0)), ik_ok(q_delta(1.5, index=0))],
        step_deg=10.0,
        lead_deg=1.0,
        follow_tolerance_deg=100.0,
        follow_timeout_s=100.0,
    )
    feedback = make_feedback(BASE_Q)  # 反馈不随被接受命令变化

    o1 = ctx.controller.tick(feedback, TARGET)
    o2 = ctx.controller.tick(feedback, TARGET)

    assert o1.sent is True and o1.error_code is None
    assert o2.sent is True and o2.error_code is None
    assert_action_close(o2.action, expected_action(q_delta(1.0, index=0)))
    assert o2.detail is not None
    assert len(ctx.sender.calls) == 2


# ---------------------------------------------------------------------------
# T-C8.0 — 冻结词汇: ControlError 提供 FOLLOW_TIMEOUT 成员.
# ---------------------------------------------------------------------------
def test_t_c8_0_follow_timeout_error_vocabulary_exists() -> None:
    """冻结词汇：ControlError 提供 FOLLOW_TIMEOUT 成员。"""
    mod = load_controller()
    assert "FOLLOW_TIMEOUT" in mod.ControlError.__members__


# ---------------------------------------------------------------------------
# T-C8.1 — 跟随误差持续超过超时阈值: 终止步进、FOLLOW_TIMEOUT、零发送.
# 周期1 下发 +0.5 (t=0); 周期2 误差 0.5>tol=0.1 起计时 (t=0.5, 未到超时 -> 正常发送);
# 周期3 (t=1.0, 已持续 0.5>0.4) -> FOLLOW_TIMEOUT, sender 调用数不再增加.
# 每周期重置计时的实现永不触发 -> RED; 超时仍发送前进解的实现 sender 数增加 -> RED.
# ---------------------------------------------------------------------------
def test_t_c8_1_follow_error_timeout_terminates_with_zero_send() -> None:
    """跟随误差持续超时后终止步进、报 FOLLOW_TIMEOUT、零发送。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5)) for _ in range(4)],
        step_deg=2.0,
        lead_deg=30.0,
        follow_tolerance_deg=0.1,
        follow_timeout_s=0.4,
    )
    feedback = make_feedback(BASE_Q)  # 实测始终未跟随 (停在 BASE_Q)

    o1 = ctx.controller.tick(feedback, TARGET)
    assert o1.sent is True
    assert o1.error_code is None
    assert len(ctx.sender.calls) == 1

    ctx.clock.advance(0.5)
    o2 = ctx.controller.tick(feedback, TARGET)
    assert o2.error_code is None, "未到超时应正常推进"
    assert o2.sent is True
    assert len(ctx.sender.calls) == 2

    ctx.clock.advance(0.5)
    o3 = ctx.controller.tick(feedback, TARGET)
    assert o3.sent is False
    assert o3.action is None
    assert o3.error_code is ctx.mod.ControlError.FOLLOW_TIMEOUT
    assert len(ctx.sender.calls) == 2, "超时周期不得再发送前进解"


# ---------------------------------------------------------------------------
# T-C8.2 — 误差回落后计时重置.
# 周期2 (t=0.3) 误差 0.5>tol 起计时; 周期3 (t=0.5) 反馈追平 -> 误差 0 重置;
# 周期4 (t=0.9, 距 t=0.3 已 0.6>0.4) 若未重置会误报 FOLLOW_TIMEOUT -> RED.
# ---------------------------------------------------------------------------
def test_t_c8_2_follow_error_recovery_resets_timer() -> None:
    """误差回落后重新计时，不在旧起点上误报超时。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5)) for _ in range(4)],
        step_deg=2.0,
        lead_deg=30.0,
        follow_tolerance_deg=0.1,
        follow_timeout_s=0.4,
    )
    fb_lag = make_feedback(BASE_Q)  # 未跟随
    fb_caught = make_feedback(q_delta(0.5))  # 已跟随到已下发目标

    o1 = ctx.controller.tick(fb_lag, TARGET)
    assert o1.sent is True

    ctx.clock.advance(0.3)
    o2 = ctx.controller.tick(fb_lag, TARGET)  # 误差出现, 起计时
    assert o2.sent is True and o2.error_code is None

    ctx.clock.advance(0.2)  # t=0.5
    o3 = ctx.controller.tick(fb_caught, TARGET)  # 误差回落 -> 重置
    assert o3.sent is True and o3.error_code is None

    ctx.clock.advance(0.4)  # t=0.9
    o4 = ctx.controller.tick(fb_caught, TARGET)
    assert o4.sent is True
    assert o4.error_code is None, "误差已回落却仍在计时 (计时未重置)"


# ---------------------------------------------------------------------------
# T-C8.3 — 超时边界严格大于: 恰好 follow_error_timeout_s 不触发, 略超即触发.
# 用 >= 判断的实现会在恰好阈值处误终止 -> RED.
# ---------------------------------------------------------------------------
def test_t_c8_3_follow_timeout_boundary_is_strict() -> None:
    """超时边界严格大于：恰好阈值不触发，略超即触发。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5)) for _ in range(5)],
        step_deg=2.0,
        lead_deg=30.0,
        follow_tolerance_deg=0.1,
        follow_timeout_s=0.4,
    )
    feedback = make_feedback(BASE_Q)

    ctx.controller.tick(feedback, TARGET)  # t=0 下发 +0.5

    ctx.clock.advance(0.5)
    o2 = ctx.controller.tick(feedback, TARGET)  # 误差出现, 起计时 t=0.5
    assert o2.error_code is None

    ctx.clock.advance(0.4)  # t=0.9 == 起点 + timeout 恰好
    o3 = ctx.controller.tick(feedback, TARGET)
    assert o3.sent is True
    assert o3.error_code is None, "恰好等于超时阈值不应终止"

    ctx.clock.advance(0.001)  # t=0.901 > 起点 + timeout
    o4 = ctx.controller.tick(feedback, TARGET)
    assert o4.sent is False
    assert o4.action is None
    assert o4.error_code is ctx.mod.ControlError.FOLLOW_TIMEOUT


# ---------------------------------------------------------------------------
# T-C8.4 — 计时以 clock 秒计, 不是周期计数: 多周期但总时长未超阈值 -> 不触发.
# 「误差持续 N 个周期即超时」的实现会误触发 -> RED.
# ---------------------------------------------------------------------------
def test_t_c8_4_follow_error_many_cycles_short_time_does_not_timeout() -> None:
    """超时以 clock 秒计，多周期但总时长不足不触发。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5)) for _ in range(10)],
        step_deg=2.0,
        lead_deg=30.0,
        follow_tolerance_deg=0.1,
        follow_timeout_s=1.0,
    )
    feedback = make_feedback(BASE_Q)

    ctx.controller.tick(feedback, TARGET)  # t=0
    outcomes = []
    for _ in range(6):  # 6 个额外周期, 仅推进 0.05s/周期 (总计 0.3s < 1.0s)
        ctx.clock.advance(0.05)
        outcomes.append(ctx.controller.tick(feedback, TARGET))

    assert all(o.error_code is None for o in outcomes), "按周期计数冒充超时"
    assert all(o.sent is True for o in outcomes)


# ---------------------------------------------------------------------------
# T-C8.5 — 少量周期但单次时间大跳变超过阈值 -> 触发.
# 需要多个误差周期才触发的 (伪) 实现会漏报 -> RED.
# ---------------------------------------------------------------------------
def test_t_c8_5_follow_timeout_triggers_after_large_clock_jump() -> None:
    """少量周期但单次时间大跳变超过阈值即触发超时。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5)) for _ in range(4)],
        step_deg=2.0,
        lead_deg=30.0,
        follow_tolerance_deg=0.1,
        follow_timeout_s=0.4,
    )
    feedback = make_feedback(BASE_Q)

    ctx.controller.tick(feedback, TARGET)  # t=0

    ctx.clock.advance(0.1)
    o2 = ctx.controller.tick(feedback, TARGET)  # 误差出现, 起计时 t=0.1
    assert o2.error_code is None

    ctx.clock.advance(5.0)  # 单次大跳变
    o3 = ctx.controller.tick(feedback, TARGET)
    assert o3.sent is False
    assert o3.error_code is ctx.mod.ControlError.FOLLOW_TIMEOUT


# ---------------------------------------------------------------------------
# T-C9.0 — 冻结词汇: ControlError 提供 UNSUPPORTED_TARGET 成员.
# ---------------------------------------------------------------------------
def test_t_c9_0_unsupported_target_error_vocabulary_exists() -> None:
    """冻结词汇：ControlError 提供 UNSUPPORTED_TARGET 成员。"""
    mod = load_controller()
    assert "UNSUPPORTED_TARGET" in mod.ControlError.__members__


def gripper_open_command(*, side: str = "left") -> object:
    """构造打开指定侧夹爪的 GripperCommand。"""
    from alohamini_cartesian.control.commands import GripperAction, GripperCommand, Side

    return GripperCommand(
        side=Side.LEFT if side == "left" else Side.RIGHT, action=GripperAction.OPEN
    )


def lift_command(*, delta_mm: float = 5.0) -> object:
    """构造抬升 delta_mm 毫米的 LiftCommand。"""
    from alohamini_cartesian.control.commands import LiftCommand

    return LiftCommand(delta_mm=delta_mm)


# ---------------------------------------------------------------------------
# T-C9.1 — ACTIVE(ARM) + GripperCommand: ARM 类同阶段无操作, 零发送、不解 IK.
# else 兜底实现会把它当 arm pose 求解并发送 -> RED.
# ---------------------------------------------------------------------------
def test_t_c9_1_gripper_command_is_arm_stage_interlock_only() -> None:
    """ACTIVE(ARM) 下 GripperCommand 为同阶段无操作：零发送、不解 IK。"""
    ctx = make_controller([ik_ok(q_delta(0.5))])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), gripper_open_command())

    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []
    assert ctx.machine.state.name == "ACTIVE"


# ---------------------------------------------------------------------------
# T-C9.2 — ACTIVE(BASE_LIFT) + GripperCommand: 跨阶段冲突 -> BLOCKED + MotionBlocked、零发送.
# else 兜底实现会返回 NOT_ACTIVE (不置 BLOCKED, 不抛) -> RED.
# ---------------------------------------------------------------------------
def test_t_c9_2_gripper_command_cross_stage_blocks_on_base_lift() -> None:
    """ACTIVE(BASE_LIFT) 下 GripperCommand 跨阶段冲突：BLOCKED、抛出、零发送。"""
    from alohamini_cartesian.control.commands import ControlState, MotionBlocked, Stage

    ctx = make_base_controller()
    assert ctx.machine.active_stage is Stage.BASE_LIFT

    with pytest.raises(MotionBlocked):
        ctx.controller.tick(make_feedback(BASE_Q), gripper_open_command())

    assert ctx.machine.state is ControlState.BLOCKED
    assert ctx.machine.active_stage is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C9.3 — ACTIVE(BASE_LIFT) + LiftCommand: BASE_LIFT 类同阶段无操作, 零发送、不解 IK.
# else 兜底实现会返回 NOT_ACTIVE (error_code 非 None) -> RED.
# ---------------------------------------------------------------------------
def test_t_c9_3_lift_command_is_base_stage_interlock_only() -> None:
    """ACTIVE(BASE_LIFT) 下 LiftCommand 为同阶段无操作：零发送、不解 IK。"""
    ctx = make_base_controller()

    outcome = ctx.controller.tick(make_feedback(BASE_Q), lift_command())

    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C9.4 — ACTIVE(ARM) + LiftCommand: 跨阶段冲突 -> BLOCKED + MotionBlocked、零发送.
# else 兜底实现会把它当 arm pose 求解并发送 (未抛, sender 非空) -> RED.
# ---------------------------------------------------------------------------
def test_t_c9_4_lift_command_cross_stage_blocks_on_arm() -> None:
    """ACTIVE(ARM) 下 LiftCommand 跨阶段冲突：BLOCKED、抛出、零发送。"""
    from alohamini_cartesian.control.commands import ControlState, MotionBlocked, Stage

    ctx = make_controller([ik_ok(q_delta(0.5))], stage="arm")
    assert ctx.machine.active_stage is Stage.ARM

    with pytest.raises(MotionBlocked):
        ctx.controller.tick(make_feedback(BASE_Q), lift_command())

    assert ctx.machine.state is ControlState.BLOCKED
    assert ctx.machine.active_stage is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C9.5 — 未识别 target 类型 fail-closed: 显式拒绝 UNSUPPORTED_TARGET、零发送、不解 IK、不抛.
# else 兜底实现会在 ACTIVE(ARM) 下真的求解并发出机械臂动作 (sent=True, sender 非空) -> RED.
# ---------------------------------------------------------------------------
def test_t_c9_5_unrecognized_target_type_is_rejected_fail_closed() -> None:
    """未识别 target 类型 fail-closed：UNSUPPORTED_TARGET、零发送、不解 IK。"""
    ctx = make_controller([ik_ok(q_delta(0.5))])

    outcome = ctx.controller.tick(make_feedback(BASE_Q), object())

    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code is ctx.mod.ControlError.UNSUPPORTED_TARGET
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ===========================================================================
# P4-B cycle 6 —— T-C9 错误码完备 + T-C16 report_blocked + T-C18 保活 deadline
# (appended; 既有 84 例未动)
# ===========================================================================
#
# Required behavior (acceptance, 本 cycle 冻结):
# - T-C9 错误码完备 (plan「错误码完备」):
#     * `ControlError` 的成员集合与冻结词表**集合相等** (无漏码、无多余码、无别名/重复值);
#       冻结词表 = {STEP_LIMIT, NOT_ACTIVE, IK_FAILED, ADAPTER_FAILED, RESIDUAL,
#                    JOINT_LIMIT, NON_FINITE, OVERCURRENT_HOLD, FOLLOW_TIMEOUT,
#                    UNSUPPORTED_TARGET};
#     * 每条拒绝路径映射唯一码 (确定性), 且每个冻结码都至少被一条拒绝路径产生 (完备);
#     * 行为断言: 各拒绝场景返回预期码 (含 solve 抛异常 -> IK_FAILED、adapter 各阶段
#       失败 -> ADAPTER_FAILED)。
# - T-C16 report_blocked (B5.1 已裁定):
#     * `CartesianController.report_blocked(reason)` 为外部入口, 非抛出 (D015 生命周期语义);
#     * 调用后状态机进 `BLOCKED`、清空活动阶段、记录 reason;
#     * 其后机械臂 pose 周期一律零发送、不解 IK (fail-closed);
#     * BLOCKED 不自动恢复: 必须走既有恢复路径 (ReleaseCommand 武装窗口 + 停稳窗口 -> READY)
#       才能重新驱动, 恢复后控制器可再次正常发送。
# - T-C18 保活 deadline (D013 已裁定: 暴露只读属性, 不自动重发):
#     * controller 暴露只读属性 `next_deadline`;
#     * 语义: `None` 直到发生**实际发送**; 之后 = 上次实际发送时刻 (注入 clock) +
#       安全间隔 `keepalive_interval_s`;
#     * 被拒周期不推进 deadline; dry_run 无实际发送 -> 恒 `None` (本 cycle 设计并报告);
#     * 必须使用注入 clock (非真实墙钟) 与注入的安全间隔 (非硬编码)。
#
# 冻结/建议 seam (待主 Agent 固化):
#   - `CartesianController.report_blocked(reason: str) -> None` (非抛出, 外部入口)。
#   - `CartesianController.next_deadline` 只读属性 (无 setter); 未发送时为 `None`。
#   - 构造参数 `keepalive_interval_s: float` (建议关键字, 需带默认值以避免破坏既有
#     84 例构造; 本 cycle 测试一律显式注入, 不依赖默认值)。
#
# 本 cycle 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - 漏码 (枚举缺一个成员) 或别名 (两成员同值) -> T-C9.6 集合相等/计数失败;
# - 某拒绝路径静默接受 (返回 None) 或返回其它码 -> T-C9.7;
# - solve 异常被吞成接受, 或 adapter 失败与 IK 失败坍缩为同一码 -> T-C9.8;
# - `report_blocked` 只置内部标志不改状态机 -> 其后 tick 仍求解并发送 -> T-C16.1/.2;
# - `report_blocked` 后自动恢复 (未 Release 即 READY) -> T-C16.2;
# - `next_deadline` 用真实墙钟 (`time.monotonic`) -> 与注入 clock 微秒级值不符 -> T-C18.1/.2;
# - 安全间隔硬编码 -> 注入 0.25 vs 2.0 的 deadline 相同 -> T-C18.2;
# - 被拒周期也推进 deadline -> T-C18.3;
# - dry_run 无发送却武装 deadline -> T-C18.4;
# - `next_deadline` 为可写普通属性 -> T-C18.0 只读断言失败。
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.
# ===========================================================================

# 冻结错误码词表 (冻结字面量, 不从被测枚举回读).
FROZEN_ERROR_CODES: frozenset[str] = frozenset(
    {
        "STEP_LIMIT",
        "NOT_ACTIVE",
        "IK_FAILED",
        "ADAPTER_FAILED",
        "RESIDUAL",
        "JOINT_LIMIT",
        "NON_FINITE",
        "OVERCURRENT_HOLD",
        "FOLLOW_TIMEOUT",
        "UNSUPPORTED_TARGET",
    }
)

# 每条拒绝路径 -> 唯一预期码 (冻结字面量).
EXPECTED_REJECTION_CODES: dict[str, str] = {
    "step_limit": "STEP_LIMIT",
    "not_active": "NOT_ACTIVE",
    "ik_exception": "IK_FAILED",
    "adapter_missing_metadata": "ADAPTER_FAILED",
    "residual": "RESIDUAL",
    "joint_limit": "JOINT_LIMIT",
    "non_finite": "NON_FINITE",
    "overcurrent_hold": "OVERCURRENT_HOLD",
    "follow_timeout": "FOLLOW_TIMEOUT",
    "unsupported_target": "UNSUPPORTED_TARGET",
}

KEEPALIVE_INTERVAL_S = 0.5
KEEPALIVE_CLOCK_START = 1000.0


class RaisingIK:
    """arm_ik 替身: solve 恒抛异常, 用于命中 IK_FAILED 兜底路径."""

    def __init__(self, exc: Exception, *, residual_limit_m: float = 0.005) -> None:
        """保存注入异常与 profile 残差门限。"""
        self._exc = exc
        self.profile = SimpleNamespace(residual_limit_m=float(residual_limit_m))
        self.calls: list[dict] = []

    def solve(self, *args: object, **kwargs: object) -> object:
        """记录入参后抛出注入异常，用于命中 IK_FAILED 兜底路径。"""
        self.calls.append({"args": tuple(args), "kwargs": dict(kwargs)})
        raise self._exc


class FailingAdapter:
    """包装真实 adapter, 在指定阶段抛异常 (覆盖 ADAPTER_FAILED 的多条来源路径)."""

    def __init__(self, real: object, *, fail_on: str) -> None:
        """包装真实 adapter 并指定注入失败的阶段。"""
        self._real = real
        self._fail_on = fail_on
        self._signs = real._signs

    def observation_to_kinematic_deg(
        self, side: str, feedback: dict, metadata: dict
    ) -> dict[str, float]:
        """在 observe 阶段按需注入失败，其余委托真实 adapter。"""
        if self._fail_on == "observe":
            raise RuntimeError("injected observation_to_kinematic_deg failure")
        return self._real.observation_to_kinematic_deg(side, feedback, metadata)

    def effective_limits_deg(self, side: str, metadata: dict) -> dict:
        """在 limits 阶段按需注入失败，其余委托真实 adapter。"""
        if self._fail_on == "limits":
            raise RuntimeError("injected effective_limits_deg failure")
        return self._real.effective_limits_deg(side, metadata)

    def kinematic_deg_to_action(
        self, side: str, q_deg: np.ndarray, metadata: dict
    ) -> dict[str, float]:
        """在 action 阶段按需注入失败，其余委托真实 adapter。"""
        if self._fail_on == "action":
            raise RuntimeError("injected kinematic_deg_to_action failure")
        return self._real.kinematic_deg_to_action(side, q_deg, metadata)


def make_cycle6_controller(
    results: object = (),
    *,
    step_deg: float = 2.0,
    lead_deg: float = 30.0,
    follow_tolerance_deg: float = 100.0,
    follow_timeout_s: float = 100.0,
    dry_run: bool = False,
    active: bool = True,
    stage: str = "arm",
    side: str = CTRL_SIDE,
    residual_limit_m: float = 0.005,
    clock_start: float = KEEPALIVE_CLOCK_START,
    keepalive_interval_s: float | None = None,
    adapter: object | None = None,
    ik: object | None = None,
) -> SimpleNamespace:
    """构造控制器, 支持注入 FakeClock 起点 / 安全间隔 / adapter / arm_ik 替身.

    `keepalive_interval_s` 仅在显式传入时作为关键字转发; 便于验证「安全间隔注入而非
    硬编码」, 同时不依赖生产默认值.
    """
    from alohamini_cartesian.control.commands import (
        ArmCommand,
        BaseCommand,
        ControlStateMachine,
        Side,
    )

    mod = load_controller()
    adapter_double = make_adapter(side) if adapter is None else adapter
    ik_double = ScriptedIK(results, residual_limit_m=residual_limit_m) if ik is None else ik
    sender = RecordingSender()
    clock = FakeClock(start=clock_start)

    machine = ControlStateMachine(settled_samples=1)
    machine.on_feedback(1, settled=True)
    machine.resume()
    if active:
        if stage == "arm":
            side_enum = Side.LEFT if side == "left" else Side.RIGHT
            machine.handle(ArmCommand(side=side_enum, dx=0.01, dy=0.0, dz=0.0))
        elif stage == "base":
            machine.handle(BaseCommand(x=0.0, y=0.0, turn=0.0, seconds=0.0))
        else:
            raise ValueError(f"unknown stage {stage!r}")

    kwargs: dict = {
        "state_machine": machine,
        "arm_ik": ik_double,
        "adapter": adapter_double,
        "side": side,
        "clock": clock,
        "sender": sender,
        "dry_run": dry_run,
        "max_joint_step_deg": step_deg,
        "max_joint_lead_deg": lead_deg,
        "follow_error_tolerance_deg": follow_tolerance_deg,
        "follow_error_timeout_s": follow_timeout_s,
    }
    if keepalive_interval_s is not None:
        kwargs["keepalive_interval_s"] = keepalive_interval_s
    controller = mod.CartesianController(**kwargs)
    return SimpleNamespace(
        controller=controller,
        ik=ik_double,
        sender=sender,
        adapter=adapter_double,
        machine=machine,
        clock=clock,
        mod=mod,
    )


def _produce_rejection_code(kind: str) -> object:
    """Drive one rejection path end-to-end and return the emitted ControlError member."""
    if kind == "step_limit":
        ctx = make_controller([ik_ok(q_delta(3.0))])
        return ctx.controller.tick(make_feedback(BASE_Q), TARGET).error_code
    if kind == "not_active":
        ctx = make_controller([ik_ok(q_delta(0.5))], active=False)
        return ctx.controller.tick(make_feedback(BASE_Q), TARGET).error_code
    if kind == "ik_exception":
        ctx = make_cycle6_controller(ik=RaisingIK(RuntimeError("injected solver failure")))
        return ctx.controller.tick(make_feedback(BASE_Q), TARGET).error_code
    if kind == "adapter_missing_metadata":
        ctx = make_controller([ik_ok(q_delta(0.5))])
        feedback = make_feedback(BASE_Q)
        del feedback["_robot_metadata"]
        return ctx.controller.tick(feedback, TARGET).error_code
    if kind == "residual":
        ctx = make_controller([ik_ok(q_delta(1.0), position_error_m=0.006)])
        return ctx.controller.tick(make_feedback(BASE_Q), TARGET).error_code
    if kind == "joint_limit":
        ctx = make_controller([ik_fail("JOINT_LIMIT", q_delta(1.0))])
        return ctx.controller.tick(make_feedback(BASE_Q), TARGET).error_code
    if kind == "non_finite":
        ctx = make_controller([ik_fail("NON_FINITE", q_delta(1.0))])
        return ctx.controller.tick(make_feedback(BASE_Q), TARGET).error_code
    if kind == "overcurrent_hold":
        ctx = make_controller([])
        fb = feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0))
        return ctx.controller.tick(fb, TARGET).error_code
    if kind == "follow_timeout":
        ctx = make_supervision_controller(
            [ik_ok(q_delta(0.5)) for _ in range(4)],
            step_deg=2.0,
            lead_deg=30.0,
            follow_tolerance_deg=0.1,
            follow_timeout_s=0.4,
        )
        feedback = make_feedback(BASE_Q)
        ctx.controller.tick(feedback, TARGET)  # t=0 下发
        ctx.clock.advance(0.5)
        ctx.controller.tick(feedback, TARGET)  # 误差起计时
        ctx.clock.advance(0.5)
        return ctx.controller.tick(feedback, TARGET).error_code  # 超时
    if kind == "unsupported_target":
        ctx = make_controller([ik_ok(q_delta(0.5))])
        return ctx.controller.tick(make_feedback(BASE_Q), object()).error_code
    raise ValueError(f"unknown rejection kind {kind!r}")


# ---------------------------------------------------------------------------
# T-C9.6 — 冻结词表集合相等: 无漏码、无多余码、无别名 (值互异).
# 枚举缺一个成员 -> 集合不等; 多一个成员 -> 集合不等; 两成员同值产生别名 -> 计数不足.
# ---------------------------------------------------------------------------
def test_t_c9_6_control_error_vocabulary_exactly_matches_frozen_set() -> None:
    """ControlError 词表集合相等：无漏码、无多余码、无别名。"""
    mod = load_controller()

    assert set(mod.ControlError.__members__) == set(FROZEN_ERROR_CODES), (
        f"ControlError 成员与冻结词表不一致: {sorted(mod.ControlError.__members__)}"
    )
    assert len(mod.ControlError) == len(FROZEN_ERROR_CODES), "存在别名 (重复值) 错误码"
    values = [member.value for member in mod.ControlError]
    assert len(values) == len(set(values))
    for member in mod.ControlError:
        assert isinstance(member.value, str) and member.value


# ---------------------------------------------------------------------------
# T-C9.7 — 每条拒绝路径映射唯一码, 且冻结词表每个码均可被行为触达 (完备).
# 任一拒绝路径静默接受 (返回 None) 或返回它码 -> 断言失败; 若某码从不被产生 -> 完备性失败.
# ---------------------------------------------------------------------------
def test_t_c9_7_all_frozen_error_codes_are_behavior_reachable() -> None:
    """每条拒绝路径映射唯一码，且冻结词表每个码均可被行为触达。"""
    observed: dict[str, str] = {}
    for kind, expected_name in EXPECTED_REJECTION_CODES.items():
        code = _produce_rejection_code(kind)
        assert code is not None, f"{kind} 未返回错误码 (拒绝路径静默通过)"
        assert code.name == expected_name, (kind, code, expected_name)
        observed[kind] = code.name

    assert set(observed.values()) == set(FROZEN_ERROR_CODES), (
        "存在从未被任何拒绝路径产生的错误码: "
        f"{sorted(set(FROZEN_ERROR_CODES) - set(observed.values()))}"
    )
    assert len(set(observed.values())) == len(FROZEN_ERROR_CODES)


# ---------------------------------------------------------------------------
# T-C9.8 — adapter 三段失败 (观测转换 / 限位读取 / 动作转换) 均映射 ADAPTER_FAILED、零发送.
# 若某段失败被吞成接受或坍缩为它码 -> RED; 且这些段位不得误报 IK_FAILED.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("fail_on", ["observe", "limits", "action"])
def test_t_c9_8_adapter_conversion_failures_map_to_adapter_failed(fail_on: str) -> None:
    """Adapter 三段失败（观测/限位/动作）均映射 ADAPTER_FAILED 且零发送。"""
    real = make_adapter()
    ctx = make_cycle6_controller(
        adapter=FailingAdapter(real, fail_on=fail_on),
        ik=ScriptedIK([ik_ok(q_delta(1.0))]),
    )

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code is ctx.mod.ControlError.ADAPTER_FAILED
    assert outcome.error_code is not ctx.mod.ControlError.IK_FAILED
    assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C16.0 — report_blocked 为外部非抛出入口, 调用后状态机进 BLOCKED、记录 reason、零发送.
# 方法缺失 -> hasattr 断言失败 (RED).
# ---------------------------------------------------------------------------
def test_t_c16_0_report_blocked_is_external_non_throwing_entry() -> None:
    """report_blocked 为外部非抛出入口：进 BLOCKED、记录 reason、零发送。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_controller([ik_ok(q_delta(0.5))])
    assert hasattr(ctx.controller, "report_blocked")

    ctx.controller.report_blocked("host reported blocked")  # 非抛出 (D015)

    assert ctx.machine.state is ControlState.BLOCKED
    assert "host reported blocked" in (ctx.machine.last_reason or "")
    assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C16.1 — report_blocked 当周期起零发送: ACTIVE(ARM) 被锁存为 BLOCKED、清空阶段,
# 其后 pose 周期不解 IK、不发送. 只置标志不改状态的实现会照常求解发送 -> RED.
# ---------------------------------------------------------------------------
def test_t_c16_1_report_blocked_latches_blocked_and_zero_sends_arm_cycle() -> None:
    """report_blocked 当周期即零发送，其后 pose 周期不解 IK。"""
    from alohamini_cartesian.control.commands import ControlState, Stage

    ctx = make_controller([ik_ok(q_delta(0.5))], stage="arm")
    assert ctx.machine.active_stage is Stage.ARM

    ctx.controller.report_blocked("external block")

    assert ctx.machine.state is ControlState.BLOCKED
    assert ctx.machine.active_stage is None

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert outcome.sent is False
    assert outcome.action is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == [], "BLOCKED 周期不得再求解 IK"


# ---------------------------------------------------------------------------
# T-C16.2 — BLOCKED 不自动恢复, 必须经既有恢复路径 (Release + 停稳窗口) 才回到 READY,
# 恢复后可重新驱动并正常发送. 停稳反馈在未 Release 时即恢复 -> RED.
# ---------------------------------------------------------------------------
def test_t_c16_2_report_blocked_recovers_via_existing_release_settle_path() -> None:
    """BLOCKED 不自动恢复，须经 Release 加停稳窗口回到 READY。"""
    from alohamini_cartesian.control.commands import (
        ArmCommand,
        ControlState,
        ReleaseCommand,
        Side,
    )

    ctx = make_controller([ik_ok(q_delta(0.5))])
    feedback = make_feedback(BASE_Q)

    ctx.controller.report_blocked("external block")
    assert ctx.machine.state is ControlState.BLOCKED

    # BLOCKED persists across pose cycles; still zero sends / no IK.
    for _ in range(3):
        out = ctx.controller.tick(feedback, TARGET)
        assert out.sent is False
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []

    # Settled feedback alone must NOT auto-recover (fail-safe): release is required.
    for sample_id in (2, 3):
        ctx.machine.on_feedback(sample_id, settled=True)
        assert ctx.machine.state is ControlState.BLOCKED

    # Existing recovery path: release arms the window, then a settled window -> READY.
    ctx.machine.handle(ReleaseCommand())
    assert ctx.machine.state is ControlState.BLOCKED
    ctx.machine.on_feedback(4, settled=True)
    assert ctx.machine.state is ControlState.READY

    # Re-activate the arm and confirm normal sending resumes.
    ctx.machine.handle(ArmCommand(side=Side.LEFT, dx=0.01, dy=0.0, dz=0.0))
    assert ctx.machine.state is ControlState.ACTIVE
    out = ctx.controller.tick(feedback, TARGET)
    assert out.sent is True
    assert out.error_code is None
    assert len(ctx.sender.calls) == 1


# ---------------------------------------------------------------------------
# T-C18.0 — next_deadline 只读属性: 首次实际发送前为 None; 赋值抛 AttributeError.
# 普通可写属性 -> pytest.raises(AttributeError) 失败 (RED); 缺属性 -> hasattr 失败.
# ---------------------------------------------------------------------------
def test_t_c18_0_next_deadline_is_read_only_and_unset_before_first_send() -> None:
    """next_deadline 为只读属性：首次发送前 None，赋值抛 AttributeError。"""
    ctx = make_cycle6_controller([ik_ok(q_delta(0.5))], keepalive_interval_s=KEEPALIVE_INTERVAL_S)

    assert hasattr(ctx.controller, "next_deadline")
    assert ctx.controller.next_deadline is None
    with pytest.raises(AttributeError):
        ctx.controller.next_deadline = 123.0


# ---------------------------------------------------------------------------
# T-C18.1 — deadline = 上次实际发送时刻 (注入 clock) + 注入安全间隔.
# 用真实墙钟的实现会得到 ~1e5, 与 1000.x 不符 -> RED.
# ---------------------------------------------------------------------------
def test_t_c18_1_deadline_is_last_actual_send_time_plus_injected_interval() -> None:
    """Deadline 等于上次实际发送时刻加注入的安全间隔。"""
    ctx = make_cycle6_controller(
        [ik_ok(q_delta(0.5)), ik_ok(q_delta(0.5))],
        keepalive_interval_s=KEEPALIVE_INTERVAL_S,
        clock_start=KEEPALIVE_CLOCK_START,
    )
    feedback = make_feedback(BASE_Q)

    assert ctx.controller.next_deadline is None

    o1 = ctx.controller.tick(feedback, TARGET)  # 实际发送 @ t=1000.0
    assert o1.sent is True
    assert ctx.controller.next_deadline == pytest.approx(
        KEEPALIVE_CLOCK_START + KEEPALIVE_INTERVAL_S
    )

    ctx.clock.advance(0.3)
    o2 = ctx.controller.tick(feedback, TARGET)  # 实际发送 @ t=1000.3
    assert o2.sent is True
    assert ctx.controller.next_deadline == pytest.approx(
        KEEPALIVE_CLOCK_START + 0.3 + KEEPALIVE_INTERVAL_S
    )


# ---------------------------------------------------------------------------
# T-C18.2 — 安全间隔来自注入参数而非硬编码: 0.25 与 2.0 必须给出不同 deadline.
# 硬编码同一常数的实现两个 deadline 相等 -> RED.
# ---------------------------------------------------------------------------
def test_t_c18_2_deadline_tracks_injected_interval_not_hardcoded() -> None:
    """安全间隔来自注入参数而非硬编码：不同间隔给出不同 deadline。"""
    ctx_short = make_cycle6_controller(
        [ik_ok(q_delta(0.5))], keepalive_interval_s=0.25, clock_start=10.0
    )
    ctx_long = make_cycle6_controller(
        [ik_ok(q_delta(0.5))], keepalive_interval_s=2.0, clock_start=10.0
    )

    ctx_short.controller.tick(make_feedback(BASE_Q), TARGET)
    ctx_long.controller.tick(make_feedback(BASE_Q), TARGET)

    assert ctx_short.controller.next_deadline == pytest.approx(10.25)
    assert ctx_long.controller.next_deadline == pytest.approx(12.0)
    assert ctx_short.controller.next_deadline != ctx_long.controller.next_deadline


# ---------------------------------------------------------------------------
# T-C18.3 — 被拒周期 (无实际发送) 不推进 deadline.
# 每周期刷新 deadline 的实现会把 1000.5 改成 1005.5 -> RED.
# ---------------------------------------------------------------------------
def test_t_c18_3_rejected_cycle_does_not_advance_deadline() -> None:
    """被拒周期没有实际发送，不推进 deadline。"""
    ctx = make_cycle6_controller(
        [ik_ok(q_delta(0.5)), ik_ok(q_delta(3.0))],
        keepalive_interval_s=KEEPALIVE_INTERVAL_S,
        clock_start=KEEPALIVE_CLOCK_START,
    )
    feedback = make_feedback(BASE_Q)

    ctx.controller.tick(feedback, TARGET)  # 发送 @ 1000.0
    expected = KEEPALIVE_CLOCK_START + KEEPALIVE_INTERVAL_S

    ctx.clock.advance(5.0)
    o2 = ctx.controller.tick(feedback, TARGET)  # 步长 3.0 > 2.0 -> 拒绝, 零发送
    assert o2.sent is False
    assert o2.error_code is ctx.mod.ControlError.STEP_LIMIT
    assert ctx.controller.next_deadline == pytest.approx(expected), "被拒周期不得推进 deadline"


# ---------------------------------------------------------------------------
# T-C18.4 — dry_run 无实际发送 -> deadline 恒 None (本 cycle 设计: 保活针对真实链路).
# 按「拟发送」武装 deadline 的实现会给出非 None -> RED.
# ---------------------------------------------------------------------------
def test_t_c18_4_dry_run_never_arms_deadline() -> None:
    """dry_run 无实际发送，deadline 恒为 None。"""
    ctx = make_cycle6_controller(
        [ik_ok(q_delta(0.5)), ik_ok(q_delta(0.5))],
        keepalive_interval_s=KEEPALIVE_INTERVAL_S,
        dry_run=True,
    )
    feedback = make_feedback(BASE_Q)

    for _ in range(2):
        ctx.clock.advance(1.0)
        out = ctx.controller.tick(feedback, TARGET)
        assert out.sent is False
        assert out.error_code is None  # 已算出拟发送动作
        assert ctx.controller.next_deadline is None, "dry_run 无实际发送不得武装 deadline"
    assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C18.5 — 任何实际发送 (含底盘显式零速) 都武装 deadline (「上次实际发送时刻」泛化).
# 仅在 arm 路径更新 deadline 的实现会在此为 None -> RED (seam 待主 Agent 裁定).
# ---------------------------------------------------------------------------
def test_t_c18_5_actual_base_send_also_arms_deadline() -> None:
    """任何实际发送（含底盘显式零速）都武装 deadline。"""
    ctx = make_cycle6_controller(
        [],
        stage="base",
        keepalive_interval_s=KEEPALIVE_INTERVAL_S,
        clock_start=KEEPALIVE_CLOCK_START,
    )

    assert ctx.controller.next_deadline is None
    out = ctx.controller.tick(make_feedback(BASE_Q), base_command(x=0.1, y=0.0, turn=0.0))
    assert out.sent is True
    assert len(ctx.sender.calls) == 1
    assert ctx.controller.next_deadline == pytest.approx(
        KEEPALIVE_CLOCK_START + KEEPALIVE_INTERVAL_S
    )


# ===========================================================================
# P4-B cycle 6-fix —— T-C16 report_blocked 在 FAULT 下不降级 (appended; 既有 98 例未动)
#
# 缺陷事实 (tester 实测): controller.report_blocked 直写 ControlStateMachine 私有
# 属性; FAULT 状态下调用把 FAULT 降级为 BLOCKED, 随后 release+停稳直达 READY,
# 绕过 D016「唯一恢复入口 reset()」。修复后 report_blocked 迁移到公开 `machine.block(reason)`,
# FAULT 下为锁存 no-op。
#
# This binding discriminates the reasonable-but-WRONG implementation:
# - report_blocked 仍直写 `_state = BLOCKED` 绕过 machine.block -> T-C16.3 状态降级;
# - FAULT 降级后 ReleaseCommand 重新武装恢复窗口 -> T-C16.4 ReleaseCommand 不再抛;
# - 故障原因被 block reason 覆盖 -> T-C16.3 reason 断言失败。
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.
# ===========================================================================


def _faulted_cycle6_controller() -> SimpleNamespace:
    """构造 ACTIVE(ARM) 控制器后经 feedback 倒退驱动状态机进入 FAULT (设计 §6 ACTIVE->FAULT)。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5))])
    # make_cycle6_controller 已消费 id=1 (高水位). 未见过的更小 id 在 ACTIVE 下 => FAULT.
    ctx.machine.on_feedback(0, settled=True)
    assert ctx.machine.state is ControlState.FAULT, ctx.machine.state
    return ctx


# ---------------------------------------------------------------------------
# T-C16.3 — FAULT 下 report_blocked 为锁存 no-op: 保持 FAULT (不降级 BLOCKED)、
# 保留故障原因、且控制器后续 pose 周期仍 fail-closed (零发送、不解 IK)。
# 直写私有属性的实现 -> 状态变为 BLOCKED -> 断言失败。
# ---------------------------------------------------------------------------
def test_t_c16_3_report_blocked_does_not_downgrade_fault() -> None:
    """FAULT 下 report_blocked 为锁存 no-op：不降级、保留原因、仍 fail-closed。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = _faulted_cycle6_controller()
    reason_before = ctx.machine.last_reason
    assert isinstance(reason_before, str) and reason_before

    ctx.controller.report_blocked("external block")  # 必须非抛出

    assert ctx.machine.state is ControlState.FAULT
    assert ctx.machine.state is not ControlState.BLOCKED
    assert ctx.machine.last_reason == reason_before  # 故障原因保持可见

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert outcome.sent is False
    assert outcome.action is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C16.4 — FAULT 下 report_blocked 后不得为绕过路径打开恢复捷径: ReleaseCommand
# 仍被 MotionBlocked 拒绝, 停稳反馈也不得把状态机拉回 READY (D016 唯一 reset() 恢复)。
# 降级为 BLOCKED 的实现会让 ReleaseCommand 武装窗口 -> pytest.raises 失败。
# ---------------------------------------------------------------------------
def test_t_c16_4_report_blocked_fault_opens_no_recovery_shortcut() -> None:
    """FAULT 下 report_blocked 不打开恢复捷径：Release 仍被拒、停稳不回 READY。"""
    from alohamini_cartesian.control.commands import (
        ControlState,
        MotionBlocked,
        ReleaseCommand,
    )

    ctx = _faulted_cycle6_controller()

    ctx.controller.report_blocked("external block")
    assert ctx.machine.state is ControlState.FAULT

    with pytest.raises(MotionBlocked):
        ctx.machine.handle(ReleaseCommand())

    for sample_id in range(2, 8):
        ctx.machine.on_feedback(sample_id, settled=True)
        assert ctx.machine.state is ControlState.FAULT

    assert ctx.machine.state is not ControlState.READY
    assert ctx.machine.state is not ControlState.BLOCKED


# ===========================================================================
# P4-B cycle 7 —— T-C12 控制器层: tick 反馈倒退接线 -> FAULT 锁存当周期零发送
# (appended; 既有 100 例未动)
#
# 裁定: tick 中反馈经 `machine.on_feedback` 接线后, 反馈样本 id 倒退会锁存 FAULT;
# 控制器在同一周期即 fail-closed 零发送 (不求解 IK、不调用 sender), 后续周期保持
# 零发送直到 reset()。既有 T-C16.3 已覆盖「外部锁存 FAULT -> tick 零发送」, 本
# cycle 新增的是「FAULT 由 tick 自身的反馈摄取触发」这条接线路径 (如实区分: 状态
# 门禁的零发送部分与既有行为重叠, 触发来源不同)。
#
# 本 binding 有意区分以下 reasonable-but-WRONG 实现:
# - tick 不摄取反馈 sample_id (无 on_feedback 接线) -> 状态仍 ACTIVE, 状态断言失败;
# - 摄取/锁存发生在发送或求解之后 -> 当周期已发送, sender 断言失败;
# - 锁存 FAULT 但仍继续求解/发送 -> sender/ik 断言失败;
# - FAULT 随下一正向样本自愈 -> 后续周期断言失败。
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.
# 期望值为冻结字面量, 不从被测实现回读.
# ===========================================================================

# make_cycle6_controller 构造时已消费 id=1 作为高水位; 0 是未见过且低于高水位的 id.
CYCLE7_SAMPLE_ID = 0


# ---------------------------------------------------------------------------
# T-C12.6 — tick 摄取反馈 sample_id 倒退 -> 锁存 FAULT, 且**当周期**即零发送
# (不求解 IK、不调用 sender)。无接线实现 -> 状态仍 ACTIVE 且已发送 -> 断言失败;
# 先发送后锁存 -> sender 断言失败。
# ---------------------------------------------------------------------------
def test_t_c12_tick_feedback_regression_latches_fault_and_zero_sends_this_cycle() -> None:
    """Tick 摄取反馈序号倒退：当周期即锁存 FAULT 并零发送。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5))])
    assert ctx.machine.state is ControlState.ACTIVE  # 前提: 控制器处于 ACTIVE(ARM)

    outcome = ctx.controller.tick(make_feedback(BASE_Q, sample_id=CYCLE7_SAMPLE_ID), TARGET)

    assert ctx.machine.state is ControlState.FAULT
    assert ctx.machine.state is not ControlState.ACTIVE
    assert ctx.machine.active_stage is None
    assert outcome.sent is False
    assert outcome.action is None
    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ---------------------------------------------------------------------------
# T-C12.7 — 由 tick 反馈倒退锁存的 FAULT 不随后续正向有效反馈自愈: 后续周期保持
# FAULT 且继续零发送 (与 T-C16.3 的锁存语义一致, 只是触发来源不同)。
# ---------------------------------------------------------------------------
def test_t_c12_tick_feedback_regression_keeps_fault_and_zero_sends_next_cycles() -> None:
    """Tick 锁存的 FAULT 不随后续有效反馈自愈，继续零发送。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5)), ik_ok(q_delta(1.0)), ik_ok(q_delta(1.5))])
    ctx.controller.tick(make_feedback(BASE_Q, sample_id=CYCLE7_SAMPLE_ID), TARGET)
    assert ctx.machine.state is ControlState.FAULT

    for sample_id in (2, 3):  # 正向有效反馈也不得自愈
        out = ctx.controller.tick(make_feedback(BASE_Q, sample_id=sample_id), TARGET)
        assert ctx.machine.state is ControlState.FAULT
        assert out.sent is False
        assert out.action is None

    assert ctx.sender.calls == []
    assert ctx.ik.calls == []


# ===========================================================================
# P4-B cycle 8 (最终) —— T-C11 IK 调用契约 + T-C17 真 ArmIK 集成 + 会话切换
# (appended; 既有 102 例未动)
# ===========================================================================
#
# Required behavior (acceptance, 本 cycle 冻结):
# - T-C11 controller -> ArmIK.solve 调用契约 (真 JointKinematicAdapter 接线):
#     * q 种子为 adapter 转换后的 kinematic deg, shape (6,);
#     * joint_limits_deg 必须来自 `adapter.effective_limits_deg` 的「标定 ∩ URDF」交集
#       (design §12.4), 严禁 ±180° / (-inf, inf) 占位; 且随注入的 URDF 窗口变化;
#     * 必须向 solve 转发 `orientation_weight` (D006 软姿态权重, 设计默认 0.01).
# - T-C17 真 ArmIK (placo) + 真 adapter 完整 tick 集成:
#     * 可解小目标 (FK 位姿 + 5 mm) -> 接受并发送恰 6 键机械臂动作; 由动作独立反解出的
#       q 经真 FK 必须落在残差门限内 (证明整链真的走到了真 IK + 真 adapter 转换);
#     * 真 IK 判定不可达的目标 -> 控制器拒绝, 零发送.
# - 会话切换 (已裁定): controller 检测 `_control_status.host_session_id` 变化 -> 视为新
#   会话 (重置反馈跟踪 / 已知 id 状态), 新会话首帧的低 sample_sequence 不得误判 FAULT;
#   同会话内 sample_id 倒退仍锁存 FAULT (D016 不回退).
#
# 冻结 / 建议 seam (待主 Agent 固化):
#   - `orientation_weight` 由控制器在 solve 调用中以关键字转发 (设计默认 0.01, D006);
#     本 cycle 只断言 solve 入参, 不绑定控制器构造参数形态.
#   - 会话切换最小语义: controller 记住最近一次 feedback 的 host_session_id, 变化时清除
#     「已知 id / 反馈跟踪」基线. 实现可选 `machine.reset()` 或新增仅清反馈证据的入口;
#     本 cycle 只断言「新会话低序号不误判 FAULT / 同会话倒退仍 FAULT / 新基线被重建」,
#     不绑定状态机是否被重置到 STANDBY.
#
# 本 binding 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - solve 调用省略 orientation_weight (现状) -> T-C11.3;
# - joint_limits_deg 用 ±180° / (-inf, inf) 占位或硬编码 -> T-C11.1 / T-C11.2;
# - 直接转发 IK q 而不经 adapter 转换 -> T-C17.1 (动作键 / 独立反解真 FK 残差);
# - 忽略真 IK 失败照常发送 -> T-C17.2;
# - 忽略 host_session_id, 把新会话低序号当倒退 (现状: FAULT) -> T-C13.1;
# - 每帧都清空已知 id (而非仅会话切换) -> T-C13.2 同会话倒退不再 FAULT;
# - 会话切换只重置状态机却保留旧 id 高水位 -> T-C13.3 (旧高水位下仍误判).
#
# Pure logic + real placo kinematics only (T-C17): no hardware / network / clock /
# randomness. T-C11 / T-C13 经 make_cycle6_controller 注入 FakeClock.
# ===========================================================================

# adapter URDF 关节名映射 (冻结字面量, 不从被测模块回读).
URDF_JOINT_NAMES_FROZEN: dict[str, str] = {
    "shoulder_pan": "left_shoulder_pan",
    "shoulder_lift": "left_shoulder_lift",
    "elbow_flex": "left_elbow_flex",
    "wrist_flex": "left_wrist_flex",
    "wrist_yaw": "left_wrist_yaw_joint",
    "wrist_roll": "left_wrist_roll",
}

# 两组互异、且都明显偏离 ±180° 的 URDF 窗口 (证明限位随注入窗口变化, 非占位).
CYCLE8_URDF_LIMITS_A: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-95.0, 95.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-110.0, 110.0),
    "wrist_flex": (-120.0, 120.0),
    "wrist_yaw": (-130.0, 130.0),
    "wrist_roll": (-140.0, 140.0),
}
CYCLE8_URDF_LIMITS_B: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-80.0, 80.0),
    "shoulder_lift": (-85.0, 85.0),
    "elbow_flex": (-90.0, 90.0),
    "wrist_flex": (-95.0, 95.0),
    "wrist_yaw": (-105.0, 105.0),
    "wrist_roll": (-115.0, 115.0),
}

CYCLE8_ORIENTATION_WEIGHT = 0.01  # D006 / ArmIK.solve 设计默认

_URDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "alohamini_cartesian"
    / "kinematics"
    / "assets"
    / "am_arm200"
    / "kinematics.urdf"
)
REAL_TARGET_FRAME = "left_Fixed_Jaw"
REAL_JOINT_NAMES_FROZEN: tuple[str, ...] = tuple(URDF_JOINT_NAMES_FROZEN[j] for j in JOINT_ORDER)
REAL_RESIDUAL_LIMIT_M = 0.005
REAL_SMALL_TARGET_DZ_M = 0.005


def make_adapter_with_urdf_limits(
    windows: dict[str, tuple[float, float]], *, side: str = CTRL_SIDE
) -> object:
    """真 JointKinematicAdapter, 注入逐关节 URDF 窗口 (DEGREES 模式下限位 = URDF 窗口)."""
    from alohamini_cartesian.kinematics.joint_adapter import JointKinematicAdapter

    return JointKinematicAdapter(
        signs={side: dict(CTRL_SIGNS)},
        zeros={side: dict(CTRL_ZEROS)},
        urdf_limits={side: {joint: windows[joint] for joint in JOINT_ORDER}},
    )


def expected_intersection_limits(
    windows: dict[str, tuple[float, float]],
) -> dict[str, tuple[float, float]]:
    """Independent expectation of the solve-side limits (URDF joint names -> window)."""
    return {
        URDF_JOINT_NAMES_FROZEN[joint]: (float(windows[joint][0]), float(windows[joint][1]))
        for joint in JOINT_ORDER
    }


def make_real_arm_ik() -> object:
    """Real placo-backed ArmIK for the frozen left-arm URDF / target frame."""
    from alohamini_cartesian.control.arm_ik import ArmIK, ArmKinematicProfile

    return ArmIK(
        ArmKinematicProfile(
            urdf_path=_URDF_PATH,
            target_frame=REAL_TARGET_FRAME,
            joint_names=REAL_JOINT_NAMES_FROZEN,
        )
    )


def solve_kwargs(ctx: SimpleNamespace) -> dict:
    """Return the kwargs of the single recorded solve call (asserts exactly one)."""
    assert len(ctx.ik.calls) == 1, ctx.ik.calls
    return ctx.ik.calls[0]["kwargs"]


def make_session_feedback(q_deg: object, *, session: str, seq: int, side: str = CTRL_SIDE) -> dict:
    """Feedback with an explicit host_session_id + sample_sequence (session-switch cases)."""
    feedback = make_feedback(q_deg, side=side, sample_id=seq)
    feedback["_control_status"]["host_session_id"] = session
    return feedback


# ---------------------------------------------------------------------------
# T-C11.1 — 真 adapter 接线: solve 的 q 种子为 shape-(6,) 的 adapter 转换 deg, 且限位为
# 「标定 ∩ URDF」交集 (非 ±180° / 非无界占位).
# 占位限位 / 直接转发 observation 会在此被数值断言击杀.
# ---------------------------------------------------------------------------
def test_t_c11_1_solve_contract_seed_shape_and_adapter_intersection_limits() -> None:
    """Solve 的 q 种子为 shape-(6,) 转换 deg，限位为标定与 URDF 的交集。"""
    adapter = make_adapter_with_urdf_limits(CYCLE8_URDF_LIMITS_A)
    ctx = make_cycle6_controller([ik_ok(q_delta(0.5))], adapter=adapter)

    outcome = ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    assert outcome.sent is True
    kwargs = solve_kwargs(ctx)

    seed = extract_seed(ctx.ik.calls[0])
    assert seed.shape == (6,)
    assert seed == pytest.approx(np.asarray(BASE_Q, dtype=float), abs=1e-9)

    limits = kwargs["joint_limits_deg"]
    expected = expected_intersection_limits(CYCLE8_URDF_LIMITS_A)
    assert set(limits) == set(expected)
    for joint, window in expected.items():
        assert tuple(limits[joint]) == pytest.approx(window, abs=1e-9), (
            joint,
            limits[joint],
            window,
        )

    # 显式排除 ±180° 与该模式下未求交的 (-inf, inf) 占位.
    assert all(tuple(window) != (-180.0, 180.0) for window in limits.values())
    assert all(np.isfinite(window[0]) and np.isfinite(window[1]) for window in limits.values())


# ---------------------------------------------------------------------------
# T-C11.2 — solve 限位来自 adapter 的注入窗口而非硬编码/占位: 两组不同窗口必须得到两组
# 不同且各自精确的限位. 硬编码 ±180° 或忽略 adapter 的实现会两处相等或数值不符.
# ---------------------------------------------------------------------------
def test_t_c11_2_solve_limits_track_injected_adapter_windows_not_placeholder() -> None:
    """Solve 限位随注入的 adapter URDF 窗口变化，而非占位或硬编码。"""
    ctx_a = make_cycle6_controller(
        [ik_ok(q_delta(0.5))], adapter=make_adapter_with_urdf_limits(CYCLE8_URDF_LIMITS_A)
    )
    ctx_b = make_cycle6_controller(
        [ik_ok(q_delta(0.5))], adapter=make_adapter_with_urdf_limits(CYCLE8_URDF_LIMITS_B)
    )

    ctx_a.controller.tick(make_feedback(BASE_Q), TARGET)
    ctx_b.controller.tick(make_feedback(BASE_Q), TARGET)

    limits_a = solve_kwargs(ctx_a)["joint_limits_deg"]
    limits_b = solve_kwargs(ctx_b)["joint_limits_deg"]

    assert {j: tuple(v) for j, v in limits_a.items()} != {j: tuple(v) for j, v in limits_b.items()}
    for joint, window in expected_intersection_limits(CYCLE8_URDF_LIMITS_A).items():
        assert tuple(limits_a[joint]) == pytest.approx(window, abs=1e-9)
    for joint, window in expected_intersection_limits(CYCLE8_URDF_LIMITS_B).items():
        assert tuple(limits_b[joint]) == pytest.approx(window, abs=1e-9)


# ---------------------------------------------------------------------------
# T-C11.3 — 控制器必须向 solve 转发 orientation_weight (D006 软姿态权重, 默认 0.01).
# 现状 (solve 只收 q_current_deg/target_pose/joint_limits_deg) -> 缺键 RED.
# ---------------------------------------------------------------------------
def test_t_c11_3_solve_receives_orientation_weight() -> None:
    """控制器必须向 solve 转发 orientation_weight（默认 0.01）。"""
    adapter = make_adapter_with_urdf_limits(CYCLE8_URDF_LIMITS_A)
    ctx = make_cycle6_controller([ik_ok(q_delta(0.5))], adapter=adapter)

    ctx.controller.tick(make_feedback(BASE_Q), TARGET)

    kwargs = solve_kwargs(ctx)
    assert "orientation_weight" in kwargs, "controller 未向 ArmIK.solve 转发 orientation_weight"
    assert kwargs["orientation_weight"] == pytest.approx(CYCLE8_ORIENTATION_WEIGHT)


# ---------------------------------------------------------------------------
# T-C17.1 — 真 ArmIK + 真 adapter 完整 tick: 可解小目标 (FK + 5mm) 被接受, 发送恰 6 键
# 机械臂动作; 由动作独立反解出的 q 经真 FK 落在残差门限内 (证明真链真的走到 IK+adapter).
# 直接转发 IK q / 不接 adapter 的实现会在反解 FK 残差或动作键上失败.
# ---------------------------------------------------------------------------
def test_t_c17_1_real_arm_ik_and_adapter_full_tick_produces_converted_six_key_action() -> None:
    """真 ArmIK 加真 adapter 完整 tick：小目标被接受且反解 FK 落在残差内。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    ctx = make_cycle6_controller(
        adapter=make_adapter_with_urdf_limits(CYCLE8_URDF_LIMITS_A), ik=make_real_arm_ik()
    )
    q_seed = np.asarray(BASE_Q, dtype=float)
    target = np.copy(ctx.ik.fk(q_seed))
    target[2, 3] += REAL_SMALL_TARGET_DZ_M

    outcome = ctx.controller.tick(make_feedback(BASE_Q), target)

    assert outcome.sent is True
    assert outcome.error_code is None
    assert outcome.action is not None
    expected_keys = {f"arm_{CTRL_SIDE}_{joint}.pos" for joint in JOINT_ORDER}
    assert set(outcome.action) == expected_keys
    assert len(ctx.sender.calls) == 1
    assert ctx.sender.calls[0] == dict(outcome.action)

    # 独立反解 (affine 逆) + 真 FK: 发送的动作确实把末端带到目标位姿.
    q_recovered = q_from_action(outcome.action)
    for index, joint in enumerate(JOINT_ORDER):
        lo, hi = CYCLE8_URDF_LIMITS_A[joint]
        assert lo <= q_recovered[index] <= hi, (joint, q_recovered[index], (lo, hi))
    reached = ctx.ik.fk(q_recovered)
    assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) <= REAL_RESIDUAL_LIMIT_M


# ---------------------------------------------------------------------------
# T-C17.2 — 真 ArmIK 判定不可达的目标 (10 m) -> 控制器拒绝 (RESIDUAL/JOINT_LIMIT), 零发送.
# 忽略真 IK 失败照常发送的实现会让 sender 非空 / action 非 None.
# ---------------------------------------------------------------------------
def test_t_c17_2_real_arm_ik_unreachable_target_is_rejected_with_zero_send() -> None:
    """真 ArmIK 判定不可达目标时控制器拒绝且零发送。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    ctx = make_cycle6_controller(
        adapter=make_adapter_with_urdf_limits(CYCLE8_URDF_LIMITS_A), ik=make_real_arm_ik()
    )
    target = np.copy(ctx.ik.fk(np.asarray(BASE_Q, dtype=float)))
    target[0, 3] += 10.0

    outcome = ctx.controller.tick(make_feedback(BASE_Q), target)

    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code in (ctx.mod.ControlError.RESIDUAL, ctx.mod.ControlError.JOINT_LIMIT)
    assert ctx.sender.calls == []


# ---------------------------------------------------------------------------
# T-C13.1 — host_session_id 变化 + 新会话首帧低 sample_sequence (50 < 旧高水位 100)
# 不得误判 FAULT. 现状 (忽略会话 id) 会把 50 当倒退 -> FAULT -> RED.
# ---------------------------------------------------------------------------
def test_t_c13_1_host_session_change_with_lower_sequence_does_not_latch_fault() -> None:
    """换会话后首帧序号变小不得误判 FAULT。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5)) for _ in range(4)])

    ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=100), TARGET)
    assert ctx.machine.state is ControlState.ACTIVE

    ctx.controller.tick(make_session_feedback(BASE_Q, session="s2", seq=50), TARGET)

    assert ctx.machine.state is not ControlState.FAULT
    assert "regression" not in (ctx.machine.last_reason or "")


# ---------------------------------------------------------------------------
# T-C13.2 — 同会话内 sample_id 倒退仍锁存 FAULT (D016 不回退).
# 每帧无条件清空已知 id 的过度重置实现会在此失去倒退检测 -> RED.
# ---------------------------------------------------------------------------
def test_t_c13_2_same_session_sequence_regression_still_latches_fault() -> None:
    """同会话内序号倒退仍锁存 FAULT。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5)) for _ in range(3)])

    ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=100), TARGET)
    assert ctx.machine.state is ControlState.ACTIVE

    outcome = ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=99), TARGET)

    assert ctx.machine.state is ControlState.FAULT
    assert outcome.sent is False
    assert outcome.action is None


# ---------------------------------------------------------------------------
# T-C13.3 — 会话切换必须重建已知 id 基线: 新会话里 52 (> 新基线 50/51, < 旧高水位 100)
# 不得误判 FAULT. 只重置状态机却保留旧高水位的实现会在旧高水位下误判 -> RED.
# 兼容「切会话重置到 STANDBY」与「保留状态只清证据」两种实现 (必要时经状态机公开入口回到
# ACTIVE(ARM)).
# ---------------------------------------------------------------------------
def test_t_c13_3_session_change_rebuilds_known_id_baseline() -> None:
    """会话切换重建已知序号基线，新会话较小序号不误判。"""
    from alohamini_cartesian.control.commands import ArmCommand, ControlState, Side, Stage

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5)) for _ in range(6)])

    ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=100), TARGET)
    ctx.controller.tick(make_session_feedback(BASE_Q, session="s2", seq=50), TARGET)
    assert ctx.machine.state is not ControlState.FAULT

    # 回到 ACTIVE(ARM): 兼容「会话切换重置到 STANDBY」的实现 (保留状态实现下为空操作).
    ctx.controller.tick(make_session_feedback(BASE_Q, session="s2", seq=51), TARGET)
    if ctx.machine.state is ControlState.STANDBY:
        ctx.machine.resume()
    if ctx.machine.state is ControlState.READY:
        ctx.machine.handle(ArmCommand(side=Side.LEFT, dx=0.01, dy=0.0, dz=0.0))
    assert ctx.machine.state is ControlState.ACTIVE
    assert ctx.machine.active_stage is Stage.ARM

    ctx.controller.tick(make_session_feedback(BASE_Q, session="s2", seq=52), TARGET)

    assert ctx.machine.state is not ControlState.FAULT


# ===========================================================================
# P4-B 终审修复轮 M1 / m1 / m3 / i2 (appended; 既有 110 例未动)
# ===========================================================================
#
# Required behavior (acceptance, 本 cycle 冻结):
# - M1 会话切换清理运动基线: controller 检测到 `host_session_id` 变化 (与 `machine.reset()`
#   同一路径) 时, 除了重置状态机反馈证据, 还必须同步清除运动基线
#   `_last_accepted_q` 与跟随计时 `_follow_error_start_time`. 重连后从当周期反馈重建基线,
#   不得把旧会话的「上一被接受命令」当参考而永久拒动, 也不得让旧会话的跟随计时器
#   在新会话里立即误报超时.
# - m1 缺 `host_session_id` 帧 fail-closed: `_control_status` 存在但缺 `host_session_id`
#   的帧必须早退 —— 不触发会话重置 (不把 `None != 旧 id` 当成新会话), 也不得借重置清掉
#   已锁存的 FAULT.
# - m3 adapter 公开 sign 访问器: adapter 暴露 `sign(side, joint)` (返回注入的 ±1 gain),
#   controller 的 hold 释放方向映射改用它, 不再读私有 `_signs`.
# - i2 hold 路径同步复位计时: `_handle_overcurrent_hold` 清 `_last_accepted_q` 时必须同步清
#   `_follow_error_start_time`, 否则旧计时器残留, hold 结束后首个新跟随误差即误报超时.
#
# 本 binding 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一个用例捕获):
# - 会话切换只重置状态机却保留 `_last_accepted_q` -> T-C13.4 (重连位移被误判 STEP_LIMIT);
# - 会话切换只重置状态机却保留 `_follow_error_start_time` -> T-C13.5 (新会话立即超时);
# - 缺 `host_session_id` 被当作会话切换做 machine.reset -> T-C13.6 (FAULT 被清);
# - 缺 `host_session_id` 触发会话重置 -> T-C13.7 (ACTIVE 被踢到 STANDBY);
# - hold 退让仍读私有 `_signs` -> T-C10.8 (仅暴露 sign 访问器的 adapter 替身 -> AttributeError);
# - hold 清 `_last_accepted_q` 但漏清 `_follow_error_start_time` -> T-C8.6 (误报 FOLLOW_TIMEOUT).
#
# Pure logic only: 注入 FakeClock, 无真实时钟 / sleep / IO / 随机 / 硬件.
# ===========================================================================


def _resume_active_arm(ctx: SimpleNamespace) -> None:
    """会话切换重置到 STANDBY 后, 经既有恢复路径重新进入 ACTIVE(ARM)."""
    from alohamini_cartesian.control.commands import (
        ArmCommand,
        ControlState,
        Side,
        Stage,
    )

    if ctx.machine.state is ControlState.STANDBY:
        ctx.machine.resume()
    if ctx.machine.state is ControlState.READY:
        ctx.machine.handle(ArmCommand(side=Side.LEFT, dx=0.01, dy=0.0, dz=0.0))
    assert ctx.machine.state is ControlState.ACTIVE
    assert ctx.machine.active_stage is Stage.ARM


# ---------------------------------------------------------------------------
# T-C13.4 (M1) — 会话切换清除 `_last_accepted_q`: 重连到位移后的位置不得被旧基线误拒.
# 周期1 接受 BASE_Q+1.0 (旧基线). 会话切换后反馈位移 10°, IK 解 = 当前反馈 (零步长).
# 保留旧基线的实现: 步长 = 9° > 2° -> 误判 STEP_LIMIT 永久拒动 -> RED.
# ---------------------------------------------------------------------------
def test_t_c13_4_session_change_clears_last_accepted_q_reconnect_displacement_accepted() -> None:
    """会话切换清除步长基线，重连位移不被误判 STEP_LIMIT。"""
    from alohamini_cartesian.control.commands import ControlState

    moved = np.array(BASE_Q, dtype=float)
    moved[0] += 10.0  # 重连后机器人已移动的位置
    ctx = make_cycle6_controller([ik_ok(q_delta(1.0)), ik_ok(moved)], step_deg=2.0)

    # 周期1: 接受 +1.0, 旧基线 = BASE_Q + 1.0.
    o1 = ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=100), TARGET)
    assert o1.sent is True
    assert o1.error_code is None

    # 会话切换: controller 与 machine.reset() 同步, 必须一并清运动基线.
    ctx.controller.tick(make_session_feedback(moved, session="s2", seq=1), TARGET)
    assert ctx.machine.state is ControlState.STANDBY
    _resume_active_arm(ctx)

    # 新会话首帧: IK 解 = 当前反馈 (相对反馈零步长). 旧基线残留 -> 9° 步长被误拒.
    o2 = ctx.controller.tick(make_session_feedback(moved, session="s2", seq=2), TARGET)
    assert o2.error_code is None, "会话切换后仍沿用旧 _last_accepted_q -> 重连位移被误判 STEP_LIMIT"
    assert o2.sent is True
    assert len(ctx.sender.calls) == 2


# ---------------------------------------------------------------------------
# T-C13.5 (M1) — 会话切换清除 `_follow_error_start_time`: 新会话首个跟随误差从当前时刻
# 重新计时, 不继承旧会话起点. 周期2 (t=0.3) 起计时; 会话切换后 (t=1.01/1.02) 出现新误差.
# 旧计时器残留 -> 1.02-0.3=0.72>0.4 -> 立即误报 FOLLOW_TIMEOUT -> RED.
# ---------------------------------------------------------------------------
def test_t_c13_5_session_change_clears_follow_error_timer() -> None:
    """会话切换清除跟随计时起点，新会话首个误差不立即超时。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5)) for _ in range(4)],
        step_deg=2.0,
        lead_deg=30.0,
        follow_tolerance_deg=0.1,
        follow_timeout_s=0.4,
    )

    # 周期1 t=0: 接受 +0.5.
    o1 = ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=10), TARGET)
    assert o1.sent is True

    # 周期2 t=0.3: 反馈滞后, 误差 0.5 > tol -> 计时起点 t=0.3.
    ctx.clock.advance(0.3)
    o2 = ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=11), TARGET)
    assert o2.sent is True
    assert o2.error_code is None

    # 周期3 t=1.0: 会话切换 (STANDBY). 修复须清 last 与计时器.
    ctx.clock.advance(0.7)
    ctx.controller.tick(make_session_feedback(BASE_Q, session="s2", seq=1), TARGET)
    _resume_active_arm(ctx)

    # 周期4 t=1.01: 新会话正常接受, 重建 last.
    ctx.clock.advance(0.01)
    o4 = ctx.controller.tick(make_session_feedback(BASE_Q, session="s2", seq=2), TARGET)
    assert o4.sent is True, "会话切换未清跟随计时/基线 -> 新会话周期即误报 FOLLOW_TIMEOUT"
    assert o4.error_code is None

    # 周期5 t=1.02: 反馈仍滞后 -> 新跟随误差. 修复后应从 t=1.02 起计时, 不超时.
    ctx.clock.advance(0.01)
    o5 = ctx.controller.tick(make_session_feedback(BASE_Q, session="s2", seq=3), TARGET)
    assert o5.error_code is None, (
        "会话切换未清 _follow_error_start_time -> 新会话首次误差即误报超时"
    )
    assert o5.sent is True


# ---------------------------------------------------------------------------
# T-C13.6 (m1) — 缺 `host_session_id` 的帧不得清 FAULT 锁存 (fail-closed).
# 同会话序号倒退锁存 FAULT 后, 一帧无 host_session_id 的 `_control_status` 不得被当作
# 会话切换而 machine.reset() 把 FAULT 抹掉 (当前实现: reset -> STANDBY -> RED).
# ---------------------------------------------------------------------------
def test_t_c13_6_missing_host_session_id_frame_does_not_clear_fault_latch() -> None:
    """缺 host_session_id 的帧不得清除 FAULT 锁存。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5))])

    ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=100), TARGET)
    assert ctx.machine.state is ControlState.ACTIVE

    # 同会话内序号倒退 -> FAULT 锁存.
    ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=50), TARGET)
    assert ctx.machine.state is ControlState.FAULT

    # 缺 host_session_id 的帧: fail-closed, 早退, 不得清 FAULT.
    feedback = make_feedback(BASE_Q, sample_id=101)
    del feedback["_control_status"]["host_session_id"]
    outcome = ctx.controller.tick(feedback, TARGET)

    assert ctx.machine.state is ControlState.FAULT, "缺 host_session_id 的帧清除了 FAULT 锁存"
    assert outcome.sent is False
    assert outcome.action is None


# ---------------------------------------------------------------------------
# T-C13.7 (m1) — 缺 `host_session_id` 的帧不得触发会话重置 (从 ACTIVE 掉到 STANDBY).
# 当前实现把 `None != 旧 id` 当会话切换 -> reset -> STANDBY -> RED. fail-closed 须保持 ACTIVE.
# 断言对「完全不 ingest 该帧」(早退) 与「只跳过会话判断但仍 ingest」两种修复都稳健.
# ---------------------------------------------------------------------------
def test_t_c13_7_missing_host_session_id_frame_does_not_trigger_session_reset() -> None:
    """缺 host_session_id 的帧不得触发会话重置。"""
    from alohamini_cartesian.control.commands import ControlState

    ctx = make_cycle6_controller([ik_ok(q_delta(0.5)) for _ in range(2)])

    ctx.controller.tick(make_session_feedback(BASE_Q, session="s1", seq=100), TARGET)
    assert ctx.machine.state is ControlState.ACTIVE

    feedback = make_session_feedback(BASE_Q, session="s1", seq=101)
    del feedback["_control_status"]["host_session_id"]
    ctx.controller.tick(feedback, TARGET)

    assert ctx.machine.state is ControlState.ACTIVE, "缺 host_session_id 的帧触发了会话重置"


class SignAccessorAdapter:
    """adapter 替身: 仅公开 `sign()` 访问器 (无 `_signs`), 其余方法委托真 adapter.

    用于证明 controller 的 hold 释放方向映射经公开 `sign(side, joint)` 取 sign, 而非读私有
    `_signs`. 当前实现在 hold 路径读 `_signs` -> AttributeError -> RED.
    """

    def __init__(self, real: object) -> None:
        """保存被包装的真实 adapter。"""
        self._real = real

    def observation_to_kinematic_deg(
        self, side: str, feedback: dict, metadata: dict
    ) -> dict[str, float]:
        """委托真实 adapter 执行观测转换。"""
        return self._real.observation_to_kinematic_deg(side, feedback, metadata)

    def effective_limits_deg(self, side: str, metadata: dict) -> dict:
        """委托真实 adapter 读取有效限位。"""
        return self._real.effective_limits_deg(side, metadata)

    def kinematic_deg_to_action(
        self, side: str, q_deg: np.ndarray, metadata: dict
    ) -> dict[str, float]:
        """委托真实 adapter 把 kinematic deg 转成动作。"""
        return self._real.kinematic_deg_to_action(side, q_deg, metadata)

    def sign(self, side: str, joint: str) -> float:
        """暴露公开符号增益访问器（替代私有 _signs）。"""
        return self._real.sign(side, joint)


# ---------------------------------------------------------------------------
# T-C10.8 (m3) — hold 退让必须经 adapter 公开 `sign()` 访问器取 sign, 不得读私有 `_signs`.
# 注入一个只暴露 `sign()` (无 `_signs`) 的 adapter 替身: 读私有属性的实现 AttributeError;
# 满足接受的退让方向仍须正确 (shoulder_lift sign=-1, blocked=+1 -> 运动学释放 +1).
# ---------------------------------------------------------------------------
def test_t_c10_8_hold_retreat_uses_public_sign_accessor_not_private_signs() -> None:
    """Hold 退让经 adapter 公开 sign() 访问器，不读私有 _signs。"""
    wrapper = SignAccessorAdapter(make_adapter())
    joint = "shoulder_lift"  # CTRL_SIGNS[joint] == -1.0
    ctx = make_cycle6_controller([ik_ok(q_delta(1.9, index=1))], step_deg=1.0, adapter=wrapper)
    feedback = feedback_with_holds(
        BASE_Q,
        joint_holds={f"arm_left_{joint}": make_hold_entry(blocked_direction=1.0)},
    )

    outcome = ctx.controller.tick(feedback, TARGET)

    assert outcome.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD
    assert len(ctx.sender.calls) == 1
    delta = q_from_action(ctx.sender.calls[0])[1] - BASE_Q[1]
    assert np.sign(delta) == 1.0, "hold 退让方向错误 (未经公开 sign 访问器映射)"
    assert abs(delta) <= 1.0 + 1e-9


# ---------------------------------------------------------------------------
# T-C8.6 (i2) — hold 清 `_last_accepted_q` 时须同步清 `_follow_error_start_time`.
# 周期2 (t=0.3) 起计时; 周期3 hold; 周期4 重建 last; 周期5 (t=1.02) 出现新误差.
# 旧计时器残留 -> 1.02-0.3=0.72>0.4 -> 误报 FOLLOW_TIMEOUT -> RED.
# ---------------------------------------------------------------------------
def test_t_c8_6_hold_reset_also_clears_follow_error_timer() -> None:
    """Hold 清除步长基线时同步复位跟随超时计时。"""
    ctx = make_supervision_controller(
        [ik_ok(q_delta(0.5)) for _ in range(4)],
        step_deg=2.0,
        lead_deg=30.0,
        follow_tolerance_deg=0.1,
        follow_timeout_s=0.4,
    )

    # 周期1 t=0: 接受 +0.5, last = BASE_Q + 0.5.
    o1 = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o1.sent is True

    # 周期2 t=0.3: 反馈滞后, 误差 0.5 > tol -> 计时起点 t=0.3.
    ctx.clock.advance(0.3)
    o2 = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o2.sent is True
    assert o2.error_code is None

    # 周期3 t=1.0: 过流 hold -> 清 _last_accepted_q; 必须同步清跟随计时器.
    ctx.clock.advance(0.7)
    o3 = ctx.controller.tick(
        feedback_with_holds(BASE_Q, joint_holds=shoulder_pan_hold(1.0)), TARGET
    )
    assert o3.error_code is ctx.mod.ControlError.OVERCURRENT_HOLD

    # 周期4 t=1.01: 恢复正常接受, 重建 last.
    ctx.clock.advance(0.01)
    o4 = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o4.sent is True
    assert o4.error_code is None

    # 周期5 t=1.02: 反馈仍滞后 -> 新跟随误差. 计时器已复位 -> 重新计时, 不超时.
    ctx.clock.advance(0.01)
    o5 = ctx.controller.tick(make_feedback(BASE_Q), TARGET)
    assert o5.error_code is None, "hold 清 _last_accepted_q 时未同步清 follow 计时器 -> 误报超时"
    assert o5.sent is True


if __name__ == "__main__":
    pytest.main([__file__])
