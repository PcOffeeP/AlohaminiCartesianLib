#!/usr/bin/env python

"""控制状态机命令层（``ControlStateMachine`` 与纯数据命令对象）的行为契约测试。

被测模块是本库的 ``alohamini_cartesian.control.commands``（控制层已从上游 lerobot 仓库
迁入本库）：覆盖 STANDBY 解锁门禁、READY 命令分派、阶段互锁与 BLOCKED、停机优先级、
FAULT 锁存与恢复、命令对象构造期自校验、反馈完整性异常等裁定语义。

纯逻辑用例：无 sleep、无时钟、无 I/O、无随机、无硬件。期望值全部是冻结字面量，不从被测
实现读回断言值。
"""

# Behavior B1.1 (实施包 P2「通用命令与状态抽象」, design §6 规则 1/2 与 D004):
# the STANDBY start gate of the control state machine that P4-B's controller.py
# will consume. The module `alohamini_cartesian.control.commands` belongs to this
# library (the control layer was migrated here out of the upstream lerobot
# repository); every case imports it inside a helper (same pattern as the
# cartesian protocol tests): a missing implementation fails each case with
# ImportError individually instead of erroring the whole module.
#
# Required behavior (acceptance):
# - the machine starts in STANDBY and generates no motion while there;
# - `resume()` is the ONLY unlock entry (A3) and moves STANDBY -> READY only
#   once BOTH gates are satisfied at call time:
#     1) settled proof: a window of N consecutive *valid new* feedback samples
#        that report settled=True (N injected at construction; A4 — a sample
#        count, never a clock);
#     2) current feedback validity: the most recent feedback is valid.
#   Otherwise the machine stays in STANDBY.
# - `stop()` targets STANDBY (A3).
# - `resume()` in READY is idempotent (A9).
#
# This binding deliberately discriminates the "true gate" from reasonable but
# WRONG implementations:
# - constructed directly in READY (caught by T-B1.1.1);
# - `resume()` that returns READY unconditionally / on a single settled sample
#   (caught by T-B1.1.2/T-B1.1.3/T-B1.1.5);
# - a hardcoded settle window N instead of the injected one (T-B1.1.4 vs
#   T-B1.1.5 with N=3 and N=2);
# - a window that is not required to be *consecutive* (T-B1.1.6);
# - a window that counts repeated / stale sample ids (T-B1.1.7);
# - a gate that ignores current feedback validity (T-B1.1.8/T-B1.1.9);
# - auto-transitioning to READY just because feedback looks settled (T-B1.1.10).
#
# Seam frozen for this binding (event seam adapted from the task suggestion and
# reported to the main agent):
#   module  : alohamini_cartesian.control.commands
#   class   : ControlStateMachine(settled_samples=N)   # A4: N injected, no clock
#   read    : .state -> ControlState ; .active_stage -> Stage | None ;
#             .last_reason -> str | None
#   drive   : .on_feedback(sample_id:int, settled:bool, valid:bool=True)
#             .resume() ; .stop()
#   enum    : ControlState.{STANDBY,READY,ACTIVE,SETTLING,BLOCKED,FAULT} (A2/A8)
#             Stage.{ARM,BASE_LIFT} (A8)
#   exc     : MotionBlocked (defined in commands.py, A2/A8)
# `.handle(command)` (single-category motion requests) is reserved for B1.2 and
# is intentionally NOT exercised by B1.1.
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# Expected values are frozen literals; nothing is read back from the
# implementation under test.

import ast
import contextlib
import dataclasses
import importlib
import inspect
import pkgutil
from types import SimpleNamespace
from typing import Any

import pytest

# Frozen vocabularies (design A2/A8). Set equality, so an extra or renamed
# member is a failure.
EXPECTED_STATE_MEMBERS = frozenset({"STANDBY", "READY", "ACTIVE", "SETTLING", "BLOCKED", "FAULT"})
EXPECTED_STAGE_MEMBERS = frozenset({"ARM", "BASE_LIFT"})


def load_commands() -> SimpleNamespace:
    """Import the seam lazily so a missing module fails per case, not at collection."""
    from alohamini_cartesian.control.commands import (
        ControlState,
        ControlStateMachine,
        MotionBlocked,
        Stage,
    )

    return SimpleNamespace(
        ControlState=ControlState,
        ControlStateMachine=ControlStateMachine,
        MotionBlocked=MotionBlocked,
        Stage=Stage,
    )


def make_machine(settled_samples: int) -> tuple[SimpleNamespace, Any]:
    """构造 commands 模块与给定停稳窗口的 ControlStateMachine。"""
    commands = load_commands()
    return commands, commands.ControlStateMachine(settled_samples=settled_samples)


def feed(machine: Any, sample_id: int, settled: bool, *, valid: bool = True) -> int:
    """喂入一条反馈样本并返回下一个 sample_id。"""
    machine.on_feedback(sample_id, settled, valid=valid)
    return sample_id + 1


def feed_count(machine: Any, start_id: int, count: int, settled: bool) -> int:
    """连续喂入 count 条同停稳标记的新反馈样本。"""
    next_id = start_id
    for _ in range(count):
        next_id = feed(machine, next_id, settled)
    return next_id


# ---------------------------------------------------------------------------
# T-B1.1.0 — frozen vocabulary and construction.
# ---------------------------------------------------------------------------
def test_frozen_state_and_stage_vocabulary_and_motion_blocked_exception() -> None:
    """T-B1.1.0：状态/阶段词表冻结，MotionBlocked 是异常且可构造机器。"""
    commands = load_commands()

    assert {member.name for member in commands.ControlState} == EXPECTED_STATE_MEMBERS
    assert {member.name for member in commands.Stage} == EXPECTED_STAGE_MEMBERS
    assert issubclass(commands.MotionBlocked, Exception)

    # Construction with an explicit settle window must be accepted (A4).
    machine = commands.ControlStateMachine(settled_samples=3)
    assert machine is not None


# ---------------------------------------------------------------------------
# T-B1.1.1 — a fresh machine is in STANDBY and is motion-idle.
# ---------------------------------------------------------------------------
def test_fresh_machine_starts_in_standby_and_is_motion_idle() -> None:
    """T-B1.1.1：新机器处于 STANDBY 且不产生运动。"""
    commands, machine = make_machine(settled_samples=3)

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None
    assert machine.last_reason is None or isinstance(machine.last_reason, str)


# ---------------------------------------------------------------------------
# T-B1.1.2 — resume() alone, with no feedback at all, keeps STANDBY.
# ---------------------------------------------------------------------------
def test_resume_without_any_feedback_stays_in_standby() -> None:
    """T-B1.1.2：没有任何反馈时 resume() 仍停留 STANDBY。"""
    commands, machine = make_machine(settled_samples=1)

    machine.resume()

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.1.3 — valid but NOT settled feedback never satisfies the gate.
# ---------------------------------------------------------------------------
def test_resume_with_valid_but_unsettled_feedback_stays_in_standby() -> None:
    """T-B1.1.3：有效但未停稳的反馈不满足解锁门禁。"""
    commands, machine = make_machine(settled_samples=3)

    # Plenty of valid samples, but none reports that the robot has settled.
    feed_count(machine, 1, 10, settled=False)
    machine.resume()

    assert machine.state is commands.ControlState.STANDBY


# ---------------------------------------------------------------------------
# T-B1.1.4 — the settle window is exactly N consecutive settled samples.
# ---------------------------------------------------------------------------
def test_settle_window_boundary_n_minus_one_vs_n() -> None:
    """T-B1.1.4：停稳窗口边界 N-1 不解锁、第 N 条解锁。"""
    commands, machine = make_machine(settled_samples=3)

    feed_count(machine, 1, 2, settled=True)  # N - 1
    machine.resume()
    assert machine.state is commands.ControlState.STANDBY

    feed(machine, 3, settled=True)  # N-th
    machine.resume()
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.1.5 — N comes from the constructor, not from code.
# ---------------------------------------------------------------------------
def test_settle_window_is_injected_by_constructor() -> None:
    """T-B1.1.5：停稳窗口 N 由构造参数注入，而不是硬编码。"""
    commands_two, machine_two = make_machine(settled_samples=2)
    feed(machine_two, 1, settled=True)
    machine_two.resume()
    assert machine_two.state is commands_two.ControlState.STANDBY  # needs 2
    feed(machine_two, 2, settled=True)
    machine_two.resume()
    assert machine_two.state is commands_two.ControlState.READY

    commands_four, machine_four = make_machine(settled_samples=4)
    feed_count(machine_four, 1, 3, settled=True)
    machine_four.resume()
    assert machine_four.state is commands_four.ControlState.STANDBY  # needs 4
    feed(machine_four, 4, settled=True)
    machine_four.resume()
    assert machine_four.state is commands_four.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.1.6 — the window must be consecutive; an unsettled sample resets it.
# ---------------------------------------------------------------------------
def test_non_settled_sample_resets_consecutive_window() -> None:
    """T-B1.1.6：窗口必须连续，未停稳样本会重置计数。"""
    commands, machine = make_machine(settled_samples=3)

    feed_count(machine, 1, 2, settled=True)
    feed(machine, 3, settled=False)  # reset in the middle
    feed_count(machine, 4, 2, settled=True)  # only 2 consecutive again
    machine.resume()
    assert machine.state is commands.ControlState.STANDBY

    feed(machine, 6, settled=True)  # now 3 consecutive
    machine.resume()
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.1.7 — only *new* samples count; repeated/stale sample ids do not.
# ---------------------------------------------------------------------------
def test_repeated_sample_id_does_not_advance_window() -> None:
    """T-B1.1.7：只有新样本推进窗口，重复或陈旧 id 不算。"""
    commands, machine = make_machine(settled_samples=2)

    feed(machine, 1, settled=True)
    feed(machine, 1, settled=True)  # same id: not a new sample
    machine.resume()
    assert machine.state is commands.ControlState.STANDBY

    feed(machine, 2, settled=True)  # new id completes the window
    machine.resume()
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.1.8 — invalid feedback alone never proves settled.
# ---------------------------------------------------------------------------
def test_invalid_feedback_never_proves_settled() -> None:
    """T-B1.1.8：无效反馈永不证明已停稳。"""
    commands, machine = make_machine(settled_samples=2)

    # Valid shape but explicitly invalid frames; settled=True must be ignored.
    machine.on_feedback(1, True, valid=False)
    machine.on_feedback(2, True, valid=False)
    machine.resume()

    assert machine.state is commands.ControlState.STANDBY


# ---------------------------------------------------------------------------
# T-B1.1.9 — a settle window followed by invalid feedback keeps STANDBY.
# ---------------------------------------------------------------------------
def test_window_then_invalid_feedback_keeps_standby() -> None:
    """T-B1.1.9：窗口之后出现无效反馈仍停留 STANDBY。"""
    commands, machine = make_machine(settled_samples=2)

    feed_count(machine, 1, 2, settled=True)  # window is full
    machine.on_feedback(3, True, valid=False)  # latest feedback lost/invalid
    machine.resume()

    assert machine.state is commands.ControlState.STANDBY


# ---------------------------------------------------------------------------
# T-B1.1.10 — resume() is the unique unlock entry; settled feedback alone is
# not enough (no auto-transition).
# ---------------------------------------------------------------------------
def test_settled_feedback_alone_does_not_auto_transition() -> None:
    """T-B1.1.10：仅凭停稳反馈不会自动迁移到 READY。"""
    commands, machine = make_machine(settled_samples=2)

    feed_count(machine, 1, 6, settled=True)  # far beyond the window
    assert machine.state is commands.ControlState.STANDBY

    machine.resume()
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.1.11 — READY stays motion-idle and resume() is idempotent (A9).
# ---------------------------------------------------------------------------
def test_ready_is_idle_and_resume_is_idempotent() -> None:
    """T-B1.1.11：READY 保持空闲，重复 resume() 幂等。"""
    commands, machine = make_machine(settled_samples=2)

    feed_count(machine, 1, 2, settled=True)
    machine.resume()
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None

    machine.resume()  # A9: idempotent in READY
    machine.resume()
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.1.12 — stop() targets STANDBY (A3).
# ---------------------------------------------------------------------------
def test_stop_targets_standby_from_standby_and_from_ready() -> None:
    """T-B1.1.12：从 STANDBY 与 READY 调用 stop() 都目标 STANDBY。"""
    commands, machine = make_machine(settled_samples=2)

    machine.stop()  # no-op safe in STANDBY
    assert machine.state is commands.ControlState.STANDBY

    feed_count(machine, 1, 2, settled=True)
    machine.resume()
    assert machine.state is commands.ControlState.READY

    machine.stop()
    assert machine.state is commands.ControlState.STANDBY


# ---------------------------------------------------------------------------
# T-B1.1.13 — refused resume() records a non-empty reason on the read seam.
# ---------------------------------------------------------------------------
def test_refused_resume_sets_non_empty_last_reason() -> None:
    """T-B1.1.13：被拒绝的 resume() 在读取面上留下非空原因。"""
    commands, machine = make_machine(settled_samples=2)

    machine.resume()  # refused: nothing settled
    assert machine.state is commands.ControlState.STANDBY
    assert isinstance(machine.last_reason, str)
    assert machine.last_reason


# ===========================================================================
# Behavior B1.2 (实施包 P2「通用命令与状态抽象」, design §6 规则 2/3/4 与 A7):
# READY + 单一类别运动请求 -> ACTIVE(stage=ARM 或 BASE_LIFT)。
#
# Protected acceptance:
# - from READY, a single-category motion request drives the machine to ACTIVE
#   with exactly the matching stage: ARM (arm + gripper; both LEFT/RIGHT per A7)
#   or BASE_LIFT (base velocity / lift);
# - a machine that is NOT READY (STANDBY) refuses every motion request and never
#   activates (direct negative for the READY gate; the full illegal-transition
#   matrix stays in B1.5);
# - the command abstraction is pure, device-independent data: frozen value
#   dataclasses with no bus/port/motor-id/device fields, and the module imports
#   no zmq / Client / Host transport (B6.5 device-independence spirit).
#
# Seam frozen for B1.2 (reported to the main agent for consolidation):
#   enum    : Side.{LEFT,RIGHT} ; GripperAction.{OPEN,CLOSE,SET}
#   data    : ArmCommand(side, dx, dy, dz)
#             GripperCommand(side, action, position=None)  # position 0..100 for SET
#             BaseCommand(x, y, turn, seconds)             # D014: explicit 0 velocity = base stop
#             LiftCommand(delta_mm)
#             StopCommand() ; ResumeCommand()
#   drive   : ControlStateMachine.handle(command)  # READY + motion -> ACTIVE(stage)
#
# This binding discriminates reasonable-but-WRONG implementations:
# - handle() that activates unconditionally, even from STANDBY (T-B1.2.9);
# - handle() that records the wrong stage or leaves active_stage None/stale
#   (T-B1.2.6/T-B1.2.7/T-B1.2.8);
# - handle() that silently drops a sub-type (gripper / lift / explicit-zero base)
#   instead of routing it to its category stage (T-B1.2.6/T-B1.2.7);
# - command classes that leak device addressing or mutate in flight
#   (T-B1.2.2/T-B1.2.3/T-B1.2.4/T-B1.2.5).
#
# Expected values are frozen literals; nothing is read back from the
# implementation under test beyond the public read seam. Pure logic only: no
# sleep, no clock, no I/O, no randomness, no hardware.
# ===========================================================================

EXPECTED_SIDE_MEMBERS = frozenset({"LEFT", "RIGHT"})
EXPECTED_GRIPPER_ACTION_MEMBERS = frozenset({"OPEN", "CLOSE", "SET"})

# Frozen field sets, so an added/renamed/removed seam field is a failure.
EXPECTED_COMMAND_FIELDS = {
    "ArmCommand": frozenset({"side", "dx", "dy", "dz"}),
    "GripperCommand": frozenset({"side", "action", "position"}),
    "BaseCommand": frozenset({"x", "y", "turn", "seconds"}),
    "LiftCommand": frozenset({"delta_mm"}),
    "StopCommand": frozenset(),
    "ResumeCommand": frozenset(),
}

# A command must never carry hardware addressing; these names (and any compound
# name containing one of these tokens) are forbidden on every command class.
FORBIDDEN_DEVICE_FIELD_NAMES = frozenset(
    {
        "bus",
        "port",
        "ports",
        "motor",
        "motor_id",
        "motor_ids",
        "motors",
        "device",
        "devices",
        "channel",
        "channels",
        "host",
        "endpoint",
        "socket",
        "ip",
        "addr",
        "address",
        "serial",
        "tty",
    }
)
FORBIDDEN_DEVICE_FIELD_TOKENS = (
    "bus",
    "port",
    "motor",
    "device",
    "channel",
    "socket",
    "zmq",
    "serial",
    "host",
    "endpoint",
    "addr",
)
FORBIDDEN_IMPORT_ROOTS = frozenset({"zmq", "socket", "serial"})
# 具体模块路径级的禁令（与 FORBIDDEN_IMPORT_ROOTS 互补）：客户端/主机模块，以及
# 本库把「设备运行时适配层」单独成包后的 ``alohamini_cartesian.adapter``。
# 条目只追加不删除：lerobot 时代的两条历史条目保留，便于同一守卫在旧仓库继续生效。
FORBIDDEN_IMPORT_PATHS = (
    "alohamini_client",
    "alohamini_host",
    "lerobot.robots.alohamini.alohamini",
    "alohamini_cartesian.adapter",
)

CONTROL_PACKAGE = "alohamini_cartesian.control"


def _imported_absolute_modules(source: str, module_package: str) -> set[str]:
    """解析源码里的全部 import，返回绝对模块名集合。

    本库库内导入全是相对导入，``ast.ImportFrom.module`` 只给出相对路径的尾巴
    （``from ..adapter.protocol import X`` 的 ``node.module`` 只有
    ``adapter.protocol``），因此必须按 ``node.level`` 用被扫描模块自身的包名
    ``module_package`` 回退出绝对模块名，否则路径级禁令对最自然的写法恒不命中：
    ``level=1`` 指当前包，``level=2`` 退一层，以此类推。``ast.Import`` 与
    ``level == 0`` 的 ``ast.ImportFrom`` 行为不变。
    """
    package_parts = module_package.split(".") if module_package else []
    imported: set[str] = set()

    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0:
                if node.module:
                    imported.add(node.module)
                continue

            up = node.level - 1
            if up > len(package_parts):
                # 越过包根：语法可解析但不可导入，原样保留供调用方查看。
                imported.add("." * node.level + (node.module or ""))
                continue

            base = package_parts[: len(package_parts) - up]
            if node.module:
                imported.add(".".join([*base, node.module]))
            else:
                # ``from .. import adapter``：names 本身就是子模块，逐名拼绝对名。
                imported.update(".".join([*base, alias.name]) for alias in node.names)

    return imported


def _transport_import_leaks(source: str, module_package: str) -> set[str]:
    """返回源码里命中 ``FORBIDDEN_IMPORT_PATHS`` 的绝对模块名。"""
    return {
        name
        for name in _imported_absolute_modules(source, module_package)
        if any(part in name for part in FORBIDDEN_IMPORT_PATHS)
    }


def _control_module_sources() -> dict[str, tuple[str, str]]:
    """返回 control 包内「除 api.py 之外」每个模块的 (源码, 包名)。

    ``api.py`` 是唯一允许接触适配层的模块（它接收并驱动 adapter），不纳入本守卫。
    包自身的 ``__init__.py`` 虽只做再导出，也不允许直接触达适配层，因此一并纳入
    （``pkgutil.iter_modules`` 不会返回包自身的 ``__init__.py``）。
    """
    package = importlib.import_module(CONTROL_PACKAGE)
    sources: dict[str, tuple[str, str]] = {}

    for info in pkgutil.iter_modules(package.__path__, prefix=f"{CONTROL_PACKAGE}."):
        if info.name == f"{CONTROL_PACKAGE}.api":
            continue
        module = importlib.import_module(info.name)
        sources[info.name] = (
            inspect.getsource(module),
            module.__package__ or CONTROL_PACKAGE,
        )

    sources[CONTROL_PACKAGE] = (
        inspect.getsource(package),
        package.__package__ or CONTROL_PACKAGE,
    )

    return sources


ARM_COMMAND_KEYS = ("arm_left", "arm_right", "gripper_open_left", "gripper_set_right")
BASE_LIFT_COMMAND_KEYS = ("base_drive", "lift_up", "base_explicit_zero")
ALL_MOTION_COMMAND_KEYS = ARM_COMMAND_KEYS + BASE_LIFT_COMMAND_KEYS

MOTION_KEY_TO_STAGE = {
    "arm_left": "ARM",
    "arm_right": "ARM",
    "gripper_open_left": "ARM",
    "gripper_set_right": "ARM",
    "base_drive": "BASE_LIFT",
    "lift_up": "BASE_LIFT",
    "base_explicit_zero": "BASE_LIFT",
}


def load_command_types() -> SimpleNamespace:
    """Import the B1.2 command vocabulary lazily (missing class -> per-case failure)."""
    from alohamini_cartesian.control.commands import (
        ArmCommand,
        BaseCommand,
        GripperAction,
        GripperCommand,
        LiftCommand,
        ResumeCommand,
        Side,
        StopCommand,
    )

    return SimpleNamespace(
        ArmCommand=ArmCommand,
        BaseCommand=BaseCommand,
        GripperAction=GripperAction,
        GripperCommand=GripperCommand,
        LiftCommand=LiftCommand,
        ResumeCommand=ResumeCommand,
        Side=Side,
        StopCommand=StopCommand,
    )


def ready_machine(settled_samples: int = 2) -> tuple[SimpleNamespace, Any]:
    """Drive the shared B1.1 gate to READY using the validated resume() path."""
    commands, machine = make_machine(settled_samples=settled_samples)
    feed_count(machine, 1, settled_samples, settled=True)
    machine.resume()
    assert machine.state is commands.ControlState.READY
    return commands, machine


def build_motion_command(command_types: SimpleNamespace, key: str) -> Any:
    """Build one representative motion command for a category key.

    Every field is passed explicitly so the case does not depend on dataclass
    defaults (those are frozen separately by T-B1.2.3).
    """
    ct = command_types
    builders = {
        "arm_left": lambda: ct.ArmCommand(side=ct.Side.LEFT, dx=0.01, dy=0.0, dz=0.0),
        "arm_right": lambda: ct.ArmCommand(side=ct.Side.RIGHT, dx=0.0, dy=-0.02, dz=0.0),
        "gripper_open_left": lambda: ct.GripperCommand(
            side=ct.Side.LEFT, action=ct.GripperAction.OPEN, position=None
        ),
        "gripper_set_right": lambda: ct.GripperCommand(
            side=ct.Side.RIGHT, action=ct.GripperAction.SET, position=50
        ),
        "base_drive": lambda: ct.BaseCommand(x=0.1, y=0.0, turn=0.0, seconds=1.0),
        "base_explicit_zero": lambda: ct.BaseCommand(x=0.0, y=0.0, turn=0.0, seconds=0.0),
        "lift_up": lambda: ct.LiftCommand(delta_mm=20.0),
    }
    assert key in builders, f"unknown motion command key: {key}"
    return builders[key]()


# ---------------------------------------------------------------------------
# T-B1.2.1 — frozen Side / GripperAction vocabulary.
# ---------------------------------------------------------------------------
def test_b12_command_side_and_gripper_action_vocabulary() -> None:
    """T-B1.2.1：Side 与 GripperAction 词表冻结。"""
    cmd = load_command_types()

    assert {member.name for member in cmd.Side} == EXPECTED_SIDE_MEMBERS
    assert {member.name for member in cmd.GripperAction} == EXPECTED_GRIPPER_ACTION_MEMBERS


# ---------------------------------------------------------------------------
# T-B1.2.2 — commands are immutable value objects (pure data, no aliasing).
# ---------------------------------------------------------------------------
def test_b12_command_classes_are_frozen_value_dataclasses() -> None:
    """T-B1.2.2：命令对象是不可变值对象且无别名共享。"""
    cmd = load_command_types()

    arm = cmd.ArmCommand(side=cmd.Side.LEFT, dx=0.01, dy=0.0, dz=0.0)
    same = cmd.ArmCommand(side=cmd.Side.LEFT, dx=0.01, dy=0.0, dz=0.0)
    other = cmd.ArmCommand(side=cmd.Side.RIGHT, dx=0.01, dy=0.0, dz=0.0)

    assert arm == same  # value semantics
    assert arm != other
    assert hash(arm) == hash(same)  # usable as a frozen key

    # Immutability: a command cannot be mutated mid-flight.
    with pytest.raises(dataclasses.FrozenInstanceError):
        arm.dx = 0.5


# ---------------------------------------------------------------------------
# T-B1.2.3 — frozen field sets; a renamed/added field breaks callers.
# ---------------------------------------------------------------------------
def test_b12_command_field_sets_are_frozen() -> None:
    """T-B1.2.3：各命令的字段集合与默认值冻结。"""
    cmd = load_command_types()

    for name, expected in EXPECTED_COMMAND_FIELDS.items():
        cls = getattr(cmd, name)
        assert dataclasses.is_dataclass(cls), name
        actual = frozenset(field.name for field in dataclasses.fields(cls))
        assert actual == expected, f"{name}: {sorted(actual)}"


# ---------------------------------------------------------------------------
# T-B1.2.4 — no command leaks hardware addressing (bus/port/motor-id/device).
# ---------------------------------------------------------------------------
def test_b12_commands_carry_no_device_addressing_fields() -> None:
    """T-B1.2.4：命令不携带任何硬件寻址字段。"""
    cmd = load_command_types()

    for name in EXPECTED_COMMAND_FIELDS:
        cls = getattr(cmd, name)
        field_names = [field.name for field in dataclasses.fields(cls)]
        lowered = {field_name.lower() for field_name in field_names}

        leaked = lowered & FORBIDDEN_DEVICE_FIELD_NAMES
        assert not leaked, f"{name} exposes device fields {sorted(leaked)}"

        for field_name in lowered:
            for token in FORBIDDEN_DEVICE_FIELD_TOKENS:
                assert token not in field_name, f"{name}.{field_name} mentions {token!r}"


# ---------------------------------------------------------------------------
# T-B1.2.5 — the commands module itself imports no device transport.
# ---------------------------------------------------------------------------
def test_b12_commands_module_is_device_independent() -> None:
    """T-B1.2.5：commands 模块不导入任何设备传输层或适配层。"""
    # Requires the B1.2 vocabulary to exist (this is part of the frozen seam).
    load_command_types()

    module = importlib.import_module(f"{CONTROL_PACKAGE}.commands")
    module_package = module.__package__ or CONTROL_PACKAGE
    source = inspect.getsource(module)

    imported = _imported_absolute_modules(source, module_package)

    roots = {name.split(".")[0] for name in imported}
    assert roots.isdisjoint(FORBIDDEN_IMPORT_ROOTS), sorted(imported)

    leaked = _transport_import_leaks(source, module_package)
    assert not leaked, sorted(leaked)


def test_b12_no_control_module_outside_api_imports_transport_or_adapter() -> None:
    """T-B1.2.5（扩展）：control 包内除 api.py 外，每个模块都不泄漏传输/适配层。"""
    load_command_types()

    sources = _control_module_sources()
    # 自检：扫到的集合必须含有本次迁移新增的 commands.py 与包自身，且真的排除了 api.py。
    assert f"{CONTROL_PACKAGE}.commands" in sources
    assert CONTROL_PACKAGE in sources
    assert f"{CONTROL_PACKAGE}.api" not in sources

    offenders: dict[str, list[str]] = {}
    for module_name in sorted(sources):
        source, module_package = sources[module_name]
        imported = _imported_absolute_modules(source, module_package)
        roots = {name.split(".")[0] for name in imported}
        assert roots.isdisjoint(FORBIDDEN_IMPORT_ROOTS), (module_name, sorted(imported))

        leaked = _transport_import_leaks(source, module_package)
        if leaked:
            offenders[module_name] = sorted(leaked)

    assert offenders == {}


def test_b12_transport_import_guard_catches_relative_adapter_import() -> None:
    """T-B1.2.5（正向控制）：相对导入到 adapter 的源码必须被守卫命中。

    库内导入全是相对导入，只看 ``node.module`` 的实现对 ``from ..adapter.protocol
    import X`` 恒不命中；本条用合成源码钉住这个退化，防止守卫再次变成恒真。
    """
    source = (
        "from __future__ import annotations\n"
        "\n"
        "from ..adapter.protocol import RobotTransport\n"
        "\n"
        "VALUE = RobotTransport\n"
    )

    leaked = _transport_import_leaks(source, CONTROL_PACKAGE)

    assert leaked == {"alohamini_cartesian.adapter.protocol"}


def test_b12_transport_import_guard_ignores_legitimate_relative_import() -> None:
    """T-B1.2.5（反向控制）：合法的库内相对导入不得被守卫误报。"""
    source = "from __future__ import annotations\n\nfrom ..commands import Side\n\nVALUE = Side\n"

    imported = _imported_absolute_modules(source, CONTROL_PACKAGE)

    # ``__future__`` 是 level 0 的绝对导入，按原行为原样保留；关键是被解析出的
    # 相对导入必须还原成 ``alohamini_cartesian.commands`` 而不是 ``commands``。
    assert imported == {"__future__", "alohamini_cartesian.commands"}
    assert _transport_import_leaks(source, CONTROL_PACKAGE) == set()


# ---------------------------------------------------------------------------
# T-B1.2.6 — an ARM-category request from READY enters ACTIVE(ARM).
# Covers both sides (A7), arm deltas and gripper commands (design rule 3).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ARM_COMMAND_KEYS)
def test_b12_arm_category_request_enters_active_arm_stage(key: str) -> None:
    """T-B1.2.6：READY 下的 ARM 类请求进入 ACTIVE(ARM)。"""
    cmd_types = load_command_types()
    commands, machine = ready_machine()

    machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM
    assert machine.active_stage is not commands.Stage.BASE_LIFT


# ---------------------------------------------------------------------------
# T-B1.2.7 — a BASE_LIFT-category request from READY enters ACTIVE(BASE_LIFT).
# Covers base drive, lift move and the D014 explicit-zero base command.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", BASE_LIFT_COMMAND_KEYS)
def test_b12_base_lift_category_request_enters_active_base_lift_stage(key: str) -> None:
    """T-B1.2.7：READY 下的 BASE_LIFT 类请求进入 ACTIVE(BASE_LIFT)。"""
    cmd_types = load_command_types()
    commands, machine = ready_machine()

    machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.BASE_LIFT
    assert machine.active_stage is not commands.Stage.ARM


# ---------------------------------------------------------------------------
# T-B1.2.8 — stage uniqueness: one request selects exactly one stage and the two
# categories never collapse onto the same stage.
# ---------------------------------------------------------------------------
def test_b12_active_stage_is_unique_and_categories_are_disjoint() -> None:
    """T-B1.2.8：一次请求只选一个阶段，两类互斥。"""
    cmd_types = load_command_types()
    commands, _ = ready_machine()

    seen: dict[str, object] = {}
    for key in ALL_MOTION_COMMAND_KEYS:
        _, machine = ready_machine()
        machine.handle(build_motion_command(cmd_types, key))
        assert machine.state is commands.ControlState.ACTIVE, key
        assert isinstance(machine.active_stage, commands.Stage), key
        assert machine.active_stage.name == MOTION_KEY_TO_STAGE[key], key
        seen[key] = machine.active_stage

    arm_stages = {seen[key] for key in ARM_COMMAND_KEYS}
    base_stages = {seen[key] for key in BASE_LIFT_COMMAND_KEYS}
    assert arm_stages == {commands.Stage.ARM}
    assert base_stages == {commands.Stage.BASE_LIFT}
    assert arm_stages.isdisjoint(base_stages)


# ---------------------------------------------------------------------------
# T-B1.2.9 — a motion request outside READY is refused and never activates.
# The refusal MECHANISM is deferred to B1.5, so either MotionBlocked or a silent
# no-op is accepted; the safety invariant (still STANDBY, no active stage) is
# what this case binds.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ALL_MOTION_COMMAND_KEYS)
def test_b12_motion_request_is_refused_in_standby(key: str) -> None:
    """T-B1.2.9：STANDBY 下的运动请求被拒绝且不激活任何阶段。"""
    cmd_types = load_command_types()
    commands, machine = make_machine(settled_samples=2)
    assert machine.state is commands.ControlState.STANDBY

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.2.10 — D014: base stop is expressible as an explicit zero velocity vector
# on the same pure-data command (no separate device call, no watchdog reliance).
# ---------------------------------------------------------------------------
def test_b12_d014_base_stop_is_explicit_zero_velocity() -> None:
    """T-B1.2.10：D014 的底座停止表达为显式零速度命令。"""
    cmd_types = load_command_types()

    stopped = cmd_types.BaseCommand(x=0.0, y=0.0, turn=0.0, seconds=0.0)
    driving = cmd_types.BaseCommand(x=0.1, y=0.0, turn=0.0, seconds=1.0)

    assert (stopped.x, stopped.y, stopped.turn) == (0.0, 0.0, 0.0)
    assert type(stopped) is type(driving)  # same BASE_LIFT-category command type


# ===========================================================================
# Behavior B1.3 + B1.4 (实施包 P2「通用命令与状态抽象」, design §6 规则 3/5 与 D004):
#
# B1.3 — completing / releasing the active action:
#   an ACTIVE(ARM|BASE_LIFT) action that finishes or whose key is released moves
#   the machine to SETTLING (design §6 rule 3: "当前动作完成 / 松键").  SETTLING
#   no longer accepts any new motion advance (MotionBlocked) and does NOT jump to
#   READY on its own.
#
# B1.4 — confirming standstill while SETTLING:
#   exactly N *consecutive valid new* feedback samples with settled=True move
#   SETTLING -> READY, reusing the B1.1/A4 window semantics unchanged: N is
#   injected at construction (never a clock), freshness is by never-before-seen
#   sample_id, an invalid or non-settled new sample resets the window, and the
#   transition happens automatically when the N-th fresh sample lands.  N-1
#   samples do not transition; the N-th does.
#
# Completion seam frozen here (reported to the main agent for consolidation):
#   data    : ReleaseCommand()          # frozen pure-data value object with an
#                                       # empty field set; expresses 动作完成 /
#                                       # 松键 (design §6 rule 3).  Distinct from
#                                       # stop(): an explicit stop targets
#                                       # STANDBY (D004), a release targets
#                                       # SETTLING.
#   drive   : ControlStateMachine.handle(ReleaseCommand())
#               ACTIVE -> SETTLING ; refused anywhere else (never activates).
#
# This binding deliberately discriminates reasonable-but-WRONG implementations:
# - a release that jumps ACTIVE -> READY, silently skipping SETTLING
#   (T-B1.3.1/T-B1.3.2);
# - a SETTLING state that still accepts a new motion advance (T-B1.3.4);
# - SETTLING that releases immediately / on the first sample, without waiting
#   for the window (T-B1.4.1, T-B1.4.6);
# - a hardcoded settle window instead of the injected N (T-B1.4.2);
# - a window that counts stale / repeated sample ids, or that carries the
#   pre-ACTIVE settled count across the ACTIVE/SETTLING boundary
#   (T-B1.4.4, T-B1.4.6, T-B1.4.7);
# - a release command that persists the stage / leaks aliasing
#   (T-B1.3.0).
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# Expected values are frozen literals; nothing is read back from the
# implementation under test beyond the public read seam.
# ===========================================================================

# Sample ids reserved for the B1.3/B1.4 cases.  Each helper starts a fresh
# machine, so these only need to be *distinct and greater* than the ids the
# B1.1 start gate already consumed (1..settled_samples); using a fixed high base
# makes "stale id" tests explicit rather than incidental.
FRESH_SAMPLE_BASE = 10_000


def load_completion_types() -> SimpleNamespace:
    """Import the B1.3 completion seam lazily (missing class -> per-case failure)."""
    from alohamini_cartesian.control.commands import ReleaseCommand

    return SimpleNamespace(ReleaseCommand=ReleaseCommand)


def active_machine(stage_key: str, settled_samples: int = 3) -> tuple[SimpleNamespace, Any]:
    """Drive the shared B1.1/B1.2 path to ACTIVE(stage) via one motion request."""
    cmd_types = load_command_types()
    commands, machine = ready_machine(settled_samples=settled_samples)
    machine.handle(build_motion_command(cmd_types, stage_key))
    assert machine.state is commands.ControlState.ACTIVE, stage_key
    assert machine.active_stage is not None, stage_key
    return commands, machine


def released_machine(stage_key: str, settled_samples: int = 3) -> tuple[SimpleNamespace, Any]:
    """Drive ACTIVE(stage) and dispatch the frozen completion command."""
    completion = load_completion_types()
    commands, machine = active_machine(stage_key, settled_samples=settled_samples)
    machine.handle(completion.ReleaseCommand())
    return commands, machine


# ---------------------------------------------------------------------------
# T-B1.3.0 — the completion command is a frozen, empty, pure-data value object.
# ---------------------------------------------------------------------------


def test_b13_release_command_is_frozen_empty_value_object() -> None:
    """T-B1.3.0：释放命令是冻结的空纯数据值对象。"""
    completion = load_completion_types()
    release_cls = completion.ReleaseCommand

    assert dataclasses.is_dataclass(release_cls)
    fields = frozenset(field.name for field in dataclasses.fields(release_cls))
    assert fields == frozenset()  # carries no payload and no device addressing

    first = release_cls()
    second = release_cls()
    assert first == second  # value semantics
    assert hash(first) == hash(second)

    with pytest.raises(dataclasses.FrozenInstanceError):
        first.extra = 1


# ---------------------------------------------------------------------------
# T-B1.3.1 / T-B1.3.2 — releasing / completing an ACTIVE action enters SETTLING
# for both stage categories (ARM and BASE_LIFT), never directly READY.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ["arm_left", "base_drive"])
def test_b13_active_action_release_enters_settling(key: str) -> None:
    """T-B1.3.1/2：释放或完成 ACTIVE 动作进入 SETTLING。"""
    commands, machine = released_machine(key)

    assert machine.state is commands.ControlState.SETTLING
    assert machine.state is not commands.ControlState.READY
    assert machine.state is not commands.ControlState.ACTIVE


# ---------------------------------------------------------------------------
# T-B1.3.3 — a release outside ACTIVE is refused and never activates anything;
# the refusal MECHANISM is not bound here (B1.5), only the safety invariant.
# ---------------------------------------------------------------------------


def test_b13_release_outside_active_is_refused() -> None:
    """T-B1.3.3：ACTIVE 之外的释放被拒绝且不激活任何阶段。"""
    completion = load_completion_types()
    commands, machine = make_machine(settled_samples=2)

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(completion.ReleaseCommand())
    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None

    feed_count(machine, 1, 2, settled=True)
    machine.resume()
    assert machine.state is commands.ControlState.READY

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(completion.ReleaseCommand())
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.3.4 — SETTLING is a hardened state: it refuses every new motion advance
# (MotionBlocked) and never activates a stage.  This is the direct negative for
# "SETTLING 不再接受新推进".
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ALL_MOTION_COMMAND_KEYS)
def test_b13_settling_refuses_new_motion_advance(key: str) -> None:
    """T-B1.3.4：SETTLING 拒绝任何新的运动推进。"""
    cmd_types = load_command_types()
    commands, machine = released_machine("arm_left")
    assert machine.state is commands.ControlState.SETTLING

    with pytest.raises(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.SETTLING


# ---------------------------------------------------------------------------
# T-B1.3.5 — D004 stop priority: stop() from ACTIVE and from SETTLING goes to
# STANDBY (never to READY), clearing any active stage.
# ---------------------------------------------------------------------------


def test_b13_stop_from_active_and_settling_targets_standby() -> None:
    """T-B1.3.5：D004 停机优先，ACTIVE 与 SETTLING 都目标 STANDBY。"""
    commands, machine = active_machine("arm_left")
    assert machine.state is commands.ControlState.ACTIVE
    machine.stop()
    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None

    commands, machine = released_machine("base_drive")
    assert machine.state is commands.ControlState.SETTLING
    machine.stop()
    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.4.1 — the SETTLING -> READY window boundary: N-1 fresh settled samples
# leave the machine SETTLING, the N-th transitions to READY (idle, no stage).
# ---------------------------------------------------------------------------


def test_b14_settling_window_boundary_n_minus_one_vs_n() -> None:
    """T-B1.4.1：SETTLING 到 READY 的窗口边界 N-1 与 N。"""
    commands, machine = released_machine("arm_left", settled_samples=3)
    assert machine.state is commands.ControlState.SETTLING

    feed_count(machine, FRESH_SAMPLE_BASE, 2, settled=True)  # N - 1
    assert machine.state is commands.ControlState.SETTLING

    feed(machine, FRESH_SAMPLE_BASE + 2, settled=True)  # N-th
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.4.2 — the SETTLING window is the injected N, not a hardcoded constant.
# ---------------------------------------------------------------------------


def test_b14_settling_window_is_injected_by_constructor() -> None:
    """T-B1.4.2：SETTLING 窗口取注入的 N，不是硬编码常量。"""
    commands_two, machine_two = released_machine("arm_left", settled_samples=2)
    feed(machine_two, FRESH_SAMPLE_BASE, settled=True)
    assert machine_two.state is commands_two.ControlState.SETTLING  # needs 2
    feed(machine_two, FRESH_SAMPLE_BASE + 1, settled=True)
    assert machine_two.state is commands_two.ControlState.READY

    commands_four, machine_four = released_machine("base_drive", settled_samples=4)
    feed_count(machine_four, FRESH_SAMPLE_BASE, 3, settled=True)
    assert machine_four.state is commands_four.ControlState.SETTLING  # needs 4
    feed(machine_four, FRESH_SAMPLE_BASE + 3, settled=True)
    assert machine_four.state is commands_four.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.4.3 — the window must be consecutive; a valid but non-settled new sample
# resets it during SETTLING (same semantics as the B1.1 start gate).
# ---------------------------------------------------------------------------


def test_b14_non_settled_sample_resets_settling_window() -> None:
    """T-B1.4.3：有效但未停稳的新样本重置 SETTLING 窗口。"""
    commands, machine = released_machine("arm_left", settled_samples=3)

    feed_count(machine, FRESH_SAMPLE_BASE, 2, settled=True)
    feed(machine, FRESH_SAMPLE_BASE + 2, settled=False)  # reset in the middle
    feed_count(machine, FRESH_SAMPLE_BASE + 3, 2, settled=True)  # only 2 again
    assert machine.state is commands.ControlState.SETTLING

    feed(machine, FRESH_SAMPLE_BASE + 5, settled=True)  # now 3 consecutive
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.4.4 — only *new* sample ids prove standstill while SETTLING: re-feeding
# ids consumed before ACTIVE (stale) and repeating one fresh id both fail to
# advance the window (B8 spirit applied to the SETTLING path).
# ---------------------------------------------------------------------------


def test_b14_stale_and_repeated_sample_ids_do_not_advance_settling() -> None:
    """T-B1.4.4：陈旧或重复的样本 id 不推进 SETTLING 窗口。"""
    commands, machine = released_machine("arm_left", settled_samples=2)
    assert machine.state is commands.ControlState.SETTLING

    # Ids 1..2 already proved the STANDBY gate; they are stale for SETTLING.
    feed(machine, 1, settled=True)
    feed(machine, 2, settled=True)
    assert machine.state is commands.ControlState.SETTLING

    # Repeating one fresh id is not two new samples.
    feed(machine, FRESH_SAMPLE_BASE, settled=True)
    feed(machine, FRESH_SAMPLE_BASE, settled=True)
    assert machine.state is commands.ControlState.SETTLING

    feed(machine, FRESH_SAMPLE_BASE + 1, settled=True)  # second new id -> N
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.4.5 — invalid feedback never proves standstill: invalid samples are
# ignored (and reset the window), so N valid fresh settled samples are needed.
# ---------------------------------------------------------------------------


def test_b14_invalid_feedback_never_confirms_settling() -> None:
    """T-B1.4.5：无效反馈永不确认停稳。"""
    commands, machine = released_machine("arm_left", settled_samples=2)

    machine.on_feedback(FRESH_SAMPLE_BASE, True, valid=False)
    machine.on_feedback(FRESH_SAMPLE_BASE + 1, True, valid=False)
    assert machine.state is commands.ControlState.SETTLING

    feed(machine, FRESH_SAMPLE_BASE + 2, settled=True)
    assert machine.state is commands.ControlState.SETTLING  # 1 of 2 valid
    feed(machine, FRESH_SAMPLE_BASE + 3, settled=True)
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.4.6 — entering SETTLING resets the window: settled samples observed while
# ACTIVE must NOT be reused as standstill proof (no ACTIVE -> READY shortcut
# through a stale counter).
# ---------------------------------------------------------------------------


def test_b14_settling_window_does_not_carry_over_from_active() -> None:
    """T-B1.4.6：进入 SETTLING 时窗口清零，不沿用 ACTIVE 期间的样本。"""
    completion = load_completion_types()
    commands, machine = active_machine("arm_left", settled_samples=2)

    # Robot looks settled during ACTIVE (e.g. between two streamed commands).
    feed_count(machine, FRESH_SAMPLE_BASE, 5, settled=True)
    machine.handle(completion.ReleaseCommand())
    assert machine.state is commands.ControlState.SETTLING  # not already READY

    feed(machine, FRESH_SAMPLE_BASE + 5, settled=True)
    assert machine.state is commands.ControlState.SETTLING  # 1 of 2 fresh
    feed(machine, FRESH_SAMPLE_BASE + 6, settled=True)
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B1.4.7 — SETTLING -> READY -> ACTIVE round trip: READY is idle and accepts a
# new single-category request, and a second SETTLING cycle requires a fresh
# window (the counter is reset per cycle, not accumulated).
# ---------------------------------------------------------------------------


def test_b14_settling_returns_to_ready_and_ready_accepts_new_motion() -> None:
    """T-B1.4.7：SETTLING 回到 READY 后 READY 接受新运动请求。"""
    cmd_types = load_command_types()
    completion = load_completion_types()
    commands, machine = released_machine("arm_left", settled_samples=2)

    feed_count(machine, FRESH_SAMPLE_BASE, 2, settled=True)
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None

    machine.handle(build_motion_command(cmd_types, "base_drive"))
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.BASE_LIFT

    machine.handle(completion.ReleaseCommand())
    assert machine.state is commands.ControlState.SETTLING

    feed(machine, FRESH_SAMPLE_BASE + 100, settled=True)
    assert machine.state is commands.ControlState.SETTLING  # 1 of 2 fresh again
    feed(machine, FRESH_SAMPLE_BASE + 101, settled=True)
    assert machine.state is commands.ControlState.READY


# ===========================================================================
# Behavior B1.5 (实施包 P2「通用命令与状态抽象」, design §6「不得 any-to-any」):
# 非法转移拒绝矩阵——任何未在合法转移图中的 (状态 × 事件) 组合都必须被拒绝
# 且机器状态保持不变。
#
# P2 B3/B9.1 裁定（本文件唯一一次既有用例修订，见交付说明）:
#   MotionBlocked 只用于**运动类命令**（motion + release）；
#   `handle(ResumeCommand())` 统一为非抛出语义（等价 `resume()` 方法）：
#   STANDBY 走解锁判定、READY 幂等、ACTIVE/SETTLING/BLOCKED 非抛出拒绝。
#
# design §6 的合法转移（本阶段只认这些边）：
#   STANDBY + resume（停稳窗口满 + 最新反馈有效） -> READY
#   READY   + 单一类别运动请求                    -> ACTIVE(stage)
#   ACTIVE  + ReleaseCommand                      -> SETTLING
#   SETTLING+ 连续有效新反馈证实停稳               -> READY（由 on_feedback 驱动）
#   任意态  + stop()                              -> STANDBY
#
# Protected acceptance:
# - 不属于上述边的 (状态 × 事件) 组合必须被拒绝，且 state/active_stage 在拒绝后
#   与拒绝前完全一致（不得 any-to-any 放行）；运动/放行类以 MotionBlocked 拒绝，
#   ResumeCommand 为非抛出拒绝（见 B15_NON_THROWING_RESUME_CASES）；
# - 重点覆盖：READY+ReleaseCommand、STANDBY+ReleaseCommand、STANDBY+motion、
#   SETTLING+motion、ACTIVE+ResumeCommand（非抛出）、STANDBY+ResumeCommand（非抛出）；
# - resume() 方法只在 STANDBY 是解锁入口（READY 幂等），从 ACTIVE/SETTLING 调用
#   不得把机器伪装成 READY（尤其不得留下 READY + 非空 active_stage 的非法组合）。
#
# 本阶段边界：
# - B2 跨阶段并发 -> BLOCKED、B2.3 同阶段并发允许、B5 FAULT 均不在本阶段；
#   本阶段只保证这些路径不产生非法状态（BLOCKED/FAULT 不由 B1.5 判定）；
# - `handle(ResumeCommand())` 的语义已由 P2 B3/B9.1 裁定为非抛出（等价 resume()）；
# - `handle(StopCommand())` 的语义已由 B3 裁定为等价 stop()（任意态 -> STANDBY，不抛出）。
#
# This binding deliberately discriminates reasonable-but-WRONG implementations:
# - `resume()` that only checks the settle window and ignores the current state,
#   so a plain ACTIVE + resume() silently jumps to READY while keeping the stale
#   ARM stage (T-B1.5.14 — the only genuinely missing behavior in this stage);
# - `handle()` that passes an unknown/out-of-graph command straight through, or
#   that changes state before/while refusing (T-B1.5.1..T-B1.5.11);
# - `handle()` that any-to-any forwards a release command from a state it is not
#   valid in, or that raises MotionBlocked for a ResumeCommand
#   (T-B1.5.1..T-B1.5.11, B15_NON_THROWING_RESUME_CASES).
#
# Expected values are frozen literals; nothing is read back from the
# implementation under test beyond the public read seam. Pure logic only: no
# sleep, no clock, no I/O, no randomness, no hardware.
# ===========================================================================


def load_b15_types() -> SimpleNamespace:
    """Import the B1.5 event vocabulary lazily (missing class -> per-case failure)."""
    from alohamini_cartesian.control.commands import (
        ReleaseCommand,
        ResumeCommand,
    )

    return SimpleNamespace(ReleaseCommand=ReleaseCommand, ResumeCommand=ResumeCommand)


def machine_in_state(state_name: str, settled_samples: int = 2) -> tuple[SimpleNamespace, Any]:
    """Build a machine in one of the four B1.5 states using only validated edges.

    BLOCKED / FAULT are intentionally unreachable here: they belong to B2/B5 and
    this stage must not construct them.
    """
    if state_name == "STANDBY":
        commands, machine = make_machine(settled_samples=settled_samples)
    elif state_name == "READY":
        commands, machine = ready_machine(settled_samples=settled_samples)
    elif state_name == "ACTIVE":
        commands, machine = active_machine("arm_left", settled_samples=settled_samples)
    elif state_name == "SETTLING":
        commands, machine = released_machine("arm_left", settled_samples=settled_samples)
    else:
        raise AssertionError(f"unsupported B1.5 state: {state_name}")

    assert machine.state.name == state_name, (state_name, machine.state)
    return commands, machine


def dispatch_b15_event(machine: Any, event_key: str) -> None:
    """Dispatch one matrix event; every event is expected to be REFUSED here."""
    cmd_types = load_command_types()
    b15 = load_b15_types()
    if event_key == "release":
        machine.handle(b15.ReleaseCommand())
    elif event_key == "resume_command":
        machine.handle(cmd_types.ResumeCommand())
    elif event_key.startswith("motion:"):
        machine.handle(build_motion_command(cmd_types, event_key.split(":", 1)[1]))
    else:
        raise AssertionError(f"unknown B1.5 event key: {event_key}")


# (state, event) pairs that are NOT edges of the design §6 graph and therefore
# must be refused with MotionBlocked while leaving state/stage untouched.
# Motion-class and release-class commands are the MotionBlocked family.
# `handle(ResumeCommand())` is deliberately NOT here: per the P2 B3/B9.1 ruling it
# is unified with `resume()` and is non-throwing from every state (see
# B15_NON_THROWING_RESUME_CASES below).
B15_ILLEGAL_CASES = (
    *[("STANDBY", f"motion:{key}") for key in ALL_MOTION_COMMAND_KEYS],
    ("STANDBY", "release"),
    ("READY", "release"),
    *[("SETTLING", f"motion:{key}") for key in ALL_MOTION_COMMAND_KEYS],
    ("SETTLING", "release"),
)


# ---------------------------------------------------------------------------
# T-B1.5.1 .. T-B1.5.11 — every out-of-graph motion/release (state, event) pair
# raises MotionBlocked and leaves both state and active_stage exactly as they
# were.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    "state_name,event_key",
    B15_ILLEGAL_CASES,
    ids=[f"{state}-{event}" for state, event in B15_ILLEGAL_CASES],
)
def test_b15_out_of_graph_combination_is_refused_and_state_invariant(
    state_name: str, event_key: str
) -> None:
    """T-B1.5：状态图之外的组合被拒绝且状态与阶段不变。"""
    commands, machine = machine_in_state(state_name)
    state_before = machine.state
    stage_before = machine.active_stage

    with pytest.raises(commands.MotionBlocked):
        dispatch_b15_event(machine, event_key)

    assert machine.state is state_before, (state_name, event_key, machine.state)
    assert machine.active_stage is stage_before, (state_name, event_key, machine.active_stage)
    # No illegal supervision state may be manufactured by a refused command.
    assert machine.state is not commands.ControlState.BLOCKED
    assert machine.state is not commands.ControlState.FAULT


# Ruling revision (P2 B3/B9.1, 2025-xx): `handle(ResumeCommand())` is unified
# with `resume()` and is NON-throwing from every state.  The two out-of-graph
# resume_command combinations therefore keep the exact state/active_stage
# invariance assertions above but replace `pytest.raises(MotionBlocked)` with a
# plain call (asserting the absence of any exception).
#   - STANDBY + ResumeCommand : settle window not yet full -> stays STANDBY.
#   - ACTIVE  + ResumeCommand : refused without raising -> stays ACTIVE/ARM.
B15_NON_THROWING_RESUME_CASES = (
    ("STANDBY", "resume_command"),
    ("ACTIVE", "resume_command"),
)


@pytest.mark.parametrize(
    "state_name,event_key",
    B15_NON_THROWING_RESUME_CASES,
    ids=[f"{state}-{event}" for state, event in B15_NON_THROWING_RESUME_CASES],
)
def test_b15_resume_command_out_of_graph_is_non_throwing_and_state_invariant(
    state_name: str, event_key: str
) -> None:
    """T-B1.5：图外的 ResumeCommand 非抛出拒绝且状态不变。"""
    commands, machine = machine_in_state(state_name)
    state_before = machine.state
    stage_before = machine.active_stage

    dispatch_b15_event(machine, event_key)  # ruling: must not raise

    assert machine.state is state_before, (state_name, event_key, machine.state)
    assert machine.active_stage is stage_before, (state_name, event_key, machine.active_stage)
    # No illegal supervision state may be manufactured by a refused command.
    assert machine.state is not commands.ControlState.BLOCKED
    assert machine.state is not commands.ControlState.FAULT


# ---------------------------------------------------------------------------
# T-B1.5.14 — resume() must only unlock from STANDBY (READY is idempotent).
# A resume() call from ACTIVE must never masquerade as READY, and must never
# leave the illegal READY + active_stage combination.  This is the behavior the
# current implementation still gets wrong (settle window is checked, the source
# state is not): ACTIVE + resume() jumps to READY with Stage.ARM still set.
# The refusal MECHANISM (silent last_reason vs MotionBlocked) is not bound here,
# only the safety invariant.
# ---------------------------------------------------------------------------
def test_b15_resume_from_active_never_unlocks_or_leaves_stale_stage() -> None:
    """T-B1.5：ACTIVE 中 resume 不解锁，也不留残余阶段。"""
    commands, machine = active_machine("arm_left")
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM

    with contextlib.suppress(commands.MotionBlocked):
        machine.resume()

    assert machine.state is commands.ControlState.ACTIVE
    assert machine.state is not commands.ControlState.READY
    assert machine.active_stage is commands.Stage.ARM
    # The forbidden combination: READY while a stage is still serviced.
    assert not (machine.state is commands.ControlState.READY and machine.active_stage is not None)


# ---------------------------------------------------------------------------
# T-B1.5.15 — resume() from SETTLING is refused too: SETTLING only leaves via
# the settle window (-> READY) or stop() (-> STANDBY).
# ---------------------------------------------------------------------------
def test_b15_resume_from_settling_is_refused() -> None:
    """T-B1.5：SETTLING 中 resume 被拒绝。"""
    commands, machine = released_machine("arm_left")
    assert machine.state is commands.ControlState.SETTLING

    with contextlib.suppress(commands.MotionBlocked):
        machine.resume()

    assert machine.state is commands.ControlState.SETTLING
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B1.5.16 — B2.3 boundary: a repeated SAME-stage request while ACTIVE is
# allowed-or-refused by B2.3, which is out of scope here.  This stage only binds
# that it never yields an illegal supervision state (no BLOCKED/FAULT, no
# leaving ACTIVE) and never drops the active stage.  Cross-stage concurrency
# (B2 -> BLOCKED) is deliberately NOT exercised.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ARM_COMMAND_KEYS)
def test_b15_same_stage_motion_in_active_stays_active_without_blocked(
    key: str,
) -> None:
    """T-B1.5.16：ACTIVE 中的同阶段重复请求保持 ACTIVE 且不 BLOCKED。"""
    cmd_types = load_command_types()
    commands, machine = active_machine("arm_left")
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM
    assert machine.state is not commands.ControlState.BLOCKED
    assert machine.state is not commands.ControlState.FAULT


# ===========================================================================
# Behavior B1.6 / P2 终审 finding F2 (design §2「失败显式报错」):
#   命令分派对「未知命令类型」必须显式拒绝：READY 不是软状态，未知命令对象
#   （如 object()）必须与其余五态一致地抛 MotionBlocked，禁止静默吞掉。
#
# Protected acceptance:
# - 六态（STANDBY/READY/ACTIVE/SETTLING/BLOCKED/FAULT）下 handle(未知对象) ->
#   MotionBlocked，且 state / active_stage 不变，不制造新的 BLOCKED/FAULT。
#
# Seam frozen:
#   drive : ControlStateMachine.handle(<unknown object>) -> MotionBlocked。
#
# This binding discriminates the reasonable-but-WRONG implementations:
# - READY 分支把非运动 / 非 release / 非 resume 的命令当幂等 no-op 静默返回
#   （现状），调用方无法区分「已接受」与「被拒绝」（T-B1.6.1/T-B1.6.2）；
# - 只对 READY 特判抛错、其余态放行，六态不一致（T-B1.6.1）。
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# ===========================================================================

F2_ALL_STATES = ("STANDBY", "READY", "ACTIVE", "SETTLING", "BLOCKED", "FAULT")


def machine_in_f2_state(state_name: str) -> tuple[SimpleNamespace, Any]:
    """构造 F2 关注的六个监督状态（BLOCKED 走 B2，FAULT 走 B5）。"""
    if state_name == "FAULT":
        return faulted_machine()
    return machine_in_any_state(state_name)


# ---------------------------------------------------------------------------
# T-B1.6.1 — 六态一致性：未知命令类型在任一监督状态下都必须显式抛
# MotionBlocked，且状态 / stage 不变。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state_name", F2_ALL_STATES)
def test_b16_unknown_command_type_is_refused_from_every_state(state_name: str) -> None:
    """T-B1.6.1：六态一致，未知命令类型一律抛 MotionBlocked 且状态不变。"""
    commands, machine = machine_in_f2_state(state_name)
    state_before = machine.state
    stage_before = machine.active_stage
    assert state_before.name == state_name

    with pytest.raises(commands.MotionBlocked):
        machine.handle(object())

    assert machine.state is state_before, (state_name, machine.state)
    assert machine.active_stage is stage_before, (state_name, machine.active_stage)


# ---------------------------------------------------------------------------
# T-B1.6.2 — READY 是发现 F2 的状态：未知命令必须被显式拒绝，而不是被当成
# 幂等 no-op 静默返回（现状），也不得顺带制造 BLOCKED/FAULT。
# ---------------------------------------------------------------------------
def test_b16_ready_unknown_command_is_not_silently_swallowed() -> None:
    """T-B1.6.2：READY 下的未知命令被显式拒绝，不得静默吞掉。"""
    commands, machine = ready_machine()
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None

    with pytest.raises(commands.MotionBlocked):
        machine.handle(object())

    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None
    assert machine.state is not commands.ControlState.BLOCKED
    assert machine.state is not commands.ControlState.FAULT


# ===========================================================================
# Behavior B2 (D003 阶段互锁, design §6 规则 4 与状态图):
#   跨阶段并发 -> BLOCKED；同阶段并发允许；BLOCKED 需条件恢复。
#
# Protected acceptance:
# - B2.1: ACTIVE(ARM) 收到 BASE_LIFT 类别请求 -> BLOCKED，记录 reason，请求不产生推进；
# - B2.2: ACTIVE(BASE_LIFT) 收到 ARM 类别请求 -> BLOCKED（对称）；
# - B2.3: 同阶段请求（含左右臂切换 / 夹爪）允许，保持 ACTIVE(stage)，不置 BLOCKED；
# - B2.4: BLOCKED + 全释放（ReleaseCommand）+ 停稳窗口证实 -> READY；
#   未满足任一条件保持 BLOCKED；BLOCKED 期间拒绝一切运动推进；
#   BLOCKED + stop() -> STANDBY（D004 停机优先，无条件）；
# - A5: 同一 tick 的跨类别冲突在顺序事件批下确定性置 BLOCKED。
#
# Seam frozen for B2 (returned to the main agent for consolidation):
#   entry : ControlStateMachine.handle(<cross-stage motion command>)
#             ACTIVE(stage) -> BLOCKED, .last_reason set, active_stage cleared,
#             no stage advance, AND MotionBlocked raised (single authoritative
#             contract, aligned with V03).
#   same  : ACTIVE(stage) + same-category command -> no raise, stays ACTIVE(stage).
#   recover: ControlStateMachine.handle(ReleaseCommand())
#             BLOCKED -> "released", still BLOCKED until the settle window lands;
#           ControlStateMachine.on_feedback(id, settled=True, valid=True)
#             N consecutive FRESH settled samples -> BLOCKED -> READY (idle).
#   stop  : ControlStateMachine.stop()  BLOCKED -> STANDBY unconditionally (D004).
#
# P2 终审 finding F1 授权收紧（第二次既有用例修改，2026-xx，主 Agent 已裁定）:
#   T-B2.1.*/T-B2.2.*/T-B2-A5 原以 contextlib.suppress(MotionBlocked) 双向失盲：
#   既不要求抛错、也不禁止抛错，于是「只置 BLOCKED 但不抛」的静默失败与
#   「抛错但不置 BLOCKED」都被放行。现收紧为 pytest.raises(MotionBlocked) +
#   状态断言（BLOCKED、active_stage is None、last_reason 非空）。
#   diff 摘要：
#     - T-B2.1/T-B2.2: `with contextlib.suppress(MotionBlocked):` ->
#       `with pytest.raises(commands.MotionBlocked):`；
#       `assert machine.active_stage is not commands.Stage.BASE_LIFT/ARM` ->
#       `assert machine.active_stage is None`（更强的「stage 清空」）。
#     - T-B2-A5: 批内第一个运动命令显式非抛出（READY -> ACTIVE），第二个
#       冲突命令 `pytest.raises(MotionBlocked)`；新增 active_stage is None。
#     - 仅改断言机制，未改状态序列、参数化顺序或既有不变性语义。
#
# This binding discriminates reasonable-but-WRONG implementations:
# - last-writer-wins: a cross-stage request overwrites active_stage and stays
#   ACTIVE (T-B2.1.*, T-B2.2.*);
# - one-directional lock: ARM->BASE_LIFT blocks but the symmetric direction does
#   not (T-B2.2.*);
# - same-stage request wrongly treated as a conflict and refused (T-B2.3.*);
# - BLOCKED recovering with no conditions: auto-READY on feedback alone
#   (T-B2.4.2), release alone without the window (T-B2.4.1/T-B2.4.4), or a
#   resume() shortcut bypassing the release gate (T-B2.4.3);
# - BLOCKED still accepting motion or leaving a stale active stage at READY
#   (T-B2.4.5 / T-B2.4.1);
# - stop() no longer hard-stopping from BLOCKED (T-B2.4.6);
# - cross-stage conflict that silently sets BLOCKED without raising MotionBlocked
#   (caller cannot distinguish a refused request from an accepted one) or that
#   raises without leaving the hardened BLOCKED state (T-B2.1.*/T-B2.2.*/T-B2-A5).
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# Expected values are frozen literals; nothing is read back from the
# implementation under test beyond the public read seam.
# ===========================================================================

# Sample ids reserved for the B2 recovery window; distinct from the STANDBY gate
# ids (1..N) and the B1.4 ids (10_000+) so "fresh" is explicit.
B2_SAMPLE_BASE = 20_000


def opposite_stage_key(stage_key: str) -> str:
    """Return a representative motion key of the *other* stage category."""
    return "base_drive" if stage_key in ARM_COMMAND_KEYS else "arm_left"


def blocked_machine(stage_key: str, settled_samples: int = 3) -> tuple[SimpleNamespace, Any]:
    """Drive ACTIVE(stage) then dispatch one cross-stage request -> BLOCKED.

    The cross-stage command is suppressed here so this *state constructor*
    works regardless of the raise mechanism; the F1 contract (MotionBlocked
    raised AND BLOCKED entered with a cleared stage) is bound explicitly by
    T-B2.1.* / T-B2.2.* / T-B2-A5.
    """
    cmd_types = load_command_types()
    commands, machine = active_machine(stage_key, settled_samples=settled_samples)
    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, opposite_stage_key(stage_key)))
    assert machine.state is commands.ControlState.BLOCKED, (stage_key, machine.state)
    return commands, machine


# ---------------------------------------------------------------------------
# T-B2.1.* — ACTIVE(ARM) + any BASE_LIFT-category request -> MotionBlocked raised
# AND BLOCKED entered with a recorded reason, active_stage cleared, and the
# requested BASE_LIFT category never serviced (F1 authoritative contract).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("cross_key", BASE_LIFT_COMMAND_KEYS)
def test_b21_active_arm_cross_stage_base_request_blocks(cross_key: str) -> None:
    """T-B2.1：ACTIVE(ARM) 收到 BASE_LIFT 类请求进入 BLOCKED。"""
    cmd_types = load_command_types()
    commands, machine = active_machine("arm_left")
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM

    with pytest.raises(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, cross_key))

    assert machine.state is commands.ControlState.BLOCKED
    assert machine.active_stage is None
    assert isinstance(machine.last_reason, str) and machine.last_reason


# ---------------------------------------------------------------------------
# T-B2.2.* — ACTIVE(BASE_LIFT) + any ARM-category request -> MotionBlocked
# raised AND BLOCKED (symmetry).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("cross_key", ARM_COMMAND_KEYS)
def test_b22_active_base_lift_cross_stage_arm_request_blocks(cross_key: str) -> None:
    """T-B2.2：ACTIVE(BASE_LIFT) 收到 ARM 类请求进入 BLOCKED。"""
    cmd_types = load_command_types()
    commands, machine = active_machine("base_drive")
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.BASE_LIFT

    with pytest.raises(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, cross_key))

    assert machine.state is commands.ControlState.BLOCKED
    assert machine.active_stage is None
    assert isinstance(machine.last_reason, str) and machine.last_reason


# ---------------------------------------------------------------------------
# T-B2.3.* — a same-category request while ACTIVE is ALLOWED: it must not raise,
# keeps ACTIVE(stage) and never manufactures BLOCKED/FAULT.  Covers left/right
# arm switching and gripper commands (A7) as well as base/lift sub-types.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ARM_COMMAND_KEYS)
def test_b23_same_stage_arm_request_stays_active_arm(key: str) -> None:
    """T-B2.3：同阶段的 ARM 请求保持 ACTIVE(ARM)。"""
    cmd_types = load_command_types()
    commands, machine = active_machine("arm_left")

    machine.handle(build_motion_command(cmd_types, key))  # allowed: must not raise

    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM
    assert machine.state is not commands.ControlState.BLOCKED
    assert machine.state is not commands.ControlState.FAULT


@pytest.mark.parametrize("key", BASE_LIFT_COMMAND_KEYS)
def test_b23_same_stage_base_lift_request_stays_active_base_lift(key: str) -> None:
    """T-B2.3：同阶段的 BASE_LIFT 请求保持 ACTIVE(BASE_LIFT)。"""
    cmd_types = load_command_types()
    commands, machine = active_machine("base_drive")

    machine.handle(build_motion_command(cmd_types, key))  # allowed: must not raise

    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.BASE_LIFT
    assert machine.state is not commands.ControlState.BLOCKED
    assert machine.state is not commands.ControlState.FAULT


# ---------------------------------------------------------------------------
# T-B2.4.1 — the canonical recovery: full release, then N fresh settled samples.
# Release alone is not enough; N-1 samples are not enough; the N-th reaches
# READY (idle, no residual stage).
# ---------------------------------------------------------------------------
def test_b24_full_release_and_settle_window_reaches_ready() -> None:
    """T-B2.4.1：完整释放加停稳窗口后回到 READY。"""
    completion = load_completion_types()
    commands, machine = blocked_machine("arm_left", settled_samples=3)

    machine.handle(completion.ReleaseCommand())  # 全释放路径，不得抛错
    assert machine.state is commands.ControlState.BLOCKED  # window not confirmed yet

    feed_count(machine, B2_SAMPLE_BASE, 2, settled=True)  # N - 1
    assert machine.state is commands.ControlState.BLOCKED

    feed(machine, B2_SAMPLE_BASE + 2, settled=True)  # N-th
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B2.4.2 — without the full release the settle window alone never recovers;
# BLOCKED does not auto-heal from feedback ("无需条件即恢复" 被拒).
# ---------------------------------------------------------------------------
def test_b24_settle_window_without_release_stays_blocked() -> None:
    """T-B2.4.2：只有停稳窗口、没有完整释放仍需 BLOCKED。"""
    commands, machine = blocked_machine("arm_left", settled_samples=2)

    feed_count(machine, B2_SAMPLE_BASE, 6, settled=True)  # far beyond the window

    assert machine.state is commands.ControlState.BLOCKED
    assert machine.state is not commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B2.4.3 — resume() is not a BLOCKED recovery shortcut: even with a full
# settle window, resume() must not bypass the release gate.
# ---------------------------------------------------------------------------
def test_b24_resume_from_blocked_does_not_bypass_release_gate() -> None:
    """T-B2.4.3：BLOCKED 下的 resume 不绕过释放门禁。"""
    commands, machine = blocked_machine("arm_left", settled_samples=2)

    feed_count(machine, B2_SAMPLE_BASE, 2, settled=True)  # window full, no release
    with contextlib.suppress(commands.MotionBlocked):
        machine.resume()

    assert machine.state is commands.ControlState.BLOCKED


# ---------------------------------------------------------------------------
# T-B2.4.4 — release without confirmed standstill stays BLOCKED: neither
# non-settled nor invalid fresh samples are accepted as standstill proof.
# ---------------------------------------------------------------------------
def test_b24_release_without_confirmed_standstill_stays_blocked() -> None:
    """T-B2.4.4：未确认停稳的释放仍停留 BLOCKED。"""
    completion = load_completion_types()
    commands, machine = blocked_machine("arm_left", settled_samples=2)

    machine.handle(completion.ReleaseCommand())
    feed_count(machine, B2_SAMPLE_BASE, 3, settled=False)
    assert machine.state is commands.ControlState.BLOCKED

    machine.on_feedback(B2_SAMPLE_BASE + 50, True, valid=False)
    assert machine.state is commands.ControlState.BLOCKED


# ---------------------------------------------------------------------------
# T-B2.4.5 — BLOCKED is a hardened state: it refuses every motion advance with
# MotionBlocked and stays BLOCKED.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ALL_MOTION_COMMAND_KEYS)
def test_b24_blocked_refuses_every_motion_advance(key: str) -> None:
    """T-B2.4.5：BLOCKED 拒绝任何运动推进。"""
    cmd_types = load_command_types()
    commands, machine = blocked_machine("arm_left")

    with pytest.raises(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.BLOCKED


# ---------------------------------------------------------------------------
# T-B2.4.6 — D004 stop priority: stop() from BLOCKED unconditionally targets
# STANDBY and clears any residual stage.
# ---------------------------------------------------------------------------
def test_b24_stop_from_blocked_targets_standby() -> None:
    """T-B2.4.6：D004 停机优先，BLOCKED 下 stop 目标 STANDBY。"""
    commands, machine = blocked_machine("base_drive")

    machine.stop()

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B2-A5 — the same-tick conflicting-categories case, modelled as an ordered
# event batch: the first category legitimately enters ACTIVE, the conflicting
# second one raises MotionBlocked and hard-blocks the machine.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    "order",
    [("arm_left", "base_drive"), ("base_drive", "arm_left")],
    ids=["arm-then-base", "base-then-arm"],
)
def test_b2_a5_same_tick_conflicting_batch_blocks(order: tuple[str, str]) -> None:
    """T-B2-A5：同一 tick 内冲突类别的批处理进入 BLOCKED。"""
    cmd_types = load_command_types()
    commands, machine = ready_machine()

    # 批内第一个运动命令是合法边：READY -> ACTIVE(stage)，不得抛出。
    machine.handle(build_motion_command(cmd_types, order[0]))
    assert machine.state is commands.ControlState.ACTIVE

    # 第二个命令与已服务的阶段冲突：必须置 BLOCKED 且抛 MotionBlocked。
    with pytest.raises(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, order[1]))

    assert machine.state is commands.ControlState.BLOCKED
    assert machine.active_stage is None
    assert isinstance(machine.last_reason, str) and machine.last_reason


# ===========================================================================
# Behavior B3 + B9.1 (实施包 P2，design §6 与 D004「停机最高优先」):
#   命令优先级 stop > resume > motion。
#
# Protected acceptance（本阶段裁定冻结的两条语义）:
# - B3.1/B3.2: handle(StopCommand()) 在任意状态（含 ACTIVE/SETTLING/BLOCKED）
#   等价 stop() -> STANDBY：无条件、立即生效（不排队、不忽略），清 active_stage；
#   同一 tick 的「运动请求 + stop」（顺序事件批两种顺序建模）最终态必须是 STANDBY
#   且无残余推进；即使运动先到使状态发生变化，随后的 stop 仍回到 STANDBY。
# - B3.3: stop 之后未 resume 时重发任何类别的运动请求 -> 被拒绝，不自动续动；
#   即使随后收到远超停稳窗口的新鲜停稳反馈，也不得自动重新接受运动
#   （resume()/ResumeCommand 之外的路径不得解锁）。
# - B9.1: 同一 tick 批 [motion, resume, stop] / [stop, motion] / [stop, resume] /
#   [motion, stop, resume] 的最终态确定：stop 压过 resume 与 motion，
#   批次结束后为 STANDBY 且 active_stage is None。
#
# Seam frozen for B3/B9.1（主 Agent 已裁定，P2 B3/B9.1）:
#   drive : ControlStateMachine.handle(StopCommand())  任意态 -> STANDBY，不抛出
#           （裁定用语是「等价 stop()」，而 stop() 从不抛出：停机是安全路径，
#            调用方不该为了停机去捕获异常。故此处绑定「不得抛出」。）
#           ControlStateMachine.handle(ResumeCommand()) == resume() 语义：
#           STANDBY 走解锁判定、READY 幂等、其它态非抛出拒绝；
#           resume() 命令不得绕过 SETTLING/BLOCKED 的门禁。
#
# binding 冲突已解决（P2 B3/B9.1 裁定）:
#   原本 B1.5 把 ("ACTIVE", "resume_command") / ("STANDBY", "resume_command")
#   冻结为「抛 MotionBlocked」，与本阶段「非抛出拒绝」冲突。主 Agent 裁定统一为
#   非抛出语义，并授权修订这两个既有用例（已从 MotionBlocked 矩阵移入
#   B15_NON_THROWING_RESUME_CASES，仅改断言机制，其它不变性断言保留）。
#   因此 T-B9.2.2 的 suppress 仅为兼容历史写法，不再覆盖真实分歧。
#
# 本 binding 有意区分以下 reasonable-but-WRONG 实现:
# - handle(StopCommand()) 被忽略 / 被当成等待下一 tick 才生效的请求：调用返回后
#   状态必须已经是 STANDBY（T-B3.1.1，含 READY 静默 no-op 的现状）；
# - StopCommand 只在 READY 特判（其余状态照旧抛异常或 no-op）：T-B3.1.1 覆盖
#   STANDBY/ACTIVE/SETTLING/BLOCKED；
# - stop 只改状态、不清理停稳证据：stop 后一次 resume()（无新反馈）就会重新
#   READY（T-B3.1.2）；
# - stop 之后运动请求仍被服务 / 或 stop 只作用于「后到的」运动：T-B3.2.1 两种
#   顺序、T-B3.3.1 全类别；
# - stop 后仅凭新鲜停稳反馈就自动续动（跳过显式 resume）：T-B3.3.2；
# - resume 命令无条件解锁 / 在 stop 之后又把机器拉回 READY：T-B9.1 的
#   ("stop","resume") 与 ("motion","stop","resume") 批次、T-B9.2.2；
# - resume 命令绕过 SETTLING/BLOCKED 门禁：T-B9.2.3 / T-B9.2.4。
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# Expected values are frozen literals; nothing is read back from the
# implementation under test beyond the public read seam.
# ===========================================================================

# 本阶段自用的新鲜反馈 id 段，与 B1.1(1..N)、B1.4(10_000+)、B2(20_000+) 区分开。
B3_SAMPLE_BASE = 30_000

# B3.1 的「任意状态」集合：B1.5 可合法构造的四个状态 + B2 的 BLOCKED。
B3_ANY_STATES = ("STANDBY", "READY", "ACTIVE", "SETTLING", "BLOCKED")


def machine_in_any_state(state_name: str, settled_samples: int = 2) -> tuple[SimpleNamespace, Any]:
    """构造 B3 关注的状态之一；BLOCKED 复用 B2 的跨阶段冲突路径。"""
    if state_name == "BLOCKED":
        return blocked_machine("arm_left", settled_samples=settled_samples)
    return machine_in_state(state_name, settled_samples=settled_samples)


def dispatch_priority_event(machine: Any, event_key: str) -> None:
    """派发 B3.2/B9.1 事件批中的一个事件（运动/恢复/停机）。"""
    cmd_types = load_command_types()
    if event_key == "motion":
        machine.handle(build_motion_command(cmd_types, "arm_left"))
    elif event_key == "resume":
        machine.handle(load_b15_types().ResumeCommand())
    elif event_key == "stop":
        machine.handle(cmd_types.StopCommand())
    else:
        raise AssertionError(f"unknown priority event key: {event_key}")


# ---------------------------------------------------------------------------
# T-B3.1.1 — D004: handle(StopCommand()) 从任意状态无条件目标 STANDBY，且不抛出。
# 覆盖：STANDBY（现状抛异常）、READY（现状静默 no-op，最危险：停机被忽略）、
# ACTIVE / SETTLING / BLOCKED（现状抛异常，停机被「仅在某状态有效」地拒绝）。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state_name", B3_ANY_STATES)
def test_b31_stop_command_targets_standby_from_every_state(state_name: str) -> None:
    """T-B3.1.1：StopCommand 从任意状态无条件目标 STANDBY 且不抛出。"""
    cmd_types = load_command_types()
    commands, machine = machine_in_any_state(state_name)
    assert machine.state.name == state_name

    machine.handle(cmd_types.StopCommand())  # 立即生效：返回后即为停机态

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B3.1.2 — 停机立即终止推进且不残留解锁证据：stop 之后（无新反馈）直接 resume()
# 不得重新解锁；必须提供新鲜停稳证据后 resume() 才回到 READY（停机不等于把机器
# 卡死，也不等于保留旧的解锁条件）。
# ---------------------------------------------------------------------------
def test_b31_stop_command_clears_residual_unlock_evidence() -> None:
    """T-B3.1.2：停机立即终止推进，不残留解锁证据。"""
    cmd_types = load_command_types()
    commands, machine = machine_in_state("READY", settled_samples=2)
    assert machine.state is commands.ControlState.READY  # 停稳窗口已满

    machine.handle(cmd_types.StopCommand())
    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None

    machine.resume()  # 无新证据：不得解锁（旧窗口不得被复用）
    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None

    feed_count(machine, B3_SAMPLE_BASE, 2, settled=True)  # 新鲜证据
    machine.resume()
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B3.2.1 — 同一 tick 的「运动请求 + stop」，两种顺序建模；无论谁先到，
# 最终态都是 STANDBY 且无残余推进（stop 压过运动；运动先到产生的中途状态
# 不得存活到 tick 结束）。拒绝机制不绑定（B3.1 已绑定 stop 不抛出，运动被拒
# 可抛可静默），因此逐事件 suppress MotionBlocked。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state_name", ("READY", "ACTIVE", "SETTLING"))
@pytest.mark.parametrize(
    "order",
    (("motion", "stop"), ("stop", "motion")),
    ids=["motion-then-stop", "stop-then-motion"],
)
def test_b32_same_tick_motion_and_stop_resolves_to_standby(
    state_name: str, order: tuple[str, str]
) -> None:
    """T-B3.2.1：同一 tick 的运动加 stop，两种顺序都收敛到 STANDBY。"""
    commands, machine = machine_in_any_state(state_name)

    for event_key in order:
        with contextlib.suppress(commands.MotionBlocked):
            dispatch_priority_event(machine, event_key)

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B3.3.1 — stop 之后未 resume 时重发任何类别的运动请求都被拒绝，不自动续动。
# 覆盖 ARM 与 BASE_LIFT 两个类别的全部代表命令（防止「按类别特判放过」）。
# 拒绝机制不绑定（可抛 MotionBlocked，也可静默），只绑定安全不变量。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ALL_MOTION_COMMAND_KEYS)
def test_b33_motion_after_stop_command_is_refused_until_resume(key: str) -> None:
    """T-B3.3.1：停机后未 resume 前任何类别的运动请求都被拒绝。"""
    cmd_types = load_command_types()
    commands, machine = active_machine("arm_left")

    machine.handle(cmd_types.StopCommand())
    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# --------------------------------------------------------------------------
# T-B3.3.2 — stop 之后即便收到远超窗口的新鲜停稳反馈，也不得自动重新接受运动。
# 捕获「stop 只是临时标记，反馈一停稳就自行续动/自行解锁」的实现。
# --------------------------------------------------------------------------
def test_b33_settled_feedback_after_stop_does_not_resume_motion() -> None:
    """T-B3.3.2：停机后即便收到新鲜停稳反馈也不自动续动。"""
    cmd_types = load_command_types()
    commands, machine = active_machine("arm_left")

    machine.handle(cmd_types.StopCommand())
    feed_count(machine, B3_SAMPLE_BASE, 6, settled=True)  # 远超窗口

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, "arm_left"))

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B9.1.1 — 命令优先级 stop > resume > motion：同一 tick 事件批（顺序建模）
# 的最终态确定性断言。每个批次的合法/非法事件混在一起，拒绝机制不绑定，
# 只绑定「stop 存在则最终 STANDBY 且无残余 stage」。
# ---------------------------------------------------------------------------
B9_PRIORITY_BATCHES = (
    ("motion", "resume", "stop"),
    ("stop", "motion"),
    ("stop", "resume"),
    ("motion", "stop", "resume"),
)


@pytest.mark.parametrize(
    "batch",
    B9_PRIORITY_BATCHES,
    ids=["-".join(batch) for batch in B9_PRIORITY_BATCHES],
)
def test_b91_stop_dominates_resume_and_motion_in_same_tick_batch(batch: tuple[str, ...]) -> None:
    """T-B9.1.1：命令优先级 stop 高于 resume 高于 motion 的最终态确定。"""
    commands, machine = ready_machine()

    for event_key in batch:
        with contextlib.suppress(commands.MotionBlocked):
            dispatch_priority_event(machine, event_key)

    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B9.2.1 — READY + ResumeCommand()：幂等放行（A9 的命令路径），不抛出、保持
# READY、不留 stage。（本阶段为回归守卫：现状与裁定一致，不是新 RED。）
# ---------------------------------------------------------------------------
def test_b92_resume_command_in_ready_is_idempotent() -> None:
    """T-B9.2.1：READY 下的 ResumeCommand 幂等放行且不留残余阶段。"""
    b15 = load_b15_types()
    commands, machine = machine_in_state("READY")

    machine.handle(b15.ResumeCommand())  # must not raise

    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B9.2.2 — STANDBY + ResumeCommand() 必须真的走解锁判定：窗口未满不解锁
# （机制不绑定：既有 B1.5 用例要求此输入抛 MotionBlocked，故 suppress），
# 窗口满则必须经命令路径解锁到 READY。捕获「ResumeCommand 在 STANDBY 恒为
# 死 no-op」以及「ResumeCommand 恒解锁（绕过门禁）」两种实现。
# ---------------------------------------------------------------------------
def test_b92_resume_command_in_standby_runs_unlock_gate() -> None:
    """T-B9.2.2：STANDBY 下的 ResumeCommand 仍走解锁判定。"""
    b15 = load_b15_types()
    commands, machine = make_machine(settled_samples=2)

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(b15.ResumeCommand())  # 停稳窗口未满
    assert machine.state is commands.ControlState.STANDBY
    assert machine.active_stage is None

    feed_count(machine, B3_SAMPLE_BASE, 2, settled=True)  # 窗口满
    machine.handle(b15.ResumeCommand())

    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B9.2.3 — SETTLING + ResumeCommand()：非抛出拒绝，状态不变（SETTLING 只能
# 经停稳窗口 -> READY 或 stop -> STANDBY 离开）。
# ---------------------------------------------------------------------------
def test_b92_resume_command_in_settling_is_non_throwing_refusal() -> None:
    """T-B9.2.3：SETTLING 下的 ResumeCommand 非抛出拒绝且状态不变。"""
    b15 = load_b15_types()
    commands, machine = machine_in_state("SETTLING")
    assert machine.state is commands.ControlState.SETTLING

    machine.handle(b15.ResumeCommand())  # 裁定：其它态非抛出拒绝

    assert machine.state is commands.ControlState.SETTLING
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B9.2.4 — BLOCKED + ResumeCommand()：非抛出拒绝且不绕过释放门禁（T-B2.4.3
# 的命令路径镜像）。捕获「resume 命令是 BLOCKED 恢复捷径」的实现。
# ---------------------------------------------------------------------------
def test_b92_resume_command_in_blocked_is_non_throwing_refusal() -> None:
    """T-B9.2.4：BLOCKED 下的 ResumeCommand 非抛出拒绝且不绕过门禁。"""
    b15 = load_b15_types()
    commands, machine = machine_in_any_state("BLOCKED")
    assert machine.state is commands.ControlState.BLOCKED

    machine.handle(b15.ResumeCommand())  # 裁定：其它态非抛出拒绝

    assert machine.state is commands.ControlState.BLOCKED
    assert machine.active_stage is None


# ===========================================================================
# Behavior B5 (实施包 P2「通用命令与状态抽象」, design §6 异常分支与 V05):
#   反馈完整性异常 -> FAULT；FAULT 状态隔离与显式恢复。
#
# Protected acceptance:
# - B5.2: 反馈完整性异常（严重的 sample_id 倒退 / 乱序）从 ACTIVE 置 FAULT；
#   必须区分「陈旧重复（已见过的旧 id）仅忽略」与「倒退（未见过但低于高水位
#   的 id）-> FAULT」；FAULT 后隔离：active_stage 清空、不接受运动、不被其它
#   状态污染，也不因后续新鲜反馈自愈。
# - B5.3: FAULT + 显式恢复入口 reset() -> STANDBY（不是 READY）；禁止
#   FAULT 直接 -> READY；resume()（方法或 ResumeCommand）不得从 FAULT 解锁；
#   恢复后必须重新满足 STANDBY 停稳门禁才能再次 READY（旧证据不得复用）。
# - B5.4: FAULT 期间拒绝一切运动命令与 release（MotionBlocked），状态不变。
#
# Seam frozen for B5 (reported to the main agent for consolidation):
#   drive : ControlStateMachine.on_feedback(sample_id, settled, valid=True)
#             已见 id（重复 / 陈旧）-> 静默忽略，绝不 FAULT；
#             未见过且低于已接受高水位的 id（倒退）-> FAULT
#             （design §6 边 `ACTIVE --> FAULT: 反馈丢帧 / 乱序 / 跳变 / 重启`）。
#   drive : ControlStateMachine.reset()   FAULT -> STANDBY（唯一显式恢复入口；
#             design §6 边 `FAULT --> STANDBY: 故障解除 + 重新连接`）。reset()
#             只在 FAULT 源态被绑定；其在其它源态的行为不在本阶段冻结。
#   read  : 复用 .state / .active_stage / .last_reason。
#
# 非绑定 / 待裁定项（见交付说明）:
# - B5.1 的 report_blocked(reason) 外部入口（限位 / IK / 过流 -> BLOCKED）本
#   阶段不冻结：触发源在 P4-B controller，过早固定 API 名称 / 签名有返工风险；
# - 反馈倒退从 ACTIVE 之外（READY / SETTLING / BLOCKED / STANDBY）触发 FAULT
#   是否必需（design 图只画了 ACTIVE -> FAULT）；
# - reset() 是否也应清空已见 sample_id 集合。
#
# P2 终审 finding F3 已裁定冻结（见 T-B5.5.*）:
#   FAULT + stop()（方法或 StopCommand）保持 FAULT：不抛、状态不变、故障原因
#   保持可见；解除故障只能走显式 reset()。故从上述「待裁定项」移除。
#
# This binding deliberately discriminates reasonable-but-WRONG implementations:
# - 倒退被静默忽略、当作普通新样本（T-B5.2.1）；
# - 把「重复 / 陈旧 id」也当作倒退而误报 FAULT（T-B5.2.2）；
# - FAULT 后保留 active_stage / 仍接受运动 / 自行愈合（T-B5.2.3、T-B5.4.1/2）；
# - resume()（方法或命令）从 FAULT 借 FAULT 前的停稳证据直接解锁到 READY
#   （T-B5.3.1/T-B5.3.2）；
# - reset() 从 FAULT 直接进 READY 而非 STANDBY（T-B5.3.3）；
# - reset() 后复用 FAULT 前的停稳证据再次 READY（T-B5.3.4）；
# - D004「停机优先」被过度泛化：stop()/StopCommand 从 FAULT 把故障清成 STANDBY，
#   使故障原因丢失、恢复路径被跳过（T-B5.5.1/T-B5.5.2）。
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# Expected values are frozen literals; nothing is read back from the
# implementation under test beyond the public read seam.
# ===========================================================================

# 本阶段自用的新鲜反馈 id 段，与 B1.1(1..N)、B1.4(10_000+)、B2(20_000+)、
# B3(30_000+) 区分开。
B5_SAMPLE_BASE = 40_000


def faulted_machine(
    stage_key: str = "arm_left", settled_samples: int = 2
) -> tuple[SimpleNamespace, Any]:
    """Drive ACTIVE, fill the settle window, then inject a sample_id regression.

    The machine is expected to FAULT through the design §6 `ACTIVE -> FAULT`
    edge only.  The settle window is deliberately full before the regression so
    that a later resume() would have stale evidence to abuse if the FAULT
    handling forgot to clear it.
    """
    commands, machine = active_machine(stage_key, settled_samples=settled_samples)
    feed_count(machine, B5_SAMPLE_BASE, settled_samples, settled=True)  # window full
    machine.on_feedback(B5_SAMPLE_BASE - 10, settled=True)  # unseen id below the high-water mark
    assert machine.state is commands.ControlState.FAULT, (stage_key, machine.state)
    return commands, machine


# ---------------------------------------------------------------------------
# T-B5.2.1 — a never-seen sample_id below the accepted high-water mark (severe
# out-of-order / regression) while ACTIVE drives the machine to FAULT.  Covers
# both stage categories.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("stage_key", ["arm_left", "base_drive"])
def test_b52_sample_id_regression_from_active_faults(stage_key: str) -> None:
    """T-B5.2.1：ACTIVE 下 sample_id 倒退锁存 FAULT。"""
    commands, machine = faulted_machine(stage_key)

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.ACTIVE
    assert machine.active_stage is None  # isolation: no residual serviced stage
    assert isinstance(machine.last_reason, str) and machine.last_reason


# ---------------------------------------------------------------------------
# T-B5.2.2 — the discriminator for B5.2: repeated / stale ids (already seen) are
# only IGNORED — they never FAULT, even though they are numerically below the
# high-water mark.  Forward progress is still accepted afterwards.
# ---------------------------------------------------------------------------
def test_b52_duplicate_and_stale_ids_do_not_fault() -> None:
    """T-B5.2.2：已见过的重复或陈旧 id 不触发 FAULT。"""
    commands, machine = active_machine("arm_left", settled_samples=2)
    feed(machine, B5_SAMPLE_BASE, settled=True)  # high-water mark

    machine.on_feedback(B5_SAMPLE_BASE, settled=True)  # exact duplicate of the max id
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.state is not commands.ControlState.FAULT

    machine.on_feedback(1, settled=True)  # stale id already consumed by the STANDBY gate
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.state is not commands.ControlState.FAULT
    assert machine.active_stage is commands.Stage.ARM

    feed(machine, B5_SAMPLE_BASE + 1, settled=True)  # forward progress still accepted
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM


# ---------------------------------------------------------------------------
# T-B5.2.3 — FAULT is not self-healing: fresh, forward, valid, settled samples
# arriving after the fault must NOT drag the machine back to READY.
# ---------------------------------------------------------------------------
def test_b52_after_fault_fresh_feedback_does_not_self_heal() -> None:
    """T-B5.2.3：FAULT 不可自愈，正向新鲜反馈也不恢复。"""
    commands, machine = faulted_machine(settled_samples=2)
    assert machine.state is commands.ControlState.FAULT

    feed_count(machine, B5_SAMPLE_BASE + 100, 10, settled=True)  # far beyond the window

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B5.4.1 — FAULT refuses every motion command with MotionBlocked and leaves
# state/active_stage untouched (B5.4).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key", ALL_MOTION_COMMAND_KEYS)
def test_b54_fault_refuses_every_motion_command(key: str) -> None:
    """T-B5.4.1：FAULT 拒绝所有运动命令并保持 FAULT。"""
    cmd_types = load_command_types()
    commands, machine = faulted_machine()
    assert machine.state is commands.ControlState.FAULT

    with pytest.raises(commands.MotionBlocked):
        machine.handle(build_motion_command(cmd_types, key))

    assert machine.state is commands.ControlState.FAULT
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B5.4.2 — a release command is likewise not an edge out of FAULT; it is
# refused via the MotionBlocked family (consistent with the B1.5 ruling that
# release is MotionBlocked outside its legal ACTIVE/BLOCKED sources).
# ---------------------------------------------------------------------------
def test_b54_fault_refuses_release_command() -> None:
    """T-B5.4.2：释放命令也不是离开 FAULT 的边。"""
    completion = load_completion_types()
    commands, machine = faulted_machine()

    with pytest.raises(commands.MotionBlocked):
        machine.handle(completion.ReleaseCommand())

    assert machine.state is commands.ControlState.FAULT
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B5.3.1 — resume() must never unlock from FAULT: even with a full settle
# window carried from before the fault, it stays FAULT (never READY).
# The refusal MECHANISM is not bound (suppress), only the safety invariant.
# ---------------------------------------------------------------------------
def test_b53_resume_from_fault_does_not_unlock() -> None:
    """T-B5.3.1：即使窗口已满，resume() 也不从 FAULT 解锁。"""
    commands, machine = faulted_machine(settled_samples=2)
    assert machine.state is commands.ControlState.FAULT

    with contextlib.suppress(commands.MotionBlocked):
        machine.resume()

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B5.3.2 — the command path mirrors resume(): ResumeCommand from FAULT is a
# refusal and never unlocks to READY (B1.5/B3 unified ResumeCommand == resume()).
# ---------------------------------------------------------------------------
def test_b53_resume_command_from_fault_does_not_unlock() -> None:
    """T-B5.3.2：FAULT 下的 ResumeCommand 与 resume() 同语义。"""
    b15 = load_b15_types()
    commands, machine = faulted_machine()
    assert machine.state is commands.ControlState.FAULT

    with contextlib.suppress(commands.MotionBlocked):
        machine.handle(b15.ResumeCommand())

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B5.3.3 — the explicit recovery entry reset() targets STANDBY, never READY:
# clearing a fault must send the machine back through the STANDBY start gate.
# ---------------------------------------------------------------------------
def test_b53_reset_from_fault_targets_standby_not_ready() -> None:
    """T-B5.3.3：显式恢复入口 reset() 目标 STANDBY 而不是 READY。"""
    commands, machine = faulted_machine()
    assert machine.state is commands.ControlState.FAULT

    machine.reset()

    assert machine.state is commands.ControlState.STANDBY
    assert machine.state is not commands.ControlState.READY
    assert machine.active_stage is None


# ---------------------------------------------------------------------------
# T-B5.3.4 — after reset() the STANDBY gate is re-armed from scratch: the settle
# evidence gathered before the fault must NOT re-unlock; a fresh window and a
# new resume() are required, after which the machine is fully operational again.
# ---------------------------------------------------------------------------
def test_b53_reset_requires_fresh_window_and_full_cycle_recovers() -> None:
    """T-B5.3.4：reset 后门禁从零重新武装，完整周期可恢复。"""
    cmd_types = load_command_types()
    commands, machine = faulted_machine(settled_samples=2)  # window was full pre-FAULT
    machine.reset()
    assert machine.state is commands.ControlState.STANDBY

    machine.resume()  # no fresh feedback -> stale evidence must not unlock
    assert machine.state is commands.ControlState.STANDBY
    assert machine.state is not commands.ControlState.READY

    feed_count(machine, B5_SAMPLE_BASE + 1000, 2, settled=True)  # fresh window
    machine.resume()
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None

    machine.handle(build_motion_command(cmd_types, "base_drive"))
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.BASE_LIFT


# ===========================================================================
# Behavior B5.5 / P2 终审 finding F3（已裁定语义）:
#   FAULT 对停机是锁存态：FAULT + stop()（方法或 StopCommand）必须保持 FAULT，
#   不抛出、状态与故障原因不变；解除故障只能走显式 reset()。
#
# Seam frozen:
#   drive : ControlStateMachine.stop()                FAULT -> FAULT (no raise)
#           ControlStateMachine.handle(StopCommand())  FAULT -> FAULT (no raise)
#
# This binding discriminates the reasonable-but-WRONG implementation:
# - D004「停机优先」被过度泛化为「FAULT 也能被 stop 清成 STANDBY」，使故障原因
#   丢失、恢复路径被跳过（T-B5.5.1/T-B5.5.2）。
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# ===========================================================================


# ---------------------------------------------------------------------------
# T-B5.5.1 — FAULT + stop() 方法：保持 FAULT，不抛，故障原因不变。
# ---------------------------------------------------------------------------
def test_b55_fault_stop_method_latches_fault() -> None:
    """T-B5.5.1：FAULT 下 stop() 锁存 FAULT。"""
    commands, machine = faulted_machine()
    assert machine.state is commands.ControlState.FAULT
    reason_before = machine.last_reason
    assert isinstance(reason_before, str) and reason_before

    machine.stop()  # 必须不抛

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.STANDBY
    assert machine.active_stage is None
    assert machine.last_reason == reason_before  # 故障原因保持可见


# ---------------------------------------------------------------------------
# T-B5.5.2 — FAULT + handle(StopCommand())：命令路径与 stop() 方法同语义（锁存）。
# ---------------------------------------------------------------------------
def test_b55_fault_stop_command_latches_fault() -> None:
    """T-B5.5.2：FAULT 下的 StopCommand 与 stop() 同语义锁存。"""
    cmd_types = load_command_types()
    commands, machine = faulted_machine()
    assert machine.state is commands.ControlState.FAULT
    reason_before = machine.last_reason
    assert isinstance(reason_before, str) and reason_before

    machine.handle(cmd_types.StopCommand())  # 必须不抛

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.STANDBY
    assert machine.active_stage is None
    assert machine.last_reason == reason_before


# ===========================================================================
# Behavior B7 (实施包 P2「通用命令与状态抽象」, design §2「失败显式报错」):
#   纯数据命令对象在构造期自校验；非法 / 矛盾 / 类型错误的输入必须显式抛错，
#   禁止静默 clamp、静默忽略，或把非法值带入内存。
#
# Protected acceptance:
# - B7.1 夹爪：GripperCommand(SET) 的 position 必须有限且落在 [0, 100]（含端点）；
#   <0 / >100 / NaN / ±inf / 缺省(None) / 错误类型 一律在构造期拒绝，
#   绝不静默 clamp 到边界。
# - B7.2 底盘：BaseCommand 的速度分量 (x, y, turn) 必须有限；seconds 必须有限
#   且 >= 0（0.0 合法：D014 显式零速即底盘停止）；负 seconds、非有限值、
#   错误类型字段一律拒绝。
#
# 组合语义裁定（本阶段冻结，报告给主 Agent）：
#   GripperCommand.action == OPEN/CLOSE 时 position 必须为 None；显式携带
#   position 属「矛盾请求」，按 design §2「禁止静默截断/忽略」与 A5 冲突原则
#   显式拒绝（而非静默忽略）。action == SET 时 position 必须存在且合法。
#
# 校验位置裁定（本阶段冻结，报告给主 Agent，理由如下）：
#   放在 frozen dataclass 的 __post_init__，即**构造期**自校验。
#   理由：(1) 纯数据对象自一致性——非法命令根本无法存在于内存，任何下游
#   (P4-B controller / SDK api / dry-run 报告) 都不可能观察到半合法命令，
#   最早边界 fail-fast；(2) 与项目既有约定一致（config_alohamini.py 的
#   dataclass __post_init__ -> ValueError）；(3) 与状态机的「状态合法性」职责
#   分离：handle() 只管状态迁移，数据合法性在数据对象构造时一次性裁决，
#   可在无状态机前提下直接测试；(4) frozen 对象不可变，__post_init__ 在对象
#   可用前运行，不引入可变窗口。
#
# 异常契约（本阶段冻结）：
#   - 取值域违规（越界 / 非有限 / 负 seconds / SET 缺 position / OPEN-CLOSE 带
#     position）-> ValueError（或其子类）；
#   - 错误类型字段 -> TypeError 或 ValueError（二者皆是显式拒绝，Python 惯例
#     允许对错误类型用 TypeError）。
#
# This binding deliberately discriminates reasonable-but-WRONG implementations:
# - 静默 clamp：position = max(0, min(100, position))（T-B7.1.2，pytest.raises
#   直接失败）；
# - 只在 handle() 校验、构造期放行（下面所有用例都直接构造命令、不经状态机，
#   构造期不抛错即失败）；
# - 只用 0 <= position <= 100 判断而漏掉 NaN（NaN 比较恒 False 会被放行，
#   T-B7.1.3 捕获）；
# - 静默忽略 OPEN/CLOSE 上多余的 position，或把 SET 的 None 当默认值
#   （T-B7.1.4/T-B7.1.5）；
# - 负 seconds 被静默接受或 clamp 到 0（T-B7.2.3）；
# - 错误类型字段被原样存入（T-B7.1.7/T-B7.2.4）。
#
# 期望值为冻结字面量；只观察公开构造契约，不读取被测实现的内部状态。
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# ===========================================================================

VALID_GRIPPER_SET_POSITIONS = (0.0, 50.0, 100.0)
INVALID_GRIPPER_SET_POSITIONS = (-0.001, -1.0, 100.001, 1000.0)
NON_FINITE_VALUES = (float("nan"), float("inf"), float("-inf"))


# ---------------------------------------------------------------------------
# T-B7.1.1 — SET position boundary: endpoints 0 and 100 are inclusive and legal.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("position", VALID_GRIPPER_SET_POSITIONS)
def test_b71_gripper_set_position_boundaries_are_accepted(position: float) -> None:
    """T-B7.1.1：夹爪 SET 的位置边界值被接受。"""
    ct = load_command_types()

    command = ct.GripperCommand(side=ct.Side.LEFT, action=ct.GripperAction.SET, position=position)

    assert command.position == position


# ---------------------------------------------------------------------------
# T-B7.1.2 — SET position outside [0, 100] is rejected, never silently clamped.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("position", INVALID_GRIPPER_SET_POSITIONS)
def test_b71_gripper_set_position_out_of_range_is_rejected(position: float) -> None:
    """T-B7.1.2：夹爪位置超出 0 到 100 被拒绝而不是静默截断。"""
    ct = load_command_types()

    with pytest.raises(ValueError):
        ct.GripperCommand(side=ct.Side.LEFT, action=ct.GripperAction.SET, position=position)


# ---------------------------------------------------------------------------
# T-B7.1.3 — non-finite position (NaN / ±inf) is rejected explicitly, so a plain
# range comparison (which lets NaN through) is not enough.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("position", NON_FINITE_VALUES)
def test_b71_gripper_set_position_non_finite_is_rejected(position: float) -> None:
    """T-B7.1.3：非有限的夹爪位置被显式拒绝。"""
    ct = load_command_types()

    with pytest.raises(ValueError):
        ct.GripperCommand(side=ct.Side.LEFT, action=ct.GripperAction.SET, position=position)


# ---------------------------------------------------------------------------
# T-B7.1.4 — SET without a position (None) is missing required data and refused.
# ---------------------------------------------------------------------------
def test_b71_gripper_set_without_position_is_rejected() -> None:
    """T-B7.1.4：SET 缺少位置（None）属于缺必需数据，被拒绝。"""
    ct = load_command_types()

    with pytest.raises(ValueError):
        ct.GripperCommand(side=ct.Side.RIGHT, action=ct.GripperAction.SET, position=None)


# ---------------------------------------------------------------------------
# T-B7.1.5 — combination semantics: OPEN/CLOSE must NOT carry a position; a
# contradictory explicit position is refused rather than silently ignored.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("action_name", ["OPEN", "CLOSE"])
def test_b71_gripper_open_close_with_position_is_rejected(action_name: str) -> None:
    """T-B7.1.5：OPEN/CLOSE 携带位置被拒绝。"""
    ct = load_command_types()

    with pytest.raises(ValueError):
        ct.GripperCommand(
            side=ct.Side.LEFT,
            action=getattr(ct.GripperAction, action_name),
            position=50.0,
        )


# ---------------------------------------------------------------------------
# T-B7.1.6 — the legitimate OPEN/CLOSE form (position=None) still constructs, so
# the validator must not reject the whole action class.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("action_name", ["OPEN", "CLOSE"])
def test_b71_gripper_open_close_without_position_is_accepted(action_name: str) -> None:
    """T-B7.1.6：合法的 OPEN/CLOSE（无位置）仍可构造。"""
    ct = load_command_types()
    action = getattr(ct.GripperAction, action_name)

    command = ct.GripperCommand(side=ct.Side.LEFT, action=action, position=None)

    assert command.position is None
    assert command.action is action


# ---------------------------------------------------------------------------
# T-B7.1.7 — a wrong-typed position field is rejected instead of being stored.
# ---------------------------------------------------------------------------
def test_b71_gripper_position_wrong_type_is_rejected() -> None:
    """T-B7.1.7：位置字段类型错误被拒绝而不被存储。"""
    ct = load_command_types()

    with pytest.raises((TypeError, ValueError)):
        ct.GripperCommand(side=ct.Side.LEFT, action=ct.GripperAction.SET, position="50")


# ---------------------------------------------------------------------------
# T-B7.2.1 — legal base commands construct, including the D014 explicit-zero
# (seconds == 0.0) form.  Guards against a validator that rejects the legitimate
# stop or every command.
# ---------------------------------------------------------------------------
def test_b72_valid_base_commands_are_accepted() -> None:
    """T-B7.2.1：合法底座命令可构造，含 D014 的显式零速度。"""
    ct = load_command_types()

    stop = ct.BaseCommand(x=0.0, y=0.0, turn=0.0, seconds=0.0)
    drive = ct.BaseCommand(x=0.1, y=-0.2, turn=0.3, seconds=1.5)

    assert (stop.x, stop.y, stop.turn, stop.seconds) == (0.0, 0.0, 0.0, 0.0)
    assert (drive.x, drive.y, drive.turn, drive.seconds) == (0.1, -0.2, 0.3, 1.5)


# ---------------------------------------------------------------------------
# T-B7.2.2 — every velocity component must be finite: NaN/inf is refused on any
# of x / y / turn.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("field_name", ["x", "y", "turn"])
@pytest.mark.parametrize("value", NON_FINITE_VALUES)
def test_b72_non_finite_velocity_is_rejected(field_name: str, value: float) -> None:
    """T-B7.2.2：底座速度分量必须有限，NaN 或 inf 被拒绝。"""
    ct = load_command_types()
    kwargs = {"x": 0.0, "y": 0.0, "turn": 0.0, "seconds": 1.0}
    kwargs[field_name] = value

    with pytest.raises(ValueError):
        ct.BaseCommand(**kwargs)


# ---------------------------------------------------------------------------
# T-B7.2.3 — seconds must be finite and non-negative; a negative or non-finite
# duration is refused, never clamped to 0.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("seconds", [-0.001, -1.0, *NON_FINITE_VALUES])
def test_b72_bad_seconds_is_rejected(seconds: float) -> None:
    """T-B7.2.3：seconds 必须有限且非负，否则被拒绝。"""
    ct = load_command_types()

    with pytest.raises(ValueError):
        ct.BaseCommand(x=0.1, y=0.0, turn=0.0, seconds=seconds)


# ---------------------------------------------------------------------------
# T-B7.2.4 — a wrong-typed numeric field is rejected instead of being stored.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("field_name", ["x", "seconds"])
def test_b72_wrong_typed_field_is_rejected(field_name: str) -> None:
    """T-B7.2.4：数值字段类型错误被拒绝而不被存储。"""
    ct = load_command_types()
    kwargs = {"x": 0.0, "y": 0.0, "turn": 0.0, "seconds": 1.0}
    kwargs[field_name] = "fast"

    with pytest.raises((TypeError, ValueError)):
        ct.BaseCommand(**kwargs)


# ===========================================================================
# Behavior B7-ext (B7 校验补全，主 Agent 裁定):
#   把 B7 的「纯数据命令构造期自校验」补齐到剩余命令对象——ArmCommand 的平移
#   分量、LiftCommand 的行程，以及 BaseCommand 的零时长矛盾请求。
#
# Protected acceptance:
# - B7.3 机械臂：ArmCommand(side, dx, dy, dz) 的 dx/dy/dz 必须有限（NaN/±inf
#   构造期拒绝），错误类型字段显式拒绝；合法（含 0.0 / 负值）仍可构造。
# - B7.4 升降：LiftCommand(delta_mm) 的 delta_mm 必须有限，错误类型显式拒绝；
#   合法（含负值）仍可构造。
# - B7.5 底盘零时长矛盾：BaseCommand 在 seconds == 0 时任一速度分量 (x/y/turn)
#   非零即属「零时长还要运动」的矛盾请求 -> 构造期拒绝；seconds == 0 且全零速度
#   （D014 显式停车）以及 seconds > 0 且非零速度仍合法。
#
# 异常契约沿用 B7：取值域违规 -> ValueError（或其子类）；错误类型字段 ->
# TypeError 或 ValueError（二者皆是显式拒绝）。校验位置沿用 B7 裁定：构造期
# frozen dataclass.__post_init__，不静默 clamp、不静默忽略。
#
# This binding deliberately discriminates reasonable-but-WRONG implementations:
# - 漏掉 ArmCommand / LiftCommand 的有限性校验：NaN/inf 被原样接受
#   （T-B7.3.2 / T-B7.4.2，pytest.raises 直接失败，即当前实现）；
# - 只做符号 / 量级判断而漏掉 NaN（NaN 的比较恒 False）:T-B7.3.2 / T-B7.4.2；
# - 错误类型字段被原样存入：T-B7.3.3 / T-B7.4.3；
# - 零时长 + 非零速度的矛盾命令被静默接受（当前实现）：T-B7.5.1；
# - 过度校验：拒掉 seconds == 0 的 D014 显式停车、或嫌所有含零分量的命令
#   （T-B7.5.2 / T-B7.5.3 正例护栏捕获）。
#
# 期望值为冻结字面量；只观察公开构造契约，不读取被测实现的内部状态。
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# ===========================================================================

# Deliberate alias of the B7 NaN/inf vocabulary: same three non-finite values.
NON_FINITE_DELTAS = NON_FINITE_VALUES


# ---------------------------------------------------------------------------
# T-B7.3.1 — legal arm deltas still construct (positive guardrail against
# over-validation): zero, positive and negative finite deltas are all accepted.
# ---------------------------------------------------------------------------
def test_b73_valid_arm_commands_are_accepted() -> None:
    """T-B7.3.1：合法的机械臂增量命令可构造。"""
    ct = load_command_types()

    rest = ct.ArmCommand(side=ct.Side.LEFT, dx=0.0, dy=0.0, dz=0.0)
    move = ct.ArmCommand(side=ct.Side.RIGHT, dx=-0.01, dy=0.02, dz=-0.03)

    assert (rest.dx, rest.dy, rest.dz) == (0.0, 0.0, 0.0)
    assert (move.dx, move.dy, move.dz) == (-0.01, 0.02, -0.03)


# ---------------------------------------------------------------------------
# T-B7.3.2 — every arm delta component must be finite: NaN/inf is refused on any
# of dx / dy / dz (a plain magnitude/sign check would let NaN through).
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("field_name", ["dx", "dy", "dz"])
@pytest.mark.parametrize("value", NON_FINITE_DELTAS)
def test_b73_non_finite_arm_delta_is_rejected(field_name: str, value: float) -> None:
    """T-B7.3.2：机械臂增量分量必须有限，NaN 或 inf 被拒绝。"""
    ct = load_command_types()
    kwargs = {"side": ct.Side.LEFT, "dx": 0.0, "dy": 0.0, "dz": 0.0}
    kwargs[field_name] = value

    with pytest.raises(ValueError):
        ct.ArmCommand(**kwargs)


# ---------------------------------------------------------------------------
# T-B7.3.3 — a wrong-typed arm delta field is rejected instead of being stored.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("field_name", ["dx", "dy", "dz"])
def test_b73_wrong_typed_arm_delta_is_rejected(field_name: str) -> None:
    """T-B7.3.3：机械臂增量字段类型错误被拒绝。"""
    ct = load_command_types()
    kwargs = {"side": ct.Side.LEFT, "dx": 0.0, "dy": 0.0, "dz": 0.0}
    kwargs[field_name] = "0.01"

    with pytest.raises((TypeError, ValueError)):
        ct.ArmCommand(**kwargs)


# ---------------------------------------------------------------------------
# T-B7.4.1 — legal lift deltas still construct (including negative travel).
# ---------------------------------------------------------------------------
def test_b74_valid_lift_commands_are_accepted() -> None:
    """T-B7.4.1：合法的升降增量（含负行程）可构造。"""
    ct = load_command_types()

    up = ct.LiftCommand(delta_mm=20.0)
    down = ct.LiftCommand(delta_mm=-20.0)

    assert up.delta_mm == 20.0
    assert down.delta_mm == -20.0


# ---------------------------------------------------------------------------
# T-B7.4.2 — delta_mm must be finite; NaN / ±inf is refused at construction.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("value", NON_FINITE_DELTAS)
def test_b74_non_finite_lift_delta_is_rejected(value: float) -> None:
    """T-B7.4.2：升降 delta_mm 必须有限，NaN 或 inf 被拒绝。"""
    ct = load_command_types()

    with pytest.raises(ValueError):
        ct.LiftCommand(delta_mm=value)


# ---------------------------------------------------------------------------
# T-B7.4.3 — a wrong-typed delta_mm is rejected instead of being stored.
# ---------------------------------------------------------------------------
def test_b74_wrong_typed_lift_delta_is_rejected() -> None:
    """T-B7.4.3：升降 delta_mm 类型错误被拒绝。"""
    ct = load_command_types()

    with pytest.raises((TypeError, ValueError)):
        ct.LiftCommand(delta_mm="20")


# ---------------------------------------------------------------------------
# T-B7.5.1 — a zero-duration command with a non-zero velocity is a contradictory
# request: seconds == 0 means there is no time to move, so any non-zero x/y/turn
# is refused at construction.  Covers every velocity component independently.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("field_name", ["x", "y", "turn"])
def test_b75_zero_seconds_with_nonzero_velocity_is_rejected(field_name: str) -> None:
    """T-B7.5.1：零时长配非零速度属矛盾命令，被拒绝。"""
    ct = load_command_types()
    kwargs = {"x": 0.0, "y": 0.0, "turn": 0.0, "seconds": 0.0}
    kwargs[field_name] = 0.1

    with pytest.raises(ValueError):
        ct.BaseCommand(**kwargs)


# ---------------------------------------------------------------------------
# T-B7.5.2 — positive guardrail: the D014 explicit stop (seconds == 0 with an
# all-zero velocity vector) still constructs; the new contradiction guard must
# not reject the legitimate base stop.
# ---------------------------------------------------------------------------
def test_b75_zero_seconds_with_zero_velocity_is_accepted() -> None:
    """T-B7.5.2：D014 显式停止（秒为 0 且速度 0）仍可构造。"""
    ct = load_command_types()

    stop = ct.BaseCommand(x=0.0, y=0.0, turn=0.0, seconds=0.0)

    assert (stop.x, stop.y, stop.turn, stop.seconds) == (0.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# T-B7.5.3 — positive guardrail: a timed non-zero move still constructs, so the
# validator must not reject every command carrying a non-zero velocity.
# ---------------------------------------------------------------------------
def test_b75_nonzero_velocity_with_positive_seconds_is_accepted() -> None:
    """T-B7.5.3：定时非零运动仍可构造。"""
    ct = load_command_types()

    drive = ct.BaseCommand(x=0.1, y=-0.2, turn=0.3, seconds=1.5)

    assert (drive.x, drive.y, drive.turn, drive.seconds) == (0.1, -0.2, 0.3, 1.5)


# ===========================================================================
# Behavior B5.6 / P4-B cycle 6-fix —— 公开 block() API + FAULT 锁存保护
#   (appended; 既有 216 例未动)
#
# 缺陷事实 (tester 实测): 外部 BLOCKED 入口此前直写 ControlStateMachine 私有属性,
# 在 FAULT 状态下会把 FAULT 降级为 BLOCKED; 随后 ReleaseCommand + 停稳窗口直达
# READY, 绕过 D016「唯一恢复入口 reset()」。本 binding 冻结公开接缝:
#
#   接缝: ControlStateMachine.block(reason: str) -> None  (公开、非抛出, D015 生命周期)
#   语义: 任意非 FAULT 源态 STANDBY/READY/ACTIVE/SETTLING/BLOCKED -> BLOCKED，
#         active_stage 清空、last_reason == reason、统一簿记 (停稳窗口与已武装的
#         释放恢复一起复位); FAULT 为锁存态: block() 是 no-op，保持 FAULT 且
#         last_reason 不变 (保留故障原因，与既有 stop()-from-FAULT 语义一致)。
#
# 已被本 binding 有意区分的 reasonable-but-WRONG 实现 (每个都会被至少一例捕获):
# - block() 不存在 / 非公开 -> 参数化用例 AttributeError (RED);
# - block() 从 FAULT 也写 BLOCKED (本 cycle 缺陷本体) -> T-B5.6.2 状态断言失败，
#   且 T-B5.6.3 中 ReleaseCommand 不再抛 MotionBlocked、停稳后直达 READY;
# - block() 不清 active_stage -> T-B5.6.1 阶段断言失败;
# - block() 不写 reason / 写错 reason -> T-B5.6.1 reason 断言失败;
# - block() 不复位已武装的释放恢复窗口 -> T-B5.6.4 停稳反馈会自行 READY;
# - block() 破坏既有恢复路径 (Release+停稳窗口无法回到 READY) -> T-B5.6.5。
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# ===========================================================================

# 本阶段自用的新鲜反馈 id 段，与 B1.1(1..N)、B1.4(10_000+)、B2(20_000+)、
# B3(30_000+)、B5(40_000+) 区分开。
B6_SAMPLE_BASE = 50_000

NON_FAULT_BLOCKABLE_STATES = (
    "STANDBY",
    "READY",
    "ACTIVE_ARM",
    "ACTIVE_BASE_LIFT",
    "SETTLING",
    "BLOCKED",
)

# 每个 block() 源态 key -> 预期的监督状态名 (ACTIVE 两个阶段共用同一状态名).
BLOCKABLE_STATE_NAMES = {
    "STANDBY": "STANDBY",
    "READY": "READY",
    "ACTIVE_ARM": "ACTIVE",
    "ACTIVE_BASE_LIFT": "ACTIVE",
    "SETTLING": "SETTLING",
    "BLOCKED": "BLOCKED",
}


def blockable_machine(state_key: str, settled_samples: int = 3) -> tuple[SimpleNamespace, Any]:
    """用既有已验证边界构造六个监督状态之一（仅用于 block() 源态矩阵）。"""
    if state_key == "STANDBY":
        commands, machine = make_machine(settled_samples=settled_samples)
    elif state_key == "READY":
        commands, machine = ready_machine(settled_samples=settled_samples)
    elif state_key == "ACTIVE_ARM":
        commands, machine = active_machine("arm_left", settled_samples=settled_samples)
    elif state_key == "ACTIVE_BASE_LIFT":
        commands, machine = active_machine("base_drive", settled_samples=settled_samples)
    elif state_key == "SETTLING":
        commands, machine = released_machine("arm_left", settled_samples=settled_samples)
    elif state_key == "BLOCKED":
        commands, machine = blocked_machine("arm_left", settled_samples=settled_samples)
    elif state_key == "FAULT":
        return faulted_machine(settled_samples=settled_samples)
    else:
        raise AssertionError(f"unsupported blockable state key: {state_key}")

    assert machine.state.name == BLOCKABLE_STATE_NAMES[state_key], (state_key, machine.state)
    return commands, machine


# ---------------------------------------------------------------------------
# T-B5.6.1 — block() 从每个非 FAULT 源态 (STANDBY/READY/ACTIVE(ARM|BASE_LIFT)/
# SETTLING/BLOCKED) 进入 BLOCKED: 非抛出、清空 active_stage、记录 reason。
# block() 缺失 -> AttributeError; 不清 stage / 不写 reason -> 对应断言失败。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state_key", NON_FAULT_BLOCKABLE_STATES)
def test_b56_block_from_every_non_fault_state_enters_blocked(state_key: str) -> None:
    """T-B5.6.1：block() 从每个非 FAULT 源态都进入 BLOCKED。"""
    commands, machine = blockable_machine(state_key)
    reason = f"external block from {state_key}"

    machine.block(reason)  # 必须非抛出 (D015 生命周期语义)

    assert machine.state is commands.ControlState.BLOCKED, (state_key, machine.state)
    assert machine.state is not commands.ControlState.FAULT
    assert machine.active_stage is None
    assert machine.last_reason == reason


# ---------------------------------------------------------------------------
# T-B5.6.2 — block() 从 FAULT 是锁存 no-op: 保持 FAULT、故障原因不变。
# 「FAULT 也写 BLOCKED」的缺陷实现 -> 状态降级断言失败。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("stage_key", ["arm_left", "base_drive"])
def test_b56_block_from_fault_keeps_fault_and_reason(stage_key: str) -> None:
    """T-B5.6.2：FAULT 下的 block() 是锁存 no-op，状态与原因不变。"""
    commands, machine = faulted_machine(stage_key)
    assert machine.state is commands.ControlState.FAULT
    reason_before = machine.last_reason
    assert isinstance(reason_before, str) and reason_before

    machine.block("external block attempt")  # 必须非抛出

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.BLOCKED
    assert machine.active_stage is None
    assert machine.last_reason == reason_before  # 保留故障原因


# --------------------------------------------------------------------------
# T-B5.6.3 — block() 不得为 FAULT 打开任何恢复捷径: 其后 ReleaseCommand 仍被
# MotionBlocked 拒绝、停稳反馈也不得把机器拉回 READY (D016 唯一 reset() 恢复)。
# 降级为 BLOCKED 的实现会让 ReleaseCommand 武装恢复窗口并最终 READY -> 两处断言失败。
# ---------------------------------------------------------------------------
def test_b56_block_from_fault_opens_no_recovery_shortcut() -> None:
    """T-B5.6.3：block() 不为 FAULT 打开任何恢复捷径。"""
    commands, machine = faulted_machine(settled_samples=2)
    completion = load_completion_types()
    assert machine.state is commands.ControlState.FAULT

    machine.block("external block attempt")
    assert machine.state is commands.ControlState.FAULT

    with pytest.raises(commands.MotionBlocked):
        machine.handle(completion.ReleaseCommand())

    feed_count(machine, B6_SAMPLE_BASE, 5, settled=True)

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.READY
    assert machine.state is not commands.ControlState.BLOCKED


# ---------------------------------------------------------------------------
# T-B5.6.4 — block() 统一簿记: 在「已武装释放恢复窗口」的 BLOCKED 上调用后,
# 停稳反馈不得自行把机器拉回 READY, 必须重新走一次显式释放。
# 不复位 _blocked_released 的实现 -> 停稳窗口凑满即 READY -> 断言失败。
# ---------------------------------------------------------------------------
def test_b56_block_disarms_armed_release_recovery() -> None:
    """T-B5.6.4：block() 统一簿记，停稳反馈不得自行把机器拉回 READY。"""
    commands, machine = blocked_machine("arm_left", settled_samples=3)
    completion = load_completion_types()
    assert machine.state is commands.ControlState.BLOCKED

    machine.handle(completion.ReleaseCommand())  # 武装恢复窗口
    assert machine.state is commands.ControlState.BLOCKED
    feed_count(machine, B6_SAMPLE_BASE, 2, settled=True)  # 窗口未满
    assert machine.state is commands.ControlState.BLOCKED

    machine.block("external block resets recovery evidence")
    assert machine.state is commands.ControlState.BLOCKED
    assert machine.last_reason == "external block resets recovery evidence"

    feed_count(machine, B6_SAMPLE_BASE + 10, 6, settled=True)  # 远超窗口

    assert machine.state is commands.ControlState.BLOCKED, "block() 未复位释放窗口证据"
    assert machine.state is not commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-B5.6.5 — 正向前提: block() 后仍可走既有恢复路径 (ReleaseCommand + 停稳窗口
# -> READY), 之后重新接受运动命令。若 block() 把机器置于不可恢复状态 -> 断言失败。
# ---------------------------------------------------------------------------
def test_b56_block_preserves_existing_release_settle_recovery() -> None:
    """T-B5.6.5：block() 之后仍可走既有的释放加停稳窗口恢复路径。"""
    commands, machine = active_machine("arm_left", settled_samples=2)
    completion = load_completion_types()
    cmd_types = load_command_types()
    assert machine.state is commands.ControlState.ACTIVE

    machine.block("external block")
    assert machine.state is commands.ControlState.BLOCKED
    assert machine.active_stage is None

    machine.handle(completion.ReleaseCommand())
    assert machine.state is commands.ControlState.BLOCKED

    feed_count(machine, B6_SAMPLE_BASE, 2, settled=True)
    assert machine.state is commands.ControlState.READY
    assert machine.active_stage is None

    machine.handle(build_motion_command(cmd_types, "arm_left"))
    assert machine.state is commands.ControlState.ACTIVE
    assert machine.active_stage is commands.Stage.ARM


# ===========================================================================
# P4-B cycle 7 —— T-C12 反馈倒退 FAULT 源态扩展 (appended; 既有 227 例未动)
#
# 裁定 (design §6 规则 1/2 与 D016 扩展): 反馈倒退 = 一个**未见过**且**低于已接受
# 高水位**的 sample_id。该完整性违例不再只限于 ACTIVE 源态: 在 READY/SETTLING/
# BLOCKED 下同样锁存 FAULT; STANDBY 下没有运动在推进, 倒退仅忽略 (不 FAULT);
# 已在 FAULT 下保持 FAULT (锁存, 不自愈)。既有 B5.2 只绑定 ACTIVE 源态, 本 cycle
# 补齐其余源态并冻结「未见过」判别条件。
#
# 本 binding 有意区分以下 reasonable-but-WRONG 实现 (每个都会被至少一例捕获):
# - 仍只在 ACTIVE 下检查倒退 -> READY/SETTLING/BLOCKED 源态用例状态断言失败;
# - 把「任意低于高水位的 id」当倒退 (漏 '未见过' 条件) -> 陈旧 id 用例状态断言失败;
# - 倒退检查扩到 STANDBY (无运动也 FAULT) -> STANDBY 用例状态断言失败;
# - 倒退只写 last_reason 不改状态 / 不清 active_stage -> 状态与阶段断言失败;
# - FAULT 可被后续正向停稳反馈自愈 -> no-self-heal 用例状态断言失败。
#
# Pure logic only: no sleep, no clock, no I/O, no randomness, no hardware.
# Expected values are frozen literals; nothing is read back from the
# implementation under test beyond the public read seam.
# ===========================================================================

# cycle 7 自用的新鲜反馈 id 段, 与 B1(1..N)、B1.4(10_000+)、B2(20_000+)、
# B3(30_000+)、B5(40_000+)、B6(50_000+) 区分开。
C7_SAMPLE_BASE = 60_000


def regress_after_high_water(machine: Any, *, base: int = C7_SAMPLE_BASE) -> None:
    """抬高高水位, 再注入一个未见过且低于高水位的 id (倒退)。"""
    machine.on_feedback(base, settled=True)  # 新高水位
    machine.on_feedback(base - 1, settled=True)  # 未见过, 低于高水位


# ---------------------------------------------------------------------------
# T-C12.1 — 未见过且低于高水位的 sample_id 在 READY/SETTLING/BLOCKED 源态下也
# 锁存 FAULT, 清空 active_stage 并记录非空原因 (D016 源态扩展)。
# 只在 ACTIVE 检查的上一版实现 -> 三个源态用例均状态断言失败。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state_name", ["READY", "SETTLING", "BLOCKED"])
def test_t_c12_sample_id_regression_from_non_active_state_faults(state_name: str) -> None:
    """T-C12.1：非 ACTIVE 源态下的 sample_id 倒退同样锁存 FAULT。"""
    commands, machine = machine_in_any_state(state_name, settled_samples=2)
    assert machine.state.name == state_name

    regress_after_high_water(machine)

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.ACTIVE
    assert machine.active_stage is None  # 无残留被服务的阶段
    assert isinstance(machine.last_reason, str) and machine.last_reason


# ---------------------------------------------------------------------------
# T-C12.2 — 判别条件「未见过」: 已见过的陈旧 id (数值低于高水位) 在非 ACTIVE
# 源态下只被忽略, 绝不 FAULT。漏掉 '未见过' 条件的实现 -> 状态断言失败。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state_name", ["READY", "SETTLING", "BLOCKED"])
def test_t_c12_already_seen_lower_ids_do_not_fault_from_non_active(state_name: str) -> None:
    """T-C12.2：已见过的低值 id 在非 ACTIVE 源态下被忽略，绝不 FAULT。"""
    commands, machine = machine_in_any_state(state_name, settled_samples=2)
    # id 1 已被 STANDBY gate 消费 (已见过), 数值上低于高水位, 但不构成倒退。
    machine.on_feedback(1, settled=True)

    assert machine.state.name == state_name
    assert machine.state is not commands.ControlState.FAULT


# ---------------------------------------------------------------------------
# T-C12.3 — STANDBY 下没有运动在推进: 倒退仅忽略, 不 FAULT, 且不破坏 Start gate
# (后续正向停稳样本仍可 resume -> READY)。把倒退检查扩到 STANDBY 的实现 -> RED。
# ---------------------------------------------------------------------------
def test_t_c12_standby_regression_is_ignored_and_gate_still_works() -> None:
    """T-C12.3：STANDBY 下不计倒退，停稳门禁仍正常工作。"""
    commands, machine = make_machine(settled_samples=2)

    machine.on_feedback(C7_SAMPLE_BASE, settled=True)  # 高水位, 窗口 1/2
    machine.on_feedback(C7_SAMPLE_BASE - 1, settled=True)  # 倒退: STANDBY 下忽略

    assert machine.state is commands.ControlState.STANDBY
    assert machine.state is not commands.ControlState.FAULT

    machine.on_feedback(C7_SAMPLE_BASE + 1, settled=True)  # 正向停稳仍在计数
    machine.resume()
    assert machine.state is commands.ControlState.READY


# ---------------------------------------------------------------------------
# T-C12.4 — FAULT 是锁存态: 已 FAULT 时再来一个倒退样本保持 FAULT 且原因不变
# (on_feedback 在 FAULT 下为早退, 不重新评估)。FAULT 自愈/重写原因的实现 -> RED。
# ---------------------------------------------------------------------------
def test_t_c12_regression_while_faulted_keeps_fault_and_reason() -> None:
    """T-C12.4：已 FAULT 时再来倒退样本保持 FAULT 且原因不变。"""
    commands, machine = faulted_machine("arm_left", settled_samples=2)
    reason_before = machine.last_reason
    assert machine.state is commands.ControlState.FAULT
    assert isinstance(reason_before, str) and reason_before

    machine.on_feedback(B5_SAMPLE_BASE - 20, settled=True)  # 又一个未见过且低于高水位的 id

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.READY
    assert machine.last_reason == reason_before


# ---------------------------------------------------------------------------
# T-C12.5 — 非 ACTIVE 源态锁存的 FAULT 同样不可自愈: 远超过停稳窗口的正向有效停稳
# 样本也不得把机器拉回 READY (与 B5.2.3 的 ACTIVE 源态语义一致)。
# 允许自愈的实现 -> 状态断言失败。
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state_name", ["READY", "SETTLING", "BLOCKED"])
def test_t_c12_non_active_fault_is_latched_and_not_self_healing(state_name: str) -> None:
    """T-C12.5：非 ACTIVE 锁存的 FAULT 同样不可自愈。"""
    commands, machine = machine_in_any_state(state_name, settled_samples=2)
    regress_after_high_water(machine)
    assert machine.state is commands.ControlState.FAULT

    feed_count(machine, C7_SAMPLE_BASE + 100, 5, settled=True)  # 远超窗口正向样本

    assert machine.state is commands.ControlState.FAULT
    assert machine.state is not commands.ControlState.READY
    assert machine.active_stage is None


if __name__ == "__main__":
    pytest.main([__file__])
