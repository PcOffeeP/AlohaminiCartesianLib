#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""控制层公开 API（``alohamini_cartesian.control``）的行为测试。

搬运自 lerobot 仓库 ``tests/robots/test_alohamini_control_api.py``：原文件是围绕
``AlohaMiniClient`` 的 P5 用例集，此处按「观测形态」而非真实 client 驱动同一个
控制层，本文件因此不再 import lerobot，改用本地假 client 复现生产观测形态。

用例分四组：

* ``test_char_3_*`` / ``test_char_4_*`` / ``test_char_5_*`` —— 冻结的现状记录：
  ``GripperCommand`` 只作 ARM 互锁（不发不求解）、``ControlStateMachine.reset()``
  清空 ``last_reason``、10 个 ``ControlError`` 错误码词表快照。
* ``test_red_b16_*`` / ``test_red_b5_*`` / ``test_red_connect_*`` —— 包导出契约、
  ``enable_motion`` 门禁、connect 骨架（``client_factory`` 注入、句柄接线、
  幂等 ``disconnect``、上下文管理器、``stop``）。
* ``test_red_b4_*`` / ``test_red_b2_*`` / ``test_red_b3_*`` / ``test_red_b6_*`` /
  ``test_red_b17_*`` / ``test_red_b9_*`` / ``test_red_b10_*`` / ``test_red_b7_*`` /
  ``test_red_b8_*`` / ``test_red_b11_*`` / ``test_red_b12_*`` / ``test_red_b14_*`` /
  ``test_red_b18_*`` / ``test_red_b19_*`` / ``test_red_c3_*`` / ``test_red_c4_*`` /
  ``test_red_c6_*`` / ``test_red_c7_*`` —— 运动语义：connect 不 home、阻塞式
  ``move_to``、单臂隔离、整路径预检、互锁、keepalive、base 归零、故障原因上浮、
  生产观测形态下的 settle/valid 派生、适配器监督。

实现契约变化：本库 ``control/api.py`` 的 ``_default_client_factory`` 不再 import
lerobot，未注入 ``client_factory`` 时 ``connect`` 抛
``alohamini_cartesian.control.RuntimeUnavailable``。原文件的 12 处 ``connect`` 调用
全部显式注入 ``client_factory``，因此该契约原本无覆盖；文件末尾新增
``test_default_client_factory_without_runtime_raises_runtime_unavailable`` 专项钉住
本次迁移唯一的行为变更（fail-closed 且不引入 lerobot）。

原先断言 ``AlohaMiniClient`` 自身行为（``send_action`` 线上键集合、
``get_observation`` 侧信道原子提交）的两个用例留在 ``test_control_api_lerobot_parity.py``，
因为只有真实 client 才能作 oracle。

必须用真实 ``AlohaMiniClient`` 才能成立的差分对照用例在
``test_control_api_lerobot_parity.py``（需要 runtime extra）。
"""

from __future__ import annotations

import importlib
import json
import sys
from typing import Any

import numpy as np
import pytest

from alohamini_cartesian.control.commands import (
    ControlState,
    ControlStateMachine,
    GripperAction,
    GripperCommand,
    Side,
    Stage,
)
from alohamini_cartesian.control.controller import CartesianController, ControlError
from alohamini_cartesian.kinematics.model_specs import arm_state_keys_for_robot_model

_CONTROL_PACKAGE = "alohamini_cartesian.control"

# ``lerobot.utils.constants`` 里两个键名常量：本库不依赖 lerobot，故本地定义。
# 取值与上游一致（``ACTION = "action"``、``OBS_STATE = "observation.state"``）。
ACTION = "action"
OBS_STATE = "observation.state"


# ===========================================================================
# Harness: 本地假 client（无 zmq、无 socket、无线程），只复现被测的对外行为。
# ===========================================================================


class _FakeAlohaClient:
    """本地假 client：产出生产形态的观测，并按顺序记录每次 ``send_action``。

    替代真实 ``AlohaMiniClient`` 被这些用例观察到的那些面：观测里不含侧信道
    （``_robot_metadata`` / ``_control_status`` / ``_host_timing`` 只落到同名
    属性上）、``send_action`` 只发出调用方的键集合、``connect`` / ``disconnect``
    可被会话层驱动。
    """

    def __init__(self, *, robot_model: str = "alohamini2pro") -> None:
        """按 robot_model 固定状态键顺序（与生产 client 的 ``_state_order`` 一致）。"""
        left_keys, right_keys = arm_state_keys_for_robot_model(robot_model)
        self._left_arm_state_keys = left_keys
        self._right_arm_state_keys = right_keys
        self._state_order = (
            *left_keys,
            *right_keys,
            "x.vel",
            "y.vel",
            "theta.vel",
            "lift_axis.height_mm",
        )
        self.sends: list[dict] = []
        self.last_remote_state: dict = {}
        self.logs: dict = {}
        self.latest_host_timing: dict = {}
        self.latest_robot_metadata: dict = {}
        self.latest_control_status: dict | None = None
        self._observation_sequence = 0
        self._host_session_id: str | None = None
        self._host_sample_sequence: int | None = None
        self._is_connected = False
        self._receive_observation_response = None

    @property
    def is_connected(self) -> bool:
        """连接状态（与生产 client 同名的只读属性）。"""
        return self._is_connected

    @property
    def observation_sequence(self) -> int:
        """已成功接收的观测帧数（与生产 client 同名的只读属性）。"""
        return self._observation_sequence

    def connect(self) -> None:
        """标记为已连接：不建立任何传输。"""
        self._is_connected = True

    def disconnect(self) -> None:
        """标记为已断开：不关闭任何传输。"""
        self._is_connected = False

    def get_observation(self, *, include_cameras: bool = True) -> dict:
        """取一帧脚本化观测：侧信道落到属性上，返回值只留状态键与 state 向量。"""
        parts = self._receive_observation_response(b"request-token", 20)
        if parts is None:
            return self.last_remote_state
        frame = json.loads(parts[0].decode("utf-8"))
        flat_state = {key: frame.get(key, 0.0) for key in self._state_order}
        observation = {
            **flat_state,
            OBS_STATE: np.array([flat_state[key] for key in self._state_order], dtype=np.float32),
        }
        self.latest_host_timing = dict(frame.get("_host_timing", {}))
        self.latest_robot_metadata = dict(frame.get("_robot_metadata", {}))
        status = frame.get("_control_status")
        self.latest_control_status = dict(status) if isinstance(status, dict) else None
        if self.latest_control_status is not None:
            session_id = self.latest_control_status.get("host_session_id")
            if session_id is not None:
                self._host_session_id = session_id
            sample_sequence = self.latest_control_status.get("sample_sequence")
            if sample_sequence is not None:
                self._host_sample_sequence = sample_sequence
        self._observation_sequence += 1
        self.last_remote_state = observation
        return observation

    def send_action(self, action: dict) -> dict:
        """记录一次发送：JSON 往返等价于生产 client 的线上载荷，并原样返回入参。"""
        self.sends.append(json.loads(json.dumps(action)))
        return dict(action)


# ===========================================================================
# Harness: host-client stand-in used by the connect-skeleton RED tests.
# ===========================================================================


class _FakeHostClient:
    """Client-like object the api layer is expected to drive.

    Counts every call unconditionally so the api layer's own idempotency is
    observable (a blindly-forwarding ``disconnect`` shows up as 2).
    """

    def __init__(self, host: str, **kwargs: Any) -> None:
        self.host = host
        self.extra_kwargs = kwargs
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.connected = False

    def connect(self) -> None:
        self.connect_calls += 1
        self.connected = True

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.connected = False

    @property
    def is_connected(self) -> bool:
        return self.connected

    def get_observation(self, *, include_cameras: bool = True) -> Any:
        return {}

    def send_action(self, action: Any) -> Any:
        return action


class _ClientFactory:
    """Injectable ``client_factory``: records calls and exposes the last client."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, dict]] = []
        self.client: _FakeHostClient | None = None

    def __call__(self, host: str, **kwargs: Any) -> _FakeHostClient:
        self.calls.append((host, kwargs))
        self.client = _FakeHostClient(host, **kwargs)
        return self.client


# ===========================================================================
# Characterization baseline (GREEN today).
# ===========================================================================


def test_char_3_controller_gripper_command_is_interlock_only() -> None:
    """A GripperCommand arms ACTIVE(ARM) but never sends or solves.

    A wrong implementation that forwards the gripper command to the sender, or
    that reports ``sent=True``, is caught here.
    """

    class _RecordingSender:
        def __init__(self) -> None:
            self.calls: list[dict] = []

        def __call__(self, action: Any) -> None:
            self.calls.append(dict(action))

    machine = ControlStateMachine(settled_samples=1)
    machine.on_feedback(1, settled=True)
    machine.resume()
    sender = _RecordingSender()

    controller = CartesianController(
        state_machine=machine,
        arm_ik=None,
        adapter=None,
        side="left",
        clock=lambda: 0.0,
        sender=sender,
        max_joint_lead_deg=30.0,
        follow_error_tolerance_deg=1.0,
        follow_error_timeout_s=1.0,
    )

    outcome = controller.tick({}, GripperCommand(side=Side.LEFT, action=GripperAction.OPEN))

    assert outcome.sent is False
    assert outcome.action is None
    assert outcome.error_code is None
    assert sender.calls == []
    assert machine.state is ControlState.ACTIVE
    assert machine.active_stage is Stage.ARM


def test_char_4_fault_reset_clears_last_reason() -> None:
    """Current record: reset() from a latched FAULT clears last_reason."""
    machine = ControlStateMachine(settled_samples=1)
    machine.on_feedback(5, settled=True)
    machine.resume()
    assert machine.state is ControlState.READY

    machine.on_feedback(3, settled=True)
    assert machine.state is ControlState.FAULT
    assert machine.last_reason is not None
    assert "regression" in machine.last_reason

    machine.reset()
    assert machine.state is ControlState.STANDBY
    assert machine.last_reason is None


def test_char_5_control_error_vocabulary_snapshot() -> None:
    """Frozen 10-code vocabulary; a silent add/rename/remove fails here."""
    expected = {
        "STEP_LIMIT": "step_limit",
        "NOT_ACTIVE": "not_active",
        "IK_FAILED": "ik_failed",
        "ADAPTER_FAILED": "adapter_failed",
        "RESIDUAL": "residual",
        "JOINT_LIMIT": "joint_limit",
        "NON_FINITE": "non_finite",
        "OVERCURRENT_HOLD": "overcurrent_hold",
        "FOLLOW_TIMEOUT": "follow_timeout",
        "UNSUPPORTED_TARGET": "unsupported_target",
    }
    actual = {member.name: member.value for member in ControlError}
    assert actual == expected
    assert len(ControlError) == 10


# ===========================================================================
# New RED — export contract (B16).
# ===========================================================================


def test_red_b16_package_exports_and_error_reason_contract() -> None:
    """Exact public import surface plus the reason-carrying error contract.

    Catches an api module that exists but is not re-exported, an error type
    missing ``.reason``, a dropped error type, or a duplicated ``MotionBlocked``
    that is not the commands-layer class.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)

    from alohamini_cartesian.control import (
        ArmHandle,
        BaseHandle,
        FeedbackLost,
        GripperHandle,
        LiftHandle,
        MotionBlocked,
        MotionNotEnabled,
        MotionTimeout,
        OutOfRange,
        connect,
    )
    from alohamini_cartesian.control.commands import (
        MotionBlocked as CommandsMotionBlocked,
    )

    assert MotionBlocked is CommandsMotionBlocked
    assert callable(connect)
    assert isinstance(control.AlohaMiniControl, type)

    for error_type in (FeedbackLost, OutOfRange, MotionTimeout, MotionNotEnabled):
        assert isinstance(error_type, type)
        assert issubclass(error_type, Exception)
        instance = error_type("synthetic reason")
        assert instance.reason == "synthetic reason"

    for handle_type in (ArmHandle, LiftHandle, BaseHandle, GripperHandle):
        assert isinstance(handle_type, type)


# ===========================================================================
# New RED — B5 enable_motion gate.
# ===========================================================================


def test_red_b5_gate_fails_closed_before_any_host_io() -> None:
    """dry_run=False + enable_motion=False raises MotionNotEnabled at construction.

    Catches an implementation that connects the host and only then refuses, or
    that silently allows real-motion construction without the enable flag.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    factory = _ClientFactory()

    with pytest.raises(control.MotionNotEnabled) as excinfo:
        control.connect(
            "192.168.0.10",
            dry_run=False,
            enable_motion=False,
            client_factory=factory,
        )

    assert excinfo.value.reason
    assert factory.calls == [], "the enable gate must fire before touching the host"


def test_red_b5_dry_run_needs_no_enable_flag_and_still_connects_host() -> None:
    """dry_run=True constructs with only the default enable_motion=False.

    Catches an implementation that wrongly requires enable_motion for dry runs,
    or that skips the host connection in dry-run mode.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    factory = _ClientFactory()

    session = control.connect("192.168.0.10", dry_run=True, client_factory=factory)

    assert session.dry_run is True
    assert len(factory.calls) == 1
    assert factory.calls[0][0] == "192.168.0.10"
    assert factory.client is not None
    assert factory.client.connect_calls == 1

    session.disconnect()


def test_red_b5_enable_motion_allows_real_motion_construction() -> None:
    """dry_run=False + enable_motion=True constructs and connects the host."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    factory = _ClientFactory()

    session = control.connect(
        "192.168.0.10",
        dry_run=False,
        enable_motion=True,
        client_factory=factory,
    )

    assert session.dry_run is False
    assert factory.client is not None
    assert factory.client.connect_calls == 1

    session.disconnect()


# ===========================================================================
# New RED — connect skeleton (factory injection, wiring, lifecycle).
# ===========================================================================


def test_red_connect_wires_session_handles_and_uses_injected_factory() -> None:
    """connect() returns AlohaMiniControl with left/right/lift/base wiring.

    Catches a connect() that ignores ``client_factory`` (and would otherwise
    build a real AlohaMiniClient), or that returns a bare host client instead of
    the session object with handles.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    factory = _ClientFactory()

    session = control.connect("host-1", dry_run=True, client_factory=factory)

    assert isinstance(session, control.AlohaMiniControl)
    assert isinstance(session.left, control.ArmHandle)
    assert isinstance(session.right, control.ArmHandle)
    assert isinstance(session.lift, control.LiftHandle)
    assert isinstance(session.base, control.BaseHandle)
    assert isinstance(session.left.gripper, control.GripperHandle)
    assert isinstance(session.right.gripper, control.GripperHandle)

    assert session.dry_run is True
    assert session.last_reason is None
    assert session.state is not None
    assert len(factory.calls) == 1
    assert factory.calls[0][0] == "host-1"

    session.disconnect()


def test_red_connect_disconnect_is_idempotent() -> None:
    """A second disconnect() is a no-op; the host client is closed exactly once."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    factory = _ClientFactory()

    session = control.connect("host-1", dry_run=True, client_factory=factory)
    assert factory.client is not None

    session.disconnect()
    session.disconnect()

    assert factory.client.disconnect_calls == 1


def test_red_connect_context_manager_disconnects_on_exit() -> None:
    """``with connect(...) as session`` yields the session and closes on exit."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    factory = _ClientFactory()

    with control.connect("host-1", dry_run=True, client_factory=factory) as session:
        assert isinstance(session, control.AlohaMiniControl)
        assert factory.client is not None
        assert factory.client.disconnect_calls == 0

    assert factory.client is not None
    assert factory.client.disconnect_calls == 1


def test_red_connect_session_stop_is_non_throwing() -> None:
    """Session-level stop() is callable and does not raise."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    factory = _ClientFactory()

    session = control.connect("host-1", dry_run=True, client_factory=factory)
    session.stop()

    session.disconnect()


# ===========================================================================
# P5 cycle 2 — B4 (no homing on connect) + B2 (blocking move_to stepping).
# ---------------------------------------------------------------------------
# Appended to the cycle-1 suite; the 13 cycle-1 cases above are untouched.
#
# Interpretation pinned by these tests (V07 for B4; D011 直线步进落点见 control/api.py；
#   继承见 docs/ADR.md D17):
#
# * B4 — after ``connect`` the SDK plans/dispatches NO homing/return
#   trajectory.  Its first arm action is a ZERO-DISPLACEMENT hold at the
#   current measured pose, built WITHOUT any ``ArmIK.solve`` call.
# * B2 — ``ArmHandle.move_to(x, y, z)`` runs a 30 Hz loop that drives the real
#   ``CartesianController`` until (a) the measured FK position residual is
#   within ``arrival_tolerance_m`` AND (b) a NEW feedback sample confirms
#   standstill.  It never returns early; after ``move_timeout_s`` it raises
#   ``MotionTimeout`` carrying ``moved``; it returns a ``MoveReport`` whose
#   dry-run form carries per-step ``StepDiagnostic`` entries.
#
# Injection seam (the session owns the motion machinery; the test injects the
# leaves)::
#
#   connect(host, *, dry_run, enable_motion, client_factory,
#           clock, sleeper, ik_factory, adapter,
#           control_period_s, arrival_tolerance_m, move_timeout_s)
#
# ``ik_factory(side)`` builds the arm IK seam, ``adapter`` is the real
# ``JointKinematicAdapter``, and the session builds a real
# ``CartesianController`` per side on top of them.  ``sleeper`` advances the
# injected ``clock`` so a whole 30 Hz move runs deterministically; a tick
# budget turns a runaway loop into an AssertionError instead of a hang.
# ===========================================================================

_C2_JOINT_ORDER = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)

_C2_SIGNS = {
    "shoulder_pan": 1.0,
    "shoulder_lift": -1.0,
    "elbow_flex": 1.0,
    "wrist_flex": -1.0,
    "wrist_yaw": 1.0,
    "wrist_roll": -1.0,
}

_C2_ZEROS = dict.fromkeys(_C2_JOINT_ORDER, 0.0)

_C2_MODEL = "alohamini2pro"

# Frozen current pose and reachable/unreachable probes.  FK maps
# ``q[:3] * 0.001`` to the target translation, so the position residual is
# exact:
#   Q0      -> (0.010, -0.020, 0.030)  residual 0.0065  (> tol)
#   Q_STUCK -> (0.0105,-0.020, 0.030)  residual 0.0060  (> tol, moved 0.5 mm)
#   Q_ARR   -> (0.012, -0.020, 0.030)  residual 0.0045  (<= tol)
_C2_Q0 = (10.0, -20.0, 30.0, -40.0, 50.0, -60.0)
_C2_Q_STUCK = (10.5, -20.0, 30.0, -40.0, 50.0, -60.0)
_C2_Q_ARRIVED = (12.0, -20.0, 30.0, -40.0, 50.0, -60.0)
_C2_TARGET_XYZ = (0.0165, -0.020, 0.030)

_C2_TOL = 0.005
_C2_PERIOD = 1.0 / 30.0


def _c2_motor_pos(q_deg: Any, side: str = "left") -> dict:
    """Motor-degree positions whose adapter forward map yields exactly q_deg."""
    q = np.asarray(q_deg, dtype=float)
    return {
        f"arm_{side}_{joint}.pos": _C2_SIGNS[joint] * (q[index] - _C2_ZEROS[joint])
        for index, joint in enumerate(_C2_JOINT_ORDER)
    }


def _c2_metadata(side: str = "left") -> dict:
    motors = {}
    for index, joint in enumerate(_C2_JOINT_ORDER):
        motors[f"arm_{side}_{joint}"] = {
            "id": index + 1,
            "model": f"syn-c2-{index + 1}",
            "normalization": "degrees",
            "drive_mode": 1 if index % 2 else 0,
            "range_min": 41 + index,
            "range_max": 4001 + index,
            "homing_offset": 100 * (index + 1) - 500,
            "resolution": 6399 + index,
        }
    return {"schema_version": 2, "robot_model": _C2_MODEL, "motors": motors}


def _c2_obs(q_deg: Any, *, side: str = "left", settled: bool = True) -> dict:
    """One scripted observation: arm state + metadata + control_status."""
    observation = dict(_c2_motor_pos(q_deg, side))
    observation["_robot_metadata"] = _c2_metadata(side)
    observation["_control_status"] = {
        "schema_version": 1,
        "host_session_id": "c2-session",
        "sample_sequence": 1,
        "settled": bool(settled),
        "valid": True,
        "joint_holds": {},
        "gripper_holds": {},
    }
    return observation


def _c2_fk(q_deg: Any) -> Any:
    """Deterministic linear FK: translation == q[:3] * 0.001, identity rotation."""
    q = np.asarray(q_deg, dtype=float)
    pose = np.eye(4)
    pose[0, 3] = q[0] * 0.001
    pose[1, 3] = q[1] * 0.001
    pose[2, 3] = q[2] * 0.001
    return pose


def _c2_ik_ok(q_deg: Any, *, position_error_m: float = 0.0) -> Any:
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


def _c2_extract_seed(args: Any, kwargs: Any) -> Any:
    if args:
        return np.asarray(args[0], dtype=float)
    for key in ("q_current_deg", "q_seed_deg", "q_deg", "seed_deg"):
        if key in kwargs:
            return np.asarray(kwargs[key], dtype=float)
    raise AssertionError(f"no IK seed in solve call: args={args!r} kwargs={kwargs!r}")


def _c2_step_solver(*args: Any, **kwargs: Any) -> Any:
    """A bounded 0.5 deg step from the seed; always reports a perfect residual."""
    q = np.array(_c2_extract_seed(args, kwargs), dtype=float)
    q[0] += 0.5
    return _c2_ik_ok(q)


class _C2ScriptedIK:
    """arm_ik seam: deterministic ``fk`` plus a scriptable ``solve``."""

    def __init__(
        self, *, solve_fn: Any = None, results: Any = None, residual_limit_m: float = 0.005
    ) -> None:
        from types import SimpleNamespace

        self._solve_fn = solve_fn
        self._results = list(results) if results is not None else []
        self.solve_calls: list[dict] = []
        self.fk_calls: list[np.ndarray] = []
        self.profile = SimpleNamespace(residual_limit_m=float(residual_limit_m))

    def fk(self, q_deg: Any) -> Any:
        q = np.asarray(q_deg, dtype=float)
        self.fk_calls.append(np.copy(q))
        return _c2_fk(q)

    def solve(self, *args: Any, **kwargs: Any) -> Any:
        self.solve_calls.append({"args": tuple(args), "kwargs": dict(kwargs)})
        if self._solve_fn is not None:
            return self._solve_fn(*args, **kwargs)
        if not self._results:
            raise AssertionError("_C2ScriptedIK exhausted: solved more than scripted")
        return self._results.pop(0)


class _C2FakeClient:
    """Observation-producing client stand-in with deep-copied send_action records.

    ``observer(client)`` is re-evaluated on every ``get_observation`` so the
    scripted pose can react to loop progress (the number of IK solves / sends)
    without the test needing to know how many priming ticks the loop uses.
    ``sample_sequence`` is always incremented by the fake, guaranteeing the
    standstill gate sees a *new* sample each tick.
    """

    def __init__(self, *, observer: Any) -> None:
        self.observer = observer
        self.sends: list[dict] = []
        self.connect_calls = 0
        self.disconnect_calls = 0
        self._sample = 0

    def connect(self) -> None:
        self.connect_calls += 1

    def disconnect(self) -> None:
        self.disconnect_calls += 1

    @property
    def is_connected(self) -> bool:
        return True

    def get_observation(self, **_kwargs: Any) -> Any:
        self._sample += 1
        observation = dict(self.observer(self))
        control_status = dict(observation.get("_control_status") or {})
        control_status["sample_sequence"] = self._sample
        control_status.setdefault("host_session_id", "c2-session")
        observation["_control_status"] = control_status
        return observation

    def send_action(self, action: Any) -> Any:
        import copy

        self.sends.append({key: copy.deepcopy(value) for key, value in dict(action).items()})
        return dict(action)


class _C2FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.t = float(start)

    def __call__(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += float(seconds)


class _C2RecordingSleeper:
    """Advances the fake clock and hard-caps the tick budget."""

    def __init__(self, clock: _C2FakeClock, *, max_ticks: int = 300) -> None:
        self.clock = clock
        self.sleeps: list[float] = []
        self.max_ticks = max_ticks

    def __call__(self, seconds: float) -> None:
        self.sleeps.append(float(seconds))
        self.clock.advance(seconds)
        if len(self.sleeps) > self.max_ticks:
            raise AssertionError(
                f"move loop exceeded {self.max_ticks} ticks (possible infinite loop)"
            )


def _c2_adapter(side: str = "left") -> Any:
    from alohamini_cartesian.kinematics.joint_adapter import JointKinematicAdapter

    return JointKinematicAdapter(signs={side: dict(_C2_SIGNS)}, zeros={side: dict(_C2_ZEROS)})


def _c2_connect(
    *,
    dry_run: bool,
    client: _C2FakeClient,
    clock: _C2FakeClock,
    sleeper: _C2RecordingSleeper,
    ik: _C2ScriptedIK,
    adapter: Any,
    move_timeout_s: float = 10.0,
    arrival_tolerance_m: float = _C2_TOL,
) -> Any:
    control = importlib.import_module(_CONTROL_PACKAGE)
    return control.connect(
        "host-c2",
        dry_run=dry_run,
        enable_motion=not dry_run,
        client_factory=lambda host, **kwargs: client,
        clock=clock,
        sleeper=sleeper,
        ik_factory=lambda side: ik,
        adapter=adapter,
        control_period_s=_C2_PERIOD,
        arrival_tolerance_m=arrival_tolerance_m,
        move_timeout_s=move_timeout_s,
    )


def _constant_ik_factory(ik: Any) -> Any:
    """Build an ``ik_factory`` that hands the same IK seam to every side."""

    def factory(_side: Any) -> Any:
        return ik

    return factory


def _c2_field(obj: Any, name: Any) -> Any:
    if isinstance(obj, dict):
        return obj[name]
    return getattr(obj, name)


def _c2_assert_hold(action: Any, q_deg: Any, *, side: str = "left") -> None:
    """A zero-displacement hold must replay the current measured positions."""
    expected = _c2_motor_pos(q_deg, side)
    for key, value in expected.items():
        assert key in action, f"hold action missing joint key {key!r}: {action!r}"
        assert action[key] == pytest.approx(value, abs=1e-9), f"hold key {key!r} moved"
    assert any(abs(float(value)) > 1e-9 for value in action.values()), (
        "hold must not be an all-zero homing action"
    )


# ---------------------------------------------------------------------------
# B4 / V07 / D005 — connect never homes; first arm action holds the current pose.
# ---------------------------------------------------------------------------
def test_red_b4_connect_does_not_home_or_solve_ik() -> None:
    """``connect`` must not solve IK for, or dispatch, a homing/return trajectory.

    Catches a connect path that computes a home pose via IK and/or dispatches a
    motion action that differs from the current measured pose.
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))

    session = _c2_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )

    assert ik.solve_calls == [], "connect must not solve IK (no homing plan)"
    for send in client.sends:
        _c2_assert_hold(send, _C2_Q0)

    session.disconnect()


def test_red_b4_first_arm_action_is_zero_displacement_hold() -> None:
    """The SDK's first arm action is a zero-displacement hold at the current pose.

    Catches a connect path that jumps straight to a precomputed home/start pose
    instead of holding where the arm already is (D005: no extra PC homing).
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))

    session = _c2_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )

    assert len(client.sends) >= 1, "connect must dispatch an initial hold action"
    _c2_assert_hold(client.sends[0], _C2_Q0)

    session.disconnect()


def test_red_b4_connect_dry_run_stays_silent() -> None:
    """A dry-run connect performs no IK solve and no host action (V09)."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))

    session = _c2_connect(
        dry_run=True, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )

    assert client.sends == []
    assert ik.solve_calls == []

    session.disconnect()


# ---------------------------------------------------------------------------
# B2 / V01 — move_to blocks until arrival + standstill; times out; reports.
# ---------------------------------------------------------------------------
def test_red_b2_move_to_blocks_until_arrival_and_returns_report() -> None:
    """move_to returns only after the measured residual is within tolerance.

    The scripted arm only reports arrival once the loop has actually stepped
    (an IK solve happened).  A single-tick implementation that returns before
    driving the real controller cannot produce the send/sleep evidence.
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(
        observer=lambda _c: (
            _c2_obs(_C2_Q0, settled=True)
            if not ik.solve_calls
            else _c2_obs(_C2_Q_ARRIVED, settled=True)
        )
    )
    session = _c2_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=1.0,
    )

    report = session.left.move_to(*_C2_TARGET_XYZ)

    assert report is not None
    assert float(_c2_field(report, "residual_m")) <= _C2_TOL
    assert len(client.sends) >= 1, "arrival must have been driven through the controller"
    assert len(sleeper.sleeps) >= 1, "move_to must pace the loop"
    for period in sleeper.sleeps:
        assert period == pytest.approx(_C2_PERIOD, rel=1e-6)

    session.disconnect()


def test_red_b2_move_to_does_not_return_early_and_times_out_with_moved() -> None:
    """An arm that never reaches the target must not return; it times out.

    Catches the "returns after a single tick" failure mode: the loop must keep
    driving until ``move_timeout_s`` and then raise ``MotionTimeout`` carrying
    the measured ``moved`` amount.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(
        observer=lambda _c: (
            _c2_obs(_C2_Q0, settled=True)
            if not ik.solve_calls
            else _c2_obs(_C2_Q_STUCK, settled=True)
        )
    )
    session = _c2_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        # Long enough that the loop clears STANDBY priming and actually moves
        # the arm before the deadline, so ``moved`` is a positive measurement.
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)

    assert excinfo.value.reason
    moved = float(excinfo.value.moved)
    assert np.isfinite(moved)
    assert moved > 0.0, "MotionTimeout must carry the measured moved amount"
    assert len(sleeper.sleeps) >= 2, "must not return after a single tick"

    session.disconnect()


def test_red_b2_move_to_waits_for_settled_new_feedback() -> None:
    """Reaching the position is not enough: a NEW feedback must confirm standstill.

    Catches an implementation that ignores the settled flag and returns as soon
    as the residual is within tolerance.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(
        observer=lambda _c: (
            _c2_obs(_C2_Q0, settled=True)
            if not ik.solve_calls
            else _c2_obs(_C2_Q_ARRIVED, settled=False)
        )
    )
    session = _c2_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        # Long enough that arrival is actually reached and only the missing
        # settled flag keeps the loop alive; an ignores-settled implementation
        # would return a report here instead of timing out.
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    assert len(sleeper.sleeps) >= 2

    session.disconnect()


def test_red_b2_move_to_dry_run_returns_step_diagnostics_and_never_sends() -> None:
    """dry-run move_to never sends and returns a per-step diagnostic report (V09)."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(
        observer=lambda _c: (
            _c2_obs(_C2_Q0, settled=True)
            if not ik.solve_calls
            else _c2_obs(_C2_Q_ARRIVED, settled=True)
        )
    )
    session = _c2_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=1.0,
    )

    report = session.left.move_to(*_C2_TARGET_XYZ)

    assert client.sends == []
    assert float(_c2_field(report, "residual_m")) <= _C2_TOL
    steps = _c2_field(report, "steps")
    assert len(steps) >= 1, "dry-run report must carry the planned steps"
    assert any(_c2_field(step, "action") is not None for step in steps), (
        "each dry-run step diagnostic must carry its planned action"
    )

    session.disconnect()


# ===========================================================================
# P5 cycle 3 — B6 (whole-path precheck) + B3 (single-arm isolation)
#                + B17 (interlock wiring) + BLOCKED/FAULT fast-fail.
# ---------------------------------------------------------------------------
# Appended to the cycle-1/2 suite; the 20 earlier cases are untouched.
#
# Interpretation pinned by these tests (V08 / V02 / V03；D003 继承见 docs/ADR.md D17):
#
# * B6 — ``ArmHandle.move_to`` discretises the straight line from the current
#   measured FK pose to the target and, *before dispatching any arm action*,
#   runs offline IK + limit checks on the sampled waypoints.  An unreachable
#   endpoint OR an unreachable intermediate waypoint raises ``OutOfRange``
#   (carrying ``.target``) with ZERO actions dispatched on the wire.
# * B3 — a ``left.move_to`` only ever drives the left controller: every wire
#   action is ``arm_left_*`` and the right arm's IK is never solved.
# * B17 — ``base.drive`` / ``lift.move_to`` called while the shared state
#   machine is ACTIVE(ARM) raise ``MotionBlocked`` and latch BLOCKED.
# * BLOCKED/FAULT fast-fail — the move_to loop must notice the state machine
#   entering BLOCKED/FAULT and raise ``MotionBlocked`` immediately instead of
#   burning the whole ``move_timeout_s``.
#
# Precheck seam (cycle-3 refinement): the sampling grid is derived from the
# straight-line distance between the current FK pose and the target with a
# bounded Cartesian step; the waypoint count is ``ceil(distance / step)`` and
# the endpoint is always included.  The B6 tests below are deliberately
# step-agnostic (they place a wide "wall" that any interior-sample
# implementation rejects while an endpoint-only check misses), so the exact
# step stays an implementation tuning value rather than a test contract.
# ===========================================================================


def _c3_connect(
    *,
    dry_run: bool,
    client: Any,
    clock: Any,
    sleeper: Any,
    adapter: Any,
    ik: Any = None,
    ik_factory: Any = None,
    move_timeout_s: float = 10.0,
    arrival_tolerance_m: float = _C2_TOL,
) -> Any:
    """Cycle-3 connect helper: like ``_c2_connect`` but injects per-side IK.

    ``ik`` keeps the cycle-2 single-IK form; ``ik_factory`` is the cycle-3
    form used by the single-arm isolation tests to tell the two sides apart.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    if ik_factory is None:
        ik_factory = _constant_ik_factory(ik)
    return control.connect(
        "host-c3",
        dry_run=dry_run,
        enable_motion=not dry_run,
        client_factory=lambda host, **kwargs: client,
        clock=clock,
        sleeper=sleeper,
        ik_factory=ik_factory,
        adapter=adapter,
        control_period_s=_C2_PERIOD,
        arrival_tolerance_m=arrival_tolerance_m,
        move_timeout_s=move_timeout_s,
    )


def _c3_adapter_both() -> Any:
    """Kinematic adapter configured for both sides (isolation tests need both)."""
    from alohamini_cartesian.kinematics.joint_adapter import JointKinematicAdapter

    return JointKinematicAdapter(
        signs={"left": dict(_C2_SIGNS), "right": dict(_C2_SIGNS)},
        zeros={"left": dict(_C2_ZEROS), "right": dict(_C2_ZEROS)},
    )


def _c3_both_obs(q_left: Any, q_right: Any, *, settled: bool = True) -> dict:
    """Observation carrying BOTH arms' state + metadata.

    Providing the right arm's data means a wrong implementation that ticks the
    right controller would succeed and leak ``arm_right_*`` keys instead of
    failing for lack of input, so the isolation assertions stay discriminating.
    """
    observation: dict = {}
    observation.update(_c2_motor_pos(q_left, "left"))
    observation.update(_c2_motor_pos(q_right, "right"))
    motors = dict(_c2_metadata("left")["motors"])
    motors.update(_c2_metadata("right")["motors"])
    observation["_robot_metadata"] = {
        "schema_version": 2,
        "robot_model": _C2_MODEL,
        "motors": motors,
    }
    observation["_control_status"] = {
        "schema_version": 1,
        "host_session_id": "c3-session",
        "sample_sequence": 1,
        "settled": bool(settled),
        "valid": True,
        "joint_holds": {},
        "gripper_holds": {},
    }
    return observation


def _c3_ik_fail(reason: str = "RESIDUAL") -> Any:
    from alohamini_cartesian.control.arm_ik import IKResult

    return IKResult(
        q_deg=np.zeros(6, dtype=float),
        ok=False,
        reason=reason,
        position_error_m=float("inf"),
        orientation_error_deg=0.0,
        iterations=1,
        solve_time_s=0.0,
    )


def _c3_target_pose_from_call(args: Any, kwargs: Any) -> np.ndarray:
    if "target_pose" in kwargs:
        return np.asarray(kwargs["target_pose"], dtype=float)
    if len(args) >= 2:
        return np.asarray(args[1], dtype=float)
    raise AssertionError(f"no target_pose in solve call: args={args!r} kwargs={kwargs!r}")


def _c3_solver_rejecting_x(predicate: Any) -> Any:
    """Solved IK seam that rejects targets whose X falls in ``predicate``.

    Any target outside the band is reported ``ok`` at the seed, so the only
    way to observe the band is to actually sample a waypoint inside it.
    """

    def solve(*args: Any, **kwargs: Any) -> Any:
        x = float(_c3_target_pose_from_call(args, kwargs)[0, 3])
        if predicate(x):
            return _c3_ik_fail("RESIDUAL")
        return _c2_ik_ok(_c2_extract_seed(args, kwargs))

    return solve


class _C3ActionSleeper:
    """Paces the move loop and fires one probe once ARM motion is ACTIVE."""

    def __init__(self, clock: Any, holder: dict, *, action: Any, max_ticks: int = 600) -> None:
        self._inner = _C2RecordingSleeper(clock, max_ticks=max_ticks)
        self._holder = holder
        self._action = action
        self._fired = False
        self.trigger_time: float | None = None

    @property
    def sleeps(self) -> Any:
        return self._inner.sleeps

    def __call__(self, seconds: float) -> None:
        self._inner(seconds)
        session = self._holder.get("session")
        if session is None or self._fired:
            return
        if session.state.state is ControlState.ACTIVE and session.state.active_stage is Stage.ARM:
            self._fired = True
            self.trigger_time = self._inner.clock.t
            self._action(session)


def _c3_prime_active_arm(session: Any) -> None:
    """Drive the session's shared state machine into ACTIVE(ARM) directly."""
    from alohamini_cartesian.control.commands import ArmCommand

    for sample_id in (1, 2, 3):
        session.state.on_feedback(sample_id, settled=True)
    session.state.resume()
    session.state.handle(ArmCommand(side=Side.LEFT, dx=0.0, dy=0.0, dz=0.0))
    assert session.state.state is ControlState.ACTIVE
    assert session.state.active_stage is Stage.ARM


def _c3_assert_error_target(exc: Any, expected_xyz: Any) -> None:
    """``OutOfRange`` must carry the requested target as a 3-vector (or pose)."""
    target = getattr(exc, "target", None)
    assert target is not None, "OutOfRange must carry .target"
    target_array = np.asarray(target, dtype=float)
    if target_array.shape == (4, 4):
        target_array = target_array[:3, 3]
    assert target_array.reshape(-1) == pytest.approx(tuple(expected_xyz))


# ---------------------------------------------------------------------------
# B6 / §2 — the whole straight line is pre-checked before any action is sent.
# ---------------------------------------------------------------------------
def test_red_b6_unreachable_target_zero_action_raises_out_of_range() -> None:
    """An unreachable endpoint aborts with OutOfRange(carrying target), 0 sends.

    Catches an implementation that only discovers unreachability inside the
    motion loop (ending in ``MotionTimeout``) or that dispatches an action
    before refusing.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    # Reachable everywhere up to x=0.0150; the requested 0.0165 endpoint is not.
    ik = _C2ScriptedIK(solve_fn=_c3_solver_rejecting_x(lambda x: x > 0.0150))
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c3_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=1.0,
    )
    client.sends.clear()

    with pytest.raises(control.OutOfRange) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)

    assert client.sends == [], "an unreachable precheck must dispatch no action"
    assert excinfo.value.reason
    _c3_assert_error_target(excinfo.value, _C2_TARGET_XYZ)

    session.disconnect()


def test_red_b6_midpath_blocked_point_is_prechecked_before_any_send() -> None:
    """A reachable endpoint is still refused when an interior waypoint is blocked.

    Start x=0.010, target x=0.0165; the endpoint is reachable while a wide
    interior band is not.  An endpoint-only check (or no precheck at all) lets
    the move proceed and dispatch actions, so this is the whole-path witness.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)

    def wall(x: Any) -> bool:
        """Unreachable band: the precheck must sample a waypoint inside it."""
        return 0.0102 <= x <= 0.0163

    ik = _C2ScriptedIK(solve_fn=_c3_solver_rejecting_x(wall))
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(
        observer=lambda _c: (
            _c2_obs(_C2_Q0, settled=True)
            if not ik.solve_calls
            else _c2_obs(_C2_Q_ARRIVED, settled=True)
        )
    )
    session = _c3_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )
    client.sends.clear()

    with pytest.raises(control.OutOfRange) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)

    assert client.sends == [], "a blocked interior waypoint must be caught before departure"
    _c3_assert_error_target(excinfo.value, _C2_TARGET_XYZ)

    session.disconnect()


# ---------------------------------------------------------------------------
# B3 / V02 — a single-arm move is isolated to that arm's controller and keys.
# ---------------------------------------------------------------------------
def test_red_b3_left_move_sends_only_left_arm_keys_and_never_solves_right_ik() -> None:
    """``left.move_to`` only emits ``arm_left_*`` and never ticks the right arm."""
    left_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    right_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(
        observer=lambda _c: _c3_both_obs(
            _C2_Q_ARRIVED if left_ik.solve_calls else _C2_Q0,
            _C2_Q0,
        )
    )
    session = _c3_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik_factory=lambda side: left_ik if side == "left" else right_ik,
    )
    client.sends.clear()

    session.left.move_to(*_C2_TARGET_XYZ)

    assert client.sends, "the left move must dispatch at least one action"
    for action in client.sends:
        leaked = [key for key in action if not key.startswith("arm_left_")]
        assert leaked == [], f"left move leaked non-left keys: {leaked!r}"
    assert right_ik.solve_calls == [], "the right controller must not be ticked"
    assert right_ik.fk_calls == [], "the right arm IK must not be used"

    session.disconnect()


def test_red_b3_right_move_sends_only_right_arm_keys_and_never_solves_left_ik() -> None:
    """``right.move_to`` only emits ``arm_right_*`` and never ticks the left arm."""
    left_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    right_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(
        observer=lambda _c: _c3_both_obs(
            _C2_Q0,
            _C2_Q_ARRIVED if right_ik.solve_calls else _C2_Q0,
        )
    )
    session = _c3_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik_factory=lambda side: left_ik if side == "left" else right_ik,
    )
    client.sends.clear()

    session.right.move_to(*_C2_TARGET_XYZ)

    assert client.sends, "the right move must dispatch at least one action"
    for action in client.sends:
        leaked = [key for key in action if not key.startswith("arm_right_")]
        assert leaked == [], f"right move leaked non-right keys: {leaked!r}"
    assert left_ik.solve_calls == [], "the left controller must not be ticked"
    assert left_ik.fk_calls == [], "the left arm IK must not be used"

    session.disconnect()


# ---------------------------------------------------------------------------
# B17 / V03 — base/lift calls are refused while the machine is ACTIVE(ARM).
# ---------------------------------------------------------------------------
def test_red_b17_base_drive_refused_during_arm_motion() -> None:
    """``base.drive`` during ACTIVE(ARM) raises MotionBlocked and latches BLOCKED."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c3_connect(
        dry_run=True, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )

    _c3_prime_active_arm(session)

    with pytest.raises(control.MotionBlocked):
        session.base.drive(x=0.1, y=0.0, turn=0.0, seconds=1.0)

    assert session.state.state is ControlState.BLOCKED
    assert session.state.last_reason
    base_keys = {"x.vel", "y.vel", "theta.vel"}
    assert not any(key in base_keys for action in client.sends for key in action), (
        "a refused base call must not dispatch a base velocity"
    )

    session.disconnect()


def test_red_b17_lift_move_to_refused_during_arm_motion() -> None:
    """``lift.move_to`` during ACTIVE(ARM) raises MotionBlocked and latches BLOCKED."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c3_connect(
        dry_run=True, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )

    _c3_prime_active_arm(session)

    with pytest.raises(control.MotionBlocked):
        session.lift.move_to(150)

    assert session.state.state is ControlState.BLOCKED
    assert session.state.last_reason

    session.disconnect()


# ---------------------------------------------------------------------------
# BLOCKED / FAULT fast-fail inside the move_to loop (no timeout burn).
# ---------------------------------------------------------------------------
def test_red_move_to_fast_fails_on_blocked_without_timeout() -> None:
    """A mid-loop BLOCKED raises MotionBlocked immediately, not MotionTimeout."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    holder: dict = {}
    sleeper = _C3ActionSleeper(clock, holder, action=lambda s: s.state.block("synthetic_block"))
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c3_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=5.0,
    )
    holder["session"] = session

    with pytest.raises(control.MotionBlocked) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)

    assert sleeper.trigger_time is not None, "the probe must have run during ACTIVE(ARM)"
    assert clock.t < 1.0, "BLOCKED must fail fast, not wait for move_timeout_s"
    assert "synthetic_block" in str(excinfo.value)

    session.disconnect()


def test_red_move_to_fast_fails_on_fault_without_timeout() -> None:
    """A mid-loop FAULT raises MotionBlocked carrying last_reason, not a timeout."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    holder: dict = {}
    sleeper = _C3ActionSleeper(clock, holder, action=lambda s: s.state.on_feedback(0, settled=True))
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c3_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=5.0,
    )
    holder["session"] = session

    with pytest.raises(control.MotionBlocked) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)

    assert clock.t < 1.0, "FAULT must fail fast, not wait for move_timeout_s"
    assert session.state.state is ControlState.FAULT
    reason = session.state.last_reason
    assert reason and "regression" in reason
    assert reason in str(excinfo.value)

    session.disconnect()


# ===========================================================================
# P5 cycle 4 — B9 (D013 keepalive) + B10 (D014 base parking)
#                + B7 (FAULT reason presentation) + B14 (FeedbackLost)
#                + B8 (settled/valid fail-closed arrival gate).
# ---------------------------------------------------------------------------
# Appended to the cycle-1/2/3 suite; the 28 earlier cases are untouched.
#
# Interpretation pinned by these tests (D013 keepalive / D014 底盘显式零速，落点见
#   control/controller.py 与 V09；继承见 docs/ADR.md D17):
#
# * B9 — ``connect(..., keepalive_interval_s=K)`` arms each per-side
#   controller's ``next_deadline`` (``last_send + K``).  While a blocking
#   ``move_to`` loop runs, a step/poll that consumes more than ``K`` seconds
#   since the last dispatch must trigger a *resend of the current target*
#   on top of the regular 30 Hz step.  With a loop period larger than ``K``
#   that shows up as strictly more sends than feedback polls; without any
#   keepalive configuration it must not (negative control).
# * B10 — ``base.drive(seconds=N)`` is a blocking velocity loop that ends by
#   dispatching an explicit all-zero velocity; ``base.stop()`` dispatches an
#   explicit all-zero velocity; ``disconnect()`` dispatches an explicit
#   all-zero velocity iff a base velocity was actually dispatched before.
# * B7 — the session surfaces the FAULT reason: ``session.last_reason`` is
#   populated at fault time (not lazily read after a reset cleared the state
#   machine's copy) and survives ``state.reset()`` (design §12.4: the reason
#   is discarded by the reset path, so the SDK must present it).
# * B14 — a sustained loss of new observations aborts a blocking move with
#   ``FeedbackLost`` instead of silently driving to ``MotionTimeout``.
# * B8 — a missing ``settled``/``valid`` field, or an explicit ``valid=False``
#   on an already-active arm, is fail-closed: reaching the position residual
#   is not accepted as arrival.
#
# Pinned seams / assumptions to adjudicate (flagged, not silently absorbed):
#   - B9 uses a session-level ``keepalive_interval_s`` kwarg on ``connect`` and
#     observes resends through the send stream (the per-side controller already
#     exposes its ``next_deadline``; the session consumes it).  If the desired
#     seam is a public ``session.keepalive()`` instead, this must be re-planned.
#   - B14 uses the client's new-frame counter (``observation_sequence``) and the
#     ``_control_status.sample_sequence`` side channel; the stale client below
#     freezes BOTH so the test is agnostic to which signal the SDK keys on.
#   - B7 uses ``state.reset()`` as the public proxy for the reset path that
#     clears ``state.last_reason``.
# ===========================================================================

_C4_BASE_KEYS = ("x.vel", "y.vel", "theta.vel")


def _c4_connect(
    *,
    dry_run: bool,
    client: Any,
    clock: Any,
    sleeper: Any,
    adapter: Any,
    ik: Any = None,
    ik_factory: Any = None,
    control_period_s: float = _C2_PERIOD,
    move_timeout_s: float = 10.0,
    arrival_tolerance_m: float = _C2_TOL,
    **session_kwargs: Any,
) -> Any:
    """Cycle-4 connect helper: forwards optional session kwargs (e.g. keepalive)."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    if ik_factory is None:
        ik_factory = _constant_ik_factory(ik)
    return control.connect(
        "host-c4",
        dry_run=dry_run,
        enable_motion=not dry_run,
        client_factory=lambda host, **kwargs: client,
        clock=clock,
        sleeper=sleeper,
        ik_factory=ik_factory,
        adapter=adapter,
        control_period_s=control_period_s,
        arrival_tolerance_m=arrival_tolerance_m,
        move_timeout_s=move_timeout_s,
        **session_kwargs,
    )


class _C4StaleClient(_C2FakeClient):
    """Observation client whose feedback sequences freeze after priming.

    The first ``fresh_polls`` polls advance both the client-level
    ``observation_sequence`` counter and the ``_control_status.sample_sequence``
    side channel.  From then on both are pinned to ``fresh_polls``, simulating
    a stalled feedback stream whose cached arm data never refreshes.
    """

    def __init__(self, *, observer: Any, fresh_polls: int = 5) -> None:
        super().__init__(observer=observer)
        self.fresh_polls = int(fresh_polls)
        self.observation_sequence = 0

    def get_observation(self, **kwargs: Any) -> Any:
        observation = super().get_observation(**kwargs)
        poll = self._sample
        if poll <= self.fresh_polls:
            self.observation_sequence = poll
            sequence = poll
        else:
            self.observation_sequence = self.fresh_polls
            sequence = self.fresh_polls
        control_status = dict(observation.get("_control_status") or {})
        control_status["sample_sequence"] = sequence
        control_status["host_session_id"] = "c4-session"
        observation["_control_status"] = control_status
        return observation


def _c4_prime_ready(session: Any) -> None:
    """Drive the shared state machine into READY (motion-command precondition)."""
    for sample_id in (1, 2, 3):
        session.state.on_feedback(sample_id, settled=True)
    session.state.resume()
    assert session.state.state is ControlState.READY


def _c4_has_base_keys(action: Any) -> bool:
    return any(key in action for key in _C4_BASE_KEYS)


def _c4_assert_base_zero(action: Any) -> None:
    for key in _C4_BASE_KEYS:
        assert key in action, f"base stop must carry {key!r}: {action!r}"
        assert float(action[key]) == pytest.approx(0.0, abs=1e-9), f"{key} must be zero"


# ---------------------------------------------------------------------------
# B9 / D013 — keepalive resends the current target inside a blocking loop.
# ---------------------------------------------------------------------------
def test_red_b9_keepalive_resends_current_target_within_blocking_loop() -> None:
    """With ``keepalive_interval_s`` shorter than the loop period, resends occur.

    A loop period of 0.1 s crosses many 0.02 s keepalive deadlines; the SDK must
    resend the current target on top of the regular step, so the send count
    strictly exceeds the number of feedback polls.  Catches an implementation
    that ignores the configured deadline.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    polls: list[float] = []

    def observer(_c: Any) -> Any:
        polls.append(clock.t)
        return _c2_obs(_C2_Q0 if not ik.solve_calls else _C2_Q_STUCK, settled=True)

    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        control_period_s=0.1,
        move_timeout_s=1.5,
        keepalive_interval_s=0.02,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    assert len(polls) >= 5, "sanity: the loop must have polled several times"
    assert len(client.sends) > len(polls), (
        "a keepalive deadline crossed between steps must trigger a resend on top "
        f"of the regular step (sends={len(client.sends)}, polls={len(polls)})"
    )

    session.disconnect()


def test_red_b9_without_keepalive_config_no_extra_resends() -> None:
    """Negative control: no keepalive configured => no resend on top of steps.

    Guards the positive B9 case against an implementation that unconditionally
    double-sends (or that would pass B9 for the wrong reason).
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    polls: list[float] = []

    def observer(_c: Any) -> Any:
        polls.append(clock.t)
        return _c2_obs(_C2_Q0 if not ik.solve_calls else _C2_Q_STUCK, settled=True)

    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        control_period_s=0.1,
        move_timeout_s=1.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    assert len(polls) >= 5, "sanity: the loop must have polled several times"
    assert len(client.sends) < len(polls), (
        "without a keepalive interval the loop must not double-send "
        f"(sends={len(client.sends)}, polls={len(polls)})"
    )

    session.disconnect()


# ---------------------------------------------------------------------------
# B10 / D014 — the base is parked explicitly on every stop path.
# ---------------------------------------------------------------------------
def test_red_b10_base_drive_timed_loop_ends_with_explicit_zero() -> None:
    """``base.drive(seconds=N)`` blocks for N seconds then sends an all-zero stop.

    Catches a drive that returns immediately, that never dispatches the
    requested velocity, or that relies on the host watchdog instead of an
    explicit zero.
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )
    _c4_prime_ready(session)
    client.sends.clear()

    session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.5)

    base_sends = [action for action in client.sends if _c4_has_base_keys(action)]
    assert base_sends, "base.drive must dispatch a base velocity command"
    assert any(float(action["x.vel"]) == pytest.approx(0.2) for action in base_sends), (
        "the requested forward velocity must be dispatched"
    )
    _c4_assert_base_zero(base_sends[-1])
    assert clock.t >= 0.5 - 1e-9, "base.drive must block for the requested duration"

    session.disconnect()


def test_red_b10_base_stop_dispatches_explicit_zero() -> None:
    """``base.stop()`` dispatches a fresh all-zero velocity command."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )
    _c4_prime_ready(session)
    session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.2)
    client.sends.clear()

    session.base.stop()

    assert client.sends, "base.stop() must dispatch an explicit stop command"
    assert _c4_has_base_keys(client.sends[-1]), "the stop must be a base velocity action"
    _c4_assert_base_zero(client.sends[-1])

    session.disconnect()


def test_red_b10_disconnect_after_base_activity_dispatches_explicit_zero() -> None:
    """``disconnect()`` parks the base first when a base velocity was sent."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )
    _c4_prime_ready(session)
    session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.2)
    before = len(client.sends)

    session.disconnect()

    assert len(client.sends) == before + 1, "disconnect must dispatch an explicit base stop"
    _c4_assert_base_zero(client.sends[-1])
    assert client.disconnect_calls == 1


def test_red_b10_disconnect_without_base_activity_sends_no_base_zero() -> None:
    """No base traffic => ``disconnect()`` must not inject a base stop.

    Catches an implementation that unconditionally emits zeros on disconnect
    (which would also mask the "only if a base send happened" contract).
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, ik=ik, adapter=_c2_adapter()
    )
    before = len(client.sends)

    session.disconnect()

    assert len(client.sends) == before, "no base activity must mean no disconnect stop"
    assert not any(_c4_has_base_keys(action) for action in client.sends)


# ---------------------------------------------------------------------------
# B7 / §12.4 — the session presents the FAULT reason and keeps it across reset.
# ---------------------------------------------------------------------------
def _c4_fault_session() -> Any:
    """Run a move that latches FAULT via a sample-id regression; return session+exc."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    holder: dict = {}
    sleeper = _C3ActionSleeper(clock, holder, action=lambda s: s.state.on_feedback(0, settled=True))
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c3_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=5.0,
    )
    holder["session"] = session
    with pytest.raises(control.MotionBlocked) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)
    return session, excinfo.value


def test_red_b7_session_last_reason_surfaces_fault_reason() -> None:
    """``session.last_reason`` presents the FAULT reason, matching the exception."""
    session, exc = _c4_fault_session()

    assert session.state.state is ControlState.FAULT
    reason = session.state.last_reason
    assert reason and "regression" in reason
    assert session.last_reason == reason, "the session must surface the fault reason"
    assert reason in str(exc)

    session.disconnect()


def test_red_b7_session_last_reason_survives_state_reset() -> None:
    """The captured fault reason survives the reset that clears ``state.last_reason``.

    Catches an implementation that lazily mirrors ``state.last_reason`` instead
    of latching the reason at fault time: after reset the state machine's copy
    is gone, so the reason would silently disappear.
    """
    session, _exc = _c4_fault_session()
    reason = session.state.last_reason
    assert reason and session.last_reason == reason

    session.state.reset()

    assert session.state.state is ControlState.STANDBY
    assert session.state.last_reason is None
    assert session.last_reason == reason, "reset must not erase the presented fault reason"

    session.disconnect()


# ---------------------------------------------------------------------------
# B14 — lost/stale feedback aborts motion with FeedbackLost.
# ---------------------------------------------------------------------------
def test_red_b14_stale_feedback_aborts_move_with_feedback_lost() -> None:
    """A stalled feedback stream must raise ``FeedbackLost``, not time out.

    Both sequence signals are frozen after priming, so an implementation keyed
    on either the client counter or the ``_control_status`` side channel must
    detect the loss.  Catches an implementation that keeps commanding using a
    cached observation until ``MotionTimeout``.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=400)
    client = _C4StaleClient(
        observer=lambda _c: _c2_obs(_C2_Q0 if not ik.solve_calls else _C2_Q_STUCK, settled=True),
        fresh_polls=5,
    )
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=3.0,
    )

    with pytest.raises(control.FeedbackLost):
        session.left.move_to(*_C2_TARGET_XYZ)

    session.disconnect()


# ---------------------------------------------------------------------------
# B8 — missing settled/valid, or explicit valid=False, is not arrival.
# ---------------------------------------------------------------------------
def test_red_b8_missing_settled_valid_is_fail_closed_not_arrival() -> None:
    """A ``_control_status`` without ``settled``/``valid`` must not report arrival.

    The arm reaches the position residual on every tick, but the standstill
    criterion is absent; fail-closed means the loop must keep waiting.  Catches
    an implementation defaulting the missing flags to ``True``.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)

    def observer(_c: Any) -> Any:
        observation = _c2_obs(_C2_Q0 if not ik.solve_calls else _C2_Q_ARRIVED, settled=True)
        status = dict(observation["_control_status"])
        status.pop("settled", None)
        status.pop("valid", None)
        observation["_control_status"] = status
        return observation

    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    session.disconnect()


def test_red_b8_explicit_invalid_feedback_is_not_arrival() -> None:
    """An explicit ``valid=False`` while the arm is at the target is not arrival.

    Priming uses valid frames; once the arm reaches the target the frame turns
    ``valid=False`` while still ``settled=True``.  Catches an arrival gate that
    checks only ``settled`` and ignores the validity of the evidence.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)

    def observer(_c: Any) -> Any:
        arrived = bool(ik.solve_calls)
        observation = _c2_obs(_C2_Q_ARRIVED if arrived else _C2_Q0, settled=True)
        status = dict(observation["_control_status"])
        status["valid"] = not arrived
        observation["_control_status"] = status
        return observation

    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    session.disconnect()


# ===========================================================================
# P5 cycle 5 — B11 (move_by, measured-pose base) + B18 (body->arm rotation)
#                + B19 (gripper action path) + B12 (lift mm semantics).
# ---------------------------------------------------------------------------
# Appended to the cycle-1..4 suite; the 39 earlier cases are untouched.
#
# Required behavior (acceptance, 本 cycle 冻结; D011 落点见 control/api.py 的
#   move_to/move_by、LiftHandle、BaseHandle；继承见 docs/ADR.md D17):
#
# * B11 — ``ArmHandle.move_by(dx, dy, dz)`` adds the body-frame increment to the
#   CURRENT measured FK pose (never to the previous commanded target), so a
#   partially reached move cannot accumulate drift.
# * B18 — the body-frame delta is rotated per side by the injected
#   ``r_body_to_arm[side]`` matrix: the same ``move_by`` yields DIFFERENT
#   arm-base-frame targets on the left and right arm.  The default is the
#   identity matrix (pending the B02 calibration closure).
# * B19 — ``ArmHandle.gripper.open()/close()/set(0..100)`` dispatches a single
#   ``arm_{side}_gripper.pos`` action on that side only and never emits the six
#   ``arm_{side}_*.pos`` joint keys.
# * B12 — ``LiftHandle.height()`` reads the measured lift height in mm
#   (``lift_axis.height_mm``, 0 = lowest); ``move_to(mm)`` is an ABSOLUTE target
#   height; ``move_by(delta_mm)`` is RELATIVE to the measured height.
#
# Pinned seams / assumptions to adjudicate (flagged, not silently absorbed):
#   - B18 injects the per-side rotation as ``connect(..., r_body_to_arm={side:
#     3x3})``.  If the intended seam differs, this cycle must be re-planned.
#   - B19 observes the gripper send stream (``client.send_action``) and is
#     deliberately agnostic to whether the dispatch is produced by the
#     controller (``_tick_gripper_command``) or by the session; it constrains
#     only the wire contract.  NOTE: extending the controller's
#     ``_tick_gripper_command`` to actually send breaks the frozen
#     ``test_char_3_*`` (this file) AND the controller ``T-C9.1``; see the
#     cycle-5 report for the conflict.
#   - B12 observes the dispatched ``{"lift_axis.height_mm": ...}`` target and
#     tolerates both a one-shot dispatch and a blocking convergence loop.
#
# Pure logic only: FakeClock / recording sleeper, no real clock / sleep / IO.
# ===========================================================================


def _c5_targets_xyz(ik: Any) -> list[np.ndarray]:
    """Target translations submitted to ``ik.solve`` (precheck waypoints + tick)."""
    targets: list[np.ndarray] = []
    for call in ik.solve_calls:
        pose = None
        if "target_pose" in call["kwargs"]:
            pose = np.asarray(call["kwargs"]["target_pose"], dtype=float)
        elif len(call["args"]) >= 2:
            pose = np.asarray(call["args"][1], dtype=float)
        if pose is not None and pose.shape == (4, 4):
            targets.append(np.copy(pose[:3, 3]))
    return targets


def _c5_max_target_x(ik: Any) -> float:
    xs = [float(target[0]) for target in _c5_targets_xyz(ik)]
    assert xs, "the move must have submitted at least one IK target"
    return max(xs)


def _c5_has_target(ik: Any, xyz: Any, abs_tol: float = 1e-4) -> bool:
    wanted = np.asarray(xyz, dtype=float)
    return any(np.linalg.norm(target - wanted) <= abs_tol for target in _c5_targets_xyz(ik))


def _c5_prime_ready(session: Any, client: Any) -> None:
    """Drive the shared state machine into READY and keep the next poll new.

    The manually fed sample ids 1..3 occupy the machine's seen set; ``_sample``
    is advanced so that a path which polls feedback (e.g. via the controller)
    never observes a sample-id regression.
    """
    for sample_id in (1, 2, 3):
        session.state.on_feedback(sample_id, settled=True)
    session.state.resume()
    assert session.state.state is ControlState.READY
    client._sample = 3


def _c5_prime_active_base_lift(session: Any) -> None:
    """Drive the shared state machine into ACTIVE(BASE_LIFT)."""
    from alohamini_cartesian.control.commands import LiftCommand

    for sample_id in (1, 2, 3):
        session.state.on_feedback(sample_id, settled=True)
    session.state.resume()
    session.state.handle(LiftCommand(delta_mm=0.0))
    assert session.state.state is ControlState.ACTIVE
    assert session.state.active_stage is Stage.BASE_LIFT


def _c5_lift_obs(height_mm: float, *, settled: bool = True) -> dict:
    """Observation carrying arm state, lift metadata and a measured lift height."""
    observation = dict(_c2_motor_pos(_C2_Q0, "left"))
    observation["lift_axis.height_mm"] = float(height_mm)
    metadata = dict(_c2_metadata("left"))
    metadata["lift_axis"] = {"soft_min_mm": 0.0, "soft_max_mm": 600.0}
    observation["_robot_metadata"] = metadata
    observation["_control_status"] = {
        "schema_version": 1,
        "host_session_id": "c5-session",
        "sample_sequence": 1,
        "settled": bool(settled),
        "valid": True,
        "joint_holds": {},
        "gripper_holds": {},
    }
    return observation


def _c5_lift_observer(initial_mm: float) -> Any:
    """Observer that echoes the last dispatched lift target so a loop converges."""

    def observer(client: Any) -> Any:
        height = float(initial_mm)
        for action in reversed(client.sends):
            if "lift_axis.height_mm" in action:
                height = float(action["lift_axis.height_mm"])
                break
        return _c5_lift_obs(height)

    return observer


def _c5_dispatched_lift_heights(client: Any) -> list[float]:
    return [
        float(action["lift_axis.height_mm"])
        for action in client.sends
        if "lift_axis.height_mm" in action
    ]


# ---------------------------------------------------------------------------
# B11 — move_by is relative to the CURRENT MEASURED pose (anti-drift).
# ---------------------------------------------------------------------------
def test_red_b11_move_by_is_relative_to_measured_pose() -> None:
    """``move_by`` composes the increment with the measured current FK pose.

    Current pose maps to x=0.010; ``move_by(dx=0.006)`` must request x=0.016.
    The arm is held fixed so the loop cannot arrive early; the submitted IK
    target is inspected directly (it stays observable even in dry-run mode).
    Catches an implementation that treats the increment as an absolute target
    (x=0.006) or that ignores the increment entirely (x=0.010).
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    session = _c3_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_by(dx=0.006, dy=0.0, dz=0.0)

    max_x = _c5_max_target_x(ik)
    assert max_x == pytest.approx(0.016, abs=1e-4), f"expected measured(0.010)+delta; got x={max_x}"
    assert abs(max_x - 0.006) > 1e-3, "move_by must not treat the delta as an absolute target"
    session.disconnect()


def test_red_b11_move_by_uses_measured_pose_not_previous_target() -> None:
    """``move_by`` must base the increment on the measured pose, not the last target.

    Phase 1 commands x=0.0165 but the arm only reaches x=0.012 (measured).  The
    phase-2 ``move_by(0.003)`` must therefore target x=0.015 (= 0.012 + 0.003),
    not x=0.0195 (= 0.0165 + 0.003).  Also catches a base taken from the last
    accepted joint command (x=0.0105 -> 0.0135).
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    baseline = {"n": None}

    def observer(c: Any) -> Any:
        reached = baseline["n"] is not None and len(c.sends) > baseline["n"]
        return _c2_obs(_C2_Q_ARRIVED if reached else _C2_Q0, settled=True)

    client = _C2FakeClient(observer=observer)
    session = _c3_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=1.0,
    )
    baseline["n"] = len(client.sends)

    session.left.move_to(0.0165, -0.020, 0.030)
    ik.solve_calls.clear()

    session.left.move_by(dx=0.003, dy=0.0, dz=0.0)

    max_x = _c5_max_target_x(ik)
    assert max_x == pytest.approx(0.015, abs=2e-4), (
        f"move_by must base on the measured pose 0.012; observed target x={max_x}"
    )
    assert abs(max_x - 0.0195) > 1e-3, "move_by must not base on the previous target"
    session.disconnect()


# ---------------------------------------------------------------------------
# B18 — injected per-side body->arm rotation changes the arm-frame target.
# ---------------------------------------------------------------------------
def test_red_b18_default_rotation_is_identity_for_both_arms() -> None:
    """Default (no injection): the same body delta gives the same target on both arms.

    Catches a framework that bakes in a non-identity default (e.g. a mirrored
    right arm) before the B02 calibration closes the mapping.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    left_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    right_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0))
    session = _c4_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik_factory=lambda side: left_ik if side == "left" else right_ik,
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_by(0.006, 0.0, 0.0)
    left_max = _c5_max_target_x(left_ik)
    with pytest.raises(control.MotionTimeout):
        session.right.move_by(0.006, 0.0, 0.0)
    right_max = _c5_max_target_x(right_ik)

    assert left_max == pytest.approx(0.016, abs=1e-4)
    assert right_max == pytest.approx(0.016, abs=1e-4)
    assert left_max == pytest.approx(right_max, abs=1e-4), (
        "default identity: same body delta, same target"
    )
    session.disconnect()


def test_red_b18_injected_rotation_yields_side_specific_targets() -> None:
    """A non-trivial injected rotation is applied per side to the body delta.

    Left uses a +90 deg rotation about Z, right the identity, with the same
    body delta ``(0.006, 0, 0)``.  Left must target y = -0.020 + 0.006 while
    right targets x = 0.010 + 0.006; the two arm-frame targets must differ.
    Catches an implementation that ignores ``r_body_to_arm`` (both sides then
    land on the unrotated target).
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    left_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    right_ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0))
    rotate_z_90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    session = _c4_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik_factory=lambda side: left_ik if side == "left" else right_ik,
        move_timeout_s=0.5,
        r_body_to_arm={"left": rotate_z_90, "right": np.eye(3)},
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_by(0.006, 0.0, 0.0)
    with pytest.raises(control.MotionTimeout):
        session.right.move_by(0.006, 0.0, 0.0)

    left_target = (0.010, -0.014, 0.030)
    right_target = (0.016, -0.020, 0.030)
    assert _c5_has_target(left_ik, left_target), "left must apply R_body_to_arm"
    assert not _c5_has_target(left_ik, right_target), "left must not use the unrotated target"
    assert _c5_has_target(right_ik, right_target), (
        "right (identity) must land on the unrotated target"
    )
    assert not _c5_has_target(right_ik, left_target), "right must not use the left rotation"

    start = np.array([0.010, -0.020, 0.030])
    left_delta = np.array(left_target) - start
    assert float(np.linalg.norm(left_delta)) == pytest.approx(0.006, abs=1e-4), (
        "a pure rotation must preserve the delta magnitude"
    )
    session.disconnect()


# ---------------------------------------------------------------------------
# B19 — gripper open/close/set dispatch only that side's gripper key.
# ---------------------------------------------------------------------------
def test_red_b19_gripper_set_sends_side_gripper_key_only() -> None:
    """``gripper.set(50)`` dispatches ``arm_left_gripper.pos`` and no joint keys."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0))
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik=ik,
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.left.gripper.set(50)

    assert client.sends, "gripper.set must dispatch a gripper action"
    action = client.sends[-1]
    assert "arm_left_gripper.pos" in action, f"missing gripper key: {action!r}"
    assert float(action["arm_left_gripper.pos"]) == pytest.approx(50.0)
    joint_keys = [
        key for key in action if key.startswith("arm_left_") and key != "arm_left_gripper.pos"
    ]
    assert joint_keys == [], f"gripper action must not carry joint keys: {joint_keys!r}"
    assert not any(key.startswith("arm_right_") for key in action), (
        "gripper action must be single-sided"
    )
    session.disconnect()


def test_red_b19_gripper_open_and_close_are_distinct_in_range() -> None:
    """``open()`` and ``close()`` command distinct in-range gripper positions."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0))
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik=ik,
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.left.gripper.open()
    open_action = client.sends[-1]
    session.left.gripper.close()
    close_action = client.sends[-1]

    assert "arm_left_gripper.pos" in open_action
    assert "arm_left_gripper.pos" in close_action
    open_pos = float(open_action["arm_left_gripper.pos"])
    close_pos = float(close_action["arm_left_gripper.pos"])
    assert 0.0 <= open_pos <= 100.0 and 0.0 <= close_pos <= 100.0, (
        "gripper position must be in [0, 100]"
    )
    assert open_pos != pytest.approx(close_pos), "open and close must command different positions"
    session.disconnect()


def test_red_b19_right_gripper_is_side_isolated() -> None:
    """``right.gripper.open()`` only emits ``arm_right_gripper.pos``."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0))
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik=ik,
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.right.gripper.open()

    assert client.sends, "right gripper.open must dispatch a gripper action"
    action = client.sends[-1]
    assert "arm_right_gripper.pos" in action
    assert not any(key.startswith("arm_left_") for key in action), (
        "right gripper must not leak left keys"
    )
    joint_keys = [
        key for key in action if key.startswith("arm_right_") and key != "arm_right_gripper.pos"
    ]
    assert joint_keys == [], f"gripper action must not carry joint keys: {joint_keys!r}"
    session.disconnect()


def test_red_b19_gripper_refused_during_base_lift() -> None:
    """A gripper command while ACTIVE(BASE_LIFT) is refused with ``MotionBlocked``."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0))
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c3_adapter_both(),
        ik=ik,
    )
    _c5_prime_active_base_lift(session)
    client.sends.clear()

    with pytest.raises(control.MotionBlocked):
        session.left.gripper.open()

    assert session.state.state is ControlState.BLOCKED
    assert client.sends == [], "a refused gripper call must dispatch nothing"
    session.disconnect()


# ---------------------------------------------------------------------------
# B12 — lift height reads measured mm; move_to absolute; move_by relative.
# ---------------------------------------------------------------------------
def test_red_b12_lift_height_reads_measured_mm() -> None:
    """``lift.height()`` returns the measured ``lift_axis.height_mm`` value."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=lambda _c: _c5_lift_obs(250.0))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)

    assert session.lift.height() == pytest.approx(250.0)
    session.disconnect()


def test_red_b12_lift_move_to_is_absolute_mm() -> None:
    """``lift.move_to(300)`` dispatches an absolute 300 mm target (not measured+300)."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=_c5_lift_observer(250.0))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.lift.move_to(300.0)

    heights = _c5_dispatched_lift_heights(client)
    assert heights, "lift.move_to must dispatch a lift_axis.height_mm target"
    assert heights[0] == pytest.approx(300.0), "move_to(mm) is an absolute height"
    assert heights[0] != pytest.approx(550.0), "move_to(mm) must not add the measured height"
    session.disconnect()


def test_red_b12_lift_move_by_is_relative_mm() -> None:
    """``lift.move_by(-50)`` from a measured 250 mm dispatches 200 mm (relative)."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=_c5_lift_observer(250.0))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.lift.move_by(-50.0)

    heights = _c5_dispatched_lift_heights(client)
    assert heights, "lift.move_by must dispatch a lift_axis.height_mm target"
    assert heights[0] == pytest.approx(200.0), "move_by(delta) is relative to the measured height"
    assert heights[0] != pytest.approx(-50.0), "move_by(delta) must not send the raw delta"
    session.disconnect()


# ===========================================================================
# P5 cycle 6 — B20 (lift stage release) + B21 (move_by observation failure).
# ---------------------------------------------------------------------------
# Appended to the cycle-1..5 suite; the 50 earlier cases are untouched.
#
# Required behavior (acceptance, 本 cycle 冻结):
#
# * B20 — ``LiftHandle.move_to`` / ``move_by`` dispatch the lift target and
#   then RELEASE the ``BASE_LIFT`` stage through the state machine's
#   ``ReleaseCommand`` path (ACTIVE -> SETTLING), so a subsequent ARM motion
#   can re-activate normally.  It must NOT use ``StopCommand`` (which would
#   drop the machine to STANDBY) and must NOT leave the machine stuck in
#   ACTIVE(BASE_LIFT) (the next ARM move would then be a cross-stage conflict
#   -> BLOCKED and would dispatch nothing).
# * B21 — ``ArmHandle.move_by`` obtains its FK baseline from a fresh
#   observation.  When that observation cannot be read (transport raises, or
#   the frame is unusable because ``_robot_metadata`` is absent), the failure
#   is reported as ``FeedbackLost`` rather than ``MotionNotEnabled``.  In
#   dry-run mode the observation is STILL required for the FK baseline; only
#   the error type is corrected.
# ===========================================================================


class _C6ObservationFailureClient(_C2FakeClient):
    """Cycle-6 seam: a client whose observation read can be made to fail.

    ``mode``:

    * ``"raise"`` — ``get_observation`` raises (transport lost).
    * ``"no_metadata"`` — the frame omits ``_robot_metadata`` so no FK
      baseline can be derived.
    """

    def __init__(self, *, mode: str) -> None:
        super().__init__(observer=lambda _c: _c5_lift_obs(250.0))
        self.mode = mode
        self.get_observation_calls = 0

    def get_observation(self, **kwargs: Any) -> Any:
        self.get_observation_calls += 1
        if self.mode == "raise":
            raise RuntimeError("synthetic observation transport failure")
        observation = super().get_observation(**kwargs)
        if self.mode == "no_metadata":
            observation.pop("_robot_metadata", None)
        return observation


def _c6_arm_lift_obs(q_deg: Any, height_mm: float, *, settled: bool = True) -> dict:
    """Observation carrying a scripted arm pose AND a measured lift height."""
    observation = dict(_c2_motor_pos(q_deg, "left"))
    observation["lift_axis.height_mm"] = float(height_mm)
    metadata = dict(_c2_metadata("left"))
    metadata["lift_axis"] = {"soft_min_mm": 0.0, "soft_max_mm": 600.0}
    observation["_robot_metadata"] = metadata
    observation["_control_status"] = {
        "schema_version": 1,
        "host_session_id": "c6-session",
        "sample_sequence": 1,
        "settled": bool(settled),
        "valid": True,
        "joint_holds": {},
        "gripper_holds": {},
    }
    return observation


# ---------------------------------------------------------------------------
# B20 — lift dispatches the target and releases BASE_LIFT (not stop / not stuck).
# ---------------------------------------------------------------------------
def test_red_c6_lift_move_to_dispatches_then_releases_base_lift_stage() -> None:
    """``lift.move_to`` sends the target, then releases BASE_LIFT via ReleaseCommand.

    Catches an implementation that leaves the machine stuck in
    ACTIVE(BASE_LIFT) (the next ARM motion then becomes a cross-stage
    conflict) or that uses ``StopCommand`` (the machine lands in STANDBY,
    which is explicitly the wrong release path).
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=_c5_lift_observer(250.0))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.lift.move_to(300.0)

    heights = _c5_dispatched_lift_heights(client)
    assert heights and heights[0] == pytest.approx(300.0), "the lift target must be dispatched"
    assert session.state.active_stage is None, "lift must release the BASE_LIFT stage"
    assert session.state.state is ControlState.SETTLING, (
        "release must take the ReleaseCommand path (ACTIVE -> SETTLING), not a stop (STANDBY)"
    )
    session.disconnect()


def test_red_c6_lift_move_by_dispatches_relative_target_then_releases() -> None:
    """``lift.move_by`` sends the relative target, then releases BASE_LIFT.

    Same release contract as ``move_to``: relative target dispatched, machine
    released through SETTLING rather than stuck ACTIVE or stopped to STANDBY.
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=_c5_lift_observer(250.0))
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.lift.move_by(-50.0)

    heights = _c5_dispatched_lift_heights(client)
    assert heights and heights[0] == pytest.approx(200.0), (
        "move_by must dispatch the relative target"
    )
    assert session.state.active_stage is None, "lift must release the BASE_LIFT stage"
    assert session.state.state is ControlState.SETTLING, (
        "release must take the ReleaseCommand path (ACTIVE -> SETTLING), not a stop (STANDBY)"
    )
    session.disconnect()


def test_red_c6_arm_motion_activates_after_lift_release() -> None:
    """After a lift release, a subsequent ARM move re-activates and dispatches.

    Catches the no-release regression: the machine would stay
    ACTIVE(BASE_LIFT), the ARM move would hit the cross-stage conflict, and no
    arm action would ever be dispatched.
    """
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)

    def observer(c: Any) -> Any:
        height = 250.0
        for action in reversed(c.sends):
            if "lift_axis.height_mm" in action:
                height = float(action["lift_axis.height_mm"])
                break
        return _c6_arm_lift_obs(_C2_Q_ARRIVED if ik.solve_calls else _C2_Q0, height, settled=True)

    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=_c2_adapter(),
        ik=ik,
        move_timeout_s=1.0,
    )
    _c5_prime_ready(session, client)

    session.lift.move_to(300.0)
    client.sends.clear()

    session.left.move_to(*_C2_TARGET_XYZ)

    arm_actions = [
        action for action in client.sends if any(key.startswith("arm_left_") for key in action)
    ]
    assert arm_actions, "the ARM must re-activate and dispatch after the lift releases BASE_LIFT"
    session.disconnect()


# ---------------------------------------------------------------------------
# B21 — ``move_by`` observation failure is FeedbackLost (not MotionNotEnabled).
# ---------------------------------------------------------------------------
def test_red_c6_move_by_read_failure_is_feedback_lost() -> None:
    """A failing observation read in ``move_by`` surfaces as ``FeedbackLost``.

    Catches a transport read failure being classified as ``MotionNotEnabled``
    (an enable-gate error) instead of a lost-feedback error.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C6ObservationFailureClient(mode="raise")
    session = _c4_connect(
        dry_run=False, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)
    client.get_observation_calls = 0

    with pytest.raises(control.FeedbackLost):
        session.left.move_by(dx=0.001, dy=0.0, dz=0.0)

    assert client.get_observation_calls >= 1, "move_by must have attempted the observation read"
    session.disconnect()


def test_red_c6_dry_run_move_by_still_reads_and_reports_feedback_lost() -> None:
    """dry-run ``move_by`` still needs the observation FK baseline.

    A failing read in dry-run must surface as ``FeedbackLost``; the observation
    is not optional just because no action is sent.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C6ObservationFailureClient(mode="raise")
    session = _c4_connect(
        dry_run=True, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)
    client.get_observation_calls = 0

    with pytest.raises(control.FeedbackLost):
        session.left.move_by(dx=0.001, dy=0.0, dz=0.0)

    assert client.get_observation_calls >= 1, (
        "dry_run move_by still needs the observation FK baseline"
    )
    session.disconnect()


def test_red_c6_move_by_missing_metadata_is_feedback_lost() -> None:
    """An observation without ``_robot_metadata`` is an unusable FK baseline.

    Catches the missing-metadata branch being reported as ``MotionNotEnabled``
    rather than ``FeedbackLost``.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C6ObservationFailureClient(mode="no_metadata")
    session = _c4_connect(
        dry_run=True, client=client, clock=clock, sleeper=sleeper, adapter=_c2_adapter(), ik=ik
    )
    _c5_prime_ready(session, client)

    with pytest.raises(control.FeedbackLost):
        session.left.move_by(dx=0.001, dy=0.0, dz=0.0)
    session.disconnect()


# ===========================================================================
# P5 cycle 7 — B1 (non-arm handles self-prime from STANDBY) + B2 (dry-run
#               zero-send regression guards) + B3 (arrival gate fail-closed on
#               missing ``valid``).
# ---------------------------------------------------------------------------
# Appended to the cycle-1..6 suite; the 56 earlier cases are untouched.
#
# Required behavior (acceptance, 本 cycle 冻结):
#
# * B1 — ``GripperHandle`` / ``LiftHandle`` / ``BaseHandle`` must re-qualify the
#   shared state machine from STANDBY (the same 3-settled-sample resume gate the
#   arm path satisfies inside its move loop), so the FIRST non-arm operation of
#   a session (and after ``stop()``) succeeds instead of raising
#   ``MotionBlocked("machine is not READY")``.  Priming may live in a
#   session-level helper or be pushed into each handle entry; these tests only
#   observe public handle behavior.
# * T11 — priming is bounded: feedback keeps arriving but never reports settled
#   -> ``MotionTimeout`` within ``move_timeout_s`` (A3).
# * B2 — ``dry_run=True`` never calls ``send_action`` on any non-arm path (V09):
#   gripper open/close/set, lift move_to/move_by, base drive/stop, disconnect.
#   These are regression guards (currently GREEN).
# * B3 — the arrival standstill gate is fail-closed: a frame that carries
#   ``settled=True`` but OMITS ``valid`` must NOT be treated as arrival
#   (V05).
#
# Healthy-frame convention: ``_c2_obs`` / ``_c3_both_obs`` / ``_c5_lift_obs`` /
# ``_c6_arm_lift_obs`` now advertise ``valid: True`` explicitly.  Tests that
# need the absent-``valid`` path delete the key from a dict copy (never from the
# builder).
# ===========================================================================


def _c7_real_session(*, observer: Any, adapter: Any = None, move_timeout_s: float = 10.0) -> Any:
    """Fresh non-dry-run session with the standard fake seams."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter() if adapter is None else adapter,
        move_timeout_s=move_timeout_s,
    )
    return session, client, clock, sleeper


def _c7_dry_session(*, observer: Any, adapter: Any = None) -> Any:
    """Fresh dry-run session with the standard fake seams."""
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter() if adapter is None else adapter,
    )
    return session, client, clock, sleeper


def _c7_gripper_actions(client: Any) -> list[dict]:
    return [action for action in client.sends if "arm_left_gripper.pos" in action]


# ---------------------------------------------------------------------------
# B1 — fresh STANDBY: each non-arm handle self-primes and succeeds.
# ---------------------------------------------------------------------------
def test_red_c7_gripper_self_primes_from_standby() -> None:
    """The first ``gripper.open()`` of a session must succeed, not raise.

    The session starts in STANDBY with healthy settled feedback available on
    the wire.  The handle must consume that feedback through the resume gate
    (3 consecutive settled new samples) before dispatching, exactly as the arm
    ``move_to`` loop does.  Catches a gripper path that dispatches
    ``session.state.handle(...)`` directly from STANDBY (``MotionBlocked`` at
    commands.py:438).
    """
    session, client, _clock, _sleeper = _c7_real_session(
        observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0), adapter=_c3_adapter_both()
    )
    client.sends.clear()

    session.left.gripper.open()

    actions = _c7_gripper_actions(client)
    assert actions, "a primed gripper.open must dispatch arm_left_gripper.pos"
    assert float(actions[-1]["arm_left_gripper.pos"]) == pytest.approx(100.0)
    assert not any(key.startswith("arm_right_") for key in actions[-1]), (
        "the left gripper action must stay single-sided"
    )
    session.disconnect()


def test_red_c7_lift_self_primes_from_standby() -> None:
    """The first ``lift.move_to`` of a session must succeed, not raise.

    Catches a lift path that reaches ``state.handle(LiftCommand(...))`` while
    the machine is still STANDBY.
    """
    session, client, _clock, _sleeper = _c7_real_session(observer=lambda _c: _c5_lift_obs(250.0))
    client.sends.clear()

    session.lift.move_to(300.0)

    heights = _c5_dispatched_lift_heights(client)
    assert heights, "a primed lift.move_to must dispatch lift_axis.height_mm"
    assert heights[0] == pytest.approx(300.0)
    session.disconnect()


def test_red_c7_base_self_primes_from_standby() -> None:
    """The first ``base.drive`` of a session must succeed, not raise.

    Catches a base path that reaches ``state.handle(BaseCommand(...))`` while
    the machine is still STANDBY.  A successful drive must still dispatch the
    requested velocity and end with an explicit all-zero stop.
    """
    session, client, clock, _sleeper = _c7_real_session(observer=lambda _c: _c2_obs(_C2_Q0))
    client.sends.clear()

    session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.2)

    base_sends = [action for action in client.sends if _c4_has_base_keys(action)]
    assert base_sends, "a primed base.drive must dispatch a base velocity"
    assert any(float(action["x.vel"]) == pytest.approx(0.2) for action in base_sends)
    _c4_assert_base_zero(base_sends[-1])
    assert clock.t >= 0.2 - 1e-9
    session.disconnect()


def test_red_c7_stop_then_resume_gripper_re_qualifies() -> None:
    """After ``stop()`` the next non-arm op must re-qualify on NEW samples.

    ``stop()`` returns the machine to STANDBY and clears the settle window.
    ``_c5_prime_ready`` advances the fake client's sample counter to 3 so the
    priming path observes genuinely new ids after the stop.  Catches priming
    that caches/forces READY or that reuses the pre-stop window instead of
    counting fresh settled samples.
    """
    session, client, _clock, _sleeper = _c7_real_session(
        observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0)
    )
    _c5_prime_ready(session, client)
    session.stop()
    assert session.state.state is ControlState.STANDBY
    client.sends.clear()

    session.left.gripper.close()

    actions = _c7_gripper_actions(client)
    assert actions, "a re-qualified gripper.close must dispatch arm_left_gripper.pos"
    assert float(actions[-1]["arm_left_gripper.pos"]) == pytest.approx(0.0)
    session.disconnect()


def test_red_c7_priming_with_never_settled_feedback_times_out() -> None:
    """Feedback in stream but never settled -> bounded priming raises MotionTimeout.

    The wire keeps producing new samples (``_C2FakeClient`` increments
    ``sample_sequence``) but ``settled=False`` forever, so the 3-settled-sample
    gate can never open.  Per the A3 ruling priming is bounded by
    ``move_timeout_s`` and must surface ``MotionTimeout`` -- never force READY,
    never dispatch.  Catches unbounded priming (hang) and gate bypass.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    session, client, _clock, _sleeper = _c7_real_session(
        observer=lambda _c: _c2_obs(_C2_Q0, settled=False), move_timeout_s=0.5
    )

    with pytest.raises(control.MotionTimeout):
        session.left.gripper.open()

    dispatched = {key for action in client.sends for key in action}
    assert "arm_left_gripper.pos" not in dispatched, (
        "a never-settled priming must dispatch no gripper action"
    )
    session.disconnect()


# ---------------------------------------------------------------------------
# B2 + B1 — fresh STANDBY, dry-run: no raise and no send on any non-arm path.
# ---------------------------------------------------------------------------
def test_red_c7_dry_run_fresh_standby_non_arm_paths_stay_silent() -> None:
    """dry-run from fresh STANDBY: every non-arm op self-primes and sends nothing.

    Combines B1 (self-qualification) with V09 (dry_run has zero
    ``send_action``).  Each op uses its own fresh session because a gripper
    command leaves the machine in ACTIVE(ARM) (this test deliberately does not
    bind that post-state) and a lift command leaves SETTLING, so mixing them in
    one session would test the cross-stage interlock instead of priming.
    """
    gripper_session, gripper_client, _c, _s = _c7_dry_session(observer=lambda _c: _c2_obs(_C2_Q0))
    gripper_session.left.gripper.open()
    gripper_session.left.gripper.close()
    gripper_session.left.gripper.set(50.0)
    assert gripper_client.sends == []
    gripper_session.disconnect()

    lift_session, lift_client, _c, _s = _c7_dry_session(observer=lambda _c: _c5_lift_obs(250.0))
    lift_session.lift.move_to(300.0)
    assert lift_client.sends == []
    lift_session.disconnect()

    base_session, base_client, _c, _s = _c7_dry_session(observer=lambda _c: _c2_obs(_C2_Q0))
    base_session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.2)
    base_session.base.stop()
    assert base_client.sends == []
    base_session.disconnect()


# ---------------------------------------------------------------------------
# B3 — arrival gate is fail-closed when ``valid`` is absent.
# ---------------------------------------------------------------------------
def test_red_c7_missing_valid_only_is_fail_closed_not_arrival() -> None:
    """``settled=True`` with the ``valid`` key absent must NOT be arrival.

    The arm reaches the position residual on every tick, but the arriving frame
    carries no ``valid`` field.  Fail-closed means ``move_to`` must keep waiting
    and time out.  Catches the ``control_status.get("valid", True)`` fail-open
    default at api.py:339.  Companion to the existing :1914 case, which is
    intercepted by the missing ``settled`` flag and never exercises this path.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    ik = _C2ScriptedIK(solve_fn=_c2_step_solver)

    def observer(_c: Any) -> Any:
        arrived = bool(ik.solve_calls)
        observation = _c2_obs(_C2_Q_ARRIVED if arrived else _C2_Q0, settled=True)
        if arrived:
            status = dict(observation["_control_status"])
            status.pop("valid", None)
            observation["_control_status"] = status
        return observation

    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock)
    client = _C2FakeClient(observer=observer)
    session = _c4_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        ik=ik,
        adapter=_c2_adapter(),
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    session.disconnect()


# ---------------------------------------------------------------------------
# B2 regression guards — dry-run zero-send at each non-arm send site.
# Pre-primed to READY so the command actually reaches the guarded send site;
# without the guard the send would be observable.
# ---------------------------------------------------------------------------
def test_red_c7_dry_run_gripper_never_sends() -> None:
    """dry-run gripper open/close/set must dispatch nothing (api.py:100 guard)."""
    session, client, _clock, _sleeper = _c7_dry_session(
        observer=lambda _c: _c3_both_obs(_C2_Q0, _C2_Q0)
    )
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.left.gripper.open()
    session.left.gripper.close()
    session.left.gripper.set(50.0)

    assert client.sends == [], "dry-run gripper commands must never send_action"
    session.disconnect()


def test_red_c7_dry_run_lift_never_sends() -> None:
    """dry-run lift move_to/move_by must dispatch nothing (api.py:438 guard)."""
    session, client, _clock, _sleeper = _c7_dry_session(observer=_c5_lift_observer(250.0))
    _c5_prime_ready(session, client)
    client.sends.clear()
    session.lift.move_to(300.0)
    assert client.sends == [], "dry-run lift.move_to must never send_action"
    session.disconnect()

    session2, client2, _c, _s = _c7_dry_session(observer=_c5_lift_observer(250.0))
    _c5_prime_ready(session2, client2)
    client2.sends.clear()
    session2.lift.move_by(-50.0)
    assert client2.sends == [], "dry-run lift.move_by must never send_action"
    session2.disconnect()


def test_red_c7_dry_run_base_drive_and_stop_never_send() -> None:
    """dry-run base drive/stop must dispatch nothing (api.py:484/:490/:501)."""
    session, client, clock, sleeper = _c7_dry_session(observer=lambda _c: _c2_obs(_C2_Q0))
    _c5_prime_ready(session, client)
    client.sends.clear()

    session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.2)
    session.base.stop()

    assert client.sends == [], "dry-run base drive/stop must never send_action"
    assert clock.t >= 0.2 - 1e-9, "the drive loop must still run (reached the guarded send site)"
    assert len(sleeper.sleeps) >= 1
    session.disconnect()


def test_red_c7_dry_run_disconnect_never_sends() -> None:
    """dry-run disconnect after base activity must dispatch nothing (api.py:608)."""
    session, client, _clock, _sleeper = _c7_dry_session(observer=lambda _c: _c2_obs(_C2_Q0))
    _c5_prime_ready(session, client)
    session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.2)
    client.sends.clear()

    session.disconnect()

    assert client.sends == [], "dry-run disconnect must never send_action"
    assert client.disconnect_calls == 1


# ===========================================================================
# P5 cycle 2 — B4 (production-form settle/valid injection), B5 (validation
#                order), B6 (priming shared ingest), B7 (clock/sleeper guard).
#
# Cycle-2 acceptance (design §6 rule 5 / §12.4:323, U1-U6 rulings):
#
# * B4 — the SDK resolves the STANDBY->READY settle gate and the arrival gate
#   from the PRODUCTION observation form.  ``AlohaMiniClient.get_observation()``
#   never carries ``_control_status`` (nor ``_robot_metadata``): char_2 freezes
#   that exclusion.  The session must therefore fall back to the client
#   attributes (``latest_control_status`` / ``latest_robot_metadata`` /
#   ``observation_sequence``) for seq/session and DERIVE ``settled``/``valid``
#   from the measured joint motion (velocity / residual).  The protocol v1 host
#   side-channel has no ``settled``/``valid`` fields.
# * B5 — a programming error in a user argument (out-of-range gripper,
#   non-finite lift mm, contradictory base command) must raise ValueError from
#   the frozen command ``__post_init__`` BEFORE any priming host I/O (D017).
# * B6 — priming must share the feedback-ingest path so a host restart
#   (host_session_id change with sample_sequence restarting) discards the old
#   sample-id evidence instead of silently starving (D016).
# * B7 — priming must fail closed with MotionNotEnabled when no clock/sleeper is
#   injected, matching BaseHandle.drive's guard (D015).
#
# No numeric settle threshold is invented (U3): the tests only use the ENDPOINTS
# zero displacement vs large displacement.
# ===========================================================================


# FK is linear: q[:3] * 0.001 -> translation.  Target is (0.0165, -0.020, 0.030)
# and the tolerance is 5 mm, so both probe positions below are IN tolerance while
# their joint-0 delta (2 deg/sample) is a large measured velocity.
_C8_Q_AT_TARGET_A = (14.5, -20.0, 30.0, -40.0, 50.0, -60.0)
_C8_Q_AT_TARGET_B = (16.5, -20.0, 30.0, -40.0, 50.0, -60.0)

# A far-from-target probe used to keep the measured velocity high during priming.
_C8_Q_FAR = (110.0, -20.0, 30.0, -40.0, 50.0, -60.0)


def _c8_connect(
    *,
    dry_run: Any,
    client: Any,
    clock: Any,
    sleeper: Any,
    move_timeout_s: float = 10.0,
    arrival_tolerance_m: float = _C2_TOL,
    adapter: Any = None,
    ik: Any = None,
    **session_kwargs: Any,
) -> Any:
    """Cycle-2 connect helper: shares one scripted IK seam across both arms."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    shared_ik = ik if ik is not None else _C2ScriptedIK(solve_fn=_c2_step_solver)
    return control.connect(
        "host-c8",
        dry_run=dry_run,
        enable_motion=not dry_run,
        client_factory=lambda host, **kwargs: client,
        clock=clock,
        sleeper=sleeper,
        ik_factory=lambda side: shared_ik,
        adapter=_c2_adapter() if adapter is None else adapter,
        control_period_s=_C2_PERIOD,
        arrival_tolerance_m=arrival_tolerance_m,
        move_timeout_s=move_timeout_s,
        **session_kwargs,
    )


class _ProdObsStream:
    """Scripted responder in production observation form (char_2-shaped frames).

    Every response is a char_2-shaped candidate frame: full ``_state_order``
    plus the schema-v2 ``_robot_metadata`` and the schema-v1 ``_control_status``
    side channels.  The fake client consumes both side channels onto its
    attributes and returns an observation that carries NEITHER.  The v1 status
    intentionally omits ``settled``/``valid`` - exactly like the host's
    ``HostControlStatusTracker.record_sample``.
    """

    def __init__(
        self,
        client: Any,
        position_fn: Any,
        *,
        side: str = "left",
        host_session_id: str = "prod-session",
        include_metadata: bool = True,
    ) -> None:
        self.client = client
        self.position_fn = position_fn
        self.side = side
        self.host_session_id = host_session_id
        self.include_metadata = include_metadata
        self.polls = 0

    def __call__(self, request_token: Any, timeout_ms: Any) -> Any:
        self.polls += 1
        q_deg = self.position_fn(self.polls)
        frame = dict.fromkeys(self.client._state_order, 0.0)
        frame.update(_c2_motor_pos(q_deg, self.side))
        if self.include_metadata:
            frame["_robot_metadata"] = _c2_metadata(self.side)
        frame["_control_status"] = {
            "schema_version": 1,
            "host_session_id": self.host_session_id,
            "sample_sequence": self.polls,
            "joint_holds": {},
            "gripper_holds": {},
        }
        return [json.dumps(frame).encode("utf-8")]


def _prod_client(
    position_fn: Any,
    *,
    side: str = "left",
    include_metadata: bool = True,
) -> Any:
    """Build the local fake client driven by a production-form scripted stream."""
    client = _FakeAlohaClient()
    stream = _ProdObsStream(
        client,
        position_fn,
        side=side,
        include_metadata=include_metadata,
    )
    client._receive_observation_response = stream
    return client, stream


# ---------------------------------------------------------------------------
# B4 — production observation form: the settle/valid signal is derived, not read
# from an observation-dict side channel.
# ---------------------------------------------------------------------------
def test_red_b4_priming_satisfied_from_production_observation_form() -> None:
    """A production-form observation with no ``_control_status`` must still prime.

    The measured joints are held exactly constant (zero velocity), so the
    derived settle signal must open the STANDBY->READY gate.  The observation
    returned by the production client never carries ``_control_status``
    nor ``_robot_metadata`` (char_2 contract); the v1 side-channel on the
    attribute has no ``settled``/``valid`` fields.

    Catches an implementation that keeps reading ``settled``/``valid`` from
    ``feedback["_control_status"]``: in production form that dict is absent,
    so priming can never qualify and times out.
    """
    client, _stream = _prod_client(lambda _n: _C2_Q0)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=400)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=0.5,
    )

    probe = client.get_observation()
    assert "_control_status" not in probe
    assert "_robot_metadata" not in probe
    assert client.latest_control_status is not None
    assert "settled" not in client.latest_control_status
    assert "valid" not in client.latest_control_status

    session.left.gripper.open()

    assert session.state.state is ControlState.ACTIVE
    session.disconnect()


def test_red_b4_arrival_satisfied_from_production_observation_form() -> None:
    """The production observation form must let ``move_to`` report arrival once the arm is steady.

    The scripted arm reaches the target residual (0.0045 m <= 5 mm) and then
    holds exactly constant, so both halves of the arrival gate (residual AND
    derived standstill) are satisfiable from the production observation form.
    The move must also return well before the deadline, proving the post-arrival
    release loop no longer depends on ``feedback["_control_status"]`` either.

    Catches an arrival gate that still requires an observation-dict
    ``settled``/``valid`` (MotionTimeout), and one that fixed the arrival check
    but left the release feedback loop keyed on the missing side channel (only
    returns at the deadline).
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda n: _C2_Q0 if n < 3 else _C2_Q_ARRIVED)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=2000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=3.0,
    )

    report = session.left.move_to(*_C2_TARGET_XYZ)

    assert isinstance(report, control.MoveReport)
    assert report.residual_m <= _C2_TOL
    assert clock.t < 3.0, "move_to must return on arrival, not at the deadline"
    session.disconnect()


def test_red_b4_moving_joints_never_settle_in_production_form() -> None:
    """Continuously moving joints must not let priming qualify (fail-closed).

    The arm alternates between two far-apart poses on every sample, so the
    measured velocity is far above any reasonable tolerance.  Priming must stay
    bounded and raise MotionTimeout instead of opening the gate.

    Discriminator: an implementation that hardcodes ``settled=True`` (or that
    only checks "a new frame arrived") passes the success-path tests but fails
    here.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda n: _C2_Q0 if n % 2 else _C8_Q_FAR)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=400)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.gripper.open()

    assert session.state.state is ControlState.STANDBY
    session.disconnect()


def test_red_b4_residual_met_while_moving_is_not_arrival() -> None:
    """Residual inside tolerance while the arm is still moving is NOT arrival.

    Both scripted poses are within 5 mm of the target, but joint 0 oscillates by
    2 deg per sample, so the measured standstill half of the arrival criterion
    is never met.  ``move_to`` must keep waiting and time out.

    Discriminator: an implementation that returns on the position residual alone
    (i.e. defaults the missing ``settled`` to true) returns a MoveReport here and
    fails the expected MotionTimeout.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda n: _C8_Q_AT_TARGET_A if n % 2 else _C8_Q_AT_TARGET_B)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=2000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=0.5,
    )

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    session.disconnect()


# ---------------------------------------------------------------------------
# B5 — argument validation must precede priming (D017).
# ---------------------------------------------------------------------------
def _b5_never_settled_session(*, move_timeout_s: float = 0.5, settled: bool = False) -> Any:
    """Fresh dry-run session whose feedback never settles (priming would block)."""
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=settled))
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=400)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=move_timeout_s,
    )
    return session, client, clock, sleeper


def test_red_b5_gripper_range_error_precedes_priming() -> None:
    """``gripper.set(150)`` is a ValueError before any priming host I/O.

    With feedback that never settles, priming-before-validation surfaces
    MotionTimeout instead of the programming error.  The invalid call must
    reject from the frozen command ``__post_init__`` (D017) before polling.
    """
    session, client, clock, sleeper = _b5_never_settled_session()

    with pytest.raises(ValueError):
        session.left.gripper.set(150.0)

    assert client._sample == 0, "invalid input must be rejected before any feedback poll"
    assert sleeper.sleeps == []
    assert clock.t == 0.0
    session.disconnect()


def test_red_b5_gripper_nonfinite_error_precedes_priming() -> None:
    """``gripper.set(nan)`` is a ValueError before any priming host I/O."""
    session, client, clock, sleeper = _b5_never_settled_session()

    with pytest.raises(ValueError):
        session.left.gripper.set(float("nan"))

    assert client._sample == 0
    assert sleeper.sleeps == []
    assert clock.t == 0.0
    session.disconnect()


def test_red_b5_base_command_error_precedes_priming() -> None:
    """Contradictory/invalid base commands are ValueErrors before priming.

    A zero-duration non-zero velocity is an explicitly rejected contradiction
    (D017); a negative duration is likewise invalid.  Both must be rejected from
    ``BaseCommand.__post_init__`` before the priming loop.
    """
    session, client, _clock, sleeper = _b5_never_settled_session()

    with pytest.raises(ValueError):
        session.base.drive(x=1.0, y=0.0, turn=0.0, seconds=0.0)
    assert client._sample == 0
    assert sleeper.sleeps == []

    with pytest.raises(ValueError):
        session.base.drive(x=0.0, y=0.0, turn=0.0, seconds=-1.0)
    assert client._sample == 0
    assert sleeper.sleeps == []
    session.disconnect()


def test_red_b5_lift_nonfinite_error_precedes_priming() -> None:
    """A non-finite lift target is a ValueError before any priming host I/O."""
    session, client, _clock, sleeper = _b5_never_settled_session()

    with pytest.raises(ValueError):
        session.lift.move_to(float("nan"))
    assert client._sample == 0
    assert sleeper.sleeps == []

    with pytest.raises(ValueError):
        session.lift.move_to(float("inf"))
    assert client._sample == 0
    assert sleeper.sleeps == []
    session.disconnect()


def test_red_b5_valid_gripper_still_primes_and_reaches_active() -> None:
    """Guard: a valid argument still primes first and reaches ACTIVE(ARM).

    Catches an over-correction that skips or reorders priming for valid input.
    """
    session, _client, _clock, _sleeper = _b5_never_settled_session(settled=True)

    session.left.gripper.set(50.0)

    assert session.state.state is ControlState.ACTIVE
    session.disconnect()


# ---------------------------------------------------------------------------
# B6 — priming must share the feedback-ingest reset on host_session_id change.
# ---------------------------------------------------------------------------
class _C8RestartClient(_C2FakeClient):
    """Client whose host session id can be swapped, with per-session sequence."""

    def __init__(self, *, observer: Any, session_id: str = "sess-A") -> None:
        super().__init__(observer=observer)
        self.session_id = session_id

    def get_observation(self, **kwargs: Any) -> Any:
        observation = super().get_observation(**kwargs)
        status = dict(observation["_control_status"])
        status["host_session_id"] = self.session_id
        status["sample_sequence"] = self._sample
        observation["_control_status"] = status
        return observation


class _C8FreezeClient(_C2FakeClient):
    """Client that can pin the sample_sequence while keeping the same session."""

    def __init__(self, *, observer: Any) -> None:
        super().__init__(observer=observer)
        self.freeze = False

    def get_observation(self, **kwargs: Any) -> Any:
        observation = super().get_observation(**kwargs)
        if self.freeze:
            status = dict(observation["_control_status"])
            status["sample_sequence"] = 3
            observation["_control_status"] = status
        return observation


def test_red_b6_priming_recovers_after_host_session_restart() -> None:
    """A host restart must discard the old sample-id evidence, not starve.

    After a long session (high-water far beyond what a restart replays), the
    host restarts: ``host_session_id`` changes and ``sample_sequence`` restarts
    from 1.  Priming must route feedback through the shared ingest path so the
    session change resets the state machine and the restarted samples qualify.

    Catches priming that feeds ``state.on_feedback`` directly: the restarted ids
    are still below the high-water mark and already seen, so the settle window
    never fills and priming times out.
    """
    client = _C8RestartClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=20000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=0.5,
    )

    session.left.gripper.open()
    assert session.state.state is ControlState.ACTIVE
    session.stop()
    assert session.state.state is ControlState.STANDBY

    # Long sess-A history: the seen high-water sits far above the replay range.
    # ``stop()`` then clears the settle window without clearing the seen ids, so
    # the restart can only qualify if the session change discards old evidence.
    for sample_id in range(4, 2001):
        session.state.on_feedback(sample_id, settled=True)
    session.stop()
    assert session.state.state is ControlState.STANDBY

    client.session_id = "sess-B"
    client._sample = 0

    session.left.gripper.close()

    assert session.state.state is ControlState.ACTIVE
    assert session.state.state is not ControlState.FAULT
    session.disconnect()


def test_red_b6_same_session_frozen_sequence_is_fail_closed() -> None:
    """Guard: an unchanged host session with a frozen sequence must not requalify.

    After ``stop()`` in the SAME session the sequence is pinned, so no genuinely
    new sample arrives.  Priming must fail closed (MotionTimeout/FeedbackLost)
    rather than reset its evidence on every poll and re-count a repeated id.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client = _C8FreezeClient(observer=lambda _c: _c2_obs(_C2_Q0, settled=True))
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=400)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=0.5,
        feedback_timeout_s=1.0,
    )

    session.left.gripper.open()
    session.stop()
    client.freeze = True

    with pytest.raises((control.MotionTimeout, control.FeedbackLost)):
        session.left.gripper.close()

    client.freeze = False
    session.disconnect()


# ---------------------------------------------------------------------------
# B7 — priming entry guards a missing clock/sleeper like BaseHandle.drive.
# ---------------------------------------------------------------------------
def test_red_b7_gripper_priming_without_clock_raises_motion_not_enabled() -> None:
    """Priming with no injected clock must fail closed, not raise TypeError."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0))
    session = _c8_connect(dry_run=True, client=client, clock=None, sleeper=None)

    with pytest.raises(control.MotionNotEnabled):
        session.left.gripper.open()

    session.disconnect()


def test_red_b7_lift_priming_without_clock_raises_motion_not_enabled() -> None:
    """Lift priming with no injected clock must fail closed, not raise TypeError."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    client = _C2FakeClient(observer=lambda _c: _c5_lift_obs(250.0))
    session = _c8_connect(dry_run=True, client=client, clock=None, sleeper=None)

    with pytest.raises(control.MotionNotEnabled):
        session.lift.move_to(300.0)

    session.disconnect()


def test_red_b7_base_drive_without_clock_guard_unchanged() -> None:
    """Guard: BaseHandle.drive already fails closed without a clock/sleeper."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    client = _C2FakeClient(observer=lambda _c: _c2_obs(_C2_Q0))
    session = _c8_connect(dry_run=True, client=client, clock=None, sleeper=None)

    with pytest.raises(control.MotionNotEnabled):
        session.base.drive(x=0.2, y=0.0, turn=0.0, seconds=0.2)

    session.disconnect()


def test_red_b7_priming_with_clock_does_not_raise_motion_not_enabled() -> None:
    """Guard: a properly injected clock/sleeper must not trip the new guard."""
    control = importlib.import_module(_CONTROL_PACKAGE)
    session, _client, _clock, _sleeper = _b5_never_settled_session(settled=True)

    session.left.gripper.open()

    assert session.state.state is ControlState.ACTIVE
    assert control.MotionNotEnabled is not None
    session.disconnect()


# ===========================================================================
# P5 cycle 3 — B8 (dispatch-side metadata channel), B9 (frozen-frame fake
#               settled), F4 (production-form fail-closed guards).
#
# Cycle-3 acceptance (terminal-review findings F1/F2/F4):
#
# * B8 (F1, major) — the DISPATCH side must resolve metadata from the
#   production observation form.  ``AlohaMiniClient.get_observation()`` never
#   carries ``_robot_metadata`` (char_2), so:
#     - ``CartesianController._tick_arm_pose`` must receive the resolved
#       metadata through the tick channel instead of reading only the
#       observation dict (otherwise every non-dry-run tick is ADAPTER_FAILED
#       and no arm command ever reaches the wire);
#     - ``ArmHandle.move_by`` / ``_compute_moved`` must fall back to
#       ``client.latest_robot_metadata`` (otherwise they raise immediately /
#       report zero displacement).
# * B9 (F2) — ``_resolve_feedback`` must treat only a FORWARD sample_id as new
#   evidence.  A frozen host stream (``observation_sequence`` stops advancing,
#   the client serves its cached ``last_remote_state``) must NOT derive
#   ``settled=True`` from a repeated sample: otherwise the arrival gate fires
#   while the arm is still moving and ``move_to`` returns a bogus MoveReport.
# * F4 — guards: (a) the production form that genuinely lacks metadata
#   (``include_metadata=False``) stays fail-closed (no arm command on the wire,
#   a reason-carrying control error); (b) a host restart observed through the
#   ``client._host_session_id`` attribute fallback resets the stale sample-id
#   evidence so the restarted session re-qualifies instead of latching FAULT.
# ===========================================================================


def _wire_actions(client: Any) -> list[dict]:
    """Every action the client transmitted, in send order (JSON round-tripped)."""
    return client.sends


# ---------------------------------------------------------------------------
# B8 — production-form dispatch: metadata must reach the controller/move_by.
# ---------------------------------------------------------------------------
def test_red_c3_b8_move_to_dispatches_arm_commands_in_production_form() -> None:
    """Non-dry-run ``move_to`` must put arm joint commands on the wire.

    The arm holds ``_C2_Q0`` until priming qualifies, then settles at
    ``_C2_Q_ARRIVED`` (inside the 5 mm tolerance).  ``_prod_client`` serves the
    production form, whose observation never carries ``_robot_metadata``.

    Catches the dispatch-side metadata gap: when the controller reads only
    ``feedback["_robot_metadata"]`` it reports ADAPTER_FAILED on every tick, so
    (beyond the single connect hold) no arm command reaches the wire while
    ``move_to`` still returns a success-shaped report.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda n: _C2_Q0 if n < 6 else _C2_Q_ARRIVED)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=2000)
    session = _c8_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=3.0,
    )
    sent_before = len(client.sends)

    report = session.left.move_to(*_C2_TARGET_XYZ)

    assert isinstance(report, control.MoveReport)
    assert report.residual_m <= _C2_TOL
    dispatched = _wire_actions(client)[sent_before:]
    assert dispatched, "production-form non-dry-run move_to must dispatch arm commands on the wire"
    assert report.steps, "move_to must record the dispatched control steps"
    for action in dispatched:
        assert any(key.startswith("arm_left_") for key in action), (
            f"dispatched action is not an arm-joint command: {action!r}"
        )
    session.disconnect()


def test_red_c3_b8_move_by_available_in_production_form() -> None:
    """Non-dry-run ``move_by`` must read the pose via the attribute fallback.

    ``move_by`` needs the measured FK baseline before it can build the target.
    In the production form that baseline comes from
    ``client.latest_robot_metadata`` (the observation never carries it).

    Catches ``move_by`` raising ``FeedbackLost`` immediately because it still
    reads only ``feedback["_robot_metadata"]``.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda n: _C2_Q0 if n < 6 else _C2_Q_ARRIVED)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=2000)
    session = _c8_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=3.0,
    )
    sent_before = len(client.sends)

    report = session.left.move_by(dx=0.001, dy=0.0, dz=0.0)

    assert isinstance(report, control.MoveReport)
    assert report.residual_m <= _C2_TOL
    assert _wire_actions(client)[sent_before:], "production-form move_by must dispatch arm commands"
    session.disconnect()


def test_red_c3_b8_dry_run_move_by_available_in_production_form() -> None:
    """dry-run ``move_by`` still needs the FK baseline from the attribute.

    The observation is not optional just because no action is sent (B21): the
    production form must source the pose from ``latest_robot_metadata`` and
    return a report instead of ``FeedbackLost``.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda n: _C2_Q0 if n < 6 else _C2_Q_ARRIVED)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=2000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=3.0,
    )

    report = session.left.move_by(dx=0.001, dy=0.0, dz=0.0)

    assert isinstance(report, control.MoveReport)
    assert report.residual_m <= _C2_TOL
    assert client.sends == [], "dry-run must never send"
    session.disconnect()


def test_red_c3_b8_timeout_reports_moved_in_production_form() -> None:
    """A production-form ``move_to`` timeout must measure the real displacement.

    The scripted arm keeps moving away from an unreachable target, so the move
    deadline expires.  ``MotionTimeout.moved`` is derived by ``_compute_moved``
    from the latest measured pose, which in the production form is only
    available through the client attribute.

    Catches ``_compute_moved`` returning 0.0 for every production-form timeout
    because it reads only ``feedback["_robot_metadata"]``.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda n: (10.0 + n, -20.0, 30.0, -40.0, 50.0, -60.0))
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=4000)
    session = _c8_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=0.5,
        feedback_timeout_s=5.0,
    )

    with pytest.raises(control.MotionTimeout) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)

    assert excinfo.value.moved > 0.0, (
        "a production-form timeout must report the real Cartesian displacement"
    )
    session.disconnect()


# ---------------------------------------------------------------------------
# B9 — a frozen host stream must not be mistaken for fresh settled evidence.
# ---------------------------------------------------------------------------
class _B9FreezeStream(_ProdObsStream):
    """Production-form stream that stops serving new frames mid-motion.

    Until ``phase`` flips to ``"move"`` it serves a steady priming pose.  It
    then serves alternating in-tolerance moving poses for
    ``freeze_after_move_polls`` samples and afterwards returns ``None``: the
    fake client then serves its cached ``last_remote_state`` with a frozen
    ``observation_sequence``, exactly like a host that stopped streaming.
    """

    def __init__(
        self, client: Any, *, side: str = "left", host_session_id: str = "prod-session"
    ) -> None:
        super().__init__(client, lambda _n: _C2_Q0, side=side, host_session_id=host_session_id)
        self.phase = "prime"
        self.freeze_after_move_polls: int | None = None
        self.move_polls = 0
        self.frozen = False

    def __call__(self, request_token: Any, timeout_ms: Any) -> Any:
        if self.frozen:
            return None
        self.polls += 1
        if self.phase == "prime":
            q_deg = _C2_Q0
        else:
            self.move_polls += 1
            q_deg = _C8_Q_AT_TARGET_A if self.move_polls % 2 else _C8_Q_AT_TARGET_B
            if (
                self.freeze_after_move_polls is not None
                and self.move_polls >= self.freeze_after_move_polls
            ):
                self.frozen = True
        frame = dict.fromkeys(self.client._state_order, 0.0)
        frame.update(_c2_motor_pos(q_deg, self.side))
        frame["_robot_metadata"] = _c2_metadata(self.side)
        frame["_control_status"] = {
            "schema_version": 1,
            "host_session_id": self.host_session_id,
            "sample_sequence": self.polls,
            "joint_holds": {},
            "gripper_holds": {},
        }
        return [json.dumps(frame).encode("utf-8")]


def _b9_prod_client() -> Any:
    """Build the local fake client driven by a freezable scripted stream."""
    client = _FakeAlohaClient()
    stream = _B9FreezeStream(client)
    client._receive_observation_response = stream
    return client, stream


def test_red_c3_b9_frozen_stream_mid_motion_is_not_arrival() -> None:
    """A host that stops streaming mid-motion must not be reported as arrival.

    The arm is primed ACTIVE(ARM) on a steady pose, then moves between two
    in-tolerance poses, then the host stops.  The residual is already inside
    tolerance, so only a *fresh* standstill sample may open the arrival gate.
    A repeated/frozen sample is not fresh evidence.

    Catches ``_resolve_feedback`` deriving ``settled=True`` by comparing a
    repeated frozen frame against the baseline it just updated; that makes
    ``move_to`` return a bogus ``MoveReport`` while the arm is still moving.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, stream = _b9_prod_client()
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=4000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=5.0,
        feedback_timeout_s=1.0,
    )

    session.left.gripper.open()
    assert session.state.state is ControlState.ACTIVE
    assert session.state.active_stage is Stage.ARM

    stream.phase = "move"
    stream.freeze_after_move_polls = 2

    with pytest.raises((control.FeedbackLost, control.MotionTimeout)):
        session.left.move_to(*_C2_TARGET_XYZ)

    assert stream.frozen, "the scripted host stream must have stopped"
    assert stream.move_polls >= 2, "the arm must have been moving when the stream froze"
    session.disconnect()


# ---------------------------------------------------------------------------
# F4 — production-form fail-closed guards.
# ---------------------------------------------------------------------------
def test_red_c3_f4_production_form_without_metadata_is_fail_closed() -> None:
    """A production frame that genuinely lacks metadata must fail closed.

    With ``include_metadata=False`` the client commits an empty metadata side
    channel, so no kinematic conversion is possible.  ``move_to`` must not
    silently dispatch a bogus command or return a success-shaped report.

    Guard for the B8 fix: the resolved-metadata channel must preserve the
    fail-closed behaviour when the metadata is truly absent.
    """
    from alohamini_cartesian.control.commands import MotionBlocked

    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda _n: _C2_Q0, include_metadata=False)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=4000)
    session = _c8_connect(
        dry_run=False,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=0.5,
        feedback_timeout_s=5.0,
    )
    sent_before = len(client.sends)

    with pytest.raises(
        (control.MotionTimeout, control.FeedbackLost, control.OutOfRange, MotionBlocked)
    ):
        session.left.move_to(*_C2_TARGET_XYZ)

    assert len(client.sends) == sent_before, (
        "no arm command may reach the wire without usable metadata"
    )
    session.disconnect()


class _F4RestartStream(_ProdObsStream):
    """Production-form stream whose host session can restart mid-operation."""

    def __init__(self, client: Any, *, side: str = "left") -> None:
        super().__init__(client, lambda n: _C2_Q0 if n < 2 else _C2_Q_ARRIVED, side=side)
        self.restart_on_next = False

    def __call__(self, request_token: Any, timeout_ms: Any) -> Any:
        if self.restart_on_next:
            self.restart_on_next = False
            self.host_session_id = "prod-session-B"
            self.polls = 0
        return super().__call__(request_token, timeout_ms)


def _f4_restart_prod_client() -> Any:
    client = _FakeAlohaClient()
    stream = _F4RestartStream(client)
    client._receive_observation_response = stream
    return client, stream


def test_red_c3_f4_host_restart_recovers_via_client_attribute() -> None:
    """A restarted host observed through ``client._host_session_id`` must recover.

    After an ACTIVE session whose sample-id high-water sits far above the
    restarted sequence, the host restarts (new ``host_session_id`` and
    ``sample_sequence`` back to 1).  In the production form the session id is
    only available through the client attribute.

    Catches the missing attribute fallback: without it the restarted sample ids
    are unseen yet below the high-water mark, so the state machine latches
    FAULT and ``move_to`` raises ``MotionBlocked`` instead of completing.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, stream = _f4_restart_prod_client()
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=4000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        move_timeout_s=3.0,
        feedback_timeout_s=5.0,
    )

    session.left.gripper.open()
    assert session.state.state is ControlState.ACTIVE

    # Long sess-A history: the seen high-water sits far above the replay range.
    for sample_id in range(100, 2001):
        session.state.on_feedback(sample_id, settled=True)
    assert session.state.state is ControlState.ACTIVE

    stream.restart_on_next = True

    report = session.left.move_to(*_C2_TARGET_XYZ)

    assert isinstance(report, control.MoveReport)
    assert client._host_session_id == "prod-session-B"
    session.disconnect()


# ===========================================================================
# P5 cycle 4 - adapter-supervision minors (cycle-3 terminal review).
#
# * C4a (minor 1) - the ``move_to`` loop must abort with a reason-carrying
#   ``MotionBlocked`` after ``_MAX_CONSECUTIVE_ADAPTER_FAILURES`` (3)
#   consecutive ADAPTER_FAILED ticks, and must reset that counter on any
#   non-ADAPTER_FAILED tick (fail/fail/success interleaving must stay a plain
#   ``MotionTimeout``, never a spurious block).
# * C4b (minor 2, main-agent contract) - a session opened with ``adapter=None``
#   whose production-form feedback carries no explicit ``settled``/``valid``
#   field can never qualify the motion gate, so the first motion-handle
#   operation must fail closed with a reason-carrying ``MotionNotEnabled``
#   that names the missing adapter, instead of spinning until a silent
#   ``MotionTimeout``.
# ===========================================================================


class _C4ScriptedAdapter:
    """Adapter proxy whose forward conversion can be scripted to fail.

    Every other member (including ``_signs``) is delegated to the wrapped
    ``JointKinematicAdapter`` so ``_resolve_feedback`` still drives the real
    side configuration; only ``observation_to_kinematic_deg`` can be toggled.
    """

    def __init__(self, inner: Any) -> None:
        self._inner = inner
        self.fail = False
        self.raises = 0

    def __getattr__(self, name: Any) -> Any:
        return getattr(self._inner, name)

    def observation_to_kinematic_deg(self, side: Any, feedback: Any, metadata: Any) -> Any:
        if self.fail:
            self.raises += 1
            raise RuntimeError("C4 scripted adapter failure")
        return self._inner.observation_to_kinematic_deg(side, feedback, metadata)


def _c4_arm_is_active(session: Any) -> bool:
    """True once the shared machine services the ACTIVE(ARM) stage."""
    return (
        session is not None
        and session.state.state is ControlState.ACTIVE
        and session.state.active_stage is Stage.ARM
    )


def test_red_c4_a1_consecutive_adapter_failures_block_with_reason() -> None:
    """3 consecutive ADAPTER_FAILED ticks abort ``move_to`` with the reason.

    The scripted adapter converts feedback correctly only while priming; as
    soon as the arm path is ACTIVE(ARM) it fails every forward conversion, so
    each ``move_to`` tick is ADAPTER_FAILED.  The loop must abort at the
    configured threshold with a ``MotionBlocked`` that names the adapter
    failure and carries the controller's detail, instead of burning the move
    timeout (``MotionTimeout``).

    Catches an implementation that never counts consecutive ADAPTER_FAILED
    ticks (silent timeout) or that aborts without surfacing
    ``adapter_failed``/the detail.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    adapter = _C4ScriptedAdapter(_c2_adapter())
    holder: dict = {}
    probe = {"active_seen": False, "fail_ticks": 0}

    def position_fn(_polls: int) -> Any:
        session = holder.get("session")
        if _c4_arm_is_active(session):
            probe["active_seen"] = True
        adapter.fail = probe["active_seen"]
        if adapter.fail:
            probe["fail_ticks"] += 1
        return _C2_Q0

    client, _stream = _prod_client(position_fn)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=4000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=adapter,
        move_timeout_s=0.5,
    )
    holder["session"] = session

    with pytest.raises(control.MotionBlocked) as excinfo:
        session.left.move_to(*_C2_TARGET_XYZ)

    text = str(excinfo.value)
    assert "adapter_failed" in text, f"MotionBlocked must name the adapter failure: {text!r}"
    assert "C4 scripted adapter failure" in text, f"MotionBlocked must carry the detail: {text!r}"
    assert probe["active_seen"], "the scripted failure must begin only once the arm path is ACTIVE"
    assert probe["fail_ticks"] >= 3, "the move loop must observe >=3 consecutive failing ticks"
    session.disconnect()


def test_red_c4_a2_interleaved_adapter_failures_reset_the_counter() -> None:
    """A non-ADAPTER_FAILED tick must reset the consecutive-failure counter.

    Once ACTIVE(ARM), the scripted adapter fails, fails, then succeeds and
    repeats.  Three ADAPTER_FAILED ticks therefore occur, but never three in a
    row, so the move must run to its deadline and raise ``MotionTimeout``.

    Catches an implementation that increments the counter without clearing it
    on an intervening success: the third failing tick would then raise
    ``MotionBlocked`` and the assert target would be violated.
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    adapter = _C4ScriptedAdapter(_c2_adapter())
    holder: dict = {}
    plan = (True, True, False)
    probe = {"active_seen": False, "tick": 0, "fail_ticks": 0}

    def position_fn(_polls: int) -> Any:
        session = holder.get("session")
        if _c4_arm_is_active(session):
            probe["active_seen"] = True
        if probe["active_seen"]:
            value = plan[probe["tick"] % len(plan)]
            probe["tick"] += 1
            adapter.fail = value
            if value:
                probe["fail_ticks"] += 1
        else:
            adapter.fail = False
        return _C2_Q0

    client, _stream = _prod_client(position_fn)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=4000)
    session = _c8_connect(
        dry_run=True,
        client=client,
        clock=clock,
        sleeper=sleeper,
        adapter=adapter,
        move_timeout_s=0.5,
    )
    holder["session"] = session

    with pytest.raises(control.MotionTimeout):
        session.left.move_to(*_C2_TARGET_XYZ)

    assert probe["fail_ticks"] >= 3, "the interleaved schedule must produce >=3 failing ticks"
    session.disconnect()


def test_red_c4_b_adapter_none_blocks_non_arm_handle_with_adapter_reason() -> None:
    """``adapter=None`` fails closed naming the adapter, not a silent timeout.

    A dry-run session is opened with ``adapter=None`` and a production-form client
    whose observation carries neither ``_control_status`` nor
    ``_robot_metadata``; the side channel likewise has no explicit
    ``settled``/``valid``.  Without an adapter those fields can never be
    derived, so the motion gate is unqualifiable and ``gripper.open()`` must
    refuse with a reason-carrying ``MotionNotEnabled`` that names the adapter.

    The committed behavior is: with the adapter missing and no explicit
    ``settled``/``valid`` evidence, ``_prime_state_machine`` fast-fails on its
    first tick and raises ``MotionNotEnabled`` whose reason names the missing
    adapter (the pre-fix signature raised ``MotionTimeout`` instead).
    """
    control = importlib.import_module(_CONTROL_PACKAGE)
    client, _stream = _prod_client(lambda _polls: _C2_Q0)
    clock = _C2FakeClock()
    sleeper = _C2RecordingSleeper(clock, max_ticks=400)

    session = control.connect(
        "host-c4b",
        dry_run=True,
        client_factory=lambda host, **kwargs: client,
        clock=clock,
        sleeper=sleeper,
        adapter=None,
        control_period_s=_C2_PERIOD,
        move_timeout_s=0.5,
    )

    probe = client.get_observation()
    assert "_control_status" not in probe
    assert "_robot_metadata" not in probe
    assert "settled" not in (client.latest_control_status or {})
    assert "valid" not in (client.latest_control_status or {})

    with pytest.raises(control.MotionNotEnabled) as excinfo:
        session.left.gripper.open()

    assert "adapter" in excinfo.value.reason.lower(), (
        f"the refusal must name the missing adapter: {excinfo.value.reason!r}"
    )
    session.disconnect()


# ===========================================================================
# 迁移后唯一的行为变更 — 默认 client 工厂必须 fail-closed。
# ---------------------------------------------------------------------------
# 本库把控制层从上游 lerobot 迁入后，``control/api.py`` 的
# ``_default_client_factory`` 不再 import lerobot 的 ``AlohaMiniClient``，而是抛
# ``alohamini_cartesian.control.RuntimeUnavailable``。上面全部用例都显式注入
# ``client_factory``，这条断言是本次迁移**唯一**行为变更的专项覆盖：
#
# * 未注入 ``client_factory`` 时 ``connect`` 必须 fail-closed 抛
#   ``RuntimeUnavailable``，绝不能静默连上或退回一个隐式运行时；
# * 抛错路径不得顺手把 lerobot 拉进 ``sys.modules``（前后快照差集口径，不依赖
#   会话级“lerobot 从未被 import”，因此装了 runtime extra 的环境同样成立）。
# ===========================================================================
def test_default_client_factory_without_runtime_raises_runtime_unavailable() -> None:
    """未注入 client_factory 时默认工厂必须 fail-closed：抛 RuntimeUnavailable 且不引入 lerobot。"""
    control = importlib.import_module(_CONTROL_PACKAGE)

    def _lerobot_modules() -> set[str]:
        return {name for name in sys.modules if name == "lerobot" or name.startswith("lerobot.")}

    before = _lerobot_modules()

    with pytest.raises(control.RuntimeUnavailable) as excinfo:
        control.connect("127.0.0.1")

    after = _lerobot_modules()

    assert after - before == set()
    assert "client_factory" in str(excinfo.value), (
        f"the refusal must point at injecting client_factory: {str(excinfo.value)!r}"
    )
