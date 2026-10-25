"""最小离线示例：一次 ``move_by`` 的 ``dry_run`` 全链路。

这个示例只演示库的调用形状，不碰任何真实硬件：

1. 这是离线演示，不连机器人、不联网：脚本里的 ``_FakeTransport`` 是内存里的假
   transport，观测由常量关节角生成，不发任何串口或网络报文。
2. 真机使用要注入厂商客户端：``connect(..., client_factory=...)``，由注入的工厂返回
   ``AlohaMiniClient``（或任何满足 ``adapter/protocol.py`` 里 ``RobotTransport`` 协议的
   对象）。本库不含 lerobot 运行时，未注入 ``client_factory`` 时 ``connect`` 会直接
   ``RuntimeUnavailable``。
3. ``dry_run=True`` 下不会下发任何动作：控制器照常求解、照常产出动作字典，但
   ``CartesianController._send`` 在 dry-run 直接返回，所以
   ``send_action`` 一次都不会被调用。脚本结尾用真实计数器断言 ``0 次发送``。

因为 dry-run 不发动作，假 transport 的观测始终停在起始姿态，最终报告的
``residual`` 就等于本次请求的位移本身（4 mm），仍在 5 mm 到达容差之内。
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from alohamini_cartesian.control.api import connect
from alohamini_cartesian.control.arm_ik import IKResult
from alohamini_cartesian.kinematics.joint_adapter import JointKinematicAdapter

_SIDE = "left"
_ROBOT_MODEL = "alohamini2pro"
_HOST = "dry-run-host"

# 与 ``kinematics/joint_adapter.py`` 的 JOINT_ORDER 保持一致。
_JOINT_ORDER = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)

# 冻结的假标定：符号翻转 + 零位，全部为零位，使「动作 → 观测 → 运动学角」可逆。
_SIGNS = {
    "shoulder_pan": 1.0,
    "shoulder_lift": -1.0,
    "elbow_flex": 1.0,
    "wrist_flex": -1.0,
    "wrist_yaw": 1.0,
    "wrist_roll": -1.0,
}

# 起始关节角（度）。假 FK 把前三个关节角按 1 度 = 1 毫米映射到平移。
_START_Q_DEG = (10.0, -20.0, 30.0, -40.0, 50.0, -60.0)

# 本次请求的位移（米）：取小增量，落在默认 5 mm 到达容差之内。
_STEP_M = 0.004


def _make_metadata() -> dict[str, Any]:
    """造一份 schema v2 的最小元数据，供关节换算与限位计算使用。

    Returns:
        含 ``schema_version``、``robot_model`` 与六个 ``arm_<side>_<joint>`` 电机条目的映射。
    """
    motors: dict[str, Any] = {}
    for index, joint in enumerate(_JOINT_ORDER):
        motors[f"arm_{_SIDE}_{joint}"] = {
            "id": index + 1,
            "model": f"dry-run-{index + 1}",
            "normalization": "degrees",
            "drive_mode": 1 if index % 2 else 0,
            "range_min": 41 + index,
            "range_max": 4001 + index,
            "homing_offset": 100 * (index + 1) - 500,
            "resolution": 6399 + index,
        }
    return {"schema_version": 2, "robot_model": _ROBOT_MODEL, "motors": motors}


def _make_observation(q_deg: Sequence[float], *, sample_id: int) -> dict[str, Any]:
    """把关节角装成一帧观测：电机角度 + 元数据 + 显式 ``settled``/``valid`` 声明。

    Args:
        q_deg: 六个关节的运动学角度（度）。
        sample_id: 观测序号，必须单调递增，否则控制层视为陈旧样本。

    Returns:
        键为 ``arm_<side>_<joint>.pos`` 的扁平常量，附带 ``_robot_metadata`` 与
        ``_control_status``。
    """
    observation: dict[str, Any] = {
        f"arm_{_SIDE}_{joint}.pos": _SIGNS[joint] * float(q_deg[index])
        for index, joint in enumerate(_JOINT_ORDER)
    }
    observation["_robot_metadata"] = _make_metadata()
    observation["_control_status"] = {
        "schema_version": 1,
        "host_session_id": "dry-run-session",
        "sample_sequence": sample_id,
        "settled": True,
        "valid": True,
        "joint_holds": {},
        "gripper_holds": {},
    }
    return observation


class _FakeTransport:
    """内存版 transport：实现 ``RobotTransport`` 的三个方法，并真实统计下发次数。

    真机上这一层由厂商客户端（``AlohaMiniClient``）担任；这里只保证协议齐全，
    且 ``send_action`` 的调用次数被如实记账，供脚本结尾断言 ``0 次发送``。
    """

    def __init__(self, q_deg: Sequence[float]) -> None:
        """记录起始关节角；观测序号从 0 开始自增。

        Args:
            q_deg: 六个关节的起始运动学角度（度），观测永远停在这一姿态。
        """
        self._q_deg = tuple(float(value) for value in q_deg)
        self._sample_id = 0
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.send_action_calls = 0
        self.prime_calls = 0
        self.include_cameras_primed: bool | None = None
        self._host_session_id = "dry-run-session"

    def connect(self) -> None:
        """假连接：只累加计数，不做任何 I/O。"""
        self.connect_calls += 1

    def disconnect(self) -> None:
        """假断开：只累加计数，不做任何 I/O。"""
        self.disconnect_calls += 1

    @property
    def sample_count(self) -> int:
        """已产出的观测帧数。

        Returns:
            帧数。
        """
        return self._sample_id

    def get_observation(self) -> Mapping[str, Any]:
        """返回一帧常量姿态观测，并把 ``sample_sequence`` 自增一。

        Returns:
            一帧观测映射。
        """
        self._sample_id += 1
        return _make_observation(self._q_deg, sample_id=self._sample_id)

    def send_action(self, action: Mapping[str, float]) -> Mapping[str, Any]:
        """记录一次下发（真实计数），并原样回显这份动作。

        Args:
            action: 归一化单位的动作映射。

        Returns:
            与入参内容相同的动作映射。
        """
        self.send_action_calls += 1
        return dict(action)

    def prime_observation_request_window(self, *, include_cameras: bool) -> None:
        """假预热：只累加计数，并记下本次是否连相机一起预热。

        Args:
            include_cameras: 是否连相机一起预热。
        """
        self.prime_calls += 1
        self.include_cameras_primed = include_cameras


class _FakeArmIK:
    """线性假 IK：``fk`` 把前三个关节角映射成平移，``solve`` 原样返回种子。

    真实实现是 placo 数值 IK；示例只需要一个确定、可逆的替身，让控制循环
    不必依赖求解器数值行为。
    """

    def __init__(self, side: str) -> None:
        """绑定手臂侧别并清零求解计数。

        Args:
            side: ``"left"`` 或 ``"right"``。
        """
        self.side = side
        self.solve_calls = 0

    def fk(self, q_deg: Any) -> np.ndarray:
        """正运动学：平移 = 前三个关节角 × 1 mm/度，姿态取单位阵。

        Args:
            q_deg: 六个关节的运动学角度（度）。

        Returns:
            4×4 齐次变换矩阵。
        """
        q = np.asarray(q_deg, dtype=float)
        pose = np.eye(4, dtype=float)
        pose[0, 3] = q[0] * 0.001
        pose[1, 3] = q[1] * 0.001
        pose[2, 3] = q[2] * 0.001
        return pose

    def solve(
        self,
        *,
        q_current_deg: Any,
        target_pose: Any,
        joint_limits_deg: Any,
        orientation_weight: float,
    ) -> IKResult:
        """把当前位形当作解返回（零误差），使求解永远成功、每次只走 0 步。

        Args:
            q_current_deg: 求解种子，即当前运动学关节角（度）。
            target_pose: 目标 4×4 位姿（本替身不解算，仅占位）。
            joint_limits_deg: 关节限位映射（本替身不检查，仅占位）。
            orientation_weight: 姿态软约束权重（本替身不使用，仅占位）。

        Returns:
            成功且零位置误差的求解结果。
        """
        self.solve_calls += 1
        return IKResult(
            q_deg=np.asarray(q_current_deg, dtype=float),
            ok=True,
            reason=None,
            position_error_m=0.0,
            orientation_error_deg=0.0,
            iterations=1,
            solve_time_s=0.0,
        )


class _FakeClock:
    """假单调时钟：时间只由 ``_FakeSleeper`` 推进，脚本不消耗真实时间。"""

    def __init__(self) -> None:
        """从 0 秒开始计时。"""
        self.now = 0.0

    def __call__(self) -> float:
        """返回当前假时间（秒）。

        Returns:
            秒数。
        """
        return self.now


class _FakeSleeper:
    """假休眠：只推进假时钟并记录次数，让 30 Hz 控制循环瞬间跑完。"""

    def __init__(self, clock: _FakeClock) -> None:
        """绑定被推进的假时钟。

        Args:
            clock: 要推进的假时钟。
        """
        self._clock = clock
        self.calls = 0

    def __call__(self, seconds: float) -> None:
        """推进假时钟，不做真实等待。

        Args:
            seconds: 本次「休眠」时长（秒）。
        """
        self.calls += 1
        self._clock.now += float(seconds)


def _make_adapter() -> JointKinematicAdapter:
    """造一个只配了左臂符号与零位的真实关节换算器。

    Returns:
        运动学适配器。
    """
    return JointKinematicAdapter(
        signs={_SIDE: dict(_SIGNS)},
        zeros={_SIDE: dict.fromkeys(_JOINT_ORDER, 0.0)},
    )


def main() -> None:
    """跑一次 ``move_by`` 的 dry-run 全链路，打印每步诊断与最终报告。

    Raises:
        AssertionError: dry-run 下真的调用了 ``send_action``（说明 dry-run 门禁失效）。
    """
    transport = _FakeTransport(_START_Q_DEG)
    clock = _FakeClock()
    sleeper = _FakeSleeper(clock)

    session = connect(
        _HOST,
        dry_run=True,
        client_factory=lambda host, **_kwargs: transport,
        clock=clock,
        sleeper=sleeper,
        ik_factory=_FakeArmIK,
        adapter=_make_adapter(),
    )

    print(f"host={_HOST}，dry_run=True：不会下发任何动作。")
    print(f"请求：left.move_by(dx={_STEP_M} m)")
    try:
        report = session.left.move_by(dx=_STEP_M)
    finally:
        session.disconnect()

    for index, step in enumerate(report.steps, start=1):
        keys = sorted(step.action) if step.action is not None else []
        first = keys[0] if keys else "<无>"
        print(f"  第 {index} 步诊断：动作键 {len(keys)} 个，首键 {first}")

    print(f"最终报告：steps={len(report.steps)} 步，residual={report.residual_m:.6f} m")
    print(
        f"循环共读 {transport.sample_count} 帧观测、假休眠 {sleeper.calls} 次"
        f"（假时钟推进到 {clock():.3f} s，未消耗真实时间）"
    )

    assert transport.send_action_calls == 0, (
        f"dry_run=True 下不应下发动作，实际调用 {transport.send_action_calls} 次"
    )
    print(f"send_action 调用次数: {transport.send_action_calls} 次发送（dry_run=True，期望 0）")


if __name__ == "__main__":
    main()
