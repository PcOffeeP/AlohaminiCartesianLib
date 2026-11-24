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

"""Public control SDK API for AlohaMini.

This module exposes the session-level ``connect`` entry point, the handle
abstractions for arms/lift/base/gripper, and the small vocabulary of
motion-related exceptions used by the P5 control layer.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..adapter.protocol import RobotClient
from .commands import (
    ArmCommand,
    BaseCommand,
    ControlState,
    ControlStateMachine,
    GripperAction,
    GripperCommand,
    LiftCommand,
    ReleaseCommand,
    Side,
    Stage,
    StopCommand,
)
from .commands import (
    MotionBlocked as _MotionBlocked,
)
from .controller import CartesianController, ControlError, ResolvedControlStatus

DEFAULT_PRECHECK_STEP_M = 0.001

# Maximum joint-space displacement between two consecutive feedback samples that
# is treated as "no motion" when deriving the standstill signal from measured
# kinematic state.  Configurable per session via ``settled_max_joint_delta_deg``.
_SETTLED_MAX_JOINT_DELTA_DEG = 0.1

# Maximum lift-axis height change (mm) between consecutive samples treated as
# standstill when priming Stage.BASE_LIFT without an arm kinematic adapter.
_SETTLED_MAX_LIFT_DELTA_MM = 0.5
_SETTLED_MAX_BASE_VEL = 1e-3

# Maximum recursive bisection depth when subdividing a waypoint segment whose
# joint delta exceeds ``max_joint_step_deg``.
_MAX_BISECT_DEPTH = 6

# Consecutive controller ticks that report an adapter hard error before the
# move loop aborts with a reason-carrying MotionBlocked instead of timing out.
_MAX_CONSECUTIVE_ADAPTER_FAILURES = 3


def _read_state_observation(client: Any) -> Mapping[str, Any]:
    """Read a state observation, requesting ``include_cameras=False`` when supported."""
    try:
        return client.get_observation(include_cameras=False)
    except TypeError:
        return client.get_observation()


def _validate_orientation(orientation: Any) -> np.ndarray:
    """Validate an explicit 3x3 proper SO(3) rotation matrix target."""
    rot = np.asarray(orientation, dtype=float)
    if rot.shape != (3, 3) or not np.all(np.isfinite(rot)):
        raise ValueError(f"orientation must be a finite (3, 3) matrix, got shape {rot.shape}")
    if not np.allclose(rot.T @ rot, np.eye(3, dtype=float), atol=1e-4):
        raise ValueError("orientation must be an orthogonal SO(3) rotation matrix")
    det = float(np.linalg.det(rot))
    if not math.isclose(det, 1.0, abs_tol=1e-4):
        raise ValueError(f"orientation must have det=+1 (proper rotation), got det={det:.6f}")
    return rot.copy()


def _rotation_axis_angle(r_start: np.ndarray, r_end: np.ndarray) -> tuple[np.ndarray, float]:
    """Return unit axis and angle (rad) mapping ``r_start`` to ``r_end`` on SO(3)."""
    r_rel = r_start.T @ r_end
    cos_theta = float(np.clip((float(np.trace(r_rel)) - 1.0) / 2.0, -1.0, 1.0))
    theta = float(math.acos(cos_theta))
    if theta < 1e-9:
        return np.array([1.0, 0.0, 0.0], dtype=float), 0.0
    if math.pi - theta < 1e-6:
        # Near 180 deg: pick the largest-norm column of (R_rel + I) as rotation axis.
        mat = r_rel + np.eye(3, dtype=float)
        norms = np.linalg.norm(mat, axis=0)
        axis = mat[:, int(np.argmax(norms))]
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            return np.array([1.0, 0.0, 0.0], dtype=float), theta
        return axis / norm, theta
    skew = (r_rel - r_rel.T) / (2.0 * math.sin(theta))
    axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0], dtype=float), 0.0
    return axis / norm, theta


def _interpolate_rotation(r_start: np.ndarray, r_end: np.ndarray, t: float) -> np.ndarray:
    """Interpolate on SO(3) from ``r_start`` to ``r_end`` at fraction ``t`` in [0, 1]."""
    if t <= 0.0:
        return r_start.copy()
    if t >= 1.0:
        return r_end.copy()
    axis, theta = _rotation_axis_angle(r_start, r_end)
    if theta < 1e-9:
        return r_start.copy()
    angle = t * theta
    kx, ky, kz = float(axis[0]), float(axis[1]), float(axis[2])
    k_mat = np.array(
        [[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]],
        dtype=float,
    )
    r_step = (
        np.eye(3, dtype=float) + math.sin(angle) * k_mat + (1.0 - math.cos(angle)) * (k_mat @ k_mat)
    )
    return r_start @ r_step


class _ReasonCarryingError(Exception):  # noqa: N818
    """Base for exceptions that surface a human-readable ``reason``."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


class FeedbackLost(_ReasonCarryingError):  # noqa: N818
    """Feedback from the host was lost."""


class OutOfRange(_ReasonCarryingError):  # noqa: N818
    """Target lies outside the allowed range."""

    def __init__(self, reason: str, target: Any = None) -> None:
        """Record the refusal reason and the rejected target, if any."""
        self.target = target
        super().__init__(reason)


class MotionTimeout(_ReasonCarryingError):  # noqa: N818
    """Motion did not complete within the allowed time."""

    def __init__(self, reason: str, *, moved: float = 0.0) -> None:
        """Record the timeout reason and the distance moved before giving up."""
        self.moved = float(moved)
        super().__init__(reason)


class MotionNotEnabled(_ReasonCarryingError):  # noqa: N818
    """Real-motion connections require ``enable_motion=True``."""


class RuntimeUnavailable(RuntimeError):
    """默认 client 工厂不可用：本库不包含 lerobot 运行时。

    连接真实主机必须显式注入 ``client_factory``；若要使用默认连接，请自行
    构造 runtime extra（``lerobot``）里的 ``AlohaMiniClient`` 后注入。
    """


class GripperHandle:
    """Handle to a single gripper."""

    def __init__(self, side: str, session: AlohaMiniControl | None = None) -> None:
        """Bind the handle to ``side`` and, optionally, to a live session."""
        self.side = side
        self._session = session

    def _send_position(self, position: float) -> None:
        """Interlock and dispatch a single gripper position command."""
        session = self._session
        if session is None:
            raise MotionNotEnabled("GripperHandle is not attached to a session")
        side_enum = Side.LEFT if self.side == "left" else Side.RIGHT
        # Validate the command before any priming host I/O (B5).
        command = GripperCommand(side=side_enum, action=GripperAction.SET, position=float(position))
        session._prime_state_machine(stage=Stage.ARM)
        session.state.handle(command)
        if not session.dry_run:
            session.client.send_action({f"arm_{self.side}_gripper.pos": float(position)})

    def open(self) -> None:
        """Open the gripper."""
        self._send_position(100.0)

    def close(self) -> None:
        """Close the gripper."""
        self._send_position(0.0)

    def set(self, value: float) -> None:
        """Set the gripper position to ``value`` in the 0..100 range."""
        self._send_position(float(value))


@dataclass(frozen=True)
class StepDiagnostic:
    """One planned control step recorded during a dry-run move."""

    action: Mapping[str, float] | None


@dataclass(frozen=True)
class MoveReport:
    """Result of a completed ``ArmHandle.move_to`` call."""

    residual_m: float
    steps: Sequence[StepDiagnostic]


def _precheck_straight_path(
    arm_ik: Any,
    adapter: Any,
    side: str,
    q_current_deg: np.ndarray,
    target_pose: np.ndarray,
    metadata: Mapping[str, Any],
    precheck_step_m: float,
    orientation_weight: float,
    max_joint_step_deg: float = 2.0,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Plan and precheck waypoints along the straight line + SO(3) path.

    Samples ``i/n`` (endpoint included, start excluded) with linear position
    interpolation and SO(3) rotation interpolation from the current FK pose to
    ``target_pose``, solving IK with chained joint seeds.  When an adjacent
    segment exceeds ``max_joint_step_deg``, it is recursively bisected up to
    ``_MAX_BISECT_DEPTH`` levels so every consecutive step stays within the
    controller's joint-step budget.

    Returns:
        Non-empty list of ``(waypoint_pose, waypoint_q_deg)`` pairs ending at
        ``target_pose``.
    """
    current_pose = arm_ik.fk(q_current_deg)
    start_pos = np.asarray(current_pose[:3, 3], dtype=float)
    start_rot = np.asarray(current_pose[:3, :3], dtype=float)
    end_pos = np.asarray(target_pose[:3, 3], dtype=float)
    end_rot = np.asarray(target_pose[:3, :3], dtype=float)
    delta = end_pos - start_pos
    distance = float(np.linalg.norm(delta))

    _, rot_rad = _rotation_axis_angle(start_rot, end_rot)
    rot_deg = math.degrees(rot_rad)
    n_pos = int(math.ceil(distance / precheck_step_m)) if distance > 0.0 else 0
    n_rot = int(math.ceil(rot_deg / max(max_joint_step_deg, 1e-6))) if rot_deg > 1e-6 else 0
    n = max(1, n_pos, n_rot)

    joint_limits_deg = adapter.effective_limits_deg(side, metadata)

    def _pose_at(t: float) -> np.ndarray:
        waypoint = np.eye(4, dtype=float)
        waypoint[:3, :3] = _interpolate_rotation(start_rot, end_rot, t)
        waypoint[:3, 3] = start_pos + t * delta
        return waypoint

    def _solve_segment(
        t_a: float,
        q_a: np.ndarray,
        t_b: float,
        depth: int,
    ) -> tuple[list[tuple[np.ndarray, np.ndarray]], np.ndarray]:
        pose_b = _pose_at(t_b)
        result = arm_ik.solve(
            q_current_deg=q_a,
            target_pose=pose_b,
            joint_limits_deg=joint_limits_deg,
            orientation_weight=orientation_weight,
        )
        if not result.ok:
            raise OutOfRange(
                f"precheck failed at t={t_b:.4f}: {result.reason}",
                target=target_pose,
            )
        q_b = np.asarray(result.q_deg, dtype=float)
        joint_step = float(np.max(np.abs(q_b - q_a)))
        if math.isfinite(joint_step) and joint_step <= max_joint_step_deg:
            return [(pose_b, q_b)], q_b
        if not math.isfinite(joint_step) or depth >= _MAX_BISECT_DEPTH or t_b - t_a <= 1e-5:
            raise OutOfRange(
                f"precheck failed at t={t_b:.4f}: joint step {joint_step:.6g} deg "
                f"exceeds limit {max_joint_step_deg:.6g} deg or is non-finite",
                target=target_pose,
            )
        t_mid = 0.5 * (t_a + t_b)
        left_wps, q_mid = _solve_segment(t_a, q_a, t_mid, depth + 1)
        right_wps, q_end = _solve_segment(t_mid, q_mid, t_b, depth + 1)
        return left_wps + right_wps, q_end

    waypoints: list[tuple[np.ndarray, np.ndarray]] = []
    q_prev = np.asarray(q_current_deg, dtype=float)
    t_prev = 0.0
    for i in range(1, n + 1):
        t_curr = i / n
        seg_wps, q_prev = _solve_segment(t_prev, q_prev, t_curr, depth=0)
        waypoints.extend(seg_wps)
        t_prev = t_curr
    return waypoints


class ArmHandle:
    """Handle to a single arm, including its gripper."""

    def __init__(self, side: str, session: AlohaMiniControl | None = None) -> None:
        """Bind the handle to ``side`` and, optionally, to a live session."""
        self.side = side
        self.gripper = GripperHandle(side, session=session)
        self._session = session

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        timeout_s: float | None = None,
        *,
        orientation: np.ndarray | None = None,
    ) -> MoveReport:
        """Blocking Cartesian move to ``(x, y, z)`` in meters.

        Runs a paced loop that drives :class:`CartesianController` along
        prechecked straight-line + SO(3) waypoints until the measured FK
        position residual is within ``arrival_tolerance_m`` and the latest
        feedback sample reports ``settled=True``.  When ``orientation`` is
        omitted (``None``), the arm holds its initial measured FK orientation
        instead of rotating toward world identity.
        """
        session = self._session
        if session is None:
            raise MotionNotEnabled("ArmHandle is not attached to a session")

        controller = session._controllers.get(self.side)
        if controller is None:
            raise MotionNotEnabled(f"no controller available for {self.side}")

        fx, fy, fz = float(x), float(y), float(z)
        if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(fz)):
            raise ValueError(f"target coordinates must be finite, got ({x!r}, {y!r}, {z!r})")

        explicit_rot = _validate_orientation(orientation) if orientation is not None else None
        target: np.ndarray | None = None
        if explicit_rot is not None:
            target = np.eye(4, dtype=float)
            target[:3, :3] = explicit_rot
            target[0, 3] = fx
            target[1, 3] = fy
            target[2, 3] = fz

        timeout = session._move_timeout_s if timeout_s is None else float(timeout_s)
        deadline = session._clock() + timeout
        side_enum = Side.LEFT if self.side == "left" else Side.RIGHT

        initial_q: np.ndarray | None = None
        steps: list[StepDiagnostic] = []
        last_residual = float("inf")
        prechecked = False
        waypoints: list[tuple[np.ndarray, np.ndarray]] = []
        wp_idx = 0
        last_action: Mapping[str, float] | None = None
        saw_follow_timeout = False

        last_sample_id: int | None = None
        last_feedback_time = session._clock()
        consecutive_adapter_failures = 0

        while True:
            # Fast-fail on supervisor-level BLOCKED/FAULT before burning timeout.
            if session.state.state is ControlState.BLOCKED:
                reason = session.state.last_reason or "motion blocked"
                raise _MotionBlocked(reason)
            if session.state.state is ControlState.FAULT:
                reason = session.state.last_reason or "machine fault"
                session.last_reason = reason
                raise _MotionBlocked(reason)

            now = session._clock()
            if now >= deadline:
                moved = _compute_moved(session, controller, self.side, initial_q)
                suffix = " (FOLLOW_TIMEOUT)" if saw_follow_timeout else ""
                raise MotionTimeout(
                    f"move_to timed out after {timeout}s{suffix}",
                    moved=moved,
                )

            # Keepalive: resend the last accepted target if the deadline passed.
            if (
                last_action is not None
                and controller.next_deadline is not None
                and now > controller.next_deadline
            ):
                controller._send(last_action)

            try:
                feedback = _read_state_observation(session.client)
            except Exception:
                feedback = {}

            resolved = session._resolve_feedback(feedback, stage=Stage.ARM)
            sample_id = resolved.sample_id
            if sample_id != last_sample_id:
                last_sample_id = sample_id
                last_feedback_time = session._clock()
            elif sample_id is not None:
                if session._clock() - last_feedback_time > session._feedback_timeout_s:
                    raise FeedbackLost(
                        f"no new feedback for {session._feedback_timeout_s}s "
                        f"(sample_id={sample_id})"
                    )

            metadata = feedback.get("_robot_metadata")
            if metadata is None:
                metadata = getattr(session.client, "latest_robot_metadata", None)

            # Extract the current kinematic configuration early: it seeds the
            # controller, locks the held orientation (when orientation=None),
            # feeds the arrival residual, and enables whole-path precheck.
            q_current: np.ndarray | None = None
            adapter_error_detail: str | None = None
            if metadata is None:
                adapter_error_detail = "feedback is missing _robot_metadata"
            elif session._adapter is not None:
                try:
                    q_candidate = session._adapter.observation_to_kinematic_deg(
                        self.side, feedback, metadata
                    )
                    if np.all(np.isfinite(q_candidate)):
                        q_current = q_candidate
                        if initial_q is None:
                            initial_q = np.copy(q_current)
                    else:
                        adapter_error_detail = "non-finite kinematic observation"
                except Exception as exc:  # noqa: BLE001
                    q_current = None
                    adapter_error_detail = str(exc)

            if target is None and q_current is not None:
                start_pose = controller._arm_ik.fk(q_current)
                target = np.eye(4, dtype=float)
                target[:3, :3] = np.asarray(start_pose[:3, :3], dtype=float)
                target[0, 3] = fx
                target[1, 3] = fy
                target[2, 3] = fz

            # Prime the shared state machine until the arm path is active.
            if session.state.state is ControlState.STANDBY:
                session.state.resume()
            if session.state.state is ControlState.READY:
                session.state.handle(ArmCommand(side=side_enum, dx=0.0, dy=0.0, dz=0.0))

            # Lazy whole-path precheck & waypoint planning: runs once the arm
            # path becomes ACTIVE, immediately before the first solve/send.
            if (
                session.state.state is ControlState.ACTIVE
                and session.state.active_stage is Stage.ARM
                and not prechecked
                and q_current is not None
                and target is not None
                and session._adapter is not None
            ):
                waypoints = _precheck_straight_path(
                    controller._arm_ik,
                    session._adapter,
                    self.side,
                    q_current,
                    target,
                    metadata,
                    session._precheck_step_m,
                    controller._orientation_weight,
                    max_joint_step_deg=controller._max_joint_step_deg,
                )
                wp_idx = 0
                prechecked = True

            # Record whether this tick actually targets the final waypoint
            # before ``wp_idx`` is advanced, so arrival never triggers early.
            ticked_final_waypoint = (not waypoints) or (wp_idx == len(waypoints) - 1)
            if waypoints:
                tick_target: Any = waypoints[wp_idx][0]
            elif target is not None:
                tick_target = target
            else:
                # Fail-closed: never fabricate an identity orientation target
                # before the initial measured orientation has been locked.
                tick_target = None

            outcome = controller.tick(feedback, tick_target, resolved=resolved)

            # Fast-fail again in case the controller ingested a fault this tick.
            if session.state.state is ControlState.BLOCKED:
                reason = session.state.last_reason or "motion blocked"
                raise _MotionBlocked(reason)
            if session.state.state is ControlState.FAULT:
                reason = session.state.last_reason or "machine fault"
                session.last_reason = reason
                raise _MotionBlocked(reason)

            if outcome.error_code is ControlError.FOLLOW_TIMEOUT:
                saw_follow_timeout = True

            is_adapter_failure = outcome.error_code is ControlError.ADAPTER_FAILED or (
                tick_target is None
                and session.state.state is ControlState.ACTIVE
                and session.state.active_stage is Stage.ARM
                and adapter_error_detail is not None
            )
            if is_adapter_failure:
                consecutive_adapter_failures += 1
                if consecutive_adapter_failures >= _MAX_CONSECUTIVE_ADAPTER_FAILURES:
                    detail = outcome.detail or adapter_error_detail or "no detail"
                    raise _MotionBlocked(
                        f"adapter failed ({ControlError.ADAPTER_FAILED.value}): {detail}"
                    )
            else:
                consecutive_adapter_failures = 0

            if outcome.action is not None:
                steps.append(StepDiagnostic(action=dict(outcome.action)))
                last_action = outcome.action

            if (
                waypoints
                and wp_idx < len(waypoints) - 1
                and outcome.error_code is None
                and outcome.action is not None
            ):
                if session.dry_run:
                    wp_idx += 1
                elif (
                    q_current is not None
                    and float(np.max(np.abs(q_current - waypoints[wp_idx][1])))
                    <= session._waypoint_lead_deg
                ):
                    wp_idx += 1

            residual = float("inf")
            current_pose: np.ndarray | None = None
            if q_current is not None and target is not None:
                try:
                    current_pose = controller._arm_ik.fk(q_current)
                    residual = float(np.linalg.norm(current_pose[:3, 3] - target[:3, 3]))
                    last_residual = residual
                except Exception:
                    current_pose = None

            orientation_ok = True
            if explicit_rot is not None and target is not None:
                if current_pose is None:
                    orientation_ok = False
                else:
                    _, rot_err_rad = _rotation_axis_angle(
                        np.asarray(current_pose[:3, :3], dtype=float),
                        target[:3, :3],
                    )
                    rot_tol_rad = math.radians(max(2.0, controller._follow_error_tolerance_deg))
                    orientation_ok = rot_err_rad <= rot_tol_rad

            if (
                session.state.state is ControlState.ACTIVE
                and session.state.active_stage is Stage.ARM
                and target is not None
                and ticked_final_waypoint
                and (not waypoints or outcome.error_code is None)
                and residual <= session._arrival_tolerance_m
                and orientation_ok
                and resolved.settled
                and resolved.valid
            ):
                session.state.handle(ReleaseCommand())
                while session.state.state in (ControlState.ACTIVE, ControlState.SETTLING):
                    if session._clock() >= deadline:
                        break
                    try:
                        release_feedback = _read_state_observation(session.client)
                    except Exception:
                        release_feedback = {}
                    release_resolved = session._resolve_feedback(release_feedback, stage=Stage.ARM)
                    if release_resolved.sample_id is not None:
                        session.state.on_feedback(
                            release_resolved.sample_id,
                            release_resolved.settled,
                            valid=release_resolved.valid,
                        )
                    session._sleeper(session._control_period_s)
                return MoveReport(residual_m=last_residual, steps=steps)

            session._sleeper(session._control_period_s)

    def measured_pose(self) -> np.ndarray:
        """Read the measured end-effector pose in the arm frame (4x4, meters)."""
        session = self._session
        if session is None:
            raise MotionNotEnabled("ArmHandle is not attached to a session")
        controller = session._controllers.get(self.side)
        if controller is None or session._adapter is None:
            raise MotionNotEnabled(f"no controller available for {self.side}")

        try:
            feedback = _read_state_observation(session.client)
            metadata = feedback.get("_robot_metadata") or getattr(
                session.client, "latest_robot_metadata", None
            )
            if metadata is None:
                raise FeedbackLost("feedback is missing _robot_metadata")
            q_current = session._adapter.observation_to_kinematic_deg(self.side, feedback, metadata)
            pose = np.asarray(controller._arm_ik.fk(q_current), dtype=float)
            if pose.shape != (4, 4) or not np.isfinite(pose).all():
                raise ValueError("invalid measured FK pose")
            return pose.copy()
        except Exception as exc:
            raise FeedbackLost(f"cannot read current pose for {self.side}: {exc}") from exc

    def move_by(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        timeout_s: float | None = None,
        *,
        orientation: np.ndarray | None = None,
    ) -> MoveReport:
        """Blocking Cartesian move by ``(dx, dy, dz)`` in meters.

        The increment is interpreted in the body frame, rotated per side by
        ``session._r_body_to_arm[side]``, and added to the current measured
        FK pose.  The resulting target is then passed to :meth:`move_to`.
        """
        current_trans = self.measured_pose()[:3, 3]
        session = self._session
        delta = np.array([float(dx), float(dy), float(dz)], dtype=float)
        r = session._r_body_to_arm.get(self.side)
        if r is not None:
            delta = np.asarray(r, dtype=float) @ delta

        target = current_trans + delta
        return self.move_to(
            float(target[0]),
            float(target[1]),
            float(target[2]),
            timeout_s=timeout_s,
            orientation=orientation,
        )


class LiftHandle:
    """Handle to the lift axis."""

    def __init__(self, session: AlohaMiniControl | None = None) -> None:
        """Bind the handle to a live session, if one is given."""
        self._session = session

    def height(self) -> float:
        """Return the measured lift height in millimeters."""
        session = self._session
        if session is None:
            raise MotionNotEnabled("LiftHandle is not attached to a session")
        try:
            feedback = _read_state_observation(session.client)
        except Exception as exc:
            raise FeedbackLost(f"failed to read lift height: {exc}") from exc
        key = "lift_axis.height_mm"
        if key not in feedback:
            raise FeedbackLost(f"observation missing {key!r}")
        return float(feedback[key])

    def move_to(self, mm: float) -> None:
        """Move the lift to an absolute height ``mm`` in millimeters.

        The motion is refused while ARM motion is active.  The target is
        dispatched as ``{lift_axis.height_mm: mm}``.
        """
        session = self._session
        if session is None:
            raise MotionNotEnabled("LiftHandle is not attached to a session")
        if not math.isfinite(mm):
            raise ValueError("lift target mm must be finite")
        session._prime_state_machine(stage=Stage.BASE_LIFT)
        command = LiftCommand(delta_mm=0.0)
        try:
            session.state.handle(command)
        except _MotionBlocked:
            raise
        if not session.dry_run:
            session.client.send_action({"lift_axis.height_mm": float(mm)})
        session.state.handle(ReleaseCommand())

    def move_by(self, delta_mm: float) -> None:
        """Move the lift by ``delta_mm`` relative to the measured height."""
        current = self.height()
        self.move_to(current + float(delta_mm))


class BaseHandle:
    """Handle to the mobile base."""

    def __init__(self, session: AlohaMiniControl | None = None) -> None:
        """Bind the handle to a live session, if one is given."""
        self._session = session

    def drive(self, x: float, y: float, turn: float, seconds: float) -> None:
        """Run a blocking base velocity command; refused while ARM motion is active.

        The requested velocity is dispatched at ``session._control_period_s``
        until ``seconds`` have elapsed, followed by an explicit all-zero
        velocity command.
        """
        session = self._session
        if session is None:
            raise MotionNotEnabled("BaseHandle is not attached to a session")
        if session._clock is None or session._sleeper is None:
            raise MotionNotEnabled("BaseHandle requires an injected clock and sleeper")

        # Validate the command before any priming host I/O (B5).
        command = BaseCommand(
            x=float(x),
            y=float(y),
            turn=float(turn),
            seconds=float(seconds),
        )
        session._prime_state_machine(stage=Stage.BASE_LIFT)
        try:
            session.state.handle(command)
        except _MotionBlocked:
            raise

        action = {"x.vel": command.x, "y.vel": command.y, "theta.vel": command.turn}
        zero_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        start = session._clock()
        is_active = bool(command.x or command.y or command.turn)

        while session._clock() - start < command.seconds:
            if not session.dry_run:
                session.client.send_action(action)
                if is_active:
                    session._base_activity = True
            session._sleeper(session._control_period_s)

        if not session.dry_run:
            session.client.send_action(zero_action)

        # Release the base-lift stage so the next motion can start from READY.
        session.state.handle(StopCommand())

    def stop(self) -> None:
        """Dispatch an explicit all-zero base velocity command."""
        session = self._session
        if session is None:
            raise MotionNotEnabled("BaseHandle is not attached to a session")
        if not session.dry_run and hasattr(session.client, "send_action"):
            session.client.send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})


class AlohaMiniControl:
    """Session object returned by :func:`connect`.

    Holds the host client, exposes per-subsystem handles (left/right arm,
    lift, base, grippers), and coordinates the supervision state machine.
    """

    def __init__(
        self,
        client: RobotClient | Any,
        *,
        dry_run: bool,
        clock: Callable[[], float] | None = None,
        sleeper: Callable[[float], None] | None = None,
        ik_factory: Callable[[str], Any] | None = None,
        adapter: Any | None = None,
        control_period_s: float = 1.0 / 30.0,
        arrival_tolerance_m: float = 0.005,
        move_timeout_s: float = 10.0,
        precheck_step_m: float = DEFAULT_PRECHECK_STEP_M,
        feedback_timeout_s: float = 1.0,
        keepalive_interval_s: float | None = None,
        r_body_to_arm: Mapping[str, np.ndarray] | None = None,
        max_joint_step_deg: float = 2.0,
        follow_error_tolerance_deg: float = 1.0,
        follow_error_timeout_s: float = 1.0,
        settled_max_joint_delta_deg: float = _SETTLED_MAX_JOINT_DELTA_DEG,
        waypoint_lead_deg: float | None = None,
    ) -> None:
        """Bind the host client and build the per-subsystem handles."""
        self.client = client
        self.dry_run = dry_run
        self.state = ControlStateMachine()
        self.last_reason: str | None = None
        self.left = ArmHandle("left", session=self)
        self.right = ArmHandle("right", session=self)
        self.lift = LiftHandle(session=self)
        self.base = BaseHandle(session=self)
        self._disconnected = False
        self._base_activity = False

        # Derived-feedback state: last resolved host session, the last
        # measured kinematic configuration / lift height used to compute
        # standstill, and the last accepted sample id.
        self._last_resolved_session_id: str | None = None
        self._last_resolved_q: dict[str, np.ndarray] | None = None
        self._last_resolved_lift_mm: float | None = None
        self._last_resolved_sample_id: int | None = None

        self._clock = clock
        self._sleeper = sleeper
        self._ik_factory = ik_factory
        self._adapter = adapter
        self._control_period_s = float(control_period_s)
        self._arrival_tolerance_m = float(arrival_tolerance_m)
        self._move_timeout_s = float(move_timeout_s)
        self._precheck_step_m = float(precheck_step_m)
        self._feedback_timeout_s = float(feedback_timeout_s)
        self._keepalive_interval_s = keepalive_interval_s
        self._settled_max_joint_delta_deg = float(settled_max_joint_delta_deg)
        self._waypoint_lead_deg = (
            max(2.0 * float(max_joint_step_deg), float(follow_error_tolerance_deg))
            if waypoint_lead_deg is None
            else float(waypoint_lead_deg)
        )

        self._r_body_to_arm: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            r = (r_body_to_arm or {}).get(side)
            if r is None:
                self._r_body_to_arm[side] = np.eye(3, dtype=float)
            else:
                self._r_body_to_arm[side] = np.asarray(r, dtype=float)

        self._controllers: dict[str, CartesianController] = {}
        if (
            adapter is not None
            and ik_factory is not None
            and clock is not None
            and sleeper is not None
        ):
            for side in ("left", "right"):
                self._controllers[side] = CartesianController(
                    state_machine=self.state,
                    arm_ik=ik_factory(side),
                    adapter=adapter,
                    side=side,
                    clock=clock,
                    sender=client.send_action,
                    dry_run=dry_run,
                    max_joint_step_deg=float(max_joint_step_deg),
                    max_joint_lead_deg=30.0,
                    follow_error_tolerance_deg=float(follow_error_tolerance_deg),
                    follow_error_timeout_s=float(follow_error_timeout_s),
                    keepalive_interval_s=self._keepalive_interval_s,
                )

        if not dry_run:
            self._send_initial_hold()

    def _resolve_feedback(
        self,
        feedback: Mapping[str, Any],
        *,
        stage: Stage | None = None,
    ) -> ResolvedControlStatus:
        """Resolve a fail-closed control status from the observation and client side channels."""
        control_status_from_obs = feedback.get("_control_status")
        if control_status_from_obs is None:
            control_status = getattr(self.client, "latest_control_status", None) or {}
            derive_missing = True
        else:
            control_status = control_status_from_obs
            derive_missing = False

        metadata = feedback.get("_robot_metadata")
        if metadata is None:
            metadata = getattr(self.client, "latest_robot_metadata", None)

        sample_id = control_status.get("sample_sequence")
        if sample_id is None:
            sample_id = getattr(self.client, "observation_sequence", None)

        host_session_id = control_status.get("host_session_id")
        if host_session_id is None:
            host_session_id = getattr(self.client, "_host_session_id", None)

        explicit_settled = control_status.get("settled")
        explicit_valid = control_status.get("valid")

        # Host restart: discard stale evidence so the new session can qualify.
        if host_session_id is not None and host_session_id != self._last_resolved_session_id:
            self._last_resolved_session_id = host_session_id
            self._last_resolved_q = None
            self._last_resolved_lift_mm = None
            self._last_resolved_sample_id = None
            self.state.reset()
            for controller in self._controllers.values():
                controller.reset_feedback_baseline()

        # Freshness gate: only a forward sample_id is new motion evidence.
        fresh = True
        if sample_id is not None and self._last_resolved_sample_id is not None:
            fresh = sample_id > self._last_resolved_sample_id

        # Derive evidence quality from measured state.
        q_by_side: dict[str, np.ndarray] | None = None
        derived_valid = False
        derived_settled = False
        if fresh and metadata is not None and self._adapter is not None:
            q_by_side = {}
            derived_valid = True
            if hasattr(self._adapter, "_signs"):
                sides = list(self._adapter._signs)
            else:
                sides = list(self._controllers.keys()) or ["left", "right"]
            try:
                for side in sides:
                    q_by_side[side] = self._adapter.observation_to_kinematic_deg(
                        side, feedback, metadata
                    )
            except Exception:
                derived_valid = False
            if derived_valid and q_by_side:
                if not all(np.isfinite(q).all() for q in q_by_side.values()):
                    derived_valid = False
                if derived_valid and self._last_resolved_q is not None:
                    shared = [s for s in q_by_side if s in self._last_resolved_q]
                    if shared:
                        max_delta = max(
                            float(np.max(np.abs(q_by_side[s] - self._last_resolved_q[s])))
                            for s in shared
                        )
                        derived_settled = max_delta <= self._settled_max_joint_delta_deg
            if derived_valid and q_by_side is not None:
                self._last_resolved_q = {side: np.copy(q) for side, q in q_by_side.items()}
                if sample_id is not None:
                    self._last_resolved_sample_id = sample_id
        elif fresh and self._adapter is None and stage is Stage.BASE_LIFT:
            # Stage.BASE_LIFT without an arm kinematic adapter: derive validity
            # and standstill from lift height and base velocity keys when present.
            raw_lift = feedback.get("lift_axis.height_mm")
            base_vel_keys = ("x.vel", "y.vel", "theta.vel")
            present_vel_keys = [k for k in base_vel_keys if k in feedback]
            lift_val: float | None = None
            if raw_lift is not None and not isinstance(raw_lift, bool):
                try:
                    candidate = float(raw_lift)
                    if math.isfinite(candidate):
                        lift_val = candidate
                except (TypeError, ValueError):
                    lift_val = None
            vels_finite = True
            vels_still = True
            for k in present_vel_keys:
                raw_v = feedback[k]
                if isinstance(raw_v, bool):
                    vels_finite = False
                    break
                try:
                    fv = float(raw_v)
                except (TypeError, ValueError):
                    vels_finite = False
                    break
                if not math.isfinite(fv):
                    vels_finite = False
                    break
                if abs(fv) > _SETTLED_MAX_BASE_VEL:
                    vels_still = False

            if lift_val is not None and vels_finite:
                derived_valid = True
                lift_still = (
                    self._last_resolved_lift_mm is not None
                    and abs(lift_val - self._last_resolved_lift_mm) <= _SETTLED_MAX_LIFT_DELTA_MM
                )
                derived_settled = lift_still and vels_still
                self._last_resolved_lift_mm = lift_val
                if sample_id is not None:
                    self._last_resolved_sample_id = sample_id

        # Fail-closed: explicit claims are invalidated by non-finite/missing
        # evidence, and a stale/repeated sample is never fresh standstill.
        if derive_missing:
            settled = explicit_settled if explicit_settled is not None else derived_settled
            valid = explicit_valid if explicit_valid is not None else derived_valid
        else:
            settled = explicit_settled if explicit_settled is not None else False
            valid = explicit_valid if explicit_valid is not None else False
        if not fresh:
            settled = False
        if not derived_valid:
            valid = False
            settled = False

        return ResolvedControlStatus(
            sample_id=sample_id,
            host_session_id=host_session_id,
            settled=bool(settled),
            valid=bool(valid),
            metadata=metadata,
        )

    def _has_explicit_settled_valid(self, feedback: Mapping[str, Any]) -> bool:
        """Return True when the feedback channel carries explicit settled/valid claims."""
        control_status = feedback.get("_control_status")
        if control_status is None:
            control_status = getattr(self.client, "latest_control_status", None) or {}
        return "settled" in control_status and "valid" in control_status

    def _prime_state_machine(self, *, stage: Stage = Stage.ARM) -> None:
        """Prime the shared state machine from STANDBY to READY."""
        if self.state.state is not ControlState.STANDBY:
            return
        if self._clock is None or self._sleeper is None:
            raise MotionNotEnabled("motion requires an injected clock and sleeper")

        timeout = self._move_timeout_s
        deadline = self._clock() + timeout
        last_sample_id: int | None = None
        last_feedback_time = self._clock()

        while self.state.state is ControlState.STANDBY:
            if self._clock() >= deadline:
                raise MotionTimeout(
                    f"priming timed out after {timeout}s waiting for settled feedback"
                )

            try:
                feedback = _read_state_observation(self.client)
            except Exception:
                feedback = {}

            if (
                stage is Stage.ARM
                and self._adapter is None
                and not self._has_explicit_settled_valid(feedback)
            ):
                raise MotionNotEnabled(
                    "motion requires a kinematic adapter; connect() was called without one"
                )

            resolved = self._resolve_feedback(feedback, stage=stage)
            sample_id = resolved.sample_id

            if sample_id != last_sample_id:
                last_sample_id = sample_id
                last_feedback_time = self._clock()
            elif sample_id is not None:
                if self._clock() - last_feedback_time > self._feedback_timeout_s:
                    raise FeedbackLost(
                        f"no new feedback for {self._feedback_timeout_s}s (sample_id={sample_id})"
                    )

            if sample_id is not None:
                self.state.on_feedback(sample_id, resolved.settled, valid=resolved.valid)

            self.state.resume()
            self._sleeper(self._control_period_s)

    def _send_initial_hold(self) -> None:
        """Hold the current measured pose without solving IK."""
        if self._adapter is None:
            return
        try:
            feedback = _read_state_observation(self.client)
        except Exception:
            feedback = {}
        metadata = feedback.get("_robot_metadata")
        if metadata is None:
            metadata = getattr(self.client, "latest_robot_metadata", None)
        if metadata is None:
            return
        for side in ("left", "right"):
            try:
                q_deg = self._adapter.observation_to_kinematic_deg(side, feedback, metadata)
                if not np.all(np.isfinite(q_deg)):
                    continue
                action = self._adapter.kinematic_deg_to_action(side, q_deg, metadata)
                self.client.send_action(action)
            except Exception:
                pass

    def stop(self) -> None:
        """Stop the session without raising."""
        self.state.stop()

    def disconnect(self) -> None:
        """Close the host connection at most once."""
        if self._disconnected:
            return
        self._disconnected = True
        if not self.dry_run and self._base_activity and hasattr(self.client, "send_action"):
            self.client.send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
        if hasattr(self.client, "disconnect"):
            self.client.disconnect()

    def __enter__(self) -> AlohaMiniControl:
        """Return this session for use in a ``with`` statement."""
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        """Disconnect the session when the ``with`` block exits."""
        self.disconnect()


def _compute_moved(
    session: AlohaMiniControl,
    controller: CartesianController,
    side: str,
    initial_q: np.ndarray | None,
) -> float:
    """Return the Cartesian distance moved from ``initial_q`` to the latest pose."""
    if initial_q is None or session._adapter is None:
        return 0.0
    try:
        feedback = _read_state_observation(session.client)
        metadata = feedback.get("_robot_metadata") or getattr(
            session.client, "latest_robot_metadata", None
        )
        if metadata is None:
            return 0.0
        q_current = session._adapter.observation_to_kinematic_deg(side, feedback, metadata)
        initial_pose = controller._arm_ik.fk(initial_q)
        current_pose = controller._arm_ik.fk(q_current)
        return float(np.linalg.norm(current_pose[:3, 3] - initial_pose[:3, 3]))
    except Exception:
        return 0.0


def _default_client_factory(host: str, **kwargs: Any) -> Any:
    """连接真实主机的默认 client 工厂：本库不含 lerobot 运行时，恒不可用。

    Args:
        host: 真实主机地址（默认实现不再消费，保留签名供注入方对齐）。
        **kwargs: 透传给 client 的额外参数（同上，保留签名）。

    Raises:
        RuntimeUnavailable: 总是抛出；调用方需显式注入 ``client_factory``，
            或自行用 runtime extra 里的 ``AlohaMiniClient`` 构造后注入。
    """
    raise RuntimeUnavailable(
        f"no default client factory for host {host!r}: this library does not bundle "
        "the lerobot runtime; inject client_factory explicitly or build a client "
        "from the runtime extra yourself"
    )


def connect(
    host: str,
    *,
    dry_run: bool = True,
    enable_motion: bool = False,
    client_factory: Callable[..., Any] | None = None,
    clock: Callable[[], float] | None = None,
    sleeper: Callable[[float], None] | None = None,
    ik_factory: Callable[[str], Any] | None = None,
    adapter: Any | None = None,
    control_period_s: float = 1.0 / 30.0,
    arrival_tolerance_m: float = 0.005,
    move_timeout_s: float = 10.0,
    feedback_timeout_s: float = 1.0,
    keepalive_interval_s: float | None = None,
    r_body_to_arm: Mapping[str, np.ndarray] | None = None,
    max_joint_step_deg: float = 2.0,
    follow_error_tolerance_deg: float = 1.0,
    follow_error_timeout_s: float = 1.0,
    settled_max_joint_delta_deg: float = _SETTLED_MAX_JOINT_DELTA_DEG,
    waypoint_lead_deg: float | None = None,
    **kwargs: Any,
) -> AlohaMiniControl:
    """Open a control session to an AlohaMini host.

    The ``enable_motion`` gate is evaluated before any host I/O so that
    accidental non-dry-run connections fail closed.
    """
    if not dry_run and not enable_motion:
        raise MotionNotEnabled("dry_run=False connections require enable_motion=True")

    factory = client_factory if client_factory is not None else _default_client_factory
    client = factory(host, **kwargs)
    client.connect()
    return AlohaMiniControl(
        client,
        dry_run=dry_run,
        clock=clock,
        sleeper=sleeper,
        ik_factory=ik_factory,
        adapter=adapter,
        control_period_s=control_period_s,
        arrival_tolerance_m=arrival_tolerance_m,
        move_timeout_s=move_timeout_s,
        feedback_timeout_s=feedback_timeout_s,
        keepalive_interval_s=keepalive_interval_s,
        r_body_to_arm=r_body_to_arm,
        max_joint_step_deg=max_joint_step_deg,
        follow_error_tolerance_deg=follow_error_tolerance_deg,
        follow_error_timeout_s=follow_error_timeout_s,
        settled_max_joint_delta_deg=settled_max_joint_delta_deg,
        waypoint_lead_deg=waypoint_lead_deg,
    )
