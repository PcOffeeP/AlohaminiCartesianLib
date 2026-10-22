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
# kinematic state.  The 0.1° value is intentionally tight: it lets the test
# suite's zero-motion poses read as settled while treating any deliberate joint
# motion as moving.  On a real robot it must be re-tuned to sit just above the
# observed sensor/control-rate jitter rather than any fixed noise-floor claim.
_SETTLED_MAX_JOINT_DELTA_DEG = 0.1

# Consecutive controller ticks that report an adapter hard error before the
# move loop aborts with a reason-carrying MotionBlocked (D015) instead of
# timing out silently (B8.c).
_MAX_CONSECUTIVE_ADAPTER_FAILURES = 3


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
        session._prime_state_machine()
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
) -> None:
    """Lazy whole-path precheck: sample the straight line and verify IK.

    Samples ``i/n`` (endpoint included, start excluded) along the straight
    line from the current FK position to the target position, with
    ``n = max(1, ceil(distance / precheck_step_m))``.  Each waypoint is
    submitted to ``arm_ik.solve`` with the adapter's joint limits.  The first
    unreachable or limit-violating waypoint raises :class:`OutOfRange`
    carrying ``target_pose``.
    """
    current_pose = arm_ik.fk(q_current_deg)
    start = np.asarray(current_pose[:3, 3], dtype=float)
    end = np.asarray(target_pose[:3, 3], dtype=float)
    delta = end - start
    distance = float(np.linalg.norm(delta))
    n = max(1, int(math.ceil(distance / precheck_step_m)))

    joint_limits_deg = adapter.effective_limits_deg(side, metadata)

    for i in range(1, n + 1):
        t = i / n
        waypoint = np.eye(4, dtype=float)
        waypoint[:3, :3] = target_pose[:3, :3]
        waypoint[:3, 3] = start + t * delta

        result = arm_ik.solve(
            q_current_deg=q_current_deg,
            target_pose=waypoint,
            joint_limits_deg=joint_limits_deg,
            orientation_weight=orientation_weight,
        )
        if not result.ok:
            raise OutOfRange(
                f"precheck failed at waypoint {i}/{n}: {result.reason}",
                target=target_pose,
            )


class ArmHandle:
    """Handle to a single arm, including its gripper."""

    def __init__(self, side: str, session: AlohaMiniControl | None = None) -> None:
        """Bind the handle to ``side`` and, optionally, to a live session."""
        self.side = side
        self.gripper = GripperHandle(side, session=session)
        self._session = session

    def move_to(self, x: float, y: float, z: float, timeout_s: float | None = None) -> MoveReport:
        """Blocking Cartesian move to ``(x, y, z)`` in meters.

        Runs a 30 Hz loop (paced by ``session._sleeper``) that drives the
        real ``CartesianController`` until the measured FK position residual
        is within ``arrival_tolerance_m`` and the latest feedback sample
        reports ``settled=True``.  In dry-run mode no action is sent and a
        per-step diagnostic report is returned instead.

        Before the first action is dispatched the straight line from the
        current FK pose to the target is lazily pre-checked: every sampled
        waypoint must be IK-reachable and inside the joint limits.
        """
        session = self._session
        if session is None:
            raise MotionNotEnabled("ArmHandle is not attached to a session")

        controller = session._controllers.get(self.side)
        if controller is None:
            raise MotionNotEnabled(f"no controller available for {self.side}")

        target = np.eye(4, dtype=float)
        target[0, 3] = float(x)
        target[1, 3] = float(y)
        target[2, 3] = float(z)

        timeout = session._move_timeout_s if timeout_s is None else float(timeout_s)
        deadline = session._clock() + timeout
        side_enum = Side.LEFT if self.side == "left" else Side.RIGHT

        initial_q: np.ndarray | None = None
        steps: list[StepDiagnostic] = []
        last_residual = float("inf")
        prechecked = False
        last_action: Mapping[str, float] | None = None

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
                raise MotionTimeout(
                    f"move_to timed out after {timeout}s",
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
                feedback = session.client.get_observation()
            except Exception:
                feedback = {}

            resolved = session._resolve_feedback(feedback)
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

            metadata = feedback.get("_robot_metadata") or getattr(
                session.client, "latest_robot_metadata", None
            )

            # Extract the current kinematic configuration early: it seeds the
            # controller, feeds the arrival residual, and is required for the
            # lazy whole-path precheck before the first send.
            q_current: np.ndarray | None = None
            if metadata is not None and session._adapter is not None:
                try:
                    q_current = session._adapter.observation_to_kinematic_deg(
                        self.side, feedback, metadata
                    )
                    if initial_q is None:
                        initial_q = np.copy(q_current)
                except Exception:
                    q_current = None

            # Prime the shared state machine until the arm path is active.
            if session.state.state is ControlState.STANDBY:
                session.state.resume()
            if session.state.state is ControlState.READY:
                session.state.handle(ArmCommand(side=side_enum, dx=0.0, dy=0.0, dz=0.0))

            # Lazy whole-path precheck: run once the arm path becomes ACTIVE,
            # immediately before the controller is asked to solve/send the first
            # step.  This keeps the entry path free of eager IK solves required
            # by B2/B4 and guarantees zero on-wire actions for unreachable paths.
            if (
                session.state.state is ControlState.ACTIVE
                and session.state.active_stage is Stage.ARM
                and not prechecked
                and q_current is not None
                and session._adapter is not None
            ):
                _precheck_straight_path(
                    controller._arm_ik,
                    session._adapter,
                    self.side,
                    q_current,
                    target,
                    metadata,
                    session._precheck_step_m,
                    controller._orientation_weight,
                )
                prechecked = True

            outcome = controller.tick(feedback, target, resolved=resolved)

            # Fast-fail again in case the controller ingested a fault this tick.
            if session.state.state is ControlState.BLOCKED:
                reason = session.state.last_reason or "motion blocked"
                raise _MotionBlocked(reason)
            if session.state.state is ControlState.FAULT:
                reason = session.state.last_reason or "machine fault"
                session.last_reason = reason
                raise _MotionBlocked(reason)

            if outcome.error_code is ControlError.ADAPTER_FAILED:
                consecutive_adapter_failures += 1
                if consecutive_adapter_failures >= _MAX_CONSECUTIVE_ADAPTER_FAILURES:
                    detail = outcome.detail or "no detail"
                    raise _MotionBlocked(f"adapter failed ({outcome.error_code.value}): {detail}")
            else:
                consecutive_adapter_failures = 0

            if outcome.action is not None:
                steps.append(StepDiagnostic(action=dict(outcome.action)))
                last_action = outcome.action

            residual = float("inf")
            if q_current is not None:
                try:
                    current_pose = controller._arm_ik.fk(q_current)
                    residual = float(np.linalg.norm(current_pose[:3, 3] - target[:3, 3]))
                    last_residual = residual
                except Exception:
                    pass

            if (
                session.state.state is ControlState.ACTIVE
                and session.state.active_stage is Stage.ARM
                and residual <= session._arrival_tolerance_m
                and resolved.settled
                and resolved.valid
            ):
                # Release the ARM stage so the next motion category (lift/base)
                # can activate without a cross-stage conflict.  Feed feedback
                # until the machine settles back to READY or the move deadline
                # expires; returning while still SETTLING is accepted as a
                # best-effort timeout so the caller can still use the report.
                session.state.handle(ReleaseCommand())
                while session.state.state in (ControlState.ACTIVE, ControlState.SETTLING):
                    if session._clock() >= deadline:
                        break
                    try:
                        release_feedback = session.client.get_observation()
                    except Exception:
                        release_feedback = {}
                    release_resolved = session._resolve_feedback(release_feedback)
                    if release_resolved.sample_id is not None:
                        session.state.on_feedback(
                            release_resolved.sample_id,
                            release_resolved.settled,
                            valid=release_resolved.valid,
                        )
                    session._sleeper(session._control_period_s)
                return MoveReport(residual_m=last_residual, steps=steps)

            session._sleeper(session._control_period_s)

    def move_by(
        self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, timeout_s: float | None = None
    ) -> MoveReport:
        """Blocking Cartesian move by ``(dx, dy, dz)`` in meters.

        The increment is interpreted in the body frame, rotated per side by
        ``session._r_body_to_arm[side]``, and added to the current measured
        FK pose.  The resulting target is then passed to :meth:`move_to`.
        """
        session = self._session
        if session is None:
            raise MotionNotEnabled("ArmHandle is not attached to a session")
        controller = session._controllers.get(self.side)
        if controller is None or session._adapter is None:
            raise MotionNotEnabled(f"no controller available for {self.side}")

        try:
            feedback = session.client.get_observation()
            metadata = feedback.get("_robot_metadata") or getattr(
                session.client, "latest_robot_metadata", None
            )
            if metadata is None:
                raise FeedbackLost("feedback is missing _robot_metadata")
            q_current = session._adapter.observation_to_kinematic_deg(self.side, feedback, metadata)
        except Exception as exc:
            raise FeedbackLost(f"cannot read current pose for {self.side}: {exc}") from exc

        current_pose = controller._arm_ik.fk(q_current)
        current_trans = np.asarray(current_pose[:3, 3], dtype=float)

        delta = np.array([float(dx), float(dy), float(dz)], dtype=float)
        r = session._r_body_to_arm.get(self.side)
        if r is not None:
            delta = np.asarray(r, dtype=float) @ delta

        target = current_trans + delta
        return self.move_to(
            float(target[0]), float(target[1]), float(target[2]), timeout_s=timeout_s
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
            feedback = session.client.get_observation()
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
        session._prime_state_machine()
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
        session._prime_state_machine()
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

    Holds the host client, exposes per-subsystem handles, and manages the
    control lifecycle.  This is intentionally a skeleton for P5 cycle 1:
    motion methods will be added in later cycles.
    """

    def __init__(
        self,
        client: Any,
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
        # measured kinematic configuration used to compute standstill, and the
        # last accepted sample id so repeated/frozen frames do not fake arrival.
        self._last_resolved_session_id: str | None = None
        self._last_resolved_q: dict[str, np.ndarray] | None = None
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
                    max_joint_lead_deg=30.0,
                    follow_error_tolerance_deg=1.0,
                    follow_error_timeout_s=1.0,
                    keepalive_interval_s=self._keepalive_interval_s,
                )

        if not dry_run:
            self._send_initial_hold()

    def _resolve_feedback(self, feedback: Mapping[str, Any]) -> ResolvedControlStatus:
        """Resolve a fail-closed control status from the production observation form.

        Priority (U1):

        1. ``_control_status`` / ``_robot_metadata`` inside the observation dict.
        2. The client's committed side-channel attributes
           (``latest_control_status`` / ``latest_robot_metadata`` /
           ``observation_sequence`` / ``_host_session_id``).

        ``settled`` and ``valid`` are derived from the measured kinematic
        evidence when the side channel does not supply them:

        * ``valid`` -- the frame provides finite joint positions for every
          configured arm side.
        * ``settled`` -- the joint positions did not change by more than
          ``_SETTLED_MAX_JOINT_DELTA_DEG`` from the previous sample of the
          same host session.

        A host-session change discards the cached kinematic baseline and
        resets the shared state machine so restarted sample sequences can
        re-qualify (B6).
        """
        # Side-channel priority (U1): observation-dict fields are authoritative
        # when present.  When the observation dict carries no ``_control_status``
        # at all (production ``AlohaMiniClient`` form), missing ``settled``/
        # ``valid`` are derived from measured kinematics; otherwise missing
        # fields are fail-closed ``False``.
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
        # This must run before the freshness gate so a restarted sample-id
        # sequence is not rejected as stale (c3-f4-2 guard).
        if host_session_id is not None and host_session_id != self._last_resolved_session_id:
            self._last_resolved_session_id = host_session_id
            self._last_resolved_q = None
            self._last_resolved_sample_id = None
            self.state.reset()
            for controller in self._controllers.values():
                controller.reset_feedback_baseline()

        # Freshness gate (B9): only a forward sample_id is new motion evidence.
        # A repeated/frozen sample must not be treated as a fresh standstill.
        fresh = True
        if sample_id is not None and self._last_resolved_sample_id is not None:
            fresh = sample_id > self._last_resolved_sample_id

        # Derive evidence quality from measured kinematic state.
        q_by_side: dict[str, np.ndarray] | None = None
        derived_valid = False
        derived_settled = False
        if fresh and metadata is not None and self._adapter is not None:
            q_by_side = {}
            derived_valid = True
            # Adapters may expose their configured sides via ``_signs``
            # (JointKinematicAdapter); fall back to the controller set otherwise.
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
                        derived_settled = max_delta <= _SETTLED_MAX_JOINT_DELTA_DEG

        if fresh and derived_valid and q_by_side is not None:
            self._last_resolved_q = {side: np.copy(q) for side, q in q_by_side.items()}
            if sample_id is not None:
                self._last_resolved_sample_id = sample_id

        # Fail-closed: explicit claims are invalidated by non-finite/missing
        # evidence (U2 / U5).  A stale/repeated sample is never fresh standstill
        # evidence, even if the host side-channel claims settled (B9).
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
        """Return True when the feedback channel carries explicit settled/valid claims.

        Mirrors the side-channel priority in ``_resolve_feedback``: an
        observation-dict ``_control_status`` is authoritative when present,
        otherwise the client's committed side-channel attribute is consulted.
        """
        control_status = feedback.get("_control_status")
        if control_status is None:
            control_status = getattr(self.client, "latest_control_status", None) or {}
        return "settled" in control_status and "valid" in control_status

    def _prime_state_machine(self) -> None:
        """Prime the shared state machine from STANDBY to READY.

        Polls feedback, feeds the state machine via ``on_feedback``, and calls
        ``resume()`` until the 3-settled-sample gate opens.  Bounded by
        ``_move_timeout_s`` and ``_feedback_timeout_s``.  Dry-run safe: no
        actions are dispatched during priming.

        Raises:
            MotionNotEnabled: when no clock/sleeper is available (B7).
        """
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
                feedback = self.client.get_observation()
            except Exception:
                feedback = {}

            # C4b: without a kinematic adapter ``_resolve_feedback`` fail-closes
            # settled/valid to False, so the motion gate can never open.  When the
            # feedback carries no explicit settled/valid claim, fail fast with a
            # reason that names the missing adapter instead of spinning to a
            # confusing MotionTimeout.  An adapter-less session that *does* carry an
            # explicit settled/valid claim skips this guard, but ``_resolve_feedback``
            # still overwrites settled/valid to False unconditionally (``derived_valid``
            # requires an adapter), so that session likewise ends in MotionTimeout --
            # without the adapter-naming message.  Whether such an explicit-claim
            # adapter-less path should be supported is an open contract question;
            # today it is not.
            if self._adapter is None and not self._has_explicit_settled_valid(feedback):
                raise MotionNotEnabled(
                    "motion requires a kinematic adapter; connect() was called without one"
                )

            resolved = self._resolve_feedback(feedback)
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
        """Hold the current measured pose without solving IK (B4 / D005)."""
        if self._adapter is None:
            return
        try:
            feedback = self.client.get_observation()
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
        feedback = session.client.get_observation()
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
    **kwargs: Any,
) -> AlohaMiniControl:
    """Open a control session to an AlohaMini host.

    The ``enable_motion`` gate is evaluated before any host I/O so that
    accidental non-dry-run connections fail closed.

    Args:
        host: Host address passed to ``client_factory``.
        dry_run: When ``True``, no real motion is commanded.
        enable_motion: Must be ``True`` for non-dry-run connections.
        client_factory: Injectable factory callable ``(host, **kwargs) -> client``.
            When omitted, the session fails closed with ``RuntimeUnavailable``:
            this library does not bundle the lerobot runtime.
        clock: Injectable monotonic clock callable; required for paced motion.
        sleeper: Injectable sleep callable; required for paced motion.
        ik_factory: Injectable factory ``(side) -> arm_ik`` used by the solver.
        adapter: Injected kinematics adapter used to convert feedback/actions.
        control_period_s: Loop period in seconds for the paced motion loops.
        arrival_tolerance_m: Cartesian arrival tolerance in meters.
        move_timeout_s: Overall timeout in seconds for one blocking move.
        feedback_timeout_s: Seconds without a new feedback sample before abort.
        keepalive_interval_s: Optional keepalive period in seconds, or ``None``.
        r_body_to_arm: Optional per-side body-to-arm rotation matrices.
        **kwargs: Forwarded to ``client_factory``.

    Returns:
        An :class:`AlohaMiniControl` session.

    Raises:
        MotionNotEnabled: If ``dry_run=False`` and ``enable_motion=False``.
        RuntimeUnavailable: 未注入 ``client_factory`` 时；本库不含 lerobot 运行时，
            默认工厂恒不可用。
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
    )
