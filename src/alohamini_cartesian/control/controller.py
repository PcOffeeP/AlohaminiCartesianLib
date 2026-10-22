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

"""Cartesian motion controller for a single AlohaMini arm.

The controller is pure logic: each ``tick`` ingests feedback and a target
pose, asks the injected ``arm_ik`` seam for an IK solution, gates the
solution against the maximum allowed joint step from the last accepted
command, and forwards the adapter-converted action to the sender.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

import numpy as np

from ..kinematics.joint_adapter import JOINT_ORDER
from .commands import (
    ArmCommand,
    BaseCommand,
    ControlState,
    GripperCommand,
    LiftCommand,
    Stage,
)


class ControlError(Enum):
    """Controller-layer error codes returned inside ``CycleOutcome``."""

    STEP_LIMIT = "step_limit"
    NOT_ACTIVE = "not_active"
    IK_FAILED = "ik_failed"
    ADAPTER_FAILED = "adapter_failed"
    RESIDUAL = "residual"
    JOINT_LIMIT = "joint_limit"
    NON_FINITE = "non_finite"
    OVERCURRENT_HOLD = "overcurrent_hold"
    FOLLOW_TIMEOUT = "follow_timeout"
    UNSUPPORTED_TARGET = "unsupported_target"


@dataclass(frozen=True)
class CycleOutcome:
    """Result of one controller tick."""

    sent: bool
    action: Mapping[str, float] | None
    error_code: ControlError | None
    detail: str | None


@dataclass(frozen=True)
class ResolvedControlStatus:
    """Unified, fail-closed control-status seam used by api and controller."""

    sample_id: int | None = None
    host_session_id: str | None = None
    settled: bool = False
    valid: bool = False
    metadata: Mapping[str, Any] | None = None


class CartesianController:
    """Single-arm Cartesian controller seam.

    Args:
        state_machine: Supervision state machine; the controller only sends
            when it reports ``ACTIVE`` with ``active_stage == Stage.ARM``.
        arm_ik: Kinematic seam exposing ``solve(q_current_deg, target_pose,
            joint_limits_deg) -> IKResult``.
        adapter: ``JointKinematicAdapter`` used to convert observations to
            kinematic degrees and solutions back to motor commands.
        side: ``"left"`` or ``"right"``.
        clock: Injectable time source returning seconds as a float.
        sender: Injectable action sink accepting a motor-command mapping.
        dry_run: If ``True``, the controller computes the action and reports
            it but never calls ``sender``.
        max_joint_step_deg: Maximum allowed joint-space step from the last
            accepted command (inclusive boundary).
        max_joint_lead_deg: Reserved for future follow-error supervision;
            required at construction but unused in this cycle.
        follow_error_tolerance_deg: Reserved for future follow-error
            supervision; required at construction but unused in this cycle.
        follow_error_timeout_s: Reserved for future follow-error supervision;
            required at construction but unused in this cycle.
        keepalive_interval_s: Optional keepalive interval in seconds.  When
            set, each actual send arms ``next_deadline`` to ``send_time +
            keepalive_interval_s``.  ``None`` disables the deadline.
        orientation_weight: Weight forwarded to ``arm_ik.solve`` for the soft
            orientation goal (default 0.01).
    """

    def __init__(
        self,
        *,
        state_machine: Any,
        arm_ik: Any,
        adapter: Any,
        side: str,
        clock: Callable[[], float],
        sender: Callable[[Mapping], None],
        dry_run: bool = False,
        max_joint_step_deg: float = 2.0,
        max_joint_lead_deg: float,
        follow_error_tolerance_deg: float,
        follow_error_timeout_s: float,
        keepalive_interval_s: float | None = None,
        orientation_weight: float = 0.01,
    ) -> None:
        """Store the injected seams, limits and timing parameters."""
        self._machine = state_machine
        self._arm_ik = arm_ik
        self._adapter = adapter
        self._side = side
        self._clock = clock
        self._sender = sender
        self._dry_run = dry_run
        self._max_joint_step_deg = float(max_joint_step_deg)
        self._max_joint_lead_deg = float(max_joint_lead_deg)
        self._follow_error_tolerance_deg = float(follow_error_tolerance_deg)
        self._follow_error_timeout_s = float(follow_error_timeout_s)
        self._keepalive_interval_s = keepalive_interval_s
        self._orientation_weight = float(orientation_weight)

        # Last accepted kinematic joint configuration; ``None`` until the
        # first accepted solution.
        self._last_accepted_q: np.ndarray | None = None

        # Follow-error supervision: timestamp (from ``self._clock``) when the
        # tracking error first exceeded the tolerance, or ``None``.
        self._follow_error_start_time: float | None = None

        # Keepalive deadline: timestamp (from ``self._clock``) by which the
        # next command must be dispatched.  ``None`` until the first actual
        # send, and stays ``None`` in dry-run mode.
        self._next_deadline: float | None = None

        # Track the host feedback session so a session switch can reset the
        # feedback evidence baseline (sample-id high-water mark / seen set).
        self._host_session_id: str | None = None

    @property
    def next_deadline(self) -> float | None:
        """Timestamp of the next keepalive deadline, or ``None`` if unset."""
        return self._next_deadline

    def report_blocked(self, reason: str) -> None:
        """Non-throwing entry point to latch the state machine as BLOCKED."""
        self._machine.block(reason)

    def _send(self, action: Mapping[str, float]) -> None:
        """Dispatch ``action`` to the sender and arm the keepalive deadline."""
        if self._dry_run:
            return
        now = self._clock()
        self._sender(action)
        if self._keepalive_interval_s is not None:
            self._next_deadline = now + self._keepalive_interval_s

    def reset_feedback_baseline(self) -> None:
        """Drop session-bound feedback evidence (host restart recovery)."""
        self._host_session_id = None
        self._last_accepted_q = None
        self._follow_error_start_time = None
        self._next_deadline = None

    def _ingest_feedback(
        self,
        feedback: Mapping,
        resolved: ResolvedControlStatus | None = None,
    ) -> bool:
        """Feed the feedback sample into the supervision state machine.

        When ``resolved`` is supplied it is the single source of truth for
        ``sample_id``, ``host_session_id``, ``settled`` and ``valid``
        (design U5).  The production path through ``AlohaMiniControl`` always
        supplies a fail-closed ``resolved`` status.  For direct controller
        callers that omit ``resolved`` (legacy test seams), the observation-dict
        side channel is read and missing ``settled``/``valid`` default to
        ``True`` to preserve prior behavior.

        A change in ``host_session_id`` marks a new feedback session.  The
        controller resets the state machine so that the new session's first
        sample is not compared against the previous session's sample-id
        baseline (which would otherwise latch FAULT for a lower sequence
        number).  Within a session, a sample-id regression still latches FAULT
        as usual.

        Returns:
            ``True`` when the frame can be acted on.  When a session id is
            missing the sample-id semantics are still processed, but the caller
            must fail-closed and not use the frame for motion.
        """
        if resolved is not None:
            sample_id = resolved.sample_id
            session_id = resolved.host_session_id
            settled = resolved.settled
            valid = resolved.valid
        else:
            control_status = feedback.get("_control_status") or {}
            sample_id = control_status.get("sample_sequence")
            if sample_id is None:
                return True
            settled = control_status.get("settled", True)
            valid = control_status.get("valid", True)
            session_id = control_status.get("host_session_id")

        if sample_id is None:
            return True

        if session_id is None:
            # Fail-closed: do not treat a missing session id as a session
            # change, but still process the sample-id semantics.
            self._machine.on_feedback(sample_id, settled, valid=valid)
            return False

        if self._host_session_id is not None and session_id != self._host_session_id:
            self._machine.reset()
            self._last_accepted_q = None
            self._follow_error_start_time = None
            self._next_deadline = None
        self._host_session_id = session_id

        self._machine.on_feedback(sample_id, settled, valid=valid)
        return True

    def tick(
        self,
        feedback: Mapping,
        target: Any,
        resolved: ResolvedControlStatus | None = None,
    ) -> CycleOutcome:
        """Run one control cycle.

        Args:
            feedback: Observation mapping including the arm motor keys,
                ``_robot_metadata`` and ``_control_status``.
            target: Target motion request.  May be a base/lift command, an
                arm/gripper command (interlock only), ``None`` (base gap),
                or a 4x4 homogeneous pose for the arm Cartesian path.
            resolved: Optional pre-computed, fail-closed control status from
                ``AlohaMiniControl._resolve_feedback``.

        Returns:
            A frozen ``CycleOutcome`` describing the decision.
        """
        # Ingest feedback first so a sample-id regression latches FAULT before
        # any motion is computed or dispatched this cycle.
        if not self._ingest_feedback(feedback, resolved=resolved):
            return CycleOutcome(sent=False, action=None, error_code=None, detail=None)

        # Dispatch by the frozen seam: BaseCommand drives the base path, None is
        # a base gap, ArmCommand is interlock-only, and 4x4 poses use the arm path.
        if isinstance(target, BaseCommand):
            return self._tick_base_command(feedback, target)
        if target is None:
            return self._tick_base_gap(feedback)
        if isinstance(target, ArmCommand):
            return self._tick_arm_command(feedback, target)
        if isinstance(target, GripperCommand):
            return self._tick_gripper_command(feedback, target)
        if isinstance(target, LiftCommand):
            return self._tick_lift_command(feedback, target)
        if isinstance(target, np.ndarray):
            return self._tick_arm_pose(
                feedback, target, metadata=getattr(resolved, "metadata", None)
            )

        return CycleOutcome(
            sent=False,
            action=None,
            error_code=ControlError.UNSUPPORTED_TARGET,
            detail=None,
        )

    def _base_action(self, target: Any) -> dict[str, float]:
        """Return the alohamini base velocity action dict for a base target."""
        if target is None:
            return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        return {
            "x.vel": float(target.x),
            "y.vel": float(target.y),
            "theta.vel": float(target.turn),
        }

    def _tick_base_command(self, feedback: Mapping, target: Any) -> CycleOutcome:
        """Base-category command path.

        When already ACTIVE, the command is submitted to the state machine so
        a cross-stage conflict latches BLOCKED and raises ``MotionBlocked``.
        The requested velocity is sent only while ACTIVE(BASE_LIFT); every
        non-ACTIVE state receives an explicit zero-velocity safety command.
        """
        if self._machine.state is ControlState.ACTIVE:
            self._machine.handle(target)

        if (
            self._machine.state is ControlState.ACTIVE
            and self._machine.active_stage is Stage.BASE_LIFT
        ):
            action = self._base_action(target)
        else:
            action = self._base_action(None)

        self._send(action)

        return CycleOutcome(
            sent=not self._dry_run,
            action=action,
            error_code=None,
            detail=None,
        )

    def _tick_base_gap(self, feedback: Mapping) -> CycleOutcome:
        """Base gap path: explicit zero velocity unless ACTIVE(ARM).

        A ``None`` target during arm motion is a NOT_ACTIVE gap; during base
        motion or any non-ACTIVE state it is a safety stop.
        """
        if self._machine.state is ControlState.ACTIVE and self._machine.active_stage is Stage.ARM:
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.NOT_ACTIVE,
                detail="state machine is ACTIVE(ARM)",
            )

        action = self._base_action(None)
        self._send(action)

        return CycleOutcome(
            sent=not self._dry_run,
            action=action,
            error_code=None,
            detail=None,
        )

    def _tick_arm_command(self, feedback: Mapping, target: Any) -> CycleOutcome:
        """Arm-category command path: state-machine interlock only.

        The command is handed to the state machine; a cross-stage conflict
        raises ``MotionBlocked`` and zero is sent.  Same-stage commands are a
        no-op because the actual motion target is a subsequent 4x4 pose.
        """
        self._machine.handle(target)
        return CycleOutcome(
            sent=False,
            action=None,
            error_code=None,
            detail=None,
        )

    def _tick_gripper_command(self, feedback: Mapping, target: Any) -> CycleOutcome:
        """Gripper command path: ARM-category interlock only."""
        self._machine.handle(target)
        return CycleOutcome(
            sent=False,
            action=None,
            error_code=None,
            detail=None,
        )

    def _tick_lift_command(self, feedback: Mapping, target: Any) -> CycleOutcome:
        """Lift command path: BASE_LIFT-category interlock only."""
        self._machine.handle(target)
        return CycleOutcome(
            sent=False,
            action=None,
            error_code=None,
            detail=None,
        )

    def _tick_arm_pose(
        self,
        feedback: Mapping,
        target: Any,
        metadata: Mapping[str, Any] | None = None,
    ) -> CycleOutcome:
        """Cartesian arm pose path."""
        # 1. State gate: only ACTIVE(ARM) may send or solve.
        if (
            self._machine.state is not ControlState.ACTIVE
            or self._machine.active_stage is not Stage.ARM
        ):
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.NOT_ACTIVE,
                detail="state machine is not ACTIVE(ARM)",
            )

        if metadata is None:
            metadata = feedback.get("_robot_metadata")
        if metadata is None:
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.ADAPTER_FAILED,
                detail="feedback is missing _robot_metadata",
            )

        # 2. Convert feedback to kinematic degrees for the IK seed.
        try:
            q_seed = self._adapter.observation_to_kinematic_deg(self._side, feedback, metadata)
        except Exception as exc:  # noqa: BLE001
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.ADAPTER_FAILED,
                detail=str(exc),
            )

        # 3. Overcurrent hold gate: when the host reports a joint hold for
        #    this side, terminate the forward step, clear accumulated target
        #    reference, and emit a bounded retreat along the release direction.
        control_status = feedback.get("_control_status") or {}
        joint_holds = control_status.get("joint_holds", {})
        side_prefix = f"arm_{self._side}_"
        held: dict[str, Mapping[str, Any]] = {}
        for key, entry in joint_holds.items():
            if not isinstance(key, str) or not key.startswith(side_prefix):
                continue
            joint = key[len(side_prefix) :]
            if joint not in JOINT_ORDER or not isinstance(entry, Mapping):
                continue
            held[joint] = entry
        if held:
            return self._handle_overcurrent_hold(q_seed, held, metadata)

        # 4. Reference for the step gate: last accepted command, or the
        #    current feedback on the first cycle.
        reference = q_seed if self._last_accepted_q is None else self._last_accepted_q

        # 4.5 Follow-error timeout supervision (kinematic space).  The first
        # cycle has no dispatched command and is therefore not evaluated.
        if self._last_accepted_q is not None:
            follow_error = float(np.max(np.abs(q_seed - self._last_accepted_q)))
            now = self._clock()
            if follow_error > self._follow_error_tolerance_deg:
                if self._follow_error_start_time is None:
                    self._follow_error_start_time = now
                elif now - self._follow_error_start_time > self._follow_error_timeout_s:
                    return CycleOutcome(
                        sent=False,
                        action=None,
                        error_code=ControlError.FOLLOW_TIMEOUT,
                        detail=None,
                    )
            else:
                self._follow_error_start_time = None

        # 5. Solve IK with limits exposed by the adapter.
        try:
            joint_limits_deg = self._adapter.effective_limits_deg(self._side, metadata)
        except Exception as exc:  # noqa: BLE001
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.ADAPTER_FAILED,
                detail=str(exc),
            )

        try:
            result = self._arm_ik.solve(
                q_current_deg=q_seed,
                target_pose=target,
                joint_limits_deg=joint_limits_deg,
                orientation_weight=self._orientation_weight,
            )
        except Exception as exc:  # noqa: BLE001
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.IK_FAILED,
                detail=str(exc),
            )

        if not result.ok:
            error_code = getattr(ControlError, result.reason, ControlError.IK_FAILED)
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=error_code,
                detail=result.reason,
            )

        q_sol = np.asarray(result.q_deg, dtype=float)

        # 6. Residual gate (inclusive boundary): even when ArmIK reports
        #    ok=True, the controller re-checks against the configured limit.
        residual_limit_m = float(
            getattr(getattr(self._arm_ik, "profile", None), "residual_limit_m", 0.005)
        )
        if result.position_error_m > residual_limit_m:
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.RESIDUAL,
                detail=(
                    f"position_error_m {result.position_error_m} exceeds "
                    f"residual_limit_m {residual_limit_m}"
                ),
            )

        # 7. Step-limit gate (inclusive boundary).
        step = float(np.max(np.abs(q_sol - reference)))
        if step > self._max_joint_step_deg:
            # Rejected solutions must not update the reference.
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.STEP_LIMIT,
                detail=None,
            )

        # 8. Lead clamp (per joint, bounded by the measured feedback).
        #    Only values strictly outside ``feedback ± lead`` are clamped;
        #    the inclusive boundary is left untouched.
        lead_diff = q_sol - q_seed
        q_clamped = np.clip(
            q_sol,
            q_seed - self._max_joint_lead_deg,
            q_seed + self._max_joint_lead_deg,
        )
        if np.any(np.abs(lead_diff) > self._max_joint_lead_deg):
            q_sol = q_clamped
            detail = "joint lead clamped"
        else:
            detail = None

        # 9. Convert the accepted solution back to motor-space action.
        try:
            action = self._adapter.kinematic_deg_to_action(self._side, q_sol, metadata)
        except Exception as exc:  # noqa: BLE001
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.ADAPTER_FAILED,
                detail=str(exc),
            )

        # 10. Send unless this is a dry run.
        self._send(action)

        # 11. Update the accepted reference.
        self._last_accepted_q = np.copy(q_sol)

        return CycleOutcome(
            sent=not self._dry_run,
            action=action,
            error_code=None,
            detail=detail,
        )

    def _handle_overcurrent_hold(
        self,
        q_seed: np.ndarray,
        held: Mapping[str, Mapping[str, Any]],
        metadata: Mapping[str, Any],
    ) -> CycleOutcome:
        """Emit a bounded retreat for held joints and freeze the rest at feedback.

        The release direction is expressed in kinematic space:
        ``release_dir = -blocked_direction * adapter_sign(side, joint)``.
        ``blocked_direction=None`` is treated as fail-safe: no motion is sent.
        """
        self._last_accepted_q = None
        self._follow_error_start_time = None

        q_retreat = np.copy(q_seed)

        for joint, entry in held.items():
            blocked = entry.get("blocked_direction")
            if blocked not in (1.0, -1.0):
                return CycleOutcome(
                    sent=False,
                    action=None,
                    error_code=ControlError.OVERCURRENT_HOLD,
                    detail=f"unknown blocked_direction for {joint}",
                )
            index = JOINT_ORDER.index(joint)
            sign = self._adapter.sign(self._side, joint)
            release_dir = -float(blocked) * float(sign)
            q_retreat[index] += release_dir * self._max_joint_step_deg

        try:
            action = self._adapter.kinematic_deg_to_action(self._side, q_retreat, metadata)
        except Exception as exc:  # noqa: BLE001
            return CycleOutcome(
                sent=False,
                action=None,
                error_code=ControlError.ADAPTER_FAILED,
                detail=str(exc),
            )

        self._send(action)

        return CycleOutcome(
            sent=not self._dry_run,
            action=action,
            error_code=ControlError.OVERCURRENT_HOLD,
            detail=None,
        )
