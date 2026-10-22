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

"""Pure-logic control state machine for AlohaMini command dispatch.

This module implements the B1–B7 supervision behaviors for the AlohaMini
command seam: the ``STANDBY`` start gate (B1.1), single-category motion
dispatch (B1.2/B2), stop/resume/release priority (B3/B9.1), explicit
refusal via :class:`MotionBlocked` (B1.5/B1.6/B7), fault latching (B5),
and explicit recovery through :meth:`ControlStateMachine.reset`.  It
contains no I/O, no clock, and no hardware coupling.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto


class ControlState(Enum):
    """High-level supervision state of the command state machine."""

    STANDBY = auto()
    READY = auto()
    ACTIVE = auto()
    SETTLING = auto()
    BLOCKED = auto()
    FAULT = auto()


class Stage(Enum):
    """Motion stage category for a single-category command."""

    ARM = auto()
    BASE_LIFT = auto()


class MotionBlocked(Exception):  # noqa: N818
    """Raised when a motion command is refused because the machine is blocked."""


class Side(Enum):
    """Arm side selector."""

    LEFT = auto()
    RIGHT = auto()


class GripperAction(Enum):
    """Gripper action selector."""

    OPEN = auto()
    CLOSE = auto()
    SET = auto()


@dataclass(frozen=True)
class ArmCommand:
    """Arm delta motion command."""

    side: Side
    dx: float
    dy: float
    dz: float

    def __post_init__(self) -> None:
        """Reject a non-finite delta on any axis."""
        for name in ("dx", "dy", "dz"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"arm {name} must be finite")


@dataclass(frozen=True)
class GripperCommand:
    """Gripper action command."""

    side: Side
    action: GripperAction
    position: float | None = None

    def __post_init__(self) -> None:
        """Require a finite position for the SET action."""
        if self.action is GripperAction.SET:
            if self.position is None:
                raise ValueError("SET gripper action requires a position")
            if not math.isfinite(self.position):
                raise ValueError("gripper position must be finite")
            if not (0 <= self.position <= 100):
                raise ValueError("gripper position must be in [0, 100]")
        elif self.position is not None:
            raise ValueError(f"gripper action {self.action.name} does not accept a position")


@dataclass(frozen=True)
class BaseCommand:
    """Base velocity command.

    An explicit zero velocity vector (``x=y=turn=0``) is still a BASE_LIFT
    category command and expresses a base stop in pure data.
    """

    x: float
    y: float
    turn: float
    seconds: float

    def __post_init__(self) -> None:
        """Reject non-finite velocities and negative or inconsistent durations."""
        for name in ("x", "y", "turn"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"base {name} must be finite")
        if not math.isfinite(self.seconds):
            raise ValueError("base seconds must be finite")
        if self.seconds < 0:
            raise ValueError("base seconds must be non-negative")
        if self.seconds == 0 and (self.x != 0 or self.y != 0 or self.turn != 0):
            raise ValueError("zero-duration base command requires zero velocity")


@dataclass(frozen=True)
class LiftCommand:
    """Lift delta motion command."""

    delta_mm: float

    def __post_init__(self) -> None:
        """Reject a non-finite lift delta."""
        if not math.isfinite(self.delta_mm):
            raise ValueError("lift delta_mm must be finite")


@dataclass(frozen=True)
class StopCommand:
    """Stop request (pure data; transition semantics belong to B1.3+)."""


@dataclass(frozen=True)
class ResumeCommand:
    """Resume request (pure data; transition semantics belong to B1.3+)."""


@dataclass(frozen=True)
class ReleaseCommand:
    """Release / complete the active action (pure data)."""


class ControlStateMachine:
    """Control state machine for AlohaMini command dispatch.

    The machine supervises motion requests across the six states
    ``STANDBY``, ``READY``, ``ACTIVE``, ``SETTLING``, ``BLOCKED`` and
    ``FAULT``.  It implements the B1–B7 behaviors: the ``STANDBY`` resume
    gate, single-category motion activation, cross-stage interlock,
    stop/release/resume handling, explicit refusal of illegal commands,
    fault latching, and reset recovery.

    It starts in ``STANDBY``.  It moves to ``READY`` only when
    :meth:`resume` is called and both gates are satisfied:

    1. A window of ``settled_samples`` consecutive *valid new* feedback samples
       reported ``settled=True``.
    2. The most recent feedback sample is valid.
    """

    def __init__(self, settled_samples: int = 3) -> None:
        """Start in STANDBY with no reason and an empty settled-sample window."""
        self._settled_samples = settled_samples
        self._state = ControlState.STANDBY
        self._active_stage: Stage | None = None
        self._last_reason: str | None = None
        self._last_valid = False
        self._consecutive_settled = 0
        self._seen_sample_ids: set[int] = set()
        self._blocked_released = False

    @property
    def state(self) -> ControlState:
        """Current supervision state."""
        return self._state

    @property
    def active_stage(self) -> Stage | None:
        """Stage currently being serviced, or ``None`` when idle."""
        return self._active_stage

    @property
    def last_reason(self) -> str | None:
        """Human-readable reason for the most recent refused transition."""
        return self._last_reason

    def on_feedback(self, sample_id: int, settled: bool, valid: bool = True) -> None:
        """Ingest a feedback sample.

        A sample is considered *new* only the first time its ``sample_id`` is
        seen.  Repeated or stale ids are ignored and do not advance the settle
        window.

        A never-seen ``sample_id`` that is below the accepted high-water mark
        indicates a severe feedback integrity violation.  In any motion-related
        state (``READY``, ``ACTIVE``, ``SETTLING`` or ``BLOCKED``) this latches
        the machine into ``FAULT``.  ``STANDBY`` ignores regressions because no
        motion is in flight, and ``FAULT`` is already latched.

        Args:
            sample_id: Monotonically increasing identifier for the feedback frame.
            settled: Whether the robot reports it has settled.
            valid: Whether the feedback frame itself is valid.
        """
        if self._state is ControlState.FAULT:
            return

        if sample_id in self._seen_sample_ids:
            return

        if (
            self._state
            in (
                ControlState.READY,
                ControlState.ACTIVE,
                ControlState.SETTLING,
                ControlState.BLOCKED,
            )
            and self._seen_sample_ids
            and sample_id < max(self._seen_sample_ids)
        ):
            self._state = ControlState.FAULT
            self._active_stage = None
            self._last_reason = (
                f"sample_id regression: {sample_id} below high-water mark "
                f"{max(self._seen_sample_ids)}"
            )
            return

        self._seen_sample_ids.add(sample_id)

        if not valid:
            self._consecutive_settled = 0
            self._last_valid = False
            return

        self._last_valid = True
        if settled:
            self._consecutive_settled += 1
        else:
            self._consecutive_settled = 0

        if (
            self._state is ControlState.SETTLING
            and self._consecutive_settled >= self._settled_samples
        ):
            self._state = ControlState.READY
            self._active_stage = None
            self._last_reason = None

        if (
            self._state is ControlState.BLOCKED
            and self._blocked_released
            and self._consecutive_settled >= self._settled_samples
        ):
            self._state = ControlState.READY
            self._active_stage = None
            self._blocked_released = False
            self._last_reason = None

    def resume(self) -> None:
        """Request transition from ``STANDBY`` to ``READY``.

        The transition succeeds only when called from ``STANDBY`` and both the
        settle window is full and the most recent feedback is valid.  If the
        machine is already ``READY`` this call is idempotent.  ``FAULT`` is
        latched and refuses resume until ``reset()`` is called.
        """
        if self._state is ControlState.READY:
            return

        if self._state is ControlState.FAULT:
            self._last_reason = "resume not allowed while FAULT"
            return

        if self._state is ControlState.ACTIVE:
            self._last_reason = "resume not allowed while ACTIVE"
            return

        if self._state is ControlState.SETTLING:
            self._last_reason = "resume not allowed while SETTLING"
            return

        if self._state is ControlState.BLOCKED:
            self._last_reason = "resume not allowed while BLOCKED"
            return

        if self._last_valid and self._consecutive_settled >= self._settled_samples:
            self._state = ControlState.READY
            self._last_reason = None
            return

        if not self._last_valid:
            self._last_reason = "latest feedback is not valid"
        elif self._consecutive_settled < self._settled_samples:
            self._last_reason = "not enough consecutive settled samples"
        else:
            self._last_reason = "resume gate not satisfied"

    def stop(self) -> None:
        """Return the machine to ``STANDBY``.

        ``FAULT`` is latched: stop() leaves the machine in ``FAULT`` so the
        fault reason remains visible and recovery must be explicit via
        ``reset()``.
        """
        if self._state is ControlState.FAULT:
            return

        self._state = ControlState.STANDBY
        self._active_stage = None
        self._consecutive_settled = 0
        self._last_valid = False
        self._last_reason = None
        self._blocked_released = False

    def reset(self) -> None:
        """Return the machine to ``STANDBY`` from any state.

        Clears all feedback evidence (seen sample ids, settle window,
        blocked-release state, and the last validity flag) so the machine
        must re-qualify through the ``STANDBY`` gate before becoming
        ``READY`` again.
        """
        self._state = ControlState.STANDBY
        self._active_stage = None
        self._consecutive_settled = 0
        self._last_valid = False
        self._last_reason = None
        self._blocked_released = False
        self._seen_sample_ids.clear()

    def block(self, reason: str) -> None:
        """Non-throwing entry point to latch the machine as ``BLOCKED``.

        From any state except ``FAULT`` the machine moves to ``BLOCKED``,
        clears the active stage, records ``reason`` as ``last_reason``,
        resets the settle window, and clears any pending release.  ``FAULT``
        is latched and remains unchanged so the fault reason stays visible.
        """
        if self._state is ControlState.FAULT:
            return

        self._state = ControlState.BLOCKED
        self._active_stage = None
        self._last_reason = reason
        self._consecutive_settled = 0
        self._blocked_released = False

    def _stage_for_command(self, command: object) -> Stage | None:
        """Return the motion stage category for a command, or ``None``."""
        if isinstance(command, (ArmCommand, GripperCommand)):
            return Stage.ARM
        if isinstance(command, (BaseCommand, LiftCommand)):
            return Stage.BASE_LIFT
        return None

    def handle(self, command: object) -> None:
        """Dispatch a command.

        From ``READY`` a single-category motion command moves the machine to
        ``ACTIVE`` with the matching stage.  A same-stage motion command while
        ``ACTIVE`` is allowed.  A cross-stage motion command while ``ACTIVE``
        moves the machine to ``BLOCKED``.  From ``ACTIVE`` a release command
        moves the machine to ``SETTLING``; from ``BLOCKED`` a release command
        arms the recovery window.  ``StopCommand`` moves the machine to
        ``STANDBY`` without raising, except from ``FAULT`` where the fault is
        latched.  ``ResumeCommand`` mirrors ``resume()``: it unlocks from
        ``STANDBY`` when the gate is satisfied, is idempotent in ``READY``, and
        is a silent refusal in all other states.  While ``FAULT``, all motion
        commands and ``ReleaseCommand`` are refused with :class:`MotionBlocked`.
        All other commands from all other states are refused with
        :class:`MotionBlocked`.
        """
        if isinstance(command, StopCommand):
            self.stop()
            return

        if isinstance(command, ResumeCommand):
            self.resume()
            return

        if self._state is ControlState.FAULT:
            raise MotionBlocked("machine is FAULT")

        if isinstance(command, ReleaseCommand):
            if self._state is ControlState.ACTIVE:
                self._state = ControlState.SETTLING
                self._active_stage = None
                self._consecutive_settled = 0
                self._last_reason = None
                return
            if self._state is ControlState.BLOCKED:
                self._blocked_released = True
                self._consecutive_settled = 0
                self._last_reason = "released; waiting for standstill confirmation"
                return
            raise MotionBlocked("release only valid from ACTIVE or BLOCKED")

        command_stage = self._stage_for_command(command)
        is_motion = command_stage is not None

        if self._state is ControlState.BLOCKED:
            raise MotionBlocked("machine is BLOCKED")

        if self._state is ControlState.SETTLING:
            raise MotionBlocked("machine is SETTLING")

        if self._state is ControlState.ACTIVE:
            if not is_motion:
                raise MotionBlocked("machine is ACTIVE")
            if command_stage is self._active_stage:
                return
            # Cross-stage conflict: block, clear the active stage, and report.
            active_stage = self._active_stage
            self._state = ControlState.BLOCKED
            self._active_stage = None
            self._consecutive_settled = 0
            self._blocked_released = False
            self._last_reason = (
                f"cross-stage conflict: requested {command_stage.name} "
                f"while active stage is {active_stage.name if active_stage else 'None'}"
            )
            raise MotionBlocked(self._last_reason)

        if self._state is not ControlState.READY:
            raise MotionBlocked("machine is not READY")

        # READY state.
        if is_motion:
            self._state = ControlState.ACTIVE
            self._active_stage = command_stage
            self._consecutive_settled = 0
            self._last_reason = None
            return

        # Non-motion, non-release commands from READY are refused explicitly.
        raise MotionBlocked("machine is READY: command not recognized")
