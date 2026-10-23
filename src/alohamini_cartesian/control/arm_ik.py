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

"""Decoupled single-arm kinematics seam for AlohaMini.

This module holds the pure-data contracts (:class:`ArmKinematicProfile` and
:class:`IKResult`) and the :class:`ArmIK` entry point for forward/inverse
kinematics.  It imports without the optional ``placo`` dependency; the
numerical solver is only needed when calling :meth:`ArmIK.fk` or
:meth:`ArmIK.solve`.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np

FAILURE_REASONS = ("NON_FINITE", "JOINT_LIMIT", "RESIDUAL")

# Floating-point tolerance (degrees) used when classifying joint limits: it absorbs
# solver round-off of order 1e-14 deg for targets that sit exactly on a limit face,
# so only a deviation beyond this tolerance counts as a real limit violation.
LIMIT_EPS_DEG = 1e-9


@dataclass(frozen=True)
class IKResult:
    """Result of an inverse-kinematics solve attempt.

    The dataclass is immutable and validates its fields at construction time:
    ``ok`` is ``True`` exactly when ``reason`` is ``None``; ``reason`` must
    belong to the design vocabulary; and ``q_deg`` must be a length-6 vector.
    """

    q_deg: np.ndarray
    ok: bool
    reason: str | None
    position_error_m: float
    orientation_error_deg: float
    iterations: int
    solve_time_s: float

    def __post_init__(self) -> None:
        """Validate the ``ok``/``reason`` pairing and the ``q_deg`` shape."""
        if not isinstance(self.ok, bool):
            raise TypeError(f"ok must be a bool, got {type(self.ok).__name__}")

        if self.reason is None:
            if not self.ok:
                raise ValueError("ok=False with reason=None is contradictory")
        else:
            if self.ok:
                raise ValueError("ok=True with a failure reason is contradictory")
            if self.reason not in FAILURE_REASONS:
                raise ValueError(
                    f"reason must be None or one of {FAILURE_REASONS!r}, got {self.reason!r}"
                )

        q = np.asarray(self.q_deg)
        if q.shape != (6,):
            raise ValueError(f"q_deg must have shape (6,), got {q.shape}")


@dataclass(frozen=True)
class ArmKinematicProfile:
    """Static description of the arm kinematics problem.

    ``joint_names`` is stored as a frozen 6-tuple in URDF order.
    """

    urdf_path: str
    target_frame: str
    joint_names: tuple[str, ...]
    max_iterations: int = 5
    residual_limit_m: float = 0.005

    def __post_init__(self) -> None:
        """Freeze ``joint_names`` as a tuple and require exactly six joints."""
        joint_names = tuple(self.joint_names)
        object.__setattr__(self, "joint_names", joint_names)

        if len(joint_names) != 6:
            raise ValueError(f"joint_names must contain exactly 6 joints, got {len(joint_names)}")


def _require_placo() -> None:
    """Lazy guard: raise a guided ImportError when placo is unavailable."""
    try:
        import placo  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "placo is required for arm kinematics. Install the optional dependency "
            "placo, then retry."
        ) from exc


def _clamp_to_joint_limits(
    q_deg: np.ndarray,
    limits_deg: dict[str, tuple[float, float]],
    joint_names: tuple[str, ...],
) -> tuple[bool, np.ndarray]:
    """Clamp ``q_deg`` onto the joint limits and classify the result.

    A joint counts as within limits when it lies in the epsilon-widened closed
    interval ``[lo - LIMIT_EPS_DEG, hi + LIMIT_EPS_DEG]``; the tolerance absorbs
    solver round-off (~1e-14 deg) for targets sitting exactly on a limit face.

    Args:
        q_deg: Joint positions in degrees, shape ``(6,)``.  Never modified.
        limits_deg: Mapping from joint name to ``(min, max)`` in degrees.
        joint_names: Joint names in the same order as ``q_deg``.

    Returns:
        ``(within_limits, q_clamped)`` where ``within_limits`` is ``True`` exactly
        when every joint is inside the epsilon-widened interval and ``q_clamped``
        is a new vector with each component clamped onto ``[lo, hi]``.
    """
    values = np.asarray(q_deg, dtype=float)
    q_clamped = values.copy()
    within_limits = True
    for index, (name, value) in enumerate(zip(joint_names, values, strict=True)):
        lo_deg, hi_deg = limits_deg[name]
        if not (lo_deg - LIMIT_EPS_DEG <= value <= hi_deg + LIMIT_EPS_DEG):
            within_limits = False
        q_clamped[index] = min(max(value, lo_deg), hi_deg)

    return within_limits, q_clamped


class ArmIK:
    """Forward and inverse kinematics for a single AlohaMini arm.

    The constructor stores the profile, resolves the URDF path, and validates
    that every configured joint and the target frame are present in the URDF.
    The underlying placo solver is created lazily when a numerical method such
    as :meth:`fk` or :meth:`solve` is called.
    """

    def __init__(self, profile: ArmKinematicProfile) -> None:
        """Store the profile, resolve the URDF path and validate its joints/frame."""
        if not isinstance(profile, ArmKinematicProfile):
            raise TypeError(f"profile must be an ArmKinematicProfile, got {type(profile).__name__}")
        self.profile = profile

        urdf_path = Path(profile.urdf_path).expanduser()
        if not urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {profile.urdf_path}")

        # T-IK-10: validate that every required joint is declared in the URDF.
        # This needs placo; if the optional dependency is absent, surface the
        # same guided ImportError that fk()/solve() use (T-IK-12 compatibility).
        _require_placo()
        import placo  # noqa: F401

        self._urdf_path = str(urdf_path)
        robot = placo.RobotWrapper(self._urdf_path)
        available_joints = set(robot.joint_names())
        for name in profile.joint_names:
            if name not in available_joints:
                raise ValueError(f"URDF missing required joint: {name}")

        # F2: validate that the target frame exists in the URDF at construction
        # time, so a missing frame surfaces a clear ValueError instead of a
        # later placo RuntimeError inside fk()/solve().
        try:
            robot.get_T_world_frame(profile.target_frame)
        except RuntimeError as exc:
            raise ValueError(f"URDF missing target frame: {profile.target_frame}") from exc

    def fk(self, q_deg: np.ndarray) -> np.ndarray:
        """Compute the forward-kinematics pose of the target frame.

        Args:
            q_deg: Joint positions in degrees, shape ``(6,)``.

        Returns:
            4x4 homogeneous transform of the target frame in world coordinates.
        """
        _require_placo()
        import placo  # noqa: F811

        q_deg = np.asarray(q_deg, dtype=float)
        if q_deg.shape != (6,):
            raise ValueError(f"q_deg must have shape (6,), got {q_deg.shape}")

        q_rad = np.deg2rad(q_deg)
        robot = placo.RobotWrapper(self._urdf_path)
        for name, value in zip(self.profile.joint_names, q_rad, strict=True):
            robot.set_joint(name, float(value))
        robot.update_kinematics()

        return robot.get_T_world_frame(self.profile.target_frame)

    def _solve_once(
        self,
        q_current_deg: np.ndarray,
        target_pose: np.ndarray,
        limits_deg: dict[str, tuple[float, float]],
        orientation_weight: float,
    ) -> tuple[np.ndarray, float, float, int, float, np.ndarray]:
        """Run one IK attempt with the given limits and return raw solve data."""
        import placo  # noqa: F811

        robot = placo.RobotWrapper(self._urdf_path)
        solver = placo.KinematicsSolver(robot)
        solver.mask_fbase(True)

        for name in self.profile.joint_names:
            lo_deg, hi_deg = limits_deg[name]
            robot.set_joint_limits(name, float(np.deg2rad(lo_deg)), float(np.deg2rad(hi_deg)))

        q_seed_rad = np.deg2rad(q_current_deg)
        for name, value in zip(self.profile.joint_names, q_seed_rad, strict=True):
            robot.set_joint(name, float(value))
        robot.update_kinematics()

        tip = solver.add_frame_task(self.profile.target_frame, np.eye(4))
        tip.T_world_frame = target_pose
        tip.configure(self.profile.target_frame, "soft", 1.0, orientation_weight)

        start_s = perf_counter()
        position_error_m = float("inf")
        iterations = 0
        reached_pose = np.eye(4)
        for i in range(self.profile.max_iterations):
            solver.solve(True)
            robot.update_kinematics()
            reached_pose = robot.get_T_world_frame(self.profile.target_frame)
            position_error_m = float(np.linalg.norm(reached_pose[:3, 3] - target_pose[:3, 3]))
            iterations = i + 1
            if position_error_m <= self.profile.residual_limit_m:
                break
        solve_time_s = perf_counter() - start_s

        q_sol_deg = np.rad2deg(
            np.array([robot.get_joint(name) for name in self.profile.joint_names])
        )

        rotation_error = target_pose[:3, :3].T @ reached_pose[:3, :3]
        trace = float(np.trace(rotation_error))
        orientation_error_deg = float(
            np.degrees(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))
        )

        return (
            q_sol_deg,
            position_error_m,
            orientation_error_deg,
            iterations,
            solve_time_s,
            reached_pose,
        )

    def solve(
        self,
        q_current_deg: np.ndarray,
        target_pose: np.ndarray,
        joint_limits_deg: dict[str, tuple[float, float]],
        *,
        orientation_weight: float = 0.01,
    ) -> IKResult:
        """Solve inverse kinematics for the target frame.

        Args:
            q_current_deg: Current joint positions in degrees, shape ``(6,)``.
            target_pose: Desired 4x4 homogeneous transform.
            joint_limits_deg: Mapping from joint name to ``(min, max)`` in degrees.
            orientation_weight: Weight of the soft orientation goal relative to
                the position goal (keyword-only, default 0.01).

        Returns:
            An :class:`IKResult` describing the solution attempt.
        """
        _require_placo()
        import placo  # noqa: F811

        q_current_deg = np.asarray(q_current_deg, dtype=float)
        if q_current_deg.shape != (6,):
            raise ValueError(f"q_current_deg must have shape (6,), got {q_current_deg.shape}")
        target_pose = np.asarray(target_pose, dtype=float)
        if target_pose.shape != (4, 4):
            raise ValueError(f"target_pose must have shape (4, 4), got {target_pose.shape}")

        # T-IK-5: guard non-finite inputs before any numerical work so that
        # NaN/Inf never reaches the QP solver.
        start_s = perf_counter()
        if not np.all(np.isfinite(q_current_deg)) or not np.all(np.isfinite(target_pose)):
            q_return = (
                q_current_deg.copy()
                if np.all(np.isfinite(q_current_deg))
                else np.zeros(6, dtype=float)
            )
            solve_time_s = perf_counter() - start_s
            return IKResult(
                q_deg=q_return,
                ok=False,
                reason="NON_FINITE",
                position_error_m=float("inf"),
                orientation_error_deg=float("inf"),
                iterations=1,
                solve_time_s=solve_time_s,
            )

        # F2: validate that every required joint has an explicit limit entry
        # before any numerical work, surfacing a clear ValueError instead of a
        # bare KeyError deep in the solver.
        for name in self.profile.joint_names:
            if name not in joint_limits_deg:
                raise ValueError(f"joint_limits_deg missing required joint: {name}")

        # T-IK-4 classification: first solve with the URDF limits to establish
        # whether the target is physically reachable at all.  If the best
        # achievable residual is above the threshold, the target is genuinely
        # unreachable -> RESIDUAL.  This avoids misclassifying physically
        # unreachable targets whose solution happens to sit on a limit boundary.
        robot = placo.RobotWrapper(self._urdf_path)
        urdf_limits_deg = {
            name: tuple(np.rad2deg(robot.get_joint_limits(name)).tolist())
            for name in self.profile.joint_names
        }
        (
            q_urdf,
            position_error_urdf_m,
            orientation_error_urdf_deg,
            iterations_urdf,
            solve_time_urdf_s,
            _reached_urdf,
        ) = self._solve_once(q_current_deg, target_pose, urdf_limits_deg, orientation_weight)

        if position_error_urdf_m > self.profile.residual_limit_m:
            return IKResult(
                q_deg=q_urdf,
                ok=False,
                reason="RESIDUAL",
                position_error_m=position_error_urdf_m,
                orientation_error_deg=orientation_error_urdf_deg,
                iterations=iterations_urdf,
                solve_time_s=solve_time_urdf_s,
            )

        # Physically reachable; now solve again with the injected limits.  If
        # the solution cannot stay inside those limits while meeting the
        # residual, the failure is a joint-limit failure rather than residual.
        (
            q_sol_deg,
            position_error_m,
            orientation_error_deg,
            iterations,
            solve_time_s,
            _reached_pose,
        ) = self._solve_once(q_current_deg, target_pose, joint_limits_deg, orientation_weight)

        # T-IK-17: judge the limits with a small epsilon so a solution sitting on a
        # limit face is not rejected by its last float bit; the returned vector is
        # clamped onto [lo, hi] so no caller ever sees the 1e-14 deg overflow.
        within_limits, q_sol_deg = _clamp_to_joint_limits(
            q_sol_deg, joint_limits_deg, self.profile.joint_names
        )

        ok = position_error_m <= self.profile.residual_limit_m and within_limits
        if ok:
            return IKResult(
                q_deg=q_sol_deg,
                ok=True,
                reason=None,
                position_error_m=position_error_m,
                orientation_error_deg=orientation_error_deg,
                iterations=iterations,
                solve_time_s=solve_time_s,
            )

        return IKResult(
            q_deg=q_sol_deg,
            ok=False,
            reason="JOINT_LIMIT",
            position_error_m=position_error_m,
            orientation_error_deg=orientation_error_deg,
            iterations=iterations,
            solve_time_s=solve_time_s,
        )
