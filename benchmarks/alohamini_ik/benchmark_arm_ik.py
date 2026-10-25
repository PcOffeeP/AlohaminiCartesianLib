#!/usr/bin/env python
"""Offline IK speed benchmark for the AlohaMini arm.

Runs ``ArmIK.solve()`` against a deterministic pseudo-random target stream and
reports percentile solve/step latencies, residual quality, and joint-motion
smoothness.  Exit code 0 means the run completed within the performance budget.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np

from alohamini_cartesian.control.arm_ik import ArmIK, ArmKinematicProfile

# ---------------------------------------------------------------------------
# Constants matching the AM-ARM200 derived URDF.
# ---------------------------------------------------------------------------

JOINT_NAMES = [
    "left_shoulder_pan",
    "left_shoulder_lift",
    "left_elbow_flex",
    "left_wrist_flex",
    "left_wrist_yaw_joint",
    "left_wrist_roll",
]
TARGET_FRAME = "left_Fixed_Jaw"

DEFAULT_STEPS = 1000
DEFAULT_WARMUP = 10
DEFAULT_SEED = 0

RESIDUAL_LIMIT_M = 0.005
# Performance budget (30 Hz ⇒ p99 ≤ 7 ms solve, ≤ 15 ms per step). The judgement
# built on these numbers is the pure function `within_budget()` below.
SOLVE_BUDGET_MS = 7.0
STEP_BUDGET_MS = 15.0

# Roughly every 5th measurement sample is pushed out of reach.
UNREACHABLE_EVERY = 5
# Reach of the mini arm is well below 0.5 m; 1.0 m radial offset guarantees
# an unreachable target while still yielding a finite QP residual.
UNREACHABLE_OFFSET_M = 1.0

# Small perturbation scale for reachable targets (meters / radians).
POSITION_SIGMA_M = 0.01
ORIENTATION_SIGMA_RAD = 0.08


def _default_urdf() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return (
        repo_root
        / "src"
        / "alohamini_cartesian"
        / "kinematics"
        / "assets"
        / "am_arm200"
        / "kinematics.urdf"
    )


def _urdf_limits_deg(urdf_path: Path) -> dict[str, tuple[float, float]]:
    import placo  # type: ignore[import-not-found]

    robot = placo.RobotWrapper(str(urdf_path))
    return {
        name: tuple(float(v) for v in np.rad2deg(robot.get_joint_limits(name)))
        for name in JOINT_NAMES
    }


def _perturb_pose(
    pose: np.ndarray, rng: np.random.Generator, scale_m: float, scale_rad: float
) -> np.ndarray:
    """Return a new pose near ``pose`` with small position/rotation noise."""
    new_pose = pose.copy()
    new_pose[:3, 3] += rng.normal(0.0, scale_m, size=3)

    # Small angle-axis rotation around a random axis.
    axis = rng.normal(0.0, 1.0, size=3)
    axis /= np.linalg.norm(axis) + 1e-12
    angle = float(rng.normal(0.0, scale_rad))
    rot = _axis_angle_to_matrix(axis, angle)
    new_pose[:3, :3] = new_pose[:3, :3] @ rot
    return new_pose


def _axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' formula: rotation matrix from axis-angle."""
    x, y, z = axis / (np.linalg.norm(axis) + 1e-12)
    c = np.cos(angle)
    s = np.sin(angle)
    c1 = 1.0 - c
    return np.array(
        [
            [x * x * c1 + c, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, y * y * c1 + c, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, z * z * c1 + c],
        ]
    )


def _shortest_angle_delta(q_new_deg: np.ndarray, q_old_deg: np.ndarray) -> np.ndarray:
    """Return the shortest signed angular difference for each joint in degrees.

    Joints are treated as continuous, so a jump from -179° to +179° is 2°, not
    358°.
    """
    delta = q_new_deg - q_old_deg
    delta = ((delta + 180.0) % 360.0) - 180.0
    return delta


def _generate_target(
    current_pose: np.ndarray,
    rng: np.random.Generator,
    unreachable: bool,
) -> np.ndarray:
    """Generate one target pose from the current pose.

    Reachable targets are small position/orientation perturbations so the IK
    can re-use the current configuration as a seed.  Unreachable targets push
    the position far beyond the arm's workspace.
    """
    if unreachable:
        direction = rng.normal(0.0, 1.0, size=3)
        direction /= np.linalg.norm(direction) + 1e-12
        target = current_pose.copy()
        target[:3, 3] += direction * UNREACHABLE_OFFSET_M
        # Add small orientation noise so unreachable samples are not
        # suspiciously identical.
        return _perturb_pose(target, rng, 0.0, ORIENTATION_SIGMA_RAD)
    return _perturb_pose(current_pose, rng, POSITION_SIGMA_M, ORIENTATION_SIGMA_RAD)


def _percentile(values: list[float] | np.ndarray, pct: float) -> float:
    if len(values) == 0:
        return float("nan")
    return float(np.percentile(values, pct))


def within_budget(solve_p99_ms: float, step_p99_ms: float) -> bool:
    """Return whether the measured percentiles satisfy the performance budget.

    判据是纯函数：测试用构造耗时直接验证它；脚本自身仍真跑真计时，只把实测到的
    两个 p99 交给它判定。
    """
    return bool(solve_p99_ms <= SOLVE_BUDGET_MS and step_p99_ms <= STEP_BUDGET_MS)


def run_benchmark(urdf_path: Path, steps: int, warmup: int, seed: int) -> dict[str, Any]:
    """运行一次基准并返回 JSON 报告字典（键名即 CLI `--json` 的输出契约）。"""
    profile = ArmKinematicProfile(
        urdf_path=str(urdf_path),
        target_frame=TARGET_FRAME,
        joint_names=tuple(JOINT_NAMES),
        max_iterations=5,
        residual_limit_m=RESIDUAL_LIMIT_M,
    )
    arm = ArmIK(profile)
    limits_deg = _urdf_limits_deg(urdf_path)

    rng = np.random.default_rng(seed)

    solve_times_ms: list[float] = []
    step_times_ms: list[float] = []
    residuals_ok: list[float] = []
    failure_count = 0
    success_count = 0
    max_joint_delta_deg = 0.0
    max_reacquire_delta_deg = 0.0

    # Start from a neutral, known-valid configuration.
    q_current_deg = np.zeros(6, dtype=float)
    q_last_accepted_deg: np.ndarray | None = None
    q_pre_reset_accepted_deg: np.ndarray | None = None
    q_home_deg = q_current_deg.copy()

    total = warmup + steps
    for i in range(total):
        current_pose = arm.fk(q_current_deg)
        unreachable = i >= warmup and (i - warmup) % UNREACHABLE_EVERY == 0
        target_pose = _generate_target(current_pose, rng, unreachable)

        step_start_s = perf_counter()
        result = arm.solve(q_current_deg, target_pose, limits_deg)
        step_time_s = perf_counter() - step_start_s

        # Chain seed: use the previous solve result as the next seed.
        q_next_deg = np.asarray(result.q_deg, dtype=float)

        # Only measurement steps (after warmup) are recorded.
        if i < warmup:
            q_current_deg = q_next_deg.copy() if result.ok else q_home_deg.copy()
            continue

        solve_times_ms.append(result.solve_time_s * 1000.0)
        step_times_ms.append(step_time_s * 1000.0)

        if result.ok:
            success_count += 1
            residuals_ok.append(result.position_error_m)
            if q_last_accepted_deg is not None:
                # Consecutive accepted step: count toward the smoothness metric.
                delta = float(
                    np.max(np.abs(_shortest_angle_delta(q_next_deg, q_last_accepted_deg)))
                )
                if delta > max_joint_delta_deg:
                    max_joint_delta_deg = delta
            elif q_pre_reset_accepted_deg is not None:
                # Reacquired after a reset: count toward the cross-reset metric.
                delta = float(
                    np.max(np.abs(_shortest_angle_delta(q_next_deg, q_pre_reset_accepted_deg)))
                )
                if delta > max_reacquire_delta_deg:
                    max_reacquire_delta_deg = delta
            q_last_accepted_deg = q_next_deg.copy()
            q_pre_reset_accepted_deg = None
            q_current_deg = q_next_deg.copy()
        else:
            failure_count += 1
            # After an unreachable (or unexpectedly failing) target, reset the
            # seed to home so the stream stays in the reachable workspace, and
            # remember the last accepted solution so the cross-reset reacquire
            # delta can be reported separately.
            if q_last_accepted_deg is not None:
                q_pre_reset_accepted_deg = q_last_accepted_deg.copy()
            q_last_accepted_deg = None
            q_current_deg = q_home_deg.copy()

    solve_p50 = _percentile(solve_times_ms, 50.0)
    solve_p95 = _percentile(solve_times_ms, 95.0)
    solve_p99 = _percentile(solve_times_ms, 99.0)
    step_p99 = _percentile(step_times_ms, 99.0)

    # Runtime module-level lookup: the judge is the single source of truth for the
    # budget verdict (tests monkeypatch the module attribute to prove it).
    budget_ok = within_budget(solve_p99, step_p99)

    report: dict[str, Any] = {
        "urdf": str(urdf_path),
        "steps": steps,
        "warmup": warmup,
        "seed": seed,
        "solve_time_ms": {
            "p50": solve_p50,
            "p95": solve_p95,
            "p99": solve_p99,
        },
        "step_time_ms": {
            "p99": step_p99,
        },
        "max_residual_m": max(residuals_ok) if residuals_ok else 0.0,
        "max_joint_delta_deg": max_joint_delta_deg,
        "max_reacquire_delta_deg": max_reacquire_delta_deg,
        "residual_limit_m": RESIDUAL_LIMIT_M,
        "failure_count": failure_count,
        "success_count": success_count,
        "within_budget": budget_ok,
    }
    return report


def _human_summary(report: dict[str, Any]) -> str:
    solve = report["solve_time_ms"]
    step = report["step_time_ms"]
    lines = [
        "AlohaMini arm IK benchmark",
        f"  URDF: {report['urdf']}",
        f"  steps={report['steps']} warmup={report['warmup']} seed={report['seed']}",
        f"  solve_time_ms  p50={solve['p50']:.3f} p95={solve['p95']:.3f} p99={solve['p99']:.3f}",
        f"  step_time_ms   p99={step['p99']:.3f}",
        f"  max_residual_m={report['max_residual_m']:.6f} "
        f"max_joint_delta_deg={report['max_joint_delta_deg']:.3f} "
        f"max_reacquire_delta_deg={report['max_reacquire_delta_deg']:.3f}",
        f"  failure_count={report['failure_count']} success_count={report['success_count']}",
        f"  residual_limit_m={report['residual_limit_m']}",
        f"  within_budget={report['within_budget']}",
    ]
    return "\n".join(lines)


def main() -> int:
    """解析命令行参数、运行基准并返回进程退出码。"""
    parser = argparse.ArgumentParser(description="Benchmark AlohaMini ArmIK solve speed.")
    parser.add_argument(
        "--urdf", type=Path, default=_default_urdf(), help="Path to the kinematics URDF"
    )
    parser.add_argument(
        "--steps", type=int, default=DEFAULT_STEPS, help="Number of measured IK steps"
    )
    parser.add_argument("--warmup", type=int, default=DEFAULT_WARMUP, help="Number of warmup steps")
    parser.add_argument(
        "--seed", type=int, default=DEFAULT_SEED, help="Pseudo-random seed for the target stream"
    )
    parser.add_argument("--json", action="store_true", help="Emit a single JSON object on stdout")
    args = parser.parse_args()

    urdf_path = args.urdf.expanduser()
    if not urdf_path.is_file():
        msg = f"URDF not found: {urdf_path}"
        if args.json:
            print(json.dumps({"error": msg}), file=sys.stdout)
        print(msg, file=sys.stderr)
        return 1

    try:
        report = run_benchmark(urdf_path, args.steps, args.warmup, args.seed)
    except Exception as exc:  # noqa: BLE001
        msg = f"Benchmark failed: {exc}"
        if args.json:
            print(json.dumps({"error": msg}), file=sys.stdout)
        print(msg, file=sys.stderr)
        return 1

    summary = _human_summary(report)
    if args.json:
        print(json.dumps(report))
        print(summary, file=sys.stderr)
    else:
        print(summary)

    return 0 if report["within_budget"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
