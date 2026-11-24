#!/usr/bin/env python3
"""笛卡尔 ``ArmHandle.move_by`` / ``move_to`` 正式驱动 CLI（沉淀自现场手册 §10.7）。

三档运行模式：
  1. ``--dry-run``：纯离线 URDF + IK + 直线航点预检，零网络、零发送、不连设备；
  2. ``--limits-only``：连机只读（``dry_run=True``），打印 ``URDF ∩ 标定`` 有效限位窗口；
  3. 真实运动：必须显式传 ``--enable-motion`` + ``--signs-json`` + ``--zeros-json``，
     缺标定文件一律按退出码 ``15`` 拒绝，绝不伪造或猜测标定。

退出码：
  0: 成功
  2: 参数或配置错误（含缺失 ``--enable-motion``）
  10: IK 或路径预检拒绝（``OutOfRange``）
  11: 运动超时（``MotionTimeout``）
  12: 反馈失联（``FeedbackLost``）
  13: 运动被阻断（``MotionBlocked``）
  14: 运动未启用、缺少 ``lerobot`` / ``placo`` 运行时或连接失败
  15: 缺少或无效的 ``signs`` / ``zeros`` 标定文件
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from alohamini_cartesian.config.overlay import ConfigError, load_settings
from alohamini_cartesian.control import (
    FeedbackLost,
    MotionBlocked,
    MotionNotEnabled,
    MotionTimeout,
    OutOfRange,
    connect,
)
from alohamini_cartesian.control.api import _precheck_straight_path
from alohamini_cartesian.control.arm_ik import ArmIK, ArmKinematicProfile
from alohamini_cartesian.kinematics.joint_adapter import (
    JOINT_ORDER,
    URDF_JOINT_NAMES,
    JointKinematicAdapter,
)
from alohamini_cartesian.kinematics.model_specs import validate_robot_model

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_PRECHECK_REJECTED = 10
EXIT_MOTION_TIMEOUT = 11
EXIT_FEEDBACK_LOST = 12
EXIT_MOTION_BLOCKED = 13
EXIT_NOT_ENABLED_OR_RUNTIME = 14
EXIT_MISSING_CALIB = 15

ARRIVAL_TOLERANCE_M = 0.005

DEFAULT_TARGET_FRAME = "left_Fixed_Jaw"
DEFAULT_URDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "alohamini_cartesian"
    / "kinematics"
    / "assets"
    / "am_arm200"
    / "kinematics.urdf"
)
DEFAULT_SEED_Q_DEG: tuple[float, ...] = (10.0, -20.0, 30.0, -40.0, 20.0, -10.0)


def _load_flat_joint_json(path: Path | None, *, label: str) -> dict[str, float]:
    """加载扁平六键 JSON；路径缺失或字段非法抛 ``ValueError``。"""
    if path is None:
        raise ValueError(f"缺少必需的 {label} 标定文件（不得猜值或填默认值）")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取 {label} 文件 {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError(f"{label} 顶层必须是扁平六键 JSON 对象")
    out: dict[str, float] = {}
    for joint in JOINT_ORDER:
        if joint not in raw:
            raise ValueError(f"{label} 缺少关节键 {joint!r}")
        val = raw[joint]
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
            raise ValueError(f"{label}[{joint!r}] 必须是有限数值，拿到 {val!r}")
        out[joint] = float(val)
    return out


def _read_urdf_limits_deg(urdf_path: Path) -> dict[str, tuple[float, float]]:
    """直接从 URDF XML 读取 6 个手臂关节的限位（度），不依赖 ``placo``。"""
    root = ET.parse(urdf_path).getroot()
    by_urdf: dict[str, tuple[float, float]] = {}
    for joint_el in root.findall("joint"):
        name = joint_el.attrib.get("name")
        limit_el = joint_el.find("limit")
        if name is None or limit_el is None:
            continue
        lower = limit_el.attrib.get("lower")
        upper = limit_el.attrib.get("upper")
        if lower is None or upper is None:
            continue
        by_urdf[name] = (math.degrees(float(lower)), math.degrees(float(upper)))
    limits: dict[str, tuple[float, float]] = {}
    for joint in JOINT_ORDER:
        urdf_name = URDF_JOINT_NAMES[joint]
        if urdf_name not in by_urdf:
            raise ValueError(f"URDF {urdf_path} 缺少关节 {urdf_name!r} 的 <limit>")
        limits[joint] = by_urdf[urdf_name]
    return limits


def _build_arm_ik(urdf_path: Path = DEFAULT_URDF_PATH) -> ArmIK:
    """按本库 URDF 构造单臂 ``ArmIK`` 实例。"""
    profile = ArmKinematicProfile(
        urdf_path=str(urdf_path),
        target_frame=DEFAULT_TARGET_FRAME,
        joint_names=tuple(URDF_JOINT_NAMES[j] for j in JOINT_ORDER),
    )
    return ArmIK(profile)


def build_vendor_client_factory(
    *,
    robot_model: str,
    observation_request_window: int,
) -> Callable[..., Any]:
    """惰性导入厂商 ``AlohaMiniClient`` 的工厂构造器（模块顶层不 import ``lerobot``）。"""

    def factory(host: str, **kwargs: Any) -> Any:
        try:
            from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient
            from lerobot.robots.alohamini.config_alohamini import AlohaMiniClientConfig
        except ImportError as exc:
            raise RuntimeError(
                f"无法导入厂商运行时 lerobot.robots.alohamini（{exc}）："
                "注意 PyPI 上的 lerobot 不含 alohamini 模块，请使用 vendor fork 环境"
            ) from exc

        cfg = AlohaMiniClientConfig(
            remote_ip=host,
            robot_model=robot_model,
            observation_request_window=observation_request_window,
            **kwargs,
        )
        return AlohaMiniClient(cfg)

    return factory


def _parse_seed_q(raw: str) -> np.ndarray:
    """解析逗号分隔的 6 关节 seed 角（度）。"""
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) != len(JOINT_ORDER):
        raise ValueError(f"--q-seed-deg 需要 6 个角度，收到 {len(parts)} 个")
    arr = np.array([float(p) for p in parts], dtype=float)
    if not np.all(np.isfinite(arr)):
        raise ValueError("--q-seed-deg 包含非有限数值")
    return arr


def _run_offline_dry_run(
    *,
    dx: float,
    dy: float,
    dz: float,
    points: int,
    q_seed_deg: np.ndarray,
    identity_orientation: bool,
    max_joint_step_deg: float,
    urdf_path: Path,
) -> tuple[int, dict[str, Any]]:
    """纯离线预检：按 ``points`` 分段算 FK、插值与二分规划路径，零网络、零发送。"""
    ik = _build_arm_ik(urdf_path)
    start_pose = ik.fk(q_seed_deg)
    target_rot = np.eye(3, dtype=float) if identity_orientation else start_pose[:3, :3]
    total_delta = np.array([dx, dy, dz], dtype=float)

    class _UrdfLimitsAdapter:
        def effective_limits_deg(self, _side: str, _meta: Any) -> dict[str, tuple[float, float]]:
            return dict(ik._urdf_limits_deg)

    q_curr = np.copy(q_seed_deg)
    total_waypoints = 0
    max_hop = 0.0
    segment_reports: list[dict[str, Any]] = []

    for idx in range(1, points + 1):
        frac = idx / points
        seg_target = np.eye(4, dtype=float)
        seg_target[:3, :3] = target_rot
        seg_target[:3, 3] = start_pose[:3, 3] + total_delta * frac
        try:
            waypoints = _precheck_straight_path(
                ik,
                _UrdfLimitsAdapter(),
                "left",
                q_curr,
                seg_target,
                {},
                precheck_step_m=0.001,
                orientation_weight=0.01,
                max_joint_step_deg=max_joint_step_deg,
            )
        except OutOfRange as exc:
            return EXIT_PRECHECK_REJECTED, {
                "mode": "dry-run",
                "ok": False,
                "points": points,
                "failed_point": idx,
                "error": str(exc),
                "send_action_calls": 0,
            }

        prev = q_curr
        seg_max_hop = 0.0
        for _, q_wp in waypoints:
            seg_max_hop = max(seg_max_hop, float(np.max(np.abs(q_wp - prev))))
            prev = q_wp
        q_curr = waypoints[-1][1]
        end_pose = ik.fk(q_curr)
        seg_residual = float(np.linalg.norm(end_pose[:3, 3] - seg_target[:3, 3]))
        total_waypoints += len(waypoints)
        max_hop = max(max_hop, seg_max_hop)
        segment_reports.append(
            {
                "point": idx,
                "t": frac,
                "waypoint_count": len(waypoints),
                "max_joint_hop_deg": seg_max_hop,
                "residual_m": seg_residual,
                "q_deg": [float(v) for v in q_curr],
            }
        )

    final_target_xyz = start_pose[:3, 3] + total_delta
    final_pose = ik.fk(q_curr)
    final_residual_m = float(np.linalg.norm(final_pose[:3, 3] - final_target_xyz))
    return EXIT_OK, {
        "mode": "dry-run",
        "ok": True,
        "points": points,
        "orientation_mode": "identity" if identity_orientation else "hold",
        "start_xyz_m": [float(v) for v in start_pose[:3, 3]],
        "target_xyz_m": [float(v) for v in final_target_xyz],
        "waypoint_count": total_waypoints,
        "max_joint_hop_deg": max_hop,
        "final_q_deg": [float(v) for v in q_curr],
        "final_residual_m": final_residual_m,
        "segments": segment_reports,
        "send_action_calls": 0,
    }


def _emit_output(payload: dict[str, Any], *, out_path: Path | None, as_json: bool) -> None:
    """按参数写产物文件并在 stdout 输出 JSON 或可读摘要。"""
    if out_path is not None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    if as_json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return
    mode = payload.get("mode", "unknown")
    if mode == "dry-run":
        if payload.get("ok"):
            print(
                f"[dry-run] ok=True points={payload['points']} "
                f"orientation={payload['orientation_mode']} "
                f"waypoints={payload['waypoint_count']} "
                f"max_hop_deg={payload['max_joint_hop_deg']:.3f} "
                f"residual_m={payload['final_residual_m']:.6f} send_action_calls=0"
            )
        else:
            print(f"[dry-run] ok=False error={payload.get('error')}")
    elif mode == "limits-only":
        print(f"[limits-only] side={payload['side']} robot_model={payload['robot_model']}")
        for joint, bounds in payload["effective_limits_deg"].items():
            print(f"  {joint}: [{bounds[0]:.2f}, {bounds[1]:.2f}] deg")
    elif payload.get("exit_code") != EXIT_OK:
        print(
            f"[motion] {payload['phase']}: {payload['error']}: {payload['reason']}", file=sys.stderr
        )
    else:
        print(
            f"[motion] side={payload['side']} points={payload['points']} "
            f"orientation={payload['orientation_mode']} "
            f"moves={len(payload['moves'])} residual_m={payload['residual_m']:.6f} "
            f"steps={payload['steps']}"
        )
    if out_path is not None:
        print(f"[产物] {out_path}")


def _run_motion(
    arm: Any, args: argparse.Namespace, robot_model: str, clock: Callable[[], float]
) -> dict[str, Any]:
    """Execute absolute waypoints and retain measured progress on refusal or feedback loss."""
    moves: list[dict[str, Any]] = []
    payload: dict[str, Any] = {
        "mode": "motion",
        "side": args.side,
        "robot_model": robot_model,
        "points": args.points,
        "orientation_mode": "identity" if args.identity_orientation else "hold",
        "moves": moves,
        "steps": 0,
        "residual_m": None,
        "phase": "start_measurement",
        "start_xyz_m": None,
        "target_xyz_m": None,
        "final_xyz_m": None,
    }
    active_move = None
    try:
        start_pose = arm.measured_pose()
        start = start_pose[:3, 3]
        delta = np.array([args.dx, args.dy, args.dz])
        target = start + delta
        orientation = np.eye(3) if args.identity_orientation else start_pose[:3, :3]
        payload.update(start_xyz_m=start.tolist(), target_xyz_m=target.tolist(), phase="move")
        for idx in range(1, args.points + 1):
            waypoint = start + (idx / args.points) * delta
            active_move = {"point": idx, "target_xyz_m": waypoint.tolist()}
            moves.append(active_move)
            t0 = clock()
            report = arm.move_to(*waypoint, timeout_s=args.timeout_s, orientation=orientation)
            active_move.update(
                residual_m=report.residual_m, steps=len(report.steps), elapsed_s=clock() - t0
            )
            payload["steps"] += len(report.steps)
            active_move = None

        payload["phase"] = "final_measurement"
        end = arm.measured_pose()[:3, 3]
        residual = float(np.linalg.norm(end - target))
        payload.update(final_xyz_m=end.tolist(), residual_m=residual)
        if residual > ARRIVAL_TOLERANCE_M:
            raise MotionBlocked(
                f"measured endpoint residual {residual:.6f} m exceeds {ARRIVAL_TOLERANCE_M} m"
            )
        payload.update(phase="complete", exit_code=EXIT_OK)
    except (
        OutOfRange,
        MotionTimeout,
        FeedbackLost,
        MotionBlocked,
        MotionNotEnabled,
        RuntimeError,
        OSError,
    ) as exc:
        if isinstance(exc, OutOfRange):
            code = EXIT_PRECHECK_REJECTED
        elif isinstance(exc, MotionTimeout):
            code = EXIT_MOTION_TIMEOUT
        elif isinstance(exc, FeedbackLost):
            code = EXIT_FEEDBACK_LOST
        elif isinstance(exc, MotionBlocked):
            code = EXIT_MOTION_BLOCKED
        else:
            code = EXIT_NOT_ENABLED_OR_RUNTIME
        if active_move is not None:
            active_move.update(error=type(exc).__name__, reason=str(exc), elapsed_s=clock() - t0)
        payload.update(exit_code=code, error=type(exc).__name__, reason=str(exc))
        if payload["residual_m"] is None:
            payload["notes"] = "Final measured position and path residual unavailable."
    return payload


def _build_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description=(
            "AlohaMini 笛卡尔受控移动 CLI（支持纯离线 --dry-run、只读 --limits-only 与实机运动）。"
        )
    )
    parser.add_argument("--side", choices=("left", "right"), default="left", help="手臂侧别")
    parser.add_argument("--host", default="127.0.0.1", help="Pi Host 地址")
    parser.add_argument("--template", type=Path, default=None, help="配置模板 YAML")
    parser.add_argument("--site", type=Path, default=None, help="现场覆盖 YAML")
    parser.add_argument("--robot-model", default=None, help="覆盖配置中的 robot_model")
    parser.add_argument("--dx", type=float, default=0.0, help="X 总位移（米）")
    parser.add_argument("--dy", type=float, default=0.0, help="Y 总位移（米）")
    parser.add_argument("--dz", type=float, default=0.09, help="Z 总位移（米），默认 +0.09")
    parser.add_argument("--points", type=int, default=1, help="沿总位移等分的段数（默认 1）")
    parser.add_argument("--timeout-s", type=float, default=15.0, help="单段移动超时（秒）")
    parser.add_argument("--max-joint-step-deg", type=float, default=2.0, help="单周期关节步长上限")
    parser.add_argument(
        "--follow-error-tolerance-deg",
        "--follow-error-tol-deg",
        dest="follow_error_tolerance_deg",
        type=float,
        default=2.5,
        help="跟随误差容差（度），默认 2.5（对齐 D20 肩轴稳态残余）",
    )
    parser.add_argument(
        "--settled-max-joint-delta-deg",
        "--settled-delta-deg",
        dest="settled_max_joint_delta_deg",
        type=float,
        default=0.2,
        help="停稳判定关节变化阈值（度），默认 0.2（> 2 tick）",
    )
    parser.add_argument(
        "--waypoint-lead-deg",
        type=float,
        default=None,
        help="真机航点推进容差（度），默认沿用 max_joint_step_deg",
    )
    parser.add_argument(
        "--identity-orientation",
        action="store_true",
        help="显式要求转向世界单位姿态（默认保持起始末端姿态）",
    )
    parser.add_argument("--signs-json", type=Path, default=None, help="扁平六键 signs JSON")
    parser.add_argument("--zeros-json", type=Path, default=None, help="扁平六键 zeros JSON")
    parser.add_argument("--dry-run", action="store_true", help="纯离线预检，零网络、零发送")
    parser.add_argument("--limits-only", action="store_true", help="连机只读有效限位窗口，零发送")
    parser.add_argument("--enable-motion", action="store_true", help="显式确认允许真机运动")
    parser.add_argument(
        "--q-seed-deg",
        default=",".join(str(v) for v in DEFAULT_SEED_Q_DEG),
        help="纯离线 --dry-run 的起始六关节角（度）",
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH, help="URDF 文件路径")
    parser.add_argument("--out", type=Path, default=None, help="可选：JSON 报告落盘路径")
    parser.add_argument("--json", action="store_true", help="在 stdout 以纯 JSON 格式输出结果")
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    client_factory: Callable[..., Any] | None = None,
    clock: Callable[[], float] | None = None,
    sleeper: Callable[[float], None] | None = None,
) -> int:
    """命令行主入口。"""
    args = _build_parser().parse_args(argv)
    if args.points < 1:
        print(f"参数错误：--points 必须 >= 1，拿到 {args.points!r}", file=sys.stderr)
        return EXIT_USAGE

    numeric_names = (
        "dx",
        "dy",
        "dz",
        "timeout_s",
        "max_joint_step_deg",
        "follow_error_tolerance_deg",
        "settled_max_joint_delta_deg",
    )
    for name in numeric_names:
        val = getattr(args, name)
        if not math.isfinite(val):
            print(f"参数错误：--{name.replace('_', '-')} 必须有限，拿到 {val!r}", file=sys.stderr)
            return EXIT_USAGE
    if (
        args.timeout_s <= 0.0
        or args.max_joint_step_deg <= 0.0
        or args.follow_error_tolerance_deg <= 0.0
        or args.settled_max_joint_delta_deg <= 0.0
    ):
        print(
            "参数错误：timeout / max-joint-step / follow-error-tolerance / "
            "settled-max-joint-delta 必须为正数",
            file=sys.stderr,
        )
        return EXIT_USAGE
    if args.waypoint_lead_deg is not None and (
        not math.isfinite(args.waypoint_lead_deg) or args.waypoint_lead_deg <= 0.0
    ):
        print(
            f"参数错误：--waypoint-lead-deg 必须是有限正数，拿到 {args.waypoint_lead_deg!r}",
            file=sys.stderr,
        )
        return EXIT_USAGE

    try:
        settings = load_settings(template=args.template, site=args.site)
        robot_model = args.robot_model or settings.robot_model
        validate_robot_model(robot_model)
    except (ConfigError, ValueError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    if args.dry_run:
        try:
            q_seed = _parse_seed_q(args.q_seed_deg)
        except ValueError as exc:
            print(f"参数错误：{exc}", file=sys.stderr)
            return EXIT_USAGE
        try:
            code, payload = _run_offline_dry_run(
                dx=args.dx,
                dy=args.dy,
                dz=args.dz,
                points=args.points,
                q_seed_deg=q_seed,
                identity_orientation=args.identity_orientation,
                max_joint_step_deg=args.max_joint_step_deg,
                urdf_path=args.urdf,
            )
        except ImportError as exc:
            print(f"缺少运动学求解器依赖：{exc}", file=sys.stderr)
            return EXIT_NOT_ENABLED_OR_RUNTIME
        payload["exit_code"] = code
        _emit_output(payload, out_path=args.out, as_json=args.json)
        return code

    # Both --limits-only and live motion require real signs/zeros calibration files.
    try:
        signs = _load_flat_joint_json(args.signs_json, label="signs")
        zeros = _load_flat_joint_json(args.zeros_json, label="zeros")
        urdf_lim = _read_urdf_limits_deg(args.urdf)
        adapter = JointKinematicAdapter(
            signs={args.side: signs},
            zeros={args.side: zeros},
            urdf_limits={args.side: urdf_lim},
        )
    except ValueError as exc:
        print(f"标定缺失或非法：{exc}", file=sys.stderr)
        return EXIT_MISSING_CALIB

    if not args.limits_only and not args.enable_motion:
        print(
            "拒绝运动：真实运动必须显式传入 --enable-motion（或改用 --dry-run / --limits-only）",
            file=sys.stderr,
        )
        return EXIT_USAGE

    factory = client_factory or build_vendor_client_factory(
        robot_model=robot_model,
        observation_request_window=settings.observation_request_window,
    )
    run_clock = clock or time.monotonic
    run_sleeper = sleeper or time.sleep

    try:
        ik_factory = (lambda _side: _build_arm_ik(args.urdf)) if not args.limits_only else None
        session = connect(
            args.host,
            dry_run=bool(args.limits_only),
            enable_motion=bool(args.enable_motion and not args.limits_only),
            client_factory=factory,
            clock=run_clock,
            sleeper=run_sleeper,
            ik_factory=ik_factory,
            adapter=adapter,
            move_timeout_s=args.timeout_s,
            arrival_tolerance_m=ARRIVAL_TOLERANCE_M,
            max_joint_step_deg=args.max_joint_step_deg,
            follow_error_tolerance_deg=args.follow_error_tolerance_deg,
            settled_max_joint_delta_deg=args.settled_max_joint_delta_deg,
            waypoint_lead_deg=args.waypoint_lead_deg,
        )
    except (RuntimeError, ImportError, MotionNotEnabled) as exc:
        print(f"连接或运行时不可用：{exc}", file=sys.stderr)
        return EXIT_NOT_ENABLED_OR_RUNTIME

    try:
        if args.limits_only:
            prime = getattr(session.client, "prime_observation_request_window", None)
            if callable(prime):
                prime(include_cameras=False)
            obs = session.client.get_observation(include_cameras=False)
            meta = obs.get("_robot_metadata")
            if meta is None:
                meta = getattr(session.client, "latest_robot_metadata", None)
            if meta is None:
                print("对端未提供 metadata", file=sys.stderr)
                return EXIT_FEEDBACK_LOST
            limits = adapter.effective_limits_deg(args.side, meta)
            payload = {
                "mode": "limits-only",
                "side": args.side,
                "robot_model": robot_model,
                "effective_limits_deg": {k: [float(v[0]), float(v[1])] for k, v in limits.items()},
                "exit_code": EXIT_OK,
            }
            _emit_output(payload, out_path=args.out, as_json=args.json)
            return EXIT_OK

        arm = session.left if args.side == "left" else session.right
        payload = _run_motion(arm, args, robot_model, run_clock)
        _emit_output(payload, out_path=args.out, as_json=args.json)
        return payload["exit_code"]
    finally:
        session.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
