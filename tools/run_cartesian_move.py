#!/usr/bin/env python3
"""笛卡尔 ``ArmHandle.move_by`` / ``move_to`` 正式驱动 CLI（沉淀自现场手册 §10.7）。

三档运行模式：
  1. ``--dry-run``：纯离线 URDF + IK + 直线航点预检，零网络、零发送、不连设备；
  2. ``--limits-only``：连机只读（``dry_run=True``），打印 ``effective_limits_deg`` 窗口；
  3. 真实运动：必须显式传 ``--enable-motion`` + ``--signs-json`` + ``--zeros-json``，
     缺标定文件一律按退出码 ``15`` 拒绝，绝不伪造或猜测标定。

退出码：
  0: 成功
  2: 参数或配置错误
  3: 运行时/连接失败
  10: IK 或路径预检拒绝（``OutOfRange``）
  11: 运动超时（``MotionTimeout``）
  12: 反馈失联或状态机阻断（``FeedbackLost`` / ``MotionBlocked``）
  15: 缺少或无效的 ``signs`` / ``zeros`` 标定文件
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
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
EXIT_CONNECT_FAILED = 3
EXIT_PRECHECK_REJECTED = 10
EXIT_MOTION_TIMEOUT = 11
EXIT_FEEDBACK_OR_BLOCKED = 12
EXIT_MISSING_CALIB = 15

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
    q_seed_deg: np.ndarray,
    identity_orientation: bool,
    max_joint_step_deg: float,
    urdf_path: Path,
) -> tuple[int, dict[str, Any]]:
    """纯离线预检：算 FK、按航点插值与二分规划路径，零网络、零发送。"""
    ik = _build_arm_ik(urdf_path)
    start_pose = ik.fk(q_seed_deg)
    target = np.eye(4, dtype=float)
    target[:3, :3] = np.eye(3, dtype=float) if identity_orientation else start_pose[:3, :3]
    target[:3, 3] = start_pose[:3, 3] + np.array([dx, dy, dz], dtype=float)

    class _UrdfLimitsAdapter:
        def effective_limits_deg(self, _side: str, _meta: Any) -> dict[str, tuple[float, float]]:
            return dict(ik._urdf_limits_deg)

    try:
        waypoints = _precheck_straight_path(
            ik,
            _UrdfLimitsAdapter(),
            "left",
            q_seed_deg,
            target,
            {},
            precheck_step_m=0.001,
            orientation_weight=0.01,
            max_joint_step_deg=max_joint_step_deg,
        )
    except OutOfRange as exc:
        return EXIT_PRECHECK_REJECTED, {
            "mode": "dry-run",
            "ok": False,
            "error": str(exc),
            "send_action_calls": 0,
        }

    q_end = waypoints[-1][1]
    max_hop = 0.0
    prev = q_seed_deg
    for _, q_wp in waypoints:
        max_hop = max(max_hop, float(np.max(np.abs(q_wp - prev))))
        prev = q_wp
    end_pose = ik.fk(q_end)
    residual_m = float(np.linalg.norm(end_pose[:3, 3] - target[:3, 3]))
    return EXIT_OK, {
        "mode": "dry-run",
        "ok": True,
        "orientation_mode": "identity" if identity_orientation else "hold",
        "start_xyz_m": [float(v) for v in start_pose[:3, 3]],
        "target_xyz_m": [float(v) for v in target[:3, 3]],
        "waypoint_count": len(waypoints),
        "max_joint_hop_deg": max_hop,
        "final_q_deg": [float(v) for v in q_end],
        "final_residual_m": residual_m,
        "send_action_calls": 0,
    }


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
    parser.add_argument("--dx", type=float, default=0.0, help="X 位移（米）")
    parser.add_argument("--dy", type=float, default=0.0, help="Y 位移（米）")
    parser.add_argument("--dz", type=float, default=0.09, help="Z 位移（米），默认 +0.09")
    parser.add_argument("--timeout-s", type=float, default=15.0, help="单次移动超时（秒）")
    parser.add_argument("--max-joint-step-deg", type=float, default=2.0, help="单周期关节步长上限")
    parser.add_argument(
        "--follow-error-tol-deg",
        type=float,
        default=2.5,
        help="跟随误差容差（度），默认 2.5（对齐 D20 肩轴稳态残余）",
    )
    parser.add_argument(
        "--settled-delta-deg",
        type=float,
        default=0.2,
        help="停稳判定关节变化阈值（度），默认 0.2（> 2 tick）",
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
    parser.add_argument("--json", action="store_true", help="以 JSON 格式输出结果")
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
    for name in ("dx", "dy", "dz", "timeout_s", "max_joint_step_deg", "follow_error_tol_deg"):
        val = getattr(args, name)
        if not math.isfinite(val):
            print(f"参数错误：--{name.replace('_', '-')} 必须有限，拿到 {val!r}", file=sys.stderr)
            return EXIT_USAGE
    if args.timeout_s <= 0.0 or args.max_joint_step_deg <= 0.0 or args.follow_error_tol_deg <= 0.0:
        print("参数错误：timeout / step / follow-error 必须为正数", file=sys.stderr)
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
                q_seed_deg=q_seed,
                identity_orientation=args.identity_orientation,
                max_joint_step_deg=args.max_joint_step_deg,
                urdf_path=args.urdf,
            )
        except ImportError as exc:
            print(f"缺少运动学求解器依赖：{exc}", file=sys.stderr)
            return EXIT_CONNECT_FAILED
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return code

    # Both --limits-only and live motion require real signs/zeros calibration files.
    try:
        signs = _load_flat_joint_json(args.signs_json, label="signs")
        zeros = _load_flat_joint_json(args.zeros_json, label="zeros")
        adapter = JointKinematicAdapter(signs={args.side: signs}, zeros={args.side: zeros})
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
            max_joint_step_deg=args.max_joint_step_deg,
            follow_error_tolerance_deg=args.follow_error_tol_deg,
            settled_max_joint_delta_deg=args.settled_delta_deg,
        )
    except (RuntimeError, ImportError, MotionNotEnabled) as exc:
        print(f"连接失败：{exc}", file=sys.stderr)
        return EXIT_CONNECT_FAILED

    try:
        if args.limits_only:
            obs = session.client.get_observation(include_cameras=False)
            meta = obs.get("_robot_metadata") or getattr(
                session.client, "latest_robot_metadata", None
            )
            if meta is None:
                print("对端未提供 metadata", file=sys.stderr)
                return EXIT_FEEDBACK_OR_BLOCKED
            limits = adapter.effective_limits_deg(args.side, meta)
            payload = {
                "mode": "limits-only",
                "side": args.side,
                "robot_model": robot_model,
                "effective_limits_deg": {k: [float(v[0]), float(v[1])] for k, v in limits.items()},
            }
            print(json.dumps(payload, indent=2, ensure_ascii=False))
            return EXIT_OK

        arm = session.left if args.side == "left" else session.right
        orientation = np.eye(3, dtype=float) if args.identity_orientation else None
        report = arm.move_by(
            dx=args.dx,
            dy=args.dy,
            dz=args.dz,
            timeout_s=args.timeout_s,
            orientation=orientation,
        )
        payload = {
            "mode": "motion",
            "side": args.side,
            "robot_model": robot_model,
            "orientation_mode": "identity" if args.identity_orientation else "hold",
            "residual_m": float(report.residual_m),
            "steps": len(report.steps),
        }
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return EXIT_OK
    except OutOfRange as exc:
        print(f"路径预检拒绝：{exc}", file=sys.stderr)
        return EXIT_PRECHECK_REJECTED
    except MotionTimeout as exc:
        print(f"运动超时：{exc}（moved={exc.moved:.4f} m）", file=sys.stderr)
        return EXIT_MOTION_TIMEOUT
    except (FeedbackLost, MotionBlocked) as exc:
        print(f"控制中止：{exc}", file=sys.stderr)
        return EXIT_FEEDBACK_OR_BLOCKED
    finally:
        session.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
