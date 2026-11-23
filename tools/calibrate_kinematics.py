#!/usr/bin/env python3
"""机械臂运动学 ``signs`` / ``zeros`` 标定与自检工具（对应现场手册 §10.8）。

从一帧观测（离线 JSON 快照或只读连机读取）结合已知参考位形（默认 C2b：
``(0, -72, +73, 0, 0, 0)°``）与实测方向 ``signs``，按 ``JointKinematicAdapter``
精确逆关系算出六关节 ``zeros`` 并落盘扁平六键 JSON：

    q_kinematic = sign * motor_deg + zero  =>  zero = q_ref - sign * motor_deg

退出码：
    0: 成功（含自检通过）
    2: 参数或输入格式错误
    3: 依赖/连接失败
    15: 缺少必需的 signs 或观测字段
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from alohamini_cartesian.config.overlay import ConfigError, load_settings
from alohamini_cartesian.kinematics.joint_adapter import (
    JOINT_ORDER,
    URDF_JOINT_NAMES,
    JointKinematicAdapter,
)
from alohamini_cartesian.kinematics.model_specs import validate_robot_model

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_CONNECT_FAILED = 3
EXIT_MISSING_CALIB = 15

# 参考位形 C2b（URDF 运动学角，单位度）：大臂竖直向上、小臂水平朝正前。
DEFAULT_REF_Q_DEG: tuple[float, ...] = (0.0, -72.0, 73.0, 0.0, 0.0, 0.0)
SELF_CHECK_TOL_DEG = 3.0
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


def load_flat_six_joint_json(path: Path, *, label: str) -> dict[str, float]:
    """读取扁平六键 JSON（严格校验键集合与有限数值，拒绝嵌套包装）。"""
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取 {label} 文件 {path}: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise ValueError(f"{label} 必须是 JSON 对象（扁平六键），实际拿到 {type(raw).__name__}")
    result: dict[str, float] = {}
    for joint in JOINT_ORDER:
        if joint not in raw:
            raise ValueError(f"{label} 缺少关节键 {joint!r}（必须是扁平六键）")
        val = raw[joint]
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
            raise ValueError(f"{label}[{joint!r}] 必须是有限数值，拿到 {val!r}")
        result[joint] = float(val)
    return result


def validate_signs_map(signs: Mapping[str, float]) -> dict[str, float]:
    """校验六关节符号表：每个值必须严格为 +1 或 -1。"""
    validated: dict[str, float] = {}
    for joint in JOINT_ORDER:
        if joint not in signs:
            raise ValueError(f"signs 缺少关节 {joint!r}")
        val = float(signs[joint])
        if val not in (-1.0, 1.0):
            raise ValueError(f"signs[{joint!r}] 必须为 +1 或 -1，拿到 {val!r}")
        validated[joint] = val
    return validated


def compute_zeros_from_observation(
    side: str,
    observation: Mapping[str, Any],
    metadata: Mapping[str, Any],
    signs: Mapping[str, float],
    ref_q_deg: Sequence[float] = DEFAULT_REF_Q_DEG,
) -> dict[str, float]:
    """按参考位形与观测值反推六关节 ``zeros``。

    利用 ``JointKinematicAdapter``（临时以 ``zeros=0`` 构造）精确复用
    ``range_m100_100`` / ``degrees`` 到电机度的换算，再按
    ``zero = q_ref - sign * motor_deg`` 求解。
    """
    if side not in ("left", "right"):
        raise ValueError(f"side 必须是 'left' 或 'right'，拿到 {side!r}")
    if len(ref_q_deg) != len(JOINT_ORDER):
        raise ValueError(f"ref_q_deg 必须有 6 个角，拿到 {len(ref_q_deg)}")
    validated_signs = validate_signs_map(signs)
    zero_probe = JointKinematicAdapter(
        signs={side: validated_signs},
        zeros={side: dict.fromkeys(JOINT_ORDER, 0.0)},
    )
    # With zeros=0, observation_to_kinematic_deg returns sign * motor_deg.
    signed_motor_deg = zero_probe.observation_to_kinematic_deg(side, observation, metadata)
    zeros: dict[str, float] = {}
    for idx, joint in enumerate(JOINT_ORDER):
        zeros[joint] = float(ref_q_deg[idx]) - float(signed_motor_deg[idx])
    return zeros


def self_check_calibration(
    side: str,
    observation: Mapping[str, Any],
    metadata: Mapping[str, Any],
    signs: Mapping[str, float],
    zeros: Mapping[str, float],
    ref_q_deg: Sequence[float] = DEFAULT_REF_Q_DEG,
    *,
    tol_deg: float = SELF_CHECK_TOL_DEG,
) -> dict[str, Any]:
    """构造真实 ``JointKinematicAdapter`` 并复算参考位形误差与有效限位窗口。"""
    adapter = JointKinematicAdapter(
        signs={side: dict(signs)},
        zeros={side: dict(zeros)},
    )
    q_kin = adapter.observation_to_kinematic_deg(side, observation, metadata)
    q_ref = np.asarray(ref_q_deg, dtype=float)
    max_err_deg = float(np.max(np.abs(q_kin - q_ref)))
    limits = adapter.effective_limits_deg(side, metadata)
    report: dict[str, Any] = {
        "side": side,
        "q_kinematic_deg": [float(v) for v in q_kin],
        "q_ref_deg": [float(v) for v in q_ref],
        "max_error_deg": max_err_deg,
        "within_tolerance": max_err_deg <= tol_deg,
        "tolerance_deg": tol_deg,
        "effective_limits_deg": {k: [float(v[0]), float(v[1])] for k, v in limits.items()},
    }
    try:
        from alohamini_cartesian.control.arm_ik import ArmIK, ArmKinematicProfile

        profile = ArmKinematicProfile(
            urdf_path=str(DEFAULT_URDF_PATH),
            target_frame=DEFAULT_TARGET_FRAME,
            joint_names=tuple(URDF_JOINT_NAMES[j] for j in JOINT_ORDER),
        )
        fk_pose = ArmIK(profile).fk(q_kin)
        report["fk_tip_xyz_m"] = [float(v) for v in fk_pose[:3, 3]]
    except ImportError:
        report["fk_tip_xyz_m"] = None
    return report


def _default_live_observation_reader(
    host: str,
    robot_model: str,
    observation_request_window: int,
) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    """惰性导入厂商 ``AlohaMiniClient`` 只读抓取一帧观测与 metadata。"""
    try:
        from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient
        from lerobot.robots.alohamini.config_alohamini import AlohaMiniClientConfig
    except ImportError as exc:
        raise RuntimeError(
            f"缺少厂商运行时 lerobot.robots.alohamini（{exc}）；"
            "请在装有 vendor fork 的环境运行，或改用 --from-observation-json 离线模式"
        ) from exc

    cfg = AlohaMiniClientConfig(
        remote_ip=host,
        robot_model=robot_model,
        observation_request_window=observation_request_window,
    )
    client = AlohaMiniClient(cfg)
    client.connect()
    try:
        client.prime_observation_request_window(include_cameras=False)
        obs = client.get_observation(include_cameras=False)
        metadata = obs.get("_robot_metadata") or getattr(client, "latest_robot_metadata", None)
        if not isinstance(metadata, Mapping):
            raise RuntimeError("对端未提供 _robot_metadata / latest_robot_metadata")
        return obs, metadata
    finally:
        client.disconnect()


def _parse_ref_q(raw: str) -> tuple[float, ...]:
    """解析逗号分隔的 6 个参考位形角度。"""
    parts = [item.strip() for item in raw.split(",") if item.strip()]
    if len(parts) != len(JOINT_ORDER):
        raise ValueError(f"--ref-q-deg 需要 6 个角度，收到 {len(parts)} 个")
    values = tuple(float(p) for p in parts)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("--ref-q-deg 包含非有限数值")
    return values


def _build_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="计算并自检 AlohaMini 单臂 signs/zeros 运动学标定映射（扁平六键 JSON）。"
    )
    parser.add_argument("--side", choices=("left", "right"), required=True, help="标定侧别")
    parser.add_argument("--signs-json", type=Path, required=True, help="扁平六键 signs JSON 路径")
    parser.add_argument(
        "--from-observation-json",
        type=Path,
        default=None,
        help="离线观测快照 JSON（含 observation 与 metadata，或顶层平铺关节键 + _robot_metadata）",
    )
    parser.add_argument("--host", default="127.0.0.1", help="在线只读采帧时的 Pi Host 地址")
    parser.add_argument("--template", type=Path, default=None, help="配置模板 YAML（默认按仓库）")
    parser.add_argument("--site", type=Path, default=None, help="现场覆盖 YAML（默认按仓库）")
    parser.add_argument("--robot-model", default=None, help="覆盖配置中的 robot_model")
    parser.add_argument(
        "--ref-q-deg",
        default=",".join(str(v) for v in DEFAULT_REF_Q_DEG),
        help="参考位形六个 URDF 关节角（度），默认 C2b: 0,-72,73,0,0,0",
    )
    parser.add_argument("--zeros-out", type=Path, default=None, help="输出扁平六键 zeros JSON 路径")
    parser.add_argument("--json", action="store_true", help="以 JSON 格式打印结果")
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    live_reader: Callable[[str, str, int], tuple[Mapping[str, Any], Mapping[str, Any]]]
    | None = None,
) -> int:
    """命令行主入口。"""
    args = _build_parser().parse_args(argv)
    try:
        ref_q = _parse_ref_q(args.ref_q_deg)
    except ValueError as exc:
        print(f"参数错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    try:
        settings = load_settings(template=args.template, site=args.site)
        robot_model = args.robot_model or settings.robot_model
        validate_robot_model(robot_model)
    except (ConfigError, ValueError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    try:
        signs = validate_signs_map(load_flat_six_joint_json(args.signs_json, label="signs"))
    except ValueError as exc:
        print(f"标定输入错误：{exc}", file=sys.stderr)
        return EXIT_MISSING_CALIB

    if args.from_observation_json is not None:
        try:
            doc = json.loads(args.from_observation_json.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"观测文件不可读：{exc}", file=sys.stderr)
            return EXIT_USAGE
        if not isinstance(doc, Mapping):
            print("观测文件顶层必须是 JSON 对象", file=sys.stderr)
            return EXIT_USAGE
        observation = doc.get("observation", doc)
        metadata = doc.get("metadata") or doc.get("_robot_metadata") or doc.get("robot_metadata")
        if not isinstance(observation, Mapping) or not isinstance(metadata, Mapping):
            print("观测文件缺少有效的 observation 或 metadata", file=sys.stderr)
            return EXIT_MISSING_CALIB
    else:
        reader = live_reader or _default_live_observation_reader
        try:
            observation, metadata = reader(
                args.host, robot_model, settings.observation_request_window
            )
        except Exception as exc:  # noqa: BLE001
            print(f"只读采帧失败：{exc}", file=sys.stderr)
            return EXIT_CONNECT_FAILED

    try:
        zeros = compute_zeros_from_observation(args.side, observation, metadata, signs, ref_q)
        check = self_check_calibration(args.side, observation, metadata, signs, zeros, ref_q)
    except ValueError as exc:
        print(f"标定计算失败：{exc}", file=sys.stderr)
        return EXIT_MISSING_CALIB

    if args.zeros_out is not None:
        args.zeros_out.parent.mkdir(parents=True, exist_ok=True)
        args.zeros_out.write_text(
            json.dumps(zeros, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    payload = {
        "side": args.side,
        "robot_model": robot_model,
        "signs": signs,
        "zeros": zeros,
        "self_check": check,
    }
    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        print(f"side={args.side} robot_model={robot_model}")
        print(f"zeros={json.dumps(zeros, ensure_ascii=False)}")
        print(
            f"self_check: max_error_deg={check['max_error_deg']:.4f} "
            f"within_tolerance={check['within_tolerance']}"
        )
        if args.zeros_out is not None:
            print(f"zeros written: {args.zeros_out}")
    return EXIT_OK if check["within_tolerance"] else EXIT_MISSING_CALIB


if __name__ == "__main__":
    raise SystemExit(main())
