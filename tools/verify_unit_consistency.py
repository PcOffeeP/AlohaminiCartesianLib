#!/usr/bin/env python3
"""核对真机 metadata 指纹与标定文件的单位口径是否一致（只读、离线）。

用法::

    verify_unit_consistency.py FINGERPRINT_JSON CALIBRATION_JSON

只读这两个 JSON，不硬编码任何路径，不下发命令、不连设备、不联网。逐电机输出一行，
末尾一行汇总；退出码 0 表示全一致、1 表示有差异、2 表示输入不可解析或不可用于核对。

输入结构容错（两种取电机映射的位置都接受）:

* 指纹：优先 ``robot_metadata.motors``，其次顶层 ``motors``；每台电机取
  ``normalization`` / ``range_min`` / ``range_max`` / ``resolution``，另取
  ``id`` / ``drive_mode`` / ``homing_offset`` 用于逐项核对。
* 标定：顶层直接是电机映射（``{电机名: {字段: 值}}``），每台电机只有
  ``id`` / ``drive_mode`` / ``homing_offset`` / ``range_min`` / ``range_max``，
  **没有** ``resolution``、**没有** ``normalization``。

字段语义与出处:

* ``resolution`` 是编码器每圈刻度数，``range_min`` / ``range_max`` 是同一刻度系的
  tick 值（与 ``homing_offset`` 同系）；本库只从 metadata 指纹拿到 ``resolution``。
* ``deg_per_unit`` 不由本脚本重写公式，而是直接调库的
  :func:`alohamini_cartesian.capability.detect.resolve_deg_per_unit`，口径即
  ``deg_per_unit = ((range_max - range_min) / span) * 360 / (resolution - 1)``
  （``capability/detect.py:298``）；``span`` 取自 ``SPAN_BY_NORMALIZATION``
  （``capability/detect.py:15``）：``range_m100_100`` = 200.0、``range_0_100`` = 100.0、
  ``degrees`` = 1.0。
* ``normalization == "degrees"`` 时线上值已经是度，``deg_per_unit`` 恒为 1.0，
  ``range_min`` / ``range_max`` / ``resolution`` 只是惰性占位（``capability/detect.py:204-206``）。
  因此该电机的 ``fp_deg_span`` / ``cal_deg_span`` / ``delta_deg`` 三列打印 ``n/a``；
  range 仍照常按整数比较，range 不一致照样判 ``MISMATCH``。
* 判等的字段：``range_min``、``range_max``（整数严格相等），另加
  ``id`` / ``drive_mode`` / ``homing_offset``（两侧都有值时才比），以及
  ``presence``（只在指纹或只在标定里出现的电机）。``delta_deg`` 另用
  ``abs(delta) <= 1e-9`` 兜一道派生量的判等。

退出码 2 的输入（stderr 会点名电机与字段，不静默跳过）：文件读不了、JSON 坏、顶层
不是对象、缺 ``robot_metadata.motors`` 与顶层 ``motors``、某台指纹电机缺
``normalization`` / ``range_min`` / ``range_max`` / ``resolution``、这些字段不是整数、
``resolution <= 1``、``normalization`` 不在已知三种里，以及库的换算函数拒绝该电机
（例如 ``range_max <= range_min``）。
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from alohamini_cartesian.capability.detect import (
    SPAN_BY_NORMALIZATION,
    Capabilities,
    ConversionUnavailable,
    MotorInfo,
    resolve_deg_per_unit,
)

EXIT_OK = 0
EXIT_MISMATCH = 1
EXIT_UNUSABLE = 2

# ``delta_deg`` 是派生量：range 已按整数严格比较过，这里再给浮点结果一道容差。
DELTA_TOLERANCE_DEG = 1e-9

# 两侧都有值时才逐项比较的身份字段（标定文件与指纹同系，见模块 docstring）。
IDENTITY_FIELDS = ("id", "drive_mode", "homing_offset")

HEADER = (
    f"{'motor':<22} {'normalization':<15} {'resolution':>10} {'fp_range':>12} "
    f"{'cal_range':>12} {'deg_per_unit':>12} {'fp_deg_span':>11} {'cal_deg_span':>12} "
    f"{'delta_deg':>9} flag"
)


class InputError(ValueError):
    """输入不可解析或不可用于核对（对应退出码 2）。"""


@dataclass(frozen=True)
class Mismatch:
    """一条差异：哪台电机、哪个字段、两侧各是什么。"""

    motor: str
    field: str
    fingerprint: str
    calibration: str


@dataclass(frozen=True)
class MotorRow:
    """单台电机的一行核对结果；不可得的数值一律为 ``None``，渲染成 ``n/a``。"""

    motor: str
    normalization: str | None
    resolution: int | None
    fp_range_min: int | None
    fp_range_max: int | None
    cal_range_min: int | None
    cal_range_max: int | None
    deg_per_unit: float | None
    fp_deg_span: float | None
    cal_deg_span: float | None
    delta_deg: float | None
    mismatched: bool


@dataclass(frozen=True)
class CompareResult:
    """一次核对的全部结果：逐电机行与差异清单。"""

    rows: tuple[MotorRow, ...]
    mismatches: tuple[Mismatch, ...]

    @property
    def mismatched_motors(self) -> int:
        """至少有一个字段不一致的电机台数。"""
        return len({mismatch.motor for mismatch in self.mismatches})

    @property
    def exit_code(self) -> int:
        """全一致返回 0，有差异返回 1。"""
        return EXIT_OK if not self.mismatches else EXIT_MISMATCH


def read_json(path_text: str, label: str) -> Mapping[str, Any]:
    """读一个 JSON 文件并要求顶层是对象，失败时抛 :class:`InputError`。

    Args:
        path_text: 文件路径（来自命令行，不硬编码）。
        label: 出错信息里用的输入名，``"fingerprint"`` 或 ``"calibration"``。

    Returns:
        解析后的顶层映射。

    Raises:
        InputError: 文件读不了、JSON 解析失败，或顶层不是 JSON 对象。
    """
    path = Path(path_text)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise InputError(f"{label}: 读不了 {path}（{exc}）") from exc
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise InputError(f"{label}: {path} 不是合法 JSON（{exc}）") from exc
    if not isinstance(payload, Mapping):
        raise InputError(f"{label}: {path} 的顶层必须是 JSON 对象，实际是 {type(payload).__name__}")
    return payload


def compare(fingerprint: Mapping[str, Any], calibration: Mapping[str, Any]) -> CompareResult:
    """逐电机核对两侧字段，返回每行的数值与差异清单。

    Args:
        fingerprint: metadata 指纹的顶层内容。
        calibration: 标定文件的顶层内容。

    Returns:
        逐电机行与差异清单；``exit_code`` 属性即应返回的进程退出码。

    Raises:
        InputError: 输入结构不完整或不可用于核对。
    """
    raw_motors = _fingerprint_motors(fingerprint)
    raw_calibration = _calibration_motors(calibration)
    motors = _validate_fingerprint_motors(raw_motors)
    calibration_entries = _validate_calibration_motors(raw_calibration)
    caps = _capabilities(fingerprint, motors)

    rows: list[MotorRow] = []
    mismatches: list[Mismatch] = []
    for name in sorted(set(motors) | set(calibration_entries)):
        motor_info = motors.get(name)
        cal_entry = calibration_entries.get(name)
        motor_mismatches = _compare_fields(name, motor_info, cal_entry)
        mismatches.extend(motor_mismatches)
        rows.append(_build_row(name, motor_info, cal_entry, caps, bool(motor_mismatches)))
    return CompareResult(rows=tuple(rows), mismatches=tuple(mismatches))


def render(result: CompareResult) -> str:
    """把核对结果渲染成「表头 + 逐电机行 + 差异行 + 汇总行」的文本。"""
    lines = [HEADER]
    lines.extend(_format_row(row) for row in result.rows)
    for mismatch in result.mismatches:
        lines.append(
            f"MISMATCH: motor={mismatch.motor} field={mismatch.field} "
            f"fingerprint={mismatch.fingerprint} calibration={mismatch.calibration}"
        )
    lines.append(
        f"SUMMARY: motors={len(result.rows)} "
        f"mismatched={result.mismatched_motors} exit={result.exit_code}"
    )
    return "\n".join(lines) + "\n"


def main(argv: Sequence[str]) -> int:
    """解析参数、读两个 JSON、核对并打印结果；返回进程退出码。"""
    parser = argparse.ArgumentParser(
        description="核对 metadata 指纹与标定文件的单位口径（只读，不下发命令）"
    )
    parser.add_argument("fingerprint_json", help="metadata 指纹 JSON 路径")
    parser.add_argument("calibration_json", help="标定文件 JSON 路径")
    args = parser.parse_args(list(argv))
    try:
        fingerprint = read_json(args.fingerprint_json, "fingerprint")
        calibration = read_json(args.calibration_json, "calibration")
        result = compare(fingerprint, calibration)
    except InputError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_UNUSABLE
    sys.stdout.write(render(result))
    return result.exit_code


def _fingerprint_motors(fingerprint: Mapping[str, Any]) -> Mapping[str, Any]:
    """取指纹里的电机映射：优先 ``robot_metadata.motors``，其次顶层 ``motors``。"""
    candidates: list[tuple[str, Any]] = []
    metadata = fingerprint.get("robot_metadata")
    if isinstance(metadata, Mapping) and "motors" in metadata:
        candidates.append(("robot_metadata.motors", metadata["motors"]))
    if "motors" in fingerprint:
        candidates.append(("motors", fingerprint["motors"]))
    for label, motors in candidates:
        if isinstance(motors, Mapping) and motors:
            return motors
        if motors is not None and not isinstance(motors, Mapping):
            raise InputError(f"fingerprint: {label} 不是映射（{type(motors).__name__}）")
    if not candidates:
        raise InputError("fingerprint: 既没有 robot_metadata.motors，也没有顶层 motors")
    raise InputError("fingerprint: 电机映射为空，没有可核对的电机")


def _calibration_motors(calibration: Mapping[str, Any]) -> Mapping[str, Any]:
    """标定文件顶层直接就是电机映射；空文件不可用于核对。"""
    if not calibration:
        raise InputError("calibration: 顶层为空，没有可核对的电机")
    return calibration


def _validate_fingerprint_motors(raw_motors: Mapping[str, Any]) -> dict[str, MotorInfo]:
    """逐台校验指纹电机的必需字段，返回可直接交给库的 ``MotorInfo`` 映射。"""
    motors: dict[str, MotorInfo] = {}
    for raw_name, entry in raw_motors.items():
        name = str(raw_name)
        if not isinstance(entry, Mapping):
            raise InputError(f"fingerprint: 电机 {name!r} 的条目不是映射")
        normalization = _require_normalization(entry, name)
        range_min = _require_int(entry, "range_min", "fingerprint", name)
        range_max = _require_int(entry, "range_max", "fingerprint", name)
        resolution = _require_int(entry, "resolution", "fingerprint", name)
        if resolution <= 1:
            raise InputError(
                f"fingerprint: 电机 {name!r} 的 resolution={resolution} <= 1，无法换算"
            )
        model = entry.get("model")
        motors[name] = MotorInfo(
            id=_optional_int(entry, "id"),
            model=model if isinstance(model, str) else None,
            normalization=normalization,
            drive_mode=_optional_int(entry, "drive_mode"),
            range_min=range_min,
            range_max=range_max,
            homing_offset=_optional_int(entry, "homing_offset"),
            resolution=resolution,
        )
    return motors


def _validate_calibration_motors(raw_motors: Mapping[str, Any]) -> dict[str, dict[str, int | None]]:
    """逐台校验标定电机的 range 字段，返回只含所需列的普通字典。"""
    entries: dict[str, dict[str, int | None]] = {}
    for raw_name, entry in raw_motors.items():
        name = str(raw_name)
        if not isinstance(entry, Mapping):
            raise InputError(f"calibration: 电机 {name!r} 的条目不是映射")
        entries[name] = {
            "range_min": _require_int(entry, "range_min", "calibration", name),
            "range_max": _require_int(entry, "range_max", "calibration", name),
            "id": _optional_int(entry, "id"),
            "drive_mode": _optional_int(entry, "drive_mode"),
            "homing_offset": _optional_int(entry, "homing_offset"),
        }
    return entries


def _require_normalization(entry: Mapping[str, Any], motor: str) -> str:
    """取 ``normalization`` 并确认它是已知三种之一。"""
    if "normalization" not in entry:
        raise InputError(f"fingerprint: 电机 {motor!r} 缺少字段 normalization")
    value = entry["normalization"]
    if not isinstance(value, str):
        raise InputError(f"fingerprint: 电机 {motor!r} 的 normalization 不是字符串（{value!r}）")
    if value not in SPAN_BY_NORMALIZATION:
        known = " / ".join(sorted(SPAN_BY_NORMALIZATION))
        raise InputError(
            f"fingerprint: 电机 {motor!r} 的 normalization={value!r} 不在已知三种里（{known}）"
        )
    return value


def _require_int(entry: Mapping[str, Any], field: str, side: str, motor: str) -> int:
    """取一个必须是整数的字段；缺失或类型不对时报 :class:`InputError`。"""
    if field not in entry:
        raise InputError(f"{side}: 电机 {motor!r} 缺少字段 {field}")
    value = entry[field]
    if isinstance(value, bool) or not isinstance(value, int):
        raise InputError(f"{side}: 电机 {motor!r} 的 {field} 不是整数（{value!r}）")
    return value


def _optional_int(entry: Mapping[str, Any], field: str) -> int | None:
    """取可选整数字段；缺失或类型不对都按 ``None``（不参与该项比较）。"""
    value = entry.get(field)
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _capabilities(fingerprint: Mapping[str, Any], motors: dict[str, MotorInfo]) -> Capabilities:
    """把指纹装成库的 :class:`Capabilities`，唯一目的是复用库的换算函数。"""
    metadata = fingerprint.get("robot_metadata")
    schema = metadata.get("schema_version") if isinstance(metadata, Mapping) else None
    return Capabilities(
        metadata_schema=schema
        if isinstance(schema, int) and not isinstance(schema, bool)
        else None,
        has_control_status=False,
        has_host_timing=False,
        robot_model=None,
        motors=motors,
        notes=(),
    )


def _compare_fields(
    name: str, motor_info: MotorInfo | None, cal_entry: Mapping[str, int | None] | None
) -> list[Mismatch]:
    """比对一台电机两侧的字段；只在一侧出现的电机只报 ``presence``。"""
    if motor_info is None or cal_entry is None:
        return [
            Mismatch(
                motor=name,
                field="presence",
                fingerprint="missing" if motor_info is None else "present",
                calibration="missing" if cal_entry is None else "present",
            )
        ]
    found: list[Mismatch] = []
    for field in ("range_min", "range_max"):
        fingerprint_value = getattr(motor_info, field)
        calibration_value = cal_entry[field]
        if fingerprint_value != calibration_value:
            found.append(Mismatch(name, field, str(fingerprint_value), str(calibration_value)))
    for field in IDENTITY_FIELDS:
        fingerprint_value = getattr(motor_info, field)
        calibration_value = cal_entry[field]
        if fingerprint_value is None or calibration_value is None:
            continue
        if fingerprint_value != calibration_value:
            found.append(Mismatch(name, field, str(fingerprint_value), str(calibration_value)))
    return found


def _build_row(
    name: str,
    motor_info: MotorInfo | None,
    cal_entry: Mapping[str, int | None] | None,
    caps: Capabilities,
    mismatched: bool,
) -> MotorRow:
    """算一台电机那一行的全部数值；不可得的列留 ``None``。"""
    if motor_info is None:
        # 只在标定一侧出现：没有 resolution，也就没有 deg_per_unit 与角度行程。
        return MotorRow(
            motor=name,
            normalization=None,
            resolution=None,
            fp_range_min=None,
            fp_range_max=None,
            cal_range_min=cal_entry["range_min"] if cal_entry is not None else None,
            cal_range_max=cal_entry["range_max"] if cal_entry is not None else None,
            deg_per_unit=None,
            fp_deg_span=None,
            cal_deg_span=None,
            delta_deg=None,
            mismatched=True,
        )

    range_min = motor_info.range_min
    range_max = motor_info.range_max
    resolution = motor_info.resolution
    if range_min is None or range_max is None or resolution is None:
        # 校验阶段已经拦下，这里只是让类型收敛。
        raise InputError(f"fingerprint: 电机 {name!r} 的 range / resolution 不完整")
    degrees_mode = motor_info.normalization == "degrees"
    fp_deg_span = None if degrees_mode else _deg_span(range_min, range_max, resolution)
    cal_deg_span = (
        None
        if degrees_mode or cal_entry is None
        else _deg_span(
            _cal_int(cal_entry, "range_min"), _cal_int(cal_entry, "range_max"), resolution
        )
    )
    delta_deg = None if fp_deg_span is None or cal_deg_span is None else cal_deg_span - fp_deg_span
    if delta_deg is not None and abs(delta_deg) > DELTA_TOLERANCE_DEG:
        mismatched = True
    return MotorRow(
        motor=name,
        normalization=motor_info.normalization,
        resolution=resolution,
        fp_range_min=range_min,
        fp_range_max=range_max,
        cal_range_min=None if cal_entry is None else _cal_int(cal_entry, "range_min"),
        cal_range_max=None if cal_entry is None else _cal_int(cal_entry, "range_max"),
        deg_per_unit=_deg_per_unit(caps, name),
        fp_deg_span=fp_deg_span,
        cal_deg_span=cal_deg_span,
        delta_deg=delta_deg,
        mismatched=mismatched,
    )


def _cal_int(entry: Mapping[str, int | None], field: str) -> int:
    """取标定条目里已校验过的整数字段。"""
    value = entry[field]
    if value is None:
        raise InputError(f"calibration: 缺少字段 {field}")
    return value


def _deg_span(range_min: int, range_max: int, resolution: int) -> float:
    """满 range 对应的机械角行程（度）：``(max - min) * 360 / (resolution - 1)``。"""
    return (range_max - range_min) * 360 / (resolution - 1)


def _deg_per_unit(caps: Capabilities, motor: str) -> float:
    """调库的换算函数取 ``deg_per_unit``；库拒绝该电机时按输入不可用处理。"""
    try:
        return resolve_deg_per_unit(caps, motor).deg_per_unit
    except ConversionUnavailable as exc:
        raise InputError(f"fingerprint: 电机 {motor!r} 的单位换算不可用（{exc}）") from exc


def _format_row(row: MotorRow) -> str:
    """按固定列顺序渲染一行；不可得的数值打印 ``n/a``。"""
    flag = "MISMATCH" if row.mismatched else "OK"
    return (
        f"{row.motor:<22} {_text(row.normalization):<15} {_number(row.resolution):>10} "
        f"{_range_text(row.fp_range_min, row.fp_range_max):>12} "
        f"{_range_text(row.cal_range_min, row.cal_range_max):>12} "
        f"{_decimal(row.deg_per_unit, 6):>12} {_decimal(row.fp_deg_span, 2):>11} "
        f"{_decimal(row.cal_deg_span, 2):>12} {_decimal(row.delta_deg, 3):>9} "
        f"flag={flag}"
    )


def _text(value: str | None) -> str:
    """文本列：``None`` 打印 ``n/a``。"""
    return "n/a" if value is None else value


def _number(value: int | None) -> str:
    """整数列：``None`` 打印 ``n/a``。"""
    return "n/a" if value is None else str(value)


def _range_text(low: int | None, high: int | None) -> str:
    """``min..max`` 形式的 range；任一侧缺失打印 ``n/a``。"""
    if low is None or high is None:
        return "n/a"
    return f"{low}..{high}"


def _decimal(value: float | None, digits: int) -> str:
    """浮点列：``None`` 打印 ``n/a``，否则按固定小数位。"""
    return "n/a" if value is None else f"{value:.{digits}f}"


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
