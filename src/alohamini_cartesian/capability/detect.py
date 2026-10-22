"""从观测里探测机器人能力，并提供归一化单位到度的换算。

设计原则是 **fail-closed**：读不到可信的 metadata 就不猜，直接抛出
:class:`CapabilityError`；无法确定换算系数就不给数字，直接抛出
:class:`ConversionUnavailable`。
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

SPAN_BY_NORMALIZATION: dict[str, float] = {
    "range_m100_100": 200.0,
    "range_0_100": 100.0,
    "degrees": 1.0,
}
DEFAULT_NORMALIZATION = "range_m100_100"
SUPPORTED_METADATA_SCHEMAS: tuple[int, ...] = (1, 2)
MOTOR_FIELDS = (
    "id",
    "model",
    "normalization",
    "drive_mode",
    "range_min",
    "range_max",
    "homing_offset",
    "resolution",
)
POS_SUFFIX = ".pos"
METADATA_KEY = "_robot_metadata"
CONTROL_STATUS_KEY = "_control_status"
HOST_TIMING_KEY = "_host_timing"


class CapabilityError(RuntimeError):
    """探测能力失败。

    fail-closed 语义：metadata 缺失或不可信时抛出，调用方不得据此继续运动，
    不得用「缺省值」把错误的换算系数带进控制回路。
    """


class ConversionUnavailable(RuntimeError):
    """无法确定某个电机的归一化单位到度的换算系数。

    fail-closed 语义：缺少 range 或 resolution 等必需字段时抛出，调用方必须
    停止下发以度为单位的指令，而不是退回一个猜测的系数。
    """


@dataclass(frozen=True)
class MotorInfo:
    """单个电机的 metadata 摘要，缺失字段一律为 ``None``（不是 0）。"""

    id: int | None
    model: str | None
    normalization: str | None
    drive_mode: int | None
    range_min: int | None
    range_max: int | None
    homing_offset: int | None
    resolution: int | None


@dataclass(frozen=True)
class Capabilities:
    """一次能力探测的结果。"""

    metadata_schema: int | None
    has_control_status: bool
    has_host_timing: bool
    robot_model: str | None
    motors: Mapping[str, MotorInfo]
    notes: tuple[str, ...]


@dataclass(frozen=True)
class Conversion:
    """某个电机的归一化单位到度的换算系数及其来源。

    ``source`` 取值：``"metadata"`` | ``"calibration-file"`` | ``"model-table"``。
    """

    deg_per_unit: float
    range_min: int
    range_max: int
    resolution: int
    source: str


def detect_from_observation(observation: Mapping[str, Any]) -> Capabilities:
    """从一帧观测里提取机器人能力。

    采用 fail-closed 策略：``observation[METADATA_KEY]`` 缺失，或其中的
    ``schema_version`` 缺失、不是整数（含 ``bool``、``1.0``、``"1"`` 这类
    看起来像整数但类型不对的值）时，一律抛出 :class:`CapabilityError`。

    metadata schema 支持 :data:`SUPPORTED_METADATA_SCHEMAS` 中的 1 与 2 两代；
    两代都接受，但缺哪些字段会写进返回值的 ``notes`` 里，由调用方决定是否降级。

    每个电机按 :data:`MOTOR_FIELDS` 抽取；缺失的字段填 ``None``，**不是** 0
    或 4096 —— 填假值会让后续换算静默算出一个看似合理的错误系数。

    Args:
        observation: 运行时返回的一帧观测。

    Returns:
        该帧对应的能力描述。

    Raises:
        CapabilityError: metadata 缺失或版本信息不可信。
    """
    if not isinstance(observation, Mapping):
        raise CapabilityError(f"observation 必须是 Mapping，实际拿到 {type(observation).__name__}")

    metadata = observation.get(METADATA_KEY)
    if not isinstance(metadata, Mapping):
        raise CapabilityError(
            f"observation 缺少可信的 {METADATA_KEY}（拿到 {metadata!r}），按 fail-closed 拒绝探测"
        )
    if not metadata:
        raise CapabilityError(
            f"observation[{METADATA_KEY}] 为空，缺少 schema_version，按 fail-closed 拒绝探测"
        )

    schema = metadata.get("schema_version")
    if isinstance(schema, bool) or not isinstance(schema, int):
        raise CapabilityError(f"{METADATA_KEY}.schema_version 必须是整数，拿到 {schema!r}")
    if schema not in SUPPORTED_METADATA_SCHEMAS:
        raise CapabilityError(
            f"{METADATA_KEY}.schema_version 不受支持: {schema!r}，"
            f"仅支持 {SUPPORTED_METADATA_SCHEMAS}"
        )

    notes: list[str] = []
    robot_model = metadata.get("robot_model")
    if not isinstance(robot_model, str):
        if robot_model is not None:
            notes.append(f"{METADATA_KEY}.robot_model 不是字符串，按 None 处理")
        robot_model = None

    motors: dict[str, MotorInfo] = {}
    raw_motors = metadata.get("motors")
    if not isinstance(raw_motors, Mapping) or not raw_motors:
        notes.append(
            f"{METADATA_KEY}.motors 缺失或为空，按空映射处理；"
            "换算需回落到 calibration-file / model-table"
        )
    else:
        for raw_name, raw_motor in raw_motors.items():
            if not isinstance(raw_motor, Mapping):
                notes.append(f"{METADATA_KEY}.motors[{raw_name!r}] 不是 Mapping，已跳过")
                continue
            motors[str(raw_name)] = _parse_motor(raw_motor)

    for motor_name, motor_info in motors.items():
        missing = [
            field_name
            for field_name in ("homing_offset", "resolution")
            if getattr(motor_info, field_name) is None
        ]
        if missing:
            notes.append(
                f"电机 {motor_name!r} 的 metadata 缺少 {' / '.join(missing)}，"
                "按 None 处理；换算需回落到 calibration-file / model-table"
            )

    return Capabilities(
        metadata_schema=schema,
        has_control_status=isinstance(observation.get(CONTROL_STATUS_KEY), Mapping),
        has_host_timing=isinstance(observation.get(HOST_TIMING_KEY), Mapping),
        robot_model=robot_model,
        motors=motors,
        notes=tuple(notes),
    )


def resolve_deg_per_unit(
    caps: Capabilities,
    motor: str,
    *,
    calibration: Mapping[str, Mapping[str, Any]] | None = None,
    model_resolution: Mapping[str, int] | None = None,
) -> Conversion:
    """解析某个电机的归一化单位到度的换算系数。

    换算依赖 normalization 对应的 span：

    ====================  ===========  ==========================================
    normalization         span         说明
    ====================  ===========  ==========================================
    ``range_m100_100``    200.0        臂关节默认；normalization 缺失时也按此兼容
    ``range_0_100``       100.0        gripper 等 0..100 区间
    ``degrees``           不参与计算   线上值已经是度，``deg_per_unit`` 恒为 1.0
    ====================  ===========  ==========================================

    非 ``degrees`` 时使用公式::

        deg_per_unit = ((range_max - range_min) / span) * 360 / (resolution - 1)

    ``degrees`` 下线上值已经是度，``deg_per_unit`` 恒为 1.0；此时
    ``Conversion`` 里的 ``range_min`` / ``range_max`` / ``resolution`` 只是
    惰性占位值（缺值时回填 ``0, 0, 1``），**不代表任何刻度范围**。

    查找顺序为 metadata → 外部 calibration → 模型分辨率表，命中后记录在
    :attr:`Conversion.source`（``"metadata"`` | ``"calibration-file"`` |
    ``"model-table"``）。``source`` 表示换算最终回落到哪一级；混合来源时报**最弱
    的那一级**（例如 range 由标定文件补齐、resolution 由型号表补齐时报
    ``"calibration-file"``）。

    Args:
        caps: :func:`detect_from_observation` 的探测结果。
        motor: 电机名，可带 ``.pos`` 后缀。
        calibration: 外部标定文件内容，形状为 ``{motor: {field: value}}``。
        model_resolution: 型号到编码器分辨率的映射表。

    Returns:
        换算系数及其来源。

    Raises:
        ConversionUnavailable: 缺少 range 或 resolution 等必需字段。
    """
    name = _motor_name(motor)
    motor_info = caps.motors.get(name)
    if motor_info is None:
        raise ConversionUnavailable(
            f"capabilities 里没有电机 {name!r}，无法确定归一化单位到度的换算"
        )

    normalization = motor_info.normalization
    if normalization is not None and normalization not in SPAN_BY_NORMALIZATION:
        raise ConversionUnavailable(
            f"电机 {name!r} 的 normalization={normalization!r} 未知，不猜 span"
        )

    calibration_entry = _calibration_entry(calibration, name)
    table_resolution = _model_table_resolution(model_resolution, motor_info.model)
    range_min, range_max, resolution, source = _resolve_triple(
        motor_info, calibration_entry, table_resolution
    )

    if normalization == "degrees":
        # degrees 下线上值已经是度，三个字段只是惰性占位（缺值时回填 0/0/1），
        # 不代表任何刻度范围。三源均无刻度信息时才回填占位；来源真实时仍校验
        # 区间非倒置，不把倒置区间当成有效范围静默返回。
        no_scale_source = (
            motor_info.range_min is None
            and motor_info.range_max is None
            and motor_info.resolution is None
            and _coerce_int(calibration_entry.get("range_min")) is None
            and _coerce_int(calibration_entry.get("range_max")) is None
            and table_resolution is None
        )
        if no_scale_source:
            return Conversion(
                deg_per_unit=1.0,
                range_min=0,
                range_max=0,
                resolution=1,
                source=source,
            )
        if range_min is not None and range_max is not None and range_max <= range_min:
            raise ConversionUnavailable(
                f"电机 {name!r} 的 range 无效: range_max ({range_max}) <= range_min ({range_min})"
            )
        return Conversion(
            deg_per_unit=1.0,
            range_min=0 if range_min is None else range_min,
            range_max=0 if range_max is None else range_max,
            resolution=1 if resolution is None else resolution,
            source=source,
        )

    missing_range = [
        field_name
        for field_name, value in (("range_min", range_min), ("range_max", range_max))
        if value is None
    ]
    if missing_range:
        raise ConversionUnavailable(
            f"电机 {name!r} 缺少 range: missing {', '.join(missing_range)}，不猜换算系数"
        )
    if resolution is None:
        raise ConversionUnavailable(
            f"电机 {name!r} 缺少 resolution（metadata 与模型分辨率表都没有），不猜换算系数"
        )
    if range_max <= range_min:
        raise ConversionUnavailable(
            f"电机 {name!r} 的 range 无效: range_max ({range_max}) <= range_min ({range_min})"
        )
    if resolution <= 1:
        raise ConversionUnavailable(f"电机 {name!r} 的 resolution 无效: {resolution} <= 1")

    span = SPAN_BY_NORMALIZATION[DEFAULT_NORMALIZATION if normalization is None else normalization]
    deg_per_unit = ((range_max - range_min) / span) * 360 / (resolution - 1)
    if not math.isfinite(deg_per_unit) or deg_per_unit <= 0:
        raise ConversionUnavailable(
            f"电机 {name!r} 算出的 deg_per_unit 非正或非有限: {deg_per_unit!r}"
        )

    return Conversion(
        deg_per_unit=deg_per_unit,
        range_min=range_min,
        range_max=range_max,
        resolution=resolution,
        source=source,
    )


def _motor_name(joint_key: str) -> str:
    """把关节键还原成电机名：去掉结尾的 ``.pos`` 并 strip。"""
    name = joint_key.strip()
    if name.endswith(POS_SUFFIX):
        name = name[: -len(POS_SUFFIX)]
    return name


def _coerce_int(value: Any) -> int | None:
    """把 metadata / 标定值收敛成 ``int``；``bool``、非整数 float 与其它类型一律 ``None``。"""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return None


def _parse_motor(raw: Mapping[str, Any]) -> MotorInfo:
    """按 :data:`MOTOR_FIELDS` 的规则解析单个电机；缺失字段一律 ``None``。"""
    model = raw.get("model")
    normalization = raw.get("normalization")
    return MotorInfo(
        id=_coerce_int(raw.get("id")),
        model=model if isinstance(model, str) else None,
        normalization=normalization if isinstance(normalization, str) else None,
        drive_mode=_coerce_int(raw.get("drive_mode")),
        range_min=_coerce_int(raw.get("range_min")),
        range_max=_coerce_int(raw.get("range_max")),
        homing_offset=_coerce_int(raw.get("homing_offset")),
        resolution=_coerce_int(raw.get("resolution")),
    )


def _calibration_entry(
    calibration: Mapping[str, Mapping[str, Any]] | None, motor: str
) -> Mapping[str, Any]:
    """取出标定文件里该电机的条目；缺失或形状不对时返回空映射。"""
    if calibration is None:
        return {}
    entry = calibration.get(motor)
    return entry if isinstance(entry, Mapping) else {}


def _model_table_resolution(
    model_resolution: Mapping[str, int] | None, model: str | None
) -> int | None:
    """按型号查编码器分辨率；型号或表缺失时返回 ``None``。"""
    if model_resolution is None or model is None:
        return None
    return _coerce_int(model_resolution.get(model))


def _resolve_triple(
    motor_info: MotorInfo,
    calibration: Mapping[str, Any],
    table_resolution: int | None,
) -> tuple[int | None, int | None, int | None, str]:
    """逐字段回落取值（metadata → calibration → 模型表）并判定 ``source``。"""
    cal_range_min = _coerce_int(calibration.get("range_min"))
    cal_range_max = _coerce_int(calibration.get("range_max"))
    range_min = motor_info.range_min if motor_info.range_min is not None else cal_range_min
    range_max = motor_info.range_max if motor_info.range_max is not None else cal_range_max
    resolution = motor_info.resolution if motor_info.resolution is not None else table_resolution

    range_from_calibration = motor_info.range_min is None or motor_info.range_max is None
    if range_from_calibration and (range_min is not None or range_max is not None):
        source = "calibration-file"
    elif motor_info.resolution is None and table_resolution is not None:
        source = "model-table"
    else:
        source = "metadata"
    return range_min, range_max, resolution, source
