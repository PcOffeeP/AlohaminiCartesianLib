"""标定代际的身份（``calib_id``）、结构校验与变化分类。

一个「代际」= 某一次标定产出的 18 台电机参数快照。身份只由五件事决定：
``id`` / ``drive_mode`` / ``homing_offset`` / ``range_min`` / ``range_max``。电机名排序、
字段名排序、紧凑 JSON 之后取 sha256 前 12 位即 :func:`calib_id`，因此：

* 电机顺序、字段顺序、JSON 缩进都不影响身份；
* v2 指纹里多出的 ``model`` / ``normalization`` / ``resolution`` 不参与身份，
  所以同一份数据无论以「标定文件形状」还是「指纹形状」输入，算出的 id 相同；
* 任一标定字段动一位，id 就变（变化分类见 :func:`classify_change`）。

校验分两级，互不替代：

* :func:`validate_structure` 判「这份标定是否自洽」——电机数、字段齐备、取值类型、
  ``range_min < range_max``、``drive_mode ∈ {0, 1}``。返回问题清单（空清单 = 通过），
  **不抛异常**。真损坏（缺字段、``None``、区间反向）由调用方决定是否算阻断。
* :func:`validate_plausibility` 判「这份标定像不像真的」——按经验区间给告警，
  永远不阻断。阈值是经验值、非厂商规定，见模块下方常量旁的注释。

单位口径：``range_min`` / ``range_max`` / ``homing_offset`` 都是与标定文件同一刻度系的
tick（每圈 :data:`TICK_RESOLUTION` 个刻度），换算见 :data:`DEG_PER_TICK`。
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

# 参与标定身份（calib_id）的五个字段，顺序即规范形里的字段顺序。
CANONICAL_FIELDS: tuple[str, ...] = (
    "id",
    "drive_mode",
    "homing_offset",
    "range_min",
    "range_max",
)
# 一台 AlohaMini 的电机总数：双臂各 7 台 + 底盘 3 个轮子 + 升降轴 1 根。
EXPECTED_MOTOR_COUNT = 18
# 编码器每圈刻度数（与标定文件同一刻度系）。
TICK_RESOLUTION = 4096
# 1 tick 对应的角度，等于 360 / 4096 = 0.087890625。
DEG_PER_TICK = 360.0 / TICK_RESOLUTION
# calib_id 取 sha256 十六进制串的前多少位：12 位十六进制 ≈ 48 bit，够本地登记表用。
CALIB_ID_LENGTH = 12
# drive_mode 的合法取值：0 与 1 对应两种方向约定。
ALLOWED_DRIVE_MODES = (0, 1)

# ---- 以下判定阈值全部是**经验值、非厂商规定**，来自本项目已看过的那几代真实标定 ----
# 换机型或换批次后要重新核对，别当规格用。
# 夹爪行程宽度：约 120–135 度（旧代际左夹爪只有 51 tick ≈ 4.5 度，属已知异常）。
GRIPPER_WIDTH_DEG_RANGE = (120.0, 135.0)
# 肩（shoulder_pan / shoulder_lift）行程宽度：约 190–240 度。
SHOULDER_WIDTH_DEG_RANGE = (190.0, 240.0)
# 腕（wrist_flex / wrist_yaw / wrist_roll）行程宽度：约 160–190 度。
WRIST_WIDTH_DEG_RANGE = (160.0, 190.0)
# (0, 4095) 满圈是厂商标定的占位值，不是量出来的行程：放行，且不做宽度判定。
PLACEHOLDER_FULL_RANGE = (0, 4095)
# 归零位：标定的 half-turn 归零把 ``Present_Position`` 定在 2047 附近。
ZERO_POSITION_TICK = 2047
# range 中心离归零位超过这个角度就告警（数量级核对，约半圈的半数）。
ZERO_POINT_OFFSET_WARN_DEG = 90.0

# 电机名后缀 → 该族的经验宽度区间；不在表里的电机（底盘轮子、升降轴）不判宽度。
_WIDTH_RANGE_BY_SUFFIX: tuple[tuple[str, tuple[float, float]], ...] = (
    ("_gripper", GRIPPER_WIDTH_DEG_RANGE),
    ("_shoulder_pan", SHOULDER_WIDTH_DEG_RANGE),
    ("_shoulder_lift", SHOULDER_WIDTH_DEG_RANGE),
    ("_wrist_flex", WRIST_WIDTH_DEG_RANGE),
    ("_wrist_yaw", WRIST_WIDTH_DEG_RANGE),
    ("_wrist_roll", WRIST_WIDTH_DEG_RANGE),
)
# 文档级键名：出现在顶层就说明这份 JSON 不是「顶层即电机映射」的标定文件。
_DOCUMENT_KEYS = frozenset(
    {
        "robot_metadata",
        "_robot_metadata",
        "motors",
        "captured_utc",
        "observation_state",
        "schema_version",
        "robot_model",
    }
)
# 底盘与升降轴的电机名特征：它们单独变动就不算「只动手臂」。
_GROUND_PLANE_PREFIX = "base_"
_GROUND_PLANE_NAMES = frozenset({"lift_axis"})


class CalibrationStructureError(ValueError):
    """标定结构不可用：认不出的输入形状、缺必需字段、值不是整数。"""


class ChangePattern(Enum):
    """两代标定之间的变化形态。

    四条的判定**顺序**（先命中先返回，与 :func:`_classify_pattern` 逐条对应）：

    1. ``UNCHANGED``：无差异；
    2. ``ARM_ONLY``：变动电机的名字全部以 ``arm_`` 开头（**只看名字前缀，不看键集合是否完整**）；
    3. ``BROAD``：18 台全变，或变动里出现**名字以 ``base_`` 开头**的电机或 ``lift_axis``；
       这一态**优先于** ``REGION_INCONSISTENT``——例如把 ``arm_left_gripper`` 换成
       ``base_extra_wheel``，算出的是 ``BROAD`` 而不是 ``REGION_INCONSISTENT``；
    4. ``REGION_INCONSISTENT``：其余（变动里出现不属于那 18 台的电机名，且不命中前两条）。

    本枚举只描述「两份电机表之间的差异形状」，**不守表完整性**：18 台是否齐、字段是否全、
    有没有 ``None``，由 :func:`validate_structure` 与 ``tools/calib_health.py`` 的 18 台校验守。
    """

    UNCHANGED = "UNCHANGED"
    ARM_ONLY = "ARM_ONLY"
    BROAD = "BROAD"
    REGION_INCONSISTENT = "REGION_INCONSISTENT"


@dataclass(frozen=True)
class FieldDiff:
    """一个标定字段在代际间的差异，tick 与度两套口径都给。

    ``old`` / ``new`` 是同一刻度系的 tick 值；``delta_deg`` 由
    ``delta_tick * 360 / 4096`` 得出，只是同一事实的另一种写法。
    """

    motor: str
    field: str
    old: int
    new: int
    delta_tick: int
    delta_deg: float


@dataclass(frozen=True)
class ChangeReport:
    """两代标定的变化分类结果：形态 + 逐字段差异 + 变动电机名单。"""

    pattern: ChangePattern
    diffs: tuple[FieldDiff, ...]
    changed_motors: tuple[str, ...]


def is_tick_value(value: Any) -> bool:
    """``value`` 是否是合法 tick 值：``int`` 且不是 ``bool``。

    ``bool`` 是 ``int`` 的子类，但 ``True`` / ``False`` 不是刻度数，所以显式排除。
    """
    return isinstance(value, int) and not isinstance(value, bool)


def _as_int(motor: str, field: str, value: Any) -> int:
    """把标定字段收成 ``int``；``None``、``bool``、浮点、字符串一律拒绝。"""
    if not is_tick_value(value):
        raise CalibrationStructureError(
            f"电机 {motor} 的字段 {field} 不是整数：{value!r}（类型 {type(value).__name__}）"
        )
    return value


def extract_motors(document: Any) -> dict[str, Any]:
    """从标定文件或 v2 指纹里取出「电机名 → 条目」映射。

    认三种形状（按顺序）：

    1. v2 指纹：``{"robot_metadata": {"motors": {...}}}``；
    2. 指纹简写：``{"motors": {...}}``；
    3. 标定文件：顶层直接就是电机映射，即每个值都是映射、且键名都不是文档级键名。

    认不出就抛 :class:`CalibrationStructureError`，不静默返回空表。
    """
    if not isinstance(document, Mapping):
        raise CalibrationStructureError(f"顶层不是映射，而是 {type(document).__name__}")
    if "robot_metadata" in document:
        metadata = document["robot_metadata"]
        motors = metadata.get("motors") if isinstance(metadata, Mapping) else None
        if not isinstance(motors, Mapping):
            raise CalibrationStructureError("指纹里取不到 robot_metadata.motors 映射")
        return dict(motors)
    if "motors" in document:
        motors = document["motors"]
        if not isinstance(motors, Mapping):
            raise CalibrationStructureError("顶层 motors 不是映射")
        return dict(motors)
    looks_like_motors = bool(document) and all(
        isinstance(entry, Mapping) for entry in document.values()
    )
    if looks_like_motors and not (_DOCUMENT_KEYS & set(document)):
        return dict(document)
    raise CalibrationStructureError(
        "认不出的顶层形状：既没有 robot_metadata.motors，也不是「顶层即电机映射」的标定文件"
    )


def canonicalize_motors(motors: Mapping[str, Any]) -> dict[str, dict[str, int]]:
    """把电机映射收成规范形：只留 :data:`CANONICAL_FIELDS`，值一律 ``int``。

    电机名按字典序排列；指纹独有的 ``model`` / ``normalization`` / ``resolution`` 等字段
    在此丢弃。缺必需字段或值不可收成整数时抛 :class:`CalibrationStructureError`。
    """
    if not isinstance(motors, Mapping):
        raise CalibrationStructureError(f"电机表不是映射，而是 {type(motors).__name__}")
    canonical: dict[str, dict[str, int]] = {}
    for name in sorted(motors):
        entry = motors[name]
        if not isinstance(entry, Mapping):
            raise CalibrationStructureError(f"电机 {name} 的条目不是映射")
        row: dict[str, int] = {}
        for field in CANONICAL_FIELDS:
            if field not in entry:
                raise CalibrationStructureError(f"电机 {name} 缺必需字段 {field}")
            row[field] = _as_int(name, field, entry[field])
        canonical[str(name)] = row
    return canonical


def canonical_json(motors: Mapping[str, Any]) -> str:
    """规范形 JSON：键排序、紧凑分隔符、不转义非 ASCII。"""
    return json.dumps(
        canonicalize_motors(motors),
        separators=(",", ":"),
        sort_keys=True,
        ensure_ascii=False,
    )


def calib_id(motors: Mapping[str, Any]) -> str:
    """标定代际身份：规范 JSON 的 sha256 前 :data:`CALIB_ID_LENGTH` 位十六进制。"""
    digest = hashlib.sha256(canonical_json(motors).encode("utf-8")).hexdigest()
    return digest[:CALIB_ID_LENGTH]


def file_sha256(path: str | Path) -> str:
    """整份文件的 sha256（十六进制全串）。

    这只是溯源辅助：同样的 18×5 标定值换个字节格式（缩进、键顺序）就算出不同的文件哈希，
    所以身份判定一律用 :func:`calib_id`，不用文件哈希。
    """
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_structure(motors: Mapping[str, Any]) -> list[str]:
    """结构校验：返回问题清单，空清单 = 通过。

    只报不抛，也不判断严重程度；哪些问题算「文件损坏」由调用方决定
    （例如 ``tools/calib_health.py`` 把缺字段 / ``None`` / 区间反向归到 ``CALIB_CORRUPT``）。
    """
    if not isinstance(motors, Mapping):
        return [f"电机表不是映射，而是 {type(motors).__name__}"]
    problems: list[str] = []
    if len(motors) != EXPECTED_MOTOR_COUNT:
        problems.append(f"电机数量是 {len(motors)}，期望 {EXPECTED_MOTOR_COUNT} 台")
    for name in sorted(motors):
        entry = motors[name]
        if not isinstance(entry, Mapping):
            problems.append(f"电机 {name} 的条目不是映射（{type(entry).__name__}）")
            continue
        for field in CANONICAL_FIELDS:
            if field not in entry:
                problems.append(f"电机 {name} 缺必需字段 {field}")
                continue
            value = entry[field]
            if value is None:
                problems.append(
                    f"电机 {name} 的字段 {field} 是 None（空值不等于 0，也不是合法标定）"
                )
                continue
            if not is_tick_value(value):
                problems.append(
                    f"电机 {name} 的字段 {field} 不是整数：{value!r}（类型 {type(value).__name__}）"
                )
                continue
            if field == "drive_mode" and value not in ALLOWED_DRIVE_MODES:
                problems.append(
                    f"电机 {name} 的 drive_mode 是 {value}，只允许 "
                    f"{' 或 '.join(str(item) for item in ALLOWED_DRIVE_MODES)}"
                )
        low, high = entry.get("range_min"), entry.get("range_max")
        if is_tick_value(low) and is_tick_value(high) and low >= high:
            problems.append(f"电机 {name} 的 range_min={low} >= range_max={high}（区间反向或退化）")
    return problems


def _width_range_for(name: str) -> tuple[float, float] | None:
    """按电机名后缀取该族的经验宽度区间；不认识的电机返回 ``None``。"""
    for suffix, width_range in _WIDTH_RANGE_BY_SUFFIX:
        if name.endswith(suffix):
            return width_range
    return None


def describe_plausibility() -> tuple[str, ...]:
    """把合理性判定用的经验阈值渲染成可打印的行（供 ``tools/calib_register.py`` 展示）。

    这些数字是**经验值、非厂商规定**，只在看过的几代真实标定上核对过。
    """
    return (
        "经验区间（经验值、非厂商规定，换机型或换批次要重新核对）："
        f"夹爪 {GRIPPER_WIDTH_DEG_RANGE[0]:.0f}–{GRIPPER_WIDTH_DEG_RANGE[1]:.0f}°、"
        f"肩 {SHOULDER_WIDTH_DEG_RANGE[0]:.0f}–{SHOULDER_WIDTH_DEG_RANGE[1]:.0f}°、"
        f"腕 {WRIST_WIDTH_DEG_RANGE[0]:.0f}–{WRIST_WIDTH_DEG_RANGE[1]:.0f}°",
        f"{PLACEHOLDER_FULL_RANGE} 满圈视为占位行程，放行且不做宽度判定；"
        f"range 中心偏离归零位 {ZERO_POSITION_TICK} 超过 "
        f"{ZERO_POINT_OFFSET_WARN_DEG:.0f}° 即告警",
    )


def validate_plausibility(motors: Mapping[str, Any]) -> list[str]:
    """合理性校验：返回告警清单，**永不阻断**。

    两类判定，阈值都是经验值、非厂商规定（见模块常量）：

    1. 行程宽度落在该族经验区间之外（按电机名后缀分族：夹爪 / 肩 / 腕）；
       ``(0, 4095)`` 满圈占位值放行，不做宽度判定。
    2. ``range_min`` / ``range_max`` 的中点离归零位 :data:`ZERO_POSITION_TICK` 超过
       :data:`ZERO_POINT_OFFSET_WARN_DEG`——即 ``homing_offset`` 把整个行程推得离归零位
       太远，数量级不对。

    字段缺失或不是整数时直接跳过：那是 :func:`validate_structure` 的活。
    """
    warnings: list[str] = []
    if not isinstance(motors, Mapping):
        return warnings
    for name in sorted(motors):
        entry = motors[name]
        if not isinstance(entry, Mapping):
            continue
        low, high = entry.get("range_min"), entry.get("range_max")
        if not (is_tick_value(low) and is_tick_value(high)):
            continue
        if (low, high) == PLACEHOLDER_FULL_RANGE:
            continue
        width_ticks = high - low
        width_deg = width_ticks * DEG_PER_TICK
        expected = _width_range_for(name)
        if expected is not None and not (expected[0] <= width_deg <= expected[1]):
            warnings.append(
                f"电机 {name} 的 range 宽度 {width_ticks} tick ≈ {width_deg:.1f}°，"
                f"不在经验区间 {expected[0]:.0f}–{expected[1]:.0f}° 内（经验值、非厂商规定）"
            )
        center_deg = ((low + high) / 2 - ZERO_POSITION_TICK) * DEG_PER_TICK
        if abs(center_deg) > ZERO_POINT_OFFSET_WARN_DEG:
            warnings.append(
                f"电机 {name} 的 range 中心偏离归零位 {ZERO_POSITION_TICK} 约 "
                f"{center_deg:+.1f}°，超出 {ZERO_POINT_OFFSET_WARN_DEG:.0f}° 的数量级上限"
                "（经验值、非厂商规定）"
            )
    return warnings


def _field_diff(
    motor: str, old: Mapping[str, int], new: Mapping[str, int], field: str
) -> FieldDiff | None:
    """单个字段的差异；两侧相等返回 ``None``。"""
    old_value, new_value = old[field], new[field]
    if old_value == new_value:
        return None
    delta_tick = new_value - old_value
    return FieldDiff(
        motor=motor,
        field=field,
        old=old_value,
        new=new_value,
        delta_tick=delta_tick,
        delta_deg=delta_tick * 360.0 / TICK_RESOLUTION,
    )


def _is_ground_plane(motor: str) -> bool:
    """是否是底盘轮子或升降轴（不属于手臂）。"""
    return motor.startswith(_GROUND_PLANE_PREFIX) or motor in _GROUND_PLANE_NAMES


def _classify_pattern(changed_motors: tuple[str, ...]) -> ChangePattern:
    """按变动电机名单定形态；判定顺序即优先级，先命中先返回。

    1. 变动集合为空 → ``UNCHANGED``；
    2. 变动集合非空、且全部是 ``arm_*`` → ``ARM_ONLY``；
    3. 变动台数等于 :data:`EXPECTED_MOTOR_COUNT`（18 台全变）→ ``BROAD``；
    4. 变动集合里出现**名字以 ``base_`` 开头**的电机或 ``lift_axis`` → ``BROAD``
       （含底盘 / 升降变动就算全量，不看变动面大小；认的是 ``base_`` 前缀，不是
       ``base_*_wheel`` 后缀——``_is_ground_plane`` 只看 ``startswith("base_")``）；
    5. 其余 → ``REGION_INCONSISTENT``：例如变动集合里出现 18 台之外的电机名
       （输入多一个键或少一个键，两边的电机表压根不是同一套）。
    """
    if not changed_motors:
        return ChangePattern.UNCHANGED
    if all(motor.startswith("arm_") for motor in changed_motors):
        return ChangePattern.ARM_ONLY
    if len(changed_motors) == EXPECTED_MOTOR_COUNT:
        return ChangePattern.BROAD
    if any(_is_ground_plane(motor) for motor in changed_motors):
        return ChangePattern.BROAD
    return ChangePattern.REGION_INCONSISTENT


def classify_change(old: Mapping[str, Any], new: Mapping[str, Any]) -> ChangeReport:
    """把「旧代际 → 新代际」的变化分四态。

    判定顺序（先命中先返回）：

    1. 无任何差异 → :attr:`ChangePattern.UNCHANGED`；
    2. 变动电机的名字全部以 ``arm_`` 开头 → :attr:`ChangePattern.ARM_ONLY`
       （真实旧→新代际就是这一态：14 台手臂电机动、3 个轮子与 ``lift_axis`` 未动）。
       **这一态只看名字前缀，不看键集合是否完整**：单侧多一个 ``arm_left_extra`` 或少一台
       ``arm_left_gripper`` 都仍判 ``ARM_ONLY``；表完整性不在本函数的责任范围。
    3. 变动台数等于 :data:`EXPECTED_MOTOR_COUNT`（18 台全变）→ :attr:`ChangePattern.BROAD`；
    4. 变动里出现 ``base_*`` / ``lift_axis`` → :attr:`ChangePattern.BROAD`
       （含底盘 / 升降变动就算全量，不看变动了几台）；
    5. 其余 → :attr:`ChangePattern.REGION_INCONSISTENT`。这一态只在变更集合里出现**不属于
       那 18 台**的电机名时产生（例如多一个 ``spare_motor``）：不是全部 ``arm_*``、不是 18 台
       全变、也没有 ``base_*`` / ``lift_axis``，剩下可解释的理由就是「两边压根不是同一套
       电机表」。

    **表完整性不由本函数守**：18 台是否齐、字段是否齐、有没有 ``None`` 或非整数，一律由
    :func:`validate_structure` 判（``tools/calib_health.py`` 另加 18 台校验）。本函数假定两边
    的字段已可收成整数（否则抛 :class:`CalibrationStructureError`），只回答「差在哪些字段上」。

    ``diffs`` 只覆盖 :data:`CANONICAL_FIELDS` 里的五个字段，不含指纹独有的
    ``model`` / ``normalization`` / ``resolution``；只在某一侧出现的电机算「有变动」，
    但不产生逐字段差异（没有另一侧的取值可比，不编造 ``0``）。
    """
    old_canonical = canonicalize_motors(old)
    new_canonical = canonicalize_motors(new)
    diffs: list[FieldDiff] = []
    changed: list[str] = []
    for motor in sorted(set(old_canonical) | set(new_canonical)):
        before, after = old_canonical.get(motor), new_canonical.get(motor)
        if before is None or after is None:
            changed.append(motor)
            continue
        motor_diffs = [
            diff
            for field in CANONICAL_FIELDS
            if (diff := _field_diff(motor, before, after, field)) is not None
        ]
        if motor_diffs:
            changed.append(motor)
            diffs.extend(motor_diffs)
    return ChangeReport(
        pattern=_classify_pattern(tuple(changed)),
        diffs=tuple(diffs),
        changed_motors=tuple(changed),
    )
