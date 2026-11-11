"""三层关节限位的判定工具：URDF 结构限位 / 现场实测极限 / 标定 range。

**本模块不接入控制路径。** ``control/controller.py`` 与 ``control/api.py`` 仍然只走
``JointKinematicAdapter.effective_limits_deg``。理由：把第三层判定接进控制循环的前提，是
现场实测层（层 2）真的有一批可用数据；本轮没有这类数据（``configs/limits-observed.yaml``
是空模板）。在数据缺席时先把分支接进去，等于凭空改变运动行为——拒绝与放行的边界都会变，
而没有任何一条真机证据支持新边界。所以本轮只把判定写成可测的纯函数，接入留到有实测层之后。

三层各自的权威性：

======  ==========================  ========================================
层     来源                        ``source_of_each_bound`` 的取值
======  ==========================  ========================================
1      URDF 机型声明               ``hard_min`` / ``hard_max`` = ``"urdf"``
2      现场实测极限                ``hard_min`` / ``hard_max`` = ``"observed"``
3      标定 range                  ``calib_min`` / ``calib_max`` = ``"calibration"``
缺席   ——                          ``"absent"``（**不拿别的层顶替**）
======  ==========================  ========================================

硬限 = 层 1 ∩ 层 2（有实测时）；没有实测层时硬限就是 URDF；两层都没有就是 ``None``。
**实测层缺失时不得用标定值顶替硬限**：标定 range 是「标定时量到的行程」，不是「结构上能到
哪里」，把两者混为一谈会让结构限位悄悄变成标定限位。

单位：三层数值必须由**调用方**放进同一个单位后传入（测试统一用度）。本模块不做 tick ↔ 度
换算，因为那需要按代际取的 ``signs`` / ``zeros``——同一台电机在不同代际的换算系数可能不同，
本模块拿不到代际上下文，静默假设一个换算系数就是造假数据。

判定顺序（先命中先返回）：

1. 硬限自身交叉（``hard_min > hard_max``）→ :data:`LIMIT_CONFLICT`，文案按在场层数分流：
   两层都在 → 「结构限位与实测层互相矛盾」；只有层 1 → URDF 自身倒挂；只有层 2 → 实测层
   自身倒挂。三种都**不提重标定**——矛盾不在标定侧；
2. 硬限有效、但与标定 range 交集为空 → :data:`LIMIT_CONFLICT`，这一支才提示重标定；
3. 目标出硬限 → :data:`STRUCTURAL_LIMIT`；
4. 目标在硬限内、出标定 range → :data:`CALIB_RANGE_NARROWED`；若调用方显式传
   ``allow_outside_calib_range=True``，则放行并留下 :class:`OverrideRecord` 留痕。

拒绝路径一律返回结构化结果（``allowed=False`` + ``reason_code`` + 人话 ``message``），
不抛裸异常，``override_record`` 为 ``None``。覆盖开关只对层 3 生效，不放开硬限。

入参形状错误不属于「三层判定失败」：三个限位参数不是 ``(low, high)`` 两个数时抛带字段名的
:class:`ValueError`，``target`` 不是有限数时抛 :class:`ValueError`（非数值类型则由
``math.isfinite`` 抛 :class:`TypeError`）。这些都是调用方的编程错误，应当响亮失败；
**三层判定失败一律走** :class:`LimitDecision`。
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .identity import is_tick_value

try:  # pragma: no cover - 仅在缺少 extra 时触发
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]

# 缺 PyYAML 时错误信息里点名的 extra。
YAML_EXTRA = "alohamini-cartesian[yaml]"
# 实测层文件的 schema 版本。
OBSERVED_SCHEMA_VERSION = 1

# 结构化原因码（不是枚举，因为调用方会把它直接写进日志与 CLI 输出）。
STRUCTURAL_LIMIT = "STRUCTURAL_LIMIT"
CALIB_RANGE_NARROWED = "CALIB_RANGE_NARROWED"
LIMIT_CONFLICT = "LIMIT_CONFLICT"

# 边界来源标记。
SOURCE_URDF = "urdf"
SOURCE_OBSERVED = "observed"
SOURCE_CALIBRATION = "calibration"
SOURCE_ABSENT = "absent"


class ObservedLimitError(ValueError):
    """实测限位文件不可用：缺 PyYAML、YAML 坏、或条目不合冻结 schema。"""


@dataclass(frozen=True)
class ObservedLimit:
    """一台电机的一次现场实测极限，带溯源。

    ``min_tick`` / ``max_tick`` 与标定文件同一 tick 系；``calib_id`` 是这批实测对应的代际，
    为 ``None`` 表示「这批实测没绑定代际」——它不能冒充「绑定到了某一代」。
    """

    min_tick: int
    max_tick: int
    measured_on: str | None = None
    calib_id: str | None = None
    method: str | None = None


@dataclass(frozen=True)
class OverrideRecord:
    """一次显式覆盖的留痕：为什么放行了、放行的是哪个值、当时的边界是什么。"""

    motor: str
    target: float
    hard_limit: tuple[float, float] | None
    calib_range: tuple[float, float] | None
    reason: str


@dataclass(frozen=True)
class LimitDecision:
    """一台电机在一个目标值上的限位判定结果。"""

    motor: str
    target: float
    allowed: bool
    reason_code: str | None
    hard_limit: tuple[float, float] | None
    calib_range: tuple[float, float] | None
    source_of_each_bound: dict[str, str]
    message: str
    override_record: OverrideRecord | None = None


def _pair(limit: Sequence[float] | None, label: str) -> tuple[float, float] | None:
    """把 ``(low, high)`` 收成两个浮点的元组；形状不对抛带 ``label`` 的 :class:`ValueError`。"""
    if limit is None:
        return None
    if isinstance(limit, (str, bytes)):
        raise ValueError(f"{label} 必须是 (low, high) 两个数，实际是 {limit!r}")
    # 只取一次序列：传进来的是生成器时，二次取值会拿到空序列、丢掉下面的诊断信息。
    values = tuple(limit)
    if len(values) != 2:
        raise ValueError(f"{label} 必须是 (low, high) 两个数，实际是 {values!r}")
    return (float(values[0]), float(values[1]))


def _merge_hard_limit(
    urdf: tuple[float, float] | None, observed: tuple[float, float] | None
) -> tuple[tuple[float, float] | None, dict[str, str]]:
    """硬限 = URDF ∩ 实测；逐边界记录「哪一层给了这个边界」。

    只给一层的边界就记那一层；两层都给了取更紧的一侧（相等时记 ``observed``，
    因为实测层是更具体的那一侧）。两层都没有就是 ``None`` + ``absent``。
    """
    sources = {"hard_min": SOURCE_ABSENT, "hard_max": SOURCE_ABSENT}
    lowers: list[tuple[str, float]] = []
    uppers: list[tuple[str, float]] = []
    if urdf is not None:
        lowers.append((SOURCE_URDF, urdf[0]))
        uppers.append((SOURCE_URDF, urdf[1]))
    if observed is not None:
        lowers.append((SOURCE_OBSERVED, observed[0]))
        uppers.append((SOURCE_OBSERVED, observed[1]))
    if not lowers:
        return None, sources
    lower_source, lower = max(lowers, key=lambda item: (item[1], item[0] == SOURCE_OBSERVED))
    upper_source, upper = min(uppers, key=lambda item: (item[1], item[0] != SOURCE_OBSERVED))
    sources["hard_min"] = lower_source
    sources["hard_max"] = upper_source
    return (lower, upper), sources


def _calib_sources(calib: tuple[float, float] | None) -> dict[str, str]:
    """标定 range 的边界来源：有就是 ``calibration``，没有就是 ``absent``。"""
    source = SOURCE_CALIBRATION if calib is not None else SOURCE_ABSENT
    return {"calib_min": source, "calib_max": source}


def _bounds_text(limit: tuple[float, float] | None) -> str:
    """把边界渲染成 ``[low, high]``；``None`` 渲染成「缺席」。"""
    return "缺席" if limit is None else f"[{limit[0]:g}, {limit[1]:g}]"


def _intersects(left: tuple[float, float] | None, right: tuple[float, float] | None) -> bool:
    """两个区间是否有交集；任一侧缺席时不判（返回 ``True``）。"""
    if left is None or right is None:
        return True
    return max(left[0], right[0]) <= min(left[1], right[1])


def _crossed_hard_limit_message(
    motor: str,
    urdf: tuple[float, float] | None,
    observed: tuple[float, float] | None,
    hard_limit: tuple[float, float],
) -> str:
    """硬限自身交叉（``hard_min > hard_max``）的文案：按在场层数分流。

    三种形态各自点名真正矛盾的那一层：两层都在 → 层 1 与层 2 之间；只有层 1 → URDF 自身
    倒挂；只有层 2 → 实测层自身倒挂。这一支**从不提「重标定」**：矛盾不在标定侧。
    """
    if urdf is not None and observed is not None:
        return (
            f"电机 {motor} 的硬限自相矛盾：结构限位给出 {_bounds_text(urdf)}、"
            f"实测层给出 {_bounds_text(observed)}，两侧交集为空（算出 "
            f"{_bounds_text(hard_limit)}）。矛盾在层 1 与层 2 之间：先核对这份实测记录"
            "是否属于该电机、量程方向对不对；标定 range 与这个矛盾无关。"
        )
    if urdf is not None:
        return (
            f"电机 {motor} 的结构限位（URDF）自身倒挂：{_bounds_text(urdf)} 的下界大于上界。"
            "先核对机型与限位参数，改 URDF 参数后重跑；标定 range 与这个矛盾无关。"
        )
    return (
        f"电机 {motor} 的实测层自身倒挂：{_bounds_text(observed)} 的下界大于上界，"
        "先核对这份记录（电机是否对应、量程方向是否写反）。"
    )


def resolve_joint_limit(
    *,
    motor: str,
    target: float,
    urdf_limit: Sequence[float] | None = None,
    observed: Sequence[float] | None = None,
    calib_range: Sequence[float] | None = None,
    allow_outside_calib_range: bool = False,
) -> LimitDecision:
    """判一个目标值是否可下发：三层限位逐层过，返回结构化结果、不抛异常。

    参数里的三个限位都必须是**同一个单位**（本库统一用度）；``observed`` 是层 2 的实测
    极限，``None`` 表示层 2 缺席，此时硬限退化成 ``urdf_limit``——**不会**拿
    ``calib_range`` 顶上去。

    ``allow_outside_calib_range=True`` **只对层 3（标定 range）生效**：目标出硬限一律拒，
    且拒绝路径不带 :class:`OverrideRecord`（``override_record is None``）。只有「在硬限内、
    出标定 range、被显式放行」这一条路径才产生 :class:`OverrideRecord` 留痕。
    """
    if not math.isfinite(target):
        raise ValueError(f"target 必须是有限数，实际是 {target!r}")
    urdf = _pair(urdf_limit, "urdf_limit")
    observed_limit = _pair(observed, "observed")
    calib = _pair(calib_range, "calib_range")
    hard_limit, sources = _merge_hard_limit(urdf, observed_limit)
    sources.update(_calib_sources(calib))

    def decision(
        *,
        allowed: bool,
        reason_code: str | None,
        message: str,
        override_record: OverrideRecord | None = None,
    ) -> LimitDecision:
        """按当前边界装配结果，避免每处重复写六个字段。"""
        return LimitDecision(
            motor=motor,
            target=target,
            allowed=allowed,
            reason_code=reason_code,
            hard_limit=hard_limit,
            calib_range=calib,
            source_of_each_bound=dict(sources),
            message=message,
            override_record=override_record,
        )

    if hard_limit is not None and hard_limit[0] > hard_limit[1]:
        return decision(
            allowed=False,
            reason_code=LIMIT_CONFLICT,
            message=_crossed_hard_limit_message(motor, urdf, observed_limit, hard_limit),
        )

    if not _intersects(hard_limit, calib):
        return decision(
            allowed=False,
            reason_code=LIMIT_CONFLICT,
            message=(
                f"电机 {motor} 的硬限 {_bounds_text(hard_limit)} 与标定 range "
                f"{_bounds_text(calib)} 交集为空：这份标定与结构/实测边界互相矛盾，"
                "该电机当前不能下发任何目标。请重标定这台电机，并核对标定文件与机型的对应关系。"
            ),
        )

    if hard_limit is not None and not (hard_limit[0] <= target <= hard_limit[1]):
        return decision(
            allowed=False,
            reason_code=STRUCTURAL_LIMIT,
            message=(
                f"电机 {motor} 的目标 {target:g} 超出结构硬限 {_bounds_text(hard_limit)}"
                f"（hard_min 来源 {sources['hard_min']}、hard_max 来源 {sources['hard_max']}）"
                "，属结构限制，不是标定问题。"
            ),
        )

    if calib is not None and not (calib[0] <= target <= calib[1]):
        hard_text = _bounds_text(hard_limit)
        if allow_outside_calib_range:
            override = OverrideRecord(
                motor=motor,
                target=target,
                hard_limit=hard_limit,
                calib_range=calib,
                reason=(
                    f"调用方显式覆盖（allow_outside_calib_range=True）；目标在硬限 {hard_text} "
                    f"内、但出标定 range {_bounds_text(calib)}"
                ),
            )
            return decision(
                allowed=True,
                reason_code=None,
                message=(
                    f"电机 {motor} 的目标 {target:g} 出标定 range {_bounds_text(calib)}，"
                    f"但仍在硬限 {hard_text} 内，且调用方显式要求覆盖，故放行并留痕。"
                ),
                override_record=override,
            )
        return decision(
            allowed=False,
            reason_code=CALIB_RANGE_NARROWED,
            message=(
                f"电机 {motor} 的目标 {target:g} 被标定区间限制：标定 range 是 "
                f"{_bounds_text(calib)}，目标越界；这不是结构限制（结构硬限 {hard_text} "
                "允许该位置，标定只是标定时量到的行程）。两条出路："
                "① 跑一次现场实测，建立层 2 的实测极限（observed），把该位置纳入实测证据；"
                "② 显式覆盖：传 allow_outside_calib_range=True，判定会放行并留下 OverrideRecord。"
            ),
        )

    return decision(
        allowed=True,
        reason_code=None,
        message=(
            f"电机 {motor} 的目标 {target:g} 在硬限 {_bounds_text(hard_limit)} 与标定 range "
            f"{_bounds_text(calib)} 之内，允许下发。"
        ),
    )


def load_observed_limits(path: str | Path | None) -> dict[str, ObservedLimit]:
    """读实测限位文件（``configs/limits-observed.yaml`` 的 schema）。

    ``None``、路径不存在、``safe_load`` 得到空文档、顶层没有 ``motors``、``motors: {}``
    一律返回 ``{}``——空表表示层 2 缺席，**不抛异常、也不编造条目**。文件存在且有条目时
    逐台读出五个字段；条目缺 ``min_tick`` / ``max_tick``、两者不是整数、或
    ``min_tick >= max_tick`` 时抛 :class:`ObservedLimitError`——坏掉的实测记录不能被静默
    丢掉，否则「层 2 缺席」和「层 2 写错了」会变成同一个结果。
    """
    if path is None:
        return {}
    observed_path = Path(path)
    if not observed_path.exists():
        return {}
    if yaml is None:
        raise ObservedLimitError(f"读实测限位文件需要 PyYAML，请安装 {YAML_EXTRA}")
    try:
        text = observed_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ObservedLimitError(f"实测限位文件读不了：{observed_path}（{exc}）") from exc
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ObservedLimitError(f"实测限位文件不是合法 YAML：{observed_path}（{exc}）") from exc
    if not isinstance(document, Mapping):
        return {}
    version = document.get("schema_version")
    if version is not None and version != OBSERVED_SCHEMA_VERSION:
        raise ObservedLimitError(
            f"实测限位文件 schema_version 是 {version!r}，本库只认 {OBSERVED_SCHEMA_VERSION}"
        )
    motors = document.get("motors")
    if not isinstance(motors, Mapping) or not motors:
        return {}
    limits: dict[str, ObservedLimit] = {}
    for name in sorted(motors):
        limits[str(name)] = _parse_observed_entry(str(name), motors[name])
    return limits


def _parse_observed_entry(name: str, raw: Any) -> ObservedLimit:
    """解析一台电机的实测条目；形状不对抛 :class:`ObservedLimitError`。"""
    if not isinstance(raw, Mapping):
        raise ObservedLimitError(f"实测条目 {name} 不是映射，而是 {type(raw).__name__}")
    min_tick, max_tick = raw.get("min_tick"), raw.get("max_tick")
    for field_name, value in (("min_tick", min_tick), ("max_tick", max_tick)):
        if not is_tick_value(value):
            raise ObservedLimitError(f"实测条目 {name} 的 {field_name} 不是整数：{value!r}")
    if min_tick >= max_tick:
        raise ObservedLimitError(
            f"实测条目 {name} 的 min_tick={min_tick} >= max_tick={max_tick}（区间反向或退化）"
        )
    return ObservedLimit(
        min_tick=min_tick,
        max_tick=max_tick,
        measured_on=_optional_text(name, "measured_on", raw.get("measured_on")),
        calib_id=_optional_text(name, "calib_id", raw.get("calib_id")),
        method=_optional_text(name, "method", raw.get("method")),
    )


def _optional_text(name: str, field_name: str, value: Any) -> str | None:
    """可选文本字段：缺席或 ``None`` 记 ``None``，在场但不是字符串则报错。"""
    if value is None:
        return None
    if not isinstance(value, str):
        raise ObservedLimitError(f"实测条目 {name} 的 {field_name} 不是字符串：{value!r}")
    return value
