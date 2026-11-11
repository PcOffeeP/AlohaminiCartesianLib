"""标定包络判定：经验阈值与四态结论的唯一来源。

「包络」= 一台电机的实测行程该落在什么范围里，人话：这份标定像不像一台整机的真标定。

**阈值只有一处**，就是本模块的四个常量。改阈值只改这里；
:func:`alohamini_cartesian.calibration.identity.describe_plausibility` /
:func:`alohamini_cartesian.calibration.identity.validate_plausibility` 是本模块的薄封装，
不再自带任何数字。依赖方向是 ``identity ← envelope``：本模块用 ``identity`` 的刻度换算与
``calib_id``，``identity`` 只在函数体内延迟 import 本模块（避免模块级双向 import 成环）。

四态（:class:`EnvelopeVerdict`）：

===========================  ==========================================================
``OK``                       行程宽度落在经验区间内（或该电机不属任何宽度族），可继续
``UNMEASURED_RANGE``         ``(0, 4095)`` 满圈占位：放行、不计不合格、不做宽度判定
``IMPLAUSIBLE``              行程宽度超出经验区间：提醒、可继续（要留痕）
``WRONG_DATASET``            只有手臂、没有底盘与升降轴：遥操臂标定，不可继续
===========================  ==========================================================

判定流程（:func:`classify_motors`）：

1. **数据集形状**：既没有任何 ``base_*``、也没有 ``lift_axis`` → ``WRONG_DATASET``。
   18 台整机标定必然含这两类；缺它们说明拿到的是遥操臂（leader）那一侧的数据集。
2. **逐台电机**（按电机名排序）：

   - ``range_min`` / ``range_max`` 不是整数 → 跳过、不打 verdict（那是
     :func:`~alohamini_cartesian.calibration.identity.validate_structure` 的活）；
   - ``(0, 4095)`` 满圈占位 → ``UNMEASURED_RANGE``，放行；
   - 行程宽度按电机名族判（夹爪 / 肩 / 腕三族，见 :func:`_width_family`）：
     不在经验区间 → ``IMPLAUSIBLE``；
   - 命中接受名单（``motor`` + 实测 range + ``calib_id`` 三者全对上）→ ``OK`` 并带
     「已接受」信息行；只 ``motor`` 对上而 range / ``calib_id`` 对不上 → 名单过期，
     照旧按经验区间判并在文案里写明「名单过期」。

3. **汇总**：``WRONG_DATASET`` > ``IMPLAUSIBLE`` > ``OK`` > ``UNMEASURED_RANGE``。
   占位不算不合格，所以「6 台占位 + 12 台合格」的汇总是 ``OK``。

``id`` / ``drive_mode`` / ``homing_offset`` **不参与任何判定**，也不产生 verdict：
前两个是接线与驱动方向约定（换线由 ``tools/calib_health.py`` 的跨代际比对拦），
``homing_offset`` 是标定时算出来的偏移量，不设包络。

接受名单（``configs/calib-envelope-accepted.yaml``）只登记「实测行程落在经验区间之外、
但已由人现场确认为真实机械限位」的条目；文件坏了按 :class:`AcceptedListError` 抛
（``ValueError`` 的子类：坏名单是可预期的输入错误，不是内部故障）。

依赖 PyYAML（与 ``config/overlay.py``、``calibration/registry.py`` 同一套 try/import 兜底）；
缺依赖时点名 ``alohamini-cartesian[yaml]``。本模块不 import ``lerobot``。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from .identity import DEG_PER_TICK, CalibrationStructureError, calib_id, is_tick_value
from .registry import YAML_EXTRA

try:  # pragma: no cover - 仅在缺少 extra 时触发
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]

# ---- 以下判定阈值全部是**经验值、非厂商规定**，来自本项目已看过的那几代真实标定 ----
# 换机型或换批次后要重新核对，别当规格用。改阈值只改本模块，别在别处再写一套。
# 夹爪行程宽度（经验值、非厂商规定）：两代真标定里三台有效夹爪实测 128.8–130.0°，彼此同量级
# （代际 B 左夹爪 `2018..3483` = 1465 tick ≈ 128.8°、代际 A 右夹爪 1469 tick ≈ 129.1°、
# 代际 B 右夹爪 1479 tick ≈ 130.0°）；区间放宽到 60–160° 以容纳重标定波动，两侧留 ≥30° 余量
# （下界 128.8−60 = 68.8°、上界 160−130.0 = 30.0°）。旧代际左夹爪 51 tick ≈ 4.5° 仍落在区间外，
# 能被抓到。
GRIPPER_WIDTH_DEG_RANGE = (60.0, 160.0)
# 肩（shoulder_pan / shoulder_lift）行程宽度（经验值、非厂商规定）：两代八台肩里七台有效（排除
# 代际 B 的 `arm_right_shoulder_lift` `905..2048` = 1143 tick ≈ 100.5°，本轮仍判不合格）实测
# 193.0–237.3°（最窄代际 B 左肩 lift `1005..3201` = 2196 tick ≈ 193.0°、最宽代际 A 左肩 pan
# `627..3327` = 2700 tick ≈ 237.3°），区间 180–260° 两侧留 ≥13° 余量（下界 193.0−180 = 13.0°、
# 上界 260−237.3 = 22.7°）。代际 B 右肩 pan `1231..3462` = 2231 tick ≈ 196.1° 距新下界 16.1°
# （旧区间 190–240° 时只剩 6.1°，贴边，是本次放宽的来源）。
SHOULDER_WIDTH_DEG_RANGE = (180.0, 260.0)
# 腕（wrist_flex / wrist_yaw / wrist_roll）行程宽度（经验值、非厂商规定）：两代八台有效腕
# 实测 160.5–186.5°（最窄代际 A 右腕 yaw `1053..2879` = 1826 tick ≈ 160.5°、最宽代际 A 右腕 flex
# `969..3091` = 2122 tick ≈ 186.5°），区间 150–200° 两侧留 ≥10° 余量（下界 160.5−150 = 10.5°、
# 上界 200−186.5 = 13.5°）。四台 `*_wrist_roll` 都是满圈占位，不计入上述实测。
WRIST_WIDTH_DEG_RANGE = (150.0, 200.0)
# (0, 4095) 满圈是厂商标定的占位值、不是量出来的行程（经验值、非厂商规定）：放行，且不做宽度判定。
PLACEHOLDER_FULL_RANGE = (0, 4095)
# **阈值类**判据只有上列四类（三族宽度 + 满圈占位）；另有数据集形状（``WRONG_DATASET``）与
# 接受名单两条机制，均非阈值。曾有一条「`range` 中点离归零位太远即告警」的判据，
# 本轮整条删除：它只有夹爪有物理依据（vendor 侧 half-turn 归零），套到肩／腕属外推；
# 其目标场景（旧代际左夹爪 51 tick ≈ 4.5°）已由夹爪宽度区间覆盖。恢复须先给肩／腕的
# 物理依据，见 ``docs/ADR.md`` D23 决定 17。

# 接受名单文件的 schema 版本；改动条目语义时才加。
SCHEMA_VERSION = 1
# 接受名单条目必须具备的字段，顺序即报错时的列举顺序。
ACCEPTED_FIELDS: tuple[str, ...] = (
    "motor",
    "observed_range_min",
    "observed_range_max",
    "measured_on_tick",
    "method",
    "confirmed_by",
    "confirmed_at_utc",
    "calib_id",
    "reason",
)
# 条目里的字符串字段（其余两个是整数 tick 区间）。
_ACCEPTED_TEXT_FIELDS: tuple[str, ...] = (
    "motor",
    "measured_on_tick",
    "method",
    "confirmed_by",
    "confirmed_at_utc",
    "calib_id",
    "reason",
)


class EnvelopeVerdict(Enum):
    """单台电机与整份标定的包络结论。

    成员名与取值都是冻结契约：``UNMEASURED_RANGE`` 的取值是小写的 ``unmeasured_range``，
    其余三个与成员名同形。整份标定用同一个枚举——``WRONG_DATASET`` 只出现在汇总层
    （数据集形状是整份的事，不是某台电机的事）。
    """

    OK = "OK"
    UNMEASURED_RANGE = "unmeasured_range"
    IMPLAUSIBLE = "IMPLAUSIBLE"
    WRONG_DATASET = "WRONG_DATASET"


class AcceptedListError(ValueError):
    """接受名单不可用：文件读不了、YAML 坏、字段缺、取值类型不对、条目重复。"""


@dataclass(frozen=True)
class AcceptedEntry:
    """接受名单里的一条：**某一份具体标定**里某台电机的实测行程已被现场确认。

    九个字段全必填：``motor`` / 实测 ``observed_range_min`` … ``observed_range_max`` /
    ``calib_id`` 是命中判据的三要素，其余五个（``measured_on_tick`` / ``method`` /
    ``confirmed_by`` / ``confirmed_at_utc`` / ``reason``）是留痕，缺了这条名单就没有问责链。
    """

    motor: str
    observed_range_min: int
    observed_range_max: int
    measured_on_tick: str
    method: str
    confirmed_by: str
    confirmed_at_utc: str
    calib_id: str
    reason: str


@dataclass(frozen=True)
class AcceptedList:
    """一份接受名单：条目序列（空模板就是零条）。"""

    entries: tuple[AcceptedEntry, ...] = ()


@dataclass(frozen=True)
class MotorVerdict:
    """一台电机的包络结论与可打印的 ``detail``（人读文案，机器只认 ``verdict``）。"""

    motor: str
    verdict: EnvelopeVerdict
    detail: str


@dataclass(frozen=True)
class EnvelopeReport:
    """整份标定的包络报告：汇总结论 + 逐台结论 + 数据集形状说明（正常时为 ``None``）。"""

    verdict: EnvelopeVerdict
    motors: tuple[MotorVerdict, ...]
    dataset_note: str | None = None

    @property
    def implausible(self) -> tuple[MotorVerdict, ...]:
        """判定为 ``IMPLAUSIBLE`` 的电机（名单已接受的不算）。"""
        return tuple(item for item in self.motors if item.verdict is EnvelopeVerdict.IMPLAUSIBLE)

    @property
    def unmeasured(self) -> tuple[MotorVerdict, ...]:
        """判定为 ``UNMEASURED_RANGE`` 的电机：``(0, 4095)`` 满圈占位，放行不计不合格。"""
        return tuple(
            item for item in self.motors if item.verdict is EnvelopeVerdict.UNMEASURED_RANGE
        )


def rejected(report: EnvelopeReport) -> bool:
    """这份标定是否**不可继续**（``WRONG_DATASET``）。

    只有这一态不可继续；``IMPLAUSIBLE`` 是提醒，调用方负责「要留痕才放行」。
    """
    return report.verdict is EnvelopeVerdict.WRONG_DATASET


def _width_family(name: str) -> tuple[str, tuple[float, float]] | None:
    """按电机名取宽度族（族名、经验区间）；不属三族返回 ``None``。

    三族：``*_gripper``（夹爪）、``*_shoulder_pan`` / ``*_shoulder_lift``（肩）、
    ``*_wrist_flex`` / ``*_wrist_yaw`` / ``*_wrist_roll``（腕）。其余（``*_elbow_flex``、
    ``base_*``、``lift_axis``）**没有宽度判定**。

    区间在**调用时**从模块全局读，所以改常量（含测试里的 monkeypatch）立刻反映到判定与文案。
    """
    if name.endswith("_gripper"):
        return "夹爪", GRIPPER_WIDTH_DEG_RANGE
    if name.endswith(("_shoulder_pan", "_shoulder_lift")):
        return "肩", SHOULDER_WIDTH_DEG_RANGE
    if name.endswith(("_wrist_flex", "_wrist_yaw", "_wrist_roll")):
        return "腕", WRIST_WIDTH_DEG_RANGE
    return None


def _width_deg(low: int, high: int) -> float:
    """行程宽度换算成角度（``(high - low) * 360 / 4096``）。"""
    return (high - low) * DEG_PER_TICK


def _width_note(name: str, low: int, high: int) -> tuple[str | None, str]:
    """宽度判定：返回（不合格文案或 ``None``，合格时的人读说明）。"""
    width_ticks = high - low
    width_deg = _width_deg(low, high)
    family = _width_family(name)
    if family is None:
        return None, (
            f"行程宽度 {width_ticks} tick ≈ {width_deg:.1f}°；"
            "该电机不属夹爪／肩／腕任一宽度族，不做宽度判定"
        )
    label, (expected_low, expected_high) = family
    window = f"{label}经验区间 {expected_low:.0f}–{expected_high:.0f}°（经验值、非厂商规定）"
    if expected_low <= width_deg <= expected_high:
        return None, f"行程宽度 {width_ticks} tick ≈ {width_deg:.1f}°，在{window}内"
    return f"行程宽度 {width_ticks} tick ≈ {width_deg:.1f}°，不在{window}内", ""


def _accepted_detail(entry: AcceptedEntry, problems: list[str]) -> str:
    """名单命中的信息行：谁、什么时候、用什么方法确认了这份行程。"""
    detail = (
        f"已接受：{entry.reason}（method={entry.method}，"
        f"confirmed_by={entry.confirmed_by}，confirmed_at_utc={entry.confirmed_at_utc}，"
        f"measured_on_tick={entry.measured_on_tick}）"
    )
    if problems:
        detail += f"；该条覆盖 {len(problems)} 条经验区间告警"
    return detail


def _stale_note(entry: AcceptedEntry, low: int, high: int, current_id: str) -> str:
    """名单未命中的说明：登记值与本次实测不符，按「名单过期」重新告警。"""
    return (
        f"名单过期：接受名单登记 {entry.motor} 是 range ({entry.observed_range_min}, "
        f"{entry.observed_range_max})、calib_id {entry.calib_id}；本次实测 range ({low}, {high})、"
        f"calib_id {current_id}。{entry.confirmed_by} 于 {entry.confirmed_at_utc} 的确认"
        f"只对那一代标定有效，现场核对后更新 configs/calib-envelope-accepted.yaml"
    )


def _overall_verdict(items: tuple[MotorVerdict, ...], wrong_dataset: bool) -> EnvelopeVerdict:
    """汇总四态：``WRONG_DATASET`` > ``IMPLAUSIBLE`` > ``OK`` > ``UNMEASURED_RANGE``。

    占位（``UNMEASURED_RANGE``）放行、不算不合格，所以只要有一台 ``OK`` 汇总就不是
    ``UNMEASURED_RANGE``；全部都是占位时才落到 ``UNMEASURED_RANGE``（一辆没量过的车）。

    还有第三种落到 ``UNMEASURED_RANGE`` 的情形：``items`` 为空，即**一台可判定的电机都没有**
    （每台都因 ``range_min`` / ``range_max`` 不是整数而被跳过，或条目不是映射）。这与「真量过但
    只有占位」不是一回事：前者是「判不了」，后者是「量了、但值是满圈占位」。两者共用同一枚举
    值，所以读结论时别把空表当成占位证据。实际低概率——上游 :func:`validate_structure`
    会先把「不是整数 / 不是映射」这类表拦下。
    """
    if wrong_dataset:
        return EnvelopeVerdict.WRONG_DATASET
    verdicts = {item.verdict for item in items}
    if EnvelopeVerdict.IMPLAUSIBLE in verdicts:
        return EnvelopeVerdict.IMPLAUSIBLE
    if EnvelopeVerdict.OK in verdicts:
        return EnvelopeVerdict.OK
    return EnvelopeVerdict.UNMEASURED_RANGE


def _dataset_note(motors: Mapping[str, Any]) -> str | None:
    """数据集形状判定：缺 ``base_*`` 与 ``lift_axis`` 时返回「这是遥操臂」的说明。

    这条判据是**粗筛**，只拦「一台底盘轮子与升降轴都没有」的形状：反过来，一份「臂 +
    ``lift_axis`` 但没有轮子」的遥操侧数据集不会被它拦下（只要有 ``lift_axis`` 就算「有地面」），
    只靠 18 台数与结构自洽校验兜底——真正的 leader 数据集电机数少、字段也不全，会先被那两道拦下。
    """
    has_ground_plane = "lift_axis" in motors or any(
        str(name).startswith("base_") for name in motors
    )
    if has_ground_plane:
        return None
    return (
        "只看到手臂关节，没有任何 base_* 底盘轮子、也没有 lift_axis 升降轴："
        "这是遥操臂（leader）那一侧的标定，不是 18 台整机标定，不可用于下发命令"
    )


def _accepted_by_motor(
    motors: Mapping[str, Any], accepted: AcceptedList | None
) -> tuple[dict[str, AcceptedEntry], str | None]:
    """建立「电机名 → 名单条目」索引，并算出本次标定的 ``calib_id``。

    ``calib_id`` 算不出来（结构不可收成 18×5 整数）时返回空索引：那种表由结构校验拦，
    这里不冒充命中、也不凭空造「名单过期」。
    """
    if accepted is None or not accepted.entries:
        return {}, None
    try:
        current_id = calib_id(motors)
    except CalibrationStructureError:
        return {}, None
    return {entry.motor: entry for entry in accepted.entries}, current_id


def classify_motors(
    motors: Mapping[str, Any], accepted: AcceptedList | None = None
) -> EnvelopeReport:
    """给一份「电机名 → 五字段」表打四态结论；``accepted`` 为接受名单（可省）。

    逐台跳过与放行的口径见模块 docstring。返回的 ``motors`` 按电机名排序，与输入顺序无关。
    """
    if not isinstance(motors, Mapping):
        return EnvelopeReport(
            verdict=EnvelopeVerdict.WRONG_DATASET,
            motors=(),
            dataset_note=f"电机表不是映射，而是 {type(motors).__name__}：没有可判定的电机",
        )
    dataset_note = _dataset_note(motors)
    entries_by_motor, current_id = _accepted_by_motor(motors, accepted)

    items: list[MotorVerdict] = []
    for raw_name in sorted(motors, key=str):
        name = str(raw_name)
        entry = motors[raw_name]
        if not isinstance(entry, Mapping):
            continue
        low, high = entry.get("range_min"), entry.get("range_max")
        if not (is_tick_value(low) and is_tick_value(high)):
            continue
        if (low, high) == PLACEHOLDER_FULL_RANGE:
            items.append(
                MotorVerdict(
                    motor=name,
                    verdict=EnvelopeVerdict.UNMEASURED_RANGE,
                    detail=(
                        f"range ({low}, {high}) 是厂商标定的满圈占位值，不是量出来的行程："
                        "放行、不做宽度判定"
                    ),
                )
            )
            continue

        width_problem, width_note = _width_note(name, low, high)
        problems = [width_problem] if width_problem else []
        notes = [width_note] if width_note else []
        accepted_entry = entries_by_motor.get(name)
        if accepted_entry is not None and current_id is not None:
            registered_range = (
                accepted_entry.observed_range_min,
                accepted_entry.observed_range_max,
            )
            if registered_range == (low, high) and accepted_entry.calib_id == current_id:
                items.append(
                    MotorVerdict(
                        motor=name,
                        verdict=EnvelopeVerdict.OK,
                        detail=_accepted_detail(accepted_entry, problems),
                    )
                )
                continue
            notes.append(_stale_note(accepted_entry, low, high, current_id))

        verdict = EnvelopeVerdict.IMPLAUSIBLE if problems else EnvelopeVerdict.OK
        items.append(MotorVerdict(motor=name, verdict=verdict, detail="；".join(problems + notes)))

    return EnvelopeReport(
        verdict=_overall_verdict(tuple(items), wrong_dataset=dataset_note is not None),
        motors=tuple(items),
        dataset_note=dataset_note,
    )


def plausibility_warnings(motors: Mapping[str, Any]) -> list[str]:
    """合理性告警清单：只报告、不阻断，**永不抛异常**。

    条目取不合格电机的 ``detail`` 并补上电机名；数据集形状不对时把那条说明也列上。
    这是 ``identity.validate_plausibility`` 的实体，identity 侧只做转发。
    """
    report = classify_motors(motors)
    warnings = [f"电机 {item.motor}: {item.detail}" for item in report.implausible]
    if report.dataset_note is not None:
        warnings.append(report.dataset_note)
    return warnings


def describe_thresholds() -> tuple[str, ...]:
    """把本模块的经验阈值渲染成可打印的行（供 ``tools/calib_register.py`` 等展示）。

    这些数字是**经验值、非厂商规定**，只在看过的几代真实标定上核对过；
    渲染时读模块全局，改常量即改输出。
    """
    return (
        "经验区间（经验值、非厂商规定，换机型或换批次要重新核对）："
        f"夹爪 {GRIPPER_WIDTH_DEG_RANGE[0]:.0f}–{GRIPPER_WIDTH_DEG_RANGE[1]:.0f}°、"
        f"肩 {SHOULDER_WIDTH_DEG_RANGE[0]:.0f}–{SHOULDER_WIDTH_DEG_RANGE[1]:.0f}°、"
        f"腕 {WRIST_WIDTH_DEG_RANGE[0]:.0f}–{WRIST_WIDTH_DEG_RANGE[1]:.0f}°",
        f"{PLACEHOLDER_FULL_RANGE} 满圈视为占位行程，放行且不做宽度判定",
    )


def _entry_from_document(raw: Any, index: int, path: Path) -> AcceptedEntry:
    """把 YAML 里的一条收成 :class:`AcceptedEntry`；形状不对抛 :class:`AcceptedListError`。"""
    if not isinstance(raw, Mapping):
        raise AcceptedListError(
            f"接受名单第 {index} 条不是映射，而是 {type(raw).__name__}（{path}）"
        )
    missing = [field for field in ACCEPTED_FIELDS if field not in raw]
    if missing:
        raise AcceptedListError(
            f"接受名单第 {index} 条缺必需字段 {', '.join(missing)}（{path}）；"
            f"九个字段全必填，留痕缺一项这条确认就不可追溯"
        )
    for field in _ACCEPTED_TEXT_FIELDS:
        value = raw[field]
        if not isinstance(value, str) or not value.strip():
            raise AcceptedListError(
                f"接受名单第 {index} 条的 {field} 不是非空字符串：{value!r}（{path}）"
            )
    for field in ("observed_range_min", "observed_range_max"):
        value = raw[field]
        if not is_tick_value(value):
            raise AcceptedListError(
                f"接受名单第 {index} 条的 {field} 不是整数 tick 值：{value!r}（{path}）"
            )
    return AcceptedEntry(
        motor=raw["motor"],
        observed_range_min=raw["observed_range_min"],
        observed_range_max=raw["observed_range_max"],
        measured_on_tick=raw["measured_on_tick"],
        method=raw["method"],
        confirmed_by=raw["confirmed_by"],
        confirmed_at_utc=raw["confirmed_at_utc"],
        calib_id=raw["calib_id"],
        reason=raw["reason"],
    )


def load_accepted(path: str | Path) -> AcceptedList:
    """读一份接受名单 YAML；空模板 ``accepted: []`` 得到零条。

    坏名单（读不了、YAML 坏、顶层不是映射、``schema_version`` 不是 :data:`SCHEMA_VERSION`、
    ``accepted`` 不是列表、条目缺字段或取值类型不对、同 ``motor`` 重复）一律抛
    :class:`AcceptedListError`，不静默丢条目。
    """
    if yaml is None:  # pragma: no cover - 仅在缺少 extra 时触发
        raise AcceptedListError(f"读接受名单需要 PyYAML，请安装 {YAML_EXTRA}")
    accepted_path = Path(path)
    try:
        text = accepted_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise AcceptedListError(f"接受名单读不了：{accepted_path}（{exc}）") from exc
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise AcceptedListError(f"接受名单不是合法 YAML：{accepted_path}（{exc}）") from exc
    if not isinstance(document, Mapping):
        raise AcceptedListError(
            f"接受名单顶层不是映射，而是 {type(document).__name__}（{accepted_path}）"
        )
    version = document.get("schema_version")
    if version != SCHEMA_VERSION:
        raise AcceptedListError(
            f"接受名单的 schema_version 是 {version!r}，本库只认 {SCHEMA_VERSION}"
            f"（{accepted_path}）"
        )
    raw_entries = document.get("accepted")
    if not isinstance(raw_entries, list):
        raise AcceptedListError(
            f"接受名单的 accepted 不是列表，而是 {type(raw_entries).__name__}"
            f"（{accepted_path}）；空名单要写成 accepted: []"
        )
    entries = tuple(
        _entry_from_document(raw, index, accepted_path)
        for index, raw in enumerate(raw_entries, start=1)
    )
    seen: set[str] = set()
    duplicates: list[str] = []
    for entry in entries:
        if entry.motor in seen:
            duplicates.append(entry.motor)
        seen.add(entry.motor)
    if duplicates:
        repeated = ", ".join(sorted(set(duplicates)))
        raise AcceptedListError(f"接受名单里同一台电机登记了多次：{repeated}（{accepted_path}）")
    return AcceptedList(entries=entries)
