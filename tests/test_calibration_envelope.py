"""``calibration/envelope.py`` 的离线契约测试（阈值与判定的唯一来源）。

**这是离线契约测试，当前 23 条全绿，不是待实现的红阶段草稿。** 被测模块
``src/alohamini_cartesian/calibration/envelope.py`` 已实现。契约意图不变——断言先于实现，
**缺实现文件要响亮失败、不 ``skip``**：缺文件是「契约没有对象」，不是「可以忽略」。不得为了
让测试变绿去改产品代码迁就，也不得放宽断言。

锁的六件事：

- **四态判定**：``OK`` / ``UNMEASURED_RANGE``（``(0, 4095)`` 满圈占位，放行且不计不合格）/
  ``IMPLAUSIBLE``（提醒、可继续）/ ``WRONG_DATASET``（只有臂、没有底盘与升降轴 → 不可继续，
  文案点出「遥操臂」）。
- **阈值只有一处**：夹爪 120–135°、肩 190–240°、腕 160–190°、占位 ``(0, 4095)``、归零位
  2047、中心偏离 90°，常量名与值都冻在 ``envelope`` 里；``identity.describe_plausibility`` /
  ``identity.validate_plausibility`` 降级成薄封装（改 ``envelope`` 常量要反映到输出）。
- **身份字段不参与判定**：``id`` / ``drive_mode`` / ``homing_offset`` 怎么变都不改 verdict
  与 ``detail``；range 不是整数交给结构校验，``envelope`` 跳过该电机、不打 verdict。
- **宽度判定按电机名族**：夹爪 / 肩 / 腕三族按宽度区间判；``*_elbow_flex``、``base_*``、
  ``lift_axis`` 无宽度判定 → ``OK``。
- **中心偏离**：宽度合格但 range 中心离归零位 2047 超过 90° → 仍 ``IMPLAUSIBLE``。
- **接受名单**：``motor`` + ``observed_range_min/max`` + ``calib_id`` 三者都对上 → 该电机
  ``OK`` 并带「已接受」信息行；只 motor 对上而 range / ``calib_id`` 对不上 → 名单过期，
  照旧 ``IMPLAUSIBLE`` 且 ``detail`` 含「名单过期」。

本文件只覆盖模块层（阈值、四态、名单、identity 薄封装）；把它接到 CLI 的那一层——门禁顺序、
退出码、失败路径不落盘、``--dry-run`` 与 ssh 路径的具体文案——由
``tests/test_calib_fetch.py`` 覆盖，两边不重复。

全部用例离线：只读仓库内的两份真机 fixture 或纯合成的电机表，不连设备、不联网。
"""

from __future__ import annotations

import copy
import dataclasses
import importlib
import json
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
import yaml

from alohamini_cartesian.calibration import identity

_REPO_ROOT = Path(__file__).resolve().parents[1]
_ENVELOPE_PATH = _REPO_ROOT / "src" / "alohamini_cartesian" / "calibration" / "envelope.py"
_SHIPPED_ACCEPTED = _REPO_ROOT / "configs" / "calib-envelope-accepted.yaml"
_FIXTURES = Path(__file__).resolve().parent / "fixtures" / "calibration"
_GEN_A = _FIXTURES / "AlohaMiniRobot.20261022.mac-snapshot.json"
_GEN_B = _FIXTURES / "AlohaMiniRobot.20261110.pi-live.json"

# 两份真机 fixture 里 range 是 (0, 4095) 满圈占位的那 6 台：2 个腕 roll + 3 个轮子 + 升降轴。
# 逐台核对过：代际 A 与代际 B 的这 6 台成员完全相同（只有 range 与宽度判定会变），
# 因此不需要按代际拆名，合成表 ``_plausible_motors()`` 也照同一批名字构造。
_PLACEHOLDER_MOTORS = frozenset(
    {
        "arm_left_wrist_roll",
        "arm_right_wrist_roll",
        "base_back_wheel",
        "base_left_wheel",
        "base_right_wheel",
        "lift_axis",
    }
)
# 代际 A（tests/fixtures/calibration/AlohaMiniRobot.20261022.mac-snapshot.json）里按宽度区间判
# OK 的那 11 台：18 台减去 1 台不合格与 6 台占位。本代不合格的是左夹爪
# ``arm_left_gripper`` 2021..2072 = 51 tick ≈ 4.5°（夹爪区间 120–135° 之外）；
# 本代的 ``arm_right_shoulder_lift`` 939..3258 = 2319 tick ≈ 203.8° 合格。
_GEN_A_OK_MOTORS = frozenset(
    {
        "arm_left_elbow_flex",
        "arm_left_shoulder_lift",
        "arm_left_shoulder_pan",
        "arm_left_wrist_flex",
        "arm_left_wrist_yaw",
        "arm_right_elbow_flex",
        "arm_right_gripper",
        "arm_right_shoulder_pan",
        "arm_right_shoulder_lift",
        "arm_right_wrist_flex",
        "arm_right_wrist_yaw",
    }
)
# 代际 B（tests/fixtures/calibration/AlohaMiniRobot.20261110.pi-live.json）里判 OK 的那 11 台：
# 18 台减去 1 台不合格与 6 台占位。本代不合格的是右肩 lift
# ``arm_right_shoulder_lift`` 905..2048 = 1143 tick ≈ 100.5°（肩区间 190–240° 之外）；
# 本代的左夹爪 ``arm_left_gripper`` 2018..3483 = 1465 tick ≈ 128.8° 落在夹爪区间 120–135° 内，
# 且中心离归零位 2047 只偏 +61.8°（< 90°），因此合格。
# 两代的差别正是「左夹爪」与「右肩 lift」这一对互换，故必须分成两个集合，不能共用一个常量。
_GEN_B_OK_MOTORS = frozenset(
    {
        "arm_left_elbow_flex",
        "arm_left_gripper",
        "arm_left_shoulder_lift",
        "arm_left_shoulder_pan",
        "arm_left_wrist_flex",
        "arm_left_wrist_yaw",
        "arm_right_elbow_flex",
        "arm_right_gripper",
        "arm_right_shoulder_pan",
        "arm_right_wrist_flex",
        "arm_right_wrist_yaw",
    }
)

# 合成表的各族 range：宽度都落在经验区间内，中心都压归零位 2047（偏离 0）。
_PLAUSIBLE_RANGES: dict[str, tuple[int, int]] = {
    "shoulder_pan": (852, 3242),  # 2390 tick ≈ 210.0°
    "shoulder_lift": (852, 3242),
    "elbow_flex": (900, 3100),  # 不属于任何宽度族：不判宽度
    "wrist_flex": (1080, 3014),  # 1934 tick ≈ 170.0°
    "wrist_yaw": (1080, 3014),
    "wrist_roll": (0, 4095),  # 满圈占位
    "gripper": (1319, 2775),  # 1456 tick ≈ 128.0°
}


def _load_envelope() -> ModuleType:
    """确认实现文件存在后，以包内模块身份导入 ``envelope``。

    缺文件时这条断言先炸（红阶段就是要响），不走 ``skip``。
    """
    assert _ENVELOPE_PATH.is_file(), (
        f"缺少实现文件：{_ENVELOPE_PATH}"
        "（红阶段预期：envelope 模块尚未实现；缺文件要响亮失败，不跳过）"
    )
    return importlib.import_module("alohamini_cartesian.calibration.envelope")


def _fixture_motors(path: Path) -> dict[str, Any]:
    """读一份真机 fixture，取出「电机名 → 五字段」映射（每次现读，用例间不共享可变状态）。"""
    return identity.extract_motors(json.loads(path.read_text(encoding="utf-8")))


def _plausible_motors() -> dict[str, Any]:
    """合成一份 18 台的电机表：手臂宽度全部落在经验区间内，底盘与升降轴是满圈占位。"""
    motors: dict[str, Any] = {}
    for side in ("left", "right"):
        for motor_id, (suffix, (low, high)) in enumerate(_PLAUSIBLE_RANGES.items(), start=1):
            motors[f"arm_{side}_{suffix}"] = {
                "id": motor_id,
                "drive_mode": 0,
                "homing_offset": 0,
                "range_min": low,
                "range_max": high,
            }
    ground_plane = (
        ("base_left_wheel", 8),
        ("base_back_wheel", 9),
        ("base_right_wheel", 10),
        ("lift_axis", 11),
    )
    for name, motor_id in ground_plane:
        motors[name] = {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
    return motors


def _teleop_only_motors() -> dict[str, Any]:
    """合成遥操式的「只有手臂」电机表：14 台 ``arm_*``，没有任何 ``base_*`` 与 ``lift_axis``。"""
    return {name: entry for name, entry in _plausible_motors().items() if name.startswith("arm_")}


def _gripper_51_tick_motors() -> dict[str, Any]:
    """合成表 + 左夹爪换成 2021..2072（51 tick ≈ 4.5°，已知的左夹爪异常形态）。"""
    motors = _plausible_motors()
    motors["arm_left_gripper"]["range_min"] = 2021
    motors["arm_left_gripper"]["range_max"] = 2072
    return motors


def _verdict_map(report: Any) -> dict[str, str]:
    """把报告收成「电机名 → verdict 值」映射，便于逐台对照。"""
    return {item.motor: item.verdict.value for item in report.motors}


def _detail_of(report: Any, motor: str) -> str:
    """取某台电机的 ``detail`` 文本。"""
    for item in report.motors:
        if item.motor == motor:
            return item.detail
    raise AssertionError(f"报告里没有电机 {motor}；实际有：{sorted(_verdict_map(report))}")


def _report_text(report: Any) -> str:
    """把报告里所有可读文本拼起来（汇总层的字符串字段 + 每台电机的 detail）。

    汇总层的文案字段没有被逐个冻结（``EnvelopeReport`` 只冻了 ``verdict`` / ``motors``），
    所以「遥操臂」这类文案只在「报告里某处出现」这一层断言，不钉死具体挂在哪个字段。
    """
    parts: list[str] = []
    if dataclasses.is_dataclass(report):
        for field in dataclasses.fields(report):
            value = getattr(report, field.name)
            if isinstance(value, str):
                parts.append(value)
    parts.extend(item.detail for item in report.motors)
    return " ".join(parts)


def _is_rejected(envelope: ModuleType, report: Any) -> bool:
    """``WRONG_DATASET`` 的便捷判定：模块函数 ``rejected()`` 或报告上的同名属性都接受。"""
    function = getattr(envelope, "rejected", None)
    if callable(function):
        return bool(function(report))
    return bool(report.rejected)


def _write_accepted(path: Path, entries: list[dict[str, Any]]) -> Path:
    """把接受名单写成 ``load_accepted`` 吃的 YAML 文件。"""
    path.write_text(
        yaml.safe_dump(
            {"schema_version": 1, "accepted": entries},
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return path


def _accepted_entry(motors: dict[str, Any], **overrides: Any) -> dict[str, Any]:
    """一条「左夹爪 51 tick 已接受」的名单条目，字段全给，可逐字段覆盖。"""
    entry: dict[str, Any] = {
        "motor": "arm_left_gripper",
        "observed_range_min": 2021,
        "observed_range_max": 2072,
        "measured_on_tick": "2026-10-22",
        "method": "manual-teach-pendant",
        "confirmed_by": "pcoffeep",
        "confirmed_at_utc": "2026-11-11T00:00:00Z",
        "calib_id": identity.calib_id(motors),
        "reason": "左夹爪实物行程只有 51 tick，经验区间不适用于夹爪机械限位",
    }
    entry.update(overrides)
    return entry


# ---------------------------------------------------------------- 契约常量与枚举形态


def test_module_exposes_the_frozen_threshold_constants() -> None:
    """阈值常量的名字与值都冻结：夹爪 120–135°、肩 190–240°、腕 160–190°、占位、归零位、90°。"""
    envelope = _load_envelope()

    assert envelope.GRIPPER_WIDTH_DEG_RANGE == (120.0, 135.0)
    assert envelope.SHOULDER_WIDTH_DEG_RANGE == (190.0, 240.0)
    assert envelope.WRIST_WIDTH_DEG_RANGE == (160.0, 190.0)
    assert envelope.PLACEHOLDER_FULL_RANGE == (0, 4095)
    assert envelope.ZERO_POSITION_TICK == 2047
    assert envelope.ZERO_POINT_OFFSET_WARN_DEG == 90.0


def test_verdict_enum_member_names_and_values_are_frozen() -> None:
    """``EnvelopeVerdict`` 四个成员的名字与值都固定（``unmeasured_range`` 是小写那个）。"""
    envelope = _load_envelope()
    verdict = envelope.EnvelopeVerdict

    assert verdict.OK.value == "OK"
    assert verdict.UNMEASURED_RANGE.value == "unmeasured_range"
    assert verdict.IMPLAUSIBLE.value == "IMPLAUSIBLE"
    assert verdict.WRONG_DATASET.value == "WRONG_DATASET"
    assert {member.name for member in verdict} == {
        "OK",
        "UNMEASURED_RANGE",
        "IMPLAUSIBLE",
        "WRONG_DATASET",
    }


# ---------------------------------------------------------------- 两份真机 fixture


def test_generation_a_flags_the_left_gripper_with_a_non_empty_detail() -> None:
    """代际 A（2026-10-22 快照）：左夹爪 51 tick → ``IMPLAUSIBLE``，``detail`` 写明实测与区间。"""
    envelope = _load_envelope()

    report = envelope.classify_motors(_fixture_motors(_GEN_A))

    assert _verdict_map(report)["arm_left_gripper"] == "IMPLAUSIBLE"
    detail = _detail_of(report, "arm_left_gripper")
    assert detail.strip() != ""
    # 契约要求 detail 写明：实测度数、该族区间、以及「经验值非厂商规定」的性质。
    assert "°" in detail
    assert "120" in detail and "135" in detail
    assert "经验" in detail


def test_generation_a_splits_the_rest_into_six_unmeasured_and_eleven_ok() -> None:
    """代际 A：6 台满圈占位 ``unmeasured_range``、11 台宽度合格 ``OK``，汇总 ``IMPLAUSIBLE``。

    唯一不合格的是左夹爪 ``arm_left_gripper``（51 tick ≈ 4.5°），不是右肩 lift；
    逐名精确比较 6 台占位与 11 台 OK 两个集合，不用数量或子集代替。
    """
    envelope = _load_envelope()

    report = envelope.classify_motors(_fixture_motors(_GEN_A))

    verdicts = _verdict_map(report)
    assert len(verdicts) == 18
    assert {name for name, value in verdicts.items() if value == "unmeasured_range"} == set(
        _PLACEHOLDER_MOTORS
    )
    assert {name for name, value in verdicts.items() if value == "OK"} == set(_GEN_A_OK_MOTORS)
    assert report.verdict.value == "IMPLAUSIBLE"
    assert [item.motor for item in report.motors] == sorted(verdicts)


def test_generation_b_flags_the_right_shoulder_lift() -> None:
    """代际 B（2026-11-10 现场）：右肩 lift 只有 1143 tick ≈ 100.4° → ``IMPLAUSIBLE``。"""
    envelope = _load_envelope()

    report = envelope.classify_motors(_fixture_motors(_GEN_B))

    assert _verdict_map(report)["arm_right_shoulder_lift"] == "IMPLAUSIBLE"
    detail = _detail_of(report, "arm_right_shoulder_lift")
    assert detail.strip() != ""
    assert "190" in detail and "240" in detail
    assert "经验" in detail


def test_generation_b_splits_the_rest_into_six_unmeasured_and_eleven_ok() -> None:
    """代际 B 与 A 同形（6 台占位、11 台 ``OK``、汇总 ``IMPLAUSIBLE``），但 11 台 OK 的名单不同。

    本代唯一不合格的是右肩 lift ``arm_right_shoulder_lift``（1143 tick ≈ 100.5°），**不是**左夹爪；
    左夹爪 1465 tick ≈ 128.8° 在本代合格，右肩 lift 在代际 A 合格——两代名单正好互换这两台。
    """
    envelope = _load_envelope()

    report = envelope.classify_motors(_fixture_motors(_GEN_B))

    verdicts = _verdict_map(report)
    assert {name for name, value in verdicts.items() if value == "unmeasured_range"} == set(
        _PLACEHOLDER_MOTORS
    )
    assert {name for name, value in verdicts.items() if value == "OK"} == set(_GEN_B_OK_MOTORS)
    assert {name for name, value in verdicts.items() if value == "IMPLAUSIBLE"} == {
        "arm_right_shoulder_lift"
    }
    assert report.verdict.value == "IMPLAUSIBLE"


# ---------------------------------------------------------------- 构造用例


def test_a_51_tick_left_gripper_is_implausible() -> None:
    """构造左夹爪 51 tick（2021..2072，约 4.5°）→ ``IMPLAUSIBLE``，不是占位也不是 OK。"""
    envelope = _load_envelope()

    report = envelope.classify_motors(_gripper_51_tick_motors())

    assert _verdict_map(report)["arm_left_gripper"] == "IMPLAUSIBLE"
    assert report.verdict.value == "IMPLAUSIBLE"
    assert len(report.implausible) == 1
    assert report.implausible[0].motor == "arm_left_gripper"


def test_placeholder_full_range_does_not_count_as_a_failure() -> None:
    """满圈占位只进 ``unmeasured``：6 台占位被放行，汇总仍是 ``OK``、不合格名单为空。"""
    envelope = _load_envelope()

    report = envelope.classify_motors(_plausible_motors())

    assert {item.motor for item in report.unmeasured} == set(_PLACEHOLDER_MOTORS)
    assert report.implausible == ()
    assert report.verdict.value == "OK"
    assert _verdict_map(report)["base_left_wheel"] == "unmeasured_range"


def test_an_arm_only_table_is_the_wrong_dataset() -> None:
    """只有 14 台手臂关节、没有 ``base_*`` 也没有 ``lift_axis`` → ``WRONG_DATASET``（遥操臂）。"""
    envelope = _load_envelope()

    report = envelope.classify_motors(_teleop_only_motors())

    assert report.verdict.value == "WRONG_DATASET"
    assert _is_rejected(envelope, report) is True
    assert "遥操臂" in _report_text(report)


def test_a_full_robot_table_is_not_rejected() -> None:
    """18 台齐（含底盘与升降轴）时汇总不判 ``WRONG_DATASET``：提醒型结论不该被当成不可继续。"""
    envelope = _load_envelope()

    report = envelope.classify_motors(_fixture_motors(_GEN_A))

    assert _is_rejected(envelope, report) is False


def test_identity_fields_do_not_change_any_verdict_or_detail() -> None:
    """``id`` / ``drive_mode`` / ``homing_offset`` 全改：verdict 与 ``detail`` 逐台不变。"""
    envelope = _load_envelope()
    baseline = envelope.classify_motors(_plausible_motors())
    mutated_motors = copy.deepcopy(_plausible_motors())
    mutated_motors["arm_left_gripper"]["homing_offset"] = -3000
    mutated_motors["arm_left_wrist_yaw"]["id"] = 99
    mutated_motors["arm_right_shoulder_pan"]["drive_mode"] = 1

    mutated = envelope.classify_motors(mutated_motors)

    assert [(item.motor, item.verdict, item.detail) for item in mutated.motors] == [
        (item.motor, item.verdict, item.detail) for item in baseline.motors
    ]
    assert mutated.verdict is baseline.verdict


def test_a_range_center_far_from_zero_position_is_implausible_even_when_width_fits() -> None:
    """宽度 1456 tick ≈ 128° 落在夹爪区间内，但中心偏离归零位 95° > 90° → 仍 ``IMPLAUSIBLE``。"""
    envelope = _load_envelope()
    motors = _plausible_motors()
    motors["arm_left_gripper"]["range_min"] = 2400
    motors["arm_left_gripper"]["range_max"] = 3856

    report = envelope.classify_motors(motors)

    assert _verdict_map(report)["arm_left_gripper"] == "IMPLAUSIBLE"
    detail = _detail_of(report, "arm_left_gripper")
    assert detail.strip() != ""
    # 措辞自由，但必须点出这是中心/归零位的问题，而不是宽度的问题。
    assert any(word in detail for word in ("中心", "偏离", "2047"))


def test_a_non_integer_range_is_skipped_without_any_verdict() -> None:
    """Range 不是整数时不打 verdict：结构校验的活，envelope 跳过该电机、汇总不受影响。"""
    envelope = _load_envelope()
    motors = _plausible_motors()
    motors["arm_left_gripper"]["range_min"] = "not-a-tick"

    report = envelope.classify_motors(motors)

    assert "arm_left_gripper" not in _verdict_map(report)
    assert len(report.motors) == 17
    assert report.verdict.value == "OK"


# ---------------------------------------------------------------- 接受名单


def test_load_accepted_reads_an_empty_template(tmp_path: Path) -> None:
    """空模板 ``accepted: []`` 读得通：条目数为 0，不抛异常。"""
    envelope = _load_envelope()
    path = _write_accepted(tmp_path / "accepted.yaml", [])

    accepted = envelope.load_accepted(path)

    assert _accepted_entries(accepted) == ()


@pytest.mark.parametrize("case", ["missing-field", "accepted-not-a-list", "broken-yaml"])
def test_load_accepted_rejects_a_broken_file(tmp_path: Path, case: str) -> None:
    """缺字段 / ``accepted`` 不是列表 / YAML 不可解析 → 抛 ``AcceptedListError``。

    该异常是 ``ValueError`` 的子类：坏名单是可预期的输入错误，不是内部故障。
    """
    envelope = _load_envelope()
    path = tmp_path / "accepted.yaml"
    if case == "missing-field":
        entry = _accepted_entry(_plausible_motors())
        del entry["method"]
        path = _write_accepted(path, [entry])
    elif case == "accepted-not-a-list":
        path.write_text("schema_version: 1\naccepted: 大字符串\n", encoding="utf-8")
    else:
        path.write_text("schema_version: 1\naccepted: [\n", encoding="utf-8")

    with pytest.raises(envelope.AcceptedListError) as excinfo:
        envelope.load_accepted(path)

    assert issubclass(envelope.AcceptedListError, ValueError)
    assert str(excinfo.value).strip() != ""


def _accepted_entries(accepted: Any) -> tuple[Any, ...]:
    """取接受名单里的条目序列。

    容器 dataclass 的字段名没被冻结（只冻了条目字段），所以按常见名字试探，最后退回可迭代。
    """
    for attribute in ("entries", "accepted", "items", "records"):
        value = getattr(accepted, attribute, None)
        if value is not None:
            return tuple(value)
    return tuple(accepted)


def test_an_accepted_entry_turns_the_gripper_into_ok_with_an_information_line(
    tmp_path: Path,
) -> None:
    """名单命中（motor + range + calib_id 三者一致）→ 该电机 ``OK``，并带「已接受」信息行。"""
    envelope = _load_envelope()
    motors = _gripper_51_tick_motors()
    accepted = envelope.load_accepted(
        _write_accepted(tmp_path / "accepted.yaml", [_accepted_entry(motors)])
    )

    report = envelope.classify_motors(motors, accepted=accepted)

    assert _verdict_map(report)["arm_left_gripper"] == "OK"
    assert report.verdict.value == "OK"
    detail = _detail_of(report, "arm_left_gripper")
    assert "已接受" in detail
    text = _report_text(report)
    assert "manual-teach-pendant" in text
    assert "pcoffeep" in text
    assert "2026-11-11T00:00:00Z" in text


def test_a_stale_accepted_entry_with_a_different_range_still_warns(tmp_path: Path) -> None:
    """名单登记 range 与实测不符 → 未命中，照旧 ``IMPLAUSIBLE``，``detail`` 含「名单过期」。"""
    envelope = _load_envelope()
    motors = _gripper_51_tick_motors()
    stale = _accepted_entry(motors, observed_range_max=2073)
    accepted = envelope.load_accepted(_write_accepted(tmp_path / "accepted.yaml", [stale]))

    report = envelope.classify_motors(motors, accepted=accepted)

    assert _verdict_map(report)["arm_left_gripper"] == "IMPLAUSIBLE"
    assert "名单过期" in _detail_of(report, "arm_left_gripper")
    assert report.verdict.value == "IMPLAUSIBLE"


def test_a_stale_accepted_entry_with_a_different_calib_id_still_warns(tmp_path: Path) -> None:
    """名单登记 ``calib_id`` 与本次不同 → 未命中，照旧 ``IMPLAUSIBLE`` 且含「名单过期」。"""
    envelope = _load_envelope()
    motors = _gripper_51_tick_motors()
    stale = _accepted_entry(motors, calib_id="000000000000")
    accepted = envelope.load_accepted(_write_accepted(tmp_path / "accepted.yaml", [stale]))

    report = envelope.classify_motors(motors, accepted=accepted)

    assert _verdict_map(report)["arm_left_gripper"] == "IMPLAUSIBLE"
    assert "名单过期" in _detail_of(report, "arm_left_gripper")


# ---------------------------------------------------------------- identity 侧薄封装


def test_describe_plausibility_reflects_the_envelope_constants(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``identity.describe_plausibility`` 是薄封装：改 ``envelope`` 常量就改输出。

    返回类型仍是 ``tuple[str, ...]``；阈值说明由 ``envelope`` 渲染，identity 不再自己带数字。
    """
    envelope = _load_envelope()
    monkeypatch.setattr(envelope, "GRIPPER_WIDTH_DEG_RANGE", (101.0, 202.0))

    lines = identity.describe_plausibility()

    assert isinstance(lines, tuple)
    assert lines
    assert all(isinstance(line, str) for line in lines)
    text = " ".join(lines)
    assert "101" in text and "202" in text


def test_validate_plausibility_delegates_to_plausibility_warnings() -> None:
    """``identity.validate_plausibility`` 委托 ``envelope.plausibility_warnings``。

    内容一致、返回 ``list[str]``、永不抛异常（只报告不阻断）。
    """
    envelope = _load_envelope()
    motors = _fixture_motors(_GEN_A)

    warnings = identity.validate_plausibility(motors)

    assert isinstance(warnings, list)
    assert all(isinstance(item, str) for item in warnings)
    assert warnings == envelope.plausibility_warnings(motors)
    assert any("arm_left_gripper" in item for item in warnings)


def test_validate_plausibility_never_raises_on_an_arm_only_table() -> None:
    """遥操臂形态（只有 14 台手臂）也不抛异常：合理性校验只告警、不阻断。"""
    envelope = _load_envelope()
    motors = _teleop_only_motors()

    warnings = identity.validate_plausibility(motors)

    assert isinstance(warnings, list)
    assert warnings == envelope.plausibility_warnings(motors)
