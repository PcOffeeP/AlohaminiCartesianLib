"""``alohamini_cartesian.calibration.identity`` 的离线契约测试（TDD 红阶段）。

被测模块尚未实现，本文件按任务书冻结的契约写断言，运行必然失败（``ModuleNotFoundError``）；
这是预期，不得为了让测试变绿而放宽断言或加软着陆。

锁的六件事：

- **代际身份稳定**：``calib_id`` 只由 5 个标定字段决定，与电机顺序、字段顺序、输入形状、
  指纹里多出的 ``model`` / ``normalization`` / ``resolution`` 无关；任一字段动一位就换 id。
- **两种输入形状同 id**（最关键）：同一份数据分别以「标定文件形状」与「指纹形状」喂
  :func:`extract_motors` + :func:`calib_id`，两个 id 必须相等。
- **真实数据**：两份 fixture 是真实标定原件的逐字节副本，先硬断言 sha256 与任务书给的
  字节指纹逐字相同（防 fixture 被无意改坏），再实算两个**不同**的 id（不硬编码 id 字面值）。
- **结构校验只报不抛**：:func:`validate_structure` 返回问题清单，空列表才算通过。
- **规范形**：:func:`canonicalize_motors` 只留 5 个标定字段、值转 ``int``、缺字段或 ``None``
  抛 :class:`CalibrationStructureError`。
- **变化分类**：``UNCHANGED`` / ``ARM_ONLY`` / ``BROAD`` / ``REGION_INCONSISTENT`` 四态，
  外加逐字段的 ``delta_tick`` / ``delta_deg`` 数值。

单位口径：``delta_deg`` 按任务书给的例子锁 1 tick = ``0.087890625`` 度（360 / 4096）。

全部用例离线：只读 ``tests/fixtures/calibration/`` 下两份原件副本，不连设备、不联网。
"""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import pytest

from alohamini_cartesian.calibration.identity import (
    CANONICAL_FIELDS,
    EXPECTED_MOTOR_COUNT,
    CalibrationStructureError,
    ChangePattern,
    calib_id,
    canonical_json,
    canonicalize_motors,
    classify_change,
    extract_motors,
    file_sha256,
    validate_plausibility,
    validate_structure,
)

_FIXTURES = Path(__file__).resolve().parent / "fixtures" / "calibration"
OLD_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261022.mac-snapshot.json"
NEW_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261110.pi-live.json"

# 任务书给的字节指纹：两份 fixture 必须逐字对上，改一位就红。
OLD_SHA256 = "6d089d3879b538a806dd5008dba66cde1bdb5a819aa7c4987becd0bf6b6e4c56"
NEW_SHA256 = "d6594b999eb8533932adf20b6b5be413768c521f9f082109454cd323e183dfcf"
OLD_BYTE_SIZE = 2880
NEW_BYTE_SIZE = 2882

# 1 tick 对应的角度：360 度 / 4096 tick（任务书例子 0.087890625）。
_DEG_PER_TICK = 0.087890625


# 旧→新代际的实测事实：14 台手臂电机动，底盘 3 个轮子与 lift_axis 未动。
_GROUND_PLANE_MOTORS = frozenset(
    {"base_left_wheel", "base_back_wheel", "base_right_wheel", "lift_axis"}
)
_ARM_MOTOR_COUNT = 14


def _read_json(path: Path) -> dict[str, Any]:
    """读一份 JSON 原件，返回解析后的顶层对象。"""
    return json.loads(path.read_text(encoding="utf-8"))


def _old_motors() -> dict[str, Any]:
    """旧代际那 18 台电机的标定字段（每次现读，用例之间不共享可变状态）。"""
    return extract_motors(_read_json(OLD_FIXTURE))


def _copy_of_old() -> dict[str, Any]:
    """旧代际标定字段的一份深拷贝（可自由破坏，不碰 fixture 与其它用例）。"""
    return copy.deepcopy(_old_motors())


def _set_fields(motors: dict[str, Any], motor: str, **fields: Any) -> None:
    """就地改写某台电机的若干字段，例如 ``_set_fields(motors, "lift_axis", id=11)``。"""
    motors[motor].update(fields)


def _fingerprint_document(motors: dict[str, Any]) -> dict[str, Any]:
    """把 5 字段的标定电机映射包成指纹形状（多带 ``model``/``normalization``/``resolution``）。"""
    return {
        "captured_utc": "2026-10-22T00:00:00Z",
        "robot_metadata": {
            "schema_version": 2,
            "robot_model": "aloha_mini_2_pro",
            "motors": {
                name: {
                    **entry,
                    "model": "sts3250",
                    "normalization": "range_m100_100",
                    "resolution": 4096,
                }
                for name, entry in motors.items()
            },
        },
        "observation_state": {"schema_version": 1},
    }


def _real_generations() -> tuple[dict[str, Any], dict[str, Any]]:
    """两份真实 fixture 的 5 字段标定电机映射，返回 ``(旧, 新)``。"""
    return (
        extract_motors(_read_json(OLD_FIXTURE)),
        extract_motors(_read_json(NEW_FIXTURE)),
    )


def test_calib_id_is_unchanged_when_order_is_shuffled() -> None:
    """同一份 motors 打乱电机顺序与字段顺序后，``calib_id`` 与规范 JSON 都不变。"""
    motors = _old_motors()
    shuffled = {
        name: dict(reversed(list(entry.items()))) for name, entry in reversed(list(motors.items()))
    }

    assert calib_id(shuffled) == calib_id(motors)
    assert canonical_json(shuffled) == canonical_json(motors)
    # 规范形自身的输出顺序也要按字典序，而不是只靠 ``json.dumps(sort_keys=True)`` 蒙对。
    assert list(canonicalize_motors(motors)) == sorted(motors)


def test_canonical_fields_are_frozen() -> None:
    """5 个标定字段的名字与顺序是冻结契约（改字段集就换了每一代的身份）。"""
    assert CANONICAL_FIELDS == ("id", "drive_mode", "homing_offset", "range_min", "range_max")


@pytest.mark.parametrize("field", CANONICAL_FIELDS)
def test_calib_id_changes_when_one_canonical_field_moves_by_one(field: str) -> None:
    """5 个标定字段各自 +1，``calib_id`` 都必须与原件不同。"""
    base = _old_motors()
    shifted = _copy_of_old()
    _set_fields(shifted, "arm_left_elbow_flex", **{field: base["arm_left_elbow_flex"][field] + 1})

    assert calib_id(shifted) != calib_id(base)


def test_calibration_file_and_fingerprint_shapes_yield_the_same_calib_id() -> None:
    """同一份标定在「标定文件形状」与「指纹形状」下必须算出同一个 id。

    指纹形状多带 ``model`` / ``normalization`` / ``resolution`` 三个字段，它们不是标定身份
    的组成部分；两个 id 不等就是身份口径出错（失败信息打印两个 id）。
    """
    motors = _old_motors()
    from_calibration = calib_id(extract_motors(_read_json(OLD_FIXTURE)))
    from_fingerprint = calib_id(extract_motors(_fingerprint_document(motors)))

    assert from_calibration == from_fingerprint, (
        "同一份标定在两种输入形状下算出不同 calib_id："
        f"标定文件形状={from_calibration} 指纹形状={from_fingerprint}"
    )


def test_extract_motors_accepts_the_short_fingerprint_shape() -> None:
    """顶层 ``{"motors": {...}}`` 的指纹简写形状也要认。"""
    motors = _old_motors()

    assert extract_motors({"motors": motors}) == motors


def test_extract_motors_rejects_an_unrecognised_document() -> None:
    """认不出的顶层形状必须抛 :class:`CalibrationStructureError`，不静默返回空表。"""
    with pytest.raises(CalibrationStructureError):
        extract_motors({"captured_utc": "2026-10-22T00:00:00Z"})


def test_the_two_real_generations_have_different_calib_ids() -> None:
    """两份真实 fixture 算出两个不同的 id（id 值由实算得出，不硬编码）。"""
    old_id = calib_id(extract_motors(_read_json(OLD_FIXTURE)))
    new_id = calib_id(extract_motors(_read_json(NEW_FIXTURE)))
    print(f"OLD_CALIB_ID={old_id} NEW_CALIB_ID={new_id}")

    assert old_id != new_id, f"两个代际算出同一个 calib_id：{old_id}"


@pytest.mark.parametrize(
    ("path", "expected_sha256", "expected_byte_size"),
    [
        (OLD_FIXTURE, OLD_SHA256, OLD_BYTE_SIZE),
        (NEW_FIXTURE, NEW_SHA256, NEW_BYTE_SIZE),
    ],
    ids=["old-generation", "new-generation"],
)
def test_the_fixture_bytes_match_the_recorded_sha256(
    path: Path, expected_sha256: str, expected_byte_size: int
) -> None:
    """两份 fixture 是原件逐字节副本：sha256 与字节数都要对上任务书给的值。"""
    assert file_sha256(path) == expected_sha256
    assert path.stat().st_size == expected_byte_size


def test_validate_structure_accepts_a_real_generation() -> None:
    """真实标定原件的结构合法：问题清单必须是空列表（空列表 = 通过）。"""
    assert validate_structure(_old_motors()) == []


def test_validate_structure_flags_an_inverted_range() -> None:
    """``range_min >= range_max`` 必须被报出来，并点名那台电机。"""
    motors = _copy_of_old()
    _set_fields(
        motors,
        "arm_left_elbow_flex",
        range_min=motors["arm_left_elbow_flex"]["range_max"],
    )

    problems = validate_structure(motors)

    assert problems
    assert any("arm_left_elbow_flex" in message for message in problems)


def test_validate_structure_flags_a_missing_required_field() -> None:
    """缺必需字段必须被报出来，并同时点名电机与字段。"""
    motors = _copy_of_old()
    del motors["arm_left_wrist_flex"]["homing_offset"]

    problems = validate_structure(motors)

    assert problems
    assert any(
        "arm_left_wrist_flex" in message and "homing_offset" in message for message in problems
    )


def test_validate_structure_flags_a_none_value() -> None:
    """字段在场但值为 ``None`` 必须被报出来（``None`` 不等于 0，也不是合法标定）。"""
    motors = _copy_of_old()
    motors["arm_left_gripper"]["range_min"] = None

    problems = validate_structure(motors)

    assert problems
    assert any("arm_left_gripper" in message for message in problems)


def test_validate_structure_flags_a_wrong_motor_count() -> None:
    """电机数量不等于 18 必须被报出来，且消息里带期望台数。"""
    motors = _copy_of_old()
    del motors["lift_axis"]

    problems = validate_structure(motors)

    assert problems
    assert str(EXPECTED_MOTOR_COUNT) in "".join(problems)


def test_validate_structure_flags_an_illegal_drive_mode() -> None:
    """``drive_mode`` 只允许 0 或 1，其它值必须被报出来并点名电机。"""
    motors = _copy_of_old()
    _set_fields(motors, "arm_left_shoulder_pan", drive_mode=7)

    problems = validate_structure(motors)

    assert problems
    assert any("arm_left_shoulder_pan" in message for message in problems)


def test_validate_structure_flags_a_non_integer_field() -> None:
    """字段值不是整数时也要报出来（``11.5`` 不是 tick 数）。"""
    motors = _copy_of_old()
    _set_fields(motors, "lift_axis", id=11.5)

    problems = validate_structure(motors)

    assert problems
    assert any("lift_axis" in message for message in problems)


@pytest.mark.parametrize(
    ("field", "value"),
    [("drive_mode", True), ("homing_offset", True), ("drive_mode", "0"), ("homing_offset", "0")],
)
def test_a_bool_or_a_string_is_never_a_tick_value(field: str, value: Any) -> None:
    """``True``（``bool`` 是 ``int`` 子类）与 ``"0"`` 都不是刻度数：结构要报、身份要拒。"""
    motors = _copy_of_old()
    _set_fields(motors, "arm_left_elbow_flex", **{field: value})

    problems = validate_structure(motors)

    assert problems, f"validate_structure 没报出 {field}={value!r}"
    assert any("arm_left_elbow_flex" in message for message in problems)
    with pytest.raises(CalibrationStructureError):
        calib_id(motors)


def test_validate_plausibility_reports_without_blocking() -> None:
    """合理性校验是经验区间告警：真实原件返回 ``list[str]``，且不抛异常、不阻断。"""
    warnings = validate_plausibility(_old_motors())

    assert isinstance(warnings, list)
    assert all(isinstance(item, str) for item in warnings)


def test_canonicalize_motors_keeps_only_the_five_canonical_fields() -> None:
    """规范形只留 5 个标定字段、值一律 ``int``，指纹独有的三字段被丢弃。"""
    fingerprint_motors = _fingerprint_document(_old_motors())["robot_metadata"]["motors"]

    canonical = canonicalize_motors(fingerprint_motors)

    assert set(canonical) == set(fingerprint_motors)
    for entry in canonical.values():
        assert set(entry) == set(CANONICAL_FIELDS)
        assert all(isinstance(value, int) for value in entry.values())


def test_canonicalize_motors_rejects_a_missing_required_field() -> None:
    """缺必需字段的 motors 进规范形要抛 :class:`CalibrationStructureError`。"""
    motors = _copy_of_old()
    del motors["arm_left_gripper"]["drive_mode"]

    with pytest.raises(CalibrationStructureError):
        canonicalize_motors(motors)


def test_classify_change_reports_unchanged_for_identical_generations() -> None:
    """同一份 motors 与自己的副本比：``UNCHANGED``，无差异、无变动电机。"""
    motors = _old_motors()

    report = classify_change(motors, copy.deepcopy(motors))

    assert report.pattern is ChangePattern.UNCHANGED
    assert report.diffs == ()
    assert report.changed_motors == ()


# 旧→新代际的实测事实见 _GROUND_PLANE_MOTORS / _ARM_MOTOR_COUNT。
def test_the_real_regeneration_is_arm_only() -> None:
    """真实的 2026-10-22 → 2026-11-10：14 台手臂电机动、底盘与升降轴未动 → ``ARM_ONLY``。"""
    old, new = _real_generations()

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.ARM_ONLY
    assert len(report.changed_motors) == _ARM_MOTOR_COUNT
    assert _GROUND_PLANE_MOTORS & set(report.changed_motors) == set()
    assert all(motor.startswith("arm_") for motor in report.changed_motors)


def test_the_real_regeneration_is_arm_only_in_reverse_too() -> None:
    """反向比（新 → 旧）同属手臂局部变更：``ARM_ONLY``（变化形态不取决于比较方向）。"""
    old, new = _real_generations()

    report = classify_change(new, old)

    assert report.pattern is ChangePattern.ARM_ONLY
    assert len(report.changed_motors) == _ARM_MOTOR_COUNT
    assert _GROUND_PLANE_MOTORS & set(report.changed_motors) == set()


def test_classify_change_reports_arm_only_when_only_arm_fields_moved() -> None:
    """只动两条手臂的字段（底盘与 lift_axis 不动）→ ``ARM_ONLY``。"""
    old = _old_motors()
    new = _copy_of_old()
    _set_fields(
        new,
        "arm_left_shoulder_pan",
        range_min=old["arm_left_shoulder_pan"]["range_min"] + 1,
    )
    _set_fields(
        new,
        "arm_right_gripper",
        homing_offset=old["arm_right_gripper"]["homing_offset"] - 3,
    )

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.ARM_ONLY
    assert set(report.changed_motors) == {"arm_left_shoulder_pan", "arm_right_gripper"}
    assert len(report.changed_motors) == 2


def test_classify_change_reports_broad_when_every_motor_moved() -> None:
    """18 台电机全部变动（每台各改一位）→ ``BROAD``。"""
    old = _old_motors()
    new = _copy_of_old()
    for entry in new.values():
        entry["range_max"] = entry["range_max"] - 1

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.BROAD
    assert len(report.changed_motors) == EXPECTED_MOTOR_COUNT


def test_classify_change_reports_broad_when_only_a_base_wheel_moved() -> None:
    """只动底盘一个轮子的一个字段 → ``BROAD``（任务书：含底盘/升降变动就算全量）。"""
    old = _old_motors()
    new = _copy_of_old()
    _set_fields(new, "base_left_wheel", range_min=5)

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.BROAD
    assert set(report.changed_motors) == {"base_left_wheel"}


def test_classify_change_reports_broad_when_only_the_lift_axis_moved() -> None:
    """只动 ``lift_axis`` 一个字段 → ``BROAD``（升降轴属底盘侧，不是手臂局部变更）。"""
    old = _old_motors()
    new = _copy_of_old()
    _set_fields(new, "lift_axis", homing_offset=17)

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.BROAD
    assert set(report.changed_motors) == {"lift_axis"}


def test_classify_change_reports_broad_when_one_arm_and_the_lift_axis_moved() -> None:
    """1 台手臂 + ``lift_axis`` 一起变 → ``BROAD``（混入手臂以外的变动就不再是手臂局部变更）。"""
    old = _old_motors()
    new = _copy_of_old()
    _set_fields(new, "arm_left_elbow_flex", range_min=old["arm_left_elbow_flex"]["range_min"] + 1)
    _set_fields(new, "lift_axis", homing_offset=17)

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.BROAD
    assert set(report.changed_motors) == {"arm_left_elbow_flex", "lift_axis"}


def test_classify_change_reports_region_inconsistent_for_a_motor_outside_the_eighteen() -> None:
    """``REGION_INCONSISTENT`` 当前的触发面：变动集合里出现 18 台之外的名字。

    按本单任务书，「含底盘/升降变动」归 ``BROAD``，所以「只动底盘」不再触发
    ``REGION_INCONSISTENT``。这一态剩下能触发它的形状是「这份文件与登记表压根不是同一套
    电机表」——例如多出 / 少掉一个键，名字不在那 18 台里。
    """
    old = _old_motors()
    new = _copy_of_old()
    new["spare_motor"] = {
        "id": 12,
        "drive_mode": 0,
        "homing_offset": 0,
        "range_min": 0,
        "range_max": 4095,
    }

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.REGION_INCONSISTENT
    assert set(report.changed_motors) == {"spare_motor"}


def test_a_one_tick_change_carries_the_tick_and_degree_delta() -> None:
    """``range_min`` 动 1 tick：``delta_tick == 1``，``delta_deg == 0.087890625``（360/4096）。"""
    old = _old_motors()
    new = _copy_of_old()
    base = old["arm_left_elbow_flex"]["range_min"]
    _set_fields(new, "arm_left_elbow_flex", range_min=base + 1)

    report = classify_change(old, new)

    diff = next(
        item
        for item in report.diffs
        if item.motor == "arm_left_elbow_flex" and item.field == "range_min"
    )
    assert len(report.diffs) == 1
    assert (diff.old, diff.new, diff.delta_tick) == (base, base + 1, 1)
    assert diff.delta_deg == pytest.approx(_DEG_PER_TICK, abs=1e-9)


def test_classify_change_reports_arm_only_for_an_extra_arm_motor_name() -> None:
    """单侧多一个 ``arm_left_extra`` 仍是 ``ARM_ONLY``。

    ``ARM_ONLY`` 只看变动电机名的前缀（``arm_``），不看键集合是否完整；电机表齐不齐由
    :func:`validate_structure` 与健康检查的 18 台校验守，不靠变化分类。
    """
    old = _old_motors()
    new = _copy_of_old()
    new["arm_left_extra"] = {
        "id": 12,
        "drive_mode": 0,
        "homing_offset": 0,
        "range_min": 100,
        "range_max": 200,
    }

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.ARM_ONLY
    assert set(report.changed_motors) == {"arm_left_extra"}


def test_classify_change_reports_arm_only_when_an_arm_motor_is_missing() -> None:
    """单侧少一台 ``arm_left_gripper`` 同样是 ``ARM_ONLY``（前缀判定，不看键集合完整度）。"""
    old = _old_motors()
    new = _copy_of_old()
    del new["arm_left_gripper"]

    report = classify_change(old, new)

    assert report.pattern is ChangePattern.ARM_ONLY
    assert set(report.changed_motors) == {"arm_left_gripper"}
