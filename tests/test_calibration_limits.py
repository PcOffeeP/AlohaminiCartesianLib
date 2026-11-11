"""``alohamini_cartesian.calibration.limits`` 的离线契约测试（TDD 红阶段）。

被测模块尚未实现，本文件按任务书冻结的契约写断言，运行必然失败（``ModuleNotFoundError``）；
这是预期，不得为了让测试变绿而放宽断言或加软着陆。

锁的六件事：

- ``STRUCTURAL_LIMIT``：目标超出 ``hard_limit`` 被拒。
- ``CALIB_RANGE_NARROWED``：目标在硬限内、超出标定 range，文案必须同时说明「是标定区间
  限制、不是结构限制」，并给出两条出路（跑实测建立层 2 / 显式覆盖）。
- ``LIMIT_CONFLICT``：``hard_limit`` 与 ``calib_range`` 交集为空，文案提示重标定。
- **无实测层不得拿标定值顶替**：``observed=None`` 时 ``hard_limit == urdf_limit``，
  ``source_of_each_bound`` 的 ``hard_min`` / ``hard_max`` 是 ``"urdf"``。
- ``allow_outside_calib_range=True`` 时放行但留痕：``override_record`` 是非 ``None`` 的
  :class:`OverrideRecord`，字段值可核对（不是静默放行）。
- ``load_observed_limits``：``None`` / 不存在的路径 / 空 ``motors`` / 缺顶层 ``motors`` 一律返回
  ``{}``（不编造、不抛）；文件存在且合 schema 时逐台读出五个字段。

实测层 YAML 的 schema 由任务书冻结为 ``schema_version`` + ``motors: {电机名: {min_tick,
max_tick, measured_on, calib_id, method}}``；本文件里的解析样本是 ``tmp_path`` 下现造的**合成
样本**，只验证解析口径，不是任何一次真机实测的结果，不得当实测证据引用。

单位口径：三层限位（URDF 结构限位 / 实测极限 / 标定 range）的数值由调用方放在同一个单位里
传入，本文件统一用「度」，因此传给 :func:`resolve_joint_limit` 的 ``observed`` 也是
``(min_deg, max_deg)`` 元组。

全部用例离线：只用常量与 ``tmp_path``，不读设备、不联网。
"""

from __future__ import annotations

from pathlib import Path

import yaml

from alohamini_cartesian.calibration.limits import (
    CALIB_RANGE_NARROWED,
    LIMIT_CONFLICT,
    STRUCTURAL_LIMIT,
    ObservedLimit,
    OverrideRecord,
    load_observed_limits,
    resolve_joint_limit,
)

_URDF = (-90.0, 90.0)
_NARROW_CALIB = (-10.0, 10.0)

_REPO_ROOT = Path(__file__).resolve().parents[1]
_OBSERVED_CONFIG = _REPO_ROOT / "configs" / "limits-observed.yaml"

# 解析用**合成样本**，不是实测值：数值只为分辨两台电机，不来自任何一次真机测量。
# 字段顺序与 ``configs/limits-observed.yaml`` 冻结的 schema 一致。
_SYNTHETIC_OBSERVED_YAML = """\
# 解析用合成样本，不是实测值
schema_version: 1
motors:
  arm_left_shoulder_pan:
    min_tick: 100
    max_tick: 300
    measured_on: "2026-11-11"
    calib_id: "0123456789ab"
    method: "manual sweep + Present_Position"
  arm_left_elbow_flex:
    min_tick: 400
    max_tick: 800
    measured_on: "2026-11-11"
    calib_id: "0123456789ac"
    method: "synthetic"
"""


def test_observed_limit_carries_the_measurement_provenance() -> None:
    """``ObservedLimit`` 的字段名与 ``calib_id`` 可空语义（未绑定代际的实测不能冒充）。"""
    observed = ObservedLimit(
        min_tick=100,
        max_tick=3000,
        measured_on="2026-11-10",
        calib_id=None,
        method="sweep",
    )

    assert (observed.min_tick, observed.max_tick) == (100, 3000)
    assert (observed.measured_on, observed.method) == ("2026-11-10", "sweep")
    assert observed.calib_id is None


def test_a_target_beyond_the_hard_limit_is_rejected_as_structural() -> None:
    """目标超出 ``hard_limit``：``allowed=False``、``reason_code=STRUCTURAL_LIMIT``。"""
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=_URDF,
        observed=None,
        calib_range=_URDF,
        target=120.0,
    )

    assert decision.allowed is False
    assert decision.reason_code == STRUCTURAL_LIMIT
    assert decision.hard_limit == _URDF
    assert decision.override_record is None


def test_an_observed_layer_narrows_the_hard_limit() -> None:
    """有实测层时 ``hard_limit = urdf ∩ observed``，边界来源记成 ``"observed"``。"""
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=_URDF,
        observed=(-30.0, 30.0),
        calib_range=_URDF,
        target=40.0,
    )

    assert decision.hard_limit == (-30.0, 30.0)
    assert decision.allowed is False
    assert decision.reason_code == STRUCTURAL_LIMIT
    assert decision.source_of_each_bound["hard_min"] == "observed"
    assert decision.source_of_each_bound["hard_max"] == "observed"


def test_without_an_observed_layer_the_hard_limit_is_the_urdf_limit() -> None:
    """没有实测层时 ``hard_limit == urdf_limit``，且不得把标定值当实测层顶上去。

    ``urdf`` 与 ``calib`` 在这里**取不同值**（``(-90, 90)`` vs ``(-30, 30)``）：取同一组值时
    「硬限来自 URDF」与「硬限来自标定」两种行为看不出区别，断言会变成空话。
    """
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=_URDF,
        observed=None,
        calib_range=(-30.0, 30.0),
        target=0.0,
    )

    bounds = decision.source_of_each_bound
    assert decision.allowed is True
    assert decision.reason_code is None
    assert (decision.hard_limit, decision.calib_range) == (_URDF, (-30.0, 30.0))
    assert bounds["hard_min"] == "urdf"
    assert bounds["hard_max"] == "urdf"
    assert bounds["calib_min"] == "calibration"
    assert bounds["calib_max"] == "calibration"
    assert "observed" not in bounds.values()


def test_an_absent_calib_range_is_reported_as_absent() -> None:
    """没有标定 range 时边界来源记成 ``"absent"``，不拿其它层顶替。"""
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=_URDF,
        observed=None,
        calib_range=None,
        target=0.0,
    )

    assert decision.allowed is True
    assert (decision.calib_range, decision.hard_limit) == (None, _URDF)
    assert decision.source_of_each_bound["calib_min"] == "absent"
    assert decision.source_of_each_bound["calib_max"] == "absent"


def test_a_target_inside_the_hard_limit_but_outside_the_calib_range_names_both_limits() -> None:
    """目标在硬限内、超出标定 range：拒绝并说明「标定区间限制」「不是结构限制」加两条出路。"""
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=_URDF,
        observed=None,
        calib_range=_NARROW_CALIB,
        target=20.0,
    )

    message = decision.message
    assert decision.allowed is False
    assert decision.reason_code == CALIB_RANGE_NARROWED
    assert decision.override_record is None
    assert "标定区间限制" in message
    assert "不是结构限制" in message
    assert any(token in message for token in ("实测", "observed", "measure")), message
    assert any(token in message for token in ("覆盖", "override", "allow_outside_calib_range")), (
        message
    )


def test_an_empty_intersection_between_hard_limit_and_calib_range_is_a_conflict() -> None:
    """``hard_limit`` 与 ``calib_range`` 交集为空：``LIMIT_CONFLICT`` 并提示重标定。"""
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=_URDF,
        observed=None,
        calib_range=(100.0, 200.0),
        target=0.0,
    )

    assert decision.allowed is False
    assert decision.reason_code == LIMIT_CONFLICT
    assert "重标定" in decision.message


def test_allow_outside_calib_range_records_an_explicit_override() -> None:
    """显式覆盖时放行且留痕：``override_record`` 非 ``None``，字段值可核对。"""
    decision = resolve_joint_limit(
        motor="arm_left_gripper",
        urdf_limit=_URDF,
        observed=None,
        calib_range=_NARROW_CALIB,
        target=20.0,
        allow_outside_calib_range=True,
    )

    override = decision.override_record
    assert decision.allowed is True
    assert decision.reason_code is None
    assert isinstance(override, OverrideRecord)
    assert override.motor == "arm_left_gripper"
    assert override.target == 20.0
    assert override.hard_limit == _URDF
    assert override.calib_range == _NARROW_CALIB
    assert override.reason


def test_allow_outside_calib_range_does_not_open_the_structural_limit() -> None:
    """覆盖开关只放开层 3：目标出硬限照样拒，且不产生覆盖记录。"""
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=_URDF,
        observed=None,
        calib_range=_URDF,
        target=120.0,
        allow_outside_calib_range=True,
    )

    assert decision.allowed is False
    assert decision.reason_code == STRUCTURAL_LIMIT
    assert decision.override_record is None


def test_load_observed_limits_returns_nothing_for_absent_input(tmp_path: Path) -> None:
    """``None`` 与不存在的路径都返回 ``{}``（不得用标定值顶替实测层、不得抛）。"""
    assert load_observed_limits(None) == {}
    assert load_observed_limits(tmp_path / "absent-observed-limits.yaml") == {}


def test_load_observed_limits_parses_the_frozen_yaml_schema(tmp_path: Path) -> None:
    """文件存在时按冻结 schema 逐台读出五字段，``min_tick`` / ``max_tick`` 是 ``int``。"""
    path = tmp_path / "limits-observed-synthetic.yaml"
    path.write_text(_SYNTHETIC_OBSERVED_YAML, encoding="utf-8")

    observed = load_observed_limits(path)

    assert set(observed) == {"arm_left_shoulder_pan", "arm_left_elbow_flex"}
    shoulder = observed["arm_left_shoulder_pan"]
    elbow = observed["arm_left_elbow_flex"]
    assert isinstance(shoulder.min_tick, int)
    assert isinstance(shoulder.max_tick, int)
    assert (shoulder.min_tick, shoulder.max_tick) == (100, 300)
    assert shoulder.measured_on == "2026-11-11"
    assert shoulder.calib_id == "0123456789ab"
    assert shoulder.method == "manual sweep + Present_Position"
    assert (elbow.min_tick, elbow.max_tick) == (400, 800)
    assert elbow.measured_on == "2026-11-11"
    assert elbow.calib_id == "0123456789ac"
    assert elbow.method == "synthetic"


def test_load_observed_limits_returns_empty_for_an_empty_motors_map(tmp_path: Path) -> None:
    """``motors: {}`` 的空模板（只有说明性注释）返回 ``{}``。"""
    path = tmp_path / "limits-observed-empty.yaml"
    path.write_text(
        "# 解析用合成样本，不是实测值\nschema_version: 1\nmotors: {}\n", encoding="utf-8"
    )

    assert load_observed_limits(path) == {}


def test_load_observed_limits_returns_empty_when_motors_is_absent(tmp_path: Path) -> None:
    """顶层没有 ``motors`` 键时返回 ``{}``，不抛。"""
    path = tmp_path / "limits-observed-no-motors.yaml"
    path.write_text("# 解析用合成样本，不是实测值\nschema_version: 1\n", encoding="utf-8")

    assert load_observed_limits(path) == {}


def test_load_observed_limits_returns_empty_for_a_comment_only_template(tmp_path: Path) -> None:
    """整份文件只有注释（``yaml.safe_load`` 得到 ``None``）时返回 ``{}``，不抛。"""
    path = tmp_path / "limits-observed-template.yaml"
    path.write_text("# 解析用合成样本，不是实测值\n# 尚未测量任何电机\n", encoding="utf-8")

    assert load_observed_limits(path) == {}


def test_the_shipped_observed_limits_config_is_an_empty_template() -> None:
    """仓库里的 ``configs/limits-observed.yaml`` 必须是空模板：层 2 缺席，不得编造实测值。"""
    data = yaml.safe_load(_OBSERVED_CONFIG.read_text(encoding="utf-8"))

    assert load_observed_limits(_OBSERVED_CONFIG) == {}
    assert isinstance(data, dict)
    assert data.get("motors") in (None, {}), f"实测层配置里出现了电机条目：{data.get('motors')!r}"


def test_a_crossed_hard_limit_blames_the_structure_and_the_observed_layer() -> None:
    """硬限自身交叉（URDF 与实测层互相矛盾）时文案必须指向那两层，不得引去重标定。

    ``urdf=(0, 10)`` 与 ``observed=(20, 30)`` 交集为空，算出的硬限 ``(20, 10)`` 自相矛盾；
    真实矛盾在层 1 与层 2 之间，跟这份标定无关，所以文案不得出现「重标定」。
    """
    decision = resolve_joint_limit(
        motor="arm_left_elbow_flex",
        urdf_limit=(0.0, 10.0),
        observed=(20.0, 30.0),
        calib_range=(0.0, 10.0),
        target=5.0,
    )

    assert decision.allowed is False
    assert decision.reason_code == LIMIT_CONFLICT
    assert "结构" in decision.message
    assert "实测" in decision.message
    assert "重标定" not in decision.message
