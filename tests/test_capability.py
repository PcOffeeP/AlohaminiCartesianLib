"""``alohamini_cartesian.capability.detect`` 的契约测试（TDD 红阶段）。

当前 ``src/alohamini_cartesian/capability/detect.py`` 里所有函数体都是
``NotImplementedError``，本文件预期全红。断言按对外契约（签名、dataclass 字段、
fail-closed 语义、错误文案关键字）写，不按实现细节写。
"""

from __future__ import annotations

import ast
import importlib
import math
import re
import sys
from pathlib import Path
from typing import Any

import pytest

import alohamini_cartesian
from alohamini_cartesian.capability.detect import (
    CONTROL_STATUS_KEY,
    HOST_TIMING_KEY,
    METADATA_KEY,
    SPAN_BY_NORMALIZATION,
    SUPPORTED_METADATA_SCHEMAS,
    Capabilities,
    CapabilityError,
    Conversion,
    ConversionUnavailable,
    MotorInfo,
    detect_from_observation,
    resolve_deg_per_unit,
)

M100_100_EXPECTED = ((3400 - 700) / 200) * 360 / (4096 - 1)
RANGE_0_100_EXPECTED = ((3400 - 700) / 100) * 360 / (4096 - 1)


def _motor(**overrides: Any) -> MotorInfo:
    """构造一个字段齐全的 MotorInfo，可逐字段覆盖。"""
    fields: dict[str, Any] = {
        "id": 1,
        "model": "sts3215",
        "normalization": "range_m100_100",
        "drive_mode": 0,
        "range_min": 700,
        "range_max": 3400,
        "homing_offset": -12,
        "resolution": 4096,
    }
    fields.update(overrides)
    return MotorInfo(**fields)


def _caps(
    motors: dict[str, MotorInfo] | None = None,
    *,
    schema: int | None = 2,
) -> Capabilities:
    """构造一次能力探测结果，默认带一个可换算的 shoulder_pan 电机。"""
    return Capabilities(
        metadata_schema=schema,
        has_control_status=True,
        has_host_timing=True,
        robot_model="alohamini2",
        motors={} if motors is None else motors,
        notes=(),
    )


def _v2_metadata() -> dict[str, Any]:
    """v2 metadata：八个电机字段齐全。"""
    return {
        "schema_version": 2,
        "robot_model": "alohamini2",
        "motors": {
            "shoulder_pan": {
                "id": 1,
                "model": "sts3215",
                "normalization": "range_m100_100",
                "drive_mode": 0,
                "range_min": 700,
                "range_max": 3400,
                "homing_offset": -12,
                "resolution": 4096,
            }
        },
    }


def _v1_metadata() -> dict[str, Any]:
    """v1 metadata：只有六个电机字段，缺 homing_offset / resolution。"""
    return {
        "schema_version": 1,
        "robot_model": "alohamini1",
        "motors": {
            "shoulder_pan": {
                "id": 6,
                "model": "sts3215",
                "normalization": "range_0_100",
                "drive_mode": 1,
                "range_min": 900,
                "range_max": 3200,
            }
        },
    }


def _v2_observation() -> dict[str, Any]:
    """带齐 metadata / control status / host timing 三个侧信道键的观测。"""
    return {
        METADATA_KEY: _v2_metadata(),
        "shoulder_pan.pos": 0.0,
        CONTROL_STATUS_KEY: {"state": "ok"},
        HOST_TIMING_KEY: {"loop_dt_ms": 20.0},
    }


# --------------------------------------------------------------------------- #
# A. detect_from_observation
# --------------------------------------------------------------------------- #


def test_module_constants_match_documented_contract() -> None:
    """模块级常量是对外契约的一部分，必须与文档一致。"""
    assert SPAN_BY_NORMALIZATION == {
        "range_m100_100": 200.0,
        "range_0_100": 100.0,
        "degrees": 1.0,
    }
    assert SUPPORTED_METADATA_SCHEMAS == (1, 2)
    assert METADATA_KEY == "_robot_metadata"
    assert CONTROL_STATUS_KEY == "_control_status"
    assert HOST_TIMING_KEY == "_host_timing"


def test_detect_parses_full_v2_metadata() -> None:
    """A1：v2 metadata 的顶层与电机八个字段都应逐个解析出来。"""
    caps = detect_from_observation(_v2_observation())

    assert caps.metadata_schema == 2
    assert caps.robot_model == "alohamini2"
    assert caps.has_control_status is True
    assert caps.has_host_timing is True

    motor = caps.motors["shoulder_pan"]
    assert motor.id == 1
    assert motor.model == "sts3215"
    assert motor.normalization == "range_m100_100"
    assert motor.drive_mode == 0
    assert motor.range_min == 700
    assert motor.range_max == 3400
    assert motor.homing_offset == -12
    assert motor.resolution == 4096


def test_detect_parses_v1_metadata_and_records_missing_field_notes() -> None:
    """A2：v1 缺的字段填 None，并把缺口名写进 tuple 类型的 notes。"""
    observation = {METADATA_KEY: _v1_metadata()}

    caps = detect_from_observation(observation)

    assert caps.metadata_schema == 1
    assert caps.robot_model == "alohamini1"
    motor = caps.motors["shoulder_pan"]
    assert motor.id == 6
    assert motor.homing_offset is None
    assert motor.resolution is None
    assert isinstance(caps.notes, tuple)
    assert any("homing_offset" in note for note in caps.notes)
    assert any("resolution" in note for note in caps.notes)


@pytest.mark.parametrize("observation", [{}, {"shoulder_pan.pos": 0.0}])
def test_detect_raises_when_metadata_key_absent(observation: dict[str, Any]) -> None:
    """A3：观测里没有 _robot_metadata 时 fail-closed。"""
    with pytest.raises(CapabilityError, match="_robot_metadata"):
        detect_from_observation(observation)


@pytest.mark.parametrize("metadata", [{}, None])
def test_detect_raises_when_metadata_is_empty_or_none(metadata: Any) -> None:
    """A3：metadata 为空 dict 或 None 同样不猜，直接抛错。"""
    with pytest.raises(CapabilityError):
        detect_from_observation({METADATA_KEY: metadata})


@pytest.mark.parametrize("schema_version", [True, 1.0, "1", None])
def test_detect_rejects_non_integer_schema_version(schema_version: Any) -> None:
    """A4：bool / float / str / None 都不算整数版本，包括 isinstance(True, int) 为真的 True。"""
    metadata = _v2_metadata()
    metadata["schema_version"] = schema_version

    with pytest.raises(CapabilityError, match="schema_version"):
        detect_from_observation({METADATA_KEY: metadata})


def test_detect_rejects_unsupported_schema_version() -> None:
    """A5：不在 SUPPORTED_METADATA_SCHEMAS 里的版本号必须拒绝。"""
    metadata = _v2_metadata()
    metadata["schema_version"] = 3

    with pytest.raises(CapabilityError, match="schema_version"):
        detect_from_observation({METADATA_KEY: metadata})


@pytest.mark.parametrize(
    "observation",
    [
        {METADATA_KEY: _v2_metadata(), "shoulder_pan.pos": 0.0},
        {METADATA_KEY: _v2_metadata(), "shoulder_pan.pos": 0.0, CONTROL_STATUS_KEY: None},
    ],
)
def test_detect_reports_absent_control_status_without_raising(
    observation: dict[str, Any],
) -> None:
    """A6：_control_status 缺失或为 None 只降级为 False，不抛异常。"""
    caps = detect_from_observation(observation)

    assert caps.has_control_status is False
    assert caps.metadata_schema == 2


def test_detect_reports_absent_host_timing_without_raising() -> None:
    """A7：_host_timing 缺失只降级为 False，不抛异常。"""
    observation = {METADATA_KEY: _v2_metadata(), CONTROL_STATUS_KEY: {"state": "ok"}}

    caps = detect_from_observation(observation)

    assert caps.has_host_timing is False
    assert caps.has_control_status is True


def test_detect_fills_missing_motor_fields_with_none() -> None:
    """A8：缺失的电机字段必须是 None，不能填 0 或 4096 这类假值。"""
    metadata = {
        "schema_version": 2,
        "robot_model": "alohamini2",
        "motors": {"shoulder_pan": {"id": 1}},
    }

    motor = detect_from_observation({METADATA_KEY: metadata}).motors["shoulder_pan"]

    assert motor.id == 1
    assert motor.model is None
    assert motor.normalization is None
    assert motor.drive_mode is None
    assert motor.range_min is None
    assert motor.range_max is None
    assert motor.homing_offset is None
    assert motor.resolution is None
    assert motor.range_min != 0
    assert motor.resolution != 4096


def test_detect_rejects_non_mapping_observation() -> None:
    """A9：观测不是 Mapping 时抛 CapabilityError。"""
    with pytest.raises(CapabilityError):
        detect_from_observation(None)  # type: ignore[arg-type]


@pytest.mark.parametrize("motors", ["ABSENT", {}])
def test_detect_tolerates_missing_or_empty_motors(motors: Any) -> None:
    """A9：motors 缺失或为空 Mapping 时返回空映射并在 notes 里留痕。"""
    metadata: dict[str, Any] = {"schema_version": 2, "robot_model": "alohamini2"}
    if motors != "ABSENT":
        metadata["motors"] = motors

    caps = detect_from_observation({METADATA_KEY: metadata})

    assert caps.motors == {}
    assert len(caps.notes) > 0


# --------------------------------------------------------------------------- #
# B. resolve_deg_per_unit
# --------------------------------------------------------------------------- #


def test_resolve_uses_metadata_fields_first() -> None:
    """B1：metadata 里 range 与 resolution 齐全时，source 为 metadata。"""
    caps = _caps({"shoulder_pan": _motor()})

    conv = resolve_deg_per_unit(caps, "shoulder_pan")

    assert isinstance(conv, Conversion)
    assert conv.source == "metadata"
    assert conv.deg_per_unit == pytest.approx(M100_100_EXPECTED)
    assert conv.range_min == 700
    assert conv.range_max == 3400
    assert conv.resolution == 4096


def test_resolve_falls_back_to_calibration_file() -> None:
    """B2：metadata 缺 range 时用外部标定文件的 range + 模型表分辨率。"""
    caps = _caps({"shoulder_pan": _motor(range_min=None, range_max=None, resolution=None)})

    conv = resolve_deg_per_unit(
        caps,
        "shoulder_pan",
        calibration={"shoulder_pan": {"range_min": 700, "range_max": 3400}},
        model_resolution={"sts3215": 4096},
    )

    assert conv.source == "calibration-file"
    assert conv.deg_per_unit == pytest.approx(M100_100_EXPECTED)
    assert conv.range_min == 700
    assert conv.range_max == 3400
    assert conv.resolution == 4096


def test_resolve_falls_back_to_model_table() -> None:
    """B3：range 来自 metadata、resolution 来自模型表时，source 为 model-table。"""
    caps = _caps({"shoulder_pan": _motor(resolution=None)})

    conv = resolve_deg_per_unit(caps, "shoulder_pan", model_resolution={"sts3215": 4096})

    assert conv.source == "model-table"
    assert conv.deg_per_unit == pytest.approx(M100_100_EXPECTED)
    assert conv.range_min == 700
    assert conv.range_max == 3400
    assert conv.resolution == 4096


def test_resolve_raises_when_range_missing() -> None:
    """B4：三个来源都没有 range 时 fail-closed。"""
    caps = _caps({"shoulder_pan": _motor(range_min=None, range_max=None)})

    with pytest.raises(ConversionUnavailable, match="range"):
        resolve_deg_per_unit(caps, "shoulder_pan", model_resolution={"sts3215": 4096})


def test_resolve_range_error_names_the_missing_field() -> None:
    """B4：错误信息要点出缺口名字（range_max 缺失）。"""
    caps = _caps({"shoulder_pan": _motor(range_max=None)})

    with pytest.raises(ConversionUnavailable) as excinfo:
        resolve_deg_per_unit(caps, "shoulder_pan")

    assert "range_max" in str(excinfo.value)


@pytest.mark.parametrize("model_resolution", [None, {"other_model": 8192}])
def test_resolve_raises_when_resolution_missing(
    model_resolution: dict[str, int] | None,
) -> None:
    """B5：metadata 与模型表都给不出分辨率时 fail-closed。"""
    caps = _caps({"shoulder_pan": _motor(resolution=None)})

    with pytest.raises(ConversionUnavailable, match="resolution"):
        resolve_deg_per_unit(caps, "shoulder_pan", model_resolution=model_resolution)


@pytest.mark.parametrize(
    ("normalization", "expected_deg_per_unit"),
    [
        ("range_m100_100", M100_100_EXPECTED),
        ("range_0_100", RANGE_0_100_EXPECTED),
        ("degrees", 1.0),
    ],
)
def test_resolve_span_by_normalization(normalization: str, expected_deg_per_unit: float) -> None:
    """B6：同一 range/resolution 下，三种 normalization 各自的 span 决定系数。"""
    caps = _caps({"shoulder_pan": _motor(normalization=normalization)})

    conv = resolve_deg_per_unit(caps, "shoulder_pan")

    assert conv.deg_per_unit == pytest.approx(expected_deg_per_unit)


def test_resolve_range_0_100_is_twice_range_m100_100() -> None:
    """B7：range_0_100 的 span 是 range_m100_100 的一半，系数据此翻倍。"""
    caps_m100 = _caps({"shoulder_pan": _motor(normalization="range_m100_100")})
    caps_0_100 = _caps({"shoulder_pan": _motor(normalization="range_0_100")})

    conv_m100 = resolve_deg_per_unit(caps_m100, "shoulder_pan")
    conv_0_100 = resolve_deg_per_unit(caps_0_100, "shoulder_pan")

    assert conv_0_100.deg_per_unit == pytest.approx(conv_m100.deg_per_unit * 2)


def test_resolve_rejects_unknown_normalization() -> None:
    """B8：未知 normalization 不猜 span，直接抛错且信息里带 normalization。"""
    caps = _caps({"shoulder_pan": _motor(normalization="percent")})

    with pytest.raises(ConversionUnavailable, match="normalization"):
        resolve_deg_per_unit(caps, "shoulder_pan")


def test_resolve_rejects_unknown_motor() -> None:
    """B9：电机不在 caps.motors 里时，错误信息要含该电机名。"""
    caps = _caps({"shoulder_pan": _motor()})

    with pytest.raises(ConversionUnavailable, match="nonexistent_joint"):
        resolve_deg_per_unit(caps, "nonexistent_joint")


@pytest.mark.parametrize(("range_min", "range_max"), [(3400, 700), (1000, 1000)])
def test_resolve_rejects_empty_or_inverted_range(range_min: int, range_max: int) -> None:
    """B10：range_max <= range_min 时无有效区间，必须抛错。"""
    caps = _caps({"shoulder_pan": _motor(range_min=range_min, range_max=range_max)})

    with pytest.raises(ConversionUnavailable):
        resolve_deg_per_unit(caps, "shoulder_pan")


@pytest.mark.parametrize("resolution", [1, 0])
def test_resolve_rejects_non_positive_resolution_span(resolution: int) -> None:
    """B10：resolution <= 1 会让分母 <= 0，必须抛错。"""
    caps = _caps({"shoulder_pan": _motor(resolution=resolution)})

    with pytest.raises(ConversionUnavailable):
        resolve_deg_per_unit(caps, "shoulder_pan")


def test_resolve_accepts_pos_suffix() -> None:
    """B11：带 .pos 后缀的关节名与裸电机名解析出完全相同的换算。"""
    caps = _caps({"shoulder_pan": _motor()})

    with_suffix = resolve_deg_per_unit(caps, "shoulder_pan.pos")
    without_suffix = resolve_deg_per_unit(caps, "shoulder_pan")

    assert with_suffix == without_suffix
    assert with_suffix.deg_per_unit == pytest.approx(M100_100_EXPECTED)


def test_resolve_deg_per_unit_is_positive_and_finite() -> None:
    """B12：换算系数必须是正的有限值。"""
    caps = _caps({"shoulder_pan": _motor()})

    conv = resolve_deg_per_unit(caps, "shoulder_pan")

    assert math.isfinite(conv.deg_per_unit)
    assert 0 < conv.deg_per_unit < math.inf


# --------------------------------------------------------------------------- #
# C. 审查回归补充
# --------------------------------------------------------------------------- #


def _v1_metadata_with_full_motor_fields() -> dict[str, Any]:
    """第 1 代 metadata 的 motor 条目意外带齐八个字段（含 homing_offset / resolution）。"""
    return {
        "schema_version": 1,
        "robot_model": "alohamini1",
        "motors": {
            "shoulder_pan": {
                "id": 1,
                "model": "sts3215",
                "normalization": "range_m100_100",
                "drive_mode": 0,
                "range_min": 700,
                "range_max": 3400,
                "homing_offset": -12,
                "resolution": 4096,
            }
        },
    }


def _imported_module_names(path: Path) -> list[str]:
    """返回某个源文件里所有 import 语句引用的模块名（只统计绝对 import）。"""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module is not None:
            names.append(node.module)
    return names


def _lerobot_imports(path: Path) -> list[str]:
    """返回某个源文件里 import 到的 lerobot 模块名（``lerobot`` 及其子模块）。"""
    return [
        name
        for name in _imported_module_names(path)
        if name == "lerobot" or name.startswith("lerobot.")
    ]


def test_schema1_notes_do_not_claim_fields_that_are_present() -> None:
    """告警只声明实际缺失的字段，不断言存在的 homing_offset / resolution 按 None 处理。"""
    observation = {METADATA_KEY: _v1_metadata_with_full_motor_fields()}

    caps = detect_from_observation(observation)

    motor = caps.motors["shoulder_pan"]
    assert motor.homing_offset == -12
    assert motor.resolution == 4096
    assert not any("homing_offset" in note for note in caps.notes)
    assert not any("resolution" in note for note in caps.notes)


def test_schema1_notes_still_name_missing_fields() -> None:
    """护栏：schema 1 真的缺字段时，notes 必须仍然点名 homing_offset 与 resolution。"""
    caps = detect_from_observation({METADATA_KEY: _v1_metadata()})

    motor = caps.motors["shoulder_pan"]
    assert motor.homing_offset is None
    assert motor.resolution is None
    assert any("homing_offset" in note for note in caps.notes)
    assert any("resolution" in note for note in caps.notes)


def test_resolve_degrees_without_any_calibration_returns_placeholder_triple() -> None:
    """换算单位为度且三个来源全空时，占位三元组固定为 0/0/1，source 记 metadata。"""
    caps = _caps(
        {
            "shoulder_pan": _motor(
                normalization="degrees",
                range_min=None,
                range_max=None,
                resolution=None,
            )
        }
    )

    conv = resolve_deg_per_unit(caps, "shoulder_pan", calibration=None, model_resolution=None)

    assert conv.deg_per_unit == pytest.approx(1.0)
    assert conv.range_min == 0
    assert conv.range_max == 0
    assert conv.resolution == 1
    assert conv.source == "metadata"


def test_resolve_mixed_sources_reports_the_fallback_tier() -> None:
    """混合来源时 source 报最弱的一级：range 有字段回落即算 calibration-file。"""
    caps = _caps(
        {
            "shoulder_pan": _motor(
                model="sts3215",
                normalization="range_m100_100",
                range_min=700,
                range_max=None,
                resolution=None,
            )
        }
    )

    conv = resolve_deg_per_unit(
        caps,
        "shoulder_pan",
        calibration={"shoulder_pan": {"range_max": 3400}},
        model_resolution={"sts3215": 4096},
    )

    assert conv.range_min == 700
    assert conv.range_max == 3400
    assert conv.resolution == 4096
    assert conv.source == "calibration-file"
    assert conv.deg_per_unit == pytest.approx(((3400 - 700) / 200) * 360 / (4096 - 1))


def test_detect_rejects_bool_in_motor_fields() -> None:
    """布尔值不是整数：id / homing_offset / range_min 拿到 True/False 一律按 None 处理。"""
    metadata = {
        "schema_version": 2,
        "robot_model": "alohamini2",
        "motors": {
            "shoulder_pan": {
                "id": True,
                "model": "sts3215",
                "normalization": "range_m100_100",
                "drive_mode": 0,
                "range_min": True,
                "range_max": 3400,
                "homing_offset": False,
                "resolution": 4096,
            }
        },
    }

    motor = detect_from_observation({METADATA_KEY: metadata}).motors["shoulder_pan"]

    assert motor.id is None
    assert motor.homing_offset is None
    assert motor.range_min is None
    assert motor.model == "sts3215"
    assert motor.normalization == "range_m100_100"
    assert motor.drive_mode == 0


def test_resolve_degrees_rejects_inverted_range_from_a_source() -> None:
    """换算单位为度时也要校验来源给出的真实区间：range_max <= range_min 必须抛错。"""
    caps = _caps(
        {
            "shoulder_pan": _motor(
                normalization="degrees",
                range_min=3400,
                range_max=700,
                resolution=4096,
            )
        }
    )

    with pytest.raises(ConversionUnavailable, match="range"):
        resolve_deg_per_unit(caps, "shoulder_pan")


def test_no_module_outside_adapter_imports_lerobot() -> None:
    """适配层是唯一接触面：源码里没有任何模块 import lerobot，protocol.py 自身也不 import。"""
    package_root = Path(alohamini_cartesian.__file__).parent
    protocol_path = package_root / "adapter" / "protocol.py"

    offenders = {
        str(path.relative_to(package_root)): _lerobot_imports(path)
        for path in sorted(package_root.rglob("*.py"))
        if path != protocol_path and _lerobot_imports(path)
    }

    assert offenders == {}
    assert protocol_path.exists()
    assert _lerobot_imports(protocol_path) == []

    # 运行期断言按「前后快照差集」的口径写，不断言重放会话级的 ``"lerobot"
    # not in sys.modules``：装了文档承诺的 ``[runtime]`` extra 的用户跑整套
    # 测试时，parity 文件会在收集期真的 ``import lerobot`` 并留下
    # ``sys.modules`` 条目，那条全局断言于是把「导入本库的这两个无适配器模块
    # 没有拉起 lerobot」错判成会话顺序问题。这里只钉住被测动作本身不引入任何
    # lerobot 模块，与收集顺序无关。
    def _lerobot_modules() -> set[str]:
        return {name for name in sys.modules if name == "lerobot" or name.startswith("lerobot.")}

    before = _lerobot_modules()

    importlib.import_module("alohamini_cartesian.capability.detect")
    importlib.import_module("alohamini_cartesian.config.overlay")

    after = _lerobot_modules()
    assert after - before == set()


# --------------------------------------------------------------------------- #
# D. V11 兼容性现状锁定（v1 帧「帧平滑回退」）
#
# 背景：退役仓库的 `docs/alohamini-control/Alohamini-implementation-plan.md:80`
# 把「v1 格式帧平滑回退」列为 V11 的要求之一。本库现状拆成三件事：
#   1) v1 帧能被收下：schema 1 缺字段填 None，缺口写进 notes；
#   2) 换算按**字段**回落（metadata → calibration-file → model-table），
#      缺到无处可落时抛 ConversionUnavailable，不猜 4096；
#   3) capability 层没有任何跨帧平滑：没 EMA、没低通、没 slew 限速，也没有「保持
#      上一帧」的缓存；`detect_from_observation` 与 `resolve_deg_per_unit` 都是
#      逐帧无状态纯函数（下面 V11-3 / V11-4 锁这一层）。
#
# 边界（不要把它读成本款已满足）：控制层自带每帧关节步长门禁与跨帧状态 ——
# `control/controller.py:145` 的 `_last_accepted_q`、`:520-532` 的 step-limit 门禁
# （`_max_joint_step_deg` 默认 2.0，超过即 `ControlError.STEP_LIMIT`、不发）、
# `:534-546` 的 lead clamp（`max_joint_lead_deg=30.0`，`control/api.py:668` 传入）。
# 那些是安全门禁与截断，不是对帧序列做平滑或插值，也不按 metadata 版本切换；
# 它们不在 V11 第三款的判据内，本条只记「没有平滑」这个现状。
# D 组把这三件事钉住，等于给 V11 第三款留下可核对的现状记录：谁要加平滑，
# 先改这里与 `docs/acceptance-baseline.md` 的 V11 行。
# --------------------------------------------------------------------------- #

# 期望换算：`100` 是 `range_0_100` 的 span，这里故意写字面量、不从实现里 import
# 反推；`4096` 来自下面用例注入的型号表（v1 帧本身没有 resolution 字段）。
V1_FRAME_EXPECTED_DEG_PER_UNIT = ((3200 - 900) / 100) * 360 / (4096 - 1)

# 平滑原语的符号级特征词。用词边界而不是裸子串：裸 `ema` 会被 `schema_version` 命中。
# `_ema` 这种前导下划线写法单独列一条（下划线是词内字符，`\bema` 抓不到）。
FRAME_SMOOTHING_PATTERN = re.compile(
    r"平滑|滤波|低通|_ema[a-z0-9_]*|\b(?:ema|slew|smooth|low_?pass)[a-z0-9_]*",
    re.IGNORECASE,
)


def _v1_frame_observation() -> dict[str, Any]:
    """一帧 v1 观测：metadata 走 schema 1（缺 homing_offset / resolution）。"""
    return {METADATA_KEY: _v1_metadata()}


def _v2_frame_with_other_range_and_resolution() -> dict[str, Any]:
    """一帧数值口径完全不同的 v2 观测，用来检测跨帧残留（保持上一帧 / 平滑）。"""
    return {
        METADATA_KEY: {
            "schema_version": 2,
            "robot_model": "alohamini2",
            "motors": {
                "shoulder_pan": {
                    "id": 1,
                    "model": "sts3215",
                    "normalization": "range_m100_100",
                    "drive_mode": 0,
                    "range_min": -2000,
                    "range_max": 2000,
                    "homing_offset": 7,
                    "resolution": 8192,
                }
            },
        }
    }


def test_v11_v1_frame_parses_and_conversion_falls_back_by_field() -> None:
    """V11-1：v1 帧可解析；换算回落在「缺哪个字段补哪个」，无源可补时 fail-closed。"""
    # 前半段与 A2（`:157` 起）、C 段（`:496` 起）有意重复：V11 要一条自成一体的证据。
    caps = detect_from_observation(_v1_frame_observation())

    assert caps.metadata_schema == 1
    motor = caps.motors["shoulder_pan"]
    assert motor.homing_offset is None
    assert motor.resolution is None
    assert any("homing_offset" in note for note in caps.notes)
    assert any("resolution" in note for note in caps.notes)
    assert any("shoulder_pan" in note for note in caps.notes)

    # v1 帧自身有 range、没有 resolution；没有任何可回落来源时不猜。
    with pytest.raises(ConversionUnavailable, match="resolution") as excinfo:
        resolve_deg_per_unit(caps, "shoulder_pan")
    assert "不猜" in str(excinfo.value)

    # 给出型号分辨率表后，resolution 从 model-table 回落，range 仍来自 v1 帧。
    conv = resolve_deg_per_unit(caps, "shoulder_pan", model_resolution={"sts3215": 4096})

    assert conv.source == "model-table"
    assert conv.resolution == 4096
    assert conv.range_min == 900
    assert conv.range_max == 3200
    assert conv.deg_per_unit == pytest.approx(V1_FRAME_EXPECTED_DEG_PER_UNIT)


def test_v11_v1_frame_with_full_fields_converts_from_metadata_like_v2() -> None:
    """V11-2：回落按字段是否存在判定，不按版本号——八字段齐的 v1 帧与 v2 同解。

    这只说明 capability 层能算换算，不等于关节换算适配器被使能：
    `tests/test_kinematics_adapter.py::test_t_b7_d_schema_version_missing_or_not_two_rejected`
    要求 schema 1 必须被拒。
    """
    v1_caps = detect_from_observation({METADATA_KEY: _v1_metadata_with_full_motor_fields()})
    v2_caps = detect_from_observation({METADATA_KEY: _v2_metadata()})

    v1_conv = resolve_deg_per_unit(v1_caps, "shoulder_pan")
    v2_conv = resolve_deg_per_unit(v2_caps, "shoulder_pan")

    assert v1_caps.metadata_schema == 1
    assert v1_conv.source == "metadata"
    assert v1_conv == v2_conv


def test_v11_consecutive_frames_are_parsed_independently_no_smoothing() -> None:
    """V11-3：逐帧无状态——后一帧不依赖前一帧，没有跨帧平滑或「保持上一帧」。

    两帧的 range 与 resolution 都不同（v1 帧 900..3200 / 无分辨率 → 型号表 4096；
    v2 帧 -2000..2000 / 8192）。若实现里加了 EMA、低通或按电机名缓存上一帧的口径，
    第二次解 v1 帧的结果会偏离第一次，本用例即红。

    抓得到的边界：控制层的平滑 / 限速（`control/controller.py` 的 step-limit 门禁与
    lead clamp）不在本用例判别面内 —— D 段只碰 capability 层。
    """
    table = {"sts3215": 4096}

    first = resolve_deg_per_unit(
        detect_from_observation(_v1_frame_observation()),
        "shoulder_pan",
        model_resolution=table,
    )
    middle = resolve_deg_per_unit(
        detect_from_observation(_v2_frame_with_other_range_and_resolution()), "shoulder_pan"
    )
    again = resolve_deg_per_unit(
        detect_from_observation(_v1_frame_observation()),
        "shoulder_pan",
        model_resolution=table,
    )

    assert middle.source == "metadata"
    assert (middle.range_min, middle.range_max, middle.resolution) == (-2000, 2000, 8192)
    assert first == again
    assert (first.range_min, first.range_max, first.resolution) == (900, 3200, 4096)

    # 解析不改动输入：传进去的观测字典不得被就地改写。
    observation = _v1_frame_observation()
    before = repr(observation)
    detect_from_observation(observation)
    assert repr(observation) == before, "解析不得就地改写输入观测"

    # 同 schema / 同电机 / 同 normalization，只改 range：缓存若按 (版本, 电机) 或
    # 按 span 做键、或先预热若干帧再返回，这里就会偏离自己的期望值。
    other = _v1_metadata()
    other["motors"]["shoulder_pan"].update({"range_min": 1000, "range_max": 3100})
    other_conv = resolve_deg_per_unit(
        detect_from_observation({METADATA_KEY: other}),
        "shoulder_pan",
        model_resolution=table,
    )
    assert (other_conv.range_min, other_conv.range_max) == (1000, 3100)
    assert other_conv.deg_per_unit == pytest.approx(((3100 - 1000) / 100) * 360 / (4096 - 1))


def test_v11_no_frame_smoothing_primitive_in_the_package() -> None:
    """V11-4：capability 层没有任何帧平滑原语——这一款是「无实现」，不是「无测试」。

    `detect_from_observation` 与 `resolve_deg_per_unit` 这层既没有 EMA / 低通 / slew 限速，
    也没有保持上一帧的缓存；下面的扫描面覆盖整个 `src/alohamini_cartesian/`（现状零命中）。
    要加实现时本用例会先红，连同 `docs/acceptance-baseline.md` 的 V11 行一起改。

    边界：控制层的每帧关节步长门禁（`control/controller.py:520-532`）与 lead clamp
    （`:534-546`）属安全门禁与截断，不是帧序列平滑，不在本款判据内。
    """
    package_root = Path(alohamini_cartesian.__file__).parent

    offenders = {
        str(path.relative_to(package_root)): sorted(
            {
                hit.lower()
                for hit in FRAME_SMOOTHING_PATTERN.findall(path.read_text(encoding="utf-8"))
            }
        )
        for path in sorted(package_root.rglob("*.py"))
    }
    offenders = {name: tokens for name, tokens in offenders.items() if tokens}

    assert offenders == {}
