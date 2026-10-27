"""``tools/verify_unit_consistency.py`` 的离线契约测试。

被测脚本按路径加载（``importlib.util.spec_from_file_location``），它不是包内模块；
另有一例用 ``subprocess`` 跑真实 CLI，锁 ``__main__`` 路径与进程退出码。

全部用例只用 ``tmp_path`` 里构造的小样本，不读任何真机证据文件、不连设备、不联网。
断言按对外契约写：退出码、汇总行、逐行的列顺序、``MISMATCH:`` 诊断的可 grep 形态，
以及 ``deg_per_unit`` 与库 :func:`resolve_deg_per_unit` 的口径一致。
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from alohamini_cartesian.capability.detect import Capabilities, MotorInfo, resolve_deg_per_unit

_REPO_ROOT = Path(__file__).resolve().parents[1]
_SCRIPT_PATH = _REPO_ROOT / "tools" / "verify_unit_consistency.py"

_CALIBRATION_FIELDS = ("id", "drive_mode", "homing_offset", "range_min", "range_max")


def _load_script() -> ModuleType:
    """按路径加载被核对脚本（它不是包内模块）。"""
    assert _SCRIPT_PATH.is_file(), (
        f"被测脚本不存在：{_SCRIPT_PATH}（契约是冻的，缺文件要响亮失败，不跳过）"
    )
    spec = importlib.util.spec_from_file_location("verify_unit_consistency", _SCRIPT_PATH)
    assert spec is not None and spec.loader is not None, f"无法为 {_SCRIPT_PATH} 构造导入 spec"
    module = importlib.util.module_from_spec(spec)
    # ``dataclass`` 会按 ``cls.__module__`` 回查模块命名空间来解析字符串注解，
    # 所以先登记进 ``sys.modules`` 再执行。
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def tool() -> ModuleType:
    """被测脚本的模块对象，整个测试模块只加载一次。"""
    return _load_script()


def _motor(**overrides: Any) -> dict[str, Any]:
    """构造一条字段齐全的电机 metadata，可逐字段覆盖。"""
    fields: dict[str, Any] = {
        "id": 1,
        "model": "sts3250",
        "normalization": "range_m100_100",
        "drive_mode": 0,
        "range_min": 627,
        "range_max": 3327,
        "homing_offset": -1359,
        "resolution": 4096,
    }
    fields.update(overrides)
    return fields


def _cal_entry(motor: Mapping[str, Any]) -> dict[str, Any]:
    """从一条 metadata 里取出标定文件实际拥有的 5 个字段。"""
    return {name: motor[name] for name in _CALIBRATION_FIELDS}


def _consistent_pair() -> tuple[dict[str, Any], dict[str, Any]]:
    """构造一对两侧完全一致的三电机样本（含一个 ``range_0_100`` 夹爪）。"""
    motors = {
        "arm_left_shoulder_pan": _motor(id=1, range_min=627, range_max=3327, homing_offset=-1359),
        "arm_left_elbow_flex": _motor(
            id=3, model="sts3095", range_min=947, range_max=3109, homing_offset=-1322
        ),
        "arm_left_gripper": _motor(
            id=7,
            normalization="range_0_100",
            range_min=2021,
            range_max=2072,
            homing_offset=1571,
        ),
    }
    fingerprint: dict[str, Any] = {
        "schema_version": 1,
        "control_status": {"schema_version": 1},
        "robot_metadata": {
            "schema_version": 2,
            "robot_model": "aloha_mini_2_pro",
            "motors": motors,
        },
    }
    calibration = {name: _cal_entry(entry) for name, entry in motors.items()}
    return fingerprint, calibration


def _write(tmp_path: Path, name: str, payload: Any) -> Path:
    """把 payload 序列化成 ``tmp_path`` 下的 JSON 文件并返回其路径。"""
    path = tmp_path / name
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_all_consistent_returns_zero_and_flags_every_motor_ok(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """三台电机两侧一致时返回 0，汇总行报 ``mismatched=0``，每台电机 ``flag=OK``。"""
    fingerprint, calibration = _consistent_pair()
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 0
    assert "MISMATCH:" not in out
    assert "SUMMARY: motors=3 mismatched=0 exit=0" in out
    rows = [line for line in out.splitlines() if line.startswith("arm_")]
    assert len(rows) == 3
    for line in rows:
        assert line.rstrip().endswith("flag=OK")


def test_one_field_difference_returns_one_and_names_motor_and_field(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """某电机 ``range_max`` 差 1 时返回 1，并打出点名电机与字段的诊断行。"""
    fingerprint, calibration = _consistent_pair()
    calibration["arm_left_elbow_flex"]["range_max"] = 3108
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 1
    assert "SUMMARY: motors=3 mismatched=1 exit=1" in out
    assert (
        "MISMATCH: motor=arm_left_elbow_flex field=range_max fingerprint=3109 calibration=3108"
        in out
    )
    elbow_row = next(line for line in out.splitlines() if line.startswith("arm_left_elbow_flex"))
    assert elbow_row.rstrip().endswith("flag=MISMATCH")


def test_integral_float_homing_offset_is_compared_not_skipped(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """指纹侧 ``homing_offset`` 写成整值浮点（``-1359.0``）时照库口径取值并参与判等。"""
    fingerprint, calibration = _consistent_pair()
    fingerprint["robot_metadata"]["motors"]["arm_left_shoulder_pan"]["homing_offset"] = -1359.0
    calibration["arm_left_shoulder_pan"]["homing_offset"] = -1400
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 1
    assert "SUMMARY: motors=3 mismatched=1 exit=1" in out
    assert (
        "MISMATCH: motor=arm_left_shoulder_pan field=homing_offset "
        "fingerprint=-1359 calibration=-1400" in out
    )


def test_a_homing_offset_that_is_not_an_integer_returns_two(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """字段在场但收不成整数时返回 2，stderr 同时点名电机与字段。"""
    fingerprint, calibration = _consistent_pair()
    fingerprint["robot_metadata"]["motors"]["arm_left_elbow_flex"]["homing_offset"] = "x"
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    err = capsys.readouterr().err
    assert code == 2
    assert "arm_left_elbow_flex" in err
    assert "homing_offset" in err


def test_presence_difference_returns_one_and_names_the_presence_field(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """只在标定一侧出现的电机报 ``field=presence``，退出码 1。"""
    fingerprint, calibration = _consistent_pair()
    calibration["arm_left_wrist_flex"] = _cal_entry(_motor(id=4, range_min=934, range_max=3049))
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 1
    assert "SUMMARY: motors=4 mismatched=1 exit=1" in out
    assert (
        "MISMATCH: motor=arm_left_wrist_flex field=presence "
        "fingerprint=missing calibration=present" in out
    )


def test_calibration_resolution_difference_returns_one(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """标定侧多出的 ``resolution`` 参与判等：与指纹的 4096 不同即退出码 1。"""
    fingerprint, calibration = _consistent_pair()
    calibration["arm_left_elbow_flex"]["resolution"] = 2048
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 1
    assert "SUMMARY: motors=3 mismatched=1 exit=1" in out
    assert (
        "MISMATCH: motor=arm_left_elbow_flex field=resolution "
        "fingerprint=4096 calibration=2048" in out
    )


def test_calibration_only_unit_fields_print_a_note_without_affecting_the_exit(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """标定侧独有而指纹侧无该字段时打 ``NOTE``，不把它算成一条差异。"""
    fingerprint, calibration = _consistent_pair()
    calibration["arm_left_wrist_flex"] = {
        **_cal_entry(_motor(id=4, range_min=934, range_max=3049)),
        "resolution": 2048,
        "normalization": "range_0_100",
    }
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 1
    assert "SUMMARY: motors=4 mismatched=1 exit=1" in out
    assert "MISMATCH: motor=arm_left_wrist_flex field=presence" in out
    assert (
        "NOTE: motor=arm_left_wrist_flex calibration 侧带有 resolution=2048，"
        "指纹侧没有这台电机，未比较" in out
    )
    assert (
        "NOTE: motor=arm_left_wrist_flex calibration 侧带有 normalization=range_0_100，"
        "指纹侧没有这台电机，未比较" in out
    )


def test_missing_file_returns_two(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """标定文件不存在时返回 2，stderr 点名读不到的路径。"""
    fingerprint, _ = _consistent_pair()
    fp = _write(tmp_path, "fp.json", fingerprint)

    code = tool.main([str(fp), str(tmp_path / "absent.json")])

    err = capsys.readouterr().err
    assert code == 2
    assert "absent.json" in err


def test_broken_json_returns_two(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """输入不是合法 JSON 时返回 2，stderr 点名是哪个输入与为什么。"""
    _, calibration = _consistent_pair()
    cal = _write(tmp_path, "cal.json", calibration)
    bad = tmp_path / "bad.json"
    bad.write_text("{not json", encoding="utf-8")

    code = tool.main([str(bad), str(cal)])

    err = capsys.readouterr().err
    assert code == 2
    assert "bad.json" in err
    assert "JSON" in err


@pytest.mark.parametrize("missing_field", ["resolution", "normalization", "range_min", "range_max"])
def test_fingerprint_motor_with_a_missing_required_field_returns_two(
    tool: ModuleType,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    missing_field: str,
) -> None:
    """指纹里某电机缺必需字段时返回 2，stderr 同时点名电机与字段。"""
    fingerprint, calibration = _consistent_pair()
    del fingerprint["robot_metadata"]["motors"]["arm_left_elbow_flex"][missing_field]
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    err = capsys.readouterr().err
    assert code == 2
    assert "arm_left_elbow_flex" in err
    assert missing_field in err


_UNUSABLE_CASES = (
    ("resolution_at_one", "arm_left_elbow_flex"),
    ("unknown_normalization", "arm_left_elbow_flex"),
    ("inverted_range", "arm_left_elbow_flex"),
    ("calibration_missing_range_min", "arm_left_elbow_flex"),
    ("top_level_not_an_object", "fingerprint"),
)


@pytest.mark.parametrize(("case", "expected_in_stderr"), _UNUSABLE_CASES)
def test_unusable_input_returns_two(
    tool: ModuleType,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    case: str,
    expected_in_stderr: str,
) -> None:
    """五类不可用于核对的输入一律返回 2，stderr 点名该电机或该输入。"""
    fingerprint, calibration = _consistent_pair()
    fp_payload: Any = fingerprint
    cal_payload: Any = calibration
    if case == "resolution_at_one":
        fingerprint["robot_metadata"]["motors"]["arm_left_elbow_flex"]["resolution"] = 1
    elif case == "unknown_normalization":
        fingerprint["robot_metadata"]["motors"]["arm_left_elbow_flex"]["normalization"] = "range_x"
    elif case == "inverted_range":
        fingerprint["robot_metadata"]["motors"]["arm_left_elbow_flex"].update(
            {"range_min": 3109, "range_max": 947}
        )
        calibration["arm_left_elbow_flex"].update({"range_min": 3109, "range_max": 947})
    elif case == "calibration_missing_range_min":
        del calibration["arm_left_elbow_flex"]["range_min"]
    else:
        fp_payload = [1, 2, 3]
    fp = _write(tmp_path, "fp.json", fp_payload)
    cal = _write(tmp_path, "cal.json", cal_payload)

    code = tool.main([str(fp), str(cal)])

    err = capsys.readouterr().err
    assert code == 2
    assert expected_in_stderr in err


def test_deg_per_unit_matches_the_library_resolution(tool: ModuleType, tmp_path: Path) -> None:
    """脚本算出的 ``deg_per_unit`` 与库 ``resolve_deg_per_unit`` 的返回值相等。"""
    fingerprint, calibration = _consistent_pair()
    motors = fingerprint["robot_metadata"]["motors"]
    caps = Capabilities(
        metadata_schema=2,
        has_control_status=True,
        has_host_timing=False,
        robot_model="aloha_mini_2_pro",
        motors={name: MotorInfo(**entry) for name, entry in motors.items()},
        notes=(),
    )
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    result = tool.compare(tool.read_json(str(fp), "fingerprint"), tool.read_json(str(cal), "cal"))

    rows = {row.motor: row for row in result.rows}
    for name in motors:
        expected = resolve_deg_per_unit(caps, name).deg_per_unit
        assert rows[name].deg_per_unit == pytest.approx(expected, rel=1e-12)


def test_deg_span_values_follow_the_tick_formula(tool: ModuleType, tmp_path: Path) -> None:
    """角度行程按 ``(max - min) * 360 / (resolution - 1)`` 算：臂 237.3626…、夹爪 4.4835…。"""
    fingerprint, calibration = _consistent_pair()
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    result = tool.compare(tool.read_json(str(fp), "fingerprint"), tool.read_json(str(cal), "cal"))

    rows = {row.motor: row for row in result.rows}
    shoulder = rows["arm_left_shoulder_pan"]
    gripper = rows["arm_left_gripper"]
    assert shoulder.fp_deg_span == pytest.approx((3327 - 627) * 360 / 4095, rel=1e-12)
    assert shoulder.cal_deg_span == pytest.approx((3327 - 627) * 360 / 4095, rel=1e-12)
    assert gripper.fp_deg_span == pytest.approx((2072 - 2021) * 360 / 4095, rel=1e-12)
    assert gripper.cal_deg_span == pytest.approx((2072 - 2021) * 360 / 4095, rel=1e-12)


def test_row_columns_keep_their_order(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """整行按列序断言：样本两侧故意不对称，``fp_`` 列在前、``cal_`` 列在后。

    把标定侧 ``arm_left_elbow_flex.range_max`` 改成 ``3108``（指纹侧仍是 ``3109``）：于是
    ``fp_range=947..3109`` 与 ``cal_range=947..3108`` 不同、``fp_deg_span=190.07`` 与
    ``cal_deg_span=189.98`` 不同，逐列断言能抓住列对调。该电机因此 ``flag=MISMATCH``、
    退出码 1。
    """
    fingerprint, calibration = _consistent_pair()
    calibration["arm_left_elbow_flex"]["range_max"] = 3108
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 1
    line = next(item for item in out.splitlines() if item.startswith("arm_left_elbow_flex"))
    fields = line.split()
    assert fields[0] == "arm_left_elbow_flex"
    assert fields[1] == "range_m100_100"
    assert fields[2] == "4096"
    assert fields[3] == "947..3109"
    assert fields[4] == "947..3108"
    assert fields[5] == "0.950330"
    assert fields[6] == "190.07"
    assert fields[7] == "189.98"
    assert fields[8] == "-0.088"
    assert fields[9] == "flag=MISMATCH"


def test_degrees_motor_prints_unit_factor_and_na_angles(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``normalization == "degrees"`` 的电机打 ``1.000000`` 与 ``n/a`` 角度列，range 一致即 OK。"""
    fingerprint, calibration = _consistent_pair()
    lift = _motor(
        id=11,
        model="sts3095",
        normalization="degrees",
        range_min=0,
        range_max=4095,
        homing_offset=0,
    )
    fingerprint["robot_metadata"]["motors"]["lift_axis"] = lift
    calibration["lift_axis"] = _cal_entry(lift)
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 0
    line = next(item for item in out.splitlines() if item.startswith("lift_axis"))
    assert "degrees" in line
    assert "1.000000" in line
    assert line.count("n/a") == 3
    assert line.rstrip().endswith("flag=OK")


def test_fingerprint_motors_may_sit_at_the_top_level(
    tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """指纹没有 ``robot_metadata`` 时退到顶层 ``motors``，结论不变。"""
    fingerprint, calibration = _consistent_pair()
    flat = {"motors": fingerprint["robot_metadata"]["motors"]}
    fp = _write(tmp_path, "fp.json", flat)
    cal = _write(tmp_path, "cal.json", calibration)

    code = tool.main([str(fp), str(cal)])

    out = capsys.readouterr().out
    assert code == 0
    assert "SUMMARY: motors=3 mismatched=0 exit=0" in out


def test_cli_through_subprocess_reports_the_exit_code(tmp_path: Path) -> None:
    """真实 CLI（``__main__`` 路径）把差异报成进程退出码 1，汇总行进 stdout。"""
    fingerprint, calibration = _consistent_pair()
    calibration["arm_left_gripper"]["range_min"] = 2022
    fp = _write(tmp_path, "fp.json", fingerprint)
    cal = _write(tmp_path, "cal.json", calibration)

    proc = subprocess.run(
        [sys.executable, str(_SCRIPT_PATH), str(fp), str(cal)],
        capture_output=True,
        text=True,
        check=False,
    )

    assert proc.returncode == 1
    assert "SUMMARY: motors=3 mismatched=1 exit=1" in proc.stdout
    assert "field=range_min" in proc.stdout
