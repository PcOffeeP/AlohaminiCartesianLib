"""``tools/calib_health.py`` 与 ``tools/calib_register.py`` 的离线契约测试（TDD 红阶段）。

两个被测脚本都还没实现，本文件按任务书冻结的契约写断言，运行必然失败（加载脚本时的
断言失败，随后才是包内模块缺失）；这是预期，不得为了让测试变绿去实现或放宽断言。

被测脚本不是包内模块，按路径加载（``importlib.util.spec_from_file_location``，与
``tests/test_verify_unit_consistency.py`` 同一写法）；另有用例用 ``subprocess`` 跑真实 CLI，
锁 ``__main__`` 路径与进程退出码。

锁的四件事：

- **健康检查四态**：``CALIB_KNOWN``(0) / ``CALIB_NEW_UNREGISTERED``(1，非阻断且提示
  ``tools/calib_register.py``) / ``CALIB_SUSPECT``(3) / ``CALIB_CORRUPT``(4)，
  每条都断言退出码数字，stdout 必须有 ``CALIB_STATE: <STATUS>`` 机器可读行。
- **损坏优先于可疑**：JSON 不可解析、缺字段、值为 ``None``、``range_min >= range_max``
  归 ``CALIB_CORRUPT``；可解析但结构校验不过（如非法 ``drive_mode``）归 ``CALIB_SUSPECT``。
- **登记工具默认 dry-run**：只打印 ``CALIB_ID: <id>`` 等结论、不写盘；``--write`` 幂等；
  结构不合格退出 1、文件不可解析退出 3。
- **用法错误退出 2**：``calib_health.py`` 既不给 ``--calib-file`` 也不给 ``--fingerprint``。

全部用例离线：只在 ``tmp_path`` 里造注册表与小样本，读真实 fixture 只为取一份标定原文。
"""

from __future__ import annotations

import copy
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from alohamini_cartesian.calibration.identity import (
    calib_id,
    canonicalize_motors,
    extract_motors,
    file_sha256,
)
from alohamini_cartesian.calibration.registry import CalibrationRegistry, GenerationRecord

_REPO_ROOT = Path(__file__).resolve().parents[1]
_HEALTH_PATH = _REPO_ROOT / "tools" / "calib_health.py"
_REGISTER_PATH = _REPO_ROOT / "tools" / "calib_register.py"

_FIXTURES = Path(__file__).resolve().parent / "fixtures" / "calibration"
OLD_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261022.mac-snapshot.json"
NEW_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261110.pi-live.json"


def _load_script(path: Path, module_name: str) -> ModuleType:
    """按路径加载被测脚本（它不是包内模块）；缺文件要响亮失败，不跳过。"""
    assert path.is_file(), f"被测脚本不存在：{path}（契约是冻的，缺文件要响亮失败，不跳过）"
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None, f"无法为 {path} 构造导入 spec"
    module = importlib.util.module_from_spec(spec)
    # ``dataclass`` 会按 ``cls.__module__`` 回查模块命名空间来解析字符串注解，
    # 所以先登记进 ``sys.modules`` 再执行。
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def health() -> ModuleType:
    """被测 ``tools/calib_health.py`` 的模块对象。"""
    return _load_script(_HEALTH_PATH, "calib_health")


@pytest.fixture(scope="module")
def register_tool() -> ModuleType:
    """被测 ``tools/calib_register.py`` 的模块对象。"""
    return _load_script(_REGISTER_PATH, "calib_register")


def _old_motors() -> dict[str, Any]:
    """旧代际那 18 台电机的标定字段（每次现读，用例之间不共享可变状态）。"""
    return extract_motors(json.loads(OLD_FIXTURE.read_text(encoding="utf-8")))


def _new_motors() -> dict[str, Any]:
    """新代际（2026-11-10 现场重标定）那 18 台电机的标定字段。"""
    return extract_motors(json.loads(NEW_FIXTURE.read_text(encoding="utf-8")))


def _fingerprint_document(motors: dict[str, Any]) -> dict[str, Any]:
    """把 5 字段标定电机映射包成最小 v2 指纹（补 ``model`` / ``normalization`` / ``resolution``）。

    三个指纹独有字段取常数，只为让文档形状合法；它们不参与标定身份。
    """
    return {
        "captured_utc": "2026-11-10T03:00:00Z",
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
    }


def _write_json(tmp_path: Path, name: str, payload: Any) -> Path:
    """把 payload 写成 ``tmp_path`` 下的 JSON 文件并返回路径。"""
    path = tmp_path / name
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _registry_with(tmp_path: Path, fixture: Path) -> Path:
    """在 ``tmp_path`` 里造一份登记了 ``fixture`` 那份代际的注册表，返回其路径。"""
    path = tmp_path / "calib-registry.yaml"
    motors = extract_motors(json.loads(fixture.read_text(encoding="utf-8")))
    registry = CalibrationRegistry.load(path)
    registry.append_if_absent(
        GenerationRecord(
            calib_id=calib_id(motors),
            first_seen="2026-10-22T01:00:00Z",
            source=fixture.name,
            file_sha256=file_sha256(fixture),
            byte_size=fixture.stat().st_size,
            motors=canonicalize_motors(motors),
            note=None,
            acknowledged_by=None,
        )
    )
    registry.save()
    return path


def _registry_with_the_old_generation(tmp_path: Path) -> Path:
    """在 ``tmp_path`` 里造一份登记了旧代际的注册表，返回其路径。"""
    return _registry_with(tmp_path, OLD_FIXTURE)


def test_a_registered_generation_is_known(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """登记的代际 → ``CALIB_STATE: CALIB_KNOWN``，退出码 0。"""
    registry_path = _registry_with_the_old_generation(tmp_path)

    code = health.main(["--calib-file", str(OLD_FIXTURE), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 0
    assert health.EXIT_CALIB_KNOWN == 0
    assert "CALIB_STATE: CALIB_KNOWN" in out


def test_an_unregistered_but_valid_generation_is_new_and_not_blocking(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """结构合法但不在表里 → ``CALIB_NEW_UNREGISTERED``，退出码 1，提示去用登记工具。"""
    registry_path = tmp_path / "calib-registry.yaml"  # 不存在 = 空表

    code = health.main(["--calib-file", str(OLD_FIXTURE), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 1
    assert health.EXIT_CALIB_NEW_UNREGISTERED == 1
    assert "CALIB_STATE: CALIB_NEW_UNREGISTERED" in out
    assert "tools/calib_register.py" in out
    # 新代际未登记是**非阻断**结论：文案不得吓人成事故，也不得要求停机。
    assert "非阻断" in out
    assert "事故" not in out
    assert "停" not in out


def test_a_broad_change_against_the_latest_generation_is_suspect(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """与已登记 ``latest()`` 相比 18 台全变 → ``CALIB_SUSPECT``，退出码 3。"""
    registry_path = _registry_with_the_old_generation(tmp_path)
    changed = copy.deepcopy(_old_motors())
    for entry in changed.values():
        entry["range_max"] = entry["range_max"] - 1
    candidate = _write_json(tmp_path, "candidate.json", changed)

    code = health.main(["--calib-file", str(candidate), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 3
    assert health.EXIT_CALIB_SUSPECT == 3
    assert "CALIB_STATE: CALIB_SUSPECT" in out


def test_a_structurally_invalid_but_parsable_document_is_suspect(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """JSON 能解析但结构校验不过（非法 ``drive_mode``）→ ``CALIB_SUSPECT``，退出码 3。"""
    registry_path = _registry_with_the_old_generation(tmp_path)
    changed = copy.deepcopy(_old_motors())
    changed["arm_left_shoulder_pan"]["drive_mode"] = 7
    candidate = _write_json(tmp_path, "bad-drive-mode.json", changed)

    code = health.main(["--calib-file", str(candidate), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 3
    assert "CALIB_STATE: CALIB_SUSPECT" in out


@pytest.mark.parametrize("case", ["broken-json", "missing-field", "none-value", "inverted-range"])
def test_a_corrupt_calibration_file_is_reported_as_corrupt(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str], case: str
) -> None:
    """四类损坏（语法坏 / 缺字段 / 值为 ``None`` / range 反向）都归 ``CALIB_CORRUPT``，退出码 4。"""
    registry_path = _registry_with_the_old_generation(tmp_path)
    path = tmp_path / "corrupt.json"
    if case == "broken-json":
        path.write_text("{not json", encoding="utf-8")
    else:
        broken = copy.deepcopy(_old_motors())
        if case == "missing-field":
            del broken["arm_left_elbow_flex"]["homing_offset"]
        elif case == "none-value":
            broken["arm_left_elbow_flex"]["range_min"] = None
        else:
            broken["arm_left_elbow_flex"]["range_max"] = broken["arm_left_elbow_flex"]["range_min"]
        path.write_text(json.dumps(broken), encoding="utf-8")

    code = health.main(["--calib-file", str(path), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 4
    assert health.EXIT_CALIB_CORRUPT == 4
    assert "CALIB_STATE: CALIB_CORRUPT" in out


def test_cli_through_subprocess_reports_the_exit_code(tmp_path: Path) -> None:
    """真实 CLI（``__main__`` 路径）把状态报成进程退出码 1，状态行进 stdout。"""
    proc = subprocess.run(
        [
            sys.executable,
            str(_HEALTH_PATH),
            "--calib-file",
            str(OLD_FIXTURE),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert proc.returncode == 1
    assert "CALIB_STATE: CALIB_NEW_UNREGISTERED" in proc.stdout


def test_cli_without_an_input_argument_exits_two(tmp_path: Path) -> None:
    """既没给 ``--calib-file`` 也没给 ``--fingerprint``：真实 CLI 按用法错误退出 2。"""
    proc = subprocess.run(
        [sys.executable, str(_HEALTH_PATH), "--registry", str(tmp_path / "calib-registry.yaml")],
        capture_output=True,
        text=True,
        check=False,
    )

    assert proc.returncode == 2


def test_register_dry_run_prints_the_calib_id_and_does_not_write(
    register_tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """默认 dry-run：打印 ``CALIB_ID: <id>``（id 与库算出的相同），登记表不被创建。"""
    registry_path = tmp_path / "calib-registry.yaml"

    code = register_tool.main(["--from-file", str(OLD_FIXTURE), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 0
    assert register_tool.EXIT_OK == 0
    assert f"CALIB_ID: {calib_id(_old_motors())}" in out
    assert not registry_path.exists()


def test_register_write_is_idempotent(
    register_tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``--write`` 两次：两次都退出 0，登记表里仍只有一条同 id 记录。"""
    registry_path = tmp_path / "calib-registry.yaml"
    argv = ["--from-file", str(OLD_FIXTURE), "--registry", str(registry_path), "--write"]

    first = register_tool.main(argv)
    capsys.readouterr()
    second = register_tool.main(argv)
    capsys.readouterr()

    registry = CalibrationRegistry.load(registry_path)
    assert (first, second) == (0, 0)
    assert len(registry.generations) == 1
    assert registry.find(calib_id(_old_motors())) is not None


def test_register_rejects_a_structurally_invalid_file(
    register_tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """结构不合格（电机只有 17 台）→ 退出 1（``EXIT_STRUCTURE``），不改登记表。"""
    registry_path = tmp_path / "calib-registry.yaml"
    short = copy.deepcopy(_old_motors())
    del short["lift_axis"]
    candidate = _write_json(tmp_path, "short.json", short)

    code = register_tool.main(["--from-file", str(candidate), "--registry", str(registry_path)])

    capsys.readouterr()
    assert code == 1
    assert register_tool.EXIT_STRUCTURE == 1
    assert not registry_path.exists()


def test_register_reports_an_unparsable_file(
    register_tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """输入不是合法 JSON → 退出 3（``EXIT_UNPARSABLE``）。"""
    registry_path = tmp_path / "calib-registry.yaml"
    broken = tmp_path / "broken.json"
    broken.write_text("{not json", encoding="utf-8")

    code = register_tool.main(["--from-file", str(broken), "--registry", str(registry_path)])

    capsys.readouterr()
    assert code == 3
    assert register_tool.EXIT_UNPARSABLE == 3


def test_a_fingerprint_and_its_calibration_file_yield_the_same_calib_id(tmp_path: Path) -> None:
    """端到端同 id：同一份数据分别以 v2 指纹与标定文件形状过 ``extract_motors`` + ``calib_id``。"""
    fingerprint_path = _write_json(
        tmp_path, "fingerprint.json", _fingerprint_document(_new_motors())
    )
    from_calibration = calib_id(_new_motors())
    from_fingerprint = calib_id(
        extract_motors(json.loads(fingerprint_path.read_text(encoding="utf-8")))
    )

    assert from_calibration == from_fingerprint, (
        f"同一份数据在指纹与标定文件两种形状下算出不同 calib_id："
        f"标定文件={from_calibration} 指纹={from_fingerprint}"
    )


def test_a_fingerprint_of_a_registered_generation_is_known(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``--fingerprint`` 吃 v2 指纹：同代际（按标定文件登记）已入表 → ``CALIB_KNOWN``，退出 0。"""
    registry_path = _registry_with(tmp_path, NEW_FIXTURE)
    fingerprint_path = _write_json(
        tmp_path, "fingerprint.json", _fingerprint_document(_new_motors())
    )

    code = health.main(["--fingerprint", str(fingerprint_path), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 0
    assert "CALIB_STATE: CALIB_KNOWN" in out


def test_register_dry_run_from_a_fingerprint_prints_the_same_calib_id(
    register_tool: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``--from-fingerprint`` dry-run：退出 0，``CALIB_ID:`` 与同一份数据标定文件算出的一致。"""
    registry_path = tmp_path / "calib-registry.yaml"
    fingerprint_path = _write_json(
        tmp_path, "fingerprint.json", _fingerprint_document(_new_motors())
    )

    code = register_tool.main(
        ["--from-fingerprint", str(fingerprint_path), "--registry", str(registry_path)]
    )

    out = capsys.readouterr().out
    assert code == 0
    assert f"CALIB_ID: {calib_id(_new_motors())}" in out
    assert not registry_path.exists()


def test_a_cross_generation_id_swap_is_suspect(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """跨代际把两台电机的 ``id`` 对调（换线）不可能是重标定结果：``CALIB_SUSPECT``、exit 3。

    这类变动会让 ``calib_id`` 变化、从而变成「跨代际」，与 ``registry.latest()`` 比出来的
    形态仍可能是 ``ARM_ONLY``；所以四态判定必须在「跨代际」这一支上单独看 ``id`` /
    ``drive_mode``，否则换线会落在 ``CALIB_NEW_UNREGISTERED``（非阻断）里没人拦。
    """
    registry_path = _registry_with_the_old_generation(tmp_path)
    swapped = copy.deepcopy(_old_motors())
    pan_id = swapped["arm_left_shoulder_pan"]["id"]
    elbow_id = swapped["arm_left_elbow_flex"]["id"]
    assert pan_id != elbow_id
    swapped["arm_left_shoulder_pan"]["id"] = elbow_id
    swapped["arm_left_elbow_flex"]["id"] = pan_id
    candidate = _write_json(tmp_path, "wire-swap.json", swapped)

    code = health.main(["--calib-file", str(candidate), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 3
    assert health.EXIT_CALIB_SUSPECT == 3
    assert "CALIB_STATE: CALIB_SUSPECT" in out
    assert "arm_left_shoulder_pan.id" in out
    assert "arm_left_elbow_flex.id" in out


def test_a_cross_generation_drive_mode_flip_is_suspect(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """跨代际把一台电机的 ``drive_mode`` 翻成 1 → ``CALIB_SUSPECT``、exit 3，点名该字段。"""
    registry_path = _registry_with_the_old_generation(tmp_path)
    flipped = copy.deepcopy(_old_motors())
    flipped["arm_right_wrist_flex"]["drive_mode"] = 1
    candidate = _write_json(tmp_path, "drive-mode-flip.json", flipped)

    code = health.main(["--calib-file", str(candidate), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 3
    assert health.EXIT_CALIB_SUSPECT == 3
    assert "CALIB_STATE: CALIB_SUSPECT" in out
    assert "arm_right_wrist_flex.drive_mode" in out


def test_a_cross_generation_homing_offset_change_is_still_new_unregistered(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """正向对照：跨代际只改 ``homing_offset``（``id``/``drive_mode`` 不动）→ 仍非阻断 exit 1。

    正常的重标定就是这种形态（14 台手臂电机的 homing/range 变、``id`` 与 ``drive_mode``
    不动）；把这条也判成可疑就等于把正常重标定也拦下来。
    """
    registry_path = _registry_with_the_old_generation(tmp_path)
    shifted = copy.deepcopy(_old_motors())
    shifted["arm_left_elbow_flex"]["homing_offset"] = (
        shifted["arm_left_elbow_flex"]["homing_offset"] + 25
    )
    candidate = _write_json(tmp_path, "homing-shift.json", shifted)

    code = health.main(["--calib-file", str(candidate), "--registry", str(registry_path)])

    out = capsys.readouterr().out
    assert code == 1
    assert health.EXIT_CALIB_NEW_UNREGISTERED == 1
    assert "CALIB_STATE: CALIB_NEW_UNREGISTERED" in out
    assert "CALIB_STATE: CALIB_SUSPECT" not in out
