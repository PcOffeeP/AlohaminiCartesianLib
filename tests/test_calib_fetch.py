"""``tools/calib_fetch.py``（标定抓取）与相关改造点的离线契约测试。

**这是离线契约测试，共 30 条，不是待实现的红阶段草稿。** 其中 29 条绿，1 条红：
``test_a_failed_overrides_write_is_blocked_without_a_traceback`` 钉的是「留痕写不出来 = 不可
继续（退出 ``4``）时不留下任何落盘文件」，而当前 ``tools/calib_fetch.py`` 先把快照写盘、
再试写留痕，于是退出 ``4`` 时快照已经落在盘上。该红项是待裁决的实现顺序问题（不是测试写错），
按真实契约保留，不放宽断言。其余被测对象都已落地：
``tools/calib_fetch.py``、接受名单 ``configs/calib-envelope-accepted.yaml``、
``CalibrationRegistry.append_if_absent``，以及 ``tools/calib_health.py`` 的
``--accepted`` / ``--session-anchor`` 两个参数。契约意图不变——断言先于实现，**缺实现文件
要响亮失败、不 ``skip``**：缺文件是「契约没有对象」，不是「可以忽略」。不得为了让测试变绿
去改产品代码迁就，也不得放宽断言。

锁的七件事（前六件是初版冻结项，第七件是本轮补齐的缺口用例）：

- **抓取脚本的机器可读行**：``CALIB_ID:`` / ``CALIB_SOURCE:``（``ssh`` 或 ``file``）/
  每台一行 ``CALIB_MOTOR: <motor> <verdict> <detail>`` / ``CALIB_ENVELOPE: <汇总>`` /
  ``CALIB_ENVELOPE_IMPLAUSIBLE: <台数>``，以及未声明不合格时的 ``continue_hint`` 提示行。
- **退出码**：``0`` 通过（含已用 ``--allow-implausible`` 留痕）；``1`` 有 ``IMPLAUSIBLE`` 且
  未声明理由（提醒、不阻断，仍写出快照）；``4`` 不可继续（``WRONG_DATASET`` / 结构自洽失败 /
  读不到远端文件 / 接受名单坏）。
- **ssh 失败不回落本地缓存**：非零退出、或 stderr 有内容而 stdout 空 → 直接退出 ``4``，
  且 ``--out`` 指定的文件**不存在**（不产生冒充标定的落盘文件）。
- **``--out`` 写原文**：``--ssh-binary`` 把 fixture ``cat`` 出来时，落盘内容与 fixture 字节相同。
- **``--dry-run`` 不执行、不落盘**：退出 ``0``，stdout 打出将要执行的 ssh 命令（含远端路径）。
- **登记与健康检查**：``--record`` 幂等（同一 ``calib_id`` 只一条）；``calib_health.py`` 的
  ``--accepted`` / ``--session-anchor`` 只追加包络相关行，``CALIB_STATE`` 与既有退出码不变，
  ``--session-anchor`` 不符时打 ``SESSION_ANCHOR: MISMATCH`` + ``CALIB_STATE: CALIB_SESSION_DRIFT``
  并退出 ``4``。
- **本轮补齐的缺口（每条断言具体文案，不只断「有输出」或「未发生」）**：ssh stdout 不是合法
  JSON → 退出 ``4`` +「输入不是合法 JSON」，且 ``--out`` 路径不存在；``--out`` 所在目录不可
  写 → 退出 ``4`` +「快照写不了」+ 具体路径字样；``--from-file --dry-run`` → 退出 ``0`` +
  ``DRY-RUN`` 行含本地输入路径且不落盘；结构自洽失败 →「结构自洽失败，不可继续」+
  「未产生任何标定落盘文件」+ ``--out`` 不存在；接受名单坏 →「接受名单不可用」+ 缺的字段名 +
  「未产生任何标定落盘文件」；``calib_health.py`` 拿到坏名单只跳过包络行（``CALIB_STATE``、
  退出码不变，stdout 无 ``CALIB_MOTOR:``）；留痕写不出来 → 退出 ``4``（**不是** ``1``）+
  「留痕文件写不出来」+ 输出里无 ``Traceback``（快照不落盘那一条就是上面提到的红项）；
  ``--kind teleop`` 文案点「遥操侧（leader）」；``--user`` 在 dry-run 打印的 ssh 命令里落成
  ``-l <user>`` 且带 ``BatchMode`` / ``ConnectTimeout`` 与默认远端路径；``calib_health.py``
  不传 ``--accepted`` 也打包络行；``WRONG_DATASET`` 门禁带 ``--out`` 也不落盘。

全程离线：不连设备、不联网、不用真实 ssh（假程序是 ``tmp_path`` 里 ``#!/bin/sh`` 的脚本）。
"""

from __future__ import annotations

import copy
import importlib
import importlib.util
import json
import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
import yaml

from alohamini_cartesian.calibration.identity import (
    calib_id,
    canonicalize_motors,
    extract_motors,
    file_sha256,
)
from alohamini_cartesian.calibration.registry import CalibrationRegistry, GenerationRecord

_REPO_ROOT = Path(__file__).resolve().parents[1]
_FETCH_PATH = _REPO_ROOT / "tools" / "calib_fetch.py"
_HEALTH_PATH = _REPO_ROOT / "tools" / "calib_health.py"
_SHIPPED_ACCEPTED = _REPO_ROOT / "configs" / "calib-envelope-accepted.yaml"
_FIXTURES = Path(__file__).resolve().parent / "fixtures" / "calibration"
_GEN_A = _FIXTURES / "AlohaMiniRobot.20261022.mac-snapshot.json"
_GEN_B = _FIXTURES / "AlohaMiniRobot.20261110.pi-live.json"

_REMOTE_PATH = (
    "/home/pi5/.cache/huggingface/lerobot/calibration/robots/alohamini/AlohaMiniRobot.json"
)

# 合成表的各族 range：宽度都落在经验区间内，中心都压归零位 2047。
_PLAUSIBLE_RANGES: dict[str, tuple[int, int]] = {
    "shoulder_pan": (852, 3242),  # 2390 tick ≈ 210.0°
    "shoulder_lift": (852, 3242),
    "elbow_flex": (900, 3100),  # 不属于任何宽度族
    "wrist_flex": (1080, 3014),  # 1934 tick ≈ 170.0°
    "wrist_yaw": (1080, 3014),
    "wrist_roll": (0, 4095),  # 满圈占位
    "gripper": (1319, 2775),  # 1456 tick ≈ 128.0°
}


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
def fetch() -> ModuleType:
    """被测 ``tools/calib_fetch.py`` 的模块对象。"""
    return _load_script(_FETCH_PATH, "calib_fetch_under_test")


@pytest.fixture(scope="module")
def health() -> ModuleType:
    """被测 ``tools/calib_health.py`` 的模块对象（本文件只用它的新参数）。"""
    return _load_script(_HEALTH_PATH, "calib_health_under_test")


def _plausible_motors() -> dict[str, Any]:
    """合成一份 18 台、判定全落在经验区间内的电机表（底盘与升降轴是满圈占位）。"""
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


def _gripper_51_tick_motors() -> dict[str, Any]:
    """合成表 + 左夹爪换成 2021..2072（51 tick ≈ 4.5°，已知的左夹爪异常形态）。"""
    motors = _plausible_motors()
    motors["arm_left_gripper"]["range_min"] = 2021
    motors["arm_left_gripper"]["range_max"] = 2072
    return motors


def _teleop_only_motors() -> dict[str, Any]:
    """遥操式「只有手臂」电机表：14 台 ``arm_*``，没有任何 ``base_*`` 与 ``lift_axis``。"""
    return {name: entry for name, entry in _plausible_motors().items() if name.startswith("arm_")}


def _arm_only_full_table() -> dict[str, Any]:
    """18 台全 ``arm_*``（无 ``base_*``、无 ``lift_axis``）：结构自洽通过，但数据集形状是遥操臂。

    ``_teleop_only_motors()`` 只有 14 台，会先在结构校验（期望 18 台）被拦下，根本进不到
    ``WRONG_DATASET`` 那道门；要覆盖「数据集形状门禁带 ``--out`` 也不落盘」必须用这张 18 台的表。
    把 4 台地面电机（3 个轮子 + 升降轴）改名成 ``arm_*``，字段取值不变。
    """
    ground_plane_renames = {
        "base_left_wheel": "arm_left_wheel",
        "base_back_wheel": "arm_back_wheel",
        "base_right_wheel": "arm_right_wheel",
        "lift_axis": "arm_extra_axis",
    }
    motors: dict[str, Any] = {}
    for name, entry in _plausible_motors().items():
        motors[ground_plane_renames.get(name, name)] = entry
    return motors


def _write_json(path: Path, payload: Any) -> Path:
    """把 payload 写成 JSON 文件并返回路径。"""
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _write_empty_accepted(path: Path) -> Path:
    """写一份空接受名单（``accepted: []``）：用例默认用它，不读仓库内那份。"""
    path.write_text("schema_version: 1\naccepted: []\n", encoding="utf-8")
    return path


def _write_accepted(path: Path, entries: Sequence[dict[str, Any]]) -> Path:
    """写一份带条目的接受名单。"""
    path.write_text(
        yaml.safe_dump(
            {"schema_version": 1, "accepted": list(entries)},
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
        "calib_id": calib_id(motors),
        "reason": "左夹爪实物行程只有 51 tick，经验区间不适用于夹爪机械限位",
    }
    entry.update(overrides)
    return entry


def _write_fake_ssh(tmp_path: Path, body: str) -> Path:
    """在 ``tmp_path`` 里造一个假 ``ssh``（``#!/bin/sh`` 脚本，可执行）。"""
    path = tmp_path / "fake-ssh.sh"
    path.write_text("#!/bin/sh\n" + body, encoding="utf-8")
    path.chmod(0o755)
    return path


def _run_cli(*args: str) -> subprocess.CompletedProcess[str]:
    """用当前解释器跑真实 CLI，返回进程结果（``__main__`` 路径与退出码都在断言范围内）。"""
    assert _FETCH_PATH.is_file(), (
        f"被测脚本不存在：{_FETCH_PATH}（红阶段预期：tools/calib_fetch.py 尚未实现）"
    )
    return subprocess.run(
        [sys.executable, str(_FETCH_PATH), *args],
        capture_output=True,
        text=True,
        check=False,
        cwd=_REPO_ROOT,
    )


def _registry_with(tmp_path: Path, fixture: Path, name: str = "calib-registry.yaml") -> Path:
    """在 ``tmp_path`` 里造一份登记了 ``fixture`` 那份代际的登记表，返回其路径。"""
    path = tmp_path / name
    motors = extract_motors(json.loads(fixture.read_text(encoding="utf-8")))
    registry = CalibrationRegistry.load(path)
    registry.register(
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


def _fixture_motors(path: Path) -> dict[str, Any]:
    """读一份真机 fixture，取出「电机名 → 五字段」映射。"""
    return extract_motors(json.loads(path.read_text(encoding="utf-8")))


# ---------------------------------------------------------------- --from-file：两个真机代际


def test_from_file_generation_a_reports_the_gripper_and_exits_one(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """代际 A 走 ``--from-file``：``CALIB_MOTOR: arm_left_gripper IMPLAUSIBLE``，退出码 1。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    registry = tmp_path / "calib-registry.yaml"

    code = fetch.main(
        [
            "--from-file",
            str(_GEN_A),
            "--accepted",
            str(accepted),
            "--registry",
            str(registry),
            "--out",
            str(tmp_path / "snapshot.json"),
        ]
    )

    out = capsys.readouterr().out
    assert code == 1
    assert fetch.EXIT_IMPLAUSIBLE == 1
    assert f"CALIB_ID: {calib_id(_fixture_motors(_GEN_A))}" in out
    assert "CALIB_SOURCE: file" in out
    assert "CALIB_MOTOR: arm_left_gripper IMPLAUSIBLE" in out
    assert "CALIB_ENVELOPE: IMPLAUSIBLE" in out
    assert "CALIB_ENVELOPE_IMPLAUSIBLE: 1" in out
    # 未声明理由时要有可继续的提示行，且点出开关本身。
    assert "--allow-implausible" in out
    # 提醒、不阻断：快照照样落盘。
    assert (tmp_path / "snapshot.json").is_file()


def test_from_file_generation_b_reports_the_right_shoulder_lift_and_exits_one(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """代际 B 走 ``--from-file``：``CALIB_MOTOR: arm_right_shoulder_lift IMPLAUSIBLE``。

    右肩 lift 只有 1143 tick ≈ 100.4°，落在肩的 190–240° 之外，退出码 1。
    """
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")

    code = fetch.main(
        [
            "--from-file",
            str(_GEN_B),
            "--accepted",
            str(accepted),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
        ]
    )

    out = capsys.readouterr().out
    assert code == 1
    assert "CALIB_MOTOR: arm_right_shoulder_lift IMPLAUSIBLE" in out
    assert "CALIB_ENVELOPE_IMPLAUSIBLE: 1" in out
    assert "CALIB_ENVELOPE: IMPLAUSIBLE" in out


@pytest.mark.parametrize("case", ["entry", "subprocess"])
def test_a_constructed_51_tick_gripper_file_exits_one(
    fetch: ModuleType,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    case: str,
) -> None:
    """构造 51 tick 左夹爪的标定文件 → ``IMPLAUSIBLE``、退出码 1（模块内与真实 CLI 各一遍）。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    candidate = _write_json(tmp_path / "candidate.json", _gripper_51_tick_motors())
    argv = [
        "--from-file",
        str(candidate),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
    ]

    if case == "entry":
        code = fetch.main(argv)
        out = capsys.readouterr().out
    else:
        proc = _run_cli(*argv)
        code, out = proc.returncode, proc.stdout

    assert code == 1
    assert "CALIB_MOTOR: arm_left_gripper IMPLAUSIBLE" in out
    assert "CALIB_ENVELOPE: IMPLAUSIBLE" in out


# ---------------------------------------------------------------- 不可继续的两种形态（退出码 4）


def test_an_arm_only_remote_dataset_exits_four_with_the_teleop_wording(tmp_path: Path) -> None:
    """只有手臂关节的遥操式数据 → 退出码 4，文案含「遥操臂」（真实 CLI 子进程）。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    candidate = _write_json(tmp_path / "teleop.json", _teleop_only_motors())

    proc = _run_cli(
        "--from-file",
        str(candidate),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
    )

    assert proc.returncode == 4
    assert "遥操臂" in proc.stdout + proc.stderr


def test_an_inverted_range_exits_four_and_names_the_motor(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``range_min >= range_max`` 属结构自洽失败：退出码 4，文案点名该电机且声明不落盘。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    broken = copy.deepcopy(_plausible_motors())
    broken["arm_left_shoulder_pan"]["range_max"] = broken["arm_left_shoulder_pan"]["range_min"]
    candidate = _write_json(tmp_path / "inverted.json", broken)
    out_path = tmp_path / "snapshot.json"

    code = fetch.main(
        [
            "--from-file",
            str(candidate),
            "--accepted",
            str(accepted),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
            "--out",
            str(out_path),
        ]
    )

    captured = capsys.readouterr()
    assert code == 4
    assert fetch.EXIT_BLOCKED == 4
    assert "arm_left_shoulder_pan" in captured.out + captured.err
    # 两句具体文案：「结构自洽失败」必须点名不可继续，且「未产生任何标定落盘文件」给结论。
    assert "结构自洽失败，不可继续" in captured.err
    assert "未产生任何标定落盘文件" in captured.err
    assert not out_path.exists()


# ---------------------------------------------------------------- --allow-implausible 与 overrides


def test_allow_implausible_exits_zero_and_writes_the_overrides_file(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``--allow-implausible "理由"``：退出码 0，overrides 落盘。

    默认文件名与 ``--out`` 同目录、形如 ``overrides-<out 的 stem>.json``。
    """
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    reason = "左夹爪 51 tick 是已知实物行程，已人工核对"

    code = fetch.main(
        [
            "--from-file",
            str(_GEN_A),
            "--accepted",
            str(accepted),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
            "--out",
            str(tmp_path / "snapshot.json"),
            "--allow-implausible",
            reason,
        ]
    )

    capsys.readouterr()
    overrides = tmp_path / "overrides-snapshot.json"
    assert code == 0
    assert fetch.EXIT_OK == 0
    assert overrides.is_file()
    text = overrides.read_text(encoding="utf-8")
    assert reason in text
    assert calib_id(_fixture_motors(_GEN_A)) in text
    assert "arm_left_gripper" in text
    assert isinstance(json.loads(text), (dict, list))


# ---------------------------------------------------------------- 接受名单（命中 / 过期 / 坏）


def test_an_accepted_entry_silences_the_count_but_keeps_the_information_line(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """名单命中（登记值 = 实测值）→ 不合格台数归 0、退出码 0，但 stdout 仍有「已接受」信息行。"""
    motors = _gripper_51_tick_motors()
    accepted = _write_accepted(tmp_path / "accepted.yaml", [_accepted_entry(motors)])
    candidate = _write_json(tmp_path / "candidate.json", motors)

    code = fetch.main(
        [
            "--from-file",
            str(candidate),
            "--accepted",
            str(accepted),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
        ]
    )

    out = capsys.readouterr().out
    assert code == 0
    assert "CALIB_ENVELOPE_IMPLAUSIBLE: 0" in out
    assert "CALIB_ENVELOPE: OK" in out
    assert "已接受" in out


def test_a_stale_accepted_entry_warns_again_with_the_stale_wording(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """名单登记值与实测不符 → 重新报警（退出码 1），文案含「名单过期」。"""
    motors = _gripper_51_tick_motors()
    stale = _accepted_entry(motors, observed_range_max=2073)
    accepted = _write_accepted(tmp_path / "accepted.yaml", [stale])
    candidate = _write_json(tmp_path / "candidate.json", motors)

    code = fetch.main(
        [
            "--from-file",
            str(candidate),
            "--accepted",
            str(accepted),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
        ]
    )

    out = capsys.readouterr().out
    assert code == 1
    assert "CALIB_ENVELOPE_IMPLAUSIBLE: 1" in out
    assert "名单过期" in out


def test_a_broken_accepted_list_exits_four(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """接受名单坏（条目缺字段）→ 退出码 4（不可继续），文案点名缺的字段与不落盘。"""
    broken_entry = _accepted_entry(_gripper_51_tick_motors())
    del broken_entry["method"]
    accepted = _write_accepted(tmp_path / "accepted.yaml", [broken_entry])

    code = fetch.main(
        [
            "--from-file",
            str(_GEN_A),
            "--accepted",
            str(accepted),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
        ]
    )

    captured = capsys.readouterr()
    assert code == 4
    # 具体文案：「接受名单不可用」定调不可继续，缺的字段名要点出来，且声明未落盘。
    assert "接受名单不可用" in captured.err
    assert "缺必需字段 method" in captured.err
    assert "未产生任何标定落盘文件" in captured.err
    assert "原因：接受名单不可用" in captured.err


def test_the_shipped_accepted_list_is_an_empty_template() -> None:
    """交付的 ``configs/calib-envelope-accepted.yaml`` 是空模板：条目为 0、不点名任何电机。"""
    envelope_path = _REPO_ROOT / "src" / "alohamini_cartesian" / "calibration" / "envelope.py"
    assert envelope_path.is_file(), (
        f"缺少实现文件：{envelope_path}（红阶段预期：envelope 模块尚未实现，不跳过）"
    )
    envelope = importlib.import_module("alohamini_cartesian.calibration.envelope")

    accepted = envelope.load_accepted(_SHIPPED_ACCEPTED)

    entries = getattr(accepted, "entries", None)
    if entries is None:
        entries = getattr(accepted, "accepted", None)
    if entries is None:
        entries = tuple(accepted)
    assert _SHIPPED_ACCEPTED.is_file()
    assert tuple(entries) == ()
    assert "arm_right_shoulder_lift" not in _SHIPPED_ACCEPTED.read_text(encoding="utf-8")


# ---------------------------------------------------------------- ssh 路径（假程序注入）


def test_ssh_failure_with_a_non_zero_exit_does_not_write_a_snapshot(tmp_path: Path) -> None:
    """假 ssh 恒 ``exit 1`` → 退出码 4，且 ``--out`` 指定的快照文件不存在（不冒充标定）。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    fake = _write_fake_ssh(tmp_path, "exit 1\n")
    out_path = tmp_path / "snapshot.json"

    proc = _run_cli(
        "--host",
        "pi5",
        "--ssh-binary",
        str(fake),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
        "--out",
        str(out_path),
    )

    assert proc.returncode == 4
    assert not out_path.exists()
    assert proc.stdout.strip() != "" or proc.stderr.strip() != ""


def test_ssh_failure_with_stderr_and_no_stdout_does_not_write_a_snapshot(tmp_path: Path) -> None:
    """假 ssh 退出 0 但只有 stderr、stdout 空（实际失败形态）→ 仍退出码 4，不落盘。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    fake = _write_fake_ssh(tmp_path, 'echo "Permission denied (publickey)." >&2\nexit 0\n')
    out_path = tmp_path / "snapshot.json"

    proc = _run_cli(
        "--host",
        "pi5",
        "--ssh-binary",
        str(fake),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
        "--out",
        str(out_path),
    )

    assert proc.returncode == 4
    assert not out_path.exists()


def test_a_working_ssh_matches_the_from_file_run_and_writes_the_same_bytes(
    tmp_path: Path,
) -> None:
    """假 ssh ``cat`` 出 fixture：退出码与 ``--from-file`` 一致，``--out`` 字节与 fixture 相同。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    fake = _write_fake_ssh(tmp_path, f"cat {_GEN_A}\n")
    out_path = tmp_path / "ssh-snapshot.json"

    from_file = _run_cli(
        "--from-file",
        str(_GEN_A),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "from-file-registry.yaml"),
    )
    proc = _run_cli(
        "--host",
        "pi5",
        "--ssh-binary",
        str(fake),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "ssh-registry.yaml"),
        "--out",
        str(out_path),
    )

    assert proc.returncode == from_file.returncode
    assert proc.returncode == 1
    assert "CALIB_SOURCE: ssh" in proc.stdout
    assert f"CALIB_ID: {calib_id(_fixture_motors(_GEN_A))}" in proc.stdout
    assert out_path.read_bytes() == _GEN_A.read_bytes()


def test_dry_run_prints_the_ssh_command_and_writes_nothing(tmp_path: Path) -> None:
    """``--dry-run``：退出码 0、不落盘、stdout 含将要执行的 ssh 命令。

    命令里必须能看到 ``--remote-path`` 传进去的那个远端路径。
    """
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    fake = _write_fake_ssh(tmp_path, "exit 1\n")
    out_path = tmp_path / "snapshot.json"

    proc = _run_cli(
        "--host",
        "pi5",
        "--ssh-binary",
        str(fake),
        "--remote-path",
        _REMOTE_PATH,
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
        "--out",
        str(out_path),
        "--dry-run",
    )

    assert proc.returncode == 0
    assert not out_path.exists()
    assert _REMOTE_PATH in proc.stdout
    assert "ssh" in proc.stdout


# ---------------------------------------------------------------- --record 与登记表幂等


def test_record_is_idempotent_for_the_same_calibration(
    fetch: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """同一份标定连续两次 ``--record``：登记表里该 ``calib_id`` 只有一条记录。"""
    motors = _plausible_motors()
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    registry = tmp_path / "calib-registry.yaml"
    candidate = _write_json(tmp_path / "candidate.json", motors)
    argv = [
        "--from-file",
        str(candidate),
        "--accepted",
        str(accepted),
        "--registry",
        str(registry),
        "--record",
    ]

    first = fetch.main(argv)
    capsys.readouterr()
    second = fetch.main(argv)
    capsys.readouterr()

    loaded = CalibrationRegistry.load(registry)
    assert (first, second) == (0, 0)
    assert len(loaded.generations) == 1
    assert loaded.find(calib_id(motors)) is not None


def test_registry_append_if_absent_reports_false_for_a_duplicate(tmp_path: Path) -> None:
    """``CalibrationRegistry.append_if_absent``：同 ``calib_id`` 第二次返回 ``False``。

    而且不重复写：落盘后重读，仍然只有一条记录。
    """
    motors = _plausible_motors()
    registry = CalibrationRegistry.load(tmp_path / "calib-registry.yaml")
    record = GenerationRecord(
        calib_id=calib_id(motors),
        first_seen="2026-11-11T00:00:00Z",
        source="test",
        file_sha256="0" * 64,
        byte_size=1,
        motors=canonicalize_motors(motors),
        note=None,
        acknowledged_by=None,
    )

    first = registry.append_if_absent(record)
    second = registry.append_if_absent(record)

    assert (first, second) == (True, False)
    registry.save()
    reloaded = CalibrationRegistry.load(tmp_path / "calib-registry.yaml")
    assert len(reloaded.generations) == 1
    assert reloaded.find(calib_id(motors)) is not None


# ---------------------------------------------------------------- calib_health.py 的两个新参数


def test_session_anchor_mismatch_is_a_fifth_state_with_exit_four(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``--session-anchor`` 与文件里的 ``calib_id`` 不符 → 退出码 4。

    stdout 必须同时有 ``SESSION_ANCHOR: MISMATCH`` 与第五个状态 ``CALIB_SESSION_DRIFT``。
    """
    registry = _registry_with(tmp_path, _GEN_A)

    code = health.main(
        [
            "--calib-file",
            str(_GEN_A),
            "--registry",
            str(registry),
            "--session-anchor",
            "ffffffffffff",
        ]
    )

    out = capsys.readouterr().out
    assert code == 4
    assert "SESSION_ANCHOR: MISMATCH" in out
    assert "ffffffffffff" in out
    assert "CALIB_STATE: CALIB_SESSION_DRIFT" in out


def test_session_anchor_that_matches_keeps_the_plain_conclusion(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """锚点与文件一致时结论不变：退出码与 ``CALIB_STATE`` 行都与不带该参数时相同。"""
    registry = _registry_with(tmp_path, _GEN_A)
    anchor = calib_id(_fixture_motors(_GEN_A))

    baseline = health.main(["--calib-file", str(_GEN_A), "--registry", str(registry)])
    baseline_out = capsys.readouterr().out
    with_anchor = health.main(
        ["--calib-file", str(_GEN_A), "--registry", str(registry), "--session-anchor", anchor]
    )
    out = capsys.readouterr().out

    assert with_anchor == baseline
    assert "CALIB_STATE: CALIB_KNOWN" in out
    assert "CALIB_STATE: CALIB_KNOWN" in baseline_out
    assert "CALIB_SESSION_DRIFT" not in out


def test_accepted_option_appends_envelope_lines_without_changing_the_state(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``--accepted`` 只追加包络行：``CALIB_STATE`` 与退出码与不带该参数时完全一致。"""
    registry = _registry_with(tmp_path, _GEN_A)
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")

    baseline = health.main(["--calib-file", str(_GEN_A), "--registry", str(registry)])
    capsys.readouterr()
    with_accepted = health.main(
        ["--calib-file", str(_GEN_A), "--registry", str(registry), "--accepted", str(accepted)]
    )
    out = capsys.readouterr().out

    assert with_accepted == baseline == 0
    assert "CALIB_STATE: CALIB_KNOWN" in out
    assert "CALIB_MOTOR: arm_left_gripper IMPLAUSIBLE" in out
    assert "CALIB_ENVELOPE: IMPLAUSIBLE" in out


# ---------------------------------------------------------------- 本轮补齐：ssh / --out / dry-run


def test_ssh_stdout_that_is_not_json_exits_four_and_writes_nothing(tmp_path: Path) -> None:
    """假 ssh 吐 ``not json{``：退出码 4，stderr 点名「输入不是合法 JSON」，``--out`` 不存在。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    fake = _write_fake_ssh(tmp_path, "printf 'not json{'\n")
    out_path = tmp_path / "snapshot.json"

    proc = _run_cli(
        "--host",
        "pi5",
        "--ssh-binary",
        str(fake),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
        "--out",
        str(out_path),
    )

    assert proc.returncode == 4
    # 具体文案：点名这是 JSON 解析问题，而不是「形状认不出」或「ssh 失败」。
    assert "输入不是合法 JSON" in proc.stderr
    assert "未产生标定落盘文件" in proc.stderr
    assert not out_path.exists()


def test_an_unwritable_out_directory_exits_four_and_names_the_path(tmp_path: Path) -> None:
    """``--out`` 指到只读目录：退出码 4，stderr 点名「快照写不了」与具体路径，且不产生该文件。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    readonly_dir = tmp_path / "readonly"
    readonly_dir.mkdir()
    out_path = readonly_dir / "snapshot.json"

    readonly_dir.chmod(0o500)
    try:
        proc = _run_cli(
            "--from-file",
            str(_GEN_A),
            "--accepted",
            str(accepted),
            "--registry",
            str(tmp_path / "calib-registry.yaml"),
            "--out",
            str(out_path),
        )
    finally:
        # 恢复可写，否则 pytest 的 tmp_path 清理会失败。
        readonly_dir.chmod(0o700)

    assert proc.returncode == 4
    assert "快照写不了" in proc.stderr
    assert str(out_path) in proc.stderr
    assert not out_path.exists()


def test_dry_run_with_from_file_prints_the_local_path_and_writes_nothing(tmp_path: Path) -> None:
    """``--from-file --dry-run``：退出码 0、不落盘，stdout 的 ``DRY-RUN`` 行含本地输入路径。"""
    out_path = tmp_path / "snapshot.json"

    proc = _run_cli("--from-file", str(_GEN_A), "--out", str(out_path), "--dry-run")

    assert proc.returncode == 0
    assert not out_path.exists()
    assert "DRY-RUN" in proc.stdout
    assert str(_GEN_A) in proc.stdout
    assert "不执行 ssh、不落盘" in proc.stdout


def test_kind_teleop_is_blocked_with_the_teleop_side_wording(tmp_path: Path) -> None:
    """``--kind teleop`` 取代际 B 的整机 fixture：退出码 4，文案点「遥操侧（leader）」且不落盘。"""
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    out_path = tmp_path / "snapshot.json"

    proc = _run_cli(
        "--from-file",
        str(_GEN_B),
        "--kind",
        "teleop",
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
        "--out",
        str(out_path),
    )

    assert proc.returncode == 4
    assert "遥操侧（leader）" in proc.stderr
    assert "不能用于下发" in proc.stderr
    assert "未产生任何标定落盘文件" in proc.stderr
    assert not out_path.exists()


def test_dry_run_with_a_custom_user_keeps_the_ssh_command_copyable() -> None:
    """``--user tester --dry-run``：退出码 0，ssh 命令含 ``-l tester``、BatchMode 与远端路径。"""
    proc = _run_cli("--user", "tester", "--dry-run", "--host", "example.invalid")

    assert proc.returncode == 0
    assert "DRY-RUN: 将要执行的 ssh 命令" in proc.stdout
    assert "-l tester" in proc.stdout
    assert "BatchMode=yes" in proc.stdout
    assert "ConnectTimeout=" in proc.stdout
    assert _REMOTE_PATH in proc.stdout
    assert "未执行、未落盘" in proc.stdout


# ---------------------------------------------------------------- 本轮补齐：不可继续时不落盘


def test_a_wrong_dataset_gate_leaves_no_snapshot_even_with_out(tmp_path: Path) -> None:
    """``WRONG_DATASET``（18 台全手臂、无底盘与升降轴）+ ``--out``：退出码 4 且 ``--out`` 不存在。

    用 18 台的表而不是 ``_teleop_only_motors()``：14 台会先在结构校验被拦下，进不到这道门。
    """
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    candidate = _write_json(tmp_path / "arm-only-18.json", _arm_only_full_table())
    out_path = tmp_path / "snapshot.json"

    proc = _run_cli(
        "--from-file",
        str(candidate),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
        "--out",
        str(out_path),
    )

    assert proc.returncode == 4
    assert "CALIB_ENVELOPE: WRONG_DATASET" in proc.stdout
    assert "数据集形状不对" in proc.stderr
    assert "未产生任何标定落盘文件" in proc.stderr
    assert "原因：数据集形状不对：WRONG_DATASET" in proc.stderr
    assert not out_path.exists()


def test_a_failed_overrides_write_is_blocked_without_a_traceback(tmp_path: Path) -> None:
    """``--allow-implausible`` 但留痕路径被同名目录占位：退出码 4（不是 1），无 ``Traceback``。

    留痕写不出来就不放行——这一条锁的是「留痕失败 = 不可继续」，以及它不抛未捕获异常。
    """
    accepted = _write_empty_accepted(tmp_path / "accepted.yaml")
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    # 同名目录占位：overrides-<out stem>.json 写不进去（写文件写到目录上）。
    (out_dir / "overrides-snapshot.json").mkdir()
    out_path = out_dir / "snapshot.json"

    proc = _run_cli(
        "--from-file",
        str(_GEN_A),
        "--accepted",
        str(accepted),
        "--registry",
        str(tmp_path / "calib-registry.yaml"),
        "--out",
        str(out_path),
        "--allow-implausible",
        "左夹爪 51 tick 是已知实物行程，已人工核对",
    )
    combined = proc.stdout + proc.stderr

    assert proc.returncode == 4
    assert proc.returncode != 1
    assert "留痕文件写不出来" in proc.stderr
    assert "没有留痕就不放行" in proc.stderr
    assert "Traceback" not in combined
    # 契约：不可继续（4）的路径不留下任何看着像标定的落盘文件。
    assert not out_path.exists()


# ---------------------------------------------------------------- 本轮补齐：calib_health.py


def test_a_broken_accepted_list_on_health_only_skips_the_envelope_lines(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``calib_health.py --accepted`` 指向坏名单：``CALIB_STATE`` 与退出码不变，只跳过包络行。"""
    registry = _registry_with(tmp_path, _GEN_A)
    broken_entry = _accepted_entry(_fixture_motors(_GEN_A))
    del broken_entry["method"]
    accepted = _write_accepted(tmp_path / "accepted.yaml", [broken_entry])

    code = health.main(
        ["--calib-file", str(_GEN_A), "--registry", str(registry), "--accepted", str(accepted)]
    )
    captured = capsys.readouterr()

    assert code == 0
    assert "CALIB_STATE: CALIB_KNOWN" in captured.out
    assert "CALIB_MOTOR:" not in captured.out
    assert "接受名单不可读" in captured.err
    assert "包络判定跳过；CALIB_STATE 与退出码不受影响。" in captured.err


def test_health_without_explicit_accepted_still_prints_the_envelope_lines(
    health: ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """不传 ``--accepted`` 也跑包络判定：包络行照打，状态与退出码与传空模板时一致。"""
    registry = _registry_with(tmp_path, _GEN_A)
    empty = _write_empty_accepted(tmp_path / "accepted.yaml")

    default_code = health.main(["--calib-file", str(_GEN_A), "--registry", str(registry)])
    default_out = capsys.readouterr().out
    with_empty_code = health.main(
        ["--calib-file", str(_GEN_A), "--registry", str(registry), "--accepted", str(empty)]
    )
    with_empty_out = capsys.readouterr().out

    assert default_code == with_empty_code == 0
    assert "CALIB_STATE: CALIB_KNOWN" in default_out
    assert "CALIB_MOTOR: arm_left_gripper IMPLAUSIBLE" in default_out
    assert "CALIB_ENVELOPE: IMPLAUSIBLE" in default_out
    # 默认名单是空模板，所以 CALIB_* 行与显式传空模板逐行相同。
    assert [line for line in default_out.splitlines() if line.startswith("CALIB_")] == [
        line for line in with_empty_out.splitlines() if line.startswith("CALIB_")
    ]
