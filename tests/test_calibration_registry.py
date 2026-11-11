"""``alohamini_cartesian.calibration.registry`` 的离线契约测试（TDD 红阶段）。

被测模块尚未实现，本文件按任务书冻结的契约写断言，运行必然失败（``ModuleNotFoundError``）；
这是预期，不得为了让测试变绿而放宽断言或加软着陆。

锁的四件事：

- **登记幂等**：同一个 ``calib_id`` 登记两次 → 第一次 ``True``、第二次 ``False``，
  ``generations`` 长度不变，已登记记录不被覆盖。
- **查询语义**：``find`` 命中 / 未命中、``latest`` 按登记顺序取最后一条、空表 ``latest()``
  返回 ``None``、``load`` 一个不存在的路径得到空表且不报错。
- **存取往返**：``register`` + ``save`` + ``load`` 之后 18 台电机 × 5 个标定字段无损，
  ``note`` / ``source`` / ``file_sha256`` / ``byte_size`` 等溯源字段一并保住。
- **登记表文件是 YAML**：用 ``yaml.safe_load`` 读得回同一批记录。

全部用例离线：只在 ``tmp_path`` 里造登记表，读真实 fixture 只为取一份标定原文。
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from typing import Any

import yaml

from alohamini_cartesian.calibration.identity import (
    CANONICAL_FIELDS,
    EXPECTED_MOTOR_COUNT,
    calib_id,
    canonicalize_motors,
    extract_motors,
    file_sha256,
)
from alohamini_cartesian.calibration.registry import CalibrationRegistry, GenerationRecord

_REPO_ROOT = Path(__file__).resolve().parents[1]
# 仓库里真实在用的登记表：它的每一条都必须能被库函数重算对上。
SHIPPED_REGISTRY = _REPO_ROOT / "configs" / "calib-registry.yaml"

_FIXTURES = Path(__file__).resolve().parent / "fixtures" / "calibration"
OLD_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261022.mac-snapshot.json"
NEW_FIXTURE = _FIXTURES / "AlohaMiniRobot.20261110.pi-live.json"

# 登记记录的 ``source`` → 对应的原件副本（对账用）。
_SOURCE_FIXTURES = {
    "archive-20261022": OLD_FIXTURE,
    "pi-live-20261110": NEW_FIXTURE,
}


def _old_motors() -> dict[str, Any]:
    """旧代际那 18 台电机的标定字段（每次现读，用例之间不共享可变状态）。"""
    return extract_motors(json.loads(OLD_FIXTURE.read_text(encoding="utf-8")))


def _record(
    motors: dict[str, Any], *, first_seen: str, source: str, note: str | None = None
) -> GenerationRecord:
    """用给定 motors 造一条登记记录，溯源字段取自真实 fixture。"""
    return GenerationRecord(
        calib_id=calib_id(motors),
        first_seen=first_seen,
        source=source,
        file_sha256=file_sha256(OLD_FIXTURE),
        byte_size=OLD_FIXTURE.stat().st_size,
        motors=canonicalize_motors(motors),
        note=note,
        acknowledged_by=None,
    )


def _shifted_motors(field_value: int) -> dict[str, Any]:
    """旧代际的深拷贝，把 ``lift_axis`` 的 ``homing_offset`` 改成给定值（造第二个代际）。"""
    motors = json.loads(json.dumps(_old_motors()))
    motors["lift_axis"]["homing_offset"] = field_value
    return motors


def test_register_is_idempotent_for_the_same_calib_id(tmp_path: Path) -> None:
    """同一 ``calib_id`` 二次登记返回 ``False``，记录数与首条记录内容都不变。"""
    registry = CalibrationRegistry.load(tmp_path / "calib-registry.yaml")
    record = _record(_old_motors(), first_seen="2026-10-22T01:00:00Z", source="mac-snapshot")

    first = registry.register(record)
    second = registry.register(replace(record, note="重复登记"))

    stored = registry.find(record.calib_id)

    assert (first, second) == (True, False)
    assert len(registry.generations) == 1
    assert stored is not None
    assert stored.note is None


def test_find_returns_the_registered_generation_and_none_for_an_unknown_id(
    tmp_path: Path,
) -> None:
    """``find`` 命中的是登记进去的那条记录；未登记的 id 返回 ``None``。"""
    registry = CalibrationRegistry.load(tmp_path / "calib-registry.yaml")
    record = _record(_old_motors(), first_seen="2026-10-22T01:00:00Z", source="mac-snapshot")
    registry.register(record)

    found = registry.find(record.calib_id)

    assert found is not None
    assert found.calib_id == record.calib_id
    assert found.motors == record.motors
    assert registry.find("ffffffffffff") is None


def test_latest_returns_the_last_registered_generation(tmp_path: Path) -> None:
    """``latest()`` 按登记顺序取最后一条，不受 calib_id 字典序影响。"""
    registry = CalibrationRegistry.load(tmp_path / "calib-registry.yaml")
    old = _record(_old_motors(), first_seen="2026-10-22T01:00:00Z", source="mac-snapshot")
    new = _record(_shifted_motors(-7), first_seen="2026-11-10T02:00:00Z", source="pi-live")
    registry.register(old)
    registry.register(new)

    latest = registry.latest()

    assert latest is not None
    assert latest.calib_id == new.calib_id
    assert len(registry.generations) == 2


def test_latest_is_none_for_an_empty_registry(tmp_path: Path) -> None:
    """空表 ``latest()`` 返回 ``None``，不抛异常。"""
    registry = CalibrationRegistry.load(tmp_path / "calib-registry.yaml")

    assert registry.generations == ()
    assert registry.latest() is None


def test_load_of_a_missing_path_yields_an_empty_registry(tmp_path: Path) -> None:
    """登记表文件不存在时得到空表（首次登记不该要求先手写文件）。"""
    registry = CalibrationRegistry.load(tmp_path / "absent-registry.yaml")

    assert registry.generations == ()
    assert registry.find("000000000000") is None


def test_save_and_load_round_trip_keeps_the_motors_and_provenance(tmp_path: Path) -> None:
    """``register`` + ``save`` + ``load`` 后 18×5 标定字段与溯源字段全部无损。"""
    path = tmp_path / "calib-registry.yaml"
    motors = _old_motors()
    registry = CalibrationRegistry.load(path)
    record = _record(
        motors,
        first_seen="2026-11-10T02:00:00Z",
        source="pi-live",
        note="2026-11-10 整机重标定",
    )
    assert registry.register(record) is True

    registry.save()
    reloaded = CalibrationRegistry.load(path)
    found = reloaded.find(record.calib_id)

    assert found is not None
    assert len(found.motors) == EXPECTED_MOTOR_COUNT
    assert found.motors == canonicalize_motors(motors)
    assert (found.first_seen, found.source) == ("2026-11-10T02:00:00Z", "pi-live")
    assert found.file_sha256 == record.file_sha256
    assert found.byte_size == record.byte_size
    assert found.note == "2026-11-10 整机重标定"
    assert found.acknowledged_by is None
    assert found == reloaded.latest()


def test_the_registry_file_is_yaml(tmp_path: Path) -> None:
    """登记表落盘是 YAML：``yaml.safe_load`` 读得回一份非空文档。"""
    path = tmp_path / "calib-registry.yaml"
    registry = CalibrationRegistry.load(path)
    registry.register(
        _record(_old_motors(), first_seen="2026-10-22T01:00:00Z", source="mac-snapshot")
    )

    registry.save()

    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document


def _motors_of(fixture: Path) -> dict[str, dict[str, int]]:
    """读一份 fixture 的 5 字段标定电机映射。"""
    return extract_motors(json.loads(fixture.read_text(encoding="utf-8")))


def test_the_shipped_registry_reevaluates_to_its_own_calib_ids() -> None:
    """仓库真实的 ``configs/calib-registry.yaml`` 逐条重算：手填的 id 与 motors 必须自洽。

    这是防「手改登记表后没人发现」的唯一守门点：``calib_id`` 由库函数从 ``motors`` 重算，
    相等才说明这条记录不是手写编的。
    """
    registry = CalibrationRegistry.load(SHIPPED_REGISTRY)

    assert len(registry.generations) >= 2, f"登记表只有 {len(registry.generations)} 条记录"
    for record in registry.generations:
        assert calib_id(record.motors) == record.calib_id, (
            f"登记表里的 calib_id 与 motors 重算值不一致："
            f"source={record.source} 登记={record.calib_id} 重算={calib_id(record.motors)}"
        )
        assert len(record.motors) == EXPECTED_MOTOR_COUNT, record.source
        for name, entry in record.motors.items():
            assert set(entry) == set(CANONICAL_FIELDS), f"{record.source} / {name} 的字段集不对"


def test_the_shipped_registry_provenance_matches_its_fixtures() -> None:
    """每条登记的 ``file_sha256`` / ``byte_size`` 等于其 ``source`` 对应 fixture 的实算值。"""
    registry = CalibrationRegistry.load(SHIPPED_REGISTRY)

    for record in registry.generations:
        fixture = _SOURCE_FIXTURES[record.source]
        assert record.file_sha256 == file_sha256(fixture), record.source
        assert record.byte_size == fixture.stat().st_size, record.source


def test_the_shipped_registry_latest_is_the_new_generation() -> None:
    """``latest()`` 是 2026-11-10 那一代，且与其它代际的 ``calib_id`` 各不相同。"""
    registry = CalibrationRegistry.load(SHIPPED_REGISTRY)
    new_id = calib_id(_motors_of(NEW_FIXTURE))

    latest = registry.latest()

    assert latest is not None
    assert latest.source == "pi-live-20261110"
    assert latest.calib_id == new_id
    ids = [record.calib_id for record in registry.generations]
    assert len(ids) == len(set(ids)), f"登记表里有重复 calib_id：{ids}"
    assert all(calib_id_value != latest.calib_id for calib_id_value in ids[:-1])
