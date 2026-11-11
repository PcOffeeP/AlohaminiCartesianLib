#!/usr/bin/env python3
"""判断一份标定文件或 v2 指纹属于哪一代，并给出四态机器可读结论。

用法::

    calib_health.py [--calib-file PATH | --fingerprint PATH] [--registry PATH]
                    [--session-anchor CALIB_ID] [--accepted PATH]

``--registry`` 默认仓库内的 ``configs/calib-registry.yaml``（路径由本文件位置推出，
不依赖 cwd）。stdout 必含一行严格格式 ``CALIB_STATE: <状态之一>``，便于脚本 grep。

状态与退出码：

=======================  ====  ==================================================
状态                     码    含义与后续动作
=======================  ====  ==================================================
``CALIB_KNOWN``          0    这个 ``calib_id`` 已登记，照常用
``CALIB_NEW_UNREGISTERED`` 1  结构通过但没登记过：新代际未登记、**非阻断**，
                              按提示跑 ``tools/calib_register.py`` 登记
``CALIB_SUSPECT``        3    能解析但不像正常标定：结构校验不过，或与已登记最新
                              代际比是 ``BROAD`` / ``REGION_INCONSISTENT``。需人核对
``CALIB_CORRUPT``        4    文件损坏：文件不存在、JSON 不可解析、顶层形状认不出、缺字段、
                              值为 ``null``、``range_min >= range_max``
``CALIB_SESSION_DRIFT``  4    第五态，只在传 ``--session-anchor`` 时可能命中：锚点与文件
                              里的 ``CALIB_ID`` 不同（本次会话认的不是文件这一代）。
                              不可继续
=======================  ====  ==================================================

``--session-anchor <calib_id>``（可选）守的是「跑着跑着标定被换了」：本次会话锚定的是某一
代 ``calib_id``，文件却已经是另一代。不符时打 ``SESSION_ANCHOR: MISMATCH …`` +
``CALIB_STATE: CALIB_SESSION_DRIFT``、退出 ``4``；相符时不打任何额外行，结论与退出码
与不传该参数**逐字相同**。

``--accepted <path>``（默认仓库内的 ``configs/calib-envelope-accepted.yaml``）**只追加**
包络判定行（``CALIB_MOTOR:`` / ``CALIB_ENVELOPE:`` /
``CALIB_ENVELOPE_IMPLAUSIBLE:`` / ``CALIB_DATASET:``），**默认就跑**包络判定、不改
``CALIB_STATE``、不改退出码：
它回答「这份标定像不像一台整机的真标定」，与``CALIB_STATE`` 回答的「这个 ``calib_id``
以前见过吗」是两件事。包络判定与阈值见
``src/alohamini_cartesian/calibration/envelope.py``；名单坏（``AcceptedListError``）只把原因
打到 stderr 并跳过包络行，四态与退出码不受影响。

注：退出码同号不同义。本文件的 ``1`` = 新代际未登记（非阻断）、``3`` = 可疑；
而 ``tools/calib_register.py`` 的 ``1`` = 结构校验不合格（阻断）、``3`` = 输入不可解析。
两个工具各自独立，读码请以本表为准。

判定优先级（先命中先返回，已裁决）：

1. 文件不存在 / 不是普通文件、JSON 不可解析 / 顶层形状认不出 → ``CALIB_CORRUPT``
   （输出里把「文件不存在（不是内容损坏）」与「内容损坏」分开措辞，但同属这一态）；
2. 缺必需字段、值为 ``null``、电机条目不是映射、``range_min >= range_max`` → ``CALIB_CORRUPT``；
3. 结构校验还有别的失败项（例如 ``drive_mode=7``、电机数不是 18）→ ``CALIB_SUSPECT``；
4. 表里已登记（``registry.find()`` 命中）→ ``CALIB_KNOWN``【本支在代码里最先返回，早于下面两条】；
5. 未登记、但与 ``latest()`` 比任一台电机的 ``id`` 或 ``drive_mode`` 不同 →
   ``CALIB_SUSPECT``（换线 / 翻向，不可能是正常重标定的结果；按 ``电机名.字段`` 点名）；
6. 未登记、与 ``latest()`` 比是 ``BROAD`` / ``REGION_INCONSISTENT`` →
   ``CALIB_SUSPECT``（全量变更或底盘被动，超出「只动手臂」的预期面）；
7. 未登记、以上都不命中 → ``CALIB_NEW_UNREGISTERED``（非阻断）。

注意第 4 条的位置：``registry.find()`` 命中即返回，**早于**第 5 / 第 6 条——已登记代际
不再与 ``latest()`` 比 ``id`` / ``drive_mode``，也不参与变化形态比对。口径：登记即视为
已由人确认（``acknowledged_by``），所以不要拿「工具报 ``CALIB_KNOWN``」当「接线与驱动方向
没问题」的证据；那两项要靠逐字段人工比对（见 ``docs/field-ops/runbook/3-health.md`` §4.1c
A2 行）。

``homing_offset`` / ``range_min`` / ``range_max`` 的变动**不**归第 5 条：那是正常重标定的
产物，走第 7 条（非阻断、需登记）。

``CALIB_NEW_UNREGISTERED`` 是非阻断结论：新代际本身正常，只是还没登记。
同 id 的「已登记 / 未登记」只看 ``calib_id``，不看 ``file_sha256``（后者换个字节格式就变）。
``--registry`` 指向一个不可读的登记表时，四态之一都给不出来，按用法错误返回 2。

``CALIB_ID`` 行只在电机台数等于 18（:data:`EXPECTED_MOTOR_COUNT`）时才给出可当身份用的 id；
台数不对时打印 ``CALIB_ID: 不可算（电机台数 N ≠ 18）``，因为对不完整的电机表算出的哈希
不具身份意义（``sha256("{}")`` 也能算出值，看着像个真 id）。
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from alohamini_cartesian.calibration.envelope import (
    AcceptedListError,
    classify_motors,
    load_accepted,
)
from alohamini_cartesian.calibration.identity import (
    CANONICAL_FIELDS,
    EXPECTED_MOTOR_COUNT,
    CalibrationStructureError,
    ChangePattern,
    calib_id,
    classify_change,
    extract_motors,
    is_tick_value,
    validate_structure,
)
from alohamini_cartesian.calibration.registry import CalibrationRegistry, RegistryError

EXIT_CALIB_KNOWN = 0
EXIT_CALIB_NEW_UNREGISTERED = 1
EXIT_USAGE = 2
EXIT_CALIB_SUSPECT = 3
EXIT_CALIB_CORRUPT = 4
# ``CALIB_SESSION_DRIFT`` 与 ``CALIB_CORRUPT`` 同码不同义（都是「不可继续」，原因不同）。
EXIT_CALIB_SESSION_DRIFT = 4

STATUS_KNOWN = "CALIB_KNOWN"
STATUS_NEW_UNREGISTERED = "CALIB_NEW_UNREGISTERED"
STATUS_SUSPECT = "CALIB_SUSPECT"
STATUS_CORRUPT = "CALIB_CORRUPT"
STATUS_SESSION_DRIFT = "CALIB_SESSION_DRIFT"

# 仓库根：本脚本在 <repo>/tools/ 下，取父目录的父目录，不依赖 cwd。
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "calib-registry.yaml"
# 包络接受名单的默认值：默认就跑包络判定（只追加 stdout 行，不改四态与退出码）。
# 交付的那份是空模板（``accepted: []``），零条目时判定一律按经验区间来；填了条目才放行
# 对应电机，而且要 ``motor`` + 实测 range + ``calib_id`` 三者全对上才算命中。
DEFAULT_ACCEPTED = REPO_ROOT / "configs" / "calib-envelope-accepted.yaml"

# 与已登记最新代际比时，这两种变化形态算「超出预期面」，要人核对。
SUSPECT_PATTERNS = (ChangePattern.BROAD, ChangePattern.REGION_INCONSISTENT)


def _build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器；两个输入参数互斥，二者都不给按用法错误处理。"""
    parser = argparse.ArgumentParser(
        description="判定标定文件／指纹的代际状态，输出 CALIB_STATE 机器可读行。",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--calib-file", metavar="PATH", help="标定文件（顶层即电机映射）")
    group.add_argument("--fingerprint", metavar="PATH", help="v2 指纹 JSON")
    parser.add_argument(
        "--registry",
        metavar="PATH",
        help=f"登记表路径（默认 {DEFAULT_REGISTRY.relative_to(REPO_ROOT)}）",
    )
    parser.add_argument(
        "--session-anchor",
        metavar="CALIB_ID",
        help="本次会话锚定的标定代际；与文件里的 CALIB_ID 不符时打 CALIB_SESSION_DRIFT 并退出 4",
    )
    parser.add_argument(
        "--accepted",
        metavar="PATH",
        default=str(DEFAULT_ACCEPTED),
        help=(
            "包络接受名单（默认仓库内 configs/calib-envelope-accepted.yaml）；"
            "默认就跑包络判定、只追加 CALIB_MOTOR / CALIB_ENVELOPE 行，"
            "不改 CALIB_STATE 与退出码"
        ),
    )
    return parser


def _emit(status: str, code: int, *details: str) -> int:
    """打印机器可读状态行与若干细节行，返回退出码。"""
    print(f"CALIB_STATE: {status}")
    for detail in details:
        print(f"  {detail}")
    return code


def _corruption_reasons(motors: Mapping[str, Any]) -> list[str]:
    """列出属于「文件损坏」的硬伤：缺必需字段、值为 ``null``、range 反向或退化。"""
    reasons: list[str] = []
    for name in sorted(motors):
        entry = motors[name]
        if not isinstance(entry, Mapping):
            reasons.append(f"电机 {name} 的条目不是映射（{type(entry).__name__}）")
            continue
        for field in CANONICAL_FIELDS:
            if field not in entry:
                reasons.append(f"电机 {name} 缺必需字段 {field}")
            elif entry[field] is None:
                reasons.append(f"电机 {name} 的字段 {field} 是 null（空值不是 0）")
        low, high = entry.get("range_min"), entry.get("range_max")
        if is_tick_value(low) and is_tick_value(high) and low >= high:
            reasons.append(f"电机 {name} 的 range_min={low} >= range_max={high}")
    return reasons


def _read_document(path: Path) -> tuple[Any, str | None]:
    """读 JSON；返回 ``(文档, 失败原因)``，失败时文档为 ``None``。"""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        return None, f"输入文件读不了：{path}（{exc}）"
    try:
        return json.loads(text), None
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f"JSON 不可解析：{path}（{exc}）"


def _identity_field_diffs(
    latest_motors: Mapping[str, Mapping[str, int]], motors: Mapping[str, Any]
) -> list[str]:
    """比两份标定的 ``id`` 与 ``drive_mode``，返回 ``电机名.字段: 旧 -> 新`` 形式的差异清单。

    只比这两个身份字段：``homing_offset`` / ``range_min`` / ``range_max`` 的变动是正常重标定
    的产物，不在这里拦。

    **只比两侧都存在的电机**（循环上就是 ``set(latest_motors) & set(motors)``）。只在一侧出现
    的电机交给 :func:`classify_change` 处理，而它只看 ``arm_`` 前缀：于是「把
    ``arm_left_gripper`` 改名成 ``arm_left_gripper_spare`` 同时改 ``id``」这种组合不会被本函数
    看到（两个名字都带 ``arm_`` → 仍判 ``ARM_ONLY``），属已知残余，靠登记时的人工核对兜。
    """
    diffs: list[str] = []
    for motor in sorted(set(latest_motors) & set(motors)):
        for field in ("id", "drive_mode"):
            before = latest_motors[motor].get(field)
            after = motors[motor].get(field) if isinstance(motors[motor], Mapping) else None
            if isinstance(before, int) and isinstance(after, int) and before != after:
                diffs.append(f"{motor}.{field}: {before} -> {after}")
    return diffs


def _judge_against_registry(
    registry: CalibrationRegistry,
    identity: str,
    motors: dict[str, Any],
    source_path: Path,
    registration_flag: str,
) -> int:
    """在登记表里查这份标定，返回四态之一对应的退出码。"""
    record = registry.find(identity)
    if record is not None:
        return _emit(
            STATUS_KNOWN,
            EXIT_CALIB_KNOWN,
            f"已登记：source={record.source} first_seen={record.first_seen}",
            f"note={record.note!r} acknowledged_by={record.acknowledged_by!r}",
        )
    latest = registry.latest()
    if latest is not None:
        identity_diffs = _identity_field_diffs(latest.motors, motors)
        if identity_diffs:
            return _emit(
                STATUS_SUSPECT,
                EXIT_CALIB_SUSPECT,
                f"与已登记最新代际 {latest.calib_id}（{latest.source}）比，"
                f"id / drive_mode 有 {len(identity_diffs)} 处变动：",
                *identity_diffs,
                "这类变动不可能是正常重标定的结果（本次重标定 18 项 id / drive_mode 全未变），"
                "按接线 / 驱动方向错误处置。",
            )
        pattern = classify_change(latest.motors, motors).pattern
        if pattern in SUSPECT_PATTERNS:
            return _emit(
                STATUS_SUSPECT,
                EXIT_CALIB_SUSPECT,
                f"与已登记最新代际 {latest.calib_id}（{latest.source}）比，变化形态是 "
                f"{pattern.name}",
                "该形态超出「只动手臂、底盘与升降轴不动」的预期面，需人核对这次标定是否预期。",
            )
        comparison = f"与已登记最新代际 {latest.calib_id}（{latest.source}）比是 {pattern.name}"
    else:
        comparison = f"登记表 {registry.path} 里没有任何代际，无从比较"
    return _emit(
        STATUS_NEW_UNREGISTERED,
        EXIT_CALIB_NEW_UNREGISTERED,
        f"结论：新代际未登记、非阻断（{comparison}）。",
        "结构校验通过，这份标定本身可用；只是它还没进登记表。",
        "登记命令：uv run --no-sync python tools/calib_register.py "
        f"{registration_flag} {source_path} --registry {registry.path} --write",
    )


def _session_anchor_note(anchor: str, identity: str | None) -> str | None:
    """锚点与文件 identity 相符时返回 ``None``，否则返回「哪一代 vs 哪一代」的说明。"""
    if identity is not None and anchor == identity:
        return None
    if identity is None:
        return f"本次锚点 {anchor} 无法核对：文件里的 CALIB_ID 不可算（电机台数不是 18 台）"
    return f"本次锚点 {anchor} ≠ 文件里的 CALIB_ID {identity}"


def _print_envelope(motors: Mapping[str, Any], accepted_path: Path) -> None:
    """追加包络判定行；名单坏时只报到 stderr 并跳过，不碰 ``CALIB_STATE`` 与退出码。"""
    try:
        accepted = load_accepted(accepted_path)
    except AcceptedListError as exc:
        print(f"接受名单不可读：{accepted_path}（{exc}）", file=sys.stderr)
        print("包络判定跳过；CALIB_STATE 与退出码不受影响。", file=sys.stderr)
        return
    report = classify_motors(motors, accepted=accepted)
    for item in report.motors:
        print(f"CALIB_MOTOR: {item.motor} {item.verdict.value} {item.detail}".rstrip())
    print(f"CALIB_ENVELOPE: {report.verdict.value}")
    print(f"CALIB_ENVELOPE_IMPLAUSIBLE: {len(report.implausible)}")
    if report.dataset_note is not None:
        print(f"CALIB_DATASET: {report.dataset_note}")


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 入口：读输入、分级判定、打印 ``CALIB_STATE`` 行并返回退出码。"""
    args = _build_parser().parse_args(argv)
    if args.calib_file is None and args.fingerprint is None:
        print("用法错误：必须给 --calib-file 或 --fingerprint 之一（见 --help）。", file=sys.stderr)
        return EXIT_USAGE
    from_file = args.calib_file is not None
    mode = "calib-file" if from_file else "fingerprint"
    registration_flag = "--from-file" if from_file else "--from-fingerprint"
    source_path = Path(args.calib_file if from_file else args.fingerprint)
    registry_path = Path(args.registry) if args.registry is not None else DEFAULT_REGISTRY
    print(f"INPUT: {source_path}（形状 {mode}）")

    if not source_path.exists():
        return _emit(
            STATUS_CORRUPT,
            EXIT_CALIB_CORRUPT,
            f"文件不存在（不是内容损坏）：{source_path}；无文件可读，也没有内容可判损坏",
        )
    if not source_path.is_file():
        return _emit(
            STATUS_CORRUPT,
            EXIT_CALIB_CORRUPT,
            f"输入路径不是普通文件（不是内容损坏）：{source_path}",
        )
    document, failure = _read_document(source_path)
    if document is None:
        return _emit(STATUS_CORRUPT, EXIT_CALIB_CORRUPT, str(failure))
    try:
        motors = extract_motors(document)
    except CalibrationStructureError as exc:
        return _emit(STATUS_CORRUPT, EXIT_CALIB_CORRUPT, f"顶层形状认不出：{exc}")

    reasons = _corruption_reasons(motors)
    if reasons:
        return _emit(STATUS_CORRUPT, EXIT_CALIB_CORRUPT, *reasons)
    print(f"MOTORS: {len(motors)} 台")
    identity: str | None
    if len(motors) != EXPECTED_MOTOR_COUNT:
        # 台数不对时不给 id：对不完整的电机表算出的哈希不具身份意义（看起来却像个真 id）。
        identity = None
        print(f"CALIB_ID: 不可算（电机台数 {len(motors)} ≠ {EXPECTED_MOTOR_COUNT}）")
    else:
        try:
            identity = calib_id(motors)
            print(f"CALIB_ID: {identity}")
        except CalibrationStructureError as exc:
            identity = None
            print(f"CALIB_ID: 不可算（{exc}）")

    if args.session_anchor is not None:
        note = _session_anchor_note(args.session_anchor, identity)
        if note is not None:
            print(f"SESSION_ANCHOR: MISMATCH {note}")
            return _emit(
                STATUS_SESSION_DRIFT,
                EXIT_CALIB_SESSION_DRIFT,
                "本次会话锚定的代际与 --calib-file 指向的标定不同：续跑等于拿没核对过的"
                "标定下发命令。先停手，确认该用哪一代，再重新起会话。",
            )

    problems = validate_structure(motors)
    if problems:
        return _emit(
            STATUS_SUSPECT,
            EXIT_CALIB_SUSPECT,
            "结构校验不通过（文件能解析，但取值不像正常标定），需人核对：",
            *problems,
        )

    try:
        registry = CalibrationRegistry.load(registry_path)
    except RegistryError as exc:
        print(f"登记表不可读：{registry_path}（{exc}）", file=sys.stderr)
        print("四态之一都给不出来，按用法错误返回 2。", file=sys.stderr)
        return EXIT_USAGE
    if identity is None:  # 结构通过时 identity 必然可算，这里只作防御
        return _emit(STATUS_SUSPECT, EXIT_CALIB_SUSPECT, "无法算出 calib_id，需人核对")
    code = _judge_against_registry(registry, identity, motors, source_path, registration_flag)
    _print_envelope(motors, Path(args.accepted))
    return code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
