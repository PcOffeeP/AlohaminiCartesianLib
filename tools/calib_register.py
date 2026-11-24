#!/usr/bin/env python3
"""把一份标定文件或 v2 指纹登记进标定代际登记表（默认 dry-run，只打印不写盘）。

用法::

    calib_register.py (--from-file PATH | --from-fingerprint PATH) [--registry PATH] [--write]

``--from-file`` 吃「顶层即电机映射」的标定文件，``--from-fingerprint`` 吃 v2 指纹
（``{"robot_metadata": {"motors": {...}}}``）。两者走同一个 :func:`extract_motors`，
因此同一份标定在两种形状下算出的 ``CALIB_ID`` 相同。

输出分五段：``INPUT`` / ``STRUCTURE`` / ``CALIB_ID`` / ``PLAUSIBILITY`` / ``CHANGE``，
最后是 ``REGISTRY`` 的结论。``CHANGE`` 段拿登记表里 ``latest()`` 那一条做对照，打印
变化形态（``UNCHANGED`` / ``ARM_ONLY`` / ``BROAD`` / ``REGION_INCONSISTENT``）与逐字段差异
（tick 与度两套口径）。

退出码：

=====  ==================================================================
0      成功（含 dry-run、含「该代际已登记，未改动」）
1      结构校验不合格（电机数、缺字段、取值非法），不写盘
2      参数错误（既没给 ``--from-file`` 也没给 ``--from-fingerprint``）
3      输入文件不可解析（读不了、JSON 坏、认不出的顶层形状）
4      登记表不可读或不可写
=====  ==================================================================

注：退出码同号不同义。本文件的 ``1`` = 结构校验不合格（阻断）、``3`` = 输入不可解析；
而 ``tools/calib_health.py`` 的 ``1`` = 新代际未登记（非阻断）、``3`` = 可疑。
两个工具各自独立，读码请以本表为准。

登记表的身份一律用 ``calib_id``；``file_sha256`` 只是溯源辅助。``first_seen`` 记的是
**登记时刻**（UTC），不是首次实测时刻——后者需要人在 ``note`` 里补。
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from alohamini_cartesian.calibration.envelope import (
    describe_thresholds,
    plausibility_warnings,
)
from alohamini_cartesian.calibration.identity import (
    CANONICAL_FIELDS,
    EXPECTED_MOTOR_COUNT,
    CalibrationStructureError,
    calib_id,
    canonicalize_motors,
    classify_change,
    extract_motors,
    file_sha256,
    validate_structure,
)
from alohamini_cartesian.calibration.registry import (
    CalibrationRegistry,
    GenerationRecord,
    RegistryError,
)

EXIT_OK = 0
EXIT_STRUCTURE = 1
EXIT_USAGE = 2
EXIT_UNPARSABLE = 3
EXIT_REGISTRY = 4

# 仓库根：本脚本在 <repo>/tools/ 下，故取父目录的父目录，不依赖 cwd。
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "calib-registry.yaml"


def _build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器；``--from-file`` 与 ``--from-fingerprint`` 互斥。"""
    parser = argparse.ArgumentParser(
        description="把标定文件或 v2 指纹登记进标定代际登记表（默认 dry-run）。",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--from-file", metavar="PATH", help="标定文件（顶层即电机映射）")
    group.add_argument("--from-fingerprint", metavar="PATH", help="v2 指纹 JSON")
    parser.add_argument(
        "--registry",
        metavar="PATH",
        help=f"登记表路径（默认 {DEFAULT_REGISTRY.relative_to(REPO_ROOT)}）",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="真登记并写盘；不给则只打印结论（dry-run）",
    )
    return parser


def _read_motors(path: Path) -> dict[str, Any] | int:
    """读输入文件并取出电机映射；失败时打印原因并返回退出码。"""
    try:
        raw_bytes = path.read_bytes()
    except OSError as exc:
        print(f"输入文件读不了：{path}（{exc}）", file=sys.stderr)
        return EXIT_UNPARSABLE
    try:
        document = json.loads(raw_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        print(f"输入文件不是合法 JSON：{path}（{exc}）", file=sys.stderr)
        return EXIT_UNPARSABLE
    try:
        return extract_motors(document)
    except CalibrationStructureError as exc:
        print(f"输入文件既不是标定文件也不是 v2 指纹：{path}（{exc}）", file=sys.stderr)
        return EXIT_UNPARSABLE


def _print_structure(motors: dict[str, Any]) -> bool:
    """打印结构校验结果；合格返回 ``True``，不合格返回 ``False`` 并逐条列出问题。"""
    problems = validate_structure(motors)
    if not problems:
        print(
            f"STRUCTURE: 合格（{EXPECTED_MOTOR_COUNT} 台 × {len(CANONICAL_FIELDS)} 个标定字段"
            "齐备、取值合法）"
        )
        return True
    print(f"STRUCTURE: 不合格（{len(problems)} 条问题）")
    for problem in problems:
        print(f"  - {problem}")
    return False


def _print_plausibility(motors: dict[str, Any]) -> None:
    """打印合理性告警；阈值随结论一起打印，方便现场核对判定口径。"""
    print("PLAUSIBILITY: 以下判定用经验区间，只告警、不阻断登记")
    for line in describe_thresholds():
        print(f"  阈值 {line}")
    warnings = plausibility_warnings(motors)
    if not warnings:
        print("  结果 无告警")
        return
    print(f"  结果 {len(warnings)} 条告警")
    for warning in warnings:
        print(f"  WARN {warning}")


def _print_change(registry: CalibrationRegistry, motors: dict[str, Any]) -> None:
    """打印与登记表 ``latest()`` 的变化形态与逐字段差异明细。"""
    latest = registry.latest()
    if latest is None:
        print(f"CHANGE: 登记表为空（{registry.path} 无记录），没有可比的前代")
        return
    report = classify_change(latest.motors, motors)
    print(
        f"CHANGE: {report.pattern.name}（与最新代际 {latest.calib_id}"
        f"／{latest.source}／{latest.first_seen} 比，{len(report.changed_motors)} 台电机有变动）"
    )
    if not report.diffs:
        print("  逐字段差异：无")
    for diff in report.diffs:
        print(
            f"  {diff.motor}.{diff.field}: {diff.old} -> {diff.new}"
            f"（{diff.delta_tick:+d} tick，{diff.delta_deg:+.3f}°）"
        )
    changed = set(report.changed_motors)
    unchanged = [name for name in sorted(motors) if name not in changed]
    print(f"  未变动电机：{', '.join(unchanged) if unchanged else '无'}")


def _utc_now_text() -> str:
    """当前 UTC 时刻，秒级精度、以 ``Z`` 结尾（登记时刻，不是首次实测时刻）。"""
    return datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 入口：读输入、校验、打印结论，``--write`` 时才登记。"""
    args = _build_parser().parse_args(argv)
    if args.from_file is None and args.from_fingerprint is None:
        print(
            "用法错误：必须给 --from-file 或 --from-fingerprint 之一（见 --help）。",
            file=sys.stderr,
        )
        return EXIT_USAGE
    from_file = args.from_file is not None
    mode = "calib-file" if from_file else "fingerprint"
    flag = "--from-file" if from_file else "--from-fingerprint"
    source_path = Path(args.from_file if from_file else args.from_fingerprint)
    registry_path = Path(args.registry) if args.registry is not None else DEFAULT_REGISTRY

    motors = _read_motors(source_path)
    if isinstance(motors, int):
        return motors
    print(f"INPUT: {source_path}（形状 {mode}，{len(motors)} 台电机）")
    if not _print_structure(motors):
        print("未登记：结构校验不合格。", file=sys.stderr)
        return EXIT_STRUCTURE
    identity = calib_id(motors)
    print(f"CALIB_ID: {identity}")
    _print_plausibility(motors)

    try:
        registry = CalibrationRegistry.load(registry_path)
    except RegistryError as exc:
        print(f"登记表不可读：{registry_path}（{exc}）", file=sys.stderr)
        return EXIT_REGISTRY
    _print_change(registry, motors)

    existing = registry.find(identity)
    if existing is not None:
        print(
            f"REGISTRY: 该代际已登记（source={existing.source}，"
            f"first_seen={existing.first_seen}），不追加、不覆盖"
        )
        return EXIT_OK
    if not args.write:
        print("REGISTRY: dry-run，未写盘。加 --write 才登记。")
        print(
            f"  登记命令：uv run --no-sync python tools/calib_register.py {flag} {source_path}"
            f" --registry {registry_path} --write"
        )
        return EXIT_OK

    record = GenerationRecord(
        calib_id=identity,
        first_seen=_utc_now_text(),
        source=f"{mode}:{source_path.name}",
        file_sha256=file_sha256(source_path),
        byte_size=source_path.stat().st_size,
        motors=canonicalize_motors(motors),
        note=f"由 tools/calib_register.py 以 {mode} 自动登记；登记人与首次实测时刻待人工补",
        acknowledged_by=None,
    )
    if not registry.append_if_absent(record):
        print("REGISTRY: 该代际已登记（写入前复查发现），不追加")
        return EXIT_OK
    try:
        registry.save()
    except RegistryError as exc:
        print(f"登记表写不了：{registry_path}（{exc}）", file=sys.stderr)
        return EXIT_REGISTRY
    print(f"REGISTRY: 已写入 {registry_path}")
    print(f"  first_seen={record.first_seen} 是登记时刻（UTC），不是首次实测时刻")
    print("  note / acknowledged_by 需人工补：脚本不代填，不用空字符串假装填过")
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
