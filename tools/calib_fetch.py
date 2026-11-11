#!/usr/bin/env python3
"""抓取一份标定文件、判定包络、可选登记（默认只读，不写盘）。

用法::

    calib_fetch.py (--from-file PATH | --host HOST) [--kind {robot,teleop}]
                   [--remote-path PATH] [--user USER] [--ssh-binary PATH]
                   [--accepted PATH] [--registry PATH]
                   [--out PATH] [--record] [--allow-implausible REASON] [--dry-run]

``--from-file`` 读本地一份标定文件；``--host`` 走 ssh 到机器人上 ``cat`` 远端标定文件
（远端路径由 ``--remote-path`` 给，默认按 ``--kind`` 取值：``robot`` →
``/home/pi5/.cache/huggingface/lerobot/calibration/robots/alohamini/AlohaMiniRobot.json``；
``teleop`` → ``/home/pi5/.cache/huggingface/lerobot/calibration/teleoperators/am_leader_6dof/
am_leader_6dof.json``（**推定值、待现场核对**，对不上就用 ``--remote-path`` 显式给））。

``--kind robot``（默认）期望的是 18 台跟随臂（follower）整机标定；``--kind teleop`` 是操作者
**显式声明**本次取的是遥操（leader）侧数据。遥操侧数据不能用于下发命令，所以声明 ``teleop``
时一律按不可继续处理（退出 ``4``），结论写明「按 ``--kind teleop`` 取到的是遥操侧（leader）
标定，不能用于下发」，而不是只给一句含糊的形状判词。

``--user`` 默认 ``pi5``，拼成 ssh 的 ``-l <user>``（刻意不用 ``user@host`` 写法：``--host``
保持原样，``--dry-run`` 打出来的命令可以原样拿去现场敲）。现场等价的抓取命令::

    ssh -o BatchMode=yes -o ConnectTimeout=20 -l pi5 <host> cat <remote-path>

``-o BatchMode=yes`` 让 ssh 在需要口令或 host key 确认时**直接失败**，不挂住等输入；
``ConnectTimeout`` 与 ``subprocess.run(timeout=)`` 都取模块常量 :data:`SSH_TIMEOUT_S`
（超时秒数，现场网络慢可调这一处），超时按「读不到远端」退出 ``4``。

**ssh 失败不回落本地缓存**：非零退出、起不了 ssh、stdout 为空（stderr 有内容也算失败形态）、
或超时，一律退出 ``4``，且不产生任何标定落盘文件。拿缓存冒充「刚抓的标定」比抓不到更危险。

stdout 的机器可读行：

* ``CALIB_ID:`` 电机台数等于 18 时的代际身份（台数不对时写「不可算」，理由同
  ``tools/calib_health.py``：不完整的电机表算出的哈希看着像真 id）；
* ``CALIB_SOURCE: file`` / ``CALIB_SOURCE: ssh`` 这份标定的来源；
* 每台电机一行 ``CALIB_MOTOR: <motor> <verdict> <detail>``；
* ``CALIB_ENVELOPE: <汇总结论>`` 与 ``CALIB_ENVELOPE_IMPLAUSIBLE: <不合格台数>``；
* 数据集形状不对时另有 ``CALIB_DATASET: <说明>``（点名「遥操臂」）；
* ``CALIB_RECORD:`` 只在 ``--record`` 时出现。

退出码：

=====  ==========================================================================
0      通过（含 ``--dry-run``、含用 ``--allow-implausible`` 留痕后的通过）
1      有 ``IMPLAUSIBLE`` 且未声明理由：提醒、不阻断；给了 ``--out`` 才落盘（门禁之后）
2      用法错误（既没给 ``--from-file`` 也没给 ``--host``（或两者同时给）；
       ``--allow-implausible`` 缺 ``--out``）
4      不可继续：ssh 抓取失败或超时、``--kind teleop``（遥操侧数据不能用于下发）、输入不是
       合法 JSON、顶层形状认不出、结构自洽失败、``WRONG_DATASET``（遥操臂）、接受名单坏、
       登记表读写失败、``--out`` 写不了、``--allow-implausible`` 的留痕写不出来
=====  ==========================================================================

``--out`` 的落盘时机在**门禁之后**：``--kind teleop``、结构自洽失败、``WRONG_DATASET``、
接受名单坏这些「不可继续」的路径不写盘，stderr 会点名「未产生任何标定落盘文件（原因：…）」，
免得用失败输入留下一份看着像标定的文件。门禁顺序是「``--kind teleop`` → 结构自洽 →
数据集形状」，任一不过即退出 ``4``。``IMPLAUSIBLE``（退出 ``1``）与 ``--allow-implausible``
（退出 ``0``）都已过门禁，照旧落盘。

**退出 ``4`` 一律不留快照**：门禁之后的写盘顺序是「留痕 → 登记 → 快照」，快照是最后一个
写盘动作。``--allow-implausible`` 的留痕写不出来也属不可继续（退出 ``4``），那时快照还没写，
所以不会留下一份看着像标定的文件；留痕与登记都成功了才落快照。

注：退出码同号不同义。本文件的 ``1`` = 包络提醒（非阻断）；``tools/calib_health.py`` 的
``1`` = 新代际未登记、``3`` = 可疑。各工具读码以各自文件为准。

包络判定与阈值只在 ``src/alohamini_cartesian/calibration/envelope.py``；结构自洽复用
``identity.validate_structure``。``--allow-implausible`` 的留痕落在 ``--out`` 同目录下的
``overrides-<out 的 stem>.json``（所以要留痕必须给 ``--out``，脚本不猜路径）。
本脚本不 import ``lerobot``、不连设备。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shlex
import subprocess
import sys
from collections.abc import Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from alohamini_cartesian.calibration.envelope import (
    AcceptedListError,
    EnvelopeReport,
    EnvelopeVerdict,
    classify_motors,
    load_accepted,
    rejected,
)
from alohamini_cartesian.calibration.identity import (
    EXPECTED_MOTOR_COUNT,
    CalibrationStructureError,
    calib_id,
    canonicalize_motors,
    extract_motors,
    validate_structure,
)
from alohamini_cartesian.calibration.registry import (
    CalibrationRegistry,
    GenerationRecord,
    RegistryError,
)

EXIT_OK = 0
EXIT_IMPLAUSIBLE = 1
EXIT_USAGE = 2
EXIT_BLOCKED = 4

# 仓库根：本脚本在 <repo>/tools/ 下，故取父目录的父目录，不依赖 cwd。
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "calib-registry.yaml"
DEFAULT_ACCEPTED = REPO_ROOT / "configs" / "calib-envelope-accepted.yaml"

# ``--kind`` 的两个取值：期望的数据来源侧（跟随臂 / 遥操 leader）。
KIND_ROBOT = "robot"
KIND_TELEOP = "teleop"
# 默认远端路径按 ``--kind`` 取值（``--remote-path`` 显式给出时以它为准）。
DEFAULT_REMOTE_PATHS = {
    KIND_ROBOT: (
        "/home/pi5/.cache/huggingface/lerobot/calibration/robots/alohamini/AlohaMiniRobot.json"
    ),
    # 遥操（leader）侧的这条是**推定值、待现场核对**：lerobot 的标定缓存落在
    # ``~/.cache/huggingface/lerobot/calibration/teleoperators/<类型>/<id>.json``，本项目的
    # leader 型号在 vendor 侧叫 ``am-leader-6dof``；实际文件名（id）与那份文件是否也在 Pi
    # 的这个路径下都要现场 ``ls`` 确认，对不上就用 ``--remote-path`` 显式给。
    KIND_TELEOP: (
        "/home/pi5/.cache/huggingface/lerobot/calibration/teleoperators/am_leader_6dof/"
        "am_leader_6dof.json"
    ),
}
# ssh 的超时预算（秒）：``-o ConnectTimeout`` 与 ``subprocess.run(timeout=)`` 用同一个数。
# 现场网络慢（Pi 走 Wi-Fi、跨网段）时可临时调大，只改这一处；取 20 秒是「等连接 + cat 一份
# 几 KB 的 JSON 足够，又不至于让一次抓取把现场流程挂住」的折中（经验值、非厂商规定）。
SSH_TIMEOUT_S = 20


def _build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器：``--from-file`` 与 ``--host`` 互斥。"""
    parser = argparse.ArgumentParser(
        description="抓取标定文件、判定包络，可选登记（默认只读，不写盘）。",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--from-file", metavar="PATH", help="本地标定文件（顶层即电机映射）")
    group.add_argument("--host", metavar="HOST", help="远端主机（经 ssh 抓取）")
    parser.add_argument(
        "--kind",
        choices=(KIND_ROBOT, KIND_TELEOP),
        default=KIND_ROBOT,
        help=(
            "期望的数据来源侧：robot（默认，18 台跟随臂整机标定）或 teleop"
            "（操作者声明取的是遥操 leader 侧数据；遥操侧数据不能用于下发，一律退出 4）"
        ),
    )
    parser.add_argument(
        "--remote-path",
        metavar="PATH",
        help=(
            "远端标定文件路径（默认按 --kind 取值："
            f"{KIND_ROBOT} → {DEFAULT_REMOTE_PATHS[KIND_ROBOT]}；"
            f"{KIND_TELEOP} → {DEFAULT_REMOTE_PATHS[KIND_TELEOP]}，该条为推定值、待现场核对）"
        ),
    )
    parser.add_argument(
        "--user",
        metavar="USER",
        default="pi5",
        help="ssh 登录用户名（默认 pi5），拼成 ssh 的 -l <user>，--dry-run 打印的命令里也带",
    )
    parser.add_argument(
        "--ssh-binary",
        metavar="PATH",
        default="ssh",
        help="ssh 可执行文件（默认 ssh；离线验证时指向假程序）",
    )
    parser.add_argument(
        "--accepted",
        metavar="PATH",
        help=f"包络接受名单（默认 {DEFAULT_ACCEPTED.relative_to(REPO_ROOT)}）",
    )
    parser.add_argument(
        "--registry",
        metavar="PATH",
        help=f"登记表路径（默认 {DEFAULT_REGISTRY.relative_to(REPO_ROOT)}）",
    )
    parser.add_argument("--out", metavar="PATH", help="把抓到的原文照抄到这个路径（不给则不落盘）")
    parser.add_argument(
        "--record",
        action="store_true",
        help="把这份代际按 calib_id 幂等登记进登记表（--registry 指定）",
    )
    parser.add_argument(
        "--allow-implausible",
        metavar="REASON",
        help="声明已知情并接受 IMPLAUSIBLE（需同时给 --out；理由落在 overrides-*.json）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印将要执行的抓取动作，不执行、不落盘",
    )
    return parser


def _default_remote_path(kind: str) -> str:
    """按 ``--kind`` 取默认远端路径（未识别的 kind 由 argparse 的 choices 先拦）。"""
    return DEFAULT_REMOTE_PATHS.get(kind, DEFAULT_REMOTE_PATHS[KIND_ROBOT])


def _ssh_argv(ssh_binary: str, host: str, remote_path: str, user: str) -> list[str]:
    """将要执行的 ssh 命令（原样可复制去现场跑）。

    ``-o BatchMode=yes``：需要口令或 host key 确认时直接失败，不挂住等交互输入。
    ``-o ConnectTimeout=<SSH_TIMEOUT_S>``：限住连接建立时间（秒）。
    用户名用 ``-l <user>`` 拼，不用 ``user@host``：``--host`` 保持原样，``--dry-run`` 打印的
    命令与现场手敲的写法一致。
    """
    return [
        ssh_binary,
        "-o",
        "BatchMode=yes",
        "-o",
        f"ConnectTimeout={SSH_TIMEOUT_S}",
        "-l",
        user,
        host,
        "cat",
        remote_path,
    ]


def _fetch_over_ssh(
    host: str, remote_path: str, ssh_binary: str, user: str
) -> tuple[bytes | None, str]:
    """经 ssh 取回远端标定原文；失败返回 ``(None, 原因)``，**不回落任何本地缓存**。"""
    argv = _ssh_argv(ssh_binary, host, remote_path, user)
    try:
        completed = subprocess.run(argv, capture_output=True, check=False, timeout=SSH_TIMEOUT_S)
    except OSError as exc:
        return None, f"起不了 ssh 程序 {ssh_binary}（{exc}）；不回落本地缓存，也不产生落盘文件"
    except subprocess.TimeoutExpired:
        return None, (
            f"ssh 抓取超时：{shlex.join(argv)} 超过 {SSH_TIMEOUT_S} 秒未返回"
            f"（目标 {user}@{host}）；不回落本地缓存，也不产生标定落盘文件"
        )
    stderr_text = completed.stderr.decode("utf-8", "replace").strip()
    if completed.returncode != 0:
        return None, (
            f"ssh 抓取失败：{shlex.join(argv)} 退出码 {completed.returncode}"
            f"（stderr：{stderr_text or '空'}）；不回落本地缓存，也不产生标定落盘文件"
        )
    if not completed.stdout.strip():
        return None, (
            "ssh 抓取失败：stdout 为空"
            f"（stderr：{stderr_text or '空'}）；stdout 空就不是标定原文，"
            "不回落本地缓存，也不产生标定落盘文件"
        )
    return completed.stdout, ""


def _utc_now_text() -> str:
    """当前 UTC 时刻，秒级精度、以 ``Z`` 结尾（登记时刻，不是首次实测时刻）。"""
    return datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _emit_envelope(report: EnvelopeReport) -> None:
    """打印包络判定行：每台一行 ``CALIB_MOTOR``，再加汇总与数据集说明。"""
    for item in report.motors:
        print(f"CALIB_MOTOR: {item.motor} {item.verdict.value} {item.detail}".rstrip())
    print(f"CALIB_ENVELOPE: {report.verdict.value}")
    print(f"CALIB_ENVELOPE_IMPLAUSIBLE: {len(report.implausible)}")
    if report.dataset_note is not None:
        print(f"CALIB_DATASET: {report.dataset_note}")


def _overrides_path(out_path: Path) -> Path:
    """覆盖记录的落盘路径：``--out`` 同目录、``overrides-<stem>.json``。"""
    return out_path.with_name(f"overrides-{out_path.stem}.json")


def _no_snapshot_note(reason: str) -> str:
    """「不可继续」路径的落盘声明：**没有**产生任何标定落盘文件，并点明原因。"""
    return f"未产生任何标定落盘文件（原因：{reason}）"


def _write_overrides(
    out_path: Path, identity: str | None, source: str, reason: str, report: EnvelopeReport
) -> tuple[Path | None, str]:
    """把「已知情接受 IMPLAUSIBLE」写成 JSON 留痕；返回 ``(落盘路径, 失败原因)``。

    写不出来时返回 ``(None, 原因)``：没有留痕就不能算「已声明理由」，调用方按不可继续处理。
    """
    document = {
        "schema_version": 1,
        "calib_id": identity,
        "source": source,
        "declared_at_utc": _utc_now_text(),
        "reason": reason,
        "envelope_verdict": report.verdict.value,
        "implausible": [
            {"motor": item.motor, "verdict": item.verdict.value, "detail": item.detail}
            for item in report.implausible
        ],
    }
    path = _overrides_path(out_path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except OSError as exc:
        return None, str(exc)
    return path, ""


def _record(
    registry_path: Path,
    identity: str,
    source: str,
    payload: bytes,
    motors: Mapping[str, Any],
    report: EnvelopeReport,
) -> int:
    """按 ``calib_id`` 幂等登记这份代际；返回退出码（0 成功 / 4 登记表不可用）。"""
    try:
        registry = CalibrationRegistry.load(registry_path)
    except RegistryError as exc:
        print(f"登记表不可读：{registry_path}（{exc}）", file=sys.stderr)
        return EXIT_BLOCKED
    record = GenerationRecord(
        calib_id=identity,
        first_seen=_utc_now_text(),
        source=source,
        file_sha256=hashlib.sha256(payload).hexdigest(),
        byte_size=len(payload),
        motors=canonicalize_motors(motors),
        note=(
            "由 tools/calib_fetch.py 自动登记；"
            f"包络结论 {report.verdict.value}；登记人与首次实测时刻待人工补"
        ),
        acknowledged_by=None,
    )
    if not registry.append_if_absent(record):
        existing = registry.find(identity)
        first_seen = existing.first_seen if existing is not None else "?"
        source_of_record = existing.source if existing is not None else "?"
        print(
            f"CALIB_RECORD: 该代际已登记（source={source_of_record}，first_seen={first_seen}），"
            "不追加、不覆盖"
        )
        return EXIT_OK
    try:
        registry.save()
    except RegistryError as exc:
        print(f"登记表写不了：{registry_path}（{exc}）", file=sys.stderr)
        return EXIT_BLOCKED
    print(f"CALIB_RECORD: 已写入 {registry_path}（calib_id={identity}）")
    print(f"  first_seen={record.first_seen} 是登记时刻（UTC），不是首次实测时刻")
    return EXIT_OK


def _blocked(message: str) -> int:
    """不可继续：把原因打到 stderr 并返回 ``EXIT_BLOCKED``。"""
    print(message, file=sys.stderr)
    return EXIT_BLOCKED


def _blocked_usage(message: str) -> int:
    """用法错误：把原因打到 stderr 并返回 ``EXIT_USAGE``。"""
    print(f"用法错误：{message}", file=sys.stderr)
    return EXIT_USAGE


def _dry_run(
    from_file: str | None, host: str | None, remote_path: str, ssh_binary: str, user: str
) -> int:
    """只打印将要执行的抓取动作，不执行、不落盘。"""
    if from_file is not None:
        print(f"DRY-RUN: 将读取本地标定文件 {from_file}（不执行 ssh、不落盘）")
        return EXIT_OK
    command = shlex.join(_ssh_argv(ssh_binary, str(host), remote_path, user))
    print(f"DRY-RUN: 将要执行的 ssh 命令：{command}")
    print("DRY-RUN: 未执行、未落盘")
    return EXIT_OK


def main(argv: Sequence[str] | None = None) -> int:
    """CLI 入口：抓取 → 包络判定 → 门禁 →（可选）留痕 →（可选）登记 →（可选）落原文 → 退出码。

    写盘顺序刻意把快照放最后：退出码 ``4``（不可继续）的任何路径都不留快照。
    """
    args = _build_parser().parse_args(argv)
    if args.from_file is None and args.host is None:
        return _blocked_usage("必须给 --from-file 或 --host 之一（见 --help）。")
    if args.allow_implausible is not None and args.out is None:
        return _blocked_usage(
            "--allow-implausible 要留痕，必须同时给 --out"
            "（覆盖记录写在 --out 同目录的 overrides-*.json，脚本不猜路径）。"
        )
    from_file = args.from_file is not None
    source = "file" if from_file else "ssh"
    remote_path = (
        args.remote_path if args.remote_path is not None else _default_remote_path(args.kind)
    )
    if args.dry_run:
        return _dry_run(args.from_file, args.host, remote_path, args.ssh_binary, args.user)

    if from_file:
        input_path = Path(str(args.from_file))
        try:
            payload = input_path.read_bytes()
        except OSError as exc:
            return _blocked(f"输入文件读不了：{input_path}（{exc}）")
        source_label = f"{source}:{input_path.name}"
    else:
        payload, failure = _fetch_over_ssh(str(args.host), remote_path, args.ssh_binary, args.user)
        if payload is None:
            return _blocked(failure)
        source_label = f"{source}:{args.host}"

    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return _blocked(f"输入不是合法 JSON：{source_label}（{exc}）；未产生标定落盘文件")
    try:
        motors = extract_motors(document)
    except CalibrationStructureError as exc:
        return _blocked(f"输入既不是标定文件也不是 v2 指纹：{source_label}（{exc}）")

    print(f"INPUT: {source_label}（形状 {source}，{len(motors)} 台电机）")
    print(f"CALIB_SOURCE: {source}")
    identity: str | None = None
    if len(motors) == EXPECTED_MOTOR_COUNT:
        try:
            identity = calib_id(motors)
        except CalibrationStructureError:
            identity = None
    if identity is not None:
        print(f"CALIB_ID: {identity}")
    else:
        # 台数不对时不给 id：不完整的电机表算出的哈希看起来像个真 id，但没身份意义。
        print(f"CALIB_ID: 不可算（电机台数 {len(motors)} ≠ {EXPECTED_MOTOR_COUNT}）")

    accepted_path = DEFAULT_ACCEPTED if args.accepted is None else Path(args.accepted)
    try:
        accepted = load_accepted(accepted_path)
    except AcceptedListError as exc:
        return _blocked(
            f"接受名单不可用：{accepted_path}（{exc}）；{_no_snapshot_note('接受名单不可用')}"
        )
    print(f"CALIB_ACCEPTED: {accepted_path}（{len(accepted.entries)} 条）")

    report = classify_motors(motors, accepted=accepted)
    _emit_envelope(report)

    # 门禁一：操作者显式声明取的是遥操（leader）侧数据。遥操数据不能用于下发，一律不可继续。
    if args.kind == KIND_TELEOP:
        return _blocked(
            "按 --kind teleop 取到的是遥操侧（leader）标定，不能用于下发（不可继续）；"
            + _no_snapshot_note("--kind teleop 声明的是遥操侧数据")
        )

    problems = validate_structure(motors)
    if problems:
        print(f"CALIB_STRUCTURE: 不合格（{len(problems)} 条问题）", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        print("结构自洽失败，不可继续。", file=sys.stderr)
        return _blocked(_no_snapshot_note("结构自洽失败"))
    if rejected(report):
        print("数据集形状不对（遥操臂标定，不是 18 台整机标定），不可继续。", file=sys.stderr)
        return _blocked(_no_snapshot_note("数据集形状不对：WRONG_DATASET"))

    # 落盘顺序：留痕 → 登记 → 快照。退出码 ``4``（不可继续）的任何路径都不得留下标定快照，
    # 所以可能返回 ``4`` 的写盘动作排在快照之前：**快照是最后一个写盘动作**。
    overrides_written: Path | None = None
    if report.verdict is EnvelopeVerdict.IMPLAUSIBLE and args.allow_implausible is not None:
        # 入口已拦过 args.out 为 None；这里显式判断而不是 assert（``python -O`` 下 assert
        # 会整条消失，而这里承担的是「没有 --out 就没有留痕位置」的契约收窄）。
        out_text = args.out
        if out_text is None:
            return _blocked_usage(
                "--allow-implausible 要留痕，必须同时给 --out（入口已拦，这里是二道防线）。"
            )
        overrides_written, failure = _write_overrides(
            Path(out_text), identity, source_label, args.allow_implausible, report
        )
        if overrides_written is None:
            return _blocked(
                f"留痕文件写不出来，因此不能算已声明理由：{_overrides_path(Path(out_text))}"
                f"（{failure}）；没有留痕就不放行，按不可继续处理"
            )

    if args.record:
        if identity is None:  # 结构通过时必然可算，这里只作防御
            return _blocked("无法算出 calib_id，不登记。")
        registry_path = DEFAULT_REGISTRY if args.registry is None else Path(args.registry)
        code = _record(registry_path, identity, source_label, payload, motors, report)
        if code != EXIT_OK:
            return code

    # 过了门禁才落盘：失败路径不得留下一份看着像标定的文件。
    if args.out is not None:
        out_path = Path(args.out)
        try:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_bytes(payload)
        except OSError as exc:
            return _blocked(f"快照写不了：{out_path}（{exc}）")
        print(f"CALIB_SNAPSHOT: {out_path}（{len(payload)} 字节，原文照抄）")

    if report.verdict is EnvelopeVerdict.IMPLAUSIBLE:
        if overrides_written is not None:
            print(f"CALIB_OVERRIDES: {overrides_written}（{len(report.implausible)} 台已知情接受）")
            print(f"结论：已声明理由，可继续（{args.allow_implausible}）。")
            return EXIT_OK
        snapshot_text = (
            f"快照已落盘到 {args.out}" if args.out is not None else "快照未落盘（未给 --out）"
        )
        print(
            f"提醒（非阻断）：{len(report.implausible)} 台电机落在经验区间之外，"
            f'{snapshot_text}；确认实物行程无误后加 --allow-implausible "理由" 留痕再继续。'
        )
        return EXIT_IMPLAUSIBLE
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
