#!/usr/bin/env python

"""离线 IK 基准脚本 ``benchmarks/alohamini_ik/benchmark_arm_ik.py`` 的 CLI 契约测试。

从 lerobot 仓库的 ``tests/robots/test_alohamini_arm_ik_benchmark.py`` 搬运，路径常量按本库
``src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf`` 重新定位；这些用例用
subprocess 真跑脚本，因此模块级要求可选依赖 placo。
"""

# Behavior: 实施包 P4-A cycle 5 —— 离线 IK 测速基准脚本的 CLI 契约
# `benchmarks/alohamini_ik/benchmark_arm_ik.py`（design §4 / 实施计划 P4-A / V13）。
#
# 被测脚本由 viber 新建，本文件用 subprocess 真实拉起脚本并解析其 JSON 报告，
# 因此不 import 任何 production 模块，也不依赖脚本内部函数名。
#
# READ 目标行为（主 Agent 给定）：
# - T-IK-14：脚本可运行 `uv run --no-sync python benchmarks/alohamini_ik/
#   benchmark_arm_ik.py --steps 20`，报告 p50/p95/p99 solve 耗时、
#   整步 p99、最大残差、最大相邻关节变化、失败计数；支持
#   `--urdf --steps --warmup --json --seed`。
# - T-IK-15：目标流含确定性不可达样本时，脚本报告失败计数且不崩溃，失败
#   100% 标记（固定 seed 伪随机目标流，内含少量不可达点）。
# - T-IK-16（30 Hz / 1000 步 p99 ≤ 7 ms 性能门限）**不对真实墙钟断言**：
#   预算判据的正确性由 T-IK-16.1–16.5 用**构造的**耗时直接验证（纯函数
#   `within_budget(solve_p99_ms, step_p99_ms)`）；脚本层面只断言结构（报告字段、
#   退出码与 `within_budget` 自洽、真跑到底），**不施加任何绝对毫秒上限，也不
#   要求小规模运行必须 exit 0**——本文件早先版本的这两类断言在机器满载时会因
#   p99 偶发超标而误报（20 步的 p99 是近极大次序统计量，一次 ~10 ms 的调度停顿
#   就足以让它越过 7 ms 预算，进而 exit 1 让全部 `assert_exit_zero` 用例一起翻红）。
#
# ===========================================================================
# CLI 契约（本 binding 裁定；若 viber 采用不同命名需先回到主 Agent 重新规划）
# ===========================================================================
#
# 调用：`[python] benchmarks/alohamini_ik/benchmark_arm_ik.py [flags]`
#
# flags:
#   --urdf PATH   默认 am_arm200 派生模型
#                 `src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf`
#   --steps N     计入统计的测量步数，默认 1000
#   --warmup N    预热步数（求解但不计入统计），默认 > 0
#   --seed N      目标流伪随机种子，默认固定字面量（例如 0）
#   --json        仅此时：stdout 恰好是“一个 JSON 对象”，人类可读摘要走 stderr
#
# 退出码（pytest 只断言这一语义，不断言墙钟数值）：
#   0    运行完成且性能预算满足
#   1    运行完成但性能预算不满足（墙钟 p99 超标），或输入不可用 / 运行期异常
#        （此类情况 --json 的 stdout 是 `{"error": ...}`，报告里没有 `steps`）
#   pytest 由此把「运行到底」定义为：退出码 ∈ {0, 1} 且 stdout 是单个 JSON 对象
#   且不含 `error` 键；退出码是否 0 只用来与报告里的 `within_budget` 对账。
#
# JSON schema（--json 时的 stdout，键名精确）：
#   {
#     "urdf":            str,    # 实际使用的 URDF 路径（绝对或相对皆可，按 basename 断言）
#     "steps":           int,    # == 请求的测量步数
#     "warmup":          int,    # == 请求的预热步数
#     "seed":            int,    # == 请求的种子
#     "solve_time_ms":   {"p50": float, "p95": float, "p99": float},
#     "step_time_ms":    {"p99": float},
#     "max_residual_m":  float,  # 仅统计 ok=True 样本的位置残差最大值（米）
#     "max_joint_delta_deg": float,  # 仅*连续已接受步*的最大单关节绝对变化（度）；
#                                    #   失败/复位产生的跨复位跳变不计入（F1）
#     "max_reacquire_delta_deg": float,  # 复位后重新捕获步的最大单关节绝对变化
#                                        #   （度），即被 max_joint_delta_deg 排除的
#                                        #   跨复位 delta（失败后再捕获，F1）
#     "residual_limit_m": float, # 判定 ok 的位置残差门限，== 0.005
#     "failure_count":   int,    # ok=False 的测量样本数
#     "success_count":   int,    # ok=True 的测量样本数
#     "within_budget":   bool    # 性能预算是否满足（与退出码一致）
#   }
#
# 目标流契约（T-IK-15 可判定性的前提）：
#   固定 seed 的内置目标流 = 可解小扰动样本 + **确定性注入的少量不可达样本**
#   （约每 5 个测量样本 1 个，位置被推到臂展之外）。因此对 `--steps 20`：
#   1 <= failure_count <= steps//4 + 2，且 success_count >= 1；不可达样本必须 100%
#   计入 failure_count（不得被当作成功吞掉），success + failure == steps。
#
# 鉴别意图（合理但错误的实现必须失败）：
# - 脚本不存在 / 跑不完（退出码 ∉ {0,1}）     -> 全部用例（RED 基线）；
# - 退出码与本份报告里的 within_budget 矛盾    -> report_for（经它记账的用例：14-2..14-6、
#                                              15-1..15-5；其中 14-2..14-5 与 15-1/15-2/
#                                              15-4 是经 primary_report() 间接调用）。不
#                                              经它的 14-1/14-7/14-8/14-9 各做各自的检查
#                                              ——14-8 走 parse_json_report，只做「纯
#                                              JSON + 无 error 键」检查，不对账退出码；
# - --json 把人类文本和 JSON 混在 stdout     -> T14-1/T14-2（json.loads 失败）；
# - 缺少任一必需统计字段 / 字段类型错误       -> T14-2；
# - 百分位错序（p50 > p95 > p99）或非有限     -> T14-3；
# - 整步耗时未独立计时（0 / 缺失 / 直接拷贝 solve 的副本）或与 solve 单位错配
#                                            -> T14-4（严格 `step_p99 > solve_p99`）；
# - 忽略 --seed/--steps/--warmup 回显         -> T14-5；
# - 忽略 --urdf / 默认 URDF 非 am_arm200      -> T14-6/T14-5；
# - 不可达样本被吞（failure_count == 0）       -> T15-1/T15-2；
# - 把所有样本都标失败（failure_count == steps）-> T15-2；
# - success+failure != steps（漏计/重复计）    -> T15-2；
# - 失败样本残差混入 max_residual_m            -> T15-2（不可达残差 >> 5 mm）；
# - 失败时不复位“上一接受解”，跨复位跳变混入
#   max_joint_delta_deg；max_reacquire_delta_deg 缺失/恒 0/直接拷贝 -> T15-4/T15-5；
# - 目标流非确定性（同 seed 两次结果不同）      -> T15-3；
# - 不可用 URDF 静默成功，或与「墙钟超预算 exit 1」
#   混为一谈（报告带了 steps，看不出没跑到底）-> T14-9；
# - 预算判据内联在 run_benchmark 里、没有可单测的
#   纯函数（阈值比较写错方向 / 漏一维 / 当成开区间）-> T16.1–T16.4；
# - run_benchmark 不走纯函数、自己再内联一份比较 -> T16.5；
# - 调用点把 (solve p99, step p99) 两个实参传反（两个维度都远低于各自阈值，
#   传反后布尔值不变，只有记录实参才抓得到）-> T16.5；
# - `main()` 超预算却不返回 1（与「运行期异常」同为 exit 1 的两个分支被拆开）->
#   T16.6；`main()` 吞掉 `run_benchmark` 的异常、或把失败报成带 `steps` 的报告 ->
#   T16.7；
# - 报告缺 `within_budget` 时报裸 KeyError 而不是「缺哪个键」 -> report_for（S3）里
#   `_MISSING` 哨兵的那条描述性失败。**当前合同下不可达**：脚本 `main()` 在 try 之外
#   先调 `_human_summary(report)`，缺键时脚本自身先崩、stdout 为空，`parse_json_report`
#   先在空 stdout 上失败，走不到哨兵比较。故它是防御性分支，不构成一条鉴别力。
#
# **故意不断言**的两件事（断言它们才是抖动源）：
# - 任何**绝对毫秒上限**（曾有的 `p50 < 1000` / `step_p99 < 10000` 已删）：
#   它们在满载时也会误报。相对关系仍被钉住：T14-4 要求 **严格** `step_p99 >
#   solve_p99`。余量来自两条计时各自的量级（本机实测 solve p99 ≈ 0.034 ms、
#   step p99 ≈ 0.73 ms，约 22×）：`step_time_s` 由脚本在 `arm.solve(...)` 外面用
#   自己的 `perf_counter()` 计时，而 `solve_time_s` 只在 `_solve_once` 的求解循环
#   内计时，两者不可能相等。把 step 换成 solve 的副本会得到相等值，因此一个都跑
#   不掉；用 `>=` 则对这份副本恒真、零鉴别力（这正是本轮修掉的问题）。
# - 与运行规模无关：无论 5 步还是 20 步，都不要求 exit 0（20 步的 p99 是近极大
#   次序统计量，一次 ~10 ms 的调度停顿就足以越过 7 ms 预算并 exit 1）。
#
# 环境事实（本库）：开发环境默认不含 placo，缺它时本模块由下面的
# pytest.importorskip 整模块跳过；装了 placo 的环境才会真跑这些 subprocess 用例。
# 所有 subprocess 以仓库根为 cwd，传入小 steps 保证快速。

from __future__ import annotations

import functools
import importlib.util
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

# 这些用例用 subprocess 真跑基准脚本，脚本必须有 placo 才能出数。
pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")


# ---------------------------------------------------------------------------
# 冻结契约常量
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_REL = "benchmarks/alohamini_ik/benchmark_arm_ik.py"
SCRIPT = REPO_ROOT / SCRIPT_REL

URDF_REL = "src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf"
URDF = REPO_ROOT / URDF_REL

RESIDUAL_LIMIT_M = 0.005

# T-IK-16 的预算常量被钉住：它们是脚本对外的性能契约数字，不靠“测出来是多少”
# 决定。判据的正确性（T16.1–T16.4）用构造耗时对着这两个值直接验证。
SOLVE_BUDGET_MS = 7.0
STEP_BUDGET_MS = 15.0

# 「运行到底」的退出码集合：0 = 预算满足，1 = 跑完了但墙钟超出预算（或输入不可用 /
# 运行期异常）——后者由 `error` 键与缺失的 `steps` 字段区分（见 T14-9）。
ACCEPTED_EXIT_CODES = (0, 1)

SOLVE_PERCENTILE_KEYS = ("p50", "p95", "p99")
REQUIRED_TOP_LEVEL_KEYS = (
    "urdf",
    "steps",
    "warmup",
    "seed",
    "solve_time_ms",
    "step_time_ms",
    "max_residual_m",
    "max_joint_delta_deg",
    "max_reacquire_delta_deg",
    "residual_limit_m",
    "failure_count",
    "success_count",
    "within_budget",
)

# 主运行参数：20 步（T-IK-14 验收示例规模），warmup=1，JSON 输出。
PRIMARY_ARGS = ("--steps", "20", "--warmup", "1", "--json")
PRIMARY_STEPS = 20
PRIMARY_WARMUP = 1

SUBPROCESS_TIMEOUT_S = 180

# `_MISSING` 让「字段缺失」与「字段值为 None」能分开报错：`payload.get(key)` 返回的
# None 无法区分两者。注意这是**防御性**哨兵，当前合同下不可达——脚本 `main()` 在 try
# 之外先调 `_human_summary(report)`，报告缺 `within_budget` 时脚本自身先崩、stdout
# 为空，`report_for` 走不到哨兵比较那一步（详见 `report_for` 函数体的说明）。
_MISSING = object()


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def run_cli(args: tuple[str, ...], *, use_uv: bool = False) -> subprocess.CompletedProcess:
    """以仓库根为 cwd 拉起基准脚本；默认使用当前解释器（与 pytest 同一 placo 环境）。"""
    assert SCRIPT.is_file(), f"benchmark script missing: {SCRIPT_REL} (expected at {SCRIPT})"
    if use_uv:
        cmd = ["uv", "run", "--no-sync", "python", str(SCRIPT), *args]
    else:
        cmd = [sys.executable, str(SCRIPT), *args]
    return subprocess.run(
        cmd,
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=SUBPROCESS_TIMEOUT_S,
    )


@functools.cache
def _uv_python_has_placo() -> bool:
    """探测 `uv run --no-sync python` 的解释器能否导入 placo（不安装任何包）。"""
    probe = subprocess.run(
        ["uv", "run", "--no-sync", "python", "-c", "import placo"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=SUBPROCESS_TIMEOUT_S,
    )
    return probe.returncode == 0


def _run(args: tuple[str, ...]) -> subprocess.CompletedProcess:
    # 名字不叫 `_cached_run`：这里**没有**缓存（`functools.cache` 套在 subprocess 上会让
    # T15-3 的「同 seed 两次跑」变成同一份对象，也会掩盖抖动）。每次调用都真起一个进程。
    return run_cli(args)


def _describe(proc: subprocess.CompletedProcess, args: tuple[str, ...]) -> str:
    return (
        f"command args={args!r} cwd={REPO_ROOT}\n"
        f"exit={proc.returncode}\n"
        f"--- stdout ---\n{proc.stdout}\n"
        f"--- stderr ---\n{proc.stderr}"
    )


def parse_json_report(proc: subprocess.CompletedProcess, args: tuple[str, ...]) -> dict:
    """要求脚本「跑到底」并返回 stdout 里的单个 JSON 对象。

    这里把「跑到底」定义为：退出码 ∈ {0, 1}（1 表示跑完但墙钟超出预算）且 stdout
    恰好是一个 JSON 对象且**不含 `error` 键**（脚本内部异常 / URDF 不可用时输出
    `{"error": ...}`）——后者保证「退出 1」不会掩盖真失败。
    """
    assert proc.returncode in ACCEPTED_EXIT_CODES, (
        f"benchmark must run to completion (exit 0 or 1), got {proc.returncode}\n"
        f"{_describe(proc, args)}"
    )
    try:
        payload = json.loads(proc.stdout)
    except json.JSONDecodeError as exc:  # noqa: PERF203
        pytest.fail(f"--json stdout is not a single JSON object ({exc})\n{_describe(proc, args)}")
    assert isinstance(payload, dict), f"JSON report must be an object, got {type(payload).__name__}"
    assert "error" not in payload, (
        f"exit {proc.returncode} carrying an error payload: the benchmark did not run to "
        f"completion, the report is {payload!r}\n{_describe(proc, args)}"
    )
    return payload


def report_for(args: tuple[str, ...]) -> dict:
    """运行脚本并返回解析后的 JSON 报告（允许退出码 0 或 1，只要求跑到底）。

    退出码与报告里的 `within_budget` 必须自洽：退出码 0 ⇔ within_budget=True。这样
    exit 1（跑完但墙钟超出预算）不再被当作失败，而退出码语义仍被钉住。
    """
    proc = _run(args)
    payload = parse_json_report(proc, args)
    # 用 `.get` + 哨兵而不是 `payload["within_budget"]`：字段缺失时这条分支会给出
    # 「缺了哪个键、报告里有哪些键」的描述性失败。但它当前**不可达**（防御性分支）：
    # 脚本 `main()` 在 try 之外先调 `_human_summary(report)`，缺 `within_budget` 时
    # 脚本自身先抛 KeyError、stdout 为空，`parse_json_report` 已在空的 stdout 上失败，
    # 根本走不到这里。此分支保留只为在脚本侧调用顺序变化后仍不退化成一个裸 KeyError。
    budget = payload.get("within_budget", _MISSING)
    assert budget is not _MISSING, (
        f"report is missing `within_budget`, so the exit code cannot be reconciled against "
        f"it; got keys {sorted(payload)}\n{_describe(proc, args)}"
    )
    assert budget is (proc.returncode == 0), (
        f"exit code {proc.returncode} disagrees with within_budget={budget!r}\n"
        f"{_describe(proc, args)}"
    )
    return payload


def load_benchmark_module() -> ModuleType:
    """按路径导入被测脚本模块（其模块级不 import placo，可安全导入）。

    预算判据是纯函数，用构造耗时直接验证，比“真跑一次看是否超时”确定得多。
    """
    spec = importlib.util.spec_from_file_location("benchmark_arm_ik_under_test", SCRIPT)
    assert spec is not None and spec.loader is not None, f"cannot load {SCRIPT_REL}"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def primary_report() -> dict:
    """返回主运行参数（20 步 / warmup=1）下的 JSON 报告。"""
    return report_for(PRIMARY_ARGS)


# ---------------------------------------------------------------------------
# T-IK-14  script runnable + statistics + flags
# ---------------------------------------------------------------------------


def test_t_ik_14_1_runs_to_completion_with_json_report() -> None:
    """T-IK-14：脚本运行完成（退出码 0 或 1），stdout 是纯 JSON。"""
    proc = _run(PRIMARY_ARGS)
    assert proc.returncode in ACCEPTED_EXIT_CODES, (
        f"benchmark must run to completion (exit 0 or 1), got {proc.returncode}\n"
        f"{_describe(proc, PRIMARY_ARGS)}"
    )
    payload = json.loads(proc.stdout)  # 纯 JSON：混入人类文本会在此失败
    assert isinstance(payload, dict)
    assert "error" not in payload, (
        f"an exit {proc.returncode} run must still carry a real report, got {payload!r}\n"
        f"{_describe(proc, PRIMARY_ARGS)}"
    )


def test_t_ik_14_2_json_report_is_single_object_with_required_fields() -> None:
    """T-IK-14：JSON 报告是单个对象且含全部必需字段与类型。"""
    report = primary_report()
    missing = [key for key in REQUIRED_TOP_LEVEL_KEYS if key not in report]
    assert not missing, f"missing report keys: {missing}; got {sorted(report)}"

    assert isinstance(report["urdf"], str) and report["urdf"]
    assert isinstance(report["steps"], int) and not isinstance(report["steps"], bool)
    assert isinstance(report["warmup"], int) and not isinstance(report["warmup"], bool)
    assert isinstance(report["seed"], int) and not isinstance(report["seed"], bool)
    assert isinstance(report["failure_count"], int) and not isinstance(
        report["failure_count"], bool
    )
    assert isinstance(report["success_count"], int) and not isinstance(
        report["success_count"], bool
    )
    assert isinstance(report["within_budget"], bool)
    # within_budget 与退出码必须自洽（由 report_for() 统一对账：exit 0 ⇔ True）；
    # 这里不再要求它必须为 True——那是墙钟抖动，不是契约。T-IK-16 的阈值本身
    # 由 T16.1–T16.4 用构造耗时直接验证。
    for key in (
        "max_residual_m",
        "max_joint_delta_deg",
        "max_reacquire_delta_deg",
        "residual_limit_m",
    ):
        assert isinstance(report[key], (int, float)) and not isinstance(report[key], bool), key

    solve = report["solve_time_ms"]
    step = report["step_time_ms"]
    assert isinstance(solve, dict), f"solve_time_ms must be an object, got {type(solve).__name__}"
    assert isinstance(step, dict), f"step_time_ms must be an object, got {type(step).__name__}"
    for key in SOLVE_PERCENTILE_KEYS:
        assert key in solve, f"solve_time_ms missing {key!r}: {solve}"
        assert isinstance(solve[key], (int, float)) and not isinstance(solve[key], bool)
    assert "p99" in step, f"step_time_ms missing 'p99': {step}"
    assert isinstance(step["p99"], (int, float)) and not isinstance(step["p99"], bool)


def test_t_ik_14_3_solve_percentiles_are_ordered_finite_and_positive() -> None:
    """T-IK-14：solve 百分位有序、有限且为正。"""
    report = primary_report()
    solve = report["solve_time_ms"]
    p50, p95, p99 = (float(solve[k]) for k in SOLVE_PERCENTILE_KEYS)

    assert all(x == x for x in (p50, p95, p99)), f"solve percentiles must be finite: {solve}"
    assert 0.0 < p50 <= p95 <= p99, f"solve percentiles out of order / non-positive: {solve}"
    # 刻意不设绝对毫秒上限（曾有 `p50 < 1000.0`，已删）：满载时它同样会误报，而它
    # 能抓到的「数值被放大一个单位量级」并不是契约的一部分。百分位的结构（有限、
    # 为正、有序）才是本用例守的东西。


def test_t_ik_14_4_step_p99_is_finite_positive_and_exceeds_solve_p99() -> None:
    """T-IK-14：整步 p99 有限、为正且**严格大于** solve p99。"""
    report = primary_report()
    step_p99 = float(report["step_time_ms"]["p99"])
    solve_p99 = float(report["solve_time_ms"]["p99"])

    assert step_p99 == step_p99 and step_p99 > 0.0, (
        f"step p99 must be finite and positive: {step_p99}"
    )
    # 刻意不设绝对毫秒上限（曾有 `step_p99 < 10000.0`，已删）：满载时会误报。
    #
    # 为什么是严格 `>` 而不是 `>=`：`step_time_s` 与 `solve_time_s` 来自两条独立计时
    # —— 前者是脚本在 `arm.solve(...)` 外面自己起的 `perf_counter()`，后者只在
    # `_solve_once` 的求解循环内计时 —— 所以同一批样本里 step 恒大于 solve，且差距
    # 不是浮点噪声而是量级差（本机实测 solve p99 ≈ 0.034 ms、step p99 ≈ 0.73 ms，
    # 约 22×）。用 `>=` 会把这个关系写成恒真式：把 step 换成 solve 的副本
    # （`step_times_ms.append(solve_times_ms[-1])`）后 `solve_p99 == step_p99`，`>=`
    # 一条都不红，`>` 全红。
    #
    # 它抓的错类：整步耗时没被独立计时——拷贝 solve 的副本、漏测循环体里的开销、
    # 两个量用了不同单位（一个 ms 一个 s）。这些都是「报告里有 step_p99 但它是假的」。
    assert step_p99 > solve_p99, (
        f"step p99 ({step_p99} ms) must be strictly greater than solve p99 ({solve_p99} ms): "
        f"step time is measured outside `arm.solve(...)` and has to include it, so equality "
        f"means the step timing is not independent of the solve timing (e.g. a copied sample)"
    )


def test_t_ik_14_5_report_echoes_requested_steps_warmup_seed_and_default_urdf() -> None:
    """T-IK-14：报告回显 steps/warmup/seed 且默认 URDF 为 am_arm200。"""
    report = primary_report()
    assert report["steps"] == PRIMARY_STEPS
    assert report["warmup"] == PRIMARY_WARMUP
    assert isinstance(report["seed"], int)
    assert report["urdf"].replace("\\", "/").endswith("am_arm200/kinematics.urdf"), report["urdf"]


def test_t_ik_14_6_explicit_urdf_and_seed_flags_are_honoured() -> None:
    """T-IK-14：--urdf/--seed/--steps/--warmup 显式传参被遵守。"""
    args = (
        "--urdf",
        URDF_REL,
        "--steps",
        "5",
        "--warmup",
        "0",
        "--seed",
        "12345",
        "--json",
    )
    report = report_for(args)
    assert report["urdf"].replace("\\", "/").endswith("am_arm200/kinematics.urdf"), report["urdf"]
    assert report["steps"] == 5
    assert report["warmup"] == 0
    assert report["seed"] == 12345, f"--seed not honoured: {report['seed']}"


def test_t_ik_14_7_human_readable_run_completes_without_json_flag() -> None:
    """T-IK-14：不带 --json 时人类摘要走 stdout，且脚本跑到底（退出码 0 或 1）。"""
    args = ("--steps", "5", "--warmup", "0")
    proc = run_cli(args)
    assert proc.returncode in ACCEPTED_EXIT_CODES, (
        f"benchmark must run to completion (exit 0 or 1), got {proc.returncode}\n"
        f"{_describe(proc, args)}"
    )
    assert proc.stdout.strip(), "expected a human-readable summary on stdout"
    # 退出码 1 不得是「崩溃/输入不可用」：真跑到底的摘要里必有预算行。
    assert any(line.strip().startswith("within_budget=") for line in proc.stdout.splitlines()), (
        f"human summary must report within_budget=, got:\n{proc.stdout}\n{_describe(proc, args)}"
    )


def test_t_ik_14_8_documented_uv_invocation_runs_to_completion() -> None:
    """T-IK-14：文档化的 `uv run --no-sync python` 入口跑到底（退出码 0 或 1）。"""
    if shutil.which("uv") is None:
        pytest.skip("uv not on PATH; cannot exercise the documented acceptance entry point")
    if not _uv_python_has_placo():
        pytest.skip("uv run --no-sync 的解释器没有 placo，跳过 uv 入口用例")
    args = ("--steps", "5", "--warmup", "0", "--json")
    proc = run_cli(args, use_uv=True)
    payload = parse_json_report(proc, args)
    assert payload["steps"] == 5


def test_t_ik_14_9_unusable_urdf_fails_loud_with_nonzero_exit() -> None:
    """T-IK-14：不可用 URDF 必须非零退出而不是静默成功，且不得与「墙钟超预算 exit 1」混淆。"""
    bad_urdf = "src/alohamini_cartesian/kinematics/assets/am_arm200/does_not_exist.urdf"
    args = ("--urdf", bad_urdf, "--steps", "5", "--warmup", "0", "--json")
    proc = run_cli(args)
    assert proc.returncode != 0, (
        f"a non-existent URDF must not silently succeed\n{_describe(proc, args)}"
    )
    # 加强：「预算超标 exit 1」也是非零，所以光看退出码无法区分两种失败。真跑到底
    # 的报告里必有 `steps` 字段且无 `error` 键；URDF 不可用时 stdout 必须是
    # `{"error": ...}` 且没有 `steps`。
    payload = json.loads(proc.stdout)
    assert isinstance(payload, dict), f"JSON output must be an object\n{_describe(proc, args)}"
    assert "error" in payload, (
        f"an unusable URDF must report `error`, not a benchmark report: {payload!r}\n"
        f"{_describe(proc, args)}"
    )
    assert "steps" not in payload, (
        f"an unusable URDF must not emit a report with `steps` (that would be "
        f"indistinguishable from a budget overrun): {payload!r}\n{_describe(proc, args)}"
    )


# ---------------------------------------------------------------------------
# T-IK-15  deterministic unreachable samples are counted, marked, not fatal
# ---------------------------------------------------------------------------


def test_t_ik_15_1_unreachable_samples_are_counted_not_crashed() -> None:
    """T-IK-15：目标流含不可达样本时计入失败且不崩溃。"""
    # 不崩溃：report_for() 只要求跑到底（退出码 ∈ {0,1}、纯 JSON、无 error 键）。
    report = primary_report()
    assert report["failure_count"] >= 1, (
        f"built-in seeded stream must include unreachable targets, got failure_count="
        f"{report['failure_count']} (report: {report})"
    )


def test_t_ik_15_2_failures_are_marked_not_swallowed() -> None:
    """T-IK-15：失败被标记而非被吞，success+failure==steps。"""
    report = primary_report()
    steps = report["steps"]
    failures = report["failure_count"]
    successes = report["success_count"]

    assert steps == PRIMARY_STEPS
    assert successes + failures == steps, (
        f"success_count + failure_count ({successes} + {failures}) != steps ({steps})"
    )
    assert 0 < failures < steps, f"expected a *few* marked failures, got {failures}/{steps}"
    # 上界 = 注入数（约 steps//4）+ 显式余量 2，避免恰好卡在边界时翻红（F4）。
    upper = steps // 4 + 2
    assert failures <= upper, f"'少量'不可达样本预期 <= {upper}, got {failures}"
    assert successes >= 1

    # 失败不得被吞：不可达样本的残差 >> 门限，若混入 max_residual_m 会暴露；成功
    # 样本的残差必须先满足 5 mm 门限，才谈得上被标记为 ok=True。
    assert report["residual_limit_m"] == pytest.approx(RESIDUAL_LIMIT_M), report["residual_limit_m"]
    assert report["max_residual_m"] <= RESIDUAL_LIMIT_M + 1e-9, (
        f"max_residual_m ({report['max_residual_m']}) exceeds the 5 mm limit: a failed "
        f"(unreachable) solve was likely counted as success"
    )
    assert report["max_residual_m"] >= 0.0
    assert report["max_joint_delta_deg"] >= 0.0
    assert report["max_joint_delta_deg"] == report["max_joint_delta_deg"]


def test_t_ik_15_3_fixed_seed_is_deterministic() -> None:
    """T-IK-15：固定 seed 下目标流相关指标可复现。"""
    first = report_for(PRIMARY_ARGS)
    second = report_for(PRIMARY_ARGS)

    # 只断言“目标流相关”的量确定；wall-clock 耗时天然会波动，不作确定性要求。
    assert first["failure_count"] == second["failure_count"], (
        f"failure_count not deterministic for a fixed seed: {first['failure_count']} vs "
        f"{second['failure_count']}"
    )
    assert first["success_count"] == second["success_count"]
    assert first["steps"] == second["steps"] == PRIMARY_STEPS
    assert first["max_residual_m"] == pytest.approx(second["max_residual_m"], rel=1e-9, abs=1e-12)
    assert first["max_joint_delta_deg"] == pytest.approx(
        second["max_joint_delta_deg"], rel=1e-9, abs=1e-12
    )
    assert first["max_reacquire_delta_deg"] == pytest.approx(
        second["max_reacquire_delta_deg"], rel=1e-9, abs=1e-12
    )


# ---------------------------------------------------------------------------
# T-IK-15 (F1)  reset / reacquire separation of the joint-delta metrics
# ---------------------------------------------------------------------------

# 固定 seed 28 的确定目标流（--steps 20 --warmup 1）已实测标定：
#   连续已接受步最大 delta  ~= 12.7 deg
#   跨复位（失败->home 复位->再捕获）最大 delta  ~= 46.7（相对 home）/
#                                               ~= 58.7（相对失败前接收解）deg
# 因此“仅连续已接受步”的实现应使 max_reacquire_delta_deg >> max_joint_delta_deg；
# 未复位上一接受解的实现则两者比值 <= 1。
REACQUIRE_SEED = 28
REACQUIRE_ARGS = ("--steps", "20", "--warmup", "1", "--seed", str(REACQUIRE_SEED), "--json")


def test_t_ik_15_4_cross_reset_reacquire_delta_is_reported_and_nonzero() -> None:
    """T-IK-15：跨复位重新捕获的关节变化被单独报告且非零。"""
    report = primary_report()
    assert report["failure_count"] >= 1, report
    reacquire = report["max_reacquire_delta_deg"]
    assert isinstance(reacquire, (int, float)) and not isinstance(reacquire, bool), type(reacquire)
    assert math.isfinite(reacquire), reacquire
    assert reacquire > 0.0, (
        "a stream with unreachable samples resets to home and reacquires the pose; the "
        f"cross-reset delta must be reported, got {reacquire!r} (report: {report})"
    )


def test_t_ik_15_5_cross_reset_delta_is_excluded_from_consecutive_joint_delta() -> None:
    """T-IK-15：跨复位跳变不计入连续关节变化指标。"""
    report = report_for(REACQUIRE_ARGS)
    assert report["seed"] == REACQUIRE_SEED
    assert report["failure_count"] >= 1, report
    consecutive = float(report["max_joint_delta_deg"])
    reacquire = float(report["max_reacquire_delta_deg"])

    assert 0.0 < consecutive < reacquire, (
        f"cross-reset delta must be tracked separately: consecutive={consecutive} "
        f"reacquire={reacquire} (report: {report})"
    )
    # 标定：正确分离时该流上 reacquire/consecutive >= ~3.7；未复位时 <= 1.0。
    # 阈值 2.0 落在两者之间，且对 placo 分支选择的轻微差异保留余量。
    assert reacquire >= 2.0 * consecutive, (
        f"max_joint_delta_deg still contains the cross-reset jump: consecutive={consecutive} "
        f"reacquire={reacquire} (report: {report})"
    )


# ---------------------------------------------------------------------------
# T-IK-16  budget judgement is a pure function of the supplied percentiles
# ---------------------------------------------------------------------------
#
# 这一段是本轮抖动修复的核心：预算判据从「真跑一次看墙钟是否超时」改成对
# `within_budget(solve_p99_ms, step_p99_ms)` 这个纯函数用**构造的**耗时直接验证。
# 下面的用例一条 subprocess 都不跑，因此不受机器负载影响，同时比“跑一次”更有鉴别力：
# 阈值被写错方向 / 漏掉一维 / 用成开区间 / 对 nan 静默返回 True，都会立刻变红。


def test_t_ik_16_1_budget_thresholds_are_pinned_constants() -> None:
    """T-IK-16.1：两条预算阈值被钉在模块常量上（7.0 ms solve / 15.0 ms step）。

    守的是：阈值本身不靠“测出来是多少”决定。有人把 7.0 改成 8.0、或把 step 预算
    与 solve 预算写成一个常量，这里会直接变红。
    """
    module = load_benchmark_module()

    assert module.SOLVE_BUDGET_MS == SOLVE_BUDGET_MS == 7.0, (
        f"SOLVE_BUDGET_MS must stay pinned at 7.0 ms, got module={module.SOLVE_BUDGET_MS!r}, "
        f"test={SOLVE_BUDGET_MS!r}"
    )
    assert module.STEP_BUDGET_MS == STEP_BUDGET_MS == 15.0, (
        f"STEP_BUDGET_MS must stay pinned at 15.0 ms, got module={module.STEP_BUDGET_MS!r}, "
        f"test={STEP_BUDGET_MS!r}"
    )


def test_t_ik_16_2_budget_judgement_accepts_values_within_the_closed_bounds() -> None:
    """T-IK-16.2：闭区间——恰好等于阈值算在预算内；远低于阈值更是。

    守的是「边界算通过」这一侧：把 `<=` 写成 `<` 会让恰好压线的真实运行被判超标，
    这里用构造的 (7.0, 15.0) 直接把它钉住，不依赖任何真实测量。
    """
    module = load_benchmark_module()

    # 闭区间边界：两个维度都恰好压线。
    assert module.within_budget(SOLVE_BUDGET_MS, STEP_BUDGET_MS) is True, (
        f"the closed-boundary case ({SOLVE_BUDGET_MS!r}, {STEP_BUDGET_MS!r}) must count as "
        f"within budget, got within_budget({SOLVE_BUDGET_MS!r}, {STEP_BUDGET_MS!r}) = False"
    )
    # 充裕余量：绝大多数真实运行都在这一档。
    assert module.within_budget(0.2, 0.4) is True, (
        "ample headroom (0.2 ms solve, 0.4 ms step) must count as within budget, got "
        "within_budget(0.2, 0.4) = False"
    )


@pytest.mark.parametrize(
    ("solve_p99_ms", "step_p99_ms", "axis"),
    [
        (SOLVE_BUDGET_MS + 1e-6, STEP_BUDGET_MS, "solve"),
        (SOLVE_BUDGET_MS, STEP_BUDGET_MS + 1e-6, "step"),
    ],
)
def test_t_ik_16_3_budget_judgement_rejects_marginal_overrun_on_either_axis(
    solve_p99_ms: float, step_p99_ms: float, axis: str
) -> None:
    """T-IK-16.3：任一维度略超阈值（+1e-6 ms）即判超预算。

    守的是「两个维度都必须看」：漏掉 step 那一维（只看 solve_p99）会让本用例的
    step 分支变红；用开区间写法则会把 16.2 的边界分支弄红。1e-6 ms 足够大以越过浮点
    噪声（ulp(7.0) ≈ 8.9e-16、ulp(15.0) ≈ 1.8e-15，1e-6 比两者高约 10 个量级，`+1e-6`
    结果精确可表示），又足够小以贴近阈值，使“超了”与“没超”只差一点点。
    """
    module = load_benchmark_module()

    assert module.within_budget(solve_p99_ms, step_p99_ms) is False, (
        f"a {axis}-axis overrun of 1e-6 ms past the budget must be rejected, got "
        f"within_budget({solve_p99_ms!r}, {step_p99_ms!r}) = True"
    )


@pytest.mark.parametrize(
    ("solve_p99_ms", "step_p99_ms", "axis"),
    [
        (float("nan"), 0.4, "solve"),
        (0.2, float("nan"), "step"),
    ],
)
def test_t_ik_16_4_budget_judgement_rejects_non_finite_percentiles(
    solve_p99_ms: float, step_p99_ms: float, axis: str
) -> None:
    """T-IK-16.4：任一维度为 nan 时判超预算（不允许 nan 静默通过）。

    `nan <= 7.0` 在 IEEE-754 下是 False，所以 `and` 表达式会把 nan 判成超预算；
    这条用例把这个「失败要显式」的行为钉住。若实现改成 `not (x > budget)` 之类
    的等价写法，nan 会翻转成「通过」，本用例随即变红——这正是要守的东西。
    """
    module = load_benchmark_module()

    assert module.within_budget(solve_p99_ms, step_p99_ms) is False, (
        f"a NaN {axis} percentile must not be judged within budget, got "
        f"within_budget({solve_p99_ms!r}, {step_p99_ms!r}) = True"
    )


def test_t_ik_16_5_run_benchmark_derives_within_budget_from_the_pure_judgement(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T-IK-16.5：run_benchmark 报告里的 within_budget 来自那个纯函数，且参数顺序正确。

    守两件事：
    1. 「只有一份判据」——把纯函数换成恒 False 后，报告里的 within_budget 必须变
       False；若 run_benchmark 自己又内联了一份比较，本用例会保持 True 而变红。
    2. 「调用点没把两个参数传反」——stub 记录实参并与报告里的 p99 逐个对账。两个
       维度在真实运行中都远低于各自阈值（solve p99 ≈ 0.03 ms、step p99 ≈ 0.7 ms，
       阈值 7 / 15 ms），所以传反了布尔判定**不会变**，单看 within_budget 的值永远
       抓不到这个错类；只有记录实参才能抓到。
    """
    module = load_benchmark_module()
    urdf = module._default_urdf()

    calls: list[tuple[float, float]] = []

    def recording_stub(solve_p99_ms: float, step_p99_ms: float) -> bool:
        calls.append((solve_p99_ms, step_p99_ms))
        return False

    monkeypatch.setattr(module, "within_budget", recording_stub)
    forced = module.run_benchmark(urdf, steps=2, warmup=0, seed=0)

    assert forced["within_budget"] is False, (
        "run_benchmark must take its within_budget from the module-level pure judgement; "
        f"a stubbed False was ignored (report={forced})"
    )
    assert forced["steps"] == 2
    assert len(calls) == 1, f"the budget judgement must be called exactly once, got {calls!r}"

    solve_p99 = float(forced["solve_time_ms"]["p99"])
    step_p99 = float(forced["step_time_ms"]["p99"])
    # 前提：两个量确实可区分（整步计时包住 solve），否则「顺序传反」这条断言重新
    # 退化成恒真。真实比值约 20×，这里只要求严格不等，不施加任何绝对毫秒上限。
    assert step_p99 > solve_p99, (
        f"premise broken: step p99 ({step_p99} ms) must exceed solve p99 ({solve_p99} ms), "
        f"otherwise passing them in the wrong order is undetectable"
    )
    assert calls[0] == (solve_p99, step_p99), (
        f"within_budget must be called as (solve p99, step p99); got {calls[0]!r} for "
        f"report solve p99={solve_p99!r} step p99={step_p99!r}"
    )

    monkeypatch.undo()
    real = module.run_benchmark(urdf, steps=2, warmup=0, seed=0)

    # 真实耗时下 within_budget 是否 True 取决于机器，这里只要求它是 bool 且与纯函数
    # 对同一批百分位的判断一致——绝不要求它为 True（那会重新引入墙钟依赖）。
    assert isinstance(real["within_budget"], bool), real
    expected = module.within_budget(real["solve_time_ms"]["p99"], real["step_time_ms"]["p99"])
    assert real["within_budget"] is expected, real


# ---------------------------------------------------------------------------
# T-IK-16  main() exit codes: in-process, no wall clock involved
# ---------------------------------------------------------------------------
#
# 下面两条用例在**进程内**调 `main()` 并用 monkeypatch 直接给出判定结果，所以既不跑
# subprocess、也不看墙钟；它们把两个「退出码都是 1」的分支拆开：
#   - 跑完了但预算不满足  -> exit 1 且 stdout 是完整报告（有 steps，无 error）
#   - 运行期抛异常        -> exit 1 且 stdout 是 {"error": ...}（无 steps）
# 只靠 `parse_json_report()` 里的 `error` 键是被动遇到才覆盖，也区分不了「谁触发的
# exit 1」；这里主动把两个分支各自钕住。


def _main_argv(*extra: str) -> list[str]:
    """构造 `sys.argv`：极小组次 + JSON，使 `main()` 跑得很快。"""
    return ["benchmark_arm_ik.py", "--steps", "2", "--warmup", "0", "--json", *extra]


def test_t_ik_16_6_main_returns_one_with_a_full_report_when_over_budget(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """T-IK-16.6：预算不满足时 `main()` 返回 1，且 stdout 仍是一份完整报告。

    守的是「超预算 = 1」这个分支本身：判据被 stub 成恒 False，因此本用例与真实墙钟
    无关。同时钉住它与「运行期异常」的可区分性——这里的报告字段完整且**没有**
    `error` 键，`steps` 也在。
    """
    module = load_benchmark_module()
    monkeypatch.setattr(module, "within_budget", lambda solve_p99_ms, step_p99_ms: False)
    monkeypatch.setattr(sys, "argv", _main_argv())

    exit_code = module.main()
    stdout = capsys.readouterr().out

    assert exit_code == 1, f"an over-budget run must exit 1, got {exit_code}; stdout={stdout!r}"
    payload = json.loads(stdout)
    assert isinstance(payload, dict), f"--json stdout must be one object, got {payload!r}"
    assert "error" not in payload, (
        f"an over-budget exit 1 must not carry an `error` payload: {payload!r}"
    )
    missing = [key for key in REQUIRED_TOP_LEVEL_KEYS if key not in payload]
    assert not missing, (
        f"over-budget report is incomplete, missing {missing}; got {sorted(payload)}"
    )
    assert payload["within_budget"] is False, payload["within_budget"]
    assert payload["steps"] == 2, payload["steps"]


def test_t_ik_16_7_main_returns_one_with_an_error_payload_when_run_benchmark_raises(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """T-IK-16.7：`run_benchmark` 抛异常时 `main()` 返回 1，stdout 是 `{"error": ...}`。

    守的是 `except Exception` 分支：退出码同样是 1，但 stdout 必须**没有** `steps`
    字段——否则「跑完了但超预算」与「根本没跑完」在机器可读的输出里无法区分（脚本
    会抱着一份不存在的报告说超预算）。
    """
    module = load_benchmark_module()

    def raising_run_benchmark(*args: object, **kwargs: object) -> dict:
        raise RuntimeError("synthetic benchmark failure")

    monkeypatch.setattr(module, "run_benchmark", raising_run_benchmark)
    monkeypatch.setattr(sys, "argv", _main_argv())

    exit_code = module.main()
    stdout = capsys.readouterr().out

    assert exit_code == 1, (
        f"a raising run_benchmark must exit 1, got {exit_code}; stdout={stdout!r}"
    )
    payload = json.loads(stdout)
    assert isinstance(payload, dict), f"--json stdout must be one object, got {payload!r}"
    assert "error" in payload, f"an internal failure must report `error`: {payload!r}"
    assert "synthetic benchmark failure" in str(payload["error"]), payload["error"]
    assert "steps" not in payload, (
        f"an internal failure must not emit a report with `steps` (that would look like a "
        f"budget overrun): {payload!r}"
    )
