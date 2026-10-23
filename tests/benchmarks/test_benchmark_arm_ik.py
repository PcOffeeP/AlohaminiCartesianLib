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
#   benchmark_arm_ik.py --steps 20`，退出码 0；报告 p50/p95/p99 solve 耗时、
#   整步 p99、最大残差、最大相邻关节变化、失败计数；支持
#   `--urdf --steps --warmup --json --seed`。
# - T-IK-15：目标流含确定性不可达样本时，脚本报告失败计数且不崩溃，失败
#   100% 标记（固定 seed 伪随机目标流，内含少量不可达点）。
# - T-IK-16（30 Hz / 1000 步 p99 ≤ 7 ms 性能门限）**不在 pytest 中断言**：
#   脚本自身在超标时非零退出，由 acceptance 运行承担；本文件仅在小组次运行
#   满足预算时要求退出码 0。
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
# 退出码：
#   0    运行完成且性能预算满足（小规模运行必须为 0）
#   非 0 性能门限超标（T-IK-16，pytest 不断言该门限本身），或输入不可用
#        （例如 --urdf 指向不存在的文件）
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
# - 脚本不存在 / 非零退出                    -> 全部用例（RED 基线）；
# - --json 把人类文本和 JSON 混在 stdout     -> T14-1/T14-2（json.loads 失败）；
# - 缺少任一必需统计字段 / 字段类型错误       -> T14-2；
# - 百分位错序（p50 > p95 > p99）或非有限     -> T14-3；
# - 整步耗时未测量（0 / 缺失）或与 solve 单位错配 -> T14-4；
# - 忽略 --seed/--steps/--warmup 回显         -> T14-5；
# - 忽略 --urdf / 默认 URDF 非 am_arm200      -> T14-6/T14-5；
# - 不可达样本被吞（failure_count == 0）       -> T15-1/T15-2；
# - 把所有样本都标失败（failure_count == steps）-> T15-2；
# - success+failure != steps（漏计/重复计）    -> T15-2；
# - 失败样本残差混入 max_residual_m            -> T15-2（不可达残差 >> 5 mm）；
# - 失败时不复位“上一接受解”，跨复位跳变混入
#   max_joint_delta_deg；max_reacquire_delta_deg 缺失/恒 0/直接拷贝 -> T15-4/T15-5；
# - 目标流非确定性（同 seed 两次结果不同）      -> T15-3；
# - 不可用 URDF 静默成功                       -> T14-9。
#
# 环境事实（本库）：开发环境默认不含 placo，缺它时本模块由下面的
# pytest.importorskip 整模块跳过；装了 placo 的环境才会真跑这些 subprocess 用例。
# 所有 subprocess 以仓库根为 cwd，传入小 steps 保证快速。

from __future__ import annotations

import functools
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

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


def _cached_run(args: tuple[str, ...]) -> subprocess.CompletedProcess:
    return run_cli(args)


def _describe(proc: subprocess.CompletedProcess, args: tuple[str, ...]) -> str:
    return (
        f"command args={args!r} cwd={REPO_ROOT}\n"
        f"exit={proc.returncode}\n"
        f"--- stdout ---\n{proc.stdout}\n"
        f"--- stderr ---\n{proc.stderr}"
    )


def assert_exit_zero(proc: subprocess.CompletedProcess, args: tuple[str, ...]) -> None:
    """断言子进程退出码为 0，失败时打印完整调用现场。"""
    assert proc.returncode == 0, f"expected exit 0\n{_describe(proc, args)}"


def report_for(args: tuple[str, ...]) -> dict:
    """运行（带缓存）脚本并要求退出码 0，返回解析后的 JSON 报告。"""
    proc = _cached_run(args)
    assert_exit_zero(proc, args)
    try:
        payload = json.loads(proc.stdout)
    except json.JSONDecodeError as exc:  # noqa: PERF203
        pytest.fail(f"--json stdout is not a single JSON object ({exc})\n{_describe(proc, args)}")
    assert isinstance(payload, dict), f"JSON report must be an object, got {type(payload).__name__}"
    return payload


def primary_report() -> dict:
    """返回主运行参数（20 步 / warmup=1）下的 JSON 报告。"""
    return report_for(PRIMARY_ARGS)


# ---------------------------------------------------------------------------
# T-IK-14  script runnable + statistics + flags
# ---------------------------------------------------------------------------


def test_t_ik_14_1_runs_and_exits_zero_with_json_report() -> None:
    """T-IK-14：脚本可运行、退出码 0、stdout 是纯 JSON。"""
    proc = _cached_run(PRIMARY_ARGS)
    assert_exit_zero(proc, PRIMARY_ARGS)
    payload = json.loads(proc.stdout)  # 纯 JSON：混入人类文本会在此失败
    assert isinstance(payload, dict)


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
    # 退出码 0 与预算状态必须自洽（T-IK-16 的阈值本身不在 pytest 中断言）。
    assert report["within_budget"] is True, f"exit 0 but within_budget={report['within_budget']!r}"
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
    # 合理量级：solve 处于亚毫秒~数十毫秒区间；捕获秒/微秒单位错配与恒 0。
    assert p50 < 1000.0, f"solve p50 implausibly large (unit mix-up?): {solve}"


def test_t_ik_14_4_step_p99_is_finite_positive_and_covers_solve_p99() -> None:
    """T-IK-14：整步 p99 有限、为正且覆盖 solve p99。"""
    report = primary_report()
    step_p99 = float(report["step_time_ms"]["p99"])
    solve_p99 = float(report["solve_time_ms"]["p99"])

    assert step_p99 == step_p99 and step_p99 > 0.0, (
        f"step p99 must be finite and positive: {step_p99}"
    )
    assert step_p99 < 10000.0, f"step p99 implausibly large (unit mix-up?): {step_p99}"
    # 整步耗时应覆盖该步 IK 求解耗时（同一批样本的 p99）。
    assert step_p99 + 1e-9 >= solve_p99, (
        f"step p99 ({step_p99} ms) < solve p99 ({solve_p99} ms): step time must include the solve"
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


def test_t_ik_14_7_human_readable_run_exits_zero_without_json_flag() -> None:
    """T-IK-14：不带 --json 时人类摘要走 stdout 且退出码 0。"""
    args = ("--steps", "5", "--warmup", "0")
    proc = run_cli(args)
    assert_exit_zero(proc, args)
    assert proc.stdout.strip(), "expected a human-readable summary on stdout"


def test_t_ik_14_8_documented_uv_invocation_exits_zero() -> None:
    """T-IK-14：文档化的 `uv run --no-sync python` 入口可运行。"""
    if shutil.which("uv") is None:
        pytest.skip("uv not on PATH; cannot exercise the documented acceptance entry point")
    if not _uv_python_has_placo():
        pytest.skip("uv run --no-sync 的解释器没有 placo，跳过 uv 入口用例")
    args = ("--steps", "5", "--warmup", "0", "--json")
    proc = run_cli(args, use_uv=True)
    assert_exit_zero(proc, args)
    payload = json.loads(proc.stdout)
    assert isinstance(payload, dict) and payload["steps"] == 5


def test_t_ik_14_9_unusable_urdf_fails_loud_with_nonzero_exit() -> None:
    """T-IK-14：不可用 URDF 必须非零退出而不是静默成功。"""
    bad_urdf = "src/alohamini_cartesian/kinematics/assets/am_arm200/does_not_exist.urdf"
    args = ("--urdf", bad_urdf, "--steps", "5", "--warmup", "0", "--json")
    proc = run_cli(args)
    assert proc.returncode != 0, (
        f"a non-existent URDF must not silently succeed\n{_describe(proc, args)}"
    )


# ---------------------------------------------------------------------------
# T-IK-15  deterministic unreachable samples are counted, marked, not fatal
# ---------------------------------------------------------------------------


def test_t_ik_15_1_unreachable_samples_are_counted_not_crashed() -> None:
    """T-IK-15：目标流含不可达样本时计入失败且不崩溃。"""
    # 不崩溃：primary_report() 已要求退出码 0。
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
    cached = _cached_run(PRIMARY_ARGS)
    fresh = run_cli(PRIMARY_ARGS)
    assert_exit_zero(fresh, PRIMARY_ARGS)

    first = json.loads(cached.stdout)
    second = json.loads(fresh.stdout)

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
