#!/usr/bin/env python

"""单臂运动学接缝（``ArmIK`` / ``IKResult`` / ``ArmKinematicProfile``）的行为契约测试。

从 lerobot 仓库的 ``tests/robots/test_alohamini_arm_ik.py`` 搬运，模块路径按本库映射为
``alohamini_cartesian.control.arm_ik``（旧 ``lerobot.robots.alohamini.control.arm_ik``），
URDF 资产指向本库
``src/alohamini_cartesian/kinematics/assets/am_arm200/kinematics.urdf``。

placo 在本库中是未安装的可选依赖：只测纯数据契约（T-IK-8 / T-IK-9）与 placo 缺失路径
（T-IK-12 / T-IK-10.1）的用例照常运行，不依赖真 placo；真正需要 placo 数值求解的用例
（必须构造 ``ArmIK``，或调用 ``fk()`` / ``solve()``）在函数体首行用
``pytest.importorskip("placo", ...)`` 声明依赖，在无 placo 的环境里跳过而不是误报失败。
"""

# Behavior: 实施包 P4-A cycle 1 —— 契约层 T-IK-8 / T-IK-9 / T-IK-12 for the
# decoupled single-arm kinematics component used by P4-B's controller.
#
# Every case imports the module under test inside a helper (same pattern as
# tests/test_control_commands.py): a missing implementation fails each case
# with ImportError individually instead of erroring the whole module.
#
# Required behavior (adjudicated contract / design §3):
# - T-IK-8  IKResult contract: frozen dataclass, exact field set/order
#           (q_deg, ok, reason, position_error_m, orientation_error_deg,
#           iterations, solve_time_s), q_deg shape (6,), reason vocabulary
#           {None, "NON_FINITE", "JOINT_LIMIT", "RESIDUAL"} and the
#           ok=True  <=>  reason is None invariant. Construction-time
#           validation follows the project convention (design D017:
#           frozen pure data + explicit __post_init__ ValueError/TypeError,
#           no silent clamp), which is how the invariant is observable in a
#           placo-free environment.
# - T-IK-9  ArmKinematicProfile: frozen dataclass, default max_iterations=5 /
#           residual_limit_m=0.005, joint_names a frozen 6-tuple in the URDF
#           order.
# - T-IK-12 base-env import safety: module imports and Profile/IKResult are
#           constructible WITHOUT placo; fk()/solve() raise a guided
#           ImportError pointing at the `kinematics` extra; the module never
#           imports Client/Host/network code, and does not pull placo at
#           module-import time (lazy only).
#
# ENVIRONMENT NOTE (updated for this library): placo is NOT installed here, so
# the numeric cases (fk/solve) declare `pytest.importorskip("placo")` and skip;
# they never run against a stand-in solver. The placo-free guard cases are still
# exercised deterministically against a *simulated* absence by removing the
# optional dependency in-process with a sys.meta_path blocker (not a mock of the
# SUT), so those cases stay discriminating whether or not placo is installed,
# and stay controlled (no network, no subprocess, state restored after each
# case).
#
# Discriminative intent (a reasonable but WRONG implementation must fail):
# - a plain field-bag IKResult without __post_init__ validation (T-IK-8 shape /
#   vocab / invariant cases);
# - an ok/reason table that allows ok=True with a failure reason, or ok=False
#   with reason None (T-IK-8 invariant);
# - a mutable / defaulted Profile, or wrong default numbers (T-IK-9);
# - module-level `import placo` (would break base-env import, T-IK-12);
# - an unguided bare `ImportError` / `ModuleNotFoundError` from fk/solve
#   (T-IK-12 guard cases require the `kinematics` extra hint);
# - importing Client/Host/network code into the pure math component
#   (T-IK-12 AST case).
#
# Assumed seam reported to the main agent (task fixed module/classes/fields but
# not the construction-validation seam): the ok<=>reason invariant, the reason
# vocabulary and the q_deg shape are enforced at IKResult construction time via
# __post_init__ raising ValueError, mirroring design D017. If the implementer
# prefers enforcement elsewhere, the binding must be re-planned first.
#
# Cycle 1 is pure data / static analysis only: no placo, no FK/IK numerics, no
# hardware, no clock, no randomness. Expected values are frozen literals;
# nothing is read back from the implementation under test. Later cycles add
# numerics whose oracles are frozen external URDF-chain literals (see below),
# never the SUT's own output.

import ast
import contextlib
import dataclasses
import importlib
import importlib.abc
import inspect
import shutil
import sys
from collections.abc import Callable, Iterator
from decimal import Decimal
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

# Frozen contract vocabulary (design §3). Set equality, so an extra or renamed
# field/reason is a failure.
IKRESULT_FIELDS = (
    "q_deg",
    "ok",
    "reason",
    "position_error_m",
    "orientation_error_deg",
    "iterations",
    "solve_time_s",
)
PROFILE_FIELDS = (
    "urdf_path",
    "target_frame",
    "joint_names",
    "max_iterations",
    "residual_limit_m",
)
REASON_VOCAB = (None, "NON_FINITE", "JOINT_LIMIT", "RESIDUAL")
FAILURE_REASONS = ("NON_FINITE", "JOINT_LIMIT", "RESIDUAL")

JOINT_NAMES = (
    "left_shoulder_pan",
    "left_shoulder_lift",
    "left_elbow_flex",
    "left_wrist_flex",
    "left_wrist_yaw_joint",
    "left_wrist_roll",
)
TARGET_FRAME = "left_Fixed_Jaw"
_URDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "alohamini_cartesian"
    / "kinematics"
    / "assets"
    / "am_arm200"
    / "kinematics.urdf"
)
URDF_PATH = str(_URDF_PATH)

# Modules that the pure math component must never import, at any nesting level.
# never allowed anywhere
FORBIDDEN_IMPORT_ROOTS = frozenset({"socket", "http", "requests", "urllib", "urllib3", "grpc"})
# only allowed lazily (never at module-import time)
LAZY_ONLY_IMPORT_ROOTS = frozenset({"placo", "pinocchio", "pin"})
FORBIDDEN_IMPORT_SUBSTRINGS = ("alohamini_client", "alohamini_host", "alohamini.alohamini")

ARM_IK_MODULE = "alohamini_cartesian.control.arm_ik"


def load_arm_ik() -> SimpleNamespace:
    """Import the seam lazily so a missing module fails per case, not at collection."""
    from alohamini_cartesian.control.arm_ik import ArmIK, ArmKinematicProfile, IKResult

    return SimpleNamespace(ArmIK=ArmIK, ArmKinematicProfile=ArmKinematicProfile, IKResult=IKResult)


class _PlacoBlocker(importlib.abc.MetaPathFinder):
    """Make `import placo` fail as if the optional dependency were not installed."""

    def find_spec(self, fullname: str, path: object = None, target: object = None) -> object:
        """拦截 placo 及其子模块的导入，模拟可选依赖缺失。"""
        if fullname == "placo" or fullname.startswith("placo."):
            raise ModuleNotFoundError(f"No module named '{fullname}' (blocked by test)")
        return None


@contextlib.contextmanager
def placo_absent() -> Iterator[None]:
    """在 placo 不可见的进程中运行，并强制重新导入被测模块。

    Run with placo unpresent, forcing a fresh import of the module under test.
    """
    restorable = {
        name: module
        for name, module in sys.modules.items()
        if name == "placo" or name.startswith("placo.") or name == ARM_IK_MODULE
    }
    for name in list(restorable):
        del sys.modules[name]
    blocker = _PlacoBlocker()
    sys.meta_path.insert(0, blocker)
    try:
        yield
    finally:
        sys.meta_path.remove(blocker)
        sys.modules.pop(ARM_IK_MODULE, None)
        sys.modules.update(restorable)


def make_profile() -> tuple[SimpleNamespace, Any]:
    """返回被测模块与一个使用默认参数的 ArmKinematicProfile。"""
    arm_ik = load_arm_ik()
    return arm_ik, arm_ik.ArmKinematicProfile(
        urdf_path=URDF_PATH,
        target_frame=TARGET_FRAME,
        joint_names=JOINT_NAMES,
    )


def make_result(**overrides: Any) -> Any:
    """构造一条字段自洽的 IKResult；overrides 用于驱动契约用例。

    A consistent, fully-populated IKResult; overrides drive the contract cases.
    """
    arm_ik = load_arm_ik()
    kwargs = {
        "q_deg": np.zeros(6, dtype=float),
        "ok": True,
        "reason": None,
        "position_error_m": 0.0,
        "orientation_error_deg": 0.0,
        "iterations": 1,
        "solve_time_s": 0.001,
    }
    kwargs.update(overrides)
    return arm_ik.IKResult(**kwargs)


def import_roots_of(source: str) -> tuple[set[str], set[str]]:
    """(any-level module roots, top-level module roots) referenced by imports in `source`."""
    tree = ast.parse(source)
    any_level: set[str] = set()
    top_level: set[str] = set()

    def walk(node: ast.AST, is_top_level: bool) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                # A lazily-imported module inside a callable is allowed; recurse
                # to still catch violations of the never-import rule.
                walk(child, False)
                continue
            if isinstance(child, (ast.Import, ast.ImportFrom)):
                root = child.module if isinstance(child, ast.ImportFrom) else None
                if isinstance(child, ast.Import):
                    for alias in child.names:
                        any_level.add(alias.name)
                        if is_top_level:
                            top_level.add(alias.name)
                else:
                    if root:
                        any_level.add(root)
                        if is_top_level:
                            top_level.add(root)
                continue
            walk(child, is_top_level)

    walk(tree, True)
    return any_level, top_level


# ---------------------------------------------------------------------------
# T-IK-8  IKResult contract
# ---------------------------------------------------------------------------


def test_t_ik_8_1_ikresult_field_set_order_and_required() -> None:
    """T-IK-8.1：IKResult 的字段集合、顺序与全部必填。"""
    arm_ik = load_arm_ik()
    assert dataclasses.is_dataclass(arm_ik.IKResult)
    assert tuple(f.name for f in dataclasses.fields(arm_ik.IKResult)) == IKRESULT_FIELDS
    # Every field is required (no defaults): dropping one must be rejected.
    for name in IKRESULT_FIELDS:
        kwargs = {
            "q_deg": np.zeros(6),
            "ok": True,
            "reason": None,
            "position_error_m": 0.0,
            "orientation_error_deg": 0.0,
            "iterations": 1,
            "solve_time_s": 0.001,
        }
        del kwargs[name]
        with pytest.raises(TypeError):
            arm_ik.IKResult(**kwargs)


def test_t_ik_8_2_ikresult_is_frozen() -> None:
    """T-IK-8.2：IKResult 是冻结 dataclass，赋值被拒绝。"""
    result = make_result()
    with pytest.raises(dataclasses.FrozenInstanceError):
        result.ok = False
    with pytest.raises(dataclasses.FrozenInstanceError):
        result.reason = "RESIDUAL"


def test_t_ik_8_3_q_deg_must_be_shape_six() -> None:
    """T-IK-8.3：q_deg 必须是长度 6 的向量。"""
    result = make_result()
    assert np.asarray(result.q_deg).shape == (6,)
    for bad in (np.zeros(5), np.zeros(7), np.zeros((6, 1))):
        with pytest.raises((ValueError, TypeError)):
            make_result(q_deg=bad)


def test_t_ik_8_4_reason_vocabulary_and_ok_invariant_accepted() -> None:
    """T-IK-8.4：reason 词表与 ok 不变量的正向构造。"""
    # ok=True <=> reason is None ; ok=False <=> reason is a failure token.
    assert make_result(ok=True, reason=None).reason is None
    for reason in FAILURE_REASONS:
        result = make_result(ok=False, reason=reason)
        assert result.ok is False
        assert result.reason == reason


def test_t_ik_8_5_unknown_reason_token_rejected() -> None:
    """T-IK-8.5：未知 reason 词被拒绝。"""
    for bad in ("OK", "TIMEOUT", "RESIDUAL ", "", "Joint_Limit"):
        with pytest.raises(ValueError):
            make_result(ok=False, reason=bad)


def test_t_ik_8_6_ok_reason_invariant_enforced() -> None:
    """T-IK-8.6：ok 与 reason 的矛盾组合被拒绝。"""
    # ok=True with a failure reason is contradictory ...
    for reason in FAILURE_REASONS:
        with pytest.raises(ValueError):
            make_result(ok=True, reason=reason)
    # ... and ok=False with reason None is equally contradictory.
    with pytest.raises(ValueError):
        make_result(ok=False, reason=None)


def test_t_ik_8_7_ok_must_be_bool() -> None:
    """T-IK-8.7：ok 必须是 bool，而不是真值字符串。"""
    with pytest.raises((TypeError, ValueError)):
        make_result(ok="yes", reason=None)


# ---------------------------------------------------------------------------
# T-IK-9  ArmKinematicProfile
# ---------------------------------------------------------------------------


def test_t_ik_9_1_profile_field_set_order() -> None:
    """T-IK-9.1：ArmKinematicProfile 的字段集合与顺序。"""
    arm_ik = load_arm_ik()
    assert dataclasses.is_dataclass(arm_ik.ArmKinematicProfile)
    assert tuple(f.name for f in dataclasses.fields(arm_ik.ArmKinematicProfile)) == PROFILE_FIELDS


def test_t_ik_9_2_profile_defaults() -> None:
    """T-IK-9.2：profile 的 max_iterations / residual_limit_m 默认值。"""
    _, profile = make_profile()
    assert profile.max_iterations == 5
    assert profile.residual_limit_m == 0.005
    assert profile.urdf_path == URDF_PATH
    assert profile.target_frame == TARGET_FRAME


def test_t_ik_9_3_profile_is_frozen() -> None:
    """T-IK-9.3：profile 是冻结 dataclass。"""
    _, profile = make_profile()
    with pytest.raises(dataclasses.FrozenInstanceError):
        profile.max_iterations = 99
    with pytest.raises(dataclasses.FrozenInstanceError):
        profile.residual_limit_m = 1.0


def test_t_ik_9_4_joint_names_are_frozen_six_tuple_in_urdf_order() -> None:
    """T-IK-9.4：joint_names 是按 URDF 顺序的冻结六元组。"""
    _, profile = make_profile()
    assert isinstance(profile.joint_names, tuple)
    assert profile.joint_names == JOINT_NAMES
    assert len(profile.joint_names) == 6
    with pytest.raises(TypeError):
        profile.joint_names[0] = "mutated"


def test_t_ik_9_5_joint_names_wrong_length_rejected() -> None:
    """T-IK-9.5：joint_names 长度不为 6 时被拒绝。"""
    arm_ik = load_arm_ik()
    with pytest.raises((ValueError, TypeError)):
        arm_ik.ArmKinematicProfile(
            urdf_path=URDF_PATH,
            target_frame=TARGET_FRAME,
            joint_names=JOINT_NAMES[:5],
        )
    with pytest.raises((ValueError, TypeError)):
        arm_ik.ArmKinematicProfile(
            urdf_path=URDF_PATH,
            target_frame=TARGET_FRAME,
            joint_names=JOINT_NAMES + ("left_extra",),
        )


# ---------------------------------------------------------------------------
# T-IK-12  base-env import safety & placo guard
# ---------------------------------------------------------------------------


def test_t_ik_12_1_module_imports_with_placo_absent() -> None:
    """T-IK-12.1：placo 缺失时模块仍可导入。"""
    # Discriminates a module-level `import placo` (would break the pure-data
    # contract on a machine without the optional extra).
    with placo_absent():
        module = importlib.import_module(ARM_IK_MODULE)
        assert inspect.ismodule(module)
        assert module.IKResult is not None
        assert module.ArmKinematicProfile is not None


def test_t_ik_12_2_profile_and_result_constructible_with_placo_absent() -> None:
    """T-IK-12.2：placo 缺失时仍可构造 Profile 与 IKResult。"""
    with placo_absent():
        arm_ik = load_arm_ik()
        profile = arm_ik.ArmKinematicProfile(
            urdf_path=URDF_PATH, target_frame=TARGET_FRAME, joint_names=JOINT_NAMES
        )
        profile2 = dataclasses.replace(profile, max_iterations=7)
        assert profile2.max_iterations == 7
        result = make_result()
        assert np.asarray(result.q_deg).shape == (6,)


def test_t_ik_12_3_fk_raises_guided_import_error_with_placo_absent() -> None:
    """T-IK-12.3：placo 缺失时 fk 抛出带指引的 ImportError。"""
    with placo_absent():
        arm_ik = load_arm_ik()
        profile = arm_ik.ArmKinematicProfile(
            urdf_path=URDF_PATH, target_frame=TARGET_FRAME, joint_names=JOINT_NAMES
        )
        with pytest.raises(ImportError) as excinfo:
            arm_ik.ArmIK(profile).fk(np.zeros(6))
        assert "kinematics" in str(excinfo.value)


def test_t_ik_12_4_solve_raises_guided_import_error_with_placo_absent() -> None:
    """T-IK-12.4：placo 缺失时 solve 抛出带指引的 ImportError。"""
    limits = dict.fromkeys(JOINT_NAMES, (-180.0, 180.0))
    with placo_absent():
        arm_ik = load_arm_ik()
        profile = arm_ik.ArmKinematicProfile(
            urdf_path=URDF_PATH, target_frame=TARGET_FRAME, joint_names=JOINT_NAMES
        )
        with pytest.raises(ImportError) as excinfo:
            arm_ik.ArmIK(profile).solve(np.zeros(6), np.eye(4), limits)
        assert "kinematics" in str(excinfo.value)


def test_t_ik_12_5_module_has_no_client_host_or_network_imports() -> None:
    """T-IK-12.5：模块不含 Client/Host/网络导入，placo 仅惰性导入。"""
    module = importlib.import_module(ARM_IK_MODULE)
    source = Path(inspect.getfile(module)).read_text(encoding="utf-8")
    any_level, top_level = import_roots_of(source)

    for name in any_level:
        assert name.split(".")[0] not in FORBIDDEN_IMPORT_ROOTS, f"forbidden import: {name}"
        assert not any(bad in name for bad in FORBIDDEN_IMPORT_SUBSTRINGS), (
            f"forbidden import: {name}"
        )

    # placo must be lazy: importing the module must not import it at top level.
    for name in top_level:
        assert name.split(".")[0] not in LAZY_ONLY_IMPORT_ROOTS, (
            f"module-import-time placo dependency: {name}"
        )


# ===========================================================================
# P4-A cycle 2  --  T-IK-1 / T-IK-2 / T-IK-3  (placo-backed numerics)
# ===========================================================================
#
# These cases exercise the numerical behavior of `ArmIK.fk` / `ArmIK.solve`.
# placo is available in the designated runner (`uv run --no-sync`), so the
# cycle-1 `NotImplementedError(stub)` bodies are the expected RED target here.
#
# Frozen FK references below are the P3 five-pose spot-check table
# (`assets/am_arm200/fk_spot_check.py`, run against `kinematics.urdf`).  Each
# 4x4 matrix is the world pose of the `left_Fixed_Jaw` link origin and was
# independently reproduced by a hand-rolled URDF homogeneous-transform chain
# (product of per-joint `T_origin * R_axis(q)`), agreeing with placo to ~1e-12.
# Comparing against these literals (not against the implementation under test)
# keeps the oracle external.
#
# Units / seam facts assumed (reported to the main agent):
# - fk(q_deg) takes DEGREES; a rad/deg mix-up shifts every non-zero pose far
#   beyond the 1e-6 tolerance, so the five-pose comparison is unit-discriminating;
# - solve(q_current_deg, target_pose, joint_limits_deg) takes DEGREES for the
#   seed and limits and returns `q_deg` in degrees within those limits.
#
# Discriminative intent (a reasonable but WRONG implementation must fail):
# - rad/deg confusion inside fk/solve          -> T-IK-1 five-pose, T-IK-2;
# - wrong end frame (e.g. base/identity link)  -> T-IK-1 zero-pose;
# - solve returns the seed without solving      -> T-IK-2 independent FK reach
#   check (target sits > 1 cm from the seed, asserted as a test premise);
# - solve always reports ok=True               -> T-IK-3;
# - solve lies about position_error_m           -> T-IK-2 independent FK reach;
# - solution escapes the injected joint limits -> T-IK-2 limit assertions.

FK_ATOL = 1e-6
SOLVE_LIMITS_DEG = (-90.0, 90.0)
SOLVE_RESIDUAL_LIMIT_M = 0.005

# label -> (q_deg, frozen 4x4 world pose of left_Fixed_Jaw origin)
FROZEN_FK = {
    "zero": (
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        np.array(
            [
                [
                    -9.9999999999325417e-01,
                    -3.6732051070797607e-06,
                    -5.6047639474056372e-13,
                    -9.6632659786485679e-04,
                ],
                [0.0, 1.5258510711636084e-07, -9.9999999999998901e-01, -1.8807138731545400e-01],
                [
                    3.6732051076349145e-06,
                    -9.9999999999324274e-01,
                    -1.5258510711533145e-07,
                    1.3542222324406736e-01,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    ),
    "shoulder_pan_90": (
        (90.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        np.array(
            [
                [
                    -6.1232339956954593e-17,
                    1.5258510711636060e-07,
                    -9.9999999999998901e-01,
                    -1.8807138731545400e-01,
                ],
                [
                    9.9999999999325417e-01,
                    3.6732051070797607e-06,
                    5.6041516240060634e-13,
                    9.6632659786484530e-04,
                ],
                [
                    3.6732051076349145e-06,
                    -9.9999999999324274e-01,
                    -1.5258510711533145e-07,
                    1.3542222324406736e-01,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    ),
    "shoulder_lift_90": (
        (0.0, 90.0, 0.0, 0.0, 0.0, 0.0),
        np.array(
            [
                [
                    -9.9999999999325417e-01,
                    -5.6391808580021805e-13,
                    3.6732051036380697e-06,
                    -9.6579621214462098e-04,
                ],
                [
                    3.9968028886505635e-15,
                    -9.9999999999998879e-01,
                    -1.5258510717759314e-07,
                    -1.4391780305547221e-02,
                ],
                [
                    3.6732051036381117e-06,
                    -1.5258510717655118e-07,
                    9.9999999999324296e-01,
                    2.7981539086409452e-01,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    ),
    "elbow_flex_90": (
        (0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
        np.array(
            [
                [
                    -9.9999999999325417e-01,
                    -5.6391808580021805e-13,
                    3.6732051036380697e-06,
                    -9.6517292394001394e-04,
                ],
                [
                    3.9968028886505635e-15,
                    -9.9999999999998879e-01,
                    -1.5258510717759314e-07,
                    7.2951829533239754e-02,
                ],
                [
                    3.6732051036381117e-06,
                    -1.5258510717655118e-07,
                    9.9999999999324296e-01,
                    4.4950049883302079e-01,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    ),
    "mixed": (
        (45.0, -30.0, 60.0, -45.0, 30.0, -15.0),
        np.array(
            [
                [
                    -0.32634233437434434,
                    -0.794550587091443,
                    -0.5120449641855169,
                    -0.1320250564742237,
                ],
                [0.856672456909342, -0.47756139329530545, 0.1950574715481555, -0.08753140009528647],
                [-0.3995159350614395, -0.3749993069146805, 0.8365181034774408, 0.3892861579867697],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    ),
}

# label -> (q_seed_deg, q_target_deg, frozen_seed_pose, frozen_target_pose).
# The target differs from the seed on a single joint by a small step; every
# angle is well inside SOLVE_LIMITS_DEG.  Both 4x4 poses are frozen external
# references (same independent URDF-chain oracle as FROZEN_FK), NOT the SUT's
# own fk() output, so solve is exercised independently of the fk implementation.
SOLVE_REACHABLE_CASES = {
    "elbow_bend": (
        (10.0, -20.0, 30.0, -40.0, 20.0, -10.0),
        (10.0, -20.0, 40.0, -40.0, 20.0, -10.0),
        np.array(
            [
                [
                    -0.8506629550518582,
                    -0.2850714523290988,
                    -0.4417089584437003,
                    -0.05468105247520649,
                ],
                [0.5049154477382511, -0.6770137198916388, -0.53545570658437, -0.18441771913039604],
                [-0.1463998891315207, -0.6785180101850383, 0.719847471563813, 0.2851843953745279],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
        np.array(
            [
                [
                    -0.855999859313263,
                    -0.3036413127035612,
                    -0.41840912283951454,
                    -0.05150641820718307,
                ],
                [
                    0.4746495964689357,
                    -0.7823314803390116,
                    -0.40331775988970214,
                    -0.16641443445361637,
                ],
                [-0.20487069440881578, -0.5438376670388224, 0.8137988636525738, 0.3306117486653046],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    ),
    "shoulder_lift_step": (
        (-15.0, 25.0, -30.0, 35.0, -20.0, 15.0),
        (-15.0, 37.0, -30.0, 35.0, -20.0, 15.0),
        np.array(
            [
                [-0.8543045739585784, 0.05667292935261431, 0.5166738565014161, 0.05623194094553429],
                [-0.3186783829158416, 0.7281765773962062, -0.606797297615679, -0.13299609532665974],
                [
                    -0.41061878083643033,
                    -0.6830424959035423,
                    -0.6040241432378866,
                    0.01050445943813569,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
        np.array(
            [
                [-0.832698696482753, 0.09732311268936199, 0.5451064965787676, 0.06152140019304791],
                [-0.3993122321951968, 0.5764662075303031, -0.712907043586272, -0.1527362554506099],
                [
                    -0.38361780732287903,
                    -0.811304457840633,
                    -0.4411603502046657,
                    0.03661267355234813,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    ),
}

# Seed configuration and its frozen world pose (external oracle) reused by the
# unreachable-target cases.
SOLVE_UNREACHABLE_SEED_Q = (10.0, -20.0, 30.0, -40.0, 20.0, -10.0)
SOLVE_UNREACHABLE_SEED_POSE = np.array(
    [
        [-0.8506629550518582, -0.2850714523290988, -0.4417089584437003, -0.05468105247520649],
        [0.5049154477382511, -0.6770137198916388, -0.53545570658437, -0.18441771913039604],
        [-0.1463998891315207, -0.6785180101850383, 0.719847471563813, 0.2851843953745279],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


# label -> world-frame translation offset (metres) applied to a reachable pose
# to make it unreachable.  Arm span is ~0.3 m; these offsets are > 1 m.
SOLVE_UNREACHABLE_OFFSETS = {
    "far_diagonal": (1.0, 1.0, 1.0),
    "far_x": (5.0, 0.0, 0.0),
}


def make_arm_ik() -> tuple[SimpleNamespace, Any]:
    """返回被测模块与一个基于默认 profile 的 ArmIK 实例。"""
    arm_ik, profile = make_profile()
    return arm_ik, arm_ik.ArmIK(profile)


def fk_pose(q_deg: object) -> np.ndarray:
    """用 ArmIK.fk 计算给定关节角（度）下的 4x4 世界位姿。"""
    _arm_ik, arm = make_arm_ik()
    return np.asarray(arm.fk(np.asarray(q_deg, dtype=float)), dtype=float)


def limits_for(
    lo: float = SOLVE_LIMITS_DEG[0], hi: float = SOLVE_LIMITS_DEG[1]
) -> dict[str, tuple[float, float]]:
    """六个关节统一取同一个对称限位（度）。"""
    return dict.fromkeys(JOINT_NAMES, (lo, hi))


# ---------------------------------------------------------------------------
# T-IK-1  forward kinematics
# ---------------------------------------------------------------------------


def test_t_ik_1_1_fk_returns_valid_homogeneous_transform() -> None:
    """T-IK-1.1：fk 返回合法的齐次变换矩阵。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    pose = fk_pose([15.0, -25.0, 35.0, -20.0, 10.0, 5.0])
    assert pose.shape == (4, 4)
    assert np.all(np.isfinite(pose))
    assert np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9), pose[3]
    rotation = pose[:3, :3]
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)


def test_t_ik_1_2_fk_zero_pose_matches_frozen_left_fixed_jaw_origin() -> None:
    """T-IK-1.2：零位 FK 与冻结的 left_Fixed_Jaw 原点一致。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    q_deg, expected = FROZEN_FK["zero"]
    pose = fk_pose(q_deg)
    assert np.allclose(pose, expected, atol=FK_ATOL), (
        f"zero-pose FK mismatch:\n{pose}\n!=\n{expected}"
    )
    # The end frame is the left_Fixed_Jaw link origin, not the base/identity:
    # the frozen position is a distinct, non-zero point ~0.23 m from the base.
    position = pose[:3, 3]
    assert np.allclose(
        position,
        (-9.6632659786485679e-04, -1.8807138731545400e-01, 1.3542222324406736e-01),
        atol=FK_ATOL,
    )
    assert np.linalg.norm(position) > 0.1


@pytest.mark.parametrize("label", list(FROZEN_FK))
def test_t_ik_1_3_fk_matches_p3_five_pose_reference(label: str) -> None:
    """T-IK-1.3：FK 与 P3 五姿态冻结参考逐一一致。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    q_deg, expected = FROZEN_FK[label]
    pose = fk_pose(q_deg)
    assert np.allclose(pose, expected, atol=FK_ATOL), (
        f"pose '{label}' mismatch:\n{pose}\n!=\n{expected}"
    )


# ---------------------------------------------------------------------------
# T-IK-2  solve on a near-reachable FK-derived target
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("label", list(SOLVE_REACHABLE_CASES))
def test_t_ik_2_1_solve_reaches_fk_derived_nearby_target(label: str) -> None:
    """T-IK-2.1：solve 能到达 FK 派生的近邻目标。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    q_seed_deg, _q_target_deg, seed_pose, target_pose = SOLVE_REACHABLE_CASES[label]
    seed = np.asarray(q_seed_deg, dtype=float)
    limits = limits_for()

    seed_position = seed_pose[:3, 3]
    target_position = target_pose[:3, 3]

    # Test premise: the target is a small but non-trivial move (> 1 cm), so an
    # implementation that simply echoes the seed cannot satisfy the residual
    # bound or the independent reach check below.
    displacement = float(np.linalg.norm(target_position - seed_position))
    assert displacement > 0.01, f"test premise broken: target only {displacement:.4f} m from seed"

    result = arm.solve(seed, target_pose, limits)

    assert result.ok is True, f"expected ok=True, got ok={result.ok!r} reason={result.reason!r}"
    assert result.reason is None
    assert result.position_error_m <= SOLVE_RESIDUAL_LIMIT_M, result.position_error_m

    q_deg = np.asarray(result.q_deg, dtype=float)
    assert q_deg.shape == (6,)
    assert np.all(np.isfinite(q_deg))
    for name, value in zip(JOINT_NAMES, q_deg, strict=True):
        lo, hi = limits[name]
        assert lo <= value <= hi, f"{name}={value} outside injected limits ({lo}, {hi})"

    # Independent reach check: the returned configuration must actually place
    # the end frame on the target (catches a bogus position_error_m).
    reached_position = np.asarray(arm.fk(q_deg), dtype=float)[:3, 3]
    reached_error = float(np.linalg.norm(reached_position - target_position))
    assert reached_error <= SOLVE_RESIDUAL_LIMIT_M + 1e-6, (
        f"returned q does not reach the target: {reached_error:.6f} m "
        f"(reported {result.position_error_m})"
    )


# ---------------------------------------------------------------------------
# T-IK-3  solve on an unreachable target
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("label", list(SOLVE_UNREACHABLE_OFFSETS))
def test_t_ik_3_1_unreachable_target_reports_residual_failure(label: str) -> None:
    """T-IK-3.1：不可达目标报 RESIDUAL 失败。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_UNREACHABLE_SEED_Q, dtype=float)
    limits = limits_for(lo=-170.0, hi=170.0)

    target_pose = SOLVE_UNREACHABLE_SEED_POSE.copy()
    offset = np.asarray(SOLVE_UNREACHABLE_OFFSETS[label], dtype=float)
    assert float(np.linalg.norm(offset)) > 1.0  # far beyond the ~0.3 m arm span
    target_pose[:3, 3] += offset

    result = arm.solve(seed, target_pose, limits)

    assert result.ok is False, (
        f"unreachable target must not report ok=True (reason={result.reason!r})"
    )
    assert result.reason == "RESIDUAL", result.reason
    assert result.position_error_m > SOLVE_RESIDUAL_LIMIT_M, result.position_error_m


# ===========================================================================
# P4-A cycle 3  --  T-IK-4 / T-IK-5 / T-IK-6 / T-IK-13 (failure semantics)
# ===========================================================================
#
# Cycle 3 refines the failure semantics of `ArmIK.solve`:
# - T-IK-4  a physically reachable target that only needs to *exceed the
#           injected joint limits* must fail with reason "JOINT_LIMIT" (not the
#           catch-all "RESIDUAL"), while a target reachable inside those same
#           limits is unaffected;
# - T-IK-5  a NON-FINITE q_seed / target_pose must fail cleanly with reason
#           "NON_FINITE" (no bare exception escaping solve, no polluted q_deg);
# - T-IK-6  the orientation goal is SOFT (default `orientation_weight=0.01`,
#           a keyword-only solve argument per design §3): on a target where
#           position and orientation compete, position must still be met
#           (residual <= threshold, ok=True) and the orientation error is only
#           recorded, never a rejection reason;
# - T-IK-13 every returned IKResult exposes a finite, non-negative
#           `solve_time_s` float and `iterations >= 1` (existence only, no
#           performance gate).
#
# Frozen target poses below are the product of an independent, hand-rolled URDF
# homogeneous-transform chain (`T_origin * R_axis(q)` per joint, parsed by hand
# from `kinematics.urdf`), NOT the SUT's own fk() output.  The chain was first
# validated by reproducing the cycle-2 frozen seed pose to ~1e-16, so these new
# literals keep the oracle external.
#
# Assumed seams reported to the main agent (the task fixed the reasons but not
# every observable detail):
# - on a NON_FINITE failure the returned `q_deg` is required to be finite (the
#   "no polluted solution" clause); its exact value is unspecified;
# - `iterations >= 1` is asserted on the new NON_FINITE path too, per the
#   acceptance wording ("iterations >= 1");
# - the synthetic limits are deliberately NOT physical limits: they are a tiny
#   +/-5 deg window purely to force the over-limit condition, and are labelled as
#   such.
#
# Discriminative intent (a reasonable but WRONG implementation must fail):
# - one catch-all failure reason ("RESIDUAL")            -> T-IK-4;
# - a classifer that flags JOINT_LIMIT whenever the solution sits on a limit
#   boundary (would also mis-flag the physically-unreachable T-IK-3 target,
#   whose solution is pinned at +/-170 deg)               -> T-IK-3 + T-IK-4;
# - NaN/Inf passing through to the solver (raises RuntimeError today) or being
#   returned as a polluted q_deg                          -> T-IK-5;
# - treating orientation as a hard constraint / rejecting on orientation error,
#   or silently zeroing the recorded orientation error    -> T-IK-6;
# - dropping/omitting the timing fields on a failure return -> T-IK-13.

ORIENTATION_WEIGHT_DEFAULT = 0.01
SYNTHETIC_LIMITS_HALF_WIDTH_DEG = 5.0

# Seed / target configs.  SOLVE_UNREACHABLE_SEED_Q is reused from cycle 2.
SOLVE_SEED_Q = SOLVE_UNREACHABLE_SEED_Q

# World pose of `left_Fixed_Jaw` at q = (10, -20, 45, -40, 20, -10) deg.
# Physically reachable (the wide-limit premise below asserts ok=True), but it
# requires elbow_flex = 45 deg, i.e. > 1.5x the +/-5 deg synthetic window.
FROZEN_OVER_LIMIT_TARGET_POSE = np.array(
    [
        [-0.8593112659133991, -0.31132779182580217, -0.4057821512969504, -0.049415875459843814],
        [0.4558702476667393, -0.8259252310339994, -0.3317074464550055, -0.15455882892517672],
        [-0.2318759702223947, -0.4700239555366349, 0.8516519333948103, 0.35195834637176937],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

# World pose of `left_Fixed_Jaw` at q = (10, -20, 32, -40, 20, -10) deg: the
# elbow stays at 32 deg, i.e. inside the +/-5 deg synthetic window.
FROZEN_WITHIN_LIMIT_TARGET_POSE = np.array(
    [
        [-0.8515872107963393, -0.28910757782741936, -0.43728255264859794, -0.05415708620649713],
        [0.4996739890852766, -0.6999042625410901, -0.510352748506775, -0.18144635611006876],
        [-0.15850907557524235, -0.6531085910624476, 0.7404890554496445, 0.294514142603057],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

# Position/orientation competition target: same position as the frozen seed pose
# but with the orientation rotated by Ry(150 deg) about the world Y axis (via an
# external rotation matrix, so this stays independent of the SUT).  The arm can
# hold the position but cannot satisfy that orientation there, so a soft
# orientation goal must keep the position and merely record the error.
FROZEN_POSITION_ORIENTATION_COMPETITION_POSE = np.array(
    [
        [0.663495784567489, -0.09237988548179495, 0.7424549148733158, -0.05468105247520649],
        [0.5049154477382511, -0.6770137198916388, -0.53545570658437, -0.18441771913039604],
        [0.5521175006250513, 0.730149559910061, -0.4025517180024083, 0.2851843953745279],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def synthetic_narrow_limits() -> dict[str, tuple[float, float]]:
    """A SYNTHETIC +/-5 deg window around the seed (NOT a real hardware limit)."""
    half = SYNTHETIC_LIMITS_HALF_WIDTH_DEG
    return {
        name: (seed - half, seed + half)
        for name, seed in zip(JOINT_NAMES, SOLVE_SEED_Q, strict=True)
    }


# ---------------------------------------------------------------------------
# T-IK-4  JOINT_LIMIT vs RESIDUAL
# ---------------------------------------------------------------------------


def test_t_ik_4_1_physically_reachable_but_over_synthetic_limits_reports_joint_limit() -> None:
    """T-IK-4.1：物理可达但超出注入限位时报 JOINT_LIMIT。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)

    # Premise: the target IS physically reachable -- with wide limits it solves.
    # If this ever breaks the case stops being a "needs over-limit" case, so it
    # is asserted rather than assumed.
    wide = arm.solve(seed, FROZEN_OVER_LIMIT_TARGET_POSE, limits_for())
    assert wide.ok is True, (
        f"test premise broken: target unreachable even with wide limits ({wide.reason!r})"
    )

    result = arm.solve(seed, FROZEN_OVER_LIMIT_TARGET_POSE, synthetic_narrow_limits())

    assert result.ok is False, "over-limit target must not report ok=True"
    assert result.reason == "JOINT_LIMIT", (
        f"reachable-but-over-limit target must be JOINT_LIMIT, got {result.reason!r}"
    )
    assert result.position_error_m > SOLVE_RESIDUAL_LIMIT_M, result.position_error_m


def test_t_ik_4_2_within_synthetic_limits_target_is_unaffected() -> None:
    """T-IK-4.2：限位内的目标不受影响。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    narrow = synthetic_narrow_limits()

    result = arm.solve(seed, FROZEN_WITHIN_LIMIT_TARGET_POSE, narrow)

    assert result.ok is True, f"within-limit target must not be rejected (reason={result.reason!r})"
    assert result.reason is None
    assert result.position_error_m <= SOLVE_RESIDUAL_LIMIT_M, result.position_error_m
    q_deg = np.asarray(result.q_deg, dtype=float)
    for name, value in zip(JOINT_NAMES, q_deg, strict=True):
        lo, hi = narrow[name]
        assert lo <= value <= hi, f"{name}={value} outside synthetic limits ({lo}, {hi})"


# ---------------------------------------------------------------------------
# T-IK-5  NON_FINITE inputs
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "label",
    ["seed_nan", "seed_inf", "target_position_nan", "target_position_inf", "target_rotation_nan"],
)
def test_t_ik_5_1_non_finite_inputs_report_non_finite_failure(label: str) -> None:
    """T-IK-5.1：非有限输入报 NON_FINITE 失败且不污染 q_deg。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    target_pose = FROZEN_OVER_LIMIT_TARGET_POSE.copy()

    if label == "seed_nan":
        seed = seed.copy()
        seed[2] = np.nan
    elif label == "seed_inf":
        seed = seed.copy()
        seed[0] = np.inf
    elif label == "target_position_nan":
        target_pose[0, 3] = np.nan
    elif label == "target_position_inf":
        target_pose[1, 3] = np.inf
    elif label == "target_rotation_nan":
        target_pose[2, 2] = np.nan
    else:  # pragma: no cover - guards a typo in the parametrisation
        raise AssertionError(label)

    # Must not raise a bare exception: a non-finite input is a normal, explicit
    # failure that is reported through the IKResult contract.
    result = arm.solve(seed, target_pose, limits_for())

    assert result.ok is False, "non-finite input must not report ok=True"
    assert result.reason == "NON_FINITE", result.reason
    q_deg = np.asarray(result.q_deg, dtype=float)
    assert q_deg.shape == (6,)
    assert np.all(np.isfinite(q_deg)), f"polluted (non-finite) solution returned: {q_deg}"


# ---------------------------------------------------------------------------
# T-IK-6  soft orientation constraint
# ---------------------------------------------------------------------------


def test_t_ik_6_1_solve_exposes_default_orientation_weight_of_0_01() -> None:
    """T-IK-6.1：solve 暴露默认 0.01 的仅关键字方位权重。"""
    arm_ik = load_arm_ik()
    signature = inspect.signature(arm_ik.ArmIK.solve)

    assert "orientation_weight" in signature.parameters, signature
    parameter = signature.parameters["orientation_weight"]
    assert parameter.default == ORIENTATION_WEIGHT_DEFAULT, parameter.default
    assert parameter.kind is inspect.Parameter.KEYWORD_ONLY, parameter.kind


def test_t_ik_6_2_explicit_soft_orientation_weight_keeps_position_and_records_error() -> None:
    """T-IK-6.2：显式软方位权重下位置达标并记录方位误差。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)

    result = arm.solve(
        seed,
        FROZEN_POSITION_ORIENTATION_COMPETITION_POSE,
        limits_for(),
        orientation_weight=ORIENTATION_WEIGHT_DEFAULT,
    )

    assert result.ok is True, (
        f"soft orientation must not reject the target (reason={result.reason!r})"
    )
    assert result.reason is None
    assert result.position_error_m <= SOLVE_RESIDUAL_LIMIT_M, result.position_error_m
    assert np.isfinite(result.orientation_error_deg)
    # The two goals genuinely compete here, so the recorded orientation error is
    # non-zero -- and it is NOT a rejection reason (ok is True above).
    assert result.orientation_error_deg > 1.0, result.orientation_error_deg


def test_t_ik_6_3_default_call_matches_explicit_default_weight() -> None:
    """T-IK-6.3：默认调用与显式默认权重等价。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)

    result = arm.solve(seed, FROZEN_POSITION_ORIENTATION_COMPETITION_POSE, limits_for())

    assert result.ok is True, f"default soft orientation must not reject (reason={result.reason!r})"
    assert result.reason is None
    assert result.position_error_m <= SOLVE_RESIDUAL_LIMIT_M, result.position_error_m
    assert result.orientation_error_deg > 1.0, result.orientation_error_deg


# ---------------------------------------------------------------------------
# T-IK-13  timing / iteration fields
# ---------------------------------------------------------------------------


def assert_sane_timing(result: object) -> None:
    """断言 solve_time_s 与 iterations 满足存在性要求。"""
    assert isinstance(result.solve_time_s, float), type(result.solve_time_s)
    assert np.isfinite(result.solve_time_s), result.solve_time_s
    assert result.solve_time_s >= 0.0, result.solve_time_s
    assert isinstance(result.iterations, (int, np.integer)), type(result.iterations)
    assert not isinstance(result.iterations, bool)
    assert result.iterations >= 1, result.iterations


def test_t_ik_13_1_success_result_exposes_sane_timing() -> None:
    """T-IK-13.1：成功结果的计时与迭代字段合理。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    result = arm.solve(seed, FROZEN_OVER_LIMIT_TARGET_POSE, limits_for())
    assert result.ok is True, result.reason
    assert_sane_timing(result)


def test_t_ik_13_2_residual_failure_result_exposes_sane_timing() -> None:
    """T-IK-13.2：RESIDUAL 失败结果的计时与迭代字段合理。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    target_pose = SOLVE_UNREACHABLE_SEED_POSE.copy()
    target_pose[:3, 3] += np.asarray((1.0, 1.0, 1.0), dtype=float)
    result = arm.solve(seed, target_pose, limits_for(lo=-170.0, hi=170.0))
    assert result.reason == "RESIDUAL", result.reason
    assert_sane_timing(result)


def test_t_ik_13_3_non_finite_failure_result_exposes_sane_timing() -> None:
    """T-IK-13.3：NON_FINITE 失败结果的计时与迭代字段合理。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    seed = seed.copy()
    seed[2] = np.nan
    result = arm.solve(seed, FROZEN_OVER_LIMIT_TARGET_POSE, limits_for())
    assert result.reason == "NON_FINITE", result.reason
    assert_sane_timing(result)


# ===========================================================================
# P4-A cycle 4  --  T-IK-7 / T-IK-10 / T-IK-11  (continuity & robustness)
# ===========================================================================
#
# Cycle 4 locks in the continuity and robustness behaviors of `ArmIK.solve`:
# - T-IK-7  continuity: a chain-seeded walk (each accepted solution becomes the
#           next solve's seed) along +/-X/+/-Y/+/-Z in small Cartesian steps
#           keeps every accepted step's largest per-joint change <= 2.0 deg
#           (design "2.0 deg/cycle") with no branch jump.  A failed step is
#           explicitly marked and does not break the chain (the last accepted
#           solution remains a valid seed).
#
#           ADJUDICATED (reported to the main agent): `ArmIK.solve` does NOT
#           hard-reject a large jump; the test only *observes* continuity.  The
#           walk is anchored at the established mid-workspace reference pose
#           SOLVE_SEED_Q (the pose used by cycles 2/3).  The all-zero "home"
#           pose is deliberately NOT used: it sits near a wrist singularity
#           where even a 1 mm move already needs several degrees of joint
#           motion, so it is not a "safe pose" for the Gate D 2.0 deg metric.
# - T-IK-10 constructor-time validation: `ArmIK(profile)` rejects a missing
#           URDF path (ValueError/FileNotFoundError naming the path) and a URDF
#           that lacks one of the profile's joints (error naming the joint) at
#           CONSTRUCTION time, not deferred to the first fk/solve.
# - T-IK-11 failure isolation: a failed `solve` never writes into the caller's
#           q_seed / target_pose buffers, and repeating an identical call
#           yields identical q_deg / ok / reason / residual / orientation /
#           iterations.  solve_time_s is the one field intentionally NOT pinned
#           (wall-clock), per the cycle-3 T-IK-13 "existence only" rule.
#
# Discriminative intent (a reasonable but WRONG implementation must fail):
# - an implementation that ignores the incoming seed (fixed seed / zero seed)
#   and solves every target independently -> a branch jump > 2.0 deg on the +Y
#   leg of the chain walk (measured at the 1 mm / 40-step calibration: fixed
#   seed 2.26 deg, zero seed 2.95 deg, versus <= 0.61 deg for chain seeding);
# - a constructor that stores the profile blindly and only touches the URDF on
#   the first fk/solve -> both T-IK-10 cases fail (the constructor must raise);
# - a solve that rewrites/normalises the seed in place, clamps it with
#   `q_seed[...] = ...`, or seeds from a module-level buffer -> T-IK-11_1;
# - a solve that leaks process state (a reused global solver, a
#   wall-clock-dependent iteration count) -> T-IK-11_2.
#
# External-oracle discipline: targets are built from frozen cycle-2/3
# references and from `arm.fk` of the chain's own accepted solution -- the
# chain walk is inherently a closed-loop observation, so no frozen literal is
# ever produced by the code under test.

CHAIN_SEED_Q_DEG = SOLVE_SEED_Q
CHAIN_STEP_M = 0.001
CHAIN_STEPS = 40
MAX_ADJACENT_JOINT_STEP_DEG = 2.0
CHAIN_DIRECTIONS = {
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "+Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
}


def chain_walk(
    direction: object,
    step_m: float = CHAIN_STEP_M,
    n_steps: int = CHAIN_STEPS,
    q0: object = CHAIN_SEED_Q_DEG,
) -> tuple[list[tuple], float]:
    """Chain-seeded walk; returns (records, max_adjacent_delta_deg).

    Each record is ``(step_index, ok, reason, q_deg, adjacent_delta_deg)`` where
    ``adjacent_delta_deg`` is the largest per-joint change relative to the
    current seed (the last accepted solution).  A failed step is recorded but
    does not advance the seed, mirroring how a controller must keep the last
    accepted solution when a step is rejected.
    """
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(q0, dtype=float)
    base_pose = np.asarray(arm.fk(seed), dtype=float)
    unit = np.asarray(direction, dtype=float)
    unit = unit / np.linalg.norm(unit)
    limits = limits_for(lo=-170.0, hi=170.0)

    accepted = seed.copy()
    records = []
    max_delta = 0.0
    for i in range(1, n_steps + 1):
        target = base_pose.copy()
        target[:3, 3] = base_pose[:3, 3] + unit * step_m * i
        result = arm.solve(accepted, target, limits)
        q_sol = np.asarray(result.q_deg, dtype=float)
        adjacent = float(np.max(np.abs(q_sol - accepted)))
        records.append((i, bool(result.ok), result.reason, q_sol, adjacent))
        if result.ok:
            max_delta = max(max_delta, adjacent)
            accepted = q_sol
    return records, max_delta


# ---------------------------------------------------------------------------
# T-IK-7  chain-seeded continuity
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("direction_label", list(CHAIN_DIRECTIONS))
def test_t_ik_7_1_chain_seeded_small_steps_stay_continuous(direction_label: str) -> None:
    """T-IK-7.1：链式播种的小步长行走保持连续。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    records, max_delta = chain_walk(CHAIN_DIRECTIONS[direction_label])

    # Premise: this is a non-trivial translation, and every step inside the
    # small reachable window is accepted (otherwise the observation is partial).
    assert CHAIN_STEPS * CHAIN_STEP_M > 0.02
    failed = [r for r in records if not r[1]]
    assert not failed, f"{direction_label}: unexpected failed steps {failed}"

    # Premise: the chain actually moved the configuration (not an echo of the
    # seed), so a branch jump would be observable at all.
    seed = np.asarray(CHAIN_SEED_Q_DEG, dtype=float)
    assert float(np.max(np.abs(records[-1][3] - seed))) > 1.0

    for i, _ok, reason, q_sol, adjacent in records:
        assert reason is None, (i, reason)
        assert np.all(np.isfinite(q_sol))
        assert adjacent <= MAX_ADJACENT_JOINT_STEP_DEG, (
            f"{direction_label} step {i}: branch jump {adjacent:.4f} deg exceeds "
            f"{MAX_ADJACENT_JOINT_STEP_DEG} deg"
        )
    assert max_delta <= MAX_ADJACENT_JOINT_STEP_DEG


def test_t_ik_7_2_failed_step_is_marked_and_chain_survives() -> None:
    """T-IK-7.2：失败步被标记且链可从上次接受解继续。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    limits = limits_for(lo=-170.0, hi=170.0)
    seed = np.asarray(CHAIN_SEED_Q_DEG, dtype=float)
    base_pose = np.asarray(arm.fk(seed), dtype=float)

    # 1) one accepted small step becomes the "last accepted solution".
    near = base_pose.copy()
    near[:3, 3] = base_pose[:3, 3] + np.asarray((0.0, 0.0, CHAIN_STEP_M), dtype=float)
    first = arm.solve(seed, near, limits)
    assert first.ok is True, first.reason
    accepted = np.asarray(first.q_deg, dtype=float)

    # 2) an unreachable target is explicitly marked and leaves the previously
    #    accepted solution / caller buffer untouched.
    seed_before = seed.copy()
    far = base_pose.copy()
    far[:3, 3] += np.asarray((1.0, 1.0, 1.0), dtype=float)
    failed = arm.solve(seed, far, limits)
    assert failed.ok is False
    assert failed.reason == "RESIDUAL", failed.reason
    assert failed.position_error_m > SOLVE_RESIDUAL_LIMIT_M
    assert np.asarray(failed.q_deg).shape == (6,)
    assert np.all(np.isfinite(np.asarray(failed.q_deg, dtype=float)))
    assert np.array_equal(seed, seed_before), "the failed solve mutated its seed argument"

    # 3) the chain resumes from the last accepted solution: the next small
    #    target still solves, and stays continuous with the accepted solution.
    resume = base_pose.copy()
    resume[:3, 3] = base_pose[:3, 3] + np.asarray((0.0, 0.0, 2.0 * CHAIN_STEP_M), dtype=float)
    resumed = arm.solve(accepted, resume, limits)
    assert resumed.ok is True, resumed.reason
    delta = float(np.max(np.abs(np.asarray(resumed.q_deg, dtype=float) - accepted)))
    assert delta <= MAX_ADJACENT_JOINT_STEP_DEG, f"chain did not resume smoothly: {delta:.4f} deg"


# ---------------------------------------------------------------------------
# T-IK-10  constructor-time validation
# ---------------------------------------------------------------------------


def test_t_ik_10_1_nonexistent_urdf_path_raises_at_construction(tmp_path: Path) -> None:
    """T-IK-10.1：URDF 路径不存在时在构造期报错并点名路径。"""
    arm_ik = load_arm_ik()
    missing = tmp_path / "does_not_exist_arm.urdf"
    assert not missing.exists()

    profile = arm_ik.ArmKinematicProfile(
        urdf_path=str(missing), target_frame=TARGET_FRAME, joint_names=JOINT_NAMES
    )
    with pytest.raises((ValueError, FileNotFoundError)) as excinfo:
        arm_ik.ArmIK(profile)
    assert str(missing) in str(excinfo.value), (
        f"construction error must name the offending URDF path, got: {excinfo.value!r}"
    )


def test_t_ik_10_2_urdf_missing_required_joint_raises_at_construction(tmp_path: Path) -> None:
    """T-IK-10.2：URDF 缺少必需关节时在构造期报错并点名关节。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    import xml.etree.ElementTree as ET

    arm_ik = load_arm_ik()
    tree = ET.parse(URDF_PATH)
    renamed = 0
    for joint in tree.getroot().findall("joint"):
        if joint.get("name") == "left_elbow_flex":
            joint.set("name", "left_elbow_flex_renamed")
            renamed += 1
    assert renamed == 1, "fixture drift: left_elbow_flex not found in the URDF"
    malformed = tmp_path / "missing_left_elbow_flex.urdf"
    tree.write(malformed)
    assert malformed.exists()

    profile = arm_ik.ArmKinematicProfile(
        urdf_path=str(malformed), target_frame=TARGET_FRAME, joint_names=JOINT_NAMES
    )
    with pytest.raises((ValueError, KeyError, RuntimeError)) as excinfo:
        arm_ik.ArmIK(profile)
    assert "left_elbow_flex" in str(excinfo.value), (
        f"construction error must name the missing joint, got: {excinfo.value!r}"
    )


def test_t_ik_10_3_valid_urdf_still_constructs() -> None:
    """T-IK-10.3：合法 URDF 仍可正常构造。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    # Guard against over-eager validation: the real profile must still build.
    arm_ik, profile = make_profile()
    arm = arm_ik.ArmIK(profile)
    assert isinstance(arm, arm_ik.ArmIK)


# ---------------------------------------------------------------------------
# T-IK-11  failure isolation & determinism
# ---------------------------------------------------------------------------


def make_solve_case(label: str) -> tuple[Any, ...]:
    """(arm, seed, target_pose, limits, expected_ok, expected_reason) for label.

    Inputs are frozen cycle-2/3 references only (never SUT output).
    """
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    if label == "success":
        return arm, seed, FROZEN_OVER_LIMIT_TARGET_POSE.copy(), limits_for(), True, None
    if label == "residual":
        target = SOLVE_UNREACHABLE_SEED_POSE.copy()
        target[:3, 3] += np.asarray((1.0, 1.0, 1.0), dtype=float)
        return arm, seed, target, limits_for(lo=-170.0, hi=170.0), False, "RESIDUAL"
    if label == "joint_limit":
        return (
            arm,
            seed,
            FROZEN_OVER_LIMIT_TARGET_POSE.copy(),
            synthetic_narrow_limits(),
            False,
            "JOINT_LIMIT",
        )
    if label == "non_finite":
        seed = seed.copy()
        seed[2] = np.nan
        return arm, seed, FROZEN_OVER_LIMIT_TARGET_POSE.copy(), limits_for(), False, "NON_FINITE"
    raise AssertionError(f"unknown case {label!r}")


@pytest.mark.parametrize(
    ("label", "expected_reason"),
    [("residual", "RESIDUAL"), ("joint_limit", "JOINT_LIMIT"), ("non_finite", "NON_FINITE")],
)
def test_t_ik_11_1_failed_solve_does_not_mutate_caller_inputs(
    label: str, expected_reason: str
) -> None:
    """T-IK-11.1：失败的 solve 不写回调用方的 seed/target 缓冲。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    arm, seed, target, limits, expected_ok, reason = make_solve_case(label)
    assert expected_ok is False and reason == expected_reason
    # float64 so solve's np.asarray(..., dtype=float) aliases the caller's
    # buffer instead of copying it; an in-place write then becomes observable.
    assert seed.dtype == np.float64

    seed_before = seed.copy()
    target_before = target.copy()

    result = arm.solve(seed, target, limits)

    assert result.ok is False
    assert result.reason == expected_reason, result.reason
    assert np.array_equal(seed, seed_before, equal_nan=True), "solve wrote into the caller's q_seed"
    assert np.array_equal(target, target_before, equal_nan=True), (
        "solve wrote into the caller's target_pose"
    )


@pytest.mark.parametrize("label", ["success", "residual", "joint_limit", "non_finite"])
def test_t_ik_11_2_repeated_identical_calls_are_deterministic(label: str) -> None:
    """T-IK-11.2：重复的相同调用结果确定。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    arm, seed, target, limits, _expected_ok, _expected_reason = make_solve_case(label)

    # Fresh copies per call: compare two independent calls with equal-valued
    # inputs, never aliased buffers.
    first = arm.solve(seed.copy(), target.copy(), limits)
    second = arm.solve(seed.copy(), target.copy(), limits)

    assert first.ok == second.ok
    assert first.reason == second.reason
    assert first.iterations == second.iterations
    assert np.array_equal(np.asarray(first.q_deg), np.asarray(second.q_deg)), (
        f"non-deterministic q_deg for {label}: {first.q_deg} vs {second.q_deg}"
    )
    assert first.position_error_m == second.position_error_m
    assert first.orientation_error_deg == second.orientation_error_deg


# ===========================================================================
# P4-A cycle 6  --  F2 input-validation alignment
# ===========================================================================
#
# Final-review fix round F2 aligns `ArmIK` input validation with the module's
# explicit-failure convention (D017: frozen data + explicit ValueError, never a
# silent/lazy failure):
# - `solve` validates `target_pose` is a (4, 4) homogeneous transform and
#   raises a clear ValueError otherwise, instead of letting placo throw an
#   opaque ArgumentError/RuntimeError deep in the solver;
# - `solve` rejects a `joint_limits_deg` dict missing one of the profile's
#   joints with a clear ValueError naming that joint, instead of the bare
#   KeyError a dict subscript produces today;
# - `ArmIK.__init__` validates that the profile's `target_frame` exists in the
#   URDF (mirroring the T-IK-10 joint check) instead of deferring the failure
#   to the first fk()/solve();
# - `profile.urdf_path` is expanduser()'d consistently so a `~`-based path the
#   constructor accepts is also usable by fk()/solve().
#
# Current behaviour probed before writing this binding (RED evidence):
# - np.eye(3) target -> pybind11 ArgumentError; (4,)/(16,) -> RuntimeError;
#   none is the required ValueError.
# - missing joint key -> KeyError: "'left_wrist_roll'" (not a ValueError).
# - missing target_frame -> construction SUCCEEDS; fk() later raises RuntimeError.
# - "~/arm.urdf" -> construction OK (already expanduser'd), but fk()/solve() use
#   the raw `self.profile.urdf_path` and fail with "does not contain a valid
#   URDF model".
#
# ADJUDICATED seam (reported to the main agent): the F2 wording fixed
# "明确 ValueError" for target_pose but not the exact type for the missing
# joint-limit key; this binding requires ValueError for both (project D017
# convention, same as the existing T-IK-10 joint/URDF checks). If the
# implementer prefers a different explicit type, the binding must be re-planned
# first.
#
# Discriminative intent (a reasonable but WRONG implementation must fail):
# - no target_pose shape check -> placo's ArgumentError/RuntimeError, not
#   ValueError;
# - no explicit joint-limit validation -> bare KeyError, not ValueError;
# - a constructor that stores the profile blindly and only touches the frame on
#   the first fk() -> the target_frame case fails;
# - fk()/solve() that use the un-expanded `self.profile.urdf_path` -> the tilde
#   case fails.


@pytest.mark.parametrize(
    "bad_shape",
    [np.eye(3), np.zeros(4), np.zeros(16), np.zeros((4, 4, 1)), np.zeros(0)],
    ids=["3x3", "flat4", "flat16", "3d", "empty"],
)
def test_t_ik_2_2_solve_rejects_non_4x4_target_pose_with_value_error(bad_shape: object) -> None:
    """T-IK-2.2：solve 对非 4x4 的 target_pose 抛 ValueError。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    bad = np.asarray(bad_shape, dtype=float)
    assert bad.shape != (4, 4), "fixture drift: bad_shape must not be a valid 4x4"

    with pytest.raises(ValueError) as excinfo:  # NOT placo's ArgumentError/RuntimeError
        arm.solve(seed, bad, limits_for())
    assert str(excinfo.value), "the shape error must carry a message"


def test_t_ik_2_3_solve_rejects_unknown_joint_limits_key_naming_the_joint() -> None:
    """T-IK-2.3：solve 缺少关节限位键时抛 ValueError 并点名关节。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    _arm_ik, arm = make_arm_ik()
    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    missing = "left_wrist_roll"
    incomplete = {name: (-90.0, 90.0) for name in JOINT_NAMES if name != missing}
    assert missing not in incomplete

    with pytest.raises(ValueError) as excinfo:
        arm.solve(seed, FROZEN_OVER_LIMIT_TARGET_POSE, incomplete)
    assert missing in str(excinfo.value), (
        f"the missing-joint-limit error must name {missing!r}, got: {str(excinfo.value)!r}"
    )


def test_t_ik_10_4_missing_target_frame_rejected_at_construction() -> None:
    """T-IK-10.4：目标 frame 不存在时在构造期报错并点名。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    arm_ik = load_arm_ik()
    missing_frame = "left_Fixed_Jaw_typo"
    profile = arm_ik.ArmKinematicProfile(
        urdf_path=URDF_PATH, target_frame=missing_frame, joint_names=JOINT_NAMES
    )

    with pytest.raises((ValueError, KeyError, RuntimeError)) as excinfo:
        arm_ik.ArmIK(profile)
    assert missing_frame in str(excinfo.value), (
        f"construction error must name the missing target frame, got: {str(excinfo.value)!r}"
    )


def test_t_ik_10_5_tilde_urdf_path_is_expanded_for_fk_and_solve(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """T-IK-10.5：波浪线开头的 URDF 路径在 fk/solve 中被展开。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    # `~` is resolved through the HOME env var by Path.expanduser(), so a temp
    # HOME keeps the case hermetic (nothing is written outside tmp_path).
    arm_ik = load_arm_ik()
    shutil.copyfile(URDF_PATH, tmp_path / "arm.urdf")
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("USERPROFILE", str(tmp_path))  # Windows parity, harmless here

    profile = arm_ik.ArmKinematicProfile(
        urdf_path="~/arm.urdf", target_frame=TARGET_FRAME, joint_names=JOINT_NAMES
    )
    # Construction already resolves `~`; the fix is that fk()/solve() must too.
    arm = arm_ik.ArmIK(profile)

    pose = np.asarray(arm.fk(np.zeros(6)), dtype=float)
    assert pose.shape == (4, 4)
    assert np.all(np.isfinite(pose))

    seed = np.asarray(SOLVE_SEED_Q, dtype=float)
    result = arm.solve(seed, FROZEN_OVER_LIMIT_TARGET_POSE, limits_for())
    assert result.ok is True, f"tilde URDF path must be usable by solve (reason={result.reason!r})"


# ===========================================================================
# T-IK-17  limit-face tolerance (closed-interval classification)
# ===========================================================================
#
# 缺陷（本轮修复前）：`ArmIK.solve` 末尾用闭区间 ``lo <= value <= hi`` 判越界
# （``control/arm_ik.py`` 约 :325-330），于是「目标恰好压在限位面上」时结论由浮点
# 最后一位决定：同一目标在 placo 0.10.1 下解出 90.00000000000001（超 1.42e-14 度）
# 判 JOINT_LIMIT，在 placo 0.9.15 下解出 89.99999999999997 判 ok。修复方向：只有
# 超出容差才算 JOINT_LIMIT，且返回的 q_deg 仍夹在 ``[lo, hi]`` 内。两项均已落地
# （``control/arm_ik.py`` 的 ``LIMIT_EPS_DEG`` 与 ``_clamp_to_joint_limits``）。
#
# 断言的层次（本段两类用例的分工）：
# - 端到端用例（T-IK-17.1 / 17.11 / 17.12）只断 ``ArmIK.solve()`` 的可观察行为
#   （ok / reason / 返回的 q_deg），不引用实现内部常量名；
# - 纯逻辑用例（T-IK-17.2 - 17.10）直接测实现接缝：``LIMIT_EPS_DEG`` 的取值量级
#   与 ``_clamp_to_joint_limits`` 的判定/夹紧语义。它们是有意的白盒断言 —— 隔离度
#   更高，一条红点能直接指到「容差量级错」还是「夹紧/接线错」，不必再从数值求解
#   的浮点噪声里去分辨。
#
# 构造方式：取一个边界姿态 q*，把被测关节 j 的注入限位面精确设在 q*[j] 上（下限面
# 或上限面），目标位姿取 ``fk(q*)`` —— 目标与该限位面精确重合，残差 ~1e-17 m，
# 物理上完全可达。此时任何 JOINT_LIMIT 结论都只能来自「边界上的浮点最后一位」，
# 不可能来自残差或真实越界，这正是本段要钉住的那一类误判。
#
# 修复前实测（本机 placo 0.10.1，3/3 次重复一致，无随机、无时钟）：
# - 上限面 ``left_wrist_roll`` = 90 -> 解出 ``90.00000000000001``（+1.421e-14 度）
# - 下限面 ``left_wrist_yaw_joint`` = 90 -> 解出 ``89.99999999999996``（-4.263e-14 度）
# 两例都被当前代码判成 ``JOINT_LIMIT``（本段的红证据）。
#
# 断言的语义（修复后必须成立）：
# 1. ``ok is True`` / ``reason is None`` —— 压在限位面上不算越界；
# 2. ``position_error_m <= residual_limit_m`` —— 目标物理可达（残差判据不变）；
# 3. 返回的 ``q_deg`` 每个分量都落在 ``[lo, hi]`` 闭区间内 —— 返回值被夹紧，
#    不得因为容差放宽而带着 1e-14 量级的外溢返回给调用方。
#
# 鉴别力（合理但错误的实现会挂）：
# - 保留闭区间严格比较 -> 第 1 条先红（ok=False / reason=JOINT_LIMIT）；
# - 放宽判定但忘记夹紧 -> 第 3 条红（q 仍外溢 1.4e-14 度）；
# - 为通过而删掉越界判定 / 放宽残差 -> T-IK-4.1 仍要求「物理可达但超注入限位」
#   的目标报 JOINT_LIMIT，两条要求不能同时被削弱。
#
# 说明（外部 oracle 取舍）：目标位姿由被测 ``fk()`` 从其自身边界姿态取得，不是冻结
# 外部字面量。本段只检验限位分类，不检验 FK 数值；FK 的数值正确性已由 T-IK-1 用
# 外部冻结位姿独立钉住，且 FK 若漂移会让本段的宽度前提 / 残差断言失败（不会静默
# 通过），因此不引入「拿 SUT 输出当 oracle」的静默风险。

# (face, joint, value)：face="upper" 表示 value 是该关节的 *上* 限位，"lower" 为下限位。
# 两个关节的漂移方向不同（实测：wrist_roll 偏正、wrist_yaw_joint 偏负），所以上下
# 两个限位面各用一条实测能复现漂移的组合；漂移方向若在 placo 升级后翻转，这些用例
# 只会失去鉴别力，不会产生假失败。
LIMIT_FACE_CASES = (
    ("upper", "left_wrist_roll", 90.0),
    ("lower", "left_wrist_yaw_joint", 90.0),
)


def make_limit_face_case(
    face: str, joint: str, value: float
) -> tuple[Any, np.ndarray, np.ndarray, dict[str, tuple[float, float]]]:
    """构造「目标恰好压在限位面」的输入：(arm, seed, target_pose, limits)。"""
    _arm_ik, arm = make_arm_ik()
    index = JOINT_NAMES.index(joint)

    q_boundary_deg = list(SOLVE_SEED_Q)
    q_boundary_deg[index] = value
    seed = np.asarray(q_boundary_deg, dtype=float)
    target_pose = np.asarray(arm.fk(seed), dtype=float)

    limits = limits_for(lo=-170.0, hi=170.0)
    limits[joint] = (value, 170.0) if face == "lower" else (-170.0, value)
    return arm, seed, target_pose, limits


@pytest.mark.parametrize(("face", "joint", "value"), LIMIT_FACE_CASES)
def test_t_ik_17_1_target_on_limit_face_is_not_a_joint_limit_failure(
    face: str, joint: str, value: float
) -> None:
    """T-IK-17.1：目标恰好压在限位面上时判 ok，且返回的 q 被夹在闭区间内。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    arm, seed, target_pose, limits = make_limit_face_case(face, joint, value)
    index = JOINT_NAMES.index(joint)

    # 前提：该关节的注入限位面确实与边界姿态的关节值精确重合（夹具漂移检查）。
    lo_face, hi_face = limits[joint]
    assert value == lo_face or value == hi_face, (value, (lo_face, hi_face))

    # 前提：同一目标在宽限位下可达 —— 下面的失败只可能来自限位面分类。
    wide = arm.solve(seed, target_pose, limits_for(lo=-170.0, hi=170.0))
    assert wide.ok is True, f"test premise broken: target unreachable ({wide.reason!r})"

    result = arm.solve(seed, target_pose, limits)

    q_raw_index = float(np.asarray(result.q_deg, dtype=float)[index])
    assert result.ok is True, (
        f"target exactly on the {face} limit face of {joint} (= {value!r} deg) must not be "
        f"rejected: ok={result.ok!r} reason={result.reason!r} "
        f"q[{joint}]={q_raw_index!r} (off the face by {q_raw_index - value!r} deg) "
        f"position_error_m={result.position_error_m!r}"
    )
    assert result.reason is None, result.reason
    assert result.position_error_m <= SOLVE_RESIDUAL_LIMIT_M, result.position_error_m

    q_deg = np.asarray(result.q_deg, dtype=float)
    assert q_deg.shape == (6,)
    for name, q_value in zip(JOINT_NAMES, q_deg, strict=True):
        lo, hi = limits[name]
        assert lo <= q_value <= hi, (
            f"returned {name}={q_value!r} escaped the closed limits ({lo}, {hi})"
        )

    # 解仍停在限位面上（既没外溢也没被推离）。断言容差 1e-6 度只是「还贴在面上」的粗
    # 边界，**故意与 ``LIMIT_EPS_DEG`` 解耦**（同值会让人误以为是实现容差的重述）：1e-6
    # 度 = 1.7e-8 rad，0.3 m 力臂上约 5 nm，物理上与「在面上」不可区分，却足以让「把解
    # 主动推离限位面」的实现（例如 clamp 后又向区间内部让一个小量）变红。
    assert abs(float(q_deg[index]) - value) <= 1e-6, (float(q_deg[index]), value)


# ---------------------------------------------------------------------------
# T-IK-17.2 - T-IK-17.8  纯逻辑：限位容差与夹紧语义（不依赖 placo）
# ---------------------------------------------------------------------------
#
# T-IK-17.1 端到端地钉住「目标压在限位面上判 ok」，但它必须借 placo 做数值求解，
# 隔离度低（一个红点无法区分「容差判定错」还是「求解器数值漂移」）。本轮修复把判定
# 与夹紧抽成了纯逻辑接缝 ``_clamp_to_joint_limits`` + ``LIMIT_EPS_DEG``，于是本段
# 直接用合成关节值测这两条语义，**不 import placo、不构造 ArmIK**：
#
# - 容差语义：``[lo - LIMIT_EPS_DEG, hi + LIMIT_EPS_DEG]`` 为内（两端都是闭的），
#   超出容差才算越界；边界点本身算内。
# - 夹紧语义：返回向量逐分量夹回 ``[lo, hi]``（容差只放宽「判内」，不放宽「返回值」），
#   且不就地修改入参。
#
# 为什么必须钉住「容差只放宽判定、不放宽返回值」：T-IK-17.1 只断言返回 q 落在闭区间，
# 若实现把容差直接写进返回向量（q 允许外溢 1e-9 度），端到端用例也会通过；本段的
# ``q_clamped[probe] == face`` 逐位断言让这种实现变红。
#
# 合成数据：六个关节共用同一条对称限位 (-170, 170) 度，基准姿态全部落在内部且不与任何
# 限位面重合；每个用例只改探针关节（``left_wrist_roll``）一个分量，其余分量恒定。这样
# 一条断言失败只可能来自探针分量的分类 / 夹紧逻辑。


def load_limit_clamp() -> tuple[float, Any]:
    """惰性导入纯逻辑接缝：``LIMIT_EPS_DEG`` 与 ``_clamp_to_joint_limits``。

    与本文件其它用例一致，导入放在 helper 里而不是模块顶层：实现缺失时只会让本段的
    用例各自 ImportError，不会因为收集期报错而拖垮整个模块（也不触发任何 placo 导入）。
    """
    from alohamini_cartesian.control.arm_ik import LIMIT_EPS_DEG, _clamp_to_joint_limits

    return LIMIT_EPS_DEG, _clamp_to_joint_limits


CLAMP_LIMITS_DEG: dict[str, tuple[float, float]] = dict.fromkeys(JOINT_NAMES, (-170.0, 170.0))
CLAMP_BASELINE_Q = np.asarray([-10.0, 20.0, 30.0, -40.0, 50.0, 60.0], dtype=float)
CLAMP_PROBE_INDEX = JOINT_NAMES.index("left_wrist_roll")
CLAMP_LO_DEG, CLAMP_HI_DEG = CLAMP_LIMITS_DEG["left_wrist_roll"]
# (face, 面值)：upper/lower 两个面各一组对称用例，所有容差用例都跑这两个方向。
CLAMP_FACES = (("upper", CLAMP_HI_DEG), ("lower", CLAMP_LO_DEG))
# 容差内部的偏移（用 LIMIT_EPS_DEG 表达，不写裸魔数）。
# ``one_ulp_above_face`` 是「面值旁边最小可表示的浮点数」：本段面值是 ±170.0，
# ``math.ulp(170.0) == 2 ** -45 ≈ 2.842e-14``，正好是实测求解器舍入的量级。
# 这里**不能**用字面量 1e-14：ulp(170.0) 的一半是 1.42e-14，1e-14 小于它，
# ``170.0 + 1e-14`` 舍入后逐位等于 170.0，参数退化成「恰好压面」（= T-IK-17.3），
# 任何容差下都恒绿、零鉴别力。
TOLERANCE_INSIDE_OFFSETS = ("half_eps", "one_ulp_above_face")
# 容差外的偏移：刚好两倍容差（钉住"容差外的偏离才算越界"这一侧界）+ 一个明显越界值。
TOLERANCE_OUTSIDE_OFFSETS = ("two_eps", "one_e_minus_3")


def face_sign(face: str) -> float:
    """返回「越过该面」的数值方向：上限面为 +1，下限面为 -1。"""
    return 1.0 if face == "upper" else -1.0


def clamp_offset_deg(kind: str) -> float:
    """把偏移量名字翻译成度数（正数，方向由调用方按上下限决定）。"""
    eps, _clamp = load_limit_clamp()
    offsets = {
        "half_eps": eps / 2.0,
        "one_ulp_above_face": 2.0**-45,
        "two_eps": 2.0 * eps,
        "one_e_minus_3": 1e-3,
    }
    return offsets[kind]


def q_with_probe(value_deg: float) -> np.ndarray:
    """返回基准姿态的副本，只把探针关节改成 ``value_deg``。"""
    q_deg = CLAMP_BASELINE_Q.copy()
    q_deg[CLAMP_PROBE_INDEX] = value_deg
    return q_deg


def test_t_ik_17_2_interior_joint_values_pass_through_unchanged() -> None:
    """T-IK-17.2：六关节全在闭区间内部时判内，且返回值与入参逐分量相等。"""
    # 鉴别力：若容差判定写反（把内部当越界）、或用严开区间 ``lo < v < hi`` 且实现把
    # 边界值当内部、或对内部值做了任何再量化/取整，都会让下面两条断言之一先红。
    _eps, clamp = load_limit_clamp()
    q_deg = CLAMP_BASELINE_Q.copy()

    within_limits, q_clamped = clamp(q_deg, dict(CLAMP_LIMITS_DEG), JOINT_NAMES)

    assert within_limits is True
    assert q_clamped.shape == (6,)
    assert np.array_equal(q_clamped, q_deg), (q_clamped, q_deg)


@pytest.mark.parametrize(("face", "face_deg"), CLAMP_FACES)
def test_t_ik_17_3_value_exactly_on_limit_face_is_inside(face: str, face_deg: float) -> None:
    """T-IK-17.3：关节值恰好等于上限或下限时判内，且返回值不被推离该面。"""
    # 鉴别力：若容差被设成 0 且判定写成严开区间（``lo < v < hi``），压在面上会判
    # 越界 → 第 1 条红；若实现为了"保险"把面值向区间内部推动（例如 clamp 后再减一个
    # 小量），第 2 条的逐位相等会红。
    _eps, clamp = load_limit_clamp()
    q_deg = q_with_probe(face_deg)

    within_limits, q_clamped = clamp(q_deg, dict(CLAMP_LIMITS_DEG), JOINT_NAMES)

    assert within_limits is True, f"{face} face value {face_deg!r} must count as inside"
    assert q_clamped[CLAMP_PROBE_INDEX] == face_deg, (float(q_clamped[CLAMP_PROBE_INDEX]), face_deg)


@pytest.mark.parametrize(("face", "face_deg"), CLAMP_FACES)
@pytest.mark.parametrize("offset_kind", TOLERANCE_INSIDE_OFFSETS)
def test_t_ik_17_4_overflow_within_tolerance_is_inside_and_clamped_onto_face(
    face: str, face_deg: float, offset_kind: str
) -> None:
    """T-IK-17.4：越面幅度在 ``LIMIT_EPS_DEG`` 以内时判内，但返回值被夹回恰好该面。"""
    # 鉴别力（互斥的实现错法）：
    # - 容差被设成 0（判定退化成闭区间严格比较）→ ``one_ulp_above_face`` 这条参数的
    #   偏移（1 ulp ≈ 2.84e-14 度，在 170 量级上可表示）落到面外，within_limits 变
    #   False，第 1 条红；``half_eps`` 这条在 eps=0 时偏移归 0、退化成「恰好压面」
    #   （等价 T-IK-17.3），它抓不到 eps=0。抓 eps=0 / 极小 eps 的是 T-IK-17.5（两倍
    #   容差偏移同时归零，面值被要求判越界）与 T-IK-17.9（直接钉容差下界）。
    # - 判内但忘记夹紧（直接返回原值）→ 返回值比面值大 eps/2 或 1 ulp，第 2 条红；
    # - 容差被放大到 1e-4 / 1e-3 量级 → 本用例仍绿，由 T-IK-17.10（直接钉容差上界）
    #   与 T-IK-17.5 的 one_e_minus_3 参数抓。
    _eps, clamp = load_limit_clamp()
    value_deg = face_deg + face_sign(face) * clamp_offset_deg(offset_kind)
    q_deg = q_with_probe(value_deg)

    # 前提：该偏移在 ±170 度的量级上真的可表示（不为 0）。否则本参数退化成「恰好
    # 压面」、与 T-IK-17.3 重复，失去全部鉴别力 —— 这条断言就是为守 1 ulp 那类偏移。
    assert value_deg != face_deg, (value_deg, face_deg, offset_kind)

    within_limits, q_clamped = clamp(q_deg, dict(CLAMP_LIMITS_DEG), JOINT_NAMES)

    assert within_limits is True, (
        f"{value_deg!r} deg is {value_deg - face_deg!r} beyond the {face} face, "
        "which is inside the epsilon tolerance and must not count as a violation"
    )
    assert q_clamped[CLAMP_PROBE_INDEX] == face_deg, (
        f"returned {float(q_clamped[CLAMP_PROBE_INDEX])!r} must be clamped exactly onto the "
        f"{face} face {face_deg!r}, not left {float(q_clamped[CLAMP_PROBE_INDEX]) - face_deg!r} "
        "outside it"
    )
    lo, hi = CLAMP_LIMITS_DEG[JOINT_NAMES[CLAMP_PROBE_INDEX]]
    assert lo <= float(q_clamped[CLAMP_PROBE_INDEX]) <= hi


@pytest.mark.parametrize(("face", "face_deg"), CLAMP_FACES)
@pytest.mark.parametrize("offset_kind", TOLERANCE_OUTSIDE_OFFSETS)
def test_t_ik_17_5_overflow_beyond_tolerance_is_outside_and_clamped_onto_face(
    face: str, face_deg: float, offset_kind: str
) -> None:
    """T-IK-17.5：越面幅度超过 ``LIMIT_EPS_DEG`` 时判越界，返回值仍夹在该面上。"""
    # 鉴别力：
    # - 容差被放大（例如 1e-3 或更大）→ 这些点会变成判内，第 1 条红；
    # - 容差方向写反（只在区间内侧放宽，即 ``lo <= v <= hi`` 配 ``lo-eps`` 误用）→
    #   上下两个面的符号覆盖会让其中一半变红；
    # - 判越界但夹紧被删 → 第 2、3 条红（返回值仍在外侧）。
    _eps, clamp = load_limit_clamp()
    value_deg = face_deg + face_sign(face) * clamp_offset_deg(offset_kind)
    q_deg = q_with_probe(value_deg)

    within_limits, q_clamped = clamp(q_deg, dict(CLAMP_LIMITS_DEG), JOINT_NAMES)

    assert within_limits is False, (
        f"{value_deg!r} deg is {value_deg - face_deg!r} beyond the {face} face, "
        "which exceeds the epsilon tolerance and must count as a violation"
    )
    assert q_clamped[CLAMP_PROBE_INDEX] == face_deg, (float(q_clamped[CLAMP_PROBE_INDEX]), face_deg)
    for name, q_value in zip(JOINT_NAMES, q_clamped, strict=True):
        lo, hi = CLAMP_LIMITS_DEG[name]
        assert lo <= float(q_value) <= hi, f"clamped {name}={float(q_value)!r} escaped ({lo}, {hi})"


@pytest.mark.parametrize(
    ("face", "face_deg", "offsets_outward", "expect_clamped"),
    (
        ("upper", CLAMP_HI_DEG, True, True),  # hi + eps：恰在边界外侧，算内
        ("upper", CLAMP_HI_DEG, False, False),  # hi - eps：显然在内，不该被夹
        ("lower", CLAMP_LO_DEG, True, True),  # lo - eps：恰在边界外侧，算内
        ("lower", CLAMP_LO_DEG, False, False),  # lo + eps：显然在内，不该被夹
    ),
)
def test_t_ik_17_6_tolerance_boundary_is_inclusive(
    face: str, face_deg: float, offsets_outward: bool, expect_clamped: bool
) -> None:
    """T-IK-17.6：恰好距面 ``LIMIT_EPS_DEG`` 的点算**内**（容差区间两端都是闭的）。"""
    # 鉴别力：这条钉的是"包含还是排除"的侧界。若实现写成严开区间
    # （``lo - eps < v < hi + eps``），外侧的两条（offsets_outward=True）会因
    # within_limits 变 False 而红；若容差只加在不该加的一侧，上下不对称会让其中一条红。
    # 第二条断言同时防止"容差放宽后不再夹紧"：边缘外侧点仍必须被夹回面值。
    eps, clamp = load_limit_clamp()
    sign = face_sign(face)
    value_deg = face_deg + sign * eps if offsets_outward else face_deg - sign * eps
    q_deg = q_with_probe(value_deg)

    within_limits, q_clamped = clamp(q_deg, dict(CLAMP_LIMITS_DEG), JOINT_NAMES)

    assert within_limits is True, (
        f"{value_deg!r} deg sits exactly LIMIT_EPS_DEG ({eps!r}) from the {face} face "
        "and must count as inside (closed interval on both sides)"
    )
    expected_deg = face_deg if expect_clamped else value_deg
    assert q_clamped[CLAMP_PROBE_INDEX] == expected_deg, (
        float(q_clamped[CLAMP_PROBE_INDEX]),
        expected_deg,
    )


def test_t_ik_17_7_input_vector_is_not_modified_in_place() -> None:
    """T-IK-17.7：调用后入参数组内容逐位不变，返回的是新数组。"""
    # 鉴别力：若实现直接在入参上夹紧（``values[:] = np.clip(...)`` 后返回 ``values``），
    # 第 2 条红 —— 调用方的 q_deg 会被悄悄改写，这在 solve() 的调用链里等于篡改上次解。
    # 用超过容差的点驱动，保证夹紧分支一定被执行到（否则本用例没有实际覆盖夹紧路径）。
    eps, clamp = load_limit_clamp()
    q_deg = q_with_probe(CLAMP_HI_DEG + 2.0 * eps)
    before = q_deg.copy()

    within_limits, q_clamped = clamp(q_deg, dict(CLAMP_LIMITS_DEG), JOINT_NAMES)

    assert within_limits is False, "premise: this probe value must take the clamping branch"
    assert np.array_equal(q_deg, before), (q_deg, before)
    assert q_clamped is not q_deg, "clamp must return a new array, not the caller's array"
    assert q_clamped[CLAMP_PROBE_INDEX] == CLAMP_HI_DEG


def test_t_ik_17_8_pure_logic_clamp_is_usable_with_placo_absent() -> None:
    """T-IK-17.8：placo 不可见时纯逻辑接缝仍可导入并夹紧（本段不依赖 placo 的证据）。"""
    # 用本文件已有的 placo_absent() 在进程内屏蔽 placo 并强制重新导入被测模块：若
    # ``_clamp_to_joint_limits`` 或它的模块顶层链条里出现任何 `import placo`，本用例
    # 会 ImportError；若它偷偷走求解器，同样会因为 placo 缺失而失败。
    with placo_absent():
        eps, clamp = load_limit_clamp()
        q_deg = q_with_probe(CLAMP_HI_DEG + 2.0 * eps)

        within_limits, q_clamped = clamp(q_deg, dict(CLAMP_LIMITS_DEG), JOINT_NAMES)

    assert within_limits is False
    assert q_clamped[CLAMP_PROBE_INDEX] == CLAMP_HI_DEG


# ---------------------------------------------------------------------------
# T-IK-17.9 - T-IK-17.10  容差量级的允许区间（直接读常量，不乘任何系数）
# ---------------------------------------------------------------------------
#
# 为什么必须直接钉量级：本段其余用例都用 ``LIMIT_EPS_DEG`` 的**相对**倍数（``eps/2``、
# ``2*eps``）或一个远大于任何合理容差的绝对值（1e-3）表达偏移。前者对任何 eps > 0 都
# 自洽（实测把容差改成 1e-6 甚至 1e-4，整个文件仍全绿），后者只能抓 >= 1e-3 的放大。
# 于是补两条直接断言常量落区的用例：下界盖住求解器实测舍入，上界远小于任何物理上可
# 分辨的关节偏差。
#
# 允许区间 ``[1e-12, 1e-6]`` 度的依据（以后要改容差的人以此为准）：
# - 下界 1e-12：本机 placo 0.10.1 实测最大外溢 4.263e-14 度（见段首），1e-12 约为它的
#   23 倍；再小就有在限位面上误判越界的风险 —— 那正是本段要修的缺陷；
# - 上界 1e-6：关节编码器分辨率 / 机械回差在 1e-2 度量级，1e-6 度比它小四个数量级，
#   放到这里仍不可能把「真实越界」判成「在容差内」。
#
# 有意把容差调到区间外时，请连同这两条用例与本段注释一起重新论证，而不是直接放宽断言。
LIMIT_EPS_MIN_DEG = 1e-12
LIMIT_EPS_MAX_DEG = 1e-6


def test_t_ik_17_9_limit_tolerance_absorbs_solver_round_off() -> None:
    """T-IK-17.9：``LIMIT_EPS_DEG`` 足够大，能盖住实测求解器舍入（量级 1e-14 度）。"""
    # 鉴别力：容差被设成 0 / 1e-15 / 1e-14 一类量级时本用例红。这些值会让「2*eps 偏移」
    # 类用例跟着缩小偏移而继续绿，所以必须有一条直接读常量的断言把它们抓住。
    eps, _clamp = load_limit_clamp()

    assert eps >= LIMIT_EPS_MIN_DEG, (
        f"LIMIT_EPS_DEG={eps!r} deg is too tight: solver round-off on a limit face measures up to "
        f"4.263e-14 deg (placo 0.10.1), so the tolerance must stay >= {LIMIT_EPS_MIN_DEG!r}"
    )


def test_t_ik_17_10_limit_tolerance_stays_far_below_physical_deviations() -> None:
    """T-IK-17.10：``LIMIT_EPS_DEG`` 足够小，放大到它仍不可能掩盖真实越界。"""
    # 鉴别力：容差被放大到 1e-4 / 1e-3 时本用例红。其中 1e-4 是 T-IK-17.5 抓不到的区间
    # （它的越界偏移取 2*eps 与 1e-3），只有直接钉上界能覆盖。
    eps, _clamp = load_limit_clamp()

    assert eps <= LIMIT_EPS_MAX_DEG, (
        f"LIMIT_EPS_DEG={eps!r} deg is too loose: a real limit violation of 1e-2 deg "
        f"(encoder / backlash scale) must stay detectable, so the tolerance must stay "
        f"<= {LIMIT_EPS_MAX_DEG!r}"
    )


# ---------------------------------------------------------------------------
# T-IK-17.11 - T-IK-17.12  端到端接线：``within_limits`` 必须真的进 ``ok``
# ---------------------------------------------------------------------------
#
# 为什么需要这两条：真实的 qpmad 求解器在注入限位下几乎不外溢（实测 1e-15 ~ 1e-14
# 度），所以本段其余端到端用例里 ``JOINT_LIMIT`` 一律由残差分支产生 —— 把 ``solve()``
# 里的 ``ok = ... and within_limits`` 改成忽略 ``within_limits``，它们仍全绿。限位分类
# 与最终 ``ok`` 的接线因此没有任何覆盖，这两条用假求解器把它单独钉住。
#
# 做法：``monkeypatch`` 掉 ``ArmIK._solve_once``。``solve()`` 会调用它两次 —— 先用 URDF
# 限位判物理可达性（残差超限就走 RESIDUAL 分支），再用调用方注入的限位做分类。假实现
# 按「注入字典的对象身份」区分这两次，并让两次的残差都达标：于是任何失败结论都只可能
# 来自限位分类，不可能来自 RESIDUAL。这也是唯一能确定性制造「残差达标 + 真实越界」的
# 方式（真实求解器造不出来）。
#
# 前提断言 ``calls == {"urdf": 1, "injected": 1}`` 是接线前提：如果 solve() 以后改掉
# 「两次求解」的结构，或不再把调用方的字典原样透传，本段的失败会明确报「前提破了」，
# 而不是伪装成分类错误。


def solve_with_faked_solver(
    monkeypatch: pytest.MonkeyPatch,
    injected_limits: dict[str, tuple[float, float]],
    q_injected: np.ndarray,
) -> tuple[Any, dict[str, int]]:
    """用假 ``_solve_once`` 驱动一次 ``solve()``，返回 (结果, 两次调用的计数)。

    ``q_injected`` 是「注入限位那次」求解要返回的关节解；URDF 限位那次固定返回干净的
    种子解（残差同样达标），保证流程不会停在 RESIDUAL 分支。
    """
    arm_ik, arm = make_arm_ik()
    q_urdf = np.asarray(SOLVE_SEED_Q, dtype=float)
    calls = {"urdf": 0, "injected": 0}

    def fake_solve_once(
        self: Any,
        q_current_deg: np.ndarray,
        target_pose: np.ndarray,
        limits_deg: dict[str, tuple[float, float]],
        orientation_weight: float,
    ) -> tuple[np.ndarray, float, float, int, float, np.ndarray]:
        """返回残差达标的解；按注入字典的对象身份区分 URDF 那次与注入那次。"""
        del q_current_deg, target_pose, orientation_weight  # 本次只测限位分类接线
        if limits_deg is injected_limits:
            calls["injected"] += 1
            q_deg = q_injected
        else:
            calls["urdf"] += 1
            q_deg = q_urdf
        # (q_deg, position_error_m, orientation_error_deg, iterations, solve_time_s, pose)
        return np.asarray(q_deg, dtype=float).copy(), 1e-6, 0.0, 1, 1e-4, np.eye(4)

    monkeypatch.setattr(arm_ik.ArmIK, "_solve_once", fake_solve_once)
    result = arm.solve(q_urdf, np.eye(4), injected_limits)
    return result, calls


def test_t_ik_17_11_solve_reports_joint_limit_when_solution_leaves_the_limits(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T-IK-17.11：残差达标但某关节真实越限 -> ok=False / JOINT_LIMIT，且被夹回限位面。"""
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_for(lo=-170.0, hi=170.0)
    joint = "left_wrist_roll"
    index = JOINT_NAMES.index(joint)
    _lo_deg, hi_deg = limits[joint]
    # 越面幅度 1e-3 度：大于容差允许上界 1e-6 三个数量级，是任何合理实现都必须判越界的
    # 真实越限（不是浮点最后一位）。
    q_over = np.asarray(SOLVE_SEED_Q, dtype=float)
    q_over[index] = hi_deg + 1e-3

    result, calls = solve_with_faked_solver(monkeypatch, limits, q_over)

    # 前提：两次求解都真的走了假实现，且注入限位那次被识别出来了。
    assert calls == {"urdf": 1, "injected": 1}, calls
    # 前提：残差本来就达标 —— 下面的失败结论只可能来自限位分类，不是 RESIDUAL。
    assert result.position_error_m <= SOLVE_RESIDUAL_LIMIT_M, result.position_error_m

    q_deg = np.asarray(result.q_deg, dtype=float)
    assert result.ok is False, (
        "a solution outside the injected limits must not report ok=True: "
        f"q[{joint}]={float(q_deg[index])!r} vs limit hi={hi_deg!r}, reason={result.reason!r}"
    )
    assert result.reason == "JOINT_LIMIT", (
        f"over-limit solution with acceptable residual must be JOINT_LIMIT, got {result.reason!r}"
    )
    # 夹紧：返回的该关节逐位等于限位面值（容差只放宽「判内」，不放宽「返回值」）。
    assert float(q_deg[index]) == hi_deg, (float(q_deg[index]), hi_deg)
    lo_face, hi_face = limits[joint]
    assert lo_face <= float(q_deg[index]) <= hi_face, (float(q_deg[index]), (lo_face, hi_face))


def test_t_ik_17_12_solve_keeps_ok_when_solution_sits_on_a_limit_face(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T-IK-17.12：残差达标且解恰好停在限位面上 -> ok=True / reason=None（接线不恒 False）。"""
    # 与 17.11 对称：同一条接线在「容差内」必须给出 ok=True。若实现把 ``ok`` 写成恒
    # False、或把容差判定写反（内部当越界），本用例红；两条合起来说明 ``ok`` 真正
    # 依赖限位分类的结果，而不是被某个常量钉死。
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_for(lo=-170.0, hi=170.0)
    joint = "left_wrist_roll"
    index = JOINT_NAMES.index(joint)
    _lo_deg, hi_deg = limits[joint]
    q_on_face = np.asarray(SOLVE_SEED_Q, dtype=float)
    q_on_face[index] = hi_deg

    result, calls = solve_with_faked_solver(monkeypatch, limits, q_on_face)

    assert calls == {"urdf": 1, "injected": 1}, calls
    q_deg = np.asarray(result.q_deg, dtype=float)
    assert result.ok is True, (
        f"a solution sitting exactly on the limit face must keep ok=True: "
        f"q[{joint}]={float(q_deg[index])!r}, reason={result.reason!r}"
    )
    assert result.reason is None, result.reason
    assert float(q_deg[index]) == hi_deg, (float(q_deg[index]), hi_deg)


# ===========================================================================
# T-IK-18  joint-limit VALUE validation at the ``solve()`` entry
# ===========================================================================
#
# 为什么需要这一段：``solve()`` 目前只检查「每个必需关节都有一个键」（T-IK-2.3），对限位
# 的**取值**不做任何校验，直接 ``np.deg2rad`` 后交给 placo。于是调用方把限位写错（倒置、
# 退化、非有限、类型不对、bool 当数字）时，失败要么来自求解器内部（不可包装的
# ``RuntimeError('QPError: ...')``），要么被伪装成一次正常的求解失败（``ok=False`` /
# ``reason='JOINT_LIMIT'``），要么根本不报错。本段要求入口一律给出 ``ValueError``，并在
# 消息里点名出问题的关节 —— 与 D017「冻结数据 + 显式 ValueError，绝不静默」一致。
#
# 契约（本段钉住的语义）：
# - 每个必需关节的限位必须是「长度 2 的实数序列」，且 ``lo < hi``；
# - ``NaN`` / ``inf`` 不是合法取值（``lo``、``hi`` 任一都必须有限）；
# - ``bool`` 不算数字，``True`` / ``False`` 一律拒绝；
# - 违规一律 ``ValueError``，消息含该关节名与该侧取值；
# - 多余的键（不属于 ``profile.joint_names``）被忽略，不改变求解结果。
#
# Current behaviour probed before writing this binding (RED evidence，v0.4.0)，
# 种子 ``q = (10, -20, 30, -40, 20, -10)``、目标取 ``SOLVE_REACHABLE_CASES['elbow_bend']``
# 的冻结位姿、六个关节合法限位 ``(-90, 90)``，只改一个关节：
# - ``(90.0, -90.0)``        -> RuntimeError: QPError: Problem: Infeasible QP ...（未包装）
# - ``(0.0, 0.0)``           -> 不抛，返回 ok=False / reason='JOINT_LIMIT'
# - ``(nan, 90.0)``          -> RuntimeError: QPError: Problem: NaN in the QP solution
# - ``(-90.0, inf)``         -> 不抛，返回 ok=True / reason=None（inf 被静默接受）
# - ``(90.0,)``              -> ValueError: not enough values to unpack (expected 2, got 1)
#                              （是 ValueError，但消息不含关节名）
# - ``(0.0, 1.0, 2.0)``      -> ValueError: too many values to unpack (expected 2)
#                              （同样不含关节名）
# - ``("a", "b")``           -> TypeError: ufunc 'deg2rad' not supported ...
# - ``(True, False)``        -> RuntimeError: QPError: Problem: Infeasible QP ...
# - 多余键 ``left_extra_joint`` -> 静默忽略（现状已符合契约，T-IK-18.4 红阶段即绿）
#
# 追加取证（在当前工作区实现上实跑，不是 v0.4.0）：
# - ``b"ab"`` / ``bytearray(b"ab")`` -> ValueError: ... must be a (lo, hi) pair, got b'ab'
#   （工作区已含文本类型排除；把 ``isinstance(limit, (str, bytes, bytearray))`` 拿掉后
#   变回「静默当成 ``(97, 98)`` 继续求解」，见 T-IK-18.8 的判别意图）
# - ``(10**400, 10**401)``   -> 加 ``OverflowError`` 兜底**之前**的中间态是
#   ``OverflowError: int too large to convert to float``（从 ``math.isfinite`` 裸逃出，
#   不是 ValueError）。**当前实现已不再抛 ``OverflowError``**：溢出被捕获后落回「有限性」
#   这一支，实测抛 ``ValueError: joint_limits_deg['left_elbow_flex'] must be finite, got
#   lo=<401 位十进制>, hi=<402 位十进制>``（``arm_ik.py`` 约 383 行），由 T-IK-18.9 钉住。
#
# Discriminative intent（一个「看起来合理但错」的实现必须失败）：
# - 只查键存在、不查取值 -> 18.1 / 18.2 直接失败；
# - 用 ``float(x)`` 做转换后只比 ``lo < hi`` -> false / "" 之类会被 float() 拒绝但
#   ``True`` / ``False`` 会被接受成 1.0 / 0.0，18.6 失败；
# - 只查 ``np.isfinite`` 不查类型 -> 18.5 的类型实例失败；
# - 校验时按字典顺序在必需关节之前报错 -> 18.4 失败；
# - 把校验从 ``profile.joint_names`` 蔓延到整个 ``joint_limits_deg`` 字典（多余键也校验）
#   -> 18.4 仍绿（它给多余键填的是合法值），必须靠 18.7 才杀得动；
# - 错误消息只写「哪一类错」而不带收到的取值 -> 18.3 / 18.5 / 18.6 失败；
# - 把校验放在 placo 调用之后 -> 18.1 / 18.3 仍然抛 RuntimeError，失败。
#
# 断言纪律：18.3 必须钉「有限性」这一支（消息含 ``finite``），不能只断言关节名与取值——
# 删掉 ``math.isfinite`` 后 ``NaN`` 会被后面的 ``not lo < hi`` 拦住并照样抛 ValueError，
# 且消息里照样有 ``nan``；只有分支专属字样能区分。本段不含恒真断言。

# 被破坏限位的关节。取 ``left_elbow_flex``：目标位姿要求它到 40 deg（seed 为 30 deg），
# 因此它同时落在「解必须动」与「动到限位之外」两边，倒置/退化/超窄都真的会与求解冲突。
LIMIT_VALIDATION_JOINT = "left_elbow_flex"
# 复用 T-IK-2.1 已有的冻结外部参考位姿（elbow_bend 的目标，配 SOLVE_SEED_Q 物理可达），
# 保证这些用例走到 solve() 的限位校验，而不是停在「不可达 -> RESIDUAL」这类更早的分支。
LIMIT_VALIDATION_TARGET_POSE = SOLVE_REACHABLE_CASES["elbow_bend"][3]


def limits_replacing(joint: str, limit_value: object) -> dict[str, object]:
    """六个必需关节都给合法 ``(-90, 90)``，再把 ``joint`` 的限位换成 ``limit_value``。"""
    limits: dict[str, object] = limits_for()
    limits[joint] = limit_value
    return limits


def solve_with_limits(limits: dict[str, object]) -> Any:
    """用 T-IK-2.1 的种子/目标位姿调一次 ``solve()``。"""
    _arm_ik, arm = make_arm_ik()
    return arm.solve(np.asarray(SOLVE_SEED_Q, dtype=float), LIMIT_VALIDATION_TARGET_POSE, limits)


def test_t_ik_18_1_inverted_joint_limit_rejected_naming_joint_and_both_values() -> None:
    """T-IK-18.1：倒置限位 ``(90, -90)`` 抛 ValueError，消息点名关节并给出两侧取值。

    不得是 placo 的裸 ``RuntimeError('QPError: Problem: Infeasible QP ...')``：未包装的
    求解器异常把「调用方把限位写反了」伪装成「这次求解无解」。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, (90.0, -90.0))

    # NOT placo's raw RuntimeError / pybind11 QPError: lo > hi is a caller error.
    with pytest.raises(ValueError) as excinfo:
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the inverted-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    # 精确片段，不做子串重叠：``"90" in message`` 同时被 ``hi=-90.0`` 满足（``"90"`` 是
    # ``"-90"`` 的子串），只报一侧的实现也能双双通过。``lo=`` / ``hi=`` 前缀把两侧钉开。
    assert "lo=90.0" in message, f"the error must show the lo side as 'lo=90.0', got: {message!r}"
    assert "hi=-90.0" in message, f"the error must show the hi side as 'hi=-90.0', got: {message!r}"


def test_t_ik_18_2_degenerate_joint_limit_rejected_naming_joint() -> None:
    """T-IK-18.2：退化限位 ``(0, 0)``（``lo == hi``）抛 ValueError，消息点名关节。

    现状是不抛、返回 ``ok=False`` / ``reason='JOINT_LIMIT'``——把「限位区间退化成一点」这个
    调用方构造错误伪装成一次正常的求解失败，调用方无法区分「机器人到不了」与「我把限位写错了」。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, (0.0, 0.0))

    with pytest.raises(ValueError) as excinfo:
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the degenerate-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )


@pytest.mark.parametrize(
    ("limit_value", "expected_fragments"),
    [
        ((float("nan"), 90.0), ("finite", "lo=nan", "hi=90.0")),
        ((-90.0, float("inf")), ("finite", "lo=-90.0", "hi=inf")),
    ],
    ids=["nan_lo", "inf_hi"],
)
def test_t_ik_18_3_non_finite_joint_limit_rejected_naming_joint(
    limit_value: tuple, expected_fragments: tuple[str, ...]
) -> None:
    """T-IK-18.3：限位含 ``NaN`` 或 ``inf`` 时抛 ValueError，消息点名关节并给出两侧取值。

    NaN 现状是求解器抛 ``RuntimeError('QPError: ... NaN in the QP solution')``；``inf`` 更
    糟——被静默接受，求解照常返回 ``ok=True``，等于没有上界。

    断言里必须有 ``finite`` 这个分支专属字样：删掉 ``math.isfinite`` 判断后，``NaN`` 不会
    走到求解器，而是被后面的 ``not lo < hi`` 拦住（``nan < 90.0`` 为 False），照样抛
    ValueError，消息里也照样出现 ``nan``——只断言「关节名 + 取值」的实现会漏掉这个变异。
    而 ``inf`` 在那一支里反而不抛（``-90 < inf`` 成立），会在 ``pytest.raises`` 处失败。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, limit_value)

    with pytest.raises(ValueError) as excinfo:
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the non-finite-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    for fragment in expected_fragments:
        assert fragment in message, (
            f"the non-finite-limit error must report {fragment!r} (branch marker + both sides); "
            f"missing from: {message!r}"
        )


def test_t_ik_18_4_extra_joint_limit_key_is_ignored() -> None:
    """T-IK-18.4：多余的限位键被忽略，结果与只有 6 个必需键时逐字段一致。

    比较字段：``ok`` / ``reason`` / ``position_error_m`` / ``orientation_error_deg`` /
    ``iterations`` / ``q_deg``（``q_deg`` 逐位比较，与 T-IK-11.2 的确定性比较同一风格）。
    ``solve_time_s`` 天然不等，不参与比较。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    extra_key = "left_extra_joint"
    assert extra_key not in JOINT_NAMES, "fixture drift: the extra key must not be a real joint"

    reference = solve_with_limits(limits_for())
    # 前提：没有多余键时这次求解确实成功，否则「一致」可能只是两条相同的失败。
    assert reference.ok is True, (
        f"test premise broken: the reference solve must succeed, got reason={reference.reason!r}"
    )

    with_extra = limits_for()
    with_extra[extra_key] = (SOLVE_LIMITS_DEG[0], SOLVE_LIMITS_DEG[1])
    result = solve_with_limits(with_extra)

    assert result.ok is reference.ok, (
        f"an extra limit key must not change ok: {result.ok!r} vs {reference.ok!r} "
        f"(compared: ok / reason / position_error_m / orientation_error_deg / iterations / q_deg)"
    )
    assert result.reason == reference.reason, (
        f"an extra limit key must not change reason: {result.reason!r} vs {reference.reason!r} "
        f"(compared: ok / reason / position_error_m / orientation_error_deg / iterations / q_deg)"
    )
    assert result.position_error_m == reference.position_error_m, (
        f"position_error_m drifted with an extra key: {result.position_error_m!r} vs "
        f"{reference.position_error_m!r} (compared: ok / reason / position_error_m / "
        f"orientation_error_deg / iterations / q_deg)"
    )
    assert result.orientation_error_deg == reference.orientation_error_deg, (
        f"orientation_error_deg drifted with an extra key: {result.orientation_error_deg!r} vs "
        f"{reference.orientation_error_deg!r} (compared: ok / reason / position_error_m / "
        f"orientation_error_deg / iterations / q_deg)"
    )
    assert result.iterations == reference.iterations, (
        f"iterations drifted with an extra key: {result.iterations!r} vs "
        f"{reference.iterations!r} (compared: ok / reason / position_error_m / "
        f"orientation_error_deg / iterations / q_deg)"
    )
    assert np.array_equal(np.asarray(result.q_deg), np.asarray(reference.q_deg)), (
        f"q_deg drifted with an extra key: {result.q_deg} vs {reference.q_deg} "
        f"(compared: ok / reason / position_error_m / orientation_error_deg / iterations / q_deg)"
    )


@pytest.mark.parametrize(
    ("limit_value", "expected_fragments"),
    [
        ((90.0,), ("got (90.0,)",)),
        ((0.0, 1.0, 2.0), ("got (0.0, 1.0, 2.0)",)),
        (("a", "b"), ("lo='a'", "hi='b'")),
    ],
    ids=["single_element", "three_elements", "non_numeric_elements"],
)
def test_t_ik_18_5_malformed_joint_limit_rejected_naming_joint(
    limit_value: tuple, expected_fragments: tuple[str, ...]
) -> None:
    """T-IK-18.5：限位不是「长度 2 的实数序列」时抛 ValueError，消息含收到的坏形态。

    现状：单元素 / 三元组由元组解包抛 ValueError（消息是
    ``not enough / too many values to unpack``，不含关节名，调用方看不出是哪个关节）；
    非实数元素由 ``np.deg2rad`` 抛 TypeError。

    除了关节名，还必须断言消息里带上收到的坏形态原样（``got (90.0,)`` /
    ``got (0.0, 1.0, 2.0)`` / ``lo='a', hi='b'``）：删掉取值后，调用方只能看到
    「限位不对」而看不到「不对成什么样」。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, limit_value)

    with pytest.raises(ValueError) as excinfo:  # NOT np.deg2rad's TypeError
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the malformed-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    for fragment in expected_fragments:
        assert fragment in message, (
            f"the malformed-limit error must echo the received value as {fragment!r}; "
            f"missing from: {message!r}"
        )


@pytest.mark.parametrize(
    ("limit_value", "bad_side_fragment"),
    [
        ((True, False), "lo=True"),
        ((0.0, True), "hi=True"),
    ],
    ids=["bool_pair", "bool_mixed_with_number"],
)
def test_t_ik_18_6_bool_joint_limit_is_not_accepted_as_a_number(
    limit_value: tuple, bad_side_fragment: str
) -> None:
    """T-IK-18.6：bool 不被当数字放行，抛 ValueError 并点名关节与坏取值。

    ``bool`` 是 ``int`` 的子类，``isinstance(x, (int, float))`` 和 ``np.deg2rad`` 都会放行：
    ``(True, False)`` 变成 ``(1.0, 0.0)``（倒置），``(0.0, True)`` 变成 ``(0.0, 1.0)``（顺序
    合法但语义荒谬）。后者是真正的鉴别点：只按数值比 ``lo < hi`` 的实现会接受它。

    注意：这里**刻意不写** ``assert isinstance(True, int)`` 之类的断言——那在任何解释器上
    都恒为真，只会给测试加噪音。陷阱在于 CPython 里 ``bool`` 是 ``int`` 的子类，因此任何
    仅做数值比较（``isinstance(x, (int, float))`` / ``lo < hi`` / ``np.deg2rad``）的实现都
    会放行 ``True`` / ``False``；真正的鉴别点是本用例要求它被当作「非实数」拒绝，并且
    消息里带上坏的那一侧取值。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, limit_value)

    with pytest.raises(ValueError) as excinfo:
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the bool-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    # 「非实数」这一支，而不是被后面的 ``not lo < hi`` 拦住（``(True, False)`` 当数字看是
    # 1.0 > 0.0，同样会抛 ValueError）。
    assert "real numbers" in message, (
        f"the bool limit must be rejected as a non-real number, got: {message!r}"
    )
    assert bad_side_fragment in message, (
        f"the bool-limit error must echo the offending side as {bad_side_fragment!r}; "
        f"missing from: {message!r}"
    )


def assert_solve_results_match(actual: Any, reference: Any, context: str) -> None:
    """逐字段断言两次 ``solve()`` 结果一致（与 T-IK-18.4 比较的字段集相同）。

    ``solve_time_s`` 天然不等，不参与比较；``q_deg`` 逐位比较，与 T-IK-11.2 的确定性比较
    同一风格。
    """
    compared = "ok / reason / position_error_m / orientation_error_deg / iterations / q_deg"
    assert actual.ok is reference.ok, (
        f"{context} must not change ok: {actual.ok!r} vs {reference.ok!r} (compared: {compared})"
    )
    assert actual.reason == reference.reason, (
        f"{context} must not change reason: {actual.reason!r} vs {reference.reason!r} "
        f"(compared: {compared})"
    )
    assert actual.position_error_m == reference.position_error_m, (
        f"position_error_m drifted ({context}): {actual.position_error_m!r} vs "
        f"{reference.position_error_m!r} (compared: {compared})"
    )
    assert actual.orientation_error_deg == reference.orientation_error_deg, (
        f"orientation_error_deg drifted ({context}): {actual.orientation_error_deg!r} vs "
        f"{reference.orientation_error_deg!r} (compared: {compared})"
    )
    assert actual.iterations == reference.iterations, (
        f"iterations drifted ({context}): {actual.iterations!r} vs {reference.iterations!r} "
        f"(compared: {compared})"
    )
    assert np.array_equal(np.asarray(actual.q_deg), np.asarray(reference.q_deg)), (
        f"q_deg drifted ({context}): {actual.q_deg} vs {reference.q_deg} (compared: {compared})"
    )


def test_t_ik_18_7_extra_joint_limit_key_with_invalid_value_is_ignored() -> None:
    """T-IK-18.7：多余键即使填**非法**限位也完全不参与校验，求解结果与 6 键基线一致。

    本用例钉的是「多余键完全不参与校验」——校验只覆盖 ``profile.joint_names``。

    为什么必须单独一条：T-IK-18.4 给多余键填的是合法值 ``(-90, 90)``，所以「遍历整个
    ``joint_limits_deg`` 字典做校验」这类错误实现同样能通过 18.4。把多余键
    ``left_extra_joint`` 填成倒置的 ``(90, -90)``，它就是个判别点：错误实现会抛
    ``ValueError``，正确实现根本不看它，``solve()`` 正常返回且结果与基线逐字段一致。

    比较字段同 T-IK-18.4：``ok`` / ``reason`` / ``position_error_m`` /
    ``orientation_error_deg`` / ``iterations`` / ``q_deg``（``solve_time_s`` 天然不等，
    不参与比较）。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    extra_key = "left_extra_joint"
    assert extra_key not in JOINT_NAMES, "fixture drift: the extra key must not be a real joint"

    # Arrange：6 个必需键的基线求解（无多余键）。
    reference = solve_with_limits(limits_for())
    # 前提：没有多余键时这次求解确实成功，否则「一致」可能只是两条相同的失败。
    assert reference.ok is True, (
        f"test premise broken: the reference solve must succeed, got reason={reference.reason!r}"
    )

    # Arrange：多一个不属于任何必需关节的键，且填的是倒置（非法）限位。
    with_extra = limits_for()
    with_extra[extra_key] = (SOLVE_LIMITS_DEG[1], SOLVE_LIMITS_DEG[0])

    # Act：若校验蔓延到多余键，这一行就会抛 ValueError。
    result = solve_with_limits(with_extra)

    # Assert：正常返回，且与 6 键基线逐字段一致。
    assert result.ok is True, (
        f"an illegal limit on an extra (non-joint) key must be ignored, so the solve must "
        f"still succeed; got ok={result.ok!r} reason={result.reason!r}"
    )
    assert_solve_results_match(result, reference, "an extra limit key (even with an illegal value)")


@pytest.mark.parametrize(
    ("limit_value", "expected_repr_fragment"),
    [
        ("90, -90", "'90, -90'"),
        (b"ab", "b'ab'"),
        (bytearray(b"ab"), "bytearray(b'ab')"),
    ],
    ids=["str", "bytes", "bytearray"],
)
def test_t_ik_18_8_text_joint_limit_rejected_as_mis_shaped(
    limit_value: object, expected_repr_fragment: str
) -> None:
    """T-IK-18.8：``str`` / ``bytes`` / ``bytearray`` 不是限位序列，抛 ValueError 并回显收到的对象。

    契约：限位是「长度 2 的实数序列」。文本类型在 Python 里恰好也是 ``Sequence``，且
    ``len(b"ab") == 2``、可下标取值、元素是 ``int``——所以只做「长度 2 的序列 + 实数取值」
    的实现会把 ``b"ab"`` 静默当成 ``(97, 98)``（一个合法但荒谬的限位）继续求解；
    ``bytearray(b"ab")`` 同理。``str`` 的 ``"90, -90"`` 长度不是 2，会落到同一条「形态不对」
    分支，但它的判别力来自「文本类型必须被显式排除」这一意图的回归网（删掉排除后它仍会
    被 ``len != 2`` 拦住，此点已在报告的 M-a 变异里实测）。

    断言三件事：抛 ``ValueError``（不是 placo 的裸 ``RuntimeError``，也不是静默求解）；消息点名
    关节；消息属于「形态不对」这一支（``must be a (lo, hi) pair``）并把收到的对象原样 repr
    回显（``b'ab'`` / ``bytearray(b'ab')`` / ``'90, -90'``）——否则调用方只看到「限位不对」
    而看不到「不对成什么样」。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, limit_value)

    with pytest.raises(ValueError) as excinfo:  # NOT placo's RuntimeError, NOT silent acceptance
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the mis-shaped-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    # 分支专属字样：「形态不对」这一支，而不是「实数值」或「有限性」那两支。
    assert "must be a (lo, hi) pair" in message, (
        f"a text-type limit must be rejected by the (lo, hi)-pair shape branch, got: {message!r}"
    )
    assert expected_repr_fragment in message, (
        f"the error must echo the received object as {expected_repr_fragment!r}; "
        f"missing from: {message!r}"
    )


def test_t_ik_18_9_overflowing_integer_joint_limit_stays_a_value_error() -> None:
    """T-IK-18.9：超出 ``float`` 范围的整数限位不得以 ``OverflowError`` 逃出 ``solve()``。

    ``10**400`` 类型上通得过 ``int`` 白名单（``math.isfinite`` 之前的类型检查放行），但
    ``math.isfinite(10**400)`` 本身抛 ``OverflowError: int too large to convert to float``，
    于是这个「调用方把限位写成了天文数字」的输入以一个既不是 :class:`IKResult` 也不是
    ``ValueError`` 的异常穿出 ``solve()``，破坏契约：调用方既不能用 ``IKResult`` 处理，也
    无法按「限位非法」这一类 ``ValueError`` 统一捕获。

    断言：仍然抛 ``ValueError``（不是 ``OverflowError``），且消息点名关节。消息正文里那串
    400 位数字不作断言，只钉「仍然是 ValueError + 点名关节」这个契约面。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, (10**400, 10**401))

    with pytest.raises(ValueError) as excinfo:  # NOT OverflowError
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the out-of-float-range-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: "
        f"{message[:200]!r}..."
    )


@pytest.mark.parametrize(
    "numpy_limits",
    [
        (np.float64(-90), np.float32(90)),
        (np.int64(-90), np.int64(90)),
    ],
    ids=["np_float64_np_float32", "np_int64_np_int64"],
)
def test_t_ik_18_10_numpy_numeric_scalar_joint_limits_accepted(
    numpy_limits: tuple[object, object],
) -> None:
    """T-IK-18.10：numpy 数值标量（``np.float*`` / ``np.int*``）限位被接受，且不改变求解结果。

    契约：限位可以是 Python ``int`` / ``float``，也可以是 numpy 数值标量——调用方从
    ``np.array`` / ``np.rad2deg(...)`` 里直接切片取值是常态。白名单因此写成
    ``(int, float, np.integer, np.floating)``；收窄成 ``(int, float)`` 会把 ``np.int64`` /
    ``np.float64`` 当成「调用方写错了限位」而误拒，本用例就是那一刀的分辨点。

    断言两件事：

    1. ``solve()`` 正常返回且 ``ok is True``——不是抛 ``ValueError``，不是返回失败结果；
    2. 与「同值 Python ``float`` 限位」的基线逐字段一致：``q_deg`` 用 ``np.allclose``，
       ``position_error_m`` 与 ``iterations`` 精确相等（``solve_time_s`` 天然不等，不比）。
       「接受」不等于「行为等价」；本用例同时钉住「numpy 标量走的是同一条数值路径」。

    注意 ``np.float32`` 一侧：实现内部用 ``float(np.deg2rad(value))`` 转换，``np.float32(90)``
    会先算成 float32 的 ``1.5707964``（与 float64 的 ``1.5707963267948966`` 差约 4e-8 rad）。
    本用例实测该差异不改变本次求解的 ``q_deg`` / ``position_error_m`` / ``iterations``，因此
    精确相等可写；若换目标位姿使 QP 更敏感，这里会先红，届时按数值容差处理而不是放宽断言。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")

    # Arrange：6 个关节都用 Python float 的合法限位，作为同值基线。
    reference = solve_with_limits(limits_for())
    assert reference.ok is True, (
        f"test premise broken: the reference solve must succeed, got reason={reference.reason!r}"
    )
    # Arrange：只把 LIMIT_VALIDATION_JOINT 的限位换成同值的 numpy 数值标量。
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, numpy_limits)

    # Act：白名单收窄成 (int, float) 时，这一行会抛 ValueError。
    result = solve_with_limits(limits)

    # Assert：numpy 标量被当实数接受，且求解走的是同一条数值路径。
    assert result.ok is True, (
        f"numpy numeric scalars are real numbers and must be accepted as limits; got "
        f"ok={result.ok!r} reason={result.reason!r} for {numpy_limits!r}"
    )
    assert np.allclose(np.asarray(result.q_deg), np.asarray(reference.q_deg)), (
        f"numpy scalar limits must not change q_deg: {result.q_deg} vs {reference.q_deg} "
        f"(limits: {numpy_limits!r})"
    )
    assert result.position_error_m == reference.position_error_m, (
        f"position_error_m drifted with numpy scalar limits: {result.position_error_m!r} vs "
        f"{reference.position_error_m!r} (limits: {numpy_limits!r})"
    )
    assert result.iterations == reference.iterations, (
        f"iterations drifted with numpy scalar limits: {result.iterations!r} vs "
        f"{reference.iterations!r} (limits: {numpy_limits!r})"
    )


@pytest.mark.parametrize(
    ("limit_value", "expected_repr_fragment"),
    [
        ((Fraction(-90, 1), Fraction(90, 1)), "Fraction(-90, 1)"),
        ((Decimal("-90"), Decimal("90")), "Decimal('-90')"),
    ],
    ids=["fraction", "decimal"],
)
def test_t_ik_18_11_fraction_and_decimal_joint_limits_rejected_as_value_error(
    limit_value: tuple[object, object], expected_repr_fragment: str
) -> None:
    """T-IK-18.11：``Fraction`` / ``Decimal`` 限位抛 ``ValueError``，**不是** ``TypeError``。

    契约：限位白名单是「Python ``int`` / ``float`` + numpy 数值标量」。``Fraction`` 是
    ``numbers.Real`` 的子类但不是 ``int`` / ``float``，``Decimal`` 连 ``numbers.Real`` 都不是：
    两者都得走同一条「非实数」``ValueError`` 分支。

    为什么必须钉住异常类型：把白名单换成 ``isinstance(value, numbers.Real)`` 会放行
    ``Fraction``，随后 ``np.deg2rad(Fraction)`` 抛裸 ``TypeError``——调用方按「限位非法」
    捕获 ``ValueError`` 的代码就漏掉了这个输入，看到的是一条来自求解器内部的类型错误。
    因此本用例的 ``pytest.raises(ValueError)`` 是判别点本身：``TypeError`` 不是
    ``ValueError`` 的子类，漏出时本用例直接红。

    断言四件事：

    1. 抛 ``ValueError``（不是 ``TypeError``、不是 placo 的裸异常、不是静默求解）；
    2. 消息点名关节 ``left_elbow_flex``；
    3. 消息属于「类型白名单」这一支（``must hold real numbers``），而不是「形态不对」
       （``must be a (lo, hi) pair``）或「有限性」那两支；
    4. 消息把收到的对象原样 repr 回显（``Fraction(-90, 1)`` / ``Decimal('-90')``）。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, limit_value)

    # NOT TypeError (what a numbers.Real whitelist lets np.deg2rad raise), NOT silent
    # acceptance of a non-int/float real-looking number.
    with pytest.raises(ValueError) as excinfo:
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the non-real-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    # 分支专属字样：「类型白名单」这一支，而不是形态 / 有限性 / lo < hi 那三支。
    assert "must hold real numbers" in message, (
        f"a non-int/float numeric limit must be rejected by the real-number whitelist "
        f"branch, got: {message!r}"
    )
    assert expected_repr_fragment in message, (
        f"the error must echo the received value as {expected_repr_fragment!r}; "
        f"missing from: {message!r}"
    )


# ---------------------------------------------------------------------------
# T-IK-18.12 / 18.13 —— 补 T-IK-18 回归网的两处空洞
#
# 变异实测（在本工作区实现上实跑）暴露了两个契约没有用例守着：
# - 删掉实现里 ``not isinstance(limit, Sequence)`` 后 T-IK-18 全部 19 条仍全绿
#   （``108`` 是当时**整文件**的条数，不是 T-IK-18 的条数；补上 18.12 / 18.13 之后
#   T-IK-18 现为 26 条），
#   而迭代器 / 生成器会漏出 ``TypeError: object of type 'list_iterator' has no len()``
#   （``len()`` 的裸 TypeError 逃出 ``solve()``）——见 T-IK-18.12；
# - ADR 声称 ``np.bool_`` 也被拒绝，但只覆盖了 Python ``bool``——把白名单扩成包含
#   ``np.bool_`` 后无用例变红——见 T-IK-18.13。
#
# 现状实测（未改实现，命令与原文见交付报告）：
# - ``iter([-90.0, 90.0])`` / ``(x for x in (-90.0, 90.0))`` / ``None`` /
#   ``{"lo": -90.0, "hi": 90.0}`` / ``40.0``
#   -> ValueError: ... must be a (lo, hi) pair, got <...>
# - ``(np.bool_(True), np.bool_(False))``
#   -> ValueError: ... must hold real numbers, got lo=np.True_ (full entry: ...)
# - ``(np.float64(-90), np.bool_(True))``
#   -> ValueError: ... must hold real numbers, got hi=np.True_ (full entry: ...)
# - ``memoryview(b"ab")`` -> **不抛**：``memoryview`` 在 ``collections.abc.Sequence`` 里，
#   被静默当成 ``(97, 98)`` 继续求解，返回 ``ok=False`` / ``reason='JOINT_LIMIT'``。
#   这是实现缺口而不是已满足的契约，因此**不**写进参数化列表（不写当下必红的用例），
#   留作待裁决项上报。
# ---------------------------------------------------------------------------


def _mis_shaped_iterator_limit() -> object:
    """迭代器形态的「限位」：不是 ``Sequence``，``len()`` 会抛 ``TypeError``。"""
    return iter([-90.0, 90.0])


def _mis_shaped_generator_limit() -> object:
    """生成器形态的「限位」：不是 ``Sequence``，``len()`` 会抛 ``TypeError``。"""
    return (x for x in (-90.0, 90.0))


def _mis_shaped_none_limit() -> object:
    """``None`` 作为「限位」：既不是序列，也没有长度。"""
    return None


def _mis_shaped_dict_limit() -> object:
    """两键 ``dict`` 作为「限位」：``len() == 2``、可迭代出键，但不是 ``Sequence``。"""
    return {"lo": -90.0, "hi": 90.0}


def _mis_shaped_scalar_limit() -> object:
    """裸 ``float`` 作为「限位」：既没有长度，也不能解包。"""
    return 40.0


@pytest.mark.parametrize(
    "limit_factory",
    [
        _mis_shaped_iterator_limit,
        _mis_shaped_generator_limit,
        _mis_shaped_none_limit,
        _mis_shaped_dict_limit,
        _mis_shaped_scalar_limit,
    ],
    ids=["iterator", "generator", "none", "dict", "float_scalar"],
)
def test_t_ik_18_12_non_sequence_joint_limit_rejected_as_mis_shaped(
    limit_factory: Callable[[], object],
) -> None:
    """T-IK-18.12：非序列限位（迭代器 / 生成器 / ``None`` / ``dict`` / 裸标量）抛 ValueError。

    契约：限位是「长度 2 的实数序列」。实现先用 ``not isinstance(limit, Sequence)`` 把它们
    挡在数值工作之前；删掉这条判据后，``len(limit)`` 会对迭代器 / 生成器 / ``None`` /
    ``float`` 抛裸 ``TypeError: object of type '...' has no len()``，而两键 ``dict`` 会通过
    ``len() == 2`` 并被解包成键 ``("lo", "hi")``，随后落到「非实数」那一支——异常类型或
    消息分支都不对。调用方按「限位非法 = ValueError」捕获，看到的是上述三种错法之一。

    参数用工厂函数而不是裸对象：生成器 / 迭代器是一次性的，若把对象直接放进
    ``parametrize`` 列表（模块导入时构造一次），同一参数的重跑（``--count``、重复收集）
    会拿到已被抽干的实例，变成依赖执行顺序的测试。

    断言三件事：抛 ``ValueError``（不是 ``len()`` 的 ``TypeError``、不是静默求解）；消息点名
    关节；消息属于「形态不对」这一支（``must be a (lo, hi) pair``），而不是「非实数」或
    「有限性」那两支。迭代器 / 生成器的 ``repr`` 带进程内地址，不作断言。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, limit_factory())

    with pytest.raises(ValueError) as excinfo:  # NOT len()'s TypeError, NOT silent solve
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the non-sequence-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    # 分支专属字样：「形态不对」这一支，而不是「实数值」或「有限性」那两支。
    assert "must be a (lo, hi) pair" in message, (
        f"a non-sequence limit must be rejected by the (lo, hi)-pair shape branch, got: {message!r}"
    )


@pytest.mark.parametrize(
    ("limit_value", "expected_fragments"),
    [
        ((np.bool_(True), np.bool_(False)), ("got lo=np.True_", "hi=np.False_")),
        ((np.float64(-90), np.bool_(True)), ("got hi=np.True_",)),
    ],
    ids=["np_bool_pair", "np_bool_mixed_with_number"],
)
def test_t_ik_18_13_numpy_bool_joint_limit_is_not_accepted_as_a_number(
    limit_value: tuple[object, object], expected_fragments: tuple[str, ...]
) -> None:
    """T-IK-18.13：``np.bool_`` 不被当数字放行，抛 ValueError 并点名关节与坏取值。

    ``np.bool_`` 既不是 ``int`` 的子类（与 Python ``bool`` 不同），也不在
    ``(int, float, np.integer, np.floating)`` 白名单里，所以当前实现把它挡在「非实数」这一支。
    风险在另一侧：``np.bool_`` 是 numpy 标量，白名单一旦为了「宽容 numpy 标量」扩成
    ``(..., np.bool_)``，``(np.bool_(True), np.bool_(False))`` 会变成 ``(1.0, 0.0)``（倒置，
    随后被 ``lo < hi`` 拦住、消息换了分支），``(np.float64(-90), np.bool_(True))`` 会变成
    ``(-90.0, 1.0)``（顺序合法，求解照常返回）——两个都不该发生。T-IK-18.6 只覆盖 Python
    ``bool``，守不住这一刀；本条是它的 numpy 补充。

    断言四件事：抛 ``ValueError``（不是 placo 的裸 ``RuntimeError``，也不是静默求解）；消息
    点名关节；消息属于「类型白名单」这一支（``must hold real numbers``），而不是顺序 / 有限性
    那两支；消息把坏的那一侧以 ``got <侧>=<repr>`` 前缀原样回显，并带上 ``np.True_`` /
    ``np.False_`` 的 repr（numpy 2.x 的标量 repr；见交付报告的遗留一节）。
    ``got lo=`` / ``got hi=`` 前缀把「哪一侧坏了」钉开：只回显整个条目的实现漏掉前缀就红。

    ``(np.float64(-90), np.bool_(True))`` 的参数是真正的鉴别点：只按数值比 ``lo < hi`` 的实现
    会接受它（``-90.0 < 1.0``）。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, limit_value)

    with pytest.raises(ValueError) as excinfo:  # NOT silent acceptance of np.bool_ as a number
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the np.bool_ limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    # 分支专属字样：「类型白名单」这一支，而不是顺序 / 有限性那两支。
    assert "must hold real numbers" in message, (
        f"an np.bool_ limit must be rejected by the real-number whitelist branch, got: {message!r}"
    )
    for fragment in expected_fragments:
        assert fragment in message, (
            f"the np.bool_ limit error must echo the offending side as {fragment!r}; "
            f"missing from: {message!r}"
        )


# ---------------------------------------------------------------------------
# T-IK-18.14 / 18.15 —— 上一段结尾上报的两处缺口，本轮裁决为「写进回归网」
#
# 18.12 / 18.13 的小节注释当时把 ``memoryview(b"ab")`` 记为「实现缺口、留作待裁决项上报」
# 而没有写成用例；本轮裁决是把它钉成契约（T-IK-18.14），并补上大整数在 ``np.deg2rad``
# 抛裸 ``TypeError`` 的同类缺口（T-IK-18.15）。两条都是**当下实跑为红**的红阶段用例。
#
# 现状实测（未改实现，种子 ``SOLVE_SEED_Q`` + ``elbow_bend`` 冻结位姿，只改
# ``LIMIT_VALIDATION_JOINT`` 的限位，命令与原文见交付报告）：
# - ``memoryview(b"ab")``   -> **不抛**：``memoryview`` 是 ``collections.abc.Sequence``、
#   ``len() == 2``、元素是 ``int``（97 / 98），依次穿过形态 / 实数 / 有限性 / ``lo < hi``
#   四道判据，被当成「97 deg – 98 deg」这个荒谬限位继续求解，返回 ``ok=False`` /
#   ``reason='JOINT_LIMIT'``（``q[left_elbow_flex]`` 实测被夹到 97.0）——
#   调用方看到的是「机器人到不了」，而不是「我把限位写错了」。
# - ``memoryview(b"abcd")`` -> 已被拦住：``len() != 2`` 落回「形态不对」这一支
#   （``arm_ik.py`` 第 357 / 360 行），实测 ValueError: ... must be a (lo, hi) pair,
#   got <memory at 0x...>；本轮只核对，不新增用例。
# - ``bytearray(b"ab")``    -> 已被拦住：``isinstance(limit, (str, bytes, bytearray))``
#   （``arm_ik.py`` 第 355 行）落回同一支，实测 ValueError: ... must be a (lo, hi) pair,
#   got bytearray(b'ab')；同样只核对，既有用例是 T-IK-18.8。
# - ``(2**63, 2**64)``       -> 四道判据全过（``math.isfinite(2**64)`` 为 ``True``），随后在
#   ``_solve_once`` 的 ``np.deg2rad``（``arm_ik.py`` 第 228 行）抛裸
#   ``TypeError: loop of ufunc does not support argument 0 of type int which has no
#   callable deg2rad method``，直接穿出 ``solve()``。
# ---------------------------------------------------------------------------


def test_t_ik_18_14_memoryview_joint_limit_rejected_as_mis_shaped() -> None:
    """T-IK-18.14：``memoryview(b"ab")`` 不是限位序列，抛 ValueError 并回显收到的对象。

    契约：限位是「长度 2 的实数序列」，而「序列」在这里的含义是**限位的两个端点**，不是
    「任何恰好有长度 2 的可下标对象」。``memoryview`` 与 ``str`` / ``bytes`` / ``bytearray``
    一样是「恰好也长得像长度 2 序列」的非限位类型：它被注册进
    ``collections.abc.Sequence``，``len(memoryview(b"ab")) == 2``，元素是 ``int``
    （97 / 98），因此能依次穿过形态 / 实数 / 有限性 / ``lo < hi`` 四道判据，被静默当成
    ``(97, 98)`` 这个合法但荒谬的限位继续求解——现状返回 ``ok=False`` /
    ``reason='JOINT_LIMIT'``，把「调用方把限位写错成了二进制缓冲」伪装成一次正常的求解
    失败（与 T-IK-18.2 的退化限位同一类错误）。

    断言三件事：抛 ``ValueError``（不是静默求解、不是 ``ok=False`` 结果，也不是 placo 的裸
    ``RuntimeError``）；消息点名关节 ``left_elbow_flex``；消息属于「形态不对」这一支
    （``must be a (lo, hi) pair``）并把收到的对象按 repr 回显。

    回显断言只钉 ``repr`` 的稳定前缀 ``got <memory at 0x``：``memoryview`` 的 repr 里带
    进程内地址，写全等于把随机地址钉进断言（与 T-IK-18.12 不对迭代器 ``repr`` 断言的
    理由相同）；``<memory at 0x`` 这个前缀已经是「回显的是 memoryview 本身，而不是
    ``(97, 98)`` 之类的解包结果」的判别点。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    # 长度 2 的 memoryview：元素恰好是两个 int，形态判据（含 len == 2）挡不住它。
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, memoryview(b"ab"))

    # NOT a returned ok=False / reason='JOINT_LIMIT', NOT silent acceptance as (97, 98).
    with pytest.raises(ValueError) as excinfo:
        solve_with_limits(limits)

    message = str(excinfo.value)
    assert LIMIT_VALIDATION_JOINT in message, (
        f"the memoryview-limit error must name {LIMIT_VALIDATION_JOINT!r}, got: {message!r}"
    )
    # 分支专属字样：「形态不对」这一支，而不是「实数值」或「有限性」那两支。
    assert "must be a (lo, hi) pair" in message, (
        f"a memoryview limit must be rejected by the (lo, hi)-pair shape branch, got: {message!r}"
    )
    assert "got <memory at 0x" in message, (
        f"the error must echo the received memoryview (repr prefix 'got <memory at 0x'), "
        f"not an unpacked value; missing from: {message!r}"
    )


def test_t_ik_18_15_huge_integer_joint_limit_never_raises_type_error() -> None:
    """T-IK-18.15：``(2**63, 2**64)`` 限位不得让裸 ``TypeError`` 逃出 ``solve()``。

    ``2**63`` / ``2**64`` 是合法的 Python ``int``，``math.isfinite(2**64)`` 为 ``True``，
    ``2**63 < 2**64`` 也成立——入口的四道判据全部放行。真正出问题的地方在数值路径：
    ``_solve_once`` 里的 ``np.deg2rad(lo_deg)``（``arm_ik.py`` 第 228 行）对这个量级的
    ``int`` 抛裸 ``TypeError: loop of ufunc does not support argument 0 of type int which
    has no callable deg2rad method``，于是「调用方把限位写成了天文数字」以一个既不是
    :class:`IKResult` 也不是 ``ValueError`` 的异常穿出 ``solve()``。调用方按「限位非法 =
    ``ValueError``」统一捕获的代码漏掉这个输入，看到的是一条来自 ufunc 内部的类型错误
    （与 T-IK-18.9 的 ``OverflowError``、T-IK-18.11 的 ``Fraction`` ``TypeError`` 同一类
    逃逸路径）。

    本用例只钉「不许有 ``TypeError`` 逃出」这一条，不钉定位修复后的具体结局：
    ``ValueError``（入口按非法限位拒绝）、placo 的 ``RuntimeError``（``QPError:
    Infeasible QP``）、以及正常返回（``ok`` 为 ``True`` 或 ``False`` 均可，``reason`` 不限）
    都算通过。这样写的理由是这个输入的「正确结局」是待裁决的设计选择，红阶段只钉住
    已经确定的契约面——异常类型不许是 ``TypeError``。
    """
    pytest.importorskip("placo", reason="ArmIK 的数值求解需要可选依赖 placo")
    limits = limits_replacing(LIMIT_VALIDATION_JOINT, (2**63, 2**64))

    try:
        result = solve_with_limits(limits)
    except TypeError as exc:
        # np.deg2rad 的裸 TypeError 从这里逃出 solve()：本轮要钉住的缺口本身。
        pytest.fail(f"limit validation must not let TypeError escape solve(): {exc!r}")
    except (ValueError, RuntimeError):
        # ValueError：入口按「限位非法」拒绝（首选）。
        # RuntimeError：placo 的 QPError，可接受的次选结局。
        return

    # 正常返回同样可接受：ok / reason 取值不限，只要求返回的是 IKResult 而不是异常。
    for field in ("q_deg", "ok", "reason"):
        assert hasattr(result, field), (
            f"solve() must return an IKResult (missing {field!r}): {result!r}"
        )
