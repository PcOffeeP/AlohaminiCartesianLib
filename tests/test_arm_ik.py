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
from collections.abc import Iterator
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
