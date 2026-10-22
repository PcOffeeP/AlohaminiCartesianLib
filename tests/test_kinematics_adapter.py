"""关节运动学适配器（``JointKinematicAdapter``）的行为契约测试（TDD 红阶段）。

从 lerobot 仓库的 ``tests/robots/test_alohamini_cartesian_adapter.py`` 搬运，
模块路径按本库映射为 ``alohamini_cartesian.kinematics.joint_adapter``（旧
``lerobot.robots.alohamini.joint_kinematic_adapter``）。

本文件不导入 lerobot、不导入 AlohaMini / FeetechMotorsBus：全部用例只用合成
metadata、注入的 signs/zeros 与冻结字面期望，不连设备、不联网。凡是必须以
lerobot 生产 bus / 生产发送端为 oracle 的差分对照用例拆到
``tests/test_kinematics_lerobot_parity.py``（未装 runtime extra 时整体跳过）。
"""

# Behavior T-B1 (P1-B 单位适配, design §8): forward conversion of the six
# kinematic joints in DEGREES mode by the new module
# `lerobot.robots.alohamini.joint_kinematic_adapter.JointKinematicAdapter`:
#
#   observation_to_kinematic_deg(side, observation, metadata)
#
# returns a shape (6,) ndarray with values q_urdf = sign * motor_deg + zero,
# one per joint, in the frozen order
# [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw,
#  wrist_roll]. The metadata motors dict may be inserted in any order and may
# mix gripper / base-wheel / lift / other-arm interference keys without
# affecting the result. Sign and zero are injected per side via the
# constructor; the observation direction only checks finiteness and
# completeness of the six required keys (no range/limit checks in T-B1) and
# must NOT re-apply homing_offset / drive_mode / resolution: in DEGREES mode
# the bus has already turned ticks into motor degrees, so the adapter sees
# plain degrees (MotorsBus._normalize DEGREES branch).
#
# Cases:
# - T-B1.1 left main case: frozen order + frozen literal expectations. Catches
#   the identity implementation (ignores sign/zero), a negated zero, a
#   re-applied homing_offset/drive_mode, and any tick math or Feetech
#   resolution-table lookup (synthetic model names + resolutions).
# - T-B1.2 right main case: per-side injection with values different from the
#   left arm. Catches reading the other arm and applying the left-side
#   sign/zero to a right request.
# - T-B1.3 shuffled variant: metadata motors inserted in scrambled order with
#   non-monotonic motor ids, observation dict scrambled too. Catches dict
#   insertion-order dependence and motor-id-based ordering.
# - T-B1.4 interference variant: metadata and observation additionally carry
#   the other arm, both grippers, three base wheels and the lift axis. Catches
#   leaking any of those into the six-joint vector (shape/value mismatch).
# - T-B1.5 identity-config auxiliary case: all signs +1 / zeros 0 pass the
#   values through unchanged. It only proves the identity configuration does
#   not crash and passes values through; it deliberately CANNOT distinguish an
#   identity implementation — that is T-B1.1's job.
#
# SYNTHETIC FIXTURE NOTICE: every sign, zero, calibration and motor-model
# value below is SYNTHETIC, ASYMMETRIC and NOT USABLE ON REAL HARDWARE.
# Model names ("syn-enc-*") and resolutions deliberately avoid the real
# Feetech MODEL_RESOLUTION table (4096 / 1024). Real sign/zero/limits require
# Gate A/B evidence and must never be copied from this file.
#
# Harness (same pattern as the protocol tests): pure objects only. The
# adapter is imported inside make_adapter() so a missing implementation fails
# each case individually instead of erroring the whole module (which would
# drag unrelated cases down with it). No bus, no clock, no network, no
# randomness, no sleeps. Expected values are frozen literal constants; float
# comparisons use pytest.approx(abs=1e-9).

import copy
import inspect

import numpy as np
import pytest

# Frozen kinematic joint order (six joints; the gripper is NOT a kinematic
# joint). The output vector follows exactly this order.
JOINT_ORDER = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)

# Per-side kinematic injection content for the constructor seam (see
# make_adapter). Signs mix +1 and -1 on both sides; zeros are asymmetric,
# non-zero and different per side, so any swapped/negated/identity use of
# them shifts the frozen expectations.
KINEMATIC_SIGNS = {
    "left": {
        "shoulder_pan": 1.0,
        "shoulder_lift": -1.0,
        "elbow_flex": 1.0,
        "wrist_flex": -1.0,
        "wrist_yaw": 1.0,
        "wrist_roll": -1.0,
    },
    "right": {
        "shoulder_pan": -1.0,
        "shoulder_lift": 1.0,
        "elbow_flex": -1.0,
        "wrist_flex": 1.0,
        "wrist_yaw": -1.0,
        "wrist_roll": 1.0,
    },
}
KINEMATIC_ZEROS = {
    "left": {
        "shoulder_pan": 17.5,
        "shoulder_lift": -43.25,
        "elbow_flex": 91.75,
        "wrist_flex": -8.5,
        "wrist_yaw": 63.0,
        "wrist_roll": -120.5,
    },
    "right": {
        "shoulder_pan": -71.25,
        "shoulder_lift": 55.5,
        "elbow_flex": -12.0,
        "wrist_flex": 144.5,
        "wrist_yaw": -97.75,
        "wrist_roll": 31.0,
    },
}

# Motor-degree observations (as the DEGREES-mode bus would report them).
LEFT_MOTOR_DEG = {
    "shoulder_pan": 12.0,
    "shoulder_lift": -34.5,
    "elbow_flex": 7.25,
    "wrist_flex": 100.0,
    "wrist_yaw": -88.75,
    "wrist_roll": 0.5,
}
RIGHT_MOTOR_DEG = {
    "shoulder_pan": -9.5,
    "shoulder_lift": 21.0,
    "elbow_flex": 44.25,
    "wrist_flex": -60.0,
    "wrist_yaw": 3.75,
    "wrist_roll": -77.0,
}

# Frozen expected outputs, one literal per joint in JOINT_ORDER, computed by
# hand as sign * motor_deg + zero with the injected constants above.
LEFT_EXPECTED_KINEMATIC_DEG = (29.5, -8.75, 99.0, -108.5, -25.75, -121.0)
RIGHT_EXPECTED_KINEMATIC_DEG = (-61.75, 76.5, -56.25, 84.5, -101.5, -46.0)

# Synthetic per-motor metadata entries, mirroring the real metadata v2 shape
# (build_robot_metadata): id / model / normalization / drive_mode /
# range_min / range_max / homing_offset / resolution. All values are
# synthetic; models and resolutions avoid the real Feetech tables;
# homing_offsets are non-zero and drive_mode mixes 0/1 so a re-application of
# either is caught.
LEFT_MOTOR_SPECS = {
    "shoulder_pan": {
        "id": 1,
        "model": "syn-enc-a1",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 41,
        "range_max": 4001,
        "homing_offset": 333,
        "resolution": 6399,
    },
    "shoulder_lift": {
        "id": 2,
        "model": "syn-enc-a2",
        "normalization": "degrees",
        "drive_mode": 1,
        "range_min": 173,
        "range_max": 3863,
        "homing_offset": -1717,
        "resolution": 5777,
    },
    "elbow_flex": {
        "id": 3,
        "model": "syn-enc-a3",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 911,
        "range_max": 3201,
        "homing_offset": 907,
        "resolution": 7331,
    },
    "wrist_flex": {
        "id": 4,
        "model": "syn-enc-a4",
        "normalization": "degrees",
        "drive_mode": 1,
        "range_min": 613,
        "range_max": 3517,
        "homing_offset": -555,
        "resolution": 6143,
    },
    "wrist_yaw": {
        "id": 5,
        "model": "syn-enc-a5",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 227,
        "range_max": 3739,
        "homing_offset": 99,
        "resolution": 8191,
    },
    "wrist_roll": {
        "id": 6,
        "model": "syn-enc-a6",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 7,
        "range_max": 4061,
        "homing_offset": -2049,
        "resolution": 9999,
    },
}
RIGHT_MOTOR_SPECS = {
    "shoulder_pan": {
        "id": 1,
        "model": "syn-enc-b1",
        "normalization": "degrees",
        "drive_mode": 1,
        "range_min": 97,
        "range_max": 3977,
        "homing_offset": -808,
        "resolution": 5405,
    },
    "shoulder_lift": {
        "id": 2,
        "model": "syn-enc-b2",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 331,
        "range_max": 3673,
        "homing_offset": 1234,
        "resolution": 6673,
    },
    "elbow_flex": {
        "id": 3,
        "model": "syn-enc-b3",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 787,
        "range_max": 3329,
        "homing_offset": -66,
        "resolution": 7507,
    },
    "wrist_flex": {
        "id": 4,
        "model": "syn-enc-b4",
        "normalization": "degrees",
        "drive_mode": 1,
        "range_min": 149,
        "range_max": 3949,
        "homing_offset": 3011,
        "resolution": 5821,
    },
    "wrist_yaw": {
        "id": 5,
        "model": "syn-enc-b5",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 449,
        "range_max": 3613,
        "homing_offset": -77,
        "resolution": 8123,
    },
    "wrist_roll": {
        "id": 6,
        "model": "syn-enc-b6",
        "normalization": "degrees",
        "drive_mode": 1,
        "range_min": 61,
        "range_max": 4033,
        "homing_offset": 4144,
        "resolution": 9377,
    },
}

# Interference-only motors: grippers (RANGE_0_100 like the real robot),
# base wheels (RANGE_M100_100) and the lift axis (DEGREES). Never part of the
# six-joint output.
GRIPPER_MOTOR_SPECS = {
    "left": {
        "id": 7,
        "model": "syn-enc-g1",
        "normalization": "range_0_100",
        "drive_mode": 0,
        "range_min": 911,
        "range_max": 3987,
        "homing_offset": -234,
        "resolution": 7111,
    },
    "right": {
        "id": 7,
        "model": "syn-enc-g2",
        "normalization": "range_0_100",
        "drive_mode": 1,
        "range_min": 1013,
        "range_max": 3869,
        "homing_offset": 917,
        "resolution": 6599,
    },
}
BASE_MOTOR_SPECS = {
    "base_left_wheel": {
        "id": 8,
        "model": "syn-enc-w1",
        "normalization": "range_m100_100",
        "drive_mode": 0,
        "range_min": 355,
        "range_max": 3743,
        "homing_offset": 512,
        "resolution": 7013,
    },
    "base_back_wheel": {
        "id": 9,
        "model": "syn-enc-w2",
        "normalization": "range_m100_100",
        "drive_mode": 1,
        "range_min": 289,
        "range_max": 3809,
        "homing_offset": -671,
        "resolution": 6541,
    },
    "base_right_wheel": {
        "id": 10,
        "model": "syn-enc-w3",
        "normalization": "range_m100_100",
        "drive_mode": 0,
        "range_min": 401,
        "range_max": 3697,
        "homing_offset": 128,
        "resolution": 6793,
    },
    "lift_axis": {
        "id": 11,
        "model": "syn-enc-l1",
        "normalization": "degrees",
        "drive_mode": 0,
        "range_min": 131,
        "range_max": 3967,
        "homing_offset": -1290,
        "resolution": 6317,
    },
}

# Frozen scrambled insertion sequences (no runtime randomness). Motor ids in
# T-B1.3 are non-monotonic so an id-sorting implementation is caught too.
SCRAMBLED_JOINT_SEQUENCE = (
    "wrist_roll",
    "shoulder_pan",
    "wrist_yaw",
    "elbow_flex",
    "shoulder_lift",
    "wrist_flex",
)
SCRAMBLED_LEFT_IDS = {
    "shoulder_pan": 4,
    "shoulder_lift": 1,
    "elbow_flex": 6,
    "wrist_flex": 2,
    "wrist_yaw": 5,
    "wrist_roll": 3,
}

# Interference-only observation values (finite, deliberately distinct from
# every arm value so a leaked merge shifts the frozen expectation).
INTERFERENCE_OBS_DEG = {
    "arm_left_gripper.pos": 61.0,
    "arm_right_gripper.pos": 89.5,
    "base_left_wheel.pos": -53.5,
    "base_back_wheel.pos": 17.25,
    "base_right_wheel.pos": 0.5,
    "lift_axis.pos": 1184.5,
}


def make_adapter(
    *,
    identity: bool = False,
    urdf_limits: dict[str, dict[str, tuple[float, float]]] | None = None,
    zero_overrides: dict[str, dict[str, float]] | None = None,
    sign_overrides: dict[str, dict[str, float]] | None = None,
    drop_sign_joints: dict[str, tuple[str, ...]] | None = None,
    drop_zero_joints: dict[str, tuple[str, ...]] | None = None,
) -> object:
    """Sole constructor seam of these tests.

    Binds the public seam: the class `JointKinematicAdapter` is injected, per
    side, with its kinematic signs and zero offsets (joint -> value). The
    exact constructor keyword shape is bound here and only here; if the
    implementation chooses a different shape, that is a binding-level change
    to be re-planned, not silently adapted.

    T-B6 extensions (same seam, still bound here and only here):
    - `urdf_limits`: per-side per-joint URDF joint-limit window
      (side -> joint -> (lower, upper)) in kinematic degrees, injected into
      the constructor. `None` (the default) does NOT pass the keyword at all,
      so the T-B1–T-B5 constructor shape (signs/zeros only) stays valid and
      all pre-existing cases are unaffected.
    - `zero_overrides`: side -> joint -> value merged into the default zeros
      (used by T-B6.g3 to inject a non-finite zero offset).

    T-B11 extensions (same seam, still bound here and only here):
    - `sign_overrides`: side -> joint -> value merged into the default signs
      (T-B11: inject invalid sign values — non ±1 gains, numeric strings,
      bools). The merge uses setdefault, so it may ADD a new side key: that
      is exactly the unknown-side injection shape.
    - `drop_sign_joints` / `drop_zero_joints`: side -> joint names removed
      from the default signs / zeros respectively (T-B11: missing-joint
      configs, per dict or both).
    """
    from alohamini_cartesian.kinematics.joint_adapter import JointKinematicAdapter

    if identity:
        signs = {side: dict.fromkeys(JOINT_ORDER, 1.0) for side in ("left", "right")}
        zeros = {side: dict.fromkeys(JOINT_ORDER, 0.0) for side in ("left", "right")}
    else:
        signs = {side: dict(KINEMATIC_SIGNS[side]) for side in ("left", "right")}
        zeros = {side: dict(KINEMATIC_ZEROS[side]) for side in ("left", "right")}
    if zero_overrides:
        for side, overrides in zero_overrides.items():
            zeros.setdefault(side, {}).update(overrides)
    if sign_overrides:
        for side, overrides in sign_overrides.items():
            signs.setdefault(side, {}).update(overrides)
    if drop_sign_joints:
        for side, joints in drop_sign_joints.items():
            for joint in joints:
                signs.get(side, {}).pop(joint, None)
    if drop_zero_joints:
        for side, joints in drop_zero_joints.items():
            for joint in joints:
                zeros.get(side, {}).pop(joint, None)
    if urdf_limits is None:
        # Pre-T-B6 constructor shape: no URDF-limit injection at all.
        return JointKinematicAdapter(signs=signs, zeros=zeros)
    return JointKinematicAdapter(signs=signs, zeros=zeros, urdf_limits=urdf_limits)


def make_arm_observation(side: str, motor_deg: dict[str, float]) -> dict[str, float]:
    """Observation state dict for one arm: `arm_{side}_{joint}.pos` keys."""
    return {f"arm_{side}_{joint}.pos": motor_deg[joint] for joint in JOINT_ORDER}


def make_metadata(motors: dict) -> dict:
    """Metadata v2 envelope, mirroring build_robot_metadata's public shape."""
    return {
        "schema_version": 2,
        "robot_model": "alohamini2pro",
        "motors": motors,
    }


def make_arm_motors(
    side: str,
    insertion_sequence: tuple[str, ...] = JOINT_ORDER,
    *,
    id_overrides: dict[str, int] | None = None,
) -> dict[str, dict]:
    """Metadata motors entries for one arm in the given insertion order."""
    specs = LEFT_MOTOR_SPECS if side == "left" else RIGHT_MOTOR_SPECS
    motors = {}
    for joint in insertion_sequence:
        entry = dict(specs[joint])
        if id_overrides is not None:
            entry["id"] = id_overrides[joint]
        motors[f"arm_{side}_{joint}"] = entry
    return motors


def test_t_b1_left_forward_conversion_six_joints_frozen_order_and_values() -> None:
    """T-B1.1：left 正向换算按冻结关节顺序产出冻结字面值。"""
    adapter = make_adapter()
    observation = make_arm_observation("left", LEFT_MOTOR_DEG)
    metadata = make_metadata(make_arm_motors("left"))

    result = adapter.observation_to_kinematic_deg("left", observation, metadata)

    assert isinstance(result, np.ndarray)
    assert result.shape == (6,)
    assert result == pytest.approx(list(LEFT_EXPECTED_KINEMATIC_DEG), abs=1e-9)


def test_t_b1_right_forward_conversion_uses_right_side_injection() -> None:
    """T-B1.2：right 正向换算使用右侧注入的 sign/zero。"""
    adapter = make_adapter()
    observation = {
        **make_arm_observation("left", LEFT_MOTOR_DEG),
        **make_arm_observation("right", RIGHT_MOTOR_DEG),
    }
    metadata = make_metadata({**make_arm_motors("left"), **make_arm_motors("right")})

    result = adapter.observation_to_kinematic_deg("right", observation, metadata)

    assert isinstance(result, np.ndarray)
    assert result.shape == (6,)
    assert result == pytest.approx(list(RIGHT_EXPECTED_KINEMATIC_DEG), abs=1e-9)


def test_t_b1_shuffled_metadata_insertion_order_and_ids_preserve_result() -> None:
    """T-B1.3：metadata 插入顺序与电机 id 乱序不改变结果。"""
    adapter = make_adapter()
    # Both the metadata motors dict and the observation dict are built in a
    # frozen scrambled order, with non-monotonic motor ids.
    shuffled_observation = {
        f"arm_left_{joint}.pos": LEFT_MOTOR_DEG[joint] for joint in SCRAMBLED_JOINT_SEQUENCE
    }
    metadata = make_metadata(
        make_arm_motors(
            "left",
            insertion_sequence=SCRAMBLED_JOINT_SEQUENCE,
            id_overrides=SCRAMBLED_LEFT_IDS,
        )
    )

    result = adapter.observation_to_kinematic_deg("left", shuffled_observation, metadata)

    assert isinstance(result, np.ndarray)
    assert result.shape == (6,)
    assert result == pytest.approx(list(LEFT_EXPECTED_KINEMATIC_DEG), abs=1e-9)


def test_t_b1_interference_keys_gripper_base_other_arm_do_not_leak() -> None:
    """T-B1.4：夹爪/底轮/升降/另一臂的干扰键不得泄漏进六关节向量。"""
    adapter = make_adapter()
    observation = {
        **make_arm_observation("left", LEFT_MOTOR_DEG),
        **make_arm_observation("right", RIGHT_MOTOR_DEG),
        **INTERFERENCE_OBS_DEG,
    }
    metadata = make_metadata(
        {
            **make_arm_motors("left"),
            **make_arm_motors("right"),
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )

    result = adapter.observation_to_kinematic_deg("left", observation, metadata)

    assert isinstance(result, np.ndarray)
    assert result.shape == (6,)
    assert result == pytest.approx(list(LEFT_EXPECTED_KINEMATIC_DEG), abs=1e-9)


def test_t_b1_identity_config_passes_through_all_six_joints() -> None:
    # Auxiliary case only: proves the identity configuration does not crash
    # and passes values through per joint. It cannot distinguish an identity
    # implementation — that is the main case's job.
    """T-B1.5：恒等配置原样透传六个关节值。"""
    adapter = make_adapter(identity=True)
    observation = make_arm_observation("left", LEFT_MOTOR_DEG)
    metadata = make_metadata(make_arm_motors("left"))

    result = adapter.observation_to_kinematic_deg("left", observation, metadata)

    assert isinstance(result, np.ndarray)
    assert result.shape == (6,)
    assert result == pytest.approx([LEFT_MOTOR_DEG[joint] for joint in JOINT_ORDER], abs=1e-9)


# ======================================================================
# Behavior T-B2 (P1-B): inverse conversion kinematic_deg_to_action
#
#   kinematic_deg_to_action(side, q_deg, metadata) -> dict[str, float]
#
# returns EXACTLY six keys `arm_{side}_{joint}.pos` (one per JOINT_ORDER
# joint; no gripper, no base wheel, no lift, no other-arm key) whose values
# are the exact inverse of the T-B1 forward map, per the design §8 pair
#
#   q_urdf    = sign * motor_deg + zero
#   motor_deg = sign * (q_urdf - zero)          (sign ∈ {−1, +1})
#
# with the same per-side injected sign/zero as observation_to_kinematic_deg.
# The `.pos` suffix is a hard contract: AlohaMini.send_action selects arm
# keys with the literal predicate
#
#   k.endswith(".pos") and k.startswith("arm_{side}_")
#     and k.replace(".pos", "") in self.{side}_bus.motors
#
# so a key without the suffix is silently dropped and the joint command is
# lost. The metadata motors dict may contain interference entries (other
# arm, grippers, base wheels, lift) without affecting the result, and the
# input q_deg / metadata must not be modified.
#
# Cases:
# - T-B2.1 left main case: frozen q_deg -> frozen motor-degree literals,
#   exactly six `arm_left_*.pos` keys, q_deg and metadata unmodified.
#   Catches the identity implementation, the forward formula used as the
#   inverse, a negated zero, a swapped sign, suffix-less bare-name keys,
#   and any gripper/base/other-arm key leak.
# - T-B2.2 right main case: per-side injection with a q vector different
#   from the left one and BOTH arms present in metadata. Catches reading
#   the other arm and applying the left-side sign/zero to a right request.
# - T-B2.3 round trip (both sides): forward (T-B1, frozen intermediates)
#   then inverse returns the original motor degrees. Catches an inverse
#   that does not invert THIS adapter's forward (e.g. reuse of the forward
#   formula, or a different sign/zero convention between directions).
# - T-B2.4 send_action predicate alignment: every produced key survives a
#   literal restatement of the production filter; teeth checks prove the
#   restatement drops suffix-less bare names, wrong-side keys and motors-
#   foreign keys; a textual pin asserts the restated predicate still
#   matches AlohaMini.send_action's source (a production filter change
#   must re-review this alignment, not silently desynchronize it).
# - T-B2.5 wrong length q_deg (5 and 7) raises ValueError; the length-5
#   error names the missing joint (wrist_roll, per the JOINT_ORDER
#   positional contract). Catches silent truncation/broadcast
#   misalignment and zero-fill of missing joints.
# - T-B2.6 non-finite q_deg (inf at shoulder_pan, nan at wrist_flex)
#   raises ValueError naming the offending joint. Catches silently
#   propagating nan/inf into motor commands.
# - T-B2.7 invalid side raises ValueError naming the side. Catches a
#   KeyError crash, a silent empty dict, or a wrong default side.
#
# All expected values below are hand-derived literals from the formula
# motor_deg = sign * (q_deg - zero) with the same KINEMATIC_SIGNS /
# KINEMATIC_ZEROS injected above; no runtime computation, no tolerance
# beyond pytest.approx(abs=1e-9).
# ======================================================================

# Frozen kinematic q_deg inputs for the inverse cases (asymmetric, all
# values distinct, deliberately NOT the T-B1 forward outputs so the inverse
# expectations cannot be satisfied by echoing T-B1 constants).
LEFT_Q_DEG = (5.5, -71.25, 33.5, 12.25, -44.0, 88.75)
RIGHT_Q_DEG = (-13.5, 40.25, -7.0, 155.5, -66.25, 9.5)

# Frozen expected motor degrees per JOINT_ORDER, hand-derived as
# sign * (q_deg - zero) with the injected constants:
#   left:  (+1*(5.5-17.5), -1*(-71.25+43.25), 33.5-91.75, -1*(12.25+8.5),
#          -44.0-63.0, -1*(88.75+120.5))
#   right: (-1*(-13.5+71.25), 40.25-55.5, -1*(-7.0+12.0), 155.5-144.5,
#          -1*(-66.25+97.75), 9.5-31.0)
LEFT_EXPECTED_MOTOR_DEG = (-12.0, 28.0, -58.25, -20.75, -107.0, -209.25)
RIGHT_EXPECTED_MOTOR_DEG = (-57.75, -15.25, -5.0, 11.0, -31.5, -21.5)

# Literal arm-key predicates from AlohaMini.send_action (whitespace-
# normalized), pinned in T-B2.4 so the restatement cannot silently drift
# from the production filter.
_LEFT_PREDICATE_SNIPPET = (
    'k.endswith(".pos") and k.startswith("arm_left_")'
    ' and k.replace(".pos", "") in self.left_bus.motors'
)
_RIGHT_PREDICATE_SNIPPET = (
    'k.endswith(".pos") and k.startswith("arm_right_")'
    " and self.right_bus is not None"
    ' and k.replace(".pos", "") in self.right_bus.motors'
)


def restated_send_action_arm_filter(action: dict, side: str, metadata: dict) -> dict[str, float]:
    """Literal restatement of AlohaMini.send_action's arm-key comprehension.

    Mirrors the production filter (alohamini.py send_action):

        left_pos  = {k: v for k, v in action.items()
                     if k.endswith(".pos") and k.startswith("arm_left_")
                     and k.replace(".pos", "") in self.left_bus.motors}
        right_pos = {k: v for k, v in action.items()
                     if k.endswith(".pos") and k.startswith("arm_right_")
                     and self.right_bus is not None
                     and k.replace(".pos", "") in self.right_bus.motors}

    with `self.{side}_bus.motors` replaced by the metadata motors dict,
    whose keys are the same bare motor names. The production
    `self.right_bus is not None` guard is a runtime robot-state check, not
    a key predicate, and is intentionally not reproduced. T-B2.4 pins this
    restatement to the live production source textually.
    """
    motors = metadata["motors"]
    if side == "left":
        return {
            k: v
            for k, v in action.items()
            if k.endswith(".pos") and k.startswith("arm_left_") and k.replace(".pos", "") in motors
        }
    return {
        k: v
        for k, v in action.items()
        if k.endswith(".pos") and k.startswith("arm_right_") and k.replace(".pos", "") in motors
    }


def test_t_b2_left_inverse_conversion_six_pos_keys_frozen_values() -> None:
    """T-B2.1：left 逆换算产出六个 .pos 键与冻结电机度。"""
    adapter = make_adapter()
    q_deg = np.array(LEFT_Q_DEG, dtype=np.float64)
    q_deg_before = q_deg.copy()
    metadata = make_metadata(
        {
            **make_arm_motors("left"),
            **make_arm_motors("right"),
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )
    metadata_before = copy.deepcopy(metadata)

    action = adapter.kinematic_deg_to_action("left", q_deg, metadata)

    assert isinstance(action, dict)
    assert set(action) == {f"arm_left_{joint}.pos" for joint in JOINT_ORDER}
    for joint, expected in zip(JOINT_ORDER, LEFT_EXPECTED_MOTOR_DEG, strict=False):
        assert action[f"arm_left_{joint}.pos"] == pytest.approx(expected, abs=1e-9)
    assert np.array_equal(q_deg, q_deg_before)
    assert metadata == metadata_before


def test_t_b2_right_inverse_conversion_uses_right_side_injection() -> None:
    """T-B2.2：right 逆换算使用右侧注入的 sign/zero。"""
    adapter = make_adapter()
    q_deg = np.array(RIGHT_Q_DEG, dtype=np.float64)
    metadata = make_metadata(
        {
            **make_arm_motors("left"),
            **make_arm_motors("right"),
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )

    action = adapter.kinematic_deg_to_action("right", q_deg, metadata)

    assert isinstance(action, dict)
    assert set(action) == {f"arm_right_{joint}.pos" for joint in JOINT_ORDER}
    for joint, expected in zip(JOINT_ORDER, RIGHT_EXPECTED_MOTOR_DEG, strict=False):
        assert action[f"arm_right_{joint}.pos"] == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b2_round_trip_through_t_b1_forward_returns_original_motor_deg(
    side: str,
) -> None:
    """T-B2.3：经 T-B1 正向再逆换算回到原电机度（双侧）。"""
    adapter = make_adapter()
    motor_deg = LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG
    expected_forward = (
        LEFT_EXPECTED_KINEMATIC_DEG if side == "left" else RIGHT_EXPECTED_KINEMATIC_DEG
    )
    observation = make_arm_observation(side, motor_deg)
    metadata = make_metadata(make_arm_motors(side))

    kinematic_deg = adapter.observation_to_kinematic_deg(side, observation, metadata)
    assert kinematic_deg.shape == (6,)
    assert kinematic_deg == pytest.approx(list(expected_forward), abs=1e-9)

    action = adapter.kinematic_deg_to_action(side, kinematic_deg, metadata)

    assert set(action) == {f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER}
    for joint in JOINT_ORDER:
        assert action[f"arm_{side}_{joint}.pos"] == pytest.approx(motor_deg[joint], abs=1e-9)


@pytest.mark.parametrize("bad_length", [5, 7])
def test_t_b2_wrong_length_q_deg_raises_without_partial_dict(bad_length: int) -> None:
    """T-B2.5：q_deg 长度错误时报错且不返回部分 dict。"""
    adapter = make_adapter()
    q_deg = np.full(bad_length, 3.25, dtype=np.float64)
    metadata = make_metadata(make_arm_motors("left"))

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("left", q_deg, metadata)

    if bad_length == 5:
        # A short vector is missing the last JOINT_ORDER joint's value; the
        # error must name it (design §19: errors include side/joint/reason).
        assert "wrist_roll" in str(excinfo.value)
    # bad_length == 7 has no identifiable missing joint; the raise itself is
    # the contract (no silent truncation, no zero-fill, no partial dict).


@pytest.mark.parametrize(
    ["bad_index", "bad_value"],
    [
        (0, float("inf")),
        (3, float("nan")),
    ],
)
def test_t_b2_non_finite_q_deg_raises_with_joint_name(bad_index: int, bad_value: float) -> None:
    """T-B2.6：q_deg 含 inf/nan 时报错并点名关节。"""
    adapter = make_adapter()
    values = [1.5, -2.5, 3.75, -4.25, 5.5, -6.75]
    values[bad_index] = bad_value
    q_deg = np.array(values, dtype=np.float64)
    metadata = make_metadata(make_arm_motors("left"))

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("left", q_deg, metadata)

    assert JOINT_ORDER[bad_index] in str(excinfo.value)


def test_t_b2_invalid_side_raises_with_side_name() -> None:
    """T-B2.7：side 非法时报错并点名该值。"""
    adapter = make_adapter()
    q_deg = np.array(LEFT_Q_DEG, dtype=np.float64)
    metadata = make_metadata(make_arm_motors("left"))

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("center", q_deg, metadata)

    assert "center" in str(excinfo.value)


# ======================================================================
# Behavior T-B3 (P1-B 单位适配, design §8): DEFAULT RANGE_M100_100 full
# loop round trip, with the production MotorsBus semantics as oracle.
#
#   q_deg --kinematic_deg_to_action--> action values (normalized units)
#        --FeetechMotorsBus._unnormalize--> raw ticks (int)
#        --FeetechMotorsBus._normalize----> normalized observation values
#        --observation_to_kinematic_deg---> ≈ original q_deg
#
# with per-joint tolerance tol = 360 / (resolution - 1) / 2 + eps (half a
# tick in degrees, derived from the metadata `resolution` field; eps covers
# float residue only). This behavior exercises the unit seam the DEGREES
# cases (T-B1/T-B2) deliberately do not: with metadata normalization
# "range_m100_100" the adapter must convert motor degrees <-> normalized
# values through the metadata fields (resolution / range_min / range_max /
# drive_mode), because the bus no longer speaks degrees. The synthetic
# motor-degree value m behind each q_deg is q_deg = sign * m + zero with the
# SAME injected KINEMATIC_SIGNS / KINEMATIC_ZEROS as T-B1/T-B2.
#
# Oracle wiring (single source of truth): T_B3_SPECS below feeds BOTH the
# metadata envelope AND the synthetic FeetechMotorsBus. The bus is created
# with object.__new__(FeetechMotorsBus), bypassing __init__ (no SDK, no
# port, no serial), then injected with:
#   .motors            dict[str, Motor]  (norm_mode from the spec string,
#                                          exactly what build_robot_metadata
#                                          publishes as `normalization`)
#   .calibration       dict[str, MotorCalibration]
#   ._id_to_model_dict / ._id_to_name_dict  (normally built in __init__)
# apply_drive_mode (True) and model_resolution_table are class attributes
# and need no injection; the RANGE_M100_100 branches never consult the
# resolution table, so the synthetic model names are safe.
#
# Sample design (why these literals): range_max - range_min = resolution-1
# is a power of two, which makes the bus' deg<->tick chain BITWISE EXACT for
# integer ticks, so _unnormalize's int() truncation lands exactly on the
# quantized tick. Two sample classes per side:
#   A-class: motor_deg = -mid*360/W exactly -> t_cont == 0.0 -> t* = 0, the
#     only truncation point safe under ANY float perturbation of n. These
#     prove the correct implementation round-trips at 0 error.
#   B-class: t_cont = t* - 0.0625 (t* >= 1) -> the identity-truncation
#     fraction is 0.9375, so a conversion-correct-but-unquantized
#     implementation deterministically exceeds tolerance by 1.875x, while
#     the correct (round-quantized) implementation errs only 0.125x tol.
# All |motor_deg| > 100, so the CURRENT identity implementation (which
# treats normalized values as motor degrees and skips the unit conversion)
# trips the bus' [-100, 100] clip and errs by 30-50 degrees -- far beyond
# tolerance on every joint of both sides.
#
# What this binding catches: (a) missing normalized<->degree conversion
# (identity implementation), (b) conversion without round-quantization to
# whole ticks (B-class joints), (c) single-sided drive_mode handling errors
# (drive_mode=1 joints: a missing negation on exactly one side shifts the
# round trip by hundreds of degrees), (d) outputs outside the normalized
# domain (the bus can only consume [-100, 100]). Known non-goals of a pure
# round trip (both are self-healing inside the loop and therefore NOT
# discriminated here): a consistently-wrong-but-symmetric conversion
# (res instead of res-1, or drive_mode applied/negelected on BOTH sides) —
# those need frozen intermediate-value assertions, not a round trip.
#
# SYNTHETIC FIXTURE NOTICE: every id / model / calibration / resolution
# value below is SYNTHETIC, ASYMMETRIC and NOT USABLE ON REAL HARDWARE.
# Model names ("syn-enc-d*" / "syn-enc-e*") and resolutions (4097, 8193,
# 2049, 16385, 1025, 65537) deliberately avoid the real Feetech
# MODEL_RESOLUTION table (4096 / 1024). range_min values are negative by
# design (synthetic calibration window) and must never be copied to real
# hardware.
# ======================================================================

T_B3_EPS = 1e-9

# Per-side synthetic specs for the six kinematic joints, all in the DEFAULT
# range_m100_100 normalization mode, drive_mode mixing 0/1 on both sides.
# range_max - range_min == resolution - 1 == 2**k (see sample design above).
T_B3_SPECS = {
    "left": {
        "shoulder_pan": {
            "id": 1,
            "model": "syn-enc-d1",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -3548,
            "range_max": 548,
            "homing_offset": 333,
            "resolution": 4097,
        },
        "shoulder_lift": {
            "id": 2,
            "model": "syn-enc-d2",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -96,
            "range_max": 8096,
            "homing_offset": -1717,
            "resolution": 8193,
        },
        "elbow_flex": {
            "id": 3,
            "model": "syn-enc-d3",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -244,
            "range_max": 1804,
            "homing_offset": 907,
            "resolution": 2049,
        },
        "wrist_flex": {
            "id": 4,
            "model": "syn-enc-d4",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -1692,
            "range_max": 14692,
            "homing_offset": -555,
            "resolution": 16385,
        },
        "wrist_yaw": {
            "id": 5,
            "model": "syn-enc-d5",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -12,
            "range_max": 1012,
            "homing_offset": 99,
            "resolution": 1025,
        },
        "wrist_roll": {
            "id": 6,
            "model": "syn-enc-d6",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -7768,
            "range_max": 57768,
            "homing_offset": -2049,
            "resolution": 65537,
        },
    },
    "right": {
        "shoulder_pan": {
            "id": 1,
            "model": "syn-enc-e1",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -996,
            "range_max": 7196,
            "homing_offset": -808,
            "resolution": 8193,
        },
        "shoulder_lift": {
            "id": 2,
            "model": "syn-enc-e2",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -3648,
            "range_max": 448,
            "homing_offset": 1234,
            "resolution": 4097,
        },
        "elbow_flex": {
            "id": 3,
            "model": "syn-enc-e3",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": 808,
            "range_max": 17192,
            "homing_offset": -66,
            "resolution": 16385,
        },
        "wrist_flex": {
            "id": 4,
            "model": "syn-enc-e4",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -174,
            "range_max": 1874,
            "homing_offset": 3011,
            "resolution": 2049,
        },
        "wrist_yaw": {
            "id": 5,
            "model": "syn-enc-e5",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -912,
            "range_max": 112,
            "homing_offset": -77,
            "resolution": 1025,
        },
        "wrist_roll": {
            "id": 6,
            "model": "syn-enc-e6",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -4768,
            "range_max": 60768,
            "homing_offset": 4144,
            "resolution": 65537,
        },
    },
}

# Frozen q_deg inputs, hand-derived as q = sign * motor_deg + zero from the
# same KINEMATIC_SIGNS / KINEMATIC_ZEROS injected for T-B1/T-B2. The backing
# synthetic motor degrees m per JOINT_ORDER are (A = truncation-safe class,
# B = unquantization-discriminating class):
#   left:  ( +131.8359375 A, -133.20098876953125 B, -137.109375 A,
#            -142.822265625 A, -133.26416015625 B, -137.3291015625 A )
#   right: ( -136.23046875 A, +140.625 A, -133.19961547851562 B,
#            -149.4140625 A, +140.625 A, +133.1978988647461 B )
# All are exact binary fractions, so the adapter's inverse formula
# sign * (q_deg - zero) recovers m bitwise.
T_B3_Q_DEG = {
    "left": (
        149.3359375,
        89.95098876953125,
        -45.359375,
        134.322265625,
        -70.26416015625,
        16.8291015625,
    ),
    "right": (
        64.98046875,
        196.125,
        121.19961547851562,
        -4.9140625,
        -238.375,
        164.1978988647461,
    ),
}


def make_t_b3_metadata() -> dict:
    """构造双臂 RANGE_M100_100 的 metadata v2 信封（含标准干扰条目）。

    Metadata v2 envelope: both arms in range_m100_100 mode plus the same
    interference entries as T-B1/T-B2 (grippers / base wheels / lift). The
    oracle bus and the metadata come from the SAME T_B3_SPECS (single source
    of truth for normalization / resolution / range / drive_mode).
    """
    return make_metadata(
        {
            **{f"arm_left_{joint}": dict(spec) for joint, spec in T_B3_SPECS["left"].items()},
            **{f"arm_right_{joint}": dict(spec) for joint, spec in T_B3_SPECS["right"].items()},
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )


# ======================================================================
# Behavior T-B4 (P1-B 单位适配, design §8): drive_mode 语义 + 中间值冻结断言
# —— 闭合 T-B3 全环路 round trip 声明的两个盲区（盲区 ⑤ res−1、
#    盲区 ⑥ drive_mode 双侧一致漏取反）。
#
# 1a. DEGREES 电机 drive_mode=1 与 =0 的换算完全相同。设计锚点（production
#     只读，已核实）：MotorsBus._normalize 的 DEGREES 分支
#     `(val - mid) * 360 / max_res` 与 _unnormalize 的 DEGREES 分支
#     `int(val * max_res / 360 + mid)` 都不读取 drive_mode（取反只存在于
#     RANGE 分支）。因此 adapter 的 DEGREES 路径对 drive_mode 必须完全不
#     敏感：同一观测 / 同一 q_deg 在全 0 与全 1 两种 drive_mode 元数据下
#     产出**完全相同**的结果，且等于 T-B1 / T-B2 已冻结的字面期望。若实现
#     对 DEGREES 也取反（把 RANGE 的 drive_mode 语义误套到 DEGREES），dm=1
#     元数据下每个关节值都会翻符号，冻结字面值与 dm 对不变性同时 RED。
#     另以 production bus 本体做 oracle 前提自检（同 T-B3 模式）：合成
#     FeetechMotorsBus 在 DEGREES 规格下 _normalize/_unnormalize 的输出对
#     drive_mode=0/1 完全相同，且等于按元数据 resolution 字段的纯度换算。
#     该前提若失效说明 production bus 语义漂移，需重审本对齐，而非 adapter
#     的 RED。
#
# 1b. RANGE_M100_100 同一 kinematic q 在 drive_mode=0/1 下产生**不同**的
#     normalized action 值（镜像真实 bus 的取反语义），且两种元数据下全环
#     路 round trip 均成立。逐关节断言 act(dm 翻转) == -act(dm 原值)
#     （abs=1e-9）：tick 与 drive_mode 无关，两侧共享同一量化 tick，取反
#     关系精确成立（含 ±100 边界对称截断）；obs 侧同理
#     obs(dm 翻转) == -obs(dm 原值)。
#
# 2a. res−1 盲区（动作方向 + 观测方向）：冻结中间值字面常数。
#     动作方向：给定 q_deg 与合成 resolution，adapter 的换算结果
#     normalized == 冻结字面常数，且 oracle bus `_unnormalize(action)` 的
#     raw tick == 冻结 int 字面常数。样本设计使真连续 tick **恰好**等于
#     t* + 0.25（远离 0.5 舍入边界，真 tick = t* 唯一确定）：若实现用
#     resolution 而非 resolution−1，连续 tick 偏移 c + 0.25 + 0.25/W 落在
#     [1.625, 1.9375]，round 必落 t*+2 —— 翻转 +2 tick（远超 1 tick，且
#     距边界 ≥0.125，不依赖脆弱舍入边界），normalized 偏移 400/span =
#     100/W ≥ 0.003，两个冻结断言必 RED。
#     观测方向：直接以冻结 normalized 字面常数构造 obs 输入（等价于 bus
#     在冻结 t2 = mid − (W+3+j) 处的回报），冻结 adapter 输出 q == 字面
#     常数。正确实现经 (tick−mid)·360/(resolution−1) 精确恢复 motor 度；
#     res 变异（误用 resolution）使输出偏移 (W+3+j)·360/(W(W+1)) ≥ 0.011
#     度，远超 abs=1e-9。
# 2b. drive_mode 双侧盲区：2a 的冻结 normalized 字面值同时冻结符号（dm=1
#     关节为负的具体字面值、dm=0 为正），观测方向冻结 q 字面值同理冻结
#     取反。双侧一致漏取反在闭环内自愈（T-B3 已声明不可判别），但中间值
#     断言直接暴露：dm=1 关节变异后发出 +|norm|（符号翻转，字面值必偏），
#     且 oracle bus 对未取反的 +|norm| 取反后恢复镜像 tick = 2·mid − t* ≠
#     t*（偏移 2c·W ≥ 2816 tick），两个冻结断言同时 RED；观测侧漏取反恢复
#     镜像 tick，q 偏移 ≈ 2·|m_deg2|（数百度），字面值必偏。
#
# 变异击杀矩阵（/tmp 变异实测，production 只读）：
#   res 替代 res−1（双侧一致）-> 2a 动作方向 tick 翻转 +2 / normalized 偏移
#       ≥0.003 + 2a 观测方向 q 偏移 ≥0.011；T-B3 round trip 自愈（盲区 ⑤
#       闭合证据）
#   dm 双侧一致漏取反       -> 2a 动作方向 normalized 符号翻转 + oracle
#       tick 镜像 + 2a 观测方向 q 偏移数百度 + 1b 翻转关系破坏；T-B3 round
#       trip 自愈（盲区 ⑥ 闭合证据）
#   DEGREES 路径误取反      -> 1a 双向冻结字面值 + dm 对不变性 RED
#
# 样本设计（为何是这些字面量）：span = range_max − range_min = 4·W，其中
# W = resolution − 1 = 2^k（k ≥ 10），使 bus 的 deg↔tick 链对整数 tick 逐
# 位精确（同 T-B3 的 2 的幂设计）。真连续 tick 恰为 t*+0.25，其中
# t* = mid + c·W，per-joint c ∈ {1.5, 1.375, 1.625, 1.4375, 1.5625,
# 1.6875} 互不相同；res 变异偏移 c+0.25+0.25/W 落在 [1.625, 1.9375]，round
# 翻转 +2 tick 的余量 ≥0.125。per-joint 互异的 normalized 字面值
# （±75.0/±68.75/±81.25/±71.875/±78.125/±84.375）使恒定输出类变异也被逐
# 关节断言捕获。dm=1 关节的 normalized 字面值为负、dm=0 为正 —— 符号即
# 2b 的冻结内容。所有中间值均为精确二进制分数，abs=1e-9 断言安全。
#
# SYNTHETIC FIXTURE NOTICE: every id / model / calibration / resolution /
# range / motor-degree value below is SYNTHETIC, ASYMMETRIC and NOT USABLE
# ON REAL HARDWARE. Model names ("syn-enc-f*" / "syn-enc-g*") avoid the real
# Feetech MODEL_RESOLUTION table (4096 / 1024). The synthetic calibration
# windows are wider than 360 degrees by design (span = 4·(resolution−1)) and
# must never be copied to real hardware. Signs / zeros are the SAME injected
# KINEMATIC_SIGNS / KINEMATIC_ZEROS as T-B1/T-B2/T-B3.
# ======================================================================

# Per-side synthetic specs for the six kinematic joints, all in the DEFAULT
# range_m100_100 normalization mode, drive_mode mixing 0/1 on both sides.
# span = range_max - range_min = 4 * (resolution - 1) = 2^(k+2): the bus'
# normalized<->tick chain stays bitwise exact for integer ticks (same
# power-of-two design as T-B3).
T_B4_SPECS = {
    "left": {
        "shoulder_pan": {
            "id": 1,
            "model": "syn-enc-f1",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -3500,
            "range_max": 596,
            "homing_offset": 431,
            "resolution": 1025,
        },
        "shoulder_lift": {
            "id": 2,
            "model": "syn-enc-f2",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -720,
            "range_max": 7472,
            "homing_offset": -1613,
            "resolution": 2049,
        },
        "elbow_flex": {
            "id": 3,
            "model": "syn-enc-f3",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -900,
            "range_max": 15484,
            "homing_offset": 1019,
            "resolution": 4097,
        },
        "wrist_flex": {
            "id": 4,
            "model": "syn-enc-f4",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -40,
            "range_max": 32728,
            "homing_offset": -659,
            "resolution": 8193,
        },
        "wrist_yaw": {
            "id": 5,
            "model": "syn-enc-f5",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -6100,
            "range_max": 59436,
            "homing_offset": 173,
            "resolution": 16385,
        },
        "wrist_roll": {
            "id": 6,
            "model": "syn-enc-f6",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -260,
            "range_max": 130812,
            "homing_offset": -2153,
            "resolution": 32769,
        },
    },
    "right": {
        "shoulder_pan": {
            "id": 1,
            "model": "syn-enc-g1",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -180,
            "range_max": 3916,
            "homing_offset": -919,
            "resolution": 1025,
        },
        "shoulder_lift": {
            "id": 2,
            "model": "syn-enc-g2",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -1400,
            "range_max": 6792,
            "homing_offset": 1337,
            "resolution": 2049,
        },
        "elbow_flex": {
            "id": 3,
            "model": "syn-enc-g3",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -55,
            "range_max": 16329,
            "homing_offset": -71,
            "resolution": 4097,
        },
        "wrist_flex": {
            "id": 4,
            "model": "syn-enc-g4",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -3000,
            "range_max": 29768,
            "homing_offset": 3113,
            "resolution": 8193,
        },
        "wrist_yaw": {
            "id": 5,
            "model": "syn-enc-g5",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": -95,
            "range_max": 65441,
            "homing_offset": -181,
            "resolution": 16385,
        },
        "wrist_roll": {
            "id": 6,
            "model": "syn-enc-g6",
            "normalization": "range_m100_100",
            "drive_mode": 1,
            "range_min": -7000,
            "range_max": 124072,
            "homing_offset": 4243,
            "resolution": 32769,
        },
    },
}

# Frozen q_deg inputs, hand-derived as q = sign * m + zero with the SAME
# injected KINEMATIC_SIGNS / KINEMATIC_ZEROS as T-B1/T-B2/T-B3. The backing
# synthetic motor degrees m per joint are exact binary fractions chosen so
# the TRUE continuous tick is EXACTLY t* + 0.25 with t* = mid + c*W
# (per-joint c: 1.5, 1.375, 1.625, 1.4375, 1.5625, 1.6875), i.e.
# m = c*360 + 90/W:
#   left  m: (540.087890625, 495.0439453125, 585.02197265625,
#             517.510986328125, 562.5054931640625, 607.5027465820312)
#   right m: (same six values; ranges/zeros differ per side)
# All values below are exact float literals; float round-trip is bitwise.
T_B4_Q_DEG = {
    "left": (
        557.587890625,  # +1.0 * 540.087890625 + 17.5
        -538.2939453125,  # -1.0 * 495.0439453125 - 43.25
        676.77197265625,  # +1.0 * 585.02197265625 + 91.75
        -526.010986328125,  # -1.0 * 517.510986328125 - 8.5
        625.5054931640625,  # +1.0 * 562.5054931640625 + 63.0
        -728.0027465820312,  # -1.0 * 607.5027465820312 - 120.5
    ),
    "right": (
        -611.337890625,  # -1.0 * 540.087890625 - 71.25
        550.5439453125,  # +1.0 * 495.0439453125 + 55.5
        -597.02197265625,  # -1.0 * 585.02197265625 - 12.0
        662.010986328125,  # +1.0 * 517.510986328125 + 144.5
        -660.2554931640625,  # -1.0 * 562.5054931640625 - 97.75
        638.5027465820312,  # +1.0 * 607.5027465820312 + 31.0
    ),
}

# Frozen action intermediates (2b sign freeze included): the adapter's
# normalized output per joint, and the oracle bus' raw tick for the produced
# action. Hand-derived (W = resolution - 1, mid = (range_min+range_max)/2,
# t* = mid + c*W, cont = t* + 0.25 exactly):
#   unnegated norm = ((t* - range_min)/span)*200 - 100 = 50*(2 + c) - 100
#                  = 50*c + ... -> per-joint c gives
#   norm(dm=0) = +{75.0, 68.75, 81.25, 71.875, 78.125, 84.375};
#   norm(dm=1) = -norm(dm=0)  (negated, i.e. NEGATIVE frozen literal).
# Oracle tick = t* (int literals below).
T_B4_EXPECTED_ACTION = {
    "left": {
        "shoulder_pan": -75.0,  # dm=1: -(50*1.5)
        "shoulder_lift": 68.75,  # dm=0: 50*1.375
        "elbow_flex": -81.25,  # dm=1: -(50*1.625)
        "wrist_flex": 71.875,  # dm=0: 50*1.4375
        "wrist_yaw": -78.125,  # dm=1: -(50*1.5625)
        "wrist_roll": 84.375,  # dm=0: 50*1.6875
    },
    "right": {
        "shoulder_pan": 75.0,  # dm=0
        "shoulder_lift": -68.75,  # dm=1
        "elbow_flex": 81.25,  # dm=0
        "wrist_flex": -71.875,  # dm=1
        "wrist_yaw": 78.125,  # dm=0
        "wrist_roll": -84.375,  # dm=1
    },
}
T_B4_EXPECTED_BUS_TICK = {
    "left": {
        "shoulder_pan": 84,  # mid(-1452) + 1.5*1024
        "shoulder_lift": 6192,  # mid(3376) + 1.375*2048
        "elbow_flex": 13948,  # mid(7292) + 1.625*4096
        "wrist_flex": 28120,  # mid(16344) + 1.4375*8192
        "wrist_yaw": 52268,  # mid(26668) + 1.5625*16384
        "wrist_roll": 120572,  # mid(65276) + 1.6875*32768
    },
    "right": {
        "shoulder_pan": 3404,  # mid(1868) + 1.5*1024
        "shoulder_lift": 5512,  # mid(2696) + 1.375*2048
        "elbow_flex": 14793,  # mid(8137) + 1.625*4096
        "wrist_flex": 25160,  # mid(13384) + 1.4375*8192
        "wrist_yaw": 58273,  # mid(32673) + 1.5625*16384
        "wrist_roll": 113832,  # mid(58536) + 1.6875*32768
    },
}

# Frozen observation-direction intermediates (closes the ⑤/⑥ obs side):
# directly constructed obs inputs — the exact value the bus would report at
# the frozen t2 = mid - (W + 3 + j) tick — and the frozen kinematic-degree
# expectation for that input:
#   g(t2) = ((t2 - range_min)/span)*200 - 100 = -50 - 50*(3+j)/W;
#   obs input = -g(t2) if dm==1 else g(t2);
#   expected q = sign * (t2 - mid) * 360 / W + zero.
T_B4_OBS_INPUT = {
    "left": {
        "shoulder_pan": 50.146484375,  # dm=1: -g(t2=-2479)
        "shoulder_lift": -50.09765625,  # dm=0: g(t2=1324)
        "elbow_flex": 50.06103515625,  # dm=1: -g(t2=3191)
        "wrist_flex": -50.03662109375,  # dm=0: g(t2=8146)
        "wrist_yaw": 50.0213623046875,  # dm=1: -g(t2=10277)
        "wrist_roll": -50.01220703125,  # dm=0: g(t2=32500)
    },
    "right": {
        "shoulder_pan": -50.146484375,  # dm=0: g(t2=841)
        "shoulder_lift": 50.09765625,  # dm=1: -g(t2=644)
        "elbow_flex": -50.06103515625,  # dm=0: g(t2=4036)
        "wrist_flex": 50.03662109375,  # dm=1: -g(t2=5186)
        "wrist_yaw": -50.0213623046875,  # dm=0: g(t2=16282)
        "wrist_roll": 50.01220703125,  # dm=1: -g(t2=25760)
    },
}
T_B4_EXPECTED_OBS_Q = {
    "left": (
        -343.5546875,  # +1.0*(-1027*360/1024) + 17.5
        317.453125,  # -1.0*(-2052*360/2048) - 43.25
        -268.689453125,  # +1.0*(-4101*360/4096) + 91.75
        351.763671875,  # -1.0*(-8198*360/8192) - 8.5
        -297.15380859375,  # +1.0*(-16391*360/16384) + 63.0
        239.587890625,  # -1.0*(-32776*360/32768) - 120.5
    ),
    "right": (
        289.8046875,  # -1.0*(-1027*360/1024) - 71.25
        -305.203125,  # +1.0*(-2052*360/2048) + 55.5
        348.439453125,  # -1.0*(-4101*360/4096) - 12.0
        -215.763671875,  # +1.0*(-8198*360/8192) + 144.5
        262.40380859375,  # -1.0*(-16391*360/16384) - 97.75
        -329.087890625,  # +1.0*(-32776*360/32768) + 31.0
    ),
}


def make_t_b4_metadata() -> dict:
    """从 T_B4_SPECS 构造双臂 metadata v2 信封（含标准干扰条目）。

    Metadata v2 envelope: both arms from T_B4_SPECS plus the same
    interference entries as T-B1/T-B2/T-B3. The oracle bus and the metadata
    come from the SAME T_B4_SPECS (single source of truth).
    """
    return make_metadata(
        {
            **{f"arm_left_{joint}": dict(spec) for joint, spec in T_B4_SPECS["left"].items()},
            **{f"arm_right_{joint}": dict(spec) for joint, spec in T_B4_SPECS["right"].items()},
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b4_2a_range_obs_direction_intermediates_frozen_literals(side: str) -> None:
    """T-B4.2a（观测方向）：冻结 normalized 观测字面值必须产出冻结 q 字面值。

    T-B4.2a (observation direction, closes the ⑤ obs-side + ⑥ obs-side
    blind spots): directly constructed obs inputs — frozen normalized
    literals, the values the bus would report at the frozen t2 = mid −
    (W+3+j) ticks — produce frozen kinematic-degree literals.

    The correct obs-side conversion recovers motor degrees through
    (tick - mid) * 360 / (resolution - 1). An implementation using
    `resolution` instead of `resolution - 1` shifts every joint's output by
    (W+3+j)*360/(W*(W+1)) >= 0.011 degrees (>> 1e-9). A missing drive_mode
    negation on the obs side recovers the mirror tick 2*mid - t2 for dm=1
    joints, shifting the output by ~2*|(t2-mid)*360/W| (hundreds of degrees).
    T-B3's closed round trip self-heals both; this direct-input freeze does
    not.
    """
    observation = {f"arm_{side}_{joint}.pos": T_B4_OBS_INPUT[side][joint] for joint in JOINT_ORDER}
    metadata = make_t_b4_metadata()

    adapter = make_adapter()
    result = adapter.observation_to_kinematic_deg(side, observation, metadata)

    assert isinstance(result, np.ndarray)
    assert result.shape == (6,)
    for index, joint in enumerate(JOINT_ORDER):
        expected = T_B4_EXPECTED_OBS_Q[side][index]
        assert result[index] == pytest.approx(expected, abs=1e-9), (
            f"frozen obs-direction mismatch for joint {joint!r} on side {side!r}: "
            f"{result[index]!r} != {expected!r}"
        )


# ======================================================================
# Behavior T-B5 (P1-B, design §8): homing_offset is NEVER re-applied.
#
# In Feetech v0 the homing offset is handled by the FIRMWARE: calibration
# writes it into the device's Homing_Offset register (see
# FeetechMotorsBus.enable_torque / write("Homing_Offset", ...)), and the
# device reports already-offset positions. Neither MotorsBus._normalize nor
# MotorsBus._unnormalize references homing_offset anywhere, and neither does
# the adapter. Consequence for the adapter: changing ONLY the metadata
# `homing_offset` field (leaving obs / range_min / range_max / resolution /
# drive_mode / id / model / normalization untouched) must not change ANY
# conversion output — bitwise — in every mode, both directions, both sides:
#
#   metadata(homing=X) -> forward / inverse outputs   ==   metadata(homing=Y)
#
# A re-application (adding or subtracting the offset — tick-valued, in
# degrees, or inside the normalized domain) in ANY of the four paths shifts
# the variant outputs away from the originals, failing the bitwise-equality
# assertions below AND the frozen T-B1/T-B2/T-B4 literals.
#
# Pair construction: HOMING_VARIANT_VALUES cycles 0 / -777 / +1234 / -5 /
# 888 / 1 per joint. The set includes the zero boundary and negative and
# positive offsets, and every value differs from the corresponding original
# homing_offset in LEFT_MOTOR_SPECS / RIGHT_MOTOR_SPECS / T_B4_SPECS, so the
# pair never degenerates. All other fields are copied verbatim; the pair
# differs in homing_offset and NOTHING else.
#
# Equality discipline: the adapter is a deterministic pure function, and a
# correct implementation never even READS homing_offset, so both pair runs
# execute the identical code path on identical inputs — outputs must be
# BITWISE identical (np.array_equal / dict ==). pytest.approx is used only
# for the frozen-literal cross-checks, with the same 1e-9 tolerance as
# T-B1/T-B2/T-B4.
#
# SYNTHETIC FIXTURE NOTICE: every id / model / calibration / resolution /
# homing value below is SYNTHETIC, ASYMMETRIC and NOT USABLE ON REAL
# HARDWARE. Real sign/zero/offset/limits require Gate A/B evidence and must
# never be copied from this file.
# ======================================================================

# Per-joint replacement homing offsets (index = position in JOINT_ORDER).
# 0 exercises the zero boundary; -777 / +1234 are the task-named values;
# -5 / 888 / 1 add further sign/magnitude spread. Verified non-colliding
# with every original homing_offset in this file's spec tables.
HOMING_VARIANT_VALUES = (0, -777, 1234, -5, 888, 1)


def _homing_variant_specs(specs: dict[str, dict]) -> dict[str, dict]:
    """配对 fixture：只替换 homing_offset 的同一份 per-joint specs。

    Pair fixture: the SAME per-joint specs with ONLY homing_offset
    replaced (per-joint cycle from HOMING_VARIANT_VALUES). Every other field
    (id / model / normalization / drive_mode / range_min / range_max /
    resolution) is copied verbatim.
    """
    return {
        joint: {**entry, "homing_offset": HOMING_VARIANT_VALUES[index]}
        for index, (joint, entry) in enumerate(specs.items())
    }


def _make_arm_metadata(side: str, specs: dict[str, dict]) -> dict:
    """按给定 per-joint specs 构造单臂 metadata 信封（含标准干扰条目）。

    Metadata envelope for one arm from the GIVEN per-joint specs plus the
    standard interference entries (same envelope as make_t_b4_metadata).
    """
    return make_metadata(
        {
            **{f"arm_{side}_{joint}": dict(spec) for joint, spec in specs.items()},
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b5_1_degrees_homing_offset_variant_bitwise_identical(side: str) -> None:
    """T-B5.1：DEGREES 双方向换算对 homing_offset 变体逐位不变。

    T-B5.1: DEGREES-mode forward AND inverse conversions are bitwise
    identical under a homing-offset-only metadata variant.

    Pair: LEFT/RIGHT_MOTOR_SPECS (all homing_offsets non-zero) vs the same
    specs with homing_offset cycled through HOMING_VARIANT_VALUES. The same
    frozen motor-degree observation and the same frozen q_deg are converted
    with both metadata versions. The variant outputs must equal the frozen
    T-B1 / T-B2 literals (so the ORIGINAL run is anchored to hand-computed
    values, not just self-consistent) and must be BITWISE identical to the
    original-run outputs. Catches adding/subtracting the homing_offset in the
    DEGREES branch of either direction (the offset is tick-valued; applying
    it as degrees, ticks, or normalized units all shift the output).
    """
    adapter = make_adapter()
    motor_deg = LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG
    expected_forward = (
        LEFT_EXPECTED_KINEMATIC_DEG if side == "left" else RIGHT_EXPECTED_KINEMATIC_DEG
    )
    q_deg = LEFT_Q_DEG if side == "left" else RIGHT_Q_DEG
    expected_inverse = LEFT_EXPECTED_MOTOR_DEG if side == "left" else RIGHT_EXPECTED_MOTOR_DEG
    observation = make_arm_observation(side, motor_deg)
    # Joint-name-keyed specs (LEFT/RIGHT_MOTOR_SPECS); _make_arm_metadata
    # applies the arm_{side}_{joint} prefix itself.
    specs = LEFT_MOTOR_SPECS if side == "left" else RIGHT_MOTOR_SPECS
    metadata_orig = _make_arm_metadata(side, specs)
    metadata_variant = _make_arm_metadata(side, _homing_variant_specs(specs))
    # Pair premise: the variant really differs per joint (no degenerate pair).
    for index, joint in enumerate(JOINT_ORDER):
        assert specs[joint]["homing_offset"] != HOMING_VARIANT_VALUES[index]
    q = np.array(q_deg, dtype=np.float64)
    q_before = q.copy()

    out_orig = adapter.observation_to_kinematic_deg(side, observation, metadata_orig)
    out_variant = adapter.observation_to_kinematic_deg(side, observation, metadata_variant)
    act_orig = adapter.kinematic_deg_to_action(side, q, metadata_orig)
    act_variant = adapter.kinematic_deg_to_action(side, q, metadata_variant)

    assert np.array_equal(q, q_before)
    assert out_orig.shape == (6,) and out_variant.shape == (6,)
    # Variant outputs equal the frozen literals (T-B1 forward / T-B2 inverse).
    assert out_variant == pytest.approx(list(expected_forward), abs=1e-9)
    keys = [f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER]
    assert set(act_variant) == set(keys)
    for key, expected in zip(keys, expected_inverse, strict=False):
        assert act_variant[key] == pytest.approx(expected, abs=1e-9)
    # And the pair is BITWISE identical: homing_offset has zero influence.
    assert np.array_equal(out_variant, out_orig), (
        "DEGREES forward output changed when only metadata homing_offset changed: "
        f"variant={out_variant!r} vs original={out_orig!r} (side={side!r})"
    )
    assert act_variant == act_orig, (
        "DEGREES inverse output changed when only metadata homing_offset changed: "
        f"variant={act_variant!r} vs original={act_orig!r} (side={side!r})"
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b5_2_range_m100_100_homing_offset_variant_bitwise_identical(side: str) -> None:
    """T-B5.2：RANGE_M100_100 双方向换算对 homing_offset 变体逐位不变。

    T-B5.2: RANGE_M100_100-mode forward AND inverse conversions are
    bitwise identical under a homing-offset-only metadata variant.

    Pair: T_B4_SPECS (all homing_offsets non-zero, drive_mode mixing 0/1)
    vs the homing-cycled variant. Inputs are the frozen T-B4 normalized
    observation literals (the values the bus reports at the frozen t2 ticks)
    and the frozen T_B4_Q_DEG. The variant outputs must equal the frozen
    T-B4 literals (T-B4.2a obs-direction expectations / T-B4.2a action
    expectations) and must be BITWISE identical to the original-run outputs.
    Catches a homing re-application inside the tick recovery
    (motor_deg += homing * 360 / (resolution - 1)), inside the normalized
    value, or inside the action-side tick quantization.
    """
    adapter = make_adapter()
    specs = T_B4_SPECS[side]
    variant_specs = _homing_variant_specs(specs)
    metadata_orig = _make_arm_metadata(side, specs)
    metadata_variant = _make_arm_metadata(side, variant_specs)
    # Pair premise: homing_offset is the ONLY differing field, and it really
    # differs per joint.
    for index, joint in enumerate(JOINT_ORDER):
        assert specs[joint]["homing_offset"] != HOMING_VARIANT_VALUES[index]
        assert variant_specs[joint] == {
            **specs[joint],
            "homing_offset": HOMING_VARIANT_VALUES[index],
        }
    observation = {f"arm_{side}_{joint}.pos": T_B4_OBS_INPUT[side][joint] for joint in JOINT_ORDER}
    q = np.array(T_B4_Q_DEG[side], dtype=np.float64)
    q_before = q.copy()

    out_orig = adapter.observation_to_kinematic_deg(side, observation, metadata_orig)
    out_variant = adapter.observation_to_kinematic_deg(side, observation, metadata_variant)
    act_orig = adapter.kinematic_deg_to_action(side, q, metadata_orig)
    act_variant = adapter.kinematic_deg_to_action(side, q, metadata_variant)

    assert np.array_equal(q, q_before)
    assert out_orig.shape == (6,) and out_variant.shape == (6,)
    # Variant outputs equal the frozen T-B4 literals.
    for index, joint in enumerate(JOINT_ORDER):
        assert out_variant[index] == pytest.approx(T_B4_EXPECTED_OBS_Q[side][index], abs=1e-9), (
            f"variant homing run broke the frozen obs expectation for {joint!r} "
            f"on side {side!r}: {out_variant[index]!r}"
        )
    keys = [f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER]
    assert set(act_variant) == set(keys)
    for joint in JOINT_ORDER:
        key = f"arm_{side}_{joint}.pos"
        assert act_variant[key] == pytest.approx(T_B4_EXPECTED_ACTION[side][joint], abs=1e-9), (
            f"variant homing run broke the frozen action expectation for {key!r} "
            f"on side {side!r}: {act_variant[key]!r}"
        )
    # And the pair is BITWISE identical in both directions.
    assert np.array_equal(out_variant, out_orig), (
        "RANGE forward output changed when only metadata homing_offset changed: "
        f"variant={out_variant!r} vs original={out_orig!r} (side={side!r})"
    )
    assert act_variant == act_orig, (
        "RANGE inverse output changed when only metadata homing_offset changed: "
        f"variant={act_variant!r} vs original={act_orig!r} (side={side!r})"
    )


# ======================================================================
# Behavior T-B6 (P1-B, design §8/§19): kinematic_deg_to_action 导出前范围
# 门禁（安全最高 behavior）。
#
#   effective range（每关节、按侧）= 换算后标定范围 ∩ URDF 限位
#
# 其中「换算后标定范围」是 metadata 标定 tick 窗口 [range_min, range_max]
# 经镜像公式 motor_deg = (tick - mid) * 360 / (resolution - 1) 换算为电机
# 度窗口、再经注入仿射 q = sign * motor_deg + zero 映射到 kinematic 度的
# 窗口（与 drive_mode 无关：取反只作用于 normalized 值，不改变
# tick<->deg 关系）。
#
# 模式作用域（由已 GREEN 的 T-B1–T-B5 冻结值反向钉死，不可协商）：
# - RANGE_M100_100：标定范围门禁无条件生效——bus 的 _unnormalize 对越界
#   normalized 值静默 clamp（bounded_val = min(100, max(-100, val))），
#   adapter 必须在导出前拒绝（§19：不依赖底层静默截断）。
# - DEGREES：bus 的 _unnormalize DEGREES 分支不做标定 clamp（仅 int 截断），
#   标定范围对导出无约束力 -> 换算后标定范围 = 无界。若按 tick 窗口换算
#   门禁，LEFT_MOTOR_SPECS 的 elbow_flex（标定 ±56.2°）/ wrist_yaw
#   (±77.2°) / wrist_roll (±73.0°) 会直接拒绝 T-B2 冻结值 -58.25 / -107.0
#   / -209.25 —— T-B1–T-B5 全线破坏，binding 排除该实现。DEGREES 的门禁
#   = URDF 限位（机械安全界，bus 不会替 adapter 把关）。
#
# URDF 限位经构造器注入（与 sign/zero 同 seam，按侧按关节，q_deg 空间的
# [lower, upper]），由 make_adapter 的 urdf_limits 参数绑定（binding
# seam，仅此一处）；urdf_limits=None 不传该关键字，T-B1–T-B5 构造形态
# 不变（DEGREES 无门禁、RANGE 仅标定范围门禁）。
#
# Cases（对齐任务 (a)-(h)）：
# - T-B6.a1 端点含端点可过 + 端点导出值精确（RANGE 标定门禁，全关节×双
#   侧×上下端点）。[既有实现在端点已正确导出 —— 预先满足，作回归护栏]
# - T-B6.a2 端点外一 epsilon 报错且不返回 dict（全关节×双侧×两侧）。
#   [RED：越界值被静默接受]
# - T-B6.a3 DEGREES 模式 URDF 限位门禁：端点可过（导出值精确）、端点外
#   epsilon 报错。[RED：构造器无 urdf_limits seam]
# - T-B6.b 仅一侧收窄：上下界分别来自 URDF/标定（判别「总用标定」
#   「总用 URDF」）。[RED：seam 缺失]
# - T-B6.c 交集为空（换算范围整体在限位下/上侧）→ 每次换算报错。
#   [RED：seam 缺失；构造期拒绝的 fail-fast 变体同样满足 §8「不得下发
#   运动」，用例接受两种形态并注明]
# - T-B6.d1 RANGE 退化 range_min==range_max → 报错且点名关节。[既有
#   _check_range 已覆盖 —— 回归护栏，钉住「导出前、不返回 dict」不退化]
# - T-B6.d2 DEGREES 退化 range → 报错且点名关节（bus 对退化标定的拒绝是
#   模式无关的，adapter 镜像之）。[RED：DEGREES 分支不查标定]
# - T-B6.e URDF 限位零宽 → 空交集，任何换算（含恰在零宽点上）报错。
#   [RED：seam 缺失]
# - T-B6.f 任一关节越界 → 整体拒绝：无 5 关节部分 dict、无零值填补。
#   [RED：越界被静默接受并返回被截断的 dict]
# - T-B6.g 非有限 metadata（range=±inf/NaN、res=NaN/0、zero=NaN、
#   limits=NaN）→ 导出前报错且错误点名关节。[RED：inf 经 round() 走
#   OverflowError、res=0 静默产出垃圾 normalized 值、DEGREES zero=NaN 静默
#   产出 NaN 命令、URDF NaN 无 seam；经 round(nan) 意外抛出的无关节名
#   ValueError 也被点名断言判 RED]
# - T-B6.h RANGE 越界 oracle 对照：bus _unnormalize 会静默 clamp ——
#   adapter 必须先拒绝；「返回了被截断的 dict」直接判失败（用例内逐字重
#   述 bus RANGE 分支作 oracle，失败信息同时给出请求连续 tick 与静默下发
#   tick）。
#
# 合理错误实现识别矩阵：
# - 依赖底层截断（不设门禁，靠 bus clamp 兜底）：T-B6.a2/f/h 与
#   T-B6.g(res=0 / zero=NaN-DEGREES) DID NOT RAISE + oracle 显示命令被截断。
# - 只查一侧（总用标定 / 总用 URDF）：T-B6.b 双向 ε 用例各击杀一种。
# - 部分导出+零值填补：T-B6.f else 分支的 len==6 与 !=0.0 断言。
# - 端点开区间误实现（拒绝端点本身）：T-B6.a1/a3/b/e 的端点可过断言。
# - DEGREES 误用标定范围门禁：直接破坏 T-B1–T-B5 冻结值（binding 排除），
#   T-B6.a3 钉死 DEGREES 门禁 = URDF。
#
# fixture 纪律：URDF 限位为合成窗口（以各关节 zero 为中心的非对称窗口，
# 下宽 350°/上宽 250°），两侧随 zero 不同而天然不对称，且刻意不等于任何
# 换算后标定范围（T_B3 ±180°、T_B4 ±720°）。SYNTHETIC FIXTURE NOTICE:
# 全部限位/标定/zero 值为合成、非对称、不可用于实机；真实限位需 Gate A/B
# 证据，严禁从本文件拷贝。纯对象、无 bus/时钟/网络/随机；端点与 ε 全部
# 落在 round 量化安全区（ε=1e-6° ≪ 最小半 tick 0.176°）。
# ======================================================================

# 端点外 epsilon（kinematic 度）。远小于任何合成 resolution 的半 tick
# （最小 W=1024 -> 半 tick = 360/1024/2 = 0.176°），不会触及 round 量化
# 边界，从而把「越界被静默接受」与量化抖动彻底解耦。
T_B6_EPS = 1e-6

# T-B6.b 的单侧收窄关节（每侧不同，捕获跨侧限位串扰）。
T_B6_NARROW_JOINT = {"left": "shoulder_lift", "right": "elbow_flex"}

# 合成 URDF 限位基线（构造器注入，按侧按关节，q_deg 空间 [lower, upper]）。
# 以各关节 zero 为中心的非对称窗口（下宽 350°/上宽 250°）；对 T_B3 标定
# （±180°）两侧皆更宽 -> 交集 = 标定范围；两侧随 zero 不同而互异；刻意
# 不等于任何换算后标定范围。
BASE_URDF_LIMITS = {
    "left": {
        "shoulder_pan": (-332.5, 267.5),
        "shoulder_lift": (-393.25, 206.75),
        "elbow_flex": (-258.25, 341.75),
        "wrist_flex": (-358.5, 241.5),
        "wrist_yaw": (-287.0, 313.0),
        "wrist_roll": (-470.5, 129.5),
    },
    "right": {
        "shoulder_pan": (-421.25, 178.75),
        "shoulder_lift": (-294.5, 305.5),
        "elbow_flex": (-362.0, 238.0),
        "wrist_flex": (-205.5, 394.5),
        "wrist_yaw": (-447.75, 152.25),
        "wrist_roll": (-319.0, 281.0),
    },
}


def converted_cal_q_range(side: str, joint: str, specs: dict[str, dict]) -> tuple[float, float]:
    """换算后标定范围（q_deg 空间，[lower, upper]）。

    metadata 标定 tick 窗口 [range_min, range_max] 经镜像公式
    motor_deg = (tick - mid) * 360 / (resolution - 1) 换算为电机度窗口，
    再经注入仿射 q = sign * motor_deg + zero 映射到 kinematic 度。与
    drive_mode 无关（取反只作用于 normalized 值，不改 tick<->deg 关系）。
    该公式与 T-B3/T-B4 冻结中间值所钉死的换算是同一条；若 production
    换算语义漂移，T-B3/T-B4 先 RED，此处无需重复钉值。
    """
    spec = specs[joint]
    sign = KINEMATIC_SIGNS[side][joint]
    zero = KINEMATIC_ZEROS[side][joint]
    mid = (spec["range_min"] + spec["range_max"]) / 2
    w = spec["resolution"] - 1
    m_lo = (spec["range_min"] - mid) * 360.0 / w
    m_hi = (spec["range_max"] - mid) * 360.0 / w
    q_lo = sign * m_lo + zero
    q_hi = sign * m_hi + zero
    return (q_lo, q_hi) if q_lo <= q_hi else (q_hi, q_lo)


def make_urdf_limits(
    overrides: dict[str, dict[str, tuple[float, float]]] | None = None,
) -> dict[str, dict[str, tuple[float, float]]]:
    """构造按侧全量 URDF 限位窗口（基线 + overrides 覆盖）。

    按侧全量 URDF 限位窗口：BASE_URDF_LIMITS 逐侧逐关节重建 +
    overrides（side -> joint -> (lower, upper)）替换。输入常量不被修改。
    """
    limits = {
        side: {joint: BASE_URDF_LIMITS[side][joint] for joint in JOINT_ORDER}
        for side in ("left", "right")
    }
    if overrides:
        for side, joints in overrides.items():
            limits[side].update(joints)
    return limits


def bus_unnormalize_range_oracle(normalized_value: float, spec: dict) -> int:
    """MotorsBus._unnormalize 的 RANGE_M100_100 分支逐字重述（oracle，只读）：

        val = -val if drive_mode else val
        bounded_val = min(100.0, max(-100.0, val))
        tick = int(((bounded_val + 100) / 200) * (range_max - range_min) + range_min)

    越界 normalized 值在此被静默 clamp 到标定边界 —— T-B6.h/f 以它证明
    adapter 返回的 dict 依赖了底层截断（§19 禁止）。
    """
    val = -normalized_value if spec["drive_mode"] == 1 else normalized_value
    bounded = min(100.0, max(-100.0, val))
    return int(
        ((bounded + 100.0) / 200.0) * (spec["range_max"] - spec["range_min"]) + spec["range_min"]
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b6_a_cal_endpoint_inclusive_passes_with_exact_value(side: str) -> None:
    """T-B6.a1（(a) 的端点半边）：换算后标定范围端点含端点可过，且端点 导出值精确。

    T_B3_SPECS 的 span == resolution-1 -> 每关节换算后标定范围恰为
    zero ± 180°；端点 q 映射到标定边界 tick（range_min/range_max），
    normalized 恰为 ±100，经 drive_mode 取反与 sign 翻转后导出值
    = sign * (+100 if dm==0 else -100)（上端点）/ 其相反数（下端点）。
    拒绝端点本身的「开区间误实现」在此被击杀。既有实现在端点已正确导出
    （其 clamp 对 ±100 是无操作）——本用例预先满足，作为门禁重构的回归
    护栏钉住端点行为不退化。
    """
    specs = T_B3_SPECS[side]
    adapter = make_adapter()
    metadata = _make_arm_metadata(side, specs)

    for index, joint in enumerate(JOINT_ORDER):
        spec = specs[joint]
        sign = KINEMATIC_SIGNS[side][joint]
        cal_lo, cal_hi = converted_cal_q_range(side, joint, specs)
        expected_hi = 100.0 * sign if spec["drive_mode"] == 0 else -100.0 * sign
        expected_lo = -expected_hi
        for q_value, expected in ((cal_hi, expected_hi), (cal_lo, expected_lo)):
            q = np.array(T_B3_Q_DEG[side], dtype=np.float64)
            q[index] = q_value
            action = adapter.kinematic_deg_to_action(side, q, metadata)
            assert isinstance(action, dict)
            assert set(action) == {f"arm_{side}_{j}.pos" for j in JOINT_ORDER}
            assert action[f"arm_{side}_{joint}.pos"] == pytest.approx(expected, abs=1e-9), (
                f"calibration-range endpoint q={q_value!r} for {joint!r} on "
                f"side {side!r} must be accepted (inclusive) with exact export "
                f"{expected!r}; got {action[f'arm_{side}_{joint}.pos']!r}"
            )


@pytest.mark.parametrize(
    ["side", "joint"], [(s, j) for s in ("left", "right") for j in JOINT_ORDER]
)
def test_t_b6_a_epsilon_beyond_effective_range_raises(side: str, joint: str) -> None:
    """T-B6.a2：有效范围端点外一 epsilon 必须报错且点名关节。

    T-B6.a2（(a) 的 epsilon 半侧 + (f) 的逐关节覆盖）：换算后标定范围
    端点外一 epsilon -> 报错且不返回 dict，错误点名关节。

    每侧每关节的上/下端点各越界一次（其余关节保持在各自界内），故门禁
    必须逐关节、逐方向生效；「越界值被静默接受/靠 bus clamp 兜底」的实现
    （当前实现：normalized 被 min/max 静默 clamp 后照常返回 dict）在此
    全部 DID NOT RAISE。量化安全：ε=1e-6° 对应 tick 偏移 <2e-4，越界样本
    量化后仍落在端点 tick 上 —— 拒绝必须发生在输入 q_deg 上（§19），而非
    量化之后。
    """
    specs = T_B3_SPECS[side]
    cal_lo, cal_hi = converted_cal_q_range(side, joint, specs)
    # 前提：T_B3 标定窗口宽 == resolution-1 -> 换算后标定范围宽度恰 360°。
    assert cal_hi - cal_lo == pytest.approx(360.0, abs=1e-9)
    adapter = make_adapter()
    metadata = _make_arm_metadata(side, specs)
    index = JOINT_ORDER.index(joint)

    for bad_q, direction in ((cal_hi + T_B6_EPS, "upper"), (cal_lo - T_B6_EPS, "lower")):
        q = np.array(T_B3_Q_DEG[side], dtype=np.float64)
        q[index] = bad_q
        with pytest.raises(ValueError) as excinfo:
            adapter.kinematic_deg_to_action(side, q, metadata)
        assert joint in str(excinfo.value), (
            f"q beyond the {direction} effective-range endpoint of {joint!r} "
            f"on side {side!r} must raise ValueError naming the joint; got: "
            f"{excinfo.value!r}"
        )


def test_t_b6_a_degrees_urdf_limit_gate_endpoint_and_epsilon() -> None:
    """T-B6.a3：DEGREES 模式下 URDF 限位门禁同样生效（端点含端点可过， 端点外 epsilon 报错）。

    DEGREES 的换算后标定范围 = 无界（bus 的 DEGREES 分支不做标定 clamp），
    故本用例的门禁完全来自注入的 URDF 限位：left wrist_roll 窗口
    (−150, −100)，端点 −100/−150 导出值恰为 sign*(q−zero) = −20.5/+29.5
    （纯仿射、无量化），端点外 ε 报错点名关节。机械安全界只有 adapter 能
    把关（DEGREES 模式 bus 会原样下发任意度数）。
    """
    limits = make_urdf_limits(overrides={"left": {"wrist_roll": (-150.0, -100.0)}})
    adapter = make_adapter(urdf_limits=limits)
    metadata = _make_arm_metadata("left", LEFT_MOTOR_SPECS)
    index = JOINT_ORDER.index("wrist_roll")

    cases = (
        (-100.0, -20.5, True),  # URDF 上端点：可过，导出值精确
        (-100.0 + T_B6_EPS, None, False),  # 上端点外 ε：报错
        (-150.0, 29.5, True),  # URDF 下端点：可过，导出值精确
        (-150.0 - T_B6_EPS, None, False),  # 下端点外 ε：报错
    )
    for q_value, expected, should_pass in cases:
        q = np.array(LEFT_Q_DEG, dtype=np.float64)
        q[index] = q_value
        if should_pass:
            action = adapter.kinematic_deg_to_action("left", q, metadata)
            assert set(action) == {f"arm_left_{j}.pos" for j in JOINT_ORDER}
            assert action["arm_left_wrist_roll.pos"] == pytest.approx(expected, abs=1e-9), (
                f"URDF endpoint q={q_value!r} must export exactly {expected!r}; "
                f"got {action['arm_left_wrist_roll.pos']!r}"
            )
        else:
            with pytest.raises(ValueError) as excinfo:
                adapter.kinematic_deg_to_action("left", q, metadata)
            assert "wrist_roll" in str(excinfo.value)


@pytest.mark.parametrize(
    ["side", "narrow"],
    [(s, w) for s in ("left", "right") for w in ("top", "bottom")],
)
def test_t_b6_b_one_sided_narrowing_bounds_from_urdf_and_cal(side: str, narrow: str) -> None:
    """T-B6.b：仅一侧收窄时，上下界分别取自 URDF 与标定。

    T-B6.b：仅一侧收窄时，上下界分别来自 URDF/标定 —— 判别「总用标定」
    与「总用 URDF」两类误实现。

    目标关节（left: shoulder_lift，right: elbow_flex）注入单侧收窄窗口：
    - top 收窄：URDF = (标定下界−100, 标定上界−50) -> 有效范围下界仍来自
      标定（总用 URDF 的实现会接受下界外 ε -> 被击杀），上界来自 URDF
      （总用标定的实现会接受上界外 ε -> 被击杀）。
    - bottom 收窄：URDF = (标定下界+60, 标定上界+100)，对称击杀。
    两侧用不同关节，捕获跨侧限位串扰。端点可过断言同时排除「开区间误
    实现」。
    """
    specs = T_B3_SPECS[side]
    joint = T_B6_NARROW_JOINT[side]
    cal_lo, cal_hi = converted_cal_q_range(side, joint, specs)
    if narrow == "top":
        urdf_window = (cal_lo - 100.0, cal_hi - 50.0)
        eff_lo, eff_hi = cal_lo, cal_hi - 50.0
    else:
        urdf_window = (cal_lo + 60.0, cal_hi + 100.0)
        eff_lo, eff_hi = cal_lo + 60.0, cal_hi
    # 前提：窗口确实只收窄预期的一侧，且收窄后非退化。
    if narrow == "top":
        assert urdf_window[1] < cal_hi and urdf_window[0] < cal_lo
    else:
        assert urdf_window[0] > cal_lo and urdf_window[1] > cal_hi
    assert eff_lo < eff_hi

    limits = make_urdf_limits(overrides={side: {joint: urdf_window}})
    adapter = make_adapter(urdf_limits=limits)
    metadata = _make_arm_metadata(side, specs)
    index = JOINT_ORDER.index(joint)
    key = f"arm_{side}_{joint}.pos"

    cases = (
        (eff_hi, True),  # 收窄侧端点（URDF 或标定提供）可过
        (eff_hi + T_B6_EPS, False),  # 上界外 ε：报错（判别上界来源）
        (eff_lo, True),  # 另一侧端点（标定或 URDF 提供）可过
        (eff_lo - T_B6_EPS, False),  # 下界外 ε：报错（判别下界来源）
    )
    for q_value, should_pass in cases:
        q = np.array(T_B3_Q_DEG[side], dtype=np.float64)
        q[index] = q_value
        if should_pass:
            action = adapter.kinematic_deg_to_action(side, q, metadata)
            assert set(action) == {f"arm_{side}_{j}.pos" for j in JOINT_ORDER}
            value = action[key]
            assert np.isfinite(value) and -100.0 - 1e-9 <= value <= 100.0 + 1e-9, (
                f"effective-range endpoint q={q_value!r} for {key!r} must export "
                f"an in-domain finite value; got {value!r}"
            )
        else:
            with pytest.raises(ValueError) as excinfo:
                adapter.kinematic_deg_to_action(side, q, metadata)
            assert joint in str(excinfo.value), (
                f"q beyond the narrowed effective range of {joint!r} on side "
                f"{side!r} must raise naming the joint; got: {excinfo.value!r}"
            )


@pytest.mark.parametrize("window", ["below", "above"])
def test_t_b6_c_empty_intersection_rejects_every_conversion(window: str) -> None:
    """T-B6.c：交集为空（换算后标定范围整体在 URDF 限位下/上侧）→ 每次 换算报错。

    left wrist_flex 标定范围 = (−188.5, 171.5)；注入整体在其下侧 (−600,
    −400) 或上侧 (300, 500) 的 URDF 窗口 -> 交集为空。两次换算（一次 q
    在标定范围内、一次 q 恰在 URDF 窗口内——后者专杀「只查 URDF」的误实
    现）都必须报错并点名关节。构造期拒绝（fail-fast）同样满足 §8「不得
    下发运动」，用例接受两种形态：构造抛 ValueError 视为通过，否则逐次
    调用断言报错。
    """
    specs = T_B3_SPECS["left"]
    cal_lo, cal_hi = converted_cal_q_range("left", "wrist_flex", specs)
    urdf_window = (-600.0, -400.0) if window == "below" else (300.0, 500.0)
    # 前提：URDF 窗口与标定范围整体不相交（空交集确由错位配置导致）。
    assert urdf_window[1] < cal_lo or urdf_window[0] > cal_hi

    limits = make_urdf_limits(overrides={"left": {"wrist_flex": urdf_window}})
    try:
        adapter = make_adapter(urdf_limits=limits)
    except ValueError:
        # 构造期拒绝变体：空交集在 seam 上已使「不得下发运动」成立。
        return
    metadata = _make_arm_metadata("left", specs)
    index = JOINT_ORDER.index("wrist_flex")

    q_in_cal = np.array(T_B3_Q_DEG["left"], dtype=np.float64)  # wrist_flex=134.32 在标定范围内
    q_in_urdf = q_in_cal.copy()
    q_in_urdf[index] = -500.0 if window == "below" else 400.0  # 恰在 URDF 窗口内
    for label, q in (("q_within_calibration", q_in_cal), ("q_within_urdf_window", q_in_urdf)):
        with pytest.raises(ValueError) as excinfo:
            adapter.kinematic_deg_to_action("left", q, metadata)
        assert "wrist_flex" in str(excinfo.value), (
            f"empty effective range ({window} window) must reject EVERY "
            f"conversion ({label}) naming wrist_flex; got: {excinfo.value!r}"
        )


def test_t_b6_d_range_degenerate_calibration_raises_before_export() -> None:
    """T-B6.d1：RANGE 退化标定（range_min==range_max）导出前即拒绝。

    T-B6.d1（(d) 的 RANGE 半侧）：RANGE_M100_100 元数据退化
    range_min == range_max -> 导出前报错且不返回 dict，错误点名关节。

    既有实现已在 RANGE 分支经 _check_range 拒绝（与 bus 的模式无关退化
    检查一致）——本用例预先满足，作为门禁重构的回归护栏：钉住「退化
    标定在导出前拒绝、错误点名关节」不退化。
    """
    specs = dict(T_B3_SPECS["right"])
    specs["shoulder_lift"] = {
        **T_B3_SPECS["right"]["shoulder_lift"],
        "range_min": 2500,
        "range_max": 2500,
    }
    metadata = _make_arm_metadata("right", specs)
    adapter = make_adapter()
    q = np.array(T_B3_Q_DEG["right"], dtype=np.float64)

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("right", q, metadata)
    assert "shoulder_lift" in str(excinfo.value)


def test_t_b6_d_degrees_degenerate_calibration_raises_before_export() -> None:
    """T-B6.d2：DEGREES 退化标定同样导出前拒绝并点名关节。

    T-B6.d2（(d) 的 DEGREES 半侧）：DEGREES 元数据退化 range_min ==
    range_max -> 同样导出前报错并点名关节。

    bus 对退化标定的拒绝是模式无关的（_normalize/_unnormalize 都在分支
    之前检查 max_ == min_），adapter 镜像之；「DEGREES 分支不查标定」的
    实现会静默返回 dict —— RED。
    """
    specs = dict(LEFT_MOTOR_SPECS)
    specs["shoulder_pan"] = {
        **LEFT_MOTOR_SPECS["shoulder_pan"],
        "range_min": 2000,
        "range_max": 2000,
    }
    metadata = _make_arm_metadata("left", specs)
    adapter = make_adapter()
    q = np.array(LEFT_Q_DEG, dtype=np.float64)

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("left", q, metadata)
    assert "shoulder_pan" in str(excinfo.value), (
        f"degenerate range_min==range_max on a DEGREES motor must raise "
        f"naming shoulder_pan (the bus rejects degenerate calibration in "
        f"every mode); got: {excinfo.value!r}"
    )


def test_t_b6_e_zero_width_urdf_limit_rejects_every_conversion() -> None:
    """T-B6.e：URDF 限位零宽（lower == upper）→ 空交集，任何换算报错。

    right wrist_roll 标定范围 = (−149, 211)；注入零宽窗口 (10, 10) 严格
    位于标定范围内（前提断言排除「标定为空所致」）。两次换算：一次 q 恰
    等于零宽点本身（「把零宽窗口当可命令单点」的误实现会接受它 -> 被击
    杀），一次 q 在窗口外 —— 都必须报错并点名关节。构造期拒绝变体同
    T-B6.c 一样被接受。
    """
    specs = T_B3_SPECS["right"]
    cal_lo, cal_hi = converted_cal_q_range("right", "wrist_roll", specs)
    assert cal_lo < 10.0 < cal_hi  # 零宽窗口严格在标定范围内

    limits = make_urdf_limits(overrides={"right": {"wrist_roll": (10.0, 10.0)}})
    try:
        adapter = make_adapter(urdf_limits=limits)
    except ValueError:
        return  # 构造期拒绝变体：同样满足「不得下发运动」。
    metadata = _make_arm_metadata("right", specs)
    index = JOINT_ORDER.index("wrist_roll")

    q_point = np.array(T_B3_Q_DEG["right"], dtype=np.float64)
    q_point[index] = 10.0  # 恰在零宽点上
    q_outside = q_point.copy()
    q_outside[index] = 50.0
    for label, q in (("q_at_zero_width_point", q_point), ("q_outside_window", q_outside)):
        with pytest.raises(ValueError) as excinfo:
            adapter.kinematic_deg_to_action("right", q, metadata)
        assert "wrist_roll" in str(excinfo.value), (
            f"zero-width URDF window must reject EVERY conversion ({label}) "
            f"naming wrist_roll; got: {excinfo.value!r}"
        )


def test_t_b6_f_any_joint_out_of_range_rejects_whole_conversion() -> None:
    """T-B6.f：任一关节越界 → 整体拒绝：无 5 关节部分 dict、无零值填补。

    T_B4_SPECS[right] 标定范围 = zero ± 720°；仅 wrist_yaw 置为 −900
    （q 空间低于其下界 −817.75 达 82.25°，经注入仿射 sign=-1 映射为电机
    度上越界 +802.25°，其余五关节均在各自界内）。报错必须点名 wrist_yaw；
    若实现未报错而返回 dict，else 分支逐项诊断：部分 dict（len<6）、零值
    填补（越界关节 == 0.0）、以及 bus 静默 clamp oracle（返回值重述后
    tick 恰为标定边界，方向由注入符号决定 —— 「依赖底层截断」的直接证
    据）。
    """
    specs = T_B4_SPECS["right"]
    joint = "wrist_yaw"
    spec = specs[joint]
    cal_lo, cal_hi = converted_cal_q_range("right", joint, specs)
    assert cal_lo - 1.0 > -900.0  # 前提：−900 严格低于下界（非边界抖动）
    adapter = make_adapter()
    metadata = _make_arm_metadata("right", specs)
    q = np.array(T_B4_Q_DEG["right"], dtype=np.float64)
    q[JOINT_ORDER.index(joint)] = -900.0

    try:
        action = adapter.kinematic_deg_to_action("right", q, metadata)
    except ValueError as exc:
        assert joint in str(exc), f"out-of-range joint must be named in the error; got: {exc!r}"
    else:
        key = f"arm_right_{joint}.pos"
        assert len(action) == 6, (
            f"T-B6.f: 越界时返回了部分 dict（{len(action)} 键，缺 "
            f"{sorted({f'arm_right_{j}.pos' for j in JOINT_ORDER} - set(action))!r}）"
            f"—— 部分导出被禁止"
        )
        assert action[key] != 0.0, (
            f"T-B6.f: 越界关节 {key!r} 被零值填补（{action[key]!r}）—— 部分导出+零值填补被禁止"
        )
        # q 空间低于下界的样本经注入仿射（sign=-1）映射为电机度上越界：
        # 静默 clamp 落在标定窗口的哪一端由 sign 决定，此处按符号推导。
        offender_deg = KINEMATIC_SIGNS["right"][joint] * (-900.0 - KINEMATIC_ZEROS["right"][joint])
        clamp_bound = spec["range_max"] if offender_deg > 0 else spec["range_min"]
        oracle_tick = bus_unnormalize_range_oracle(action[key], spec)
        assert oracle_tick == clamp_bound, (
            f"T-B6.f: 返回值 {action[key]!r} 经 bus oracle 重述得到 tick "
            f"{oracle_tick}，不等于静默 clamp 边界（{clamp_bound}）—— "
            f"越界实现形态与预期不符，需重审"
        )
        pytest.fail(
            f"T-B6.f: 越界 q=-900.0（wrist_yaw 有效范围 ({cal_lo}, {cal_hi})）被"
            f"静默接受并返回被截断的 dict（action[{key!r}]={action[key]!r}）；"
            f"bus _unnormalize 会静默 clamp 并下发 tick {oracle_tick}"
            f"（=标定边界 {clamp_bound}）—— adapter 必须在导出前整体拒绝"
        )


@pytest.mark.parametrize(
    ["field", "value"],
    [
        ("range_max", float("inf")),
        ("range_min", float("-inf")),
        ("range_min", float("nan")),
    ],
    ids=["range_max_inf", "range_min_neg_inf", "range_min_nan"],
)
def test_t_b6_g1_non_finite_calibration_range_rejected_before_export(
    field: str, value: float
) -> None:
    """T-B6.g1（(g) 的 range 半侧）：非有限标定范围（±inf / NaN）→ 导出 前报错且点名关节。

    left elbow_flex 元数据逐变体替换一个 range 字段。当前实现不设导出前
    有限性检查：±inf 经 round(inf) 意外抛出 OverflowError（错误类型与
    契约不符、且发生在量化中途），NaN 经 round(nan) 抛出无关节名的
    ValueError —— 点名断言对两者都 RED。正确实现应在导出前以 ValueError
    拒绝并命名 elbow_flex（§19：导出前重查 finite）。
    """
    specs = dict(T_B3_SPECS["left"])
    specs["elbow_flex"] = {**T_B3_SPECS["left"]["elbow_flex"], field: value}
    metadata = _make_arm_metadata("left", specs)
    adapter = make_adapter()
    q = np.array(T_B3_Q_DEG["left"], dtype=np.float64)

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("left", q, metadata)
    assert "elbow_flex" in str(excinfo.value), (
        f"non-finite calibration range ({field}={value!r}) must raise "
        f"ValueError naming elbow_flex BEFORE export; got: {excinfo.value!r}"
    )


@pytest.mark.parametrize("resolution", [0, float("nan")], ids=["res_zero", "res_nan"])
def test_t_b6_g2_zero_or_nan_resolution_rejected_before_export(
    resolution: float,
) -> None:
    """T-B6.g2（(g) 的 resolution 半侧）：resolution 为 0 或 NaN → 导出前 报错且点名关节。

    right wrist_flex 元数据逐变体替换 resolution。当前实现：res=0 使
    (res-1) = -1，tick/normalized 静默产出与请求无关的垃圾值（本样本恰
    产出 normalized = -0.0）并照常返回 dict —— 「静默垃圾命令」比崩溃更
    危险；res=NaN 经 round(nan) 抛出无关节名的 ValueError。两者都必须在
    导出前以点名关节的 ValueError 拒绝。
    """
    specs = dict(T_B3_SPECS["right"])
    specs["wrist_flex"] = {
        **T_B3_SPECS["right"]["wrist_flex"],
        "resolution": resolution,
    }
    metadata = _make_arm_metadata("right", specs)
    adapter = make_adapter()
    q = np.array(T_B3_Q_DEG["right"], dtype=np.float64)

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("right", q, metadata)
    assert "wrist_flex" in str(excinfo.value), (
        f"resolution={resolution!r} must raise ValueError naming wrist_flex "
        f"BEFORE export (no silent garbage command); got: {excinfo.value!r}"
    )


@pytest.mark.parametrize("mode", ["degrees", "range"])
def test_t_b6_g3_non_finite_zero_offset_rejected_before_export(mode: str) -> None:
    """T-B6.g3（(g) 的 zero 半侧）：注入 zero = NaN → 导出前报错并点名 关节。

    zero 是注入仿射的组成部分（两种模式都使用）：NaN zero 使 motor_deg
    为 NaN。DEGREES 变体：当前实现静默返回含 NaN 命令值的 dict（比崩溃
    更危险，直接 RED）；RANGE 变体：NaN 经 round(nan) 抛出无关节名的
    ValueError（点名断言 RED）。正确实现都应在导出前点名 elbow_flex
    拒绝。T-B11 裁决：构造期拒绝（fail-fast）变体同样满足「导出前拒绝」
    语义，与 T-B6.g4/c/e 一致被接受；构造成功路径维持导出期断言原强度。
    """
    try:
        adapter = make_adapter(zero_overrides={"left": {"elbow_flex": float("nan")}})
    except ValueError:
        return  # 构造期拒绝变体：同样满足「导出前拒绝」。
    if mode == "degrees":
        metadata = _make_arm_metadata("left", LEFT_MOTOR_SPECS)
        q = np.array(LEFT_Q_DEG, dtype=np.float64)
    else:
        metadata = _make_arm_metadata("left", T_B3_SPECS["left"])
        q = np.array(T_B3_Q_DEG["left"], dtype=np.float64)

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("left", q, metadata)
    assert "elbow_flex" in str(excinfo.value), (
        f"non-finite zero offset (mode={mode!r}) must raise ValueError naming "
        f"elbow_flex BEFORE export; got: {excinfo.value!r}"
    )


@pytest.mark.parametrize(
    ["lo", "hi"],
    [(float("nan"), 100.0), (-100.0, float("nan"))],
    ids=["lower_nan", "upper_nan"],
)
def test_t_b6_g4_non_finite_urdf_limit_rejected_before_export(lo: float, hi: float) -> None:
    """T-B6.g4（(g) 的 URDF 限位半侧）：URDF 限位含 NaN → 导出前报错并 点名关节。

    left shoulder_pan 注入含 NaN 的窗口（(NaN, 100) / (−100, NaN)），
    样本 q 严格位于有限侧界之内 —— 只有非有限限位检查能拒绝（NaN 比较
    恒 False 的「忽略 NaN」实现会放行 -> 被击杀）。构造期拒绝（fail-fast）
    变体同样满足 §8，与 T-B6.c/e 一致被接受。
    """
    limits = make_urdf_limits(overrides={"left": {"shoulder_pan": (lo, hi)}})
    try:
        adapter = make_adapter(urdf_limits=limits)
    except ValueError:
        return  # 构造期拒绝变体：同样满足「不得下发运动」。
    metadata = _make_arm_metadata("left", T_B3_SPECS["left"])
    q = np.array(T_B3_Q_DEG["left"], dtype=np.float64)
    q[0] = 50.0  # 严格位于有限侧界之内：只有非有限限位检查能拒绝

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action("left", q, metadata)
    assert "shoulder_pan" in str(excinfo.value), (
        f"non-finite URDF limit ({lo!r}, {hi!r}) must raise ValueError naming "
        f"shoulder_pan; got: {excinfo.value!r}"
    )


def test_t_b6_h_bus_silent_clamp_pre_rejected_with_oracle() -> None:
    """T-B6.h：越界必须先拒绝，不得依赖 bus _unnormalize 的静默 clamp。

    T-B6.h（(h) oracle 对照）：RANGE_M100_100 越界时 bus _unnormalize 会
    静默 clamp —— adapter 必须先拒绝；「返回了被截断的 dict」直接判失败。

    T_B4_SPECS[left] 标定范围 = zero ± 720°；shoulder_pan 置为 800（高于
    上界 737.5 达 62.5°，其余五关节在界内）。oracle 用例内逐字重述
    MotorsBus._unnormalize 的 RANGE 分支：若 adapter 未报错而返回 dict，
    其返回值经 oracle 重述得到 tick == 标定上界（静默 clamp 的直接证
    据），而请求位置对应连续 tick ≈ 773.96 —— 命令被静默截断 178 tick。
    失败信息同时给出两者，指认「依赖底层截断」。
    """
    specs = T_B4_SPECS["left"]
    joint = "shoulder_pan"
    spec = specs[joint]
    cal_lo, cal_hi = converted_cal_q_range("left", joint, specs)
    assert cal_hi < 800.0 < cal_hi + 100.0  # 前提：800 严格高于上界
    adapter = make_adapter()
    metadata = _make_arm_metadata("left", specs)
    q = np.array(T_B4_Q_DEG["left"], dtype=np.float64)
    q[0] = 800.0

    try:
        action = adapter.kinematic_deg_to_action("left", q, metadata)
    except ValueError as exc:
        assert joint in str(exc), f"out-of-range joint must be named in the error; got: {exc!r}"
    else:
        key = f"arm_left_{joint}.pos"
        returned = action[key]
        oracle_tick = bus_unnormalize_range_oracle(returned, spec)
        requested_cont_tick = (
            KINEMATIC_SIGNS["left"][joint] * (800.0 - KINEMATIC_ZEROS["left"][joint])
        ) * (spec["resolution"] - 1) / 360.0 + (spec["range_min"] + spec["range_max"]) / 2
        assert oracle_tick == spec["range_max"], (
            f"T-B6.h: 返回值 {returned!r} 经 bus oracle 重述得到 tick "
            f"{oracle_tick}，不等于静默 clamp 上边界 range_max={spec['range_max']} "
            f"—— 越界实现形态与预期不符，需重审"
        )
        pytest.fail(
            f"T-B6.h: 越界 q_deg=800.0（高于 shoulder_pan 换算后标定上界 "
            f"{cal_hi}）被静默接受：返回了被截断的 dict "
            f"(action[{key!r}]={returned!r})。bus _unnormalize 会静默 clamp 并"
            f"下发 tick {oracle_tick}（=标定上界 {spec['range_max']}），而请求位"
            f"置对应连续 tick {requested_cont_tick:.2f} —— 命令被静默截断，"
            f"adapter 必须在导出前拒绝，不得依赖底层截断"
        )


# ======================================================================
# Behavior T-B7 (P1-B 单位适配, design §8/§9.1/§19): 缺失/非法输入拒绝路径。
#
# 裁决契约（逐条显式化；主 Agent 已裁决，production 只读）：
#   a) 逐个删除每电机必需字段（resolution / range_min / range_max /
#      normalization / drive_mode / homing_offset）→ 报错且含 side/joint/
#      字段名（§9.1：Cartesian 要求完整 v2 metadata，不得降级使能；
#      字段完整性按 metadata 整体校验，与 normalization 模式无关）。
#   b) 数值字段（resolution / range_min / range_max / drive_mode）为
#      None / 字符串 / inf / bool → 报错（F-1d 先例：bool 不得借
#      True==1 混入数值运算；字符串不得被宽松强转收下）。
#   c) `schema_version` 缺失或 ≠2 → 报错（v1 metadata 不得被收下使能）。
#   d) obs 缺一个 `.pos` 键 → 报错，不得零值填补（显式化 T-B1 隐含契约）。
#   e) 未知 normalization 值 → 报错（显式化 T-B3 已覆盖的 _motor_entry 契约）。
#   f) 多余电机键（gripper / base / lift）→ 忽略不报错；校验只针对六必需
#      关节电机条目（实机 bus 必含夹爪；宿主层会跳过未标定电机，多余
#      条目不得被要求字段完整）。
#   g) side 非 "left"/"right" → obs 方向报错且含该值（T-B2.7 已覆盖
#      action 方向；"LEFT" 变体击杀大小写不敏感实现）。
#
# 要击杀的合理错误实现：缺字段默认 0 补全（静默继续）；收下 v1 / 缺
# schema_version 的 metadata；bool=True 当 1 用（穿过 drive_mode 的 ==1
# 比较）；float() 宽松强转（数字字符串 "6399" 被静默接受）；只在单方向
# （obs 或 action）或单模式（DEGREES）校验；校验蔓延到多余电机条目。
#
# 探针关节固定为 JOINT_ORDER 序列中部的 wrist_flex（index 3）：击杀「只
# 校验首个条目」的实现形态。RANGE 组探针为 T_B3 left elbow_flex
# （drive_mode=0）：drive_mode=True 被误用为 1 时取反被错误施加，静默产出
# 错误值 —— F-1d 危害的最尖锐形态。
#
# 分层与 seam：纯对象 component 测试，复用既有 make_adapter /
# make_arm_observation / make_metadata / _make_arm_metadata / LEFT_* /
# T_B3_* fixture。无 bus / 网络 / 时钟 / 随机 / sleep。错误契约遵循 §19
# 与 T-B6.g 先例：ValueError 且点名 side/joint/字段（envelope 级
# schema_version 无 joint 语义，只断言 ValueError 类型）。
# ======================================================================

# 每电机必需字段（主 Agent 裁决列表；id/model 不在 adapter 校验范围）。
T_B7_REQUIRED_FIELDS = (
    "resolution",
    "range_min",
    "range_max",
    "normalization",
    "drive_mode",
    "homing_offset",
)

# 换算数学实际消费的数值字段（b) 组逐字段投毒的范围）。homing_offset
# 不被 adapter 消费，不做类型校验断言（只要求存在，见 a 组）。
T_B7_NUMERIC_FIELDS = ("resolution", "range_min", "range_max", "drive_mode")

# b 组非法值：None（缺值强转 TypeError）、"6399"（数字字符串——宽松
# float() 强转的击杀样本）、inf（非有限）、True（bool 借 True==1 混入）。
_T_B7_BAD_NUMERIC_VALUES = (
    ("none", None),
    ("numeric_str", "6399"),
    ("inf", float("inf")),
    ("bool_true", True),
)

# a/b 组探针：JOINT_ORDER index 3（序列中部，击杀只查首条目的实现）。
T_B7_PROBE_JOINT = "wrist_flex"
# c 组探针：T_B3 left elbow_flex（drive_mode=0，True 被误用时取反被错误
# 施加 → 静默错误值）。
_T_B7_RANGE_PROBE_JOINT = "elbow_flex"


def _t_b7_left_degrees_motors() -> dict[str, dict]:
    """干净的 DEGREES 左臂 motors 字典（恰好六条必需条目）。

    Clean DEGREES-mode left-arm motors dict (exactly six required entries,
    each with the full v2 field set of LEFT_MOTOR_SPECS).
    """
    return {f"arm_left_{joint}": dict(entry) for joint, entry in LEFT_MOTOR_SPECS.items()}


def _t_b7_run(
    direction: str,
    adapter: object,
    side: str,
    metadata: dict,
    *,
    observation: dict | None = None,
    q_deg: np.ndarray | None = None,
) -> object:
    """按方向跑一次 adapter 调用（默认用该侧冻结的 DEGREES 输入）。

    One adapter call in the given direction. Defaults are the side's
    frozen T-B1/T-B2 DEGREES-mode inputs; explicit observation / q_deg
    overrides serve the RANGE-mode cases (T_B3 fixtures).
    """
    if direction == "obs":
        if observation is None:
            observation = make_arm_observation(
                side, LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG
            )
        return adapter.observation_to_kinematic_deg(side, observation, metadata)
    if q_deg is None:
        q_deg = np.array(LEFT_Q_DEG if side == "left" else RIGHT_Q_DEG, dtype=np.float64)
    return adapter.kinematic_deg_to_action(side, q_deg, metadata)


def _t_b7_assert_named(excinfo: object, *, side: str, joint: str, field: str) -> None:
    """§19 错误契约：报错必须点名 side、joint 与字段。

    §19 error contract: the message must name the side, the joint and the
    offending field (bare KeyError / unnamed ValueError is a wrong shape).
    """
    message = str(excinfo.value)
    assert side in message, f"rejection message must name the side {side!r}; got: {message!r}"
    assert joint in message, f"rejection message must name the joint {joint!r}; got: {message!r}"
    assert field in message, f"rejection message must name the field {field!r}; got: {message!r}"


@pytest.mark.parametrize("direction", ["obs", "action"], ids=["obs_dir", "action_dir"])
@pytest.mark.parametrize("field", T_B7_REQUIRED_FIELDS)
def test_t_b7_a_missing_required_metadata_field_rejected_with_side_joint_field(
    field: str, direction: str
) -> None:
    """T-B7.a：缺每电机必需字段时双方向拒绝并点名 side/joint/字段。

    T-B7.a（§9.1 完整 v2）：逐个删除 wrist_flex 电机条目的每电机必需
    字段（其余五条目完整）→ 双方向均报 ValueError 且含 side/joint/字段名。

    当前实现（DEGREES 模式）只读 normalization：删除其余任一字段后静默
    照常换算 —— 「缺字段默认 0 补全/静默继续」类实现直接 RED。action 方向
    删除 range_min/range_max 时当前实现经 _check_range 的裸下标访问抛出
    KeyError（非 ValueError、无 side/joint 命名）—— 错误形态契约 RED。
    正确实现应在校验层以点名 side/joint/字段的 ValueError 拒绝。
    """
    motors = _t_b7_left_degrees_motors()
    del motors[f"arm_left_{T_B7_PROBE_JOINT}"][field]
    metadata = make_metadata(motors)
    adapter = make_adapter()

    with pytest.raises(ValueError) as excinfo:
        _t_b7_run(direction, adapter, "left", metadata)

    _t_b7_assert_named(excinfo, side="left", joint=T_B7_PROBE_JOINT, field=field)


@pytest.mark.parametrize("direction", ["obs", "action"], ids=["obs_dir", "action_dir"])
@pytest.mark.parametrize(["value_id", "bad_value"], _T_B7_BAD_NUMERIC_VALUES)
@pytest.mark.parametrize("field", T_B7_NUMERIC_FIELDS)
def test_t_b7_b_invalid_numeric_metadata_field_rejected(
    field: str, value_id: str, bad_value: object, direction: str
) -> None:
    """T-B7.b：数值字段为 None/字符串/inf/bool 时双方向拒绝并点名。

    T-B7.b（F-1d 先例）：数值字段为 None / 数字字符串 / inf / bool →
    双方向均报 ValueError 且含 side/joint/字段名。

    DEGREES 模式下当前实现不消费这四个字段：任一非法值都静默照常换算
    （bool=True 借 True==1 潜入、数字字符串 "6399" 可被宽松 float() 强转
    收下、None/inf 留给下游运算中途崩溃）—— 全部变体 RED。正确实现应把
    metadata 数值有效性作为模式无关的整体校验，在换算前点名拒绝。
    """
    motors = _t_b7_left_degrees_motors()
    motors[f"arm_left_{T_B7_PROBE_JOINT}"][field] = bad_value
    metadata = make_metadata(motors)
    adapter = make_adapter()

    with pytest.raises(ValueError) as excinfo:
        _t_b7_run(direction, adapter, "left", metadata)

    _t_b7_assert_named(excinfo, side="left", joint=T_B7_PROBE_JOINT, field=field)


@pytest.mark.parametrize("direction", ["obs", "action"], ids=["obs_dir", "action_dir"])
@pytest.mark.parametrize(["value_id", "bad_value"], [("bool_true", True), ("none", None)])
def test_t_b7_c_range_drive_mode_bool_and_none_rejected(
    value_id: str, bad_value: object, direction: str
) -> None:
    """T-B7.b（RANGE 半侧）：drive_mode=True/None 换算前即拒绝。

    T-B7.b 的 RANGE 模式半侧（F-1d 核心危害）：drive_mode=True/None →
    换算前报 ValueError 且点名 side/joint/drive_mode。

    探针为 T_B3 left elbow_flex（drive_mode=0）：`entry["drive_mode"] == 1`
    把 True 当 1 用时取反被错误施加（obs 方向 q 偏移、action 方向 normalized
    符号翻转）—— 静默错误值比崩溃更危险；None 被当 0 静默放行。观测值
    取自合法 metadata 的 adapter 逆换算（T-B3 全环路已验证值域），只破坏
    metadata 的 drive_mode。当前实现两方向均静默返回 —— RED。
    """
    adapter = make_adapter()
    valid_metadata = _make_arm_metadata("left", T_B3_SPECS["left"])
    q = np.array(T_B3_Q_DEG["left"], dtype=np.float64)
    observation = dict(adapter.kinematic_deg_to_action("left", q, valid_metadata))

    specs = {joint: dict(entry) for joint, entry in T_B3_SPECS["left"].items()}
    specs[_T_B7_RANGE_PROBE_JOINT] = {
        **specs[_T_B7_RANGE_PROBE_JOINT],
        "drive_mode": bad_value,
    }
    corrupted_metadata = _make_arm_metadata("left", specs)

    with pytest.raises(ValueError) as excinfo:
        if direction == "obs":
            adapter.observation_to_kinematic_deg("left", observation, corrupted_metadata)
        else:
            adapter.kinematic_deg_to_action("left", q, corrupted_metadata)

    _t_b7_assert_named(excinfo, side="left", joint=_T_B7_RANGE_PROBE_JOINT, field="drive_mode")


_T_B7_SCHEMA_VARIANTS = (
    ("missing", "pop"),
    ("v1", "set_1"),
    ("string_two", "set_str_2"),
)


def _t_b7_mutate_schema_version(metadata: dict, variant: str) -> None:
    if variant == "pop":
        metadata.pop("schema_version")
    elif variant == "set_1":
        metadata["schema_version"] = 1
    else:
        metadata["schema_version"] = "2"


@pytest.mark.parametrize("direction", ["obs", "action"], ids=["obs_dir", "action_dir"])
@pytest.mark.parametrize(["variant_id", "variant"], _T_B7_SCHEMA_VARIANTS)
def test_t_b7_d_schema_version_missing_or_not_two_rejected(
    variant_id: str, variant: str, direction: str
) -> None:
    """T-B7.c：schema_version 缺失或非 2（含字符串）时双方向拒绝。

    T-B7.c（§9.1 不能降级使能）：schema_version 缺失 / =1（v1）/ ="2"
    （字符串宽松比较击杀样本）→ 双方向均报 ValueError。

    「收下 v1 metadata」与 `str(sv) == "2"` 式宽松比较都被本组拒绝；当前
    实现完全不读 schema_version，全部变体静默放行 —— RED。envelope 级
    错误无 joint 语义，只断言 ValueError 类型（模块统一错误契约）。
    """
    metadata = make_metadata(_t_b7_left_degrees_motors())
    _t_b7_mutate_schema_version(metadata, variant)
    adapter = make_adapter()

    with pytest.raises(ValueError):
        _t_b7_run(direction, adapter, "left", metadata)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("joint", JOINT_ORDER)
def test_t_b7_e_missing_observation_pos_key_rejected_without_zero_fill(
    joint: str, side: str
) -> None:
    """T-B7.d：obs 缺一个 .pos 键必须报错，不得零值填补。

    T-B7.d：obs 缺一个 `.pos` 键 → 报 ValueError 且点名缺失键，不得
    零值填补（协议层已有同型契约；本组在 adapter 公共入口显式锁定）。

    零值填补会把关节静默钉在注入 zero 位置 —— 实机上是不可见的错误姿态。
    当前实现已逐键检查并点名缺失键：预期 PASS（回归锁定）。
    """
    adapter = make_adapter()
    motor_deg = LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG
    observation = make_arm_observation(side, motor_deg)
    key = f"arm_{side}_{joint}.pos"
    del observation[key]
    metadata = make_metadata(make_arm_motors(side))

    with pytest.raises(ValueError) as excinfo:
        adapter.observation_to_kinematic_deg(side, observation, metadata)

    message = str(excinfo.value)
    assert joint in message, (
        f"missing-key rejection must name the joint {joint!r}; got: {message!r}"
    )
    assert key in message, f"missing-key rejection must name the key {key!r}; got: {message!r}"


@pytest.mark.parametrize("direction", ["obs", "action"], ids=["obs_dir", "action_dir"])
def test_t_b7_f_unsupported_normalization_value_rejected(direction: str) -> None:
    """T-B7.e：不支持的 normalization 值必须报错并点名关节与取值。

    T-B7.e：normalization = "range_0_100"（真实夹爪在用的合法 bus 值、
    kinematic 关节不支持）→ 报 ValueError 且含该值与关节名。

    选该值击杀「把夹爪 normalization 误用于关节换算」的实现。当前
    _motor_entry 已拒绝：预期 PASS（回归锁定，显式化 T-B3 隐含契约）。
    """
    motors = _t_b7_left_degrees_motors()
    motors[f"arm_left_{T_B7_PROBE_JOINT}"]["normalization"] = "range_0_100"
    metadata = make_metadata(motors)
    adapter = make_adapter()

    with pytest.raises(ValueError) as excinfo:
        _t_b7_run(direction, adapter, "left", metadata)

    message = str(excinfo.value)
    assert T_B7_PROBE_JOINT in message, (
        f"unsupported-normalization rejection must name the joint; got: {message!r}"
    )
    assert "range_0_100" in message, (
        f"unsupported-normalization rejection must name the value; got: {message!r}"
    )


def test_t_b7_g1_extra_motor_keys_ignored_both_directions() -> None:
    """T-B7.f：多余电机条目（夹爪/底轮/升降）双方向被忽略。

    T-B7.f：metadata 含 gripper / base 轮 / lift 多余条目、obs 含干扰
    键 → 双方向照常成功，结果与干净 metadata 逐位一致。

    实机 bus 必含夹爪：多余条目被忽略（不读取、不校验、不泄漏）。「读取
    多余条目」或「拒绝多余键」的实现被逐位等价断言击杀。当前实现已满足：
    预期 PASS（回归锁定，显式化裁决）。
    """
    adapter = make_adapter()
    q = np.array(LEFT_Q_DEG, dtype=np.float64)
    clean_metadata = make_metadata(_t_b7_left_degrees_motors())
    obs_reference = adapter.observation_to_kinematic_deg(
        "left", make_arm_observation("left", LEFT_MOTOR_DEG), clean_metadata
    )
    action_reference = adapter.kinematic_deg_to_action("left", q, clean_metadata)

    extras_metadata = _make_arm_metadata("left", LEFT_MOTOR_SPECS)
    extras_observation = {**make_arm_observation("left", LEFT_MOTOR_DEG), **INTERFERENCE_OBS_DEG}
    obs_with_extras = adapter.observation_to_kinematic_deg(
        "left", extras_observation, extras_metadata
    )
    action_with_extras = adapter.kinematic_deg_to_action("left", q, extras_metadata)

    assert np.array_equal(obs_with_extras, obs_reference), (
        "extra motor keys must not affect the observation conversion"
    )
    assert set(action_with_extras) == {f"arm_left_{joint}.pos" for joint in JOINT_ORDER}, (
        "action keys must stay exactly the six required joints"
    )
    assert action_with_extras == action_reference, (
        "extra motor keys must not affect the action conversion"
    )


def test_t_b7_g2_incomplete_extra_motor_entry_still_ignored() -> None:
    """T-B7.f（范围界定）：字段不全的多余条目仍被忽略。

    T-B7.f（范围界定）：夹爪条目缺 range_min / resolution / homing_offset
    （宿主层跳过未标定电机后的合法形态）→ 双方向照常成功且结果不变。

    校验只要求六必需关节条目完整；蔓延到全部条目的实现会拒绝实机合法
    metadata 形态，被本组击杀。当前实现不读多余条目：预期 PASS（回归
    锁定）。
    """
    adapter = make_adapter()
    q = np.array(LEFT_Q_DEG, dtype=np.float64)
    complete_metadata = _make_arm_metadata("left", LEFT_MOTOR_SPECS)
    obs_reference = adapter.observation_to_kinematic_deg(
        "left", make_arm_observation("left", LEFT_MOTOR_DEG), complete_metadata
    )
    action_reference = adapter.kinematic_deg_to_action("left", q, complete_metadata)

    stripped_gripper = {
        key: value
        for key, value in GRIPPER_MOTOR_SPECS["left"].items()
        if key not in ("range_min", "resolution", "homing_offset")
    }
    incomplete_metadata = _make_arm_metadata("left", LEFT_MOTOR_SPECS)
    incomplete_metadata["motors"]["arm_left_gripper"] = stripped_gripper

    obs_result = adapter.observation_to_kinematic_deg(
        "left", make_arm_observation("left", LEFT_MOTOR_DEG), incomplete_metadata
    )
    action_result = adapter.kinematic_deg_to_action("left", q, incomplete_metadata)

    assert np.array_equal(obs_result, obs_reference), (
        "an incomplete EXTRA motor entry must not affect the observation conversion"
    )
    assert action_result == action_reference, (
        "an incomplete EXTRA motor entry must not affect the action conversion"
    )


@pytest.mark.parametrize("bad_side", ["center", "LEFT"], ids=["unknown_word", "wrong_case"])
def test_t_b7_h_obs_direction_invalid_side_rejected_with_value(bad_side: str) -> None:
    """T-B7.g：obs 方向 side 非法时拒绝并点名该值。

    T-B7.g：obs 方向 side 非 "left"/"right" → 报 ValueError 且含该值
    （T-B2.7 已覆盖 action 方向，本组补 obs 方向）。

    "LEFT" 变体击杀 side.lower() 式大小写不敏感实现。当前实现已拒绝并
    点名：预期 PASS（回归锁定）。
    """
    adapter = make_adapter()
    observation = make_arm_observation("left", LEFT_MOTOR_DEG)
    metadata = make_metadata(_t_b7_left_degrees_motors())

    with pytest.raises(ValueError) as excinfo:
        adapter.observation_to_kinematic_deg(bad_side, observation, metadata)

    assert bad_side in str(excinfo.value), (
        f"invalid-side rejection must name the value {bad_side!r}; got: {excinfo.value!r}"
    )


# ======================================================================
# Behavior T-B8 (P1-B 单位适配, design §8): 左右臂隔离。
#
# 裁决契约（主 Agent 给定的当前行为；production 只读）：
#   a) 观测同时含双臂 `.pos` 键（另一臂键为巨大干扰值）时，每侧换算与
#      「仅本侧观测」结果逐位相同。DEGREES 与 RANGE 两种 metadata 模式
#      都验证：DEGREES 锚定冻结字面量；RANGE 模式下两侧标定窗口 /
#      分辨率 / drive_mode 互不相同，额外击杀 metadata 层的模糊条目
#      匹配（DEGREES 分支不消费标定字段，该形态只有 RANGE 能判别）。
#   b) 左右 sign/zero 配置不同 → 同一 q 向量在两侧调用产生各自设备键
#      （arm_left_* / arm_right_*）与各自值（逐关节互异）。
#   b2) 左右 URDF 限位不同 → 同一 q 一侧通过、另一侧被整体拒绝（错误
#       含 side 与关节名）。击杀限位按关节共享、不按 side 存取的实现。
#   c) 仅含一侧 metadata 条目时，另一侧请求（obs/action）报 ValueError
#      且含 side 名与缺失电机前缀。击杀「另一臂条目缺失时用本臂条目
#      兜底换算」的静默错误值。
#   d) 另一臂 metadata 条目损坏（空条目 / 非法数值 / 非法 normalization /
#      条目为 None）不影响本侧换算——实现不隐式读另一臂，校验层也不得
#      蔓延到另一臂条目（T-B7.f 多余条目先例延伸到对侧必需条目）。
#
# 要击杀的合理错误实现：
#   ① obs 键模糊匹配（`joint in key` / endswith——arm_right_shoulder_pan.pos
#      同样「含」shoulder_pan）→ 本侧换算混入另一臂巨大值（取首个匹配
#      被对侧用例击杀，取末个匹配被本侧用例击杀）；
#   ② 左右共享可变配置（右侧误用左侧 sign/zero/限位注入）→ 同 q 两侧
#      输出相同值/相同键，或限位判定错侧；
#   ③ metadata 兜底（对侧条目缺失 → 用本臂条目）→ 静默产出错误值而非
#      点名 side 报错；
#   ④ 校验层蔓延（对 metadata 全部条目执行 _validate_motor_entry）→
#      对侧条目损坏时本侧换算被连带拒绝或崩溃。
#
# 分层与 seam：纯对象 component 测试，复用 make_adapter / make_arm_observation /
# make_metadata / make_arm_motors / make_t_b3_metadata 与全部冻结常量。
# 无 bus / 网络 / 时钟 / 随机 / sleep。逐位比较用 np.array_equal / dict ==；
# 绝对值锚定沿用既有冻结字面量（DEGREES 模式）与 T-B3/T-B4 已独立验证
# 的 RANGE 往返值（仅作输入生成，不重复其锚定职责）。
# ======================================================================

# 巨大干扰值（有限、互异、量级远离正常关节值）：若实现模糊匹配取到另
# 一臂键，结果偏移 ~1e4–1e5 度，任何合理容差都遮不住。DEGREES 模式下
# 直接进入 q；RANGE 模式经 tick 量化后仍偏移 ~1e4 度。
T_B8_HUGE_ARM_DEG = {
    "left": {joint: 100_000.0 + 11.25 * index for index, joint in enumerate(JOINT_ORDER)},
    "right": {joint: 100_000.0 + 7.5 * index for index, joint in enumerate(JOINT_ORDER)},
}

# b2) 限位隔离场景：通过侧注宽窗、拒绝侧注窄窗，q 用通过侧的冻结向量。
# 两个冻结 q 都有多个关节越出窄窗 (-30, 30)，且全部落在宽窗内。
_T_B8_WIDE_WINDOW = (-1000.0, 1000.0)
_T_B8_NARROW_WINDOW = (-30.0, 30.0)
_T_B8_LIMIT_SCENARIOS = (
    ("left_within_right_beyond", "left", "right", LEFT_Q_DEG, LEFT_EXPECTED_MOTOR_DEG),
    ("right_within_left_beyond", "right", "left", RIGHT_Q_DEG, RIGHT_EXPECTED_MOTOR_DEG),
)

# d) 对侧条目损坏变体：空条目（全字段缺失）、非法数值（宽松强转/bool/
# inf 的 F-1d 样本）、非法 normalization 值、条目本身为 None。
_T_B8_CORRUPT_VARIANTS = (
    "empty_entries",
    "bad_numeric",
    "bad_normalization",
    "none_entries",
)


def _t_b8_corrupted_motors(side: str, variant: str) -> dict:
    """按变体构造单臂六条全损坏的 metadata 条目。

    One arm's six metadata entries, all corrupted per `variant`. Values are
    SYNTHETIC and follow the same F-1d poisoning samples as T-B7.b.
    """
    specs = LEFT_MOTOR_SPECS if side == "left" else RIGHT_MOTOR_SPECS
    motors = {}
    for joint in JOINT_ORDER:
        if variant == "empty_entries":
            entry = {}
        elif variant == "bad_numeric":
            entry = {
                **dict(specs[joint]),
                "resolution": "6399",
                "range_min": None,
                "range_max": float("inf"),
                "drive_mode": True,
            }
        elif variant == "bad_normalization":
            entry = {**dict(specs[joint]), "normalization": "range_0_100"}
        else:  # none_entries
            entry = None
        motors[f"arm_{side}_{joint}"] = entry
    return motors


def _t_b8_range_solo_obs(adapter: object, side: str) -> dict:
    """RANGE 模式本侧合法 normalized 观测（由冻结 q 逆换算）。

    RANGE 模式下本侧合法 normalized 观测：由 T-B3 冻结 q 经 adapter
    逆换算得到（T-B3/T-B4 已独立锚定该方向；此处仅作合法输入生成）。
    """
    q = np.array(T_B3_Q_DEG[side], dtype=np.float64)
    return dict(adapter.kinematic_deg_to_action(side, q, make_t_b3_metadata()))


@pytest.mark.parametrize("mode", ["degrees", "range"], ids=["degrees_mode", "range_mode"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b8_a_obs_both_arms_keys_bitwise_identical_to_solo_observation(
    side: str, mode: str
) -> None:
    """T-B8.a：obs 同时含双臂键（另一臂为巨大干扰值）→ 每侧换算与「仅本 侧观测」逐位相同。

    DEGREES 模式额外锚定冻结字面量；RANGE 模式两侧 metadata 标定互异，
    击杀 metadata 模糊条目匹配。取首匹配 / 取末匹配 / 求平均的模糊键查找
    分别被对侧/本侧/双侧用例击杀。当前实现逐键精确查找本侧键：预期
    PASS（回归锁定）。
    """
    adapter = make_adapter()
    other = "right" if side == "left" else "left"
    solo_obs = make_arm_observation(side, LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG)
    huge_other_obs = {
        f"arm_{other}_{joint}.pos": T_B8_HUGE_ARM_DEG[other][joint] for joint in JOINT_ORDER
    }
    # 两种插入顺序都验证：「取首个含关节名的键」的模糊匹配在本侧键在前时
    # 会碰巧逃逸，只有两种顺序都逐位一致才能击杀任意匹配选择策略。
    both_orders = (
        {**solo_obs, **huge_other_obs},
        {**huge_other_obs, **solo_obs},
    )
    if mode == "degrees":
        metadata = make_metadata({**make_arm_motors("left"), **make_arm_motors("right")})
        frozen = LEFT_EXPECTED_KINEMATIC_DEG if side == "left" else RIGHT_EXPECTED_KINEMATIC_DEG
    else:
        metadata = make_t_b3_metadata()
        frozen = None

    reference = adapter.observation_to_kinematic_deg(side, solo_obs, metadata)

    assert np.all(np.isfinite(reference)), (
        f"T-B8.a 前提失效：{side} solo 参考结果必须有限; got {reference!r}"
    )
    for order_index, both_obs in enumerate(both_orders):
        result = adapter.observation_to_kinematic_deg(side, both_obs, metadata)
        assert np.array_equal(result, reference), (
            f"T-B8.a: 加入另一臂巨大干扰键后（插入顺序 {order_index}）{side} 换算"
            f"必须与仅本侧观测逐位相同; got diff {result - reference!r}"
        )
        if frozen is not None:
            assert result == pytest.approx(list(frozen), abs=1e-9), (
                f"T-B8.a: {side} 换算必须等于冻结字面量 {frozen!r}; got {result!r}"
            )


@pytest.mark.parametrize("mode", ["degrees", "range"], ids=["degrees_mode", "range_mode"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b8_a_range_obs_solo_observation_is_valid_normalized_input(side: str, mode: str) -> None:
    """T-B8.a 前提锚定：solo 观测换算结果必须有限且形状正确。

    T-B8.a 前提锚定（RANGE 半侧的独立断言形态）：RANGE solo 观测必须
    与本侧干净 metadata 的换算结果有限且形状正确，确保 _t_b8_range_solo_obs
    生成的输入合法，T-B8.a 的逐位比较不是空转。

    DEGREES 模式沿用冻结字面量重申；RANGE 模式仅断言有限性与形状（绝对
    值由 T-B3/T-B4 锚定）。当前实现满足：预期 PASS（回归锁定）。
    """
    adapter = make_adapter()
    if mode == "degrees":
        solo_obs = make_arm_observation(side, LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG)
        metadata = make_metadata(make_arm_motors(side))
    else:
        solo_obs = _t_b8_range_solo_obs(adapter, side)
        metadata = make_t_b3_metadata()

    result = adapter.observation_to_kinematic_deg(side, solo_obs, metadata)

    assert result.shape == (6,)
    assert np.all(np.isfinite(result)), (
        f"T-B8.a 前提失效：{side} RANGE/DEGREES solo 换算必须有限; got {result!r}"
    )


def test_t_b8_b_same_q_both_sides_produce_own_device_keys_and_values() -> None:
    """T-B8.b：同一 q 向量分别以 left/right 调用 → 各自设备键与各自值 （逐关节互异）。

    右侧误用左侧 sign/zero（共享/别名配置）会把两侧值变成相同或使右侧
    期望不匹配，被逐关节互异断言与冻结式断言击杀；键前缀断言击杀键名
    不随 side 切换的实现。当前实现按 side 注入逐位满足：预期 PASS
    （回归锁定）。
    """
    adapter = make_adapter()
    metadata = make_metadata({**make_arm_motors("left"), **make_arm_motors("right")})
    q = np.array(LEFT_Q_DEG, dtype=np.float64)

    action_left = adapter.kinematic_deg_to_action("left", q, metadata)
    action_right = adapter.kinematic_deg_to_action("right", q, metadata)

    assert set(action_left) == {f"arm_left_{joint}.pos" for joint in JOINT_ORDER}, (
        f"T-B8.b: left 输出键必须恰为 arm_left_* 六键; got {sorted(action_left)!r}"
    )
    assert set(action_right) == {f"arm_right_{joint}.pos" for joint in JOINT_ORDER}, (
        f"T-B8.b: right 输出键必须恰为 arm_right_* 六键; got {sorted(action_right)!r}"
    )
    for index, joint in enumerate(JOINT_ORDER):
        q_value = LEFT_Q_DEG[index]
        expected_left = KINEMATIC_SIGNS["left"][joint] * (q_value - KINEMATIC_ZEROS["left"][joint])
        expected_right = KINEMATIC_SIGNS["right"][joint] * (
            q_value - KINEMATIC_ZEROS["right"][joint]
        )
        left_value = action_left[f"arm_left_{joint}.pos"]
        right_value = action_right[f"arm_right_{joint}.pos"]
        assert left_value == pytest.approx(expected_left, abs=1e-9), (
            f"T-B8.b: left {joint} 期望 {expected_left!r}; got {left_value!r}"
        )
        assert right_value == pytest.approx(expected_right, abs=1e-9), (
            f"T-B8.b: right {joint} 期望 {expected_right!r}; got {right_value!r}"
        )
        assert left_value != right_value, (
            f"T-B8.b: 同一 q 下左右 {joint} 值相同（{left_value!r}）—— 左右配置被共享/混用"
        )


@pytest.mark.parametrize(
    ["scenario_id", "pass_side", "beyond_side", "q_frozen", "frozen_expected"],
    _T_B8_LIMIT_SCENARIOS,
    ids=[scenario[0] for scenario in _T_B8_LIMIT_SCENARIOS],
)
def test_t_b8_b2_urdf_limits_isolated_per_side_same_q_one_side_rejected(
    scenario_id: str,
    pass_side: str,
    beyond_side: str,
    q_frozen: tuple,
    frozen_expected: tuple,
) -> None:
    """T-B8.b2：左右 URDF 限位互不串扰，同一 q 一侧通过一侧被拒。

    T-B8.b2：左右 URDF 限位不同 → 同一 q 通过侧照常导出（冻结期望），
    拒绝侧整体 ValueError 且含 side 与关节名。

    限位按关节共享（不按 side 存取）的实现会让通过侧被窄窗拒绝、拒绝侧
    被宽窗放行，两个断言方向都被击杀。当前实现按 side 存取限位：预期
    PASS（回归锁定）。
    """
    adapter = make_adapter(
        urdf_limits={
            pass_side: dict.fromkeys(JOINT_ORDER, _T_B8_WIDE_WINDOW),
            beyond_side: dict.fromkeys(JOINT_ORDER, _T_B8_NARROW_WINDOW),
        }
    )
    metadata = make_metadata({**make_arm_motors("left"), **make_arm_motors("right")})
    q = np.array(q_frozen, dtype=np.float64)

    action = adapter.kinematic_deg_to_action(pass_side, q, metadata)
    for index, joint in enumerate(JOINT_ORDER):
        assert action[f"arm_{pass_side}_{joint}.pos"] == pytest.approx(
            frozen_expected[index], abs=1e-9
        ), (
            f"T-B8.b2: pass 侧 {pass_side} {joint} 期望 {frozen_expected[index]!r}; "
            f"got {action[f'arm_{pass_side}_{joint}.pos']!r}"
        )

    with pytest.raises(ValueError) as excinfo:
        adapter.kinematic_deg_to_action(beyond_side, q, metadata)
    message = str(excinfo.value)
    assert beyond_side in message, (
        f"T-B8.b2: 被拒侧报错必须含 side {beyond_side!r}; got {message!r}"
    )
    assert any(joint in message for joint in JOINT_ORDER), (
        f"T-B8.b2: 被拒侧报错必须点名关节; got {message!r}"
    )


@pytest.mark.parametrize("direction", ["obs", "action"], ids=["obs_dir", "action_dir"])
@pytest.mark.parametrize("requested_side", ["left", "right"])
def test_t_b8_c_missing_side_metadata_rejected_with_side_name(
    requested_side: str, direction: str
) -> None:
    """T-B8.c：仅含另一侧 metadata 条目时，本侧请求 → ValueError 且含 side 名与缺失电机前缀。

    「对侧条目缺失 → 用本臂条目兜底」的实现会静默产出错误值而非报错，
    被 raises 断言击杀（obs 方向兜底键模糊匹配同样被击杀）。当前实现
    _motor_entry 按精确名查找并点名缺失：预期 PASS（回归锁定）。
    """
    adapter = make_adapter()
    owner_side = "right" if requested_side == "left" else "left"
    metadata = make_metadata(make_arm_motors(owner_side))

    if direction == "obs":
        observation = make_arm_observation(
            requested_side,
            LEFT_MOTOR_DEG if requested_side == "left" else RIGHT_MOTOR_DEG,
        )
        with pytest.raises(ValueError) as excinfo:
            adapter.observation_to_kinematic_deg(requested_side, observation, metadata)
    else:
        q = np.array(LEFT_Q_DEG if requested_side == "left" else RIGHT_Q_DEG, dtype=np.float64)
        with pytest.raises(ValueError) as excinfo:
            adapter.kinematic_deg_to_action(requested_side, q, metadata)

    message = str(excinfo.value)
    assert requested_side in message, (
        f"T-B8.c: 拒绝消息必须含 side {requested_side!r}; got {message!r}"
    )
    assert f"arm_{requested_side}" in message, (
        f"T-B8.c: 拒绝消息必须含缺失电机前缀 arm_{requested_side!r}; got {message!r}"
    )


@pytest.mark.parametrize("direction", ["obs", "action"], ids=["obs_dir", "action_dir"])
@pytest.mark.parametrize("variant", _T_B8_CORRUPT_VARIANTS)
@pytest.mark.parametrize("clean_side", ["left", "right"])
def test_t_b8_d_corrupt_other_side_entries_do_not_affect_clean_side(
    clean_side: str, variant: str, direction: str
) -> None:
    """T-B8.d：对侧条目损坏不影响本侧换算（逐位相同）。

    T-B8.d：另一臂全部六条目损坏（空 / 非法数值 / 非法 normalization /
    None）→ 本侧 obs（含双臂键）与 action 换算与干净基线逐位相同。

    「校验层蔓延到全部条目」的实现会对损坏条目报错/崩溃（T-B7.f 裁决：
    校验不得蔓延），被本组击杀；「隐式读另一臂条目/obs 键」的实现产出
    偏移值或异常，同样被逐位等价断言击杀。当前实现只查本侧条目：预期
    PASS（回归锁定）。
    """
    adapter = make_adapter()
    other_side = "right" if clean_side == "left" else "left"
    clean_metadata = make_metadata({**make_arm_motors("left"), **make_arm_motors("right")})
    solo_obs = make_arm_observation(
        clean_side, LEFT_MOTOR_DEG if clean_side == "left" else RIGHT_MOTOR_DEG
    )
    other_obs = make_arm_observation(
        other_side, LEFT_MOTOR_DEG if other_side == "left" else RIGHT_MOTOR_DEG
    )
    both_obs = {**solo_obs, **other_obs}
    q = np.array(LEFT_Q_DEG if clean_side == "left" else RIGHT_Q_DEG, dtype=np.float64)

    if direction == "obs":
        reference = adapter.observation_to_kinematic_deg(clean_side, solo_obs, clean_metadata)
    else:
        reference = adapter.kinematic_deg_to_action(clean_side, q, clean_metadata)

    corrupted_metadata = make_metadata(
        {
            **make_arm_motors("left"),
            **make_arm_motors("right"),
            **_t_b8_corrupted_motors(other_side, variant),
        }
    )

    if direction == "obs":
        result = adapter.observation_to_kinematic_deg(clean_side, both_obs, corrupted_metadata)
        assert np.array_equal(result, reference), (
            f"T-B8.d: 对侧条目损坏（{variant}）不得影响 {clean_side} obs 换算; "
            f"got diff {result - reference!r}"
        )
    else:
        result = adapter.kinematic_deg_to_action(clean_side, q, corrupted_metadata)
        assert result == reference, (
            f"T-B8.d: 对侧条目损坏（{variant}）不得影响 {clean_side} action 换算; "
            f"got {result!r} vs reference {reference!r}"
        )


# ======================================================================
# Behavior T-B9 (P1-B 单位适配): 输入不可变 + 返回类型契约。
#
# T-B9.1 输入不可变: observation_to_kinematic_deg / kinematic_deg_to_action
#   是纯转换——调用后 observation、q_deg、metadata 必须与各自深拷贝逐位
#   相同（键集合、值类型、值位模式全不变）。覆盖 DEGREES 与 RANGE_M100_100
#   两种 normalization × 左右两侧。击杀「range 方向把 normalized 观测就地
#   unnormalize 写回输入 dict」「action 方向复用 np.asarray 视图就地改写
#   q_deg」「校验/规范化写回 metadata」类实现：生产循环反复携带同一
#   observation / q_deg / metadata 状态，就地污染会被下一轮读取放大。
#
# T-B9.2 返回类型契约: obs 方向返回 np.ndarray 且 shape (6,)（浮点 dtype）；
#   action 方向返回 dict、恰六个 arm_{side}_{joint}.pos 键、值全为 Python
#   float。np.float64 是 float 的子类，isinstance(v, float) 无法区分，故
#   必须用 type(v) is float 断言——击杀「跳过 float() 收敛、让 numpy 标量
#   漏进 action dict」的实现（下游 dict 序列化 / JSON / 总线写值路径会
#   因此改变行为）。
#
# T-B9.3 返回值互不别名: 连续多次调用，obs 的 ndarray、action 的 dict 均
#   不得是同一对象；污染上一次返回值不得影响后续调用。击杀「复用内部
#   缓冲 / 缓存 dict」类性能优化——调用者持有一份结果时，下一次换算会
#   静默改写它（np.array_equal 之类的值断言对同一对象恒真，只有对象身份
#   检查能区分）。
#
# 当前实现输入只读、每次新建返回容器、action 值经 float() 收敛：预期
# 全部 PASS（回归锁定）；杀伤力由 /tmp shadow 变异运行独立证实。
# ======================================================================


def _t_b9_bitwise_unchanged(container: object, snapshot: object, path: str) -> None:
    """递归断言容器与其深拷贝快照逐位相同。

    dict 比较键集合并递归；标量先比类型（int 不得漂移成 float、
    np.float64 不得伪装成 float）；float 再比 64 位位模式（.hex() 覆盖
    ±0 与 NaN 的位差异）；其余标量比 ==。
    """
    if isinstance(container, dict):
        assert set(container) == set(snapshot), (
            f"T-B9: {path} 键集合被修改: "
            f"added={sorted(set(container) - set(snapshot))!r} "
            f"removed={sorted(set(snapshot) - set(container))!r}"
        )
        for key in snapshot:
            _t_b9_bitwise_unchanged(container[key], snapshot[key], f"{path}[{key!r}]")
        return
    assert type(container) is type(snapshot), (
        f"T-B9: {path} 值类型被改写: {type(container).__name__} != "
        f"{type(snapshot).__name__} (value={container!r})"
    )
    if isinstance(snapshot, float):
        assert container.hex() == snapshot.hex(), (
            f"T-B9: {path} 浮点值位模式被修改: {container!r} != {snapshot!r}"
        )
    else:
        assert container == snapshot, f"T-B9: {path} 值被修改: {container!r} != {snapshot!r}"


def _t_b9_range_obs(adapter: object, side: str) -> dict[str, float]:
    """RANGE 模式合法 normalized 观测（由 T-B3 冻结 q 逆换算生成）。

    RANGE 模式合法 normalized 观测（值域 [-100, 100]）：由 T-B3 冻结 q
    经 adapter 逆换算生成——该方向已由 T-B3/T-B4 冻结中间值独立锚定，
    此处仅作合法输入生成（与 _t_b8_range_solo_obs 同一既定模式）。
    """
    q = np.array(T_B3_Q_DEG[side], dtype=np.float64)
    return dict(adapter.kinematic_deg_to_action(side, q, make_t_b3_metadata()))


def _t_b9_inputs(mode: str, side: str, adapter: object) -> tuple[dict, dict, np.ndarray]:
    """T-B9 输入三元组 (observation, metadata, q_deg)，按 mode 选取。

    DEGREES：T-B1/T-B2 冻结字面量输入；RANGE：T-B3 冻结标定下的合法
    normalized 观测 + T-B3 冻结 q_deg。
    """
    if mode == "degrees":
        observation = make_arm_observation(
            side, LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG
        )
        metadata = make_metadata(make_arm_motors(side))
        q_deg = np.array(LEFT_Q_DEG if side == "left" else RIGHT_Q_DEG, dtype=np.float64)
    else:
        observation = _t_b9_range_obs(adapter, side)
        metadata = make_t_b3_metadata()
        q_deg = np.array(T_B3_Q_DEG[side], dtype=np.float64)
    return observation, metadata, q_deg


@pytest.mark.parametrize("mode", ["degrees", "range"], ids=["degrees_mode", "range_mode"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b9_1_inputs_bitwise_unchanged_after_calls(side: str, mode: str) -> None:
    """T-B9.1：调用后 observation / q_deg / metadata 逐位不变。

    T-B9.1：obs / action 两个方向调用后，observation、q_deg、metadata
    与各自深拷贝逐位相同（键集合、值类型、值位模式全不变）。

    DEGREES 与 RANGE_M100_100 两种 normalization 都覆盖：RANGE 方向存在
    tick 往返中间步骤，是「就地 unnormalize 写回输入」类省内存实现最可能
    出现的位置。就地污染会放大到生产循环的下一轮读取。当前实现输入只读：
    预期 PASS（回归锁定）。
    """
    adapter = make_adapter()
    observation, metadata, q_deg = _t_b9_inputs(mode, side, adapter)
    obs_snapshot = copy.deepcopy(observation)
    metadata_snapshot = copy.deepcopy(metadata)
    q_snapshot = q_deg.copy()

    adapter.observation_to_kinematic_deg(side, observation, metadata)
    _t_b9_bitwise_unchanged(observation, obs_snapshot, "observation")
    _t_b9_bitwise_unchanged(metadata, metadata_snapshot, "metadata")

    adapter.kinematic_deg_to_action(side, q_deg, metadata)
    _t_b9_bitwise_unchanged(metadata, metadata_snapshot, "metadata")
    assert q_deg.dtype == q_snapshot.dtype and q_deg.shape == q_snapshot.shape
    assert q_deg.tobytes() == q_snapshot.tobytes(), (
        "T-B9: q_deg 输入被就地修改（asarray 视图写回 / 就地归一化类实现）; "
        f"before={q_snapshot!r} after={q_deg!r} (side={side!r}, mode={mode!r})"
    )


@pytest.mark.parametrize("mode", ["degrees", "range"], ids=["degrees_mode", "range_mode"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b9_2_return_type_contract(side: str, mode: str) -> None:
    """T-B9.2：obs 返回 ndarray、action 返回 dict 且值为 Python float。

    T-B9.2：obs 方向返回 np.ndarray、shape (6,)、浮点 dtype；action 方向
    返回 dict、恰六个 arm_{side}_{joint}.pos 键、值全为 Python float。

    np.float64 是 float 的子类，isinstance(v, float) 无法区分，必须
    type(v) is float：击杀「跳过 float() 收敛、让 numpy 标量漏进 action」
    的实现。值正确性由 T-B1/T-B2/T-B3 锚定，此处只锁定类型/形状契约。
    当前实现满足：预期 PASS（回归锁定）。
    """
    adapter = make_adapter()
    observation, metadata, q_deg = _t_b9_inputs(mode, side, adapter)

    q_vec = adapter.observation_to_kinematic_deg(side, observation, metadata)
    assert isinstance(q_vec, np.ndarray), (
        f"T-B9: obs 方向必须返回 np.ndarray, got {type(q_vec).__name__}"
    )
    assert q_vec.shape == (6,), f"T-B9: obs 方向返回 shape {q_vec.shape}, 期望 (6,)"
    assert np.issubdtype(q_vec.dtype, np.floating), (
        f"T-B9: obs 方向返回 dtype {q_vec.dtype}, 期望浮点"
    )

    action = adapter.kinematic_deg_to_action(side, q_deg, metadata)
    assert isinstance(action, dict), f"T-B9: action 方向必须返回 dict, got {type(action).__name__}"
    expected_keys = {f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER}
    assert set(action) == expected_keys, (
        f"T-B9: action 键集合 {sorted(action)!r} != 期望 {sorted(expected_keys)!r}"
    )
    for key, value in action.items():
        assert type(value) is float, (
            f"T-B9: action[{key!r}] 必须是 Python float (type is float), "
            f"got {type(value).__name__} value={value!r} (side={side!r}, mode={mode!r})"
        )
        assert not isinstance(value, np.generic), (
            f"T-B9: action[{key!r}] 不得是 numpy 标量: {value!r}"
        )


@pytest.mark.parametrize("mode", ["degrees", "range"], ids=["degrees_mode", "range_mode"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b9_3_repeated_calls_do_not_alias(side: str, mode: str) -> None:
    """T-B9.3：连续调用返回值互不别名，污染不泄漏到后续调用。

    T-B9.3：连续多次调用，obs 的 ndarray 与 action 的 dict 互不别名；
    污染上一次返回值不得影响后续调用结果。

    「复用内部缓冲 / 缓存 dict」的换算在调用者持有上一次结果时，会被
    下一次调用静默改写；值断言对同一对象恒真，只有对象身份检查能区分。
    DEGREES 模式额外锚定 T-B1/T-B2 冻结字面量，证明污染未泄漏进值。
    当前实现每次新建返回容器：预期 PASS（回归锁定）。
    """
    adapter = make_adapter()
    observation, metadata, q_deg = _t_b9_inputs(mode, side, adapter)

    out1 = adapter.observation_to_kinematic_deg(side, observation, metadata)
    out2 = adapter.observation_to_kinematic_deg(side, observation, metadata)
    assert out1 is not out2, "T-B9: obs 方向连续调用不得复用同一内部缓冲（返回值互不别名）"
    out1[0] = 999.0  # 污染第一次返回值，验证不泄漏
    out3 = adapter.observation_to_kinematic_deg(side, observation, metadata)
    assert out2 is not out3, "T-B9: obs 方向第二次与第三次调用返回值互不别名"
    assert np.array_equal(out2, out3), "T-B9: 第一次 obs 返回值的污染不得影响后续调用结果"
    if mode == "degrees":
        expected = LEFT_EXPECTED_KINEMATIC_DEG if side == "left" else RIGHT_EXPECTED_KINEMATIC_DEG
        assert out3 == pytest.approx(list(expected), abs=1e-9)

    d1 = adapter.kinematic_deg_to_action(side, q_deg, metadata)
    d2 = adapter.kinematic_deg_to_action(side, q_deg, metadata)
    assert d1 is not d2, "T-B9: action 方向连续调用不得复用同一内部 dict（返回值互不别名）"
    first_key = f"arm_{side}_{JOINT_ORDER[0]}.pos"
    d1[first_key] = 999.0  # 污染第一次返回值，验证不泄漏
    d3 = adapter.kinematic_deg_to_action(side, q_deg, metadata)
    assert d2 is not d3, "T-B9: action 方向第二次与第三次调用返回值互不别名"
    assert d2 == d3, "T-B9: 第一次 action 返回值的污染不得影响后续调用结果"
    assert d3[first_key] != 999.0
    if mode == "degrees":
        expected_action = LEFT_EXPECTED_MOTOR_DEG if side == "left" else RIGHT_EXPECTED_MOTOR_DEG
        for joint, exp in zip(JOINT_ORDER, expected_action, strict=False):
            assert d3[f"arm_{side}_{joint}.pos"] == pytest.approx(exp, abs=1e-9)


# ======================================================================
# Behavior T-B10 (P1-B, design §19): URDF 关节名显式映射契约面。
#
# design §19 原文: 官方 Pro 源模型对应名称为 left_shoulder_pan、
#   left_shoulder_lift、left_elbow_flex、left_wrist_flex、
#   left_wrist_yaw_joint、left_wrist_roll；「必须显式映射 wrist_yaw 与
#   left_wrist_yaw_joint，不能仅拼接前缀猜名」；共用单臂源模型时右臂设备
#   键仍是 arm_right_*，模型内部可保持 left_* 名称。
#
# 冻结映射值的来源核对（本区块的权威锚点）:
# - wrist_yaw -> left_wrist_yaw_joint: design §19 原文显式给定（主锚点，
#   _joint 后缀是全部六名中唯一的形态差异）。
# - 其余五名: design §19 原文给定，并与未验证参考 URDF 逐一交叉核对一致:
#   docs/alohamini-control/reference/urdf-sources/am-arm200pro.urdf（Pro 单臂优先
#   候选, commit 5b03af1e..., sha256 c38669d4...，八关节名逐一核对）、
#   am-arm200.urdf（普通版单臂, sha256 a8caac56...，六名一致）、
#   alohamini2pro.urdf（Pro 整机左臂链, commit 17c6a98d..., sha256
#   bd1b1e6d...，左臂链一致）。三文件 sha256 均与同目录 sources.json 记录
#   一致（2026-09-21 取得，未修改）。
# - 来源性质: sources.json 明确 hardware_verified=false、
#   placo_load_tested=false、meshes_downloaded=false——未验证参考，
#   不入运动 profile；冻结值以 design §19 原文 + 上游参考一致性为锚，
#   Gate A/B 实机证据若推翻，须修订设计并同步改本表，不得静默改值。
#
# 合理错误实现识别（本区块的杀伤目标）:
# - T-B10.1 无公开映射面: 映射藏死在转换函数内部 → 映射漂移不可测，
#   后续实现各处硬编码字符串；契约面必须可按 (side, joint) 公开查询，
#   且与注入的 signs/zeros 无关（模型名不是随臂配置变化的量）。
# - T-B10.2 f"left_{joint}" 拼名: wrist_yaw 会得 left_wrist_yaw（缺
#   _joint），冻结值断言直接暴露差异。
# - T-B10.3 f"right_{joint}" / f"{side}_{joint}" 侧前缀替换: right 侧
#   会得 right_shoulder_pan 等——共用单臂源模型里根本没有 right_* 关节
#   名；正确行为是两侧同表（模型内部保持 left_*）。
# - T-B10.4 拼名 + hardcode wrist_yaw 特判补丁: 行为等价但映射表不可
#   审计、违反「显式映射表」约束；静态源码守卫抓 f-string / 字符串拼接
#   构造 URDF 模型名前缀的运行时拼名（f"arm_{side}_{joint}" 设备键构造
#   是合法形态，不在被禁模式内）。该用例当前实现下预期 PASS（约束守卫，
#   性质同 T-B9「回归锁定」注释先例），不是 RED 用例。
#
# 公开 seam 的形状由实现方定（design §19 只要求显式、可查询）。测试绑定
# 形状白名单 discovery: 模块级/类级公开常量（joint->urdf 映射、
# side->joint->urdf 映射、或按冻结顺序的六名序列）或公开可调用/property
# （(side, joint) / (joint) / (side) / () 形状，候选名有限列举）。实现
# 选择白名单之外的名称/形状时测试 fail 并列出全部尝试——那是 binding 层
# 的重新对齐信号，不是静默适配的理由。
#
# Harness: 纯对象（复用 make_adapter 与 JOINT_ORDER），无 bus/时钟/网络/
# 随机性；映射值是字符串精确相等比较，无浮点。
# ======================================================================


# 冻结的 URDF 关节名映射（六关节，均为模型内部名称，左右侧共用——见
# design §19 共用单臂源模型条款）。来源标注见区块头注释。
T_B10_EXPECTED_URDF_NAMES = {
    "shoulder_pan": "left_shoulder_pan",
    "shoulder_lift": "left_shoulder_lift",
    "elbow_flex": "left_elbow_flex",
    "wrist_flex": "left_wrist_flex",
    # 主锚点: design §19 原文显式给定，禁止 f"left_{joint}" 拼名得到。
    "wrist_yaw": "left_wrist_yaw_joint",
    "wrist_roll": "left_wrist_roll",
}
_T_B10_JOINT_SET = frozenset(JOINT_ORDER)

# 公开常量候选名（有限白名单；映射面形状白名单见区块头注释）。
_T_B10_CONSTANT_CANDIDATES = (
    "URDF_JOINT_NAMES",
    "URDF_JOINT_NAME_MAP",
    "URDF_JOINT_NAME_MAPPING",
    "URDF_JOINT_NAME_TABLE",
    "URDF_JOINT_NAMES_BY_SIDE",
    "JOINT_TO_URDF_NAMES",
    "JOINT_TO_URDF_NAME",
    "KINEMATIC_TO_URDF_JOINT_NAMES",
    "KINEMATIC_TO_URDF_NAME",
    "URDF_JOINTS",
)

# 公开可调用 / property 候选名（有限白名单）。
_T_B10_CALLABLE_CANDIDATES = (
    "urdf_joint_name",
    "urdf_joint_name_for",
    "urdf_joint_names",
    "urdf_joint_name_map",
    "urdf_joint_name_mapping",
    "joint_to_urdf_name",
    "joint_urdf_name",
    "urdf_name_for_joint",
)

_MISSING = object()


def _t_b10_explain_mapping(value: object) -> tuple[dict, str] | None:
    """把候选值解释为 joint -> urdf-name 查询表；解释不了返回 None。

    接受三种白名单形状:
    - Mapping 以六关节名为键（side 无关单臂源模型）；
    - Mapping 以 side 为键、值为六关节完整 Mapping；
    - 长度 6 的字符串序列，按冻结 JOINT_ORDER 有序对应。
    值必须是非空 str。
    """
    from collections.abc import Mapping

    if isinstance(value, Mapping):
        keys = set(value)
        if keys >= _T_B10_JOINT_SET:
            sub, shape = value, "joint->urdf mapping (side-agnostic)"
        elif keys and keys <= {"left", "right"}:
            first = next(iter(value.values()))
            if not isinstance(first, Mapping) or not (set(first) >= _T_B10_JOINT_SET):
                return None
            sub, shape = first, "side->joint->urdf mapping"
        else:
            return None
        if not all(isinstance(v, str) and v for v in sub.values()):
            return None
        return dict(sub), shape
    if (
        isinstance(value, (list, tuple))
        and len(value) == 6
        and all(isinstance(v, str) and v for v in value)
    ):
        return dict(zip(JOINT_ORDER, value, strict=False)), "ordered six-name sequence"
    return None


def _t_b10_accessor_from_callable(func: object) -> tuple[object, str] | None:
    """按公开可调用的签名形状构造 accessor(side, joint) -> str。

    白名单签名: (side, joint)->str、(joint)->str、(side)->per-side 映射、
    ()->全局映射。返回 (accessor, description) 或 (None, reason)。
    """
    try:
        sig = inspect.signature(func)
    except (TypeError, ValueError):
        return None, "signature not introspectable"
    params = [
        p
        for p in sig.parameters.values()
        if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD) and p.default is p.empty
    ]
    names = [p.name for p in params]
    if len(params) >= 2 and names[:2] == ["side", "joint"]:

        def acc(side: str, joint: str, _f: object = func) -> str:
            return _f(side, joint)

        return acc, f"callable {getattr(func, '__name__', repr(func))}(side, joint)"
    if len(params) == 1 and names[:1] == ["joint"]:

        def acc(side: str, joint: str, _f: object = func) -> str:
            return _f(joint)

        return acc, f"callable {getattr(func, '__name__', repr(func))}(joint)"
    if len(params) == 1 and names[:1] == ["side"]:
        value = func("left")
        explained = _t_b10_explain_mapping(value)
        if explained is None:
            return (
                None,
                f"{getattr(func, '__name__', repr(func))}(side) did not return a usable mapping",
            )
        mapping, shape = explained

        def acc(side: str, joint: str, _g: object = func, _m: object = mapping) -> str:
            if side == "left":
                return _m[joint]
            return _t_b10_explain_mapping(_g(side))[0][joint]

        return acc, f"callable {getattr(func, '__name__', repr(func))}(side) -> {shape}"
    if not params:
        value = func()
        explained = _t_b10_explain_mapping(value)
        if explained is None:
            return (
                None,
                f"{getattr(func, '__name__', repr(func))}() did not return a usable mapping",
            )
        mapping, shape = explained

        def acc(side: str, joint: str, _m: object = mapping) -> str:
            return _m[joint]

        return acc, f"callable {getattr(func, '__name__', repr(func))}() -> {shape}"
    return None, f"unsupported signature params={names!r}"


def _t_b10_find_mapping_surface(make_adapter_kwargs: dict | None = None) -> tuple[str, object]:
    """定位公开 URDF 关节名映射面，返回 (description, accessor)。

    形状白名单与查找顺序见区块头注释。任何形状都不可用时 raise
    AssertionError，消息列出全部已尝试候选（binding 层重新对齐的依据）。
    """
    from alohamini_cartesian.kinematics import joint_adapter as adapter_module

    attempts = []

    # 1) 模块级公开常量。
    for name in _T_B10_CONSTANT_CANDIDATES:
        value = getattr(adapter_module, name, _MISSING)
        if value is _MISSING:
            continue
        explained = _t_b10_explain_mapping(value)
        if explained is not None:
            mapping, shape = explained
            return (
                f"module constant {name!r} ({shape})",
                lambda side, joint, _m=mapping: _m[joint],
            )
        attempts.append(f"module constant {name!r} exists but is not a usable mapping")

    # 2) 模块级公开可调用。
    for name in _T_B10_CALLABLE_CANDIDATES:
        func = getattr(adapter_module, name, None)
        if callable(func):
            acc, desc = _t_b10_accessor_from_callable(func)
            if acc is not None:
                return f"module function {desc}", acc
            attempts.append(f"module function {name!r}: {desc}")

    # 3) 类级公开常量（仅本类定义，不吸收继承成员）。
    cls = adapter_module.JointKinematicAdapter
    for name in _T_B10_CONSTANT_CANDIDATES:
        if name in vars(cls):
            explained = _t_b10_explain_mapping(getattr(cls, name))
            if explained is not None:
                mapping, shape = explained
                return (
                    f"class constant JointKinematicAdapter.{name} ({shape})",
                    lambda side, joint, _m=mapping: _m[joint],
                )
            attempts.append(f"class constant {name!r} exists but is not a usable mapping")

    # 4) 实例方法 / property（用两种注入配置各查一遍，保证映射与注入无关）。
    for kwargs in ({"identity": True}, dict(make_adapter_kwargs or {})):
        adapter = make_adapter(**kwargs)
        for name in _T_B10_CALLABLE_CANDIDATES:
            attr = getattr(adapter, name, _MISSING)
            if attr is _MISSING:
                continue
            if callable(attr):
                acc, desc = _t_b10_accessor_from_callable(attr)
                if acc is not None:
                    return (
                        f"instance member {type(adapter).__name__}.{name} ({desc})",
                        acc,
                    )
                attempts.append(f"instance member {name!r}: {desc}")
            else:
                explained = _t_b10_explain_mapping(attr)
                if explained is not None:
                    mapping, shape = explained
                    return (
                        f"instance property {type(adapter).__name__}.{name} ({shape})",
                        lambda side, joint, _m=mapping: _m[joint],
                    )
                attempts.append(f"instance property {name!r} exists but is not a usable mapping")

    raise AssertionError(
        "T-B10: 未找到公开 URDF 关节名映射契约面（design §19 要求显式映射、"
        "可公开查询；形状白名单外的实现属 binding 层重新对齐）。已尝试: "
        + ("; ".join(attempts) if attempts else "（无任何候选成员存在）")
    )


def test_t_b10_urdf_joint_mapping_surface_is_public_and_injection_independent() -> None:
    """T-B10.1: 公开映射面存在、可按 (side, joint) 查询、与注入无关。"""
    desc_identity, acc_identity = _t_b10_find_mapping_surface({"identity": True})
    desc_default, acc_default = _t_b10_find_mapping_surface()
    for side in ("left", "right"):
        for joint in JOINT_ORDER:
            value_identity = acc_identity(side, joint)
            value_default = acc_default(side, joint)
            assert isinstance(value_identity, str) and value_identity, (
                f"T-B10: 映射面 ({desc_identity}) 对 ({side!r}, {joint!r}) 返回 "
                f"非字符串: {value_identity!r}"
            )
            assert value_identity == value_default, (
                f"T-B10: 映射与实例注入无关 violated——identity 配置与默认注入 "
                f"对 ({side!r}, {joint!r}) 返回不同 URDF 名: "
                f"{value_identity!r} vs {value_default!r}（模型名不是随臂"
                f"signs/zeros 配置变化的量）"
            )


def test_t_b10_left_side_urdf_joint_names_frozen_design_s19_and_reference() -> None:
    """T-B10.2: left 侧六关节 URDF 名逐项冻结（wrist_yaw _joint 后缀锚点）。"""
    desc, acc = _t_b10_find_mapping_surface()
    for joint in JOINT_ORDER:
        expected = T_B10_EXPECTED_URDF_NAMES[joint]
        actual = acc("left", joint)
        assert actual == expected, (
            f"T-B10: URDF 关节名映射冻结值不符（surface={desc}）: "
            f"joint {joint!r} side 'left' 期望 {expected!r}，实际 {actual!r}。"
            f'若实际形如 f"left_{{joint}}" 拼名产物（如 left_wrist_yaw 缺 '
            f"'_joint' 后缀），即为 design §19 明令禁止的运行时拼名实现"
        )


def test_t_b10_right_side_urdf_joint_names_keep_left_prefix_no_substitution() -> None:
    """T-B10.3: right 侧共用单臂源模型，映射保持 left_*，无前缀替换。"""
    desc, acc = _t_b10_find_mapping_surface()
    side = "right"
    for joint in JOINT_ORDER:
        expected = T_B10_EXPECTED_URDF_NAMES[joint]
        actual = acc(side, joint)
        assert actual == expected, (
            f"T-B10: right 侧 URDF 关节名不得做前缀替换（surface={desc}）: "
            f"joint {joint!r} side {side!r} 期望 {expected!r}（共用单臂源"
            f"模型，模型内部保持 left_*，见 design §19），实际 {actual!r}。"
            f'若实际是 right_* / f"{{side}}_{{joint}}" 拼名产物，即把设备键'
            f"前缀错误套用到模型关节名上"
        )


# T-B10.4 静态守卫的被禁模式清单（有限列举，属 binding 内容）: 只针对
# URDF 模型名前缀（left_/right_）与 side 模板前缀的运行时拼名构造；
# f"arm_{side}_{joint}" 设备键构造是 design §19 认可的合法形态，不在
# 清单内。实现若因清单误报受阻，属 binding 层沟通项，不是静默适配的理由。
_T_B10_FORBIDDEN_NAME_CONCAT_PATTERNS = (
    'f"left_',
    "f'left_",
    'f"right_',
    "f'right_",
    '"left_" +',
    "'left_' +",
    '"right_" +',
    "'right_' +",
    '+ "left_"',
    "+ 'left_'",
    '+ "right_"',
    "+ 'right_'",
    '"left_{}',
    "'left_{}",
    '"right_{}',
    "'right_{}",
    '"left_%s',
    "'left_%s",
    '"right_%s',
    "'right_%s",
    'f"{side}_',
    "f'{side}_",
    '"{side}_"',
    "'{side}_'",
    '"{}_{}".format',
    "'{}_{}'.format",
    '"%s_%s"',
    "'%s_%s'",
)


def test_t_b10_urdf_names_are_explicit_table_not_runtime_concatenation() -> None:
    """T-B10.4: 映射显式表驱动——模块源码无 URDF 模型名运行时拼名构造。

    约束守卫（当前实现下预期 PASS，性质同 T-B9 回归锁定先例，非 RED
    用例）: 抓「f"left_{joint}" 拼名 + hardcode wrist_yaw 特判补丁」这类
    行为等价但绕过显式映射表约束的实现——T-B10.2 的冻结值断言对它无
    差异，只有源码形态检查能区分。
    """
    from alohamini_cartesian.kinematics import joint_adapter as adapter_module

    try:
        source = inspect.getsource(adapter_module)
    except (OSError, TypeError) as exc:
        raise AssertionError(f"T-B10: 无法读取 adapter 模块源码做静态守卫检查: {exc!r}") from exc
    hits = [pattern for pattern in _T_B10_FORBIDDEN_NAME_CONCAT_PATTERNS if pattern in source]
    assert not hits, (
        f"T-B10: URDF 关节名映射必须是显式映射表，禁止运行时拼名（design "
        f"§19）。源码命中被禁拼名模式: {hits!r}。允许形态: 映射表字面量或"
        f'等价的显式常量；f"arm_{{side}}_{{joint}}" 设备键构造不在被禁'
        f"清单内"
    )


# Behavior T-B11 (P1-B 单位适配, design §8): 构造期 sign/zero 注入配置校验。
#
# sign 与 zero 是注入仿射 q = sign * motor_deg + zero 的两个常数。它们来自
# 外部注入（标定/URDF 侧写产物），不是运行期测量值，因此唯一正确的失败时
# 机是构造期（fail-fast）：换算期才炸意味着带着坏配置完成了整个装配，可能
# 已经下发过运动命令。
#
# 核心击杀目标：
# - sign 被当作任意浮点增益（0.7 / 0.5 / 2 / 0 静默通过 → 注入仿射退化为
#   错误齿轮比假设，所有 T-B1/T-B2 冻结值前提全部失效）；bool True 数值上
#   == 1，只查数值的实现对它静默放行，必须被类型检查击杀。
# - zero 不查（NaN/inf/字符串/bool 进构造函数）；NaN zero 在 obs 方向当前
#   会静默产出 NaN 向量（action 方向有换算期检查，obs 方向没有）。
# - 配置缺关节被零值填补（缺的 joint 静默取 sign=1/zero=0）。
#
# 未知 side 键处理决策（设计未明确，本 Test List 选定）：拒绝（fail-fast）。
# 理由：(1) signs/zeros 是注入配置而非运行输入，多出的 side 键只能是拼写
# 错误或用错关节集的产物，静默忽略会把配置错误变成死数据并掩盖上游装配
# 缺陷；(2) 模块既有 fail-fast 先例（T-B6 构造期 URDF 窗口校验、T-B7 元数
# 据不完整即拒绝）均拒绝可疑配置而非容忍；(3) design §9.1 明确 adapter 不
# 得在配置不完整时降级使能，同一原则覆盖注入配置。
#
# 缺关节报错内容：任务最小要求为「报错」；本 Test List 额外要求错误信息含
# 缺失关节名——模块内全部校验错误（T-B6.g、T-B7.a）均点名 side+joint，缺
# 关节错误若不点名关节则无法定位。side 名不作断言（两臂配置共用关节名，
# 点名关节已可定位，避免超出任务最小要求的过度强断言）。
#
# 有效配置（±1.0 / 有限 zero / 恰好 left+right × 六关节）仍可构造——由
# T-B1–T-B10 的既有回归覆盖，不另设通过型用例。
#
# SYNTHETIC FIXTURE NOTICE: 本节所有坏值（0.7 / "0.7" / "12.5" 等）均为
# 合成注入，与真实标定无关；它们的存在本身就是要被构造期校验拒绝的内容。


@pytest.mark.parametrize(
    ["side", "joint", "bad_sign"],
    [
        ("left", "elbow_flex", 0),
        ("right", "wrist_roll", 2),
        ("left", "shoulder_pan", 0.5),
        ("right", "shoulder_lift", 0.7),
        ("left", "wrist_yaw", "0.7"),
        ("right", "elbow_flex", True),
    ],
    ids=[
        "zero_gain",
        "double_gain",
        "half_gain",
        "fractional_gain_0_7",
        "numeric_string",
        "bool_true",
    ],
)
def test_t_b11_sign_not_pm1_rejected_at_construction(
    side: str, joint: str, bad_sign: float | str | bool
) -> None:
    """T-B11.1：sign ∉ {−1,+1}（含 bool/字符串）构造期即拒绝并点名。

    T-B11.1: sign ∉ {−1, +1}（0/2/0.5/0.7/数字字符串/bool）→ 构造即报
    ValueError，错误信息含 side 与 joint 名。

    0.7 是核心击杀目标：任何把 sign 当任意浮点增益的实现（齿轮比假设错误）
    都会在构造期静默通过并被此用例击杀。True 数值上 == 1，只查数值的实现
    （sign in (-1, 1)）对它放行——只有类型检查能拒绝，故必杀「值等检查不查
    类型」。数字字符串 "0.7" 击杀隐式 float 强转容忍实现。
    """
    with pytest.raises(ValueError) as excinfo:
        make_adapter(sign_overrides={side: {joint: bad_sign}})
    message = str(excinfo.value)
    assert side in message and joint in message, (
        f"T-B11.1: 非法 sign {bad_sign!r}（side={side!r}, joint={joint!r}）的"
        f"构造期错误必须点名 side 与 joint; got: {message!r}"
    )


@pytest.mark.parametrize(
    ["side", "joint", "bad_zero"],
    [
        ("left", "elbow_flex", float("nan")),
        ("right", "wrist_yaw", float("inf")),
        ("left", "shoulder_lift", float("-inf")),
        ("right", "shoulder_pan", "12.5"),
        ("left", "wrist_roll", True),
    ],
    ids=["nan", "inf", "neg_inf", "numeric_string", "bool_true"],
)
def test_t_b11_zero_non_finite_or_non_numeric_rejected_at_construction(
    side: str, joint: str, bad_zero: float | str | bool
) -> None:
    """T-B11.2：zero 非有限或非数值（含 bool）构造期即拒绝并点名。

    T-B11.2: zero 非有限（NaN/±inf）或字符串/bool → 构造即报 ValueError，
    错误信息含 side 与 joint 名。

    NaN/inf 击杀「zero 不查」：NaN zero 在 obs 方向当前实现会静默产出 NaN
    向量。数字字符串 "12.5" 击杀隐式 float 强转容忍实现（float("12.5") 可
    成功，强转后仍需类型拒绝）。bool True 击杀只查 np.isfinite 的实现
    （np.isfinite(True) 为 True）——必须显式排除 bool 类型。
    """
    with pytest.raises(ValueError) as excinfo:
        make_adapter(zero_overrides={side: {joint: bad_zero}})
    message = str(excinfo.value)
    assert side in message and joint in message, (
        f"T-B11.2: 非法 zero {bad_zero!r}（side={side!r}, joint={joint!r}）的"
        f"构造期错误必须点名 side 与 joint; got: {message!r}"
    )


@pytest.mark.parametrize("which", ["signs", "zeros", "both"])
def test_t_b11_missing_joint_config_rejected_at_construction(which: str) -> None:
    """T-B11.3: signs / zeros / 两者缺 elbow_flex → 构造即报 ValueError， 错误信息点名缺失关节。

    击杀「缺关节零值填补」（缺的 joint 静默取 sign=1/zero=0）与「换算期才
    KeyError」（晚失败）。三个变体分别击杀只校验 zeros、只校验 signs 的
    不对称实现；自然实现对称遍历 JOINT_ORDER，三者皆应拒绝。
    """
    drop = {"left": ("elbow_flex",)}
    kwargs: dict = {}
    if which in ("signs", "both"):
        kwargs["drop_sign_joints"] = drop
    if which in ("zeros", "both"):
        kwargs["drop_zero_joints"] = drop

    with pytest.raises(ValueError) as excinfo:
        make_adapter(**kwargs)
    message = str(excinfo.value)
    assert "elbow_flex" in message, (
        f"T-B11.3 ({which}): 缺关节配置的构造期错误必须点名缺失关节 elbow_flex; got: {message!r}"
    )


@pytest.mark.parametrize("which", ["signs", "zeros"])
def test_t_b11_unknown_side_key_rejected_at_construction(which: str) -> None:
    """T-B11.4：signs/zeros 出现未知 side 键时构造期即拒绝。

    T-B11.4: signs 或 zeros 出现未知 side 键 "center"（六关节值全部合法
    ±1/0，只有 side 键本身非法）→ 构造即报 ValueError，错误信息含
    "center"。

    决策依据见本节头注释：拒绝（fail-fast）。值全部合法保证只有 side 键
    触发拒绝，隔离被测行为。
    """
    kwargs: dict = {}
    if which == "signs":
        kwargs["sign_overrides"] = {"center": dict.fromkeys(JOINT_ORDER, 1.0)}
    else:
        kwargs["zero_overrides"] = {"center": dict.fromkeys(JOINT_ORDER, 0.0)}

    with pytest.raises(ValueError) as excinfo:
        make_adapter(**kwargs)
    message = str(excinfo.value)
    assert "center" in message, (
        f"T-B11.4 ({which}): 未知 side 键的构造期错误必须含未知键名 'center'; got: {message!r}"
    )


# ======================================================================
# P1-B 终审 findings（F1 / F2 / F3）—— 绑定 RED 测试（production 只读；
# 修复由后续 Viber cycle 实施，本节先行锁定验收）。
# ======================================================================
# Oracle 接线沿用 T-B3 建立的真实 bus oracle 模式：合成 FeetechMotorsBus
# + object.__new__（无 SDK / 无串口），只注入 _normalize/_unnormalize 触及
# 的属性；RANGE 模式分支不查 model 分辨率表，合成 model 名安全。
#
# F1（非 2 幂 span 的导出量化落位）：RANGE 导出方向把 motor_deg 经
#   round() 量化为意图整 tick 后编码 normalized；真实 bus `_unnormalize`
#   对 ((normalized+100)/200)·span+min 做 int() 截断。非 2 幂 span 下
#   浮点往返会把一部分整 tick 编码值截到「意图 tick − 1」（终审实测：
#   窗口 (110,1661)、res=4096、dm=0 时意图 tick 111 实际落 110；恒等
#   注入下该点 q=(111−885.5)·360/4095≈−68.0879°；全 tick 域失配率
#   183/1552 ≈ 11.8% ≈ ~12%）。失配点上全环往返
#   q→action→bus raw→obs→q 偏差恰为一个 tick（360/(res−1)≈0.0879°），
#   超出半 tick 容差（≈0.0440°）。修复方向（Viber 实施，测试只锁验收）：
#   导出编码取 bus 截断区间中点。
#
# F2（obs 方向 res 守卫）：`_check_finite_calibration` 目前只挂在导出
#   方向，obs 方向 RANGE 分支不查 resolution 可用性 → resolution=0.5
#   静默产出 ~−9e4° 垃圾、resolution=0 静默产出垃圾、resolution=1 裸
#   ZeroDivisionError（无 joint 名）。验收：obs 方向对 resolution < 2 或
#   非有限报 ValueError 且点名 joint。（resolution=NaN 目前已由
#   `_validate_motor_entry` 的有限性检查以含 joint 名的 ValueError 拒绝
#   ——该参数当前 GREEN，锁定不回归；其余三参数预期 RED。）
#
# F3（drive_mode ∉ {0,1} 拒绝；主 Agent 已裁决 fail-fast）：drive_mode
#   只被 `== 1` 消费，2 / −1 / 0.5 在 obs 与 action 双向都被当成 0 静默
#   接受（镜像取反被静默跳过）。验收：双向对 ∉{0,1} 的 drive_mode 报
#   ValueError 且点名 joint。（六参数全部预期 RED。）
#
# SYNTHETIC FIXTURE NOTICE: 本节所有 id / model / 标定 / 分辨率 /
# homing_offset 均为 SYNTHETIC、ASYMMETRIC、不可用于真实硬件。model 名
# （syn-enc-k* / syn-enc-n* / syn-enc-p* / syn-enc-r*）避开既有用例前缀
# 与真实 Feetech 表；唯一例外是 F1 的 resolution=4096（与真实 Feetech
# 表取值一致）——这是终审实测窗口的确定性复现所必需，model 名仍避开
# 真实表。reviewer 窗口 (110,1661) 仅作失配机制的确定性复现载体，其
# 数值不构成真实标定证据。
# ======================================================================


# ----------------------------------------------------------------------
# F2 / F3 共用底座：2 幂安全窗口 (0,1023) + res=1024（span = res − 1，
# 与 T_B3 采样同理的截断安全窗），dm=0；除被探测关节按用例覆盖单个字段
# 外全部合法。右臂同构（id 7..12）仅为 envelope 完整性，从不被左臂请求
# 消费。F2 / F3 的断言发生在 adapter 内部（obs 正向 / action 导入），
# 无需 bus oracle。
# ----------------------------------------------------------------------
def _f2_f3_specs(side: str, override_joint: str | None = None, **overrides: object) -> dict:
    specs = {}
    offset = 0 if side == "left" else 6
    letter = "p" if side == "left" else "r"
    for index, joint in enumerate(JOINT_ORDER):
        spec = {
            "id": index + 1 + offset,
            "model": f"syn-enc-{letter}{index + 1}",
            "normalization": "range_m100_100",
            "drive_mode": 0,
            "range_min": 0,
            "range_max": 1023,
            "homing_offset": (101 * (index + 1)) if side == "left" else (-101 * (index + 1)),
            "resolution": 1024,
        }
        if joint == override_joint:
            spec.update(overrides)
        specs[joint] = spec
    return specs


def _f2_f3_metadata(override_joint: str | None = None, **overrides: object) -> dict:
    return make_metadata(
        {
            **{
                f"arm_left_{j}": s
                for j, s in _f2_f3_specs("left", override_joint, **overrides).items()
            },
            **{f"arm_right_{j}": s for j, s in _f2_f3_specs("right").items()},
        }
    )


@pytest.mark.parametrize(
    "bad_resolution",
    [0, 0.5, 1, float("nan")],
    ids=["res-zero", "res-half", "res-one", "res-nan"],
)
def test_f2_obs_direction_unusable_resolution_rejected_with_joint_name(
    bad_resolution: float,
) -> None:
    """F2：obs 方向 RANGE 分支对不可用 resolution 必须 ValueError 点名 joint。

    探测关节 wrist_flex（其余五关节合法），obs 全部喂合法 normalized 值
    25.0，保证拒绝只能来自 resolution 守卫。当前实现：res=0.5 / res=0
    静默产出 ~−9e4° 垃圾（DID NOT RAISE）、res=1 裸 ZeroDivisionError
    —— 三参数 RED；res=NaN 已由 `_validate_motor_entry` 有限性检查以含
    joint 名的 ValueError 拒绝（该参数当前 GREEN，锁定不回归）。修复后
    四参数全 GREEN。
    """
    adapter = make_adapter()
    joint = "wrist_flex"
    metadata = _f2_f3_metadata(joint, resolution=bad_resolution)
    observation = {f"arm_left_{j}.pos": 25.0 for j in JOINT_ORDER}
    with pytest.raises(ValueError) as excinfo:
        adapter.observation_to_kinematic_deg("left", observation, metadata)
    message = str(excinfo.value)
    assert joint in message, (
        f"F2 (resolution={bad_resolution!r}): obs 方向拒绝不可用 resolution 时"
        f"必须点名 joint {joint!r}; got: {message!r}"
    )


@pytest.mark.parametrize("direction", ["obs", "action"])
@pytest.mark.parametrize("bad_drive_mode", [2, -1, 0.5], ids=["dm-plus-2", "dm-minus-1", "dm-half"])
def test_f3_drive_mode_outside_domain_rejected_with_joint_name(
    bad_drive_mode: float, direction: str
) -> None:
    """F3：drive_mode ∉ {0,1} 在 obs / action 双向都必须 ValueError 点名 joint。

    主 Agent 裁决 fail-fast：drive_mode 只被 `== 1` 消费，2 / −1 / 0.5
    目前被静默当作 0（镜像取反被跳过）→ 六参数全部 RED（DID NOT RAISE）。
    修复后（域检查报错）全 GREEN。探测关节 elbow_flex（其余五关节合法），
    obs=25.0 与 q 均取 gate 内合法值，保证拒绝只来自 drive_mode 域检查。
    """
    adapter = make_adapter()
    joint = "elbow_flex"
    metadata = _f2_f3_metadata(joint, drive_mode=bad_drive_mode)
    if direction == "obs":
        observation = {f"arm_left_{j}.pos": 25.0 for j in JOINT_ORDER}
        with pytest.raises(ValueError) as excinfo:
            adapter.observation_to_kinematic_deg("left", observation, metadata)
    else:
        # q 取窗口 (0,1023)/res=1024 内部 tick 512 对应的合法值：
        # q = sign·(512 − 511.5)·360/1023 + zero（gate = zero ± 180.0，
        # 严格界内，gate 不可能是拒绝来源）。
        q_vec = np.array(
            [
                KINEMATIC_SIGNS["left"][j] * (512 - 511.5) * 360.0 / 1023.0
                + KINEMATIC_ZEROS["left"][j]
                for j in JOINT_ORDER
            ],
            dtype=np.float64,
        )
        with pytest.raises(ValueError) as excinfo:
            adapter.kinematic_deg_to_action("left", q_vec, metadata)
    message = str(excinfo.value)
    assert joint in message, (
        f"F3 (drive_mode={bad_drive_mode!r}, {direction}): ∉{{0,1}} 的 "
        f"drive_mode 必须被双向拒绝且点名 joint {joint!r}; got: {message!r}"
    )


# ===========================================================================
# m3 (P4-B 终审修复轮) —— adapter 公开 `sign(side, joint)` 访问器
# (appended; 既有 259 例未动)
# ===========================================================================
#
# Required behavior (acceptance, 冻结): `JointKinematicAdapter` 必须暴露公开
# `sign(side, joint)` 访问器, 逐 side / 逐 joint 返回构造时注入的 affine gain (±1).
# controller 的 overcurrent-hold 释放方向映射依赖它, 不再读私有 `_signs`.
#
# 本用例有意区分: 恒 +1 的身份退化实现 / 返回整 dict 的实现 / 按 side 交换的实现 ——
# KINEMATIC_SIGNS 在两侧都混合 ±1, 任何退化都会在数值断言上失败. 当前 adapter 无
# `sign` 方法 -> AttributeError -> RED.
# ===========================================================================


def test_public_sign_accessor_returns_injected_sign_per_side_and_joint() -> None:
    """m3：`adapter.sign(side, joint)` 返回注入的 ±1 gain (逐 side / 逐 joint)."""
    adapter = make_adapter()
    for side in ("left", "right"):
        for joint in JOINT_ORDER:
            value = adapter.sign(side, joint)
            assert value == KINEMATIC_SIGNS[side][joint], (side, joint, value)
            assert value in (-1.0, 1.0)
