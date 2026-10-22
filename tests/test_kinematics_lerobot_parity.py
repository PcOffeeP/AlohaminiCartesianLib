"""关节运动学适配器与 lerobot 生产实现（MotorsBus / AlohaMini）的差分对照用例。

这些用例必须用 lerobot 的 ``FeetechMotorsBus`` / ``Motor`` /
``MotorCalibration`` / ``MotorNormMode`` / ``AlohaMini`` 作 oracle，或把发送端
过滤谓词钉在 ``AlohaMini.send_action`` 的源码上，否则无法判别适配器与生产 bus
语义是否对齐。未装 lerobot 的环境由 ``pytest.importorskip`` 整体跳过——这是
预期结果，不得为了变绿删减断言。搬运来源：lerobot 仓库
``tests/robots/test_alohamini_cartesian_adapter.py``，模块路径按本库映射为
``alohamini_cartesian.kinematics.joint_adapter``。
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

pytest.importorskip("lerobot", reason="lerobot 差分对照用例只在装有 runtime extra 的环境运行")

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


def test_t_b2_action_keys_survive_send_action_predicate_restatement() -> None:
    """T-B2.4：action 键全部通过生产 send_action 过滤谓词，并钉住其源码形态。"""
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

    # Exactly the six right-arm `.pos` keys, nothing else.
    assert isinstance(action, dict)
    assert set(action) == {f"arm_right_{joint}.pos" for joint in JOINT_ORDER}
    for joint, expected in zip(JOINT_ORDER, RIGHT_EXPECTED_MOTOR_DEG, strict=False):
        assert action[f"arm_right_{joint}.pos"] == pytest.approx(expected, abs=1e-9)

    # Literal restatement of the send_action arm filter: every produced key
    # must survive it (a suffix-less key would be silently dropped and the
    # joint command lost).
    assert restated_send_action_arm_filter(action, "right", metadata) == action

    # Teeth checks: the same restatement drops suffix-less bare names,
    # wrong-side keys and motors-foreign keys, so the alignment above is
    # not vacuous.
    bare_names = {f"arm_right_{joint}": 1.0 for joint in JOINT_ORDER}
    assert restated_send_action_arm_filter(bare_names, "right", metadata) == {}
    assert (
        restated_send_action_arm_filter({"arm_left_shoulder_pan.pos": 1.0}, "right", metadata) == {}
    )
    assert restated_send_action_arm_filter({"arm_right_ghost.pos": 1.0}, "right", metadata) == {}

    # Pin the restatement to the live production source: if the literal
    # predicate in AlohaMini.send_action changes shape, this fails and the
    # restatement must be re-reviewed (binding-level change), not silently
    # adapted.
    from lerobot.robots.alohamini.alohamini import AlohaMini

    normalized_source = " ".join(inspect.getsource(AlohaMini.send_action).split())
    assert _LEFT_PREDICATE_SNIPPET in normalized_source
    assert _RIGHT_PREDICATE_SNIPPET in normalized_source


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


def make_t_b3_oracle_bus(side: str) -> object:
    """构造单侧六关节的合成 FeetechMotorsBus oracle（绕过 __init__，无 SDK/串口）。

    Synthetic production FeetechMotorsBus oracle for one side (six joints
    only). object.__new__ bypasses __init__: no feetech SDK, no port, no
    serial traffic. Only the attributes touched by _normalize/_unnormalize
    are injected; apply_drive_mode and model_resolution_table are class
    attributes of FeetechMotorsBus.
    """
    from lerobot.motors.feetech.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode

    specs = T_B3_SPECS[side]
    bus = object.__new__(FeetechMotorsBus)
    bus.motors = {
        f"arm_{side}_{joint}": Motor(
            id=spec["id"],
            model=spec["model"],
            norm_mode=MotorNormMode(spec["normalization"]),
        )
        for joint, spec in specs.items()
    }
    bus.calibration = {
        f"arm_{side}_{joint}": MotorCalibration(
            id=spec["id"],
            drive_mode=spec["drive_mode"],
            homing_offset=spec["homing_offset"],
            range_min=spec["range_min"],
            range_max=spec["range_max"],
        )
        for joint, spec in specs.items()
    }
    bus._id_to_model_dict = {m.id: m.model for m in bus.motors.values()}
    bus._id_to_name_dict = {m.id: name for name, m in bus.motors.items()}
    return bus


def run_t_b3_full_loop_round_trip(side: str, q_deg: np.ndarray, metadata: dict) -> dict:
    """跑一次 adapter + 生产 bus 语义的全环路，返回全部中间量。

    One full loop through the adapter and the production bus semantics.
    Returns every intermediate so the test can assert on them.
    """
    adapter = make_adapter()
    action = adapter.kinematic_deg_to_action(side, q_deg, metadata)
    bus = make_t_b3_oracle_bus(side)
    ids = {f"arm_{side}_{joint}.pos": T_B3_SPECS[side][joint]["id"] for joint in JOINT_ORDER}
    raw = bus._unnormalize({ids[key]: value for key, value in action.items()})
    normalized = bus._normalize(raw)
    observation = {
        f"arm_{side}_{joint}.pos": normalized[T_B3_SPECS[side][joint]["id"]]
        for joint in JOINT_ORDER
    }
    result = adapter.observation_to_kinematic_deg(side, observation, metadata)
    return {"action": action, "raw": raw, "observation": observation, "result": result}


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b3_range_m100_100_full_loop_round_trip(side: str) -> None:
    """T-B3：RANGE_M100_100 全环路往返在生产 bus 语义下半 tick 内回到原 q。"""
    q_deg = np.array(T_B3_Q_DEG[side], dtype=np.float64)
    q_deg_before = q_deg.copy()
    metadata = make_t_b3_metadata()
    metadata_before = copy.deepcopy(metadata)

    loop = run_t_b3_full_loop_round_trip(side, q_deg, metadata)
    action = loop["action"]
    raw = loop["raw"]
    observation = loop["observation"]
    result = loop["result"]

    # Inputs untouched (same contract as T-B2.1).
    assert np.array_equal(q_deg, q_deg_before)
    assert metadata == metadata_before

    # Exactly the six `.pos` keys of this side (no gripper / base / other arm).
    assert isinstance(action, dict)
    assert set(action) == {f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER}

    # MAIN acceptance assertion: the full loop returns the original q_deg,
    # per joint, within half a tick derived from the metadata resolution.
    assert isinstance(result, np.ndarray)
    assert result.shape == (6,)
    for index, joint in enumerate(JOINT_ORDER):
        spec = T_B3_SPECS[side][joint]
        half_tick = 360.0 / (spec["resolution"] - 1) / 2.0
        assert result[index] == pytest.approx(q_deg[index], abs=half_tick + T_B3_EPS), (
            f"joint {joint!r} on side {side!r} failed the full-loop round trip"
        )

    # Oracle sanity: raw values are integers inside the calibration window,
    # and the bus-produced observation values are finite normalized values.
    for joint in JOINT_ORDER:
        spec = T_B3_SPECS[side][joint]
        raw_value = raw[spec["id"]]
        assert isinstance(raw_value, int)
        assert spec["range_min"] <= raw_value <= spec["range_max"]
        obs_value = observation[f"arm_{side}_{joint}.pos"]
        assert np.isfinite(obs_value)
        assert -100.0 <= obs_value <= 100.0

    # Normalized-domain sanity of the produced action values (the bus can
    # only consume [-100, 100]; this catches outputs in tick or degree
    # units). Placed after the main assertion so a RED failure surfaces as
    # the round-trip error first.
    for key, value in action.items():
        assert np.isfinite(value)
        assert -100.0 - T_B3_EPS <= value <= 100.0 + T_B3_EPS, (
            f"action value for {key!r} on side {side!r} is outside the normalized domain: {value!r}"
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


def _make_specs_oracle_bus(side: str, specs: dict[str, dict]) -> object:
    """按任意 per-joint specs 构造合成 FeetechMotorsBus oracle。

    Synthetic production FeetechMotorsBus oracle from ARBITRARY per-joint
    specs (same injection pattern as T-B3's oracle; object.__new__ bypasses
    __init__: no feetech SDK, no port, no serial traffic).
    """
    from lerobot.motors.feetech.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode

    bus = object.__new__(FeetechMotorsBus)
    bus.motors = {
        f"arm_{side}_{joint}": Motor(
            id=spec["id"],
            model=spec["model"],
            norm_mode=MotorNormMode(spec["normalization"]),
        )
        for joint, spec in specs.items()
    }
    bus.calibration = {
        f"arm_{side}_{joint}": MotorCalibration(
            id=spec["id"],
            drive_mode=spec["drive_mode"],
            homing_offset=spec["homing_offset"],
            range_min=spec["range_min"],
            range_max=spec["range_max"],
        )
        for joint, spec in specs.items()
    }
    bus._id_to_model_dict = {m.id: m.model for m in bus.motors.values()}
    bus._id_to_name_dict = {m.id: name for name, m in bus.motors.items()}
    return bus


def _run_range_loop(side: str, q_deg: np.ndarray, metadata: dict, specs: dict) -> dict:
    """按给定 specs 跑一次 RANGE 模式全环路（支持 drive_mode 翻转变体）。

    One full loop through the adapter and the production bus semantics,
    with the oracle bus built from the GIVEN per-joint specs (so drive_mode-
    flipped variants exercise the flipped calibration).
    """
    adapter = make_adapter()
    action = adapter.kinematic_deg_to_action(side, q_deg, metadata)
    bus = _make_specs_oracle_bus(side, specs)
    ids = {f"arm_{side}_{joint}.pos": specs[joint]["id"] for joint in JOINT_ORDER}
    raw = bus._unnormalize({ids[key]: value for key, value in action.items()})
    normalized = bus._normalize(raw)
    observation = {
        f"arm_{side}_{joint}.pos": normalized[specs[joint]["id"]] for joint in JOINT_ORDER
    }
    result = adapter.observation_to_kinematic_deg(side, observation, metadata)
    return {"action": action, "raw": raw, "observation": observation, "result": result}


def _flipped_drive_mode_specs(specs: dict[str, dict]) -> dict[str, dict]:
    """配对 fixture：只把每个 drive_mode 取反（0<->1）。

    Fixture derivation for the pair-invariance cases: the SAME per-joint
    specs with every drive_mode flipped (0 <-> 1), so the pair differs in
    drive_mode and nothing else.
    """
    return {name: {**entry, "drive_mode": 1 - entry["drive_mode"]} for name, entry in specs.items()}


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b4_1a_degrees_drive_mode_zero_and_one_convert_identically(side: str) -> None:
    """T-B4.1a: DEGREES conversion is completely drive_mode-insensitive.

    The same motor-degree observation and the same q_deg produce EXACTLY the
    T-B1 / T-B2 frozen expectations under all-dm=0 and all-dm=1 metadata (the
    bus' DEGREES branch ignores drive_mode; applying it in the adapter would
    flip every dm=1 joint's value and fail the frozen literals AND the pair
    invariance).
    """
    adapter = make_adapter()
    motor_deg = LEFT_MOTOR_DEG if side == "left" else RIGHT_MOTOR_DEG
    expected_forward = (
        LEFT_EXPECTED_KINEMATIC_DEG if side == "left" else RIGHT_EXPECTED_KINEMATIC_DEG
    )
    expected_inverse = LEFT_EXPECTED_MOTOR_DEG if side == "left" else RIGHT_EXPECTED_MOTOR_DEG
    q_deg = LEFT_Q_DEG if side == "left" else RIGHT_Q_DEG
    observation = make_arm_observation(side, motor_deg)
    specs = make_arm_motors(side)
    metadata_dm0 = make_metadata(specs)
    metadata_dm1 = make_metadata(_flipped_drive_mode_specs(specs))

    # Observation direction: identical under both drive_mode variants, and
    # equal to the frozen T-B1 expectations.
    out_dm0 = adapter.observation_to_kinematic_deg(side, observation, metadata_dm0)
    out_dm1 = adapter.observation_to_kinematic_deg(side, observation, metadata_dm1)
    assert out_dm0.shape == (6,)
    assert out_dm0 == pytest.approx(list(expected_forward), abs=1e-9)
    assert out_dm1 == pytest.approx(out_dm0, abs=1e-9)

    # Action direction: identical under both drive_mode variants, and equal
    # to the frozen T-B2 expectations (the bus' DEGREES _unnormalize branch
    # consumes plain motor degrees and ignores drive_mode).
    q = np.array(q_deg, dtype=np.float64)
    act_dm0 = adapter.kinematic_deg_to_action(side, q, metadata_dm0)
    act_dm1 = adapter.kinematic_deg_to_action(side, q, metadata_dm1)
    assert set(act_dm0) == {f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER}
    for joint, expected in zip(JOINT_ORDER, expected_inverse, strict=False):
        assert act_dm0[f"arm_{side}_{joint}.pos"] == pytest.approx(expected, abs=1e-9)
    assert act_dm1 == pytest.approx(act_dm0, abs=1e-9)

    # Oracle premise (same pattern as T-B3's oracle sanity): the production
    # bus' DEGREES branch itself ignores drive_mode. _normalize and
    # _unnormalize outputs are identical for dm=0 and dm=1 motors, and equal
    # to the plain-degree conversion through the metadata resolution field
    # (instance attribute shadows the class resolution table with the
    # synthetic metadata resolutions).
    from lerobot.motors.feetech.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode

    def make_degrees_oracle_bus(drive_mode: int) -> object:
        motor_entry = lambda joint: specs[f"arm_{side}_{joint}"]  # noqa: E731
        bus = object.__new__(FeetechMotorsBus)
        bus.motors = {
            f"arm_{side}_{joint}": Motor(
                id=motor_entry(joint)["id"],
                model=motor_entry(joint)["model"],
                norm_mode=MotorNormMode.DEGREES,
            )
            for joint in JOINT_ORDER
        }
        bus.calibration = {
            f"arm_{side}_{joint}": MotorCalibration(
                id=motor_entry(joint)["id"],
                drive_mode=drive_mode,
                homing_offset=motor_entry(joint)["homing_offset"],
                range_min=motor_entry(joint)["range_min"],
                range_max=motor_entry(joint)["range_max"],
            )
            for joint in JOINT_ORDER
        }
        bus._id_to_model_dict = {m.id: m.model for m in bus.motors.values()}
        bus._id_to_name_dict = {m.id: name for name, m in bus.motors.items()}
        bus.model_resolution_table = {
            motor_entry(joint)["model"]: motor_entry(joint)["resolution"] for joint in JOINT_ORDER
        }
        return bus

    bus0 = make_degrees_oracle_bus(0)
    bus1 = make_degrees_oracle_bus(1)
    norm_in = {specs[f"arm_{side}_{joint}"]["id"]: motor_deg[joint] for joint in JOINT_ORDER}
    norm0 = bus0._normalize(norm_in)
    norm1 = bus1._normalize(norm_in)
    assert norm0 == norm1
    deg0 = bus0._unnormalize(norm0)
    deg1 = bus1._unnormalize(norm1)
    assert deg0 == deg1
    for joint in JOINT_ORDER:
        entry = specs[f"arm_{side}_{joint}"]
        mid_id = entry["id"]
        mid = (entry["range_min"] + entry["range_max"]) / 2
        expected_norm = (motor_deg[joint] - mid) * 360.0 / (entry["resolution"] - 1)
        assert norm0[mid_id] == pytest.approx(expected_norm, abs=1e-9)


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b4_1b_range_drive_mode_flip_changes_normalized_values_and_round_trips(
    side: str,
) -> None:
    """T-B4.1b：dm 翻转使 normalized 值取反，且两种变体都全环往返成立。

    T-B4.1b: RANGE_M100_100 dm=0/1 produce DIFFERENT normalized values and
    both round-trip. Same q_deg, same ranges/resolutions, drive_mode flipped
    per joint: act(dm-flip) == -act(dm-orig) exactly per joint (the tick is
    dm-independent, so both variants share one quantized tick and the
    negation relation is exact; the pair is non-degenerate), obs(dm-flip) ==
    -obs(dm) likewise, and the full loop returns the original q_deg within
    half a tick under BOTH variants (the dm flip mirrors the real bus'
    symmetric semantics).
    """
    q_deg = np.array(T_B3_Q_DEG[side], dtype=np.float64)
    q_deg_before = q_deg.copy()

    specs_orig = T_B3_SPECS[side]
    specs_flip = _flipped_drive_mode_specs(specs_orig)
    metadata_orig = make_t_b3_metadata()
    metadata_flip = make_metadata(
        {
            **{f"arm_{side}_{joint}": specs_flip[joint] for joint in JOINT_ORDER},
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )

    adapter = make_adapter()
    act_orig = adapter.kinematic_deg_to_action(side, q_deg, metadata_orig)
    act_flip = adapter.kinematic_deg_to_action(side, q_deg, metadata_flip)

    assert isinstance(act_orig, dict) and isinstance(act_flip, dict)
    assert set(act_orig) == {f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER}
    assert set(act_flip) == set(act_orig)

    # (1) dm flip changes every normalized value, mirroring the real bus:
    # act(dm-flip) == -act(dm) exactly per joint (dm-independent tick).
    for joint in JOINT_ORDER:
        key = f"arm_{side}_{joint}.pos"
        assert act_flip[key] == pytest.approx(-act_orig[key], abs=1e-9), (
            f"drive_mode flip did not negate the normalized action for {key!r}: "
            f"{act_flip[key]!r} vs {act_orig[key]!r}"
        )
        # and the pair genuinely differs (not a degenerate zero pair).
        assert abs(act_orig[key]) > 0.0

    # (2) Full loop round trip holds under BOTH drive_mode variants (the
    # oracle bus is built from the SAME specs as each metadata variant).
    for metadata, loop_specs in (
        (metadata_orig, specs_orig),
        (metadata_flip, specs_flip),
    ):
        loop = _run_range_loop(side, q_deg, metadata, loop_specs)
        result = loop["result"]
        assert np.array_equal(q_deg, q_deg_before)
        assert isinstance(result, np.ndarray) and result.shape == (6,)
        for index, joint in enumerate(JOINT_ORDER):
            spec = specs_orig[joint]  # resolutions/ranges are dm-independent
            half_tick = 360.0 / (spec["resolution"] - 1) / 2.0
            assert result[index] == pytest.approx(q_deg[index], abs=half_tick), (
                f"joint {joint!r} on side {side!r} failed the dm-variant full loop"
            )

    # (3) The bus-produced observation values also flip with drive_mode
    # (obs(dm-flip) == -obs(dm) exactly): same tick, negated _normalize.
    loop_orig = _run_range_loop(side, q_deg, metadata_orig, specs_orig)
    loop_flip = _run_range_loop(side, q_deg, metadata_flip, specs_flip)
    for joint in JOINT_ORDER:
        key = f"arm_{side}_{joint}.pos"
        assert loop_flip["observation"][key] == pytest.approx(
            -loop_orig["observation"][key], abs=1e-9
        )


@pytest.mark.parametrize("side", ["left", "right"])
def test_t_b4_2a_range_action_intermediates_frozen_literals(side: str) -> None:
    """T-B4.2a（动作方向）：冻结 normalized 值与 oracle bus tick 字面量。

    T-B4.2a (action direction, closes blind spot ⑤ + ⑥): the adapter's
    normalized action value AND the oracle bus' raw tick are frozen literals.

    Given q_deg and the synthetic resolution, the true continuous tick is
    EXACTLY t* + 0.25, so the true tick is the frozen int t*. An implementation
    using `resolution` instead of `resolution - 1` shifts the continuous tick
    by c + 0.25 + 0.25/W ∈ [1.625, 1.9375] and rounds to t* + 2 — a +2 tick
    flip (far beyond one tick, margin >= 0.125 from the 0.5 boundary), which
    shifts the normalized output by 400/span = 100/W >= 0.003: BOTH frozen
    assertions fail. A both-side missing drive_mode negation emits +|norm|
    for dm=1 joints (sign flip vs the frozen negative literal) and the bus
    recovers the mirror tick 2*mid - t* != t*: BOTH frozen assertions fail.
    T-B3's closed round trip self-heals both mutants; these intermediate
    assertions do not.
    """
    q_deg = np.array(T_B4_Q_DEG[side], dtype=np.float64)
    q_deg_before = q_deg.copy()
    metadata = make_t_b4_metadata()

    adapter = make_adapter()
    action = adapter.kinematic_deg_to_action(side, q_deg, metadata)

    assert np.array_equal(q_deg, q_deg_before)
    assert isinstance(action, dict)
    assert set(action) == {f"arm_{side}_{joint}.pos" for joint in JOINT_ORDER}

    # Frozen normalized literals (sign included: dm=1 -> negative, dm=0 -> +).
    for joint in JOINT_ORDER:
        key = f"arm_{side}_{joint}.pos"
        expected = T_B4_EXPECTED_ACTION[side][joint]
        assert action[key] == pytest.approx(expected, abs=1e-9), (
            f"frozen normalized action mismatch for {key!r}: {action[key]!r} != {expected!r}"
        )
        # The sign IS the 2b freeze: dm=1 joints negative, dm=0 positive.
        if T_B4_SPECS[side][joint]["drive_mode"] == 1:
            assert action[key] < 0.0
        else:
            assert action[key] > 0.0

    # Oracle bus single step: the raw tick for the produced action is a
    # frozen int literal (the exact tick the correct conversion quantized to).
    bus = _make_specs_oracle_bus(side, T_B4_SPECS[side])
    ids = {f"arm_{side}_{joint}.pos": T_B4_SPECS[side][joint]["id"] for joint in JOINT_ORDER}
    raw = bus._unnormalize({ids[key]: value for key, value in action.items()})
    for joint in JOINT_ORDER:
        spec = T_B4_SPECS[side][joint]
        expected_tick = T_B4_EXPECTED_BUS_TICK[side][joint]
        assert isinstance(raw[spec["id"]], int)
        assert raw[spec["id"]] == expected_tick, (
            f"frozen oracle bus tick mismatch for arm_{side}_{joint!r}: "
            f"{raw[spec['id']]!r} != {expected_tick!r}"
        )
        assert spec["range_min"] <= raw[spec["id"]] <= spec["range_max"]


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
def test_t_b5_3_full_loop_real_bus_nonzero_homing_round_trip_and_variant_identical(
    side: str,
) -> None:
    """T-B5.3：非零 homing 下真实 bus 全环往返，且 homing 变体逐位相同。

    T-B5.3: the full-loop round trip through the REAL production bus math
    holds with all-non-zero homing offsets, the bus semantics themselves are
    homing-insensitive, and the whole loop is bitwise identical under the
    homing-only variant.

    Loop (same shape as T-B3/T-B4): adapter.kinematic_deg_to_action ->
    FeetechMotorsBus._unnormalize (real calibration incl. non-zero
    homing_offset) -> FeetechMotorsBus._normalize -> adapter's forward.
    Assertions:
    (1) bus premise: _unnormalize of the SAME action under the original and
        the homing-variant calibration yields BITWISE identical raw ticks and
        identical _normalize outputs -- the firmware-register offset never
        enters the bus math (design §8's premise, verified against the real
        MotorsBus implementation);
    (2) the full loop returns the original q_deg within half a tick under
        the original (all-non-zero-homing) calibration;
    (3) the variant loop (variant metadata + variant bus calibration)
        produces BITWISE identical action / raw / observation / result -- no
        stage of the loop may re-apply the homing offset.
    """
    q = np.array(T_B4_Q_DEG[side], dtype=np.float64)
    q_before = q.copy()
    specs = T_B4_SPECS[side]
    variant_specs = _homing_variant_specs(specs)
    metadata_orig = _make_arm_metadata(side, specs)
    metadata_variant = _make_arm_metadata(side, variant_specs)
    ids = {f"arm_{side}_{joint}.pos": specs[joint]["id"] for joint in JOINT_ORDER}
    bus_orig = _make_specs_oracle_bus(side, specs)
    bus_variant = _make_specs_oracle_bus(side, variant_specs)
    adapter = make_adapter()

    action = adapter.kinematic_deg_to_action(side, q, metadata_orig)
    raw_orig = bus_orig._unnormalize({ids[key]: value for key, value in action.items()})
    raw_variant = bus_variant._unnormalize({ids[key]: value for key, value in action.items()})
    # (1) bus premise: the offset lives in the firmware register, not in the
    # bus math -- raw ticks and normalized values are bitwise identical.
    assert raw_variant == raw_orig, (
        "FeetechMotorsBus._unnormalize changed with a homing-only calibration "
        f"variant (side={side!r}): {raw_variant!r} vs {raw_orig!r}"
    )
    normalized_orig = bus_orig._normalize(raw_orig)
    normalized_variant = bus_variant._normalize(raw_variant)
    assert normalized_variant == normalized_orig

    # (2) full loop round trip under the original non-zero-homing calib.
    observation = {
        f"arm_{side}_{joint}.pos": normalized_orig[specs[joint]["id"]] for joint in JOINT_ORDER
    }
    result = adapter.observation_to_kinematic_deg(side, observation, metadata_orig)
    assert np.array_equal(q, q_before)
    assert isinstance(result, np.ndarray) and result.shape == (6,)
    for index, joint in enumerate(JOINT_ORDER):
        half_tick = 360.0 / (specs[joint]["resolution"] - 1) / 2.0
        assert result[index] == pytest.approx(q[index], abs=half_tick), (
            f"full loop with non-zero homing offsets failed for {joint!r} "
            f"on side {side!r}: {result[index]!r} vs {q[index]!r}"
        )

    # (3) variant loop: bitwise-identical at EVERY stage.
    action_variant = adapter.kinematic_deg_to_action(side, q, metadata_variant)
    observation_variant = {
        f"arm_{side}_{joint}.pos": normalized_variant[variant_specs[joint]["id"]]
        for joint in JOINT_ORDER
    }
    result_variant = adapter.observation_to_kinematic_deg(
        side, observation_variant, metadata_variant
    )
    assert action_variant == action, (
        "full-loop action changed when only metadata homing_offset changed "
        f"(side={side!r}): {action_variant!r} vs {action!r}"
    )
    assert observation_variant == observation
    assert np.array_equal(result_variant, result), (
        "full-loop result changed when only metadata homing_offset changed "
        f"(side={side!r}): {result_variant!r} vs {result!r}"
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
# F1 fixture：reviewer 的非 2 幂标定窗口。六关节共用同一窗口与分辨率，
# drive_mode 双侧混布（设计期仿真证实失配集合与 dm 无关：dm=1 的取反在
# bus `_unnormalize` 内被精确恢复，截断数学与 dm=0 完全一致）。
# ----------------------------------------------------------------------
F1_RANGE_MIN = 110
F1_RANGE_MAX = 1661
F1_RESOLUTION = 4096
F1_DRIVE_MODES = {"left": (0, 1, 0, 1, 0, 1), "right": (1, 0, 1, 0, 1, 0)}

F1_SPECS = {
    side: {
        joint: {
            "id": index + 1,
            "model": (f"syn-enc-k{index + 1}" if side == "left" else f"syn-enc-n{index + 1}"),
            "normalization": "range_m100_100",
            "drive_mode": F1_DRIVE_MODES[side][index],
            "range_min": F1_RANGE_MIN,
            "range_max": F1_RANGE_MAX,
            "homing_offset": (
                333 - 97 * (index + 1) if side == "left" else -808 + 89 * (index + 1)
            ),
            "resolution": F1_RESOLUTION,
        }
        for index, joint in enumerate(JOINT_ORDER)
    }
    for side in ("left", "right")
}


def make_f1_metadata() -> dict:
    """Metadata v2 envelope：双臂 F1 窗口 + 既有 gripper / base 干扰项。"""
    return make_metadata(
        {
            **{f"arm_left_{joint}": dict(spec) for joint, spec in F1_SPECS["left"].items()},
            **{f"arm_right_{joint}": dict(spec) for joint, spec in F1_SPECS["right"].items()},
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )


def f1_q_from_tick(side: str, joint: str, tick: int) -> float:
    """意图 bus tick -> 注入 q_deg（F1 采样推导，非被测行为）。

    q(T) = sign * (T − mid) · 360 / (res − 1) + zero —— 刻意与 production
    `_effective_q_range` 的 q_at_min / q_at_max 表达式逐位同构（同一左结合
    运算顺序），端点护栏的端点 q 因此被 gate 原样放行；主用例的半 tick
    容差对该表达式最后一位 ulp 不敏感。
    """
    sign = KINEMATIC_SIGNS[side][joint]
    zero = KINEMATIC_ZEROS[side][joint]
    mid = (F1_RANGE_MIN + F1_RANGE_MAX) / 2
    return sign * (tick - mid) * 360.0 / (F1_RESOLUTION - 1) + zero


# 采样设计：从终审实测失配点（意图 tick 111）起步、步长 13 横扫
# [111, 1658] 共 120 点，全部严格位于 gate 内。设计期仿真（与 production
# 逐算子一致的浮点链）给出 16 个确定性失配点 / 120（13.3%；全 tick 域
# 183/1552 ≈ 11.8%，与终审 ~12% 一致），低端（111/163/176）、中段
# （891..969 连片）、高端（1645）均有分布；其余 104 点为应当干净的对照
# 点（击杀「无差别全失败」实现）。失配集合对 dm=0/1 一致。
F1_PROBE_TICKS = tuple(111 + 13 * step for step in range(120))
# 非探测关节统一钉在 886（设计期仿真确认的干净 tick：raw 恰为 886、
# 往返误差 0），其噪声不会混入探测关节的诊断。
F1_PIN_TICK = 886


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("joint", JOINT_ORDER)
def test_f1_non_power_of_two_window_full_loop_round_trip(side: str, joint: str) -> None:
    """F1 主用例：非 2 幂窗口下 gate 内多点全环往返必须逐点半 tick 回到原 q。

    全环：q_deg --adapter 导出--> normalized --真实 bus `_unnormalize`
    (int 截断)--> raw --真实 bus `_normalize`--> obs --adapter 正向--> q'。
    当前实现在确定性失配 tick 上 raw 落到意图 tick − 1（含终审实测点：
    意图 tick 111 落 110），q' 偏差恰为一个 tick（0.0879° > 半 tick
    0.0440°）→ 预期 RED（每个 (side, joint) 参数 16/120 确定性失配）。
    修复（截断区间中点编码）后全部 120 点必须逐点 GREEN。
    """
    specs = F1_SPECS[side]
    metadata = make_f1_metadata()
    half_tick = 360.0 / (specs[joint]["resolution"] - 1) / 2.0
    pin_q = {j: f1_q_from_tick(side, j, F1_PIN_TICK) for j in JOINT_ORDER}
    mismatches = []
    for tick in F1_PROBE_TICKS:
        q_vec = np.array(
            [f1_q_from_tick(side, joint, tick) if j == joint else pin_q[j] for j in JOINT_ORDER],
            dtype=np.float64,
        )
        loop = _run_range_loop(side, q_vec, metadata, specs)
        for index, j in enumerate(JOINT_ORDER):
            error = abs(float(loop["result"][index]) - q_vec[index])
            if error > half_tick + T_B3_EPS:
                mismatches.append((j, tick, loop["raw"][specs[j]["id"]], error))
    assert not mismatches, (
        f"F1 ({side}, {joint}): {len(mismatches)}/{len(F1_PROBE_TICKS)} 个采样点"
        f"在 bus int() 截断后丢失意图 tick（半 tick 容差 {half_tick!r}°）；逐点：\n"
        + "\n".join(
            f"  joint={j!r} 意图tick={t} -> bus raw={r} 偏差={e!r}°" for j, t, r, e in mismatches
        )
    )


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("joint", JOINT_ORDER)
def test_f1_calibration_endpoint_round_trip_is_exact(side: str, joint: str) -> None:
    """F1 护栏：标定端点（意图 tick = range_min / range_max）必须精确落位。

    「截断区间中点」修复会把内部 tick 的编码值整体偏移半个 bus 截断区间；
    本护栏锁定端点不受该偏移影响：raw 恰为端点 tick（int 相等），q 往返
    误差 ≤ 1e-9（远小于半 tick ≈0.0440°）。当前实现端点本就精确（±100
    的编码值不经过浮点损耗）——本用例是修复的回归护栏，不是 RED 驱动
    （当前预期 GREEN，修复后必须仍 GREEN）。
    """
    specs = F1_SPECS[side]
    metadata = make_f1_metadata()
    spec = specs[joint]
    half_tick = 360.0 / (spec["resolution"] - 1) / 2.0
    pin_q = {j: f1_q_from_tick(side, j, F1_PIN_TICK) for j in JOINT_ORDER}
    index = JOINT_ORDER.index(joint)
    for tick in (F1_RANGE_MIN, F1_RANGE_MAX):
        q_vec = np.array(
            [f1_q_from_tick(side, j, tick) if j == joint else pin_q[j] for j in JOINT_ORDER],
            dtype=np.float64,
        )
        loop = _run_range_loop(side, q_vec, metadata, specs)
        raw_tick = loop["raw"][spec["id"]]
        assert raw_tick == tick, (
            f"F1 护栏 ({side}, {joint}): 端点意图 tick={tick} 被 bus 落到 "
            f"raw={raw_tick!r} —— 端点必须精确落位"
        )
        endpoint_error = abs(float(loop["result"][index]) - q_vec[index])
        assert endpoint_error <= 1e-9, (
            f"F1 护栏 ({side}, {joint}): 端点 tick={tick} 的 q 往返偏差 "
            f"{endpoint_error!r}° 超过精确性护栏 1e-9"
        )
        # 钉在干净 tick 的其余五关节只须满足半 tick 往返（上下文完整性，
        # 不是本用例的验收点）。
        for i, j in enumerate(JOINT_ORDER):
            if j == joint:
                continue
            assert abs(float(loop["result"][i]) - q_vec[i]) <= half_tick + T_B3_EPS


# ----------------------------------------------------------------------
# F1 负半区（fuzz 复核暴露，主 Agent 已裁决属 F1 acceptance 范围）——
# 绑定 RED 测试（production 只读；修复由后续 Viber cycle 实施）。
#
# F1 修复（截断区间中点 = normalized + 100/span）只对 t_cont ≥ 0 成立。
# 真实 bus `_unnormalize` 对 ((normalized+100)/200)·span + range_min 取
# Python int() —— 向零截断：正 t_cont 的截断区间是 [T, T+1)，中点编码
# +100/span 正确；负 t_cont 的截断区间镜像为 (T−1, T]，正确中点编码是
# −100/span（向下）。当前实现无条件下重编码 +100/span：负域失配点上
# t_cont′ = T + 0.5 仍被 int() 向零截到 T + 1，意图 tick 确定性丢失
# （fuzz trial#28 数值钉死：span=11622、res=16385、意图 tick −132：
# t_cont′=−131.5 仍 int→−131 错）。
#
# 设计期基线（真实 adapter + 真实 bus oracle 全域横扫，与本节用例同一
# oracle 接线）:
# - reviewer 窗口 (−1559, 10063)/res=16385（span=11622，非 2 幂）：负域
#   [−1559, 0] 全部 1560 个 tick 中 610 个确定性失配（39.10%），每个失配
#   点 raw = 意图 tick + 1（向零截断特征）；失配集合与 side / joint /
#   drive_mode 无关（双侧、dm=0/1 混布逐点核实）。
# - 对称窗口 (−8031, 8031)/res=16385（span=16062，非 2 幂）：负域全部
#   8032 个 tick 中 516 个确定性失配（6.42%），同样 side/joint/dm 无关。
# - 符号感知修复（t_cont < 0 用 −100/span）的设计期仿真经真实 bus
#   oracle 复核：两窗口负域失配全部归零 → 修复后本节用例必须逐点
#   GREEN；正半区（t_cont ≥ 0，含 t=0 边界）在两窗口均 0 失配（修复不
#   得触碰），t=0 是两侧截断区间的公共端点，天然截断安全。
#
# SYNTHETIC FIXTURE NOTICE: 本节所有 id / model / 标定 / 分辨率 /
# homing_offset 均为 SYNTHETIC；model 名（syn-enc-t* / syn-enc-u*）避开
# 既有用例前缀（a/b/d/e/f/g/k/l/n/w）与真实 Feetech 表，resolution=
# 16385 不在真实 Feetech 表（仅 4096 / 1024）中；负 range_min 仅作
# bus 截断数学的确定性复现载体，数值不构成真实标定证据。
# ----------------------------------------------------------------------

# reviewer 窗口：负 range_min + 非 2 幂 span=11622（fuzz trial#28 同窗）。
NF1A_RANGE_MIN = -1559
NF1A_RANGE_MAX = 10063
NF1A_RESOLUTION = 16385
NF1A_PIN_TICK = 4252  # 设计期核实的中段干净 tick（六关节双侧往返误差 0）

# 采样设计：步长 88 横扫 [−1540, −44] 共 18 点（设计期真实 oracle 分类：
# 12 个确定性失配点 / 6 个干净对照点，低/中/高端均有分布），显式并入
# 负端点 −1559 与 t=0 截断符号边界（两端当前均精确落位，作对照点，
# 防止修复越界触碰端点/边界）。步长序列天然包含 fuzz trial#28 的实测
# 钉死点 −132（−1540 + 88·16）与 −44；其余失配点 −1540 / −1452 / −1364 /
# −1276 / −1188 / −572 / −484 / −396 / −308 / −220 为低/中/高分布的
# 确定性触发点，对照点 −1100 / −1012 / −924 / −836 / −748 / −660 击杀
# 「无差别全失败」实现。
NF1A_PROBE_TICKS = (
    -1559,
    -1540,
    -1452,
    -1364,
    -1276,
    -1188,
    -1100,
    -1012,
    -924,
    -836,
    -748,
    -660,
    -572,
    -484,
    -396,
    -308,
    -220,
    -132,
    -44,
    0,
)

# 对称窗口：range_min = −range_max（normalized 符号与 tick 符号严格同
# 号，t=0 ↔ normalized=0 精确成立），span=16062 非 2 幂。
NF1B_RANGE_MIN = -8031
NF1B_RANGE_MAX = 8031
NF1B_RESOLUTION = 16385
NF1B_PIN_TICK = 100  # 设计期核实的干净 tick（六关节双侧往返误差 0）

# 采样设计：步长 631 均匀横扫 [−8031, −459] 共 13 点（负域失配率仅
# 6.42%，均匀步长不足以必然命中失配），显式并入设计期实测的确定性失配
# 钉点 −8027 / −6183 / −2415 / −1214 / −38（端点邻域 / 中段 / 近边界
# 分布）与负端点 −8031、t=0 边界对照。设计期分类：失配 7 点（含步长点
# −7400 / −1721），干净对照 12 点。
NF1B_PROBE_TICKS = (
    -8031,
    -8027,
    -7400,
    -6769,
    -6183,
    -6138,
    -5507,
    -4876,
    -4245,
    -3614,
    -2983,
    -2415,
    -2352,
    -1721,
    -1214,
    -1090,
    -459,
    -38,
    0,
)

# drive_mode 双侧混布沿用 F1（设计期证实负域失配集合与 dm 无关：dm=1
# 的取反在 bus `_unnormalize` 内被精确恢复，截断数学与 dm=0 一致）。
NF1_WINDOWS = {
    "reviewer": {
        "range_min": NF1A_RANGE_MIN,
        "range_max": NF1A_RANGE_MAX,
        "resolution": NF1A_RESOLUTION,
        "pin_tick": NF1A_PIN_TICK,
        "probes": NF1A_PROBE_TICKS,
    },
    "symmetric": {
        "range_min": NF1B_RANGE_MIN,
        "range_max": NF1B_RANGE_MAX,
        "resolution": NF1B_RESOLUTION,
        "pin_tick": NF1B_PIN_TICK,
        "probes": NF1B_PROBE_TICKS,
    },
}


def make_nf1_specs(window: dict, side: str) -> dict:
    """负半区窗口的 per-joint specs（合成、双侧同窗、dm 混布）。"""
    return {
        joint: {
            "id": index + 1,
            "model": f"syn-enc-t{index + 1}" if side == "left" else f"syn-enc-u{index + 1}",
            "normalization": "range_m100_100",
            "drive_mode": F1_DRIVE_MODES[side][index],
            "range_min": window["range_min"],
            "range_max": window["range_max"],
            "homing_offset": (
                333 - 97 * (index + 1) if side == "left" else -808 + 89 * (index + 1)
            ),
            "resolution": window["resolution"],
        }
        for index, joint in enumerate(JOINT_ORDER)
    }


def make_nf1_metadata(window: dict) -> dict:
    """Metadata v2 envelope：双臂负半区窗口 + 既有 gripper / base 干扰项。"""
    return make_metadata(
        {
            **{
                f"arm_left_{joint}": dict(spec)
                for joint, spec in make_nf1_specs(window, "left").items()
            },
            **{
                f"arm_right_{joint}": dict(spec)
                for joint, spec in make_nf1_specs(window, "right").items()
            },
            "arm_left_gripper": GRIPPER_MOTOR_SPECS["left"],
            "arm_right_gripper": GRIPPER_MOTOR_SPECS["right"],
            **BASE_MOTOR_SPECS,
        }
    )


def nf1_q_from_tick(window: dict, side: str, joint: str, tick: int) -> float:
    """意图 bus tick -> 注入 q_deg（负半区采样推导，非被测行为）。

    与 production `_effective_q_range` 的 q_at_min / q_at_max 表达式逐位
    同构（同一左结合运算顺序），窗口内 tick 严格界内时 gate 必然放行。
    """
    sign = KINEMATIC_SIGNS[side][joint]
    zero = KINEMATIC_ZEROS[side][joint]
    mid = (window["range_min"] + window["range_max"]) / 2
    return sign * (tick - mid) * 360.0 / (window["resolution"] - 1) + zero


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("joint", JOINT_ORDER)
def test_f1_negative_half_region_full_loop_round_trip_reviewer_window(
    side: str, joint: str
) -> None:
    """F1 负半区（reviewer 窗口）：负域多点全环往返必须回到原 q。

    F1 负半区主用例（reviewer 窗口）：负 range_min 非 2 幂窗口下 gate 内
    负域多点全环往返必须逐点半 tick 回到原 q。

    全环与 F1 主用例相同：q_deg --adapter 导出--> normalized --真实 bus
    `_unnormalize` (int 向零截断)--> raw --真实 bus `_normalize`--> obs
    --adapter 正向--> q'。当前实现在确定性失配 tick 上 raw 落到
    意图 tick + 1（含 trial#28 实测点：意图 −132 落 −131；t_cont′ =
    −131.5 仍被 int() 向零截断），q' 偏差恰为一个 tick（360/16384 ≈
    0.02197° > 半 tick ≈0.01099°）→ 预期 RED（每个 (side, joint) 参数
    12/20 确定性失配）。符号感知修复（t_cont < 0 用 −100/span）后全部
    20 点必须逐点 GREEN。
    """
    window = NF1_WINDOWS["reviewer"]
    specs = make_nf1_specs(window, side)
    metadata = make_nf1_metadata(window)
    half_tick = 360.0 / (specs[joint]["resolution"] - 1) / 2.0
    pin_tick = window["pin_tick"]
    pin_q = {j: nf1_q_from_tick(window, side, j, pin_tick) for j in JOINT_ORDER}
    mismatches = []
    for tick in window["probes"]:
        q_vec = np.array(
            [
                nf1_q_from_tick(window, side, joint, tick) if j == joint else pin_q[j]
                for j in JOINT_ORDER
            ],
            dtype=np.float64,
        )
        loop = _run_range_loop(side, q_vec, metadata, specs)
        for index, j in enumerate(JOINT_ORDER):
            error = abs(float(loop["result"][index]) - q_vec[index])
            if error > half_tick + T_B3_EPS:
                mismatches.append((j, tick, loop["raw"][specs[j]["id"]], error))
    assert not mismatches, (
        f"F1 负半区 (reviewer 窗口, {side}, {joint}): "
        f"{len(mismatches)}/{len(window['probes'])} 个负域采样点在 bus "
        f"int() 向零截断后丢失意图 tick（半 tick 容差 {half_tick!r}°）；逐点：\n"
        + "\n".join(
            f"  joint={j!r} 意图tick={t} -> bus raw={r} 偏差={e!r}°" for j, t, r, e in mismatches
        )
    )


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("joint", JOINT_ORDER)
def test_f1_negative_half_region_full_loop_round_trip_symmetric_window(
    side: str, joint: str
) -> None:
    """F1 负半区（对称窗口）：负域多点全环往返必须回到原 q。

    F1 负半区主用例（对称窗口）：range_min = −range_max 的非 2 幂窗口下
    负域多点全环往返必须逐点半 tick 回到原 q。

    对称窗口使 t=0 ↔ normalized=0 精确成立（负 tick ⇔ 负 normalized），
    验证符号感知截断行为跨窗口泛化，不绑定 reviewer 窗口的特定 span /
    阈值。当前实现确定性失配 → 预期 RED（每个 (side, joint) 参数 7/19
    确定性失配）；符号感知修复后全部 19 点必须逐点 GREEN。
    """
    window = NF1_WINDOWS["symmetric"]
    specs = make_nf1_specs(window, side)
    metadata = make_nf1_metadata(window)
    half_tick = 360.0 / (specs[joint]["resolution"] - 1) / 2.0
    pin_tick = window["pin_tick"]
    pin_q = {j: nf1_q_from_tick(window, side, j, pin_tick) for j in JOINT_ORDER}
    mismatches = []
    for tick in window["probes"]:
        q_vec = np.array(
            [
                nf1_q_from_tick(window, side, joint, tick) if j == joint else pin_q[j]
                for j in JOINT_ORDER
            ],
            dtype=np.float64,
        )
        loop = _run_range_loop(side, q_vec, metadata, specs)
        for index, j in enumerate(JOINT_ORDER):
            error = abs(float(loop["result"][index]) - q_vec[index])
            if error > half_tick + T_B3_EPS:
                mismatches.append((j, tick, loop["raw"][specs[j]["id"]], error))
    assert not mismatches, (
        f"F1 负半区 (对称窗口, {side}, {joint}): "
        f"{len(mismatches)}/{len(window['probes'])} 个负域采样点在 bus "
        f"int() 向零截断后丢失意图 tick（半 tick 容差 {half_tick!r}°）；逐点：\n"
        + "\n".join(
            f"  joint={j!r} 意图tick={t} -> bus raw={r} 偏差={e!r}°" for j, t, r, e in mismatches
        )
    )
