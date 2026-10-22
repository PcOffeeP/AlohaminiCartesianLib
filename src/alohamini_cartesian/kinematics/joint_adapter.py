#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit adapter between raw motor-degree observations and kinematic joint degrees.

In DEGREES mode the motors bus has already turned ticks into motor degrees
(``MotorsBus._normalize`` DEGREES branch), so the adapter sees plain degrees
and must NOT re-apply ``homing_offset`` / ``drive_mode`` / ``resolution``.

Behavior T-B1 (design §8): the forward conversion of the six kinematic joints
is a per-joint affine map on the requested side only:

    q_kinematic = sign * motor_deg + zero

Behavior T-B2 (design §8): the inverse conversion back to a motor command
dict is the exact inverse of the forward map, on the same frozen joint order:

    motor_deg = sign * (q_kinematic - zero)          (sign ∈ {−1, +1})

Behavior T-B3 (design §8): when a motor's metadata `normalization` is the
default ``"range_m100_100"``, the bus no longer speaks degrees, so the
adapter must convert motor degrees <-> normalized units through the metadata
fields (``resolution`` / ``range_min`` / ``range_max`` / ``drive_mode``),
mirroring ``MotorsBus._unnormalize`` / ``MotorsBus._normalize``:

    tick        = round(motor_deg * (resolution - 1) / 360 + mid)
    normalized  = ((tick - range_min) / (range_max - range_min)) * 200 - 100
    normalized  = -normalized            (drive_mode == 1)

and the exact inverse on the observation side. The round-quantization to a
whole tick is required because the bus' ``int()`` truncation of a
continuous value would err by up to almost one tick (more than the half-tick
round-trip tolerance). For calibration spans that are not powers of two the
exact tick encoding can still lose one tick to the bus' ``int()`` truncation
through compounded float error, so the export path verifies the encoding
against the bus' own truncation formula and re-encodes at the midpoint of
the bus' truncation interval only when the exact encoding would lose the
tick (endpoints stay exact). ``homing_offset`` is never re-applied, and the
``"degrees"`` normalization keeps the T-B1/T-B2 affine-only behavior.

The output vector follows the frozen joint order below; the gripper is not a
kinematic joint and interference keys (gripper / base wheels / lift / other
arm) never leak into the result.
"""

from collections.abc import Mapping
from typing import Any

import numpy as np

# Frozen kinematic joint order (six joints; the gripper is NOT a kinematic
# joint). The output vector follows exactly this order, regardless of the
# insertion order of the observation dict or the metadata motors dict.
JOINT_ORDER: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)

# Explicit URDF joint-name mapping (Behavior T-B10, design §19): kinematic
# joint name -> joint name inside the shared single-arm source model. The
# right arm reuses the same left-arm source model, so both sides keep the
# ``left_*`` model names (only the device keys differ, ``arm_{side}_{joint}``).
# The table is explicit by design and must never be replaced by runtime
# prefix concatenation: ``wrist_yaw`` maps to ``left_wrist_yaw_joint`` (the
# ``_joint`` suffix), which prefix guessing would miss.
URDF_JOINT_NAMES: dict[str, str] = {
    "shoulder_pan": "left_shoulder_pan",
    "shoulder_lift": "left_shoulder_lift",
    "elbow_flex": "left_elbow_flex",
    "wrist_flex": "left_wrist_flex",
    "wrist_yaw": "left_wrist_yaw_joint",
    "wrist_roll": "left_wrist_roll",
}


# Supported metadata `normalization` values for the kinematic joints.
_RANGE_M100_100 = "range_m100_100"
_DEGREES = "degrees"

# Per-motor metadata fields required by the schema v2 envelope (design §9.1:
# the Cartesian adapter must not degrade-enable on incomplete metadata). The
# id/model fields are owned by the bus layer and are not validated here.
_REQUIRED_MOTOR_FIELDS: tuple[str, ...] = (
    "resolution",
    "range_min",
    "range_max",
    "normalization",
    "drive_mode",
    "homing_offset",
)

# Sides the kinematic adapter knows about. Signs/zeros are injected
# configuration (not runtime input), so an extra side key can only be a
# misspelling or a wrong joint set arriving from upstream assembly; it is
# rejected fail-fast at construction instead of becoming dead data (T-B11).
_VALID_SIDES: tuple[str, ...] = ("left", "right")

# Numeric motor fields whose type/finiteness is validated mode-agnostically
# (T-B7.b, F-1d precedent: bools must not sneak in via True == 1 and numeric
# strings must not be coerced). `homing_offset` is tick-valued firmware state
# that the adapter never consumes, so only its presence is required.
_NUMERIC_MOTOR_FIELDS: tuple[str, ...] = (
    "resolution",
    "range_min",
    "range_max",
    "drive_mode",
)


class JointKinematicAdapter:
    """Per-side unit adapter from motor degrees to kinematic joint degrees.

    Signs and zero offsets are injected per side at construction time
    (joint name -> value). Optionally, per-side per-joint URDF joint limits
    (side -> joint -> (lower, upper), in kinematic degrees) are injected as
    well; they form the mechanical safety gate on exported actions (T-B6).
    The conversion is direction-agnostic: it checks only finiteness and
    completeness of the six required observation keys and never zero-fills a
    missing joint.
    """

    def __init__(
        self,
        *,
        signs: Mapping[str, Mapping[str, float]],
        zeros: Mapping[str, Mapping[str, float]],
        urdf_limits: Mapping[str, Mapping[str, tuple[float, float]]] | None = None,
    ) -> None:
        """Validate the injected signs/zeros and URDF limits, then store copies."""
        self._validate_signs(signs)
        self._validate_zeros(zeros)
        self._signs: dict[str, dict[str, float]] = {side: dict(s) for side, s in signs.items()}
        self._zeros: dict[str, dict[str, float]] = {side: dict(z) for side, z in zeros.items()}
        self._urdf_limits: dict[str, dict[str, tuple[float, float]]] = {}
        if urdf_limits is not None:
            for side, joints in urdf_limits.items():
                side_limits: dict[str, tuple[float, float]] = {}
                for joint, window in joints.items():
                    side_limits[joint] = self._checked_urdf_window(
                        side=side, joint=joint, window=window
                    )
                self._urdf_limits[side] = side_limits

    def sign(self, side: str, joint: str) -> float:
        """Return the injected kinematic sign (+1 or -1) for ``side``/``joint``.

        The sign is the direction constant of the affine map
        ``q_kinematic = sign * motor_deg + zero`` used in both conversion
        directions.  Only the six kinematic joints are exposed; an unknown
        side or joint raises ``ValueError``.
        """
        if side not in self._signs:
            raise ValueError(f"Unknown side {side!r}; expected one of {sorted(self._signs)}")
        side_signs = self._signs[side]
        if joint not in side_signs:
            raise ValueError(
                f"Unknown joint {joint!r} for side {side!r}; expected one of {JOINT_ORDER}"
            )
        return float(side_signs[joint])

    @staticmethod
    def _validate_signs(signs: Mapping[str, Mapping[str, float]]) -> None:
        """Validate the injected per-side signs fail-fast at construction (T-B11).

        A sign is the direction constant of the affine map
        ``q = sign * motor_deg + zero``, so only +1/-1 is meaningful; any
        other gain silently turns the injected affine into a wrong
        gear-ratio assumption and voids every frozen conversion value.
        Bools are rejected before the numeric check (``True == 1`` would
        otherwise pass a value-only check) and numeric strings are rejected
        instead of being coerced.
        """
        for side in signs:
            if side not in _VALID_SIDES:
                raise ValueError(
                    f"Unknown side key {side!r} in signs; expected only {_VALID_SIDES}"
                )
        for side, joints in signs.items():
            for joint in JOINT_ORDER:
                if joint not in joints:
                    raise ValueError(f"signs are missing joint {joint!r} on side {side!r}")
                value = joints[joint]
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not np.isfinite(value)
                    or value not in (-1.0, 1.0)
                ):
                    raise ValueError(
                        f"Invalid sign {value!r} for joint {joint!r} on side {side!r}: "
                        f"expected +1 or -1 (bools and strings are not allowed)"
                    )

    @staticmethod
    def _validate_zeros(zeros: Mapping[str, Mapping[str, float]]) -> None:
        """Validate the injected per-side zero offsets fail-fast (T-B11).

        A zero offset is a constant of the injected affine, not a runtime
        measurement, so a NaN/inf offset must never reach a conversion:
        NaN zeros would silently poison the whole observation vector. Bools
        and numeric strings are rejected instead of being coerced.
        """
        for side in zeros:
            if side not in _VALID_SIDES:
                raise ValueError(
                    f"Unknown side key {side!r} in zeros; expected only {_VALID_SIDES}"
                )
        for side, joints in zeros.items():
            for joint in JOINT_ORDER:
                if joint not in joints:
                    raise ValueError(f"zeros are missing joint {joint!r} on side {side!r}")
                value = joints[joint]
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not np.isfinite(value)
                ):
                    raise ValueError(
                        f"Invalid zero offset {value!r} for joint {joint!r} on side "
                        f"{side!r}: expected a finite number (bools and strings are "
                        f"not allowed)"
                    )

    @staticmethod
    def _checked_urdf_window(
        *,
        side: str,
        joint: str,
        window: tuple[float, float],
    ) -> tuple[float, float]:
        """Validate one injected URDF limit window (fail-fast at construction).

        A limit gate that is non-finite, inverted, or zero-wide can never
        admit a command, so it is rejected before any conversion runs.
        """
        try:
            lower, upper = float(window[0]), float(window[1])
        except (TypeError, ValueError, IndexError) as exc:
            raise ValueError(
                f"Invalid URDF limit for joint {joint!r} on side {side!r}: {window!r}"
            ) from exc
        if not (np.isfinite(lower) and np.isfinite(upper)):
            raise ValueError(
                f"Non-finite URDF limit for joint {joint!r} on side {side!r}: {window!r}"
            )
        if lower > upper:
            raise ValueError(
                f"Inverted URDF limit for joint {joint!r} on side {side!r}: "
                f"lower {lower!r} > upper {upper!r}"
            )
        if lower == upper:
            raise ValueError(
                f"Zero-width URDF limit for joint {joint!r} on side {side!r}: {lower!r}"
            )
        return lower, upper

    @staticmethod
    def _validate_metadata_envelope(metadata: Mapping[str, Any]) -> None:
        """Reject any metadata envelope that is not a complete schema v2 document.

        Design §9.1: the Cartesian adapter requires the full v2 metadata and
        must not degrade-enable on a v1 document or on a missing/loosely
        matched ``schema_version`` (a string ``"2"`` or a bool is not v2).
        """
        schema_version = metadata.get("schema_version") if isinstance(metadata, Mapping) else None
        if schema_version != 2 or isinstance(schema_version, bool):
            raise ValueError(
                f"Unsupported metadata schema_version {schema_version!r}; "
                f"schema v2 is required and downgrade is not allowed"
            )

    @staticmethod
    def _validate_motor_entry(
        *,
        side: str,
        joint: str,
        entry: Mapping[str, Any],
        motor_name: str,
    ) -> None:
        """Validate one kinematic joint's metadata entry (T-B7.a/T-B7.b).

        Runs before the normalization-mode branch and before any field is
        consumed, so the checks are direction- and mode-agnostic: a missing
        required field or an unusable numeric value (None / numeric string /
        inf / bool) is rejected with a ValueError naming the side, the joint
        and the field, instead of a bare KeyError or a silent True == 1.
        Extra motor entries (gripper / base / lift) are never looked up here,
        so they are never validated either.
        """
        for field in _REQUIRED_MOTOR_FIELDS:
            if field not in entry:
                raise ValueError(
                    f"Metadata motor entry {motor_name!r} is missing required field "
                    f"{field!r} (side {side!r}, joint {joint!r})"
                )
        for field in _NUMERIC_MOTOR_FIELDS:
            value = entry[field]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not np.isfinite(value)
            ):
                raise ValueError(
                    f"Invalid numeric metadata field {field!r} for motor {motor_name!r} "
                    f"(side {side!r}, joint {joint!r}): {value!r}; expected a finite "
                    f"int or float (bools and strings are not allowed)"
                )

    @staticmethod
    def _motor_entry(
        *,
        side: str,
        joint: str,
        metadata: Mapping[str, Any],
        key: str,
    ) -> tuple[Mapping[str, Any], str]:
        """Fetch one joint's metadata motor entry and its normalization mode.

        Returns the entry and the normalization string. Only the six
        kinematic joints are ever looked up; interference motors (grippers,
        base wheels, lift, other arm) are never read.
        """
        motors = metadata.get("motors") if isinstance(metadata, Mapping) else None
        motor_name = f"arm_{side}_{joint}"
        if motors is None or motor_name not in motors:
            raise ValueError(f"Metadata is missing required motor entry {motor_name!r} ({key!r})")
        entry = motors[motor_name]
        JointKinematicAdapter._validate_motor_entry(
            side=side, joint=joint, entry=entry, motor_name=motor_name
        )
        mode = entry.get("normalization")
        if mode not in (_DEGREES, _RANGE_M100_100):
            raise ValueError(
                f"Unsupported normalization {mode!r} for motor {motor_name!r} "
                f"({key!r}); expected {_DEGREES!r} or {_RANGE_M100_100!r}"
            )
        return entry, mode

    @staticmethod
    def _check_range(entry: Mapping[str, Any], motor_name: str) -> None:
        """Mirror the bus' invalid-calibration guard (MotorsBus._normalize).

        The bus rejects a degenerate calibration window in every
        normalization mode (the check runs before the norm-mode branch), so
        the adapter mirrors it mode-agnostically on the export path (T-B6.d).
        """
        if entry["range_max"] == entry["range_min"]:
            raise ValueError(
                f"Invalid calibration for motor {motor_name!r}: min and max are equal."
            )

    @staticmethod
    def _check_finite_calibration(entry: Mapping[str, Any], motor_name: str) -> None:
        """Reject non-finite or unusable RANGE calibration before any math.

        Without this guard a non-finite range or resolution would escape as a
        bare ``OverflowError``/garbage value from the tick quantization below
        instead of a joint-naming ``ValueError`` before export (T-B6.g).
        """
        for field in ("range_min", "range_max", "resolution"):
            if not np.isfinite(entry[field]):
                raise ValueError(
                    f"Non-finite calibration field {field!r} for motor {motor_name!r}: "
                    f"{entry[field]!r}"
                )
        if entry["resolution"] < 2:
            raise ValueError(
                f"Calibration resolution for motor {motor_name!r} must be >= 2, "
                f"got {entry['resolution']!r}"
            )

    @staticmethod
    def _check_drive_mode(entry: Mapping[str, Any], motor_name: str) -> None:
        """Reject a drive_mode outside the bus' binary domain {0, 1} (F3).

        The bus only defines drive_mode 0 (normal) and 1 (inverted); the
        adapter's mirror consumes it via ``== 1``, so any other value would
        be silently treated as 0 while the bus truth diverges. Bools are
        already rejected earlier by the numeric-field validation.
        """
        if entry["drive_mode"] not in (0, 1):
            raise ValueError(
                f"Unsupported drive_mode for motor {motor_name!r}: {entry['drive_mode']!r}; "
                "expected 0 or 1"
            )

    def _effective_q_range(
        self,
        *,
        side: str,
        joint: str,
        entry: Mapping[str, Any],
        mode: str,
    ) -> tuple[float, float]:
        """Effective commandable window in q_deg space (endpoints inclusive).

        DEGREES mode: the bus' ``_unnormalize`` DEGREES branch applies no
        calibration clamp, so the converted calibration range is unbounded
        and the gate is the injected URDF limit alone. RANGE mode: the
        calibration tick window is mirrored to motor degrees and affinely
        mapped into q_deg space (same formula as the conversion below). The
        effective window is the intersection with the URDF limits; an empty
        intersection rejects every conversion (T-B6.c).
        """
        sign = self._signs[side][joint]
        zero = self._zeros[side][joint]
        if mode == _RANGE_M100_100:
            mid = (entry["range_min"] + entry["range_max"]) / 2
            w = entry["resolution"] - 1
            q_at_min = sign * (entry["range_min"] - mid) * 360.0 / w + zero
            q_at_max = sign * (entry["range_max"] - mid) * 360.0 / w + zero
            lo, hi = (q_at_min, q_at_max) if q_at_min <= q_at_max else (q_at_max, q_at_min)
        else:
            lo, hi = float("-inf"), float("inf")
        urdf = self._urdf_limits.get(side, {}).get(joint)
        if urdf is not None:
            lo = max(lo, urdf[0])
            hi = min(hi, urdf[1])
        if lo > hi:
            raise ValueError(
                f"Empty effective command range for joint {joint!r} on side {side!r}: "
                f"converted calibration range and URDF limits do not overlap"
            )
        return lo, hi

    def effective_limits_deg(
        self,
        side: str,
        metadata: Mapping[str, Any],
    ) -> dict[str, tuple[float, float]]:
        """Return the effective commandable limits for each kinematic joint.

        The returned mapping uses URDF joint names (``left_*``) to match the
        ``arm_ik.solve`` contract.  Each limit is the intersection of the
        converted calibration range (in ``range_m100_100`` mode) and the
        injected URDF limits, in kinematic degrees, endpoints inclusive.
        This is a thin public accessor over the existing
        :meth:`_effective_q_range` calculation.
        """
        if side not in self._signs or side not in self._zeros:
            raise ValueError(f"Unknown side {side!r}; expected one of {sorted(self._signs)}")
        self._validate_metadata_envelope(metadata)

        limits: dict[str, tuple[float, float]] = {}
        for joint in JOINT_ORDER:
            key = f"arm_{side}_{joint}.pos"
            entry, mode = self._motor_entry(side=side, joint=joint, metadata=metadata, key=key)
            lo, hi = self._effective_q_range(side=side, joint=joint, entry=entry, mode=mode)
            limits[URDF_JOINT_NAMES[joint]] = (lo, hi)
        return limits

    def observation_to_kinematic_deg(
        self,
        side: str,
        observation: Mapping[str, float],
        metadata: Mapping[str, Any],
    ) -> np.ndarray:
        """Convert one arm's motor-degree observation to kinematic degrees.

        Args:
            side: Arm side ("left" or "right"). Only this side's observation
                keys and injected sign/zero values are used; the other arm's
                keys, even if present, are never read.
            observation: Flat state dict with `{motor_name}.pos` keys in motor
                degrees, as reported by the DEGREES-mode bus.
            metadata: Robot metadata envelope (schema v2). The motor entry
                ``motors[arm_{side}_{joint}]`` selects the conversion: the
                ``"degrees"`` normalization means the observation already
                holds plain motor degrees (affine only), while
                ``"range_m100_100"`` means the observation holds normalized
                units that must be converted back to motor degrees through
                ``resolution`` / ``range_min`` / ``range_max`` / ``drive_mode``
                (mirroring ``MotorsBus._normalize``). Interference entries
                (gripper / base wheels / lift / other arm) are never read.

        Returns:
            Shape (6,) ndarray of kinematic joint degrees in the frozen
            ``JOINT_ORDER``, computed as ``sign * motor_deg + zero``.

        Raises:
            ValueError: If the side is unknown or any of the six required
                observation keys is missing or non-finite.
        """
        if side not in self._signs or side not in self._zeros:
            raise ValueError(f"Unknown side {side!r}; expected one of {sorted(self._signs)}")
        self._validate_metadata_envelope(metadata)

        signs = self._signs[side]
        zeros = self._zeros[side]

        kinematic_deg = np.empty(len(JOINT_ORDER), dtype=np.float64)
        for index, joint in enumerate(JOINT_ORDER):
            key = f"arm_{side}_{joint}.pos"
            if key not in observation:
                raise ValueError(f"Observation is missing required joint key {key!r}")
            motor_deg = float(observation[key])
            if not np.isfinite(motor_deg):
                raise ValueError(f"Observation value for {key!r} is not finite: {motor_deg!r}")
            entry, mode = self._motor_entry(side=side, joint=joint, metadata=metadata, key=key)
            if mode == _RANGE_M100_100:
                motor_name = f"arm_{side}_{joint}"
                self._check_range(entry, motor_name)
                # Same unusable-calibration guard as the export path, before
                # any math: a non-finite range/resolution would escape as a
                # bare ZeroDivisionError (resolution == 1) or garbage degrees
                # (resolution < 2) instead of a joint-naming ValueError (F2).
                self._check_finite_calibration(entry, motor_name)
                self._check_drive_mode(entry, motor_name)
                span = float(entry["range_max"] - entry["range_min"])
                # Undo the bus' drive_mode negation (MotorsBus._normalize),
                # recover the whole tick the bus quantized to, then degrees.
                value = -motor_deg if entry["drive_mode"] == 1 else motor_deg
                tick = round(((value + 100.0) / 200.0) * span + entry["range_min"])
                mid = (entry["range_min"] + entry["range_max"]) / 2
                motor_deg = (tick - mid) * 360.0 / (entry["resolution"] - 1)
            # mode == _DEGREES: the bus already reported plain motor degrees.
            kinematic_deg[index] = signs[joint] * motor_deg + zeros[joint]
        return kinematic_deg

    def kinematic_deg_to_action(
        self,
        side: str,
        q_deg: Any,
        metadata: Mapping[str, Any],
    ) -> dict[str, float]:
        """Convert kinematic joint degrees back to a motor command dict.

        Exact inverse of :meth:`observation_to_kinematic_deg` on the same
        per-side injected sign/zero values:

            motor_deg = sign * (q_deg - zero)

        Args:
            side: Arm side ("left" or "right"). Only this side's injected
                sign/zero values are used; the other arm is never read.
            q_deg: Kinematic joint degrees, one per joint in the frozen
                ``JOINT_ORDER``. The input is not modified.
            metadata: Robot metadata envelope (schema v2). The motor entry
                ``motors[arm_{side}_{joint}]`` selects the conversion: the
                ``"degrees"`` normalization means the command must be plain
                motor degrees (as in T-B2), while ``"range_m100_100"`` means
                the command must be a normalized value in [-100, 100],
                converted from motor degrees through ``resolution`` /
                ``range_min`` / ``range_max`` / ``drive_mode`` (mirroring
                ``MotorsBus._unnormalize``, with round-quantization to a
                whole tick so the bus' int() truncation lands exactly).
                Interference entries (other arm, grippers, base wheels,
                lift) never leak into the result. The input is not modified.

        Returns:
            Dict with exactly six keys ``arm_{side}_{joint}.pos`` (one per
            ``JOINT_ORDER`` joint, no gripper / base / other-arm key) mapping
            to motor degrees as plain floats. The ``.pos`` suffix matches the
            ``AlohaMini.send_action`` arm-key filter, so every produced key
            survives the production predicate.

        Raises:
            ValueError: If the side is unknown, if ``q_deg`` does not have
                exactly one value per joint (a short vector names the missing
                joint), if any value is NaN/Inf (naming the joint), if a
                calibration field is degenerate or non-finite (naming the
                joint), or if any joint value lies outside its effective
                command range — converted calibration range intersected with
                the injected URDF limits, endpoints inclusive (T-B6). The
                whole conversion is rejected: no partial dict is returned.
        """
        if side not in self._signs or side not in self._zeros:
            raise ValueError(f"Unknown side {side!r}; expected one of {sorted(self._signs)}")
        self._validate_metadata_envelope(metadata)

        values = np.asarray(q_deg, dtype=np.float64)
        if values.shape != (len(JOINT_ORDER),):
            if values.ndim == 1 and values.shape[0] < len(JOINT_ORDER):
                missing = JOINT_ORDER[values.shape[0]]
                raise ValueError(
                    f"q_deg has length {values.shape[0]}, expected {len(JOINT_ORDER)}; "
                    f"missing joint {missing!r} for side {side!r}"
                )
            raise ValueError(
                f"q_deg has shape {values.shape}, expected ({len(JOINT_ORDER)},) "
                f"(one value per joint, in JOINT_ORDER) for side {side!r}"
            )

        signs = self._signs[side]
        zeros = self._zeros[side]

        action: dict[str, float] = {}
        for index, joint in enumerate(JOINT_ORDER):
            value = float(values[index])
            if not np.isfinite(value):
                raise ValueError(
                    f"q_deg value for joint {joint!r} on side {side!r} is not finite: {value!r}"
                )
            joint_zero = zeros[joint]
            if not np.isfinite(joint_zero):
                raise ValueError(
                    f"Zero offset for joint {joint!r} on side {side!r} is not finite: "
                    f"{joint_zero!r}"
                )
            key = f"arm_{side}_{joint}.pos"
            entry, mode = self._motor_entry(side=side, joint=joint, metadata=metadata, key=key)
            motor_name = f"arm_{side}_{joint}"
            # Degenerate calibration is rejected mode-agnostically, before
            # the norm-mode branch (mirroring the bus, T-B6.d).
            self._check_range(entry, motor_name)
            if mode == _RANGE_M100_100:
                self._check_finite_calibration(entry, motor_name)
                self._check_drive_mode(entry, motor_name)
                # T-B6 gate: reject out-of-range commands on the input q_deg,
            # before any quantization — the bus' _unnormalize would silently
            # clamp the excess, which the adapter must not rely on.
            lo, hi = self._effective_q_range(side=side, joint=joint, entry=entry, mode=mode)
            if value < lo or value > hi:
                raise ValueError(
                    f"q_deg value {value!r} for joint {joint!r} on side {side!r} is outside "
                    f"the effective command range [{lo!r}, {hi!r}]"
                )
            motor_deg = signs[joint] * (value - joint_zero)
            if mode == _RANGE_M100_100:
                # Mirror MotorsBus._unnormalize's RANGE_M100_100 branch, but
                # round-quantize to a whole tick first so the bus' int()
                # truncation lands exactly on the quantized tick.
                span = float(entry["range_max"] - entry["range_min"])
                mid = (entry["range_min"] + entry["range_max"]) / 2
                tick = round(motor_deg * (entry["resolution"] - 1) / 360.0 + mid)
                normalized = ((tick - entry["range_min"]) / span) * 200.0 - 100.0
                # The bus' int() truncation of this exact encoding can land
                # one tick low when the calibration span is not a power of two
                # (the deg<->tick chain is then not bitwise exact; ~12% of
                # ticks at span 1551). Verify the exact encoding against the
                # bus' own truncation formula and, only when it would lose
                # the tick, re-encode at the midpoint of the bus' truncation
                # interval (half a normalized step, 100/span) so int() is
                # robust to the FP error. Python int() truncates toward zero,
                # so the midpoint offset is sign-aware on the bus-space value
                # t_cont = ((bus_value + 100)/200)*span + range_min: for
                # t_cont >= 0 the truncation interval is [T, T+1) with
                # midpoint T + 0.5 (+100/span); for t_cont < 0 it is
                # (T - 1, T] with midpoint T - 0.5 (-100/span). The drive_mode
                # flip below is undone by the bus' own negation, so the value
                # the bus truncates is this pre-flip `normalized`. Endpoints
                # (normalized == ±100) are always FP-lossless and keep the
                # exact encoding, so calibration endpoints land exactly (F1).
                bus_value = min(100.0, max(-100.0, normalized))
                t_cont = ((bus_value + 100.0) / 200.0) * span + entry["range_min"]
                if int(t_cont) != tick:
                    normalized += 100.0 / span if t_cont >= 0.0 else -100.0 / span
                if entry["drive_mode"] == 1:
                    normalized = -normalized
                # The bus can only consume [-100, 100] (it clips the same way).
                motor_deg = float(min(100.0, max(-100.0, normalized)))
            # mode == _DEGREES: the bus' DEGREES branch consumes plain degrees
            # and ignores drive_mode.
            action[key] = motor_deg
        return action
