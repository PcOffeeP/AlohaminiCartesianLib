"""Shared constants, unit conversion, stats, CSV I/O and capability probes."""

from __future__ import annotations

import contextlib
import csv
import math
import sys
from collections.abc import Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

from alohamini_cartesian.capability.detect import (
    DEFAULT_NORMALIZATION,
    METADATA_KEY,
    SPAN_BY_NORMALIZATION,
    CapabilityError,
    ConversionUnavailable,
    detect_from_observation,
    resolve_deg_per_unit,
)

# ---------------------------------------------------------------------------
# Frozen CLI contract.
# ---------------------------------------------------------------------------

EXIT_OK = 0
EXIT_STEP_RESPONSE_ABORTED = 1
EXIT_CONNECT_FAILED = 3
EXIT_WRITE_FAILED = 5
EXIT_MOTION_GATE_REFUSED = 6
EXIT_INSUFFICIENT_SAMPLES = 7
# The freeze command itself was rejected: unlike 1, the stop command never
# reached the host, so the robot may still be moving.
EXIT_HOLD_FAILED = 8
# The client cannot measure what this mode measures, in either of the two shapes
# that refusal takes: it publishes no ``latest_request_timing`` at all, or it
# publishes it without the keys rtt reads holding usable values (absent, ``None``,
# non-finite, a ``bool`` stamp/age, a negative ``age_s``, a ``token`` that is not
# text or is blank -- a half-filled face is refused as well as an all-empty one,
# and a present-but-unusable value as well as an absent one).  Both mean
# there is no request age to record, so the rtt run is refused instead of writing
# a CSV of empty timing cells -- and one code covers all of them, because the
# caller sees one outcome and one remedy.  Three moments share the code and are
# named apart on stderr:
#   * ``missing_client_capabilities`` asks "is the face there?" in ``main``, after
#     ``connect()`` returned and ahead of ``_prime_state_only_window``, so a client
#     without the face is refused before a request window is spent on it;
#   * ``unfilled_client_capabilities`` asks "does the face carry
#     ``REQUIRED_REQUEST_TIMING_KEYS``?" in ``collect_rtt_samples``, on every
#     growth of ``observation_sequence`` and ahead of that frame's
#     ``rows.append``: a fresh frame is what makes an unfilled face tellable from
#     one the host has not timed yet, and probing before the append is what keeps
#     an empty row -- and the CSV header -- from being written;
#   * the same probe runs once more at the moment the loop gives up: a frozen
#     ``observation_sequence`` never grows, so the per-growth probe never fires,
#     and without the give-up probe an untimed face would be reported as a
#     sampling-budget problem (7) instead of a capability one (9).  A face that
#     *does* carry the keys still ends in 7 there.
EXIT_CLIENT_UNSUPPORTED = 9

# One commanded step never exceeds this many degrees.
MAX_STEP_DELTA_DEG = 1.0
# The host control loop is bounded above by this rate, which bounds the phase
# a request can spend waiting inside one host period.
HOST_LOOP_HZ_UPPER_BOUND = 50.0
# Product of the per-step and per-run caps: a tracking run may travel this far.
MAX_TOTAL_TRAVEL_DEG = 20.0
# What a tracking step lays its goal on.  ``present`` re-reads the measured
# position every step and drives ``delta_deg`` above it, so the lead never
# exceeds one step and an error cannot accumulate over steps; ``command``
# anchors on the target the previous step sent (the first step falls back to the
# measured position), so the lead accumulates and reproduces the
# error-accumulating shape of a teleoperated run.
ANCHOR_PRESENT = "present"
ANCHOR_COMMAND = "command"
ANCHOR_CHOICES = (ANCHOR_PRESENT, ANCHOR_COMMAND)
DEFAULT_ANCHOR = ANCHOR_PRESENT
# Consecutive ``get_observation`` calls whose ``observation_sequence`` did not
# grow before a tracking run gives up on the link.  The client answers a timed
# out observation with its cached frame instead of raising, so growth of that
# counter is the only liveness signal there is.
FEEDBACK_STALL_LIMIT = 3
# Abort reason of a dead feedback link: names the signal and the limit so the
# report and the CSV say *why* the probe stopped.
STALE_FEEDBACK_REASON = (
    f"feedback stale: observation_sequence did not grow for {FEEDBACK_STALL_LIMIT} polls"
)
# Abort reason of an operator interrupt; ``main`` reports it on stderr.
INTERRUPT_ABORT_REASON = "interrupted by the operator (KeyboardInterrupt) before the run finished"

DEFAULT_PORT = 5556
# AlohaMini 2 Pro has 7 joints per arm (it adds ``wrist_yaw``); the client
# default ``alohamini1`` would mis-name every motor, so the benchmark defaults
# to the Pro model and lets the operator override it.
DEFAULT_ROBOT_MODEL = "alohamini2pro"
DEFAULT_MODE = "rtt"
DEFAULT_RATE_HZ = 30.0
# Command rate of the tracking loop: two consecutive commands are at least
# ``1 / DEFAULT_CONTROL_HZ`` apart.  Independent of ``DEFAULT_RATE_HZ``, which is
# the observation sampling rate.  50 Hz matches the host loop bound and keeps the
# host's ~1 s command watchdog fed.
DEFAULT_CONTROL_HZ = 50.0
# Below this sampling rate the idle time one response spends waiting in the
# client socket for the next poll dominates ``age_s``, so the net round-trip
# estimate stops being a network measurement (see module docstring, "Sampling
# pace vs. network round trip").  Only warns; nothing changes in the rows.
RTT_RATE_WARNING_HZ = 200.0
# Lower bound on the wait of a poll that grew no row, for every period: a direct
# caller can hand in a ``rate_hz`` that is non-finite or non-positive (a wait of
# ``0.0`` would reopen the full-speed ``get_observation`` poll the idle arm
# exists to prevent), and a finite but huge rate (``--rate-hz 1e9``) gives a
# ``period_s`` far below what the wall clock honours.  Both are clamped to this
# floor, so no idle wait is shorter than it.
# 1 ms is the *lower* bound and not a second sampling rate: it sits two orders
# below the 20 ms host loop period and below every period this measurement uses,
# so a loop waiting 1 ms already polls at 1 kHz -- such a caller has no request
# rhythm to follow, and repeating frames are not worth polling that finely.
IDLE_POLL_FLOOR_S = 1e-3
DEFAULT_SAMPLES = 500
DEFAULT_TIMEOUT_S = 30.0
DEFAULT_DELTA_DEG = 1.0
DEFAULT_STEPS = 20
DEFAULT_SETTLE_TIMEOUT_S = 1.0
DEFAULT_SETTLE_TOLERANCE_DEG = 0.15
# Mirrors ``AlohaMiniClientConfig.observation_request_window``, but the default
# here is ``1``: the client consumes the OLDEST token of the window and refills
# it after the answer, so with a window of 3 the measured age carries
# ``window - 1`` client periods of prefetch wait that are not a round trip.
# ``1`` is the RTT measurement shape; ``3`` is the deployment shape and is an
# explicit opt-in.
DEFAULT_REQUEST_WINDOW = 1
# The phase bound defaults to the upper bound of the host loop, so a run that
# does not measure the host rate keeps the round-1 numbers.
DEFAULT_HOST_LOOP_HZ = HOST_LOOP_HZ_UPPER_BOUND

ARTIFACT_DIR = Path("artifacts") / "alohamini-io"

RTT_CSV_FIELDS = (
    "sample_index",
    "token",
    "observation_sequence",
    "sent_monotonic_s",
    "received_monotonic_s",
    "age_s",
    "host_sample_duration_s",
    "phase_bound_s",
    "phase_mean_s",
    "queue_bound_s",
    "queue_mean_s",
    "prefetch_bound_s",
    "prefetch_mean_s",
    "host_timing_missing",
    "net_rtt_est_s",
    "net_rtt_low_s",
    "net_rtt_high_s",
    "state_sample_unix_ns",
)

TRACKING_CSV_FIELDS = (
    "step_index",
    "t_send_monotonic_s",
    "received_monotonic_s",
    "cmd_deg",
    "cmd_units",
    "deg_per_unit",
    "cmd_echo_deg",
    "baseline_deg",
    "measured_deg",
    "error_deg",
    "rise_90_time_s",
    "overshoot_deg",
    "settled",
    "note",
)

# Sidecar columns of the tracking mode: one row per motor per fresh safety frame
# (``safety_frame_rows``).  Deliberately NOT appended to ``TRACKING_CSV_FIELDS``:
# that table is frozen at 14 columns because downstream evidence files parse it
# as 14 columns, so the safety snapshot gets its own file (``safety_csv_path``).
# ``target_source`` is the ninth and last column, appended after ``hold_active``:
# ``hold_active`` is structurally ``False`` on the pinned vendor client (nothing
# writes ``joint_holds``), so it cannot answer "is this joint being held back";
# the whole-frame ``target_source`` plus the ``current_ma`` trend can.
SAFETY_CSV_FIELDS = (
    "step_index",
    "sample_sequence",
    "t_monotonic_s",
    "motor",
    "current_ma",
    "requested_target",
    "accepted_target",
    "hold_active",
    "target_source",
)
# The four values the vendor frame may carry in ``target_source`` (a WHOLE-frame
# attribute: which layer produced this frame's targets).  Only these four strings
# are written to the sidecar; anything else is an empty cell, never a guess and
# never a defaulted value (``docs/ADR.md`` D18).
TARGET_SOURCE_VALUES = frozenset({"none", "command", "watchdog", "protection"})

# Base body velocities are pinned to zero in every command: a command that
# omits them would leave the base chasing whatever was last sent.
BASE_VELOCITY_KEYS = ("x.vel", "y.vel", "theta.vel")
# Lift height key of the host action packet.  It is pinned to the latest
# measured height when the feedback carries it (see ``run_tracking_hold``).
LIFT_HEIGHT_KEY = "lift_axis.height_mm"
# Suffix of every arm/gripper position key of an observation.
POS_SUFFIX = ".pos"
# ``MotorNormMode`` values of ``lerobot.motors.motors_bus`` as the host publishes
# them in ``motors.<motor>.normalization``.  Spelled out on purpose: this harness
# must not import ``lerobot`` at module scope.
NORMALIZATION_RANGE_M100_100 = "range_m100_100"
NORMALIZATION_RANGE_0_100 = "range_0_100"
NORMALIZATION_DEGREES = "degrees"
# Width of the normalized interval each range mode maps the tick range onto (see
# ``MotorsBus._normalize``).  Arm joints inherit ``RANGE_M100_100``; every
# ``gripper`` entry of ``_ARM_PROFILES`` is explicitly ``RANGE_0_100``, so the
# same ticks are worth twice the degrees on the gripper.
SPAN_M100_100 = 200.0
SPAN_0_100 = 100.0
# ``schema_version`` of the host robot metadata this harness reads.  Version 2 is
# the layout that publishes ``motors.<motor>.normalization`` next to the
# calibration triple; ``capture_robot_fingerprint.validate_metadata`` refuses any
# other value before it stores a fingerprint, and Gate F applies the same rule.
HOST_METADATA_SCHEMA_VERSION = 2
# Client attributes rtt mode cannot run without.  ``latest_request_timing`` is the
# face the request age is measured from; a vendor client that publishes only a
# semantically different neighbour (``latest_safety_status``) is refused with
# ``EXIT_CLIENT_UNSUPPORTED`` rather than sampled into empty columns.
CLIENT_CAPABILITIES = ("latest_request_timing",)
# Client attribute the tracking *sidecar* reads (``safety_frame_rows``).  A second
# tuple beside ``CLIENT_CAPABILITIES`` on purpose: the safety frame is a different
# capability from the request timing, a client may publish one without the other,
# and neither answer may stand in for the other.
#
# Substitution is not what this is: ``docs/ADR.md`` D12 forbids reading
# ``latest_safety_status`` in place of ``latest_control_status`` (different
# frames, different structure), and this harness does not do that.  The safety
# face lands in its OWN sidecar columns, and every ``latest_control_status`` read
# path is unchanged, line for line.  The name lives in its own tuple so a reader
# can tell the two faces apart at the definition as well as at the call site.
SAFETY_CLIENT_CAPABILITIES = ("latest_safety_status",)
# The keys of ``latest_request_timing`` rtt mode actually consumes, i.e. the
# *necessary* set a face has to carry before a row may be built from it.  Taken
# from the consumers, never from "whatever the client happens to publish": a
# newer client may add fields, and an extra key must not refuse a run.
#   * ``_request_row`` (measure_latency.py:1351, token at :1362, stamps at
#     :1378-1379, age at :1380) reads ``token``, ``sent_monotonic_s``,
#     ``received_monotonic_s`` and ``age_s``;
#   * ``net_rtt_estimate_s`` (measure_latency.py:830) reads ``age_s``.
# The host stamps that estimator also subtracts live on ``latest_host_timing``,
# which is a different face, so they are not in this set.
REQUIRED_REQUEST_TIMING_KEYS = ("token", "sent_monotonic_s", "received_monotonic_s", "age_s")


# The class name is frozen by tests/benchmarks: renaming it would break the
# contract, so the Error suffix lint rule is waived here.
class InsufficientSamples(RuntimeError):  # noqa: N818
    """Raised when fewer fresh observations than requested arrived in time."""

    def __init__(self, *, required: int, collected: int) -> None:
        super().__init__(
            f"collected {collected} of {required} requested samples before the timeout"
        )
        self.required = required
        self.collected = collected


# Same naming rule: the exception name is part of the contract and has to carry
# ``unit`` so a caller can tell a missing calibration from a missing joint.
class UnitConversionUnavailable(RuntimeError):  # noqa: N818
    """Raised when the degree value of one normalized ``.pos`` unit is unknown.

    Never caught to fall back to a guessed factor: a wrong factor rescales every
    motion limit by an unknown amount, in an unknown direction.
    """


# ---------------------------------------------------------------------------
# Small helpers.
# ---------------------------------------------------------------------------


def _finite_or_none(value: Any) -> float | None:
    """Return ``value`` as a finite float, or ``None`` when that is impossible."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _request_token_text(token: Any) -> str:
    """The text ``_request_row`` writes into the ``token`` cell.

    Shared with the half-filled probe on purpose: a token that converts to an
    empty string is exactly the token that lands as an empty CSV cell, so
    "is it filled?" has to judge the same text the row writes.
    """
    if isinstance(token, bytes):
        # The client stores the request token as bytes; decode before it reaches
        # a dict/CSV so the cell is text and not ``b'...'``.
        return token.decode("ascii", errors="replace")
    if token is None:
        return ""
    return str(token)


# Gate of the quoted value in the missing-key clause: the clause is one line of
# refusal, so a quote longer than a CSV cell (32 characters) is a dumped object, not
# evidence the operator can read.  Past this length the quote is dropped and the
# reason stays (`_short_value`).
_QUOTED_VALUE_MAX_CHARS = 32


def _short_value(value: Any) -> str | None:
    """``value``'s ``repr`` when it can be quoted in the key list, else ``None``.

    ``None`` means "drop the quote, keep the reason": only the quoted value is
    optional in the annotation, never the reason itself.

    The clause is split on ``", "`` to read the key names back out of it
    (``_missing_key_names`` in tests/benchmarks) and cut at the first newline
    (``_missing_key_clause``), so quoting a value is only legal when the quote is
    **unchanged**: ``"1.0,  2.0"`` would arrive as one more key, a multi-line
    ``repr`` would end the list mid-word and push the ``, so ...`` remediation off
    it, and a ``repr`` longer than ``_QUOTED_VALUE_MAX_CHARS`` would dump an object
    into a one-line refusal.  ``repr`` that raises (``__repr__`` is client code)
    cannot be quoted either -- and that shape must not become a transport failure:
    the refusal is about a value, not about the link.

    Re-writing the quote instead (normalising its whitespace, deleting the
    whitespace after a comma) does **not** fix the key list: the quote sits inside
    the annotated entry (``token (empty text ('   '))``), so any quote at all
    makes the clause read as a different key name.  Hence the gate: a value whose
    ``repr`` is not already a short, printable, `", "``-free one-liner is not
    quoted, and the reason is printed alone.
    """
    try:
        text = repr(value)
    except Exception:
        # ``__repr__`` is client code: a proxy over a closed resource, a lazily
        # formatted object.  Letting it escape turned the refusal into
        # "observation failed" (exit 3), i.e. a broken *link* reported for a broken
        # *value*, and the key name was lost with it.
        return None
    if len(text) > _QUOTED_VALUE_MAX_CHARS or ", " in text or not text.isprintable():
        return None
    return text


def _timing_key_problem(key: str, value: Any) -> str | None:
    """Why ``value`` cannot fill ``key`` of the timing face, or ``None`` when it can.

    The criterion is "a finite value that physically means a measurement", not
    "a value ``float()``/``str()`` accepts": every shape refused here converts
    happily, which is how it used to reach the CSV as a cell that reads like a
    measurement (see ``missing_request_timing_keys`` for the four rules).  Only
    the timing face is judged this way; ``_finite_or_none`` keeps its wider
    contract for the other consumers (``net_rtt_estimate_s``,
    ``_checked_unit_scale``, the CLI paths).

    The reason carries **no value**: quoting the offending value is a separate
    decision (``_short_value``), because a quote that cannot be printed unchanged
    has to be dropped instead.
    """
    if key == "token":
        # ``bytes`` stays usable because ``_request_token_text`` decodes it -- that
        # is the shape the vendor client stores.  Anything else only reaches the
        # cell through ``str()``, i.e. a number or an object is not a token.
        if not isinstance(value, (str, bytes)):
            return "not a string"
        if not _request_token_text(value).strip():
            return "empty text"
        return None
    # ``bool`` is an ``int`` in Python, so it passes ``float()``; a timing cell of
    # ``True``/``False`` became 1.0/0.0, a stamp at the start of the epoch.
    if isinstance(value, bool):
        return "bool is not a number"
    try:
        number = _finite_or_none(value)
    except ArithmeticError:
        # ``float(10**400)`` raises ``OverflowError``, which ``_finite_or_none``
        # does not catch (it only turns ``TypeError``/``ValueError`` into ``None``).
        # An integer too large to be a double is not a finite number either, and
        # letting it escape made the probe's caller read it as a transport failure
        # (exit 3) instead of a capability one (exit 9).
        number = None
    if number is None:
        return "not a finite number"
    # A negative age is physically impossible (a request cannot be answered
    # before it was sent) and ``net_rtt_estimate_s`` clamps the net estimate up to
    # zero, so the row would read like a fast round trip.  ``0.0`` is legal.
    if key == "age_s" and number < 0.0:
        return "negative value"
    return None


def _request_timing_key_problems(timing: Any) -> list[tuple[str, str | None]]:
    """``(key, annotation)`` for ``REQUIRED_REQUEST_TIMING_KEYS`` entries ``timing`` cannot fill.

    The second element is what goes in parentheses after the key name, or
    ``None`` for a key the clause spells bare.  ``None`` is reserved for the two
    shapes that have nothing to explain, because they are spelled the same way and
    a half-filled face must keep listing bare key names:

    * the key is absent from the face;
    * the key is present but ``None`` -- the cell would be empty because the value
      is not there, just one step earlier.

    A key that **is** there with an unusable value always carries a reason
    (``"<reason>"``); the quoted value is appended only when ``_short_value``
    says it can be printed unchanged (``"<reason> (<quoted value>)"``).  The two
    failures that shaped this contract cannot be traded against each other: a bare
    key sends the operator to "this cell is missing" for a cell the face does
    carry, and a quote that holds ``", "`` or a line break splits the key list (a
    key that does not exist) or ends it mid-word.  Dropping only the quote keeps
    both sides intact.
    """
    if not isinstance(timing, Mapping):
        return [(key, None) for key in REQUIRED_REQUEST_TIMING_KEYS]
    problems: list[tuple[str, str | None]] = []
    for key in REQUIRED_REQUEST_TIMING_KEYS:
        if key not in timing or timing[key] is None:
            problems.append((key, None))
            continue
        reason = _timing_key_problem(key, timing[key])
        if reason is None:
            continue
        quoted = _short_value(timing[key])
        problems.append((key, reason if quoted is None else f"{reason} ({quoted})"))
    return problems


def _observation_sequence(client: Any) -> int | None:
    """The client's successful-receive counter, or ``None`` when it has none.

    This is the only liveness signal of the link: the client serves its cached
    frame instead of raising when the observation transport times out, so a
    counter that stops growing is what a dead link looks like.
    """
    try:
        return int(client.observation_sequence)
    except (AttributeError, TypeError, ValueError):
        return None


def _motor_name(joint_key: str) -> str:
    """Motor name of an observation key: ``--joint`` with or without ``.pos``."""
    name = str(joint_key).strip()
    return name[: -len(POS_SUFFIX)] if name.endswith(POS_SUFFIX) else name


def _checked_unit_scale(scale: Any) -> float:
    """``scale`` as a finite, strictly positive deg-per-unit factor."""
    number = _finite_or_none(scale)
    if number is None or number <= 0.0:
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: deg per unit must be finite and positive, got {scale!r}"
        )
    return number


def _checked_anchor(anchor: Any) -> str:
    """``anchor`` as one of ``ANCHOR_CHOICES``.

    Checked before the first observation and the first command of a tracking run:
    a run whose aim cannot be honoured may not spend a request window on it, and a
    robot that received one command of an unknown-shape run is exactly the case
    this refuses.
    """
    if anchor not in ANCHOR_CHOICES:
        raise ValueError(f"anchor must be one of {ANCHOR_CHOICES} (got {anchor!r})")
    return str(anchor)


def deg_per_unit(metadata: Any, joint_key: str) -> float:
    """Degrees covered by one normalized ``.pos`` unit of ``joint_key``.

    The span is the ``normalization`` the host publishes for the motor:
    ``_normalize`` maps the tick range onto ``200`` units for
    ``range_m100_100`` (every arm joint) and onto ``100`` for ``range_0_100``
    (the gripper), so one unit spans ``(range_max - range_min) / span`` ticks and

        deg_per_unit = ((range_max - range_min) / span) * 360 / (resolution - 1)

    An absent ``normalization`` is read as ``range_m100_100`` (metadata written
    before the field existed carries only the calibration triple), while an
    unrecognised one is refused.  ``degrees`` is already a degree and returns
    ``1.0`` before any calibration value is judged.

    The calibration is read from the host robot metadata
    (``motors.<motor>.range_min`` / ``range_max`` / ``resolution``).  Anything
    missing, non-finite or self-contradictory raises ``UnitConversionUnavailable``
    instead of falling back to ``1.0``: a guessed factor would silently rescale
    every degree limit of the run.
    """
    motor = _motor_name(joint_key)
    if not isinstance(metadata, Mapping):
        raise UnitConversionUnavailable(
            "unit conversion unavailable: host metadata is not a mapping "
            f"(got {type(metadata).__name__})"
        )
    motors = metadata.get("motors")
    if not isinstance(motors, Mapping) or not motors:
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: host metadata carries no motor calibration for {motor!r}"
        )
    entry = motors.get(motor)
    if not isinstance(entry, Mapping):
        raise UnitConversionUnavailable(
            "unit conversion unavailable: host metadata has no calibration entry "
            f"for motor {motor!r}"
        )
    normalization = entry.get("normalization")
    # ``degrees`` is already a degree: there is no span, no resolution and no
    # interval to divide, so even a degenerate calibration is usable as is.
    if normalization == NORMALIZATION_DEGREES:
        return 1.0
    if normalization is not None and normalization not in SPAN_BY_NORMALIZATION:
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: motor {motor!r} reports the unsupported normalization "
            f"{normalization!r}; expected {NORMALIZATION_RANGE_M100_100!r}, "
            f"{NORMALIZATION_RANGE_0_100!r} or {NORMALIZATION_DEGREES!r}"
        )
    range_min = _finite_or_none(entry.get("range_min"))
    range_max = _finite_or_none(entry.get("range_max"))
    resolution = _finite_or_none(entry.get("resolution"))
    missing = [
        name
        for name, value in (
            ("range_min", range_min),
            ("range_max", range_max),
            ("resolution", resolution),
        )
        if value is None
    ]
    if missing:
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: calibration of motor {motor!r} is missing {missing} "
            "(or they are not finite)"
        )
    if range_max <= range_min:
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: calibration of motor {motor!r} has "
            f"range_max={range_max!r} <= range_min={range_min!r}"
        )
    if resolution <= 1.0:
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: calibration of motor {motor!r} has "
            f"resolution={resolution!r}, which spans no travel"
        )
    # Reuse the single source of truth in ``alohamini_cartesian.capability.detect``
    # for all real integer-tick calibrations; fall back to the shared span table
    # only for synthetic non-integer float triples in unit tests.
    if range_min.is_integer() and range_max.is_integer() and resolution.is_integer():
        norm_meta: dict[str, Any] = {
            **metadata,
            "schema_version": HOST_METADATA_SCHEMA_VERSION,
        }
        try:
            caps = detect_from_observation({METADATA_KEY: norm_meta})
            return resolve_deg_per_unit(caps, motor).deg_per_unit
        except (CapabilityError, ConversionUnavailable) as exc:
            raise UnitConversionUnavailable(f"unit conversion unavailable: {exc}") from exc
    span = SPAN_BY_NORMALIZATION[DEFAULT_NORMALIZATION if normalization is None else normalization]
    return ((range_max - range_min) / span) * 360.0 / (resolution - 1.0)


def units_from_deg(deg: float, scale: float) -> float:
    """Normalized ``.pos`` units equivalent to ``deg`` degrees on this joint."""
    return float(deg) / _checked_unit_scale(scale)


def deg_from_units(units: float, scale: float) -> float:
    """Degrees covered by ``units`` normalized ``.pos`` units on this joint."""
    return float(units) * _checked_unit_scale(scale)


def percentile(values: Sequence[float] | np.ndarray, pct: float) -> float:
    """Linear-interpolated percentile; ``nan`` for an empty sequence.

    Same contract as ``_percentile`` in ``benchmarks/alohamini_ik/benchmark_arm_ik.py``.
    """
    if len(values) == 0:
        return float("nan")
    return float(np.percentile(values, pct))


def _quantiles(values: Sequence[float]) -> dict[str, float | int]:
    clean = [
        number for number in (_finite_or_none(value) for value in values) if number is not None
    ]
    if not clean:
        nan = float("nan")
        return {"count": 0, "min": nan, "p50": nan, "p95": nan, "p99": nan, "max": nan}
    return {
        "count": len(clean),
        "min": min(clean),
        "p50": percentile(clean, 50.0),
        "p95": percentile(clean, 95.0),
        "p99": percentile(clean, 99.0),
        "max": max(clean),
    }


def summary_stats(values_ms: Sequence[float]) -> dict[str, float | int]:
    """count/min/p50/p95/p99/max over millisecond samples."""
    stats = _quantiles(values_ms)
    return {
        "count": stats["count"],
        "min_ms": stats["min"],
        "p50_ms": stats["p50"],
        "p95_ms": stats["p95"],
        "p99_ms": stats["p99"],
        "max_ms": stats["max"],
    }


def host_sample_duration_s(host_timing: Mapping[str, Any] | None) -> float | None:
    """Host-internal sampling duration, or ``None`` when the stamps are unusable.

    Only the difference of the two host stamps is taken, never a difference
    against a PC stamp: the host monotonic clock is a different clock domain.
    """
    if not isinstance(host_timing, Mapping):
        return None
    started_s = _finite_or_none(host_timing.get("state_sample_started_monotonic_s"))
    finished_s = _finite_or_none(host_timing.get("state_sample_finished_monotonic_s"))
    if started_s is None or finished_s is None:
        return None
    duration_s = finished_s - started_s
    if duration_s < 0.0:
        # A finished-before-started pair means the host timing face is unusable
        # (clock skew, stale dict, swapped keys); fail safe instead of inflating
        # the estimate with a negative correction.
        return None
    return duration_s


def _host_loop_hz_or_upper_bound(host_loop_hz: float) -> float:
    """``host_loop_hz`` when it can serve as a period source, else the upper bound.

    The one normalisation every consumer of the knob goes through.  The CSV path
    already fell back to ``HOST_LOOP_HZ_UPPER_BOUND`` here while
    ``_format_rtt_report`` handed the raw value to ``effective_request_window``,
    whose gate is ``rate_hz >= host_loop_hz``: a direct caller's ``nan`` was read
    as 50 Hz by the rows and as "the request rate is never at or above the host
    rate" by the report, so one run printed ``queue_bound_s=0.04`` in the CSV and
    ``queue_bound_ms=0.000`` in the report that must not contradict it.  The CLI
    rejects a non-finite ``--host-loop-hz`` outright (exit 2), so this only
    decides what a direct caller sees -- and a finite positive value is returned
    unchanged, so no printed number moves.  The positive test is load-bearing:
    ``nan`` fails it (every comparison is false), a ``<= 0.0`` test would not.
    """
    return host_loop_hz if host_loop_hz > 0.0 else HOST_LOOP_HZ_UPPER_BOUND


def _queue_terms_s(
    request_window: int, host_loop_hz: float = HOST_LOOP_HZ_UPPER_BOUND
) -> tuple[float, float]:
    """Bound and mean of the host queueing time for ``request_window`` requests in flight."""
    loop_hz = _host_loop_hz_or_upper_bound(host_loop_hz)
    queued_requests = max(0, int(request_window) - 1)
    queue_bound_s = queued_requests / loop_hz
    return queue_bound_s, queue_bound_s / 2.0


def effective_request_window(request_window: int, rate_hz: float, host_loop_hz: float) -> int:
    """Requests that are really queued ahead of the answered one.

    The client refills its observation window after each consumed frame, so the
    window only stays full while requests arrive at least as fast as the host
    serves them.  Asking slower than the host drains the queue, and then the
    ``window - 1`` periods of queueing simply do not exist: subtracting them
    anyway structurally pulls a healthy LAN estimate down to zero.

    The comparison uses the *requested* rate (``--rate-hz``) and not the
    achieved one, so the term a row reports is deterministic and the report
    cannot contradict the CSV.
    """
    if rate_hz >= host_loop_hz:
        return int(request_window)
    return 1


def _prefetch_terms_s(
    request_window: int, rate_hz: float | None, host_loop_hz: float
) -> tuple[float, float]:
    """Bound and mean of the prefetch wait of ``request_window`` in-flight requests.

    ``_poll_and_get_latest_message`` consumes the *oldest* token and refills the
    window only after the answer arrived, so the answered request was sent
    ``window`` client calls ago: ``window - 1`` of those calls are pure prefetch
    wait.  One call takes at least ``max(1 / rate_hz, 1 / host_loop_hz)`` — the
    client cannot beat the host it waits for.

    ``rate_hz=None`` means "the client rate is unknown": the term is zero so a
    caller without a rate reproduces the round-3 numbers exactly.  Unlike the
    queue term, this one is NOT gated on the request rate: the wait exists even
    when the client is the slower side (that is the case where it dominates).
    """
    if rate_hz is None:
        return 0.0, 0.0
    loop_hz = _host_loop_hz_or_upper_bound(host_loop_hz)
    rate = _finite_or_none(rate_hz)
    # A non-positive client rate is rejected by the CLI; if it ever reaches here
    # the host period is the only defensible period of a client call.
    period_s = 1.0 / loop_hz if rate is None or rate <= 0.0 else max(1.0 / rate, 1.0 / loop_hz)
    bound_s = max(0, int(request_window) - 1) * period_s
    return bound_s, bound_s / 2.0


def net_rtt_estimate_s(
    age_s: Any,
    host_timing: Mapping[str, Any] | None,
    *,
    host_loop_hz: float = HOST_LOOP_HZ_UPPER_BOUND,
    request_window: int = 1,
    rate_hz: float | None = None,
) -> dict[str, Any]:
    """Estimate the network round trip from the request age (see module docstring).

    Returns the point estimate plus the ``low``/``high`` bounds that bracket it.
    ``request_window`` is the number of observation requests the client keeps in
    flight: ``window - 1`` of them are queued ahead of the answered one (the
    queue terms, gated on ``rate_hz >= host_loop_hz``) and the same ``window - 1``
    requests are prefetched ahead of it (the prefetch terms, ungated).  The
    default ``request_window=1`` queues and prefetches nothing, so the plain
    round-1 numbers are reproduced exactly; ``rate_hz=None`` switches the
    prefetch term off and reproduces the round-3 numbers instead.
    """
    loop_hz = _host_loop_hz_or_upper_bound(host_loop_hz)
    phase_bound_s = 1.0 / loop_hz
    phase_mean_s = phase_bound_s / 2.0
    window = _finite_or_none(request_window)
    raw_window = 1 if window is None else int(window)
    rate = _finite_or_none(rate_hz)
    queued_window = (
        raw_window if rate is None else effective_request_window(raw_window, rate, loop_hz)
    )
    queue_bound_s, queue_mean_s = _queue_terms_s(queued_window, loop_hz)
    prefetch_bound_s, prefetch_mean_s = _prefetch_terms_s(raw_window, rate, loop_hz)
    sample_duration_s = host_sample_duration_s(host_timing)
    age = _finite_or_none(age_s)

    estimate: dict[str, Any] = {
        "age_s": age,
        "host_sample_duration_s": sample_duration_s,
        "phase_bound_s": phase_bound_s,
        "phase_mean_s": phase_mean_s,
        "queue_bound_s": queue_bound_s,
        "queue_mean_s": queue_mean_s,
        "prefetch_bound_s": prefetch_bound_s,
        "prefetch_mean_s": prefetch_mean_s,
        "host_timing_missing": sample_duration_s is None,
    }
    if age is None:
        nan = float("nan")
        estimate.update(net_rtt_est_s=nan, net_rtt_low_s=nan, net_rtt_high_s=nan)
    elif sample_duration_s is None:
        # Nothing can be subtracted without the host duration: the age itself is
        # the upper bound, and the estimate says it is one.
        estimate.update(net_rtt_est_s=age, net_rtt_low_s=age, net_rtt_high_s=age)
    else:
        estimate.update(
            net_rtt_est_s=max(
                0.0,
                age - sample_duration_s - phase_mean_s - queue_mean_s - prefetch_mean_s,
            ),
            net_rtt_low_s=max(
                0.0,
                age - sample_duration_s - phase_bound_s - queue_bound_s - prefetch_bound_s,
            ),
            # The upper bound discounts the host sample only: subtracting a
            # waiting term here would make it a lower bound of the network round
            # trip, and ``low <= est <= high`` would no longer hold.
            net_rtt_high_s=max(0.0, age - sample_duration_s),
        )
    return estimate


def write_csv(path: Path | str, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> None:
    """Write ``rows`` with a header of ``fields``; a missing key raises ``KeyError``.

    Values are read with ``row[field]`` on purpose: a row that lacks a declared
    field must fail loudly instead of writing a silently shifted CSV line.
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(list(fields))
        for row in rows:
            writer.writerow([row[field] for field in fields])


def _write_csv_after_interrupt(
    path: Path | str, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]
) -> str | None:
    """Best-effort ``write_csv`` for an interrupt that landed in a run's tail.

    ``None`` when the rows reached the disk, and also when the retry itself was
    interrupted again: an interrupt is skipped here, never re-raised, which is the
    whole point of this path.  A real ``OSError`` is returned as its text so the
    caller keeps counting it as a write failure (missing evidence outranks the
    abort code).
    """
    try:
        write_csv(path, rows, fields)
    except OSError as exc:
        return str(exc)
    except KeyboardInterrupt:
        return None
    return None


def _write_safety_sidecar(
    path: Path, rows: Sequence[Mapping[str, Any]], *, after_interrupt: bool = False
) -> None:
    """Best-effort write of the safety sidecar: a failure costs a warning, not the run.

    The tracking CSV is the primary artefact and the sidecar is the snapshot taken
    beside it, so an ``OSError`` here (a full disk, a read-only mount) is printed as
    one warning and never becomes ``EXIT_WRITE_FAILED`` -- the run measured what it
    measured, and only the currents snapshot is missing.  The caller's verdict order
    (hold failure > write failure > abort) therefore depends on the main CSV alone.

    ``after_interrupt`` selects the interrupt-tolerant retry
    (``_write_csv_after_interrupt``), which turns a second Ctrl-C into "not written"
    instead of an exception escaping ``main``.  On the normal path a Ctrl-C is left
    to propagate: an operator interrupt is still an interrupt there, and only the
    write retry of the interrupt branch swallows a repeat.
    """
    if after_interrupt:
        error = _write_csv_after_interrupt(path, rows, SAFETY_CSV_FIELDS)
    else:
        try:
            write_csv(path, rows, SAFETY_CSV_FIELDS)
        except OSError as exc:
            error = str(exc)
        else:
            error = None
    if error is not None:
        print(
            f"warning: the safety sidecar {path} could not be written: {error} "
            "(the tracking CSV and the run itself are unaffected)",
            file=sys.stderr,
        )


def _default_csv_path(mode: str) -> Path:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return ARTIFACT_DIR / f"{mode}-{stamp}.csv"


def safety_csv_path(csv_path: Path | str) -> Path:
    """The safety sidecar belonging to a tracking CSV path.

    Same directory, the trailing ``.csv`` replaced by ``-safety.csv``: ``a.b.csv``
    becomes ``a.b-safety.csv`` because only the suffix is dropped (a ``.`` in the
    stem or in a parent directory name survives -- ``Path('/tmp/dir.with.dots/'
    'tracking.csv')`` keeps the directory as it is), and a name that never carried
    ``.csv`` still gets the suffix (``notes`` -> ``notes-safety.csv``).

    Pure name arithmetic, so the sidecar always lands next to the file it belongs
    to and no caller has to re-derive the naming rule: the path is what ties the
    snapshot to its run, and two runs that differ only in their timestamped file
    name must not share a sidecar.
    """
    path = Path(csv_path)
    stem = path.name[: -len(".csv")] if path.name.endswith(".csv") else path.name
    return path.with_name(f"{stem}-safety.csv")


def default_client_factory(host: str, port: int, robot_model: str, request_window: int) -> Any:
    """Build a real ``AlohaMiniClient``.

    The hardware imports stay inside this function so that loading the module
    (from a test or from ``--help``) never pulls in ``zmq``/``cv2``.
    """
    from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient
    from lerobot.robots.alohamini.config_alohamini import AlohaMiniClientConfig

    config = AlohaMiniClientConfig(
        remote_ip=host,
        port_zmq_observations=port,
        robot_model=robot_model,
        observation_request_window=request_window,
    )
    return AlohaMiniClient(config)


def _close_quietly(client: Any) -> None:
    if client is None:
        return
    with contextlib.suppress(Exception):
        client.disconnect()


def missing_client_capabilities(
    client: Any, names: Sequence[str] = CLIENT_CAPABILITIES
) -> list[str]:
    """The client attributes of ``names`` the connected client does not publish.

    Pure and offline, so the surface is testable without a robot.  The result
    keeps the order of ``names`` and is empty when every face exists; the caller
    names the missing one on stderr and refuses the run, because a silently
    absent timing face would turn every rtt cell into an empty or ``NaN`` value
    that reads like a measurement.

    This is the *first* of the two capability probes: it answers "is the face
    there?".  The second one, ``unfilled_client_capabilities``, answers "is the
    face filled?" -- a published but always-``{}`` face is not caught here.
    """
    return [name for name in names if not hasattr(client, name)]


def missing_safety_capabilities(
    client: Any, names: Sequence[str] = SAFETY_CLIENT_CAPABILITIES
) -> list[str]:
    """The client attributes of ``names`` the connected client does not publish.

    The same ``hasattr`` probe ``missing_client_capabilities`` applies to the rtt
    timing face, kept as its own function and fed its own tuple
    (``SAFETY_CLIENT_CAPABILITIES``): a client may publish the safety frame without
    the request timing, so neither answer may stand in for the other.  Pure and
    offline -- it reads the named attribute and nothing else, which is what makes it
    testable without a robot -- and the result keeps the order of ``names``; empty
    means the face is there.

    Unlike the rtt timing face this is NOT a refusal: a missing
    ``latest_safety_status`` costs the sidecar its contents -- ``motor=None`` and
    empty safety cells, one row per frame (``safety_frame_rows``) -- and one stderr
    warning, never the run and never the exit code.  The caller reports the name
    this probe returns instead of reading the attribute through a defaulted
    ``getattr``, so an absent face can never be mistaken for a frame that said
    "nothing to report": the sidecar says which of the two it is by naming the
    motor or leaving the ``motor`` cell empty.
    """
    return [name for name in names if not hasattr(client, name)]


# Same naming rule as ``InsufficientSamples``/``UnitConversionUnavailable``: the
# name is part of the contract, and ``Unfilled`` is the word that keeps this
# refusal apart from a missing capability face.
class UnfilledClientCapability(RuntimeError):  # noqa: N818
    """Raised when a capability face exists but has never been filled in.

    ``names`` keeps the meaning it always had -- the faces the probe named -- so
    ``UnfilledClientCapability(["latest_request_timing"])`` still constructs and
    ``.names`` still lists the face.  ``missing_keys`` says *which* cells of those
    faces hold nothing, which is the part the operator can act on: "the client is
    broken" is not actionable, "``age_s`` is missing" is.  ``missing_reasons``
    (optional) says *why* a key that **is** there cannot be used, keyed by key
    name, as the text to print in parentheses (``"<reason>"``, or
    ``"<reason> (<quoted value>)"`` when the value can be quoted safely); a key
    without an entry is named bare.  The reason is mandatory for an unusable
    value and only the *quote* is optional (``_short_value``), so the operator
    is never told "missing" about a cell the face does carry.
    ``missing_key_labels`` is what the caller prints -- ``"<key> (<reason>)"``
    for an annotated value, the bare key otherwise.
    """

    def __init__(
        self,
        names: Sequence[str],
        missing_keys: Sequence[str] = (),
        missing_reasons: Mapping[str, str] | None = None,
    ) -> None:
        self.names = tuple(names)
        self.missing_keys = tuple(missing_keys)
        self.missing_reasons = dict(missing_reasons or {})
        # One label per key: a bare name stays a bare name (a half-filled face is
        # refused as exactly the keys it dropped), while a present-but-unusable
        # value explains itself in place.  An annotation never holds ``", "`` or a
        # line break -- ``_request_timing_key_problems`` only passes one built
        # from a quotable value -- so the printed list still splits per key.
        self.missing_key_labels = tuple(
            key if self.missing_reasons.get(key) is None else f"{key} ({self.missing_reasons[key]})"
            for key in self.missing_keys
        )
        message = f"{', '.join(self.names)} is published but never filled"
        if self.missing_key_labels:
            message += f" (missing key(s): {', '.join(self.missing_key_labels)})"
        super().__init__(message)


def missing_request_timing_keys(timing: Any) -> tuple[str, ...]:
    """The ``REQUIRED_REQUEST_TIMING_KEYS`` that ``timing`` holds nothing usable for.

    Empty means the face can build a real row; non-empty means it cannot, and the
    result names exactly which cells would come out empty (or ``NaN``).  A key
    counts as filled only when it can give a finite value that physically means a
    measurement -- ``float()``/``str()`` accepting the value is not the bar, and
    each rule below is a shape that converts happily and still writes a
    measurement-looking cell:

    * it is absent from the face, or its value is ``None`` -- what
      ``_request_row`` writes as an empty cell;
    * ``token`` has to be a ``str`` (or the ``bytes`` the client stores, decoded by
      ``_request_token_text``) whose stripped text is not empty: a number becomes
      ``"123"`` and ``"   "`` is non-empty *text* but a blank cell, and neither
      can be paired with a request.  A merely meaningless token (``"abc"``) is a
      legal shape and passes;
    * ``sent_monotonic_s`` / ``received_monotonic_s`` have to be finite numbers,
      and a ``bool`` is not a number: ``float(True) == 1.0`` wrote a stamp at the
      very start of the monotonic epoch;
    * ``age_s`` has to be a finite number ``>= 0``, and a ``bool`` is not a number:
      a request cannot be answered before it was sent, and
      ``net_rtt_estimate_s`` clamps the net estimate up to zero, so a negative age
      was written as a cell that reads like a fast round trip.  ``0.0`` stays
      legal -- a host that measures no age reports exactly that.

    "Not a finite number" also covers a value ``float()`` cannot represent at all:
    an integer too large to be a double (``float(10**400)`` raises
    ``OverflowError``, which ``_finite_or_none`` does not catch).  That shape used
    to leave the probe as an exception and reached the CLI's generic handler as a
    transport failure (exit 3) instead of a capability one (exit 9).

    All those shapes are one defect from the operator's side: the face says
    "there was a timed request" while the fields it holds cannot support that
    claim, so a measurement of zero gets written where no measurement exists.
    The reason for a present-but-unusable value is ``_timing_key_problem``, and
    the *quoted* value next to it is ``_short_value``: the reason is mandatory
    (a bare key would read as "this cell is absent" for a cell the face carries),
    while the quote is optional -- it is dropped when its ``repr`` is longer than
    ``_QUOTED_VALUE_MAX_CHARS``, holds ``", "``, spans lines or raises, because
    such a quote would split the key list or end it mid-word.  Neither failure
    direction is allowed to buy the other: dropping the reason is the same defect
    as dropping the key name, and a broken clause is the same defect as no clause
    (see ``_request_timing_key_problems``).

    Pure and offline: the face is read, never sampled or mutated.  A face that is
    not a mapping reports every necessary key as missing, because nothing can be
    read from it.
    """
    return tuple(key for key, _ in _request_timing_key_problems(timing))


def _capability_face_filled(name: str, value: Any) -> bool:
    """Whether one capability face holds what its consumer reads.

    ``latest_request_timing`` is judged key by key
    (``missing_request_timing_keys``): a *non-empty* face that misses one of the
    four necessary keys, or carries one with a value no row can hold, still
    writes an empty or ``NaN`` cell, so "not empty" is not a high enough bar.
    Every other face name keeps the truthiness test the probe always used.
    """
    if name == "latest_request_timing":
        return not missing_request_timing_keys(value)
    return bool(value)


def unfilled_client_capabilities(
    client: Any, names: Sequence[str] = CLIENT_CAPABILITIES
) -> list[str]:
    """The client attributes of ``names`` that exist but fill nothing usable.

    The *second* capability probe.  ``missing_client_capabilities`` asks whether
    the face exists; this one asks whether the face it found carries what the rtt
    row reads -- a client can publish ``latest_request_timing`` and leave it
    ``{}`` for the whole run (a half-implemented vendor build, a frame the host
    did not time), or carry only part of it (a token but no ``age_s``).
    ``_request_row`` reads that face as ``dict(client.latest_request_timing or {})``,
    so an empty face would be written out as empty ``token``/stamp/``age_s``
    cells (a ``NaN`` lands only in ``net_rtt_est_s``), and a half-filled one as a
    *looks*-timed row with the same empty cells: a CSV that reads like a
    measurement instead of like a missing one.  The caller therefore refuses the
    run with ``EXIT_CLIENT_UNSUPPORTED``, naming the face as empty rather than
    absent.

    "Filled" is therefore key-based for ``latest_request_timing``: the face has
    to carry ``REQUIRED_REQUEST_TIMING_KEYS`` with usable values
    (``missing_request_timing_keys``).  Extra keys are allowed -- the bar is "the
    four are there", never "the face is exactly these four".

    Where the caller probes: on EVERY growth of ``client.observation_sequence``
    and before that frame's row is appended (``collect_rtt_samples``), not once
    at the first growth -- a face that is filled at the start and empties later
    would slip through a one-shot latch and be written out from then on.  The
    same probe runs a second time at the moment the loop runs out of budget
    (see ``collect_rtt_samples``): a frozen sequence never grows, so the
    per-growth probe never runs and an untimed face would otherwise be reported
    as a sampling-budget refusal.  The price of the per-growth strictness is the
    mirror case: a client whose *first* fresh frame is untimed is refused as
    well, even when every later frame is timed.  That is a deliberate strict
    trade-off (refusing a possibly legal client beats writing empty timing
    rows), not an oversight; whether a real vendor client takes that shape is
    unverified.  A refused run writes no CSV at all -- not even a header -- because
    the check sits ahead of the first ``rows.append``, and ``main``'s ``finally``
    still closes the connection.

    Pure and offline; the result keeps the order of ``names`` and is empty when
    every face carries something.  ``getattr`` may default here because the
    first probe already refused an absent face; a defaulted ``None`` counts as
    unfilled.
    """
    return [
        name for name in names if not _capability_face_filled(name, getattr(client, name, None))
    ]


def _unfilled_capability(client: Any) -> UnfilledClientCapability | None:
    """The capability refusal to raise for the timing face, if it holds nothing usable.

    Both probe moments (per growth, and at the give-up deadline of
    ``collect_rtt_samples``) go through this one builder so they cannot drift
    apart in wording or in which keys they report.  The reasons come from the
    same ``_request_timing_key_problems`` pass that names the keys, so the
    annotation can never describe a key the list does not carry.
    """
    unfilled = unfilled_client_capabilities(client)
    if not unfilled:
        return None
    problems = _request_timing_key_problems(getattr(client, "latest_request_timing", None))
    return UnfilledClientCapability(
        unfilled,
        [key for key, _ in problems],
        {key: reason for key, reason in problems if reason is not None},
    )


def _prime_state_only_window(client: Any) -> None:
    """Refill the client's in-flight window with state-only requests.

    ``connect()`` fills the window with ``include_cameras=True``, so the first
    ``observation_request_window`` responses carry camera frames whose JPEG
    encoding work never shows up in ``host_sample_duration_s`` and would pollute
    the first rows.  Guarded with ``getattr`` because a client without the
    method is valid; a failing prime costs a warning, not the run.
    """
    prime = getattr(client, "prime_observation_request_window", None)
    if not callable(prime):
        return
    try:
        prime(include_cameras=False)
    except Exception as exc:  # a failed prime must never abort the run
        print(f"warning: prime_observation_request_window failed: {exc}", file=sys.stderr)
