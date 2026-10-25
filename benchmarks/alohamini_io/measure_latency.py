#!/usr/bin/env python
"""AlohaMini observation-latency and joint step-response benchmark.

Mode A — ``rtt`` (default): ZERO MOTION
    Ask the host for state-only observations (``include_cameras=False``) at
    ``--rate-hz`` and record, per accepted frame, the client-measured request
    age plus a net round-trip estimate.  No motion command is ever produced in
    this mode, so it is safe to run with the robot powered on the bench.
    Sampling paces against an absolute deadline (``start + n / rate_hz``), so a
    blocking read is charged to the period instead of being added on top of it;
    the report prints the rate the run actually achieved next to the requested
    one.  ``connect()`` prefills the in-flight request window with
    ``include_cameras=True``, so the run re-primes that window state-only right
    after connecting (``prime_observation_request_window``, when the client
    offers it): the JPEG encoding of camera frames would otherwise land in the
    first responses, and that work is not part of the subtracted
    ``host_sample_duration_s``.

Mode B — ``tracking``: REAL ROBOT MOTION
    Step one joint by ``--delta-deg`` at a time (capped at ``MAX_STEP_DELTA_DEG``)
    and measure the step response: 90 % rise time, overshoot and steady-state
    error.  This mode moves the robot.  It requires ``--enable-motion`` plus
    ``--joint`` (Gate A) and the Gate B-F checks that run after the first
    observation, and it must only be used after Gate A-F pass and on-site
    authorization for the motion trial has been obtained.  It must not be used
    for an unapproved trial movement.  Every command repeats all measured
    ``*.pos`` goals of both arms plus ``x.vel`` / ``y.vel`` / ``theta.vel``
    pinned to ``0.0``, each step row records both the requested goal and the
    value the client echoed back, and the closing hold is issued from a
    ``finally`` block so no exception skips the stop command.  Commands are
    paced at ``--control-hz`` (see "Command pacing" below).

    Every command also pins ``lift_axis.height_mm`` to the height the latest
    feedback reported, when the feedback carries one (see ``run_tracking_hold``):
    a command stream at 5-50 Hz keeps the host's 1 s command watchdog from ever
    firing, and the host writes no lift register at all when the key is absent,
    so a packet without it leaves the lift running on the goal velocity the
    previous session armed.  The key is never invented: a made-up ``0.0`` would
    command a descent.

    The report labels the closing command ``hold_issued``, never "sent": the
    command was written to the action socket, which is a PUSH/CONFLATE link with
    no acknowledgement, so a write that no peer received is indistinguishable
    from a delivered one and "sent" would read as a receipt.

Command pacing
--------------
``--control-hz`` (default ``50.0``, must be a FINITE number ``>= 1``) is the
COMMAND rate: two consecutive commands of a tracking run are at least
``1 / control_hz`` seconds apart.  Finiteness is part of the contract and not
an accident of the parser: ``inf`` is ``>= 1`` and yields ``1 / inf == 0.0``, so
a rate without an upper bound would switch the pacing off again — the exact
burst this pacer exists to prevent.  ``1e400`` parses to ``inf``, ``nan``
fails every comparison, and the same check is repeated in
``run_tracking_steps`` so a direct caller cannot bypass it.  It is independent of ``--rate-hz``, the OBSERVATION sampling rate —
``--rate-hz`` decides how often a frame is read, ``--control-hz`` how often a
command may leave, and neither overwrites the other.  The pacing covers the
first command of every step and every settle-loop repeat, so an unthrottled
burst of commands is no longer a possible behaviour of the tracking path.

The pause is the *remaining* time to the next slot, so a single pause is at
most one command interval, and it goes through the injected ``sleep``
(``time.sleep`` only as the default, never hardcoded).  The settle budget of a
step and the global ``--timeout-s`` are judged against that same injected
clock, so a pause is charged to them: a step settling at 50 Hz spends part of
its budget waiting for the next command slot, which is what a real run does.  A
settle budget shorter than one command interval therefore ends the step after at
most one repeat instead of sleeping, and it never sleeps more than one command
interval at a time.

The closing hold is issued outside that pacer: it is one write from a
``finally`` block, and delaying the stop command to a command slot would delay
the stop.

Pacing never moves ``rise_90``: the crossing is booked against the arrival stamp
of the frame that crossed (``received_monotonic_s``), i.e. against when a frame
arrived, not against when a command was repeated.

The tracking report states the rates the run actually achieved —
``achieved_command_hz`` (commands written per second of the run) and
``achieved_rate_hz`` (frames read per second of the run) — next to the requested
``control_hz`` and ``rate_hz``; both are ``nan`` when a rate cannot be computed.
A run whose pacing thinned its sampling shows that instead of hiding it behind
the requested values.

The settle tolerance decides whether ``rise_90`` is observable at all
---------------------------------------------------------------------
The 90 % crossing is booked when ``|travel| >= 0.9 * |delta|``, which is the
same condition as "the residual is at most ``0.1 * |delta|``".  The settle loop
then breaks as soon as the residual is at most ``--settle-tolerance-deg``.  When
``settle_tolerance_deg >= 0.1 * |delta|`` the break condition is the LOOSER of
the two, so the loop exits on a frame whose residual can still sit in
``(0.1 * |delta|, settle_tolerance_deg]`` — such a run books no crossing and
``rise_90_ms`` reports ``count=0`` with ``p50=nan``.  The default pair
(``--delta-deg 1.0``, ``--settle-tolerance-deg 0.15``) is exactly that case; a
tolerance below ``0.1 * |delta|`` is what makes the crossing observable.

This relation is REPORTED, not enforced: the tolerance keeps its meaning, the
judgement order and the sampling are untouched, and an empty ``rise_90`` is a
structural consequence of the knobs rather than a broken measurement.  The
tracking banner prints the note whenever the relation holds, and a report whose
``rise_90_ms`` came out empty carries a hint pointing back at it.

Units: ``.pos`` is a normalized unit, never a degree
----------------------------------------------------
``AlohaMini`` normalizes a joint with the mode the host publishes as
``motors.<motor>.normalization``.  ``AlohaMiniConfig.use_degrees`` defaults to
``False`` and the host offers no switch for it, so every arm joint is
``MotorNormMode.RANGE_M100_100``: ``MotorsBus._normalize`` maps a tick ``t`` to
``((t - min) / (max - min)) * 200 - 100``, and one observed unit spans
``(range_max - range_min) / 200`` ticks.  Every ``gripper`` entry of
``_ARM_PROFILES`` is explicitly ``RANGE_0_100`` instead, which maps the same
ticks onto a 100 wide interval: one gripper unit spans twice the ticks and is
therefore worth twice the degrees.  ``degrees`` means the wire value already is
a degree (``deg_per_unit = 1.0``), and a motor that carries no ``normalization``
is read as ``RANGE_M100_100``.  Hence

    deg_per_unit = ((range_max - range_min) / span) * 360 / (resolution - 1)

with ``span`` = 200 (``range_m100_100``, or absent) or 100 (``range_0_100``), and
no formula at all for ``degrees``.  That factor is a DIFFERENT one per joint.
``--delta-deg``, ``--settle-tolerance-deg``, ``MAX_STEP_DELTA_DEG``,
``MAX_TOTAL_TRAVEL_DEG`` and every ``*_deg`` column or report figure are degrees:
a goal put on the wire is ``baseline_units + delta_deg / deg_per_unit``
(``cmd_units``) and a feedback value is converted back with
``units * deg_per_unit`` before it is compared with any limit or tolerance.

The factor comes from the calibration the host publishes in its robot metadata
(``motors.<motor>.range_min`` / ``range_max`` / ``resolution``).  A run that
cannot read it is refused with Gate F / ``EXIT_MOTION_GATE_REFUSED`` and zero
commands: assuming the factor is 1.0 would silently scale every motion limit by
an unknown amount, in an unknown direction.

Gate F is only entered for a metadata payload whose ``schema_version`` is the
integer ``2``, the same rule ``capture_robot_fingerprint.validate_metadata``
applies before it stores a fingerprint: a payload of another layout (or one that
only spells the right number as ``"2"``) may name or span its calibration fields
differently, so it is refused instead of being read with v2 semantics.

Net round-trip estimate
-----------------------
Symbols:

    age_s           request age measured on the PC clock,
                    age_s = received_monotonic_s - sent_monotonic_s
    host_sample_s   host-internal state sampling duration,
                    host_sample_s = state_sample_finished_monotonic_s
                                  - state_sample_started_monotonic_s
    phase_bound_s   = 1 / host_loop_hz with host_loop_hz taken from
                      --host-loop-hz (default HOST_LOOP_HZ_UPPER_BOUND), the
                      longest a request can wait inside one host loop period
                      (20 ms at 50 Hz, 50 ms at 20 Hz)
    phase_mean_s    = phase_bound_s / 2, the expected wait under a uniform
                      phase assumption
    queue_bound_s   = max(0, effective_window - 1) / host_loop_hz, the
                      queueing bound of the other requests kept in flight
    queue_mean_s    = queue_bound_s / 2
    prefetch_period_s = max(1 / rate_hz, 1 / host_loop_hz), one client call
    prefetch_bound_s  = (request_window - 1) * prefetch_period_s
    prefetch_mean_s   = prefetch_bound_s / 2

Why a queue term exists: the client keeps ``observation_request_window``
requests in flight and answers the oldest one per call while replenishing the
window (``_poll_and_get_latest_message`` / ``_fill_observation_request_window``),
and the host answers one request per loop turn.  The answered request is
therefore preceded by ``window - 1`` requests of the same window, which the age
carries as ``queue_s``.  ``request_window = 1`` keeps nothing queued and zeroes
the term.

Why a prefetch term exists: ``_poll_and_get_latest_message`` consumes the
*oldest* token of the window and refills the window only after the answer
arrived, so the frame being read was requested ``window`` client calls ago.
``window - 1`` of those calls are prefetch wait that the age carries without
being any part of a round trip, and one call takes at least
``max(1 / rate_hz, 1 / host_loop_hz)`` (the client cannot beat the host it
waits for).  At 30 Hz with the deployment window of 3 that is 2/30 s = 67 ms,
i.e. more than the whole LAN round trip.  ``--request-window`` therefore
defaults to ``1``, where a plain run measures one explicable round trip; ``3``
is the deployment shape and has to be asked for explicitly.

The queue term is gated by the *request* rate: it is only counted when
``--rate-hz >= --host-loop-hz``.  A client that asks slower than the host serves
drains the request window instead of queueing behind it, so no ``(window - 1)``
periods of queueing exist and subtracting them anyway would structurally pull a
healthy LAN estimate down to zero.  With the gate closed the row is computed
with ``request_window = 1``, i.e. ``queue_bound_s == 0.0``; the report applies
the same gate, so it can never contradict the CSV.  The gate compares the
*requested* rate and not the achieved one, so a row is deterministic.

The age decomposes as

    age_s = net_rtt_s + host_sample_s + phase_s + queue_s + prefetch_s

and the estimate subtracts the known parts:

    net_rtt_est_s  = max(0.0, age_s - host_sample_s - phase_mean_s
                              - queue_mean_s - prefetch_mean_s)
    net_rtt_low_s  = max(0.0, age_s - host_sample_s - phase_bound_s
                              - queue_bound_s - prefetch_bound_s)
    net_rtt_high_s = max(0.0, age_s - host_sample_s)
    net_rtt_low_s <= net_rtt_est_s <= net_rtt_high_s

The point estimate subtracts the mean phase, queue and prefetch waits; the low
bound subtracts their upper bounds; the high bound discounts no waiting term at
all, which is what keeps it an upper bound by construction (a mean never
exceeds its bound).

This is an ESTIMATE of the network round trip, NOT a pure network RTT
measurement: ``age_s`` also contains the host's own state-sampling work and the
waiting phase and queueing inside the host loop, and the host does not say when
exactly the request joined its queue.  That unknown ingress moment is the source
of the reported interval: no single correction of a window larger than one is
more than an average, so the interval is reported next to the point estimate.
When the host timing face is missing, non-finite or negative, nothing is
subtracted, ``host_timing_missing`` is set and ``net_rtt_est_s == age_s`` is
reported as a raw upper bound instead.

Sampling pace vs. network round trip
------------------------------------
``age_s`` also contains the time a response spends *idle in the client socket*
waiting for the next poll.  ``_poll_and_get_latest_message`` sends the next
request only after the previous response arrived, so when the script sleeps
``1 / --rate-hz`` between two ``get_observation`` calls the next response is
already waiting there and ``age_s`` is roughly ``1 / --rate-hz`` plus the real
round trip (a window above one stacks the prefetch wait on top of that).
Measuring the network round trip therefore requires ``--rate-hz`` well above
``1 / expected_rtt``: for an expected RTT in the single-digit milliseconds use
``--rate-hz 1000``, or at least ``200``, and only then is ``age_s`` dominated
by the round trip.  A run at ``--rate-hz 30`` measures the END-TO-END
OBSERVATION LAG at the deployment sampling rate, NOT the network round trip.
At such a low rate the idle time swamps ``net_rtt_est_s``, so it has to be read
together with ``achieved_rate_hz`` from the report and never on its own.  The
script prints an explicit warning when ``--rate-hz`` is below
``RTT_RATE_WARNING_HZ`` (200 Hz).

Clock domains
-------------
The host monotonic stamps (``state_sample_started_monotonic_s`` and friends)
live on the Raspberry Pi clock.  They are never subtracted from the PC clock:
two unrelated monotonic origins make that difference meaningless.  Only
host-internal differences (``finished - started``) are used.

Feedback liveness
-----------------
``AlohaMiniClient`` does not raise when the observation transport times out: it
serves the *cached* frame, so an exception-based probe would keep measuring a
dead link.  The only liveness signal is ``client.observation_sequence``, which
is bumped on a successful receive only.  ``run_tracking_steps`` therefore reads
it after every poll and counts the polls that did not grow it;
``FEEDBACK_STALL_LIMIT`` consecutive polls without growth abort the run (the
counter is never reset by a step boundary), the command stops being re-issued,
and the last cached frame is still merged into the hold state so the closing
command stays full-shape instead of freezing only the joints it can see.

Ctrl-C
------
``main`` never lets ``KeyboardInterrupt`` escape, in either mode: the interrupt
is named on stderr and the exit code is ``EXIT_STEP_RESPONSE_ABORTED``, after the
evidence has been written.

In a tracking run the interrupted step is recorded as a row
(``note=interrupted``), the CSV is written, the report is printed, and the hold
is issued from ``run_tracking_steps``' ``finally`` block.
``run_tracking_steps`` itself keeps propagating the interrupt, and hands the
partial summary to ``main`` through its optional ``record`` out-parameter.

In an rtt run there is no ``finally`` block to hook into: ``collect_rtt_samples``
appends every row to its optional ``rows_out`` list while it samples, so the
frames already taken survive the interrupt, the CSV is written (header only when
not a single sample was accepted) and the quantile summary of the partial set is
printed.  A CSV write that fails on this path still maps to
``EXIT_WRITE_FAILED``: missing evidence outranks the abort code.

Observation lag
---------------
The client keeps ``observation_request_window`` requests in flight
(``--request-window``, default ``1``), so a returned state can be about
``window - 1`` client periods old and that lag is already inside ``age_s`` (it
is the ``prefetch_bound_s`` column).  ``--request-window 1`` is the RTT
measurement shape: exactly one request in flight, no prefetch wait to
subtract.  ``--request-window 3`` is the deployment shape and is what a
control-stack measurement has to use; the model of the prefetch term is an
inference from the client's window bookkeeping and has not been verified on
hardware.

Exit codes
----------
0  run completed (CSV written)
1  the run stopped early with the evidence on disk: in tracking mode a rejected
   command, an exhausted --timeout-s or stale feedback (the step-response report
   and the CSV are still written and the hold was still issued), and in either
   mode an operator Ctrl-C (the rows collected so far are written and the report
   is printed)
3  connect failure (client factory, ``connect()`` or the observation transport)
5  the CSV could not be written
6  Gate A-F refused: ``--mode tracking`` without ``--enable-motion`` or
   ``--joint``, an unreadable joint schema/robot model/baseline, a travel plan
   over the cap, or a joint whose unit conversion cannot be resolved
7  fewer fresh samples than requested before ``--timeout-s``
8  the tracking hold (freeze) command was rejected, so the stop command did not
   reach the host: treat the robot as still moving.  Distinct from 1, where the
   hold was issued and the evidence is on disk.
9  the connected client does not publish the capability rtt mode measures with
   (``latest_request_timing``), so the request age cannot be measured at all:
   the run was refused as soon as ``connect()`` returned, before the state-only
   window prime sends its first request and before the first sampled frame, and
   no CSV was written -- not even a header.  ``--mode tracking`` does not read
   that face and is unaffected.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import math
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

# numpy only for the percentile definition shared with
# ``benchmarks/alohamini_ik/benchmark_arm_ik.py``.
import numpy as np

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
# The client cannot measure what this mode measures: it publishes no
# ``latest_request_timing``, so there is no request age to record and the rtt run
# is refused instead of writing a CSV of empty timing cells.
EXIT_CLIENT_UNSUPPORTED = 9

# One commanded step never exceeds this many degrees.
MAX_STEP_DELTA_DEG = 1.0
# The host control loop is bounded above by this rate, which bounds the phase
# a request can spend waiting inside one host period.
HOST_LOOP_HZ_UPPER_BOUND = 50.0
# Product of the per-step and per-run caps: a tracking run may travel this far.
MAX_TOTAL_TRAVEL_DEG = 20.0
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
            f"unit conversion unavailable: host metadata is not a mapping (got {type(metadata).__name__})"
        )
    motors = metadata.get("motors")
    if not isinstance(motors, Mapping) or not motors:
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: host metadata carries no motor calibration for {motor!r}"
        )
    entry = motors.get(motor)
    if not isinstance(entry, Mapping):
        raise UnitConversionUnavailable(
            f"unit conversion unavailable: host metadata has no calibration entry for motor {motor!r}"
        )
    normalization = entry.get("normalization")
    # ``degrees`` is already a degree: there is no span, no resolution and no
    # interval to divide, so even a degenerate calibration is usable as is.
    if normalization == NORMALIZATION_DEGREES:
        return 1.0
    if normalization is None or normalization == NORMALIZATION_RANGE_M100_100:
        # Absent is the compatibility case: old metadata only carries the
        # calibration triple, and every arm joint genuinely is RANGE_M100_100.
        span = SPAN_M100_100
    elif normalization == NORMALIZATION_RANGE_0_100:
        span = SPAN_0_100
    else:
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


def _queue_terms_s(
    request_window: int, host_loop_hz: float = HOST_LOOP_HZ_UPPER_BOUND
) -> tuple[float, float]:
    """Bound and mean of the host queueing time for ``request_window`` requests in flight."""
    loop_hz = host_loop_hz if host_loop_hz > 0.0 else HOST_LOOP_HZ_UPPER_BOUND
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
    loop_hz = host_loop_hz if host_loop_hz > 0.0 else HOST_LOOP_HZ_UPPER_BOUND
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
    loop_hz = host_loop_hz if host_loop_hz > 0.0 else HOST_LOOP_HZ_UPPER_BOUND
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


def _default_csv_path(mode: str) -> Path:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return ARTIFACT_DIR / f"{mode}-{stamp}.csv"


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
    """
    return [name for name in names if not hasattr(client, name)]


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


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", required=True, help="AlohaMini host IP address (required)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Host observation port")
    parser.add_argument(
        "--robot-model",
        default=DEFAULT_ROBOT_MODEL,
        help="Robot model reported by the host (must match the host side)",
    )
    parser.add_argument(
        "--mode",
        choices=("rtt", "tracking"),
        default=DEFAULT_MODE,
        help="rtt: request-age only, zero motion; tracking: real robot motion",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="Observation sampling rate (independent of --control-hz, the command rate)",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=DEFAULT_CONTROL_HZ,
        help=(
            "Command rate: two consecutive goal commands are at least 1/control_hz seconds "
            "apart. Independent of --rate-hz, which is the OBSERVATION sampling rate: --rate-hz "
            "decides how often a frame is read, --control-hz how often a command may leave, and "
            "neither overwrites the other (sampling faster than commanding is the intended "
            "shape). The pacing covers the first command of every step and every settle-loop "
            "repeat and goes through the injected sleeper, one remaining-time pause at a time, so "
            "a single pause is never longer than 1/control_hz. An unthrottled burst of commands "
            "is therefore no longer a possible behaviour of the tracking run; the closing hold is "
            "issued outside the pacer. Must be >= 1"
        ),
    )
    parser.add_argument(
        "--samples", type=int, default=DEFAULT_SAMPLES, help="Fresh frames to record (rtt)"
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="Wall-clock budget for the whole sampling or tracking run",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Destination CSV path (default: artifacts/alohamini-io/<mode>-<utc>.csv)",
    )
    parser.add_argument(
        "--request-window",
        type=int,
        default=DEFAULT_REQUEST_WINDOW,
        help=(
            "Observation requests the client keeps in flight; forwarded to "
            "AlohaMiniClientConfig.observation_request_window. 1 (the default) is the RTT "
            "measurement shape: one request in flight, no prefetch wait to subtract. 3 is the "
            "deployment shape of the control stack and has to be asked for: the answered "
            "request was then sent window-1 client periods earlier, which the row reports as "
            "prefetch_bound_s (its window-1 queued requests are additionally subtracted only "
            "while --rate-hz keeps up with --host-loop-hz)"
        ),
    )
    parser.add_argument(
        "--host-loop-hz",
        type=float,
        default=DEFAULT_HOST_LOOP_HZ,
        help=(
            "Host control-loop rate: sets the phase bound (1/host_loop_hz), the queue period "
            "and the floor of one client period in the prefetch term of the net round-trip "
            "estimate. Feed a measured host rate back in here. The queue term is only counted "
            "when --rate-hz >= --host-loop-hz"
        ),
    )
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help=(
            "Gate A switch: required by --mode tracking. Enables real robot motion; "
            "only pass it after Gate A-F pass and on-site authorization."
        ),
    )
    parser.add_argument(
        "--joint",
        default=None,
        help=(
            "Gate A: joint to step in --mode tracking, as an observation key "
            "(e.g. arm_left_shoulder_pan.pos; the .pos suffix is optional)"
        ),
    )
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=DEFAULT_DELTA_DEG,
        help=(
            f"Requested step size in DEGREES (clamped to {MAX_STEP_DELTA_DEG} deg). A .pos "
            "value on the wire is a normalized unit, not a degree: the goal is sent as "
            "baseline_units + delta_deg / deg_per_unit, with the deg-per-unit factor read from "
            "the host metadata (motors.<motor>.range_min/range_max/resolution and its "
            "normalization: the span is 200 for range_m100_100 or absent, 100 for range_0_100, "
            "and 1.0 for degrees). A joint whose "
            "factor cannot be read is refused (Gate F) before anything is commanded"
        ),
    )
    parser.add_argument("--steps", type=int, default=DEFAULT_STEPS, help="Number of tracking steps")
    parser.add_argument(
        "--settle-timeout-s",
        type=float,
        default=DEFAULT_SETTLE_TIMEOUT_S,
        help="Per-step settling budget (a slow joint is reported, not aborted)",
    )
    parser.add_argument(
        "--settle-tolerance-deg",
        type=float,
        default=DEFAULT_SETTLE_TOLERANCE_DEG,
        help="A step counts as settled when the feedback is this many DEGREES from the target",
    )
    args = parser.parse_args(argv)
    # A nonsense run shape is a usage error (2), not a connect failure (3): the
    # ranges are checked here so no socket is ever opened for them.
    if args.samples < 1:
        parser.error(f"--samples must be >= 1 (got {args.samples})")
    if args.rate_hz <= 0.0:
        parser.error(f"--rate-hz must be > 0 (got {args.rate_hz})")
    # Two judgements on purpose, and the first one is load-bearing: ``inf`` is
    # ``>= 1.0`` and ``1 / inf == 0.0``, so a finiteness-blind check let
    # ``--control-hz inf`` (and ``1e400``, which argparse parses to ``inf``)
    # through and the pacing was skipped — the unthrottled burst reopened.
    # ``nan`` makes every comparison false, hence the ``not (a and b)`` shape:
    # ``x < 1.0`` alone would let it reach the division.
    if not (math.isfinite(args.control_hz) and args.control_hz >= 1.0):
        parser.error(f"--control-hz must be a finite number >= 1 (got {args.control_hz})")
    if args.request_window < 1:
        parser.error(f"--request-window must be >= 1 (got {args.request_window})")
    if args.host_loop_hz <= 0.0:
        parser.error(f"--host-loop-hz must be > 0 (got {args.host_loop_hz})")
    # Tracking knobs: a zero step count commands nothing while looking like a
    # run, and a non-positive settle budget or a negative tolerance makes the
    # settle judgement meaningless.
    if args.steps <= 0:
        parser.error(f"--steps must be >= 1 (got {args.steps})")
    if args.settle_timeout_s <= 0.0:
        parser.error(f"--settle-timeout-s must be > 0 (got {args.settle_timeout_s})")
    if args.settle_tolerance_deg < 0.0:
        parser.error(f"--settle-tolerance-deg must be >= 0 (got {args.settle_tolerance_deg})")
    # A non-positive step is not a step.  Checked before the tolerance cross-check
    # below, which would otherwise blame the clamp for a value the operator typed:
    # ``--delta-deg 0`` used to surface as "settle-tolerance-deg must be below the
    # effective --delta-deg ... delta 0.0 deg after the 1.0 deg clamp".
    if args.delta_deg <= 0.0:
        parser.error(f"--delta-deg must be > 0 (got {args.delta_deg})")
    # A tolerance at or above the step the run actually takes judges the joint
    # settled on the first poll, which leaves ``rise_90`` empty in every row.  The
    # reference is the CLAMPED step (``resolve_delta_deg``): ``--delta-deg 5.0``
    # commands 1.0 deg, so a 1.0 deg tolerance is already the empty-column case.
    # Checked in every mode on purpose: rtt parses the same knobs and ignores them,
    # and the default pair (1.0 / 0.15) stays legal.
    effective_delta_deg = resolve_delta_deg(args)
    if args.settle_tolerance_deg >= effective_delta_deg:
        parser.error(
            f"--settle-tolerance-deg must be below the effective --delta-deg "
            f"(got settle-tolerance {args.settle_tolerance_deg} deg >= delta "
            f"{effective_delta_deg} deg after the {MAX_STEP_DELTA_DEG} deg clamp)"
        )
    if args.csv is None:
        args.csv = _default_csv_path(args.mode)
    return args


# ---------------------------------------------------------------------------
# Mode A: request-age sampling (zero motion).
# ---------------------------------------------------------------------------


def _host_clock_ns(host_timing: Any) -> int | str:
    """Host-side unix-nanosecond baseline of the sample, or ``""`` when absent.

    The host converts its own monotonic stamp into unix nanoseconds, so this
    value is the only bridge between the two clock domains.  An absent or
    non-finite stamp becomes an empty CSV cell: writing ``0`` instead would look
    like a valid 1970 stamp.
    """
    if not isinstance(host_timing, Mapping):
        return ""
    value = host_timing.get("state_sample_unix_ns")
    if isinstance(value, int):
        return value
    number = _finite_or_none(value)
    return "" if number is None else int(number)


def _request_row(
    client: Any,
    *,
    index: int,
    sequence: int,
    request_window: int,
    host_loop_hz: float,
    rate_hz: float,
) -> dict[str, Any]:
    timing = dict(client.latest_request_timing or {})
    host_timing = client.latest_host_timing
    token = timing.get("token")
    if isinstance(token, bytes):
        # The client stores the request token as bytes; decode before it reaches
        # a dict/CSV so the cell is text and not ``b'...'``.
        token = token.decode("ascii", errors="replace")
    elif token is None:
        token = ""
    else:
        token = str(token)
    # The RAW window and the requested rate go in: ``net_rtt_estimate_s`` owns
    # both gates.  Passing an already gated window here would also gate the
    # prefetch term, i.e. it would hide exactly the wait that dominates when the
    # client is slower than the host.
    estimate = net_rtt_estimate_s(
        timing.get("age_s"),
        host_timing,
        host_loop_hz=host_loop_hz,
        request_window=int(request_window),
        rate_hz=float(rate_hz),
    )
    return {
        "sample_index": index,
        "token": token,
        "observation_sequence": sequence,
        "sent_monotonic_s": _finite_or_none(timing.get("sent_monotonic_s")),
        "received_monotonic_s": _finite_or_none(timing.get("received_monotonic_s")),
        "age_s": estimate["age_s"],
        "host_sample_duration_s": estimate["host_sample_duration_s"],
        "phase_bound_s": estimate["phase_bound_s"],
        "phase_mean_s": estimate["phase_mean_s"],
        "queue_bound_s": estimate["queue_bound_s"],
        "queue_mean_s": estimate["queue_mean_s"],
        "prefetch_bound_s": estimate["prefetch_bound_s"],
        "prefetch_mean_s": estimate["prefetch_mean_s"],
        "host_timing_missing": estimate["host_timing_missing"],
        "net_rtt_est_s": estimate["net_rtt_est_s"],
        "net_rtt_low_s": estimate["net_rtt_low_s"],
        "net_rtt_high_s": estimate["net_rtt_high_s"],
        "state_sample_unix_ns": _host_clock_ns(host_timing),
    }


def collect_rtt_samples(
    client: Any,
    samples: int,
    rate_hz: float,
    timeout_s: float,
    sleep: Callable[[float], None] | None = None,
    clock: Callable[[], float] | None = None,
    request_window: int = DEFAULT_REQUEST_WINDOW,
    host_loop_hz: float = DEFAULT_HOST_LOOP_HZ,
    rows_out: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    """Collect one row per *growth* of ``client.observation_sequence``.

    Repeated frames are skipped instead of counted twice, and a sequence that
    never grows ends in ``InsufficientSamples`` rather than a short CSV.

    ``rows_out``, when given, is the very list the rows are appended to, so a
    caller that catches an exception raised out of this loop (``KeyboardInterrupt``
    in particular) still holds every row collected up to that point instead of
    losing them to the local scope.

    The loop paces against an absolute deadline: sample ``n`` is due at
    ``start + n / rate_hz``, so a blocking read is charged to the period instead
    of being added on top of it (sleeping one whole period after every read
    would cap the achieved rate at ``1 / (read + period)``).

    ``request_window`` and ``rate_hz`` reach the row unconverted: the queue and
    prefetch terms of ``net_rtt_estimate_s`` own their own gates, so a row can
    never report a term the estimator did not subtract (or the other way round);
    ``host_loop_hz`` also sets the phase bound.
    """
    sleep_fn = time.sleep if sleep is None else sleep
    clock_fn = time.monotonic if clock is None else clock
    period_s = 1.0 / rate_hz if rate_hz > 0 else 0.0
    start_s = clock_fn()
    deadline_s = start_s + timeout_s
    last_sequence = client.observation_sequence
    rows: list[dict[str, Any]] = [] if rows_out is None else rows_out

    while len(rows) < samples:
        now_s = clock_fn()
        if now_s >= deadline_s:
            raise InsufficientSamples(required=samples, collected=len(rows))
        client.get_observation(include_cameras=False)
        sequence = client.observation_sequence
        if sequence > last_sequence:
            last_sequence = sequence
            rows.append(
                _request_row(
                    client,
                    index=len(rows),
                    sequence=sequence,
                    request_window=int(request_window),
                    host_loop_hz=float(host_loop_hz),
                    rate_hz=float(rate_hz),
                )
            )
            if len(rows) >= samples:
                break
        # Only top up to the next absolute deadline, never past the run budget.
        target_s = start_s + len(rows) * period_s
        pause_s = min(target_s - now_s, deadline_s - now_s)
        if pause_s > 0.0:
            sleep_fn(pause_s)

    return rows


def resolve_delta_deg(args: argparse.Namespace) -> float:
    """Per-step delta actually used: the request clamped to ``MAX_STEP_DELTA_DEG``."""
    return min(float(args.delta_deg), MAX_STEP_DELTA_DEG)


def _achieved_rate_hz(sample_count: int, elapsed_s: float) -> float:
    """Samples per second the run actually achieved (``nan`` when unknown)."""
    if sample_count <= 0 or not math.isfinite(elapsed_s) or elapsed_s <= 0.0:
        return float("nan")
    return sample_count / elapsed_s


def _rtt_rate_warning(args: argparse.Namespace) -> str | None:
    """Report line for a sampling rate too low for ``age_s`` to be a round trip.

    ``None`` when ``--rate-hz`` is at or above ``RTT_RATE_WARNING_HZ``, so the
    report of a network-grade run is unchanged.
    """
    rate_hz = float(args.rate_hz)
    if rate_hz >= RTT_RATE_WARNING_HZ:
        return None
    return (
        f"  WARNING: sampling rate_hz={rate_hz:.1f} < {RTT_RATE_WARNING_HZ:.0f}: at this sampling "
        "rate age_s is mostly the idle time one sampling period long that a response waits in the "
        "client socket for the next poll, so net_rtt_estimate_ms is NOT a network round trip.  "
        "Raise --rate-hz (e.g. 1000) when the target is the network round trip."
    )


def _format_rtt_report(
    rows: Sequence[Mapping[str, Any]], args: argparse.Namespace, elapsed_s: float
) -> str:
    ages_ms = [row["age_s"] * 1000.0 for row in rows if _finite_or_none(row["age_s"]) is not None]
    net_ms = [
        row["net_rtt_est_s"] * 1000.0
        for row in rows
        if _finite_or_none(row["net_rtt_est_s"]) is not None
    ]
    low_ms = [
        row["net_rtt_low_s"] * 1000.0
        for row in rows
        if _finite_or_none(row["net_rtt_low_s"]) is not None
    ]
    high_ms = [
        row["net_rtt_high_s"] * 1000.0
        for row in rows
        if _finite_or_none(row["net_rtt_high_s"]) is not None
    ]
    missing = sum(1 for row in rows if row["host_timing_missing"])
    age_stats = summary_stats(ages_ms)
    net_stats = summary_stats(net_ms)
    low_stats = summary_stats(low_ms)
    high_stats = summary_stats(high_ms)
    achieved = _achieved_rate_hz(len(rows), elapsed_s)
    # Same gates as the rows: the report may not print a queue or prefetch term
    # the CSV did not subtract (a drained window queues nothing when the request
    # rate is slower; the prefetch wait is always counted).
    queued_window = effective_request_window(
        int(args.request_window), float(args.rate_hz), float(args.host_loop_hz)
    )
    queue_bound_s, queue_mean_s = _queue_terms_s(queued_window, float(args.host_loop_hz))
    prefetch_bound_s, prefetch_mean_s = _prefetch_terms_s(
        int(args.request_window), float(args.rate_hz), float(args.host_loop_hz)
    )
    # Printed next to achieved_rate_hz on purpose: at a low sampling rate the
    # idle time of one sampling period dominates age_s (see module docstring),
    # so the estimate has to be read with the rate.  No existing field and no
    # exit code changes with it.
    rate_warning = _rtt_rate_warning(args)
    return "\n".join(
        [
            "AlohaMini observation latency report (mode: rtt, zero motion)",
            f"  host={args.host} port={args.port} robot_model={args.robot_model} samples={len(rows)}",
            f"  achieved_rate_hz={achieved:.1f} (requested {float(args.rate_hz):.1f} Hz) "
            f"over {elapsed_s:.3f} s",
            *([] if rate_warning is None else [rate_warning]),
            f"  request_age_ms      count={age_stats['count']} min={age_stats['min_ms']:.3f} "
            f"p50={age_stats['p50_ms']:.3f} p95={age_stats['p95_ms']:.3f} "
            f"p99={age_stats['p99_ms']:.3f} max={age_stats['max_ms']:.3f}",
            f"  net_rtt_estimate_ms count={net_stats['count']} min={net_stats['min_ms']:.3f} "
            f"p50={net_stats['p50_ms']:.3f} p95={net_stats['p95_ms']:.3f} "
            f"p99={net_stats['p99_ms']:.3f} max={net_stats['max_ms']:.3f}",
            f"  net_rtt_bounds_ms   low p50={low_stats['p50_ms']:.3f} p95={low_stats['p95_ms']:.3f} "
            f"| high p50={high_stats['p50_ms']:.3f} p95={high_stats['p95_ms']:.3f}",
            f"  request_window={args.request_window} queued_window={queued_window} "
            f"host_loop_hz={float(args.host_loop_hz):.1f} "
            f"queue_bound_ms={queue_bound_s * 1000.0:.3f} "
            f"queue_mean_ms={queue_mean_s * 1000.0:.3f} "
            f"prefetch_bound_ms={prefetch_bound_s * 1000.0:.3f} "
            f"prefetch_mean_ms={prefetch_mean_s * 1000.0:.3f} "
            f"host_timing_missing={missing}/{len(rows)}",
            "  NOTE: net_rtt_estimate_ms is an ESTIMATE, not a pure network round trip: age_s also "
            "contains the host state-sampling duration, the waiting phase of the request inside the "
            "host loop, the queueing of the other in-flight requests and the prefetch wait of the "
            "window (the client answers the oldest request and refills the window only afterwards).  "
            "The moment the request entered the host queue is unknown, so the point estimate "
            "subtracts the mean phase, queue and prefetch waits while net_rtt_bounds_ms spans the "
            "bound-to-bound interval around it.",
            f"  csv={args.csv}",
        ]
    )


def _run_rtt_mode(client: Any, args: argparse.Namespace) -> int:
    started_s = time.monotonic()
    # The rows live here, not in ``collect_rtt_samples``' scope: an operator Ctrl-C
    # unwinds that frame, and the samples already taken are the only evidence the
    # run produced.
    rows: list[dict[str, Any]] = []
    interrupted = False
    try:
        collect_rtt_samples(
            client,
            args.samples,
            args.rate_hz,
            args.timeout_s,
            request_window=int(args.request_window),
            host_loop_hz=float(args.host_loop_hz),
            rows_out=rows,
        )
    except InsufficientSamples as exc:
        print(f"insufficient samples: {exc}", file=sys.stderr)
        return EXIT_INSUFFICIENT_SAMPLES
    except KeyboardInterrupt:
        interrupted = True
        print(
            f"probe aborted: {INTERRUPT_ABORT_REASON}; writing the {len(rows)} sample(s) collected so far.",
            file=sys.stderr,
        )
    except Exception as exc:  # transport died mid-sampling
        print(f"observation failed: {exc}", file=sys.stderr)
        return EXIT_CONNECT_FAILED
    elapsed_s = time.monotonic() - started_s

    # The tail runs after the sampling loop is gone, so a second Ctrl-C can only
    # land in the write or the report.  It may not escape ``main`` either: the rows
    # are retried once, the interrupt is named on stderr, and a write that still
    # fails keeps the documented precedence (missing evidence outranks the abort).
    try:
        write_csv(args.csv, rows, RTT_CSV_FIELDS)
        print(_format_rtt_report(rows, args, elapsed_s))
    except OSError as exc:
        print(f"write failed: {exc}", file=sys.stderr)
        return EXIT_WRITE_FAILED
    except KeyboardInterrupt:
        print(
            f"probe aborted: {INTERRUPT_ABORT_REASON}; interrupted while writing the "
            f"{len(rows)} sample(s) or printing the report.",
            file=sys.stderr,
        )
        write_error = _write_csv_after_interrupt(args.csv, rows, RTT_CSV_FIELDS)
        if write_error is not None:
            print(f"write failed: {write_error}", file=sys.stderr)
            return EXIT_WRITE_FAILED
        return EXIT_STEP_RESPONSE_ABORTED

    if interrupted:
        return EXIT_STEP_RESPONSE_ABORTED
    return EXIT_OK


# ---------------------------------------------------------------------------
# Mode B: joint step response (real robot motion, gated).
# ---------------------------------------------------------------------------


def _motion_gate_message(args: argparse.Namespace) -> str | None:
    """Gate A: decide before any client is built, so a refusal never connects."""
    if not args.enable_motion:
        return (
            "Gate A refused: --mode tracking moves the real robot but --enable-motion was not "
            "given.\n"
            "  enable_motion is the first precondition of Gate A-F; nothing was connected and no "
            "motion command was issued.\n"
            "  Re-run with --enable-motion --joint <joint.pos> only after Gate A-F pass and "
            "on-site authorization."
        )
    if not args.joint:
        return (
            "Gate A refused: --mode tracking requires an explicit --joint <joint.pos>.\n"
            "  enable_motion was given, but no joint was named, so no target can be built; "
            "nothing was connected and no motion command was issued."
        )
    return None


def _joint_state_key(joint: str) -> str:
    name = str(joint).strip()
    return name if name.endswith(".pos") else f"{name}.pos"


def _tracking_preflight(
    client: Any,
    args: argparse.Namespace,
    observation: Mapping[str, Any],
    joint_key: str,
    delta_deg: float,
) -> tuple[float | None, str | None]:
    """Gates B-F; every one of them runs before the first motion command.

    Returns the deg-per-unit factor of the stepped joint together with the
    refusal message (``(scale, None)`` when everything passed, ``(None, text)``
    when a gate refused).  Gate F runs after Gate C (the robot model check): the
    calibration table it reads is part of the same host metadata, and the model
    check is what makes the joint names in it trustworthy.  Gate F also requires
    ``schema_version == 2`` (the rule ``capture_robot_fingerprint`` applies): the
    calibration fields of another layout need not mean what this gate assumes.
    """
    if not args.enable_motion:
        return None, "Gate A refused: --enable-motion is required for --mode tracking."

    if joint_key not in observation:
        available = ", ".join(sorted(str(key) for key in observation)) or "<empty observation>"
        return None, (
            f"Gate B refused: {joint_key!r} is not part of the connected state schema.\n"
            f"  available: {available}\n"
            "  Check --joint and --robot-model against the host side; nothing was commanded."
        )

    metadata = client.latest_robot_metadata
    reported_model = metadata.get("robot_model") if isinstance(metadata, Mapping) else None
    if not reported_model:
        # Fail closed: a host that cannot name its model leaves the joint names
        # and units unverified, which is exactly the case this gate exists for.
        return None, (
            f"Gate C refused: the host metadata carries no usable robot_model (got "
            f"{reported_model!r}).\n"
            f"  --robot-model={args.robot_model!r} cannot be confirmed against the host, so the "
            "joint names and units are unverified; nothing was commanded."
        )
    if reported_model != args.robot_model:
        return None, (
            f"Gate C refused: the host reports robot_model={reported_model!r} but "
            f"--robot-model={args.robot_model!r} was requested.\n"
            "  Joint names and units would not match; nothing was commanded."
        )

    if _finite_or_none(observation.get(joint_key)) is None:
        return None, (
            f"Gate D refused: baseline feedback for {joint_key!r} is missing or non-finite; "
            "nothing was commanded."
        )

    if delta_deg <= 0.0 or int(args.steps) < 1:
        return (
            None,
            "Gate E refused: --delta-deg and --steps must both be positive; nothing was commanded.",
        )
    total_travel_deg = delta_deg * int(args.steps)
    if total_travel_deg > MAX_TOTAL_TRAVEL_DEG:
        return None, (
            f"Gate E refused: planned travel {total_travel_deg:.2f} deg exceeds the "
            f"{MAX_TOTAL_TRAVEL_DEG:.1f} deg cap ({delta_deg:.2f} deg x {args.steps} steps); "
            "nothing was commanded."
        )

    # Strict on purpose: ``True``/``False`` compare equal to 1/0 and the string
    # ``"2"`` compares equal to nothing, but all three describe a payload this gate
    # cannot read, so the bare ``!= 2`` is combined with an explicit bool rejection
    # instead of a truthiness test.
    schema_version = metadata.get("schema_version") if isinstance(metadata, Mapping) else None
    if isinstance(schema_version, bool) or schema_version != HOST_METADATA_SCHEMA_VERSION:
        return None, (
            f"Gate F refused: the host metadata schema_version is {schema_version!r}, expected "
            f"{HOST_METADATA_SCHEMA_VERSION}.\n"
            "  The calibration table under motors.<motor> would come from another metadata layout, "
            "so its fields (range_min / range_max / resolution / normalization) and every degree "
            "figure derived from them are unverified; nothing was commanded."
        )

    try:
        scale = deg_per_unit(metadata, joint_key)
    except UnitConversionUnavailable as exc:
        # Gate F: without the factor neither --delta-deg nor any of the degree
        # caps can be turned into wire units, and a guessed factor would rescale
        # every one of them by an unknown amount.
        return None, (
            f"Gate F refused: {exc}.\n"
            f"  --joint {joint_key!r} needs the deg-per-unit factor of the host metadata "
            "(motors.<motor>.range_min / range_max / resolution) to convert --delta-deg, the "
            "travel cap and the settle tolerance into normalized wire units; nothing was commanded."
        )
    return scale, None


def _tracking_command(
    state: Mapping[str, Any], overrides: Mapping[str, float] | None = None
) -> dict[str, float]:
    """A full command: measured ``*.pos`` goals, the measured lift height, zero base velocity.

    The client builds the whole packet from the dict it is handed and sends
    ``0.0`` for a key it cannot find, so a partly-built dict would drive the
    other joints and the base towards zero.  Every command therefore repeats the
    measured position of every joint and pins the base to a stop.

    ``lift_axis.height_mm`` is pinned to the height the latest feedback
    measured, and only when the feedback carries one.  A command stream at
    5-50 Hz keeps the host's 1 s command watchdog from ever firing, and the
    host's ``LiftAxis.apply_action`` writes no lift register at all when the key
    is absent — the lift then keeps moving on whatever ``Goal_Velocity`` the
    previous session left behind.  Writing the measured height back is what
    pins the axis.  An absent key stays absent: an invented ``0.0`` would
    command a descent.
    """
    command: dict[str, float] = {}
    for key, value in (state or {}).items():
        name = str(key)
        number = _finite_or_none(value)
        if number is None:
            continue
        if name.endswith(POS_SUFFIX) or name == LIFT_HEIGHT_KEY:
            command[name] = number
    for key, value in (overrides or {}).items():
        command[str(key)] = float(value)
    for key in BASE_VELOCITY_KEYS:
        command[key] = 0.0
    return command


def _echoed_deg(echo: Any, joint_key: str, scale: float) -> float | None:
    """Value the client echoed back for ``joint_key``, in degrees.

    ``None`` when the client said nothing.  The echo arrives in normalized units
    like every other ``.pos`` value, so it is converted with the same factor: a
    ``*_deg`` column that mixes units is worse than no column.
    """
    if not isinstance(echo, Mapping):
        return None
    units = _finite_or_none(echo.get(joint_key))
    return None if units is None else deg_from_units(units, scale)


def _arrival_monotonic_s(client: Any) -> float | None:
    """PC-clock arrival stamp of the last frame, or ``None`` when it has none.

    ``client.latest_request_timing["received_monotonic_s"]`` is stamped when the
    response actually arrived; the poll clock is read *before* the blocking read,
    so using it as the moment of a 90 % crossing inflates the rise time by one
    observation.
    """
    timing = getattr(client, "latest_request_timing", None)
    if not isinstance(timing, Mapping):
        return None
    return _finite_or_none(timing.get("received_monotonic_s"))


def _tracking_summary(
    rows: Sequence[Mapping[str, Any]],
    abort_reason: str | None,
    hold_action: Mapping[str, float],
    hold_error: str | None,
    *,
    commands_issued: int = 0,
    observations: int = 0,
    elapsed_s: float = float("nan"),
) -> dict[str, Any]:
    """Assemble the tracking report from the rows recorded so far.

    ``commands_issued``/``observations`` are the counts the run really produced
    (the closing hold included) and ``elapsed_s`` is the time it took, so
    ``achieved_command_hz`` and ``achieved_rate_hz`` are measured evidence rather
    than the requested knobs echoed back: a run whose pacing thinned its sampling
    reports the thinned rate.  Both are ``nan`` when a rate cannot be computed.
    """
    rise_times_s = [row["rise_90_time_s"] for row in rows if row["rise_90_time_s"] is not None]
    settle_errors_deg = [abs(row["error_deg"]) for row in rows if row["settled"]]
    overshoots_deg = [row["overshoot_deg"] for row in rows]
    return {
        "rows": list(rows),
        "steps_completed": len(rows),
        "abort_reason": abort_reason,
        "run_elapsed_s": elapsed_s,
        "commands_issued": commands_issued,
        "observations": observations,
        "achieved_command_hz": _achieved_rate_hz(commands_issued, elapsed_s),
        "achieved_rate_hz": _achieved_rate_hz(observations, elapsed_s),
        "rise_90_ms": _quantiles([value * 1000.0 for value in rise_times_s]),
        "steady_state_error_deg": _quantiles(settle_errors_deg),
        "max_overshoot_deg": max(overshoots_deg) if overshoots_deg else float("nan"),
        "hold_action": dict(hold_action),
        "hold_error": hold_error,
    }


def run_tracking_steps(
    client: Any,
    *,
    joint_key: str,
    delta_deg: float,
    steps: int,
    rate_hz: float,
    settle_timeout_s: float,
    settle_tolerance_deg: float,
    timeout_s: float,
    control_hz: float = DEFAULT_CONTROL_HZ,
    sleep: Callable[[float], None] | None = None,
    clock: Callable[[], float] | None = None,
    record: dict[str, Any] | None = None,
    deg_per_unit: float = 1.0,
) -> dict[str, Any]:
    """Step one joint and measure the response (real robot motion, Gate A-F passed).

    For every step the current feedback is read, the goal is set ``delta_deg``
    away from it, and the same goal is re-issued on every feedback poll so the
    host watchdog never fires while the joint settles.  The loop stops on a
    rejected command, missing/non-finite feedback or a dead feedback link.

    Units: a ``.pos`` value is a normalized unit
    (``MotorNormMode.RANGE_M100_100``) and ``deg_per_unit`` is what one of those
    units is worth in degrees on this joint (see ``deg_per_unit``).  Every
    judgement — ``delta_deg``, ``settle_tolerance_deg``, the 90 % crossing and
    the overshoot — happens in degrees, and only the value put on the wire is
    converted: the goal is ``baseline_units + delta_deg / deg_per_unit``
    (``cmd_units``) while every ``*_deg`` column and the report stay degrees.
    The default ``1.0`` keeps a caller that already works in degrees exact.

    Feedback liveness is checked on every poll: the client answers a timed out
    observation with its cached frame instead of raising, so
    ``client.observation_sequence`` is the only proof that frames still arrive.
    ``FEEDBACK_STALL_LIMIT`` consecutive polls without growth abort the run with
    ``STALE_FEEDBACK_REASON``; the count is consecutive over the whole run and is
    never reset at a step boundary.  The cached frame is still merged into the
    hold state, so the closing command stays full-shape.

    The closing hold (see ``run_tracking_hold``) is issued from a ``finally``
    block, so a rejected command, a failing feedback read, a stalled link or a
    ``KeyboardInterrupt`` never skips the stop command.  Each row keeps both the
    requested goal (``cmd_deg``/``cmd_units``) and the value the client echoed
    back in degrees (``cmd_echo_deg``), which may differ once the client clips
    the target.  ``rise_90_time_s`` is measured against the arrival stamp of the
    frame that crossed 90 % (``received_monotonic_s``), not against the poll
    clock that is read before the blocking read; a frame without a finite stamp
    falls back to the poll clock instead of dropping the measurement.

    ``record`` is an optional out-parameter: the summary (including the row of
    the interrupted step) is stored in ``record["summary"]`` from the ``finally``
    block, i.e. also when a ``KeyboardInterrupt`` propagates out of this
    function.  The interrupt itself keeps propagating unchanged; ``record`` only
    keeps the evidence from being lost with it.

    ``control_hz`` is the COMMAND rate and is independent of ``rate_hz``, the
    observation sampling rate: two consecutive commands are at least
    ``1 / control_hz`` apart, and the wait goes through the injected ``sleep``.
    Every command of the loop is paced — the first one of a step and every
    settle-loop repeat — so the burst a fake clock used to produce is not a
    possible behaviour any more.  The pause is only the remaining time to the
    next slot (at most one command interval), and ``rise_90_time_s`` is
    untouched by it because the crossing stays booked against the arrival stamp
    of the frame that crossed.  The closing hold is issued outside the pacer:
    delaying the stop command to a slot would delay the stop.

    ``control_hz`` must be a finite number ``> 0``; a non-finite or non-positive
    rate raises ``ValueError`` before anything is read or commanded.  It fails
    loud on purpose: ``1 / inf`` and ``1 / 0`` are both ``0.0``, and a pacing
    interval of ``0.0`` disables the throttling, so silently accepting such a
    value would reopen the unthrottled burst this pacer exists to prevent.  The
    CLI refuses the same values anyway; the check here covers direct callers.

    The settle tolerance decides whether a rise time can be measured at all.
    The 90 % crossing is booked when ``|travel| >= 0.9 * |delta_deg|``, i.e.
    when the residual is at most ``0.1 * |delta_deg|``, while the loop breaks as
    soon as the residual is at most ``settle_tolerance_deg``.  When
    ``settle_tolerance_deg >= 0.1 * |delta_deg|`` the break condition is the
    looser one, so the loop can exit on a frame whose residual never crossed
    ``0.1 * |delta_deg|``: ``reached_s`` stays ``None`` and ``rise_90_time_s``
    stays empty for that row.  That is a property of the knobs, not a broken
    measurement — ``--delta-deg 1.0`` with ``--settle-tolerance-deg 0.15`` is
    the default instance of it — and this function does not change the judgement
    order or the sampling to paper over it; the report surfaces it instead.
    """
    sleep_fn = time.sleep if sleep is None else sleep
    clock_fn = time.monotonic if clock is None else clock
    # Before anything is read or commanded: ``1 / inf`` and ``1 / 0`` are both
    # ``0.0``, and an interval of ``0.0`` is exactly "no pacing" — the hole this
    # check closes (``_pace_command`` no longer uses the interval as a switch).
    if not (math.isfinite(control_hz) and control_hz > 0.0):
        raise ValueError(
            f"control_hz must be a finite number > 0 (got {control_hz!r}); "
            "a non-finite or non-positive rate would disable the command pacing"
        )
    scale = _checked_unit_scale(deg_per_unit)
    period_s = 1.0 / rate_hz if rate_hz > 0 else 0.0
    # Minimum spacing between two commands.  Finite and positive by the check
    # above, so the pacer never has to interpret a ``<= 0`` interval as "off".
    command_interval_s = 1.0 / control_hz
    started_s = clock_fn()
    deadline_s = started_s + timeout_s
    rows: list[dict[str, Any]] = []
    abort_reason: str | None = None
    hold_state: dict[str, float] = {}
    hold_action: dict[str, float] = {}
    hold_error: str | None = None
    last_sequence = _observation_sequence(client) or 0
    stale_polls = 0
    # Counted for the report: they are what shows that the pacing worked and how
    # much sampling it cost (``achieved_command_hz`` / ``achieved_rate_hz``).
    commands_issued = 0
    observations = 0
    last_command_s: float | None = None

    def _pace_command(now_s: float) -> float:
        """Wait for the next command slot and return the stamp of that write.

        Consecutive commands stay at least ``1 / control_hz`` apart.  The pause
        is only the *remaining* time to the next slot, so one pause is at most
        one command interval and a loop that is already late sleeps nothing.
        It goes through the injected ``sleep``; the clock is read back afterwards
        because a real pause oversleeps, and the settle budget plus
        ``--timeout-s`` are judged against that same clock, so the time a pause
        costs has to reach them.
        """
        nonlocal last_command_s
        slot_s = now_s
        # ``command_interval_s`` is finite and positive (guaranteed by the check
        # at the top), so it is a duration here and never a "pacing on/off"
        # flag: using ``> 0.0`` as the switch is what let an infinite rate skip
        # the throttling entirely.
        if last_command_s is not None:
            remaining_s = command_interval_s - (now_s - last_command_s)
            if remaining_s > 0.0:
                sleep_fn(remaining_s)
                slot_s = clock_fn()
        last_command_s = slot_s
        return slot_s

    def _observe() -> tuple[Any, str | None]:
        """Read one frame, remember its pose and update the liveness counter."""
        nonlocal last_sequence, stale_polls, observations
        state = client.get_observation(include_cameras=False)
        observations += 1
        # Kept even when the frame is a cached repeat: the closing hold has to
        # repeat the last known pose of every joint, not an empty command.
        _merge_hold_state(hold_state, state)
        sequence = _observation_sequence(client)
        if sequence is not None and sequence > last_sequence:
            last_sequence = sequence
            stale_polls = 0
        else:
            stale_polls += 1
        return state, (STALE_FEEDBACK_REASON if stale_polls >= FEEDBACK_STALL_LIMIT else None)

    try:
        for step_index in range(steps):
            if clock_fn() >= deadline_s:
                abort_reason = "global --timeout-s exhausted before the step could start"
                break
            try:
                state, stale = _observe()
            except Exception as exc:  # any feedback failure stops the probe
                abort_reason = f"feedback read failed before the step: {exc}"
                break
            if stale is not None:
                abort_reason = stale
                break
            baseline_units = _finite_or_none(state.get(joint_key))
            if baseline_units is None:
                abort_reason = f"baseline feedback for {joint_key} is missing or non-finite"
                break

            baseline_deg = deg_from_units(baseline_units, scale)
            target_units = baseline_units + units_from_deg(delta_deg, scale)
            target_deg = baseline_deg + delta_deg
            # Paced before the row exists, so ``t_send_s`` is the stamp of the
            # write itself: a pacing pause may not inflate ``rise_90_time_s``.
            t_send_s = _pace_command(clock_fn())
            # The row exists before the first command and is appended in a
            # ``finally``: an interrupt in the middle of the step must still leave
            # that step in the CSV (``main`` reports it as an aborted run).
            row: dict[str, Any] = {
                "step_index": step_index,
                "t_send_monotonic_s": t_send_s,
                "received_monotonic_s": None,
                "cmd_deg": target_deg,
                "cmd_units": target_units,
                "deg_per_unit": scale,
                "cmd_echo_deg": None,
                "baseline_deg": baseline_deg,
                "measured_deg": baseline_deg,
                "error_deg": target_deg - baseline_deg,
                "rise_90_time_s": None,
                "overshoot_deg": 0.0,
                "settled": False,
                "note": "interrupted",
            }
            try:
                try:
                    echo = client.send_action(
                        _tracking_command(hold_state, {joint_key: target_units})
                    )
                    commands_issued += 1
                except Exception as exc:  # a rejected command stops the probe
                    abort_reason = f"command rejected: {exc}"
                    row["note"] = f"aborted: {abort_reason}"
                    break
                row["cmd_echo_deg"] = _echoed_deg(echo, joint_key, scale)

                measured_deg = baseline_deg
                peak_travel_deg = 0.0
                reached_s: float | None = None
                reached_received_s: float | None = None
                note = "settle budget exhausted"

                while True:
                    now_s = clock_fn()
                    if now_s >= deadline_s:
                        abort_reason = "global --timeout-s exhausted while settling"
                        break
                    if now_s - t_send_s >= settle_timeout_s:
                        break
                    try:
                        # Re-issue the goal on every poll: the host watchdog stops the
                        # robot after ~1 s without a command, so staying silent while the
                        # joint settles would be read as a lost link.  Paced like the head
                        # command, so the repeats cannot become a burst either.
                        _pace_command(now_s)
                        client.send_action(_tracking_command(hold_state, {joint_key: target_units}))
                        commands_issued += 1
                    except Exception as exc:  # a rejected command stops the probe
                        abort_reason = f"command rejected: {exc}"
                        break
                    try:
                        state, stale = _observe()
                    except Exception as exc:  # any feedback failure stops the probe
                        abort_reason = f"feedback read failed while settling: {exc}"
                        break
                    if stale is not None:
                        abort_reason = stale
                        break
                    measured_units = _finite_or_none(state.get(joint_key))
                    if measured_units is None:
                        abort_reason = f"feedback for {joint_key} is missing or non-finite"
                        break
                    measured_deg = deg_from_units(measured_units, scale)
                    travel_deg = measured_deg - baseline_deg
                    peak_travel_deg = max(peak_travel_deg, abs(travel_deg))
                    if reached_s is None and abs(travel_deg) >= 0.9 * abs(delta_deg):
                        # The crossing happened when this frame ARRIVED, which is
                        # stamped by the client; ``now_s`` was read before the
                        # blocking read and is one observation too early.
                        reached_received_s = _arrival_monotonic_s(client)
                        reached_s = now_s if reached_received_s is None else reached_received_s
                    if abs(measured_deg - target_deg) <= settle_tolerance_deg:
                        note = "settled"
                        break

                settled = abs(measured_deg - target_deg) <= settle_tolerance_deg
                row.update(
                    measured_deg=measured_deg,
                    error_deg=target_deg - measured_deg,
                    received_monotonic_s=reached_received_s,
                    rise_90_time_s=None if reached_s is None else reached_s - t_send_s,
                    overshoot_deg=max(0.0, peak_travel_deg - abs(delta_deg)),
                    settled=settled,
                    note="ok" if settled else note,
                )
                if abort_reason is not None:
                    row["note"] = f"aborted: {abort_reason}"
            finally:
                rows.append(row)
            if abort_reason is not None:
                break
            if period_s > 0.0:
                sleep_fn(period_s)
    except KeyboardInterrupt:
        # Recorded, then handed back to the caller unchanged: the interrupt keeps
        # propagating, but the rows and the summary stay readable through
        # ``record`` so ``main`` can write the interrupted run to disk.
        abort_reason = INTERRUPT_ABORT_REASON
        raise
    finally:
        # The hold lives in ``finally`` on purpose: on abort it is the stop
        # command, on a clean finish it freezes the joints instead of leaving the
        # last goal armed, and no exception may skip it.
        hold_action, hold_error = run_tracking_hold(client, hold_state)
        if hold_action:
            # The closing hold is a command of the run as well, so the achieved
            # command rate counts it.  It is not paced (see the docstring).
            commands_issued += 1
        summary = _tracking_summary(
            rows,
            abort_reason,
            hold_action,
            hold_error,
            commands_issued=commands_issued,
            observations=observations,
            elapsed_s=clock_fn() - started_s,
        )
        if record is not None:
            record["summary"] = summary

    return summary


def _merge_hold_state(hold_state: dict[str, float], state: Mapping[str, Any]) -> None:
    """Remember the last *finite* value of every measured key.

    A later frame that carries a NaN (or drops a key) must not erase the last
    known-good position, otherwise the hold command would silently omit that
    joint and leave it chasing its previous goal.
    """
    for key, value in (state or {}).items():
        number = _finite_or_none(value)
        if number is not None:
            hold_state[str(key)] = number


def run_tracking_hold(client: Any, state: Mapping[str, Any]) -> tuple[dict[str, float], str | None]:
    """Explicitly freeze the robot: measured joint goals plus zero base velocity.

    Positions are the last measured ``*.pos`` values (a hold where the robot is,
    not a return home), and the base body velocities are pinned to zero.  The
    hold covers the same key set as every other command: all measured ``*.pos``
    joints plus ``x.vel``/``y.vel``/``theta.vel``.

    It also pins ``lift_axis.height_mm`` to the height the last feedback
    measured, like every step command does, because that is the only part of
    the packet that stops the lift axis.  The reasoning: the command stream runs
    at 5-50 Hz, so the host's 1 s command watchdog never fires during a run, and
    the host's ``LiftAxis.apply_action`` writes no lift register at all for a
    packet without the key — the lift keeps moving on whatever ``Goal_Velocity``
    the previous session left behind.  Writing the measured height back is what
    pins the axis; a key the feedback never reported is not invented (a made-up
    ``0.0`` would command a descent).

    Returns the command that was written to the action socket and, when the host
    rejected it, the error text.  A rejected hold is not a normal abort: the stop
    command never reached the robot, so the caller must report it as
    ``EXIT_HOLD_FAILED``.
    """
    hold = _tracking_command(state)
    if not any(key.endswith(".pos") for key in hold):
        # No pose was ever measured, so no command was ever sent either: a packet
        # with only zero velocities would make the client fill every joint with
        # 0.0, i.e. command the arms to zero.  Nothing to freeze, nothing sent.
        return {}, None
    try:
        client.send_action(hold)
    except Exception as exc:  # report to the caller, which owns the exit code
        return {}, str(exc)
    return hold, None


def _format_tracking_report(
    summary: Mapping[str, Any],
    args: argparse.Namespace,
    joint_key: str,
    delta_deg: float,
    scale: float,
) -> str:
    rise = summary["rise_90_ms"]
    error = summary["steady_state_error_deg"]
    lines = [
        "AlohaMini joint step-response report (mode: tracking, REAL ROBOT MOTION)",
        f"  host={args.host} joint={joint_key} delta_deg={delta_deg:.3f} "
        f"(requested {float(args.delta_deg):.3f}) steps={args.steps}",
        f"  units: .pos feedback is normalized (RANGE_M100_100); 1 unit = {scale:.6f} deg, "
        f"so delta_deg={delta_deg:.3f} went out as {units_from_deg(delta_deg, scale):.4f} "
        "normalized units.  Every *_deg figure here is a degree",
        f"  steps_completed={summary['steps_completed']} abort_reason={summary['abort_reason']}",
        f"  rise_90_ms count={rise['count']} p50={rise['p50']:.1f} p95={rise['p95']:.1f} "
        f"max={rise['max']:.1f}",
        f"  max_overshoot_deg={summary['max_overshoot_deg']:.3f}",
        f"  steady_state_error_deg count={error['count']} p50={error['p50']:.3f} "
        f"p95={error['p95']:.3f} max={error['max']:.3f}",
        f"  settle_tolerance_deg={args.settle_tolerance_deg} "
        f"settle_timeout_s={args.settle_timeout_s} rate_hz={args.rate_hz}",
        # Achieved next to requested: the two rates the run really produced are
        # what shows that the pacing worked and what the sampling cost, so a
        # report can never imply the requested knobs were met.
        f"  achieved_command_hz={summary['achieved_command_hz']:.1f} "
        f"(requested control_hz={float(args.control_hz):.1f}) "
        f"achieved_rate_hz={summary['achieved_rate_hz']:.1f} "
        f"(requested rate_hz={float(args.rate_hz):.1f}) "
        f"commands={summary['commands_issued']} observations={summary['observations']} "
        f"over {summary['run_elapsed_s']:.3f} s",
        # "issued", not "sent": the command reached the action socket, which is
        # a PUSH/CONFLATE link without acknowledgement, so a write no peer
        # received looks exactly like a delivered one.
        f"  hold_issued={'yes' if summary['hold_action'] else 'no'} "
        "(written to the action socket, not a delivery receipt)",
        f"  csv={args.csv}",
    ]
    if rise["count"] == 0:
        # An empty column is a structural consequence of the knobs, not a failed
        # measurement: a crossing needs the residual at or below 0.1 * |delta|
        # before the settle loop ends, and the loop ends at the tolerance.  Point
        # at that relation instead of leaving a bare ``nan`` for the operator to
        # misread as a broken probe.
        lines.append(
            f"  note: rise_90_ms is empty (count=0) because no frame booked a 90 % "
            f"crossing; a crossing needs a residual at or below 0.1 * delta "
            f"({0.1 * abs(delta_deg):.3f} deg) before the settle loop ends, and the "
            f"settle tolerance ({args.settle_tolerance_deg} deg) decides which of the "
            "two comes first"
        )
    return "\n".join(lines)


def _run_tracking_mode(client: Any, args: argparse.Namespace) -> int:
    joint_key = _joint_state_key(args.joint)
    delta_deg = resolve_delta_deg(args)
    steps = int(args.steps)
    print(
        f"tracking mode (REAL ROBOT MOTION): joint={joint_key} delta_deg={delta_deg:.3f} "
        f"steps={steps} rate_hz={args.rate_hz} control_hz={args.control_hz}"
    )
    if args.settle_tolerance_deg >= 0.1 * abs(delta_deg):
        # Stated in the banner because it is a property of the knobs, not a sign
        # that this run went wrong: the crossing is booked at residual
        # <= 0.1 * |delta|, the settle loop breaks at residual <= tolerance, and a
        # tolerance at or above 0.1 * |delta| is the looser of the two.
        print(
            f"note: settle tolerance {args.settle_tolerance_deg} deg >= 0.1 * delta "
            f"({delta_deg:.3f} deg): the 90 % crossing may never be observed, "
            "so rise_90 can be empty"
        )

    try:
        observation = client.get_observation(include_cameras=False)
    except Exception as exc:  # no feedback, no motion
        print(f"connect failed: first feedback unavailable: {exc}", file=sys.stderr)
        return EXIT_CONNECT_FAILED

    scale, refusal = _tracking_preflight(client, args, observation, joint_key, delta_deg)
    if scale is None:
        print(
            refusal
            or (
                "Gate F refused: the unit conversion of the target joint is unavailable; "
                "nothing was commanded."
            ),
            file=sys.stderr,
        )
        return EXIT_MOTION_GATE_REFUSED

    record: dict[str, Any] = {}
    try:
        summary = run_tracking_steps(
            client,
            joint_key=joint_key,
            delta_deg=delta_deg,
            steps=steps,
            rate_hz=args.rate_hz,
            settle_timeout_s=args.settle_timeout_s,
            settle_tolerance_deg=args.settle_tolerance_deg,
            timeout_s=args.timeout_s,
            control_hz=args.control_hz,
            record=record,
            deg_per_unit=scale,
        )
    except KeyboardInterrupt:
        # The stop command was issued by ``run_tracking_steps``' ``finally`` block
        # and the partial summary is in ``record``: the interrupted run still
        # writes its rows, its report and a named interrupt on stderr, and it must
        # never look like a clean run.
        summary = record.get("summary") or _tracking_summary([], INTERRUPT_ABORT_REASON, {}, None)

    write_error: str | None = None
    # Same tail-interrupt rule as in rtt mode: the hold is already on the wire, the
    # write and the report are all that is left, and a Ctrl-C landing there may not
    # escape ``main``.  The interrupt becomes the abort reason, so the verdict order
    # below is unchanged (hold failure > write failure > interrupt).
    try:
        write_csv(args.csv, summary["rows"], TRACKING_CSV_FIELDS)
        print(_format_tracking_report(summary, args, joint_key, delta_deg, scale))
    except OSError as exc:
        print(f"write failed: {exc}", file=sys.stderr)
        write_error = str(exc)
    except KeyboardInterrupt:
        print(
            f"probe aborted: {INTERRUPT_ABORT_REASON}; interrupted while writing the "
            f"{len(summary['rows'])} row(s) or printing the report; the hold was already issued.",
            file=sys.stderr,
        )
        write_error = _write_csv_after_interrupt(args.csv, summary["rows"], TRACKING_CSV_FIELDS)
        if write_error is not None:
            print(f"write failed: {write_error}", file=sys.stderr)
        if summary["abort_reason"] is None:
            summary["abort_reason"] = INTERRUPT_ABORT_REASON

    if summary["hold_error"] is not None:
        # Reported before the CSV verdict below: a hold the host never accepted is
        # more urgent than a missing report file.
        print(f"hold command failed: {summary['hold_error']}", file=sys.stderr)
        print(
            "the hold (freeze) command did not reach the host: treat the robot as still moving, "
            "stop the host and cut power if anything keeps moving.",
            file=sys.stderr,
        )
        return EXIT_HOLD_FAILED
    if write_error is not None:
        return EXIT_WRITE_FAILED
    if summary["abort_reason"] is not None:
        print(f"probe aborted: {summary['abort_reason']}", file=sys.stderr)
        return EXIT_STEP_RESPONSE_ABORTED
    return EXIT_OK


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------


def main(
    argv: Sequence[str] | None = None,
    *,
    client_factory: Callable[[str, int, str, int], Any] | None = None,
) -> int:
    args = parse_args(argv)

    # Gate A runs before any client is built: a refused tracking run must never
    # open a socket.
    if args.mode == "tracking":
        refusal = _motion_gate_message(args)
        if refusal is not None:
            print(refusal, file=sys.stderr)
            return EXIT_MOTION_GATE_REFUSED

    factory = default_client_factory if client_factory is None else client_factory
    client = None
    try:
        # Four positional arguments: host, port, robot model and the observation
        # request window.  The window is part of the measurement (see the queue
        # term above), so it reaches the client config and never stays implicit.
        client = factory(args.host, args.port, args.robot_model, args.request_window)
        client.connect()
    except Exception as exc:  # every connect failure maps to one exit code
        print(f"connect failed: {exc}", file=sys.stderr)
        _close_quietly(client)
        return EXIT_CONNECT_FAILED

    # rtt mode reads ``latest_request_timing`` on every sampled frame, so a client
    # that does not publish it cannot be measured at all: name the missing face
    # and refuse here, ahead of ``_prime_state_only_window``.  The ordering is the
    # point: the prime is what sends this harness's own first batch of state-only
    # requests, and the deployed vendor ``main`` client does implement the prime,
    # so probing after it would spend a whole request window on a run that is then
    # refused.  That matches the exit code 9 docstring -- "before the state-only
    # window prime sends its first request", i.e. after ``connect()`` returned
    # (which prefilled the window with camera requests) and before the prime.
    # For a client with no ``prime_observation_request_window`` at all (the vendor
    # versions up to tag ``v0.6.0``) ``_prime_state_only_window`` getattr-misses
    # and returns without sending anything, so the order costs nothing there
    # either.  Missing faces are named explicitly, never as a defaulted ``None``
    # -- empty timing cells would read like a measurement of zero.
    if args.mode == "rtt":
        missing = missing_client_capabilities(client)
        if missing:
            print(
                "unsupported client: it publishes no "
                + ", ".join(missing)
                + ", so rtt cannot be measured.\n"
                "  This vendor client reports no per-request timing (a neighbour such as "
                "latest_safety_status is a different frame and is never substituted for it), "
                "and the request age is exactly what this mode reads.\n"
                "  Use a vendor version that publishes it and re-run; nothing was sampled "
                "and no CSV was written.",
                file=sys.stderr,
            )
            _close_quietly(client)
            return EXIT_CLIENT_UNSUPPORTED

    # ``connect()`` prefilled the window with camera requests: replace them with
    # state-only ones before the first sample is taken.
    _prime_state_only_window(client)

    try:
        if args.mode == "tracking":
            return _run_tracking_mode(client, args)
        return _run_rtt_mode(client, args)
    finally:
        _close_quietly(client)


if __name__ == "__main__":
    raise SystemExit(main())
