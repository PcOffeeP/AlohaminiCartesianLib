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
    Step one joint by ``--delta-deg`` at a time (its magnitude capped at
    ``MAX_STEP_DELTA_DEG``; a negative value steps the joint down) and measure
    the step response: 90 % rise time, overshoot and steady-state
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
``run_tracking_steps`` so a direct caller cannot bypass it.  It is independent
of ``--rate-hz``, the OBSERVATION sampling rate —
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
``--delta-deg`` may be negative (the same step taken the other way), and the
sign survives both the ``MAX_STEP_DELTA_DEG`` clamp and that conversion: only
the magnitude is capped, and only the limits are compared against ``abs()``.
``--anchor`` picks what that goal is laid on: ``present`` (the default) re-reads
that ``baseline_units`` from the measurement on every step, so the lead never
exceeds one delta and an error cannot accumulate, while ``command`` replaces
``baseline_units`` with the target the previous step sent (the first step falls
back to the measurement), so the lead accumulates across steps.  The row keeps
``baseline_deg`` / ``measured_deg`` as measurements either way.

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

Safety sidecar (tracking mode)
------------------------------
The vendor host publishes a second, *side-channel* frame on
``client.latest_safety_status`` every control loop: the signed motor currents in
milliamps, the hold flags and hold events, and the requested / accepted joint
targets.  It is part of neither ``get_observation()`` nor the tracking CSV, so a
tracking run writes it to a sidecar beside the tracking CSV
(``safety_csv_path``: ``tracking-<utc>-safety.csv``), one row per motor per
FRESH frame (``safety_frame_rows``).  The tracking table itself is frozen at its
14 columns -- downstream evidence files parse it as 14 columns -- so the safety
snapshot never becomes a column of it.  The sidecar carries nine columns: the
frame coordinates, the motor and its current, the requested / accepted target,
the hold flag and, last, ``target_source`` (a whole-frame attribute: which layer
produced that frame's targets, one of ``none`` / ``command`` / ``watchdog`` /
``protection``, empty when the frame says nothing usable).  ``hold_active`` is
structurally ``False`` on the pinned vendor client, which is why the column that
answers "was this target held back" is ``target_source`` read together with the
current trend (``docs/ADR.md`` D18).

"Fresh" is the liveness signal above: rows are booked only on a frame whose
``observation_sequence`` grew, at both recording sites (the step-start
observation and every settle-loop observation), so a cached repeat frame adds no
row.  ``t_monotonic_s`` is the frame's arrival stamp
(``latest_request_timing["received_monotonic_s"]``, ``_arrival_monotonic_s``)
with the poll clock of that observation as its fallback; the sidecar adds no
clock read of its own, so the pacing, the settle budgets and the achieved rates
keep being computed from exactly the readings they used before.  A run that had
to fall back names it on stderr exactly once -- one line for the whole run, not
one per frame -- saying that at least one of its ``t_monotonic_s`` stamps is the
poll clock because ``latest_request_timing`` could not answer; a run whose timing
face answered every frame stays silent, and the notice never names the safety
face, so it stays distinguishable from the two safety warnings below.

Size: one row per motor per fresh frame, so the number of rows grows with
``frames x motors``, and the frames grow with ``--steps``, ``--settle-timeout-s``
and ``--control-hz``.  On a long, fast run both the sidecar CSV and the row list
this harness holds in memory until the run ends get large -- the same shape the
tracking CSV has always had (one row per step, on a list held the same way), not
something the sidecar introduced.

Nothing here substitutes one face for another: ``latest_safety_status`` is a
different frame from ``latest_control_status`` and ``docs/ADR.md`` D12 forbids
standing one in for the other.  The safety snapshot lands in its own sidecar
columns, and every ``latest_control_status`` read of this harness is untouched.

A client that does not publish ``latest_safety_status`` still gets a sidecar: the
run probes the capability once (``missing_safety_capabilities``), names the face
on stderr exactly once, and writes rows whose ``motor`` and safety columns are
empty.  The face that *is* published and left at least one recorded frame without
a readable ``currents_ma`` motor table -- the degradation a real client really
hits, its attribute always exists -- gets the same empty cells and one warning
that counts the whole run (``frames=<recorded frames>``,
``frames_with_motor_table=<how many of them carried a table>``,
``_safety_frame_counts``), so 3 broken frames out of 4 are named instead of
passing as a healthy run.  A run whose every recorded frame carried a table is not
warned about at all, and the two branches are mutually exclusive: one stderr line
at most.  An empty cell is never a ``0`` -- see ``safety_frame_rows`` for why that
is a refusal to fabricate a measurement rather than a silent degradation.  A
sidecar that cannot be written is a warning, never an exit code: the tracking CSV
is the primary artefact of the run, and ``--mode rtt`` writes no sidecar at all
(it does not move the robot, so it has no safety frame to snapshot).

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
9  the connected client cannot provide the capability rtt mode measures with
   (``latest_request_timing``), so the request age cannot be measured at all.
   Three shapes share the code: the client does not publish that attribute
   (refused as soon as ``connect()`` returned, before the state-only window prime
   sends its first request and before the first sampled frame), it publishes it
   without all four keys the rtt row reads (``token``, ``sent_monotonic_s``,
   ``received_monotonic_s``, ``age_s``) holding usable values, or that unfilled
   face is still there when the sampling loop gives up.  The absent shape has its
   own sentence on stderr; the unfilled one -- the second and the third shape,
   i.e. one defect met at two moments -- names the unusable keys key by key,
   annotating a present-but-unusable value with its reason inside that same
   clause.  A key counts as filled only when it can give a finite value that
   physically means a measurement, so a face that carries all four keys with a
   ``None`` ``age_s`` is refused as well as a half-filled one, and so is one
   whose ``age_s`` is ``-0.5``, whose stamp or age is a ``bool``, or whose
   ``token`` is a number or blank.  The four rules, and why each one is a refusal
   rather than a conversion (``float()``/``str()`` accept every rejected shape,
   which is how they used to reach the CSV):

   * ``token`` has to be a ``str`` -- or the ``bytes`` the client stores, which
     ``_request_token_text`` decodes -- whose stripped text is not empty: a
     number becomes ``"123"`` and ``"   "`` is non-empty *text* but a blank cell,
     and neither can be paired with a request.  A merely meaningless token
     (``"abc"``) is a legal shape and is measured;
   * ``sent_monotonic_s`` / ``received_monotonic_s`` have to be finite numbers,
     and a ``bool`` is not a number: ``float(True) == 1.0`` wrote a stamp at the
     very start of the monotonic epoch;
   * ``age_s`` has to be a finite number ``>= 0``, and a ``bool`` is not a number:
     a request cannot be answered before it was sent, and
     ``net_rtt_estimate_s`` clamps the net estimate up to zero, so a negative age
     was written as a cell that reads like a fast round trip.  ``0.0`` stays
     legal -- a host that measures no age reports exactly that.

   "Not a finite number" covers a value ``float()`` cannot represent at all: an
   integer too large to be a double (``float(10**400)`` raises ``OverflowError``,
   which ``_finite_or_none`` does not catch).  That shape used to escape the probe
   and come back as exit 3, i.e. as a broken link instead of a broken client.
   The same holds for a value whose ``repr`` raises: the refusal cannot print it,
   but it can still say *why* it is unusable.

   The annotation contract inside the missing-key clause: an item is either
   ``<key>`` (the face does not carry that key at all) or ``<key> (<reason>)``
   with the reason **mandatory**, and the quoted value appended to it only when
   it can be quoted unchanged (``_short_value``: at most
   ``_QUOTED_VALUE_MAX_CHARS``, printable, no ``", "``).  The reason cannot be
   dropped to protect the clause and the clause cannot be broken to keep the
   reason: a bare key for a cell the face *does* carry sends the operator to the
   wrong defect, and a quote holding ``", "`` reads as one more key while a
   multi-line quote ends the list mid-word (both read back as keys that do not
   exist).  Dropping only the quote keeps both sides intact.

   The unfilled shape is judged strictly on EVERY fresh frame -- every growth of
   ``observation_sequence``, judged before that frame's row is appended -- so a
   face that goes empty in the middle of a run is refused as well, not only one
   that was empty from the start.  The timing of that probe is dictated by the
   data: a fresh frame is the earliest moment an unfilled face can be told from
   one the host simply had not timed yet.  It costs a client whose FIRST fresh
   frame is untimed the whole run (that client is refused even when every later
   frame is timed); that is a deliberate strict trade-off, not an oversight.  The
   same probe runs once more when the sampling loop gives up: a sequence frozen
   for the whole run never grows, so an untimed face is reported as 9 rather than
   as 7 ("insufficient samples") -- only after ``--timeout-s``, because nothing
   separates a frozen sequence from a slow one before the deadline.  A frozen
   sequence whose face *does* carry the keys keeps giving 7.  Before the first
   fresh frame the run has already done its single state-only window prime
   (``prime_observation_request_window``), i.e. one request at the default
   ``DEFAULT_REQUEST_WINDOW = 1``, so the empty-face refusal costs exactly that
   prime.  Either way no CSV was written -- not even a header -- and the
   connection is closed by ``main``'s ``finally``.  ``--mode tracking`` does not
   read that face and is unaffected.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

_BENCH_DIR = str(Path(__file__).resolve().parent)
if _BENCH_DIR not in sys.path:
    sys.path.insert(0, _BENCH_DIR)
_REPO_SRC = str(Path(__file__).resolve().parents[2] / "src")
if Path(_REPO_SRC).is_dir() and _REPO_SRC not in sys.path:
    sys.path.insert(0, _REPO_SRC)

from _ml_impl.cli import parse_args as _parse_args_impl  # noqa: E402
from _ml_impl.common import (  # noqa: E402, F401
    _QUOTED_VALUE_MAX_CHARS,
    ANCHOR_CHOICES,
    ANCHOR_COMMAND,
    ANCHOR_PRESENT,
    ARTIFACT_DIR,
    BASE_VELOCITY_KEYS,
    CLIENT_CAPABILITIES,
    DEFAULT_ANCHOR,
    DEFAULT_CONTROL_HZ,
    DEFAULT_DELTA_DEG,
    DEFAULT_HOST_LOOP_HZ,
    DEFAULT_MODE,
    DEFAULT_PORT,
    DEFAULT_RATE_HZ,
    DEFAULT_REQUEST_WINDOW,
    DEFAULT_ROBOT_MODEL,
    DEFAULT_SAMPLES,
    DEFAULT_SETTLE_TIMEOUT_S,
    DEFAULT_SETTLE_TOLERANCE_DEG,
    DEFAULT_STEPS,
    DEFAULT_TIMEOUT_S,
    EXIT_CLIENT_UNSUPPORTED,
    EXIT_CONNECT_FAILED,
    EXIT_HOLD_FAILED,
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_MOTION_GATE_REFUSED,
    EXIT_OK,
    EXIT_STEP_RESPONSE_ABORTED,
    EXIT_WRITE_FAILED,
    FEEDBACK_STALL_LIMIT,
    HOST_LOOP_HZ_UPPER_BOUND,
    HOST_METADATA_SCHEMA_VERSION,
    IDLE_POLL_FLOOR_S,
    INTERRUPT_ABORT_REASON,
    LIFT_HEIGHT_KEY,
    MAX_STEP_DELTA_DEG,
    MAX_TOTAL_TRAVEL_DEG,
    NORMALIZATION_DEGREES,
    NORMALIZATION_RANGE_0_100,
    NORMALIZATION_RANGE_M100_100,
    POS_SUFFIX,
    REQUIRED_REQUEST_TIMING_KEYS,
    RTT_CSV_FIELDS,
    RTT_RATE_WARNING_HZ,
    SAFETY_CLIENT_CAPABILITIES,
    SAFETY_CSV_FIELDS,
    SPAN_0_100,
    SPAN_M100_100,
    STALE_FEEDBACK_REASON,
    TARGET_SOURCE_VALUES,
    TRACKING_CSV_FIELDS,
    InsufficientSamples,
    UnfilledClientCapability,
    UnitConversionUnavailable,
    _capability_face_filled,
    _checked_anchor,
    _checked_unit_scale,
    _close_quietly,
    _default_csv_path,
    _finite_or_none,
    _host_loop_hz_or_upper_bound,
    _motor_name,
    _observation_sequence,
    _prefetch_terms_s,
    _prime_state_only_window,
    _quantiles,
    _queue_terms_s,
    _request_timing_key_problems,
    _request_token_text,
    _short_value,
    _timing_key_problem,
    _unfilled_capability,
    _write_csv_after_interrupt,
    _write_safety_sidecar,
    default_client_factory,
    deg_from_units,
    deg_per_unit,
    effective_request_window,
    host_sample_duration_s,
    missing_client_capabilities,
    missing_request_timing_keys,
    missing_safety_capabilities,
    net_rtt_estimate_s,
    percentile,
    safety_csv_path,
    summary_stats,
    unfilled_client_capabilities,
    units_from_deg,
    write_csv,
)
from _ml_impl.rtt import (  # noqa: E402, F401
    _achieved_rate_hz,
    _format_rtt_report,
    _host_clock_ns,
    _request_row,
    _rtt_rate_warning,
    _run_rtt_mode,
    collect_rtt_samples,
    resolve_delta_deg,
)
from _ml_impl.safety import (  # noqa: E402, F401
    _arrival_monotonic_s,
    _echoed_deg,
    _format_tracking_report,
    _joint_state_key,
    _merge_hold_state,
    _motion_gate_message,
    _safety_frame_counts,
    _safety_hold_flag,
    _safety_mapping,
    _safety_number,
    _safety_row,
    _safety_target,
    _safety_target_source,
    _tracking_command,
    _tracking_preflight,
    _tracking_summary,
    safety_frame_rows,
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments for the AlohaMini observation/tracking benchmark."""
    return _parse_args_impl(argv, description=__doc__)


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
    deg_per_unit: float = 1.0,  # noqa: F811
    anchor: str = DEFAULT_ANCHOR,
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

    ``anchor`` picks what that goal is laid on.  ``ANCHOR_PRESENT`` (the default)
    lays it on the measurement of this step, so the lead never exceeds one delta
    and an error cannot accumulate; ``ANCHOR_COMMAND`` lays it on the target the
    previous step sent (the first step falls back to the measurement), so the goal
    walks away from a joint that does not follow and the lead accumulates.  Only
    the anchor moves: ``baseline_deg`` / ``measured_deg`` stay measurements,
    ``error_deg`` stays ``target - measured``, and the ``MAX_STEP_DELTA_DEG``
    clamp and the ``MAX_TOTAL_TRAVEL_DEG`` cap are untouched.  An unknown value
    raises ``ValueError`` before anything is read or commanded.

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

    Safety sidecar: every frame whose ``observation_sequence`` grew is also
    snapshotted into ``summary["safety_rows"]`` (``safety_frame_rows``, one row per
    motor), at the step-start observation as well as on every settle poll, in
    arrival order; a cached repeat frame adds nothing, because the same growth test
    that feeds the stall counter decides it.  ``summary["safety_missing"]`` is the
    capability probe of the run's start (``missing_safety_capabilities``); when it
    names ``latest_safety_status`` the rows are still recorded, with empty cells,
    and ``_run_tracking_mode`` says so on stderr once.  These rows are evidence
    *beside* the measurement and never change its outcome: an absent face and a
    corrupt one leave the rows, the abort verdicts and the exit code untouched.
    Nothing is substituted either: the safety face fills its own sidecar columns and
    no ``latest_control_status`` path of this harness is touched (``docs/ADR.md``
    D12).

    ``t_monotonic_s`` of a sidecar row is the frame's arrival stamp
    (``_arrival_monotonic_s``) and, when the frame carries none, the poll-clock
    reading the caller had already taken for that observation -- no clock read is
    added here, so the pacing, the settle budgets and the achieved rates keep being
    judged against exactly the readings they were judged against before the sidecar
    existed.  The fallback is a property of the whole run, not of a frame: as soon
    as one recorded frame (or the 90 % crossing) had to be stamped by the poll
    clock, ``summary["timestamp_fallback"]`` is ``True`` and ``_run_tracking_mode``
    names it on stderr exactly once, instead of repeating the note on every frame
    or leaving the operator to guess why the stamps look like poll instants.
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
    # Same rule for the aim of the run, and in the same place: before the first
    # ``get_observation``, the first ``send_action`` and the safety-face probe.
    anchor = _checked_anchor(anchor)
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
    # The target of the last accepted command: the anchor of the next step in
    # ``ANCHOR_COMMAND`` mode.  ``None`` until one command was accepted, so the
    # first step (and a run whose first step never reaches the wire) falls back to
    # the measured position.
    last_target_units: float | None = None
    last_sequence = _observation_sequence(client) or 0
    stale_polls = 0
    # Counted for the report: they are what shows that the pacing worked and how
    # much sampling it cost (``achieved_command_hz`` / ``achieved_rate_hz``).
    commands_issued = 0
    observations = 0
    last_command_s: float | None = None
    # Safety sidecar: the capability is probed ONCE, at the run's start, so the
    # answer cannot drift between "the client publishes the face" and "the run said
    # it does not", and every fresh frame of the run is recorded the same way.  The
    # rows accumulate in arrival order and reach the caller through the summary; a
    # missing face is reported by name (``safety_missing``) instead of being papered
    # over by a defaulted ``getattr``.
    safety_missing = missing_safety_capabilities(client)
    safety_rows: list[dict[str, Any]] = []
    # Run-wide, not per frame: set as soon as one recorded frame (or the 90 %
    # crossing) had to be stamped with the poll clock because the timing face could
    # not answer an arrival stamp.  It travels to the caller through the summary so
    # ``_run_tracking_mode`` can name the fallback once for the whole run.
    timestamp_fallback = False

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

    def _safety_face() -> Any:
        """The safety frame the client publishes, or ``None`` when it has none.

        Gated on the run-start probe instead of a defaulted ``getattr``: a client
        without the face is *named* (``safety_missing``, one stderr line in
        ``_run_tracking_mode``) and its rows carry empty cells, so an absent face is
        never quietly read as a frame that said nothing.  The attribute read is
        guarded as well: a face that vanishes mid-run reads as an absent one, because
        no side-channel read may abort the measurement.
        """
        if safety_missing:
            return None
        try:
            return client.latest_safety_status
        except Exception:  # a face that vanished mid-run is not a broken link
            return None

    def _observe(step_index: int, poll_s: float) -> tuple[Any, str | None]:
        """Read one frame, remember its pose and update the liveness counter.

        ``poll_s`` is the poll-clock reading the caller took for this observation
        (before the blocking read).  It stamps the safety snapshot of a frame whose
        client publishes no arrival stamp; it is passed in rather than read here so
        that an observation costs exactly the clock reads it always cost -- the
        pacing, the settle budgets and the achieved rates are arithmetic over that
        clock -- and so ``step_index`` is the step the frame was polled in.
        """
        nonlocal last_sequence, stale_polls, observations, timestamp_fallback
        state = client.get_observation(include_cameras=False)
        observations += 1
        # Kept even when the frame is a cached repeat: the closing hold has to
        # repeat the last known pose of every joint, not an empty command.
        _merge_hold_state(hold_state, state)
        sequence = _observation_sequence(client)
        if sequence is not None and sequence > last_sequence:
            last_sequence = sequence
            stale_polls = 0
            # One snapshot per FRESH frame, booked on the growth of the very counter
            # the stall detector watches (never at the step tail only): a cached
            # repeat carries the numbers the sidecar already holds, so re-booking it
            # would duplicate a frame the sidecar already has.  Both recording sites
            # are this function, so the step-start frame and every settle-loop poll
            # are covered alike.
            arrival_s = _arrival_monotonic_s(client)
            if arrival_s is None:
                timestamp_fallback = True
            safety_rows.extend(
                safety_frame_rows(
                    _safety_face(),
                    step_index=step_index,
                    sample_sequence=sequence,
                    t_monotonic_s=poll_s if arrival_s is None else arrival_s,
                )
            )
        else:
            stale_polls += 1
        return state, (STALE_FEEDBACK_REASON if stale_polls >= FEEDBACK_STALL_LIMIT else None)

    try:
        for step_index in range(steps):
            # The deadline check reads the poll clock anyway; the reading is kept
            # because it is the poll stamp of this step-start observation, i.e. the
            # one ``_observe`` needs for the safety snapshot of that frame.
            poll_s = clock_fn()
            if poll_s >= deadline_s:
                abort_reason = "global --timeout-s exhausted before the step could start"
                break
            try:
                state, stale = _observe(step_index, poll_s)
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
            # ``baseline_units`` / ``baseline_deg`` stay measurements: the anchor is
            # what the goal is laid on, and it is the measurement itself in
            # ``ANCHOR_PRESENT`` mode and the target of the previous step in
            # ``ANCHOR_COMMAND`` mode.  The default mode therefore does exactly the
            # arithmetic it always did, while the command mode lifts the one-step
            # lead so an error can accumulate.
            anchor_units = (
                baseline_units
                if anchor == ANCHOR_PRESENT or last_target_units is None
                else last_target_units
            )
            anchor_deg = deg_from_units(anchor_units, scale)
            target_units = anchor_units + units_from_deg(delta_deg, scale)
            target_deg = anchor_deg + delta_deg
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
                # The command reached the wire: in ``ANCHOR_COMMAND`` mode it becomes
                # the anchor of the next step.  Recorded after the accepted
                # ``send_action``, so a rejected step leaves the anchor where it was.
                last_target_units = target_units

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
                        state, stale = _observe(step_index, now_s)
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
                        if reached_received_s is None:
                            timestamp_fallback = True
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
            safety_rows=safety_rows,
            safety_missing=safety_missing,
            timestamp_fallback=timestamp_fallback,
        )
        if record is not None:
            record["summary"] = summary

    return summary


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
    if args.anchor == ANCHOR_COMMAND:
        # Only for this mode: ``present`` is the shape every earlier run had, so its
        # stdout stays exactly what it was.
        print(
            "note: anchor=command: each step anchors on the previous target, so the lead "
            "accumulates across steps (present re-anchors on the measured position every step); "
            "rise_90_time_s, overshoot_deg and error_deg are not comparable between the two "
            "anchors: a step can close a lead spanning several deltas, so rise_90 is booked "
            "early, overshoot_deg becomes (travel - one delta) and error_deg settles at the "
            "accumulated lead instead of at one step's error"
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
            anchor=args.anchor,
        )
    except KeyboardInterrupt:
        # The stop command was issued by ``run_tracking_steps``' ``finally`` block
        # and the partial summary is in ``record``: the interrupted run still
        # writes its rows, its report and a named interrupt on stderr, and it must
        # never look like a clean run.
        summary = record.get("summary") or _tracking_summary([], INTERRUPT_ABORT_REASON, {}, None)

    write_error: str | None = None
    sidecar_path = safety_csv_path(args.csv)
    # Counted from the very rows the sidecar is written from, so the warning and the
    # file can never disagree about how much of the run came out empty.
    frames, frames_with_motor_table = _safety_frame_counts(summary["safety_rows"])
    if summary["safety_missing"]:
        # Exactly one line, and it names the face the probe is missing: the absent
        # capability is reported, never read through a defaulted ``getattr``.  The
        # sidecar is still written (empty safety cells, one row per frame) and the
        # exit code is untouched -- no column, no verdict and no rate of this run
        # depends on the safety face.
        print(
            f"warning: the client publishes no {', '.join(summary['safety_missing'])}; the safety "
            f"sidecar {sidecar_path} records empty cells for the motor currents, the requested / "
            "accepted targets and the hold flags.  The run itself is unaffected: the tracking CSV, "
            "the report and the exit code do not depend on that face.",
            file=sys.stderr,
        )
    elif frames_with_motor_table < frames:
        # The face exists but at least one of the recorded frames carried nothing
        # usable -- the degradation the deployed client really hits, since the
        # attribute always exists (initial ``{}``) and only its contents can be
        # empty.  Judged over the whole run and not over its first frame: a run of 4
        # frames with 3 broken ones used to pass in silence (it only warned when NO
        # frame carried a table), leaving the blank ``motor`` cells of the sidecar
        # unmentioned.  ``frames_with_motor_table < frames`` is false for a run that
        # recorded no frame at all, so an abort before the first frame warns nothing;
        # a host that fills the frame on every recorded poll gets no warning either.
        # The two branches are mutually exclusive, so one line at most, and the exit
        # code never depends on this face.
        print(
            "warning: the client publishes latest_safety_status, but "
            f"{frames - frames_with_motor_table} of the {frames} frame(s) this run recorded "
            "carried no readable currents_ma motor table (frames="
            f"{frames}, frames_with_motor_table={frames_with_motor_table}), so the safety "
            f"sidecar {sidecar_path} records empty cells for the motor currents, the requested / "
            "accepted targets and the hold flags on those frames.  The run itself is unaffected: "
            "the tracking CSV, the report and the exit code do not depend on that face.",
            file=sys.stderr,
        )
    # A separate ``if`` and not another branch of the chain above: that chain is
    # about the *safety* face and its two branches are mutually exclusive, while
    # this notice is about the *timing* face and may accompany either of them.  One
    # line for the whole run, never one per frame or per poll: the fallback is a
    # property of the run, and the operator needs to know once that at least one
    # of its timestamps -- a sidecar ``t_monotonic_s`` stamp or the 90 % crossing's
    # arrival instant, not necessarily every one of them -- is a poll instant rather
    # than the client's arrival stamp.  It names ``latest_request_timing`` and never
    # ``latest_safety_status``, so the two kinds of line stay distinguishable.
    if summary.get("timestamp_fallback"):
        print(
            f"warning: at least one timestamp of this run is the poll clock, not the client's "
            f"arrival time: latest_request_timing answered no usable received_monotonic_s for "
            f"the t_monotonic_s stamp of a safety sidecar row ({sidecar_path}) or for the "
            "arrival instant of the 90 % rise crossing, so that stamp is the reading taken "
            "before the blocking read.  The measurement itself is unaffected: no clock read "
            "was added, and the pacing, the settle budgets and the achieved rates are judged "
            "against the same readings as before.",
            file=sys.stderr,
        )
    # Same tail-interrupt rule as in rtt mode: the hold is already on the wire, the
    # write and the report are all that is left, and a Ctrl-C landing there may not
    # escape ``main``.  The interrupt becomes the abort reason, so the verdict order
    # below is unchanged (hold failure > write failure > interrupt).
    try:
        write_csv(args.csv, summary["rows"], TRACKING_CSV_FIELDS)
        # After the primary artefact, and best-effort: a sidecar that cannot be
        # written is a warning, never ``EXIT_WRITE_FAILED`` (see
        # ``_write_safety_sidecar``).
        _write_safety_sidecar(sidecar_path, summary["safety_rows"])
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
        # The sidecar gets the same best-effort retry, and with the same verdict:
        # its own failure (or a second interrupt) is a warning, not a write failure
        # of the run.
        _write_safety_sidecar(sidecar_path, summary["safety_rows"], after_interrupt=True)
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
    """CLI entrypoint for ``measure_latency.py``."""
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
