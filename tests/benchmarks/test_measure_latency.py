"""Offline contract tests for ``benchmarks/alohamini_io/measure_latency.py``.

The script is loaded **by path** (``importlib.util.spec_from_file_location``)
because it is not a package module; ``_load_script`` does that and the module-scope
``latency`` fixture hands the module to every test.  Nothing here needs a robot:
the file ships its own scriptable stand-ins (``_FakeClient``, ``_TrackingFake``,
``_ScriptedTrackingFake``, ``_PacingClient``), an injected clock
(``_FakeClock`` / ``_ManualClock``) and a no-op sleep, so no socket is opened, no
port is bound, no ``lerobot`` hardware import happens and no test waits on real
time.  Every artefact goes to ``tmp_path``.

What the tests below lock down (the banners group them by topic):

* **percentile semantics** — linear interpolation between ranks (``p95`` of
  ``[1..5]`` is ``4.8``, not the nearest-rank ``5.0``), ``nan`` for an empty
  sequence, and the five-number summary of a run;
* **unit conversion** — a ``.pos`` value is a normalized unit, so one unit is
  ``((range_max - range_min) / span) * 360 / (resolution - 1)`` degrees, with
  ``span`` 200 for ``range_m100_100``, 100 for the ``range_0_100`` gripper and
  ``1.0`` for ``degrees``; a missing or nonsense calibration is a refusal, never
  a guessed factor (sections ``J`` and ``O``);
* **net round trip and its three bounds** — ``net_rtt_est_s`` / ``_low`` /
  ``_high`` after the host sample, the phase mean and bound, the queue term (only
  at or above the host rate) and the client prefetch term of ``window - 1``
  periods (sections ``H`` and ``K``);
* **CSV row structure** — the header equals the frozen field tuples, one line per
  sample/step, an empty cell (never ``0``) for an absent host clock baseline, a
  ``KeyError`` for a row that misses a field, and text tokens only;
* **exit-code layering** — 0 ok / 1 aborted / 3 connect / 5 write / 6 gate /
  7 insufficient samples / 8 hold failed, each reachable through ``main``;
* **the motion gates** — Gate A (``--enable-motion`` / ``--joint``) before any
  client is built, Gate B (joint absent from the state schema), Gate C
  (``robot_model`` missing or different), Gate D/E (baseline, step size, travel
  cap) and Gate F (unit conversion unavailable, or a metadata ``schema_version``
  that is not ``2``) — every refusal is zero ``send_action`` and no CSV;
* **feedback liveness** — a frozen ``observation_sequence`` (the client answers a
  timed out observation with its cached frame) aborts after
  ``FEEDBACK_STALL_LIMIT`` consecutive polls, keeps the evidence and still holds
  the robot (section ``E``);
* **stop and hold shape** — every command is full-shaped (all measured pose
  goals, the three base velocities pinned to zero, the measured lift height), the
  hold is issued from a ``finally`` block, its wording is ``hold_issued`` and a
  rejected hold has its own exit code (the tracking banners and section ``L``);
* **Ctrl-C accounting** — in both modes: the CSV keeps what was already
  collected, the tracking hold is issued, the interrupt is named on stderr and
  ``main`` returns 1 instead of letting ``KeyboardInterrupt`` escape (section
  ``F``);
* **CLI validation** — nonsense ``--samples`` / ``--rate-hz`` /
  ``--request-window`` / ``--steps`` / settle shapes, and a
  ``--settle-tolerance-deg`` at or above the effective ``--delta-deg``, are usage
  errors (2) instead of failed connections (3) (sections ``I`` and ``N``);
* **timing evidence** — ``rise_90_time_s`` is measured against the arrival stamp
  of the crossing frame, not the poll clock (section ``M``);
* **command pacing** — ``--control-hz`` (default ``50.0``) spaces two consecutive
  ``send_action`` calls by at least ``1 / control_hz`` seconds independently of
  ``--rate-hz``, the pacing goes through the injected ``sleep`` and never through
  ``time.sleep``, and the tracking summary states the achieved command *and*
  sample rates (section ``Q``).

Contract note (frozen here as the expectation):
``resolve_delta_deg(args) -> float`` is a pure helper on the module. The
tracking branch must clamp ``--delta-deg`` through it and print the value
actually used. The clamp applies to the **magnitude** and keeps the sign
(``min(max(delta, -MAX_STEP_DELTA_DEG), MAX_STEP_DELTA_DEG)``), because a
negative ``--delta-deg`` is a legal step: the same step-response experiment run
in the opposite direction (field manual §10.5, the "M2" entry; section ``AA``
below). What ``parse_args`` refuses is a **zero** step (``0`` / ``-0.0``) and
the non-finite values ``math.isfinite`` catches -- the sign bound is
``!= 0``, never ``> 0``, and ``MAX_STEP_DELTA_DEG`` stays ``1.0``.

Regression intent (each entry is a plausible-but-wrong implementation that
must turn this file red):

* **percentile by nearest rank** — ``p95`` of ``[1,2,3,4,5]`` would be ``5.0``
  instead of the linear-interpolated ``4.8``; ``p99`` would be ``5.0`` instead
  of ``4.96``;
* **clock-domain mix-up** — subtracting absolute host monotonic stamps from a
  PC-clock ``age_s`` (instead of the host *duration*) must fail the
  ``net_rtt_estimate_s`` value assertions, including the clamp case;
* **sneaky motion in rtt mode** — an rtt run that calls ``send_action`` must
  fail the zero-call assertion, and a ``send_action`` reference outside a
  tracking function must fail the source-line assertion;
* **gate after connect** — checking the motion gate after connecting must fail
  the "factory never called" assertion;
* **bytes token in CSV** — a raw ``bytes`` token reaches ``csv`` as ``b'...'``,
  so every emitted token must be ``str``;
* **silent short sampling** — returning fewer rows than requested instead of
  raising ``InsufficientSamples`` must fail the row-count assertions;
* **missing-key tolerance in write_csv** — a row lacking a declared field must
  raise ``KeyError`` rather than write a silently shifted line;
* **capability probe after the window prime** — moving the rtt probe back behind
  ``_prime_state_only_window`` must fail the ``prime_calls == []`` assertion of
  the exit-9 test, and dropping the prime from the tracking path must fail
  section ``S``;
* **one-shot capability probe** — a ``probed`` latch that checks the timing face
  only at the first growth of ``observation_sequence`` must fail R11 (a face that
  empties from row 3 on would otherwise be written out as empty cells and the
  run would exit 0).
* **half-filled timing face accepted** — a non-empty face missing one of the four
  keys the row reads (``token`` / ``sent_monotonic_s`` /
  ``received_monotonic_s`` / ``age_s``) must fail R20: exit 9, no CSV, and stderr
  naming the missing key.  R21/R22 keep the other side green (all four keys, or
  four plus extra ones, must stay measurable), so the probe cannot be turned into
  "refuse anything that is not exactly these four keys".
* **frozen sequence at 0 rows spins** — a pacing loop whose pause is
  ``min(start + 0 * period - now, deadline - now)`` never sleeps, so the frozen
  case polls at the client's maximum speed; R24 bounds the poll count and R23
  requires that same run to be reported as a *capability* refusal (9), not as
  "insufficient samples" (7), whenever the timing face is untimed.

.. warning::
   ``send_action`` may only be *called* inside a tracking function of the module
   source — the source assertion below parses the module with ``ast`` and fails
   on any reference outside a ``def`` whose name contains ``tracking``, and on
   any string literal that mentions the name.

Round-2 contracts (still the frozen expectation; the tests below pin exactly
this behaviour):

* ``net_rtt_estimate_s(age_s, host_timing, *, host_loop_hz=HOST_LOOP_HZ_UPPER_BOUND,
  request_window=1)`` returns ``age_s``, ``host_sample_duration_s``,
  ``phase_bound_s``, ``phase_mean_s``, ``queue_bound_s``, ``queue_mean_s``,
  ``host_timing_missing``, ``net_rtt_est_s``, ``net_rtt_low_s``,
  ``net_rtt_high_s``.  The client keeps ``observation_request_window`` requests
  in flight and the host answers one per loop turn, so ``window - 1`` requests
  are queued ahead of the answer:
  ``queue_bound_s = max(0, request_window - 1) / host_loop_hz`` and
  ``queue_mean_s = queue_bound_s / 2``.  The point estimate subtracts
  ``sample + phase_mean + queue_mean``, the low bound subtracts
  ``sample + phase_bound + queue_bound`` and the high bound subtracts only
  ``sample``.  ``request_window=1`` is the default and keeps the round-1 numbers
  bit-for-bit, so the three round-1 assertions stay untouched.
* ``--request-window`` exists, defaults to ``1`` (round 4; it was ``3``, the
  client default, in round 2 — see the prefetch bullet below) and is forwarded
  to the client factory as the **fourth positional argument**:
  ``client_factory(host, port, robot_model, request_window)``.  ``_Factory``
  below records it in ``window_args`` and keeps ``calls`` as three-tuples so the
  round-1 factory assertion stays valid.
* every command ``run_tracking_steps`` sends is a *full* command: all ``*.pos``
  keys of the last feedback plus ``x.vel`` / ``y.vel`` / ``theta.vel`` pinned to
  ``0.0``.  A step command carries ``baseline + delta_deg`` on the target joint,
  the closing hold carries the measured positions.
* each row records ``cmd_deg`` (the requested target) and ``cmd_echo_deg`` (the
  value the client *echoed back* through ``send_action`` — it may differ from
  the request, and the record must show the echo, not the request).
* the hold is issued from a ``finally`` block, so it is still sent when the
  stepping loop raises ``KeyboardInterrupt``.
* a hold that itself fails returns the new ``EXIT_HOLD_FAILED == 8`` and prints
  ``hold`` on stderr (it must not be folded into ``EXIT_STEP_RESPONSE_ABORTED``).
* ``collect_rtt_samples`` paces against an **absolute** deadline
  (``start + n / rate_hz``) instead of sleeping one whole period after each
  blocking read, and the rtt report prints the achieved rate using the words
  ``achieved`` and ``Hz``.
* ``RTT_CSV_FIELDS`` carries ``state_sample_unix_ns`` (the host clock baseline,
  an empty cell when the host does not report one) and every emitted row covers
  every declared field.

Round-4 contracts (a later reviewer round; the two findings behind them are
factual errors of the round-3 harness that must not be "improved" back):

* **A ``.pos`` feedback value is a normalized unit, not a degree.**
  ``AlohaMiniConfig.use_degrees`` defaults to ``False`` and there is no Host CLI
  switch that turns it on, so ``AlohaMini`` normalizes every arm joint to
  ``MotorNormMode.RANGE_M100_100`` and ``MotorsBus._normalize`` maps that to
  ``((raw - min) / (max - min)) * 200 - 100``.  One observed unit is therefore
  ``(range_max - range_min) / 200`` ticks, i.e. ``((range_max - range_min) /
  200) * 360 / (resolution - 1)`` degrees.  Every degree figure this harness
  prints or compares — ``--delta-deg``, ``MAX_STEP_DELTA_DEG``,
  ``MAX_TOTAL_TRAVEL_DEG``, ``settle_tolerance_deg``, ``cmd_deg``,
  ``measured_deg``, ``baseline_deg``, ``error_deg``, ``overshoot_deg`` and
  ``rise_90`` — must be produced through ``deg_per_unit`` + ``units_from_deg`` /
  ``deg_from_units``, never assumed.  A joint whose calibration cannot be read
  is a refused run (``EXIT_MOTION_GATE_REFUSED``), never a guessed factor.
* **``age_s`` at the default client rate is dominated by the prefetch
  pipeline.**  ``AlohaMiniClient._poll_and_get_latest_message`` consumes the
  *oldest* token and refills the window only after the answer, so the answered
  request was sent ``window`` client calls ago: ``age_s`` carries
  ``(window - 1) * max(1 / rate_hz, 1 / host_loop_hz)`` of prefetch wait that
  round 3 never subtracted, and the round-3 gate had switched it off exactly
  when the client is slower than the host.  ``--request-window`` therefore
  defaults to **1** so that a plain default run measures one explicable round
  trip; ``3`` is the deployment shape and has to be asked for explicitly.
  ``net_rtt_estimate_s`` gains ``rate_hz``; the prefetch term always uses the
  *requested* window (the deployment number), the queue term keeps its round-3
  gate.
* **``net_rtt_high_s = max(0, age_s - host_sample_duration_s)``** — the upper
  bound discounts the host sample only and subtracts *no* waiting term, while
  the point estimate and the low bound subtract the prefetch / phase / queue
  terms.  The waiting terms therefore act on ``net_rtt_est_s`` and
  ``net_rtt_low_s`` alone, which keeps ``low <= est <= high`` true by
  construction (the mean never exceeds its bound).
* **Every command pins ``lift_axis.height_mm`` to the latest measured height.**
  A command stream at 5-50 Hz keeps the Host's 1 s command watchdog from ever
  firing, and the Host's ``LiftAxis.apply_action`` writes no lift register at
  all when the key is absent — the lift then keeps moving on whatever
  ``Goal_Velocity`` the previous session left behind.  Writing the measured
  height back is what pins the axis; when the feedback carries no such key the
  command must not invent one (a made-up ``0.0`` would command a descent).
* **``rise_90_time_s`` is measured against the frame arrival stamp.**
  ``client.latest_request_timing["received_monotonic_s"]`` is when the crossing
  sample was actually received; the poll-clock read happens before the blocking
  read and inflates the rise time by one observation.  The row records that
  stamp in ``received_monotonic_s``; a frame that carries no finite stamp falls
  back to the poll clock (round-3 behaviour) instead of dropping the measurement.

Round-3 contracts (a reviewer round; the tests below pin exactly this
behaviour).  The motivating
fact is in the client: ``_get_data`` returns the *cached* frame when the
observation transport times out, so ``get_observation`` never raises on a dead
link and the only liveness signal is ``client.observation_sequence`` growing
(it is bumped only on a successful receive).

* **Feedback liveness (chosen variant: the call-count limit).**
  ``run_tracking_steps`` tracks ``client.observation_sequence`` on the
  step-boundary poll *and* on every settle-loop poll; the count of consecutive
  ``get_observation`` calls whose sequence did not grow is consecutive over the
  whole run and is **not** reset at a step boundary.  When it reaches
  ``FEEDBACK_STALL_LIMIT == 3`` the run aborts with an ``abort_reason`` that
  contains both ``stale`` and ``feedback``, breaks out of the loop immediately,
  still merges the returned (cached) feedback into the hold state so the
  closing ``send_action`` is a full-shape hold, and returns
  ``steps_completed < steps``.  ``main`` maps that abort to
  ``EXIT_STEP_RESPONSE_ABORTED == 1`` and still writes the CSV (at least the
  header) so the evidence survives.  A timeout-based variant
  (``FEEDBACK_STALL_TIMEOUT_S``) was considered and rejected here: the tests
  drive the clock, and a call-count limit is what the assertions pin.
* **Ctrl-C.** ``main()`` never lets ``KeyboardInterrupt`` escape, in either
  mode: it writes the CSV (at least one row, i.e. the interrupted step is
  recorded — in rtt mode the samples already taken), prints the
  report (rtt: the quantile summary of the partial sample set), issues the hold
  (tracking only) and returns ``EXIT_STEP_RESPONSE_ABORTED == 1`` with
  ``interrupt`` on stderr.  ``run_tracking_steps`` itself keeps propagating
  ``KeyboardInterrupt`` (round-2 test above), so the recording has to happen
  where the partial step is known.
* **Hold wording.** The tracking report labels the closing command
  ``hold_issued`` (issued, no receipt) and must not print ``hold_sent=yes``:
  ``send_action`` is ZMQ PUSH + CONFLATE, so a send that no peer received looks
  exactly like a delivered one and "sent" reads as an acknowledgement.
* **Gate C fail-closed.** Empty host metadata, metadata without
  ``robot_model`` and a ``robot_model`` that differs from ``--robot-model`` all
  refuse with ``EXIT_MOTION_GATE_REFUSED``, zero ``send_action`` calls and no
  CSV.
* **Gate F and the metadata schema.** Besides an unavailable unit conversion,
  Gate F refuses a ``latest_robot_metadata`` whose ``schema_version`` is absent
  or not the integer ``2`` — the same rule ``capture_robot_fingerprint``
  applies before it stores a fingerprint, so a calibration table of an unknown
  layout is never trusted.  Refusal is ``EXIT_MOTION_GATE_REFUSED``, zero
  ``send_action`` and no CSV.
* **``--host-loop-hz``** (default ``50.0``) feeds the phase bound and lets a
  measured host rate be fed back in.  The queue term of a row is only counted
  when the *requested* ``args.rate_hz >= args.host_loop_hz`` (the achieved rate
  is not used, so the row is deterministic); otherwise the row is computed with
  ``request_window=1``, i.e. ``queue_bound_s == 0.0``.  The request rate, not
  the host rate, decides, because a client asking slower than the host serves
  drains the request window instead of queueing behind it.
  ``queue_bound_s`` joins ``RTT_CSV_FIELDS``.
* **CLI ranges.** ``--samples < 1``, ``--rate-hz <= 0`` and
  ``--request-window < 1`` are rejected by ``parse_args`` with
  ``SystemExit(2)``, never by falling through to ``EXIT_CONNECT_FAILED == 3``.
  The same applies to a ``--settle-tolerance-deg`` at or above the *effective*
  step (``--delta-deg`` after the ``MAX_STEP_DELTA_DEG`` clamp, compared on the
  **magnitude** so a negative step is judged exactly like its positive twin): a
  tolerance that large judges the joint settled on the first poll, which leaves
  ``rise_90`` empty for every row.  The check is unconditional (rtt mode parses
  the same knobs and ignores them) because the default pair ``1.0`` / ``0.15``
  is safe.

Round 6: command pacing (frozen here as the expectation; section Q).

* **``--control-hz``** (default ``50.0``, must be ``>= 1``) is the *command* rate
  and is independent of ``--rate-hz`` (the observation rate): two consecutive
  ``send_action`` calls on the tracking path are at least ``1 / control_hz``
  seconds apart.  Every command on that path is paced — the first command of a
  step and every settle-loop repeat.  ``parse_args`` rejects a non-positive value
  with ``SystemExit(2)`` and ``--control-hz`` on stderr (the same
  ``parser.error`` shape as ``--rate-hz``), never by falling through to a run.
* **``run_tracking_steps`` takes the same knob as ``control_hz: float = 50.0``**
  and paces through its existing injection point (``sleep=``): a caller that
  injects a sleeper keeps full control, the function never calls
  ``time.sleep`` directly, and ``main`` passes the real implementation.  The
  sleep argument is the *remaining* time to the next allowed command, so one
  pacing pause is at most ``1 / control_hz``; the only longer pause a tracking
  run issues is the pre-existing ``rate_hz`` period between two steps.  A run
  must never sleep the whole settle budget in one call.
* **pacing does not move ``rise_90``.**  The crossing is still booked against
  the arrival stamp of the frame that crossed
  (``latest_request_timing["received_monotonic_s"]``): the command frequency
  limits how often the goal is repeated, not when a frame arrived.  Section M
  is unchanged by this round.
* **the tracking summary reports both achieved rates**: ``achieved_command_hz``
  (commands per second of the run) and ``achieved_rate_hz`` (observations per
  second of the run), each ``nan`` when it cannot be computed, and the printed
  tracking report names both keys.  They are what makes a run readable: a report
  that only states the *requested* rates cannot show that pacing worked.
* **the fake clock of section Q** is a ``_PacingHarness``: time moves only when
  the injected sleeper sleeps (plus an explicit per-read cost), and the fake
  client stamps the harness clock at every ``send_action``.  The rate assertion
  is then arithmetic on measured timestamps: ``count - 1`` gaps of at least
  ``1 / control_hz`` cannot fit into a shorter span.
"""

from __future__ import annotations

import ast
import csv
import importlib.util
import math
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPT_PATH = _REPO_ROOT / "benchmarks" / "alohamini_io" / "measure_latency.py"


def _load_script():
    """Load the benchmark script by path (it is not a package module)."""
    assert _SCRIPT_PATH.is_file(), (
        f"script under test: {_SCRIPT_PATH} (the contract below is frozen; "
        "a missing script fails loudly instead of skipping)"
    )
    spec = importlib.util.spec_from_file_location("bench_measure_latency", _SCRIPT_PATH)
    assert spec is not None and spec.loader is not None, (
        f"cannot build import spec for {_SCRIPT_PATH}"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def latency():
    """The module under test: the script loaded by path, imported once per run."""
    return _load_script()


# ===========================================================================
# Harness: scriptable fake client + deterministic clock (no sockets, no sleeps)
# ===========================================================================


class _FakeClock:
    """Monotonic clock stub: every read advances by ``step_s``."""

    def __init__(self, *, start: float = 0.0, step_s: float = 0.02) -> None:
        self._now = start
        self._step_s = step_s
        self.calls = 0

    def __call__(self) -> float:
        self.calls += 1
        current = self._now
        self._now += self._step_s
        return current


# Sentinel for ``_FakeClient(host_unix_ns=...)``: the key is *absent* from the
# host timing face, so the CSV cell must be empty instead of ``0``.
_MISSING = object()


def _fast_sleep(_seconds: float) -> None:
    """Sleep stub: keeps pacing/timeout logic driven by the injected clock only."""


class _FakeClient:
    """Stand-in for ``AlohaMiniClient`` that never touches a socket.

    Per call ``i`` the exposed obs
    ``observation_sequence`` is ``sequences[i]`` (default ``i + 1``), the
    reported ``age_s`` is ``ages[i]`` (default 0.030) and the host sample
    duration is ``host_durations[i]`` (default 0.004).  ``interrupt_at`` makes
    call ``i >= interrupt_at`` raise ``KeyboardInterrupt`` instead of answering,
    which is how an operator pressing Ctrl-C in the middle of a run looks to the
    script.  ``prime_calls`` records every window prime the script issues (the
    ``include_cameras`` flag per call), so a test can see *whether* and *how* the
    in-flight window was refilled and, crucially, *when*.

    ``empty_timing_from`` / ``empty_timing_until`` script the *later* shapes of
    the second capability defect (see R11-R13): with ``empty_timing_from=N``
    every call ``i >= N`` leaves ``latest_request_timing`` as ``{}`` -- frames
    that were timed, then a host that stops timing them; with
    ``empty_timing_until=N`` every call ``i < N`` leaves it ``{}`` -- an empty
    first frame, timed from frame ``N`` on.  Both default to ``None``, i.e. every
    frame is timed, so no existing test changes shape.
    """

    def __init__(
        self,
        *,
        sequences: list[int] | None = None,
        ages: list[float] | None = None,
        host_durations: list[float] | None = None,
        host_unix_ns: list[Any] | None = None,
        connect_error: Exception | None = None,
        interrupt_at: int | None = None,
        empty_timing_from: int | None = None,
        empty_timing_until: int | None = None,
    ) -> None:
        self._sequences = sequences
        self._ages = ages
        self._host_durations = host_durations
        self._host_unix_ns = host_unix_ns
        self._connect_error = connect_error
        self._interrupt_at = interrupt_at
        self._empty_timing_from = empty_timing_from
        self._empty_timing_until = empty_timing_until
        self._sequence = 0
        self.is_connected = False
        self.get_observation_calls = 0
        self.send_action_calls = 0
        self.prime_calls: list[bool] = []
        self.sent_actions: list[dict] = []
        self.latest_request_timing: dict = {}
        self.latest_host_timing: dict = {}
        self.latest_robot_metadata: dict = {"schema_version": 2, "robot_model": "alohamini2pro"}
        self.latest_control_status: dict | None = None
        self.last_state: dict = {"arm_left_shoulder_pan.pos": 1.0}

    @property
    def observation_sequence(self) -> int:
        return self._sequence

    @staticmethod
    def _value_at(script: list | None, index: int, default):
        """Scripted value, saturating on the last entry (frozen-tail semantics)."""
        if not script:
            return default
        return script[min(index, len(script) - 1)]

    def connect(self) -> None:
        if self._connect_error is not None:
            raise self._connect_error
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def prime_observation_request_window(self, *, include_cameras: bool) -> None:
        """Record the window prime instead of refilling a real request queue.

        The script calls this once per run, after ``connect()`` and before the
        first sample, to replace the camera requests ``connect()`` queued with
        state-only ones.  Only the flag is recorded: the observation counters
        must stay exactly what the scripted run produces.
        """
        self.prime_calls.append(include_cameras)

    def get_observation(self, include_cameras: bool = True) -> dict:
        index = self.get_observation_calls
        if self._interrupt_at is not None and index >= self._interrupt_at:
            raise KeyboardInterrupt(f"operator pressed Ctrl-C before observation {index}")
        self.get_observation_calls += 1
        self._sequence = self._value_at(self._sequences, index, index + 1)
        age_s = self._value_at(self._ages, index, 0.030)
        duration_s = self._value_at(self._host_durations, index, 0.004)
        started_s = 1000.0 + index
        self.latest_request_timing = {
            "token": f"tok-{index}".encode("ascii"),
            "sent_monotonic_s": started_s,
            "received_monotonic_s": started_s + age_s,
            "age_s": age_s,
        }
        if self._empty_timing_from is not None and index >= self._empty_timing_from:
            self.latest_request_timing = {}
        if self._empty_timing_until is not None and index < self._empty_timing_until:
            self.latest_request_timing = {}
        host_timing: dict = {
            "state_sample_started_monotonic_s": started_s,
            "state_sample_finished_monotonic_s": started_s + duration_s,
        }
        unix_ns = (
            0 if self._host_unix_ns is None else self._value_at(self._host_unix_ns, index, None)
        )
        if unix_ns is not _MISSING:
            host_timing["state_sample_unix_ns"] = unix_ns
        self.latest_host_timing = host_timing
        return dict(self.last_state)

    def send_action(self, action: dict) -> dict:
        self.send_action_calls += 1
        self.sent_actions.append(dict(action))
        return dict(action)


class _Factory:
    """Injectable ``client_factory`` that records calls and can fail.

    Contract of this file: the factory is called with four *positional*
    arguments, ``client_factory(host, port, robot_model, request_window)``.  The
    window is recorded separately in ``window_args`` so that ``calls`` keeps the
    three-tuple shape the round-1 assertion froze.
    """

    def __init__(self, client: Any = None, *, raises: Exception | None = None) -> None:
        self.client = client
        self.raises = raises
        self.calls: list[tuple[Any, ...]] = []
        self.window_args: list[Any] = []

    def __call__(self, host: str, port: int, robot_model: str, request_window: Any = None):
        self.calls.append((host, port, robot_model))
        self.window_args.append(request_window)
        if self.raises is not None:
            raise self.raises
        return self.client


# ===========================================================================
# CLI surface
# ===========================================================================


def test_help_should_exit_zero_and_document_the_motion_gate() -> None:
    """``--help`` must succeed and advertise the Gate A switch."""
    result = subprocess.run(
        [sys.executable, str(_SCRIPT_PATH), "--help"],
        capture_output=True,
        text=True,
        check=False,
        cwd=str(_REPO_ROOT),
        timeout=120,
    )

    combined = result.stdout + result.stderr

    assert result.returncode == 0, combined
    assert "--enable-motion" in combined
    assert "Gate" in combined


def test_parse_args_should_apply_contract_defaults(latency) -> None:
    """Defaults: rtt mode, 5556, 30 Hz, 500 samples, 30 s, delta 1.0."""
    args = latency.parse_args(["--host", "127.0.0.1"])

    assert args.host == "127.0.0.1"
    assert args.port == 5556
    assert args.robot_model == "alohamini2pro"
    assert args.mode == "rtt"
    assert args.rate_hz == pytest.approx(30.0)
    assert args.samples == 500
    assert args.timeout_s == pytest.approx(30.0)
    assert args.enable_motion is False
    assert args.joint is None
    assert args.delta_deg == pytest.approx(1.0)
    assert args.steps == 20
    csv_path = str(args.csv)
    assert "artifacts/alohamini-io/rtt-" in csv_path
    assert csv_path.endswith(".csv")


def test_parse_args_should_reject_unknown_mode(latency) -> None:
    """``--mode`` only accepts rtt/tracking."""
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", "--mode", "bogus"])

    assert excinfo.value.code == 2


def test_parse_args_should_default_request_window_to_one(latency) -> None:
    """Round 4: the window defaults to ``1``, not to the client default of 3.

    With the client default the answered request was sent ``window`` client
    calls earlier, so ``age_s`` is ~2 extra client periods of prefetch wait and
    is not a round trip at all.  ``1`` keeps a plain default run explicable;
    the deployment shape (``3``) is now an explicit opt-in.
    """
    args = latency.parse_args(["--host", "127.0.0.1"])

    assert getattr(args, "request_window", None) == 1


def test_module_constants_should_match_contract(latency) -> None:
    """Exit codes, the motion-step cap and the loop-rate bound are frozen."""
    assert latency.EXIT_OK == 0
    assert latency.EXIT_CONNECT_FAILED == 3
    assert latency.EXIT_WRITE_FAILED == 5
    assert latency.EXIT_MOTION_GATE_REFUSED == 6
    assert latency.EXIT_INSUFFICIENT_SAMPLES == 7
    assert latency.MAX_STEP_DELTA_DEG == 1.0
    assert latency.HOST_LOOP_HZ_UPPER_BOUND == 50.0


def test_module_constants_should_include_hold_failed(latency) -> None:
    """A rejected hold is its own failure, not a step-response abort."""
    assert getattr(latency, "EXIT_HOLD_FAILED", None) == 8


# ===========================================================================
# Statistics helpers
# ===========================================================================


def test_percentile_should_be_nan_for_empty_sequence(latency) -> None:
    """Same contract as ``benchmarks/alohamini_ik/benchmark_arm_ik.py``."""
    assert math.isnan(latency.percentile([], 95.0))


def test_percentile_should_interpolate_linearly(latency) -> None:
    """Nearest-rank percentiles (5.0 / 5.0) must not pass here."""
    values = [1.0, 2.0, 3.0, 4.0, 5.0]

    assert latency.percentile(values, 50.0) == pytest.approx(3.0)
    assert latency.percentile(values, 95.0) == pytest.approx(4.8)
    assert latency.percentile(values, 99.0) == pytest.approx(4.96)


def test_percentile_should_return_the_only_value_for_a_single_sample(latency) -> None:
    """A one-element sequence is its own every percentile."""
    assert latency.percentile([7.5], 99.0) == pytest.approx(7.5)


def test_host_sample_duration_should_differ_the_two_host_stamps(latency) -> None:
    """Only the host-internal delta is used: absolute stamps cancel out."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.004,
    }

    assert latency.host_sample_duration_s(host_timing) == pytest.approx(0.004)


@pytest.mark.parametrize(
    "host_timing",
    [
        {},
        {"state_sample_started_monotonic_s": 1000.0},
        {"state_sample_finished_monotonic_s": 1000.004},
    ],
    ids=["empty", "missing_finished", "missing_started"],
)
def test_host_sample_duration_should_be_none_when_stamps_are_missing(latency, host_timing) -> None:
    """Incomplete host timing cannot be subtracted, so it reports ``None``."""
    assert latency.host_sample_duration_s(host_timing) is None


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_host_sample_duration_should_be_none_for_non_finite_stamps(latency, bad_value) -> None:
    """A non-finite stamp must never leak into the RTT estimate."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": bad_value,
    }

    assert latency.host_sample_duration_s(host_timing) is None


def test_host_sample_duration_should_be_none_when_finished_precedes_started(latency) -> None:
    """Ruling of this file: a negative duration is reported as ``None``.

    Rationale: a finished-before-started stamp pair means the host timing face
    is unusable (clock skew, stale dict, swapped keys). Fail-safe ``None``
    makes the caller fall back to not subtracting anything, instead of adding
    a bogus negative correction that would inflate the net RTT.
    """
    host_timing = {
        "state_sample_started_monotonic_s": 1000.004,
        "state_sample_finished_monotonic_s": 1000.0,
    }

    assert latency.host_sample_duration_s(host_timing) is None


def test_net_rtt_estimate_should_subtract_host_duration_and_phase_mean(latency) -> None:
    """50 Hz upper bound → 0.02 s phase window, mean 0.01 s subtracted."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.004,
    }

    estimate = latency.net_rtt_estimate_s(0.035, host_timing)

    assert estimate["phase_bound_s"] == pytest.approx(0.02)
    assert estimate["host_timing_missing"] is False
    assert estimate["net_rtt_est_s"] == pytest.approx(0.035 - 0.004 - 0.01)


def test_net_rtt_estimate_should_not_subtract_when_host_timing_is_missing(latency) -> None:
    """Missing host timing keeps the raw age — and says so."""
    estimate = latency.net_rtt_estimate_s(0.035, {})

    assert estimate["host_timing_missing"] is True
    assert estimate["net_rtt_est_s"] == pytest.approx(0.035)


def test_net_rtt_estimate_should_clamp_at_zero(latency) -> None:
    """A tiny age cannot be pushed negative by the corrections."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.004,
    }

    estimate = latency.net_rtt_estimate_s(0.005, host_timing)

    assert estimate["net_rtt_est_s"] == pytest.approx(0.0)
    assert estimate["net_rtt_est_s"] >= 0.0


# The round-2 keywords are the contract; a helper keeps a signature drift a
# readable assertion failure instead of a bare ``TypeError``.
def _estimate(latency, age_s, host_timing, **kwargs):
    """Call ``net_rtt_estimate_s`` with the round-2 keyword arguments."""
    try:
        return latency.net_rtt_estimate_s(age_s, host_timing, **kwargs)
    except TypeError as exc:
        pytest.fail(f"net_rtt_estimate_s does not accept the round-2 keywords: {exc}")


def test_net_rtt_estimate_should_report_every_bound_key(latency) -> None:
    """The queue terms and the two error bounds are part of the return shape."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.004,
    }

    estimate = _estimate(latency, 0.05, host_timing, request_window=2)

    required = {
        "age_s",
        "host_sample_duration_s",
        "phase_bound_s",
        "phase_mean_s",
        "queue_bound_s",
        "queue_mean_s",
        "host_timing_missing",
        "net_rtt_est_s",
        "net_rtt_low_s",
        "net_rtt_high_s",
    }
    assert required <= set(estimate)
    # One extra request is queued ahead of the answered one: (2 - 1) / 50 Hz.
    assert estimate["queue_bound_s"] == pytest.approx(0.02)
    assert estimate["queue_mean_s"] == pytest.approx(0.01)


def test_net_rtt_estimate_should_subtract_the_queued_requests(latency) -> None:
    """``request_window=3`` queues two host periods: 40 ms bound, 20 ms mean.

    The link this models: the client keeps three requests in flight,
    ``_poll_and_get_latest_message`` refills the window after consuming one
    frame and the host answers one request per loop turn, so besides the size
    of the answered request the age carries ``window - 1`` whole host periods
    of queueing.  Round 1 subtracted the phase mean only.
    """
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.004,
    }

    estimate = _estimate(latency, 0.035, host_timing, request_window=3)

    assert estimate["queue_bound_s"] == pytest.approx(0.04)
    assert estimate["queue_mean_s"] == pytest.approx(0.02)
    assert estimate["net_rtt_est_s"] == pytest.approx(0.035 - 0.004 - 0.01 - 0.02)
    assert estimate["net_rtt_high_s"] == pytest.approx(0.035 - 0.004)
    assert estimate["net_rtt_low_s"] == pytest.approx(0.0)
    assert estimate["net_rtt_low_s"] <= estimate["net_rtt_est_s"] <= estimate["net_rtt_high_s"]


@pytest.mark.parametrize("request_window", [1, 0])
def test_net_rtt_estimate_should_never_report_a_negative_queue_bound(
    latency, request_window
) -> None:
    """A window of one (or a nonsense zero) queues nothing: the term clamps at 0."""
    estimate = _estimate(latency, 0.035, {}, request_window=request_window)

    assert estimate["queue_bound_s"] == pytest.approx(0.0)
    assert estimate["queue_mean_s"] == pytest.approx(0.0)
    assert estimate["queue_bound_s"] >= 0.0


def test_net_rtt_estimate_should_scale_the_queue_with_the_host_loop_rate(latency) -> None:
    """The injected loop bound drives both the phase and the queue window."""
    estimate = _estimate(latency, 0.2, {}, host_loop_hz=20.0, request_window=3)

    assert estimate["phase_bound_s"] == pytest.approx(0.05)
    assert estimate["phase_mean_s"] == pytest.approx(0.025)
    assert estimate["queue_bound_s"] == pytest.approx(0.1)
    assert estimate["queue_mean_s"] == pytest.approx(0.05)


def test_summary_stats_should_report_the_five_requested_quantiles(latency) -> None:
    """count/min/p50/p95/p99/max over ms samples, linear interpolation."""
    stats = latency.summary_stats([10.0, 20.0, 30.0, 40.0, 50.0])

    assert stats["count"] == 5
    assert stats["min_ms"] == pytest.approx(10.0)
    assert stats["p50_ms"] == pytest.approx(30.0)
    assert stats["p95_ms"] == pytest.approx(48.0)
    assert stats["p99_ms"] == pytest.approx(49.6)
    assert stats["max_ms"] == pytest.approx(50.0)


def test_summary_stats_should_be_nan_for_empty_sequence(latency) -> None:
    """No samples: count 0 and every statistic ``nan``, never an exception."""
    stats = latency.summary_stats([])

    assert stats["count"] == 0
    for key in ("min_ms", "p50_ms", "p95_ms", "p99_ms", "max_ms"):
        assert math.isnan(stats[key])


# ===========================================================================
# CSV writing
# ===========================================================================


def test_csv_fields_should_be_tuples_of_strings_with_a_token_column(latency) -> None:
    """Both field tuples are frozen tuples and carry the request token."""
    for fields in (latency.RTT_CSV_FIELDS, latency.TRACKING_CSV_FIELDS):
        assert isinstance(fields, tuple)
        assert all(isinstance(field, str) for field in fields)
        assert len(fields) == len(set(fields))
    assert "token" in latency.RTT_CSV_FIELDS


def test_write_csv_should_emit_header_and_one_line_per_row(latency, tmp_path) -> None:
    """Header matches ``fields``; every row parses to the same column count."""
    fields = latency.RTT_CSV_FIELDS
    rows = [{field: f"{field}-{index}" for field in fields} for index in range(3)]
    out = tmp_path / "nested" / "deeper" / "rtt.csv"

    latency.write_csv(out, rows, fields)

    assert out.is_file()
    with out.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        assert tuple(reader.fieldnames or ()) == tuple(fields)
        parsed = list(reader)
    assert len(parsed) == 3
    for index, parsed_row in enumerate(parsed):
        assert len(parsed_row) == len(fields)
        assert parsed_row[fields[0]] == f"{fields[0]}-{index}"


def test_write_csv_should_raise_key_error_when_a_row_lacks_a_field(latency, tmp_path) -> None:
    """A short row is a programming error, not a shifted CSV line."""
    fields = latency.RTT_CSV_FIELDS
    rows = [dict.fromkeys(fields[1:], 1.0)]

    with pytest.raises(KeyError):
        latency.write_csv(tmp_path / "rtt.csv", rows, fields)


# ===========================================================================
# RTT sampling
# ===========================================================================


def test_collect_rtt_samples_should_emit_one_row_per_requested_sample(latency) -> None:
    """Rows carry a text token, the age and the clipped net estimate."""
    client = _FakeClient(ages=[0.030, 0.031, 0.032])

    rows = latency.collect_rtt_samples(
        client,
        samples=3,
        rate_hz=200.0,
        timeout_s=5.0,
        sleep=_fast_sleep,
        clock=_FakeClock(start=0.0, step_s=0.001),
    )

    assert len(rows) == 3
    assert [row["age_s"] for row in rows] == pytest.approx([0.030, 0.031, 0.032])
    for index, row in enumerate(rows):
        assert isinstance(row["token"], str)
        assert row["token"] == f"tok-{index}"
        for field in latency.RTT_CSV_FIELDS:
            assert field in row


def test_collect_rtt_samples_should_skip_frames_without_sequence_growth(latency) -> None:
    """Only growth counts: 4 distinct sequences arrived via 5 observations."""
    client = _FakeClient(sequences=[1, 2, 2, 3, 4, 5])

    rows = latency.collect_rtt_samples(
        client,
        samples=4,
        rate_hz=200.0,
        timeout_s=5.0,
        sleep=_fast_sleep,
        clock=_FakeClock(start=0.0, step_s=0.001),
    )

    assert len(rows) == 4
    assert client.get_observation_calls == 5


def test_collect_rtt_samples_should_raise_when_sequence_never_grows(latency) -> None:
    """A frozen sequence ends in ``InsufficientSamples``, not a short list."""
    client = _FakeClient(sequences=[1])

    with pytest.raises(latency.InsufficientSamples) as excinfo:
        latency.collect_rtt_samples(
            client,
            samples=5,
            rate_hz=50.0,
            timeout_s=1.0,
            sleep=_fast_sleep,
            clock=_FakeClock(start=0.0, step_s=1.0),
        )

    assert excinfo.value.required == 5
    assert excinfo.value.collected < 5


class _ManualClock:
    """Deterministic clock: only explicit ``advance`` (or a sleep) moves it."""

    def __init__(self, start: float = 0.0) -> None:
        self.now = start
        self.reads = 0

    def __call__(self) -> float:
        self.reads += 1
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class _PacingClient:
    """Fake client whose blocking observation consumes clock time.

    ``get_observation`` advances the injected clock by ``cost_s``, which is what
    a real blocking ``recv`` does.  Pacing must therefore be computed against an
    absolute deadline: a ``sleep(period)`` *after* the read adds the read cost
    on top of the period and silently halves the achieved rate.
    """

    def __init__(self, clock: _ManualClock, *, cost_s: float = 0.02, age_s: float = 0.030) -> None:
        self._clock = clock
        self._cost_s = cost_s
        self._age_s = age_s
        self._sequence = 0
        self.get_observation_calls = 0
        self.latest_request_timing: dict = {}
        self.latest_host_timing: dict = {}

    @property
    def observation_sequence(self) -> int:
        return self._sequence

    def get_observation(self, include_cameras: bool = True) -> dict:
        self._clock.advance(self._cost_s)
        index = self.get_observation_calls
        self.get_observation_calls += 1
        self._sequence += 1
        sent_s = 1000.0 + index
        self.latest_request_timing = {
            "token": f"tok-{index}".encode("ascii"),
            "sent_monotonic_s": sent_s,
            "received_monotonic_s": sent_s + self._age_s,
            "age_s": self._age_s,
        }
        self.latest_host_timing = {
            "state_sample_started_monotonic_s": 900000.0 + index,
            "state_sample_finished_monotonic_s": 900000.0 + index + 0.004,
            "state_sample_unix_ns": 1_700_000_000_000_000_000 + index,
        }
        return {"arm_left_shoulder_pan.pos": 1.0}


def test_collect_rtt_samples_should_pace_against_the_absolute_rate_deadline(latency) -> None:
    """Ten samples at 30 Hz cost ~10 periods, not 10 x (read + period).

    With a 20 ms blocking read the round-1 loop cost 10 x (0.02 + 0.0333) =
    0.533 s, i.e. an achieved rate of ~18.8 Hz instead of the requested 30 Hz.
    Absolute deadlines give 10 x 0.0333 = 0.333 s.
    """
    clock = _ManualClock()
    client = _PacingClient(clock, cost_s=0.02)
    period_s = 1.0 / 30.0

    rows = latency.collect_rtt_samples(
        client,
        samples=10,
        rate_hz=30.0,
        timeout_s=60.0,
        sleep=clock.advance,
        clock=clock,
    )
    elapsed_s = clock.now

    assert len(rows) == 10
    # One period of slack for the final partial tick.
    assert elapsed_s <= 10 * period_s + period_s
    # And the rate must not be exceeded either: sample n lands at n periods, so
    # the ninth gap alone is already 9 periods of pacing (a loop that never
    # sleeps would finish in 10 x 0.02 = 0.2 s).
    assert elapsed_s >= 9 * period_s


def test_main_should_report_the_achieved_rtt_rate(latency, tmp_path, capsys) -> None:
    """The report states the rate the run actually achieved, not only the target."""
    client = _FakeClient(ages=[0.030, 0.031, 0.032])
    out = tmp_path / "rtt.csv"

    code = latency.main(
        ["--host", "127.0.0.1", "--samples", "3", "--rate-hz", "200", "--csv", str(out)],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()
    lowered = captured.out.lower()

    assert code == latency.EXIT_OK
    assert "achieved" in lowered
    assert "hz" in lowered


def test_rtt_csv_fields_should_carry_the_host_clock_baseline(latency) -> None:
    """``state_sample_unix_ns`` lets a later analysis re-derive host wall time."""
    assert "state_sample_unix_ns" in latency.RTT_CSV_FIELDS


def test_collect_rtt_samples_should_record_the_host_clock_baseline(latency) -> None:
    """The host ``unix_ns`` stamp lands in the row, one value per sample."""
    stamps = [1_700_000_000_000_000_000, 1_700_000_000_000_000_500]
    client = _FakeClient(ages=[0.030, 0.031], host_unix_ns=stamps)

    rows = latency.collect_rtt_samples(
        client,
        samples=2,
        rate_hz=200.0,
        timeout_s=5.0,
        sleep=_fast_sleep,
        clock=_FakeClock(start=0.0, step_s=0.001),
    )

    recorded = [row.get("state_sample_unix_ns") for row in rows]
    assert all(value is not None for value in recorded), (
        f"rows do not carry the host clock baseline: {sorted(rows[0])}"
    )
    assert [int(value) for value in recorded] == stamps


def test_collect_rtt_samples_should_leave_the_host_clock_baseline_empty_when_absent(
    latency,
) -> None:
    """No host ``unix_ns`` stamp means an empty cell, never a silently wrong 0."""
    client = _FakeClient(ages=[0.030], host_unix_ns=[_MISSING])

    rows = latency.collect_rtt_samples(
        client,
        samples=1,
        rate_hz=200.0,
        timeout_s=5.0,
        sleep=_fast_sleep,
        clock=_FakeClock(start=0.0, step_s=0.001),
    )

    recorded = rows[0].get("state_sample_unix_ns")
    assert recorded == "", f"expected an empty cell, got {recorded!r}"


# ===========================================================================
# Motion gate (must be decided before connect)
# ===========================================================================


def test_main_should_refuse_tracking_without_enable_motion_before_connecting(
    latency, tmp_path, capsys
) -> None:
    """Gate A: tracking without ``--enable-motion`` never touches the factory."""
    factory = _Factory(_FakeClient())

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--joint",
            "arm_left_shoulder_pan",
            # Explicit target so a broken gate cannot write into the repo.
            "--csv",
            str(tmp_path / "refused.csv"),
        ],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert factory.calls == []
    assert "enable_motion" in captured.err
    assert "Gate A" in captured.err
    assert "--enable-motion" in captured.err


def test_main_should_refuse_tracking_without_joint_before_connecting(
    latency, tmp_path, capsys
) -> None:
    """Gate A also requires an explicit joint name."""
    factory = _Factory(_FakeClient())

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--csv",
            str(tmp_path / "refused.csv"),
        ],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert factory.calls == []
    assert "Gate A" in captured.err


def test_resolve_delta_deg_should_clamp_to_max_step_delta(latency) -> None:
    """Contract addition of this file: the clamp is a pure, testable helper."""
    requested = latency.parse_args(["--host", "127.0.0.1", "--delta-deg", "3.0"])

    assert latency.resolve_delta_deg(requested) == pytest.approx(latency.MAX_STEP_DELTA_DEG)


def test_resolve_delta_deg_should_keep_values_within_the_cap(latency) -> None:
    """A legal ``--delta-deg`` is passed through untouched."""
    requested = latency.parse_args(["--host", "127.0.0.1", "--delta-deg", "0.25"])

    assert latency.resolve_delta_deg(requested) == pytest.approx(0.25)


# ===========================================================================
# Tracking commands: full-shape commands, echo recording, hold in a finally
# ===========================================================================

_TRACKING_JOINT = "arm_left_shoulder_pan.pos"
_TRACKING_BASELINE_DEG = 1.0
# The lift column is the one command key that is pinned from the feedback without
# ending in ``.pos``: the host publishes the measured height in millimetres
# (``lift_axis.height_mm``, *not* ``lift_axis.height.pos``).
_LIFT_HEIGHT_KEY = "lift_axis.height_mm"
# A real feedback frame: arm joints and grippers report ``.pos`` values, the base
# reports three velocities and the lift column reports a height.  The frame is
# deliberately a *subset* of the 14 arm joints — Gate B below needs a joint that
# is genuinely absent from the schema (``arm_left_wrist_yaw``).  The gripper
# keys are the ``range_0_100`` motors of section O.
_TRACKING_STATE = {
    "arm_left_shoulder_pan.pos": _TRACKING_BASELINE_DEG,
    "arm_right_shoulder_pan.pos": -2.5,
    "arm_left_gripper.pos": 0.25,
    "arm_right_gripper.pos": -0.3,
    # Non-zero on purpose: a command that echoes the measured velocity instead of
    # pinning the base to a stop must fail the ``0.0`` assertions.
    "x.vel": 0.012,
    "y.vel": -0.008,
    "theta.vel": 0.001,
    _LIFT_HEIGHT_KEY: 10.0,
}
# Only ``.pos`` values are pose goals: ``x/y/theta.vel`` are velocities and
# ``lift_axis.height_mm`` is a height, so neither belongs to the pose-goal set.
_TRACKING_POS_KEYS = tuple(key for key in _TRACKING_STATE if key.endswith(".pos"))
# The same frame without the lift column, i.e. what a robot without a lift axis
# reports (section L2: an absent key must stay absent, never be invented).
_TRACKING_STATE_WITHOUT_LIFT = {
    key: value for key, value in _TRACKING_STATE.items() if key != _LIFT_HEIGHT_KEY
}
# Host metadata for the stepped joint.  ``use_degrees`` is off on the Host, so
# this calibration is what converts a normalized ``.pos`` unit into degrees:
# (range_max - range_min) / 200 * 360 / (resolution - 1) = 0.8791208791 deg/unit.
_TRACKING_MOTOR = "arm_left_shoulder_pan"
_TRACKING_SCALE_DEG = 0.8791208791208791
_TRACKING_MOTOR_ENTRY = {"range_min": 1000, "range_max": 3000, "resolution": 4096}
_TRACKING_METADATA = {
    "schema_version": 2,
    "robot_model": "alohamini2pro",
    "motors": {_TRACKING_MOTOR: dict(_TRACKING_MOTOR_ENTRY)},
}
_BASE_VEL_KEYS = ("x.vel", "y.vel", "theta.vel")
# Every command ``run_tracking_steps`` sends is a *full* command: all measured
# pose goals, the three base velocities pinned to ``0.0`` and — only when the
# feedback carries it — the measured lift height (``_TRACKING_STATE`` does).
_TRACKING_FULL_COMMAND_KEYS = (*_TRACKING_POS_KEYS, *_BASE_VEL_KEYS, _LIFT_HEIGHT_KEY)
# The fake client echoes every ``.pos`` goal back 0.01 deg lower, so a record
# that stores the *request* cannot be mistaken for one that stores the *echo*.
_ECHO_OFFSET_DEG = -0.01


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _is_hold_shape(action: dict) -> bool:
    """A hold carries every measured ``.pos`` key and pins the base to zero."""
    return all(key in action for key in _TRACKING_POS_KEYS) and all(
        action.get(key) == 0.0 for key in _BASE_VEL_KEYS
    )


class _TrackingFake:
    """Multi-joint tracking stand-in: fixes the feedback, echoes goals shifted.

    ``observation_sequence`` grows by one per ``get_observation`` call, which is
    what a live link does and what the round-3 stall detector watches.
    ``sequence_freezes_after`` freezes it once ``n`` calls have happened (``0``
    = it never grows at all), which models the transport-timeout case where the
    client keeps serving the cached frame without raising.
    """

    def __init__(
        self,
        state: dict | None = None,
        *,
        echo_offset_deg: float = _ECHO_OFFSET_DEG,
        fail_send_at: int | None = None,
        fail_send_when_hold: bool = False,
        fail_send_error: BaseException | None = None,
        sequence_freezes_after: int | None = None,
    ) -> None:
        self.state = dict(_TRACKING_STATE if state is None else state)
        self._echo_offset_deg = echo_offset_deg
        self._fail_send_at = fail_send_at
        self._fail_send_when_hold = fail_send_when_hold
        self._fail_send_error = fail_send_error
        self._sequence_freezes_after = sequence_freezes_after
        self.sent_actions: list[dict] = []
        self.get_observation_calls = 0
        self.is_connected = False
        self.observation_sequence = 0
        self.latest_robot_metadata: dict = {
            "schema_version": 2,
            "robot_model": "alohamini2pro",
            "motors": {key: dict(value) for key, value in _TRACKING_METADATA["motors"].items()},
        }
        self.latest_control_status: dict | None = None
        self.latest_request_timing: dict = {}
        self.latest_host_timing: dict = {}

    def connect(self) -> None:
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def get_observation(self, include_cameras: bool = True) -> dict:
        index = self.get_observation_calls
        self.get_observation_calls += 1
        if self._sequence_freezes_after is None or index < self._sequence_freezes_after:
            self.observation_sequence += 1
        return dict(self.state)

    def send_action(self, action: dict) -> dict:
        self.sent_actions.append({str(key): value for key, value in action.items()})
        index = len(self.sent_actions)
        error = self._fail_send_error or RuntimeError("command rejected by host")
        if self._fail_send_when_hold and _is_hold_shape(action):
            raise error
        if self._fail_send_at is not None and index == self._fail_send_at:
            raise error
        return {
            key: (value + self._echo_offset_deg if key.endswith(".pos") else value)
            for key, value in action.items()
        }


def _run_steps(latency, client, **overrides) -> dict:
    """Run ``run_tracking_steps`` with a deterministic clock and a sleep stub."""
    kwargs = {
        "joint_key": _TRACKING_JOINT,
        "delta_deg": 1.0,
        "steps": 2,
        "rate_hz": 30.0,
        "settle_timeout_s": 0.2,
        "settle_tolerance_deg": 0.15,
        "timeout_s": 30.0,
        "sleep": _fast_sleep,
        "clock": _FakeClock(start=0.0, step_s=0.05),
    }
    kwargs.update(overrides)
    return latency.run_tracking_steps(client, **kwargs)


class _ScriptedTrackingFake:
    """Tracking stand-in with a scripted feedback value (and lift / arrival stamps).

    The per-poll value of :data:`_TRACKING_JOINT` is ``values[i]`` (saturating on
    the last entry), the per-poll ``lift_axis.height_mm`` is ``lift[i]`` and the
    per-poll arrival stamp is ``received[i]`` — the latter two are only attached
    when a script is given, which is what distinguishes "the feedback carries the
    key" from "the command invented it".  Every observation grows
    ``observation_sequence``, so the round-3 stall detector stays quiet.
    """

    def __init__(
        self,
        values: list[float],
        *,
        lift: list[float] | None = None,
        received: list[float] | None = None,
    ) -> None:
        self._values = list(values)
        self._lift = None if lift is None else list(lift)
        self._received = None if received is None else list(received)
        self.sent_actions: list[dict] = []
        self.observed_lift_heights: list[float] = []
        # The lift height that was the *latest measurement* when each command was
        # sent, i.e. the value a command that pins the axis must carry.  Recorded
        # at ``send_action`` time so the comparison is against the frame that had
        # actually arrived, not against a value computed after the run.
        self.lift_at_send: list[float | None] = []
        self.get_observation_calls = 0
        self.observation_sequence = 0
        self.is_connected = False
        self.latest_robot_metadata: dict = {
            "schema_version": 2,
            "robot_model": "alohamini2pro",
            "motors": {_TRACKING_MOTOR: dict(_TRACKING_MOTOR_ENTRY)},
        }
        self.latest_control_status: dict | None = None
        self.latest_request_timing: dict = {}
        self.latest_host_timing: dict = {}

    @staticmethod
    def _value_at(script: list[float], index: int) -> float:
        return script[min(index, len(script) - 1)]

    def connect(self) -> None:
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def get_observation(self, include_cameras: bool = True) -> dict:
        index = self.get_observation_calls
        self.get_observation_calls += 1
        self.observation_sequence += 1
        state: dict[str, float] = {_TRACKING_JOINT: self._value_at(self._values, index)}
        if self._lift is not None:
            height = self._value_at(self._lift, index)
            state["lift_axis.height_mm"] = height
            self.observed_lift_heights.append(height)
        if self._received is not None:
            self.latest_request_timing = {
                "received_monotonic_s": self._value_at(self._received, index),
                "sent_monotonic_s": self._value_at(self._received, index) - 0.03,
                "age_s": 0.03,
            }
        return state

    def send_action(self, action: dict) -> dict:
        self.sent_actions.append({str(key): value for key, value in action.items()})
        self.lift_at_send.append(
            self.observed_lift_heights[-1] if self.observed_lift_heights else None
        )
        return dict(action)


def test_run_tracking_steps_should_send_complete_commands(latency) -> None:
    """Every command carries the pose goals, the zero base velocities and the lift height.

    ``AlohaMiniClient.send_action`` builds the whole packet from the dict it is
    given, so a single-key dict silently drops the other joints and the base.
    """
    client = _TrackingFake()

    _run_steps(latency, client)

    assert client.sent_actions
    for action in client.sent_actions:
        missing = [key for key in _TRACKING_FULL_COMMAND_KEYS if key not in action]
        assert not missing, f"command dropped {missing}: {sorted(action)}"
        for key in _BASE_VEL_KEYS:
            assert action[key] == 0.0, f"{key} must be pinned to 0.0, got {action[key]!r}"


def test_run_tracking_steps_should_command_the_baseline_plus_the_delta(latency) -> None:
    """Step commands ask for ``baseline + delta``; only the hold freezes at the pose."""
    client = _TrackingFake()

    _run_steps(latency, client)

    target_deg = _TRACKING_BASELINE_DEG + 1.0
    request_deg = [action.get(_TRACKING_JOINT) for action in client.sent_actions]
    step_commands = [value for value in request_deg if value == pytest.approx(target_deg)]
    hold_commands = [
        value for value in request_deg if value == pytest.approx(_TRACKING_BASELINE_DEG)
    ]

    assert len(step_commands) >= 2, f"expected one command per requested step, got {request_deg}"
    assert hold_commands, "the run must finish by holding the measured position"
    assert len(step_commands) + len(hold_commands) == len(request_deg), (
        f"commands asked for neither baseline + delta nor the held pose: {request_deg}"
    )


def test_run_tracking_steps_should_record_the_requested_and_echoed_command(latency) -> None:
    """Rows keep ``cmd_deg`` (the goal) and ``cmd_echo_deg`` (what came back)."""
    client = _TrackingFake()

    summary = _run_steps(latency, client)

    rows = summary["rows"]
    assert rows
    for row in rows:
        assert row["cmd_deg"] == pytest.approx(_TRACKING_BASELINE_DEG + 1.0)
        echoed = row.get("cmd_echo_deg")
        assert echoed is not None, f"row does not record the echoed command: {sorted(row)}"
        assert echoed == pytest.approx(_TRACKING_BASELINE_DEG + 1.0 + _ECHO_OFFSET_DEG)
        assert echoed != pytest.approx(row["cmd_deg"])


def test_run_tracking_steps_should_finish_with_a_full_hold(latency) -> None:
    """The last command is a hold on every measured joint, not a partly-built one."""
    client = _TrackingFake()

    _run_steps(latency, client)

    last = client.sent_actions[-1]
    assert _is_hold_shape(last), f"last command is not a hold: {sorted(last)}"
    assert last[_TRACKING_JOINT] == pytest.approx(_TRACKING_BASELINE_DEG)
    for key in _TRACKING_POS_KEYS:
        assert _is_finite_number(last[key])


def test_run_tracking_steps_should_still_send_the_hold_when_a_command_is_rejected(latency) -> None:
    """A rejected command aborts the probe but must never skip the stop command."""
    client = _TrackingFake(fail_send_at=1, fail_send_error=RuntimeError("rejected by host"))

    summary = _run_steps(latency, client)

    assert summary["abort_reason"], "a rejected command must be reported as an abort"
    assert client.sent_actions
    assert _is_hold_shape(client.sent_actions[-1]), (
        f"the hold was not sent after the rejection: {client.sent_actions[-1]}"
    )


def test_run_tracking_steps_should_send_the_hold_before_a_keyboard_interrupt_escapes(
    latency,
) -> None:
    """Ctrl-C mid-probe must still freeze the robot: the hold lives in a ``finally``."""
    client = _TrackingFake(fail_send_at=2, fail_send_error=KeyboardInterrupt())

    with pytest.raises(KeyboardInterrupt):
        _run_steps(latency, client)

    assert client.sent_actions
    assert _is_hold_shape(client.sent_actions[-1]), (
        f"no hold was sent before the interrupt escaped: {client.sent_actions[-1]}"
    )


def test_main_should_report_hold_failed_when_the_hold_is_rejected(
    latency, tmp_path, capsys
) -> None:
    """A failed hold is its own exit code: the robot is still moving."""
    client = _TrackingFake(fail_send_when_hold=True, fail_send_error=RuntimeError("hold dropped"))
    factory = _Factory(client)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--settle-timeout-s",
            "0.01",
            "--csv",
            str(out),
        ],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    expected = getattr(latency, "EXIT_HOLD_FAILED", None)
    assert expected == 8, "EXIT_HOLD_FAILED must exist and be 8"
    assert code == expected
    assert code != latency.EXIT_STEP_RESPONSE_ABORTED
    assert "hold" in captured.err.lower()


def test_main_should_refuse_tracking_when_the_joint_is_not_in_the_state_schema(
    latency, tmp_path, capsys
) -> None:
    """Gate B: an unknown joint is refused before anything is commanded."""
    client = _TrackingFake()
    factory = _Factory(client)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_wrist_yaw",
            "--steps",
            "1",
            "--csv",
            str(out),
        ],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert client.sent_actions == []
    assert "schema" in captured.err.lower()
    assert not out.exists()


def test_main_should_refuse_tracking_when_the_host_reports_another_robot_model(
    latency, tmp_path, capsys
) -> None:
    """Gate C: a model mismatch means the joint names/units are not trustworthy."""
    client = _TrackingFake()
    client.latest_robot_metadata = {"schema_version": 2, "robot_model": "alohamini1"}
    factory = _Factory(client)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--csv",
            str(out),
        ],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert client.sent_actions == []
    assert "robot_model" in captured.err.lower()
    assert not out.exists()


# ===========================================================================
# main(): rtt mode end to end (offline) + zero-motion invariants
# ===========================================================================


def test_main_should_write_rtt_csv_without_sending_actions(latency, tmp_path, capsys) -> None:
    """rtt run: exit 0, 5 CSV rows, zero motion, honest RTT wording."""
    client = _FakeClient(ages=[0.030, 0.031, 0.032, 0.033, 0.034])
    factory = _Factory(client)
    out = tmp_path / "artifacts" / "rtt.csv"

    code = latency.main(
        ["--host", "127.0.0.1", "--samples", "5", "--rate-hz", "200", "--csv", str(out)],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_OK == 0
    assert out.is_file()
    with out.open(newline="", encoding="utf-8") as handle:
        parsed = list(csv.DictReader(handle))
    assert len(parsed) == 5
    assert client.send_action_calls == 0
    for marker in ("p50", "p95", "p99"):
        assert marker in captured.out
    assert "estimate" in captured.out.lower()
    assert factory.calls == [("127.0.0.1", 5556, "alohamini2pro")]


def test_main_should_forward_the_request_window_to_the_client_factory(latency, tmp_path) -> None:
    """``--request-window`` reaches the client config, otherwise the queue term is a guess."""
    client = _FakeClient(ages=[0.030, 0.031, 0.032])
    factory = _Factory(client)
    out = tmp_path / "rtt.csv"

    try:
        code = latency.main(
            [
                "--host",
                "127.0.0.1",
                "--samples",
                "3",
                "--rate-hz",
                "200",
                "--request-window",
                "2",
                "--csv",
                str(out),
            ],
            client_factory=factory,
        )
    except SystemExit as exc:  # argparse rejected an unknown flag
        pytest.fail(f"main() rejected the round-2 CLI (--request-window): {exc!r}")

    assert code == latency.EXIT_OK
    assert factory.window_args == [2]


def test_main_should_report_insufficient_samples_in_rtt_mode(latency, tmp_path) -> None:
    """A frozen sequence in rtt mode maps to EXIT_INSUFFICIENT_SAMPLES."""
    client = _FakeClient(sequences=[1])
    out = tmp_path / "rtt.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--samples",
            "5",
            "--rate-hz",
            "100",
            "--timeout-s",
            "0.2",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_INSUFFICIENT_SAMPLES
    assert not out.exists()


def test_main_should_report_connect_failed_when_factory_raises(latency, tmp_path) -> None:
    """A raising factory maps to EXIT_CONNECT_FAILED, not a traceback."""
    code = latency.main(
        ["--host", "127.0.0.1", "--csv", str(tmp_path / "rtt.csv")],
        client_factory=_Factory(raises=RuntimeError("no host")),
    )

    assert code == latency.EXIT_CONNECT_FAILED


def test_send_action_should_only_appear_inside_tracking_functions(latency) -> None:
    """Gate B in source form: every *code* reference sits in a tracking function.

    Uses ``ast`` positions rather than text-line guessing: each reference line
    must fall inside ``lineno .. end_lineno`` of a ``def`` whose name contains
    ``tracking``.  String literals (docstrings, comments) may not mention the
    name either, so the module cannot advertise motion it does not perform.
    """
    tree = ast.parse(Path(latency.__file__).read_text(encoding="utf-8"))
    tracking_functions = [
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and "tracking" in node.name.lower()
    ]
    references = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute) and node.attr == "send_action"
    ]
    mentioned_in_text = [
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and "send_action" in node.value
    ]

    assert tracking_functions, "expected a tracking-mode function in the module"
    assert references, "tracking mode must drive the robot through send_action"
    assert not mentioned_in_text, "send_action belongs in code, not in a docstring or message"
    for node in references:
        inside = [
            function.name
            for function in tracking_functions
            if function.lineno <= node.lineno <= (function.end_lineno or function.lineno)
        ]
        assert inside, f"send_action at line {node.lineno} is outside every tracking function"


# ===========================================================================
# E. Feedback liveness: a frozen ``observation_sequence`` is a dead link
# ===========================================================================
#
# ``AlohaMiniClient._get_data`` returns the *cached* frame and raises nothing
# when the observation transport times out, so a probe that only watches for
# exceptions keeps measuring a corpse.  The only liveness signal is
# ``client.observation_sequence``, bumped on every successful receive.  Round-3
# contract (see the module docstring of this file): ``FEEDBACK_STALL_LIMIT``
# consecutive polls without growth abort the run with a reason that names the
# stale feedback, break out immediately and still issue the hold.

_STALE_KEYWORDS = ("stale", "feedback")


def _reason_text(summary: dict) -> str:
    return str(summary.get("abort_reason") or "").lower()


def _assert_stale_reason(summary: dict) -> None:
    reason = _reason_text(summary)
    missing = [word for word in _STALE_KEYWORDS if word not in reason]
    assert not missing, (
        f"abort_reason does not name the dead feedback link ({missing} missing): {reason!r}"
    )


def test_module_should_document_the_feedback_stall_limit(latency) -> None:
    """The chosen variant is a call count: three polls without growth abort."""
    assert getattr(latency, "FEEDBACK_STALL_LIMIT", None) == 3


def test_run_tracking_steps_should_abort_when_the_observation_sequence_never_grows(latency) -> None:
    """A frozen sequence with a non-raising ``get_observation`` is an abort.

    This is the transport-timeout case: the client keeps answering with the
    cached frame, so the probe must notice that the feedback stopped rather than
    keep commanding a robot it can no longer see.
    """
    client = _TrackingFake(sequence_freezes_after=0)

    summary = _run_steps(latency, client, steps=5)

    _assert_stale_reason(summary)
    assert summary["steps_completed"] < 5, "a dead feedback link must not finish the run"
    assert client.sent_actions
    assert _is_hold_shape(client.sent_actions[-1]), (
        f"the run aborted on stale feedback without issuing a hold: {client.sent_actions[-1]}"
    )


def test_main_should_still_write_the_csv_when_the_feedback_dies(latency, tmp_path) -> None:
    """Stale feedback maps to a non-zero exit code, and the evidence stays on disk."""
    client = _TrackingFake(sequence_freezes_after=0)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "2",
            "--settle-timeout-s",
            "0.05",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_STEP_RESPONSE_ABORTED
    assert code != 0, "a dead feedback link must never look like a clean run"
    assert out.is_file(), "the aborted run must still leave its CSV on disk"
    with out.open(newline="", encoding="utf-8") as handle:
        header = next(csv.reader(handle))
    assert tuple(header) == tuple(latency.TRACKING_CSV_FIELDS)
    assert client.sent_actions
    assert _is_hold_shape(client.sent_actions[-1])


def test_run_tracking_steps_should_abort_after_the_feedback_freezes_mid_run(latency) -> None:
    """A healthy stream that freezes mid-run aborts late, not early and not never.

    «Late» is checked on the fake itself (the probe kept polling past the freeze
    point before deciding), «not never» by ``steps_completed < steps``.
    """
    freeze_after = 10
    client = _TrackingFake(sequence_freezes_after=freeze_after)

    summary = _run_steps(latency, client, steps=20, settle_timeout_s=0.3)

    _assert_stale_reason(summary)
    assert 0 < summary["steps_completed"] < 20, (
        f"expected an abort in the middle of the run, got {summary['steps_completed']} of 20 steps"
    )
    assert client.get_observation_calls > freeze_after, (
        "the probe stopped polling before the feedback froze, so it aborted for another reason"
    )


def test_tracking_report_should_say_the_hold_was_issued_not_delivered(
    latency, tmp_path, capsys
) -> None:
    """``hold_issued`` is the honest word: a PUSH/CONFLATE send has no receipt."""
    client = _TrackingFake()
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--settle-timeout-s",
            "0.01",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    printed = capsys.readouterr().out.lower()

    assert code == latency.EXIT_OK
    assert "hold_issued" in printed, f"the report does not label the hold as issued: {printed}"
    assert "hold_sent=yes" not in printed, (
        "the report claims the hold was sent as if delivery had been acknowledged"
    )


# ===========================================================================
# F. Ctrl-C and the exit-code layering
# ===========================================================================


def test_exit_step_response_aborted_should_be_one(latency) -> None:
    """The abort code is frozen at 1 so a shell wrapper can rely on it."""
    assert latency.EXIT_STEP_RESPONSE_ABORTED == 1


def _main_without_escaping_interrupt(latency, argv: list[str], factory: _Factory) -> int:
    """Call ``main`` and turn an escaping ``KeyboardInterrupt`` into a clean failure.

    ``pytest`` treats a ``KeyboardInterrupt`` leaving a test as Ctrl-C and aborts
    the whole session, so the escape is caught here and reported as the failure it
    is: ``main`` must write the CSV (and, in tracking mode, issue the hold) and
    return ``EXIT_STEP_RESPONSE_ABORTED``.
    """
    try:
        return latency.main(argv, client_factory=factory)
    except KeyboardInterrupt:
        pytest.fail(
            "main() let KeyboardInterrupt escape: it must write the CSV (and issue the hold in "
            "tracking mode) and return EXIT_STEP_RESPONSE_ABORTED"
        )
    except SystemExit as exc:  # argparse must not be involved at this point
        pytest.fail(f"main() exited through SystemExit while handling an interrupt: {exc!r}")


def _interrupting_argv(out: Path) -> list[str]:
    return [
        "--host",
        "127.0.0.1",
        "--mode",
        "tracking",
        "--enable-motion",
        "--joint",
        "arm_left_shoulder_pan",
        "--steps",
        "1",
        "--settle-timeout-s",
        "0.05",
        "--rate-hz",
        "200",
        "--csv",
        str(out),
    ]


def test_main_should_return_the_abort_code_when_the_probe_is_interrupted(latency, tmp_path) -> None:
    """Ctrl-C on the second command: exit 1, one recorded row, hold still last."""
    client = _TrackingFake(fail_send_at=2, fail_send_error=KeyboardInterrupt())
    out = tmp_path / "tracking.csv"

    code = _main_without_escaping_interrupt(latency, _interrupting_argv(out), _Factory(client))

    assert code == latency.EXIT_STEP_RESPONSE_ABORTED == 1
    assert out.is_file(), "an interrupted run must still write its CSV"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) >= 1, f"the interrupted step is missing from the CSV: {rows}"
    assert client.sent_actions
    assert _is_hold_shape(client.sent_actions[-1]), (
        f"Ctrl-C skipped the hold: {client.sent_actions[-1]}"
    )


def test_main_should_name_the_interrupt_on_stderr(latency, tmp_path, capsys) -> None:
    """The operator must be told the run was interrupted, not that it aborted for another reason."""
    client = _TrackingFake(fail_send_at=2, fail_send_error=KeyboardInterrupt())
    out = tmp_path / "tracking.csv"

    _main_without_escaping_interrupt(latency, _interrupting_argv(out), _Factory(client))
    printed = capsys.readouterr()

    assert "interrupt" in printed.err.lower(), (
        f"stderr does not name the interrupt: {printed.err!r}"
    )


def _rtt_interrupt_argv(out: Path) -> list[str]:
    """Argument list of a default rtt run whose CSV is written under ``tmp_path``."""
    return [
        "--host",
        "127.0.0.1",
        "--mode",
        "rtt",
        "--samples",
        "5",
        "--rate-hz",
        "200",
        "--csv",
        str(out),
    ]


def test_main_should_write_the_partial_rtt_csv_when_the_run_is_interrupted(
    latency, tmp_path, capsys
) -> None:
    """F2: Ctrl-C during rtt sampling writes the samples already taken and returns 1.

    ``EXIT_STEP_RESPONSE_ABORTED`` is reused on purpose: in both modes it means
    "stopped early, evidence on disk".  A run that took four frames and lost them
    to a traceback is the failure this test locks out.
    """
    client = _FakeClient(interrupt_at=2)
    out = tmp_path / "rtt.csv"

    code = _main_without_escaping_interrupt(latency, _rtt_interrupt_argv(out), _Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_STEP_RESPONSE_ABORTED == 1
    assert out.is_file(), "an interrupted rtt run must still write the samples it collected"
    with out.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        assert tuple(reader.fieldnames or ()) == tuple(latency.RTT_CSV_FIELDS)
        rows = list(reader)
    assert len(rows) >= 1, f"the collected samples are missing from the CSV: {rows}"
    assert "interrupt" in printed.err.lower(), (
        f"stderr does not name the interrupt: {printed.err!r}"
    )
    assert "p50" in printed.out.lower(), "the partial run did not print a quantile summary"
    assert client.send_action_calls == 0, "rtt mode must stay motion-free even when interrupted"


def test_main_should_not_let_a_keyboard_interrupt_escape_the_rtt_mode(latency, tmp_path) -> None:
    """F3: the call itself is the assertion — no ``pytest.raises``, the interrupt may not escape."""
    client = _FakeClient(interrupt_at=2)

    code = _main_without_escaping_interrupt(
        latency, _rtt_interrupt_argv(tmp_path / "rtt.csv"), _Factory(client)
    )

    assert code == latency.EXIT_STEP_RESPONSE_ABORTED


def test_main_should_write_a_header_only_rtt_csv_when_no_sample_was_collected(
    latency, tmp_path, capsys
) -> None:
    """F4: an interrupt before the first frame still leaves a readable CSV.

    ``interrupt_at=0`` makes the very first observation raise, so the run has zero
    rows — every other interrupt test keeps at least one.  The script writes the
    header anyway and returns the abort code: a missing file would lose the fact
    that a run was attempted at all, and the header is what re-reads the file.  No
    motion in rtt mode, interrupted or not.
    """
    client = _FakeClient(interrupt_at=0)
    out = tmp_path / "rtt.csv"

    code = _main_without_escaping_interrupt(latency, _rtt_interrupt_argv(out), _Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_STEP_RESPONSE_ABORTED == 1
    assert out.is_file(), "an interrupted rtt run must still write its CSV"
    with out.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = tuple(reader.fieldnames or ())
        rows = list(reader)
    assert rows == [], f"a run that collected no sample must not write data rows: {rows}"
    assert fieldnames == tuple(latency.RTT_CSV_FIELDS)
    assert "interrupt" in printed.err.lower(), (
        f"stderr does not name the interrupt: {printed.err!r}"
    )
    assert client.send_action_calls == 0, "rtt mode must stay motion-free even when interrupted"


# ===========================================================================
# G. Gate C must fail closed
# ===========================================================================


@pytest.mark.parametrize(
    "metadata",
    [{}, {"schema_version": 2}, {"schema_version": 2, "robot_model": None}],
    ids=["empty_metadata", "no_robot_model", "null_robot_model"],
)
def test_main_should_refuse_tracking_when_the_host_cannot_name_its_model(
    latency, tmp_path, capsys, metadata
) -> None:
    """Without a trustworthy ``robot_model`` the joint names/units are unverified.

    Fail closed: the old implementation only refused a *contradicting* model and
    let a missing one through, which is exactly the unverified case.
    """
    client = _TrackingFake()
    client.latest_robot_metadata = metadata
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert client.sent_actions == [], "a refused Gate C must not command anything"
    lowered = printed.err.lower()
    assert "metadata" in lowered or "robot_model" in lowered, (
        f"Gate C refusal does not name the missing field: {printed.err!r}"
    )
    assert not out.exists()


def _metadata_with_schema(schema_version: Any) -> dict:
    """Copy of the tracking metadata with ``schema_version`` set, or removed."""
    metadata: dict = {
        "robot_model": _TRACKING_METADATA["robot_model"],
        "motors": {key: dict(value) for key, value in _TRACKING_METADATA["motors"].items()},
    }
    if schema_version is not _MISSING:
        metadata["schema_version"] = schema_version
    return metadata


# ``_MISSING`` = the key is absent altogether; ``1`` = an older schema;
# ``"2"`` = the right number in the wrong type (a JSON string, not an int);
# ``true``/``false`` = a bool, which ``==`` reads as ``1``/``0`` and which is not
# the v2 layout either (``isinstance(x, bool)`` is checked before the equality).
_SCHEMA_VARIANTS = {
    "missing": _MISSING,
    "one": 1,
    "string": "2",
    "true": True,
    "false": False,
}


@pytest.mark.parametrize(
    "schema_version",
    list(_SCHEMA_VARIANTS.values()),
    ids=list(_SCHEMA_VARIANTS),
)
def test_main_should_refuse_tracking_when_the_metadata_schema_is_not_two(
    latency, tmp_path, capsys, schema_version: Any
) -> None:
    """G2: Gate F rejects a metadata schema that is missing or not ``2``.

    ``capture_robot_fingerprint`` already refuses a fingerprint whose
    ``schema_version`` is not ``2``; the motion unit gate has to use the same rule,
    otherwise a calibration table of an unknown layout is trusted and every degree
    figure of the run is derived from it.  ``"2"`` is included on purpose: a
    string that *looks* like the right version is still not the v2 layout.
    """
    client = _TrackingFake()
    client.latest_robot_metadata = _metadata_with_schema(schema_version)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert client.sent_actions == [], "a refused metadata schema must not command anything"
    lowered = printed.err.lower()
    assert "schema" in lowered or "metadata" in lowered, (
        f"the refusal does not name the metadata schema: {printed.err!r}"
    )
    assert not out.exists()


def test_the_legal_metadata_fixtures_should_carry_a_v2_schema() -> None:
    """Guard rail for the gate above: a legal fixture states ``schema_version == 2``.

    Not a test of the script but of this file's fixtures: once Gate F rejects any
    other schema, a fixture that forgot the field would turn an unrelated test red
    for the wrong reason.
    """
    for label, metadata in (
        ("_TRACKING_METADATA", _TRACKING_METADATA),
        ("_unit_metadata()", _unit_metadata()),
        ("_gripper_metadata()", _gripper_metadata()),
    ):
        assert metadata.get("schema_version") == 2, f"{label} does not state schema_version == 2"


# ===========================================================================
# H. Net round trip: the queue term only exists at or above the host rate
# ===========================================================================


def test_parse_args_should_default_host_loop_hz_to_50(latency) -> None:
    """``--host-loop-hz`` mirrors the host loop; the phase bound is derived from it."""
    args = latency.parse_args(["--host", "127.0.0.1"])

    assert hasattr(args, "host_loop_hz"), "--host-loop-hz is missing from the CLI"
    assert args.host_loop_hz == pytest.approx(50.0)


def test_rtt_csv_fields_should_carry_the_queue_bound(latency) -> None:
    """A row must state the queueing bound it subtracted, not only the phase bound."""
    assert "queue_bound_s" in latency.RTT_CSV_FIELDS


def _rtt_rows(latency, tmp_path, extra_argv: list[str], *, samples: int = 2) -> list[dict]:
    """Run rtt mode through ``main`` and return the CSV rows (no sockets)."""
    client = _FakeClient(ages=[0.030] * samples)
    out = tmp_path / "rtt.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--samples",
            str(samples),
            "--csv",
            str(out),
            *extra_argv,
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, f"rtt run failed with {code}"
    assert client.send_action_calls == 0, "rtt mode must stay motion-free"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == samples
    for row in rows:
        missing = [field for field in latency.RTT_CSV_FIELDS if field not in row]
        assert not missing, f"row does not cover {missing}"
    return rows


def test_rtt_rows_should_drop_the_queue_term_when_the_request_rate_is_below_the_host_rate(
    latency, tmp_path
) -> None:
    """``--rate-hz 30`` against a 50 Hz host drains the window: nothing is queued.

    The client refills the window after each consumed frame, so at a request
    rate below the host service rate the window is empty and there is no
    ``window - 1`` periods of queueing to subtract.
    """
    rows = _rtt_rows(
        latency, tmp_path, ["--rate-hz", "30", "--host-loop-hz", "50", "--request-window", "3"]
    )

    bounds = [float(row["queue_bound_s"]) for row in rows]
    assert bounds == pytest.approx([0.0] * len(rows)), f"queue term leaked into the rows: {bounds}"


def test_rtt_rows_should_count_the_queue_term_when_the_request_rate_matches_the_host_rate(
    latency, tmp_path
) -> None:
    """``--rate-hz 60`` against a 50 Hz host keeps the window queued: 2 / 50 Hz = 40 ms."""
    rows = _rtt_rows(
        latency, tmp_path, ["--rate-hz", "60", "--host-loop-hz", "50", "--request-window", "3"]
    )

    bounds = [float(row["queue_bound_s"]) for row in rows]
    assert bounds == pytest.approx([0.04] * len(rows)), (
        f"queue term missing from the rows: {bounds}"
    )


def test_rtt_rows_should_derive_the_phase_bound_from_the_host_loop_hz(latency, tmp_path) -> None:
    """``--host-loop-hz 20`` means 1 / 20 Hz = 50 ms of phase wait, not 20 ms."""
    rows = _rtt_rows(latency, tmp_path, ["--rate-hz", "30", "--host-loop-hz", "20"])

    bounds = [float(row["phase_bound_s"]) for row in rows]
    assert bounds == pytest.approx([0.05] * len(rows)), f"phase bound ignored the flag: {bounds}"


# ===========================================================================
# I. CLI range validation (argparse level, not the connect-failure path)
# ===========================================================================


@pytest.mark.parametrize(
    "argv",
    [
        ["--samples=0"],
        ["--samples=-1"],
        ["--rate-hz=0"],
        ["--rate-hz=-1"],
        ["--request-window=0"],
    ],
    ids=["samples_zero", "samples_negative", "rate_zero", "rate_negative", "window_zero"],
)
def test_parse_args_should_reject_out_of_range_values(latency, argv: list[str]) -> None:
    """A nonsense run shape is a usage error (2), not a connect failure (3)."""
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", *argv])

    assert excinfo.value.code == 2


# ===========================================================================
# J. Units: a ``.pos`` feedback value is a normalized unit, never a degree
# ===========================================================================
#
# ``AlohaMiniConfig.use_degrees`` is ``False`` by default and the Host exposes no
# switch for it, so ``AlohaMini`` normalizes every arm joint to
# ``MotorNormMode.RANGE_M100_100`` (``((raw - min) / (max - min)) * 200 - 100``).
# One observed unit therefore spans ``(range_max - range_min) / 200`` ticks, i.e.
# ``((range_max - range_min) / 200) * 360 / (resolution - 1)`` degrees, and every
# degree figure this harness clamps, compares or prints has to be converted with
# that factor from the Host metadata.  Round 3 treated the unit as a degree.

_J_UNIT_MOTOR = _TRACKING_MOTOR
_J_UNIT_JOINT = _TRACKING_JOINT
# (3000 - 1000) / 200 * 360 / (4096 - 1) = 0.8791208791208791 deg per unit.
_J_UNIT_SCALE_DEG = _TRACKING_SCALE_DEG


def _unit_metadata(**overrides: Any) -> dict:
    """Host metadata carrying the conversion fields of the stepped joint."""
    entry = dict(_TRACKING_MOTOR_ENTRY)
    entry.update(overrides)
    return {
        "schema_version": 2,
        "robot_model": "alohamini2pro",
        "motors": {_J_UNIT_MOTOR: entry},
    }


def _unit_error_class(latency) -> type[BaseException]:
    """The module's unit-conversion exception: its name has to carry ``unit``."""
    candidates = [
        value
        for name, value in vars(latency).items()
        if isinstance(value, type) and issubclass(value, Exception) and "unit" in name.lower()
    ]
    assert candidates, (
        "the module must define a unit-conversion exception whose name contains 'unit' "
        "(e.g. ``UnitConversionUnavailable``)"
    )
    return candidates[0]


def test_deg_per_unit_should_convert_one_normalized_unit_into_degrees(latency) -> None:
    """range_min=1000, range_max=3000, resolution=4096 → 0.8791208791 deg/unit."""
    metadata = _unit_metadata()

    scale = latency.deg_per_unit(metadata, _J_UNIT_JOINT)

    assert scale == pytest.approx(_J_UNIT_SCALE_DEG)


def test_deg_per_unit_should_accept_the_joint_without_its_pos_suffix(latency) -> None:
    """``--joint`` may be given with or without ``.pos``; the lookup is the same."""
    metadata = _unit_metadata()

    assert latency.deg_per_unit(metadata, _J_UNIT_MOTOR) == pytest.approx(_J_UNIT_SCALE_DEG)


@pytest.mark.parametrize(
    "metadata",
    [
        {},
        {"schema_version": 2, "robot_model": "alohamini2pro"},
        {"schema_version": 2, "robot_model": "alohamini2pro", "motors": {}},
        {"motors": {_J_UNIT_MOTOR: {"range_max": 3000, "resolution": 4096}}},
        {"motors": {_J_UNIT_MOTOR: {"range_min": 1000, "resolution": 4096}}},
        {"motors": {_J_UNIT_MOTOR: {"range_min": 1000, "range_max": 3000}}},
        _unit_metadata(range_max=900),
        _unit_metadata(range_max=1000),
        _unit_metadata(resolution=1),
        _unit_metadata(resolution=0),
        _unit_metadata(range_max=float("inf")),
        _unit_metadata(resolution=float("nan")),
    ],
    ids=[
        "empty_metadata",
        "no_motors_mapping",
        "empty_motors_mapping",
        "missing_range_min",
        "missing_range_max",
        "missing_resolution",
        "range_max_below_range_min",
        "range_max_equal_range_min",
        "resolution_one",
        "resolution_zero",
        "range_max_not_finite",
        "resolution_not_finite",
    ],
)
def test_deg_per_unit_should_raise_instead_of_guessing(latency, metadata: dict) -> None:
    """J2-J5: a missing or unusable calibration is a refusal, never a fallback.

    A guessed factor would silently convert the unit as if it were a degree and
    every motion limit built on it would be wrong in an unknown direction.
    """
    error = _unit_error_class(latency)

    with pytest.raises(error) as excinfo:
        latency.deg_per_unit(metadata, _J_UNIT_JOINT)

    assert "unit" in str(excinfo.value).lower(), (
        f"the refusal does not name the unit conversion: {excinfo.value}"
    )


def test_units_from_deg_should_divide_by_the_scale(latency) -> None:
    """1.0 deg on a 0.9 deg/unit joint is 1.111... units, not 0.9 and not 1.0."""
    assert latency.units_from_deg(1.0, 0.9) == pytest.approx(1.0 / 0.9, rel=0, abs=1e-12)
    assert latency.units_from_deg(1.8, 1.8) == pytest.approx(1.0, rel=0, abs=1e-12)


def test_deg_from_units_should_multiply_by_the_scale(latency) -> None:
    """The inverse direction: 2 units on a 0.9 deg/unit joint is 1.8 deg."""
    assert latency.deg_from_units(2.0, 0.9) == pytest.approx(1.8, rel=0, abs=1e-12)
    assert latency.deg_from_units(0.0, 1.8) == pytest.approx(0.0, rel=0, abs=1e-12)


@pytest.mark.parametrize("scale", [0.8791208791208791, 0.9, 1.0, 1.8])
def test_unit_conversions_should_round_trip(latency, scale: float) -> None:
    """J6: ``deg_from_units(units_from_deg(d)) == d`` and vice versa."""
    for delta_deg in (-1.0, 0.0, 0.25, 1.8):
        assert latency.deg_from_units(
            latency.units_from_deg(delta_deg, scale), scale
        ) == pytest.approx(delta_deg, rel=0, abs=1e-12)
    for units in (-3.0, 0.0, 2.5):
        assert latency.units_from_deg(latency.deg_from_units(units, scale), scale) == pytest.approx(
            units, rel=0, abs=1e-12
        )


@pytest.mark.parametrize("scale", [0.9, 1.8])
def test_run_tracking_steps_should_command_the_delta_converted_into_units(
    latency, scale: float
) -> None:
    """J7: ``--delta-deg 1.0`` is 1.0/scale units on the wire, not 1.0 units.

    The feedback is already normalized, so the goal built on top of it is in the
    same unit.  Sending the degree figure through unchanged commands 1.111 units
    as if it were 1.111 deg — 12 % of extra travel on 0.9 deg/unit, 80 % on 1.8.
    """
    client = _TrackingFake()
    baseline_units = _TRACKING_STATE[_TRACKING_JOINT]
    expected_units = baseline_units + 1.0 / scale

    summary = _run_steps(latency, client, deg_per_unit=scale, steps=2)

    values = [action[_TRACKING_JOINT] for action in client.sent_actions]
    step_values = [
        value for value in values if not math.isclose(value, baseline_units, abs_tol=1e-12)
    ]
    assert summary["steps_completed"] == 2
    assert step_values == pytest.approx([expected_units] * len(step_values), rel=0, abs=1e-9)
    # The closing hold freezes at the measured pose, in units, like every other value.
    assert values[-1] == pytest.approx(baseline_units, rel=0, abs=1e-9)


def test_run_tracking_rows_should_record_degrees_and_the_sent_units(latency) -> None:
    """``cmd_deg``/``baseline_deg``/``error_deg`` are degrees, ``cmd_units`` is not."""
    scale = 0.9
    client = _TrackingFake()
    baseline_units = _TRACKING_STATE[_TRACKING_JOINT]

    summary = _run_steps(latency, client, deg_per_unit=scale, steps=1)

    row = summary["rows"][0]
    assert row["deg_per_unit"] == pytest.approx(scale, rel=0, abs=1e-12)
    assert row["baseline_deg"] == pytest.approx(baseline_units * scale, rel=0, abs=1e-9)
    assert row["cmd_deg"] == pytest.approx(baseline_units * scale + 1.0, rel=0, abs=1e-9)
    assert row["measured_deg"] == pytest.approx(baseline_units * scale, rel=0, abs=1e-9)
    assert row["error_deg"] == pytest.approx(row["cmd_deg"] - row["measured_deg"], rel=0, abs=1e-12)
    assert row["cmd_units"] == pytest.approx(baseline_units + 1.0 / scale, rel=0, abs=1e-9)


def test_run_tracking_rows_should_report_the_overshoot_in_degrees(latency) -> None:
    """A 1.4 unit overshoot on a 0.9 deg/unit joint is 1.26 deg, i.e. 0.26 deg over."""
    scale = 0.9
    # Unit feedback per poll: baseline, +0.6, +1.4 (peak), +1.05 (settled).
    client = _ScriptedTrackingFake(values=[1.0, 1.6, 2.4, 2.05])

    summary = _run_steps(latency, client, deg_per_unit=scale, steps=1, settle_timeout_s=0.4)

    row = summary["rows"][0]
    peak_travel_deg = (2.4 - 1.0) * scale
    assert row["overshoot_deg"] == pytest.approx(peak_travel_deg - 1.0, rel=0, abs=1e-9)
    assert row["settled"] is True


def test_tracking_csv_fields_should_carry_the_units_and_the_scale(latency) -> None:
    """A row states both what was asked in degrees and what went out in units."""
    fields = set(latency.TRACKING_CSV_FIELDS)
    missing = {"cmd_units", "deg_per_unit"} - fields

    assert not missing, f"TRACKING_CSV_FIELDS does not carry {sorted(missing)}"


def test_main_should_refuse_tracking_when_the_joint_units_cannot_be_resolved(
    latency, tmp_path, capsys
) -> None:
    """J8: no calibration for the target joint → refused, zero motion.

    Fail closed: without ``range_min``/``range_max``/``resolution`` the degree
    value of one unit is unknown, so neither the step size nor any cap can be
    honoured.  The gate runs after Gate C (the model check) and before the first
    ``send_action``.
    """
    client = _TrackingFake()
    client.latest_robot_metadata = {"schema_version": 2, "robot_model": "alohamini2pro"}
    factory = _Factory(client)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--csv",
            str(out),
        ],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert client.sent_actions == [], "a run without a unit factor must not command anything"
    assert "unit" in captured.err.lower(), (
        f"the refusal does not name the unit conversion: {captured.err!r}"
    )
    assert not out.exists()


def test_main_should_judge_the_travel_cap_in_degrees(latency, tmp_path, capsys) -> None:
    """21 x 1.0 deg is over the 20 deg cap although it is only 11.7 units here.

    The cap protects the real robot, so it is judged on degrees: a run that
    applied it to the normalized units would let this one through.
    """
    client = _TrackingFake()
    # (4095 - 0) / 200 * 360 / (4096 - 1) = 1.8 deg/unit → 1.0 deg = 0.5556 units.
    client.latest_robot_metadata = _unit_metadata(range_min=0, range_max=4095, resolution=4096)
    factory = _Factory(client)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--delta-deg",
            "1.0",
            "--steps",
            "21",
            "--csv",
            str(out),
        ],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED
    assert client.sent_actions == []
    assert "Gate E" in captured.err


# ===========================================================================
# K. The prefetch pipeline: ``age_s`` carries ``window - 1`` client periods
# ===========================================================================
#
# The client consumes the *oldest* token and refills the window only after the
# answer, so the frame being read was requested ``window`` client calls ago.
# That wait (``(window - 1) * max(1 / rate_hz, 1 / host_loop_hz)``) is part of
# ``age_s`` and has to leave the net estimate; round 3 had no term for it and
# its queue gate switched the window off exactly when the client is the slower
# side, i.e. in the case where the prefetch wait dominates.


def test_net_rtt_estimate_should_subtract_the_client_prefetch_wait(latency) -> None:
    """K1: window 3 at 30 Hz / 50 Hz host → 2/30 s bound, 1/30 s mean of prefetch."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.003,
    }

    estimate = latency.net_rtt_estimate_s(
        0.100, host_timing, request_window=3, rate_hz=30.0, host_loop_hz=50.0
    )

    # 1/30 s dominates 1/50 s: two prefetch actions ahead, mean = one period.
    assert estimate["prefetch_bound_s"] == pytest.approx(2.0 / 30.0, rel=0, abs=1e-12)
    assert estimate["prefetch_mean_s"] == pytest.approx(1.0 / 30.0, rel=0, abs=1e-12)
    # The round-3 queue gate is untouched: 30 Hz < 50 Hz drains the window.
    assert estimate["queue_bound_s"] == pytest.approx(0.0, rel=0, abs=1e-12)
    assert estimate["queue_mean_s"] == pytest.approx(0.0, rel=0, abs=1e-12)
    assert estimate["phase_mean_s"] == pytest.approx(0.010, rel=0, abs=1e-12)
    # 0.100 - 0.003 - 0.010 - 0.0 - 1/30 = 0.0536666667
    assert estimate["net_rtt_est_s"] == pytest.approx(
        0.100 - 0.003 - 0.010 - 1.0 / 30.0, rel=0, abs=1e-9
    )
    # low  = 0.100 - 0.003 - 0.020 - 0.0 - 2/30 = 0.0103333333
    assert estimate["net_rtt_low_s"] == pytest.approx(
        0.100 - 0.003 - 0.020 - 2.0 / 30.0, rel=0, abs=1e-9
    )
    # high = 0.100 - 0.003 = 0.097: the upper bound removes the host sample only
    # and discounts no waiting term (no prefetch, no phase, no queue), so it is
    # by construction >= the point estimate, which does remove their means.
    assert estimate["net_rtt_high_s"] == pytest.approx(0.100 - 0.003, rel=0, abs=1e-9)
    assert estimate["net_rtt_low_s"] <= estimate["net_rtt_est_s"] <= estimate["net_rtt_high_s"]


def test_net_rtt_estimate_should_keep_the_round_3_numbers_without_a_rate(latency) -> None:
    """K2: ``rate_hz=None`` → no prefetch term and bit-for-bit the old estimate."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.004,
    }

    estimate = latency.net_rtt_estimate_s(0.035, host_timing, request_window=3)

    assert estimate["prefetch_bound_s"] == pytest.approx(0.0, rel=0, abs=1e-12)
    assert estimate["prefetch_mean_s"] == pytest.approx(0.0, rel=0, abs=1e-12)
    assert estimate["queue_bound_s"] == pytest.approx(0.04, rel=0, abs=1e-12)
    assert estimate["net_rtt_est_s"] == pytest.approx(0.035 - 0.004 - 0.01 - 0.02, rel=0, abs=1e-9)


def test_net_rtt_estimate_should_have_no_prefetch_term_for_a_single_window(latency) -> None:
    """K3: one request in flight prefetches nothing, whatever the client rate."""
    host_timing = {
        "state_sample_started_monotonic_s": 1000.0,
        "state_sample_finished_monotonic_s": 1000.004,
    }

    estimate = latency.net_rtt_estimate_s(
        0.035, host_timing, request_window=1, rate_hz=30.0, host_loop_hz=50.0
    )

    assert estimate["prefetch_bound_s"] == pytest.approx(0.0, rel=0, abs=1e-12)
    assert estimate["prefetch_mean_s"] == pytest.approx(0.0, rel=0, abs=1e-12)


def test_rtt_rows_should_subtract_the_prefetch_wait_of_the_window(latency, tmp_path) -> None:
    """K4: the CSV row states the prefetch term, and ``age_s`` stops being the estimate.

    ``--request-window 3`` is the deployment shape; at 30 Hz against a 50 Hz host
    the queue term stays zero (round-3 gate) while the prefetch wait is 2/30 s.
    """
    assert {"prefetch_bound_s", "prefetch_mean_s"} <= set(latency.RTT_CSV_FIELDS)
    client = _FakeClient(ages=[0.120, 0.121])
    out = tmp_path / "rtt.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--samples",
            "2",
            "--rate-hz",
            "30",
            "--host-loop-hz",
            "50",
            "--request-window",
            "3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2
    for row in rows:
        assert float(row["prefetch_bound_s"]) == pytest.approx(2.0 / 30.0, rel=0, abs=1e-12)
        assert float(row["prefetch_mean_s"]) == pytest.approx(1.0 / 30.0, rel=0, abs=1e-12)
        assert float(row["queue_bound_s"]) == pytest.approx(0.0, rel=0, abs=1e-12)
        # age_s and net_rtt_est_s must no longer be all but equal: the prefetch
        # wait + host sample + phase mean is what separates them.
        separation = float(row["age_s"]) - float(row["net_rtt_est_s"])
        assert separation == pytest.approx(0.004 + 0.010 + 1.0 / 30.0, rel=0, abs=1e-9)


# ===========================================================================
# L. The lift axis is pinned in every command
# ===========================================================================
#
# The command stream runs at 5-50 Hz, so the Host's 1 s command watchdog never
# fires during a run.  ``LiftAxis.apply_action`` writes no lift register when the
# key is absent, and the lift then keeps moving on the ``Goal_Velocity`` the
# previous session left behind.  Writing the measured height back is what pins
# the axis; the key must not be invented when the feedback does not carry it.

# ``_LIFT_HEIGHT_KEY`` is declared with the tracking fixture above (the
# full-command key set is built from it), so section L only refers to it.


def test_run_tracking_steps_should_pin_the_measured_lift_height_in_every_command(latency) -> None:
    """L1: every step command *and* the closing hold carry the latest measured height."""
    heights = [40.0, 40.5, 41.0, 41.5, 42.0, 42.5, 43.0, 43.5]
    client = _ScriptedTrackingFake(values=[1.0], lift=heights)

    _run_steps(latency, client, steps=1, settle_timeout_s=0.3)

    assert client.sent_actions
    for action in client.sent_actions:
        assert _LIFT_HEIGHT_KEY in action, f"command drops the lift axis: {sorted(action)}"
    commanded = [action[_LIFT_HEIGHT_KEY] for action in client.sent_actions]
    assert all(height in client.observed_lift_heights for height in commanded), (
        f"a lift target was not a measured height: {commanded} vs {client.observed_lift_heights}"
    )
    assert len(set(commanded)) > 1, (
        "the lift target is frozen at one value instead of following the feedback"
    )
    assert commanded[-1] == pytest.approx(client.observed_lift_heights[-1])


def test_run_tracking_steps_should_not_invent_a_lift_height(latency) -> None:
    """L2: feedback without ``lift_axis.height_mm`` must not produce a made-up 0.0."""
    client = _TrackingFake(state=_TRACKING_STATE_WITHOUT_LIFT)

    _run_steps(latency, client)

    assert client.sent_actions
    for action in client.sent_actions:
        assert _LIFT_HEIGHT_KEY not in action, (
            f"the command invented a lift height: {action[_LIFT_HEIGHT_KEY]!r}"
        )


def test_run_tracking_steps_should_pin_the_measured_lift_height_verbatim(latency) -> None:
    """L3: each command repeats the height the *latest frame* measured, not a constant.

    Two different heights are scripted (12.5 mm before the step, 13.0 mm from the
    first settle poll on), so the three wrong answers are all distinguishable:
    a frozen first value (12.5 everywhere), the settle-time value pinched back
    into the step command (13.0 everywhere), and a made-up ``0.0``.  The expected
    value per command is recorded by the fake at ``send_action`` time, which is
    exactly the frame that had arrived — no post-hoc reconstruction.
    """
    client = _ScriptedTrackingFake(values=[1.0], lift=[12.5, 13.0])

    _run_steps(latency, client, steps=1, settle_timeout_s=0.3)

    assert client.sent_actions
    assert len(client.lift_at_send) == len(client.sent_actions)
    assert all(_LIFT_HEIGHT_KEY in action for action in client.sent_actions), (
        f"a command dropped the lift axis: {[sorted(action) for action in client.sent_actions]}"
    )
    commanded = [action[_LIFT_HEIGHT_KEY] for action in client.sent_actions]
    assert all(value != 0.0 for value in commanded), f"a lift height was invented: {commanded}"
    assert commanded == pytest.approx(client.lift_at_send), (
        f"a command did not pin the latest measured height: {commanded} vs {client.lift_at_send}"
    )
    # The step command is issued after the 12.5 mm frame, the hold after 13.0 mm:
    # two distinct values prove the pin follows the feedback instead of freezing.
    assert commanded[0] == pytest.approx(12.5)
    assert commanded[-1] == pytest.approx(13.0)
    assert set(commanded) == {12.5, 13.0}


# ===========================================================================
# M. ``rise_90`` is timed against the arrival stamp of the crossing frame
# ===========================================================================


def test_run_tracking_steps_should_time_the_rise_against_the_frame_arrival(latency) -> None:
    """M1: ``received_monotonic_s`` of the crossing frame, not the poll clock.

    The poll clock is read *before* the blocking read, so a clock-based rise time
    is one observation short.  The fake stamps its frames three orders of
    magnitude away from the poll clock, which makes the two variants
    distinguishable instead of nearly equal.
    """
    arrivals = [10.0, 2000.0, 3000.0]
    client = _ScriptedTrackingFake(values=[1.0, 1.5, 1.95], received=arrivals)

    summary = _run_steps(latency, client, steps=1, settle_timeout_s=0.3)

    row = summary["rows"][0]
    assert "received_monotonic_s" in latency.TRACKING_CSV_FIELDS
    assert row["received_monotonic_s"] == pytest.approx(arrivals[2], rel=0, abs=1e-9)
    assert row["rise_90_time_s"] == pytest.approx(
        row["received_monotonic_s"] - row["t_send_monotonic_s"], rel=0, abs=1e-9
    )
    assert row["rise_90_time_s"] > 1000.0, (
        "rise_90_time_s is a poll-clock delta, not the arrival stamp of the crossing frame"
    )


# ===========================================================================
# N. CLI range validation for the tracking knobs
# ===========================================================================


@pytest.mark.parametrize(
    "argv",
    [
        ["--steps=0"],
        ["--steps=-1"],
        ["--settle-timeout-s=0"],
        ["--settle-timeout-s=-1"],
        ["--settle-tolerance-deg=-0.1"],
    ],
    ids=[
        "steps_zero",
        "steps_negative",
        "settle_timeout_zero",
        "settle_timeout_negative",
        "settle_tolerance_negative",
    ],
)
def test_parse_args_should_reject_a_nonsense_step_or_settle_shape(latency, argv: list[str]) -> None:
    """N1: a zero step count or a negative budget is a usage error (2), not a run."""
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", *argv])

    assert excinfo.value.code == 2


# N2/N3: ``--settle-tolerance-deg`` is checked against the *effective* step size
# at parse time, i.e. against ``--delta-deg`` after the ``MAX_STEP_DELTA_DEG``
# clamp.  A tolerance at or above the step makes the first settle poll judge the
# joint settled, so every row keeps ``rise_90_time_s`` empty and the column is
# silently useless.
#
# The reference is the *magnitude* of that effective step, not its signed value:
# the settle judgement itself is ``abs(measured - target) <= tolerance``, so a
# ``--delta-deg -1.0`` run has exactly the same empty-column failure mode as
# ``--delta-deg 1.0`` and is refused by the same rule (section ``AA``).  A
# signed comparison would be vacuously true for every negative step
# (``0.15 >= -1.0``) and would refuse the whole negative direction.
#
# Scope ruling of this file: **unconditional**, not ``--mode tracking`` only.
# The knob is parsed in every mode and the default pair (``delta 1.0`` /
# ``tolerance 0.15``) is safe, so a mode-conditional check would only add a
# branch (and a hole: ``--delta-deg`` is not rejected in rtt mode either).  The
# rtt mode ignores both values, so rejecting the combination early costs it
# nothing.


@pytest.mark.parametrize(
    "argv",
    [
        ["--delta-deg", "1.0", "--settle-tolerance-deg", "1.0"],
        ["--delta-deg", "0.5", "--settle-tolerance-deg", "0.6"],
        ["--delta-deg", "5.0", "--settle-tolerance-deg", "1.0"],
    ],
    ids=["equal", "above", "above_with_clamped_delta"],
)
def test_parse_args_should_reject_a_settle_tolerance_at_or_above_the_step(
    latency, capsys, argv: list[str]
) -> None:
    """N2: tolerance >= effective step is a usage error (2), never a silent empty column.

    The third variant asks for ``--delta-deg 5.0``, which the clamp turns into
    ``MAX_STEP_DELTA_DEG == 1.0``: the value the tolerance is compared against is
    the one actually used, not the one typed.
    """
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", *argv])

    assert excinfo.value.code == 2
    message = capsys.readouterr().err.lower()
    assert "settle-tolerance" in message, f"the error does not name the tolerance: {message!r}"
    assert "delta" in message, f"the error does not name the step size: {message!r}"


def test_parse_args_should_accept_a_settle_tolerance_below_the_step(latency) -> None:
    """N3: a tolerance strictly below the step parses normally (no SystemExit)."""
    args = latency.parse_args(
        ["--host", "127.0.0.1", "--delta-deg", "1.0", "--settle-tolerance-deg", "0.9"]
    )

    assert args.delta_deg == pytest.approx(1.0)
    assert args.settle_tolerance_deg == pytest.approx(0.9)
    assert latency.resolve_delta_deg(args) > args.settle_tolerance_deg


# ===========================================================================
# O. The gripper is ``RANGE_0_100``: the span is 100, not 200
# ===========================================================================
#
# ``_ARM_PROFILES`` of the host hands every arm joint ``None`` (so it inherits
# ``norm_mode_body = RANGE_M100_100``) but each ``gripper`` entry an explicit
# ``MotorNormMode.RANGE_0_100``, and ``build_robot_metadata`` publishes that
# choice as ``motors.<motor>.normalization`` (``motor.norm_mode.value``).
# ``MotorsBus._normalize`` maps a ``RANGE_0_100`` tick to
# ``((raw - min) / (max - min)) * 100``, so one observed unit is
# ``(range_max - range_min) / 100`` ticks — half the tick span of a
# ``RANGE_M100_100`` joint and therefore *twice* the degrees per unit.  A
# ``deg_per_unit`` that hard-codes 200 silently commands the gripper with a ~2x
# step: the goal is built as ``baseline + delta_deg / scale``, so a scale that is
# too small asks for twice the travel.
#
# Compatibility rule frozen here: the ``normalization`` key stays optional.
# Metadata that does not carry it is read as ``range_m100_100`` (span 200), which
# keeps the round-3 J1 case and every calibration fixture written before the
# gripper bug was found exact.  An *unrecognised* value is refused instead — a
# typo must never fall back to the arm span.

_GRIPPER_MOTOR = "arm_left_gripper"
_GRIPPER_JOINT = "arm_left_gripper.pos"
# (3000 - 1000) / 100 * 360 / (4096 - 1) = 1.7582417582417582 deg per unit,
# exactly twice the RANGE_M100_100 factor 0.8791208791208791 of section J.
_GRIPPER_SCALE_DEG = 1.7582417582417582


def _gripper_metadata(**overrides: Any) -> dict:
    """Host metadata of a ``RANGE_0_100`` gripper motor (span 100)."""
    entry: dict[str, Any] = {**_TRACKING_MOTOR_ENTRY, "normalization": "range_0_100"}
    entry.update(overrides)
    return {
        "schema_version": 2,
        "robot_model": "alohamini2pro",
        "motors": {_GRIPPER_MOTOR: entry},
    }


def test_deg_per_unit_should_halve_the_span_for_a_range_0_100_joint(latency) -> None:
    """P1: a 100-wide normalization covers twice the degrees of a 200-wide one.

    ``RANGE_0_100`` normalizes to ``((raw - min) / (max - min)) * 100``, so one
    unit is ``(range_max - range_min) / 100`` ticks:
    ``(3000 - 1000) / 100 * 360 / 4095 = 1.758241758...`` deg/unit — twice the
    0.879120879... deg/unit the same calibration gives as ``RANGE_M100_100``.
    """
    arm_scale = latency.deg_per_unit(_unit_metadata(normalization="range_m100_100"), _J_UNIT_JOINT)
    gripper_scale = latency.deg_per_unit(_gripper_metadata(), _GRIPPER_JOINT)

    assert gripper_scale == pytest.approx(1.7582417582417582, rel=0, abs=1e-12)
    assert arm_scale == pytest.approx(_J_UNIT_SCALE_DEG, rel=0, abs=1e-12)
    assert gripper_scale == pytest.approx(2.0 * arm_scale, rel=0, abs=1e-15)


def test_deg_per_unit_should_return_one_for_a_degrees_normalized_joint(latency) -> None:
    """P2: ``degrees`` is already a degree — no span, no resolution, no interval.

    A degenerate calibration (``range_min == range_max``, which is refused for
    every range-based mode) returns ``1.0`` here: the degree mode never divides
    by the range, so there is nothing to reject.
    """
    metadata = _gripper_metadata(normalization="degrees", range_min=1000, range_max=1000)

    assert latency.deg_per_unit(metadata, _GRIPPER_JOINT) == pytest.approx(1.0, rel=0, abs=0.0)


def test_deg_per_unit_should_raise_for_an_unknown_normalization(latency) -> None:
    """P3: an unrecognised ``normalization`` is refused, never read as the arm span."""
    error = _unit_error_class(latency)
    metadata = _gripper_metadata(normalization="range_bogus")

    with pytest.raises(error) as excinfo:
        latency.deg_per_unit(metadata, _GRIPPER_JOINT)

    assert "unit" in type(excinfo.value).__name__.lower()
    assert "unit" in str(excinfo.value).lower(), (
        f"the refusal does not name the unit conversion: {excinfo.value}"
    )


def test_deg_per_unit_should_read_an_absent_normalization_as_range_m100_100(latency) -> None:
    """P4: compatibility — a missing ``normalization`` key keeps the 200 span.

    Fixtures and host snapshots that predate the field carry only the
    calibration triple, and the arm joints genuinely are ``RANGE_M100_100``
    (``use_degrees`` is off and ``norm_mode_body`` is the inherited default), so
    an absent key must stay valid and equal the explicit arm mode.
    """
    absent = _unit_metadata()
    explicit = _unit_metadata(normalization="range_m100_100")

    assert "normalization" not in absent["motors"][_J_UNIT_MOTOR]
    absent_scale = latency.deg_per_unit(absent, _J_UNIT_JOINT)
    explicit_scale = latency.deg_per_unit(explicit, _J_UNIT_JOINT)

    assert absent_scale == pytest.approx(_J_UNIT_SCALE_DEG, rel=0, abs=1e-12)
    assert absent_scale == pytest.approx(explicit_scale, rel=0, abs=0.0)


def test_run_tracking_steps_should_command_the_gripper_delta_in_gripper_units(latency) -> None:
    """P5: behaviour level — 1.0 deg on the gripper is 1/1.7582 units, not 1/0.8791.

    The expected unit count is written from the ``range_0_100`` factor on
    purpose: a scale that still assumes the 200 span builds a goal twice as far
    from the measured pose, which is the silent ~2x step this section freezes.
    """
    scale = latency.deg_per_unit(_gripper_metadata(), _GRIPPER_JOINT)
    client = _TrackingFake()
    baseline_units = _TRACKING_STATE[_GRIPPER_JOINT]
    expected_units = baseline_units + 1.0 / _GRIPPER_SCALE_DEG

    summary = _run_steps(latency, client, joint_key=_GRIPPER_JOINT, deg_per_unit=scale, steps=2)

    values = [action[_GRIPPER_JOINT] for action in client.sent_actions]
    step_values = [
        value for value in values if not math.isclose(value, baseline_units, abs_tol=1e-12)
    ]
    assert summary["steps_completed"] == 2
    assert step_values, f"the run never commanded the gripper: {values}"
    assert step_values == pytest.approx([expected_units] * len(step_values), rel=0, abs=1e-9)
    # The closing hold freezes at the measured pose, in the same units.
    assert values[-1] == pytest.approx(baseline_units, rel=0, abs=1e-9)


# ===========================================================================
# Q. Round 6: command pacing (``--control-hz``), independent of ``--rate-hz``
# ===========================================================================
#
# The tracking loop re-issues the goal on every feedback poll, so the number of
# commands per second used to be an artifact of the poll rate: a fake clock read
# as fast as the loop could run produced ~87k ``send_action`` calls in 0.238 s.
# ``--control-hz`` (default ``50.0``, ``>= 1``) is the contract frozen here: two
# consecutive commands are at least ``1 / control_hz`` seconds apart, every
# command of the tracking path is paced (step head and settle repeats), the
# pacing goes through the injected ``sleep`` and never through ``time.sleep``,
# and ``rise_90`` keeps its arrival-stamp semantics.  See the "Round 6" block of
# the module docstring for the full contract.

_CONTROL_HZ = 50.0


class _PacingHarness:
    """Fake clock plus the sleeper that moves it: nothing else advances time.

    ``read_cost_s`` charges a (tiny) amount to every clock read so a *buggy* run
    cannot spin forever: the settle loop is bounded by the injected clock, and a
    clock that only moves inside ``sleep`` would never leave a loop that does not
    sleep.  ``sleeps`` records every ``dt`` the script handed to the injected
    sleeper, in order.
    """

    def __init__(self, *, start: float = 0.0, read_cost_s: float = 0.0) -> None:
        self.now = start
        self._read_cost_s = read_cost_s
        self.sleeps: list[float] = []
        self.reads = 0

    def clock(self) -> float:
        self.reads += 1
        current = self.now
        self.now += self._read_cost_s
        return current

    def sleep(self, seconds: float) -> None:
        duration = float(seconds)
        self.sleeps.append(duration)
        if duration > 0.0:
            self.now += duration

    def advance(self, seconds: float) -> None:
        """Burn time inside a fake read: a blocking observation costs clock time."""
        if seconds > 0.0:
            self.now += float(seconds)


class _PacedTrackingFake(_TrackingFake):
    """Tracking stand-in whose observations cost clock time and whose sends are stamped.

    ``send_times`` is the harness clock at every ``send_action``.  The pacing
    assertion is arithmetic on those stamps (``count - 1`` gaps of at least
    ``1 / control_hz`` have to fit into the measured span), so the fake records
    them itself instead of letting the test reconstruct them afterwards.
    """

    def __init__(self, harness: _PacingHarness, *, obs_cost_s: float = 0.0) -> None:
        super().__init__()
        self._harness = harness
        self._obs_cost_s = obs_cost_s
        self.send_times: list[float] = []

    def get_observation(self, include_cameras: bool = True) -> dict:
        self._harness.advance(self._obs_cost_s)
        return super().get_observation(include_cameras=include_cameras)

    def send_action(self, action: dict) -> dict:
        self.send_times.append(self._harness.now)
        return super().send_action(action)


def _paced_run(
    latency,
    *,
    control_hz: float,
    steps: int = 1,
    settle_timeout_s: float = 0.2,
    obs_cost_s: float = 0.004,
    read_cost_s: float = 0.0,
    rate_hz: float = 30.0,
) -> tuple[_PacingHarness, _PacedTrackingFake, dict]:
    """Run one tracked probe against the pacing harness (no real time involved)."""
    harness = _PacingHarness(read_cost_s=read_cost_s)
    client = _PacedTrackingFake(harness, obs_cost_s=obs_cost_s)

    summary = _run_steps(
        latency,
        client,
        steps=steps,
        control_hz=control_hz,
        settle_timeout_s=settle_timeout_s,
        rate_hz=rate_hz,
        clock=harness.clock,
        sleep=harness.sleep,
    )
    return harness, client, summary


def test_run_tracking_steps_should_rate_limit_the_commands(latency) -> None:
    """R6-1: ``count - 1`` gaps of ``1 / control_hz`` cannot fit into a shorter span.

    The scenario is the one that produced ~87k commands before: a feedback that
    never settles, so the loop would re-issue the goal as fast as the clock lets
    it.  With ``control_hz=50`` and a four-step probe a correct run sends roughly
    ``elapsed * 50 + 2`` commands (the first command of every step and the closing
    hold are the two extras the bound allows); an unpaced run sends thousands.
    ``count > 3`` keeps the assertion from passing on an empty loop.
    """
    control_hz = _CONTROL_HZ
    _, client, summary = _paced_run(latency, control_hz=control_hz, steps=4)

    count = len(client.send_times)
    assert count > 3, f"the probe sent {count} command(s): the assertion would be vacuous"
    elapsed_s = client.send_times[-1] - client.send_times[0]
    assert summary["steps_completed"] == 4, f"the probe aborted: {summary['abort_reason']}"
    assert elapsed_s > 0.0, "the injected sleeper never moved the clock"
    assert count <= elapsed_s * control_hz + 2.0, (
        f"{count} commands in {elapsed_s:.3f} s exceeds --control-hz={control_hz}: "
        f"the minimum interval of {1.0 / control_hz:.3f} s is not enforced"
    )


def test_run_tracking_steps_should_send_more_commands_at_a_higher_control_hz(latency) -> None:
    """R6-2: the knob is wired — a higher ``control_hz`` means more commands.

    Same probe, same feedback, same observation cost; only the command rate
    changes.  A run that ignores ``control_hz`` sends the same number of commands
    in both cases, which is exactly what this comparison rules out.
    """
    _, slow_client, _ = _paced_run(latency, control_hz=25.0, steps=2)
    _, fast_client, _ = _paced_run(latency, control_hz=100.0, steps=2)

    assert len(fast_client.send_times) > len(slow_client.send_times), (
        f"25 Hz sent {len(slow_client.send_times)} command(s) and 100 Hz sent "
        f"{len(fast_client.send_times)}: --control-hz does not reach the loop"
    )


def test_run_tracking_pacing_should_use_the_injected_sleeper(latency) -> None:
    """R6-3: the wait goes through ``sleep``, one command interval at a time.

    The observation rate is set *above* the command rate (100 Hz sampling, 50 Hz
    commanding — the stack is meant to sample faster than it commands), so the only
    pauses the injected sleeper sees are the pacing pauses plus the pre-existing
    ``rate_hz`` period the loop sleeps after a step, and that period is itself
    shorter than one command interval.  Their sum is then ``(count - 1)`` command
    intervals, and no single pause may exceed one interval: a run that slept the
    whole settle budget in one call would keep the average right and the
    responsiveness wrong.  A one-second bound is asserted separately — the robot's
    command watchdog is ~1 s, which is why the settle loop re-issues at all.
    """
    control_hz = _CONTROL_HZ
    interval_s = 1.0 / control_hz
    harness, client, _ = _paced_run(
        latency,
        control_hz=control_hz,
        steps=1,
        obs_cost_s=0.0,
        read_cost_s=1e-6,
        rate_hz=100.0,
    )

    count = len(client.send_times)
    assert count >= 3, f"the probe sent {count} command(s): nothing to measure"
    total_sleep_s = sum(harness.sleeps)
    expected_s = (count - 1) * interval_s
    assert total_sleep_s == pytest.approx(expected_s, abs=2.0 / control_hz), (
        f"{count} commands slept {total_sleep_s:.4f} s in total, expected about "
        f"{expected_s:.4f} s at --control-hz={control_hz}: the pacing does not use the "
        "injected sleeper"
    )
    longest_s = max(harness.sleeps) if harness.sleeps else 0.0
    assert longest_s <= interval_s + 1e-6, (
        f"one sleep lasted {longest_s:.4f} s, more than the {interval_s:.4f} s command "
        "interval: the loop sleeps to the next command instead of in steps"
    )
    assert longest_s <= 1.0, f"one sleep lasted {longest_s:.4f} s: the settle loop is asleep"


@pytest.mark.parametrize("control_hz", ["0", "-1"], ids=["zero", "negative"])
def test_parse_args_should_reject_a_non_positive_control_hz(
    latency, capsys, control_hz: str
) -> None:
    """R6-4: a non-positive command rate is a usage error (2), not a silent run.

    ``--control-hz 0`` would divide the interval by zero; a negative rate has no
    meaning at all.  Same ``parser.error`` shape as ``--rate-hz``: the option name
    reaches stderr so the operator sees which knob was wrong.
    """
    # The option has to exist before its range is worth testing: an *unknown*
    # ``--control-hz`` would also exit 2, but for the wrong reason (argparse calls
    # it an unrecognized argument), so a legal value is parsed first.
    legal = latency.parse_args(["--host", "127.0.0.1", "--control-hz", "50"])
    assert getattr(legal, "control_hz", None) == pytest.approx(50.0)

    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--control-hz={control_hz}"])

    assert excinfo.value.code == 2
    message = capsys.readouterr().err.lower()
    assert "control-hz" in message, f"the error does not name the option: {message!r}"


def test_parse_args_should_default_control_hz_to_50(latency) -> None:
    """R6-5: ``--control-hz`` defaults to 50 and accepts the legal floor ``1``."""
    args = latency.parse_args(["--host", "127.0.0.1"])

    assert hasattr(args, "control_hz"), "--control-hz is missing from the CLI"
    assert args.control_hz == pytest.approx(50.0)

    floor = latency.parse_args(["--host", "127.0.0.1", "--control-hz", "1"])
    assert floor.control_hz == pytest.approx(1.0)
    # Independent of the observation rate: the two knobs never overwrite each other.
    assert floor.rate_hz == pytest.approx(30.0)


def test_run_tracking_steps_should_report_the_command_and_sample_rates(latency) -> None:
    """R6-6: the summary states both achieved rates, and they are finite.

    ``achieved_command_hz`` is how often the goal was actually written and
    ``achieved_rate_hz`` how often a frame was actually read; a summary that only
    echoes ``--rate-hz`` cannot show that pacing worked.  The command rate is
    compared against ``--control-hz`` within a factor of two (the hold and the
    step head are the two commands the ratio does not account for), which is what
    makes the keys evidence instead of decoration.
    """
    control_hz = _CONTROL_HZ
    _, _, summary = _paced_run(latency, control_hz=control_hz, steps=1)

    for key in ("achieved_command_hz", "achieved_rate_hz"):
        assert key in summary, f"the tracking summary does not report {key}: {sorted(summary)}"
        assert _is_finite_number(summary[key]), f"{key} is not a finite number: {summary[key]!r}"
        assert summary[key] > 0.0, f"{key} is not positive: {summary[key]!r}"
    assert 0.5 * control_hz <= summary["achieved_command_hz"] <= 2.0 * control_hz, (
        f"achieved_command_hz={summary['achieved_command_hz']!r} does not reflect --control-hz={control_hz}"
    )


def test_main_should_print_the_achieved_command_and_sample_rates(latency, tmp_path, capsys) -> None:
    """R6-6 (report level): a tracking report names both achieved rates."""
    client = _TrackingFake()
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--settle-timeout-s",
            "0.05",
            "--rate-hz",
            "200",
            "--control-hz",
            "50",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    printed = capsys.readouterr()
    lowered = printed.out.lower()

    assert code == latency.EXIT_OK, f"the tracking run failed with {code}: {printed.err!r}"
    assert "achieved_command_hz" in lowered, f"the report omits the command rate: {printed.out!r}"
    assert "achieved_rate_hz" in lowered, f"the report omits the sample rate: {printed.out!r}"


# ===========================================================================
# R. The two holes the round-6 limit-speed review closed, kept closed
# ===========================================================================
#
# Both holes were "a guard that looked right and let the bad value through":
#
# R-1  ``--control-hz`` was checked with ``x >= 1.0`` only.  ``inf`` satisfies
#      that (``1 / inf == 0.0``), and so does ``1e400`` because argparse parses
#      it to ``inf``; ``nan`` satisfies *nothing*, so a plain ``x < 1.0`` check
#      passes it too.  Either way the pacing interval became ``0.0`` and the
#      unthrottled command burst the pacer exists to prevent came back.
# R-2  ``run_tracking_steps`` had no entry check at all, so a direct caller
#      (or a future one) could hand it ``inf`` / ``nan`` / ``0`` / a negative
#      rate and get an unthrottled loop, a ``ZeroDivisionError`` or a nonsense
#      interval instead of a loud refusal.
# R-3  ``rise_90`` can be structurally empty when the settle tolerance is at or
#      above ``0.1 * |delta|`` (the loop's break is looser than the crossing
#      test).  That is a property of the knobs, not a failed measurement, so the
#      tracking banner states it and the report's ``count=0`` hint points back
#      at it.  Neither may be silent, and the banner must not fire for a
#      tolerance that does allow the crossing.
#
# Every case below is a *targeted* assertion on one behaviour; none of them
# waits on real time (the CLI runs use ``--settle-timeout-s 0.05``) and every
# artefact goes to ``tmp_path``.


@pytest.mark.parametrize(
    "control_hz",
    ["nan", "inf", "1e400", "-inf"],
    ids=["nan", "inf", "overflow_to_inf", "negative_inf"],
)
def test_parse_args_should_reject_a_non_finite_control_hz(latency, capsys, control_hz: str) -> None:
    """R-1: a non-finite ``--control-hz`` is a usage error (2), never a run.

    ``inf`` is ``>= 1.0`` and ``1 / inf == 0.0``; ``1e400`` *is* ``inf`` after
    ``float()``; ``nan`` makes every comparison false.  Each of the four would
    hand the pacer a zero or nonsense interval, i.e. no throttling at all.

    The legal value is parsed first on purpose: an *unknown* option (or a
    mistyped one) also exits 2, so without this guard the test would go green on
    a ``--control-hz`` that does not exist any more.
    """
    # Arrange: the option exists and a legal value reaches the namespace.
    legal = latency.parse_args(["--host", "127.0.0.1", "--control-hz", "50"])
    assert getattr(legal, "control_hz", None) == pytest.approx(50.0), (
        "--control-hz 50 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    # Act + Assert: the non-finite value is refused by the parser itself.
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--control-hz={control_hz}"])

    assert excinfo.value.code == 2, f"--control-hz={control_hz} exited with {excinfo.value.code}"
    message = capsys.readouterr().err.lower()
    assert "control-hz" in message, f"the error does not name the option: {message!r}"
    assert "finite" in message, f"the error does not explain the reason: {message!r}"


@pytest.mark.parametrize(
    "control_hz",
    [float("inf"), float("nan"), 0.0, -50.0],
    ids=["inf", "nan", "zero", "negative"],
)
def test_run_tracking_steps_should_reject_a_non_finite_control_hz(
    latency, control_hz: float
) -> None:
    """R-2: the direct-caller entry point refuses before reading or commanding.

    The CLI guard covers the operator; this one covers every other caller.  The
    refusal must be an exception with a reason (``ValueError`` naming
    ``finite``), not a silent run, and it must happen *before* the first
    feedback read: the fake's counters are the evidence.
    """
    # Arrange: a fake that would happily accept a whole probe.
    client = _TrackingFake()

    # Act + Assert: the entry check fires with the contract message.
    with pytest.raises(ValueError, match="finite"):
        _run_steps(latency, client, control_hz=control_hz)

    # Nothing was read and nothing was written: the check is an entry check.
    assert client.sent_actions == [], (
        f"control_hz={control_hz!r} still commanded {client.sent_actions!r}"
    )
    assert client.get_observation_calls == 0, (
        f"control_hz={control_hz!r} read {client.get_observation_calls} frame(s) before refusing"
    )


def _banner_tolerance_note(stdout: str) -> list[str]:
    """Return the banner lines that warn the tolerance hides the 90 % crossing.

    The report's ``count=0`` hint quotes the same two words (``90 %`` and
    ``settle tolerance``), so the keyword pair alone cannot tell the two apart.
    The banner sentence is identified by its own opening *and* by the
    ``>= 0.1 * delta`` half-sentence, which only the banner carries (the hint
    says ``at or below 0.1 * delta``).
    """
    return [
        line
        for line in stdout.lower().splitlines()
        if line.startswith("note: settle tolerance") and ">= 0.1 * delta" in line
    ]


def _tracking_argv(out: Path, *, delta_deg: str, settle_tolerance_deg: str) -> list[str]:
    """A complete tracking run whose only varying knobs are the two settlings."""
    return [
        "--host",
        "127.0.0.1",
        "--mode",
        "tracking",
        "--enable-motion",
        "--joint",
        "arm_left_shoulder_pan",
        "--steps",
        "1",
        "--settle-timeout-s",
        "0.05",
        "--rate-hz",
        "200",
        "--control-hz",
        "50",
        "--delta-deg",
        delta_deg,
        "--settle-tolerance-deg",
        settle_tolerance_deg,
        "--csv",
        str(out),
    ]


def test_tracking_banner_should_warn_that_the_tolerance_hides_the_crossing(
    latency, tmp_path, capsys
) -> None:
    """R-3a: a tolerance at or above ``0.1 * delta`` is stated in the banner.

    ``--delta-deg 1.0`` books the crossing at a residual of ``0.1 deg`` while the
    settle loop breaks at ``0.15 deg``, so ``rise_90`` may never be observed.
    The operator has to learn that from the banner, before the run, with the
    relation that explains it.
    """
    # Arrange: the tolerance is the looser of the two judgements.
    out = tmp_path / "tracking.csv"

    # Act: a full tracking run through ``main`` with a static fake feedback.
    code = latency.main(
        _tracking_argv(out, delta_deg="1.0", settle_tolerance_deg="0.15"),
        client_factory=_Factory(_TrackingFake()),
    )
    printed = capsys.readouterr()

    # Assert: the run reached the report and the banner carries the warning.
    assert code == latency.EXIT_OK, f"the tracking run failed with {code}: {printed.err!r}"
    notes = _banner_tolerance_note(printed.out)
    assert len(notes) == 1, f"expected exactly one banner note, found {notes!r} in {printed.out!r}"
    note = notes[0]
    assert "90 %" in note, f"the note does not name the crossing: {note!r}"
    assert "settle tolerance" in note, f"the note does not name the tolerance: {note!r}"
    assert "rise_90" in note, f"the note does not name the empty column: {note!r}"


def test_tracking_banner_should_stay_quiet_when_the_tolerance_allows_the_crossing(
    latency, tmp_path, capsys
) -> None:
    """R-3b: a tolerance below ``0.1 * delta`` must not trigger the banner note.

    Negative control for R-3a.  The banner note and the report's ``count=0``
    *hint* share the words ``90 %`` and ``settle tolerance``, and this run still
    prints the hint (the fake's feedback never moves, so no frame crosses): the
    assertion therefore targets the banner sentence only, and the presence of
    the hint is what proves the keywords alone would not have distinguished the
    two.
    """
    # Arrange: 0.05 deg < 0.1 * 1.0 deg, so the crossing stays observable.
    out = tmp_path / "tracking.csv"

    # Act
    code = latency.main(
        _tracking_argv(out, delta_deg="1.0", settle_tolerance_deg="0.05"),
        client_factory=_Factory(_TrackingFake()),
    )
    printed = capsys.readouterr()
    lowered = printed.out.lower()

    # Assert: no banner warning ...
    assert code == latency.EXIT_OK, f"the tracking run failed with {code}: {printed.err!r}"
    notes = _banner_tolerance_note(printed.out)
    assert notes == [], f"the banner warns although the tolerance allows the crossing: {notes!r}"
    assert ">= 0.1 * delta" not in lowered, (
        f"the banner note leaked into the output: {printed.out!r}"
    )
    # ... while the count=0 hint is still there, so the keywords of the banner
    # note are present in this very output and their absence is meaningful.
    assert "count=0" in lowered, (
        f"the report printed no hint, so the negative control is vacuous: {printed.out!r}"
    )
    assert "90 %" in lowered, f"the hint no longer quotes the crossing: {printed.out!r}"
    assert "settle tolerance" in lowered, (
        f"the hint no longer quotes the tolerance: {printed.out!r}"
    )


# ===========================================================================
# R. Client capability probe: a vendor without per-request timing is refused
# ===========================================================================
#
# Round 7.  The deployed vendor client (``AlohaMiniClient`` on ``main``) has no
# ``latest_request_timing`` — only a semantic and structural neighbour,
# ``latest_safety_status``.  The rtt branch below read the timings face
# directly, so on that client the measurement died with ``AttributeError`` at
# the first sampled frame instead of saying what was wrong.
#
# Ruling of this round: probe the capability **explicitly**, before the sampling
# loop, and abort with ``EXIT_CLIENT_UNSUPPORTED == 9`` when it is absent.
# ``getattr(client, ..., None)`` is forbidden here: a silent ``None`` would turn
# every timing cell into an empty/``NaN`` value and the CSV would look like a
# measurement.  A refusal is the only honest answer.


class _ClientWithoutRequestTiming(_FakeClient):
    """The vendor client as it ships today: ``latest_request_timing`` is absent.

    ``_FakeClient`` publishes the timing dict in ``__init__`` *and* on every
    ``get_observation``, which is exactly what hid the gap — the attribute always
    existed.  This subclass removes it on both paths, so ``hasattr`` answers the
    way it does on the deployed client.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        del self.latest_request_timing

    def get_observation(self, include_cameras: bool = True) -> dict:
        state = super().get_observation(include_cameras=include_cameras)
        # ``_FakeClient.get_observation`` re-creates the attribute; a client that
        # never publishes per-request timing does not.
        del self.latest_request_timing
        return state


def test_client_capabilities_should_declare_only_the_request_timing_face(latency) -> None:
    """R1: rtt mode needs exactly one timing face, and says so in one place."""
    assert latency.CLIENT_CAPABILITIES == ("latest_request_timing",)


def test_missing_client_capabilities_should_be_empty_when_the_face_exists(latency) -> None:
    """R2: a client that publishes per-request timing is complete."""
    client = _FakeClient()

    assert latency.missing_client_capabilities(client) == []


def test_missing_client_capabilities_should_name_the_absent_face(latency) -> None:
    """R3: the vendor client is reported by name, so the message can name it."""
    client = _ClientWithoutRequestTiming()

    assert latency.missing_client_capabilities(client) == ["latest_request_timing"]


def test_missing_client_capabilities_should_follow_the_requested_name_order(latency) -> None:
    """R4: the result follows ``names``, not alphabetical or set order.

    ``latest_control_status`` sorts *before* ``latest_request_timing``, so a
    sorted or set-derived result would come back reversed here.
    """
    client = _ClientWithoutRequestTiming()
    del client.latest_control_status

    missing = latency.missing_client_capabilities(
        client, names=("latest_request_timing", "latest_control_status")
    )

    assert missing == ["latest_request_timing", "latest_control_status"]


def test_module_constants_should_include_client_unsupported(latency) -> None:
    """R5: an unsupported client is its own exit code, not a connect failure."""
    assert getattr(latency, "EXIT_CLIENT_UNSUPPORTED", None) == 9


def test_main_should_refuse_rtt_before_sampling_when_the_face_is_absent(
    latency, tmp_path, capsys
) -> None:
    """R6: no timing face → exit 9, the attribute named on stderr, and no CSV.

    The refusal happens *before* the sampling loop (zero observations) and writes
    nothing at all, not even a header: a CSV with empty timing columns reads like
    a measurement, which is the failure mode this round locks out.

    The refused run still owns the socket it opened, so it must hand it back: a
    refusal that returns without ``disconnect()`` leaks the connection for the
    rest of the process.

    Ordering, locked here: the probe runs **before** ``_prime_state_only_window``.
    The prime is the harness's first **state-only** request batch (``connect()``
    has already filled the window with camera requests), and the deployed vendor
    ``main`` client does implement the prime, so a probe placed after it would
    refuse only *after* spending a whole request window and would falsify the
    exit-code documentation ("before the state-only window prime sends its first
    request").  ``prime_calls == []`` is what makes that order observable: with
    the probe moved back after the prime, the refusal already costs one prime and
    this assertion turns red.
    """
    client = _ClientWithoutRequestTiming()
    out = tmp_path / "x.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "5",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == 9, f"a client without per-request timing must exit 9, got {code}"
    assert "latest_request_timing" in captured.err, (
        f"stderr does not name the missing attribute: {captured.err!r}"
    )
    assert "vendor" in captured.err.lower(), (
        f"stderr does not say the vendor client is the cause: {captured.err!r}"
    )
    assert client.get_observation_calls == 0, (
        f"the probe sampled {client.get_observation_calls} frame(s) before refusing"
    )
    assert not out.exists(), f"a refused run wrote {out}"
    assert client.prime_calls == [], (
        "the exit 9 refusal came after the state-only window prime, so it already "
        f"sent {len(client.prime_calls)} request(s): {client.prime_calls}"
    )
    assert client.is_connected is False, (
        "the exit 9 refusal returned without closing the client it had connected"
    )


def test_main_should_still_write_the_full_rtt_csv_when_the_face_exists(latency, tmp_path) -> None:
    """R7: the supported path is unchanged — 18 columns, every key cell filled."""
    client = _FakeClient(ages=[0.030, 0.031, 0.032])
    out = tmp_path / "rtt.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK
    assert out.is_file()
    with out.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        rows = list(reader)
    assert len(fieldnames) == 18, f"the rtt header changed shape: {fieldnames}"
    assert len(rows) == 3
    for row in rows:
        for field in (
            "token",
            "sent_monotonic_s",
            "received_monotonic_s",
            "age_s",
            "net_rtt_est_s",
        ):
            assert row[field] != "", f"row {row['sample_index']} has an empty {field}: {row}"


# ===========================================================================
# S. Ordering: the state-only window prime still happens on the tracking path
# ===========================================================================
#
# Section R moved the capability probe *ahead* of
# ``_prime_state_only_window(client)``.  The direction of that change is easy to
# over-apply: deleting the prime, or moving it into the rtt branch only, would
# leave a tracking run sampling a window that ``connect()`` filled with camera
# requests — the JPEG encoding work of those frames then shows up in the first
# rows and the prime's whole purpose is gone.  The test below pins the prime on
# the tracking path and pins its flag to state-only.


class _PrimedTrackingFake(_TrackingFake):
    """``_TrackingFake`` plus the window prime the real client exposes.

    ``_TrackingFake`` deliberately has no ``prime_observation_request_window``
    (the script's ``getattr`` guard is what makes that legal), so this subclass
    adds the recorder the ordering assertion needs and nothing else.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.prime_calls: list[bool] = []

    def prime_observation_request_window(self, *, include_cameras: bool) -> None:
        self.prime_calls.append(include_cameras)


def test_main_should_prime_the_state_only_window_in_tracking_mode(
    latency, tmp_path, capsys
) -> None:
    """``--mode tracking`` primes once, with ``include_cameras=False``.

    The run is the shortest legal one (one step, a 50 ms settle budget) and its
    outcome is asserted only so a refusal cannot pass for a primed run.
    """
    client = _PrimedTrackingFake()
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            "arm_left_shoulder_pan",
            "--steps",
            "1",
            "--settle-timeout-s",
            "0.05",
            "--rate-hz",
            "200",
            "--control-hz",
            "50",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the tracking run failed with {code}: {printed.err!r}"
    assert len(client.prime_calls) == 1, (
        f"the tracking run primed the window {len(client.prime_calls)} time(s): "
        f"{client.prime_calls}"
    )
    assert client.prime_calls[0] is False, (
        f"the tracking run primed with include_cameras={client.prime_calls[0]!r}, "
        "so the camera frames the prime exists to drop would still be sampled"
    )


# ===========================================================================
# T. Capability, second probe: the timing face exists but is never filled
# ===========================================================================
#
# Section R asked ``hasattr``: does the client publish ``latest_request_timing``?
# That question has a blind spot.  A client can publish the attribute and never
# put anything in it -- ``{}`` in ``__init__`` and nothing afterwards, a
# half-implemented vendor build, or a frame the host did not time.
# ``_request_row`` reads it as ``dict(client.latest_request_timing or {})``
# (约 benchmarks/alohamini_io/measure_latency.py:1360), so an always-empty dict
# becomes an all-empty timing row: ``""``/``NaN`` cells that read like a
# measurement of zero.  So the capability probe needs a second question next to
# ``missing_client_capabilities`` (约 measure_latency.py:964): *is the face it found
# actually filled?*  ``missing_client_capabilities`` itself stays a pure
# ``hasattr`` check -- R2-R4 freeze that -- and the second probe is pinned below
# from the outside, through ``main()``.

# The words that mark the *empty* refusal.  None of them occurs in the
# absent-attribute message of section R ("publishes no latest_request_timing",
# "reports no per-request timing"), which is what makes the two refusals
# tellable apart on stderr alone.  Checked against the lowered stderr, so the
# implementation may capitalise them.  Permissive by design: today's message
# says "never fills it", so only ``empty`` matches -- the other three entries are
# tolerated spellings, not words the implementation promises to print.
_EMPTY_TIMING_WORDS = ("empty", "never filled", "not filled", "no timing")


class _ClientWithEmptyTiming(_FakeClient):
    """The timing face exists, but every value it ever holds is ``{}``.

    Same shape as ``_ClientWithoutRequestTiming`` one section up, with the
    opposite defect: ``get_observation`` re-creates ``latest_request_timing`` on
    every frame (as ``_FakeClient`` does) and then empties it again, so
    ``hasattr`` answers ``True`` for the whole run while the timing face never
    carries a token, a stamp or an age.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.latest_request_timing = {}

    def get_observation(self, include_cameras: bool = True) -> dict:
        state = super().get_observation(include_cameras=include_cameras)
        self.latest_request_timing = {}
        return state


def test_main_should_refuse_rtt_when_the_timing_face_is_never_filled(
    latency, tmp_path, capsys
) -> None:
    """R8: the attribute exists but stays ``{}`` → exit 9, no CSV.

    Locked assertions, and why each one is here:

    * exit ``EXIT_CLIENT_UNSUPPORTED`` (9), not 0: at the time of writing the run
      reports success and writes a CSV whose ``token`` / ``sent_monotonic_s`` /
      ``received_monotonic_s`` cells are empty and whose ``age_s`` is ``NaN`` --
      a file that reads like a measurement.
    * stderr names ``latest_request_timing``: the operator has to know *which*
      face is at fault.
    * stderr also says the face is empty, i.e. contains one of
      ``_EMPTY_TIMING_WORDS``.  That tuple is a *permissive* matcher -- four
      tolerated spellings, not four promised words: today's message reads
      "publishes ... but never fills it", so only ``empty`` from the tuple
      matches it, and ``never filled`` / ``not filled`` / ``no timing`` do not.
      What is frozen is that the empty refusal and the absent-attribute refusal
      have to be tellable apart on stderr alone; the exact wording is the
      implementation's choice, and R10 below locks the negative side of it.
      "The client has no such attribute" and "the client has it but nothing is in
      it" have different fixes (upgrade the vendor client vs. find out why the
      host stopped timing requests).
    * the CSV path does not exist: a refused run writes nothing, not even a
      header.

    Deliberately *not* asserted: the number of ``get_observation`` calls.  A
    probe that decides "filled?" may legitimately take one frame first (the
    absent-attribute probe could decide without sampling, this one cannot), so
    freezing the count here would forbid a correct implementation.
    """
    client = _ClientWithEmptyTiming()
    out = tmp_path / "empty-timing.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "5",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"a client whose timing face is never filled must exit 9, got {code}; "
        f"stderr: {captured.err!r}"
    )
    assert "latest_request_timing" in captured.err, (
        f"stderr does not name the empty attribute: {captured.err!r}"
    )
    lowered = captured.err.lower()
    assert any(word in lowered for word in _EMPTY_TIMING_WORDS), (
        "stderr does not say the timing face is empty / never filled, so it cannot be "
        f"told apart from the absent-attribute refusal (looked for {_EMPTY_TIMING_WORDS}): "
        f"{captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"


def test_main_should_measure_rtt_when_the_timing_face_is_filled(latency, tmp_path) -> None:
    """R9: the filled face keeps measuring -- the second probe must not refuse it.

    Named explicitly (``_FakeClient`` fills ``latest_request_timing`` on every
    ``get_observation``) so that R8/R9 read as one rule: refuse the empty face,
    keep measuring the filled one.  The row count equals ``--samples`` and the
    ``token`` / ``age_s`` cells are non-empty -- those are exactly the cells the
    empty face leaves blank.
    """
    client = _FakeClient(ages=[0.030, 0.031, 0.032, 0.033])
    out = tmp_path / "filled-timing.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "4",
            "--rate-hz",
            "200",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, (
        f"a client that fills its timing face must still be measurable, got {code}"
    )
    assert out.is_file(), f"the rtt run wrote no CSV at {out}"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 4, f"expected one row per requested sample, got {len(rows)}"
    for row in rows:
        assert row["token"] != "", f"row {row['sample_index']} has an empty token: {row}"
        assert row["age_s"] != "", f"row {row['sample_index']} has an empty age_s: {row}"


def test_main_should_refuse_rtt_with_the_absent_wording_when_the_face_is_missing(
    latency, tmp_path, capsys
) -> None:
    """R10: the absent-attribute refusal is unchanged, and it is not the empty one.

    R6 already locks this path end to end (exit 9, attribute named, zero
    observations, no CSV, no window prime); this test adds the one thing R6 could
    not state before R8 existed: the two refusals are mutually exclusive on
    stderr.  Without that, an implementation could satisfy R8 by reusing the
    "publishes no ..." wording for both cases and leave the operator unable to
    tell "upgrade the client" from "the host stopped timing requests".
    """
    client = _ClientWithoutRequestTiming()
    out = tmp_path / "absent-timing.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "5",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"the absent timing face still exits 9, got {code}: {captured.err!r}"
    )
    assert "latest_request_timing" in captured.err, (
        f"stderr no longer names the absent attribute: {captured.err!r}"
    )
    lowered = captured.err.lower()
    assert not any(word in lowered for word in _EMPTY_TIMING_WORDS), (
        "the absent-attribute refusal uses the empty-timing wording, so the two "
        f"defects are no longer distinguishable on stderr: {captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"


# ---------------------------------------------------------------------------
# R11-R18: the second probe runs on *every* growth, and it is a pure function
# ---------------------------------------------------------------------------
#
# R8 pins the *always*-empty face, and one implementation can satisfy it with a
# one-shot latch: keep a ``probed`` flag in ``collect_rtt_samples``, call
# ``unfilled_client_capabilities`` at the first growth of
# ``observation_sequence`` and never look at the timing face again.  That latch
# has two blind spots, and both are frozen below.
#
# * A face that is filled at the first frame and empties afterwards is never
#   noticed (R11): the run reports success and writes rows whose ``token``,
#   ``sent_monotonic_s`` and ``received_monotonic_s`` cells are empty and whose
#   ``age_s`` cell is empty too.  That is the same "empty cells read like a
#   measurement" file the second probe exists to prevent, only from row 3 on.
# * A face that is empty on the first frame and filled from the second one on is
#   refused, although the client is measurable from frame 2 (R12).
#
# Ruling of this round: probe strictly, on every growth and before any row is
# appended, so a client can never produce a row with an empty timing face.  R11
# follows from that, and R12 keeps the reading that comes with it -- a client
# whose first frame is untimed is refused too.  That is an *intentional strict
# trade-off* (refusing a possibly legal client beats writing empty timing rows),
# not a defect: R12 locks it as such.  Whether a real vendor client has the R12
# shape (``{}`` on the frame right after ``connect()``, filled afterwards) is
# unverified -- no field run has produced it, see ``docs/s4-field-smoke.md``.


def test_main_should_refuse_rtt_when_the_timing_face_empties_mid_run(
    latency, tmp_path, capsys
) -> None:
    """R11: frames 1-2 timed, frame 3 onwards ``{}`` → exit 9, and no CSV.

    Locks exactly one rule: **no row may be written with an empty timing face**.
    The one-shot probe breaks it -- it latches at the first growth, sees a filled
    face and closes satisfied -- so the run goes on to append rows from the
    untimed frames, exits 0 and writes a CSV whose later rows carry an empty
    ``token``, empty ``sent_monotonic_s`` / ``received_monotonic_s`` and an empty
    ``age_s`` cell.  Those are precisely the cells that read like a measurement
    of zero.

    The refusal must happen before the row is appended, so the CSV must not exist
    at all: an exit 9 that still wrote the two timed rows would leave the
    "looks like a measurement" file this contract forbids.

    Deliberately *not* asserted: the number of ``get_observation`` calls (a probe
    may read a frame before deciding, as R8 states) and the row count.
    """
    # Frames 1 and 2 (calls 0 and 1) fill ``latest_request_timing``; from frame
    # 3 (call 2) it is ``{}`` for the rest of the run.
    client = _FakeClient(empty_timing_from=2)
    out = tmp_path / "mid-run-empty.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "5",
            "--rate-hz",
            "200",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"a timing face that empties mid-run must exit 9, got {code}; stderr: {captured.err!r}"
    )
    assert "latest_request_timing" in captured.err, (
        f"stderr does not name the empty attribute: {captured.err!r}"
    )
    assert not out.exists(), (
        f"a run refused for an empty timing face still wrote {out}; no row may be "
        "written with an empty timing face, not even the timed rows before it"
    )


def test_main_should_still_refuse_rtt_when_the_timing_face_fills_from_the_second_frame(
    latency, tmp_path, capsys
) -> None:
    """R12: first frame ``{}``, filled afterwards → still exit 9 (intended).

    This one is a *strict trade-off*, not a defect, and it is locked as such.  A
    per-growth probe runs at the first growth of ``observation_sequence``, which
    is also the frame the client has just answered; if that frame carries no
    timing, the run is refused -- even when every later frame is timed and the
    measurement would have been fine from row 2 on.

    Why accept that cost: the probe cannot tell "this host never times a request"
    from "this host had not timed it yet" on the frame it is looking at.  Note
    what the grace alternative (wait one more growth, then decide) would *not*
    cost: the probe runs at every growth, so it does not reopen the R11 hole -- a
    face that empties later in the run is still refused on the frame it empties
    on.  Its real price is one row: grace on the first growth would let that
    frame append a row whose ``token``, stamps and ``age_s`` are empty, i.e.
    exactly the line R11 forbids.  Refusing at the first growth keeps "no row is
    ever written from an empty face" absolute, and the price is refusing a
    possibly legal slow-fill client (one re-run); the grace alternative would
    write a CSV of empty cells that reads like a measurement.  The refusal is the
    honest side of that asymmetry.

    Unverified (do not read this test as a claim about hardware): whether the
    real vendor client can even take this shape -- ``latest_request_timing == {}``
    on the first frame after ``connect()`` and filled from the second frame on --
    has not been observed.  If a field run ever produces it, the trade-off itself
    has to be re-argued (a bounded grace of one frame plus a hard refusal after
    it would be the candidate), and this test is the one that has to change.
    """
    # Call 0 leaves ``latest_request_timing`` as ``{}``; from call 1 it is timed.
    client = _FakeClient(empty_timing_until=1)
    out = tmp_path / "late-filled.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "5",
            "--rate-hz",
            "200",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        "a first frame without timing is refused on purpose (strict per-growth "
        f"probe), got exit {code}; stderr: {captured.err!r}"
    )
    assert "latest_request_timing" in captured.err, (
        f"stderr does not name the empty attribute: {captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"


def test_main_should_close_the_client_when_the_timing_face_empties_mid_run(
    latency, tmp_path, capsys
) -> None:
    """R13: the mid-run-empty refusal hands the socket back, as R6 requires.

    Same run as R11, same shape of assertion as R6: the refusal path must not
    leave the connected client open, or the port stays taken for the rest of the
    process.  Kept separate from R11 so that a leak shows up as one failing test
    about the leak instead of a failing refusal.
    """
    client = _FakeClient(empty_timing_from=2)
    out = tmp_path / "leak.csv"

    latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "5",
            "--rate-hz",
            "200",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    capsys.readouterr()

    assert client.get_observation_calls > 0, (
        "the run never sampled a frame, so it did not reach the per-growth probe"
    )
    assert client.is_connected is False, (
        "the exit 9 refusal for an empty timing face returned without closing the "
        "client it had connected"
    )


def test_main_should_not_use_the_absent_wording_when_the_timing_face_is_empty(
    latency, tmp_path, capsys
) -> None:
    """R14: the empty-timing refusal never reuses ``publishes no``.

    R10 locks one direction of the mutual exclusion (the absent-attribute refusal
    avoids the empty-timing wording); this is the other one, and it is the one a
    lazier implementation breaks: it can satisfy R8/R11 by printing the same
    "publishes no latest_request_timing" sentence for both defects, which leaves
    the operator unable to tell "upgrade the client" from "find out why the host
    stopped timing requests".

    ``publishes no`` is the word chosen because it is the stable half of the
    absent-attribute sentence in ``main`` ("unsupported client: it publishes no
    latest_request_timing, so rtt cannot be measured."): it states that the face
    is *not there*, and no honest empty-face message can claim that.  The other
    words a reader might reach for are rejected on purpose -- the two refusals
    legitimately share boilerplate, so they cannot separate anything:

    * ``so rtt cannot be measured.`` -- both refusals say it, correctly;
    * ``no CSV was written`` / ``no sample was recorded`` -- both say it;
    * ``latest_request_timing`` -- both must name the face, R8/R11 require it;
    * ``unsupported client`` -- both open with it, the exit code is shared.

    Asserting on any of those would turn a harmless wording change into a red
    and would say nothing about the defect the operator has to identify.
    """
    client = _ClientWithEmptyTiming()
    out = tmp_path / "empty-wording.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "5",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"the empty-timing refusal still exits 9, got {code}: {captured.err!r}"
    )
    assert "publishes no" not in captured.err.lower(), (
        "the empty-timing refusal reuses the absent-attribute wording ('publishes "
        "no'), so an operator can no longer tell a missing face from an unfilled "
        f"one: {captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"


# --- R15-R18: the second probe is a pure function --------------------------
#
# Symmetric to R2-R4 for ``missing_client_capabilities``.  Signature checked
# against the source before writing (``unfilled_client_capabilities`` at
# 约 benchmarks/alohamini_io/measure_latency.py:1052, signature
# ``(client: Any, names: Sequence[str] = CLIENT_CAPABILITIES) -> list[str]``,
# body ``[name for name in names if not getattr(client, name, None)]``): the
# ordering test therefore has to pass ``names`` in a non-alphabetical order, and
# the purity tests have to get an empty list back only for a face that carries
# something.


def test_unfilled_client_capabilities_should_be_empty_when_the_face_is_filled(
    latency,
) -> None:
    """R15: a filled timing face is not reported, so the probe cannot refuse it.

    ``_FakeClient`` publishes ``{}`` in ``__init__`` and fills the face on every
    ``get_observation``, so one frame is taken first: that is exactly the state
    the sampling loop probes in.
    """
    client = _FakeClient()
    client.get_observation()

    assert latency.unfilled_client_capabilities(client) == []


def test_unfilled_client_capabilities_should_name_the_face_that_stays_empty(
    latency,
) -> None:
    """R16: an always-``{}`` face is reported by name (the R8 defect, at unit level)."""
    client = _ClientWithEmptyTiming()
    client.get_observation()

    assert latency.unfilled_client_capabilities(client) == ["latest_request_timing"]


def test_unfilled_client_capabilities_should_follow_the_requested_name_order(
    latency,
) -> None:
    """R17: the result follows ``names``, not alphabetical or set order.

    ``latest_control_status`` sorts *before* ``latest_request_timing``, so a
    sorted or set-derived result would come back reversed here.  Both faces are
    empty: the subclass empties the timing face on every frame, and
    ``latest_control_status`` is set to ``{}`` so that ``None`` (its
    ``_FakeClient`` default) is not what makes it count.
    """
    client = _ClientWithEmptyTiming()
    client.latest_control_status = {}

    unfilled = latency.unfilled_client_capabilities(
        client, names=("latest_request_timing", "latest_control_status")
    )

    assert unfilled == ["latest_request_timing", "latest_control_status"]


def test_unfilled_client_capabilities_should_not_touch_the_client(latency) -> None:
    """R18: the probe is pure -- it reads the face, it neither samples nor mutates it.

    Pinned because the caller places it inside the sampling loop: a probe that
    took a frame itself (or cleared the face to decide) would add a row, skip a
    frame or change what ``_request_row`` writes.
    """
    client = _ClientWithEmptyTiming()
    timing_before = client.latest_request_timing
    calls_before = client.get_observation_calls

    unfilled = latency.unfilled_client_capabilities(client)

    assert unfilled == ["latest_request_timing"]
    assert client.latest_request_timing == timing_before, (
        "the probe changed the timing face it was asked to inspect"
    )
    assert client.latest_request_timing == {}
    assert client.get_observation_calls == calls_before, (
        f"the probe sampled {client.get_observation_calls - calls_before} frame(s)"
    )


# --- R19: the probe sits ahead of the row append ---------------------------
#
# R8/R11-R13 observe the refusal through ``main``: exit 9 and no CSV.  Those
# assertions cannot see *where in the loop* the refusal happens, because a probe
# moved behind ``rows.append(...)`` still raises before ``write_csv`` and still
# leaves no file behind on disk.  The position is what makes the CSV provable --
# a row that is appended and then thrown away was still built from an untimed
# frame -- so it is pinned here at the unit level, on the rows list itself.


def test_collect_rtt_samples_should_append_no_row_for_the_frame_it_refuses(latency) -> None:
    """R19: an untimed frame never enters ``rows``, not even one that is never written.

    Locks the *position* of the second capability probe, not its existence: it
    has to run after a growth of ``observation_sequence`` and **before** that
    frame's ``rows.append(...)``.  Moving it one statement later still raises
    ``UnfilledClientCapability`` and still lets ``main`` return
    ``EXIT_CLIENT_UNSUPPORTED`` with no CSV on disk, so the end-to-end tests
    (R8/R11-R13) cannot tell the two orderings apart; the only observable that
    can is the caller-supplied ``rows`` list.

    The face is filled on the first two answered frames and ``{}`` from the third
    one on (``_FakeClient(empty_timing_from=2)``), i.e. the mid-run shape -- so
    the two rows of the timed frames must be in the list and the row of the frame
    that triggered the refusal must not be, whatever happens to it afterwards.

    Why the position matters to the CSV: only this ordering makes "the CSV holds
    no empty-timing row" provable instead of incidental.  Appended-then-discarded
    rows are rows that were already built out of an untimed frame.
    """
    # Calls 0 and 1 time the request; from call 2 the face is ``{}`` for good.
    client = _FakeClient(empty_timing_from=2)
    rows: list[dict[str, Any]] = []

    with pytest.raises(latency.UnfilledClientCapability) as excinfo:
        latency.collect_rtt_samples(
            client,
            samples=5,
            rate_hz=200.0,
            timeout_s=5.0,
            sleep=_fast_sleep,
            clock=_FakeClock(start=0.0, step_s=0.001),
            rows_out=rows,
        )

    # Row count is the assertion: two timed growths, zero rows for the refused one.
    assert len(rows) == 2, (
        f"the refused frame left {len(rows) - 2} extra row(s) in rows: the probe ran "
        "after rows.append(...), so a row built from the untimed frame is already in "
        "the list -- the CSV can no longer be proven free of empty-timing rows"
    )
    assert all(row["token"] for row in rows), (
        "an untimed frame reached rows: the row appended there carries an empty "
        f"token/age cell -- {[sorted(row) for row in rows]}"
    )
    # The refusal stays readable: it names the face the caller has to look at.
    assert excinfo.value.names == ("latest_request_timing",), (
        f"the refusal does not name the unfilled face: {excinfo.value.names!r}"
    )


# ===========================================================================
# U. Gate F, second segment: v2 schema, right model, broken conversion fields
# ===========================================================================
#
# Gate F has two segments in ``_tracking_preflight``
# (约 benchmarks/alohamini_io/measure_latency.py:1779): the schema check
# (``schema_version == 2``, 约 measure_latency.py:1849) and the unit check
# ``deg_per_unit(metadata, joint_key)`` (约 measure_latency.py:1859, raising
# ``UnitConversionUnavailable`` from ``deg_per_unit`` at 约
# measure_latency.py:621).  G2 above locks the first segment; the variants below
# lock the second one end to end through ``main()``, because the refusal text is
# what a field operator reads when the robot does not move.
#
# What ``deg_per_unit`` actually reads (约 measure_latency.py:621-701), verified
# against the source before writing these cases: ``normalization`` (optional,
# absent is read as ``range_m100_100``), ``range_min``, ``range_max`` and
# ``resolution``.  It does **not** read ``homing_offset`` -- that field belongs to
# the absolute-tick homing of ``capture_robot_fingerprint``, not to the
# deg-per-unit factor.  So there is deliberately no ``homing_offset`` variant
# here: it would assert a refusal the gate does not make (and should not make),
# i.e. a made-up red/green case.

_CALIBRATION_VARIANTS = {
    "missing_resolution": {"drop": ("resolution",)},
    "missing_range_min": {"drop": ("range_min",)},
    "range_max_at_range_min": {"range_max": _TRACKING_MOTOR_ENTRY["range_min"]},
}


def _incomplete_unit_metadata(*, drop: tuple[str, ...] = (), **overrides: Any) -> dict:
    """v2 metadata whose target motor entry misses or contradicts a field.

    Same shape as ``_unit_metadata`` (a legal v2 payload for the stepped joint),
    plus key *removal*: an absent field is what deployed metadata looks like,
    whereas ``{"resolution": None}`` would test a different defect path ("present
    but not finite").
    """
    entry = {key: value for key, value in _TRACKING_MOTOR_ENTRY.items() if key not in drop}
    entry.update(overrides)
    return {
        "schema_version": 2,
        "robot_model": _TRACKING_METADATA["robot_model"],
        "motors": {_TRACKING_MOTOR: entry},
    }


class _CountingTrackingFake(_TrackingFake):
    """``_TrackingFake`` plus the ``send_action_calls`` counter ``_FakeClient`` has.

    Added here rather than to the shared fake so the motion-free assertions of
    this section read exactly like the rtt ones (``send_action_calls == 0``) and
    every existing tracking test keeps its fixture untouched.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.send_action_calls = 0

    def send_action(self, action: dict) -> dict:
        self.send_action_calls += 1
        return super().send_action(action)


@pytest.mark.parametrize(
    "overrides",
    list(_CALIBRATION_VARIANTS.values()),
    ids=list(_CALIBRATION_VARIANTS),
)
def test_main_should_refuse_tracking_when_the_conversion_fields_are_incomplete(
    latency, tmp_path, capsys, overrides: dict
) -> None:
    """J9: schema v2 and the right model, but no usable deg-per-unit factor.

    The payload passes Gate C (the model is correct) and Gate F's first segment
    (``schema_version == 2``) and is refused by Gate F's second segment, so this
    is the case where the operator must be told about *calibration*, not about
    versions.  Locked assertions:

    * exit ``EXIT_MOTION_GATE_REFUSED`` (6), before any ``send_action``.
    * stderr carries both ``Gate F`` and ``unit conversion unavailable``: which
      gate refused and what about it.
    * stderr does **not** carry ``schema_version``: that string only appears in
      the first segment's text, so its absence proves this refusal came from the
      conversion fields.
    * stderr names the target motor, so a field operator can look up that exact
      entry on the host side.
    * nothing was commanded (``send_action_calls == 0``) and no CSV exists.
    """
    client = _CountingTrackingFake()
    client.latest_robot_metadata = _incomplete_unit_metadata(**overrides)
    out = tmp_path / "tracking.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "tracking",
            "--enable-motion",
            "--joint",
            _TRACKING_MOTOR,
            "--steps",
            "1",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    printed = capsys.readouterr()
    lowered = printed.err.lower()

    assert code == latency.EXIT_MOTION_GATE_REFUSED, (
        f"an unusable calibration must be refused with 6, got {code}: {printed.err!r}"
    )
    assert "Gate F" in printed.err, f"the refusal does not name Gate F: {printed.err!r}"
    assert "unit conversion unavailable" in lowered, (
        f"the refusal does not name the unit conversion: {printed.err!r}"
    )
    assert "schema_version" not in lowered, (
        "the refusal quotes the metadata schema, so it came from Gate F's first "
        f"segment instead of the conversion fields: {printed.err!r}"
    )
    assert _TRACKING_MOTOR in printed.err, (
        f"the refusal does not name the motor to look up on the host: {printed.err!r}"
    )
    assert client.send_action_calls == 0, (
        f"a refused calibration sent {client.send_action_calls} command(s)"
    )
    assert not out.exists(), f"a refused run wrote {out}"


def test_main_should_still_measure_rtt_when_the_conversion_fields_are_incomplete(
    latency, tmp_path
) -> None:
    """rtt never reads the calibration, so a broken entry cannot refuse it.

    Recorded as *current* behaviour, not as a rule to keep:
    ``collect_rtt_samples`` (约 measure_latency.py:1396) and ``_run_rtt_mode``
    (约 measure_latency.py:1666) measure the age of the last request; the only
    readers of ``client.latest_robot_metadata`` in the script are
    ``_tracking_preflight`` (约 measure_latency.py:1779) and the tracking report,
    and rtt does no unit conversion at all.  A payload that Gate F refuses under
    ``--mode tracking`` is therefore simply not consulted here and the run
    succeeds.  Asserted so the mode split is on the record: if rtt ever starts
    reading the metadata, this test is the first thing to revisit.
    """
    client = _FakeClient()
    client.latest_robot_metadata = _incomplete_unit_metadata(drop=("resolution",))
    out = tmp_path / "rtt-incomplete-units.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "2",
            "--rate-hz",
            "200",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, (
        f"rtt does not read the calibration, so the run must still succeed, got {code}"
    )
    assert out.is_file(), f"the rtt run wrote no CSV at {out}"
    assert client.send_action_calls == 0, "rtt mode must stay motion-free"


# ===========================================================================
# V. Capability, third probe: a half-filled timing face is refused
# ===========================================================================
#
# Section T asked "is the face filled?" and settled for ``not getattr(client,
# name, None)``: anything non-empty passes.  That reading has a third blind spot,
# and it is the one this section freezes: a dict that carries only *some* of the
# keys the rtt row reads.
#
# Two consumers of ``latest_request_timing`` decide which keys are necessary:
#
# * ``_request_row`` (约 benchmarks/alohamini_io/measure_latency.py:1351) reads
#   ``token``, ``sent_monotonic_s``, ``received_monotonic_s`` and ``age_s``;
# * ``net_rtt_estimate_s`` (约 benchmarks/alohamini_io/measure_latency.py:830)
#   reads ``age_s`` (约 measure_latency.py:861) -- the host stamps it also subtracts live on
#   ``latest_host_timing``, which is a different face.
#
# So the necessary key set is exactly those four (``_MISSING_TIMING_KEYS``): a
# face missing any one of them cannot produce a readable measurement.  Today the
# non-empty probe lets such a face through, so the run reports success and the
# CSV holds rows whose ``age_s`` cell is empty (missing ``age_s``), whose
# ``sent_monotonic_s`` / ``received_monotonic_s`` / ``token`` cells are empty
# (missing one of those) and whose ``net_rtt_*`` cells are ``NaN`` while
# ``host_timing_missing`` still reads ``False`` -- a file that looks like a
# measurement of zero.
#
# The cases below drive the shape from the outside through ``main()``: the
# refusal section T already owns (exit ``EXIT_CLIENT_UNSUPPORTED``, no CSV at
# all) plus the one thing the half-filled case needs on top of it -- stderr has to
# *name the missing key*, because "the client is broken" is not actionable while
# "``latest_request_timing`` has no ``age_s``" is.

# The four keys the rtt row consumes; the parametrisation drops one at a time.
_MISSING_TIMING_KEYS = ("token", "sent_monotonic_s", "received_monotonic_s", "age_s")

# The two substrings ``_run_rtt_mode`` wraps the missing-key list in
# (约 benchmarks/alohamini_io/measure_latency.py:1666): the line it prints is
# ``...; missing key(s): <keys>, so a run would write empty token, stamp and age
# cells ...``.  They are *implementation contract*: if the refusal is reworded,
# both move with it.
#
# Why the list has to be bounded at all: the explanation that follows it names
# ``token`` / ``stamp`` / ``age`` in prose, so a bare ``key in stderr`` is green
# for ``token`` even when the refusal reports no key whatsoever -- the fake green
# this section was written to close.
_MISSING_KEYS_MARKER = "missing key(s):"
_MISSING_KEYS_TERMINATOR = ", so "

# Item shape inside that clause: ``<key>`` or ``<key> (<reason>)``.  The suffix is
# stripped from the right (greedy), so a reason quoting a parenthesised value --
# ``token (empty text ('   '))`` -- still leaves the bare key name behind.
_ANNOTATION_SUFFIX = re.compile(r"\s*\(.*\)$")


def _missing_key_clause(stderr: str) -> str:
    """The part of ``stderr`` that lists the keys the half-filled face misses.

    Everything between ``_MISSING_KEYS_MARKER`` and the ``_MISSING_KEYS_TERMINATOR``
    join (or the end of the line, whichever comes first), stripped: for the
    current wording that is exactly ``"age_s"`` / ``"token"`` /
    ``"sent_monotonic_s"`` / ``"received_monotonic_s"``, never the boilerplate
    sentence.  An absent marker yields the empty string, so a refusal that names
    no key at all cannot satisfy any containment check made on the clause.

    Only the two constants above are assumed about the wording: the separator
    inside the key list, the punctuation after it and the rest of the sentence
    are not.  The key-list separator *is* assumed one step later, by
    ``_missing_key_names``.
    """
    clause = stderr.partition(_MISSING_KEYS_MARKER)[2]
    for terminator in (_MISSING_KEYS_TERMINATOR, "\n"):
        clause = clause.partition(terminator)[0]
    return clause.strip()


def _missing_key_names(stderr: str) -> set[str]:
    """The key names the missing-key clause lists, as a set.

    Item shape (the annotation contract, see ``_ANNOTATION_SUFFIX`` below and
    ``_key_carries_a_reason``): one item is either ``<key>`` -- a key the face does
    not carry at all -- or ``<key> (<reason>)``, a key it carries with a value no
    row can hold.  The reason itself never contains ``", "`` (the implementation
    guarantees it, and ``_short_value`` decides whether a quoted value may be
    appended to it), so the separator below still splits *items* and never a reason.

    Parsing rules, each chosen to fail *closed* rather than open:

    * the input is ``_missing_key_clause(stderr)`` -- marker to terminator (or
      end of line), stripped.  An absent marker gives ``""`` and therefore
      ``set()``, which equals no expected key set at all;
    * the clause is split on ``", "``, the separator ``", ".join(...)`` puts
      between two items in ``_run_rtt_mode``.  A comma without a space does not
      split, so the unsplit remainder stays one item and is not a key name;
    * every item is stripped and empty items are dropped, so a stray separator
      at either end does not invent a key;
    * the trailing `` (<reason>)`` is stripped off every item -- greedy, i.e. from
      the first ``" ("`` through the *last* ``")"`` of that item, so a reason that
      quotes a value in parentheses (``token (empty text ('   '))``) still yields
      the bare key.  This is a *strengthening*, not a relaxation: the equality
      assertions of R20 (absent keys, listed bare) and of R28/R33 (unusable
      values, listed with a reason) are the same assertion on the key names, and
      "does this key carry a reason?" is a separate check made on the clause text
      (``_key_carries_a_reason``);
    * nothing else is dropped.  Prose residue is deliberately *kept*: if the
      terminator disappears, the clause runs to the end of the line and the
      sentence that spells ``token`` / ``stamp`` / ``age`` out in prose lands in
      the set -- the equality is red instead of ``token`` quietly going green on
      that prose.

    So the result is ``{"<key>"}`` only when the clause is a list holding exactly
    that one key name, with or without a reason next to it.
    """
    clause = _missing_key_clause(stderr)
    names: set[str] = set()
    for item in clause.split(", "):
        item = item.strip()
        if item:
            names.add(_ANNOTATION_SUFFIX.sub("", item).strip())
    return names


def _key_carries_a_reason(stderr: str, key: str) -> bool:
    """Whether ``key`` is annotated, i.e. the clause holds ``"<key> ("``.

    The other half of the annotation contract: a key whose *value* is unusable has
    to say why (``age_s (negative value (-0.5))``), while a key the face simply
    does not carry stays bare (R20).  Checked on the clause text, not on the parsed
    names, because the names are deliberately reason-free.
    """
    return f"{key} (" in _missing_key_clause(stderr)


class _ClientWithPartialTiming(_FakeClient):
    """``_FakeClient`` with one key removed from the timing face after every frame.

    ``_FakeClient`` fills all four keys on every ``get_observation``; this
    subclass pops ``dropped_key`` afterwards, so the face is *non-empty* (section
    T's probe is satisfied) and *incomplete* (the row that would be written has an
    empty cell or a ``nan``).  Key removal, not ``{key: None}``: an absent key is
    what a half-implemented client leaves behind, and it is also the shape that
    makes the "name the missing key" requirement checkable.

    Defaults are untouched -- the base fake still fills every key.
    """

    def __init__(self, *, dropped_key: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._dropped_key = dropped_key

    def get_observation(self, include_cameras: bool = True) -> dict:
        """Take the frame the base fake builds, then drop the scripted key."""
        state = super().get_observation(include_cameras=include_cameras)
        self.latest_request_timing.pop(self._dropped_key, None)
        return state


class _ClientWithNoneValuedTiming(_FakeClient):
    """``_FakeClient`` with one timing key holding ``None`` after every frame.

    The sibling of ``_ClientWithPartialTiming`` one step earlier: that class *pops*
    the key (absent), this one leaves it in the face with the value ``None``.  Two
    ways to have no value at all, and one refusal for both:
    ``_request_timing_key_problems`` lists a ``None`` value as a *missing* key,
    spelled bare, because the cell would be empty for the same reason.

    The face stays non-empty (section T's probe is satisfied) and every other key
    stays usable, so the only thing under test is the ``None`` shape.

    Defaults are untouched -- the base fake still fills every key.
    """

    def __init__(self, *, none_key: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._none_key = none_key

    def get_observation(self, include_cameras: bool = True) -> dict:
        """Take the frame the base fake builds, then set the scripted key to ``None``."""
        state = super().get_observation(include_cameras=include_cameras)
        self.latest_request_timing[self._none_key] = None
        return state


@pytest.mark.parametrize("dropped_key", _MISSING_TIMING_KEYS)
def test_main_should_refuse_rtt_when_the_timing_face_misses_a_key(
    latency, tmp_path, capsys, dropped_key: str
) -> None:
    """R20: a face holding three of the four keys is refused, and the key is named.

    Locked assertions, and why each one is here:

    * exit ``EXIT_CLIENT_UNSUPPORTED`` (9): at the time of writing the run exits
      0 and writes a CSV, i.e. a half-filled face is accepted as a measurement;
    * the CSV does not exist -- a refused run writes nothing, not even a header,
      for the same reason R8/R11 require it;
    * stderr names the face (``latest_request_timing``, as in R8/R11/R14) *and*
      the missing key, so the operator can tell which cell would have been blank
      instead of guessing which of the four the client does not fill.  The key has
      to be named *inside the missing-key clause* (``_missing_key_clause``), not
      merely somewhere in stderr: the explanation that follows the clause spells
      ``token`` / ``stamp`` / ``age`` out in prose, so a bare containment check was
      green for ``token`` even when the refusal reported no key at all.  The clause
      boundary is the implementation contract documented at
      ``_MISSING_KEYS_MARKER`` / ``_MISSING_KEYS_TERMINATOR`` and has to move with
      the wording in ``_run_rtt_mode``.
    * the set of keys parsed out of that clause is *exactly* ``{dropped_key}``
      (``_missing_key_names``), not merely a set that contains it: a refusal that
      always lists all four keys (the ``"token"`` / ``"received_monotonic_s"``
      cases would stay green on containment alone, the other two go red) and a
      clause whose terminator vanished (it would then swallow the prose sentence
      and go green for ``token`` again) both fail this one.  Containment stays as
      the first, more legible message; the equality is the assertion that locks
      the shape.
    * the key is listed **bare**, with no `` (<reason>)`` annotation: an absent key
      has nothing to explain -- the cell would be empty because the value is not
      there -- and annotating it would send the operator looking for a value defect
      in a field that does not exist.  ``_missing_key_names`` cannot see this (it
      strips the annotation by design), so the clause text is checked directly:
      without this assertion an implementation that annotates *every* item (e.g.
      ``(key, "absent")``) satisfies the equality above and only R33's mixed test
      would notice.

    Nothing here names an implementation constant: the four keys are the ones
    ``_request_row`` / ``net_rtt_estimate_s`` consume, and the test only states
    that dropping one of them must not produce a measurement.

    ``--timeout-s 0.3`` and 200 Hz: the run must be over quickly whichever way it
    goes (three paced frames today, one refused frame after the fix), and the
    budget is loose enough that a refusal may look at more than one frame.
    """
    client = _ClientWithPartialTiming(dropped_key=dropped_key, ages=[0.030, 0.031, 0.032])
    out = tmp_path / f"half-filled-{dropped_key}.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"a timing face without {dropped_key!r} was accepted as a measurement: exit "
        f"{code} (expected 9), csv_exists={out.exists()}; stderr: {captured.err!r}"
    )
    assert not out.exists(), (
        f"a run refused for a half-filled timing face still wrote {out}: a CSV built "
        f"from a face without {dropped_key!r} holds empty/{'nan'} cells that read like "
        "a measurement"
    )
    assert "latest_request_timing" in captured.err, (
        f"stderr does not name the half-filled face: {captured.err!r}"
    )
    # The key has to be named *in the missing-key clause*, not merely somewhere in
    # stderr: the wording that follows the clause spells ``token`` / ``stamp`` /
    # ``age`` out in prose, so ``dropped_key in captured.err`` was green for
    # ``token`` even with the key list deleted from the message (verified by
    # mutation: dropping ``missing_keys`` from the refusal left the ``token`` case
    # green and turned the other three red).
    assert _MISSING_KEYS_MARKER in captured.err, (
        "the refusal never says which keys are missing, so an operator cannot tell "
        f"which cell would have been blank: {captured.err!r}"
    )
    clause = _missing_key_clause(captured.err)
    assert dropped_key in clause, (
        f"the missing-key clause does not name {dropped_key!r}: clause={clause!r}, "
        f"stderr={captured.err!r}"
    )
    names = _missing_key_names(captured.err)
    assert names == {dropped_key}, (
        f"the missing-key clause does not name exactly {dropped_key!r}: "
        f"keys={sorted(names)!r}, clause={clause!r}, stderr={captured.err!r}; a clause "
        "that lists all four keys, lists none, or runs past its terminator into the "
        "prose sentence must not read as 'exactly the one key that is missing'"
    )
    # The other half of the annotation contract (R33 covers the annotated half): an
    # *absent* key is listed bare.  ``_missing_key_names`` strips `` (<reason>)``, so
    # it cannot tell the two shapes apart -- this check is on the clause text.
    assert f"{dropped_key} (" not in clause, (
        f"the absent key {dropped_key!r} is annotated in the missing-key clause: "
        f"clause={clause!r}, stderr={captured.err!r}; a key the face does not carry has "
        "no value to explain, and an annotation would send the operator hunting for a "
        "value defect in a field that is not there"
    )


@pytest.mark.parametrize("none_key", _MISSING_TIMING_KEYS)
def test_main_should_treat_a_none_valued_timing_key_as_a_missing_key(
    latency, tmp_path, capsys, none_key: str
) -> None:
    """R20: a key holding ``None`` is *missing*, not "there with an unusable value".

    The fourth shape of the same face, and the only one that had a code path but no
    test: ``_request_timing_key_problems`` puts ``key not in timing`` and
    ``timing[key] is None`` in the same branch, so both are listed as a bare key
    name in the missing-key clause -- the cell would be empty because the value is
    not there, just one step earlier.

    That reading has to stay apart from the annotated shape
    (``age_s (negative value (-0.5))``, section Y): a bare key sends the operator
    to "this cell is missing", an annotated one to "this value is broken".
    Collapsing ``None`` into "unusable" would annotate a value that does not exist,
    and ``_missing_key_names`` strips annotations by design, so the two assertions
    below are made together: the parsed names are exactly ``{none_key}`` *and* the
    clause carries no ``"<key> ("`` for it.

    Locked per case (exit code and "no CSV" are R20's, the clause shape is new):

    * exit ``EXIT_CLIENT_UNSUPPORTED`` (9), no CSV at all -- a refused run writes
      nothing, not even a header (R8/R11/R20);
    * the clause lists exactly ``{none_key}`` (``_missing_key_names``), not a set
      that merely contains it, so a refusal listing all four keys or none fails;
    * ``_key_carries_a_reason`` is ``False`` for that key: ``None`` carries no
      value to explain, so the item has to read ``<key>``, never ``<key> (<reason>)``.
      The parsed names cannot see this (they strip the annotation), hence the
      separate check on the clause text.

    ``--samples 3`` / 200 Hz / ``--timeout-s 0.3`` matches R20: the run is over on
    the first growth, and the budget is loose enough that a refusal may look at
    more than one frame.
    """
    client = _ClientWithNoneValuedTiming(none_key=none_key, ages=[0.030, 0.031, 0.032])
    out = tmp_path / f"none-valued-{none_key}.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()
    clause = _missing_key_clause(captured.err)

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"a timing face whose {none_key!r} is None was accepted as a measurement: exit "
        f"{code} (expected 9), csv_exists={out.exists()}; stderr: {captured.err!r}"
    )
    assert not out.exists(), (
        f"a run refused for a None-valued timing key still wrote {out}: the row built "
        f"from {none_key}=None holds an empty cell that reads like a measurement"
    )
    names = _missing_key_names(captured.err)
    assert names == {none_key}, (
        f"the missing-key clause does not name exactly {none_key!r}: keys={sorted(names)!r}, "
        f"clause={clause!r}, stderr={captured.err!r}; a None value is *missing* (the cell "
        "would be empty), so it belongs in the same clause as an absent key -- not in a "
        "refusal that lists all four keys or none"
    )
    assert not _key_carries_a_reason(captured.err, none_key), (
        f"the None-valued key {none_key!r} is annotated in the missing-key clause: "
        f"clause={clause!r}, stderr={captured.err!r}; an annotation (``<key> (<reason>)``) "
        "means 'the value is there and unusable' (section Y), and a None value sends the "
        "operator hunting for a value defect in a field that holds nothing"
    )


def test_main_should_measure_rtt_when_the_timing_face_carries_all_four_keys(
    latency, tmp_path
) -> None:
    """R21: the four keys are *sufficient* -- a legal client is not refused.

    The other half of R20's rule, and the reason R20 cannot be satisfied by
    refusing every client: ``_FakeClient`` fills exactly ``token`` /
    ``sent_monotonic_s`` / ``received_monotonic_s`` / ``age_s``, so this is the
    shape the necessity set has to let through.  Locked: exit 0, one CSV row per
    ``--samples``, and non-empty ``token`` / ``age_s`` cells -- precisely the cells
    the half-filled variants blank out.
    """
    client = _FakeClient(ages=[0.030, 0.031, 0.032])
    out = tmp_path / "complete-timing.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, (
        f"a client that fills all four timing keys must stay measurable, got {code}"
    )
    assert out.is_file(), f"the rtt run wrote no CSV at {out}"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 3, f"expected one row per requested sample, got {len(rows)}"
    for row in rows:
        assert row["token"] != "", f"row {row['sample_index']} has an empty token: {row}"
        assert row["age_s"] != "", f"row {row['sample_index']} has an empty age_s: {row}"


# The four necessary keys plus two a newer client might add: a superset of the
# necessity set, not a different one.
_EXTRA_TIMING_KEYS = {
    "token": b"tok-extra",
    "sent_monotonic_s": 1000.0,
    "received_monotonic_s": 1000.030,
    "age_s": 0.030,
    "sent_unix_ns": 1_700_000_000_000_000_000,
    "attempts": 1,
}


class _ClientWithExtraTimingKeys(_FakeClient):
    """The four necessary timing keys plus two a newer client might add."""

    def get_observation(self, include_cameras: bool = True) -> dict:
        """Overwrite the timing face with the six-key shape after the frame."""
        state = super().get_observation(include_cameras=include_cameras)
        self.latest_request_timing = dict(_EXTRA_TIMING_KEYS)
        return state


def test_main_should_measure_rtt_when_the_timing_face_carries_extra_keys(latency, tmp_path) -> None:
    """R22: the necessary set is "these four", not "exactly these four".

    A newer client may add keys (a unix stamp, a retry counter); the refusal has to
    be "one of the four is missing", never "the face is not exactly the four keys"
    -- otherwise the probe would reject every client that grows a field.  This case
    is what keeps R20 from being implemented as an equality check.
    """
    client = _ClientWithExtraTimingKeys(ages=[0.030, 0.031, 0.032])
    out = tmp_path / "extra-timing-keys.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "2",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, (
        f"a timing face carrying extra keys is still measurable, got {code}"
    )
    assert out.is_file(), f"the rtt run wrote no CSV at {out}"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2, f"expected one row per requested sample, got {len(rows)}"
    for row in rows:
        assert row["age_s"] != "", f"row {row['sample_index']} has an empty age_s: {row}"


# ===========================================================================
# W. Frozen feedback: the exit code names the cause, and the loop sleeps
# ===========================================================================
#
# Section E froze the *tracking* half of a dead link (``FEEDBACK_STALL_LIMIT``).
# The rtt half has exactly one outcome today: a frozen
# ``client.observation_sequence`` ends in ``InsufficientSamples`` -> exit 7
# ("insufficient samples"), whatever it was that stood still.  That wording sends
# the operator to ``--timeout-s`` / ``--samples``, while the cause can be the
# client's capability: a client that publishes ``latest_request_timing`` and never
# fills it produces the same frozen, row-less run, and section T's per-growth
# probe cannot see it because there is no growth to probe on.
#
# Two things are frozen in this section.
#
# 1. **The exit code says which cause it was** (``_FROZEN_SEQUENCE_CASES`` below).
#    Frozen sequence + a never-filled timing face is a *capability* refusal (9)
#    whose stderr must not talk about "insufficient samples"; frozen sequence + a
#    filled timing face stays 7.  Both run through ``main()`` with a 50 ms budget,
#    so neither can be mistaken for a slow run.
# 2. **The loop waits instead of spinning.**  The pacing arithmetic *was*
#    ``target_s = start_s + len(rows) * period_s``, so with 0 rows
#    ``target_s == start_s`` and ``pause_s <= 0``: no sleep was issued and the
#    loop polled as fast as the client answers until the deadline.  The current
#    arm of that line is ``... if grew else now_s + period_s``
#    (约 benchmarks/alohamini_io/measure_latency.py:1555): a poll that grew no row
#    waits one whole period instead of collapsing onto ``start_s``.  Measured on
#    this workspace before the fix (``~/pi-workspace/tmp-measure-latency-red/
#    spin_measure.py``, frozen sequence, real ``time.sleep``, 50 ms budget):
#    216,888 ``get_observation`` calls, i.e. ~4.3 M calls/s.  The bound in
#    ``test_collect_rtt_samples_should_bound_the_observation_calls_when_the_sequence_is_frozen``
#    is 25 for a 100 ms budget at 100 Hz: it locks the *order of magnitude* of the
#    wait against the ``--rate-hz`` period (one 10 ms period per poll, plus
#    slack), not the shape of the sleep, and the spin it separates from that is
#    three to four orders above it.


# id -> (client builder, name of the exit code the run must return).  The two
# cases sit side by side on purpose: they are one rule, and a fix that returns 9
# whenever the sequence is frozen breaks the second one.
_FROZEN_SEQUENCE_CASES: dict[str, tuple[Any, str]] = {
    "untimed_timing_face": (
        lambda: _ClientWithEmptyTiming(sequences=[0]),
        "EXIT_CLIENT_UNSUPPORTED",
    ),
    "timed_timing_face": (lambda: _FakeClient(sequences=[1]), "EXIT_INSUFFICIENT_SAMPLES"),
}

# Words that belong to the sampling-budget refusal only.  The capability refusal
# must not use them: "insufficient samples" is the sentence that sends the field
# operator to the wrong knob.
_BUDGET_WORDS = ("insufficient samples",)


@pytest.mark.parametrize("case_id", list(_FROZEN_SEQUENCE_CASES), ids=list(_FROZEN_SEQUENCE_CASES))
def test_main_should_tell_a_frozen_sequence_with_an_untimed_face_from_a_timed_one(
    latency, tmp_path, capsys, case_id: str
) -> None:
    """R23: frozen + never-filled face exits 9 (not 7); frozen + filled stays 7.

    The empty-face case is the defect: the sequence never grows, so section T's
    per-growth probe never runs, ``InsufficientSamples`` is raised at the deadline
    and ``main`` prints "insufficient samples" -- the run reports a sampling-budget
    problem for what is a client-capability problem.  The sequence is therefore
    pinned at *zero* (``sequences=[0]``, so ``observation_sequence`` is 0 on every
    frame and never grows): with ``sequences=[1]`` the first frame would already
    grow, and that shape is refused as unfilled today already (R8), i.e. it cannot
    show this defect.

    The filled-face case is the guard rail: a client whose timing is fine and whose
    feedback is simply frozen is *not* a capability problem, so the fix must keep
    answering 7 there.  Asserting both in one parametrisation is what makes "9 only
    when the face is untimed" a testable rule instead of a guess.

    Asserted per case: the exit code; no CSV on disk (neither refusal has a row to
    write); and for the capability case only, stderr neither says "insufficient
    samples" nor leaves the fault unnamed -- it has to name
    ``latest_request_timing``, like every other exit-9 refusal (R8/R11/R14).
    """
    build, code_attr = _FROZEN_SEQUENCE_CASES[case_id]
    expected_code = getattr(latency, code_attr)
    client = build()
    out = tmp_path / f"frozen-{case_id}.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "100",
            "--timeout-s",
            "0.05",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()
    lowered = captured.err.lower()

    assert code == expected_code, (
        f"a frozen sequence ({case_id}) must exit {expected_code}, "
        f"got {code}; stderr: {captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"
    if expected_code == latency.EXIT_CLIENT_UNSUPPORTED:
        assert not any(word in lowered for word in _BUDGET_WORDS), (
            "the capability refusal reuses the sampling-budget wording, so the operator "
            f"is sent to --timeout-s / --samples instead of the client: {captured.err!r}"
        )
        assert "latest_request_timing" in captured.err, (
            f"stderr does not name the face whose absence refused the run: {captured.err!r}"
        )


class _SleepDrivenClock:
    """Clock that moves only when the script sleeps -- the way a real loop waits.

    ``__call__`` never advances (a wall clock read costs nothing to the run), so a
    loop that computes a non-positive pause and skips ``sleep`` never leaves the
    iteration: its deadline can only be reached through the injected sleeper.
    ``sleeps`` records every pause the script issued, which is what a bound on the
    poll count is really about.
    """

    def __init__(self, start: float = 0.0) -> None:
        self.now = start
        self.sleeps: list[float] = []

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        """Advance the fake clock by the pause the script asked for."""
        self.sleeps.append(seconds)
        self.now += seconds


class _SpinGuardClient(_FakeClient):
    """``_FakeClient`` that raises instead of letting a spinning loop run forever.

    The frozen-sequence case is an infinite loop unless the loop waits, so a plain
    fake would hang the test instead of failing it.  The guard turns the spin into
    a ``RuntimeError("spin")`` at a hard cap -- a red with a number in it, not a
    hung CI job.
    """

    def __init__(self, *, limit: int, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._limit = limit

    def get_observation(self, include_cameras: bool = True) -> dict:
        """Refuse to answer past the cap, which only a spinning loop reaches."""
        if self.get_observation_calls >= self._limit:
            raise RuntimeError("spin")
        return super().get_observation(include_cameras=include_cameras)


def test_collect_rtt_samples_should_bound_the_observation_calls_when_the_sequence_is_frozen(
    latency,
) -> None:
    """R24: a frozen sequence must cost a bounded number of polls, not a spin.

    ``collect_rtt_samples`` paces against ``start_s + len(rows) * period_s``.  With
    no growth there is no row, so ``target_s == start_s``, ``pause_s <= 0`` and the
    loop skips ``sleep`` entirely -- with a real client that is ~4.3 M
    ``get_observation`` calls per second until the deadline (measured: 216,888
    calls in 50 ms; the 10 s review run saw ~5.96e7).

    The bound is ``2 * (timeout_s * rate_hz) + 5`` = 25 calls for a 100 ms budget
    at 100 Hz.  It locks the *order of magnitude* of the wait against the
    ``--rate-hz`` period, not the shape of the sleep:

    * the slowest legal shape -- one poll per period -- needs 10 polls, and a loop
      that sleeps the remaining budget in a single pause needs 1-2;
    * the factor 2 covers a loop that polls once before and once after each
      period, and the ``+ 5`` is slack for the first/last poll and for a refusal
      that looks at a frame before deciding;
    * the defect is three to four orders of magnitude above it (thousands of calls
      inside one period), so no plausible paced implementation lands in between.

    What the bound does exclude, deliberately: a wait an order of magnitude finer
    than one sampling period.  A fixed 1 ms floor at 100 Hz costs ~100 polls in
    the same budget, i.e. a loop polling at 1 kHz -- that is not pacing at
    ``--rate-hz``, it only spends the CPU more slowly than a spin.  Anything at
    the scale of one period (10 ms here, or the whole remaining budget in one
    pause) fits.

    The timing face is *filled* here (``_FakeClient``) and only the sequence is
    frozen: that keeps this case about the pacing arithmetic alone, so it does not
    depend on how the fix words the untimed-face refusal of R23.  The frozen run
    must still end in ``InsufficientSamples`` with no rows -- waiting longer is not
    an excuse to invent samples.
    """
    rate_hz = 100.0
    timeout_s = 0.1
    upper_bound = 2 * int(timeout_s * rate_hz) + 5  # 25
    clock = _SleepDrivenClock()
    rows: list[dict[str, Any]] = []
    client = _SpinGuardClient(limit=2000, sequences=[0])

    raised: BaseException | None = None
    try:
        latency.collect_rtt_samples(
            client,
            samples=3,
            rate_hz=rate_hz,
            timeout_s=timeout_s,
            sleep=clock.sleep,
            clock=clock,
            rows_out=rows,
        )
    except Exception as exc:  # the guard's RuntimeError is the interesting failure
        raised = exc

    calls = client.get_observation_calls

    assert isinstance(raised, latency.InsufficientSamples), (
        "a frozen sequence must end in InsufficientSamples once the budget is over; "
        f"got {type(raised).__name__}({raised}) after {calls} get_observation calls: "
        "the loop never slept, so it polled until the spin guard stopped it"
    )
    assert rows == [], f"a run without a single growth still produced rows: {rows}"
    assert calls <= upper_bound, (
        f"a frozen sequence cost {calls} get_observation calls in a {timeout_s} s budget "
        f"at {rate_hz} Hz (at most {upper_bound} paced polls): the loop is spinning "
        "because pause_s <= 0 while it holds no rows"
    )


# ===========================================================================
# X. Non-finite run shape: the rate the loop paces on and the budget it stops on
# ===========================================================================
#
# Section R closed ``--control-hz`` against non-finite values; the two knobs the
# *rtt* loop paces and stops on kept the finiteness-blind checks.
#
# 1. ``--rate-hz`` was checked with ``args.rate_hz <= 0.0``; the guard that closes
#    that hole today is ``not (math.isfinite(args.rate_hz) and args.rate_hz > 0.0)``
#    (约 benchmarks/alohamini_io/measure_latency.py:1273-1274).  ``inf`` and ``1e400``
#    (argparse parses the latter to ``inf``) are ``> 0`` and pass; ``nan`` makes
#    every comparison false, so it passes too.  ``collect_rtt_samples`` then
#    computes ``period_s = 1.0 / rate_hz if rate_hz > 0 else 0.0``
#    (约 measure_latency.py:1494): for ``nan`` that is ``0.0``, every target
#    collapses onto "now", no sleep is issued and the run polls at the client's
#    maximum speed -- the spin R24 bounds, reached through a second door.
# 2. ``--timeout-s`` had no check at all; it is rejected today by the same
#    finiteness shape (约 measure_latency.py:1294-1295).  ``nan`` / ``inf`` make
#    ``deadline_s = start_s + timeout_s`` (约 measure_latency.py:1504) a value the
#    ``now_s >= deadline_s`` test never satisfies (``nan`` compares false, ``inf``
#    never arrives), so the run can never give up on its own; ``0`` and ``-1``
#    make the budget already expired before the first frame, i.e. a run that read
#    nothing (the give-up probe deliberately keeps that as
#    ``InsufficientSamples``, so it is not even a loud failure).
#
# R25/R26 are parser-level on purpose: a nonsense run shape is a usage error (2)
# before a socket is opened, and neither of them samples anything.  R27 covers
# the direct caller the CLI cannot protect (the same split as section R's R-1 /
# R-2 pair).


@pytest.mark.parametrize(
    "rate_hz",
    ["nan", "inf", "1e400"],
    ids=["nan", "inf", "overflow_to_inf"],
)
def test_parse_args_should_reject_a_non_finite_rate_hz(latency, capsys, rate_hz: str) -> None:
    """R25: a non-finite ``--rate-hz`` is a usage error (2), never a full-speed run.

    ``inf`` (and ``1e400``, which *is* ``inf`` after ``float()``) satisfies
    ``> 0``, and ``nan`` satisfies nothing, so the ``<= 0.0`` check of
    ``parse_args`` lets all three reach ``collect_rtt_samples``, where
    ``period_s`` degenerates to ``0.0`` and the pacing is skipped: the frozen
    sequence then costs a full-speed poll loop (R24 measures that shape, R27
    reaches it without the CLI).

    Asserted: exit code 2 (usage error, so no socket is opened) and the option
    name on stderr, the shape ``--control-hz``'s non-finite case (R-1) uses.  The
    legal value is parsed first on purpose -- an unknown or mistyped option also
    exits 2, so without that guard this test would go green on a ``--rate-hz``
    that no longer exists.
    """
    # Arrange: the option exists and a legal value reaches the namespace.
    legal = latency.parse_args(["--host", "127.0.0.1", "--rate-hz", "30"])
    assert getattr(legal, "rate_hz", None) == pytest.approx(30.0), (
        "--rate-hz 30 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    # Act + Assert: the non-finite value is refused by the parser itself.
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--rate-hz={rate_hz}"])

    assert excinfo.value.code == 2, (
        f"--rate-hz={rate_hz} exited with {excinfo.value.code} instead of the usage "
        "error 2: a non-finite sampling rate reaches collect_rtt_samples, whose "
        "period_s degenerates to 0.0 and turns the run into a full-speed poll loop"
    )
    message = capsys.readouterr().err.lower()
    assert "rate-hz" in message, f"the error does not name the option: {message!r}"


@pytest.mark.parametrize(
    "timeout_s",
    ["nan", "inf", "1e400", "0", "-1"],
    ids=["nan", "inf", "overflow_to_inf", "zero", "negative"],
)
def test_parse_args_should_reject_a_nonsense_timeout_s(latency, capsys, timeout_s: str) -> None:
    """R26: a non-finite or non-positive ``--timeout-s`` is a usage error (2).

    ``--timeout-s`` is the only stop condition of the rtt run
    (``deadline_s = start_s + timeout_s`` against the loop clock), and it is not
    validated at all:

    * ``nan`` / ``inf`` / ``1e400`` -> the deadline is never reached, so the loop
      runs until ``--samples`` fresh frames arrive however long that takes -- with
      a frozen sequence, until the operator kills it;
    * ``0`` / ``-1`` -> the budget is already expired before the first
      ``get_observation``, so the run collects nothing and reports
      ``InsufficientSamples`` (exit 7) for a budget the operator typed wrong.

    All five parse cleanly today, which is what makes this red.  Asserted: exit
    code 2 and ``--timeout-s`` on stderr.  The legal budget is parsed first for
    the same reason as in R25.
    """
    # Arrange: the option exists and a legal value reaches the namespace.
    legal = latency.parse_args(["--host", "127.0.0.1", "--timeout-s", "10"])
    assert getattr(legal, "timeout_s", None) == pytest.approx(10.0), (
        "--timeout-s 10 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    # Act + Assert: the nonsense budget is refused by the parser itself.
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--timeout-s={timeout_s}"])

    assert excinfo.value.code == 2, (
        f"--timeout-s={timeout_s} exited with {excinfo.value.code} instead of the "
        "usage error 2: an unvalidated budget either never expires (nan/inf) or is "
        "already over before the first frame (0/-1)"
    )
    message = capsys.readouterr().err.lower()
    assert "timeout-s" in message, f"the error does not name the option: {message!r}"


# The finest per-poll wait that still deserves the name "pacing" for this
# measurement: 1 ms is already two orders below the 20 ms host loop period, and a
# loop waiting 1 ms between polls is polling at 1 kHz, not sampling at
# ``--rate-hz``.  R27's bound is derived from it (see the docstring).
_FINEST_PACING_WAIT_S = 1e-3


def test_collect_rtt_samples_should_not_spin_when_the_rate_is_not_finite(latency) -> None:
    """R27: a direct caller's non-finite ``rate_hz`` must not turn the loop loose.

    ``parse_args`` (R25) is not a guarantee for this function: a caller can hand
    it ``rate_hz=float("nan")`` straight away.  ``period_s = 1.0 / rate_hz if
    rate_hz > 0 else 0.0`` then yields ``0.0`` -- the same ``pause_s <= 0`` shape
    R24 bounds for a frozen sequence -- so the loop never sleeps, never reaches
    its deadline and polls at full speed until something outside stops it.

    Reuses R24's harness: ``_SleepDrivenClock`` moves only when the loop sleeps,
    so a spinning loop cannot reach this run's deadline, and ``_SpinGuardClient``
    turns that spin into a ``RuntimeError`` at 2000 calls instead of hanging the
    test.  The timing face is filled (``_FakeClient``) and only the sequence is
    frozen, so this case is about the pacing arithmetic alone.

    Locked, and what each part says:

    * ``InsufficientSamples`` -- the run still has to *end*, and a 100 ms budget
      with no fresh frame is a short budget, not a capability gap (the face is
      timed);
    * ``rows_out == []`` -- waiting longer is not an excuse to invent a row;
    * ``get_observation_calls <= 2 * int(timeout_s / _FINEST_PACING_WAIT_S)`` =
      **200** -- the poll count is bounded.  Derivation: whatever floor the fix
      picks for one wait, the finest one that still paces at this measurement's
      scale is 1 ms (``_FINEST_PACING_WAIT_S``), so a 100 ms budget fits at most
      100 such polls; the factor 2 covers a poll before and after each pause.  The
      defect is a full order of magnitude above the bound (2000 calls, where the
      guard fires), and the guard itself must therefore never be reached by a
      paced loop -- which is why the cap is 2000 and not, say, 250.
    """
    timeout_s = 0.1
    upper_bound = 2 * int(timeout_s / _FINEST_PACING_WAIT_S)  # 200
    clock = _SleepDrivenClock()
    rows: list[dict[str, Any]] = []
    client = _SpinGuardClient(limit=2000, sequences=[0])

    raised: BaseException | None = None
    try:
        latency.collect_rtt_samples(
            client,
            samples=3,
            rate_hz=float("nan"),
            timeout_s=timeout_s,
            sleep=clock.sleep,
            clock=clock,
            rows_out=rows,
        )
    except Exception as exc:  # the guard's RuntimeError is the interesting failure
        raised = exc

    calls = client.get_observation_calls

    assert isinstance(raised, latency.InsufficientSamples), (
        "a non-finite rate_hz must not open an unbounded loop: a 100 ms budget with "
        f"no fresh frame has to end in InsufficientSamples; got "
        f"{type(raised).__name__}({raised}) after {calls} get_observation calls "
        f"({sum(clock.sleeps)} s of simulated wait in {len(clock.sleeps)} pauses)"
    )
    assert rows == [], f"a run without a single growth still produced rows: {rows}"
    assert calls <= upper_bound, (
        f"a non-finite rate_hz cost {calls} get_observation calls in a {timeout_s} s "
        f"budget (at most {upper_bound} paced polls): period_s became 0.0, so the "
        "loop never sleeps and polls at the client's maximum speed"
    )


# ===========================================================================
# Y. Capability, fourth probe: a key that is there, holding an unusable value
# ===========================================================================
#
# Section T asks "is the face there?", section V asks "does the face carry all
# four keys?".  Both are satisfied by a key that is *present* and holds a value
# no rtt cell can carry, and that is the defect this section freezes.
#
# ``missing_request_timing_keys`` (约 benchmarks/alohamini_io/measure_latency.py:1162)
# judges each of ``REQUIRED_REQUEST_TIMING_KEYS`` (约 :568) key by key through
# ``_timing_key_problem`` (约 :649): a finite number that means a measurement (not a
# ``bool``, not negative for ``age_s``), or a ``str``/``bytes`` token whose stripped
# text is not empty.  *Before* the fix (HEAD 9a9f9d5) the same function used two
# converters and nothing else: ``_finite_or_none`` (约 :599, ``float(value)`` inside
# a ``try``) for the three numbers, ``_request_token_text`` (约 :608, ``str`` of
# whatever arrives) for the token -- so the probe answered "can the value be
# converted?", never "is the converted value a measurement".  Measured at 9a9f9d5
# through ``main()`` (the cases below), each of the following exited **0 with a CSV
# on disk**, which is what this section was written against:
#
#   * ``age_s = -0.5``   -> ``float(-0.5)`` is finite -> accepted; the row carried a
#     *negative* age, and ``net_rtt_estimate_s`` clamps the net estimate up to zero,
#     so the CSV read like a fast round trip.
#   * ``age_s = True``   -> ``float(True) == 1.0`` -> accepted; a bool was written as
#     "one second".
#   * ``sent_monotonic_s = True`` -> same conversion -> accepted; the stamp cell
#     read ``1.0``, i.e. a time at the very start of the monotonic epoch.
#   * ``received_monotonic_s = True`` -> same conversion -> accepted, and neither of
#     the two cases above covers it: a fix that judges ``sent_monotonic_s`` by type
#     could still write a ``1.0`` receive stamp.
#   * ``token = 123``    -> ``str(123) == "123"`` -> accepted; the token cell was not
#     a request token at all.
#   * ``token = "   "``  -> non-empty *text* -> accepted; the cell was blank.
#   * ``age_s = "abc"``  -> ``float("abc")`` raises ``ValueError`` ->
#     ``_finite_or_none`` returned ``None`` -> refused (exit 9, no CSV, ``age_s`` in
#     the missing-key clause) *before* the fix as well.  Kept as the one case of
#     this family that never needed the fix, so the family cannot be "implemented"
#     by refusing everything.
#   * ``age_s = 10**400`` / ``sent_monotonic_s = 10**400`` -> ``float()`` raises
#     ``OverflowError``, which ``_finite_or_none`` did not catch -> the probe blew up,
#     ``_run_rtt_mode``'s generic ``except Exception`` caught it, and the run exited
#     **3** with ``observation failed: int too large to convert to float``: a big
#     integer is a value that cannot be used, not a dead transport.  The two
#     ``huge_int_*`` cases pin it to the capability refusal (9, key named).
#
# The criterion this section locks (given by the task, not invented here): a key
# counts as *filled* iff it can give a finite, physically sensible value --
#
#   * ``token``: a ``str`` whose stripped text is not empty (``"abc"`` therefore
#     qualifies: a meaningless-but-real token is a *legal* shape, and the
#     pass-side test below pins it so it is not refused by an over-eager fix);
#   * ``sent_monotonic_s`` / ``received_monotonic_s``: a finite number, and a
#     ``bool`` is not a number;
#   * ``age_s``: a finite number ``>= 0``, and a ``bool`` is not a number.
#
# Three calibres that look wrong and are accepted on purpose (no test, so a future
# "tidy-up" knows they were decided, not missed):
#
#   * ``token`` may be **bytes**: ``_request_token_text`` decodes what the vendor
#     client stores, and every fixture in this file scripts bytes tokens;
#   * a **numeric string** (``sent_monotonic_s = "1000.0"``) is a usable number:
#     ``float()`` accepts it and ``_request_row`` writes the *number* into the CSV
#     cell, so no garbage-looking measurement is produced;
#   * ``age_s = -0.0`` is legal: ``-0.0 == 0.0``, i.e. the ``>= 0`` boundary.
#
# Any key failing that test -> exit ``EXIT_CLIENT_UNSUPPORTED`` (9), no CSV, and
# stderr names the reason *next to the key, inside the missing-key clause*
# (``_MISSING_KEYS_MARKER`` .. ``_MISSING_KEYS_TERMINATOR``, the two constants
# section V froze).  The clause boundary matters here for the same reason it
# matters in R20: the sentence after the terminator spells ``token`` / ``stamp``
# / ``age`` and the word "empty" out in prose, so a bare ``word in stderr`` would
# go green on boilerplate.
#
# **Item shape of the clause: ``<key>`` or ``<key> (<reason>)``, and the reason is
# mandatory for a value that is there.**  Only the optional *quoted value* inside
# the reason can be dropped, and ``_short_value`` (约
# benchmarks/alohamini_io/measure_latency.py:624) gates it in this order: ``repr()``
# longer than 32 characters -> not quoted; quote containing ``", "`` -> not quoted;
# quote not printable -> not quoted; otherwise ``<reason> (<quote>)``.  A quote that
# raises must be caught before any of those gates (R34), or the refusal degrades
# into a transport failure.
#
# How this coexists with R20 (``test_main_should_refuse_rtt_when_the_timing_face_misses_a_key``):
# R20 parses the clause with ``_missing_key_names`` and requires it to be exactly
# ``{dropped_key}``, but it *drops* keys (the value is absent) and the reader strips
# the `` (<reason>)`` suffix, so a bare list and an annotated one satisfy the same
# equality.  What tells them apart is ``_key_carries_a_reason`` (R33): an absent key
# stays bare, a present-but-unusable one explains itself.
#
# Not duplicated here: the all-four-keys and four-plus-extra-keys pass sides are
# already held by ``test_main_should_measure_rtt_when_the_timing_face_carries_all_four_keys``
# (R21) and ``..._carries_extra_keys`` (R22).

# A face every row can be built from; each case below overrides exactly one value,
# so "refused" cannot be explained by any other key of the face.
_USABLE_TIMING_FACE: dict[str, Any] = {
    "token": b"tok-usable",
    "sent_monotonic_s": 1000.0,
    "received_monotonic_s": 1000.030,
    "age_s": 0.030,
}


class _ClientWithScriptedTiming(_FakeClient):
    """``_FakeClient`` whose timing face is ``_USABLE_TIMING_FACE`` + ``overrides``.

    The base fake rewrites ``latest_request_timing`` on every ``get_observation``;
    this subclass then replaces it with a fixed, fully usable face and applies the
    scripted override, so the one value under test keeps its scripted shape on
    *every* frame while the other three stay usable.  That is the difference from
    ``_ClientWithPartialTiming`` (section V): there a key is *absent*, here it is
    present and its value is the defect.

    ``dropped_keys`` removes keys from that face, which is how the mixed shape
    (one key absent, another present-but-unusable) is scripted: the two are
    different defects with different annotations, and a refusal has to list both.
    """

    def __init__(
        self,
        *,
        overrides: dict[str, Any],
        dropped_keys: tuple[str, ...] = (),
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._overrides = dict(overrides)
        self._dropped_keys = tuple(dropped_keys)

    def get_observation(self, include_cameras: bool = True) -> dict:
        """Take the frame the base fake builds, then install the scripted face."""
        state = super().get_observation(include_cameras=include_cameras)
        face = dict(_USABLE_TIMING_FACE)
        face.update(self._overrides)
        for key in self._dropped_keys:
            face.pop(key, None)
        self.latest_request_timing = face
        return state


# id -> (the one value scripted into the usable face, the word the refusal has to
# use for it inside the missing-key clause, or ``None`` when the wording is not
# pinned).  ``None`` is used for ``non_numeric_age`` and the two ``huge_int_*``
# cases on purpose: those shapes were refused before the fix too (``float("abc")``
# raises ``ValueError``) or are about *which* refusal they get, so this section has
# no reason to prescribe how the refusal words them -- only that the key is named.
_UNUSABLE_TIMING_CASES: dict[str, tuple[dict[str, Any], str | None]] = {
    "negative_age": ({"age_s": -0.5}, "negative"),
    "bool_age": ({"age_s": True}, "bool"),
    # Both stamps, not just the one the row prints first: ``age_s`` and
    # ``sent_monotonic_s`` being covered did not make a fix cover
    # ``received_monotonic_s``, and a missed stamp would be written as ``1.0`` (the
    # ``float(True)`` conversion) with no test going red.
    "bool_sent_stamp": ({"sent_monotonic_s": True}, "bool"),
    "bool_received_stamp": ({"received_monotonic_s": True}, "bool"),
    "non_numeric_age": ({"age_s": "abc"}, None),
    "non_string_token": ({"token": 123}, "not a string"),
    "blank_token": ({"token": "   "}, "empty"),
    # ``float(10**400)`` raises ``OverflowError`` (an ``ArithmeticError``, not the
    # ``TypeError``/``ValueError`` ``_finite_or_none`` caught), so these two used to
    # leave the probe as an exception and come back as exit 3.  No reason word is
    # prescribed: the requirement is "the same refusal as any other unusable
    # value", not a particular sentence for big integers.
    "huge_int_age": ({"age_s": 10**400}, None),
    "huge_int_sent_stamp": ({"sent_monotonic_s": 10**400}, None),
}


@pytest.mark.parametrize("case_id", list(_UNUSABLE_TIMING_CASES), ids=list(_UNUSABLE_TIMING_CASES))
def test_main_should_refuse_rtt_when_a_timing_value_is_not_usable(
    latency, tmp_path, capsys, case_id: str
) -> None:
    """R28: a present-but-unusable timing value is refused, with the reason named.

    Locked per case, and why each assertion is here:

    * exit ``EXIT_CLIENT_UNSUPPORTED`` (9): at HEAD 9a9f9d5 ``negative_age``,
      ``bool_age``, ``bool_sent_stamp``, ``bool_received_stamp``,
      ``non_string_token`` and ``blank_token`` exited 0 and wrote a CSV whose cells
      held a negative age / a ``bool`` turned into ``1.0`` / a non-token / a blank
      cell -- a file that reads like a measurement -- while the two ``huge_int_*``
      cases exited 3 (``OverflowError`` escaping the probe into the generic
      transport handler).  ``non_numeric_age`` is the one case that was refused at
      9a9f9d5 as well: it is here so the family cannot be "fixed" by refusing
      nothing at all.
    * the CSV does not exist: a refused run writes nothing, not even a header
      (R8/R11/R20 all freeze this).
    * stderr names the face (``latest_request_timing``) *and* the offending key
      *inside the missing-key clause*, and -- when this section prescribes a word
      for the shape -- that word is in the same clause.  "The client is broken"
      is not actionable; "``age_s`` is negative" is.  The clause boundary keeps the
      check off the boilerplate sentence ("... would write empty token, stamp and
      age cells ..."), which contains "empty" and the three key-adjacent nouns.
    * the key named is the one that was scripted unusable: the face carries all
      four keys with three of them usable, so a refusal that names a different key
      is reporting the wrong cell.

    Driven through ``main()`` on purpose (section T/V style): the criterion is
    about what the *CLI* refuses, and the exit code plus "no CSV" only exist
    there.  ``--samples 3`` / 200 Hz / ``--timeout-s 0.3`` matches R20, so a
    refusal may look at one or two frames without the test waiting on the budget.
    """
    overrides, reason_word = _UNUSABLE_TIMING_CASES[case_id]
    client = _ClientWithScriptedTiming(overrides=overrides, ages=[0.030, 0.031, 0.032])
    out = tmp_path / f"unusable-{case_id}.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"a timing face whose {', '.join(overrides)} cannot be used as a measurement "
        f"was accepted: exit {code} (expected 9), csv_exists={out.exists()}; "
        f"stderr: {captured.err!r}"
    )
    assert not out.exists(), (
        f"a run refused for an unusable timing value still wrote {out}: the row built "
        f"from {overrides} holds a cell that reads like a measurement"
    )
    assert "latest_request_timing" in captured.err, (
        f"stderr does not name the timing face: {captured.err!r}"
    )
    assert _MISSING_KEYS_MARKER in captured.err, (
        "the refusal never says which key is unusable, so the operator cannot tell "
        f"which cell is wrong: {captured.err!r}"
    )
    clause = _missing_key_clause(captured.err)
    for key in overrides:
        assert key in clause, (
            f"the missing-key clause does not name {key!r}, the key whose value was "
            f"scripted unusable: clause={clause!r}, stderr={captured.err!r}"
        )
    if reason_word is not None:
        assert reason_word in clause.lower(), (
            f"the refusal does not say *why* {', '.join(overrides)} is unusable (looked "
            f"for {reason_word!r} inside the missing-key clause, not merely somewhere in "
            f"stderr): clause={clause!r}, stderr={captured.err!r}"
        )


def test_main_should_measure_rtt_when_age_is_zero(latency, tmp_path) -> None:
    """R29: ``age_s = 0.0`` is the boundary of "finite and ``>= 0``" -- measured.

    The pass side of the ``age_s`` rule, and the reason it cannot be written as
    ``age_s > 0``: a host that timestamps a request with zero measurable age
    reports ``0.0``, which is a legitimate value.  A refusal here would reject a
    legal client; the row's ``age_s`` cell stays non-empty so the value is carried
    into the CSV instead of being dropped.
    """
    client = _ClientWithScriptedTiming(overrides={"age_s": 0.0}, ages=[0.0])
    out = tmp_path / "zero-age.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, (
        f"age_s == 0.0 is a legal age (the boundary of the >= 0 rule), got exit {code}"
    )
    assert out.is_file(), f"the rtt run wrote no CSV at {out}"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 3, f"expected one row per requested sample, got {len(rows)}"
    for row in rows:
        assert row["age_s"] != "", f"row {row['sample_index']} has an empty age_s: {row}"


def test_main_should_measure_rtt_when_age_is_an_integer(latency, tmp_path) -> None:
    """R29: an ``int`` age is a number -- measured, not refused as "not a float".

    Python hands back an ``int`` from plenty of paths (``0``, ``2``, a JSON
    round trip), and ``float(2)`` is a finite number, so "finite number ``>= 0``"
    has to accept it.  A rule written as ``isinstance(value, float)`` would refuse
    this client; the pass side is what keeps the fix from becoming a type test.
    """
    client = _ClientWithScriptedTiming(overrides={"age_s": 2}, ages=[2])
    out = tmp_path / "integer-age.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "2",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, (
        f"an integer age_s is a finite number (age_s >= 0 holds), got exit {code}"
    )
    assert out.is_file(), f"the rtt run wrote no CSV at {out}"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2, f"expected one row per requested sample, got {len(rows)}"
    for row in rows:
        assert row["age_s"] != "", f"row {row['sample_index']} has an empty age_s: {row}"


def test_main_should_measure_rtt_when_the_token_is_a_meaningless_string(latency, tmp_path) -> None:
    """R29: any non-blank ``str`` token is *filled* -- the criterion is text, not sense.

    ``"abc"`` is not a request id the host would ever issue, and that is
    deliberate: the probe judges whether the cell holds text, never whether the
    text is meaningful (it has no way to know what the host would issue).  Pinning
    this shape keeps the fix from turning into a validator of token contents --
    the shapes that must be refused are "not a string" and "blank", both scripted
    in R28 above.  The token also has to survive into the CSV cell verbatim: a
    refusal-free run that drops the token would pass an exit-code-only assertion.
    """
    client = _ClientWithScriptedTiming(overrides={"token": "abc"}, ages=[0.030, 0.031])
    out = tmp_path / "meaningless-token.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "2",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )

    assert code == latency.EXIT_OK, (
        f"a non-blank str token is a filled token cell (only 'not a str' and 'blank' "
        f"are refused), got exit {code}"
    )
    assert out.is_file(), f"the rtt run wrote no CSV at {out}"
    with out.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2, f"expected one row per requested sample, got {len(rows)}"
    for row in rows:
        assert row["token"] == "abc", (
            f"row {row['sample_index']} does not carry the scripted token: {row}"
        )


# The reason for an unusable value is printed *inside* the missing-key clause
# (``_UNUSABLE_TIMING_CASES`` above), and R20 reads that clause back by splitting
# it on ``", "``: an annotated entry that itself holds the separator reads as two
# keys, so a refusal would name a key that does not exist.  The same clause is cut
# at the first newline (``_missing_key_clause``), so a reason that brought one
# would be truncated.
#
# The contract that resolves this (see ``_short_value``, 约
# benchmarks/alohamini_io/measure_latency.py:624) is not "rewrite the quote until
# it is printable" but "**drop the quote, keep the reason**":
#
#   * the *reason* is mandatory and is a fixed phrase (``not a finite number``,
#     ``negative value``, ...), so it can never carry a separator or a line break;
#   * the *quoted value* appended to it is optional, and the gates decide, in this
#     order: ``repr()`` longer than 32 characters -> not quoted; the quote contains
#     ``", "`` -> not quoted; the quote is not printable (any real newline) -> not
#     quoted; otherwise it is appended as ``<reason> (<quote>)``;
#   * gates that short-circuit matter: a value is checked for length before it is
#     checked for printability, so a 401-digit integer is rejected for length
#     without its quote ever being split, and a value whose ``repr()`` *raises*
#     (``_ExplodingRepr``, R34) must be caught before any of this, or the refusal
#     turns into a transport failure.
#
# So what the shapes below have to produce is not a wording but a *parseable* list
# with an annotation: ``_missing_key_names(stderr) == {"<the one key>"}`` and
# ``_key_carries_a_reason(stderr, "<the one key>")`` -- R20's reader plus the
# contract marker, applied to a value-shape refusal.


class _MultiLineReprStamp:
    r"""A stamp whose ``repr`` spans lines -- the shape that truncates the clause.

    ``repr`` escapes line breaks inside a ``str``/``bytes`` (``repr("a\nb")`` is
    ``"'a\nb'"``), so no *string* value can put a real newline into the clause and a
    ``token = "a\nb"`` case would prove nothing.  What does put one there is an
    object whose ``repr`` is genuinely multi-line -- a numpy scalar array, a wrapped
    exception, a dataclass holding a message: exactly the shapes a client can leave
    on a timing field.
    """

    def __repr__(self) -> str:
        return "stamp(\n  1.0\n)"


def _refusal_for_hostile_value(
    latency, tmp_path, capsys, *, overrides: dict[str, Any], name: str
) -> tuple[int, Path, str]:
    """Run ``main()`` with one hostile timing value; return ``(code, csv_path, stderr)``.

    Shared by the two clause-parseability tests below so the *shapes* stay in them
    and the driving of the CLI stays in one place: ``--samples 3`` / 200 Hz /
    ``--timeout-s 0.3`` as in R20/R28, so a refusal costs one or two frames.
    """
    client = _ClientWithScriptedTiming(overrides=overrides, ages=[0.030, 0.031, 0.032])
    out = tmp_path / f"hostile-{name}.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    return code, out, capsys.readouterr().err


def test_main_should_keep_the_missing_key_clause_parseable_when_the_rejected_value_contains_a_comma(
    latency, tmp_path, capsys
) -> None:
    """R32: a value holding ``", "`` is not quoted, and the reason survives without it.

    ``sent_monotonic_s = "1.0,  2.0"`` is two numbers in one string, i.e. not a
    stamp, so the run is refused.  Quoting the value would make the clause read
    ``sent_monotonic_s (not a finite number ('1.0, 2.0'))``, and R20 reads that
    clause back by splitting on ``", "``, so the quote would arrive as a second key
    (``"2.0'))"``) -- a key the face does not have.

    The contract asserted here is ``_short_value``'s: the quote is dropped, the
    reason kept, so the clause reads ``sent_monotonic_s (not a finite number)``.
    Exit 9 (a *value* defect, not a transport one), no CSV, and
    ``_missing_key_names(stderr) == {"sent_monotonic_s"}`` -- R20's own clause
    reader on a value-shape refusal, so exactly one key and no prose residue.
    ``_key_carries_a_reason`` is checked by R33; here the point is that the
    *unparseable* shape is gone.
    """
    code, out, stderr = _refusal_for_hostile_value(
        latency,
        tmp_path,
        capsys,
        overrides={"sent_monotonic_s": "1.0,  2.0"},
        name="comma",
    )

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"a timing face whose sent_monotonic_s is not a number must be refused with "
        f"exit 9, got {code}; stderr: {stderr!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"
    names = _missing_key_names(stderr)
    assert names == {"sent_monotonic_s"}, (
        "the missing-key clause does not read back as exactly the one unusable key: "
        f"parsed {sorted(names)!r} from clause {_missing_key_clause(stderr)!r} "
        f'(stderr: {stderr!r}); a reason that carries ", " splits the clause, so the '
        "refusal names keys that do not exist"
    )


def test_main_should_keep_the_missing_key_clause_parseable_when_the_rejected_value_contains_a_newline(
    latency, tmp_path, capsys
) -> None:
    r"""R32: an unprintable value is not quoted, so the clause and the refusal survive.

    ``repr`` escapes line breaks inside a ``str``/``bytes`` (``repr("a\nb")`` is
    ``"'a\nb'"``), so a string value cannot reach this shape; what does is an
    object whose ``repr`` is genuinely multi-line (``_MultiLineReprStamp``: a numpy
    scalar array, a wrapped exception, a dataclass holding a message -- all shapes a
    client can leave on a timing field).  Quoting it would put a real newline inside
    the clause, which ``_missing_key_clause`` cuts at, so the clause would read
    ``sent_monotonic_s (not a finite number (stamp(`` -- the key name survives but
    the list ends mid-word, and the ``", so ..."`` remediation sentence is pushed off
    the first line with it.

    The contract asserted here is ``_short_value``'s: a quote that is not printable
    is dropped and the reason kept, so the clause reads ``sent_monotonic_s (not a
    finite number)``.  Locked: exit 9, no CSV, ``_missing_key_names(stderr) ==
    {"sent_monotonic_s"}``, and the ``", so "`` terminator still present -- both
    true at once only if the newline never reaches the refusal, which is what "do
    not quote it" buys.
    """
    code, out, stderr = _refusal_for_hostile_value(
        latency,
        tmp_path,
        capsys,
        overrides={"sent_monotonic_s": _MultiLineReprStamp()},
        name="newline",
    )

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"a timing face whose sent_monotonic_s is not a number must be refused with "
        f"exit 9, got {code}; stderr: {stderr!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"
    names = _missing_key_names(stderr)
    assert names == {"sent_monotonic_s"}, (
        "the missing-key clause does not read back as exactly the one unusable key: "
        f"parsed {sorted(names)!r} from clause {_missing_key_clause(stderr)!r} "
        f"(stderr: {stderr!r}); a reason that spans lines is cut at the first newline"
    )
    assert _MISSING_KEYS_TERMINATOR in stderr, (
        'the newline in the reason ate the rest of the refusal (the ", so ..." clause '
        f"terminator and the remediation sentence after it): {stderr!r}"
    )


# The annotation contract, in one place: every item of the missing-key clause is
# ``<key>`` or ``<key> (<reason>)``, and a key that is *there* with an unusable
# value always carries the reason -- a bare ``age_s`` in the clause says "the face
# does not carry this cell", which is the wrong fix for a cell that is carried and
# holds ``-0.5``.  Only the optional *quoted value* inside the reason can be
# dropped (``_short_value``: unprintable, longer than 32 characters, or holding
# ``", "``), never the reason itself.
#
# The readers are the upgraded ``_missing_key_names`` (key names, reason stripped)
# and ``_key_carries_a_reason`` (the ``"<key> ("`` marker), i.e. the two halves are
# asserted separately: a refusal that annotates the wrong key, or none at all,
# fails one of them.


@pytest.mark.parametrize("case_id", list(_UNUSABLE_TIMING_CASES), ids=list(_UNUSABLE_TIMING_CASES))
def test_main_should_name_a_reason_for_every_unusable_timing_value(
    latency, tmp_path, capsys, case_id: str
) -> None:
    """R33: every unusable value is refused *and* explained, next to its own key.

    Same nine shapes as R28 (``_UNUSABLE_TIMING_CASES``), read with the contract
    readers instead of the per-case word: exit 9, no CSV, the clause's key names
    are exactly ``{key}``, and that key carries a `` (<reason>)`` annotation.

    Why the reason is not optional here: the two ``huge_int_*`` values are refused
    with a bare key by an implementation that drops the whole annotation when the
    value cannot be quoted (the value is a 401-digit integer, past
    ``_short_value``'s 32-character gate), so the operator is told "``age_s`` is
    missing" for a face that carries ``age_s = 10**400`` -- actionable in the wrong
    direction.  ``non_numeric_age`` is the mirror: it was refused before this
    section existed, and it still has to say *why*.

    Not asserted: which word the reason uses (R28 pins ``negative`` / ``bool`` /
    ``not a string`` / ``empty`` where the wording matters and leaves it open for
    the huge-integer and non-numeric shapes) and whether a value is quoted, which
    is ``_short_value``'s call.
    """
    overrides, _reason_word = _UNUSABLE_TIMING_CASES[case_id]
    (key,) = overrides
    client = _ClientWithScriptedTiming(overrides=overrides, ages=[0.030, 0.031, 0.032])
    out = tmp_path / f"annotated-{case_id}.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        f"{key}={overrides[key]!r:.48} on the timing face must be refused with exit 9, "
        f"got {code}; stderr: {captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"
    names = _missing_key_names(captured.err)
    assert names == {key}, (
        f"the missing-key clause does not read back as exactly {key!r}: "
        f"parsed {sorted(names)!r} from clause {_missing_key_clause(captured.err)!r} "
        f"(stderr: {captured.err!r})"
    )
    assert _key_carries_a_reason(captured.err, key), (
        f"the refusal names {key!r} without saying why it is unusable: clause "
        f"{_missing_key_clause(captured.err)!r} (stderr: {captured.err!r}); an "
        "operator sent to a cell that the face *does* carry cannot act on it"
    )


def test_main_should_name_both_keys_when_one_is_missing_and_one_is_unusable(
    latency, tmp_path, capsys
) -> None:
    """R33: an absent key and an unusable one are both listed, annotated only once.

    ``token`` is dropped from the face and ``age_s`` carries ``-0.5``: two
    different defects in one refusal.  The clause has to read ``token, age_s
    (negative value (-0.5))`` -- the absent key bare (there is nothing to explain:
    the cell would be empty because the value is not there) and the unusable one
    explained.

    Locked: exit 9, no CSV, ``_missing_key_names`` is exactly the *set*
    ``{"token", "age_s"}`` (the order is ``REQUIRED_REQUEST_TIMING_KEYS``'s
    business, not the operator's), the reason marker sits on ``age_s``, and
    ``token`` carries *no* marker -- both halves of the item shape in one clause, so
    a refusal that annotates every item (or none) fails here.  This is the case
    R20's equality cannot see: the reader strips `` (<reason>)``, so a clause that
    explains an absent key still parses to the right key names.
    """
    client = _ClientWithScriptedTiming(
        overrides={"age_s": -0.5},
        dropped_keys=("token",),
        ages=[0.030, 0.031, 0.032],
    )
    out = tmp_path / "missing-and-unusable.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        "a face with one absent and one unusable key must be refused with exit 9, "
        f"got {code}; stderr: {captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"
    names = _missing_key_names(captured.err)
    assert names == {"token", "age_s"}, (
        f"the refusal does not list both defects: parsed {sorted(names)!r} from clause "
        f"{_missing_key_clause(captured.err)!r} (stderr: {captured.err!r})"
    )
    assert _key_carries_a_reason(captured.err, "age_s"), (
        "the unusable value (age_s = -0.5) is listed without a reason, so it reads "
        "like a cell the face does not carry: clause "
        f"{_missing_key_clause(captured.err)!r}"
    )
    # The other half, on the same clause: the *absent* key stays bare.  Asserting
    # only "age_s is annotated" would be satisfied by a refusal that annotates every
    # item, which is the shape R20's new bare-key assertion rejects for the
    # one-defect case; here both shapes are in one clause, so both are checked.
    assert "token (" not in _missing_key_clause(captured.err), (
        "the absent key 'token' is annotated in the missing-key clause: clause "
        f"{_missing_key_clause(captured.err)!r}; a key the face does not carry has no "
        "value to explain"
    )


class _ExplodingRepr:
    """A timing value that cannot even be printed: ``repr()`` raises.

    Not exotic: ``__repr__`` is client code, and the shapes that reach a timing
    field (a lazily formatted object, a proxy holding a closed resource, a numpy
    scalar whose workspace is gone) can raise.  The refusal must survive that: a
    refusal that turns into a transport failure because it tried to quote the value
    it was refusing reports the wrong cause (exit 3, "observation failed") and
    loses the key name with it.
    """

    def __repr__(self) -> str:
        raise RuntimeError("repr is not available")


def test_main_should_refuse_rtt_when_a_rejected_value_cannot_be_printed(
    latency, tmp_path, capsys
) -> None:
    """R34: a value whose ``repr`` raises is refused as a *value*, not a transport.

    ``sent_monotonic_s = _ExplodingRepr()`` is unusable (it is not a number), so
    the run owes the same refusal as any other value defect: exit 9, no CSV, the
    key named with its reason.  An implementation that quotes the value
    unconditionally raises out of the probe instead, ``_run_rtt_mode``'s generic
    ``except Exception`` catches it as ``observation failed: repr is not
    available`` and returns ``EXIT_CONNECT_FAILED`` (3) -- the operator is sent to
    the link, and the key that caused it is never named.

    Locked: exit 9, no CSV, ``_missing_key_names == {"sent_monotonic_s"}`` and the
    reason marker on that key.  Whether the reason quotes the value is not asserted
    (it cannot be printed, so the only correct answer is "do not quote it").
    """
    client = _ClientWithScriptedTiming(
        overrides={"sent_monotonic_s": _ExplodingRepr()}, ages=[0.030, 0.031, 0.032]
    )
    out = tmp_path / "unprintable-value.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "3",
            "--rate-hz",
            "200",
            "--timeout-s",
            "0.3",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_CLIENT_UNSUPPORTED, (
        "a timing value that cannot be printed is still a *value* defect: expected "
        f"exit 9, got {code}; stderr: {captured.err!r}"
    )
    assert not out.exists(), f"a refused run wrote {out}"
    names = _missing_key_names(captured.err)
    assert names == {"sent_monotonic_s"}, (
        f"the refusal does not name the unprintable key: parsed {sorted(names)!r} from "
        f"clause {_missing_key_clause(captured.err)!r} (stderr: {captured.err!r})"
    )
    assert _key_carries_a_reason(captured.err, "sent_monotonic_s"), (
        f"the refusal names the key without a reason: clause {_missing_key_clause(captured.err)!r}"
    )


# ===========================================================================
# Z. Mid-run freeze and duplicate frames: what the pacing costs
# ===========================================================================
#
# Two run shapes whose cost is decided by the pacing arithmetic alone.  Row
# deadlines are absolute and a poll that grew no row only has to wait until the
# next one is due (约 benchmarks/alohamini_io/measure_latency.py:1751-1758):
#
#   ``next_row_s = start_s + len(rows) * period_s``
#   ``target_s   = next_row_s                            # a fresh frame``
#   ``target_s   = max(next_row_s, now + IDLE_POLL_FLOOR_S)  # duplicate, rows > 0``
#   ``target_s   = now + idle_pause_s         # duplicate, no row yet (R24's case)``
#   ``idle_pause_s = max(period_s, IDLE_POLL_FLOOR_S)``   (约 :1689, floor ``1e-3``,
#                                                         ``IDLE_POLL_FLOOR_S`` :470)
#
# Before the fix (HEAD 9a9f9d5) the middle arm did not exist: a duplicate frame
# waited ``now_s + idle_pause_s``, one whole period, so the achieved rate was
# ``rate_hz / (1 + duplicates per growth)``.  Both sets of numbers are recorded
# because the difference is the whole point of the two tests below (sleep-driven
# clock, 20 rows, ``--rate-hz 200``):
#
#   duplicates per growth |    9a9f9d5      |   current
#   ----------------------+------------------+------------------
#   1                     | 0.1000 s, 1.00 x | 0.0960 s, 1.04 x (39 polls)
#   2                     | 0.1950 s, 0.51 x | 0.0970 s, 1.03 x (58 polls)
#
#   (current: 206-208 Hz, i.e. slightly *above* ``--rate-hz``, because a duplicate
#   waits only until the next row is due and the absolute deadline lets the loop
#   catch up; 9a9f9d5: 200.0 Hz and 102.6 Hz respectively.)
#
# 1. **A sequence that grows N times and then stands still** has no test of its
#    own elsewhere: R24 freezes at *zero* rows and
#    ``test_main_should_report_insufficient_samples_in_rtt_mode`` freezes at one.
#    The shape worth locking is the one an operator actually meets (three good
#    frames, then a dead link), because it is the only one where rows *and* the
#    give-up moment are both observable: ``exc.collected`` has to be the number of
#    rows that made it into ``rows_out``, and the polls have to stay paced.  A
#    frozen tail keeps chasing the same ``next_row_s`` deadline, so the arm above
#    spends its wait one ``IDLE_POLL_FLOOR_S`` at a time: measured 73 polls for a
#    100 ms budget at 100 Hz (9a9f9d5: 11 polls), while a run frozen with *zero*
#    rows still costs 11 (the ``idle_pause_s`` arm, unchanged -- that is R24's
#    bound, and both sit far under it).
# 2. **A sequence that repeats** is the shape the honest host produces when it
#    answers slower than the client polls.  It is worth its own test because the
#    cost depends on the *number* of duplicates per growth, not on "slower than the
#    client": one duplicate per growth was already absorbed by the fresh frame's
#    absolute deadline at 9a9f9d5 (1.00 x), and two were not (0.51 x, i.e. exactly
#    ``rate_hz / 3``).  R31 parametrises both, so the rule is locked in one place,
#    and the defect it describes is "the duplicate poll waits a whole period" --
#    the loop's own pacing decides the achieved rate, which is why such a fix
#    belongs in ``collect_rtt_samples`` and not in the fake.
#
# Both use ``_SleepDrivenClock`` (section W): the clock moves only when the loop
# sleeps, so "how long this run took" is exactly "how much the loop waited" and
# the assertions are about pacing, never about wall time.  ``_SpinGuardClient``
# turns a loop that stops sleeping into a ``RuntimeError`` with a call count in it
# instead of a hung test.
#
# The quarter of ``_FakeClient``'s scriptable knobs these two need: ``sequences``
# saturates on its last entry (``_value_at``), so ``sequences=[1, 2, 3]`` is
# "grow three times, then frozen" without a new fake.

# Poll bound for a run whose sequence stops growing after a few rows ("frozen
# tail"): the loop owes at most one wait per sampling period, and a legitimate
# implementation may poll once before and once after each wait, so
# ``2 * timeout_s * rate_hz`` polls; the ``+ 20`` covers the growth phase, the
# final poll and the frame a refusal looks at.
#
# The shape it separates from is a loop that waits ``IDLE_POLL_FLOOR_S`` (1 ms)
# instead of the period: at 100 Hz that is 10 polls per period, i.e.
# ``timeout_s / 1e-3`` polls over the budget -- 1000 for a 1 s budget, 4.5x this
# bound, and 29,903 measured for the default 30 Hz / 30 s run (bound 1820).  So
# "one wait per period" and "one wait per millisecond" cannot be confused, which
# is what the two bounds below are for.
_FROZEN_TAIL_POLL_SLACK = 20


def _frozen_tail_poll_bound(*, rate_hz: float, timeout_s: float) -> int:
    """Polls a loop paced at ``rate_hz`` may spend on a budget of ``timeout_s``.

    Two per period (see ``_FROZEN_TAIL_POLL_SLACK``) plus a fixed slack: 40 for the
    100 ms @ 100 Hz case, 220 for 1 s @ 100 Hz.
    """
    return int(2 * timeout_s * rate_hz) + _FROZEN_TAIL_POLL_SLACK


# The capability refusal's wording, which a budget refusal must not borrow (the
# mirror of section W's ``_BUDGET_WORDS``): "never fills it" and the missing-key
# clause are what sections T/V print when the *client* is at fault.
_CAPABILITY_WORDS = ("never fills it", "missing key(s):")


def test_collect_rtt_samples_should_lock_the_mid_run_freeze_behaviour(latency) -> None:
    """R30: three growths, then a frozen sequence -> three rows, paced, then 7.

    Locks the shape an operator meets in the field (a link that dies mid-run)
    rather than R24's idealised zero-row freeze:

    * the run still ends in ``InsufficientSamples`` -- a dead link is a budget
      problem, not a capability one, and the timing face is filled here;
    * ``exc.collected == 3`` and ``len(rows_out) == 3``: the three samples taken
      before the freeze are kept (they are the only evidence the run produced) and
      the count the refusal reports agrees with the list the caller holds;
    * ``get_observation`` stays paced to **one wait per sampling period**: at most
      ``2 * timeout_s * rate_hz + 20`` = **40** calls for a 100 ms budget at 100 Hz
      (``_frozen_tail_poll_bound``).  Measured 73 before that bound existed: the
      frozen tail keeps chasing the same ``next_row_s`` deadline, so the
      ``IDLE_POLL_FLOOR_S`` arm fires once per millisecond -- 10 polls per period at
      100 Hz, i.e. a 1 kHz poll loop ("waiting" only in the sense that it sleeps 1 ms
      at a time).  The bound is two polls per period, so a fix that waits the period
      fits and a floor-driven loop does not; the spin guard sits far above it (2000),
      so a guard firing is reported as a failure instead of being read as "bounded".
      The 1 s variant of the same shape is
      ``test_collect_rtt_samples_should_not_poll_a_frozen_tail_more_often_than_one_period``.
    """
    samples = 10
    rate_hz = 100.0
    timeout_s = 0.1
    upper_bound = _frozen_tail_poll_bound(rate_hz=rate_hz, timeout_s=timeout_s)  # 40
    clock = _SleepDrivenClock()
    rows: list[dict[str, Any]] = []
    client = _SpinGuardClient(limit=2000, sequences=[1, 2, 3])

    raised: BaseException | None = None
    try:
        latency.collect_rtt_samples(
            client,
            samples=samples,
            rate_hz=rate_hz,
            timeout_s=timeout_s,
            sleep=clock.sleep,
            clock=clock,
            rows_out=rows,
        )
    except Exception as exc:  # the guard's RuntimeError is the interesting failure
        raised = exc

    calls = client.get_observation_calls

    assert isinstance(raised, latency.InsufficientSamples), (
        "a sequence that grows three times and then stands still must end in "
        f"InsufficientSamples (the timing face is filled, so this is a budget "
        f"refusal, not a capability one); got {type(raised).__name__}({raised}) after "
        f"{calls} get_observation calls and {len(rows)} row(s)"
    )
    assert raised.required == samples, (
        f"the refusal reports required={raised.required}, the run asked for {samples}"
    )
    assert raised.collected == 3, (
        f"the refusal reports collected={raised.collected}, but three frames grew "
        f"before the freeze (rows_out holds {len(rows)})"
    )
    assert len(rows) == 3, (
        f"rows_out holds {len(rows)} row(s) instead of the three samples taken before "
        f"the freeze: the evidence of a dead link is exactly what this run keeps"
    )
    assert calls <= upper_bound, (
        f"a mid-run freeze cost {calls} get_observation calls in a {timeout_s} s budget "
        f"at {rate_hz} Hz (at most {upper_bound}, i.e. two polls per sampling period): "
        "the frozen tail is waited on one IDLE_POLL_FLOOR_S at a time, so the loop "
        "polls at 1 kHz instead of at --rate-hz"
    )


def test_collect_rtt_samples_should_not_poll_a_frozen_tail_more_often_than_one_period(
    latency,
) -> None:
    """R35: a frozen tail waits the sampling period, not ``IDLE_POLL_FLOOR_S``.

    Same shape as R30 (three growths, then the sequence stands still) with the
    budget the CLI actually defaults to: ``rate_hz=100`` and ``timeout_s=1.0``.  A
    frozen tail keeps ``next_row_s`` at ``start + 3 * period``, so every poll finds
    the deadline in the past; the duplicate arm clamps that to
    ``now + IDLE_POLL_FLOOR_S`` (1 ms), and the loop then polls ten times per
    sampling period -- 973 calls for one second of budget.  The default run
    (30 Hz, 30 s) measured 29,903 calls: a "waiting" loop that spends the whole
    budget waking up 1,000 times per second.

    Bound: ``2 * timeout_s * rate_hz + 20`` = **220** (``_frozen_tail_poll_bound``).
    Derivation: a loop that owes one wait per period costs ``timeout_s * rate_hz``
    = 100 polls for this budget; the factor 2 covers a poll before and after each
    wait; the ``+ 20`` covers the three growing frames, the give-up poll and the
    frame a refusal looks at.  A pace of one poll per ``IDLE_POLL_FLOOR_S`` costs
    ``timeout_s / 1e-3`` = 1000 polls here (4.5x the bound) and 29,903 in the
    default shape (16x its bound of 1820), so the two shapes cannot be confused --
    and a fix that waits the period lands near 100, comfortably inside.

    The fix this pins is the one the R31 duplicate arm cannot make on its own: while
    the loop is still catching up its beat, waiting ``next_row_s`` is right; once it
    is a whole period behind (the frozen tail), the wait has to go back to a full
    period, or the floor turns "paced" into "1 kHz".  R31's two cases are asserted
    unchanged, since the catch-up arm is untouched by that fallback.

    Asserted: exit ``InsufficientSamples`` with the three rows kept (a dead link is
    a budget problem, and the evidence survives), and the poll bound.
    """
    rate_hz = 100.0
    timeout_s = 1.0
    samples = 10
    upper_bound = _frozen_tail_poll_bound(rate_hz=rate_hz, timeout_s=timeout_s)  # 220
    clock = _SleepDrivenClock()
    rows: list[dict[str, Any]] = []
    client = _SpinGuardClient(limit=20000, sequences=[1, 2, 3])

    raised: BaseException | None = None
    try:
        latency.collect_rtt_samples(
            client,
            samples=samples,
            rate_hz=rate_hz,
            timeout_s=timeout_s,
            sleep=clock.sleep,
            clock=clock,
            rows_out=rows,
        )
    except Exception as exc:  # the guard's RuntimeError is the interesting failure
        raised = exc

    calls = client.get_observation_calls

    assert isinstance(raised, latency.InsufficientSamples), (
        "a frozen tail with a filled timing face ends in InsufficientSamples (a "
        f"budget refusal, not a capability one); got {type(raised).__name__}"
        f"({raised}) after {calls} get_observation calls and {len(rows)} row(s)"
    )
    assert len(rows) == 3, (
        f"the three rows collected before the freeze must be kept, got {len(rows)}"
    )
    assert calls <= upper_bound, (
        f"a frozen tail cost {calls} get_observation calls in a {timeout_s} s budget at "
        f"{rate_hz} Hz (at most {upper_bound}, i.e. two polls per sampling period): the "
        "loop waits one IDLE_POLL_FLOOR_S per poll once the row deadline is in the "
        "past, so a 100 Hz run polls at 1 kHz"
    )


# ``rate_hz`` shapes that make ``period_s`` unusable: ``1.0 / rate_hz if rate_hz > 0
# else 0.0`` is ``0.0`` for nan (no comparison holds), inf (``1 / inf``) and for the
# non-positive values.  The frozen-tail arm has to keep waiting something even then,
# because there is no period to fall back on.
_NO_USABLE_PERIOD_CASES: dict[str, float] = {
    "nan": float("nan"),
    "inf": float("inf"),
    "zero": 0.0,
    "negative": -5.0,
}


@pytest.mark.parametrize(
    "case_id", list(_NO_USABLE_PERIOD_CASES), ids=list(_NO_USABLE_PERIOD_CASES)
)
def test_collect_rtt_samples_should_still_wait_when_a_frozen_tail_has_no_usable_period(
    latency, case_id: str
) -> None:
    """R36: a frozen tail waits the floor when ``rate_hz`` gives no period at all.

    R35 bounds a frozen tail at one wait per *period*; this case is the degenerate
    side of the same arm.  ``period_s = 1.0 / rate_hz if rate_hz > 0 else 0.0``
    (约 benchmarks/alohamini_io/measure_latency.py:1683) is ``0.0`` for all four
    parameters here -- ``nan`` compares false, ``1 / inf`` is 0, and ``0`` / ``-5``
    take the ``else`` -- so a frozen-tail arm that waits ``period_s`` waits
    *nothing*: every poll finds the row deadline in the past, the pause is zero,
    ``sleep`` is never called and the loop polls at the client's maximum speed
    until the budget expires.  The clock is sleep-driven, so such a loop never
    advances the fake clock either: it would only end on the spin guard.

    Asserted, because each one fails for a different reason:

    * ``InsufficientSamples`` -- the run still has to end, and a filled timing face
      with a frozen sequence is a budget refusal, not a capability one;
    * ``len(rows_out) == 3`` -- the three samples taken before the freeze survive;
    * ``get_observation`` calls ``<= 200`` -- measured 53 per parameter (50 floor
      waits + 3 growth polls), so the bound is ~4x the paced count; a busy loop
      runs to the guard (5000) and is reported, not hung.  The bound is on the
      *count*, i.e. it separates "waits the floor" from "waits nothing";
    * every ``sleep`` argument is ``> 0`` and ``min(sleeps) >= IDLE_POLL_FLOOR_S -
      1e-12`` -- the direct statement of the defect, and the one assertion a
      zero-length wait cannot satisfy.  ``timeout_s = 0.05`` is 50 whole floors
      deliberately, so the last wait is a full floor too rather than the remainder
      of the budget (measured minimum: ``1e-3 - 3.4e-17``, i.e. float noise well
      inside the 1e-12 tolerance).
    """
    rate_hz = _NO_USABLE_PERIOD_CASES[case_id]
    timeout_s = 0.05
    samples = 5
    floor_s = latency.IDLE_POLL_FLOOR_S
    upper_bound = 200
    clock = _SleepDrivenClock()
    rows: list[dict[str, Any]] = []
    client = _SpinGuardClient(limit=5000, sequences=[1, 2, 3])

    raised: BaseException | None = None
    try:
        latency.collect_rtt_samples(
            client,
            samples=samples,
            rate_hz=rate_hz,
            timeout_s=timeout_s,
            sleep=clock.sleep,
            clock=clock,
            rows_out=rows,
        )
    except Exception as exc:  # the guard's RuntimeError is the interesting failure
        raised = exc

    calls = client.get_observation_calls
    sleeps = clock.sleeps

    assert isinstance(raised, latency.InsufficientSamples), (
        f"rate_hz={rate_hz!r} with a frozen tail must end in InsufficientSamples; got "
        f"{type(raised).__name__}({raised}) after {calls} get_observation calls, "
        f"{len(rows)} row(s) and {len(sleeps)} sleep(s)"
    )
    assert len(rows) == 3, (
        f"the three rows collected before the freeze must be kept, got {len(rows)}"
    )
    assert calls <= upper_bound, (
        f"rate_hz={rate_hz!r} cost {calls} get_observation calls in a {timeout_s} s "
        f"budget (at most {upper_bound}; the paced shape is ~53): period_s is 0.0, so "
        "an arm that waits the period waits nothing and the loop spins"
    )
    assert sleeps, (
        f"rate_hz={rate_hz!r} issued no sleep at all in {len(sleeps)} polls: the loop "
        "is not waiting between polls"
    )
    assert all(pause_s > 0.0 for pause_s in sleeps), (
        f"rate_hz={rate_hz!r} issued a zero-length wait: sleeps={sleeps[:5]!r}..."
    )
    assert min(sleeps) >= floor_s - 1e-12, (
        f"rate_hz={rate_hz!r} waited {min(sleeps)!r} s, below the IDLE_POLL_FLOOR_S "
        f"floor ({floor_s!r}): with no usable period the frozen tail has to fall back "
        "to the floor, not to a zero pause"
    )


def test_main_should_report_insufficient_samples_when_the_sequence_freezes_mid_run(
    latency, tmp_path, capsys
) -> None:
    """R30: the same mid-run freeze through ``main()`` -> exit 7, no CSV on disk.

    The CLI half of R30: the operator sees exit ``EXIT_INSUFFICIENT_SAMPLES`` (7)
    and stderr's "insufficient samples", i.e. the documented wording for a budget
    that ran out (the capability refusal's wording is section W's business, and it
    must *not* appear here -- the face is filled).

    The CSV question is answered by what the run actually does, and the answer is
    "no file at all": ``_run_rtt_mode`` prints the refusal and returns before
    ``write_csv`` (约 benchmarks/alohamini_io/measure_latency.py:1885-1887), so the
    three rows collected before the freeze never reach disk -- they are discarded
    with the run.  That is the same shape as the one-row freeze
    (``test_main_should_report_insufficient_samples_in_rtt_mode``) and it is
    asserted, not wished away: "the CSV does not exist" is the observable
    consequence of the row list being local to the mode function.

    ``--timeout-s 0.05`` at 100 Hz: the run waits ~10 ms per poll, so the budget
    is over after a handful of polls and the test costs real time only because
    ``main`` has no injectable clock (the same trade R8/R20 make).
    """
    client = _FakeClient(sequences=[1, 2, 3])
    out = tmp_path / "mid-run-freeze.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--mode",
            "rtt",
            "--samples",
            "10",
            "--rate-hz",
            "100",
            "--timeout-s",
            "0.05",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == latency.EXIT_INSUFFICIENT_SAMPLES, (
        f"a sequence that freezes after three growths must exit 7, got {code}; "
        f"stderr: {captured.err!r}"
    )
    assert "insufficient samples" in captured.err.lower(), (
        f"stderr does not report the sampling budget as the cause: {captured.err!r}"
    )
    assert not any(word in captured.err.lower() for word in _CAPABILITY_WORDS), (
        "a filled timing face plus a frozen sequence is a budget problem, so the "
        f"capability wording must not appear: {captured.err!r}"
    )
    assert not out.exists(), (
        f"the run wrote {out}: the rows collected before the freeze are counted by the "
        "refusal (R30) but the mode function returns before write_csv, so no CSV -- not "
        "even a header-only one -- is the current shape"
    )


class _DuplicateFrameClient(_SpinGuardClient):
    """``_FakeClient`` whose ``observation_sequence`` grows once every ``1 + m`` frames.

    The honest-host shape: the client polls faster than the host answers, so one
    growth is followed by ``m`` frames that repeat the sequence number
    (``m = 0`` is "every frame grows").  ``_FakeClient`` overwrites the sequence on
    every call, so the patterned value is installed *after* the base call, from the
    call counter the base keeps (``get_observation_calls`` is already incremented
    when it returns): call ``n`` reports ``1 + (n - 1) // (1 + m)``, i.e.
    ``1, 1, 1, 2, 2, 2, ...`` for ``m = 2``, and the very first frame grows
    (otherwise the run would look like section T's never-filled case).

    Extends ``_SpinGuardClient`` so a loop that stops sleeping under this pattern
    turns into a ``RuntimeError`` with a call count instead of a hang.
    """

    def __init__(self, *, duplicates: int, limit: int = 5000) -> None:
        super().__init__(limit=limit, sequences=[0])
        self._duplicates = duplicates

    def get_observation(self, include_cameras: bool = True) -> dict:
        """Answer the frame, then report the duplicate-frame sequence pattern."""
        state = super().get_observation(include_cameras=include_cameras)
        self._sequence = 1 + (self.get_observation_calls - 1) // (1 + self._duplicates)
        return state


# id -> duplicates per growth.  ``one_duplicate`` is the shape the fresh frame's
# absolute deadline already absorbed before the fix (it is kept as a regression
# guard); ``two_duplicates`` is the shape where the wait was actually paid
# (``rate_hz / 3`` at 9a9f9d5), which is what the fix had to remove.
_DUPLICATE_THROTTLE_CASES: dict[str, int] = {
    "one_duplicate": 1,
    "two_duplicates": 2,
}


@pytest.mark.parametrize(
    "case_id", list(_DUPLICATE_THROTTLE_CASES), ids=list(_DUPLICATE_THROTTLE_CASES)
)
def test_collect_rtt_samples_should_not_let_duplicate_frames_throttle_the_row_rate(
    latency, case_id: str
) -> None:
    """R31: a host that answers slower than the client polls still runs at the rate.

    A duplicate frame has no sampling instant of its own, and the question is what
    it waits: charging it a whole period (``now_s + idle_pause_s``, the arithmetic
    at 9a9f9d5) makes an honest host that answers one growth every ``1 + m`` frames
    cost ``m`` periods per row, i.e. ``rate_hz / (1 + m)``.  Waiting only until the
    next row is due (``start_s + len(rows) * period_s``, floored at
    ``IDLE_POLL_FLOOR_S``) costs one period per *row* and nothing for the
    duplicates.

    Measured (sleep-driven clock, 20 rows, ``--rate-hz 200``), 9a9f9d5 -> current:

    * ``one_duplicate`` (m = 1): 0.1000 s / 200.0 Hz / **1.00 x** (39 polls, 21
      pauses) -> 0.0960 s / 208.3 Hz / **1.04 x** (39 polls).  The fresh frame's
      absolute deadline already absorbed the single duplicate, so this case is the
      regression guard: a rewrite that polls once per frame, or one that charges a
      duplicate a full period again, drops it below the bound.
    * ``two_duplicates`` (m = 2): 0.1950 s / 102.6 Hz / **0.51 x** (58 polls, 39
      pauses) -> 0.0970 s / 206.2 Hz / **1.03 x** (58 polls).  This is the case
      that made the defect visible: the loop throttled itself to ``rate_hz / 3``.

    The ratio sits slightly above 1.00 for a *denominator* reason, not a pacing
    one: this clock is sleep-driven, so a ``get_observation`` costs the run zero
    simulated time and ``N`` rows span only ``N - 1`` periods -- 20 rows over
    19 periods is theoretically ``1 / 0.95 = 1.0526``.  The measured 1.03-1.04 is
    that ceiling minus the overshoot a floored wait leaves per row; a genuinely
    faster loop is not what the number says.

    Asserted: the run completes (20 rows) and ``len(rows) / clock.now`` is at least
    ``0.9 * rate_hz``.  The clock is sleep-driven, so ``clock.now`` is exactly the
    time the loop spent waiting and the ratio is a pacing measurement, not a
    wall-clock one.  The 10% slack is for a legal implementation that lands a hair
    under the nominal rate (a floor, an extra poll); one third of the rate is
    nowhere near it, so the two shapes cannot be confused.  The failure message
    carries the measured ratio, the duration and the poll count.
    """
    duplicates = _DUPLICATE_THROTTLE_CASES[case_id]
    samples = 20
    rate_hz = 200.0
    timeout_s = 60.0
    clock = _SleepDrivenClock()
    rows: list[dict[str, Any]] = []
    client = _DuplicateFrameClient(duplicates=duplicates)

    raised: BaseException | None = None
    try:
        latency.collect_rtt_samples(
            client,
            samples=samples,
            rate_hz=rate_hz,
            timeout_s=timeout_s,
            sleep=clock.sleep,
            clock=clock,
            rows_out=rows,
        )
    except Exception as exc:  # the guard's RuntimeError is the interesting failure
        raised = exc

    calls = client.get_observation_calls
    elapsed_s = clock.now

    assert raised is None, (
        f"{duplicates} duplicate frame(s) per growth must not end the run: got "
        f"{type(raised).__name__}({raised}) after {calls} get_observation calls, "
        f"{len(rows)} of {samples} row(s), {elapsed_s:.4f} s of simulated wait"
    )
    assert len(rows) == samples, f"expected {samples} rows, got {len(rows)}"
    assert elapsed_s > 0.0, (
        "the run never slept, so the paced duration is 0 and the achieved rate is "
        f"undefined ({calls} get_observation calls)"
    )
    achieved_hz = len(rows) / elapsed_s

    assert achieved_hz >= 0.9 * rate_hz, (
        f"with {duplicates} duplicate frame(s) per growth the run achieved "
        f"{achieved_hz:.1f} Hz, only {achieved_hz / rate_hz:.0%} of --rate-hz "
        f"{rate_hz:.0f} Hz: {len(rows)} rows took {elapsed_s:.4f} s of simulated wait "
        f"in {calls} get_observation calls, because every duplicate frame waits a "
        "whole period (idle_pause_s) instead of pacing the next sampling deadline"
    )


# ===========================================================================
# R28-R31: the rest of the numeric knobs reject non-finite values, and the
# queue/prefetch terms have exactly one reading
# ===========================================================================
#
# ``--rate-hz`` / ``--timeout-s`` / ``--control-hz`` already carry the
# ``not (math.isfinite(x) and x > 0.0)`` shape (R25/R26).  The four knobs below
# were left with a sign-only check, which ``nan`` (every comparison false) and
# ``inf`` (``> 0`` is true) both walk through.  The tests here pin the same
# shape for them, plus the one thing the shape cannot fix on its own: the CSV
# path and the report path used to normalise ``host_loop_hz`` differently, so a
# *direct* caller's ``nan`` produced ``queue_bound_s=0.04`` in the rows and
# ``queue_bound_ms=0.000`` in the report of the same run.
#
# ``--delta-deg`` is the one knob of the four whose sign bound is **not**
# ``> 0.0``: a negative step is a legal run (the opposite direction of the same
# experiment), so its finiteness test is wrapped around ``!= 0`` and the
# ``MAX_STEP_DELTA_DEG`` cap is applied to the magnitude.  Section ``AA`` owns
# that contract; the R31 tests here only keep the non-finite and zero refusals.


@pytest.mark.parametrize("value", ["nan", "inf", "1e400"], ids=["nan", "inf", "overflow_to_inf"])
def test_parse_args_should_reject_a_non_finite_host_loop_hz(latency, capsys, value: str) -> None:
    """R28: a non-finite ``--host-loop-hz`` is a usage error (2), never a run.

    ``inf`` is ``> 0`` and ``nan`` makes ``<= 0.0`` false, so the sign-only check
    let all three through (``1e400`` *is* ``inf`` after ``float()``).  ``inf``
    then makes ``phase_bound_s`` and ``queue_bound_s`` collapse to ``0.0``, and
    ``nan`` is read two ways at once: the CSV path falls back to
    ``HOST_LOOP_HZ_UPPER_BOUND`` while the report path hands the raw ``nan`` to
    ``effective_request_window``, whose ``rate_hz >= nan`` is false -- so the same
    run prints ``queue_bound_s=0.04`` in the CSV and ``queue_bound_ms=0.000`` in
    the report.  The parser refuses the value before any socket is opened, which
    is the only entry of the knob, so neither path can ever see it.

    Asserted: exit code 2 and the option name on stderr, the shape R25 uses.  The
    legal value is parsed first on purpose -- an unknown option also exits 2, so
    without that guard this test would go green on an option that no longer
    exists.
    """
    # Arrange: the option exists and a legal value reaches the namespace.
    legal = latency.parse_args(["--host", "127.0.0.1", "--host-loop-hz", "20"])
    assert getattr(legal, "host_loop_hz", None) == pytest.approx(20.0), (
        "--host-loop-hz 20 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    # Act + Assert: the non-finite value is refused by the parser itself.
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--host-loop-hz={value}"])

    assert excinfo.value.code == 2, (
        f"--host-loop-hz={value} exited with {excinfo.value.code} instead of the usage "
        "error 2: a non-finite host loop rate makes the phase/queue terms collapse to "
        "0.0 (inf) or be read two contradictory ways (nan)"
    )
    message = capsys.readouterr().err.lower()
    assert "host-loop-hz" in message, f"the error does not name the option: {message!r}"


@pytest.mark.parametrize("value", ["nan", "inf", "1e400"], ids=["nan", "inf", "overflow_to_inf"])
def test_parse_args_should_reject_a_non_finite_settle_timeout_s(
    latency, capsys, value: str
) -> None:
    """R29: a non-finite ``--settle-timeout-s`` is a usage error (2).

    The knob is one stop condition of a tracking run; ``inf`` never expires and
    ``nan`` makes every ``now - started >= budget`` comparison false, so a joint
    that never settles runs until the operator kills the process.  The legal
    value is parsed first for the same reason as in R25.
    """
    legal = latency.parse_args(["--host", "127.0.0.1", "--settle-timeout-s", "5"])
    assert getattr(legal, "settle_timeout_s", None) == pytest.approx(5.0), (
        "--settle-timeout-s 5 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--settle-timeout-s={value}"])

    assert excinfo.value.code == 2, (
        f"--settle-timeout-s={value} exited with {excinfo.value.code} instead of the "
        "usage error 2: a non-finite settle budget never expires"
    )
    message = capsys.readouterr().err.lower()
    assert "settle-timeout-s" in message, f"the error does not name the option: {message!r}"


@pytest.mark.parametrize("value", ["nan", "inf", "1e400"], ids=["nan", "inf", "overflow_to_inf"])
def test_parse_args_should_reject_a_non_finite_settle_tolerance_deg(
    latency, capsys, value: str
) -> None:
    """R30: a non-finite ``--settle-tolerance-deg`` is a usage error (2).

    ``nan`` makes ``residual >= tolerance`` false forever (no step is ever
    settled, so every row keeps its settle columns empty) and ``inf`` settles the
    first poll (``rise_90_time_s`` empty in every row) -- both silently, which is
    what the cross-check against the effective ``--delta-deg`` cannot catch
    because ``nan >= delta`` and ``delta >= inf`` are false too.  ``0`` stays
    legal: the semantics are "settled when the residual is at or below the
    tolerance" (``test_parse_args_should_keep_a_zero_settle_tolerance_legal``).
    """
    legal = latency.parse_args(["--host", "127.0.0.1", "--settle-tolerance-deg", "0.2"])
    assert getattr(legal, "settle_tolerance_deg", None) == pytest.approx(0.2), (
        "--settle-tolerance-deg 0.2 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--settle-tolerance-deg={value}"])

    assert excinfo.value.code == 2, (
        f"--settle-tolerance-deg={value} exited with {excinfo.value.code} instead of the "
        "usage error 2: a non-finite tolerance settles every step (inf) or none (nan)"
    )
    message = capsys.readouterr().err.lower()
    assert "settle-tolerance-deg" in message, f"the error does not name the option: {message!r}"


def test_parse_args_should_keep_a_zero_settle_tolerance_legal(latency) -> None:
    """R30: ``--settle-tolerance-deg 0`` stays legal -- the bound is ``>= 0``.

    Nothing is tightened here on purpose: the documented semantics are "a step
    counts as settled when the residual is at or below the tolerance", so ``0``
    is the strictest *legal* setting, not a nonsense one.
    """
    args = latency.parse_args(["--host", "127.0.0.1", "--settle-tolerance-deg", "0"])

    assert args.settle_tolerance_deg == pytest.approx(0.0)


def _parser_error_text(stderr: str) -> str:
    """The ``error:`` line of an argparse refusal, lowercased.

    Matching the whole stderr is vacuous here: ``parser.error`` prints the usage
    block first, and that block lists **every** option name, so
    ``"settle-tolerance-deg" in stderr`` is true for a run that failed on
    ``--delta-deg`` (and true for an option that no longer exists, as long as the
    usage string still mentions it).  Only the ``error:`` line says *why* the
    parser refused, so that is the line an assertion may talk about.
    """
    lines = [line for line in stderr.splitlines() if "error:" in line]
    assert lines, f"argparse printed no error line: {stderr!r}"
    return lines[-1].lower()


@pytest.mark.parametrize(
    "value",
    ["nan", "inf", "1e400", "-inf"],
    ids=["nan", "inf", "overflow_to_inf", "negative_inf"],
)
def test_parse_args_should_reject_a_non_finite_delta_deg(latency, capsys, value: str) -> None:
    """R31: a non-finite ``--delta-deg`` is a usage error (2).

    ``inf`` passes the sign test (it is non-zero); ``resolve_delta_deg`` then
    clamps it to ``MAX_STEP_DELTA_DEG``, so the run silently commands a step the
    operator never asked for instead of refusing a value that cannot be a step.
    ``nan`` makes ``<= 0.0`` false *and* ``!= 0`` true and survives to the clamp,
    i.e. every step-size comparison in the tracking path is false.  ``-inf`` is
    the negative twin the magnitude clamp must not be able to "fix": a sign-only
    ``!= 0`` test would let it through, so ``math.isfinite`` is what catches it
    (both infinities have to be refused for the same reason: neither is a step).
    ``1e400`` is ``inf`` after ``float()``.  The legal value is parsed first for
    the same reason as in R25.

    Note on the argument form: every variant is passed as ``--delta-deg=<value>``.
    That is required, not stylistic: argparse only classifies a leading-dash token
    as a *value* when it matches its negative-number pattern
    (``^-<digits>$`` or ``^-<digits>.<digits>$``), which ``-inf`` does not -- as a
    separate token it would be read as an unknown option and fail with "expected one
    argument" instead of the refusal asserted here.
    """
    legal = latency.parse_args(["--host", "127.0.0.1", "--delta-deg", "0.5"])
    assert getattr(legal, "delta_deg", None) == pytest.approx(0.5), (
        "--delta-deg 0.5 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--delta-deg={value}"])

    assert excinfo.value.code == 2, (
        f"--delta-deg={value} exited with {excinfo.value.code} instead of the usage "
        "error 2: a non-finite step is silently clamped (inf) or poisons every "
        "step-size comparison (nan)"
    )
    message = _parser_error_text(capsys.readouterr().err)
    assert "delta-deg" in message, f"the error does not name the option: {message!r}"


@pytest.mark.parametrize("value", ["0", "-0.0"], ids=["positive_zero", "negative_zero"])
def test_parse_args_should_still_reject_a_zero_delta_deg(latency, capsys, value: str) -> None:
    """R31/AA: ``--delta-deg 0`` / ``-0.0`` stay usage errors (2) -- no regression.

    This replaced ``test_parse_args_should_reject_a_non_positive_delta_deg``, whose
    ``"-1"`` arm is now obsolete: a **non-zero negative** step is a legal input
    (the opposite direction of the same experiment), so the refusal it pins is the
    zero one only -- see the negative-``--delta-deg`` section (``AA``) below for
    ``-0.5`` / ``-1.0`` / ``-10``.  The finiteness check is added *around* this
    rule, not instead of it: a zero step commands nothing, so it keeps its own
    refusal (and its own wording) rather than being folded into the cross-check
    that would blame the clamp for a value the operator typed.

    Both zeros are passed as ``--delta-deg=<value>``: ``-0.0`` does match
    argparse's negative-number pattern, but the ``=`` form keeps the two arms of
    this test in one shape and pins the value the operator typed.
    """
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", f"--delta-deg={value}"])

    assert excinfo.value.code == 2, (
        f"--delta-deg={value} exited with {excinfo.value.code} instead of the usage "
        "error 2: a zero step commands nothing"
    )
    message = _parser_error_text(capsys.readouterr().err)
    assert "delta-deg" in message, f"the error does not name the option: {message!r}"


_QUEUE_TERM_PATTERN = re.compile(
    r"(queued_window|queue_bound_ms|prefetch_bound_ms|queue_mean_ms|prefetch_mean_ms)=([0-9.]+)"
)


def _report_queue_terms(report: str) -> dict[str, float]:
    """The queue/prefetch cells of the report's ``request_window=...`` line.

    Parsed from the text on purpose: the point of the test is what the operator
    *sees*, so an implementation that computes the right number and prints
    another one has to fail.
    """
    line = next((line for line in report.splitlines() if "queued_window=" in line), None)
    assert line is not None, f"the report prints no queue/prefetch line: {report!r}"
    terms = dict(_QUEUE_TERM_PATTERN.findall(line))
    assert len(terms) == 5, f"the report line does not state all five terms: {line!r}"
    return {key: float(value) for key, value in terms.items()}


def _rtt_row(latency, args, host_loop_hz: float) -> dict:
    """One CSV-shaped row for ``args``, exactly as ``_request_row`` builds it."""
    return latency.net_rtt_estimate_s(
        0.030,
        {},
        host_loop_hz=host_loop_hz,
        request_window=int(args.request_window),
        rate_hz=float(args.rate_hz),
    )


@pytest.mark.parametrize(
    ("extra_argv", "expected_queued_window"),
    [
        (["--rate-hz", "50", "--host-loop-hz", "20", "--request-window", "3"], 3),
        (["--rate-hz", "10", "--host-loop-hz", "50", "--request-window", "3"], 1),
    ],
    ids=["queue_gate_open", "queue_gate_closed"],
)
def test_rtt_report_should_print_the_same_queue_terms_as_the_rows(
    latency, extra_argv: list[str], expected_queued_window: int
) -> None:
    """R28: the report subtracts exactly the queue/prefetch term the CSV row does.

    The report's own comment promises that it may not print a queue term the CSV
    did not subtract, so the two are derived from the same ``--rate-hz`` /
    ``--host-loop-hz`` pair.  Two shapes, because the gate has to agree in both
    directions: at or above the host rate the window stays queued (2 / 20 Hz =
    100 ms over a 3-request window) and below it the queue term is genuinely
    absent (0 ms), which is the case a "print the raw window" implementation
    would get wrong in the opposite direction.
    """
    args = latency.parse_args(["--host", "127.0.0.1", *extra_argv])
    row = _rtt_row(latency, args, float(args.host_loop_hz))

    terms = _report_queue_terms(latency._format_rtt_report([row], args, 1.0))

    assert terms["queued_window"] == expected_queued_window, (
        f"the report gates the window differently than the rows: {terms}"
    )
    assert terms["queue_bound_ms"] == pytest.approx(row["queue_bound_s"] * 1000.0, abs=1e-6)
    assert terms["queue_mean_ms"] == pytest.approx(row["queue_mean_s"] * 1000.0, abs=1e-6)
    assert terms["prefetch_bound_ms"] == pytest.approx(row["prefetch_bound_s"] * 1000.0, abs=1e-6)
    assert terms["prefetch_mean_ms"] == pytest.approx(row["prefetch_mean_s"] * 1000.0, abs=1e-6)


@pytest.mark.parametrize("host_loop_hz", [float("nan"), 0.0, -1.0], ids=["nan", "zero", "negative"])
def test_rtt_report_should_agree_with_the_rows_for_an_unusable_host_loop_hz(
    latency, host_loop_hz: float
) -> None:
    """R28: a direct caller's unusable ``host_loop_hz`` is read one way, not two.

    The CLI refuses a non-finite ``--host-loop-hz``, so this is the residual
    path: ``net_rtt_estimate_s`` (and the CSV it feeds) falls back to
    ``HOST_LOOP_HZ_UPPER_BOUND`` when the value cannot be a period, while
    ``_format_rtt_report`` used to hand the raw value to
    ``effective_request_window``.  For ``nan`` that gate is ``rate_hz >= nan``,
    i.e. always false, so the same row reported ``queue_bound_s=0.04`` and
    ``queue_bound_ms=0.000`` at once.  Both sides now go through one
    normalisation, so the report's queue and prefetch terms equal the row's.
    """
    args = latency.parse_args(["--host", "127.0.0.1", "--rate-hz", "50", "--request-window", "3"])
    args.host_loop_hz = host_loop_hz
    row = _rtt_row(latency, args, host_loop_hz)
    # The fallback is the upper bound, which the 50 Hz request rate keeps up with:
    # the window stays queued, exactly as ``net_rtt_estimate_s`` computed it.
    assert row["queue_bound_s"] == pytest.approx(2 / latency.HOST_LOOP_HZ_UPPER_BOUND)

    terms = _report_queue_terms(latency._format_rtt_report([row], args, 1.0))

    assert terms["queued_window"] == 3, (
        f"the report read host_loop_hz={host_loop_hz!r} differently than the rows: {terms}"
    )
    assert terms["queue_bound_ms"] == pytest.approx(row["queue_bound_s"] * 1000.0, abs=1e-6)
    assert terms["queue_mean_ms"] == pytest.approx(row["queue_mean_s"] * 1000.0, abs=1e-6)
    assert terms["prefetch_bound_ms"] == pytest.approx(row["prefetch_bound_s"] * 1000.0, abs=1e-6)
    assert terms["prefetch_mean_ms"] == pytest.approx(row["prefetch_mean_s"] * 1000.0, abs=1e-6)


# ===========================================================================
# AA. A negative ``--delta-deg``: the M2 entry (the same step, taken *down*)
# ===========================================================================
#
# Field manual §10.5 asks for the step-response measurement in both directions:
# a joint that only ever steps up leaves the sign handling of every downstream
# judgement (the target sum, the 90 % crossing, the overshoot, the settle
# residual, the travel cap and the tolerance cross-check) untested, and the
# plant's behaviour is not symmetric either.
#
# Contract frozen here:
#
# 1. ``parse_args`` accepts a **finite non-zero** ``--delta-deg`` of either sign.
#    ``0`` / ``-0.0`` stay usage errors (``test_parse_args_should_still_reject_a_zero_delta_deg``,
#    section R31) and so do ``nan`` / ``inf`` / ``-inf`` / ``1e400``.
# 2. ``resolve_delta_deg`` clamps the **magnitude**:
#    ``min(max(delta, -MAX_STEP_DELTA_DEG), MAX_STEP_DELTA_DEG)``.  The safety
#    cap stays 1 deg per step in either direction, and the sign of the request
#    survives the clamp.  The old one-armed ``min(delta, cap)`` is a no-op for
#    every negative value, i.e. ``--delta-deg -10`` would command 10 deg.
# 3. The settle-tolerance cross-check and Gate E's travel cap compare the
#    **magnitude** as well: ``abs(delta)`` against the tolerance, and
#    ``abs(delta) * steps`` against ``MAX_TOTAL_TRAVEL_DEG``.  A signed
#    comparison would either be vacuous (``0.15 >= -1.0``) or let 21 negative
#    steps through as "-21 deg of travel".
# 4. ``run_tracking_steps`` already works in the negative direction (its target,
#    crossing and overshoot judgements are ``abs()``-based); the row-level tests
#    below freeze that so a later "simplification" that drops an ``abs()``
#    turns red instead of silently reporting a rise time of ``None``.
#
# Argument forms, deliberately: a negative *number* works both as
# ``--delta-deg -0.5`` and as ``--delta-deg=-0.5``, because argparse's
# ``_negative_number_matcher`` reclassifies a token matching ``^-\d+$`` /
# ``^-\d*\.\d+$`` as a value (no option string of this parser looks like a
# negative number).  ``-inf`` matches neither pattern, so it is *only* usable in
# the ``=`` form -- see R31's non-finite test.  Both forms of a plain negative
# number are pinned below, since an operator will type the space form.


@pytest.mark.parametrize(
    ("argv", "expected"),
    [
        pytest.param(["--delta-deg", "-0.5"], -0.5, id="space_form_half_degree"),
        pytest.param(["--delta-deg", "-1.0"], -1.0, id="space_form_one_degree"),
        pytest.param(["--delta-deg=-0.5"], -0.5, id="equals_form_half_degree"),
    ],
)
def test_parse_args_should_accept_a_negative_delta_deg(
    latency, argv: list[str], expected: float
) -> None:
    """AA1: a negative step is a legal input, not a usage error (2).

    The old rule was ``math.isfinite(x) and x > 0.0``: it refused every negative
    ``--delta-deg`` in the parser, so the whole M2 direction was unreachable from
    the CLI even though ``run_tracking_steps`` handles it.
    """
    args = latency.parse_args(["--host", "127.0.0.1", *argv])

    assert args.delta_deg == pytest.approx(expected), (
        f"{argv} parsed into delta_deg={args.delta_deg!r} instead of {expected!r}"
    )


def test_parse_args_should_report_a_negative_delta_deg_without_its_sign_lost(latency) -> None:
    """AA1: the parsed namespace carries the sign, not just the magnitude.

    ``--delta-deg -0.5`` must not arrive as ``0.5``: the direction of the run is
    the whole point of the knob, and a ``abs()`` applied at parse time would be
    invisible in every later ``abs()``-based judgement.
    """
    args = latency.parse_args(["--host", "127.0.0.1", "--delta-deg", "-0.5"])

    assert args.delta_deg < 0.0


def test_resolve_delta_deg_should_pass_a_negative_value_within_the_cap_through(latency) -> None:
    """AA2: a legal negative step is passed through untouched, like ``0.25`` is."""
    requested = latency.parse_args(["--host", "127.0.0.1", "--delta-deg", "-0.5"])

    assert latency.resolve_delta_deg(requested) == pytest.approx(-0.5)


@pytest.mark.parametrize(
    ("text", "sign"),
    [("-10", -1.0), ("10", +1.0)],
    ids=["negative_request", "positive_request"],
)
def test_resolve_delta_deg_should_clamp_the_magnitude_in_both_directions(
    latency, text: str, sign: float
) -> None:
    """AA2: the 1 deg cap applies to the magnitude, and the sign survives it.

    ``min(delta, MAX_STEP_DELTA_DEG)`` is the wrong shape for a signed knob: it
    clamps ``+10`` to ``+1`` but returns ``-10`` unchanged, so the negative arm
    would command a step ten times the documented safety cap.  The two arms here
    pin the symmetric clamp
    ``min(max(delta, -MAX_STEP_DELTA_DEG), MAX_STEP_DELTA_DEG)``; the existing
    ``3.0 -> 1.0`` and ``0.25 -> 0.25`` cases (round-3 J7) stay untouched and pin
    that the positive side did not regress.
    """
    requested = latency.parse_args(["--host", "127.0.0.1", "--delta-deg", text])

    clamped = latency.resolve_delta_deg(requested)

    assert clamped == pytest.approx(latency.MAX_STEP_DELTA_DEG * sign), (
        f"--delta-deg {text} resolved to {clamped!r}, not "
        f"{latency.MAX_STEP_DELTA_DEG * sign!r}: the clamp is one-armed"
    )


def test_resolve_delta_deg_should_clamp_a_large_negative_request_to_minus_the_cap(latency) -> None:
    """AA2: the brief's exact shape -- ``-10`` must not stay ``-10``.

    Same rule as the parametrised test above, written out separately because this
    is the value that makes the one-armed clamp's failure mode unambiguous (a
    ten-degree step on a real robot instead of a one-degree one).
    """
    requested = latency.parse_args(["--host", "127.0.0.1", "--delta-deg", "-10"])

    assert latency.resolve_delta_deg(requested) == pytest.approx(latency.MAX_STEP_DELTA_DEG * -1)


def test_parse_args_should_accept_a_settle_tolerance_below_a_negative_step(latency) -> None:
    """AA3: ``--delta-deg -1.0 --settle-tolerance-deg 0.15`` is the legal default pair.

    The cross-check compares the tolerance against the *magnitude* of the
    effective step.  Written as ``tolerance >= effective_delta`` it is vacuously
    true for every negative step (``0.15 >= -1.0``), so the negative direction
    would be refused for the one combination that is legal -- and the same
    signed comparison would accept ``-0.05`` with a ``0.15`` tolerance, which is
    the empty-``rise_90`` case the rule exists to prevent.
    """
    args = latency.parse_args(
        [
            "--host",
            "127.0.0.1",
            "--delta-deg",
            "-1.0",
            "--settle-tolerance-deg",
            "0.15",
        ]
    )

    assert latency.resolve_delta_deg(args) == pytest.approx(-1.0)
    assert args.settle_tolerance_deg == pytest.approx(0.15)


@pytest.mark.parametrize(
    ("delta_deg", "tolerance"),
    [("-1.0", "1.0"), ("-0.05", "0.15")],
    ids=["tolerance_equals_the_magnitude", "tolerance_above_the_magnitude"],
)
def test_parse_args_should_reject_a_settle_tolerance_at_or_above_a_negative_steps_magnitude(
    latency, capsys, delta_deg: str, tolerance: str
) -> None:
    """AA3: the tolerance is judged against ``abs(delta)``, not against ``delta``.

    Both arms are the *same* knobs as N2/N3 with the sign flipped, so they must
    be refused for the same reason and with the same wording: the tolerance wins
    the settle race and ``rise_90`` stays empty in every row.  The error names
    ``settle-tolerance`` (not ``delta-deg``): blaming the step for a tolerance
    the operator typed would point at the wrong knob.
    """
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(
            [
                "--host",
                "127.0.0.1",
                "--delta-deg",
                delta_deg,
                "--settle-tolerance-deg",
                tolerance,
            ]
        )

    assert excinfo.value.code == 2, (
        f"--delta-deg {delta_deg} with --settle-tolerance-deg {tolerance} exited with "
        f"{excinfo.value.code} instead of the usage error 2"
    )
    message = _parser_error_text(capsys.readouterr().err)
    assert "settle-tolerance" in message, f"the refusal does not name the tolerance: {message!r}"


def _negative_tracking_argv(out: Path, *, steps: str, delta_deg: str = "-1.0") -> list[str]:
    """A complete ``main``-level tracking run that steps *down* by ``delta_deg``."""
    return [
        "--host",
        "127.0.0.1",
        "--mode",
        "tracking",
        "--enable-motion",
        "--joint",
        "arm_left_shoulder_pan",
        "--delta-deg",
        delta_deg,
        "--steps",
        steps,
        "--settle-timeout-s",
        "0.05",
        "--rate-hz",
        "200",
        "--control-hz",
        "50",
        "--csv",
        str(out),
    ]


def test_main_should_run_a_negative_step_end_to_end(latency, tmp_path, capsys) -> None:
    """AA4: Gate E must not refuse a negative step -- the command really goes out.

    Gate E's first arm was ``delta_deg <= 0.0``, i.e. "``--delta-deg`` and
    ``--steps`` must both be positive".  That made every negative run a
    ``EXIT_MOTION_GATE_REFUSED`` with zero motion.  Asserted: exit 0, a non-empty
    ``sent_actions``, and a first step goal that is *below* the measured baseline
    in normalized units (the fake fixes the feedback at one unit on a 0.879
    deg/unit joint, so the direction is visible on the wire).
    """
    client = _TrackingFake()
    factory = _Factory(client)
    out = tmp_path / "tracking-down.csv"

    code = latency.main(_negative_tracking_argv(out, steps="2"), client_factory=factory)
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, (
        f"the negative tracking run exited with {code} instead of 0: {printed.err!r}"
    )
    assert client.sent_actions, "the negative run issued no command at all"
    baseline_units = _TRACKING_STATE[_TRACKING_JOINT]
    expected_units = baseline_units + latency.units_from_deg(-1.0, _TRACKING_SCALE_DEG)
    first_goal = client.sent_actions[0][_TRACKING_JOINT]
    assert first_goal == pytest.approx(expected_units, rel=0, abs=1e-9), (
        f"the first command asked for {first_goal!r} instead of {expected_units!r} "
        f"(baseline {baseline_units!r} on a {_TRACKING_SCALE_DEG!r} deg/unit joint)"
    )
    assert first_goal < baseline_units, "the step goal was not below the baseline"


def test_main_should_judge_the_travel_cap_by_magnitude_for_a_negative_step(
    latency, tmp_path, capsys
) -> None:
    """AA4: 21 x (-1.0 deg) is -21 deg of travel, i.e. over the 20 deg cap.

    The signed comparison ``-21.0 > 20.0`` is false, so the negative arm of the
    travel cap lets the run through and commands a 21-degree march in the
    direction nothing is looking at.  Asserted: the gate's own exit code, the
    ``Gate E`` wording on stderr and zero commands.
    """
    client = _TrackingFake()
    factory = _Factory(client)
    out = tmp_path / "tracking-down-cap.csv"

    code = latency.main(_negative_tracking_argv(out, steps="21"), client_factory=factory)
    printed = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED, (
        f"21 x -1.0 deg exited with {code} instead of {latency.EXIT_MOTION_GATE_REFUSED} "
        f"(the travel cap is judged on the signed value): {printed.err!r}"
    )
    assert client.sent_actions == [], (
        f"a run over the travel cap commanded {len(client.sent_actions)} action(s)"
    )
    assert "Gate E" in printed.err, f"the refusal does not name Gate E: {printed.err!r}"
    # Not just "Gate E refused": the *cap* arm has to be the one that fired.  A
    # signed comparison either lets this run through (``-21.0 > 20.0`` is false) or
    # the old sign arm refuses it with "must both be positive" -- which would make
    # every assertion above pass while the magnitude rule is still missing.
    assert "cap" in printed.err.lower(), (
        f"the refusal does not name the travel cap: {printed.err!r}"
    )
    assert "positive" not in printed.err.lower(), (
        'Gate E refused on the sign arm ("must both be positive") instead of on the '
        f"travel cap: {printed.err!r}"
    )


def test_run_tracking_rows_should_command_below_the_baseline_for_a_negative_delta(latency) -> None:
    """AA5: ``cmd_deg`` / ``cmd_units`` are ``baseline + delta`` with the sign kept.

    The joint's scale is deliberately not 1 (0.879 deg/unit), so an implementation
    that mixed degrees and units, or that dropped the sign of the converted
    step, cannot pass: ``cmd_deg`` is below ``baseline_deg`` while ``cmd_units``
    is below ``baseline_units`` by ``|delta| / scale``, not by ``|delta|``.
    """
    scale = _TRACKING_SCALE_DEG
    client = _TrackingFake()
    baseline_units = _TRACKING_STATE[_TRACKING_JOINT]

    summary = _run_steps(latency, client, delta_deg=-1.0, steps=1, deg_per_unit=scale)

    row = summary["rows"][0]
    assert row["cmd_deg"] < row["baseline_deg"], (
        f"cmd_deg={row['cmd_deg']!r} is not below baseline_deg={row['baseline_deg']!r}"
    )
    assert row["cmd_deg"] == pytest.approx(row["baseline_deg"] + (-1.0), rel=0, abs=1e-9)
    assert row["cmd_units"] == pytest.approx(baseline_units + (-1.0) / scale, rel=0, abs=1e-9)
    assert row["cmd_units"] < baseline_units, (
        f"cmd_units={row['cmd_units']!r} is not below the baseline {baseline_units!r}"
    )
    # The row is not decoration: the same value went out on the wire.
    assert client.sent_actions[0][_TRACKING_JOINT] == pytest.approx(
        row["cmd_units"], rel=0, abs=1e-9
    )


def test_run_tracking_rows_should_keep_the_sign_of_a_negative_error_deg(latency) -> None:
    """AA5: ``error_deg`` is ``cmd_deg - measured_deg``, i.e. negative here.

    The feedback is frozen at the baseline, so the residual is exactly the
    (negative) step: ``abs()`` in the wrong place would report ``+1.0`` and the
    CSV would claim the joint was above its goal.
    """
    client = _TrackingFake()

    summary = _run_steps(latency, client, delta_deg=-1.0, steps=1)

    row = summary["rows"][0]
    assert row["measured_deg"] == pytest.approx(row["baseline_deg"], rel=0, abs=1e-9)
    assert row["error_deg"] == pytest.approx(row["cmd_deg"] - row["measured_deg"], rel=0, abs=1e-12)
    assert row["error_deg"] == pytest.approx(-1.0, rel=0, abs=1e-9), (
        f"error_deg={row['error_deg']!r}: the residual lost its sign"
    )


def test_run_tracking_rows_should_book_the_rise_of_a_negative_step_symmetrically(latency) -> None:
    """AA5: the 90 % crossing is booked on ``abs(travel)``, so both directions rise.

    Two runs of the same shape, one stepping up and one stepping down, with the
    same scripted arrival stamps.  A signed crossing test
    (``travel >= 0.9 * delta``) would never fire in the negative arm
    (``-0.9 >= -0.9`` is true only because the comparison is symmetric by
    accident; ``-0.54 >= -0.9`` is *also* true, i.e. it would book the crossing a
    whole poll early), so the assertion is not just "not None": the negative
    rise time must equal the positive one exactly.
    """
    scale = 0.9
    arrivals = [10.0, 20.0, 30.0]
    up = _run_steps(
        latency,
        _ScriptedTrackingFake(values=[1.0, 1.6, 2.0], received=arrivals),
        delta_deg=1.0,
        steps=1,
        settle_timeout_s=0.4,
        deg_per_unit=scale,
    )
    down = _run_steps(
        latency,
        _ScriptedTrackingFake(values=[1.0, 0.4, 0.0], received=arrivals),
        delta_deg=-1.0,
        steps=1,
        settle_timeout_s=0.4,
        deg_per_unit=scale,
    )

    up_row = up["rows"][0]
    down_row = down["rows"][0]
    assert up_row["rise_90_time_s"] is not None, "the positive arm booked no crossing"
    assert down_row["rise_90_time_s"] is not None, (
        "the negative arm booked no 90 % crossing: the crossing test is signed"
    )
    assert down_row["received_monotonic_s"] == pytest.approx(arrivals[2], rel=0, abs=1e-9)
    assert down_row["rise_90_time_s"] == pytest.approx(
        down_row["received_monotonic_s"] - down_row["t_send_monotonic_s"], rel=0, abs=1e-9
    )
    assert down_row["rise_90_time_s"] == pytest.approx(
        up_row["rise_90_time_s"], rel=0, abs=1e-12
    ), "the two directions of the same step reported different rise times"


def test_run_tracking_rows_should_report_the_overshoot_of_a_negative_step_as_a_magnitude(
    latency,
) -> None:
    """AA5: ``overshoot_deg`` is ``peak_travel - abs(delta)`` and never negative.

    The scripted feedback undershoots the goal by 40 % (travel 1.26 deg on a
    1.0 deg step), which is the negative twin of the round-3 overshoot case.
    ``peak_travel - delta`` (signed) would report 2.26 deg here and the row
    would claim a wildly larger overshoot than happened.
    """
    scale = 0.9
    # Unit feedback per poll: baseline, -0.6, -1.4 (peak), -1.1 (settled).
    client = _ScriptedTrackingFake(values=[1.0, 0.4, -0.4, -0.1])

    summary = _run_steps(
        latency, client, delta_deg=-1.0, steps=1, settle_timeout_s=0.4, deg_per_unit=scale
    )

    row = summary["rows"][0]
    peak_travel_deg = (1.0 - (-0.4)) * scale
    assert row["overshoot_deg"] == pytest.approx(peak_travel_deg - 1.0, rel=0, abs=1e-9)
    assert row["overshoot_deg"] > 0.0, (
        f"overshoot_deg={row['overshoot_deg']!r}: the measured overshoot was dropped"
    )
    assert row["overshoot_deg"] >= 0.0
    assert row["error_deg"] == pytest.approx(row["cmd_deg"] - row["measured_deg"], rel=0, abs=1e-12)
    assert row["settled"] is True, (
        f"the negative step did not settle on |residual| <= tolerance: {row!r}"
    )
    # The flag is not taken on trust: the residual the test recomputes itself has to
    # be inside the tolerance, otherwise ``settled`` fired a poll early -- exactly
    # what a settle comparison without ``abs()`` does in this direction (the
    # residual is negative while the joint is still short of the goal).
    assert abs(row["measured_deg"] - row["cmd_deg"]) <= 0.15, (
        f"the negative step was judged settled on a residual of "
        f"{row['measured_deg'] - row['cmd_deg']!r} deg, i.e. before it was within the "
        "0.15 deg tolerance"
    )
