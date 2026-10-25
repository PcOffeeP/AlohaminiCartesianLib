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
tracking branch must clamp ``--delta-deg`` to ``MAX_STEP_DELTA_DEG`` through it
and print the value actually used.

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
  section ``S``.

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
  step (``--delta-deg`` after the ``MAX_STEP_DELTA_DEG`` clamp): a tolerance that
  large judges the joint settled on the first poll, which leaves ``rise_90``
  empty for every row.  The check is unconditional (rtt mode parses the same
  knobs and ignores them) because the default pair ``1.0`` / ``0.15`` is safe.

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
    ) -> None:
        self._sequences = sequences
        self._ages = ages
        self._host_durations = host_durations
        self._host_unix_ns = host_unix_ns
        self._connect_error = connect_error
        self._interrupt_at = interrupt_at
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
