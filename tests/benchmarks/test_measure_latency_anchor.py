"""Contract tests for the ``--anchor`` knob of the latency harness.

The script under test is ``benchmarks/alohamini_io/measure_latency.py``, loaded
**by path** (``importlib.util.spec_from_file_location``) because it is not a
package module; ``_load_script`` does that and the module-scope ``latency``
fixture hands the module to every test.  Nothing here needs a robot and nothing
here waits on real time in the direct-call tests: the file ships its own
scriptable feedback stand-ins (``_ScriptedFeedbackFake``), an injecting clock
(``_FakeClock``) and a no-op sleeper, so no socket is opened, no port is bound
and no ``lerobot`` hardware import happens.  The four ``main``-level tests that
reach the tracking loop do go through the real ``time.sleep`` pacer of the script
(the CLI offers no injection point), which is why their runs are kept to one or two
short steps and no assertion depends on a duration; the other ``main``-level tests
are refused by the parser or by Gate E before any pacing happens.

The knob under test (the contract these tests freeze):

* ``--anchor`` takes exactly ``{"present", "command"}`` and defaults to
  ``present``; anything else is an argparse usage error (2) that still names both
  legal values and never reaches the client factory;
* ``run_tracking_steps(..., anchor: str = "present")`` takes the same knob as a
  keyword after the existing parameters, so every existing caller is unchanged;
  an unknown value raises ``ValueError`` naming ``anchor`` **before** the first
  observation and the first command;
* ``present`` is the behaviour every earlier run had, byte for byte: the goal of
  every step is ``measured units + delta_deg / deg_per_unit``, so the lead never
  exceeds one step and an error cannot accumulate over steps;
* ``command`` anchors the goal of a step on the target **sent by the previous
  step** (the first step falls back to the measured position), so the goal walks
  away from a joint that does not follow: step ``n`` sits ``n + 1`` steps away
  from the measured position.  That is precisely the shape a static-friction
  break-in run needs -- the lead grows instead of being clamped to one delta;
* the row semantics do not move: ``baseline_deg`` / ``measured_deg`` stay
  measurements (never the anchor), ``error_deg`` stays ``target - measured``,
  ``cmd_units`` is what went on the wire, and ``settled`` / ``overshoot_deg`` /
  ``rise_90_time_s`` keep their meaning;
* the safety bounds do not move either: ``MAX_STEP_DELTA_DEG`` (magnitude, sign
  kept) and ``MAX_TOTAL_TRAVEL_DEG`` still gate the run, and the frozen
  ``TRACKING_CSV_FIELDS`` (14 columns) is unchanged;
* the operator note is part of the command mode only: a ``command`` run prints
  exactly one stdout line starting ``note: anchor=command``, and a run with the
  default anchor prints no anchor note at all -- the stdout of a default run is
  what it always was.

No module constant is imported for the two values (``ANCHOR_PRESENT`` /
``ANCHOR_COMMAND`` / ``DEFAULT_ANCHOR`` are the implementation's business): the
tests only ever spell the literals ``"present"`` and ``"command"``.

Regression intent (each entry is a plausible-but-wrong implementation that must
turn this file red):

* **anchor silently ignored** -- a ``command`` run that still re-anchors on the
  measured position keeps every target at ``measured + delta`` and fails the
  accumulation assertions;
* **anchor as a second code path** -- re-deriving the ``present`` arithmetic
  differently (an extra conversion round trip, a different order of operations)
  fails the row-by-row whole-dict comparison of the default run *and* the
  bit-exact first-step assertion of the ``present`` mode, which is written on a
  feedback value where the two candidates are different doubles;
* **anchor without the first-step fallback** -- anchoring step 0 on ``0.0`` or on
  ``delta / deg_per_unit`` fails the fallback test, which is written on a joint
  whose measured position is deliberately not zero;
* **anchor bypassing a safety bound** -- a ``command`` run that walks past
  ``MAX_TOTAL_TRAVEL_DEG`` or past the ``MAX_STEP_DELTA_DEG`` clamp fails the two
  gate tests;
* **anchor validated too late** -- raising after the first ``get_observation``
  fails the "nothing was read or commanded" assertion;
* **the note always printed** -- printing the command-mode note unconditionally
  (or at all in the default mode) fails the default-mode stdout assertion: the
  note is the one stdout change the knob is allowed to make, and only in one of
  its two values.
"""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
from typing import Any

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPT_PATH = _REPO_ROOT / "benchmarks" / "alohamini_io" / "measure_latency.py"

# The stepped joint in the observation-key shape the script's ``--joint`` accepts
# with or without the ``.pos`` suffix, and the bare motor name the calibration
# table is keyed by.
_JOINT_KEY = "arm_left_shoulder_pan.pos"
_MOTOR = "arm_left_shoulder_pan"
# Normalized ``.pos`` feedback of a joint nothing has moved.  Deliberately not 0
# so that an anchor of ``0.0`` (the first-step fallback bug) cannot hide.
_BASELINE_UNITS = 1.0
# (range_max - range_min) / 200 * 360 / (resolution - 1) = 0.8791208791 deg/unit:
# the calibration the fake host publishes, i.e. a joint whose wire unit is *not*
# a degree.
_MOTOR_ENTRY = {"range_min": 1000, "range_max": 3000, "resolution": 4096}

# Local literals of the contract, never the module's constant names.
_ANCHOR_PRESENT = "present"
_ANCHOR_COMMAND = "command"


def _load_script() -> Any:
    """Load the benchmark script by path (it is not a package module)."""
    assert _SCRIPT_PATH.is_file(), (
        f"script under test: {_SCRIPT_PATH} (the contract below is frozen; "
        "a missing script fails loudly instead of skipping)"
    )
    spec = importlib.util.spec_from_file_location("bench_measure_latency_anchor", _SCRIPT_PATH)
    assert spec is not None and spec.loader is not None, (
        f"cannot build import spec for {_SCRIPT_PATH}"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def latency() -> Any:
    """The module under test: the script loaded by path, imported once per run."""
    return _load_script()


# ===========================================================================
# Harness: scripted feedback, deterministic clock, no-op sleeper, factory spy
# ===========================================================================


class _FakeClock:
    """Monotonic clock stub: every read advances by ``step_s``.

    An *advancing* clock is what a run needs that reads before it commands: with a
    frozen reading the settle loop's ``now - t_send >= settle_timeout_s`` never
    becomes true and the loop would spin.  Two instances built with the same
    arguments read the same sequence, which is what makes two runs comparable.
    """

    def __init__(self, *, start: float = 0.0, step_s: float = 0.05) -> None:
        """Freeze the starting reading and the per-read advance."""
        self._now = start
        self._step_s = step_s
        self.reads = 0

    def __call__(self) -> float:
        """Return the current reading and advance the stub by one step."""
        current = self._now
        self._now += self._step_s
        self.reads += 1
        return current


def _noop_sleep(_seconds: float) -> None:
    """Sleep stub: a run never waits on real time."""


class _ScriptedFeedbackFake:
    """Tracking stand-in whose per-poll feedback comes from a script.

    ``values[i]`` is the ``.pos`` value reported on the ``i``-th
    ``get_observation`` call and the last entry saturates, so ``[1.0]`` is a joint
    that never moves however often it is commanded.  ``send_action`` only records
    the command: the feedback is scripted, never a function of the goal, which is
    what separates "the target accumulated" from "the joint followed".

    Every call grows ``observation_sequence`` (the round-3 stall detector stays
    quiet), the host metadata carries the schema version, the robot model and one
    calibration entry (Gate C and Gate F), and no safety face is published -- the
    sidecar then records empty cells, which is orthogonal to the anchor.
    """

    def __init__(self, values: list[float]) -> None:
        """Build the fake from the scripted ``.pos`` feedback values."""
        self._values = list(values)
        self.sent_actions: list[dict[str, Any]] = []
        self.get_observation_calls = 0
        self.observation_sequence = 0
        self.is_connected = False
        self.prime_calls: list[bool] = []
        self.latest_robot_metadata: dict[str, Any] = {
            "schema_version": 2,
            "robot_model": "alohamini2pro",
            "motors": {_MOTOR: dict(_MOTOR_ENTRY)},
        }
        self.latest_control_status: dict[str, Any] | None = None
        self.latest_request_timing: dict[str, Any] = {}
        self.latest_host_timing: dict[str, Any] = {}

    def _value_at(self, index: int) -> float:
        """The scripted value of poll ``index`` (saturating on the last entry)."""
        return self._values[min(index, len(self._values) - 1)]

    def connect(self) -> None:
        """Mark the fake as connected."""
        self.is_connected = True

    def disconnect(self) -> None:
        """Mark the fake as disconnected."""
        self.is_connected = False

    def prime_observation_request_window(self, *, include_cameras: bool) -> None:
        """Record the window prime instead of refilling a real request queue."""
        self.prime_calls.append(include_cameras)

    def get_observation(self, include_cameras: bool = True) -> dict[str, Any]:
        """Answer one scripted frame and grow the liveness counter."""
        index = self.get_observation_calls
        self.get_observation_calls += 1
        self.observation_sequence += 1
        return {_JOINT_KEY: self._value_at(index)}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Record the command and echo it back unchanged."""
        self.sent_actions.append(dict(action))
        return dict(action)


class _Factory:
    """Injectable ``client_factory`` that records every call it receives."""

    def __init__(self, client: Any) -> None:
        """Wrap ``client`` and start with an empty call log."""
        self.client = client
        self.calls: list[tuple[Any, ...]] = []

    def __call__(self, host: str, port: int, robot_model: str, request_window: Any = None) -> Any:
        """Record the four positional arguments and hand back the client."""
        self.calls.append((host, port, robot_model, request_window))
        return self.client


def _run_steps(latency: Any, client: Any, **overrides: Any) -> dict[str, Any]:
    """Run ``run_tracking_steps`` on a deterministic clock with a no-op sleeper.

    ``settle_timeout_s=0.05`` against a clock that advances 0.05 s per read keeps
    the settle loop short and fixed, and ``settle_tolerance_deg=0.05`` stays below
    ``0.1 * |delta_deg|`` so a 90 % crossing is still measurable.  A fresh clock per
    call is what makes two runs of the same script comparable reading for reading.

    The call is forwarded as-is: a ``TypeError`` out of it is a real contract
    break of ``run_tracking_steps`` and must reach the test unchanged instead of
    being reinterpreted here.
    """
    kwargs: dict[str, Any] = {
        "joint_key": _JOINT_KEY,
        "delta_deg": 1.0,
        "steps": 3,
        "rate_hz": 30.0,
        "settle_timeout_s": 0.05,
        "settle_tolerance_deg": 0.05,
        "timeout_s": 30.0,
        "sleep": _noop_sleep,
        "clock": _FakeClock(start=0.0, step_s=0.05),
        "deg_per_unit": 1.0,
    }
    kwargs.update(overrides)
    return latency.run_tracking_steps(client, **kwargs)


def _parser_error_line(stderr: str) -> str:
    """The ``error:`` line of an argparse refusal, lowercased.

    Only that line says *why* the parser refused: ``parser.error`` prints the whole
    usage block first, and that block lists every option name, so a substring test
    against the full stderr is satisfied by a run that failed on something else.
    """
    lines = [line for line in stderr.splitlines() if "error:" in line]
    assert lines, f"argparse printed no error line: {stderr!r}"
    return lines[-1].lower()


def _tracking_argv(
    out: Path,
    *,
    steps: int,
    delta_deg: str = "1.0",
    anchor: str | None = None,
) -> list[str]:
    """A complete ``main``-level tracking run against the fake host.

    ``--anchor`` is appended only when ``anchor`` is given, so a test can run the
    default shape through the same helper as the explicit one.
    """
    argv = [
        "--host",
        "127.0.0.1",
        "--mode",
        "tracking",
        "--enable-motion",
        "--joint",
        _MOTOR,
        "--delta-deg",
        delta_deg,
        "--steps",
        str(steps),
        "--rate-hz",
        "200",
        "--control-hz",
        "50",
        "--settle-timeout-s",
        "0.05",
        "--csv",
        str(out),
    ]
    if anchor is not None:
        argv += ["--anchor", anchor]
    return argv


def _read_csv(path: Path) -> tuple[tuple[str, ...], list[dict[str, str]]]:
    """Parse ``path`` into its header and its rows (empty cells stay ``""``)."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        header = tuple(reader.fieldnames or ())
        rows = [dict(row) for row in reader]
    return header, rows


# ===========================================================================
# A. The CLI surface: ``--anchor`` with a default and exactly two values
# ===========================================================================


def test_parse_args_should_default_the_anchor_to_present(latency: Any) -> None:
    """No ``--anchor`` on the command line means the measurement shape, not the lead one.

    Anchoring on the measured position is the behaviour every existing invocation
    (and every field run so far) has, so the command-anchored shape has to be
    asked for explicitly.
    """
    args = latency.parse_args(["--host", "127.0.0.1"])

    assert args.anchor == _ANCHOR_PRESENT


def test_parse_args_should_accept_the_command_anchor(latency: Any) -> None:
    """``--anchor command`` is one of exactly two legal values."""
    args = latency.parse_args(["--host", "127.0.0.1", "--anchor", _ANCHOR_COMMAND])

    assert args.anchor == _ANCHOR_COMMAND


def test_parse_args_should_reject_an_unknown_anchor_value(latency: Any, capsys: Any) -> None:
    """A third value is a usage error that names the option and both legal values.

    Asserted on the ``error:`` line instead of the whole stderr: the usage block
    lists every option name, so ``"--anchor" in stderr`` would also hold for a
    parser that does not know the option at all.  The two legal values appear in
    that line only when the value set really is ``{"present", "command"}``.
    """
    with pytest.raises(SystemExit) as excinfo:
        latency.parse_args(["--host", "127.0.0.1", "--anchor", "bogus"])
    printed = capsys.readouterr()

    assert excinfo.value.code == 2, (
        f"an unknown anchor exited with {excinfo.value.code} instead of the usage error 2"
    )
    message = _parser_error_line(printed.err)
    assert "--anchor" in message, f"the refusal does not name the option: {message!r}"
    assert _ANCHOR_PRESENT in message, (
        f"the refusal does not name the default value it accepts: {message!r}"
    )
    assert _ANCHOR_COMMAND in message, (
        f"the refusal does not name the command value it accepts: {message!r}"
    )


def test_main_should_refuse_an_unknown_anchor_before_building_a_client(
    latency: Any, tmp_path: Path, capsys: Any
) -> None:
    """An unusable ``--anchor`` is a usage error, never a failed connection.

    The factory must not be called: refusing after ``connect()`` would open a
    socket for a run that was never allowed to start.

    The error line has to name both legal values as well, not just the option:
    today's parser rejects ``--anchor`` as an *unknown option* (
    ``unrecognized arguments``), which satisfies a bare name check while proving
    nothing about the value set.  Naming ``present`` and ``command`` is only true
    once the option exists with exactly those two choices.
    """
    factory = _Factory(_ScriptedFeedbackFake([_BASELINE_UNITS]))

    with pytest.raises(SystemExit) as excinfo:
        latency.main(
            _tracking_argv(tmp_path / "refused.csv", steps=1, anchor="bogus"),
            client_factory=factory,
        )
    printed = capsys.readouterr()

    assert excinfo.value.code == 2, (
        f"an unknown anchor exited with {excinfo.value.code} instead of the usage error 2"
    )
    assert factory.calls == [], f"the factory was called for a refused run: {factory.calls!r}"
    message = _parser_error_line(printed.err)
    assert "--anchor" in message, f"the refusal does not name the option: {message!r}"
    assert _ANCHOR_PRESENT in message and _ANCHOR_COMMAND in message, (
        f"the refusal does not name both legal anchor values: {message!r}"
    )


# ===========================================================================
# B. The default path is unchanged, byte for byte
# ===========================================================================


def test_run_tracking_steps_should_keep_the_default_anchor_rows_identical_to_present(
    latency: Any,
) -> None:
    """No ``anchor`` argument and ``anchor="present"`` are the same run, value for value.

    Strong assertion: the two row lists compare equal **element by element as whole
    dicts**, i.e. every column (``cmd_units``, ``t_send_monotonic_s``,
    ``rise_90_time_s``, ``overshoot_deg``, ``settled``, ``note`` ...) matches bit
    for bit.  An implementation that re-derives the measurement arithmetic for the
    new keyword -- an extra conversion round trip, a different order of operations
    -- fails here even when every single quantity is "close enough".
    """
    # Per settled step: 1.0 (60 % of the step), 2.4 (the 90 % crossing), 2.05 (settled).
    script = [1.0, 1.6, 2.4, 2.05]

    default_rows = _run_steps(
        latency, _ScriptedFeedbackFake(script), steps=2, settle_timeout_s=0.4
    )["rows"]
    present_rows = _run_steps(
        latency,
        _ScriptedFeedbackFake(script),
        steps=2,
        settle_timeout_s=0.4,
        anchor=_ANCHOR_PRESENT,
    )["rows"]

    assert len(default_rows) == len(present_rows) == 2
    # Guard: the rows have to be worth comparing -- a settled step that really
    # crossed 90 %, not two empties that trivially agree.
    assert default_rows[0]["settled"] is True, (
        f"the scripted step did not settle: {default_rows[0]}"
    )
    assert default_rows[0]["rise_90_time_s"] is not None, (
        f"the scripted step recorded no 90 % crossing: {default_rows[0]}"
    )
    assert default_rows == present_rows


def test_run_tracking_steps_should_command_the_measured_position_plus_the_delta(
    latency: Any,
) -> None:
    """In ``present`` mode every step's goal is ``measured units + delta / deg_per_unit``.

    The joint never moves, so re-anchoring on the measurement keeps every target at
    the same place: one delta above the measured position, never two.  That is the
    clamp the command anchor exists to lift, and it must not change.
    """
    scale = 0.5
    client = _ScriptedFeedbackFake([_BASELINE_UNITS])

    summary = _run_steps(
        latency,
        client,
        steps=2,
        delta_deg=1.0,
        deg_per_unit=scale,
        anchor=_ANCHOR_PRESENT,
    )

    rows = summary["rows"]
    assert len(rows) == 2
    expected_units = _BASELINE_UNITS + 1.0 / scale
    for row in rows:
        assert row["baseline_deg"] == pytest.approx(_BASELINE_UNITS * scale, rel=0, abs=1e-12)
        assert row["measured_deg"] == pytest.approx(_BASELINE_UNITS * scale, rel=0, abs=1e-12)
        assert row["cmd_units"] == pytest.approx(expected_units, rel=0, abs=1e-12)
        assert row["cmd_units"] == pytest.approx(
            row["baseline_deg"] / row["deg_per_unit"] + 1.0 / row["deg_per_unit"],
            rel=0,
            abs=1e-12,
        )
        assert row["error_deg"] == pytest.approx(1.0, rel=0, abs=1e-12)


# The measured feedback of the bit-exact present-mode arithmetic test, in
# normalized units.  ``3.0`` is not arbitrary: it is a value at which the correct
# target and the ``units_from_deg(deg_from_units(...))`` round trip of the same
# target land on **different** doubles at both scales below (one unit in the last
# place apart), so the assertion can tell the two apart instead of accepting
# whichever one the implementation computes.
_PRESENT_MEASURED_UNITS = 3.0
# ``cmd_units`` of a 1.0 deg present-mode step laid on those units, written out as
# the resulting doubles instead of the arithmetic that produces them: the
# assertion states *what goes on the wire* and does not restate the
# implementation's formula.  ``_PRESENT_ROUND_TRIP_UNITS`` is what the same step
# commands when the target is routed through degrees first.
_PRESENT_CMD_UNITS = {0.9: 4.111111111111111, 1.8: 3.5555555555555554}
_PRESENT_ROUND_TRIP_UNITS = {0.9: 4.111111111111112, 1.8: 3.555555555555556}
# ``cmd_deg`` of the same step: the measured position in degrees plus one delta.
_PRESENT_CMD_DEG = {0.9: 3.7, 1.8: 6.4}


@pytest.mark.parametrize("scale", [0.9, 1.8])
def test_run_tracking_steps_should_command_the_measured_units_plus_one_scaled_delta(
    latency: Any, scale: float
) -> None:
    """``present`` mode puts exactly ``1.0 / scale`` units on top of the measurement.

    The expected wire value is a literal (``4.111111111111111`` at 0.9 deg/unit,
    ``3.5555555555555554`` at 1.8), not the formula, because the difference this
    test exists for is *in the last bit*: routing the same target through degrees
    (``units_from_deg(deg_from_units(anchor, scale) + delta, scale)``) moves it by
    one ulp -- ``4.111111111111112`` / ``3.555555555555556`` -- which a relative
    tolerance of ``1e-15`` would wave through, since that tolerance is about 4.5
    ulp at these magnitudes.  The assertion is therefore bit-exact, and the two
    candidate values are asserted to be different doubles first so it cannot go
    vacuous if someone edits the constants.
    """
    client = _ScriptedFeedbackFake([_PRESENT_MEASURED_UNITS])

    summary = _run_steps(
        latency,
        client,
        steps=1,
        delta_deg=1.0,
        deg_per_unit=scale,
        anchor=_ANCHOR_PRESENT,
    )

    row = summary["rows"][0]
    expected_units = _PRESENT_CMD_UNITS[scale]
    round_trip_units = _PRESENT_ROUND_TRIP_UNITS[scale]
    # Guard: the two candidates are different numbers here, so "bit-exact" below
    # is a real distinction and not a comparison of two identical constants.
    assert round_trip_units != expected_units
    assert row["cmd_units"] == expected_units, (
        f"a 1.0 deg present-mode step on {_PRESENT_MEASURED_UNITS} units at {scale} deg/unit "
        f"commanded {row['cmd_units']!r} instead of {expected_units!r} "
        f"(the degree-first round trip would command {round_trip_units!r})"
    )
    # The same number is what left on the wire, not only what the row recorded.
    assert client.sent_actions[0][_JOINT_KEY] == expected_units
    # The degree column keeps its own arithmetic: one delta above the measurement.
    assert row["cmd_deg"] == _PRESENT_CMD_DEG[scale]


# ===========================================================================
# C. The command anchor lets the error accumulate over steps
# ===========================================================================


def test_run_tracking_steps_should_accumulate_the_target_with_the_command_anchor(
    latency: Any,
) -> None:
    """A frozen joint is walked away from step by step: ``n`` deltas of lead after ``n`` steps.

    The feedback never moves, so the accumulated goal is visible as plain
    arithmetic: step 0 asks ``measured + 1``, step 1 ``measured + 2``, step 2
    ``measured + 3``.  ``baseline_deg`` and ``measured_deg`` stay measurements (the
    anchor is never recorded as a measurement), and ``error_deg`` -- still
    ``target - measured`` -- grows with the lead instead of being pinned to one
    delta.  This is the whole point of the knob: a lead a static friction cannot
    absorb, because it is never reset.
    """
    client = _ScriptedFeedbackFake([_BASELINE_UNITS])

    summary = _run_steps(
        latency,
        client,
        steps=3,
        delta_deg=1.0,
        deg_per_unit=1.0,
        anchor=_ANCHOR_COMMAND,
    )

    rows = summary["rows"]
    assert len(rows) == 3, f"expected one row per step, got {len(rows)}"
    assert [row["measured_deg"] for row in rows] == pytest.approx([1.0, 1.0, 1.0], rel=0, abs=1e-12)
    assert [row["baseline_deg"] for row in rows] == pytest.approx([1.0, 1.0, 1.0], rel=0, abs=1e-12)
    assert [row["cmd_units"] for row in rows] == pytest.approx([2.0, 3.0, 4.0], rel=0, abs=1e-12)
    assert [row["cmd_deg"] for row in rows] == pytest.approx([2.0, 3.0, 4.0], rel=0, abs=1e-12)
    errors = [row["error_deg"] for row in rows]
    assert errors == pytest.approx([1.0, 2.0, 3.0], rel=0, abs=1e-12)
    assert errors == sorted(errors), f"the error did not grow over the steps: {errors!r}"
    assert len(set(errors)) == 3, f"the error repeated instead of accumulating: {errors!r}"


def test_run_tracking_steps_should_keep_the_present_anchor_on_the_measurement(
    latency: Any,
) -> None:
    """Control for the accumulation test: the same frozen joint in ``present`` mode stays put.

    Without this pair the accumulation assertions could be satisfied by a run that
    accumulates in both modes, which would silently change the measurement shape
    every field run so far was taken in.
    """
    client = _ScriptedFeedbackFake([_BASELINE_UNITS])

    summary = _run_steps(
        latency,
        client,
        steps=3,
        delta_deg=1.0,
        deg_per_unit=1.0,
        anchor=_ANCHOR_PRESENT,
    )

    rows = summary["rows"]
    assert [row["cmd_units"] for row in rows] == pytest.approx([2.0, 2.0, 2.0], rel=0, abs=1e-12)
    assert [row["error_deg"] for row in rows] == pytest.approx([1.0, 1.0, 1.0], rel=0, abs=1e-12)


def test_run_tracking_steps_should_anchor_the_first_step_on_the_measured_position(
    latency: Any,
) -> None:
    """The first command-anchored step has no previous target, so it falls back to the measurement.

    The measured position is 1.0 unit on a 0.5 deg/unit joint, so the three
    candidates are distinguishable: ``3.0`` units (measured + ``delta/scale``, the
    contract), ``2.0`` units (anchored on zero) and ``0.0`` units (anchored on a
    missing value).  A fallback to any constant would be invisible on a joint that
    starts at zero -- this one does not.
    """
    scale = 0.5
    client = _ScriptedFeedbackFake([_BASELINE_UNITS])

    summary = _run_steps(
        latency,
        client,
        steps=1,
        delta_deg=1.0,
        deg_per_unit=scale,
        anchor=_ANCHOR_COMMAND,
    )

    row = summary["rows"][0]
    assert row["baseline_deg"] == pytest.approx(_BASELINE_UNITS * scale, rel=0, abs=1e-12)
    assert row["cmd_units"] == pytest.approx(_BASELINE_UNITS + 1.0 / scale, rel=0, abs=1e-12)
    assert row["cmd_units"] != pytest.approx(1.0 / scale, rel=0, abs=1e-9), (
        "the first step anchored on zero instead of on the measured position"
    )
    assert row["cmd_units"] != pytest.approx(0.0, rel=0, abs=1e-9)
    assert row["cmd_deg"] == pytest.approx(_BASELINE_UNITS * scale + 1.0, rel=0, abs=1e-12)


# ===========================================================================
# D. Direction: a negative step accumulates downwards and keeps its sign
# ===========================================================================


def test_run_tracking_steps_should_accumulate_a_negative_step_without_flipping_the_sign(
    latency: Any,
) -> None:
    """``--delta-deg -1.0`` with the command anchor walks *down*, one step per delta.

    The 1 deg/unit script keeps the numbers readable: from a measured 1.0 unit the
    targets are 0.0, -1.0, -2.0.  An ``abs()`` anywhere in the anchor arithmetic,
    or a clamp that mirrors the negative target into the positive half, fails the
    monotonicity assertion.
    """
    client = _ScriptedFeedbackFake([_BASELINE_UNITS])

    summary = _run_steps(
        latency,
        client,
        steps=3,
        delta_deg=-1.0,
        deg_per_unit=1.0,
        anchor=_ANCHOR_COMMAND,
    )

    rows = summary["rows"]
    units = [row["cmd_units"] for row in rows]
    assert units == pytest.approx([0.0, -1.0, -2.0], rel=0, abs=1e-12)
    assert units[0] < _BASELINE_UNITS, (
        f"the first negative step did not go below the measured pose: {units!r}"
    )
    assert all(later < earlier for earlier, later in zip(units, units[1:], strict=False)), (
        f"the negative targets did not decrease monotonically: {units!r}"
    )
    assert [row["cmd_deg"] for row in rows] == pytest.approx([0.0, -1.0, -2.0], rel=0, abs=1e-12)
    assert [row["error_deg"] for row in rows] == pytest.approx([-1.0, -2.0, -3.0], rel=0, abs=1e-12)


def test_main_should_apply_the_step_cap_to_every_command_anchor_step(
    latency: Any, tmp_path: Path, capsys: Any
) -> None:
    """``--delta-deg -2.0`` is still clamped to ``MAX_STEP_DELTA_DEG`` on the wire.

    The clamp lives in ``resolve_delta_deg`` and is applied by the CLI before
    ``run_tracking_steps`` runs, so the anchor mode must not add a second, unclamped
    path to the wire: with the command anchor the two consecutive targets differ by
    exactly the clamped step (1.0 deg) and not by the requested 2.0 deg.  Judged on
    the CSV the run wrote -- ``cmd_deg`` in degrees and ``cmd_units`` in wire units
    -- so a clamp kept only in the report cannot pass.
    """
    out = tmp_path / "tracking-anchor-down.csv"

    code = latency.main(
        _tracking_argv(out, steps=2, delta_deg="-2.0", anchor=_ANCHOR_COMMAND),
        client_factory=_Factory(_ScriptedFeedbackFake([_BASELINE_UNITS])),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the clamped negative run exited with {code}: {printed.err!r}"
    header, rows = _read_csv(out)
    assert header == tuple(latency.TRACKING_CSV_FIELDS)
    assert len(rows) == 2, f"expected one row per step, got {len(rows)}"
    first, second = rows
    scale = float(first["deg_per_unit"])
    step_deg = float(second["cmd_deg"]) - float(first["cmd_deg"])
    step_units = float(second["cmd_units"]) - float(first["cmd_units"])
    assert step_deg == pytest.approx(-1.0, rel=0, abs=1e-9), (
        f"two command-anchored steps differ by {step_deg!r} deg instead of the clamped -1.0 deg"
    )
    assert step_units == pytest.approx(-1.0 / scale, rel=0, abs=1e-9), (
        f"the wire step is {step_units!r} units instead of -1.0 deg = {-1.0 / scale!r} units"
    )
    assert float(first["cmd_deg"]) < float(first["baseline_deg"]), (
        "the first negative step did not point below the measured pose"
    )


# ===========================================================================
# E. The travel cap still refuses, with or without the command anchor
# ===========================================================================


def test_main_should_still_refuse_a_run_over_the_travel_cap_with_the_command_anchor(
    latency: Any, tmp_path: Path, capsys: Any
) -> None:
    """21 x 1.0 deg is over ``MAX_TOTAL_TRAVEL_DEG`` in *both* anchor modes.

    The cap is what keeps the accumulated lead of a command-anchored run from
    turning into an unbounded march, so the anchor must not become a way around
    Gate E: the refusal happens before the first command and names the cap.
    """
    client = _ScriptedFeedbackFake([_BASELINE_UNITS])

    code = latency.main(
        _tracking_argv(tmp_path / "tracking-anchor-cap.csv", steps=21, anchor=_ANCHOR_COMMAND),
        client_factory=_Factory(client),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_MOTION_GATE_REFUSED, (
        f"21 x 1.0 deg with --anchor command exited with {code} instead of "
        f"{latency.EXIT_MOTION_GATE_REFUSED}: {printed.err!r}"
    )
    assert client.sent_actions == [], (
        f"a run over the travel cap commanded {len(client.sent_actions)} action(s)"
    )
    assert "Gate E" in printed.err, f"the refusal does not name Gate E: {printed.err!r}"
    assert "cap" in printed.err.lower(), (
        f"the refusal does not name the travel cap: {printed.err!r}"
    )


# ===========================================================================
# F. The keyword: an unknown value fails before anything is read or commanded
# ===========================================================================


def test_run_tracking_steps_should_reject_an_unknown_anchor_before_reading_feedback(
    latency: Any,
) -> None:
    """A direct caller's typo is a ``ValueError``, not a silently measured run.

    The check has to happen before the first observation and the first command: a
    run that reads feedback first would already have spent a window on a run whose
    shape it cannot honour, and a robot that received one command of an
    unknown-shape run is the case this refuses.
    """
    client = _ScriptedFeedbackFake([_BASELINE_UNITS])

    with pytest.raises(ValueError) as excinfo:
        _run_steps(latency, client, steps=1, anchor="bogus")

    assert "anchor" in str(excinfo.value), (
        f"the ValueError does not name the knob: {excinfo.value!r}"
    )
    assert client.get_observation_calls == 0, (
        f"the run read {client.get_observation_calls} frame(s) before rejecting the anchor"
    )
    assert client.sent_actions == [], (
        f"the run commanded {len(client.sent_actions)} action(s) before rejecting the anchor"
    )


# ===========================================================================
# G. The evidence file keeps its frozen shape in the command-anchor mode
# ===========================================================================


def test_main_should_keep_the_frozen_tracking_csv_in_the_command_anchor_mode(
    latency: Any, tmp_path: Path, capsys: Any
) -> None:
    """``--anchor command`` changes the goal, never the table it is written into.

    The 14 columns are what every consumer of the field evidence parses, so the
    knob may not add a column, drop one or change the row-per-step shape: the
    header equals ``TRACKING_CSV_FIELDS`` and there is one row per step even when
    the joint never settles (the frozen feedback keeps every goal out of reach).
    """
    out = tmp_path / "tracking-anchor.csv"

    code = latency.main(
        _tracking_argv(out, steps=2, anchor=_ANCHOR_COMMAND),
        client_factory=_Factory(_ScriptedFeedbackFake([_BASELINE_UNITS])),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the command-anchor run exited with {code}: {printed.err!r}"
    header, rows = _read_csv(out)
    assert header == tuple(latency.TRACKING_CSV_FIELDS), (
        f"the command-anchor mode changed the CSV columns: {header!r}"
    )
    assert len(header) == 14, f"the main CSV is no longer 14 columns: {header!r}"
    assert len(rows) == 2, f"expected one row per step, got {len(rows)}"
    for row in rows:
        for field in ("cmd_deg", "cmd_units", "cmd_echo_deg", "baseline_deg", "measured_deg"):
            assert row[field] != "", f"row lost {field} in the command-anchor mode: {row!r}"


# ===========================================================================
# H. The operator note: command mode only, and exactly one line of it
# ===========================================================================
#
# The command anchor changes what the printed ``error_deg`` / ``overshoot_deg`` /
# ``rise_90_time_s`` mean, so the run says so on stdout before it starts.  The
# note is the only stdout change the knob is allowed to make, and only in one of
# its two values: a default run must print exactly what it always printed.


def test_main_should_print_one_anchor_note_line_for_the_command_anchor(
    latency: Any, tmp_path: Path, capsys: Any
) -> None:
    """A ``command`` run names its anchor on stdout, on a single line.

    Two things are asserted, and they fail for different reasons: the note exists
    (``anchor=command`` appears at all) and it is *one* line starting with its own
    prefix -- a note printed as several lines either repeats the prefix (caught by
    the exact-one-line count) or leaves a fragmentary first line (caught by the
    length floor), which is the "newline fragment" an operator would read as two
    unrelated notes.
    """
    out = tmp_path / "tracking-anchor-note.csv"

    code = latency.main(
        _tracking_argv(out, steps=1, anchor=_ANCHOR_COMMAND),
        client_factory=_Factory(_ScriptedFeedbackFake([_BASELINE_UNITS])),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the command-anchor run exited with {code}: {printed.err!r}"
    note_lines = [line for line in printed.out.splitlines() if "anchor=command" in line]
    assert len(note_lines) == 1, (
        f"expected exactly one stdout line naming the command anchor, got {note_lines!r}"
    )
    note = note_lines[0]
    assert note.startswith("note: anchor=command"), (
        f"the anchor note is not one line starting with its own prefix: {note!r}"
    )
    # The note is a full sentence; a wrapped or split print leaves a short fragment.
    assert len(note) > 100, f"the anchor note reads like a line fragment: {note!r}"


def test_main_should_print_no_anchor_note_with_the_default_anchor(
    latency: Any, tmp_path: Path, capsys: Any
) -> None:
    """The default run's stdout carries no anchor note at all.

    ``present`` is the shape every earlier run had, so its stdout may not grow a
    line: printing the note unconditionally -- the one-line ``if`` mistake this
    test exists for -- is caught here while the command-mode test above stays
    green.
    """
    out = tmp_path / "tracking-default-note.csv"

    code = latency.main(
        _tracking_argv(out, steps=1),
        client_factory=_Factory(_ScriptedFeedbackFake([_BASELINE_UNITS])),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the default run exited with {code}: {printed.err!r}"
    assert "anchor=command" not in printed.out, (
        f"the default run printed the command-anchor note: {printed.out!r}"
    )
    assert "note: anchor=" not in printed.out, (
        f"the default run printed an anchor note: {printed.out!r}"
    )
