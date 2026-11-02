"""Offline contract tests for the tracking safety sidecar of ``measure_latency.py``.

The vendor Host publishes a second, *side-channel* frame on
``client.latest_safety_status`` every control loop: the 18 motor currents in
milliamps (signed), the hold flags, the hold events and the requested / accepted
joint targets.  It is not part of ``get_observation()``, and it does not belong
in the 14-column tracking CSV either (that table is frozen).  This file pins the
contract of the *sidecar* that records it while ``--mode tracking`` moves the
robot: a second CSV next to the tracking CSV, one row per motor per growing
frame.

The script is loaded **by path** (``importlib.util.spec_from_file_location``)
because it is not a package module.  Nothing here needs a robot or a socket: the
file ships its own scriptable stand-in (``_SafetyFake``), a frozen clock
(``_FrozenClock``) and a no-op sleeper, and every artefact goes to ``tmp_path``.

Frozen interface (the tests below are the expectation, not a description of an
implementation):

* ``SAFETY_CSV_FIELDS`` — the nine sidecar columns in this order:
  ``step_index``, ``sample_sequence``, ``t_monotonic_s``, ``motor``,
  ``current_ma``, ``requested_target``, ``accepted_target``, ``hold_active``,
  ``target_source`` (`target_source` is appended *after* `hold_active`; the
  eight columns that came before it keep their position and their meaning).
* ``SAFETY_CLIENT_CAPABILITIES == ("latest_safety_status",)``.
* ``TRACKING_CSV_FIELDS`` stays the frozen 14-column table.
* ``safety_csv_path(csv_path)`` — the main CSV path without its ``.csv`` suffix
  plus ``-safety.csv``, in the same directory.
* ``missing_safety_capabilities(client, names=SAFETY_CLIENT_CAPABILITIES)`` —
  pure ``hasattr`` probe; the absent faces, in ``names`` order.
* ``safety_frame_rows(face, *, step_index, sample_sequence, t_monotonic_s)`` —
  pure, **never raises**, one row per motor of ``face["currents_ma"]`` (or
  exactly one row with ``motor=None`` when no usable motor list exists).
* ``target_source`` is a WHOLE-frame attribute of the vendor frame
  (``face["target_source"]``), not a per-motor one: every row of a frame carries
  the same value.  Only the four vendor values (``none``, ``command``,
  ``watchdog``, ``protection``) are ever written.  A frame without the key, a
  value that cannot be read, and a value that is not one of those four strings
  (``bool``, number, list and an unknown string alike) all leave the cell empty —
  and only that cell: no other column of the row changes, and nothing is
  defaulted in.  An empty cell is "unavailable", never "none": ``none`` is a
  vendor statement about the frame, an empty cell is the absence of one.
* the arrival-stamp fallback is NAMED, once per run: a run that had to stamp at
  least one recorded frame (or the 90 % crossing) with the poll clock instead of
  ``latest_request_timing["received_monotonic_s"]`` prints exactly one stderr
  line saying so — the stamp is the poll instant, not the client's timestamp, and
  the reason is that face.  One line for the whole run, not one per frame, and
  its wording is distinguishable from the two safety warnings below: it names
  ``t_monotonic_s`` / ``latest_request_timing`` and never
  ``latest_safety_status``.  A run whose timing face answers a finite
  ``received_monotonic_s`` stays silent.
* ``run_tracking_steps`` summary carries ``safety_rows`` (rows in arrival order)
  and ``safety_missing`` (the probe result of that run's start).
* the sidecar is written on the tracking path only; a missing
  ``latest_safety_status`` degrades to empty cells plus exactly one warning that
  names the face, and does not change the exit code.
* the frames are the *vendor* key shapes, not a convenient symmetric one:
  ``currents_ma`` and ``joint_holds`` are keyed by bare motor name
  (``alohamini_host.py:300-301``, ``alohamini.py:985``) while
  ``requested_targets`` / ``accepted_targets`` are keyed by ``<motor>.pos``
  (``alohamini.py:987-988``).  Both target key shapes are read; a lookup that
  only knows the bare name writes two empty target columns on every frame of a
  real run.  ``hold_active`` is ``False`` on every row of the pinned vendor
  client, because ``joint_holds`` has no write site there (``alohamini.py:298``
  initialises ``_joint_hold_goal`` and ``:985``/``:1076``/``:1088``/``:1128``/
  ``:1131`` only read or pop it), so ``joint_holds`` is ``{}`` on a real run and
  ``False`` -- not an empty cell -- is the honest answer for "not held".  That
  also means the hold column can NOT answer "is the shoulder being held back by
  an over-current": the only columns that can are ``current_ma`` (the current
  itself) and, when a target was clipped, ``requested_target`` vs
  ``accepted_target``.
* the attribute always exists on the pinned vendor client (initial ``{}``), so
  the degradation that really happens is *content*: a frame whose ``currents_ma``
  carries no readable motor table.  Judged over the whole run and reported
  **once**: as soon as at least one recorded frame has no readable
  ``currents_ma`` motor table, the run prints exactly one warning naming
  ``latest_safety_status`` and carrying machine-readable frame counts
  (``frames=<recorded frames>``, ``frames_with_motor_table=<how many of them had
  one>``), so an operator can see how many sidecar frames are empty without
  opening the file.  Rows degrade per frame as always (one ``motor=None`` row,
  empty safety cells), the sidecar is still written and the exit code is
  untouched.  The absent-face branch (no attribute at all) keeps its own single
  warning; the two branches are mutually exclusive, so a run prints at most one.
* a pathological ``latest_request_timing`` (a ``Mapping`` whose readers raise)
  costs the row its arrival stamp and nothing else: the poll clock stamps it, the
  run finishes, the sidecar is written.

Regression intents (each is a plausible-but-wrong implementation that must turn
this file red):

* **one row per poll instead of per growing frame** — the stale-frame test
  (``_SafetyFake(sequence_freezes_after=2)``) has 5 polls and 2 frames, i.e. 4
  rows instead of 10;
* **only the step tail recorded** — a step here costs two observations
  (step-start + one settle poll), so 2 steps are 4 frames / 8 rows; recording
  the tail only writes 4;
* **``0`` instead of an empty cell** — a motor missing from
  ``requested_targets`` / a ``bool`` current / a non-numeric current must leave
  the cell empty, never ``0`` / ``0.0`` (a ``0 mA`` current reads like a real
  measurement);
* **``getattr(client, "latest_safety_status", {})``** — the silent fallback
  writes the same empty sidecar as the honest refusal, so the *warning count*
  (exactly one stderr line naming the face) is what fails it;
* **a safety column in the main CSV** — the 14-column header assertion;
* **``str`` treated as a hold container** — a motor name is a substring of the
  joint name, so ``motor in "arm_left_shoulder_pan.pos"`` is ``True``; the hold
  flag of a non-container must stay empty.
* **bare-name-only target lookup** — ``requested_targets[_MOTOR]`` misses the
  vendor frame, whose table is keyed ``<motor>.pos``; both target columns come
  out empty for every frame of every real run (Critical 1);
* **a published-but-empty face taken as a silent frame** — writing an empty
  sidecar without naming the face leaves the operator unable to tell "the host
  does not fill this frame" from "the currents really are blank" (Critical 2);
* **the mixture taken as a healthy run** — only warning when *no* frame carries a
  table lets 3 broken frames out of 4 through in silence; the mixed case pins the
  whole-run count (``frames=4``, ``frames_with_motor_table=1``) and the empty
  ``motor`` rows of the sidecar;
* **a warning without the frame counts** — "not one frame carried a motor table"
  is the wording of the old, narrower criterion; the counts are what tell the
  operator how much of the sidecar is empty, so their absence is a red test;
* **an unguarded read of the arrival stamp** — ``latest_request_timing[...]`` on
  a face whose readers raise aborts the step as a feedback failure instead of
  falling back to the poll clock (Important 3.2).

Regression intents of the ninth column and the named fallback (same rule: a
plausible-but-wrong implementation has to turn this file red):

* **one fallback notice per frame or per poll** — the fallback cases count the
  notices (``== 1``) over a run of 4 unstamped frames, so per-frame wording
  writes 4 and per-poll wording writes 5;
* **a fallback notice worded like a safety warning** — the notice may not name
  ``latest_safety_status``, and the safety-face lines of the same otherwise-clean
  run have to be 0: two warnings an operator cannot tell apart are one warning;
* **``getattr(face, ..., None)`` / ``face.get(...)`` with a default** — a
  defaulted read of ``target_source`` turns "the frame said nothing" into a
  fabricated value, so the missing-key case fails;
* **no value check on ``target_source``** — writing ``"manual"``, ``""``, ``5``
  or ``True`` straight into the column is what the unusable-value cases refuse
  (``none`` is the honest empty answer, not a placeholder for anything else);
* **``target_source`` read per motor** — e.g. looked up in ``requested_targets``
  under ``<motor>.pos``, the shape the two target columns use; the per-motor
  tables never carry the key, so the whole-frame consistency case fails;
* **``target_source`` inserted before or inside the other columns** — the frozen
  nine-column order and the sidecar header pin it directly after
  ``hold_active``;
* **the first frame's ``target_source`` reused for the whole run** — the
  mixed-frame case has one frame carrying ``"command"`` and the next carrying
  nothing, and each frame's rows have to answer for their own frame.
* **the degraded ``motor=None`` row losing the frame's ``target_source``** — the
  frame whose motor table is unreadable is the one an operator reaches for after
  an over-current, so the corrupt-currents and empty-table cases pin the source
  on the placeholder row (and only that cell: the rest stays empty);
* **the poll-clock notice folded into the safety warning's branch** — an
  ``elif`` on the safety chain silently drops it on exactly the run that hits
  both faces, so the both-degraded case counts the two notices separately, pins
  that they are two different stderr lines, and pins that their identifying
  markers do not overlap (otherwise the two counts are one measurement).
"""

from __future__ import annotations

import csv
import importlib.util
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPT_PATH = _REPO_ROOT / "benchmarks" / "alohamini_io" / "measure_latency.py"


def _load_script() -> Any:
    """Load the benchmark script by path (it is not a package module)."""
    assert _SCRIPT_PATH.is_file(), (
        f"script under test: {_SCRIPT_PATH} (the contract below is frozen; "
        "a missing script fails loudly instead of skipping)"
    )
    spec = importlib.util.spec_from_file_location("bench_measure_latency_safety", _SCRIPT_PATH)
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
# Harness: fake clients, frozen clock, no-op sleeper (no sockets, no sleeps)
# ===========================================================================

_JOINT_KEY = "arm_left_shoulder_pan.pos"
_MOTOR = "arm_left_shoulder_pan"
_OTHER_MOTOR = "arm_right_shoulder_pan"
# The vendor keys its two target tables by ``<motor>.pos`` (``alohamini.py:987-988``,
# where ``_arm_goal_positions`` / ``_arm_sent_positions`` are filled from ``left_pos``
# / ``right_pos`` and filtered with ``key.removesuffix(".pos") in bus.motors``), while
# ``currents_ma`` / ``joint_holds`` carry the bare motor name
# (``alohamini_host.py:300-301``, ``alohamini.py:985``).  The fakes below reproduce
# that asymmetry on purpose: it is the shape that makes a bare-name-only target
# lookup fail, so the defect cannot hide behind a symmetric convenience fake.
_TARGET_KEY = f"{_MOTOR}.pos"
_OTHER_TARGET_KEY = f"{_OTHER_MOTOR}.pos"
# The stepped joint's normalized feedback when nothing has been commanded yet.
_BASELINE_UNITS = 1.0
_MOTOR_ENTRY = {"range_min": 1000, "range_max": 3000, "resolution": 4096}
# Sentinel: the client does not publish ``latest_safety_status`` at all, which is
# what the deployed vendor client looks like before the safety face existed.
_NO_FACE = object()


def _face_for_poll(index: int) -> dict[str, Any]:
    """A healthy safety frame in the vendor's key shape; the stepped current follows the poll.

    The per-poll current is what proves the sidecar snapshots every frame instead
    of repeating one: frame ``i`` carries ``120 + i`` mA on the stepped motor.

    The key shapes are the vendor's (see ``_TARGET_KEY``): the currents and holds
    are keyed by bare motor name, the two target tables by ``<motor>.pos``.  A
    recorder that looks the bare name up in the target tables therefore writes two
    empty target columns for every frame -- which is exactly what it does on the
    real robot, and what these tests must not let pass silently.
    """
    return {
        "currents_ma": {_MOTOR: 120.0 + index, _OTHER_MOTOR: -45.5},
        "joint_holds": [],
        "joint_hold_events": [],
        "requested_targets": {_TARGET_KEY: 1.5, _OTHER_TARGET_KEY: -2.5},
        "accepted_targets": {_TARGET_KEY: 1.5, _OTHER_TARGET_KEY: -2.5},
        "watchdog_active": False,
        "watchdog_events": 0,
    }


# The four values the vendor frame may carry in ``target_source`` (a whole-frame
# attribute: which layer produced this frame's targets).
_TARGET_SOURCE_VALUES = ("none", "command", "watchdog", "protection")


def _face_with_source(source: Any = None, *, include: bool = True) -> dict[str, Any]:
    """A healthy vendor-shaped frame, optionally carrying ``target_source``.

    ``include=False`` builds the frame without the key at all, which is the
    "this vendor frame does not publish the attribute" shape: the column has to
    stay empty rather than take a default.  The value is written verbatim,
    including values the vendor should never send (a ``bool``, a number, a list,
    an unknown string) -- those are the unusable-value cases.
    """
    face = _face_for_poll(0)
    if include:
        face["target_source"] = source
    return face


class _FrozenClock:
    """Clock stub that never advances: pacing and settle budgets stay exact."""

    def __init__(self, now: float = 0.0) -> None:
        """Freeze the clock at ``now`` and start counting reads."""
        self.now = now
        self.reads = 0

    def __call__(self) -> float:
        """Return the frozen reading and count the read."""
        self.reads += 1
        return self.now


def _noop_sleep(_seconds: float) -> None:
    """Sleep stub: a run never waits on real time."""


class _SafetyFake:
    """Tracking stand-in that publishes ``latest_safety_status``.

    One ``get_observation`` call grows ``observation_sequence`` by one (unless
    ``sequence_freezes_after`` says otherwise) and answers with the stepped
    joint's last commanded goal, so the joint settles on its very first settle
    poll: a step costs exactly two observations (the step-start frame and one
    settle poll).  Every row count below is that arithmetic, not an estimate.

    ``face`` is what the client publishes on ``latest_safety_status``: a plain
    value (a dict, ``None``, a string, a list -- anything), a callable of the
    poll index for a face that changes per frame, or :data:`_NO_FACE`, which
    means the attribute does not exist on the instance at all.

    ``timing`` replaces the per-poll ``latest_request_timing`` dict with one fixed
    object on every frame (a pathological face whose readers raise); ``None`` keeps
    the scripted per-poll dict of a timed frame, and ``timed=False`` publishes the
    empty ``{}`` a client without timing data holds.
    """

    def __init__(
        self,
        *,
        face: Any = _face_for_poll,
        sequence_freezes_after: int | None = None,
        timed: bool = True,
        timing: Any = None,
    ) -> None:
        """Build the fake with a scripted safety face and liveness behaviour."""
        self._face = face
        self._freeze_after = sequence_freezes_after
        self._timed = timed
        self._timing = timing
        self._last_goal: float | None = None
        self.observation_sequence = 0
        # The sequence each observation call reported, in call order: it is what
        # the recorded ``sample_sequence`` values are compared against.
        self.observations: list[int] = []
        self.get_observation_calls = 0
        self.sent_actions: list[dict[str, Any]] = []
        self.is_connected = False
        self.latest_robot_metadata: dict[str, Any] = {
            "schema_version": 2,
            "robot_model": "alohamini2pro",
            "motors": {_MOTOR: dict(_MOTOR_ENTRY)},
        }
        self.latest_control_status: dict[str, Any] | None = None
        self.latest_request_timing: Any = {}
        self.latest_host_timing: dict[str, Any] = {}
        if face is not _NO_FACE:
            self.latest_safety_status = self._face_at(0)

    def _face_at(self, index: int) -> Any:
        """The face this poll publishes (a callable is called with the index)."""
        face = self._face
        return face(index) if callable(face) else face

    def connect(self) -> None:
        """Mark the fake as connected."""
        self.is_connected = True

    def disconnect(self) -> None:
        """Mark the fake as disconnected."""
        self.is_connected = False

    def get_observation(self, include_cameras: bool = True) -> dict[str, Any]:
        """Answer one frame: grow the sequence, refresh both side-channel faces."""
        index = self.get_observation_calls
        self.get_observation_calls += 1
        if self._freeze_after is None or index < self._freeze_after:
            self.observation_sequence += 1
        self.observations.append(self.observation_sequence)
        if self._timing is not None:
            self.latest_request_timing = self._timing
        elif self._timed:
            self.latest_request_timing = {
                "token": f"tok-{index}".encode("ascii"),
                "sent_monotonic_s": 1000.0 + index - 0.03,
                "received_monotonic_s": 1000.0 + index,
                "age_s": 0.03,
            }
        else:
            self.latest_request_timing = {}
        if self._face is not _NO_FACE:
            self.latest_safety_status = self._face_at(index)
        measured = _BASELINE_UNITS if self._last_goal is None else self._last_goal
        return {_JOINT_KEY: measured}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Record the command, remember the stepped joint's goal and echo it back."""
        self.sent_actions.append(dict(action))
        goal = action.get(_JOINT_KEY)
        if isinstance(goal, (int, float)) and not isinstance(goal, bool):
            self._last_goal = float(goal)
        return dict(action)


class _Factory:
    """Injectable ``client_factory`` recording the call and handing back a client."""

    def __init__(self, client: Any) -> None:
        """Wrap ``client`` and start with an empty call log."""
        self.client = client
        self.calls: list[tuple[Any, ...]] = []

    def __call__(self, host: str, port: int, robot_model: str, request_window: Any = None) -> Any:
        """Record the four positional arguments and return the wrapped client."""
        self.calls.append((host, port, robot_model, request_window))
        return self.client


class _CorruptCurrents(dict):
    """A ``currents_ma`` mapping that cannot be read at all (a corrupt frame)."""

    def __init__(self) -> None:
        """Build an empty ``dict`` whose readers all raise."""
        super().__init__()

    def __len__(self) -> int:
        """Raise instead of reporting a size."""
        raise RuntimeError("corrupt currents_ma")

    def __iter__(self) -> Any:
        """Raise instead of iterating the motors."""
        raise RuntimeError("corrupt currents_ma")

    def items(self) -> Any:
        """Raise instead of listing the motors."""
        raise RuntimeError("corrupt currents_ma")

    def keys(self) -> Any:
        """Raise instead of listing the motor names."""
        raise RuntimeError("corrupt currents_ma")

    def values(self) -> Any:
        """Raise instead of listing the currents."""
        raise RuntimeError("corrupt currents_ma")


class _ExplodingTiming(Mapping):
    """A ``latest_request_timing`` that *is* a ``Mapping`` and cannot be read at all.

    Every reader raises: ``__getitem__``, ``get``, ``__iter__`` and ``__len__``.
    This is the pathological side of the timing face, and it is deliberately not
    one of the two neighbours the harness already knows:

    * an *absent* face is refused by the rtt probe, but the tracking path reads it
      only for the arrival stamp, so nothing refuses it there;
    * an *empty* ``{}`` is legal (a host that timed nothing), and
      ``_arrival_monotonic_s`` already falls back to the poll clock for it.

    A face whose readers raise is the third shape: client state, like any other,
    that may cost a frame its stamp but not the run.
    """

    def __getitem__(self, key: str) -> Any:
        """Raise instead of answering the lookup."""
        raise RuntimeError("pathological latest_request_timing")

    def get(self, key: str, default: Any = None) -> Any:
        """Raise instead of answering the lookup."""
        raise RuntimeError("pathological latest_request_timing")

    def __iter__(self) -> Any:
        """Raise instead of iterating the keys."""
        raise RuntimeError("pathological latest_request_timing")

    def __len__(self) -> int:
        """Raise instead of reporting a size."""
        raise RuntimeError("pathological latest_request_timing")


def _run_tracking(latency: Any, client: Any, **overrides: Any) -> dict[str, Any]:
    """Run ``run_tracking_steps`` on a frozen clock with a no-op sleeper.

    ``steps=2`` costs four observations with :class:`_SafetyFake` (step-start +
    one settle poll per step).  ``deg_per_unit=1.0`` keeps the scripted state
    values readable as degrees; ``settle_tolerance_deg=0.05`` is below
    ``0.1 * delta`` so a 90 % crossing is observable and nothing is masked by the
    tolerance.
    """
    kwargs: dict[str, Any] = {
        "joint_key": _JOINT_KEY,
        "delta_deg": 1.0,
        "steps": 2,
        "rate_hz": 30.0,
        "settle_timeout_s": 0.2,
        "settle_tolerance_deg": 0.05,
        "timeout_s": 30.0,
        "sleep": _noop_sleep,
        "clock": _FrozenClock(),
        "deg_per_unit": 1.0,
    }
    kwargs.update(overrides)
    return latency.run_tracking_steps(client, **kwargs)


def _main_argv(out: Path, *, steps: int = 2) -> list[str]:
    """A complete tracking run; ``steps`` steps of two observations each."""
    return [
        "--host",
        "127.0.0.1",
        "--mode",
        "tracking",
        "--enable-motion",
        "--joint",
        "arm_left_shoulder_pan",
        "--steps",
        str(steps),
        "--rate-hz",
        "200",
        "--control-hz",
        "50",
        "--delta-deg",
        "1.0",
        "--settle-tolerance-deg",
        "0.05",
        "--csv",
        str(out),
    ]


def _read_csv(path: Path) -> tuple[tuple[str, ...], list[dict[str, str]]]:
    """Parse ``path`` into its header and its rows (empty cells stay ``""``)."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        header = tuple(reader.fieldnames or ())
        rows = [dict(row) for row in reader]
    return header, rows


def _lines_mentioning(text: str, needle: str) -> list[str]:
    """Every line of ``text`` that names ``needle``."""
    return [line for line in text.splitlines() if needle in line]


def _frame_counts(line: str) -> dict[str, int]:
    """The machine-readable ``name=<int>`` counters of a warning line.

    Read back as a mapping instead of one brittle substring per test: the warning
    has to carry ``frames`` and ``frames_with_motor_table``, and *how* it spells
    the separators around them is not part of the contract -- the numbers are.
    """
    return {name: int(value) for name, value in re.findall(r"([A-Za-z_]+)=(\d+)", line)}


def _frame_rows(latency: Any, face: Any, **coordinates: Any) -> list[dict[str, Any]]:
    """Call ``safety_frame_rows`` with frozen coordinates unless overridden."""
    frozen: dict[str, Any] = {
        "step_index": 2,
        "sample_sequence": 9,
        "t_monotonic_s": 12.5,
    }
    frozen.update(coordinates)
    return latency.safety_frame_rows(face, **frozen)


# ===========================================================================
# A. Frozen constants
# ===========================================================================


def test_safety_csv_fields_should_be_the_frozen_nine_columns(latency: Any) -> None:
    """The sidecar column tuple is frozen, in order, and free of duplicates.

    ``target_source`` is the ninth column and sits directly after ``hold_active``;
    the eight columns before it keep their order and their meaning (the main CSV
    and the evidence files are parsed by position, only the sidecar grew).
    """
    assert isinstance(latency.SAFETY_CSV_FIELDS, tuple)
    assert latency.SAFETY_CSV_FIELDS == (
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
    assert len(latency.SAFETY_CSV_FIELDS) == 9, (
        f"the sidecar has to be nine columns, got {latency.SAFETY_CSV_FIELDS!r}"
    )
    assert latency.SAFETY_CSV_FIELDS[7] == "hold_active", (
        f"target_source has to be appended after the hold flag: {latency.SAFETY_CSV_FIELDS!r}"
    )
    assert latency.SAFETY_CSV_FIELDS[8] == "target_source", (
        f"target_source has to be the last column: {latency.SAFETY_CSV_FIELDS!r}"
    )
    assert len(set(latency.SAFETY_CSV_FIELDS)) == 9


def test_tracking_csv_fields_should_still_be_the_frozen_fourteen_columns(latency: Any) -> None:
    """The main tracking table did not grow a safety column."""
    assert latency.TRACKING_CSV_FIELDS == (
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
    assert len(latency.TRACKING_CSV_FIELDS) == 14


def test_safety_client_capabilities_should_name_the_safety_face(latency: Any) -> None:
    """Only ``latest_safety_status`` is the safety capability."""
    assert latency.SAFETY_CLIENT_CAPABILITIES == ("latest_safety_status",)


# ===========================================================================
# B. ``safety_csv_path`` — pure path arithmetic
# ===========================================================================


def test_safety_csv_path_should_place_the_sidecar_next_to_the_main_csv(latency: Any) -> None:
    """A timestamped ``str`` path becomes a sibling ``-safety.csv`` ``Path``."""
    main_csv = "/tmp/artifacts/alohamini-io/tracking-20261025T000000Z.csv"

    sidecar = latency.safety_csv_path(main_csv)

    assert isinstance(sidecar, Path), f"expected a Path, got {type(sidecar)!r}"
    assert sidecar == Path("/tmp/artifacts/alohamini-io/tracking-20261025T000000Z-safety.csv")
    assert sidecar.parent == Path("/tmp/artifacts/alohamini-io")


def test_safety_csv_path_should_accept_a_path_object(latency: Any) -> None:
    """A ``Path`` in, the same sidecar out."""
    main_csv = Path("/tmp/artifacts/alohamini-io/tracking.csv")

    assert latency.safety_csv_path(main_csv) == Path(
        "/tmp/artifacts/alohamini-io/tracking-safety.csv"
    )


def test_safety_csv_path_should_keep_the_dots_of_a_multi_dot_name(latency: Any) -> None:
    """Only the trailing ``.csv`` suffix is removed: ``a.b.csv`` → ``a.b-safety.csv``."""
    sidecar = latency.safety_csv_path(Path("/tmp/a.b.csv"))

    assert sidecar.name == "a.b-safety.csv"
    assert sidecar == Path("/tmp/a.b-safety.csv")


def test_safety_csv_path_should_keep_the_dots_of_the_parent_directory(latency: Any) -> None:
    """Dots in the directory name are not a suffix (a ``split('.')[0]`` would eat them)."""
    sidecar = latency.safety_csv_path(Path("/tmp/dir.with.dots/tracking.csv"))

    assert sidecar == Path("/tmp/dir.with.dots/tracking-safety.csv")


def test_safety_csv_path_should_suffix_a_name_without_a_csv_suffix(latency: Any) -> None:
    """A name that never had ``.csv`` still gets the sidecar suffix."""
    assert latency.safety_csv_path(Path("/tmp/notes")) == Path("/tmp/notes-safety.csv")


# ===========================================================================
# C. ``missing_safety_capabilities`` — pure probe
# ===========================================================================


class _ProbeClient:
    """Client stub that records every attribute name looked up on it."""

    def __init__(self, *, publishes_face: bool) -> None:
        """Record nothing yet; ``publishes_face`` decides what ``hasattr`` finds."""
        self.touched: list[str] = []
        if publishes_face:
            object.__setattr__(self, "latest_safety_status", {"currents_ma": {}})

    def __getattr__(self, name: str) -> Any:
        """Record the lookup and answer like an object without that attribute."""
        self.touched.append(name)
        raise AttributeError(name)


def test_missing_safety_capabilities_should_be_empty_for_a_client_that_publishes_the_face(
    latency: Any,
) -> None:
    """A client carrying ``latest_safety_status`` is complete."""
    assert latency.missing_safety_capabilities(_SafetyFake()) == []


def test_missing_safety_capabilities_should_name_the_absent_face(latency: Any) -> None:
    """The exact string the warning has to quote."""
    client = _ProbeClient(publishes_face=False)

    missing = latency.missing_safety_capabilities(client)

    assert missing == ["latest_safety_status"], f"expected the face name, got {missing!r}"
    assert isinstance(missing, list)


def test_missing_safety_capabilities_should_only_probe_the_named_attribute(latency: Any) -> None:
    """Pure: it touches the probed name and nothing else (it reads no state)."""
    client = _ProbeClient(publishes_face=False)

    latency.missing_safety_capabilities(client)

    assert client.touched == ["latest_safety_status"], (
        f"the probe read more than the capability name: {client.touched!r}"
    )


# ===========================================================================
# D. ``safety_frame_rows`` — pure, never raises, one row per motor
# ===========================================================================


def test_safety_frame_rows_should_return_exactly_the_declared_columns(latency: Any) -> None:
    """Every row carries the nine declared keys, no more and no less."""
    rows = _frame_rows(latency, _face_for_poll(0))

    assert len(rows) == 2, f"one row per motor of currents_ma, got {len(rows)}"
    for row in rows:
        assert set(row) == set(latency.SAFETY_CSV_FIELDS), f"row keys: {sorted(row)}"
        assert len(row) == 9
        assert "target_source" in row, f"the ninth column is missing from the row: {sorted(row)}"


def test_safety_frame_rows_should_pass_the_frame_coordinates_through(latency: Any) -> None:
    """``None`` face: one row, ``motor=None``, the coordinates unchanged."""
    rows = _frame_rows(latency, None, step_index=4, sample_sequence=77, t_monotonic_s=None)

    assert len(rows) == 1
    row = rows[0]
    assert row["step_index"] == 4
    assert row["sample_sequence"] == 77
    assert row["t_monotonic_s"] is None
    assert row["motor"] is None
    assert row["current_ma"] is None
    assert row["requested_target"] is None
    assert row["accepted_target"] is None
    assert row["hold_active"] is None


def test_safety_frame_rows_should_return_one_none_motor_row_for_an_empty_mapping(
    latency: Any,
) -> None:
    """An empty face is "nothing to say", not "no frame": one placeholder row."""
    rows = _frame_rows(latency, {})

    assert len(rows) == 1
    assert rows[0]["motor"] is None
    assert rows[0]["current_ma"] is None


def test_safety_frame_rows_should_return_one_none_motor_row_when_currents_ma_is_empty(
    latency: Any,
) -> None:
    """No motor keys is the same as no motor list: exactly one row, ``motor=None``."""
    rows = _frame_rows(latency, {"currents_ma": {}, "joint_holds": []})

    assert len(rows) == 1
    assert rows[0]["motor"] is None
    assert rows[0]["hold_active"] is None


@pytest.mark.parametrize("currents_ma", ["nope", [], None, 5.0], ids=["str", "list", "none", "num"])
def test_safety_frame_rows_should_return_one_none_motor_row_when_currents_ma_is_not_a_mapping(
    latency: Any, currents_ma: Any
) -> None:
    """A ``currents_ma`` that is not a mapping carries no motor name."""
    rows = _frame_rows(latency, {"currents_ma": currents_ma, "joint_holds": [_MOTOR]})

    assert len(rows) == 1, f"expected exactly one placeholder row, got {len(rows)}"
    assert rows[0]["motor"] is None
    assert rows[0]["current_ma"] is None
    # No motor name means no hold judgement either, even with a hold container.
    assert rows[0]["hold_active"] is None


def test_safety_frame_rows_should_return_one_none_motor_row_when_a_motor_name_is_not_a_string(
    latency: Any,
) -> None:
    """A non-``str`` key would reach the CSV as a numeric motor name: no motor list."""
    rows = _frame_rows(latency, {"currents_ma": {1: 2.0, _MOTOR: 5.0}})

    assert len(rows) == 1, f"expected one placeholder row, got {len(rows)}"
    assert rows[0]["motor"] is None


@pytest.mark.parametrize(
    "face",
    [object(), "nope", [1, 2], 42, b"bytes"],
    ids=["object", "str", "list", "int", "bytes"],
)
def test_safety_frame_rows_should_never_raise_for_a_face_that_is_not_a_mapping(
    latency: Any, face: Any
) -> None:
    """Any value may sit on the face: a non-mapping yields the placeholder row."""
    rows = _frame_rows(latency, face)

    assert len(rows) == 1
    assert rows[0]["motor"] is None
    assert rows[0]["current_ma"] is None


def test_safety_frame_rows_should_emit_one_row_per_motor_in_mapping_order(latency: Any) -> None:
    """Rows follow the ``currents_ma`` iteration order, with signed currents."""
    face = {"currents_ma": {_MOTOR: 120.0, _OTHER_MOTOR: -45.5}, "joint_holds": []}

    rows = _frame_rows(latency, face)

    assert [row["motor"] for row in rows] == [_MOTOR, _OTHER_MOTOR]
    assert [row["current_ma"] for row in rows] == [120.0, -45.5]


def test_safety_frame_rows_should_turn_an_integer_current_into_a_float(latency: Any) -> None:
    """``12`` mA is a measurement; the column holds ``12.0``, a float."""
    row = _frame_rows(latency, {"currents_ma": {_MOTOR: 12}})[0]

    assert row["current_ma"] == 12.0
    assert isinstance(row["current_ma"], float)


@pytest.mark.parametrize("current_ma", [True, False], ids=["true", "false"])
def test_safety_frame_rows_should_leave_a_bool_current_empty(latency: Any, current_ma: Any) -> None:
    """``True`` is not a current: ``float(True) == 1.0`` would be a fake 1 mA."""
    row = _frame_rows(latency, {"currents_ma": {_MOTOR: current_ma}})[0]

    assert row["motor"] == _MOTOR
    assert row["current_ma"] is None, f"a bool must never become a current, got {row!r}"


@pytest.mark.parametrize(
    "current_ma",
    ["abc", None, float("nan"), float("inf"), float("-inf")],
    ids=["text", "none", "nan", "inf", "-inf"],
)
def test_safety_frame_rows_should_leave_a_non_numeric_current_empty(
    latency: Any, current_ma: Any
) -> None:
    """Non-finite or non-numeric values cannot be a current."""
    row = _frame_rows(latency, {"currents_ma": {_MOTOR: current_ma}})[0]

    assert row["current_ma"] is None


def test_safety_frame_rows_should_leave_a_current_too_large_for_a_float_empty(
    latency: Any,
) -> None:
    """``10**400`` overflows ``float()``: the cell stays empty and nothing raises.

    ``float(10**400)`` raises ``OverflowError``, which is not a ``TypeError`` /
    ``ValueError`` -- an int is numeric, it simply has no double.  The value is
    client state, so the sidecar contract ("never raise") decides: an empty cell.
    """
    row = _frame_rows(latency, {"currents_ma": {_MOTOR: 10**400}})[0]

    assert row["motor"] == _MOTOR
    assert row["current_ma"] is None


def test_safety_frame_rows_should_read_a_bare_name_target_table(latency: Any) -> None:
    """A *bare-name* target table is read too: both key shapes are recognised.

    The vendor frame keys these tables ``<motor>.pos`` (see ``_TARGET_KEY``), but a
    bare-name table is the older / synthetic shape and must not crash or come out
    blank: the two shapes are the same information, and an implementation that
    only knows one of them silently drops the other.  This is the second key shape
    the file header says has its own case, so it gets its own named test instead of
    living on as a dead docstring inside the ``10**400`` one.
    """
    face = {
        "currents_ma": {_MOTOR: 120.0},
        "requested_targets": {_MOTOR: 3},
        "accepted_targets": {_MOTOR: 2.5},
    }

    row = _frame_rows(latency, face)[0]

    assert row["requested_target"] == 3.0
    assert isinstance(row["requested_target"], float)
    assert row["accepted_target"] == 2.5


def test_safety_frame_rows_should_read_a_target_keyed_by_the_dot_pos_motor_name(
    latency: Any,
) -> None:
    """The vendor keys both target tables by ``<motor>.pos``: the row still lands per motor.

    Critical 1.  This is the shape the real host publishes
    (``alohamini.py:987-988``: ``dict(self._arm_goal_positions)`` /
    ``dict(self._arm_sent_positions)``, filled from the ``left_pos`` / ``right_pos``
    action keys), so an implementation that looks the bare motor name up in those
    tables writes two empty target columns for every frame of every real run --
    including the shoulder, whose target is the whole point of the column.
    """
    face = {
        "currents_ma": {_MOTOR: 120.0, _OTHER_MOTOR: -45.5},
        "requested_targets": {_TARGET_KEY: 1.5, _OTHER_TARGET_KEY: -2.5},
        "accepted_targets": {_TARGET_KEY: 1.4375, _OTHER_TARGET_KEY: -2.625},
    }

    rows = _frame_rows(latency, face)
    by_motor = {row["motor"]: row for row in rows}

    assert set(by_motor) == {_MOTOR, _OTHER_MOTOR}
    assert by_motor[_MOTOR]["requested_target"] == 1.5
    assert by_motor[_MOTOR]["accepted_target"] == 1.4375
    assert by_motor[_OTHER_MOTOR]["requested_target"] == -2.5
    assert by_motor[_OTHER_MOTOR]["accepted_target"] == -2.625


def test_safety_frame_rows_should_keep_requested_and_accepted_apart_when_the_target_is_clipped(
    latency: Any,
) -> None:
    """A clipped command keeps both numbers: the goal and what the host actually took.

    This is the "did the shoulder target get held back" judgement, so no column may
    be written from the other mapping (and neither may be derived from the current).
    """
    face = {
        "currents_ma": {_MOTOR: 310.0},
        "requested_targets": {_TARGET_KEY: 12.0},
        "accepted_targets": {_TARGET_KEY: 9.5},
    }

    row = _frame_rows(latency, face)[0]

    assert row["requested_target"] == 12.0
    assert row["accepted_target"] == 9.5
    assert row["requested_target"] != row["accepted_target"]


def test_safety_frame_rows_should_accept_a_frame_that_mixes_both_target_key_shapes(
    latency: Any,
) -> None:
    """One frame, two key shapes: each motor's target is found in whichever table names it."""
    face = {
        "currents_ma": {_MOTOR: 120.0, _OTHER_MOTOR: -45.5},
        "requested_targets": {_TARGET_KEY: 1.5, _OTHER_MOTOR: -2.5},
        "accepted_targets": {_MOTOR: 1.25, _OTHER_TARGET_KEY: -2.75},
    }

    by_motor = {row["motor"]: row for row in _frame_rows(latency, face)}

    assert by_motor[_MOTOR]["requested_target"] == 1.5
    assert by_motor[_MOTOR]["accepted_target"] == 1.25
    assert by_motor[_OTHER_MOTOR]["requested_target"] == -2.5
    assert by_motor[_OTHER_MOTOR]["accepted_target"] == -2.75


def test_safety_frame_rows_should_not_fill_a_missing_target_with_zero(latency: Any) -> None:
    """A motor absent from the target mappings has no target, not a target of 0.

    Written in the vendor key shape (``<motor>.pos``) on purpose: the no-``0``
    refusal has to hold on the shape a real frame carries, not only on a bare-name
    one.
    """
    face = {
        "currents_ma": {_MOTOR: 120.0, _OTHER_MOTOR: -45.5},
        "requested_targets": {_TARGET_KEY: 1.5},
        "accepted_targets": {_TARGET_KEY: 1.5},
    }

    rows = _frame_rows(latency, face)
    by_motor = {row["motor"]: row for row in rows}

    assert by_motor[_MOTOR]["requested_target"] == 1.5
    assert by_motor[_OTHER_MOTOR]["requested_target"] is None
    assert by_motor[_OTHER_MOTOR]["requested_target"] != 0.0
    assert by_motor[_OTHER_MOTOR]["requested_target"] != 0
    assert by_motor[_OTHER_MOTOR]["accepted_target"] is None
    assert by_motor[_OTHER_MOTOR]["accepted_target"] != 0.0


def test_safety_frame_rows_should_leave_a_bool_target_empty(latency: Any) -> None:
    """A bool target is not a target, in either column."""
    face = {
        "currents_ma": {_MOTOR: 1.0},
        "requested_targets": {_MOTOR: True},
        "accepted_targets": {_MOTOR: False},
    }

    row = _frame_rows(latency, face)[0]

    assert row["requested_target"] is None
    assert row["accepted_target"] is None


def test_safety_frame_rows_should_leave_a_bool_dot_pos_target_empty(latency: Any) -> None:
    """The vendor ``<motor>.pos`` lookup goes through the same value rule: a bool is no target.

    Whichever key shape carried the value, ``True`` is a flag and not a target: the
    conversion must happen once, behind the type rule, or the ``.pos`` path becomes
    a second, laxer entry point into the same column.
    """
    face = {
        "currents_ma": {_MOTOR: 1.0},
        "requested_targets": {_TARGET_KEY: True},
        "accepted_targets": {_TARGET_KEY: False},
    }

    row = _frame_rows(latency, face)[0]

    assert row["requested_target"] is None
    assert row["accepted_target"] is None


@pytest.mark.parametrize(
    "container",
    [
        pytest.param(lambda motor: [motor], id="list"),
        pytest.param(lambda motor: (motor,), id="tuple"),
        pytest.param(lambda motor: {motor}, id="set"),
        pytest.param(lambda motor: {motor: True}, id="mapping"),
    ],
)
def test_safety_frame_rows_should_read_the_hold_flag_from_a_container(
    latency: Any, container: Any
) -> None:
    """Membership in any of the four hold containers is a hold."""
    face = {"currents_ma": {_MOTOR: 120.0}, "joint_holds": container(_MOTOR)}

    row = _frame_rows(latency, face)[0]

    assert row["hold_active"] is True


def test_safety_frame_rows_should_report_false_for_a_motor_that_is_not_held(latency: Any) -> None:
    """A readable hold container that lacks the motor answers ``False``, not ``None``."""
    face = {"currents_ma": {_MOTOR: 120.0}, "joint_holds": [_OTHER_MOTOR]}

    assert _frame_rows(latency, face)[0]["hold_active"] is False


@pytest.mark.parametrize(
    "joint_holds",
    [5, _MOTOR, None, True],
    ids=["int", "joint_name", "none", "bool"],
)
def test_safety_frame_rows_should_leave_the_hold_flag_empty_for_a_non_container(
    latency: Any, joint_holds: Any
) -> None:
    """``_MOTOR in _MOTOR`` is ``True`` for a substring, so a ``str`` is not a container."""
    face = {"currents_ma": {_MOTOR: 120.0}, "joint_holds": joint_holds}

    assert _frame_rows(latency, face)[0]["hold_active"] is None


def test_safety_frame_rows_should_never_raise_for_a_corrupt_currents_mapping(latency: Any) -> None:
    """The "never raises" clause covers a mapping whose readers explode too."""
    face = {"currents_ma": _CorruptCurrents(), "joint_holds": [_MOTOR]}

    rows = _frame_rows(latency, face)

    assert len(rows) == 1
    assert rows[0]["motor"] is None
    assert rows[0]["current_ma"] is None


# ---------------------------------------------------------------------------
# D2. ``target_source`` — the ninth column, a WHOLE-frame attribute
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("source", _TARGET_SOURCE_VALUES)
def test_safety_frame_rows_should_stamp_every_row_of_a_frame_with_its_target_source(
    latency: Any, source: str
) -> None:
    """One frame, two motors, one source: the attribute is per frame, not per motor.

    ``target_source`` sits on the frame (``face["target_source"]``), so both rows
    of a frame carry the same value and every one of the four vendor values is
    written through unchanged.  A lookup in the per-motor tables
    (``requested_targets`` / ``accepted_targets``, keyed ``<motor>.pos``) finds
    nothing there and writes two empty cells for every frame of a real run.
    """
    rows = _frame_rows(latency, _face_with_source(source))

    assert len(rows) == 2, f"one row per motor of currents_ma, got {len(rows)}"
    written = [row.get("target_source") for row in rows]
    assert written == [source, source], (
        f"every row of the frame has to carry {source!r}: {written!r}"
    )
    assert len({row.get("target_source") for row in rows}) == 1


def test_safety_frame_rows_should_leave_target_source_empty_when_the_frame_has_none(
    latency: Any,
) -> None:
    """No ``target_source`` key: an empty cell, and no other column moves.

    A defaulted read (``getattr``/``dict.get`` with a fallback) turns "the frame
    said nothing" into a fabricated value, and a per-motor read would move the
    empty cell into a different shape.  The comparison below is the other half of
    the refusal: the missing key may cost exactly its own column.
    """
    with_source = _frame_rows(latency, _face_with_source("command"))
    without_source = _frame_rows(latency, _face_with_source(include=False))

    for row in without_source:
        assert "target_source" in row, f"the column has to exist and be empty: {sorted(row)}"
        assert len(row) == 9, f"the row is not nine columns wide: {sorted(row)}"
    assert [row.get("target_source") for row in without_source] == [None, None], (
        f"a frame without the key has no source to report: "
        f"{[row.get('target_source') for row in without_source]!r}"
    )
    other_columns = [
        {key: value for key, value in row.items() if key != "target_source"}
        for row in without_source
    ]
    expected = [
        {key: value for key, value in row.items() if key != "target_source"} for row in with_source
    ]
    assert other_columns == expected, (
        f"the missing key changed another column: {other_columns!r} != {expected!r}"
    )


@pytest.mark.parametrize(
    "source",
    [5, 5.0, True, False, ["command"], {"command"}, "manual", "", "None"],
    ids=[
        "int",
        "float",
        "true",
        "false",
        "list",
        "set",
        "unknown-str",
        "empty-str",
        "capitalised",
    ],
)
def test_safety_frame_rows_should_leave_an_unusable_target_source_empty(
    latency: Any, source: Any
) -> None:
    """Anything but one of the four vendor strings is no source: the cell stays empty.

    ``True`` is an ``int`` in Python, so a bare ``str(value)`` / ``in`` check that
    skips the type rule would write ``"True"``; an unvalidated pass-through would
    write ``"manual"``.  Both read like a vendor statement the host never made.
    The rest of the frame is still read -- an unusable source costs its own column
    and nothing else.
    """
    rows = _frame_rows(latency, _face_with_source(source))

    for row in rows:
        assert "target_source" in row, f"the column has to exist and be empty: {sorted(row)}"
        assert len(row) == 9, f"the row is not nine columns wide: {sorted(row)}"
    assert [row.get("target_source") for row in rows] == [None, None], (
        f"{source!r} is not one of {_TARGET_SOURCE_VALUES}: "
        f"{[row.get('target_source') for row in rows]!r}"
    )
    assert [row["motor"] for row in rows] == [_MOTOR, _OTHER_MOTOR]
    assert [row["current_ma"] for row in rows] == [120.0, -45.5]
    assert [row["hold_active"] for row in rows] == [False, False]


def test_safety_frame_rows_should_stamp_the_corrupt_currents_row_with_the_frame_source(
    latency: Any,
) -> None:
    """A frame whose motor table is unreadable still said which layer produced it.

    ``target_source`` is a whole-frame attribute, so the degraded ``motor=None``
    row -- the one shape that has no motor to be per-motor about -- has to carry
    the frame's own value.  Dropping it there loses the only column that can
    answer "was this joint being held back", which is exactly the frame an
    operator reaches for after an over-current event, and it loses it silently:
    an empty cell reads like "the frame said nothing".  Every other column stays
    empty and the coordinates stay the caller's, so the source is the one thing
    the degradation does not take away.
    """
    face = {
        "currents_ma": _CorruptCurrents(),
        "joint_holds": [_MOTOR],
        "target_source": "protection",
    }

    rows = _frame_rows(latency, face)

    assert len(rows) == 1, f"a frame with no motor table is one placeholder row: {rows!r}"
    row = rows[0]
    assert set(row) == set(latency.SAFETY_CSV_FIELDS), f"row keys: {sorted(row)}"
    assert row["motor"] is None
    assert row["target_source"] == "protection", (
        f"the degraded row has to carry the frame's target_source, got {row['target_source']!r}"
    )
    for column in ("current_ma", "requested_target", "accepted_target", "hold_active"):
        assert row[column] is None, f"{column} is no measurement on this frame: {row[column]!r}"
    assert (row["step_index"], row["sample_sequence"], row["t_monotonic_s"]) == (2, 9, 12.5)


@pytest.mark.parametrize(
    "face",
    [object(), "command", None],
    ids=["object", "str-vendor-value", "none"],
)
def test_safety_frame_rows_should_leave_the_degraded_source_empty_for_a_non_mapping_face(
    latency: Any, face: Any
) -> None:
    """A face that is no ``Mapping`` carries no frame attribute at all.

    ``"command"`` is deliberate: the *string* is one of the four vendor values,
    but here it is the whole face, not ``face["target_source"]``.  A reader that
    falls back to the face itself, or that checks the value without first
    checking that the frame is a mapping, would stamp the placeholder row with a
    statement the host never made.
    """
    rows = _frame_rows(latency, face)

    assert len(rows) == 1, f"a non-mapping face is one placeholder row: {rows!r}"
    assert rows[0]["motor"] is None
    assert rows[0]["target_source"] is None, (
        f"a face that is no mapping has no target_source: {rows[0]['target_source']!r}"
    )


def test_safety_frame_rows_should_stamp_the_empty_currents_row_with_the_frame_source(
    latency: Any,
) -> None:
    """``currents_ma == {}`` is the deployed degradation: the source survives it.

    The vendor client's face always exists and is often *empty*, so this is the
    shape a real run hits (not the absent attribute).  The motor table is empty,
    the frame still names the layer that produced its targets, and that one cell
    is filled while the motor-dependent cells stay empty.
    """
    face = {"currents_ma": {}, "joint_holds": [], "target_source": "command"}

    rows = _frame_rows(latency, face)

    assert len(rows) == 1, f"no motor keys is one placeholder row, got {len(rows)}"
    assert rows[0]["motor"] is None
    assert rows[0]["target_source"] == "command", (
        f"the empty-table row has to carry the frame's target_source, "
        f"got {rows[0]['target_source']!r}"
    )
    assert rows[0]["current_ma"] is None


# ===========================================================================
# E. ``run_tracking_steps`` — the summary carries the safety rows
# ===========================================================================


def test_run_tracking_steps_should_record_one_row_per_motor_per_growing_frame(latency: Any) -> None:
    """2 steps = 4 growing frames (step-start + settle poll), 2 motors each = 8 rows.

    A recorder that only books the step tail writes 4 rows, so both polling
    sites are required by this one count.
    """
    client = _SafetyFake()

    summary = _run_tracking(latency, client)

    rows = summary["safety_rows"]
    assert client.get_observation_calls == 4, f"the frame budget moved: {client.observations}"
    assert len(rows) == 8, f"expected 4 growing frames x 2 motors, got {len(rows)} rows"
    assert [row["sample_sequence"] for row in rows] == [
        sequence for sequence in client.observations for _ in range(2)
    ]
    by_frame: dict[int, list[str]] = {}
    for row in rows:
        by_frame.setdefault(row["sample_sequence"], []).append(row["motor"])
    assert by_frame == {
        1: [_MOTOR, _OTHER_MOTOR],
        2: [_MOTOR, _OTHER_MOTOR],
        3: [_MOTOR, _OTHER_MOTOR],
        4: [_MOTOR, _OTHER_MOTOR],
    }
    # Frames 1-2 belong to step 0, frames 3-4 to step 1 (two rows per frame).
    assert [row["step_index"] for row in rows] == [0, 0, 0, 0, 1, 1, 1, 1]


def test_run_tracking_steps_should_stamp_each_row_with_the_frame_arrival_stamp(
    latency: Any,
) -> None:
    """``t_monotonic_s`` is when the frame arrived, not the poll-clock reading."""
    summary = _run_tracking(latency, _SafetyFake(), clock=_FrozenClock(500.0))

    stamps = {row["sample_sequence"]: row["t_monotonic_s"] for row in summary["safety_rows"]}

    assert stamps == {1: 1000.0, 2: 1001.0, 3: 1002.0, 4: 1003.0}


def test_run_tracking_steps_should_fall_back_to_the_injected_clock_when_the_frame_is_untimed(
    latency: Any,
) -> None:
    """No ``received_monotonic_s`` on the frame: the injected clock stamps the row."""
    summary = _run_tracking(latency, _SafetyFake(timed=False), clock=_FrozenClock(250.0))

    rows = summary["safety_rows"]

    assert len(rows) == 8
    assert {row["t_monotonic_s"] for row in rows} == {250.0}


@pytest.mark.parametrize(
    ("face", "expected"),
    [
        pytest.param(_face_for_poll, [], id="published"),
        pytest.param(_NO_FACE, ["latest_safety_status"], id="absent"),
    ],
)
def test_run_tracking_steps_should_report_the_safety_capability_probe(
    latency: Any, face: Any, expected: list[str]
) -> None:
    """``summary["safety_missing"]`` is the probe result of the run's start."""
    summary = _run_tracking(latency, _SafetyFake(face=face))

    assert summary["safety_missing"] == expected


def test_run_tracking_steps_should_leave_every_safety_cell_empty_without_the_face(
    latency: Any,
) -> None:
    """A missing face still records one row per growing frame, all cells ``None``."""
    summary = _run_tracking(latency, _SafetyFake(face=_NO_FACE))

    rows = summary["safety_rows"]

    assert len(rows) == 4, f"one row per growing frame, got {len(rows)}"
    assert [row["sample_sequence"] for row in rows] == [1, 2, 3, 4]
    for row in rows:
        assert row["motor"] is None
        assert row["current_ma"] is None
        assert row["requested_target"] is None
        assert row["accepted_target"] is None
        assert row["hold_active"] is None


def test_run_tracking_steps_should_not_repeat_a_stale_frame(latency: Any) -> None:
    """2 growing frames out of 5 polls = 4 rows, not 10 rows / 5 frame numbers.

    ``_SafetyFake(sequence_freezes_after=2)`` freezes the sequence after two
    calls; the loop then keeps polling (the client answers a timed out
    observation with its cached frame) until the stall limit aborts the run.  A
    recorder that books every poll writes one row per poll instead.
    """
    client = _SafetyFake(sequence_freezes_after=2)

    summary = _run_tracking(latency, client, steps=3)

    rows = summary["safety_rows"]
    assert client.get_observation_calls == 5, (
        f"the stalled run no longer costs 5 polls: {client.observations}"
    )
    assert [row["sample_sequence"] for row in rows] == [1, 1, 2, 2], (
        f"the recorded frames are not the two that grew: {[r['sample_sequence'] for r in rows]}"
    )
    assert len(rows) == 4, f"expected 2 growing frames x 2 motors, got {len(rows)} rows"
    assert len({(row["sample_sequence"], row["motor"]) for row in rows}) == 4
    assert summary["steps_completed"] < 3


def test_run_tracking_steps_should_record_the_hold_flag_per_motor(latency: Any) -> None:
    """``joint_holds`` membership is per motor, in memory, as a real bool."""
    face = dict(_face_for_poll(0), joint_holds=[_MOTOR])

    rows = _run_tracking(latency, _SafetyFake(face=face))["safety_rows"]

    held = {row["motor"]: row["hold_active"] for row in rows if row["sample_sequence"] == 1}

    assert held == {_MOTOR: True, _OTHER_MOTOR: False}
    assert held[_MOTOR] is True
    assert held[_OTHER_MOTOR] is False


def test_run_tracking_steps_should_read_target_source_per_recorded_frame(latency: Any) -> None:
    """Frame 1 carries ``"command"``, frame 2 carries nothing: each row answers for its own frame.

    A recorder that reads the attribute once (at the run start, or at the first
    frame) and reuses it writes ``"command"`` on all four frames; one that reads
    it per motor writes nothing anywhere.  The alternating frame script pins both:
    the four recorded frames are ``command`` / empty / ``command`` / empty, with
    both motors of a frame agreeing with each other.
    """

    def face(index: int) -> Any:
        """Every even poll carries the source, every odd poll does not."""
        frame = _face_for_poll(index)
        if index % 2 == 0:
            frame["target_source"] = "command"
        return frame

    rows = _run_tracking(latency, _SafetyFake(face=face))["safety_rows"]

    by_frame: dict[Any, list[Any]] = {}
    for row in rows:
        by_frame.setdefault(row["sample_sequence"], []).append(row.get("target_source"))
    assert by_frame == {
        1: ["command", "command"],
        2: [None, None],
        3: ["command", "command"],
        4: [None, None],
    }, f"each frame has to answer for itself: {by_frame!r}"


# ===========================================================================
# F. The sidecar on disk (tracking path only) and the untouched main CSV
# ===========================================================================


def test_main_tracking_should_write_a_safety_sidecar_next_to_the_tracking_csv(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """4 growing frames x 2 motors = 8 sidecar rows, one snapshot per frame.

    Every frame of this run carries a readable motor table, so the run-wide
    degradation warning is not allowed to fire: exactly zero safety lines on
    stderr.  Without this case the new criterion ("at least one frame without a
    motor table") could be implemented as "warn always" and still look green.
    """
    out = tmp_path / "tracking-20261025T000000Z.csv"
    client = _SafetyFake()

    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the tracking run failed with {code}"
    safety_warnings = [line for line in printed.err.splitlines() if "warning" in line.lower()]
    assert safety_warnings == [], f"a fully healthy run warned: {safety_warnings!r}"
    assert _lines_mentioning(printed.err, "latest_safety_status") == [], (
        f"a fully healthy run named the safety face on stderr: {printed.err!r}"
    )
    sidecar = latency.safety_csv_path(out)
    assert sidecar == tmp_path / "tracking-20261025T000000Z-safety.csv"
    assert sidecar.is_file(), f"no sidecar next to {out}"
    header, rows = _read_csv(sidecar)
    assert header == latency.SAFETY_CSV_FIELDS
    # 1 preflight observation (no step yet, so not a step row) + 4 in-run frames.
    assert client.get_observation_calls == 5, f"frame budget: {client.observations}"
    assert len(rows) == 8, f"expected 4 growing frames x 2 motors, got {len(rows)} rows"
    by_frame: dict[int, list[dict[str, str]]] = {}
    for row in rows:
        by_frame.setdefault(int(row["sample_sequence"]), []).append(row)
    assert sorted(by_frame) == [2, 3, 4, 5], f"recorded frames: {sorted(by_frame)}"
    for frame_rows in by_frame.values():
        assert [row["motor"] for row in frame_rows] == [_MOTOR, _OTHER_MOTOR]
    # The current follows the poll index, so each frame is really a new snapshot.
    currents = {sequence: float(frame[0]["current_ma"]) for sequence, frame in by_frame.items()}
    assert currents == {2: 121.0, 3: 122.0, 4: 123.0, 5: 124.0}
    # The two target columns are filled from the vendor-shaped (``<motor>.pos``)
    # tables on disk as well: a bare-name-only lookup writes 8 empty cells here.
    targets = {
        sequence: (float(frame[0]["requested_target"]), float(frame[1]["accepted_target"]))
        for sequence, frame in by_frame.items()
    }
    assert targets == {2: (1.5, -2.5), 3: (1.5, -2.5), 4: (1.5, -2.5), 5: (1.5, -2.5)}


def test_main_tracking_should_degrade_to_empty_cells_when_the_safety_face_is_absent(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """No ``latest_safety_status``: the sidecar is written, empty, warned once.

    A silent ``getattr(client, "latest_safety_status", {})`` fallback writes the
    same empty sidecar; the single stderr line naming the face is what it cannot
    produce.

    The line has to say *the client publishes no face*, and it has to carry no
    frame counts.  Both halves are load-bearing: naming the face alone is not
    enough, because the neighbouring branch ("the face is there, N of M frames
    carried no motor table") also prints exactly one line naming it, with
    ``frames=M, frames_with_motor_table=K``.  A test that only counted the lines
    naming the face therefore stayed green with this branch deleted -- and the run
    then told the operator the opposite of what happened ("the attribute is there
    and is empty" instead of "there is no such attribute").  The absent branch has
    no recorded-frame statistics to report, which is why the counts are what
    separates the two wordings.
    """
    capable_out = tmp_path / "tracking-capable.csv"
    out = tmp_path / "tracking-but-no-face.csv"
    client = _SafetyFake(face=_NO_FACE)
    assert hasattr(client, "latest_safety_status") is False, "the fake still publishes the face"

    capable_code = latency.main(_main_argv(capable_out), client_factory=_Factory(_SafetyFake()))
    capsys.readouterr()
    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the degraded run must still exit 0, got {code}"
    assert code == capable_code, f"the probe changed the exit code: {capable_code} -> {code}"
    sidecar = latency.safety_csv_path(out)
    assert sidecar.is_file(), f"no sidecar next to {out}"
    header, rows = _read_csv(sidecar)
    assert header == latency.SAFETY_CSV_FIELDS
    assert len(rows) == 4, f"one row per growing frame, got {len(rows)}"
    for row in rows:
        assert row["motor"] == ""
        assert row["current_ma"] == ""
        assert row["requested_target"] == ""
        assert row["accepted_target"] == ""
        assert row["hold_active"] == ""
    warnings = _lines_mentioning(printed.err, "latest_safety_status")
    assert len(warnings) == 1, f"expected exactly one warning naming the face, got {warnings!r}"
    # The wording of the absent branch, pinned verbatim (it is the string
    # ``main`` prints for ``summary["safety_missing"]``): "publishes no
    # latest_safety_status", not "publishes latest_safety_status, but".
    assert "publishes no latest_safety_status" in warnings[0], (
        f"the absent face has to be reported as absent, not as an empty one: {warnings[0]!r}"
    )
    counts = _frame_counts(warnings[0])
    assert counts == {}, (
        f"this branch has no recorded-frame statistics to report; a line carrying "
        f"{counts!r} is the content branch's wording: {warnings[0]!r}"
    )
    assert "frames_with_motor_table" not in warnings[0], (
        f"the absent face has no motor-table count: {warnings[0]!r}"
    )
    # The line still has to point at the artefact it is talking about.
    assert str(sidecar) in warnings[0], f"the warning does not name the sidecar: {warnings[0]!r}"
    # The main CSV is untouched by the degraded sidecar.
    capable_header, capable_rows = _read_csv(capable_out)
    degraded_header, degraded_rows = _read_csv(out)
    assert degraded_header == capable_header
    assert len(degraded_rows) == len(capable_rows) == 2


def test_main_tracking_should_keep_the_frozen_fourteen_column_main_csv(
    latency: Any, tmp_path: Path
) -> None:
    """Regression guard: the safety columns stay in the sidecar, never in the main CSV."""
    out = tmp_path / "tracking-main.csv"

    code = latency.main(_main_argv(out), client_factory=_Factory(_SafetyFake()))

    assert code == latency.EXIT_OK
    header, rows = _read_csv(out)
    assert len(header) == 14, f"the main CSV grew a column: {header}"
    assert header == latency.TRACKING_CSV_FIELDS
    # ``step_index`` names a column of *both* tables, so a column of the sidecar
    # is only a leak of the main CSV when the main table does not already use
    # that name.  The shared set is pinned first: if the two tables ever stopped
    # sharing exactly ``step_index``, the leak check below would change meaning
    # (or go vacuous) instead of failing.
    shared_columns = set(latency.TRACKING_CSV_FIELDS) & set(latency.SAFETY_CSV_FIELDS)
    assert shared_columns == {"step_index"}, (
        f"the column shared by the main CSV and the sidecar moved: {sorted(shared_columns)}"
    )
    safety_only_columns = set(latency.SAFETY_CSV_FIELDS) - shared_columns
    assert len(safety_only_columns) == 8, (
        f"the sidecar-exclusive columns moved: {sorted(safety_only_columns)}"
    )
    leaked = [field for field in header if field in safety_only_columns]
    assert leaked == [], f"safety columns leaked into the main CSV: {leaked}"
    assert len(rows) == 2, f"one row per step, got {len(rows)}"


@pytest.mark.parametrize("currents_ma", ["nope", [], None], ids=["str", "list", "none"])
def test_main_tracking_should_survive_a_currents_ma_that_is_not_a_mapping(
    latency: Any, tmp_path: Path, currents_ma: Any
) -> None:
    """A broken face neither crashes the run nor writes a motor name."""
    out = tmp_path / "tracking-bad-currents.csv"
    client = _SafetyFake(face={"currents_ma": currents_ma, "joint_holds": [_MOTOR]})

    code = latency.main(_main_argv(out), client_factory=_Factory(client))

    assert code == latency.EXIT_OK, f"a corrupt side face must not change the exit code: {code}"
    header, rows = _read_csv(latency.safety_csv_path(out))
    assert header == latency.SAFETY_CSV_FIELDS
    assert len(rows) == 4, f"one row per growing frame, got {len(rows)}"
    for row in rows:
        assert row["motor"] == ""
        assert row["current_ma"] == ""
        assert row["requested_target"] == ""
        assert row["accepted_target"] == ""
        assert row["hold_active"] == ""


@pytest.mark.parametrize(
    "current_ma",
    ["abc", float("nan"), True],
    ids=["text", "nan", "bool"],
)
def test_main_tracking_should_survive_a_non_numeric_current(
    latency: Any, tmp_path: Path, current_ma: Any
) -> None:
    """The motor stays known; only the unusable current cell is empty."""
    out = tmp_path / "tracking-bad-current.csv"
    face = _face_for_poll(0)
    face["currents_ma"] = {_MOTOR: current_ma}
    client = _SafetyFake(face=face)

    code = latency.main(_main_argv(out), client_factory=_Factory(client))

    assert code == latency.EXIT_OK, f"a corrupt current must not change the exit code: {code}"
    _, rows = _read_csv(latency.safety_csv_path(out))
    assert len(rows) == 4
    for row in rows:
        assert row["motor"] == _MOTOR
        assert row["current_ma"] == "", f"expected an empty cell, got {row['current_ma']!r}"
        assert row["current_ma"] != "0"
        # Only the current degraded: the target of the same motor is still there.
        assert float(row["requested_target"]) == pytest.approx(1.5)


def test_main_tracking_should_not_fill_a_missing_target_with_zero(
    latency: Any, tmp_path: Path
) -> None:
    """A motor without a target keeps an empty cell, never ``0``.

    Written in the vendor key shape (``<motor>.pos``): the refusal has to hold on
    the shape a real frame carries, and it has to hold *per motor* -- the motor
    that is in the table keeps its value while the other one stays blank.
    """
    out = tmp_path / "tracking-missing-target.csv"
    face = _face_for_poll(0)
    face["requested_targets"] = {_TARGET_KEY: 1.5}
    face["accepted_targets"] = {_TARGET_KEY: 1.5}
    client = _SafetyFake(face=face)

    code = latency.main(_main_argv(out), client_factory=_Factory(client))

    assert code == latency.EXIT_OK
    _, rows = _read_csv(latency.safety_csv_path(out))
    assert len(rows) == 8
    by_motor = {row["motor"]: row for row in rows}
    assert by_motor[_OTHER_MOTOR]["requested_target"] == ""
    assert by_motor[_OTHER_MOTOR]["requested_target"] != "0"
    assert by_motor[_OTHER_MOTOR]["requested_target"] != "0.0"
    assert by_motor[_OTHER_MOTOR]["accepted_target"] == ""
    assert by_motor[_OTHER_MOTOR]["accepted_target"] != "0"
    assert float(by_motor[_MOTOR]["requested_target"]) == pytest.approx(1.5)


def test_main_tracking_should_not_change_the_exit_code_when_the_sidecar_cannot_be_written(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A sidecar that cannot be written is one warning; the run's verdict is the main CSV's.

    The sidecar path is occupied by a non-empty directory, so the write raises
    ``IsADirectoryError`` *after* the tracking CSV reached the disk.  The run
    measured what it measured, and only the currents snapshot is missing, so the
    verdict stays ``EXIT_OK`` (a ``write_csv`` bug that lets the sidecar's failure
    escape as ``EXIT_WRITE_FAILED`` would throw away a complete measurement).
    """
    out = tmp_path / "tracking-sidecar-blocked.csv"
    sidecar = latency.safety_csv_path(out)
    sidecar.mkdir()
    (sidecar / "occupied.txt").write_text("not a csv\n", encoding="utf-8")

    code = latency.main(_main_argv(out), client_factory=_Factory(_SafetyFake()))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"a sidecar write failure changed the exit code: {code}"
    assert code != latency.EXIT_WRITE_FAILED, (
        f"the sidecar lent its failure the main verdict: {code}"
    )
    header, rows = _read_csv(out)
    assert header == latency.TRACKING_CSV_FIELDS
    assert len(header) == 14, f"the main CSV grew a column: {header}"
    assert len(rows) == 2, f"one row per step, got {len(rows)}"
    warnings = _lines_mentioning(printed.err, "safety sidecar")
    assert len(warnings) == 1, f"expected exactly one sidecar warning, got {warnings!r}"
    assert "could not be written" in warnings[0], (
        f"the warning does not say what failed: {warnings!r}"
    )
    assert str(sidecar) in warnings[0], f"the warning does not name the path: {warnings!r}"


def test_main_tracking_should_not_abort_when_the_arrival_timing_face_is_pathological(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A timing face whose readers raise costs the arrival stamp, not the run.

    ``t_monotonic_s`` is read from ``latest_request_timing["received_monotonic_s"]``
    (``_arrival_monotonic_s``), at frame booking *and* at the 90 % crossing.  A
    ``Mapping`` that raises on ``__getitem__``/``get`` is client state like any
    other: the poll clock has to stamp the row instead.  An unguarded read starts a
    chain that ends in a lost run -- the booking site turns it into "feedback read
    failed before the step", i.e. a non-zero abort code, and the crossing site has
    no handler at all and escapes ``main``.
    """
    out = tmp_path / "tracking-pathological-timing.csv"
    client = _SafetyFake(timing=_ExplodingTiming())

    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, (
        f"the pathological timing face aborted the run: exit {code}, stderr={printed.err!r}"
    )
    assert code != latency.EXIT_STEP_RESPONSE_ABORTED, (
        f"the run was reported as an aborted step: exit {code}, stderr={printed.err!r}"
    )
    _, main_rows = _read_csv(out)
    assert len(main_rows) == 2, f"both steps have to be recorded, got {len(main_rows)}"
    header, rows = _read_csv(latency.safety_csv_path(out))
    assert header == latency.SAFETY_CSV_FIELDS
    assert len(rows) == 8, f"the frames went down with the stamp: got {len(rows)} rows"
    stamps = [row["t_monotonic_s"] for row in rows]
    assert all(stamp != "" for stamp in stamps), f"a row lost its stamp entirely: {stamps!r}"
    assert all(float(stamp) >= 0.0 for stamp in stamps), f"not a clock reading: {stamps!r}"


@pytest.mark.parametrize(
    "face",
    [
        pytest.param({}, id="empty-mapping"),
        pytest.param({"currents_ma": {}, "joint_holds": []}, id="empty-currents-ma"),
    ],
)
def test_main_tracking_should_warn_once_when_the_published_face_never_carries_a_motor_table(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str], face: Any
) -> None:
    """Critical 2: the face exists and fills nothing -- named once, not silently empty.

    On the deployed vendor client the attribute *always* exists (initial ``{}``), so
    the degradation that really happens is "present but no usable content", not
    "absent".  It gets the absent-face treatment -- empty cells, sidecar written,
    exit code untouched -- plus exactly one warning naming the face: an empty
    sidecar without a word is indistinguishable from "the currents really are
    blank".
    """
    capable_out = tmp_path / "tracking-capable.csv"
    out = tmp_path / "tracking-empty-face.csv"
    client = _SafetyFake(face=face)
    assert hasattr(client, "latest_safety_status"), "the fake still publishes the face"

    capable_code = latency.main(_main_argv(capable_out), client_factory=_Factory(_SafetyFake()))
    capsys.readouterr()
    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"an unfilled face must not change the exit code: {code}"
    assert code == capable_code, f"the warning changed the exit code: {capable_code} -> {code}"
    sidecar = latency.safety_csv_path(out)
    assert sidecar.is_file(), f"no sidecar next to {out}"
    header, rows = _read_csv(sidecar)
    assert header == latency.SAFETY_CSV_FIELDS
    assert len(header) == 9, f"the sidecar lost a column: {header}"
    assert len(rows) == 4, f"one placeholder row per growing frame, got {len(rows)}"
    for row in rows:
        assert row["motor"] == ""
        assert row["current_ma"] == ""
        assert row["requested_target"] == ""
        assert row["accepted_target"] == ""
        assert row["hold_active"] == ""
    warnings = _lines_mentioning(printed.err, "latest_safety_status")
    assert len(warnings) == 1, f"expected exactly one warning naming the face, got {warnings!r}"
    assert "warning" in warnings[0], f"the line does not read as a warning: {warnings!r}"
    counts = _frame_counts(warnings[0])
    assert counts.get("frames") == 4, (
        f"the warning has to count the 4 recorded frames, got {counts!r} in {warnings[0]!r}"
    )
    assert counts.get("frames_with_motor_table") == 0, (
        f"no frame of this run carried a table, got {counts!r} in {warnings[0]!r}"
    )
    # The main CSV is untouched by the unfilled face.
    capable_header, capable_rows = _read_csv(capable_out)
    degraded_header, degraded_rows = _read_csv(out)
    assert degraded_header == capable_header
    assert len(degraded_header) == 14, f"the main CSV grew a column: {degraded_header}"
    assert len(degraded_rows) == len(capable_rows) == 2


def test_main_tracking_should_warn_once_with_frame_counts_when_only_some_frames_carry_a_motor_table(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Two placeholder frames then two filled ones: one warning, counting 2 of 4.

    The old criterion ("not one frame of the run carried a table") said this run
    was fine, and it is not: half of its sidecar frames are empty, so an operator
    reading the file cannot tell a missing table from a corrosively blank one
    without being told.  The rule is the whole run's count, not its first frame and
    not "all frames": the run prints exactly one warning carrying
    ``frames=4 frames_with_motor_table=2`` -- and the two filled frames do carry
    their values, so the count is a count of the same rows an operator would see.
    """
    out = tmp_path / "tracking-late-face.csv"
    # Poll 0 is the preflight (not recorded) and polls 1-2 are the first two
    # recorded frames; from poll 3 on the face carries the healthy motor table.
    client = _SafetyFake(face=lambda index: {} if index < 3 else _face_for_poll(index))

    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the run failed with {code}"
    warnings = _lines_mentioning(printed.err, "latest_safety_status")
    assert len(warnings) == 1, (
        f"2 of the 4 recorded frames had no motor table: exactly one warning fits, got {warnings!r}"
    )
    assert "warning" in warnings[0], f"the line does not read as a warning: {warnings!r}"
    counts = _frame_counts(warnings[0])
    assert counts.get("frames") == 4, (
        f"the warning has to count the 4 recorded frames, got {counts!r} in {warnings[0]!r}"
    )
    assert counts.get("frames_with_motor_table") == 2, (
        f"two frames did carry a table, got {counts!r} in {warnings[0]!r}"
    )
    _, rows = _read_csv(latency.safety_csv_path(out))
    by_frame: dict[int, list[dict[str, str]]] = {}
    for row in rows:
        by_frame.setdefault(int(row["sample_sequence"]), []).append(row)
    assert sorted(by_frame) == [2, 3, 4, 5], f"recorded frames: {sorted(by_frame)}"
    # Two placeholder frames, then two frames with a motor table: 1 + 1 + 2 + 2 rows.
    assert len(rows) == 6, f"expected 6 rows, got {len(rows)}"
    assert [row["motor"] for row in by_frame[2]] == [""]
    assert [row["motor"] for row in by_frame[3]] == [""]
    assert [row["motor"] for row in by_frame[4]] == [_MOTOR, _OTHER_MOTOR]
    assert [row["motor"] for row in by_frame[5]] == [_MOTOR, _OTHER_MOTOR]
    assert float(by_frame[4][0]["current_ma"]) == 123.0
    assert float(by_frame[5][0]["current_ma"]) == 124.0


def test_main_tracking_should_warn_once_with_counts_when_only_one_frame_of_four_is_readable(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """3 broken frames of 4, each broken differently: one warning, ``frames=4`` / 1.

    This is the field shape the operator hit: the run recorded 4 frames, 3 of them
    had no readable ``currents_ma`` motor table, one was healthy -- and the old
    "not one frame carried a table" criterion stayed silent, leaving 3 blank
    ``motor`` cells in the sidecar unmentioned.  The three broken frames are broken
    in the three ways a real frame can be: a mapping whose readers raise, an empty
    ``currents_ma``, and a face that is no mapping at all.  The count has to see all
    three as one kind of failure (no readable motor table), print exactly one
    warning, and the sidecar still has to carry its rows -- 3 placeholder frames
    plus the 2 motors of the healthy one -- with an exit code of 0.
    """
    out = tmp_path / "tracking-mixed-face.csv"

    def broken(index: int) -> Any:
        """Frame content for poll ``index``: broken twice, empty once, healthy last."""
        if index == 1:
            return {"currents_ma": _CorruptCurrents()}
        if index == 2:
            return {"currents_ma": {}, "joint_holds": []}
        if index == 3:
            return "no frame at all"
        return _face_for_poll(index)

    client = _SafetyFake(face=broken)

    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, (
        f"a partly unreadable face must not change the exit code: {code}"
    )
    sidecar = latency.safety_csv_path(out)
    assert sidecar.is_file(), f"no sidecar next to {out}"
    header, rows = _read_csv(sidecar)
    assert header == latency.SAFETY_CSV_FIELDS
    by_frame: dict[int, list[dict[str, str]]] = {}
    for row in rows:
        by_frame.setdefault(int(row["sample_sequence"]), []).append(row)
    assert sorted(by_frame) == [2, 3, 4, 5], f"recorded frames: {sorted(by_frame)}"
    assert [row["motor"] for row in by_frame[2]] == [""]
    assert [row["motor"] for row in by_frame[3]] == [""]
    assert [row["motor"] for row in by_frame[4]] == [""]
    assert [row["motor"] for row in by_frame[5]] == [_MOTOR, _OTHER_MOTOR]
    # 3 placeholder frames + 2 motor rows: 5 rows, 3 of them with an empty motor cell.
    assert len(rows) == 5, f"expected 3 + 2 rows, got {len(rows)}"
    assert sum(1 for row in rows if row["motor"] == "") == 3, (
        f"the sidecar lost a placeholder row: {[r['motor'] for r in rows]!r}"
    )
    warnings = _lines_mentioning(printed.err, "latest_safety_status")
    assert len(warnings) == 1, (
        f"3 unreadable frames of 4 have to be named exactly once, got {warnings!r}"
    )
    assert "warning" in warnings[0], f"the line does not read as a warning: {warnings!r}"
    counts = _frame_counts(warnings[0])
    assert counts.get("frames") == 4, (
        f"the warning has to count the 4 recorded frames, got {counts!r} in {warnings[0]!r}"
    )
    assert counts.get("frames_with_motor_table") == 1, (
        f"one frame of the four was readable, got {counts!r} in {warnings[0]!r}"
    )
    # The main CSV is not part of this degradation: 14 columns, both steps.
    main_header, main_rows = _read_csv(out)
    assert len(main_header) == 14, f"the main CSV grew a column: {main_header}"
    assert len(main_rows) == 2, f"the run lost a step: {main_rows!r}"


def test_main_rtt_should_not_write_a_safety_sidecar(latency: Any, tmp_path: Path) -> None:
    """``--mode rtt`` never moves the robot, so it has no safety recording."""
    out = tmp_path / "rtt-20261025T000000Z.csv"

    code = latency.main(
        [
            "--host",
            "127.0.0.1",
            "--samples",
            "2",
            "--rate-hz",
            "200",
            "--csv",
            str(out),
        ],
        client_factory=_Factory(_SafetyFake()),
    )

    assert code == latency.EXIT_OK, f"the rtt run failed with {code}"
    assert out.is_file(), "the rtt CSV itself must still be written"
    assert not latency.safety_csv_path(out).exists(), (
        f"rtt mode wrote {latency.safety_csv_path(out)}"
    )


# ===========================================================================
# G. ``target_source`` on disk — the ninth sidecar column
# ===========================================================================


@pytest.mark.parametrize(
    ("source", "cell", "include"),
    [
        pytest.param("none", "none", True, id="none"),
        pytest.param("command", "command", True, id="command"),
        pytest.param("watchdog", "watchdog", True, id="watchdog"),
        pytest.param("protection", "protection", True, id="protection"),
        pytest.param(None, "", False, id="missing-key"),
        pytest.param("manual", "", True, id="unknown-str"),
        pytest.param(5, "", True, id="int"),
    ],
)
def test_main_tracking_should_write_the_target_source_of_each_row(
    latency: Any, tmp_path: Path, source: Any, cell: str, include: bool
) -> None:
    """The ninth column of a healthy frame's rows is the frame's source, or an empty cell.

    A frame without the key and a frame with an unusable value both leave the cell
    empty -- never a default, never the string form of the value -- while the eight
    columns beside it stay exactly what the same frame produced before the column
    existed (motor names, currents, targets and the hold flag all filled).
    """
    out = tmp_path / "tracking-target-source.csv"
    client = _SafetyFake(face=_face_with_source(source, include=include))

    code = latency.main(_main_argv(out), client_factory=_Factory(client))

    assert code == latency.EXIT_OK, f"the run failed with {code}"
    header, rows = _read_csv(latency.safety_csv_path(out))
    assert header == latency.SAFETY_CSV_FIELDS
    assert len(header) == 9, f"the sidecar is not nine columns: {header}"
    assert len(rows) == 8, f"expected 4 growing frames x 2 motors, got {len(rows)} rows"
    assert [row["target_source"] for row in rows] == [cell] * 8, (
        f"expected {cell!r} on every row, got {[row['target_source'] for row in rows]!r}"
    )
    # The other eight columns are untouched by the empty / unusable source.
    assert [row["motor"] for row in rows] == [_MOTOR, _OTHER_MOTOR] * 4
    assert all(row["current_ma"] != "" for row in rows), [row["current_ma"] for row in rows]
    assert all(row["requested_target"] != "" for row in rows)
    assert all(row["accepted_target"] != "" for row in rows)
    assert all(row["hold_active"] != "" for row in rows)


def test_main_tracking_should_write_a_nine_column_sidecar_and_a_fourteen_column_main_csv(
    latency: Any, tmp_path: Path
) -> None:
    """``target_source`` is appended after ``hold_active``; the main CSV is still frozen at 14.

    Two tables, two widths: the sidecar grew the one column it was allowed to grow,
    in last position, and the tracking table (parsed by position downstream) did
    not move at all.
    """
    out = tmp_path / "tracking-columns.csv"

    code = latency.main(_main_argv(out), client_factory=_Factory(_SafetyFake()))

    assert code == latency.EXIT_OK, f"the run failed with {code}"
    sidecar_header, sidecar_rows = _read_csv(latency.safety_csv_path(out))
    main_header, main_rows = _read_csv(out)
    assert sidecar_header == latency.SAFETY_CSV_FIELDS
    assert len(sidecar_header) == 9, f"the sidecar is not nine columns: {sidecar_header}"
    assert sidecar_header[8] == "target_source", (
        f"target_source has to be the last sidecar column: {sidecar_header}"
    )
    assert main_header == latency.TRACKING_CSV_FIELDS
    assert len(main_header) == 14, f"the main CSV grew a column: {main_header}"
    assert len(sidecar_rows) == 8
    assert len(main_rows) == 2


# ===========================================================================
# H. The poll-clock fallback is named once per run
# ===========================================================================

# Ways ``latest_request_timing`` can fail to answer an arrival stamp: an empty
# face, a face without the key, a non-finite stamp, and readers that raise.
_UNUSABLE_TIMING = [
    pytest.param({"timed": False}, id="empty-face"),
    pytest.param({"timing": {"sent_monotonic_s": 1000.0}}, id="received-missing"),
    pytest.param({"timing": {"received_monotonic_s": float("nan")}}, id="received-nan"),
    pytest.param({"timing": {"received_monotonic_s": float("inf")}}, id="received-inf"),
    pytest.param({"timing": _ExplodingTiming()}, id="readers-raise"),
]


def _fallback_notices(stderr: str) -> list[str]:
    """The stderr lines that name the poll-clock fallback of the arrival stamp.

    Matched on the three things the notice has to say -- the column it is about
    (``t_monotonic_s``), what the value really is (``poll clock``) and why
    (``latest_request_timing`` is unusable) -- instead of on one frozen sentence:
    the wording belongs to the implementation, these three do not.  A safety
    warning is deliberately NOT matched: it never names the timing face.
    """
    return [
        line
        for line in stderr.splitlines()
        if "t_monotonic_s" in line and "poll clock" in line and "latest_request_timing" in line
    ]


@pytest.mark.parametrize("timing_kwargs", _UNUSABLE_TIMING)
def test_main_tracking_should_name_the_poll_clock_fallback_exactly_once(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str], timing_kwargs: dict[str, Any]
) -> None:
    """4 unstamped frames, 1 notice: the fallback is a property of the run, not of a frame.

    Every recorded frame of this run falls back to the poll clock, so wording that
    fires per frame writes 4 lines and wording that fires per poll writes 5.  The
    operator needs to know once that ``t_monotonic_s`` is a poll instant rather
    than the client's timestamp -- and the sidecar is still written, stamped, with
    the same number of rows as a timed run.
    """
    out = tmp_path / "tracking-untimed.csv"
    client = _SafetyFake(**timing_kwargs)

    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the fallback changed the exit code: {code}"
    notices = _fallback_notices(printed.err)
    assert len(notices) == 1, (
        f"the poll-clock fallback has to be named exactly once, got {notices!r} "
        f"(stderr={printed.err!r})"
    )
    # The rows are still there and still stamped -- the fallback is a real clock
    # reading, not an empty cell.
    header, rows = _read_csv(latency.safety_csv_path(out))
    assert header == latency.SAFETY_CSV_FIELDS
    assert len(rows) == 8, f"the frames went down with the stamp: {len(rows)} rows"
    assert all(row["t_monotonic_s"] != "" for row in rows), [r["t_monotonic_s"] for r in rows]
    assert all(float(row["t_monotonic_s"]) >= 0.0 for row in rows)
    # Distinguishable from the two safety warnings: it never names that face, and
    # this otherwise-clean client produced no safety warning at all.
    assert "latest_safety_status" not in notices[0], (
        f"the fallback notice reads like a safety warning: {notices[0]!r}"
    )
    assert _lines_mentioning(printed.err, "latest_safety_status") == [], (
        f"a clean safety face warned: {printed.err!r}"
    )


def test_main_tracking_should_not_name_the_poll_clock_when_the_frame_carries_a_stamp(
    latency: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A finite ``received_monotonic_s`` on every frame: zero notices.

    Without this case a "warn always" implementation would look green on the
    fallback tests and leave an operator unable to tell a timed run from an
    untimed one.
    """
    out = tmp_path / "tracking-timed.csv"
    client = _SafetyFake()

    code = latency.main(_main_argv(out), client_factory=_Factory(client))
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"the run failed with {code}"
    notices = _fallback_notices(printed.err)
    assert notices == [], f"a timed run warned about the poll clock: {notices!r}"
    assert _lines_mentioning(printed.err, "latest_request_timing") == [], (
        f"a timed run named the timing face on stderr: {printed.err!r}"
    )


# The safety face and the timing face fail independently of one another, so a run
# can hit both at once.  The two notices are about different faces, printed by
# different branches, and both are needed on exactly that run -- folding either
# one into the other's branch silences one of them.
_DEGRADED_SAFETY_FACES = [
    pytest.param(lambda: _NO_FACE, id="face-absent"),
    pytest.param(
        lambda: {"currents_ma": _CorruptCurrents(), "joint_holds": [_MOTOR]},
        id="currents-corrupt",
    ),
    pytest.param(lambda: {"currents_ma": {}, "joint_holds": []}, id="currents-empty"),
]

_UNUSABLE_TIMING_FACES = [
    pytest.param({"timed": False}, id="timing-empty"),
    pytest.param({"timing": _ExplodingTiming()}, id="timing-readers-raise"),
]


@pytest.mark.parametrize("safe_face", _DEGRADED_SAFETY_FACES)
@pytest.mark.parametrize("timing_kwargs", _UNUSABLE_TIMING_FACES)
def test_main_tracking_should_warn_about_both_faces_when_both_degrade_together(
    latency: Any,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    safe_face: Any,
    timing_kwargs: dict[str, Any],
) -> None:
    """Both faces down at once: two warnings, one per face, neither suppressing the other.

    The safety warning is about what the sidecar cannot record; the poll-clock
    notice is about what its ``t_monotonic_s`` column really is.  An operator
    hitting a degraded host usually gets both, so the pair is the normal bad case,
    not an exotic one -- and it is the case a shared ``if``/``elif`` chain loses.
    Each face is therefore counted on its own and then compared to the run where
    only the safety face is down: that run has to name the safety face and say
    nothing about the poll clock, which is what makes the second notice here the
    timing face's own line rather than a second wording of the first.

    The two identifying substrings are exclusive in the implementation, which is
    what makes the two counts independent: the fallback notice names
    ``t_monotonic_s`` / ``poll clock`` / ``latest_request_timing`` and never
    ``latest_safety_status``, while the safety warning names
    ``latest_safety_status`` and none of the three timing markers.  The assertions
    on the lines themselves keep that exclusivity pinned: if the wording ever
    starts to overlap, these two counts stop being two independent measurements
    and this test says so instead of counting one line twice.
    """
    single_out = tmp_path / "tracking-safety-only.csv"
    both_out = tmp_path / "tracking-both-degraded.csv"

    single_code = latency.main(
        _main_argv(single_out), client_factory=_Factory(_SafetyFake(face=safe_face()))
    )
    single = capsys.readouterr()
    code = latency.main(
        _main_argv(both_out),
        client_factory=_Factory(_SafetyFake(face=safe_face(), **timing_kwargs)),
    )
    printed = capsys.readouterr()

    assert code == latency.EXIT_OK, f"two degraded faces must not change the exit code: {code}"
    assert code == single_code, (
        f"the timing degradation changed the exit code: {single_code} -> {code}"
    )

    # Baseline: the same safety degradation with a healthy timing face.  It names
    # the safety face once and stays silent about the poll clock.
    assert len(_lines_mentioning(single.err, "latest_safety_status")) == 1, (
        f"the safety-only run has to name the face exactly once: {single.err!r}"
    )
    assert _fallback_notices(single.err) == [], (
        f"the safety face must not trigger the poll-clock notice: {_fallback_notices(single.err)!r}"
    )

    safety_lines = _lines_mentioning(printed.err, "latest_safety_status")
    notices = _fallback_notices(printed.err)
    assert len(safety_lines) == 1, (
        f"the safety face has to be named exactly once, got {safety_lines!r} "
        f"(stderr={printed.err!r})"
    )
    assert len(notices) == 1, (
        f"the poll-clock fallback has to be named exactly once, got {notices!r} "
        f"(stderr={printed.err!r})"
    )

    # Two different lines, and the markers really are exclusive.
    assert "latest_safety_status" not in notices[0], (
        f"the fallback notice reads like a safety warning: {notices[0]!r}"
    )
    overlapping = [
        marker
        for marker in ("poll clock", "latest_request_timing", "t_monotonic_s")
        if marker in safety_lines[0]
    ]
    assert overlapping == [], (
        f"the safety warning picked up the timing markers {overlapping!r}: {safety_lines[0]!r} "
        "-- the two notice counts would no longer be independent measurements"
    )
    assert notices[0] not in safety_lines[0], (
        f"the two warnings collapsed into one line: {notices[0]!r}"
    )
    assert safety_lines[0] not in notices[0], (
        f"the two warnings collapsed into one line: {safety_lines[0]!r}"
    )
    stderr_lines = printed.err.splitlines()
    assert stderr_lines.index(safety_lines[0]) != stderr_lines.index(notices[0]), (
        f"both notices landed on the same stderr line: {stderr_lines!r}"
    )

    # Neither degradation costs a row: the sidecar is the same file the
    # safety-only run wrote (one placeholder row per recorded frame, still
    # stamped), and the main CSV still carries its two steps.
    header, rows = _read_csv(latency.safety_csv_path(both_out))
    single_header, single_rows = _read_csv(latency.safety_csv_path(single_out))
    assert header == single_header == latency.SAFETY_CSV_FIELDS
    assert len(rows) == len(single_rows) == 4, (
        f"one placeholder row per recorded frame (4), got {len(rows)} vs {len(single_rows)}"
    )
    assert [row["motor"] for row in rows] == [""] * 4, [row["motor"] for row in rows]
    assert all(row["t_monotonic_s"] != "" for row in rows), (
        f"the fallback is a clock reading, not an empty cell: "
        f"{[r['t_monotonic_s'] for r in rows]!r}"
    )
    main_header, main_rows = _read_csv(both_out)
    single_main_header, single_main_rows = _read_csv(single_out)
    assert main_header == single_main_header
    assert len(main_rows) == len(single_main_rows) == 2, (
        f"the run lost a step: {len(main_rows)} vs {len(single_main_rows)}"
    )
