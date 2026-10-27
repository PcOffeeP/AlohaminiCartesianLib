"""Offline contract tests for ``benchmarks/alohamini_io/capture_robot_fingerprint.py``.

The script is loaded **by path** (``importlib.util.spec_from_file_location``)
because it is not a package module; ``_load_script`` does that and the
module-scope ``fingerprint`` fixture hands the module to every test.  Nothing
here needs a robot: the only handle is the scriptable ``_FakeClient`` below, so
no socket is opened, no port is bound, no ``lerobot`` hardware import happens and
no test waits on real time.  Every artefact goes to ``tmp_path``.

What the tests below lock down (the banners group them by topic):

* **the read-only guarantee** — ``--help`` advertises it, the happy path prints
  the read-only banner, the fake client counts zero action packets and the module
  source never mentions the action method at all;
* **the payload is the fingerprint** — ``build_payload`` keeps the host metadata
  verbatim (``schema_version``, ``robot_model`` and all eight per-motor fields),
  and ``main`` writes a JSON file that ``json.loads`` reproduces; a trimmed or
  ``NaN``-carrying payload is not a fingerprint;
* **sampling** — one sample per ``observation_sequence`` growth, ``InsufficientSamples``
  instead of a silently thinner capture, and cameras never pulled
  (``include_cameras=False`` on every call);
* **metadata validation** — empty metadata, a ``schema_version`` that is not
  ``2`` and a payload without a ``motors`` mapping are refused with
  ``EXIT_INVALID_METADATA`` before anything is written;
* **exit-code layering** — 0 ok / 3 connect / 4 insufficient samples / 5 write /
  6 invalid metadata, each reachable through ``main`` without a traceback;
* **write failures** — an unwritable ``--out`` and an unserialisable value both
  map to ``EXIT_WRITE_FAILED`` with ``write`` on stderr;
* **CLI validation** — ``--samples < 1`` and a non-finite or non-positive
  ``--timeout-s`` are usage errors (2) from ``parse_args``, not a failed
  connection (3) and not a sample-starvation exit (4).

Regression intent (each entry is a plausible-but-wrong implementation that
must turn this file red):

* **payload truncation** — writing a trimmed metadata subset (forgetting
  ``schema_version``, ``robot_model`` or one of the eight per-motor fields)
  must break the equality assertions in the ``main`` success test;
* **zero-motion breach** — any ``send_action`` reference in the module source
  fails the source scan, and the fake client's call counter must stay at 0;
* **exit-code collapse** — connect failure / sample starvation / write failure
  must stay distinguishable (3 / 4 / 5), never one generic failure code;
* **silent short capture** — collecting fewer samples than requested must raise
  instead of writing a thinner payload;
* **cameras pulled anyway** — ``get_observation`` must be called with
  ``include_cameras=False``.

.. warning::
   The source scan is a raw ``"send_action" in source`` substring check, so the
   benchmark module must not mention that name even in its docstring or
   comments.

Round-2 contracts (still the frozen expectation; the tests below pin exactly
this behaviour):

* ``EXIT_INVALID_METADATA == 6``.  The host metadata face is validated before
  anything is written: an empty dict, a ``schema_version`` that is not ``2`` or
  a payload without a ``motors`` mapping is a refusal (stderr mentions
  ``metadata``, no success wording on stdout).  Storing an invalid fingerprint
  as if it succeeded is the failure mode being locked out.
* ``write_json`` serialises with ``allow_nan=False`` and reports a
  ``TypeError``/``ValueError`` (unserialisable value, ``NaN``/``Infinity``) as
  ``EXIT_WRITE_FAILED`` with ``write`` on stderr, instead of letting the
  exception escape ``main``.  A JSON file that ``json.loads`` cannot reproduce
  is not a fingerprint.

Round-3 contract (a reviewer round; the test below pins exactly this):
``--samples < 1`` is a usage error and is rejected by ``parse_args`` with
``SystemExit(2)``, never by running the capture loop and reporting
``EXIT_CONNECT_FAILED == 3``.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPT_PATH = _REPO_ROOT / "benchmarks" / "alohamini_io" / "capture_robot_fingerprint.py"

# The eight per-motor metadata columns ``render_table`` must expose and the
# fingerprint payload must preserve verbatim.
MOTOR_FIELDS = (
    "id",
    "model",
    "normalization",
    "drive_mode",
    "range_min",
    "range_max",
    "homing_offset",
    "resolution",
)

_READ_ONLY_BANNER = "read-only: no action sent"


def _load_script():
    """Load the benchmark script by path (it is not a package module)."""
    assert _SCRIPT_PATH.is_file(), (
        f"script under test: {_SCRIPT_PATH} (the contract below is frozen; "
        "a missing script fails loudly instead of skipping)"
    )
    spec = importlib.util.spec_from_file_location("bench_capture_robot_fingerprint", _SCRIPT_PATH)
    assert spec is not None and spec.loader is not None, (
        f"cannot build import spec for {_SCRIPT_PATH}"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def fingerprint():
    """The module under test: the script loaded by path, imported once per run."""
    return _load_script()


# ===========================================================================
# Harness: scriptable fake client + deterministic clock (no sockets, no sleeps)
# ===========================================================================


class _FakeClock:
    """Monotonic clock stub: every read advances by ``step_s``."""

    def __init__(self, *, start: float = 0.0, step_s: float = 1.0) -> None:
        self._now = start
        self._step_s = step_s
        self.calls = 0

    def __call__(self) -> float:
        self.calls += 1
        current = self._now
        self._now += self._step_s
        return current


def _fake_metadata() -> dict:
    """Two-motor v2 metadata: every field differs between motors."""
    return {
        "schema_version": 2,
        "robot_model": "alohamini2pro",
        "motors": {
            "arm_left_shoulder_pan": {
                "id": 1,
                "model": "sts3215",
                "normalization": "range_m100_100",
                "drive_mode": 0,
                "range_min": 0,
                "range_max": 4095,
                "homing_offset": -2048,
                "resolution": 4096,
            },
            "arm_right_elbow_flex": {
                "id": 7,
                "model": "sts3215",
                "normalization": "degrees",
                "drive_mode": 1,
                "range_min": 32,
                "range_max": 4000,
                "homing_offset": 120,
                "resolution": 4096,
            },
        },
    }


def _metadata_variants() -> dict[str, dict]:
    """Shapes of host metadata that must NOT be stored as a valid fingerprint."""
    empty: dict = {}
    wrong_schema = _fake_metadata()
    wrong_schema["schema_version"] = 1
    no_motors = _fake_metadata()
    del no_motors["motors"]
    return {"empty": empty, "wrong_schema_version": wrong_schema, "no_motors": no_motors}


_METADATA_VARIANTS = _metadata_variants()


def _fake_control_status() -> dict:
    return {
        "schema_version": 1,
        "host_session_id": "fp-session",
        "sample_sequence": 5,
        "settled": True,
        "valid": True,
        "joint_holds": {},
        "gripper_holds": {},
    }


class _FakeClient:
    """Stand-in for ``AlohaMiniClient`` that never touches a socket."""

    def __init__(
        self,
        *,
        sequences: list[int] | None = None,
        ages: list[float] | None = None,
        connect_error: Exception | None = None,
    ) -> None:
        self._sequences = list(sequences) if sequences is not None else None
        self._ages = list(ages) if ages is not None else None
        self._connect_error = connect_error
        self._sequence = 0
        self._calls = 0
        self.is_connected = False
        self.send_action_calls = 0
        self.get_observation_calls = 0
        self.include_cameras_values: list[bool] = []
        # Seeded so that a ``main`` that reads metadata before the first
        # observation still sees the full fingerprint.
        self.latest_robot_metadata: dict = _fake_metadata()
        self.latest_control_status: dict | None = _fake_control_status()
        self.latest_request_timing: dict = {}
        self.latest_host_timing: dict = {}
        self.last_state: dict = {"arm_left_shoulder_pan.pos": 1.0}

    @property
    def observation_sequence(self) -> int:
        return self._sequence

    def connect(self) -> None:
        if self._connect_error is not None:
            raise self._connect_error
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def get_observation(self, include_cameras: bool = True) -> dict:
        index = self._calls
        self._calls += 1
        self.get_observation_calls += 1
        self.include_cameras_values.append(include_cameras)
        if self._sequences is None:
            self._sequence += 1
        else:
            self._sequence = self._sequences[min(index, len(self._sequences) - 1)]
        age_s = self._ages[min(index, len(self._ages) - 1)] if self._ages else 0.01 * (index + 1)
        sent_s = 1000.0 + index
        self.latest_request_timing = {
            "token": f"tok-{index}".encode("ascii"),
            "sent_monotonic_s": sent_s,
            "received_monotonic_s": sent_s + age_s,
            "age_s": age_s,
        }
        self.latest_host_timing = {
            "state_sample_started_monotonic_s": 900000.0 + index,
            "state_sample_finished_monotonic_s": 900000.0 + index + 0.004,
            "state_sample_unix_ns": 0,
        }
        return dict(self.last_state)

    def send_action(self, action: dict) -> dict:
        self.send_action_calls += 1
        return dict(action)


class _Factory:
    """Injectable ``client_factory`` that records calls and can fail."""

    def __init__(self, client: Any = None, *, raises: Exception | None = None) -> None:
        self.client = client
        self.raises = raises
        self.calls: list[tuple[Any, ...]] = []

    def __call__(self, host: str, port: int, robot_model: str):
        self.calls.append((host, port, robot_model))
        if self.raises is not None:
            raise self.raises
        return self.client


def _fast_sleep(_seconds: float) -> None:
    """Sleep stub: keeps timeout logic driven by the injected clock only."""


# ===========================================================================
# CLI surface
# ===========================================================================


def test_help_should_exit_zero_and_advertise_read_only() -> None:
    """``--help`` must succeed and state that the script is read-only."""
    result = subprocess.run(
        [sys.executable, str(_SCRIPT_PATH), "--help"],
        capture_output=True,
        text=True,
        check=False,
        cwd=str(_REPO_ROOT),
        timeout=120,
    )

    combined = (result.stdout + result.stderr).lower()

    assert result.returncode == 0, result.stderr
    assert "read-only" in combined


def test_parse_args_should_require_host(fingerprint) -> None:
    """``--host`` is mandatory: argparse exits with status 2 on an empty argv."""
    with pytest.raises(SystemExit) as excinfo:
        fingerprint.parse_args([])

    assert excinfo.value.code == 2


def test_parse_args_should_apply_contract_defaults(fingerprint) -> None:
    """Defaults: port 5556, alohamini2pro, 5 samples, 30 s, timestamped out."""
    args = fingerprint.parse_args(["--host", "127.0.0.1"])

    assert args.host == "127.0.0.1"
    assert args.port == 5556
    assert args.robot_model == "alohamini2pro"
    assert args.samples == 5
    assert args.timeout_s == pytest.approx(30.0)
    out = str(args.out)
    assert "artifacts/alohamini-io/fingerprint-" in out
    assert out.endswith(".json")


def test_exit_code_constants_should_match_contract(fingerprint) -> None:
    """The four exit codes are a frozen part of the CLI contract."""
    assert fingerprint.EXIT_OK == 0
    assert fingerprint.EXIT_CONNECT_FAILED == 3
    assert fingerprint.EXIT_INSUFFICIENT_SAMPLES == 4
    assert fingerprint.EXIT_WRITE_FAILED == 5


def test_exit_code_constants_should_include_invalid_metadata(fingerprint) -> None:
    """A host that broadcasts no usable metadata is its own failure mode."""
    assert getattr(fingerprint, "EXIT_INVALID_METADATA", None) == 6


# ===========================================================================
# Zero-motion hard constraint
# ===========================================================================


def test_module_source_should_not_mention_send_action(fingerprint) -> None:
    """The fingerprint script is read-only: the name must not appear at all."""
    assert "send_action" not in inspect.getsource(fingerprint)


# ===========================================================================
# Sampling
# ===========================================================================


def test_collect_samples_should_record_one_sample_per_sequence_growth(fingerprint) -> None:
    """A sample is recorded only when ``observation_sequence`` grows."""
    client = _FakeClient(sequences=[1, 2, 3, 3], ages=[0.010, 0.020, 0.030, 0.040])

    samples = fingerprint.collect_samples(
        client,
        samples=3,
        timeout_s=5.0,
        sleep=_fast_sleep,
        clock=lambda: 0.0,
    )

    assert len(samples) == 3
    assert [sample["observation_sequence"] for sample in samples] == [1, 2, 3]
    assert [sample["age_s"] for sample in samples] == pytest.approx([0.010, 0.020, 0.030])
    for sample in samples:
        assert {"observation_sequence", "age_s", "sent_monotonic_s", "received_monotonic_s"} <= set(
            sample
        )


def test_collect_samples_should_raise_when_sequence_never_grows(fingerprint) -> None:
    """A frozen sequence must end in ``InsufficientSamples``, never fewer rows."""
    client = _FakeClient(sequences=[1])
    clock = _FakeClock(start=0.0, step_s=5.0)

    with pytest.raises(fingerprint.InsufficientSamples) as excinfo:
        fingerprint.collect_samples(
            client, samples=3, timeout_s=1.0, sleep=_fast_sleep, clock=clock
        )

    assert excinfo.value.required == 3
    assert excinfo.value.collected < 3


# ===========================================================================
# Rendering + payload
# ===========================================================================


def test_render_table_should_list_all_eight_fields_and_every_motor(fingerprint) -> None:
    """The human-readable table carries the eight columns and one row per motor."""
    metadata = _fake_metadata()

    table = fingerprint.render_table(metadata)

    for field in MOTOR_FIELDS:
        assert field in table
    for motor in metadata["motors"]:
        assert motor in table


def test_build_payload_should_keep_metadata_verbatim(fingerprint) -> None:
    """Metadata is copied through unchanged; a missing control status is allowed."""
    metadata = _fake_metadata()

    payload = fingerprint.build_payload(
        metadata=metadata,
        control_status=None,
        observation_state={"arm_left_shoulder_pan.pos": 1.0},
        samples=3,
        host="127.0.0.1",
        robot_model="alohamini2pro",
    )

    assert payload["robot_metadata"] == metadata
    assert payload["control_status"] is None
    assert payload["observation_state"] == {"arm_left_shoulder_pan.pos": 1.0}
    assert payload["sample_count"] == 3
    assert payload["host"] == "127.0.0.1"
    assert payload["requested_robot_model"] == "alohamini2pro"
    assert payload["read_only"] is True
    assert "schema_version" in payload
    assert "captured_utc" in payload


def test_write_json_should_create_parent_dirs_and_keep_unicode(fingerprint, tmp_path) -> None:
    """``write_json`` mkdirs the parents, sorts keys and keeps UTF-8 readable."""
    target = tmp_path / "nested" / "deeper" / "fingerprint.json"

    fingerprint.write_json(target, {"zeta": 1, "alpha": "中文"})

    text = target.read_text(encoding="utf-8")
    assert json.loads(text) == {"zeta": 1, "alpha": "中文"}
    assert "中文" in text
    assert text.index('"alpha"') < text.index('"zeta"')


# ===========================================================================
# main(): happy path and failure exit codes
# ===========================================================================


def test_main_should_write_full_fingerprint_and_never_send_action(
    fingerprint, tmp_path, capsys
) -> None:
    """Happy path: exit 0, read-only banner, untrimmed metadata, zero motion."""
    client = _FakeClient(sequences=[1, 2, 3], ages=[0.010, 0.020, 0.030])
    factory = _Factory(client)
    out = tmp_path / "artifacts" / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
        client_factory=factory,
    )
    captured = capsys.readouterr()

    assert code == fingerprint.EXIT_OK == 0
    assert _READ_ONLY_BANNER in captured.out
    payload = json.loads(out.read_text(encoding="utf-8"))
    expected = _fake_metadata()
    assert payload["robot_metadata"] == expected
    assert payload["robot_metadata"]["schema_version"] == 2
    assert payload["robot_metadata"]["robot_model"] == "alohamini2pro"
    for motor, entry in expected["motors"].items():
        for field in MOTOR_FIELDS:
            assert payload["robot_metadata"]["motors"][motor][field] == entry[field]
    assert payload["sample_count"] == 3
    assert payload["read_only"] is True
    assert payload["requested_robot_model"] == "alohamini2pro"
    assert payload["host"] == "127.0.0.1"
    assert client.send_action_calls == 0
    assert client.include_cameras_values
    assert all(value is False for value in client.include_cameras_values)
    assert factory.calls == [("127.0.0.1", 5556, "alohamini2pro")]


def test_main_should_report_connect_failed_when_factory_raises(
    fingerprint, tmp_path, capsys
) -> None:
    """A factory that blows up maps to EXIT_CONNECT_FAILED and writes nothing."""
    factory = _Factory(raises=RuntimeError("no host"))
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(["--host", "127.0.0.1", "--out", str(out)], client_factory=factory)
    captured = capsys.readouterr()

    assert code == fingerprint.EXIT_CONNECT_FAILED
    assert "connect" in captured.err.lower()
    assert not out.exists()


def test_main_should_report_connect_failed_when_client_connect_raises(
    fingerprint, tmp_path
) -> None:
    """A raising ``client.connect()`` is a connect failure too, not a crash."""
    client = _FakeClient(connect_error=OSError("refused"))
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out)], client_factory=_Factory(client)
    )

    assert code == fingerprint.EXIT_CONNECT_FAILED
    assert not out.exists()


def test_main_should_report_insufficient_samples(fingerprint, tmp_path, capsys) -> None:
    """A frozen observation sequence ends in EXIT_INSUFFICIENT_SAMPLES."""
    client = _FakeClient(sequences=[1])
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        [
            "--host",
            "127.0.0.1",
            "--out",
            str(out),
            "--samples",
            "3",
            "--timeout-s",
            "0.05",
        ],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == fingerprint.EXIT_INSUFFICIENT_SAMPLES
    assert "sample" in captured.err.lower()
    assert not out.exists()


def test_main_should_report_write_failed_when_out_is_not_writable(
    fingerprint, tmp_path, capsys
) -> None:
    """An unwritable ``--out`` maps to EXIT_WRITE_FAILED instead of raising."""
    blocker = tmp_path / "blocker"
    blocker.write_text("not a directory", encoding="utf-8")
    out = blocker / "nested" / "fingerprint.json"
    client = _FakeClient(sequences=[1, 2, 3])

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3"],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == fingerprint.EXIT_WRITE_FAILED
    assert "write" in captured.err.lower()


def test_main_should_report_write_failed_for_unserialisable_metadata(
    fingerprint, tmp_path, capsys
) -> None:
    """A ``bytes`` field cannot be stored, so the run fails as a write failure.

    ``bytes`` is not JSON: encoding it verbatim would either raise out of
    ``main`` or produce a file no consumer can load, so the payload must be
    rejected instead of half-written.
    """
    client = _FakeClient(sequences=[1, 2, 3])
    client.latest_robot_metadata["motors"]["arm_left_shoulder_pan"]["model"] = b"sts3215"
    out = tmp_path / "fingerprint.json"

    try:
        code = fingerprint.main(
            ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
            client_factory=_Factory(client),
        )
    except (TypeError, ValueError) as exc:  # the defect being locked out
        pytest.fail(f"main() let the serialisation error escape instead of mapping it: {exc!r}")
    captured = capsys.readouterr()

    assert code == fingerprint.EXIT_WRITE_FAILED
    assert "write" in captured.err.lower()


def test_main_should_never_write_non_finite_json_literals(fingerprint, tmp_path) -> None:
    """``NaN`` in the metadata is a write failure, never a ``NaN`` literal on disk."""
    client = _FakeClient(sequences=[1, 2, 3])
    client.latest_robot_metadata["motors"]["arm_left_shoulder_pan"]["homing_offset"] = float("nan")
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
        client_factory=_Factory(client),
    )

    assert code == fingerprint.EXIT_WRITE_FAILED
    if out.exists():
        text = out.read_text(encoding="utf-8")
        assert "NaN" not in text
        assert "Infinity" not in text


@pytest.mark.parametrize(
    "metadata",
    list(_METADATA_VARIANTS.values()),
    ids=list(_METADATA_VARIANTS),
)
def test_main_should_reject_invalid_metadata(fingerprint, tmp_path, capsys, metadata) -> None:
    """Empty / wrong-schema / motor-less metadata is refused, not stored as success."""
    client = _FakeClient(sequences=[1, 2, 3])
    client.latest_robot_metadata = metadata
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    expected = getattr(fingerprint, "EXIT_INVALID_METADATA", None)
    assert expected == 6, "EXIT_INVALID_METADATA must exist and be 6"
    assert code == expected
    assert "metadata" in captured.err.lower()
    assert "payload written" not in captured.out.lower()


# ===========================================================================
# Round 3: CLI range validation (argparse level, not the connect-failure path)
# ===========================================================================


@pytest.mark.parametrize("argv", [["--samples=0"], ["--samples=-1"]], ids=["zero", "negative"])
def test_parse_args_should_reject_a_non_positive_sample_count(fingerprint, argv: list[str]) -> None:
    """``--samples < 1`` is a usage error (2), not a failed connection (3)."""
    with pytest.raises(SystemExit) as excinfo:
        fingerprint.parse_args(["--host", "127.0.0.1", *argv])

    assert excinfo.value.code == 2


@pytest.mark.parametrize(
    "timeout_s",
    ["nan", "inf", "1e400", "0", "-1"],
    ids=["nan", "inf", "overflow_to_inf", "zero", "negative"],
)
def test_parse_args_should_reject_a_nonsense_timeout_s(fingerprint, capsys, timeout_s: str) -> None:
    """``--timeout-s`` is validated like ``measure_latency.py``'s, before any socket.

    ``--timeout-s`` is the *only* stop condition of a capture, and it was not
    validated at all, so all five values parsed cleanly and reached
    ``collect_samples`` (``deadline_s = clock() + timeout_s``):

    * ``inf`` (and ``1e400``, which ``float()`` turns into ``inf``) never expires;
    * ``nan`` never expires *and* spins: ``now_s >= deadline_s`` is false forever
      and ``pause_s = min(POLL_INTERVAL_S, max(0.0, deadline_s - now_s))`` is
      ``0.0`` because ``max(0.0, nan) == 0.0``, so the poll pause is skipped
      entirely;
    * ``0`` / ``-1`` expire the budget before the first frame and were reported as
      ``InsufficientSamples`` (exit 4) for a budget the operator typed wrong.

    Ruling: all five are now a usage error (2), the shape the rtt benchmark uses
    and the same ``not (math.isfinite(x) and x > 0.0)`` test.  The exit-code table
    does not move -- ``4`` keeps its meaning (the sequence froze / not enough fresh
    samples within a *legal* budget) and simply stops being reachable from a
    nonsense budget.

    Asserted: exit code 2 and ``--timeout-s`` on stderr, and the legal budget is
    parsed first on purpose -- an unknown option also exits 2, so without that
    guard this test would go green on an option that no longer exists.
    """
    # Arrange: the option exists and a legal value reaches the namespace.
    legal = fingerprint.parse_args(["--host", "127.0.0.1", "--timeout-s", "5"])
    assert getattr(legal, "timeout_s", None) == pytest.approx(5.0), (
        "--timeout-s 5 does not parse, so the rejection below proves nothing"
    )
    capsys.readouterr()

    # Act + Assert: the nonsense budget is refused by the parser itself.
    with pytest.raises(SystemExit) as excinfo:
        fingerprint.parse_args(["--host", "127.0.0.1", f"--timeout-s={timeout_s}"])

    assert excinfo.value.code == 2, (
        f"--timeout-s={timeout_s} exited with {excinfo.value.code} instead of the usage "
        "error 2: an unvalidated budget either never expires (nan/inf, and nan also "
        "skips the poll pause) or is already over before the first frame (0/-1)"
    )
    message = capsys.readouterr().err.lower()
    assert "timeout-s" in message, f"the error does not name the option: {message!r}"


def test_parse_args_should_keep_a_legal_timeout_s(fingerprint) -> None:
    """The validated budget still parses and keeps the value the operator typed."""
    args = fingerprint.parse_args(["--host", "127.0.0.1", "--timeout-s", "5"])

    assert args.timeout_s == pytest.approx(5.0)


# ===========================================================================
# Round 4: the client capability probe (a vendor without the timing faces)
# ===========================================================================
#
# The deployed vendor client (``AlohaMiniClient`` on ``main``) has neither
# ``latest_request_timing`` nor ``latest_control_status``; it publishes
# ``latest_safety_status``, a semantically and structurally different dict.  The
# capture read both faces directly, so on that client it died with
# ``AttributeError`` mid-capture.
#
# Ruling: probe the capabilities explicitly and record an **explicit ``None``**
# for what the client cannot measure.  Never fall back — not to a made-up empty
# timing dict (which would print ``age_ms`` 0.000), not to
# ``latest_safety_status`` (a different frame under a different name).  The
# payload keeps its ten top-level keys and ``PAYLOAD_SCHEMA_VERSION == 1``, and
# the exit-code semantics (0 / 3 / 4 / 5 / 6) do not move.


class _ClientWithoutTimingFaces(_FakeClient):
    """The vendor client as it ships today: neither timing face exists.

    ``_FakeClient`` sets both attributes in ``__init__`` and re-creates the
    request timing on every ``get_observation``, which is exactly what hid the
    gap.  This subclass removes them the way the deployed client never has them.

    ``latest_safety_status`` is present on purpose: that is the attribute the
    deployed client *does* publish, and it is a different frame (settle/valid
    flags), so it must never be substituted for ``latest_control_status``.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        del self.latest_request_timing
        del self.latest_control_status
        self.latest_safety_status: dict = {
            "schema_version": 1,
            "host_session_id": "safety-session",
            "sample_sequence": 7,
            "settled": True,
            "valid": True,
        }

    def get_observation(self, include_cameras: bool = True) -> dict:
        state = super().get_observation(include_cameras=include_cameras)
        # ``_FakeClient.get_observation`` re-creates the request timing dict.
        del self.latest_request_timing
        return state


# The frozen payload surface: ten top-level keys, no new one for a missing face.
_PAYLOAD_TOP_LEVEL_KEYS = frozenset(
    {
        "schema_version",
        "captured_utc",
        "host",
        "requested_robot_model",
        "read_only",
        "sample_count",
        "samples",
        "robot_metadata",
        "control_status",
        "observation_state",
    }
)


def test_client_capabilities_should_declare_both_timing_faces(fingerprint) -> None:
    """R1: the probe names both faces, request timing first."""
    assert fingerprint.CLIENT_CAPABILITIES == (
        "latest_request_timing",
        "latest_control_status",
    )


def test_missing_client_capabilities_should_be_empty_when_both_faces_exist(
    fingerprint,
) -> None:
    """R2: a client that publishes both faces is complete."""
    client = _FakeClient()

    assert fingerprint.missing_client_capabilities(client) == []


def test_missing_client_capabilities_should_name_only_the_absent_face(fingerprint) -> None:
    """R3: one absent face is reported alone, not as "everything is missing"."""
    client = _FakeClient()
    del client.latest_control_status

    assert fingerprint.missing_client_capabilities(client) == ["latest_control_status"]


def test_missing_client_capabilities_should_list_both_faces_in_capability_order(
    fingerprint,
) -> None:
    """R4: the vendor client misses both, reported in ``CLIENT_CAPABILITIES`` order."""
    client = _ClientWithoutTimingFaces()

    assert fingerprint.missing_client_capabilities(client) == [
        "latest_request_timing",
        "latest_control_status",
    ]


def test_main_should_record_null_timings_when_both_faces_are_absent(
    fingerprint, tmp_path, capsys
) -> None:
    """R5: a capture from the vendor client succeeds with explicit ``None`` rows.

    No exception, exit ``0``, exactly one warning on stderr naming the attribute,
    and every value the client cannot measure stored as ``None`` — a JSON
    ``null``, which is visibly "not measured" instead of a plausible ``0``.

    "Exactly one" is the contract, not "at least one": a warning printed once per
    sample turns stderr into noise the operator learns to scroll past.
    """
    client = _ClientWithoutTimingFaces(sequences=[1, 2, 3])
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == fingerprint.EXIT_OK == 0, (
        f"a capture without the timing faces must still succeed, got {code}: {captured.err!r}"
    )
    assert "latest_request_timing" in captured.err, (
        f"stderr does not warn about the missing attribute: {captured.err!r}"
    )
    warning_count = captured.err.count("latest_request_timing")
    assert warning_count == 1, (
        "the missing-attribute warning must be emitted once per capture, not once per "
        f"sample; stderr named it {warning_count} time(s): {captured.err!r}"
    )
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert len(payload["samples"]) == 3
    for sample in payload["samples"]:
        assert sample["age_s"] is None, f"age_s was invented: {sample}"
        assert sample["sent_monotonic_s"] is None, f"sent stamp was invented: {sample}"
        assert sample["received_monotonic_s"] is None, f"received stamp was invented: {sample}"
    assert payload["control_status"] is None, (
        "control_status was invented instead of recorded as unknown "
        f"(latest_safety_status must not stand in for it): {payload['control_status']!r}"
    )


def test_main_should_keep_the_payload_surface_when_both_faces_are_absent(
    fingerprint, tmp_path
) -> None:
    """R6: the ten top-level keys and ``schema_version == 1`` do not move.

    A missing capability must not change the format of the fingerprint: a
    consumer that reads the payload of a supported client reads this one too.
    """
    client = _ClientWithoutTimingFaces(sequences=[1, 2, 3])
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
        client_factory=_Factory(client),
    )

    assert code == fingerprint.EXIT_OK
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert set(payload) == _PAYLOAD_TOP_LEVEL_KEYS, (
        f"payload keys changed: extra={sorted(set(payload) - _PAYLOAD_TOP_LEVEL_KEYS)} "
        f"missing={sorted(_PAYLOAD_TOP_LEVEL_KEYS - set(payload))}"
    )
    assert payload["schema_version"] == fingerprint.PAYLOAD_SCHEMA_VERSION == 1
    assert payload["robot_metadata"] == _fake_metadata()


def test_main_should_still_refuse_invalid_metadata_without_the_timing_faces(
    fingerprint, tmp_path, capsys
) -> None:
    """R7: the exit-code semantics do not depend on the capability probe."""
    client = _ClientWithoutTimingFaces(sequences=[1, 2, 3])
    client.latest_robot_metadata = {}
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
        client_factory=_Factory(client),
    )
    captured = capsys.readouterr()

    assert code == fingerprint.EXIT_INVALID_METADATA == 6
    assert "metadata" in captured.err.lower()
    assert not out.exists()


def test_main_should_keep_the_measured_timings_when_both_faces_exist(fingerprint, tmp_path) -> None:
    """R8: the supported path is unchanged — numbers in, numbers out."""
    status = _fake_control_status()
    client = _FakeClient(sequences=[1, 2, 3], ages=[0.010, 0.020, 0.030])
    client.latest_control_status = status
    out = tmp_path / "fingerprint.json"

    code = fingerprint.main(
        ["--host", "127.0.0.1", "--out", str(out), "--samples", "3", "--timeout-s", "5"],
        client_factory=_Factory(client),
    )

    assert code == fingerprint.EXIT_OK
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert [sample["age_s"] for sample in payload["samples"]] == pytest.approx(
        [0.010, 0.020, 0.030]
    )
    assert all(sample["sent_monotonic_s"] is not None for sample in payload["samples"])
    assert payload["control_status"] == status
