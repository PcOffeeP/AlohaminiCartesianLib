#!/usr/bin/env python
"""Read-only capture of an AlohaMini I/O fingerprint.

One run collects, into a single JSON payload:

* ``robot_metadata`` — the host-side v2 metadata exactly as broadcast
  (``schema_version``, ``robot_model`` and the eight per-motor fields ``id``,
  ``model``, ``normalization``, ``drive_mode``, ``range_min``, ``range_max``,
  ``homing_offset``, ``resolution``), so a later run can prove the hardware and
  calibration never changed;
* ``control_status`` — the host control-status frame of the last observation,
  if the host sent one;
* ``observation_state`` — one state-only observation of the connected robot;
* ``samples`` — one row per accepted observation request (remote sequence
  number plus the client-measured request age), useful to sanity-check the link
  before a latency run.

The script is strictly read-only: it only asks the host for observations, and
no motion command is ever constructed.  ``--help`` repeats that in the banner.

``connect()`` prefills the client's in-flight request window with
``include_cameras=True``, so the run re-primes that window state-only right
after connecting (``prime_observation_request_window``, when the client offers
it).  Otherwise the first responses would carry camera frames, whose JPEG
encoding work is not part of the state-only request age this capture reports.

The host metadata is validated before anything is written: ``schema_version``
must be ``2`` and ``motors`` must be a non-empty mapping, otherwise the run is
refused (exit 6) instead of archiving an empty or foreign-format fingerprint as
if it had succeeded.  The payload is serialised with ``allow_nan=False``, so a
value JSON cannot hold (``NaN``, ``Infinity``, ``bytes``) fails the run (exit 5)
instead of producing a file that cannot be loaded back.

What the client can measure is probed, not assumed: ``latest_request_timing``
(the per-request age) and ``latest_control_status``.  A client that publishes
neither -- the deployed vendor client publishes ``latest_safety_status``, a
different frame under a different name, and is never substituted for either --
still produces a complete payload: the fields it cannot measure are recorded as
an explicit ``null`` (one warning names the missing attribute on stderr) instead
of an invented ``0``.  Such a capture is not a failure, so the exit codes below
are unchanged by a missing capability.

Exit codes
----------
0  payload written
3  connect failure (client factory or ``connect()`` raised, or feedback died)
4  fewer fresh samples than requested before ``--timeout-s``
5  the payload could not be written (unwritable path, or a value JSON cannot hold)
6  the host metadata is unusable (empty, ``schema_version`` other than 2, or no
   ``motors`` mapping), so nothing was written
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Frozen CLI contract.
# ---------------------------------------------------------------------------

EXIT_OK = 0
EXIT_CONNECT_FAILED = 3
EXIT_INSUFFICIENT_SAMPLES = 4
EXIT_WRITE_FAILED = 5
# The host metadata face is unusable: storing it would archive a fingerprint that
# proves nothing about the calibration.
EXIT_INVALID_METADATA = 6

PAYLOAD_SCHEMA_VERSION = 1
# Version of the host metadata face this script knows how to interpret.
HOST_METADATA_SCHEMA_VERSION = 2
# Client attributes the capture reads.  Recorded as ``None`` (JSON ``null``) when
# the connected client does not publish one, never guessed and never replaced by
# a semantically different neighbour.
CLIENT_CAPABILITIES = ("latest_request_timing", "latest_control_status")

DEFAULT_PORT = 5556
# AlohaMini 2 Pro has 7 joints per arm (it adds ``wrist_yaw``); the client
# default ``alohamini1`` would mis-name every motor, so the benchmark defaults
# to the Pro model and lets the operator override it.
DEFAULT_ROBOT_MODEL = "alohamini2pro"
DEFAULT_SAMPLES = 5
DEFAULT_TIMEOUT_S = 30.0

# Small pause between observation polls: a hot spin would starve the ZMQ
# receive thread without making the samples any fresher.
POLL_INTERVAL_S = 0.002

ARTIFACT_DIR = Path("artifacts") / "alohamini-io"

READ_ONLY_BANNER = "read-only: no action sent"

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


# ---------------------------------------------------------------------------
# Small helpers.
# ---------------------------------------------------------------------------


def _finite_or_none(value: Any) -> float | None:
    """Return ``value`` as a finite float, or ``None`` when that is impossible."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or number in (float("inf"), float("-inf")):
        return None
    return number


def _default_out_path() -> Path:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return ARTIFACT_DIR / f"fingerprint-{stamp}.json"


def default_client_factory(host: str, port: int, robot_model: str) -> Any:
    """Build a real ``AlohaMiniClient``.

    The hardware imports stay inside this function so that loading the module
    (for example from a test or from ``--help``) never pulls in ``zmq``/``cv2``.
    """
    from lerobot.robots.alohamini.alohamini_client import AlohaMiniClient
    from lerobot.robots.alohamini.config_alohamini import AlohaMiniClientConfig

    config = AlohaMiniClientConfig(
        remote_ip=host,
        port_zmq_observations=port,
        robot_model=robot_model,
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
    records an explicit ``null`` for what is missing instead of inventing a
    number or reading a different frame under that name.
    """
    return [name for name in names if not hasattr(client, name)]


def _prime_state_only_window(client: Any) -> None:
    """Refill the client's in-flight window with state-only requests.

    ``connect()`` fills the window with ``include_cameras=True``, so the first
    responses of a capture would carry camera frames whose JPEG encoding work is
    no part of the state-only request age recorded here.  Guarded with
    ``getattr`` because a client without the method is valid; a failing prime
    costs a warning, not the capture.
    """
    prime = getattr(client, "prime_observation_request_window", None)
    if not callable(prime):
        return
    try:
        prime(include_cameras=False)
    except Exception as exc:  # a failed prime must never abort the capture
        print(f"warning: prime_observation_request_window failed: {exc}", file=sys.stderr)


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture an AlohaMini I/O fingerprint (read-only: no action sent). "
            "Reads state, robot metadata, control status and observation timing only; "
            "no motion command is ever constructed."
        ),
        epilog=(
            "This script is read-only by construction: it never builds or sends a command "
            "packet, so the arms, gripper, base and lift stay where they are."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", required=True, help="AlohaMini host IP address (required)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Host observation port")
    parser.add_argument(
        "--robot-model",
        default=DEFAULT_ROBOT_MODEL,
        help="Robot model reported by the host (must match the host side)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=DEFAULT_SAMPLES,
        help="Number of fresh observation requests to record (must be >= 1)",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="Wall-clock budget for collecting all samples",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=_default_out_path(),
        help="Destination JSON path (parent directories are created)",
    )
    args = parser.parse_args(argv)
    # A nonsense run shape is a usage error (2), not a failed connection (3): the
    # range is checked here so no socket is ever opened for it.
    if args.samples < 1:
        parser.error(f"--samples must be >= 1 (got {args.samples})")
    # ``--timeout-s`` is the only stop condition of a capture
    # (``deadline_s = clock() + timeout_s``), and it is checked with the same
    # ``not (math.isfinite(x) and x > 0.0)`` shape as ``measure_latency.py``:
    # ``inf`` (and ``1e400``, which ``float()`` turns into ``inf``) never expires;
    # ``nan`` never expires *and* spins, because ``now_s >= deadline_s`` is false
    # forever and ``pause_s = min(POLL_INTERVAL_S, max(0.0, deadline_s - now_s))``
    # is ``0.0`` (``max(0.0, nan) == 0.0``), so the poll pause is skipped
    # entirely.  ``0`` / negative values expire the budget before the first frame
    # and used to surface as ``InsufficientSamples`` (4), i.e. a sampling problem
    # the operator did not have.  All five are usage errors (2) instead -- the
    # exit-code table itself does not move: 4 keeps its meaning (the sequence
    # froze / too few fresh samples within a *legal* budget).
    if not (math.isfinite(args.timeout_s) and args.timeout_s > 0.0):
        parser.error(f"--timeout-s must be a finite number > 0 (got {args.timeout_s})")
    return args


# ---------------------------------------------------------------------------
# Sampling.
# ---------------------------------------------------------------------------


def _sample_from_timing(sequence: int, timing: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "observation_sequence": int(sequence),
        "age_s": _finite_or_none(timing.get("age_s")),
        "sent_monotonic_s": _finite_or_none(timing.get("sent_monotonic_s")),
        "received_monotonic_s": _finite_or_none(timing.get("received_monotonic_s")),
    }


def collect_samples(
    client: Any,
    samples: int,
    timeout_s: float,
    sleep: Callable[[float], None] | None = None,
    clock: Callable[[], float] | None = None,
) -> list[dict[str, Any]]:
    """Collect ``samples`` rows, one per *growth* of ``client.observation_sequence``.

    A request that comes back with an unchanged sequence number is a repeat of a
    frame we already recorded, so it is skipped instead of counted twice.  When
    the sequence never grows the run fails with ``InsufficientSamples`` rather
    than silently writing a thinner payload.

    A client without the per-request timing face does not abort the capture: its
    timing fields are recorded as an explicit ``None`` and one warning naming the
    missing attribute is printed to stderr.  The warning is emitted once per call
    so a run cannot bury stderr under one line per sample.
    """
    sleep_fn = time.sleep if sleep is None else sleep
    clock_fn = time.monotonic if clock is None else clock
    deadline_s = clock_fn() + timeout_s
    last_sequence = client.observation_sequence
    collected: list[dict[str, Any]] = []
    timing_missing = "latest_request_timing" in missing_client_capabilities(client)
    if timing_missing:
        print(
            "warning: the client publishes no latest_request_timing; age_s, sent_monotonic_s "
            "and received_monotonic_s are recorded as null for every sample (the values are "
            "missing, not zero).",
            file=sys.stderr,
        )

    while len(collected) < samples:
        now_s = clock_fn()
        if now_s >= deadline_s:
            raise InsufficientSamples(required=samples, collected=len(collected))
        client.get_observation(include_cameras=False)
        sequence = client.observation_sequence
        if sequence > last_sequence:
            last_sequence = sequence
            timing = {} if timing_missing else dict(client.latest_request_timing or {})
            collected.append(_sample_from_timing(sequence, timing))
        pause_s = min(POLL_INTERVAL_S, max(0.0, deadline_s - now_s))
        if pause_s > 0.0:
            sleep_fn(pause_s)

    return collected


# ---------------------------------------------------------------------------
# Rendering + payload.
# ---------------------------------------------------------------------------


def _cell(value: Any) -> str:
    return "" if value is None else str(value)


def render_table(metadata: Mapping[str, Any]) -> str:
    """Render one row per motor with all eight calibration fields as columns."""
    motors = metadata.get("motors") if isinstance(metadata, Mapping) else None
    motors = motors if isinstance(motors, Mapping) else {}
    name_width = max([len(str(motor)) for motor in motors] + [len("motor")])
    header = f"{'motor':<{name_width}}  " + "  ".join(f"{field:>14}" for field in MOTOR_FIELDS)
    lines = [header, "-" * len(header)]
    for motor, entry in motors.items():
        entry = entry if isinstance(entry, Mapping) else {}
        values = "  ".join(f"{_cell(entry.get(field)):>14}" for field in MOTOR_FIELDS)
        lines.append(f"{str(motor):<{name_width}}  {values}")
    return "\n".join(lines)


def build_payload(
    *,
    metadata: Mapping[str, Any],
    control_status: Mapping[str, Any] | None,
    observation_state: Mapping[str, float],
    samples: int,
    host: str,
    robot_model: str,
    sample_rows: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Assemble the fingerprint payload; ``robot_metadata`` is copied verbatim."""
    return {
        "schema_version": PAYLOAD_SCHEMA_VERSION,
        "captured_utc": datetime.now(UTC).isoformat(timespec="seconds"),
        "host": host,
        "requested_robot_model": robot_model,
        "read_only": True,
        "sample_count": int(samples),
        "samples": [dict(row) for row in (sample_rows or ())],
        "robot_metadata": metadata,
        "control_status": control_status,
        "observation_state": observation_state,
    }


def write_json(path: Path | str, payload: Mapping[str, Any]) -> None:
    """Write ``payload`` as UTF-8 JSON with sorted keys.

    ``allow_nan=False`` keeps ``NaN``/``Infinity`` out of the file: those
    literals are not JSON, and a file that ``json.loads`` cannot reproduce is not
    a fingerprint.  A value JSON cannot hold therefore raises here (``TypeError``
    for ``bytes`` and friends, ``ValueError`` for non-finite floats) instead of
    being written in a form no consumer can read.
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
    target.write_text(text + "\n", encoding="utf-8")


def validate_metadata(metadata: Any) -> str | None:
    """Return a refusal message when the host metadata is unusable, else ``None``.

    A fingerprint is evidence of the calibration, so an empty payload, a foreign
    ``schema_version`` or a missing ``motors`` mapping is refused instead of
    being archived as a successful capture.
    """
    if not isinstance(metadata, Mapping) or not metadata:
        return (
            "invalid robot metadata: the host reported no metadata payload.\n"
            "  Check that the host broadcasts the v2 metadata face and re-run; nothing was written."
        )
    schema_version = metadata.get("schema_version")
    if schema_version != HOST_METADATA_SCHEMA_VERSION:
        return (
            f"invalid robot metadata: schema_version={schema_version!r}, expected "
            f"{HOST_METADATA_SCHEMA_VERSION}.\n"
            "  The host speaks another metadata format, so the per-motor fields would not "
            "mean what this fingerprint claims; nothing was written."
        )
    motors = metadata.get("motors")
    if not isinstance(motors, Mapping) or not motors:
        return (
            "invalid robot metadata: no non-empty 'motors' mapping.\n"
            "  Without it the payload cannot prove the calibration of any motor; nothing was written."
        )
    return None


def _plain_state(observation: Mapping[str, Any]) -> dict[str, float]:
    """Keep only JSON-safe numeric state entries."""
    state: dict[str, float] = {}
    for key, value in observation.items():
        number = _finite_or_none(value)
        if number is not None:
            state[str(key)] = number
    return state


def _age_summary(samples: Sequence[Mapping[str, Any]]) -> str:
    ages = [
        age for age in (_finite_or_none(row.get("age_s")) for row in samples) if age is not None
    ]
    if not ages:
        return "ages unavailable"
    return f"age_ms min={min(ages) * 1e3:.3f} max={max(ages) * 1e3:.3f}"


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------


def main(
    argv: Sequence[str] | None = None,
    *,
    client_factory: Callable[[str, int, str], Any] | None = None,
) -> int:
    args = parse_args(argv)
    print(f"AlohaMini I/O fingerprint capture ({READ_ONLY_BANNER})")

    factory = default_client_factory if client_factory is None else client_factory
    client = None
    try:
        client = factory(args.host, args.port, args.robot_model)
        client.connect()
    except Exception as exc:  # every connect failure maps to one exit code
        print(f"connect failed: {exc}", file=sys.stderr)
        _close_quietly(client)
        return EXIT_CONNECT_FAILED

    # ``connect()`` prefilled the window with camera requests: replace them with
    # state-only ones before the first sample is taken.
    _prime_state_only_window(client)

    try:
        samples = collect_samples(client, samples=args.samples, timeout_s=args.timeout_s)
    except InsufficientSamples as exc:
        print(f"insufficient samples: {exc}", file=sys.stderr)
        _close_quietly(client)
        return EXIT_INSUFFICIENT_SAMPLES
    except Exception as exc:  # transport died mid-capture
        print(f"connect failed while sampling: {exc}", file=sys.stderr)
        _close_quietly(client)
        return EXIT_CONNECT_FAILED

    raw_metadata = client.latest_robot_metadata
    metadata = dict(raw_metadata) if isinstance(raw_metadata, Mapping) else {}
    refusal = validate_metadata(metadata)
    if refusal is not None:
        print(refusal, file=sys.stderr)
        _close_quietly(client)
        return EXIT_INVALID_METADATA

    try:
        observation_state = _plain_state(client.get_observation(include_cameras=False))
    except Exception as exc:  # transport died mid-capture
        print(f"connect failed while reading the final state: {exc}", file=sys.stderr)
        _close_quietly(client)
        return EXIT_CONNECT_FAILED

    # Read, never invented: a client without the control-status face (the vendor
    # client publishes ``latest_safety_status``, a different frame) records
    # ``null``.  Falling back to that neighbour would change what the payload
    # means under the same key.
    control_status = getattr(client, "latest_control_status", None)
    payload = build_payload(
        metadata=metadata,
        control_status=control_status,
        observation_state=observation_state,
        samples=len(samples),
        host=args.host,
        robot_model=args.robot_model,
        sample_rows=samples,
    )

    try:
        write_json(args.out, payload)
    except (OSError, TypeError, ValueError) as exc:
        print(f"write failed: {exc}", file=sys.stderr)
        _close_quietly(client)
        return EXIT_WRITE_FAILED

    _close_quietly(client)

    print(render_table(metadata))
    print(
        f"captured {len(samples)} sample(s) ({_age_summary(samples)}) "
        f"robot_model={metadata.get('robot_model')!r} "
        f"metadata_schema_version={metadata.get('schema_version')!r}"
    )
    print(f"payload written to {args.out}")
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
