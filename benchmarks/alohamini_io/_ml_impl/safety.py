"""Tracking preflight, safety-sidecar extraction, and summary helpers."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from typing import Any

from .common import (
    BASE_VELOCITY_KEYS,
    HOST_METADATA_SCHEMA_VERSION,
    LIFT_HEIGHT_KEY,
    MAX_TOTAL_TRAVEL_DEG,
    POS_SUFFIX,
    TARGET_SOURCE_VALUES,
    UnitConversionUnavailable,
    _finite_or_none,
    _quantiles,
    deg_from_units,
    deg_per_unit,
    units_from_deg,
)
from .rtt import _achieved_rate_hz

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

    if delta_deg == 0.0 or int(args.steps) < 1:
        return (
            None,
            "Gate E refused: --delta-deg must be non-zero and --steps at least 1; "
            "nothing was commanded.",
        )
    # Judged on the magnitude: a signed comparison is vacuous for a run that steps
    # down (``-21.0 > 20.0`` is false), so 21 x (-1.0 deg) would walk 21 degrees in
    # the direction nothing is watching.
    total_travel_deg = abs(delta_deg) * int(args.steps)
    if total_travel_deg > MAX_TOTAL_TRAVEL_DEG:
        return None, (
            f"Gate E refused: planned travel {total_travel_deg:.2f} deg exceeds the "
            f"{MAX_TOTAL_TRAVEL_DEG:.1f} deg cap (|{delta_deg:.2f}| deg x {args.steps} steps); "
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

    Never raises, and ``None`` is its whole failure mode: the timing face is client
    state like any other, and this function is now read twice per frame (when the
    safety sidecar row of a fresh frame is booked, and at the 90 % crossing), so a
    face whose readers raise -- a ``Mapping`` whose ``get`` explodes, an attribute
    read that throws -- may cost a frame its stamp but not the run.  Both call
    sites fall back to the poll-clock reading they already hold when this returns
    ``None``, so the fallback is a real clock reading and never an empty cell.

    The rtt path is deliberately untouched: it has its own
    ``unfilled_client_capabilities`` refusal for an unusable timing face, which
    this lenient read must not soften.
    """
    try:
        timing = getattr(client, "latest_request_timing", None)
        if not isinstance(timing, Mapping):
            return None
        return _finite_or_none(timing.get("received_monotonic_s"))
    except Exception:  # a timing face whose readers raise is not a broken link
        return None


def _safety_number(value: Any) -> float | None:
    """``value`` as a finite float when it *is* a measurement, else ``None``.

    ``bool`` is refused explicitly, and before the conversion: ``True`` is an
    ``int`` in Python, so ``float(True) == 1.0`` would write a 1 mA current (or a
    target of 1) out of a flag.  Everything ``_finite_or_none`` rejects is refused
    as well -- text, ``None``, ``nan``, ``inf`` -- and so is a value whose
    conversion raises anything else, an integer too large to be a double
    (``float(10**400)`` raises ``OverflowError``) included: the value is client
    state, and the sidecar contract is "never raise".
    """
    if isinstance(value, bool):
        return None
    try:
        return _finite_or_none(value)
    except Exception:  # a client value may raise out of "just float it" as well
        return None


def _safety_mapping(face: Any, key: str) -> dict[str, Any] | None:
    """The ``{name: value}`` pairs of ``face[key]``, or ``None`` when it holds none.

    ``None`` means "no readable mapping", which covers every shape the caller has
    to treat the same way: ``face`` is not a ``Mapping``, the key is absent, the
    value is not a ``Mapping``, or the mapping cannot be read at all.  Every read
    is guarded because the face is client state -- a proxy whose
    ``items()``/``__iter__`` raises is a corrupt frame, not a reason to lose the
    run -- and the pairs are copied into a plain ``dict`` so that later per-motor
    lookups cannot hit the same broken reader twice.
    """
    if not isinstance(face, Mapping):
        return None
    try:
        container = face[key]
    except Exception:  # absent key, or a mapping whose subscript raises
        return None
    if not isinstance(container, Mapping):
        return None
    try:
        return {name: value for name, value in container.items()}
    except Exception:  # a mapping whose readers raise instead of reporting
        return None


def _safety_target(targets: Mapping[str, Any] | None, motor: str) -> float | None:
    """The value ``targets`` holds for ``motor``, or ``None`` when it holds none.

    Both key shapes of the vendor frame are read, because the host does not use
    one shape for both tables: ``currents_ma`` / ``joint_holds`` are keyed by the
    bare motor name, while ``requested_targets`` / ``accepted_targets`` are keyed
    by ``<motor>.pos`` -- they are filled from the ``.pos`` action keys
    (``alohamini.py:987-988``).  A lookup that knows only the bare name writes two
    empty target columns on every frame of a real run, and a lookup that knows
    only ``.pos`` breaks on the older / synthetic bare-name frame.

    When a frame carries **both** keys for the same motor, the bare name wins: it
    is the key space the ``motor`` column of this row is printed from, so the row
    stays described by the name it carries.  A ``motor`` that already ends in
    ``.pos`` is not suffixed a second time.

    A motor absent from the mapping has no target, and the cell therefore stays
    empty: ``0.0`` would read as a target the host did report.  One function for
    both target columns so the two cannot drift apart in that rule.
    """
    if targets is None:
        return None
    if motor in targets:
        return _safety_number(targets[motor])
    if motor.endswith(POS_SUFFIX):
        # Already the qualified shape: there is no ``<motor>.pos.pos`` key to try.
        return None
    return _safety_number(targets.get(f"{motor}{POS_SUFFIX}"))


def _safety_hold_flag(face: Any, motor: str) -> bool | None:
    """Whether ``motor`` is held, read from ``face["joint_holds"]``.

    A real container answers the membership question, and a motor it does not list
    is ``False`` (a readable hold list that omits the motor says "not held").  A
    value that is no container at all -- ``None``, a number, a ``bool`` -- is no
    evidence either way, so the cell stays empty (``None``).

    On the pinned vendor client (local fork HEAD ``ec65318f``, the build the Pi
    runs) ``_joint_hold_goal`` has no write site at all: ``alohamini.py:298``
    initialises it and ``:985``/``:1076``/``:1088``/``:1128``/``:1131`` only read or
    pop it.  ``joint_holds`` is therefore ``{}`` on every real run, and this column
    is ``False`` on every row in the field -- ``False`` is the honest answer there
    (a readable hold table that omits the motor says "not held"), not a filled-in
    placeholder.  It also means this column can NOT answer "was the shoulder target
    held back by an over-current": that judgement rests on ``current_ma`` (a
    sustained high current with no progress) and, when the target was clipped, on
    ``requested_target`` vs ``accepted_target``.

    ``str`` is the trap this refuses: a motor name is a substring of its joint key
    (``"arm_left_shoulder_pan" in "arm_left_shoulder_pan.pos"`` is ``True``), so a
    membership test on a string would report a hold that no frame reported.  The
    membership test itself is guarded for the same reason as every other read of
    this face: a container whose ``__contains__`` raises is a corrupt frame.
    """
    try:
        holds = face["joint_holds"]
    except Exception:  # absent key, or a mapping whose subscript raises
        return None
    if not isinstance(holds, (Mapping, list, tuple, set)):
        return None
    try:
        return motor in holds
    except Exception:  # a container whose membership test raises
        return None


def _safety_target_source(face: Any) -> str | None:
    """Which layer produced the frame's targets, or ``None`` when it says nothing usable.

    ``target_source`` is a WHOLE-frame attribute of the vendor frame, not a
    per-motor one: the host stamps which layer produced the targets of this frame
    (``none`` / ``command`` / ``watchdog`` / ``protection``,
    ``TARGET_SOURCE_VALUES``), so every row of a frame carries the same value.

    Only those four strings are written.  Everything else -- a frame without the
    key, a value the frame cannot answer, a ``bool``, a number, a list, an empty
    string, or a string the vendor never sends -- leaves the cell EMPTY, and an
    empty cell is not the same statement as ``none``: ``none`` is the frame saying
    "no layer produced these targets", an empty cell is "this row has no source to
    report".  A defaulted read (``getattr`` / ``dict.get`` with a fallback) would turn
    the second into the first, so the key is read as it is and the VALUE is what
    gets checked.

    Never raises, for the same reason as every other read of this face: it is client
    state, and a proxy whose subscript raises is a corrupt frame, not a reason to
    lose the run.
    """
    if not isinstance(face, Mapping):
        return None
    try:
        value = face["target_source"]
    except Exception:  # absent key, or a mapping whose subscript raises
        return None
    if not isinstance(value, str) or value not in TARGET_SOURCE_VALUES:
        return None
    return value


def _safety_row(
    *,
    step_index: Any,
    sample_sequence: Any,
    t_monotonic_s: Any,
    motor: str | None,
    current_ma: float | None,
    requested_target: float | None,
    accepted_target: float | None,
    hold_active: bool | None,
    target_source: str | None,
) -> dict[str, Any]:
    """One sidecar row: exactly the nine ``SAFETY_CSV_FIELDS`` keys, no more.

    A single construction site for both shapes ``safety_frame_rows`` returns, so an
    insert into the sidecar can never come out with a field missing (``write_csv``
    reads ``row[field]`` and would raise ``KeyError`` on one) or with an extra key
    no header declares.
    """
    return {
        "step_index": step_index,
        "sample_sequence": sample_sequence,
        "t_monotonic_s": t_monotonic_s,
        "motor": motor,
        "current_ma": current_ma,
        "requested_target": requested_target,
        "accepted_target": accepted_target,
        "hold_active": hold_active,
        "target_source": target_source,
    }


def safety_frame_rows(
    face: Any,
    *,
    step_index: Any,
    sample_sequence: Any,
    t_monotonic_s: Any,
) -> list[dict[str, Any]]:
    """Snapshot one safety frame as sidecar rows: one row per motor, never raising.

    ``face`` is whatever ``client.latest_safety_status`` holds -- a mapping of the
    vendor side-channel frame -- but nothing here trusts that.  Any object may sit
    on that attribute, so a face that is not a mapping, a ``currents_ma`` that is
    not a non-empty mapping, a motor name that is not a ``str``, and a mapping
    whose readers raise all degrade to the same shape: **exactly one** row with
    ``motor=None`` and the frame's coordinates passed through.  The reads are
    wrapped in ``try``/``except Exception`` on purpose -- ``currents_ma`` is client
    state, so a proxy whose ``items()``/``__iter__``/``__contains__`` raises is a
    corrupt frame, and a corrupt side frame may cost the sidecar its contents but
    never the run.

    Motor names come from ``face["currents_ma"]`` in mapping iteration order, so a
    frame's rows follow the order the frame listed them; ``current_ma``,
    ``requested_target`` and ``accepted_target`` are that motor's entries of
    ``currents_ma`` / ``requested_targets`` / ``accepted_targets`` as finite
    floats (``_safety_number``: ``bool`` and non-numbers are no measurement), and
    ``hold_active`` is membership in ``joint_holds`` (``True``/``False`` for a real
    container, ``None`` when the value is no container -- ``_safety_hold_flag``).
    ``target_source`` is the frame's own source stamp (``_safety_target_source``):
    a whole-frame attribute, so every row of the frame carries the same value on
    both shapes -- the degraded ``motor=None`` row included, because the frame did
    say which layer produced its targets even when its motor table is unreadable.

    A cell stays empty (``None``, i.e. an empty CSV cell) when the frame carries no
    usable value, and it is never filled with ``0``.  That is a refusal to
    fabricate a measurement, not a silent degradation: a ``0 mA`` current reads
    like a measured zero, and a ``0`` target reads as a target the host never
    accepted.  The row still names the motor and the frame it came from, so the gap
    stays visible in the sidecar instead of hiding behind a plausible number.

    Nothing is substituted here: ``latest_safety_status`` is a different frame from
    ``latest_control_status`` and ``docs/ADR.md`` D12 forbids reading one in place
    of the other.  The safety snapshot lands in its own sidecar columns, and no
    control-status read path is touched -- this function reads the face it is
    handed and nothing else.

    Pure: the face is read, never sampled or mutated, and the coordinates
    (``step_index``, ``sample_sequence``, ``t_monotonic_s``) are the caller's
    provenance, passed through unchanged rather than derived here.
    """
    current_values = _safety_mapping(face, "currents_ma") or {}
    motors = [name for name in current_values if isinstance(name, str)]
    if len(motors) != len(current_values):
        # One non-``str`` key means the frame cannot be named per motor at all: a
        # numeric motor name beside the real ones would leave the operator unable
        # to tell which motor a row belongs to, so the whole frame degrades.
        motors = []
    target_source = _safety_target_source(face)
    if not motors:
        return [
            _safety_row(
                step_index=step_index,
                sample_sequence=sample_sequence,
                t_monotonic_s=t_monotonic_s,
                motor=None,
                current_ma=None,
                requested_target=None,
                accepted_target=None,
                hold_active=None,
                target_source=target_source,
            )
        ]
    requested_targets = _safety_mapping(face, "requested_targets")
    accepted_targets = _safety_mapping(face, "accepted_targets")
    return [
        _safety_row(
            step_index=step_index,
            sample_sequence=sample_sequence,
            t_monotonic_s=t_monotonic_s,
            motor=motor,
            current_ma=_safety_number(current_values[motor]),
            requested_target=_safety_target(requested_targets, motor),
            accepted_target=_safety_target(accepted_targets, motor),
            hold_active=_safety_hold_flag(face, motor),
            target_source=target_source,
        )
        for motor in motors
    ]


def _safety_frame_counts(rows: Sequence[Mapping[str, Any]]) -> tuple[int, int]:
    """``(frames, frames_with_motor_table)`` of the sidecar rows a run recorded.

    ``frames`` is the number of frames this run RECORDED, de-duplicated by
    ``sample_sequence``: one frame is booked per growth of the liveness counter
    (``safety_frame_rows``), a cached repeat is never recorded, so this is the same
    number as "how many frames grew" without the run carrying a second counter for
    it.  Rows whose ``sample_sequence`` is ``None`` form one group together -- that
    path records nothing today (the growth test in ``run_tracking_steps`` needs an
    ``int``), but a coordinate that is not a number must not be counted once per
    row.

    ``frames_with_motor_table`` is how many of those frames carried at least one
    row with a motor name, i.e. ``motor is not None``: that is exactly a frame whose
    ``currents_ma`` was a readable, ``str``-keyed, non-empty motor table, because
    ``safety_frame_rows`` degrades every other shape to a single ``motor=None`` row.
    Equal numbers therefore mean every recorded frame carried a table, and the
    difference is the number of frames whose sidecar rows have an empty ``motor``
    cell -- the degradation a whole-run warning is about.
    """
    frames: dict[Any, bool] = {}
    for row in rows:
        sequence = row["sample_sequence"]
        frames[sequence] = frames.get(sequence, False) or row["motor"] is not None
    return len(frames), sum(1 for has_table in frames.values() if has_table)


def _tracking_summary(
    rows: Sequence[Mapping[str, Any]],
    abort_reason: str | None,
    hold_action: Mapping[str, float],
    hold_error: str | None,
    *,
    commands_issued: int = 0,
    observations: int = 0,
    elapsed_s: float = float("nan"),
    safety_rows: Sequence[Mapping[str, Any]] = (),
    safety_missing: Sequence[str] = (),
    timestamp_fallback: bool = False,
) -> dict[str, Any]:
    """Assemble the tracking report from the rows recorded so far.

    ``commands_issued``/``observations`` are the counts the run really produced
    (the closing hold included) and ``elapsed_s`` is the time it took, so
    ``achieved_command_hz`` and ``achieved_rate_hz`` are measured evidence rather
    than the requested knobs echoed back: a run whose pacing thinned its sampling
    reports the thinned rate.  Both are ``nan`` when a rate cannot be computed.

    ``safety_rows`` carries the sidecar snapshots in arrival order (one row per
    motor per fresh frame, ``safety_frame_rows``) and ``safety_missing`` is the
    capability probe of the run's start (``missing_safety_capabilities``): the
    report itself does not print either of them -- they travel with the summary so
    the caller writes the sidecar beside the tracking CSV, and so a summary read by
    a test or a caller cannot silently lose the probe result that explains an empty
    sidecar.

    ``timestamp_fallback`` is the run-wide answer to "did any recorded frame (or the
    90 % crossing) have to be stamped with the poll clock instead of
    ``latest_request_timing["received_monotonic_s"]``".  It is a property of the run
    and not of a frame, so ``_run_tracking_mode`` names it on stderr exactly once; a
    run whose timing face answered every frame carries ``False`` and warns nothing.
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
        "safety_rows": list(safety_rows),
        "safety_missing": list(safety_missing),
        "timestamp_fallback": timestamp_fallback,
    }


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
