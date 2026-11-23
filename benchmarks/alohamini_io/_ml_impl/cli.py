"""CLI argument parser for ``measure_latency.py``."""

from __future__ import annotations

import argparse
import math
from collections.abc import Sequence
from pathlib import Path

from .common import (
    ANCHOR_CHOICES,
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
    MAX_STEP_DELTA_DEG,
    _default_csv_path,
)
from .rtt import resolve_delta_deg

# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------


def parse_args(
    argv: Sequence[str] | None = None,
    *,
    description: str | None = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__ if description is None else description,
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
            f"Requested step size in DEGREES; a negative value steps the joint down (the "
            f"magnitude is clamped to {MAX_STEP_DELTA_DEG} deg, the sign is kept). A .pos "
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
    parser.add_argument(
        "--anchor",
        choices=ANCHOR_CHOICES,
        default=DEFAULT_ANCHOR,
        help=(
            "What a tracking step lays its goal on; only --mode tracking reads it (--mode rtt "
            "accepts the flag and ignores it). present (the default, and the behaviour "
            "every earlier run had) re-reads the measured position every step and drives "
            "delta_deg above it, so the lead is always one single step and never accumulates; "
            "command anchors on the target the previous step sent (the first step falls back "
            "to the measured position), so the lead accumulates across steps and reproduces "
            "the error-accumulating shape of a teleoperated run"
        ),
    )
    args = parser.parse_args(argv)
    # A nonsense run shape is a usage error (2), not a connect failure (3): the
    # ranges are checked here so no socket is ever opened for them.
    if args.samples < 1:
        parser.error(f"--samples must be >= 1 (got {args.samples})")
    # Finiteness and sign together, in the same ``not (a and b)`` shape as
    # ``--control-hz`` below: ``--rate-hz`` is the period ``collect_rtt_samples``
    # paces on and the rate the prefetch/queue terms are gated with, and ``inf``
    # is ``> 0`` while ``nan`` makes every comparison false.  Letting either
    # through degenerates ``period_s`` to ``0.0``, i.e. the full-speed poll loop
    # the frozen-sequence arm of that loop exists to prevent.
    if not (math.isfinite(args.rate_hz) and args.rate_hz > 0.0):
        parser.error(f"--rate-hz must be a finite number > 0 (got {args.rate_hz})")
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
    # ``inf`` is ``> 0`` and ``1 / inf == 0.0``, so the phase and queue terms both
    # collapse to zero for a host that cannot be that fast; ``nan`` makes every
    # comparison false and used to be read two ways at once (the rows fall back to
    # ``HOST_LOOP_HZ_UPPER_BOUND``, the report's ``effective_request_window``
    # reads the raw value).  ``1e400`` is ``inf`` after ``float()``.  Same
    # ``not (a and b)`` shape as ``--control-hz``: ``x <= 0.0`` alone lets ``nan``
    # through.
    if not (math.isfinite(args.host_loop_hz) and args.host_loop_hz > 0.0):
        parser.error(f"--host-loop-hz must be a finite number > 0 (got {args.host_loop_hz})")
    # ``--timeout-s`` is the only stop condition of the rtt loop
    # (``deadline_s = start_s + timeout_s``): ``inf`` never arrives and ``nan``
    # makes ``now_s >= deadline_s`` false forever, so a frozen sequence would run
    # until the operator kills the process.  ``0`` and negative values expire the
    # budget before the first frame, and the give-up probe deliberately keeps that
    # shape as ``InsufficientSamples`` (7) -- a capability gap would be reported
    # as a sampling-budget problem.  Same finiteness shape as ``--control-hz``.
    if not (math.isfinite(args.timeout_s) and args.timeout_s > 0.0):
        parser.error(f"--timeout-s must be a finite number > 0 (got {args.timeout_s})")
    # Tracking knobs: a zero step count commands nothing while looking like a
    # run, and a non-positive settle budget or a negative tolerance makes the
    # settle judgement meaningless.
    if args.steps <= 0:
        parser.error(f"--steps must be >= 1 (got {args.steps})")
    # The settle budget is a stop condition of a tracking run, so ``inf`` never
    # expires and ``nan`` makes every "is this step still settling" comparison
    # false: a joint that never settles runs until the operator kills the process.
    # ``1e400`` is ``inf`` after ``float()``.  Finiteness and sign in one
    # ``not (a and b)`` test for the same reason as ``--control-hz``: a sign-only
    # ``<= 0.0`` check is false for ``nan``.
    if not (math.isfinite(args.settle_timeout_s) and args.settle_timeout_s > 0.0):
        parser.error(
            f"--settle-timeout-s must be a finite number > 0 (got {args.settle_timeout_s})"
        )
    # ``0`` stays legal: a step is settled when the residual is at or below the
    # tolerance, so ``0`` is the strictest legal setting and the bound is
    # ``>= 0``.  What is refused is a non-finite tolerance, which the cross-check
    # below cannot catch -- ``nan >= delta`` is false (no step would ever count as
    # settled, emptying the settle columns) and ``delta >= inf`` is false too.
    if not (math.isfinite(args.settle_tolerance_deg) and args.settle_tolerance_deg >= 0.0):
        parser.error(
            f"--settle-tolerance-deg must be a finite number >= 0 (got {args.settle_tolerance_deg})"
        )
    # A zero-magnitude step is not a step, and a negative step IS a step (the same
    # experiment in the other direction, field manual §10.5 M2): the bound is
    # ``!= 0`` and the ``MAX_STEP_DELTA_DEG`` cap in ``resolve_delta_deg`` applies
    # to the magnitude, so the sign of the request survives it.  Checked before
    # the tolerance cross-check below, which would otherwise blame the clamp for a
    # value the operator typed: ``--delta-deg 0`` used to surface as
    # "settle-tolerance-deg must be below the effective --delta-deg ... delta 0.0
    # deg after the 1.0 deg clamp".
    # ``inf`` is ``!= 0`` and would pass a zero-only check, after which
    # ``resolve_delta_deg`` silently clamps it to ``MAX_STEP_DELTA_DEG``: the run
    # would command a step the operator never asked for instead of refusing a
    # value that cannot be a step.  ``-inf`` is the negative twin the magnitude
    # clamp must not "fix".  ``nan`` is ``!= 0`` true and makes every step-size
    # comparison downstream false.  ``1e400`` is ``inf`` after ``float()``.  ``0``
    # and ``-0.0`` keep their own refusal (below), so the finiteness test is added
    # around that rule, not instead of it.
    if not (math.isfinite(args.delta_deg) and args.delta_deg != 0.0):
        parser.error(
            f"--delta-deg must be a finite and non-zero number (got {args.delta_deg}); "
            "negative steps are legal, a zero step commands nothing"
        )
    # A tolerance at or above the step the run actually takes judges the joint
    # settled on the first poll, which leaves ``rise_90`` empty in every row.  The
    # reference is the CLAMPED step (``resolve_delta_deg``): ``--delta-deg 5.0``
    # commands 1.0 deg, so a 1.0 deg tolerance is already the empty-column case.
    # Compared on the magnitude: the residual the settle loop tests is
    # ``abs(measured - target)``, so ``0.15 >= -1.0`` would wave every negative
    # step through while accepting ``-0.05`` with a ``0.15`` tolerance.
    # Checked in every mode on purpose: rtt parses the same knobs and ignores them,
    # and the default pair (1.0 / 0.15) stays legal.
    effective_delta_deg = resolve_delta_deg(args)
    if args.settle_tolerance_deg >= abs(effective_delta_deg):
        parser.error(
            f"--settle-tolerance-deg must be below the effective --delta-deg magnitude "
            f"(got settle-tolerance {args.settle_tolerance_deg} deg >= "
            f"|delta| {abs(effective_delta_deg)} deg after the {MAX_STEP_DELTA_DEG} deg clamp)"
        )
    if args.csv is None:
        args.csv = _default_csv_path(args.mode)
    return args
