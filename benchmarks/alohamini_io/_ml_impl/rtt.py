"""RTT measurement mode for ``measure_latency.py``."""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from .common import (
    DEFAULT_HOST_LOOP_HZ,
    DEFAULT_REQUEST_WINDOW,
    EXIT_CLIENT_UNSUPPORTED,
    EXIT_CONNECT_FAILED,
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_OK,
    EXIT_STEP_RESPONSE_ABORTED,
    EXIT_WRITE_FAILED,
    IDLE_POLL_FLOOR_S,
    INTERRUPT_ABORT_REASON,
    MAX_STEP_DELTA_DEG,
    RTT_CSV_FIELDS,
    RTT_RATE_WARNING_HZ,
    InsufficientSamples,
    UnfilledClientCapability,
    _finite_or_none,
    _host_loop_hz_or_upper_bound,
    _prefetch_terms_s,
    _queue_terms_s,
    _request_token_text,
    _unfilled_capability,
    _write_csv_after_interrupt,
    effective_request_window,
    net_rtt_estimate_s,
    summary_stats,
    write_csv,
)

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
    token = _request_token_text(timing.get("token"))
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
    never grows ends in ``InsufficientSamples`` rather than a short CSV -- unless
    the timing face itself is unusable, in which case the give-up moment below
    reports the capability refusal instead.

    ``rows_out``, when given, is the very list the rows are appended to, so a
    caller that catches an exception raised out of this loop (``KeyboardInterrupt``
    in particular) still holds every row collected up to that point instead of
    losing them to the local scope.

    The loop paces against an absolute deadline: sample ``n`` is due at
    ``start + n / rate_hz``, so a blocking read is charged to the period instead
    of being added on top of it (sleeping one whole period after every read
    would cap the achieved rate at ``1 / (read + period)``).  Four arms decide
    the wait of one poll, and the reason they are four is that "paced" has two
    opposite failure modes:

    * **a fresh frame** targets its own sampling instant,
      ``start + len(rows) * period`` -- unchanged since the first version;
    * **a duplicate frame still near its beat** (the row deadline is at most one
      period behind) chases that deadline,
      ``max(next_row, now + IDLE_POLL_FLOOR_S)``.  A duplicate does not owe a
      whole period of its own: charging it one throttled an honest host that
      answers slower than the client polls to ``rate_hz / (1 + duplicates)``
      (two duplicate frames per growth measured 0.51 x the requested rate).  The
      floor keeps the chase non-zero once the deadline is already in the past,
      so a catch-up costs a floor-sized wait instead of a spin;
    * **a duplicate frame more than one period behind** is a frozen tail: the
      sequence stopped growing, so the row deadline stays in the past for the
      rest of the run and the floor-size chase would poll at 1 kHz -- 973 calls
      for a 1 s budget at 100 Hz and 29,903 for the 30 s / 30 Hz default, i.e. a
      "waiting" loop that wakes 1,000 times per second.  There is nothing left to
      catch up to, so the wait goes back to ``idle_pause_s``, one sampling
      period: one wait per period is the cost of a dead link, and the budget
      (not the poll rate) ends the run.  Same bound as R24's arm, for the same
      reason;
    * **a duplicate frame before the first row** has no beat to chase at all
      (``next_row`` is ``start``), and waiting nothing there was the 4.3e6
      ``get_observation`` calls/s spin measured before this arm existed; it waits
      ``idle_pause_s`` too.

    ``idle_pause_s`` is ``max(period_s, IDLE_POLL_FLOOR_S)``: the floor bounds
    *every* rate's wait (a huge finite rate such as ``--rate-hz 1e9`` gave a 1 ns
    wait before) and is the whole wait when ``rate_hz`` gives no usable period
    (non-finite or non-positive, refused for the CLI by ``parse_args`` but
    reachable by a direct caller).  With no usable period there is no beat to
    compare against, so the catch-up and frozen-tail arms cannot be told apart --
    ``period_s`` is ``0``, which makes every duplicate poll take the frozen-tail
    arm, and that arm's ``idle_pause_s`` *is* the floor there: the wait stays
    non-zero without a second special case.

    ``request_window`` and ``rate_hz`` reach the row unconverted: the queue and
    prefetch terms of ``net_rtt_estimate_s`` own their own gates, so a row can
    never report a term the estimator did not subtract (or the other way round);
    ``host_loop_hz`` also sets the phase bound.

    The second capability probe runs at *every* growth of
    ``observation_sequence`` and before that frame's row is appended
    (``unfilled_client_capabilities``): a face that exists and holds nothing
    usable is only distinguishable from one the host has not timed yet once a
    fresh frame has arrived, and refusing there keeps the run from writing a row
    -- or a CSV -- of empty timing cells.  Probing per growth and not once is
    deliberate: a face that is filled on the first frames and empties later
    would slip past a one-shot latch and be written out from the frame it
    emptied on.  "Holds nothing usable" is key-based, not "is the dict empty":
    a face missing one of ``REQUIRED_REQUEST_TIMING_KEYS`` is refused here too
    (a half-filled face would write the same empty/``NaN`` cells), and so is one
    that carries all four keys while a value cannot be used as a measurement --
    a negative ``age_s``, a ``bool`` stamp or age (``float(True) == 1.0``), a
    ``token`` that is not text or is blank; the four rules and their reasons are
    spelled out at ``missing_request_timing_keys``.

    The same probe runs a second time at the moment the loop gives up: that is
    the only point where a capability gap can be told from a genuinely short
    budget, and only this ordering tells them apart:

    * on the give-up moment (``now >= deadline``) the timing face is probed
      *before* choosing the exception: unusable face -> ``UnfilledClientCapability``
      (with the missing keys), usable face -> ``InsufficientSamples``.  A
      sequence frozen for the whole run never grows, so the per-growth probe
      never fires; without this one a client that publishes an untimed face
      would be reported as a sampling-budget problem (exit 7) and sent to
      ``--timeout-s`` / ``--samples`` instead of to the client.  The probe needs
      at least one frame of this run: a budget that expired before the first
      ``get_observation`` leaves the face unwritten by this loop, which says
      nothing about the client, so that run keeps ``InsufficientSamples``;
    * the per-growth probe is unchanged and stays where it is, so a client that
      *does* grow keeps being refused on the frame whose face is unusable, before
      that frame's row exists.

    A genuinely short run keeps ``InsufficientSamples``: a filled face with a
    frozen sequence (a dead feedback link, not a capability gap) still gives
    exit 7.  The give-up answer costs the whole budget to arrive -- the loop can
    only know a sequence never grew once the deadline has passed -- so a frozen,
    untimed run is reported as 9 only after ``--timeout-s``, not immediately.

    The cost of that strictness is the mirror case, accepted on purpose: a client
    whose *first* fresh frame carries no timing is refused too, even when every
    later frame is timed.  The probe cannot tell "this host never times a
    request" from "this host had not timed it yet" on the frame it looks at, and
    waiting one frame before deciding would reopen the start of the run, so the
    run is refused instead of sampled.  Refusing a possibly legal client costs one
    re-run; accepting an untimed frame writes empty cells that read like a
    measurement.  Whether the real vendor client ever takes that shape is
    unverified.

    On refusal ``UnfilledClientCapability`` propagates out of this loop before
    the row is appended, so neither the CSV nor its header is written and only
    the rows of a caller-supplied ``rows_out`` exist.  Closing the connection is
    ``main``'s job (its ``finally``); this function never connects or closes.
    """
    sleep_fn = time.sleep if sleep is None else sleep
    clock_fn = time.monotonic if clock is None else clock
    period_s = 1.0 / rate_hz if rate_hz > 0 else 0.0
    # The wait of a poll that grew no row: never below ``IDLE_POLL_FLOOR_S``.
    # ``period_s`` is the interval to pace against when it is finite and positive,
    # but a huge rate makes it far too fine to wait on (``--rate-hz 1e9`` gives
    # 1 ns), and a direct caller can pass ``rate_hz=nan``/``inf`` -- both land on
    # ``0.0`` here, which would make the wait zero and the loop poll at the
    # client's maximum speed.  Both shapes get the floor.
    period_usable = math.isfinite(period_s) and period_s > 0.0
    idle_pause_s = max(period_s, IDLE_POLL_FLOOR_S) if period_usable else IDLE_POLL_FLOOR_S
    start_s = clock_fn()
    deadline_s = start_s + timeout_s
    last_sequence = client.observation_sequence
    rows: list[dict[str, Any]] = [] if rows_out is None else rows_out
    observed = 0

    while len(rows) < samples:
        now_s = clock_fn()
        if now_s >= deadline_s:
            # The give-up moment: budget is over and no more rows are coming.
            # Before blaming the budget, ask whether this client can be measured
            # at all -- a sequence that never grew hides an unfilled face from the
            # per-growth probe below, and "insufficient samples" would send the
            # operator to --timeout-s instead of to the client.  A *filled* face
            # still ends in InsufficientSamples here (a dead feedback link is not
            # a capability gap), and so does a budget that expired before the
            # first frame: a face no frame ever wrote says nothing about the
            # client, only that this run asked it for nothing.
            if observed > 0:
                refusal = _unfilled_capability(client)
                if refusal is not None:
                    raise refusal
            raise InsufficientSamples(required=samples, collected=len(rows))
        client.get_observation(include_cameras=False)
        observed += 1
        sequence = client.observation_sequence
        grew = sequence > last_sequence
        if grew:
            last_sequence = sequence
            # Strictly per growth, and before the row: a face that empties in the
            # middle of a run must never reach the CSV as an empty timing row.  A
            # first frame without timing is refused as well (see docstring).
            refusal = _unfilled_capability(client)
            if refusal is not None:
                raise refusal
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
        # Four arms, decided by "did this poll grow the sequence" and "is the loop
        # still near its beat":
        #
        # * a fresh frame targets its own sampling instant, unchanged
        #   (``start_s + len(rows) * period_s``);
        # * a duplicate frame whose row deadline is at most one period behind
        #   chases that deadline, floored at ``IDLE_POLL_FLOOR_S`` so a deadline
        #   already in the past costs a floor-sized wait instead of a spin.  This
        #   is the arm that keeps the achieved rate at ``--rate-hz`` when an honest
        #   host answers slower than the client polls -- charging a duplicate a
        #   whole period throttled it to ``rate_hz / (1 + duplicates)`` (two
        #   duplicates per growth measured 0.51 x the requested rate);
        # * a duplicate frame more than one period behind is a *frozen tail*: the
        #   sequence stopped growing, so the deadline stays in the past forever and
        #   chasing it with the floor would poll at 1 kHz (29,903 calls over the
        #   30 s / 30 Hz default budget).  There is nothing left to catch up to, so
        #   the wait goes back to ``idle_pause_s``, one period: one wait per
        #   sampling period is the cost of a dead link, and the budget (not the
        #   loop) ends the run;
        # * a duplicate frame before the first row has no beat to chase
        #   (``next_row_s`` is ``start_s``) and waits ``idle_pause_s`` too -- the arm
        #   that bounds the polls of a run whose sequence never grows (R24).
        #
        # With an unusable ``period_s`` (non-finite or non-positive: no beat to
        # compare against) every duplicate poll takes the frozen-tail arm by
        # construction -- ``next_row_s`` is ``start_s`` and ``period_s`` is 0 -- and
        # ``idle_pause_s`` is already the floor, so the wait stays non-zero.
        next_row_s = start_s + len(rows) * period_s
        if grew:
            target_s = next_row_s
        elif rows:
            if next_row_s < now_s - period_s:
                target_s = now_s + idle_pause_s
            else:
                target_s = max(next_row_s, now_s + IDLE_POLL_FLOOR_S)
        else:
            target_s = now_s + idle_pause_s
        pause_s = min(target_s - now_s, deadline_s - now_s)
        if pause_s > 0.0:
            sleep_fn(pause_s)

    return rows


def resolve_delta_deg(args: argparse.Namespace) -> float:
    """Per-step delta actually used: the request with its magnitude clamped to the cap.

    The cap is on the MAGNITUDE, so both directions get the same safety limit and
    the sign of the request survives it: ``--delta-deg -10`` commands -1.0 deg,
    not a ten degree step down.
    """
    delta = float(args.delta_deg)
    return min(max(delta, -MAX_STEP_DELTA_DEG), MAX_STEP_DELTA_DEG)


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
    # rate is slower; the prefetch wait is always counted).  That includes the
    # loop rate itself: the rows normalise an unusable ``host_loop_hz`` to
    # ``HOST_LOOP_HZ_UPPER_BOUND``, so the report gates the window with that same
    # value instead of the raw flag (see ``_host_loop_hz_or_upper_bound``).
    report_loop_hz = _host_loop_hz_or_upper_bound(float(args.host_loop_hz))
    queued_window = effective_request_window(
        int(args.request_window), float(args.rate_hz), report_loop_hz
    )
    queue_bound_s, queue_mean_s = _queue_terms_s(queued_window, report_loop_hz)
    prefetch_bound_s, prefetch_mean_s = _prefetch_terms_s(
        int(args.request_window), float(args.rate_hz), report_loop_hz
    )
    # Printed next to achieved_rate_hz on purpose: at a low sampling rate the
    # idle time of one sampling period dominates age_s (see module docstring),
    # so the estimate has to be read with the rate.  No existing field and no
    # exit code changes with it.
    rate_warning = _rtt_rate_warning(args)
    return "\n".join(
        [
            "AlohaMini observation latency report (mode: rtt, zero motion)",
            f"  host={args.host} port={args.port} robot_model={args.robot_model} "
            f"samples={len(rows)}",
            f"  achieved_rate_hz={achieved:.1f} (requested {float(args.rate_hz):.1f} Hz) "
            f"over {elapsed_s:.3f} s",
            *([] if rate_warning is None else [rate_warning]),
            f"  request_age_ms      count={age_stats['count']} min={age_stats['min_ms']:.3f} "
            f"p50={age_stats['p50_ms']:.3f} p95={age_stats['p95_ms']:.3f} "
            f"p99={age_stats['p99_ms']:.3f} max={age_stats['max_ms']:.3f}",
            f"  net_rtt_estimate_ms count={net_stats['count']} min={net_stats['min_ms']:.3f} "
            f"p50={net_stats['p50_ms']:.3f} p95={net_stats['p95_ms']:.3f} "
            f"p99={net_stats['p99_ms']:.3f} max={net_stats['max_ms']:.3f}",
            "  net_rtt_bounds_ms   "
            f"low p50={low_stats['p50_ms']:.3f} p95={low_stats['p95_ms']:.3f} "
            f"| high p50={high_stats['p50_ms']:.3f} p95={high_stats['p95_ms']:.3f}",
            f"  request_window={args.request_window} queued_window={queued_window} "
            f"host_loop_hz={float(args.host_loop_hz):.1f} "
            f"queue_bound_ms={queue_bound_s * 1000.0:.3f} "
            f"queue_mean_ms={queue_mean_s * 1000.0:.3f} "
            f"prefetch_bound_ms={prefetch_bound_s * 1000.0:.3f} "
            f"prefetch_mean_ms={prefetch_mean_s * 1000.0:.3f} "
            f"host_timing_missing={missing}/{len(rows)}",
            "  NOTE: net_rtt_estimate_ms is an ESTIMATE, not a pure network round trip: age_s "
            "also contains the host state-sampling duration, the waiting phase of the request "
            "inside the host loop, the queueing of the other in-flight requests and the prefetch "
            "wait of the window (the client answers the oldest request and refills the window "
            "only afterwards).  "
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
    except UnfilledClientCapability as exc:
        # Caught ahead of the generic ``Exception`` below: this is a capability
        # refusal, not a dead transport, and it shares exit code 9 with the
        # absent-face refusal.  The wording is the only thing that tells the two
        # apart, so it names the face as unfilled -- never the "publishes no ..."
        # sentence of ``main`` -- and it names the keys that face is missing: a
        # half-filled face is only actionable when the operator sees *which* cell
        # would have been blank.  It also never says "insufficient samples": a
        # frozen sequence reaches this branch from the give-up moment, and the
        # operator has to be sent to the client, not to ``--timeout-s``.
        # The clause between ``missing key(s):`` and ``, so`` is the key list: a
        # bare name for a key the face does not carry (or carries as ``None``), and
        # ``<key> (<reason>)`` for one it carries with an unusable value -- the
        # reason is always there, the quoted value only when ``_short_value`` says
        # it prints unchanged.  Nothing printed here holds ``", "`` or a line break,
        # so the list still splits into one item per key and the ``, so ...``
        # remediation after it survives.
        missing_keys = (
            ""
            if not exc.missing_key_labels
            else "; missing key(s): " + ", ".join(exc.missing_key_labels)
        )
        print(
            "unsupported client: it publishes "
            + ", ".join(exc.names)
            + " but never fills it completely, so rtt cannot be measured.\n"
            "  The face is there but not filled completely"
            + missing_keys
            + ", so a run would write empty token, stamp and age cells that read like a\n"
            "  measurement.  Fix the client (or the host that stopped timing requests) and\n"
            f"  re-run; no CSV was written ({len(rows)} collected row(s) were discarded).",
            file=sys.stderr,
        )
        return EXIT_CLIENT_UNSUPPORTED
    except KeyboardInterrupt:
        interrupted = True
        print(
            f"probe aborted: {INTERRUPT_ABORT_REASON}; "
            f"writing the {len(rows)} sample(s) collected so far.",
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
