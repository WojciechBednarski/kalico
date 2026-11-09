"""Pressure advance equivalence for [extruder_profile] profile moves.

These drive the real extruder kinematics offline -- a steppersync, a
stepcompress and a serialqueue over a pipe, the same harness
test_stepcompress_hp.py uses -- so they need the compiled C helper and a
POSIX host.  They are the proof behind the claim in extruder_profile.py
that a profile move is compensated exactly as a printing move is; the
module's own startup self-check only verifies that the hook points still
have the expected shape.

What is pinned here:
  * a profile move (axes_r = (+1, 0, 0)) receives pressure advance;
  * an extrude-only move as PrinterExtruder writes it today
    (axes_r = (0, 0, axis_r)) receives none -- the behaviour this feature
    exists to work around, so a change upstream shows up as a failure here
    rather than as silently uncompensated test runs;
  * a profile move's stepper positions equal those of an equivalent
    axis-aligned +X printing move, sample for sample (V2);
  * they equal an independent closed-form reference (V1);
  * a retraction segment receives no advance (R3).
"""

import math
import os

import pytest

from klippy import chelper
from klippy.extras.extruder_profile import parse_profile

MCU_FREQ = 16000000.0
MAX_ERROR = int(0.000025 * MCU_FREQ)
STEP_DIST = 0.000625  # 1.25mm rotation / 2000 steps, a typical extruder
SMOOTH_TIME = 0.040
SMOOTHER_COEFFS = [15.0 / 8.0, 0.0, -15.0, 0.0, 30.0]
START_TIME = 0.100


# ----------------------------------------------------------------------
# Offline step generation
# ----------------------------------------------------------------------


class _Rig:
    """One extruder stepper fed from one trapq, generating steps offline."""

    def __init__(
        self,
        ffi_main,
        ffi_lib,
        advance,
        time_offset=0.0,
        smooth_time=SMOOTH_TIME,
        model="linear",
        model_params=None,
    ):
        self.ffi_main, self.ffi_lib = ffi_main, ffi_lib
        self.read_fd, self.write_fd = os.pipe()
        self.serialqueue = ffi_main.gc(
            ffi_lib.serialqueue_alloc(self.write_fd, b"u", 0, b"ep_test"),
            ffi_lib.serialqueue_free,
        )
        self.ssm = ffi_main.gc(
            ffi_lib.steppersyncmgr_alloc(), ffi_lib.steppersyncmgr_free
        )
        ss = ffi_lib.steppersyncmgr_alloc_steppersync(self.ssm)
        se = ffi_lib.steppersync_alloc_syncemitter(ss, b"ep_test", 1)
        self.sc = ffi_lib.syncemitter_get_stepcompress(se)
        ffi_lib.stepcompress_fill(self.sc, 1, MAX_ERROR, 1, 2)
        ffi_lib.steppersync_setup_movequeue(ss, self.serialqueue, 512)
        ffi_lib.steppersync_set_time(ss, 0.0, MCU_FREQ)

        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.sk = ffi_main.gc(
            ffi_lib.extruder_stepper_alloc(), ffi_lib.extruder_stepper_free
        )
        for axis in (b"x", b"y", b"z"):
            assert (
                ffi_lib.extruder_set_smoothing_params(
                    self.sk,
                    axis,
                    len(SMOOTHER_COEFFS),
                    SMOOTHER_COEFFS,
                    smooth_time,
                    0.0,
                )
                == 0
            )
        if model_params is None:
            model_params = [advance]
        func = {
            "linear": ffi_lib.pressure_advance_linear_model_func,
            "tanh": ffi_lib.pressure_advance_tanh_model_func,
            "recipr": ffi_lib.pressure_advance_recipr_model_func,
        }[model]
        ffi_lib.extruder_set_pressure_advance(
            self.sk, 0.0, len(model_params), model_params, func, time_offset
        )
        ffi_lib.itersolve_set_trapq(self.sk, self.trapq, STEP_DIST)
        ffi_lib.syncemitter_set_stepper_kinematics(se, self.sk)

    def append(
        self,
        print_time,
        accel_t,
        cruise_t,
        decel_t,
        start_pos,
        axes_r,
        start_v,
        cruise_v,
        accel,
    ):
        self.ffi_lib.trapq_append(
            self.trapq,
            print_time,
            accel_t,
            cruise_t,
            decel_t,
            start_pos[0],
            start_pos[1],
            start_pos[2],
            axes_r[0],
            axes_r[1],
            axes_r[2],
            start_v,
            cruise_v,
            accel,
        )

    def generate(self, end_time):
        # Generate well past the profile so the trailing smoothing window is
        # complete and nothing is truncated (V4).
        flush_to = end_time + 1.0
        assert (
            self.ffi_lib.steppersyncmgr_gen_steps(
                self.ssm, flush_to, flush_to, 0.0
            )
            == 0
        )

    def position_at(self, print_time):
        clock = int(print_time * MCU_FREQ)
        return (
            self.ffi_lib.stepcompress_find_past_position(self.sc, clock)
            * STEP_DIST
        )

    def close(self):
        self.ffi_lib.serialqueue_exit(self.serialqueue)
        os.close(self.read_fd)


def _append_profile(rig, segments, start_time, channel):
    """Write `segments` into `rig`, in the given extruder-queue channel.

    channel 0 is what a profile move uses (and what an axis-aligned +X
    printing move produces); channel 2 is where an extrude-only move is
    parked today.
    """
    print_time = start_time
    position = 0.0
    for seg in segments:
        direction = seg.direction()
        speed_start, speed_end = abs(seg.v_start), abs(seg.v_end)
        accel_t = cruise_t = decel_t = 0.0
        if speed_end > speed_start:
            accel_t, start_v, cruise_v = seg.move_t, speed_start, speed_end
        elif speed_end < speed_start:
            decel_t, start_v, cruise_v = seg.move_t, speed_start, speed_start
        else:
            cruise_t, start_v, cruise_v = seg.move_t, speed_start, speed_start
        start_pos = [0.0, 0.0, 0.0]
        axes_r = [0.0, 0.0, 0.0]
        start_pos[channel] = position
        axes_r[channel] = direction
        rig.append(
            print_time,
            accel_t,
            cruise_t,
            decel_t,
            start_pos,
            axes_r,
            start_v,
            cruise_v,
            abs(seg.accel),
        )
        print_time += seg.move_t
        position += seg.distance
    return print_time


def _sample_times(segments, start_time, n=60):
    total = sum(s.move_t for s in segments)
    return [start_time + total * i / float(n) for i in range(1, n)]


# ----------------------------------------------------------------------
# Independent closed-form reference (the second oracle for V1)
# ----------------------------------------------------------------------


def _nominal_velocity(segments, start_time, t):
    """Signed nominal (uncompensated) extruder velocity at time t."""
    edge = start_time
    for seg in segments:
        if t < edge + seg.move_t:
            return seg.v_start + seg.accel * (t - edge)
        edge += seg.move_t
    return 0.0


def _nominal_position(segments, start_time, t):
    edge = start_time
    pos = 0.0
    for seg in segments:
        if t < edge + seg.move_t:
            dt = t - edge
            return pos + seg.v_start * dt + 0.5 * seg.accel * dt * dt
        pos += seg.distance
        edge += seg.move_t
    return pos


def _smoothing_weight(dt, smooth_time):
    half = 0.5 * smooth_time
    if abs(dt) > half:
        return 0.0
    u = dt / smooth_time
    return (15.0 / 8.0 - 15.0 * u**2 + 30.0 * u**4) / smooth_time


def _smoothed_velocity(segments, start_time, t, smooth_time, steps=4001):
    """Numerically convolve the nominal velocity with Kalico's kernel.

    Deliberately a different method from the C helper's closed-form
    per-move antiderivatives, so agreement is evidence and not a shared
    derivation.  Retracting phases contribute nothing, which is how the C
    code's per-move `can_pressure_advance` gate behaves.
    """
    half = 0.5 * smooth_time
    total = 0.0
    lo, hi = t - half, t + half
    h = (hi - lo) / (steps - 1)
    for i in range(steps):
        tau = lo + i * h
        v = _nominal_velocity(segments, start_time, tau)
        if v < 0.0:
            v = 0.0
        w = _smoothing_weight(tau - t, smooth_time)
        weight = 0.5 if i in (0, steps - 1) else 1.0
        total += weight * v * w * h
    return total


def _reference_position(
    segments, start_time, t, advance, smooth_time=SMOOTH_TIME, time_offset=0.0
):
    shifted = t + time_offset
    position = _nominal_position(segments, start_time, shifted)
    pa_velocity = _smoothed_velocity(segments, start_time, shifted, smooth_time)
    if pa_velocity <= 0.0:
        return position
    return position + advance * pa_velocity


# ----------------------------------------------------------------------
# Fixtures / profiles
# ----------------------------------------------------------------------

TRAPEZOID = "V0=0:V1=5:ACCEL=500, V1=5:DISTANCE=4, V1=0:ACCEL=500"
SPEED_STEPS = (
    "V0=0:V1=5:ACCEL=500, V1=5:DURATION=0.15,"
    " V1=2:ACCEL=500, V1=2:DURATION=0.15,"
    " V1=5:ACCEL=500, V1=5:DURATION=0.15, V1=0:ACCEL=500"
)


@pytest.fixture
def ffi():
    return chelper.get_ffi()


def _run(ffi, segments, channel, advance, **kwargs):
    ffi_main, ffi_lib = ffi
    rig = _Rig(ffi_main, ffi_lib, advance, **kwargs)
    try:
        end_time = _append_profile(rig, segments, START_TIME, channel)
        rig.generate(end_time)
        return [
            (t, rig.position_at(t)) for t in _sample_times(segments, START_TIME)
        ]
    finally:
        rig.close()


# ----------------------------------------------------------------------
# The gate
# ----------------------------------------------------------------------


@pytest.mark.parametrize("profile", [TRAPEZOID, SPEED_STEPS])
def test_profile_channel_receives_pressure_advance(ffi, profile):
    segments = parse_profile(profile)
    advance = 0.05
    with_pa = _run(ffi, segments, 0, advance)
    without_pa = _run(ffi, segments, 0, 0.0)
    diffs = [a[1] - b[1] for a, b in zip(with_pa, without_pa)]
    # Advance is present, positive, and of the expected magnitude while the
    # extruder is up to speed.
    assert max(diffs) > 0.5 * advance * 5.0
    assert min(diffs) >= -1e-6


@pytest.mark.parametrize("profile", [TRAPEZOID, SPEED_STEPS])
def test_extrude_only_channel_receives_none(ffi, profile):
    """The behaviour this whole feature exists to route around.

    If this ever fails, upstream has started compensating extrude-only
    moves and [extruder_profile] may no longer be needed -- but it also
    means its channel-layout assumption has changed, so read it before
    deleting this test.
    """
    segments = parse_profile(profile)
    with_pa = _run(ffi, segments, 2, 0.05)
    without_pa = _run(ffi, segments, 2, 0.0)
    for (t, a), (_, b) in zip(with_pa, without_pa):
        assert a == pytest.approx(b, abs=2 * STEP_DIST), (
            "pressure advance leaked into the extrude-only channel at %.4f" % t
        )


# ----------------------------------------------------------------------
# V2: equivalence with printing
# ----------------------------------------------------------------------


def _append_printing(rig, segments, start_time, xy_length):
    """Write the extruder queue entries an equivalent +X printing move makes.

    Goes the long way round on purpose: a toolhead move along +X of length
    `xy_length` that extrudes the same filament as `segments`, pushed
    through PrinterExtruder.process_move()'s own arithmetic --

        axis_r  = e_distance / xy_length          (move.axes_r[ea_index])
        accel   = move.accel   * abs(axis_r)
        start_v = move.start_v * abs(axis_r)
        extr_r  = [copysign(r*r, axis_r) for r in move.axes_r[:3]]

    -- so the toolhead speeds are scaled up by 1/axis_r and then scaled back
    down by the real formula.  With xy_length deliberately not equal to the
    extruded length, axis_r is a non-round number and the round trip is a
    genuine exercise of that path rather than a restatement of it.
    """
    e_distance = sum(abs(s.distance) for s in segments)
    axis_r = e_distance / xy_length
    assert axis_r != 1.0, "pick an xy_length that makes axis_r non-trivial"
    print_time = start_time
    position = 0.0
    for seg in segments:
        direction = seg.direction()
        speed_start, speed_end = abs(seg.v_start), abs(seg.v_end)
        # Toolhead-frame speeds for this phase.
        th_start_v = speed_start / axis_r
        th_cruise_v = speed_end / axis_r
        th_accel = abs(seg.accel) / axis_r
        signed_axis_r = math.copysign(axis_r, direction)
        # What process_move() derives from them.
        accel = th_accel * abs(signed_axis_r)
        start_v = th_start_v * abs(signed_axis_r)
        cruise_v = th_cruise_v * abs(signed_axis_r)
        extr_r = [math.copysign(r * r, signed_axis_r) for r in (1.0, 0.0, 0.0)]
        # copysign(0.0, -1) is -0.0, which is still a zero channel.
        assert extr_r[0] == direction
        accel_t = cruise_t = decel_t = 0.0
        if speed_end > speed_start:
            accel_t = seg.move_t
        elif speed_end < speed_start:
            decel_t = seg.move_t
            cruise_v = start_v
        else:
            cruise_t = seg.move_t
            cruise_v = start_v
        rig.append(
            print_time,
            accel_t,
            cruise_t,
            decel_t,
            [position, 0.0, 0.0],
            extr_r,
            start_v,
            cruise_v,
            accel,
        )
        print_time += seg.move_t
        position += seg.distance
    return print_time


@pytest.mark.parametrize("profile", [TRAPEZOID, SPEED_STEPS])
def test_matches_equivalent_x_printing_move(ffi, profile):
    """V2: a profile move equals an equivalent axis-aligned +X print move.

    Both sides go through the real C helper; only the way the extruder
    queue entries were derived differs.
    """
    segments = parse_profile(profile)
    advance = 0.04
    profile_run = _run(ffi, segments, 0, advance)

    ffi_main, ffi_lib = ffi
    rig = _Rig(ffi_main, ffi_lib, advance)
    try:
        end_time = _append_printing(rig, segments, START_TIME, xy_length=37.0)
        rig.generate(end_time)
        printing_run = [
            (t, rig.position_at(t)) for t in _sample_times(segments, START_TIME)
        ]
    finally:
        rig.close()

    assert len(profile_run) == len(printing_run)
    for (t, a), (_, b) in zip(profile_run, printing_run):
        assert a == pytest.approx(b, abs=2 * STEP_DIST), (
            "profile and equivalent printing move diverged at %.4f:"
            " %.6f vs %.6f" % (t, a, b)
        )


# ----------------------------------------------------------------------
# V1: numerical equivalence with an independent reference
# ----------------------------------------------------------------------


@pytest.mark.parametrize(
    "profile,advance,smooth_time,time_offset",
    [
        (TRAPEZOID, 0.04, 0.040, 0.0),
        (TRAPEZOID, 0.04, 0.020, 0.0),
        (TRAPEZOID, 0.04, 0.040, 0.010),
        (TRAPEZOID, 0.04, 0.040, -0.010),
        (SPEED_STEPS, 0.03, 0.040, 0.0),
        (SPEED_STEPS, 0.03, 0.020, 0.005),
    ],
)
def test_matches_independent_reference(
    ffi, profile, advance, smooth_time, time_offset
):
    segments = parse_profile(profile)
    got = _run(
        ffi,
        segments,
        0,
        advance,
        smooth_time=smooth_time,
        time_offset=time_offset,
    )
    # Step quantisation plus the stepcompress error budget; the reference is
    # continuous, the rig emits whole steps.
    tol = 4 * STEP_DIST + 0.000025 * 5.0
    for t, position in got:
        expected = _reference_position(
            segments,
            START_TIME,
            t,
            advance,
            smooth_time=smooth_time,
            time_offset=time_offset,
        )
        assert position == pytest.approx(expected, abs=tol), (
            "at t=%.4f: got %.6f, reference %.6f" % (t, position, expected)
        )


# ----------------------------------------------------------------------
# R3: retraction
# ----------------------------------------------------------------------


def test_retraction_segment_gets_no_advance(ffi):
    segments = parse_profile(
        "V0=0:V1=-5:ACCEL=500, V1=-5:DISTANCE=-1, V1=0:ACCEL=500"
    )
    with_pa = _run(ffi, segments, 0, 0.05)
    without_pa = _run(ffi, segments, 0, 0.0)
    for (t, a), (_, b) in zip(with_pa, without_pa):
        assert a == pytest.approx(b, abs=2 * STEP_DIST), (
            "advance applied to a retraction at %.4f" % t
        )


def test_retraction_then_prime_advances_only_on_the_prime(ffi):
    segments = parse_profile(
        "V0=0:V1=-5:ACCEL=500, V1=-5:DISTANCE=-1, V1=0:ACCEL=500,"
        " V1=5:ACCEL=500, V1=5:DISTANCE=4, V1=0:ACCEL=500"
    )
    advance = 0.05
    with_pa = _run(ffi, segments, 0, advance)
    without_pa = _run(ffi, segments, 0, 0.0)
    retract_t = sum(s.move_t for s in segments[:3])
    diffs_retract = [
        a[1] - b[1]
        for a, b in zip(with_pa, without_pa)
        if a[0] < START_TIME + retract_t - 0.5 * SMOOTH_TIME
    ]
    diffs_prime = [
        a[1] - b[1]
        for a, b in zip(with_pa, without_pa)
        if a[0] > START_TIME + retract_t + 0.5 * SMOOTH_TIME
    ]
    assert diffs_retract and max(abs(d) for d in diffs_retract) < 2 * STEP_DIST
    assert diffs_prime and max(diffs_prime) > 0.5 * advance * 5.0


# ----------------------------------------------------------------------
# V4: no truncated compensation at the edges or at joints
# ----------------------------------------------------------------------


def test_compensation_is_continuous_across_joints(ffi):
    """No step in the compensated position where segments meet.

    The joints of SPEED_STEPS carry non-zero speed, so a missing
    look-behind or look-ahead in the smoothing window would show up as a
    discontinuity in the generated trajectory rather than as a wrong value.
    """
    segments = parse_profile(SPEED_STEPS)
    ffi_main, ffi_lib = ffi
    rig = _Rig(ffi_main, ffi_lib, 0.05)
    try:
        end_time = _append_profile(rig, segments, START_TIME, 0)
        rig.generate(end_time)
        total = sum(s.move_t for s in segments)
        times = [START_TIME + total * i / 400.0 for i in range(1, 400)]
        positions = [rig.position_at(t) for t in times]
    finally:
        rig.close()
    dt = total / 400.0
    velocities = [(b - a) / dt for a, b in zip(positions, positions[1:])]
    # The commanded velocity stays bounded: with PA the peak is the nominal
    # peak plus the advance term's contribution, never a spike from a
    # truncated window.
    assert max(velocities) < 5.0 + 0.05 * 500.0 + 2.0
    assert min(velocities) > -2.0
    assert positions == sorted(positions), "profile moved backwards"
