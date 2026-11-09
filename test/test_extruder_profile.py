"""Profile parsing and segment arithmetic for [extruder_profile].

Pure Python: no chelper, no printer objects, so these run anywhere.  The
numerical-equivalence and pressure-advance behaviour tests live in
test_extruder_profile_equivalence.py, which needs the compiled C helper.
"""

import math

import pytest

from klippy.extras.extruder_profile import (
    MAX_RETRACT_DISTANCE,
    ProfileError,
    build_segment,
    parse_profile,
)


def _total_distance(segments):
    return sum(s.distance for s in segments)


# ----------------------------------------------------------------------
# build_segment
# ----------------------------------------------------------------------


def test_accel_segment_from_accel():
    seg = build_segment(0.0, 5.0, accel=500.0)
    assert seg.move_t == pytest.approx(0.01)
    assert seg.distance == pytest.approx(0.025)
    assert seg.accel == pytest.approx(500.0)


def test_decel_segment_accel_is_signed_but_magnitude_given():
    seg = build_segment(5.0, 2.0, accel=500.0)
    assert seg.move_t == pytest.approx(0.006)
    assert seg.accel == pytest.approx(-500.0)
    assert seg.distance == pytest.approx(0.021)


def test_cruise_segment_from_distance():
    seg = build_segment(5.0, 5.0, distance=10.0)
    assert seg.move_t == pytest.approx(2.0)
    assert seg.accel == pytest.approx(0.0)
    assert seg.distance == pytest.approx(10.0)


def test_cruise_segment_from_duration():
    seg = build_segment(4.0, 4.0, duration=0.5)
    assert seg.distance == pytest.approx(2.0)


def test_segment_from_duration_derives_accel():
    seg = build_segment(2.0, 6.0, duration=0.02)
    assert seg.accel == pytest.approx(200.0)
    assert seg.distance == pytest.approx(0.08)


def test_retraction_speeding_up_is_accepted():
    seg = build_segment(0.0, -5.0, accel=500.0)
    assert seg.distance == pytest.approx(-0.025)
    assert seg.direction() == -1.0


def test_retraction_slowing_down_is_accepted():
    seg = build_segment(-5.0, 0.0, accel=500.0)
    assert seg.distance == pytest.approx(-0.025)
    assert seg.direction() == -1.0


def test_segment_may_not_change_direction():
    with pytest.raises(ProfileError, match="may not change direction"):
        build_segment(5.0, -5.0, accel=500.0)


def test_segment_needs_exactly_one_of_accel_duration_distance():
    with pytest.raises(ProfileError, match="exactly one of"):
        build_segment(0.0, 5.0)
    with pytest.raises(ProfileError, match="exactly one of"):
        build_segment(0.0, 5.0, accel=500.0, duration=0.01)


def test_accel_cannot_describe_a_constant_velocity_segment():
    with pytest.raises(ProfileError, match="constant-velocity"):
        build_segment(5.0, 5.0, accel=500.0)


def test_distance_cannot_describe_a_zero_mean_velocity_segment():
    # A segment may not change direction, so the only zero-mean segment
    # left is the degenerate 0 -> 0 one.
    with pytest.raises(ProfileError, match="mean velocity is"):
        build_segment(0.0, 0.0, distance=1.0)


def test_distance_sign_must_match_direction():
    with pytest.raises(ProfileError, match="opposite sign"):
        build_segment(5.0, 5.0, distance=-10.0)


def test_negative_accel_magnitude_is_rejected():
    with pytest.raises(ProfileError, match="must be positive"):
        build_segment(0.0, 5.0, accel=-500.0)


def test_zero_duration_is_rejected():
    with pytest.raises(ProfileError, match="DURATION must be positive"):
        build_segment(0.0, 5.0, duration=0.0)


# ----------------------------------------------------------------------
# parse_profile
# ----------------------------------------------------------------------

TRAPEZOID = "V0=0:V1=5:ACCEL=500, V1=5:DISTANCE=10, V1=0:ACCEL=500"


def test_parse_trapezoid():
    segs = parse_profile(TRAPEZOID)
    assert len(segs) == 3
    assert segs[0].v_start == 0.0 and segs[0].v_end == 5.0
    # V0 defaults to the previous segment's V1, which is the point: joint
    # speeds are non-zero without the caller having to restate them.
    assert segs[1].v_start == 5.0 and segs[1].v_end == 5.0
    assert segs[2].v_start == 5.0 and segs[2].v_end == 0.0
    assert _total_distance(segs) == pytest.approx(10.05)


def test_parse_speed_changes_with_non_zero_joints():
    segs = parse_profile(
        "V0=0:V1=5:ACCEL=500, V1=5:DURATION=0.2,"
        " V1=2:ACCEL=500, V1=2:DURATION=0.2,"
        " V1=5:ACCEL=500, V1=5:DURATION=0.2, V1=0:ACCEL=500"
    )
    assert len(segs) == 7
    joints = [(s.v_start, s.v_end) for s in segs]
    assert joints[2] == (5.0, 2.0)
    assert joints[4] == (2.0, 5.0)
    # Every interior joint is continuous.
    for prev, nxt in zip(segs, segs[1:]):
        assert prev.v_end == nxt.v_start
    # And nothing in the middle stops.
    assert all(s.v_end != 0.0 for s in segs[:-1])


def test_parse_requires_v1():
    with pytest.raises(ProfileError, match="needs V1"):
        parse_profile("V0=0:ACCEL=500")


def test_parse_rejects_unknown_field():
    with pytest.raises(ProfileError, match="unknown field"):
        parse_profile("V0=0:V1=5:ACCEL=500:SPEED=3")


def test_parse_rejects_non_numeric():
    with pytest.raises(ProfileError, match="non-numeric"):
        parse_profile("V0=0:V1=fast:ACCEL=500")


def test_parse_rejects_duplicate_field():
    with pytest.raises(ProfileError, match="twice"):
        parse_profile("V0=0:V1=5:V1=6:ACCEL=500")


def test_parse_rejects_field_without_equals():
    with pytest.raises(ProfileError, match="without '='"):
        parse_profile("V0=0:V1=5:ACCEL")


def test_parse_rejects_empty():
    with pytest.raises(ProfileError, match="empty"):
        parse_profile("   ")


def test_profile_must_start_at_rest():
    with pytest.raises(ProfileError, match="start at rest"):
        parse_profile("V0=5:V1=5:DISTANCE=10, V1=0:ACCEL=500")


def test_profile_must_end_at_rest():
    with pytest.raises(ProfileError, match="end at rest"):
        parse_profile("V0=0:V1=5:ACCEL=500, V1=5:DISTANCE=10")


def test_parse_tolerates_whitespace_and_trailing_comma():
    segs = parse_profile("  V0=0 : V1=5 : ACCEL=500 ,  V1=0:ACCEL=500 ,  ")
    assert len(segs) == 2


def test_parse_is_case_insensitive_on_keys():
    segs = parse_profile("v0=0:v1=5:accel=500, v1=0:accel=500")
    assert len(segs) == 2


# ----------------------------------------------------------------------
# Limit checking (R11-R13), against a minimal stand-in extruder
# ----------------------------------------------------------------------


class _Heater:
    def __init__(self, can_extrude=True):
        self.can_extrude = can_extrude


class _Extruder:
    def __init__(self, can_extrude=True, max_v=100.0, max_a=5000.0, max_d=50.0):
        self._heater = _Heater(can_extrude)
        self.max_e_velocity = max_v
        self.max_e_accel = max_a
        self.max_e_dist = max_d

    def get_heater(self):
        return self._heater

    def get_name(self):
        return "extruder"


class _Checker:
    """Just the limit checker, without constructing the whole extra."""

    max_run_distance = 100.0
    max_run_duration = 120.0

    from klippy.extras.extruder_profile import ExtruderProfile as _EP

    _check_limits = _EP._check_limits


def _check(segments, **kwargs):
    return _Checker()._check_limits(_Extruder(**kwargs), segments)


def test_limits_accept_a_sane_profile():
    total_t, forward_d, net_d = _check(parse_profile(TRAPEZOID))
    assert forward_d == pytest.approx(10.05)
    assert net_d == pytest.approx(10.05)
    assert total_t == pytest.approx(0.02 + 2.0)


def test_limits_reject_below_min_extrude_temp():
    with pytest.raises(ProfileError, match="below minimum temp"):
        _check(parse_profile(TRAPEZOID), can_extrude=False)


def test_limits_reject_over_velocity():
    with pytest.raises(ProfileError, match="max_extrude_only_velocity"):
        _check(parse_profile(TRAPEZOID), max_v=4.0)


def test_limits_reject_over_accel():
    with pytest.raises(ProfileError, match="max_extrude_only_accel"):
        _check(parse_profile(TRAPEZOID), max_a=100.0)


def test_limits_reject_over_extrude_only_distance():
    with pytest.raises(ProfileError, match="max_extrude_only_distance"):
        _check(parse_profile(TRAPEZOID), max_d=1.0)


def test_limits_reject_over_run_distance():
    segs = parse_profile(
        "V0=0:V1=5:ACCEL=500, V1=5:DISTANCE=200, V1=0:ACCEL=500"
    )
    with pytest.raises(ProfileError, match="max_run_distance"):
        _check(segs, max_d=1000.0)


def test_limits_reject_over_run_duration():
    checker = _Checker()
    checker.max_run_duration = 0.5
    segs = parse_profile(TRAPEZOID)
    with pytest.raises(ProfileError, match="max_run_duration"):
        checker._check_limits(_Extruder(), segs)


def test_limits_reject_long_retraction():
    segs = parse_profile(
        "V0=0:V1=-5:ACCEL=500, V1=-5:DISTANCE=-3, V1=0:ACCEL=500,"
        " V1=5:ACCEL=500, V1=5:DISTANCE=10, V1=0:ACCEL=500"
    )
    with pytest.raises(
        ProfileError, match="at most %.1fmm" % MAX_RETRACT_DISTANCE
    ):
        _check(segs)


def test_limits_accept_short_retraction_that_is_primed_back():
    segs = parse_profile(
        "V0=0:V1=-5:ACCEL=500, V1=-5:DISTANCE=-1, V1=0:ACCEL=500,"
        " V1=5:ACCEL=500, V1=5:DISTANCE=10, V1=0:ACCEL=500"
    )
    total_t, forward_d, net_d = _check(segs)
    assert net_d > 0.0
    assert forward_d == pytest.approx(10.05)


def test_limits_reject_a_profile_left_retracted():
    segs = parse_profile(
        "V0=0:V1=-5:ACCEL=500, V1=-5:DISTANCE=-1, V1=0:ACCEL=500"
    )
    with pytest.raises(ProfileError, match="primed back"):
        _check(segs)


def test_retraction_distance_is_per_segment_not_cumulative():
    # Two retractions of 1.5mm each: each is under the cap, so this passes
    # even though they sum to 3mm, because the hotend only ever sees 1.5mm
    # of filament pulled back at a time.
    segs = parse_profile(
        "V0=0:V1=-5:ACCEL=500, V1=-5:DISTANCE=-1.5, V1=0:ACCEL=500,"
        " V1=5:ACCEL=500, V1=5:DISTANCE=3, V1=0:ACCEL=500,"
        " V1=-5:ACCEL=500, V1=-5:DISTANCE=-1.5, V1=0:ACCEL=500,"
        " V1=5:ACCEL=500, V1=5:DISTANCE=10, V1=0:ACCEL=500"
    )
    _, _, net_d = _check(segs)
    assert net_d > 0.0


def test_all_limits_are_checked_before_anything_moves():
    # The checker is a pure function of (extruder, segments): it is called
    # in run_profile() before the first set_trapq(), so a rejected profile
    # cannot have moved the extruder.  Guard that it stays side-effect free.
    segs = parse_profile(TRAPEZOID)
    before = [(s.v_start, s.v_end, s.move_t, s.distance) for s in segs]
    with pytest.raises(ProfileError):
        _check(segs, max_v=1.0)
    after = [(s.v_start, s.v_end, s.move_t, s.distance) for s in segs]
    assert before == after


def test_segment_distance_matches_closed_form():
    # distance == integral of v dt for a constant-acceleration phase.
    for v0, v1, dur in (
        (0.0, 5.0, 0.01),
        (5.0, 2.0, 0.006),
        (2.0, 2.0, 0.5),
        (0.0, -5.0, 0.01),
        (-5.0, 0.0, 0.01),
    ):
        seg = build_segment(v0, v1, duration=dur)
        expected = 0.5 * (v0 + v1) * dur
        assert seg.distance == pytest.approx(expected)
        assert math.isfinite(seg.accel)
