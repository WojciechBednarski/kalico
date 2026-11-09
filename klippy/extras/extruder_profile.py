# Extruder-only "profile moves" with pressure advance applied
#
# Drives the extruder alone through a caller-specified speed profile, with
# non-zero speeds at the segment joints, and with pressure advance applied
# exactly as it is applied to a printing move.  Intended for verifying PA
# settings against a force/pressure sensor without printing anything.
#
# Copyright (C) 2026  Wojciech Bednarski
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# How this gets pressure advance without touching the normal move path
# --------------------------------------------------------------------
# Kalico applies PA inside the extruder's own motion queue.  The gate lives
# in klippy/chelper/kin_extruder.c, in pa_move_integrate():
#
#     int can_pressure_advance = m->axes_r.x > 0. || m->axes_r.y > 0.;
#
# where `m` is a move in the *extruder* trapq, not a toolhead move.  The
# three coordinate channels of that queue are bookkeeping: they carry the
# extruder motion split per toolhead axis so the per-axis input shaper can
# be applied to it (kin_extruder.c extruder_calc_position() sums all three).
# PrinterExtruder.process_move() fills them in; for a kinematic move it
# writes copysign(r_i**2, axis_r) per axis, and for an extrude-only move it
# writes [0, 0, axis_r] -- which is why extrude-only moves get no PA today.
#
# A +X printing move whose motion is entirely along X has r_x == 1 and
# therefore axes_r == (+1, 0, 0) in the extruder queue.  So appending a pure
# extruder segment with axes_r == (+1, 0, 0) is not a trick or a special
# case: it is byte-for-byte the encoding an axis-aligned +X printing move
# already produces, and the existing C code applies PA to it for exactly the
# same reason.  No change to the C helper, and no change to how ordinary
# moves are classified.
#
# Retraction segments are appended with axes_r == (-1, 0, 0), which fails
# the gate above -- the same rule printing follows for a retracting move.
#
# Everything else follows the pattern force_move.manual_move() established:
# a private trapq owned by this module, the extruder steppers pointed at it
# for the duration, and the toolhead told about the occupied time window.
# Unlike force_move this keeps each stepper's own kinematics object, since
# that object is what carries the PA parameters and the smoothers.
import inspect
import logging
import math
import os
import re

from klippy import chelper

# Bumped when the command/Python interface changes in a way a caller can see.
API_VERSION = 1

# The Kalico commit this module's assumptions were verified against.  Reported
# by EXTRUDER_PROFILE_STATUS so a caller can tell whether it is running on the
# tree the feature was tested on.
VERIFIED_AGAINST = "3df76e7c8"

# Channel of the extruder motion queue the profile is written to.  Must be 0
# (x) or 1 (y) for the C gate quoted above to accept it.
PA_CHANNEL = 0

# A segment shorter than this is treated as zero-length and dropped: the trapq
# stores times, and sub-microsecond phases only add numerical noise.
MIN_SEGMENT_T = 1e-6

# Hard ceiling on a single retraction segment, in mm of filament.  A
# direct-drive hotend can clog on more than this.
MAX_RETRACT_DISTANCE = 2.0


class ProfileError(Exception):
    """A malformed profile.  Translated to a command error by the callers."""


class Segment:
    """One constant-acceleration phase of a profile, in filament mm.

    `v_start` and `v_end` are signed: positive extrudes, negative retracts.
    A segment never changes sign; the parser splits such a request.
    """

    def __init__(self, v_start, v_end, accel, move_t, distance):
        self.v_start = v_start
        self.v_end = v_end
        self.accel = accel
        self.move_t = move_t
        self.distance = distance

    def direction(self):
        if self.v_start < 0.0 or self.v_end < 0.0:
            return -1.0
        return 1.0

    def as_dict(self, start_time):
        return {
            "start_time": start_time,
            "end_time": start_time + self.move_t,
            "v_start": self.v_start,
            "v_end": self.v_end,
            "accel": self.accel,
            "duration": self.move_t,
            "distance": self.distance,
        }


def build_segment(v_start, v_end, accel=None, duration=None, distance=None):
    """Build one Segment from any two of (accel, duration, distance).

    All inputs are E-axis filament units: mm/s and mm/s**2.  Extruder
    acceleration is never derived from a toolhead/XY acceleration.
    """
    if (v_start < 0.0) != (v_end < 0.0) and v_start != 0.0 and v_end != 0.0:
        raise ProfileError(
            "A segment may not change direction (%.4f -> %.4f mm/s);"
            " split it into two segments through zero" % (v_start, v_end)
        )
    given = [
        k
        for k, v in (
            ("ACCEL", accel),
            ("DURATION", duration),
            ("DISTANCE", distance),
        )
        if v is not None
    ]
    if len(given) != 1:
        raise ProfileError(
            "A segment needs exactly one of ACCEL, DURATION or DISTANCE,"
            " got %s" % (", ".join(given) if given else "none",)
        )
    dv = v_end - v_start
    mean_v = 0.5 * (v_start + v_end)
    if duration is not None:
        if duration <= 0.0:
            raise ProfileError("DURATION must be positive")
        move_t = duration
        accel = dv / move_t
    elif distance is not None:
        if mean_v == 0.0:
            raise ProfileError(
                "DISTANCE cannot determine a segment whose mean velocity is"
                " zero (%.4f -> %.4f mm/s); give DURATION instead"
                % (v_start, v_end)
            )
        if (distance < 0.0) != (mean_v < 0.0):
            raise ProfileError(
                "DISTANCE %.4fmm has the opposite sign to the segment's"
                " mean velocity %.4f mm/s" % (distance, mean_v)
            )
        move_t = distance / mean_v
        accel = dv / move_t
    else:
        if dv == 0.0:
            raise ProfileError(
                "ACCEL cannot determine a constant-velocity segment"
                " (%.4f mm/s); give DURATION or DISTANCE instead" % (v_start,)
            )
        if accel <= 0.0:
            raise ProfileError("ACCEL must be positive (it is a magnitude)")
        move_t = abs(dv) / accel
        accel = dv / move_t
    distance = (v_start + 0.5 * dv) * move_t
    return Segment(v_start, v_end, accel, move_t, distance)


class ExtruderProfile:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.max_run_distance = config.getfloat(
            "max_run_distance", 100.0, above=0.0
        )
        self.max_run_duration = config.getfloat(
            "max_run_duration", 120.0, above=0.0
        )
        self.motion_queuing = self.printer.load_object(config, "motion_queuing")
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        # Result of the startup self-check (R8).  None until klippy:connect.
        self.disabled_reason = None
        self.last_run = None
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "EXTRUDER_PROFILE_MOVE",
            self.cmd_EXTRUDER_PROFILE_MOVE,
            desc=self.cmd_EXTRUDER_PROFILE_MOVE_help,
        )
        gcode.register_command(
            "EXTRUDER_PROFILE_STATUS",
            self.cmd_EXTRUDER_PROFILE_STATUS,
            desc=self.cmd_EXTRUDER_PROFILE_STATUS_help,
        )

    # ------------------------------------------------------------------
    # Startup self-check
    # ------------------------------------------------------------------

    def _handle_connect(self):
        try:
            self.disabled_reason = self._self_check()
        except Exception as e:
            # Never prevent klippy from starting (R8).
            self.disabled_reason = "self-check raised %s" % (e,)
        if self.disabled_reason is not None:
            logging.warning(
                "extruder_profile: disabled, %s", self.disabled_reason
            )

    def _self_check(self):
        """Verify the hook points this module relies on still exist.

        Checked behaviourally where possible rather than by inspecting
        source, so a cosmetic upstream edit does not trip it and a
        semantic one does.  Returns None when usable, else a reason.
        """
        ffi_main, ffi_lib = chelper.get_ffi()
        for name in (
            "extruder_stepper_alloc",
            "extruder_set_smoothing_params",
            "extruder_set_pressure_advance",
            "itersolve_calc_position_from_coord",
        ):
            if not hasattr(ffi_lib, name):
                return "chelper is missing %s()" % (name,)
        for name in (
            "allocate_trapq",
            "lookup_trapq_append",
            "wipe_trapq",
            "note_mcu_movequeue_activity",
        ):
            if not hasattr(self.motion_queuing, name):
                return "motion_queuing is missing %s()" % (name,)
        toolhead = self.printer.lookup_object("toolhead")
        for name in ("flush_step_generation", "get_last_move_time", "dwell"):
            if not hasattr(toolhead, name):
                return "toolhead is missing %s()" % (name,)
        # The load-bearing assumptions about code this module does not own.
        # Proving them behaviourally needs offline step generation (a
        # steppersync, a stepcompress and a serialqueue), which is far too
        # much to set up during klippy startup; the test suite does that
        # instead -- see test/test_extruder_profile_equivalence.py.  What is
        # checked here is that the two hook points still have the shape the
        # module was written against, so an upstream change is caught before
        # it can silently produce uncompensated motion.
        return self._check_hook_shapes()

    def _check_hook_shapes(self):
        # 1. The C gate that decides whether a move in the extruder queue
        #    receives pressure advance, in pa_move_integrate().
        kin_extruder = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "chelper",
            "kin_extruder.c",
        )
        try:
            with open(kin_extruder) as f:
                source = f.read()
        except OSError as e:
            return "cannot read kin_extruder.c (%s)" % (e,)
        gate = re.compile(
            r"can_pressure_advance\s*=\s*m->axes_r\.x\s*>\s*0\.?\s*\|\|"
            r"\s*m->axes_r\.y\s*>\s*0\.?"
        )
        if not gate.search(source):
            return (
                "kin_extruder.c no longer gates pressure advance on"
                " 'm->axes_r.x > 0. || m->axes_r.y > 0.'; a profile move"
                " would not be compensated the way this module assumes"
            )
        # 2. The Python encoding this module mirrors: an extrude-only move
        #    parks its motion in the third channel, which is what makes the
        #    first channel free for a profile move to claim.
        try:
            import klippy.kinematics.extruder as extruder_mod

            process_move = inspect.getsource(
                extruder_mod.PrinterExtruder.process_move
            )
        except Exception as e:
            return "cannot inspect PrinterExtruder.process_move (%s)" % (e,)
        if not re.search(
            r"extr_r\s*=\s*\[\s*0\.0\s*,\s*0\.0\s*,\s*axis_r\s*\]",
            process_move,
        ):
            return (
                "PrinterExtruder.process_move() no longer encodes an"
                " extrude-only move as [0.0, 0.0, axis_r]; the channel"
                " layout this module relies on has changed"
            )
        return None

    def _handle_shutdown(self):
        # Nothing is left behind on shutdown: a run in progress is unwound by
        # its own finally block, and this only drops queued moves.
        try:
            self.motion_queuing.wipe_trapq(self.trapq)
        except Exception:
            logging.exception("extruder_profile: wipe_trapq on shutdown")

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def _lookup_extruder(self, extruder_name):
        if extruder_name is None:
            toolhead = self.printer.lookup_object("toolhead")
            extruder = toolhead.get_extruder()
            if not hasattr(extruder, "get_extruder_steppers"):
                raise ProfileError("No extruder is active")
            return extruder
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None or not hasattr(extruder, "get_extruder_steppers"):
            raise ProfileError(
                "'%s' is not a valid extruder" % (extruder_name,)
            )
        return extruder

    def _check_idle(self):
        vsd = self.printer.lookup_object("virtual_sdcard", None)
        if vsd is not None and vsd.is_active():
            raise ProfileError(
                "Refusing to run a profile while a print is active"
            )
        stats = self.printer.lookup_object("print_stats", None)
        if stats is not None:
            state = stats.get_status(
                self.printer.get_reactor().monotonic()
            ).get("state")
            if state in ("printing", "paused"):
                raise ProfileError(
                    "Refusing to run a profile while a print is %s" % (state,)
                )

    def _check_limits(self, extruder, segments):
        """Check the WHOLE profile before anything moves (R11-R13)."""
        heater = extruder.get_heater()
        if not heater.can_extrude:
            raise ProfileError(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details"
            )
        max_v = extruder.max_e_velocity
        max_a = extruder.max_e_accel
        max_d = extruder.max_e_dist
        total_t = 0.0
        forward_d = 0.0
        net_d = 0.0
        for i, seg in enumerate(segments):
            peak_v = max(abs(seg.v_start), abs(seg.v_end))
            if peak_v > max_v:
                raise ProfileError(
                    "Segment %d reaches %.3f mm/s, above the extruder's"
                    " max_extrude_only_velocity of %.3f mm/s"
                    % (i + 1, peak_v, max_v)
                )
            if abs(seg.accel) > max_a:
                raise ProfileError(
                    "Segment %d uses %.3f mm/s^2, above the extruder's"
                    " max_extrude_only_accel of %.3f mm/s^2"
                    % (i + 1, abs(seg.accel), max_a)
                )
            if seg.distance < 0.0:
                if -seg.distance > MAX_RETRACT_DISTANCE:
                    raise ProfileError(
                        "Segment %d retracts %.3fmm; at most %.1fmm per"
                        " retraction segment is allowed"
                        % (i + 1, -seg.distance, MAX_RETRACT_DISTANCE)
                    )
            else:
                forward_d += seg.distance
            net_d += seg.distance
            total_t += seg.move_t
        if forward_d > self.max_run_distance:
            raise ProfileError(
                "Profile extrudes %.3fmm, above the 'max_run_distance' of"
                " %.3fmm" % (forward_d, self.max_run_distance)
            )
        if forward_d > max_d:
            raise ProfileError(
                "Profile extrudes %.3fmm, above the extruder's"
                " max_extrude_only_distance of %.3fmm" % (forward_d, max_d)
            )
        if total_t > self.max_run_duration:
            raise ProfileError(
                "Profile runs for %.3fs, above the 'max_run_duration' of"
                " %.3fs" % (total_t, self.max_run_duration)
            )
        if net_d < -1e-9:
            raise ProfileError(
                "Profile ends %.3fmm retracted; every retraction must be"
                " primed back within the profile" % (-net_d,)
            )
        return total_t, forward_d, net_d

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def run_profile(self, segments, extruder_name=None):
        """Run a profile and return a record of what was scheduled.

        `segments` is a sequence of Segment objects (see build_segment).
        The returned dict carries the exact print_time of every segment
        start and end, the nominal velocity profile, and the pressure
        advance parameters in effect, so the compensated trajectory can be
        reproduced offline (R4).
        """
        if self.disabled_reason is not None:
            raise ProfileError(
                "extruder_profile is disabled: %s" % (self.disabled_reason,)
            )
        segments = [s for s in segments if s.move_t > MIN_SEGMENT_T]
        if not segments:
            raise ProfileError("Profile has no segments")
        extruder = self._lookup_extruder(extruder_name)
        self._check_idle()
        total_t, forward_d, net_d = self._check_limits(extruder, segments)
        steppers = [es.stepper for es in extruder.get_extruder_steppers()]
        if not steppers:
            raise ProfileError(
                "Extruder '%s' has no steppers" % (extruder.get_name(),)
            )
        pa_params = self._collect_pa_params(extruder)

        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()
        prev_trapqs = []
        try:
            for stepper in steppers:
                prev_trapqs.append((stepper, stepper.set_trapq(self.trapq)))
                stepper.set_position((0.0, 0.0, 0.0))
            start_time = toolhead.get_last_move_time()
            records = self._append_segments(segments, start_time)
            end_time = start_time + total_t
            self.motion_queuing.note_mcu_movequeue_activity(end_time)
            toolhead.dwell(total_t)
            toolhead.flush_step_generation()
        finally:
            # Restore unconditionally: on success, on a command error and on
            # an internal failure (R9).  The extruder's own last_position is
            # the reference the next ordinary move will build on, so the
            # steppers are put back onto it and the profile's filament travel
            # is deliberately not counted into the E coordinate.
            for stepper, prev_trapq in prev_trapqs:
                stepper.set_trapq(prev_trapq)
                stepper.set_position(extruder.last_position)
            self.motion_queuing.wipe_trapq(self.trapq)
        self.last_run = {
            "api_version": API_VERSION,
            "extruder": extruder.get_name(),
            "start_time": start_time,
            "end_time": end_time,
            "duration": total_t,
            "extruded": forward_d,
            "net": net_d,
            "segments": records,
            "pressure_advance": pa_params,
        }
        return self.last_run

    def _append_segments(self, segments, start_time):
        """Write the profile into this module's own trapq.

        Each segment becomes exactly one trapq_append(): an accelerating
        segment as its accel phase, a decelerating one as its decel phase
        and a steady one as its cruise phase.  They are contiguous in
        print_time, so trapq_add_move() inserts no null moves between them
        and the PA smoothing window sees real neighbours at every joint.
        """
        records = []
        print_time = start_time
        position = 0.0
        for seg in segments:
            direction = seg.direction()
            # The trapq works in unsigned speeds along axes_r, so a
            # retracting segment is a positive speed along -x.  Classify by
            # speed MAGNITUDE, not by signed velocity: 0 -> -5 mm/s is a
            # retraction speeding up, which is the trapq's accel phase even
            # though v_end < v_start numerically.
            speed_start = abs(seg.v_start)
            speed_end = abs(seg.v_end)
            accel_t = cruise_t = decel_t = 0.0
            if speed_end > speed_start:
                accel_t = seg.move_t
                start_v, cruise_v = speed_start, speed_end
            elif speed_end < speed_start:
                decel_t = seg.move_t
                start_v, cruise_v = speed_start, speed_start
            else:
                cruise_t = seg.move_t
                start_v, cruise_v = speed_start, speed_start
            accel = abs(seg.accel)
            start_pos = [0.0, 0.0, 0.0]
            axes_r = [0.0, 0.0, 0.0]
            start_pos[PA_CHANNEL] = position
            axes_r[PA_CHANNEL] = direction
            self.trapq_append(
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
            records.append(seg.as_dict(print_time))
            print_time += seg.move_t
            # seg.distance is signed, and the trapq advances start_pos by
            # axes_r.x * move_dist, so this tracks the channel exactly.
            position += seg.distance
        return records

    def _collect_pa_params(self, extruder):
        out = []
        for es in extruder.get_extruder_steppers():
            status = es.get_status(self.printer.get_reactor().monotonic())
            entry = {"name": es.name}
            entry.update(status)
            out.append(entry)
        return out

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    cmd_EXTRUDER_PROFILE_MOVE_help = (
        "Drive the extruder through a speed profile with pressure advance"
    )

    def cmd_EXTRUDER_PROFILE_MOVE(self, gcmd):
        profile = gcmd.get("PROFILE")
        extruder_name = gcmd.get("EXTRUDER", None)
        try:
            segments = parse_profile(profile)
            result = self.run_profile(segments, extruder_name=extruder_name)
        except ProfileError as e:
            raise gcmd.error(str(e))
        gcmd.respond_info(
            "extruder_profile: %d segments, %.3fs, %.3fmm extruded,"
            " print_time %.6f..%.6f"
            % (
                len(result["segments"]),
                result["duration"],
                result["extruded"],
                result["start_time"],
                result["end_time"],
            )
        )

    cmd_EXTRUDER_PROFILE_STATUS_help = (
        "Report whether extruder-only profile moves are available"
    )

    def cmd_EXTRUDER_PROFILE_STATUS(self, gcmd):
        if self.disabled_reason is None:
            gcmd.respond_info(
                "extruder_profile: available, api_version=%d,"
                " verified_against=%s" % (API_VERSION, VERIFIED_AGAINST)
            )
        else:
            gcmd.respond_info(
                "extruder_profile: DISABLED (%s), api_version=%d,"
                " verified_against=%s"
                % (self.disabled_reason, API_VERSION, VERIFIED_AGAINST)
            )

    def is_available(self):
        return self.disabled_reason is None

    def get_status(self, eventtime):
        return {
            "available": self.disabled_reason is None,
            "disabled_reason": self.disabled_reason,
            "api_version": API_VERSION,
            "verified_against": VERIFIED_AGAINST,
            "last_run": self.last_run,
        }


def parse_profile(text):
    """Parse the PROFILE= argument of EXTRUDER_PROFILE_MOVE.

    Comma-separated segments, each a space-free list of KEY=VALUE joined by
    ':'.  Velocities are signed filament mm/s, accel is a magnitude in
    mm/s^2, duration is seconds and distance is signed filament mm:

        PROFILE="V0=0:V1=5:ACCEL=500, V0=5:V1=5:DISTANCE=10,
                 V0=5:V1=2:ACCEL=500, V0=2:V1=2:DURATION=0.5,
                 V0=2:V1=0:ACCEL=500"

    V0 defaults to the previous segment's V1, so only the first segment
    needs it; this is what makes non-zero joint speeds the natural thing to
    write rather than something to remember.
    """
    segments = []
    prev_v_end = 0.0
    for raw in text.split(","):
        raw = raw.strip()
        if not raw:
            continue
        fields = {}
        for item in raw.split(":"):
            item = item.strip()
            if not item:
                continue
            if "=" not in item:
                raise ProfileError(
                    "Segment '%s' has a field without '=': '%s'" % (raw, item)
                )
            key, _, value = item.partition("=")
            key = key.strip().upper()
            if key in fields:
                raise ProfileError("Segment '%s' sets %s twice" % (raw, key))
            try:
                fields[key] = float(value)
            except ValueError:
                raise ProfileError(
                    "Segment '%s' has a non-numeric %s='%s'"
                    % (raw, key, value.strip())
                )
        unknown = set(fields) - {"V0", "V1", "ACCEL", "DURATION", "DISTANCE"}
        if unknown:
            raise ProfileError(
                "Segment '%s' has unknown field(s): %s"
                % (raw, ", ".join(sorted(unknown)))
            )
        if "V1" not in fields:
            raise ProfileError("Segment '%s' needs V1" % (raw,))
        v_start = fields.get("V0", prev_v_end)
        v_end = fields["V1"]
        for key in ("V0", "V1"):
            if key in fields and not math.isfinite(fields[key]):
                raise ProfileError(
                    "Segment '%s' has a non-finite %s" % (raw, key)
                )
        segments.append(
            build_segment(
                v_start,
                v_end,
                accel=fields.get("ACCEL"),
                duration=fields.get("DURATION"),
                distance=fields.get("DISTANCE"),
            )
        )
        prev_v_end = v_end
    if not segments:
        raise ProfileError("PROFILE is empty")
    if segments[0].v_start != 0.0:
        raise ProfileError(
            "A profile must start at rest (first segment V0=%.4f)"
            % (segments[0].v_start,)
        )
    if segments[-1].v_end != 0.0:
        raise ProfileError(
            "A profile must end at rest (last segment V1=%.4f)"
            % (segments[-1].v_end,)
        )
    return segments


def load_config(config):
    return ExtruderProfile(config)
