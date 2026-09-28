# Drive mechanically coupled steppers from the step events of one stepper
#
# Copyright (C) 2026  Wojciech Bednarski
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import functools

# Margin beyond the last generated step before relinking pins on the mcu
IDLE_MARGIN_TIME = 0.050


class StepperMirror:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}
        self.enable_tracked = set()
        self.in_enable_update = False
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler(
            "homing:homing_move_begin", self._handle_homing_move_begin
        )
        self.printer.register_event_handler(
            "stepper:mirror_stopped", self._handle_mirror_stopped
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SYNC_STEPPER_TO_AXIS",
            self.cmd_SYNC_STEPPER_TO_AXIS,
            desc=self.cmd_SYNC_STEPPER_TO_AXIS_help,
        )

    def _handle_connect(self):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        self.steppers = {s.get_name(): s for s in kin.get_steppers()}

    def _followers(self, primary=None):
        followers = []
        for stepper in self.steppers.values():
            p = stepper.get_mirror_primary()
            if p is not None and (primary is None or p is primary):
                followers.append(stepper)
        return followers

    def _wait_idle(self):
        # Steps (eg, input shaper smoothing) may run past the last move's end
        toolhead = self.printer.lookup_object("toolhead")
        motion_queuing = self.printer.lookup_object("motion_queuing")
        toolhead.dwell(motion_queuing.get_kin_flush_delay() + IDLE_MARGIN_TIME)
        toolhead.wait_moves()

    def _link(self, follower, primary):
        follower.link_mirror(primary)
        stepper_enable = self.printer.lookup_object("stepper_enable")
        for stepper in (primary, follower):
            name = stepper.get_name()
            if name not in self.enable_tracked:
                self.enable_tracked.add(name)
                stepper_enable.lookup_enable(name).register_state_callback(
                    functools.partial(self._handle_enable_state, stepper)
                )

    def _unlink(self, follower):
        follower.unlink_mirror()

    def _handle_enable_state(self, stepper, print_time, enabled):
        # A linked group is one mechanical unit, so it shares one enable state
        if self.in_enable_update:
            return
        primary = stepper.get_mirror_primary() or stepper
        group = [primary] + self._followers(primary)
        if len(group) < 2:
            return
        stepper_enable = self.printer.lookup_object("stepper_enable")
        self.in_enable_update = True
        try:
            for s in group:
                if s is stepper:
                    continue
                enable_line = stepper_enable.lookup_enable(s.get_name())
                if enabled:
                    enable_line.motor_enable(print_time)
                else:
                    enable_line.motor_disable(print_time)
        finally:
            self.in_enable_update = False

    def unlink_steppers(self, steppers, reason):
        # Unlink every group containing any of the given steppers
        steppers = set(steppers)
        followers = []
        for follower in self._followers():
            if {follower, follower.get_mirror_primary()} & steppers:
                followers.append(follower)
        if not followers:
            return
        self._wait_idle()
        msgs = []
        for follower in followers:
            primary = follower.get_mirror_primary()
            msgs.append(
                "%s from %s" % (follower.get_name(), primary.get_name())
            )
            self._unlink(follower)
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info("Unlinked %s for %s" % (", ".join(msgs), reason))

    def _handle_homing_move_begin(self, hmove):
        # Endstops stop steppers individually, so they must not be linked
        endstops = hmove.get_mcu_endstops()
        steppers = [s for es in endstops for s in es.get_steppers()]
        self.unlink_steppers(steppers, "homing")

    def _handle_mirror_stopped(self, follower, primary):
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info(
            "%s no longer mirrors %s (stopped by an endstop or probe)"
            % (follower.get_name(), primary.get_name())
        )

    def _check_link(self, gcmd, follower, axis):
        if len(axis) != 1 or axis not in "xyz":
            raise gcmd.error("Invalid AXIS '%s'" % (axis,))
        pname = "stepper_" + axis
        fname = follower.get_name()
        primary = self.steppers.get(pname)
        if (
            primary is None
            or not fname.startswith(pname)
            or not fname[len(pname) :].isdigit()
        ):
            raise gcmd.error(
                "%s is not an extra stepper of %s" % (fname, pname)
            )
        if follower.get_mirror_primary() is not None:
            raise gcmd.error(
                "%s already mirrors %s"
                % (fname, follower.get_mirror_primary().get_name())
            )
        mcu = primary.get_mcu()
        max_mirrors = primary.get_max_mirrors()
        if not max_mirrors:
            raise gcmd.error(
                "Firmware on mcu '%s' does not support stepper mirroring;"
                " rebuild it with 'Enable extra low-level configuration"
                " options' and 'Support mirroring steps to mechanically"
                " coupled steppers' selected in make menuconfig"
                % (mcu.get_name(),)
            )
        if follower.get_mcu() is not mcu:
            raise gcmd.error(
                "%s and %s must be on the same mcu" % (fname, pname)
            )
        if len(self._followers(primary)) >= max_mirrors:
            raise gcmd.error(
                "%s can't be mirrored by more than %d steppers"
                % (pname, max_mirrors)
            )
        if follower.get_step_dist() != primary.get_step_dist():
            raise gcmd.error(
                "%s and %s must have the same step distance" % (fname, pname)
            )
        if follower.get_dir_inverted()[0] != primary.get_dir_inverted()[0]:
            raise gcmd.error(
                "%s and %s must have the same dir_pin polarity (wire the"
                " motors so the same dir signal turns both the same way)"
                % (fname, pname)
            )
        if (
            follower.is_step_inverted() != primary.is_step_inverted()
            or follower.get_pulse_duration() != primary.get_pulse_duration()
        ):
            raise gcmd.error(
                "%s and %s must have the same step_pin polarity and"
                " step_pulse_duration" % (fname, pname)
            )
        if follower.get_trapq() != primary.get_trapq():
            raise gcmd.error(
                "%s is not following the same motion as %s" % (fname, pname)
            )
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        eventtime = self.printer.get_reactor().monotonic()
        if axis not in kin.get_status(eventtime)["homed_axes"]:
            raise gcmd.error(
                "Must home axis %s before linking %s" % (axis, fname)
            )
        stepper_enable = self.printer.lookup_object("stepper_enable")
        for name in (pname, fname):
            if not stepper_enable.lookup_enable(name).is_motor_enabled():
                raise gcmd.error("Stepper %s is not enabled" % (name,))
        return primary

    cmd_SYNC_STEPPER_TO_AXIS_help = (
        "Mirror a stepper onto the primary stepper of an axis"
    )

    def cmd_SYNC_STEPPER_TO_AXIS(self, gcmd):
        name = gcmd.get("STEPPER")
        follower = self.steppers.get(name)
        if follower is None:
            raise gcmd.error("Unknown stepper '%s'" % (name,))
        axis = gcmd.get("AXIS").lower()
        current = follower.get_mirror_primary()
        if not axis:
            if current is not None:
                self._wait_idle()
                self._unlink(follower)
                gcmd.respond_info(
                    "%s no longer mirrors %s" % (name, current.get_name())
                )
            return
        if current is not None and current.get_name() == "stepper_" + axis:
            return
        primary = self._check_link(gcmd, follower, axis)
        self._wait_idle()
        self._link(follower, primary)
        gcmd.respond_info("%s now mirrors %s" % (name, primary.get_name()))

    def get_status(self, eventtime):
        return {
            "linked": {
                f.get_name(): f.get_mirror_primary().get_name()
                for f in self._followers()
            }
        }


def load_config(config):
    return StepperMirror(config)
