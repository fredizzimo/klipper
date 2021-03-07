# Printer stepper support
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import chelper

class error(Exception):
    pass


######################################################################
# Steppers
######################################################################

# Interface to low-level mcu and chelper code
class MCU_stepper:
    def __init__(self, name, step_pin_params, dir_pin_params, step_dist,
                 units_in_radians=False, smooth_stop_decel=0):
        self._name = name
        self._step_dist = step_dist
        self._units_in_radians = units_in_radians
        self._smooth_stop_decel = smooth_stop_decel
        self._mcu = step_pin_params['chip']
        self._oid = oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        self._step_pin = step_pin_params['pin']
        self._invert_step = step_pin_params['invert']
        if dir_pin_params['chip'] is not self._mcu:
            raise self._mcu.get_printer().config_error(
                "Stepper dir pin must be on same mcu as step pin")
        self._dir_pin = dir_pin_params['pin']
        self._invert_dir = dir_pin_params['invert']
        self._mcu_position_offset = self._tag_position = 0.
        self._min_stop_interval = 0.
        self._reset_cmd_id = self._get_position_cmd = None
        self._active_callbacks = []
        ffi_main, self._ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(self._ffi_lib.stepcompress_alloc(oid),
                                      self._ffi_lib.stepcompress_free)
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepper_kinematics = None
        self._itersolve_generate_steps = self._ffi_lib.itersolve_generate_steps
        self._itersolve_check_active = self._ffi_lib.itersolve_check_active
        self._trapq = ffi_main.NULL
    def get_mcu(self):
        return self._mcu
    def get_name(self, short=False):
        if short and self._name.startswith('stepper_'):
            return self._name[8:]
        return self._name
    def units_in_radians(self):
        # Returns true if distances are in radians instead of millimeters
        return self._units_in_radians
    def _dist_to_time(self, dist, start_velocity, accel):
        # Calculate the time it takes to travel a distance with constant accel
        time_offset = start_velocity / accel
        return math.sqrt(2. * dist / accel + time_offset**2) - time_offset
    def set_max_jerk(self, max_halt_velocity, max_accel):
        # Calculate the firmware's maximum halt interval time
        last_step_time = self._dist_to_time(self._step_dist,
                                            max_halt_velocity, max_accel)
        second_last_step_time = self._dist_to_time(2. * self._step_dist,
                                                   max_halt_velocity, max_accel)
        self._min_stop_interval = second_last_step_time - last_step_time
    def setup_itersolve(self, alloc_func, *params):
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        self.set_stepper_kinematics(sk)
    def _build_config(self):
        max_error = self._mcu.get_max_stepper_error()
        min_stop_interval = max(0., self._min_stop_interval - max_error)
        self._calc_smooth_stop_profile()
        self._mcu.add_config_cmd(
            "config_stepper oid=%d step_pin=%s dir_pin=%s"
            " min_stop_interval=%d invert_step=%d"
            " num_decel_segments=%d" % (
                self._oid, self._step_pin, self._dir_pin,
                self._mcu.seconds_to_clock(min_stop_interval),
                self._invert_step, 0))
        self._mcu.add_config_cmd("reset_step_clock oid=%d clock=0"
                                 % (self._oid,), on_restart=True)
        step_cmd_id = self._mcu.lookup_command_id(
            "queue_step oid=%c interval=%u count=%hu add=%hi")
        dir_cmd_id = self._mcu.lookup_command_id(
            "set_next_step_dir oid=%c dir=%c")
        self._reset_cmd_id = self._mcu.lookup_command_id(
            "reset_step_clock oid=%c clock=%u")
        self._get_position_cmd = self._mcu.lookup_query_command(
            "stepper_get_position oid=%c",
            "stepper_position oid=%c pos=%i", oid=self._oid)
        self._ffi_lib.stepcompress_fill(
            self._stepqueue, self._mcu.seconds_to_clock(max_error),
            self._invert_dir, step_cmd_id, dir_cmd_id)
    def get_oid(self):
        return self._oid
    def get_step_dist(self):
        return self._step_dist
    def set_step_dist(self, dist):
        self._step_dist = dist
        self.set_stepper_kinematics(self._stepper_kinematics)
    def is_dir_inverted(self):
        return self._invert_dir
    def calc_position_from_coord(self, coord):
        return self._ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2])
    def set_position(self, coord):
        opos = self.get_commanded_position()
        sk = self._stepper_kinematics
        self._ffi_lib.itersolve_set_position(sk, coord[0], coord[1], coord[2])
        self._mcu_position_offset += opos - self.get_commanded_position()
    def get_commanded_position(self):
        sk = self._stepper_kinematics
        return self._ffi_lib.itersolve_get_commanded_pos(sk)
    def get_mcu_position(self):
        mcu_pos_dist = self.get_commanded_position() + self._mcu_position_offset
        mcu_pos = mcu_pos_dist / self._step_dist
        if mcu_pos >= 0.:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)
    def get_tag_position(self):
        return self._tag_position
    def set_tag_position(self, position):
        self._tag_position = position
    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        self._stepper_kinematics = sk
        if sk is not None:
            self._ffi_lib.itersolve_set_stepcompress(sk, self._stepqueue,
                                                     self._step_dist)
            self.set_trapq(self._trapq)
        return old_sk
    def note_homing_end(self, did_trigger=False):
        ret = self._ffi_lib.stepcompress_reset(self._stepqueue, 0)
        if ret:
            raise error("Internal error in stepcompress")
        data = (self._reset_cmd_id, self._oid, 0)
        ret = self._ffi_lib.stepcompress_queue_msg(
            self._stepqueue, data, len(data))
        if ret:
            raise error("Internal error in stepcompress")
        if not did_trigger or self._mcu.is_fileoutput():
            return
        params = self._get_position_cmd.send([self._oid])
        mcu_pos_dist = params['pos'] * self._step_dist
        if self._invert_dir:
            mcu_pos_dist = -mcu_pos_dist
        self._mcu_position_offset = mcu_pos_dist - self.get_commanded_position()
    def set_trapq(self, tq):
        if tq is None:
            ffi_main, self._ffi_lib = chelper.get_ffi()
            tq = ffi_main.NULL
        self._ffi_lib.itersolve_set_trapq(self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        return old_tq
    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)
    def generate_steps(self, flush_time):
        # Check for activity if necessary
        if self._active_callbacks:
            ret = self._itersolve_check_active(self._stepper_kinematics,
                                               flush_time)
            if ret:
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        # Generate steps
        ret = self._itersolve_generate_steps(self._stepper_kinematics,
                                             flush_time)
        if ret:
            raise error("Internal error in stepcompress")
    def is_active_axis(self, axis):
        return self._ffi_lib.itersolve_is_active_axis(
            self._stepper_kinematics, axis)
    def _calc_smooth_stop_profile(self):
        decel = self._smooth_stop_decel
        step_dist = self._step_dist
        logging.info("step_dist %f", step_dist)
        if (decel == 0):
            return []
        def get_step_time(step):
            return self._mcu.seconds_to_clock(math.sqrt(2.0 * step * step_dist / decel))
        def build_segment(step):
            logging.info("build segment %i", step)
            t1 = get_step_time(step)
            t2 = get_step_time(step + 1)
            t3 = get_step_time(step + 2)
            interval = t2 - t1
            add = (t3 - t2) - interval
            return interval, add, t1
        def eval_segement(interval, add, count):
            return interval*count + 0.5 * add * (count - 1)*count
            
        prev_t = 0
        for i in range(1,100):
            t = self._mcu.clock_to_seconds(get_step_time(i))
            dist = step_dist * i
            speed = step_dist / (t - prev_t)
            logging.info("step %i dist %f time %f speed %f" % (i, dist, t, speed))
            prev_t = t
        n = 0
        segments = []
        interval, add, segment_start_time = build_segment(n)
        while abs(add) > 32767:
            segments.append((interval, 0, 1))
            n += 1
            interval, add, segment_start_time = build_segment(n)

        max_error = self._mcu.seconds_to_clock(self._mcu.get_max_stepper_error())
        while add != 0:
            segment_error = 0
            count = 2
            while abs(segment_error) < max_error: 
                count += 1
                desired_segment_time = get_step_time(n + count) - segment_start_time
                actual_segment_time = eval_segement(interval, add, count)
                segment_error = desired_segment_time - actual_segment_time
                #logging.info("Segment error %f %f %f" % (segment_error, desired_segment_time, actual_segment_time))
            count -= 1
            n += count
            segments.append((interval + add * (count-1), -add, count))
            logging.info("n %i, count %i", n, count)
            interval, add, segment_start_time = build_segment(n)
        for segment in segments:
            speed = step_dist / self._mcu.clock_to_seconds(segment[0])
            logging.info("interval %i add %i count %i" % segment)
            logging.info("speed %f", speed)

        interval = step_dist / 35.0
        interval = self._mcu.seconds_to_clock(interval)
        logging.info("Searching for %i", interval)
        for index, s in enumerate(segments):
            if s[0] <= interval:
                break

        if index != len(segments) and s[1] != 0:
            logging.info("%i < %i" % (s[0], interval))
            logging.info(s)
            
            start = s[0] + s[1] * (s[2] - 1)
            logging.info(start)
            #logging.info(interval)
            #logging.info((start - interval) / s[1])
            #count = (start - interval) / s[1]

            # TODO: add can be zero
            ignore_count = 1 + (interval - s[0]) / s[1]
            logging.info(ignore_count)
            count = s[2] - ignore_count
            logging.info("count %i", count)
            interval = s[0] + s[1] * ignore_count
            if count > 0:
                logging.info("Executing %i add %i, count %i, end %i" % (interval, s[1], count, interval + s[1] * (count-1)))


            #segments_end = s->decel_segments - 1;
            for s in reversed(segments[:index]):
                logging.info("Executing %i add %i, count %i, end %i" % (s[0], s[1], s[2], s[0] + s[1] * (s[2]-1)))

            #// Add the rest of the segments
            #for (--segment;segment != segments_end; --segment) {
                #logging.info("Executing %i add %i, count %i" % s[0], s[1], s[2])
            #s->stop_steps = count;
            

# Helper code to build a stepper object from a config section
def PrinterStepper(config, units_in_radians=False):
    printer = config.get_printer()
    name = config.get_name()
    # Stepper definition
    ppins = printer.lookup_object('pins')
    step_pin = config.get('step_pin')
    step_pin_params = ppins.lookup_pin(step_pin, can_invert=True)
    dir_pin = config.get('dir_pin')
    dir_pin_params = ppins.lookup_pin(dir_pin, can_invert=True)
    step_dist = parse_step_distance(config, units_in_radians, True)
    smooth_stop_decel = config.getfloat('smooth_stop_decel', 0, 0)
    mcu_stepper = MCU_stepper(name, step_pin_params, dir_pin_params, step_dist,
                              units_in_radians, smooth_stop_decel)
    # Support for stepper enable pin handling
    stepper_enable = printer.load_object(config, 'stepper_enable')
    stepper_enable.register_stepper(mcu_stepper, config.get('enable_pin', None))
    # Register STEPPER_BUZZ command
    force_move = printer.load_object(config, 'force_move')
    force_move.register_stepper(mcu_stepper)
    return mcu_stepper

# Parse stepper gear_ratio config parameter
def parse_gear_ratio(config, note_valid):
    gear_ratio = config.get('gear_ratio', None, note_valid=note_valid)
    if gear_ratio is None:
        return 1.
    result = 1.
    try:
        gears = gear_ratio.split(',')
        for gear in gears:
            g1, g2 = [float(v.strip()) for v in gear.split(':')]
            result *= g1 / g2
    except:
        raise config.error("Unable to parse gear_ratio: %s" % (gear_ratio,))
    return result

# Obtain "step distance" information from a config section
def parse_step_distance(config, units_in_radians=None, note_valid=False):
    if units_in_radians is None:
        # Caller doesn't know if units are in radians - infer it
        rd = config.get('rotation_distance', None, note_valid=False)
        gr = config.get('gear_ratio', None, note_valid=False)
        units_in_radians = rd is None and gr is not None
    if units_in_radians:
        rotation_dist = 2. * math.pi
        config.get('gear_ratio', note_valid=note_valid)
    else:
        rotation_dist = config.getfloat('rotation_distance', None,
                                        above=0., note_valid=note_valid)
    if rotation_dist is None:
        # Older config format with step_distance
        return config.getfloat('step_distance', above=0., note_valid=note_valid)
    # Newer config format with rotation_distance
    microsteps = config.getint('microsteps', minval=1, note_valid=note_valid)
    full_steps = config.getint('full_steps_per_rotation', 200, minval=1,
                               note_valid=note_valid)
    if full_steps % 4:
        raise config.error("full_steps_per_rotation invalid in section '%s'"
                           % (config.get_name(),))
    gearing = parse_gear_ratio(config, note_valid)
    return rotation_dist / (full_steps * microsteps * gearing)

######################################################################
# Stepper controlled rails
######################################################################

# A motor control "rail" with one (or more) steppers and one (or more)
# endstops.
class PrinterRail:
    def __init__(self, config, need_position_minmax=True,
                 default_position_endstop=None, units_in_radians=False):
        # Primary stepper and endstop
        self.stepper_units_in_radians = units_in_radians
        self.steppers = []
        self.endstops = []
        self.add_extra_stepper(config)
        mcu_stepper = self.steppers[0]
        self.get_commanded_position = mcu_stepper.get_commanded_position
        self.get_tag_position = mcu_stepper.get_tag_position
        self.set_tag_position = mcu_stepper.set_tag_position
        self.calc_position_from_coord = mcu_stepper.calc_position_from_coord
        # Primary endstop position
        mcu_endstop = self.endstops[0][0]
        if hasattr(mcu_endstop, "get_position_endstop"):
            self.position_endstop = mcu_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat(
                'position_endstop', default_position_endstop)
        # Axis range
        if need_position_minmax:
            self.position_min = config.getfloat('position_min', 0.)
            self.position_max = config.getfloat(
                'position_max', above=self.position_min)
        else:
            self.position_min = 0.
            self.position_max = self.position_endstop
        if (self.position_endstop < self.position_min
            or self.position_endstop > self.position_max):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name())
        # Homing mechanics
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.second_homing_speed = config.getfloat(
            'second_homing_speed', self.homing_speed/2., above=0.)
        self.homing_retract_speed = config.getfloat(
            'homing_retract_speed', self.homing_speed, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean(
            'homing_positive_dir', None)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'" % (
                        config.get_name(),))
        elif ((self.homing_positive_dir
               and self.position_endstop == self.position_min)
              or (not self.homing_positive_dir
                  and self.position_endstop == self.position_max)):
            raise config.error(
                "Invalid homing_positive_dir / position_endstop in '%s'"
                % (config.get_name(),))
    def get_range(self):
        return self.position_min, self.position_max
    def get_homing_info(self):
        homing_info = collections.namedtuple('homing_info', [
            'speed', 'position_endstop', 'retract_speed', 'retract_dist',
            'positive_dir', 'second_homing_speed'])(
                self.homing_speed, self.position_endstop,
                self.homing_retract_speed, self.homing_retract_dist,
                self.homing_positive_dir, self.second_homing_speed)
        return homing_info
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return list(self.endstops)
    def add_extra_stepper(self, config):
        stepper = PrinterStepper(config, self.stepper_units_in_radians)
        self.steppers.append(stepper)
        if self.endstops and config.get('endstop_pin', None) is None:
            # No endstop defined - use primary endstop
            self.endstops[0][0].add_stepper(stepper)
            return
        printer = config.get_printer()
        ppins = printer.lookup_object('pins')
        mcu_endstop = ppins.setup_pin('endstop', config.get('endstop_pin'))
        mcu_endstop.config_sampling(config, 'endstop')
        mcu_endstop.add_stepper(stepper)
        name = stepper.get_name(short=True)
        self.endstops.append((mcu_endstop, name))
        query_endstops = printer.load_object(config, 'query_endstops')
        query_endstops.register_endstop(mcu_endstop, name)
    def setup_itersolve(self, alloc_func, *params):
        for stepper in self.steppers:
            stepper.setup_itersolve(alloc_func, *params)
    def generate_steps(self, flush_time):
        for stepper in self.steppers:
            stepper.generate_steps(flush_time)
    def set_trapq(self, trapq):
        for stepper in self.steppers:
            stepper.set_trapq(trapq)
    def set_max_jerk(self, max_halt_velocity, max_accel):
        for stepper in self.steppers:
            stepper.set_max_jerk(max_halt_velocity, max_accel)
    def set_position(self, coord):
        for stepper in self.steppers:
            stepper.set_position(coord)

# Wrapper for dual stepper motor support
def LookupMultiRail(config):
    rail = PrinterRail(config)
    for i in range(1, 99):
        if not config.has_section(config.get_name() + str(i)):
            break
        rail.add_extra_stepper(config.getsection(config.get_name() + str(i)))
    return rail
