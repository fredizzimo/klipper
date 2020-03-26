# -*- coding: utf-8 -*-

# Plans the feedrate with lookahead functionality
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import division
import chelper

class MoveQueue(object):
    def __init__(self, size):
        ffi_main, ffi_lib = chelper.get_ffi()
        self.move_reserve = ffi_lib.move_reserve
        self.move_commit = ffi_lib.move_commit
        self.queue = ffi_main.gc(ffi_lib.move_queue_alloc(size),
            ffi_lib.move_queue_free)
        self.ffi_lib = ffi_lib
        self.ffi_main = ffi_main

    def reserve(self, start_pos, end_pos, speed, accel, accel_to_decel, jerk):
        return self.move_reserve(start_pos, end_pos, speed, accel,
            accel_to_decel, jerk, self.queue)
    def commit(self):
        self.move_commit(self.queue)


# Class to track each move request
class Move(object):
    @property
    def move_d(self):
        return self.c_move.move_d
    @property
    def start_pos(self):
        return tuple(self.c_move.start_pos)
    @property
    def end_pos(self):
        return tuple(self.c_move.end_pos)
    @property    
    def axes_d(self):
        return tuple(self.c_move.axes_d)
    @property    
    def axes_r(self):
        return tuple(self.c_move.axes_r)
    @property
    def is_kinematic_move(self):
        return self.c_move.is_kinematic_move
    @property
    def start_a(self):
        return self.c_move.start_a
    @property
    def start_v(self):
        return self.c_move.start_v
    @property
    def cruise_v(self):
        return self.c_move.cruise_v
    @property
    def end_v(self):
        return self.c_move.end_v
    @property
    def accel_t(self):
        return self.c_move.accel_t
    @property
    def cruise_t(self):
        return self.c_move.cruise_t
    @property
    def decel_t(self):
        return self.c_move.decel_t
    @property    
    def jerk_t(self):
        return self.c_move.jerk_t
    @property    
    def max_junction_v2(self):
        return self.c_move.max_junction_v2
    @property    
    def max_start_v2(self):
        return self.c_move.max_start_v2
    @property    
    def max_smoothed_v2(self):
        return self.c_move.max_smoothed_v2
    @property    
    def accel(self):
        return self.c_move.accel
    @property    
    def jerk(self):
        return self.c_move.jerk
    @property    
    def max_cruise_v2(self):
        return self.c_move.max_cruise_v2
    @property    
    def delta_v2(self):
        return self.c_move.delta_v2
    @property    
    def smooth_delta_v2(self):
        return self.c_move.smooth_delta_v2
    @property    
    def min_move_t(self):
        return self.c_move.min_move_t

    def __init__(self, start_pos, end_pos, speed, accel, accel_to_decel, jerk,
            queue):
        self.c_limit_speed = queue.ffi_lib.limit_speed
        self.c_calc_junction = queue.ffi_lib.calc_junction
        self.c_set_trapezoidal_times = queue.ffi_lib.set_trapezoidal_times
        self.c_calculate_trapezoidal = queue.ffi_lib.calculate_trapezoidal
        self.c_calculate_jerk = queue.ffi_lib.calculate_jerk
        self.c_move = queue.reserve(start_pos, end_pos, speed, accel,
            accel_to_decel, jerk) 
        self.queue = queue
        self.timing_callbacks = []

    def limit_speed(self, speed, accel, max_accel_to_decel=-1):
        self.c_limit_speed(self.c_move, speed, accel, max_accel_to_decel)

    def calc_junction(self, prev_move, junction_deviation,
            extruder_instant_v):
        self.c_calc_junction(self.c_move, prev_move.c_move,
            junction_deviation, extruder_instant_v)

    def set_trapezoidal_times(self, distance, start_v2, cruise_v2, end_v2,
                             accel):
        self.c_set_trapezoidal_times(self.c_move, distance, start_v2, cruise_v2,
            end_v2, accel)

    def calculate_trapezoidal(self, start_v, end_v):
        self.c_calculate_trapezoidal(self.c_move, start_v, end_v)

    def calculate_jerk(self, start_v, end_v):
        self.c_calculate_jerk(self.c_move, start_v, end_v)

    @staticmethod
    def get_max_allowed_jerk_end_speed(distance, start_v, end_v, max_a, jerk):
        # TODO don't lookup the function each time
        _, ffi_lib = chelper.get_ffi()
        return ffi_lib.get_max_allowed_jerk_end_speed(distance, start_v, end_v,
            max_a, jerk)

    @staticmethod
    def can_accelerate_fully(distance, start_v, end_v, accel, jerk):
        # TODO don't lookup the function each time
        _, ffi_lib = chelper.get_ffi()
        return ffi_lib.can_accelerate_fully(distance, start_v, end_v, accel,
            jerk)


LOOKAHEAD_FLUSH_TIME = 0.250

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class FeedratePlanner(object):
    planner_type = "invalid"

    def __init__(self, move_queue, flush_callback):
        self.flush_callback = flush_callback
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

        planner_type = self.planner_type
        ffi_main, ffi_lib = chelper.get_ffi()
        planner_alloc = getattr(ffi_lib, planner_type + "_planner_alloc")
        planner_free = getattr(ffi_lib, planner_type + "_planner_free")
        planner_flush = getattr(ffi_lib, planner_type + "_planner_flush")
        planner_reset = getattr(ffi_lib, planner_type + "_planner_reset")

        self.c_planner = ffi_main.gc(
            planner_alloc(move_queue.queue),
            planner_free)
        self.c_flush = planner_flush
        self.c_reset = planner_reset

    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        self.c_reset(self.c_planner)
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def add_move(self, move, junction_deviation, extruder_instant_v):
        self.queue.append(move)
        move.queue.commit()
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2], junction_deviation,
            extruder_instant_v)
        self.junction_flush -= move.min_move_t
        if self.junction_flush <= 0.:
            # Enough moves have been queued to reach the target flush time.
            self.flush(lazy=True)
    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None
    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        flush_count = self.c_flush(self.c_planner, lazy)
        if flush_count > 0:
            # Generate step times for all moves ready to be flushed
            self.flush_callback(self.queue[:flush_count])
            # Remove processed moves from the queue
            del self.queue[:flush_count]
    
class TrapezoidalFeedratePlanner(FeedratePlanner):
    planner_type = "trapezoidal"

class JerkFeedratePlanner(FeedratePlanner):
    planner_type = "jerk"

class SmoothExtrusionFeedratePlanner(object):
    MODE_NONE = 0
    MODE_JERK = 1
    MODE_TRAPEZOIDAL = 2

    def __init__(self, move_queue, flush_callback):
        self.trapezoidal_planner = TrapezoidalFeedratePlanner(move_queue,
            flush_callback)
        self.jerk_planner = JerkFeedratePlanner(move_queue, flush_callback)
        self.mode = self.MODE_NONE

    def reset(self):
        self.trapezoidal_planner.reset()
        self.jerk_planner.reset()

    def set_flush_time(self, flush_time):
        self.trapezoidal_planner.set_flush_time(flush_time)
        self.jerk_planner.set_flush_time(flush_time)

    def add_move(self, move, junction_deviation, extruder_instant_v):
        if move.is_kinematic_move and move.axes_d[3] != 0:
            if self.mode == self.MODE_TRAPEZOIDAL:
                self.flush(False)
            self.mode = self.MODE_JERK
            self.jerk_planner.add_move(move, junction_deviation,
                extruder_instant_v)
        else:
            if self.mode == self.MODE_JERK:
                self.flush(False)
            self.mode = self.MODE_TRAPEZOIDAL
            self.trapezoidal_planner.add_move(move, junction_deviation,
                extruder_instant_v)

    def flush(self, lazy=False):
        if self.mode == self.MODE_JERK:
            self.jerk_planner.flush(lazy)
        elif self.mode == self.MODE_TRAPEZOIDAL:
            self.trapezoidal_planner.flush(lazy)

        # Make sure that both planners are using the lookahead flush time
        self.set_flush_time(LOOKAHEAD_FLUSH_TIME)

    def get_last(self):
        if self.mode == self.MODE_JERK:
            return self.jerk_planner.get_last()
        else:
            return self.trapezoidal_planner.get_last()