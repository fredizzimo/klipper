# -*- coding: utf-8 -*-

# Plans the feedrate with lookahead functionality
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import division
from abc import abstractmethod
import math
from mathutil import newton_raphson
from sys import float_info
import chelper

class MoveQueue(object):
    def __init__(self, size):
        ffi_main, ffi_lib = chelper.get_ffi()
        self.move_alloc = ffi_lib.move_alloc
        self.queue = ffi_main.gc(ffi_lib.move_queue_alloc(size),
            ffi_lib.move_queue_free)
        self.ffi_lib = ffi_lib
        self.ffi_main = ffi_main

    def alloc(self, start_pos, end_pos, speed, accel, accel_to_decel, jerk):
        # TODO: Let the moves re-use the same C move until it's added to the 
        # planner
        # That ensures that the moves are continuous for the planner
        return self.move_alloc(start_pos, end_pos, speed, accel, accel_to_decel,
            jerk, self.queue)


# Class to track each move request
class Move(object):
    tolerance = 1e-13
    time_tolerance = 1e-6

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
    @start_a.setter
    def start_a(self, start_a):
        self.c_move.start_a = start_a
    @property
    def start_v(self):
        return self.c_move.start_v
    @start_v.setter
    def start_v(self, start_v):
        self.c_move.start_v = start_v
    @property
    def cruise_v(self):
        return self.c_move.cruise_v
    @cruise_v.setter
    def cruise_v(self, cruise_v):
        self.c_move.cruise_v = cruise_v
    @property
    def end_v(self):
        return self.c_move.end_v
    @end_v.setter
    def end_v(self, end_v):
        self.c_move.end_v = end_v
    @property
    def accel_t(self):
        return self.c_move.accel_t
    @accel_t.setter
    def accel_t(self, accel_t):
        self.c_move.accel_t = accel_t
    @property
    def cruise_t(self):
        return self.c_move.cruise_t
    @cruise_t.setter
    def cruise_t(self, cruise_t):
        self.c_move.cruise_t = cruise_t
    @property
    def decel_t(self):
        return self.c_move.decel_t
    @decel_t.setter
    def decel_t(self, decel_t):
        self.c_move.decel_t = decel_t
    @property    
    def jerk_t(self):
        return self.c_move.jerk_t
    @jerk_t.setter
    def jerk_t(self, jerk_t):
        self.c_move.jerk_t = jerk_t
    @property    
    def max_junction_v2(self):
        return self.c_move.max_junction_v2
    @max_junction_v2.setter
    def max_junction_v2(self, max_junction_v2):
        self.c_move.max_junction_v2 = max_junction_v2
    @property    
    def max_start_v2(self):
        return self.c_move.max_start_v2
    @max_start_v2.setter
    def max_start_v2(self, max_start_v2):
        self.c_move.max_start_v2 = max_start_v2
    @property    
    def max_smoothed_v2(self):
        return self.c_move.max_smoothed_v2
    @max_smoothed_v2.setter
    def max_smoothed_v2(self, max_smoothed_v2):
        self.c_move.max_smoothed_v2 = max_smoothed_v2
    @property    
    def accel(self):
        return self.c_move.accel
    @accel.setter
    def accel(self, accel):
        self.c_move.accel = accel
    @property    
    def jerk(self):
        return self.c_move.jerk
    @jerk.setter
    def jerk(self, jerk):
        self.c_move.jerk = jerk
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
        self.c_move = queue.alloc(start_pos, end_pos, speed, accel,
            accel_to_decel, jerk) 
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

class FeedratePlanner(object):
    def __init__(self, flush_callback):
        self.flush_callback = flush_callback
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def add_move(self, move, junction_deviation, extruder_instant_v):
        self.queue.append(move)
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
    @abstractmethod
    def flush(self, lazy=False):
        return

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class TrapezoidalFeedratePlanner(FeedratePlanner):
    def __init__(self, flush_callback):
        super(TrapezoidalFeedratePlanner, self).__init__(flush_callback)
    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.
        for i in range(flush_count-1, -1, -1):
            move = queue[i]
            reachable_start_v2 = next_end_v2 + move.delta_v2
            start_v2 = min(move.max_start_v2, reachable_start_v2)
            reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                    or delayed):
                    # This move can decelerate or this is a full accel
                    # move after a full decel move
                    if update_flush_count and peak_cruise_v2:
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(move.max_cruise_v2, (
                        smoothed_v2 + reachable_smoothed_v2) * .5)
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves
                        if not update_flush_count and i < flush_count:
                            mc_v2 = peak_cruise_v2
                            for m, ms_v2, me_v2 in reversed(delayed):
                                mc_v2 = min(mc_v2, ms_v2)
                                m.set_trapezoidal_times(m.move_d,
                                    ms_v2, mc_v2, me_v2, m.accel)
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min((start_v2 + reachable_start_v2) * .5
                                    , move.max_cruise_v2, peak_cruise_v2)
                    move.set_trapezoidal_times(move.move_d, start_v2,
                        cruise_v2, next_end_v2, move.accel)
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count or not flush_count:
            return

        # Generate step times for all moves ready to be flushed
        self.flush_callback(queue[:flush_count])
        # Remove processed moves from the queue
        del queue[:flush_count]


class JerkFeedratePlanner(FeedratePlanner):
    class VirtualMove(object):
        jerk_multipliers = [
            1,
            0,
            -1,
            0,
            -1,
            0,
            1
        ]

        def __init__(self, start_v, accel, jerk):
            self.start_v = start_v
            self.accel = accel
            self.distance = 0
            self.jerk = jerk
            self.end_v = 0
            self.cruise_v = 0
            self.moves = []
            self.move = None


            self.x = 0
            self.v = 0
            self.a = 0
            self.segment_start_x = 0
            self.segment_start_v = 0
            self.segment_start_a = 0
            self.segment_end_x = 0
            self.segment_end_v = 0
            self.segment_end_a = 0

            self.current_segment = 0
            self.current_segment_offset = 0

        def calculate_profile(self, queue):
            start_pos = (0, 0, 0, 0)
            end_pos = (self.distance, 0, 0, 0)
            move = Move(start_pos, end_pos, self.cruise_v, self.accel,
                self.accel, self.jerk, queue)
            move.calculate_jerk(self.start_v, self.end_v)
            self.move = move

        def calculate_first_segment(self):
            self.x = 0
            self.v = self.start_v
            self.a = 0
            self.segment_start_x = self.x
            self.segment_start_v = self.v
            self.segment_start_a = self.a
            self.current_segment = 0
            self.calculate_segment_end()

        def calculate_next_segment(self):
            self.x = self.segment_end_x
            self.v = self.segment_end_v
            self.a = self.segment_end_a
            self.segment_start_x = self.x
            self.segment_start_v = self.v
            self.segment_start_a = self.a
            self.current_segment += 1
            assert self.current_segment < 7
            self.calculate_segment_end()

        def calculate_segment_end(self):
            j = self.jerk_multipliers[self.current_segment] * self.jerk
            t = self.move.jerk_t[self.current_segment]

            x = self.segment_start_x
            v = self.segment_start_v
            a = self.segment_start_a

            self.segment_end_x = self.calculate_x(x, v, a, j, t)
            self.segment_end_v = self.calculate_v(v, a, j, t)
            self.segment_end_a = self.calculate_a(a, j, t)

            self.current_segment_offset = 0

        def calculate_x(self, x, v, a, j, t):
            x += v*t
            x += 0.5 * a*t**2
            x += j*t**3/6.0
            return x

        def calculate_v(self, v, a, j, t):
            v += a*t
            v += 0.5 * j*t**2
            return v

        def calculate_a(self, a, j, t):
            return a + j*t

        def move_to(self, d):
            tolerance = 1e-16
            t = 0.5 * self.move.jerk_t[self.current_segment]
            x = self.segment_start_x - d
            v = self.segment_start_v
            a = self.segment_start_a
            j = self.jerk_multipliers[self.current_segment] * self.jerk
            new_x = 0
            new_v = 0

            def f(t):
                new_x = self.calculate_x(x, v, a, j, t)
                new_v = self.calculate_v(v, a, j, t)
                return new_x, new_v

            t, new_x, new_v = newton_raphson(
                f, 0, self.move.jerk_t[self.current_segment], tolerance, 16)

            self.x = new_x
            self.v = new_v
            self.a = self.calculate_a(a, j, t)
            ret = t - self.current_segment_offset
            self.current_segment_offset = t

            return ret


    def __init__(self, flush_callback):
        super(JerkFeedratePlanner, self).__init__(flush_callback)
        self.virtual_moves = []
        self.current_v = 0
        # TODO: Make sure that we use the same size as the main toolhead queue
        self.virtual_move_queue = MoveQueue(2048)

    @staticmethod
    def can_combine_with_next(next_move, distance, start_v, end_v, end_v2,
        accel, jerk):
        reachable_end_v = Move.get_max_allowed_jerk_end_speed(
            distance, start_v, end_v, accel, jerk)

        if next_move is None or next_move.accel != accel or \
                next_move.jerk != jerk:
            return (False, reachable_end_v)

        can_reach_end = reachable_end_v >= end_v
        if can_reach_end:
            return (False, reachable_end_v)

        # TODO: Do something about this
        # The virtual move and normal move does not have the same attributes
        if hasattr(next_move, "max_cruise_v2"):
            if next_move.max_cruise_v2 == end_v2:
                return (True, end_v)
        else:
            if next_move.cruise_v == end_v:
                return (True, end_v)

        return (Move.can_accelerate_fully(distance,
            start_v, end_v, accel, jerk), reachable_end_v)


    def forward_pass(self):
        v_move = None
        current_v = self.current_v
        for i, move in enumerate(self.queue):
            if i != len(self.queue) - 1:
                next_move = self.queue[i+1]
                end_v2 = next_move.max_junction_v2
            else:
                next_move = None
                end_v2 = move.max_cruise_v2
            if v_move is None:
                v_move = self.VirtualMove(
                    start_v=current_v,
                    accel=move.accel,
                    jerk=move.jerk
                )
            end_v = math.sqrt(end_v2)

            v_move.moves.append(move)

            v_move.distance += move.move_d

            can_combine_with_next, reachable_end_v = self.can_combine_with_next(
                next_move, v_move.distance, v_move.start_v, end_v, end_v2,
                v_move.accel, v_move.jerk
            )

            if not can_combine_with_next:
                current_v = min(end_v, reachable_end_v)
                v_move.end_v = current_v
                v_move.cruise_v =\
                    max(v_move.end_v, math.sqrt(move.max_cruise_v2))
                self.virtual_moves.append(v_move)
                v_move = None


    def backward_pass(self):
        current_v = 0
        output = []
        for i in reversed(range(len(self.virtual_moves))):
            move = self.virtual_moves[i]
            if i != 0:
                prev_move = self.virtual_moves[i-1]
            else:
                prev_move = None

            if move.end_v > current_v:
                move.end_v = current_v

            start_v = move.start_v
            start_v2 = start_v**2

            can_combine_with_next, reachable_start_v = \
                self.can_combine_with_next(
                    prev_move, move.distance, move.end_v, start_v,
                    start_v2, move.accel, move.jerk
            )

            if not can_combine_with_next:
                current_v = min(start_v, reachable_start_v)
                move.start_v = current_v
                output.append(move)
            else:
                prev_move.distance += move.distance
                prev_move.moves.extend(move.moves)

        self.virtual_moves = reversed(output)


    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        if not self.queue:
            return
        tolerance = 1e-9
        self.virtual_moves = []
        self.forward_pass()
        self.backward_pass()
        flush_count = 0
        move_count = 0
        for vmove in self.virtual_moves:
            vmove.calculate_profile(self.virtual_move_queue)
            vmove.calculate_first_segment()

            d = 0

            for move in vmove.moves:
                move_count += 1

                move.jerk = vmove.jerk

                d += move.move_d

                move.start_v = vmove.v
                move.start_a = vmove.a
                move.jerk_t = [0.0] * 7
                cruise_v = vmove.segment_end_v
                at_end = False
                while d >= vmove.segment_end_x - tolerance:
                    s = vmove.current_segment
                    move.jerk_t[s] = vmove.move.jerk_t[s]\
                        - vmove.current_segment_offset
                    cruise_v = max(cruise_v, vmove.segment_start_v)
                    if s == 6:
                        at_end = True
                        break

                    vmove.calculate_next_segment()

                if d < vmove.segment_end_x - tolerance:
                    move.jerk_t[vmove.current_segment] = vmove.move_to(d)
                    move.end_v = vmove.v
                else:
                    move.end_v = vmove.segment_end_v

                move.cruise_v = max(cruise_v, vmove.v)

                target_end_v2 = move.max_cruise_v2
                if move_count < len(self.queue):
                    target_end_v2 = self.queue[move_count].max_junction_v2
                # Flush when the top speed is reached, and there's no
                # acceleration (at a cruise segment, or at the end)
                if vmove.current_segment == 3 or at_end:
                    if abs(move.end_v**2 - target_end_v2) < tolerance:
                        flush_count = move_count

                move.start_v = max(0, move.start_v)
                move.end_v = max(0, move.end_v)
        if not lazy:
            flush_count = move_count
        if flush_count > 0:
            self.flush_callback(self.queue[:flush_count])
            self.current_v = self.queue[flush_count-1].end_v
            del self.queue[:flush_count]
        self.virtual_moves = []

class SmoothExtrusionFeedratePlanner(object):
    MODE_NONE = 0
    MODE_JERK = 1
    MODE_TRAPEZOIDAL = 2

    def __init__(self, flush_callback):
        self.trapezoidal_planner = TrapezoidalFeedratePlanner(flush_callback)
        self.jerk_planner = JerkFeedratePlanner(flush_callback)
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

    def get_last(self):
        if self.mode == self.MODE_JERK:
            return self.jerk_planner.get_last()
        else:
            return self.trapezoidal_planner.get_last()