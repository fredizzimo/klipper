# -*- coding: utf-8 -*-
# Plans the feedrate with lookahead functionality
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import division
import math

# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)


class MoveProfile(object):
    tolerance = 1e-12
    def __init__(self):
        self.start_v = 0.0
        self.cruise_v = 0.0
        self.end_v = 0.0

        self.accel_t = 0.0
        self.cruise_t = 0.0
        self.decel_t = 0.0

        self.accel = 0

        self.is_jerk = False

        self.jerk_t = [0.0] * 7
        self. jerk = 0

    def set_trapezoidal_times(self, distance, start_v2, cruise_v2, end_v2,
                             accel):
        start_v2 = min(start_v2, cruise_v2)
        end_v2 = min(end_v2, cruise_v2)
        self.accel = accel
        # Determine accel, cruise, and decel portions of the move distance
        half_inv_accel = .5 / accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = distance - accel_d - decel_d
        # Make sure that all distances and therefore the times are positive
        # Clamp to zero if close to it, so that the whole segment is removed
        if accel_d < MoveProfile.tolerance:
            accel_d = 0
        if decel_d < MoveProfile.tolerance:
            decel_d = 0
        if cruise_d < MoveProfile.tolerance:
            cruise_d = 0

        # Determine move velocities
        self.start_v = start_v = math.sqrt(start_v2)
        self.cruise_v = cruise_v = math.sqrt(cruise_v2)
        self.end_v = end_v = math.sqrt(end_v2)
        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        self.accel_t = accel_d / ((start_v + cruise_v) * 0.5)
        self.cruise_t = cruise_d / cruise_v
        self.decel_t = decel_d / ((end_v + cruise_v) * 0.5)

    def calculate_trapezoidal(self, distance, start_v, max_v, end_v, accel):
        max_v2 = max_v**2
        start_v2 = start_v**2
        end_v2 = end_v**2
        cruise_v2 = distance * accel + 0.5 * (start_v2 + end_v2)
        cruise_v2 = min(max_v2, cruise_v2)
        self.set_trapezoidal_times(distance, start_v2, cruise_v2, end_v2, accel)

    def calculate_jerk(self, distance, start_v, max_v, end_v, accel, jerk):
        # Calculate a jerk limited profile based on the paper
        # FIR filter-based online jerk-constrained trajectory generation
        # by Pierre Besset and Richard Béarée
        self.jerk = jerk
        jerk_t = accel / jerk
        # The distance needs fixup
        delta_distance = start_v * (jerk_t / 2) + end_v * (jerk_t / 2)
        fixed_distance = distance - delta_distance
        # Start by a trapezoidal profile
        self.calculate_trapezoidal(fixed_distance, start_v, max_v, end_v, accel)
        t1 = self.accel_t - jerk_t
        t3 = self.cruise_t - jerk_t
        t5 = self.decel_t - jerk_t

        # Case II
        if t3 <= -MoveProfile.tolerance:
            # Generate a trapezoidal profile with a cruise time of exactly
            # jerk_t, by solving for cruise_v from
            # distance = accel_d + cruise_d + decel_d
            # which gives 
            max_v = math.sqrt((accel*jerk_t)**2.0 + 4.0*accel*fixed_distance +
                2.0*start_v**2.0 + 2.0*end_v**2.0)
            max_v -= jerk_t * accel
            max_v /= 2.0
            return self.calculate_jerk(distance, start_v, max_v, end_v, accel,
                jerk)

        # Clamp to zero to remove empty segments
        if t1 < MoveProfile.tolerance:
            t1 = 0
        if t3 < MoveProfile.tolerance:
            t3 = 0
        if t5 < MoveProfile.tolerance:
            t5 = 0

        self.jerk_t[0] = jerk_t
        self.jerk_t[1] = t1
        self.jerk_t[2] = jerk_t
        self.jerk_t[3] = t3
        self.jerk_t[4] = jerk_t
        self.jerk_t[5] = t5
        self.jerk_t[6] = jerk_t


# Class to track each move request
class Move(object):
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        self.axes_d = axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.move_d = move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        if move_d < .000000001:
            # Extrude only move
            self.end_pos = (start_pos[0], start_pos[1], start_pos[2],
                            end_pos[3])
            axes_d[0] = axes_d[1] = axes_d[2] = 0.
            self.move_d = move_d = abs(axes_d[3])
            inv_move_d = 0.
            if move_d:
                inv_move_d = 1. / move_d
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1. / move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity
        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_start_v2 = 0.
        self.max_cruise_v2 = velocity**2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel

        self.profile = MoveProfile()
    def limit_speed(self, speed, accel):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)
    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        # Allow extruder to calculate its maximum junction
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        # Find max velocity using "approximated centripetal velocity"
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(axes_r[0] * prev_axes_r[0]
                               + axes_r[1] * prev_axes_r[1]
                               + axes_r[2] * prev_axes_r[2])
        if junction_cos_theta > 0.999999:
            return
        junction_cos_theta = max(junction_cos_theta, -0.999999)
        sin_theta_d2 = math.sqrt(0.5*(1.0-junction_cos_theta))
        R = (self.toolhead.junction_deviation * sin_theta_d2
             / (1. - sin_theta_d2))
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5*(1.0+junction_cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
        prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                    * prev_move.accel)
        self.max_start_v2 = min(
            R * self.accel, R * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            extruder_v2, self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)
        self.max_smoothed_v2 = min(
            self.max_start_v2
            , prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)

LOOKAHEAD_FLUSH_TIME = 0.250

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class TrapezoidalFeedratePlanner:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None
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
                                m.profile.set_trapezoidal_times(m.move_d,
                                    ms_v2, mc_v2, me_v2, m.accel)
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min((start_v2 + reachable_start_v2) * .5
                                    , move.max_cruise_v2, peak_cruise_v2)
                    move.profile.set_trapezoidal_times(move.move_d, start_v2,
                        cruise_v2, next_end_v2, move.accel)
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count or not flush_count:
            return
        # Generate step times for all moves ready to be flushed
        self.toolhead._process_moves(queue[:flush_count])
        # Remove processed moves from the queue
        del queue[:flush_count]
    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])
        self.junction_flush -= move.min_move_t
        if self.junction_flush <= 0.:
            # Enough moves have been queued to reach the target flush time.
            self.flush(lazy=True)