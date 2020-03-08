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
# TODO: Remove this dependency
import numpy as np


class MoveProfile(object):
    tolerance = 1e-12
    def __init__(self, start_pos=0, is_kinematic_move=True, axes_r=None,
                 axes_d=None, end_pos=None):
        self.start_pos = start_pos

        self.start_v = 0.0
        self.cruise_v = 0.0
        self.end_v = 0.0
        self.start_a = 0.0

        self.accel_t = 0.0
        self.cruise_t = 0.0
        self.decel_t = 0.0

        self.accel = 0
        self.decel = 0

        self.jerk_t = [0.0] * 7
        self.jerk = 0

        self.is_kinematic_move = is_kinematic_move
        self.axes_r = axes_r
        self.axes_d = axes_d
        self.end_pos = end_pos

    def set_trapezoidal_times(self, distance, start_v2, cruise_v2, end_v2,
                             accel, decel=None):
        if decel is None:
            decel = accel
        start_v2 = min(start_v2, cruise_v2)
        end_v2 = min(end_v2, cruise_v2)
        self.accel = accel
        self.decel = decel
        # Determine accel, cruise, and decel portions of the move distance
        half_inv_accel = .5 / accel
        half_inv_decel = .5 / decel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_decel
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

    def calculate_trapezoidal(self, distance, start_v, max_v, end_v, accel,
            decel=None):

        max_v2 = max_v**2
        start_v2 = start_v**2
        end_v2 = end_v**2
        # The formula is calculated by solving cruise_v2 from
        # distance = (cruise_v2 - start_v2) / 2 + (cruise_v2 - end_v2) / 2
        # which is derived from the standard timeless kinematic formula
        if decel is None:
            decel = accel
            cruise_v2 = distance * accel + 0.5 * (start_v2 + end_v2)
        else:
            cruise_v2 = 2.0 * accel * decel * distance
            cruise_v2 += accel * end_v2
            cruise_v2 += decel * start_v2
            cruise_v2 /= accel + decel
        cruise_v2 = min(max_v2, cruise_v2)
        self.set_trapezoidal_times(distance, start_v2, cruise_v2, end_v2, accel,
            decel)


    def get_min_allowed_jerk_distance(self, v1, v2, a_max, jerk):
        if a_max == 0:
            return 0
        v_s = min(v1, v2)
        v_e = max(v1, v2)
        distance = v_s*a_max**2 + v_e*a_max**2 - jerk*v_s**2 + jerk*v_e**2
        distance /= 2.0 * a_max*jerk
        return distance

    def calculate_jerk(self, distance, start_v, max_v, end_v, accel, jerk, decel=None):
        # Calculate a jerk limited profile based on the paper
        # FIR filter-based online jerk-constrained trajectory generation
        # by Pierre Besset and Richard Béarée

        # Make sure that max_v not smaller than the endpoints, due to rounding
        # errors
        if decel is None:
            decel = accel
        
        max_v = max(max_v, start_v, end_v)

        accel_jerk_t = accel / jerk
        decel_jerk_t = decel / jerk
        delta_accel_v = max_v - start_v
        delta_decel_v = max_v - end_v
        accel_t = delta_accel_v / accel
        decel_t = delta_decel_v / decel
        accel_const_t = accel_t - accel_jerk_t
        decel_const_t = decel_t - decel_jerk_t

        # type III adaptations
        if accel_const_t < 0:
            accel = math.sqrt(jerk * delta_accel_v)
        if decel_const_t < 0:
            decel = math.sqrt(jerk * delta_decel_v)

        start_v2 = start_v**2
        max_v2 = max_v**2
        end_v2 = end_v**2
        accel_decel = accel * decel
        two_accel_decel = 2.0 * accel_decel
        two_accel_decel_jerk = two_accel_decel * jerk
        two_accel_decel_distance_jerk = two_accel_decel_jerk*distance

        dist_cruise = accel*start_v + accel*max_v + decel*max_v + decel*end_v
        dist_cruise *= -accel_decel
        dist_cruise +=  two_accel_decel_distance_jerk
        dist_cruise += accel*jerk*(end_v2-max_v2)
        dist_cruise += decel*jerk*(start_v2-max_v2)
        dist_cruise /= two_accel_decel_jerk

        if dist_cruise < 0:
            # Type II
            dist_cruise = 0

            m_accel_m_decel = -accel - decel
            accel_2 = accel**2
            decel_2 = decel**2

            a = m_accel_m_decel
            a /= two_accel_decel

            b = m_accel_m_decel 
            b /= 2.0 * jerk

            c = -accel_2 * decel * start_v  
            c -= decel_2 * accel * end_v
            c += two_accel_decel_distance_jerk
            c += accel * jerk * end_v2
            c += decel * jerk * start_v2
            c /= two_accel_decel_jerk

            max_v = -b - math.sqrt(b**2 - 4.0*a*c)
            max_v /= 2.0*a

            accel_jerk_t = accel / jerk
            decel_jerk_t = decel / jerk
            delta_accel_v = max_v - start_v
            delta_decel_v = max_v - end_v
            accel_t = delta_accel_v / accel
            decel_t = delta_decel_v / decel
            accel_const_t = accel_t - accel_jerk_t
            decel_const_t = decel_t - decel_jerk_t
            # Type IIII-a
            if accel_const_t < 0:
                x0 = jerk**(-2)
                x1 = 1/(2*decel)
                x2 = x0*x1
                x3 = decel_2
                x4 = 2*start_v
                x5 = 1/jerk
                a = x2
                b = x0
                c = x2*(jerk*x4 + x3)
                d = x4*x5
                e = x1*x5*(-2*decel*distance*jerk - jerk*end_v2 + jerk*start_v2\
                    + end_v*x3 + start_v*x3)
                roots = np.roots((a, b, c, d, e))
                for root in roots:
                    if np.isreal(root) and root > 0:
                        accel = np.real(root)
                        max_v = accel**2 / jerk + start_v
                        break
                



        # TODO: This code is duplicated
        accel_jerk_t = accel / jerk
        decel_jerk_t = decel / jerk
        delta_accel_v = max_v - start_v
        delta_decel_v = max_v - end_v
        accel_t = delta_accel_v / accel
        decel_t = delta_decel_v / decel
        accel_const_t = accel_t - accel_jerk_t
        decel_const_t = decel_t - decel_jerk_t

        self.jerk = jerk
        self.start_v = start_v
        self.cruise_v = max_v
        self.end_v = end_v
        cruise_t = dist_cruise / max_v

        # Clamp to zero to remove empty segments
        if accel_const_t < MoveProfile.tolerance:
            accel_const_t = 0
        if cruise_t < MoveProfile.tolerance:
            cruise_t = 0
        if decel_const_t < MoveProfile.tolerance:
            decel_const_t = 0

        self.jerk_t = [
            accel_jerk_t,
            accel_const_t,
            accel_jerk_t,
            cruise_t,
            decel_jerk_t,
            decel_const_t,
            decel_jerk_t
        ]


    def calculate_jerk_old(self, distance, start_v, max_v, end_v, accel, jerk,
                       decel=None, last_type=0):
        # Calculate a jerk limited profile based on the paper
        # FIR filter-based online jerk-constrained trajectory generation
        # by Pierre Besset and Richard Béarée

        # Make sure that max_v not smaller than the endpoints, due to rounding
        # errors
        max_v = max(max_v, start_v, end_v)

        # If no speed change is allowed, then create a constant
        # profile
        if (abs(start_v - end_v) <= self.tolerance and
            abs(start_v - max_v) <= self.tolerance):
                cruise_t = distance / max_v
                self.jerk = jerk
                self.start_v = start_v
                self.cruise_v = max_v
                self.end_v = end_v
                self.jerk_t = [
                    0,
                    0,
                    0,
                    cruise_t,
                    0,
                    0,
                    0,
                ]
                return

        # The algorithm only works when the distance is big enough
        if distance < self.get_min_allowed_jerk_distance(
                start_v, end_v, max(accel, decel), jerk):
            # If the speeds are the same, then create a jerk profile
            # without constant acc segments
            # Note that the distance is short enough, so the constant
            # acceleration can never be achieved
            if abs(start_v - end_v) <= self.tolerance:

                # There are two cases, either the maximum speed is reached 
                # or not

                acc_t = max_v - start_v
                acc_t /= jerk
                acc_t = math.sqrt(acc_t)
                jerk_dist = 2*start_v + jerk*acc_t**2
                jerk_dist *= 2*acc_t
                cruise_d = distance - jerk_dist
                if cruise_d >= 0:
                    # The cruise speed is reached
                    cruise_t = cruise_d / max_v
                    self.jerk = jerk
                    self.start_v = start_v
                    self.cruise_v = max_v
                    self.end_v = end_v
                    self.jerk_t = [
                        acc_t,
                        0,
                        acc_t,
                        cruise_t,
                        acc_t,
                        0,
                        acc_t
                    ]
                else:
                    # The cruise speed is not reached
                    a = 2.0*jerk
                    b = 4.0*start_v
                    c = 6.0*jerk
                    t = self.tolerance
                    for _ in range(10):
                        d = a*t**3 + t*b - distance
                        e = c*t**2 + b
                        old_t = t
                        t = old_t - d / e
                        if abs(t - old_t) < self.tolerance:
                            break
                    self.jerk = jerk
                    self.start_v = start_v
                    self.cruise_v = jerk * t**2 + start_v 
                    self.end_v = end_v
                    self.jerk_t = [
                        t,
                        0,
                        t,
                        0,
                        t,
                        0,
                        t
                    ]
            else:
                # When the start and end profiles are different, then create
                # a fully accelerating or decelrating profile
                acc_t = 4.0 * abs(end_v - start_v)
                acc_t /= jerk
                acc_t = math.sqrt(acc_t)
                acc_t /= 2.0
                self.jerk = jerk
                self.start_v = start_v
                self.cruise_v = max(end_v, start_v)
                self.end_v = end_v
                if end_v > start_v:
                    self.jerk_t = [
                        acc_t,
                        0,
                        acc_t,
                        0,
                        0,
                        0,
                        0
                    ]
                else:
                    self.jerk_t = [
                        0,
                        0,
                        0,
                        0,
                        acc_t,
                        0,
                        acc_t
                    ]
            
            return

        if decel is None:
            decel = accel
        self.jerk = jerk
        accel_jerk_t = accel / jerk
        decel_jerk_t = decel / jerk

        # The distance needs fixup
        # The delta is calculated as the difference between the distance
        # traveled by the jerk profile and the actual segment distance
        delta_distance = accel * start_v + decel * max_v + decel * end_v
        delta_distance -= accel * max_v
        delta_distance /= 2.0 * jerk
        fixed_distance = distance - delta_distance

        # Avoid zeros, for the rest of the calculations
        accel = max(accel, self.tolerance)
        decel = max(decel, self.tolerance)
        # Start by a trapezoidal profile
        self.calculate_trapezoidal(fixed_distance, start_v, max_v, end_v, accel,
                                   decel)
        t1 = self.accel_t - accel_jerk_t
        t3 = self.cruise_t - accel_jerk_t
        t5 = self.decel_t - decel_jerk_t
        if max_v - self.cruise_v > self.tolerance:
            return self.calculate_jerk(distance, start_v, self.cruise_v, end_v,
                accel, jerk, decel, last_type)

        if (last_type != 3 and
            (t1 <= -MoveProfile.tolerance or t5 <= -MoveProfile.tolerance)):
            # Type III-a
            if t1 <= -MoveProfile.tolerance:
                delta_v = max_v - start_v
                accel = math.sqrt(jerk * delta_v)

            # Type III-b
            if t5 <= -MoveProfile.tolerance:
                delta_v = max_v - end_v
                decel = math.sqrt(jerk * delta_v)
            
            return self.calculate_jerk(distance, start_v, max_v, end_v, accel,
                jerk, decel, 3)

        # Type II
        if t3 <= -MoveProfile.tolerance:
            # Generate a trapezoidal profile with a cruise time of exactly
            # jerk_t, by solving for cruise_v from
            # distance = accel_d + cruise_d + decel_d
            jerk_t = accel_jerk_t
            jerk_t2 = jerk_t**2
            ad = accel*decel
            start_v2 = start_v**2
            end_v2 = end_v**2
            accel2 = accel**2
            decel2 = decel**2
            distance2 = fixed_distance*2
            max_v = ad*(start_v2 + end_v2)
            max_v += accel2*(decel*distance2 + end_v2)
            max_v += decel2*(accel*distance2 + jerk_t2*accel2 + start_v2)
            max_v = math.sqrt(max_v)
            max_v -= jerk_t*ad
            max_v /= (accel + decel)
            # If the calculated max_v can't be reached, then set it to either
            # the start or end velocity. This is an indication that either the
            # start acceleration or end deceleration is missing
            # A type III adaptation will follow, which changes the fixup
            # distance, so that the velocity actually is reached
            # Note: This case is slightly suboptimal since the move ends with a
            # constant speed segement, rather than deceleration
            max_v = max(max_v, start_v, end_v)
            return self.calculate_jerk(distance, start_v, max_v, end_v, accel,
                jerk, decel, 2)

        # Clamp to zero to remove empty segments
        if t1 < MoveProfile.tolerance:
            t1 = 0
        if t3 < MoveProfile.tolerance:
            t3 = 0
        if t5 < MoveProfile.tolerance:
            t5 = 0

        self.jerk_t[0] = accel_jerk_t
        self.jerk_t[1] = t1
        self.jerk_t[2] = accel_jerk_t
        self.jerk_t[3] = t3
        self.jerk_t[4] = decel_jerk_t
        self.jerk_t[5] = t5
        self.jerk_t[6] = decel_jerk_t

    @staticmethod
    def get_max_allowed_jerk_end_speed(distance, start_v, max_a, jerk):
        tolerance = 1e-6
        max_a_dist = max_a**3 / jerk**2 + 2.0 * max_a * start_v / jerk
        if distance < max_a_dist:
            d2 = distance**2
            jerk_times_d2 = jerk*d2
            def iter(v):
                ve_minus_vs = v - start_v
                vs_plus_ve = start_v + v
                val = ve_minus_vs * vs_plus_ve**2 - jerk_times_d2
                val /= vs_plus_ve * (3.0*v - start_v)
                return v - val

            end_v = start_v + 0.5 * jerk * (max_a / jerk)**2
            for _ in range(10):
                new_v = iter(end_v)
                if abs(end_v - new_v) < tolerance:
                    break
                end_v = new_v

            return new_v
        else:
            end_v = 8.0 * max_a * distance + 4.0 * start_v**2
            end_v *= jerk
            end_v -= 4.0 * max_a**2 * start_v
            end_v *= jerk
            end_v += max_a**4
            end_v = math.sqrt(end_v)
            end_v -= max_a**2
            end_v /= 2.0 * jerk
            return end_v

    @staticmethod
    def can_accelerate_fully(distance, start_v, end_v, accel, jerk):
        jerk_t2 = end_v - start_v 
        jerk_t2 /= jerk
        jerk_t2 *= 2

        # If there's a constant acceleration phase
        if jerk_t2 > (accel / jerk)**2:
            d1 = end_v**2 - start_v**2
            d1 /= 2.0 * accel

            d2 = accel**2 / (12.0*jerk)
            d2 += start_v
            d2 *= accel / (2*jerk)
            d = d1 + d2
        else:
            d = math.sqrt(jerk_t2)
            d *= 2*start_v + end_v
            d /= 3.0
        return d > distance 

    def calculate_jerk_accelerate_only(
            self, distance, start_v, end_v, max_acc, jerk):
        self.start_v = start_v
        self.cruise_v = end_v
        self.end_v = end_v
        self.jerk = jerk
        if end_v < start_v + max_acc**2 / jerk:
            t = math.sqrt(4.0 * (end_v - start_v) / jerk)
            t_div_2 = t / 2.0
            self.jerk_t[0] = t_div_2
            self.jerk_t[1] = 0
            self.jerk_t[2] = t_div_2
        else:
            t_j = max_acc / jerk
            t_c = (end_v - start_v) / max_acc
            t_c -= max_acc / jerk
            self.jerk_t[0] = t_j
            self.jerk_t[1] = t_c
            self.jerk_t[2] = t_j

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
        self.axes_r = tuple((d * inv_move_d for d in axes_d))
        self.min_move_t = move_d / velocity
        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_junction_v2 = 0.
        self.max_start_v2 = 0.
        self.max_cruise_v2 = velocity**2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel

        self.profile = MoveProfile(self.start_pos, self.is_kinematic_move,
            self.axes_r, self.axes_d, self.end_pos)
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
        self.max_junction_v2 = min(
            R * self.accel, R * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            extruder_v2, self.max_cruise_v2, prev_move.max_cruise_v2)
        self.max_start_v2 = min(
            self.max_junction_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)
        self.max_smoothed_v2 = min(
            self.max_start_v2
            , prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)

LOOKAHEAD_FLUSH_TIME = 0.250

class FeedratePlanner(object):
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])
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
    def __init__(self, toolhead):
        super(TrapezoidalFeedratePlanner, self).__init__(toolhead)
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
        
        profiles = (move.profile for move in queue[:flush_count])
        # Generate step times for all moves ready to be flushed
        self.toolhead._process_moves(profiles)
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
            self.profile = None


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

        def calculate_profile(self, profile):
            profile = MoveProfile()
            profile.calculate_jerk(self.distance, self.start_v,
                self.cruise_v, self.end_v, self.accel, self.jerk)
            self.profile = profile
        
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
            t = self.profile.jerk_t[self.current_segment]

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
            t = 0.5 * self.profile.jerk_t[self.current_segment]
            x = self.segment_start_x - d
            v = self.segment_start_v
            a = self.segment_start_a
            j = self.jerk_multipliers[self.current_segment] * self.jerk
            new_x = 0
            new_v = 0

            for _ in range(10):
                new_x = self.calculate_x(x, v, a, j, t)
                new_v = self.calculate_v(v, a, j, t)
                new_t = t - new_x / new_v
                if abs(new_t - t) < tolerance:
                    t = new_t
                    break
                t = new_t
            
            # Return either end or start if t is out of bounds due to 
            # precision issues
            if t > self.profile.jerk_t[self.current_segment]:
                self.x = self.segment_end_x
                self.v = self.segment_end_v
                self.a = self.segment_end_a
                ret = self.profile.jerk_t[self.current_segment] - t
                self.current_segment_offset = \
                    self.profile.jerk_t[self.current_segment]
            elif t > 0.0:
                self.x = new_x
                self.v = new_v
                self.a = self.calculate_a(a, j, t)
                ret = t - self.current_segment_offset
                self.current_segment_offset = t
            else:
                ret = 0


            return ret


    def __init__(self, toolhead, jerk):
        super(JerkFeedratePlanner, self).__init__(toolhead)
        self.virtual_moves = []
        self.jerk = jerk
        self.current_v = 0

    @staticmethod
    def can_combine_with_next(next_move, distance, start_v, end_v, end_v2,
        accel, jerk):
        reachable_end_v = MoveProfile.get_max_allowed_jerk_end_speed(
            distance, start_v, accel, jerk)

        if next_move is None:
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
        
        return (MoveProfile.can_accelerate_fully(distance,
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
                # TODO: Deal with changing acceleration
                v_move = self.VirtualMove(
                    start_v=current_v,
                    accel=move.accel,
                    jerk=self.jerk
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
                v_move.cruise_v = max(v_move.end_v, math.sqrt(move.max_cruise_v2))
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
        if not self.queue:
            return
        tolerance = 1e-9
        self.virtual_moves = []
        self.forward_pass()
        self.backward_pass()
        profiles = []
        flush_count = 0
        move_count = 0
        for vmove in self.virtual_moves:
            vmove.calculate_profile(vmove.profile)
            vmove.calculate_first_segment()

            d = 0

            for move in vmove.moves:
                move_count += 1
                is_kinematic_move = move.is_kinematic_move
                start_pos = move.start_pos
                axes_r = move.axes_r
                axes_d = move.axes_d
                end_pos = move.end_pos

                profile = MoveProfile(start_pos, is_kinematic_move,
                    axes_r=axes_r, axes_d=axes_d, end_pos=end_pos)
                profile.jerk = vmove.jerk

                d += move.move_d

                profile.start_v = vmove.v
                profile.start_a = vmove.a
                cruise_v = vmove.segment_end_v
                at_end = False
                while d >= vmove.segment_end_x - tolerance:
                    s = vmove.current_segment
                    profile.jerk_t[s] = vmove.profile.jerk_t[s] - vmove.current_segment_offset
                    cruise_v = max(cruise_v, vmove.segment_start_v)
                    if s == 6:
                        at_end = True
                        break

                    vmove.calculate_next_segment()

                if d < vmove.segment_end_x - tolerance:
                    profile.jerk_t[vmove.current_segment] = vmove.move_to(d)
                    profile.end_v = vmove.v
                else:
                    profile.end_v = vmove.segment_end_v

                profile.cruise_v = max(cruise_v, vmove.v)

                target_end_v2 = move.max_cruise_v2 
                if move_count < len(self.queue):
                    target_end_v2 = self.queue[move_count].max_junction_v2
                # Flush when the top speed is reached, and there's no
                # acceleration (at a cruise segment, or at the end)
                if vmove.current_segment == 3 or at_end:
                    if abs(profile.end_v**2 - target_end_v2) < tolerance:
                        flush_count = move_count

                profiles.append(profile)
        if not lazy:
            flush_count = move_count
        if profiles and flush_count > 0:
            self.toolhead._process_moves(profiles[:flush_count])
            self.current_v = profiles[flush_count-1].end_v
            del self.queue[:flush_count]
        self.virtual_moves = []