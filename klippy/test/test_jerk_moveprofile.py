# Tests for the MoveProfile with jerk movement
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from moveplotter import move_plotter, move_plotter_module
from feedrateplanner import MoveProfile
import pytest
from math import sqrt
from profile_test_helpers import check_jerk_profile as check_profile


def get_min_allowed_distance_with_const_acc(v1, v2, a_max, jerk):
    v_s = min(v1, v2)
    v_e = max(v1, v2)
    distance = v_s*a_max**2 + v_e*a_max**2 - jerk*v_s**2 + jerk*v_e**2
    distance /= 2.0 * a_max*jerk
    return distance

def get_min_allowed_distance(v1, v2, jerk):
    v_s = min(v1, v2)
    v_e = max(v1, v2)
    distance = sqrt(jerk*(v_e - v_s))
    distance *= v_e + v_s
    distance /= jerk
    return distance

def test_zero_to_zero_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 0, 100, 0, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(
        profile,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=0,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )

def test_higher_to_lower_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 70, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=70,
        cruise_v=100,
        end_v=30,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )

def test_lower_to_higher_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 30, 100, 70, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=30,
        cruise_v=100,
        end_v=70,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )

def test_cruise_to_type2_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(10.5, 0, 100, 0, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=10.5,
        start_v=0,
        cruise_v=97.5914226434,
        end_v=0,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )
    assert profile.jerk_t[3] == 0

def test_cruise_to_type2_adaptation_to_higher(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(10.5, 5, 100, 20, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=10.5,
        start_v=5,
        cruise_v=98.0169888902,
        end_v=20,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )
    assert profile.jerk_t[3] == 0

def test_no_cruise_to_II_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(5, 50, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=50,
        cruise_v=74.5298686029,
        end_v=30,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )
    assert profile.jerk_t[3] == 0

def test_type_III_a_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 95, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=95,
        cruise_v=100,
        end_v=30,
        max_accel=707.106781187,
        max_decel=1000,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0

def test_type_III_b_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 30, 100, 95, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=30,
        cruise_v=100,
        end_v=95,
        max_accel=1000,
        max_decel=707.106781187,
        jerk=100000
    )
    assert profile.jerk_t[5] == 0

def test_type_III_c_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 95, 100, 95, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=95,
        cruise_v=100,
        end_v=95,
        max_accel=707.106781187,
        max_decel=707.106781187,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0
    assert profile.jerk_t[5] == 0

def test_type_ii_adaptation_not_needed_after_type_iii_a(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(6.6, 95, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=6.6,
        start_v=95,
        cruise_v=100,
        end_v=30,
        max_accel=707.106781187,
        max_decel=1000,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0

def test_type_ii_adaptation_not_needed_after_type_iii_b(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(6.6, 30, 100, 95, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=6.6,
        start_v=30,
        cruise_v=100,
        end_v=95,
        max_accel=1000,
        max_decel=707.106781187,
        jerk=100000
    )
    assert profile.jerk_t[5] == 0

def test_type_ii_adaptation_not_needed_after_type_iii_c(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(2.8, 95, 100, 95, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=2.8,
        start_v=95,
        cruise_v=100,
        end_v=95,
        max_accel=707.106781187,
        max_decel=707.106781187,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0
    assert profile.jerk_t[5] == 0

def test_type_III_a_to_type_IIII_a_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(6.4, 95, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=6.4,
        start_v=95,
        cruise_v=99.2966980975,
        end_v=30,
        max_accel=655.492036378,
        max_decel=1000,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0

def test_type_III_b_to_type_IIII_b_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(6.4, 30, 100, 95, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=6.4,
        start_v=30,
        cruise_v=99.2966980975,
        end_v=95,
        max_accel=1000,
        max_decel=655.492036378,
        jerk=100000
    )
    assert profile.jerk_t[5] == 0

def test_type_III_c_to_type_IIII_c_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(2.7, 95, 100, 95, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=2.7,
        start_v=95,
        cruise_v=99.8026170978,
        end_v=95,
        max_accel=693.009170057,
        max_decel=693.009170057,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0
    assert profile.jerk_t[5] == 0

def test_type_III_c_to_type_IIII_c_adaptation_increasing(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(2.6, 95, 100, 96, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=2.6,
        start_v=95,
        cruise_v=99.939862381,
        end_v=96,
        max_accel=702.841545514,
        max_decel=627.683230698,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0
    assert profile.jerk_t[5] == 0

def test_type_III_c_to_type_IIII_c_adaptation_decreasing(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(2.6, 96, 100, 95, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=2.6,
        start_v=96,
        cruise_v=99.939862381,
        end_v=95,
        max_accel=627.683230698,
        max_decel=702.841545514,
        jerk=100000
    )
    assert profile.jerk_t[1] == 0
    assert profile.jerk_t[5] == 0

def test_lower_to_higher_with_no_initial_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(5, 30, 100, 70, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=30,
        cruise_v=81.1684396981,
        end_v=70,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )

def test_higher_to_lower_with_no_initial_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(5, 70, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=70,
        cruise_v=81.1684396981,
        end_v=30,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )

def test_lower_to_higher_with_no_initial_cruise_II_b_adaptation(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(3.5, 30, 100, 70, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=3.5,
        start_v=30,
        cruise_v=72.940614708,
        end_v=70,
        max_accel=1000,
        max_decel=542.274350122,
        jerk=100000
    )

def test_no_deceleration_max_a_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(30, 70, 1000, 100000)
    profile.calculate_jerk(distance, 30, 100, 70, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=70,
        end_v=70,
        max_accel=1000,
        max_decel=0,
        jerk=100000
    )

def test_slight_deceleration_max_a_reached_and_dist_slightly_longer(
        move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(30, 70, 1000, 100000)
    distance += 0.1
    profile.calculate_jerk(distance, 30, 100, 70, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=70.0474224415,
        end_v=70,
        max_accel=1000,
        max_decel=68.8639539118,
        jerk=100000
    )

def test_no_deceleration_max_a_exactly_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(30, 40, 1000, 100000)
    profile.calculate_jerk(distance, 30, 100, 40, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=40,
        end_v=40,
        max_accel=1000,
        max_decel=0,
        jerk=100000
    )

def test_no_deceleration_max_a_not_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 35, 100000)
    profile.calculate_jerk(distance, 30, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=35,
        end_v=35,
        max_accel=707.106781187,
        max_decel=0,
        jerk=100000
    )

def test_slight_deceleration_max_a_not_reached_dist_slightly_longer(
        move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 35, 100000)
    distance += 0.1
    profile.calculate_jerk(distance, 30, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=35.1685746428,
        end_v=35,
        max_accel=718.927996591,
        max_decel=129.836298013,
        jerk=100000
    )

def test_longer_deceleration_max_a_not_reached_dist_even_longer(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 35, 100000)
    distance += 0.2
    profile.calculate_jerk(distance, 30, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=35.5788103203,
        end_v=35,
        max_accel=746.914340488,
        max_decel=240.584770978,
        jerk=100000
    )

def test_no_acceleration_max_a_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(70, 30, 1000, 100000)
    profile.calculate_jerk(distance, 70, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=70,
        cruise_v=70,
        end_v=30,
        max_accel=0,
        max_decel=1000,
        jerk=100000
    )

def test_slight_acceleration_max_a_reached_and_dist_slightly_longer(
        move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(70, 30, 1000, 100000)
    distance += 0.1
    profile.calculate_jerk(distance, 70, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=70,
        cruise_v=70.0474224415,
        end_v=30,
        max_accel=68.8639539118,
        max_decel=1000,
        jerk=100000
    )

def test_no_acceleration_max_a_exactly_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(40, 30, 1000, 100000)
    profile.calculate_jerk(distance, 40, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=40,
        cruise_v=40,
        end_v=30,
        max_accel=0,
        max_decel=1000,
        jerk=100000
    )

def test_no_acceleration_max_a_not_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 30, 100000)
    profile.calculate_jerk(distance, 35, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35,
        end_v=30,
        max_accel=0,
        max_decel=707.106781187,
        jerk=100000
    )

def test_slight_acceleration_max_a_not_reached_dist_slightly_longer(
        move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 30, 100000)
    distance += 0.1
    profile.calculate_jerk(distance, 35, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.1685746428,
        end_v=30,
        max_accel=129.836298013,
        max_decel=718.927996591,
        jerk=100000
    )

def test_longer_acceleration_max_a_not_reached_dist_even_longer(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 30, 100000)
    distance += 0.2
    profile.calculate_jerk(distance, 35, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.5788103203,
        end_v=30,
        max_accel=240.584770978,
        max_decel=746.914340488,
        jerk=100000
    )

def test_no_speed_change_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 35, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=35,
        cruise_v=100,
        end_v=35,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )

def test_no_speed_change_no_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(5, 35, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=35,
        cruise_v=71.8114574787,
        end_v=35,
        max_accel=1000,
        max_decel=1000,
        jerk=100000
    )

def test_no_speed_change_no_full_acc(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(1, 35, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=1,
        start_v=35,
        cruise_v=39.5038375983,
        end_v=35,
        max_accel=671.106369978,
        max_decel=671.106369978,
        jerk=100000
    )

def test_no_speed_change_short(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 35, 1000, 100000)
    profile.calculate_jerk(distance, 35, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.6141752555,
        end_v=35,
        max_accel=247.825595026,
        max_decel=247.825595026,
        jerk=100000
    )

def test_no_speed_change_a_bit_longer(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 35, 1000, 100000)
    distance += 0.01
    profile.calculate_jerk(distance, 35, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.6491295495,
        end_v=35,
        max_accel=254.780209109,
        max_decel=254.780209109,
        jerk=100000
    )

def test_very_short_move_with_cruise_speed_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 35, 1000, 100000)
    distance -= 0.1
    profile.calculate_jerk(distance, 35, 35.1, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.1,
        end_v=35,
        max_accel=100,
        max_decel=100,
        jerk=100000
    )

def test_very_short_move_with_cruise_speed_not_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 35, 1000, 100000)
    distance -= 0.1
    profile.calculate_jerk(distance, 35, 35.4, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.3160177609,
        end_v=35,
        max_accel=177.768883933,
        max_decel=177.768883933,
        jerk=100000
    )

def test_no_speed_change_allowed_long(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 35, 35, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=35,
        cruise_v=35,
        end_v=35,
        max_accel=0,
        max_decel=0,
        jerk=100000
    )

def test_no_speed_change_allowed_medium(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(5, 35, 35, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=35,
        cruise_v=35,
        end_v=35,
        max_accel=0,
        max_decel=0,
        jerk=100000
    )

def test_no_speed_change_allowed_short(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(1, 35, 35, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=1,
        start_v=35,
        cruise_v=35,
        end_v=35,
        max_accel=0,
        max_decel=0,
        jerk=100000
    )

def test_no_speed_change_allowed_shorter(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 35, 1000, 100000)
    profile.calculate_jerk(distance, 35, 35, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35,
        end_v=35,
        max_accel=0,
        max_decel=0,
        jerk=100000
    )

def test_no_speed_change_allowed_even_shorter(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 35, 1000, 100000)
    distance += 0.01
    profile.calculate_jerk(distance, 35, 35, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35,
        end_v=35,
        max_accel=0,
        max_decel=0,
        jerk=100000
    )

def test_no_speed_change_allowed_very_short(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 35, 1000, 100000)
    distance -= 0.1
    profile.calculate_jerk(distance, 35, 35, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35,
        end_v=35,
        max_accel=0,
        max_decel=0,
        jerk=100000
    )

def test_forced_decelerate_long(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 35, 35, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=35,
        cruise_v=35,
        end_v=30,
        max_accel=0,
        max_decel=707.106781187,
        jerk=100000
    )

def test_forced_decelerate_medium(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(5, 35, 35, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=35,
        cruise_v=35,
        end_v=30,
        max_accel=0,
        max_decel=707.106781187,
        jerk=100000
    )

def test_forced_decelerate_short(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(1, 35, 35, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=1,
        start_v=35,
        cruise_v=35,
        end_v=30,
        max_accel=0,
        max_decel=707.106781187,
        jerk=100000
    )

def test_forced_decelerate_shorter(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 30, 1000, 100000)
    distance += 0.01
    profile.calculate_jerk(distance, 35, 35, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35,
        end_v=30,
        max_accel=0,
        max_decel=707.106781187,
        jerk=100000
    )

def test_forced_decelerate_even_shorter(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 30, 1000, 100000)
    profile.calculate_jerk(distance, 35, 35, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35,
        end_v=30,
        max_accel=0,
        max_decel=707.106781187,
        jerk=100000
    )

def test_forced_decelerate_very_short(move_plotter):
    # This is actually an error condition, the end speed is not reached
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(35, 30, 1000, 100000)
    distance -= 0.1
    end_speed = MoveProfile.get_max_allowed_jerk_end_speed(
        distance, 35, 0, -1000, -100000)
    profile.calculate_jerk(distance, 35, 35, end_speed, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35,
        end_v=end_speed,
        max_accel=0,
        max_decel=581.688750598,
        jerk=100000
    )

def test_forced_accelerate(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(30, 35, 1000, 100000)
    end_speed = MoveProfile.get_max_allowed_jerk_end_speed(
        distance, 30, 100, 1000, 100000)
    profile.calculate_jerk(distance, 30, end_speed, end_speed, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=end_speed,
        end_v=end_speed,
        max_accel=743.89154774,
        max_decel=0,
        jerk=100000
    )

def test_forced_accelerate_very_short(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance_with_const_acc(30, 35, 1000, 100000)
    distance -= 0.1
    end_speed = MoveProfile.get_max_allowed_jerk_end_speed(
        distance, 30, 100, 1000, 100000)
    profile.calculate_jerk(distance, 30, end_speed, end_speed, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=end_speed,
        end_v=end_speed,
        max_accel=608.315671506,
        max_decel=0,
        jerk=100000
    )

def test_jerk_to_max_allowed_no_const_acc(move_plotter):
    profile = MoveProfile()
    end_v = profile.get_max_allowed_jerk_end_speed(0.5, 35, 100, 1000, 100000)
    profile.calculate_jerk_accelerate_only(0.5, 35, end_v, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=0.5,
        start_v=35,
        cruise_v=end_v,
        end_v=end_v,
        max_accel=671.106369978,
        max_decel=0,
        jerk=100000
    )

def test_jerk_to_max_allowed_const_acc(move_plotter):
    profile = MoveProfile()
    end_v = profile.get_max_allowed_jerk_end_speed(1, 35, 100, 1000, 100000)
    profile.calculate_jerk_accelerate_only(1, 35, end_v, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=1,
        start_v=35,
        cruise_v=end_v,
        end_v=end_v,
        max_accel=1000,
        max_decel=0,
        jerk=100000
    )

def test_jerk_to_max_allowed_from_zero_no_const_acc(move_plotter):
    profile = MoveProfile()
    end_v = profile.get_max_allowed_jerk_end_speed(0.5, 0, 100, 2000, 100000)
    profile.calculate_jerk_accelerate_only(0.5, 0, end_v, 2000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=0.5,
        start_v=0,
        cruise_v=end_v,
        end_v=end_v,
        max_accel=1709.97594668,
        max_decel=0,
        jerk=100000
    )

def test_very_low_jerk(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(4, 0, 100, 0, 1000, 10)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=4,
        start_v=0,
        cruise_v=3.41995189335,
        end_v=0,
        max_accel=5.84803547643,
        max_decel=5.84803547643,
        jerk=10
    )
