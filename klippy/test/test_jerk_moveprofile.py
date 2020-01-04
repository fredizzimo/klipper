# Tests for the MoveProfile with jerk movement
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from moveplotter import move_plotter, move_plotter_module
from feedrateplanner import MoveProfile
import pytest
from math import sqrt

def calculate_move(profile):
    x = 0
    v = profile.start_v
    a = 0
    j = profile.jerk
    jerk_multipliers = [
        1,
        0,
        -1,
        0,
        -1,
        0,
        1
    ]

    distances = []
    speeds = []
    accs = []
    jerks = []

    for i, segment in enumerate(profile.jerk_t):
        t = segment
        if t:
            j = profile.jerk * jerk_multipliers[i]
            x += v * t + 0.5 * a * t**2 + j * t**3 / 6.0
            v += a * t + 0.5 * j * t**2
            a += j * t
        distances.append(x)
        speeds.append(v)
        accs.append(a)
        jerks.append(j)
    return distances, speeds, accs, jerks

def check_profile(profile, distance, start_v, cruise_v, end_v, max_accel, max_decel, jerk):
    distances, speeds, accs, _ = calculate_move(profile)
    for t in profile.jerk_t:
        assert t >= 0, str(profile.jerk_t)
    assert profile.jerk == jerk
    assert profile.start_v >= 0
    assert pytest.approx(profile.start_v) == start_v
    assert pytest.approx(speeds[2]) == cruise_v
    assert pytest.approx(speeds[-1]) == end_v
    assert pytest.approx(accs[0]) == max_accel
    assert pytest.approx(accs[4]) == -max_decel
    assert pytest.approx(distances[-1]) == distance


def get_min_allowed_distance(v1, v2, a_max, jerk):
    a_max = 1000.0
    v_s = min(v1, v2)
    v_e = max(v1, v2)
    jerk = 100000.0
    distance = v_s*a_max**2 + v_e*a_max**2 - jerk*v_s**2 + jerk*v_e**2
    distance /= 2.0 * a_max*jerk
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
        cruise_v=99.2776922435,
        end_v=30,
        max_accel=654.040690135,
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
        cruise_v=99.286165187,
        end_v=95,
        max_accel=1000,
        max_decel=654.688107957,
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
        cruise_v=99.8007248714,
        end_v=95,
        max_accel=692.872634136,
        max_decel=692.872634136,
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
        cruise_v=99.9397386669,
        end_v=96,
        max_accel=702.832744463,
        max_decel=627.673375801,
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
        cruise_v=99.9395915,
        end_v=95,
        max_accel=627.661652485,
        max_decel=702.822274832,
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
        cruise_v=72.5544399189,
        end_v=70,
        max_accel=1000,
        max_decel=505.414673203,
        jerk=100000
    )

def test_no_deceleration_max_a_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 70, 1000, 100000)
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

def test_no_deceleration_max_a_reached_and_dist_slightly_longer(move_plotter):
    # Note that this seems to be slightly sub-optimal, see the comment in 
    # the implementation type II adapatation for an explanation
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 70, 1000, 100000)
    distance += 0.1
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

def test_no_deceleration_max_a_exactly_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 40, 1000, 100000)
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
    distance = get_min_allowed_distance(30, 35, 1000, 100000)
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

def test_no_deceleration_max_a_not_reached_dist_slightly_longer(move_plotter):
    # Note that this seems to be slightly sub-optimal, see the comment in 
    # the implementation type II adapatation for an explanation
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 35, 1000, 100000)
    distance += 0.1
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

def test_no_deceleration_max_a_not_reached_dist_even_longer(move_plotter):
    # In this case the distance is long enough for an actual acceleration above
    # v_e
    profile = MoveProfile()
    distance = get_min_allowed_distance(30, 35, 1000, 100000)
    distance += 0.2
    profile.calculate_jerk(distance, 30, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=30,
        cruise_v=35.3297131117,
        end_v=35,
        max_accel=730.048841635,
        max_decel=181.580040677,
        jerk=100000
    )

def test_no_acceleration_max_a_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(70, 30, 1000, 100000)
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

def test_no_acceleration_max_a_reached_and_dist_slightly_longer(move_plotter):
    # Note that this seems to be slightly sub-optimal, see the comment in 
    # the implementation type II adapatation for an explanation
    profile = MoveProfile()
    distance = get_min_allowed_distance(70, 30, 1000, 100000)
    distance += 0.1
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

def test_no_acceleration_max_a_exactly_reached(move_plotter):
    profile = MoveProfile()
    distance = get_min_allowed_distance(40, 30, 1000, 100000)
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
    distance = get_min_allowed_distance(35, 30, 1000, 100000)
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

def test_no_acceleration_max_a_not_reached_dist_slightly_longer(move_plotter):
    # Note that this seems to be slightly sub-optimal, see the comment in 
    # the implementation type II adapatation for an explanation
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 30, 1000, 100000)
    distance += 0.1
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

def test_no_acceleration_max_a_not_reached_dist_even_longer(move_plotter):
    # In this case the distance is long enough for an actual acceleration above
    # v_e
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 30, 1000, 100000)
    distance += 0.2
    profile.calculate_jerk(distance, 35, 100, 30, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.2661134255,
        end_v=30,
        max_accel=163.129833424,
        max_decel=725.679917424,
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
        cruise_v=39.009489801,
        end_v=35,
        max_accel=633.205322228,
        max_decel=633.205322228,
        jerk=100000
    )

def test_no_speed_change_no_acc_at_all(move_plotter):
    # NOTE: I don't know if it's possible to accelerate with jerk at all
    # when the distance is too short or if it's a shortcoming of the algorithm
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 35, 1000, 100000)
    profile.calculate_jerk(distance, 35, 100, 35, 1000, 100000)
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

def test_no_speed_change_very_slightly_acc(move_plotter):
    # NOTE: It seems like it could accelerate a bit more, but I haven't checked
    # the math
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 35, 1000, 100000)
    distance += 0.01
    profile.calculate_jerk(distance, 35, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=35,
        cruise_v=35.1425667816,
        end_v=35,
        max_accel=119.401332326,
        max_decel=119.401332326,
        jerk=100000
    )

def test_no_speed_change_no_acc_at_all_very_short_move(move_plotter):
    # NOTE: I don't know if it's possible to accelerate with jerk at all
    # when the distance is too short or if it's a shortcoming of the algorithm
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 35, 1000, 100000)
    distance -= 0.1
    profile.calculate_jerk(distance, 35, 100, 35, 1000, 100000)
    move_plotter.plot(profile)
    # A trapezoidal profile with no speed change is generated
    assert profile.jerk == 0
    assert profile.start_v == 35
    assert profile.cruise_v == 35
    assert profile.end_v == 35
    assert profile.accel_t == 0
    assert profile.decel_t == 0
    assert profile.cruise_t == distance / 35.0

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
    distance = get_min_allowed_distance(35, 35, 1000, 100000)
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
    distance = get_min_allowed_distance(35, 35, 1000, 100000)
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
    # NOTE: I don't know if it's possible to accelerate with jerk at all
    # when the distance is too short or if it's a shortcoming of the algorithm
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 35, 1000, 100000)
    distance -= 0.1
    profile.calculate_jerk(distance, 35, 35, 35, 1000, 100000)
    move_plotter.plot(profile)
    # A trapezoidal profile with no speed change is generated
    assert profile.jerk == 0
    assert profile.start_v == 35
    assert profile.cruise_v == 35
    assert profile.end_v == 35
    assert profile.accel_t == 0
    assert profile.decel_t == 0
    assert profile.cruise_t == distance / 35.0

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
    distance = get_min_allowed_distance(35, 30, 1000, 100000)
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
    distance = get_min_allowed_distance(35, 30, 1000, 100000)
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

def test_forced_decelerate_very_short(move_plotter):
    # This is actually an error condition, the end speed is not reached
    profile = MoveProfile()
    distance = get_min_allowed_distance(35, 30, 1000, 100000)
    distance -= 0.1
    profile.calculate_jerk(distance, 35, 35, 30, 1000, 100000)
    move_plotter.plot(profile)
    # A trapezoidal profile with no speed change is generated
    assert profile.jerk == 0
    assert profile.start_v == 35
    assert profile.cruise_v == 35
    assert profile.end_v == 35
    assert profile.accel_t == 0
    assert profile.decel_t == 0
    assert profile.cruise_t == distance / 35.0