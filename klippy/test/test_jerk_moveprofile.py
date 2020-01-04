# Tests for the MoveProfile with jerk movement
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from moveplotter import move_plotter, move_plotter_module
from feedrateplanner import MoveProfile
import pytest

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
    a_max = 1000.0
    v_s = 30.0
    v_e = 70.0
    jerk = 100000.0
    distance = v_s*a_max**2 + v_e*a_max**2 - jerk*v_s**2 + jerk*v_e**2
    distance /= 2.0 * a_max*jerk
    profile.calculate_jerk(distance, v_s, 100, v_e, a_max, jerk)
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

def test_no_acceleration_max_a_reached(move_plotter):
    profile = MoveProfile()
    a_max = 1000.0
    v_s = 70.0
    v_e = 30.0
    jerk = 100000.0
    distance = v_s*a_max**2 + v_e*a_max**2 + jerk*v_s**2 - jerk*v_e**2
    distance /= 2.0 * a_max*jerk
    profile.calculate_jerk(distance, v_s, 100, v_e, a_max, jerk)
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