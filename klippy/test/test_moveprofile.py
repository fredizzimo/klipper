# Tests for the MoveProfile
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from moveplotter import move_plotter, move_plotter_module
from feedrateplanner import MoveProfile
import pytest

def calculate_speed(v, a, t):
    return v + a * t

def calculate_cruise_speed(profile):
    return calculate_speed(profile.start_v, profile.accel,
        profile.accel_t)

def calculate_end_speed(profile):
    return calculate_speed(profile.cruise_v, -profile.accel,
        profile.decel_t)

def calculate_distance(profile):
    return (profile.start_v * profile.accel_t +
        0.5 * profile.accel * profile.accel_t**2 +
        profile.cruise_v * (profile.cruise_t + profile.decel_t) -
        0.5 * profile.accel * profile.decel_t**2)

def check_profile(profile, distance, start_v, cruise_v, end_v):
    assert pytest.approx(profile.start_v) == start_v
    assert pytest.approx(profile.cruise_v) == cruise_v
    assert pytest.approx(profile.end_v) == end_v
    assert pytest.approx(calculate_cruise_speed(profile)) == cruise_v
    assert pytest.approx(calculate_end_speed(profile)) == end_v
    assert pytest.approx(calculate_distance(profile)) == distance

def test_zero_to_zero_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(20, 0, 100, 0, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=0,
    )

def test_zero_to_zero_with_no_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(9, 0, 100, 0, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=9,
        start_v=0,
        cruise_v=94.86832980505137,
        end_v=0,
    )

def test_higher_to_lower_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(20, 70, 100, 30, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=70,
        cruise_v=100,
        end_v=30,
    )

def test_higher_to_lower_with_no_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(5, 70, 100, 30, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=70,
        cruise_v=88.8819441732,
        end_v=30,
    )

def test_lower_to_higher_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(20, 30, 100, 70, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=30,
        cruise_v=100,
        end_v=70,
    )

def test_lower_to_higher_with_no_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(5, 30, 100, 70, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=5,
        start_v=30,
        cruise_v=88.8819441732,
        end_v=70,
    )

def test_start_at_cruise_speed(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(20, 100, 100, 20, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=100,
        cruise_v=100,
        end_v=20,
    )

def test_end_at_cruise_speed(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(20, 10, 100, 100, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=20,
        start_v=10,
        cruise_v=100,
        end_v=100,
    )

def test_full_accel(move_plotter):
    profile = MoveProfile()
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2
    profile.calculate_trapezoidal(distance, 10, 100, 60, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=10,
        cruise_v=60,
        end_v=60,
    )

def test_full_decel(move_plotter):
    profile = MoveProfile()
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2
    profile.calculate_trapezoidal(distance, 60, 100, 10, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=60,
        cruise_v=60,
        end_v=10,
    )

def test_too_short_accel(move_plotter):
    # Note that the end speed is modified, so these kind of profiles should not
    # really be generated, however it's good to supports speeds being slightly
    # off due to precision issues
    profile = MoveProfile()
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2 - 1
    profile.calculate_trapezoidal(distance, 10, 100, 60, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=10,
        cruise_v=50.9901951359,
        end_v=50.9901951359,
    )

def test_too_short_decel(move_plotter):
    # Note that the start speed is modified, so these kind of profiles should
    # not really be generated, however it's good to supports speeds being
    # slightly off due to precision issues
    profile = MoveProfile()
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2 - 1
    profile.calculate_trapezoidal(distance, 60, 100, 10, 1000)
    move_plotter.plot(profile)
    check_profile(profile,
        distance=distance,
        start_v=50.9901951359,
        cruise_v=50.9901951359,
        end_v=10,
    )