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
    return calculate_speed(profile.start_v, profile.accel, profile.accel_t)

def calculate_end_speed(profile):
    return calculate_speed(profile.cruise_v, -profile.accel, profile.decel_t)

def calculate_distance(profile):
    return (profile.start_v * profile.accel_t +
        0.5 * profile.accel * profile.accel_t**2 +
        profile.cruise_v * (profile.cruise_t + profile.decel_t) -
        0.5 * profile.accel * profile.decel_t**2)

def check_profile(profile, distance, start_v, cruise_v, end_v):
    assert pytest.approx(profile.start_v) == start_v
    assert pytest.approx(profile.cruise_v) == cruise_v
    assert pytest.approx(profile.start_v) == end_v
    assert pytest.approx(calculate_cruise_speed(profile)) == cruise_v
    assert pytest.approx(calculate_end_speed(profile)) == end_v
    assert pytest.approx(calculate_distance(profile)) == distance

def test_zero_to_zero_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(20, 0, 100, 0, 1000)
    check_profile(profile,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=0,
    )
    move_plotter.plot(profile)

def test_zero_to_zero_with_no_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_trapezoidal(9, 0, 100, 0, 1000)
    check_profile(profile,
        distance=9,
        start_v=0,
        cruise_v=94.86832980505137,
        end_v=0,
    )
    move_plotter.plot(profile)