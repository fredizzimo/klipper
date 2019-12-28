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