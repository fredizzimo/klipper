# Helpers for testing move profiles
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import pytest
from pytest import assume

def calculate_move(profile):
    x = 0
    v = profile.start_v
    a = profile.start_a
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

def check_jerk_profile(profile, distance, start_v, cruise_v, end_v, max_accel,
                       max_decel, jerk):
    distances, speeds, accs, _ = calculate_move(profile)
    for t in profile.jerk_t:
        with assume: assert t >= 0, str(profile.jerk_t)
    with assume: assert profile.jerk == jerk
    with assume: assert profile.start_v >= 0
    with assume: assert pytest.approx(profile.start_v) == start_v
    with assume: assert pytest.approx(speeds[2]) == cruise_v
    with assume: assert pytest.approx(speeds[-1]) == end_v
    with assume: assert pytest.approx(accs[0]) == max_accel
    with assume: assert pytest.approx(accs[4] if profile.jerk_t[4]>0 else 0) ==\
        -max_decel
    with assume: assert pytest.approx(distances[-1]) == distance

