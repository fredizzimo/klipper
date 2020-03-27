# Helpers for testing move profiles
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import pytest
from pytest import assume

def calculate_move(move):
    x = 0
    v = move.start_v
    a = move.start_a
    j = move.jerk
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

    for i, segment in enumerate(move.jerk_t):
        t = segment
        if t:
            j = move.jerk * jerk_multipliers[i]
            x += v * t + 0.5 * a * t**2 + j * t**3 / 6.0
            v += a * t + 0.5 * j * t**2
            a += j * t
        distances.append(x)
        speeds.append(v)
        accs.append(a)
        jerks.append(j)
    return distances, speeds, accs, jerks

def check_jerk_move(move, distance, start_v, cruise_v, end_v, max_accel,
                    max_decel, jerk, high_precision_end_pos=False):
    distances, speeds, accs, _ = calculate_move(move)

    has_acc = (move.jerk_t[0] > 0 or move.jerk_t[1] > 0 or
        move.jerk_t[2] > 0)
    has_dec = (move.jerk_t[4] > 0 or move.jerk_t[5] > 0 or
        move.jerk_t[6] > 0)

    for t in move.jerk_t:
        with assume: assert t >= 0, str(move.jerk_t)
    with assume: assert move.jerk == jerk
    with assume: assert move.start_v >= 0
    with assume: assert pytest.approx(move.start_v) == start_v
    with assume: assert pytest.approx(speeds[2]) == cruise_v
    with assume: assert pytest.approx(speeds[-1]) == end_v
    with assume: assert pytest.approx(accs[0] if has_acc else 0) == max_accel
    with assume: assert pytest.approx(accs[4] if has_dec else 0) == -max_decel
    with assume: assert pytest.approx(distances[-1]) == distance

    if high_precision_end_pos:
        end_pos_tol = 1e-12
    else:
        end_pos_tol = None

    actual_end_pos = tuple(move.start_pos[i] + move.axes_r[i]*distances[-1]
          for i in range(4))
    with assume: 
        assert pytest.approx(move.end_pos, abs=end_pos_tol) == actual_end_pos
    
