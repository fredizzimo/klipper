# Tests for moves with trapezoidal movement
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from moveplotter import move_plotter, move_plotter_module
from feedrateplanner import MoveQueue, Move
from move_test_helpers import check_trapezoidal_move as check_move
import pytest

move_queue = MoveQueue(16)

def calculate_trapezoidal(distance, start_v, max_v, end_v, accel):
    start_pos = (0, 0, 0, 0)
    end_pos = (distance, 0, 0, 0)
    move = Move(start_pos, end_pos, max_v, accel, accel, 0, move_queue)
    move.calculate_trapezoidal(start_v, end_v)
    return move

def test_zero_to_zero_with_cruise(move_plotter):
    move = calculate_trapezoidal(20, 0, 100, 0, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=0,
    )

def test_zero_to_zero_with_no_cruise(move_plotter):
    move = calculate_trapezoidal(9, 0, 100, 0, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=9,
        start_v=0,
        cruise_v=94.86832980505137,
        end_v=0,
    )
    assert move.cruise_t == 0

def test_higher_to_lower_with_cruise(move_plotter):
    move = calculate_trapezoidal(20, 70, 100, 30, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=20,
        start_v=70,
        cruise_v=100,
        end_v=30,
    )

def test_higher_to_lower_with_no_cruise(move_plotter):
    move = calculate_trapezoidal(5, 70, 100, 30, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=5,
        start_v=70,
        cruise_v=88.8819441732,
        end_v=30,
    )
    assert move.cruise_t == 0

def test_lower_to_higher_with_cruise(move_plotter):
    move = calculate_trapezoidal(20, 30, 100, 70, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=20,
        start_v=30,
        cruise_v=100,
        end_v=70,
    )

def test_lower_to_higher_with_no_cruise(move_plotter):
    move = calculate_trapezoidal(5, 30, 100, 70, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=5,
        start_v=30,
        cruise_v=88.8819441732,
        end_v=70,
    )
    assert move.cruise_t == 0

def test_start_at_cruise_speed(move_plotter):
    move = calculate_trapezoidal(20, 100, 100, 20, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=20,
        start_v=100,
        cruise_v=100,
        end_v=20,
    )
    assert move.accel_t == 0

def test_end_at_cruise_speed(move_plotter):
    move = calculate_trapezoidal(20, 10, 100, 100, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=20,
        start_v=10,
        cruise_v=100,
        end_v=100,
    )
    assert move.decel_t == 0

def test_full_accel(move_plotter):
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2
    move = calculate_trapezoidal(distance, 10, 100, 60, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=distance,
        start_v=10,
        cruise_v=60,
        end_v=60,
    )
    assert move.cruise_t == 0
    assert move.decel_t == 0

def test_full_decel(move_plotter):
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2
    move = calculate_trapezoidal(distance, 60, 100, 10, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=distance,
        start_v=60,
        cruise_v=60,
        end_v=10,
    )
    assert move.accel_t == 0
    assert move.cruise_t == 0

def test_too_short_accel(move_plotter):
    # Test an acceleration that is very slightly too short for the given
    # conditions. The algorithm should correctly handle that, and clamp the
    # times to zero instead of returning negative times (at the cost of slightly
    # wrong travel distance)
    # The condition tested could occur because of precision loss in other
    # calculations
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2 - 1e-10
    move = calculate_trapezoidal(distance, 10, 100, 60, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=distance,
        start_v=10,
        cruise_v=60,
        end_v=60,
    )
    assert move.cruise_t == 0
    assert move.decel_t == 0

def test_too_short_decel(move_plotter):
    # Test an deleration that is very slightly too short for the given
    # conditions. The algorithm should correctly handle that, and clamp the
    # times to zero instead of returning negative times (at the cost of slightly
    # wrong travel distance)
    # The condition tested could occur because of precision loss in other
    # calculations
    time = 50.0 / 1000
    distance = 10 * time + 0.5 * 1000.0 * time**2 - 1e-10
    move = calculate_trapezoidal(distance, 60, 100, 10, 1000)
    move_plotter.plot(move)
    check_move(move,
        distance=distance,
        start_v=60,
        cruise_v=60,
        end_v=10,
    )
    assert move.accel_t == 0
    assert move.cruise_t == 0
