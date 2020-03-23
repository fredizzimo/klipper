# Tests for the feedrate planner
#
# Copyright (C) 2019-2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from moveplotter import move_plotter, move_plotter_module
from testtoolhead import trapezoidal_toolhead as toolhead
from math import sqrt

# A move long enough to accelerate to cruise_v
def test_single_long_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(100, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.95,
        decel_t=0.05,
        distance=100)

# A move short enough to not accelerate to the cruise_v
def test_single_short_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(4, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=2000 * sqrt(2 / (0.5*2000)),
        accel_t=sqrt(2 / (0.5*2000)),
        cruise_t=0,
        decel_t=sqrt(2 / (0.5*2000)),
        distance=4)

# A move the exact lenght to accelerate to cruise_v, but no cruise_phase
def test_single_no_cruise_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(5, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.0,
        decel_t=0.05,
        distance=5)

def test_move_with_accel_decel_limit(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(5, max_speed=100)
    toolhead.flush()
    virt_accel_t = sqrt(2.5 / (0.5*1000))
    cruise_v = 1000 * virt_accel_t
    accel_t = cruise_v / 2000
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=cruise_v,
        accel_t=accel_t,
        cruise_t=0.0353553390593,
        decel_t=accel_t,
        distance=5)

def test_move_with_accel_decel_limit_longer_distance(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(6, max_speed=100)
    toolhead.flush()
    virt_accel_t = sqrt(3 / (0.5*1000))
    cruise_v = 1000 * virt_accel_t
    accel_t = cruise_v / 2000
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=cruise_v,
        accel_t=accel_t,
        cruise_t=0.0387298334621,
        decel_t=accel_t,
        distance=6)

def test_move_with_long_enough_distance_fo_no_accel_decel_limit(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.05,
        decel_t=0.05,
        distance=10)

def test_two_long_moves(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(100, max_speed=100)
    toolhead.move(200, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.975,
        decel_t=0,
        distance=100)
    toolhead.check_move(1,
        pos=100,
        start_v=100,
        cruise_v=100,
        accel_t=0,
        cruise_t=0.975,
        decel_t=0.05,
        distance=100)

def test_short_and_long_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(2, max_speed=100)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    accel1_t=sqrt(2 / (0.5*2000))
    cruise1_v=2000 * accel1_t
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=cruise1_v,
        accel_t=accel1_t,
        cruise_t=0,
        decel_t=0,
        distance=2)
    accel2_t = (100 - cruise1_v) / 2000
    toolhead.check_move(1,
        pos=2,
        start_v=cruise1_v,
        cruise_v=100,
        accel_t=accel2_t,
        cruise_t=0.15,
        decel_t=0.05,
        distance=18)

def test_long_and_short_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(20, max_speed=100)
    toolhead.move(22, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.17,
        decel_t=0.00527864045,
        distance=20)
    toolhead.check_move(1,
        pos=20,
        start_v=89.4427191,
        cruise_v=89.4427191,
        cruise_t=0,
        accel_t=0,
        decel_t=0.04472135955,
        distance=2)

def test_required_decelerate_due_to_upcoming_slow_segment(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.move(12, max_speed=100)
    toolhead.move(20, max_speed=20)
    toolhead.flush()
    assert len(toolhead.moves) == 3
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.071,
        decel_t=0.00417424305044,
        distance=10)
    toolhead.check_move(1,
        pos=10,
        start_v=91.6515138991,
        cruise_v=91.6515138991,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0358257569496,
        distance=2)
    toolhead.check_move(2,
        pos=12,
        start_v=20,
        cruise_v=20,
        accel_t=0,
        cruise_t=0.395,
        decel_t=0.01,
        distance=8)

def test_accelerate_to_a_faster_segment(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=50)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=50,
        accel_t=0.025,
        cruise_t=0.1875,
        decel_t=0,
        distance=10)
    toolhead.check_move(1,
        pos=10,
        start_v=50,
        cruise_v=100,
        accel_t=0.025,
        cruise_t=0.05625,
        decel_t=0.05,
        distance=10)

def test_square_corner(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move((10, 0), max_speed=100)
    toolhead.move((10, 10), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.0500625,
        decel_t=0.0475,
        distance=10)
    toolhead.check_move(1,
        pos=10,
        start_v=5,
        cruise_v=100,
        accel_t=0.0475,
        cruise_t=0.0500625,
        decel_t=0.05,
        distance=10)

def test_lookahead_slow_corner(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move((10, 0), max_speed=100)
    toolhead.move((11, 0.01), max_speed=100)
    toolhead.move((12, 0.02), max_speed=100)
    toolhead.move((10, 0.02), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_move(0,
        pos=(0, 0),
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.0700011300623,
        decel_t=0.00527737701979,
        distance=10)
    toolhead.check_move(1,
        pos=(10, 0),
        start_v=89.4452459604,
        cruise_v=89.4452459604,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0130988501585,
        distance=sqrt(1.0**2 + 0.01**2))
    toolhead.check_move(2,
        pos=(11, 0.01),
        start_v=63.2475456434,
        cruise_v=63.2475456434,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0315097170036,
        distance=sqrt(1.0**2 + 0.01**2))
    toolhead.check_move(3,
        pos=(12, 0.02),
        start_v=0.228111636339,
        cruise_v=63.2457588891,
        accel_t=0.0315088236264,
        cruise_t=0,
        decel_t=0.0316228794446,
        distance=2)

def test_accel_decel_limit_over_multiple_moves_at_end(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(1, max_speed=100)
    toolhead.move(2, max_speed=100)
    toolhead.move(3, max_speed=100)
    toolhead.move(4, max_speed=100)
    toolhead.move(5, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 5
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=63.2455532034,
        accel_t=0.0316227766017,
        cruise_t=0,
        decel_t=0,
        distance=1)
    toolhead.check_move(1,
        pos=1,
        start_v=63.2455532034,
        cruise_v=70.7106781187,
        accel_t=0.00373256245764,
        cruise_t=0.0106066017178,
        decel_t=0,
        distance=1)
    toolhead.check_move(2,
        pos=2,
        start_v=70.7106781187,
        cruise_v=70.7106781187,
        accel_t=0,
        cruise_t=0.0141421356237,
        decel_t=0,
        distance=1)
    toolhead.check_move(3,
        pos=3,
        start_v=70.7106781187,
        cruise_v=70.7106781187,
        accel_t=0,
        cruise_t=0.0106066017178,
        decel_t=0.00373256245764,
        distance=1)
    toolhead.check_move(4,
        pos=4,
        start_v=63.2455532034,
        cruise_v=63.2455532034,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0316227766017,
        distance=1)

def test_accel_decel_limit_over_multiple_moves_at_start(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(1, max_speed=100)
    toolhead.move(2, max_speed=100)
    toolhead.move(3, max_speed=100)
    toolhead.move(4, max_speed=100)
    toolhead.move(5, max_speed=10)
    toolhead.move(15, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 6
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=63.2455532034,
        accel_t=0.0316227766017,
        cruise_t=0,
        decel_t=0,
        distance=1)
    toolhead.check_move(1,
        pos=1,
        start_v=63.2455532034,
        cruise_v=63.6396103068,
        accel_t=0.000197028551711,
        cruise_t=0.015517065476,
        decel_t=0,
        distance=1)
    toolhead.check_move(2,
        pos=2,
        start_v=63.6396103068,
        cruise_v=63.6396103068,
        accel_t=0,
        cruise_t=0.0157134840264,
        decel_t=0,
        distance=1)
    toolhead.check_move(3,
        pos=3,
        start_v=63.6396103068,
        cruise_v=63.6396103068,
        accel_t=0,
        cruise_t=0.00019641855033,
        decel_t=0.0268198051534,
        distance=1)
    toolhead.check_move(4,
        pos=4,
        start_v=10,
        cruise_v=10,
        accel_t=0,
        cruise_t=0.1,
        decel_t=0,
        distance=1)
    toolhead.check_move(5,
        pos=5,
        start_v=10,
        cruise_v=100,
        accel_t=90.0 / 2000,
        cruise_t=0.05025,
        decel_t=100.0 / 2000,
        distance=10)

def test_a_single_move_is_not_lazily_flushed(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(100, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 0
    toolhead.flush()
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.95,
        decel_t=0.05,
        distance=100)

def test_flushing_with_top_speed_reached(
    toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.move(20, max_speed=100)
    toolhead.move(30, max_speed=100)
    toolhead.flush(True)
    # Note, the code is conservative, it could flush the two first moves
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.075,
        decel_t=0,
        distance=10)
    toolhead.move(40, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 2
    toolhead.check_move(1,
        pos=10,
        start_v=100,
        cruise_v=100,
        accel_t=0,
        cruise_t=0.1,
        decel_t=0,
        distance=10)
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_move(2,
        pos=20,
        start_v=100,
        cruise_v=100,
        accel_t=0,
        cruise_t=0.1,
        decel_t=0,
        distance=10)
    toolhead.check_move(3,
        pos=30,
        start_v=100,
        cruise_v=100,
        accel_t=0,
        cruise_t=0.075,
        decel_t=0.05,
        distance=10)

def test_flushing_with_speed_peak_reached(
    toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.move(20, max_speed=10)
    toolhead.move(22, max_speed=100)
    toolhead.flush(True)
    # Note, the code is conservative, it could flush the two first moves
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.05025,
        decel_t=0.045,
        distance=10)
    # Still conservative, one extra move after the peak is needed
    # So no new move is generated
    toolhead.move(23, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 1
    # This creates a new peak at the end, but only one move is flushed
    # Because moves are delayed until the move following the next peak
    toolhead.move(30, max_speed=10)
    toolhead.flush(True)
    assert len(toolhead.moves) == 2
    toolhead.flush()
    assert len(toolhead.moves) == 5

def test_flushing_with_speed_peak_reached2(
    toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.move(20, max_speed=10)
    toolhead.move(21, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.05025,
        decel_t=0.045,
        distance=10)
    # Still conservative, one extra move after the peak is needed
    # So no new move is generated
    toolhead.move(23, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 1
    # This creates a new peak at the end, and therefore two new moves are
    # flushed (until the second last peak)
    toolhead.move(30, max_speed=10)
    toolhead.flush(True)
    assert len(toolhead.moves) == 3
    toolhead.flush()
    assert len(toolhead.moves) == 5
