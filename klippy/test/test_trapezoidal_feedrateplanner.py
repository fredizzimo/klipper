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
    toolhead.check_trapezoidal_move(0,
        distance=100,
        start_v=0,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(100, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=4,
        start_v=0,
        cruise_v=2000 * sqrt(2 / (0.5*2000)),
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(4, 0, 0, 0),
        end_pos=(4, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=5,
        start_v=0,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(5, 0, 0, 0),
        end_pos=(5, 0, 0, 0)
    )

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
    assert len(toolhead.moves) == 1
    toolhead.check_trapezoidal_move(0,
        distance=5,
        start_v=0,
        cruise_v=cruise_v,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(5, 0, 0, 0),
        end_pos=(5, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=6,
        start_v=0,
        cruise_v=cruise_v,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(6, 0, 0, 0),
        end_pos=(6, 0, 0, 0)
    )

def test_move_with_long_enough_distance_fo_no_accel_decel_limit(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=100,
        start_v=0,
        cruise_v=100,
        end_v=100,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(100, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=100,
        start_v=100,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(200, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=2,
        start_v=0,
        cruise_v=cruise1_v,
        end_v=cruise1_v,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(2, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=18,
        start_v=cruise1_v,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(18, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=89.4427191,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(20, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=2,
        start_v=89.4427191,
        cruise_v=89.4427191,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(22, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=91.6515138991,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=2,
        start_v=91.6515138991,
        cruise_v=91.6515138991,
        end_v=20,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(12, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(2,
        distance=8,
        start_v=20,
        cruise_v=20,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(8, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=50,
        end_v=50,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=10,
        start_v=50,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=5,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=10,
        start_v=5,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(0, 1, 0, 0),
        axes_d=(0, 10, 0, 0),
        end_pos=(10, 10, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=89.4452459604,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=1.00004999875,
        start_v=89.4452459604,
        cruise_v=89.4452459604,
        end_v=63.2475456434,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(0.99995000375, 0.0099995000375, 0, 0),
        axes_d=(1, 0.01, 0, 0),
        end_pos=(11, 0.01, 0, 0)
    )
    toolhead.check_trapezoidal_move(2,
        distance=1.00004999875,
        start_v=63.247545643,
        cruise_v=63.2475456434,
        end_v=0.228111636339,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(0.99995000375, 0.0099995000375, 0, 0),
        axes_d=(1, 0.01, 0, 0),
        end_pos=(12, 0.02, 0, 0)
    )
    toolhead.check_trapezoidal_move(3,
        distance=2,
        start_v=0.228111636339,
        cruise_v=63.2457588891,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(-1, 0, 0, 0),
        axes_d=(-2, 0, 0, 0),
        end_pos=(10, 0.02, 0, 0)
    )


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
    toolhead.check_trapezoidal_move(0,
        distance=1,
        start_v=0,
        cruise_v=63.2455532034,
        end_v=63.2455532034,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(1, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=1,
        start_v=63.2455532034,
        cruise_v=70.7106781187,
        end_v=70.7106781187,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(2, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(2,
        distance=1,
        start_v=70.7106781187,
        cruise_v=70.7106781187,
        end_v=70.7106781187,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(3, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(3,
        distance=1,
        start_v=70.7106781187,
        cruise_v=70.7106781187,
        end_v=63.2455532034,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(4, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(4,
        distance=1,
        start_v=63.2455532034,
        cruise_v=63.2455532034,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(5, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=1,
        start_v=0,
        cruise_v=63.2455532034,
        end_v=63.2455532034,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(1, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(1,
        distance=1,
        start_v=63.2455532034,
        cruise_v=63.6396103068,
        end_v=63.6396103068,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(2, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(2,
        distance=1,
        start_v=63.6396103068,
        cruise_v=63.6396103068,
        end_v=63.6396103068,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(3, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(3,
        distance=1,
        start_v=63.6396103068,
        cruise_v=63.6396103068,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(4, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(4,
        distance=1,
        start_v=10,
        cruise_v=10,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(5, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(5,
        distance=10,
        start_v=10,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(15, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=100,
        start_v=0,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(100, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=100,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.move(40, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 2
    toolhead.check_trapezoidal_move(1,
        distance=10,
        start_v=100,
        cruise_v=100,
        end_v=100,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_trapezoidal_move(2,
        distance=10,
        start_v=100,
        cruise_v=100,
        end_v=100,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(30, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(3,
        distance=10,
        start_v=100,
        cruise_v=100,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(40, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
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
    toolhead.check_trapezoidal_move(1,
        distance=10,
        start_v=10,
        cruise_v=10,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.flush()
    assert len(toolhead.moves) == 5
    toolhead.check_trapezoidal_move(2,
        distance=2,
        start_v=10,
        cruise_v=78.102496759,
        end_v=64.0312423743,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(22, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(3,
        distance=1,
        start_v=64.0312423743,
        cruise_v=64.0312423743,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(23, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(4,
        distance=7,
        start_v=10,
        cruise_v=10,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(7, 0, 0, 0),
        end_pos=(30, 0, 0, 0)
    )

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
    toolhead.check_trapezoidal_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
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
    toolhead.check_trapezoidal_move(1,
        distance=10,
        start_v=10,
        cruise_v=10,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(2,
        distance=1,
        start_v=10,
        cruise_v=64.0312423743,
        end_v=64.0312423743,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(21, 0, 0, 0)
    )
    toolhead.flush()
    assert len(toolhead.moves) == 5
    toolhead.check_trapezoidal_move(3,
        distance=2,
        start_v=64.0312423743,
        cruise_v=78.1024967591,
        end_v=10,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(23, 0, 0, 0)
    )
    toolhead.check_trapezoidal_move(4,
        distance=7,
        start_v=10,
        cruise_v=10,
        end_v=0,
        accel=2000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(7, 0, 0, 0),
        end_pos=(30, 0, 0, 0)
    )
