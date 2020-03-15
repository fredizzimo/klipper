# Tests for the jerk feedrate planner
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from moveplotter import move_plotter, move_plotter_module
from feedrateplanner_test_helpers import jerk_toolhead as toolhead
from math import sqrt

def test_single_long_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(100, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_jerk_move(0,
        distance=100,
        start_v=0,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(100, 0, 0, 0)
        )

def test_accel_decel_limit_is_not_in_use(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(5, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_jerk_move(0,
        distance=5,
        start_v=0,
        cruise_v=81.9803902719,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(5.0, 0, 0, 0),
        end_pos=(5.0, 0, 0, 0)
    )

def test_two_long_moves(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(100, max_speed=100)
    toolhead.move(200, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=100,
        start_v=0,
        cruise_v=100,
        end_v=100,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(100, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=100,
        start_v=100,
        cruise_v=100,
        end_v=0,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(200, 0, 0, 0)
    )

def test_single_short_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(0.5, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_jerk_move(0,
        distance=0.5,
        start_v=0,
        cruise_v=18.4201574932,
        end_v=0,
        max_accel=1357.2088083,
        max_decel=1357.2088083,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(0.5, 0, 0, 0)
    )

def test_two_combined_short_moves(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(0.5, max_speed=100)
    toolhead.move(1.0, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=0.5,
        start_v=0,
        cruise_v=29.2401773821,
        end_v=29.2401773821,
        max_accel=1709.97594668,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(0.5, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=0.5,
        start_v=29.2401773821,
        cruise_v=29.2401773821,
        end_v=0,
        max_accel=0,
        max_decel=1709.97594668,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(1.0, 0, 0, 0)
    )

def test_two_combined_asymetric_short_moves(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(0.5, max_speed=100)
    toolhead.move(0.7, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=0.5,
        start_v=0,
        cruise_v=23.0521814603,
        end_v=20.7895052431,
        max_accel=1518.29448594,
        max_decel=672.707398081,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(0.5, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=0.2,
        start_v=20.7895052431,
        cruise_v=20.7895052431,
        end_v=0,
        max_accel=0,
        max_decel=1518.29448594,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.2, 0, 0, 0),
        end_pos=(0.7, 0, 0, 0)
    )

def test_moves_with_different_velocities(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(5, max_speed=50)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=5,
        start_v=0,
        cruise_v=50,
        end_v=50,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(5, 0, 0, 0),
        end_pos=(5, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=15,
        start_v=50,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(15, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

def test_combine_acceleration_with_different_velocities_const_seg(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(0.5, max_speed=50)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=0.5,
        start_v=0,
        cruise_v=43.2049379894,
        end_v=43.2049379894,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(0.5, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=19.5,
        start_v=43.2049379894,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(19.5, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

def test_combine_acceleration_with_different_velocities_no_const_seg(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(0.1, max_speed=50)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=0.1,
        start_v=0,
        cruise_v=16.5096362445,
        end_v=16.5096362445,
        max_accel=1817.12059283,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.1, 0, 0, 0),
        end_pos=(0.1, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=19.9,
        start_v=16.5096362445,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(19.9, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

def test_dont_comb_acc_when_it_would_violate_vel_limit_const_phase(toolhead):
    # The velocity limit would be violated, when there's a constant acceleration
    # phase
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(1, max_speed=50)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=1,
        start_v=0,
        cruise_v=46.3324958071,
        end_v=46.3324958071,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(1, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=19,
        start_v=46.3324958071,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(19, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

def test_dont_comb_acc_when_it_would_violate_vel_limit_no_const_phase(toolhead):
    # The velocity limit would be violated, when there's no constant
    # acceleration phase
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(0.2, max_speed=20)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=0.2,
        start_v=0,
        cruise_v=15.8740105197,
        end_v=15.8740105197,
        max_accel=1259.92104989,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.2, 0, 0, 0),
        end_pos=(0.2, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=19.8,
        start_v=15.8740105197,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(19.8, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

def test_slowdown_for_next_segment(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(20, max_speed=100)
    toolhead.move(25, max_speed=50)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=50,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(20, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=5,
        start_v=50,
        cruise_v=50,
        end_v=0,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(5, 0, 0, 0),
        end_pos=(25, 0, 0, 0)
    )

def test_limit_backward_pass_speed(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(20, max_speed=100)
    toolhead.move(21, max_speed=50)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=46.3324958071,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(20, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=1,
        start_v=46.3324958071,
        cruise_v=46.3324958071,
        end_v=0,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(21, 0, 0, 0)
    )

def test_combine_backwards(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(20, max_speed=100)
    toolhead.move(20.5, max_speed=50)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=20,
        start_v=0,
        cruise_v=100,
        end_v=43.2049379894,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(20, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=0.5,
        start_v=43.2049379894,
        cruise_v=43.2049379894,
        end_v=0,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(20.5, 0, 0, 0)
    )
    
def test_a_single_move_is_not_lazily_flushed(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(100, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 0
    toolhead.flush()
    toolhead.check_jerk_move(0,
        distance=100,
        start_v=0,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(100, 0, 0, 0)
    )

def test_flush_when_top_speed_reached(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(10, max_speed=100)
    toolhead.move(20, max_speed=100)
    toolhead.move(30, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=100,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=10,
        start_v=100,
        cruise_v=100,
        end_v=100,
        max_accel=0,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.move(40, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 3
    toolhead.check_jerk_move(2,
        distance=10,
        start_v=100,
        cruise_v=100,
        end_v=100,
        max_accel=0,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(30, 0, 0, 0)
    )
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_jerk_move(3,
        distance=10,
        start_v=100,
        cruise_v=100,
        end_v=0,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
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
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(10, max_speed=100)
    toolhead.move(20, max_speed=10)
    toolhead.move(22, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=10,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=10,
        start_v=10,
        cruise_v=10,
        end_v=10,
        max_accel=0,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

    toolhead.move(23, max_speed=100)
    toolhead.flush(True)
    # This move didn't reach the max speed, so nothing can be flushed
    assert len(toolhead.moves) == 2

    toolhead.move(30, max_speed=10)
    toolhead.flush(True)
    # The last move is not flushed, because we don't know what the next one is
    assert len(toolhead.moves) == 4
    toolhead.check_jerk_move(2,
        distance=2,
        start_v=10,
        cruise_v=58.1024967591,
        end_v=54.2295896854,
        max_accel=2000,
        max_decel=880.103070519,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(22, 0, 0, 0)
    )
    toolhead.check_jerk_move(3,
        distance=1,
        start_v=54.2295896854,
        cruise_v=54.2295896854,
        end_v=10,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(23, 0, 0, 0)
    )
    toolhead.flush()
    assert len(toolhead.moves) == 5
    toolhead.check_jerk_move(4,
        distance=7,
        start_v=10,
        cruise_v=10,
        end_v=0,
        max_accel=0,
        max_decel=1000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(7, 0, 0, 0),
        end_pos=(30, 0, 0, 0)
    )

def test_flushing_with_speed_peak_reached2(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(10, max_speed=100)
    toolhead.move(20, max_speed=10)
    toolhead.move(21, max_speed=100)
    toolhead.flush(True)
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=10,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=10,
        start_v=10,
        cruise_v=10,
        end_v=10,
        max_accel=0,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )
    toolhead.move(23, max_speed=100)
    toolhead.flush(True)
    # This move didn't reach the max speed, so nothing can be flushed
    assert len(toolhead.moves) == 2
    toolhead.move(30, max_speed=10)
    toolhead.flush(True)
    # The last move is not flushed, because we don't know what the next one is
    assert len(toolhead.moves) == 4
    toolhead.check_jerk_move(2,
        distance=1,
        start_v=10,
        cruise_v=54.2295896854,
        end_v=54.2295896854,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(21, 0, 0, 0)
    )
    toolhead.check_jerk_move(3,
        distance=2,
        start_v=54.2295896854,
        cruise_v=58.102496759,
        end_v=10,
        max_accel=880.103070519,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(23, 0, 0, 0)
    )
    toolhead.flush()
    assert len(toolhead.moves) == 5
    toolhead.check_jerk_move(4,
        distance=7,
        start_v=10,
        cruise_v=10,
        end_v=0,
        max_accel=0,
        max_decel=1000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(7, 0, 0, 0),
        end_pos=(30, 0, 0, 0)
    )

def test_dont_flush_when_still_accelerating(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    # Calculate the exact distance to accelerate to the first segment speed 
    speed = 20.0
    jerk_t2 = speed
    jerk_t2 /= 100000
    jerk_t2 *= 2
    d = sqrt(jerk_t2)
    d *= speed
    d /= 3.0
    d-=1e-16
    toolhead.move(d, max_speed=speed)
    toolhead.move(5, max_speed=100)
    toolhead.flush(True)
    # The top speed is reached for the first move, but it's still accelerating
    # The second move does not reach the top speed, so no moves should be 
    # output. 
    assert len(toolhead.moves) == 0
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 3
    toolhead.check_jerk_move(0,
        distance=d,
        start_v=0,
        cruise_v=speed,
        end_v=speed,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(d, 0, 0, 0),
        end_pos=(d, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=5-d,
        start_v=speed,
        cruise_v=100,
        end_v=100,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(5-d, 0, 0, 0),
        end_pos=(5, 0, 0, 0)
    )
    toolhead.check_jerk_move(2,
        distance=15,
        start_v=100,
        cruise_v=100,
        end_v=0,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(15, 0, 0, 0),
        end_pos=(20, 0, 0, 0)
    )

def test_square_corner(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move((10, 0), max_speed=100)
    toolhead.move((10, 10), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=5,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=10,
        start_v=5,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
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
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move((10, 0), max_speed=100)
    toolhead.move((11, 0.01), max_speed=100)
    toolhead.move((12, 0.02), max_speed=100)
    toolhead.move((10, 0.02), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_jerk_move(0,
        distance=10,
        start_v=0,
        cruise_v=100,
        end_v=87.7351033589,
        max_accel=2000,
        max_decel=1566.19900658,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=sqrt(1**2 + 0.01**2),
        start_v=87.7351033589,
        cruise_v=87.7351033589,
        end_v=62.1111441783,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(0.99995000375, 0.0099995000375, 0, 0),
        axes_d=(1, 0.01, 0, 0),
        end_pos=(11, 0.01, 0, 0)
    )
    toolhead.check_jerk_move(2,
        distance=sqrt(1**2 + 0.01**2),
        start_v=62.1111441783,
        cruise_v=62.1111441783,
        end_v=0.228111636339,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(0.99995000375, 0.0099995000375, 0, 0),
        axes_d=(1, 0.01, 0, 0),
        end_pos=(12, 0.02, 0, 0)
    )
    toolhead.check_jerk_move(3,
        distance=2,
        start_v=0.228111636339,
        cruise_v=46.2982939806,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(-1, 0, 0, 0),
        axes_d=(-2, 0, 0, 0),
        end_pos=(10, 0.02, 0, 0)
    )

def test_very_low_jerk(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=10)
    toolhead.move(10, max_speed=100)
    toolhead.move(11, max_speed=100)
    toolhead.move(12, max_speed=10)
    toolhead.move(15, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_jerk_move(0,
        distance=10,
        start_v=0,
        cruise_v=8.25481812224,
        end_v=7.77803510691,
        max_accel=9.08560296416,
        max_decel=3.08798644855,
        jerk=10,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(10, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=1,
        start_v=7.77803510691,
        cruise_v=7.77803510691,
        end_v=7.28085678388,
        max_accel=0,
        max_decel=4.41352770095,
        jerk=10,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(11, 0, 0, 0)
    )
    toolhead.check_jerk_move(2,
        distance=1,
        start_v=7.28085678388,
        cruise_v=7.28085678388,
        end_v=6.53957630692,
        max_accel=0,
        max_decel=5.85703306345,
        jerk=10,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(12, 0, 0, 0)
    )
    toolhead.check_jerk_move(3,
        distance=3,
        start_v=6.53957630692,
        cruise_v=6.53957630692,
        end_v=0,
        max_accel=0,
        max_decel=9.08560296416,
        jerk=10,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(3, 0, 0, 0),
        end_pos=(15, 0, 0, 0)
    )

def test_short_moves_near_low_speed_segment(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(0.1, max_speed=100)
    toolhead.move(0.11, max_speed=100)
    toolhead.move(0.12, max_speed=10)
    toolhead.move(15, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_jerk_move(0,
        distance=0.1,
        start_v=0,
        cruise_v=10.023172396,
        end_v=10.023172396,
        max_accel=1001.16097153,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.1, 0, 0, 0),
        end_pos=(0.1, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=0.01,
        start_v=10.023172396,
        cruise_v=10.0232329092,
        end_v=10,
        max_accel=3.47888428402,
        max_decel=48.2005282117,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.01, 0, 0, 0),
        end_pos=(0.11, 0, 0, 0)
    )
    toolhead.check_jerk_move(2,
        distance=0.01,
        start_v=10,
        cruise_v=10,
        end_v=10,
        max_accel=0,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.01, 0, 0, 0),
        end_pos=(0.12, 0, 0, 0)
    )
    toolhead.check_jerk_move(3,
        distance=14.88,
        start_v=10,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(14.88, 0, 0, 0),
        end_pos=(15, 0, 0, 0)
    )

def test_changing_acceleration_does_not_combine_moves(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(2, max_speed=100)
    toolhead.set_limits(
        max_vel=100,
        max_acc=1000,
        max_acc_to_dec=1000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move(3, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=2,
        start_v=0,
        cruise_v=49.7225535338,
        end_v=40,
        max_accel=2000,
        max_decel=986.030097603,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(2, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=1,
        start_v=40,
        cruise_v=40,
        end_v=0,
        max_accel=0,
        max_decel=1000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(1, 0, 0, 0),
        end_pos=(3, 0, 0, 0)
    )
