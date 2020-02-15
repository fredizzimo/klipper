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
        square_corner_velocity=5)
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
        square_corner_velocity=5)
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
        square_corner_velocity=5)
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
        square_corner_velocity=5)
    toolhead.move(0.5, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_jerk_move(0,
        distance=0.5,
        start_v=0,
        cruise_v=17.9397554578,
        end_v=0,
        max_accel=1339.39372321,
        max_decel=1339.39372321,
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
        square_corner_velocity=5)
    toolhead.move(0.5, max_speed=100)
    toolhead.move(1.0, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=0.5,
        start_v=0,
        cruise_v=28.9897948557,
        end_v=28.9897948557,
        max_accel=1702.63897687,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(0.5, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=0.5,
        start_v=28.9897948557,
        cruise_v=28.9897948557,
        end_v=0,
        max_accel=0,
        max_decel=1702.63897687,
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
        square_corner_velocity=5)
    toolhead.move(0.5, max_speed=100)
    toolhead.move(0.7, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=0.5,
        start_v=0,
        cruise_v=22.5716778697,
        end_v=20.5539663235,
        max_accel=1502.38736249,
        max_decel=635.249800672,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0),
        axes_d=(0.5, 0, 0, 0),
        end_pos=(0.5, 0, 0, 0)
    )
    toolhead.check_jerk_move(1,
        distance=0.2,
        start_v=20.5539663235,
        cruise_v=20.5539663235,
        end_v=0,
        max_accel=-635.249800672,
        max_decel=1502.38736249 ,
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
        square_corner_velocity=5)
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
        square_corner_velocity=5)
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
        square_corner_velocity=5)
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
        square_corner_velocity=5)
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
        square_corner_velocity=5)
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