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