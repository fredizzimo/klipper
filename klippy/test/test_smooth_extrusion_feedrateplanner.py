# Tests for the smooth extrusion feedrate planner
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from moveplotter import move_plotter, move_plotter_module
from feedrateplanner_test_helpers import smooth_extrusion_toolhead as toolhead
from math import sqrt

def test_single_long_extrusion_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move((100, 0, 0, 100), max_speed=100)
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
        axes_r=(1, 0, 0, 1),
        axes_d=(100, 0, 0, 100),
        end_pos=(100, 0, 0, 100)
        )

def test_single_long_non_extrusion_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move((100, 0, 0, 0), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.95,
        decel_t=0.05,
        distance=100,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(100, 0, 0, 0)
    )

def test_extrusion_move_followed_by_non_extrusion(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move((100, 0, 0, 100), max_speed=100)
    toolhead.move((200, 0, 0, 100), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_jerk_move(0,
        distance=100,
        start_v=0,
        cruise_v=100,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(100, 0, 0, 100),
        end_pos=(100, 0, 0, 100)
        )
    toolhead.check_move(1,
        pos=(100, 0, 0, 100),
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.95,
        decel_t=0.05,
        distance=100,
        axes_r=(1, 0, 0, 0),
        axes_d=(100, 0, 0, 0),
        end_pos=(200, 0, 0, 100)
    )