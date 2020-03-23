# Tests for the smooth extrusion feedrate planner
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from moveplotter import move_plotter, move_plotter_module
from testtoolhead import smooth_extrusion_toolhead as toolhead
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

def test_a_few_moves_with_non_extrusion_between(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000)
    toolhead.move((2, 0, 0, 2), max_speed=100)
    toolhead.move((10, 0, 0, 10), max_speed=100)
    toolhead.move((12, 0, 0, 10), max_speed=100)
    toolhead.move((20, 0, 0, 10), max_speed=100)
    toolhead.move((25, 0, 0, 15), max_speed=100)
    toolhead.move((30, 0, 0, 20), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 6
    toolhead.check_jerk_move(0,
        distance=2,
        start_v=0,
        cruise_v=87.7737682239,
        end_v=87.7737682239,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(2, 0, 0, 2),
        end_pos=(2, 0, 0, 2)
    )
    toolhead.check_jerk_move(1,
        distance=8,
        start_v=87.7737682239,
        cruise_v=100,
        end_v=0,
        max_accel=1563.72835084,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(8, 0, 0, 8),
        end_pos=(10, 0, 0, 10)
    )
    toolhead.check_move(2,
        pos=(10, 0, 0, 10),
        start_v=0,
        cruise_v=89.4427191,
        accel_t=0.04472135955,
        cruise_t=0,
        decel_t=0,
        distance=2,
        axes_r=(1, 0, 0, 0),
        axes_d=(2, 0, 0, 0),
        end_pos=(12, 0, 0, 10)
    )
    toolhead.check_move(3,
        pos=(12, 0, 0, 10),
        start_v=89.4427191,
        cruise_v=100,
        accel_t=0.00527864045,
        cruise_t=0.05,
        decel_t=0.05,
        distance=8,
        axes_r=(1, 0, 0, 0),
        axes_d=(8, 0, 0, 0),
        end_pos=(20, 0, 0, 10)
    )
    toolhead.check_jerk_move(4,
        distance=5,
        start_v=0,
        cruise_v=100,
        end_v=100,
        max_accel=2000,
        max_decel=0,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(5, 0, 0, 5),
        end_pos=(25, 0, 0, 15)
    )
    toolhead.check_jerk_move(5,
        distance=5,
        start_v=100,
        cruise_v=100,
        end_v=0,
        max_accel=0,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(5, 0, 0, 5),
        end_pos=(30, 0, 0, 20)
    )

def test_extrude_only_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000,
        max_extrude_acc=50)
    toolhead.move((0, 0, 0, 2), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=(0, 0, 0, 0),
        start_v=0,
        cruise_v=10,
        accel_t=0.2,
        cruise_t=0,
        decel_t=0.2,
        distance=2,
        axes_r=(0, 0, 0, 1),
        axes_d=(0, 0, 0, 2),
        end_pos=(0, 0, 0, 2)
    )

def test_extrude_only_move_uses_max_acc_to_dec(toolhead):
    # NOTE: That this should probably not be the case, but this is how it works
    # currently
    toolhead.set_limits(
        max_vel=100,
        max_acc=1000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000,
        max_extrude_acc=5000)
    toolhead.move((0, 0, 0, 2), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=(0, 0, 0, 0),
        start_v=0,
        cruise_v=63.2455532034,
        accel_t=0.0126491106407,
        cruise_t=0.018973665961,
        decel_t=0.0126491106407,
        distance=2,
        axes_r=(0, 0, 0, 1),
        axes_d=(0, 0, 0, 2),
        end_pos=(0, 0, 0, 2)
    )

def test_extrude_retract_move_unretract_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000,
        max_extrude_acc=5000)
    toolhead.move((5, 0, 0, 5), max_speed=100)
    toolhead.move((5, 0, 0, 2), max_speed=100)
    toolhead.move((15, 0, 0, 2), max_speed=100)
    toolhead.move((15, 0, 0, 5), max_speed=100)
    toolhead.move((19, 0, 0, 9), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 5
    toolhead.check_jerk_move(0,
        distance=5,
        start_v=0,
        cruise_v=81.9803902719,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(5, 0, 0, 5),
        end_pos=(5, 0, 0, 5)
    )
    toolhead.check_move(1,
        pos=(5, 0, 0, 5),
        start_v=0,
        cruise_v=77.4596669241,
        accel_t=0.0154919333848,
        cruise_t=0.0232379000772,
        decel_t=0.0154919333848,
        distance=3,
        axes_r=(0, 0, 0, -1),
        axes_d=(0, 0, 0, -3),
        end_pos=(5, 0, 0, 2)
    )
    toolhead.check_move(2,
        pos=(5, 0, 0, 2),
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.05,
        decel_t=0.05,
        distance=10,
        axes_r=(1, 0, 0, 0),
        axes_d=(10, 0, 0, 0),
        end_pos=(15, 0, 0, 2)
    )
    toolhead.check_move(3,
        pos=(15, 0, 0, 2),
        start_v=0,
        cruise_v=77.4596669241,
        accel_t=0.0154919333848,
        cruise_t=0.0232379000772,
        decel_t=0.0154919333848,
        distance=3,
        axes_r=(0, 0, 0, 1),
        axes_d=(0, 0, 0, 3),
        end_pos=(15, 0, 0, 5)
    )
    toolhead.check_jerk_move(4,
        distance=4,
        start_v=0,
        cruise_v=71.6515138991,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(4, 0, 0, 4),
        end_pos=(19, 0, 0, 9)
    )

def test_change_to_lower_extrusion_speed(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000,
        max_extrude_acc=5000,
        extruder_corner_v=10)
    toolhead.move((5, 0, 0, 5), max_speed=100)
    toolhead.move((10, 0, 0, 7), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    # NOTE: That the extrusion speed change doesn't effect the toolhead
    # velocity
    toolhead.check_jerk_move(0,
        distance=5,
        start_v=0,
        cruise_v=81.0225497379,
        end_v=16.6666666667,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(5, 0, 0, 5),
        end_pos=(5, 0, 0, 5)
    )
    toolhead.check_jerk_move(1,
        distance=5,
        start_v=16.6666666667,
        cruise_v=81.0225497379,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0.4),
        axes_d=(5, 0, 0, 2),
        end_pos=(10, 0, 0, 7)
    )

def test_change_to_higher_extrusion_speed(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5,
        jerk=100000,
        max_extrude_acc=5000,
        extruder_corner_v=10)
    toolhead.move((5, 0, 0, 2), max_speed=100)
    toolhead.move((10, 0, 0, 7), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    # NOTE: That the extrusion speed change doesn't effect the toolhead
    # velocity
    toolhead.check_jerk_move(0,
        distance=5,
        start_v=0,
        cruise_v=81.0225497379,
        end_v=16.6666666667,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 0.4),
        axes_d=(5, 0, 0, 2),
        end_pos=(5, 0, 0, 2)
    )
    toolhead.check_jerk_move(1,
        distance=5,
        start_v=16.6666666667,
        cruise_v=81.0225497379,
        end_v=0,
        max_accel=2000,
        max_decel=2000,
        jerk=100000,
        is_kinematic_move=True,
        axes_r=(1, 0, 0, 1),
        axes_d=(5, 0, 0, 5),
        end_pos=(10, 0, 0, 7)
    )
