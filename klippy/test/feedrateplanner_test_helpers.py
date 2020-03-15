# Helpers for the feedrate planner tests
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import pytest
from feedrateplanner import (TrapezoidalFeedratePlanner, JerkFeedratePlanner, 
    Move)
from kinematics.extruder import DummyExtruder
from math import sqrt
import numpy as np
from profile_test_helpers import check_jerk_profile
from pytest import assume

class ToolHead(object):
    def __init__(self, FeedratePlanner):
        self.moves = []
        self.feedrate_planner = FeedratePlanner(self)
        self.max_accel = None
        self.jerk = None
        self.max_velocity = None
        self.max_accel_to_decel = None
        self.square_corner_velocity = None
        self.junction_deviation = None
        self.extruder = DummyExtruder()
        self.pos = np.zeros(4)
        self.input_moves = []

    def set_limits(self, max_vel, max_acc, max_acc_to_dec,
        square_corner_velocity, jerk=None):
        self.max_accel = max_acc
        self.max_velocity = max_vel
        self.max_accel_to_decel = max_acc_to_dec
        scv2 = square_corner_velocity**2
        self.junction_deviation = scv2 * (sqrt(2.) - 1.) / self.max_accel
        self.jerk = jerk

    def _process_moves(self, moves):
        self.moves += moves

    def move(self, end_pos, max_speed):
        if type(end_pos) == tuple:
            end = np.array([p for p in end_pos] + [0] * (4-len(end_pos)))
        else:
            end = np.array((end_pos, 0, 0, 0))
        move = Move(self, self.pos, end, max_speed)
        self.input_moves.append(move)
        self.feedrate_planner.add_move(move)
        self.pos = end

    def flush(self, lazy=False):
        self.feedrate_planner.flush(lazy)

    def check_move(self, idx, pos, start_v, cruise_v, accel_t, cruise_t,
        decel_t, distance):
        move = self.moves[idx]
        if type(pos) == tuple:
            pos = tuple([p for p in pos] + [0] * (4-len(pos)))
        else:
            pos = (pos, 0, 0, 0)
        assert move.start_pos == pos
        assert pytest.approx(move.start_v) == start_v
        assert pytest.approx(move.cruise_v) == cruise_v
        assert pytest.approx(move.accel_t) == accel_t
        assert pytest.approx(move.cruise_t) == cruise_t
        assert pytest.approx(move.decel_t) == decel_t
        assert pytest.approx(get_distance(move)) == distance

    def check_jerk_move(self, idx, distance, start_v, cruise_v, end_v,
                        max_accel, max_decel, jerk, is_kinematic_move, axes_r,
                        axes_d, end_pos):
        move = self.moves[idx]
        check_jerk_profile(move, distance, start_v, cruise_v, end_v, max_accel,
                           max_decel, jerk)
        with assume: assert move.is_kinematic_move == is_kinematic_move
        with assume: assert pytest.approx(move.axes_r) == axes_r
        with assume: assert pytest.approx(move.axes_d) == axes_d
        with assume: assert pytest.approx(move.end_pos) == end_pos

def get_distance(move):
    return (move.start_v * move.accel_t +
        0.5 * move.accel * move.accel_t**2 +
        move.cruise_v * (move.cruise_t + move.decel_t) -
        0.5 * move.accel * move.decel_t**2)

@pytest.fixture
def trapezoidal_toolhead(move_plotter, request):
    toolhead = ToolHead(TrapezoidalFeedratePlanner)
    yield toolhead
    move_plotter.plot(toolhead.moves, input_moves=toolhead.input_moves)

@pytest.fixture
def jerk_toolhead(move_plotter, request):
    toolhead = ToolHead(JerkFeedratePlanner)
    yield toolhead
    move_plotter.plot(toolhead.moves, input_moves=toolhead.input_moves)