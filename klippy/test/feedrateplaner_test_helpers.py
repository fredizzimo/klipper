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

class ToolHead(object):
    def __init__(self, FeedratePlanner):
        self.moves = []
        self.feedrate_planner = FeedratePlanner(self)
        self.max_accel = None
        self.max_velocity = None
        self.max_accel_to_decel = None
        self.square_corner_velocity = None
        self.junction_deviation = None
        self.extruder = DummyExtruder()
        self.pos = np.zeros(4)

    def set_limits(self, max_vel, max_acc, max_acc_to_dec,
        square_corner_velocity):
        self.max_accel = max_acc
        self.max_velocity = max_vel
        self.max_accel_to_decel = max_acc_to_dec
        scv2 = square_corner_velocity**2
        self.junction_deviation = scv2 * (sqrt(2.) - 1.) / self.max_accel

    def _process_moves(self, moves):
        self.moves += moves

    def move(self, end_pos, max_speed):
        if type(end_pos) == tuple:
            end = np.array([p for p in end_pos] + [0] * (4-len(end_pos)))
        else:
            end = np.array((end_pos, 0, 0, 0))
        self.feedrate_planner.add_move(Move(self, self.pos, end, max_speed))
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
        assert pytest.approx(move.profile.start_v) == start_v
        assert pytest.approx(move.profile.cruise_v) == cruise_v
        assert pytest.approx(move.profile.accel_t) == accel_t
        assert pytest.approx(move.profile.cruise_t) == cruise_t
        assert pytest.approx(move.profile.decel_t) == decel_t
        assert pytest.approx(get_distance(move)) == distance

def get_distance(move):
    return (move.profile.start_v * move.profile.accel_t +
        0.5 * move.profile.accel * move.profile.accel_t**2 +
        move.profile.cruise_v * (move.profile.cruise_t + move.profile.decel_t) -
        0.5 * move.profile.accel * move.profile.decel_t**2)

@pytest.fixture
def trapezoidal_toolhead(move_plotter, request):
    toolhead = ToolHead(TrapezoidalFeedratePlanner)
    yield toolhead
    move_plotter.plot(toolhead.moves)

@pytest.fixture
def jerk_toolhead(move_plotter, request):
    toolhead = ToolHead(JerkFeedratePlanner)
    yield toolhead
    move_plotter.plot(toolhead.moves)