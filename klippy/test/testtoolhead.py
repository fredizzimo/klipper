# A fake toolhad for unit tests of moves
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import pytest
from feedrateplanner import (TrapezoidalFeedratePlanner, JerkFeedratePlanner,
    SmoothExtrusionFeedratePlanner, Move, MoveQueue)
from kinematics.extruder import DummyExtruder, PrinterExtruder
from math import sqrt
import numpy as np
from move_test_helpers import check_jerk_move
from pytest import assume


class TestToolHead(object):
    def __init__(self, FeedratePlanner):
        self.moves = []
        self.move_queue = MoveQueue(2048)
        self.feedrate_planner = FeedratePlanner(self.move_queue,
            self._process_moves)
        self.max_accel = None
        self.jerk = None
        self.max_velocity = None
        self.max_accel_to_decel = None
        self.square_corner_velocity = None
        self.max_extrude_acc = None
        self.junction_deviation = None
        self.extruder = DummyExtruder()
        self.pos = [0.0, 0.0, 0.0, 0.0]
        self.input_moves = []

    def set_limits(self, max_vel, max_acc, max_acc_to_dec,
        square_corner_velocity, jerk=0, max_extrude_acc=None,
        extruder_corner_v=0):
        self.max_accel = max_acc
        self.max_velocity = max_vel
        self.max_accel_to_decel = max_acc_to_dec
        scv2 = square_corner_velocity**2
        self.junction_deviation = scv2 * (sqrt(2.) - 1.) / self.max_accel
        self.jerk = jerk
        self.max_extrude_acc = max_extrude_acc
        self.extruder.instant_corner_v = extruder_corner_v

    def _process_moves(self, moves):
        self.moves += moves

    def move(self, end_pos, max_speed):
        if type(end_pos) == tuple:
            end = [p for p in end_pos] + [0] * (4-len(end_pos))
        else:
            end = [end_pos, 0, 0, 0]
        max_speed = min(max_speed, self.max_velocity)
        move = Move(self.pos, end, max_speed, self.max_accel,
            self.max_accel_to_decel, self.jerk, self.move_queue)
        # Extrude only move, set extrude acceleration
        if not move.is_kinematic_move:
            move.limit_speed(max_speed, self.max_extrude_acc)

        self.input_moves.append(move)
        self.feedrate_planner.add_move(move, self.junction_deviation,
            self.extruder.instant_corner_v)
        self.pos = end

    def flush(self, lazy=False):
        self.feedrate_planner.flush(lazy)

    def check_move(self, idx, pos, start_v, cruise_v, accel_t, cruise_t,
        decel_t, distance, axes_r=None, axes_d=None, end_pos=None):
        move = self.moves[idx]
        if type(pos) == tuple:
            pos = tuple([p for p in pos] + [0] * (4-len(pos)))
        else:
            pos = (pos, 0, 0, 0)
        with assume: assert move.start_pos == pos
        with assume: assert pytest.approx(move.start_v) == start_v
        with assume: assert pytest.approx(move.cruise_v) == cruise_v
        with assume: assert pytest.approx(move.accel_t) == accel_t
        with assume: assert pytest.approx(move.cruise_t) == cruise_t
        with assume: assert pytest.approx(move.decel_t) == decel_t
        with assume: assert pytest.approx(get_distance(move)) == distance
        if axes_r is not None:
            with assume: assert pytest.approx(move.axes_r) == axes_r
        if axes_d is not None:
            with assume: assert pytest.approx(move.axes_d) == axes_d
        if end_pos is not None:
            with assume: assert pytest.approx(move.end_pos) == end_pos

    def check_jerk_move(self, idx, distance, start_v, cruise_v, end_v,
                        max_accel, max_decel, jerk, is_kinematic_move, axes_r,
                        axes_d, end_pos):
        move = self.moves[idx]
        check_jerk_move(move, distance, start_v, cruise_v, end_v, max_accel,
                           max_decel, jerk, high_precision_end_pos=True)
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
    toolhead = TestToolHead(TrapezoidalFeedratePlanner)
    yield toolhead
    move_plotter.plot(
        toolhead.moves,
        simulated_extrusion=True,
        input_moves=toolhead.input_moves)

@pytest.fixture
def jerk_toolhead(move_plotter, request):
    toolhead = TestToolHead(JerkFeedratePlanner)
    yield toolhead
    move_plotter.plot(
        toolhead.moves,
        simulated_extrusion=True,
        input_moves=toolhead.input_moves)

@pytest.fixture
def smooth_extrusion_toolhead(move_plotter, request):
    toolhead = TestToolHead(SmoothExtrusionFeedratePlanner)
    yield toolhead
    move_plotter.plot(
        toolhead.moves,
        simulated_extrusion=False,
        input_moves=toolhead.input_moves)
