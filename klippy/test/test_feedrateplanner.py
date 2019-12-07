# Tests for the feedrate planner
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import pytest
from toolhead import MoveQueue, Move

class ToolHead(object):
    def __init__(self):
        self.max_accel = 1000
        self.max_velocity = 200
        self.max_accel_to_decel = self.max_accel * 0.5
        self.moves = []
        self.queue = MoveQueue(self)

    def _process_moves(self, moves):
        self.moves += moves

    def move1d(self, start, end, max_speed):
        self.queue.add_move(Move(self, (start,0,0,0), (end, 0, 0, 0), max_speed))

    def flush(self):
        self.queue.flush()

    def check_move(self, idx, pos, start_v, cruise_v, accel_t, decel_t, distance):
        move = self.moves[idx]
        assert move.start_pos == (pos, 0, 0, 0)
        assert pytest.approx(move.start_v) == start_v
        assert pytest.approx(move.cruise_v) == cruise_v
        assert pytest.approx(move.accel_t) == accel_t
        assert pytest.approx(move.decel_t) == decel_t
        assert pytest.approx(get_distance(move)) == distance

def get_distance(move):
    total_time = move.accel_t + move.cruise_t + move.decel_t
    return (move.start_v * total_time +
        0.5 * move.accel * move.accel_t**2 +
        move.cruise_v * (move.cruise_t + move.decel_t) -
        0.5 * move.accel * move.decel_t**2)


# A move long enough to accelerate to cruise_v
def test_single_long_move():
    toolhead = ToolHead()
    toolhead.move1d(0, 100, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos = 0,
        start_v = 0,
        cruise_v = 100,
        accel_t = 0.1,
        decel_t = 0.1,
        distance = 100)