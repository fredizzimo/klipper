# Tests for the MoveProfile with jerk movement
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from moveplotter import move_plotter, move_plotter_module
from feedrateplanner import MoveProfile
import pytest

def test_zero_to_zero_with_cruise(move_plotter):
    profile = MoveProfile()
    profile.calculate_jerk(20, 0, 100, 0, 1000, 100000)
    move_plotter.plot(profile)