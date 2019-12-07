# Tests for the feedrate planner
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import pytest
from toolhead import ToolHead


class MCU(object):
    def __init__(self):
        pass

    def is_fileoutput(self):
        return False

    def create_oid(self):
        return 0

    def register_config_callback(self, config):
        pass

    def get_printer(self):
        return Printer()

    def register_stepqueue(self, queue):
        pass

    def add_stepper(self, stepper):
        pass


class Reactor(object):
    def __init__(self):
        pass

    def register_timer(self, handler):
        pass


class Pins(object):
    def __init__(self, mcu):
        self.error = Exception
        self.mcu = mcu
    
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False,
                   share_type=None):
        return {
            "chip": self.mcu,
            "pin": 0,
            "invert": False
        }

    def setup_pin(self, pin_type, pin_params):
        return self.mcu


class StepperEnable(object):
    def __init__(self):
        pass

    def register_stepper(self, stepper, pin):
        pass


class ForceMove(object):
    def __init__(self):
        pass

    def register_stepper(self, stepper):
        pass


class QueryEndstops(object):
    def __init__(self):
        pass

    def register_endstop(self, endstop, name):
        pass


class GCode(object):
    def __init__(self):
        pass

    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        pass


class Printer(object):
    def __init__(self):
        self.config_error = Exception
        self.mcu = MCU()
        self.values = {
            "stepper_enable": StepperEnable(),
            "force_move": ForceMove(),
            "query_endstops": QueryEndstops(),
            "pins": Pins(self.mcu),
            "gcode": GCode(),
            "idle_timeout": None,
            "statistics": None,
            "manual_probe": None,
            "tuning_tower": None,
        }
    
    def get_reactor(self):
        return Reactor()

    def lookup_objects(self, module):
        assert module == "mcu"
        return [("mcu", self.mcu)]

    def lookup_object(self, name, default=None):
        return self.values[name]

    def register_event_handler(self, event, callback):
        pass

    def try_load_module(self, config, section):
        return self.values[section]


class Config(object):
    def __init__(self):
        self.values = {
            "max_velocity": 200.0,
            "max_accel": 2000.0,
            "max_accel_to_decel": 1000.0,
            "square_corner_velocity": 5.0, 
            "buffer_time_low": 1.0,
            "buffer_time_high": 2.0,
            "buffer_time_start": 0.250,
            "move_flush_time": 0.050,
            "kinematics": "cartesian",
            "step_pin": 0,
            "dir_pin": 0,
            "enable_pin": 0,
            "endstop_pin": 0,
            "step_distance": 0.0125,
            "position_endstop": 0.0,
            "position_min": 0.0,
            "position_max": 0.0,
            "homing_speed": 0.0,
            "second_homing_speed": 0.0,
            "homing_retract_dist": 0.0,
            "max_z_velocity": 0.0,
            "max_z_accel": 0.0,
        }
        self.error = Exception

    def get_printer(self):
        return Printer()

    def getfloat(self, option, default=None,
                 minval=None, maxval=None, above=None, below=None):
        return self.values[option]

    def getboolean(self, option, default=None):
        return False

    def get(self, option, default=None):
        return self.values[option]

    def getsection(self, section):
        return self

    def get_name(self):
        return ""

    def has_section(self, name):
        return False


def test_create_toolhead():
    config = Config()
    toolhead = ToolHead(config)