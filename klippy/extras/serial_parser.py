#!/usr/bin/env python2
# Graph the movement parsed from a serial port dump file
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
from abc import ABCMeta, abstractmethod
import numpy as np
import pandas as pd

from msgproto import MessageParser
from configfile import PrinterConfig
from klippy import Printer

MASK_32_BIT = 0xFFFFFFFF
MASK_32_BIT_HIGH = MASK_32_BIT << 32

# Try to maintain backwards compatibility with this interface, so that
# The klipper dash visualizer tool continues to work when klipper is
# updated
# Similarly if the tool is updated to use new functionality it should 
# check if the functionality is available before attempting to use it.
class SerialParserInterface(object):
    __metaclass__ = ABCMeta
    def __init__(self, input, config, dict):
        pass

    @abstractmethod
    def get_stepper_names(self):
        #type: () -> List(str)
        pass

    @abstractmethod
    def get_steps(self, stepper, start):
        #type (int, float) -> Generator[Tuple[float, float]]
        # Should return a genertor generating tuples of (time, step)
        # starting from the start time for the given stepper number
        pass

class SerialParser(SerialParserInterface):
    def __init__(self, input, config, dict):
        SerialParserInterface.__init__(self, input, config, dict)
        self.printer = self.read_printer_config(config)
        self.steppers = self.parse(input, dict)
        self.stepper_data = self.generate_stepper_data()

    def get_stepper_names(self):
        return [s.name for s in self.steppers]

    def get_steps(self, stepper, start):
        # TODO: Start is ignore for now
        for s in self.steppers[stepper].steps:
            yield (s[0], s[1])

    def generate_stepper_data(self):
        def find_stepper(name):
            for s in self.steppers:
                if s.mcu._name == name:
                    return s
            return None
        
        stepper_x = find_stepper("stepper_x")
        stepper_y = find_stepper("stepper_y")
        stepper_z = find_stepper("stepper_z")
        # Only support 3d graphs if there are x, y and z steppers
        if (stepper_x is not None and stepper_y is not None and
                stepper_z is not None):
            merged = pd.DataFrame(stepper_x.steps, columns=["time", "x"])
            merged = merged.merge(pd.DataFrame(stepper_y.steps, columns=["time", "y"]), how="outer", on="time")
            merged = merged.merge(pd.DataFrame(stepper_z.steps, columns=["time", "z"]), how="outer", on="time")
            merged.sort_values(by="time", inplace=True)
            merged.fillna(method="ffill", inplace=True)
            return merged
        else:
            return pd.DataFrame(columns=["time", "x", "y", "z"])

    def get_spatial_coordinates(self):
        data = self.stepper_data 
        a = np.empty((data.shape[0], 3))
        a[:,0] = data.x
        a[:,1] = data.y
        a[:,2] = data.z
        return a.flatten()

    def get_printer_dimensions(self):
        toolhead = self.printer.lookup_object("toolhead")
        rails = toolhead.kin.rails
        # Not all kinematics types are supported by this at the moment
        if len(rails) == 3:
            return [(rail.position_min, rail.position_max) for rail in rails]
        else:
            return [(0,100), (0,100), (0,100)]

    def parse(self, input, dictionary_file):
        printer = self.printer
        dictionary = dictionary_file.read()
        dictionary_file.close()
        message_parser = MessageParser()
        message_parser.process_identify(dictionary, decompress=False)
        messages = message_parser.parse_file(input)
        clock_freq = message_parser.get_constant_float('CLOCK_FREQ')
        steppers = {}
        endstops = {}
        for m in messages:
            name = m["name"]
            if name == "queue_step":
                steppers[m["oid"]].queue_step(m)
            elif name == "config_stepper":
                stepper = Stepper(m, clock_freq)
                steppers[stepper.oid] = stepper
            elif name == "config_endstop":
                endstops[m["oid"]] = Endstop(m)
            elif name == "endstop_set_stepper":
                endstops[m["oid"]].set_stepper(m, steppers)
            elif name == "reset_step_clock":
                steppers[m["oid"]].reset_step_clock(m)
            elif name == "set_next_step_dir":
                steppers[m["oid"]].set_next_step_dir(m)
            elif name == "endstop_home":
                endstops[m["oid"]].home()
            elif name == "finalize_config":
                self.apply_config(steppers, printer)
        
        start_time = messages[0]["timestamp"]

        for s in steppers.itervalues():
            s.calculate_moves(start_time)

        steppers = sorted(list(steppers.values()), key=lambda x: x.oid) 

        return steppers

    @staticmethod
    def read_printer_config(config_file):
        start_args = {
            "debuginput": True,
            "config_file": config_file.name
        }

        printer = Printer(None, None, start_args)
        printer._read_config()
        return printer

    @staticmethod
    def apply_config(steppers, printer):
        toolhead = printer.lookup_object("toolhead")
        for rail in toolhead.kin.rails:
            for stepper in rail.steppers:
                oid = stepper._oid
                steppers[oid].set_rail(rail)
                steppers[oid].set_mcu(stepper)

        extruders = printer.lookup_objects("extruder")
        for _, extruder in extruders:
            if hasattr(extruder, "stepper"):
                oid = extruder.stepper._oid
                steppers[oid].set_extruder(extruder)

class Stepper(object):
    def __init__(self, message, clock_freq):
        self.oid = message["oid"]
        self.invert_dir = False
        self.step_clock = 0
        self.dir = True
        self.steps = [(0, 0)]
        self.pos = 0
        self.freq = clock_freq
        self.mcu = None
        self.extruder = None
        self.rail = None
        self.is_homing = False
        self.velocity = None
        self.acceleration = None

    @property
    def name(self):
        return self.mcu._name

    def reset_step_clock(self, message):
        self.step_clock = message["clock"]
        # Fixup the step position to the homing pos
        # This does not work properly for all kinematic types
        # it also assumes that there's a reset step clock after homing
        if self.is_homing:
            self.pos = int(self.rail.position_endstop / self.mcu._step_dist)
            self.is_homing = False
            # Generate two steps with the same positions to avoid
            # too high artifical acceleration and speed
            self.steps[-1] = (self.steps[-1][0], self.pos)
            time = self.steps[-1][0] + 1e-12
            self.steps.append((time, self.pos))

    def set_next_step_dir(self, message):
        self.dir = message["dir"]

    def queue_step(self, message):
        interval = message["interval"]
        count = message["count"]
        add = message["add"]
        base_clock = self.get_message_clock(message)
        base_low = base_clock & MASK_32_BIT
        base_high = base_clock & MASK_32_BIT_HIGH

        wrapped_first = self.step_clock + interval
        wrapped_first &= MASK_32_BIT
        if (wrapped_first < base_low):
            base_high += 0x100000000
        t = base_high + wrapped_first - interval
        pos = self.pos
        step = 1 if self.dir else -1
        for _ in range(count):
            t += interval
            self.steps.append((t / self.freq, pos))
            pos += step
            interval += add
        self.pos = pos
        self.step_clock = t & MASK_32_BIT

    def calculate_moves(self, start_time):
        self.steps = np.array(self.steps, dtype=np.float)
        self.steps[0,0] = start_time
        self.steps[:,0] -= start_time
        step_dist = self.mcu._step_dist
        if self.invert_dir:
            step_dist *= -1.0
        self.steps[:,1] *= step_dist

    def get_message_clock(self, message):
        return message["timestamp"]

    def set_mcu(self, mcu):
        self.mcu = mcu
        self.invert_dir = self.mcu.is_dir_inverted()

    def set_rail(self, rail):
        self.rail = rail

    def set_extruder(self, extruder):
        self.extruder = extruder
        self.set_mcu(extruder.stepper)

    def home(self):
        self.is_homing = True

class Endstop(object):
    def __init__(self, message):
        self.stepper = None
        self.steppers = [None]*message["stepper_count"]
    def set_stepper(self, message, steppers):
        stepper = steppers[message["stepper_oid"]]
        pos = message["pos"]
        self.steppers[pos] = stepper
    def home(self):
        for s in self.steppers:
            if s is not None:
                s.home()
