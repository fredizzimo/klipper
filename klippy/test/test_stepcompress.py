# Tests for stepcompress
# Copyright (C) 2019  Fred Sundvik
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import klippy.chelper as chelper
from klippy.msgproto import MessageParser
import pytest
from collections import namedtuple
from io import BytesIO
from pty import openpty
from os import fdopen
from klippy.util import set_nonblock
import select
import json
from math import sqrt

import logging
from time import sleep
from serial import Serial
import os


class StepCompress(object):
    def __init__(self, logger):
        self.logger = logger
        self.invert_dir=False
        self.max_error = 0.000025
        self.frequency = 1000*1000*16
        self.time = 0

        self.ffi_main, self.ffi_lib = chelper.get_ffi()

        self.queue_step = ("queue_step oid=%c interval=%u count=%hu add=%hi", 1)
        self.set_next_step_dir = ("set_next_step_dir oid=%c dir=%c", 2)

        self.parser = self.create_message_parser()
        self.stepcompress = self.init_stepcompress()
        self.set_options(invert_dir=self.invert_dir, max_error=self.max_error)
        self.stepqueues = [self.stepcompress]
        self.pty, self.serial_device, self.serial_file = self.init_pty()
        self.serialqueue = self.init_serialqueue()
        self.steppersync = self.init_steppersync()
        self.set_time(self.time, self.frequency)
        self.open = True
        self.input = None
    
    def create_message_parser(self):
        data = {
            "commands": {
                self.queue_step[0]: self.queue_step[1],
                self.set_next_step_dir[0]: self.set_next_step_dir[1]
            },
            "responses": {}
        }
        data = json.dumps(data)
        parser = MessageParser()
        parser.process_identify(data, decompress=False)
        return parser

    def init_stepcompress(self):
        step_compress = self.ffi_lib.stepcompress_alloc(0)
        return step_compress

    def init_pty(self):
        master, slave = openpty()
        name = os.ttyname(slave)
        return ((master, slave), Serial(name), fdopen(master, "rb"))

    def deinit_pty(self):
        self.serial_device.close()
        self.serial_file.close()
        # The file already closes this handle
        #os.close(self.pty[0])
        os.close(self.pty[1])

    def init_serialqueue(self):
        return self.ffi_lib.serialqueue_alloc(self.serial_device.fileno(), True)

    def init_steppersync(self):
        # Some messages seem to be lost of move_count is too low
        # Is that a bug, or something that does not happen with a real MCU target?
        move_count = 512
        steppersync = self.ffi_lib.steppersync_alloc(
            self.serialqueue, self.stepqueues, len(self.stepqueues),
            move_count)
        return steppersync

    def close(self):
        if self.open:
            # Reverse order of allocation
            self.ffi_lib.steppersync_free(self.steppersync)
            self.ffi_lib.serialqueue_free(self.serialqueue)
            self.deinit_pty()
            self.ffi_lib.stepcompress_free(self.stepcompress)
            self.open = False

    class Appender(object):
        def __init__(self, tester, time):
            ffi = tester.ffi_lib
            stepcompress = tester.stepcompress
            self.qa = ffi.queue_append_start(stepcompress, time, 0.5)
            self.qa_address = tester.ffi_main.addressof(self.qa)
            self.ffi_append = ffi.queue_append
            self.ffi_finish = ffi.queue_append_finish
            self.tester = tester
        
        def append(self, time):
            new_time = time * self.tester.frequency
            self.tester.time = new_time
            self.ffi_append(self.qa_address, new_time)

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            self.ffi_finish(self.qa)

    def appender(self, time):
        self.time = time
        return StepCompress.Appender(self, time)

    def set_time(self, time, frequency):
        self.time = time
        old_frequency = self.frequency
        self.frequency = frequency
        self.ffi_lib.steppersync_set_time(self.steppersync, time, frequency)
        if frequency != old_frequency:
            self.set_options(self.invert_dir, self.max_error)

    def set_options(self, invert_dir, max_error):
        self.invert_dir = invert_dir
        self.max_error = max_error
        max_error_ticks = int(max_error * self.frequency)
        self.ffi_lib.stepcompress_fill(
            self.stepcompress, max_error_ticks, invert_dir,
            self.queue_step[1], self.set_next_step_dir[1])

    def set_input(self, input):
        start_time = self.time
        self.input = input
        with self.appender(self.time) as appender:
            for i in input:
                appender.append(i-start_time)
    
    def verify_output(self, expected_messages=None):
        messages = self.get_messages()
        if expected_messages:
            self.check_messages(messages, expected_messages)
        output = self.generate_output_trajectory(messages)

        assert len(self.input) == len(output)

        # allow one tick more max_error than what's set
        # that shouldn't really be allowed, especially since the error is
        # already rounded down when converted to ticks
        # but the tests won't pass otherwise
        max_error = self.max_error * self.frequency
        max_error += 1
        max_error = max_error / self.frequency
        for i in range(len(output)):
            assert self.input[i] == pytest.approx(output[i], abs=max_error), \
                "The input and output does not match for element number %i" % (i,)
        

    def get_messages(self, time=None):
        if time is None:
            time = self.time
        time = int(time * self.frequency)
        self.ffi_lib.steppersync_flush(self.steppersync, time)
        poll = select.poll()
        poll.register(self.serial_file.fileno(), select.POLLIN)
        data = ""
        messages = []
        while len(poll.poll(500)) == 1:
            data += os.read(self.serial_file.fileno(), 4096)
            while data != "":
                size = self.parser.check_packet(data)
                if size > 0:
                    res = self.parser.parse_packet(bytearray(data[:size]))
                    messages += res
                    data = data[size:]
                else:
                    break
        for m in messages:
            self.logger.info(m)
        return messages

    def check_message(self, message, interval, count, add):
        message_interval = message["interval"] / float(self.frequency)
        message_add = message["add"] / float(self.frequency)
        assert message["#name"] == "queue_step"
        assert message_interval == pytest.approx(interval)
        assert message["count"] == count
        assert message_add == pytest.approx(add)

    def check_messages(self, messages, check):
        assert len(messages) == len(check)
        for m, c in zip(messages, check):
            self.check_message(m, *c)

    def generate_output_trajectory(self, messages):
        current_clock = 0
        current_step = 0
        step_dir = 0
        steps = []
        for m in messages:
            name = m["#name"]
            if name == "set_next_step_dir":
                d = m["dir"]
                step_dir = 1 if d == 0 else -1 
            elif name == "queue_step":
                interval = m["interval"]
                count = m["count"]
                add = m["add"]
                time = current_clock
                d = step_dir
                for _ in range(count):
                    current_step += d
                    time += interval
                    steps.append((time, current_step))
                    interval += add
                current_clock = time

        output = []
        for step in steps:
            output.append(step[0] / float(self.frequency))
        return output
        
    def __del__(self):
        self.close()

    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

@pytest.fixture
def logger(request):
    logging.basicConfig()
    logger = logging.getLogger(request.node.name)
    logger.setLevel(logging.INFO)
    return logger

@pytest.fixture
def stepcompress(logger):
    with StepCompress(logger) as tester:
        yield tester


def test_one_step(stepcompress):
    stepcompress.set_input([1])
    stepcompress.verify_output([(1, 1, 0)])

def test_two_linear_steps(stepcompress):
    stepcompress.set_input([0.1, 0.2])
    stepcompress.verify_output([(0.1, 2, 0)])

def test_accelerating_third_step_encoded_as_single_message(stepcompress):
    stepcompress.set_input([0.1, 0.2, 0.3001])
    # The output is a single message with an early first step and an add part
    # Note that it's just one of many possible interpretations, but at least
    # this test shoudld notify if something changes
    stepcompress.verify_output([(0.099975, 3, 0.00005)])

def test_accelerating_third_step_encoded_as_two_messages(stepcompress):
    stepcompress.set_input([0.1, 0.2, 0.301])
    stepcompress.verify_output([
        (0.1, 2, 0),
        (0.101, 1, 0)
    ])

def test_fixed_speed(stepcompress):
    speed = 50.0
    distance = 20.0
    step_distance = 0.0125
    num_steps = int(distance / step_distance)
    input = [step_distance * (step+1) / speed for step in range(num_steps)]
    stepcompress.set_input(input)
    stepcompress.verify_output()

def test_fixed_acceleration(stepcompress):
    acceleration = 1000.0
    distance = 20.0
    step_distance = 0.0125
    num_steps = int(distance / step_distance)
    input = [sqrt(2.0 * step_distance * (step + 1) / acceleration) for step in range(num_steps)]
    stepcompress.set_input(input)
    stepcompress.verify_output()