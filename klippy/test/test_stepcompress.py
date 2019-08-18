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

import logging
from time import sleep
from serial import Serial
import os

@pytest.fixture
def ffi():
    ffi_main, ffi_lib = chelper.get_ffi()
    return namedtuple("FFI", "main lib")(ffi_main, ffi_lib)

@pytest.fixture
def parser():
    data = r"""{
        "commands": {
            "queue_step oid=%c interval=%u count=%hu add=%hi": 1,
            "set_next_step_dir oid=%c dir=%c": 2
        },
        "responses": {}
    }"""
    parser = MessageParser()
    parser.process_identify(data, decompress=False)
    return parser

@pytest.fixture
def virtual_pty():
    master, slave = openpty()
    name = os.ttyname(slave)
    # This will set the right flags for binary writing
    serial = Serial(name)
    yield (master, serial.fileno())
        

@pytest.fixture
def pty_file(virtual_pty):
    return fdopen(virtual_pty[0], "rb")

@pytest.fixture
def serialqueue(ffi, virtual_pty, pty_file):
    return ffi.lib.serialqueue_alloc(virtual_pty[1], True)

@pytest.fixture
def step_compress(ffi, parser):
    step_compress = ffi.main.gc(ffi.lib.stepcompress_alloc(0), ffi.lib.stepcompress_free)
    step_cmd_id = parser.lookup_command(
        "queue_step oid=%c interval=%u count=%hu add=%hi").msgid
    dir_cmd_id = parser.lookup_command(
        "set_next_step_dir oid=%c dir=%c").msgid
    max_error = 0
    invert_dir = False
    ffi.lib.stepcompress_fill(
        step_compress, max_error,
        invert_dir, step_cmd_id, dir_cmd_id)
    return step_compress

@pytest.fixture
def stepqueues(step_compress):
    # Use a generator to make sure that the list remains until the test is finished
    queues = [step_compress]
    yield queues


@pytest.fixture
def steppersync(ffi, serialqueue, stepqueues):
    move_count = 16
    steppersync = ffi.lib.steppersync_alloc(
        serialqueue, stepqueues, len(stepqueues),
        move_count)
    return steppersync



def test_one_step(ffi, step_compress, parser, steppersync, pty_file):
    qa = ffi.lib.queue_append_start(step_compress, 0, 0.5)
    ffi.lib.queue_append(ffi.main.addressof(qa), 1)
    ffi.lib.queue_append_finish(qa)
    ffi.lib.steppersync_flush(steppersync, 2)

    poll = select.poll()
    poll.register(pty_file.fileno(), select.POLLIN)
    data = ""
    messages = []
    while len(poll.poll(500)) == 1:
        data += os.read(pty_file.fileno(), 4096)
        while data != "":
            size = parser.check_packet(data)
            if size > 0:
                res = parser.parse_packet(bytearray(data[:size]))
                messages += res
                data = data[size:]
            else:
                break
    print(messages)
    assert len(messages) == 1
    assert messages[0]["#name"] == "queue_step"
    assert messages[0]["interval"] == 1
    assert messages[0]["count"] == 1
    assert messages[0]["add"] == 0