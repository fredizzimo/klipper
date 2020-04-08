#!/usr/bin/env python2
# Graph the movement parsed from a serial port dump file
#
# Copyright (C) 2020  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import sys
import os
import errno
import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
from plotly.subplots import make_subplots

from os import path
sys.path.append(path.normpath(
    path.join(path.split(__file__)[0] + "/..")))
from klippy.msgproto import MessageParser

MASK_32_BIT = 0xFFFFFFFF
MASK_32_BIT_HIGH = MASK_32_BIT << 32

class Stepper(object):
    def __init__(self, message, clock_freq):
        self.oid = message["oid"]
        self.invert_step = message["invert_step"]
        self.step_clock = 0
        self.dir = not self.invert_step
        self.steps = [(0, 0)]
        self.pos = 0
        self.freq = clock_freq
    def reset_step_clock(self, message):
        self.step_clock = message["clock"]
    def set_next_step_dir(self, message):
        self.dir = message["dir"] ^ self.invert_step
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
        step = -1 if self.dir else 1
        for _ in range(count):
            t += interval
            self.steps.append((t / self.freq, pos))
            pos += step
            interval += add
        self.pos = pos
        self.step_clock = t & MASK_32_BIT
    def calculate_moves(self, start_time):
        self.steps = np.array(self.steps)
        self.steps[0][0] = start_time
        self.steps[:,0] -= start_time
        pass
    def get_message_clock(self, message):
        return int(message["timestamp"]*self.freq)


def graph_moves(steppers, output_path):
    fig = go.Figure()
    layout = {}
    spacing = 0.01
    domains = np.linspace(0, 1+spacing, len(steppers)+1)
    for i, stepper in enumerate(steppers):
        yaxis = "y%i" % (i+1)
        fig.add_trace(go.Scatter(
            x=stepper.steps[:,0], y=stepper.steps[:,1], name="%i" % stepper.oid,
            line=go.scatter.Line(),
            yaxis=yaxis
        ))
        layout["yaxis%i" % (i+1)] = go.layout.YAxis(
            anchor="x",
            domain=(domains[i], domains[i+1]-spacing),
            showline=True,
            fixedrange=False,
        )

    layout["xaxis"] = go.layout.XAxis(
        rangeslider=go.layout.xaxis.Rangeslider(
            visible=True,
            yaxis = go.layout.xaxis.rangeslider.YAxis(
                rangemode="fixed"
            )
    ))
    
    fig.update_layout(layout)
    
    filename = path.join(output_path, "steppers.html")
    pio.write_html(fig, filename, include_plotlyjs=True, full_html=True)

def create_output_directory(path):
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

def parse_serial(input, dictionary_file):
    dictionary = dictionary_file.read()
    dictionary_file.close()
    message_parser = MessageParser()
    message_parser.process_identify(dictionary, decompress=False)
    messages = message_parser.parse_file(input)
    clock_freq = message_parser.get_constant_float('CLOCK_FREQ')
    steppers = {}
    for m in messages:
        name = m["name"]
        if name == "queue_step":
            steppers[m["oid"]].queue_step(m)
        elif name == "config_stepper":
            stepper = Stepper(m, clock_freq)
            steppers[stepper.oid] = stepper
        elif name == "reset_step_clock":
            steppers[m["oid"]].reset_step_clock(m)
        elif name == "set_next_step_dir":
            steppers[m["oid"]].set_next_step_dir(m)
    
    start_time = messages[0]["timestamp"]
    
    for s in steppers.itervalues():
        s.calculate_moves(start_time)

    steppers = sorted(list(steppers.values()), key=lambda x: x.oid) 

    return steppers

def main():
    parser = argparse.ArgumentParser(description=
        "Utility to graph the movement parsed from a serial dump file")
    parser.add_argument("--dict", type=argparse.FileType(mode="rb"),
        help="Path to the dictionary file")
    parser.add_argument("--output", default="",
        help="Path to the output directory, the default is the current "
        "directory")
    parser.add_argument("input", type=argparse.FileType(mode="rb"),
        help="Path to the input serial port dump file")
    args = parser.parse_args()

    steppers = parse_serial(args.input, args.dict)

    create_output_directory(args.output)
    graph_moves(steppers, args.output)


if __name__ == "__main__":
    main()