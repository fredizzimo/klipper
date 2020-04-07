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

from os import path
sys.path.append(path.normpath(
    path.join(path.split(__file__)[0] + "/..")))
from klippy.msgproto import MessageParser

class Stepper(object):
    def __init__(self, message):
        self.oid = message["oid"]
        self.invert_step = message["invert_step"]
        self.step_clock = 0
        self.dir = not self.invert_step
        self.steps = [(0, 0)]
        self.pos = 0
    def reset_step_clock(self, message):
        self.step_clock = message["clock"]
    def set_next_step_dir(self, message):
        self.dir = message["dir"] ^ self.invert_step
    def queue_step(self, message):
        interval = message["interval"]
        count = message["count"]
        add = message["add"]
        t = self.step_clock
        pos = self.pos
        step = -1 if self.dir else 1
        for _ in range(count):
            t += interval
            pos += step
            self.steps.append((t, pos))
            interval += add
        self.pos = pos
        self.step_clock = t
    def calculate_moves(self):
        self.steps = np.array(self.steps)

def graph_moves(steppers, output_path):
    steppers = sorted(list(steppers.values()), key=lambda x: x.oid) 
    fig = go.Figure()
    for stepper in steppers:
        fig.add_trace(go.Scatter(
            x=stepper.steps[:,0], y=stepper.steps[:,1], name="%i" % stepper.oid,
            line=go.scatter.Line()))
    
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

    dictionary = args.dict.read()
    args.dict.close()
    message_parser = MessageParser()
    message_parser.process_identify(dictionary, decompress=False)
    messages = message_parser.parse_file(args.input)
    steppers = {}
    for m in messages:
        name = m["name"]
        if name == "queue_step":
            steppers[m["oid"]].queue_step(m)
        elif name == "config_stepper":
            stepper = Stepper(m)
            steppers[stepper.oid] = stepper
        elif name == "finalize_config":
            print("Num steppers %i" % len(steppers))
        elif name == "reset_step_clock":
            steppers[m["oid"]].reset_step_clock(m)
        elif name == "set_next_step_dir":
            steppers[m["oid"]].set_next_step_dir(m)
    
    for s in steppers.itervalues():
        s.calculate_moves()

    create_output_directory(args.output)
    graph_moves(steppers, args.output)


if __name__ == "__main__":
    main()