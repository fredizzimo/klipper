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
from plotly.subplots import make_subplots
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output, State

from msgproto import MessageParser
from configfile import PrinterConfig
from klippy import Printer as KlippyPrinter

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
        self.mcu = None
        self.extruder = None
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
    def set_mcu(self, mcu):
        self.mcu = mcu
    def set_extruder(self, extruder):
        self.extruder = extruder
        self.set_mcu(extruder.stepper)

# Dummy printer class, which is needed because PrinterConfig depends on it
class Printer(object):
    def __init__(self, configfile):
        self.configfile = configfile.name
        configfile.close()
        self.config = PrinterConfig(self)
        self.config.read_main_config()

    def lookup_object(self, obj):
        class GCode(object):
            def register_command(self, cmd, func, when_not_ready=False,
                    desc=None):
                pass
        return GCode()

    def get_start_args(self):
        return {
            "config_file": self.configfile
        }

def graph_moves(steppers):
    fig = go.Figure()
    layout = {}
    spacing = 0.01
    domains = list(reversed(np.linspace(0, 1+spacing, len(steppers)+1)))
    for i, stepper in enumerate(steppers):
        yaxis = "y%i" % (i+1)
        fig.add_trace(go.Scatter(
            x=stepper.steps[:,0], y=stepper.steps[:,1], name=stepper.mcu._name,
            line=go.scatter.Line(),
            yaxis=yaxis
        ))
        layout["yaxis%i" % (i+1)] = go.layout.YAxis(
            anchor="x",
            domain=(domains[i+1], domains[i]-spacing),
            showline=True,
            fixedrange=True,
        )

    layout["xaxis"] = go.layout.XAxis(
        fixedrange=False,
    )

    fig.update_layout(layout)
    return fig
    

def run_app(steppers):
    app = dash.Dash(assets_folder="graph_movement_assets")
    figure = graph_moves(steppers)
    app.layout = html.Div(children=[
        dcc.Graph(
            id='example-graph',
            figure=figure,
            style = {
                "height": "100%"
            }
        )],
        style = {
            "height": "100vh"
        }
    )

    app.clientside_callback(
        """
        function(relayoutData, fig) {
            return zoom_figure_y(fig);
        }
        """,
        Output("example-graph", "figure"),
        [Input("example-graph", "relayoutData")],
        [State("example-graph", "figure")]
    )

    app.run_server(debug=True)


def parse(input, dictionary_file, config_file):
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

    apply_config(steppers, config_file)

    for s in steppers.itervalues():
        s.calculate_moves(start_time)

    steppers = sorted(list(steppers.values()), key=lambda x: x.oid) 

    return steppers

def apply_config(steppers, config_file):
    start_args = {
        "debuginput": True,
        "config_file": config_file.name
    }

    printer = KlippyPrinter(None, None, start_args)
    printer._read_config()
    toolhead = printer.lookup_object("toolhead")
    for rail in toolhead.kin.rails:
        for stepper in rail.steppers:
            oid = stepper._oid
            steppers[oid].set_mcu(stepper)

    extruders = printer.lookup_objects("extruder")
    for _, extruder in extruders:
        if hasattr(extruder, "stepper"):
            oid = extruder.stepper._oid
            steppers[oid].set_extruder(extruder)

def main():
    parser = argparse.ArgumentParser(description=
        "Utility to graph the movement parsed from a serial dump file")
    parser.add_argument("--dict", type=argparse.FileType(mode="rb"),
        required=True,
        help="Path to the dictionary file")
    parser.add_argument("--config", type=argparse.FileType(mode="r"),
        required=True,
        help="Path to the printer config file")
    parser.add_argument("input", type=argparse.FileType(mode="rb"),
        help="Path to the input serial port dump file")
    args = parser.parse_args()

    steppers = parse(args.input, args.dict, args.config)

    run_app(steppers)


if __name__ == "__main__":
    main()