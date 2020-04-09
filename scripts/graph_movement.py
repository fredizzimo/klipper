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


def graph_moves(steppers):
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
            fixedrange=True,
        )

    layout["xaxis"] = go.layout.XAxis(
        fixedrange=False,
    )

    fig.update_layout(layout)
    return fig
    

def run_app(steppers):
    app = dash.Dash()
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
    axis_names = ["yaxis%i" % (i+1) if i >0 else "yaxis"
        for i in range(len(steppers))]

    @app.callback(Output("example-graph", "figure"),
    [Input("example-graph", "relayoutData")],
    [State("example-graph", "figure")])
    def display_relayout_data(relayoutData, fig):
        if relayoutData is None:
            return fig
        
        fig_layout = fig["layout"]
        range = relayoutData.get("xaxis.range", None)
        default = fig_layout["xaxis"]["range"]
        if range is None:
            range = [
                relayoutData.get("xaxis.range[0]", None),
                relayoutData.get("xaxis.range[1]", None)
            ]
        if range[0] is None:
            if range[1] is None:
                return fig
            else:
                range[0] = default[0]
        elif range[1] is None:
            range[1] = default[1]
        
        for i,stepper in enumerate(steppers):
            stepper = steppers[i]
            step_times = stepper.steps[:,0]
            num_steps = stepper.steps.shape[0]
            if num_steps == 0:
                range_low = 100
                range_high = -100
            else:
                i_low = np.searchsorted(step_times, range[0], side="left")
                i_high = np.searchsorted(step_times, range[1], side="right")
                if i_low >= num_steps:
                    range_low = stepper.steps[-1,1]
                    range_high = range_low
                elif i_low == i_high:
                    range_low = stepper.steps[i_low,1]
                    range_high = range_low
                else:
                    range_low = np.min(stepper.steps[i_low:i_high,1])
                    range_high = np.max(stepper.steps[i_low:i_high,1])
                diff = range_high - range_low
                margin = diff*0.1
                range_high += margin
                range_low -= margin

            axis = fig_layout[axis_names[i]]
            axis["range"]=[range_low, range_high]
            axis["autorange"]=False
        return fig

    app.run_server(debug=True)


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
    parser.add_argument("input", type=argparse.FileType(mode="rb"),
        help="Path to the input serial port dump file")
    args = parser.parse_args()

    steppers = parse_serial(args.input, args.dict)

    run_app(steppers)


if __name__ == "__main__":
    main()