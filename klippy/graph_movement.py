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
import subprocess
from whichcraft import which
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from plotly.colors import DEFAULT_PLOTLY_COLORS
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output, State

from msgproto import MessageParser
from configfile import PrinterConfig
from klippy import Printer as KlippyPrinter

from klipper_dash_renderer import KlipperDashRenderer, KlipperViewer

MASK_32_BIT = 0xFFFFFFFF
MASK_32_BIT_HIGH = MASK_32_BIT << 32

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
    def reset_step_clock(self, message):
        self.step_clock = message["clock"]
        # Fixup the step position to the homing pos
        # This does not work properly for all kinematic types
        # it also assumes that there's a reset step clock after homing
        if self.is_homing:
            self.pos = int(self.rail.position_endstop / self.mcu._step_dist)
            self.is_homing = False

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
        self.steps[0][0] = start_time
        self.steps[:,0] -= start_time
        step_dist = self.mcu._step_dist
        if self.invert_dir:
            step_dist *= -1.0
        self.steps[:,1] *= step_dist
        self.calculate_velocities_and_accelerations()

    def calculate_velocities_and_accelerations(self):
        length = self.steps.shape[0]
        if length < 3:
            self.velocity = np.zeros(length)
            self.acceleration = np.zeros(length)
            return
        
        self.velocity = np.empty(length)
        self.acceleration = np.empty(length)
        # Assume that the first and last velocities and accelerations are zero
        self.velocity[0] = 0.0
        self.velocity[-1] = 0.0
        self.acceleration[0] = 0.0
        self.acceleration[-1] = 0.0

        # Calcuate the rest using 3 point central differences
        diffs = self.steps[1:,0] - self.steps[:-1,0]
        diffs_2 = diffs**2

        diffs_p0 = diffs[:-1]
        diffs_p2 = diffs[1:]

        diffs_2_p0 = diffs_2[:-1]
        diffs_2_p2 = diffs_2[1:]

        f_0 = self.steps[0:-2,1]
        f_1 = self.steps[1:-1,1]
        f_2 = self.steps[2:,1]

        b = diffs_p0*diffs_p2
        a = diffs_2_p0 + b
        c = diffs_2_p2 + b

        self.velocity[1:-1] = f_1 * (diffs_p2 - diffs_p0) / b
        self.velocity[1:-1] -= f_0 * diffs_p2 / a
        self.velocity[1:-1] += f_2 * diffs_p0 / c

        self.acceleration[1:-1] = (f_0 / a) * 2.0
        self.acceleration[1:-1] -= (f_1 / b) * 2.0
        self.acceleration[1:-1] += (f_2 / c) * 2.0

    def get_message_clock(self, message):
        return int(message["timestamp"]*self.freq)
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

def graph_steppers(steppers):
    fig = go.Figure()
    layout = {}
    spacing = 0.01
    domains = list(reversed(np.linspace(0, 1+spacing, len(steppers)+1)))
    y_axis_spacing = 0.03

    for i, stepper in enumerate(steppers):
        yaxis1 = "yaxis%i" % (3*i+1)
        yaxis2 = "yaxis%i" % (3*i+2)
        yaxis3 = "yaxis%i" % (3*i+3)
        y1 = "y%i" % (3*i+1)
        y2 = "y%i" % (3*i+2)
        y3 = "y%i" % (3*i+3)
        color = DEFAULT_PLOTLY_COLORS[i]
        fig.add_trace(go.Scatter(
            x=stepper.steps[:,0], y=stepper.steps[:,1],
            name="%s pos" % stepper.mcu._name,
            line=go.scatter.Line(color=color),
            yaxis=y1
        ))
        fig.add_trace(go.Scatter(
            x=stepper.steps[:,0], y=stepper.velocity,
            name="%s vel" % stepper.mcu._name,
            line=go.scatter.Line(dash="dash", color=color),
            yaxis=y2
        ))
        fig.add_trace(go.Scatter(
            x=stepper.steps[:,0], y=stepper.acceleration,
            name="%s acc" % stepper.mcu._name,
            line=go.scatter.Line(dash="dot", color=color),
            yaxis=y3
        ))
        layout[yaxis1] = go.layout.YAxis(
            anchor="x",
            domain=(domains[i+1], domains[i]-spacing),
            showline=True,
            fixedrange=True,
            position=y_axis_spacing*0.0
        )
        layout[yaxis2] = go.layout.YAxis(
            anchor="free",
            overlaying=y1,
            side="left",
            position=y_axis_spacing*1.0,
            fixedrange=True
        )
        layout[yaxis3] = go.layout.YAxis(
            anchor="free",
            overlaying=y1,
            side="left",
            position=y_axis_spacing*2.0,
            fixedrange=True
        )

    layout["xaxis"] = go.layout.XAxis(
        fixedrange=False,
        domain=[y_axis_spacing*3.0,1]
    )

    fig.update_layout(layout)
    return fig

def get_stepper_data(steppers):
    def find_stepper(name):
        for s in steppers:
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

def get_spatial_coordinates(data):
    a = np.empty((data.shape[0], 3))
    a[:,0] = data.x
    a[:,1] = data.y
    a[:,2] = data.z
    return a.flatten()

def get_printer_dimensions(printer):
    toolhead = printer.lookup_object("toolhead")
    rails = toolhead.kin.rails
    # Not all kinematics types are supported by this at the moment
    if len(rails) == 3:
        return [(rail.position_min, rail.position_max) for rail in rails]
    else:
        return [(0,100), (0,100), (0,100)]

def run_app(steppers, printer):
    stepper_data = get_stepper_data(steppers)
    app = dash.Dash(
        assets_folder="graph_movement_assets",
        include_assets_files=False,
        external_stylesheets= [
            "/assets/graph_movements.css"
        ]
    )
    app.layout = html.Div(
        children = [
            dcc.Graph(
                id="steppers",
                figure=graph_steppers(steppers),
            ),
            html.Div(
                id="renderer_container",
                children=[
                    dcc.Markdown(
"""
Click and use the mouse and arrow keys to zoom and move around.
Click outside to when done
""",
                        id="renderer_instructions"
                    ),
                    KlipperDashRenderer(
                        id="renderer",
                        vertices=get_spatial_coordinates(stepper_data),
                        printer_dimensions=get_printer_dimensions(printer),
                        times=stepper_data.time
                    )
                ]
            )
        ]
    )

    app.clientside_callback(
        """
        function(relayoutData, fig) {
            return window.klipper_dash_renderer.zoom_figure_y(fig);
        }
        """,
        Output("steppers", "figure"),
        [Input("steppers", "relayoutData")],
        [State("steppers", "figure")]
    )

    app.clientside_callback(
        """
        function(relayoutData, fig) {
            return [...fig.layout.xaxis.range];
        }
        """,
        Output("renderer", "selected_time"),
        [Input("steppers", "relayoutData")],
        [State("steppers", "figure")]
    )

    app.run_server(debug=True)


def parse(input, dictionary_file, printer):
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
            apply_config(steppers, printer)
    
    start_time = messages[0]["timestamp"]

    for s in steppers.itervalues():
        s.calculate_moves(start_time)

    steppers = sorted(list(steppers.values()), key=lambda x: x.oid) 

    return steppers

def read_printer_config(config_file):
    start_args = {
        "debuginput": True,
        "config_file": config_file.name
    }

    printer = KlippyPrinter(None, None, start_args)
    printer._read_config()
    return printer

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

npm_packages = ["three@0.115.0", "@babel/standalone@7.9.5"]
def check_npm_packages(asset_folder):
    npm = which("npm")
    if npm is None:
        print("npm does not seem to be installed on your system, please "
            "install it first")
        return False
    output = subprocess.check_output(["npm", "ls", "--prefix", asset_folder])
    all_found = True
    for package in npm_packages:
        if output.find(package) == -1:
            print("npm package %s not found. Run graph_movement with --install "
                "to install it" % package)
            all_found = False
    return all_found

def install_npm_packages(asset_folder):
    args = ["npm", "install"] + npm_packages + ["--prefix", asset_folder]
    output = subprocess.check_output(args)

def main():
    parser = argparse.ArgumentParser(description=
        "Utility to graph the movement parsed from a serial dump file")
    parser.add_argument("--dict", type=argparse.FileType(mode="rb"),
        required=True,
        help="Path to the dictionary file")
    parser.add_argument("--config", type=argparse.FileType(mode="r"),
        required=True,
        help="Path to the printer config file")
    parser.add_argument("--install", required=False, action="store_true",
        help="Install required javascript npm packages")
    parser.add_argument("input", type=argparse.FileType(mode="rb"),
        help="Path to the input serial port dump file")
    args = parser.parse_args()

    current_dir = os.path.split(__file__)[0]
    asset_folder = os.path.join(current_dir, "graph_movement_assets")

    if args.install:
        install_npm_packages(asset_folder)
        return
    elif not check_npm_packages(asset_folder):
        return


    printer = read_printer_config(args.config)
    steppers = parse(args.input, args.dict, printer)

    run_app(steppers, printer)


if __name__ == "__main__":
    main()