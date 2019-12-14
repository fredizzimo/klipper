# Tests for the feedrate planner
#
# Copyright (C) 2019  Fred Sundvik <fsundvik@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import pytest
from toolhead import MoveQueue, Move
from kinematics.extruder import DummyExtruder
from math import sqrt, ceil
import plotly.graph_objects as go
import plotly.io as pio
from plotly.colors import DEFAULT_PLOTLY_COLORS
import os
import numpy as np

class Plotter(object):
    def __init__(self):
        self.html = ""

    def plot(self, testname, moves):
        first = self.html == ""
        fig = go.Figure()

        dt = 0.001
        times = []
        xs = []
        vs = []
        accs = []
        allowed_vs = []

        def add_move(t, x, v, a, allowed_v):
            current_time = 0
            current_x = 0
            if len(times) > 0:
                current_time = times[-1][-1]
                current_x = xs[-1][-1]
            t += current_time
            x += current_x
            current_time = t[-1]
            current_x = x[-1]
            times.append(t)
            xs.append(x)
            vs.append(v)
            accs.append(a)
            allowed_vs.append(allowed_v)

        move_indices = [-1]

        for move in moves:
            num_ticks = 0
            if move.accel_t > 0:
                t = np.linspace(0.0, move.accel_t, int(ceil(move.accel_t / dt)),
                    endpoint=True, dtype=np.float)
                x = move.start_v * t + 0.5 * move.accel * t**2
                v = move.start_v + move.accel * t
                a = np.full(t.shape[0], move.accel)
                allowed_v = np.full(t.shape[0], sqrt(move.max_cruise_v2))
                if num_ticks == 0:
                    allowed_v[0] = sqrt(move.max_start_v2)
                add_move(t, x, v, a, allowed_v)
                num_ticks += t.shape[0]
            if move.cruise_t > 0:
                t = np.linspace(0.0, move.cruise_t,
                    int(ceil(move.cruise_t / dt)),
                    endpoint=True, dtype=np.float)
                x = move.cruise_v * t
                v = np.full(t.shape[0], move.cruise_v)
                a = np.zeros(t.shape[0])
                allowed_v = np.full(t.shape[0], sqrt(move.max_cruise_v2))
                if num_ticks == 0:
                    allowed_v[0] = sqrt(move.max_start_v2)
                add_move(t, x, v, a, allowed_v)
                num_ticks += t.shape[0]
            if move.decel_t > 0:
                t = np.linspace(0.0, move.decel_t,
                    int(ceil(move.decel_t / dt)), endpoint=True,
                    dtype=np.float)
                x = move.cruise_v * t - 0.5 * move.accel * t**2
                v = move.cruise_v - move.accel * t
                a = np.full(t.shape[0], -move.accel)
                allowed_v = np.full(t.shape[0], sqrt(move.max_cruise_v2))
                if num_ticks == 0:
                    allowed_v[0] = sqrt(move.max_start_v2)
                add_move(t, x, v, a, allowed_v)
                num_ticks += t.shape[0]
            move_indices.append(move_indices[-1] + num_ticks)

        move_indices[0] = 0
        times = np.concatenate(times)
        x = np.concatenate(xs)
        v = np.concatenate(vs)
        a = np.concatenate(accs)
        allowed_v = np.concatenate(allowed_vs)
        x_color = DEFAULT_PLOTLY_COLORS[0]
        v_color = DEFAULT_PLOTLY_COLORS[1]
        a_color = DEFAULT_PLOTLY_COLORS[2]
        av_color = DEFAULT_PLOTLY_COLORS[3]

        fig.add_trace(go.Scatter(
            x=times, y=x, name="position",
            legendgroup="position",
            line=go.scatter.Line(color=x_color)))
        fig.add_trace(go.Scatter(
            x=times, y=v, name="velocity", yaxis="y2",
            legendgroup="velocity",
            line=go.scatter.Line(color=v_color)))
        fig.add_trace(go.Scatter(x=times, y=a, name="acceleration", yaxis="y3",
            legendgroup="acceleration",
            line=go.scatter.Line(color=a_color)))
        fig.add_trace(go.Scatter(
            x=times, y=allowed_v, name="allowed velocity", yaxis="y2",
            legendgroup="allowedvelocity",
            line=go.scatter.Line(color=av_color, dash="dot")))
        fig.add_trace(go.Scatter(
            x=times[move_indices], y=x[move_indices], mode="markers",
            showlegend=False,
            legendgroup="position",
            marker=go.scatter.Marker(color=x_color)))
        fig.add_trace(go.Scatter(
            x=times[move_indices], y=v[move_indices], mode="markers",
            yaxis="y2",
            showlegend=False,
            legendgroup="velocity",
            marker=go.scatter.Marker(color=v_color)))
        fig.add_trace(go.Scatter(
            x=times[move_indices], y=a[move_indices], mode="markers",
            yaxis="y3",
            showlegend=False,
            legendgroup="acceleration",
            marker=go.scatter.Marker(color=a_color)))
        fig.add_trace(go.Scatter(
            x=times[move_indices], y=allowed_v[move_indices], mode="markers",
            yaxis="y2",
            showlegend=False,
            legendgroup="allowedvelocity",
            marker=go.scatter.Marker(color=av_color)))

        fig.update_layout(
            title=go.layout.Title(
                text=testname,
                xref="paper",
                x=0
            ),
            xaxis=go.layout.XAxis(
                title=go.layout.xaxis.Title(
                    text="s"
                ),
                domain=[0.15,1]
            ),
            yaxis=go.layout.YAxis(
                title=go.layout.yaxis.Title(
                    text="mm"
                )
            ),
            yaxis2=go.layout.YAxis(
                title=go.layout.yaxis.Title(
                    text="mm/s"
                ),
                anchor="free",
                overlaying="y",
                side="left",
                position=0.1
            ),
            yaxis3=go.layout.YAxis(
                title=go.layout.yaxis.Title(
                    text="mm/s^2"
                ),
                anchor="free",
                overlaying="y",
                side="left",
                position=0.05            )
        )

        plot_html = pio.to_html(fig, include_plotlyjs=first, full_html=first)
        if first:
            self.html = plot_html
        else:
            self.html = self.html.replace("</body>", plot_html + "\n</body>")

    def write(self):
        output_dir = os.path.dirname(os.path.realpath(__file__))
        output_dir = os.path.join(output_dir, "output")
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        output_file = os.path.join(output_dir, "test_feedrateplanner.html")
        with open(output_file, "w") as o:
            o.write(self.html)

class ToolHead(object):
    def __init__(self):
        self.moves = []
        self.queue = MoveQueue(self)
        self.max_accel = None
        self.max_velocity = None
        self.max_accel_to_decel = None
        self.square_corner_velocity = None
        self.junction_deviation = None
        self.extruder = DummyExtruder()
        self.pos = np.zeros(4)

    def set_limits(self, max_vel, max_acc, max_acc_to_dec,
        square_corner_velocity):
        self.max_accel = max_acc
        self.max_velocity = max_vel
        self.max_accel_to_decel = max_acc_to_dec
        scv2 = square_corner_velocity**2
        self.junction_deviation = scv2 * (sqrt(2.) - 1.) / self.max_accel

    def _process_moves(self, moves):
        self.moves += moves

    def move(self, end_pos, max_speed):
        if type(end_pos) == tuple:
            end = np.array([p for p in end_pos] + [0] * (4-len(end_pos)))
        else:
            end = np.array((end_pos, 0, 0, 0))
        self.queue.add_move(Move(self, self.pos, end, max_speed))
        self.pos = end

    def flush(self):
        self.queue.flush()

    def check_move(self, idx, pos, start_v, cruise_v, accel_t, cruise_t,
        decel_t, distance):
        move = self.moves[idx]
        if type(pos) == tuple:
            pos = tuple([p for p in pos] + [0] * (4-len(pos)))
        else:
            pos = (pos, 0, 0, 0)
        assert move.start_pos == pos
        assert pytest.approx(move.start_v) == start_v
        assert pytest.approx(move.cruise_v) == cruise_v
        assert pytest.approx(move.accel_t) == accel_t
        assert pytest.approx(move.cruise_t) == cruise_t
        assert pytest.approx(move.decel_t) == decel_t
        assert pytest.approx(get_distance(move)) == distance

def get_distance(move):
    return (move.start_v * move.accel_t +
        0.5 * move.accel * move.accel_t**2 +
        move.cruise_v * (move.cruise_t + move.decel_t) -
        0.5 * move.accel * move.decel_t**2)

@pytest.fixture(scope="module")
def plotter():
    p = Plotter()
    yield p
    p.write()

@pytest.fixture
def toolhead(plotter, request):
    toolhead = ToolHead()
    yield toolhead
    plotter.plot(request.node.name, toolhead.moves)


# A move long enough to accelerate to cruise_v
def test_single_long_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(100, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.95,
        decel_t=0.05,
        distance=100)

# A move short enough to not accelerate to the cruise_v
def test_single_short_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(4, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=2000 * sqrt(2 / (0.5*2000)),
        accel_t=sqrt(2 / (0.5*2000)),
        cruise_t=0,
        decel_t=sqrt(2 / (0.5*2000)),
        distance=4)

# A move the exact lenght to accelerate to cruise_v, but no cruise_phase
def test_single_no_cruise_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(5, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.0,
        decel_t=0.05,
        distance=5)

def test_move_with_accel_decel_limit(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(5, max_speed=100)
    toolhead.flush()
    virt_accel_t = sqrt(2.5 / (0.5*1000))
    cruise_v = 1000 * virt_accel_t
    accel_t = cruise_v / 2000
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=cruise_v,
        accel_t=accel_t,
        cruise_t=0.0353553390593,
        decel_t=accel_t,
        distance=5)

def test_move_with_accel_decel_limit_longer_distance(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(6, max_speed=100)
    toolhead.flush()
    virt_accel_t = sqrt(3 / (0.5*1000))
    cruise_v = 1000 * virt_accel_t
    accel_t = cruise_v / 2000
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=cruise_v,
        accel_t=accel_t,
        cruise_t=0.0387298334621,
        decel_t=accel_t,
        distance=6)

def test_move_with_long_enough_distance_fo_no_accel_decel_limit(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 1
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.05,
        decel_t=0.05,
        distance=10)

def test_two_long_moves(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(100, max_speed=100)
    toolhead.move(200, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.975,
        decel_t=0,
        distance=100)
    toolhead.check_move(1,
        pos=100,
        start_v=100,
        cruise_v=100,
        accel_t=0,
        cruise_t=0.975,
        decel_t=0.05,
        distance=100)

def test_short_and_long_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(2, max_speed=100)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    accel1_t=sqrt(2 / (0.5*2000))
    cruise1_v=2000 * accel1_t
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=cruise1_v,
        accel_t=accel1_t,
        cruise_t=0,
        decel_t=0,
        distance=2)
    accel2_t = (100 - cruise1_v) / 2000
    toolhead.check_move(1,
        pos=2,
        start_v=cruise1_v,
        cruise_v=100,
        accel_t=accel2_t,
        cruise_t=0.15,
        decel_t=0.05,
        distance=18)

def test_long_and_short_move(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(20, max_speed=100)
    toolhead.move(22, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.17,
        decel_t=0.00527864045,
        distance=20)
    toolhead.check_move(1,
        pos=20,
        start_v=89.4427191,
        cruise_v=89.4427191,
        cruise_t=0,
        accel_t=0,
        decel_t=0.04472135955,
        distance=2)

def test_required_decelerate_due_to_upcoming_slow_segment(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=100)
    toolhead.move(12, max_speed=100)
    toolhead.move(20, max_speed=20)
    toolhead.flush()
    assert len(toolhead.moves) == 3
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.071,
        decel_t=0.00417424305044,
        distance=10)
    toolhead.check_move(1,
        pos=10,
        start_v=91.6515138991,
        cruise_v=91.6515138991,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0358257569496,
        distance=2)
    toolhead.check_move(2,
        pos=12,
        start_v=20,
        cruise_v=20,
        accel_t=0,
        cruise_t=0.395,
        decel_t=0.01,
        distance=8)

def test_accelerate_to_a_faster_segment(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move(10, max_speed=50)
    toolhead.move(20, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=50,
        accel_t=0.025,
        cruise_t=0.1875,
        decel_t=0,
        distance=10)
    toolhead.check_move(1,
        pos=10,
        start_v=50,
        cruise_v=100,
        accel_t=0.025,
        cruise_t=0.05625,
        decel_t=0.05,
        distance=10)

def test_square_corner(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move((10, 0), max_speed=100)
    toolhead.move((10, 10), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 2
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.0500625,
        decel_t=0.0475,
        distance=10)
    toolhead.check_move(1,
        pos=10,
        start_v=5,
        cruise_v=100,
        accel_t=0.0475,
        cruise_t=0.0500625,
        decel_t=0.05,
        distance=10)

def test_lookahead_slow_corner(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=2000,
        square_corner_velocity=5)
    toolhead.move((10, 0), max_speed=100)
    toolhead.move((11, 0.01), max_speed=100)
    toolhead.move((12, 0.02), max_speed=100)
    toolhead.move((10, 0.02), max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 4
    toolhead.check_move(0,
        pos=(0, 0),
        start_v=0,
        cruise_v=100,
        accel_t=0.05,
        cruise_t=0.0700011300623,
        decel_t=0.00527737701979,
        distance=10)
    toolhead.check_move(1,
        pos=(10, 0),
        start_v=89.4452459604,
        cruise_v=89.4452459604,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0130988501585,
        distance=sqrt(1.0**2 + 0.01**2))
    toolhead.check_move(2,
        pos=(11, 0.01),
        start_v=63.2475456434,
        cruise_v=63.2475456434,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0315097170036,
        distance=sqrt(1.0**2 + 0.01**2))
    toolhead.check_move(3,
        pos=(12, 0.02),
        start_v=0.228111636339,
        cruise_v=63.2457588891,
        accel_t=0.0315088236264,
        cruise_t=0,
        decel_t=0.0316228794446,
        distance=2)

def test_accel_decel_limit_over_multiple_moves_at_end(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(1, max_speed=100)
    toolhead.move(2, max_speed=100)
    toolhead.move(3, max_speed=100)
    toolhead.move(4, max_speed=100)
    toolhead.move(5, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 5
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=63.2455532034,
        accel_t=0.0316227766017,
        cruise_t=0,
        decel_t=0,
        distance=1)
    toolhead.check_move(1,
        pos=1,
        start_v=63.2455532034,
        cruise_v=70.7106781187,
        accel_t=0.00373256245764,
        cruise_t=0.0106066017178,
        decel_t=0,
        distance=1)
    toolhead.check_move(2,
        pos=2,
        start_v=70.7106781187,
        cruise_v=70.7106781187,
        accel_t=0,
        cruise_t=0.0141421356237,
        decel_t=0,
        distance=1)
    toolhead.check_move(3,
        pos=3,
        start_v=70.7106781187,
        cruise_v=70.7106781187,
        accel_t=0,
        cruise_t=0.0106066017178,
        decel_t=0.00373256245764,
        distance=1)
    toolhead.check_move(4,
        pos=4,
        start_v=63.2455532034,
        cruise_v=63.2455532034,
        accel_t=0,
        cruise_t=0,
        decel_t=0.0316227766017,
        distance=1)

def test_accel_decel_limit_over_multiple_moves_at_start(toolhead):
    toolhead.set_limits(
        max_vel=100,
        max_acc=2000,
        max_acc_to_dec=1000,
        square_corner_velocity=5)
    toolhead.move(1, max_speed=100)
    toolhead.move(2, max_speed=100)
    toolhead.move(3, max_speed=100)
    toolhead.move(4, max_speed=100)
    toolhead.move(5, max_speed=10)
    toolhead.move(15, max_speed=100)
    toolhead.flush()
    assert len(toolhead.moves) == 6
    toolhead.check_move(0,
        pos=0,
        start_v=0,
        cruise_v=63.2455532034,
        accel_t=0.0316227766017,
        cruise_t=0,
        decel_t=0,
        distance=1)
    toolhead.check_move(1,
        pos=1,
        start_v=63.2455532034,
        cruise_v=63.6396103068,
        accel_t=0.000197028551711,
        cruise_t=0.015517065476,
        decel_t=0,
        distance=1)
    toolhead.check_move(2,
        pos=2,
        start_v=63.6396103068,
        cruise_v=63.6396103068,
        accel_t=0,
        cruise_t=0.0157134840264,
        decel_t=0,
        distance=1)
    toolhead.check_move(3,
        pos=3,
        start_v=63.6396103068,
        cruise_v=63.6396103068,
        accel_t=0,
        cruise_t=0.00019641855033,
        decel_t=0.0268198051534,
        distance=1)
    toolhead.check_move(4,
        pos=4,
        start_v=10,
        cruise_v=10,
        accel_t=0,
        cruise_t=0.1,
        decel_t=0,
        distance=1)
    toolhead.check_move(5,
        pos=5,
        start_v=10,
        cruise_v=100,
        accel_t=90.0 / 2000,
        cruise_t=0.05025,
        decel_t=100.0 / 2000,
        distance=10)