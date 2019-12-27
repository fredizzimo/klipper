import plotly.graph_objects as go
import plotly.io as pio
from plotly.colors import DEFAULT_PLOTLY_COLORS
import os
import numpy as np
from math import ceil, sqrt
import pytest
from feedrateplanner import Move
import collections


class MovePlotter(object):
    def __init__(self, name):
        self.html = ""
        self.name = name
        self.test_name = None

    def set_test_name(self, name):
        self.test_name = name

    def plot(self, moves, name=None):
        if not isinstance(moves, collections.Sequence):
            moves = (moves,)
        if name is None:
            name = self.test_name
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
            if allowed_v is not None:
                allowed_vs.append(allowed_v)

        move_indices = [-1]

        for move in moves:
            if type(move) is Move:
                profile = move.profile
            else:
                profile = move
                allowed_v = None
            num_ticks = 0
            if profile.accel_t > 0:
                t = np.linspace(0.0, profile.accel_t,
                        int(ceil(profile.accel_t / dt)),
                        endpoint=True, dtype=np.float)
                x = profile.start_v * t + 0.5 * profile.accel * t**2
                v = profile.start_v + profile.accel * t
                a = np.full(t.shape[0], profile.accel)
                if type(move) is Move:
                    allowed_v = np.full(t.shape[0], sqrt(move.max_cruise_v2))
                    if num_ticks == 0:
                        allowed_v[0] = sqrt(move.max_start_v2)
                add_move(t, x, v, a, allowed_v)
                num_ticks += t.shape[0]
            if profile.cruise_t > 0:
                t = np.linspace(0.0, profile.cruise_t,
                        int(ceil(profile.cruise_t / dt)),
                        endpoint=True, dtype=np.float)
                x = profile.cruise_v * t
                v = np.full(t.shape[0], profile.cruise_v)
                a = np.zeros(t.shape[0])
                if type(move) is Move:
                    allowed_v = np.full(t.shape[0], sqrt(move.max_cruise_v2))
                    if num_ticks == 0:
                        allowed_v[0] = sqrt(move.max_start_v2)
                add_move(t, x, v, a, allowed_v)
                num_ticks += t.shape[0]
            if profile.decel_t > 0:
                t = np.linspace(0.0, profile.decel_t,
                        int(ceil(profile.decel_t / dt)), endpoint=True,
                        dtype=np.float)
                x = profile.cruise_v * t - 0.5 * profile.accel * t**2
                v = profile.cruise_v - profile.accel * t
                a = np.full(t.shape[0], -profile.accel)
                if type(move) is Move:
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
        if allowed_vs:
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
        if allowed_vs:
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
        if allowed_vs:
            fig.add_trace(go.Scatter(
                x=times[move_indices], y=allowed_v[move_indices], mode="markers",
                yaxis="y2",
                showlegend=False,
                legendgroup="allowedvelocity",
                marker=go.scatter.Marker(color=av_color)))

        fig.update_layout(
            title=go.layout.Title(
                text=name,
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
        name = os.path.splitext(self.name)[0]
        name += ".html"
        output_file = os.path.join(output_dir, name)
        with open(output_file, "w") as o:
            o.write(self.html)

@pytest.fixture(scope="module")
def move_plotter_module(request):
    p = MovePlotter(request.node.name)
    yield p
    p.write()

@pytest.fixture
def move_plotter(move_plotter_module, request):
    move_plotter_module.set_test_name(request.node.name)
    return move_plotter_module
