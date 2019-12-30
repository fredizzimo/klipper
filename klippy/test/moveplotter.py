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
        jerks = []

        move_indices = [-1]

        x = 0
        a = 0
        t = 0
        j = 0
        jerk_multipliers = [
            1,
            0,
            -1,
            0,
            -1,
            0,
            1
        ]

        for move in moves:
            if type(move) is Move:
                profile = move.profile
            else:
                profile = move
            
            segments = []

            if profile.jerk == 0:
                if profile.accel_t:
                    segments.append((profile.accel_t, profile.accel, 0))
                if profile.cruise_t:
                    segments.append((profile.cruise_t, 0, 0))
                if profile.decel_t:
                    segments.append((profile.decel_t, -profile.decel, 0))
            else:
                acceleration = 0
                for index, t in enumerate(profile.jerk_t):
                    if t:
                        segments.append((t, acceleration,
                            profile.jerk * jerk_multipliers[index]))
                        acceleration = np.nan
                    

            v = profile.start_v

            num_ticks = 0
            for segment in segments:
                segment_t = segment[0]
                ts = np.linspace(0.0, segment_t,
                        max(int(ceil(segment_t / dt)), 2),
                        endpoint=True, dtype=np.float)
                if not np.isnan(segment[1]):
                    a = segment[1]
                j = segment[2]
                xs.append(x + v * ts + 0.5 * a * ts**2 + j * ts**3 / 6.0)
                vs.append(v + a * ts + 0.5 * j * ts**2)
                accs.append(a + j * ts)
                jerks.append(np.full(ts.shape[0], j))
                ts += t
                times.append(ts)
                x = xs[-1][-1]
                v = vs[-1][-1]
                t = ts[-1]
                a = accs[-1][-1]
                num_ticks += ts.shape[0]

            move_indices.append(move_indices[-1] + num_ticks)

        move_indices[0] = 0
        times = np.concatenate(times)
        x = np.concatenate(xs)
        v = np.concatenate(vs)
        a = np.concatenate(accs)
        j = np.concatenate(jerks)
        x_color = DEFAULT_PLOTLY_COLORS[0]
        v_color = DEFAULT_PLOTLY_COLORS[1]
        a_color = DEFAULT_PLOTLY_COLORS[2]
        j_color = DEFAULT_PLOTLY_COLORS[3]

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
        fig.add_trace(go.Scatter(x=times, y=j, name="jerk", yaxis="y4",
            legendgroup="jerk",
            line=go.scatter.Line(color=j_color)))
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
            x=times[move_indices], y=j[move_indices], mode="markers",
            yaxis="y4",
            showlegend=False,
            legendgroup="jerk",
            marker=go.scatter.Marker(color=a_color)))

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
                position=0.05
            ),
            yaxis4=go.layout.YAxis(
                title=go.layout.yaxis.Title(
                    text="mm/s^3"
                ),
                anchor="free",
                overlaying="y",
                side="left",
                position=0.0
            )
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
