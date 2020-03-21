import plotly.graph_objects as go
import plotly.io as pio
from plotly.colors import DEFAULT_PLOTLY_COLORS
from plotly.subplots import make_subplots
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

    def calculate_actual_extrusion(self, dt, pressure_factor, last_pos,
            extruder_pos):
        x = last_pos
        c = pressure_factor
        output_x = np.empty(extruder_pos.shape[0])
        output_v = np.empty(extruder_pos.shape[0])
        for i, p in enumerate(extruder_pos):
            # p = x + c*v
            # c*v = p - x
            # v = (p - x) / c
            v = (p - x) / c
            output_v[i] = v
            output_x[i] = x
            x += v * dt
        return (output_x, output_v)

            

    def plot(self, moves, simulated_extrusion=True, name=None, input_moves=None):
        pressure_factor = 0.01
        if not isinstance(moves, collections.Sequence):
            moves = (moves,)
        if name is None:
            name = self.test_name
        first = self.html == ""
        fig = go.Figure()

        dt = 0.0001
        times = []
        xs = []
        vs = []
        accs = []
        jerks = []
        extruder_xs = []
        extruder_vs = []
        extruder_accs = []
        extruder_jerks = []
        actual_extrusion_x = []
        actual_extrusion_v = []

        move_indices = [-1]

        x = 0
        a = 0
        t = 0
        j = 0
        last_extruder_pos = 0.0
        actual_extrusion_pos = 0.0

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
            if simulated_extrusion:
                extrusion_rate = 1.0
            else:
                extrusion_rate = move.axes_r[3]
            start_x = x


            if profile.jerk == 0:
                if profile.accel_t:
                    segments.append((profile.accel_t, profile.accel, 0))
                if profile.cruise_t:
                    segments.append((profile.cruise_t, 0, 0))
                if profile.decel_t:
                    segments.append((profile.decel_t, -profile.decel, 0))
            else:
                acceleration = profile.start_a
                for index, jt in enumerate(profile.jerk_t):
                    if jt:
                        segments.append((jt, acceleration,
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
                    a = np.float(segment[1])
                j = np.float(segment[2])

                # t stands for toolhead
                t_x = x + v * ts + 0.5 * a * ts**2 + j * ts**3 / 6.0
                t_v = v + a * ts + 0.5 * j * ts**2
                t_a = a + j * ts
                t_j = np.full(ts.shape[0], j)

                
                # e stands for extruder
                e_x = v * extrusion_rate * ts
                e_x += 0.5 * a * extrusion_rate * ts**2
                e_x += j * extrusion_rate * ts**3 / 6.0
                e_v = t_v * extrusion_rate
                e_a = t_a * extrusion_rate
                e_j = t_j * extrusion_rate

                if move.is_kinematic_move:
                    e_x += pressure_factor*e_v
                    e_v += pressure_factor*e_a
                    e_a += pressure_factor*e_j
                
                # Make sure that the extruder position is continous
                e_x += last_extruder_pos - e_x[0]
                last_extruder_pos = e_x[-1]
            
                extruder_xs.append(e_x)
                extruder_vs.append(e_v)
                extruder_accs.append(e_a)
                extruder_jerks.append(e_j)
                a_ex, a_ev = self.calculate_actual_extrusion(dt,
                    pressure_factor, actual_extrusion_pos, e_x)
                actual_extrusion_pos = a_ex[-1]
                actual_extrusion_x.append(a_ex)
                actual_extrusion_v.append(a_ev)

                ts += t
                times.append(ts)
                if move.is_kinematic_move:
                    xs.append(t_x)
                    vs.append(t_v)
                    accs.append(t_a)
                    jerks.append(t_j)
                else:
                    vals = np.full(ts.shape[0], 0.0)
                    xs.append(vals + start_x)
                    vs.append(vals)
                    accs.append(vals)
                    jerks.append(vals)


                x = t_x[-1]
                v = t_v[-1]
                t = ts[-1]
                a = t_a[-1]
                num_ticks += ts.shape[0]

            if not move.is_kinematic_move:
                x = start_x
            move_indices.append(move_indices[-1] + num_ticks)

        move_indices[0] = 0
        times = np.concatenate(times)
        x = np.concatenate(xs)
        v = np.concatenate(vs)
        a = np.concatenate(accs)
        j = np.concatenate(jerks)
        extruder_x = np.concatenate(extruder_xs)
        extruder_v = np.concatenate(extruder_vs)
        extruder_a = np.concatenate(extruder_accs)
        extruder_j = np.concatenate(extruder_jerks)
        actual_extrusion_x = np.concatenate(actual_extrusion_x)
        actual_extrusion_v = np.concatenate(actual_extrusion_v)
        if input_moves is not None:
            allowed_v = np.empty(x.shape[0])
            input_itr = iter(input_moves)
            input_start_dist = -2.0
            input_end_dist = 0
            input_move = None
            for i, dist in enumerate(x):
                if dist >= input_end_dist:
                    try:
                        input_move = next(input_itr)
                        if not input_move.is_kinematic_move:
                            allowed_v[i] = allowed_v[i-1]
                            continue
                        input_start_dist = input_end_dist
                        input_end_dist = input_start_dist + input_move.move_d
                        allowed_v[i] = sqrt(input_move.max_junction_v2)
                    except:
                        allowed_v[i] = allowed_v[i-1]
                else:
                    allowed_v[i] = sqrt(input_move.max_cruise_v2)

        x_color = DEFAULT_PLOTLY_COLORS[0]
        v_color = DEFAULT_PLOTLY_COLORS[1]
        a_color = DEFAULT_PLOTLY_COLORS[2]
        j_color = DEFAULT_PLOTLY_COLORS[3]

        fig.add_trace(go.Scatter(
            x=times, y=x, name="Position",
            legendgroup="position",
            line=go.scatter.Line(color=x_color)))
        fig.add_trace(go.Scatter(
            x=times, y=v, name="Velocity", yaxis="y2",
            legendgroup="velocity",
            line=go.scatter.Line(color=v_color)))
        if input_moves is not None:
            fig.add_trace(go.Scatter(
                x=times, y=allowed_v, name="Allowed Velocity", yaxis="y2",
                legendgroup="allowed_velocity",
                line=go.scatter.Line(color=v_color, dash="dash")))
        fig.add_trace(go.Scatter(x=times, y=a, name="Acceleration", yaxis="y3",
            legendgroup="acceleration",
            line=go.scatter.Line(color=a_color)))
        fig.add_trace(go.Scatter(x=times, y=j, name="Jerk", yaxis="y4",
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
            marker=go.scatter.Marker(color=j_color)))
        fig.add_trace(go.Scatter(
            x=times, y=extruder_x,
            yaxis="y1",
            name="Extruder Position",
            line=go.scatter.Line(color=x_color, dash="dot")))
        fig.add_trace(go.Scatter(
            x=times, y=extruder_v,
            yaxis="y2",
            name="Extruder Velocity",
            line=go.scatter.Line(color=v_color, dash="dot")))
        fig.add_trace(go.Scatter(
            x=times, y=extruder_a,
            yaxis="y3",
            name="Extruder Acceleration",
            line=go.scatter.Line(color=a_color, dash="dot")))
        fig.add_trace(go.Scatter(
            x=times, y=extruder_j,
            yaxis="y4",
            name="Extruder Jerk",
            line=go.scatter.Line(color=j_color, dash="dot")))
        fig.add_trace(go.Scatter(
            x=times, y=actual_extrusion_x,
            yaxis="y1",
            name="Actual extrusion pos",
            line=go.scatter.Line(color=x_color, dash="dashdot")))
        fig.add_trace(go.Scatter(
            x=times, y=actual_extrusion_v,
            yaxis="y2",
            name="Actual extrusion velocity",
            line=go.scatter.Line(color=v_color, dash="dashdot")))

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
            ),
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
