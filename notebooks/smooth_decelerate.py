# %%
import sympy as sy
from math import sqrt, floor
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# %%
delta_x = sy.Symbol("Delta_x")
v = sy.Symbol("v")
a = sy.Symbol("a")
eq_delta_x = sy.Eq(delta_x, v**2 / (2*a))
display(eq_delta_x)


# %%
delta_step0 = sy.Symbol("Delta_step0")
t_0 = sy.Symbol("t_0")
eq_delta_step0 = sy.Eq(delta_step0, v*t_0 - a/2*t_0**2)
display(eq_delta_step0)
eq_t_step0 = sy.Eq(t_0, sy.solve(eq_delta_step0, t_0)[0])
display(eq_t_step0)

# %%
C = sy.Function("C")
n = sy.Symbol("n")
a = sy.Symbol("a")
m = sy.Symbol("m")
eq_c_n = sy.Eq(C(n), C(n-1) - 2*C(n-1) / (4*(n-delta_x) + 1))
eq_c_0 = sy.Eq(C(0), t_0)
display(eq_c_n)
display(eq_c_0)

# %%
def generate_intervals(v, a, step_dist, freq):
    #emulate what we have on the firmware side
    interval = 1.0 / v
    interval *= step_dist
    interval *= freq
    interval = int(round(interval))

    # a is passed as a/step_dist
    a = int(round(float(a)/step_dist))
    
    # these can be precalculated when configuring
    a_inv = 1.0 / float(a)
    two_a = 2*a
    two_a_inv = 1.0 / two_a
    
    # actual code that needs to be implemented starts here
    
    v = float(freq) / interval
    v2 = v*v

    radicand = v2 - two_a

    if radicand > 0:
        t_step_0 = (v - sqrt(radicand)) * a_inv

        # Use integer math, so that everything is deterministic
        num_steps = int(round(v2 * two_a_inv))
        
        c = int(round(t_step_0 * freq))
        intervals = [c]

        divisor = 4 - 4 * num_steps + 1
        for n in range(1, num_steps-1):
            c = c - (2 * c) / divisor
            divisor += 4
            intervals.append(c)
        return intervals
    else:
        return []

def convert_intervals(intervals, freq):
    return [float(t) / freq for t in intervals]
    
def generate_t(intervals):
    ts = []
    t = 0
    for interval in intervals:
        t += interval
        ts.append(t)
    return ts

def generate_x(ts, step_dist):
    return [(i+1)*step_dist for i in range(len(ts))]

def generate_v(intervals, ts, step_dist):
    shifted_left = [0] + ts
    ts = [prev + (next-prev) / 2.0 for prev, next in zip(shifted_left[:-1], ts)]
    vs = [step_dist / interval for interval in intervals]
    return vs, ts


# %%
def plot_segments(v, a, step_dist, freq):
    intervals = generate_intervals(v, a, step_dist, freq)
    intervals = convert_intervals(intervals, freq)
    ts = generate_t(intervals)
    xs = generate_x(ts, step_dist)
    vs, vts = generate_v(intervals, ts, step_dist)
    v2s = [v - a*t for t in ts] 
    fig = make_subplots(specs=[[{"secondary_y": True}]])
    fig.add_trace(go.Scatter(x=ts, y=xs, name="x"), secondary_y=False)
    fig.add_trace(go.Scatter(x=vts, y=vs, name="v"), secondary_y=True)
    fig.add_trace(go.Scatter(x=ts, y=v2s, name="vref"), secondary_y=True)
    fig.show()

plot_segments(5, 300, 0.0025, 72000000)
# %%
