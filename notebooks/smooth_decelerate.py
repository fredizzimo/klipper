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
def generate_intervals(v, a, step_dist, freq, extra_steps):
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
        num_steps = int(floor(v2 * two_a_inv))
        
        c = int(round(t_step_0 * freq))

        divisor = -4 * num_steps + 5
        increment = 4
         
        # We need an even number of steps
        if (num_steps + extra_steps) & 1:
            extra_steps += 1
        
        intervals = [(interval, 1)] * extra_steps

        num_deceleration_steps = num_steps
        num_acceleration_steps = (num_steps + extra_steps) / 2
        
        num_steps = num_deceleration_steps + 2 * num_acceleration_steps
        deceleration_end = num_steps - num_deceleration_steps
        acceleration_end = deceleration_end - num_acceleration_steps

        direction = 1
        while num_steps > 0:
            intervals.append((c, direction))
            num_steps -= 1
            if num_steps == deceleration_end:
                # Switch both step direction and acceleration direction
                direction = -1
                divisor = -divisor + 6
            elif num_steps == acceleration_end:
                # Switch acceleration direction
                divisor = -divisor + 6
            c = c - (2 * c) / divisor
            divisor += increment

        return intervals
    else:
        return []

def convert_intervals(intervals, freq):
    return [(float(t) / freq, d) for t, d in intervals]
    
def generate_t(intervals):
    ts = []
    t = 0
    for interval, _ in intervals:
        t += interval
        ts.append(t)
    return ts

def generate_x(intervals, step_dist):
    dist = 0.0
    ret = []
    for _, d in intervals:
        dist += step_dist * d
        ret.append(dist)
    return ret

def generate_v(intervals, ts, step_dist):
    shifted_left = [0] + ts
    ts = [prev + (next-prev) / 2.0 for prev, next in zip(shifted_left[:-1], ts)]
    vs = [d*step_dist / interval for (interval, d) in intervals]
    return vs, ts


# %%
def plot_segments(v, a, step_dist, freq, extra_steps):
    intervals = generate_intervals(v, a, step_dist, freq, extra_steps)
    intervals = convert_intervals(intervals, freq)
    ts = generate_t(intervals)
    xs = generate_x(intervals, step_dist)
    vs, vts = generate_v(intervals, ts, step_dist)
    v2s = [v - a*t for t in ts] 
    fig = make_subplots(specs=[[{"secondary_y": True}]])
    fig.add_trace(go.Scatter(x=ts, y=xs, name="x"), secondary_y=False)
    fig.add_trace(go.Scatter(x=vts, y=vs, name="v"), secondary_y=True)
    fig.add_trace(go.Scatter(x=ts, y=v2s, name="vref"), secondary_y=True)
    fig.show()

plot_segments(5, 300, 0.0025, 72000000, extra_steps=0)
# %%
