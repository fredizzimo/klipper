# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
import sys
if not "../klippy" in sys.path:
    sys.path.insert(0, "../klippy")
import msgproto
import logging
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np


# %%
dict_path = "../ci_build/dict/atmega2560.dict"
serial_path = "../_test_output"
clock_freq = 16000000
step_dist = {
    2: 0.0025
}


# %%
msgs = []
with open(dict_path, "rb") as dfile:
    dictionary = dfile.read()
    mp = msgproto.MessageParser()
    mp.process_identify(dictionary, decompress=False)
    with open(serial_path, "rb") as ofile:
        data = ""
        error = False
        while not error:
            newdata = ofile.read(4096)
            if not newdata:
                break
            data += newdata
            while 1:
                l = mp.check_packet(data)
                if l == 0:
                    break
                if l < 0:
                    logging.error("Invalid data")
                    error = True
                    break
                msgs += mp.parse_packet(bytearray(data[:l]))
                data = data[l:]


# %%
decel_segments = dict()
for msg in msgs:
    if msg["name"] == "config_stepper":
        if msg["num_decel_segments"] > 0:
            decel_segments[msg["oid"]] = []
        print(msg)
    if msg["name"] == "set_decel_segment":
        interval = msg["interval"]
        add = msg["add"]
        count = msg["count"]
        decel_segments[msg["oid"]].append((interval, add, count))
        print(msg)


# %%
for oid, segments in decel_segments.iteritems():
    print("oid %i" % (oid,))
    for segment in segments:
        print("interval %d add %d count %d" % segment)

# %%
def reverse_segments(segments):
    return [(interval + add * (count - 1), -add, count) for interval, add, count in segments]

# %%
def generate_intervals(segments):
    intervals = []
    for interval, add, count in segments:
        for step in range(count):
            intervals.append(float(interval) / clock_freq)
            interval += add
    return intervals


# %%
def generate_t(intervals):
    ts = []
    t = 0
    for interval in intervals:
        t += interval
        ts.append(t)
    return ts


# %%
def generate_x(ts, oid):
    sd = step_dist[oid]
    return [i*sd for i in range(len(ts))]


# %%
def generate_v(intervals, oid):
    vs = []
    sd = step_dist[oid]
    return [sd / interval for interval in intervals]     


# %%
def plot_segments(oid, reverse):
    segments = decel_segments[oid]
    if reverse:
        segments = reverse_segments(segments)
    intervals = generate_intervals(segments)
    t = generate_t(intervals)
    x = generate_x(t, oid)
    v = generate_v(intervals, oid)
    fig = make_subplots(specs=[[{"secondary_y": True}]])
    fig.add_trace(go.Scatter(x=t, y=x, name="x"), secondary_y=False)
    fig.add_trace(go.Scatter(x=t, y=v, name="v"), secondary_y=True)
    fig.show()


# %%
plot_segments(2, True)

# %%
