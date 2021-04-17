
# %%
from os import path
klippy_path = path.split(path.abspath(__name__))[0]
klippy_path = path.split(klippy_path)[0]
klippy_path = os.path.join(klippy_path, "klippy")
if klippy_path not in sys.path:
    sys.path.append(klippy_path)
from math import ceil, sqrt
from extras.bed_mesh import ZMesh
import plotly.graph_objects as go
from ConfigParser import RawConfigParser
from StringIO import StringIO
import numpy as np


# %%
def read_mesh_from_config(config_file, name):
    header = ["#*# <---------------------- SAVE_CONFIG ---------------------->",
        "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated."]

    def clean_line(line):
        line = line.rstrip()
        if line in header:
            return ""
        if line.startswith("#*#"):
            return line[4:]
        return line

    def read_mesh(config, name):
        section = "bed_mesh " + name
        ret = {
            "min_x": float(config.get(section, "min_x")),
            "max_x": float(config.get(section, "max_x")),
            "min_y": float(config.get(section, "min_y")),
            "max_y": float(config.get(section, "max_y")),
            "x_count": int(config.get(section, "x_count")),
            "y_count": int(config.get(section, "y_count"))
        }
        mesh = config.get(section, "points")
        rows = mesh.split("\n")
        rows = (row.split(",") for row in rows)
        rows = [[float(point.strip()) for point in row] for row in rows if len(row) > 1]
        return ret, rows

    with open(config_file) as config:
        lines = (clean_line(line) for line in config.readlines())
        lines = (line for line in lines if line)
        buffer = StringIO("\n".join(lines))
        config = RawConfigParser()
        config.readfp(buffer)
        return read_mesh(config, name)


# %%
def plot_mesh(config_file, name, params):
    surfaces = []
    offset = 0
    num_surfaces = len(params)
    num_columns = int(ceil(sqrt(num_surfaces)))
    num_rows  = num_surfaces / num_columns
    spacing=10.0
    c_min = 1000000 
    c_max = -1000000
    for i, (algo, x_pps, y_pps, tension, downsample) in enumerate(params):
        config, matrix = read_mesh_from_config("klippy.cfg", name)
        if downsample: 
            matrix = np.array(matrix)
            matrix = matrix[::x_pps+1,::y_pps+1]
            config["x_count"]  = matrix.shape[0]
            config["y_count"]  = matrix.shape[1]
        config["algo"] = algo
        config["tension"] = tension
        config["mesh_x_pps"] = x_pps
        config["mesh_y_pps"] = y_pps
        mesh = ZMesh(config)
        mesh.build_mesh(matrix)
        x = np.array([mesh.get_x_coordinate(j) for j in range(mesh.mesh_x_count)])
        y = np.array([mesh.get_y_coordinate(j) for j in range(mesh.mesh_y_count)])
        z = np.array(mesh.mesh_matrix)
        width = x[-1] - y[0] + spacing
        height = y[-1] - y[0] + spacing
        x += (i % num_columns) * width
        y += (i / num_columns) * height
        surfaces.append((x,y,z))
        c_min = min(c_min, np.min(z))
        c_max = max(c_max, np.max(z))
        
        
    surfaces = [go.Surface(x=x,y=y,z=z,showscale=(i==0),cmin=c_min, cmax=c_max) for i, (x,y,z) in enumerate(surfaces)]
    fig = go.Figure(data=surfaces)
    fig.update_layout(width=1000, height=800, margin=dict(l=0, t=0, r=0, b=0))
    fig.update_traces(contours_z=dict(show=True, usecolormap=False, color="lightgreen",
                                      highlightcolor="limegreen", project_z=False,
                                      start=-5, end=5, size=0.05))
    fig.show()

# %%
config_name = "klippy.cfg"
mesh_name = "after_screw_adjust_80"
x_pps = 2
y_pps = 2
params = [
    ("lagrange", 0, 0, None, False),
    ("lagrange", x_pps, y_pps, None, True),
    ("bicubic", x_pps, y_pps, 0.0, True),
    ("bicubic", x_pps, y_pps, 0.15, True),
    ("bicubic", x_pps, y_pps, 0.3, True),
    ("bicubic", x_pps, y_pps, 0.45, True),
    ("bicubic", x_pps, y_pps, 0.60, True),
    ("bicubic", x_pps, y_pps, 0.75, True),
    ("bicubic", x_pps, y_pps, 1.0, True),
]
plot_mesh(config_name, mesh_name, params)