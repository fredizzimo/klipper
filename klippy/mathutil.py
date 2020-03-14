# Simple math helper functions
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, multiprocessing


######################################################################
# Coordinate descent
######################################################################

# Helper code that implements coordinate descent
def coordinate_descent(adj_params, params, error_func):
    # Define potential changes
    params = dict(params)
    dp = {param_name: 1. for param_name in adj_params}
    # Calculate the error
    best_err = error_func(params)
    logging.info("Coordinate descent initial error: %s", best_err)

    threshold = 0.00001
    rounds = 0

    while sum(dp.values()) > threshold and rounds < 10000:
        rounds += 1
        for param_name in adj_params:
            orig = params[param_name]
            params[param_name] = orig + dp[param_name]
            err = error_func(params)
            if err < best_err:
                # There was some improvement
                best_err = err
                dp[param_name] *= 1.1
                continue
            params[param_name] = orig - dp[param_name]
            err = error_func(params)
            if err < best_err:
                # There was some improvement
                best_err = err
                dp[param_name] *= 1.1
                continue
            params[param_name] = orig
            dp[param_name] *= 0.9
    logging.info("Coordinate descent best_err: %s  rounds: %d",
                 best_err, rounds)
    return params

# Helper to run the coordinate descent function in a background
# process so that it does not block the main thread.
def background_coordinate_descent(printer, adj_params, params, error_func):
    parent_conn, child_conn = multiprocessing.Pipe()
    def wrapper():
        res = coordinate_descent(adj_params, params, error_func)
        child_conn.send(res)
        child_conn.close()
    # Start a process to perform the calculation
    calc_proc = multiprocessing.Process(target=wrapper)
    calc_proc.daemon = True
    calc_proc.start()
    # Wait for the process to finish
    reactor = printer.get_reactor()
    gcode = printer.lookup_object("gcode")
    eventtime = last_report_time = reactor.monotonic()
    while calc_proc.is_alive():
        if eventtime > last_report_time + 5.:
            last_report_time = eventtime
            gcode.respond_info("Working on calibration...", log=False)
        eventtime = reactor.pause(eventtime + .1)
    # Return results
    res = parent_conn.recv()
    calc_proc.join()
    parent_conn.close()
    return res


######################################################################
# Trilateration
######################################################################

# Trilateration finds the intersection of three spheres.  See the
# wikipedia article for the details of the algorithm.
def trilateration(sphere_coords, radius2):
    sphere_coord1, sphere_coord2, sphere_coord3 = sphere_coords
    s21 = matrix_sub(sphere_coord2, sphere_coord1)
    s31 = matrix_sub(sphere_coord3, sphere_coord1)

    d = math.sqrt(matrix_magsq(s21))
    ex = matrix_mul(s21, 1. / d)
    i = matrix_dot(ex, s31)
    vect_ey = matrix_sub(s31, matrix_mul(ex, i))
    ey = matrix_mul(vect_ey, 1. / math.sqrt(matrix_magsq(vect_ey)))
    ez = matrix_cross(ex, ey)
    j = matrix_dot(ey, s31)

    x = (radius2[0] - radius2[1] + d**2) / (2. * d)
    y = (radius2[0] - radius2[2] - x**2 + (x-i)**2 + j**2) / (2. * j)
    z = -math.sqrt(radius2[0] - x**2 - y**2)

    ex_x = matrix_mul(ex, x)
    ey_y = matrix_mul(ey, y)
    ez_z = matrix_mul(ez, z)
    return matrix_add(sphere_coord1, matrix_add(ex_x, matrix_add(ey_y, ez_z)))


######################################################################
# Matrix helper functions for 3x1 matrices
######################################################################

def matrix_cross(m1, m2):
    return [m1[1] * m2[2] - m1[2] * m2[1],
            m1[2] * m2[0] - m1[0] * m2[2],
            m1[0] * m2[1] - m1[1] * m2[0]]

def matrix_dot(m1, m2):
    return m1[0] * m2[0] + m1[1] * m2[1] + m1[2] * m2[2]

def matrix_magsq(m1):
    return m1[0]**2 + m1[1]**2 + m1[2]**2

def matrix_add(m1, m2):
    return [m1[0] + m2[0], m1[1] + m2[1], m1[2] + m2[2]]

def matrix_sub(m1, m2):
    return [m1[0] - m2[0], m1[1] - m2[1], m1[2] - m2[2]]

def matrix_mul(m1, s):
    return [m1[0]*s, m1[1]*s, m1[2]*s]


######################################################################
# Newton-Raphson root finding
# A fail-safe variant with an interval and bisection if failing
# The function f should return a tuple of the value and derivative
######################################################################

def newton_raphson(f, low, high, tolerance, maxiter):
    y_low, dy_low = f(low)
    y_high, dy_high = f(high)
    if y_low == 0:
        return low, y_low, dy_low
    if y_high == 0:
        return high, y_high, dy_high

    # If there's no zero in the range, return the point closest to it
    if y_low < 0 and y_high < 0:
        if y_low > y_high:
            return low, y_low, dy_low
        else:
            return high, y_high, dy_high
    if y_low > 0 and y_high > 0:
        if y_low > y_high:
            return high, y_high, dy_high
        else:
            return low, y_low, dy_low
    
    # Always keep f(x_low) < 0
    if y_low < 0:
        x_low = low
        x_high = high
    else:
        x_low = high
        x_high = low

    # Start the search in the middle
    x = 0.5 * (low + high)
    dx = high - low
    dx_old = dx

    y, dy = f(x)

    for _ in xrange(maxiter):
        # Use bisect if newton returns out of range or the change is too small
        if (((x - x_high)*dy - y) * ((x - x_low)*dy - y) > 0 or
                abs(2.0*y) > abs(dx_old*dy)):
            dx_old = dx
            dx = 0.5 * (x_high - x_low)
            x = x_low + dx
        # Newton-Raphson
        else:
            dx_old = dx
            dx = y / dy
            x -= dx
        
        # Return if within the tolerance
        if abs(dx) < tolerance:
            return x, y, dy
        
        y, dy = f(x)
        # Maintain the bracket
        if y < 0:
            x_low = x
        else:
            x_high = x
    
    # We are hopefully close enough
    return x, y, dy