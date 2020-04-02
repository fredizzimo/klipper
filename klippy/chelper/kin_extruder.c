// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance
#include "move.h"

void __visible
trapq_append_extrude_move(struct trapq *tq, double print_time, struct move *m)
{
    double axis_r = m->axes_r[3];
    double accel = m->accel * axis_r;
    double start_v = m->start_v * axis_r;
    double cruise_v = m->cruise_v * axis_r;
    double pressure_advance = 0.0;
    if (axis_r > 0.0 && (m->axes_d[0] || m->axes_d[1]))
    {
        pressure_advance = m->pressure_advance;
    }
    // Queue movement (x is extruder movement, y is pressure advance)
    trapq_append(tq, print_time, m->accel_t, m->cruise_t, m->decel_t,
        m->start_pos[3], 0.0, 0.0, 1.0, pressure_advance, 0.,
        start_v, cruise_v, accel);
}


// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// Which is then "smoothed" using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(pa_position(x) * (smooth_time/2 - abs(t-x)) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//         / ((smooth_time/2)**2))

// Calculate the definitive integral of the motion formula:
//   position(t) = base + t * (start_v + t * half_accel)
static double
extruder_integrate(double base, double start_v, double half_accel
                   , double start, double end)
{
    double half_v = .5 * start_v, sixth_a = (1. / 3.) * half_accel;
    double si = start * (base + start * (half_v + start * sixth_a));
    double ei = end * (base + end * (half_v + end * sixth_a));
    return ei - si;
}

// Calculate the definitive integral of time weighted position:
//   weighted_position(t) = t * (base + t * (start_v + t * half_accel))
static double
extruder_integrate_time(double base, double start_v, double half_accel
                        , double start, double end)
{
    double half_b = .5 * base, third_v = (1. / 3.) * start_v;
    double eighth_a = .25 * half_accel;
    double si = start * start * (half_b + start * (third_v + start * eighth_a));
    double ei = end * end * (half_b + end * (third_v + end * eighth_a));
    return ei - si;
}

// Calculate the definitive integral of extruder for a given move
static double
pa_move_integrate(struct trapq_move *m, double start, double end,
                  double time_offset)
{
    if (start < 0.)
        start = 0.;
    if (end > m->move_t)
        end = m->move_t;
    // Calculate base position and velocity with pressure advance
    double pressure_advance = m->axes_r.y;
    double base = m->start_pos.x + pressure_advance * m->start_v;
    double start_v = m->start_v + pressure_advance * 2. * m->half_accel;
    // Calculate definitive integral
    double ha = m->half_accel;
    double iext = extruder_integrate(base, start_v, ha, start, end);
    double wgt_ext = extruder_integrate_time(base, start_v, ha, start, end);
    return wgt_ext - time_offset * iext;
}

// Calculate the definitive integral of the extruder over a range of moves
static double
pa_range_integrate(struct trapq_move *m, double move_time, double hst)
{
    // Calculate integral for the current move
    double res = 0., start = move_time - hst, end = move_time + hst;
    res += pa_move_integrate(m, start, move_time, start);
    res -= pa_move_integrate(m, move_time, end, end);
    // Integrate over previous moves
    struct trapq_move *prev = m;
    while (unlikely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        res += pa_move_integrate(prev, start, prev->move_t, start);
    }
    // Integrate over future moves
    while (unlikely(end > m->move_t)) {
        end -= m->move_t;
        m = list_next_entry(m, node);
        res -= pa_move_integrate(m, 0., end, end);
    }
    return res;
}

struct extruder_stepper {
    struct stepper_kinematics sk;
    double half_smooth_time, inv_half_smooth_time2;
};

static double
extruder_calc_position(struct stepper_kinematics *sk, struct trapq_move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = es->half_smooth_time;
    if (!hst)
        // Pressure advance not enabled
        return m->start_pos.x + move_get_distance(m, move_time);
    // Apply pressure advance and average over smooth_time
    double area = pa_range_integrate(m, move_time, hst);
    return area * es->inv_half_smooth_time2;
}

void __visible
extruder_set_smooth_time(struct stepper_kinematics *sk, double smooth_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = smooth_time * .5;
    es->half_smooth_time = hst;
    es->sk.gen_steps_pre_active = es->sk.gen_steps_post_active = hst;
    if (! hst)
        return;
    es->inv_half_smooth_time2 = 1. / (hst * hst);
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->sk.calc_position_cb = extruder_calc_position;
    es->sk.active_flags = AF_X;
    return &es->sk;
}
