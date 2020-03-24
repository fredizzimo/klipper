// Move queue
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "movequeue.h"
#include "compiler.h"
#include <stdlib.h>
#include <float.h>
#include <math.h>

static const double tolerance = 1e-13;
static const double time_tolerance = 1e-6;

struct move_queue* __visible
move_queue_alloc(unsigned int num_moves)
{
    struct move_queue *m = malloc(sizeof(*m));
    m->moves = malloc(sizeof(struct move)*num_moves);
    m->num_moves = num_moves;
    m->next_free = 0;
    return m;
}

void __visible
move_queue_free(struct move_queue *queue)
{
    free(queue->moves);
    free(queue);
}

void __visible
limit_speed(struct move *m, double speed, double accel,
    double max_accel_to_decel)
{
    double speed2 = speed * speed;
    if (speed2 < m->max_cruise_v2)
    {
        m->max_cruise_v2 = speed2;
        m->min_move_t = m->move_d / speed;
    }
    m->accel = fmin(m->accel, accel);
    m->delta_v2 = 2.0 * m->move_d * m->accel;
    if (max_accel_to_decel > 0)
    {
        double smooth_delta_v2 = 2.0 * m->move_d * max_accel_to_decel;
        m->smooth_delta_v2 = fmin(m->smooth_delta_v2, smooth_delta_v2);
    }

    m->smooth_delta_v2 = fmin(m->smooth_delta_v2, m->delta_v2);
}

static void init_move(
    struct move *m,
    double *start_pos,
    double *end_pos,
    double speed,
    double accel,
    double accel_to_decel,
    double jerk)
{
    for(int i=0;i<4;i++)
    {
        m->start_pos[i] = start_pos[i];
        m->end_pos[i] = end_pos[i];
        m->axes_d[i] = end_pos[i] - start_pos[i];
    }
    double sum = 0;
    for (int i=0;i<3;i++)
    {
        double d = m->axes_d[i];
        sum += d*d;
    }
    double move_d = sqrt(sum);
    m->move_d = move_d;
    m->is_kinematic_move = true;
    double inv_move_d = 0.0;
    if (move_d < .000000001)
    {
        // Extrude only move
        for (int i=0;i<3;i++)
        {
            m->end_pos[i] = m->start_pos[i];
            m->axes_d[i] = 0.0;
        }
        move_d = fabs(m->axes_d[3]);
        m->move_d = move_d;

        // The extruder will limit the acceleration later
        accel = 99999999.9;
        m->is_kinematic_move = false;
    }
    if (move_d > 0)
    {
        inv_move_d = 1.0 / move_d;
    }
    for(int i=0;i<4;i++)
    {
        m->axes_r[i] = m->axes_d[i] * inv_move_d;
    }
    m->start_a = 0.0;
    m->accel_t = 0.0;
    m->cruise_t = 0.0;
    m->decel_t = 0.0;
    for (int i=0;i<7;i++)
    {
        m->jerk_t[i] = 0.0;
    }
    // Junction speeds are tracked in velocity squared.  The
    // delta_v2 is the maximum amount of this squared-velocity that
    // can change in this move.
    m->max_junction_v2 = 0.0;
    m->max_start_v2 = 0.0;
    m->max_smoothed_v2 = 0.0;

    m->accel = DBL_MAX;
    m->jerk = jerk;
    m->max_cruise_v2 = DBL_MAX;
    m->smooth_delta_v2 = DBL_MAX;
    m->min_move_t = 0.0;

    // NOTE: max accel_to_decel is used for extrude only moves as well
    limit_speed(m, speed, accel, accel_to_decel);
}

static double calc_extruder_junction(struct move *m, struct move *prev_move,
    double instant_corner_v)
{
    double diff_r = m->axes_r[3] - prev_move->axes_r[3];
    if (diff_r != 0)
    {
        double v = instant_corner_v / fabs(diff_r);
        return v*v;
    }
    return m->max_cruise_v2;
}

void __visible
calc_junction(struct move *m, struct move *prev_move,
    double junction_deviation, double extruder_instant_v)
{
    if (!m->is_kinematic_move || !prev_move->is_kinematic_move)
        return;
    // Allow extruder to calculate its maximum junction
    double extruder_v2 =
        calc_extruder_junction(m, prev_move, extruder_instant_v);
    // Find max velocity using "approximated centripetal velocity"
    double *axes_r = m->axes_r;
    double *prev_axes_r = prev_move->axes_r;
    double junction_cos_theta = -(axes_r[0] * prev_axes_r[0]
                                + axes_r[1] * prev_axes_r[1]
                                + axes_r[2] * prev_axes_r[2]);
    if (junction_cos_theta > 0.999999)
        return;
    junction_cos_theta = fmax(junction_cos_theta, -0.999999);
    double sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta));
    double R = (junction_deviation * sin_theta_d2
               / (1. - sin_theta_d2));
    double tan_theta_d2 = sin_theta_d2
                         / sqrt(0.5*(1.0+junction_cos_theta));
    double move_centripetal_v2 = .5 * m->move_d * tan_theta_d2 * m->accel;
    double prev_move_centripetal_v2 = (.5 * prev_move->move_d * tan_theta_d2
                                      * prev_move->accel);
    m->max_junction_v2 = fmin(fmin(fmin(
        fmin(R * m->accel, R * prev_move->accel),
        fmin(move_centripetal_v2, prev_move_centripetal_v2)),
        fmin(extruder_v2, m->max_cruise_v2)),
        prev_move->max_cruise_v2);
    m->max_start_v2 = fmin(
        m->max_junction_v2,
        prev_move->max_start_v2 + prev_move->delta_v2);
    m->max_smoothed_v2 = fmin(
        m->max_start_v2
        , prev_move->max_smoothed_v2 + prev_move->smooth_delta_v2);
}

void __visible
set_trapezoidal_times(struct move *m, double distance, double start_v2,
    double cruise_v2, double end_v2, double accel)
{
    start_v2 = fmin(start_v2, cruise_v2);
    end_v2 = fmin(end_v2, cruise_v2);
    m->accel = accel;
    m->jerk = 0.0;
    // Determine accel, cruise, and decel portions of the move distance
    double half_inv_accel = .5 / accel;
    double accel_d = (cruise_v2 - start_v2) * half_inv_accel;
    double decel_d = (cruise_v2 - end_v2) * half_inv_accel;
    double cruise_d = distance - accel_d - decel_d;
    // Make sure that all distances and therefore the times are positive
    // Clamp to zero if close to it, so that the whole segment is removed
    if (accel_d < tolerance)
        accel_d = 0;
    if (decel_d < tolerance)
        decel_d = 0;
    if (cruise_d < tolerance)
        cruise_d = 0;

    // Determine move velocities
    double start_v = sqrt(start_v2);
    m->start_v = start_v; 
    double cruise_v = sqrt(cruise_v2);
    m->cruise_v = cruise_v; 
    double end_v = sqrt(end_v2);
    m->end_v = end_v;
    // Determine time spent in each portion of move (time is the
    // distance divided by average velocity)
    m->accel_t = accel_d / ((start_v + cruise_v) * 0.5);
    m->cruise_t = cruise_d / cruise_v;
    m->decel_t = decel_d / ((end_v + cruise_v) * 0.5);
}

void __visible
calculate_trapezoidal(struct move* m, double start_v, double end_v)
{
    double max_v2 = m->max_cruise_v2;
    double start_v2 = start_v * start_v;
    double end_v2 = end_v * end_v;
    double accel = m->accel;
    double distance = m->move_d;
    // The formula is calculated by solving cruise_v2 from
    // distance = (cruise_v2 - start_v2) / 2 + (cruise_v2 - end_v2) / 2
    // which is derived from the standard timeless kinematic formula
    double cruise_v2 = distance * accel + 0.5 * (start_v2 + end_v2);
    cruise_v2 = fmin(max_v2, cruise_v2);
    set_trapezoidal_times(m, distance, start_v2, cruise_v2, end_v2, accel);
}

struct move* __visible
move_alloc(
    double *start_pos,
    double *end_pos,
    double speed,
    double accel,
    double accel_to_decel,
    double jerk,
    struct move_queue* q)
{
    // TODO: Let the moves re-use the same C move until it's added to the 
    // planner
    // That ensures that the moves are continuous for the planner
    struct move *m = q->moves + (q->next_free % q->num_moves); 
    ++q->next_free;
    init_move(m, start_pos, end_pos, speed, accel, accel_to_decel, jerk);

    return m;
}