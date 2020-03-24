// Move queue
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "movequeue.h"
#include "compiler.h"
#include "mathutil.h"
#include "pyhelper.h"
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

struct eval_type_IIII_a_state
{
    double x0;
    double x1;
    double start_v;
    double start_v2;
    double end_v;
    double end_v2;
    double jerk;
    double distance;
    double decel;
    double decel_2;
};

static
void eval_type_IIII_a(struct newton_raphson_result *result, void* user_data)
{
    struct eval_type_IIII_a_state *state = user_data;
    double x0 = state->x0;
    double x1 = state->x1;
    double start_v = state->start_v;
    double start_v2 = state->start_v2;
    double end_v = state->end_v;
    double end_v2 = state->end_v2;
    double jerk = state->jerk;
    double distance = state->distance;
    double decel = state->decel;
    double decel_2 = state->decel_2;
    double max_v = result->x;

    double y0 = max_v*max_v;
    double y1 = max_v - start_v;
    double y2 = jerk*y1;
    double y3 = sqrt(y2);
    double y4 = x0*y1;

    double y = -distance;
    y += (y0 - start_v2) / (2.0*y3);
    y += (y0 - end_v2) / x1;
    y += max_v*y3 / jerk;
    y += (decel*(max_v + end_v) - y3*y1) / x0;
    result->y = y;

    double dy = decel_2*y1;
    dy += decel*y3*(3.0*max_v - start_v);
    dy += y4*max_v;
    dy /= y4*decel;
    result->dy = dy;
}

struct eval_type_IIII_b_state
{
    double x0;
    double x1;
    double start_v;
    double start_v2;
    double end_v;
    double end_v2;
    double jerk;
    double distance;
    double accel;
};

static
void eval_type_IIII_b(struct newton_raphson_result *result, void* user_data)
{
    struct eval_type_IIII_b_state *state = user_data;
    double x0 = state->x0;
    double x1 = state->x1;
    double start_v = state->start_v;
    double start_v2 = state->start_v2;
    double end_v = state->end_v;
    double end_v2 = state->end_v2;
    double jerk = state->jerk;
    double distance = state->distance;
    double accel = state->accel;
    double max_v = result->x;

    double y0 = max_v*max_v;
    double y1 = max_v - end_v;
    double y2 = jerk*y1;
    double y3 = sqrt(y2);
    double y4 = x0*y1;

    double y = -distance;
    y += (y0 - end_v2) / (2.0*y3);
    y += (y0 - start_v2) / x1;
    y += accel*max_v / jerk;
    y += (accel*(start_v - max_v) + y3*(max_v + end_v)) / x0;
    result->y = y;

    double dy = accel*y1;
    dy += accel*y3*(3.0*max_v - end_v);
    dy += y4*max_v;
    dy /= y4*accel;
    result->dy = dy;
}

struct eval_type_IIII_c_state
{
    double x0;
    double x1;
    double x2;
    double x3;
    double jerk;
    double distance;
    double start_v2;
    double end_v2;
};

static
void eval_type_IIII_c(struct newton_raphson_result *result, void* user_data)
{
    struct eval_type_IIII_c_state *state = user_data;
    double x0 = state->x0;
    double x1 = state->x1;
    double x2 = state->x2;
    double x3 = state->x3;
    double max_v = result->x;
    double jerk = state->jerk;
    double distance = state->distance;
    double start_v2 = state->start_v2;
    double end_v2 = state->end_v2;

    double y0 = jerk*max_v;
    double y1 = y0 - x0;
    double y2 = y0 - x1;
    double y3 = sqrt(y0-x0);
    double y4 = sqrt(y0-x1);
    double y5 = 2.0*y1*y3;
    double y6 = 2.0*y2*y4;
    double y7 = max_v*max_v;
    double y8 = jerk*y7;
    double y9 = 2.0*max_v;

    double y = distance;
    y += (start_v2 - y7) / y3;
    y += (end_v2 - y7) / y4;
    result->y = y;

    double dy = (y8 - x2)  / y5;
    dy += (y8 - x3) / y6;
    dy -= y9 / y3;
    dy -= y9 / y4;
    result->dy = dy;
}

void __visible
calculate_jerk(struct move* m, double start_v, double end_v)
{
    // Calculate a jerk limited profile based on the paper
    // FIR filter-based online jerk-constrained trajectory generation
    // by Pierre Besset and Richard Béarée

    // Make sure that max_v not smaller than the endpoints, due to rounding
    // errors
    double max_v = fmax(fmax(sqrt(m->max_cruise_v2), start_v), end_v);
    double distance = m->move_d;
    double jerk = m->jerk;
    double accel = m->accel;
    double abs_max_v = max_v;

    // If no speed change is allowed, then create a constant
    // profile
    if (fabs(start_v - end_v) <= tolerance &&
        fabs(start_v - max_v) <= tolerance)
    {
        double cruise_t = distance / max_v;
        m->jerk = jerk;
        m->start_v = start_v;
        m->cruise_v = max_v;
        m->end_v = end_v;
        m->jerk_t[0] = 0.0;
        m->jerk_t[1] = 0.0;
        m->jerk_t[2] = 0.0;
        m->jerk_t[3] = cruise_t;
        m->jerk_t[4] = 0.0;
        m->jerk_t[5] = 0.0;
        m->jerk_t[6] = 0.0;
        return;
    }

    double decel = accel;

    double accel_jerk_t = accel / jerk;
    double decel_jerk_t = decel / jerk;
    double delta_accel_v = max_v - start_v;
    double delta_decel_v = max_v - end_v;
    double accel_t = delta_accel_v / accel;
    double decel_t = delta_decel_v / decel;
    double accel_const_t = accel_t - accel_jerk_t;
    double decel_const_t = decel_t - decel_jerk_t;

    // type III adaptations
    if (accel_const_t < 0.0)
        accel = sqrt(jerk * delta_accel_v);
    if (decel_const_t < 0)
        decel = sqrt(jerk * delta_decel_v);

    double dist_cruise = 0.0;
    if (accel > 0.0 && decel > 0.0)
    {
        double start_v2 = start_v * start_v;
        double max_v2 = max_v * max_v;
        double end_v2 = end_v * end_v;
        double accel_decel = accel * decel;

        double two_accel_decel = 2.0 * accel_decel;
        double two_accel_decel_jerk = two_accel_decel * jerk;
        double two_accel_decel_distance_jerk = two_accel_decel_jerk*distance;

        dist_cruise = accel*start_v + accel*max_v + decel*max_v
            + decel*end_v;
        dist_cruise *= -accel_decel;
        dist_cruise += two_accel_decel_distance_jerk;
        dist_cruise += accel*jerk*(end_v2-max_v2);
        dist_cruise += decel*jerk*(start_v2-max_v2);
        dist_cruise /= two_accel_decel_jerk;

        if (dist_cruise < 0)
        {
            // Type II
            dist_cruise = 0.0;

            double m_accel_m_decel = -accel - decel;
            double accel_2 = accel * accel;
            double decel_2 = decel * decel;

            double a = m_accel_m_decel;
            a /= two_accel_decel;

            double b = m_accel_m_decel;
            b /= 2.0 * jerk;

            double c = -accel_2 * decel * start_v;
            c -= decel_2 * accel * end_v;
            c += two_accel_decel_distance_jerk;
            c += accel * jerk * end_v2;
            c += decel * jerk * start_v2;
            c /= two_accel_decel_jerk;

            // TODO Use the other form of the formula to prevent precision
            // issues
            max_v = -b - sqrt(b*b - 4.0*a*c);
            max_v /= 2.0*a;

            accel_jerk_t = accel / jerk;
            decel_jerk_t = decel / jerk;
            double delta_accel_v = max_v - start_v;
            double delta_decel_v = max_v - end_v;
            accel_t = delta_accel_v / accel;
            decel_t = delta_decel_v / decel;
            accel_const_t = accel_t - accel_jerk_t;
            decel_const_t = decel_t - decel_jerk_t;
            if (accel_const_t < 0)
            {
                // Type IIII-c
                if (decel_const_t < 0)
                {
                    max_v = fmax(start_v, end_v) + tolerance;
                    struct eval_type_IIII_c_state state = {
                        .x0 = jerk*start_v,
                        .x1 = jerk*end_v,
                        .x2 = jerk*start_v2,
                        .x3 = jerk*end_v2,
                        .jerk = jerk,
                        .distance = distance,
                        .start_v2 = start_v2,
                        .end_v2 = end_v2,
                    };
                    struct newton_raphson_result res;
                    newton_raphson(eval_type_IIII_c, max_v, abs_max_v,
                        tolerance, 16, &res, &state);

                    max_v = res.x;
                    accel = sqrt(jerk*(max_v - start_v));
                    decel = sqrt(jerk*(max_v - end_v));
                }
                // Type IIII-a
                else
                {
                    max_v = fmax(start_v, end_v) + tolerance;
                    struct eval_type_IIII_a_state state = {
                        .x0 = 2.0*jerk,
                        .x1 = 2.0*decel,
                        .start_v = start_v,
                        .start_v2 = start_v2,
                        .end_v = end_v,
                        .end_v2 = end_v2,
                        .jerk = jerk,
                        .distance = distance,
                        .decel = decel,
                        .decel_2 = decel_2
                    };
                    struct newton_raphson_result res;
                    newton_raphson(eval_type_IIII_a, max_v, abs_max_v,
                        tolerance, 16, &res, &state);
                    max_v = res.x;
                    accel = sqrt(jerk*(max_v - start_v));
                }
            }
            // Type IIII-b
            else if (decel_const_t < 0)
            {
                max_v = fmax(start_v, end_v) + tolerance;
                struct eval_type_IIII_b_state state = {
                    .x0 = 2.0*jerk,
                    .x1 = 2.0*accel,
                    .start_v = start_v,
                    .start_v2 = start_v2,
                    .end_v = end_v,
                    .end_v2 = end_v2,
                    .jerk = jerk,
                    .distance = distance,
                    .accel = accel,
                };
                struct newton_raphson_result res;
                newton_raphson(eval_type_IIII_b, max_v, abs_max_v,
                    tolerance, 16, &res, &state);
                max_v = res.x;
                decel = sqrt(jerk*(max_v - end_v));
            }
        }
    }
    else if (decel > 0)
    {
        dist_cruise = distance;
        dist_cruise -= (max_v*max_v - end_v*end_v) / (2.0*decel);
        dist_cruise -= (decel * (max_v + end_v)) / (2.0*jerk);
    }
    else
    {
        dist_cruise = distance;
        dist_cruise -= (max_v*max_v - start_v*start_v) / (2.0*accel);
        dist_cruise -= (accel * (start_v - max_v)) / (2.0*jerk);
        dist_cruise -= (accel * max_v) / jerk;
    }

    // TODO: This code is duplicated
    accel_jerk_t = accel / jerk;
    if (accel_jerk_t < time_tolerance)
        accel_jerk_t = 0.0;
    decel_jerk_t = decel / jerk;
    if (decel_jerk_t < time_tolerance)
        decel_jerk_t = 0.0;
    delta_accel_v = max_v - start_v;
    delta_decel_v = max_v - end_v;
    if (accel > 0.0)
        accel_t = delta_accel_v / accel;
    else
        accel_t = 0.0;
    if (decel > 0.0)
        decel_t = delta_decel_v / decel;
    else
        decel_t = 0.0;
    accel_const_t = accel_t - accel_jerk_t;
    decel_const_t = decel_t - decel_jerk_t;

    m->jerk = jerk;
    m->start_v = start_v;
    m->cruise_v = max_v;
    m->end_v = end_v;
    double cruise_t = dist_cruise / max_v;

    // Clamp to zero to remove empty segments
    if (accel_const_t < time_tolerance)
        accel_const_t = 0.0;
    if (cruise_t < time_tolerance)
        cruise_t = 0.0;
    if (decel_const_t < time_tolerance)
        decel_const_t = 0.0;

    m->jerk_t[0] = accel_jerk_t;
    m->jerk_t[1] = accel_const_t;
    m->jerk_t[2] = accel_jerk_t;
    m->jerk_t[3] = cruise_t;
    m->jerk_t[4] = decel_jerk_t;
    m->jerk_t[5] = decel_const_t;
    m->jerk_t[6] = decel_jerk_t;
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