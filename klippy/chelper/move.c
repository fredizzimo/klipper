// The C side implementation of the Move class
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "move.h"
#include "compiler.h"
#include "mathutil.h"
#include "pyhelper.h"
#include <stdlib.h>
#include <float.h>
#include <math.h>

static const double tolerance = 1e-13;
static const double time_tolerance = 1e-6;

static bool is_non_zero_power_of_two(unsigned int value)
{
    return (value & (value -1)) == 0 && value > 0;
}

struct move_queue* __visible
move_queue_alloc(unsigned int num_moves)
{
    if (!is_non_zero_power_of_two(num_moves))
    {
        errorf("The move queue size has to be a power of two");
    }
    struct move_queue *m = malloc(sizeof(*m));
    m->moves = malloc(sizeof(struct move)*num_moves);
    m->allocated_size = num_moves;
    move_queue_reset(m);
    return m;
}

void move_queue_reset(struct move_queue *queue)
{
    queue->size = 0;
    queue->first = 0;
}

void __visible
move_queue_free(struct move_queue *queue)
{
    free(queue->moves);
    free(queue);
}

void move_queue_flush(struct move_queue *queue, unsigned int count)
{
    queue->first += count;
    queue->size -= count;
}

bool __visible move_queue_is_full(struct move_queue *queue)
{
    return queue->size == queue->allocated_size;
}

void move_init(
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
    move_limit_speed(m, speed, accel, accel_to_decel);
}

void __visible
move_limit_speed(struct move *m, double speed, double accel,
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
move_calc_junction(struct move *m, struct move *prev_move,
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

void move_set_trapezoidal_times(struct move *m, double distance,
    double start_v2, double cruise_v2, double end_v2, double accel)
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
    m->total_t = m->accel_t + m->cruise_t + m->decel_t;
}

void __visible
move_calculate_trapezoidal(struct move* m, double start_v, double end_v)
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
    move_set_trapezoidal_times(m, distance, start_v2, cruise_v2, end_v2, accel);
}

static inline void limit_acceleration(double delta_v_jerk, double *accel,
    double *accel_2)
{
    *accel_2 = delta_v_jerk;
    *accel = sqrt(delta_v_jerk);
}

static inline double clamp_time(double time)
{
    if (time < time_tolerance)
        return 0.0;
    else
        return time;
}

static double calculate_dist_cruise(double start_v, double start_v2,
    double max_v, double max_v2, double end_v, double end_v2, double accel,
    double decel, double jerk, double accel_decel, double two_accel_decel_jerk,
    double two_accel_decel_distance_jerk)
{
    double dist_cruise = accel*start_v + accel*max_v + decel*max_v
        + decel*end_v;
    dist_cruise *= -accel_decel;
    dist_cruise += two_accel_decel_distance_jerk;
    dist_cruise += accel*jerk*(end_v2-max_v2);
    dist_cruise += decel*jerk*(start_v2-max_v2);
    dist_cruise /= two_accel_decel_jerk;
    return dist_cruise;
}

static void adapt_type_II(double start_v, double start_v2, double max_v,
    double max_v2, double end_v, double end_v2, double accel, double accel_2,
    double decel, double decel_2, double jerk, double two_jerk,
    double two_accel_decel, double two_accel_decel_jerk,
    double two_accel_decel_distance_jerk, double *dist_cruise,
    double *delta_accel_v, double *delta_decel_v, double *delta_accel_v_jerk,
    double *delta_decel_v_jerk)
{
    // Type II
    *dist_cruise = 0.0;

    double accel_plus_decel = accel + decel;

    double minus_a = accel_plus_decel / two_accel_decel;
    double minus_b = accel_plus_decel / two_jerk;

    double c = two_accel_decel_distance_jerk;
    c -= accel_2 * decel * start_v;
    c += accel * jerk * end_v2;
    c -= decel_2 * accel * end_v;
    c += decel * jerk * start_v2;
    c /= two_accel_decel_jerk;

    // b is always negative so use Citardauq formulation to solve the
    // quadratic equation. Which is more stable especially when
    // a*c is small compared to b^2
    // Note b and a are already negated, which saves a few operations
    // calculating them and the final results
    max_v = 2.0*c;
    max_v /= minus_b + sqrt(minus_b*minus_b + 4.0*minus_a*c);

    *delta_accel_v = max_v - start_v;
    *delta_decel_v = max_v - end_v;
    *delta_accel_v_jerk = *delta_accel_v*jerk;
    *delta_decel_v_jerk = *delta_decel_v*jerk;
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

    double y = (y0 - start_v2) / (2.0*y3);
    y += (y0 - end_v2) / x1;
    y += max_v*y3 / jerk;
    y += (decel*(max_v + end_v) - y3*y1) / x0;
    y -= distance;
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

static void adapt_type_IIII_a(double start_v, double start_v2, double end_v,
    double end_v2, double distance, double jerk, double abs_max_v, double decel,
    double decel_2, double two_jerk, double *max_v, double *accel,
    double *accel_2, double *delta_accel_v, double *delta_decel_v)
{
    *max_v = fmax(start_v, end_v) + tolerance;
    struct eval_type_IIII_a_state state = {
        .x0 = two_jerk,
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
    newton_raphson(eval_type_IIII_a, *max_v, abs_max_v,
        tolerance, 16, &res, &state);
    *max_v = res.x;

    *delta_accel_v = *max_v - start_v;
    *delta_decel_v = *max_v - end_v;
    const double delta_accel_v_jerk = *delta_accel_v*jerk;

    limit_acceleration(delta_accel_v_jerk, accel, accel_2);
}

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

    double y = (y0 - end_v2) / (2.0*y3);
    y += (y0 - start_v2) / x1;
    y += accel*max_v / jerk;
    y += (accel*(start_v - max_v) + y3*(max_v + end_v)) / x0;
    y -= distance;
    result->y = y;

    double dy = accel*y1;
    dy += accel*y3*(3.0*max_v - end_v);
    dy += y4*max_v;
    dy /= y4*accel;
    result->dy = dy;
}

static void adapt_type_IIII_b(double start_v, double start_v2, double end_v,
    double end_v2, double distance, double jerk, double abs_max_v, double accel,
    double accel_2, double two_jerk, double *max_v, double *decel,
    double *decel_2, double *delta_accel_v, double *delta_decel_v)
{
    *max_v = fmax(start_v, end_v) + tolerance;
    struct eval_type_IIII_b_state state = {
        .x0 = two_jerk,
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
    newton_raphson(eval_type_IIII_b, *max_v, abs_max_v,
        tolerance, 16, &res, &state);
    *max_v = res.x;
    *delta_accel_v = *max_v - start_v;
    *delta_decel_v = *max_v - end_v;
    const double delta_decel_v_jerk = *delta_decel_v*jerk;

    limit_acceleration(delta_decel_v_jerk, decel, decel_2);
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

static void adapt_type_IIII_c(double start_v, double start_v2, double end_v,
    double end_v2, double distance, double jerk, double abs_max_v,
    double *max_v, double *accel, double *accel_2, double *decel,
    double *decel_2, double *delta_accel_v, double *delta_decel_v)
{

    *max_v = fmax(start_v, end_v) + tolerance;
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
    newton_raphson(eval_type_IIII_c, *max_v, abs_max_v,
        tolerance, 16, &res, &state);

    *max_v = res.x;
    *delta_accel_v = *max_v - start_v;
    *delta_decel_v = *max_v - end_v;
    const double delta_accel_v_jerk = *delta_accel_v*jerk;
    const double delta_decel_v_jerk = *delta_decel_v*jerk;

    limit_acceleration(delta_accel_v_jerk, accel, accel_2);
    limit_acceleration(delta_decel_v_jerk, decel, decel_2);
}

static void adapt_no_accel(double distance, double max_v, double end_v,
    double decel, double jerk, double delta_decel_v, double *dist_cruise,
    double *decel_t)
{
    *dist_cruise = distance;
    *dist_cruise -= (max_v*max_v - end_v*end_v) / (2.0*decel);
    *dist_cruise -= (decel * (max_v + end_v)) / (2.0*jerk);
    *decel_t = delta_decel_v / decel;
}

static void adapt_no_decel(double distance, double start_v, double max_v,
    double end_v, double accel, double jerk, double delta_accel_v,
    double *dist_cruise, double *accel_t)
{
        *dist_cruise = distance;
        *dist_cruise -= (max_v*max_v - start_v*start_v) / (2.0*accel);
        *dist_cruise -= (accel * (start_v - max_v)) / (2.0*jerk);
        *dist_cruise -= (accel * max_v) / jerk;
        *accel_t = delta_accel_v / accel;
}

static void set_constant_jerk_profile(struct move *m, double distance,
    double start_v, double max_v, double end_v, double jerk)
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
    m->total_t = cruise_t;
}

static void set_jerk_profile(struct move *m, double start_v, double max_v,
    double end_v, double accel, double decel, double jerk, double accel_t,
    double decel_t, double dist_cruise)
{
    m->jerk = jerk;
    m->start_v = start_v;
    m->cruise_v = max_v;
    m->end_v = end_v;

    // Clamp to zero to remove empty and negative segments
    double accel_jerk_t = clamp_time(accel / jerk);
    double decel_jerk_t = clamp_time(decel / jerk);
    double accel_const_t = clamp_time(accel_t - accel_jerk_t);
    double decel_const_t = clamp_time(decel_t - decel_jerk_t);
    double cruise_t = clamp_time(dist_cruise / max_v);

    m->jerk_t[0] = accel_jerk_t;
    m->jerk_t[1] = accel_const_t;
    m->jerk_t[2] = accel_jerk_t;
    m->jerk_t[3] = cruise_t;
    m->jerk_t[4] = decel_jerk_t;
    m->jerk_t[5] = decel_const_t;
    m->jerk_t[6] = decel_jerk_t;

    m->total_t = accel_jerk_t;
    m->total_t += accel_const_t;
    m->total_t += accel_jerk_t;
    m->total_t += cruise_t;
    m->total_t += decel_jerk_t;
    m->total_t += decel_const_t;
    m->total_t += decel_jerk_t;
}

void __visible
move_calculate_jerk(struct move* m, double start_v, double end_v)
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
        return set_constant_jerk_profile(m, distance, start_v, max_v, end_v,
            jerk);
    }

    double decel = accel;

    double delta_accel_v = max_v - start_v;
    double delta_decel_v = max_v - end_v;
    double delta_accel_v_jerk = delta_accel_v*jerk;
    double delta_decel_v_jerk = delta_decel_v*jerk;
    double accel_2 = accel * accel;
    double decel_2 = accel_2;

    // type III adaptations
    if (delta_accel_v_jerk < accel_2)
    {
        limit_acceleration(delta_accel_v_jerk, &accel, &accel_2);
    }
    if (delta_decel_v_jerk < decel_2)
    {
        limit_acceleration(delta_decel_v_jerk, &decel, &decel_2);
    }

    double accel_t = 0.0;
    double decel_t = 0.0;

    double dist_cruise = 0.0;
    if (accel > 0.0 && decel > 0.0)
    {
        const double start_v2 = start_v * start_v;
        const double max_v2 = max_v * max_v;
        const double end_v2 = end_v * end_v;
        const double accel_decel = accel * decel;

        const double two_accel_decel = 2.0 * accel_decel;
        const double two_accel_decel_jerk = two_accel_decel * jerk;
        const double two_accel_decel_distance_jerk =
            two_accel_decel_jerk*distance;

        dist_cruise = calculate_dist_cruise(start_v, start_v2, max_v, max_v2,
            end_v, end_v2, accel, decel, jerk, accel_decel,
            two_accel_decel_jerk, two_accel_decel_distance_jerk);

        if (dist_cruise < 0)
        {
            const double two_jerk = 2.0*jerk;
            adapt_type_II(start_v, start_v2, max_v, max_v2, end_v, end_v2,
                accel, accel_2, decel, decel_2, jerk, two_jerk, two_accel_decel,
                two_accel_decel_jerk, two_accel_decel_distance_jerk,
                &dist_cruise, &delta_accel_v, &delta_decel_v,
                &delta_accel_v_jerk, &delta_decel_v_jerk);
            if (delta_accel_v_jerk < accel_2)
            {
                if (delta_decel_v_jerk < decel_2)
                {
                    adapt_type_IIII_c(start_v, start_v2, end_v, end_v2,
                        distance, jerk, abs_max_v, &max_v, &accel, &accel_2,
                        &decel, &decel_2, &delta_accel_v, &delta_decel_v);
                }
                else
                {
                    adapt_type_IIII_a(start_v, start_v2, end_v, end_v2,
                        distance, jerk, abs_max_v, decel, decel_2, two_jerk,
                        &max_v, &accel, &accel_2, &delta_accel_v,
                        &delta_decel_v);
                }
            }
            else if (delta_decel_v_jerk < decel_2)
            {
                adapt_type_IIII_b(start_v, start_v2, end_v, end_v2, distance,
                    jerk, abs_max_v, accel, accel_2, two_jerk, &max_v, &decel,
                    &decel_2, &delta_accel_v, &delta_decel_v);
            }
        }
        accel_t = delta_accel_v / accel;
        decel_t = delta_decel_v / decel;
    }
    else if (decel > 0)
    {
        adapt_no_accel(distance, max_v, end_v, decel, jerk, delta_decel_v,
            &dist_cruise, &decel_t);
    }
    else
    {
        adapt_no_decel(distance, start_v, max_v, end_v, accel, jerk,
            delta_accel_v, &dist_cruise, &accel_t);
    }

    set_jerk_profile(m, start_v, max_v, end_v, accel, decel, jerk, accel_t,
        decel_t, dist_cruise);
}

struct get_max_allowed_jerk_end_speed_state
{
    double d2;
    double jerk;
    double start_v;
};

static
void eval_get_max_allowed_jerk_end_speed(
    struct newton_raphson_result *result, void* user_data)
{
    struct get_max_allowed_jerk_end_speed_state *state = user_data;
    double d2 = state->d2;
    double jerk = state->jerk;
    double start_v = state->start_v;
    double v = result->x;

    double x0 = v - start_v;
    double x1 = v + start_v;
    double y = (x1/jerk)*x0*x1 - d2;
    result->y = y;

    double dy = x1 * (3.0*v - start_v);
    dy /= jerk;
    result->dy = dy;
}

double __visible
move_get_max_allowed_jerk_end_speed(double distance, double start_v,
    double end_v, double max_a, double jerk)
{
    // TODO Should we use the same tolerances as the global one
    double tolerance = 1e-6;

    double max_a_2 = max_a*max_a;
    double max_a_3 = max_a_2*max_a;
    double max_a_dist = max_a_3 / (jerk*jerk) + 2.0 * max_a * start_v / jerk;
    if (distance < max_a_dist)
    {
        struct get_max_allowed_jerk_end_speed_state state = {
            .d2 = distance*distance,
            .jerk = jerk,
            .start_v = start_v,
        };
        struct newton_raphson_result res;
        newton_raphson(eval_get_max_allowed_jerk_end_speed, start_v, end_v,
            tolerance, 16, &res, &state);

        return res.x;
    }
    else
    {
        double max_a_4 = max_a_3*max_a;
        double end_v = 8.0 * max_a * distance + 4.0 * start_v*start_v;
        end_v *= jerk;
        end_v -= 4.0 * max_a_2 * start_v;
        end_v *= jerk;
        end_v += max_a_4;

        end_v = sqrt(end_v);
        end_v -= max_a_2;
        end_v /= 2.0 * jerk;
        return end_v;
    }
}

bool move_can_accelerate_fully(double distance, double start_v, double end_v,
    double accel, double jerk)
{
    double jerk_t2 = end_v - start_v;
    jerk_t2 /= jerk;
    jerk_t2 *= 2.0;

    double a_div_jerk = accel / jerk;
    double d;

    // If there's a constant acceleration phase
    if (jerk_t2 > a_div_jerk*a_div_jerk)
    {
        double d1 = end_v*end_v - start_v*start_v;
        d1 /= 2.0 * accel;

        double d2 = accel*accel / (12.0*jerk);
        d2 += start_v;
        d2 *= accel / (2.0*jerk);
        d = d1 + d2;
    }
    else
    {
        d = sqrt(jerk_t2);
        d *= 2.0*start_v + end_v;
        d /= 3.0;
    }
    return d > distance;
}

struct move* __visible
move_reserve(
    double *start_pos,
    double *end_pos,
    double speed,
    double accel,
    double accel_to_decel,
    double jerk,
    struct move_queue* q)
{
    if (move_queue_is_full(q))
    {
        errorf("The move queue is full, flush it before reserving more moves");
        return NULL;
    }
    unsigned int index = (q->first + q->size) & (q->allocated_size - 1);
    struct move *m = &q->moves[index];
    move_init(m, start_pos, end_pos, speed, accel, accel_to_decel, jerk);
    return m;
}

void __visible move_commit(struct move_queue *queue)
{
    queue->size++;
}
