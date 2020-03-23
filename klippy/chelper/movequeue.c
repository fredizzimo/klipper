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
        move_d = abs(m->axes_d[3]);
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