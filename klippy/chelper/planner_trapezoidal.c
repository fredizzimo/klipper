// Feedrate planner for moves with a trapezoidal speed profile
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "planner_trapezoidal.h"
#include "move.h"
#include "compiler.h"
#include <stdlib.h>
#include <math.h>

struct delayed_move
{
    struct move *m;
    double start_v2;
    double end_v2;
};

struct trapezoidal_planner
{
    struct delayed_move* delayed_moves;
    struct move_queue* queue;
};

struct trapezoidal_planner* __visible
trapezoidal_planner_alloc(struct move_queue *queue)
{
    struct trapezoidal_planner *planner = malloc(sizeof(*planner));
    planner->queue = queue;
    planner->delayed_moves =
        malloc(sizeof(struct delayed_move) * queue->allocated_size);
    trapezoidal_planner_reset(planner);
    return planner;
}

void __visible
trapezoidal_planner_reset(struct trapezoidal_planner *planner)
{
    move_queue_reset(planner->queue);
}

void __visible
trapezoidal_planner_free(struct trapezoidal_planner *planner)
{
    free(planner->delayed_moves);
    free(planner);
}

unsigned int __visible
trapezoidal_planner_flush(struct trapezoidal_planner *planner, bool lazy)
{
    bool update_flush_count = lazy;
    int flush_count = planner->queue->size;
    // Traverse queue from last to first move and determine maximum
    // junction speed assuming the robot comes to a complete stop
    // after the last move.
    unsigned delayed_count = 0;
    double next_end_v2 = 0.0;
    double next_smoothed_v2 = 0.0;
    double peak_cruise_v2 = 0.0;
    const unsigned queue_start = planner->queue->first;
    const unsigned mask = planner->queue->allocated_size - 1;
    struct move* moves = planner->queue->moves;
    for (unsigned i=planner->queue->size - 1;i!=(unsigned)(-1);--i)
    {
        struct move *move= &moves[(queue_start + i) & mask];
        double reachable_start_v2 = next_end_v2 + move->delta_v2;
        double start_v2 = fmin(move->max_start_v2, reachable_start_v2);
        double reachable_smoothed_v2 = next_smoothed_v2 + move->smooth_delta_v2;
        double smoothed_v2 = fmin(move->max_smoothed_v2, reachable_smoothed_v2);
        if (smoothed_v2 < reachable_smoothed_v2)
        {
            // It's possible for this move to accelerate
            if (smoothed_v2 + move->smooth_delta_v2 > next_smoothed_v2
                || delayed_count)
            {
                // This move can decelerate or this is a full accel
                // move after a full decel move
                if (update_flush_count && (peak_cruise_v2 > 0.0))
                {
                    flush_count = i;
                    update_flush_count = false;
                }
                peak_cruise_v2 = fmin(
                    move->max_cruise_v2, 
                    (smoothed_v2 + reachable_smoothed_v2) * .5);
                if (delayed_count)
                {
                    // Propagate peak_cruise_v2 to any delayed moves
                    if (!update_flush_count && i < flush_count)
                    {
                        double mc_v2 = peak_cruise_v2;
                        for (int j=delayed_count-1;j>=0;j--)
                        {
                            struct delayed_move *dm=&planner->delayed_moves[j];
                            struct move *m = dm->m;
                            const double ms_v2 = dm->start_v2;
                            const double me_v2 = dm->end_v2;
                            
                            mc_v2 = fmin(mc_v2, ms_v2);
                            set_trapezoidal_times(m, m->move_d,
                                ms_v2, mc_v2, me_v2, m->accel);
                        }
                    }
                    delayed_count = 0;
                }
            }
            if (!update_flush_count && i < flush_count)
            {
                double cruise_v2 = fmin(fmin(
                    (start_v2 + reachable_start_v2) * .5,
                    move->max_cruise_v2),
                    peak_cruise_v2);
                set_trapezoidal_times(move, move->move_d, start_v2,
                    cruise_v2, next_end_v2, move->accel);
            }
        }
        else
        {
            // Delay calculating this move until peak_cruise_v2 is known
            struct delayed_move *dm=&planner->delayed_moves[delayed_count++];
            dm->m = move;
            dm->start_v2 = start_v2;
            dm->end_v2 = next_end_v2;
        }
        next_end_v2 = start_v2;
        next_smoothed_v2 = smoothed_v2;
    }
    if (update_flush_count)
        return 0;

    if (flush_count > 0)
    {
        move_queue_flush(planner->queue, flush_count);
    }

    return flush_count;
}