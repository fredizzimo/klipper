// Queue for fixed velocity/acceleration/jerk movement segments
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // unlikely
#include "segq.h" // move_get_coord
#include "move.h"

// Allocate a new 'move' object
struct segq_move *
segq_move_alloc(void)
{
    struct segq_move *m = malloc(sizeof(*m));
    memset(m, 0, sizeof(*m));
    return m;
}

// Fill and add a move to the trapezoid velocity queue
void __visible
segq_append(struct segq *tq, double print_time
             , double accel_t, double cruise_t, double decel_t
             , double start_pos_x, double start_pos_y, double start_pos_z
             , double axes_r_x, double axes_r_y, double axes_r_z
             , double start_v, double cruise_v, double accel)
{
    struct coord start_pos = { .x=start_pos_x, .y=start_pos_y, .z=start_pos_z };
    struct coord axes_r = { .x=axes_r_x, .y=axes_r_y, .z=axes_r_z };
    if (accel_t) {
        struct segq_move *m = segq_move_alloc();
        m->print_time = print_time;
        m->move_t = accel_t;
        m->start_v = start_v;
        m->half_accel = .5 * accel;
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        segq_add_move(tq, m);

        print_time += accel_t;
        start_pos = move_get_coord(m, accel_t);
    }
    if (cruise_t) {
        struct segq_move *m = segq_move_alloc();
        m->print_time = print_time;
        m->move_t = cruise_t;
        m->start_v = cruise_v;
        m->half_accel = 0.;
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        segq_add_move(tq, m);

        print_time += cruise_t;
        start_pos = move_get_coord(m, cruise_t);
    }
    if (decel_t) {
        struct segq_move *m = segq_move_alloc();
        m->print_time = print_time;
        m->move_t = decel_t;
        m->start_v = cruise_v;
        m->half_accel = -.5 * accel;
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        segq_add_move(tq, m);
    }
}

void __visible
segq_append_move(struct segq *tq, double print_time, struct move *m)
{
    segq_append(tq, print_time, m->accel_t, m->cruise_t, m->decel_t,
        m->start_pos[0], m->start_pos[1], m->start_pos[2],
        m->axes_r[0], m->axes_r[1], m->axes_r[2], m->start_v, m->cruise_v,
        m->accel);
}

// Return the distance moved given a time in a move
inline double
move_get_distance(struct segq_move *m, double move_time)
{
    return (m->start_v + m->half_accel * move_time) * move_time;
}

// Return the XYZ coordinates given a time in a move
inline struct coord
move_get_coord(struct segq_move *m, double move_time)
{
    double move_dist = move_get_distance(m, move_time);
    return (struct coord) {
        .x = m->start_pos.x + m->axes_r.x * move_dist,
        .y = m->start_pos.y + m->axes_r.y * move_dist,
        .z = m->start_pos.z + m->axes_r.z * move_dist };
}

#define NEVER_TIME 9999999999999999.9

// Allocate a new 'segq' object
struct segq * __visible
segq_alloc(void)
{
    struct segq *tq = malloc(sizeof(*tq));
    memset(tq, 0, sizeof(*tq));
    list_init(&tq->moves);
    struct segq_move *head_sentinel = segq_move_alloc();
    struct segq_move *tail_sentinel = segq_move_alloc();
    tail_sentinel->print_time = tail_sentinel->move_t = NEVER_TIME;
    list_add_head(&head_sentinel->node, &tq->moves);
    list_add_tail(&tail_sentinel->node, &tq->moves);
    return tq;
}

// Free memory associated with a 'segq' object
void __visible
segq_free(struct segq *tq)
{
    while (!list_empty(&tq->moves)) {
        struct segq_move *m = list_first_entry(&tq->moves, struct segq_move,
                                                node);
        list_del(&m->node);
        free(m);
    }
    free(tq);
}

// Update the list sentinels
void
segq_check_sentinels(struct segq *tq)
{
    struct segq_move *tail_sentinel = list_last_entry(&tq->moves,
        struct segq_move, node);
    if (tail_sentinel->print_time)
        // Already up to date
        return;
    struct segq_move *m = list_prev_entry(tail_sentinel, node);
    struct segq_move *head_sentinel =
        list_first_entry(&tq->moves, struct segq_move, node);
    if (m == head_sentinel) {
        // No moves at all on this list
        tail_sentinel->print_time = NEVER_TIME;
        return;
    }
    tail_sentinel->print_time = m->print_time + m->move_t;
    tail_sentinel->start_pos = move_get_coord(m, m->move_t);
}

#define MAX_NULL_MOVE 1.0

// Add a move to the trapezoid velocity queue
void
segq_add_move(struct segq *tq, struct segq_move *m)
{
    struct segq_move *tail_sentinel = list_last_entry(&tq->moves,
        struct segq_move, node);
    struct segq_move *prev = list_prev_entry(tail_sentinel, node);
    if (prev->print_time + prev->move_t < m->print_time) {
        // Add a null move to fill time gap
        struct segq_move *null_move = segq_move_alloc();
        null_move->start_pos = m->start_pos;
        if (!prev->print_time && m->print_time > MAX_NULL_MOVE)
            // Limit the first null move to improve numerical stability
            null_move->print_time = m->print_time - MAX_NULL_MOVE;
        else
            null_move->print_time = prev->print_time + prev->move_t;
        null_move->move_t = m->print_time - null_move->print_time;
        list_add_before(&null_move->node, &tail_sentinel->node);
    }
    list_add_before(&m->node, &tail_sentinel->node);
    tail_sentinel->print_time = 0.;
}

// Free any moves older than `print_time` from the trapezoid velocity queue
void __visible
segq_free_moves(struct segq *tq, double print_time)
{
    struct segq_move *head_sentinel = list_first_entry(&tq->moves,
                                                        struct segq_move,node);
    struct segq_move *tail_sentinel = list_last_entry(&tq->moves,
                                                       struct segq_move, node);
    for (;;) {
        struct segq_move *m = list_next_entry(head_sentinel, node);
        if (m == tail_sentinel) {
            tail_sentinel->print_time = NEVER_TIME;
            return;
        }
        if (m->print_time + m->move_t > print_time)
            return;
        list_del(&m->node);
        free(m);
    }
}
