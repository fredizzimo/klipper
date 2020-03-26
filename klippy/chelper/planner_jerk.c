// Feedrate planner for moves with a 7 segment jerk limited profile
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "planner_jerk.h"
#include "move.h"
#include "mathutil.h"
#include "compiler.h"
#include <stdlib.h>
#include <math.h>

const double tolerance = 1e-9;
double jerk_multipliers[] = {
    1.0,
    0.0,
    -1.0,
    0.0,
    -1.0,
    0.0,
    1.0
};

struct virtual_move
{
    unsigned move_count;
    unsigned start_move_index;

    double start_v;
    double accel;
    double distance;
    double jerk;
    double end_v;
    double cruise_v;

    double x;
    double v;
    double a;
    double segment_start_x;
    double segment_start_v;
    double segment_start_a;
    double segment_end_x;
    double segment_end_v;
    double segment_end_a;

    unsigned current_segment;
    double current_segment_offset;

    struct move move;
};

static void init_virtual_move(struct virtual_move * vmove, double start_v,
    double accel, double jerk)
{
    vmove->start_v = start_v;
    vmove->accel = accel;
    vmove->distance = 0.0;
    vmove->jerk = jerk;
    vmove->end_v = 0.0;
    vmove->cruise_v = 0;
    vmove->move_count = 0;
    vmove->start_move_index = 0;

    vmove->x = 0.0;
    vmove->v = 0.0;
    vmove->a = 0.0;
    vmove->segment_start_x = 0.0;
    vmove->segment_start_v = 0.0;
    vmove->segment_start_a = 0.0;
    vmove->segment_end_x = 0.0;
    vmove->segment_end_v = 0.0;
    vmove->segment_end_a = 0.0;

    vmove->current_segment = 0.0;
    vmove->current_segment_offset = 0.0;
}


struct jerk_planner
{
    struct move_queue *queue;
    struct virtual_move *virtual_moves;

    struct virtual_move *start_vmove;
    struct virtual_move *end_vmove;

    double current_v;

    struct virtual_move **output_vmoves;
    unsigned num_output_vmoves;
};

struct jerk_planner* __visible
jerk_planner_alloc(struct move_queue *queue)
{
    struct jerk_planner *planner = malloc(sizeof(*planner));
    planner->queue = queue;
    planner->virtual_moves =
        malloc(sizeof(struct virtual_move)*planner->queue->allocated_size);
    planner->output_vmoves = 
        malloc(sizeof(struct virtual_move*)*planner->queue->allocated_size);
    
    jerk_planner_reset(planner);
    
    return planner;
}

void __visible
jerk_planner_reset(struct jerk_planner * planner)
{
    planner->start_vmove = NULL;
    planner->end_vmove = NULL;
    planner->current_v = 0.0;
    planner->num_output_vmoves = 0;
}

void __visible
jerk_planner_free(struct jerk_planner *planner)
{
    free(planner->virtual_moves);
    free(planner->output_vmoves);
    free(planner);
}

static struct virtual_move *create_virtual_move(struct jerk_planner *planner, 
    double start_v, double accel, double jerk)
{
    if (planner->start_vmove == NULL)
    {
        planner->start_vmove = planner->virtual_moves;
        planner->end_vmove = planner->start_vmove + 1;
    }
    else
    {
        planner->end_vmove++;
    }

    struct virtual_move* vmove = planner->end_vmove - 1;
    init_virtual_move(vmove, start_v, accel, jerk);

    return vmove;
}

static void append_move(struct virtual_move *vmove, unsigned index)
{
    // This assumes that the indices are continuous
    if (vmove->move_count == 0)
    {
        vmove->move_count = 1;
        vmove->start_move_index = index;
    }
    else
    {
        vmove->move_count++;
    }
}

static void append_moves(struct virtual_move *to, struct virtual_move *from)
{
    // This assumes that we are moving backwards, and the moves are continuous
    to->move_count += from->move_count;
}
static double calculate_x(double x, double v, double a, double j, double t)
{
    const double t2 = t*t;
    const double t3 = t2*t;
    x += v*t;
    x += 0.5 * a*t2;
    x += j*t3/6.0;
    return x;
}

static double calculate_v(double v, double a, double j, double t)
{
    v += a*t;
    v += 0.5 * j*t*t;
    return v;
}

static double calculate_a(double a, double j, double t)
{
    return a + j*t;
}

static void calculate_segment_end(struct virtual_move *vmove)
{

    double j = jerk_multipliers[vmove->current_segment] * vmove->jerk;
    double t = vmove->move.jerk_t[vmove->current_segment];

    double x = vmove->segment_start_x;
    double v = vmove->segment_start_v;
    double a = vmove->segment_start_a;

    vmove->segment_end_x = calculate_x(x, v, a, j, t);
    vmove->segment_end_v = calculate_v(v, a, j, t);
    vmove->segment_end_a = calculate_a(a, j, t);

    vmove->current_segment_offset = 0.0;
}

static void calculate_first_segment(struct virtual_move *vmove)
{
    vmove->x = 0.0;
    vmove->v = vmove->start_v;
    vmove->a = 0.0;
    vmove->segment_start_x = vmove->x;
    vmove->segment_start_v = vmove->v;
    vmove->segment_start_a = vmove->a;
    vmove->current_segment = 0;
    calculate_segment_end(vmove);
}

static void calculate_next_segment(struct virtual_move *vmove)
{
    vmove->x = vmove->segment_end_x;
    vmove->v = vmove->segment_end_v;
    vmove->a = vmove->segment_end_a;
    vmove->segment_start_x = vmove->x;
    vmove->segment_start_v = vmove->v;
    vmove->segment_start_a = vmove->a;
    vmove->current_segment += 1;
    calculate_segment_end(vmove);
}

struct eval_move_to_state
{
    double x;
    double v;
    double a;
    double j;
};

static void eval_move_to(struct newton_raphson_result *result, void* user_data)
{
    struct eval_move_to_state *state = user_data;
    double t = result->x;
    double x = calculate_x(state->x, state->v, state->a, state->j, t);
    double v = calculate_v(state->v, state->a, state->j, t);
    result->y = x;
    result->dy = v;
}

static double move_to(struct virtual_move *vmove, double d)
{
    double tolerance = 1e-16;
    
    struct eval_move_to_state state = {
        .x = vmove->segment_start_x - d,
        .v = vmove->segment_start_v,
        .a = vmove->segment_start_a,
        .j = jerk_multipliers[vmove->current_segment] * vmove->jerk,
    };

    struct newton_raphson_result res;

    newton_raphson(eval_move_to, 0, vmove->move.jerk_t[vmove->current_segment],
        tolerance, 16, &res, &state);

    double t = res.x;
    vmove->x = res.y;
    vmove->v = res.dy;
    vmove->a = calculate_a(state.a, state.j, t);
    double ret = t - vmove->current_segment_offset;
    vmove->current_segment_offset = t;

    return ret;
}

static void calculate_profile(struct virtual_move *vmove)
{
    double start_pos[] = {0.0, 0.0, 0.0, 0.0};
    double end_pos[] = {vmove->distance, 0.0, 0.0, 0.0};
    move_init(&vmove->move, start_pos, end_pos, vmove->cruise_v, vmove->accel,
        vmove->accel, vmove->jerk);
    move_calculate_jerk(&vmove->move, vmove->start_v, vmove->end_v);
}

static bool try_combine_with_next(
    bool next_move, double next_accel, double next_jerk,
    double next_max_cruise_v2, double distance, double start_v, double end_v,
    double end_v2, double accel, double jerk, double *reachable_speed)
{
    double reachable_end_v = move_get_max_allowed_jerk_end_speed(
        distance, start_v, end_v, accel, jerk);

    if (!next_move || next_accel != accel ||
            next_jerk != jerk)
    {
        *reachable_speed = reachable_end_v;
        return false;
    }

    bool can_reach_end = reachable_end_v >= end_v;
    if (can_reach_end)
    {
        *reachable_speed = reachable_end_v;
        return false;
    }

    if (next_max_cruise_v2 == end_v2)
    {
        *reachable_speed = end_v;
        return true;
    }

    *reachable_speed = reachable_end_v;
    return move_can_accelerate_fully(
        distance, start_v, end_v, accel, jerk);
}

static bool try_combine_with_next_move(
    struct move *next_move, double distance, double start_v, double end_v,
    double end_v2, double accel, double jerk, double *reachable_speed)
{
    double next_accel;
    double next_jerk;
    double next_max_cruise_v2;
    if (next_move)
    {
        next_accel = next_move->accel;
        next_jerk = next_move->jerk;
        next_max_cruise_v2 = next_move->max_cruise_v2;
    }
    return try_combine_with_next(next_move, next_accel, next_jerk,
        next_max_cruise_v2, distance, start_v, end_v, end_v2, accel, jerk,
        reachable_speed);
}

static bool try_combine_with_next_vmove(
    struct virtual_move *next_move, double distance, double start_v,
    double end_v, double end_v2, double accel, double jerk,
    double *reachable_speed)
{
    double next_accel;
    double next_jerk;
    double next_max_cruise_v2;
    if (next_move)
    {
        next_accel = next_move->accel;
        next_jerk = next_move->jerk;
        next_max_cruise_v2 = next_move->cruise_v * next_move->cruise_v;
    }
    return try_combine_with_next(next_move, next_accel, next_jerk,
        next_max_cruise_v2, distance, start_v, end_v, end_v2, accel, jerk,
        reachable_speed);
}

static void forward_pass(struct jerk_planner *planner)
{
    struct virtual_move *v_move = NULL;
    double current_v = planner->current_v;
    const unsigned queue_start = planner->queue->first;
    const unsigned mask = planner->queue->allocated_size - 1;
    struct move* moves = planner->queue->moves;
    const unsigned queue_size = planner->queue->size;
    for (unsigned i=0; i<queue_size; i++)
    {
        struct move *move= &moves[(queue_start + i) & mask];
        struct move *next_move;
        double end_v2;
        if (i != queue_size - 1)
        {
            next_move = &moves[(queue_start + i + 1) & mask];
            end_v2 = next_move->max_junction_v2;
        }
        else
        {
            next_move = NULL;
            end_v2 = move->max_cruise_v2;
        }
        if (v_move == NULL)
        {
            v_move = create_virtual_move(
                planner, current_v, move->accel, move->jerk);
        }
        double end_v = sqrt(end_v2);

        append_move(v_move, queue_start + i);

        v_move->distance += move->move_d;

        double reachable_end_v;
        bool can_combine = try_combine_with_next_move(
            next_move, v_move->distance, v_move->start_v, end_v, end_v2,
            v_move->accel, v_move->jerk, &reachable_end_v);

        if (!can_combine)
        {
            current_v = fmin(end_v, reachable_end_v);
            v_move->end_v = current_v;
            v_move->cruise_v =
                fmax(v_move->end_v, sqrt(move->max_cruise_v2));
            v_move = NULL;
        }
    }
}

static void backward_pass(struct jerk_planner *planner)
{
    double current_v = 0;
    for(struct virtual_move *move = planner->end_vmove-1;
        move != planner->start_vmove-1;
        --move)
    {
        struct virtual_move *prev_move = NULL;
        if (move != planner->start_vmove)
        {
            prev_move = move - 1;
        }

        if (move->end_v > current_v)
        {
            move->end_v = current_v;
        }

        double start_v = move->start_v;
        double start_v2 = start_v * start_v;

        double reachable_start_v;
        bool can_combine = try_combine_with_next_vmove(
            prev_move, move->distance, move->end_v, start_v,
            start_v2, move->accel, move->jerk, &reachable_start_v);

        if (!can_combine)
        {
            current_v = fmin(start_v, reachable_start_v);
            move->start_v = current_v;
            planner->output_vmoves[planner->num_output_vmoves++] = move;
        }
        else
        {
            prev_move->distance += move->distance;
            append_moves(prev_move, move);
        }
    }
}

static void generate_output_move(
    struct jerk_planner *planner, struct move *move, struct virtual_move *vmove,
    const unsigned queue_size, const unsigned mask,
    unsigned* move_count, unsigned* flush_count, double* distance)
{
    (*move_count)++;
    move->jerk = vmove->jerk;

    double d = *distance;
    d += move->move_d;

    move->start_v = vmove->v;
    move->start_a = vmove->a;
    for (int j=0;j<7;j++)
    {
        move->jerk_t[j] = 0.0;
    }
    double cruise_v = vmove->segment_end_v;
    bool at_end = false;
    while (d >= vmove->segment_end_x - tolerance)
    {
        unsigned s = vmove->current_segment;
        move->jerk_t[s] = vmove->move.jerk_t[s]
            - vmove->current_segment_offset;
        cruise_v = fmax(cruise_v, vmove->segment_start_v);
        if (s == 6)
        {
            at_end = true;
            break;
        }

        calculate_next_segment(vmove);
    }

    if (d < vmove->segment_end_x - tolerance)
    {
        move->jerk_t[vmove->current_segment] = move_to(vmove, d);
        move->end_v = vmove->v;
    }
    else
    {
        move->end_v = vmove->segment_end_v;
    }

    move->cruise_v = fmax(cruise_v, vmove->v);

    double target_end_v2 = move->max_cruise_v2;
    if (*move_count < queue_size)
    {
        unsigned index = (planner->queue->first + *move_count) & mask;
        target_end_v2 = planner->queue->moves[index].max_junction_v2;
    }
    // Flush when the top speed is reached, and there's no
    // acceleration (at a cruise segment, or at the end)
    if (vmove->current_segment == 3 || at_end)
    {
        if (fabs(move->end_v * move->end_v - target_end_v2) < tolerance)
        {
            *flush_count = *move_count;
        }
    }

    move->start_v = fmax(0, move->start_v);
    move->end_v = fmax(0, move->end_v);
    *distance = d;
}

static void generate_output_moves(struct jerk_planner *planner,
    struct move *moves, const unsigned queue_size, const unsigned mask,
    unsigned *move_count, unsigned *flush_count)
{
    struct virtual_move **end = 
        planner->output_vmoves + planner->num_output_vmoves - 1;
    struct virtual_move **start = planner->output_vmoves - 1;
    for(struct virtual_move **itr=end; itr != start; --itr)
    {
        struct virtual_move* vmove = *itr;

        calculate_profile(vmove);
        calculate_first_segment(vmove);

        double d = 0.0;

        for (int i=0; i<vmove->move_count;i++)
        {
            struct move *move = &moves[(vmove->start_move_index + i) & mask];
            generate_output_move(planner, move, vmove, queue_size, mask,
                move_count, flush_count, &d);
        }
    }
}

unsigned int __visible
jerk_planner_flush(struct jerk_planner *planner, bool lazy)
{
    const unsigned queue_size = planner->queue->size;
    if (queue_size == 0)
        return 0;
    planner->start_vmove = NULL;
    planner->end_vmove = NULL;
    planner->num_output_vmoves = 0;

    const unsigned mask = planner->queue->allocated_size - 1;
    struct move* moves = planner->queue->moves;

    forward_pass(planner);
    backward_pass(planner);

    unsigned flush_count = 0;
    unsigned move_count = 0;
    generate_output_moves(planner, moves, queue_size, mask, &move_count,
        &flush_count);

    if (!lazy)
    {
        flush_count = move_count;
    }
    if (flush_count > 0)
    {
        struct move *last_flushed = 
            &moves[(planner->queue->first + flush_count - 1) & mask];
        planner->current_v = last_flushed->end_v;
        move_queue_flush(planner->queue, flush_count);
    }
    return flush_count;
}