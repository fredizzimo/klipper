// The C side implementation of the Move class
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#pragma once
#include <stdbool.h>

struct move {
    double start_pos[4];
    double end_pos[4];
    double axes_d[4];
    double axes_r[4];
    double move_d;
    bool is_kinematic_move;
    // TODO determine if the following three are needed
    double start_v;
    double cruise_v;
    double end_v;
    double start_a;
    double accel_t;
    double cruise_t;
    double decel_t;
    double jerk_t[7];
    double max_junction_v2;
    double max_start_v2;
    double max_smoothed_v2;
    double accel;
    double jerk;
    double max_cruise_v2;
    double delta_v2;
    double smooth_delta_v2;
    double min_move_t;
};

struct move_queue {
    struct move *moves;
    unsigned int allocated_size;
    unsigned int first;
    unsigned int size;
};

struct move_queue* move_queue_alloc(unsigned int num_moves);
void move_queue_free(struct move_queue *queue);
void move_queue_flush(struct move_queue *queue, unsigned int count);
void move_queue_reset(struct move_queue *queue);
struct move* move_reserve(
    double *start_pos,
    double *end_pos,
    double speed,
    double accel,
    double accel_to_decel,
    double jerk,
    struct move_queue* q);
void move_init(
    struct move *m,
    double *start_pos,
    double *end_pos,
    double speed,
    double accel,
    double accel_to_decel,
    double jerk);
void move_commit(struct move_queue *queue);
void move_limit_speed(struct move *m, double speed, double accel,
    double max_accel_to_decel);
void move_calc_junction(struct move *m, struct move *prev_move,
    double junction_deviation, double extruder_instant_v);
void move_set_trapezoidal_times(struct move *m, double distance,
    double start_v2, double cruise_v2, double end_v2, double accel);
void move_calculate_trapezoidal(struct move* m, double start_v, double end_v);
void move_calculate_jerk(struct move* m, double start_v, double end_v);
double move_get_max_allowed_jerk_end_speed(double distance, double start_v,
    double end_v, double max_a, double jerk);
bool move_can_accelerate_fully(double distance, double start_v, double end_v,
    double accel, double jerk);