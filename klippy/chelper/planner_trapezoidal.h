// Feedrate planner for moves with a trapezoidal speed profile
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#pragma once
#include <stdbool.h>

struct trapezoidal_planner;
struct move_queue;

struct trapezoidal_planner*
trapezoidal_planner_alloc(struct move_queue *queue);
void trapezoidal_planner_free(struct trapezoidal_planner *planner);
void trapezoidal_planner_reset(struct trapezoidal_planner *planner);
// Returns the number of moves flushed
unsigned int trapezoidal_planner_flush(struct trapezoidal_planner *planner,
    bool lazy);