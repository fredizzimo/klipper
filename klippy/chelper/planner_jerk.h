// Feedrate planner for moves with a 7 segment jerk limited profile
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#pragma once
#include <stdbool.h>

struct jerk_planner;
struct move_queue;

struct jerk_planner*
jerk_planner_alloc(struct move_queue *queue);
void jerk_planner_reset(struct jerk_planner * planner);
void jerk_planner_free(struct jerk_planner *planner);
// Returns the number of moves flushed
unsigned int jerk_planner_flush(struct jerk_planner *planner,
    bool lazy);
