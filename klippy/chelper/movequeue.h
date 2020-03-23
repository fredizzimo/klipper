// Move queue
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#pragma once

struct move {
    double move_d;
};

struct move_queue {
    struct move *moves;
    unsigned int num_moves;
    unsigned int next_free;
};

struct move_queue* move_queue_alloc(unsigned int num_moves);
void move_queue_free(struct move_queue *queue);
struct move* move_alloc(struct move_queue *queue);