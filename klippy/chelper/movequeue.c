// Move queue
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "movequeue.h"
#include "compiler.h"
#include <stdlib.h>

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

struct move* __visible
move_alloc(struct move_queue* q)
{
    struct move *m = q->moves + (q->next_free % q->num_moves); 
    ++q->next_free;
    return m;
}