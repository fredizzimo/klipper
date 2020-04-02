#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node

struct coord {
    union {
        struct {
            double x, y, z;
        };
        double axis[3];
    };
};

struct trapq_move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;

    struct list_node node;
};

struct trapq {
    struct list_head moves;
};

struct move;

struct trapq_move *trapq_move_alloc(void);
void trapq_append(struct trapq *tq, double print_time
                  , double accel_t, double cruise_t, double decel_t
                  , double start_pos_x, double start_pos_y, double start_pos_z
                  , double axes_r_x, double axes_r_y, double axes_r_z
                  , double start_v, double cruise_v, double accel);
void trapq_append_move(struct trapq *tq, double print_time, struct move *m);
double move_get_distance(struct trapq_move *m, double move_time);
struct coord move_get_coord(struct trapq_move *m, double move_time);
struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_check_sentinels(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct trapq_move *m);
void trapq_free_moves(struct trapq *tq, double print_time);

#endif // trapq.h
