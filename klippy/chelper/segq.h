#ifndef SEGQ_H
#define SEGQ_H

#include "list.h" // list_node

struct coord {
    union {
        struct {
            double x, y, z;
        };
        double axis[3];
    };
};

struct segq_move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;

    struct list_node node;
};

struct segq {
    struct list_head moves;
};

struct move;

struct segq_move *segq_move_alloc(void);
void segq_append(struct segq *tq, double print_time
                  , double accel_t, double cruise_t, double decel_t
                  , double start_pos_x, double start_pos_y, double start_pos_z
                  , double axes_r_x, double axes_r_y, double axes_r_z
                  , double start_v, double cruise_v, double accel);
void segq_append_move(struct segq *tq, double print_time, struct move *m);
void segq_append_extrude_move(struct segq *tq, double print_time,
    struct move *m);
double move_get_distance(struct segq_move *m, double move_time);
struct coord move_get_coord(struct segq_move *m, double move_time);
struct segq *segq_alloc(void);
void segq_free(struct segq *tq);
void segq_check_sentinels(struct segq *tq);
void segq_add_move(struct segq *tq, struct segq_move *m);
void segq_free_moves(struct segq *tq, double print_time);

#endif // SEGQ_H
