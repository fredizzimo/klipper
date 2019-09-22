// Stepper pulse schedule compression
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The goal of this code is to take a series of scheduled stepper
// pulse times and compress them into a handful of commands that can
// be efficiently transmitted and executed on a microcontroller (mcu).
// The mcu accepts step pulse commands that take interval, count, and
// add parameters such that 'count' pulses occur, with each step event
// calculating the next step event time using:
//  next_wake_time = last_wake_time + interval; interval += add
// This code is written in C (instead of python) for processing
// efficiency - the repetitive integer math is vastly faster in C.

#include <stddef.h> // offsetof
#include <stdint.h> // uint32_t
#include <stdio.h> // fprintf
#include <stdlib.h> // malloc
#include <string.h> // memset
#include <stdbool.h>
#include <math.h> //sqrtf
#include "compiler.h" // DIV_ROUND_UP
#include "pyhelper.h" // errorf
#include "serialqueue.h" // struct queue_message
#include "stepcompress.h" // stepcompress_alloc

#define QUEUE_START_SIZE 1024

struct queuestep {
    uint32_t clock;
    uint32_t inv_speed;
};

struct stepcompress {
    // Buffer management
    struct queuestep *queue, *queue_end, *queue_pos, *queue_next;
    // Internal tracking
    uint32_t max_error;
    double mcu_time_offset, mcu_freq;
    // Message generation
    uint64_t last_step_clock;
    uint32_t last_step_speed;
    struct list_head msg_queue;
    uint32_t queue_step_msgid, queue_steps_msgid, set_next_step_dir_msgid, oid;
    int sdir, invert_sdir;
};


/****************************************************************
 * Step compression
 ****************************************************************/

struct step_move {
    int32_t add1;
    int32_t add2;
    uint16_t count;
    uint64_t end_time;
    uint32_t end_speed;
};

#if 0

static inline int32_t
idiv_up(int32_t n, int32_t d)
{
    return (n>=0) ? DIV_ROUND_UP(n,d) : (n/d);
}

static inline int32_t
idiv_down(int32_t n, int32_t d)
{
    return (n>=0) ? (n/d) : (n - d + 1) / d;
}

struct points {
    int32_t minp, maxp;
};

// Given a requested step time, return the minimum and maximum
// acceptable times
static inline struct points
minmax_point(struct stepcompress *sc, uint32_t *pos)
{
    uint32_t lsc = sc->last_step_clock, point = *pos - lsc;
    uint32_t prevpoint = pos > sc->queue_pos ? *(pos-1) - lsc : 0;
    uint32_t max_error = (point - prevpoint) / 2;
    if (max_error > sc->max_error)
        max_error = sc->max_error;
    return (struct points){ point - max_error, point };
}

// Find a 'step_move' that covers a series of step times
static struct step_move
compress_bisect_add(struct stepcompress *sc)
{
    uint32_t *qlast = sc->queue_next;
    if (qlast > sc->queue_pos + 65535)
        qlast = sc->queue_pos + 65535;
    struct points point = minmax_point(sc, sc->queue_pos);
    int32_t outer_mininterval = point.minp, outer_maxinterval = point.maxp;
    int32_t add = 0, minadd = -0x8000, maxadd = 0x7fff;
    int32_t bestinterval = 0, bestcount = 1, bestadd = 1, bestreach = INT32_MIN;
    int32_t zerointerval = 0, zerocount = 0;

    for (;;) {
        // Find longest valid sequence with the given 'add'
        struct points nextpoint;
        int32_t nextmininterval = outer_mininterval;
        int32_t nextmaxinterval = outer_maxinterval, interval = nextmaxinterval;
        int32_t nextcount = 1;
        for (;;) {
            nextcount++;
            if (&sc->queue_pos[nextcount-1] >= qlast) {
                int32_t count = nextcount - 1;
                return (struct step_move){ interval, count, add };
            }
            nextpoint = minmax_point(sc, sc->queue_pos + nextcount - 1);
            int32_t nextaddfactor = nextcount*(nextcount-1)/2;
            int32_t c = add*nextaddfactor;
            if (nextmininterval*nextcount < nextpoint.minp - c)
                nextmininterval = DIV_ROUND_UP(nextpoint.minp - c, nextcount);
            if (nextmaxinterval*nextcount > nextpoint.maxp - c)
                nextmaxinterval = (nextpoint.maxp - c) / nextcount;
            if (nextmininterval > nextmaxinterval)
                break;
            interval = nextmaxinterval;
        }

        // Check if this is the best sequence found so far
        int32_t count = nextcount - 1, addfactor = count*(count-1)/2;
        int32_t reach = add*addfactor + interval*count;
        if (reach > bestreach
            || (reach == bestreach && interval > bestinterval)) {
            bestinterval = interval;
            bestcount = count;
            bestadd = add;
            bestreach = reach;
            if (!add) {
                zerointerval = interval;
                zerocount = count;
            }
            if (count > 0x200)
                // No 'add' will improve sequence; avoid integer overflow
                break;
        }

        // Check if a greater or lesser add could extend the sequence
        int32_t nextaddfactor = nextcount*(nextcount-1)/2;
        int32_t nextreach = add*nextaddfactor + interval*nextcount;
        if (nextreach < nextpoint.minp) {
            minadd = add + 1;
            outer_maxinterval = nextmaxinterval;
        } else {
            maxadd = add - 1;
            outer_mininterval = nextmininterval;
        }

        // The maximum valid deviation between two quadratic sequences
        // can be calculated and used to further limit the add range.
        if (count > 1) {
            int32_t errdelta = sc->max_error*QUADRATIC_DEV / (count*count);
            if (minadd < add - errdelta)
                minadd = add - errdelta;
            if (maxadd > add + errdelta)
                maxadd = add + errdelta;
        }

        // See if next point would further limit the add range
        int32_t c = outer_maxinterval * nextcount;
        if (minadd*nextaddfactor < nextpoint.minp - c)
            minadd = idiv_up(nextpoint.minp - c, nextaddfactor);
        c = outer_mininterval * nextcount;
        if (maxadd*nextaddfactor > nextpoint.maxp - c)
            maxadd = idiv_down(nextpoint.maxp - c, nextaddfactor);

        // Bisect valid add range and try again with new 'add'
        if (minadd > maxadd)
            break;
        add = maxadd - (maxadd - minadd) / 4;
    }
    if (zerocount + zerocount/16 >= bestcount)
        // Prefer add=0 if it's similar to the best found sequence
        return (struct step_move){ zerointerval, zerocount, 0 };
    return (struct step_move){ bestinterval, bestcount, bestadd };
}
#endif

static inline double fixed_to_double(int64_t fixed)
{
    return (double)fixed / (double)(1 << 16);
}

static inline int64_t fixed_divide_by_integer(int64_t dividend, int64_t divisor)
{
    int64_t v = dividend << 16;
    bool signa = v >=0;
    bool signb = divisor >=0;
    // Slightly improve the precision by rounding
    if (signa == signb)
    {
        v += divisor / 2;
    }
    else
    {
        v -= divisor / 2;        
    }
    return v / divisor;
}

static inline int64_t fixed_multiply_by_integer(int32_t fixed, uint32_t integer)
{
    return ((int64_t)(fixed) * integer) / (1 << 16);
}

static struct step_move
generate_move(struct stepcompress *sc, uint16_t count)
{
    // TODO: Detect overflow
    // start_speed overflow = send a direct step command
    // end_speed overflow = reduce the search range

    uint32_t end_time = sc->queue_pos[count-1].clock;

    int32_t start_speed = sc->last_step_speed;

    // TODO: Handle timer wrap around
    uint32_t end_speed = sc->queue_pos[count-1].inv_speed;

    //errorf("start_time %u, end_time %u, start_speed %u end_speed %u",
    //    sc->last_step_clock, end_time, start_speed, end_speed);
    //errorf("count %i", count);

    int64_t a0 = sc->last_step_clock;
    int64_t a1 = start_speed;
    int64_t a2 = (int64_t)3*end_time - 3*a0 - (int64_t)count*(2*a1 + end_speed);
    int64_t a3 = 2*a0 - (int64_t)2*end_time + (int64_t)count*(a1 + end_speed);

    uint64_t count2 = (uint64_t)count*count;
    uint64_t count3 = count2*count;

    a2 = fixed_divide_by_integer(a2, count2);
    a3 = fixed_divide_by_integer(a3, count3);

    float da2 = fixed_to_double(a2);
    float da3 = fixed_to_double(a3);

    //errorf("%ld, %ld, %ld, %ld, %f, %f", a0, a1, a2, a3, da2, da3);

    int64_t a2_addfactor = (a2*2);

    if (a2_addfactor > INT32_MAX || a2_addfactor < INT32_MIN)
    {
        errorf("a2 overflow");
        return (struct step_move){};
    }
    int64_t a3_addfactor = (a3*6);
    if (a3_addfactor > INT32_MAX || a3_addfactor < INT32_MIN)
    {
        errorf("a3 overflow");
        return (struct step_move){};
    }

#if 0
    for (int i=1;i<=count;i++)
    {
        uint64_t res = a1*i + a2*i*i + a3*i*i*i;
        uint32_t res2 = a1*i + fixed_multiply_by_integer(a2,i*i) + fixed_multiply_by_integer(a3, i*i*i);
        errorf("step %i %ld %f, %u", i, res >> 16, fixed_to_double(res), res2);

    }
    errorf("%u, %u %u, %u", sc->queue_pos[0].clock, sc->queue_pos[1].clock, sc->queue_pos[2].clock, sc->queue_pos[count-1].clock);
#endif

    // We need to calculate the real end time and speed since there's some precision loss
    // Note that the speed is not the real speed, but the number of ticks between the last 
    // two steps.

    uint64_t count_prev = count - 1;
    uint64_t count2_prev = count_prev * count_prev;
    uint64_t count3_prev = count2_prev * count_prev;

    int64_t end_time2 = a2*count2 + a3*count3;
    end_time = (int32_t)(end_time2 >> 16);
    end_time += a0 + a1*count;

    int64_t end_time_prev = a2*count2_prev + a3*count3_prev;
    int32_t end_speed2 = (end_time2 - end_time_prev) >> 16;
    end_speed2 += a1; 
    end_speed = end_speed2;

    //errorf("End time %u, end_speed %u", end_time, end_speed);

    return (struct step_move){ a2, a3, count, end_time, end_speed };
}

static const float invphi = (sqrtf(5.0f) - 1.0f) / 2.0f;
static const float invphi2 = (3.0f - sqrtf(5.0f)) / 2.0f;

static uint32_t evaluate_error(struct stepcompress *sc, struct step_move *move, uint16_t pos)
{
    uint32_t start_speed = sc->last_step_speed;
    uint32_t start_time = sc->last_step_clock;
    uint64_t count = pos + 1; 
    uint64_t count2 = count*count;
    uint64_t count3 = count2*count;

    int64_t time = move->add1*count2 + move->add2*count3;
    time = (int32_t)(time >> 16);
    time += start_time + start_speed*count;
    errorf("%ld %u", time, sc->queue_pos[pos].clock);
    return abs((int)(time - sc->queue_pos[pos].clock));
}

static bool validate_move(struct stepcompress *sc, struct step_move *move)
{
    uint32_t max_error = sc->max_error;
    uint16_t count = move->count;
    uint32_t real_end_time = sc->queue_pos[count - 1].clock;
    uint32_t error = abs((int)(real_end_time - move->end_time));
    errorf("Error1 %u, max allowed %u %u", error, max_error, move->count);
    if (error > max_error)
    {
        return false;
    }
    else 
    {
        uint32_t start_speed = sc->last_step_speed;
        uint32_t start_time = sc->last_step_clock;
        uint32_t first_step_time = start_time + start_speed + ((move->add1 + move->add2) >> 16);


        error = abs((int)(first_step_time - sc->queue_pos[0].clock));
        errorf("Error2 %u, max allowed %u %u %u, %u", error, max_error, first_step_time, sc->queue_pos[0].clock, move->count);
        if (error > max_error)
        {
            return false;
        }
    }
    if (count > 1)
    {
        // Perform a golden section search for the maximum error
        float a = 0;
        float b = count - 1;
        float h = b; 

        float c = a + invphi2 * h; 
        float d = a + invphi * h; 
        errorf("a: %f b: %f c: %f d: %f h: %f invphi2: %f, invphi: %f", a, b, c, d, h, invphi2, invphi);

        uint32_t yc = evaluate_error(sc, move, (uint32_t)(c)); 
        if (yc > max_error)
        {
            errorf("yc error %u %f", yc, c);
            return false;
        }
        uint32_t yd = evaluate_error(sc, move, (uint32_t)(d)); 
        if (yd > max_error)
        {
            errorf("yd error %u %f", yd, d);
            return false;
        }
    }

    return true;
}

static struct step_move
find_move(struct stepcompress *sc)
{
    uint16_t count = sc->queue_next - sc->queue_pos;
    int32_t low = 1;
    int32_t high = count + 1; 
    struct step_move best_move = generate_move(sc, low);
    // There's an overflow when count is 0
    if (best_move.count == 0)
    {
        return best_move;
    }
    while (low < high)
    {
        int32_t mid = (low + high) / 2;
        struct step_move move = generate_move(sc, mid);
        if (validate_move(sc, &move))
        {
            best_move = move;
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    errorf("Add move %i, %u %u", best_move.count, best_move.end_time, best_move.end_speed);
    errorf("         %u %u", sc->queue_pos[best_move.count-1].clock, sc->queue_pos[best_move.count-1].inv_speed);
    return best_move;
}


/****************************************************************
 * Step compress interface
 ****************************************************************/

// Allocate a new 'stepcompress' object
struct stepcompress * __visible
stepcompress_alloc(uint32_t oid)
{
    struct stepcompress *sc = malloc(sizeof(*sc));
    memset(sc, 0, sizeof(*sc));
    list_init(&sc->msg_queue);
    sc->oid = oid;
    sc->sdir = -1;
    return sc;
}

// Fill message id information
void __visible
stepcompress_fill(struct stepcompress *sc, uint32_t max_error
                  , uint32_t invert_sdir, uint32_t queue_step_msgid
                  , uint32_t queue_steps_msgid
                  , uint32_t set_next_step_dir_msgid)
{
    sc->max_error = max_error;
    sc->invert_sdir = !!invert_sdir;
    sc->queue_step_msgid = queue_step_msgid;
    sc->queue_steps_msgid = queue_steps_msgid;
    sc->set_next_step_dir_msgid = set_next_step_dir_msgid;
}

// Free memory associated with a 'stepcompress' object
void __visible
stepcompress_free(struct stepcompress *sc)
{
    if (!sc)
        return;
    free(sc->queue);
    message_queue_free(&sc->msg_queue);
    free(sc);
}

// Convert previously scheduled steps into commands for the mcu
static int
stepcompress_flush(struct stepcompress *sc, uint64_t move_clock)
{
    if (sc->queue_pos >= sc->queue_next)
        return 0;
    while (sc->last_step_clock < move_clock) {
        struct step_move move = find_move(sc);
        if (move.count == 0)
        {
            move.count = 1;
            uint32_t interval = sc->queue_pos[0].clock - sc->last_step_clock;
            uint32_t msg[] = {
                sc->queue_step_msgid, sc->oid, interval
            };
            struct queue_message *qm = message_alloc_and_encode(msg, sizeof(msg)/sizeof(uint32_t));
            qm->min_clock = qm->req_clock = sc->last_step_clock;
            sc->last_step_clock += interval;
            if (interval > UINT16_MAX)
            {
                interval = UINT16_MAX;
            }
            sc->last_step_speed = interval;
            list_add_tail(&qm->node, &sc->msg_queue);
        }
        else
        {
            uint32_t msg[] = {
                sc->queue_steps_msgid, sc->oid, move.count, move.add1, move.add2
            };
            struct queue_message *qm = message_alloc_and_encode(msg, sizeof(msg)/sizeof(uint32_t));
            qm->min_clock = qm->req_clock = sc->last_step_clock;
            sc->last_step_clock = move.end_time;
            sc->last_step_speed = move.end_speed;
            list_add_tail(&qm->node, &sc->msg_queue);

        }
        if (sc->queue_pos + move.count >= sc->queue_next) {
            sc->queue_pos = sc->queue_next = sc->queue;
            break;
        }
        sc->queue_pos += move.count;
    }
    return 0;
}

// Generate a queue_step for a step far in the future from the last step
// TODO: Fix and test this
static int
stepcompress_flush_far(struct stepcompress *sc, uint64_t abs_step_clock)
{
    uint32_t msg[] = {
        sc->queue_step_msgid, sc->oid, abs_step_clock - sc->last_step_clock
    };
    struct queue_message *qm = message_alloc_and_encode(msg, sizeof(msg)/sizeof(uint32_t));
    qm->min_clock = sc->last_step_clock;
    sc->last_step_clock = qm->req_clock = abs_step_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}

// Send the set_next_step_dir command
static int
set_next_step_dir(struct stepcompress *sc, int sdir)
{
    if (sc->sdir == sdir)
        return 0;
    sc->sdir = sdir;
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    uint32_t msg[3] = {
        sc->set_next_step_dir_msgid, sc->oid, sdir ^ sc->invert_sdir
    };
    struct queue_message *qm = message_alloc_and_encode(msg, 3);
    qm->req_clock = sc->last_step_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}

// Reset the internal state of the stepcompress object
int __visible
stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    sc->last_step_clock = last_step_clock;
    sc->last_step_speed = 0;
    sc->sdir = -1;
    return 0;
}

// Queue an mcu command to go out in order with stepper commands
int __visible
stepcompress_queue_msg(struct stepcompress *sc, uint32_t *data, int len)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;

    struct queue_message *qm = message_alloc_and_encode(data, len);
    qm->req_clock = sc->last_step_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}

// Set the conversion rate of 'print_time' to mcu clock
static void
stepcompress_set_time(struct stepcompress *sc
                      , double time_offset, double mcu_freq)
{
    sc->mcu_time_offset = time_offset;
    sc->mcu_freq = mcu_freq;
}

double
stepcompress_get_mcu_freq(struct stepcompress *sc)
{
    return sc->mcu_freq;
}

uint32_t
stepcompress_get_oid(struct stepcompress *sc)
{
    return sc->oid;
}

int
stepcompress_get_step_dir(struct stepcompress *sc)
{
    return sc->sdir;
}


/****************************************************************
 * Queue management
 ****************************************************************/

// Maximium clock delta between messages in the queue
#define CLOCK_DIFF_MAX (3<<28)

// Create a cursor for inserting clock times into the queue
struct queue_append __visible
queue_append_start(struct stepcompress *sc, double print_time, double adjust)
{
    double print_clock = (print_time - sc->mcu_time_offset) * sc->mcu_freq;
    return (struct queue_append) {
        .sc = sc, .qnext = sc->queue_next, .qend = sc->queue_end,
        .last_step_clock_32 = sc->last_step_clock,
        .clock_offset = (print_clock - (double)sc->last_step_clock) + adjust };
}

// Finalize a cursor created with queue_append_start()
void __visible
queue_append_finish(struct queue_append qa)
{
    qa.sc->queue_next = qa.qnext;
}

// Slow path for queue_append()
static int
queue_append_slow(struct stepcompress *sc, double rel_sc, uint32_t inv_speed)
{
    uint64_t abs_step_clock = (uint64_t)rel_sc + sc->last_step_clock;
    if (abs_step_clock >= sc->last_step_clock + CLOCK_DIFF_MAX) {
        // Avoid integer overflow on steps far in the future
        int ret = stepcompress_flush(sc, abs_step_clock - CLOCK_DIFF_MAX + 1);
        if (ret)
            return ret;

        if (abs_step_clock >= sc->last_step_clock + CLOCK_DIFF_MAX)
            return stepcompress_flush_far(sc, abs_step_clock);
    }

    if (sc->queue_next - sc->queue_pos > 65535 + 2000) {
        // No point in keeping more than 64K steps in memory
        uint32_t flush = ((sc->queue_next-65535)->clock
                          - (uint32_t)sc->last_step_clock);
        int ret = stepcompress_flush(sc, sc->last_step_clock + flush);
        if (ret)
            return ret;
    }

    if (sc->queue_next >= sc->queue_end) {
        // Make room in the queue
        int in_use = sc->queue_next - sc->queue_pos;
        if (sc->queue_pos > sc->queue) {
            // Shuffle the internal queue to avoid having to allocate more ram
            memmove(sc->queue, sc->queue_pos, in_use * sizeof(*sc->queue));
        } else {
            // Expand the internal queue of step times
            int alloc = sc->queue_end - sc->queue;
            if (!alloc)
                alloc = QUEUE_START_SIZE;
            while (in_use >= alloc)
                alloc *= 2;
            sc->queue = realloc(sc->queue, alloc * sizeof(*sc->queue));
            sc->queue_end = sc->queue + alloc;
        }
        sc->queue_pos = sc->queue;
        sc->queue_next = sc->queue + in_use;
    }

    *sc->queue_next++ = (struct queuestep){abs_step_clock, inv_speed};
    return 0;
}

// Add a clock time to the queue (flushing the queue if needed)
int __visible
queue_append(struct queue_append *qa, double step_clock, uint32_t speed)
{
    double rel_sc = step_clock + qa->clock_offset;
    if (likely(!(qa->qnext >= qa->qend || rel_sc >= (double)CLOCK_DIFF_MAX))) {
        *qa->qnext++ = (struct queuestep){qa->last_step_clock_32 + (uint32_t)rel_sc, speed};
        return 0;
    }
    // Call queue_append_slow() to handle queue expansion and integer overflow
    struct stepcompress *sc = qa->sc;
    uint64_t old_last_step_clock = sc->last_step_clock;
    sc->queue_next = qa->qnext;
    int ret = queue_append_slow(sc, rel_sc, speed);
    if (ret)
        return ret;
    qa->qnext = sc->queue_next;
    qa->qend = sc->queue_end;
    qa->last_step_clock_32 = sc->last_step_clock;
    qa->clock_offset -= sc->last_step_clock - old_last_step_clock;
    return 0;
}

inline int
queue_append_set_next_step_dir(struct queue_append *qa, int sdir)
{
    struct stepcompress *sc = qa->sc;
    uint64_t old_last_step_clock = sc->last_step_clock;
    sc->queue_next = qa->qnext;
    int ret = set_next_step_dir(sc, sdir);
    if (ret)
        return ret;
    qa->qnext = sc->queue_next;
    qa->qend = sc->queue_end;
    qa->last_step_clock_32 = sc->last_step_clock;
    qa->clock_offset -= sc->last_step_clock - old_last_step_clock;
    return 0;
}


/****************************************************************
 * Step compress synchronization
 ****************************************************************/

// The steppersync object is used to synchronize the output of mcu
// step commands.  The mcu can only queue a limited number of step
// commands - this code tracks when items on the mcu step queue become
// free so that new commands can be transmitted.  It also ensures the
// mcu step queue is ordered between steppers so that no stepper
// starves the other steppers of space in the mcu step queue.

struct steppersync {
    // Serial port
    struct serialqueue *sq;
    struct command_queue *cq;
    // Storage for associated stepcompress objects
    struct stepcompress **sc_list;
    int sc_num;
    // Storage for list of pending move clocks
    uint64_t *move_clocks;
    int num_move_clocks;
};

// Allocate a new 'steppersync' object
struct steppersync * __visible
steppersync_alloc(struct serialqueue *sq, struct stepcompress **sc_list
                  , int sc_num, int move_num)
{
    struct steppersync *ss = malloc(sizeof(*ss));
    memset(ss, 0, sizeof(*ss));
    ss->sq = sq;
    ss->cq = serialqueue_alloc_commandqueue();

    ss->sc_list = malloc(sizeof(*sc_list)*sc_num);
    memcpy(ss->sc_list, sc_list, sizeof(*sc_list)*sc_num);
    ss->sc_num = sc_num;

    ss->move_clocks = malloc(sizeof(*ss->move_clocks)*move_num);
    memset(ss->move_clocks, 0, sizeof(*ss->move_clocks)*move_num);
    ss->num_move_clocks = move_num;

    return ss;
}

// Free memory associated with a 'steppersync' object
void __visible
steppersync_free(struct steppersync *ss)
{
    if (!ss)
        return;
    free(ss->sc_list);
    free(ss->move_clocks);
    serialqueue_free_commandqueue(ss->cq);
    free(ss);
}

// Set the conversion rate of 'print_time' to mcu clock
void __visible
steppersync_set_time(struct steppersync *ss, double time_offset
                     , double mcu_freq)
{
    int i;
    for (i=0; i<ss->sc_num; i++) {
        struct stepcompress *sc = ss->sc_list[i];
        stepcompress_set_time(sc, time_offset, mcu_freq);
    }
}

// Implement a binary heap algorithm to track when the next available
// 'struct move' in the mcu will be available
static void
heap_replace(struct steppersync *ss, uint64_t req_clock)
{
    uint64_t *mc = ss->move_clocks;
    int nmc = ss->num_move_clocks, pos = 0;
    for (;;) {
        int child1_pos = 2*pos+1, child2_pos = 2*pos+2;
        uint64_t child2_clock = child2_pos < nmc ? mc[child2_pos] : UINT64_MAX;
        uint64_t child1_clock = child1_pos < nmc ? mc[child1_pos] : UINT64_MAX;
        if (req_clock <= child1_clock && req_clock <= child2_clock) {
            mc[pos] = req_clock;
            break;
        }
        if (child1_clock < child2_clock) {
            mc[pos] = child1_clock;
            pos = child1_pos;
        } else {
            mc[pos] = child2_clock;
            pos = child2_pos;
        }
    }
}

// Find and transmit any scheduled steps prior to the given 'move_clock'
int __visible
steppersync_flush(struct steppersync *ss, uint64_t move_clock)
{
    // Flush each stepcompress to the specified move_clock
    int i;
    for (i=0; i<ss->sc_num; i++) {
        int ret = stepcompress_flush(ss->sc_list[i], move_clock);
        if (ret)
            return ret;
    }

    // Order commands by the reqclock of each pending command
    struct list_head msgs;
    list_init(&msgs);
    for (;;) {
        // Find message with lowest reqclock
        uint64_t req_clock = MAX_CLOCK;
        struct queue_message *qm = NULL;
        for (i=0; i<ss->sc_num; i++) {
            struct stepcompress *sc = ss->sc_list[i];
            if (!list_empty(&sc->msg_queue)) {
                struct queue_message *m = list_first_entry(
                    &sc->msg_queue, struct queue_message, node);
                if (m->req_clock < req_clock) {
                    qm = m;
                    req_clock = m->req_clock;
                }
            }
        }
        if (!qm || (qm->min_clock && req_clock > move_clock))
            break;

        uint64_t next_avail = ss->move_clocks[0];
        if (qm->min_clock)
            // The qm->min_clock field is overloaded to indicate that
            // the command uses the 'move queue' and to store the time
            // that move queue item becomes available.
            heap_replace(ss, qm->min_clock);
        // Reset the min_clock to its normal meaning (minimum transmit time)
        qm->min_clock = next_avail;

        // Batch this command
        list_del(&qm->node);
        list_add_tail(&qm->node, &msgs);
    }

    // Transmit commands
    if (!list_empty(&msgs))
        serialqueue_send_batch(ss->sq, ss->cq, &msgs);
    return 0;
}
