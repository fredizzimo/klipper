// Handling of stepper drivers.
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // command_config_stepper
#include "math.h"
#include "stdbool.h"

DECL_CONSTANT("STEP_DELAY", CONFIG_STEP_DELAY);


/****************************************************************
 * Steppers
 ****************************************************************/

struct stepper_move {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0 };

struct stepper {
    struct timer time;
    uint32_t interval;
    int16_t add;
#if CONFIG_STEP_DELAY <= 0
    uint_fast16_t count;
#define next_step_time time.waketime
#else
    uint32_t count;
    uint32_t next_step_time;
#endif
    struct gpio_out step_pin, dir_pin;
    uint32_t position;
    struct move_queue_head mq;
    uint32_t min_stop_interval;

#if CONFIG_HAVE_SMOOTH_STOP
    float smooth_a_inv;
    float smooth_two_a;
    float smooth_two_a_inv;
    uint16_t smooth_steps;
    uint16_t smooth_deceleration;
    uint16_t smooth_acceleration;
    int32_t smooth_c;
    void (*smooth_callback)(struct endstop*);
    struct endstop *smooth_endstop;
    int16_t smooth_divisor;
#endif

    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_HAVE_ADD=1<<3,
    SF_LAST_RESET=1<<4, SF_NO_NEXT_CHECK=1<<5, SF_NEED_RESET=1<<6
};

static void schedule_move(struct stepper *s, struct stepper_move *m,
    uint32_t min_next_time)
{
    s->next_step_time += m->interval;
    s->add = m->add;
    s->interval = m->interval + m->add;
    if (CONFIG_STEP_DELAY <= 0) {
        if (CONFIG_MACH_AVR)
            // On AVR see if the add can be optimized away
            s->flags = m->add ? s->flags|SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
        s->count = m->count;
    } else {
        // On faster mcus, it is necessary to schedule unstep events
        // and so there are twice as many events.  Also check that the
        // next step event isn't too close to the last unstep.
        if (unlikely(timer_is_before(s->next_step_time, min_next_time))) {
            if ((int32_t)(s->next_step_time - min_next_time)
                < (int32_t)(-timer_from_us(1000)))
                shutdown("Stepper too far in past");
            s->time.waketime = min_next_time;
        } else {
            s->time.waketime = s->next_step_time;
        }
        s->count = (uint32_t)m->count * 2;
    }
    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    if (m->flags & MF_DIR) {
        s->position = -s->position + m->count;
        gpio_out_toggle_noirq(s->dir_pin);
    } else {
        s->position += m->count;
    }
}

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s, uint32_t min_next_time)
{
    if (move_queue_empty(&s->mq)) {
#if CONFIG_HAVE_SMOOTH_STOP
        if (s->smooth_steps > 0) {
            int32_t c = s->smooth_c;
            int16_t divisor = s->smooth_divisor;
            uint8_t flags = 0;

            uint16_t steps = --s->smooth_steps;
            if (steps == s->smooth_deceleration) {
                // Switch acceleration direction
                divisor = 6 - divisor;
                // Switch move direction
                flags = MF_DIR;
            } else if (steps == s->smooth_acceleration) {
                // Switch acceleration direction
                divisor = 6 - divisor;
            }
            struct stepper_move move = {
                .interval = c,
                .add = 0,
                .count = 1,
                .flags = flags
            };

            schedule_move(s, &move, min_next_time);
            c = c - (2 * c) / divisor;
            divisor += 4;
            s->smooth_c = c;
            s->smooth_divisor = divisor;
            s->smooth_steps = steps;
            return SF_RESCHEDULE;
        } else if (s->smooth_callback) {
            // This smooth stop is coming to an end
            (*s->smooth_callback)(s->smooth_endstop);
            s->smooth_callback = NULL;
            stepper_stop(s);
        }
#endif

        // There is no next move - the queue is empty
        if (s->interval - s->add < s->min_stop_interval
            && !(s->flags & SF_NO_NEXT_CHECK))
            shutdown("No next step");
        s->count = 0;
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move *m = container_of(mn, struct stepper_move, node);
    schedule_move(s, m, min_next_time);
    move_free(m);
    return SF_RESCHEDULE;
}

// AVR optimized step function
static uint_fast8_t
stepper_event_avr(struct stepper *s)
{
    gpio_out_toggle_noirq(s->step_pin);
    uint_fast16_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s, 0);
    gpio_out_toggle_noirq(s->step_pin);
    return ret;
}

// Optimized step function for stepping and unstepping in same function
static uint_fast8_t
stepper_event_nodelay(struct stepper *s)
{
    gpio_out_toggle_noirq(s->step_pin);
    uint_fast16_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
        gpio_out_toggle_noirq(s->step_pin);
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s, 0);
    gpio_out_toggle_noirq(s->step_pin);
    return ret;
}

// Timer callback - step the given stepper.
uint_fast8_t
stepper_event(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    if (CONFIG_STEP_DELAY <= 0 && CONFIG_MACH_AVR)
        return stepper_event_avr(s);
    if (CONFIG_STEP_DELAY <= 0)
        return stepper_event_nodelay(s);

    // Normal step code - schedule the unstep event
    if (!CONFIG_HAVE_STRICT_TIMING)
        gpio_out_toggle_noirq(s->step_pin);
    uint32_t step_delay = timer_from_us(CONFIG_STEP_DELAY);
    uint32_t min_next_time = timer_read_time() + step_delay;
    if (CONFIG_HAVE_STRICT_TIMING)
        // Toggling gpio after reading the time is a micro-optimization
        gpio_out_toggle_noirq(s->step_pin);
    s->count--;
    if (likely(s->count & 1))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(s->count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    return stepper_load_next(s, min_next_time);
reschedule_min:
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

void
command_config_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    if (!CONFIG_INLINE_STEPPER_HACK)
        s->time.func = stepper_event;
    s->flags = args[4] ? SF_INVERT_STEP : 0;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->min_stop_interval = args[3];
    s->position = -POSITION_BIAS;
    move_queue_setup(&s->mq, sizeof(struct stepper_move));
}
DECL_COMMAND(command_config_stepper,
             "config_stepper oid=%c step_pin=%c dir_pin=%c"
             " min_stop_interval=%u invert_step=%c");

#if CONFIG_HAVE_SMOOTH_STOP
#endif

// Return the 'struct stepper' for a given stepper oid
struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;

    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    flags &= ~SF_NO_NEXT_CHECK;
    if (m->count == 1 && (m->flags || flags & SF_LAST_RESET))
        // count=1 moves after a reset or dir change can have small intervals
        flags |= SF_NO_NEXT_CHECK;
    flags &= ~SF_LAST_RESET;
    if (flags & SF_NEED_RESET) {
        move_free(m);
    }
    else if (s->count) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        stepper_load_next(s, s->next_step_time + m->interval);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = waketime;
    s->flags = (s->flags & ~SF_NEED_RESET) | SF_LAST_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (CONFIG_STEP_DELAY <= 0)
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Stop all moves for a given stepper (used in end stop homing).  IRQs
// must be off.
void
stepper_stop(struct stepper *s)
{
    sched_del_timer(&s->time);
    s->next_step_time = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = (s->flags & SF_INVERT_STEP) | SF_NEED_RESET;
#if CONFIG_HAVE_SMOOTH_STOP
    s->smooth_steps = 0;
    s->smooth_callback = NULL;
#endif
    gpio_out_write(s->dir_pin, 0);
    gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        move_free(m);
    }
}

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        move_queue_clear(&s->mq);
        stepper_stop(s);
    }
}
DECL_SHUTDOWN(stepper_shutdown);

#if CONFIG_HAVE_SMOOTH_STOP
void
command_config_stepper_smooth_stop(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t accel = args[1];
    if (accel >0) {
        const float a_inv = 1.0f / accel;
        const float two_a = 2.0f * accel;
        const float two_a_inv = 1.0f / two_a;
        s->smooth_a_inv = a_inv;
        s->smooth_two_a = two_a;
        s->smooth_two_a_inv = two_a_inv;
    } else {
        s->smooth_a_inv = 0.0f;
        s->smooth_two_a = 0.0f;
        s->smooth_two_a_inv = 0.0f;
    }
}
DECL_COMMAND(command_config_stepper_smooth_stop,
             "config_stepper_smooth_stop oid=%c accel=%u");

// Stop all moves for a given stepper (used in end stop homing).  IRQs
// must be off.
void
stepper_smooth_stop(
    struct stepper *s, void (*cb)(struct endstop*), struct endstop *e)
{
    const float a_inv = s->smooth_a_inv;
    uint16_t num_steps = 0;
    const uint16_t stepper_count = s->count;
    const uint16_t stepper_steps = CONFIG_STEP_DELAY > 0 ?
      stepper_count / 2 : stepper_count;
    const bool need_unstep = CONFIG_STEP_DELAY > 0 && (stepper_count & 1) ?
        true : false;
    uint16_t num_extra_steps = CONFIG_STEP_DELAY > 0 && need_unstep ? 0 : 1;
    uint32_t interval = 0;

    // Only run when an acceleration is configured
    // and there's at least one more step left in
    // the scheduled moves
    if (a_inv > 0.0f &&
     (!move_queue_empty(&s->mq) || stepper_steps > 1)){
        const float two_a = s->smooth_two_a;
        const float two_a_inv = s->smooth_two_a_inv;
        const float freq = (float)(CONFIG_CLOCK_FREQ);

        interval = s->interval - s->add;

        const float v = freq / interval;
        const float v2 = v*v;

        const float radicand = v2 - two_a;

        if (radicand > 0) {
            const float t_step_0 = (v - sqrtf(radicand)) * a_inv;

            float v2_two_a_inv = v2 * two_a_inv;

            // Use integer math, so that everything is deterministic
            num_steps = floorf(v2_two_a_inv);
            // We need an even number of steps
            if ((num_steps + num_extra_steps) & 1)
                num_extra_steps++;
            const int32_t c = roundf(t_step_0 * freq);
            const int16_t divisor = -4 * (int16_t)num_steps + 5;

            const uint16_t num_deceleration_steps = num_steps;
            const uint16_t num_acceleration_steps =
                (num_steps + num_extra_steps) / 2;

            uint16_t steps =
                num_deceleration_steps + 2 * num_acceleration_steps;
            s->smooth_steps = steps;
            steps -= num_deceleration_steps;
            s->smooth_deceleration = steps;
            steps -= num_acceleration_steps;
            s->smooth_acceleration = steps;

            s->smooth_c = c;
            s->smooth_divisor = divisor;
        }
    }
    if (num_steps > 0) {
        while (!move_queue_empty(&s->mq)) {
            struct move_node *mn = move_queue_pop(&s->mq);
            struct stepper_move *m =
                container_of(mn, struct stepper_move, node);
            move_free(m);
        }
        s->flags = (s->flags & SF_INVERT_STEP) | SF_NEED_RESET;
        s->position = (stepper_get_position(s) + num_extra_steps ) |
            (s->position & 0x80000000);

        s->smooth_callback = cb;
        s->smooth_endstop = e;

        // Let the next scheduled step run normally, so that we don't need
        // to change the timers. Also run any extra steps with the same speed
        s->add = 0;
        s->interval = interval;
        if (CONFIG_STEP_DELAY > 0) {
            if (need_unstep) {
                s->count = 1 + 2 * num_extra_steps;
            } else {
                // Otherwise do both the step and unstep
                s->count = 2 * num_extra_steps;
            }
        }
        else {
            s->count = num_extra_steps;
        }
    } else {
        // Fallback to default stepper stop
        s->smooth_steps = 0;
        stepper_stop(s);
        cb(e);
    }
}
#endif
