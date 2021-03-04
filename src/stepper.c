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

// There's not enough program memory on the PRU to support this
#if CONFIG_MACH_PRU
#define CONFIG_HAVE_SMOOTH_STOP 0
#else
#define CONFIG_HAVE_SMOOTH_STOP 1
#endif

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

struct decel_segment {
    uint32_t interval;
    int16_t add;
    uint16_t count;
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
    struct decel_segment *decel_segments;
    uint8_t num_decel_segments;
    uint32_t stop_position;
    uint32_t stop_delay;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_HAVE_ADD=1<<3,
    SF_LAST_RESET=1<<4, SF_NO_NEXT_CHECK=1<<5, SF_NEED_RESET=1<<6,
    SF_NEED_STOP=1<<7
};

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s, uint32_t min_next_time)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
        if (s->interval - s->add < s->min_stop_interval
            && !(s->flags & SF_NO_NEXT_CHECK))
            shutdown("No next step");
        s->count = 0;
        if (s->flags & SF_NEED_STOP) {
            stepper_stop(s);
        }
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move *m = container_of(mn, struct stepper_move, node);
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
    const uint8_t num_decel_segments = args[5];
    if (num_decel_segments) {
        // 255 is reserved
        if (num_decel_segments >= 255) {
            shutdown("num_decel_segements needs to be less than 255");
        }
        s->decel_segments = alloc_chunk(sizeof(struct decel_segment) *
             num_decel_segments);
        s->num_decel_segments = num_decel_segments;
    } else {
        s->decel_segments = NULL;
        s->num_decel_segments = 0;
    }

    move_queue_setup(&s->mq, sizeof(struct stepper_move));
}
DECL_COMMAND(command_config_stepper,
             "config_stepper oid=%c step_pin=%c dir_pin=%c"
             " min_stop_interval=%u invert_step=%c"
             " num_decel_segments=%c");


// Return the 'struct stepper' for a given stepper oid
struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

void command_set_decel_segment(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t segment_nr = args[1];
    if (segment_nr < s->num_decel_segments) {
        struct decel_segment *segment = s->decel_segments + segment_nr;
        segment->interval = args[2];
        segment->count = args[3];
        segment->add = args[4];
    } else {
        shutdown("Invalid decel segment specified");
    }
}
DECL_COMMAND(command_set_decel_segment,
             "set_decel_segment oid=%c segement=%c interval=%u count=%hu"
             " add=%hi");

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
    } else if (s->count) {
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

#if CONFIG_HAVE_SMOOTH_STOP
void
command_stepper_get_stop_info(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    uint32_t stop_position = s->stop_position;
    uint32_t stop_delay = s->stop_delay;
    irq_enable();
    position -= POSITION_BIAS;
    stop_position -= POSITION_BIAS;
    sendf("stepper_stop_info oid=%c pos=%i stop_pos=%i stop_delay=%i", oid,
        position, stop_position, stop_delay);
}
DECL_COMMAND(command_stepper_get_stop_info, "stepper_get_stop_info oid=%c");
#endif

// Stop all moves for a given stepper (used for emergency stop).  IRQs
// must be off.
void
stepper_stop(struct stepper *s)
{
    sched_del_timer(&s->time);
    s->next_step_time = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = (s->flags & SF_INVERT_STEP) | SF_NEED_RESET;
    gpio_out_write(s->dir_pin, 0);
    gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        move_free(m);
    }
}

#if CONFIG_HAVE_SMOOTH_STOP
// Stop all moves for a given stepper (used in end stop homing).
// With deceleration if configured. IRQs must be off.
void
stepper_stop_smooth(struct stepper *s)
{
    if (s->num_decel_segments > 0) {
        uint32_t position = stepper_get_position(s);
        uint32_t interval = s->interval;
        s->stop_position = position;
        struct decel_segment *segment = s->decel_segments;
        struct decel_segment *segments_end = s->decel_segments +
            s->num_decel_segments;
        // Try to find the first segement with matching intervals
        while (segment->interval > interval && segment != segments_end) {
            segment++;
        }

        // Was a valid segment found?
        if (segment != segments_end) {
            // Save the waketime before it's it's deleted
            uint32_t waketime = s->time.waketime;
            sched_del_timer(&s->time);
            s->flags =
                (s->flags & SF_INVERT_STEP) | SF_NEED_RESET | SF_NEED_STOP;

            while (!move_queue_empty(&s->mq)) {
                struct move_node *mn = move_queue_pop(&s->mq);
                struct stepper_move *m =
                    container_of(mn, struct stepper_move, node);
                move_free(m);
            }

            uint16_t count = (segment->interval - interval) / segment->add;
            count += 1;
            struct stepper_move *m = move_alloc();
            m->flags = 0;
            m->interval = interval;
            m->count = count;
            m->add = segment->add;
            move_queue_push(&m->node, &s->mq);

            // Add the rest of the segments
            segment++;
            while (segment != segments_end) {
                struct stepper_move *m = move_alloc();
                m->flags = 0;
                m->interval = segment->interval;
                m->count = segment->count;
                m->add = segment->add;
                count += segment->count;
                interval += segment->interval;
                move_queue_push(&m->node, &s->mq);
            }
            s->stop_delay = interval;

            // Check if we need to unstep first
            if (CONFIG_STEP_DELAY > 0 && s->count && (s->count & 1) == 0) {
                uint32_t time = timer_read_time();
                // Normally the code above should already delay enough
                // but in case not, just busy sleep a bit more
                // We could also schedule a timer, but that gets complex
                // since this is already called from another timer, and
                // it is probably an overkill
                while (timer_is_before(time, waketime)) {
                   time = timer_read_time();
                }
                gpio_out_toggle_noirq(s->step_pin);
                s->count--;
            }
            // If we are in the middle of a move, then fix the next step time
            if (s->count)
            {
                s->next_step_time -= s->interval;
            }

            uint32_t step_delay = timer_from_us(CONFIG_STEP_DELAY);
            uint32_t min_next_time = timer_read_time() + step_delay;

            stepper_load_next(s, min_next_time);
            sched_add_timer(&s->time);
        }

    }
    // Fall back to normal stop in case no acceleration is needed
    stepper_stop(s);
}
#else
    void stepper_stop_smooth(struct stepper *s)
    {
        stepper_stop(s);
    }
#endif

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
