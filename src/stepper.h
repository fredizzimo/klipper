#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

uint_fast8_t stepper_event(struct timer *t);
struct stepper *stepper_oid_lookup(uint8_t oid);
void stepper_stop(struct stepper *s);
#if CONFIG_HAVE_SMOOTH_STOP
void stepper_smooth_stop(struct stepper *s);
#else
#define stepper_smooth_stop stepper_stop
#endif

#endif // stepper.h
