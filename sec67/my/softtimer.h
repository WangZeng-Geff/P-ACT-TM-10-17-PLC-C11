#ifndef _SOFTTIMER_H_
#define _SOFTTIMER_H_

#include "types.h"
#include "list.h"

enum soft_timer_opt
{
    TIMER_ALWAYS_ALIVE = 0x01
};

struct soft_timer
{
    struct list_head entry;

    u32 expires;
    u32 opt;
    void *data;
    void (*timer_cb)(struct soft_timer *st);
};

extern struct task soft_timer_task;

void soft_timer_add(struct soft_timer *st);
void soft_timer_del(struct soft_timer *st);
#endif
