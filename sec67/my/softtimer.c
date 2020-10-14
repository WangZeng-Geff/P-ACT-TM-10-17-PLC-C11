#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "comfunc.h"
#include "list.h"
#include "task.h"
#include "sec_tick.h"
#include "softtimer.h"

static LIST_HEAD(timer_list);

void soft_timer_add(struct soft_timer *st) 
{
    list_add_tail(&st->entry, &timer_list);
}

void soft_timer_del(struct soft_timer *st)
{
    if (st->entry.next && st->entry.prev) 
    {
        list_del(&st->entry);
    }
}

static void soft_timer_tack_callback(struct task *t)
{
    struct soft_timer *st, *n;

    list_for_each_entry_safe(st, n, &timer_list, entry)
    {
        if (time_after_eq(sec_jiffies, st->expires))
        {
        	if(st->opt != TIMER_ALWAYS_ALIVE)
        	{
        		list_del(&st->entry);
        	}
            st->timer_cb(st);
        }
    }
}

struct task soft_timer_task = 
{
    .do_task = soft_timer_tack_callback,
};

