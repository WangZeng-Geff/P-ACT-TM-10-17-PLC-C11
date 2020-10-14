#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include.h"
#include "list.h"
#include "comfunc.h"
#include "sec_printf.h"
#include "protocol.h"
#include "task.h"

static LIST_HEAD(task_head);

void task_add(struct task *t)
{
    t->state   = TS_STOPPED;
    list_add_tail(&t->entry, &task_head);
}
void task_del(struct task *t)
{
    if (!t) return;

    list_del(&t->entry);
}
void task_start(struct task *t)
{
    if (!t) return;

    t->state = TS_RUNNING;
}
void task_handle(void)
{
    struct task *t;

    MostorRealTimeMonitorTask();

    list_for_each_entry(t, &task_head, entry) 
    {
        if (t->state == TS_RUNNING || (t->flags & TF_ALWAYS_ALIVE)) 
        {
            t->state = TS_STOPPED;
            t->do_task(t->pri);
        }
    }
}
