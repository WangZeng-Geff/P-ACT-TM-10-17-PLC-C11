#ifndef __TASK_H_
#define __TASK_H_

#include "types.h"
#include "list.h"

enum task_flag
{
    TF_ALWAYS_ALIVE = 0x01
};

enum task_state
{
    TS_STOPPED,
    TS_RUNNING,
};

struct task
{
    struct list_head entry;

    enum task_state state;
    u32 flags;

    void *pri;
    void (*do_task)(struct task *t);
};

void task_add(struct task *t);
void task_del(struct task *t);
void task_start(struct task *t);
void task_handle(void);
#endif
