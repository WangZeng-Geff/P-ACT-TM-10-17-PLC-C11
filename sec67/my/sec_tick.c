#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include.h"
#include "timer.h"
#include "task.h"
#include "comfunc.h"
#include "sec_tick.h"
#include "cmd.h"
#include "sec_printf.h"

//---------------------------------------------------------------------------------------
volatile uint32_t sec_jiffies = INITIAL_JIFFIES;
//---------------------------------------------------------------------------------------
void cycle_100ms_task()
{
	;
}
void cycle_1s_task()
{
	static uint8_t TimeSecCount = 0;
    if (TimeCount > 59) {
        cycle_1min_task();
        TimeSecCount = 0;
    }
    TimeSecCount ++;
}
void cycle_1min_task()
{
	static uint8_t TimeMinCount = 0;
    if (TimeCount > 19) {
        cycle_20min_task();
        TimeMinCount = 0;
    }
    TimeMinCount ++;
}
void cycle_20min_task()
{
    if ((0x01 == eep_param.SkyModelContral) && (eep_param.SkyBodyCotrlCount > 0)){//天空模型开启且人为控制过
        eep_param.SkyMonitorTime++;
        if (eep_param.SkyMonitorTime > 71) {
            eep_param.SkyMonitorTime = 0;
            eep_param.SkyBodyCotrlCount = 0;
        }
//        flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //将参数写入flash内保存
        flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, SkyBodyCotrlCount), 
                              (uint8_t *)(&eep_param.SkyBodyCotrlCount), 
                              sizeof(eep_param.SkyBodyCotrlCount) + sizeof(eep_param.SkyMonitorTime));     //将参数写入flash内保存（优化）
    }
}



static void sec_tick_handle(ULONG arg)
{
	static uint8_t Timegount = 0;
	if(!(Timegount % 2)){
		cycle_100ms_task();//一百毫秒任务
	}
	if(Timegount > 20){
		cycle_1s_task();//一秒任务
		Timegount = 0;
	}
    extern struct task soft_timer_task;
    sec_jiffies++;
    Timegount ++;
    task_start(&soft_timer_task);
}

//-------------50ms--------------------------------------------------------------------------
int sec_tick_init(void)
{
    estimer_t tick_tmr = timer_create(TICKS_PER_SEC / 10, TICKS_PER_SEC / 10, TMR_RUNNING,
		TMR_PERIODIC, sec_tick_handle, NULL, "sec_tick_tmr");

    if (!tick_tmr)
    {
        esprintf("creat sec tick timer failed!\n");
        return -1;
    }  

    return 0;
}

unsigned int jiffies_to_msecs(const unsigned long j)
{
	return (1000 / SEC_HZ) * j;
}


//---------------------------------------------------------------------------------------
