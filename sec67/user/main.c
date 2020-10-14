#include "include.h"
#include "sec.h"
#include "uart.h"
#include "task.h"
#include "repeater.h"
#include "softtimer.h"
#include "esuart.h"
#include "sec_printf.h"
#include "sec_tick.h"


static void uart1_check(struct task *t)
{
    uint32_t len = 0, ret = 0;
    uint8_t buf[0x200];
    len = esuart_peak_data(UART_CHN_1, buf, sizeof(buf));
    if (len > 0) 
    {
        pr_info("[PLC-LOCALCAST]:");
        print_debug_array(KERN_INFO, buf, len);
        ret = adaptor_on_uart1_received(buf, len);
    }
    if (ret > 0) 
    {
        esuart_read_data(UART_CHN_DM, buf, ret);
    }
	else if(len >= sizeof(buf))
	{
		esuart_read_data(UART_CHN_DM, buf, len);
	}
}

//---------------------------------------------------------------------------------------
struct task uart1_check_task = 
{
	.flags   = TF_ALWAYS_ALIVE,
    .do_task = uart1_check,
};
//---------------------------------------------------------------------------------------
static void dev_control(struct task *t)
{	
	struct UART_MOSTOR_FRAME frame;
	static uint8_t cnt = 0;
	if(report.uart_busy)
	{
		if(++cnt >= 4)
		{
			cnt = 0;
			report.uart_busy = 0;
		}
		return;
	}
	if (peek_uart_order(&frame))
	{
//		esuart_send_data(UART_CHN_DM, (uint8_t *)&frame, frame.data_len + 4);
		esuart_send_data(UART_CHN_DM, (uint8_t *)&frame, 23);
		report.uart_busy = 1;
	}
	else
	{
		return;
	}
}

struct task dev_control_task = 
{
    .do_task = dev_control,
    .pri = &dev_control_task,
};

int sec_main(int argc, char *argv[])
{
	START_INIT();

    task_add(&soft_timer_task);
    task_add(&uart1_check_task);
    task_add(&dev_control_task);
    system_init();
	while(1)
	{
        task_handle();
	}
}
//---------------------------------------------------------------------------------------
