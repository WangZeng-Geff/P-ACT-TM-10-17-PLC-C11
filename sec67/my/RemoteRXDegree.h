#ifndef __REMOTRRXDEGREE_H__
#define __REMOTRRXDEGREE_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include.h"
#include "sdk.h"
#include "flash.h"
#include "sdk_svc.h"
#include "comfunc.h"
#include "sec_printf.h"
#include "protocol.h"
#include "update.h"
#include "timer.h"
#include "repeater.h"
#include "gpio.h"
#include "sec_tick.h"
#include "dev_show.h"
#include "task.h"
#include "softtimer.h"
#include "gpio.h"
#include "esuart.h"
#include "Motor_func.h"     //更改电机操作封装文件
#include "types.h"
#include "Motor_func.h"
#include "protocol.h"


#define offset_of(obj_type,mb)  ((uint32_t)&(((obj_type*)0)->mb))
#define SHS_FRAME_HEAD       offset_of(struct SmartFrame, data)
#define did_item(x)     (uint8_t)(x), (uint8_t)(x>>8)


#define HZ (10)
#define GATEWAY_NUM (0xFF)
#define GATEWAY_MAX_TRY_CNT (0x03)
#define MAX_SB_NUM   (0x03) 
#define REPORT_OVER 0
#define REPORT_START 1

#define     RELIABLE_REPORT   0x01
//#define     REPORT_NO         0X00
//#define     REPORT_GW         0X01
//#define     REPORT_SB         0X02


#define ADDRESS_LEN     0x04

#define BLK_NO_SHIFT    5
#define MAX_POOL_SZ     0x100
#define MAX_BUFFER_SZ   (MAX_POOL_SZ - (MAX_POOL_SZ >> BLK_NO_SHIFT))



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
///其他项目添加 
struct FBD_Frame
{
    uint8_t did[2];
    uint8_t ctrl;
    uint8_t data[1];
};

struct SHS_frame
{
    uint8_t stc;
    uint8_t said[ADDRESS_LEN];
    uint8_t taid[ADDRESS_LEN];
    uint8_t seq;
    uint8_t length;
    uint8_t infor[1];
};
extern uint8_t g_frame_buffer;

////添加完成 


enum {
        FREE,
        KEY,
        BROADCAST,
        P2P,
        GATEWAY_ACK
};

struct stage_data_t
{
    uint8_t  aid_position;
    uint8_t  equipment_gid;
    uint8_t  try_cnt;
    uint8_t  state_change_mode;
    uint8_t  sid_equal_flag;
    uint8_t  last_said[4];
    uint8_t  queue_pos;
    uint8_t  said_queue[12];
    uint32_t counter_for_gw;
    uint8_t  gw_send_flag;
    uint8_t  curtain_degree_change;
    uint8_t  wait_cnt;
    uint8_t  find_myself_flag;
    uint8_t  taker_id[4];
};

struct power_on_report_t
{
	uint8_t	  report_try_cnt;		//上报重试次数
	uint16_t  report_delay_time;  	//上报延时时间
	uint16_t  report_counter;	    //上报计时
	uint8_t	  report_flag;		    //上报网关标志
	uint8_t	  report_ack;		    //网关回复标志
	uint16_t  device_sid;
	uint8_t	  report_seq;
	uint8_t	  report_end;
};
#define	MAX_REPORT_TRY_CNT	3
#define	REPORT_END			0xFF

extern struct power_on_report_t power_on_report;
extern struct stage_data_t stage_data;




//uint8_t get_1byte_bit1_number(uint8_t data,uint8_t pos);

uint8_t get_message_source(void *source_addr,uint8_t destination_flag);

void stage_hook(void);

void check_gw_ack(void);
void stage_data_init(void);
void stage_restart_data_init(uint8_t state);
void check_state_change(void);


void poweron_report_init(void);
void check_power_on(void);
void power_on_report_task(void);
void check_power_on_report_ack(void);
void power_on_report_over(void);
uint16_t cal_report_delay_time(uint8_t try_count);
uint16_t take_small_value_with_100(uint16_t report_freq);
uint8_t org_fbd_frame(uint8_t frame_buf[], uint8_t did_l, uint8_t did_h,uint8_t data[], uint8_t data_len);

#endif
