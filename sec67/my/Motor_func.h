/**********************************************************************************
**--------------File Info----------------------------------------------------------
** File name:           motor_func.h
** Last modified Date:  2013-07-05
** Last Version:        V1.0
** Descriptions:        
**
**---------------------------------------------------------------------------------
** Created by:          ???
** Created date:        2013-07-5
** Version:             V1.0
** Descriptions:        ????????
**
**---------------------------------------------------------------------------------      
**********************************************************************************/
#ifndef MOTOR_FUNC_H
#define MOTOR_FUNC_H

#include "protocol.h"

#define INVALID_DEGREE       0xFF   //开度无效所赋值
extern struct EEPPARAM eep_param;	//flash缓存区数据



//任务属性划分
enum{
    MotorResetAct = 0,           //电机复位后动作
    MotorResetcount,
    MotorDevShowAct
};
//产品所处状态划分
enum{
    MOTORTIME_RUNING = 0,           //正常运行中
    MOTORTIME_INIT,                 //产品初始化
    MOTORTIME_SEARCH,                //产品搜索中
    MOTORTIME_SEARCH2                //产品搜索中
};

//电机运行状态性质划分
enum{
    MOTOR_FORWARD= 1,           //开窗
    MOTOR_REVERSE,              //关窗
    MOTOR_STOP                  //暂停
};

//控制电机动作报文格式  
struct UART_MOSTOR_FRAME
{
  uint8_t head[15];          //报文起始码 1byte
	uint8_t start;          //报文起始码 1byte
	uint8_t SAddr_L;        //从设备地址高八位 1byte
	uint8_t SAddr_H;        //从设备地址低八位 1byte
	uint8_t cmd;            //报文功能码
	uint8_t data[3];        //报文数据(数据地址1byte/数据长度1byte/数据信息1byte)+校验码（CRC16_L+CRC16_H）2byte
};

struct BASICCACULAPAR       //电机计算基础参数
{
    uint16_t  motorup_time0025;
    uint16_t  motorup_time2550;
    uint16_t  motorup_time5075;
    uint16_t  motorup_time7500;
    uint16_t  motordn_time0075;
    uint16_t  motordn_time7550;
    uint16_t  motordn_time5025;
    uint16_t  motordn_time2500;
};
extern struct BASICCACULAPAR BasicCalculaPar;		//电机计算基础参数

//电机运行参数集合
struct MOTORPARA
{
    uint8_t  curtain_init;      //设备状态性质  0：正常运行 1：正在校准 2：搜索设备
    uint8_t  Motoe_lastState;   //电机上次运动状态
    uint8_t  Motoe_nextState;   //当前动作完成电机下次运动状态
    uint8_t  Motoe_Runing;      //电机正在运行   0:停止 1：运行
    uint8_t  Motoe_RuningD;     //电机运行方向   1：停 2：正 3：反
    uint8_t  Motoe_TaragetD;    //电机目标运行方向 1：停 2：正 3：反
    uint16_t Motoe_RunT;        //电机运行时间
    uint16_t Motoe_TargetT;     //目标运行时间
    uint8_t  TargetDegree;      //目标开度值（0 ~ 100）
    uint8_t  CurrentDegree;     //当前开度值（0 ~ 100）
    uint8_t  LastDegree;        //上次开度值（0 ~ 100
    uint8_t  next_action_cmd;   //复位后下一次动作指令
    uint16_t next_action_time;  //复位后下一次动作时间
    uint8_t  action_cnt;        //校准计数
    uint8_t  reset_cnt;         //校准前动作次数
    uint8_t  reset_flag;        //校准标志 0不需校准；1需要校准
    uint8_t  taker_is_gateway;  //指令来源
};
extern struct MOTORPARA MostorRunPara;          //电机运行过程中所用参数



uint8_t  ControlMessage_uart_frame(uint8_t cmd, uint8_t *data, uint8_t  data_len);   //操作电机指令发送
uint8_t  curtain_time_init(void);                                                    //设定标准时间初始化函数
uint16_t msc_Moster_init(void);                                                      //毫秒单位电机动作计时初始化
uint16_t Motor_Action_Control(uint8_t action,uint16_t actime);                       //电机动作控制 
uint16_t cal_time(uint8_t dstdegree);                                                //计算窗帘动作时间
void     cal_degree(void);                                                           //计算当前窗帘开度值

//操作步骤实现
uint8_t  OneWay_Simple_Operation(uint8_t cmd, uint16_t time);   //简单操作直上直下操作
uint8_t  Percent_Complex_Operation(uint8_t Percent);            //复杂操作带开度控制
uint8_t  Set_StandAct_Time(uint16_t uptime, uint16_t dntime);   //设定标准开度时间
void  Set_DevShow_Time(void);                                   //搜索指定设备

//电机操作初始化
void MostorManageInit(void);
//实时监测任务
extern void MostorRealTimeMonitorTask(void);

#endif


