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

#define INVALID_DEGREE       0xFF   //������Ч����ֵ
extern struct EEPPARAM eep_param;	//flash����������



//�������Ի���
enum{
    MotorResetAct = 0,           //�����λ����
    MotorResetcount,
    MotorDevShowAct
};
//��Ʒ����״̬����
enum{
    MOTORTIME_RUNING = 0,           //����������
    MOTORTIME_INIT,                 //��Ʒ��ʼ��
    MOTORTIME_SEARCH,                //��Ʒ������
    MOTORTIME_SEARCH2                //��Ʒ������
};

//�������״̬���ʻ���
enum{
    MOTOR_FORWARD= 1,           //����
    MOTOR_REVERSE,              //�ش�
    MOTOR_STOP                  //��ͣ
};

//���Ƶ���������ĸ�ʽ  
struct UART_MOSTOR_FRAME
{
  uint8_t head[15];          //������ʼ�� 1byte
	uint8_t start;          //������ʼ�� 1byte
	uint8_t SAddr_L;        //���豸��ַ�߰�λ 1byte
	uint8_t SAddr_H;        //���豸��ַ�Ͱ�λ 1byte
	uint8_t cmd;            //���Ĺ�����
	uint8_t data[3];        //��������(���ݵ�ַ1byte/���ݳ���1byte/������Ϣ1byte)+У���루CRC16_L+CRC16_H��2byte
};

struct BASICCACULAPAR       //��������������
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
extern struct BASICCACULAPAR BasicCalculaPar;		//��������������

//������в�������
struct MOTORPARA
{
    uint8_t  curtain_init;      //�豸״̬����  0���������� 1������У׼ 2�������豸
    uint8_t  Motoe_lastState;   //����ϴ��˶�״̬
    uint8_t  Motoe_nextState;   //��ǰ������ɵ���´��˶�״̬
    uint8_t  Motoe_Runing;      //�����������   0:ֹͣ 1������
    uint8_t  Motoe_RuningD;     //������з���   1��ͣ 2���� 3����
    uint8_t  Motoe_TaragetD;    //���Ŀ�����з��� 1��ͣ 2���� 3����
    uint16_t Motoe_RunT;        //�������ʱ��
    uint16_t Motoe_TargetT;     //Ŀ������ʱ��
    uint8_t  TargetDegree;      //Ŀ�꿪��ֵ��0 ~ 100��
    uint8_t  CurrentDegree;     //��ǰ����ֵ��0 ~ 100��
    uint8_t  LastDegree;        //�ϴο���ֵ��0 ~ 100
    uint8_t  next_action_cmd;   //��λ����һ�ζ���ָ��
    uint16_t next_action_time;  //��λ����һ�ζ���ʱ��
    uint8_t  action_cnt;        //У׼����
    uint8_t  reset_cnt;         //У׼ǰ��������
    uint8_t  reset_flag;        //У׼��־ 0����У׼��1��ҪУ׼
    uint8_t  taker_is_gateway;  //ָ����Դ
};
extern struct MOTORPARA MostorRunPara;          //������й��������ò���



uint8_t  ControlMessage_uart_frame(uint8_t cmd, uint8_t *data, uint8_t  data_len);   //�������ָ���
uint8_t  curtain_time_init(void);                                                    //�趨��׼ʱ���ʼ������
uint16_t msc_Moster_init(void);                                                      //���뵥λ���������ʱ��ʼ��
uint16_t Motor_Action_Control(uint8_t action,uint16_t actime);                       //����������� 
uint16_t cal_time(uint8_t dstdegree);                                                //���㴰������ʱ��
void     cal_degree(void);                                                           //���㵱ǰ��������ֵ

//��������ʵ��
uint8_t  OneWay_Simple_Operation(uint8_t cmd, uint16_t time);   //�򵥲���ֱ��ֱ�²���
uint8_t  Percent_Complex_Operation(uint8_t Percent);            //���Ӳ��������ȿ���
uint8_t  Set_StandAct_Time(uint16_t uptime, uint16_t dntime);   //�趨��׼����ʱ��
void  Set_DevShow_Time(void);                                   //����ָ���豸

//���������ʼ��
void MostorManageInit(void);
//ʵʱ�������
extern void MostorRealTimeMonitorTask(void);

#endif


