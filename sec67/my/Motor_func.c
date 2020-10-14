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
#include "Motor_func.h"
#include "cmd.h"

struct MOTORPARA MostorRunPara;             //������й��������ò���
struct BASICCACULAPAR BasicCalculaPar;		//��������������
struct EEPPARAM eep_param;					//flash����������

uint16_t MosterTiming = 0xffff;

uint8_t MotorRealTimetaskList = 0;

/*-------------------------------------------------
????:unsigned int calccrc(uchar crcbuf,uint crc)
????:?crc???????CRCBUF?CRC?
--------------------------------------------------*/
unsigned int calccrc(uint16_t crcbuf,uint16_t crc)
{
	uint8_t i; 
	uint8_t chk;
	crc=crc ^ crcbuf;
	for(i=0;i<8;i++)
	{
	  chk=crc&1;
	  crc=crc>>1;
	  crc=crc&0x7fff;
	  if (chk==1)
	  crc=crc^0xa001;
	  crc=crc&0xffff;
	}
	return crc; 
}
/*------------------------------------------------
????:?????CRC???crc??????,*buf?????,x??????
--------------------------------------------------*/
uint16_t qioucrc16(uint16_t crc, uint8_t *buf, uint16_t x)
{
	uint8_t hi,lo;
	uint16_t i;
	for (i=0;i<x;i++)
	{
	   crc=calccrc(*buf,crc);
	   buf++;
	}
	hi=crc%256;
	lo=crc/256;
	crc=(hi<<8)|lo;
	return crc;
}
//---------------------------------------------------------------------------------------
uint8_t ControlMessage_uart_frame(uint8_t cmd, uint8_t *data, uint8_t  data_len)
{
    struct UART_MOSTOR_FRAME frame;
    uint8_t Action = *data;
    frame.head[0] = 0x7e;
    frame.head[1] = 0x01;
    frame.start = 0x55;
    frame.SAddr_L = 0xFE;
    frame.SAddr_H = 0xFE;
    frame.cmd = cmd;        //���Ĺ�����
    //pr_info("\nControlMessage_uart_frame => Ctotral Action = %d!\n",*data);
    
    if(1 == eep_param.location && Action != 3){

        Action = *data==1 ? 2 : 1;
    }
    frame.data[0] = Action;
	if(Action == 1){
		frame.data[1] = 0xb9;
		frame.data[2] = 0x24;
    }else if(Action == 2){
		frame.data[1] = 0xf9;
		frame.data[2] = 0x25;
    }else if(Action == 3){
		frame.data[1] = 0x38;
		frame.data[2] = 0xE5;
    }
    insert_uart_order(&frame);
    return sizeof(frame);
}

uint8_t ConversionCommand(uint8_t cmd)
{
    switch (cmd) {
    case 1:
        cmd = MOTOR_STOP;
//        pr_info("Conversion Command Function => MOTOR_STOP!\n");
        break;
    case 2:
        cmd = MOTOR_FORWARD;//(kai)
//        pr_info("Conversion Command Function => MOTOR_FORWARD!\n");
        break;
    case 3: 
        cmd = MOTOR_REVERSE;//(guan)
//        pr_info("Conversion Command Function => MOTOR_REVERSE!\n");
        break;
    default:
        cmd = 0xff;
        break;
    }
    return cmd;
}

/////////////////////////////////////////////// 
///ִ�и�λ���� 
void PerformPostCalibrationActions(void)
{
    if (!MostorRunPara.next_action_cmd || !MostorRunPara.next_action_time) return ;
    pr_info("[ACT-RESETNEST] next_action_cmd=%d;next_action_time=%d.\n",MostorRunPara.next_action_cmd,MostorRunPara.next_action_time);
    MotorRealTimetaskList = MotorRealTimetaskList | (1 << MotorResetAct);
    MotorRealTimetaskList = MotorRealTimetaskList | (1 << MotorResetcount);
}
/***************************************** 
 ****�������ʱ�����
 ****ʮ�������һ��
******************************************/
static void sec_Moster_handle(ULONG arg)
{
    static uint8_t dealyCount = 0; 
    if (MosterTiming == 0xFFFF || !MostorRunPara.Motoe_TargetT) return;
    if (MosterTiming >= MostorRunPara.Motoe_TargetT) {
        eep_param.acting =  0;
        MostorRunPara.Motoe_Runing = 0;
        MosterTiming = 0xFFFF;
        Motor_Action_Control(MOTOR_STOP,0);
        if (MostorRunPara.CurrentDegree == 0)
            PerformPostCalibrationActions();
        return ;
    }
    if ((MotorRealTimetaskList & (1 << MotorResetcount)) && !MosterTiming){
        if (dealyCount < 150) {
            dealyCount++;
            return ;
        }
        dealyCount = 0;
        MotorRealTimetaskList = MotorRealTimetaskList & (~(1 << MotorResetcount));
    }
    MosterTiming ++;
    MostorRunPara.Motoe_RunT = MosterTiming;
}
/***************************************************************** 
 ****���������ʱ��ʼ��
 ****��ʼ��Ŀ�꺯��
 *****************************************************************/
uint16_t msc_Moster_init(void)
{
    MostorRunPara.Motoe_TargetT = 0 ;//��ʼ�������תĿ��ʱ��
    estimer_t tick_tmr = timer_create(2, 2, TMR_RUNNING,
		TMR_PERIODIC, sec_Moster_handle, NULL, "msc_Moster_tmr");
    if (!tick_tmr)
        return 1;
    return 0;
}

/*************************************************************** 
 �����������
****************************************************************/
uint16_t Motor_Action_Control(uint8_t action,uint16_t time){
    uint8_t ret;
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������ʱ���½�ʱ��д�뵽�ڴ���
    ret = ControlMessage_uart_frame(0x03, &action, 1);
//    pr_info("0.0s-Motor_Action_Control time => %d\n",time);
    MostorRunPara.Motoe_TargetT = (action == MOTOR_STOP) ? 0 : time;
    MosterTiming = (action == MOTOR_STOP) ? 0xffff : 0;
    if (action == MOTOR_STOP) {
//        eep_param.acting =  0;
        //���㵱ǰ����ֵ����д���ڴ�
        cal_degree();
        MostorRunPara.Motoe_RuningD = action;
        if (MostorRunPara.curtain_init == MOTORTIME_SEARCH) MotorRealTimetaskList = MotorRealTimetaskList | (1 << MotorDevShowAct); //�趨�����ڶ�������
        MostorRunPara.curtain_init = MOTORTIME_RUNING;  //�趨Ϊ����״̬
        MostorRunPara.LastDegree = MostorRunPara.CurrentDegree;
        MostorRunPara.TargetDegree = MostorRunPara.CurrentDegree;
        Resend_report(1);       //������ݲ����ϴ�
        eep_param.degree = MostorRunPara.CurrentDegree;
    }else{
        eep_param.degree = INVALID_DEGREE;
        if ((MostorRunPara.CurrentDegree == 0) || (MostorRunPara.CurrentDegree == 100)){
            MostorRunPara.action_cnt = 0; 
            //У׼��������
            eep_param.ActionCount = 0;
//            flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
            flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, ActionCount), 
                                  (uint8_t *)(&eep_param.ActionCount), 
                                  sizeof(eep_param.ActionCount));     //������д��flash�ڱ��棨�Ż���
            MostorRunPara.reset_flag = 0;
        }else{
            if (action != MostorRunPara.Motoe_RuningD) {
//               pr_info("Motor_Action_Control's reset_cnt => MostorRunPara.reset_cnt:%d!\n",MostorRunPara.action_cnt);
                MostorRunPara.action_cnt++;
                //У׼��������
                eep_param.ActionCount ++;
//                flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
                flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, ActionCount), 
                                      (uint8_t *)(&eep_param.ActionCount), 
                                      sizeof(eep_param.ActionCount));     //������д��flash�ڱ��棨�Ż���
            if (MostorRunPara.action_cnt >= MostorRunPara.reset_cnt && 0 == eep_param.SkyModelContral){//�������ģ�Ϳ�������
               MostorRunPara.reset_flag = 1;
//               pr_info("Motor_Action_Control's reset_cnt => Arrival reset condition!\n");
                } else if (eep_param.ActionCount > eep_param.ForceCaliTime && 1 == eep_param.SkyModelContral) {
                    MostorRunPara.reset_flag = 1;
                }
          }
      }
    }
    eep_param.acting =  (action == MOTOR_STOP) ? 0 : 1;
//    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //�뵽�ڴ���
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, acting), 
                          (uint8_t *)(&eep_param.acting), 
                          sizeof(eep_param.acting));     //������д��flash�ڱ��棨�Ż���
    MostorRunPara.Motoe_Runing = (action == MOTOR_STOP) ? 0 : 1;
    MostorRunPara.Motoe_RuningD = action;
    if (time == 0) return 0;
    return !ret;
}

//////////////////////////////////////////////////
//�������Ȼ���������ʼ��
uint8_t curtain_time_init(void)
{
   uint16_t motor_time, cali_up, cali_dn;
    if (eep_param.motor_up_time >= 5000){
       cali_up = 200;
       cali_dn = 120;
       MostorRunPara.reset_cnt = 13;
     }else if (eep_param.motor_up_time >= 4200){
       cali_up = 100;
       cali_dn = 60;
        MostorRunPara.reset_cnt = 13;
			 ;
     }else if (eep_param.motor_up_time >= 3400){
       cali_up = 50;
       cali_dn = 30;
       MostorRunPara.reset_cnt = 10;
     }else if (eep_param.motor_up_time >= 2600){
       cali_up = 20;
       cali_dn = 15;
       MostorRunPara.reset_cnt = 7;
    }else{
       cali_up = 10;
       cali_dn = 5;
       MostorRunPara.reset_cnt = 7;
    }
    motor_time = eep_param.motor_up_time - (6 * cali_up);
    BasicCalculaPar.motorup_time7500 = motor_time / 4;
    BasicCalculaPar.motorup_time5075 = BasicCalculaPar.motorup_time7500 + cali_up;
    BasicCalculaPar.motorup_time2550 = BasicCalculaPar.motorup_time7500 + 2 * cali_up;
    BasicCalculaPar.motorup_time0025 = BasicCalculaPar.motorup_time7500 + 3 * cali_up;
    motor_time = eep_param.motor_dn_time - (6 * cali_dn);
    BasicCalculaPar.motordn_time0075 = motor_time / 4;
    BasicCalculaPar.motordn_time7550 = BasicCalculaPar.motordn_time0075 + cali_dn;
    BasicCalculaPar.motordn_time5025 = BasicCalculaPar.motordn_time0075 + 2 * cali_dn;
    BasicCalculaPar.motordn_time2500 = BasicCalculaPar.motordn_time0075 + 3 * cali_dn;
    return 0;
}

////////////////////////////////////////////////
//��ȡ�л���������ʱ��
uint16_t cal_time(uint8_t dstdegree)
{
   uint8_t difdegree, degree;
   uint16_t time;
   if (MostorRunPara.CurrentDegree == INVALID_DEGREE) return 0;   //����ֵ��Ч���в���
   degree = MostorRunPara.CurrentDegree > dstdegree ? (MostorRunPara.CurrentDegree - dstdegree) : (dstdegree - MostorRunPara.CurrentDegree );
   if (MostorRunPara.CurrentDegree > dstdegree){
      if (MostorRunPara.CurrentDegree >= 75){
         difdegree = MostorRunPara.CurrentDegree - 75;
         if (degree > difdegree){
            time = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motordn_time0075 / (uint32_t)25;
            degree -= difdegree;
            if (degree > 25){
               degree -= 25;
               time += BasicCalculaPar.motordn_time7550;

               if (degree > 25){
                  degree -= 25;
                  time += BasicCalculaPar.motordn_time5025;
                  time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time2500 / (uint32_t)25;
               }else{
                  time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time5025 / (uint32_t)25;
               }
            }else{
               time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time7550 / (uint32_t)25;
            }
         }else{
            time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time0075 / (uint32_t)25;
         }
      }else if (MostorRunPara.CurrentDegree >= 50){
         difdegree = MostorRunPara.CurrentDegree - 50;
         if (degree > difdegree){
            time = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motordn_time7550 / (uint32_t)25;
            degree -= difdegree;

            if (degree > 25){
               degree -= 25;
               time += BasicCalculaPar.motordn_time5025;
               time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time2500 / (uint32_t)25;
            }else{
               time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time5025 / (uint32_t)25;
            }
         }else{
            time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time7550 / (uint32_t)25;
         }
      }else if (MostorRunPara.CurrentDegree >= 25){
         difdegree = MostorRunPara.CurrentDegree - 25;
         if (degree > difdegree)
         {
            time = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motordn_time5025 / (uint32_t)25;
            degree -= difdegree;
            time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time2500 / (uint32_t)25;
         }else{
            time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time5025 / (uint32_t)25;
         }
      }else{// >= 0
         time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motordn_time2500 / (uint32_t)25;
      }
   }
   if (MostorRunPara.CurrentDegree < dstdegree){
      if (MostorRunPara.CurrentDegree <= 25){
         difdegree = 25 - MostorRunPara.CurrentDegree;
         if (degree > difdegree){
            time = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motorup_time0025 / (uint32_t)25;
            degree -= difdegree;
            if (degree > 25){
               degree -= 25;
               time += BasicCalculaPar.motorup_time2550;
               if (degree > 25){
                  degree -= 25;
                  time += BasicCalculaPar.motorup_time5075;
                  time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time7500 / (uint32_t)25;
               }else{
                  time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time5075 / (uint32_t)25;
               }
            }else{
               time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time2550 / (uint32_t)25;
            }
         }else{
            time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time0025 / (uint32_t)25;
         }
      }else if (MostorRunPara.CurrentDegree <= 50){
         difdegree = 50 - MostorRunPara.CurrentDegree;
         if (degree > difdegree){
            time = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motorup_time2550 / (uint32_t)25;
            degree -= difdegree;
            if (degree > 25){
               degree -= 25;
               time += BasicCalculaPar.motorup_time5075;
               time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time7500 / (uint32_t)25;
            }else{
               time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time5075 / (uint32_t)25;
            }
         }else{
            time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time2550 / (uint32_t)25;
         }
      }else if (MostorRunPara.CurrentDegree <= 75){
         difdegree = 75 - MostorRunPara.CurrentDegree;
         if (degree > difdegree){
            time = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motorup_time5075 / (uint32_t)25;
            degree -= difdegree;
            time += (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time7500 / (uint32_t)25;
         }else{
            time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time5075 / (uint32_t)25;
         }
      }else{// <= 100
         time = (uint32_t)degree * (uint32_t)BasicCalculaPar.motorup_time7500 / (uint32_t)25;
      }
   }

//   pr_info("\n**************cal_time => time = %d !**************\n\n",time);

   return time;
}

uint16_t time2degree(uint32_t time, uint32_t total_degree, uint32_t degree_time)
{
   if (time > degree_time)
      return (uint16_t)total_degree;
   return (uint16_t)(((time + 2) * total_degree) / degree_time); 
}
/////////////////////////////////////////////
//���㵱ǰ����ֵ
void cal_degree(void)
{
   uint16_t degree, difdegree;
   uint16_t diftime;
//   pr_info("void cal_degree(void)=>MostorRunPara.Motoe_RunT = %d\n");
   if (!eep_param.motor_up_time || !eep_param.motor_dn_time)  return;
   if(!MostorRunPara.Motoe_Runing && MostorRunPara.curtain_init != MOTORTIME_SEARCH && MostorRunPara.curtain_init != MOTORTIME_SEARCH2){
       MostorRunPara.curtain_init = MostorRunPara.curtain_init == MOTORTIME_INIT ? MOTORTIME_RUNING : MostorRunPara.curtain_init;//�����һ�ζ����Ƿ��ǳ�ʼ����������״̬���ó������У����򲻲���
       MostorRunPara.CurrentDegree = MostorRunPara.LastDegree;
//       pr_info("void cal_degree(void)=>MostorRunPara.Motoe_Runing");
//       pr_info("cal_degree1 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
       return ;
   }
   if (MostorRunPara.Motoe_RuningD == MOTOR_FORWARD){
//       pr_info("MostorRunPara.Motoe_RuningD == MOTOR_FORWARD\n");
       if (MostorRunPara.CurrentDegree > 100){
           if (MostorRunPara.Motoe_RunT >= (eep_param.motor_up_time + (eep_param.motor_up_time >> 2) - 6)) // ��ȥһ����С��������߳���ɿ���
                MostorRunPara.CurrentDegree = 100; 
//           pr_info("cal_degree2 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
           return;
        }else if (MostorRunPara.CurrentDegree >= 75){
          degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time7500);
        }else if (MostorRunPara.CurrentDegree >= 50){
          difdegree = 75 - MostorRunPara.CurrentDegree;
          diftime = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motorup_time5075 / (uint32_t)25;
          if (MostorRunPara.Motoe_RunT > diftime){
             degree = difdegree;
             MostorRunPara.Motoe_RunT -= diftime;
             degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time7500);
          }else{
             degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time5075);
          }
       }else if (MostorRunPara.CurrentDegree >= 25){
          difdegree = 50 - MostorRunPara.CurrentDegree;
          diftime = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motorup_time2550 / (uint32_t)25;
          if (MostorRunPara.Motoe_RunT >= diftime){
             degree = difdegree;
             MostorRunPara.Motoe_RunT -= diftime;
             if (MostorRunPara.Motoe_RunT >= BasicCalculaPar.motorup_time5075){
                degree += 25;
                MostorRunPara.Motoe_RunT -= BasicCalculaPar.motorup_time5075;
                degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time7500);
             }else{
                degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time5075);
             }
          }else{
             degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time2550);
          }
       }else{
          difdegree = 25 - MostorRunPara.CurrentDegree;
          diftime = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motorup_time0025 / (uint32_t)25;
          if (MostorRunPara.Motoe_RunT >= diftime){
             degree = difdegree;
             MostorRunPara.Motoe_RunT -= diftime;
             if (MostorRunPara.Motoe_RunT >= BasicCalculaPar.motorup_time2550){
                degree += 25;
                MostorRunPara.Motoe_RunT -= BasicCalculaPar.motorup_time2550;
                if (MostorRunPara.Motoe_RunT >= BasicCalculaPar.motorup_time5075){
                   degree += 25;
                   MostorRunPara.Motoe_RunT -= BasicCalculaPar.motorup_time5075;
                   degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time7500);
                }else{
                   degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time5075);
                }
             }else{
                degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time2550);
             }
          }else{
             degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motorup_time0025);
          }
       }
       MostorRunPara.Motoe_RunT = 0;
       if (MostorRunPara.CurrentDegree == 100){
//           pr_info("cal_degree3 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
           return;
       }
       if (((uint8_t)(MostorRunPara.CurrentDegree + degree)) > 100)
          MostorRunPara.CurrentDegree = 100;
       else
          MostorRunPara.CurrentDegree += (uint8_t)degree; 
//          pr_info("cal_degree3_1 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
    }
    if (MostorRunPara.Motoe_RuningD == MOTOR_REVERSE) {
//       pr_info("MostorRunPara.Motoe_RuningD == MOTOR_REVERSE\n");
       if (MostorRunPara.CurrentDegree <= 25){
          degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time2500);
       }else if (MostorRunPara.CurrentDegree <= 50){
          difdegree = MostorRunPara.CurrentDegree - 25;
          diftime = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motordn_time5025 / (uint32_t)25;
          if (MostorRunPara.Motoe_RunT > diftime){
             degree = difdegree;
             MostorRunPara.Motoe_RunT -= diftime;
             degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time2500);
          }else{
             degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time5025);
          }
       }else if (MostorRunPara.CurrentDegree <= 75){
          difdegree = MostorRunPara.CurrentDegree - 50;
          diftime = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motordn_time7550 / (uint32_t)25;
          if (MostorRunPara.Motoe_RunT >= diftime){
             degree = difdegree;
             MostorRunPara.Motoe_RunT -= diftime;
             if (MostorRunPara.Motoe_RunT >= BasicCalculaPar.motordn_time5025){
                degree += 25;
                MostorRunPara.Motoe_RunT -= BasicCalculaPar.motordn_time5025;
                degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time2500);
             }else{
                degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time5025);
             }
          }else{
             degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time7550);
          }
       }else if (MostorRunPara.CurrentDegree <= 100){
          difdegree = MostorRunPara.CurrentDegree - 75;
          diftime = (uint32_t)difdegree * (uint32_t)BasicCalculaPar.motordn_time0075 / (uint32_t)25;
          if (MostorRunPara.Motoe_RunT >= diftime){
             degree = difdegree;
             MostorRunPara.Motoe_RunT -= diftime;
             if (MostorRunPara.Motoe_RunT >= BasicCalculaPar.motordn_time7550){
                degree += 25;
                MostorRunPara.Motoe_RunT -= BasicCalculaPar.motordn_time7550;
                if (MostorRunPara.Motoe_RunT >= BasicCalculaPar.motordn_time5025){
                   degree += 25;
                   MostorRunPara.Motoe_RunT -= BasicCalculaPar.motordn_time5025;
                   degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time2500);
                }else{
                   degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time5025);
                }
             }else{
                degree += time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time7550);
             }
          }else{
             degree = time2degree(MostorRunPara.Motoe_RunT, 25, BasicCalculaPar.motordn_time0075);
          }
       }else{
           if (MostorRunPara.Motoe_RunT >= (eep_param.motor_dn_time + (eep_param.motor_dn_time >> 2) - 6)){ // ��ȥһ����С��������߳���ɿ���
               MostorRunPara.CurrentDegree = 0; 
//               pr_info("cal_degree4 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
           }
           return;
       }
       MostorRunPara.Motoe_RunT = 0;

       if (MostorRunPara.CurrentDegree == 0){
//           pr_info("cal_degree5 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
           return;
       }
       if (MostorRunPara.CurrentDegree > (uint8_t)degree)
          MostorRunPara.CurrentDegree -= (uint8_t)degree;
       else
          MostorRunPara.CurrentDegree = 0;
//       pr_info("cal_degree1_1 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
    }
//    pr_info("cal_degree6 => currentdegree = %d!\n",MostorRunPara.CurrentDegree);
    return;
}

uint8_t OneWay_Simple_Operation(uint8_t cmd, uint16_t time)
{
	uint8_t ret;
    if ((MostorRunPara.curtain_init != 0)) return DEV_BUSY;   //�趨��׼ʱ���ʼ��/����ָ���豸�в�������
    cmd = ConversionCommand(cmd);     //ת�����ת��Ϊ�������ϵ�ָ��
//���û�������г�ʱ��
    if (!(eep_param.motor_up_time) || !(eep_param.motor_dn_time)){   
//        pr_info("OneWay_Simple_Operation No standeTime RUN!\n");
        ret = Motor_Action_Control(cmd , time);  //�����������
        if(ret) return(DATA_ERR);
        return(NO_ERR);
    }
    if ((cmd != MOTOR_STOP) && (MostorRunPara.Motoe_Runing)){  //�������ת����
       if (MostorRunPara.CurrentDegree == INVALID_DEGREE){ //�ϴζ����е��磬�ϵ����δУ׼����ֵ
           if ((MostorRunPara.Motoe_RuningD == MOTOR_FORWARD)
               && (MostorRunPara.Motoe_RunT >= eep_param.motor_up_time)){ //��ת������ת��ʱ�䳬���г�ʱ��
                  MostorRunPara.CurrentDegree = 100;
                  eep_param.degree = 100;
                  eep_param.acting = 0;
//                  flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));        //����д��flash
                  flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, degree), 
                                        (uint8_t *)(&eep_param.degree), 
                                        sizeof(eep_param.degree) + sizeof(eep_param.acting));     //������д��flash�ڱ��棨�Ż���
              }
           if ((MostorRunPara.Motoe_RuningD == MOTOR_REVERSE) 
                && (MostorRunPara.Motoe_RunT >= eep_param.motor_dn_time)){ //��ת������ת��ʱ�䳬���г�ʱ��
                MostorRunPara.CurrentDegree = 0;
                eep_param.degree = 0;
                eep_param.acting = 0;
//                flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));        //����д��flash
                flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, degree), 
                                      (uint8_t *)(&eep_param.degree), 
                                      sizeof(eep_param.degree) + sizeof(eep_param.acting));     //������д��flash�ڱ��棨�Ż���
           }
       }else{
          cal_degree();
      }
   }
   if (cmd == MOTOR_FORWARD)
   {
        MostorRunPara.LastDegree = 100;
        if (MostorRunPara.CurrentDegree == 100){
           return(NO_ERR);
        }else if (MostorRunPara.CurrentDegree == INVALID_DEGREE){
           time = eep_param.motor_up_time + (eep_param.motor_up_time >> 2);
        }else{
           time = cal_time(100);
           time += eep_param.motor_up_time >> 2;
        }
//        pr_info("OneWay_Simple_Operation => MOTOR_FORWARD time = %d!\n",time);
    }
    if (cmd == MOTOR_REVERSE)
    {
       MostorRunPara.LastDegree = 0;
           if (MostorRunPara.CurrentDegree == 0){
              return(NO_ERR);
           }else if (MostorRunPara.CurrentDegree == INVALID_DEGREE){
              time = eep_param.motor_dn_time + (eep_param.motor_dn_time >> 2);
           }else{
               time = cal_time(0); 
               time += eep_param.motor_dn_time >> 2;
           }
//           pr_info("OneWay_Simple_Operation => MOTOR_REVERSE time = %d!\n",time);
    }

    MostorRunPara.next_action_cmd = 0;
    MostorRunPara.next_action_time = 0;
       
//�ش�״̬
    if ((eep_param.motor_up_time) && (eep_param.motor_dn_time) && (cmd != MOTOR_STOP))
        Resend_report(1);

     ret = Motor_Action_Control(cmd, time);  //�����������
     if(ret) 
       return(DATA_ERR);
     return(NO_ERR);
}


uint8_t Percent_Complex_Operation(uint8_t Percent)
{
    uint8_t ret;
    uint16_t time, cmd, degree;
    MostorRunPara.next_action_cmd = 0;
    MostorRunPara.next_action_time = 0;
    if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0) || Percent > 100) return (DATA_ERR);//�����½���׼ʱ���趨���
    if ((MostorRunPara.curtain_init != 0)) return DEV_BUSY; //�趨��׼ʱ���ʼ��/����ָ���豸�в�������
    if (MostorRunPara.CurrentDegree == INVALID_DEGREE) {
        MostorRunPara.action_cnt = 0;
        //У׼��������
        eep_param.ActionCount = 0;
//        flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
        flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, ActionCount), 
                              (uint8_t *)(&eep_param.ActionCount), 
                              sizeof(eep_param.ActionCount));     //������д��flash�ڱ��棨�Ż���

        MostorRunPara.reset_flag = 0;
        if (!(Percent%100)){
            time =  Percent > 0 ? eep_param.motor_up_time : eep_param.motor_dn_time; 
            time =  time + (time>>2);
            cmd  =  Percent > 0 ? MOTOR_FORWARD : MOTOR_REVERSE ; //����ֵΪ0/100
        }else{
            time = eep_param.motor_dn_time + (eep_param.motor_dn_time >> 2); 
            cmd  = MOTOR_REVERSE;
            MostorRunPara.next_action_cmd = Percent;
            MostorRunPara.CurrentDegree = 0;
            MostorRunPara.next_action_time = cal_time(Percent);
            Percent = 0;//��λ��ʼΪ0
            MostorRunPara.CurrentDegree = INVALID_DEGREE;                      //����ֵ����0/100
        }
    }else if (MostorRunPara.CurrentDegree != INVALID_DEGREE) {    
       cal_degree();
       MostorRunPara.LastDegree = Percent;
       if (!(MostorRunPara.CurrentDegree%100)){
           MostorRunPara.action_cnt = 0;  

           //У׼��������
           eep_param.ActionCount = 0;
//           flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
           flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, ActionCount), 
                                 (uint8_t *)(&eep_param.ActionCount), 
                                 sizeof(eep_param.ActionCount));     //������д��flash�ڱ��棨�Ż���
         
           MostorRunPara.reset_flag = 0;
       }
       MostorRunPara.reset_flag = Percent%100 > 0 ? MostorRunPara.reset_flag : 0;
       time = cal_time(Percent);
//       pr_info("1Percent_Complex_Operation => time = %d !\n",time);
       if (Percent%100 == 0) 
           time = Percent > 0 ? (time + (eep_param.motor_up_time>>2)) : (time + (eep_param.motor_dn_time>>2));
       if (Percent == MostorRunPara.CurrentDegree)     //��ǰ���ȵ���Ŀ�꿪��
       {
           MostorRunPara.LastDegree = Percent;//���濪��
           //ֹͣ����
           Motor_Action_Control(MOTOR_STOP,0);
           Resend_report(1);
//           pr_info("Percent == MostorRunPara.CurrentDegree !\n");
           return (NO_ERR);
       } 
       else if(((Percent < MostorRunPara.CurrentDegree)? MOTOR_REVERSE : MOTOR_FORWARD) == MostorRunPara.Motoe_RuningD){        //Ŀ�궯���뵱�¶���״̬һ��
           if ((MostorRunPara.reset_flag)&& MostorRunPara.Motoe_RuningD == MOTOR_REVERSE){
               time = cal_time(0);
//               pr_info("2Percent_Complex_Operation => Motoe_TargetT = %d !\n",MostorRunPara.Motoe_TargetT);
               cmd = MOTOR_REVERSE;
               time += eep_param.motor_dn_time >> 2;

               MostorRunPara.next_action_cmd = Percent;
               degree = MostorRunPara.CurrentDegree;
               MostorRunPara.LastDegree = 0;
               MostorRunPara.CurrentDegree = 0;
               MostorRunPara.next_action_time = cal_time(Percent);
//               pr_info("3Percent_Complex_Operation => next_action_time = %d !\n",MostorRunPara.next_action_time);
               MostorRunPara.CurrentDegree = degree;
           }
           cmd = ((Percent < MostorRunPara.CurrentDegree)? MOTOR_REVERSE : MOTOR_FORWARD);  
//           pr_info("(MostorRunPara.reset_flag)&& MostorRunPara.Motoe_RuningD == MOTOR_REVERSE\n");
       }else{                                  //Ŀ�궯���뵱�¶���״̬��һ��
           if ((MostorRunPara.reset_flag) && (((Percent < MostorRunPara.CurrentDegree)? MOTOR_REVERSE : MOTOR_FORWARD) == MOTOR_REVERSE)) {
               time = cal_time(0);
//               pr_info("4Percent_Complex_Operation => Motoe_TargetT = %d !\n",MostorRunPara.Motoe_TargetT);
               cmd = MOTOR_REVERSE;
               time += eep_param.motor_dn_time >> 2;

               MostorRunPara.next_action_cmd = Percent;
               degree = MostorRunPara.CurrentDegree;
               MostorRunPara.LastDegree = 0;
               MostorRunPara.CurrentDegree = 0;
               MostorRunPara.next_action_time = cal_time(Percent);
//               pr_info("5Percent_Complex_Operation => next_action_time = %d !\n",MostorRunPara.next_action_time);
               MostorRunPara.CurrentDegree = degree;
               Percent = 0;
           }else{
              cmd = ((Percent < MostorRunPara.CurrentDegree)? MOTOR_REVERSE : MOTOR_FORWARD);      //Ŀ�궯������ҪУ׼
//              pr_info("6Percent_Complex_Operation => cmd ,time = %d,%d !\n",cmd,time);
           }
       }
   }

   Resend_report(1);
   MostorRunPara.LastDegree = Percent;
   ret = Motor_Action_Control(cmd , time);  //�����������
   if (ret) return (DATA_ERR);
   return (NO_ERR);
}

uint8_t Set_StandAct_Time(uint16_t uptime, uint16_t dntime)
{
   uint16_t acttime = 0;
   if ((uptime < 500) || (dntime < 500))    //�������ʱ�䲻����������Ϊ����
       return (DATA_ERR);
   if ((uptime > 12000) || (dntime > 12000))//�������ʱ�����2min��ʶ����
       return (DATA_ERR);
   acttime = dntime + (dntime >> 2);        //����У׼����ʱ��Ϊ�½�ʱ���5/4
//�����½���ʱ��д��ṹ��
   eep_param.motor_up_time = uptime;
   eep_param.motor_dn_time = dntime;
//   pr_info("Set_StandAct_Time => Set uptime = %d, dntime = %d!\n",uptime,dntime);
//������ʱ���½�ʱ��д�뵽�ڴ���
//   flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������ʱ���½�ʱ��д�뵽�ڴ���
   flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, motor_up_time), 
                         (uint8_t *)(&eep_param.motor_up_time), 
                         sizeof(eep_param.motor_up_time) + sizeof(eep_param.motor_dn_time));     //������д��flash�ڱ��棨�Ż���
   curtain_time_init();      //��������У׼
   MostorRunPara.curtain_init = MOTORTIME_INIT;  //�趨ΪУ׼״̬

   MostorRunPara.next_action_cmd = 0;
   MostorRunPara.next_action_time = 0;
   MostorRunPara.action_cnt = 0; 

   //У׼��������
   eep_param.ActionCount = 0;
//   flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
   flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, ActionCount), 
                         (uint8_t *)(&eep_param.ActionCount), 
                         sizeof(eep_param.ActionCount));     //������д��flash�ڱ��棨�Ż���

   MostorRunPara.reset_flag = 0;

   MostorRunPara.LastDegree = 0;
   MostorRunPara.CurrentDegree = 100;

//   Resend_report(1);
//   stage_restart_data_init(REPORT_OVER);
   Motor_Action_Control(MOTOR_REVERSE , acttime);  //�����������
   return NO_ERR;
}


void Set_DevShow_Time()
{
    MostorRunPara.curtain_init = MOTORTIME_SEARCH;
//    if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0)){
        Motor_Action_Control(MOTOR_FORWARD,500);                       //����������� 
//    }else{MOTOR_FORWARD
//        Percent_Complex_Operation(MostorRunPara.CurrentDegree + 10);
//    }
}


void MostorManageInit(void)
{
    msc_Moster_init();
    memcpy(&eep_param, (uint8_t*)EEP_MOTOR_ADDR, sizeof(eep_param)); //��������ȡ������Ϣ
    curtain_time_init();      //��������У׼
    MostorRunPara.LastDegree    = ((eep_param.degree > 100) || eep_param.acting) ? INVALID_DEGREE : eep_param.degree;
    MostorRunPara.CurrentDegree = ((eep_param.degree > 100) || eep_param.acting) ? INVALID_DEGREE : eep_param.degree;
    eep_param.report_enable = eep_param.report_enable < 0x0f ? eep_param.report_enable : 0x03;
    eep_param.BoltCrtal = eep_param.BoltCrtal < 0x0f ? eep_param.BoltCrtal : 0x00;
    eep_param.BoltBoltStatu = eep_param.BoltBoltStatu < 0x0f ? eep_param.BoltBoltStatu : 0x00;
    eep_param.motor_dn_time = eep_param.motor_dn_time == 0xffff ? 0x00 : eep_param.motor_dn_time;
    eep_param.motor_up_time = eep_param.motor_up_time == 0xffff ? 0x00 : eep_param.motor_up_time;
    eep_param.location = eep_param.location == 0xff ? 0x00 : eep_param.location;
    dev_state.ctrl_flag = NO_CTRL_FLAG;//��ʼ���������͸տ���û�п���
//���ģ�Ͳ�����ʼ��
    eep_param.SkyModelContral = eep_param.SkyModelContral > 1 ? 0 : eep_param.SkyModelContral;//���ģ�ͽ�ʹ����Ĭ��0x00
    eep_param.ForceCaliTime   = eep_param.ForceCaliTime == 0xFF ? 30 : eep_param.ForceCaliTime;//ǿ��У׼�����趨Ĭ��30
//���ģ�Ͷ�ʱ�ر��������
    eep_param.SkyModelContral = eep_param.SkyBodyCotrlUper == 0xFF ? 0 : eep_param.SkyBodyCotrlUper;
    eep_param.SkyBodyCotrlCount = eep_param.SkyBodyCotrlCount > 5 ? 0 : eep_param.SkyBodyCotrlCount;
    eep_param.SkyMonitorTime = eep_param.SkyMonitorTime > 71 ? 0 : eep_param.SkyMonitorTime;

    esprintf("\nApplication layer initialization parameters=>\n",eep_param.degree);
    esprintf("MotoResetCount:%d (7 ,10 ,13.D)\n",MostorRunPara.reset_cnt);
    esprintf("MotorUpTime:%d (0.D ,500~12000)\n",eep_param.motor_up_time);
    esprintf("MotorDownTime:%d (0.D ,500~12000)\n",eep_param.motor_dn_time);
    esprintf("BoltStatu:%d (0.Close D ,1.Open)\n",eep_param.BoltBoltStatu);
    esprintf("BoltEnabled:%d (0.Disable D ,1.Enable)\n",eep_param.BoltCrtal);
    esprintf("CurtainOpening:%d (0~100 ,255.Unknown D)\n",eep_param.degree);
    esprintf("CurrentTurn:%d (0.Positive D ,1.Negative)\n",eep_param.location);
    esprintf("ReportEnable:%d (0.NO ,1.GW ,2.SB ,3.GWAndSB D)\n",eep_param.report_enable);
    esprintf("LastMotorAction:%d (0.Still ,1.Action ,225.Unknown D)\n",eep_param.acting);
    msc_Moster_init();
    Resend_report(2);
}


void MostorRealTimeMonitorTask()
{
    if (MotorRealTimetaskList & (1 << MotorResetAct)) {
        pr_info("[ACT-REALTASK]Start Next action after current rest finish.\n");
        MostorRunPara.Motoe_RunT = 0;
        Percent_Complex_Operation(MostorRunPara.next_action_cmd);
        MostorRunPara.next_action_cmd = 0;
        MostorRunPara.next_action_time = 0;
        MostorRunPara.reset_flag = 0;
        MotorRealTimetaskList = MotorRealTimetaskList & (~(1 << MotorResetAct));
    }
    if (MotorRealTimetaskList & (1 << MotorDevShowAct)){
        MostorRunPara.curtain_init = MOTORTIME_SEARCH2;  //�趨Ϊ����״̬
//        if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0)){
            Motor_Action_Control(MOTOR_REVERSE,500);         //����������� 
//        }else{MOTOR_REVERSE
//            Percent_Complex_Operation(MostorRunPara.CurrentDegree - 10);
//        }
        MotorRealTimetaskList = MotorRealTimetaskList | (1 << MotorResetcount);
        MotorRealTimetaskList = MotorRealTimetaskList & (~(1 << MotorDevShowAct));
    }
//    if (MosterTiming != 0xFFFF&&MostorRunPara.Motoe_TargetT)
//    {
//        cal_degree();
//        eep_param.degree = MostorRunPara.CurrentDegree;
//        flash_user_info_write(EEP_MOTOR_ADDR + 9, (uint8_t *)&eep_param.degree, sizeof(eep_param.degree));     //������ʱ���½�ʱ��д�뵽�ڴ���
//    }
}

