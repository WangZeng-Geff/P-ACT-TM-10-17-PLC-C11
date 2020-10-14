#include "RemoteRXDegree.h"
#include "comfunc.h"

///////////////////////////////////////////////////////////////////////// 
///新加变量 

//uint8_t g_frame_buffer[MAX_BUFFER_SZ];    	//发送报文缓存
struct SmartFrame Smart_Send_feame;             //发送整帧报文
uint8_t g_frame_buffer;


///结束/////////////////////////////////////////////////////////////////


uint8_t rpt_seq = 0;
struct power_on_report_t power_on_report;
struct stage_data_t stage_data;					//数据段



void stage_data_init()
{
    memset_my((uint8_t *)&stage_data, 0x00, sizeof(stage_data));
    stage_data.counter_for_gw = 10;
}

void stage_restart_data_init(uint8_t state)//5秒内有新任务，重新计数开始
{
   if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0))    //未设置开度标准时间不执行上报
      return;

    if (state==REPORT_START)  //传入状态若是开始上报则将窗帘开度变化标志位设置为变化
    {
        stage_data.curtain_degree_change = TRUE;
    }
    else    //传入状态若不是开始上报则窗帘变化设置成否报文控制设为FREE
    {
        stage_data.curtain_degree_change = FALSE;
        stage_data.state_change_mode = FREE;
    }
    stage_data.gw_send_flag = FALSE;
    stage_data.try_cnt = 0;
    stage_data.aid_position = 0;
    stage_data.counter_for_gw = 10;
}
//插入控制终端SAID到队列
static void insert_said_to_queue(void *s)   
{
    uint8_t i;
    if (0x00 == memcmp_my(eep_param.gateway_id, s, ID_LEN))      //输入SAID与缓存区的历史SAID比较如果重复中断之后操->?
    {
        return;
    }
    for (i = 0;i < MAX_SB_NUM;i++)                              //将输入SAID与缓存队列内SAID进行比较
    {
        if (0x00 == memcmp_my(&stage_data.said_queue[4*i], s, ID_LEN)){
            break;
        }
    }
    if (i >= MAX_SB_NUM)                                        //如果缓存队列内没有当前SAID
    {
        mymemcpy(&stage_data.said_queue[4*stage_data.queue_pos], s, ID_LEN); //更新队列中的SAID
	    stage_data.queue_pos++;                                              //SAID插入成功队列自加
        if (stage_data.queue_pos >= MAX_SB_NUM)                              //如果队列中SAID应将填满
        {
            stage_data.queue_pos = 0;                                        //复位更新序号，重新缓存队列
        }
    }
}

uint8_t get_message_source(void *source_addr,uint8_t destination_flag)
{
    if(destination_flag == GATEWAY_NUM){                        //校验目的地址来源类型如广播或者单对单，如果是广播类型将上次缓存地址设置成0表示广播方式并回传广播
        memset_my(stage_data.last_said, 0x00, ID_LEN);
        return (BROADCAST);
    }
    insert_said_to_queue(source_addr);                          //非广播方式命令则将SAID插入队列中进行保存更新SAID队列
    stage_data.equipment_gid = 0;                               //设置设备GID为零非广播状态
    mymemcpy(stage_data.last_said,source_addr,ID_LEN);          //将上次更新SAID设置为当前ASID
    return (P2P);
} 

//uint8_t get_1byte_bit1_number(uint8_t data,uint8_t pos)               //获取数据前X位不为零的位数
//{
//    uint8_t i;
//    uint8_t k = 0;
//    for (i=0;i<pos;i++)
//    {
//        if ((0x01&data)==0x01)
//        {
//            k++;
//	      }
//        data>>=0x01;
//    }
//    return k;
//}

uint8_t org_fbd_frame(uint8_t frame_buf[], uint8_t did_l, uint8_t did_h,
                    uint8_t data[], uint8_t data_len)       //处理报文数据获取报文有效数据
{
    struct FBD_Frame *pfbd_frame;

    pfbd_frame = (struct FBD_Frame *)frame_buf;
    pfbd_frame->did[0] = did_l;
    pfbd_frame->did[1] = did_h;

    mymemcpy(&pfbd_frame->data[0], &data[0], data_len);
		
    pfbd_frame->ctrl = data_len;
    return(data_len + FBD_FRAME_HEAD);
}

//组织报文回传
void send_state_to_subscriber(uint8_t aid_position)       //回传数据到订阅者
{                                    //关联发送报文pfbd缓存区
    uint8_t len = 0, databuf[1];
    struct SmartFrame *pshs_frame = (struct SmartFrame *)g_frame_buffer;  //关联发送报文缓存区
    struct FBD_Frame *pfbd_frame;   
		
    pshs_frame->stc = STC;           //报文字头
    mymemcpy(pshs_frame->said, eep_param.id, ID_LEN);   //写入本地SAID
    if(aid_position != GATEWAY_NUM)                     //通过SAID缓存队列标号判定是否为广播报文控制
    {
        if(!(eep_param.report_enable & REPORT_SB))      //SB报文回传使能检测
        {
            return;
        }
        mymemcpy(pshs_frame->taid,&stage_data.said_queue[4 * aid_position], ID_LEN);    //根据缓存队列序号回传报文到订阅者
    }
    else
    {
        if(!(eep_param.report_enable & REPORT_GW))      //GW报文回传报文使能
        {
            return;
        }
        mymemcpy(pshs_frame->taid,eep_param.gateway_id,ID_LEN);     //发送报文写入网关SAID
    }
    if(is_all_xx(pshs_frame->taid, 0x00, ID_LEN))           //如果写入的TAID为空则证明没有设备连接不进行后续操作
    {
        return;
    }
    rpt_seq++;              //发送帧序号处理
    pshs_frame->seq = (rpt_seq & 0x7F);         //写入帧序号
    pshs_frame->data[0] = RELIABLE_REPORT;     //设定为可靠传输报文
    pfbd_frame = (struct FBD_Frame *)&pshs_frame->data[1];//

    databuf[0] = MostorRunPara.LastDegree;
    len += org_fbd_frame((uint8_t *)pfbd_frame, did_item(0x0A03), databuf, 1);
    databuf[0] = MostorRunPara.CurrentDegree;
    len += org_fbd_frame((uint8_t *)pfbd_frame + len, did_item(0x0A05), databuf, 1);

    if(GATEWAY_NUM == aid_position)
    {
        len += org_fbd_frame((uint8_t *)pfbd_frame + len, did_item(0xC01A), stage_data.taker_id, 4);
    }
    len += 1;//cmd
    pshs_frame->len = len;
//    pshs_frame->data[len] = checksum((uint8_t *)pshs_frame, len + SHS_FRAME_HEAD);
//    len += SHS_FRAME_HEAD + 1;
//    uart_write(g_frame_buffer, len);
    do_send_frame(ram_data.last_ctrl_mac, pshs_frame);//发送数据到
}


//发送任务方法
void stage_hook(void)
{
    uint8_t i,j = 0;
    if(stage_data.gw_send_flag == TRUE) //确认有报文发送
    {
        return;
    }
    if(stage_data.state_change_mode == FREE)    //确认存在报文发送方式
    {
        return;
    }
    if(!(eep_param.report_enable & REPORT_SB))  //确认报文回传已经使能
    {
        stage_data.gw_send_flag = TRUE;
        return;
    }
    //判定队列中是否存SAID
    for (i = 0;i < MAX_SB_NUM;i++)
    {
        if (is_all_xx(&stage_data.said_queue[4*i], 0x00, ID_LEN))
        {
            j++;
        }
    }
    if(MAX_SB_NUM == j)//如果缓存队列中不存在SAID
    {
        stage_data.gw_send_flag = TRUE;
        return;
    }
    send_state_to_subscriber(stage_data.aid_position);//根据SAID缓存队列进行报文组织
    stage_data.aid_position ++;     //更新发送队列序号
    if(stage_data.aid_position >= MAX_SB_NUM)   //如果Said缓存对列中将全部回传完成
    {
        stage_data.aid_position = 0;
        stage_data.gw_send_flag = TRUE;
    }
}
//???????????????????????????????????????????????????????????????
void check_gw_ack(void)//100ms      //校验传输
{
    if(stage_data.gw_send_flag != TRUE) 
    {
        return;
    }
    if(stage_data.state_change_mode == FREE)
    {
        return;
    }

    if(!(eep_param.report_enable & REPORT_GW))//未授权
    {
        stage_restart_data_init(REPORT_OVER);
        return;
    }
    if(stage_data.state_change_mode == GATEWAY_ACK)//传输模式是网关
    {
        stage_restart_data_init(REPORT_OVER);
        return;
    }

    if(stage_data.counter_for_gw > 0)           //单条报文校验次数
    {
        stage_data.counter_for_gw--;
        return;
    }
    stage_data.try_cnt++;//单向报文回传尝试次数
    if(stage_data.try_cnt == 1)
    {
        stage_data.counter_for_gw = 100;
    }
    if(stage_data.try_cnt == 2)
    {
        stage_data.counter_for_gw = 1000;
    }
    if(stage_data.try_cnt > GATEWAY_MAX_TRY_CNT) 
    {
        stage_restart_data_init(REPORT_OVER);
        return;
    }
    send_state_to_subscriber(GATEWAY_NUM);
}

void check_state_change(void)
{
    if(TRUE == stage_data.curtain_degree_change)
    {
        if(stage_data.wait_cnt > 0)
        {
            stage_data.wait_cnt--;
            return;
        }
        stage_hook();
    }
}

void org_report_frame(uint8_t report_id[], uint8_t report)//单针报文组织上报
{
//    SmartFrame

    struct SmartFrame *pshs_frame = (struct SmartFrame *)g_frame_buffer;
    struct FBD_Frame *pfbd_frame;
    uint8_t len = 0;
    uint8_t data_buff[1];

    pshs_frame->stc = STC;
    power_on_report.report_seq++;
    pshs_frame->seq = (power_on_report.report_seq & 0x7F);

    mymemcpy(pshs_frame->said, eep_param.id, ID_LEN);
    mymemcpy(pshs_frame->taid, report_id, ID_LEN);

    pshs_frame->data[0] = report;
    pfbd_frame = (struct FBD_Frame *)&pshs_frame->data[1];

    data_buff[0] = MostorRunPara.LastDegree;
    len += org_fbd_frame((uint8_t *)pfbd_frame, did_item(0x0A03), data_buff, 1);
    data_buff[0] = MostorRunPara.CurrentDegree;
    len += org_fbd_frame((uint8_t *)pfbd_frame + len, did_item(0x0A05), data_buff, 1);
    len += org_fbd_frame((uint8_t *)pfbd_frame + len, did_item(0xC01A), eep_param.id, 4);

    len += 1;  //cmd
    pshs_frame->len = len;
//    pshs_frame->data[len] = checksum((uint8_t *)pshs_frame, len+SHS_FRAME_HEAD);
//    len += SHS_FRAME_HEAD + 1;//cs
//    uart_write(g_frame_buffer, len);
    do_send_frame(ram_data.last_ctrl_mac, pshs_frame);//?????
}


uint16_t take_small_value_with_100(uint16_t report_freq)
{
   if(0 == report_freq)
   {
      return (100);
   }
   else
   {
      return ((report_freq > 100 ? 100 : report_freq));
   }
}

uint16_t cal_report_delay_time(uint8_t try_count)
{
   power_on_report.device_sid = (eep_param.sid[1] << 8) + eep_param.sid[0];
   return ((uint16_t)(1 + try_count) * (uint16_t)60 + (power_on_report.device_sid % take_small_value_with_100(0)));
}

void power_on_report_over(void)
{
    power_on_report.report_try_cnt = MAX_REPORT_TRY_CNT;
    power_on_report.report_flag = FALSE;
    power_on_report.report_end = REPORT_END;
}
void check_power_on_report_ack(void)
{
    if ((GATEWAY_ACK == stage_data.state_change_mode) || (GATEWAY_ACK == power_on_report.report_ack))
    {
        power_on_report_over();
	return;
    }

    if (TRUE != power_on_report.report_flag) 
    {
        return;
    }

   power_on_report.report_try_cnt++;
   power_on_report.report_flag = FALSE;
   if(power_on_report.report_try_cnt < MAX_REPORT_TRY_CNT)
   {
      power_on_report.report_delay_time = cal_report_delay_time(power_on_report.report_try_cnt);
   }
}

//线上传输任务
void power_on_report_task(void)
{
   if(is_all_xx(eep_param.gateway_id, 0x00, ID_LEN))
      return;

   if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0))
      return;

   org_report_frame(eep_param.gateway_id, RELIABLE_REPORT);

   power_on_report.report_flag = TRUE;
}

void check_power_on(void)
{
   power_on_report.report_counter++;

   if ((power_on_report.report_counter >= power_on_report.report_delay_time)
          && (power_on_report.report_try_cnt < MAX_REPORT_TRY_CNT)
          && (power_on_report.report_end != REPORT_END))
   {
      power_on_report.report_counter = 0;
      power_on_report_task();
   }
}


void poweron_report_init(void)
{
    memset_my(&power_on_report, 0x00, sizeof(power_on_report));
    power_on_report.report_delay_time = cal_report_delay_time(power_on_report.report_try_cnt);
}


