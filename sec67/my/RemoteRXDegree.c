#include "RemoteRXDegree.h"
#include "comfunc.h"

///////////////////////////////////////////////////////////////////////// 
///�¼ӱ��� 

//uint8_t g_frame_buffer[MAX_BUFFER_SZ];    	//���ͱ��Ļ���
struct SmartFrame Smart_Send_feame;             //������֡����
uint8_t g_frame_buffer;


///����/////////////////////////////////////////////////////////////////


uint8_t rpt_seq = 0;
struct power_on_report_t power_on_report;
struct stage_data_t stage_data;					//���ݶ�



void stage_data_init()
{
    memset_my((uint8_t *)&stage_data, 0x00, sizeof(stage_data));
    stage_data.counter_for_gw = 10;
}

void stage_restart_data_init(uint8_t state)//5���������������¼�����ʼ
{
   if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0))    //δ���ÿ��ȱ�׼ʱ�䲻ִ���ϱ�
      return;

    if (state==REPORT_START)  //����״̬���ǿ�ʼ�ϱ��򽫴������ȱ仯��־λ����Ϊ�仯
    {
        stage_data.curtain_degree_change = TRUE;
    }
    else    //����״̬�����ǿ�ʼ�ϱ������仯���óɷ��Ŀ�����ΪFREE
    {
        stage_data.curtain_degree_change = FALSE;
        stage_data.state_change_mode = FREE;
    }
    stage_data.gw_send_flag = FALSE;
    stage_data.try_cnt = 0;
    stage_data.aid_position = 0;
    stage_data.counter_for_gw = 10;
}
//��������ն�SAID������
static void insert_said_to_queue(void *s)   
{
    uint8_t i;
    if (0x00 == memcmp_my(eep_param.gateway_id, s, ID_LEN))      //����SAID�뻺��������ʷSAID�Ƚ�����ظ��ж�֮���->?
    {
        return;
    }
    for (i = 0;i < MAX_SB_NUM;i++)                              //������SAID�뻺�������SAID���бȽ�
    {
        if (0x00 == memcmp_my(&stage_data.said_queue[4*i], s, ID_LEN)){
            break;
        }
    }
    if (i >= MAX_SB_NUM)                                        //������������û�е�ǰSAID
    {
        mymemcpy(&stage_data.said_queue[4*stage_data.queue_pos], s, ID_LEN); //���¶����е�SAID
	    stage_data.queue_pos++;                                              //SAID����ɹ������Լ�
        if (stage_data.queue_pos >= MAX_SB_NUM)                              //���������SAIDӦ������
        {
            stage_data.queue_pos = 0;                                        //��λ������ţ����»������
        }
    }
}

uint8_t get_message_source(void *source_addr,uint8_t destination_flag)
{
    if(destination_flag == GATEWAY_NUM){                        //У��Ŀ�ĵ�ַ��Դ������㲥���ߵ��Ե�������ǹ㲥���ͽ��ϴλ����ַ���ó�0��ʾ�㲥��ʽ���ش��㲥
        memset_my(stage_data.last_said, 0x00, ID_LEN);
        return (BROADCAST);
    }
    insert_said_to_queue(source_addr);                          //�ǹ㲥��ʽ������SAID��������н��б������SAID����
    stage_data.equipment_gid = 0;                               //�����豸GIDΪ��ǹ㲥״̬
    mymemcpy(stage_data.last_said,source_addr,ID_LEN);          //���ϴθ���SAID����Ϊ��ǰASID
    return (P2P);
} 

//uint8_t get_1byte_bit1_number(uint8_t data,uint8_t pos)               //��ȡ����ǰXλ��Ϊ���λ��
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
                    uint8_t data[], uint8_t data_len)       //���������ݻ�ȡ������Ч����
{
    struct FBD_Frame *pfbd_frame;

    pfbd_frame = (struct FBD_Frame *)frame_buf;
    pfbd_frame->did[0] = did_l;
    pfbd_frame->did[1] = did_h;

    mymemcpy(&pfbd_frame->data[0], &data[0], data_len);
		
    pfbd_frame->ctrl = data_len;
    return(data_len + FBD_FRAME_HEAD);
}

//��֯���Ļش�
void send_state_to_subscriber(uint8_t aid_position)       //�ش����ݵ�������
{                                    //�������ͱ���pfbd������
    uint8_t len = 0, databuf[1];
    struct SmartFrame *pshs_frame = (struct SmartFrame *)g_frame_buffer;  //�������ͱ��Ļ�����
    struct FBD_Frame *pfbd_frame;   
		
    pshs_frame->stc = STC;           //������ͷ
    mymemcpy(pshs_frame->said, eep_param.id, ID_LEN);   //д�뱾��SAID
    if(aid_position != GATEWAY_NUM)                     //ͨ��SAID������б���ж��Ƿ�Ϊ�㲥���Ŀ���
    {
        if(!(eep_param.report_enable & REPORT_SB))      //SB���Ļش�ʹ�ܼ��
        {
            return;
        }
        mymemcpy(pshs_frame->taid,&stage_data.said_queue[4 * aid_position], ID_LEN);    //���ݻ��������Żش����ĵ�������
    }
    else
    {
        if(!(eep_param.report_enable & REPORT_GW))      //GW���Ļش�����ʹ��
        {
            return;
        }
        mymemcpy(pshs_frame->taid,eep_param.gateway_id,ID_LEN);     //���ͱ���д������SAID
    }
    if(is_all_xx(pshs_frame->taid, 0x00, ID_LEN))           //���д���TAIDΪ����֤��û���豸���Ӳ����к�������
    {
        return;
    }
    rpt_seq++;              //����֡��Ŵ���
    pshs_frame->seq = (rpt_seq & 0x7F);         //д��֡���
    pshs_frame->data[0] = RELIABLE_REPORT;     //�趨Ϊ�ɿ����䱨��
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
    do_send_frame(ram_data.last_ctrl_mac, pshs_frame);//�������ݵ�
}


//�������񷽷�
void stage_hook(void)
{
    uint8_t i,j = 0;
    if(stage_data.gw_send_flag == TRUE) //ȷ���б��ķ���
    {
        return;
    }
    if(stage_data.state_change_mode == FREE)    //ȷ�ϴ��ڱ��ķ��ͷ�ʽ
    {
        return;
    }
    if(!(eep_param.report_enable & REPORT_SB))  //ȷ�ϱ��Ļش��Ѿ�ʹ��
    {
        stage_data.gw_send_flag = TRUE;
        return;
    }
    //�ж��������Ƿ��SAID
    for (i = 0;i < MAX_SB_NUM;i++)
    {
        if (is_all_xx(&stage_data.said_queue[4*i], 0x00, ID_LEN))
        {
            j++;
        }
    }
    if(MAX_SB_NUM == j)//�����������в�����SAID
    {
        stage_data.gw_send_flag = TRUE;
        return;
    }
    send_state_to_subscriber(stage_data.aid_position);//����SAID������н��б�����֯
    stage_data.aid_position ++;     //���·��Ͷ������
    if(stage_data.aid_position >= MAX_SB_NUM)   //���Said��������н�ȫ���ش����
    {
        stage_data.aid_position = 0;
        stage_data.gw_send_flag = TRUE;
    }
}
//???????????????????????????????????????????????????????????????
void check_gw_ack(void)//100ms      //У�鴫��
{
    if(stage_data.gw_send_flag != TRUE) 
    {
        return;
    }
    if(stage_data.state_change_mode == FREE)
    {
        return;
    }

    if(!(eep_param.report_enable & REPORT_GW))//δ��Ȩ
    {
        stage_restart_data_init(REPORT_OVER);
        return;
    }
    if(stage_data.state_change_mode == GATEWAY_ACK)//����ģʽ������
    {
        stage_restart_data_init(REPORT_OVER);
        return;
    }

    if(stage_data.counter_for_gw > 0)           //��������У�����
    {
        stage_data.counter_for_gw--;
        return;
    }
    stage_data.try_cnt++;//�����Ļش����Դ���
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

void org_report_frame(uint8_t report_id[], uint8_t report)//���뱨����֯�ϱ�
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

//���ϴ�������
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


