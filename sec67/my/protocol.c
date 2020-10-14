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
#include "vsh.h"
#include "cmd.h"

struct DEV_DATA dev_data;
struct DEV_STATE dev_state;
struct DEV_STATE rx_uart;
struct Report report;

struct CTRL_MEG Current_ctrl_meg;

uint8_t InsurancePlugStatusReturn = 0;       //����Ƿ�ش�����˨״̬
uint8_t BroadcastControl = 0;       //����Ƿ�Ϊ�㲥����ģʽ0����1��:

//---------------------------------���Թ�װ����------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//����aid
static int on_set_aid(uint8_t *data, int len)
{
    if (len != AID_LEN) return -1;
	
    if (!memcmp(repeater.aid, data, sizeof(repeater.aid))) 
    {
    	data[0] = CMD_NAK;
        return 1;
    }
    memcpy(repeater.aid, data, sizeof(repeater.aid));
    data[0] = CMD_ACK;
    return 1;
}
static int on_get_aid(uint8_t *data, int len)
{
    if (is_all_xx(repeater.aid, 0, sizeof(repeater.aid))) 
    {
        data[0] = CMD_NAK;
        return 1;
    }

    data[0] = CMD_ACK_AID;
    memcpy(&data[1], repeater.aid, sizeof(repeater.aid));
    return 1 + sizeof(repeater.aid);
}//---------------------------------------------------------------------------------------
static int on_get_eid(uint8_t *data, int len)
{
    sdk_svc_mac_t mac;
    sdk_err_t err = sdk_svc_get_mac(&mac);
    if (err != SDK_ERR_OK)
    {
    	pr_info("[LOCALDID-GETEID]got svc mac failed, err[%d].\n", err);
        return -1;
    }

    data[0] = CMD_ACK_EID;
    memcpy(&data[1], repeater.mac, sizeof(mac.mac));
    memset(&data[1+sizeof(repeater.mac)], 0, 2);
    return 9;
}

//---------------------------------------------------------------------------------------
static const struct LocalOps ops[] = 
{
    {"SET AID", CMD_SET_AID, on_set_aid},
    {"GET AID", CMD_GET_AID, on_get_aid},
    {"GET EID", CMD_GET_EID, on_get_eid},
};

static const struct LocalOps *get_ops(int cmd)
{
    int i;

    for (i = 0; i < array_size(ops); ++i)
    {
        if (ops[i].cmd == cmd) 
        {
            return &ops[i];
        }
    }
    return NULL;
}
static int do_local_frame(struct SmartFrame *pframe)
{
    const struct LocalOps *ops = get_ops(pframe->data[0]);
    if (!ops)
    {
        pr_info("[PROGRESS-LOCALFRAM]not find cmd, cmd[%d].\n", pframe->data[0]);
        return -1;
    }

    pr_info("[PROGRESS-LOCALFRAM]Local Frame[%s].\n", ops->name); 
    print_debug_array(KERN_INFO, pframe, frame_len(pframe));
    int ret = ops->handle(&pframe->data[1], pframe->len-1);
    if (ret > 0)
    {
        ret = code_local_frame(&pframe->data[1], ret, pframe, 0x100);//��֯һ����Ϣͨ�ű��� ������λ��pframe->data[1]��ʼ����Ϊret�����pframe����󳤶�0x100
        esuart_send_data(UART_CHN_DM, (uint8_t*)pframe, ret);
    }
    return 0;
}

//---------------------------------------------------------------------------------------
int smart_frame_handle(struct SmartFrame *frame)
{
    if (is_all_xx(frame->said, 0x00, AID_LEN) && is_all_xx(frame->taid, 0x00, AID_LEN))
    {
        return do_local_frame(frame);
    }
		return 0;
}

int adaptor_on_uart1_received(const uint8_t *data, int len)
{
    struct SmartFrame *frame = get_smart_frame(data, len);
    if (frame)
    {
        len = ((const uint8_t *)frame - data) + frame_len(frame);
        smart_frame_handle(frame);
        return len;
    }

    return 0;
}



//------------------------------������ͨ�ű���--------------------------------------------
//---------------------------------------------------------------------------------------

/// <summary> 
/// ���¶�����AID�б� 
/// </summary> <param name="aid"></param> <param 
/// name="mac"></param> <returns></returns> 
static void add_last_aid(uint8_t *aid, uint8_t *mac)
{
	if(!memcmp(repeater.gid, aid, AID_LEN))
		return;
	int i;
//���浱ǰ���ƶ˵�AID��MAC
    memset(Current_ctrl_meg.aid, 0x00, AID_LEN);
    memset(Current_ctrl_meg.mac, 0x00, MAC_LEN);
    memcpy(Current_ctrl_meg.aid, aid, AID_LEN);
	memcpy(Current_ctrl_meg.mac, mac, MAC_LEN);
//���������߻����б�Ա��Ƿ��б��д��ڴ˶����ߵ�ַ����������ڽ������±������
	for(i = 0; i < MAX_CTRL_NUM; i++)
	{
		if(!memcmp(ram_data.ctrl_meg[i].aid, aid, AID_LEN))
			return;		
	}

	for(i = 0; i < MAX_CTRL_NUM-1; i++)//����һ������Ķ����߼�������������������ǰ��
	{
		memcpy(ram_data.ctrl_meg[i].aid, ram_data.ctrl_meg[i+1].aid, AID_LEN);
		memcpy(ram_data.ctrl_meg[i].mac, ram_data.ctrl_meg[i+1].mac, MAC_LEN);
	}
    //д�����µĶ����ߵ����ĵ�ַ
	memcpy(ram_data.ctrl_meg[i].aid, aid, AID_LEN);
	memcpy(ram_data.ctrl_meg[i].mac, mac, MAC_LEN);
}
//---------------------------------------------------------------------------------------
//��ȡһ�������Ľ��ձ���(��ȡ������)
struct SmartFrame *get_smart_frame(const uint8_t *in, int len)
{
    int i = 0;

 start_lbl:
    while (i < len)//�ڻ���������Ѱ�ұ�����ͷ
    {
        if (STC == in[i])
            break;
        i++;
    }
    //���������ͷʣ�����ֽڳ���С����̱��ĸ�ʽ������������У��ͣ����ȱ�ʾ����������ֱ���˳�����
    if (len - i < SMART_FRAME_HEAD)
        return NULL;
    //��ʼ��һ�����Ľṹ��ʵ���������������ӱ�����ͷλ�ó�ʼ��ַ��ֵ����ʵ��
    struct SmartFrame *pframe = (struct SmartFrame *)&in[i];
    int dlen = pframe->len;//��ȡ�ñ��ĵ������򳤶�
    if (i + SMART_FRAME_HEAD + dlen + 1 > len)//����������ʣ�೤�ȣ�������ά�ֵ�ǰ���������������
    {
        i++;
        goto start_lbl;
//				return NULL;
    }
    //���鱨��У��Ͳ���
    if (pframe->data[dlen] != checksum(pframe, dlen + SMART_FRAME_HEAD))
    {
        i++;
        goto start_lbl;
    }
    pframe = (struct SmartFrame *)&in[i];//�ٴν���������һ֡�����ı����ڻ�������ʼ��ַ��ֵ������
    return pframe;
}
//---------------------------------------------------------------------------------------
//��֯������
//src->Դ��ַ;dest->Ŀ�ĵ�ַ;seq->֡���;cmd->������;data->������;len->���ݳ���;out->���λ��;maxlen->��󳤶�
int code_frame(const uint8_t *src, const uint8_t *dest, int seq, int cmd, 
    const uint8_t *data, int len, void *out, int maxlen)
{
    const uint8_t addr[AID_LEN] = {0x00, 0x00, 0x00, 0x00};//����һ��̶���ַ

    static uint8_t _seq = 0;                //���������֡���
    struct SmartFrame *pframe = (struct SmartFrame *)out;//����һ�����Ĳ���ʵ���������λ���໥����

    pframe->stc = STC;                       //���屨����ͷ
    //�����벢����õ�SAID��TAIDд�뱨�ĵ���   
    if (!src) src = addr;                    //��������SRC��AIDΪ����SAID�ó�0
    if (!dest) dest = addr;                  //��������destΪ����TAID�ó�0
    memcpy(pframe->said, src, AID_LEN);      
    memcpy(pframe->taid, dest, AID_LEN);
    pframe->seq = seq < 0 ? (_seq++ & 0x7F) : seq;//֡��Ŵ�����������
    pframe->data[0] = cmd;               //�������һ���ֽ�(bit7-bit4)������bit3Ϊ1��ʾ��2���ֽڷ�֡��Ϣ��Ϊ0ʱ��û�з�֡��Ϣ����bit2-bit0���� 010��ʾ��ѯ�� 111��ʾ���ã�
    memcpy(&pframe->data[1], data, len); //������д������λ��Ϣ
    pframe->len = len+1;        //��ֻ�����򳤶�
    pframe->data[pframe->len] = checksum(pframe, SMART_FRAME_HEAD + pframe->len);//����У���
    return SMART_FRAME_HEAD + pframe->len + 1;//�ش������ܳ���
}
//---------------------------------------------------------------------------------------
//��֯һ������ͨ�ű��� in->�������ݵ�ַ��len->�����򳤶� out->�����ַ maxlen->��󳤶�
int code_local_frame(const uint8_t *in, int len, void *out, int maxlen)
{
    return code_frame(NULL, NULL, 0, in[0], &in[1], len-1, out, maxlen);
}
//---------------------------------------------------------------------------------------
//��֯һ���ظ�ȷ�ϵı���
int code_ret_frame(struct SmartFrame *pframe, int len)
{
    //Դ֮��Ŀ�ĵ�ַ����
    memcpy(pframe->taid, pframe->said, AID_LEN); 
    memcpy(pframe->said, repeater.aid, AID_LEN); 
    pframe->seq |= 0x80;//�ı�֡���Ϊ�ظ�����
    pframe->len = len;  //��д���ĳ���
    pframe->data[len] = checksum(pframe, pframe->len + SMART_FRAME_HEAD);//���Ķ�У��ͽ���У��
    return pframe->len + SMART_FRAME_HEAD + 1;//�ش����µ������򳤶�
}
//��֯�����򵥸���֡Ӧ�ò�FBD�������
//did->��֡������did err->������ data->������ out->���λ�� maxlen->��󳤶�
int code_body(uint16_t did, int err, const void *data, int len, void *out, int maxlen)
{
    struct Body *body = (struct Body *)out;

    put_le_val(did, body->did, sizeof(body->did));
    body->ctrl = get_bits(len, 0, 6);
    if (err) body->ctrl |= 0x80;
    memcpy(body->data, data, len);

    return sizeof(struct Body) + len;
}
//---------------------------------------------------------------------------------------
//������ձ����Ƿ����������ڴ��豸
int is_my_smart_frame(const struct SmartFrame *frame)
{
    //�������δ��дAID�򲻽��в���������
    if (is_all_xx(repeater.aid, 0x00, AID_LEN))
    {
        pr_info("[SYSTEM-ISMY]I do not have aid, drop it.\n");
        return 0;
    }
    //������ձ��Ĳ��ǹ㲥���Ĳ��ұ��ĵ�Ŀ�ĵ�ַ�ڱ���Դ��ַ��ͬ������
    if (!is_all_xx(frame->taid, 0xFF, AID_LEN) && memcmp(frame->taid, repeater.aid, AID_LEN)) 
    {
        return 0;
    }
    return 1;
}
//---------------------------------------------------------------------------------------
//�б��ض����²��õ�DID
static int is_lowpw_frame(struct SmartFrame *pframe)
{
    int i;
    const uint16_t low_did[] = {0x0004, 0x0603, 0x0604, 0x0605, 0x0606, 0x0607, 0x0608, 0x060A, 0x060C};//�����ض�DID
    struct AppFrame *app = (struct AppFrame *)pframe->data;

	if (CMD_SHOW == app->cmd)//������Ϊ��ʾ�����1
	{
		return 1;
	}
	else
	{
        //��ǰ����did����д����ڶ����б�����ͬ���������1
		uint16_t did = get_le_val(((struct Body *)app->data)->did, DID_LEN);
		for (i = 0; i < array_size(low_did); i++)
		{
			if (did == low_did[i])
				return 1;
		}
	}
	return 0;
}
//---------------------------------------------------------------------------------------
//����һ֡�ز�����
int do_send_frame(uint8_t *mac, struct SmartFrame *frame)
{
    int key_mod = is_lowpw_frame(frame) ? SDK_KEY_INIT : SDK_KEY_USER;//���ݱ�����did�����ж�ʹ�ó�ʼ����Կ�����û���Կ
    if (is_all_xx(frame->taid, 0xFF, AID_LEN))  //�жϵ�ǰ�����Ƿ��ǹ㲥����
    {
        return do_plc_broadcast(key_mod, frame, frame_len(frame));//ͨ���㲥��ʽ�ش�
    }
    
    return do_plc_unicast(mac, key_mod, frame, frame_len(frame));//ͨ��������ʽ�ش�
}
//---------------------------------------------------------------------------------------
//�����豸
static void reboot_tmr_handle(ULONG arg)
{
    sdk_svc_reset();
}
//---------------------------------------------------------------------------------------
//�豸��λdid�б����
static int do_reboot(const uint8_t *src, struct Body *body)
{
    uint32_t wait = 2000; //default 2s
    if (get_bits(body->ctrl, 0, 6) == 2) 
        wait = get_le_val(body->data, 2); 

    estimer_t reboot_tmr = timer_create(wait/10, 0, TMR_RUNNING, TMR_TRIGGER, 
        reboot_tmr_handle, NULL, "reboot_tmr");
    if (!reboot_tmr)
    {
        pr_info("[DIDSET-0601]creat bps timer failed.\n");
        return -OTHER_ERR;
    }
    return NO_ERR;
}
//---------------------------------------------------------------------------------------
//��ȡ�豸�汾did�б����
static int do_get_dev_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_dev_ver();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
//��ȡ�豸�ز��汾did�б����
static int do_get_plc_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_plc_ver();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
//��ȡ�豸����did�б����
static int do_get_dev_type(const uint8_t *src, struct Body *body)
{
	return get_dev_type(body->data);
}
//---------------------------------------------------------------------------------------
//��ȡӦ�ò�Э�鼰�汾did�б����
static int do_get_comm_ver(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, comm_protocol_ver); 
    return strlen(comm_protocol_ver);
}
//---------------------------------------------------------------------------------------
//��ȡ��������ϱ���did�б����
static int do_get_plc_type(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, repeater_plc_type); 
    return strlen(repeater_plc_type);
}
//---------------------------------------------------------------------------------------
//��֯һ���ش����صı��Ĳ�����������д�ز�оƬע����Ϣʹ�ã�
static void do_notify_link(const uint8_t *dst, int unlink)
{
    uint8_t reg[AID_LEN+SID_LEN];
    memcpy(&reg[0], repeater.aid, sizeof(repeater.aid));
    memcpy(&reg[AID_LEN], repeater.sid, sizeof(repeater.sid)); 

    uint8_t body[0x100];
    int len = code_body(0x0603, 0, reg, sizeof(reg), body, sizeof(body));
    if (unlink) ((struct Body *)body)->ctrl |= 0x80;

    uint8_t tmp[0x100];
    len = code_frame(repeater.aid, repeater.gid, -1, CMD_SET, body, len, tmp, sizeof(tmp));

    do_plc_unicast(dst, SDK_KEY_INIT, tmp, len);
}
//---------------------------------------------------------------------------------------
//д�ز�оƬע����Ϣ
static int do_reg(const uint8_t *src, struct Body *body)
{
	struct deviceCfgInfo cfg;
    if (get_bits(body->ctrl, 0, 6) != sizeof(struct RegData)) 
        return -LEN_ERR; 

    struct RegData *reg = (struct RegData *)body->data; 
    if (is_all_xx(repeater.aid, 0x00, AID_LEN) || memcmp(repeater.aid, reg->aid, AID_LEN))
        return -OTHER_ERR;
    int unlink = 1;
    if(!memcmp(repeater.passwd, reg->pw, PW_LEN))
    {
    	memcpy(repeater.panid, reg->panid, PANID_LEN);
    	memcpy(repeater.gid, reg->gid, AID_LEN);
    	memcpy(repeater.sid, reg->sid, SID_LEN);
    	memcpy(repeater.gmac, src, MAC_LEN);
    	do_set_psk();
    	LED_ON(LED_NET);
    	cfg.magic = DEVICE_CFG_MAGIC;
    	memcpy(cfg.gid, repeater.gid, AID_LEN);
    	memcpy(cfg.gmac, repeater.gmac, MAC_LEN);
    	memcpy(cfg.panid, repeater.panid, PANID_LEN);
    	memcpy(cfg.sid, repeater.sid, SID_LEN);
    	flash_user_info_write(DEVICE_CFG_ADDR, (uint8_t*)&cfg, sizeof(cfg));
    	unlink = 0;
    }
    do_notify_link(src, unlink);
    return -NO_RETURN;
}
//---------------------------------------------------------------------------------------
static int do_notify_aid(const uint8_t *src, struct Body *body)
{
    return -NO_RETURN;
}
//---------------------------------------------------------------------------------------
//��ȡ��ǰ�豸AID
static int do_get_aid(const uint8_t *src, struct Body *body)
{
    if (is_all_xx(repeater.aid, 0, AID_LEN) || memcmp(body->data, repeater.aid, AID_LEN)) 
        return -NO_RETURN;
    memcpy(body->data, repeater.aid, AID_LEN);
    return AID_LEN;
}
//---------------------------------------------------------------------------------------
//��ȡ�����汾��
static int do_get_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_dev_ver1();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
//��ȡ�ز��汾��
static int do_get_ext_ver(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, repeater_ext_ver); 
    reverse(body->data, strlen(repeater_ext_ver));
    return strlen(repeater_ext_ver);
}
//---------------------------------------------------------------------------------------
//��DKEY��Ϣ
static int do_get_dev_key(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.dkey, DKEY_LEN);
	return DKEY_LEN;
}
//---------------------------------------------------------------------------------------
//��SN
static int do_get_dev_sn(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.sn, SN_LEN);
	return SN_LEN;
}
//---------------------------------------------------------------------------------------
//��PWD
static int do_get_pwd(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.passwd, PW_LEN);
	return PW_LEN;
}
//---------------------------------------------------------------------------------------
//�����豸�豸������Ӧ
static int do_set_dev_show(const uint8_t *src, struct Body *body)
{
   if (body->ctrl != 0x00)
   {
	   return -DATA_ERR;
   }
    Set_DevShow_Time();
	return NO_ERR;
}
//---------------------------------------------------------------------------------------
//���þ�Ĭʱ��
static int do_set_silent_time(const uint8_t *src, struct Body *body)
{
   if (body->ctrl != 0x02)
   {
	   return -DATA_ERR;
   }
   ram_data.silent_time = sec_jiffies + get_le_val(body->data, TIME_LEN) * SEC_HZ;

   return NO_ERR;
}
//---------------------------------------------------------------------------------------
//��ȡ�豸��ǰת��
static int do_get_dev_location(const uint8_t *src, struct Body *body)
{
	body->data[0] = eep_param.location;
	return(1);
}
//---------------------------------------------------------------------------------------
//�����豸ת��
static int do_set_dev_location(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
    {
        return -DATA_ERR;
    }
	if(eep_param.location != body->data[1])
	{
        eep_param.location = body->data[0];
        flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������ʱ���½�ʱ��д�뵽�ڴ���
        flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, location), 
                              (uint8_t *)(&eep_param.location), 
                              sizeof(eep_param.location));     //������д��flash�ڱ��棨�Ż���
	}
    return NO_ERR;
}
//---------------------------------------------------------------------------------------
//DID 0A01 1.stop 2.open 3.close ���ݾ��豸������LCD���豸
static int do_dev_control_compatible(const uint8_t *src, struct Body *body)	
{
	uint8_t rx_val = 0;
    InsurancePlugStatusReturn = 0;       //����Ƿ�ش�����˨״̬
	if(body->ctrl != 0x04)
	{
		return (-DATA_ERR);
	}
	rx_val = (body->data[1] & 0x0F);
	if(rx_val < 0x01 || rx_val > 0x03)
	{
		return (-DATA_ERR);
	} 
    if (MostorRunPara.curtain_init != MOTORTIME_RUNING) {
		return (-DEV_BUSY);
    }
	if(rx_val == STOP && dev_state.motor_state == STOP)
	{
		return (NO_ERR);
	}
	
    dev_state.ctrl_flag = NO_CTRL_FLAG;									//identify controlor
	if(memcmp(ram_data.last_ctrl_aid, repeater.gid, AID_LEN) == 0)
	{
		dev_state.ctrl_flag = GW_CTRL_FLAG;
	}
	else
	{
		dev_state.ctrl_flag = SUB_CTRL_FLAG;
		memcpy(ram_data.last_ctrl_mac, src, MAC_LEN);
	}

    if(eep_param.BoltCrtal
       && eep_param.BoltBoltStatu
       && (dev_state.ctrl_flag == GW_CTRL_FLAG ||1 == BroadcastControl)){
        pr_info("[DIDSET-0A01]Bolt Crtal is open.\n");
        InsurancePlugStatusReturn = 1;       //����Ƿ�ش�����˨״̬
        Resend_report(0);
        return (-ENLOCK_ERR);
    }
	
    OneWay_Simple_Operation(rx_val, 12000);
	return (NO_ERR);

}
//---------------------------------------------------------------------------------------
//DID 0A04   1.stop 2.open 3.close
static int do_dev_control(const uint8_t *src, struct Body *body)	
{
    InsurancePlugStatusReturn = 0;       //����Ƿ�ش�����˨״̬

	if(body->ctrl != 0x01)
	{
		return (-DATA_ERR);
	}
	if(body->data[0] < 0x01 || body->data[0] > 0x03)
	{
		return (-DATA_ERR);
	} 
    if (MostorRunPara.curtain_init != MOTORTIME_RUNING) {
		return (-DEV_BUSY);
    }
	if(body->data[0] == STOP && MostorRunPara.Motoe_RuningD == MOTOR_STOP)
	{
		return (NO_ERR);
	}
    dev_state.ctrl_flag = NO_CTRL_FLAG;									//identify controlor
	if(memcmp(ram_data.last_ctrl_aid, repeater.gid, AID_LEN) == 0)
	{
		dev_state.ctrl_flag = GW_CTRL_FLAG;
	}
	else
	{
		dev_state.ctrl_flag = SUB_CTRL_FLAG;
		memcpy(ram_data.last_ctrl_mac, src, MAC_LEN);
	}
    if(eep_param.BoltCrtal
       && eep_param.BoltBoltStatu
       && (dev_state.ctrl_flag == GW_CTRL_FLAG ||1 == BroadcastControl)){
        pr_info("[DIDSET-0A04]Bolt Crtal is open.\n");
        InsurancePlugStatusReturn = 1;       //����Ƿ�ش�����˨״̬
        Resend_report(0);
        return (-ENLOCK_ERR);
    }

	OneWay_Simple_Operation(body->data[0], 12000);
	return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//��ȡ�豸����ֵ
static int do_get_open_percent(const uint8_t *src, struct Body *body)
{
	body->data[0] = MostorRunPara.CurrentDegree;
	return (1);
}
//�����豸����ֵ
static int do_set_open_percent(const uint8_t *src, struct Body *body)
{
    InsurancePlugStatusReturn = 0;       //����Ƿ�ش�����˨״̬

	if(body->ctrl != 0x01)
	{
		return (-DATA_ERR);
	}
	if(body->data[0] > 100)
	{
		return (-DATA_ERR);
	}
    if (MostorRunPara.curtain_init != MOTORTIME_RUNING) {
		return (-DEV_BUSY);
    }
	dev_state.ctrl_flag = NO_CTRL_FLAG;								//identify controlor
	if(memcmp(ram_data.last_ctrl_aid, repeater.gid, AID_LEN) == 0)
	{
		dev_state.ctrl_flag = GW_CTRL_FLAG;
	}
	else
	{
		dev_state.ctrl_flag = SUB_CTRL_FLAG;
		memcpy(ram_data.last_ctrl_mac, src, MAC_LEN);
	}
    
    if(eep_param.BoltCrtal
       && eep_param.BoltBoltStatu
       && (dev_state.ctrl_flag == GW_CTRL_FLAG ||1 == BroadcastControl)){
        pr_info("[DIDSET-0A04]Bolt Crtal is open.\n");
        InsurancePlugStatusReturn = 1;       //����Ƿ�ش�����˨״̬
        Resend_report(0);
        return (-ENLOCK_ERR);
    }
	MostorRunPara.TargetDegree = body->data[0];
    if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0) || MostorRunPara.TargetDegree > 100) return (-DATA_ERR);//�����½���׼ʱ���趨���
    Percent_Complex_Operation(MostorRunPara.TargetDegree);
	return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//��ȡ�豸�г�ʱ��
static int get_motor_time(const uint8_t *src, struct Body *body)
{
   body->data[1] = (eep_param.motor_up_time >> 8) & 0xff;
   body->data[0] = eep_param.motor_up_time & 0xFF;
   body->data[3] = (eep_param.motor_dn_time >> 8) & 0xff;
   body->data[2] = eep_param.motor_dn_time & 0xFF;
   return (4);
}
//�����豸�г�ʱ��
static int set_motor_time(const uint8_t *src, struct Body *body)
{
    uint16_t uptime, dntime;
    InsurancePlugStatusReturn = 0;       //����Ƿ�ش�����˨״̬
    if(body->ctrl != 0x04) return (-DATA_ERR);
    dev_state.ctrl_flag = NO_CTRL_FLAG;									//identify controlor
	if(memcmp(ram_data.last_ctrl_aid, repeater.gid, AID_LEN) == 0)
	{
		dev_state.ctrl_flag = GW_CTRL_FLAG;
	}else{
		dev_state.ctrl_flag = SUB_CTRL_FLAG;
		memcpy(ram_data.last_ctrl_mac, src, MAC_LEN);
	}
    if(eep_param.BoltCrtal
       && eep_param.BoltBoltStatu){//|�����г�ʱ�䵥������ҲҪ�ܵ�Ӱ��
        pr_info("[DIDSET-0A02]Bolt Crtal is open.\n");
        InsurancePlugStatusReturn = 1;       //����Ƿ�ش�����˨״̬
        Resend_report(0);
        return (-ENLOCK_ERR);
    }
    uptime = body->data[1] * 256 + body->data[0];
    dntime = body->data[3] * 256 + body->data[2];
    if ((uptime < 500) || (dntime < 500))    //�������ʱ�䲻����������Ϊ����
       return (-DATA_ERR);
    if ((uptime > 12000) || (dntime > 12000))//�������ʱ�����2min��ʶ����
       return (-DATA_ERR);
    Set_StandAct_Time(uptime, dntime);

    return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//��ȡ�豸��ǰ�ϱ�����
static int get_report_enable_infor(const uint8_t *src, struct Body *body)
{
   body->data[0] = 0x00;
   body->data[1] = eep_param.report_enable;
   return (2);
}
//�����豸��ǰ�ϱ�����0:�������ϱ�1:ֻ�ϱ�����2:ֻ�ϱ�������3:���ض����߶��ϱ�
static int set_report_enable_infor(const uint8_t *src, struct Body *body)
{
   if (body->ctrl != 0x02 || body->data[1] > 3) return (-DATA_ERR);
   eep_param.report_enable = body->data[1];
   flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
   flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, report_enable), 
                         (uint8_t *)(&eep_param.report_enable), 
                         sizeof(eep_param.report_enable));     //������д��flash�ڱ��棨�Ż���

   return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//���ô��Ű�ȫ˨ʹ��״̬0:���ý���1:����ʹ��
static int do_set_bolt_cotral(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
    {
        return -DATA_ERR;
    }
    pr_info("[DIDSET-0A30] LastBoltCrtal:%d And CurrBoltCrtal:%d.\n",eep_param.BoltCrtal,body->data[0]);//��ӡ��ǰ�͸��ĺ󴰴�ʹ��״̬
    eep_param.BoltCrtal = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, BoltCrtal), 
                          (uint8_t *)(&eep_param.BoltCrtal), 
                          sizeof(eep_param.BoltCrtal));     //������д��flash�ڱ��棨�Ż���
    return (NO_ERR);
}
//��ȡ��ǰ�豸����ʹ��״̬
static int do_get_bolt_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.BoltCrtal;
    return (1);
}

//---------------------------------------------------------------------------------------
//���õ�ǰ���Ŷ���״̬
static int do_set_bolt_statu(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
    {
        return -DATA_ERR;
    }
    pr_info("[didSET-0A06] lastBoltStatu:%d And CurrBoltStatu:%d.\n",eep_param.BoltBoltStatu,body->data[0]);//��ӡ��ǰ�͸��ĺ󴰴�ʹ��״̬
    eep_param.BoltBoltStatu = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, BoltBoltStatu), 
                          (uint8_t *)(&eep_param.BoltBoltStatu), 
                          sizeof(eep_param.BoltBoltStatu));     //������д��flash�ڱ��棨�Ż���
    return (NO_ERR);
}
//��ȡ��ǰ���Ŷ���״̬
static int do_get_bolt_statu(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.BoltBoltStatu;
    return (1);
}

//---------------------------------------------------------------------------------------
//-------------------------------���ģ�Ϳ��Ʋ���-----------------------------------------
///���ģ��״̬��д
static int do_set_SkyModel_cotral(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
        return -DATA_ERR;
    eep_param.SkyModelContral = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, SkyModelContral), 
                          (uint8_t *)(&eep_param.SkyModelContral), 
                          sizeof(eep_param.SkyModelContral));     //������д��flash�ڱ��棨�Ż���
    return (NO_ERR);
}
//---------------------------------------------------------------------------------------
static int do_get_SkyModel_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.SkyModelContral;
    return (1);
}
///ǿ��У׼������д 
static int do_set_ForceCaliTime_cotral(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01)
        return -DATA_ERR;
    eep_param.ForceCaliTime = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, ForceCaliTime), 
                          (uint8_t *)(&eep_param.ForceCaliTime), 
                          sizeof(eep_param.ForceCaliTime));     //������д��flash�ڱ��棨�Ż���
    return (NO_ERR);
}
//---------------------------------------------------------------------------------------
static int do_get_ForceCaliTime_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.ForceCaliTime;
    return (1);
}
///У׼����������
static int do_get_ActionCount_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.ActionCount;
    return (1);
}
//���ģ�͸��ݼ���˹������ر����ģ��
static void Skymodel_BodyControl_count(uint8_t aid ,int did)
{
    //������ģ��δ����
    if (0x00 == eep_param.SkyModelContral) 
        return ;
    //���Դ��ַ�����ص�ַ���������²���
    if (!memcmp(repeater.gid, aid, AID_LEN))
		return;
    //���DID�����豸����ָ���˳���ǰ����
    if ((0x0A01 != did) && (0x0A03 != did) && (0x0A04 != did))
        return ;
    eep_param.SkyBodyCotrlCount++;
    if (eep_param.SkyBodyCotrlCount > eep_param.SkyBodyCotrlUper) {//�˹����Ƴ�����
        eep_param.SkyModelContral = 0x00;//�������ģ��
    }
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //������д��flash�ڱ���
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, SkyBodyCotrlCount), 
                          (uint8_t *)(&eep_param.SkyBodyCotrlCount), 
                          sizeof(eep_param.SkyBodyCotrlCount) + sizeof(eep_param.SkyBodyCotrlCount));     //������д��flash�ڱ��棨�Ż���
}


enum
{
    HCTR_LOWPW = 1 << 0,
};

struct func_ops
{
    int did;
    const char *tip;
    uint32_t ctrl;
    int (*read)  (const uint8_t *src, struct Body *body);
    int (*write) (const uint8_t *src, struct Body *body);
    int (*read_resp)  (const uint8_t *src, struct Body *body);
    int (*write_resp) (const uint8_t *src, struct Body *body);
};
const struct func_ops func_items[] = 
{
    {0x0601, "Reboot" ,    0    ,NULL, do_reboot, NULL, NULL},
    {0x0603, "Register",   0    ,NULL, do_reg, NULL, NULL},
    {0x0604, "Notify Aid", 0    ,NULL, do_notify_aid, NULL, NULL},
    {0x0605, "Get Aid",    0    ,do_get_aid,	   NULL, NULL, NULL},
    {0x0609, "GetPLCType", 0    ,do_get_plc_type, NULL, NULL, NULL},
    {0x060A, "GetPLCVer",  0    ,do_get_plc_ver,  NULL, NULL, NULL},
    {0x0004, "GetExtVer",  0    ,do_get_ext_ver,  NULL, NULL, NULL},
    {0x0606, "GetIntVer",  0    ,do_get_ver, NULL, NULL, NULL},
//�Ż����²���
//�����Ż������
    {0x0001, "DevType",	   0    ,do_get_dev_type, NULL, NULL, NULL},
    {0x0002, "ComProVer",  0    ,do_get_comm_ver, NULL, NULL, NULL},
    {0x0003, "DevVersion", 0    ,do_get_dev_ver,  NULL, NULL, NULL},
    {0x0005, "DevKey", 	   0    ,do_get_dev_key,  NULL, NULL, NULL},
    {0x0007, "DevSn",      0    ,do_get_dev_sn,   NULL, NULL, NULL},


    {0x0A03, "OpenPercent",0    ,do_get_open_percent     ,do_set_open_percent        ,NULL ,NULL}, 		//���϶�����/��ȡ
    {0x0A04, "DevContrl"  ,0    ,NULL                    ,do_dev_control             ,NULL ,NULL},		//��������/��/ͣ
	{0x0A01, "DevContrl"  ,0    ,NULL                    ,do_dev_control_compatible  ,NULL ,NULL},		//��������/��/ͣ 
    {0x0A02, "MotorTime"  ,0    ,get_motor_time          ,set_motor_time             ,NULL ,NULL},      //���ÿ��ȱ�׼����ʱ��
                                                                                                        //
	{0x0009, "SetDevShow", 0    ,NULL,          do_set_dev_show,    NULL, NULL},
    {0x000A, "GetPwd", 	   0    ,do_get_pwd,	NULL,               NULL, NULL},
    {0x000B, "SetSilTime", 0    ,NULL,          do_set_silent_time, NULL, NULL},

	{0x0A08, "DevLocation",0    ,do_get_dev_location     ,do_set_dev_location        ,NULL ,NULL}, 	    //�˶����� �������װλ������or����                                                                                                         
    {0x0A06, "BoltStatu"  ,0    ,do_get_bolt_statu       ,do_set_bolt_statu          ,NULL ,NULL},		//��ȫ˨״̬����/��ȡ
    {0x0A30, "BoltCotral" ,0    ,do_get_bolt_cotral      ,do_set_bolt_cotral         ,NULL ,NULL}, 		//��ȫ˨ʹ������/��ȡ
    {0xd005, "MotorTime"  ,0    ,get_report_enable_infor ,set_report_enable_infor    ,NULL ,NULL},      //�����ϱ�ʹ�ܲ��� 

//���ģ�Ϳ���                                                                                                    
    {0x0A0A, "SkyModelCtr",0    ,do_get_SkyModel_cotral       ,do_set_SkyModel_cotral          ,NULL ,NULL},		//���ģ��״̬����/��ȡ
    {0x0A0B, "ForceCali"  ,0    ,do_get_ForceCaliTime_cotral  ,do_set_ForceCaliTime_cotral     ,NULL ,NULL}, 		//ǿ��У׼������������/��ȡ
    {0x0A0C, "ActionCount",0    ,do_get_ActionCount_cotral    ,NULL                            ,NULL ,NULL},        //У׼����������ѯ

};
//---------------------------------------------------------------------------------------
//����DID��ȡ��Ӧ���������ĵ�ַ���ش�����û����ش���
static const struct func_ops *get_option(int did)
{
    int i;
    const struct func_ops *funcs = func_items;
    int funcs_nr = array_size(func_items);

    for (i = 0; i < funcs_nr; i++)
    {
        if (funcs[i].did == did)
            return &funcs[i];
    }
    return NULL;
}
//---------------------------------------------------------------------------------------
//��֯�����ĵ�FBD���ش������ܳ���
static int form_error_body(void *out, uint8_t * did, int err)
{
    struct Body *body = (struct Body *)out;

    memcpy(body->did, did, sizeof(body->did));
    body->ctrl = 0x82;
    put_le_val(err, body->data, 2);

    return offset_of(struct Body, data)+get_bits(body->ctrl, 0, 6);
}
//---------------------------------------------------------------------------------------
//����ͨ�Ų�������
static int do_cmd(const sdk_evt_rx_plc_t *info, int cmd, int resp, uint8_t *data, int len, uint8_t *said)
{
    int outidx = 0, inidx = 0;
    uint8_t tmp[BUF_LEN];
    memcpy(tmp, data, len);

    while (len >= FBD_FRAME_HEAD)//�������ȴ��ڵ�����С���ò�FBD����
    {
        struct Body *body = (struct Body *)&tmp[inidx];     //������������������������ַ�����
        struct Body *outbd = (struct Body *)&data[outidx];  //���������������������ַ�����

        int dlen = get_bits(body->ctrl, 0, 6);//�����ַ��ȡ�����򳤶Ȼ����������FBD�п�����ǰ��λ�������򳤶ȸ�ֵ������
        if (len < FBD_FRAME_HEAD + dlen)//�����FBD���ȱȱ�FBD����С��ʾ���ݴ����˳�ѭ���ش�����
        {
            pr_info("[PRO-DOCMD]Smart frame, body length error.\n");
            outidx += form_error_body(outbd, body->did, LEN_ERR);//��֯һ֡�ظ����Ļش�����
            break;
        }
        inidx += FBD_FRAME_HEAD + dlen;//��¼��ȡ�Ļ���������
        len -= FBD_FRAME_HEAD + dlen;//��¼ʣ������������򳤶�

        const struct func_ops *op = get_option(get_le_val(body->did, sizeof(body->did)));//����һ��did�ṹ��ʵ������ȡ��Ӧ��ǰFBD�Ĳ���
        pr_info("[PRO-DOCMD]Smart Frame DID:%x %x.\n",body->did[1],body->did[0]);
        int (*handler)(const uint8_t *src, struct Body *body) = NULL;  //����һ���յĺ������ں�����ȡ��������
        if (cmd == CMD_GET)//������Ϊ��
        {
            handler = op ? (resp ? op->read_resp : op->read) : NULL;//��ȡ���ĺ���
        }
        else//����Ϊд
        {
            handler = op ? (resp ? op->write_resp : op->write) : NULL;//��ȡд�ĺ���
        }

        if (handler)//�ж���ȡ�����Ƿ�Ϊ��
        {
            if (cmd == CMD_SET) //��������������౨�Ľ������ģ�ʹ���
                Skymodel_BodyControl_count(aid, body->did);
            //�������¿��ƶ˺�ά���������б�
			memcpy(ram_data.last_ctrl_aid, said, AID_LEN);   
            memcpy(ram_data.last_ctrl_mac, (uint8_t*)info->src, MAC_LEN);
			add_last_aid(said, (uint8_t*)info->src);
            //���ն�FBD���Ƶ�������汸��
            memcpy(outbd, body, sizeof(struct Body) + dlen);
            pr_info("[PRO-DOCMD]Do Cmd Handler:%s.\n", op->tip);
			report.broadcast_ctrl = 0;//�㲥�ϱ�״̬��־
			report.broadcast_my_seq = 0;//�㲥�ϱ�״̬ʱ�����
            int ret = handler(info->src, outbd); 
            if (ret < 0)
            {
                if (ret == -NO_RETURN) continue;//�ش�������δʵ�ִ˹�������ִ�м���������FBD
                form_error_body(outbd, body->did, -ret);//��֯�ش������FBD
            }
            else if (ret > 0)//���ڱ�ʾ��������ش��ı����������򳤶ȣ����Դ����ֶ�ͬ����ǰ�Ӹ��ţ�
            {
                memcpy(outbd->did, body->did, sizeof(body->did));
                outbd->ctrl = ret;
            }
            else
            {
                body->ctrl = 0;
//				if((op->did == 0x0A01 || op->did == 0x0A03 || op->did == 0x0A04) && (handler == op->write))
//				{
//					report.broadcast_ctrl = 0;
//					report.broadcast_my_seq = 0;
//				}
            }
        }
        else if(!resp)//δ�����÷�
        {
        	form_error_body(outbd, body->did, DID_ERR);
        }
        else
        {
        	return -1;
        }
        outidx += FBD_FRAME_HEAD + get_bits(outbd->ctrl, 0, 6);//��֯�ش���FBD���ȸ���
    }
    return outidx;
}
//---------------------------------------------------------------------------------------
//�б��Ƿ�����ͬ���Ķ�ʱ�������ط�������������������
int resend_check(const uint8_t *src, const struct SmartFrame *pframe)
{
    static uint8_t last_seq, last_src[6], last_sum;
    static uint32_t last_tm;

    uint32_t cur_seq = get_bits(pframe->seq, 0, 6);
    uint32_t cur_tm = sec_jiffies; 
    uint8_t cur_sum = pframe->data[pframe->len];
    if (!memcmp(src, last_src, sizeof(last_src)) 
        && cur_seq == last_seq 
        && cur_sum == last_sum
        && time_after(last_tm+SEC_HZ/5, cur_tm)) 
    {
        pr_info("[PRO-RCHACK]Receivd dummy frame, drop it, "
                              "last_tm[%d], cur_tm[%d]\n", last_tm, cur_tm);
        return -1;
    }
    memcpy(last_src, src, sizeof(last_src));
    last_seq = cur_seq;
    last_sum = cur_sum;
    last_tm = cur_tm;
    return 0;
}
//---------------------------------------------------------------------------------------
/*
 data[0]: bit[0~5]: data length
          bit[6~7]: group type
 group_type: 0: bit
             1: 1byte  group id
             2: 2bytes group id
*/
//����FBD��GID�ж��Ƿ��Ǳ����������ݻ�ȡ�ϱ�ʱ��
int is_gid_equal(const uint8_t * data, const uint8_t * sid)
{
    int dlen = get_bits(data[0], 0, 5);         //len_t���ַ�ֽ���
    int group_type = get_bits(data[0], 6, 7);   //type_t���ַ����
    int _sid = get_le_val(sid, 2);
	uint16_t j = 0;
	uint8_t sid_mod_8 = (_sid%8) ? (_sid%8):8;
	
    data++;//���������һλ��һλ��ʹ��
    if (group_type == 0) /* bit type */
    {
        _sid--;
        if (dlen < get_bits(_sid, 3, 7) + 1)
		{
			for(j = 0; j < dlen; j++)
			{
				report.broadcast_cnt += get_1byte_bit1_number(data[j],8);
			}
            return (0);
		}
        if (tst_bit(data[get_bits(_sid, 3, 7)], _sid & 0x07))
		{
			for(j=0; j<get_bits(_sid, 3, 7); j++)
			{
				report.broadcast_cnt += get_1byte_bit1_number(data[j],8);
			}
			report.broadcast_cnt += get_1byte_bit1_number(data[get_bits(_sid, 3, 7)], sid_mod_8);
			report.broadcast_my_seq = report.broadcast_cnt;
            return (1);
		}
		else
		{
			for(j=0; j<dlen; j++)
			{
				report.broadcast_cnt += get_1byte_bit1_number(data[j],8);
			}
			return (0);
		}
    }
    else			    /* bytes type */
    {
        int i;
        int gid_unit_len = group_type == 1 ? 1 : 2;

        for (i = 0; i < dlen; i += gid_unit_len)
        {
			report.broadcast_cnt++;
            int _gid = get_le_val(data + i, gid_unit_len);
            if ((_gid == 0) || (_sid == _gid))
			{
				report.broadcast_my_seq = report.broadcast_cnt;
                return (1);
			}
        }
    }

    return 0;
}
//---------------------------------------------------------------------------------------
int do_group_cmd(const sdk_evt_rx_plc_t *info, int resp, uint8_t *data, int len, uint8_t *said)
{
	int inidx = 0, gid_len;
	report.broadcast_cnt = 0;
    gid_len = get_bits(data[inidx], 0, 5) + 1;//���ַ�ֽ���
    while (len >= FBD_FRAME_HEAD + gid_len)//��������ֽڳ���С����չFBD��С���ĳ���ִ�����²���
    {
        struct Body *body = (struct Body *) &data[inidx + gid_len];//����һ��FBDʵ��������һ�������������ʵ����
        int body_len = gid_len + FBD_FRAME_HEAD + get_bits(body->ctrl, 0, 6);//��ȡ����ȡFBD����ʵ����
        if (len < body_len)//����������ܳ���С��FBD��ʵ���Ȳ�ִ��֮�����
            break;
        if (is_gid_equal(&data[inidx], repeater.sid))//�����ǰ�����ַ�Ǳ��ص����ַ��Ϣִ������Ĳ���
        {
            const struct func_ops *op = get_option(get_le_val(body->did, sizeof(body->did)));//����did��ȡdid�б��� �ṹ���Ա
            int (*handler) (const uint8_t * src, struct Body * body) = NULL;//����һ���յĺ���ģ��
            handler = op ? (resp ? op->write_resp : op->write) : NULL;//��ȡ��ǰ��Ա��д�������������ղ��½��Ŀ�ģ�����û���������
            if (handler)//���δ��ȡ����Ӧ��ִ�к�����ִ���ڲ�����
            {
                //���µ�ǰ���¿����豸��ַ
				memcpy(ram_data.last_ctrl_aid, said, AID_LEN);      
                memcpy(ram_data.last_ctrl_mac, (uint8_t*)info->src, MAC_LEN);
                pr_info("[PRO-DOGCMD]Do Group Handler:%s.\n", op->tip);
                report.broadcast_ctrl = 1;//�㲥�ϱ�״̬��־
                handler(info->src, body);
//              int ret = handler(info->src, body);
//				if((op->did == 0x0A01 || op->did == 0x0A03 || op->did == 0x0A04) && (NO_ERR == ret))
//				{		
//					report.broadcast_ctrl = 1;
//				}
            }
        }
        inidx += body_len;
        len -= body_len;
        gid_len = get_bits(data[inidx], 0, 5) + 1;
    }

    return 0;
}
//---------------------------------------------------------------------------------------
//�����ϴ�״̬����������У�鱨��
static int rec_gw_ack(struct SmartFrame *pframe)
{
	if(pframe->seq & 0x80)
	{
		if((CMD_RELI_REPORT == (pframe->data[0])) && (0x00 == memcmp(repeater.gid, pframe->said, AID_LEN)))
	    {
            pr_info("[PRO-RECGW]GW Return Ensure OK.\n");
			report.report_to_gw = FINISH;
	    	memset(report.rpt_7E_buf, 0x00, sizeof(report.rpt_7E_buf));
	    	return 1;
	    }
		else
			return 0;
	}
	else
		return 0;
}
//---------------------------------------------------------------------------------------
//����ִ��plc����
static int plc_frame_hook(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe)
{
    int ret = -1;       //״̬��־λ
    struct AppFrame *app = (struct AppFrame *)pframe->data;//��ȡ������Ч����

    int len = pframe->len - sizeof(struct AppFrame);
    if(rec_gw_ack(pframe))
    	return 0;
    switch (app->cmd) 
    {
    case CMD_GET:
    case CMD_SET:
        if (is_all_xx(pframe->taid, 0xff, AID_LEN) && app->cmd == CMD_SET){
            BroadcastControl = 1;
            ret = do_group_cmd(info, tst_bit(pframe->seq, 7), app->data, len, pframe->said);
        }else{
            BroadcastControl = 0;
            ret = do_cmd(info, app->cmd, tst_bit(pframe->seq, 7), app->data, len, pframe->said); 
        }
        break;
    case CMD_DEV_UPDATE:
    case CMD_NET_UPDATE:
        {
            if (tst_bit(pframe->seq, 7)) 
                return -1;
            ret = do_update(app->data, len);
        }
        break;
    case CMD_SHOW:
    {
    	ret = search_frame_opt(info, pframe);
    }
    	break;
    default:
        break;
    }
    if (ret > 0 && !is_all_xx(pframe->taid, 0xFF, sizeof(pframe->taid))) //�ǹ㲥���Ĳ���ִ�лش�ֵ��Ϊ0���鱨�Ļش���ret�ǻش����ĳ��ȣ�
    {
        ret += sizeof(struct AppFrame);
        code_ret_frame(pframe, ret);
        pr_info("[PRO-PLCHOOK]Report Data Return.\n");
        return do_send_frame((uint8_t *)info->src, pframe);
    }
    return ret;
}
//---------------------------------------------------------------------------------------
//����һ��plc���ģ�����������������������
int do_plc_frame(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe)
{
    if (info->key_mod == SDK_KEY_INIT && !is_lowpw_frame(pframe))
    {
        pr_info("[PRO-DOPLC]this frame need strong password\n");
        return 0;
    }

    return plc_frame_hook(info, pframe);
}
//---------------------------------------------------------------------------------------
static void state_report(struct soft_timer *st)
{
	if(report.init_flag)//����ϱ���ʼ����־ʹ��
	{
		report.report_to_sub = UNFINISH;//�������ϱ���־���ó�δ�ϱ������ϱ������ߣ�
		report.report_to_gw = FINISH;//�����ϱ�״̬���ó��ϱ���ɣ����ϱ���
		report.init_flag = 0;//�����ϱ���־��ʼ��
	}
	static uint8_t i = 0;//��¼�ϱ���־λλ��
	st->expires = sec_jiffies + SEC_HZ/5;//������һ�ν��붨ʱ�����ӳ�ʱ��0.2s
    if (!(eep_param.report_enable & REPORT_SB)){//���״̬�ϱ���ֹ�ϱ���������
        report.report_to_sub = FINISH;//���ö��������ϱ��������������
    }
    if (report.report_to_sub == UNFINISH)//�������ϱ�δ���
	{

        if (InsurancePlugStatusReturn && dev_state.ctrl_flag != GW_CTRL_FLAG)//�������˨ʹ�ܲ��Ѿ��򿪲��ҵ�ǰ���ƶ˲�Ϊ����
		{
            pr_info("[REPORT TO %x %x %x %x]Window magnet is enabled and open.\n"
                    ,ram_data.last_ctrl_aid[0],ram_data.last_ctrl_aid[1],ram_data.last_ctrl_aid[2],ram_data.last_ctrl_aid[3]);
            //��֯һ����Ч״̬���ķ��͸����¿��Ƶ�ַ�������������ϱ�����
            report.len_7E_buf = code_frame(repeater.aid, ram_data.last_ctrl_aid, -1, CMD_NRELI_REPORT, report.body, report.body_len, report.rpt_7E_buf, sizeof(report.rpt_7E_buf));
            do_send_frame(ram_data.last_ctrl_mac, (struct SmartFrame *)report.rpt_7E_buf);
			i = MAX_CTRL_NUM;
		}
		while(i < MAX_CTRL_NUM)//����״̬�������߶�����
		{
			if(is_all_xx(ram_data.ctrl_meg[i].aid, 0x00, AID_LEN) || memcmp(ram_data.ctrl_meg[i].aid, repeater.gid, AID_LEN) == 0)//�ж���ǰ�б�λ���Ƿ�Ϊ����Ϊ���������˴��ϱ�
			{
				i++;
				continue;
			}
            pr_info("[REPORT TO %x %x %x %x]Report status to subscribers SUB%d.\n"
                    ,ram_data.ctrl_meg[i].aid[0],ram_data.ctrl_meg[i].aid[1],ram_data.ctrl_meg[i].aid[2],ram_data.ctrl_meg[i].aid[3],i);
            //��һ���ϱ�״̬���������б��ڵ�ǰ�����ߵı��Ĳ�����
            report.len_7E_buf = code_frame(repeater.aid, ram_data.ctrl_meg[i].aid, -1, CMD_NRELI_REPORT, report.body, report.body_len, report.rpt_7E_buf, sizeof(report.rpt_7E_buf));
			do_send_frame(ram_data.ctrl_meg[i].mac, (struct SmartFrame *)report.rpt_7E_buf);
			i++;
			break;
		}
		if(report.report_to_sub == UNFINISH)//��������߻�û����ȫ�ϱ��������һ����ʱѭ��
		{
			soft_timer_add(st);
		}
		if(MAX_CTRL_NUM == i)//����ϱ������߶������
		{
			i = 0;
			report.report_to_sub = FINISH;
//            pr_info("Report report.ctrl_aid = %x %x %x %x!\n",report.ctrl_aid[0],report.ctrl_aid[1],report.ctrl_aid[2],report.ctrl_aid[3]);
//			report.body_len += code_body(0xC01A, 0, report.ctrl_aid, AID_LEN, report.body + report.body_len, sizeof(report.body));//��֯�ϱ����ص�������������ƶ�ID
		}
	}
	else if(report.report_to_sub == FINISH)
	{
		if(!is_all_xx(repeater.gid, 0x00, AID_LEN) && (eep_param.report_enable & REPORT_GW))// 
		{
			report.report_to_gw = UNFINISH;
            //��֯һ���ϴ�״̬�����صı���
			report.len_7E_buf = code_frame(repeater.aid, repeater.gid, -1, CMD_RELI_REPORT , report.body,report.body_len, report.rpt_7E_buf, sizeof(report.rpt_7E_buf));
            pr_info("[REPORT-GW 1TH]Report status Contain control AID:%x %x %x %x.\n"
                    ,report.ctrl_aid[0],report.ctrl_aid[1],report.ctrl_aid[2],report.ctrl_aid[3]);
			do_send_frame(repeater.gmac, (struct SmartFrame *)report.rpt_7E_buf);
		}
	}
}

struct soft_timer start_state_report =
{
	.timer_cb = state_report,
};
//---------------------------------------------------------------------------------------
static void identify_controllor()
{
	if(dev_state.ctrl_flag == GW_CTRL_FLAG)                         //������ؿ����豸�豸��ַΪ���ص�ַ
		memcpy(report.ctrl_aid, repeater.gid, AID_LEN);
	else if(dev_state.ctrl_flag == SUB_CTRL_FLAG)                   //��������߿����豸�ϱ������߿��Ƶ�ַ
		memcpy(report.ctrl_aid, ram_data.last_ctrl_aid, AID_LEN);
    else                                                            //���߶�����Ϊ�ϵ��ϱ������Լ��ĵ�ַ
		memcpy(report.ctrl_aid, repeater.aid, AID_LEN);
}

//---------------------------------------------------------------------------------------
//�ϱ��ӿڣ�reportTypr-> 0:��ȫ˨���쳣�ϱ�,1:����ֵ�ϱ���2�ϵ��ϱ�����ֵ��
void Resend_report(uint8_t reportTypr)
{
    uint8_t saftBoltType = 0x01;
    uint8_t saftBoltState = 0x01;
    dev_state.realtime_open_percent = MostorRunPara.LastDegree;		  
    dev_state.target_open_percent = MostorRunPara.CurrentDegree;
    identify_controllor();        
    if (reportTypr) {
        report.body_len  = code_body(0x0A03, 0, &MostorRunPara.LastDegree, 1, report.body, sizeof(report.body));//Ŀ�꿪��ֵ��֯����
        report.body_len += code_body(0x0A05, 0, &MostorRunPara.CurrentDegree, 1, report.body + report.body_len, sizeof(report.body));//��ǰ����ֵ��֯����
        pr_info("[PRO-REPORT]Report report.ctrl_aid = %x %x %x %x.\n",report.ctrl_aid[0],report.ctrl_aid[1],report.ctrl_aid[2],report.ctrl_aid[3]);
		report.body_len += code_body(0xC01A, 0, report.ctrl_aid, AID_LEN, report.body + report.body_len, sizeof(report.body));//���ƶ�ID��֯����
        if (1 == eep_param.SkyModelContral) {
            report.body_len  += code_body(0x0A0C, 0, &eep_param.ActionCount, 1, report.body, sizeof(report.body));
        }
    }else{
        pr_info("[PRO-REPORT]Safety bolt open warning.\n");
        report.body_len  = code_body(0x0A06, 0, &saftBoltType, 1, report.body, sizeof(report.body));
        report.body_len += code_body(0x0A30, 0, &saftBoltState, 1, report.body + report.body_len, sizeof(report.body));
    }
    report.init_flag = 1;
    //�򿪱��Ļش�����
	soft_timer_del(&start_state_report);
	if(report.broadcast_ctrl == 1)//����ǹ㲥���Ŀ���֮���״̬�ش�
	{
//		report.broadcast_ctrl = 0;//��λ��־
		start_state_report.expires = sec_jiffies + ((13+(report.broadcast_my_seq*5))*(SEC_HZ/10));//���ù㲥�ϱ�״̬���ӳ�ʱ��
//		report.broadcast_my_seq = 0;//�����ӳ�ʱ��
	}
	else{
        start_state_report.expires = sec_jiffies + 13 * (SEC_HZ/10);//���õ���״̬�ϱ����ӳ�ʱ��
    }
    if (reportTypr == 2) {//�ϵ��ϱ��ӳ�ʱ���趨
//        memcpy(report.ctrl_aid, repeater.aid, AID_LEN);
        start_state_report.expires = sec_jiffies + (60 + get_le_val(repeater.sid, SID_LEN)%100) * SEC_HZ;
    }
    soft_timer_add(&start_state_report);
}

//---------------------------------------------------------------------------------------
static int uart_resend_check(const uint8_t *data, uint32_t len)
{
    static uint32_t last_tm;
	static uint8_t rx_data[15] = {0};
    uint32_t cur_tm = sec_jiffies; 
	
    if (!memcmp(data, rx_data, len) && time_after(last_tm+SEC_HZ/2, cur_tm)) 
    {
        pr_info("[PRO-UARTRCHACK]Receivd dummy uart frame, drop it.\n");
        return -1;
    }
    memcpy(rx_data, data, len);
    last_tm = cur_tm;
    return 0;
}
uint8_t get_uart_smart_frame(uint8_t *data, uint32_t len)
{
	report.uart_busy = 0;
	if(len != ACK_FRAME_LEN && len != STATE_FRAME_LEN && len != 0x0A)
		return 0;
	if(data[0] != START_BYTE)
		return 0;
	if(data[len-1] != XorCheack(data, len - 1))
		return 0;
	if(uart_resend_check(data, len) < 0)
		return 0;
	return 1;
}
//---------------------------------------------------------------------------------------
//�����Ĳ���������ط��ͻ�����
struct UART_LIST uart_list;
uint8_t insert_uart_order(struct UART_MOSTOR_FRAME *frame)
{
    if(uart_list.count < MAX_UART_LIST)
    {
    	memcpy(&uart_list.frame[uart_list.count], frame, sizeof(struct UART_MOSTOR_FRAME));
        uart_list.count++;
        return (1);
    }
    return (0);
}
//��ȡ���ܱ���
uint8_t get_uart_order(struct UART_MOSTOR_FRAME *frame)
{
    if(uart_list.count > 0)
    {
        memcpy(frame, &uart_list.frame[0], sizeof(uart_list.frame[0].data) + 4);
        memmove(&uart_list.frame[0], &uart_list.frame[1], uart_list.count*sizeof(struct UART_MOSTOR_FRAME));
        uart_list.count--;

        return (1);
    }
    return (0);
}
//����������ܱ��Ĳ����лش�
uint8_t peek_uart_order(struct UART_MOSTOR_FRAME *frame)
{
    if(uart_list.count > 0)
    {
        memcpy(frame, &uart_list.frame[0], sizeof(struct UART_MOSTOR_FRAME));
        memmove(&uart_list.frame[0], &uart_list.frame[1], uart_list.count*sizeof(struct UART_MOSTOR_FRAME));
        uart_list.count--;
        return (1);
    }
    return (0);
}

