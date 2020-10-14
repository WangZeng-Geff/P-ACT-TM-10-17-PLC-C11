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

uint8_t InsurancePlugStatusReturn = 0;       //检测是否回传保险栓状态
uint8_t BroadcastControl = 0;       //检测是否为广播控制模式0不是1是:

//---------------------------------测试工装功能------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//设置aid
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
        ret = code_local_frame(&pframe->data[1], ret, pframe, 0x100);//组织一条本息通信报文 数据域位从pframe->data[1]开始长度为ret输出到pframe内最大长度0x100
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



//------------------------------电力线通信报文--------------------------------------------
//---------------------------------------------------------------------------------------

/// <summary> 
/// 更新订阅者AID列表 
/// </summary> <param name="aid"></param> <param 
/// name="mac"></param> <returns></returns> 
static void add_last_aid(uint8_t *aid, uint8_t *mac)
{
	if(!memcmp(repeater.gid, aid, AID_LEN))
		return;
	int i;
//保存当前控制端的AID和MAC
    memset(Current_ctrl_meg.aid, 0x00, AID_LEN);
    memset(Current_ctrl_meg.mac, 0x00, MAC_LEN);
    memcpy(Current_ctrl_meg.aid, aid, AID_LEN);
	memcpy(Current_ctrl_meg.mac, mac, MAC_LEN);
//遍历订阅者缓存列表对比是否列表中存在此订阅者地址如果存在则不在进行以下保存操作
	for(i = 0; i < MAX_CTRL_NUM; i++)
	{
		if(!memcmp(ram_data.ctrl_meg[i].aid, aid, AID_LEN))
			return;		
	}

	for(i = 0; i < MAX_CTRL_NUM-1; i++)//将第一个最早的订阅者挤掉将后两个订阅者往前排
	{
		memcpy(ram_data.ctrl_meg[i].aid, ram_data.ctrl_meg[i+1].aid, AID_LEN);
		memcpy(ram_data.ctrl_meg[i].mac, ram_data.ctrl_meg[i+1].mac, MAC_LEN);
	}
    //写入最新的订阅者到最后的地址
	memcpy(ram_data.ctrl_meg[i].aid, aid, AID_LEN);
	memcpy(ram_data.ctrl_meg[i].mac, mac, MAC_LEN);
}
//---------------------------------------------------------------------------------------
//获取一整完整的接收报文(获取编码框架)
struct SmartFrame *get_smart_frame(const uint8_t *in, int len)
{
    int i = 0;

 start_lbl:
    while (i < len)//在缓存区域内寻找报文字头
    {
        if (STC == in[i])
            break;
        i++;
    }
    //如果报文字头剩后余字节长度小于最短报文格式（我数据域无校验和）长度表示无完整报文直接退出操作
    if (len - i < SMART_FRAME_HEAD)
        return NULL;
    //初始化一个报文结构体实例，并将缓存区从报文字头位置初始地址赋值给该实例
    struct SmartFrame *pframe = (struct SmartFrame *)&in[i];
    int dlen = pframe->len;//获取该报文的数据域长度
    if (i + SMART_FRAME_HEAD + dlen + 1 > len)//缓存区报文剩余长度，不足以维持当前报文组成完整报文
    {
        i++;
        goto start_lbl;
//				return NULL;
    }
    //本组报文校验和不对
    if (pframe->data[dlen] != checksum(pframe, dlen + SMART_FRAME_HEAD))
    {
        i++;
        goto start_lbl;
    }
    pframe = (struct SmartFrame *)&in[i];//再次将缓存区这一帧完整的报文在缓存区初始地址赋值给变量
    return pframe;
}
//---------------------------------------------------------------------------------------
//组织编码框架
//src->源地址;dest->目的地址;seq->帧序号;cmd->命令字;data->数据域;len->数据长度;out->输出位置;maxlen->最大长度
int code_frame(const uint8_t *src, const uint8_t *dest, int seq, int cmd, 
    const uint8_t *data, int len, void *out, int maxlen)
{
    const uint8_t addr[AID_LEN] = {0x00, 0x00, 0x00, 0x00};//定义一组固定地址

    static uint8_t _seq = 0;                //定义操作的帧序号
    struct SmartFrame *pframe = (struct SmartFrame *)out;//定义一个报文操作实例并与输出位置相互关联

    pframe->stc = STC;                       //定义报文字头
    //将传入并处理好的SAID和TAID写入报文当中   
    if (!src) src = addr;                    //如果传入的SRC即AID为空则将SAID置成0
    if (!dest) dest = addr;                  //如果传入的dest为空则将TAID置成0
    memcpy(pframe->said, src, AID_LEN);      
    memcpy(pframe->taid, dest, AID_LEN);
    pframe->seq = seq < 0 ? (_seq++ & 0x7F) : seq;//帧序号处理？？？？？
    pframe->data[0] = cmd;               //数据域第一个字节(bit7-bit4)保留；bit3为1表示有2个字节分帧信息，为0时则没有分帧信息。（bit2-bit0）： 010表示查询， 111表示设置；
    memcpy(&pframe->data[1], data, len); //数据域写入数据位信息
    pframe->len = len+1;        //这只数据域长度
    pframe->data[pframe->len] = checksum(pframe, SMART_FRAME_HEAD + pframe->len);//设置校验和
    return SMART_FRAME_HEAD + pframe->len + 1;//回传报文总长度
}
//---------------------------------------------------------------------------------------
//组织一条本地通信报文 in->输入数据地址；len->数据域长度 out->输出地址 maxlen->最大长度
int code_local_frame(const uint8_t *in, int len, void *out, int maxlen)
{
    return code_frame(NULL, NULL, 0, in[0], &in[1], len-1, out, maxlen);
}
//---------------------------------------------------------------------------------------
//组织一条回复确认的报文
int code_ret_frame(struct SmartFrame *pframe, int len)
{
    //源之地目的地址调换
    memcpy(pframe->taid, pframe->said, AID_LEN); 
    memcpy(pframe->said, repeater.aid, AID_LEN); 
    pframe->seq |= 0x80;//改变帧序号为回复类型
    pframe->len = len;  //重写报文长度
    pframe->data[len] = checksum(pframe, pframe->len + SMART_FRAME_HEAD);//重心对校验和进行校验
    return pframe->len + SMART_FRAME_HEAD + 1;//回传最新的数据域长度
}
//组织数据域单个分帧应用层FBD真题操作
//did->分帧操作的did err->控制字 data->数据域 out->输出位置 maxlen->最大长度
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
//鉴别接收报文是否是是作用于此设备
int is_my_smart_frame(const struct SmartFrame *frame)
{
    //如果本身未烧写AID则不进行操作返回零
    if (is_all_xx(repeater.aid, 0x00, AID_LEN))
    {
        pr_info("[SYSTEM-ISMY]I do not have aid, drop it.\n");
        return 0;
    }
    //如果接收报文不是广播报文并且报文的目的地址于本地源地址不同返回零
    if (!is_all_xx(frame->taid, 0xFF, AID_LEN) && memcmp(frame->taid, repeater.aid, AID_LEN)) 
    {
        return 0;
    }
    return 1;
}
//---------------------------------------------------------------------------------------
//判别特定当下不用的DID
static int is_lowpw_frame(struct SmartFrame *pframe)
{
    int i;
    const uint16_t low_did[] = {0x0004, 0x0603, 0x0604, 0x0605, 0x0606, 0x0607, 0x0608, 0x060A, 0x060C};//定义特定DID
    struct AppFrame *app = (struct AppFrame *)pframe->data;

	if (CMD_SHOW == app->cmd)//命令字为显示命令返回1
	{
		return 1;
	}
	else
	{
        //当前报文did如果有存在于定义列表内相同的情况返回1
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
//发送一帧载波报文
int do_send_frame(uint8_t *mac, struct SmartFrame *frame)
{
    int key_mod = is_lowpw_frame(frame) ? SDK_KEY_INIT : SDK_KEY_USER;//根据报文中did类型判定使用初始化秘钥还是用户秘钥
    if (is_all_xx(frame->taid, 0xFF, AID_LEN))  //判断当前报文是否是广播报文
    {
        return do_plc_broadcast(key_mod, frame, frame_len(frame));//通过广播形式回传
    }
    
    return do_plc_unicast(mac, key_mod, frame, frame_len(frame));//通过单薄形式回传
}
//---------------------------------------------------------------------------------------
//重启设备
static void reboot_tmr_handle(ULONG arg)
{
    sdk_svc_reset();
}
//---------------------------------------------------------------------------------------
//设备复位did列表操作
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
//读取设备版本did列表操作
static int do_get_dev_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_dev_ver();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
//读取设备载波版本did列表操作
static int do_get_plc_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_plc_ver();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
//获取设备类型did列表操作
static int do_get_dev_type(const uint8_t *src, struct Body *body)
{
	return get_dev_type(body->data);
}
//---------------------------------------------------------------------------------------
//读取应用层协议及版本did列表操作
static int do_get_comm_ver(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, comm_protocol_ver); 
    return strlen(comm_protocol_ver);
}
//---------------------------------------------------------------------------------------
//读取网络层物料编码did列表操作
static int do_get_plc_type(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, repeater_plc_type); 
    return strlen(repeater_plc_type);
}
//---------------------------------------------------------------------------------------
//组织一条回传网关的报文并发出（配套写载波芯片注册信息使用）
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
//写载波芯片注册信息
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
//获取当前设备AID
static int do_get_aid(const uint8_t *src, struct Body *body)
{
    if (is_all_xx(repeater.aid, 0, AID_LEN) || memcmp(body->data, repeater.aid, AID_LEN)) 
        return -NO_RETURN;
    memcpy(body->data, repeater.aid, AID_LEN);
    return AID_LEN;
}
//---------------------------------------------------------------------------------------
//读取适配层版本号
static int do_get_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_dev_ver1();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
//读取载波版本号
static int do_get_ext_ver(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, repeater_ext_ver); 
    reverse(body->data, strlen(repeater_ext_ver));
    return strlen(repeater_ext_ver);
}
//---------------------------------------------------------------------------------------
//读DKEY信息
static int do_get_dev_key(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.dkey, DKEY_LEN);
	return DKEY_LEN;
}
//---------------------------------------------------------------------------------------
//读SN
static int do_get_dev_sn(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.sn, SN_LEN);
	return SN_LEN;
}
//---------------------------------------------------------------------------------------
//读PWD
static int do_get_pwd(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.passwd, PW_LEN);
	return PW_LEN;
}
//---------------------------------------------------------------------------------------
//搜索设备设备动作相应
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
//设置静默时间
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
//获取设备当前转向
static int do_get_dev_location(const uint8_t *src, struct Body *body)
{
	body->data[0] = eep_param.location;
	return(1);
}
//---------------------------------------------------------------------------------------
//设置设备转向
static int do_set_dev_location(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
    {
        return -DATA_ERR;
    }
	if(eep_param.location != body->data[1])
	{
        eep_param.location = body->data[0];
        flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //讲上升时间下降时间写入到内存中
        flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, location), 
                              (uint8_t *)(&eep_param.location), 
                              sizeof(eep_param.location));     //将参数写入flash内保存（优化）
	}
    return NO_ERR;
}
//---------------------------------------------------------------------------------------
//DID 0A01 1.stop 2.open 3.close 兼容旧设备，适配LCD等设备
static int do_dev_control_compatible(const uint8_t *src, struct Body *body)	
{
	uint8_t rx_val = 0;
    InsurancePlugStatusReturn = 0;       //检测是否回传保险栓状态
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
        InsurancePlugStatusReturn = 1;       //检测是否回传保险栓状态
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
    InsurancePlugStatusReturn = 0;       //检测是否回传保险栓状态

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
        InsurancePlugStatusReturn = 1;       //检测是否回传保险栓状态
        Resend_report(0);
        return (-ENLOCK_ERR);
    }

	OneWay_Simple_Operation(body->data[0], 12000);
	return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//读取设备开度值
static int do_get_open_percent(const uint8_t *src, struct Body *body)
{
	body->data[0] = MostorRunPara.CurrentDegree;
	return (1);
}
//设置设备开度值
static int do_set_open_percent(const uint8_t *src, struct Body *body)
{
    InsurancePlugStatusReturn = 0;       //检测是否回传保险栓状态

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
        InsurancePlugStatusReturn = 1;       //检测是否回传保险栓状态
        Resend_report(0);
        return (-ENLOCK_ERR);
    }
	MostorRunPara.TargetDegree = body->data[0];
    if ((eep_param.motor_up_time == 0) || (eep_param.motor_dn_time == 0) || MostorRunPara.TargetDegree > 100) return (-DATA_ERR);//上升下降标准时间设定检测
    Percent_Complex_Operation(MostorRunPara.TargetDegree);
	return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//读取设备行程时间
static int get_motor_time(const uint8_t *src, struct Body *body)
{
   body->data[1] = (eep_param.motor_up_time >> 8) & 0xff;
   body->data[0] = eep_param.motor_up_time & 0xFF;
   body->data[3] = (eep_param.motor_dn_time >> 8) & 0xff;
   body->data[2] = eep_param.motor_dn_time & 0xFF;
   return (4);
}
//设置设备行程时间
static int set_motor_time(const uint8_t *src, struct Body *body)
{
    uint16_t uptime, dntime;
    InsurancePlugStatusReturn = 0;       //检测是否回传保险栓状态
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
       && eep_param.BoltBoltStatu){//|设置行程时间单播控制也要受到影响
        pr_info("[DIDSET-0A02]Bolt Crtal is open.\n");
        InsurancePlugStatusReturn = 1;       //检测是否回传保险栓状态
        Resend_report(0);
        return (-ENLOCK_ERR);
    }
    uptime = body->data[1] * 256 + body->data[0];
    dntime = body->data[3] * 256 + body->data[2];
    if ((uptime < 500) || (dntime < 500))    //如果设置时间不足五秒则认为出错
       return (-DATA_ERR);
    if ((uptime > 12000) || (dntime > 12000))//如果设置时间大于2min辨识出错
       return (-DATA_ERR);
    Set_StandAct_Time(uptime, dntime);

    return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//读取设备当前上报参数
static int get_report_enable_infor(const uint8_t *src, struct Body *body)
{
   body->data[0] = 0x00;
   body->data[1] = eep_param.report_enable;
   return (2);
}
//设置设备当前上报参数0:不进行上报1:只上报网关2:只上报订阅者3:网关订阅者都上报
static int set_report_enable_infor(const uint8_t *src, struct Body *body)
{
   if (body->ctrl != 0x02 || body->data[1] > 3) return (-DATA_ERR);
   eep_param.report_enable = body->data[1];
   flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //将参数写入flash内保存
   flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, report_enable), 
                         (uint8_t *)(&eep_param.report_enable), 
                         sizeof(eep_param.report_enable));     //将参数写入flash内保存（优化）

   return (NO_ERR);
}
//---------------------------------------------------------------------------------------
//设置窗磁安全栓使能状态0:设置禁能1:设置使能
static int do_set_bolt_cotral(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
    {
        return -DATA_ERR;
    }
    pr_info("[DIDSET-0A30] LastBoltCrtal:%d And CurrBoltCrtal:%d.\n",eep_param.BoltCrtal,body->data[0]);//打印当前和更改后窗磁使能状态
    eep_param.BoltCrtal = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //将参数写入flash内保存
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, BoltCrtal), 
                          (uint8_t *)(&eep_param.BoltCrtal), 
                          sizeof(eep_param.BoltCrtal));     //将参数写入flash内保存（优化）
    return (NO_ERR);
}
//获取当前设备窗磁使能状态
static int do_get_bolt_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.BoltCrtal;
    return (1);
}

//---------------------------------------------------------------------------------------
//设置当前窗磁动作状态
static int do_set_bolt_statu(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
    {
        return -DATA_ERR;
    }
    pr_info("[didSET-0A06] lastBoltStatu:%d And CurrBoltStatu:%d.\n",eep_param.BoltBoltStatu,body->data[0]);//打印当前和更改后窗磁使能状态
    eep_param.BoltBoltStatu = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //将参数写入flash内保存
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, BoltBoltStatu), 
                          (uint8_t *)(&eep_param.BoltBoltStatu), 
                          sizeof(eep_param.BoltBoltStatu));     //将参数写入flash内保存（优化）
    return (NO_ERR);
}
//读取当前窗磁动作状态
static int do_get_bolt_statu(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.BoltBoltStatu;
    return (1);
}

//---------------------------------------------------------------------------------------
//-------------------------------天空模型控制部分-----------------------------------------
///天空模型状态读写
static int do_set_SkyModel_cotral(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01 || body->data[0] > 1)
        return -DATA_ERR;
    eep_param.SkyModelContral = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //将参数写入flash内保存
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, SkyModelContral), 
                          (uint8_t *)(&eep_param.SkyModelContral), 
                          sizeof(eep_param.SkyModelContral));     //将参数写入flash内保存（优化）
    return (NO_ERR);
}
//---------------------------------------------------------------------------------------
static int do_get_SkyModel_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.SkyModelContral;
    return (1);
}
///强制校准次数读写 
static int do_set_ForceCaliTime_cotral(const uint8_t *src, struct Body *body)
{
    if(body->ctrl != 0x01)
        return -DATA_ERR;
    eep_param.ForceCaliTime = body->data[0];
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //将参数写入flash内保存
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, ForceCaliTime), 
                          (uint8_t *)(&eep_param.ForceCaliTime), 
                          sizeof(eep_param.ForceCaliTime));     //将参数写入flash内保存（优化）
    return (NO_ERR);
}
//---------------------------------------------------------------------------------------
static int do_get_ForceCaliTime_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.ForceCaliTime;
    return (1);
}
///校准后动作次数读
static int do_get_ActionCount_cotral(const uint8_t *src, struct Body *body)
{
    body->data[0] = eep_param.ActionCount;
    return (1);
}
//天空模型根据检测人工动作关闭天空模型
static void Skymodel_BodyControl_count(uint8_t aid ,int did)
{
    //如果天空模型未开启
    if (0x00 == eep_param.SkyModelContral) 
        return ;
    //如果源地址是网关地址不进行以下操作
    if (!memcmp(repeater.gid, aid, AID_LEN))
		return;
    //如果DID不是设备控制指令退出当前操作
    if ((0x0A01 != did) && (0x0A03 != did) && (0x0A04 != did))
        return ;
    eep_param.SkyBodyCotrlCount++;
    if (eep_param.SkyBodyCotrlCount > eep_param.SkyBodyCotrlUper) {//人工控制超上限
        eep_param.SkyModelContral = 0x00;//禁能天空模型
    }
    flash_user_info_write(EEP_MOTOR_ADDR, (uint8_t*)&eep_param, sizeof(eep_param));     //将参数写入flash内保存
    flash_user_info_write(EEP_MOTOR_ADDR + offset_of(struct EEPPARAM, SkyBodyCotrlCount), 
                          (uint8_t *)(&eep_param.SkyBodyCotrlCount), 
                          sizeof(eep_param.SkyBodyCotrlCount) + sizeof(eep_param.SkyBodyCotrlCount));     //将参数写入flash内保存（优化）
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
//优化更新部分
//窗帘优化后兼容
    {0x0001, "DevType",	   0    ,do_get_dev_type, NULL, NULL, NULL},
    {0x0002, "ComProVer",  0    ,do_get_comm_ver, NULL, NULL, NULL},
    {0x0003, "DevVersion", 0    ,do_get_dev_ver,  NULL, NULL, NULL},
    {0x0005, "DevKey", 	   0    ,do_get_dev_key,  NULL, NULL, NULL},
    {0x0007, "DevSn",      0    ,do_get_dev_sn,   NULL, NULL, NULL},


    {0x0A03, "OpenPercent",0    ,do_get_open_percent     ,do_set_open_percent        ,NULL ,NULL}, 		//开合度设置/读取
    {0x0A04, "DevContrl"  ,0    ,NULL                    ,do_dev_control             ,NULL ,NULL},		//开合帘开/关/停
	{0x0A01, "DevContrl"  ,0    ,NULL                    ,do_dev_control_compatible  ,NULL ,NULL},		//开合帘开/关/停 
    {0x0A02, "MotorTime"  ,0    ,get_motor_time          ,set_motor_time             ,NULL ,NULL},      //设置开度标准基础时间
                                                                                                        //
	{0x0009, "SetDevShow", 0    ,NULL,          do_set_dev_show,    NULL, NULL},
    {0x000A, "GetPwd", 	   0    ,do_get_pwd,	NULL,               NULL, NULL},
    {0x000B, "SetSilTime", 0    ,NULL,          do_set_silent_time, NULL, NULL},

	{0x0A08, "DevLocation",0    ,do_get_dev_location     ,do_set_dev_location        ,NULL ,NULL}, 	    //运动方向 即电机安装位置在左or在右                                                                                                         
    {0x0A06, "BoltStatu"  ,0    ,do_get_bolt_statu       ,do_set_bolt_statu          ,NULL ,NULL},		//安全栓状态设置/读取
    {0x0A30, "BoltCotral" ,0    ,do_get_bolt_cotral      ,do_set_bolt_cotral         ,NULL ,NULL}, 		//安全栓使能设置/读取
    {0xd005, "MotorTime"  ,0    ,get_report_enable_infor ,set_report_enable_infor    ,NULL ,NULL},      //数据上报使能操作 

//天空模型控制                                                                                                    
    {0x0A0A, "SkyModelCtr",0    ,do_get_SkyModel_cotral       ,do_set_SkyModel_cotral          ,NULL ,NULL},		//天空模型状态设置/读取
    {0x0A0B, "ForceCali"  ,0    ,do_get_ForceCaliTime_cotral  ,do_set_ForceCaliTime_cotral     ,NULL ,NULL}, 		//强制校准上限设置设置/读取
    {0x0A0C, "ActionCount",0    ,do_get_ActionCount_cotral    ,NULL                            ,NULL ,NULL},        //校准后动作次数查询

};
//---------------------------------------------------------------------------------------
//根据DID获取相应操作函数的地址并回传，若没有则回传空
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
//组织错误报文的FBD并回传报文总长度
static int form_error_body(void *out, uint8_t * did, int err)
{
    struct Body *body = (struct Body *)out;

    memcpy(body->did, did, sizeof(body->did));
    body->ctrl = 0x82;
    put_le_val(err, body->data, 2);

    return offset_of(struct Body, data)+get_bits(body->ctrl, 0, 6);
}
//---------------------------------------------------------------------------------------
//单波通信操作步骤
static int do_cmd(const sdk_evt_rx_plc_t *info, int cmd, int resp, uint8_t *data, int len, uint8_t *said)
{
    int outidx = 0, inidx = 0;
    uint8_t tmp[BUF_LEN];
    memcpy(tmp, data, len);

    while (len >= FBD_FRAME_HEAD)//所传长度大于等于最小引用层FBD长度
    {
        struct Body *body = (struct Body *)&tmp[inidx];     //定义操作变量并与输入参数地址相关联
        struct Body *outbd = (struct Body *)&data[outidx];  //操作变脸并与输出参数地址相关联

        int dlen = get_bits(body->ctrl, 0, 6);//定义地址获取数据域长度缓存变量并将FBD中控制字前六位即数据域长度赋值给变脸
        if (len < FBD_FRAME_HEAD + dlen)//传入的FBD长度比本FBD长度小表示数据错误退出循环回传错误
        {
            pr_info("[PRO-DOCMD]Smart frame, body length error.\n");
            outidx += form_error_body(outbd, body->did, LEN_ERR);//组织一帧回复报文回传报文
            break;
        }
        inidx += FBD_FRAME_HEAD + dlen;//记录获取的缓存区长度
        len -= FBD_FRAME_HEAD + dlen;//记录剩余的输入数据域长度

        const struct func_ops *op = get_option(get_le_val(body->did, sizeof(body->did)));//定义一个did结构体实例并获取对应当前FBD的操作
        pr_info("[PRO-DOCMD]Smart Frame DID:%x %x.\n",body->did[1],body->did[0]);
        int (*handler)(const uint8_t *src, struct Body *body) = NULL;  //定义一个空的函数用于后面提取操作函数
        if (cmd == CMD_GET)//命令字为读
        {
            handler = op ? (resp ? op->read_resp : op->read) : NULL;//提取读的函数
        }
        else//否则为写
        {
            handler = op ? (resp ? op->write_resp : op->write) : NULL;//提取写的函数
        }

        if (handler)//判断提取函数是否为空
        {
            if (cmd == CMD_SET) //如果报文是设置类报文进入天空模型处理
                Skymodel_BodyControl_count(aid, body->did);
            //更新最新控制端和维护订阅者列表
			memcpy(ram_data.last_ctrl_aid, said, AID_LEN);   
            memcpy(ram_data.last_ctrl_mac, (uint8_t*)info->src, MAC_LEN);
			add_last_aid(said, (uint8_t*)info->src);
            //接收端FBD复制到输出缓存备用
            memcpy(outbd, body, sizeof(struct Body) + dlen);
            pr_info("[PRO-DOCMD]Do Cmd Handler:%s.\n", op->tip);
			report.broadcast_ctrl = 0;//广播上报状态标志
			report.broadcast_my_seq = 0;//广播上报状态时间管理
            int ret = handler(info->src, outbd); 
            if (ret < 0)
            {
                if (ret == -NO_RETURN) continue;//回传错误是未实现此功能重新执行检索缓存区FBD
                form_error_body(outbd, body->did, -ret);//组织回传错误的FBD
            }
            else if (ret > 0)//用于表示读操作后回传的报文中数据域长度（所以错误字都同意在前加负号）
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
        else if(!resp)//未发现用法
        {
        	form_error_body(outbd, body->did, DID_ERR);
        }
        else
        {
        	return -1;
        }
        outidx += FBD_FRAME_HEAD + get_bits(outbd->ctrl, 0, 6);//组织回传的FBD长度更新
    }
    return outidx;
}
//---------------------------------------------------------------------------------------
//判别是否是相同报文短时间连续重发如果是则放下这条报文
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
//根据FBD中GID判断是否是本机控制内容获取上报时间
int is_gid_equal(const uint8_t * data, const uint8_t * sid)
{
    int dlen = get_bits(data[0], 0, 5);         //len_t组地址字节数
    int group_type = get_bits(data[0], 6, 7);   //type_t组地址类型
    int _sid = get_le_val(sid, 2);
	uint16_t j = 0;
	uint8_t sid_mod_8 = (_sid%8) ? (_sid%8):8;
	
    data++;//数据域后移一位第一位已使用
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
    gid_len = get_bits(data[inidx], 0, 5) + 1;//组地址字节数
    while (len >= FBD_FRAME_HEAD + gid_len)//如果输入字节长度小于扩展FBD最小报文长不执行以下操作
    {
        struct Body *body = (struct Body *) &data[inidx + gid_len];//定义一个FBD实例并将第一个数据域关联到实例上
        int body_len = gid_len + FBD_FRAME_HEAD + get_bits(body->ctrl, 0, 6);//获取已提取FBD的真实长度
        if (len < body_len)//如果缓存区总长度小于FBD真实长度不执行之后操作
            break;
        if (is_gid_equal(&data[inidx], repeater.sid))//如果当前的组地址是本地的组地址信息执行下面的操作
        {
            const struct func_ops *op = get_option(get_le_val(body->did, sizeof(body->did)));//根据did获取did列表呢 结构体成员
            int (*handler) (const uint8_t * src, struct Body * body) = NULL;//定义一个空的函数模板
            handler = op ? (resp ? op->write_resp : op->write) : NULL;//获取当前成员的写函数并关联到刚才新建的空模板如果没有则关联空
            if (handler)//如果未获取到相应的执行函数则不执行内部操作
            {
                //更新当前最新控制设备地址
				memcpy(ram_data.last_ctrl_aid, said, AID_LEN);      
                memcpy(ram_data.last_ctrl_mac, (uint8_t*)info->src, MAC_LEN);
                pr_info("[PRO-DOGCMD]Do Group Handler:%s.\n", op->tip);
                report.broadcast_ctrl = 1;//广播上报状态标志
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
//接收上传状态到网关网关校验报文
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
//解析执行plc报文
static int plc_frame_hook(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe)
{
    int ret = -1;       //状态标志位
    struct AppFrame *app = (struct AppFrame *)pframe->data;//获取报文有效数据

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
    if (ret > 0 && !is_all_xx(pframe->taid, 0xFF, sizeof(pframe->taid))) //非广播报文并且执行回传值不为0则组报文回传（ret是回传报文长度）
    {
        ret += sizeof(struct AppFrame);
        code_ret_frame(pframe, ret);
        pr_info("[PRO-PLCHOOK]Report Data Return.\n");
        return do_send_frame((uint8_t *)info->src, pframe);
    }
    return ret;
}
//---------------------------------------------------------------------------------------
//处理一条plc报文？？？？？？？？？？？？
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
	if(report.init_flag)//如果上报初始化标志使能
	{
		report.report_to_sub = UNFINISH;//订阅者上报标志设置成未上报（先上报订阅者）
		report.report_to_gw = FINISH;//网关上报状态设置成上报完成（后上报）
		report.init_flag = 0;//禁能上报标志初始化
	}
	static uint8_t i = 0;//记录上报标志位位数
	st->expires = sec_jiffies + SEC_HZ/5;//设置下一次进入定时器的延迟时间0.2s
    if (!(eep_param.report_enable & REPORT_SB)){//如果状态上报禁止上报给订阅者
        report.report_to_sub = FINISH;//设置订阅者已上报完成跳过订阅者
    }
    if (report.report_to_sub == UNFINISH)//订阅者上报未完成
	{

        if (InsurancePlugStatusReturn && dev_state.ctrl_flag != GW_CTRL_FLAG)//如果保险栓使能并已经打开并且当前控制端不为网关
		{
            pr_info("[REPORT TO %x %x %x %x]Window magnet is enabled and open.\n"
                    ,ram_data.last_ctrl_aid[0],ram_data.last_ctrl_aid[1],ram_data.last_ctrl_aid[2],ram_data.last_ctrl_aid[3]);
            //组织一条有效状态报文发送给最新控制地址并跳过订阅者上报环节
            report.len_7E_buf = code_frame(repeater.aid, ram_data.last_ctrl_aid, -1, CMD_NRELI_REPORT, report.body, report.body_len, report.rpt_7E_buf, sizeof(report.rpt_7E_buf));
            do_send_frame(ram_data.last_ctrl_mac, (struct SmartFrame *)report.rpt_7E_buf);
			i = MAX_CTRL_NUM;
		}
		while(i < MAX_CTRL_NUM)//发送状态给订阅者订阅者
		{
			if(is_all_xx(ram_data.ctrl_meg[i].aid, 0x00, AID_LEN) || memcmp(ram_data.ctrl_meg[i].aid, repeater.gid, AID_LEN) == 0)//判定当前列表位置是否为空若为空则跳过此次上报
			{
				i++;
				continue;
			}
            pr_info("[REPORT TO %x %x %x %x]Report status to subscribers SUB%d.\n"
                    ,ram_data.ctrl_meg[i].aid[0],ram_data.ctrl_meg[i].aid[1],ram_data.ctrl_meg[i].aid[2],ram_data.ctrl_meg[i].aid[3],i);
            //组一条上报状态给订阅者列表内当前订阅者的报文并发出
            report.len_7E_buf = code_frame(repeater.aid, ram_data.ctrl_meg[i].aid, -1, CMD_NRELI_REPORT, report.body, report.body_len, report.rpt_7E_buf, sizeof(report.rpt_7E_buf));
			do_send_frame(ram_data.ctrl_meg[i].mac, (struct SmartFrame *)report.rpt_7E_buf);
			i++;
			break;
		}
		if(report.report_to_sub == UNFINISH)//如果订阅者还没有完全上报则进入下一个定时循环
		{
			soft_timer_add(st);
		}
		if(MAX_CTRL_NUM == i)//如果上报订阅者都已完成
		{
			i = 0;
			report.report_to_sub = FINISH;
//            pr_info("Report report.ctrl_aid = %x %x %x %x!\n",report.ctrl_aid[0],report.ctrl_aid[1],report.ctrl_aid[2],report.ctrl_aid[3]);
//			report.body_len += code_body(0xC01A, 0, report.ctrl_aid, AID_LEN, report.body + report.body_len, sizeof(report.body));//组织上报网关的数据域包含控制端ID
		}
	}
	else if(report.report_to_sub == FINISH)
	{
		if(!is_all_xx(repeater.gid, 0x00, AID_LEN) && (eep_param.report_enable & REPORT_GW))// 
		{
			report.report_to_gw = UNFINISH;
            //组织一条上传状态给网关的报文
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
	if(dev_state.ctrl_flag == GW_CTRL_FLAG)                         //如果网关控制设备设备地址为网关地址
		memcpy(report.ctrl_aid, repeater.gid, AID_LEN);
	else if(dev_state.ctrl_flag == SUB_CTRL_FLAG)                   //如果订阅者控制设备上报订阅者控制地址
		memcpy(report.ctrl_aid, ram_data.last_ctrl_aid, AID_LEN);
    else                                                            //两者都不是为上电上报本地自己的地址
		memcpy(report.ctrl_aid, repeater.aid, AID_LEN);
}

//---------------------------------------------------------------------------------------
//上报接口（reportTypr-> 0:安全栓打开异常上报,1:开度值上报，2上电上报开度值）
void Resend_report(uint8_t reportTypr)
{
    uint8_t saftBoltType = 0x01;
    uint8_t saftBoltState = 0x01;
    dev_state.realtime_open_percent = MostorRunPara.LastDegree;		  
    dev_state.target_open_percent = MostorRunPara.CurrentDegree;
    identify_controllor();        
    if (reportTypr) {
        report.body_len  = code_body(0x0A03, 0, &MostorRunPara.LastDegree, 1, report.body, sizeof(report.body));//目标开度值组织报文
        report.body_len += code_body(0x0A05, 0, &MostorRunPara.CurrentDegree, 1, report.body + report.body_len, sizeof(report.body));//当前开度值组织报文
        pr_info("[PRO-REPORT]Report report.ctrl_aid = %x %x %x %x.\n",report.ctrl_aid[0],report.ctrl_aid[1],report.ctrl_aid[2],report.ctrl_aid[3]);
		report.body_len += code_body(0xC01A, 0, report.ctrl_aid, AID_LEN, report.body + report.body_len, sizeof(report.body));//控制端ID组织报文
        if (1 == eep_param.SkyModelContral) {
            report.body_len  += code_body(0x0A0C, 0, &eep_param.ActionCount, 1, report.body, sizeof(report.body));
        }
    }else{
        pr_info("[PRO-REPORT]Safety bolt open warning.\n");
        report.body_len  = code_body(0x0A06, 0, &saftBoltType, 1, report.body, sizeof(report.body));
        report.body_len += code_body(0x0A30, 0, &saftBoltState, 1, report.body + report.body_len, sizeof(report.body));
    }
    report.init_flag = 1;
    //打开报文回传任务
	soft_timer_del(&start_state_report);
	if(report.broadcast_ctrl == 1)//如果是广播报文控制之后的状态回传
	{
//		report.broadcast_ctrl = 0;//复位标志
		start_state_report.expires = sec_jiffies + ((13+(report.broadcast_my_seq*5))*(SEC_HZ/10));//设置广播上报状态的延迟时间
//		report.broadcast_my_seq = 0;//清零延迟时间
	}
	else{
        start_state_report.expires = sec_jiffies + 13 * (SEC_HZ/10);//设置单播状态上报的延迟时间
    }
    if (reportTypr == 2) {//上电上报延迟时间设定
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
//将报文插入操作本地发送缓存区
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
//获取接受报文
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
//处理检索接受报文并进行回传
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

