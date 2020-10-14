/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		sdk_evt.c
Description:	The event interface for high layer.
Author:			eastsoft
Version:		v1.0
Date:			2016/01/25
History:
*/
/************************************************************************************************/

#include "types.h"
#include "sdk_evt.h"
#include "gpio.h"
#include "cmd.h"
#include "repeater.h"
#include "comfunc.h"

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tx_state
	Description	:	电力线发送异常状态上报
	Input		:	state - 上报参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_tx_state(const sdk_evt_tx_state_t *state)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_plc
	Description	:	电力线接收数据上报
	Input		:	info - 接收参数
					data - 接收数据
					len  - 数据长度
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_rx_plc(const sdk_evt_rx_plc_t *info, const u8 *data, const u32 len)
{
    repeater_on_evt_rx_plc(info, data, len);
}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_devprop_req
	Description	:	申请设备搜索属性
	Input		:	req - 请求参数
					att - 请求中的属性数据
					len - 属性数据长度
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_devprop_req(const sdk_evt_devprop_req_t *req, const u8 *att, const u8 len)
{
    repeater_on_evt_devprop_req(req, att, len);
}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_sch_rpt
	Description	:	上报搜索设备
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_sch_rpt(const sdk_evt_sch_rpt_t *resp)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtpsk_req
	Description	:	远程设置口令请求上报
	Input		:	req - 请求参数
					key - 新口令
					len - 新口令长度
	Output		:	none
	Return		:	none
	Notes		:	只有解密状态为SDK_DEC_NEW时, key和len有效
*/
/*****************************************************************************************************/
void sdk_evt_rmtpsk_req(const sdk_evt_rmtpsk_req_t *req, const u8 *key, const u8 len)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtpsk_resp
	Description	:	远程设置口令回复上报
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_rmtpsk_resp(const sdk_evt_rmtpsk_resp_t *resp)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtsgid_req
	Description	:	远程修改组播地址请求上报
	Input		:	req - 请求参数
					cnt - 组播地址个数
					gid - 组播地址列表
	Output		:	none
	Return		:	none
	Notes		:	
*/
/*****************************************************************************************************/
void sdk_evt_rmtsgid_req(const sdk_evt_rmtsgid_req_t *req, const u8 cnt, const u8 *gid)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmtsgid_resp
	Description	:	远程修改组播地址回复上报
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_rmtsgid_resp(const sdk_evt_rmtsgid_resp_t *resp)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmthiepsk_req
	Description	:	远程设置主干口令请求上报
	Input		:	req - 请求参数
					key - 新口令
					len - 新口令长度
	Output		:	none
	Return		:	none
	Notes		:	只有解密状态为SDK_DEC_NEW时, key和len有效
*/
/*****************************************************************************************************/
void sdk_evt_rmthiepsk_req(const sdk_evt_rmtpsk_req_t *req, const u8 *key, const u8 len)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rmthiepsk_resp
	Description	:	远程设置口令回复上报
	Input		:	resp - 回复参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_rmthiepsk_resp(const sdk_evt_rmtpsk_resp_t *resp)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_dns
	Description	:	DNS解析结果上报
	Input		:	name - 主机名
					len  - 主机名长度
					ip   - 主机IP
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_dns(const u8 *name, const u8 len, const u32 ip)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_skt_state
	Description	:	连接状态上报
	Input		:	sta - 连接状态
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_skt_state(const sdk_link_state_t *sta)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_ge
	Description	:	网口接收数据上报
	Input		:	pair - 套接字对
					data - 接收数据
					len  - 接收数据长度
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_rx_ge(const sdk_skt_pair_t *pair, const u8 *data, const u32 len)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tx_ledon
	Description	:	PLC TX开灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	函数内部只做GPIO反转或者置位,不做延时等处理,否则会影响性能
*/
/*****************************************************************************************************/
void sdk_evt_tx_ledon(void)
{
	LED_ON(LED_PLC);
	//gpio_out_high(PLC_TX_LED);
}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tx_ledoff
	Description	:	PLC TX关灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	函数内部只做GPIO反转或者置位,不做延时等处理,否则会影响性能
*/
/*****************************************************************************************************/
void sdk_evt_tx_ledoff(void)
{
	LED_OFF(LED_PLC);
}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_ledon
	Description	:	PLC RX开灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	函数内部只做GPIO反转或者置位,不做延时等处理,否则会影响性能
*/
/*****************************************************************************************************/
void sdk_evt_rx_ledon(void)
{
	LED_ON(LED_PLC);
}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_rx_ledoff
	Description	:	PLC RX关灯
	Input		:	none
	Output		:	none
	Return		:	none
	Notes		:	函数内部只做GPIO反转或者置位,不做延时等处理,否则会影响性能
*/
/*****************************************************************************************************/
void sdk_evt_rx_ledoff(void)
{
	LED_OFF(LED_PLC);
}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_tst_resp
	Description	:	发送测试数据接收
	Input		:	resp - 接收参数
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_tst_resp(const sdk_evt_tst_resp_t *resp)
{

}

/*****************************************************************************************************/
/*
	Function	:	sdk_evt_getsec_verdate
	Description	:	获取二次开发上层的版本号和版本日期
	Input		:	version:版本号：2字节小端
					verdate:版本日期：年月日组成的3字节版本日期，小端
	Output		:	none
	Return		:	none
	Notes		:	none
*/
/*****************************************************************************************************/
void sdk_evt_getsec_ver(u8 *version, u8 *verdate)
{
    put_le_val(REPEATER_VERSION, version, 2);
    put_le_val(REPEATER_DATE, verdate, 3); 
}
